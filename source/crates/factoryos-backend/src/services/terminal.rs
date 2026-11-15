//! Workspace IDE terminals ([`factoryos_protocol::terminal`]): shells in a PTY on this backend's host
//! ([`factoryos_pty::Pty`]) and on other machines through terminal agents' links, one WebSocket per
//! session for the viewer ([`api::TERMINAL_WS`]).
//!
//! * Who: as the IDE's files ([`FilesService::admit`]: the workspace's owner, or a caller on loopback),
//!   and from no other site: browsers don't apply CORS to WebSockets, so an `Origin` must be this
//!   host's ([`dcv::same_origin`]). Merged outside the CORS layer.
//! * Agents (`factoryos-terminal-agent`) dial [`api::TERMINAL_LINK`] through the workspace edge, which
//!   checks their IAM identity and names it in `X-Terminal-Principal`; no other link is taken, except
//!   an agent on loopback with `FACTORY_OS_TERMINAL_LOCAL_AGENTS=1` (development). A link's sessions
//!   keep running while it is down; the agent says which on its next `hello`.
//! * A session outlives its sockets: its output so far (the last [`HISTORY_BYTES`]) is replayed to
//!   each new one, so the viewer can close and come back. Killed by `DELETE`, or with the backend.

use std::collections::{HashMap, VecDeque};
use std::net::SocketAddr;
use std::path::Path;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{ConnectInfo, Path as UrlPath, State};
use axum::http::{HeaderMap, StatusCode, header};
use axum::response::Response;
use axum::routing::{delete, get};
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::terminal::link::{self, FromAgent, ToAgent};
use factoryos_protocol::terminal::{BACKEND_HOST, NewTerminal, TerminalControl, TerminalHost, TerminalInfo, TerminalNotice};
use factoryos_pty::Pty;
use futures_util::{SinkExt as _, StreamExt as _};
use tokio::sync::{broadcast, mpsc, oneshot};

use crate::services::dcv;
use crate::services::files::FilesService;

type HttpError = (StatusCode, String);

/// Output kept per session for new sockets.
pub const HISTORY_BYTES: usize = 256 * 1024;
/// Sessions per backend, on all hosts (ended ones count until closed).
pub const MAX_SESSIONS: usize = 8;
const CWD_POLL: Duration = Duration::from_secs(1);
/// How long an agent has to open a shell, and to say hello on a new link.
const AGENT_ANSWER: Duration = Duration::from_secs(10);
const PRINCIPAL: &str = "x-terminal-principal";

#[derive(Clone, Debug)]
enum Event {
    Output(Arc<[u8]>),
    Notice(TerminalNotice),
}

enum Io {
    Local(Pty),
    /// Session `num` of agent `agent` (through its link, while it is up).
    Remote { agent: String, num: u32 },
}

struct Session {
    info: Mutex<TerminalInfo>,
    io: Io,
    /// The output so far; locked while an event is sent, so a new socket's replay and its live
    /// events neither overlap nor miss anything.
    history: Mutex<VecDeque<u8>>,
    events: broadcast::Sender<Event>,
}

impl Session {
    fn new(info: TerminalInfo, io: Io) -> Self {
        Self { info: Mutex::new(info), io, history: Mutex::default(), events: broadcast::channel(1024).0 }
    }

    fn publish(&self, event: Event) {
        let mut history = self.history.lock().unwrap();
        if let Event::Output(bytes) = &event {
            history.extend(bytes.iter());
            let over = history.len().saturating_sub(HISTORY_BYTES);
            history.drain(..over);
        }
        let _ = self.events.send(event);
    }

    /// The output so far and the live events after it.
    fn attach(&self) -> (Vec<u8>, broadcast::Receiver<Event>) {
        let history = self.history.lock().unwrap();
        (history.iter().copied().collect(), self.events.subscribe())
    }

    fn info(&self) -> TerminalInfo {
        self.info.lock().unwrap().clone()
    }

    fn ended(&self, code: Option<i32>) {
        {
            let mut info = self.info.lock().unwrap();
            if !info.alive {
                return;
            }
            info.alive = false;
            info.exit_code = code;
        }
        self.publish(Event::Notice(TerminalNotice::Exit(code)));
    }

    fn set_cwd(&self, cwd: String) {
        let changed = {
            let mut info = self.info.lock().unwrap();
            (info.cwd != cwd).then(|| info.cwd = cwd.clone()).is_some()
        };
        if changed {
            self.publish(Event::Notice(TerminalNotice::Cwd(cwd)));
        }
    }

    fn remote(&self, agent: &str) -> Option<u32> {
        match &self.io {
            Io::Remote { agent: a, num } if a == agent => Some(*num),
            _ => None,
        }
    }
}

/// An agent's live link: what goes to it, and the opens it hasn't answered yet.
struct Link {
    tx: mpsc::UnboundedSender<Message>,
    pending: Mutex<HashMap<u32, oneshot::Sender<Result<(String, String), String>>>>,
}

impl Link {
    fn send(&self, message: &ToAgent) {
        let _ = self.tx.send(Message::text(serde_json::to_string(message).unwrap_or_default()));
    }
}

struct Agent {
    host: TerminalHost,
    link: Option<Arc<Link>>,
}

pub struct TerminalService {
    files: Arc<FilesService>,
    sessions: Mutex<HashMap<String, Arc<Session>>>,
    agents: Mutex<HashMap<String, Agent>>,
    next: AtomicU64,
    next_num: AtomicU32,
    /// Distinguishes this backend run's session ids from an earlier run's.
    run: String,
    /// `FACTORY_OS_TERMINAL_SHELL`, else the user's login shell.
    shell: String,
    /// `FACTORY_OS_TERMINAL_LOCAL_AGENTS=1`: agents on loopback may link without the edge.
    local_agents: bool,
}

impl TerminalService {
    pub fn new(files: Arc<FilesService>) -> Arc<Self> {
        let run = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()) % 0x10000;
        let shell = std::env::var("FACTORY_OS_TERMINAL_SHELL").ok().filter(|s| !s.trim().is_empty()).unwrap_or_else(|| factoryos_pty::default_shell().0);
        let local_agents = std::env::var("FACTORY_OS_TERMINAL_LOCAL_AGENTS").is_ok_and(|v| v == "1");
        Arc::new(Self {
            files,
            sessions: Mutex::default(),
            agents: Mutex::default(),
            next: AtomicU64::new(1),
            next_num: AtomicU32::new(1),
            run: format!("{run:04x}"),
            shell,
            local_agents,
        })
    }

    pub fn hosts(&self) -> Vec<TerminalHost> {
        let mut agents: Vec<TerminalHost> = self.agents.lock().unwrap().values().map(|a| a.host.clone()).collect();
        agents.sort_by(|a, b| a.id.cmp(&b.id));
        let backend = TerminalHost { id: BACKEND_HOST.to_owned(), label: host_label(), os: std::env::consts::OS.to_owned(), online: true };
        std::iter::once(backend).chain(agents).collect()
    }

    pub fn list(&self) -> Vec<TerminalInfo> {
        let mut list: Vec<_> = self.sessions.lock().unwrap().values().map(|s| s.info()).collect();
        list.sort_by(|a, b| natural_id(&a.id).cmp(&natural_id(&b.id)));
        list
    }

    fn session(&self, id: &str) -> Result<Arc<Session>, HttpError> {
        self.sessions.lock().unwrap().get(id).cloned().ok_or_else(|| (StatusCode::NOT_FOUND, format!("no terminal {id}")))
    }

    fn link(&self, agent: &str) -> Option<Arc<Link>> {
        self.agents.lock().unwrap().get(agent).and_then(|a| a.link.clone())
    }

    fn new_id(&self) -> String {
        format!("{}-{}", self.run, self.next.fetch_add(1, Ordering::Relaxed))
    }

    /// A new shell: on this host in `request.root`'s folder (else the first root's, else home), or
    /// on agent `request.host`.
    pub async fn open(self: &Arc<Self>, request: NewTerminal) -> Result<TerminalInfo, HttpError> {
        if self.sessions.lock().unwrap().len() >= MAX_SESSIONS {
            return Err((StatusCode::TOO_MANY_REQUESTS, format!("at most {MAX_SESSIONS} terminals: close one first")));
        }
        if request.host == BACKEND_HOST {
            return self.open_local(&request);
        }
        let link = match self.agents.lock().unwrap().get(&request.host) {
            None => return Err((StatusCode::NOT_FOUND, format!("no terminal host {}", request.host))),
            Some(agent) => agent.link.clone(),
        };
        let link = link.ok_or_else(|| (StatusCode::SERVICE_UNAVAILABLE, format!("{} is offline (no link)", request.host)))?;
        let num = self.next_num.fetch_add(1, Ordering::Relaxed);
        let (answer, answered) = oneshot::channel();
        link.pending.lock().unwrap().insert(num, answer);
        link.send(&ToAgent::Open { num, cols: request.cols, rows: request.rows });
        let unavailable = |why: String| (StatusCode::SERVICE_UNAVAILABLE, format!("{}: {why}", request.host));
        let (shell, cwd) = match tokio::time::timeout(AGENT_ANSWER, answered).await {
            Ok(Ok(Ok(opened))) => opened,
            Ok(Ok(Err(err))) => return Err(unavailable(err)),
            Ok(Err(_)) => return Err(unavailable("the link dropped".to_owned())),
            Err(_) => {
                link.pending.lock().unwrap().remove(&num);
                link.send(&ToAgent::Close { num });
                return Err(unavailable("no answer".to_owned()));
            }
        };
        let id = self.new_id();
        let info = TerminalInfo { id: id.clone(), host: request.host.clone(), shell, cwd, alive: true, exit_code: None };
        self.sessions.lock().unwrap().insert(id, Arc::new(Session::new(info.clone(), Io::Remote { agent: request.host, num })));
        Ok(info)
    }

    fn open_local(self: &Arc<Self>, request: &NewTerminal) -> Result<TerminalInfo, HttpError> {
        let cwd = match &request.root {
            Some(root) => self.files.root_dir(root)?,
            None => match self.files.roots().first() {
                Some(root) => self.files.root_dir(&root.id)?,
                None => factoryos_pty::home_dir(),
            },
        };
        let id = self.new_id();
        let session = spawn_local(&id, &self.shell, &cwd, (request.cols, request.rows)).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}")))?;
        let info = session.info();
        self.sessions.lock().unwrap().insert(id, session.clone());
        tokio::spawn(follow_cwd(Arc::downgrade(&session)));
        Ok(info)
    }

    pub fn close(&self, id: &str) -> Result<(), HttpError> {
        let session = self.sessions.lock().unwrap().remove(id).ok_or_else(|| (StatusCode::NOT_FOUND, format!("no terminal {id}")))?;
        match &session.io {
            Io::Local(pty) => pty.kill(),
            Io::Remote { agent, num } => {
                if let Some(link) = self.link(agent) {
                    link.send(&ToAgent::Close { num: *num });
                }
            }
        }
        Ok(())
    }

    fn write(&self, session: &Session, bytes: Vec<u8>) {
        match &session.io {
            Io::Local(pty) => pty.write(bytes),
            Io::Remote { agent, num } => {
                if let Some(link) = self.link(agent) {
                    let _ = link.tx.send(Message::binary(link::frame(*num, &bytes)));
                }
            }
        }
    }

    fn resize(&self, session: &Session, cols: u16, rows: u16) {
        match &session.io {
            Io::Local(pty) => pty.resize(cols, rows),
            Io::Remote { agent, num } => {
                if let Some(link) = self.link(agent) {
                    link.send(&ToAgent::Resize { num: *num, cols, rows });
                }
            }
        }
    }

    fn remote_session(&self, agent: &str, num: u32) -> Option<Arc<Session>> {
        self.sessions.lock().unwrap().values().find(|s| s.remote(agent) == Some(num)).cloned()
    }

    /// Agent `name` linked (again): its host is online; sessions it no longer runs ended, and the
    /// ones it runs that this backend doesn't know (from before a restart) closed.
    fn attach_agent(&self, name: &str, label: String, os: String, running: &[u32], link: Arc<Link>) {
        let host = TerminalHost { id: name.to_owned(), label, os, online: true };
        // A new link replaces an older one (its writer ends with its sender).
        self.agents.lock().unwrap().insert(name.to_owned(), Agent { host, link: Some(link.clone()) });
        let sessions: Vec<_> = self.sessions.lock().unwrap().values().cloned().collect();
        let known: Vec<u32> = sessions.iter().filter_map(|s| s.remote(name)).collect();
        for session in &sessions {
            if session.remote(name).is_some_and(|num| !running.contains(&num)) {
                session.ended(None);
            }
        }
        for num in running.iter().filter(|num| !known.contains(num)) {
            link.send(&ToAgent::Close { num: *num });
        }
        // Numbers are per backend run: start past the agent's, so a new one never reuses one.
        if let Some(max) = running.iter().max() {
            self.next_num.fetch_max(max + 1, Ordering::Relaxed);
        }
    }

    /// The link `link` of agent `name` is down (unless a newer one replaced it).
    fn detach_agent(&self, name: &str, link: &Arc<Link>) {
        let mut agents = self.agents.lock().unwrap();
        if let Some(agent) = agents.get_mut(name)
            && agent.link.as_ref().is_some_and(|l| Arc::ptr_eq(l, link))
        {
            agent.link = None;
            agent.host.online = false;
        }
    }

    fn on_agent(&self, name: &str, link: &Link, message: FromAgent) {
        match message {
            FromAgent::Hello { .. } => {}
            FromAgent::Opened { num, shell, cwd } => {
                if let Some(answer) = link.pending.lock().unwrap().remove(&num) {
                    let _ = answer.send(Ok((shell, cwd)));
                }
            }
            FromAgent::Failed { num, error } => {
                if let Some(answer) = link.pending.lock().unwrap().remove(&num) {
                    let _ = answer.send(Err(error));
                }
            }
            FromAgent::Cwd { num, cwd } => {
                if let Some(session) = self.remote_session(name, num) {
                    session.set_cwd(cwd);
                }
            }
            FromAgent::Exit { num, code } => {
                if let Some(session) = self.remote_session(name, num) {
                    session.ended(code);
                }
            }
        }
    }

    fn on_agent_output(&self, name: &str, frame: &[u8]) {
        if let Some((num, bytes)) = link::unframe(frame)
            && let Some(session) = self.remote_session(name, num)
        {
            session.publish(Event::Output(bytes.into()));
        }
    }

    /// Whether a link for `name` may be taken: the edge's verified principal for that name, or an
    /// agent on loopback when local agents are allowed. Never from a browser (an `Origin`).
    fn admit_link(&self, peer: SocketAddr, headers: &HeaderMap, name: &str) -> Result<(), HttpError> {
        let forbidden = |why: &str| Err((StatusCode::FORBIDDEN, format!("terminal link: {why}")));
        if !link::valid_name(name) {
            return forbidden("not an agent name");
        }
        if headers.contains_key(header::ORIGIN) {
            return forbidden("not from a browser");
        }
        match headers.get(PRINCIPAL).and_then(|v| v.to_str().ok()) {
            Some(principal) if principal == name => Ok(()),
            Some(_) => forbidden("the edge verified another agent"),
            None if self.local_agents && peer.ip().is_loopback() => Ok(()),
            None => forbidden("only through the edge (an IAM identity)"),
        }
    }
}

impl Drop for TerminalService {
    fn drop(&mut self) {
        for session in self.sessions.lock().unwrap().values() {
            if let Io::Local(pty) = &session.io {
                pty.kill();
            }
        }
    }
}

fn host_label() -> String {
    #[cfg(unix)]
    {
        let mut buf = [0u8; 256];
        // SAFETY: `buf` is valid for its length; gethostname NUL-terminates within it on success.
        if unsafe { libc::gethostname(buf.as_mut_ptr().cast(), buf.len()) } == 0 {
            let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
            if let Ok(name) = std::str::from_utf8(&buf[..end])
                && !name.is_empty()
            {
                return format!("Backend ({})", name.split('.').next().unwrap_or(name));
            }
        }
    }
    "Backend".to_owned()
}

fn natural_id(id: &str) -> (String, u64) {
    let (run, n) = id.rsplit_once('-').unwrap_or((id, "0"));
    (run.to_owned(), n.parse().unwrap_or(0))
}

fn spawn_local(id: &str, shell: &str, cwd: &Path, size: (u16, u16)) -> anyhow::Result<Arc<Session>> {
    let (shell, args) = factoryos_pty::shell_command(shell);
    // The PTY's callbacks need the session, which holds the PTY: they get it once it exists.
    let slot: Arc<Mutex<Option<std::sync::Weak<Session>>>> = Arc::default();
    let (out, ended) = (slot.clone(), slot.clone());
    let early: Arc<Mutex<Vec<u8>>> = Arc::default();
    let early_out = early.clone();
    let pty = Pty::spawn(
        &shell,
        &args,
        cwd,
        size,
        move |bytes| match out.lock().unwrap().as_ref().and_then(std::sync::Weak::upgrade) {
            Some(session) => session.publish(Event::Output(bytes.into())),
            None => early_out.lock().unwrap().extend_from_slice(bytes),
        },
        move |code| {
            if let Some(session) = ended.lock().unwrap().as_ref().and_then(std::sync::Weak::upgrade) {
                session.ended(code);
            }
        },
    )?;
    let info = TerminalInfo { id: id.to_owned(), host: BACKEND_HOST.to_owned(), shell: pty.name.clone(), cwd: cwd.display().to_string(), alive: true, exit_code: None };
    let session = Arc::new(Session::new(info, Io::Local(pty)));
    {
        let mut slot = slot.lock().unwrap();
        *slot = Some(Arc::downgrade(&session));
        let early = std::mem::take(&mut *early.lock().unwrap());
        if !early.is_empty() {
            session.publish(Event::Output(early.into()));
        }
    }
    Ok(session)
}

/// Send a local session's current folder when it changes, until it ends.
async fn follow_cwd(session: std::sync::Weak<Session>) {
    loop {
        tokio::time::sleep(CWD_POLL).await;
        let Some(session) = session.upgrade() else { return };
        if !session.info.lock().unwrap().alive {
            return;
        }
        let Io::Local(pty) = &session.io else { return };
        if let Some(cwd) = pty.cwd() {
            session.set_cwd(cwd);
        }
    }
}

// --- HTTP ---

/// Its own router (with its own state), merged outside the CORS layer.
pub fn router(terminals: Arc<TerminalService>) -> Router {
    Router::new()
        .route(api::TERMINAL_HOSTS, get(hosts))
        .route(api::TERMINALS, get(list).post(open))
        .route(api::TERMINAL, delete(close))
        .route(api::TERMINAL_WS, get(socket))
        .route(api::TERMINAL_LINK, get(agent_link))
        .with_state(terminals)
}

fn admit(terminals: &TerminalService, peer: SocketAddr, headers: &HeaderMap) -> Result<(), HttpError> {
    terminals.files.admit(peer, headers)?;
    if !dcv::same_origin(headers) {
        return Err((StatusCode::FORBIDDEN, "terminals: cross-origin request refused".to_owned()));
    }
    Ok(())
}

async fn hosts(State(t): State<Arc<TerminalService>>, ConnectInfo(peer): ConnectInfo<SocketAddr>, headers: HeaderMap) -> Result<Json<Vec<TerminalHost>>, HttpError> {
    admit(&t, peer, &headers)?;
    Ok(Json(t.hosts()))
}

async fn list(State(t): State<Arc<TerminalService>>, ConnectInfo(peer): ConnectInfo<SocketAddr>, headers: HeaderMap) -> Result<Json<Vec<TerminalInfo>>, HttpError> {
    admit(&t, peer, &headers)?;
    Ok(Json(t.list()))
}

async fn open(
    State(t): State<Arc<TerminalService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<NewTerminal>,
) -> Result<Json<TerminalInfo>, HttpError> {
    admit(&t, peer, &headers)?;
    t.open(request).await.map(Json)
}

/// `true` (JSON, as the viewer's client reads every answer).
async fn close(State(t): State<Arc<TerminalService>>, ConnectInfo(peer): ConnectInfo<SocketAddr>, headers: HeaderMap, UrlPath(id): UrlPath<String>) -> Result<Json<bool>, HttpError> {
    admit(&t, peer, &headers)?;
    t.close(&id).map(|()| Json(true))
}

async fn socket(
    State(t): State<Arc<TerminalService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath(id): UrlPath<String>,
    upgrade: WebSocketUpgrade,
) -> Result<Response, HttpError> {
    admit(&t, peer, &headers)?;
    let session = t.session(&id)?;
    Ok(upgrade.on_upgrade(move |socket| serve(socket, t, session)))
}

fn notice(notice: &TerminalNotice) -> Message {
    Message::text(serde_json::to_string(notice).unwrap_or_default())
}

async fn serve(socket: WebSocket, t: Arc<TerminalService>, session: Arc<Session>) {
    let (mut tx, mut rx) = socket.split();
    let (history, mut events) = session.attach();
    let info = session.info();
    let mut opening = vec![Message::binary(history), notice(&TerminalNotice::Cwd(info.cwd)), notice(&TerminalNotice::Replayed)];
    if !info.alive {
        opening.push(notice(&TerminalNotice::Exit(info.exit_code)));
    }
    for message in opening {
        if tx.send(message).await.is_err() {
            return;
        }
    }
    // Pings keep a quiet shell's socket open through the edge (the browser answers them by itself).
    let mut keepalive = tokio::time::interval_at(tokio::time::Instant::now() + crate::WS_KEEPALIVE, crate::WS_KEEPALIVE);
    loop {
        tokio::select! {
            _ = keepalive.tick() => {
                if tx.send(Message::Ping(Vec::new().into())).await.is_err() {
                    break;
                }
            }
            event = events.recv() => {
                let message = match event {
                    Ok(Event::Output(bytes)) => Message::binary(bytes.to_vec()),
                    Ok(Event::Notice(n)) => notice(&n),
                    // Too far behind: this socket ends; a new one replays the history.
                    Err(broadcast::error::RecvError::Lagged(_)) | Err(broadcast::error::RecvError::Closed) => break,
                };
                if tx.send(message).await.is_err() {
                    break;
                }
            }
            message = rx.next() => match message {
                Some(Ok(Message::Binary(bytes))) => t.write(&session, bytes.to_vec()),
                Some(Ok(Message::Text(text))) => {
                    if let Ok(TerminalControl::Resize { cols, rows }) = serde_json::from_str(&text) {
                        t.resize(&session, cols, rows);
                    }
                }
                Some(Ok(Message::Close(_)) | Err(_)) | None => break,
                Some(Ok(_)) => {}
            },
        }
    }
    let _ = tx.close().await;
}

async fn agent_link(
    State(t): State<Arc<TerminalService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath(name): UrlPath<String>,
    upgrade: WebSocketUpgrade,
) -> Result<Response, HttpError> {
    t.admit_link(peer, &headers, &name)?;
    Ok(upgrade.on_upgrade(move |socket| serve_link(socket, t, name)))
}

async fn serve_link(socket: WebSocket, t: Arc<TerminalService>, name: String) {
    let (mut ws_tx, mut ws_rx) = socket.split();
    // First its hello, for the name the edge verified.
    let hello = tokio::time::timeout(AGENT_ANSWER, async {
        while let Some(Ok(message)) = ws_rx.next().await {
            if let Message::Text(text) = message {
                return serde_json::from_str::<FromAgent>(&text).ok();
            }
        }
        None
    })
    .await;
    let Ok(Some(FromAgent::Hello { name: said, label, os, sessions })) = hello else {
        tracing::warn!("terminal link {name}: no hello");
        return;
    };
    if said != name {
        tracing::warn!("terminal link {name}: the agent says it is {said}");
        return;
    }
    let (tx, mut outgoing) = mpsc::unbounded_channel();
    let link = Arc::new(Link { tx, pending: Mutex::default() });
    t.attach_agent(&name, label, os, &sessions, link.clone());
    tracing::info!("terminal agent {name} linked ({} sessions running)", sessions.len());
    let writer = tokio::spawn(async move {
        // Pings keep an idle link open through the edge (the agent pings too).
        let mut keepalive = tokio::time::interval_at(tokio::time::Instant::now() + crate::WS_KEEPALIVE, crate::WS_KEEPALIVE);
        loop {
            let message = tokio::select! {
                message = outgoing.recv() => match message {
                    Some(message) => message,
                    None => break,
                },
                _ = keepalive.tick() => Message::Ping(Vec::new().into()),
            };
            if ws_tx.send(message).await.is_err() {
                break;
            }
        }
        let _ = ws_tx.close().await;
    });
    while let Some(Ok(message)) = ws_rx.next().await {
        match message {
            Message::Text(text) => match serde_json::from_str::<FromAgent>(&text) {
                Ok(message) => t.on_agent(&name, &link, message),
                Err(err) => tracing::warn!("terminal link {name}: {err}"),
            },
            Message::Binary(frame) => t.on_agent_output(&name, &frame),
            Message::Close(_) => break,
            _ => {}
        }
    }
    t.detach_agent(&name, &link);
    drop(link);
    writer.abort();
    tracing::info!("terminal agent {name}: link down");
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use crate::config_store::ConfigStore;
    use std::path::PathBuf;

    fn tmp() -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fos-term-{}-{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::canonicalize(dir).unwrap()
    }

    fn service(dir: &Path) -> Arc<TerminalService> {
        let config = Arc::new(ConfigStore::open(tmp().join("factoryos.config.json")));
        let t = TerminalService::new(FilesService::new(config, &[format!("Work={}", dir.display())], Some("jdoe".into())).unwrap());
        // Not the user's login shell and its rc files: the same everywhere.
        let mut t = Arc::try_unwrap(t).ok().unwrap();
        t.shell = "/bin/sh".to_owned();
        Arc::new(t)
    }

    async fn output_until(events: &mut broadcast::Receiver<Event>, seen: &mut String, needle: &str) {
        tokio::time::timeout(Duration::from_secs(10), async {
            while !seen.contains(needle) {
                match events.recv().await {
                    Ok(Event::Output(bytes)) => seen.push_str(&String::from_utf8_lossy(&bytes)),
                    Ok(_) => {}
                    Err(err) => panic!("{err}"),
                }
            }
        })
        .await
        .unwrap_or_else(|_| panic!("no {needle:?} in {seen:?}"));
    }

    /// Bash drops what is typed before its first prompt: wait for it (`$ `, or `# ` as root, as in CI).
    async fn ready(session: &Session) -> broadcast::Receiver<Event> {
        let (history, mut events) = session.attach();
        let mut seen = String::from_utf8_lossy(&history).into_owned();
        tokio::time::timeout(Duration::from_secs(10), async {
            while !(seen.contains("$ ") || seen.contains("# ")) {
                match events.recv().await {
                    Ok(Event::Output(bytes)) => seen.push_str(&String::from_utf8_lossy(&bytes)),
                    Ok(_) => {}
                    Err(err) => panic!("{err}"),
                }
            }
        })
        .await
        .unwrap_or_else(|_| panic!("no prompt in {seen:?}"));
        events
    }

    async fn open(t: &Arc<TerminalService>) -> TerminalInfo {
        t.open(NewTerminal { host: BACKEND_HOST.into(), root: Some("work".into()), cols: 80, rows: 24 }).await.unwrap()
    }

    fn pty(session: &Session) -> &Pty {
        match &session.io {
            Io::Local(pty) => pty,
            Io::Remote { .. } => panic!("remote"),
        }
    }

    #[test]
    fn cross_site_pages_are_refused() {
        let t = service(&tmp());
        let local: SocketAddr = "127.0.0.1:5000".parse().unwrap();
        let mut headers = HeaderMap::new();
        headers.insert("host", "127.0.0.1:8080".parse().unwrap());
        assert!(admit(&t, local, &headers).is_ok(), "a native viewer sends no Origin");
        headers.insert("origin", "http://127.0.0.1:8080".parse().unwrap());
        assert!(admit(&t, local, &headers).is_ok(), "the backend's own page");
        headers.insert("origin", "https://evil.example".parse().unwrap());
        assert_eq!(admit(&t, local, &headers).unwrap_err().0, StatusCode::FORBIDDEN);
        let mut edge = HeaderMap::new();
        edge.insert("host", "ws.example".parse().unwrap());
        edge.insert("x-user-alias", "alice".parse().unwrap());
        assert_eq!(admit(&t, "10.0.0.5:4000".parse().unwrap(), &edge).unwrap_err().0, StatusCode::FORBIDDEN, "not the owner");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_shell_runs_in_the_root_and_its_output_is_replayed() {
        let dir = tmp();
        let t = service(&dir);
        let info = open(&t).await;
        assert_eq!(info.cwd, dir.display().to_string());
        let session = t.session(&info.id).unwrap();
        let mut events = ready(&session).await;
        t.write(&session, b"printf 'fos-%s\\n' $(pwd | wc -c | tr -d ' ') ; stty size\r".to_vec());
        let mut seen = String::new();
        output_until(&mut events, &mut seen, "24 80").await;
        assert!(seen.contains("fos-"));
        t.resize(&session, 100, 30);
        t.write(&session, b"stty size\r".to_vec());
        output_until(&mut events, &mut seen, "30 100").await;
        let (history, _) = session.attach();
        assert!(String::from_utf8_lossy(&history).contains("30 100"), "a new socket sees the output so far");
        t.close(&info.id).unwrap();
        assert!(t.list().is_empty());
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_current_folder_follows_cd() {
        let dir = tmp();
        std::fs::create_dir(dir.join("biw")).unwrap();
        let t = service(&dir);
        let info = open(&t).await;
        let session = t.session(&info.id).unwrap();
        ready(&session).await;
        t.write(&session, b"cd biw\r".to_vec());
        let want = dir.join("biw").display().to_string();
        tokio::time::timeout(Duration::from_secs(10), async {
            while session.info().cwd != want {
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
        })
        .await
        .unwrap_or_else(|_| panic!("cwd stayed {} ({:?})", session.info().cwd, pty(&session).cwd()));
        t.close(&info.id).unwrap();
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn exit_is_reported_and_the_limit_holds() {
        let dir = tmp();
        let t = service(&dir);
        let info = open(&t).await;
        let session = t.session(&info.id).unwrap();
        let mut events = ready(&session).await;
        t.write(&session, b"exit\r".to_vec());
        tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                // The code is the shell's; a login profile's hooks may change it, so only its presence.
                if let Ok(Event::Notice(TerminalNotice::Exit(code))) = events.recv().await {
                    assert!(code.is_some());
                    break;
                }
            }
        })
        .await
        .unwrap();
        assert!(!t.list()[0].alive);
        for _ in 1..MAX_SESSIONS {
            open(&t).await;
        }
        let over = t.open(NewTerminal { host: BACKEND_HOST.into(), root: None, cols: 80, rows: 24 }).await;
        assert_eq!(over.unwrap_err().0, StatusCode::TOO_MANY_REQUESTS);
    }

    #[test]
    fn links_only_from_the_edge_verified_agent() {
        let t = service(&tmp());
        let edge: SocketAddr = "10.0.0.5:4000".parse().unwrap();
        let local: SocketAddr = "127.0.0.1:4000".parse().unwrap();
        let with = |pairs: &[(&str, &str)]| {
            let mut h = HeaderMap::new();
            for (k, v) in pairs {
                h.insert(axum::http::HeaderName::from_bytes(k.as_bytes()).unwrap(), v.parse().unwrap());
            }
            h
        };
        assert!(t.admit_link(edge, &with(&[(PRINCIPAL, "plc-host")]), "plc-host").is_ok());
        assert_eq!(t.admit_link(edge, &with(&[(PRINCIPAL, "isaac-1")]), "plc-host").unwrap_err().0, StatusCode::FORBIDDEN);
        assert_eq!(t.admit_link(edge, &with(&[]), "plc-host").unwrap_err().0, StatusCode::FORBIDDEN);
        assert_eq!(t.admit_link(local, &with(&[]), "plc-host").unwrap_err().0, StatusCode::FORBIDDEN, "local agents are off by default");
        assert_eq!(t.admit_link(edge, &with(&[(PRINCIPAL, "plc-host"), ("origin", "https://ws.example")]), "plc-host").unwrap_err().0, StatusCode::FORBIDDEN, "no browser");
        assert_eq!(t.admit_link(edge, &with(&[(PRINCIPAL, "Bad/Name")]), "Bad/Name").unwrap_err().0, StatusCode::FORBIDDEN);
        let mut local_ok = Arc::try_unwrap(service(&tmp())).ok().unwrap();
        local_ok.local_agents = true;
        assert!(local_ok.admit_link(local, &with(&[]), "dev").is_ok());
        assert_eq!(local_ok.admit_link(edge, &with(&[]), "dev").unwrap_err().0, StatusCode::FORBIDDEN, "loopback only");
    }

    fn text(message: Message) -> ToAgent {
        match message {
            Message::Text(text) => serde_json::from_str(&text).unwrap(),
            other => panic!("{other:?}"),
        }
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn sessions_on_an_agent_through_its_link() {
        let t = service(&tmp());
        let (tx, mut to_agent) = mpsc::unbounded_channel();
        let link = Arc::new(Link { tx, pending: Mutex::default() });
        t.attach_agent("plc-host", "PLC host".into(), "linux".into(), &[], link.clone());
        assert_eq!(t.hosts().iter().map(|h| (h.id.as_str(), h.online)).collect::<Vec<_>>(), [(BACKEND_HOST, true), ("plc-host", true)]);

        let opening = tokio::spawn({
            let t = t.clone();
            async move { t.open(NewTerminal { host: "plc-host".into(), root: None, cols: 90, rows: 20 }).await }
        });
        let ToAgent::Open { num, cols: 90, rows: 20 } = text(to_agent.recv().await.unwrap()) else { panic!("not an open") };
        t.on_agent("plc-host", &link, FromAgent::Opened { num, shell: "bash".into(), cwd: "/home/cell".into() });
        let info = opening.await.unwrap().unwrap();
        assert_eq!((info.host.as_str(), info.shell.as_str(), info.cwd.as_str()), ("plc-host", "bash", "/home/cell"));

        let session = t.session(&info.id).unwrap();
        t.write(&session, b"ls\r".to_vec());
        assert_eq!(to_agent.recv().await.unwrap(), Message::binary(link::frame(num, b"ls\r")));
        t.resize(&session, 120, 40);
        assert_eq!(text(to_agent.recv().await.unwrap()), ToAgent::Resize { num, cols: 120, rows: 40 });
        t.on_agent_output("plc-host", &link::frame(num, b"plan2cell.toml\r\n"));
        t.on_agent("plc-host", &link, FromAgent::Cwd { num, cwd: "/srv".into() });
        assert!(String::from_utf8_lossy(&session.attach().0).contains("plan2cell.toml"));
        assert_eq!(session.info().cwd, "/srv");

        // The link drops: the host is offline, the session waits; a new link without it ends it.
        t.detach_agent("plc-host", &link);
        assert!(!t.hosts()[1].online);
        assert!(session.info().alive);
        let offline = t.open(NewTerminal { host: "plc-host".into(), root: None, cols: 80, rows: 24 }).await;
        assert_eq!(offline.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        let (tx, mut to_agent) = mpsc::unbounded_channel();
        t.attach_agent("plc-host", "PLC host".into(), "linux".into(), &[99], Arc::new(Link { tx, pending: Mutex::default() }));
        assert!(!session.info().alive, "the agent no longer runs it");
        assert_eq!(text(to_agent.recv().await.unwrap()), ToAgent::Close { num: 99 }, "one from before a backend restart");

        // An agent that refuses.
        let (tx, mut to_agent) = mpsc::unbounded_channel();
        let link = Arc::new(Link { tx, pending: Mutex::default() });
        t.attach_agent("win-ide", "Windows IDE".into(), "windows".into(), &[], link.clone());
        let opening = tokio::spawn({
            let t = t.clone();
            async move { t.open(NewTerminal { host: "win-ide".into(), root: None, cols: 80, rows: 24 }).await }
        });
        let ToAgent::Open { num, .. } = text(to_agent.recv().await.unwrap()) else { panic!() };
        t.on_agent("win-ide", &link, FromAgent::Failed { num, error: "no ConPTY".into() });
        assert_eq!(opening.await.unwrap().unwrap_err().1, "win-ide: no ConPTY");
        assert_eq!(t.open(NewTerminal { host: "nope".into(), root: None, cols: 80, rows: 24 }).await.unwrap_err().0, StatusCode::NOT_FOUND);
    }
}
