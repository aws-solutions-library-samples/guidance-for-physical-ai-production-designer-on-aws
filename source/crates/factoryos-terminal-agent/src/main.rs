//! factoryos-terminal-agent: this machine's shells in the Workspace IDE of factoryos workspaces.
//!
//! For each workspace it keeps one WebSocket, dialled out (no inbound port):
//!
//!   wss://<edge>/workspace/<workspace>/terminal-link/<name>
//!
//! carrying in `X-Factoryos-Iam` a presigned `sts:GetCallerIdentity` URL (60 s) with
//! `x-factoryos-terminal: <workspace>/<name>` signed in, so it proves this machine's identity for this
//! one link only, signed with:
//!
//! * a cell's host (PLC host, KUKA VC host, Windows IDE): its own instance profile's credentials, whose
//!   session EC2 names after the instance; the edge checks that instance's tags (that workspace's cell,
//!   the role this name stands for: `plc`, `kvc-<n>`, `ide`);
//! * an Isaac Sim instance: the factoryos account's role `factoryos-isaac-<name>` (FACTORYOS_ROLE_ARN,
//!   assumed with the instance's own), checked against the Isaac registry.
//!
//! Then the request goes on, as `X-Terminal-Principal: <name>`, to the workspace's backend
//! (`services/terminal.rs`), which opens and drives shells here over the link
//! ([`factoryos_protocol::terminal::link`]).
//!
//! Shells keep running while a link is down (their output kept, the last 256 KiB), and are killed
//! after [`ORPHAN_GRACE`] without one. Settings from the environment, or `--env-file <file>`
//! (`KEY=value` lines), as the installers write it (plan2cell/infra/terminal-agent):
//!
//!   FACTORYOS_EDGE (https://…), FACTORYOS_WORKSPACES (a,b; or FACTORYOS_WORKSPACE), TERMINAL_NAME
//!   (or ISAAC_NAME), TERMINAL_LABEL, FACTORYOS_ROLE_ARN (Isaac only), STS_REGION (us-east-1),
//!   TERMINAL_SHELL, TERMINAL_CWD
//!   or, for development without the edge: TERMINAL_LINK_URL=ws://127.0.0.1:8080/terminal-link/<name>
//!   (the backend with FACTORY_OS_TERMINAL_LOCAL_AGENTS=1).

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime};

use anyhow::{Context as _, bail};
use factoryos_protocol::terminal::link::{self, FromAgent, ToAgent};
use factoryos_pty::{Osc7, Pty};
use futures_util::{SinkExt as _, StreamExt as _};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::client::IntoClientRequest as _;

const BINDING_HEADER: &str = "x-factoryos-terminal";
const IAM_HEADER: &str = "X-Factoryos-Iam";
/// Output kept per session while its link is down.
const PENDING_BYTES: usize = 256 * 1024;
/// Shells without a link this long are killed.
const ORPHAN_GRACE: Duration = Duration::from_secs(60);
const CWD_POLL: Duration = Duration::from_secs(1);
/// Pings on the link: an idle WebSocket is closed on the way (CloudFront) after a few minutes.
const KEEPALIVE: Duration = Duration::from_secs(30);

#[derive(Clone, Debug, PartialEq, Eq)]
struct Settings {
    name: String,
    label: String,
    /// (link URL, the workspace/name binding to sign; `None`: no IAM, a development link).
    links: Vec<(String, Option<String>)>,
    role_arn: Option<String>,
    sts_region: String,
    shell: Option<String>,
    cwd: PathBuf,
    /// Through the edge (an IAM identity on each link), not a development link.
    edge: bool,
}

impl Settings {
    fn from_env(env: impl Fn(&str) -> Option<String>) -> anyhow::Result<Self> {
        let get = |k: &str| env(k).map(|v| v.trim().to_owned()).filter(|v| !v.is_empty());
        // An Isaac Sim instance's link.env names these as its tunnel does.
        let name = get("TERMINAL_NAME").or_else(|| get("ISAAC_NAME")).context("TERMINAL_NAME is not set")?;
        if !link::valid_name(&name) {
            bail!("TERMINAL_NAME {name:?} is not a [a-z][a-z0-9-] name (at most 32)");
        }
        let label = get("TERMINAL_LABEL").unwrap_or_else(|| name.clone());
        let shell = get("TERMINAL_SHELL");
        let cwd = get("TERMINAL_CWD").map_or_else(factoryos_pty::home_dir, PathBuf::from);
        let sts_region = get("STS_REGION").unwrap_or_else(|| "us-east-1".to_owned());
        if let Some(url) = get("TERMINAL_LINK_URL") {
            if !url.starts_with("ws://127.0.0.1:") && !url.starts_with("ws://localhost:") && !url.starts_with("ws://[::1]:") {
                bail!("TERMINAL_LINK_URL is for a backend on this machine (ws://127.0.0.1:…); through the edge use FACTORYOS_EDGE");
            }
            return Ok(Self { name, label, links: vec![(url, None)], role_arn: None, sts_region, shell, cwd, edge: false });
        }
        let edge = get("FACTORYOS_EDGE").context("FACTORYOS_EDGE is not set")?;
        let Some(host) = edge.trim_end_matches('/').strip_prefix("https://") else { bail!("FACTORYOS_EDGE must be https://<the factoryos edge>") };
        // None: this machine's own credentials (a cell's host: its instance profile).
        let role_arn = get("FACTORYOS_ROLE_ARN");
        if let Some(arn) = &role_arn
            && (!arn.starts_with("arn:aws") || !arn.contains(":role/"))
        {
            bail!("FACTORYOS_ROLE_ARN must be the factoryos account's role arn:aws:iam::<account>:role/factoryos-isaac-{name}");
        }
        let workspaces: Vec<String> = get("FACTORYOS_WORKSPACES").or_else(|| get("FACTORYOS_WORKSPACE")).context("FACTORYOS_WORKSPACES is not set")?.split(',').map(|w| w.trim().to_owned()).filter(|w| !w.is_empty()).collect();
        let mut links = Vec::new();
        for workspace in workspaces {
            if !factoryos_protocol::workspace::is_valid_name(&workspace) {
                bail!("workspace {workspace:?} is not a workspace name");
            }
            links.push((format!("wss://{host}/workspace/{workspace}/terminal-link/{name}"), Some(format!("{workspace}/{name}"))));
        }
        if links.is_empty() {
            bail!("FACTORYOS_WORKSPACES names no workspace");
        }
        Ok(Self { name, label, links, role_arn, sts_region, shell, cwd, edge: true })
    }
}

/// A presigned `sts:GetCallerIdentity` URL with `binding` signed in as `x-factoryos-terminal`.
fn presign(credentials: &aws_credential_types::Credentials, region: &str, binding: &str, now: SystemTime) -> anyhow::Result<String> {
    use aws_sigv4::http_request::{SignableBody, SignableRequest, SignatureLocation, SigningSettings, sign};
    let host = format!("sts.{region}.amazonaws.com");
    let url = format!("https://{host}/?Action=GetCallerIdentity&Version=2011-06-15");
    let mut settings = SigningSettings::default();
    settings.signature_location = SignatureLocation::QueryParams;
    settings.expires_in = Some(Duration::from_secs(60));
    let identity = credentials.clone().into();
    let params = aws_sigv4::sign::v4::SigningParams::builder().identity(&identity).region(region).name("sts").time(now).settings(settings).build()?.into();
    let request = SignableRequest::new("GET", &url, [("host", host.as_str()), (BINDING_HEADER, binding)].into_iter(), SignableBody::Bytes(&[]))?;
    let (instructions, _) = sign(request, &params)?.into_parts();
    let mut out = url;
    for (key, value) in instructions.params() {
        out.push('&');
        out.push_str(key);
        out.push('=');
        out.push_str(&encode(value));
    }
    Ok(out)
}

fn encode(value: &str) -> String {
    value.bytes().map(|b| if b.is_ascii_alphanumeric() || b"-_.~".contains(&b) { (b as char).to_string() } else { format!("%{b:02X}") }).collect()
}

/// The credentials that sign a link: this machine's own (a cell's host), or the factoryos role
/// assumed with them (Isaac), renewed 5 min before they end.
struct Identity {
    config: aws_config::SdkConfig,
    sts: aws_sdk_sts::Client,
    role_arn: Option<String>,
    session: String,
    cached: Option<(aws_credential_types::Credentials, SystemTime)>,
}

impl Identity {
    async fn new(role_arn: Option<&str>, region: &str, name: &str) -> Self {
        let config = aws_config::defaults(aws_config::BehaviorVersion::latest()).region(aws_config::Region::new(region.to_owned())).load().await;
        Self {
            sts: aws_sdk_sts::Client::new(&config),
            config,
            role_arn: role_arn.map(str::to_owned),
            session: format!("terminal-{name}").chars().take(64).collect(),
            cached: None,
        }
    }

    async fn credentials(&mut self) -> anyhow::Result<aws_credential_types::Credentials> {
        use aws_credential_types::provider::ProvideCredentials as _;
        let Some(role_arn) = self.role_arn.clone() else {
            // The instance profile's (the session EC2 names after the instance), refreshed by the SDK.
            let provider = self.config.credentials_provider().context("no AWS credentials on this machine")?;
            return provider.provide_credentials().await.context("this machine's AWS credentials");
        };
        if let Some((creds, expires)) = &self.cached
            && SystemTime::now() + Duration::from_secs(300) < *expires
        {
            return Ok(creds.clone());
        }
        let assumed = self.sts.assume_role().role_arn(&role_arn).role_session_name(&self.session).duration_seconds(3600).send().await.context("sts:AssumeRole")?;
        let c = assumed.credentials.context("sts:AssumeRole returned no credentials")?;
        let expires = SystemTime::try_from(c.expiration).unwrap_or_else(|_| SystemTime::now() + Duration::from_secs(900));
        let creds = aws_credential_types::Credentials::new(c.access_key_id, c.secret_access_key, Some(c.session_token), Some(expires), "factoryos-terminal");
        self.cached = Some((creds.clone(), expires));
        Ok(creds)
    }
}

enum Happened {
    Output(Vec<u8>),
    Exit(Option<i32>),
}

struct Shell {
    pty: Pty,
    cwd: String,
    osc7: Osc7,
    /// Output while the link was down.
    pending: Vec<u8>,
    ended: Option<Option<i32>>,
}

/// One workspace's link and the shells it opened.
struct Link {
    url: String,
    binding: Option<String>,
    shells: HashMap<u32, Shell>,
    events: mpsc::UnboundedSender<(u32, Happened)>,
    happened: mpsc::UnboundedReceiver<(u32, Happened)>,
    down_since: Option<Instant>,
}

impl Link {
    fn new(url: String, binding: Option<String>) -> Self {
        let (events, happened) = mpsc::unbounded_channel();
        Self { url, binding, shells: HashMap::new(), events, happened, down_since: None }
    }

    fn open(&mut self, s: &Settings, num: u32, cols: u16, rows: u16) -> FromAgent {
        let (shell, args) = factoryos_pty::shell_command(s.shell.as_deref().unwrap_or(""));
        let (out, ended) = (self.events.clone(), self.events.clone());
        match Pty::spawn(&shell, &args, &s.cwd, (cols, rows), move |b| drop(out.send((num, Happened::Output(b.to_vec())))), move |c| drop(ended.send((num, Happened::Exit(c))))) {
            Ok(pty) => {
                let cwd = s.cwd.display().to_string();
                let name = pty.name.clone();
                self.shells.insert(num, Shell { pty, cwd: cwd.clone(), osc7: Osc7::default(), pending: Vec::new(), ended: None });
                FromAgent::Opened { num, shell: name, cwd }
            }
            Err(err) => FromAgent::Failed { num, error: format!("{err:#}") },
        }
    }

    /// Something a shell did, while the link is down: kept for the next link.
    fn keep(&mut self, num: u32, happened: Happened) {
        let Some(shell) = self.shells.get_mut(&num) else { return };
        match happened {
            Happened::Output(bytes) => {
                if let Some(cwd) = shell.osc7.feed(&bytes) {
                    shell.cwd = cwd;
                }
                shell.pending.extend_from_slice(&bytes);
                let over = shell.pending.len().saturating_sub(PENDING_BYTES);
                shell.pending.drain(..over);
            }
            Happened::Exit(code) => shell.ended = Some(code),
        }
    }

    /// Wait out `delay` without a link (the shells' output kept), killing orphans past the grace.
    async fn wait(&mut self, delay: Duration) {
        let until = tokio::time::Instant::now() + delay;
        loop {
            tokio::select! {
                () = tokio::time::sleep_until(until) => break,
                Some((num, happened)) = self.happened.recv() => self.keep(num, happened),
            }
        }
        if self.down_since.is_some_and(|t| t.elapsed() > ORPHAN_GRACE) && !self.shells.is_empty() {
            tracing::warn!("{}: no link for {}s, killing {} shells", self.url, ORPHAN_GRACE.as_secs(), self.shells.len());
            self.shells.clear();
        }
    }
}

type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

async fn connect(link: &Link, identity: Option<&mut Identity>, region: &str) -> anyhow::Result<Socket> {
    let mut request = link.url.as_str().into_client_request()?;
    if let (Some(identity), Some(binding)) = (identity, &link.binding) {
        let creds = identity.credentials().await?;
        let iam = presign(&creds, region, binding, SystemTime::now())?;
        request.headers_mut().insert(IAM_HEADER, iam.parse()?);
    }
    let (socket, _) = tokio_tungstenite::connect_async(request).await.with_context(|| format!("link {}", link.url))?;
    Ok(socket)
}

fn text(message: &FromAgent) -> Message {
    Message::text(serde_json::to_string(message).unwrap_or_default())
}

/// One connected link, until it drops.
async fn serve(link: &mut Link, s: &Settings, socket: Socket) -> anyhow::Result<()> {
    let (mut tx, mut rx) = socket.split();
    let running: Vec<u32> = link.shells.iter().filter(|(_, sh)| sh.ended.is_none()).map(|(n, _)| *n).collect();
    tx.send(text(&FromAgent::Hello { name: s.name.clone(), label: s.label.clone(), os: std::env::consts::OS.to_owned(), sessions: running })).await?;
    // What happened meanwhile.
    let nums: Vec<u32> = link.shells.keys().copied().collect();
    for num in nums {
        let Some(shell) = link.shells.get_mut(&num) else { continue };
        let pending = std::mem::take(&mut shell.pending);
        if !pending.is_empty() {
            tx.send(Message::binary(link::frame(num, &pending))).await?;
        }
        tx.send(text(&FromAgent::Cwd { num, cwd: shell.cwd.clone() })).await?;
        if let Some(code) = shell.ended {
            tx.send(text(&FromAgent::Exit { num, code })).await?;
            link.shells.remove(&num);
        }
    }
    link.down_since = None;
    let mut poll = tokio::time::interval(CWD_POLL);
    let mut keepalive = tokio::time::interval_at(tokio::time::Instant::now() + KEEPALIVE, KEEPALIVE);
    loop {
        tokio::select! {
            _ = keepalive.tick() => tx.send(Message::Ping(Vec::new().into())).await?,
            message = rx.next() => match message {
                Some(Ok(Message::Text(t))) => match serde_json::from_str::<ToAgent>(&t) {
                    Ok(ToAgent::Open { num, cols, rows }) => {
                        let answer = link.open(s, num, cols, rows);
                        tx.send(text(&answer)).await?;
                    }
                    Ok(ToAgent::Resize { num, cols, rows }) => {
                        if let Some(shell) = link.shells.get(&num) {
                            shell.pty.resize(cols, rows);
                        }
                    }
                    Ok(ToAgent::Close { num }) => {
                        link.shells.remove(&num);
                    }
                    Err(err) => tracing::warn!("{}: {err}", link.url),
                },
                Some(Ok(Message::Binary(frame))) => {
                    if let Some((num, bytes)) = link::unframe(&frame)
                        && let Some(shell) = link.shells.get(&num)
                    {
                        shell.pty.write(bytes.to_vec());
                    }
                }
                Some(Ok(Message::Ping(data))) => tx.send(Message::Pong(data)).await?,
                Some(Ok(Message::Close(_)) | Err(_)) | None => return Ok(()),
                Some(Ok(_)) => {}
            },
            Some((num, happened)) = link.happened.recv() => match happened {
                Happened::Output(bytes) => {
                    if let Some(shell) = link.shells.get_mut(&num)
                        && let Some(cwd) = shell.osc7.feed(&bytes)
                        && cwd != shell.cwd
                    {
                        shell.cwd = cwd.clone();
                        tx.send(text(&FromAgent::Cwd { num, cwd })).await?;
                    }
                    if let Err(err) = tx.send(Message::binary(link::frame(num, &bytes))).await {
                        link.keep(num, Happened::Output(bytes));
                        return Err(err.into());
                    }
                }
                Happened::Exit(code) => {
                    if link.shells.remove(&num).is_some() {
                        tx.send(text(&FromAgent::Exit { num, code })).await?;
                    }
                }
            },
            _ = poll.tick() => {
                let mut changed = Vec::new();
                for (num, shell) in &mut link.shells {
                    if let Some(cwd) = shell.pty.cwd()
                        && cwd != shell.cwd
                    {
                        shell.cwd = cwd.clone();
                        changed.push((*num, cwd));
                    }
                }
                for (num, cwd) in changed {
                    tx.send(text(&FromAgent::Cwd { num, cwd })).await?;
                }
            }
        }
    }
}

/// 5 s doubling to 2 min (each attempt is an STS call at the edge); reset by a link up a minute.
struct Backoff {
    next: Duration,
}

impl Backoff {
    const FIRST: Duration = Duration::from_secs(5);
    const CAP: Duration = Duration::from_secs(120);

    fn failed(&mut self) -> Duration {
        let delay = self.next;
        self.next = (self.next * 2).min(Self::CAP);
        delay
    }

    fn up_for(&mut self, time: Duration) {
        if time >= Duration::from_secs(60) {
            self.next = Self::FIRST;
        }
    }
}

async fn run_link(mut link: Link, s: Settings) {
    let mut identity = if s.edge { Some(Identity::new(s.role_arn.as_deref(), &s.sts_region, &s.name).await) } else { None };
    let mut backoff = Backoff { next: Backoff::FIRST };
    loop {
        let started = Instant::now();
        match connect(&link, identity.as_mut(), &s.sts_region).await {
            Ok(socket) => {
                tracing::info!("{}: linked", link.url);
                if let Err(err) = serve(&mut link, &s, socket).await {
                    tracing::warn!("{}: {err:#}", link.url);
                }
                tracing::info!("{}: link down", link.url);
            }
            Err(err) => tracing::warn!("{err:#}"),
        }
        link.down_since.get_or_insert_with(Instant::now);
        backoff.up_for(started.elapsed());
        let delay = backoff.failed();
        link.wait(delay).await;
    }
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into())).init();
    let _ = rustls::crypto::ring::default_provider().install_default();
    let args: Vec<String> = std::env::args().collect();
    let file = match args.iter().position(|a| a == "--env-file") {
        Some(i) => {
            let path = args.get(i + 1).context("--env-file <file>")?;
            env_file(&std::fs::read_to_string(path).with_context(|| format!("--env-file {path}"))?)
        }
        None => HashMap::new(),
    };
    // The process's environment first, then the file's.
    let s = Settings::from_env(|k| std::env::var(k).ok().or_else(|| file.get(k).cloned()))?;
    tracing::info!("terminal agent {} ({}): {} link(s)", s.name, s.label, s.links.len());
    let runtime = tokio::runtime::Runtime::new()?;
    runtime.block_on(async move {
        let tasks: Vec<_> = s.links.iter().cloned().map(|(url, binding)| tokio::spawn(run_link(Link::new(url, binding), s.clone()))).collect();
        for task in tasks {
            let _ = task.await;
        }
    });
    Ok(())
}

/// `KEY=value` lines (`#` comments; values may be quoted), as systemd's EnvironmentFile.
fn env_file(text: &str) -> HashMap<String, String> {
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| {
            let v = v.trim();
            let v = v.strip_prefix('"').and_then(|v| v.strip_suffix('"')).or_else(|| v.strip_prefix('\'').and_then(|v| v.strip_suffix('\''))).unwrap_or(v);
            (k.trim().to_owned(), v.to_owned())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env<'a>(pairs: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<String> + 'a {
        move |k| pairs.iter().find(|(key, _)| *key == k).map(|(_, v)| (*v).to_owned())
    }

    #[test]
    fn settings_for_the_edge_and_for_development() {
        let s = Settings::from_env(env(&[
            ("TERMINAL_NAME", "plc-host"),
            ("FACTORYOS_EDGE", "https://factoryos.example/"),
            ("FACTORYOS_WORKSPACES", "jdoe, ci"),
            ("FACTORYOS_ROLE_ARN", "arn:aws:iam::111122223333:role/factoryos-terminal-plc-host"),
        ]))
        .unwrap();
        assert_eq!(
            s.links,
            [
                ("wss://factoryos.example/workspace/jdoe/terminal-link/plc-host".to_owned(), Some("jdoe/plc-host".to_owned())),
                ("wss://factoryos.example/workspace/ci/terminal-link/plc-host".to_owned(), Some("ci/plc-host".to_owned())),
            ]
        );
        assert_eq!(s.label, "plc-host");
        // A cell's host: no role, its instance profile signs.
        let cell = Settings::from_env(env(&[("TERMINAL_NAME", "kvc-1"), ("FACTORYOS_EDGE", "https://e.example"), ("FACTORYOS_WORKSPACES", "jdoe")])).unwrap();
        assert!(cell.edge && cell.role_arn.is_none());
        // An Isaac instance's link.env, as its tunnel's.
        let isaac = Settings::from_env(env(&[
            ("ISAAC_NAME", "i-0123456789abcdef0"),
            ("FACTORYOS_EDGE", "https://e.example"),
            ("FACTORYOS_WORKSPACE", "jdoe"),
            ("FACTORYOS_ROLE_ARN", "arn:aws:iam::111122223333:role/factoryos-isaac-i-0123456789abcdef0"),
        ]))
        .unwrap();
        assert_eq!(isaac.links[0].0, "wss://e.example/workspace/jdoe/terminal-link/i-0123456789abcdef0");
        let dev = Settings::from_env(env(&[("TERMINAL_NAME", "dev"), ("TERMINAL_LINK_URL", "ws://127.0.0.1:8080/terminal-link/dev")])).unwrap();
        assert_eq!(dev.links, [("ws://127.0.0.1:8080/terminal-link/dev".to_owned(), None)]);
        assert!(dev.role_arn.is_none());
        for bad in [
            env(&[("TERMINAL_NAME", "Bad")]),
            env(&[("TERMINAL_NAME", "dev"), ("TERMINAL_LINK_URL", "ws://192.0.2.5:8080/terminal-link/dev")]),
            env(&[("TERMINAL_NAME", "a"), ("FACTORYOS_EDGE", "http://x"), ("FACTORYOS_WORKSPACES", "w"), ("FACTORYOS_ROLE_ARN", "arn:aws:iam::1:role/r")]),
            env(&[("TERMINAL_NAME", "a"), ("FACTORYOS_EDGE", "https://x"), ("FACTORYOS_WORKSPACES", "w"), ("FACTORYOS_ROLE_ARN", "nope")]),
        ] {
            assert!(Settings::from_env(bad).is_err());
        }
    }

    #[test]
    fn the_presigned_url_is_what_the_edge_checks() {
        let creds = aws_credential_types::Credentials::new("AKIDEXAMPLE", "secret", Some("token".into()), None, "test");
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(1_792_000_000);
        let url = presign(&creds, "us-east-1", "jdoe/plc-host", now).unwrap();
        assert!(url.starts_with("https://sts.us-east-1.amazonaws.com/?Action=GetCallerIdentity&Version=2011-06-15&"), "{url}");
        // One query parameter per line: on one line, the session token's parameter followed by the signature's reads
        // as a token assigned a key to gitleaks' generic-api-key rule (the public export's secret scan).
        for part in [
            "X-Amz-Algorithm=AWS4-HMAC-SHA256",
            "X-Amz-Expires=60",
            "X-Amz-SignedHeaders=host%3Bx-factoryos-terminal",
            "X-Amz-Security-Token=token",
            "X-Amz-Signature=",
        ] {
            assert!(url.contains(part), "{part} in {url}");
        }
    }

    #[test]
    fn env_files_as_systemd_reads_them() {
        let f = env_file("# agent\nTERMINAL_NAME=plc\nFACTORYOS_EDGE=\"https://e.example\"\n\nTERMINAL_LABEL='PLC host'\nbroken\n");
        assert_eq!(f["TERMINAL_NAME"], "plc");
        assert_eq!(f["FACTORYOS_EDGE"], "https://e.example");
        assert_eq!(f["TERMINAL_LABEL"], "PLC host");
        assert_eq!(f.len(), 3);
    }

    #[test]
    fn backoff_doubles_and_resets() {
        let mut b = Backoff { next: Backoff::FIRST };
        assert_eq!([b.failed(), b.failed(), b.failed()], [5, 10, 20].map(Duration::from_secs));
        for _ in 0..10 {
            b.failed();
        }
        assert_eq!(b.failed(), Backoff::CAP);
        b.up_for(Duration::from_secs(61));
        assert_eq!(b.failed(), Backoff::FIRST);
    }

    #[test]
    fn output_is_kept_while_the_link_is_down() {
        let mut link = Link::new("ws://127.0.0.1:1/terminal-link/t".into(), None);
        let s = Settings { name: "t".into(), label: "t".into(), links: vec![], role_arn: None, sts_region: "us-east-1".into(), shell: Some("/bin/sh".into()), cwd: std::env::temp_dir(), edge: false };
        let FromAgent::Opened { num, .. } = link.open(&s, 7, 80, 24) else { panic!("not opened") };
        link.keep(num, Happened::Output(b"\x1b]7;file://h/srv/cell\x07out".to_vec()));
        link.keep(num, Happened::Exit(Some(0)));
        let shell = &link.shells[&num];
        assert!(shell.pending.ends_with(b"out"));
        assert_eq!(shell.cwd, "/srv/cell");
        assert_eq!(shell.ended, Some(Some(0)));
    }
}
