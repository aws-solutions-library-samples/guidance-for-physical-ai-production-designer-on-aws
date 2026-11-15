//! The Agentic Planning chat, bridged from ACP to the `/chat` WebSocket.
//!
//! The viewer cannot run the planner agent itself: on the web there is no process to spawn, and the
//! agent belongs next to the cell anyway (it reaches the planner's MCP servers over localhost). So
//! the backend owns it. It spawns `kiro-cli acp` in the configured `agenticRepoDir` — where
//! `.kiro/agents/` and `.kiro/settings/mcp.json` live, so the agent finds the `factoryos-planner`
//! agent and the planner's MCP servers — speaks ACP to it, and republishes the conversation as the
//! small [`factoryos_protocol::chat`] events the viewer renders.
//!
//! ACP is driven the way Rerun's agent panel drives it (`re_agent`): the `agent-client-protocol`
//! crate brings its own `futures` executor, so the connection lives on one dedicated thread and
//! talks to the tokio side through channels.
//!
//! There are several sessions at once (the viewer's chat tabs, e.g. two planners and a diagnosis),
//! each a [`ChatSession`] with its own `kiro-cli` process, transcript, directory (`agent.log`,
//! `agent.pid`) and events; [`ChatService`] creates, lists and closes them, up to
//! [`ChatOptions::max_sessions`]. Each session is shared by every viewer attached to it, which is
//! what makes a page reload (or a second browser) show the conversation so far instead of starting
//! over. `/chat` without a session is [`DEFAULT_SESSION`], the single chat of older viewers.

use std::collections::HashMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use agent_client_protocol::schema::v1::{
    CancelNotification, ContentBlock, Implementation, InitializeRequest, NewSessionRequest,
    PermissionOptionId, PromptRequest, RequestPermissionOutcome,
    RequestPermissionRequest, RequestPermissionResponse, SelectedPermissionOutcome, SessionId,
    SessionNotification, SessionUpdate, StopReason, TextContent, ToolCallStatus,
};
use agent_client_protocol::schema::ProtocolVersion;
use agent_client_protocol::{AcpAgent, AcpAgentConfig, Agent, Client, ConnectionTo, LineDirection, Responder};
use factoryos_protocol::chat::{
    ChatCommand, ChatEntry, ChatEvent, ChatRole, ChatSessionInfo, ChatSessions, ChatState, DEFAULT_MAX_SESSIONS,
    DEFAULT_SESSION, PermissionOption, ToolStatus, session_status,
};
use futures::StreamExt as _;
use futures::channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use tokio::sync::broadcast;

use crate::config_store::ConfigStore;

/// The agent to ask for, as the native viewer's chat does. Only passed when the checkout declares
/// it; otherwise kiro-cli uses its default agent and the checkout's `mcp.json`.
const PLANNER_AGENT: &str = "factoryos-planner";

/// How far a slow WebSocket may fall behind before it is dropped.
const EVENT_CHANNEL_CAPACITY: usize = 1024;

/// How many of the agent's last stderr lines to keep for explaining a failure.
const STDERR_LINES: usize = 50;

/// What the tokio side asks the connection thread to do.
enum Command {
    Prompt(String),
    Cancel,
    Permission { id: String, option_id: String },
    Shutdown,
}

/// A permission request the agent is blocked on until a viewer answers it.
struct Pending {
    id: String,
    title: String,
    options: Vec<PermissionOption>,
    responder: Responder<RequestPermissionResponse>,
}

/// The chat as every viewer sees it.
#[derive(Default)]
struct Shared {
    state: ChatState,
    agent: Option<String>,
    entries: Vec<ChatEntry>,
    /// The entries currently being streamed into, so chunks append instead of piling up.
    open_agent: Option<usize>,
    open_thought: Option<usize>,
    /// ACP tool-call id → transcript index, so later updates find their entry.
    tools: HashMap<String, usize>,
    /// Whether a prompt is being answered. The agent takes one at a time, so this decides both
    /// whether a new prompt is accepted and whether the state is Thinking or Ready.
    turn_active: bool,
}

/// How the sessions are run.
#[derive(Clone, Debug)]
pub struct ChatOptions {
    /// Each session's directory (`<dir>/<id>/`: `agent.log`, `agent.pid`).
    pub sessions_dir: PathBuf,
    /// How many sessions may be open at once (each can run a `kiro-cli`).
    pub max_sessions: usize,
    /// Instead of `kiro-cli acp …` (`FACTORYOS_CHAT_AGENT_COMMAND`, a shell command line run in the
    /// checkout): another ACP agent, or a fake one in tests.
    pub agent_command: Option<String>,
}

impl ChatOptions {
    /// `FACTORYOS_CHAT_MAX_SESSIONS` and `FACTORYOS_CHAT_AGENT_COMMAND`, sessions under `sessions_dir`.
    pub fn from_env(sessions_dir: PathBuf) -> Self {
        let max_sessions = std::env::var("FACTORYOS_CHAT_MAX_SESSIONS")
            .ok()
            .and_then(|value| value.trim().parse().ok())
            .filter(|&max: &usize| max > 0)
            .unwrap_or(DEFAULT_MAX_SESSIONS);
        let agent_command = std::env::var("FACTORYOS_CHAT_AGENT_COMMAND").ok().filter(|c| !c.trim().is_empty());
        Self { sessions_dir, max_sessions, agent_command }
    }
}

/// How many old session directories (their logs) to keep when the backend starts.
const KEPT_SESSION_DIRS: usize = 50;

/// Every open session, oldest first.
pub struct ChatService {
    config: Arc<ConfigStore>,
    options: ChatOptions,
    sessions: Mutex<Vec<Arc<ChatSession>>>,
    next_id: AtomicU64,
}

impl ChatService {
    pub fn new(config: Arc<ConfigStore>, options: ChatOptions) -> Arc<Self> {
        prune_session_dirs(&options.sessions_dir, KEPT_SESSION_DIRS);
        Arc::new(Self { config, options, sessions: Mutex::new(Vec::new()), next_id: AtomicU64::new(1) })
    }

    pub fn list(&self) -> ChatSessions {
        let sessions = self.sessions.lock().unwrap().clone();
        ChatSessions {
            sessions: sessions.iter().map(|session| session.info()).collect(),
            max_sessions: self.options.max_sessions,
        }
    }

    pub fn get(&self, id: &str) -> Option<Arc<ChatSession>> {
        self.sessions.lock().unwrap().iter().find(|session| session.id == id).cloned()
    }

    /// A new session with a fresh agent; with `prompt`, the agent starts on it right away.
    pub fn create(&self, title: &str, prompt: Option<String>) -> Result<Arc<ChatSession>, String> {
        let millis = now_ms();
        let id = format!("{millis:x}-{}", self.next_id.fetch_add(1, Ordering::Relaxed));
        let title = if title.trim().is_empty() { "Chat" } else { title.trim() };
        let session = self.insert(id, title)?;
        if let Some(prompt) = prompt.filter(|prompt| !prompt.trim().is_empty()) {
            session.handle(ChatCommand::Prompt { text: prompt });
        }
        Ok(session)
    }

    /// The session of `/chat` without `?session=` (older viewers), created on first use.
    pub fn default_session(&self) -> Result<Arc<ChatSession>, String> {
        match self.get(DEFAULT_SESSION) {
            Some(session) => Ok(session),
            None => self.insert(DEFAULT_SESSION.to_owned(), "Chat"),
        }
    }

    fn insert(&self, id: String, title: &str) -> Result<Arc<ChatSession>, String> {
        let mut sessions = self.sessions.lock().unwrap();
        if let Some(existing) = sessions.iter().find(|session| session.id == id) {
            return Ok(existing.clone());
        }
        if sessions.len() >= self.options.max_sessions {
            return Err(format!(
                "{} agent chats are open, the most this server runs at once (each is a kiro-cli process): \
                 close a tab you no longer need, then try again",
                sessions.len()
            ));
        }
        let session = ChatSession::new(
            id,
            title.to_owned(),
            self.config.clone(),
            &self.options.sessions_dir,
            self.options.agent_command.clone(),
        );
        sessions.push(session.clone());
        Ok(session)
    }

    /// Close a session: stop its agent, tell its viewers, forget it. `false` if there is none.
    pub fn close(&self, id: &str) -> bool {
        let session = {
            let mut sessions = self.sessions.lock().unwrap();
            let Some(position) = sessions.iter().position(|session| session.id == id) else {
                return false;
            };
            sessions.remove(position)
        };
        session.close();
        true
    }

    /// Stop every agent process when the backend goes down.
    pub fn shutdown(&self) {
        for session in std::mem::take(&mut *self.sessions.lock().unwrap()) {
            session.close();
        }
    }
}

/// One agent session: one `kiro-cli` (started by the first prompt), its transcript and its viewers.
pub struct ChatSession {
    pub id: String,
    title: String,
    created_ms: u64,
    /// `agent.log` and `agent.pid` live here.
    dir: PathBuf,
    agent_command: Option<String>,
    config: Arc<ConfigStore>,
    shared: Mutex<Shared>,
    events: broadcast::Sender<ChatEvent>,
    /// `None` until the first prompt starts the agent, and again after it exits.
    commands: Mutex<Option<UnboundedSender<Command>>>,
    /// Which agent start `commands` belongs to, so an old connection thread that ends late (after
    /// Restart started a new one) does not clear the new one's.
    generation: AtomicU64,
    /// Permission requests waiting for an answer (the thread fills this, viewers drain it).
    pending: Mutex<Vec<Pending>>,
    /// The agent's recent stderr. kiro-cli explains a failed start there (e.g. not signed in),
    /// before any ACP traffic, so the connection error alone does not say why.
    stderr: Arc<Mutex<Vec<String>>>,
    log: Arc<Mutex<Option<std::fs::File>>>,
    /// Closed: nothing starts an agent any more.
    closed: AtomicBool,
}

impl ChatSession {
    fn new(
        id: String,
        title: String,
        config: Arc<ConfigStore>,
        sessions_dir: &Path,
        agent_command: Option<String>,
    ) -> Arc<Self> {
        let (events, _) = broadcast::channel(EVENT_CHANNEL_CAPACITY);
        let dir = sessions_dir.join(&id);
        let log = std::fs::create_dir_all(&dir)
            .and_then(|()| std::fs::OpenOptions::new().create(true).append(true).open(dir.join("agent.log")))
            .inspect_err(|err| tracing::warn!("chat session {id}: no log in {}: {err}", dir.display()))
            .ok();
        let session = Arc::new(Self {
            id,
            title,
            created_ms: now_ms(),
            dir,
            agent_command,
            config,
            shared: Mutex::new(Shared::default()),
            events,
            commands: Mutex::new(None),
            generation: AtomicU64::new(0),
            pending: Mutex::new(Vec::new()),
            stderr: Arc::new(Mutex::new(Vec::new())),
            log: Arc::new(Mutex::new(log)),
            closed: AtomicBool::new(false),
        });
        session.log(&format!("session {} \"{}\"", session.id, session.title));
        session
    }

    pub fn info(&self) -> ChatSessionInfo {
        let permission_pending = !self.pending.lock().unwrap().is_empty();
        let shared = self.shared.lock().unwrap();
        ChatSessionInfo {
            id: self.id.clone(),
            title: self.title.clone(),
            state: shared.state,
            status: session_status(shared.state, permission_pending, &shared.entries),
            agent: shared.agent.clone(),
            created_ms: self.created_ms,
        }
    }

    /// The session's directory (`agent.log`, `agent.pid`).
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The running agent's process id, from `agent.pid` (the shell `exec`s the agent, so it is the
    /// agent's, and the leader of its process group).
    pub fn agent_pid(&self) -> Option<i32> {
        std::fs::read_to_string(self.dir.join("agent.pid")).ok()?.trim().parse().ok().filter(|&pid| pid > 0)
    }

    /// Append a line to `agent.log`.
    fn log(&self, line: &str) {
        write_log(&self.log, line);
    }

    /// Stop the agent and tell every viewer the session is gone.
    fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
        self.log("closed");
        self.stop_agent();
        self.broadcast(ChatEvent::SessionClosed);
    }

    /// Ask the agent to stop, and make sure it does: its connection only reads commands once the
    /// session is open, so one still starting (or hung) is killed with its process group.
    fn stop_agent(&self) {
        // The old connection's thread, when it ends, leaves the session alone.
        self.generation.fetch_add(1, Ordering::SeqCst);
        if let Some(commands) = self.commands.lock().unwrap().take() {
            commands.unbounded_send(Command::Shutdown).ok();
        }
        self.pending.lock().unwrap().clear();
        if let Some(pid) = self.agent_pid() {
            std::fs::remove_file(self.dir.join("agent.pid")).ok();
            kill_process_group(pid);
        }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<ChatEvent> {
        self.events.subscribe()
    }

    /// What a freshly connected viewer needs to render the chat as it stands.
    pub fn snapshot(&self) -> Vec<ChatEvent> {
        let shared = self.shared.lock().unwrap();
        let mut events = vec![
            ChatEvent::State {
                state: shared.state,
                agent: shared.agent.clone(),
            },
            ChatEvent::Transcript {
                entries: shared.entries.clone(),
            },
        ];
        drop(shared);
        // Re-ask for anything the agent is still blocked on, so a reloaded page can answer it.
        for pending in self.pending.lock().unwrap().iter() {
            events.push(ChatEvent::Permission {
                id: pending.id.clone(),
                title: pending.title.clone(),
                options: pending.options.clone(),
            });
        }
        events
    }

    pub fn handle(self: &Arc<Self>, command: ChatCommand) {
        match command {
            ChatCommand::Prompt { text } => {
                if text.trim().is_empty() {
                    return;
                }
                // One turn at a time, for every viewer: kiro-cli rejects a second prompt while one
                // is running ("Prompt already in progress"), and the other tab cannot see why.
                {
                    let mut shared = self.shared.lock().unwrap();
                    if shared.turn_active {
                        drop(shared);
                        self.broadcast(ChatEvent::Error {
                            message: "The planner is still answering the last prompt; answer its questions, \
                                      or press Stop, then try again."
                                .to_owned(),
                        });
                        return;
                    }
                    shared.turn_active = true;
                }
                self.log(&format!("user: {text}"));
                self.push(ChatRole::User, text.clone());
                if !self.start_agent_if_needed() {
                    // Said why (fail); the next prompt may try again.
                    self.shared.lock().unwrap().turn_active = false;
                    return;
                }
                self.set_state(ChatState::Thinking);
                self.send(Command::Prompt(text));
            }
            ChatCommand::Cancel => self.send(Command::Cancel),
            ChatCommand::Permission { id, option_id } => {
                self.send(Command::Permission { id, option_id });
            }
            ChatCommand::Restart => {
                self.log("restart");
                self.stop_agent();
                *self.shared.lock().unwrap() = Shared::default();
                self.broadcast(ChatEvent::Transcript { entries: Vec::new() });
                self.set_state(ChatState::Idle);
            }
        }
    }

    // ---- transcript / event helpers -------------------------------------------------------

    fn broadcast(&self, event: ChatEvent) {
        // An error only means nobody is connected; the transcript keeps the content either way.
        self.events.send(event).ok();
    }

    /// The agent's answer of the turn that just ended, for the log.
    fn log_answer(&self) {
        let answer = {
            let shared = self.shared.lock().unwrap();
            shared.entries.iter().rev().take_while(|e| e.role != ChatRole::User).find(|e| e.role == ChatRole::Agent).map(|e| e.text.clone())
        };
        if let Some(answer) = answer {
            self.log(&format!("agent: {}", answer.trim()));
        }
    }

    fn set_state(&self, state: ChatState) {
        let agent = {
            let mut shared = self.shared.lock().unwrap();
            shared.state = state;
            shared.agent.clone()
        };
        self.broadcast(ChatEvent::State { state, agent });
    }

    fn fail(&self, message: impl Into<String>) {
        let message = message.into();
        self.log(&format!("error: {message}"));
        self.push(ChatRole::Status, message.clone());
        self.broadcast(ChatEvent::Error { message });
        self.set_state(ChatState::Failed);
    }

    /// Append an entry and tell every viewer about it.
    fn push(&self, role: ChatRole, text: String) -> usize {
        let entry = {
            let mut shared = self.shared.lock().unwrap();
            // Any new entry ends the streaming ones.
            shared.open_agent = None;
            shared.open_thought = None;
            let entry = ChatEntry {
                index: shared.entries.len(),
                role,
                text,
                tool_status: None,
            };
            shared.entries.push(entry.clone());
            entry
        };
        let index = entry.index;
        self.broadcast(ChatEvent::Entry { entry });
        index
    }

    /// Append streamed text to the open entry of `role`, starting one if there is none.
    fn stream(&self, role: ChatRole, text: &str) {
        if text.is_empty() {
            return;
        }
        let open = {
            let shared = self.shared.lock().unwrap();
            match role {
                ChatRole::Thought => shared.open_thought,
                _ => shared.open_agent,
            }
        };
        match open {
            Some(index) => {
                {
                    let mut shared = self.shared.lock().unwrap();
                    if let Some(entry) = shared.entries.get_mut(index) {
                        entry.text.push_str(text);
                    }
                }
                self.broadcast(ChatEvent::Chunk {
                    index,
                    text: text.to_owned(),
                });
            }
            None => {
                let index = self.push(role, text.to_owned());
                let mut shared = self.shared.lock().unwrap();
                match role {
                    ChatRole::Thought => shared.open_thought = Some(index),
                    _ => shared.open_agent = Some(index),
                }
            }
        }
    }

    fn on_update(&self, update: SessionUpdate) {
        match update {
            SessionUpdate::AgentMessageChunk(chunk) => {
                if let ContentBlock::Text(text) = chunk.content {
                    self.stream(ChatRole::Agent, &text.text);
                }
            }
            SessionUpdate::AgentThoughtChunk(chunk) => {
                if let ContentBlock::Text(text) = chunk.content {
                    self.stream(ChatRole::Thought, &text.text);
                }
            }
            SessionUpdate::ToolCall(call) => {
                let id = call.tool_call_id.0.to_string();
                let status = tool_status(call.status);
                self.tool_entry(id, Some(call.title), Some(status));
            }
            SessionUpdate::ToolCallUpdate(update) => {
                let id = update.tool_call_id.0.to_string();
                let status = update.fields.status.map(tool_status);
                self.tool_entry(id, update.fields.title, status);
            }
            // Plans, usage, available commands and mode changes are not shown in this chat; the
            // planner reports its plan through its own tools and the Behaviour Tree Executor.
            _ => {}
        }
    }

    /// Create or update the entry of a tool call.
    fn tool_entry(&self, id: String, title: Option<String>, status: Option<ToolStatus>) {
        let existing = self.shared.lock().unwrap().tools.get(&id).copied();
        match existing {
            Some(index) => {
                let (text, status) = {
                    let mut shared = self.shared.lock().unwrap();
                    let Some(entry) = shared.entries.get_mut(index) else {
                        return;
                    };
                    if let Some(title) = title.clone() {
                        entry.text = title;
                    }
                    if let Some(status) = status {
                        entry.tool_status = Some(status);
                    }
                    (entry.text.clone(), entry.tool_status.unwrap_or(ToolStatus::Pending))
                };
                self.broadcast(ChatEvent::ToolUpdate {
                    index,
                    status,
                    text: Some(text),
                });
            }
            None => {
                let title = title.unwrap_or_else(|| id.clone());
                self.log(&format!("tool: {title}"));
                let index = self.push(ChatRole::Tool, title);
                let status = status.unwrap_or(ToolStatus::Pending);
                let mut shared = self.shared.lock().unwrap();
                shared.tools.insert(id, index);
                if let Some(entry) = shared.entries.get_mut(index) {
                    entry.tool_status = Some(status);
                }
                drop(shared);
                self.broadcast(ChatEvent::ToolUpdate {
                    index,
                    status,
                    text: None,
                });
            }
        }
    }

    fn on_permission(&self, request: RequestPermissionRequest, responder: Responder<RequestPermissionResponse>) {
        // Show the tool the request is about, so the user sees what they are allowing.
        let id = request.tool_call.tool_call_id.0.to_string();
        let title = request.tool_call.fields.title.clone();
        self.tool_entry(id.clone(), title.clone(), Some(ToolStatus::Pending));

        let options: Vec<PermissionOption> = request
            .options
            .iter()
            .map(|option| PermissionOption {
                id: option.option_id.0.to_string(),
                name: option.name.clone(),
                kind: Some(format!("{:?}", option.kind).to_lowercase()),
            })
            .collect();
        let title = title.unwrap_or_else(|| "Run this tool?".to_owned());
        self.pending.lock().unwrap().push(Pending {
            id: id.clone(),
            title: title.clone(),
            options: options.clone(),
            responder,
        });
        self.broadcast(ChatEvent::Permission { id, title, options });
    }

    /// Answer a permission request. Without `option_id` the request is cancelled.
    fn answer_permission(&self, id: &str, option_id: Option<&str>) {
        let pending = {
            let mut queue = self.pending.lock().unwrap();
            let Some(position) = queue.iter().position(|pending| pending.id == id) else {
                return; // already answered, e.g. by another viewer
            };
            queue.remove(position)
        };
        let outcome = match option_id {
            Some(option_id) => RequestPermissionOutcome::Selected(SelectedPermissionOutcome::new(
                PermissionOptionId::new(option_id.to_owned()),
            )),
            None => RequestPermissionOutcome::Cancelled,
        };
        if let Err(err) = pending.responder.respond(RequestPermissionResponse::new(outcome)) {
            tracing::warn!("failed to answer a permission request: {err}");
        }
        self.broadcast(ChatEvent::PermissionResolved { id: id.to_owned() });
    }

    fn send(self: &Arc<Self>, command: Command) {
        let commands = self.commands.lock().unwrap();
        if let Some(sender) = commands.as_ref() {
            if sender.unbounded_send(command).is_err() {
                drop(commands);
                self.fail("the agent connection is gone; press Restart");
            }
        }
    }

    // ---- the agent process ----------------------------------------------------------------

    /// Spawn `kiro-cli acp` and its ACP connection, unless they are already running.
    /// `false` if there is no agent to send to (closed, not configured, or failed to start).
    fn start_agent_if_needed(self: &Arc<Self>) -> bool {
        if self.closed.load(Ordering::SeqCst) {
            return false;
        }
        let mut commands = self.commands.lock().unwrap();
        if commands.is_some() {
            return true;
        }
        let config = self.config.get();
        let repo_dir = PathBuf::from(config.agentic_repo_dir.trim());
        if repo_dir.as_os_str().is_empty() {
            drop(commands);
            self.fail("set Agentic Planning's repository directory in Configuration first");
            return false;
        }

        let (sender, receiver) = unbounded();
        *commands = Some(sender);
        let generation = self.generation.load(Ordering::SeqCst);
        drop(commands);
        self.set_state(ChatState::Starting);
        self.log(&format!("starting the agent in {}", repo_dir.display()));

        self.stderr.lock().unwrap().clear();
        let this = self.clone();
        let spawned = std::thread::Builder::new()
            .name(format!("factoryos-chat-{}", self.id))
            .spawn(move || {
                let result = this.connect(repo_dir, receiver);
                // Restarted or closed meanwhile: the session is someone else's now.
                if this.generation.load(Ordering::SeqCst) != generation {
                    this.log("the previous agent stopped");
                    return;
                }
                if this.closed.load(Ordering::SeqCst) {
                    return;
                }
                match result {
                    Err(_) if this.stderr_says_not_logged_in() => this.need_login(),
                    Err(err) => this.fail(format!("the agent connection failed: {err}")),
                    Ok(()) => {
                        this.log("the agent exited");
                        this.push(ChatRole::Status, "The agent exited.".to_owned());
                        this.set_state(ChatState::Idle);
                    }
                }
                this.commands.lock().unwrap().take();
                std::fs::remove_file(this.dir.join("agent.pid")).ok();
                this.shared.lock().unwrap().turn_active = false;
                // Nothing can answer these any more.
                this.pending.lock().unwrap().clear();
            });
        if let Err(err) = spawned {
            self.commands.lock().unwrap().take();
            self.fail(format!("failed to start the chat thread: {err}"));
            return false;
        }
        true
    }

    /// Runs the whole ACP connection on this thread, returning when the agent exits.
    fn connect(self: &Arc<Self>, repo_dir: PathBuf, commands: UnboundedReceiver<Command>) -> anyhow::Result<()> {
        // `--agent` only works when the checkout declares it.
        let kiro = match &self.agent_command {
            Some(command) => format!("exec {command}"),
            None => {
                let mut kiro = "exec kiro-cli acp".to_owned();
                if repo_dir.join(format!(".kiro/agents/{PLANNER_AGENT}.json")).is_file() {
                    kiro.push_str(&format!(" --agent {PLANNER_AGENT}"));
                }
                if self.config.get().agentic_trust_all_tools {
                    kiro.push_str(" --trust-all-tools");
                }
                kiro
            }
        };
        // kiro-cli reads `.kiro/` (the agent and the planner's MCP servers in `settings/mcp.json`)
        // from its PROCESS working directory; the session's cwd in `session/new` does not count. The
        // ACP launcher has no working-directory option, so start it through a shell that changes
        // there first, as Rerun's agent panel does for the native viewer.
        // `$$` stays the agent's pid through `exec`: `agent.pid` is how closing the tab stops it.
        let pid_file = self.dir.join("agent.pid");
        let script = format!(
            "echo $$ > {} 2>/dev/null; cd {} && {kiro}",
            shell_quote(&pid_file.to_string_lossy()),
            shell_quote(&repo_dir.to_string_lossy())
        );
        let stderr = self.stderr.clone();
        let log = self.log.clone();
        let config = AcpAgentConfig::new("/bin/sh")
            .args(["-c".to_owned(), script])
            .env("FACTORYOS_CHAT_SESSION", self.id.clone())
            .env("FACTORYOS_CHAT_SESSION_DIR", self.dir.to_string_lossy().into_owned());
        let agent = AcpAgent::new(config).with_debug(
            // The JSON-RPC lines are the conversation, which the transcript already holds; only
            // stderr is worth keeping, and it is where kiro-cli explains a failure to start.
            move |line, direction| {
                if direction == LineDirection::Stderr {
                    tracing::info!("kiro-cli: {line}");
                    write_log(&log, &format!("stderr: {line}"));
                    let mut stderr = stderr.lock().unwrap();
                    stderr.push(line.to_owned());
                    let excess = stderr.len().saturating_sub(STDERR_LINES);
                    stderr.drain(..excess);
                }
            },
        );

        let notifications = self.clone();
        let permissions = self.clone();
        let foreground = self.clone();
        futures::executor::block_on(
            Client
                .builder()
                .name("factoryos-backend")
                .on_receive_notification(
                    async move |notification: SessionNotification, _cx| {
                        notifications.on_update(notification.update);
                        Ok(())
                    },
                    agent_client_protocol::on_receive_notification!(),
                )
                .on_receive_request(
                    async move |request: RequestPermissionRequest, responder, _cx| {
                        permissions.on_permission(request, responder);
                        Ok(())
                    },
                    agent_client_protocol::on_receive_request!(),
                )
                .connect_with(agent, |cx: ConnectionTo<Agent>| async move {
                    foreground.drive(cx, repo_dir, commands).await
                }),
        )
        .map_err(|err| anyhow::anyhow!("{err}"))?;
        Ok(())
    }

    /// Initialize, open a session, then serve commands until the viewer asks to stop.
    async fn drive(
        self: &Arc<Self>,
        cx: ConnectionTo<Agent>,
        repo_dir: PathBuf,
        mut commands: UnboundedReceiver<Command>,
    ) -> Result<(), agent_client_protocol::Error> {
        let client_info = Implementation::new("factoryos-backend", env!("CARGO_PKG_VERSION"));
        let init = cx
            .send_request(InitializeRequest::new(ProtocolVersion::V1).client_info(client_info))
            .block_task()
            .await?;
        if let Some(info) = &init.agent_info {
            let mut shared = self.shared.lock().unwrap();
            shared.agent = Some(info.name.clone());
        }

        // The session's cwd is the checkout, which is how kiro-cli finds its MCP servers.
        let session = match cx
            .send_request(NewSessionRequest::new(repo_dir.clone()))
            .block_task()
            .await
        {
            Ok(response) => response.session_id,
            Err(err) if err.code == agent_client_protocol::Error::auth_required().code => {
                self.need_login();
                return Ok(());
            }
            Err(err) => return Err(err),
        };
        // The prompt that started the agent is usually already queued; stay Thinking for it.
        self.set_state(self.idle_or_thinking());

        while let Some(command) = commands.next().await {
            match command {
                Command::Prompt(text) => {
                    self.prompt(&cx, &session, text)?;
                }
                Command::Cancel => {
                    cx.send_notification(CancelNotification::new(session.clone()))?;
                }
                Command::Permission { id, option_id } => {
                    self.answer_permission(&id, Some(&option_id));
                }
                Command::Shutdown => break,
            }
        }
        Ok(())
    }

    fn prompt(
        self: &Arc<Self>,
        cx: &ConnectionTo<Agent>,
        session: &SessionId,
        text: String,
    ) -> Result<(), agent_client_protocol::Error> {
        let prompt = vec![ContentBlock::Text(TextContent::new(text))];
        let this = self.clone();
        cx.send_request(PromptRequest::new(session.clone(), prompt))
            .on_receiving_result(move |result| async move {
                match result {
                    Ok(response) => {
                        let stop_reason = match response.stop_reason {
                            StopReason::EndTurn => "end_turn",
                            StopReason::Cancelled => "cancelled",
                            StopReason::MaxTokens => "max_tokens",
                            StopReason::MaxTurnRequests => "max_turn_requests",
                            StopReason::Refusal => "refusal",
                            _ => "finished",
                        };
                        this.shared.lock().unwrap().turn_active = false;
                        this.log_answer();
                        this.log(&format!("turn end: {stop_reason}"));
                        this.broadcast(ChatEvent::TurnEnd {
                            stop_reason: stop_reason.to_owned(),
                        });
                        this.set_state(ChatState::Ready);
                    }
                    Err(err) => {
                        this.shared.lock().unwrap().turn_active = false;
                        this.log(&format!("turn failed: {err}"));
                        this.broadcast(ChatEvent::Error {
                            message: format!("{err}"),
                        });
                        this.set_state(ChatState::Ready);
                    }
                }
                Ok(())
            })?;
        Ok(())
    }
}

impl ChatSession {
    fn idle_or_thinking(&self) -> ChatState {
        if self.shared.lock().unwrap().turn_active {
            ChatState::Thinking
        } else {
            ChatState::Ready
        }
    }

    /// kiro-cli exits before speaking ACP when it has no login, saying so on stderr.
    fn stderr_says_not_logged_in(&self) -> bool {
        self.stderr
            .lock()
            .unwrap()
            .iter()
            .any(|line| line.to_ascii_lowercase().contains("not logged in"))
    }

    /// Signing in needs the user's browser (a device code), so it cannot happen from here.
    fn need_login(&self) {
        self.log("kiro-cli is not signed in");
        self.push(
            ChatRole::Status,
            "kiro-cli is not signed in on the backend's host. Press \"Sign in to the agent\" \
             (it opens the approval page in your browser), then press Restart."
                .to_owned(),
        );
        self.set_state(ChatState::AuthRequired);
    }
}

fn now_ms() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64)
}

fn write_log(log: &Mutex<Option<std::fs::File>>, line: &str) {
    if let Some(file) = log.lock().unwrap().as_mut() {
        let secs = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
        writeln!(file, "{secs} {line}").ok();
    }
}

/// SIGTERM the agent's process group, then SIGKILL whatever is left after a moment.
fn kill_process_group(pid: i32) {
    #[cfg(unix)]
    {
        // SAFETY: plain syscalls on a pid; an error (ESRCH) only means it is gone already.
        unsafe { libc::killpg(pid, libc::SIGTERM) };
        std::thread::spawn(move || {
            for _ in 0..30 {
                std::thread::sleep(Duration::from_millis(100));
                // SAFETY: as above; signal 0 only checks whether the group still exists.
                if unsafe { libc::killpg(pid, 0) } != 0 {
                    return;
                }
            }
            // SAFETY: as above.
            unsafe { libc::killpg(pid, libc::SIGKILL) };
        });
    }
    #[cfg(not(unix))]
    let _ = pid;
}

/// Keep the newest `keep` session directories (their logs), remove the rest.
fn prune_session_dirs(dir: &Path, keep: usize) {
    let Ok(read) = std::fs::read_dir(dir) else { return };
    let mut dirs: Vec<(SystemTime, PathBuf)> = read
        .flatten()
        .filter(|entry| entry.file_type().is_ok_and(|kind| kind.is_dir()))
        .map(|entry| (entry.metadata().and_then(|m| m.modified()).unwrap_or(UNIX_EPOCH), entry.path()))
        .collect();
    dirs.sort_by(|a, b| b.0.cmp(&a.0));
    for (_, old) in dirs.into_iter().skip(keep) {
        std::fs::remove_dir_all(old).ok();
    }
}

/// `path` as one POSIX shell word.
pub(super) fn shell_quote(path: &str) -> String {
    format!("'{}'", path.replace('\'', r"'\''"))
}

fn tool_status(status: ToolCallStatus) -> ToolStatus {
    match status {
        ToolCallStatus::Pending => ToolStatus::Pending,
        ToolCallStatus::InProgress => ToolStatus::InProgress,
        ToolCallStatus::Completed => ToolStatus::Completed,
        ToolCallStatus::Failed => ToolStatus::Failed,
        _ => ToolStatus::Pending,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quotes_paths_for_the_shell() {
        assert_eq!(shell_quote("/opt/factoryos/content/umarell"), "'/opt/factoryos/content/umarell'");
        assert_eq!(shell_quote("/a b/it's"), r"'/a b/it'\''s'");
    }

    /// A service whose "agent" is `sleep` (it never speaks ACP, which is all these tests need: a
    /// process per session), in a fresh directory.
    fn service(name: &str, max_sessions: usize) -> (Arc<ChatService>, PathBuf) {
        let root = std::env::temp_dir().join(format!("factoryos-chat-{name}-{}-{}", std::process::id(), now_ms()));
        std::fs::create_dir_all(&root).unwrap();
        let config = Arc::new(ConfigStore::open(root.join("config.json")));
        let mut settings = config.get();
        settings.agentic_repo_dir = root.to_string_lossy().into_owned();
        config.set(settings);
        let options = ChatOptions {
            sessions_dir: root.join("chat-sessions"),
            max_sessions,
            agent_command: Some("sleep 30".to_owned()),
        };
        (ChatService::new(config, options), root)
    }

    fn wait_for<T>(what: &str, mut check: impl FnMut() -> Option<T>) -> T {
        for _ in 0..100 {
            if let Some(value) = check() {
                return value;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        panic!("timed out waiting for {what}");
    }

    /// Alive and not a zombie.
    fn running(pid: i32) -> bool {
        std::process::Command::new("ps")
            .args(["-o", "stat=", "-p", &pid.to_string()])
            .output()
            .is_ok_and(|out| out.status.success() && !String::from_utf8_lossy(&out.stdout).trim().starts_with('Z'))
    }

    fn texts(session: &ChatSession) -> Vec<String> {
        session.shared.lock().unwrap().entries.iter().map(|e| e.text.clone()).collect()
    }

    #[test]
    fn sessions_are_isolated() {
        let (chat, root) = service("isolated", 6);
        let plan = chat.create("Plan jam2 · umarell_u3", Some("plan jam2".to_owned())).unwrap();
        let diagnose = chat.create("Diagnose step 3", Some("why did step 3 fail".to_owned())).unwrap();
        assert_ne!(plan.id, diagnose.id);
        assert_ne!(plan.dir(), diagnose.dir());

        let plan_pid = wait_for("the planner's agent", || plan.agent_pid());
        let diagnose_pid = wait_for("the diagnosis's agent", || diagnose.agent_pid());
        assert_ne!(plan_pid, diagnose_pid, "one process per session");
        assert!(running(plan_pid) && running(diagnose_pid));

        assert_eq!(texts(&plan), ["plan jam2"]);
        assert_eq!(texts(&diagnose), ["why did step 3 fail"]);
        let log = std::fs::read_to_string(plan.dir().join("agent.log")).unwrap();
        assert!(log.contains("user: plan jam2") && !log.contains("step 3"), "{log}");

        let list = chat.list();
        let titles: Vec<_> = list.sessions.iter().map(|s| s.title.as_str()).collect();
        assert_eq!(titles, ["Plan jam2 · umarell_u3", "Diagnose step 3"]);
        assert_eq!(list.max_sessions, 6);
        assert!(list.sessions.iter().all(|s| s.status == factoryos_protocol::chat::SessionStatus::Running));

        // A second prompt while the first is answered goes nowhere else either.
        let mut events = plan.subscribe();
        plan.handle(ChatCommand::Prompt { text: "again".to_owned() });
        assert!(matches!(events.try_recv(), Ok(ChatEvent::Error { .. })));
        assert_eq!(texts(&diagnose), ["why did step 3 fail"]);

        chat.shutdown();
        std::fs::remove_dir_all(root).ok();
    }

    #[test]
    fn the_session_limit_is_enforced() {
        let (chat, root) = service("cap", 2);
        let first = chat.create("one", None).unwrap();
        chat.default_session().unwrap();
        let err = chat.create("three", None).err().expect("over the limit");
        assert!(err.contains("2 agent chats are open") && err.contains("close a tab"), "{err}");
        assert_eq!(chat.list().sessions.len(), 2);
        // Closing one makes room.
        assert!(chat.close(&first.id));
        assert!(!chat.close(&first.id), "closed already");
        chat.create("three", None).unwrap();
        chat.shutdown();
        std::fs::remove_dir_all(root).ok();
    }

    #[test]
    fn a_failed_start_does_not_block_the_next_prompt() {
        let (chat, root) = service("unconfigured", 6);
        let mut settings = chat.config.get();
        settings.agentic_repo_dir = " ".to_owned();
        chat.config.set(settings);
        let session = chat.create("Plan", Some("plan".to_owned())).unwrap();
        assert_eq!(session.info().state, ChatState::Failed);
        assert!(!session.shared.lock().unwrap().turn_active, "the next prompt may try again");
        chat.shutdown();
        std::fs::remove_dir_all(root).ok();
    }

    #[test]
    fn old_clients_share_the_default_session() {
        let (chat, root) = service("default", 6);
        let a = chat.default_session().unwrap();
        let b = chat.default_session().unwrap();
        assert_eq!(a.id, DEFAULT_SESSION);
        assert!(Arc::ptr_eq(&a, &b));
        assert!(chat.get(DEFAULT_SESSION).is_some());
        chat.shutdown();
        std::fs::remove_dir_all(root).ok();
    }

    #[test]
    fn closing_a_session_stops_its_agent() {
        let (chat, root) = service("close", 6);
        let keep = chat.create("keep", Some("hello".to_owned())).unwrap();
        let gone = chat.create("gone", Some("hello".to_owned())).unwrap();
        let keep_pid = wait_for("an agent", || keep.agent_pid());
        let gone_pid = wait_for("an agent", || gone.agent_pid());
        let mut events = gone.subscribe();

        assert!(chat.close(&gone.id));
        wait_for("the closed session's agent to stop", || (!running(gone_pid)).then_some(()));
        assert!(matches!(events.try_recv(), Ok(ChatEvent::SessionClosed)));
        assert!(chat.get(&gone.id).is_none());
        // A closed session never starts another agent.
        gone.handle(ChatCommand::Prompt { text: "still there?".to_owned() });
        assert!(gone.agent_pid().is_none());
        // The other one keeps running.
        assert!(running(keep_pid));

        // Restart stops the agent too, and the next prompt starts a new one.
        keep.handle(ChatCommand::Restart);
        wait_for("the restarted session's old agent to stop", || (!running(keep_pid)).then_some(()));
        std::thread::sleep(Duration::from_millis(200));
        assert_eq!(keep.info().state, ChatState::Idle, "the old agent's end does not fail the restarted session");
        keep.handle(ChatCommand::Prompt { text: "again".to_owned() });
        let new_pid = wait_for("a new agent", || keep.agent_pid().filter(|&pid| pid != keep_pid));
        assert!(running(new_pid));

        chat.shutdown();
        wait_for("every agent to stop", || (!running(new_pid)).then_some(()));
        std::fs::remove_dir_all(root).ok();
    }
}
