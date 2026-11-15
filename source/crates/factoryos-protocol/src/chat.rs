//! The Agentic Planning chat: what the viewer and `factoryos-backend` exchange over the `/chat`
//! WebSocket.
//!
//! The backend owns the agent process (`kiro-cli acp`) and speaks ACP to it; this is the small,
//! wasm-safe surface the viewer sees instead. The backend keeps the transcript, so a reloaded or
//! second viewer gets the whole conversation on connect ([`ChatEvent::Transcript`]).
//!
//! There are several chats at once (the viewer's tabs), each its own agent session: its own
//! `kiro-cli` process, transcript and log. [`crate::api::CHAT_SESSIONS`] lists, creates and closes
//! them; `/chat?session=<id>` ([`crate::api::chat_ws`]) attaches to one. `/chat` without a session
//! is the [`DEFAULT_SESSION`], the one chat older viewers know.

use serde::{Deserialize, Serialize};

/// Where a transcript entry came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChatRole {
    /// Typed by the user (or sent on their behalf, e.g. the Plan prompt).
    User,
    /// The agent's answer.
    Agent,
    /// The agent's reasoning, which agents send separately from their answer.
    Thought,
    /// A tool call, e.g. one of the planner's MCP tools.
    Tool,
    /// A status line from the backend (starting up, authentication, errors).
    Status,
}

/// How a [`ChatRole::Tool`] entry is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolStatus {
    Pending,
    InProgress,
    Completed,
    Failed,
}

/// One entry of the transcript. Text arrives in pieces: [`ChatEvent::Chunk`] appends to the entry
/// with the matching index.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChatEntry {
    /// Position in the transcript, and what [`ChatEvent::Chunk`] refers to.
    pub index: usize,
    pub role: ChatRole,
    pub text: String,
    /// Only for [`ChatRole::Tool`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_status: Option<ToolStatus>,
}

/// What the agent process is doing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChatState {
    /// No agent process yet; the next prompt starts one.
    #[default]
    Idle,
    /// Spawned, initializing, opening the session.
    Starting,
    /// Session open, waiting for a prompt.
    Ready,
    /// A prompt is being answered.
    Thinking,
    /// The agent needs the user to sign in (`kiro-cli login` on the host).
    AuthRequired,
    /// The process failed or exited; the next prompt tries again.
    Failed,
}

/// One choice offered for a permission request.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PermissionOption {
    pub id: String,
    pub name: String,
    /// ACP's kind, e.g. `allow_once`, `allow_always`, `reject_once`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<String>,
}

/// Backend → viewer.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ChatEvent {
    /// Sent on connect and whenever it changes.
    State { state: ChatState, agent: Option<String> },
    /// The whole transcript, sent once per connection.
    Transcript { entries: Vec<ChatEntry> },
    /// A new entry was appended.
    Entry { entry: ChatEntry },
    /// More text for the entry with this index.
    Chunk { index: usize, text: String },
    /// A tool entry changed state (and possibly got a better title).
    ToolUpdate {
        index: usize,
        status: ToolStatus,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        text: Option<String>,
    },
    /// The agent finished the turn (ACP's stop reason, e.g. `end_turn`, `cancelled`).
    TurnEnd { stop_reason: String },
    /// The agent wants permission for a tool call. Answer with [`ChatCommand::Permission`].
    Permission {
        id: String,
        title: String,
        options: Vec<PermissionOption>,
    },
    /// A permission request is no longer waiting (answered elsewhere, or the turn was cancelled).
    PermissionResolved { id: String },
    Error { message: String },
    /// The session was closed (its tab closed, here or in another viewer) and its agent stopped;
    /// the socket closes next. Only sent to `/chat?session=<id>` connections.
    SessionClosed,
}

/// Viewer → backend.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ChatCommand {
    /// Send a prompt, starting the agent if it is not running yet.
    Prompt { text: String },
    /// Cancel the running turn.
    Cancel,
    /// Answer a [`ChatEvent::Permission`].
    Permission { id: String, option_id: String },
    /// Stop the agent process and clear the transcript; the next prompt starts a fresh session.
    Restart,
}

/// The session `/chat` without `?session=` attaches to: the single chat of older viewers.
pub const DEFAULT_SESSION: &str = "default";

/// How many sessions the backend keeps open unless told otherwise (`FACTORYOS_CHAT_MAX_SESSIONS`).
pub const DEFAULT_MAX_SESSIONS: usize = 6;

/// A session at a glance, for its tab's status dot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SessionStatus {
    /// Nothing asked yet.
    #[default]
    Idle,
    /// The agent is starting or answering.
    Running,
    /// The agent waits for the user: a permission request, or its turn ended with a question.
    Waiting,
    /// The agent finished its turn.
    Done,
    /// The agent failed, exited with an error, or needs a sign-in.
    Error,
}

/// What a tab shows: running while it works, waiting when it is blocked on the user (a permission
/// request, or a turn that ended on a question), done after a turn, error when it failed.
pub fn session_status(state: ChatState, permission_pending: bool, entries: &[ChatEntry]) -> SessionStatus {
    if permission_pending {
        return SessionStatus::Waiting;
    }
    match state {
        ChatState::Starting | ChatState::Thinking => SessionStatus::Running,
        ChatState::Failed | ChatState::AuthRequired => SessionStatus::Error,
        ChatState::Idle | ChatState::Ready => {
            let last_answer = entries.iter().rev().find(|entry| entry.role == ChatRole::Agent);
            match last_answer {
                Some(entry) if entry.text.trim_end().ends_with('?') => SessionStatus::Waiting,
                Some(_) => SessionStatus::Done,
                None if entries.iter().any(|entry| entry.role == ChatRole::User) => SessionStatus::Done,
                None => SessionStatus::Idle,
            }
        }
    }
}

/// One agent session, as [`crate::api::CHAT_SESSIONS`] lists it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChatSessionInfo {
    pub id: String,
    /// What it is for, e.g. "Plan jam2 · umarell_u3" or "Layout jam2".
    pub title: String,
    pub state: ChatState,
    pub status: SessionStatus,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub agent: Option<String>,
    /// When it was created (Unix milliseconds); the list is in this order.
    pub created_ms: u64,
}

/// `GET` [`crate::api::CHAT_SESSIONS`].
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ChatSessions {
    pub sessions: Vec<ChatSessionInfo>,
    /// How many sessions the backend allows at once.
    pub max_sessions: usize,
}

/// `POST` [`crate::api::CHAT_SESSIONS`]: a new session, with a fresh agent.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NewChatSession {
    pub title: String,
    /// Sent as the first prompt right away (the agent starts with it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub prompt: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(index: usize, role: ChatRole, text: &str) -> ChatEntry {
        ChatEntry { index, role, text: text.to_owned(), tool_status: None }
    }

    #[test]
    fn status_follows_the_agent() {
        assert_eq!(session_status(ChatState::Idle, false, &[]), SessionStatus::Idle);
        assert_eq!(session_status(ChatState::Thinking, false, &[]), SessionStatus::Running);
        assert_eq!(session_status(ChatState::Thinking, true, &[]), SessionStatus::Waiting);
        assert_eq!(session_status(ChatState::Failed, false, &[]), SessionStatus::Error);
        let asked = [entry(0, ChatRole::User, "plan"), entry(1, ChatRole::Agent, "Which planner?\n")];
        assert_eq!(session_status(ChatState::Ready, false, &asked), SessionStatus::Waiting);
        let done = [entry(0, ChatRole::User, "plan"), entry(1, ChatRole::Agent, "Saved the plan.")];
        assert_eq!(session_status(ChatState::Ready, false, &done), SessionStatus::Done);
    }
}
