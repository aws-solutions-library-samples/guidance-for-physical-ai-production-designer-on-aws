//! Workspace IDE terminals: shells on the backend's host (and, through `factoryos-terminal-agent`
//! links, on other machines of the workspace), one WebSocket each ([`crate::api::TERMINAL_WS`]).
//! Only the workspace's owner (or a caller on loopback) may use them, as the IDE's files.
//!
//! On the WebSocket, binary messages are the bytes: keystrokes to the shell, its output back.
//! Text messages are JSON control: [`TerminalControl`] from the viewer, [`TerminalNotice`] to it.

use serde::{Deserialize, Serialize};

/// The backend's own host, as a [`TerminalHost::id`].
pub const BACKEND_HOST: &str = "backend";

/// A machine terminals can run on.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TerminalHost {
    pub id: String,
    pub label: String,
    /// `linux`, `macos`, `windows`.
    pub os: String,
    pub online: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TerminalInfo {
    pub id: String,
    pub host: String,
    /// The shell's name (`zsh`, `bash`, `pwsh`).
    pub shell: String,
    /// Its current folder, as last seen.
    #[serde(default)]
    pub cwd: String,
    pub alive: bool,
    #[serde(default)]
    pub exit_code: Option<i32>,
}

/// `POST` [`crate::api::TERMINALS`]: a new shell.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NewTerminal {
    pub host: String,
    /// An IDE root ([`crate::files::FileRoot::id`]) to start in (on the backend's host).
    #[serde(default)]
    pub root: Option<String>,
    pub cols: u16,
    pub rows: u16,
}

/// From the viewer, as a text message.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TerminalControl {
    Resize { cols: u16, rows: u16 },
}

/// To the viewer, as a text message.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TerminalNotice {
    /// The shell's current folder changed.
    Cwd(String),
    /// The output so far has been sent (what follows is live).
    Replayed,
    /// The shell ended.
    Exit(Option<i32>),
}

/// A terminal agent's link to a workspace backend (`factoryos-terminal-agent`, on another machine):
/// one WebSocket at [`crate::api::TERMINAL_LINK`], dialled by the agent through the edge (which
/// verifies its IAM identity). Text messages are [`ToAgent`] / [`FromAgent`] JSON; binary ones the
/// sessions' bytes, framed by [`frame`].
pub mod link {
    use serde::{Deserialize, Serialize};

    /// From the backend.
    #[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "snake_case", tag = "type")]
    pub enum ToAgent {
        Open { num: u32, cols: u16, rows: u16 },
        Resize { num: u32, cols: u16, rows: u16 },
        Close { num: u32 },
    }

    /// From the agent.
    #[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "snake_case", tag = "type")]
    pub enum FromAgent {
        /// First on each link: who it is, and the sessions it still runs (after a reconnect).
        Hello { name: String, label: String, os: String, #[serde(default)] sessions: Vec<u32> },
        Opened { num: u32, shell: String, cwd: String },
        Failed { num: u32, error: String },
        Cwd { num: u32, cwd: String },
        Exit { num: u32, code: Option<i32> },
    }

    /// A session's bytes on the link: its number (4 bytes, big-endian), then the bytes.
    pub fn frame(num: u32, bytes: &[u8]) -> Vec<u8> {
        [num.to_be_bytes().as_slice(), bytes].concat()
    }

    pub fn unframe(frame: &[u8]) -> Option<(u32, &[u8])> {
        let (num, bytes) = frame.split_first_chunk::<4>()?;
        Some((u32::from_be_bytes(*num), bytes))
    }

    /// An agent's name: `[a-z][a-z0-9-]{0,31}` (part of its IAM role's name and the link's URL).
    pub fn valid_name(name: &str) -> bool {
        let mut chars = name.chars();
        chars.next().is_some_and(|c| c.is_ascii_lowercase())
            && name.len() <= 32
            && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_messages_are_small_json() {
        assert_eq!(serde_json::to_string(&TerminalControl::Resize { cols: 80, rows: 24 }).unwrap(), r#"{"resize":{"cols":80,"rows":24}}"#);
        assert_eq!(serde_json::to_string(&TerminalNotice::Cwd("/tmp".into())).unwrap(), r#"{"cwd":"/tmp"}"#);
        assert_eq!(serde_json::to_string(&TerminalNotice::Replayed).unwrap(), r#""replayed""#);
        assert_eq!(serde_json::to_string(&TerminalNotice::Exit(Some(0))).unwrap(), r#"{"exit":0}"#);
    }

    #[test]
    fn link_messages_and_frames() {
        use link::*;
        assert_eq!(serde_json::to_string(&ToAgent::Open { num: 3, cols: 80, rows: 24 }).unwrap(), r#"{"type":"open","num":3,"cols":80,"rows":24}"#);
        let hello: FromAgent = serde_json::from_str(r#"{"type":"hello","name":"isaac-1","label":"Isaac Sim","os":"linux"}"#).unwrap();
        assert_eq!(hello, FromAgent::Hello { name: "isaac-1".into(), label: "Isaac Sim".into(), os: "linux".into(), sessions: vec![] });
        assert_eq!(unframe(&frame(258, b"ls\r")), Some((258, b"ls\r".as_slice())));
        assert_eq!(unframe(b"abc"), None);
        assert!(valid_name("plc-host") && valid_name("win-ide-1"));
        assert!(!valid_name("") && !valid_name("1abc") && !valid_name("Isaac") && !valid_name("a/b") && !valid_name(&"a".repeat(33)));
    }
}
