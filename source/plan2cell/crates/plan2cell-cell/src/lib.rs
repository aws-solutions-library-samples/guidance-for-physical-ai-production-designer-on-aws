//! plan2cell cell: bring a plan2cell build up on the cell, deterministically.
//!
//! ```text
//! preflight  ssh to the PLC host, CODESYS MCP tools/list has every tool plan2cell calls,
//!            get_codesys_status, every DeviceID in the IDE's device repository
//! runtimes   ssh: push plc/docker/docker-compose.robots.yml, setup.sh --robots-overlay
//! projects   MCP: typed tool calls per project (robot simulators first, head last), read back
//! deploy     MCP: bind by direct IP, login, download, boot application, start (same order)
//! krl        SSM / ssh: KRL of krl/<ns> onto each KUKA virtual controller of [kuka_vc]
//! verify     KUKA VC checks, then MCP: online read of the acceptance values (and the exported
//!            OPC UA symbols of a Symbol Configuration)
//! start      runtimes up (ssh: docker restart), head logged in, KUKA VCs acknowledged and in
//!            EXT (kvc-ack.sh, else kvc-mode.sh), the head pulsed out of error / recovery into
//!            production, logged out (not part of `up`)
//! stop       the head's bStop
//! runtime    ssh: read-only, how long each CODESYS demo runtime still runs ([`runtime`])
//! ```
//!
//! A head or robots that are `factoryos/hook@1` pods (cell.toml `driver = "hook"`) run the same
//! stages through [`hook`] (kubectl exec of `/opt/factoryos/hook <verb>`); [`hooks_check`] is the
//! conformance kit image authors run (`plan2cell cell hooks-check`).
//!
//! The CODESYS side runs through the typed tools of Codesys-MCP-Master 0.19.0 ([`codesys`]
//! plans them, [`mcp`] calls them). The PLC host and the KUKA VC hosts are behind
//! [`remote::Remote`]; [`krl`] has the KUKA side.

pub mod codesys;
pub mod config;
pub mod hook;
pub mod hooks_check;
pub mod krl;
pub mod mcp;
pub mod project;
pub mod remote;
pub mod runtime;
pub mod stages;

use std::path::{Path, PathBuf};

pub use config::CellConfig;
pub use stages::{Cell, Report, Settings, Stage, Tree};
pub use stages::operate::KvcOp;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("config: {0}")]
    Config(String),
    #[error("usage: {0}")]
    Usage(String),
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("plc host: {0}")]
    Remote(String),
    #[error("codesys mcp: {0}")]
    Mcp(String),
    #[error("codesys tool {tool}: {msg}")]
    Tool { tool: String, msg: String },
    /// A CODESYS dialog nobody answered blocks the IDE ([`mcp::dialog`]): no retry helps until a
    /// person answers it.
    #[error("{msg} (codesys tool {tool})")]
    Dialog { tool: String, msg: String },
    #[error("{stage}: {msg}")]
    Failed {
        stage: &'static str,
        code: i32,
        msg: String,
    },
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    /// The plan2cell exit-code numbering (2 usage, 3 config or a human step on the IDE, 4 I/O,
    /// 7 verification, 8 a CODESYS tool failed or the build was rejected).
    pub fn exit_code(&self) -> i32 {
        match self {
            Error::Usage(_) => 2,
            Error::Config(_) | Error::Dialog { .. } => 3,
            Error::Io { .. } | Error::Remote(_) | Error::Mcp(_) => 4,
            Error::Tool { .. } => 8,
            Error::Failed { code, .. } => *code,
        }
    }

    pub fn io(path: &Path, source: std::io::Error) -> Self {
        Error::Io {
            path: path.to_path_buf(),
            source,
        }
    }
}
