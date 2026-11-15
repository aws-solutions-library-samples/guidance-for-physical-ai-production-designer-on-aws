//! Errors, diagnostics and exit codes.
//!
//! Hard failures are [`Error`]s. Everything that only weakens the translation (ignored ports,
//! approximations) is a [`Diag`] collected in [`Diagnostics`]; `--strict` promotes the
//! promotable warnings to an error.

use std::fmt;
use std::path::PathBuf;

use serde::Serialize;

use crate::backend::Flavour;

// ───────────────────────────── diagnostics ─────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Info,
    Warning,
    Error,
}

/// A diagnostic code with its fixed severity and whether `--strict` promotes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum DiagCode {
    E001,
    E002,
    E003,
    E004,
    E005,
    E006,
    E007,
    E008,
    E009,
    E010,
    E011,
    E013,
    E014,
    E015,
    E017,
    E018,
    E019,
    E020,
    E021,
    E022,
    E023,
    W001,
    W002,
    W003,
    W004,
    W005,
    W006,
    W007,
    W009,
    W010,
    W011,
    W012,
    W013,
    W014,
    W015,
    W016,
    W019,
    W020,
    W021,
    I008,
}

impl DiagCode {
    pub fn severity(self) -> Severity {
        let s = self.as_str();
        match s.as_bytes()[0] {
            b'E' => Severity::Error,
            b'W' => Severity::Warning,
            _ => Severity::Info,
        }
    }

    /// Warnings that `--strict` turns into errors (semantic approximations).
    pub fn promotable(self) -> bool {
        use DiagCode::*;
        matches!(
            self,
            W001 | W002 | W003 | W004 | W005 | W006 | W007 | W009 | W015 | W016
        )
    }

    pub fn as_str(self) -> &'static str {
        use DiagCode::*;
        match self {
            E001 => "E001",
            E002 => "E002",
            E003 => "E003",
            E004 => "E004",
            E005 => "E005",
            E006 => "E006",
            E007 => "E007",
            E008 => "E008",
            E009 => "E009",
            E010 => "E010",
            E011 => "E011",
            E013 => "E013",
            E014 => "E014",
            E015 => "E015",
            E017 => "E017",
            E018 => "E018",
            E019 => "E019",
            E020 => "E020",
            E021 => "E021",
            E022 => "E022",
            E023 => "E023",
            W001 => "W001",
            W002 => "W002",
            W003 => "W003",
            W004 => "W004",
            W005 => "W005",
            W006 => "W006",
            W007 => "W007",
            W009 => "W009",
            W010 => "W010",
            W011 => "W011",
            W012 => "W012",
            W013 => "W013",
            W014 => "W014",
            W015 => "W015",
            W016 => "W016",
            W019 => "W019",
            W020 => "W020",
            W021 => "W021",
            I008 => "I008",
        }
    }
}

impl fmt::Display for DiagCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One diagnostic, anchored to a BT path when there is one.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Diag {
    pub code: DiagCode,
    pub severity: Severity,
    pub path: Option<String>,
    pub node: Option<String>,
    pub message: String,
}

impl Diag {
    pub fn new(code: DiagCode, message: impl Into<String>) -> Self {
        Self {
            code,
            severity: code.severity(),
            path: None,
            node: None,
            message: message.into(),
        }
    }

    pub fn at(mut self, path: impl Into<String>) -> Self {
        self.path = Some(path.into());
        self
    }

    pub fn node(mut self, node: impl Into<String>) -> Self {
        self.node = Some(node.into());
        self
    }
}

impl fmt::Display for Diag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sev = match self.severity {
            Severity::Error => "error",
            Severity::Warning => "warning",
            Severity::Info => "info",
        };
        write!(f, "{sev}[{}]", self.code)?;
        if let Some(p) = &self.path {
            write!(f, " {p}")?;
        }
        if let Some(n) = &self.node {
            write!(f, " ({n})")?;
        }
        write!(f, ": {}", self.message)
    }
}

/// Collected non-fatal diagnostics (deduplicated, in emission order).
#[derive(Clone, Debug, Default)]
pub struct Diagnostics {
    items: Vec<Diag>,
}

impl Diagnostics {
    pub fn push(&mut self, d: Diag) {
        if !self.items.contains(&d) {
            self.items.push(d);
        }
    }

    pub fn warn(&mut self, code: DiagCode, path: Option<&str>, message: impl Into<String>) {
        let mut d = Diag::new(code, message);
        d.path = path.map(str::to_string);
        self.push(d);
    }

    pub fn items(&self) -> &[Diag] {
        &self.items
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    pub fn promotable_count(&self) -> usize {
        self.items.iter().filter(|d| d.code.promotable()).count()
    }

    pub fn extend(&mut self, other: Diagnostics) {
        for d in other.items {
            self.push(d);
        }
    }
}

// ───────────────────────────── errors ─────────────────────────────

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("{0}")]
    Plan(Diag),
    #[error("config: {0}")]
    Config(String),
    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("xml: {0}")]
    Xml(String),
    #[error("flavour {0} is not implemented yet (planned; only codesys is available)")]
    NotImplemented(Flavour),
    #[error("{0} warning(s) promoted to errors by --strict")]
    Strict(usize),
    #[error("simulation check failed: {0}")]
    Sim(String),
    #[error("generated ST failed the trust check:\n{0}")]
    StCheck(String),
}

impl Error {
    pub fn plan(code: DiagCode, path: Option<&str>, message: impl Into<String>) -> Self {
        let mut d = Diag::new(code, message);
        d.path = path.map(str::to_string);
        Error::Plan(d)
    }

    pub fn io(path: impl Into<PathBuf>, source: std::io::Error) -> Self {
        Error::Io {
            path: path.into(),
            source,
        }
    }

    /// Process exit code (see README "Exit codes").
    pub fn exit_code(&self) -> i32 {
        match self {
            Error::Plan(_) | Error::Json(_) | Error::Xml(_) => 1,
            Error::Config(_) => 3,
            Error::Io { .. } => 4,
            Error::NotImplemented(_) => 5,
            Error::Strict(_) => 6,
            Error::Sim(_) => 7,
            Error::StCheck(_) => 8,
        }
    }
}

pub type Result<T, E = Error> = std::result::Result<T, E>;
