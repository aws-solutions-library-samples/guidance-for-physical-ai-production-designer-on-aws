//! The machine-readable result of a build, for `plan2cell build --json` / `check --json`:
//! structured diagnostics (severity, code, file:line where there is one) and the files.

use serde::{Deserialize, Serialize};

use crate::{Build, Error, Summary};

/// One diagnostic of a build.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Diagnostic {
    /// `error`, `warning` or `info`.
    pub severity: String,
    /// bt2plc's `E001`/`W003`/..., trust's code for the ST check; none for KRL warnings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    pub message: String,
    /// A generated file, relative to the build directory (`plc/codesys/...`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub line: Option<usize>,
    /// Where in the plan: the behaviour-tree path and node, when there is one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
    /// `plan`, `plc`, `krl` or `st` (the trust type check).
    pub source: String,
}

impl Diagnostic {
    pub fn error(source: &str, message: impl Into<String>) -> Self {
        Self {
            severity: "error".into(),
            code: None,
            message: message.into(),
            file: None,
            line: None,
            location: None,
            source: source.into(),
        }
    }

    pub fn warning(source: &str, message: impl Into<String>) -> Self {
        Self {
            severity: "warning".into(),
            ..Self::error(source, message)
        }
    }

    /// A bt2plc diagnostic.
    pub fn from_plc(d: &bt2plc::error::Diag) -> Self {
        let severity = match d.severity {
            bt2plc::error::Severity::Error => "error",
            bt2plc::error::Severity::Warning => "warning",
            bt2plc::error::Severity::Info => "info",
        };
        let location = match (&d.path, &d.node) {
            (Some(p), Some(n)) => Some(format!("{p} ({n})")),
            (Some(p), None) => Some(p.clone()),
            (None, Some(n)) => Some(n.clone()),
            (None, None) => None,
        };
        Self {
            severity: severity.into(),
            code: Some(d.code.as_str().into()),
            message: d.message.clone(),
            file: None,
            line: None,
            location,
            source: "plc".into(),
        }
    }

    /// One line of the trust check (`st-error[trust:CODE] FILE:LINE:COL: MESSAGE`); `FILE` is
    /// relative to bt2plc's output, so it gets the build's `plc/` prefix.
    pub fn from_st_line(line: &str) -> Option<Self> {
        let rest = line.strip_prefix("st-")?;
        let (severity, rest) = rest.split_once("[trust:")?;
        let (code, rest) = rest.split_once("] ")?;
        let (location, message) = rest.split_once(": ")?;
        let mut parts = location.rsplitn(3, ':');
        let _col = parts.next()?;
        let line_no = parts.next()?.parse::<usize>().ok()?;
        let file = parts.next()?;
        Some(Self {
            severity: severity.into(),
            code: Some(code.into()),
            message: message.into(),
            file: Some(format!("plc/{file}")),
            line: (line_no > 0).then_some(line_no),
            location: None,
            source: "st".into(),
        })
    }
}

/// The diagnostics a failed build has to show: its error, split into its parts where it has them.
pub fn error_diagnostics(e: &Error) -> Vec<Diagnostic> {
    match e {
        Error::Plc(bt2plc::Error::Plan(d)) => vec![Diagnostic::from_plc(d)],
        Error::Plc(bt2plc::Error::StCheck(text)) => {
            let mut out: Vec<Diagnostic> = Vec::new();
            for l in text.lines().filter(|l| !l.trim().is_empty()) {
                match Diagnostic::from_st_line(l) {
                    Some(d) => out.push(d),
                    // A continuation line of the previous message.
                    None => match out.last_mut() {
                        Some(prev) => {
                            prev.message.push('\n');
                            prev.message.push_str(l);
                        }
                        None => out.push(Diagnostic::error("st", l)),
                    },
                }
            }
            if out.is_empty() {
                out.push(Diagnostic::error("st", e.to_string()));
            }
            out
        }
        Error::Krl(bt2krl::Error::Validate(text)) => text
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| Diagnostic::error("krl", l))
            .collect(),
        Error::Krl(_) => vec![Diagnostic::error("krl", e.to_string())],
        // A planned robot language (planned PLC dialects come as `Error::Plc`): the robot side.
        Error::NotImplemented(_) => vec![Diagnostic::error("krl", e.to_string())],
        Error::Plc(_) => vec![Diagnostic::error("plc", e.to_string())],
        Error::Config(_) | Error::Usage(_) | Error::Io { .. } | Error::Sim(_) => {
            vec![Diagnostic::error("plan", e.to_string())]
        }
    }
}

/// A file of the build.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileEntry {
    pub path: String,
    pub bytes: u64,
}

pub const REPORT_SCHEMA: &str = "plan2cell/build-report@1";

/// What `plan2cell build --json` prints (one JSON document on stdout, success or not).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BuildReport {
    pub schema: String,
    pub generator: String,
    pub ok: bool,
    /// The process exit code (0 ok; see [`Error::exit_code`]).
    pub exit_code: i32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// One line, as the text output prints it.
    #[serde(default)]
    pub summary_line: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub summary: Option<Summary>,
    pub diagnostics: Vec<Diagnostic>,
    pub files: Vec<FileEntry>,
    pub notes: Vec<String>,
    /// The build directory, if one was written.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub out: Option<String>,
    pub written: bool,
}

impl BuildReport {
    pub fn ok(b: &Build, summary_line: String, notes: Vec<String>) -> Self {
        Self {
            schema: REPORT_SCHEMA.into(),
            generator: crate::GENERATOR.into(),
            ok: true,
            exit_code: 0,
            error: None,
            summary_line,
            summary: Some(b.summary.clone()),
            diagnostics: b.diags.clone(),
            files: b
                .files
                .iter()
                .map(|(path, text)| FileEntry {
                    path: path.clone(),
                    bytes: text.len() as u64,
                })
                .collect(),
            notes: notes.into_iter().chain(b.notes.iter().cloned()).collect(),
            out: None,
            written: false,
        }
    }

    pub fn failed(e: &Error, diagnostics: Vec<Diagnostic>, notes: Vec<String>) -> Self {
        Self {
            schema: REPORT_SCHEMA.into(),
            generator: crate::GENERATOR.into(),
            ok: false,
            exit_code: e.exit_code(),
            error: Some(e.to_string()),
            summary_line: String::new(),
            summary: None,
            diagnostics,
            files: Vec::new(),
            notes,
            out: None,
            written: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn st_lines_parse() {
        let d = Diagnostic::from_st_line(
            "st-error[trust:E101] codesys/head/POU/FB_Cell.st:12:5: undefined: x",
        )
        .unwrap();
        assert_eq!(d.file.as_deref(), Some("plc/codesys/head/POU/FB_Cell.st"));
        assert_eq!(
            (d.line, d.code.as_deref(), d.severity.as_str()),
            (Some(12), Some("E101"), "error")
        );
        assert_eq!(d.message, "undefined: x");
        assert!(Diagnostic::from_st_line("error[E001]: x").is_none());
        let e = Error::Plc(bt2plc::Error::StCheck(
            "st-error[trust:compile] trust/head:0:0: bad\n  more\n".into(),
        ));
        let ds = error_diagnostics(&e);
        assert_eq!(ds.len(), 1);
        assert_eq!(ds[0].line, None);
        assert!(ds[0].message.ends_with("more"));
    }
}
