//! Parse + semantic check of the generated ST with trust-platform (`trust-syntax`, `trust-hir`),
//! one trust project per PLC project. With feature `sim` the bytecode compiler also runs,
//! which catches lowering-only rejections.

use serde::Serialize;
use trust_hir::db::SemanticDatabase;
use trust_hir::{Project, SourceKey};

use crate::backend::model::PlcProject;
use crate::backend::trust_render::{RenderedFile, render};

#[derive(Clone, Debug, Serialize)]
pub struct StDiag {
    pub project: String,
    /// CODESYS source file (output-relative), or `trust/<file>` for generated wrapper lines.
    pub file: String,
    pub line: usize,
    pub col: usize,
    pub code: String,
    pub severity: &'static str,
    pub message: String,
}

impl std::fmt::Display for StDiag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "st-{}[trust:{}] {}:{}:{}: {}",
            self.severity, self.code, self.file, self.line, self.col, self.message
        )
    }
}

fn line_col(text: &str, off: usize) -> (usize, usize) {
    let off = off.min(text.len());
    let pre = &text[..off];
    let line = pre.matches('\n').count() + 1;
    let col = pre[pre.rfind('\n').map_or(0, |i| i + 1)..].chars().count() + 1;
    (line, col)
}

fn diag(
    p: &PlcProject,
    f: &RenderedFile,
    off: usize,
    code: String,
    severity: &'static str,
    message: String,
) -> StDiag {
    let (l, c) = line_col(&f.text, off);
    let (file, line) = f.map_line(l);
    StDiag {
        project: p.key.clone(),
        file,
        line,
        col: c,
        code,
        severity,
        message,
    }
}

/// Errors (and, if `warnings`, warnings) of all projects, sorted by (project, file, line, col).
pub fn check_projects(projects: &[PlcProject], warnings: bool) -> Vec<StDiag> {
    let mut out = Vec::new();
    for p in projects {
        let files = render(p);
        let mut hir = Project::new();
        let mut ids = Vec::new();
        for f in &files {
            for e in trust_syntax::parser::parse(&f.text).errors() {
                out.push(diag(
                    p,
                    f,
                    u32::from(e.range.start()) as usize,
                    "parse".into(),
                    "error",
                    e.message.clone(),
                ));
            }
            ids.push(hir.set_source_text(SourceKey::from_virtual(f.name.clone()), f.text.clone()));
        }
        for (f, id) in files.iter().zip(ids) {
            for d in hir.database().diagnostics(id).iter() {
                let sev = if d.is_error() { "error" } else { "warning" };
                if sev == "warning" && !warnings {
                    continue;
                }
                out.push(diag(
                    p,
                    f,
                    u32::from(d.range.start()) as usize,
                    d.code.code().to_string(),
                    sev,
                    d.message.clone(),
                ));
            }
        }
        #[cfg(feature = "sim")]
        if !out
            .iter()
            .any(|d| d.project == p.key && d.severity == "error")
        {
            let srcs = files
                .iter()
                .map(|f| {
                    trust_runtime::harness::SourceFile::with_path(f.name.clone(), f.text.clone())
                })
                .collect();
            if let Err(e) =
                trust_runtime::harness::CompileSession::from_sources(srcs).build_bytecode_bytes()
            {
                out.push(StDiag {
                    project: p.key.clone(),
                    file: format!("trust/{}", p.key),
                    line: 0,
                    col: 0,
                    code: "compile".into(),
                    severity: "error",
                    message: e.to_string(),
                });
            }
        }
    }
    out.sort_by(|a, b| {
        (&a.project, &a.file, a.line, a.col).cmp(&(&b.project, &b.file, b.line, b.col))
    });
    out
}
