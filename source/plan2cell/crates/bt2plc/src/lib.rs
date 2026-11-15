//! bt2plc: umarell behaviour-tree plans → head-PLC code with robot-held interlock zones.
//!
//! `input` (JSON mirror / XML) → `ir` (happens-before, reduced cross-robot edges, zones,
//! schedule) → `backend` (CODESYS now) → output files, checked with trust-platform.

pub mod backend;
pub mod config;
pub mod error;
pub mod hardware;
pub mod input;
pub mod ir;
pub mod trust;
pub mod util;

pub use config::Config;
pub use error::{Diag, DiagCode, Diagnostics, Error, Result};

use backend::model::PlcProject;
use backend::{Flavour, Output};
use input::InputDoc;
use ir::CellPlan;

#[derive(Clone, Debug)]
pub struct GenerateOptions {
    pub flavour: Flavour,
    /// Promote approximation warnings to errors.
    pub strict: bool,
    /// Type-check the generated ST with trust before returning (feature `check`).
    pub st_check: bool,
}

impl Default for GenerateOptions {
    fn default() -> Self {
        Self {
            flavour: Flavour::Codesys,
            strict: false,
            st_check: cfg!(feature = "check"),
        }
    }
}

pub struct Generated {
    pub plan: CellPlan,
    pub projects: Vec<PlcProject>,
    pub output: Output,
    pub diagnostics: Diagnostics,
    /// Number of trust-checked ST errors (0 when the check did not run).
    pub st_errors: usize,
    pub st_checked: bool,
}

/// Plan only (no code generation).
pub fn plan(doc: &InputDoc, cfg: &Config, d: &mut Diagnostics) -> Result<CellPlan> {
    cfg.validate()?;
    ir::plan(doc, cfg, d)
}

/// Full pipeline in memory: plan, PLC projects, all output files, trust check.
pub fn generate(doc: &InputDoc, cfg: &Config, opts: &GenerateOptions) -> Result<Generated> {
    let mut d = Diagnostics::default();
    let plan = plan(doc, cfg, &mut d)?;
    if opts.strict && d.promotable_count() > 0 {
        for x in d.items() {
            eprintln!("{x}");
        }
        return Err(Error::Strict(d.promotable_count()));
    }
    let be = backend::backend_for(opts.flavour)?;
    let projects = be.projects(&plan, cfg)?;
    let mut output = Output::default();
    be.render(&plan, cfg, &projects, &mut output, &mut d)?;
    backend::common::render(&plan, cfg, &d, &mut output)?;
    for p in &projects {
        for f in backend::trust_render::render(p) {
            output.add(format!("trust/{}/src/{}", p.key, f.name), f.text)?;
        }
    }
    #[allow(unused_mut)]
    let (mut st_errors, mut st_checked) = (0usize, false);
    #[cfg(feature = "check")]
    if opts.st_check {
        let diags = trust::check::check_projects(&projects, false);
        st_checked = true;
        st_errors = diags.len();
        if !diags.is_empty() {
            let text: Vec<String> = diags.iter().map(ToString::to_string).collect();
            return Err(Error::StCheck(text.join("\n")));
        }
    }
    Ok(Generated {
        plan,
        projects,
        output,
        diagnostics: d,
        st_errors,
        st_checked,
    })
}
