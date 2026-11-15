//! Code-generation backends.
//!
//! The flavour-independent [`crate::ir::CellPlan`] is turned into [`model::PlcProject`]s
//! (one per PLC program) by a flavour backend, then rendered to files. Only CODESYS is
//! implemented; the other flavours are planned and rejected with
//! [`crate::Error::NotImplemented`]. What is shared across flavours: the IR, the constant
//! tables (`codesys::gvl` layouts are plain CSR arrays), `st_fmt` for IEC-ST dialects and
//! the flavour-independent outputs in `common`.

pub mod codesys;
pub mod common;
pub mod model;
pub mod st_fmt;
pub mod template;
pub mod trust_render;

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Component, Path};

use serde::Serialize;

use crate::config::Config;
use crate::error::{Diagnostics, Error, Result};
use crate::ir::CellPlan;

/// Target PLC flavour.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Flavour {
    Codesys,
    Twincat3,
    TiaScl,
    PlcopenXml,
    RockwellL5x,
}

impl Flavour {
    pub const ALL: [Flavour; 5] = [
        Flavour::Codesys,
        Flavour::Twincat3,
        Flavour::TiaScl,
        Flavour::PlcopenXml,
        Flavour::RockwellL5x,
    ];

    pub fn implemented(self) -> bool {
        self == Flavour::Codesys
    }

    pub fn describe(self) -> &'static str {
        match self {
            Flavour::Codesys => {
                "CODESYS V3.5 (ST, split declaration/implementation, PROFINET device tree)"
            }
            Flavour::Twincat3 => "Beckhoff TwinCAT 3 (TcPOU/TcDUT/TcGVL) - planned",
            Flavour::TiaScl => "Siemens TIA Portal SCL - planned",
            Flavour::PlcopenXml => "PLCopen TC6 XML - planned",
            Flavour::RockwellL5x => "Rockwell Studio 5000 L5X - planned",
        }
    }
}

impl fmt::Display for Flavour {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Flavour::Codesys => "codesys",
            Flavour::Twincat3 => "twincat3",
            Flavour::TiaScl => "tia-scl",
            Flavour::PlcopenXml => "plcopen-xml",
            Flavour::RockwellL5x => "rockwell-l5x",
        })
    }
}

/// A flavour backend: plan → PLC projects → files.
pub trait Backend {
    fn flavour(&self) -> Flavour;
    /// The PLC programs (head PLC, robot simulators) as an object model.
    fn projects(&self, plan: &CellPlan, cfg: &Config) -> Result<Vec<model::PlcProject>>;
    /// Flavour-specific files (sources, manifests, build docs).
    fn render(
        &self,
        plan: &CellPlan,
        cfg: &Config,
        projects: &[model::PlcProject],
        out: &mut Output,
        d: &mut Diagnostics,
    ) -> Result<()>;
}

pub fn backend_for(f: Flavour) -> Result<Box<dyn Backend>> {
    match f {
        Flavour::Codesys => Ok(Box::new(codesys::CodesysBackend)),
        other => Err(Error::NotImplemented(other)),
    }
}

/// Deterministic set of output files (relative paths with '/', LF line endings).
#[derive(Clone, Debug, Default)]
pub struct Output {
    files: BTreeMap<String, String>,
}

pub const MANIFEST: &str = ".bt2plc-manifest";

impl Output {
    pub fn add(&mut self, rel: impl Into<String>, text: impl Into<String>) -> Result<()> {
        let rel = rel.into();
        if !safe_rel(&rel) {
            return Err(Error::Config(format!(
                "refusing to write outside the output directory: {rel}"
            )));
        }
        if self.files.contains_key(&rel) {
            return Err(Error::Config(format!("output file {rel} produced twice")));
        }
        self.files.insert(rel, text.into());
        Ok(())
    }

    pub fn files(&self) -> &BTreeMap<String, String> {
        &self.files
    }

    pub fn get(&self, rel: &str) -> Option<&str> {
        self.files.get(rel).map(String::as_str)
    }

    /// Write all files. A non-empty directory needs `force`; files listed in the previous
    /// manifest that are no longer produced are removed.
    pub fn write_to(&self, dir: &Path, force: bool) -> Result<()> {
        if dir.exists() {
            let non_empty = std::fs::read_dir(dir)
                .map_err(|e| Error::io(dir, e))?
                .next()
                .is_some();
            if non_empty && !force {
                return Err(Error::io(
                    dir,
                    std::io::Error::other(
                        "output directory is not empty (use --force to overwrite)",
                    ),
                ));
            }
            if let Ok(old) = std::fs::read_to_string(dir.join(MANIFEST)) {
                for rel in old.lines().filter(|l| !l.is_empty()) {
                    if !self.files.contains_key(rel) && safe_rel(rel) {
                        let _ = std::fs::remove_file(dir.join(rel));
                    }
                }
            }
        }
        for (rel, text) in &self.files {
            let p = dir.join(rel);
            if let Some(parent) = p.parent() {
                std::fs::create_dir_all(parent).map_err(|e| Error::io(parent, e))?;
            }
            std::fs::write(&p, text).map_err(|e| Error::io(&p, e))?;
        }
        let manifest: String = self.files.keys().map(|k| format!("{k}\n")).collect();
        let tmp = dir.join(format!("{MANIFEST}.tmp"));
        std::fs::write(&tmp, manifest).map_err(|e| Error::io(&tmp, e))?;
        std::fs::rename(&tmp, dir.join(MANIFEST)).map_err(|e| Error::io(dir, e))?;
        Ok(())
    }
}

fn safe_rel(rel: &str) -> bool {
    Path::new(rel)
        .components()
        .all(|c| matches!(c, Component::Normal(_)))
}
