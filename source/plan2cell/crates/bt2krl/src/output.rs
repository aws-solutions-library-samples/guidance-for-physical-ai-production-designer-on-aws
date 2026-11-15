//! Deterministic output file set and the atomic writer (as `bt2plc::backend::Output`).

use std::collections::BTreeMap;
use std::path::{Component, Path};

use crate::error::{Error, Result};

/// Files written by the last run (relative paths), for cleanup with `--force`.
pub const MANIFEST: &str = ".bt2krl-files";

#[derive(Clone, Debug, Default)]
pub struct Output {
    files: BTreeMap<String, String>,
}

pub fn safe_rel(rel: &str) -> bool {
    !rel.is_empty()
        && Path::new(rel)
            .components()
            .all(|c| matches!(c, Component::Normal(_)))
}

impl Output {
    pub fn add(&mut self, rel: impl Into<String>, text: impl Into<String>) -> Result<()> {
        let rel = rel.into();
        if !safe_rel(&rel) {
            return Err(Error::Config(format!(
                "refusing to write outside the output directory: {rel}"
            )));
        }
        let key_ci = rel.to_ascii_lowercase();
        if self.files.keys().any(|k| k.to_ascii_lowercase() == key_ci) {
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

    /// Write every file atomically (temp file + rename). A non-empty directory needs `force`;
    /// files of the previous run that are no longer produced are removed.
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
                        // Also the directories that became empty (never `dir` itself).
                        let mut p = dir.join(rel);
                        while let Some(parent) = p.parent() {
                            if parent == dir || std::fs::remove_dir(parent).is_err() {
                                break;
                            }
                            p = parent.to_path_buf();
                        }
                    }
                }
            }
        }
        for (rel, text) in &self.files {
            write_atomic(&dir.join(rel), text.as_bytes())?;
        }
        let manifest: String = self.files.keys().map(|k| format!("{k}\n")).collect();
        write_atomic(&dir.join(MANIFEST), manifest.as_bytes())
    }
}

pub fn write_atomic(path: &Path, bytes: &[u8]) -> Result<()> {
    let parent = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| Error::io(parent, e))?;
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let tmp = parent.join(format!(".{name}.{}.tmp", std::process::id()));
    std::fs::write(&tmp, bytes).map_err(|e| Error::io(&tmp, e))?;
    std::fs::rename(&tmp, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        Error::io(path, e)
    })
}
