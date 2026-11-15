//! Files of the user's that other processes write: the planner's plans (`kiro-cli` drives the
//! umarell planner, which saves `cells/<cell>/agentic_planner/plans/*.json` itself) and the Virtual
//! Cell's auto-restart setting (`cells/<cell>/plan2cell/auto-restart.json`). No hook runs after
//! those writes, so [`Watch`] looks at them every [`WATCH_EVERY`] (S3 mode) and writes a new or
//! changed file through, and removes a file it saw that is gone.
//!
//! The first look at a file compares it with the store (a `HEAD`): what a new instance got from the
//! store, or what was uploaded before the restart, isn't uploaded again.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use factoryos_protocol::storage::StorageMode;

use super::{MAX_SYNC_BYTES, Storage, walk_files};

/// How often [`Watch::spawn`] looks.
pub const WATCH_EVERY: Duration = Duration::from_secs(15);

/// The planner's plans and the auto-restart setting of every cell in `cells_dir`.
pub fn cell_outputs(cells_dir: &Path) -> Vec<PathBuf> {
    crate::services::cells::list_cells(cells_dir)
        .into_iter()
        .flat_map(|cell| {
            let root = cells_dir.join(cell);
            [root.join(crate::services::cells::PLANS_DIR), root.join(crate::services::infra::AUTO_RESTART_FILE)]
        })
        .collect()
}

/// A file's size and modification time: when they change, so did the file.
type Stamp = (u64, Option<SystemTime>);

pub struct Watch {
    storage: Arc<Storage>,
    /// The files written through (or found in the store as they are), as they were then.
    seen: Mutex<HashMap<PathBuf, Stamp>>,
}

impl Watch {
    pub fn new(storage: Arc<Storage>) -> Arc<Self> {
        Arc::new(Self { storage, seen: Mutex::default() })
    }

    /// Look at `paths` (folders: every file under them; files) once: write through what is new or
    /// changed, remove what was seen and is gone. Returns how many objects it wrote or removed.
    pub async fn scan(&self, paths: Vec<PathBuf>) -> usize {
        if self.storage.mode() != StorageMode::S3 {
            return 0;
        }
        let files = tokio::task::spawn_blocking(move || {
            let mut files: Vec<(PathBuf, Stamp)> = Vec::new();
            for path in paths {
                let found = match path.symlink_metadata() {
                    Ok(meta) if meta.is_dir() => walk_files(&path),
                    Ok(meta) if meta.is_file() => vec![path],
                    _ => Vec::new(),
                };
                files.extend(found.into_iter().filter_map(|file| {
                    let meta = file.symlink_metadata().ok()?;
                    (meta.len() <= MAX_SYNC_BYTES).then(|| (file, (meta.len(), meta.modified().ok())))
                }));
            }
            files
        })
        .await
        .unwrap_or_default();
        let mut changed = 0;
        let mut now: HashMap<PathBuf, Stamp> = HashMap::new();
        for (file, stamp) in files {
            let before = self.seen.lock().unwrap().get(&file).copied();
            match before {
                Some(before) if before == stamp => {}
                Some(_) => {
                    self.storage.save_path(&file).await;
                    changed += 1;
                }
                None => {
                    // First seen: in the store as it is already (filled in, uploaded before)?
                    let Some(key) = self.storage.key_of(&file) else { continue };
                    let stored = self.storage.store().head(&key).await;
                    if !matches!(&stored, Ok(Some(info)) if info.bytes == stamp.0) {
                        self.storage.save_path(&file).await;
                        changed += 1;
                    }
                }
            }
            now.insert(file, stamp);
        }
        // Seen before, not now: removed (unless it is only not looked at any more and still there).
        let gone: Vec<PathBuf> = {
            let mut seen = self.seen.lock().unwrap();
            let gone = seen.keys().filter(|path| !now.contains_key(*path)).cloned().collect();
            *seen = now;
            gone
        };
        for path in gone {
            if path.symlink_metadata().is_err_and(|err| err.kind() == std::io::ErrorKind::NotFound) {
                self.storage.save_path(&path).await;
                changed += 1;
            }
        }
        changed
    }

    /// [`Self::scan`] of `paths()` every [`WATCH_EVERY`], from now on (S3 mode).
    pub fn spawn(self: &Arc<Self>, paths: impl Fn() -> Vec<PathBuf> + Send + Sync + 'static) {
        if self.storage.mode() != StorageMode::S3 {
            return;
        }
        let this = self.clone();
        tokio::spawn(async move {
            loop {
                let changed = this.scan(paths()).await;
                if changed > 0 {
                    tracing::info!("Storage: {changed} planner/cell file(s) written through");
                }
                tokio::time::sleep(WATCH_EVERY).await;
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::sync::tests_support::{FakeS3, user_storage};
    use factoryos_protocol::storage::Scope;

    #[tokio::test]
    async fn plans_written_by_the_planner_are_written_through() {
        let (root, s3, storage, _registered) = user_storage("watch");
        let plans = root.join("cells/weld/agentic_planner/plans");
        std::fs::create_dir_all(&plans).unwrap();
        let auto = root.join("cells/weld/plan2cell/auto-restart.json");
        std::fs::create_dir_all(auto.parent().unwrap()).unwrap();
        std::fs::write(&auto, "{\"enabled\":true}").unwrap();
        // Already in the store as it is (a new instance filled it in): not uploaded again.
        std::fs::write(plans.join("bt_old.json"), "{}").unwrap();
        s3.set(Scope::User, "cells/weld/agentic_planner/plans/bt_old.json", b"{}");
        std::fs::write(plans.join("bt_1.json"), "{\"tree_id\":1}").unwrap();
        let watch = Watch::new(storage);
        let paths = || cell_outputs(&root.join("cells"));
        assert_eq!(watch.scan(paths()).await, 2, "bt_1 and the auto-restart setting");
        let get = |key: &str| FakeS3::bytes(&s3, Scope::User, key);
        assert_eq!(get("cells/weld/agentic_planner/plans/bt_1.json").as_deref(), Some(b"{\"tree_id\":1}".as_slice()));
        assert_eq!(get("cells/weld/plan2cell/auto-restart.json").as_deref(), Some(b"{\"enabled\":true}".as_slice()));
        assert_eq!(watch.scan(paths()).await, 0, "nothing changed");
        // The planner rewrites one and saves another; then one is deleted.
        std::fs::write(plans.join("bt_1.json"), "{\"tree_id\":1,\"v\":2}").unwrap();
        std::fs::write(plans.join("bt_2.json"), "{}").unwrap();
        assert_eq!(watch.scan(paths()).await, 2);
        assert_eq!(get("cells/weld/agentic_planner/plans/bt_1.json").as_deref(), Some(b"{\"tree_id\":1,\"v\":2}".as_slice()));
        assert!(get("cells/weld/agentic_planner/plans/bt_2.json").is_some());
        std::fs::remove_file(plans.join("bt_2.json")).unwrap();
        assert_eq!(watch.scan(paths()).await, 1);
        assert!(get("cells/weld/agentic_planner/plans/bt_2.json").is_none());
        std::fs::remove_dir_all(&root).ok();
    }
}
