//! `migrate-to-s3`: the first time a workspace runs with `--storage s3`, upload what it already has
//! into its subtree `users/<owner>/…`, so switching doesn't leave existing work behind on the
//! instance's disk: the same [`super::USER_PREFIXES`] a new instance is filled from. Products and
//! cells whole; of the catalogue what is the user's own ([`users_own`]): dressed robots, published
//! products, and every file that the release's starter catalogue doesn't have as it is (Robo Fitter
//! scenes, a tool whose TCP was edited, uploads). Keys already in S3 are skipped (never
//! overwritten). Run by the backend at startup until it has once completed (a marker next to the
//! config file, naming [`VERSION`]: a workspace migrated by an older one, which left the scenes and
//! tools out, runs it once more), and by hand as `factoryos-backend migrate-to-s3`
//! (`factoryos-workspace migrate-to-s3`).

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use factoryos_protocol::storage::{Scope, StorageMode};

use super::{MAX_SYNC_BYTES, ObjectKey, Storage, USER_PREFIXES, walk_files};

/// What the marker of a migration of this version says (and what [`migrate_once`] looks for).
pub const VERSION: &str = "migration 2";
/// The user's own catalogue folders: always uploaded (the starter catalogue may ship some too).
const OWN_CATALOGUE_FOLDERS: [&str; 2] = ["urdf-dressed-robots/", "product-collection/"];

/// The marker of a completed migration, in `dir` (the config file's folder).
pub fn marker(dir: &Path) -> PathBuf {
    dir.join(".migrated-to-s3")
}

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Summary {
    pub uploaded: usize,
    pub skipped: usize,
    pub too_large: usize,
    pub failed: usize,
}

impl std::fmt::Display for Summary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} uploaded, {} already in S3, {} too large, {} failed", self.uploaded, self.skipped, self.too_large, self.failed)
    }
}

/// The user's folders to migrate: those of [`USER_PREFIXES`] the working copy has.
fn sources(storage: &Storage) -> Vec<PathBuf> {
    let layout = storage.layout();
    USER_PREFIXES
        .iter()
        .filter_map(|prefix| layout.user.iter().find(|(p, _)| format!("{p}/") == *prefix).map(|(_, dir)| dir.clone()))
        .filter(|dir| dir.is_dir())
        .collect()
}

/// Whether the working-copy file of `key` is the user's to upload: everything but the catalogue's
/// files that are the shared catalogue's (`shared`, its keys: the sync brought them) or the
/// release's starter catalogue's (`release`, when that is another folder) as it is.
pub fn users_own(key: &ObjectKey, file: &Path, shared: &HashSet<String>, release: Option<&Path>) -> bool {
    let Some(rel) = key.key().strip_prefix("catalogue/") else { return true };
    if OWN_CATALOGUE_FOLDERS.iter().any(|own| rel.starts_with(own)) {
        return true;
    }
    if shared.contains(rel) {
        return false;
    }
    let Some(release) = release else { return false };
    let shipped = release.join(rel);
    match (std::fs::metadata(&shipped), std::fs::metadata(file)) {
        (Ok(a), Ok(b)) if a.len() == b.len() => std::fs::read(&shipped).ok() != std::fs::read(file).ok(),
        (Ok(_), Ok(_)) => true,
        // Not in the release: the user's (or the shared catalogue's, which the sync brings anyway).
        (Err(_), _) => true,
        (Ok(_), Err(_)) => false,
    }
}

/// The release's catalogue, when the working copy's catalogue is another folder (else nothing of it
/// tells the user's files from the starter's, and only [`OWN_CATALOGUE_FOLDERS`] are uploaded).
fn release_catalogue(storage: &Storage) -> Option<PathBuf> {
    let release = std::fs::canonicalize(crate::paths::default_catalogue()).ok()?;
    let ours = storage.layout().user.iter().find(|(p, _)| p == "catalogue").and_then(|(_, dir)| std::fs::canonicalize(dir).ok());
    (ours.as_ref() != Some(&release)).then_some(release)
}

/// Upload what isn't in S3 yet. Locally: nothing to do.
pub async fn migrate(storage: &Storage) -> anyhow::Result<Summary> {
    migrate_with(storage, release_catalogue(storage).as_deref()).await
}

/// [`migrate`], the starter catalogue being `release`.
pub async fn migrate_with(storage: &Storage, release: Option<&Path>) -> anyhow::Result<Summary> {
    let mut summary = Summary::default();
    if storage.mode() != StorageMode::S3 {
        return Ok(summary);
    }
    let shared: HashSet<String> = storage.store().list(Scope::Catalogue, "").await?.objects.into_iter().map(|o| o.key).collect();
    for dir in sources(storage) {
        let files = tokio::task::spawn_blocking(move || walk_files(&dir)).await?;
        for file in files {
            let Some(key) = storage.key_of(&file) else { continue };
            if !users_own(&key, &file, &shared, release) {
                continue;
            }
            let Ok(meta) = tokio::fs::metadata(&file).await else { continue };
            if meta.len() > MAX_SYNC_BYTES {
                summary.too_large += 1;
                continue;
            }
            match storage.store().head(&key).await {
                Ok(Some(_)) => summary.skipped += 1,
                Ok(None) => match tokio::fs::read(&file).await {
                    Ok(bytes) => match storage.store().put(&key, bytes).await {
                        Ok(()) => summary.uploaded += 1,
                        Err(err) => {
                            tracing::warn!("migrate-to-s3: {}: {err:#}", key.key());
                            summary.failed += 1;
                        }
                    },
                    Err(err) => {
                        tracing::warn!("migrate-to-s3: {}: {err}", file.display());
                        summary.failed += 1;
                    }
                },
                Err(err) => {
                    tracing::warn!("migrate-to-s3: {}: {err:#}", key.key());
                    summary.failed += 1;
                }
            }
        }
    }
    Ok(summary)
}

/// [`migrate`] unless this [`VERSION`] of it was done before (the marker in `dir`); the marker is
/// written once nothing failed.
pub async fn migrate_once(storage: &Storage, dir: &Path) -> anyhow::Result<Option<Summary>> {
    migrate_once_with(storage, dir, release_catalogue(storage).as_deref()).await
}

async fn migrate_once_with(storage: &Storage, dir: &Path, release: Option<&Path>) -> anyhow::Result<Option<Summary>> {
    if storage.mode() != StorageMode::S3 || done(dir) {
        return Ok(None);
    }
    let summary = migrate_with(storage, release).await?;
    if summary.failed == 0 {
        write_marker(dir, &summary)?;
    }
    Ok(Some(summary))
}

/// Whether the marker in `dir` is of this [`VERSION`].
fn done(dir: &Path) -> bool {
    std::fs::read_to_string(marker(dir)).is_ok_and(|text| text.contains(VERSION))
}

/// The marker of a completed migration: its summary and [`VERSION`].
pub fn write_marker(dir: &Path, summary: &Summary) -> anyhow::Result<()> {
    super::local::replace_file(&marker(dir), format!("{summary} ({VERSION})\n").as_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::sync::tests_support::FakeS3;
    use crate::storage::{ObjectKey, Store, UserId, local};
    use std::sync::Arc;

    #[tokio::test]
    async fn uploads_what_is_missing_once() {
        let root = std::env::temp_dir().join(format!("factoryos-migrate-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        for dir in ["products/box", "cells/c1", "catalogue/urdf-dressed-robots/r__t", "catalogue/urdf-robot-collection"] {
            std::fs::create_dir_all(root.join(dir)).unwrap();
        }
        let root = std::fs::canonicalize(&root).unwrap();
        std::fs::write(root.join("products/box/product.json"), "{}").unwrap();
        std::fs::write(root.join("cells/c1/scene.yaml"), "s").unwrap();
        std::fs::write(root.join("catalogue/urdf-dressed-robots/r__t/r__t.urdf"), "d").unwrap();
        std::fs::write(root.join("catalogue/urdf-robot-collection/starter.urdf"), "starter").unwrap();
        let s3 = Arc::new(FakeS3::default());
        // Already there (e.g. from another instance): kept as it is.
        s3.set(Scope::User, "products/box/product.json", b"newer");
        let layout = {
            let root = root.clone();
            move || local::Layout {
                user: vec![("products".into(), root.join("products")), ("cells".into(), root.join("cells")), ("catalogue".into(), root.join("catalogue"))],
                catalogue: Some(root.join("catalogue")),
            }
        };
        let storage = Storage::with_store(s3.clone(), UserId::new("jdoe").unwrap(), layout);
        let summary = migrate_once_with(&storage, &root, None).await.unwrap().unwrap();
        assert_eq!(summary, Summary { uploaded: 2, skipped: 1, too_large: 0, failed: 0 });
        let get = |key: &str| futures::executor::block_on(s3.get(&ObjectKey::new(Scope::User, key).unwrap())).unwrap();
        assert_eq!(get("products/box/product.json").as_deref(), Some(b"newer".as_slice()));
        assert_eq!(get("cells/c1/scene.yaml").as_deref(), Some(b"s".as_slice()));
        assert_eq!(get("catalogue/urdf-dressed-robots/r__t/r__t.urdf").as_deref(), Some(b"d".as_slice()));
        // The starter catalogue is not the user's.
        assert_eq!(get("catalogue/urdf-robot-collection/starter.urdf"), None);
        assert!(migrate_once_with(&storage, &root, None).await.unwrap().is_none(), "once");
        // A marker of the older migration (no version): it runs once more.
        std::fs::write(marker(&root), "2 uploaded, 1 already in S3, 0 too large, 0 failed\n").unwrap();
        assert_eq!(migrate_once_with(&storage, &root, None).await.unwrap().unwrap(), Summary { uploaded: 0, skipped: 3, too_large: 0, failed: 0 });
        std::fs::remove_dir_all(&root).ok();
    }

    /// Everything the user made on one instance, migrated and written through, comes back on a
    /// fresh one (the startup fill of [`USER_PREFIXES`]); the starter's and the shared catalogue's
    /// files are not the user's.
    #[tokio::test]
    async fn a_fresh_instance_gets_everything_back() {
        use crate::storage::sync::tests_support::{eventually, storage_over, user_storage};
        let (a, s3, storage, registered) = user_storage("fresh-a");
        let release = a.join("release");
        let write = |path: &Path, text: &str| {
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, text).unwrap();
        };
        // Before S3: a product, a cell (its manifest, a plan, the hardware, a build), a dressed robot, a
        // Robo Fitter scene and an edited tool; the starter's tool as shipped; a shared file synced in.
        write(&a.join("products/box/product.json"), "{\"name\":\"box\"}");
        write(&a.join("cells/weld/factoryos-cell.json"), "{\"format\":2}");
        write(&a.join("cells/weld/agentic_planner/plans/bt_1.json"), "{}");
        write(&a.join("cells/weld/plan2cell/hardware.json"), "{\"schema\":\"factoryos/hardware@1\"}");
        write(&a.join("cells/weld/plan2cell/builds/bt_1/20261101T1Z/build.json"), "{}");
        write(&a.join("cells/weld/.view/scene.yaml"), "a view");
        write(&a.join("catalogue/urdf-dressed-robots/kr__gun/kr__gun.urdf"), "<robot name='kr__gun'/>");
        write(&a.join("catalogue/scenes/weld/scene.yaml"), "name: weld");
        for (path, text) in [("urdf-tool-collection/gun/gun.urdf", "<robot name='gun'/>"), ("urdf-tool-collection/stick/stick.urdf", "<robot name='stick'/>")] {
            write(&release.join(path), text);
            write(&a.join("catalogue").join(path), text);
        }
        write(&a.join("catalogue/urdf-tool-collection/gun/gun.urdf"), "<robot name='gun' tcp='edited'/>");
        write(&a.join("catalogue/shared/s.urdf"), "shared");
        s3.set(Scope::Catalogue, "shared/s.urdf", b"shared");
        let summary = migrate_once_with(&storage, &a, Some(&release)).await.unwrap().unwrap();
        assert_eq!(summary.failed, 0);
        // In S3 mode: the planner saves a plan (the watch), the user saves a task program (a service).
        write(&a.join("cells/weld/agentic_planner/plans/bt_2.json"), "{\"v\":2}");
        super::super::watch::Watch::new(storage.clone()).scan(super::super::watch::cell_outputs(&a.join("cells"))).await;
        write(&a.join("cells/weld/agentic_planner/tasks/t.json"), "{\"operations\":[]}");
        crate::storage::saved(a.join("cells/weld/agentic_planner/tasks/t.json"));
        eventually("the task program", || s3.bytes(Scope::User, "cells/weld/agentic_planner/tasks/t.json").is_some()).await;
        drop(registered);

        // A fresh instance: empty folders, the same bucket.
        let b = a.join("fresh");
        for dir in ["products", "cells", "catalogue"] {
            std::fs::create_dir_all(b.join(dir)).unwrap();
        }
        let fresh = storage_over(&b, s3.clone());
        for prefix in USER_PREFIXES {
            fresh.hydrate(Scope::User, prefix).await.unwrap();
        }
        for file in [
            "products/box/product.json",
            "cells/weld/factoryos-cell.json",
            "cells/weld/agentic_planner/plans/bt_1.json",
            "cells/weld/agentic_planner/plans/bt_2.json",
            "cells/weld/agentic_planner/tasks/t.json",
            "cells/weld/plan2cell/hardware.json",
            "cells/weld/plan2cell/builds/bt_1/20261101T1Z/build.json",
            "catalogue/urdf-dressed-robots/kr__gun/kr__gun.urdf",
            "catalogue/scenes/weld/scene.yaml",
            "catalogue/urdf-tool-collection/gun/gun.urdf",
        ] {
            assert_eq!(std::fs::read(b.join(file)).ok(), Some(std::fs::read(a.join(file)).unwrap()), "{file}");
        }
        // Not the user's: the starter's tool as shipped, the shared catalogue's file, the view.
        for file in ["catalogue/urdf-tool-collection/stick/stick.urdf", "catalogue/shared/s.urdf", "cells/weld/.view/scene.yaml"] {
            assert!(!b.join(file).exists(), "{file}");
        }
        std::fs::remove_dir_all(&a).ok();
    }
}
