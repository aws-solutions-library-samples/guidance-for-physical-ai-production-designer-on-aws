//! The catalogue's scenes, migrated to their current layout at every start, before the services
//! read them (and again after the user's files came down from the store, S3 mode):
//!
//! * [`migrate_local`], first the old folder: each file of `<assetRoot>/fitted-robots/` moves to
//!   `<assetRoot>/scenes/`. A file already there with the same bytes is dropped from the old
//!   folder; one with other bytes is kept beside it as `<stem>-from-fitted-robots.yaml` (nothing is
//!   overwritten). The old folder goes when it is empty. A config whose `fittedRobotsDir` names the
//!   old folder is set back to the default ([`migrate_config`]).
//! * then the folder layout ([`migrate_flat`]): every flat scene `scenes/<stem>.yaml`/`.yml` becomes
//!   the folder `scenes/<stem>/scene.yaml` (see [`super::scene_store`]): its relative asset paths
//!   (relative to `scenes/`) get `../` in front (they start at the new folder), absolute ones stay,
//!   `name:` is set to the folder name, and its USDA bundles `<stem>.zip`/`<stem>-urdf.zip` move
//!   along. Nothing is overwritten: when `scenes/<stem>/` is taken by another scene, the next free
//!   `<stem>-2`, … is used (reported); a flat file whose converted scene is already there (the same
//!   scene, e.g. one the shared catalogue's sync brought again) is dropped. Cells whose
//!   `factoryos-cell.json` names the flat file (`fitted_scene: "x.yaml"`) are updated to the
//!   scene's name (`"x"`, or the clash's new name). Every write and removal goes through to the
//!   user's store ([`crate::storage::saved`], [`crate::storage::removed`]). Idempotent.
//! * [`migrate_store`] (S3 mode, after the user's files came down): the user's keys under
//!   `catalogue/fitted-robots/` are copied to `catalogue/scenes/` (unless one is there) and deleted;
//!   the local migration that runs after it turns those flat files into folders. Keys of the shared
//!   catalogue are not rewritten (only catalogue admins may): the storage layer reads
//!   `fitted-robots/` keys as the scenes folder's ([`crate::storage::local::catalogue_rel`]), they land
//!   as flat files, which the scenes' path resolution tolerates (the legacy fallback of
//!   [`super::scene_store::asset_path`]) and the next start migrates.

use std::path::{Path, PathBuf};

use std::collections::BTreeMap;

use factoryos_protocol::fitter::{LEGACY_SCENES_DIR, SCENES_DIR, SceneFile};
use factoryos_protocol::storage::Scope;

use crate::config_store::ConfigStore;
use crate::storage::{ObjectKey, Storage};

/// What a migration did, for the log.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Migrated {
    pub moved: Vec<String>,
    pub duplicates: Vec<String>,
    /// `(old name, the name it was kept under)`.
    pub conflicts: Vec<(String, String)>,
    /// Flat scene files made folders: `(file, scene name)`.
    pub folders: Vec<(String, String)>,
    /// Of those, the ones whose folder name was taken: `(file, the name it got instead)`.
    pub renamed: Vec<(String, String)>,
    /// Flat scene files dropped because their scene's folder has it already.
    pub flat_duplicates: Vec<String>,
    /// Cells whose manifest now names the scene by its name.
    pub cells: Vec<String>,
}

impl Migrated {
    pub fn is_empty(&self) -> bool {
        self.moved.is_empty()
            && self.duplicates.is_empty()
            && self.conflicts.is_empty()
            && self.folders.is_empty()
            && self.flat_duplicates.is_empty()
            && self.cells.is_empty()
    }
}

impl std::fmt::Display for Migrated {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} moved to {SCENES_DIR}/", self.moved.len())?;
        if !self.duplicates.is_empty() {
            write!(f, ", {} already there", self.duplicates.len())?;
        }
        for (old, kept) in &self.conflicts {
            write!(f, ", {old} differs from {SCENES_DIR}/{old}: kept as {kept}")?;
        }
        write!(f, "; {} flat scene file(s) now folders {SCENES_DIR}/<name>/scene.yaml", self.folders.len())?;
        for (file, name) in &self.renamed {
            write!(f, ", {file} is scene {name} now (its own name, or the next free one)")?;
        }
        if !self.flat_duplicates.is_empty() {
            write!(f, ", {} flat file(s) already there as folders", self.flat_duplicates.len())?;
        }
        if !self.cells.is_empty() {
            write!(f, "; cells naming their scene by name now: {}", self.cells.join(", "))?;
        }
        Ok(())
    }
}

/// [`migrate_local`] for the config's asset root (and a configured `fittedRobotsDir` elsewhere),
/// logged.
pub fn run_local(config: &ConfigStore) {
    let asset_root = PathBuf::from(config.resolve(&config.get().asset_root));
    let cells = super::cells::cells_folder(config);
    match migrate_local(&asset_root, cells.as_deref()) {
        Ok(done) if done.is_empty() => {}
        Ok(done) => tracing::info!("Scenes: {}: {done}", asset_root.display()),
        Err(err) => tracing::warn!("Scenes: migrating {}: {err:#}", asset_root.display()),
    }
    let fitted = super::cells::fitted_scenes_dir(config);
    if fitted != asset_root.join(SCENES_DIR) {
        let mut done = Migrated::default();
        match migrate_flat(&fitted, cells.as_deref(), &mut done) {
            Ok(()) if done.is_empty() => {}
            Ok(()) => tracing::info!("Scenes: {}: {done}", fitted.display()),
            Err(err) => tracing::warn!("Scenes: migrating {}: {err:#}", fitted.display()),
        }
    }
}

/// Move the files of `<asset_root>/fitted-robots/` into `<asset_root>/scenes/`, then make every
/// flat scene file there a scene folder ([`migrate_flat`]), updating the cells of `cells_dir`.
pub fn migrate_local(asset_root: &Path, cells_dir: Option<&Path>) -> std::io::Result<Migrated> {
    let mut done = migrate_legacy_folder(asset_root)?;
    migrate_flat(&asset_root.join(SCENES_DIR), cells_dir, &mut done)?;
    Ok(done)
}

/// Move the files of `<asset_root>/fitted-robots/` into `<asset_root>/scenes/`.
fn migrate_legacy_folder(asset_root: &Path) -> std::io::Result<Migrated> {
    let (old, new) = (asset_root.join(LEGACY_SCENES_DIR), asset_root.join(SCENES_DIR));
    let mut done = Migrated::default();
    let entries = match std::fs::read_dir(&old) {
        Ok(entries) => entries,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => return Ok(done),
        Err(err) => return Err(err),
    };
    std::fs::create_dir_all(&new)?;
    let mut files: Vec<PathBuf> = entries.flatten().map(|e| e.path()).filter(|p| p.is_file()).collect();
    files.sort();
    for file in files {
        let name = file.file_name().unwrap_or_default().to_string_lossy().into_owned();
        let target = new.join(&name);
        if !target.exists() {
            move_file(&file, &target)?;
            crate::storage::saved(&target);
            crate::storage::removed(&file);
            done.moved.push(name);
        } else if std::fs::read(&target)? == std::fs::read(&file)? {
            std::fs::remove_file(&file)?;
            crate::storage::removed(&file);
            done.duplicates.push(name);
        } else {
            let (stem, ext) = match name.rsplit_once('.') {
                Some((stem, ext)) => (stem.to_owned(), format!(".{ext}")),
                None => (name.clone(), String::new()),
            };
            let kept = (1..)
                .map(|i| if i == 1 { format!("{stem}-from-fitted-robots{ext}") } else { format!("{stem}-from-fitted-robots-{i}{ext}") })
                .find(|n| !new.join(n).exists())
                .expect("unbounded");
            move_file(&file, &new.join(&kept))?;
            crate::storage::saved(new.join(&kept));
            crate::storage::removed(&file);
            done.conflicts.push((name, kept));
        }
    }
    // Only an empty folder goes (a sub-folder someone made stays, with the folder).
    let _ = std::fs::remove_dir(&old);
    Ok(done)
}

/// Make every flat scene file `<scenes_dir>/<stem>.yaml`/`.yml` the folder
/// `<scenes_dir>/<name>/scene.yaml` (see the module docs), and the cells in `cells_dir` that name
/// one name the scene.
pub fn migrate_flat(scenes_dir: &Path, cells_dir: Option<&Path>, done: &mut Migrated) -> std::io::Result<()> {
    let mut files: Vec<PathBuf> = match std::fs::read_dir(scenes_dir) {
        Ok(entries) => entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.is_file() && p.extension().is_some_and(|e| e == "yaml" || e == "yml"))
            .collect(),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => Vec::new(),
        Err(err) => return Err(err),
    };
    files.sort();
    // Flat file name → the scene it is now.
    let mut renames: BTreeMap<String, String> = BTreeMap::new();
    for file in files {
        let file_name = file.file_name().unwrap_or_default().to_string_lossy().into_owned();
        let stem = file.file_stem().unwrap_or_default().to_string_lossy().into_owned();
        let Some(base) = super::scene_store::sanitize(&stem) else {
            tracing::warn!("Scenes: {}: no scene name can be made of it, left as it is", file.display());
            continue;
        };
        let text = std::fs::read_to_string(&file)?;
        let parsed = super::scene_paths::parse_scene_file(&text).ok();
        // The scene's own `name:` names its folder when it differs from the file name (the Robo
        // Fitter kept `new-scene.yaml` for a scene saved as `bw48-1`), unless another scene (a
        // folder, or a flat file still to migrate) already has that name.
        let base = match parsed.as_ref().and_then(|s| super::scene_store::sanitize(&s.name)) {
            Some(own)
                if own != base
                    && !super::scene_store::scene_dir(scenes_dir, &own).exists()
                    && !scenes_dir.join(format!("{own}.yaml")).exists()
                    && !scenes_dir.join(format!("{own}.yml")).exists()
                    && !renames.values().any(|n| *n == own) =>
            {
                own
            }
            _ => base,
        };
        let convert = |name: &str| -> std::io::Result<(Option<SceneFile>, String)> {
            match &parsed {
                Some(scene) => {
                    let mut scene = scene.clone();
                    rebase_flat_paths(&mut scene);
                    scene.name = name.to_owned();
                    let text = serde_yaml::to_string(&scene).map_err(std::io::Error::other)?;
                    Ok((Some(scene), text))
                }
                // Not a scene the model reads: moved as it is (its paths resolve through the
                // legacy fallback).
                None => Ok((None, text.clone())),
            }
        };
        // The same scene in a folder already (its own name, or a clash's `-N` from an earlier run).
        let same = (1..)
            .map(|i| if i == 1 { base.clone() } else { format!("{base}-{i}") })
            .take_while(|name| super::scene_store::scene_dir(scenes_dir, name).exists())
            .find(|name| {
                let existing = super::scene_store::scene_file(scenes_dir, name);
                let Ok(existing_text) = std::fs::read_to_string(&existing) else { return false };
                match convert(name) {
                    Ok((Some(scene), _)) => super::scene_paths::parse_scene_file(&existing_text).is_ok_and(|e| e == scene),
                    Ok((None, converted)) => converted == existing_text,
                    Err(_) => false,
                }
            });
        if let Some(name) = same {
            std::fs::remove_file(&file)?;
            crate::storage::removed(&file);
            move_bundles(scenes_dir, &stem, &name)?;
            done.flat_duplicates.push(file_name.clone());
            renames.insert(file_name, name);
            continue;
        }
        let name = (1..)
            .map(|i| if i == 1 { base.clone() } else { format!("{base}-{i}") })
            .find(|name| {
                let dir = super::scene_store::scene_dir(scenes_dir, name);
                !dir.exists() || std::fs::read_dir(&dir).is_ok_and(|mut e| e.next().is_none())
            })
            .expect("unbounded");
        let (_, converted) = convert(&name)?;
        let target = super::scene_store::scene_file(scenes_dir, &name);
        std::fs::create_dir_all(target.parent().expect("in a folder"))?;
        std::fs::write(&target, converted)?;
        crate::storage::saved(&target);
        std::fs::remove_file(&file)?;
        crate::storage::removed(&file);
        move_bundles(scenes_dir, &stem, &name)?;
        if name != stem {
            done.renamed.push((file_name.clone(), name.clone()));
        }
        done.folders.push((file_name.clone(), name.clone()));
        renames.insert(file_name, name);
    }
    if let Some(cells_dir) = cells_dir {
        update_cells(cells_dir, &renames, done);
    }
    Ok(())
}

/// A flat scene's relative paths (from `scenes/`) from its new folder (`scenes/<name>/`).
fn rebase_flat_paths(scene: &mut SceneFile) {
    let rebase = |value: &mut String| {
        let rel = value.trim();
        if !rel.is_empty() && !Path::new(rel).is_absolute() {
            *value = format!("../{rel}");
        }
    };
    scene.robots.iter_mut().for_each(|r| rebase(&mut r.dressed_urdf));
    scene.fixtures.iter_mut().for_each(|f| rebase(&mut f.urdf));
    if let Some(plan) = scene.floor_plan.as_mut() {
        rebase(&mut plan.geometry_file);
    }
}

/// The USDA bundles of flat scene `stem` (`<stem>.zip`, `<stem>-urdf.zip` beside it) into scene
/// `name`'s folder, as `<name>.zip`, `<name>-urdf.zip` (never overwriting).
fn move_bundles(scenes_dir: &Path, stem: &str, name: &str) -> std::io::Result<()> {
    for (from, to) in [(format!("{stem}.zip"), format!("{name}.zip")), (format!("{stem}-urdf.zip"), format!("{name}-urdf.zip"))] {
        let (from, to) = (scenes_dir.join(from), super::scene_store::scene_dir(scenes_dir, name).join(to));
        if from.is_file() && !to.exists() {
            move_file(&from, &to)?;
            crate::storage::saved(&to);
            crate::storage::removed(&from);
        }
    }
    Ok(())
}

/// The cells in `cells_dir` whose manifest names a scene by a legacy file name (`x.yaml`): the
/// scene's name now (`renames`' for the files just migrated, else the file's stem).
fn update_cells(cells_dir: &Path, renames: &BTreeMap<String, String>, done: &mut Migrated) {
    for cell in super::cells::list_cells(cells_dir) {
        let root = cells_dir.join(&cell);
        let Some(mut manifest) = super::cell_resolve::read_manifest(&root) else { continue };
        let Some(scene) = manifest.fitted_scene.clone() else { continue };
        let name = renames.get(&scene).cloned().or_else(|| {
            let legacy = scene.ends_with(".yaml") || scene.ends_with(".yml");
            legacy.then(|| super::scene_store::name_of(&scene)).flatten()
        });
        if let Some(name) = name.filter(|n| *n != scene) {
            manifest.fitted_scene = Some(name);
            match super::cell_resolve::write_manifest(&root, &manifest) {
                Ok(()) => done.cells.push(cell),
                Err(err) => tracing::warn!("Scenes: cell {cell}: {err:#}"),
            }
        }
    }
}

/// A rename, or a copy and delete across file systems.
fn move_file(from: &Path, to: &Path) -> std::io::Result<()> {
    match std::fs::rename(from, to) {
        Ok(()) => Ok(()),
        Err(_) => {
            std::fs::copy(from, to)?;
            std::fs::remove_file(from)
        }
    }
}

/// A config's `fittedRobotsDir` that names the old folder, set back to the default (empty).
pub fn migrate_config(config: &ConfigStore) {
    let current = config.get();
    let legacy = Path::new(current.fitted_robots_dir.trim()).file_name().is_some_and(|n| n == LEGACY_SCENES_DIR);
    if legacy {
        let mut next = current.clone();
        next.fitted_robots_dir.clear();
        match config.try_set(next) {
            Ok(_) => tracing::info!("Scenes: fittedRobotsDir {} reset to the default ({SCENES_DIR}/)", current.fitted_robots_dir),
            Err(err) => tracing::warn!("Scenes: resetting fittedRobotsDir: {err}"),
        }
    }
}

/// The user's scene keys under the old folder (S3), moved to `catalogue/scenes/`.
pub async fn migrate_store(storage: &Storage) -> anyhow::Result<usize> {
    if storage.mode() != factoryos_protocol::storage::StorageMode::S3 {
        return Ok(0);
    }
    let prefix = format!("catalogue/{LEGACY_SCENES_DIR}/");
    let list = storage.store().list(Scope::User, &prefix).await?;
    let mut moved = 0;
    for object in list.objects {
        let Some(rest) = object.key.strip_prefix(&prefix) else { continue };
        let old = ObjectKey::new(Scope::User, &object.key)?;
        let new = ObjectKey::new(Scope::User, &format!("catalogue/{SCENES_DIR}/{rest}"))?;
        if storage.store().head(&new).await?.is_none() {
            storage.store().copy(&old, &new).await?;
        }
        storage.store().delete(&old).await?;
        moved += 1;
    }
    Ok(moved)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenes_move_without_overwriting() {
        let root = std::env::temp_dir().join(format!("factoryos-scene-migrate-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (old, new) = (root.join(LEGACY_SCENES_DIR), root.join(SCENES_DIR));
        std::fs::create_dir_all(&old).unwrap();
        std::fs::create_dir_all(&new).unwrap();
        std::fs::write(old.join("a.yaml"), "name: a\n").unwrap();
        std::fs::write(old.join("same.yaml"), "name: s\n").unwrap();
        std::fs::write(new.join("same.yaml"), "name: s\n").unwrap();
        std::fs::write(old.join("clash.yaml"), "name: old\n").unwrap();
        std::fs::write(new.join("clash.yaml"), "name: new\n").unwrap();
        let done = migrate_legacy_folder(&root).unwrap();
        assert_eq!(done.moved, ["a.yaml"]);
        assert_eq!(done.duplicates, ["same.yaml"]);
        assert_eq!(done.conflicts, [("clash.yaml".to_owned(), "clash-from-fitted-robots.yaml".to_owned())]);
        assert_eq!(std::fs::read_to_string(new.join("clash.yaml")).unwrap(), "name: new\n");
        assert_eq!(std::fs::read_to_string(new.join("clash-from-fitted-robots.yaml")).unwrap(), "name: old\n");
        assert!(new.join("a.yaml").is_file());
        assert!(!old.exists(), "the empty old folder goes");
        // Again: nothing to do.
        assert!(migrate_legacy_folder(&root).unwrap().is_empty());
        std::fs::remove_dir_all(&root).ok();
    }

    /// The folder layout: flat files become folders (paths from the folder, `name:` the folder's),
    /// a taken name gets `-2`, the cells follow, and a second run does nothing.
    #[test]
    fn flat_scenes_become_folders() {
        let root = std::env::temp_dir().join(format!("factoryos-scene-folders-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (scenes, cells) = (root.join(SCENES_DIR), root.join("cells"));
        let robot = root.join("urdf-dressed-robots/r/r.urdf");
        std::fs::create_dir_all(robot.parent().unwrap()).unwrap();
        std::fs::write(&robot, "<robot/>").unwrap();
        std::fs::create_dir_all(&scenes).unwrap();
        let scene = |name: &str, urdf: &str| format!("name: {name}\nrobots:\n- robot_id: r\n  dressed_urdf: {urdf}\n  position: {{x: 1.0, y: 0.0, z: 0.0}}\n");
        // The user's scene saved as new-scene.yaml under the name bw48-1.
        std::fs::write(scenes.join("new-scene.yaml"), scene("bw48-1", "../urdf-dressed-robots/r/r.urdf")).unwrap();
        std::fs::write(scenes.join("new-scene.zip"), "zip").unwrap();
        std::fs::write(scenes.join("abs.yml"), scene("abs", "/opt/robots/x.urdf")).unwrap();
        // A folder `taken` holds another scene: the flat taken.yaml becomes taken-2.
        std::fs::create_dir_all(scenes.join("taken")).unwrap();
        std::fs::write(scenes.join("taken/scene.yaml"), scene("taken", "../../urdf-dressed-robots/r/r.urdf")).unwrap();
        std::fs::write(scenes.join("taken.yaml"), scene("taken", "../urdf-dressed-robots/other/other.urdf")).unwrap();
        for (cell, file) in [("c1", "new-scene.yaml"), ("c2", "taken.yaml"), ("c3", "already")] {
            std::fs::create_dir_all(cells.join(cell)).unwrap();
            std::fs::write(cells.join(cell).join(super::super::cell_resolve::MANIFEST), format!(r#"{{"format": 2, "kind": "reference", "fitted_scene": "{file}", "cell_id": "{cell}"}}"#)).unwrap();
        }

        let done = migrate_local(&root, Some(&cells)).unwrap();
        assert_eq!(
            done.folders,
            [("abs.yml".to_owned(), "abs".to_owned()), ("new-scene.yaml".to_owned(), "bw48-1".to_owned()), ("taken.yaml".to_owned(), "taken-2".to_owned())]
        );
        assert_eq!(done.renamed, [("new-scene.yaml".to_owned(), "bw48-1".to_owned()), ("taken.yaml".to_owned(), "taken-2".to_owned())]);
        // Named after its own `name:` (bw48-1), not the file it was left in.
        let moved = super::super::scene_store::read(&scenes.join("bw48-1/scene.yaml")).unwrap();
        assert_eq!((moved.name.as_str(), moved.robots[0].dressed_urdf.as_str()), ("bw48-1", "../../urdf-dressed-robots/r/r.urdf"));
        assert!(scenes.join("bw48-1/scene.yaml").parent().unwrap().join(&moved.robots[0].dressed_urdf).is_file(), "resolves from the folder");
        assert!(scenes.join("bw48-1/bw48-1.zip").is_file() && !scenes.join("new-scene.zip").exists(), "the bundle moved along");
        assert!(!scenes.join("new-scene").exists());
        let abs = super::super::scene_store::read(&scenes.join("abs/scene.yaml")).unwrap();
        assert_eq!(abs.robots[0].dressed_urdf, "/opt/robots/x.urdf", "absolute paths stay");
        let taken = std::fs::read_to_string(scenes.join("taken/scene.yaml")).unwrap();
        assert!(taken.contains("../../urdf-dressed-robots/r/r.urdf"), "the folder's scene is not overwritten: {taken}");
        assert!(std::fs::read_to_string(scenes.join("taken-2/scene.yaml")).unwrap().contains("../../urdf-dressed-robots/other/other.urdf"));
        assert!(!scenes.join("new-scene.yaml").exists() && !scenes.join("taken.yaml").exists());
        let fitted = |cell: &str| super::super::cell_resolve::read_manifest(&cells.join(cell)).unwrap().fitted_scene.unwrap();
        assert_eq!((fitted("c1"), fitted("c2"), fitted("c3")), ("bw48-1".to_owned(), "taken-2".to_owned(), "already".to_owned()));
        assert_eq!(done.cells, ["c1", "c2"]);

        // Again: nothing to do.
        assert!(migrate_local(&root, Some(&cells)).unwrap().is_empty());
        // The same flat file brought again (e.g. by the shared catalogue's sync): dropped, also when
        // its scene had to take another name.
        std::fs::write(scenes.join("taken.yaml"), scene("taken", "../urdf-dressed-robots/other/other.urdf")).unwrap();
        let again = migrate_local(&root, Some(&cells)).unwrap();
        assert_eq!((again.flat_duplicates.as_slice(), again.folders.len()), (["taken.yaml".to_owned()].as_slice(), 0));
        assert!(!scenes.join("taken.yaml").exists() && !scenes.join("taken-3").exists());
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn legacy_keys_are_the_scenes_folder() {
        assert_eq!(crate::storage::local::catalogue_rel("fitted-robots/a.yaml"), "scenes/a/scene.yaml");
        assert_eq!(crate::storage::local::catalogue_rel("scenes/a.yml"), "scenes/a/scene.yaml");
        assert_eq!(crate::storage::local::catalogue_rel("scenes/a/scene.yaml"), "scenes/a/scene.yaml");
        assert_eq!(crate::storage::local::catalogue_rel("fitted-robots/a.zip"), "scenes/a.zip");
        assert_eq!(crate::storage::local::catalogue_rel("fitted-robots-x/a.yaml"), "fitted-robots-x/a.yaml");
        assert_eq!(crate::storage::local::catalogue_rel("urdf-dressed-robots/a/a.urdf"), "urdf-dressed-robots/a/a.urdf");
    }
}
