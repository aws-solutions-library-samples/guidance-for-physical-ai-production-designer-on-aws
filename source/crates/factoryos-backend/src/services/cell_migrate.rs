//! `fos-migrate-cells` (`factoryos-backend migrate-cells`): turn legacy cells into reference cells
//! (see [`super::cell_resolve`]), keeping everything they made.
//!
//! * A copy the old Create Cell made (manifest format 1) whose `scene.yaml` is what it made of its
//!   Robo Fitter scene as saved now, and whose robot and fixture packages hash the same as the
//!   scene's catalogue/product packages (Prepare Cell's SRDFs and `meshes_lowres/` aside), simply
//!   references that scene.
//! * Any other cell with a scene of its own (`scene.yaml`, else `scene.factoryos.yaml`; e.g.
//!   umarell's reference cell `welding_cell_2_small`, a copy whose scene changed since or whose
//!   copies differ) is imported: each robot package goes into the catalogue's
//!   `urdf-dressed-robots/<name>`, each fixture into the products folder (`product.json`) or
//!   `urdf-fixture-collection/<name>`, the floor plan into `floor-plans/` — unless one of that name
//!   with the same content is there already (then that one is used). A name taken by other content
//!   gets the variant name `<name>@<cell>` (`<name>@<cell>_2`, …; the cell's view still calls it
//!   `<name>`, see [`super::cells::without_variant`]). The cell's scene becomes the fitted scene
//!   `<cell>` (`scenes/<cell>/scene.yaml`; `<cell>-2`, … if that is another scene, see
//!   [`super::scene_store`]): the cell's text, its `name:` the scene's, with each asset path that
//!   does not already name its import from the scene's folder rewritten.
//! * A planner cell without a scene (only `agentic_planner/`) whose cell umarell's checkout has
//!   (`<agenticRepoDir>/cells/<cell id>`) is imported from there (the checkout is not changed).
//!
//! Migrating a cell: the whole folder is archived as `cells/.backup/<cell>-<unix s>.tar` first; the
//! robots' SRDFs move to `agentic_planner/srdf/`; the inputs (`robots/`, `fixtures/`,
//! `floorplans/`, `usda-extracted/`, the USDA zip, `scene.yaml`, `scene.factoryos.yaml`, a scaffold
//! `cell.yaml`) are removed; the manifest becomes format 2 and records what was imported
//! (`imported`: the scene's sha256 before and as written, so the planner router still recognises a
//! policy's training cell). Its outputs stay: `agentic_planner/` (the prepared model, task
//! programs, plans, logs), `plan2cell/` builds and deploys, and its own files (`cell.yaml`,
//! `io_names.json`, `krl/`, …), which its view links. The prepared model counts as prepared from
//! the inputs (`agentic_planner/inputs.json`) when its URDF paths are the cell's own (relative),
//! i.e. the files that were imported. A failed migration restores the folder from the backup and
//! removes what it imported for it.
//!
//! What stays legacy, with the reason: a cell with no scene at all (the KRL sample cells, whose 3D
//! scene is their shared environment directory's), a scene whose files are missing. Dry-run unless
//! `--apply`.

use std::path::{Path, PathBuf};

use anyhow::Context as _;

use super::cell_resolve::{self, CellKind, Manifest};
use super::cells::{self, CopyKind};
use super::scene_store;

/// Where a migration reads and writes.
#[derive(Clone, Debug)]
pub struct Dirs {
    pub cells: PathBuf,
    /// The Robo Fitter's scenes (`<assets>/scenes`).
    pub fitted: PathBuf,
    /// The catalogue: `urdf-dressed-robots/`, `urdf-fixture-collection/`, `floor-plans/`.
    pub assets: PathBuf,
    pub products: PathBuf,
    /// umarell's checkout's `cells/`, if there is one.
    pub umarell_cells: Option<PathBuf>,
}

/// One package (a floor plan: file) a migration uses.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Import {
    /// `robot`, `fixture`, `product` or `floor plan`.
    pub what: &'static str,
    /// The package folder (floor plan: file) it is read from.
    pub from: PathBuf,
    /// Where it is (to be) in the catalogue or products folder.
    pub to: PathBuf,
    /// Copied there (else the same content is there already).
    pub copy: bool,
    /// The URDF (floor plan) there, and its content hash ([`content_hash`]).
    pub file: PathBuf,
    pub hash: String,
}

/// How a cell becomes a reference cell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// The fitted scene it will reference (its name).
    pub scene: String,
    /// The scene text to write as `scenes/<scene>/scene.yaml` (`None`: it is there already).
    pub write_scene: Option<String>,
    /// The scene file it was made from (the cell's own, or umarell's).
    pub source_scene: PathBuf,
    /// The source is the cell's own folder (its inputs are removed), else umarell's checkout.
    pub from_cell: bool,
    pub imports: Vec<Import>,
    /// A USDA bundle to copy beside the new scene: (from, to).
    pub usda: Option<(PathBuf, PathBuf)>,
}

/// What [`plan`] / [`run`] decided for one cell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Decision {
    /// Becomes a reference cell (with `run(apply)`: has become one; its backup).
    Migrate { plan: Plan, backup: Option<PathBuf> },
    /// Stays a legacy (self-contained) cell, and why.
    Legacy(String),
    /// A reference cell already.
    Reference,
    /// Migrating it failed (it was left as it was, or restored from the backup).
    Failed(String),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellMigration {
    pub cell: String,
    pub scene: Option<String>,
    pub decision: Decision,
    /// What an applied migration changed on disk, for the store (S3 mode, see [`Touched`]).
    pub touched: Touched,
}

/// The files and folders a migration wrote and removed: the user's store (S3 mode) follows them
/// (main.rs), or a new instance would get the legacy cell's inputs back.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Touched {
    /// Written: the cell's folder, the imported packages, the fitted scene, the USDA bundle.
    pub saved: Vec<PathBuf>,
    /// Removed: the cell's copies of its inputs.
    pub removed: Vec<PathBuf>,
}

impl Touched {
    pub fn is_empty(&self) -> bool {
        self.saved.is_empty() && self.removed.is_empty()
    }

    /// The store follows (S3 mode): what was written goes up, what was removed goes from it.
    pub async fn write_through(&self, storage: &crate::storage::Storage) {
        for path in &self.saved {
            if path.is_dir() { storage.save_tree(path).await } else { storage.save_path(path).await }
        }
        for path in &self.removed {
            storage.prune(path).await;
        }
    }

    fn extend(&mut self, other: Touched) {
        self.saved.extend(other.saved);
        self.removed.extend(other.removed);
    }

    /// What migrating cell `root` by `plan` wrote and removed (`zips`: its root's zips before).
    fn of(dirs: &Dirs, root: &Path, plan: &Plan, zips: Vec<PathBuf>) -> Self {
        let mut saved = vec![root.to_owned()];
        for import in plan.imports.iter().filter(|i| i.copy) {
            saved.push(import.to.clone());
            let mtl = import.to.with_extension("mtl");
            if !import.to.is_dir() && mtl.is_file() {
                saved.push(mtl);
            }
        }
        if plan.write_scene.is_some() {
            saved.push(scene_store::scene_file(&dirs.fitted, &plan.scene));
        }
        if let Some((_, to)) = &plan.usda {
            saved.push(to.clone());
        }
        let mut removed: Vec<PathBuf> = if plan.from_cell {
            ["robots", "fixtures", "floorplans", cells::USDA_EXTRACTED, cells::SCENE_MARKER, cells::FACTORY_OS_SCENE, cells::CELL_MARKER]
                .iter()
                .map(|input| root.join(input))
                .chain(zips)
                .collect()
        } else {
            Vec::new()
        };
        // Only what is gone indeed (a cell.yaml that was no scaffold stays).
        removed.retain(|path| path.symlink_metadata().is_err());
        Self { saved, removed }
    }
}

/// Everything the applied migrations of `results` touched.
pub fn touched(results: &[CellMigration]) -> Touched {
    let mut all = Touched::default();
    for result in results {
        all.extend(result.touched.clone());
    }
    all
}

/// What [`main`] did.
pub struct Outcome {
    /// No migration failed.
    pub ok: bool,
    pub touched: Touched,
    /// The config file it read (the store's layout is that config's).
    pub config_file: PathBuf,
}

impl std::fmt::Display for CellMigration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let scene = self.scene.as_deref().map(|s| format!(" ({s})")).unwrap_or_default();
        match &self.decision {
            Decision::Migrate { plan, backup } => {
                match backup {
                    None => write!(f, "{}{scene}: would become a reference cell of {}", self.cell, plan.scene)?,
                    Some(b) => write!(f, "{}{scene}: now a reference cell of {} (backup {})", self.cell, plan.scene, b.display())?,
                }
                if plan.write_scene.is_some() {
                    let from = if plan.from_cell {
                        format!("its {}", plan.source_scene.file_name().unwrap_or_default().to_string_lossy())
                    } else {
                        plan.source_scene.display().to_string()
                    };
                    write!(f, "; scene scenes/{}/ from {from}", plan.scene)?;
                }
                for import in &plan.imports {
                    let how = if import.copy { "imported as" } else { "same as" };
                    write!(f, "; {} {} {how} {}", import.what, base(&import.from), import.to.display())?;
                }
                if let Some((_, to)) = &plan.usda {
                    write!(f, "; USDA bundle {}", to.display())?;
                }
                Ok(())
            }
            Decision::Legacy(why) => write!(f, "{}{scene}: stays a legacy cell: {why}", self.cell),
            Decision::Reference => write!(f, "{}{scene}: a reference cell already", self.cell),
            Decision::Failed(err) => write!(f, "{}{scene}: migrating failed: {err}", self.cell),
        }
    }
}

fn base(path: &Path) -> String {
    path.file_name().unwrap_or_default().to_string_lossy().into_owned()
}

/// The first line of the `cell.yaml` the old Create Cell wrote (a scaffold, not a cell's own).
const SCAFFOLD_HEADER: &str = "# Auto-generated by \"Create Cellrunner Cell\"";

/// The parts of a scene [`scene_source`] looks at.
#[derive(serde::Deserialize)]
struct SceneShape {
    #[serde(default)]
    robots: Vec<SceneRobot>,
    #[serde(default)]
    fixtures: Vec<SceneFixture>,
}

#[derive(serde::Deserialize)]
struct SceneRobot {
    #[serde(default)]
    dressed_urdf: String,
}

#[derive(serde::Deserialize)]
struct SceneFixture {
    #[serde(default)]
    urdf: String,
}

/// Whether `path` is a Robo Fitter scene (`Some(has assets)`), else `None`.
fn scene_shape(path: &Path) -> Option<bool> {
    let shape: SceneShape = serde_yaml::from_str(&std::fs::read_to_string(path).ok()?).ok()?;
    Some(shape.robots.iter().any(|r| !r.dressed_urdf.trim().is_empty()) || shape.fixtures.iter().any(|f| !f.urdf.trim().is_empty()))
}

/// Destinations planned (not yet written, in a dry run) and their content hashes: later cells of
/// the same run reuse them.
#[derive(Default)]
struct Pending(Vec<(PathBuf, String)>);

impl Pending {
    fn hash(&self, path: &Path) -> Option<&str> {
        self.0.iter().find(|(p, _)| p == path).map(|(_, h)| h.as_str())
    }
}

/// Each cell's decision, and with `apply` migrate the cells it decides to migrate (in name order:
/// a later cell reuses what an earlier one imported).
pub fn run(dirs: &Dirs, apply: bool) -> Vec<CellMigration> {
    let mut pending = Pending::default();
    let mut out = Vec::new();
    for cell in cells::list_cells(&dirs.cells) {
        let root = dirs.cells.join(&cell);
        let scene = cell_resolve::read_manifest(&root).and_then(|m| m.fitted_scene);
        let mut decision = decide(&root, dirs, &mut pending);
        let mut touched = Touched::default();
        if apply && let Decision::Migrate { plan, .. } = &decision {
            let zips = cells::root_zips(&root);
            decision = match migrate(dirs, &root, plan) {
                Ok(backup) => {
                    touched = Touched::of(dirs, &root, plan, zips);
                    Decision::Migrate { plan: plan.clone(), backup: Some(backup) }
                }
                Err(err) => Decision::Failed(format!("{err:#}")),
            };
        }
        out.push(CellMigration { cell, scene, decision, touched });
    }
    out
}

fn decide(root: &Path, dirs: &Dirs, pending: &mut Pending) -> Decision {
    let cell = base(root);
    let own = match cell_resolve::resolve_root(root, &dirs.fitted) {
        Ok(own) => own,
        Err(err) => return Decision::Failed(format!("{err:#}")),
    };
    if own.kind == CellKind::Reference {
        return Decision::Reference;
    }
    if root.join("cellrunner").exists() {
        return Decision::Legacy("its cell.yaml is in cellrunner/ (an older layout): move it to the cell's root first".to_owned());
    }
    let manifest = cell_resolve::read_manifest(root);
    if let Some(plan) = plain_copy(root, &own, manifest.as_ref(), dirs) {
        return Decision::Migrate { plan, backup: None };
    }
    let Some((source_scene, from_cell)) = scene_source(root, &cell, dirs) else {
        let umarell = dirs
            .umarell_cells
            .as_ref()
            .map(|u| format!(", nor has umarell's checkout a cell {}", u.join(cells::planner_cell_id(&cell)).display()))
            .unwrap_or_default();
        let krl = if root.join("krl").is_dir() || root.join(cells::CELL_MARKER).is_file() {
            " (a KRL cell: its 3D scene is its environment directory's, robots by model, which is no Robo Fitter scene)"
        } else {
            ""
        };
        return Decision::Legacy(format!("it has no Robo Fitter scene of its own (scene.yaml){umarell}{krl}"));
    };
    match import_plan(root, &cell, &source_scene, from_cell, manifest.as_ref(), dirs, pending) {
        Ok(plan) => Decision::Migrate { plan, backup: None },
        Err(err) => Decision::Legacy(format!("{err:#}")),
    }
}

/// A copy the old Create Cell made of its scene as saved now, with the scene's very packages: it
/// references that scene, nothing imported.
fn plain_copy(root: &Path, own: &cell_resolve::ResolvedCell, manifest: Option<&Manifest>, dirs: &Dirs) -> Option<Plan> {
    let scene = scene_store::name_of(manifest?.fitted_scene.as_deref()?)?;
    let fitted = scene_store::scene_file(&dirs.fitted, &scene);
    let fitted_text = std::fs::read_to_string(&fitted).ok()?;
    let cell_scene = std::fs::read_to_string(root.join(cells::SCENE_MARKER)).ok()?;
    if cell_scene != cells::plan_cell_scaffold(&fitted_text).scene || root.join(cells::FACTORY_OS_SCENE).exists() {
        return None;
    }
    let cell_yaml = root.join(cells::CELL_MARKER);
    if cell_yaml.exists() && !std::fs::read_to_string(&cell_yaml).is_ok_and(|t| t.starts_with(SCAFFOLD_HEADER)) {
        return None;
    }
    let reference = resolve_scene(&scene, &dirs.fitted).ok()?;
    for (ours, theirs) in [(&own.inputs.urdfs, &reference.urdfs), (&own.inputs.fixtures, &reference.fixtures)] {
        if ours.keys().ne(theirs.keys()) || ours.iter().any(|(id, hash)| hash.is_empty() || theirs.get(id) != Some(hash)) {
            return None;
        }
    }
    Some(Plan { scene, write_scene: None, source_scene: root.join(cells::SCENE_MARKER), from_cell: true, imports: Vec::new(), usda: None })
}

/// The input hashes of fitted scene `scene` (a name) as a reference cell would see them.
fn resolve_scene(scene: &str, fitted_dir: &Path) -> anyhow::Result<cell_resolve::InputHashes> {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let probe = std::env::temp_dir().join(format!("factoryos-migrate-probe-{}-{seq}", std::process::id()));
    std::fs::create_dir_all(&probe)?;
    let result = cell_resolve::write_manifest(&probe, &Manifest::reference(scene, "probe"))
        .and_then(|()| cell_resolve::resolve_root(&probe, fitted_dir));
    std::fs::remove_dir_all(&probe).ok();
    let resolved = result?;
    for asset in resolved.robots.iter().chain(&resolved.fixtures) {
        anyhow::ensure!(asset.urdf.is_file(), "{} is missing", asset.urdf.display());
    }
    Ok(resolved.inputs)
}

/// The scene a cell is imported from: its own `scene.yaml` / `scene.factoryos.yaml` (the first that
/// is a Robo Fitter scene with robots or fixtures, else the first that is one), else (a planner
/// cell) umarell's checkout's cell's `scene.yaml`. And whether that is the cell's own.
fn scene_source(root: &Path, cell: &str, dirs: &Dirs) -> Option<(PathBuf, bool)> {
    let own: Vec<(PathBuf, bool)> = [cells::SCENE_MARKER, cells::FACTORY_OS_SCENE]
        .iter()
        .map(|f| root.join(f))
        .filter_map(|p| scene_shape(&p).map(|assets| (p, assets)))
        .collect();
    if let Some((path, _)) = own.iter().find(|(_, assets)| *assets).or(own.first()) {
        return Some((path.clone(), true));
    }
    if !root.join("agentic_planner").is_dir() {
        return None;
    }
    let scene = dirs.umarell_cells.as_ref()?.join(cells::planner_cell_id(cell)).join(cells::SCENE_MARKER);
    scene_shape(&scene).filter(|assets| *assets).map(|_| (scene, false))
}

fn is_under(path: &Path, dir: &Path) -> bool {
    path.starts_with(dir)
}

/// The content hash of a package (`urdf`'s folder; see [`cell_resolve::package_hash`]), or of a
/// floor plan file with its `.mtl`.
fn content_hash(kind: CopyKind, file: &Path) -> String {
    match kind {
        CopyKind::Dir => cell_resolve::package_hash(file),
        CopyKind::File => {
            let mtl = file.with_extension("mtl");
            let mtl = if mtl.is_file() { cell_resolve::file_hash(&mtl) } else { String::new() };
            let obj = cell_resolve::file_hash(file);
            if obj.is_empty() { obj } else { format!("{obj}/{mtl}") }
        }
    }
}

/// Where the package of `file` (a URDF; a floor plan file) with content `hash` goes in
/// `collection`: `(the file there, copy it)`. Its own name if that is free or the same content,
/// else an existing variant `<name>@…` of the same content, else the first free `<name>@<cell>`,
/// `<name>@<cell>_2`, ….
fn place(kind: CopyKind, collection: &Path, name: &str, file_name: &str, cell: &str, hash: &str, pending: &Pending) -> (PathBuf, bool) {
    let (stem, ext) = match kind {
        CopyKind::File => match name.rsplit_once('.') {
            Some((stem, ext)) if !stem.is_empty() => (stem.to_owned(), format!(".{ext}")),
            _ => (name.to_owned(), String::new()),
        },
        CopyKind::Dir => (name.to_owned(), String::new()),
    };
    let at = |entry: &str| match kind {
        CopyKind::Dir => collection.join(entry).join(file_name),
        CopyKind::File => collection.join(entry),
    };
    let state = |entry: &str| -> Option<bool> {
        // Some(true): free; Some(false): the same content; None: other content.
        let file = at(entry);
        if let Some(h) = pending.hash(&file) {
            return (h == hash).then_some(false);
        }
        let exists = match kind {
            CopyKind::Dir => collection.join(entry).exists(),
            CopyKind::File => file.exists(),
        };
        if !exists {
            return Some(true);
        }
        (content_hash(kind, &file) == hash).then_some(false)
    };
    if let Some(free) = state(name) {
        return (at(name), free);
    }
    let prefix = format!("{stem}@");
    let mut variants: Vec<String> = std::fs::read_dir(collection)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .chain(pending.0.iter().filter_map(|(p, _)| {
            let entry = match kind {
                CopyKind::Dir => p.parent()?,
                CopyKind::File => p.as_path(),
            };
            (entry.parent() == Some(collection)).then(|| base(entry))
        }))
        .filter(|e| e.starts_with(&prefix) && e.ends_with(&ext))
        .collect();
    variants.sort();
    variants.dedup();
    if let Some(same) = variants.iter().find(|v| state(v) == Some(false)) {
        return (at(same), false);
    }
    for i in 1.. {
        let entry = if i == 1 { format!("{stem}@{cell}{ext}") } else { format!("{stem}@{cell}_{i}{ext}") };
        if let Some(free) = state(&entry) {
            return (at(&entry), free);
        }
    }
    unreachable!()
}

/// The value a fitted scene (in its folder `<fitted_dir>/<name>/`) gives `file`: `original` if it
/// names it from there already, an absolute path for a product (as the Robo Fitter writes them),
/// else relative to the scene's folder when `file` is beside the scenes folder
/// (`../../urdf-dressed-robots/…`), else absolute.
fn fitted_value(dirs: &Dirs, original: &str, file: &Path) -> String {
    let named = Path::new(original);
    // Every scene's folder is at the same depth: any name stands for the one not chosen yet.
    let named = if named.is_absolute() { named.to_path_buf() } else { dirs.fitted.join("scene").join(named) };
    if cell_resolve::normalize(&named) == file {
        return original.to_owned();
    }
    if !is_under(file, &dirs.products)
        && let Some(parent) = dirs.fitted.parent()
        && let Ok(rel) = file.strip_prefix(parent)
    {
        return format!("../../{}", rel.display());
    }
    file.display().to_string()
}

/// Plan importing cell `root`'s scene `source_scene` and the packages it names (see the module docs).
fn import_plan(
    root: &Path,
    cell: &str,
    source_scene: &Path,
    from_cell: bool,
    manifest: Option<&Manifest>,
    dirs: &Dirs,
    pending: &mut Pending,
) -> anyhow::Result<Plan> {
    let text = std::fs::read_to_string(source_scene).with_context(|| format!("reading {}", source_scene.display()))?;
    let source_dir = source_scene.parent().unwrap_or(root).to_path_buf();
    let mut imports: Vec<Import> = Vec::new();
    let mut error: Option<anyhow::Error> = None;
    let rewritten = cells::rewrite_asset_lines(&text, |kind, dest, value| {
        if error.is_some() {
            return None;
        }
        match locate(kind, dest, value, &source_dir, cell, dirs, pending, &mut imports) {
            Ok(new) => (new != value).then_some(new),
            Err(err) => {
                error = Some(err);
                None
            }
        }
    });
    if let Some(err) = error {
        return Err(err);
    }
    // Its USDA bundle (a copy's, at its root).
    let usda = if from_cell { cells::root_zips(root).into_iter().find(|zip| cells::has_usda(zip)) } else { None };
    let usda_hash = usda.as_deref().map(cell_resolve::file_hash);
    // The scene it was made from, if that is this very scene; else one named after the cell.
    if let Some(scene) = manifest.and_then(|m| scene_store::name_of(m.fitted_scene.as_deref()?))
        && std::fs::read_to_string(scene_store::scene_file(&dirs.fitted, &scene)).is_ok_and(|t| t == named(&rewritten, &scene))
        && cells::find_usda_bundle(&dirs.fitted, &scene).as_deref().map(cell_resolve::file_hash) == usda_hash
    {
        return Ok(Plan { scene, write_scene: None, source_scene: source_scene.to_path_buf(), from_cell, imports, usda: None });
    }
    let base_name = scene_store::sanitize(cell).unwrap_or_else(|| "cell".to_owned());
    let scene = (1..)
        .map(|i| if i == 1 { base_name.clone() } else { format!("{base_name}-{i}") })
        .find(|name| {
            let (path, text) = (scene_store::scene_file(&dirs.fitted, name), named(&rewritten, name));
            match pending.hash(&path) {
                Some(h) => h == sha256_hex(text.as_bytes()),
                None => !scene_store::scene_dir(&dirs.fitted, name).exists() || std::fs::read(&path).is_ok_and(|t| t == text.as_bytes()),
            }
        })
        .expect("unbounded");
    let rewritten = named(&rewritten, &scene);
    let path = scene_store::scene_file(&dirs.fitted, &scene);
    let write_scene = (!path.exists() && pending.hash(&path).is_none()).then(|| rewritten.clone());
    if write_scene.is_some() {
        pending.0.push((path, sha256_hex(rewritten.as_bytes())));
    }
    let usda = usda
        .filter(|_| cells::find_usda_bundle(&dirs.fitted, &scene).is_none())
        .map(|zip| (zip, scene_store::scene_dir(&dirs.fitted, &scene).join(format!("{scene}-usda.zip"))));
    Ok(Plan { scene, write_scene, source_scene: source_scene.to_path_buf(), from_cell, imports, usda })
}

/// Scene text `text` with its top-level `name:` set to `name` (added first if it has none).
fn named(text: &str, name: &str) -> String {
    let mut done = false;
    let mut lines: Vec<String> = text
        .split('\n')
        .map(|line| {
            if !done && line.starts_with("name:") {
                done = true;
                format!("name: {name}")
            } else {
                line.to_owned()
            }
        })
        .collect();
    if !done {
        lines.insert(0, format!("name: {name}"));
    }
    lines.join("\n")
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::Digest as _;
    sha2::Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

/// Where the asset `value` (of `kind`, cell subtree `dest`) of a scene in `source_dir` is, where it
/// goes, and its value in the fitted scene. Records the import.
#[allow(clippy::too_many_arguments)]
fn locate(
    kind: CopyKind,
    dest: &str,
    value: &str,
    source_dir: &Path,
    cell: &str,
    dirs: &Dirs,
    pending: &mut Pending,
    imports: &mut Vec<Import>,
) -> anyhow::Result<String> {
    let file_name = base(Path::new(value));
    let package_name = match kind {
        CopyKind::Dir => base(Path::new(value).parent().unwrap_or(Path::new(""))),
        CopyKind::File => file_name.clone(),
    };
    let what = match (kind, dest) {
        (CopyKind::File, _) => "floor plan",
        (_, "robots") => "robot",
        _ => "fixture",
    };
    // The cell's own copy (the layout the old Create Cell and umarell's cells use), else the path as
    // written (from the scene's folder).
    let own = match kind {
        CopyKind::Dir => source_dir.join(dest).join(&package_name).join(&file_name),
        CopyKind::File => {
            let stem = Path::new(&file_name).file_stem().unwrap_or_default().to_string_lossy().into_owned();
            source_dir.join(dest).join(stem).join(&file_name)
        }
    };
    let written = cell_resolve::normalize(&if Path::new(value).is_absolute() { PathBuf::from(value) } else { source_dir.join(value) });
    let (file, in_cell) = if own.is_file() {
        (own, true)
    } else if written.is_file() {
        (written, false)
    } else {
        anyhow::bail!("its {what} {value} is missing (not at {} nor {})", own.display(), written.display());
    };
    let hash = content_hash(kind, &file);
    anyhow::ensure!(!hash.is_empty(), "its {what} {} can't be read", file.display());
    let package = match kind {
        CopyKind::Dir => file.parent().unwrap_or(&file).to_path_buf(),
        CopyKind::File => file.clone(),
    };
    let (what, collection) = match (kind, dest) {
        (CopyKind::File, _) => (what, dirs.assets.join("floor-plans")),
        (_, "robots") => (what, dirs.assets.join("urdf-dressed-robots")),
        _ if package.join("product.json").is_file() => ("product", dirs.products.clone()),
        _ => (what, dirs.assets.join("urdf-fixture-collection")),
    };
    // Already in the catalogue or the products folder (a scene path that named it there): as it is.
    let in_place = !in_cell
        && [dirs.assets.join("urdf-dressed-robots"), dirs.assets.join("urdf-fixture-collection"), dirs.assets.join("floor-plans"), dirs.products.clone()]
            .iter()
            .any(|dir| match kind {
                CopyKind::Dir => package.parent() == Some(dir.as_path()),
                CopyKind::File => package.parent() == Some(dir.as_path()),
            });
    let (to, copy) = if in_place { (file.clone(), false) } else { place(kind, &collection, &package_name, &file_name, cell, &hash, pending) };
    if copy {
        pending.0.push((to.clone(), hash.clone()));
    }
    let to_package = match kind {
        CopyKind::Dir => to.parent().unwrap_or(&to).to_path_buf(),
        CopyKind::File => to.clone(),
    };
    if !imports.iter().any(|i| i.from == package && i.to == to_package) {
        imports.push(Import { what, from: package, to: to_package, copy, file: to.clone(), hash });
    }
    Ok(fitted_value(dirs, value, &to))
}

fn unix_s() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// Copy package `from` to `to` (a new folder), without SRDFs (they are the cell's) and hidden files.
fn copy_package(from: &Path, to: &Path) -> anyhow::Result<()> {
    std::fs::create_dir_all(to)?;
    for entry in std::fs::read_dir(from)? {
        let entry = entry?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with('.') {
            continue;
        }
        let target = to.join(&name);
        if std::fs::metadata(entry.path())?.is_dir() {
            copy_package(&entry.path(), &target)?;
        } else if !name.to_lowercase().ends_with(".srdf") {
            std::fs::copy(entry.path(), &target).with_context(|| format!("copying {}", entry.path().display()))?;
        }
    }
    Ok(())
}

/// Whether the prepared planner model's robots are the cell's own files (relative `urdf:` paths):
/// then it was prepared from what the migration imported.
fn prepared_from_own_files(root: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(root.join(super::planner_model::CELL_YAML)) else { return false };
    let urdfs: Vec<&str> = text.lines().filter_map(|l| l.trim_start().strip_prefix("urdf:")).map(str::trim).collect();
    !urdfs.is_empty() && urdfs.iter().all(|u| !u.is_empty() && !Path::new(u).is_absolute())
}

/// Migrate cell `root` by `plan` (see the module docs). Returns its backup.
fn migrate(dirs: &Dirs, root: &Path, plan: &Plan) -> anyhow::Result<PathBuf> {
    let cell = base(root);
    let old_manifest = cell_resolve::read_manifest(root);
    // Before its copy (the cell's own scene) is removed.
    let source_sha = cell_resolve::file_hash(&plan.source_scene);
    let backup_dir = dirs.cells.join(".backup");
    std::fs::create_dir_all(&backup_dir)?;
    let ts = unix_s();
    let backup = (0..)
        .map(|i| backup_dir.join(if i == 0 { format!("{cell}-{ts}.tar") } else { format!("{cell}-{ts}-{i}.tar") }))
        .find(|p| !p.exists())
        .expect("unbounded");
    let status = std::process::Command::new("tar")
        .arg("-cf")
        .arg(&backup)
        .arg("-C")
        .arg(&dirs.cells)
        .arg(&cell)
        .status()
        .context("running tar")?;
    anyhow::ensure!(status.success(), "tar failed ({status}): nothing changed");

    let mut created: Vec<PathBuf> = Vec::new();
    let result = (|| -> anyhow::Result<()> {
        // The imports (into a temporary name, then renamed: never a half-copied package).
        for import in plan.imports.iter().filter(|i| i.copy) {
            if import.to.exists() {
                continue; // an earlier cell of this run imported it
            }
            let parent = import.to.parent().context("no parent")?;
            if !parent.exists() {
                std::fs::create_dir_all(parent)?;
                created.push(parent.to_path_buf());
            }
            let tmp = parent.join(format!(".{}.import-{}", base(&import.to), std::process::id()));
            cells::remove_path(&tmp)?;
            if import.from.is_dir() {
                copy_package(&import.from, &tmp)?;
            } else {
                std::fs::copy(&import.from, &tmp)?;
                let mtl = import.from.with_extension("mtl");
                if mtl.is_file() {
                    std::fs::copy(&mtl, import.to.with_extension("mtl"))?;
                    created.push(import.to.with_extension("mtl"));
                }
            }
            std::fs::rename(&tmp, &import.to)?;
            created.push(import.to.clone());
        }
        if let Some(text) = &plan.write_scene {
            let path = scene_store::scene_file(&dirs.fitted, &plan.scene);
            if !path.exists() {
                let folder = scene_store::scene_dir(&dirs.fitted, &plan.scene);
                if !folder.exists() {
                    std::fs::create_dir_all(&folder)?;
                    created.push(folder.clone());
                }
                let tmp = folder.join(format!(".scene.import-{}", std::process::id()));
                std::fs::write(&tmp, text)?;
                std::fs::rename(&tmp, &path)?;
                created.push(path);
            }
        }
        if let Some((from, to)) = &plan.usda
            && !to.exists()
        {
            std::fs::copy(from, to)?;
            created.push(to.clone());
        }
        // The robots' SRDFs (Prepare Cell's), into the cell.
        let srdf_dir = root.join(cell_resolve::SRDF_DIR);
        let source_dir = plan.source_scene.parent().unwrap_or(root);
        for package in std::fs::read_dir(source_dir.join("robots")).into_iter().flatten().flatten() {
            for file in std::fs::read_dir(package.path()).into_iter().flatten().flatten() {
                let path = file.path();
                if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("srdf")) {
                    std::fs::create_dir_all(&srdf_dir)?;
                    let to = srdf_dir.join(file.file_name());
                    if to.exists() {
                        continue;
                    }
                    if plan.from_cell {
                        std::fs::rename(&path, &to)?;
                    } else {
                        std::fs::copy(&path, &to)?;
                    }
                }
            }
        }
        // The cell's copies of its inputs (they are in the backup).
        if plan.from_cell {
            let scaffold = std::fs::read_to_string(root.join(cells::CELL_MARKER)).is_ok_and(|t| t.starts_with(SCAFFOLD_HEADER));
            let mut inputs = vec!["robots", "fixtures", "floorplans", cells::USDA_EXTRACTED, cells::SCENE_MARKER, cells::FACTORY_OS_SCENE];
            if scaffold {
                inputs.push(cells::CELL_MARKER);
            }
            for owned in inputs {
                cells::remove_path(&root.join(owned))?;
            }
            for zip in cells::root_zips(root).into_iter().filter(|zip| cells::has_usda(zip)) {
                cells::remove_path(&zip)?;
            }
        }
        cells::remove_path(&root.join(cell_resolve::VIEW_DIR))?;
        let mut migrated = Manifest::reference(&plan.scene, &cell);
        if let Some(old) = &old_manifest {
            migrated.extra = old.extra.clone();
            if let Some(source) = old.fitted_scene.as_ref().filter(|s| scene_store::name_of(s).as_deref() != Some(plan.scene.as_str())) {
                migrated.extra.insert("migrated_from".to_owned(), source.clone().into());
            }
        }
        migrated.extra.insert("migrated_s".to_owned(), ts.into());
        migrated.extra.insert("backup".to_owned(), backup.display().to_string().into());
        let fitted_sha = cell_resolve::file_hash(&scene_store::scene_file(&dirs.fitted, &plan.scene));
        migrated.extra.insert(
            "imported".to_owned(),
            serde_json::json!({
                "from": plan.source_scene.display().to_string(),
                // The scene as the cell had it, and as the fitted scene was then: equal while the
                // fitted scene is unchanged (the planner router's training cell check).
                "scene_sha256": source_sha,
                "fitted_sha256": fitted_sha,
                "assets": plan.imports.iter().map(|i| serde_json::json!({
                    "what": i.what, "from": i.from.display().to_string(), "to": i.to.display().to_string(), "copied": i.copy,
                })).collect::<Vec<_>>(),
            }),
        );
        cell_resolve::write_manifest(root, &migrated)?;
        let resolved = cell_resolve::resolve_root(root, &dirs.fitted)?;
        anyhow::ensure!(resolved.kind == CellKind::Reference, "it did not become a reference cell");
        for import in &plan.imports {
            let kind = if import.from.is_dir() || import.to.is_dir() { CopyKind::Dir } else { CopyKind::File };
            anyhow::ensure!(
                content_hash(kind, &import.file) == import.hash,
                "{} {} does not hold what the cell had",
                import.what,
                import.to.display()
            );
        }
        for asset in resolved.robots.iter().chain(&resolved.fixtures) {
            anyhow::ensure!(asset.urdf.is_file(), "{} is missing after the import", asset.urdf.display());
        }
        cell_resolve::materialise_view(&resolved)?;
        if root.join(super::planner_model::CELL_YAML).is_file() && prepared_from_own_files(root) {
            cell_resolve::record_prepared(&resolved)?;
        }
        Ok(())
    })();
    if let Err(err) = result {
        // Put the cell back as it was, and drop what was imported for it.
        for path in created.iter().rev() {
            cells::remove_path(path).ok();
        }
        let restored = cells::remove_path(root).is_ok()
            && std::process::Command::new("tar")
                .arg("-xf")
                .arg(&backup)
                .arg("-C")
                .arg(&dirs.cells)
                .status()
                .is_ok_and(|s| s.success());
        let how = if restored { "restored from" } else { "NOT restored: see" };
        return Err(err.context(format!("cell {cell} {how} {}", backup.display())));
    }
    Ok(backup)
}

/// The command line: `fos-migrate-cells [--apply] [--config <factoryos.config.json>]
/// [--cells-dir <dir>] [--fitted-dir <dir>] [--assets-dir <dir>] [--products-dir <dir>]
/// [--umarell-dir <dir>]`. Prints what it would do (or did); `false` (exit code 1) if a migration
/// failed. What it changed is in the [`Outcome`], for the store.
pub fn main(args: &[String]) -> anyhow::Result<Outcome> {
    let mut apply = false;
    let (mut config, mut cells_dir, mut fitted_dir, mut assets_dir, mut products_dir, mut umarell_dir) = (None, None, None, None, None, None);
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        let mut value = |name: &str| it.next().cloned().map(PathBuf::from).with_context(|| format!("{name} needs a value"));
        match arg.as_str() {
            "--apply" => apply = true,
            "--dry-run" => apply = false,
            "--config" => config = Some(value("--config")?),
            "--cells-dir" => cells_dir = Some(value("--cells-dir")?),
            "--fitted-dir" => fitted_dir = Some(value("--fitted-dir")?),
            "--assets-dir" => assets_dir = Some(value("--assets-dir")?),
            "--products-dir" => products_dir = Some(value("--products-dir")?),
            "--umarell-dir" => umarell_dir = Some(value("--umarell-dir")?),
            "-h" | "--help" => {
                println!(
                    "fos-migrate-cells [--apply] [--config FILE] [--cells-dir DIR] [--fitted-dir DIR] [--assets-dir DIR]\n\
                     \x20                 [--products-dir DIR] [--umarell-dir DIR]\n\
                     Turns legacy cells into reference cells: copies of a scene reference it; a cell with its own scene\n\
                     and files has them imported into the catalogue, the products and scenes/ (a name taken by\n\
                     other content becomes <name>@<cell>). Outputs stay; backups in <cells>/.backup/. Dry-run unless --apply."
                );
                return Ok(Outcome { ok: true, touched: Touched::default(), config_file: PathBuf::new() });
            }
            other => anyhow::bail!("unknown argument {other} (see --help)"),
        }
    }
    let config_file = config
        .or_else(|| std::env::var_os("FACTORY_OS_CONFIG").map(PathBuf::from))
        .unwrap_or_else(crate::paths::default_config_file);
    let store = crate::config_store::ConfigStore::open(config_file.clone());
    let cfg = store.get();
    let cells_dir = cells_dir.or_else(|| cells::cells_folder(&store)).context("no cells folder: pass --cells-dir")?;
    let fitted = fitted_dir.unwrap_or_else(|| cells::fitted_scenes_dir(&store));
    let assets = assets_dir
        .or_else(|| Some(store.resolve(&cfg.asset_root)).filter(|a| !a.is_empty()).map(PathBuf::from))
        .or_else(|| fitted.parent().map(Path::to_path_buf))
        .context("no asset root: pass --assets-dir")?;
    let products = products_dir.unwrap_or_else(|| super::product::products_dir(&store));
    let umarell_cells = umarell_dir
        .or_else(|| Some(store.resolve(&cfg.agentic_repo_dir)).filter(|d| !d.is_empty()).map(PathBuf::from))
        .map(|d| d.join("cells"))
        .filter(|d| d.is_dir());
    let dirs = Dirs { cells: cells_dir, fitted, assets, products, umarell_cells };
    println!(
        "{} cells in {} (scenes {}, catalogue {}, products {}, umarell {}; config {}):",
        if apply { "Migrating" } else { "Dry run: would migrate" },
        dirs.cells.display(),
        dirs.fitted.display(),
        dirs.assets.display(),
        dirs.products.display(),
        dirs.umarell_cells.as_deref().map_or_else(|| "-".to_owned(), |d| d.display().to_string()),
        config_file.display()
    );
    let results = run(&dirs, apply);
    for result in &results {
        println!("  {result}");
    }
    let migrated: Vec<&str> = results
        .iter()
        .filter(|r| matches!(r.decision, Decision::Migrate { backup: Some(_), .. }))
        .map(|r| r.cell.as_str())
        .collect();
    // The active cell's paths point into its old layout: at its view now.
    if let Some((active, _)) = cells::active_cell(&store)
        && migrated.contains(&active.as_str())
    {
        match cells::set_active(&store, &active) {
            Ok(_) => println!(
                "The active cell {active} now runs from its view: restart the backend (it reads the config at start)."
            ),
            Err(err) => println!("Pointing the config at the active cell {active}'s view failed: {err:#}"),
        }
    }
    let ok = !results.iter().any(|r| matches!(r.decision, Decision::Failed(_)));
    Ok(Outcome { ok, touched: touched(&results), config_file })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::cell_resolve::tests::{Host, host, scene_text};

    fn dirs(h: &Host) -> Dirs {
        Dirs {
            cells: h.cells.clone(),
            fitted: h.assets.join("scenes"),
            assets: h.assets.clone(),
            products: h.products.clone(),
            umarell_cells: Some(h.root.join("umarell/cells")),
        }
    }

    fn write(path: &Path, text: &str) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }

    /// A copy as the old Create Cell made it, with Prepare Cell's SRDF, a prepared model (paths
    /// relative to the cell), a plan and a plan2cell build.
    fn copy_cell(h: &Host, cell: &str, scene: &str) -> PathBuf {
        let root = h.cells.join(cell);
        let fitted = scene_store::scene_file(&h.assets.join("scenes"), &scene_store::name_of(scene).unwrap());
        let text = std::fs::read_to_string(&fitted).unwrap();
        let plan = cells::plan_cell_scaffold(&text);
        for copy in &plan.copies {
            let from = fitted.parent().unwrap().join(&copy.from_rel);
            cells::copy_dir(&from, &root.join(&copy.to_rel)).unwrap();
        }
        std::fs::write(root.join("scene.yaml"), &plan.scene).unwrap();
        std::fs::write(root.join("cell.yaml"), cells::build_scaffold_cell_yaml(&plan.scene, scene)).unwrap();
        std::fs::write(root.join(cell_resolve::MANIFEST), format!(r#"{{"fitted_scene": "{scene}"}}"#)).unwrap();
        std::fs::write(root.join("robots/kr240__weld/kr240__weld.srdf"), "<robot/>").unwrap();
        outputs(&root);
        root
    }

    /// A cell's outputs: the prepared model, a task program, a plan, a log, a plan2cell build.
    fn outputs(root: &Path) {
        write(&root.join("agentic_planner/cell.yaml"), "robots:\n  - id: robot1\n    urdf: robots/kr240__weld/kr240__weld.urdf\n    srdf: robots/kr240__weld/kr240__weld.srdf\n");
        write(&root.join("agentic_planner/config/robots.yaml"), "robots: {}\n");
        write(&root.join("agentic_planner/tasks/welds.json"), "{}");
        write(&root.join("agentic_planner/plans/bt_1.json"), "{\"behaviour_tree\": {}}");
        write(&root.join("agentic_planner/logs/plantrace_bt_1.json"), "{}");
        write(&root.join("plan2cell/builds/bt_1/20261014T1Z/build.json"), "{\"ok\": true}");
        write(&root.join("plan2cell/auto-restart.json"), "{\"inProduction\": true}");
    }

    /// umarell's reference cell as FactoryOS ships it: its own scene.yaml (paths as in the catalogue,
    /// `../urdf-dressed-robots/…`, which don't resolve from the cell), scene.factoryos.yaml, robots/
    /// (the URDF the policies were trained on), floorplans/, its own cell.yaml and files, outputs.
    const SMALL_SCENE: &str = "description: \"\"\nrobots:\n  - robot_id: kr240__weld\n    dressed_urdf: ../urdf-dressed-robots/kr240__weld/kr240__weld.urdf\n    \
                               position:\n      x: 0.1\n      y: -3.3\n      z: 0\n  - robot_id: kr240__weld_2\n    dressed_urdf: ../urdf-dressed-robots/kr240__weld/kr240__weld.urdf\n    \
                               position:\n      x: -1.2\n      y: 1.0\n      z: 0\nfixtures: []\nfloor_plan:\n  geometry_file: ../floor-plans/hall.obj\n  scale: 1\n";

    fn small_cell(h: &Host) -> PathBuf {
        let root = h.cells.join("welding_cell_2_small");
        write(&root.join("scene.yaml"), SMALL_SCENE);
        write(&root.join("scene.factoryos.yaml"), &format!("name: welding_cell_2_small\n{}", SMALL_SCENE.replace("../urdf-dressed-robots/", "robots/").replace("../floor-plans/hall.obj", "floorplans/hall/hall.obj")));
        write(&root.join("robots/kr240__weld/kr240__weld.urdf"), "<robot name='policy'/>");
        write(&root.join("robots/kr240__weld/meshes/link1.obj"), "o link1");
        write(&root.join("robots/kr240__weld/kr240__weld.srdf"), "<robot name='policy'/>");
        write(&root.join("floorplans/hall/hall.obj"), "o hall");
        write(&root.join("cell.yaml"), "# Cell-runner manifest\nname: welding_cell_2_small\nrobots:\n  - id: robot1\n    urdf: robots/kr240__weld/kr240__weld.urdf\n");
        write(&root.join("io_names.json"), "{}");
        write(&root.join("NOTE.md"), "# note");
        outputs(&root);
        root
    }

    fn decisions(results: &[CellMigration]) -> Vec<String> {
        results.iter().map(ToString::to_string).collect()
    }

    fn tree(dir: &Path) -> Vec<String> {
        let mut out = Vec::new();
        fn walk(dir: &Path, rel: &Path, out: &mut Vec<String>) {
            for entry in std::fs::read_dir(dir.join(rel)).into_iter().flatten().flatten() {
                let path = rel.join(entry.file_name());
                out.push(path.display().to_string());
                if entry.file_type().is_ok_and(|t| t.is_dir()) {
                    walk(dir, &path, out);
                }
            }
        }
        walk(dir, Path::new(""), &mut out);
        out.sort();
        out
    }

    #[tokio::test]
    async fn the_store_follows_a_migration() {
        use crate::storage::sync::tests_support::{FakeS3, storage_of};
        use factoryos_protocol::storage::Scope;
        let h = host("migrate-store");
        let d = dirs(&h);
        let small = small_cell(&h);
        let s3 = std::sync::Arc::new(FakeS3::default());
        let storage = storage_of(s3.clone(), h.products.clone(), h.cells.clone(), h.assets.clone());
        // The legacy cell as the store has it (written through, or migrated to S3, before).
        storage.save_tree(&small).await;
        assert!(s3.bytes(Scope::User, "cells/welding_cell_2_small/scene.yaml").is_some());
        let results = run(&d, true);
        let touched = touched(&results);
        assert!(touched.removed.contains(&small.join("robots")) && touched.removed.contains(&small.join("scene.yaml")), "{touched:?}");
        touched.write_through(&storage).await;
        let keys = s3.keys(Scope::User);
        let has = |key: &str| keys.iter().any(|k| k == key);
        // Its inputs' copies are gone (a new instance would make it a legacy cell again), its outputs stay.
        assert!(!keys.iter().any(|k| k.starts_with("cells/welding_cell_2_small/robots/") || k.starts_with("cells/welding_cell_2_small/floorplans/")), "{keys:#?}");
        assert!(!has("cells/welding_cell_2_small/scene.yaml"));
        for kept in ["agentic_planner/plans/bt_1.json", "agentic_planner/srdf/kr240__weld.srdf", "cell.yaml", "io_names.json", cell_resolve::MANIFEST] {
            assert!(has(&format!("cells/welding_cell_2_small/{kept}")), "{kept}: {keys:#?}");
        }
        // What it imported, into the user's catalogue additions; the backup stays local.
        assert!(has("catalogue/scenes/welding_cell_2_small/scene.yaml"), "{keys:#?}");
        assert!(has("catalogue/urdf-dressed-robots/kr240__weld@welding_cell_2_small/kr240__weld.urdf"), "{keys:#?}");
        assert!(has("catalogue/floor-plans/hall.obj"), "{keys:#?}");
        assert!(!keys.iter().any(|k| k.contains("/.backup/") || k.contains("/.view/")), "{keys:#?}");
    }

    #[test]
    fn every_cell_with_a_scene_becomes_a_reference_cell_and_keeps_its_outputs() {
        let h = host("migrate");
        let d = dirs(&h);
        let fitted = d.fitted.clone();
        let p = |s: &str| s.replace("{products}", &h.products.display().to_string()).replace("{assets}", &h.assets.display().to_string());
        // jam2: a copy of its scene as saved now: references it.
        let jam2 = copy_cell(&h, "jam2", "jam2");
        // cell_2robot: its scene changed since the copy (bug_cube → cube1 in the copy).
        let changed = copy_cell(&h, "cell_2robot", "new-scene");
        std::fs::write(fitted.join("new-scene/scene.yaml"), scene_text(&h.products, "bug_cube")).unwrap();
        // trained: a copy whose robot differs from the catalogue's; kr240__weld@trained is taken.
        write(&fitted.join("trained/scene.yaml"), &scene_text(&h.products, "bug_cube"));
        let trained = copy_cell(&h, "trained", "trained");
        std::fs::write(trained.join("robots/kr240__weld/kr240__weld.urdf"), "<robot name='trained'/>").unwrap();
        write(&h.assets.join("urdf-dressed-robots/kr240__weld@trained/kr240__weld.urdf"), "<robot name='other'/>");
        // welding_cell_2_small: umarell's, its own files; welding_cell_2t: a planner cell, its scene
        // and files in umarell's checkout; KrlCell: a KRL cell, no scene.
        let small = small_cell(&h);
        let umarell = h.root.join("umarell/cells/welding_cell_2t");
        write(&umarell.join("scene.yaml"), &format!("{SMALL_SCENE}fixtures:\n- id: geo\n  urdf: ../urdf-fixture-collection/GEO/GEO.urdf\n").replace("fixtures: []\n", ""));
        write(&umarell.join("robots/kr240__weld/kr240__weld.urdf"), "<robot name='policy'/>");
        write(&umarell.join("robots/kr240__weld/meshes/link1.obj"), "o link1");
        write(&umarell.join("robots/kr240__weld/kr240__weld.srdf"), "<robot name='policy srdf'/>");
        write(&umarell.join("fixtures/GEO/GEO.urdf"), "<robot name='geo'/>");
        write(&umarell.join("floorplans/hall/hall.obj"), "o hall");
        let two_t = h.cells.join("welding_cell_2t");
        write(&two_t.join("agentic_planner/cell.yaml"), "robots:\n  - urdf: robots/kr240__weld/kr240__weld.urdf\n");
        let krl = h.cells.join("KrlCell");
        write(&krl.join("cell.yaml"), "special_folgen: {}\n");
        write(&krl.join("io_names.json"), "{}");

        let catalogue_before = tree(&h.assets);
        let products_before = tree(&h.products);
        let dry = run(&d, false);
        let r = |s: &str| p(s);
        assert_eq!(
            decisions(&dry),
            [
                "KrlCell: stays a legacy cell: it has no Robo Fitter scene of its own (scene.yaml), nor has umarell's checkout a cell {root}/umarell/cells/KrlCell (a KRL cell: its 3D scene is its environment directory's, robots by model, which is no Robo Fitter scene)".replace("{root}", &h.root.display().to_string()),
                r("cell_2robot (new-scene): would become a reference cell of cell_2robot; scene scenes/cell_2robot/ from its scene.yaml; robot kr240__weld same as {assets}/urdf-dressed-robots/kr240__weld; product cube1 same as {products}/cube1; fixture table same as {assets}/urdf-fixture-collection/table"),
                r("jam2 (jam2): would become a reference cell of jam2"),
                r("trained (trained): would become a reference cell of trained-2; scene scenes/trained-2/ from its scene.yaml; robot kr240__weld imported as {assets}/urdf-dressed-robots/kr240__weld@trained_2; product bug_cube same as {products}/bug_cube; fixture table same as {assets}/urdf-fixture-collection/table"),
                r("welding_cell_2_small: would become a reference cell of welding_cell_2_small; scene scenes/welding_cell_2_small/ from its scene.yaml; robot kr240__weld imported as {assets}/urdf-dressed-robots/kr240__weld@welding_cell_2_small; floor plan hall.obj imported as {assets}/floor-plans/hall.obj"),
                r("welding_cell_2t: would become a reference cell of welding_cell_2t; scene scenes/welding_cell_2t/ from {root}/umarell/cells/welding_cell_2t/scene.yaml; robot kr240__weld same as {assets}/urdf-dressed-robots/kr240__weld@welding_cell_2_small; floor plan hall.obj same as {assets}/floor-plans/hall.obj; fixture GEO imported as {assets}/urdf-fixture-collection/GEO").replace("{root}", &h.root.display().to_string()),
            ]
        );
        assert_eq!(tree(&h.assets), catalogue_before, "a dry run changes nothing");
        assert_eq!(tree(&h.products), products_before);
        assert!(jam2.join("robots").exists() && small.join("robots").exists() && !h.cells.join(".backup").exists());

        let small_before = std::fs::read(small.join("scene.yaml")).unwrap();
        let applied = run(&d, true);
        for (result, cell) in applied.iter().zip(["KrlCell", "cell_2robot", "jam2", "trained", "welding_cell_2_small", "welding_cell_2t"]) {
            assert_eq!(result.cell, cell);
            if cell == "KrlCell" {
                assert!(matches!(result.decision, Decision::Legacy(_)), "{result}");
                continue;
            }
            let Decision::Migrate { backup: Some(backup), .. } = &result.decision else { panic!("{result}") };
            assert!(backup.starts_with(h.cells.join(".backup")) && backup.is_file(), "{result}");
            let resolved = cell_resolve::resolve(&h.store, cell).unwrap();
            assert_eq!(resolved.kind, CellKind::Reference, "{cell}");
            for asset in resolved.robots.iter().chain(&resolved.fixtures) {
                assert!(asset.urdf.starts_with(&h.assets) || asset.urdf.starts_with(&h.products), "{cell}: {}", asset.urdf.display());
            }
        }
        assert!(krl.join("cell.yaml").is_file() && !krl.join(cell_resolve::MANIFEST).exists(), "the KRL cell untouched");

        // Outputs stay; the inputs' copies go; the SRDF moved into the cell.
        for root in [&jam2, &changed, &trained, &small] {
            for kept in ["agentic_planner/cell.yaml", "agentic_planner/config/robots.yaml", "agentic_planner/tasks/welds.json",
                         "agentic_planner/plans/bt_1.json", "agentic_planner/logs/plantrace_bt_1.json",
                         "plan2cell/builds/bt_1/20261014T1Z/build.json", "plan2cell/auto-restart.json", "agentic_planner/srdf/kr240__weld.srdf"] {
                assert!(root.join(kept).is_file(), "{}: {kept}", root.display());
            }
            for gone in ["robots", "fixtures", "floorplans", "scene.yaml", "scene.factoryos.yaml"] {
                assert!(!root.join(gone).exists(), "{}: {gone}", root.display());
            }
        }
        assert!(!jam2.join("cell.yaml").exists(), "the scaffold cell.yaml goes");
        for own in ["cell.yaml", "io_names.json", "NOTE.md"] {
            assert!(small.join(own).is_file(), "welding_cell_2_small's own {own} stays");
        }
        assert_eq!(std::fs::read_to_string(small.join("agentic_planner/srdf/kr240__weld.srdf")).unwrap(), "<robot name='policy'/>");
        assert_eq!(std::fs::read_to_string(two_t.join("agentic_planner/srdf/kr240__weld.srdf")).unwrap(), "<robot name='policy srdf'/>");
        assert!(umarell.join("robots/kr240__weld/kr240__weld.srdf").is_file(), "umarell's checkout is not changed");

        // welding_cell_2_small: its scene with its robot's new path only; its robot (the trained
        // URDF) and floor plan in the catalogue, without the SRDF.
        // Its paths from the scene's folder, its name the scene's.
        let scene = std::fs::read_to_string(fitted.join("welding_cell_2_small/scene.yaml")).unwrap();
        let expected = SMALL_SCENE
            .replace("../urdf-dressed-robots/", "../../urdf-dressed-robots/")
            .replace("../floor-plans/", "../../floor-plans/")
            .replace("kr240__weld/kr240__weld.urdf", "kr240__weld@welding_cell_2_small/kr240__weld.urdf");
        assert_eq!(scene, format!("name: welding_cell_2_small\n{expected}"));
        let robot = h.assets.join("urdf-dressed-robots/kr240__weld@welding_cell_2_small");
        assert_eq!(std::fs::read_to_string(robot.join("kr240__weld.urdf")).unwrap(), "<robot name='policy'/>");
        assert!(robot.join("meshes/link1.obj").is_file() && !robot.join("kr240__weld.srdf").exists());
        assert_eq!(std::fs::read_to_string(h.assets.join("floor-plans/hall.obj")).unwrap(), "o hall");
        let manifest = cell_resolve::read_manifest(&small).unwrap();
        assert_eq!((manifest.format, manifest.fitted_scene.as_deref()), (2, Some("welding_cell_2_small")));
        let imported = &manifest.extra["imported"];
        assert_eq!(imported["scene_sha256"], sha256_hex(&small_before), "the scene as the cell had it");
        assert_eq!(imported["fitted_sha256"], sha256_hex(scene.as_bytes()));
        assert_eq!(imported["assets"].as_array().unwrap().len(), 2);
        // Its view: the cell's layout again (the variant is `kr240__weld` in it), its SRDF beside the
        // URDF, its own cell.yaml and files, its outputs.
        let view = small.join(cell_resolve::VIEW_DIR);
        let view_scene = std::fs::read_to_string(view.join("scene.yaml")).unwrap();
        assert!(view_scene.contains("dressed_urdf: robots/kr240__weld/kr240__weld.urdf") && view_scene.contains("geometry_file: floorplans/hall/hall.obj"), "{view_scene}");
        assert_eq!(std::fs::read_to_string(view.join("robots/kr240__weld/kr240__weld.urdf")).unwrap(), "<robot name='policy'/>");
        assert_eq!(std::fs::read_to_string(view.join("robots/kr240__weld/kr240__weld.srdf")).unwrap(), "<robot name='policy'/>");
        assert!(view.join("robots/kr240__weld/meshes/link1.obj").is_file() && view.join("floorplans/hall/hall.obj").is_file());
        assert!(std::fs::read_to_string(view.join("cell.yaml")).unwrap().starts_with("# Cell-runner manifest"), "its own cell.yaml");
        for linked in ["io_names.json", "NOTE.md", "plan2cell/builds/bt_1/20261014T1Z/build.json", "agentic_planner/plans/bt_1.json"] {
            assert!(view.join(linked).is_file(), "{linked}");
        }
        // The prepared model's paths are the cell's own (now the imports): prepared from them.
        assert_eq!(cell_resolve::prepared_inputs(&small), Some(cell_resolve::resolve(&h.store, "welding_cell_2_small").unwrap().inputs));
        // jam2 keeps its scene; its prepared model too.
        assert_eq!(cell_resolve::read_manifest(&jam2).unwrap().fitted_scene.as_deref(), Some("jam2"));
        // cell_2robot: its own scene (cube1), the products in place; new-scene is the user's.
        let scene = std::fs::read_to_string(fitted.join("cell_2robot/scene.yaml")).unwrap();
        assert!(scene.contains(&format!("urdf: {}/cube1/cube1.urdf", h.products.display())), "{scene}");
        assert!(scene.contains("dressed_urdf: ../../urdf-dressed-robots/kr240__weld/kr240__weld.urdf"), "{scene}");
        assert!(scene.starts_with("name: cell_2robot\n"), "{scene}");
        assert_eq!(cell_resolve::read_manifest(&changed).unwrap().extra["migrated_from"], "new-scene");
        // trained: kr240__weld@trained was other content: @trained_2; its scene name taken: trained-2.
        assert_eq!(std::fs::read_to_string(h.assets.join("urdf-dressed-robots/kr240__weld@trained_2/kr240__weld.urdf")).unwrap(), "<robot name='trained'/>");
        assert!(std::fs::read_to_string(fitted.join("trained-2/scene.yaml")).unwrap().contains("kr240__weld@trained_2/kr240__weld.urdf"));
        assert_eq!(std::fs::read_to_string(fitted.join("trained/scene.yaml")).unwrap(), scene_text(&h.products, "bug_cube"), "the user's scene untouched");
        // welding_cell_2t: its robot is welding_cell_2_small's, the fixture imported.
        assert!(h.assets.join("urdf-fixture-collection/GEO/GEO.urdf").is_file());
        assert!(!h.assets.join("urdf-dressed-robots/kr240__weld@welding_cell_2t").exists(), "the same robot, imported once");
        // The backup has the whole old cell.
        let Decision::Migrate { backup: Some(backup), .. } = &applied[4].decision else { panic!() };
        let listing = std::process::Command::new("tar").arg("-tf").arg(backup).output().unwrap();
        let listing = String::from_utf8_lossy(&listing.stdout);
        assert!(listing.contains("welding_cell_2_small/robots/kr240__weld/kr240__weld.urdf") && listing.contains("welding_cell_2_small/scene.yaml"), "{listing}");

        // Idempotent: reference cells now, nothing imported again; .backup is no cell.
        let catalogue = tree(&h.assets);
        let again = run(&d, true);
        assert_eq!(again.len(), 6);
        assert!(again.iter().all(|r| matches!(r.decision, Decision::Reference) || r.cell == "KrlCell"), "{:?}", decisions(&again));
        assert_eq!(tree(&h.assets), catalogue);
    }

    #[test]
    fn a_failed_migration_restores_the_cell_and_drops_its_imports() {
        let h = host("migrate-fail");
        let d = dirs(&h);
        let small = small_cell(&h);
        let before = tree(&small);
        let catalogue = tree(&h.assets);
        let mut plan = match decide(&small, &d, &mut Pending::default()) {
            Decision::Migrate { plan, .. } => plan,
            other => panic!("{other:?}"),
        };
        // The robot's import does not hold what the cell had (e.g. a disk filling up mid-copy).
        plan.imports[0].hash = "not it".to_owned();
        let err = migrate(&d, &small, &plan).unwrap_err();
        assert!(format!("{err:#}").contains("does not hold what the cell had") && format!("{err:#}").contains("restored from"), "{err:#}");
        assert_eq!(tree(&small), before, "the cell as it was");
        assert_eq!(tree(&h.assets), catalogue, "its imports and scene removed");
        assert!(cell_resolve::resolve(&h.store, "welding_cell_2_small").unwrap().kind == CellKind::Legacy);
    }
}
