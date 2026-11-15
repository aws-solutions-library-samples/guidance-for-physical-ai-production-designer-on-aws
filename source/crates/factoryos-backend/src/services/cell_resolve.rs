//! Cells reference, never copy: the single source of truth for what a cell's inputs are.
//!
//! A cell is a folder for its outputs (the prepared planner model `agentic_planner/cell.yaml` +
//! `config/`, task programs, plans, `plan2cell/` exports and deploys, logs) plus a manifest,
//! `factoryos-cell.json`, that points at its Robo Fitter scene:
//!
//! ```json
//! {"format": 2, "kind": "reference", "fitted_scene": "jam2", "cell_id": "jam2"}
//! ```
//!
//! `fitted_scene` is the scene's name (its folder `scenes/<name>/scene.yaml`, see
//! [`super::scene_store`]); a legacy `jam2.yaml` reads as `jam2` (and the start's migration
//! rewrites it).
//!
//! Robots and products are read where they are: the catalogue's `urdf-dressed-robots/`,
//! `urdf-fixture-collection/` and the products folder, as the scene names them. [`resolve`] says
//! where everything is ([`ResolvedCell`]) and hashes it ([`InputHashes`]); every output records the
//! hashes it was built from and is stale ([`staleness`]) once they no longer match. Nothing is
//! copied, rebuilt or deleted when the scene is saved or a robot re-dressed.
//!
//! Legacy cells (format 1 `{"fitted_scene": …}` copies made by the old Create Cell, umarell's
//! reference cell `welding_cell_2_small`, the KRL sample cells) keep working unchanged: a
//! `scene.yaml` (or `scene.factoryos.yaml`) at the cell's root wins, and their files are the cell's.
//!
//! Consumers that need the files laid out under one folder (the cell-runner's `cellDir`/`envDir`
//! and its `/env`, fos-live-scene, the planner router, Docker runs) use [`materialise_view`]: a
//! disposable `<cell>/.view/` of symlinks to the catalogue's and products' packages, a generated
//! `scene.yaml` with view-relative paths and a scaffold `cell.yaml`.
//!
//! Hashes (lowercase hex sha256): a file's is that of its bytes; a package's (the folder of a
//! robot's or fixture's URDF) is the sha256 of the lines `<relative path>\t<file hash>\n` of its
//! files sorted by path, skipping `meshes_lowres/`, `*.srdf` (Prepare Cell's output, which moved
//! into `agentic_planner/srdf/`) and hidden files; a robot's is its package's, or, when its tool
//! or its robot model has planner sidecars (`urdf-tool-collection/<tool>/planner.yaml`,
//! `urdf-robot-collection/<model>/planner.yaml`, [`robot_hash`]), the sha256 of `package\t<h>\n`, a
//! `sidecar\t<tool>\t<file hash>\n` line per tool sidecar and a `robot_sidecar\t<model>\t<file hash>\n`
//! line per robot model sidecar; `combined` is the sha256 of the lines
//! `scene\t<h>\n`, `robot\t<id>\t<h>\n` (by id), `fixture\t<id>\t<h>\n` (by id) and `usda\t<h>\n`.
//! File hashes are cached by (path, mtime, size).

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

use anyhow::Context as _;
use sha2::{Digest as _, Sha256};

use super::tcp::ToolCatalogues;
use crate::config_store::ConfigStore;

pub use factoryos_protocol::CellKind;

/// The cell's manifest at its root.
pub const MANIFEST: &str = "factoryos-cell.json";
/// The manifest format of a reference cell.
pub const MANIFEST_FORMAT: u32 = 2;
/// The resolved view, at the cell's root: disposable (rebuilt when the inputs change), not backed up.
pub const VIEW_DIR: &str = ".view";
/// Where [`materialise_view`] extracts the scene's USDA bundle, inside the view.
pub const VIEW_USDA: &str = "usda";
/// The inputs the prepared planner model was made from ([`InputHashes`] as JSON).
pub const PREPARED_INPUTS: &str = "agentic_planner/inputs.json";
/// Where Prepare Cell writes the robots' SRDFs of a reference cell (never into the catalogue).
pub const SRDF_DIR: &str = "agentic_planner/srdf";
/// The view's stamp: the [`InputHashes::combined`] it was built for.
const VIEW_STAMP: &str = ".inputs";
/// What the view links besides its inputs ([`view_links`], one per line), when it was built.
const VIEW_OWN: &str = ".own";

/// A cell, resolved: where its scene and each robot's and fixture's files are.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedCell {
    /// The folder name.
    pub name: String,
    /// The cell's folder (its outputs).
    pub root: PathBuf,
    pub kind: CellKind,
    /// The scene: `scenes/<scene>/scene.yaml`, or `<root>/scene.yaml` for a legacy cell (which may
    /// not exist for a cell that has none, e.g. a KRL-only cell).
    pub scene_path: PathBuf,
    /// Where the scene's relative paths start: `scene_path.parent()`.
    pub scene_base: PathBuf,
    /// The scene (its name) the cell references (reference) or was copied from (legacy), if any.
    pub fitted_scene: Option<String>,
    /// Absolute URDF paths, in scene order.
    pub robots: Vec<ResolvedAsset>,
    /// Absolute URDF paths and their `product.json`, in scene order.
    pub fixtures: Vec<ResolvedAsset>,
    /// The scene's USDA bundle (Isaac scene), if there is one.
    pub usda_zip: Option<PathBuf>,
    pub inputs: InputHashes,
}

/// One robot or fixture of a [`ResolvedCell`].
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedAsset {
    /// `robot_id` / fixture `id` in the scene.
    pub scene_id: String,
    pub urdf: PathBuf,
    /// The fixture's `product.json` beside its URDF, if it is a product.
    pub product_json: Option<PathBuf>,
}

/// What an output was made from (see the module docs for how they are computed). Recorded as
/// `agentic_planner/inputs.json`, a plan's `planner_meta.inputs` and an export build's `build.json`.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct InputHashes {
    pub scene: String,
    /// By robot `robot_id`.
    pub urdfs: BTreeMap<String, String>,
    /// By fixture `id`.
    pub fixtures: BTreeMap<String, String>,
    pub usda: Option<String>,
    pub combined: String,
}

impl InputHashes {
    fn combine(&mut self) {
        let mut text = format!("scene\t{}\n", self.scene);
        for (id, hash) in &self.urdfs {
            text += &format!("robot\t{id}\t{hash}\n");
        }
        for (id, hash) in &self.fixtures {
            text += &format!("fixture\t{id}\t{hash}\n");
        }
        if let Some(usda) = &self.usda {
            text += &format!("usda\t{usda}\n");
        }
        self.combined = hex(&Sha256::digest(text.as_bytes()));
    }
}

/// `factoryos-cell.json`. Format 1 (`{"fitted_scene": …}`, maybe `rebuilt_s`) is a copied cell.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Manifest {
    #[serde(default = "format_one")]
    pub format: u32,
    /// `reference` (format 2) or `copy` (format 1).
    #[serde(default)]
    pub kind: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fitted_scene: Option<String>,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub cell_id: String,
    /// Anything else (format 1's `rebuilt_s`, a migration's `migrated_from`), kept.
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// The parts of a Robo Fitter scene the resolver needs (lenient: other keys and missing poses are
/// fine).
#[derive(serde::Deserialize)]
struct SceneAssets {
    #[serde(default)]
    robots: Vec<SceneRobot>,
    #[serde(default)]
    fixtures: Vec<SceneFixture>,
}

#[derive(serde::Deserialize)]
struct SceneRobot {
    #[serde(default)]
    robot_id: String,
    #[serde(default)]
    dressed_urdf: String,
}

#[derive(serde::Deserialize)]
struct SceneFixture {
    #[serde(default)]
    id: String,
    #[serde(default)]
    urdf: String,
}

fn format_one() -> u32 {
    1
}

impl Manifest {
    pub fn reference(scene: &str, cell: &str) -> Self {
        Self {
            format: MANIFEST_FORMAT,
            kind: "reference".to_owned(),
            fitted_scene: Some(scene.to_owned()),
            cell_id: super::cells::planner_cell_id(cell),
            extra: serde_json::Map::new(),
        }
    }

    pub fn is_reference(&self) -> bool {
        self.format >= MANIFEST_FORMAT && self.kind != "copy"
    }
}

/// The manifest of cell `root`, if it has one that parses.
pub fn read_manifest(root: &Path) -> Option<Manifest> {
    serde_json::from_slice(&std::fs::read(root.join(MANIFEST)).ok()?).ok()
}

pub fn write_manifest(root: &Path, manifest: &Manifest) -> anyhow::Result<()> {
    let text = serde_json::to_string_pretty(manifest)?;
    std::fs::write(root.join(MANIFEST), text + "\n").with_context(|| format!("writing {}", root.join(MANIFEST).display()))?;
    crate::storage::saved(root.join(MANIFEST));
    Ok(())
}

/// Cell `cell` (a folder in the cells folder, or the active cell by name), resolved.
pub fn resolve(cfg: &ConfigStore, cell: &str) -> anyhow::Result<ResolvedCell> {
    let root = cell_root(cfg, cell)?;
    resolve_root(&root, &super::cells::fitted_scenes_dir(cfg))
}

/// The folder of cell `cell`: in the cells folder, else the active cell if it is named so.
fn cell_root(cfg: &ConfigStore, cell: &str) -> anyhow::Result<PathBuf> {
    anyhow::ensure!(
        !cell.is_empty() && !cell.contains(['/', '\\']) && !cell.starts_with('.'),
        "not a cell name: {cell:?}"
    );
    if let Some(dir) = super::cells::cells_folder(cfg).map(|dir| dir.join(cell))
        && dir.is_dir()
    {
        return Ok(dir);
    }
    match super::cells::active_cell(cfg) {
        Some((name, root)) if name == cell => Ok(root),
        _ => anyhow::bail!("no cell {cell}"),
    }
}

/// The cell in folder `root`, resolved; `fitted_dir` is the Robo Fitter's `scenes/`.
pub fn resolve_root(root: &Path, fitted_dir: &Path) -> anyhow::Result<ResolvedCell> {
    let name = root.file_name().unwrap_or_default().to_string_lossy().into_owned();
    let manifest = read_manifest(root);
    let own_scene = [super::cells::SCENE_MARKER, super::cells::FACTORY_OS_SCENE]
        .into_iter()
        .map(|file| root.join(file))
        .find(|path| path.is_file());
    let named = manifest.as_ref().and_then(|m| m.fitted_scene.clone());
    let fitted_scene = named.as_deref().map(|scene| super::scene_store::name_of(scene).unwrap_or_else(|| scene.to_owned()));
    let reference = own_scene.is_none() && fitted_scene.is_some();
    let (kind, scene_path) = if reference {
        let scene = named.as_deref().unwrap_or_default();
        let scene = super::scene_store::name_of(scene)
            .ok_or_else(|| anyhow::anyhow!("cell {name}'s manifest names an invalid scene: {scene:?}"))?;
        let path = super::scene_store::scene_file(fitted_dir, &scene);
        anyhow::ensure!(path.is_file(), "cell {name} references the Robo Fitter scene {}, which is gone", path.display());
        (CellKind::Reference, path)
    } else {
        (CellKind::Legacy, own_scene.clone().unwrap_or_else(|| root.join(super::cells::SCENE_MARKER)))
    };
    let scene_base = scene_path.parent().map_or_else(|| root.to_path_buf(), Path::to_path_buf);

    // The robots and fixtures: a legacy umarell cell's `scene.yaml` is the planner's format, its
    // `scene.factoryos.yaml` the Robo Fitter's.
    let scene_files: Vec<PathBuf> = match kind {
        CellKind::Reference => vec![scene_path.clone()],
        CellKind::Legacy => [super::cells::FACTORY_OS_SCENE, super::cells::SCENE_MARKER].iter().map(|f| root.join(f)).collect(),
    };
    let scene = scene_files
        .iter()
        .find_map(|path| serde_yaml::from_str::<SceneAssets>(&std::fs::read_to_string(path).ok()?).ok());
    let at = |value: &str| {
        let path = Path::new(value);
        normalize(&if path.is_absolute() { path.to_path_buf() } else { super::scene_store::asset_path(&scene_base, value) })
    };
    let (mut robots, mut fixtures) = (Vec::new(), Vec::new());
    if let Some(scene) = &scene {
        for robot in scene.robots.iter().filter(|r| !r.dressed_urdf.trim().is_empty()) {
            robots.push(ResolvedAsset { scene_id: robot.robot_id.clone(), urdf: at(&robot.dressed_urdf), product_json: None });
        }
        for fixture in scene.fixtures.iter().filter(|f| !f.urdf.trim().is_empty()) {
            let urdf = at(&fixture.urdf);
            let product = urdf.parent().map(|dir| dir.join("product.json")).filter(|p| p.is_file());
            fixtures.push(ResolvedAsset { scene_id: fixture.id.clone(), urdf, product_json: product });
        }
    }
    let usda_zip = match kind {
        CellKind::Reference => super::cells::find_usda_bundle(fitted_dir, fitted_scene.as_deref().unwrap_or_default()),
        CellKind::Legacy => super::cells::root_zips(root).into_iter().find(|zip| super::cells::has_usda(zip)),
    };

    let mut inputs = InputHashes {
        scene: if scene_path.is_file() { file_hash(&scene_path) } else { String::new() },
        urdfs: robots.iter().map(|r| (r.scene_id.clone(), robot_hash(&r.urdf, &tool_catalogues(&r.urdf, fitted_dir)))).collect(),
        fixtures: fixtures.iter().map(|f| (f.scene_id.clone(), package_hash(&f.urdf))).collect(),
        usda: usda_zip.as_deref().map(file_hash),
        combined: String::new(),
    };
    inputs.combine();
    Ok(ResolvedCell { name, root: root.to_path_buf(), kind, scene_path, scene_base, fitted_scene, robots, fixtures, usda_zip, inputs })
}

/// The cell of fitted scene `scene`: a cell folder that references it (or was copied from it),
/// reference cells first, then one named after the scene; else the name a new one gets (the scene's
/// name cleaned to letters, digits and `_`, made unique).
pub fn cell_for_scene(cfg: &ConfigStore, scene: &str) -> anyhow::Result<String> {
    let scene = super::scene_store::name_of(scene).ok_or_else(|| anyhow::anyhow!("not a scene name: {scene:?}"))?;
    let scene = scene.as_str();
    let cells_dir = super::cells::cells_folder(cfg).context("set the cells folder (Configuration → Cell Runner) first")?;
    Ok(cell_for_scene_in(&cells_dir, scene))
}

fn cell_for_scene_in(cells_dir: &Path, scene: &str) -> String {
    let base = clean_name(scene);
    let mut made: Vec<(bool, String)> = super::cells::list_cells(cells_dir)
        .into_iter()
        .filter_map(|cell| {
            let manifest = read_manifest(&cells_dir.join(&cell))?;
            names_scene(&manifest, scene).then(|| (manifest.is_reference(), cell))
        })
        .collect();
    // Reference cells first, then the one named after the scene, then by name.
    made.sort_by_key(|(reference, cell)| (!reference, *cell != base, cell.clone()));
    if let Some((_, cell)) = made.into_iter().next() {
        return cell;
    }
    (1..)
        .map(|i| if i == 1 { base.clone() } else { format!("{base}_{i}") })
        .find(|name| {
            let dir = cells_dir.join(name);
            !dir.exists() || std::fs::read_dir(&dir).is_ok_and(|mut entries| entries.next().is_none())
        })
        .expect("unbounded")
}

/// Whether `manifest` names scene `scene` (a name; the manifest's may be a legacy `<name>.yaml`).
pub fn names_scene(manifest: &Manifest, scene: &str) -> bool {
    manifest.fitted_scene.as_deref().and_then(super::scene_store::name_of).is_some_and(|name| name == scene)
}

/// A cell name from a scene name (or a legacy `<name>.yaml`): letters, digits and `_` (it becomes
/// a ROS namespace).
pub fn clean_name(scene: &str) -> String {
    let stem = super::scene_store::name_of(scene).unwrap_or_else(|| scene.to_owned());
    let name: String = stem.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
    let name = name.trim_matches('_').to_owned();
    if name.is_empty() { "cell".to_owned() } else { name }
}

/// Serialises creating cells and building views (several plan jobs may ask at once).
fn lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The cell of fitted scene `scene` ([`cell_for_scene`]), made if there is none: only its folder
/// and manifest are written. Idempotent.
pub fn ensure_cell(cfg: &ConfigStore, scene: &str) -> anyhow::Result<ResolvedCell> {
    ensure_cell_named(cfg, scene, None)
}

/// [`ensure_cell`], naming a new cell `name` (letters, digits, `_`) instead of after the scene. A
/// cell of the scene that exists already is used whatever `name` says.
pub fn ensure_cell_named(cfg: &ConfigStore, scene: &str, name: Option<&str>) -> anyhow::Result<ResolvedCell> {
    let scene = super::scene_store::name_of(scene).ok_or_else(|| anyhow::anyhow!("not a scene name: {scene:?}"))?;
    let scene = scene.as_str();
    let fitted_dir = super::cells::fitted_scenes_dir(cfg);
    anyhow::ensure!(super::scene_store::exists(&fitted_dir, scene), "no Robo Fitter scene {scene} in {}", fitted_dir.display());
    let cells_dir = super::cells::cells_folder(cfg).context("set the cells folder (Configuration → Cell Runner) first")?;
    let _guard = lock();
    ensure_in(&cells_dir, &fitted_dir, scene, name)
}

fn ensure_in(cells_dir: &Path, fitted_dir: &Path, scene: &str, name: Option<&str>) -> anyhow::Result<ResolvedCell> {
    let existing = cell_for_scene_in(cells_dir, scene);
    let made = read_manifest(&cells_dir.join(&existing)).is_some_and(|m| names_scene(&m, scene));
    let cell = match name.map(str::trim).filter(|n| !n.is_empty()) {
        Some(name) if !made => {
            anyhow::ensure!(
                name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'),
                "use only letters, digits and underscores for the cell name (it becomes a ROS namespace)"
            );
            let dir = cells_dir.join(name);
            if dir.exists() && std::fs::read_dir(&dir).is_ok_and(|mut e| e.next().is_some()) {
                match read_manifest(&dir).and_then(|m| m.fitted_scene) {
                    Some(source) => anyhow::bail!("a cell named {name} already exists (made from {source}): pick another name"),
                    None => anyhow::bail!("a cell named {name} already exists: pick another name"),
                }
            }
            name.to_owned()
        }
        _ => existing,
    };
    let root = cells_dir.join(&cell);
    if read_manifest(&root).is_none() {
        std::fs::create_dir_all(&root).with_context(|| format!("creating {}", root.display()))?;
        write_manifest(&root, &Manifest::reference(scene, &cell))?;
        tracing::info!("Cell {cell}: made for the Robo Fitter scene {scene} (it references the scene; nothing copied)");
        // A new cell's folder (the manifest, and whatever was in an empty one) to the user's store.
        crate::storage::saved_tree(&root);
    }
    resolve_root(&root, fitted_dir)
}

/// The cell's files laid out under one folder, for consumers that need that: a legacy cell's own
/// folder, or `<root>/.view/` for a reference cell, rebuilt when its inputs (or its own files)
/// changed: `scene.yaml` (view-relative paths), `robots/<pkg>`, `fixtures/<pkg>` and
/// `floorplans/<stem>/<file>` symlinked to the originals (a robot package whose SRDF is the cell's,
/// `agentic_planner/srdf/<urdf stem>.srdf`, is a folder of links with that SRDF beside the URDF),
/// `agentic_planner` → `../agentic_planner`, the cell's own other files linked the same way (a
/// migrated cell's `cell.yaml`, `io_names.json`, `krl/`, `plan2cell/`, …; without its own
/// `cell.yaml`, a scaffold one) and the USDA bundle extracted into `usda/`.
pub fn materialise_view(r: &ResolvedCell) -> anyhow::Result<PathBuf> {
    if r.kind == CellKind::Legacy {
        return Ok(r.root.clone());
    }
    let view = r.root.join(VIEW_DIR);
    let _guard = lock();
    if std::fs::read_to_string(view.join(VIEW_STAMP)).is_ok_and(|stamp| stamp.trim() == r.inputs.combined)
        && std::fs::read_to_string(view.join(VIEW_OWN)).unwrap_or_default() == view_links(&r.root)
    {
        return Ok(view);
    }
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let fresh = r.root.join(format!("{VIEW_DIR}.tmp-{}-{seq}", std::process::id()));
    super::cells::remove_path(&fresh).ok();
    // Swap by two renames (the old view is moved aside first, removed after): consumers reading
    // through `.view` (a running simulation, the router's per-job views linking into it) see it
    // missing for the instant between them only, not for a whole removal.
    let old = r.root.join(format!("{VIEW_DIR}.tmp-{}-{seq}-old", std::process::id()));
    let result = build_view(r, &fresh).and_then(|()| {
        if std::fs::symlink_metadata(&view).is_ok() {
            std::fs::rename(&view, &old)?;
        }
        if let Err(err) = std::fs::rename(&fresh, &view) {
            std::fs::rename(&old, &view).ok();
            return Err(err.into());
        }
        super::cells::remove_path(&old).ok();
        Ok(())
    });
    if result.is_err() {
        super::cells::remove_path(&fresh).ok();
    }
    result.with_context(|| format!("building the view of cell {}", r.name))?;
    Ok(view)
}

/// The names in a view the view makes itself (a cell's own entry of one of these names is not linked).
const VIEW_NAMES: [&str; 8] = [
    super::cells::SCENE_MARKER,
    super::cells::FACTORY_OS_SCENE,
    "robots",
    "fixtures",
    "floorplans",
    VIEW_USDA,
    super::cells::USDA_EXTRACTED,
    "agentic_planner",
];

/// The cell's own entries a view links (`../<name>`): not hidden, not the manifest, not a name the
/// view makes ([`VIEW_NAMES`]). Sorted.
pub(crate) fn own_entries(root: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(root)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| !name.starts_with('.') && name != MANIFEST && !VIEW_NAMES.contains(&name.as_str()))
        .collect();
    names.sort();
    names
}

/// What a view links besides its inputs, one per line: the cell's own entries and its SRDFs
/// (`agentic_planner/srdf/<name>`, linked beside their URDFs; not in the inputs' hashes, so a view
/// built before Prepare Cell made them is rebuilt once they are there).
fn view_links(root: &Path) -> String {
    let mut srdfs: Vec<String> = std::fs::read_dir(root.join(SRDF_DIR))
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| !name.starts_with('.') && name.to_lowercase().ends_with(".srdf"))
        .map(|name| format!("{SRDF_DIR}/{name}"))
        .collect();
    srdfs.sort();
    let mut lines = own_entries(root);
    lines.extend(srdfs);
    lines.join("\n")
}

fn build_view(r: &ResolvedCell, dir: &Path) -> anyhow::Result<()> {
    let text = std::fs::read_to_string(&r.scene_path).with_context(|| format!("reading {}", r.scene_path.display()))?;
    let plan = super::cells::plan_cell_scaffold(&text);
    std::fs::create_dir_all(dir)?;
    for copy in &plan.copies {
        let from = normalize(&super::scene_store::asset_path(&r.scene_base, &copy.from_rel));
        anyhow::ensure!(from.exists(), "referenced asset not found: {}", from.display());
        // The link stays inside the view (a scene path like `x/../..` names no package).
        anyhow::ensure!(
            Path::new(&copy.to_rel).components().all(|c| matches!(c, std::path::Component::Normal(_))),
            "asset path {:?} does not name a package",
            copy.from_rel
        );
        let to = dir.join(&copy.to_rel);
        if let Some(parent) = to.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let srdfs = cell_srdfs(r, &from);
        if copy.kind == super::cells::CopyKind::Dir && !srdfs.is_empty() {
            // The package with the cell's SRDF beside its URDF (where the converter and the
            // cell-runner look, and where a prepared model's `srdf: robots/<pkg>/<pkg>.srdf` points).
            std::fs::create_dir_all(&to)?;
            for entry in std::fs::read_dir(&from)?.flatten() {
                symlink(&entry.path(), &to.join(entry.file_name()))?;
            }
            for (name, srdf) in srdfs {
                symlink(&srdf, &to.join(name))?;
            }
        } else {
            symlink(&from, &to)?;
        }
        let mtl = from.with_extension("mtl");
        if copy.kind == super::cells::CopyKind::File && mtl.is_file() {
            symlink(&mtl, &to.with_extension("mtl"))?;
        }
    }
    std::fs::write(dir.join(super::cells::SCENE_MARKER), &plan.scene)?;
    let own = own_entries(&r.root);
    if !own.iter().any(|name| name == super::cells::CELL_MARKER) {
        std::fs::write(
            dir.join(super::cells::CELL_MARKER),
            super::cells::build_scaffold_cell_yaml(&plan.scene, r.fitted_scene.as_deref().unwrap_or_default()),
        )?;
    }
    for name in &own {
        symlink(&Path::new("..").join(name), &dir.join(name))?;
    }
    std::fs::create_dir_all(r.root.join("agentic_planner"))?;
    symlink(Path::new("../agentic_planner"), &dir.join("agentic_planner"))?;
    if let Some(zip) = &r.usda_zip {
        super::cells::extract_zip(zip, &dir.join(VIEW_USDA))?;
    }
    std::fs::write(dir.join(VIEW_OWN), view_links(&r.root))?;
    std::fs::write(dir.join(VIEW_STAMP), &r.inputs.combined)?;
    Ok(())
}

/// The cell's SRDFs (`agentic_planner/srdf/<urdf stem>.srdf`) of the robots whose URDF is in package
/// `package` and that have none there: `(file name, SRDF)`.
fn cell_srdfs(r: &ResolvedCell, package: &Path) -> Vec<(String, PathBuf)> {
    let mut out: Vec<(String, PathBuf)> = Vec::new();
    for robot in r.robots.iter().filter(|robot| robot.urdf.parent() == Some(package)) {
        let Some(stem) = robot.urdf.file_stem().map(|s| s.to_string_lossy().into_owned()) else { continue };
        let name = format!("{stem}.srdf");
        let srdf = r.root.join(SRDF_DIR).join(&name);
        if srdf.is_file() && !package.join(&name).exists() && !out.iter().any(|(n, _)| *n == name) {
            out.push((name, srdf));
        }
    }
    out
}

#[cfg(unix)]
fn symlink(from: &Path, to: &Path) -> std::io::Result<()> {
    std::os::unix::fs::symlink(from, to)
}

#[cfg(not(unix))]
fn symlink(from: &Path, to: &Path) -> std::io::Result<()> {
    let from = if from.is_relative() { to.parent().unwrap_or(Path::new(".")).join(from) } else { from.to_path_buf() };
    if from.is_dir() { super::cells::copy_dir(&from, to).map_err(std::io::Error::other) } else { std::fs::copy(from, to).map(drop) }
}

/// Why an output made from `recorded` is stale for the cell as it is now, if it is. `None` too
/// when nothing was recorded (an output older than input hashes: unknown, not stale).
pub fn staleness(r: &ResolvedCell, recorded: &InputHashes) -> Option<String> {
    let now = &r.inputs;
    if recorded.combined.is_empty() || recorded.combined == now.combined {
        return None;
    }
    let mut reasons = Vec::new();
    if recorded.scene != now.scene {
        reasons.push(match &r.fitted_scene {
            Some(scene) if r.kind == CellKind::Reference => format!("the scene {scene} changed"),
            _ => "the cell's scene changed".to_owned(),
        });
    }
    let mut compare = |what: &str, changed: &str, before: &BTreeMap<String, String>, after: &BTreeMap<String, String>| {
        for (id, hash) in after {
            match before.get(id) {
                None => reasons.push(format!("{what} {id} was added")),
                Some(old) if old != hash => reasons.push(format!("{what} {id}'s {changed} changed")),
                Some(_) => {}
            }
        }
        for id in before.keys().filter(|id| !after.contains_key(*id)) {
            reasons.push(format!("{what} {id} was removed"));
        }
    };
    compare("robot", "URDF", &recorded.urdfs, &now.urdfs);
    compare("fixture", "files", &recorded.fixtures, &now.fixtures);
    if recorded.usda != now.usda {
        reasons.push("the USDA bundle changed".to_owned());
    }
    if reasons.is_empty() {
        reasons.push("its inputs changed".to_owned());
    }
    Some(reasons.join("; "))
}

/// The inputs the cell's prepared planner model was made from ([`PREPARED_INPUTS`]), if recorded.
pub fn prepared_inputs(root: &Path) -> Option<InputHashes> {
    serde_json::from_slice(&std::fs::read(root.join(PREPARED_INPUTS)).ok()?).ok()
}

/// Record that the cell's planner model was prepared from its inputs as they are now.
pub fn record_prepared(r: &ResolvedCell) -> anyhow::Result<()> {
    let path = r.root.join(PREPARED_INPUTS);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, serde_json::to_string_pretty(&r.inputs)? + "\n").with_context(|| format!("writing {}", path.display()))?;
    crate::storage::saved(&path);
    Ok(())
}

/// Why the cell's prepared planner model is stale, if it is prepared and recorded its inputs.
pub fn prepared_staleness(r: &ResolvedCell) -> Option<String> {
    if !r.root.join("agentic_planner/cell.yaml").is_file() {
        return None;
    }
    staleness(r, &prepared_inputs(&r.root)?)
}

/// The inputs a saved plan (its JSON) records in `planner_meta.inputs` (or `inputs`), if any.
pub fn plan_inputs(plan: &serde_json::Value) -> Option<InputHashes> {
    let inputs = plan.get("planner_meta").and_then(|meta| meta.get("inputs")).or_else(|| plan.get("inputs"))?;
    serde_json::from_value(inputs.clone()).ok()
}

// --- Hashing ---

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

type HashCache = HashMap<PathBuf, (std::time::SystemTime, u64, String)>;

fn cache() -> &'static Mutex<HashCache> {
    static CACHE: OnceLock<Mutex<HashCache>> = OnceLock::new();
    CACHE.get_or_init(Mutex::default)
}

/// The sha256 of file `path` (cached by path, mtime and size); empty if it can't be read.
pub fn file_hash(path: &Path) -> String {
    let Ok(meta) = std::fs::metadata(path) else { return String::new() };
    let (modified, len) = (meta.modified().unwrap_or(std::time::UNIX_EPOCH), meta.len());
    if let Some((m, l, hash)) = cache().lock().unwrap_or_else(std::sync::PoisonError::into_inner).get(path)
        && (*m, *l) == (modified, len)
    {
        return hash.clone();
    }
    let Ok(mut file) = std::fs::File::open(path) else { return String::new() };
    let mut hasher = Sha256::new();
    if std::io::copy(&mut file, &mut hasher).is_err() {
        return String::new();
    }
    let hash = hex(&hasher.finalize());
    cache()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .insert(path.to_path_buf(), (modified, len, hash.clone()));
    hash
}

/// The hash of the package (folder) of `urdf`: see the module docs. Empty if the URDF is missing.
pub fn package_hash(urdf: &Path) -> String {
    if !urdf.is_file() {
        return String::new();
    }
    let Some(dir) = urdf.parent() else { return file_hash(urdf) };
    // A URDF at a cell's root (a legacy cell's own files): its package would be the cell with its
    // outputs (plans, builds), which change all the time; the URDF alone counts.
    if dir.join(MANIFEST).exists() || dir.join("agentic_planner").exists() || dir.join(super::cells::SCENE_MARKER).exists() {
        return file_hash(urdf);
    }
    let mut files = Vec::new();
    walk(dir, Path::new(""), &mut files, 0);
    files.sort();
    let mut text = String::new();
    for rel in files {
        text += &format!("{}\t{}\n", rel.display(), file_hash(&dir.join(&rel)));
    }
    hex(&Sha256::digest(text.as_bytes()))
}

/// The catalogues a robot's tool sidecar may come from (as Prepare Cell looks them up, see
/// [`super::planner_model::read_scene`]): the dressed robot's own catalogue (when it is in one's
/// `urdf-dressed-robots/`), the Robo Fitter's (`fitted_dir`'s parent) and the release's.
fn tool_catalogues(urdf: &Path, fitted_dir: &Path) -> ToolCatalogues {
    let own = urdf
        .parent()
        .and_then(Path::parent)
        .filter(|dir| dir.file_name().is_some_and(|n| n == "urdf-dressed-robots"))
        .and_then(Path::parent)
        .map(Path::to_path_buf);
    let fitter = fitted_dir.parent().map(Path::to_path_buf);
    ToolCatalogues::new(own.into_iter().chain(fitter).chain([PathBuf::from(crate::paths::default_catalogue())]))
}

/// The hash of a dressed robot: its [`package_hash`], with its tool's and its robot model's
/// planner sidecars (any of `tools`' catalogues) when there are any, so editing one makes the
/// cells using the tool / the model stale.
pub fn robot_hash(urdf: &Path, tools: &ToolCatalogues) -> String {
    let package = package_hash(urdf);
    let combo = urdf.file_stem().unwrap_or_default().to_string_lossy().into_owned();
    let sidecars = tools.tool_sidecars(&combo);
    let models = tools.robot_sidecars(&combo);
    if package.is_empty() || (sidecars.is_empty() && models.is_empty()) {
        return package;
    }
    let mut text = format!("package\t{package}\n");
    for (tool, path) in sidecars {
        text += &format!("sidecar\t{tool}\t{}\n", file_hash(&path));
    }
    for (model, path) in models {
        text += &format!("robot_sidecar\t{model}\t{}\n", file_hash(&path));
    }
    hex(&Sha256::digest(text.as_bytes()))
}

/// How deep [`walk`] follows a package's folders (it follows symlinks: a link cycle ends here).
const MAX_DEPTH: usize = 12;

fn walk(dir: &Path, rel: &Path, out: &mut Vec<PathBuf>, depth: usize) {
    if depth > MAX_DEPTH {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir.join(rel)) else { return };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with('.') || (rel.as_os_str().is_empty() && name == "meshes_lowres") {
            continue;
        }
        let path = rel.join(&name);
        // Follows symlinks (a view's or a product's linked meshes).
        let Ok(meta) = std::fs::metadata(entry.path()) else { continue };
        if meta.is_dir() {
            walk(dir, &path, out, depth + 1);
        } else if !name.to_lowercase().ends_with(".srdf") {
            out.push(path);
        }
    }
}

pub(crate) fn normalize(path: &Path) -> PathBuf {
    use std::path::Component;
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::ParentDir => {
                out.pop();
            }
            Component::CurDir => {}
            other => out.push(other),
        }
    }
    out
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A host: catalogue (robot kr240__weld, fixture table), products (bug_cube), fitted scenes
    /// `jam2` and `new-scene` (`scenes/<name>/scene.yaml`), and an empty cells folder.
    pub(crate) struct Host {
        pub root: PathBuf,
        pub store: ConfigStore,
        pub assets: PathBuf,
        pub products: PathBuf,
        pub cells: PathBuf,
    }

    impl Drop for Host {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.root).ok();
        }
    }

    pub(crate) fn scene_text(products: &Path, product: &str) -> String {
        format!(
            "name: jam2\nrobots:\n- robot_id: r1\n  dressed_urdf: ../../urdf-dressed-robots/kr240__weld/kr240__weld.urdf\n  \
             position: {{x: 0.0, y: 0.0, z: 0.0}}\nfixtures:\n- id: {product}\n  urdf: {}/{product}/{product}.urdf\n  \
             position: {{x: 1.0, y: 0.0, z: 0.0}}\n- id: table\n  urdf: ../../urdf-fixture-collection/table/table.urdf\n  \
             position: {{x: 2.0, y: 0.0, z: 0.0}}\n",
            products.display()
        )
    }

    pub(crate) fn host(tag: &str) -> Host {
        let root = std::env::temp_dir().join(format!("factoryos-cell-resolve-{tag}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (assets, products, cells) = (root.join("assets"), root.join("products"), root.join("cells"));
        let write = |path: PathBuf, text: &str| {
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, text).unwrap();
        };
        write(assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.urdf"), "<robot name='r'/>");
        write(assets.join("urdf-dressed-robots/kr240__weld/meshes/link1.obj"), "o link1");
        write(assets.join("urdf-dressed-robots/kr240__weld/meshes_lowres/link1.obj"), "o low");
        write(assets.join("urdf-fixture-collection/table/table.urdf"), "<robot name='t'/>");
        for product in ["bug_cube", "cube1"] {
            write(products.join(format!("{product}/{product}.urdf")), "<robot name='p'/>");
            write(products.join(format!("{product}/product.json")), &format!("{{\"name\": \"{product}\", \"tasks\": []}}"));
        }
        write(assets.join("scenes/jam2/scene.yaml"), &scene_text(&products, "bug_cube"));
        write(assets.join("scenes/new-scene/scene.yaml"), &scene_text(&products, "cube1"));
        std::fs::create_dir_all(&cells).unwrap();
        let store = ConfigStore::open(root.join("factoryos.config.json"));
        let mut config = store.get();
        config.cells_dir = cells.display().to_string();
        config.asset_root = assets.display().to_string();
        config.products_dir = products.display().to_string();
        store.set(config);
        Host { root, store, assets, products, cells }
    }

    #[tokio::test]
    async fn a_new_cell_and_its_outputs_are_written_through_and_removals_follow() {
        use crate::storage::sync::tests_support::{FakeS3, eventually, storage_of};
        use factoryos_protocol::storage::Scope;
        use std::sync::Arc;
        let h = host("through");
        let s3 = Arc::new(FakeS3::default());
        let storage = storage_of(s3.clone(), h.products.clone(), h.cells.clone(), h.assets.clone());
        let _registered = crate::storage::register(&storage);
        // Create Cell: the new cell's folder.
        let config = super::super::cells::create_from_fitted(&h.store, "jam2.yaml", "weld").unwrap();
        assert!(config.cell_dir.contains("weld"), "{}", config.cell_dir);
        let manifest = format!("cells/weld/{MANIFEST}");
        eventually("the manifest", || s3.bytes(Scope::User, &manifest).is_some()).await;
        // Prepare Cell's record of its inputs.
        let r = resolve(&h.store, "weld").unwrap();
        record_prepared(&r).unwrap();
        let inputs = format!("cells/weld/{PREPARED_INPUTS}");
        eventually("inputs.json", || s3.bytes(Scope::User, &inputs).is_some()).await;
        // A prepared model, then unprepared (a robot's TCP changed): gone from the store too.
        std::fs::create_dir_all(r.root.join("agentic_planner/config")).unwrap();
        std::fs::write(r.root.join("agentic_planner/cell.yaml"), "robots: []\n").unwrap();
        std::fs::write(r.root.join("agentic_planner/config/robots.yaml"), "robots: {}\n").unwrap();
        crate::storage::saved_tree(r.root.join("agentic_planner"));
        let (cell_yaml, robots_yaml) = ("cells/weld/agentic_planner/cell.yaml", "cells/weld/agentic_planner/config/robots.yaml");
        eventually("the prepared model", || s3.bytes(Scope::User, cell_yaml).is_some() && s3.bytes(Scope::User, robots_yaml).is_some()).await;
        assert!(super::super::cells::unprepare(&r.root).unwrap());
        eventually("the removal", || s3.bytes(Scope::User, cell_yaml).is_none() && s3.bytes(Scope::User, robots_yaml).is_none()).await;
        assert!(s3.bytes(Scope::User, &inputs).is_some() && s3.bytes(Scope::User, &manifest).is_some(), "the rest stays");
        assert!(s3.keys(Scope::User).iter().all(|k| !k.contains("/.view/")), "the view is not the user's: {:?}", s3.keys(Scope::User));
    }

    #[test]
    fn a_reference_cell_resolves_to_the_catalogue_and_products() {
        let h = host("reference");
        let r = ensure_cell(&h.store, "jam2.yaml").unwrap();
        assert_eq!((r.name.as_str(), r.kind), ("jam2", CellKind::Reference));
        assert_eq!(r.scene_path, h.assets.join("scenes/jam2/scene.yaml"));
        assert_eq!(r.scene_base, h.assets.join("scenes/jam2"));
        assert_eq!(read_manifest(&r.root).unwrap().fitted_scene.as_deref(), Some("jam2"), "the scene by its name");
        assert_eq!(r.robots[0].urdf, h.assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.urdf"));
        assert_eq!(r.fixtures[0].urdf, h.products.join("bug_cube/bug_cube.urdf"));
        assert_eq!(r.fixtures[0].product_json, Some(h.products.join("bug_cube/product.json")));
        assert_eq!(r.fixtures[1].product_json, None);
        assert_eq!(r.inputs.urdfs.keys().collect::<Vec<_>>(), ["r1"]);
        assert_eq!(r.inputs.fixtures.len(), 2);
        assert_eq!(r.inputs.combined.len(), 64);
        let entries: Vec<_> = std::fs::read_dir(&r.root).unwrap().flatten().map(|e| e.file_name()).collect();
        assert_eq!(entries, [MANIFEST], "only the manifest is written");
        let manifest = read_manifest(&r.root).unwrap();
        assert_eq!((manifest.format, manifest.kind.as_str(), manifest.cell_id.as_str()), (2, "reference", "jam2"));
        assert_eq!(resolve(&h.store, "jam2").unwrap(), r);
        assert!(resolve(&h.store, "../etc").is_err());
    }

    #[test]
    fn a_legacy_cell_resolves_to_its_own_files() {
        let h = host("legacy");
        let cell = h.cells.join("welding_cell_2_small");
        std::fs::create_dir_all(cell.join("robots/kr")).unwrap();
        std::fs::write(cell.join("robots/kr/kr.urdf"), "<robot/>").unwrap();
        std::fs::write(
            cell.join("scene.yaml"),
            "name: w\nrobots:\n- robot_id: kr\n  dressed_urdf: robots/kr/kr.urdf\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        // A format 1 manifest of a copy: its own scene.yaml wins.
        std::fs::write(cell.join(MANIFEST), r#"{"fitted_scene": "jam2.yaml"}"#).unwrap();
        let r = resolve(&h.store, "welding_cell_2_small").unwrap();
        assert_eq!(r.kind, CellKind::Legacy);
        assert_eq!(r.scene_path, cell.join("scene.yaml"));
        assert_eq!(r.robots[0].urdf, cell.join("robots/kr/kr.urdf"));
        assert_eq!(r.fitted_scene.as_deref(), Some("jam2"), "a legacy jam2.yaml is scene jam2");
        assert_eq!(materialise_view(&r).unwrap(), cell, "a legacy cell is its own view");
        // A planner-format scene.yaml that isn't a Robo Fitter scene: no robots, still resolves.
        std::fs::write(cell.join("scene.yaml"), "cell_id: w\nrobots: {a: 1}\n").unwrap();
        assert!(resolve(&h.store, "welding_cell_2_small").unwrap().robots.is_empty());
    }

    #[test]
    fn changed_inputs_make_outputs_stale() {
        let h = host("stale");
        let r = ensure_cell(&h.store, "jam2.yaml").unwrap();
        assert_eq!(staleness(&r, &r.inputs), None);
        assert_eq!(staleness(&r, &InputHashes::default()), None, "nothing recorded: unknown, not stale");
        std::fs::create_dir_all(r.root.join("agentic_planner")).unwrap();
        std::fs::write(r.root.join("agentic_planner/cell.yaml"), "name: x\n").unwrap();
        record_prepared(&r).unwrap();
        assert_eq!(prepared_staleness(&r), None);

        // The low-res meshes and an SRDF don't count; a mesh does.
        std::fs::write(h.assets.join("urdf-dressed-robots/kr240__weld/meshes_lowres/link1.obj"), "o lower").unwrap();
        std::fs::write(h.assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.srdf"), "<robot/>").unwrap();
        assert_eq!(resolve(&h.store, "jam2").unwrap().inputs, r.inputs);
        std::fs::write(h.assets.join("urdf-dressed-robots/kr240__weld/meshes/link1.obj"), "o link1 re-dressed").unwrap();
        let now = resolve(&h.store, "jam2").unwrap();
        assert_eq!(prepared_staleness(&now).as_deref(), Some("robot r1's URDF changed"));

        // The product's welds, and the scene itself.
        std::fs::write(h.products.join("bug_cube/product.json"), "{\"name\": \"bug_cube\", \"tasks\": [1]}").unwrap();
        let mut text = scene_text(&h.products, "bug_cube");
        // Another length: file_hash caches by (mtime, size), and both writes can share an mtime tick.
        text = text.replace("x: 2.0", "x: 2.25");
        std::fs::write(h.assets.join("scenes/jam2/scene.yaml"), text).unwrap();
        let now = resolve(&h.store, "jam2").unwrap();
        assert_eq!(
            staleness(&now, &r.inputs).as_deref(),
            Some("the scene jam2 changed; robot r1's URDF changed; fixture bug_cube's files changed")
        );
        // A product swapped for another.
        std::fs::write(h.assets.join("scenes/jam2/scene.yaml"), scene_text(&h.products, "cube1")).unwrap();
        let reason = staleness(&resolve(&h.store, "jam2").unwrap(), &r.inputs).unwrap();
        assert!(reason.contains("fixture cube1 was added") && reason.contains("fixture bug_cube was removed"), "{reason}");
        // A plan's recorded inputs.
        let plan = serde_json::json!({ "planner_meta": { "inputs": r.inputs } });
        assert_eq!(plan_inputs(&plan), Some(r.inputs.clone()));
        assert_eq!(plan_inputs(&serde_json::json!({})), None);
    }

    #[test]
    fn ensuring_a_cell_is_idempotent_and_reuses_existing_ones() {
        let h = host("ensure");
        // The cell an older Create Cell made from new-scene.yaml under another name.
        let old = h.cells.join("cell_new_scene");
        std::fs::create_dir_all(&old).unwrap();
        std::fs::write(old.join(MANIFEST), r#"{"fitted_scene": "new-scene.yaml"}"#).unwrap();
        std::fs::write(old.join("scene.yaml"), "name: n\nrobots: []\n").unwrap();
        assert_eq!(cell_for_scene(&h.store, "new-scene.yaml").unwrap(), "cell_new_scene");
        let r = ensure_cell(&h.store, "new-scene.yaml").unwrap();
        assert_eq!((r.name.as_str(), r.kind), ("cell_new_scene", CellKind::Legacy));
        assert_eq!(ensure_cell_named(&h.store, "new-scene.yaml", Some("other")).unwrap().name, "cell_new_scene");

        let first = ensure_cell(&h.store, "jam2.yaml").unwrap();
        let manifest = std::fs::read(first.root.join(MANIFEST)).unwrap();
        assert_eq!(ensure_cell(&h.store, "jam2.yaml").unwrap(), first);
        assert_eq!(std::fs::read(first.root.join(MANIFEST)).unwrap(), manifest, "not rewritten");
        assert_eq!(super::super::cells::list_cells(&h.cells), ["cell_new_scene", "jam2"]);

        // A folder of that name that is another cell: a unique name.
        std::fs::create_dir_all(h.assets.join("scenes/jam-2")).unwrap();
        std::fs::write(h.assets.join("scenes/jam-2/scene.yaml"), "name: x\nrobots: []\n").unwrap();
        std::fs::create_dir_all(h.cells.join("jam_2")).unwrap();
        std::fs::write(h.cells.join("jam_2/cell.yaml"), "name: krl\n").unwrap();
        assert!(ensure_cell_named(&h.store, "jam-2.yaml", Some("bad-name")).is_err());
        assert!(ensure_cell_named(&h.store, "jam-2.yaml", Some("jam_2")).is_err(), "taken");
        assert_eq!(ensure_cell(&h.store, "jam-2.yaml").unwrap().name, "jam_2_2");
        assert_eq!(clean_name("2robot.yaml"), "2robot");
        assert_eq!(clean_name("-.yaml"), "cell");
        assert!(ensure_cell(&h.store, "missing.yaml").is_err());
        assert!(ensure_cell(&h.store, "../x.yaml").is_err());
        assert_eq!(ensure_cell(&h.store, "jam-2").unwrap().name, "jam_2_2", "by name, as by the legacy file name");
    }

    #[test]
    fn the_view_is_rebuilt_once_the_cell_has_its_srdfs() {
        let h = host("view-srdf");
        let r = ensure_cell(&h.store, "jam2.yaml").unwrap();
        // Prepare Cell builds the view before it makes the SRDFs.
        let view = materialise_view(&r).unwrap();
        assert!(std::fs::symlink_metadata(view.join("robots/kr240__weld")).unwrap().file_type().is_symlink());
        std::fs::create_dir_all(r.root.join(SRDF_DIR)).unwrap();
        std::fs::write(r.root.join(SRDF_DIR).join("kr240__weld.srdf"), "<robot/>").unwrap();
        let view = materialise_view(&r).unwrap();
        let package = view.join("robots/kr240__weld");
        assert!(!std::fs::symlink_metadata(&package).unwrap().file_type().is_symlink(), "a folder of links now");
        assert_eq!(std::fs::read_link(package.join("kr240__weld.srdf")).unwrap(), r.root.join(SRDF_DIR).join("kr240__weld.srdf"));
        assert!(package.join("kr240__weld.urdf").is_file());
    }

    #[test]
    fn the_view_lays_out_the_cell_and_follows_its_inputs() {
        let h = host("view");
        let r = ensure_cell(&h.store, "jam2.yaml").unwrap();
        let view = materialise_view(&r).unwrap();
        assert_eq!(view, r.root.join(VIEW_DIR));
        let scene = std::fs::read_to_string(view.join("scene.yaml")).unwrap();
        assert!(scene.contains("dressed_urdf: robots/kr240__weld/kr240__weld.urdf"), "{scene}");
        assert!(scene.contains("urdf: fixtures/bug_cube/bug_cube.urdf"), "{scene}");
        assert!(view.join("robots/kr240__weld/meshes/link1.obj").is_file());
        assert!(view.join("fixtures/bug_cube/product.json").is_file());
        assert!(std::fs::symlink_metadata(view.join("robots/kr240__weld")).unwrap().file_type().is_symlink());
        assert!(std::fs::read_to_string(view.join("cell.yaml")).unwrap().contains("scene_id: r1"));
        std::fs::write(r.root.join("agentic_planner/x.json"), "{}").unwrap();
        assert!(view.join("agentic_planner/x.json").is_file(), "the cell's outputs through the view");
        // Up to date: not rebuilt.
        let marker = view.join("robots/marker");
        std::fs::write(&marker, "").unwrap();
        materialise_view(&r).unwrap();
        assert!(marker.exists());
        // Another product: rebuilt.
        std::fs::write(h.assets.join("scenes/jam2/scene.yaml"), scene_text(&h.products, "cube1")).unwrap();
        let r = resolve(&h.store, "jam2").unwrap();
        materialise_view(&r).unwrap();
        assert!(!marker.exists());
        assert!(view.join("fixtures/cube1/cube1.urdf").is_file());
        assert!(!view.join("fixtures/bug_cube").exists());
        assert_eq!(super::super::cells::list_cells(&h.cells), ["jam2"], "no temporary view left");
        assert_eq!(std::fs::read_to_string(r.root.join("agentic_planner/x.json")).unwrap(), "{}", "outputs stay");

        // A scene path that names no package can't place a link outside the view; the old view stays.
        std::fs::write(
            h.assets.join("scenes/jam2/scene.yaml"),
            "name: x\nrobots:\n- robot_id: r1\n  dressed_urdf: ../../urdf-dressed-robots/../..\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        let r = resolve(&h.store, "jam2").unwrap();
        assert!(materialise_view(&r).is_err());
        assert!(view.join("fixtures/cube1/cube1.urdf").is_file(), "the previous view is kept");
        assert_eq!(super::super::cells::list_cells(&h.cells), ["jam2"]);
        assert!(std::fs::read_dir(&r.root).unwrap().flatten().all(|e| !e.file_name().to_string_lossy().contains(".tmp-")));
    }

    /// A robot's hash includes its tool's planner sidecar: editing it makes the cells stale.
    #[test]
    fn a_tools_planner_sidecar_is_part_of_the_robots_hash() {
        let dir = crate::services::tcp::tests::TestDir::new();
        let catalogue = dir.path().join("catalogue");
        let urdf = crate::services::tcp::tests::write_catalogue(&catalogue, true);
        // The release's catalogue (tool_catalogues looks there too) ships a sticktool sidecar: it
        // counts for this robot's hash already.
        let release = tool_catalogues(&urdf, &catalogue.join("scenes"));
        assert_ne!(robot_hash(&urdf, &release), package_hash(&urdf), "the release's sticktool/planner.yaml");
        let tools = ToolCatalogues::new([catalogue.clone()]);
        assert_eq!(robot_hash(&urdf, &tools), package_hash(&urdf), "no sidecar: the package's hash");
        let sidecar = catalogue.join("urdf-tool-collection/sticktool").join(crate::services::tcp::TOOL_SIDECAR);
        std::fs::write(&sidecar, "tool_link_radius: 0.02\n").unwrap();
        let with = robot_hash(&urdf, &tools);
        assert_ne!(with, package_hash(&urdf));
        // Another length: file_hash caches by (mtime, size), and both writes can share an mtime tick.
        std::fs::write(&sidecar, "tool_link_radius: 0.025\n").unwrap();
        assert_ne!(robot_hash(&urdf, &tools), with, "edited");
    }

    /// A robot's hash includes its robot model's planner sidecar too.
    #[test]
    fn a_robot_models_planner_sidecar_is_part_of_the_robots_hash() {
        let dir = crate::services::tcp::tests::TestDir::new();
        let catalogue = dir.path().join("catalogue");
        let urdf = crate::services::tcp::tests::write_catalogue(&catalogue, true);
        // Not the release's catalogue: it has a sidecar for the sticktool.
        let tools = ToolCatalogues::new([catalogue.clone()]);
        assert_eq!(robot_hash(&urdf, &tools), package_hash(&urdf), "no sidecar: the package's hash");
        let model = catalogue.join("urdf-robot-collection/kr240");
        std::fs::create_dir_all(&model).unwrap();
        let sidecar = model.join(crate::services::tcp::ROBOT_SIDECAR);
        std::fs::write(&sidecar, "joint_acceleration_limits: [3, 3, 3, 6, 6, 8]\n").unwrap();
        let with = robot_hash(&urdf, &tools);
        assert_ne!(with, package_hash(&urdf));
        std::fs::write(&sidecar, "joint_acceleration_limits: [3, 3, 3, 6, 6, 9.5]\n").unwrap();
        assert_ne!(robot_hash(&urdf, &tools), with, "edited");
    }

    #[test]
    fn a_urdf_at_a_cells_root_hashes_alone() {
        let h = host("root-urdf");
        let cell = h.cells.join("krl");
        std::fs::create_dir_all(cell.join("agentic_planner/plans")).unwrap();
        std::fs::write(cell.join("kr.urdf"), "<robot/>").unwrap();
        let before = package_hash(&cell.join("kr.urdf"));
        std::fs::write(cell.join("agentic_planner/plans/bt.json"), "{}").unwrap();
        assert_eq!(package_hash(&cell.join("kr.urdf")), before, "a saved plan is no input");
    }

    /// The shipped sample cell (assets/cells/welding_cell_2_small, provision.sh's default cell) is a reference
    /// cell of the release's catalogue (assets/catalogue): its scene, robots and fixture resolve there, its view
    /// lays them out as the planners read them (umarell_jam: `.view/scene.yaml`, the dressed URDFs, the fixture),
    /// Prepare Cell finds each robot's TCP in the dressed URDF, and its task program is one umarell_jam accepts.
    #[test]
    fn the_shipped_sample_cell_is_a_reference_cell_of_the_catalogue() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let catalogue = normalize(&repo.join("assets/catalogue"));
        let dir = std::env::temp_dir().join(format!("factoryos-sample-cell-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        let root = dir.join("cells/welding_cell_2_small");
        // The shipped files: not a view a backend run from the checkout built there (gitignored).
        fn copy_shipped(from: &Path, to: &Path) {
            std::fs::create_dir_all(to).unwrap();
            for entry in std::fs::read_dir(from).unwrap().flatten() {
                let name = entry.file_name();
                if name.to_string_lossy().starts_with(VIEW_DIR) {
                    continue;
                }
                if entry.file_type().unwrap().is_dir() {
                    copy_shipped(&entry.path(), &to.join(&name));
                } else {
                    std::fs::copy(entry.path(), to.join(&name)).unwrap();
                }
            }
        }
        copy_shipped(&repo.join("assets/cells/welding_cell_2_small"), &root);
        let r = resolve_root(&root, &catalogue.join("scenes")).unwrap();
        assert_eq!((r.kind, r.fitted_scene.as_deref()), (CellKind::Reference, Some("welding_cell_2_small")));
        assert_eq!(r.scene_path, catalogue.join("scenes/welding_cell_2_small/scene.yaml"));
        let urdf = catalogue.join("urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        let robots: Vec<_> = r.robots.iter().map(|a| (a.scene_id.as_str(), a.urdf.clone())).collect();
        assert_eq!(robots, [("kr240r2900__sticktool", urdf.clone()), ("kr240r2900__sticktool_2", urdf.clone())]);
        let fixtures: Vec<_> = r.fixtures.iter().map(|a| (a.scene_id.as_str(), a.urdf.clone(), a.product_json.clone())).collect();
        let fixture = catalogue.join("urdf-fixture-collection/car_door_fixture/car_door_fixture.urdf");
        assert_eq!(fixtures, [("car_door_fixture", fixture, None)]);
        assert!(r.robots.iter().chain(&r.fixtures).all(|a| a.urdf.is_file()));

        let view = materialise_view(&r).unwrap();
        assert_eq!(view, root.join(VIEW_DIR));
        for file in [
            "scene.yaml",
            "cell.yaml",
            "robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf",
            "fixtures/car_door_fixture/car_door_fixture.urdf",
            "fixtures/car_door_fixture/meshes/car_door_fixture.obj",
            "floorplans/hall-a-floor-plan/hall-a-floor-plan.obj",
            "agentic_planner/tasks/car_door_spot_welding.json",
        ] {
            assert!(view.join(file).is_file(), "{file}");
        }

        let tools = tool_catalogues(&urdf, &catalogue.join("scenes"));
        let models = super::super::planner_model::read_scene(&r.scene_path, &tools).unwrap();
        assert_eq!(models.len(), 2);
        for model in &models {
            let tcp = model.tcp.as_ref().unwrap();
            assert_eq!((tcp.source.clone(), tcp.tip_link.as_str()), (crate::services::tcp::TcpSource::Frame, "like_kr240r2900_link_6"));
            assert!(model.pedestal.is_none() && model.base[2] == 0.0, "{model:?}");
        }

        // One program, 15 welds; umarell_jam (cell planner umarell_jam/cell.py check_program) rejects a weld whose
        // weld_orientation differs from its orientation.
        let program: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("agentic_planner/tasks/car_door_spot_welding.json")).unwrap()).unwrap();
        assert_eq!(program["cell_id"], "welding_cell_2_small");
        let ops = program["operations"].as_array().unwrap();
        assert_eq!(ops.len(), 15);
        for op in ops {
            assert!(op.get("weld_orientation").is_none_or(|w| *w == op["orientation"]), "{op}");
        }
        std::fs::remove_dir_all(&dir).ok();
    }
}
