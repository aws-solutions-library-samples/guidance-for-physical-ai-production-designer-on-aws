//! Cells on disk, mirroring the Kiro extension (`extension.ts`, `cell-scaffold.ts`):
//! * the Cell Runner's "Active cell" picker lists the cells under the cells folder and points
//!   `cellDir` / `envDir` / `scene` at the chosen one;
//! * a Robo Fitter scene's cell is a folder for its outputs that references the scene (see
//!   [`super::cell_resolve`]): made by the first Plan Cell, Load & Play, Prepare or "Use as cell"
//!   on the scene, nothing copied. Cells the old "Create Cellrunner Cell" made (assets copied in,
//!   paths rewritten, scaffold `cell.yaml`, USDA bundle extracted) still work as legacy cells.

use std::path::{Path, PathBuf};

use anyhow::Context as _;
use factoryos_protocol::config::Config;
use factoryos_protocol::{CellEntry, CellKind, CellsInfo, PlanInfo, PlansInfo, SampleCell, TaskProgram};

use super::cell_resolve::{self, ResolvedCell};
use crate::config_store::ConfigStore;
use crate::paths;

pub(crate) const SCENE_MARKER: &str = "scene.yaml";
/// The cell-runner's scene when the cell's `scene.yaml` is in another format: umarell cells keep
/// `scene.yaml` in the planner's own format, which the cell-runner's 3D view cannot read.
pub(crate) const FACTORY_OS_SCENE: &str = "scene.factoryos.yaml";
pub(crate) const CELL_MARKER: &str = "cell.yaml";
/// The cell's manifest (see [`cell_resolve::Manifest`]): `{"fitted_scene": "<scene name>"}` in a
/// copied (legacy) cell, `{"format": 2, "kind": "reference", …}` in a reference cell.
const SOURCE_MARKER: &str = cell_resolve::MANIFEST;
/// Where the agentic planner saves its behaviour trees, relative to the cell folder.
pub(crate) const PLANS_DIR: &str = "agentic_planner/plans";
/// Where its task programs (the operations to plan) live, relative to the cell folder.
const TASKS_DIR: &str = super::product_tasks::TASKS_DIR;
/// The planner of a saved plan that does not name one: umarell's own plans carry no `planner`.
const DEFAULT_PLANNER: &str = "umarell";

/// The cells folder, the cells in it and the fitted scenes, for the current config.
pub fn info(store: &ConfigStore) -> CellsInfo {
    let config = store.get();
    let cells_dir = cells_dir(store, &config);
    let cells = cells_dir.as_deref().map(list_cells).unwrap_or_default();
    let root = cell_root(&store.resolve(&config.cell_dir));
    let active = match (&cells_dir, root.parent()) {
        (Some(dir), Some(parent)) if same_path(dir, parent) => {
            root.file_name().unwrap_or_default().to_string_lossy().into_owned()
        }
        _ => String::new(),
    };
    let fitted_robots_dir = fitted_robots_dir(store, &config);
    let entries = match &cells_dir {
        Some(dir) => cells.iter().map(|cell| cell_entry(&dir.join(cell), &fitted_robots_dir)).collect(),
        None => Vec::new(),
    };
    CellsInfo {
        cells_dir: cells_dir.map(|d| d.display().to_string()).unwrap_or_default(),
        cells,
        active,
        fitted_scenes: list_fitted_scenes(&fitted_robots_dir),
        fitted_robots_dir: fitted_robots_dir.display().to_string(),
        entries,
    }
}

/// What the cell in folder `root` is, for the cell list.
fn cell_entry(root: &Path, fitted_dir: &Path) -> CellEntry {
    let name = root.file_name().unwrap_or_default().to_string_lossy().into_owned();
    match cell_resolve::resolve_root(root, fitted_dir) {
        Ok(r) => {
            let prepared = cell_resolve::prepared_staleness(&r);
            let copy_out_of_date = (r.kind == CellKind::Legacy)
                .then(|| legacy_out_of_date(root, fitted_dir))
                .flatten()
                .map(|scene| format!("copied from {scene}, which changed since"));
            CellEntry {
                name,
                scene: r.fitted_scene.clone(),
                kind: r.kind,
                prepared_stale: prepared.is_some(),
                stale_reason: prepared.or(copy_out_of_date),
            }
        }
        // E.g. its scene is gone.
        Err(err) => CellEntry {
            name,
            scene: fitted_source(root),
            kind: CellKind::Reference,
            prepared_stale: false,
            stale_reason: Some(format!("{err:#}")),
        },
    }
}

/// Make `cell` (a folder in the cells folder) the active cell. Returns the new config.
pub fn set_active(store: &ConfigStore, cell: &str) -> anyhow::Result<Config> {
    let mut config = store.get();
    let dir = cells_dir(store, &config).context("no cells folder configured")?.join(cell);
    anyhow::ensure!(
        !cell.is_empty() && !cell.contains(['/', '\\']) && !cell.starts_with('.') && is_cell_dir(&dir),
        "{} is not a cell",
        dir.display()
    );
    let resolved = cell_resolve::resolve_root(&dir, &fitted_robots_dir(store, &config))?;
    if resolved.kind == CellKind::Reference {
        apply_reference_cell(&mut config, &cell_resolve::materialise_view(&resolved)?);
    } else {
        apply_cell(&mut config, &dir, &paths::sample_cells());
    }
    store.try_set(config.clone())?;
    Ok(store.get())
}

/// Point the config at a reference cell's view (`<cell>/.view/`, see
/// [`cell_resolve::materialise_view`]): the cell-runner's `cellDir` and `envDir`, its `scene.yaml`.
/// [`active_cell`] is still the cell's folder.
fn apply_reference_cell(config: &mut Config, view: &Path) {
    config.cell_dir = view.display().to_string();
    config.env_dir = view.display().to_string();
    config.scene = SCENE_MARKER.to_owned();
}

/// The active cell, resolved (see [`cell_resolve`]).
pub fn active_resolved(store: &ConfigStore) -> Option<ResolvedCell> {
    let (_, root) = active_cell(store)?;
    cell_resolve::resolve_root(&root, &fitted_scenes_dir(store)).ok()
}

/// Rebuild the active reference cell's view if its inputs changed (e.g. its scene was saved), so
/// the cell-runner's `cellDir`/`envDir` show the scene as it is now. Returns the view.
pub fn refresh_active_view(store: &ConfigStore) -> Option<PathBuf> {
    let resolved = active_resolved(store).filter(|r| r.kind == CellKind::Reference)?;
    match cell_resolve::materialise_view(&resolved) {
        Ok(view) => Some(view),
        Err(err) => {
            tracing::warn!("Cell {}: {err:#}", resolved.name);
            None
        }
    }
}

/// On start: point the config at the active reference cell's view (rebuilt if its inputs changed),
/// e.g. after `fos-migrate-cells` turned the active cell into one. Returns the new config if it
/// changed.
pub fn normalise_active(store: &ConfigStore) -> Option<Config> {
    let resolved = active_resolved(store).filter(|r| r.kind == CellKind::Reference)?;
    let view = refresh_active_view(store)?;
    let config = store.get();
    if same_path(Path::new(&store.resolve(&config.cell_dir)), &view) && same_path(Path::new(&store.resolve(&config.env_dir)), &view) {
        return None;
    }
    match set_active(store, &resolved.name) {
        Ok(config) => Some(config),
        Err(err) => {
            tracing::warn!("Cell {}: {err:#}", resolved.name);
            None
        }
    }
}

/// "Create Cell" / "Use as cell": the cell of the fitted scene `scene` (a cell that references
/// it or was copied from it; else a new reference cell named `name`, or after the scene when
/// `name` is empty), with its task programs derived from its products, made the active cell.
/// Nothing is copied. Returns the new config.
pub fn create_from_fitted(store: &ConfigStore, scene: &str, name: &str) -> anyhow::Result<Config> {
    let name = name.trim();
    let resolved = cell_resolve::ensure_cell_named(store, scene, (!name.is_empty()).then_some(name))?;
    derive_tasks(store, &resolved.root);
    set_active(store, &resolved.name)
}

/// Derive cell `root`'s task programs from the products placed in it (see
/// [`super::product_tasks`]), logging what changed. Returns the summary line, if any.
pub fn derive_tasks(store: &ConfigStore, root: &Path) -> Option<String> {
    let derivation = super::product_tasks::derive_cell_with(root, &products_scene(store, root), &super::product::products_dir(store));
    // The task programs (written and removed) and the marked obstacles, to the user's store (S3 mode).
    for stem in &derivation.removed {
        crate::storage::saved(root.join(TASKS_DIR).join(format!("{stem}.json")));
    }
    if !derivation.written.is_empty() {
        crate::storage::saved_tree(root.join(TASKS_DIR));
    }
    if !derivation.workpieces.is_empty() {
        crate::storage::saved(root.join(super::planner_model::OBSTACLES_YAML));
    }
    if !derivation.refreshed.is_empty() {
        // A legacy cell's copies of its products (`fixtures/<product>/product.json`).
        crate::storage::saved_tree(root.join("fixtures"));
    }
    let cell = root.file_name().unwrap_or_default().to_string_lossy();
    let summary = derivation.summary(&cell);
    if let Some(line) = &summary {
        if derivation.warnings.is_empty() {
            tracing::info!("{line}");
        } else {
            tracing::warn!("{line}");
        }
    }
    summary
}

/// The world positions of the welds of cell `root`'s products (see
/// [`super::product_tasks::weld_points`]), for Prepare Cell's near-weld refinement.
pub fn weld_points(store: &ConfigStore, root: &Path) -> Vec<[f64; 3]> {
    super::product_tasks::weld_points(&products_scene(store, root), &super::product::products_dir(store))
}

/// The cells using product `name` (a fixture of theirs is it): the cells in the cells folder and
/// the active cell. Their folders.
pub fn cells_using_product(store: &ConfigStore, name: &str) -> Vec<PathBuf> {
    let config = store.get();
    let products_dir = super::product::products_dir(store);
    let mut roots: Vec<PathBuf> = cells_dir(store, &config)
        .map(|dir| list_cells(&dir).into_iter().map(|cell| dir.join(cell)).collect())
        .unwrap_or_default();
    if let Some((_, root)) = active_cell(store)
        && !roots.iter().any(|r| same_path(r, &root))
    {
        roots.push(root);
    }
    roots.retain(|root| super::product_tasks::uses_product_in(&products_scene(store, root), &products_dir, name));
    roots
}

/// The scene that places cell `root`'s products: a reference cell's Robo Fitter scene (its paths
/// from `scenes/`, its products the products folder's own files), a legacy cell's own
/// `scene.yaml`.
fn products_scene(store: &ConfigStore, root: &Path) -> PathBuf {
    cell_resolve::resolve_root(root, &fitted_scenes_dir(store))
        .map(|r| r.scene_path)
        .unwrap_or_else(|_| super::product_tasks::cell_scene(root))
}

/// The reference cells (in the cells folder) with a robot or fixture whose URDF is one of `urdfs`.
pub fn reference_cells_using(store: &ConfigStore, urdfs: &[PathBuf]) -> Vec<String> {
    if urdfs.is_empty() {
        return Vec::new();
    }
    let Some(dir) = cells_dir(store, &store.get()) else { return Vec::new() };
    let fitted_dir = fitted_scenes_dir(store);
    let wanted: Vec<PathBuf> = urdfs.iter().map(|p| std::fs::canonicalize(p).unwrap_or_else(|_| p.clone())).collect();
    list_cells(&dir)
        .into_iter()
        .filter(|cell| {
            cell_resolve::resolve_root(&dir.join(cell), &fitted_dir).is_ok_and(|r| {
                r.kind == CellKind::Reference
                    && r.robots.iter().chain(&r.fixtures).any(|asset| {
                        let urdf = std::fs::canonicalize(&asset.urdf).unwrap_or_else(|_| asset.urdf.clone());
                        wanted.contains(&urdf)
                    })
            })
        })
        .collect()
}

/// Where a cell's USDA bundle is extracted.
pub(crate) const USDA_EXTRACTED: &str = "usda-extracted";
/// What Prepare Cell derives from the cell's geometry (the planner's cell model): removed by a
/// rebuild, so the cell is unprepared until Prepare Cell runs again.
const PREPARED_MODEL: [&str; 2] = ["agentic_planner/cell.yaml", "agentic_planner/config"];

/// Mark cell `root` unprepared, the way a rebuild does: remove the planner's cell model Prepare
/// Cell made ([`PREPARED_MODEL`]), e.g. because a robot's TCP changed. True if there was one.
pub fn unprepare(root: &Path) -> anyhow::Result<bool> {
    let mut removed = false;
    for derived in PREPARED_MODEL {
        let path = root.join(derived);
        if std::fs::symlink_metadata(&path).is_ok() {
            remove_path(&path).with_context(|| format!("removing {}", path.display()))?;
            crate::storage::removed(&path);
            removed = true;
        }
    }
    Ok(removed)
}

/// The cells (folder names in the cells folder) made from the fitted scene `scene`.
pub fn made_from(store: &ConfigStore, scene: &str) -> Vec<String> {
    let config = store.get();
    let Some(cells_dir) = cells_dir(store, &config) else {
        return Vec::new();
    };
    list_cells(&cells_dir)
        .into_iter()
        .filter(|cell| fitted_source(&cells_dir.join(cell)).is_some_and(|s| Some(s) == super::scene_store::name_of(scene)))
        .collect()
}

/// The cell folder of `cell` in the cells folder.
pub fn cell_folder(store: &ConfigStore, cell: &str) -> Option<PathBuf> {
    cells_dir(store, &store.get()).map(|dir| dir.join(cell))
}

/// Whether cell `root` differs from the scene `scene_text` (a `scenes/` manifest): its
/// `scene.yaml` is not what the old Create Cell made of it. (Asset files are not compared.)
fn differs_from(root: &Path, scene_text: &str) -> bool {
    let cell_scene = std::fs::read_to_string(root.join(SCENE_MARKER)).unwrap_or_default();
    cell_scene != plan_cell_scaffold(scene_text).scene
}

/// The fitted scene a legacy (copied) cell `root` was made from, when the copy no longer matches
/// the saved file. `None` for a reference cell (it reads the scene itself), and when the scene
/// file is gone.
fn legacy_out_of_date(root: &Path, fitted_dir: &Path) -> Option<String> {
    if !root.join(SCENE_MARKER).is_file() {
        return None;
    }
    let scene = fitted_source(root)?;
    let text = std::fs::read_to_string(super::scene_store::scene_file(fitted_dir, &scene)).ok()?;
    differs_from(root, &text).then_some(scene)
}

/// The fitted scene a legacy (copied) cell `root` was made from, when the copy no longer matches
/// its saved file (see [`legacy_out_of_date`]). Saving the scene does not rebuild copies any more:
/// migrate the cell (`fos-migrate-cells`) or use the scene's reference cell.
pub fn out_of_date_with(store: &ConfigStore, root: &Path) -> Option<String> {
    legacy_out_of_date(root, &fitted_robots_dir(store, &store.get()))
}

pub(crate) fn remove_path(path: &Path) -> std::io::Result<()> {
    match std::fs::symlink_metadata(path) {
        Ok(meta) if meta.is_dir() => std::fs::remove_dir_all(path),
        Ok(_) => std::fs::remove_file(path),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(err) => Err(err),
    }
}

/// The `.zip` files at a cell's root (the USDA bundle Create Cell copied in).
pub(crate) fn root_zips(root: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(root)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_file() && path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("zip")))
        .collect()
}

/// When the cell was last rebuilt from its fitted scene (Unix seconds), if it was.
fn rebuilt_s(cell_root: &Path) -> Option<f64> {
    let text = std::fs::read_to_string(cell_root.join(SOURCE_MARKER)).ok()?;
    serde_json::from_str::<serde_json::Value>(&text).ok()?["rebuilt_s"].as_f64()
}

/// How many behaviour trees cell `root` has saved.
pub fn plan_count(root: &Path) -> usize {
    list_plans(root, None).len()
}

/// The active cell: its folder name and folder (the cell root, not `cellrunner/`).
pub fn active_cell(store: &ConfigStore) -> Option<(String, PathBuf)> {
    let cell_dir = store.resolve(&store.get().cell_dir);
    if cell_dir.is_empty() {
        return None;
    }
    let root = cell_root(&cell_dir);
    let name = root.file_name()?.to_string_lossy().into_owned();
    Some((name, root))
}

/// The Robo Fitter scene (its name) a cell was made from, if it was. A legacy manifest's
/// `<name>.yaml` reads as `<name>`.
pub fn fitted_source(cell_root: &Path) -> Option<String> {
    let text = std::fs::read_to_string(cell_root.join(SOURCE_MARKER)).ok()?;
    let value: serde_json::Value = serde_json::from_str(&text).ok()?;
    value["fitted_scene"].as_str().and_then(super::scene_store::name_of)
}

/// The cell's robots as `(cell-runner robot id, Robo Fitter robot_id)`, from its `cell.yaml`.
pub fn robot_scene_ids(cell_root: &Path) -> Vec<(String, String)> {
    #[derive(serde::Deserialize)]
    struct CellYaml {
        #[serde(default)]
        robots: Vec<CellRobot>,
    }
    #[derive(serde::Deserialize)]
    struct CellRobot {
        id: String,
        #[serde(default)]
        scene_id: String,
    }
    let text = std::fs::read_to_string(resolve_cell_dir(cell_root).join(CELL_MARKER)).unwrap_or_default();
    serde_yaml::from_str::<CellYaml>(&text)
        .map(|cell| {
            cell.robots
                .into_iter()
                .filter(|r| !r.scene_id.is_empty())
                .map(|r| (r.id, r.scene_id))
                .collect()
        })
        .unwrap_or_default()
}

/// The behaviour trees the agentic planner saved for the active cell, newest first.
pub fn plans(store: &ConfigStore) -> PlansInfo {
    let Some((cell, root)) = active_cell(store) else {
        return PlansInfo::default();
    };
    let resolved = cell_resolve::resolve_root(&root, &fitted_scenes_dir(store)).ok();
    let plans = list_plans(&root, resolved.as_ref());
    let mut tasks: Vec<String> = std::fs::read_dir(root.join(TASKS_DIR))
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            (path.extension()? == "json").then_some(())?;
            Some(path.file_stem()?.to_string_lossy().into_owned())
        })
        .collect();
    tasks.sort();
    let (products, product_welds) = product_welds(resolved.as_ref());
    PlansInfo {
        cell,
        plans,
        // Prepare Cell writes the planner's own cell model; saving a task program only makes `tasks/`.
        prepared: root.join("agentic_planner/cell.yaml").is_file(),
        out_of_date_with: out_of_date_with(store, &root),
        task_sources: super::product_tasks::task_sources_with(
            &root,
            &resolved.as_ref().map_or_else(|| super::product_tasks::cell_scene(&root), |r| r.scene_path.clone()),
            &super::product::products_dir(store),
            &tasks,
        ),
        tasks,
        products,
        product_welds,
        kind: resolved.as_ref().map_or(CellKind::Legacy, |r| r.kind),
        scene: resolved.as_ref().and_then(|r| r.fitted_scene.clone()),
        prepared_stale_reason: resolved.as_ref().and_then(cell_resolve::prepared_staleness),
    }
}

/// The products placed in the cell (a scene fixture with a `product.json` beside its URDF: the
/// product's own, or a legacy cell's copy), with point operations or not, and how many point
/// operations they have.
fn product_welds(cell: Option<&ResolvedCell>) -> (Vec<String>, u32) {
    use factoryos_protocol::product::{Product, Target};
    let Some(cell) = cell else {
        return (Vec::new(), 0);
    };
    let mut names = Vec::new();
    let mut welds = 0;
    for fixture in &cell.fixtures {
        let Some(product) = fixture
            .product_json
            .as_ref()
            .and_then(|path| std::fs::read_to_string(path).ok())
            .and_then(|text| serde_json::from_str::<Product>(&text).ok())
        else {
            continue;
        };
        let points = product
            .tasks
            .iter()
            .flat_map(|task| &task.operations)
            .filter(|op| matches!(op.target, Target::Point { .. }))
            .count() as u32;
        // Also a product without welds yet: the user defines them in Product & Task.
        let name = if product.name.is_empty() { fixture.scene_id.clone() } else { product.name };
        if !names.contains(&name) {
            names.push(name);
        }
        welds += points;
    }
    (names, welds)
}

/// The behaviour trees the agentic planner saved in cell `root`, newest first. Stale: those whose
/// recorded inputs (`planner_meta.inputs`) the cell (`resolved`) no longer matches, and those made
/// before a legacy cell was last rebuilt from its scene.
fn list_plans(root: &Path, resolved: Option<&ResolvedCell>) -> Vec<PlanInfo> {
    let dir = root.join(PLANS_DIR);
    let rebuilt = rebuilt_s(root);
    let mut plans: Vec<PlanInfo> = std::fs::read_dir(&dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter(|entry| entry.path().extension().is_some_and(|ext| ext == "json"))
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            let value: serde_json::Value = serde_json::from_slice(&std::fs::read(entry.path()).ok()?).ok()?;
            // A saved plan, not some other JSON (e.g. a quality report).
            value.get("behaviour_tree")?;
            let modified = entry.metadata().ok()?.modified().ok()?;
            let modified_s = modified.duration_since(std::time::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
            let created_s = value["created_s"].as_f64().unwrap_or(modified_s);
            let stale_reason = resolved
                .zip(cell_resolve::plan_inputs(&value))
                .and_then(|(cell, inputs)| cell_resolve::staleness(cell, &inputs))
                .or_else(|| {
                    rebuilt
                        .is_some_and(|rebuilt| created_s < rebuilt)
                        .then(|| "planned before the cell was rebuilt from its scene".to_owned())
                });
            Some(PlanInfo {
                path: format!("{PLANS_DIR}/{name}"),
                tree_id: value["tree_id"].as_str().unwrap_or(name.trim_end_matches(".json")).to_owned(),
                created_s,
                num_operations: value["num_operations"].as_u64().unwrap_or(0),
                planner: value["planner"].as_str().filter(|p| !p.is_empty()).unwrap_or(DEFAULT_PLANNER).to_owned(),
                status: value["status"].as_str().unwrap_or("ok").to_owned(),
                stale: stale_reason.is_some(),
                stale_reason,
            })
        })
        .collect();
    plans.sort_by(|a, b| b.created_s.total_cmp(&a.created_s));
    plans
}

/// A task program's file name: letters, digits, `_` and `-` (no paths).
fn valid_task_name(name: &str) -> bool {
    !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

/// The active cell's task program `name`.
pub fn read_task(store: &ConfigStore, name: &str) -> anyhow::Result<TaskProgram> {
    anyhow::ensure!(valid_task_name(name), "invalid task program name: {name}");
    let (_, root) = active_cell(store).context("no active cell")?;
    let file = root.join(TASKS_DIR).join(format!("{name}.json"));
    let text = std::fs::read_to_string(&file).with_context(|| format!("reading {}", file.display()))?;
    Ok(TaskProgram { name: name.to_owned(), text })
}

/// Save a task program into the active cell, after checking it is a task program at all.
pub fn write_task(store: &ConfigStore, task: &TaskProgram) -> anyhow::Result<()> {
    anyhow::ensure!(valid_task_name(&task.name), "use letters, digits, _ and - for the task program name");
    let value: serde_json::Value = serde_json::from_str(&task.text).context("not valid JSON")?;
    anyhow::ensure!(
        value.get("operations").is_some_and(serde_json::Value::is_array),
        "a task program needs an `operations` list"
    );
    let (_, root) = active_cell(store).context("no active cell")?;
    let dir = root.join(TASKS_DIR);
    std::fs::create_dir_all(&dir)?;
    let file = dir.join(format!("{}.json", task.name));
    std::fs::write(&file, &task.text)?;
    crate::storage::saved(&file);
    Ok(())
}

/// The bundled sample task program (12 spot welds on the top edges of a 1 m cube at the origin),
/// with `cell_id` set to the active cell's planner id.
pub fn sample_task(store: &ConfigStore) -> TaskProgram {
    const SAMPLE: &str = include_str!("../../assets/sample-task-cube-top-edges.json");
    let mut value: serde_json::Value = serde_json::from_str(SAMPLE).expect("bundled sample task is valid JSON");
    if let Some((cell, _)) = active_cell(store) {
        value["cell_id"] = planner_cell_id(&cell).into();
    }
    TaskProgram {
        name: value["program"].as_str().unwrap_or("sample").to_owned(),
        text: serde_json::to_string_pretty(&value).unwrap_or_else(|_| SAMPLE.to_owned()),
    }
}

/// The agentic planner's id for a cell (a ROS namespace: no leading digit).
pub fn planner_cell_id(cell: &str) -> String {
    let id: String = cell.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
    if id.starts_with(|c: char| c.is_ascii_digit()) { format!("cell_{id}") } else { id }
}

/// The plan at `path` (as in [`PlanInfo::path`]) for the cell-runner's `trajectory_load`: the path
/// relative to its cell dir if it lies inside it, else the plan itself (inline).
pub fn plan_load_params(store: &ConfigStore, path: &str) -> anyhow::Result<serde_json::Value> {
    anyhow::ensure!(
        !path.contains("..") && path.starts_with(PLANS_DIR),
        "not a plan of the active cell: {path}"
    );
    let (_, root) = active_cell(store).context("no active cell")?;
    let file = root.join(path);
    let cell_dir = PathBuf::from(store.resolve(&store.get().cell_dir));
    // A reference cell's view links its `agentic_planner/`.
    if cell_dir.file_name().is_some_and(|name| name == cell_resolve::VIEW_DIR) && cell_dir.join(path).is_file() {
        return Ok(serde_json::json!({ "path": path }));
    }
    Ok(match file.strip_prefix(&cell_dir) {
        Ok(rel) => serde_json::json!({ "path": rel.to_string_lossy() }),
        Err(_) => {
            let text = std::fs::read_to_string(&file).with_context(|| format!("reading {}", file.display()))?;
            serde_json::json!({ "plan": serde_json::from_str::<serde_json::Value>(&text)? })
        }
    })
}

/// A saved plan's name (its file name in `agentic_planner/plans/` without `.json`): letters,
/// digits, `_`, `-` and `.`, and no `..` (no paths).
pub fn valid_plan_name(name: &str) -> bool {
    !name.is_empty()
        && !name.contains("..")
        && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'))
}

/// One saved plan, as [`read_plan`] found it.
#[derive(Debug, PartialEq)]
pub enum PlanFile {
    /// The file, whole (blackboard included), and its ETag.
    Plan { etag: String, bytes: Vec<u8> },
    /// The client's copy (`If-None-Match`) is still current.
    NotModified { etag: String },
}

/// The active cell's plan `agentic_planner/plans/<name>.json` for the behaviour-tree pane, with a
/// weak ETag from its modification time and size; `if_none_match` is the request's header, and a
/// match skips reading the file. Errors: an invalid name, no active cell, no such plan.
pub fn read_plan(store: &ConfigStore, name: &str, if_none_match: Option<&str>) -> anyhow::Result<PlanFile> {
    anyhow::ensure!(valid_plan_name(name), "invalid plan name: {name}");
    let (_, root) = active_cell(store).context("no active cell")?;
    let file = root.join(PLANS_DIR).join(format!("{name}.json"));
    let etag = plan_etag(&std::fs::metadata(&file).with_context(|| format!("no plan {name}"))?);
    if if_none_match.is_some_and(|header| etag_matches(header, &etag)) {
        return Ok(PlanFile::NotModified { etag });
    }
    let bytes = std::fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
    Ok(PlanFile::Plan { etag, bytes })
}

/// `W/"<mtime ns>-<size>"`, in hex: weak, since the same file goes out gzipped or not.
pub(crate) fn plan_etag(meta: &std::fs::Metadata) -> String {
    let modified = meta
        .modified()
        .ok()
        .and_then(|time| time.duration_since(std::time::UNIX_EPOCH).ok())
        .map_or(0, |d| d.as_nanos());
    format!("W/\"{modified:x}-{:x}\"", meta.len())
}

/// Whether an `If-None-Match` header lists `etag` (weak comparison, as RFC 9110 asks for it).
fn etag_matches(header: &str, etag: &str) -> bool {
    let opaque = |tag: &str| tag.trim().trim_start_matches("W/").to_owned();
    header.split(',').any(|tag| tag.trim() == "*" || opaque(tag) == opaque(etag))
}

/// The cells folder: the configured one, else the active cell's parent.
pub fn cells_folder(store: &ConfigStore) -> Option<PathBuf> {
    cells_dir(store, &store.get())
}

/// The configured cells folder, else the active cell's parent.
fn cells_dir(store: &ConfigStore, config: &Config) -> Option<PathBuf> {
    let configured = store.resolve(&config.cells_dir);
    if !configured.is_empty() {
        return Some(PathBuf::from(configured));
    }
    let cell = store.resolve(&config.cell_dir);
    (!cell.is_empty()).then(|| cell_root(&cell).parent().map(Path::to_path_buf)).flatten()
}

/// The Robo Fitter's scenes folder (`scenes/`), for the current config.
pub fn fitted_scenes_dir(store: &ConfigStore) -> PathBuf {
    fitted_robots_dir(store, &store.get())
}

/// The catalogue's scenes folder: `fittedRobotsDir` if set (a configured `…/scenes`, the
/// old default, counts as unset), else `<assetRoot>/scenes`.
fn fitted_robots_dir(store: &ConfigStore, config: &Config) -> PathBuf {
    let configured = store.resolve(&config.fitted_robots_dir);
    let legacy = Path::new(&configured).file_name().is_some_and(|n| n == factoryos_protocol::fitter::LEGACY_SCENES_DIR);
    if configured.is_empty() || legacy {
        Path::new(&store.resolve(&config.asset_root)).join(factoryos_protocol::fitter::SCENES_DIR)
    } else {
        PathBuf::from(configured)
    }
}

/// The cell folder for a `cellDir`: `cellDir` itself, or its parent for the `cellrunner/` layout
/// and a reference cell's view (`<cell>/.view`).
fn cell_root(cell_dir: &str) -> PathBuf {
    let path = PathBuf::from(cell_dir);
    if path.file_name().is_some_and(|name| name == "cellrunner" || name == cell_resolve::VIEW_DIR) {
        path.parent().map(Path::to_path_buf).unwrap_or(path)
    } else {
        path
    }
}

fn same_path(a: &Path, b: &Path) -> bool {
    let canonical = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
    canonical(a) == canonical(b)
}

/// A cell folder has a manifest (`factoryos-cell.json`: a reference cell may hold nothing else), a
/// `scene.yaml` / `scene.factoryos.yaml`, or a `cell.yaml` (at its root or in `cellrunner/`), or is
/// an Agentic Planner cell (`agentic_planner/`).
fn is_cell_dir(dir: &Path) -> bool {
    dir.join(SOURCE_MARKER).is_file()
        || dir.join(SCENE_MARKER).exists()
        || dir.join(FACTORY_OS_SCENE).exists()
        || dir.join(CELL_MARKER).exists()
        || dir.join("cellrunner").join(CELL_MARKER).exists()
        || dir.join("agentic_planner").is_dir()
}

/// The folder holding `cell.yaml`: the cell root, or its `cellrunner/` subfolder.
fn resolve_cell_dir(root: &Path) -> PathBuf {
    if !root.join(CELL_MARKER).exists() && root.join("cellrunner").join(CELL_MARKER).exists() {
        root.join("cellrunner")
    } else {
        root.to_path_buf()
    }
}

pub(crate) fn list_cells(cells_dir: &Path) -> Vec<String> {
    let mut cells: Vec<String> = std::fs::read_dir(cells_dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter(|entry| entry.path().is_dir() && is_cell_dir(&entry.path()))
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        // Not a hidden folder (`.backup/`, an old rebuild in progress `.<cell>.rebuild-<pid>`).
        .filter(|name| !name.starts_with('.'))
        .collect();
    cells.sort();
    cells
}

/// Point the config at the cell in `dir`. The bundle's sample cells keep their own env dir and
/// scene; any other cell is self-contained (it is its own env dir, `scene.yaml` at its root).
fn apply_cell(config: &mut Config, dir: &Path, samples: &[SampleCell]) {
    if let Some(sample) = samples.iter().find(|s| same_path(Path::new(&s.cell_dir), dir)) {
        config.cell_dir.clone_from(&sample.cell_dir);
        config.env_dir.clone_from(&sample.env_dir);
        config.scene.clone_from(&sample.scene);
        return;
    }
    config.cell_dir = resolve_cell_dir(dir).display().to_string();
    config.env_dir = dir.display().to_string();
    if let Some(scene) = [FACTORY_OS_SCENE, SCENE_MARKER].into_iter().find(|scene| dir.join(scene).exists()) {
        config.scene = scene.to_owned();
    }
}

/// The scenes in `dir` (`<name>/scene.yaml`), by name.
fn list_fitted_scenes(dir: &Path) -> Vec<String> {
    super::scene_store::list(dir)
}

/// Scene `scene`'s USDA bundle: a `<scene>*.zip` in its folder (or, legacy, beside the flat file
/// in the scenes folder `dir`) that contains a `.usda` (not the `-urdf.zip` asset bundle).
pub(crate) fn find_usda_bundle(dir: &Path, scene: &str) -> Option<PathBuf> {
    let stem = super::scene_store::name_of(scene)?;
    let mut candidates: Vec<PathBuf> = [super::scene_store::scene_dir(dir, &stem), dir.to_path_buf()]
        .iter()
        .filter_map(|d| std::fs::read_dir(d).ok())
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| {
            let name = path.file_name().unwrap_or_default().to_string_lossy();
            path.is_file() && name.to_lowercase().ends_with(".zip") && name.starts_with(&stem)
        })
        .collect();
    candidates.sort();
    candidates.into_iter().find(|zip| has_usda(zip))
}

/// Whether `zip` is a zip with a `.usda` in it.
pub(crate) fn has_usda(zip: &Path) -> bool {
    std::fs::File::open(zip)
        .ok()
        .and_then(|file| zip::ZipArchive::new(file).ok())
        .is_some_and(|archive| archive.file_names().any(|name| name.is_ok_and(|name| name.to_lowercase().ends_with(".usda"))))
}

pub(crate) fn extract_zip(zip: &Path, dest: &Path) -> anyhow::Result<()> {
    let mut archive = zip::ZipArchive::new(std::fs::File::open(zip)?)?;
    std::fs::create_dir_all(dest)?;
    archive.extract(dest)?;
    Ok(())
}

/// A folder copied (the non-Unix view's stand-in for symlinks; the migration tests' old copies).
#[cfg_attr(all(unix, not(test)), allow(dead_code))]
pub(crate) fn copy_dir(from: &Path, to: &Path) -> anyhow::Result<()> {
    std::fs::create_dir_all(to)?;
    for entry in std::fs::read_dir(from)? {
        let entry = entry?;
        let target = to.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_dir(&entry.path(), &target)?;
        } else {
            std::fs::copy(entry.path(), target)?;
        }
    }
    Ok(())
}

// --- The pure scaffold core (`cell-scaffold.ts`) ---

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CopyKind {
    /// A whole package directory (robots, fixtures).
    Dir,
    /// A single file (floor plan).
    File,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct AssetCopy {
    pub(crate) kind: CopyKind,
    /// As written in the manifest: relative to the manifest file.
    pub(crate) from_rel: String,
    /// Relative to the new cell's root.
    pub(crate) to_rel: String,
}

#[derive(Debug)]
pub(crate) struct ScaffoldPlan {
    /// The manifest, with asset paths rewritten to be cell-relative.
    pub(crate) scene: String,
    pub(crate) copies: Vec<AssetCopy>,
}

/// The three asset keys, and the cell subtree each is copied into.
const ASSET_KEYS: [(&str, CopyKind, &str); 3] = [
    ("dressed_urdf", CopyKind::Dir, "robots"),
    ("urdf", CopyKind::Dir, "fixtures"),
    ("geometry_file", CopyKind::File, "floorplans"),
];

fn base_name(path: &str) -> &str {
    let trimmed = path.trim_end_matches(['/', '\\']);
    trimmed.rsplit(['/', '\\']).next().unwrap_or(trimmed)
}

fn dir_name(path: &str) -> &str {
    let trimmed = path.trim_end_matches(['/', '\\']);
    trimmed.rfind(['/', '\\']).map_or("", |i| &trimmed[..i])
}

fn file_stem(file: &str) -> &str {
    let base = base_name(file);
    match base.rfind('.') {
        Some(dot) if dot > 0 => &base[..dot],
        _ => base,
    }
}

/// The name a catalogue variant has in a cell: `<name>@<cell>` (what `fos-migrate-cells` names a
/// cell's own robot, fixture or floor plan when the catalogue has another of that name) is `<name>`
/// again; a floor plan file keeps its extension (`hall@c.obj` → `hall.obj`).
pub(crate) fn without_variant(name: &str) -> String {
    let strip = |stem: &str| stem.split_once('@').map(|(base, _)| base.to_owned()).filter(|base| !base.is_empty());
    match name.rsplit_once('.') {
        Some((stem, ext)) if !stem.is_empty() => strip(stem).map_or_else(|| name.to_owned(), |base| format!("{base}.{ext}")),
        _ => strip(name).unwrap_or_else(|| name.to_owned()),
    }
}

/// Where one manifest asset goes, and its new (cell-relative) value; `local` is its package's (a
/// floor plan's file's) name in the cell.
fn map_asset(value: &str, kind: CopyKind, dest: &str, local: &str) -> AssetCopy {
    let file = base_name(value);
    match kind {
        CopyKind::Dir => {
            let parent = dir_name(value);
            AssetCopy {
                kind,
                from_rel: parent.to_owned(),
                to_rel: format!("{dest}/{local}/{file}"),
            }
        }
        CopyKind::File => AssetCopy {
            kind,
            from_rel: value.to_owned(),
            to_rel: format!("{dest}/{}/{local}", file_stem(local)),
        },
    }
}

/// The scene with each asset value (`dressed_urdf`, `urdf`, `geometry_file`) for which `f(kind,
/// cell subtree, value)` returns a new one replaced, in its quotes. Line-oriented (the manifest is
/// machine-generated), so comments and formatting survive.
pub(crate) fn rewrite_asset_lines(scene_text: &str, mut f: impl FnMut(CopyKind, &'static str, &str) -> Option<String>) -> String {
    let lines: Vec<String> = scene_text
        .split('\n')
        .map(|line| {
            // `<indent>[- ]<key>: [quote]<value>[quote]`
            let body = line.trim_start();
            let indent = &line[..line.len() - body.len()];
            let (dash, body) = match body.strip_prefix('-') {
                Some(rest) if rest.starts_with(char::is_whitespace) => {
                    let rest_trimmed = rest.trim_start();
                    (&body[..body.len() - rest_trimmed.len()], rest_trimmed)
                }
                _ => ("", body),
            };
            let Some((key, value)) = body.split_once(':') else {
                return line.to_owned();
            };
            let Some(&(_, kind, dest)) = ASSET_KEYS.iter().find(|(k, ..)| *k == key) else {
                return line.to_owned();
            };
            let spacing = &value[..value.len() - value.trim_start().len()];
            let value = value.trim();
            let quote = value.chars().next().filter(|c| *c == '"' || *c == '\'');
            let unquoted = match quote {
                Some(q) if value.len() >= 2 && value.ends_with(q) => &value[1..value.len() - 1],
                _ => value,
            };
            if unquoted.is_empty() {
                return line.to_owned();
            }
            let Some(new_value) = f(kind, dest, unquoted) else {
                return line.to_owned();
            };
            let q = quote.map(String::from).unwrap_or_default();
            format!("{indent}{dash}{key}:{spacing}{q}{new_value}{q}")
        })
        .collect();
    lines.join("\n")
}

/// Rewrite the manifest's asset lines to be cell-relative and collect the copies that realise
/// them. A package keeps its name in the cell, a catalogue variant `<name>@<cell>` its `<name>`
/// (unless another package of the scene has that name).
pub(crate) fn plan_cell_scaffold(scene_text: &str) -> ScaffoldPlan {
    // The packages (folders; a floor plan's file) of each subtree, and their names in the cell.
    let mut packages: Vec<(&'static str, String, String)> = Vec::new();
    rewrite_asset_lines(scene_text, |kind, dest, value| {
        let (package, name) = match kind {
            CopyKind::Dir => (dir_name(value).to_owned(), base_name(dir_name(value)).to_owned()),
            CopyKind::File => (value.to_owned(), base_name(value).to_owned()),
        };
        if !packages.iter().any(|(d, p, _)| *d == dest && *p == package) {
            packages.push((dest, package, name));
        }
        None
    });
    let local = |dest: &str, package: &str| -> String {
        let Some((_, _, name)) = packages.iter().find(|(d, p, _)| *d == dest && p == package) else {
            return base_name(package).to_owned();
        };
        let short = without_variant(name);
        let clash = packages.iter().any(|(d, p, other)| *d == dest && p != package && (*other == short || without_variant(other) == short));
        if clash { name.clone() } else { short }
    };
    let mut copies: Vec<AssetCopy> = Vec::new();
    let scene = rewrite_asset_lines(scene_text, |kind, dest, value| {
        let package = match kind {
            CopyKind::Dir => dir_name(value),
            CopyKind::File => value,
        };
        let copy = map_asset(value, kind, dest, &local(dest, package));
        let new_value = copy.to_rel.clone();
        let package = match kind {
            CopyKind::Dir => dir_name(&copy.to_rel).to_owned(),
            CopyKind::File => copy.to_rel.clone(),
        };
        if !copies.iter().any(|c| c.to_rel == package) {
            copies.push(AssetCopy { to_rel: package, ..copy });
        }
        Some(new_value)
    });
    ScaffoldPlan { scene, copies }
}

fn unquote(value: &str) -> &str {
    let t = value.trim();
    let bytes = t.as_bytes();
    if t.len() >= 2 && (bytes[0] == b'"' || bytes[0] == b'\'') && bytes[t.len() - 1] == bytes[0] {
        &t[1..t.len() - 1]
    } else {
        t
    }
}

/// Every value of `key` in document order (as `key:` or `- key:`).
fn collect_values<'a>(lines: &[&'a str], key: &str) -> Vec<&'a str> {
    let prefix = format!("{key}:");
    lines
        .iter()
        .filter_map(|line| {
            let body = line.trim_start().trim_start_matches("- ").trim_start();
            body.strip_prefix(&prefix).map(unquote)
        })
        .collect()
}

fn first_top_level<'a>(lines: &[&'a str], key: &str) -> &'a str {
    let prefix = format!("{key}:");
    lines
        .iter()
        .find_map(|line| line.strip_prefix(&prefix).map(unquote))
        .unwrap_or("")
}

/// A minimal `cell.yaml`: enough for the cell-runner to boot and serve `/env` so the 3D view
/// renders the fitted scene. The Agentic Planner regenerates it with the full model later.
pub(crate) fn build_scaffold_cell_yaml(scene_text: &str, source_name: &str) -> String {
    let lines: Vec<&str> = scene_text.split('\n').collect();
    let name = Some(first_top_level(&lines, "name")).filter(|n| !n.is_empty()).unwrap_or("Fitted Cell");
    let description = first_top_level(&lines, "description");
    let scene_ids = collect_values(&lines, "robot_id");
    let urdfs = collect_values(&lines, "dressed_urdf");

    let from = if source_name.is_empty() { String::new() } else { format!(" from {source_name}") };
    let mut out = vec![
        format!("# Auto-generated by \"Create Cellrunner Cell\"{from}."),
        "# Minimal scaffold: enough for the cell-runner to boot and serve /env so the 3D".to_owned(),
        "# viewer renders the fitted scene. The Agentic Planner regenerates this file with".to_owned(),
        "# the full cell model (tools, coordination, trajectory plans) later.".to_owned(),
        format!("name: {name}"),
    ];
    if !description.is_empty() {
        out.push(format!("description: {}", serde_json::to_string(description).unwrap_or_default()));
    }
    let robots: Vec<_> = scene_ids.iter().zip(&urdfs).collect();
    if robots.is_empty() {
        out.push("robots: []".to_owned());
    } else {
        out.push("robots:".to_owned());
        for (i, (scene_id, urdf)) in robots.into_iter().enumerate() {
            let n = i + 1;
            out.extend([
                format!("  - id: robot{n}"),
                format!("    scene_id: {scene_id}"),
                format!("    name: Roboter_robot{n}"),
                format!("    urdf: {urdf}"),
            ]);
        }
    }
    out.push(String::new());
    out.join("\n")
}

#[cfg(test)]
mod tests {

    #[test]
    fn product_welds_of_a_cell() {
        let dir = std::env::temp_dir().join(format!("factoryos-cells-products-{}", std::process::id()));
        let fixture = dir.join("fixtures/bug_cube");
        std::fs::create_dir_all(&fixture).unwrap();
        let welds = |dir: &Path| super::product_welds(cell_resolve::resolve_root(dir, Path::new("/nonexistent")).ok().as_ref());
        assert_eq!(welds(&dir), (Vec::new(), 0));
        std::fs::write(
            dir.join("scene.yaml"),
            "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n  \
             position: {x: 0.0, y: 0.0, z: 0.0}\n  orientation: {a: 0.0, b: 0.0, c: 0.0}\n",
        )
        .unwrap();
        let op = |id: &str, x: f32| {
            format!(r#"{{"id": "{id}", "target": {{"kind": "point", "position": [{x}, 0.5, 2.0], "normal": [0, 0, 1]}}}}"#)
        };
        std::fs::write(
            fixture.join("product.json"),
            format!(
                r#"{{"name": "bug_cube", "geometry": {{"kind": "box", "size": [2, 2, 2]}}, "tasks": [{{"name": "spot_weld_1",
                "process": "spotWeld", "operations": [{}, {}, {{"id": "S1", "target": {{"kind": "path", "points": [], "normal": [0, 0, 1]}}}}]}}]}}"#,
                op("W01", -0.5),
                op("W02", 0.5)
            ),
        )
        .unwrap();
        assert_eq!(welds(&dir), (vec!["bug_cube".to_owned()], 2));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn activating_a_cell_prefers_its_factory_os_scene() {
        let dir = std::env::temp_dir().join(format!("factoryos-cells-scene-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("scene.yaml"), "robots: []\n").unwrap();
        let mut config = Config::default();
        apply_cell(&mut config, &dir, &[]);
        assert_eq!(config.scene, "scene.yaml");
        // An umarell cell: scene.yaml is the planner's format, scene.factoryos.yaml the cell-runner's.
        std::fs::write(dir.join("scene.factoryos.yaml"), "name: x\nrobots: []\n").unwrap();
        apply_cell(&mut config, &dir, &[]);
        assert_eq!(config.scene, "scene.factoryos.yaml");
        std::fs::remove_dir_all(&dir).ok();
    }

    use super::*;

    const MANIFEST: &str = r#"name: two-robots
description: "A test: two robots"
robots:
  - robot_id: r1
    dressed_urdf: ../urdf-dressed-robots/kr240__weld/kr240__weld.urdf
    position: {x: 0, y: 0, z: 0}
  - robot_id: r2
    dressed_urdf: "../urdf-dressed-robots/kr240__weld/kr240__weld.urdf"
fixtures:
  - id: box
    urdf: ../urdf-fixture-collection/weld_fixture_box/weld_fixture_box.urdf
floor_plan:
  geometry_file: ../floor-plans/hall.obj
  scale: 1.0
"#;

    #[test]
    fn scaffold_rewrites_assets_and_dedupes_copies() {
        let plan = plan_cell_scaffold(MANIFEST);
        assert!(plan.scene.contains("    dressed_urdf: robots/kr240__weld/kr240__weld.urdf\n"));
        assert!(plan.scene.contains(r#"    dressed_urdf: "robots/kr240__weld/kr240__weld.urdf""#));
        assert!(plan.scene.contains("    urdf: fixtures/weld_fixture_box/weld_fixture_box.urdf"));
        assert!(plan.scene.contains("  geometry_file: floorplans/hall/hall.obj"));
        assert!(plan.scene.contains("    position: {x: 0, y: 0, z: 0}"), "other lines untouched");
        assert_eq!(
            plan.copies,
            vec![
                AssetCopy { kind: CopyKind::Dir, from_rel: "../urdf-dressed-robots/kr240__weld".into(), to_rel: "robots/kr240__weld".into() },
                AssetCopy {
                    kind: CopyKind::Dir,
                    from_rel: "../urdf-fixture-collection/weld_fixture_box".into(),
                    to_rel: "fixtures/weld_fixture_box".into()
                },
                AssetCopy { kind: CopyKind::File, from_rel: "../floor-plans/hall.obj".into(), to_rel: "floorplans/hall/hall.obj".into() },
            ]
        );
    }

    #[test]
    fn a_catalogue_variant_keeps_its_name_in_the_cell() {
        assert_eq!(without_variant("kr240@welding_cell_2_small"), "kr240");
        assert_eq!(without_variant("hall@w.obj"), "hall.obj");
        assert_eq!(without_variant("hall.obj"), "hall.obj");
        assert_eq!(without_variant("@x"), "@x");
        let scene = "robots:\n- dressed_urdf: ../urdf-dressed-robots/kr@w/kr.urdf\n- dressed_urdf: ../urdf-dressed-robots/kr@w/kr.urdf\n\
                     fixtures:\n- urdf: ../urdf-fixture-collection/t@w/t.urdf\n- urdf: ../urdf-fixture-collection/t/t.urdf\n\
                     floor_plan:\n  geometry_file: ../floor-plans/hall@w.obj\n";
        let plan = plan_cell_scaffold(scene);
        assert!(plan.scene.contains("- dressed_urdf: robots/kr/kr.urdf\n- dressed_urdf: robots/kr/kr.urdf\n"), "{}", plan.scene);
        assert!(plan.scene.contains("- urdf: fixtures/t@w/t.urdf\n- urdf: fixtures/t/t.urdf\n"), "a clash keeps the variant's name: {}", plan.scene);
        assert!(plan.scene.contains("geometry_file: floorplans/hall/hall.obj"), "{}", plan.scene);
        let to: Vec<&str> = plan.copies.iter().map(|c| c.to_rel.as_str()).collect();
        assert_eq!(to, ["robots/kr", "fixtures/t@w", "fixtures/t", "floorplans/hall/hall.obj"]);
        assert_eq!(plan.copies[0].from_rel, "../urdf-dressed-robots/kr@w");
    }

    #[test]
    fn scaffold_cell_yaml_lists_robots() {
        let plan = plan_cell_scaffold(MANIFEST);
        let yaml = build_scaffold_cell_yaml(&plan.scene, "two.yaml");
        assert!(yaml.starts_with("# Auto-generated by \"Create Cellrunner Cell\" from two.yaml."));
        assert!(yaml.contains("name: two-robots\n"));
        assert!(yaml.contains("description: \"A test: two robots\"\n"));
        assert!(yaml.contains("  - id: robot2\n    scene_id: r2\n    name: Roboter_robot2\n    urdf: robots/kr240__weld/kr240__weld.urdf\n"));
        assert!(build_scaffold_cell_yaml("name: empty\n", "").contains("robots: []"));
    }

    #[test]
    fn plans_name_their_planner() {
        let root = std::env::temp_dir().join(format!("factoryos-cells-planner-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cells/welding_cell_2_small");
        let plans_dir = cell.join(PLANS_DIR);
        std::fs::create_dir_all(&plans_dir).unwrap();
        let store = ConfigStore::open(root.join("factoryos.config.json"));
        let mut config = store.get();
        config.cell_dir = cell.display().to_string();
        store.set(config);
        // umarell's own plan (no `planner`), and the planner router's, published unverified
        let umarell = serde_json::json!({ "tree_id": "bt_1_c", "created_s": 1.0, "status": "ok", "behaviour_tree": {} });
        let policy = serde_json::json!({ "tree_id": "bt_2_c", "created_s": 2.0, "status": "unverified",
            "planner": "hier_q", "planner_meta": { "profile": "umarell" }, "behaviour_tree": {} });
        std::fs::write(plans_dir.join("bt_1_c.json"), umarell.to_string()).unwrap();
        std::fs::write(plans_dir.join("bt_2_c.json"), policy.to_string()).unwrap();
        let listed = plans(&store).plans;
        let got: Vec<_> = listed.iter().map(|p| (p.tree_id.as_str(), p.planner.as_str(), p.status.as_str())).collect();
        assert_eq!(got, [("bt_2_c", "hier_q", "unverified"), ("bt_1_c", "umarell", "ok")]);
        // A client of an older backend: the fields default to empty.
        let old: PlanInfo = serde_json::from_str(r#"{"path":"p","tree_id":"t","created_s":1,"num_operations":2}"#).unwrap();
        assert_eq!((old.planner.as_str(), old.status.as_str()), ("", ""));
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn cells_derive_their_task_programs_from_their_products() {
        let root = std::env::temp_dir().join(format!("factoryos-cells-product-tasks-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (assets, cells, products) = (root.join("assets"), root.join("cells"), root.join("products"));
        std::fs::create_dir_all(assets.join("scenes")).unwrap();
        std::fs::create_dir_all(products.join("bug_cube")).unwrap();
        std::fs::create_dir_all(&cells).unwrap();
        let product = std::fs::read_to_string(super::super::reach::task::tests::fixtures().join("bug_cube/product.json")).unwrap();
        std::fs::write(products.join("bug_cube/product.json"), &product).unwrap();
        std::fs::write(products.join("bug_cube/bug_cube.urdf"), "<robot/>").unwrap();
        std::fs::create_dir_all(assets.join("scenes/jam2")).unwrap();
        std::fs::write(
            assets.join("scenes/jam2/scene.yaml"),
            "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: ../../../products/bug_cube/bug_cube.urdf\n  \
             position: {x: 0.0, y: 0.0, z: 0.0}\n  orientation: {a: 0.0, b: 0.0, c: 0.0}\n",
        )
        .unwrap();
        let store = ConfigStore::open(root.join("factoryos.config.json"));
        let mut config = store.get();
        config.cells_dir = cells.display().to_string();
        config.asset_root = assets.display().to_string();
        store.set(config);

        // A legacy cell, copied by the old Create Cell: its own scene.yaml and the product's copy.
        let cell = cells.join("jam2");
        std::fs::create_dir_all(cell.join("fixtures/bug_cube")).unwrap();
        std::fs::write(cell.join("fixtures/bug_cube/product.json"), &product).unwrap();
        std::fs::write(cell.join("fixtures/bug_cube/bug_cube.urdf"), "<robot/>").unwrap();
        std::fs::write(
            cell.join("scene.yaml"),
            "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n  \
             position: {x: 0.0, y: 0.0, z: 0.0}\n  orientation: {a: 0.0, b: 0.0, c: 0.0}\n",
        )
        .unwrap();
        std::fs::write(cell.join(SOURCE_MARKER), r#"{"fitted_scene": "jam2.yaml"}"#).unwrap();
        // "Use as cell" on its scene uses it, and derives the product's task.
        create_from_fitted(&store, "jam2.yaml", "").unwrap();
        let info = plans(&store);
        assert_eq!(info.tasks, ["bug_cube__spot_weld_1"]);
        assert_eq!((info.products.as_slice(), info.product_welds), (["bug_cube".to_owned()].as_slice(), 4));
        let source = &info.task_sources[0];
        assert_eq!((source.file.as_str(), source.product.as_deref(), source.stale, source.edited), ("bug_cube__spot_weld_1", Some("bug_cube"), false, false));

        // Saving the product (PRODUCT_SAVE) updates the cells using it (here: its copy).
        let moved = product.replace("[-0.5, -0.5, 2.0]", "[-0.25, -0.5, 2.0]");
        std::fs::write(products.join("bug_cube/product.json"), &moved).unwrap();
        std::fs::write(cell.join("fixtures/bug_cube/product.json"), &moved).unwrap();
        assert!(plans(&store).task_sources[0].stale);
        assert_eq!(cells_using_product(&store, "bug_cube"), std::slice::from_ref(&cell));
        assert!(cells_using_product(&store, "other").is_empty());
        assert!(derive_tasks(&store, &cell).is_some());
        assert_eq!(std::fs::read_to_string(cell.join("fixtures/bug_cube/product.json")).unwrap(), moved);
        let program: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(cell.join(TASKS_DIR).join("bug_cube__spot_weld_1.json")).unwrap()).unwrap();
        assert_eq!(program["operations"][0]["position"], serde_json::json!([-0.25, -0.5, 2.0]));
        assert!(!plans(&store).task_sources[0].stale);
        assert_eq!(derive_tasks(&store, &cell), None, "up to date");

        // A product without welds yet is listed too.
        let no_welds = product.replace("\"point\"", "\"path\", \"points\": []");
        std::fs::write(products.join("bug_cube/product.json"), &no_welds).unwrap();
        std::fs::write(cell.join("fixtures/bug_cube/product.json"), &no_welds).unwrap();
        derive_tasks(&store, &cell);
        let info = plans(&store);
        assert_eq!((info.products, info.product_welds, info.tasks.len()), (vec!["bug_cube".to_owned()], 0, 0));
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn create_and_select_a_cell() {
        let root = std::env::temp_dir().join(format!("factoryos-cells-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let assets = root.join("assets");
        for dir in ["scenes", "urdf-dressed-robots/kr240__weld/meshes", "urdf-fixture-collection/weld_fixture_box", "floor-plans"] {
            std::fs::create_dir_all(assets.join(dir)).unwrap();
        }
        // Scene two, its paths as the flat layout had them (from scenes/): they resolve through
        // the legacy fallback. A flat file is no scene (until the start's migration).
        std::fs::create_dir_all(assets.join("scenes/two")).unwrap();
        std::fs::write(assets.join("scenes/two/scene.yaml"), MANIFEST).unwrap();
        std::fs::write(assets.join("scenes/flat.yaml"), MANIFEST).unwrap();
        std::fs::create_dir_all(assets.join("scenes/no-scene")).unwrap();
        std::fs::write(assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.urdf"), "<robot/>").unwrap();
        std::fs::write(assets.join("urdf-dressed-robots/kr240__weld/meshes/a.obj"), "o").unwrap();
        std::fs::write(assets.join("urdf-fixture-collection/weld_fixture_box/weld_fixture_box.urdf"), "<robot/>").unwrap();
        std::fs::write(assets.join("floor-plans/hall.obj"), "o").unwrap();
        std::fs::write(assets.join("floor-plans/hall.mtl"), "m").unwrap();
        let cells = root.join("cells");
        std::fs::create_dir_all(cells.join("existing")).unwrap();
        std::fs::write(cells.join("existing/cell.yaml"), "name: x\n").unwrap();

        let store = ConfigStore::open(root.join("factoryos.config.json"));
        let mut config = store.get();
        config.cells_dir = cells.display().to_string();
        config.asset_root = assets.display().to_string();
        store.set(config);

        let before = info(&store);
        assert_eq!(before.cells, vec!["existing"]);
        assert_eq!(before.fitted_scenes, vec!["two"]);

        assert!(create_from_fitted(&store, "two.yaml", "bad-name").is_err());
        assert!(create_from_fitted(&store, "two.yaml", "existing").is_err(), "another cell's folder");
        let config = create_from_fitted(&store, "two.yaml", "new_cell").unwrap();
        let cell = cells.join("new_cell");
        // A reference cell: nothing copied; the cell-runner runs its view.
        assert!(!cell.join("robots").exists() && !cell.join("scene.yaml").exists() && !cell.join("cell.yaml").exists());
        let view = cell.join(cell_resolve::VIEW_DIR);
        assert_eq!(config.env_dir, view.display().to_string());
        assert_eq!(config.cell_dir, view.display().to_string());
        assert_eq!(config.scene, "scene.yaml");
        assert!(view.join("robots/kr240__weld/meshes/a.obj").is_file());
        assert!(view.join("fixtures/weld_fixture_box/weld_fixture_box.urdf").is_file());
        assert!(view.join("floorplans/hall/hall.mtl").is_file());
        assert!(std::fs::read_to_string(view.join("cell.yaml")).unwrap().contains("scene_id: r1"));
        assert_eq!(create_from_fitted(&store, "two.yaml", "other_name").unwrap(), config, "the scene's cell, again");
        assert!(!cells.join("other_name").exists());

        // What the Robo Fitter's link and the behaviour-tree executor read from the new cell.
        assert_eq!(active_cell(&store), Some(("new_cell".to_owned(), cell.clone())));
        assert_eq!(fitted_source(&cell).as_deref(), Some("two"));
        let ids = robot_scene_ids(&view);
        assert_eq!(ids.first(), Some(&("robot1".to_owned(), "r1".to_owned())));
        assert!(plans(&store).plans.is_empty());
        let plans_dir = cell.join(PLANS_DIR);
        std::fs::create_dir_all(&plans_dir).unwrap();
        let plan = |id: &str, created: f64| {
            serde_json::json!({ "tree_id": id, "created_s": created, "num_operations": 4, "behaviour_tree": {} }).to_string()
        };
        std::fs::write(plans_dir.join("bt_1_new_cell.json"), plan("bt_1_new_cell", 1.0)).unwrap();
        std::fs::write(plans_dir.join("bt_2_new_cell.json"), plan("bt_2_new_cell", 2.0)).unwrap();
        std::fs::write(plans_dir.join("quality.json"), "{}").unwrap();
        let listed = plans(&store);
        assert_eq!(listed.cell, "new_cell");
        let ids: Vec<_> = listed.plans.iter().map(|p| p.tree_id.as_str()).collect();
        assert_eq!(ids, ["bt_2_new_cell", "bt_1_new_cell"], "newest first, only plans");
        assert_eq!(listed.plans[0].path, "agentic_planner/plans/bt_2_new_cell.json");
        assert_eq!(
            plan_load_params(&store, &listed.plans[0].path).unwrap(),
            serde_json::json!({ "path": "agentic_planner/plans/bt_2_new_cell.json" })
        );
        assert!(plan_load_params(&store, "agentic_planner/plans/../../../etc/passwd").is_err());
        // The whole plan for the behaviour-tree pane, with an ETag that a client's copy matches.
        let PlanFile::Plan { etag, bytes } = read_plan(&store, "bt_2_new_cell", None).unwrap() else {
            panic!("no plan read");
        };
        assert_eq!(bytes, plan("bt_2_new_cell", 2.0).into_bytes());
        assert!(etag.starts_with("W/\""), "{etag}");
        assert_eq!(read_plan(&store, "bt_2_new_cell", Some(&etag)).unwrap(), PlanFile::NotModified { etag: etag.clone() });
        let listed_tag = format!("\"other\", {}", etag.trim_start_matches("W/"));
        assert!(matches!(read_plan(&store, "bt_2_new_cell", Some(&listed_tag)).unwrap(), PlanFile::NotModified { .. }));
        assert!(matches!(read_plan(&store, "bt_2_new_cell", Some("\"stale\"")).unwrap(), PlanFile::Plan { .. }));
        // Changed on disk (size and so ETag): read again.
        std::fs::write(plans_dir.join("bt_2_new_cell.json"), plan("bt_2_new_cell_v2", 2.0)).unwrap();
        assert!(matches!(read_plan(&store, "bt_2_new_cell", Some(&etag)).unwrap(), PlanFile::Plan { .. }));
        for bad in ["../../cell", "..", "a/b", "", "x%2F..", "bt_2_new_cell.json/.."] {
            assert!(read_plan(&store, bad, None).is_err(), "{bad}");
        }
        assert!(read_plan(&store, "missing", None).is_err());
        // Task programs (what "Plan Cell" plans) and whether the cell is prepared.
        assert_eq!(listed.tasks, Vec::<String>::new());
        assert!(!listed.prepared, "not prepared until the planner's cell.yaml exists");
        std::fs::write(cell.join("agentic_planner/cell.yaml"), "name: x\n").unwrap();
        assert!(plans(&store).prepared);

        // Task programs: saved, listed, read back; invalid ones are refused.
        let task = |name: &str, text: &str| TaskProgram { name: name.to_owned(), text: text.to_owned() };
        assert!(write_task(&store, &task("weld_edges", r#"{"operations": []}"#)).is_ok());
        assert!(write_task(&store, &task("../escape", r#"{"operations": []}"#)).is_err());
        assert!(write_task(&store, &task("broken", "{")).is_err());
        assert!(write_task(&store, &task("no_ops", "{}")).is_err());
        std::fs::write(cell.join(TASKS_DIR).join("notes.txt"), "x").unwrap();
        assert_eq!(plans(&store).tasks, vec!["weld_edges".to_owned()]);
        assert_eq!(read_task(&store, "weld_edges").unwrap().text, r#"{"operations": []}"#);
        let sample = sample_task(&store);
        let sample_json: serde_json::Value = serde_json::from_str(&sample.text).unwrap();
        assert_eq!(sample.name, "cube_top_edges");
        assert_eq!(sample_json["cell_id"], "new_cell");
        assert_eq!(sample_json["operations"].as_array().map(Vec::len), Some(12));

        let after = info(&store);
        assert_eq!(after.cells, vec!["existing", "new_cell"]);
        assert_eq!(after.active, "new_cell");
        let kinds: Vec<_> = after.entries.iter().map(|e| (e.name.as_str(), e.scene.as_deref(), e.kind)).collect();
        assert_eq!(kinds, [("existing", None, CellKind::Legacy), ("new_cell", Some("two"), CellKind::Reference)]);
        let listed = plans(&store);
        assert_eq!((listed.kind, listed.scene.as_deref(), listed.out_of_date_with.as_deref()), (CellKind::Reference, Some("two"), None));

        let config = set_active(&store, "existing").unwrap();
        assert_eq!(config.cell_dir, cells.join("existing").display().to_string());
        assert_eq!(info(&store).active, "existing");
        assert!(set_active(&store, "missing").is_err());
        std::fs::remove_dir_all(&root).ok();
    }

    /// The user's case: a cell of new-scene.yaml with the product mycube; the scene then gets
    /// big_cube instead and is saved. Nothing is rebuilt or deleted: the prepared model and the
    /// plans made before are stale, with the reason.
    #[test]
    fn saving_a_scene_marks_its_cells_outputs_stale() {
        let root = std::env::temp_dir().join(format!("factoryos-cells-stale-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (assets, products, cells) = (root.join("assets"), root.join("products"), root.join("cells"));
        for dir in ["scenes", "urdf-dressed-robots/kr240__weld"] {
            std::fs::create_dir_all(assets.join(dir)).unwrap();
        }
        std::fs::write(assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.urdf"), "<robot/>").unwrap();
        for product in ["mycube", "big_cube"] {
            std::fs::create_dir_all(products.join(product)).unwrap();
            std::fs::write(products.join(format!("{product}/{product}.urdf")), "<robot/>").unwrap();
        }
        std::fs::create_dir_all(&cells).unwrap();
        let scene = |product: &str| {
            format!(
                "name: new-scene\nrobots:\n- robot_id: r1\n  dressed_urdf: ../../urdf-dressed-robots/kr240__weld/kr240__weld.urdf\n  \
                 position: {{x: 0, y: 0, z: 0}}\nfixtures:\n- id: {product}\n  urdf: {}/{product}/{product}.urdf\n",
                products.display()
            )
        };
        std::fs::create_dir_all(assets.join("scenes/new-scene")).unwrap();
        let fitted = assets.join("scenes/new-scene/scene.yaml");
        std::fs::write(&fitted, scene("mycube")).unwrap();
        let store = ConfigStore::open(root.join("factoryos.config.json"));
        let mut config = store.get();
        config.cells_dir = cells.display().to_string();
        config.asset_root = assets.display().to_string();
        store.set(config);

        create_from_fitted(&store, "new-scene.yaml", "").unwrap();
        let cell = cells.join("new_scene");
        assert_eq!(made_from(&store, "new-scene.yaml"), ["new_scene"]);
        assert_eq!(out_of_date_with(&store, &cell), None, "a reference cell is never out of date");
        // Prepared and planned, each recording its inputs; a task program.
        let resolved = cell_resolve::resolve(&store, "new_scene").unwrap();
        for dir in ["agentic_planner/config", "agentic_planner/plans", "agentic_planner/tasks"] {
            std::fs::create_dir_all(cell.join(dir)).unwrap();
        }
        std::fs::write(cell.join("agentic_planner/cell.yaml"), "name: x\n").unwrap();
        cell_resolve::record_prepared(&resolved).unwrap();
        std::fs::write(cell.join("agentic_planner/tasks/welds.json"), r#"{"operations": []}"#).unwrap();
        let plan = serde_json::json!({ "tree_id": "bt_1_c", "created_s": 1.0, "behaviour_tree": {},
            "planner_meta": { "inputs": resolved.inputs } });
        std::fs::write(cell.join("agentic_planner/plans/bt_1_c.json"), plan.to_string()).unwrap();
        let old = serde_json::json!({ "tree_id": "bt_0_c", "created_s": 0.5, "behaviour_tree": {} });
        std::fs::write(cell.join("agentic_planner/plans/bt_0_c.json"), old.to_string()).unwrap();
        let listed = plans(&store);
        assert!(listed.prepared && listed.prepared_stale_reason.is_none());
        assert!(listed.plans.iter().all(|p| !p.stale && p.stale_reason.is_none()), "{:?}", listed.plans);
        let entry = info(&store).entries.into_iter().find(|e| e.name == "new_scene").unwrap();
        assert_eq!((entry.prepared_stale, entry.stale_reason), (false, None));

        // The scene is saved with big_cube.
        std::fs::write(&fitted, scene("big_cube")).unwrap();
        let listed = plans(&store);
        assert!(listed.prepared, "nothing deleted");
        let reason = "the scene new-scene changed; fixture big_cube was added; fixture mycube was removed";
        assert_eq!(listed.prepared_stale_reason.as_deref(), Some(reason));
        assert_eq!(listed.out_of_date_with, None);
        assert_eq!(listed.tasks, ["welds"], "task programs stay");
        let stale: Vec<_> = listed.plans.iter().map(|p| (p.tree_id.as_str(), p.stale, p.stale_reason.as_deref())).collect();
        assert_eq!(stale, [("bt_1_c", true, Some(reason)), ("bt_0_c", false, None)], "a plan without recorded inputs: unknown");
        let entry = info(&store).entries.into_iter().find(|e| e.name == "new_scene").unwrap();
        assert_eq!((entry.prepared_stale, entry.stale_reason.as_deref()), (true, Some(reason)));
        // The view follows the scene once asked for (set active, Load & Play, …).
        assert!(cell.join(".view/fixtures/mycube").exists());
        refresh_active_view(&store).unwrap();
        assert!(cell.join(".view/fixtures/big_cube/big_cube.urdf").is_file());
        assert!(!cell.join(".view/fixtures/mycube").exists());
        assert_eq!(list_cells(&cells), ["new_scene"]);

        // A legacy copy of the scene: out of date with it, and listed so.
        let copy = cells.join("cell_new_scene");
        std::fs::create_dir_all(&copy).unwrap();
        std::fs::write(copy.join(SOURCE_MARKER), r#"{"fitted_scene": "new-scene.yaml"}"#).unwrap();
        std::fs::write(copy.join("scene.yaml"), plan_cell_scaffold(&scene("mycube")).scene).unwrap();
        assert_eq!(out_of_date_with(&store, &copy).as_deref(), Some("new-scene"));
        let entry = info(&store).entries.into_iter().find(|e| e.name == "cell_new_scene").unwrap();
        assert_eq!(entry.kind, CellKind::Legacy);
        assert_eq!(entry.stale_reason.as_deref(), Some("copied from new-scene, which changed since"));
        // The scene's reference cell still comes first.
        assert_eq!(cell_resolve::cell_for_scene(&store, "new-scene.yaml").unwrap(), "new_scene");
        std::fs::remove_dir_all(&root).ok();
    }

    /// A reference cell's task programs come from its scene's products: the products folder's own
    /// files (no copy in the cell), placed by the Robo Fitter scene.
    #[test]
    fn a_reference_cell_derives_its_task_programs_from_the_products_folder() {
        let h = cell_resolve::tests::host("derive-reference");
        let product = std::fs::read_to_string(super::super::reach::task::tests::fixtures().join("bug_cube/product.json")).unwrap();
        std::fs::write(h.products.join("bug_cube/product.json"), &product).unwrap();
        let r = cell_resolve::ensure_cell(&h.store, "jam2.yaml").unwrap();
        assert!(derive_tasks(&h.store, &r.root).is_some());
        assert!(r.root.join(TASKS_DIR).join("bug_cube__spot_weld_1.json").is_file());
        assert!(!r.root.join("fixtures").exists(), "nothing copied");
        assert_eq!(std::fs::read_to_string(h.products.join("bug_cube/product.json")).unwrap(), product, "the product is not written");
        assert_eq!(cells_using_product(&h.store, "bug_cube"), [r.root.clone()]);
        set_active(&h.store, "jam2").unwrap();
        let info = plans(&h.store);
        assert_eq!(info.tasks, ["bug_cube__spot_weld_1"]);
        assert_eq!((info.task_sources[0].product.as_deref(), info.task_sources[0].stale), (Some("bug_cube"), false));
        // The product's welds change: stale until derived again.
        std::fs::write(h.products.join("bug_cube/product.json"), product.replace("[-0.5, -0.5, 2.0]", "[-0.25, -0.5, 2.0]")).unwrap();
        assert!(plans(&h.store).task_sources[0].stale);
        assert!(derive_tasks(&h.store, &r.root).is_some());
        assert!(!plans(&h.store).task_sources[0].stale);
    }
}
