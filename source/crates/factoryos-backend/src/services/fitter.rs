//! Robo Fitter, replacing the Kiro extension's `robo-fitting-room` 2D web editor.
//!
//! The viewer edits the scene (web editor YAML format) and puts every edit on
//! `factoryos/fitter/scene`; the `robo_fitting_room` library parses/validates it, loads the floor
//! plan and composes the USDA zip. The scene is shown in 3D as a renderer-neutral scene on
//! `factoryos/scene/fitter` ([`factoryos_protocol::scene`]): every dressed robot and fixture URDF
//! at its pose (`robots/<id>`, `fixtures/<id>`; robots at their home pose), with pedestals
//! (`<key>/pedestal`), the floor-plan lines (`floor_plan/{lines,footprints}`), the task and the
//! reach domes (`world/task`, `world/reach`, see [`draw`]) and a cell's panel (`world/product`).
//!
//! "Use as cell" (or "Open Cell") links the 3D view to a cell: while linked, the robots follow
//! the simulated cell-runner's joints (`factoryos/sim/state`), e.g. while it plays a behaviour tree
//! (step 1 Load & Play). Never the live cell's (`factoryos/cell/state`: Watch the cell shows that).
//! The view's title says which it is ([`view_title`]): "Cell <cell>" or "Robo Fitter · <scene>".

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};

use axum::extract::State;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::config::Config;
use factoryos_protocol::fitter::{
    ComposeReport, ComposeRequest, CreateCellRequest, FitterState, FitterView, LoadRequest, NEW_SCENE_FILE, OpenCellRequest,
    Palette, ReachCheckRequest, ReachMode, ReachPoseRequest, ReachReport, SceneEdit, SceneFile, SourceCell,
};
use factoryos_protocol::fitter::{Abc, PedestalYaml, Xyz};
use factoryos_protocol::scene::{ModelJoints, ModelRef, Node, NodeKind, Pose3, SceneDoc, ViewHints, is_under};
use factoryos_protocol::{api, keys};
use robo_fitting_room::{ComposeOptions, SceneDefinition};

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::assets::AssetIndex;
use crate::services::pose3::pose3_from_xyz_abc_deg;
use crate::services::reach::{self, Analysis, CheckOptions, ReachContext, draw};
use crate::services::scene_paths::{self, Originals};
use crate::services::scene_store;
use crate::services::scene_pub::ScenePublisher;
use crate::services::urdf_model::UrdfModel;

/// The panel under a cell's welds (see [`panel_node`]).
const PRODUCT: &str = "world/product";

/// A URDF (robot or fixture) placed in the 3D scene.
struct Placed {
    urdf: PathBuf,
    /// Loaded in the background; `None` until then (the scene has no node for it yet).
    loaded: Option<Arc<Loaded>>,
}

/// A placed URDF, parsed and with its assets registered.
struct Loaded {
    urdf: UrdfModel,
    model: ModelRef,
}

/// Joint values (radians / metres) by joint name.
type JointMap = BTreeMap<String, f64>;

pub struct FitterService {
    bus: Bus,
    config: Arc<ConfigStore>,
    /// The 3D scene (`factoryos/scene/fitter`).
    scene: Arc<ScenePublisher>,
    assets: Arc<AssetIndex>,
    inner: Mutex<Inner>,
    /// The latest edit not yet applied; edits arriving faster than the 3D view updates coalesce.
    pending_edit: Arc<(Mutex<Option<SceneEdit>>, Condvar)>,
    /// Robot models (chain, reach dome, TCP) and where the reachability check's IK runs.
    reach: Arc<ReachContext>,
    /// Told the file of each saved scene (main.rs: written through to the storage).
    on_saved: Mutex<Option<Box<dyn Fn(&Path) + Send + Sync>>>,
}

#[derive(Default)]
struct Inner {
    state: FitterState,
    root: PathBuf,
    /// The 3D scene's id, once [`FitterService::open`] started one.
    scene_id: Option<String>,
    placed: HashMap<String, Placed>,
    /// Last scene shown in 3D, to only send what changed.
    shown: Option<SceneFile>,
    link: Option<CellLink>,
    /// The folder the scene's relative paths start from, if not the scene's own folder (a cell's scene).
    scene_base: Option<PathBuf>,
    /// The 3D view's title as last sent (see [`view_title`]).
    layout_title: Option<String>,
    /// The cell the simulation runs (`factoryos/sim/status`), if it runs.
    sim_cell: Option<String>,
    /// Product & Task's products folder, whose products the palette offers as fixtures.
    products_dir: PathBuf,
    /// The task, robots and dome estimate last shown (see [`FitterService::log_task_and_reach`]).
    analysis: Option<Arc<Analysis>>,
    /// Robots shown at a weld ([`FitterState::reach_pose`]): node id → joint values by name.
    /// Before home pose and the cell's joints.
    reach_pose: HashMap<String, JointMap>,
    /// Cancels the running reachability check.
    reach_cancel: Arc<AtomicBool>,
    reach_job: Option<std::thread::JoinHandle<()>>,
    /// Which check's progress is current (a new scene or check makes older ones stale).
    reach_generation: u64,
    /// What the domes were last drawn for (see [`draw::reach_key`]), so edits that don't move them
    /// don't send them again.
    reach_drawn: Option<u64>,
}

/// The cell the scene was turned into; the 3D view follows its robots.
struct CellLink {
    cell: String,
    /// The cell's folder.
    root: PathBuf,
    /// Cell-runner robot id (`robot1`, …) → the robot's node id (`robots/<robot_id>`).
    robots: HashMap<String, String>,
    /// Last joints shown per node id (radians / metres, URDF order of the actuated joints), to
    /// only send what moved.
    shown: HashMap<String, Vec<f64>>,
}

impl Inner {
    fn scenes_dir(&self) -> PathBuf {
        self.root.join(factoryos_protocol::fitter::SCENES_DIR)
    }

    /// Where the scene's relative asset paths start: its folder `scenes/<name>/` (one at the
    /// same depth before it is saved, [`scene_store::base_for`]), or the cell of "Open Cell".
    fn scene_base(&self) -> PathBuf {
        self.scene_base.clone().unwrap_or_else(|| scene_store::base_for(&self.scenes_dir(), &self.state.scene_file_name))
    }

    /// A name for a new scene: `scene`, else `scene-2`, … (see [`scene_store::unique_name`]).
    fn fresh_name(&self) -> String {
        scene_store::unique_name(&self.scenes_dir(), NEW_SCENE_FILE)
    }

    /// The scene as the fitting-room library sees it (positions shifted, floor plan loaded).
    fn definition(&self) -> anyhow::Result<SceneDefinition> {
        let yaml = serde_yaml::to_string(&self.state.scene)?;
        Ok(robo_fitting_room::parse_scene_from_yaml(&yaml, &self.scene_base())?)
    }

    /// The scene [`FitterService::compose`] composes: the `manifest` as saved, the file the zip
    /// records, so every key of it reaches the composer (a robot's `payload`, the floor plan's
    /// `footprint_layers`, keys this backend doesn't model). A cell's own scene has no file in
    /// `scenes/`: the scene as shown.
    fn compose_definition(&self, manifest: &Path) -> anyhow::Result<SceneDefinition> {
        if self.state.cell_scene {
            return self.definition();
        }
        let yaml = std::fs::read_to_string(manifest).map_err(|err| anyhow::anyhow!("reading {}: {err}", manifest.display()))?;
        Ok(robo_fitting_room::parse_scene_from_yaml(&yaml, &self.scene_base())?)
    }

    /// Save the scene as `scenes/<name>/scene.yaml` ([`scene_store`]), `<name>` the scene's
    /// `name` (sanitised; else the name it has), its paths to a cell's copies rewritten to the
    /// originals and its relative paths from its folder (see [`scene_paths`]); the editor adopts
    /// the scene as saved. Renaming a saved scene moves its folder, unless `in_use(old)` (a cell
    /// references it): then the scene is saved as a new one and the old one kept. A name another
    /// scene has gets `-2`, …. Returns the file and what to add to the status.
    fn save_scene(&mut self, originals: &Originals, in_use: &dyn Fn(&str) -> bool) -> anyhow::Result<(PathBuf, String)> {
        anyhow::ensure!(!self.state.cell_scene, "a cell's own scene is not saved from the Robo Fitter");
        let scenes_dir = self.scenes_dir();
        // The scene's folder now (`scene_file_name`), if it was saved; renaming moves it only when
        // this editor has it open as saved (not another scene that took the name meanwhile).
        let current = scene_store::name_of(&self.state.scene_file_name);
        let old = current.clone().filter(|name| {
            scene_store::exists(&scenes_dir, name)
                && self.state.saved.as_ref().and_then(|saved| scene_store::name_of(&saved.file_name)).as_ref() == Some(name)
        });
        let wanted = scene_store::sanitize(&self.state.scene.name)
            .or_else(|| current.clone())
            .ok_or_else(|| anyhow::anyhow!("give the scene a name (letters, digits, _, - and .)"))?;
        let mut note = String::new();
        let name = if current.as_deref() == Some(wanted.as_str()) || !scene_store::scene_dir(&scenes_dir, &wanted).exists() {
            wanted
        } else {
            let free = scene_store::unique_name(&scenes_dir, &wanted);
            note = format!(" (a scene {wanted} exists: saved as {free})");
            free
        };
        let base = self.scene_base();
        let target = scene_store::scene_dir(&scenes_dir, &name);
        let mut scene = self.state.scene.clone();
        scene_paths::to_originals(&mut scene, &base, &target, originals);
        scene.name = name.clone();
        if let Some(old) = old.filter(|old| *old != name) {
            if in_use(&old) {
                note += &format!(" (a new scene: {old} is kept, cells use it)");
            } else {
                let from = scene_store::scene_dir(&scenes_dir, &old);
                std::fs::rename(&from, &target).map_err(|err| anyhow::anyhow!("renaming {} to {name}: {err}", from.display()))?;
                for (was, now) in [(format!("{old}.zip"), format!("{name}.zip")), (format!("{old}-urdf.zip"), format!("{name}-urdf.zip"))] {
                    if target.join(&was).is_file() && !target.join(&now).exists() {
                        std::fs::rename(target.join(&was), target.join(&now)).ok();
                    }
                }
                // Gone from the user's store under the old name (S3 mode), there under the new one.
                crate::storage::removed(&from);
                crate::storage::saved_tree(&target);
                note += &format!(" (renamed from {old})");
            }
        }
        if scene != self.state.scene || name != self.state.scene_file_name {
            self.state.scene = scene;
            self.state.scene_file_name = name.clone();
            // The viewer adopts the scene as saved (it sends its own otherwise).
            self.state.scene_seq += 1;
        }
        let path = scene_store::scene_file(&scenes_dir, &name);
        std::fs::create_dir_all(&target)?;
        std::fs::write(&path, serde_yaml::to_string(&self.state.scene)?)?;
        self.state.saved = Some(SceneEdit {
            file_name: name,
            scene: self.state.scene.clone(),
        });
        Ok((path, note))
    }

    fn refresh_palette(&mut self) {
        self.state.palette = palette(&self.root, &self.products_dir);
    }
}

impl FitterService {
    /// Call `on_saved` with the file of every scene saved from now on.
    pub fn set_on_saved(&self, on_saved: impl Fn(&Path) + Send + Sync + 'static) {
        *self.on_saved.lock().unwrap() = Some(Box::new(on_saved));
    }

    fn saved(&self, path: &Path) {
        if let Some(on_saved) = &*self.on_saved.lock().unwrap() {
            on_saved(path);
        }
    }

    pub fn new(bus: &Bus, config: Arc<ConfigStore>, assets: Arc<AssetIndex>) -> Arc<Self> {
        let this = Arc::new(Self {
            bus: bus.clone(),
            scene: ScenePublisher::new(bus, keys::SCENE_FITTER),
            assets,
            inner: Mutex::default(),
            pending_edit: Arc::default(),
            reach: Arc::new(ReachContext::new(config.clone())),
            config,
            on_saved: Mutex::default(),
        });
        // Saved scenes that reference a cell's copies reference the originals from now on.
        scene_paths::migrate_scenes(&this.originals(None));
        {
            let this = this.clone();
            crate::bus::serve_latest(bus, keys::FITTER_STATE, move || Some(this.state()));
        }
        {
            let this = this.clone();
            std::thread::Builder::new()
                .name("fitter_edits".to_owned())
                .spawn(move || this.apply_edits())
                .expect("failed to spawn fitter thread");
        }
        this
    }

    pub fn state(&self) -> FitterState {
        self.inner.lock().unwrap().state.clone()
    }

    /// Where a scene's originals are (see [`scene_paths`]); `root`: the asset root in use, if any.
    fn originals(&self, root: Option<&Path>) -> Originals {
        let catalogue = match root.filter(|r| !r.as_os_str().is_empty()) {
            Some(root) => root.to_owned(),
            None => PathBuf::from(self.config.resolve(&self.config.get().asset_root)),
        };
        Originals {
            catalogue,
            products: crate::services::product::products_dir(&self.config),
            cells: super::cells::cells_folder(&self.config),
        }
    }

    /// The cell the 3D view is linked to, if any.
    pub fn linked_cell(&self) -> Option<String> {
        self.inner.lock().unwrap().state.linked_cell.clone()
    }

    fn publish(&self, inner: &mut Inner) {
        self.refresh_sources(inner);
        self.bus.put_json(keys::FITTER_STATE, &inner.state);
    }

    /// The cells made from the open scene file, and whether the cell whose own scene is open is
    /// out of date with the scene it was made from (see [`FitterState::source_cells`]).
    fn refresh_sources(&self, inner: &mut Inner) {
        let state = &mut inner.state;
        state.source_cells = if state.cell_scene || !scene_store::is_valid_name(&state.scene_file_name) {
            Vec::new()
        } else {
            super::cells::made_from(&self.config, &state.scene_file_name)
                .into_iter()
                .filter_map(|name| {
                    let root = super::cells::cell_folder(&self.config, &name)?;
                    let resolved = super::cell_resolve::resolve_root(&root, &super::cells::fitted_scenes_dir(&self.config)).ok();
                    Some(SourceCell {
                        plans: super::cells::plan_count(&root),
                        out_of_date: super::cells::out_of_date_with(&self.config, &root).is_some(),
                        kind: resolved.as_ref().map(|r| r.kind).unwrap_or_default(),
                        stale_reason: resolved.as_ref().and_then(super::cell_resolve::prepared_staleness),
                        name,
                    })
                })
                .collect()
        };
        state.cell_out_of_date_with = match (&inner.link, state.cell_scene) {
            (Some(link), true) => super::cells::cell_folder(&self.config, &link.cell)
                .and_then(|root| super::cells::out_of_date_with(&self.config, &root)),
            _ => None,
        };
    }

    /// After the scene `file_name` (its name) was saved: nothing is copied, rebuilt or stopped. The
    /// active cell's view follows the scene; the outputs of its cells made from other inputs are
    /// stale now (the caller prepares [`Self::cells_to_prepare`]). Returns what to add to the
    /// "Saved …" status (see [`stale_note`]).
    fn after_save(&self, file_name: &str) -> String {
        let cells = super::cells::made_from(&self.config, file_name);
        if super::cells::active_cell(&self.config).is_some_and(|(active, _)| cells.contains(&active)) {
            super::cells::refresh_active_view(&self.config);
        }
        stale_note(&self.cells_to_prepare(file_name))
    }

    /// The reference cells made from scene `file_name` whose planner model is stale or not made yet:
    /// what saving the scene prepares (so every planner can plan them right away).
    fn cells_to_prepare(&self, file_name: &str) -> Vec<String> {
        let fitted_dir = super::cells::fitted_scenes_dir(&self.config);
        super::cells::made_from(&self.config, file_name)
            .into_iter()
            .filter(|cell| {
                super::cells::cell_folder(&self.config, cell)
                    .and_then(|root| super::cell_resolve::resolve_root(&root, &fitted_dir).ok())
                    .is_some_and(|r| {
                        r.kind == super::cell_resolve::CellKind::Reference
                            && (!r.root.join("agentic_planner/cell.yaml").is_file() || super::cell_resolve::prepared_staleness(&r).is_some())
                    })
            })
            .collect()
    }

    /// Whether a cell references scene `name` (renaming it on save keeps it then).
    fn in_use(&self, name: &str) -> bool {
        !super::cells::made_from(&self.config, name).is_empty()
    }

    fn set_status(&self, status: impl Into<String>) {
        let mut inner = self.inner.lock().unwrap();
        inner.state.status = status.into();
        self.publish(&mut inner);
    }

    /// Receive `factoryos/fitter/scene` edits.
    pub async fn serve_edits(self: Arc<Self>) -> anyhow::Result<()> {
        let subscriber = self
            .bus
            .session()
            .declare_subscriber(keys::FITTER_SCENE)
            .await
            .map_err(|err| anyhow::anyhow!("{err}"))?;
        while let Ok(sample) = subscriber.recv_async().await {
            match serde_json::from_slice::<SceneEdit>(&sample.payload().to_bytes()) {
                Ok(edit) => {
                    let (pending, changed) = &*self.pending_edit;
                    *pending.lock().unwrap() = Some(edit);
                    changed.notify_one();
                }
                Err(err) => tracing::warn!("{}: invalid scene edit: {err}", keys::FITTER_SCENE),
            }
        }
        Ok(())
    }

    fn apply_edits(self: &Arc<Self>) {
        let (pending, changed) = &*self.pending_edit;
        loop {
            let edit = {
                let mut pending = pending.lock().unwrap();
                while pending.is_none() {
                    pending = changed.wait(pending).unwrap();
                }
                pending.take().expect("checked")
            };
            self.apply_edit(edit);
        }
    }

    fn apply_edit(self: &Arc<Self>, edit: SceneEdit) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state.scene == edit.scene && inner.state.scene_file_name == edit.file_name {
            return;
        }
        inner.state.scene = edit.scene;
        inner.state.scene_file_name = edit.file_name;
        // The weld poses were solved for the scene as it was.
        inner.reach_pose.clear();
        inner.state.reach_pose = None;
        self.sync_3d(&mut inner, false);
        self.publish(&mut inner);
    }

    /// Use the config's asset root; starts a fresh 3D scene the first time (or when the root changes).
    pub fn open(self: &Arc<Self>) {
        let root = PathBuf::from(self.config.resolve(&self.config.get().asset_root));
        let mut inner = self.inner.lock().unwrap();
        inner.products_dir = crate::services::product::products_dir(&self.config);
        if inner.scene_id.is_none() || root != inner.root {
            inner.root = root;
            inner.state.root = inner.root.display().to_string();
            // The first open shows the default scene (body_weld_4x8), when it is there.
            let default_scene = self.config.get().default_scene;
            // `defaultScene`: a scene name, or a legacy `<name>.yaml`.
            match scene_store::name_of(&default_scene).filter(|name| scene_store::exists(&inner.scenes_dir(), name)) {
                Some(name) => self.load_scene(&mut inner, &name, true),
                None => self.blank_scene(&mut inner),
            }
        } else {
            inner.refresh_palette();
        }
        self.publish(&mut inner);
    }

    pub fn refresh_palette(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.products_dir = crate::services::product::products_dir(&self.config);
        inner.refresh_palette();
        self.publish(&mut inner);
    }

    pub fn new_empty_scene(self: &Arc<Self>) {
        let mut inner = self.inner.lock().unwrap();
        self.blank_scene(&mut inner);
        self.publish(&mut inner);
    }

    /// A new, empty scene with a fresh name ([`Inner::fresh_name`]).
    fn blank_scene(self: &Arc<Self>, inner: &mut Inner) {
        let name = inner.fresh_name();
        let scene = SceneFile { name: name.clone(), ..SceneFile::default() };
        self.new_scene(inner, scene, name);
    }

    pub fn load(self: &Arc<Self>, file_name: &str) {
        let mut inner = self.inner.lock().unwrap();
        self.load_scene(&mut inner, file_name, true);
        self.publish(&mut inner);
    }

    /// Load scene `file_name` (a name, or a legacy `<name>.yaml`) from `scenes/<name>/scene.yaml`;
    /// `link_source`: link it to the active cell if that was made from it.
    fn load_scene(self: &Arc<Self>, inner: &mut Inner, file_name: &str, link_source: bool) {
        let scenes_dir = inner.scenes_dir();
        let result = scene_store::name_of(file_name)
            .ok_or_else(|| anyhow::anyhow!("not a scene name"))
            .and_then(|name| Ok((scene_store::read(&scene_store::scene_file(&scenes_dir, &name))?, name)));
        match result {
            Ok((mut scene, file_name)) => {
                let file_name = file_name.as_str();
                if scene.name.trim().is_empty() {
                    scene.name = file_name.to_owned();
                }
                // A scene with paths to a cell's copies opens with the originals (unsaved).
                let mut fixed = scene.clone();
                let folder = scene_store::scene_dir(&scenes_dir, file_name);
                let rewrite = scene_paths::to_originals(&mut fixed, &folder, &folder, &self.originals(Some(&inner.root)));
                inner.state.status = if rewrite.changed > 0 {
                    format!("Loaded {file_name}; its paths to a cell's copies now reference the originals: Save to keep that.")
                } else {
                    format!("Loaded {file_name}.")
                };
                self.new_scene(inner, fixed, file_name.to_owned());
                inner.state.saved = Some(SceneEdit { file_name: file_name.to_owned(), scene });
                if link_source
                    && let Some((cell, root)) = super::cells::active_cell(&self.config)
                    && super::cells::fitted_source(&root).as_deref() == Some(file_name)
                {
                    self.link(inner, cell, &root);
                }
            }
            Err(err) => inner.state.status = format!("Failed to load {file_name}: {err}"),
        }
    }

    /// "Stop simulation and edit" (the simulation itself is stopped by the caller): a legacy cell's
    /// own scene ("Open Cell") is replaced by the Robo Fitter scene the cell was made from, when
    /// that is in `scenes/`, else by a new scene; a Robo Fitter scene linked to its cell (a
    /// reference cell, "Use as cell") stays open, editable, and is unlinked.
    pub fn edit(self: &Arc<Self>) {
        let mut inner = self.inner.lock().unwrap();
        let cell = inner.link.as_ref().map(|link| link.cell.clone());
        let active = super::cells::active_cell(&self.config);
        match edit_target(inner.state.cell_scene, cell.as_deref(), active.as_ref(), &inner.scenes_dir()) {
            EditTarget::Load(file) => {
                self.load_scene(&mut inner, &file, false);
                if let Some(cell) = cell {
                    inner.state.status = format!("Editing {file}, the scene cell {cell} was made from.");
                }
            }
            EditTarget::New => {
                let own = &inner.state.scene;
                let from_cell = (inner.state.cell_scene && (!own.robots.is_empty() || !own.fixtures.is_empty()))
                    .then(|| (own.clone(), inner.scene_base()));
                match (cell, from_cell) {
                    (Some(cell), Some((mut scene, base))) => {
                        // The cell's layout as an editable scene, with the original files (never
                        // the cell's copies), so no cell is needed to edit it.
                        let scenes_dir = inner.scenes_dir();
                        let file = if scene_store::is_valid_name(&cell) { scene_store::unique_name(&scenes_dir, &cell) } else { inner.fresh_name() };
                        scene_paths::to_originals(&mut scene, &base, &scene_store::scene_dir(&scenes_dir, &file), &self.originals(Some(&inner.root)));
                        scene.name = file.clone();
                        self.new_scene(&mut inner, scene, file.clone());
                        inner.state.status = format!(
                            "Cell {cell} has no Robo Fitter scene: editing its layout as the new scene {file} (the catalogue's and products' files); Save to keep it."
                        );
                    }
                    (cell, _) => {
                        self.blank_scene(&mut inner);
                        inner.state.status = match cell {
                            Some(cell) => format!("Cell {cell} has no Robo Fitter scene: a new scene."),
                            None => "A new scene.".to_owned(),
                        };
                    }
                }
            }
            EditTarget::Unlink => {
                drop(inner);
                return self.unlink();
            }
        }
        self.publish(&mut inner);
    }

    /// Delete the saved scene `file_name` (a name, or a legacy `<name>.yaml`): its folder
    /// `scenes/<name>/`. If it is the open one, it stays open, unsaved.
    pub fn delete(self: &Arc<Self>, file_name: &str) {
        let mut inner = self.inner.lock().unwrap();
        let name = scene_store::name_of(file_name);
        let result = match &name {
            Some(name) if scene_store::exists(&inner.scenes_dir(), name) => {
                let dir = scene_store::scene_dir(&inner.scenes_dir(), name);
                // Gone from the user's store too (S3 mode), or a new instance would bring it back.
                std::fs::remove_dir_all(&dir).map(|()| crate::storage::removed(&dir)).map_err(anyhow::Error::from)
            }
            Some(name) => Err(anyhow::anyhow!("no scene {name}")),
            None => Err(anyhow::anyhow!("not a scene name")),
        };
        let file_name = name.as_deref().unwrap_or(file_name);
        inner.state.status = match result {
            Ok(()) => {
                if inner.state.scene_file_name == file_name {
                    inner.state.saved = None;
                }
                format!("Deleted {file_name}.")
            }
            Err(err) => format!("Failed to delete {file_name}: {err}"),
        };
        inner.refresh_palette();
        self.publish(&mut inner);
    }

    /// Save the scene; its cells' outputs made from the old one are stale now (see
    /// [`Self::after_save`]). Returns the cells to prepare ([`Self::cells_to_prepare`]).
    pub fn save(self: &Arc<Self>, edit: SceneEdit) -> Vec<String> {
        self.apply_edit(edit);
        let saved = {
            let mut inner = self.inner.lock().unwrap();
            let originals = self.originals(Some(&inner.root));
            let saved = inner.save_scene(&originals, &|old| self.in_use(old)).map(|(path, note)| (path, inner.state.scene_file_name.clone(), note));
            if let Ok((path, _, _)) = &saved {
                self.saved(path);
            }
            inner.refresh_palette();
            if let Err(err) = &saved {
                inner.state.status = format!("Save failed: {err}");
            }
            self.publish(&mut inner);
            saved
        };
        let Ok((path, file_name, renamed)) = saved else {
            return Vec::new();
        };
        let note = self.after_save(&file_name);
        let mut inner = self.inner.lock().unwrap();
        inner.state.status = format!("Saved scene {file_name} ({}){renamed}{note}.", path.display());
        self.publish(&mut inner);
        drop(inner);
        self.cells_to_prepare(&file_name)
    }

    /// Save the scene (see [`Self::save`]), then compose its USDA zip in the background: the job
    /// returns the cells to prepare then ([`Self::cells_to_prepare`]; none when nothing was saved).
    pub fn compose(self: &Arc<Self>, request: ComposeRequest) -> Option<tokio::task::JoinHandle<Vec<String>>> {
        // A linked cell (simulated by step 1) is read-only: its scene is exported as saved, the
        // viewer's edit (maybe not that scene's) neither applied nor saved.
        let linked = self.inner.lock().unwrap().link.is_some();
        if !linked {
            self.apply_edit(request.edit);
        }
        let saved = if linked {
            Ok(self.inner.lock().unwrap().state.scene_file_name.clone())
        } else {
            let mut inner = self.inner.lock().unwrap();
            let originals = self.originals(Some(&inner.root));
            let saved = inner.save_scene(&originals, &|old| self.in_use(old)).map(|(path, _)| {
                self.saved(&path);
                inner.state.scene_file_name.clone()
            });
            if let Err(err) = &saved {
                inner.state.status = format!("Save failed: {err}");
                self.publish(&mut inner);
            }
            saved
        };
        let Ok(file_name) = saved else {
            return None;
        };
        let note = if linked { String::new() } else { self.after_save(&file_name) };
        let (scene, manifest, out, bundle) = {
            let mut inner = self.inner.lock().unwrap();
            let manifest = scene_store::scene_file(&inner.scenes_dir(), &file_name);
            if !note.is_empty() {
                inner.state.status = format!("Saved {file_name}{note}.");
            }
            let scene = match inner.compose_definition(&manifest) {
                Ok(scene) => scene,
                Err(err) => {
                    inner.state.status = format!("Scene invalid: {err}");
                    self.publish(&mut inner);
                    return None;
                }
            };
            // The bundles go into the scene's folder: `scenes/<name>/<name>.zip`, `<name>-urdf.zip`.
            let stem = file_name.clone();
            let folder = scene_store::scene_dir(&inner.scenes_dir(), &stem);
            let out = folder.join(format!("{stem}.zip"));
            let bundle = request.urdf_bundle.then(|| folder.join(format!("{stem}-urdf.zip")));
            inner.state.busy = Some(format!("Generating {stem}.zip…"));
            inner.state.compose_report = None;
            inner.refresh_palette();
            self.publish(&mut inner);
            (scene, manifest, out, bundle)
        };
        let options = ComposeOptions {
            full_robots: request.full_robots,
            full_tools: request.full_tools,
        };

        let this = self.clone();
        Some(tokio::task::spawn_blocking(move || {
            let result = (|| -> anyhow::Result<String> {
                let report = robo_fitting_room::compose_stage_with_options(&scene, &manifest, &out, &options)?;
                let mut text = format!(
                    "{} — {} robots, {} fixtures, {} meshes, {:.1} MB",
                    out.file_name().unwrap_or_default().to_string_lossy(),
                    report.robot_count,
                    report.fixture_count,
                    report.mesh_count,
                    report.usda_bytes as f64 / 1e6
                );
                if let Some(bundle) = bundle {
                    robo_fitting_room::compose_urdf_bundle(&scene, &manifest, &bundle)?;
                    text += &format!(" + {}", bundle.file_name().unwrap_or_default().to_string_lossy());
                }
                Ok(text)
            })();
            let mut inner = this.inner.lock().unwrap();
            inner.state.busy = None;
            inner.state.status = match &result {
                Ok(_) if !note.is_empty() => format!("USDA generated{note}."),
                Ok(_) => "USDA generated.".to_owned(),
                Err(err) => format!("Generate failed: {err:#}"),
            };
            inner.state.compose_report = Some(match result {
                Ok(text) => ComposeReport { ok: true, text },
                Err(err) => ComposeReport {
                    ok: false,
                    text: format!("{err:#}"),
                },
            });
            this.publish(&mut inner);
            drop(inner);
            // A linked cell's scene was not saved: its cells are as they were.
            if linked { Vec::new() } else { this.cells_to_prepare(&file_name) }
        }))
    }

    /// "Use as cell" (`FITTER_CREATE_CELL`): save the scene, make sure it has a cell (a folder for
    /// its outputs that references the scene, see [`super::cell_resolve::ensure_cell`]; nothing is
    /// copied), make that the active cell and link the 3D view to it. A new cell is named
    /// `request.name`, or after the scene when that is empty. Returns the new config.
    pub fn create_cell(self: &Arc<Self>, request: CreateCellRequest) -> anyhow::Result<Config> {
        self.apply_edit(request.edit);
        let file_name = {
            let mut inner = self.inner.lock().unwrap();
            let originals = self.originals(Some(&inner.root));
            let saved = inner.save_scene(&originals, &|old| self.in_use(old));
            if let Ok((path, _)) = &saved {
                self.saved(path);
            }
            inner.refresh_palette();
            if let Err(err) = saved {
                inner.state.status = format!("Save failed: {err:#}");
                self.publish(&mut inner);
                return Err(err);
            }
            inner.state.scene_file_name.clone()
        };
        let note = self.after_save(&file_name);
        let existed = super::cell_resolve::cell_for_scene(&self.config, &file_name)
            .ok()
            .and_then(|cell| super::cells::cell_folder(&self.config, &cell))
            .is_some_and(|root| super::cell_resolve::read_manifest(&root).is_some_and(|m| super::cell_resolve::names_scene(&m, &file_name)));
        let result = super::cells::create_from_fitted(&self.config, &file_name, &request.name);
        let mut inner = self.inner.lock().unwrap();
        match (&result, super::cells::active_cell(&self.config)) {
            (Ok(_), Some((cell, root))) => {
                let what = if existed { "Using cell" } else { "Made cell" };
                inner.state.status = format!("{what} {cell} for {file_name}{note}; preparing it for planning…");
                self.link(&mut inner, cell, &root);
            }
            (Ok(_), None) => inner.state.status = "Made the cell, but it is not the active cell".to_owned(),
            (Err(err), _) => inner.state.status = format!("Use as cell failed: {err:#}"),
        }
        self.publish(&mut inner);
        result
    }

    /// "Open Cell": make `cell` the active cell and show its scene, linked to it: a reference
    /// cell's Robo Fitter scene (editable; saving it marks the cell's outputs stale), or a legacy
    /// cell's own `scene.yaml` (read-only). Returns the new config.
    pub fn open_cell(self: &Arc<Self>, cell: &str) -> anyhow::Result<Config> {
        self.open();
        let config = super::cells::set_active(&self.config, cell)?;
        let (name, root) = super::cells::active_cell(&self.config).ok_or_else(|| anyhow::anyhow!("no active cell"))?;
        let reference = super::cells::active_resolved(&self.config)
            .filter(|r| r.kind == super::cell_resolve::CellKind::Reference)
            .and_then(|r| r.fitted_scene);
        if let Some(file) = reference {
            let mut inner = self.inner.lock().unwrap();
            self.load_scene(&mut inner, &file, false);
            if inner.state.scene_file_name == file {
                self.link(&mut inner, name.clone(), &root);
                inner.state.status = format!("Opened cell {name} (its scene {file}).");
            }
            self.publish(&mut inner);
            return Ok(config);
        }
        let scene = std::fs::read_to_string(root.join("scene.yaml"))
            .map_err(anyhow::Error::from)
            .and_then(|text| Ok(scene_paths::parse_scene_file(&text)?));
        let mut inner = self.inner.lock().unwrap();
        match scene {
            Ok(mut scene) if !scene.robots.is_empty() || !scene.fixtures.is_empty() => {
                if scene.name.is_empty() {
                    scene.name = name.clone();
                }
                // Base first: `new_scene` renders right away.
                self.new_scene(&mut inner, SceneFile::default(), "scene.yaml".to_owned());
                inner.scene_base = Some(root.clone());
                inner.state.cell_scene = true;
                inner.state.scene = scene;
                self.link(&mut inner, name.clone(), &root);
                self.sync_3d(&mut inner, false);
                inner.state.status = format!("Opened cell {name}.");
            }
            _ => {
                // Don't stay linked to the previously opened cell.
                self.blank_scene(&mut inner);
                inner.state.status =
                    format!("Cell {name} is active, but has no Robo Fitter scene.yaml to show (not made by Physical AI Production Designer).");
            }
        }
        self.publish(&mut inner);
        Ok(config)
    }

    /// Stop following the cell; the robots go back to their home poses and the scene can be edited.
    /// A cell's own scene ("Open Cell") is closed instead.
    pub fn unlink(self: &Arc<Self>) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state.cell_scene {
            self.blank_scene(&mut inner);
            self.publish(&mut inner);
            return;
        }
        if let Some(link) = inner.link.take() {
            inner.state.status = format!("Unlinked from cell {}.", link.cell);
        }
        inner.state.linked_cell = None;
        clear_reach(&mut inner);
        self.sync_3d(&mut inner, true);
        self.publish(&mut inner);
    }

    fn link(&self, inner: &mut Inner, cell: String, cell_root: &Path) {
        let mut robots: HashMap<String, String> = super::cells::robot_scene_ids(cell_root)
            .into_iter()
            .map(|(id, scene_id)| (id, format!("robots/{scene_id}")))
            .collect();
        if robots.is_empty() {
            // The scaffold numbers the scene's robots `robot1`, `robot2`, … in order.
            robots = (inner.state.scene.robots.iter().enumerate())
                .map(|(i, robot)| (format!("robot{}", i + 1), format!("robots/{}", robot.robot_id)))
                .collect();
        }
        inner.state.linked_cell = Some(cell.clone());
        inner.link = Some(CellLink {
            cell,
            root: cell_root.to_owned(),
            robots,
            shown: HashMap::new(),
        });
        // The title says it is the cell's now.
        self.send_layout_if_retitled(inner);
        if inner.scene_id.is_some() {
            let origin = inner.state.scene.scene_origin.unwrap_or_default();
            let welds: Vec<[f64; 3]> = reach::task::task_welds(cell_root).iter().map(|(_, p, _)| *p).collect();
            let panel = panel_node(&welds, [origin.x, origin.y, origin.z], inner.state.scene.fixtures.is_empty());
            if panel.is_some() {
                tracing::info!("Robo Fitter: panel under the {} welds of {}", welds.len(), cell_root.display());
            }
            let (upsert, remove) = replace_under(&self.scene.doc(), PRODUCT, panel.into_iter().collect());
            self.scene.update(upsert, remove, None);
        }
        self.log_task_and_reach(inner);
    }

    /// Receive the simulation's state (`factoryos/sim/state`) and move the linked robots; its
    /// status (`factoryos/sim/status`) says which cell it runs.
    pub async fn follow_cell(self: Arc<Self>) -> anyhow::Result<()> {
        let session = self.bus.session();
        let err = |err: zenoh::Error| anyhow::anyhow!("{err}");
        let states = session.declare_subscriber(keys::SIM_STATE).await.map_err(err)?;
        let statuses = session.declare_subscriber(keys::SIM_STATUS).await.map_err(err)?;
        loop {
            tokio::select! {
                Ok(sample) = states.recv_async() => {
                    if let Ok(state) = serde_json::from_slice::<serde_json::Value>(&sample.payload().to_bytes()) {
                        self.show_cell_joints(&state);
                    }
                }
                Ok(sample) = statuses.recv_async() => {
                    if let Ok(status) = serde_json::from_slice::<factoryos_protocol::BackendStatus>(&sample.payload().to_bytes()) {
                        let running = matches!(
                            status.state,
                            factoryos_protocol::BackendState::Starting | factoryos_protocol::BackendState::Ready
                        );
                        self.inner.lock().unwrap().sim_cell = status.cell.filter(|_| running);
                    }
                }
                else => return Ok(()),
            }
        }
    }

    /// Show the linked robots' joints from a cell-runner `state` (`krl[i].joints`, radians, in
    /// URDF order of the actuated joints; a loaded behaviour tree's playback overrides them there).
    fn show_cell_joints(&self, state: &serde_json::Value) {
        let active = super::cells::active_cell(&self.config).map(|(cell, _)| cell);
        let mut inner = self.inner.lock().unwrap();
        let Inner { scene_id, placed, link, sim_cell, reach_pose, .. } = &mut *inner;
        let (Some(_), Some(link)) = (scene_id.as_ref(), link.as_mut()) else {
            return;
        };
        if !follows(&link.cell, active.as_deref(), sim_cell.as_deref()) {
            return; // Another cell is simulated.
        }
        let mut models = Vec::new();
        for entry in state["krl"].as_array().into_iter().flatten() {
            let (Some(id), Some(joints)) = (entry["robot_id"].as_str(), entry["joints"].as_array()) else {
                continue;
            };
            let Some(key) = link.robots.get(id) else {
                continue;
            };
            let Some(loaded) = placed.get(key).and_then(|p| p.loaded.as_ref()).filter(|_| !reach_pose.contains_key(key)) else {
                continue;
            };
            let values: Vec<f64> = joints.iter().filter_map(serde_json::Value::as_f64).collect();
            if link.shown.get(key) == Some(&values) {
                continue;
            }
            models.push(ModelJoints { node: key.clone(), joints: loaded.urdf.joint_map(&values) });
            link.shown.insert(key.clone(), values);
        }
        if !models.is_empty() {
            self.scene.joints(models);
        }
    }

    /// Replace the scene and start a fresh 3D scene for it.
    fn new_scene(self: &Arc<Self>, inner: &mut Inner, scene: SceneFile, file_name: String) {
        inner.placed.clear();
        inner.shown = None;
        inner.state.scene = scene;
        inner.state.scene_file_name = file_name;
        inner.state.saved = None;
        inner.link = None;
        inner.state.linked_cell = None;
        inner.scene_base = None;
        inner.state.cell_scene = false;
        inner.state.scene_seq += 1;
        inner.state.robot_dofs.clear();
        inner.layout_title = None;
        inner.reach_drawn = None;
        clear_reach(inner);
        inner.refresh_palette();
        inner.scene_id = Some(self.scene.set(Vec::new(), ViewHints { title: view_title(&inner.state) }));
        self.sync_3d(inner, false);
    }


    /// Bring the 3D view in line with the scene; the scene publisher sends only what changed.
    fn sync_3d(self: &Arc<Self>, inner: &mut Inner, force: bool) {
        if !force && inner.shown.as_ref() == Some(&inner.state.scene) {
            return;
        }
        if inner.scene_id.is_none() {
            return;
        }

        let definition = inner.definition();
        inner.state.diagnostics = match &definition {
            Ok(definition) => robo_fitting_room::validate_scene(definition)
                .iter()
                .map(|err| err.to_string())
                .collect(),
            Err(err) => vec![err.to_string()],
        };
        if !inner.state.cell_scene {
            let originals = self.originals(Some(&inner.root));
            let warnings = scene_paths::cell_copy_warnings(&inner.state.scene, &inner.scene_base(), &originals);
            inner.state.diagnostics.extend(warnings);
        }
        let definition = definition.ok();
        let scene = inner.state.scene.clone();
        let origin = scene.scene_origin.unwrap_or_default();
        let items = scene_items(&scene, &inner.scene_base());

        // Remove what's gone (or whose URDF changed); load what's new, placed once it has loaded.
        let mut remove = stale_keys(&inner.placed, &items);
        for key in &remove {
            inner.placed.remove(key);
        }
        let mut layout_changed = false;
        for item in &items {
            inner.placed.entry(item.key.clone()).or_insert_with(|| {
                layout_changed = true;
                self.load_model(&item.key, &item.urdf);
                Placed { urdf: item.urdf.clone(), loaded: None }
            });
        }
        let cell_joints = inner.link.as_ref().map(|link| &link.shown);
        let (mut upsert, pedestals_gone) = item_nodes(&items, origin, &inner.placed, &inner.reach_pose, cell_joints);
        remove.extend(pedestals_gone);

        // Floor plan lines.
        match definition.as_ref().and_then(|d| d.floor_plan.as_ref()) {
            Some(plan) => upsert.extend(floor_plan_nodes(&plan.segments, &plan.footprint_segments)),
            None => remove.push("floor_plan".to_owned()),
        }

        let title = view_title(&inner.state);
        let view = (layout_changed || inner.shown.is_none() || retitled(inner.layout_title.as_deref(), &title)).then(|| {
            inner.layout_title = Some(title.clone());
            ViewHints { title }
        });
        // An item that only moved (and its pedestal) goes as a pose.
        self.scene.update_moving(upsert, remove, view);
        inner.state.robot_dofs = robot_dofs(&inner.placed);
        inner.shown = Some(scene);
        self.log_task_and_reach(inner);
    }

    /// The task's welds and the robots' reach for the scene as it is: fills the task, estimate and
    /// TCP fields of the state and draws `world/task` and `world/reach` (or clears them) as the
    /// view says. Runs on every edit, so the welds follow a product while it is dragged.
    fn log_task_and_reach(&self, inner: &mut Inner) {
        let cell_root = inner.link.as_ref().map(|link| link.root.clone());
        let analysis = reach::analyse(&self.reach, &inner.state.scene, &inner.scene_base(), cell_root.as_deref(), inner.state.view.task_source);
        let state = &mut inner.state;
        state.scene_rev = analysis.scene_rev;
        state.task = analysis.welds.clone();
        state.task_notes = analysis.notes.clone();
        state.estimate = analysis.estimate.clone();
        state.tcp = analysis.tcp();
        let analysis = Arc::new(analysis);
        inner.analysis = Some(analysis.clone());
        self.draw_task(inner, true);
    }

    /// Draw the task (and with `reach`, the domes) from the last analysis and report.
    fn draw_task(&self, inner: &mut Inner, reach: bool) {
        let (Some(_), Some(analysis)) = (inner.scene_id.as_ref(), inner.analysis.as_ref()) else {
            return;
        };
        let o = inner.state.scene.scene_origin.unwrap_or_default();
        let origin = [o.x, o.y, o.z];
        let selected = inner.state.reach_pose.as_ref().and_then(|id| analysis.welds.iter().find(|w| &w.id == id));
        let doc = self.scene.doc();
        let task = draw::task_nodes(origin, analysis, inner.state.reach.as_ref(), selected, inner.state.view.show_task, inner.state.view.show_labels);
        let (mut upsert, mut remove) = replace_under(&doc, draw::TASK, task);
        let key = draw::reach_key(origin, analysis, inner.state.view.show_reach);
        if reach && inner.reach_drawn != Some(key) {
            let (domes, gone) = replace_under(&doc, draw::REACH, draw::reach_nodes(origin, analysis, inner.state.view.show_reach));
            upsert.extend(domes);
            remove.extend(gone);
            inner.reach_drawn = Some(key);
        }
        self.scene.update(upsert, remove, None);
    }

    /// "Show task" / "Show reach" / the task source.
    pub fn set_view(&self, view: FitterView) {
        let mut inner = self.inner.lock().unwrap();
        inner.state.view = view;
        self.log_task_and_reach(&mut inner);
        self.publish(&mut inner);
    }

    /// "Check reachability": apply the viewer's edit, then check every robot on every weld in the
    /// background; the report fills `FitterState::reach` as results arrive.
    pub fn reach_check(self: &Arc<Self>, request: ReachCheckRequest) {
        if let Some(edit) = request.edit {
            self.apply_edit(edit);
        }
        let mut inner = self.inner.lock().unwrap();
        if inner.state.reach.as_ref().is_some_and(|r| r.running) {
            inner.state.status = "A reachability check is running: cancel it first.".to_owned();
            self.publish(&mut inner);
            return;
        }
        if let Some(job) = inner.reach_job.take()
            && job.is_finished()
        {
            job.join().ok();
        }
        inner.reach_cancel = Arc::new(AtomicBool::new(false));
        inner.reach_generation += 1;
        inner.reach_pose.clear();
        inner.state.reach_pose = None;
        let generation = inner.reach_generation;
        let scene = inner.state.scene.clone();
        let base = inner.scene_base();
        let cell_root = inner.link.as_ref().map(|link| link.root.clone());
        let options = CheckOptions {
            source: inner.state.view.task_source,
            mode: request.mode,
            robots: request.robots,
            welds: request.welds,
            estimate_only: false,
        };
        let welds = inner.state.task.len();
        inner.state.reach = Some(ReachReport {
            scene_rev: inner.state.scene_rev,
            mode: options.mode,
            method: "ik".to_owned(),
            running: true,
            total: welds * scene.robots.len(),
            ..Default::default()
        });
        inner.state.status = format!(
            "Checking reachability of {welds} welds ({})…",
            if options.mode == ReachMode::Full { "with the approach" } else { "quick" }
        );
        self.publish(&mut inner);
        let (this, cancel) = (self.clone(), inner.reach_cancel.clone());
        inner.reach_job = Some(
            std::thread::Builder::new()
                .name("fitter_reach".to_owned())
                .spawn(move || {
                    let report = reach::check_scene(
                        &this.reach,
                        &scene,
                        &base,
                        cell_root.as_deref(),
                        &options,
                        |report| this.reach_progress(generation, report),
                        &cancel,
                    );
                    this.reach_progress(generation, &report);
                })
                .expect("failed to spawn the reach check"),
        );
    }

    fn reach_progress(&self, generation: u64, report: &ReachReport) {
        let mut inner = self.inner.lock().unwrap();
        if inner.reach_generation != generation {
            return;
        }
        let finished = !report.running && inner.state.reach.as_ref().is_some_and(|r| r.running);
        inner.state.reach = Some(report.clone());
        if finished {
            let reached = report.welds.iter().filter(|w| !w.reached_by.is_empty()).count();
            inner.state.status = match &report.error {
                Some(err) => format!("Reachability check stopped: {err}."),
                None => format!(
                    "Reachability ({}): {reached} of {} welds reached, {:.0} s.",
                    report.method,
                    report.welds.len(),
                    report.elapsed_s
                ),
            };
            self.draw_task(&mut inner, false);
        }
        self.publish(&mut inner);
    }

    pub fn reach_cancel(&self) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state.reach.as_ref().is_some_and(|r| r.running) {
            inner.reach_cancel.store(true, Ordering::Relaxed);
            inner.state.status = "Cancelling the reachability check…".to_owned();
        }
        self.publish(&mut inner);
    }

    /// Show the robots at weld `weld_id` with the joints the last check found (robots that don't
    /// reach it at home); `None`: back to home / the cell's joints.
    pub fn reach_pose(self: &Arc<Self>, request: ReachPoseRequest) {
        let mut inner = self.inner.lock().unwrap();
        let had_pose = !inner.reach_pose.is_empty();
        inner.reach_pose.clear();
        inner.state.reach_pose = None;
        if request.weld_id.is_none() && had_pose {
            inner.state.status = if inner.link.is_some() { "Robots back on the cell's joints." } else { "Robots back at home." }.to_owned();
        }
        if let Some(id) = request.weld_id {
            match weld_poses(&inner, &id) {
                Ok((poses, note)) => {
                    inner.reach_pose = poses;
                    inner.state.reach_pose = Some(id);
                    inner.state.status = note;
                }
                Err(err) => inner.state.status = err,
            }
        }
        if inner.scene_id.is_some() {
            self.scene.joints(robot_joints(&inner));
        }
        self.draw_task(&mut inner, false);
        self.publish(&mut inner);
    }

    /// The agent's check (`/api/v1/reach/check`, `show_in_fitter`): show `report` in the Robo
    /// Fitter, loading its scene first (a `scenes/` file, or the active cell's own scene).
    pub fn show_report(self: &Arc<Self>, file_name: Option<&str>, cell: Option<&str>, report: ReachReport) -> Option<String> {
        // The Robo Fitter's 3D scene and asset root, if no viewer opened it yet.
        self.open();
        let showing = {
            let inner = self.inner.lock().unwrap();
            match (file_name, cell) {
                (Some(file), _) => {
                    !inner.state.cell_scene
                        && scene_store::name_of(file).is_some_and(|name| inner.state.scene_file_name == name)
                        && inner.scene_id.is_some()
                }
                (None, Some(cell)) => inner.state.cell_scene && inner.state.linked_cell.as_deref() == Some(cell),
                (None, None) => false,
            }
        };
        if !showing {
            match (file_name, cell) {
                (Some(file), _) => self.load(file),
                (None, Some(cell)) if super::cells::active_cell(&self.config).is_some_and(|(active, _)| active == cell) => {
                    if let Err(err) = self.open_cell(cell) {
                        return Some(format!("not shown in the Robo Fitter: {err:#}"));
                    }
                }
                (None, Some(cell)) => return Some(format!("not shown in the Robo Fitter: cell {cell} is not the active cell")),
                (None, None) => return Some("not shown in the Robo Fitter: the scene has no file".to_owned()),
            }
        }
        let mut inner = self.inner.lock().unwrap();
        if inner.state.reach.as_ref().is_some_and(|r| r.running) {
            inner.reach_cancel.store(true, Ordering::Relaxed);
        }
        inner.reach_generation += 1;
        inner.state.view.task_source = report.source;
        inner.state.view.show_task = true;
        inner.state.reach = Some(report);
        self.log_task_and_reach(&mut inner);
        let stale = inner.state.reach.as_ref().is_some_and(|r| r.scene_rev != inner.state.scene_rev);
        inner.state.status = "Reachability checked by the agent.".to_owned();
        self.publish(&mut inner);
        stale.then(|| "shown in the Robo Fitter, but its scene there differs (unsaved edits?)".to_owned())
    }

    pub fn reach_context(&self) -> Arc<ReachContext> {
        self.reach.clone()
    }

    /// `scenes/` of the asset root.
    pub fn scenes_dir(&self) -> PathBuf {
        let inner = self.inner.lock().unwrap();
        if inner.root.as_os_str().is_empty() {
            PathBuf::from(self.config.resolve(&self.config.get().asset_root)).join(factoryos_protocol::fitter::SCENES_DIR)
        } else {
            inner.scenes_dir()
        }
    }

    /// Send the title again when it changes between a cell's and the Robo Fitter's (link /
    /// unlink). Renaming the scene shows with the next layout change, not on every key.
    fn send_layout_if_retitled(&self, inner: &mut Inner) {
        let title = view_title(&inner.state);
        if inner.scene_id.is_some() && retitled(inner.layout_title.as_deref(), &title) {
            inner.layout_title = Some(title.clone());
            self.scene.update(Vec::new(), Vec::new(), Some(ViewHints { title }));
        }
    }

    /// Load a URDF for `key` in the background (parse it, register its files as assets), then
    /// place it.
    fn load_model(self: &Arc<Self>, key: &str, urdf: &Path) {
        let (this, key, urdf) = (self.clone(), key.to_owned(), urdf.to_owned());
        std::thread::spawn(move || {
            let what = model_label(&key);
            this.set_status(format!("Loading {what}…"));
            let result = (|| -> anyhow::Result<Loaded> {
                let model = UrdfModel::load(&urdf)?;
                Ok(Loaded { model: this.assets.model_ref(&urdf)?, urdf: model })
            })();
            let mut inner = this.inner.lock().unwrap();
            match result {
                Ok(loaded) => {
                    inner.state.status = format!("Loaded {what}.");
                    if let Some(placed) = inner.placed.get_mut(&key)
                        && placed.urdf == urdf
                    {
                        placed.loaded = Some(Arc::new(loaded));
                        this.sync_3d(&mut inner, true);
                    }
                }
                Err(err) => {
                    tracing::warn!("Robo Fitter: failed to load {}: {err:#}", urdf.display());
                    inner.state.status = format!("Failed to load {what}: {err:#}");
                }
            }
            this.publish(&mut inner);
        });
    }
}

/// Forget the reachability check and the weld poses (a new scene, unlinking); a running check is
/// cancelled.
fn clear_reach(inner: &mut Inner) {
    inner.reach_cancel.store(true, Ordering::Relaxed);
    inner.reach_generation += 1;
    inner.reach_pose.clear();
    inner.state.reach_pose = None;
    inner.state.reach = None;
}

/// The joint values (by node id) that show the robots at weld `id` of the last check, and a
/// status line.
fn weld_poses(inner: &Inner, id: &str) -> Result<(HashMap<String, JointMap>, String), String> {
    let chains: HashMap<&str, Vec<String>> = (inner.analysis.iter().flat_map(|a| &a.robots))
        .filter_map(|r| Some((r.id.as_str(), r.kind.as_ref()?.chain.joint_names())))
        .collect();
    weld_poses_with(inner, id, &chains)
}

/// [`weld_poses`], with each robot's IK joint names (`chains`, the order of an attempt's
/// `joints_deg`).
fn weld_poses_with(inner: &Inner, id: &str, chains: &HashMap<&str, Vec<String>>) -> Result<(HashMap<String, JointMap>, String), String> {
    let report = inner.state.reach.as_ref().ok_or("Check reachability first: the poses come from the IK.")?;
    let weld = report.welds.iter().find(|w| w.weld.id == id).ok_or_else(|| format!("{id} is not in the last check."))?;
    let mut poses = HashMap::new();
    let (mut at, mut home) = (Vec::new(), Vec::new());
    for robot in &inner.state.scene.robots {
        let key = format!("robots/{}", robot.robot_id);
        let Some(loaded) = inner.placed.get(&key).and_then(|p| p.loaded.as_ref()) else { continue };
        let attempt = weld.attempts.iter().find(|a| a.robot_id == robot.robot_id && a.reachable && !a.joints_deg.is_empty());
        let values = match (attempt, chains.get(robot.robot_id.as_str())) {
            (Some(attempt), Some(names)) => {
                at.push(robot.robot_id.clone());
                loaded.urdf.joint_map_from_named_deg(names, &attempt.joints_deg)
            }
            _ => {
                home.push(robot.robot_id.clone());
                loaded.urdf.home_joint_map(robot.home_pose_deg.as_deref())
            }
        };

        poses.insert(key, values);
    }
    let mut note = format!("{}: ", weld.weld.label);
    note += &if at.is_empty() { "no robot reaches it".to_owned() } else { format!("{} at the weld", at.join(", ")) };
    if !home.is_empty() {
        note += &format!("; {} at home", home.join(", "));
    }
    if report.scene_rev != inner.state.scene_rev {
        note += " (the check is stale: the scene changed since)";
    }
    Ok((poses, note + "."))
}

/// What "Stop simulation and edit" opens.
#[derive(Debug, PartialEq, Eq)]
enum EditTarget {
    /// The `scenes/` scene the cell was made from.
    Load(String),
    /// A new, empty scene.
    New,
    /// The scene stays; only the link to its cell goes (a "Create Cell" scene).
    Unlink,
}

/// [`EditTarget`] for a scene that is a cell's own (`cell_scene`) or not, linked to `linked`,
/// with the `active` cell (name, folder) and the `scenes/` folder `scenes_dir`.
fn edit_target(cell_scene: bool, linked: Option<&str>, active: Option<&(String, PathBuf)>, scenes_dir: &Path) -> EditTarget {
    if !cell_scene {
        return EditTarget::Unlink;
    }
    active
        .filter(|(name, _)| Some(name.as_str()) == linked)
        .and_then(|(_, root)| super::cells::fitted_source(root))
        .filter(|name| scene_store::exists(scenes_dir, name))
        .map_or(EditTarget::New, EditTarget::Load)
}

/// What "Saved <file>" adds for the cells whose prepared planner model saving made stale:
/// "; marks the outputs of cell a stale (Prepare Cell again)".
fn stale_note(stale: &[String]) -> String {
    match stale {
        [] => String::new(),
        [cell] => format!("; preparing cell {cell} for planning"),
        cells => format!("; preparing cells {} for planning", cells.join(", ")),
    }
}

/// A 3D node key (`robots/<id>`, `fixtures/<id>`) in words: `robot <id>`, `fixture <id>`.
fn model_label(key: &str) -> String {
    match key.split_once('/') {
        Some(("robots", id)) => format!("robot {id}"),
        Some(("fixtures", id)) => format!("fixture {id}"),
        _ => key.to_owned(),
    }
}

fn palette(root: &Path, products_dir: &Path) -> Palette {
    let list = |dir: &Path, want: &dyn Fn(&Path) -> Option<String>| -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|entry| want(&entry.path()))
            .collect();
        names.sort();
        names
    };
    let urdf_dirs = |collection: &str| {
        list(&root.join(collection), &|path| {
            let name = path.file_name()?.to_string_lossy().into_owned();
            path.join(format!("{name}.urdf")).is_file().then_some(name)
        })
    };
    let files_with = |dir: PathBuf, ext: &'static str| {
        list(&dir, &move |path| {
            (path.extension()? == ext).then(|| path.file_name().unwrap_or_default().to_string_lossy().into_owned())
        })
    };
    Palette {
        scenes: scene_store::list(&root.join(factoryos_protocol::fitter::SCENES_DIR)),
        robots: urdf_dirs("urdf-dressed-robots"),
        fixtures: urdf_dirs("urdf-fixture-collection"),
        floor_plans: files_with(root.join("floor-plans"), "obj"),
        products: crate::services::product::placeable_products(products_dir),
        products_dir: products_dir.display().to_string(),
    }
}

fn robot_dofs(placed: &HashMap<String, Placed>) -> BTreeMap<String, usize> {
    placed
        .iter()
        .filter_map(|(key, placed)| {
            let id = key.strip_prefix("robots/")?;
            Some((id.to_owned(), placed.loaded.as_ref()?.urdf.actuated_joints().len()))
        })
        .collect()
}

/// A robot or fixture of the scene, as the 3D view places it.
#[derive(Clone, Debug, PartialEq)]
struct Item {
    /// The node id: `robots/<robot_id>` or `fixtures/<id>`.
    key: String,
    urdf: PathBuf,
    position: Xyz,
    orientation: Abc,
    pedestal: Option<PedestalYaml>,
    home_pose_deg: Option<Vec<f64>>,
}

/// The scene's robots, then its fixtures; relative URDF paths start at `base`.
fn scene_items(scene: &SceneFile, base: &Path) -> Vec<Item> {
    let robots = scene.robots.iter().map(|robot| Item {
        key: format!("robots/{}", robot.robot_id),
        urdf: scene_store::asset_path(base, &robot.dressed_urdf),
        position: robot.position,
        orientation: robot.orientation,
        pedestal: robot.pedestal.clone(),
        home_pose_deg: robot.home_pose_deg.clone(),
    });
    let fixtures = scene.fixtures.iter().map(|fixture| Item {
        key: format!("fixtures/{}", fixture.id),
        urdf: scene_store::asset_path(base, &fixture.urdf),
        position: fixture.position,
        orientation: fixture.orientation,
        pedestal: None,
        home_pose_deg: None,
    });
    robots.chain(fixtures).collect()
}

/// The placed items that are gone from `items`, or whose URDF changed.
fn stale_keys(placed: &HashMap<String, Placed>, items: &[Item]) -> Vec<String> {
    let keep: HashMap<&str, &Path> = items.iter().map(|item| (item.key.as_str(), item.urdf.as_path())).collect();
    let mut stale: Vec<String> = placed
        .iter()
        .filter(|(key, placed)| keep.get(key.as_str()) != Some(&placed.urdf.as_path()))
        .map(|(key, _)| key.clone())
        .collect();
    stale.sort();
    stale
}

/// Where an item's root link goes: its position relative to the scene `origin`, lifted onto its
/// pedestal, turned by its `A, B, C`.
fn placement(item: &Item, origin: Xyz) -> Pose3 {
    let lift = item.pedestal.as_ref().map_or(0.0, |p| p.height_mm / 1000.0);
    let (p, o) = (item.position, item.orientation);
    pose3_from_xyz_abc_deg([p.x - origin.x, p.y - origin.y, p.z - origin.z + lift], [o.a, o.b, o.c])
}

/// An item's pedestal (`<key>/pedestal`), if it has one: a cylinder in the world frame from the
/// floor (z = 0) up to the robot's base, under `placement`. Its pose is on the floor under the base
/// (unturned), so moving the robot moves the pedestal by its pose alone.
fn pedestal_node(item: &Item, placement: &Pose3) -> Option<Node> {
    let p = item.pedestal.as_ref().filter(|p| p.height_mm > 0.0)?;
    let (height, radius) = (p.height_mm / 1000.0, p.diameter_mm / 2000.0);
    let node = Node::new(
        format!("{}/pedestal", item.key),
        NodeKind::Cylinder { height, radius, center: [0.0, 0.0, height / 2.0], color: Some([0x5a, 0x5a, 0x64, 0xff]), solid: true },
    );
    Some(node.at(Pose3::from_xyz([placement.xyz[0], placement.xyz[1], 0.0])))
}

/// The joints a placed robot (`key`) shows: at a weld ("Reachability": a weld picked), else the
/// cell's while linked (once they arrive), else its home pose.
fn shown_joints(
    key: &str,
    home_pose_deg: Option<&[f64]>,
    model: &UrdfModel,
    reach_pose: &HashMap<String, JointMap>,
    cell_joints: Option<&HashMap<String, Vec<f64>>>,
) -> JointMap {
    match (reach_pose.get(key), cell_joints.and_then(|shown| shown.get(key))) {
        (Some(joints), _) => joints.clone(),
        (None, Some(values)) => model.joint_map(values),
        (None, None) => model.home_joint_map(home_pose_deg),
    }
}

/// The nodes of the loaded `items` (models and pedestals), and the pedestals to remove.
fn item_nodes(
    items: &[Item],
    origin: Xyz,
    placed: &HashMap<String, Placed>,
    reach_pose: &HashMap<String, JointMap>,
    cell_joints: Option<&HashMap<String, Vec<f64>>>,
) -> (Vec<Node>, Vec<String>) {
    let (mut upsert, mut remove) = (Vec::new(), Vec::new());
    for item in items {
        let Some(loaded) = placed.get(&item.key).and_then(|p| p.loaded.as_ref()) else {
            continue; // Placed once it has loaded.
        };
        let pose = placement(item, origin);
        let joints = shown_joints(&item.key, item.home_pose_deg.as_deref(), &loaded.urdf, reach_pose, cell_joints);
        upsert.push(Node::new(item.key.clone(), NodeKind::Model { model: loaded.model.clone(), joints, show_collision: false }).at(pose));
        // A robot's name over it (the viewer's "Labels" toggle shows or hides it).
        if let Some(robot_id) = item.key.strip_prefix("robots/") {
            upsert.push(crate::services::scene_pub::robot_label_node(&item.key, robot_id, loaded.urdf.shoulder_height()));
        }
        match pedestal_node(item, &pose) {
            Some(pedestal) => upsert.push(pedestal),
            None => remove.push(format!("{}/pedestal", item.key)),
        }
    }
    (upsert, remove)
}

/// Every placed robot's joints (see [`shown_joints`]).
fn robot_joints(inner: &Inner) -> Vec<ModelJoints> {
    let cell_joints = inner.link.as_ref().map(|link| &link.shown);
    (inner.state.scene.robots.iter())
        .filter_map(|robot| {
            let key = format!("robots/{}", robot.robot_id);
            let loaded = inner.placed.get(&key)?.loaded.as_ref()?;
            let joints = shown_joints(&key, robot.home_pose_deg.as_deref(), &loaded.urdf, &inner.reach_pose, cell_joints);
            Some(ModelJoints { node: key, joints })
        })
        .collect()
}

/// The floor plan's lines (`floor_plan/lines`) and footprints (`floor_plan/footprints`).
fn floor_plan_nodes(segments: &[[[f64; 3]; 2]], footprints: &[[[f64; 3]; 2]]) -> [Node; 2] {
    let strips = |segments: &[[[f64; 3]; 2]]| segments.iter().map(|[a, b]| vec![*a, *b]).collect();
    [
        Node::new(
            "floor_plan/lines",
            NodeKind::Lines {
                strips: strips(segments),
                colors: vec![[0x8a, 0x8f, 0x98, 0xff]],
                radius: Some(0.005),
                labels: Vec::new(),
                show_labels: None,
            },
        ),
        Node::new(
            "floor_plan/footprints",
            NodeKind::Lines {
                strips: strips(footprints),
                colors: vec![[0x7c, 0x4d, 0xff, 0xff]],
                radius: Some(0.01),
                labels: Vec::new(),
                show_labels: None,
            },
        ),
    ]
}

/// What makes the nodes under `prefix` exactly `nodes`: upsert them, remove the others of `doc`.
fn replace_under(doc: &SceneDoc, prefix: &str, nodes: Vec<Node>) -> (Vec<Node>, Vec<String>) {
    let remove = (doc.nodes.iter())
        .filter(|n| is_under(&n.id, prefix) && !nodes.iter().any(|m| m.id == n.id))
        .map(|n| n.id.clone())
        .collect();
    (nodes, remove)
}

const FITTER_TITLE: &str = "Robo Fitter";

/// The 3D view's title: what it shows. "Cell <cell>" while linked to a cell (the simulation of
/// step 1 moves its robots), else "Robo Fitter · <scene>" (the scene being edited).
fn view_title(state: &FitterState) -> String {
    match &state.linked_cell {
        Some(cell) => format!("Cell {cell}"),
        None => {
            let name = if state.scene.name.trim().is_empty() { &state.scene_file_name } else { &state.scene.name };
            format!("{FITTER_TITLE} · {name}")
        }
    }
}

/// Whether the robots linked to `linked` follow the simulation: it runs that cell. Picking another
/// scene in step 1 (the active cell) does not stop it, so the view keeps playing what the chip names
/// until the next Load & Play. Only when the simulation does not say its cell (an older one) is the
/// active cell taken for it.
fn follows(linked: &str, active: Option<&str>, sim_cell: Option<&str>) -> bool {
    match sim_cell {
        Some(cell) => cell == linked,
        None => active == Some(linked),
    }
}

/// Whether the title `sent` last must be sent again as `title`: when the view changes between a
/// cell's and the Robo Fitter's.
fn retitled(sent: Option<&str>, title: &str) -> bool {
    let linked = |title: &str| !title.starts_with(FITTER_TITLE);
    sent.is_some_and(|sent| linked(sent) != linked(title))
}

/// Below a cell's welds (at `welds`) when the scene has no fixture to carry them (an umarell cell,
/// `panel`): a thin translucent panel (`world/product/panel`), the welds' bounding rectangle plus
/// a margin. Visual only; the welds themselves are the task (`world/task`, see [`draw`]).
fn panel_node(welds: &[[f64; 3]], origin: [f64; 3], panel: bool) -> Option<Node> {
    if welds.is_empty() || !panel {
        return None;
    }
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for p in welds {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i] - origin[i]);
            hi[i] = hi[i].max(p[i] - origin[i]);
        }
    }
    let (margin, thickness, gap) = (0.08, 0.004, 0.015);
    let top = lo[2] - gap;
    Some(Node::new(
        format!("{PRODUCT}/panel"),
        NodeKind::Boxes {
            centers: vec![[(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, top - thickness / 2.0]],
            half_sizes: vec![[(hi[0] - lo[0]) / 2.0 + margin, (hi[1] - lo[1]) / 2.0 + margin, thickness / 2.0]],
            quats_xyzw: Vec::new(),
            colors: vec![[0x9f, 0xb4, 0xc8, 0x80]],
            labels: vec!["car door panel".to_owned()],
            solid: true,
            show_labels: None,
        },
    ))
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::FITTER_STATE, get(state))
        .route(api::FITTER_OPEN, post(open))
        .route(api::FITTER_NEW, post(new_scene))
        .route(api::FITTER_LOAD, post(load))
        .route(api::FITTER_SAVE, post(save))
        .route(api::FITTER_COMPOSE, post(compose))
        .route(api::FITTER_CREATE_CELL, post(create_cell))
        .route(api::FITTER_UNLINK, post(unlink))
        .route(api::FITTER_EDIT, post(edit))
        .route(api::FITTER_OPEN_CELL, post(open_cell))
        .route(api::FITTER_DELETE, post(delete))
        .route(api::FITTER_VIEW, post(view))
        .route(api::FITTER_REACH_CHECK, post(reach_check))
        .route(api::FITTER_REACH_CANCEL, post(reach_cancel))
        .route(api::FITTER_REACH_POSE, post(reach_pose))
        .merge(reach::http::router())
}

/// Run `action` off the async runtime (they parse scenes, load floor plans and build the 3D scene).
async fn blocking(state: &AppState, action: impl FnOnce(&Arc<FitterService>) + Send + 'static) -> Json<FitterState> {
    let fitter = state.fitter.clone();
    let fitter = tokio::task::spawn_blocking(move || {
        action(&fitter);
        fitter
    })
    .await
    .expect("fitter action panicked");
    Json(fitter.state())
}

async fn state(State(state): State<AppState>) -> Json<FitterState> {
    Json(state.fitter.state())
}

async fn open(State(state): State<AppState>) -> Json<FitterState> {
    blocking(&state, |fitter| fitter.open()).await
}

async fn new_scene(State(state): State<AppState>) -> Json<FitterState> {
    blocking(&state, |fitter| fitter.new_empty_scene()).await
}

async fn load(State(state): State<AppState>, Json(request): Json<LoadRequest>) -> Json<FitterState> {
    blocking(&state, move |fitter| fitter.load(&request.file_name)).await
}

async fn delete(State(state): State<AppState>, Json(request): Json<LoadRequest>) -> Json<FitterState> {
    blocking(&state, move |fitter| fitter.delete(&request.file_name)).await
}

/// Save, then Prepare Cell for the scene's cells whose planner model is stale or not made yet
/// (nothing is stopped: their other outputs made from the old scene are stale).
async fn save(State(state): State<AppState>, Json(edit): Json<SceneEdit>) -> Json<FitterState> {
    let fitter = state.fitter.clone();
    let cells = tokio::task::spawn_blocking(move || fitter.save(edit)).await.expect("fitter save panicked");
    state.agentic.prepare_cells(&cells);
    Json(state.fitter.state())
}

/// Save and generate the USDA; once generated, Prepare Cell for the scene's cells (the USDA is an input).
async fn compose(State(state): State<AppState>, Json(request): Json<ComposeRequest>) -> Json<FitterState> {
    let fitter = state.fitter.clone();
    let job = tokio::task::spawn_blocking(move || fitter.compose(request)).await.expect("fitter compose panicked");
    if let Some(job) = job {
        let agentic = state.agentic.clone();
        tokio::spawn(async move {
            if let Ok(cells) = job.await {
                agentic.prepare_cells(&cells);
            }
        });
    }
    Json(state.fitter.state())
}

/// "Use as cell" (the old "Create Cell"): see [`FitterService::create_cell`]; then Prepare Cell.
async fn create_cell(State(state): State<AppState>, Json(request): Json<CreateCellRequest>) -> Json<FitterState> {
    let fitter = state.fitter.clone();
    let created = tokio::task::spawn_blocking(move || fitter.create_cell(request)).await.expect("fitter create cell panicked");
    if let Ok(config) = created {
        let cell = super::cells::active_cell(&state.config).map(|(cell, _)| cell).unwrap_or_default();
        state.bus.log(format!("Using cell {cell} ({})", config.env_dir));
        state.bus.put_json(keys::CONFIG, &config);
        state.agentic.prepare();
    }
    Json(state.fitter.state())
}

async fn open_cell(State(state): State<AppState>, Json(request): Json<OpenCellRequest>) -> Json<FitterState> {
    let fitter = state.fitter.clone();
    let cell = request.cell.clone();
    let opened = tokio::task::spawn_blocking(move || fitter.open_cell(&cell))
        .await
        .expect("open cell panicked");
    match opened {
        Ok(config) => state.bus.put_json(keys::CONFIG, &config),
        Err(err) => state.fitter.set_status(format!("Open cell {} failed: {err:#}", request.cell)),
    }
    Json(state.fitter.state())
}

async fn view(State(state): State<AppState>, Json(view): Json<FitterView>) -> Json<FitterState> {
    blocking(&state, move |fitter| fitter.set_view(view)).await
}

async fn reach_check(State(state): State<AppState>, Json(request): Json<ReachCheckRequest>) -> Json<FitterState> {
    blocking(&state, move |fitter| fitter.reach_check(request)).await
}

async fn reach_cancel(State(state): State<AppState>) -> Json<FitterState> {
    blocking(&state, |fitter| fitter.reach_cancel()).await
}

async fn reach_pose(State(state): State<AppState>, Json(request): Json<ReachPoseRequest>) -> Json<FitterState> {
    blocking(&state, move |fitter| fitter.reach_pose(request)).await
}

async fn unlink(State(state): State<AppState>) -> Json<FitterState> {
    blocking(&state, |fitter| fitter.unlink()).await
}

/// "Stop simulation and edit": stop the simulated cell-runner, then [`FitterService::edit`]. Never
/// in the middle of a Load & Play opening its scene or starting the simulation (the flow lock); one
/// still waiting for the simulation to come up gives up with "Load & Play cancelled: …".
async fn edit(State(state): State<AppState>) -> Json<FitterState> {
    let flow = state.cell.flow_lock().await;
    state.cell.stop_for_edit(&flow).await;
    let edited = blocking(&state, |fitter| fitter.edit()).await;
    drop(flow);
    edited
}

#[cfg(test)]
mod tests {
    use factoryos_protocol::fitter::TaskWeld;

    use super::*;

    #[test]
    fn view_titles_say_what_is_shown() {
        let mut state = FitterState { scene_file_name: "two.yaml".to_owned(), ..Default::default() };
        state.scene.name = "Two robots".to_owned();
        assert_eq!(view_title(&state), "Robo Fitter · Two robots");
        state.scene.name.clear();
        assert_eq!(view_title(&state), "Robo Fitter · two.yaml");
        state.linked_cell = Some("welding_cell_2_small".to_owned());
        assert_eq!(view_title(&state), "Cell welding_cell_2_small");
    }

    #[test]
    fn linked_robots_follow_only_their_cells_simulation() {
        assert!(follows("a", Some("a"), Some("a")));
        assert!(follows("a", Some("a"), None), "the simulation's cell not known yet");
        assert!(!follows("a", Some("a"), Some("b")), "another cell is simulated");
        assert!(follows("a", Some("b"), Some("a")), "another scene picked for the next Load & Play");
        assert!(!follows("a", Some("b"), None), "unknown simulation, another cell active");
    }

    #[test]
    fn edit_opens_the_cells_source_scene_or_a_new_one() {
        let root = std::env::temp_dir().join(format!("factoryos-fitter-edit-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (scenes, cell) = (root.join("scenes"), root.join("cells/weld"));
        std::fs::create_dir_all(&scenes).unwrap();
        std::fs::create_dir_all(&cell).unwrap();
        let active = ("weld".to_owned(), cell.clone());

        assert_eq!(edit_target(false, Some("weld"), Some(&active), &scenes), EditTarget::Unlink, "a Create Cell scene");
        assert_eq!(edit_target(true, Some("weld"), Some(&active), &scenes), EditTarget::New, "not made from a scene");
        // A legacy manifest's `two.yaml` names scene `two`.
        std::fs::write(cell.join("factoryos-cell.json"), r#"{"fitted_scene": "two.yaml"}"#).unwrap();
        assert_eq!(edit_target(true, Some("weld"), Some(&active), &scenes), EditTarget::New, "its scene is gone");
        std::fs::create_dir_all(scenes.join("two")).unwrap();
        std::fs::write(scenes.join("two/scene.yaml"), "name: two\n").unwrap();
        assert_eq!(edit_target(true, Some("weld"), Some(&active), &scenes), EditTarget::Load("two".to_owned()));
        assert_eq!(edit_target(true, Some("other"), Some(&active), &scenes), EditTarget::New, "linked to another cell");
        std::fs::remove_dir_all(&root).ok();
    }

    /// A scene made from cell jam2 (its paths to the cell's copies) is saved with the
    /// catalogue's robot and the product, and the editor adopts that.
    #[test]
    fn saving_a_scene_from_a_cell_writes_the_original_paths() {
        let (root, originals) = scene_paths::tests::host("fitter-save");
        let mut inner = Inner { root: originals.catalogue.clone(), ..Default::default() };
        inner.state.scene = serde_yaml::from_str(&scene_paths::tests::scene_fit(originals.cells.as_ref().unwrap())).unwrap();
        inner.state.scene_file_name = "scene-fit".to_owned();
        let seq = inner.state.scene_seq;
        let (path, _) = inner.save_scene(&originals, &|_| false).unwrap();
        assert_eq!(path, originals.scenes_dir().join("scene-fit/scene.yaml"));
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("dressed_urdf: ../../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf"), "{text}");
        assert!(text.contains(&format!("urdf: {}", originals.products.join("bug_cube/bug_cube.urdf").display())), "{text}");
        assert!(!text.contains("cells/"), "{text}");
        assert_eq!(inner.state.scene_seq, seq + 1, "the editor adopts the rewritten scene");
        assert_eq!(inner.state.saved.as_ref().map(|s| &s.scene), Some(&inner.state.scene));
        inner.save_scene(&originals, &|_| false).unwrap();
        assert_eq!(inner.state.scene_seq, seq + 1, "nothing to rewrite the second time");
        std::fs::remove_dir_all(&root).ok();
    }

    /// A manifest with what the backend models only in part (payloads, a pedestal, footprint
    /// layers) and keys it doesn't know, at every level.
    const MANIFEST: &str = r#"name: two-arms
description: a synthetic cell
scene_origin: {x: 10.0, y: -4.0, z: 0.0}
layout_rev: 7
robots:
- robot_id: arm_a
  dressed_urdf: ../../urdf-dressed-robots/arm/arm.urdf
  position: {x: 12.5, y: -3.25, z: 0.0}
  orientation: {a: 90.0, b: 0.0, c: 12.345678}
  home_pose_deg: [0.0, -90.0, 90.0, 0.0, 12.345678, 0.0]
  pedestal: {height_mm: 600.0, diameter_mm: 900.0, material: concrete}
  payload:
    mass_kg: 42.5
    center_of_mass_m: {x: 0.1, y: 0.0, z: 0.25}
    inertia_kgm2: {ixx: 1.5, iyy: 2.0, izz: 2.5}
    source: datasheet
  controller: {address: 10.0.0.7, cycle_ms: 4}
  link_density_kg_m3: 1600.0
- robot_id: arm_b
  dressed_urdf: ../../urdf-dressed-robots/arm/arm.urdf
  position: {x: 14.0, y: -3.25, z: 0.0}
  orientation: {a: -90.0, b: 0.0, c: 0.0}
  payload: {mass_kg: 8.0}
fixtures:
- id: table
  urdf: ../../urdf-fixture-collection/table/table.urdf
  position: {x: 13.0, y: -1.0, z: 0.0}
  orientation: {a: 0.0, b: 0.0, c: 0.0}
  clamps: [{name: c1, closed: false}, {name: c2, closed: null}]
floor_plan:
  geometry_file: ../../floor-plans/plan.obj
  offset: {x: -10.0, y: 4.0, z: 0.0}
  scale: 1.0
  footprint_layers: [FOOTPRINTS, base_plates]
  line_width_m: 0.02
"#;

    /// A host with the floor plan [`MANIFEST`] names, and scene `file` (`scenes/<file>/scene.yaml`)
    /// holding `manifest`, opened as "Load" opens it.
    fn host_with_manifest(tag: &str, file: &str, manifest: &str) -> (PathBuf, Inner, Originals) {
        let root = std::env::temp_dir().join(format!("factoryos-fitter-{tag}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        for dir in ["scenes", "floor-plans"] {
            std::fs::create_dir_all(root.join(dir)).unwrap();
        }
        // A wall, and a plate in a layer only `footprint_layers` makes a footprint.
        let plan = "o walls\nv 0 0 0\nv 4 0 0\nl 1 2\no BASE_PLATES\nv 1 1 0\nv 2 1 0\nl 3 4\n";
        std::fs::write(root.join("floor-plans/plan.obj"), plan).unwrap();
        std::fs::create_dir_all(root.join("scenes").join(file)).unwrap();
        std::fs::write(root.join("scenes").join(file).join("scene.yaml"), manifest).unwrap();
        let originals = Originals { catalogue: root.clone(), products: root.join("products"), cells: None };
        let mut inner = Inner { root: root.clone(), ..Default::default() };
        inner.state.scene = serde_yaml::from_str(manifest).unwrap();
        inner.state.scene_file_name = file.to_owned();
        inner.state.saved = Some(SceneEdit { file_name: file.to_owned(), scene: inner.state.scene.clone() });
        (root, inner, originals)
    }

    /// Load, save, edit in the viewer (the scene goes there and back as JSON) and save again: the
    /// file says what the manifest said, but for the edit.
    #[test]
    fn saving_keeps_every_key_of_the_manifest() {
        use scene_paths::tests::yaml_value;
        let (root, mut inner, originals) = host_with_manifest("keys", "two-arms", MANIFEST);
        let (path, _) = inner.save_scene(&originals, &|_| false).unwrap();
        let saved = std::fs::read_to_string(&path).unwrap();
        assert_eq!(yaml_value(&saved), yaml_value(MANIFEST), "{saved}");
        assert_eq!(inner.state.saved.as_ref().map(|s| &s.scene), Some(&serde_yaml::from_str::<SceneFile>(&saved).unwrap()));

        let wire = serde_json::to_string(&SceneEdit { file_name: "two-arms".into(), scene: inner.state.scene.clone() }).unwrap();
        let mut edit: SceneEdit = serde_json::from_str(&wire).unwrap();
        edit.scene.robots[1].position.x = 15.5;
        inner.state.scene = edit.scene;
        inner.save_scene(&originals, &|_| false).unwrap();
        let edited = std::fs::read_to_string(&path).unwrap();
        let mut expected = yaml_value(MANIFEST);
        expected["robots"][1]["position"]["x"] = 15.5.into();
        assert_eq!(yaml_value(&edited), expected, "{edited}");
        inner.save_scene(&originals, &|_| false).unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), edited, "saving again changes nothing");
        std::fs::remove_dir_all(&root).ok();
    }

    /// "Generate USDA" composes the manifest as saved: the payloads, the pedestal and the
    /// footprint layers reach the composer, and so do keys the backend doesn't model.
    #[test]
    fn compose_gets_the_payloads_of_the_saved_manifest() {
        use scene_paths::tests::yaml_value;
        let (root, mut inner, originals) = host_with_manifest("compose", "two-arms", MANIFEST);
        let (manifest, _) = inner.save_scene(&originals, &|_| false).unwrap();
        let scene = inner.compose_definition(&manifest).unwrap();
        let payload = scene.robots[0].payload.clone().expect("arm_a's payload");
        assert_eq!(payload.mass_kg, 42.5);
        assert_eq!(payload.center_of_mass_m, Some([0.1, 0.0, 0.25]));
        assert_eq!(payload.inertia_kgm2, Some([1.5, 2.0, 2.5]));
        let light = scene.robots[1].payload.clone().expect("arm_b's payload");
        assert_eq!((light.mass_kg, light.center_of_mass_m, light.inertia_kgm2), (8.0, None, None));
        let pedestal = scene.robots[0].pedestal.expect("arm_a's pedestal");
        assert_eq!((pedestal.height_mm, pedestal.diameter_mm), (600.0, 900.0));
        assert_eq!(scene.robots[0].home_pose_deg.as_deref(), Some(&[0.0, -90.0, 90.0, 0.0, 12.345678, 0.0][..]));
        let plan = scene.floor_plan.as_ref().expect("the floor plan");
        assert_eq!((plan.segments.len(), plan.footprint_segments.len()), (1, 1), "the listed layer is a footprint");
        let input = yaml_value(&std::fs::read_to_string(&manifest).unwrap());
        assert_eq!(input["floor_plan"]["footprint_layers"], yaml_value("[FOOTPRINTS, base_plates]"));
        assert_eq!(input["robots"][0]["link_density_kg_m3"], yaml_value("1600.0"), "a key the backend doesn't model");

        // The scene as shown (the 3D view's diagnostics), and a cell's own scene, which has no
        // file in scenes/, have them too.
        assert_eq!(inner.definition().unwrap().robots[0].payload, Some(payload.clone()));
        inner.state.cell_scene = true;
        assert_eq!(inner.compose_definition(&root.join("nowhere.yaml")).unwrap().robots[0].payload, Some(payload));
        std::fs::remove_dir_all(&root).ok();
    }

    /// j1 (revolute) and j2 (prismatic) between three links.
    const ARM: &str = r#"<robot name="arm">
      <link name="base"/><link name="l1"/><link name="l2"/>
      <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
      <joint name="j2" type="prismatic"><parent link="l1"/><child link="l2"/><axis xyz="1 0 0"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>"#;

    fn loaded(urdf: &str) -> Placed {
        let model = ModelRef { urdf: factoryos_protocol::scene::AssetRef { sha256: "ab".into(), ..Default::default() }, ..Default::default() };
        Placed { urdf: PathBuf::from(urdf), loaded: Some(Arc::new(Loaded { urdf: ARM.parse().unwrap(), model })) }
    }

    fn close(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-9, "{got} != {want}");
    }

    /// The scene of [`MANIFEST`], its URDFs relative to `/s`.
    fn manifest_items() -> (SceneFile, Vec<Item>) {
        let scene: SceneFile = serde_yaml::from_str(MANIFEST).unwrap();
        let items = scene_items(&scene, Path::new("/s"));
        (scene, items)
    }

    #[test]
    fn items_are_placed_with_the_old_math() {
        let (scene, items) = manifest_items();
        let keys: Vec<&str> = items.iter().map(|i| i.key.as_str()).collect();
        assert_eq!(keys, ["robots/arm_a", "robots/arm_b", "fixtures/table"]);
        assert_eq!(items[0].urdf, Path::new("/s/../../urdf-dressed-robots/arm/arm.urdf"));
        let origin = scene.scene_origin.unwrap();

        // arm_a at (12.5, -3.25, 0) − origin (10, −4, 0), lifted 600 mm onto its pedestal, A = 90°.
        let pose = placement(&items[0], origin);
        assert_eq!(pose.xyz, [2.5, 0.75, 0.6]);
        assert_eq!(pose, pose3_from_xyz_abc_deg([2.5, 0.75, 0.6], [90.0, 0.0, 12.345678]));
        let Some(Node { id, parent: None, pose: pedestal_pose, kind: NodeKind::Cylinder { height, radius, center, color, solid: true }, .. }) =
            pedestal_node(&items[0], &pose)
        else {
            panic!("a pedestal in the world frame")
        };
        assert_eq!(id, "robots/arm_a/pedestal");
        close(height, 0.6);
        close(radius, 0.45);
        assert_eq!(pedestal_pose, Pose3::from_xyz([2.5, 0.75, 0.0]), "on the floor under the base");
        assert_eq!(center, [0.0, 0.0, 0.3], "from the floor up to the base");
        assert_eq!(color, Some([0x5a, 0x5a, 0x64, 0xff]));

        // arm_b: no pedestal, no lift; A = -90°.
        let pose = placement(&items[1], origin);
        assert_eq!(pose.xyz, [4.0, 0.75, 0.0]);
        close(pose.quat_xyzw[2], -std::f64::consts::FRAC_1_SQRT_2);
        assert!(pedestal_node(&items[1], &pose).is_none());
        let mut flat = items[0].clone();
        flat.pedestal = Some(PedestalYaml::new(0.0, 900.0));
        assert!(pedestal_node(&flat, &placement(&flat, origin)).is_none(), "zero height: none");
        assert_eq!(placement(&flat, origin).xyz[2], 0.0);
    }

    /// `sync_3d`'s diff: what is gone (or has another URDF) is removed, what has loaded is upserted
    /// with its joints (at a weld, else the cell's, else home), a pedestal that went is removed.
    #[test]
    fn placed_items_diff_into_upserts_and_removes() {
        let (scene, items) = manifest_items();
        let origin = scene.scene_origin.unwrap();
        let arm = "/s/../../urdf-dressed-robots/arm/arm.urdf";
        let mut placed = HashMap::from([
            ("robots/arm_a".to_owned(), loaded(arm)),
            ("robots/arm_b".to_owned(), loaded(arm)),
            ("robots/gone".to_owned(), loaded(arm)),
            ("fixtures/table".to_owned(), loaded("/s/old/table.urdf")),
        ]);
        let stale = stale_keys(&placed, &items);
        assert_eq!(stale, ["fixtures/table", "robots/gone"]);
        for key in &stale {
            placed.remove(key);
        }
        placed.insert("fixtures/table".to_owned(), Placed { urdf: items[2].urdf.clone(), loaded: None });

        let (upsert, remove) = item_nodes(&items, origin, &placed, &HashMap::new(), None);
        let ids: Vec<&str> = upsert.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(
            ids,
            ["robots/arm_a", "robots/arm_a/name_label", "robots/arm_a/pedestal", "robots/arm_b", "robots/arm_b/name_label"],
            "the table is still loading"
        );
        // Each robot carries its name, on its own node (it moves with it), always shown.
        for (i, name) in [(1, "arm_a"), (4, "arm_b")] {
            let label = &upsert[i];
            assert_eq!(label.parent.as_ref().map(|p| (p.node.as_str(), p.link.as_deref())), Some((upsert[i - 1].id.as_str(), None)));
            let NodeKind::Points { labels, show_labels, points, .. } = &label.kind else { panic!("{label:?}") };
            assert_eq!((labels.as_slice(), *show_labels), ([name.to_owned()].as_slice(), Some(true)));
            assert!(points[0][2] > 0.0, "above the base");
        }
        assert_eq!(remove, ["robots/arm_b/pedestal"]);
        let NodeKind::Model { model, joints, show_collision } = &upsert[0].kind else { panic!() };
        assert_eq!(model.urdf.sha256, "ab");
        assert!(!show_collision);
        assert_eq!(upsert[0].pose, placement(&items[0], origin));
        // arm_a's home_pose_deg [0, -90, …]: j2 is prismatic (mm).
        assert_eq!(joints.keys().collect::<Vec<_>>(), ["j1", "j2"]);
        close(joints["j1"], 0.0);
        close(joints["j2"], -0.09);
        // arm_b has no home pose: the default one, the same here.
        let NodeKind::Model { joints: default_home, .. } = &upsert[3].kind else { panic!() };
        assert_eq!(default_home, joints);

        // Linked: the cell's joints (radians / metres in URDF order); at a weld: those.
        let cell = HashMap::from([("robots/arm_b".to_owned(), vec![0.5, 0.25])]);
        let at_weld = HashMap::from([("robots/arm_a".to_owned(), JointMap::from([("j1".into(), 1.0), ("j2".into(), 0.0)]))]);
        let (upsert, _) = item_nodes(&items, origin, &placed, &at_weld, Some(&cell));
        let joints = |i: usize| match &upsert[i].kind {
            NodeKind::Model { joints, .. } => joints.clone(),
            _ => panic!(),
        };
        assert_eq!(joints(0), at_weld["robots/arm_a"]);
        assert_eq!(joints(3), JointMap::from([("j1".into(), 0.5), ("j2".into(), 0.25)]));

        // The joint-only path (reach pose, cell joints) sends the same values by name.
        let mut inner = Inner { placed, reach_pose: at_weld, ..Default::default() };
        inner.state.scene = scene;
        let frames = robot_joints(&inner);
        assert_eq!(frames.iter().map(|m| m.node.as_str()).collect::<Vec<_>>(), ["robots/arm_a", "robots/arm_b"]);
        assert_eq!(frames[0].joints, joints(0));
    }

    #[test]
    fn weld_poses_are_the_iks_joints_by_name() {
        let mut inner = Inner::default();
        inner.state.scene = serde_yaml::from_str(MANIFEST).unwrap();
        inner.placed = HashMap::from([("robots/arm_a".to_owned(), loaded("a.urdf")), ("robots/arm_b".to_owned(), loaded("a.urdf"))]);
        let weld = TaskWeld { id: "p/W1".into(), label: "W1".into(), ..Default::default() };
        let attempt = |robot: &str, reachable: bool| factoryos_protocol::fitter::ReachAttempt {
            robot_id: robot.into(),
            reachable,
            joints_deg: vec![250.0, 30.0], // In the IK chain's order: j2 (mm), j1 (°).
            ..Default::default()
        };
        inner.state.reach = Some(ReachReport {
            welds: vec![factoryos_protocol::fitter::WeldReach {
                weld,
                attempts: vec![attempt("arm_a", true), attempt("arm_b", false)],
                ..Default::default()
            }],
            ..Default::default()
        });
        let chains = HashMap::from([("arm_a", vec!["j2".to_owned(), "j1".to_owned()]), ("arm_b", vec!["j2".to_owned(), "j1".to_owned()])]);
        let (poses, note) = weld_poses_with(&inner, "p/W1", &chains).unwrap();
        close(poses["robots/arm_a"]["j1"], 30f64.to_radians());
        close(poses["robots/arm_a"]["j2"], 0.25);
        assert_eq!(poses["robots/arm_b"], ARM.parse::<UrdfModel>().unwrap().home_joint_map(None), "arm_b doesn't reach it: home");
        assert_eq!(note, "W1: arm_a at the weld; arm_b at home.");
        assert!(weld_poses_with(&inner, "p/W9", &chains).unwrap_err().contains("not in the last check"));
        inner.state.reach = None;
        assert!(weld_poses_with(&inner, "p/W1", &chains).unwrap_err().starts_with("Check reachability first"));
    }

    #[test]
    fn task_and_panel_nodes_replace_what_was_there() {
        let doc = SceneDoc {
            nodes: ["robots/r1", "world/task/welds", "world/task/paths", "world/taskx"]
                .map(|id| Node::new(id, NodeKind::Group))
                .to_vec(),
            ..Default::default()
        };
        let (upsert, remove) = replace_under(&doc, draw::TASK, vec![Node::new("world/task/welds", NodeKind::Group)]);
        assert_eq!((upsert.len(), remove), (1, vec!["world/task/paths".to_owned()]));
        let (_, remove) = replace_under(&doc, draw::TASK, Vec::new());
        assert_eq!(remove, ["world/task/welds", "world/task/paths"], "hidden: all of it");

        let welds = [[1.0, 1.0, 1.0], [2.0, 3.0, 1.0]];
        assert!(panel_node(&welds, [1.0, 0.0, 0.0], false).is_none(), "a fixture carries the welds");
        assert!(panel_node(&[], [0.0; 3], true).is_none());
        let Some(Node { id, kind: NodeKind::Boxes { centers, half_sizes, colors, labels, solid, .. }, .. }) =
            panel_node(&welds, [1.0, 0.0, 0.0], true)
        else {
            panic!()
        };
        assert_eq!(id, "world/product/panel");
        let (c, h) = (centers[0], half_sizes[0]);
        for (got, want) in [(c[0], 0.5), (c[1], 2.0), (c[2], 1.0 - 0.015 - 0.002), (h[0], 0.58), (h[1], 1.08), (h[2], 0.002)] {
            close(got, want);
        }
        assert!(solid && colors == [[0x9f, 0xb4, 0xc8, 0x80]] && labels == ["car door panel"]);

        let [lines, footprints] = floor_plan_nodes(&[[[0.0; 3], [4.0, 0.0, 0.0]]], &[]);
        assert_eq!((lines.id.as_str(), footprints.id.as_str()), ("floor_plan/lines", "floor_plan/footprints"));
        assert!(matches!(lines.kind, NodeKind::Lines { ref strips, radius: Some(0.005), .. } if strips == &[vec![[0.0; 3], [4.0, 0.0, 0.0]]]));
    }

    #[test]
    fn the_title_is_sent_again_when_linked_or_unlinked() {
        assert!(!retitled(None, "Cell a"), "not sent yet: sent with the scene");
        assert!(retitled(Some("Robo Fitter · two"), "Cell a"));
        assert!(retitled(Some("Cell a"), "Robo Fitter · two"));
        assert!(!retitled(Some("Robo Fitter · two"), "Robo Fitter · three"), "renaming waits for the next layout change");
    }

    #[test]
    fn saving_says_which_cells_it_made_stale() {
        assert_eq!(stale_note(&[]), "");
        assert_eq!(stale_note(&["jam2".to_owned()]), "; preparing cell jam2 for planning");
        assert_eq!(stale_note(&["a".to_owned(), "b".to_owned()]), "; preparing cells a, b for planning");
    }

    /// Saving names the folder after the scene's `name`: renaming moves the folder unless a cell
    /// uses the scene (then a new scene, the old one kept); another scene's name is never taken.
    #[test]
    fn saving_names_the_folder_after_the_scene() {
        let (root, mut inner, originals) = host_with_manifest("rename", "new-scene", MANIFEST);
        let scenes = root.join("scenes");
        // The user's case: the scene is called bw48-1, its folder new-scene.
        inner.state.scene.name = "bw48-1".to_owned();
        let (path, note) = inner.save_scene(&originals, &|_| false).unwrap();
        assert_eq!((path, note.as_str()), (scenes.join("bw48-1/scene.yaml"), " (renamed from new-scene)"));
        assert!(!scenes.join("new-scene").exists(), "the folder moved");
        assert_eq!((inner.state.scene_file_name.as_str(), inner.state.saved.as_ref().unwrap().file_name.as_str()), ("bw48-1", "bw48-1"));
        let saved = std::fs::read_to_string(scenes.join("bw48-1/scene.yaml")).unwrap();
        assert!(saved.starts_with("name: bw48-1\n") && saved.contains("../../floor-plans/plan.obj"), "{saved}");

        // In use by a cell: a new scene, the old one stays.
        inner.state.scene.name = "bw48-2".to_owned();
        let (_, note) = inner.save_scene(&originals, &|old| old == "bw48-1").unwrap();
        assert_eq!(note, " (a new scene: bw48-1 is kept, cells use it)");
        assert_eq!(scene_store::list(&scenes), ["bw48-1", "bw48-2"]);

        // Another scene's name: never overwritten.
        inner.state.scene.name = "bw48-1".to_owned();
        let (path, note) = inner.save_scene(&originals, &|_| true).unwrap();
        assert_eq!((path, note.as_str()), (scenes.join("bw48-1-2/scene.yaml"), " (a scene bw48-1 exists: saved as bw48-1-2) (a new scene: bw48-2 is kept, cells use it)"));
        assert_eq!(inner.state.scene.name, "bw48-1-2", "the editor adopts the name as saved");
        // Typed names are cleaned.
        inner.state.scene.name = "Weld 7.yaml".to_owned();
        let (path, _) = inner.save_scene(&originals, &|_| true).unwrap();
        assert_eq!(path, scenes.join("Weld-7/scene.yaml"));
        inner.state.scene.name = "  ".to_owned();
        assert_eq!(inner.save_scene(&originals, &|_| true).unwrap().0, scenes.join("Weld-7/scene.yaml"), "no name: the one it has");
        std::fs::remove_dir_all(&root).ok();
    }
}
