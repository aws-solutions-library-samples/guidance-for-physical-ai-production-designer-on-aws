//! Robo Fitter panels. The scene is edited here and put on `factoryos/fitter/scene` on every
//! change; factoryos-backend (`services/fitter.rs`) validates it, renders it in 3D (streamed back
//! as its own recording) and saves/composes it with the `robo_fitting_room` library.

use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex};

use factoryos_client::protocol::fitter::{
    Abc, ComposeReport, ComposeRequest, CreateCellRequest, FitterState, FitterView, OpenCellRequest, FixtureYaml, FloorPlanYaml, LoadRequest, NEW_SCENE_FILE, PedestalYaml,
    ReachAttempt, ReachCheckRequest, ReachMode, ReachPoseRequest, ReachReason, ReachReport, RobotYaml, SceneEdit, SceneFile, SourceCell, Suggestion, TaskSource,
    WeldStatus, Xyz,
};
use factoryos_client::protocol::{CellKind, api, keys};
use factoryos_client::{Api, Bus};
use crate::theme;
use crate::rr::egui::{self, Color32, RichText};
use crate::rr::re_log;
use crate::rr::re_ui::{self, UiExt as _};
use crate::rr::re_log_types::StoreId;

use crate::backend::Backend;
use crate::scene3d::RouteRequest;
use crate::scene3d::Scene3d;

/// The weld colours, as the backend draws them in 3D (`world/task/welds`).
const WELD_GREEN: Color32 = Color32::from_rgb(63, 185, 80);
const WELD_ORANGE: Color32 = Color32::from_rgb(240, 136, 62);
const WELD_RED: Color32 = Color32::from_rgb(248, 81, 73);
const WELD_GREY: Color32 = Color32::from_rgb(139, 148, 158);
/// Auto play shows the next reachable weld this often.
const PLAY_STEP_S: f64 = 1.5;

/// What [`Fitter::backend_restarted`] opens again after the Robo Fitter itself.
#[derive(Clone, Debug, PartialEq)]
enum Resume {
    Cell(String),
    File(String),
}

impl Resume {
    /// What `state` shows, to open again on a restarted backend; `None` for a backend's state
    /// before anything opened it (no asset root yet: a restarted one), or nothing to resume.
    fn of(state: &FitterState) -> Option<Option<Self>> {
        if state.root.is_empty() {
            return None;
        }
        Some(match (&state.linked_cell, state.cell_scene) {
            (Some(cell), true) => Some(Self::Cell(cell.clone())),
            _ if state.saved.is_some() => Some(Self::File(state.scene_file_name.clone())),
            _ => None,
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Selection {
    Robot(usize),
    Fixture(usize),
    FloorPlan,
}

/// The Robo Fitter's scene as the 3D view's mode needs it.
#[derive(Clone, Debug, Default)]
pub struct FitterFacts {
    pub scene: String,
    pub file: String,
    pub linked_cell: Option<String>,
    pub unsaved: bool,
}

pub struct Fitter {
    api: Api,
    bus: Bus,
    /// The backend's fitter state, as last published.
    state: Arc<Mutex<FitterState>>,
    pub stream: Scene3d,
    /// The asset root last opened.
    opened: Option<String>,

    /// The scene being edited; adopted from the backend whenever it replaces it (new/load).
    scene: SceneFile,
    scene_file_name: String,
    adopted_seq: u64,
    /// The edit last sent to the backend.
    sent: Option<SceneEdit>,
    selection: Option<Selection>,

    full_robots: bool,
    full_tools: bool,
    urdf_bundle: bool,
    /// The saved scene whose trash button was clicked, waiting for "Delete" to be confirmed.
    confirm_delete: Option<String>,

    /// `FACTORY_OS_FITTER` commands still to run, and the scene they wait for (after a load).
    commands: VecDeque<String>,
    awaiting_seq: Option<u64>,

    /// "Check reachability": the full check (with the approach) instead of the quick one.
    full_check: bool,
    /// The weld shown in 3D (`FITTER_REACH_POSE`) and auto play.
    play: PlayState,
    /// Suggestions already applied, by the report's `scene_rev` and their index.
    applied: HashSet<(u64, usize)>,
    /// The scene to open again after a backend restart, as the backend last showed it.
    resume: Option<Resume>,
}

impl Fitter {
    pub fn new(backend: &Backend) -> Self {
        let state = Arc::new(Mutex::new(FitterState::default()));
        let sink = state.clone();
        let repaint = backend.repaint_handle();
        // Also queried now and after a re-join: it is only published when it changes (an open
        // cell, a page loaded later, a restarted backend).
        let subscribed = backend.bus.follow(keys::FITTER_STATE, move |payload| {
            if let Ok(update) = serde_json::from_slice(&payload) {
                *sink.lock().unwrap() = update;
                repaint();
            }
        });
        if let Err(err) = subscribed {
            re_log::warn!("Robo Fitter: {err}");
        }
        Self {
            api: backend.api.clone(),
            bus: backend.bus.clone(),
            state,
            stream: Scene3d::for_backend(backend, keys::SCENE_FITTER, "robo_fitter"),
            opened: None,
            scene: SceneFile::default(),
            scene_file_name: NEW_SCENE_FILE.to_owned(),
            adopted_seq: 0,
            sent: None,
            selection: None,
            // Isaac Sim shows the robots' own meshes and paint (off: collision hulls only, a leaner zip).
            full_robots: true,
            full_tools: true,
            urdf_bundle: false,
            confirm_delete: None,
            commands: VecDeque::new(),
            awaiting_seq: None,
            full_check: false,
            play: PlayState::default(),
            applied: HashSet::new(),
            resume: None,
        }
    }

    /// Have the backend use `asset_root`; it starts a fresh 3D scene the first time (or when the root changes).
    pub fn open(&mut self, asset_root: &str) {
        if self.opened.as_deref() != Some(asset_root) {
            self.opened = Some(asset_root.to_owned());
            self.post(api::FITTER_OPEN, &());
        }
    }

    /// The backend restarted: it has no Robo Fitter open, nor a 3D scene for it. Open it again
    /// if it was open here, on the scene shown: a cell's own scene ("Open Cell") or a saved scene
    /// file (else the backend's first scene, the default one). Unsaved edits are not restored.
    pub fn backend_restarted(&mut self) {
        let Some(asset_root) = self.opened.take() else { return };
        // The state may be the new backend's already (the bus re-joined first): what the old one
        // showed was kept when it was not.
        let resume = Resume::of(&self.state.lock().unwrap()).unwrap_or_else(|| self.resume.clone());
        // The new backend counts its scenes from the start again: adopt whatever it opens.
        self.adopted_seq = u64::MAX;
        self.opened = Some(asset_root);
        let (api, state) = (self.api.clone(), self.state.clone());
        self.api.post_json(api::FITTER_OPEN, &(), move |response: Result<FitterState, String>| {
            match response {
                Ok(update) => *state.lock().unwrap() = update,
                Err(err) => {
                    state.lock().unwrap().status = format!("Failed: {err}");
                    return;
                }
            }
            let adopt = move |response: Result<FitterState, String>| match response {
                Ok(update) => *state.lock().unwrap() = update,
                Err(err) => state.lock().unwrap().status = format!("Failed: {err}"),
            };
            match resume {
                Some(Resume::Cell(cell)) => api.post_json(api::FITTER_OPEN_CELL, &OpenCellRequest { cell }, adopt),
                Some(Resume::File(file_name)) => api.post_json(api::FITTER_LOAD, &LoadRequest { file_name }, adopt),
                None => {}
            }
        });
        self.stream.resync();
    }

    /// Run comma-separated `load:<scene name>` / `compose` commands (from `FACTORY_OS_FITTER`,
    /// for demos and testing).
    pub fn run_command(&mut self, commands: &str) {
        self.commands.extend(commands.split(',').map(str::to_owned));
    }

    pub fn preview_recording(&self) -> Option<StoreId> {
        self.stream.current()
    }

    pub fn take_route_request(&mut self) -> Option<RouteRequest> {
        self.stream.take_route_request()
    }

    /// Whether the weld spots carry their text labels (`FitterView::show_labels`).
    pub fn weld_labels(&self) -> bool {
        self.state.lock().unwrap().view.show_labels
    }

    /// The weld spot labels on or off (the sidebar's "Weld labels"; "Labels" in Task & reach).
    pub fn set_weld_labels(&self, on: bool) {
        let view = {
            let mut state = self.state.lock().unwrap();
            state.view.show_labels = on;
            state.view.clone()
        };
        self.post(api::FITTER_VIEW, &view);
    }

    /// POST `body` to `path`; the resulting state arrives on `factoryos/fitter/state`.
    fn post(&self, path: &'static str, body: &impl serde::Serialize) {
        let state = self.state.clone();
        // The reply is the state after the request: adopted too (an Open of the cell already open changes nothing
        // the bus would publish).
        self.api.post_json(path, body, move |response: Result<FitterState, String>| match response {
            Ok(update) => *state.lock().unwrap() = update,
            Err(err) => state.lock().unwrap().status = format!("Failed: {err}"),
        });
    }

    fn edit(&self) -> SceneEdit {
        SceneEdit {
            file_name: self.scene_file_name.clone(),
            scene: self.scene.clone(),
        }
    }

    /// The cell the 3D view follows ("Use as cell", "Open Cell"), if any.
    pub fn linked_cell(&self) -> Option<String> {
        self.state.lock().unwrap().linked_cell.clone()
    }

    /// The saved scenes (their names, `scenes/<name>/`), as the backend last listed them.
    pub fn saved_scenes(&self) -> Vec<String> {
        self.state.lock().unwrap().palette.scenes.clone()
    }

    /// The scene file, if the scene has items that are not saved as edited:
    /// "Create Cellrunner Cell" only sees saved scenes. Never a cell's own scene ("Open Cell"),
    /// which is not saved from here.
    pub fn unsaved_file(&self) -> Option<String> {
        let has_items = !self.scene.robots.is_empty() || !self.scene.fixtures.is_empty();
        (has_items && self.has_unsaved_edits()).then(|| self.scene_file_name.clone())
    }

    /// The scene differs from its saved file (or was never saved); not for a cell's own scene.
    fn has_unsaved_edits(&self) -> bool {
        let state = self.state.lock().unwrap();
        !state.cell_scene && state.saved.as_ref() != Some(&self.edit())
    }

    fn new_scene(&mut self) {
        self.post(api::FITTER_NEW, &());
    }

    fn load_scene(&mut self, file_name: &str) {
        self.post(api::FITTER_LOAD, &LoadRequest { file_name: file_name.to_owned() });
    }

    fn delete_scene(&self, file_name: &str) {
        self.post(api::FITTER_DELETE, &LoadRequest { file_name: file_name.to_owned() });
    }

    fn save_scene(&self) {
        self.post(api::FITTER_SAVE, &self.edit());
    }

    /// "Use as cell": save the scene, and the backend uses it as a cell (it creates the cell's output
    /// folder, or reuses the one that points at the scene), makes it active and prepares it. The
    /// cell is named after the scene by the backend.
    fn use_as_cell(&self) {
        self.post(api::FITTER_CREATE_CELL, &CreateCellRequest { edit: self.edit(), name: String::new() });
    }

    /// "Open Cell": the backend makes `cell` the active cell and shows its scene, linked to it.
    pub fn open_cell(&self, cell: &str) {
        self.post(api::FITTER_OPEN_CELL, &OpenCellRequest { cell: cell.to_owned() });
    }

    /// "Open Cell" for later (e.g. once a scene's cell exists): sends nothing until called.
    pub fn cell_opener(&self) -> impl FnOnce(&str) + Send + 'static {
        let (api, state) = (self.api.clone(), self.state.clone());
        move |cell: &str| {
            api.post_json(api::FITTER_OPEN_CELL, &OpenCellRequest { cell: cell.to_owned() }, move |response: Result<FitterState, String>| match response {
                Ok(update) => *state.lock().unwrap() = update,
                Err(err) => state.lock().unwrap().status = format!("Failed: {err}"),
            });
        }
    }

    /// "Stop simulation and edit": the backend stops the simulation and opens the scene the cell
    /// was made from (or a new one) for editing; the reply is adopted right away.
    pub fn stop_and_edit(&self) {
        let state = self.state.clone();
        self.api.post_json(api::FITTER_EDIT, &(), move |response: Result<FitterState, String>| match response {
            Ok(update) => *state.lock().unwrap() = update,
            Err(err) => state.lock().unwrap().status = format!("Failed: {err}"),
        });
    }

    /// What the 3D view's mode needs (see [`crate::view_mode`]).
    pub fn view_facts(&self) -> FitterFacts {
        FitterFacts {
            scene: self.scene.name.clone(),
            file: self.scene_file_name.clone(),
            linked_cell: self.linked_cell(),
            unsaved: self.unsaved_file().is_some(),
        }
    }

    /// The scene file the reachability agent checks: the open scene, once saved (or a cell's own
    /// scene), with robots in it; the agent reads the file, not unsaved edits.
    pub fn agent_scene_file(&self) -> Option<String> {
        let state = self.state.lock().unwrap();
        let saved = state.cell_scene || state.saved.is_some();
        (saved && !self.scene.robots.is_empty()).then(|| self.scene_file_name.clone())
    }

    /// Whether the open scene is the linked cell's own `scene.yaml` ("Open Cell").
    pub fn is_cell_scene(&self) -> bool {
        self.state.lock().unwrap().cell_scene
    }

    fn compose(&self) {
        self.post(
            api::FITTER_COMPOSE,
            &ComposeRequest {
                edit: self.edit(),
                full_robots: self.full_robots,
                full_tools: self.full_tools,
                urdf_bundle: self.urdf_bundle,
            },
        );
    }

    /// Adopt scenes the backend replaced, send local edits, and run pending commands. Call every frame.
    pub fn sync_3d(&mut self) {
        let state = self.state.lock().unwrap().clone();
        if let Some(resume) = Resume::of(&state) {
            self.resume = resume;
        }
        if state.scene_seq != self.adopted_seq {
            self.adopted_seq = state.scene_seq;
            self.scene = state.scene.clone();
            self.scene_file_name = state.scene_file_name.clone();
            self.sent = Some(self.edit());
            self.selection = None;
        }
        let edit = self.edit();
        if self.sent.as_ref() != Some(&edit) {
            if let Ok(payload) = serde_json::to_vec(&edit) {
                self.bus.put(keys::FITTER_SCENE, payload);
            }
            self.sent = Some(edit);
        }

        if self.awaiting_seq.is_some_and(|seq| self.adopted_seq < seq) || state.busy.is_some() {
            return;
        }
        self.awaiting_seq = None;
        if let Some(command) = self.commands.pop_front() {
            match command.split_once(':') {
                Some(("load", file)) => {
                    self.load_scene(file);
                    self.awaiting_seq = Some(self.adopted_seq + 1);
                }
                None if command == "compose" => self.compose(),
                _ => re_log::warn!("Unknown FACTORY_OS_FITTER command: {command}"),
            }
        }
    }

    // ------------------------------------------------------------------------
    // UI

    /// The palette (scenes, dressed robots, fixtures, floor plans).
    pub fn palette_ui(&mut self, ui: &mut egui::Ui) {
        let state = self.state.lock().unwrap().clone();
        crate::ui_mode::advanced(ui, |ui| {
            ui.horizontal(|ui| {
                ui.small(RichText::new(&state.root).weak())
                    .on_hover_text("Set in Configuration → Directories → Asset root (Robo Fitter)");
            });
        });

        let palette = &state.palette;
        let (scenes, robots, fixtures, floor_plans) =
            (&palette.scenes, &palette.robots, &palette.fixtures, &palette.floor_plans);
        let (products, products_dir) = (&palette.products, &palette.products_dir);

        egui::CollapsingHeader::new("Scenes").default_open(true).show(ui, |ui| {
            if ui
                .add(egui::Button::image_and_text(re_ui::icons::ADD.as_image(), "New scene").small())
                .clicked()
            {
                self.new_scene();
            }
            if scenes.is_empty() {
                ui.weak("No saved scenes in scenes/.");
            }
            for name in scenes {
                if self.confirm_delete.as_ref() == Some(name) {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format!("Delete {name}?")).monospace());
                        if ui.small_button("Delete").clicked() {
                            self.delete_scene(name);
                            self.confirm_delete = None;
                        }
                        if ui.small_button("Cancel").clicked() {
                            self.confirm_delete = None;
                        }
                    });
                    continue;
                }
                ui.horizontal(|ui| {
                    if ui
                        .small_icon_button(&re_ui::icons::TRASH, "Delete scene")
                        .on_hover_text(format!("Delete this saved scene (its folder scenes/{name}/)"))
                        .clicked()
                    {
                        self.confirm_delete = Some(name.clone());
                    }
                    let selected = *name == self.scene_file_name;
                    if ui.add(egui::Button::selectable(selected, RichText::new(name).monospace())).clicked() {
                        self.load_scene(name);
                    }
                });
            }
        });
        let linked = state.linked_cell.is_some();
        ui.add_space(4.0);
        ui.label(RichText::new("Add to scene").small().strong())
            .on_hover_text("Products, dressed robots, fixtures and floor plans: + adds one to the open scene");
        ui.add_enabled_ui(!linked, |ui| {
            self.products_palette_ui(ui, products, products_dir);
            self.items_palette_ui(ui, robots, fixtures, floor_plans);
        });
    }

    /// The palette's robots, fixtures and floor plans, which add to the scene.
    fn items_palette_ui(&mut self, ui: &mut egui::Ui, robots: &[String], fixtures: &[String], floor_plans: &[String]) {
        egui::CollapsingHeader::new("Robots").default_open(true).show(ui, |ui| {
            if robots.is_empty() {
                ui.weak("No dressed robots — dress one in the Robo Dresser.");
            }
            for name in robots {
                ui.horizontal(|ui| {
                    if ui.small_icon_button(&re_ui::icons::ADD, "Add to scene").on_hover_text("Add to scene").clicked() {
                        self.add_robot(name);
                    }
                    ui.monospace(name);
                });
            }
        });
        egui::CollapsingHeader::new("Fixtures").default_open(true).show(ui, |ui| {
            if fixtures.is_empty() {
                ui.weak("No fixtures found.");
            }
            for name in fixtures {
                ui.horizontal(|ui| {
                    if ui.small_icon_button(&re_ui::icons::ADD, "Add to scene").on_hover_text("Add to scene").clicked() {
                        self.add_fixture(name);
                    }
                    ui.monospace(name);
                });
            }
        });
        // Floor plans: Advanced only (one already in the scene stays, and shows in Items).
        crate::ui_mode::advanced(ui, |ui| egui::CollapsingHeader::new("Floor plans").default_open(false).show(ui, |ui| {
            if floor_plans.is_empty() {
                ui.weak("No floor plans found.");
            }
            for name in floor_plans {
                // Relative to the scene's folder, scenes/<name>/.
                let geometry_file = format!("../../floor-plans/{name}");
                let current = self.scene.floor_plan.as_ref().is_some_and(|f| f.geometry_file == geometry_file);
                if ui.add(egui::Button::selectable(current, RichText::new(name).monospace())).clicked() {
                    // The file's own settings (its footprint layers, keys the viewer doesn't
                    // model) stay when the same file is picked again.
                    let same = self.scene.floor_plan.take().filter(|_| current).unwrap_or_default();
                    self.scene.floor_plan = Some(FloorPlanYaml {
                        geometry_file,
                        footprint_layers: same.footprint_layers,
                        extra: same.extra,
                        ..Default::default()
                    });
                    self.selection = Some(Selection::FloorPlan);
                }
            }
        }));
    }

    /// Product & Task's products: the parts the cell works on, placed like fixtures.
    fn products_palette_ui(&mut self, ui: &mut egui::Ui, products: &[String], products_dir: &str) {
        egui::CollapsingHeader::new("Products").default_open(true).show(ui, |ui| {
            if products_dir.is_empty() {
                // A backend from before products were placeable sends no products folder.
                ui.weak("This backend doesn't offer products yet: update it to a newer release.");
            } else if products.is_empty() {
                ui.weak("No products yet — define one in Product & Task.");
            }
            for name in products {
                ui.horizontal(|ui| {
                    if ui.small_icon_button(&re_ui::icons::ADD, "Add the part to the scene").on_hover_text("Add the part to the scene").clicked() {
                        self.add_product(name, products_dir);
                    }
                    ui.monospace(name);
                });
            }
        });
    }

    /// A product as a fixture. Its URDF is in the products folder, not the asset root, so the
    /// path is absolute; a cell made from the scene references it.
    fn add_product(&mut self, name: &str, products_dir: &str) {
        self.scene.fixtures.push(FixtureYaml {
            id: self.unique_id(name),
            urdf: format!("{}/{name}/{name}.urdf", products_dir.trim_end_matches('/')),
            position: Xyz { x: 0.0, y: 0.0, z: 0.0 },
            orientation: Abc::default(),
            ..Default::default()
        });
        self.selection = Some(Selection::Fixture(self.scene.fixtures.len() - 1));
    }

    fn unique_id(&self, base: &str) -> String {
        let taken = |id: &str| {
            self.scene.robots.iter().any(|r| r.robot_id == id) || self.scene.fixtures.iter().any(|f| f.id == id)
        };
        if !taken(base) {
            return base.to_owned();
        }
        (2..).map(|i| format!("{base}_{i}")).find(|id| !taken(id)).expect("unbounded")
    }

    fn add_robot(&mut self, name: &str) {
        let offset = self.scene.robots.len() as f64 * 3.0; // Keep new robots apart.
        self.scene.robots.push(RobotYaml {
            robot_id: self.unique_id(name),
            // Relative to the scene's folder, scenes/<name>/.
            dressed_urdf: format!("../../urdf-dressed-robots/{name}/{name}.urdf"),
            position: Xyz { x: offset, y: 0.0, z: 0.0 },
            orientation: Abc::default(),
            home_pose_deg: None,
            pedestal: None,
            ..Default::default()
        });
        self.selection = Some(Selection::Robot(self.scene.robots.len() - 1));
    }

    fn add_fixture(&mut self, name: &str) {
        let offset = self.scene.fixtures.len() as f64 * 2.0;
        self.scene.fixtures.push(FixtureYaml {
            id: self.unique_id(name),
            urdf: format!("../../urdf-fixture-collection/{name}/{name}.urdf"),
            position: Xyz { x: 0.0, y: 2.0 + offset, z: 0.0 },
            orientation: Abc::default(),
            ..Default::default()
        });
        self.selection = Some(Selection::Fixture(self.scene.fixtures.len() - 1));
    }

    /// Scene settings, items, the selected item's inspector, diagnostics, save and compose.
    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let state = self.state.lock().unwrap().clone();
        let (busy, status, report) = (state.busy.clone(), state.status.clone(), state.compose_report.clone());
        ui.horizontal_wrapped(|ui| {
            if busy.is_some() {
                ui.spinner();
            }
            ui.small(RichText::new(busy.unwrap_or(status)).weak());
        });

        self.cell_ui(ui, &state);
        self.view_ui(ui, &state);
        let linked = state.linked_cell.is_some();
        self.editor_ui(ui, &state, report.as_ref(), linked);
    }

    /// The pane's main actions, pinned to its bottom ([`theme::action_row`]): Save, Generate USDA
    /// and Use as cell (not while linked to a cell, whose scene is read-only here), with what is
    /// not saved yet above them.
    pub fn actions_ui(&mut self, ui: &mut egui::Ui) {
        let state = self.state.lock().unwrap().clone();
        let linked = state.linked_cell.is_some();
        if let Some(hint) = save_hint(&state.source_cells, self.has_unsaved_edits()) {
            ui.label(RichText::new(hint).small().color(theme::current().warn));
        }
        if self.unsaved_file().is_some() {
            ui.label(
                RichText::new("● Unsaved: press Save (or Use as cell) to keep it.")
                    .small()
                    .color(theme::current().warn),
            );
        }
        theme::action_buttons(ui, |ui| {
            if ui
                .add_enabled(!linked, theme::primary(ui, "Save"))
                .on_hover_text(
                    "Writes scenes/<name>/scene.yaml (named after the scene; renaming it renames the folder); the cells that use it reference it, so their plans and exports become stale",
                )
                .clicked()
            {
                self.save_scene();
            }
            let generate = if linked {
                "Compose scenes/<name>/<name>.zip for Isaac Sim from the cell's scene as saved (the cell stays linked)"
            } else {
                "Save, then compose scenes/<name>/<name>.zip for Isaac Sim"
            };
            if ui
                .add_enabled(!self.scene.robots.is_empty() || linked, egui::Button::new("Generate USDA"))
                .on_hover_text(generate)
                .clicked()
            {
                self.compose();
            }
            if !linked {
                let existing = existing_cell(&state.source_cells);
                let cell = existing.map_or_else(|| crate::cell_list::clean_name(&self.scene_file_name), |c| c.name.clone());
                if ui
                    .add_enabled(!self.scene.robots.is_empty(), egui::Button::new("Use as cell"))
                    .on_hover_text(format!(
                        "Save the scene and use it as cell {cell}: the cell references the scene and its catalogue robots and \
                         products (nothing is copied; its folder holds the plans, task programs and exports), becomes the \
                         active cell and is prepared for planning"
                    ))
                    .on_disabled_hover_text("Place a robot first")
                    .clicked()
                {
                    self.use_as_cell();
                }
            }
        });
    }

    /// The task (weld spots), the reach domes and the reachability check; also while linked.
    fn view_ui(&mut self, ui: &mut egui::Ui, state: &FitterState) {
        sub_heading(ui, "Task & reach");
        let linked = state.linked_cell.is_some();
        let mut view: FitterView = state.view.clone();
        let mut changed = false;
        ui.horizontal_wrapped(|ui| {
            changed |= ui
                .checkbox(&mut view.show_task, "Show task")
                .on_hover_text("The weld spots, approach arrows and paths in 3D, coloured by reachability")
                .changed();
            changed |= ui
                .add_enabled(view.show_task, egui::Checkbox::new(&mut view.show_labels, "Labels"))
                .on_hover_text("The weld spots' names (W02~: dome estimate; W02 r1: reached by r1 in the IK check)")
                .changed();
            changed |= ui
                .checkbox(&mut view.show_reach, "Show reach")
                .on_hover_text("Each robot's reach dome for the task's approach: blue = reachable, red = too close to the base")
                .changed();
        });
        // Where the task comes from: Advanced only (Auto unless picked there).
        crate::ui_mode::advanced(ui, |ui| grid_row(ui, "task", |ui| {
            egui::ComboBox::from_id_salt("fitter_task_source")
                .selected_text(task_source_label(view.task_source))
                .show_ui(ui, |ui| {
                    for source in [TaskSource::Auto, TaskSource::Product, TaskSource::CellProgram] {
                        changed |= ui.selectable_value(&mut view.task_source, source, task_source_label(source)).changed();
                    }
                })
                .response
                .on_hover_text("Auto: the cell's task program while linked, else the placed products' welds")
        }));
        if changed {
            // Shown at once; the backend's state confirms it.
            self.state.lock().unwrap().view = view.clone();
            self.post(api::FITTER_VIEW, &view);
        }

        let rows = weld_rows(state);
        ui.label(RichText::new(summary_line(&rows, &method_label(state))).small());
        for note in &state.task_notes {
            ui.label(RichText::new(format!("⚠ {note}")).small().color(theme::current().warn));
        }
        // The robots' TCPs: Advanced only.
        let tcps = crate::ui_mode::is_advanced(ui);
        for (robot, tcp) in state.tcp.iter().filter(|_| tcps) {
            let text = format!(
                "{robot}: TCP [{:.4}, {:.4}, {:.4}] on {} ({})",
                tcp.offset[0], tcp.offset[1], tcp.offset[2], tcp.tip_link, tcp.source
            );
            let color = if tcp.source == "estimated" { theme::current().warn } else { ui.visuals().weak_text_color() };
            ui.label(RichText::new(text).small().monospace().color(color));
        }

        self.check_ui(ui, state);

        egui::CollapsingHeader::new("Reachability").default_open(true).show(ui, |ui| {
            self.reachability_ui(ui, state, &rows, linked);
        });
    }

    /// "Check reachability" (or its progress and Cancel), and the report's notes.
    fn check_ui(&mut self, ui: &mut egui::Ui, state: &FitterState) {
        use std::ops::Not as _;
        let linked = state.linked_cell.is_some();
        let report = state.reach.as_ref();
        if let Some(report) = report.filter(|r| r.running) {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(RichText::new(format!("Checking {}/{}…", report.done, report.total)).small());
                if ui.small_button("Cancel").clicked() {
                    self.post(api::FITTER_REACH_CANCEL, &serde_json::json!({}));
                }
            });
        } else {
            let can_check = !self.scene.robots.is_empty() && !state.task.is_empty();
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(can_check, egui::Button::new("Check reachability"))
                    .on_hover_text("Solves the IK for every robot and weld (the scene as edited here), then colours the welds")
                    .on_disabled_hover_text("Needs robots and a task (place a product with weld spots)")
                    .clicked()
                {
                    let mode = if self.full_check { ReachMode::Full } else { ReachMode::Quick };
                    self.post(
                        api::FITTER_REACH_CHECK,
                        // While linked the scene is the cell's (read-only): the backend has it already.
                        &ReachCheckRequest { mode, robots: Vec::new(), welds: Vec::new(), edit: linked.not().then(|| self.edit()) },
                    );
                }
                crate::ui_mode::advanced(ui, |ui| ui.checkbox(&mut self.full_check, "Full (with approach)")
                    .on_hover_text("Also plans the approach to each weld (about 20 s per robot and weld instead of 1 s)"));
            });
        }
        let Some(report) = report else {
            return;
        };
        if let Some(error) = &report.error {
            ui.label(RichText::new(error).small().color(theme::current().danger));
        }
        for note in &report.notes {
            ui.label(RichText::new(note).small().weak());
        }
        if !report.running && report.elapsed_s > 0.0 {
            let stale = if is_stale(report, state.scene_rev) { ", stale: the scene changed since" } else { "" };
            ui.label(RichText::new(format!("Last check: {} welds in {:.1} s{stale}", report.welds.len(), report.elapsed_s)).small().weak());
        }
    }

    /// The welds, their reachability per robot, playback, suggestions and "Use as home pose".
    fn reachability_ui(&mut self, ui: &mut egui::Ui, state: &FitterState, rows: &[WeldRow], linked: bool) {
        if rows.is_empty() {
            ui.weak("No task: place a product with weld spots (Product & Task), or link a cell with a task program.");
            return;
        }
        let ik = state.reach.as_ref().is_some_and(|r| !is_stale(r, state.scene_rev) && r.method == "ik");
        let now = ui.input(|i| i.time);
        if let Some(weld) = self.play.tick(now, rows) {
            self.show_weld(Some(weld));
        }
        if self.play.playing {
            let wait = (self.play.last_step + PLAY_STEP_S - now).max(0.05);
            ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(wait));
        }

        ui.horizontal(|ui| {
            if ui.small_button("⏮ Prev").on_hover_text("Show the previous reachable weld").clicked() {
                self.play.playing = false;
                let weld = self.play.prev(rows);
                self.show_weld(weld);
            }
            let play_label = if self.play.playing { "⏸ Pause" } else { "▶ Play" };
            if ui
                .add_enabled(ik || self.play.playing, egui::Button::new(play_label).small())
                .on_hover_text("Moves the robots to each reachable weld in turn, every 1.5 s")
                .on_disabled_hover_text("Check reachability first: playback shows the IK's joints")
                .clicked()
            {
                if self.play.playing {
                    self.play.playing = false;
                } else {
                    self.play.play(now);
                }
            }
            if ui.small_button("Next ⏭").on_hover_text("Show the next reachable weld").clicked() {
                self.play.playing = false;
                let weld = self.play.next(rows);
                self.show_weld(weld);
            }
            if ui.small_button("Home").on_hover_text("Back to the home pose (or the cell's joints while linked)").clicked() {
                self.play = PlayState::default();
                self.show_weld(None);
            }
        });

        let shown = self.play.current.clone().or_else(|| state.reach_pose.clone());
        for row in rows {
            let selected = shown.as_deref() == Some(row.id.as_str());
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("●").color(status_color(row.status)));
                let mut text = row.label.clone();
                if row.estimate {
                    text.push('~');
                }
                if !row.reached_by.is_empty() {
                    text.push_str(&format!(" {}", row.reached_by.join(" ")));
                }
                if ui
                    .add(egui::Button::selectable(selected, RichText::new(text).monospace()))
                    .on_hover_text(format!("{}: show the robots at this weld", row.id))
                    .clicked()
                {
                    self.play.playing = false;
                    self.play.current = Some(row.id.clone());
                    self.show_weld(Some(row.id.clone()));
                }
            });
            ui.indent(("weld_robots", &row.id), |ui| {
                for cell in &row.robots {
                    let color = if cell.ok { theme::current().ok } else { theme::current().danger };
                    let label = ui.label(RichText::new(&cell.text).small().color(color));
                    if !cell.hover.is_empty() {
                        label.on_hover_text(&cell.hover);
                    }
                    if selected && cell.ok && !cell.joints.is_empty() {
                        let joints: Vec<String> = cell.joints.iter().map(|j| format!("{j:.1}")).collect();
                        ui.label(RichText::new(format!("[{}]°", joints.join(", "))).small().monospace().weak());
                        if ui
                            .add_enabled(!linked, egui::Button::new(format!("Use as {} home pose", cell.robot)).small())
                            .on_hover_text("Writes these joints into the robot's home pose (save the scene to keep it)")
                            .on_disabled_hover_text("The linked cell's scene is read-only: Stop simulation and edit first")
                            .clicked()
                        {
                            set_home_pose(&mut self.scene, &cell.robot, &cell.joints);
                        }
                    }
                }
            });
        }

        let Some(report) = state.reach.as_ref().filter(|r| !r.suggestions.is_empty()) else {
            return;
        };
        sub_heading(ui, "Suggestions");
        for (i, suggestion) in report.suggestions.iter().enumerate() {
            let key = (report.scene_rev, i);
            ui.horizontal_wrapped(|ui| {
                let applicable = suggestion.move_m.is_some() || suggestion.pedestal_height_mm.is_some();
                if applicable {
                    let done = self.applied.contains(&key);
                    if ui
                        .add_enabled(!linked && !done, egui::Button::new(if done { "Applied" } else { "Apply" }).small())
                        .on_hover_text("Changes the robot here; check reachability again to confirm")
                        .on_disabled_hover_text(if done {
                            "Already applied"
                        } else {
                            "The linked cell's scene is read-only: Stop simulation and edit first"
                        })
                        .clicked()
                        && apply_suggestion(&mut self.scene, suggestion)
                    {
                        self.applied.insert(key);
                    }
                }
                ui.label(RichText::new(&suggestion.text).small());
            });
        }
    }

    /// Show the robots at `weld` (`None`: home, or the cell's joints while linked).
    fn show_weld(&self, weld: Option<String>) {
        self.post(api::FITTER_REACH_POSE, &ReachPoseRequest { weld_id: weld });
    }

    /// "Use as cell", or the cell the scene is linked to.
    fn cell_ui(&mut self, ui: &mut egui::Ui, state: &FitterState) {
        sub_heading(ui, "Cell");
        if let Some(cell) = &state.linked_cell {
            ui.label(RichText::new(format!("🔒 Cell {cell} — simulated by step 1 (read-only)")).color(theme::current().ok));
            if let Some(scene) = linked_out_of_date(state, cell) {
                ui.label(RichText::new(legacy_out_of_date_text(cell, &scene)).small().color(theme::current().warn));
            }
            ui.small(
                RichText::new(
                    "The 3D view shows the simulation of the cell: plan in Agentic Planning, then Load & Play the \
                     behaviour tree in step 1 (sidebar).",
                )
                .weak(),
            );
            if ui
                .button("Stop simulation and edit")
                .on_hover_text(
                    "Stops the simulated cell-runner and opens the cell's scene (scenes/) for editing; saving it \
                     marks the cell's plans and exports stale.",
                )
                .clicked()
            {
                self.stop_and_edit();
            }
            return;
        }
        let existing = existing_cell(&state.source_cells);
        let cell = existing.map_or_else(|| crate::cell_list::clean_name(&self.scene_file_name), |c| c.name.clone());
        // "Use as cell" is in the pane's action row (`actions_ui`).
        let legacy = existing.is_some_and(|c| c.kind == CellKind::Legacy);
        let note = if legacy {
            format!("Cell {cell} (legacy: its own copy of the scene) · cells/{cell}/")
        } else {
            format!("Cell {cell} · its outputs go to cells/{cell}/")
        };
        ui.small(RichText::new(note).weak());
        if let Some(badge) = existing.and_then(|c| crate::cell_list::stale_badge(false, c.stale_reason.as_deref())) {
            ui.small(RichText::new(format!("{badge}: Prepare Cell again")).color(theme::current().warn));
        }
    }

    /// Scene settings, save/compose, items and the selected item's inspector (read-only while linked,
    /// but for Generate USDA: the linked cell's scene as saved).
    fn editor_ui(&mut self, ui: &mut egui::Ui, state: &FitterState, report: Option<&ComposeReport>, linked: bool) {
        sub_heading(ui, "Scene");
        ui.add_enabled_ui(!linked, |ui| {
            // The scene is the folder scenes/<name>/: Save names (or renames) it after the name.
            grid_row(ui, "name", |ui| ui.text_edit_singleline(&mut self.scene.name)).on_hover_text(
                "The scene's name, and its folder scenes/<name>/: Save renames the folder (a scene a cell uses is kept, \
                 and the renamed one saved beside it)",
            );
            // Where it is saved: Advanced only.
            crate::ui_mode::advanced(ui, |ui| {
                grid_row(ui, "folder", |ui| ui.monospace(format!("scenes/{}/", self.scene_file_name)))
                    .on_hover_text("The scene's folder (scene.yaml, its USDA zips); renaming the scene renames it on Save");
            });
            grid_row(ui, "description", |ui| ui.text_edit_singleline(&mut self.scene.description));
        });
        // Save and Generate USDA are in the pane's action row (`actions_ui`).
        // Generate USDA's options: Advanced only.
        crate::ui_mode::advanced(ui, |ui| ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.full_robots, "Full robot visuals")
                .on_hover_text("Off = collision hulls only (leaner zip)");
            ui.checkbox(&mut self.full_tools, "Full tool visuals");
            ui.checkbox(&mut self.urdf_bundle, "URDF bundle")
                .on_hover_text("Also write <name>-urdf.zip (in the scene's folder) with the source URDFs");
        }));
        match report {
            Some(report) if report.ok => {
                ui.label(RichText::new(format!("✓ {}", report.text)).small().color(theme::current().ok));
            }
            Some(report) => {
                ui.label(RichText::new(&report.text).small().color(theme::current().danger));
            }
            None => {}
        }

        ui.add_enabled_ui(!linked, |ui| {
            sub_heading(ui, "Items");
            if self.scene.robots.is_empty() && self.scene.fixtures.is_empty() {
                ui.weak("Add robots and fixtures from the palette.");
            }
            // Each item: select it, or remove it from the scene.
            let mut remove = None;
            let mut row = |ui: &mut egui::Ui, item: Selection, label: &str, selection: &mut Option<Selection>| {
                ui.horizontal(|ui| {
                    if ui
                        .small_icon_button(&re_ui::icons::TRASH, "Remove from the scene")
                        .on_hover_text("Remove from the scene")
                        .clicked()
                    {
                        remove = Some(item);
                    }
                    let selected = *selection == Some(item);
                    if ui.add(egui::Button::selectable(selected, RichText::new(label).monospace())).clicked() {
                        *selection = Some(item);
                    }
                });
            };
            for (i, robot) in self.scene.robots.iter().enumerate() {
                row(ui, Selection::Robot(i), &robot.robot_id, &mut self.selection);
            }
            for (i, fixture) in self.scene.fixtures.iter().enumerate() {
                row(ui, Selection::Fixture(i), &fixture.id, &mut self.selection);
            }
            if self.scene.floor_plan.is_some() {
                row(ui, Selection::FloorPlan, "floor plan", &mut self.selection);
            }
            if let Some(item) = remove {
                match item {
                    Selection::Robot(i) => {
                        self.scene.robots.remove(i);
                    }
                    Selection::Fixture(i) => {
                        self.scene.fixtures.remove(i);
                    }
                    Selection::FloorPlan => self.scene.floor_plan = None,
                }
                // Indices after the removed item move up; don't leave another item selected by accident.
                self.selection = None;
            }

            sub_heading(ui, "Inspector");
            match self.selection {
                Some(Selection::Robot(i)) if i < self.scene.robots.len() => self.robot_inspector(ui, i, state),
                Some(Selection::Fixture(i)) if i < self.scene.fixtures.len() => self.fixture_inspector(ui, i),
                Some(Selection::FloorPlan) if self.scene.floor_plan.is_some() => self.floor_plan_inspector(ui),
                _ => {
                    ui.weak("Select a robot or fixture to edit.");
                }
            }

            egui::CollapsingHeader::new("Diagnostics").default_open(true).show(ui, |ui| {
                if state.diagnostics.is_empty() {
                    ui.small(RichText::new("No issues.").color(theme::current().ok));
                }
                for diagnostic in &state.diagnostics {
                    ui.label(RichText::new(diagnostic).small().color(theme::current().danger));
                }
            });
        });
    }

    fn robot_inspector(&mut self, ui: &mut egui::Ui, i: usize, state: &FitterState) {
        let dof = state.robot_dofs.get(&self.scene.robots[i].robot_id).copied();
        let robot = &mut self.scene.robots[i];
        grid_row(ui, "id", |ui| ui.text_edit_singleline(&mut robot.robot_id));
        crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(&robot.dressed_urdf).weak().monospace()));
        pose_ui(ui, &mut robot.position, &mut robot.orientation);

        let mut has_pedestal = robot.pedestal.is_some();
        if ui.checkbox(&mut has_pedestal, "Pedestal").changed() {
            robot.pedestal = has_pedestal.then(|| PedestalYaml::new(500.0, 800.0));
        }
        if let Some(pedestal) = &mut robot.pedestal {
            grid_row(ui, "height", |ui| {
                ui.add(egui::DragValue::new(&mut pedestal.height_mm).speed(5.0).range(0.0..=5000.0).suffix(" mm"))
            });
            grid_row(ui, "diameter", |ui| {
                ui.add(egui::DragValue::new(&mut pedestal.diameter_mm).speed(5.0).range(10.0..=5000.0).suffix(" mm"))
            });
        }

        // The home pose's joints: Advanced only (the pose is still filled in as before).
        let advanced = crate::ui_mode::is_advanced(ui);
        if advanced {
            ui.small(RichText::new("HOME POSE (DEG)").weak());
        }
        match dof {
            Some(dof) => {
                let pose = robot.home_pose_deg.get_or_insert_with(|| default_home_pose(dof));
                pose.resize(dof, 0.0);
                for (j, angle) in pose.iter_mut().enumerate().filter(|_| advanced) {
                    grid_row(ui, &format!("A{}", j + 1), |ui| {
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                    });
                }
            }
            None => {
                ui.weak("Loading robot…");
            }
        }

        if ui.button("Remove robot").clicked() {
            self.scene.robots.remove(i);
            self.selection = None;
        }
    }

    fn fixture_inspector(&mut self, ui: &mut egui::Ui, i: usize) {
        let fixture = &mut self.scene.fixtures[i];
        grid_row(ui, "id", |ui| ui.text_edit_singleline(&mut fixture.id));
        crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(&fixture.urdf).weak().monospace()));
        pose_ui(ui, &mut fixture.position, &mut fixture.orientation);
        if ui.button("Remove fixture").clicked() {
            self.scene.fixtures.remove(i);
            self.selection = None;
        }
    }

    fn floor_plan_inspector(&mut self, ui: &mut egui::Ui) {
        let Some(plan) = &mut self.scene.floor_plan else {
            return;
        };
        crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(&plan.geometry_file).weak().monospace()));
        let offset = plan.offset.get_or_insert_with(Xyz::default);
        for (label, value) in [("offset x", &mut offset.x), ("offset y", &mut offset.y), ("offset z", &mut offset.z)] {
            grid_row(ui, label, |ui| ui.add(egui::DragValue::new(value).speed(0.01).suffix(" m")));
        }
        grid_row(ui, "scale", |ui| ui.add(egui::DragValue::new(&mut plan.scale).speed(0.001).range(0.0001..=1000.0)));
        if ui.button("Remove floor plan").clicked() {
            self.scene.floor_plan = None;
            self.selection = None;
        }
    }
}

/// The cell the backend reuses for the scene ("Use as cell"), if it has one: a reference cell
/// first, as `cell_resolve::cell_for_scene` picks it.
fn existing_cell(cells: &[SourceCell]) -> Option<&SourceCell> {
    cells.iter().find(|cell| cell.kind == CellKind::Reference).or_else(|| cells.first())
}

/// "cell a" / "cells a, b".
fn cell_names(cells: &[&SourceCell]) -> String {
    let names: Vec<&str> = cells.iter().map(|cell| cell.name.as_str()).collect();
    match names.as_slice() {
        [one] => format!("cell {one}"),
        names => format!("cells {}", names.join(", ")),
    }
}

/// The hint under Save while the scene has unsaved edits and cells use it: saving rebuilds
/// nothing; it marks the outputs of the cells that reference the scene stale, and legacy cells
/// keep their own copy.
fn save_hint(cells: &[SourceCell], unsaved: bool) -> Option<String> {
    if cells.is_empty() || !unsaved {
        return None;
    }
    let (reference, legacy): (Vec<&SourceCell>, Vec<&SourceCell>) =
        cells.iter().partition(|cell| cell.kind == CellKind::Reference);
    let mut parts = Vec::new();
    if !reference.is_empty() {
        parts.push(format!("Saving marks plans and exports of this scene stale ({})", cell_names(&reference)));
    }
    if !legacy.is_empty() {
        let (keeps, its) = if legacy.len() == 1 { ("keeps", "its") } else { ("keep", "their") };
        parts.push(format!("legacy {} {keeps} {its} own copy", cell_names(&legacy)));
    }
    let mut hint = parts.join("; ");
    if let Some(first) = hint.get(..1) {
        hint = first.to_uppercase() + &hint[1..];
    }
    Some(hint)
}

/// A legacy (self-contained, copied) cell that differs from the scene it was made from.
fn legacy_out_of_date_text(cell: &str, scene: &str) -> String {
    format!("⚠ Cell {cell} is a legacy copy and differs from its scene {scene}: it is simulated and planned with its own copy.")
}

/// The scene the linked cell is out of date with, if it is: its own scene open ("Open Cell"), or
/// the scene it was made from open ("Use as cell").
fn linked_out_of_date(state: &FitterState, cell: &str) -> Option<String> {
    state.cell_out_of_date_with.clone().or_else(|| {
        state
            .source_cells
            .iter()
            .any(|source| source.name == cell && source.out_of_date)
            .then(|| state.scene_file_name.clone())
    })
}

// ----------------------------------------------------------------------------
// Task & reach: pure helpers

/// A weld's colour: green = at least two robots (or the only one) reach it, orange = one, red =
/// none, grey = not known (no robots).
fn status_color(status: WeldStatus) -> Color32 {
    match status {
        WeldStatus::AllRobots => WELD_GREEN,
        WeldStatus::OneRobot => WELD_ORANGE,
        WeldStatus::NoRobot => WELD_RED,
        WeldStatus::Unknown => WELD_GREY,
    }
}

fn task_source_label(source: TaskSource) -> &'static str {
    match source {
        TaskSource::Auto => "Auto",
        TaskSource::Product => "Product",
        TaskSource::CellProgram => "Cell program",
    }
}

/// The report no longer matches the scene (or the task source) it was made for.
fn is_stale(report: &ReachReport, scene_rev: u64) -> bool {
    report.scene_rev != scene_rev
}

fn reason_text(reason: ReachReason) -> &'static str {
    match reason {
        ReachReason::NoSolution => "no IK solution",
        ReachReason::OutOfReach => "out of reach",
        ReachReason::InsideInnerVoid => "too close to the base",
        ReachReason::JointLimits => "joint limits",
        ReachReason::SelfCollision => "self collision",
        ReachReason::Floor => "below the floor",
        ReachReason::PoseTolerance => "pose not within tolerance",
        ReachReason::WeldInsideProduct => "weld is inside the product",
        ReachReason::ToolHitsProduct => "tool hits the product",
        ReachReason::BaseTiltUnsupported => "tilted base not supported",
        ReachReason::Error => "error",
    }
}

/// "r1: out of reach by 0.057 m" for a margin outside the dome (negative).
fn margin_text(robot: &str, margin_m: f64) -> String {
    if margin_m < 0.0 {
        format!("{robot}: out of reach by {:.3} m", -margin_m)
    } else {
        format!("{robot}: inside by {margin_m:.3} m")
    }
}

/// One robot's attempt on a weld: "r1 ✓", or why not.
fn attempt_text(attempt: &ReachAttempt) -> String {
    let robot = &attempt.robot_id;
    if attempt.reachable {
        return format!("{robot} ✓");
    }
    let out_of_reach = attempt.reasons.contains(&ReachReason::OutOfReach) && attempt.margin_m < 0.0;
    let reasons: Vec<&str> = attempt
        .reasons
        .iter()
        .filter(|r| !(out_of_reach && **r == ReachReason::OutOfReach))
        .map(|r| reason_text(*r))
        .collect();
    match (out_of_reach, reasons.is_empty()) {
        (true, true) => margin_text(robot, attempt.margin_m),
        (true, false) => format!("{}, {}", margin_text(robot, attempt.margin_m), reasons.join(", ")),
        (false, true) => format!("{robot}: {}", reason_text(ReachReason::NoSolution)),
        (false, false) => format!("{robot}: {}", reasons.join(", ")),
    }
}

/// A robot's cell in a weld's row.
#[derive(Clone, Debug, Default, PartialEq)]
struct RobotCell {
    robot: String,
    ok: bool,
    text: String,
    hover: String,
    joints: Vec<f64>,
}

/// A weld's row in "Reachability": from the IK report while it is current, else from the estimate.
#[derive(Clone, Debug, Default, PartialEq)]
struct WeldRow {
    id: String,
    label: String,
    status: WeldStatus,
    reached_by: Vec<String>,
    robots: Vec<RobotCell>,
    /// The colour is the reach dome's estimate, not a check (labels end in `~`).
    estimate: bool,
}

impl WeldRow {
    /// Playback skips welds no robot reaches (red).
    fn playable(&self) -> bool {
        !self.reached_by.is_empty() && self.status != WeldStatus::NoRobot
    }
}

fn weld_rows(state: &FitterState) -> Vec<WeldRow> {
    if let Some(report) = state.reach.as_ref().filter(|r| !is_stale(r, state.scene_rev) && !r.welds.is_empty()) {
        let estimate = report.method != "ik";
        return report
            .welds
            .iter()
            .map(|weld| WeldRow {
                id: weld.weld.id.clone(),
                label: if weld.weld.label.is_empty() { weld.weld.id.clone() } else { weld.weld.label.clone() },
                status: weld.status,
                reached_by: weld.reached_by.clone(),
                robots: weld
                    .attempts
                    .iter()
                    .map(|attempt| RobotCell {
                        robot: attempt.robot_id.clone(),
                        ok: attempt.reachable,
                        text: attempt_text(attempt),
                        hover: attempt_hover(attempt),
                        joints: attempt.joints_deg.clone(),
                    })
                    .collect(),
                estimate,
            })
            .collect();
    }
    state
        .task
        .iter()
        .map(|weld| {
            let estimate = state.estimate.iter().find(|e| e.weld_id == weld.id);
            let robots = estimate
                .map(|e| {
                    e.margins_m
                        .iter()
                        .map(|(robot, margin)| {
                            let ok = e.robots.contains(robot);
                            RobotCell {
                                robot: robot.clone(),
                                ok,
                                text: if ok { format!("{robot} ✓~ (inside by {margin:.3} m)") } else { margin_text(robot, *margin) },
                                hover: "Estimated from the reach dome (wrist position only); Check reachability solves the IK".to_owned(),
                                joints: Vec::new(),
                            }
                        })
                        .collect()
                })
                .unwrap_or_default();
            WeldRow {
                id: weld.id.clone(),
                label: if weld.label.is_empty() { weld.id.clone() } else { weld.label.clone() },
                status: estimate.map(|e| e.status).unwrap_or_default(),
                reached_by: estimate.map(|e| e.robots.clone()).unwrap_or_default(),
                robots,
                estimate: true,
            }
        })
        .collect()
}

fn attempt_hover(attempt: &ReachAttempt) -> String {
    let mut parts = Vec::new();
    if let Some(err) = attempt.pos_err_m {
        parts.push(format!("position error {:.1} mm", err * 1000.0));
    }
    if let Some(err) = attempt.axis_err_deg {
        parts.push(format!("tool axis error {err:.1}°"));
    }
    parts.push(format!("dome margin {:+.3} m", attempt.margin_m));
    if !attempt.detail.is_empty() {
        parts.push(attempt.detail.clone());
    }
    parts.join("; ")
}

/// How the colours were found: "IK", "estimate", or "estimate; IK stale".
fn method_label(state: &FitterState) -> String {
    match &state.reach {
        Some(report) if is_stale(report, state.scene_rev) => "estimate; IK stale, check again".to_owned(),
        Some(report) if report.method == "ik" => "IK".to_owned(),
        _ => "estimate".to_owned(),
    }
}

/// "4 welds: 0 green, 4 orange, 0 red (estimate)".
fn summary_line(rows: &[WeldRow], method: &str) -> String {
    let count = |status| rows.iter().filter(|r| r.status == status).count();
    let (green, orange, red, grey) =
        (count(WeldStatus::AllRobots), count(WeldStatus::OneRobot), count(WeldStatus::NoRobot), count(WeldStatus::Unknown));
    let welds = if rows.len() == 1 { "weld" } else { "welds" };
    let grey = if grey > 0 { format!(", {grey} unknown") } else { String::new() };
    format!("{} {welds}: {green} green, {orange} orange, {red} red{grey} ({method})", rows.len())
}

/// The weld shown in 3D, and auto play through the reachable welds.
#[derive(Clone, Debug, Default, PartialEq)]
struct PlayState {
    playing: bool,
    current: Option<String>,
    /// `ui.input(|i| i.time)` of the last step.
    last_step: f64,
}

impl PlayState {
    /// Start playing; the first weld shows on the next tick.
    fn play(&mut self, now: f64) {
        self.playing = true;
        self.last_step = now - PLAY_STEP_S;
    }

    fn next(&mut self, rows: &[WeldRow]) -> Option<String> {
        self.step(rows, true)
    }

    fn prev(&mut self, rows: &[WeldRow]) -> Option<String> {
        self.step(rows, false)
    }

    /// While playing, the next weld once [`PLAY_STEP_S`] passed since the last.
    fn tick(&mut self, now: f64, rows: &[WeldRow]) -> Option<String> {
        if !self.playing || now - self.last_step < PLAY_STEP_S {
            return None;
        }
        self.last_step = now;
        let next = self.next(rows);
        if next.is_none() {
            self.playing = false; // Nothing reachable to show.
        }
        next
    }

    /// The next (or previous) playable weld after the current one, wrapping around.
    fn step(&mut self, rows: &[WeldRow], forward: bool) -> Option<String> {
        let n = rows.len();
        let start = self.current.as_ref().and_then(|id| rows.iter().position(|r| &r.id == id));
        let index = (1..=n)
            .map(|k| match (start, forward) {
                (Some(s), true) => (s + k) % n,
                (Some(s), false) => (s + n - k % n) % n,
                (None, true) => k - 1,
                (None, false) => n - k,
            })
            .find(|&i| rows[i].playable())?;
        self.current = Some(rows[index].id.clone());
        self.current.clone()
    }
}

/// Apply a suggestion to the scene being edited: move the robot and/or set its pedestal height.
fn apply_suggestion(scene: &mut SceneFile, suggestion: &Suggestion) -> bool {
    let Some(robot) = suggestion
        .robot_id
        .as_ref()
        .and_then(|id| scene.robots.iter_mut().find(|r| &r.robot_id == id))
    else {
        return false;
    };
    let mut changed = false;
    if let Some([dx, dy, dz]) = suggestion.move_m {
        robot.position.x += dx;
        robot.position.y += dy;
        robot.position.z += dz;
        changed = true;
    }
    if let Some(height_mm) = suggestion.pedestal_height_mm {
        // Only the height changes: the pedestal's other keys stay.
        robot.pedestal.get_or_insert_with(|| PedestalYaml::new(height_mm, 800.0)).height_mm = height_mm;
        changed = true;
    }
    changed
}

/// "Use as home pose": the joints the check found for a weld become the robot's home pose.
fn set_home_pose(scene: &mut SceneFile, robot_id: &str, joints_deg: &[f64]) -> bool {
    match scene.robots.iter_mut().find(|r| r.robot_id == robot_id) {
        Some(robot) if !joints_deg.is_empty() => {
            robot.home_pose_deg = Some(joints_deg.to_vec());
            true
        }
        _ => false,
    }
}

fn sub_heading(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(title.to_uppercase()).small().weak());
}

fn grid_row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui) -> egui::Response) -> egui::Response {
    ui.horizontal(|ui| {
        ui.add_sized([72.0, 18.0], egui::Label::new(RichText::new(label).weak()));
        add(ui)
    })
    .inner
}

fn pose_ui(ui: &mut egui::Ui, position: &mut Xyz, orientation: &mut Abc) {
    for (label, value) in [("x", &mut position.x), ("y", &mut position.y), ("z", &mut position.z)] {
        grid_row(ui, label, |ui| ui.add(egui::DragValue::new(value).speed(0.01).max_decimals(3).suffix(" m")));
    }
    for (label, value) in [("A (z)", &mut orientation.a), ("B (y)", &mut orientation.b), ("C (x)", &mut orientation.c)] {
        grid_row(ui, label, |ui| ui.add(egui::DragValue::new(value).speed(0.5).range(-360.0..=360.0).suffix("°")));
    }
}

/// The KUKA stowed pose the composer falls back to, padded for other joint counts.
fn default_home_pose(dof: usize) -> Vec<f64> {
    let mut pose = vec![0.0, -90.0, 90.0, 0.0, 0.0, 0.0];
    pose.resize(dof, 0.0);
    pose
}

#[cfg(test)]
mod tests {
    use super::*;

    use factoryos_client::protocol::fitter::{TaskWeld, WeldEstimate, WeldReach};

    fn row(id: &str, status: WeldStatus, reached_by: &[&str]) -> WeldRow {
        WeldRow {
            id: id.to_owned(),
            label: id.to_owned(),
            status,
            reached_by: reached_by.iter().map(|r| (*r).to_owned()).collect(),
            ..Default::default()
        }
    }

    fn weld(id: &str) -> TaskWeld {
        TaskWeld { id: id.to_owned(), label: id.to_owned(), task: "weld".to_owned(), ..Default::default() }
    }

    #[test]
    fn a_restart_resumes_what_the_old_backend_showed() {
        let opened = |file: &str, saved: bool| FitterState {
            root: "/catalogue".into(),
            scene_file_name: file.into(),
            saved: saved.then(|| SceneEdit { file_name: file.into(), scene: SceneFile::default() }),
            ..Default::default()
        };
        assert_eq!(Resume::of(&opened("body_weld_3", true)), Some(Some(Resume::File("body_weld_3".into()))));
        assert_eq!(Resume::of(&opened("body_weld_3", false)), Some(None), "never saved: nothing to load");
        assert_eq!(Resume::of(&opened(NEW_SCENE_FILE, false)), Some(None), "a new scene");
        assert_eq!(Resume::of(&opened(NEW_SCENE_FILE, true)), Some(Some(Resume::File(NEW_SCENE_FILE.into()))), "a scene saved as `scene`");
        let cell = FitterState { linked_cell: Some("cell_a".into()), cell_scene: true, ..opened("scene.yaml", false) };
        assert_eq!(Resume::of(&cell), Some(Some(Resume::Cell("cell_a".into()))));
        // A restarted backend's state before anything opened it says nothing.
        assert_eq!(Resume::of(&FitterState::default()), None);
    }

    #[test]
    fn welds_are_coloured_by_status() {
        assert_eq!(status_color(WeldStatus::AllRobots), WELD_GREEN);
        assert_eq!(status_color(WeldStatus::OneRobot), WELD_ORANGE);
        assert_eq!(status_color(WeldStatus::NoRobot), WELD_RED);
        assert_eq!(status_color(WeldStatus::Unknown), WELD_GREY);
        assert_eq!(status_color(WeldStatus::default()), WELD_GREY, "no robots: grey");
    }

    #[test]
    fn playback_skips_red_welds_and_wraps_around() {
        let rows = [
            row("W01", WeldStatus::OneRobot, &["r2"]),
            row("W02", WeldStatus::NoRobot, &[]),
            row("W03", WeldStatus::AllRobots, &["r1", "r2"]),
        ];
        let mut play = PlayState::default();
        assert_eq!(play.next(&rows).as_deref(), Some("W01"), "the first reachable weld");
        assert_eq!(play.next(&rows).as_deref(), Some("W03"), "W02 is red");
        assert_eq!(play.next(&rows).as_deref(), Some("W01"), "wraps around");
        assert_eq!(play.prev(&rows).as_deref(), Some("W03"), "backwards wraps too");
        assert_eq!(play.prev(&rows).as_deref(), Some("W01"));
        let mut from_none = PlayState::default();
        assert_eq!(from_none.prev(&rows).as_deref(), Some("W03"), "prev from nothing: the last");
        // A weld that disappeared from the task: start over.
        let mut gone = PlayState { current: Some("W99".into()), ..Default::default() };
        assert_eq!(gone.next(&rows).as_deref(), Some("W01"));
        assert_eq!(PlayState::default().next(&[row("W02", WeldStatus::NoRobot, &[])]), None, "nothing to show");
        assert_eq!(PlayState::default().next(&[]), None);
    }

    #[test]
    fn playback_ticks_every_step() {
        let rows = [row("W01", WeldStatus::OneRobot, &["r1"]), row("W02", WeldStatus::OneRobot, &["r2"])];
        let mut play = PlayState::default();
        assert_eq!(play.tick(10.0, &rows), None, "paused");
        play.play(10.0);
        assert_eq!(play.tick(10.0, &rows).as_deref(), Some("W01"), "the first weld at once");
        assert_eq!(play.tick(11.0, &rows), None, "not yet");
        assert_eq!(play.tick(11.5, &rows).as_deref(), Some("W02"));
        assert_eq!(play.tick(13.0, &rows).as_deref(), Some("W01"));
        let mut red = PlayState::default();
        red.play(0.0);
        assert_eq!(red.tick(0.0, &[row("W02", WeldStatus::NoRobot, &[])]), None);
        assert!(!red.playing, "stops with nothing reachable");
    }

    #[test]
    fn a_report_for_another_scene_is_stale() {
        let report = ReachReport { scene_rev: 7, method: "ik".into(), ..Default::default() };
        assert!(!is_stale(&report, 7));
        assert!(is_stale(&report, 8));
        let mut state = FitterState { scene_rev: 7, reach: Some(report), ..Default::default() };
        assert_eq!(method_label(&state), "IK");
        state.scene_rev = 8;
        assert_eq!(method_label(&state), "estimate; IK stale, check again");
        state.reach = None;
        assert_eq!(method_label(&state), "estimate");
    }

    /// jam2: each robot reaches its own two top welds, the far ones are 57 mm out.
    fn jam2_state() -> FitterState {
        let ids = ["bug_cube/W01", "bug_cube/W02", "bug_cube/W03", "bug_cube/W04"];
        let estimate = |id: &str, r1: f64, r2: f64| WeldEstimate {
            weld_id: id.to_owned(),
            robots: [("r1", r1), ("r2", r2)].iter().filter(|(_, m)| *m >= 0.0).map(|(r, _)| (*r).to_owned()).collect(),
            margins_m: [("r1".to_owned(), r1), ("r2".to_owned(), r2)].into_iter().collect(),
            status: WeldStatus::OneRobot,
        };
        FitterState {
            scene_rev: 3,
            task: ids.iter().map(|id| weld(id)).collect(),
            estimate: vec![
                estimate(ids[0], -0.057, 0.12),
                estimate(ids[1], 0.12, -0.057),
                estimate(ids[2], -0.057, 0.12),
                estimate(ids[3], 0.12, -0.057),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn the_estimate_lists_each_weld_per_robot() {
        let state = jam2_state();
        let rows = weld_rows(&state);
        assert_eq!(rows.len(), 4);
        assert!(rows.iter().all(|r| r.estimate && r.status == WeldStatus::OneRobot));
        assert_eq!(rows[0].reached_by, ["r2"]);
        assert_eq!(rows[0].robots[0].text, "r1: out of reach by 0.057 m");
        assert!(rows[0].robots[1].ok && rows[0].robots[1].text.starts_with("r2 ✓~"));
        assert_eq!(summary_line(&rows, &method_label(&state)), "4 welds: 0 green, 4 orange, 0 red (estimate)");
    }

    #[test]
    fn a_current_ik_report_replaces_the_estimate() {
        let mut state = jam2_state();
        let attempt = |robot: &str, reachable: bool, reasons: Vec<ReachReason>, margin_m: f64| ReachAttempt {
            robot_id: robot.to_owned(),
            reachable,
            joints_deg: if reachable { vec![10.0, -40.0, 80.0, 0.0, 50.0, 0.0] } else { Vec::new() },
            margin_m,
            reasons,
            ..Default::default()
        };
        state.reach = Some(ReachReport {
            scene_rev: 3,
            method: "ik".into(),
            welds: vec![
                WeldReach {
                    weld: weld("bug_cube/W01"),
                    attempts: vec![attempt("r1", false, vec![ReachReason::OutOfReach], -0.057), attempt("r2", true, vec![], 0.12)],
                    reached_by: vec!["r2".into()],
                    status: WeldStatus::OneRobot,
                },
                WeldReach {
                    weld: weld("W07"),
                    attempts: vec![attempt("r1", false, vec![ReachReason::WeldInsideProduct], 0.3)],
                    reached_by: vec![],
                    status: WeldStatus::NoRobot,
                },
            ],
            ..Default::default()
        });
        let rows = weld_rows(&state);
        assert_eq!(rows.len(), 2);
        assert!(!rows[0].estimate);
        assert_eq!(rows[0].robots[0].text, "r1: out of reach by 0.057 m");
        assert_eq!(rows[0].robots[1].text, "r2 ✓");
        assert_eq!(rows[0].robots[1].joints.len(), 6);
        assert_eq!(rows[1].robots[0].text, "r1: weld is inside the product");
        assert_eq!(summary_line(&rows, &method_label(&state)), "2 welds: 0 green, 1 orange, 1 red (IK)");
        state.scene_rev = 4; // The robot was moved.
        assert_eq!(weld_rows(&state).len(), 4, "stale: back to the live estimate");
    }

    #[test]
    fn suggestions_move_the_robot_or_raise_it() {
        let mut scene = SceneFile {
            robots: vec![RobotYaml {
                robot_id: "r1".into(),
                dressed_urdf: "x.urdf".into(),
                position: Xyz { x: -2.5, y: 0.0, z: 0.0 },
                orientation: Abc::default(),
                home_pose_deg: None,
                pedestal: None,
                ..Default::default()
            }],
            ..Default::default()
        };
        let moved = Suggestion { robot_id: Some("r1".into()), text: "move".into(), move_m: Some([0.1, 0.0, 0.0]), pedestal_height_mm: None };
        assert!(apply_suggestion(&mut scene, &moved));
        assert!((scene.robots[0].position.x + 2.4).abs() < 1e-9);
        let raised = Suggestion { robot_id: Some("r1".into()), pedestal_height_mm: Some(400.0), ..Default::default() };
        assert!(apply_suggestion(&mut scene, &raised));
        assert_eq!(scene.robots[0].pedestal, Some(PedestalYaml::new(400.0, 800.0)));
        assert!(!apply_suggestion(&mut scene, &Suggestion { robot_id: Some("r9".into()), ..moved.clone() }));
        assert!(!apply_suggestion(&mut scene, &Suggestion { text: "note".into(), ..Default::default() }));

        assert!(set_home_pose(&mut scene, "r1", &[1.0, 2.0]));
        assert_eq!(scene.robots[0].home_pose_deg.as_deref(), Some(&[1.0, 2.0][..]));
        assert!(!set_home_pose(&mut scene, "r1", &[]));
    }


    fn cell(name: &str, plans: usize, out_of_date: bool) -> SourceCell {
        let kind = if out_of_date { CellKind::Legacy } else { CellKind::Reference };
        SourceCell { name: name.to_owned(), plans, out_of_date, kind, stale_reason: None }
    }

    #[test]
    fn saving_marks_the_scenes_outputs_stale() {
        let one = [cell("cell_new_scene", 1, false)];
        assert_eq!(save_hint(&[], true), None, "no cell uses the scene");
        assert_eq!(save_hint(&one, false), None, "nothing to save");
        assert_eq!(
            save_hint(&one, true).as_deref(),
            Some("Saving marks plans and exports of this scene stale (cell cell_new_scene)")
        );
        let two = [cell("a", 2, false), cell("c", 0, false)];
        assert_eq!(save_hint(&two, true).as_deref(), Some("Saving marks plans and exports of this scene stale (cells a, c)"));
        let mixed = [cell("a", 2, false), cell("b", 0, true)];
        assert_eq!(
            save_hint(&mixed, true).as_deref(),
            Some("Saving marks plans and exports of this scene stale (cell a); legacy cell b keeps its own copy")
        );
        assert_eq!(save_hint(&mixed[1..], true).as_deref(), Some("Legacy cell b keeps its own copy"));
    }

    #[test]
    fn use_as_cell_reuses_the_scenes_cell() {
        let name = |cells: &[SourceCell]| existing_cell(cells).map(|c| c.name.clone());
        assert_eq!(name(&[cell("cell_new_scene", 0, false)]).as_deref(), Some("cell_new_scene"), "reused");
        assert_eq!(name(&[cell("old", 0, true), cell("new", 0, false)]).as_deref(), Some("new"), "reference first");
        assert_eq!(name(&[cell("old", 0, true)]).as_deref(), Some("old"));
        assert_eq!(name(&[]), None);
        assert_eq!(
            legacy_out_of_date_text("cell_2robot", "two.yaml"),
            "⚠ Cell cell_2robot is a legacy copy and differs from its scene two.yaml: it is simulated and planned with its own copy."
        );
    }

    #[test]
    fn a_linked_cell_says_when_it_is_out_of_date() {
        let mut state = FitterState { scene_file_name: "new-scene.yaml".to_owned(), ..Default::default() };
        assert_eq!(linked_out_of_date(&state, "c"), None);
        state.source_cells = vec![cell("c", 0, true)];
        assert_eq!(linked_out_of_date(&state, "c").as_deref(), Some("new-scene.yaml"), "Use as cell's link");
        assert_eq!(linked_out_of_date(&state, "other"), None);
        state.source_cells.clear();
        state.scene_file_name = "scene.yaml".to_owned();
        state.cell_out_of_date_with = Some("new-scene.yaml".to_owned());
        assert_eq!(linked_out_of_date(&state, "c").as_deref(), Some("new-scene.yaml"), "the cell's own scene open");
    }
}
