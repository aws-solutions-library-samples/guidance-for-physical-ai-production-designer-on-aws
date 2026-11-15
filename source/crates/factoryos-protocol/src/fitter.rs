//! Robo Fitter: the backend runs the `robo_fitting_room` library and describes the scene in 3D;
//! the viewer edits the scene. Actions are HTTP (`/api/v1/fitter/…`), edits are put on
//! `factoryos/fitter/scene`, state is published on `factoryos/fitter/state`, and the 3D scene
//! is on `factoryos/scene/fitter`.

/// The catalogue's scenes folder. Every Robo Fitter scene is a folder in it,
/// `<catalogue>/scenes/<name>/scene.yaml`, and is referred to by `<name>` (the folder's name; the
/// YAML's `name:` is kept equal to it on save). A scene's relative paths start at its folder
/// (`../../urdf-dressed-robots/…`). Where the protocol says "file name" of a scene (`file_name`,
/// `scene_file`, `write_as`, …) the value is that name; a legacy `<name>.yaml` is accepted too.
pub const SCENES_DIR: &str = "scenes";
/// Where scenes were before (`<catalogue>/fitted-robots/`): migrated into [`SCENES_DIR`] at the
/// backend's start; store keys under it are read as [`SCENES_DIR`]'s.
pub const LEGACY_SCENES_DIR: &str = "fitted-robots";


use std::collections::BTreeMap;

// The scene file, in the web editor's YAML format (the `robo_fitting_room` manifest).
//
// Every mapping of it (the scene, a robot, its pedestal and payload, a fixture, the floor plan)
// keeps the keys its struct doesn't model in `extra`, so loading and saving a scene never drops
// a key the composer (or a newer editor) reads. They are written back after the modelled keys,
// in key order; comments and the original key order are not kept. The coordinate leaves
// ([`Xyz`], [`Abc`], [`Inertia`]) are closed: just their numbers.

/// The keys of a manifest mapping that its struct doesn't model, as read, by key. A value holds
/// what JSON can: read with serde alone, a YAML tag or a non-string mapping key in it fails the
/// load (the backend's `scene_paths::parse_scene_file` turns them into their YAML text first, so
/// such a scene loads and keeps them), and `.inf` and `.nan` read as null.
pub type Extra = BTreeMap<String, serde_json::Value>;

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Xyz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Abc {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PedestalYaml {
    pub height_mm: f64,
    pub diameter_mm: f64,
    #[serde(flatten)]
    pub extra: Extra,
}

impl PedestalYaml {
    pub fn new(height_mm: f64, diameter_mm: f64) -> Self {
        Self { height_mm, diameter_mm, extra: Extra::new() }
    }
}

/// Principal moments of inertia, kg·m².
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Inertia {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
}

/// The tool a robot carries: a rigid payload of the link that carries the tool.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PayloadYaml {
    pub mass_kg: f64,
    /// Metres, in the tool link's frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center_of_mass_m: Option<Xyz>,
    /// About the centre of mass, in the tool link's axes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inertia_kgm2: Option<Inertia>,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RobotYaml {
    pub robot_id: String,
    pub dressed_urdf: String,
    pub position: Xyz,
    #[serde(default)]
    pub orientation: Abc,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub home_pose_deg: Option<Vec<f64>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pedestal: Option<PedestalYaml>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub payload: Option<PayloadYaml>,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FixtureYaml {
    pub id: String,
    pub urdf: String,
    pub position: Xyz,
    #[serde(default)]
    pub orientation: Abc,
    #[serde(flatten)]
    pub extra: Extra,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorPlanYaml {
    pub geometry_file: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset: Option<Xyz>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell_origin_dxf: Option<Xyz>,
    #[serde(default = "one")]
    pub scale: f64,
    /// The layers (or OBJ groups) drawn as footprints, by name; `None`: the composer's default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub footprint_layers: Option<Vec<String>>,
    #[serde(flatten)]
    pub extra: Extra,
}

impl Default for FloorPlanYaml {
    fn default() -> Self {
        Self {
            geometry_file: String::new(),
            offset: None,
            cell_origin_dxf: None,
            scale: one(),
            footprint_layers: None,
            extra: Extra::new(),
        }
    }
}

fn one() -> f64 {
    1.0
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SceneFile {
    /// Missing in upstream umarell cells' `scene.yaml`; "Open Cell" falls back to the cell name.
    #[serde(default)]
    pub name: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub description: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scene_origin: Option<Xyz>,
    #[serde(default)]
    pub robots: Vec<RobotYaml>,
    #[serde(default)]
    pub fixtures: Vec<FixtureYaml>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub floor_plan: Option<FloorPlanYaml>,
    #[serde(flatten)]
    pub extra: Extra,
}

impl Default for SceneFile {
    fn default() -> Self {
        Self {
            name: "new-scene".to_owned(),
            description: String::new(),
            scene_origin: None,
            robots: Vec::new(),
            fixtures: Vec::new(),
            floor_plan: None,
            extra: Extra::new(),
        }
    }
}

/// The name a new scene starts from (`scene`, then `scene-2`, … when taken).
pub const NEW_SCENE_FILE: &str = "scene";

/// What's available under the asset root.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Palette {
    /// The scenes' names (`scenes/<name>/scene.yaml`).
    pub scenes: Vec<String>,
    /// `urdf-dressed-robots/<name>/<name>.urdf`
    pub robots: Vec<String>,
    /// `urdf-fixture-collection/<name>/<name>.urdf`
    pub fixtures: Vec<String>,
    /// `floor-plans/*.obj`
    pub floor_plans: Vec<String>,
    /// Products of Product & Task, placeable like fixtures: `<products_dir>/<name>/<name>.urdf`,
    /// written by the backend next to each product's `product.json`.
    pub products: Vec<String>,
    /// The products folder on the backend host (absolute).
    pub products_dir: String,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ComposeReport {
    pub ok: bool,
    pub text: String,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct FitterState {
    /// The asset root (from the config's `assetRoot`).
    pub root: String,
    /// Increments whenever the backend replaces the scene (new/load), so editors know to adopt it.
    pub scene_seq: u64,
    /// The open scene's name (its folder in `scenes/`; saving names it after the scene's `name`).
    pub scene_file_name: String,
    pub scene: SceneFile,
    /// The scene as last saved to (or loaded from) `scenes/`; `None` if it never was.
    /// Editors compare it with their edit to show unsaved changes.
    pub saved: Option<SceneEdit>,
    /// The cell this scene was turned into ("Create Cell"). While set, the 3D view follows the
    /// running cell-runner's joints and the scene is not edited.
    pub linked_cell: Option<String>,
    /// The scene is the linked cell's own `scene.yaml` ("Open Cell"), not a `scenes/` file.
    pub cell_scene: bool,
    pub palette: Palette,
    pub diagnostics: Vec<String>,
    /// Actuated joint count of each loaded robot, by `robot_id`.
    pub robot_dofs: BTreeMap<String, usize>,
    pub busy: Option<String>,
    pub status: String,
    pub compose_report: Option<ComposeReport>,
    /// The cells made from this scene file (their `factoryos-cell.json` names it): saving the
    /// scene rebuilds them. Empty for a cell's own scene.
    pub source_cells: Vec<SourceCell>,
    /// For a cell's own scene ("Open Cell"): the `scenes/` scene it was made from, when the
    /// cell no longer matches it.
    pub cell_out_of_date_with: Option<String>,
    /// What the 3D view shows besides the scene: the task's welds, the robots' reach.
    pub view: FitterView,
    /// A hash of the scene and the resolved task source: a [`ReachReport`] with another
    /// `scene_rev` is stale.
    pub scene_rev: u64,
    /// The task being shown (the resolved [`FitterView::task_source`]), in the scene frame.
    pub task: Vec<TaskWeld>,
    /// Notes on the task, e.g. that the cell program doesn't match the product.
    pub task_notes: Vec<String>,
    /// The dome estimate of each weld in [`FitterState::task`].
    pub estimate: Vec<WeldEstimate>,
    /// The TCP each robot is checked with, by `robot_id`.
    pub tcp: BTreeMap<String, ToolTcp>,
    /// The last reachability check (running or done).
    pub reach: Option<ReachReport>,
    /// The weld the robots are shown at ([`ReachPoseRequest`]); `None`: home / the cell's joints.
    pub reach_pose: Option<String>,
}

/// A cell made from the open scene file (see [`FitterState::source_cells`]).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SourceCell {
    /// The cell's folder name.
    pub name: String,
    /// Its behaviour trees.
    pub plans: usize,
    /// A legacy (copied) cell that differs from the saved scene file. Never set for a reference
    /// cell: saving only marks its outputs stale (see `stale_reason`).
    pub out_of_date: bool,
    /// Whether it references the scene or is a legacy copy of it.
    pub kind: crate::cell::CellKind,
    /// Why its prepared planner model is stale (its inputs changed), if it is.
    pub stale_reason: Option<String>,
}

/// Put on `factoryos/fitter/scene` (and the body of save/compose): the scene as edited.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SceneEdit {
    /// The scene's name (a legacy `<name>.yaml` is accepted).
    pub file_name: String,
    pub scene: SceneFile,
}

/// `POST /api/v1/fitter/load`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LoadRequest {
    /// A scene name (a legacy `<name>.yaml` is accepted).
    pub file_name: String,
}

/// `POST /api/v1/fitter/create_cell` ("Use as cell"): save the scene, make sure it has a cell (a
/// folder for its outputs that references the scene; nothing is copied), make that the active
/// cell, link the 3D view to it and prepare it for planning.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CreateCellRequest {
    pub edit: SceneEdit,
    /// The cell's folder name for a scene that has none yet (letters, digits, `_`). Empty (or
    /// omitted): named after the scene. Ignored when the scene has a cell already.
    #[serde(default)]
    pub name: String,
}

/// `POST /api/v1/fitter/open_cell`: make an existing cell the active cell and show its scene,
/// linked to it.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct OpenCellRequest {
    /// A folder in the cells folder.
    pub cell: String,
}

/// `POST /api/v1/fitter/compose`: save the scene, then compose its USDA zip.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ComposeRequest {
    pub edit: SceneEdit,
    pub full_robots: bool,
    pub full_tools: bool,
    pub urdf_bundle: bool,
}

// Task, reach dome and reachability check.

/// Where the welds shown in the Robo Fitter come from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskSource {
    /// The cell program when the scene is linked to a cell that has one, else the products.
    #[default]
    Auto,
    /// The operations of every fixture that is a product (its `product.json`), moved with it.
    Product,
    /// The linked cell's `agentic_planner/tasks/*.json`.
    CellProgram,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReachMode {
    /// One IK solve at the weld pose (~1 s per robot and weld).
    #[default]
    Quick,
    /// With the approach: the tool axis solved over a stand-off (~20 s per robot and weld).
    Full,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReachReason {
    #[default]
    NoSolution,
    OutOfReach,
    InsideInnerVoid,
    JointLimits,
    SelfCollision,
    Floor,
    PoseTolerance,
    WeldInsideProduct,
    ToolHitsProduct,
    BaseTiltUnsupported,
    Error,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WeldStatus {
    /// No robots in the scene (grey).
    #[default]
    Unknown,
    /// Reached by at least two robots, or by the only one (green).
    AllRobots,
    /// Reached by exactly one robot of several (orange).
    OneRobot,
    /// Reached by none (red).
    NoRobot,
}

/// `POST /api/v1/fitter/view`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct FitterView {
    pub show_task: bool,
    /// The weld spots' text labels (`W02~`, `W02 r1`); off leaves just the coloured spots.
    pub show_labels: bool,
    pub show_reach: bool,
    pub task_source: TaskSource,
}

impl Default for FitterView {
    fn default() -> Self {
        Self { show_task: true, show_labels: true, show_reach: false, task_source: TaskSource::Auto }
    }
}

/// A robot's tool centre point, in its tip link.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ToolTcp {
    pub tip_link: String,
    /// Metres, in the tip link's frame.
    pub offset: [f64; 3],
    /// The tool axis (unit, tip link frame): the direction the tool points at the work.
    pub axis: [f64; 3],
    /// `table:<tool>` (the backend's tool table) or `estimated` (from the tool's geometry).
    pub source: String,
}

/// One weld (point, or a sample of a path) of the task, in the scene frame (the scene YAML's
/// coordinates; `scene_origin` not subtracted).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TaskWeld {
    /// Product: `<fixture_id>/<op id>`; cell program: `<op id>`.
    pub id: String,
    pub label: String,
    pub fixture_id: Option<String>,
    pub task: String,
    pub position: [f64; 3],
    /// Unit, from the tool towards the weld.
    pub approach: [f64; 3],
}

/// The dome estimate of one weld.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct WeldEstimate {
    pub weld_id: String,
    /// The robots whose dome contains the weld.
    pub robots: Vec<String>,
    /// Signed margin by robot, metres: positive inside the dome.
    pub margins_m: BTreeMap<String, f64>,
    pub status: WeldStatus,
}

/// One robot on one weld.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ReachAttempt {
    pub robot_id: String,
    pub reachable: bool,
    pub joints_deg: Vec<f64>,
    pub pos_err_m: Option<f64>,
    pub axis_err_deg: Option<f64>,
    /// The dome margin (metres, positive inside).
    pub margin_m: f64,
    pub reasons: Vec<ReachReason>,
    pub detail: String,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct WeldReach {
    pub weld: TaskWeld,
    pub attempts: Vec<ReachAttempt>,
    pub reached_by: Vec<String>,
    pub status: WeldStatus,
}

/// A layout change that brings welds into reach (checked against the dome only).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Suggestion {
    pub robot_id: Option<String>,
    pub text: String,
    /// Move the robot by this much (metres, scene frame).
    pub move_m: Option<[f64; 3]>,
    /// Put the robot on a pedestal this high (mm, total).
    pub pedestal_height_mm: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ReachReport {
    /// [`FitterState::scene_rev`] of the scene this was checked on.
    pub scene_rev: u64,
    /// The resolved task source (never `Auto`).
    pub source: TaskSource,
    pub mode: ReachMode,
    /// `ik` or `estimate` (the IK wasn't available).
    pub method: String,
    pub running: bool,
    pub done: usize,
    pub total: usize,
    pub welds: Vec<WeldReach>,
    pub suggestions: Vec<Suggestion>,
    pub notes: Vec<String>,
    pub error: Option<String>,
    pub elapsed_s: f64,
}

/// `POST /api/v1/fitter/reach_check`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ReachCheckRequest {
    pub mode: ReachMode,
    /// Only these robots (empty: all).
    pub robots: Vec<String>,
    /// Only these welds (empty: all).
    pub welds: Vec<String>,
    /// The viewer's unsaved edit, applied first.
    pub edit: Option<SceneEdit>,
}

/// `POST /api/v1/fitter/reach_pose`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ReachPoseRequest {
    /// `None`: home / follow the cell.
    pub weld_id: Option<String>,
}

/// `POST /api/v1/reach/check` (stateless, for the agent). One of `scene_file` (a scene name, a
/// legacy `<name>.yaml`, or an absolute path to a scene YAML), `cell` (its scene) or `scene`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ReachFileRequest {
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    pub scene: Option<SceneFile>,
    pub mode: ReachMode,
    pub source: TaskSource,
    /// Also load the scene into the Robo Fitter and show the report there.
    pub show_in_fitter: bool,
}

/// `POST /api/v1/reach/suggest`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SuggestRequest {
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    /// The farthest any robot is moved, metres.
    pub max_move_m: f64,
    /// Write the moved scene as this new scene (a name; `<name>.yaml` is accepted); never
    /// overwrites.
    pub write_as: Option<String>,
}

impl Default for SuggestRequest {
    fn default() -> Self {
        Self { scene_file: None, cell: None, max_move_m: 1.0, write_as: None }
    }
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SuggestResponse {
    pub scene: SceneFile,
    pub moves: Vec<Suggestion>,
    /// The dome estimate of the moved scene.
    pub report: ReachReport,
    /// The scene written (`write_as`, its name), if any.
    pub written: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip<T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + std::fmt::Debug>(v: T) {
        let json = serde_json::to_string(&v).unwrap();
        assert_eq!(serde_json::from_str::<T>(&json).unwrap(), v, "{json}");
    }

    #[test]
    fn an_old_fitter_state_still_reads() {
        let old = r#"{"root":"/a","scene_seq":3,"scene_file_name":"jam2.yaml","scene":{"name":"jam2","robots":[],"fixtures":[]},
            "saved":null,"linked_cell":"jam2","cell_scene":false,"palette":{},"diagnostics":[],"robot_dofs":{},"busy":null,
            "status":"","compose_report":null,"source_cells":[],"cell_out_of_date_with":null}"#;
        let state: FitterState = serde_json::from_str(old).unwrap();
        assert_eq!(state.linked_cell.as_deref(), Some("jam2"));
        assert!(state.view.show_task && !state.view.show_reach);
        assert!(state.task.is_empty() && state.reach.is_none() && state.reach_pose.is_none());
        let view: FitterView = serde_json::from_str("{}").unwrap();
        assert!(view.show_task);
        let suggest: SuggestRequest = serde_json::from_str(r#"{"scene_file":"a.yaml"}"#).unwrap();
        assert_eq!(suggest.max_move_m, 1.0);
        let req: ReachFileRequest = serde_json::from_str(r#"{"scene_file":"jam2.yaml","source":"cell_program","mode":"full"}"#).unwrap();
        assert_eq!((req.source, req.mode), (TaskSource::CellProgram, ReachMode::Full));
    }

    #[test]
    fn the_new_types_round_trip() {
        let weld = TaskWeld {
            id: "bug_cube/W01".into(),
            label: "W01".into(),
            fixture_id: Some("bug_cube".into()),
            task: "spot".into(),
            position: [0.1, 0.2, 2.0],
            approach: [0.0, 0.0, -1.0],
        };
        let attempt = ReachAttempt {
            robot_id: "r1".into(),
            reachable: false,
            joints_deg: vec![1.0, 2.0],
            pos_err_m: Some(0.01),
            axis_err_deg: None,
            margin_m: -0.057,
            reasons: vec![ReachReason::OutOfReach, ReachReason::WeldInsideProduct],
            detail: "far".into(),
        };
        let report = ReachReport {
            scene_rev: 7,
            source: TaskSource::Product,
            mode: ReachMode::Full,
            method: "ik".into(),
            running: true,
            done: 1,
            total: 8,
            welds: vec![WeldReach { weld: weld.clone(), attempts: vec![attempt.clone()], reached_by: vec![], status: WeldStatus::NoRobot }],
            suggestions: vec![Suggestion { robot_id: Some("r1".into()), text: "move".into(), move_m: Some([0.1, 0.0, 0.0]), pedestal_height_mm: Some(300.0) }],
            notes: vec!["n".into()],
            error: None,
            elapsed_s: 1.5,
        };
        round_trip(weld.clone());
        round_trip(attempt);
        round_trip(report.clone());
        round_trip(FitterView { show_task: false, show_labels: false, show_reach: true, task_source: TaskSource::CellProgram });
        round_trip(ToolTcp { tip_link: "l6".into(), offset: [-0.9, 0.0, 0.0], axis: [-1.0, 0.0, 0.0], source: "estimated".into() });
        round_trip(WeldEstimate { weld_id: "w".into(), robots: vec!["r1".into()], margins_m: [("r1".to_owned(), 0.1)].into(), status: WeldStatus::OneRobot });
        round_trip(ReachCheckRequest { mode: ReachMode::Full, robots: vec!["r1".into()], welds: vec![], edit: Some(SceneEdit { file_name: "a.yaml".into(), scene: SceneFile::default() }) });
        round_trip(ReachPoseRequest { weld_id: Some("w".into()) });
        round_trip(ReachFileRequest { scene_file: Some("a.yaml".into()), cell: None, scene: Some(SceneFile::default()), mode: ReachMode::Quick, source: TaskSource::Auto, show_in_fitter: true });
        round_trip(SuggestRequest { write_as: Some("a-fit.yaml".into()), ..SuggestRequest::default() });
        round_trip(SuggestResponse { scene: SceneFile::default(), moves: vec![], report: report.clone(), written: Some("a-fit.yaml".into()) });
        let state = FitterState { task: vec![weld], reach: Some(report), reach_pose: Some("w".into()), scene_rev: 9, ..FitterState::default() };
        round_trip(state);
        assert_eq!(serde_json::to_string(&ReachReason::WeldInsideProduct).unwrap(), "\"weld_inside_product\"");
        assert_eq!(serde_json::to_string(&WeldStatus::AllRobots).unwrap(), "\"all_robots\"");
    }

    /// A manifest with keys this crate models only in part (a payload, a pedestal, footprint
    /// layers) and keys it doesn't know, at every level.
    const MANIFEST: &str = r#"
name: two arms
description: a synthetic cell
scene_origin: {x: 10.0, y: -4.0, z: 0.0}
units: metric
revision: 7
robots:
  - robot_id: arm_a
    dressed_urdf: ../urdf-dressed-robots/arm/arm.urdf
    position: {x: 12.5, y: -3.25, z: 0.0}
    orientation: {a: 90.0, b: 0.0, c: 0.0}
    home_pose_deg: [0.0, -90.0, 90.0, 0.0, 12.345678, 0.0]
    pedestal: {height_mm: 600.0, diameter_mm: 900.0, material: concrete}
    payload:
      mass_kg: 42.5
      center_of_mass_m: {x: 0.1, y: 0.0, z: 0.25}
      inertia_kgm2: {ixx: 1.5, iyy: 2.0, izz: 2.5}
      source: datasheet
    controller: {address: 10.0.0.7, cycle_ms: 4}
    tags: [welding, left]
  - robot_id: arm_b
    dressed_urdf: ../urdf-dressed-robots/arm/arm.urdf
    position: {x: 14.0, y: -3.25, z: 0.0}
    orientation: {a: -90.0, b: 0.0, c: 0.0}
    payload: {mass_kg: 8.0}
fixtures:
  - id: table
    urdf: ../urdf-fixture-collection/table/table.urdf
    position: {x: 13.0, y: -1.0, z: 0.0}
    orientation: {a: 0.0, b: 0.0, c: 0.0}
    static: true
    clamps: [{name: c1, closed: false}, {name: c2, closed: null}]
floor_plan:
  geometry_file: ../floor-plans/plan.obj
  offset: {x: -10.0, y: 4.0, z: 0.0}
  scale: 0.001
  footprint_layers: [FOOTPRINTS, base_plates]
  line_width_m: 0.02
"#;

    /// `text` as YAML, every number as a float (a key's value, not how it was written).
    fn yaml_value(text: &str) -> serde_yaml::Value {
        fn floats(v: serde_yaml::Value) -> serde_yaml::Value {
            use serde_yaml::Value;
            match v {
                Value::Number(n) => Value::Number(n.as_f64().expect("a number").into()),
                Value::Sequence(items) => Value::Sequence(items.into_iter().map(floats).collect()),
                Value::Mapping(map) => Value::Mapping(map.into_iter().map(|(k, v)| (k, floats(v))).collect()),
                other => other,
            }
        }
        floats(serde_yaml::from_str(text).unwrap())
    }

    #[test]
    fn a_manifest_keeps_every_key_through_load_and_save() {
        let scene: SceneFile = serde_yaml::from_str(MANIFEST).unwrap();
        let (a, b) = (&scene.robots[0], &scene.robots[1]);
        let payload = a.payload.as_ref().unwrap();
        assert_eq!(payload.mass_kg, 42.5);
        assert_eq!(payload.center_of_mass_m, Some(Xyz { x: 0.1, y: 0.0, z: 0.25 }));
        assert_eq!(payload.inertia_kgm2, Some(Inertia { ixx: 1.5, iyy: 2.0, izz: 2.5 }));
        assert_eq!(payload.extra["source"], "datasheet");
        assert_eq!(b.payload, Some(PayloadYaml { mass_kg: 8.0, ..Default::default() }));
        let pedestal = a.pedestal.as_ref().unwrap();
        assert_eq!((pedestal.height_mm, pedestal.diameter_mm), (600.0, 900.0));
        assert_eq!(pedestal.extra["material"], "concrete");
        assert_eq!(a.home_pose_deg.as_deref(), Some(&[0.0, -90.0, 90.0, 0.0, 12.345678, 0.0][..]));
        assert_eq!(a.extra["controller"], serde_json::json!({"address": "10.0.0.7", "cycle_ms": 4}));
        assert_eq!(a.extra["tags"], serde_json::json!(["welding", "left"]));
        assert_eq!(scene.fixtures[0].extra["clamps"][1], serde_json::json!({"name": "c2", "closed": null}));
        let plan = scene.floor_plan.as_ref().unwrap();
        assert_eq!(plan.footprint_layers.as_deref(), Some(&["FOOTPRINTS".to_owned(), "base_plates".to_owned()][..]));
        assert_eq!(plan.extra["line_width_m"], 0.02);
        assert_eq!((scene.extra["units"].as_str(), scene.extra["revision"].as_u64()), (Some("metric"), Some(7)));

        // Saved, it says the same as the manifest, and saving again changes nothing.
        let saved = serde_yaml::to_string(&scene).unwrap();
        assert_eq!(yaml_value(&saved), yaml_value(MANIFEST), "{saved}");
        let again: SceneFile = serde_yaml::from_str(&saved).unwrap();
        assert_eq!(again, scene);
        assert_eq!(serde_yaml::to_string(&again).unwrap(), saved);
    }

    #[test]
    fn unknown_keys_are_written_after_the_known_ones_in_key_order() {
        let save = |text: &str| serde_yaml::to_string(&serde_yaml::from_str::<SceneFile>(text).unwrap()).unwrap();
        let one = "name: s\nzeta: 1\nrobots:\n- robot_id: r\n  dressed_urdf: r.urdf\n  position: {x: 0.0, y: 0.0, z: 0.0}\n  b: 2\n  a: [1]\nalpha: 2\n";
        let other = "alpha: 2\nrobots:\n- a: [1]\n  position: {z: 0.0, y: 0.0, x: 0.0}\n  b: 2\n  dressed_urdf: r.urdf\n  robot_id: r\nzeta: 1\nname: s\n";
        assert_eq!(save(one), save(other));
        assert_eq!(
            save(one),
            "name: s\nrobots:\n- robot_id: r\n  dressed_urdf: r.urdf\n  position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    a: 0.0\n    b: 0.0\n    c: 0.0\n  a:\n  - 1\n  b: 2\nfixtures: []\nalpha: 2\nzeta: 1\n"
        );
    }

    /// The wire format of a scene without the new keys is what it was: the viewer and backend of
    /// another release read each other's scenes.
    #[test]
    fn a_scene_without_new_keys_is_on_the_wire_as_before() {
        let old = r#"{"name":"jam2","scene_origin":{"x":1.0,"y":2.0,"z":0.0},"robots":[{"robot_id":"r1","dressed_urdf":"a.urdf","position":{"x":0.0,"y":0.0,"z":0.0},"orientation":{"a":0.0,"b":0.0,"c":0.0},"home_pose_deg":[1.0,2.0],"pedestal":{"height_mm":500.0,"diameter_mm":800.0}}],"fixtures":[{"id":"f","urdf":"f.urdf","position":{"x":1.0,"y":0.0,"z":0.0},"orientation":{"a":0.0,"b":0.0,"c":0.0}}],"floor_plan":{"geometry_file":"p.obj","scale":1.0}}"#;
        let scene: SceneFile = serde_json::from_str(old).unwrap();
        assert!(scene.extra.is_empty() && scene.robots[0].extra.is_empty() && scene.fixtures[0].extra.is_empty());
        assert_eq!(scene.robots[0].pedestal, Some(PedestalYaml::new(500.0, 800.0)));
        assert_eq!(scene.robots[0].payload, None);
        assert_eq!(scene.floor_plan, Some(FloorPlanYaml { geometry_file: "p.obj".into(), ..Default::default() }));
        assert_eq!(serde_json::to_string(&scene).unwrap(), old);
    }

    #[test]
    fn unknown_keys_go_on_the_wire_where_they_were() {
        let scene: SceneFile = serde_yaml::from_str(MANIFEST).unwrap();
        let json = serde_json::to_value(&scene).unwrap();
        assert_eq!(json["units"], "metric");
        assert_eq!(json["robots"][0]["controller"]["cycle_ms"], 4);
        assert_eq!(json["robots"][0]["pedestal"]["material"], "concrete");
        assert_eq!(json["robots"][0]["payload"]["source"], "datasheet");
        assert_eq!(json["robots"][0]["payload"]["inertia_kgm2"]["iyy"], 2.0);
        assert_eq!(json["fixtures"][0]["static"], true);
        assert_eq!(json["floor_plan"]["footprint_layers"][1], "base_plates");
        assert_eq!(json["floor_plan"]["line_width_m"], 0.02);
        assert!(!json.to_string().contains("\"extra\""), "{json}");
        round_trip(scene.clone());
        round_trip(SceneEdit { file_name: "two-arms.yaml".into(), scene: scene.clone() });
        round_trip(FitterState { scene: scene.clone(), saved: Some(SceneEdit { file_name: "two-arms.yaml".into(), scene }), ..Default::default() });
    }

    /// What a kept key can hold: what JSON can. A YAML tag, or a mapping key that isn't a string
    /// inside a kept value, fails the load (instead of being dropped); a kept key that isn't a
    /// string becomes one; `.inf` and `.nan` read as null.
    #[test]
    fn kept_keys_hold_what_json_can() {
        let scene = |extra: &str| serde_yaml::from_str::<SceneFile>(&format!("name: s\n{extra}"));
        assert_eq!(scene("limit: .inf\n").unwrap().extra["limit"], serde_json::Value::Null);
        assert_eq!(scene("1: x\n").unwrap().extra["1"], "x");
        assert!(scene("note: !mark x\n").is_err());
        assert!(scene("table: {1: x}\n").is_err());
    }
}
