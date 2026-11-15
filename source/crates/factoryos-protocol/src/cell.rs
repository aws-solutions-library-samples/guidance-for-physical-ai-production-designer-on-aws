//! Cell runner types. The JSON matches the cell-runner's `/ws` protocol
//! (`cell-runner/src/server.rs`), so commands and state are forwarded without translation.

use serde_json::Value;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackendState {
    #[default]
    Stopped,
    Starting,
    Ready,
    Failed,
}

/// The backend's view of the cell runner process tree (the simulation, on `factoryos/sim/status`;
/// the live cell's stand-ins publish one on `factoryos/cell/status`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BackendStatus {
    pub state: BackendState,
    /// Connected to the cell-runner's WebSocket.
    pub connected: bool,
    /// Unused since protocol v4 (no more RRD); always 0. Kept because producers still send it.
    pub blobs_published: u64,
    pub error: Option<String>,
    /// The cell (folder name) the cell-runner runs, once started.
    pub cell: Option<String>,
    /// Its scene file.
    pub scene: Option<String>,
    /// The cell-runner's port (`cellRunnerWsPort`, or a free one when that is taken).
    pub port: Option<u16>,
}

/// A cell shipped in the Kiro bundle's `release/` directory (`GET /api/v1/cell/samples`).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SampleCell {
    pub name: String,
    pub note: String,
    pub cell_dir: String,
    pub env_dir: String,
    pub scene: String,
}

/// `GET /api/v1/cell/cells`: the cells to pick from, and the fitted scenes to create cells from.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellsInfo {
    /// The folder the cells are listed from (configured, or the active cell's parent).
    pub cells_dir: String,
    pub cells: Vec<String>,
    /// The active cell's folder name, if it lives in `cells_dir`.
    pub active: String,
    pub fitted_robots_dir: String,
    /// Scene manifests in `fitted_robots_dir`.
    pub fitted_scenes: Vec<String>,
    /// The cell folders in `cells_dir` (as `cells`, same order), with what they are. A fitted scene
    /// no entry names in [`CellEntry::scene`] has no cell folder yet: the first Plan Cell, Load &
    /// Play, Prepare or "Use as cell" on it makes one (named after the scene). Empty from an older
    /// backend.
    pub entries: Vec<CellEntry>,
}

/// What a cell folder is (see [`CellEntry`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CellKind {
    /// Its outputs only (prepared planner model, task programs, plans, exports, logs) and a pointer
    /// to its Robo Fitter scene (`scenes/<scene>.yaml`); robots and products are read from
    /// the catalogue and the products folder (`factoryos-cell.json` format 2).
    Reference,
    /// A self-contained cell: its own `scene.yaml`, robots and fixtures (copied by an older Create
    /// Cell, or made elsewhere, e.g. umarell's reference cell).
    #[default]
    Legacy,
}

/// One cell folder in [`CellsInfo::entries`].
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct CellEntry {
    /// The folder name (also the cell's name).
    pub name: String,
    /// The Robo Fitter scene file (in `scenes/`) it references (a reference cell) or was
    /// copied from (a legacy cell), if any.
    pub scene: Option<String>,
    pub kind: CellKind,
    /// Its prepared planner model (`agentic_planner/cell.yaml` + `config/`) was made from inputs
    /// (scene, URDFs, products, USDA) that have changed since: Prepare Cell again.
    pub prepared_stale: bool,
    /// Why, e.g. "robot r1's URDF changed": for the badge.
    pub stale_reason: Option<String>,
}

/// `GET /api/v1/cell/plans`: the behaviour trees the agentic planner saved for the active cell.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlansInfo {
    /// The active cell's folder name.
    pub cell: String,
    /// Newest first.
    pub plans: Vec<PlanInfo>,
    /// The cell's task programs (the operations to plan), by file stem; empty if it has none yet.
    pub tasks: Vec<String>,
    /// Whether the cell has been prepared for planning (it has an `agentic_planner/` folder).
    pub prepared: bool,
    /// The Robo Fitter scene (`scenes/<file>`) a legacy (copied) cell was made from, when the
    /// cell no longer matches it. Never set for a reference cell (it reads the scene itself).
    pub out_of_date_with: Option<String>,
    /// What the active cell is (see [`CellKind`]).
    pub kind: CellKind,
    /// The Robo Fitter scene file the cell references (or was copied from), if any.
    pub scene: Option<String>,
    /// Why the prepared planner model is stale (its inputs changed since Prepare Cell), if it is.
    pub prepared_stale_reason: Option<String>,
    /// The Product & Task products placed in the cell (its scene.yaml fixtures with a `product.json`
    /// beside their URDF), by name, also those without point operations (welds) yet.
    pub products: Vec<String>,
    /// Their point operations (welds), 0 when none has any: what the backend derives into the
    /// cell's task programs (`<fixture>__<task>.json`, see `task_sources`).
    pub product_welds: u32,
    /// Where each task program in `tasks` comes from, in the same order: a Product & Task task
    /// (the backend derives `<fixture>__<task>.json` from the fixture's product), or hand-written.
    pub task_sources: Vec<TaskSource>,
}

/// The origin of one of the active cell's task programs (`agentic_planner/tasks/<file>.json`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TaskSource {
    /// The file stem in `agentic_planner/tasks/`.
    pub file: String,
    /// The Product & Task product it was derived from; `None` for a hand-written program.
    pub product: Option<String>,
    /// The product's task.
    pub task: Option<String>,
    /// The cell fixture (scene.yaml fixture id) the product is placed as.
    pub fixture: Option<String>,
    /// The product changed since the program was derived, and it was not re-derived (it was edited).
    pub stale: bool,
    /// The derived file was edited by hand after the backend wrote it: kept, not re-derived.
    pub edited: bool,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlanInfo {
    /// The plan file, relative to the cell folder (`agentic_planner/plans/bt_….json`).
    pub path: String,
    pub tree_id: String,
    /// Unix seconds.
    pub created_s: f64,
    pub num_operations: u64,
    /// The planner that made it: `umarell` (the LLM planner; also when the file does not say), or a
    /// factoryos-cell-planner planner the planner router ran (`hier_la_fu`, `hier_q`, `ppo_v3`, …).
    pub planner: String,
    /// The plan's `status`: `ok`, or e.g. `unverified` (an export check failed; plan2cell refuses it).
    pub status: String,
    /// Planned for other inputs than the cell has now: its recorded input hashes
    /// (`planner_meta.inputs`) no longer match the scene, URDFs or products, or (a legacy cell) it
    /// was planned before the cell was last rebuilt from its Robo Fitter scene. Kept, not deleted.
    pub stale: bool,
    /// Why it is stale (e.g. "the scene jam2.yaml changed"), when it is and the reason is known.
    pub stale_reason: Option<String>,
}

/// A task program of the active cell (the operations the planner plans), as text:
/// `GET /api/v1/cell/task?name=…`, `PUT /api/v1/cell/task`, and `GET /api/v1/cell/task/sample`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TaskProgram {
    /// The file stem in `agentic_planner/tasks/` (letters, digits, `_`, `-`).
    pub name: String,
    /// The JSON text.
    pub text: String,
}

/// `POST /api/v1/cell/plan/load`: load a plan into the simulated cell-runner's behaviour-tree
/// executor, starting (or restarting, for another cell) the cell-runner first if needed.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LoadPlan {
    /// As in [`PlanInfo::path`].
    pub path: String,
    /// Start playing right away.
    pub play: bool,
    /// The scene (cell folder name) to simulate it in; `None`: the active cell. Made the active
    /// cell and opened in the Robo Fitter if it is not.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell: Option<String>,
}

/// `POST /api/v1/cell/active`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SetActiveCell {
    pub cell: String,
}

/// `POST /api/v1/cell/create`: make (or reuse) the cell of a fitted scene and select it. Since
/// cells reference their scene, this only writes the cell's manifest; `name` is ignored when a cell
/// of the scene exists already (the cell's name is the scene's, see `cell_resolve`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CreateCell {
    /// A scene manifest in the scenes folder.
    pub scene: String,
    /// The new cell's folder name: letters, digits and underscores only (it becomes a ROS
    /// namespace). Empty: named after the scene.
    #[serde(default)]
    pub name: String,
}

/// A cell-runner verb: `start`, `stop`, `reset`, `set_speed`, `step`, `run`, `set_all_ready`,
/// `trajectory_*`, … Sent via `POST /api/v1/cell/command` or the `factoryos/cell/cmd` queryable.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CellCommand {
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params: Option<Value>,
}

impl CellCommand {
    pub fn new(kind: impl Into<String>) -> Self {
        Self {
            kind: kind.into(),
            params: None,
        }
    }

    pub fn with_params(kind: impl Into<String>, params: Value) -> Self {
        Self {
            kind: kind.into(),
            params: Some(params),
        }
    }
}

/// The cell-runner's `{ok, result, error}` response.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CellResponse {
    pub ok: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<Value>,
}

impl CellResponse {
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            ok: false,
            result: None,
            error: Some(Value::String(message.into())),
        }
    }

    /// The error as display text.
    pub fn error_text(&self) -> String {
        match &self.error {
            Some(Value::String(text)) => text.clone(),
            Some(other) => other.to_string(),
            None => "unknown error".to_owned(),
        }
    }
}

/// The cell-runner's `state` result.
///
/// Known fields are typed; anything else is kept in `extra` so newer cell-runners pass through.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct CellState {
    pub cell_name: String,
    pub cycle: u64,
    pub running: bool,
    pub speed: f64,
    pub complete: bool,
    pub robots: Vec<RobotState>,
    pub krl: Vec<KrlState>,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, Value>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotState {
    pub robot_id: String,
    pub name: String,
    pub model: String,
    pub role: String,
    pub phase: String,
    pub sequence: Value,
    pub is_home: bool,
    pub cycles_completed: u64,
    pub waiting_for: Value,
    pub inputs: Vec<Signal>,
    pub outputs: Vec<Signal>,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, Value>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct KrlState {
    /// Which robot this is (`robot1`, …); empty if the producer doesn't say.
    pub robot_id: String,
    pub pc: u64,
    pub total: u64,
    pub last_instruction: Value,
    pub position: Option<Pose>,
    pub joints: Vec<f64>,
    pub io: Option<KrlIo>,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, Value>,
}

/// KUKA `E6POS`.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub s: f64,
    pub t: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct KrlIo {
    pub inputs: Vec<IoChannel>,
    pub outputs: Vec<IoChannel>,
}

/// A KRL I/O channel. `value` is a bool for digital I/O, a number for groups.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct IoChannel {
    pub channel: Value,
    pub value: Value,
    pub name: Option<String>,
}

/// A robot's named PLC signal. `value` is a bool or a number.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Signal {
    pub signal: String,
    pub value: Value,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn command_matches_cell_runner_wire_format() {
        let cmd = CellCommand::with_params("set_speed", json!({ "speed": 2.0 }));
        assert_eq!(
            serde_json::to_value(&cmd).unwrap(),
            json!({ "type": "set_speed", "params": { "speed": 2.0 } })
        );
        let cmd = CellCommand::new("start");
        assert_eq!(serde_json::to_value(&cmd).unwrap(), json!({ "type": "start" }));
    }

    #[test]
    fn state_keeps_unknown_fields() {
        let raw = json!({
            "cell_name": "welding_cell_2_small",
            "cycle": 3,
            "running": true,
            "speed": 1.0,
            "robots": [{ "robot_id": "r1", "name": "R1", "cycles_completed": 2, "tcp": [1, 2] }],
            "krl": [{ "pc": 4, "total": 10, "position": { "x": 1.0 },
                      "io": { "outputs": [{ "channel": 1, "value": true, "name": "clamp" }] } }],
            "future_field": "kept",
        });
        let state: CellState = serde_json::from_value(raw.clone()).unwrap();
        assert_eq!(state.robots[0].cycles_completed, 2);
        assert_eq!(state.krl[0].io.as_ref().unwrap().outputs[0].name.as_deref(), Some("clamp"));
        assert_eq!(state.extra["future_field"], "kept");
        assert_eq!(state.robots[0].extra["tcp"], json!([1, 2]));
    }

    /// What a live cell-runner of a single-robot KRL cell answers (`set_all_ready`'s result; synthetic
    /// names and poses).
    #[test]
    fn parses_live_cell_runner_state() {
        let raw = include_str!("../tests/cell_runner_state.json");
        let state: CellState = serde_json::from_str(raw).unwrap();
        assert_eq!(state.cell_name, "cell");
        let robot = &state.robots[0];
        assert_eq!(robot.model, "kr240_r2900_ultra");
        assert_eq!(robot.inputs[3].signal, "SpiegelungFolge");
        assert_eq!(robot.inputs[3].value, json!(0));
        assert!(state.krl[0].last_instruction.is_null());
        assert_eq!(state.krl[0].position.unwrap().s, 2.0);
        assert_eq!(state.krl[0].joints.len(), 6);
        assert!(state.extra.contains_key("sync_map"));
        // Round-trips losslessly, so the backend can type it without dropping fields.
        let original: Value = serde_json::from_str(raw).unwrap();
        let reserialized = serde_json::to_value(&state).unwrap();
        for key in original.as_object().unwrap().keys() {
            assert!(reserialized.get(key).is_some(), "lost field {key}");
        }
    }

    #[test]
    fn older_payloads_still_parse() {
        let status: BackendStatus = serde_json::from_value(json!({ "state": "ready", "connected": true })).unwrap();
        assert_eq!(status.state, BackendState::Ready);
        assert_eq!((status.cell, status.port), (None, None));
        let plan: LoadPlan = serde_json::from_value(json!({ "path": "agentic_planner/plans/a.json", "play": true })).unwrap();
        assert_eq!(plan.cell, None);
    }

    #[test]
    fn response_error_text() {
        let response: CellResponse =
            serde_json::from_value(json!({ "id": 1, "ok": false, "error": "no such verb" })).unwrap();
        assert_eq!(response.error_text(), "no such verb");
    }
}
