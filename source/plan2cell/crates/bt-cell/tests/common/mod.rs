//! Helpers shared by the bt-cell tests: cell-runner's own test fixtures
//! (`trajectory.rs:1023-1104`), compiling and running plans, and finding nodes and segments.
#![allow(dead_code)]

use std::collections::BTreeMap;

use bt_cell::plan::parse_plan_value;
use bt_cell::{
    CellLeaves, CellOptions, CellWarning, Compiled, ParsedPlan, Schedule, Segment, WarningKind,
    compile, parse_plan,
};
use bt_core::{Document, NodeStatus, Uid};
use bt_exec::{Executor, Override};
use serde_json::{Value, json};

/// Unless a test says otherwise.
pub const TOL: f64 = 1e-6;

/// When each step of the welding plan ends: the top Sequence's 11 children (six Parallels,
/// R1_PARK, W09, W08, W07, the homes), from cell-runner's rules and the sample's trajectories.
pub const WELDING_STEP_ENDS: [f64; 11] = [
    12.132, 21.557, 28.5075, 34.4995, 41.4118, 47.2331, 50.9044, 54.6031, 60.1971, 65.9664, 75.2969,
];

pub fn wp(q: [f64; 6], t: f64) -> Value {
    json!({ "positions": q, "time_s": t })
}

/// Two sequential welds on one robot, in the older upper-case dialect.
pub fn sample_plan() -> Value {
    let a = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0];
    let b = [1.0, -1.0, 1.0, 0.0, 0.0, 0.0];
    json!({
        "tree_id": "t1",
        "cell_id": "c1",
        "behaviour_tree": {
            "tree_id": "t1",
            "root": {
                "type": "SEQUENCE",
                "children": [
                    { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "W01",
                      "operation_type": "weld", "hold_duration_s": 0.5,
                      "precomputed_trajectory": [wp(a, 0.0), wp(b, 2.0)] },
                    { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "W02",
                      "operation_type": "weld", "hold_duration_s": 0.0,
                      "precomputed_trajectory": [wp(b, 0.0), wp(a, 1.0)] }
                ]
            }
        }
    })
}

/// Two robots in a BT.CPP `Parallel`, trajectories on the blackboard.
pub const BTCPP_XML: &str = r#"<?xml version="1.0" ?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Parallel success_count="-1" failure_count="1">
        <ExecuteTrajectory name="weld_r1_A" robot_ns="r1" operation_id="A" operation_type="weld" trajectory="{traj_A}" hold_duration_s="0.5" velocity_scale="1.0" has_interlock="false"/>
        <ExecuteTrajectory name="move_r2_B" robot_ns="r2" operation_id="B" operation_type="move" trajectory="{traj_B}" hold_duration_s="0.5" velocity_scale="2.0" has_interlock="false"/>
      </Parallel>
      <Parallel success_count="-1" failure_count="1">
        <MoveHome name="home_r1" robot_ns="r1" trajectory="{traj_H}"/>
        <MoveHome name="home_r2" robot_ns="r2"/>
      </Parallel>
    </Sequence>
  </BehaviorTree>
  <TreeNodesModel>
    <Action ID="ExecuteTrajectory">
      <input_port name="robot_ns"/>
    </Action>
  </TreeNodesModel>
</root>"#;

pub fn btcpp_plan() -> Value {
    let z = [0.0; 6];
    let q = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    json!({
        "tree_id": "bt_1",
        "cell_id": "c1",
        "num_operations": 2,
        "behaviour_tree": {
            "BTCPP_format": "4",
            "xml": BTCPP_XML,
            "blackboard": {
                "traj_A": [wp(z, 0.0), wp(q, 2.0)],
                "traj_B": [wp(z, 0.0), wp(q, 4.0)],
                "traj_H": [wp(q, 0.0), wp(z, 1.0)]
            }
        }
    })
}

/// `body` as the `MainTree` of a v4 file, with `blackboard`.
pub fn xml_plan(body: &str, blackboard: Value) -> Value {
    let xml = format!(
        r#"<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">{body}</BehaviorTree></root>"#
    );
    json!({ "behaviour_tree": { "xml": xml, "blackboard": blackboard } })
}

pub fn parse(plan: Value) -> ParsedPlan {
    parse_plan_value(plan).expect("plan parses")
}

/// cell-runner's `build`: compiled for robots `["r1"]`, no homes.
pub fn build(plan: Value) -> Compiled {
    compile(&parse(plan).tree, &["r1".to_string()], BTreeMap::new()).expect("plan compiles")
}

/// cell-runner's `build_bare`: no robots, no homes.
pub fn build_bare(plan: Value) -> Compiled {
    compile(&parse(plan).tree, &[], BTreeMap::new()).expect("plan compiles")
}

pub fn test_data(name: &str) -> String {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
    std::fs::read_to_string(format!("{path}{name}")).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// A plan from `test_data/`.
pub fn sample(name: &str) -> ParsedPlan {
    parse_plan(&test_data(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

pub fn welding() -> ParsedPlan {
    sample("bt_1790934549_welding_cell_2_small.json")
}

pub fn two_robot() -> ParsedPlan {
    sample("bt_1791274820_cell_2robot.json")
}

/// The plan's main tree in an executor, without the cell leaves.
pub fn executor(plan: &ParsedPlan) -> Executor {
    let doc = Document::parse(&plan.tree.xml).expect("XML parses");
    let tree = doc
        .instantiate(doc.main_tree_id())
        .expect("tree instantiates");
    Executor::new(tree, plan.tree.initial_blackboard())
}

/// The plan as btgui runs it: the cell leaves installed with [`CellOptions::for_executor`].
pub fn interactive(plan: &ParsedPlan) -> Executor {
    interactive_with(plan, |_| {})
}

pub fn interactive_with(plan: &ParsedPlan, options: impl FnOnce(&mut CellOptions)) -> Executor {
    let mut ex = executor(plan);
    let mut o = CellOptions::for_executor(&ex);
    options(&mut o);
    bt_cell::install(&mut ex, o);
    ex
}

pub fn cell(ex: &Executor) -> &CellLeaves {
    ex.leaf_handler::<CellLeaves>()
        .expect("the cell leaves are installed")
}

pub fn schedule(ex: &Executor) -> &Schedule {
    cell(ex).schedule()
}

/// When `uid` last finished (its latest transition to SUCCESS or FAILURE).
pub fn finished_at(ex: &Executor, uid: Uid) -> f64 {
    ex.trace()
        .of_node(uid)
        .filter(|t| t.status.is_completed())
        .last()
        .unwrap_or_else(|| panic!("node {uid} never finished"))
        .t
}

/// The `i`-th child of the top-level Sequence.
pub fn seq_child(ex: &Executor, i: usize) -> Uid {
    let top = ex.tree().nodes[0].children[0];
    ex.tree().nodes[top].children[i]
}

pub fn by_name(ex: &Executor, name: &str) -> Uid {
    ex.tree()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named {name}"))
        .uid
}

/// The only segment of operation `op` (on `robot`, if given).
pub fn segment<'a>(schedule: &'a Schedule, op: &str) -> &'a Segment {
    let found: Vec<&Segment> = schedule.segments.iter().filter(|s| s.op_id == op).collect();
    assert_eq!(found.len(), 1, "segments of {op}: {found:#?}");
    found[0]
}

pub fn segment_of<'a>(schedule: &'a Schedule, op: &str, robot: &str) -> &'a Segment {
    schedule
        .segments
        .iter()
        .find(|s| s.op_id == op && s.robot == robot)
        .unwrap_or_else(|| panic!("{op}/{robot} scheduled"))
}

pub fn warnings(schedule: &Schedule, kind: WarningKind) -> Vec<&CellWarning> {
    schedule
        .warnings
        .iter()
        .filter(|w| w.kind == kind)
        .collect()
}

pub fn messages(schedule: &Schedule) -> Vec<&str> {
    schedule
        .warnings
        .iter()
        .map(|w| w.message.as_str())
        .collect()
}

pub fn force(ex: &mut Executor, uid: Uid, status: NodeStatus) {
    ex.set_override(
        uid,
        Some(Override {
            status,
            pinned: false,
        }),
    );
}

pub fn assert_close(actual: f64, expected: f64) {
    assert_near(actual, expected, TOL);
}

pub fn assert_near(actual: f64, expected: f64, tol: f64) {
    assert!(
        (actual - expected).abs() < tol,
        "{actual} is not {expected} (± {tol})"
    );
}

pub fn assert_pose(actual: [f64; 6], expected: [f64; 6]) {
    let err = bt_cell::joints::max_joint_error(&actual, &expected);
    assert!(err < TOL, "{actual:?} is not {expected:?}");
}

/// The first waypoint of a blackboard trajectory.
pub fn start_of(plan: &ParsedPlan, key: &str) -> [f64; 6] {
    let first = &plan.tree.blackboard[key][0]["positions"];
    let q: Vec<f64> = serde_json::from_value(first.clone()).unwrap();
    bt_cell::joints::to_six(&q)
}

/// A blackboard trajectory as segment waypoints.
pub fn waypoints(plan: &ParsedPlan, key: &str) -> Vec<bt_cell::joints::Waypoint> {
    let traj: Vec<bt_cell::PlanWaypoint> =
        serde_json::from_value(plan.tree.blackboard[key].clone()).unwrap();
    traj.iter()
        .map(|w| bt_cell::joints::Waypoint {
            q: bt_cell::joints::to_six(&w.positions),
            t: w.time_s,
        })
        .collect()
}
