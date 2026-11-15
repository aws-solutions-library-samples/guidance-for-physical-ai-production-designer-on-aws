//! The sample plans compile to cell-runner's timings, and run the same interactively.

mod common;

use std::collections::BTreeMap;

use bt_cell::plan::{merge_sibling, parse_plan_value};
use bt_cell::{HaltedMotion, ParsedPlan, Schedule, compile, parse_plan};
use bt_core::NodeStatus::Success;
use bt_exec::{RunOptions, Stop};
use serde_json::{Value, json};

use common::{
    WELDING_STEP_ENDS, assert_close, assert_near, btcpp_plan, by_name, finished_at, interactive,
    interactive_with, parse, schedule, segment, segment_of, seq_child, test_data, two_robot,
    welding, wp,
};

fn compiled(plan: &ParsedPlan) -> bt_cell::Compiled {
    compile(&plan.tree, &[], BTreeMap::new()).expect("plan compiles")
}

fn step_ends(ex: &bt_exec::Executor, steps: usize) -> Vec<f64> {
    (0..steps)
        .map(|i| finished_at(ex, seq_child(ex, i)))
        .collect()
}

fn assert_ends(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?}");
    for (a, e) in actual.iter().zip(expected) {
        assert_close(*a, *e);
    }
}

#[test]
fn welding_plan_matches_cell_runner() {
    let plan = welding();
    let c = compiled(&plan);
    let s = &c.schedule;
    assert_near(s.duration, 75.2969, 1e-3);
    assert_close(s.duration, 75.2969);
    assert_eq!(s.segments.len(), 18);
    assert_eq!(s.operation_count(), 15);
    assert_eq!(s.placed_count(), 16);
    assert_eq!(s.robots, ["robot1", "robot2"]);
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
    assert_eq!(c.executor.root_status(), Success);
    assert_ends(&step_ends(&c.executor, 11), &WELDING_STEP_ENDS);

    let starts = [
        (["W01", "W13"].as_slice(), 0.0),
        (&["W02", "W15"], 12.132),
        (&["W03", "W14"], 21.557),
        (&["W06", "W10"], 28.5075),
        (&["W05", "W11"], 34.4995),
        (&["W04", "W12"], 41.4118),
        (&["R1_PARK"], 47.2331),
        (&["W09"], 50.9044),
        (&["W08"], 54.6031),
        (&["W07"], 60.1971),
    ];
    for (ops, t0) in starts {
        for op in ops {
            assert_close(segment(s, op).t0, t0);
        }
    }
    let (h1, h2) = (
        segment_of(s, "home", "robot1"),
        segment_of(s, "home", "robot2"),
    );
    assert_close(h1.t0, 65.9664);
    assert_close(h2.t0, 65.9664);
    assert_close(h1.t_end(), 74.2969);
    assert_close(h2.t_end(), 72.1956);

    // Every segment knows the leaf that placed it.
    for seg in &s.segments {
        let node = c.executor.tree().node(seg.uid.expect("placed by a leaf"));
        if seg.op_id == "home" {
            assert_eq!(node.id, "MoveHome");
            assert_eq!(node.name, format!("home_{}", seg.robot));
        } else {
            assert_eq!(node.port_value("operation_id"), Some(seg.op_id.as_str()));
            assert!(node.name.ends_with(&format!("{}_{}", seg.robot, seg.op_id)));
        }
    }
    // Each robot is parked where its first motion starts.
    assert_eq!(
        s.pose_at("robot1", -1.0),
        common::start_of(&plan, "traj_robot1_W01")
    );
}

#[test]
fn two_robot_plan_matches_cell_runner() {
    let c = compiled(&two_robot());
    let s = &c.schedule;
    assert_ends(&step_ends(&c.executor, 2), &[4.7287, 8.1479]);
    assert_close(s.duration, 8.1479);
    let stage = segment(s, "STAGE_R1");
    assert_close(stage.t_end(), 3.2287);
    assert_close(stage.t_done(), 3.7287);
    let home = segment(s, "home");
    assert_close(home.t0, 4.7287);
    assert_close(home.t_end(), 7.1479);
    assert_eq!(s.segments.len(), 2);
    // cell-runner counts only non-`move` operations.
    assert_eq!(s.operation_count(), 0);
    assert_eq!(s.robots, ["robot1"]);
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
}

/// The plan without its `xml`, so the `root` mirror is converted instead.
fn mirror(plan_file: &str) -> ParsedPlan {
    let mut value: Value = serde_json::from_str(&test_data(plan_file)).unwrap();
    value["behaviour_tree"]
        .as_object_mut()
        .unwrap()
        .remove("xml");
    parse_plan_value(value).unwrap()
}

#[test]
fn root_mirror_compiles_like_the_xml() {
    for (file, steps) in [
        ("bt_1790934549_welding_cell_2_small.json", 11),
        ("bt_1791274820_cell_2robot.json", 2),
    ] {
        let from_xml = compiled(&parse_plan(&test_data(file)).unwrap());
        let plan = mirror(file);
        let from_root = compiled(&plan);
        assert_ends(
            &step_ends(&from_root.executor, steps),
            &step_ends(&from_xml.executor, steps),
        );
        assert_eq!(from_root.schedule.segments, from_xml.schedule.segments);
        assert!(from_root.schedule.warnings.is_empty());
        // The generated model declares every cell leaf, without a `name` port.
        let ex = interactive(&plan);
        assert!(ex.tree().nodes.iter().all(|n| !n.unknown_model), "{file}");
        assert!(
            ex.tree().nodes.iter().all(|n| n.port("name").is_none()),
            "{file}"
        );
    }
}

#[test]
fn interactive_defaults_match_compile() {
    for (plan, steps) in [(welding(), 11), (two_robot(), 2)] {
        let c = compiled(&plan);
        let mut ex = interactive(&plan);
        let report = ex.run(RunOptions::VIRTUAL);
        assert_eq!(report.stop, Stop::Finished(Success));
        assert_ends(&step_ends(&ex, steps), &step_ends(&c.executor, steps));
        assert_eq!(schedule(&ex).segments, c.schedule.segments);
        assert_close(schedule(&ex).duration, c.schedule.duration);
        assert!(bt_cell::is_cell_plan(&ex));
        assert_eq!(
            bt_cell::CellLeaves::finished(common::cell(&ex)),
            Some((Success, c.schedule.duration))
        );
    }
}

#[test]
fn orchestrator_sibling_compiles() {
    // The fixture next to the XML sample is cell-runner's `btcpp_plan()`.
    let fixture: Value = serde_json::from_str(&test_data("orchestrator_two_robots.json")).unwrap();
    assert_eq!(fixture, btcpp_plan());

    let xml = test_data("orchestrator_two_robots.xml");
    let mut plan = parse_plan(&xml).unwrap();
    // On its own the XML has no trajectories, so btgui keeps it on stubs.
    assert!(!bt_cell::is_cell_plan(&common::executor(&plan)));
    let sibling = test_data("orchestrator_two_robots.json");
    merge_sibling(&mut plan, Some(&sibling), Some("orchestrator_two_robots")).unwrap();
    assert_eq!(plan.tree_id.as_deref(), Some("bt_1"));
    assert_eq!(plan.num_operations, Some(2));
    assert!(bt_cell::is_cell_plan(&common::executor(&plan)));
    let c = compiled(&plan);
    assert_close(c.schedule.duration, 11.5);
    assert!(c.schedule.warnings.is_empty(), "{:?}", c.schedule.warnings);
}

/// A Parallel that needs one success, so the slower motion is halted by the tree, and a gate
/// that waits for where that motion was going.
fn kept_or_cut() -> ParsedPlan {
    let z = [0.0; 6];
    let q = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let body = r#"<Sequence>
        <Parallel success_count="1">
          <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
          <ExecuteTrajectory name="B" robot_ns="r2" operation_id="B" trajectory="{b}"/>
        </Parallel>
        <WaitForRobot name="wait" robot_ns="r1" depends_on_joints='[{"robot": "r2", "joint_positions": [1, 0, 0, 0, 0, 0], "tolerance_rad": 0.01}]'/>
      </Sequence>"#;
    parse(common::xml_plan(
        body,
        json!({ "a": [wp(z, 0.0), wp(z, 2.0)], "b": [wp(z, 0.0), wp(q, 10.0)] }),
    ))
}

#[test]
fn tree_halted_motion_is_kept_like_cell_runner() {
    let plan = kept_or_cut();
    // cell-runner's motion leaves have no halt: B plays on, and the gate opens when it
    // arrives.
    let c = compiled(&plan);
    assert_close(c.schedule.duration, 10.0);
    assert_close(segment(&c.schedule, "B").t_end(), 10.0);
    assert!(segment(&c.schedule, "B").cut.is_none());

    let mut ex = interactive(&plan);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 10.0);
    assert_eq!(schedule(&ex).segments, c.schedule.segments);

    // Cutting it instead leaves nothing to open the gate.
    let mut ex = interactive_with(&plan, |o| o.halted_motion = HaltedMotion::Cut);
    let wait = by_name(&ex, "wait");
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Stalled(vec![wait]));
    assert_close(ex.now(), 3.5);
    let b = segment(schedule(&ex), "B");
    assert_close(b.cut.expect("B is cut").at, 3.5);
    assert_close(b.t_end(), 3.5);
    assert_eq!(
        common::warnings(schedule(&ex), bt_cell::WarningKind::Cut).len(),
        1
    );
    let reason = ex.waiting_reason(wait).unwrap();
    assert!(
        reason.contains("no motion placed so far brings r2 within 0.01 rad"),
        "{reason}"
    );
}

#[test]
fn schedule_build_is_compile() {
    let plan = two_robot();
    let built = Schedule::build(&plan.tree, &[], BTreeMap::new()).unwrap();
    assert_eq!(built.segments, compiled(&plan).schedule.segments);
    assert!(
        built
            .describe()
            .starts_with("2 segments, 8.15s total, robots=robot1")
    );
}

#[test]
fn cell_plans_are_recognised_by_their_trajectories() {
    let plan = welding();
    let ex = common::executor(&plan);
    assert!(bt_cell::uses_cell_leaves(ex.tree()));
    assert!(bt_cell::is_cell_plan(&ex));
    let options = bt_cell::CellOptions::for_executor(&ex);
    assert_eq!(options.robots, ["robot1", "robot2"]);
    assert_eq!(
        options.preroll["robot2"],
        common::start_of(&plan, "traj_robot2_W13")
    );
    assert!(options.home.is_empty());

    // Cell leaves without trajectories (the old XML samples) stay on stubs.
    for file in ["orchestrator_two_robots.xml", "v4_cell_with_gates.xml"] {
        let ex = common::executor(&parse_plan(&test_data(file)).unwrap());
        assert!(bt_cell::uses_cell_leaves(ex.tree()), "{file}");
        assert!(!bt_cell::is_cell_plan(&ex), "{file}");
    }
    let ex = common::executor(&parse_plan(&test_data("Simple.xml")).unwrap());
    assert!(!bt_cell::uses_cell_leaves(ex.tree()));

    let et = r#"<Sequence><ExecuteTrajectory robot_ns="r1" trajectory="{t}"/></Sequence>"#;
    let with = |t: Value| common::executor(&parse(common::xml_plan(et, json!({ "t": t }))));
    assert!(!bt_cell::is_cell_plan(&with(json!([1, 2, 3]))));
    assert!(!bt_cell::is_cell_plan(&with(json!([]))));
    assert!(bt_cell::is_cell_plan(&with(json!([wp([0.0; 6], 0.0)]))));
    // An inline trajectory counts too.
    let inline = r#"<Sequence><ExecuteTrajectory robot_ns="r1" trajectory='[{"positions": [0], "time_s": 1}]'/></Sequence>"#;
    let ex = common::executor(&parse(common::xml_plan(inline, json!({}))));
    assert!(bt_cell::is_cell_plan(&ex));
}

#[test]
fn previews_read_the_ports_as_they_are() {
    use bt_cell::Preview;

    let mut ex = common::executor(&welding());
    let w01 = by_name(&ex, "weld_robot1_W01");
    let Some(Preview::Execute(Ok(op))) = bt_cell::preview(&ex, w01) else {
        panic!("W01 previews its motion");
    };
    assert_eq!((op.robot.as_str(), op.op_id.as_str()), ("robot1", "W01"));
    assert_eq!(op.traj.len(), 47);
    assert_close(op.hold_s, 0.5);
    ex.set_port_value(w01, "velocity_scale", "2").unwrap();
    let Some(Preview::Execute(Ok(op))) = bt_cell::preview(&ex, w01) else {
        panic!("W01 previews its motion");
    };
    assert_close(op.velocity_scale, 2.0);

    let home = by_name(&ex, "home_robot1");
    let Some(Preview::Home { robot, planned }) = bt_cell::preview(&ex, home) else {
        panic!("home previews its path");
    };
    assert_eq!(robot.as_deref(), Some("robot1"));
    assert_eq!(planned.unwrap().unwrap().len(), 42);
    // Control nodes have no preview.
    assert_eq!(bt_cell::preview(&ex, seq_child(&ex, 0)), None);

    let wait = r#"<Sequence><WaitForRobot name="handover" depends_on_joints='[{"robot": "r1", "joint_positions": [1]}, {"robot": "r2"}]'/></Sequence>"#;
    let ex = common::executor(&parse(common::xml_plan(wait, json!({}))));
    let Some(Preview::Wait { label, deps }) = bt_cell::preview(&ex, by_name(&ex, "handover"))
    else {
        panic!("the gate previews its dependencies");
    };
    assert_eq!(label, "handover");
    let deps = deps.unwrap();
    assert_eq!(deps.len(), 1, "a dependency without joints is dropped");
    assert_close(
        deps[0].tolerance_rad,
        bt_cell::joints::DEFAULT_INTERLOCK_TOL_RAD,
    );
}
