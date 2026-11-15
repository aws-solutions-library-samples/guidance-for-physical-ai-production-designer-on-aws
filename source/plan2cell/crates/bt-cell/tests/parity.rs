//! cell-runner's own trajectory tests (`trajectory.rs:1106-1398`), ported: compiling through
//! bt-exec must place the same motion, with the same warnings.

mod common;

use bt_cell::joints::{COMPLETION_MARGIN_S, HOME_RETURN_TIME_S, angular_diff, max_joint_error};
use bt_cell::plan::parse_plan_value;
use bt_cell::{Schedule, parse_plan};
use serde_json::json;

use common::{
    BTCPP_XML, assert_close, btcpp_plan, build, build_bare, parse, sample_plan, segment,
    segment_of, wp,
};

#[test]
fn parses_every_file_layout() {
    assert!(parse_plan_value(sample_plan()).is_ok());
    assert!(parse_plan_value(btcpp_plan()).is_ok());
    let bare = json!({ "tree_id": "t", "root": { "type": "SEQUENCE", "children": [] } });
    assert!(parse_plan_value(bare).is_ok());
    let neither = json!({ "tree_id": "t" });
    assert!(parse_plan_value(neither).is_err());
    assert!(parse_plan(BTCPP_XML).is_ok());
}

#[test]
fn legacy_dialect_converts_to_btcpp_xml() {
    let parsed = parse(sample_plan());
    assert!(parsed.tree.xml.contains("<Sequence>"));
    assert!(parsed.tree.xml.contains("<ExecuteTrajectory"));
    assert!(parsed.tree.xml.contains("trajectory=\"{traj_1}\""));
    assert!(parsed.tree.blackboard.contains_key("traj_1"));
    assert!(parsed.tree.blackboard.contains_key("traj_2"));
}

#[test]
fn inline_json_ports_in_xml_are_decoded() {
    let start = [0.0; 6];
    let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree"><Sequence>
            <WaitForRobot robot_ns="r2" depends_on_joints="[{&quot;robot&quot;:&quot;r1&quot;,&quot;joint_positions&quot;:[0.0,-1.57,2.2,0.0,-0.8,0.0]}]"/>
            <ExecuteTrajectory robot_ns="r2" operation_id="B" trajectory="{t}"/>
        </Sequence></BehaviorTree></root>"#;
    let plan = json!({ "behaviour_tree": { "xml": xml, "blackboard": { "t": [wp(start, 0.0), wp(start, 1.0)] } } });
    let schedule = build_bare(plan).schedule;
    assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
    // r1 never moves, so it sits at the fallback home the gate waits for.
    assert_close(segment(&schedule, "B").t0, 0.0);
}

#[test]
fn btcpp_parallel_runs_children_from_a_common_start() {
    let schedule = build_bare(btcpp_plan()).schedule;
    assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
    let (a, b) = (
        segment_of(&schedule, "A", "r1"),
        segment_of(&schedule, "B", "r2"),
    );
    assert!(a.t0.abs() < 1e-9 && b.t0.abs() < 1e-9);
    // velocity_scale 2.0 halves B's 4s trajectory.
    assert!((b.duration() - 2.0).abs() < 1e-9);
    // Both finish at 2.0 + 0.5 hold + 1.0 settle = 3.5; homes start there.
    let (h1, h2) = (
        segment_of(&schedule, "home", "r1"),
        segment_of(&schedule, "home", "r2"),
    );
    assert!((h1.t0 - 3.5).abs() < 1e-9, "home r1 at {}", h1.t0);
    assert!((h2.t0 - 3.5).abs() < 1e-9, "home r2 at {}", h2.t0);
    // r2 has no planned return: slow fallback move, 7.0 + settle.
    assert!((schedule.duration - (3.5 + HOME_RETURN_TIME_S + COMPLETION_MARGIN_S)).abs() < 1e-9);
    assert_close(schedule.duration, 11.5);
}

#[test]
fn undeclared_port_is_rejected() {
    let mut plan = btcpp_plan();
    plan["behaviour_tree"]["xml"] =
        json!(BTCPP_XML.replace("has_interlock=\"false\"/>", "bogus=\"1\"/>"));
    let parsed = parse(plan);
    let err = Schedule::build(&parsed.tree, &[], Default::default()).expect_err("rejected");
    assert!(err.contains("bogus"), "{err}");
    assert!(err.starts_with("invalid BehaviorTree: "), "{err}");
}

#[test]
fn missing_blackboard_entry_warns_and_skips() {
    let mut plan = btcpp_plan();
    plan["behaviour_tree"]["blackboard"]
        .as_object_mut()
        .unwrap()
        .remove("traj_A");
    let schedule = build_bare(plan).schedule;
    assert!(schedule.segments.iter().all(|s| s.op_id != "A"));
    assert_eq!(
        common::messages(&schedule),
        ["ExecuteTrajectory A has no trajectory (blackboard has no `traj_A`) — skipped"]
    );
    assert_close(schedule.duration, 11.5);
}

#[test]
fn delay_decorator_holds_its_child_back() {
    let z = [0.0; 6];
    let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree">
            <Delay delay_msec="1500"><ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{t}"/></Delay>
        </BehaviorTree></root>"#;
    let plan = json!({ "behaviour_tree": { "xml": xml, "blackboard": { "t": [wp(z, 0.0), wp(z, 1.0)] } } });
    let schedule = build_bare(plan).schedule;
    assert!((schedule.segments[0].t0 - 1.5).abs() < 1e-9);
    assert_close(schedule.duration, 4.0);
}

#[test]
fn sequence_places_segments_back_to_back() {
    let schedule = build(sample_plan()).schedule;
    assert_eq!(schedule.segments.len(), 2);
    // W01: 2.0s + 0.5 hold + 1.0 settle = 3.5 -> W02 starts there.
    assert!((schedule.segments[0].t0 - 0.0).abs() < 1e-9);
    assert!((schedule.segments[1].t0 - 3.5).abs() < 1e-9);
    assert!((schedule.duration - 5.5).abs() < 1e-9);
    assert!(schedule.warnings.is_empty());
}

#[test]
fn velocity_scale_shortens_a_segment() {
    let mut plan = sample_plan();
    plan["behaviour_tree"]["root"]["children"][0]["velocity_scale"] = json!(2.0);
    let schedule = build(plan).schedule;
    assert!((schedule.segments[0].duration() - 1.0).abs() < 1e-9);
}

#[test]
fn pose_holds_final_configuration_after_a_segment_ends() {
    let schedule = build(sample_plan()).schedule;
    let end_of_w01 = schedule.segments[0].t_end();
    let held = schedule.pose_at("r1", end_of_w01 + 0.25);
    let last = schedule.segments[0].traj.last().unwrap().q;
    assert!(max_joint_error(&held, &last) < 1e-9);
    assert!(max_joint_error(&held, &[1.0, -1.0, 1.0, 0.0, 0.0, 0.0]) < 1e-9);
}

#[test]
fn parallel_ends_with_its_latest_child() {
    let a = [0.0; 6];
    let plan = json!({
        "root": { "type": "PARALLEL", "children": [
            { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A", "hold_duration_s": 0.0,
              "precomputed_trajectory": [wp(a, 0.0), wp(a, 1.0)] },
            { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B", "hold_duration_s": 0.0,
              "precomputed_trajectory": [wp(a, 0.0), wp(a, 4.0)] }
        ]}
    });
    let schedule = build_bare(plan).schedule;
    assert!((schedule.duration - 5.0).abs() < 1e-9); // 4.0 + settle
    assert_eq!(schedule.robots.len(), 2);
}

#[test]
fn hold_duration_defaults_to_the_btcpp_model_value() {
    let a = [0.0; 6];
    let plan = json!({
        "root": { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
                  "precomputed_trajectory": [wp(a, 0.0), wp(a, 1.0)] }
    });
    let schedule = build_bare(plan).schedule;
    assert!((schedule.segments[0].hold_s - 0.5).abs() < 1e-9);
}

#[test]
fn interlock_gate_opens_while_the_awaited_arm_holds_the_pose() {
    // r1 finishes at `target` and holds; r2's gate opens only after r1's
    // motion is already over, which the hold window must still satisfy.
    let start = [0.0; 6];
    let target = [1.5, -1.0, 1.0, 0.0, 0.0, 0.0];
    let plan = json!({
        "root": { "type": "SEQUENCE", "children": [
            { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
              "precomputed_trajectory": [wp(start, 0.0), wp(target, 2.0)] },
            { "type": "SEQUENCE", "children": [
                { "type": "INTERLOCK_WAIT", "id": "g1", "robot_ns": "r2",
                  "depends_on_joints": [
                    { "robot": "r1", "joint_positions": target, "tolerance_rad": 0.2 }
                  ] },
                { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B",
                  "precomputed_trajectory": [wp(start, 0.0), wp(target, 1.0)] }
            ]}
        ]}
    });
    let schedule = build_bare(plan).schedule;
    assert!(
        schedule.warnings.is_empty(),
        "gate should resolve, got {:?}",
        schedule.warnings
    );
    let b = segment(&schedule, "B");
    assert!(b.t0 >= 2.0 - 1e-9, "B started too early: {}", b.t0);
    // A ends at 3.5 (2.0 + 0.5 hold + 1.0 settle), with r1 at the target.
    assert_close(b.t0, 3.5);
    assert_close(schedule.duration, 6.0);
}

#[test]
fn wait_in_a_parallel_opens_when_the_other_arm_passes_the_pose() {
    // r2 waits (in parallel) for r1 to pass through `mid`, which r1 does at
    // t=1.0 on its way to `end` — well before r1's motion completes.
    let start = [0.0; 6];
    let mid = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let end = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let plan = json!({
        "root": { "type": "PARALLEL", "children": [
            { "type": "SEQUENCE", "children": [
                { "type": "INTERLOCK_WAIT", "robot_ns": "r2",
                  "depends_on_joints": [ { "robot": "r1", "joint_positions": mid, "tolerance_rad": 0.01 } ] },
                { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B",
                  "precomputed_trajectory": [wp(start, 0.0), wp(start, 1.0)] }
            ]},
            { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
              "precomputed_trajectory": [wp(start, 0.0), wp(mid, 1.0), wp(end, 2.0)] }
        ]}
    });
    let schedule = build_bare(plan).schedule;
    assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
    let b = segment(&schedule, "B");
    assert!((b.t0 - 1.0).abs() < 1e-9, "B started at {}", b.t0);
    assert_close(schedule.duration, 3.5);
}

#[test]
fn unreachable_gate_warns_and_falls_back() {
    let start = [0.0; 6];
    let unreachable = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let plan = json!({
        "root": { "type": "SEQUENCE", "children": [
            { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
              "precomputed_trajectory": [wp(start, 0.0), wp(start, 1.0)] },
            { "type": "INTERLOCK_WAIT", "id": "g1", "robot_ns": "r2",
              "depends_on_joints": [
                { "robot": "r1", "joint_positions": unreachable, "tolerance_rad": 0.05 }
              ] }
        ]}
    });
    let schedule = build_bare(plan).schedule;
    assert_eq!(schedule.warnings.len(), 1);
    assert!(schedule.warnings[0].message.contains("never reaches"));
    assert_eq!(
        schedule.warnings[0].message,
        "WaitForRobot r2: r1 never reaches the awaited configuration within 0.05 rad — \
         released once nothing else could move"
    );
    assert_close(schedule.duration, 2.5);
}

#[test]
fn angle_wrapping_treats_a_full_turn_as_identical() {
    assert!(angular_diff(std::f64::consts::PI, -std::f64::consts::PI) < 1e-9);
    assert!((angular_diff(0.0, std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-9);
}

#[test]
fn joints_are_reported_only_for_robots_in_the_plan() {
    let schedule = build(sample_plan()).schedule;
    assert!(schedule.robots.iter().any(|r| r == "r1"));
    assert!(!schedule.robots.iter().any(|r| r == "nobody"));
    let states = schedule.robot_states(1.0, u64::MAX);
    let robots: Vec<&str> = states.iter().map(|s| s.robot_id.as_str()).collect();
    assert_eq!(robots, ["r1"]);
}

#[test]
fn robot_states_follow_the_playhead() {
    // W01 [a@0, b@2] holds 0.5; W02 [b, a@1] starts at 3.5 (cell-runner's playback tests seek
    // a playhead over this plan; here the playhead is a parameter).
    let schedule = build(sample_plan()).schedule;
    let at = |t: f64| schedule.robot_states(t, u64::MAX).remove(0);

    let s = at(1.0);
    assert_eq!(
        (s.phase.as_str(), s.op_id.as_deref()),
        ("weld", Some("W01"))
    );
    assert_close(s.progress, 0.5);
    assert!(max_joint_error(&s.joints, &[0.5, -1.0, 1.0, 0.0, 0.0, 0.0]) < 1e-9);

    let s = at(2.25);
    assert_eq!(
        (s.phase.as_str(), s.op_id.as_deref()),
        ("hold", Some("W01"))
    );
    assert_close(s.progress, 1.0);

    let s = at(3.0);
    assert_eq!(
        (s.phase.as_str(), s.op_id.as_deref()),
        ("idle", Some("W01"))
    );

    let s = at(4.0);
    assert_eq!(
        (s.phase.as_str(), s.op_id.as_deref()),
        ("weld", Some("W02"))
    );
    assert_close(s.progress, 0.5);

    // Before its first motion a robot is parked where that motion starts.
    let s = at(-1.0);
    assert_eq!((s.phase.as_str(), s.op_id), ("idle", None));
    assert!(max_joint_error(&s.joints, &[0.0, -1.0, 1.0, 0.0, 0.0, 0.0]) < 1e-9);
    assert!(max_joint_error(&schedule.pose_at("r1", -1.0), &s.joints) < 1e-9);
}
