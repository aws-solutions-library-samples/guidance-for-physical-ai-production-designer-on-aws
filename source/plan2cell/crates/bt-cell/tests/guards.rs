//! Where an interactive run could drift from cell-runner, and the guards that keep it from
//! doing so: wakeups, stalls, port defaults, edits between events, runaway trees.

mod common;

use std::collections::BTreeMap;

use bt_cell::joints::{FALLBACK_HOME_JOINTS, MAX_SEGMENTS};
use bt_cell::{ExecuteOp, ParsedPlan, PlacedBy, PlanWaypoint, Schedule, WarningKind, compat};
use bt_core::NodeStatus::{Failure, Success};
use bt_exec::{LeafPolicy, RunOptions, Stop};
use serde_json::{Value, json};

use common::{
    assert_close, assert_pose, by_name, cell, force, interactive, messages, parse, schedule,
    segment, warnings, wp, xml_plan,
};

const Z: [f64; 6] = [0.0; 6];

fn q(x: f64) -> [f64; 6] {
    [x, 0.0, 0.0, 0.0, 0.0, 0.0]
}

fn plan(body: &str, blackboard: Value) -> ParsedPlan {
    parse(xml_plan(body, blackboard))
}

/// G1. A motion halted at 2.0 had a wakeup at 3.0, when r1 happens to pass the gate's pose
/// between two waypoints. cell-runner clears wakeups every tick, so nothing ticks at 3.0 and
/// the gate opens at the waypoint where r1 holds that pose, 9.0.
#[test]
fn stale_wakeups_do_not_open_a_gate_early() {
    let body = r#"<Sequence>
        <Parallel success_count="1">
          <ExecuteTrajectory name="SHORT" robot_ns="r3" operation_id="SHORT" trajectory="{short}"/>
          <ExecuteTrajectory name="LONG" robot_ns="r2" operation_id="LONG" trajectory="{long}"/>
        </Parallel>
        <Parallel>
          <ExecuteTrajectory name="MOVE" robot_ns="r1" operation_id="MOVE" trajectory="{move}"/>
          <Sequence>
            <WaitForRobot name="GATE" robot_ns="r4" depends_on_joints="{deps}"/>
            <ExecuteTrajectory name="AFTER" robot_ns="r4" operation_id="AFTER" trajectory="{after}"/>
          </Sequence>
        </Parallel>
      </Sequence>"#;
    let blackboard = json!({
        // 0.5 + 0.5 hold + 1 settle: done at 2.0.
        "short": [wp(Z, 0.0), wp(Z, 0.5)],
        // Done at 3.0, but halted at 2.0 when SHORT succeeds.
        "long": [wp(Z, 0.0), wp(Z, 1.5)],
        // From 2.0: through q(1) at 3.0 on the way to q(2) at 4.0, back to q(1) at 9.0.
        "move": [wp(Z, 0.0), wp(q(2.0), 2.0), wp(q(1.0), 7.0)],
        "deps": [{ "robot": "r1", "joint_positions": q(1.0), "tolerance_rad": 0.01 }],
        "after": [wp(Z, 0.0), wp(Z, 1.0)]
    });
    let plan = plan(body, blackboard);
    let compiled = bt_cell::compile(&plan.tree, &[], Default::default()).unwrap();
    let mut ex = interactive(&plan);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    for s in [&compiled.schedule, schedule(&ex)] {
        assert_close(segment(s, "AFTER").t0, 9.0);
        assert_close(s.duration, 11.5);
        assert!(s.warnings.is_empty(), "{:?}", s.warnings);
    }
    assert!(
        (1..=ex.ticks()).all(|k| (ex.time_at_tick(k).unwrap() - 3.0).abs() > 1e-6),
        "nothing ticks at 3.0"
    );
}

/// cell-runner's unreachable gate (`trajectory.rs:1310-1327`).
fn unreachable_gate() -> ParsedPlan {
    parse(json!({
        "root": { "type": "SEQUENCE", "children": [
            { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
              "precomputed_trajectory": [wp(Z, 0.0), wp(Z, 1.0)] },
            { "type": "INTERLOCK_WAIT", "name": "gate", "robot_ns": "r2",
              "depends_on_joints": [
                { "robot": "r1", "joint_positions": q(3.0), "tolerance_rad": 0.05 }
              ] }
        ]}
    }))
}

/// G3. Interactively, a gate nothing can open halts the tree where it is; releasing it like
/// cell-runner gives cell-runner's warning, forcing it an override warning.
#[test]
fn a_stalled_gate_halts_until_released_or_forced() {
    let mut ex = interactive(&unreachable_gate());
    let gate = by_name(&ex, "gate");
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Stalled(vec![gate]));
    assert_close(ex.now(), 2.5);
    assert_eq!(ex.halted(), Some(Stop::Stalled(vec![gate])));
    let reason = ex.waiting_reason(gate).unwrap();
    assert_eq!(
        reason,
        "gate: no motion placed so far brings r1 within 0.05 rad of the awaited joints \
         (3.000 rad away)"
    );
    let view = cell(&ex).gate(gate).unwrap();
    assert_eq!((view.label.as_str(), view.since), ("gate", 2.5));
    assert_eq!(view.deps.len(), 1);
    assert_eq!(view.deps[0].opens_at, None);

    let mut released = interactive(&unreachable_gate());
    released.run(RunOptions::VIRTUAL);
    released.release_stalled();
    assert_eq!(
        released.run(RunOptions::VIRTUAL).stop,
        Stop::Finished(Success)
    );
    let s = schedule(&released);
    assert_eq!(warnings(s, WarningKind::Runner).len(), 1);
    assert!(messages(s)[0].contains("never reaches"));
    assert_close(s.duration, 2.5);
    assert!(
        released
            .log()
            .iter()
            .any(|e| e.message.contains("never reaches"))
    );

    force(&mut ex, gate, Success);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    let s = schedule(&ex);
    assert!(warnings(s, WarningKind::Runner).is_empty());
    assert_eq!(
        messages(s),
        ["WaitForRobot gate: released by an override (r1 3.000 rad from the awaited joints)"]
    );
    assert_eq!(warnings(s, WarningKind::Override)[0].uid, Some(gate));

    let mut failed = interactive(&unreachable_gate());
    failed.run(RunOptions::VIRTUAL);
    force(&mut failed, gate, Failure);
    assert_eq!(
        failed.run(RunOptions::VIRTUAL).stop,
        Stop::Finished(Failure)
    );
    assert!(messages(schedule(&failed))[0].contains("gate: failed by an override"));
}

/// One `ExecuteTrajectory` with `attrs`, under a model that declares other defaults.
fn single(attrs: &str) -> ParsedPlan {
    let xml = format!(
        r#"<root BTCPP_format="4" main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree"><Sequence>
          <ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{{t}}" {attrs}/>
        </Sequence></BehaviorTree>
        <TreeNodesModel><Action ID="ExecuteTrajectory">
          <input_port name="hold_duration_s" default="2.0"/>
          <input_port name="velocity_scale" default="4.0"/>
        </Action></TreeNodesModel></root>"#
    );
    parse(
        json!({ "behaviour_tree": { "xml": xml, "blackboard": { "t": [wp(Z, 0.0), wp(Z, 2.0)] } } }),
    )
}

/// G4 / G6. behaviortree-rs ignores `<TreeNodesModel>`: a missing or empty port takes
/// cell-runner's Rust default.
#[test]
fn port_defaults_are_cell_runners_not_the_models() {
    for (attrs, hold, duration) in [
        ("", 0.5, 2.0),
        (r#"hold_duration_s="""#, 0.5, 2.0),
        (r#"velocity_scale="2""#, 0.5, 1.0),
        (r#"hold_duration_s="0" velocity_scale="0.5""#, 0.0, 4.0),
    ] {
        let s = build_bare_plan(&single(attrs));
        let a = segment(&s, "A");
        assert_close(a.hold_s, hold);
        assert_close(a.duration(), duration);
        let mut ex = interactive(&single(attrs));
        ex.run(RunOptions::VIRTUAL);
        assert_eq!(schedule(&ex).segments, s.segments, "{attrs}");
    }
}

fn build_bare_plan(plan: &ParsedPlan) -> bt_cell::Schedule {
    bt_cell::compile(&plan.tree, &[], Default::default())
        .unwrap()
        .schedule
}

fn interlock(target: &str) -> ParsedPlan {
    let body = format!(
        r#"<Sequence><ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{{traj}}" has_interlock="True" interlock_target='{target}'/></Sequence>"#
    );
    plan(
        &body,
        json!({
            "traj": [wp(Z, 0.0), wp(Z, 1.0)],
            "it": { "joint_positions": q(0.1) }
        }),
    )
}

/// G5. The interlock target is a blackboard object; written inline, behaviortree-rs reads it
/// as a pointer to nothing, so the pre-move is silently left out.
#[test]
fn interlock_pre_move_comes_from_the_blackboard() {
    let s = build_bare_plan(&interlock("{it}"));
    let pre = segment(&s, "A:interlock");
    assert_close(pre.t0, 0.0);
    assert_close(pre.duration(), 2.5);
    assert_eq!(pre.op_type, "move");
    assert_close(segment(&s, "A").t0, 2.5);
    assert_close(s.duration, 5.0);
    assert!(warnings(&s, WarningKind::Runner).is_empty());
    // A starts at Z, not where the pre-move went.
    assert_eq!(warnings(&s, WarningKind::Jump).len(), 1, "{:?}", s.warnings);
    assert_eq!(s.warnings.len(), 1);

    let s = build_bare_plan(&interlock(r#"{"joint_positions":[0.1,0,0,0,0,0]}"#));
    assert_eq!(s.segments.len(), 1);
    assert_close(segment(&s, "A").t0, 0.0);
    assert_close(s.duration, 2.5);
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
}

/// G9. cell-runner reads `delay_msec` as a float.
#[test]
fn fractional_delay() {
    let plan = plan(
        r#"<Delay delay_msec="1500.5"><ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{t}"/></Delay>"#,
        json!({ "t": [wp(Z, 0.0), wp(Z, 1.0)] }),
    );
    assert_close(segment(&build_bare_plan(&plan), "A").t0, 1.5005);
    let mut ex = interactive(&plan);
    ex.run(RunOptions::VIRTUAL);
    assert_close(segment(schedule(&ex), "A").t0, 1.5005);
}

/// r2 waits for r1 to pass `q(0.5)`, which r1 does at 1.0 between two waypoints: the gate
/// never sees it (cell-runner polls only at waypoints and the start of a hold).
fn between_waypoints() -> ParsedPlan {
    plan(
        r#"<Parallel>
          <ExecuteTrajectory name="M" robot_ns="r1" operation_id="M" trajectory="{m}"/>
          <Sequence>
            <WaitForRobot name="wait" robot_ns="r2" depends_on_joints="{deps}"/>
            <ExecuteTrajectory name="X" robot_ns="r2" operation_id="X" trajectory="{x}"/>
          </Sequence>
        </Parallel>"#,
        json!({
            "m": [wp(Z, 0.0), wp(q(1.0), 2.0)],
            "deps": [{ "robot": "r1", "joint_positions": q(0.5), "tolerance_rad": 0.01 }],
            "x": [wp(Z, 0.0), wp(Z, 1.0)]
        }),
    )
}

/// S2. Edits between events tick nothing, so they cannot open a gate early.
#[test]
fn edits_do_not_open_gates_between_waypoints() {
    let mut plain = interactive(&between_waypoints());
    let wait = by_name(&plain, "wait");
    assert_eq!(
        plain.run(RunOptions::VIRTUAL).stop,
        Stop::Stalled(vec![wait])
    );
    assert_close(plain.now(), 3.5);

    let mut ex = interactive(&between_waypoints());
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    let x = by_name(&ex, "X");
    ex.set_port_value(x, "velocity_scale", "2").unwrap();
    ex.set_breakpoint(x, true);
    ex.set_breakpoint(x, false);
    ex.set_blackboard_value("x", json!([wp(Z, 0.0), wp(Z, 3.0)]));
    assert!(!ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Stalled(vec![wait]));
    assert_close(ex.now(), 3.5);
    assert_eq!(ex.trace().transitions(), plain.trace().transitions());
    assert_eq!(schedule(&ex).segments, schedule(&plain).segments);
}

/// Forcing a gate open is an override even where its robot happens to be at the awaited pose:
/// gates are polled only at events, and cell-runner keeps this one waiting until 3.5.
#[test]
fn forcing_a_gate_between_events_is_an_override() {
    let mut ex = interactive(&between_waypoints());
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    let wait = by_name(&ex, "wait");
    force(&mut ex, wait, Success);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    let s = schedule(&ex);
    assert_eq!(
        messages(s),
        ["WaitForRobot wait: released by an override (r1 0.000 rad from the awaited joints)"]
    );
    assert!(warnings(s, WarningKind::Runner).is_empty());
    assert_close(segment(s, "X").t0, 1.0);
}

/// A gate halted without being decided (the Parallel above it forced, or a policy switch)
/// gives no override warning.
#[test]
fn halting_a_gate_without_deciding_it_says_nothing() {
    let mut ex = interactive(&between_waypoints());
    ex.run(RunOptions::until(1.0));
    let parallel = ex.tree().nodes[0].children[0];
    force(&mut ex, parallel, Success);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 1.0);
    let s = schedule(&ex);
    assert_eq!(messages(s), ["r1 M: cut at t=1.000s"]);
    assert!(s.segments.iter().all(|seg| seg.op_id != "X"));

    let mut ex = interactive(&between_waypoints());
    ex.run(RunOptions::until(1.0));
    let wait = by_name(&ex, "wait");
    ex.set_policy(wait, LeafPolicy::Succeed { after_s: 0.0 });
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    let s = schedule(&ex);
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
    assert_close(segment(s, "X").t0, 1.0);
}

/// A `MoveHome` forced while it moves stops where it is, like any motion.
#[test]
fn a_running_move_home_is_cut() {
    let plan = plan(
        r#"<Sequence><MoveHome name="H" robot_ns="r1" trajectory="{h}"/></Sequence>"#,
        json!({ "h": [wp(Z, 0.0), wp(q(1.0), 4.0)] }),
    );
    let mut ex = interactive(&plan);
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    let h = by_name(&ex, "H");
    assert!(cell(&ex).motion(h).is_some());
    force(&mut ex, h, Success);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    let s = schedule(&ex);
    let home = segment(s, "home");
    assert_close(home.t_end(), 1.0);
    assert_close(home.cut.expect("cut").planned_t_end, 4.0);
    assert_pose(s.pose_at("r1", 2.0), q(0.25));
    assert_eq!(messages(s), ["r1 home: cut at t=1.000s"]);
    assert_close(s.duration, 1.0);
}

/// An unplanned home and an interlock pre-move start where the robot's latest motion ends.
/// When the tree halted that motion and it plays on (`HaltedMotion::Keep`), the robot is not
/// there yet, and placement says so.
#[test]
fn moves_from_the_last_pose_are_checked_against_a_kept_motion() {
    let parallel = r#"<Parallel success_count="1">
          <ExecuteTrajectory name="S" robot_ns="r2" operation_id="S" trajectory="{s}"/>
          <ExecuteTrajectory name="L" robot_ns="r1" operation_id="L" trajectory="{l}"/>
        </Parallel>"#;
    let blackboard = json!({
        // Done at 2.5, when the Parallel halts L, which plays on until 10.5.
        "s": [wp(Z, 0.0), wp(Z, 1.0)],
        "l": [wp(Z, 0.0), wp(q(1.0), 10.0)],
        "n": [wp(q(2.0), 0.0), wp(q(2.0), 1.0)],
        "it": { "joint_positions": q(2.0) }
    });
    for (next, op) in [
        (r#"<MoveHome robot_ns="r1"/>"#, "home"),
        (
            r#"<ExecuteTrajectory robot_ns="r1" operation_id="N" trajectory="{n}" has_interlock="true" interlock_target="{it}"/>"#,
            "N:interlock",
        ),
    ] {
        let plan = plan(
            &format!("<Sequence>{parallel}{next}</Sequence>"),
            blackboard.clone(),
        );
        let s = build_bare_plan(&plan);
        assert_close(segment(&s, op).t0, 2.5);
        assert_pose(segment(&s, op).traj[0].q, q(1.0));
        assert_eq!(
            messages(&s),
            [
                format!(
                    "r1: {op} starts 0.750 rad from where the robot is at t=2.500s — playback jumps"
                ),
                format!("r1: {op} starts at t=2.500s while L runs until 10.500s"),
            ]
        );
        let mut ex = interactive(&plan);
        ex.run(RunOptions::VIRTUAL);
        assert_eq!(schedule(&ex).warnings, s.warnings);
    }
}

/// cell-runner reads a `WaitForRobot`'s `name` like any port (`btree.rs:387-389`): a name equal
/// to the ID still counts, `null` and empty do not, `{key}` comes from the blackboard.
#[test]
fn a_gate_is_labelled_by_its_name_port() {
    for (attrs, label) in [
        (r#"name="WaitForRobot""#, "WaitForRobot"),
        (r#"name="null""#, "r2"),
        (r#"name="""#, "r2"),
        (r#"name="{label}""#, "from the blackboard"),
        (r#"name="{nothing}""#, "r2"),
        ("", "r2"),
    ] {
        let plan = plan(
            &format!(
                r#"<Sequence><WaitForRobot {attrs} robot_ns="r2" depends_on_positions="[[1, 2, 3]]"/></Sequence>"#
            ),
            json!({ "label": "from the blackboard" }),
        );
        assert_eq!(
            messages(&build_bare_plan(&plan)),
            [format!(
                "WaitForRobot {label}: 1 Cartesian gate(s) ignored (playback synthesizes no FK)"
            )],
            "{attrs}"
        );
        let ex = interactive(&plan);
        assert!(matches!(
            bt_cell::preview(&ex, 2),
            Some(bt_cell::Preview::Wait { label: l, .. }) if l == label
        ));
    }
}

/// What behaviortree-rs refuses to build does not compile, though bt-core only warns.
#[test]
fn trees_behaviortree_rs_cannot_build_do_not_compile() {
    let et = r#"<ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{t}"/>"#;
    let blackboard = json!({ "t": [wp(Z, 0.0), wp(Z, 1.0)] });
    let compile = |plan: &ParsedPlan| bt_cell::compile(&plan.tree, &[], Default::default());
    let typo = plan(
        &format!(r#"<Fallback><SubTree ID="Typo"/>{et}</Fallback>"#),
        blackboard.clone(),
    );
    assert_eq!(
        compile(&typo).err().unwrap(),
        "invalid BehaviorTree: Can't find tree [Typo]"
    );

    let two_trees = |main: &str| {
        let xml = format!(
            r#"<root BTCPP_format="4"{main}><BehaviorTree ID="A"><Sequence>{et}</Sequence></BehaviorTree><BehaviorTree ID="B"><Sequence>{et}</Sequence></BehaviorTree></root>"#
        );
        parse(json!({ "behaviour_tree": { "xml": xml, "blackboard": blackboard } }))
    };
    assert_eq!(
        compile(&two_trees("")).err().unwrap(),
        "invalid BehaviorTree: No main tree was provided, either in the XML or as a function \
         parameter."
    );
    // cell-runner looks a missing main tree up; bt-core falls back to the first.
    assert_eq!(
        compile(&two_trees(r#" main_tree_to_execute="C""#))
            .err()
            .unwrap(),
        "invalid BehaviorTree: Can't find tree [C]"
    );
    assert!(compile(&two_trees(r#" main_tree_to_execute="B""#)).is_ok());
}

/// behaviortree-rs refuses a ReactiveSequence with two motions; bt-exec restarts the first
/// whenever the second runs, forever. The segment limit stops it.
#[test]
fn runaway_reactive_tree_is_bounded() {
    let plan = plan(
        r#"<ReactiveSequence>
          <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
          <ExecuteTrajectory name="B" robot_ns="r2" operation_id="B" trajectory="{b}"/>
        </ReactiveSequence>"#,
        json!({ "a": [wp(Z, 0.0), wp(Z, 2.0)], "b": [wp(Z, 0.0), wp(Z, 1.0)] }),
    );
    let mut ex = interactive(&plan);
    let issues = compat::check(ex.tree());
    assert_eq!(issues.len(), 1);
    assert!(
        issues[0]
            .message
            .contains("Only a single child can return Running")
    );
    let err = bt_cell::compile(&plan.tree, &[], Default::default())
        .err()
        .unwrap();
    // cell-runner builds it, and fails the tick in which B starts (`plans6/g07`).
    assert_eq!(
        err,
        "BehaviorTree tick failed at t=3.50s: [ReactiveSequence]: Only a single child can return \
         Running."
    );

    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Failure));
    let s = schedule(&ex);
    assert!(s.segments.len() <= MAX_SEGMENTS);
    assert!(s.segments.len() > MAX_SEGMENTS - 2);
    let limit = warnings(s, WarningKind::Limit);
    assert_eq!(limit.len(), 1);
    assert_eq!(
        limit[0].message,
        format!("motion limit reached ({MAX_SEGMENTS} segments) — runaway tree?")
    );
}

/// The skip and ignore warnings, word for word cell-runner's (`btree.rs:216-418`).
#[test]
fn leaf_warnings_match_cell_runner() {
    let plan = plan(
        r#"<Sequence>
          <ExecuteTrajectory operation_id="A" trajectory="{t}"/>
          <ExecuteTrajectory robot_ns="r1" trajectory="{t}"/>
          <ExecuteTrajectory robot_ns="r1" operation_id="C"/>
          <ExecuteTrajectory robot_ns="r1" operation_id="D" trajectory="[]"/>
          <ExecuteTrajectory robot_ns="r1" operation_id="E" trajectory="{nothing}"/>
          <ExecuteTrajectory robot_ns="r1" operation_id="F" trajectory="weld"/>
          <MoveHome/>
          <MoveHome robot_ns="r1" trajectory="{nothing}"/>
          <WaitForRobot robot_ns="r2" depends_on_joints="{bad}" depends_on_positions="[[1, 2, 3]]"/>
          <CheckSafety condition="door_closed"/>
        </Sequence>"#,
        json!({ "t": [wp(Z, 0.0), wp(Z, 1.0)], "bad": 3 }),
    );
    let s = build_bare_plan(&plan);
    let found = messages(&s);
    let expected = [
        "ExecuteTrajectory without robot_ns — skipped",
        "ExecuteTrajectory C has no trajectory — skipped",
        "ExecuteTrajectory D has no trajectory — skipped",
        "ExecuteTrajectory E has no trajectory (blackboard has no `nothing`) — skipped",
        "ExecuteTrajectory F has no trajectory (`trajectory` is not a waypoint list: invalid \
         type: string \"weld\", expected a sequence) — skipped",
        "MoveHome without robot_ns — skipped",
        "MoveHome r1: blackboard has no `nothing` — moving home unplanned",
        "WaitForRobot r2: unreadable depends_on_joints (invalid type: integer `3`, expected a \
         sequence) — ignored",
        "WaitForRobot r2: 1 Cartesian gate(s) ignored (playback synthesizes no FK)",
    ];
    assert_eq!(found, expected);
    // The op without an id is "op"; the unplanned home goes to the fallback home.
    assert_close(segment(&s, "op").t0, 0.0);
    let home = segment(&s, "home");
    assert_eq!(home.traj.len(), 2);
    assert_eq!(home.traj[1].q, bt_cell::joints::FALLBACK_HOME_JOINTS);
    assert_close(home.duration(), bt_cell::joints::HOME_RETURN_TIME_S);
    assert!(warnings(&s, WarningKind::Jump).is_empty());

    // Interactively, the same warnings, also in the executor log.
    let mut ex = interactive(&plan);
    ex.run(RunOptions::VIRTUAL);
    assert_eq!(messages(schedule(&ex)), expected);
    for warning in expected {
        assert!(ex.log().iter().any(|e| e.message == warning), "{warning}");
    }
    assert!(matches!(
        bt_cell::preview(&ex, by_name(&ex, "CheckSafety")),
        Some(bt_cell::Preview::Safety { condition }) if condition == "door_closed"
    ));
}

fn op(id: &str, traj: &[([f64; 6], f64)], start_delay_s: f64) -> ExecuteOp {
    ExecuteOp {
        robot: "r1".to_string(),
        op_id: id.to_string(),
        op_type: "weld".to_string(),
        traj: traj
            .iter()
            .map(|(q, t)| PlanWaypoint {
                positions: q.to_vec(),
                time_s: *t,
            })
            .collect(),
        start_delay_s,
        hold_s: 0.5,
        velocity_scale: 1.0,
        interlock_joints: None,
    }
}

fn by(uid: usize, tick: u64) -> PlacedBy {
    PlacedBy {
        uid: Some(uid),
        tick,
    }
}

#[test]
fn a_cut_drops_trims_or_shortens_and_keeps_the_past() {
    let mut placed = Schedule::new(&[], BTreeMap::new());
    // Starts at 1.0, moves until 3.0, holds until 3.5, done at 4.5.
    let p = placed.start_execute(&op("A", &[(Z, 0.0), (q(1.0), 2.0)], 1.0), 0.0, by(1, 1));
    assert_eq!(p.segments, 0..1);
    assert_close(p.done_at, 4.5);

    // Not started: dropped, as if it was never placed.
    let mut s = placed.clone();
    assert!(s.cut(&p, 0.5, 2));
    let cut = s.segments[0].cut.unwrap();
    assert!(cut.dropped && s.segments[0].is_dropped());
    assert_eq!(
        (cut.tick, cut.planned_t_end, cut.planned_t_done),
        (2, 3.0, 3.5)
    );
    assert_eq!(s.pose_at("r1", 2.0), FALLBACK_HOME_JOINTS);
    assert_eq!(s.last_pose_of("r1"), FALLBACK_HOME_JOINTS);
    assert_eq!(s.gate_time("r1", &q(1.0), 0.01, 0.5), None);
    assert_eq!((s.operation_count(), s.placed_count()), (0, 0));
    assert!(s.segments[0].visible_at(1) && !s.segments[0].visible_at(2));
    assert!(s.describe().starts_with("0 segments"));

    // Moving: it ends where it is; what came before is untouched.
    let mut s = placed.clone();
    assert!(s.cut(&p, 2.0, 2));
    let a = &s.segments[0];
    assert_close(a.t_end(), 2.0);
    assert_close(a.hold_s, 0.0);
    assert_eq!(a.traj.len(), 2);
    assert_pose(s.pose_at("r1", 1.5), placed.pose_at("r1", 1.5));
    assert_pose(s.pose_at("r1", 5.0), q(0.5));
    assert_pose(s.last_pose_of("r1"), q(0.5));
    assert_eq!(a.extent_at(1), (3.0, 3.5));
    assert_close(s.horizon(), 3.5);
    // Cut again later, it still remembers the plan.
    assert!(!s.cut(&p, 2.5, 3));
    assert_eq!(s.segments[0].cut.unwrap().planned_t_done, 3.5);

    // Holding: the hold ends.
    let mut s = placed.clone();
    assert!(s.cut(&p, 3.2, 2));
    assert_close(s.segments[0].hold_s, 0.2);
    assert_close(s.segments[0].t_done(), 3.2);
    assert_eq!(s.segments[0].traj, placed.segments[0].traj);

    // Done: nothing to cut.
    let mut s = placed.clone();
    assert!(!s.cut(&p, 4.0, 2));
    assert_eq!(s.segments, placed.segments);
}

#[test]
fn placement_checks_continuity() {
    let mut s = Schedule::new(&[], BTreeMap::new());
    s.start_execute(&op("A", &[(Z, 0.0), (q(1.0), 2.0)], 0.0), 0.0, by(1, 1));
    assert!(s.warnings.is_empty(), "the first motion is not checked");
    // Starts where A ends, after A: fine.
    s.start_execute(&op("B", &[(q(1.0), 0.0), (Z, 1.0)], 0.0), 3.5, by(2, 2));
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
    // Starts away from where the robot is, while B still holds.
    s.start_execute(&op("C", &[(q(2.0), 0.0), (Z, 1.0)], 0.0), 4.7, by(3, 3));
    let found: Vec<(WarningKind, &str)> = s
        .warnings
        .iter()
        .map(|w| (w.kind, w.message.as_str()))
        .collect();
    assert_eq!(
        found,
        [
            (
                WarningKind::Jump,
                "r1: C starts 2.000 rad from where the robot is at t=4.700s — playback jumps"
            ),
            (
                WarningKind::Overlap,
                "r1: C starts at t=4.700s while B runs until 5.000s"
            ),
        ]
    );
    assert_eq!(s.warnings[0].uid, Some(3));
    // Homes are checked too. An unplanned one starts where the robot's latest motion ends,
    // which is where it is by 10.0.
    s.start_home("r1", &[], 10.0, by(4, 4));
    assert_eq!(s.warnings.len(), 2);
    let planned = [PlanWaypoint {
        positions: q(3.0).to_vec(),
        time_s: 0.0,
    }];
    s.start_home("r1", &planned, 20.0, by(5, 5));
    assert_eq!(s.warnings.len(), 3);
    assert!(s.warnings[2].message.starts_with("r1: home starts"));
}
