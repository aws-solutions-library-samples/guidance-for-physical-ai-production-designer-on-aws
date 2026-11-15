//! The panel runs factoryos plans with cell-runner's semantics: Run to wait, Run to t, Step and
//! real time place the same motion, the toggle switches between the cell leaves and stubs, and
//! overrides, edits, breakpoints and stalls behave as in bt-cell.

use std::collections::BTreeMap;

use bt_view::NodeStatus::{Idle, Success};
use bt_view::bt_cell::{self, Schedule, Segment, WarningKind};
use bt_view::{BtPanel, CellMode, Executor, LeafPolicy, NodeStatus, Override, Stop, Uid};
use serde_json::json;

const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";
const TOL: f64 = 1e-6;

fn read(name: &str) -> String {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
    std::fs::read_to_string(format!("{path}{name}")).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn load(name: &str) -> BtPanel {
    let mut panel = BtPanel::new(name);
    panel
        .load_text(name, &read(name))
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    panel
}

fn frame(panel: &mut BtPanel, ctx: &egui::Context) {
    let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
        egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
    });
    output.textures_delta.clear();
}

fn ex(panel: &BtPanel) -> &Executor {
    panel.executor().expect("a tree is loaded")
}

fn ex_mut(panel: &mut BtPanel) -> &mut Executor {
    panel.executor_mut().expect("a tree is loaded")
}

fn schedule(panel: &BtPanel) -> &Schedule {
    panel.cell().expect("the cell leaves run").schedule()
}

fn by_name(panel: &BtPanel, name: &str) -> Uid {
    ex(panel)
        .tree()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named {name}"))
        .uid
}

/// The `i`-th child of the top-level Sequence (the welding plan's Parallels are unnamed).
fn seq_child(panel: &BtPanel, i: usize) -> Uid {
    let tree = ex(panel).tree();
    tree.nodes[tree.nodes[0].children[0]].children[i]
}

/// When `uid` last finished.
fn finished_at(panel: &BtPanel, uid: Uid) -> f64 {
    ex(panel)
        .trace()
        .of_node(uid)
        .filter(|t| t.status.is_completed())
        .last()
        .unwrap_or_else(|| panic!("node {uid} never finished"))
        .t
}

/// The only segment of operation `op`.
fn segment<'a>(s: &'a Schedule, op: &str) -> &'a Segment {
    let found: Vec<&Segment> = s.segments.iter().filter(|seg| seg.op_id == op).collect();
    assert_eq!(found.len(), 1, "segments of {op}: {found:#?}");
    found[0]
}

fn warnings(s: &Schedule, kind: WarningKind) -> Vec<&str> {
    s.warnings
        .iter()
        .filter(|w| w.kind == kind)
        .map(|w| w.message.as_str())
        .collect()
}

fn force(panel: &mut BtPanel, uid: Uid, status: NodeStatus) {
    ex_mut(panel).set_override(
        uid,
        Some(Override {
            status,
            pinned: false,
        }),
    );
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < TOL,
        "{actual} is not {expected} (± {TOL})"
    );
}

fn assert_finished_at(panel: &BtPanel, t: f64) {
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_close(ex(panel).now(), t);
    assert_close(schedule(panel).duration, t);
}

#[test]
fn sample_plans_simulate_like_cell_runner() {
    for (file, end, segments) in [(WELDING, 75.2969, 18), (TWO_ROBOT, 8.1479, 2)] {
        let mut panel = load(file);
        assert_eq!(panel.cell_mode(), CellMode::Auto);
        assert!(panel.cell().is_some(), "{file}");
        panel.run_to_wait();
        assert_finished_at(&panel, end);
        let s = schedule(&panel);
        assert_eq!(s.segments.len(), segments, "{file}");
        assert!(s.warnings.is_empty(), "{file}: {:?}", s.warnings);
        assert_eq!(
            panel.cell().unwrap().finished(),
            Some((Success, s.duration))
        );

        // Exactly what cell-runner's compile places.
        let plan = bt_cell::parse_plan(&read(file)).unwrap();
        let compiled = bt_cell::compile(&plan.tree, &[], BTreeMap::new()).unwrap();
        assert_eq!(s.segments, compiled.schedule.segments, "{file}");
    }
    let panel = load(WELDING);
    let info = panel.plan_info().unwrap();
    assert_eq!(
        info.tree_id.as_deref(),
        Some("bt_1790934549_welding_cell_2_small")
    );
    assert_eq!(info.cell_id.as_deref(), Some("welding_cell_2_small"));
    assert_eq!(info.num_operations, Some(16));
}

#[test]
fn real_time_run_places_motion_like_run_to_wait() {
    for file in [WELDING, TWO_ROBOT] {
        let mut virtual_run = load(file);
        virtual_run.run_to_wait();

        let mut real_time = load(file);
        real_time.set_running(true);
        let mut frames = 0;
        while real_time.is_running() {
            real_time.advance_real_time(1.0);
            frames += 1;
            assert!(frames < 1000, "{file} does not finish in real time");
        }
        assert_finished_at(&real_time, ex(&virtual_run).now());
        // One second of executor time per frame.
        assert_eq!(frames, ex(&virtual_run).now().ceil() as usize, "{file}");

        let (a, b) = (schedule(&virtual_run), schedule(&real_time));
        assert_eq!(a.segments.len(), b.segments.len(), "{file}");
        for (x, y) in a.segments.iter().zip(&b.segments) {
            assert_eq!((&x.op_id, &x.robot), (&y.op_id, &y.robot));
            assert!(
                (x.t0 - y.t0).abs() < 1e-9,
                "{file} {}: {} vs {}",
                x.op_id,
                x.t0,
                y.t0
            );
        }
        assert!(b.warnings.is_empty(), "{file}: {:?}", b.warnings);
    }
}

#[test]
fn old_xml_samples_keep_stubs() {
    for file in ["orchestrator_two_robots.xml", "v4_cell_with_gates.xml"] {
        let panel = load(file);
        assert!(panel.cell().is_none(), "{file}");
        assert_eq!(panel.plan_info(), Some(&bt_view::PlanInfo::default()));
    }
    let mut panel = load("orchestrator_two_robots.xml");
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_close(ex(&panel).now(), 2.0);
}

#[test]
fn stub_toggle() {
    let mut panel = load(TWO_ROBOT);
    panel.run_to_wait();
    // Already running the cell leaves: switching to On changes nothing, not even the run.
    panel.set_cell_mode(CellMode::On);
    assert!(ex(&panel).is_done());

    panel.set_cell_mode(CellMode::Off);
    assert_eq!(panel.cell_mode(), CellMode::Off);
    assert!(panel.cell().is_none());
    assert_eq!(ex(&panel).ticks(), 0, "the run starts over");
    assert_eq!(panel.last_stop(), None);
    let stage = by_name(&panel, "move_robot1_STAGE_R1");
    assert_eq!(
        ex(&panel).policy(stage),
        LeafPolicy::DEFAULT_ACTION,
        "a stub again"
    );
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_close(ex(&panel).now(), 2.0);

    panel.set_cell_mode(CellMode::On);
    assert!(panel.cell().is_some());
    assert_eq!(ex(&panel).ticks(), 0);
    panel.run_to_wait();
    assert_finished_at(&panel, 8.1479);

    // The choice is about the tree it was made on: the next file, or a reload, starts in Auto.
    panel.set_cell_mode(CellMode::Off);
    panel.load_text(WELDING, &read(WELDING)).unwrap();
    assert_eq!(panel.cell_mode(), CellMode::Auto);
    assert!(panel.cell().is_some());
    panel.run_to_wait();
    assert_finished_at(&panel, 75.2969);
    let mut v4 = load("v4_cell_with_gates.xml");
    v4.set_cell_mode(CellMode::On);
    v4.load_text(
        "orchestrator_two_robots.xml",
        &read("orchestrator_two_robots.xml"),
    )
    .unwrap();
    assert!(
        v4.cell().is_none(),
        "a tree without trajectories stays on stubs"
    );

    // On runs cell leaves without trajectories as well; trees without cell leaves stay stubs.
    let mut v4 = load("v4_cell_with_gates.xml");
    v4.set_cell_mode(CellMode::On);
    assert!(v4.cell().is_some());
    v4.set_cell_mode(CellMode::Auto);
    assert!(v4.cell().is_none());
    let mut simple = load("Simple.xml");
    simple.set_cell_mode(CellMode::On);
    assert!(simple.cell().is_none());
}

/// r1 moves for 3.5 s (motion, hold, settle), then `wait_r1` waits for r1 to reach a pose it
/// never reaches; r2's 2.5 s move after the gate ends the run at 6.0.
fn stall_plan() -> String {
    let wp = |t: f64| json!({ "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "time_s": t });
    let body = r#"<Sequence>
        <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
        <WaitForRobot name="wait_r1" robot_ns="r2" depends_on_joints="{deps}"/>
        <ExecuteTrajectory name="B" robot_ns="r2" operation_id="B" trajectory="{b}"/>
      </Sequence>"#;
    json!({
        "tree_id": "stall",
        "behaviour_tree": {
            "xml": format!(r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree">{body}</BehaviorTree></root>"#),
            "blackboard": {
                "a": [wp(0.0), wp(2.0)],
                "deps": [{ "robot": "r1", "joint_positions": [3.0, 0, 0, 0, 0, 0], "tolerance_rad": 0.05 }],
                "b": [wp(0.0), wp(1.0)]
            }
        }
    })
    .to_string()
}

fn stalled() -> (BtPanel, Uid) {
    let mut panel = BtPanel::new("stall");
    panel.load_text("stall.json", &stall_plan()).unwrap();
    assert!(panel.cell().is_some());
    let wait = by_name(&panel, "wait_r1");
    (panel, wait)
}

#[test]
fn stalled_gate_halts_and_releases() {
    let ctx = egui::Context::default();
    let (mut panel, wait) = stalled();
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Stalled(vec![wait])));
    assert_close(ex(&panel).now(), 3.5);
    assert_eq!(ex(&panel).halted(), Some(Stop::Stalled(vec![wait])));
    let reason = ex(&panel).waiting_reason(wait).unwrap();
    assert!(
        reason.contains("no motion placed so far brings r1"),
        "{reason}"
    );
    // The stop brings the gate into view, and leaves the selection alone.
    frame(&mut panel, &ctx);
    assert_eq!(panel.selected(), None);

    force(&mut panel, wait, Success);
    panel.run_to_wait();
    assert_finished_at(&panel, 6.0);
    let s = schedule(&panel);
    assert_eq!(
        warnings(s, WarningKind::Override),
        ["WaitForRobot wait_r1: released by an override (r1 3.000 rad from the awaited joints)"]
    );
    assert!(warnings(s, WarningKind::Runner).is_empty());

    // Like cell-runner: released once nothing else could move.
    let (mut panel, _) = stalled();
    panel.set_auto_release(true);
    panel.run_to_wait();
    assert_finished_at(&panel, 6.0);
    let s = schedule(&panel);
    let runner = warnings(s, WarningKind::Runner);
    assert_eq!(runner.len(), 1);
    assert!(runner[0].contains("never reaches"), "{runner:?}");
    assert!(warnings(s, WarningKind::Override).is_empty());
}

/// The shipped example of a gate that never opens: it halts where the README says, and
/// releasing it like cell-runner ends the run at cell-runner's 16.5 s.
#[test]
fn the_gate_stall_example_halts_and_releases() {
    const GATE_STALL: &str = "cell_gate_stall.json";
    let compiled = bt_cell::compile(
        &bt_cell::parse_plan(&read(GATE_STALL)).unwrap().tree,
        &[],
        BTreeMap::new(),
    )
    .unwrap();
    assert_close(compiled.schedule.duration, 16.5);

    let mut panel = load(GATE_STALL);
    let wait = by_name(&panel, "wait_robot1_parked");
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Stalled(vec![wait])));
    assert_close(ex(&panel).now(), 8.5);
    let ctx = egui::Context::default();
    frame(&mut panel, &ctx);
    panel.apply([
        bt_view::Action::ReleaseAllLikeCellRunner,
        bt_view::Action::ContinueRun,
    ]);
    assert_finished_at(&panel, 16.5);
    assert_eq!(schedule(&panel).segments, compiled.schedule.segments);
    let runner = warnings(schedule(&panel), WarningKind::Runner);
    assert_eq!(runner.len(), 1);
    assert!(runner[0].contains("robot1 never reaches"), "{runner:?}");
}

#[test]
fn a_real_time_run_waits_at_a_stall_and_stays_armed() {
    let (mut panel, wait) = stalled();
    panel.set_running(true);
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    // The clock waits at the stall; the run is still on.
    assert!(panel.is_running());
    assert_close(ex(&panel).now(), 3.5);
    assert_eq!(panel.last_stop(), Some(&Stop::Stalled(vec![wait])));

    // Turning auto-release on lets it go on in the next frame.
    panel.set_auto_release(true);
    for _ in 0..3 {
        panel.advance_real_time(1.0);
    }
    assert!(!panel.is_running());
    assert_finished_at(&panel, 6.0);
}

#[test]
fn forcing_a_running_motion_cuts_it() {
    let mut panel = load(TWO_ROBOT);
    panel.run_to_time(2.0);
    assert_eq!(panel.last_stop(), Some(&Stop::Until));
    assert_close(ex(&panel).now(), 2.0);
    let stage = by_name(&panel, "move_robot1_STAGE_R1");
    assert_eq!(ex(&panel).status(stage), NodeStatus::Running);

    force(&mut panel, stage, Success);
    panel.run_to_wait();
    assert_finished_at(&panel, 5.4192);
    let s = schedule(&panel);
    let cut = segment(s, "STAGE_R1").cut.expect("STAGE_R1 is cut");
    assert_close(cut.at, 2.0);
    assert_close(segment(s, "STAGE_R1").t_end(), 2.0);
    assert_close(segment(s, "home").t0, 2.0);
    assert_eq!(warnings(s, WarningKind::Jump).len(), 1, "{:?}", s.warnings);
    assert_eq!(warnings(s, WarningKind::Cut).len(), 1);
}

#[test]
fn forcing_a_parallel_halts_its_children() {
    let mut panel = load(WELDING);
    panel.run_to_time(5.0);
    let p1 = seq_child(&panel, 0);
    force(&mut panel, p1, Success);
    panel.run_to_wait();
    assert_finished_at(&panel, 68.1649);
    assert_close(finished_at(&panel, p1), 5.0);
    let s = schedule(&panel);
    for op in ["W01", "W13"] {
        assert_close(segment(s, op).cut.expect("cut").at, 5.0);
    }
}

#[test]
fn run_to_time_passes_manual_leaves() {
    let manual = || {
        let mut panel = load(WELDING);
        let w01 = by_name(&panel, "weld_robot1_W01");
        ex_mut(&mut panel).set_policy(w01, LeafPolicy::Manual);
        (panel, w01)
    };
    // Run to wait stops at once for the manual leaf ...
    let (mut panel, w01) = manual();
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::WaitingManual(vec![w01])));
    assert_close(ex(&panel).now(), 0.0);

    // ... Run to t goes on while it waits.
    let (mut panel, w01) = manual();
    panel.run_to_time(5.0);
    assert_eq!(panel.last_stop(), Some(&Stop::Until));
    assert_close(ex(&panel).now(), 5.0);
    assert_eq!(ex(&panel).status(w01), NodeStatus::Running);
    let s = schedule(&panel);
    assert_close(segment(s, "W13").t0, 0.0);
    assert!(s.segments.iter().all(|seg| seg.op_id != "W01"));

    force(&mut panel, w01, Success);
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_close(finished_at(&panel, seq_child(&panel, 0)), 10.686);
}

#[test]
fn edits_persist_across_reset_and_revert() {
    let mut panel = load(TWO_ROBOT);
    let stage = by_name(&panel, "move_robot1_STAGE_R1");
    // STAGE_R1 moves 3.2287 s, holds 0.5 and settles 1; the move home takes 2.4192 + 1.
    let home = 2.4192 + 1.0;
    ex_mut(&mut panel)
        .set_port_value(stage, "velocity_scale", "2.0")
        .unwrap();
    let twice_as_fast = 3.2287 / 2.0 + 1.5 + home; // ≈ 6.5336
    panel.run_to_wait();
    assert_finished_at(&panel, twice_as_fast);
    panel.reset();
    panel.run_to_wait();
    assert_finished_at(&panel, twice_as_fast);

    ex_mut(&mut panel).revert_port_value(stage, "velocity_scale");
    panel.run_to_wait();
    assert_finished_at(&panel, 8.1479);

    ex_mut(&mut panel)
        .set_port_value(stage, "hold_duration_s", "0")
        .unwrap();
    panel.run_to_wait();
    assert_finished_at(&panel, 7.6479);
}

#[test]
fn breakpoint_pauses_before_a_node_starts() {
    let ctx = egui::Context::default();
    let paused = || {
        let mut panel = load(TWO_ROBOT);
        let home = by_name(&panel, "home_robot1");
        ex_mut(&mut panel).set_breakpoint(home, true);
        panel.run_to_wait();
        (panel, home)
    };
    let (mut panel, home) = paused();
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    assert_close(ex(&panel).now(), 4.7287);
    assert_eq!(ex(&panel).status(home), Idle);
    assert_eq!(ex(&panel).halted(), Some(Stop::Breakpoint(vec![home])));
    assert!(schedule(&panel).segments.iter().all(|s| s.op_id != "home"));
    frame(&mut panel, &ctx);

    ex_mut(&mut panel).resume();
    panel.run_to_wait();
    assert_finished_at(&panel, 8.1479);

    // Step resumes too, and starts the node at once.
    let (mut panel, home) = paused();
    panel.step();
    assert_eq!(ex(&panel).status(home), NodeStatus::Running);
    assert_close(segment(schedule(&panel), "home").t0, 4.7287);
    assert_eq!(panel.last_stop(), None);
}

#[test]
fn step_lands_on_events() {
    let mut panel = load(TWO_ROBOT);
    let mut at = Vec::new();
    for _ in 0..5 {
        panel.step();
        at.push(ex(&panel).now());
    }
    for (a, e) in at.iter().zip([0.0, 0.0, 4.7287, 4.7287, 8.1479]) {
        assert_close(*a, e);
    }
    assert_finished_at(&panel, 8.1479);
    // A Step on a finished tree starts it over.
    panel.step();
    assert_eq!(ex(&panel).ticks(), 1);
    assert_eq!(panel.last_stop(), None);
}

#[test]
fn scrubbing_shows_the_past() {
    let mut panel = load(WELDING);
    panel.run_to_wait();
    let live = ex(&panel).ticks();
    panel.scrub_to_time(Some(5.0));
    assert_close(panel.shown_time(), 5.0);
    let tick = panel.shown_tick();
    assert!(tick < live);
    assert_eq!(tick, ex(&panel).tick_at_time(5.0));
    let robots = panel.cell().unwrap().robot_states(panel.shown_time(), tick);
    let robot1 = robots.iter().find(|r| r.robot_id == "robot1").unwrap();
    assert_eq!(robot1.op_id.as_deref(), Some("W01"));

    // An instant not before now is live again.
    panel.scrub_to_time(Some(1000.0));
    assert_eq!(panel.shown_tick(), live);
    assert_close(panel.shown_time(), ex(&panel).now());
}

#[test]
fn loop_restarts_the_schedule() {
    let mut panel = load(TWO_ROBOT);
    panel.set_looping(true);
    panel.set_running(true);
    let mut wrapped = false;
    for _ in 0..5 {
        let before = ex(&panel).now();
        panel.advance_real_time(3.0);
        wrapped |= ex(&panel).now() < before;
    }
    assert!(wrapped);
    assert!(panel.is_running());
    assert!(ex(&panel).now() < 8.1479);
    assert!(schedule(&panel).segments.len() <= 2);
}
