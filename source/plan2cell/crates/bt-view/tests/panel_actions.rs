//! Every [`Action`] applied to the welding plan, with a frame drawn after each, and the halt
//! banner drawn at every kind of halt: nothing panics. Decisions taken while the past is shown
//! apply at the live instant, a rejected edit lands in the error row, and a decision in the halt
//! banner lets the run go on the way it went.

use std::collections::BTreeSet;

use bt_view::bt_cell::{HaltedMotion, WarningKind};
use bt_view::{Action, BtPanel, CellMode, Executor, LeafPolicy, NodeKind, NodeStatus, Stop, Uid};
use serde_json::json;

const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";
const TOL: f64 = 1e-3;

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

/// One frame on a screen the size of a laptop's, so long rows wrap and tall ones overflow.
fn frame(panel: &mut BtPanel, ctx: &egui::Context) {
    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(1440.0, 900.0),
        )),
        ..Default::default()
    };
    let mut output = ctx.run_ui(input, |ui| {
        egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
    });
    // Nothing renders here, but epaint insists the texture deltas are consumed.
    output.textures_delta.clear();
}

fn ex(panel: &BtPanel) -> &Executor {
    panel.executor().expect("a tree is loaded")
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

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < TOL,
        "{actual} is not {expected} ± {TOL}"
    );
}

fn assert_finished_at(panel: &BtPanel, t: f64) {
    assert_eq!(
        panel.last_stop(),
        Some(&Stop::Finished(NodeStatus::Success))
    );
    assert_close(ex(panel).now(), t);
}

/// Which variant `action` is. There is no wildcard arm, so a new variant does not compile here
/// until [`every_action`] covers it.
fn variant(action: &Action) -> usize {
    match action {
        Action::Select(_) => 0,
        Action::Focus(_) => 1,
        Action::ToggleCollapse(_) => 2,
        Action::Force(..) => 3,
        Action::ClearForce(_) => 4,
        Action::ClearAllForces => 5,
        Action::Policy(..) => 6,
        Action::AllPolicies(..) => 7,
        Action::ReleaseAllLikeCellRunner => 8,
        Action::SetAutoRelease(_) => 9,
        Action::Breakpoint(..) => 10,
        Action::ClearBreakpoints => 11,
        Action::Resume => 12,
        Action::ResumeNode(_) => 13,
        Action::SetPort { .. } => 14,
        Action::SetBlackboard { .. } => 15,
        Action::RevertAllEdits => 16,
        Action::SeekTime(_) => 17,
        Action::Step => 18,
        Action::RunToWait => 19,
        Action::RunToTime(_) => 20,
        Action::SetRunning(_) => 21,
        Action::Reset => 22,
        Action::SetCellMode(_) => 23,
        Action::SetHaltedMotion(_) => 24,
        Action::ContinueRun => 25,
        Action::DismissError => 26,
        Action::RequestOpen => 27,
        Action::SetExternalActive(_) => 28,
    }
}
const VARIANTS: usize = 29;

/// At least one of each action, in an order that visits interesting states.
fn every_action(panel: &BtPanel) -> Vec<Action> {
    let w01 = by_name(panel, "weld_robot1_W01");
    let w13 = by_name(panel, "weld_robot2_W13");
    let park = by_name(panel, "move_robot1_R1_PARK");
    let p1 = seq_child(panel, 0);
    let port = |uid, value: Option<&str>| Action::SetPort {
        uid,
        port: "velocity_scale".to_string(),
        value: value.map(str::to_string),
    };
    vec![
        Action::Select(Some(w01)),
        Action::Focus(w13),
        Action::ToggleCollapse(p1),
        Action::ToggleCollapse(p1),
        Action::SeekTime(2.0),
        Action::SetExternalActive(vec![
            (w01, "robot1".to_string()),
            (9999, "nobody".to_string()),
        ]),
        Action::Force(w01, NodeStatus::Success, false),
        Action::ClearForce(w01),
        Action::Force(p1, NodeStatus::Failure, true),
        Action::ClearAllForces,
        Action::Policy(w13, LeafPolicy::Manual),
        Action::AllPolicies(NodeKind::Action, LeafPolicy::Manual),
        Action::AllPolicies(NodeKind::Action, LeafPolicy::Native),
        Action::SetAutoRelease(true),
        Action::SetAutoRelease(false),
        Action::ReleaseAllLikeCellRunner,
        Action::Breakpoint(park, true),
        Action::ContinueRun,
        Action::Resume,
        Action::ResumeNode(park),
        Action::ClearBreakpoints,
        port(w01, Some("0.5")),
        // The root has no ports: the error row says so.
        port(0, Some("2")),
        Action::DismissError,
        Action::SetBlackboard {
            key: "speed".to_string(),
            value: Some(json!({"scale": [1, 2, 3]})),
        },
        Action::SetBlackboard {
            key: "speed".to_string(),
            value: None,
        },
        port(w01, None),
        Action::SetBlackboard {
            key: "traj_robot1_W01".to_string(),
            value: Some(json!([])),
        },
        Action::RevertAllEdits,
        Action::SetHaltedMotion(HaltedMotion::Cut),
        Action::SetCellMode(CellMode::Off),
        Action::SetCellMode(CellMode::On),
        Action::SetCellMode(CellMode::Auto),
        Action::ContinueRun,
        Action::Reset,
        Action::Step,
        Action::RunToTime(3.0),
        Action::SetRunning(true),
        Action::SetRunning(false),
        Action::RunToWait,
        Action::RequestOpen,
    ]
}

/// r1 moves for 3.5 s, then `wait_r1` waits for r1 to reach a pose it never reaches.
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

/// Every node selected in turn, a frame each.
fn inspect_every_node(panel: &mut BtPanel, ctx: &egui::Context) {
    for uid in 0..ex(panel).tree().len() {
        panel.apply([Action::Select(Some(uid))]);
        frame(panel, ctx);
    }
}

#[test]
fn panel_actions_draw_headless() {
    let ctx = egui::Context::default();
    let mut panel = load(WELDING);
    frame(&mut panel, &ctx);
    inspect_every_node(&mut panel, &ctx);

    // Mid-motion (progress bars, "Stop motion now"), then the past.
    panel.run_to_time(5.0);
    inspect_every_node(&mut panel, &ctx);
    panel.apply([Action::SeekTime(2.5)]);
    assert_close(panel.shown_time(), 2.5);
    inspect_every_node(&mut panel, &ctx);
    panel.apply([Action::SeekTime(f64::INFINITY)]);

    let actions = every_action(&panel);
    let covered: BTreeSet<usize> = actions.iter().map(variant).collect();
    assert_eq!(covered, (0..VARIANTS).collect());
    for action in actions {
        let label = format!("{action:?}");
        panel.apply([action]);
        frame(&mut panel, &ctx);
        frame(&mut panel, &ctx);
        assert!(panel.executor().is_some(), "{label}");
    }
    assert!(panel.take_open_request());
    assert!(panel.error().is_none());

    // The halt banner at each kind of halt: manual leaves while a timer runs, manual leaves
    // with nothing scheduled, a breakpoint, a stalled gate.
    panel.reset();
    panel.apply([Action::Policy(
        by_name(&panel, "weld_robot1_W01"),
        LeafPolicy::Manual,
    )]);
    panel.run_to_wait();
    frame(&mut panel, &ctx);
    panel.apply([Action::AllPolicies(NodeKind::Action, LeafPolicy::Manual)]);
    panel.reset();
    panel.run_to_wait();
    assert!(ex(&panel).halted().is_some());
    frame(&mut panel, &ctx);
    panel.apply([
        Action::AllPolicies(NodeKind::Action, LeafPolicy::Native),
        Action::Breakpoint(by_name(&panel, "move_robot1_R1_PARK"), true),
    ]);
    panel.reset();
    panel.run_to_wait();
    assert!(matches!(panel.last_stop(), Some(Stop::Breakpoint(_))));
    frame(&mut panel, &ctx);

    let mut stall = BtPanel::new("stall");
    stall.load_text("stall.json", &stall_plan()).unwrap();
    stall.run_to_wait();
    assert!(matches!(stall.last_stop(), Some(Stop::Stalled(_))));
    frame(&mut stall, &ctx);
    inspect_every_node(&mut stall, &ctx);
    stall.set_running(true);
    frame(&mut stall, &ctx);

    // Actions about nodes that do not exist are ignored.
    let selected = panel.selected();
    panel.apply([
        Action::Force(10_000, NodeStatus::Success, false),
        Action::Select(Some(10_000)),
        Action::SetPort {
            uid: 10_000,
            port: "x".to_string(),
            value: None,
        },
    ]);
    assert_eq!(panel.selected(), selected);
    frame(&mut panel, &ctx);

    // Nothing loaded: every action is a no-op.
    let mut empty = BtPanel::new("empty");
    let actions = every_action(&panel);
    empty.apply(actions);
    frame(&mut empty, &ctx);
    assert!(empty.executor().is_none());
}

#[test]
fn decisions_while_scrubbed_apply_live() {
    let mut panel = load(WELDING);
    let w01 = by_name(&panel, "weld_robot1_W01");
    panel.run_to_time(5.0);

    // Looking back does not decide anything (no tick ran between 0 and 5).
    panel.apply([Action::SeekTime(2.0), Action::Breakpoint(w01, true)]);
    assert_close(panel.shown_time(), 2.0);
    panel.apply([Action::Breakpoint(w01, false)]);
    assert_close(panel.shown_time(), 2.0);

    // Forcing does, at now.
    panel.apply([Action::Force(w01, NodeStatus::Success, false)]);
    assert_close(panel.shown_time(), 5.0);
    assert!(ex(&panel).override_of(w01).is_some());
    panel.run_to_wait();
    // The motion was cut at 5.0, so P1 ends when W13 is done (cell_engine has the numbers).
    assert_finished_at(&panel, 73.8509);

    // A time at or after now is live.
    panel.apply([Action::SeekTime(10.0)]);
    assert!(panel.shown_tick() < ex(&panel).ticks());
    assert_close(panel.shown_time(), 10.0);
    panel.apply([Action::SeekTime(1e9)]);
    assert_eq!(panel.shown_tick(), ex(&panel).ticks());
    assert_close(panel.shown_time(), ex(&panel).now());
}

#[test]
fn a_rejected_edit_goes_to_the_error_row() {
    let mut panel = load(WELDING);
    let w01 = by_name(&panel, "weld_robot1_W01");
    let set = |uid, port: &str, value: Option<&str>| Action::SetPort {
        uid,
        port: port.to_string(),
        value: value.map(str::to_string),
    };
    panel.apply([set(w01, "velocity_scale", Some("0.5"))]);
    assert!(panel.error().is_none());
    let node = ex(&panel).tree().node(w01);
    assert_eq!(node.port_value("velocity_scale"), Some("0.5"));
    assert_eq!(
        ex(&panel).port_original(w01, "velocity_scale"),
        Some(Some("1.0"))
    );

    panel.apply([set(w01, "name", Some("x"))]);
    let error = panel.error().unwrap();
    assert!(error.contains("weld_robot1_W01.name"), "{error}");
    assert!(error.contains("is not a port"), "{error}");
    panel.apply([Action::DismissError]);
    assert!(panel.error().is_none());

    panel.apply([set(w01, "velocity_scale", None)]);
    assert_eq!(ex(&panel).port_original(w01, "velocity_scale"), None);
    assert_eq!(
        ex(&panel).tree().node(w01).port_value("velocity_scale"),
        Some("1.0")
    );

    // Blackboard edits and Revert all.
    panel.apply([
        Action::SetBlackboard {
            key: "x".to_string(),
            value: Some(json!(1)),
        },
        set(w01, "hold_duration_s", Some("0")),
    ]);
    assert!(ex(&panel).blackboard_edited("x"));
    panel.apply([Action::RevertAllEdits]);
    assert!(ex(&panel).blackboard_edits().is_empty());
    assert!(ex(&panel).port_edits().is_empty());
}

#[test]
fn continue_runs_on_past_manual_leaves() {
    let mut panel = load(WELDING);
    let w01 = by_name(&panel, "weld_robot1_W01");
    panel.apply([Action::Policy(w01, LeafPolicy::Manual)]);
    panel.run_to_wait();
    // W13 still moves: a stop, not a halt.
    assert_eq!(panel.last_stop(), Some(&Stop::WaitingManual(vec![w01])));
    assert_eq!(ex(&panel).now(), 0.0);
    assert_eq!(ex(&panel).halted(), None);

    // The header's Continue leaves W01 waiting while W13 finishes; then nothing moves.
    panel.apply([Action::Resume, Action::ContinueRun]);
    assert_eq!(panel.last_stop(), Some(&Stop::WaitingManual(vec![w01])));
    assert_close(ex(&panel).now(), 10.686);
    assert_eq!(ex(&panel).halted(), Some(Stop::WaitingManual(vec![w01])));

    // A row's Succeed decides and goes on: W01 never moved, so P1 ends at 10.686.
    panel.apply([
        Action::Force(w01, NodeStatus::Success, false),
        Action::ContinueRun,
    ]);
    assert_finished_at(&panel, 73.8509);
    // A finished run is left alone.
    let ticks = ex(&panel).ticks();
    panel.apply([Action::ContinueRun]);
    assert_eq!(ex(&panel).ticks(), ticks);
}

#[test]
fn continue_goes_on_the_way_the_run_went() {
    // A breakpoint during Run to wait: Continue runs on to the end.
    let mut panel = load(TWO_ROBOT);
    let home = by_name(&panel, "home_robot1");
    panel.apply([Action::Breakpoint(home, true)]);
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    panel.apply([Action::Resume, Action::ContinueRun]);
    assert_finished_at(&panel, 8.1479);

    // During Run to t: on to that instant only.
    panel.reset();
    panel.run_to_time(6.0);
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    assert_close(ex(&panel).now(), 4.7287);
    panel.apply([Action::ResumeNode(home), Action::ContinueRun]);
    assert_eq!(panel.last_stop(), Some(&Stop::Until));
    assert_close(ex(&panel).now(), 6.0);

    // After a Step: one more step.
    panel.reset();
    while panel.last_stop().is_none() {
        panel.step();
    }
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    let ticks = ex(&panel).ticks();
    panel.apply([Action::Breakpoint(home, false), Action::ContinueRun]);
    assert_eq!(ex(&panel).ticks(), ticks + 1);
    assert_eq!(ex(&panel).status(home), NodeStatus::Running);
    assert_close(ex(&panel).now(), 4.7287);

    // A real-time run stays armed: Continue leaves it to the next frame.
    panel.reset();
    panel.apply([Action::Breakpoint(home, true)]);
    assert!(ex(&panel).breakpoint(home));
    panel.set_running(true);
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    let ticks = ex(&panel).ticks();
    panel.apply([Action::ResumeNode(home), Action::ContinueRun]);
    assert_eq!(ex(&panel).ticks(), ticks);
    assert!(panel.is_running());
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    assert!(!panel.is_running());
    assert_finished_at(&panel, 8.1479);
}

/// Two stub actions under a Parallel, each with a breakpoint.
const TWO_BREAKPOINTS: &str = r#"<root BTCPP_format="4">
  <BehaviorTree ID="T">
    <Parallel success_count="-1" failure_count="1">
      <Work name="a"/><Work name="b"/>
    </Parallel>
  </BehaviorTree>
  <TreeNodesModel><Action ID="Work"/></TreeNodesModel>
</root>"#;

/// A decision about one paused node lets only that one start, whichever way the run went.
#[test]
fn a_decision_about_one_breakpoint_keeps_the_others() {
    for first in [Action::Step, Action::RunToWait] {
        let mut panel = BtPanel::new("two");
        panel.load_text("two.xml", TWO_BREAKPOINTS).unwrap();
        let (a, b) = (by_name(&panel, "a"), by_name(&panel, "b"));
        panel.apply([
            Action::Breakpoint(a, true),
            Action::Breakpoint(b, true),
            first.clone(),
        ]);
        assert_eq!(ex(&panel).paused_at(), &[a, b], "{first:?}");
        // What the halt banner's "▶ Continue" on row `a` sends.
        panel.apply([Action::ResumeNode(a), Action::ContinueRun]);
        assert_eq!(ex(&panel).status(a), NodeStatus::Running, "{first:?}");
        assert_eq!(ex(&panel).status(b), NodeStatus::Idle, "{first:?}");
        assert_eq!(ex(&panel).paused_at(), &[b], "{first:?}");
        assert_eq!(
            panel.last_stop(),
            Some(&Stop::Breakpoint(vec![b])),
            "{first:?}"
        );
        // "Remove" on `b` lets it go too (and Run to wait runs on to the end).
        panel.apply([Action::Breakpoint(b, false), Action::ContinueRun]);
        let started = ex(&panel)
            .trace()
            .of_node(b)
            .any(|t| t.status == NodeStatus::Running);
        assert!(started, "{first:?}");
    }
    // The toolbar's Step still lets every paused node start.
    let mut panel = BtPanel::new("two");
    panel.load_text("two.xml", TWO_BREAKPOINTS).unwrap();
    let (a, b) = (by_name(&panel, "a"), by_name(&panel, "b"));
    panel.apply([
        Action::Breakpoint(a, true),
        Action::Breakpoint(b, true),
        Action::Step,
        Action::Step,
    ]);
    assert_eq!(ex(&panel).status(a), NodeStatus::Running);
    assert_eq!(ex(&panel).status(b), NodeStatus::Running);
}

#[test]
fn a_decision_after_looking_back_arms_the_run_again() {
    let mut panel = load(TWO_ROBOT);
    let home = by_name(&panel, "home_robot1");
    panel.apply([Action::Breakpoint(home, true), Action::SetRunning(true)]);
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    assert!(panel.is_running());

    // Looking back pauses the run; back at live, the banner's Continue arms it again.
    panel.apply([Action::SeekTime(1.0)]);
    assert!(!panel.is_running());
    panel.scrub_to_time(None);
    panel.apply([Action::Resume, Action::ContinueRun]);
    assert!(panel.is_running());
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    assert_finished_at(&panel, 8.1479);

    // So does a decision after the user paused it.
    panel.apply([Action::SetRunning(true)]);
    for _ in 0..6 {
        panel.advance_real_time(1.0);
    }
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![home])));
    panel.apply([
        Action::SetRunning(false),
        Action::ResumeNode(home),
        Action::ContinueRun,
    ]);
    assert!(panel.is_running());
}

#[test]
fn releasing_all_stalled_gates_is_cell_runners_way_out() {
    let mut panel = BtPanel::new("stall");
    panel.load_text("stall.json", &stall_plan()).unwrap();
    let wait = by_name(&panel, "wait_r1");
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Stalled(vec![wait])));
    panel.apply([Action::ReleaseAllLikeCellRunner, Action::ContinueRun]);
    assert_finished_at(&panel, 6.0);
    let runner: Vec<_> = panel
        .cell()
        .unwrap()
        .schedule()
        .warnings
        .iter()
        .filter(|w| w.kind == WarningKind::Runner)
        .map(|w| w.message.clone())
        .collect();
    assert_eq!(runner.len(), 1);
    assert!(runner[0].contains("never reaches"), "{runner:?}");

    // Turning auto-release on in the banner releases and goes on.
    panel.reset();
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Stalled(vec![wait])));
    panel.apply([Action::SetAutoRelease(true), Action::ContinueRun]);
    assert_finished_at(&panel, 6.0);
}

#[test]
fn panel_settings_reach_the_cell_leaves() {
    let mut panel = load(TWO_ROBOT);
    let halted = |panel: &BtPanel| panel.cell().unwrap().options().halted_motion;
    assert_eq!(halted(&panel), HaltedMotion::Keep);
    panel.apply([Action::SetHaltedMotion(HaltedMotion::Cut)]);
    assert_eq!(halted(&panel), HaltedMotion::Cut);
    // Kept when the cell leaves are installed again.
    panel.apply([Action::SetCellMode(CellMode::Off)]);
    assert!(panel.cell().is_none());
    panel.apply([Action::SetCellMode(CellMode::On)]);
    assert_eq!(halted(&panel), HaltedMotion::Cut);

    // Focus selects the node.
    let home = by_name(&panel, "home_robot1");
    panel.apply([Action::Focus(home)]);
    assert_eq!(panel.selected(), Some(home));
    panel.apply([Action::Select(None)]);
    assert_eq!(panel.selected(), None);
}
