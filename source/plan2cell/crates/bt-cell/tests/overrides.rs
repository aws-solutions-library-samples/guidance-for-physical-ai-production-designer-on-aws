//! Overrides on the welding plan: forced leaves and control nodes cut motion at that instant,
//! policies and edits change what is placed, and the past stays as it was.

mod common;

use bt_cell::joints::interpolate;
use bt_cell::{HaltedMotion, WarningKind};
use bt_core::NodeStatus::{Failure, Running, Success};
use bt_exec::{Executor, LeafPolicy, RunOptions, Stop};

use common::{
    WELDING_STEP_ENDS, assert_close, assert_pose, by_name, cell, finished_at, force, schedule,
    segment, seq_child, start_of, warnings, waypoints, welding,
};

const W01: &str = "weld_robot1_W01";
const W13: &str = "weld_robot2_W13";

/// The welding plan run interactively up to t = 5, both welds of the first step moving.
fn at_five() -> Executor {
    let mut ex = common::interactive(&welding());
    assert_eq!(ex.run(RunOptions::until(5.0)).stop, Stop::Until);
    assert_close(ex.now(), 5.0);
    assert_eq!(ex.status(by_name(&ex, W01)), Running);
    ex
}

fn finish(ex: &mut Executor) -> Stop {
    ex.run(RunOptions::VIRTUAL).stop
}

#[test]
fn forcing_a_weld_mid_motion_cuts_it() {
    let mut ex = at_five();
    let w01 = by_name(&ex, W01);
    force(&mut ex, w01, Success);
    assert!(
        ex.retick_pending(),
        "W01 is RUNNING: the force applies at 5.0"
    );
    assert_eq!(finish(&mut ex), Stop::Finished(Success));

    let s = schedule(&ex);
    let w01 = segment(s, "W01");
    let cut = w01.cut.expect("W01 is cut");
    assert_close(w01.t_end(), 5.0);
    assert_close(w01.hold_s, 0.0);
    assert_close(cut.at, 5.0);
    assert!(!cut.dropped);
    assert_close(cut.planned_t_end, 10.632);
    assert_close(cut.planned_t_done, 11.132);
    // The robot stays where it was stopped.
    let plan = welding();
    let stopped = interpolate(&waypoints(&plan, "traj_robot1_W01"), 5.0);
    assert_pose(s.display_pose_at("robot1", 7.0, ex.ticks()), stopped);
    assert_pose(s.pose_at("robot1", 7.0), stopped);

    // W13 still runs to 10.686; W02 then starts away from where robot1 is.
    assert_close(finished_at(&ex, seq_child(&ex, 0)), 10.686);
    assert_close(segment(s, "W02").t0, 10.686);
    assert_eq!(warnings(s, WarningKind::Jump).len(), 1, "{:?}", s.warnings);
    assert_eq!(warnings(s, WarningKind::Cut).len(), 1);
    assert_eq!(
        warnings(s, WarningKind::Cut)[0].message,
        "robot1 W01: cut at t=5.000s"
    );
    assert!(warnings(s, WarningKind::Runner).is_empty());
    assert_eq!(s.warnings.len(), 2);
    assert_close(s.duration, 73.8509);
}

#[test]
fn failing_one_robot_halts_the_other() {
    for (motion, w01_end) in [(HaltedMotion::Keep, 10.632), (HaltedMotion::Cut, 5.0)] {
        let mut ex = at_five();
        cell_options(&mut ex).halted_motion = motion;
        let w13 = by_name(&ex, W13);
        force(&mut ex, w13, Failure);
        assert_eq!(finish(&mut ex), Stop::Finished(Failure), "{motion:?}");
        let s = schedule(&ex);
        // The Parallel halts W01: kept like cell-runner, or cut.
        assert_close(segment(s, "W01").t_end(), w01_end);
        assert_eq!(segment(s, "W01").cut.is_some(), motion == HaltedMotion::Cut);
        assert_close(segment(s, "W13").cut.expect("W13 is cut").at, 5.0);
        assert_close(s.duration, 5.0);
        let failed = "BehaviorTree returned FAILURE at t=5.00s";
        assert!(
            warnings(s, WarningKind::Runner)
                .iter()
                .any(|w| w.message == failed && w.uid.is_none())
        );
        assert!(ex.log().iter().any(|e| e.message == failed));
        assert_eq!(cell(&ex).finished(), Some((Failure, 5.0)));
    }
}

fn cell_options(ex: &mut Executor) -> &mut bt_cell::CellOptions {
    ex.leaf_handler_mut::<bt_cell::CellLeaves>()
        .unwrap()
        .options_mut()
}

#[test]
fn forcing_a_parallel_cuts_both_robots() {
    let mut ex = at_five();
    let p1 = seq_child(&ex, 0);
    force(&mut ex, p1, Success);
    assert!(ex.retick_pending());
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    let s = schedule(&ex);
    for op in ["W01", "W13"] {
        assert_close(segment(s, op).cut.expect("cut").at, 5.0);
        assert_close(segment(s, op).t_end(), 5.0);
    }
    assert_close(finished_at(&ex, p1), 5.0);
    assert_close(segment(s, "W02").t0, 5.0);
    assert_close(segment(s, "W15").t0, 5.0);
    assert_eq!(warnings(s, WarningKind::Jump).len(), 2, "{:?}", s.warnings);
    assert_eq!(warnings(s, WarningKind::Cut).len(), 2);
    assert!(warnings(s, WarningKind::Overlap).is_empty());
    assert_close(s.duration, 68.1649);
}

#[test]
fn switching_a_running_weld_to_a_stub_cuts_it() {
    let mut ex = at_five();
    let w01 = by_name(&ex, W01);
    ex.set_policy(w01, LeafPolicy::Succeed { after_s: 0.0 });
    assert!(
        ex.retick_pending(),
        "W01 is RUNNING: the switch applies at 5.0"
    );
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    let s = schedule(&ex);
    let w01 = segment(s, "W01");
    assert_close(w01.cut.expect("W01 is cut").at, 5.0);
    assert_close(w01.t_end(), 5.0);
    assert_eq!(warnings(s, WarningKind::Cut).len(), 1);
    assert_eq!(warnings(s, WarningKind::Jump).len(), 1, "{:?}", s.warnings);
    assert_eq!(s.warnings.len(), 2);
    assert_close(finished_at(&ex, seq_child(&ex, 0)), 10.686);
    assert_close(s.duration, 73.8509);
}

#[test]
fn stub_policy_skips_a_motion() {
    let plan = welding();
    let mut ex = common::interactive(&plan);
    let w01 = by_name(&ex, W01);
    ex.set_policy(w01, LeafPolicy::Succeed { after_s: 0.0 });
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    let s = schedule(&ex);
    assert!(s.segments.iter().all(|seg| seg.op_id != "W01"));
    assert_close(s.duration, 73.8509);
    assert!(s.warnings.is_empty(), "{:?}", s.warnings);
    // Before any motion of robot1 is placed it is shown where W01 would start; once W02 is
    // placed, where W02 starts.
    assert_pose(
        s.display_pose_at("robot1", 1.0, 1),
        start_of(&plan, "traj_robot1_W01"),
    );
    assert_pose(
        s.display_pose_at("robot1", 1.0, u64::MAX),
        start_of(&plan, "traj_robot1_W02"),
    );
}

#[test]
fn editing_velocity_scale_before_start() {
    let mut ex = common::interactive(&welding());
    let w01 = by_name(&ex, W01);
    ex.set_port_value(w01, "velocity_scale", "0.5").unwrap();
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    let s = schedule(&ex);
    assert_close(segment(s, "W01").duration(), 21.264);
    assert_close(finished_at(&ex, seq_child(&ex, 0)), 22.764);
    assert_close(s.duration, 85.9289);

    ex.revert_port_value(w01, "velocity_scale");
    ex.reset();
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    assert_close(schedule(&ex).duration, WELDING_STEP_ENDS[10]);
}

#[test]
fn manual_weld_waits() {
    let mut ex = common::interactive(&welding());
    let w01 = by_name(&ex, W01);
    ex.set_policy(w01, LeafPolicy::Manual);
    assert_eq!(finish(&mut ex), Stop::WaitingManual(vec![w01]));
    assert_close(ex.now(), 0.0);
    let s = schedule(&ex);
    assert!(s.segments.iter().all(|seg| seg.op_id != "W01"));
    assert_close(segment(s, "W13").t0, 0.0);

    force(&mut ex, w01, Success);
    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    assert_close(finished_at(&ex, seq_child(&ex, 0)), 10.686);
    assert_close(schedule(&ex).duration, 73.8509);
}

#[test]
fn progress_while_scrubbing() {
    let mut ex = at_five();
    let w01 = by_name(&ex, W01);
    let then = ex.ticks();
    // W01 runs 12.132 s: its trajectory, the hold and the settle.
    let expected = (5.0 / 12.132) as f32;
    let progress = |ex: &Executor, t: f64, upto: u64| cell(ex).progress(w01, t, upto);
    assert!((progress(&ex, 5.0, then).unwrap() - expected).abs() < 1e-6);
    assert!(progress(&ex, 0.0, then).unwrap().abs() < 1e-6);

    assert_eq!(finish(&mut ex), Stop::Finished(Success));
    let live = ex.ticks();
    assert!(live > then);
    // Worked out from the segments, so the past reads the same once W01 is over.
    assert!((progress(&ex, 5.0, live).unwrap() - expected).abs() < 1e-6);
    assert_eq!(progress(&ex, 13.0, live), None);
    // Before W01 was placed there is nothing to show.
    assert_eq!(progress(&ex, 0.0, 0), None);

    // A cut ends the progress at the cut, once the cut has happened.
    let mut ex = at_five();
    let then = ex.ticks();
    force(&mut ex, w01, Success);
    finish(&mut ex);
    assert!((progress(&ex, 5.0, ex.ticks()).unwrap() - expected).abs() < 1e-6);
    assert_eq!(progress(&ex, 6.0, ex.ticks()), None);
    assert!(progress(&ex, 6.0, then).is_some());
    // Robot states as of then still show W01 welding, uncut.
    let state = &cell(&ex).robot_states(5.0, then)[0];
    assert_eq!((state.phase.as_str(), state.cut), ("weld", false));
    assert_close(state.progress, 5.0 / 10.632);
    let state = &cell(&ex).robot_states(5.0, ex.ticks())[0];
    assert!(state.cut);
    // Live, the progress is on the planned scale too, and stops where robot1 stopped.
    let live = ex.ticks();
    for t in [5.0, 7.0] {
        let state = &cell(&ex).robot_states(t, live)[0];
        assert_eq!((state.phase.as_str(), state.cut), ("idle", true));
        assert_close(state.progress, 5.0 / 10.632);
    }
    assert_close(cell(&ex).robot_states(3.9, live)[0].progress, 3.9 / 10.632);
}
