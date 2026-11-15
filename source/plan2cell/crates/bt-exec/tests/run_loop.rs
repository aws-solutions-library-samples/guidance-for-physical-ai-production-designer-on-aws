//! `Executor::run`: ticks at events only, stops say why, and stalls release like cell-runner.

mod common;

use bt_core::NodeStatus::{Failure, Running, Success};
use bt_exec::{LeafPolicy, Override, RunOptions, Stop};
use serde_json::json;

use common::{assert_close, executor, logged, motion, plain_xml, tick_times, uid};

#[test]
fn stall_reports_and_releases() {
    let body = r#"<Sequence>
        <Gate name="g" robot="nobody"/>
        <Move name="m" robot="r1" secs="2"/>
      </Sequence>"#;
    let mut ex = executor(body);
    let g = uid(&ex, "g");
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!(report.stop, Stop::Stalled(vec![g]));
    assert_eq!((report.status, report.ticks), (Running, 1));
    assert_eq!(ex.blocked(), [g]);
    assert_eq!(ex.halted(), Some(Stop::Stalled(vec![g])));
    assert_eq!(ex.waiting_reason(g).as_deref(), Some("waiting for nobody"));
    // Nothing changes until the user decides.
    assert_eq!(ex.run(RunOptions::VIRTUAL).ticks, 0);

    ex.release_stalled();
    assert!(ex.retick_pending());
    assert_eq!(ex.halted(), None);
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!(report.stop, Stop::Finished(Success));
    assert_close(ex.now(), 2.0);
    assert_eq!(logged(&ex, "released"), 1);

    // Or let the run release stalls by itself, as cell-runner does.
    ex.reset();
    let report = ex.run(RunOptions {
        release_stalls: true,
        ..RunOptions::VIRTUAL
    });
    assert_eq!(report.stop, Stop::Finished(Success));
    assert_close(ex.now(), 2.0);
    assert_eq!(logged(&ex, "released"), 1);
}

#[test]
fn stale_wakeups_dropped() {
    let mut ex = executor(
        r#"<Fallback>
             <Timeout msec="200"><A name="slow"/></Timeout>
             <A name="b"/>
           </Fallback>"#,
    );
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    // `slow` was halted at 0.2; its timer at 1.0 must not tick the tree.
    let times = tick_times(&ex);
    assert_eq!(times.len(), 3, "{times:?}");
    for (t, expected) in times.into_iter().zip([0.0, 0.2, 1.2]) {
        assert_close(t, expected);
    }
}

#[test]
fn time_table() {
    let mut ex = executor(r#"<Sequence><A name="a1"/><A name="a2"/></Sequence>"#);
    assert_eq!(ex.time_at_tick(0), Some(0.0));
    assert_eq!(ex.tick_at_time(5.0), 0);
    ex.run(RunOptions::VIRTUAL);
    assert_eq!(tick_times(&ex), [0.0, 1.0, 2.0]);
    assert_eq!(ex.time_at_tick(0), Some(0.0));
    assert_eq!(ex.time_at_tick(3), Some(2.0));
    assert_eq!(ex.time_at_tick(4), None);
    assert_eq!(ex.tick_at_time(-1.0), 0);
    assert_eq!(ex.tick_at_time(1.5), 2);
    assert_eq!(ex.tick_at_time(2.0), 3);
    ex.reset();
    assert_eq!(ex.time_at_tick(1), None);
}

#[test]
fn step_is_one_event() {
    let mut ex = executor(r#"<Sequence><A name="a1"/><A name="a2"/></Sequence>"#);
    let step = RunOptions {
        max_ticks: 1,
        ..RunOptions::VIRTUAL
    };
    for (now, stop) in [
        (0.0, Stop::TickLimit),
        (1.0, Stop::TickLimit),
        (2.0, Stop::Finished(Success)),
    ] {
        let report = ex.run(step);
        assert_eq!((report.ticks, report.stop), (1, stop));
        assert_close(ex.now(), now);
    }
}

const MOVES: &str = r#"<Parallel>
    <Move name="a" robot="r1" secs="10"/>
    <Sequence>
      <Move name="b" robot="r2" secs="4"/>
      <Move name="c" robot="r2" secs="1"/>
    </Sequence>
  </Parallel>"#;

#[test]
fn mutators_only_retick_when_they_matter() {
    let started = || {
        let mut ex = executor(MOVES);
        assert_eq!(ex.run(RunOptions::until(2.5)).stop, Stop::Until);
        assert_close(ex.now(), 2.5);
        ex
    };

    // Changes to nodes that do not run take effect at the next event, not at 2.5.
    let mut ex = started();
    let c = uid(&ex, "c");
    ex.set_port_value(c, "secs", "2").unwrap();
    ex.set_breakpoint(c, true);
    ex.set_breakpoint(c, false);
    ex.set_policy(c, LeafPolicy::Manual);
    ex.set_policy(c, LeafPolicy::Native);
    ex.set_blackboard_value("x", json!(1));
    // A breakpoint never holds a RUNNING node, so toggling it changes nothing now either.
    let (a, b) = (uid(&ex, "a"), uid(&ex, "b"));
    ex.set_breakpoint(b, true);
    ex.set_breakpoint(b, false);
    ex.set_breakpoint(a, false);
    ex.set_breakpoint(b, true);
    ex.clear_breakpoints();
    ex.set_breakpoint(b, true);
    assert!(!ex.retick_pending());
    assert_eq!(ex.halted(), None);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert!(
        tick_times(&ex).iter().all(|t| (t - 2.5).abs() > 1e-9),
        "{:?}",
        tick_times(&ex)
    );
    let c = motion(&ex, "c").unwrap();
    assert_close(c.t0, 4.0);
    assert_close(c.t1, 6.0);
    assert_close(ex.now(), 10.0);

    // Forcing a RUNNING node acts at this instant.
    let mut ex = started();
    let a = uid(&ex, "a");
    ex.set_override(
        a,
        Some(Override {
            status: Success,
            pinned: false,
        }),
    );
    assert!(ex.retick_pending());
    let report = ex.run(RunOptions {
        max_ticks: 1,
        ..RunOptions::VIRTUAL
    });
    assert_eq!(report.ticks, 1);
    assert_eq!(ex.time_at_tick(ex.ticks()), Some(2.5));

    // So does a policy change of a RUNNING leaf.
    let mut ex = started();
    ex.set_policy(uid(&ex, "b"), LeafPolicy::Manual);
    assert!(ex.retick_pending());
}

#[test]
fn halted_ignores_busy_and_moving_trees() {
    let mut ex =
        executor(r#"<Parallel><A name="m"/><Move name="b" robot="r1" secs="3"/></Parallel>"#);
    let (m, b) = (uid(&ex, "m"), uid(&ex, "b"));
    ex.set_policy(m, LeafPolicy::Manual);
    assert_eq!(ex.halted(), None, "not started");
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!(report.stop, Stop::WaitingManual(vec![m]));
    assert_close(ex.now(), 0.0);
    // b still moves: the tree is waiting, not halted.
    assert_eq!(ex.wakeup_of(b), Some(3.0));
    assert_eq!(ex.halted(), None);
    assert!(ex.blocked().is_empty());

    let report = ex.run(RunOptions {
        stop_on_manual: false,
        ..RunOptions::VIRTUAL
    });
    assert_eq!(report.stop, Stop::WaitingManual(vec![m]));
    assert_close(ex.now(), 3.0);
    assert_eq!(ex.halted(), Some(Stop::WaitingManual(vec![m])));
    assert_eq!(
        ex.waiting_reason(m).as_deref(),
        Some("policy 'wait': RUNNING until you force it")
    );

    // A busy tree with no timer and no blocked leaf stops, but it is not halted.
    let mut ex =
        executor(r#"<KeepRunningUntilFailure><Script code="n := 1"/></KeepRunningUntilFailure>"#);
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!((report.ticks, report.stop), (1, Stop::Stalled(vec![])));
    assert_eq!(ex.halted(), None);
}

#[test]
fn run_virtual_keeps_its_contract() {
    // A busy tree with no timer only moves when ticked: one tick per call, as always.
    let mut ex =
        executor(r#"<KeepRunningUntilFailure><Script code="n := 1"/></KeepRunningUntilFailure>"#);
    for _ in 0..3 {
        assert_eq!(ex.run_virtual(100_000), (Running, 1));
    }
    assert_eq!(ex.ticks(), 3);

    // An empty tree fails, and the report says so although the root never changed.
    let mut ex = plain_xml(
        r#"<root BTCPP_format="4"><BehaviorTree ID="T"/></root>"#,
        Default::default(),
    );
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!((report.status, report.ticks), (Failure, 1));
    assert_eq!(report.stop, Stop::Finished(Failure));
    assert_eq!(ex.run_virtual(10), (Failure, 1));
}
