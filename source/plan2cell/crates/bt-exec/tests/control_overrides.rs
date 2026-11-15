//! Overrides on native leaves and on control nodes, and switching a leaf's policy mid-run.

mod common;

use std::collections::BTreeMap;

use bt_core::NodeStatus::{Failure, Idle, Running, Skipped, Success};
use bt_core::{Document, NodeKind};
use bt_exec::{Executor, HaltCause, LeafPolicy, Override, RunOptions, Stop};

use common::{assert_close, executor, first, force, history, motion, plain, toy, uid};

const TWO_MOVES: &str = r#"<Parallel>
    <Move name="a" robot="r1" secs="10"/>
    <Move name="b" robot="r2" secs="4"/>
  </Parallel>"#;

#[test]
fn force_cuts_motion_at_instant() {
    let mut ex = executor(TWO_MOVES);
    assert_eq!(ex.run(RunOptions::until(2.5)).stop, Stop::Until);
    let a = uid(&ex, "a");
    force(&mut ex, "a", Success);
    assert!(ex.retick_pending());
    assert_eq!(
        ex.waiting_reason(a).as_deref(),
        Some("forced SUCCESS on its next tick")
    );
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_eq!(toy(&ex).halts, [(a, 2.5, HaltCause::Forced(Success))]);
    assert_close(motion(&ex, "a").unwrap().t1, 2.5);
    assert_close(first(&ex, "a", Success).t, 2.5);
    assert_close(ex.now(), 4.0);
    assert_eq!(history(&ex, "a"), [Running, Success, Idle]);
}

#[test]
fn forced_failure_halts_sibling() {
    let mut ex = executor(TWO_MOVES);
    ex.run(RunOptions::until(2.5));
    let (a, b) = (uid(&ex, "a"), uid(&ex, "b"));
    force(&mut ex, "b", Failure);
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!(report.stop, Stop::Finished(Failure));
    // b is stopped by its override, then the failing Parallel halts a.
    assert_eq!(
        toy(&ex).halts,
        [
            (b, 2.5, HaltCause::Forced(Failure)),
            (a, 2.5, HaltCause::Tree)
        ]
    );
    assert_close(ex.now(), 2.5);
}

#[test]
fn control_override_halts_children() {
    let mut ex = executor(
        r#"<Sequence>
             <Parallel name="p">
               <Move name="a" robot="r1" secs="10"/>
               <Move name="b" robot="r2" secs="10"/>
             </Parallel>
             <Move name="c" robot="r1" secs="1"/>
           </Sequence>"#,
    );
    ex.run(RunOptions::until(2.0));
    let (p, a, b) = (uid(&ex, "p"), uid(&ex, "a"), uid(&ex, "b"));
    force(&mut ex, "p", Success);
    assert!(ex.retick_pending(), "p is RUNNING");
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_eq!(
        toy(&ex).halts,
        [
            (a, 2.0, HaltCause::AncestorForced(p)),
            (b, 2.0, HaltCause::AncestorForced(p))
        ]
    );
    assert_close(motion(&ex, "c").unwrap().t0, 2.0);
    assert_close(ex.now(), 3.0);
    assert_eq!(ex.override_of(p), None, "a one-shot override is consumed");
    assert_eq!(history(&ex, "p"), [Running, Success, Idle]);
}

const FALLBACK: &str = r#"<Sequence>
    <Fallback name="f"><A name="a"/></Fallback>
    <A name="b"/>
  </Sequence>"#;

#[test]
fn control_override_edges() {
    let mut ex = executor(FALLBACK);
    let (f, a) = (uid(&ex, "f"), uid(&ex, "a"));
    ex.set_policy(a, LeafPolicy::Manual);
    assert_eq!(
        ex.run(RunOptions::VIRTUAL).stop,
        Stop::WaitingManual(vec![a])
    );
    force(&mut ex, "f", Failure);
    assert!(ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Failure));
    assert_eq!(history(&ex, "a"), [Running, Idle]);
    assert!(history(&ex, "b").is_empty());

    // The root is not a node of the tree.
    let forced = Some(Override {
        status: Success,
        pinned: false,
    });
    ex.set_override(0, forced);
    assert_eq!(ex.override_of(0), None);

    // A skipped node is not ticked, so its override stays queued.
    let mut ex = executor(
        r#"<Sequence>
             <Script code="go := true"/>
             <Fallback name="f" _skipIf="go"><A name="a"/></Fallback>
             <A name="b"/>
           </Sequence>"#,
    );
    let f_skipped = uid(&ex, "f");
    ex.set_override(f_skipped, forced);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_eq!(history(&ex, "f"), [Skipped, Idle]);
    assert_eq!(ex.override_of(f_skipped), forced);

    // A pinned override decides f every time, so a never runs, also after a reset.
    let mut ex = executor(FALLBACK);
    let pinned = Some(Override {
        status: Success,
        pinned: true,
    });
    ex.set_override(f, pinned);
    for _ in 0..2 {
        assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
        assert_close(ex.now(), 1.0);
        assert!(history(&ex, "a").is_empty());
        ex.reset();
        assert_eq!(ex.override_of(f), pinned);
    }
    ex.clear_overrides();
    assert_eq!(ex.override_of(f), None);
}

#[test]
fn policy_switch_native_manual_native() {
    let mut ex = executor(r#"<Sequence><Move name="a" robot="r1" secs="5"/></Sequence>"#);
    let a = uid(&ex, "a");
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    ex.set_policy(a, LeafPolicy::Manual);
    assert!(ex.retick_pending());
    assert_eq!(
        ex.run(RunOptions::VIRTUAL).stop,
        Stop::WaitingManual(vec![a])
    );
    assert_eq!(toy(&ex).halts, [(a, 1.0, HaltCause::PolicyChanged)]);
    assert_close(motion(&ex, "a").unwrap().t1, 1.0);

    ex.set_policy(a, LeafPolicy::Native);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    let m = motion(&ex, "a").unwrap();
    assert_close(m.t0, 1.0);
    assert_close(m.t1, 6.0);
    assert_close(ex.now(), 6.0);
}

#[test]
fn policy_switch_clears_started_at() {
    let mut ex = executor(
        r#"<Parallel>
             <Move name="a" robot="r1" secs="5"/>
             <Move name="b" robot="r2" secs="10"/>
           </Parallel>"#,
    );
    let a = uid(&ex, "a");
    ex.set_policy(a, LeafPolicy::Manual);
    let report = ex.run(RunOptions {
        stop_on_manual: false,
        ..RunOptions::until(3.0)
    });
    assert_eq!(report.stop, Stop::Until);
    // The manual stub started a's timer at 0; simulating starts afresh at 3.
    ex.set_policy(a, LeafPolicy::Native);
    assert_eq!(ex.run(RunOptions::until(4.0)).stop, Stop::Until);
    assert_close(motion(&ex, "a").unwrap().t0, 3.0);
    // And a stub taking over at 4 times its 2 s from 4, not from 0 or 3.
    ex.set_policy(a, LeafPolicy::Succeed { after_s: 2.0 });
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(first(&ex, "a", Success).t, 6.0);
    assert_eq!(toy(&ex).halts, [(a, 4.0, HaltCause::PolicyChanged)]);
    assert_close(ex.now(), 10.0);
}

#[test]
fn changing_a_reticked_condition_acts_at_once() {
    // A running ReactiveSequence ticks its conditions again on every tick, so a change to one
    // matters now, although it is IDLE in between.
    let mut ex = plain(r#"<ReactiveSequence><C name="c"/><A name="m"/></ReactiveSequence>"#);
    let (c, m) = (uid(&ex, "c"), uid(&ex, "m"));
    ex.set_policy(m, LeafPolicy::Manual);
    assert_eq!(ex.run_virtual(100), (Running, 1));
    assert_eq!(ex.status(c), Idle);
    ex.set_policy(c, LeafPolicy::Fail { after_s: 0.0 });
    assert!(ex.retick_pending());
    assert_eq!(ex.run_virtual(100), (Failure, 1));

    // With motion: the tree stops at 1.0, not at m's own event at 5.0. A condition inside a
    // child that starts afresh on every tick counts too.
    let mut ex = executor(
        r#"<ReactiveSequence>
             <Sequence><C name="c1"/><C name="c2"/></Sequence>
             <Move name="m" robot="r1" secs="5"/>
             <C name="after"/>
           </ReactiveSequence>"#,
    );
    let m = uid(&ex, "m");
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    // What comes after the running child is not reached: it waits for the next event.
    ex.set_policy(uid(&ex, "after"), LeafPolicy::Fail { after_s: 0.0 });
    assert!(!ex.retick_pending());
    force(&mut ex, "c2", Failure);
    assert!(ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Failure));
    assert_close(ex.now(), 1.0);
    assert_eq!(toy(&ex).halts, [(m, 1.0, HaltCause::Tree)]);

    // WhileDoElse re-checks its condition on every tick.
    let mut ex = executor(
        r#"<WhileDoElse>
             <C name="c"/>
             <Move name="d" robot="r1" secs="5"/>
             <Move name="e" robot="r1" secs="1"/>
           </WhileDoElse>"#,
    );
    let (c, d) = (uid(&ex, "c"), uid(&ex, "d"));
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    ex.set_policy(c, LeafPolicy::Fail { after_s: 0.0 });
    assert!(ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_eq!(toy(&ex).halts, [(d, 1.0, HaltCause::Tree)]);
    assert_close(motion(&ex, "e").unwrap().t0, 1.0);
    assert_close(ex.now(), 2.0);
}

#[test]
fn forcing_the_safety_check_stops_a_waiting_cell() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test_data/v4_cell_with_gates.xml"
    );
    let xml = std::fs::read_to_string(path).unwrap();
    let tree = Document::parse(&xml).unwrap().instantiate("Cell").unwrap();
    let mut ex = Executor::new(tree, BTreeMap::new());
    // As `btgui --manual` does: only overrides finish the actions.
    ex.set_all_policies(NodeKind::Action, LeafPolicy::Manual);
    assert_eq!(ex.run_virtual(1_000).0, Running);
    assert!(!ex.waiting_on().is_empty());
    assert_eq!(ex.status(uid(&ex, "estop_clear")), Idle);
    force(&mut ex, "estop_clear", Failure);
    assert_eq!(ex.run_virtual(1_000), (Failure, 1));
}
