//! Breakpoints hold a node before it starts; port and blackboard edits persist until reverted.

mod common;

use std::collections::BTreeMap;

use bt_core::NodeStatus::{Idle, Running, Success};
use bt_exec::{RunOptions, Stop};
use serde_json::json;

use common::{
    MODEL, assert_close, executor, executor_bb, first, force, history, logged, motion, plain_xml,
    toy, uid,
};

#[test]
fn breakpoint_before_start() {
    let mut ex = executor(
        r#"<Sequence>
             <Move name="a" robot="r1" secs="1"/>
             <Move name="b" robot="r1" secs="2"/>
           </Sequence>"#,
    );
    let b = uid(&ex, "b");
    ex.set_breakpoint(b, true);
    assert!(ex.breakpoint(b));
    assert_eq!(ex.breakpoints(), [b]);

    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![b]));
    assert_close(ex.now(), 1.0);
    assert_eq!(ex.status(b), Idle);
    assert!(history(&ex, "b").is_empty());
    assert_eq!(ex.paused_at(), [b]);
    assert_eq!(ex.halted(), Some(Stop::Breakpoint(vec![b])));
    assert_eq!(
        ex.waiting_reason(b).as_deref(),
        Some("paused before it starts")
    );
    assert_eq!(logged(&ex, "breakpoint"), 1);
    // Paused means paused: running again does nothing.
    let again = ex.run(RunOptions::VIRTUAL);
    assert_eq!((again.ticks, again.stop), (0, Stop::Breakpoint(vec![b])));

    // An edit made while paused is what b starts with.
    ex.set_port_value(b, "secs", "5").unwrap();
    ex.resume();
    assert!(ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 6.0);
    assert_close(motion(&ex, "b").unwrap().t0, 1.0);

    // It holds again on the next run.
    ex.reset();
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![b]));
    assert_close(ex.now(), 1.0);
    // Forcing the paused node decides it without starting it.
    force(&mut ex, "b", Success);
    assert!(ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 1.0);
    assert!(motion(&ex, "b").is_none());

    ex.clear_breakpoints();
    assert!(ex.breakpoints().is_empty());
}

#[test]
fn breakpoint_on_parallel_sibling() {
    let mut ex = executor(
        r#"<Parallel>
             <Move name="a" robot="r1" secs="3"/>
             <Move name="b" robot="r2" secs="1"/>
           </Parallel>"#,
    );
    let a = uid(&ex, "a");
    ex.set_breakpoint(a, true);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![a]));
    assert_close(ex.now(), 0.0);
    assert_close(motion(&ex, "b").unwrap().t0, 0.0);
    assert!(motion(&ex, "a").is_none());
    assert_eq!(ex.status(a), Idle);

    ex.resume_node(a);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(motion(&ex, "a").unwrap().t0, 0.0);
    assert_close(ex.now(), 3.0);
}

const REACTIVE: &str = r#"<ReactiveSequence>
    <C name="c"/>
    <Move name="m" robot="r1" secs="5"/>
  </ReactiveSequence>"#;

#[test]
fn breakpoint_in_reactive_sequence_pauses_once() {
    let mut ex = executor(REACTIVE);
    let c = uid(&ex, "c");
    ex.set_breakpoint(c, true);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![c]));
    assert_close(ex.now(), 0.0);
    assert!(motion(&ex, "m").is_none());
    ex.resume();
    // c is ticked again on every tick, but holds only once while the sequence runs, and m is
    // never restarted.
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 5.0);
    assert!(toy(&ex).halts.is_empty());
    assert_eq!(logged(&ex, "breakpoint"), 1);

    // Set while m runs: c holds at m's next event, and the hold does not halt m.
    let mut ex = executor(REACTIVE);
    assert_eq!(ex.run(RunOptions::until(1.0)).stop, Stop::Until);
    ex.set_breakpoint(c, true);
    assert!(!ex.retick_pending());
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![c]));
    assert_close(ex.now(), 5.0);
    assert_eq!(ex.status(uid(&ex, "m")), Running);
    assert!(toy(&ex).halts.is_empty());
    ex.resume();
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 5.0);
    assert!(toy(&ex).halts.is_empty());

    // Forcing the paused node uses up its hold too: c is ticked again, but does not pause again.
    let mut ex = executor(REACTIVE);
    ex.set_breakpoint(c, true);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![c]));
    force(&mut ex, "c", Success);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 5.0);
    assert_close(motion(&ex, "m").unwrap().t0, 0.0);
    assert_eq!(logged(&ex, "breakpoint"), 1);
}

#[test]
fn turning_a_breakpoint_off_starts_the_paused_node() {
    let mut ex = executor(
        r#"<Sequence>
             <Move name="a" robot="r1" secs="1"/>
             <Move name="b" robot="r1" secs="2"/>
           </Sequence>"#,
    );
    let b = uid(&ex, "b");
    ex.set_breakpoint(b, true);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![b]));
    ex.set_breakpoint(b, false);
    assert!(ex.retick_pending());
    assert!(ex.paused_at().is_empty());
    assert_eq!(ex.halted(), None);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(motion(&ex, "b").unwrap().t0, 1.0);
    assert_close(ex.now(), 3.0);
}

#[test]
fn breakpoint_rearms_each_time_its_parent_starts() {
    // b's parent, the Sequence, starts once per cycle: b holds in both.
    let mut ex = executor(
        r#"<Repeat num_cycles="2"><Sequence>
             <Move name="a" robot="r1" secs="1"/>
             <Move name="b" robot="r1" secs="1"/>
           </Sequence></Repeat>"#,
    );
    let b = uid(&ex, "b");
    ex.set_breakpoint(b, true);
    for at in [1.0, 3.0] {
        assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![b]));
        assert_close(ex.now(), at);
        ex.resume();
    }
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 4.0);

    // Directly under the loop, the parent runs throughout: only the first pass holds.
    let mut ex =
        executor(r#"<Repeat num_cycles="2"><Move name="b" robot="r1" secs="1"/></Repeat>"#);
    let b = uid(&ex, "b");
    ex.set_breakpoint(b, true);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Breakpoint(vec![b]));
    ex.resume();
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(ex.now(), 2.0);
    assert_eq!(toy(&ex).motions.len(), 2);
}

#[test]
fn port_and_bb_edits_persist_and_revert() {
    let blackboard = BTreeMap::from([("secs".to_string(), json!(2))]);
    let mut ex = executor_bb(
        r#"<Sequence><Move name="m" robot="r1" secs="{secs}"/></Sequence>"#,
        blackboard.clone(),
    );
    let m = uid(&ex, "m");
    let end = |ex: &mut bt_exec::Executor| {
        ex.reset();
        assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
        ex.now()
    };
    assert_close(end(&mut ex), 2.0);

    // A port edit survives a reset; setting the file's value again drops it.
    ex.set_port_value(m, "secs", "3").unwrap();
    assert_eq!(ex.port_original(m, "secs"), Some(Some("{secs}")));
    assert_eq!(ex.port_edits(), [(m, "secs".to_string())]);
    assert_close(end(&mut ex), 3.0);
    ex.set_port_value(m, "secs", "{secs}").unwrap();
    assert!(ex.port_edits().is_empty());
    assert_eq!(ex.port_original(m, "secs"), None);
    ex.set_port_value(m, "secs", "3").unwrap();
    ex.revert_port_value(m, "secs");
    assert_eq!(ex.tree().node(m).port_value("secs"), Some("{secs}"));
    assert_close(end(&mut ex), 2.0);

    // A port the node did not have is added, and removed again on revert.
    ex.set_port_value(m, "extra", "1").unwrap();
    assert_eq!(ex.port_original(m, "extra"), Some(None));
    assert!(ex.tree().node(m).port("extra").is_some_and(|p| !p.declared));
    ex.revert_port_value(m, "extra");
    assert!(ex.tree().node(m).port("extra").is_none());

    // Not ports.
    assert!(ex.set_port_value(0, "secs", "1").is_err());
    assert!(ex.set_port_value(999, "secs", "1").is_err());
    assert!(ex.set_port_value(m, "name", "x").is_err());
    assert!(ex.set_port_value(m, "_skipIf", "true").is_err());
    let subtree = format!(
        r#"<root BTCPP_format="4" main_tree_to_execute="T">
             <BehaviorTree ID="T"><SubTree ID="Job" name="job" v="{{x}}"/></BehaviorTree>
             <BehaviorTree ID="Job"><A name="inner"/></BehaviorTree>{MODEL}
           </root>"#
    );
    let mut sub = plain_xml(&subtree, BTreeMap::new());
    let job = uid(&sub, "job");
    assert!(sub.set_port_value(job, "v", "1").is_err());
    assert!(sub.set_port_value(uid(&sub, "inner"), "v", "1").is_ok());

    // Blackboard edits: `{secs}` 2 → 4 → 2.
    ex.set_blackboard_value("secs", json!(4));
    assert!(ex.blackboard_edited("secs"));
    assert_eq!(ex.blackboard_entry(m, "secs"), Some(json!(4)));
    assert_close(end(&mut ex), 4.0);
    ex.revert_blackboard_value("secs");
    assert!(!ex.blackboard_edited("secs"));
    assert_eq!(ex.blackboard_entry(m, "secs"), Some(json!(2)));
    assert_close(end(&mut ex), 2.0);
    ex.set_blackboard_value("new", json!(true));
    ex.revert_blackboard_value("new");
    assert_eq!(ex.blackboard_entry(m, "new"), None);

    // Revert all.
    ex.set_port_value(m, "secs", "7").unwrap();
    ex.set_blackboard_value("secs", json!(9));
    ex.revert_all_edits();
    assert!(ex.port_edits().is_empty() && ex.blackboard_edits().is_empty());
    assert_eq!(ex.initial_blackboard(), &blackboard);
    assert_close(end(&mut ex), 2.0);
}

#[test]
fn delay_fractional() {
    let mut ex = executor(r#"<Delay delay_msec="1500.5"><A name="a"/></Delay>"#);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_close(first(&ex, "a", Running).t, 1.5005);
    assert_close(ex.now(), 2.5005);
}
