//! Native leaves: a handler runs them, the clock follows their events.

mod common;

use std::collections::BTreeMap;

use bt_core::NodeKind;
use bt_core::NodeStatus::{self, Failure, Running, Success};
use bt_exec::{LeafCtx, LeafHandler, LeafPolicy, RunOptions, Stop};
use serde_json::{Value, json};

use common::{MODEL, assert_close, executor, first, plain_xml, tick_times, toy, uid};

const GATE_AND_MOVE: &str = r#"<Parallel>
    <Gate name="g" robot="r2" at="1"/>
    <Move name="m" robot="r2" secs="3"/>
  </Parallel>"#;

#[test]
fn retick_opens_gate_at_pass_time() {
    let mut ex = executor(GATE_AND_MOVE);
    let report = ex.run(RunOptions::VIRTUAL);
    assert_eq!(report.stop, Stop::Finished(Success));
    assert_eq!(report.ticks, 4);
    // The gate is ticked before m places its motion, so the tree is ticked again at 0 to see
    // it; without that re-tick the gate would only open with m at 3.
    assert_eq!(tick_times(&ex), [0.0, 0.0, 1.0, 3.0]);
    assert_close(first(&ex, "g", Success).t, 1.0);
    assert_close(ex.now(), 3.0);
}

#[test]
fn realtime_chunks_equal_virtual() {
    let mut virtual_run = executor(GATE_AND_MOVE);
    virtual_run.run(RunOptions::VIRTUAL);

    let mut ex = executor(GATE_AND_MOVE);
    for k in 1.. {
        let report = ex.run(RunOptions::until(f64::from(k) * 0.37));
        if let Stop::Finished(status) = report.stop {
            assert_eq!(status, Success);
            break;
        }
        assert_eq!(report.stop, Stop::Until);
        assert!(k < 100, "never finished");
    }
    // Chunks only move the clock between events: the same ticks at the same instants.
    assert_eq!(ex.trace().transitions(), virtual_run.trace().transitions());
    assert_eq!(toy(&ex).motions, toy(&virtual_run).motions);
}

#[test]
fn handler_reset_and_downcast() {
    let mut ex =
        executor(r#"<Sequence><Move name="m" robot="r1" secs="1"/><A name="a"/></Sequence>"#);
    assert!(ex.has_leaf_handler());
    assert_eq!(toy(&ex).resets, 1, "installing starts the run over");
    ex.reset();
    assert_eq!(toy(&ex).resets, 2);

    let (m, a) = (uid(&ex, "m"), uid(&ex, "a"));
    assert!(ex.is_handled(m) && !ex.is_handled(a));
    assert_eq!(ex.policy(m), LeafPolicy::Native);
    ex.set_policy(a, LeafPolicy::Native);
    assert_eq!(
        ex.policy(a),
        LeafPolicy::DEFAULT_ACTION,
        "only handled leaves run natively"
    );

    ex.run(RunOptions::VIRTUAL);
    assert_close(ex.now(), 2.0);
    assert_eq!(toy(&ex).motions.len(), 1);
    ex.leaf_handler_mut::<common::Toy>().unwrap().resets = 10;
    assert_eq!(toy(&ex).resets, 10);

    // Taking the handler back hands its leaves to the default stub.
    let handler = ex.take_leaf_handler().expect("installed");
    assert!(!ex.has_leaf_handler() && !ex.is_handled(m));
    assert_eq!(ex.policy(m), LeafPolicy::DEFAULT_ACTION);
    assert!(ex.leaf_handler::<common::Toy>().is_none());
    ex.run(RunOptions::VIRTUAL);
    assert_close(ex.now(), 2.0);
    assert_eq!(
        common::history(&ex, "m")[..2],
        [Running, Success],
        "a stub again"
    );
    ex.set_leaf_handler(handler);
    assert_eq!(ex.policy(m), LeafPolicy::Native);
    assert_eq!(ex.ticks(), 0);
}

#[test]
fn native_policy_reaches_only_handled_leaves() {
    let mut ex =
        executor(r#"<Sequence><Move name="m" robot="r1" secs="1"/><A name="a"/></Sequence>"#);
    let (m, a) = (uid(&ex, "m"), uid(&ex, "a"));
    ex.set_all_policies(NodeKind::Action, LeafPolicy::Manual);
    assert_eq!(
        (ex.policy(m), ex.policy(a)),
        (LeafPolicy::Manual, LeafPolicy::Manual)
    );
    ex.set_all_policies(NodeKind::Action, LeafPolicy::Native);
    assert_eq!(
        (ex.policy(m), ex.policy(a)),
        (LeafPolicy::Native, LeafPolicy::Manual)
    );
}

#[test]
fn finished_reaches_the_handler() {
    let mut ex =
        executor(r#"<Sequence><Move name="m" robot="r1" secs="1"/><AlwaysFailure/></Sequence>"#);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Failure));
    assert_eq!(toy(&ex).finished, [(0, Failure, 1.0)]);
    // The handler's line is logged for the root, before the executor's own.
    let lines: Vec<_> = ex
        .log()
        .iter()
        .map(|e| (e.uid, e.message.as_str()))
        .collect();
    let handler = lines.iter().position(|&l| l == (Some(0), "finished"));
    let tree = lines
        .iter()
        .position(|&(_, m)| m.starts_with("tree finished"));
    assert!(handler.is_some() && handler < tree, "{lines:?}");
}

/// What a `Probe` leaf saw when it started.
#[derive(Default)]
struct Probe {
    attribute: Option<String>,
    input: Option<Value>,
    entry: Option<Value>,
    wrote: Vec<bool>,
}

impl LeafHandler for Probe {
    fn handles(&self, id: &str, _kind: NodeKind) -> bool {
        id == "Probe"
    }

    /// Reads `speed` and `k`, writes `out` (and tries `speed`), then runs with nothing scheduled.
    fn on_start(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        self.attribute = cx.attribute("speed").map(str::to_string);
        self.input = cx.input("speed");
        self.entry = cx.entry("k");
        self.wrote = vec![
            cx.set_output("out", json!(7)),
            cx.set_output("speed", json!(1)),
        ];
        Running
    }

    fn on_running(&mut self, _cx: &mut LeafCtx<'_>) -> NodeStatus {
        Running
    }
}

#[test]
fn ctx_reads_ports_and_the_leaf_scope() {
    let xml = format!(
        r#"<root BTCPP_format="4" main_tree_to_execute="T">
             <BehaviorTree ID="T"><SubTree ID="Job" k="{{outer}}" out="{{result}}"/></BehaviorTree>
             <BehaviorTree ID="Job"><Probe name="p" out="{{out}}"/></BehaviorTree>{MODEL}
           </root>"#
    );
    let blackboard = BTreeMap::from([("outer".to_string(), json!(3))]);
    let mut ex = plain_xml(&xml, blackboard).with_leaf_handler(Box::new(Probe::default()));
    let p = uid(&ex, "p");
    // RUNNING with no wakeup and no reason of its own: a stall, with the executor's reason.
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Stalled(vec![p]));
    assert_eq!(
        ex.waiting_reason(p).as_deref(),
        Some("RUNNING with nothing scheduled to finish it")
    );
    let probe = ex.leaf_handler::<Probe>().unwrap();
    assert_eq!(
        probe.attribute, None,
        "no attribute, so not the model default"
    );
    assert_eq!(probe.input, Some(json!("2")), "the port reads the default");
    assert_eq!(
        probe.entry,
        Some(json!(3)),
        "`k` through the SubTree remapping"
    );
    assert_eq!(
        probe.wrote,
        [true, false],
        "only a `{{key}}` port is written"
    );
    assert_eq!(ex.blackboard_entry(0, "result"), Some(json!(7)));

    // An edit is what the attribute says.
    ex.set_port_value(p, "speed", "4").unwrap();
    ex.reset();
    ex.run(RunOptions::VIRTUAL);
    let probe = ex.leaf_handler::<Probe>().unwrap();
    assert_eq!(probe.attribute.as_deref(), Some("4"));
    assert_eq!(probe.input, Some(json!("4")));
}
