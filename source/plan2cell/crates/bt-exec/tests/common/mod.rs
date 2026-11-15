//! Helpers shared by the leaf-handler tests: trees over a small model, and `Toy`, a leaf
//! handler with timed motions and gates that open when a robot's motion passes a point.
#![allow(dead_code)]

use std::collections::BTreeMap;

use bt_core::{Document, NodeKind, NodeStatus, Transition, Uid};
use bt_exec::script;
use bt_exec::{EPS, Executor, HaltCause, LeafCtx, LeafHandler, Override};
use serde_json::Value;

/// `Move robot secs`, `Gate robot at` (native under `Toy`), the plain stub `A`, condition `C`,
/// and `Probe speed out` (a port with a model default, and an output).
pub const MODEL: &str = r#"<TreeNodesModel>
  <Action ID="Move"><input_port name="robot"/><input_port name="secs"/></Action>
  <Action ID="Gate"><input_port name="robot"/><input_port name="at"/></Action>
  <Action ID="Probe"><input_port name="speed" default="2"/><output_port name="out"/></Action>
  <Action ID="A"/>
  <Condition ID="C"/>
</TreeNodesModel>"#;

/// `body` as the only BehaviorTree, `T`, with [`MODEL`].
pub fn xml(body: &str) -> String {
    format!(r#"<root BTCPP_format="4"><BehaviorTree ID="T">{body}</BehaviorTree>{MODEL}</root>"#)
}

/// Main tree `T` of `xml`, without a leaf handler.
pub fn plain_xml(xml: &str, blackboard: BTreeMap<String, Value>) -> Executor {
    let tree = Document::parse(xml).unwrap().instantiate("T").unwrap();
    Executor::new(tree, blackboard)
}

pub fn plain(body: &str) -> Executor {
    plain_xml(&xml(body), BTreeMap::new())
}

/// `body` with [`Toy`] installed.
pub fn executor(body: &str) -> Executor {
    executor_bb(body, BTreeMap::new())
}

pub fn executor_bb(body: &str, blackboard: BTreeMap<String, Value>) -> Executor {
    plain_xml(&xml(body), blackboard).with_leaf_handler(Box::new(Toy::default()))
}

pub fn uid(ex: &Executor, name: &str) -> Uid {
    ex.tree()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named {name}"))
        .uid
}

/// Statuses `name` went through, in order.
pub fn history(ex: &Executor, name: &str) -> Vec<NodeStatus> {
    ex.trace()
        .of_node(uid(ex, name))
        .map(|t| t.status)
        .collect()
}

/// The first transition of `name` to `status`.
pub fn first(ex: &Executor, name: &str, status: NodeStatus) -> Transition {
    *ex.trace()
        .of_node(uid(ex, name))
        .find(|t| t.status == status)
        .unwrap_or_else(|| panic!("{name} never went {status}"))
}

pub fn force(ex: &mut Executor, name: &str, status: NodeStatus) {
    let uid = uid(ex, name);
    ex.set_override(
        uid,
        Some(Override {
            status,
            pinned: false,
        }),
    );
}

/// When each tick so far ran.
pub fn tick_times(ex: &Executor) -> Vec<f64> {
    (1..=ex.ticks())
        .map(|k| ex.time_at_tick(k).unwrap())
        .collect()
}

pub fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-6,
        "{actual} is not {expected}"
    );
}

pub fn toy(ex: &Executor) -> &Toy {
    ex.leaf_handler::<Toy>().expect("Toy is installed")
}

/// The latest motion `name` started.
pub fn motion(ex: &Executor, name: &str) -> Option<Motion> {
    let uid = uid(ex, name);
    toy(ex).motions.iter().rev().find(|m| m.uid == uid).cloned()
}

pub fn logged(ex: &Executor, needle: &str) -> usize {
    ex.log()
        .iter()
        .filter(|e| e.message.contains(needle))
        .count()
}

#[derive(Clone, Debug, PartialEq)]
pub struct Motion {
    pub uid: Uid,
    pub robot: String,
    pub t0: f64,
    pub t1: f64,
}

/// A stand-in for the cell leaves.
///
/// - `Move robot secs`: places a motion `[now, now + secs]`, asks for a re-tick, and finishes
///   when the motion ends. A halt cuts the motion at that instant.
/// - `Gate robot at`: opens `at` seconds into the latest motion of `robot`. With no motion it
///   stays RUNNING with nothing scheduled, until a stall release lets it go.
/// - When the tree finishes it records `(uid, root status, now)` and logs "finished".
#[derive(Debug, Default)]
pub struct Toy {
    pub motions: Vec<Motion>,
    pub halts: Vec<(Uid, f64, HaltCause)>,
    pub finished: Vec<(Uid, NodeStatus, f64)>,
    pub resets: u32,
    /// Gates waiting, and for which robot.
    gates: BTreeMap<Uid, String>,
}

fn text(cx: &LeafCtx<'_>, port: &str) -> String {
    cx.input(port)
        .map(|v| script::display(&v))
        .unwrap_or_default()
}

fn number(cx: &LeafCtx<'_>, port: &str) -> f64 {
    cx.input(port)
        .and_then(|v| script::number(&v))
        .unwrap_or(0.0)
}

impl Toy {
    fn gate(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let robot = text(cx, "robot");
        let opens = self
            .motions
            .iter()
            .rev()
            .find(|m| m.robot == robot)
            .map(|m| m.t0 + number(cx, "at"));
        if opens.is_some_and(|t| cx.now() + EPS >= t) {
            self.gates.remove(&cx.uid());
            return NodeStatus::Success;
        }
        if cx.releasing() {
            cx.log("released");
            self.gates.remove(&cx.uid());
            return NodeStatus::Success;
        }
        if let Some(t) = opens {
            cx.wake_at(t);
        }
        self.gates.insert(cx.uid(), robot);
        NodeStatus::Running
    }
}

impl LeafHandler for Toy {
    fn handles(&self, id: &str, _kind: NodeKind) -> bool {
        matches!(id, "Move" | "Gate")
    }

    fn on_start(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        if cx.node().id != "Move" {
            return self.gate(cx);
        }
        let t0 = cx.now();
        let t1 = t0 + number(cx, "secs");
        self.motions.push(Motion {
            uid: cx.uid(),
            robot: text(cx, "robot"),
            t0,
            t1,
        });
        cx.retick();
        cx.await_until(t1)
    }

    fn on_running(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        if cx.node().id != "Move" {
            return self.gate(cx);
        }
        let uid = cx.uid();
        let t1 = self
            .motions
            .iter()
            .rev()
            .find(|m| m.uid == uid)
            .map_or(cx.now(), |m| m.t1);
        cx.await_until(t1)
    }

    fn on_halted(&mut self, cx: &mut LeafCtx<'_>, cause: HaltCause) {
        let (uid, now) = (cx.uid(), cx.now());
        if let Some(m) = self.motions.iter_mut().rev().find(|m| m.uid == uid) {
            m.t1 = m.t1.min(now);
        }
        self.gates.remove(&uid);
        self.halts.push((uid, now, cause));
    }

    fn on_finished(&mut self, cx: &mut LeafCtx<'_>, root: NodeStatus) {
        self.finished.push((cx.uid(), root, cx.now()));
        cx.log("finished");
    }

    fn reset(&mut self) {
        self.resets += 1;
        self.motions.clear();
        self.halts.clear();
        self.finished.clear();
        self.gates.clear();
    }

    fn waiting_reason(&self, uid: Uid) -> Option<String> {
        self.gates
            .get(&uid)
            .map(|robot| format!("waiting for {robot}"))
    }
}
