//! Native leaves: a [`LeafHandler`] gives stub leaves real semantics.
//!
//! The executor owns at most one handler, which claims leaves by model ID and keeps whatever
//! world they share (a motion schedule, say). A claimed leaf runs with
//! [`LeafPolicy::Native`](crate::LeafPolicy::Native): the executor calls [`LeafHandler::on_start`]
//! when it starts, [`LeafHandler::on_running`] on every later tick while it is RUNNING, and
//! [`LeafHandler::on_halted`] when it is stopped early. Each call gets a [`LeafCtx`] with the
//! node, the clock, its ports and the blackboard; timers, re-tick requests and log lines go back
//! through the context, so a handler never touches the executor itself.

use std::any::Any;

use bt_core::NodeStatus;
use bt_core::model::{Node, NodeKind, Port, Uid, blackboard_pointer};
use serde_json::Value;

use crate::blackboard::Blackboards;
use crate::executor::EPS;

/// Real semantics for some stub leaves.
///
/// `Any` lets the owner get its concrete handler back with
/// [`Executor::leaf_handler`](crate::Executor::leaf_handler).
pub trait LeafHandler: Any + Send {
    /// Whether this handler runs leaves of model `id`.
    fn handles(&self, id: &str, kind: NodeKind) -> bool;
    /// The leaf starts (it was not RUNNING natively).
    fn on_start(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus;
    /// The leaf is ticked again while RUNNING.
    fn on_running(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus;
    /// The leaf was RUNNING and is stopped before it finished.
    fn on_halted(&mut self, cx: &mut LeafCtx<'_>, cause: HaltCause) {
        let _ = (cx, cause);
    }
    /// The tree finished; `cx` is the root's (uid 0).
    fn on_finished(&mut self, cx: &mut LeafCtx<'_>, root: NodeStatus) {
        let _ = (cx, root);
    }
    /// The executor was reset: forget the run.
    fn reset(&mut self) {}
    /// Why a RUNNING leaf has not finished yet, for the "where it halts" list.
    fn waiting_reason(&self, uid: Uid) -> Option<String> {
        let _ = uid;
        None
    }
    /// The runtime these leaves come from. While the handler is installed, the builtins follow
    /// that runtime's rules where it differs from BehaviorTree.CPP.
    fn dialect(&self) -> Dialect {
        Dialect::BtCpp
    }
}

/// Whose rules the builtins follow where BehaviorTree.CPP and another runtime disagree.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Dialect {
    /// BehaviorTree.CPP 4.
    #[default]
    BtCpp,
    /// behaviortree-rs 0.2.5, factoryos cell-runner's runtime: its `SequenceStar` returns
    /// FAILURE once every child succeeded (`sequence_star.rs`), where BT.CPP's returns SUCCESS.
    BehaviortreeRs,
}

/// Why a RUNNING native leaf is stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HaltCause {
    /// Its parent halted it (a Parallel finished, a ReactiveSequence restarted, …).
    Tree,
    /// An override forced the leaf itself to this status.
    Forced(NodeStatus),
    /// An override forced this ancestor, which halts everything below it.
    AncestorForced(Uid),
    /// Its policy was switched away from `Native`.
    PolicyChanged,
}

/// What a handler asked the executor to do during one call.
#[derive(Debug, Default)]
pub(crate) struct LeafOut {
    pub wakeups: Vec<f64>,
    pub retick: bool,
    pub log: Vec<String>,
}

/// One leaf, as a handler sees it during a call.
pub struct LeafCtx<'a> {
    pub(crate) uid: Uid,
    pub(crate) node: &'a Node,
    pub(crate) now: f64,
    pub(crate) tick: u64,
    pub(crate) releasing: bool,
    pub(crate) blackboards: &'a mut Blackboards,
    pub(crate) out: &'a mut LeafOut,
}

impl<'a> LeafCtx<'a> {
    /// The leaf being called (0, the root, in [`LeafHandler::on_finished`]).
    pub fn uid(&self) -> Uid {
        self.uid
    }

    /// The node, borrowed for the whole call (it outlives `self`).
    pub fn node(&self) -> &'a Node {
        self.node
    }

    /// Executor time in seconds.
    pub fn now(&self) -> f64 {
        self.now
    }

    /// The tick being run (1-based).
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// The raw attribute (or a user edit of it), never the `<TreeNodesModel>` default.
    pub fn attribute(&self, port: &str) -> Option<&'a str> {
        let node = self.node;
        node.port(port)?.value.as_deref()
    }

    /// A blackboard entry as seen from this leaf's scope.
    pub fn entry(&self, key: &str) -> Option<Value> {
        self.blackboards
            .get(self.blackboards.scope_of[self.uid], key)
    }

    /// What a port reads, like [`Executor::port_value`](crate::Executor::port_value).
    pub fn input(&self, port: &str) -> Option<Value> {
        read_port(self.node, self.blackboards, port)
    }

    /// Write `value` to the blackboard entry the port points at; `false` if it is no `{key}`.
    pub fn set_output(&mut self, port: &str, value: Value) -> bool {
        let node = self.node;
        let Some(key) = node.port(port).and_then(Port::blackboard_key) else {
            return false;
        };
        let scope = self.blackboards.scope_of[self.uid];
        self.blackboards.set(scope, key, value);
        true
    }

    /// Ask to be ticked again at `t` (ignored unless it is a finite instant in the future: like
    /// cell-runner, a leaf that waits for `t = inf` stalls the tree).
    pub fn wake_at(&mut self, t: f64) {
        if t.is_finite() && t > self.now + EPS {
            self.out.wakeups.push(t);
        }
    }

    /// SUCCESS once the clock has reached `t`, else RUNNING with a wakeup at `t`.
    pub fn await_until(&mut self, t: f64) -> NodeStatus {
        if self.now + EPS >= t {
            NodeStatus::Success
        } else {
            self.wake_at(t);
            NodeStatus::Running
        }
    }

    /// Something changed that other leaves may react to at this very instant (new motion):
    /// the tree is ticked again before the clock moves (cell-runner's `Clock.moved`).
    pub fn retick(&mut self) {
        self.out.retick = true;
    }

    /// A line in the executor log, attributed to this leaf.
    pub fn log(&mut self, message: impl Into<String>) {
        self.out.log.push(message.into());
    }

    /// The tree stalled and waits that started before now should give up
    /// ([`Executor::release_stalled`](crate::Executor::release_stalled)).
    pub fn releasing(&self) -> bool {
        self.releasing
    }
}

/// A port's value with `{key}` resolved through the node's blackboard scope.
pub(crate) fn read_port(node: &Node, blackboards: &Blackboards, port: &str) -> Option<Value> {
    let raw = node.port_value(port)?;
    match blackboard_pointer(raw) {
        Some(key) => blackboards.get(blackboards.scope_of[node.uid], key),
        None => Some(Value::String(raw.to_string())),
    }
}

/// The concrete handler behind `handler`, if it is a `T` (trait upcasting to `Any`).
pub(crate) fn downcast<T: LeafHandler>(handler: &dyn LeafHandler) -> Option<&T> {
    let any: &dyn Any = handler;
    any.downcast_ref()
}

pub(crate) fn downcast_mut<T: LeafHandler>(handler: &mut dyn LeafHandler) -> Option<&mut T> {
    let any: &mut dyn Any = handler;
    any.downcast_mut()
}
