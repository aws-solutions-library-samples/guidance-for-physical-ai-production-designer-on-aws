//! Ticking a [`Tree`] the way BehaviorTree.CPP 4 does, recording every status change.
//!
//! Each node runs through `execute_tick` (BT.CPP `TreeNode::executeTick`: pre-conditions,
//! `tick()`, post-conditions, `setStatus`), and every `set_status` that changes something is
//! appended to the [`Trace`] — the same stream BT.CPP's `StatusChangeLogger` feeds Groot.
//!
//! Leaves that BT.CPP does not provide are stubs driven by a [`LeafPolicy`], or run natively by
//! an installed [`LeafHandler`]. Any node but the root can be forced to SUCCESS or FAILURE with
//! an [`Override`]; a forced control node or decorator halts what runs below it. A breakpoint
//! holds a node before it starts ([`Executor::set_breakpoint`]), and [`Executor::run`] drives
//! the clock from event to event.

use std::collections::BTreeMap;

use bt_core::model::{NodeKind, Tree, Uid, blackboard_pointer};
use bt_core::{NodeStatus, Trace, Transition};
use serde_json::Value;

use crate::blackboard::Blackboards;
use crate::leaf::{self, Dialect, HaltCause, LeafCtx, LeafHandler, LeafOut};
use crate::script::{self, Env};

mod edits;
mod run;

pub use run::{MAX_STALL_RELEASES, RunOptions, RunReport, Stop};

/// How often a `Repeat`/`Retry` of an instant child loops within one tick. A bounded loop goes
/// on in another tick at the same instant; an endless one waits for the next tick.
const MAX_LOOPS_PER_TICK: u32 = 10_000;

/// Two instants closer than this are the same instant.
pub const EPS: f64 = 1e-9;

/// What a stub leaf does when ticked.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LeafPolicy {
    /// Actions: RUNNING for `after_s` seconds of executor time, then SUCCESS (0 = at once).
    /// Conditions: SUCCESS.
    Succeed { after_s: f64 },
    /// Like `Succeed`, but FAILURE.
    Fail { after_s: f64 },
    /// Actions: RUNNING until overridden. Conditions behave like `Succeed`.
    Manual,
    /// Run the installed [`LeafHandler`] ("simulate"); only leaves it handles accept it.
    Native,
}

impl LeafPolicy {
    pub const DEFAULT_ACTION: Self = Self::Succeed { after_s: 1.0 };
    pub const DEFAULT_CONDITION: Self = Self::Succeed { after_s: 0.0 };

    /// The stub policy a leaf of `kind` starts with.
    fn default_for(kind: NodeKind) -> Self {
        match kind {
            NodeKind::Condition => Self::DEFAULT_CONDITION,
            _ => Self::DEFAULT_ACTION,
        }
    }
}

/// A status forced onto a node the next time it is ticked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Override {
    pub status: NodeStatus,
    /// Keep applying it, instead of only once.
    pub pinned: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LogEntry {
    pub tick: u64,
    pub t: f64,
    pub uid: Option<Uid>,
    pub message: String,
}

/// An instant some node asked to be ticked again at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wakeup {
    /// When to tick.
    pub t: f64,
    /// Who asked.
    pub uid: Uid,
}

/// Per-node memory between ticks (`current_child_idx_`, counters, timers, …).
#[derive(Clone, Debug, Default)]
struct NodeState {
    status: NodeStatus,
    child_idx: usize,
    skipped: usize,
    counter: i64,
    /// Parallel: children already finished, and how.
    completed: Vec<Option<NodeStatus>>,
    /// Leaf/Delay/Timeout/Sleep: when it started; RunOnce: whether it ran.
    started_at: Option<f64>,
    /// RunOnce's first result.
    stored: Option<NodeStatus>,
    /// Precondition: the child is running, don't re-check.
    child_running: bool,
    /// Parallel: children that succeeded / failed so far.
    successes: usize,
    failures: usize,
    /// Switch: the child that is running.
    running_child: Option<usize>,
    /// Leaf: RUNNING in the leaf handler (it gets `on_running`, and `on_halted` if stopped).
    native: bool,
    /// Native leaf: the tick it started in (for stall releases).
    started_tick: u64,
}

pub struct Executor {
    tree: Tree,
    states: Vec<NodeState>,
    blackboards: Blackboards,
    initial_blackboard: BTreeMap<String, Value>,
    policies: Vec<LeafPolicy>,
    overrides: Vec<Option<Override>>,
    trace: Trace,
    log: Vec<LogEntry>,
    tick: u64,
    now: f64,
    /// Instants nodes want to be ticked again at, registered during the last tick (every
    /// RUNNING timer registers again each time it is ticked).
    wakeups: Vec<Wakeup>,
    /// When each tick ran: `tick_times[k - 1]` for tick `k`.
    tick_times: Vec<f64>,
    /// The handler for native leaves, and which leaves it claims.
    leaves: Option<Box<dyn LeafHandler>>,
    handled: Vec<bool>,
    /// A native leaf changed something others react to, or a bounded loop ran out of iterations
    /// for one tick: tick again before the clock moves.
    retick: bool,
    /// A user change affects a node the next tick reaches: tick again before the clock moves.
    dirty: bool,
    /// cell-runner's stall release: native waits started before this tick give up.
    release_before: Option<u64>,
    /// Why `halt` is stopping nodes right now.
    halt_cause: HaltCause,
    /// Breakpoints, by uid.
    breakpoints: Vec<bool>,
    /// Resumed: the next tick lets the node past its breakpoint.
    pass: Vec<bool>,
    /// The breakpoint already held once during this activation of the node's parent.
    bp_spent: Vec<bool>,
    /// Nodes held at their breakpoint by the last tick, and by the tick before.
    paused: Vec<Uid>,
    paused_prev: Vec<Uid>,
    /// Holds so far; a reactive control compares it to tell "held" from "running".
    holds: u64,
    /// Edited ports: the value from the file (`None`: the port was added by the edit).
    port_originals: BTreeMap<(Uid, String), Option<String>>,
    /// Main-blackboard edits, applied again on every reset.
    bb_edits: BTreeMap<String, Value>,
}

impl Executor {
    pub fn new(tree: Tree, initial_blackboard: BTreeMap<String, Value>) -> Self {
        let n = tree.len();
        let policies = tree
            .nodes
            .iter()
            .map(|node| LeafPolicy::default_for(node.kind))
            .collect();
        let mut blackboards = Blackboards::new(&tree);
        blackboards.reset(&initial_blackboard);
        Self {
            states: vec![NodeState::default(); n],
            blackboards,
            initial_blackboard,
            policies,
            overrides: vec![None; n],
            trace: Trace::new(n),
            log: Vec::new(),
            tick: 0,
            now: 0.0,
            wakeups: Vec::new(),
            tick_times: Vec::new(),
            leaves: None,
            handled: vec![false; n],
            retick: false,
            dirty: false,
            release_before: None,
            halt_cause: HaltCause::Tree,
            breakpoints: vec![false; n],
            pass: vec![false; n],
            bp_spent: vec![false; n],
            paused: Vec::new(),
            paused_prev: Vec::new(),
            holds: 0,
            port_originals: BTreeMap::new(),
            bb_edits: BTreeMap::new(),
            tree,
        }
    }

    /// Run the leaves `handler` claims natively (their policy becomes [`LeafPolicy::Native`]),
    /// replacing any handler installed before. The run starts over.
    pub fn set_leaf_handler(&mut self, handler: Box<dyn LeafHandler>) {
        self.unclaim_leaves();
        for node in &self.tree.nodes {
            let claimed = is_stub(&node.id, node.kind) && handler.handles(&node.id, node.kind);
            if claimed {
                self.handled[node.uid] = true;
                self.policies[node.uid] = LeafPolicy::Native;
            }
        }
        self.leaves = Some(handler);
        self.reset();
    }

    /// [`set_leaf_handler`](Self::set_leaf_handler), builder style.
    pub fn with_leaf_handler(mut self, handler: Box<dyn LeafHandler>) -> Self {
        self.set_leaf_handler(handler);
        self
    }

    /// Remove the handler; its leaves go back to the default stub policies. The run starts over.
    pub fn take_leaf_handler(&mut self) -> Option<Box<dyn LeafHandler>> {
        self.unclaim_leaves();
        let handler = self.leaves.take();
        self.reset();
        handler
    }

    fn unclaim_leaves(&mut self) {
        for node in &self.tree.nodes {
            if std::mem::take(&mut self.handled[node.uid])
                && self.policies[node.uid] == LeafPolicy::Native
            {
                self.policies[node.uid] = LeafPolicy::default_for(node.kind);
            }
        }
    }

    /// Whether a leaf handler is installed.
    pub fn has_leaf_handler(&self) -> bool {
        self.leaves.is_some()
    }

    /// Whether the installed handler claims this leaf.
    pub fn is_handled(&self, uid: Uid) -> bool {
        self.handled[uid]
    }

    /// Whose rules the builtins follow: the installed handler's runtime, else BT.CPP's.
    pub fn dialect(&self) -> Dialect {
        self.leaves.as_ref().map_or(Dialect::BtCpp, |h| h.dialect())
    }

    /// The installed handler, if it is a `T`.
    pub fn leaf_handler<T: LeafHandler>(&self) -> Option<&T> {
        self.leaves.as_deref().and_then(leaf::downcast)
    }

    /// The installed handler, if it is a `T`. Changing its options does not re-tick; they
    /// take effect at the next event.
    pub fn leaf_handler_mut<T: LeafHandler>(&mut self) -> Option<&mut T> {
        self.leaves.as_deref_mut().and_then(leaf::downcast_mut)
    }

    pub fn tree(&self) -> &Tree {
        &self.tree
    }

    pub fn trace(&self) -> &Trace {
        &self.trace
    }

    pub fn log(&self) -> &[LogEntry] {
        &self.log
    }

    /// Ticks issued so far.
    pub fn ticks(&self) -> u64 {
        self.tick
    }

    /// Executor time in seconds.
    pub fn now(&self) -> f64 {
        self.now
    }

    pub fn status(&self, uid: Uid) -> NodeStatus {
        self.states[uid].status
    }

    pub fn root_status(&self) -> NodeStatus {
        self.states[0].status
    }

    /// Whether the tree has run and finished.
    pub fn is_done(&self) -> bool {
        self.tick > 0 && self.root_status() != NodeStatus::Running
    }

    pub fn policy(&self, uid: Uid) -> LeafPolicy {
        self.policies[uid]
    }

    /// Change how a stub leaf runs. `Native` is ignored unless the handler claims the leaf.
    pub fn set_policy(&mut self, uid: Uid, policy: LeafPolicy) {
        if policy == LeafPolicy::Native && !self.handled[uid] {
            return;
        }
        if self.policies[uid] != policy {
            self.policies[uid] = policy;
            self.touch(uid);
        }
    }

    /// Set the policy of every stub leaf of this kind (`Native`: every handled one).
    pub fn set_all_policies(&mut self, kind: NodeKind, policy: LeafPolicy) {
        for uid in 0..self.tree.len() {
            let node = &self.tree.nodes[uid];
            if node.kind == kind && is_stub(&node.id, kind) {
                self.set_policy(uid, policy);
            }
        }
    }

    pub fn override_of(&self, uid: Uid) -> Option<Override> {
        self.overrides[uid]
    }

    /// Force a node to finish with `status` next time it is ticked (`None` clears it).
    ///
    /// A control node or decorator halts its children first. The root (uid 0) is not a node
    /// of the tree and cannot be forced.
    pub fn set_override(&mut self, uid: Uid, value: Option<Override>) {
        if uid == 0 || self.overrides[uid] == value {
            return;
        }
        self.overrides[uid] = value;
        if value.is_some() {
            self.touch(uid);
        }
    }

    /// Drop every override, pinned or not.
    pub fn clear_overrides(&mut self) {
        self.overrides.fill(None);
    }

    /// Running stub leaves that only an override will finish.
    pub fn waiting_on(&self) -> Vec<Uid> {
        self.tree
            .nodes
            .iter()
            .filter(|n| self.waits_for_user(n.uid))
            .map(|n| n.uid)
            .collect()
    }

    /// A RUNNING `Manual` stub action without an override queued.
    fn waits_for_user(&self, uid: Uid) -> bool {
        let n = &self.tree.nodes[uid];
        n.kind == NodeKind::Action
            && self.states[uid].status == NodeStatus::Running
            && is_stub(&n.id, n.kind)
            && self.policies[uid] == LeafPolicy::Manual
            && self.overrides[uid].is_none()
    }

    /// The earliest future instant a timer (stub leaf, `Sleep`, `Delay`, `Timeout`, a native
    /// leaf) changes something.
    pub fn next_wakeup(&self) -> Option<f64> {
        self.wakeups
            .iter()
            .map(|w| w.t)
            .filter(|t| *t > self.now + EPS)
            .min_by(f64::total_cmp)
    }

    /// The wakeups registered by the last tick.
    pub fn wakeups(&self) -> &[Wakeup] {
        &self.wakeups
    }

    /// The earliest future wakeup `uid` registered itself.
    pub fn wakeup_of(&self, uid: Uid) -> Option<f64> {
        self.wakeups
            .iter()
            .filter(|w| w.uid == uid && w.t > self.now + EPS)
            .map(|w| w.t)
            .min_by(f64::total_cmp)
    }

    /// Whether the tree must be ticked again before the clock may move (new motion, or a
    /// user change to a node the next tick reaches: a RUNNING or paused one, or one its parent
    /// re-ticks every tick, like the conditions of a running `ReactiveSequence`).
    pub fn retick_pending(&self) -> bool {
        self.dirty || self.retick
    }

    /// When tick `tick` ran (tick 0 = before the first one, at 0).
    pub fn time_at_tick(&self, tick: u64) -> Option<f64> {
        match tick {
            0 => Some(0.0),
            k => self.tick_times.get(k as usize - 1).copied(),
        }
    }

    /// The last tick that ran at or before `t` (0 if none did).
    pub fn tick_at_time(&self, t: f64) -> u64 {
        self.tick_times.partition_point(|&x| x <= t + EPS) as u64
    }

    /// Main-blackboard and SubTree entries: `(scope owner, key, value)`.
    pub fn blackboard(&self) -> Vec<(Option<Uid>, &str, &Value)> {
        self.blackboards
            .entries
            .iter()
            .enumerate()
            .flat_map(|(scope, entries)| {
                let owner = self.blackboards.scopes[scope].owner;
                entries.iter().map(move |(k, v)| (owner, k.as_str(), v))
            })
            .collect()
    }

    /// A blackboard entry as `uid` sees it (through its SubTree remapping).
    pub fn blackboard_entry(&self, uid: Uid, key: &str) -> Option<Value> {
        self.blackboards.get(self.blackboards.scope_of[uid], key)
    }

    /// What a port of a node reads right now (`{key}` resolved through the blackboards).
    pub fn port_value(&self, uid: Uid, port: &str) -> Option<Value> {
        leaf::read_port(&self.tree.nodes[uid], &self.blackboards, port)
    }

    /// Back to the start: everything IDLE, blackboard re-seeded (with the edits), trace and log
    /// cleared. Policies, pinned overrides, breakpoints, edits and the leaf handler stay.
    pub fn reset(&mut self) {
        let n = self.tree.len();
        self.states = vec![NodeState::default(); n];
        let mut seed = self.initial_blackboard.clone();
        seed.extend(self.bb_edits.clone());
        self.blackboards.reset(&seed);
        self.trace = Trace::new(n);
        self.log.clear();
        self.tick = 0;
        self.now = 0.0;
        self.tick_times.clear();
        self.wakeups.clear();
        self.retick = false;
        self.dirty = false;
        self.release_before = None;
        self.halt_cause = HaltCause::Tree;
        self.paused.clear();
        self.paused_prev.clear();
        self.pass.fill(false);
        self.bp_spent.fill(false);
        for o in &mut self.overrides {
            if o.is_some_and(|o| !o.pinned) {
                *o = None;
            }
        }
        if let Some(handler) = &mut self.leaves {
            handler.reset();
        }
    }

    /// Advance the clock to `now` and tick the tree once (BT.CPP `tickOnce`).
    ///
    /// A finished tree starts over, like ticking a BT.CPP tree again.
    pub fn tick_at(&mut self, now: f64) -> NodeStatus {
        self.advance_clock(now);
        self.tick += 1;
        self.tick_times.push(self.now);
        // Every RUNNING timer registers again while it is ticked (cell-runner's `wakes.clear()`),
        // so a node halted since cannot wake the tree.
        self.wakeups.clear();
        self.paused_prev = std::mem::take(&mut self.paused);
        self.retick = false;
        self.dirty = false;
        let Some(&first) = self.tree.nodes[0].children.first() else {
            self.message(None, "the tree is empty".to_string());
            return NodeStatus::Failure;
        };
        if self.states[0].status != NodeStatus::Running {
            self.set_status(0, NodeStatus::Running);
        }
        let status = self.execute_tick(first);
        let root = match status {
            NodeStatus::Running => NodeStatus::Running,
            // A tree whose root was skipped counts as done.
            NodeStatus::Skipped | NodeStatus::Idle => NodeStatus::Success,
            done => done,
        };
        self.set_status(0, root);
        if root != NodeStatus::Running {
            self.call_leaf(0, |h, cx| h.on_finished(cx, root));
            // BT.CPP leaves a finished tree's root in its final state until ticked again.
            self.message(None, format!("tree finished: {root}"));
        }
        root
    }

    /// Tick once without moving the clock.
    pub fn tick(&mut self) -> NodeStatus {
        self.tick_at(self.now)
    }

    // ───────────────────────────── node machinery ─────────────────────────────

    fn set_status(&mut self, uid: Uid, status: NodeStatus) {
        let prev = self.states[uid].status;
        if prev == status {
            return;
        }
        self.states[uid].status = status;
        if prev == NodeStatus::Running && !self.tree.nodes[uid].kind.is_leaf() {
            // This activation is over: breakpoints below hold again next time it runs.
            for &child in &self.tree.nodes[uid].children {
                self.bp_spent[child] = false;
            }
        }
        self.trace.push(Transition {
            tick: self.tick,
            t: self.now,
            uid,
            prev,
            status,
        });
    }

    fn message(&mut self, uid: Option<Uid>, message: String) {
        self.log.push(LogEntry {
            tick: self.tick,
            t: self.now,
            uid,
            message,
        });
    }

    /// Ask for a tick at `t` on behalf of `uid` (only finite future instants count: a timer
    /// that ends at `inf` never wakes the tree, so the run stalls like cell-runner's).
    fn wake(&mut self, uid: Uid, t: f64) {
        if t.is_finite()
            && t > self.now + EPS
            && !self
                .wakeups
                .iter()
                .any(|w| w.uid == uid && (w.t - t).abs() <= EPS)
        {
            self.wakeups.push(Wakeup { t, uid });
        }
    }

    /// Move the clock forward to `t` (never back). A stall release lasts until it moves.
    fn advance_clock(&mut self, t: f64) {
        if t > self.now + EPS {
            self.release_before = None;
        }
        self.now = self.now.max(t);
    }

    /// A user change to `uid`: re-tick at this instant only if a tick now could turn out
    /// differently, i.e. the next tick reaches the node. Anything else waits for the next event,
    /// so a change between events cannot open a gate early.
    fn touch(&mut self, uid: Uid) {
        if self.ticked_next(uid) {
            self.dirty = true;
        }
    }

    /// Whether the next tick reaches `uid`, as far as the state tells without ticking: it is
    /// RUNNING or paused, or its parent ticks it again although it does not run — a reactive
    /// control's children up to the one that runs, `WhileDoElse`'s condition,
    /// `KeepRunningUntilFailure`'s child, and any child of a node that starts afresh.
    fn ticked_next(&self, uid: Uid) -> bool {
        let live =
            |u: Uid| self.states[u].status == NodeStatus::Running || self.paused.contains(&u);
        if live(uid) {
            return true;
        }
        let Some(parent) = self.tree.nodes[uid].parent.filter(|&p| p != 0) else {
            return false;
        };
        if self.paused.contains(&parent) {
            // Held at its breakpoint: what is below it is not ticked.
            return false;
        }
        if self.states[parent].status != NodeStatus::Running {
            // It starts afresh if it is reached, and earlier children may finish at once.
            return self.ticked_next(parent);
        }
        let node = &self.tree.nodes[parent];
        let index = node.children.iter().position(|&c| c == uid).unwrap_or(0);
        match node.id.as_str() {
            // Every tick starts from the first child and stops at the one that runs.
            "ReactiveSequence" | "ReactiveFallback" => {
                !node.children[..index].iter().any(|&c| live(c))
            }
            "WhileDoElse" => index == 0,
            "KeepRunningUntilFailure" => true,
            _ => false,
        }
    }

    /// Call the leaf handler for `uid` and apply what it asked for (wakeups, re-tick, log).
    /// `None` without a handler.
    fn call_leaf<R>(
        &mut self,
        uid: Uid,
        f: impl FnOnce(&mut dyn LeafHandler, &mut LeafCtx<'_>) -> R,
    ) -> Option<R> {
        let releasing = self
            .release_before
            .is_some_and(|k| self.states[uid].started_tick < k);
        let handler = self.leaves.as_deref_mut()?;
        let mut out = LeafOut::default();
        let result = {
            let mut cx = LeafCtx {
                uid,
                node: &self.tree.nodes[uid],
                now: self.now,
                tick: self.tick,
                releasing,
                blackboards: &mut self.blackboards,
                out: &mut out,
            };
            f(handler, &mut cx)
        };
        for t in out.wakeups {
            self.wake(uid, t);
        }
        self.retick |= out.retick;
        for line in out.log {
            self.message(Some(uid), line);
        }
        Some(result)
    }

    /// Stop a leaf that is RUNNING in the handler.
    fn halt_native(&mut self, uid: Uid, cause: HaltCause) {
        if std::mem::take(&mut self.states[uid].native) {
            // A stub policy that takes over later starts its own timer.
            self.states[uid].started_at = None;
            self.call_leaf(uid, |h, cx| h.on_halted(cx, cause));
        }
    }

    fn is_active(&self, uid: Uid) -> bool {
        !matches!(
            self.states[uid].status,
            NodeStatus::Idle | NodeStatus::Skipped
        )
    }

    /// BT.CPP `TreeNode::executeTick`.
    fn execute_tick(&mut self, uid: Uid) -> NodeStatus {
        let status = match self.check_preconditions(uid) {
            Some(status) => status,
            // Held at a breakpoint: not ticked, no post-conditions, and it stays IDLE.
            None if self.holds_at_breakpoint(uid) => return NodeStatus::Running,
            None => self.tick_node(uid),
        };
        self.check_postconditions(uid, status);
        // BT.CPP keeps a skipped node IDLE; we show it as SKIPPED until it is reset.
        self.set_status(uid, status);
        status
    }

    fn check_preconditions(&mut self, uid: Uid) -> Option<NodeStatus> {
        let node = &self.tree.nodes[uid];
        if node.conditions.is_empty() {
            return None;
        }
        let conditions = node.conditions.clone();
        let get = |name: &str| {
            conditions
                .iter()
                .find(|(k, _)| k == name)
                .map(|(_, v)| v.clone())
        };
        if !self.is_active(uid) {
            for (name, result) in [
                ("_failureIf", NodeStatus::Failure),
                ("_successIf", NodeStatus::Success),
                ("_skipIf", NodeStatus::Skipped),
            ] {
                if let Some(code) = get(name)
                    && self.eval_bool(uid, &code)
                {
                    return Some(result);
                }
            }
            if let Some(code) = get("_while")
                && !self.eval_bool(uid, &code)
            {
                return Some(NodeStatus::Skipped);
            }
        } else if self.states[uid].status == NodeStatus::Running
            && let Some(code) = get("_while")
            && !self.eval_bool(uid, &code)
        {
            self.halt(uid);
            return Some(NodeStatus::Skipped);
        }
        None
    }

    fn check_postconditions(&mut self, uid: Uid, status: NodeStatus) {
        let node = &self.tree.nodes[uid];
        if node.conditions.is_empty() {
            return;
        }
        let names: &[&str] = match status {
            NodeStatus::Success => &["_onSuccess", "_post"],
            NodeStatus::Failure => &["_onFailure", "_post"],
            _ => return,
        };
        let scripts: Vec<String> = names
            .iter()
            .filter_map(|n| node.condition(n).map(str::to_string))
            .collect();
        for code in scripts {
            self.run_script(uid, &code);
        }
    }

    /// BT.CPP `haltNode` + `resetStatus`: halt what runs below, then back to IDLE.
    fn halt(&mut self, uid: Uid) {
        self.halt_native(uid, self.halt_cause);
        if self.states[uid].status == NodeStatus::Running {
            for child in self.tree.nodes[uid].children.clone() {
                self.halt(child);
            }
            if let Some(code) = self.tree.nodes[uid]
                .condition("_onHalted")
                .map(str::to_string)
            {
                self.run_script(uid, &code);
            }
        }
        // RunOnce remembers that it ran across halts, like BT.CPP's `already_ticked_`.
        let old = std::mem::take(&mut self.states[uid]);
        self.states[uid] = NodeState {
            status: old.status,
            stored: old.stored,
            ..Default::default()
        };
        // A node held at its breakpoint is no longer paused once its parent stops it.
        self.pass[uid] = false;
        self.paused.retain(|&p| p != uid);
        self.set_status(uid, NodeStatus::Idle);
    }

    /// `ControlNode::haltChildren(from)` / `resetChildren`.
    fn halt_children_from(&mut self, uid: Uid, from: usize) {
        for child in self.tree.nodes[uid].children.clone().into_iter().skip(from) {
            self.halt(child);
        }
    }

    fn reset_children(&mut self, uid: Uid) {
        self.halt_children_from(uid, 0);
    }

    fn child(&self, uid: Uid, i: usize) -> Uid {
        self.tree.nodes[uid].children[i]
    }

    fn mark_running(&mut self, uid: Uid) {
        if self.states[uid].status != NodeStatus::Running {
            self.set_status(uid, NodeStatus::Running);
        }
    }

    fn tick_node(&mut self, uid: Uid) -> NodeStatus {
        if let Some(o) = self.overrides[uid] {
            return self.forced(uid, o);
        }
        let node = &self.tree.nodes[uid];
        let kind = node.kind;
        let id = node.id.clone();
        let n = node.children.len();
        match kind {
            NodeKind::Root => unreachable!("the synthetic Root is not ticked"),
            NodeKind::SubTree => {
                if n == 0 {
                    self.message(Some(uid), format!("SubTree \"{id}\" has no tree; FAILURE"));
                    return NodeStatus::Failure;
                }
                if !self.is_active(uid) {
                    let scope = self.blackboards.scope_of[self.child(uid, 0)];
                    self.blackboards.enter(scope);
                }
                self.mark_running(uid);
                let status = self.execute_tick(self.child(uid, 0));
                if status != NodeStatus::Running {
                    self.reset_children(uid);
                }
                status
            }
            NodeKind::Control => {
                if n == 0 {
                    return NodeStatus::Failure;
                }
                self.mark_running(uid);
                self.tick_control(uid, &id, n)
            }
            NodeKind::Decorator => {
                if n == 0 {
                    return NodeStatus::Failure;
                }
                self.tick_decorator(uid, &id)
            }
            NodeKind::Action | NodeKind::Condition => self.tick_leaf(uid, &id, kind),
        }
    }

    /// An override decides the node without ticking it. A RUNNING native leaf is stopped at
    /// this instant; a control node or decorator halts everything below it first.
    fn forced(&mut self, uid: Uid, o: Override) -> NodeStatus {
        if !o.pinned {
            self.overrides[uid] = None;
        }
        self.halt_native(uid, HaltCause::Forced(o.status));
        if !self.tree.nodes[uid].kind.is_leaf() {
            let cause = std::mem::replace(&mut self.halt_cause, HaltCause::AncestorForced(uid));
            self.reset_children(uid);
            self.halt_cause = cause;
        }
        let old = std::mem::take(&mut self.states[uid]);
        self.states[uid] = NodeState {
            status: old.status,
            stored: old.stored,
            ..Default::default()
        };
        self.message(Some(uid), format!("forced {}", o.status));
        o.status
    }

    // ───────────────────────────────── controls ─────────────────────────────────

    fn tick_control(&mut self, uid: Uid, id: &str, n: usize) -> NodeStatus {
        match id {
            "Sequence" => self.sequence(uid, n, false),
            "SequenceWithMemory" => self.sequence(uid, n, true),
            "SequenceStar" => self.sequence_star(uid, n),
            "ReactiveSequence" => self.reactive(uid, n, NodeStatus::Failure),
            "Fallback" => self.fallback(uid, n),
            "ReactiveFallback" => self.reactive(uid, n, NodeStatus::Success),
            "Parallel" => self.parallel(uid, n),
            "ParallelAll" => self.parallel_all(uid, n),
            "IfThenElse" => self.if_then_else(uid, n),
            "WhileDoElse" => self.while_do_else(uid, n),
            s if s.starts_with("Switch") => self.switch(uid, n),
            other => {
                if self.tick == 1 || !self.is_active(uid) {
                    self.message(
                        Some(uid),
                        format!("unknown control node {other}: run as a Sequence"),
                    );
                }
                self.sequence(uid, n, false)
            }
        }
    }

    /// `SequenceNode` / `SequenceWithMemory`.
    fn sequence(&mut self, uid: Uid, n: usize, memory: bool) -> NodeStatus {
        while self.states[uid].child_idx < n {
            let idx = self.states[uid].child_idx;
            match self.execute_tick(self.child(uid, idx)) {
                NodeStatus::Running => return NodeStatus::Running,
                NodeStatus::Failure => {
                    if memory {
                        // Resume from the failed child next time.
                        self.halt_children_from(uid, idx);
                    } else {
                        self.reset_children(uid);
                        self.states[uid].child_idx = 0;
                        self.states[uid].skipped = 0;
                    }
                    return NodeStatus::Failure;
                }
                NodeStatus::Success => self.states[uid].child_idx += 1,
                NodeStatus::Skipped => {
                    self.states[uid].child_idx += 1;
                    self.states[uid].skipped += 1;
                }
                NodeStatus::Idle => return self.idle_child(uid),
            }
        }
        let all_skipped = self.states[uid].skipped == n;
        self.reset_children(uid);
        self.states[uid].child_idx = 0;
        self.states[uid].skipped = 0;
        if all_skipped {
            NodeStatus::Skipped
        } else {
            NodeStatus::Success
        }
    }

    /// v3's `SequenceStar` (BT.CPP 4's `SequenceWithMemory`). In the behaviortree-rs dialect
    /// it returns FAILURE once every child succeeded, as cell-runner's runtime does.
    fn sequence_star(&mut self, uid: Uid, n: usize) -> NodeStatus {
        let status = self.sequence(uid, n, true);
        if status != NodeStatus::Success || self.dialect() != Dialect::BehaviortreeRs {
            return status;
        }
        self.message(
            Some(uid),
            "every child succeeded: FAILURE, as behaviortree-rs's SequenceStar returns \
             (cell-runner)"
                .to_string(),
        );
        NodeStatus::Failure
    }

    /// `FallbackNode`.
    fn fallback(&mut self, uid: Uid, n: usize) -> NodeStatus {
        while self.states[uid].child_idx < n {
            let idx = self.states[uid].child_idx;
            match self.execute_tick(self.child(uid, idx)) {
                NodeStatus::Running => return NodeStatus::Running,
                NodeStatus::Success => {
                    self.reset_children(uid);
                    self.states[uid].child_idx = 0;
                    self.states[uid].skipped = 0;
                    return NodeStatus::Success;
                }
                NodeStatus::Failure => self.states[uid].child_idx += 1,
                NodeStatus::Skipped => {
                    self.states[uid].child_idx += 1;
                    self.states[uid].skipped += 1;
                }
                NodeStatus::Idle => return self.idle_child(uid),
            }
        }
        let all_skipped = self.states[uid].skipped == n;
        self.reset_children(uid);
        self.states[uid].child_idx = 0;
        self.states[uid].skipped = 0;
        if all_skipped {
            NodeStatus::Skipped
        } else {
            NodeStatus::Failure
        }
    }

    /// `ReactiveSequence` (`stop_on` FAILURE) / `ReactiveFallback` (`stop_on` SUCCESS):
    /// every tick starts from the first child.
    fn reactive(&mut self, uid: Uid, n: usize, stop_on: NodeStatus) -> NodeStatus {
        let mut all_skipped = true;
        for index in 0..n {
            let holds = self.holds;
            let status = self.execute_tick(self.child(uid, index));
            all_skipped &= status == NodeStatus::Skipped;
            match status {
                NodeStatus::Running => {
                    // Reset the others so they start from IDLE the next time — unless the child
                    // only waits at a breakpoint, which must not restart what runs after it.
                    if self.holds == holds {
                        for i in (0..n).filter(|&i| i != index) {
                            self.halt(self.child(uid, i));
                        }
                    }
                    return NodeStatus::Running;
                }
                NodeStatus::Skipped => self.halt(self.child(uid, index)),
                NodeStatus::Idle => return self.idle_child(uid),
                s if s == stop_on => {
                    self.reset_children(uid);
                    return stop_on;
                }
                _ => {}
            }
        }
        self.reset_children(uid);
        if all_skipped {
            NodeStatus::Skipped
        } else if stop_on == NodeStatus::Failure {
            NodeStatus::Success
        } else {
            NodeStatus::Failure
        }
    }

    /// `ParallelNode` with v4's `success_count` / `failure_count` (v3: `*_threshold`).
    fn parallel(&mut self, uid: Uid, n: usize) -> NodeStatus {
        let success_threshold = self
            .port_int(uid, "success_count")
            .or_else(|| self.port_int(uid, "success_threshold"))
            .unwrap_or(-1);
        let failure_threshold = self
            .port_int(uid, "failure_count")
            .or_else(|| self.port_int(uid, "failure_threshold"))
            .unwrap_or(1);
        let required = |t: i64| {
            if t < 0 {
                (n as i64 + t + 1).max(0) as usize
            } else {
                t as usize
            }
        };
        let required_success = required(success_threshold);
        let required_failure = required(failure_threshold);
        if required_success > n || required_failure > n {
            self.message(
                Some(uid),
                format!("Parallel: thresholds exceed its {n} children"),
            );
        }
        self.states[uid].completed.resize(n, None);

        let mut skipped = 0;
        for i in 0..n {
            if self.states[uid].completed[i].is_some() {
                continue;
            }
            match self.execute_tick(self.child(uid, i)) {
                NodeStatus::Skipped => skipped += 1,
                NodeStatus::Success => {
                    self.states[uid].completed[i] = Some(NodeStatus::Success);
                    self.states[uid].successes += 1;
                }
                NodeStatus::Failure => {
                    self.states[uid].completed[i] = Some(NodeStatus::Failure);
                    self.states[uid].failures += 1;
                }
                NodeStatus::Running => {}
                NodeStatus::Idle => return self.idle_child(uid),
            }
            let (successes, failures) = (self.states[uid].successes, self.states[uid].failures);
            if successes >= required_success
                || (success_threshold < 0 && successes + skipped >= required_success)
            {
                self.finish_parallel(uid);
                return NodeStatus::Success;
            }
            // Fail when success is out of reach or enough children failed.
            if n - failures < required_success || failures == required_failure {
                self.finish_parallel(uid);
                return NodeStatus::Failure;
            }
        }
        if skipped == n {
            NodeStatus::Skipped
        } else {
            NodeStatus::Running
        }
    }

    fn finish_parallel(&mut self, uid: Uid) {
        let state = &mut self.states[uid];
        state.completed.clear();
        state.successes = 0;
        state.failures = 0;
        self.reset_children(uid);
    }

    /// `ParallelAllNode`: run every child to the end, fail on `max_failures`.
    fn parallel_all(&mut self, uid: Uid, n: usize) -> NodeStatus {
        let max_failures = self.port_int(uid, "max_failures").unwrap_or(1);
        let required_failures = if max_failures < 0 {
            (n as i64 + max_failures + 1).max(0) as usize
        } else {
            max_failures as usize
        };
        self.states[uid].completed.resize(n, None);
        let mut skipped = 0;
        for i in 0..n {
            if self.states[uid].completed[i].is_some() {
                continue;
            }
            match self.execute_tick(self.child(uid, i)) {
                NodeStatus::Skipped => {
                    skipped += 1;
                    self.states[uid].completed[i] = Some(NodeStatus::Skipped);
                }
                NodeStatus::Success => self.states[uid].completed[i] = Some(NodeStatus::Success),
                NodeStatus::Failure => {
                    self.states[uid].completed[i] = Some(NodeStatus::Failure);
                    self.states[uid].failures += 1;
                }
                NodeStatus::Running => {}
                NodeStatus::Idle => return self.idle_child(uid),
            }
        }
        if skipped == n {
            self.finish_parallel(uid);
            return NodeStatus::Skipped;
        }
        if self.states[uid].completed.iter().all(Option::is_some) {
            let failed = self.states[uid].failures >= required_failures;
            self.finish_parallel(uid);
            return if failed {
                NodeStatus::Failure
            } else {
                NodeStatus::Success
            };
        }
        NodeStatus::Running
    }

    /// `IfThenElseNode`: condition, then-branch, optional else-branch.
    fn if_then_else(&mut self, uid: Uid, n: usize) -> NodeStatus {
        if !(2..=3).contains(&n) {
            self.message(Some(uid), "IfThenElse needs 2 or 3 children".to_string());
            return NodeStatus::Failure;
        }
        if self.states[uid].child_idx == 0 {
            match self.execute_tick(self.child(uid, 0)) {
                NodeStatus::Running => return NodeStatus::Running,
                NodeStatus::Success => self.states[uid].child_idx = 1,
                NodeStatus::Failure if n == 3 => self.states[uid].child_idx = 2,
                NodeStatus::Failure => return NodeStatus::Failure,
                NodeStatus::Skipped | NodeStatus::Idle => return self.idle_child(uid),
            }
        }
        let idx = self.states[uid].child_idx;
        let status = self.execute_tick(self.child(uid, idx));
        if status != NodeStatus::Running {
            self.reset_children(uid);
            self.states[uid].child_idx = 0;
        }
        status
    }

    /// `WhileDoElseNode`: the condition is re-checked every tick.
    fn while_do_else(&mut self, uid: Uid, n: usize) -> NodeStatus {
        if !(2..=3).contains(&n) {
            self.message(Some(uid), "WhileDoElse needs 2 or 3 children".to_string());
            return NodeStatus::Failure;
        }
        let status = match self.execute_tick(self.child(uid, 0)) {
            NodeStatus::Running => return NodeStatus::Running,
            NodeStatus::Success => {
                if n == 3 {
                    self.halt(self.child(uid, 2));
                }
                self.execute_tick(self.child(uid, 1))
            }
            NodeStatus::Failure if n == 3 => {
                self.halt(self.child(uid, 1));
                self.execute_tick(self.child(uid, 2))
            }
            NodeStatus::Failure => {
                self.halt(self.child(uid, 1));
                NodeStatus::Failure
            }
            NodeStatus::Skipped | NodeStatus::Idle => return self.idle_child(uid),
        };
        if status != NodeStatus::Running {
            self.reset_children(uid);
        }
        status
    }

    /// `SwitchNode<N>`: `variable` against `case_1..N`, the last child is the default.
    fn switch(&mut self, uid: Uid, n: usize) -> NodeStatus {
        let cases = n.saturating_sub(1);
        let variable = self
            .port_value(uid, "variable")
            .map(|v| script::display(&v));
        let mut matched = cases;
        if let Some(variable) = &variable {
            for i in 1..=cases {
                let case = self
                    .port_value(uid, &format!("case_{i}"))
                    .map(|v| script::display(&v));
                let equal = case.as_ref().is_some_and(|c| {
                    c == variable || matches!((c.trim().parse::<f64>(), variable.trim().parse::<f64>()), (Ok(a), Ok(b)) if a == b)
                });
                if equal {
                    matched = i - 1;
                    break;
                }
            }
        }
        if let Some(running) = self.states[uid].running_child
            && running != matched
        {
            self.halt(self.child(uid, running));
        }
        let status = self.execute_tick(self.child(uid, matched));
        if status == NodeStatus::Running {
            self.states[uid].running_child = Some(matched);
        } else {
            self.reset_children(uid);
            self.states[uid].running_child = None;
        }
        status
    }

    /// A child returned IDLE, which BT.CPP treats as a logic error.
    fn idle_child(&mut self, uid: Uid) -> NodeStatus {
        self.message(Some(uid), "a child returned IDLE".to_string());
        self.reset_children(uid);
        NodeStatus::Failure
    }

    // ──────────────────────────────── decorators ────────────────────────────────

    fn tick_decorator(&mut self, uid: Uid, id: &str) -> NodeStatus {
        self.mark_running(uid);
        let child = self.child(uid, 0);
        match id {
            "Inverter" => match self.execute_tick(child) {
                NodeStatus::Success => self.done_with(uid, NodeStatus::Failure),
                NodeStatus::Failure => self.done_with(uid, NodeStatus::Success),
                other => other,
            },
            "ForceSuccess" => match self.execute_tick(child) {
                NodeStatus::Success | NodeStatus::Failure => {
                    self.done_with(uid, NodeStatus::Success)
                }
                other => other,
            },
            "ForceFailure" => match self.execute_tick(child) {
                NodeStatus::Success | NodeStatus::Failure => {
                    self.done_with(uid, NodeStatus::Failure)
                }
                other => other,
            },
            "KeepRunningUntilFailure" => match self.execute_tick(child) {
                NodeStatus::Success => {
                    self.reset_children(uid);
                    NodeStatus::Running
                }
                NodeStatus::Failure => self.done_with(uid, NodeStatus::Failure),
                other => other,
            },
            "Repeat" => self.repeat(uid, child),
            "RetryUntilSuccessful" | "RetryUntilSuccesful" | "Retry" => self.retry(uid, child),
            "Timeout" => self.timeout(uid, child),
            "Delay" => self.delay(uid, child),
            "RunOnce" => self.run_once(uid, child),
            "Precondition" => self.precondition(uid, child),
            s if s.starts_with("BlackboardCheck") => self.blackboard_check(uid, child),
            other => {
                if !self.trace.of_node(uid).any(|_| true) || self.tick == 1 {
                    self.message(
                        Some(uid),
                        format!("unknown decorator {other}: passes its child's status through"),
                    );
                }
                let status = self.execute_tick(child);
                if status.is_completed() {
                    self.reset_children(uid);
                }
                status
            }
        }
    }

    fn done_with(&mut self, uid: Uid, status: NodeStatus) -> NodeStatus {
        self.reset_children(uid);
        status
    }

    fn repeat(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        let Some(cycles) = self.port_int(uid, "num_cycles") else {
            self.message(Some(uid), "Repeat: missing num_cycles".to_string());
            return NodeStatus::Failure;
        };
        let mut loops = 0;
        while cycles < 0 || self.states[uid].counter < cycles {
            match self.execute_tick(child) {
                NodeStatus::Success => {
                    self.states[uid].counter += 1;
                    self.reset_children(uid);
                }
                NodeStatus::Failure => {
                    self.states[uid].counter = 0;
                    return self.done_with(uid, NodeStatus::Failure);
                }
                NodeStatus::Skipped => {
                    self.states[uid].counter = 0;
                    return self.done_with(uid, NodeStatus::Skipped);
                }
                NodeStatus::Running => return NodeStatus::Running,
                NodeStatus::Idle => return self.idle_child(uid),
            }
            loops += 1;
            if loops >= MAX_LOOPS_PER_TICK {
                // cell-runner loops on within the tick: go on at this instant. An endless
                // loop waits for the next tick instead, so it cannot hold the run here.
                self.retick |= cycles >= 0;
                return NodeStatus::Running;
            }
        }
        self.states[uid].counter = 0;
        NodeStatus::Success
    }

    fn retry(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        let Some(attempts) = self.port_int(uid, "num_attempts") else {
            self.message(
                Some(uid),
                "RetryUntilSuccessful: missing num_attempts".to_string(),
            );
            return NodeStatus::Failure;
        };
        let mut loops = 0;
        while attempts < 0 || self.states[uid].counter < attempts {
            match self.execute_tick(child) {
                NodeStatus::Success => {
                    self.states[uid].counter = 0;
                    return self.done_with(uid, NodeStatus::Success);
                }
                NodeStatus::Failure => {
                    self.states[uid].counter += 1;
                    self.reset_children(uid);
                }
                NodeStatus::Skipped => {
                    self.states[uid].counter = 0;
                    return self.done_with(uid, NodeStatus::Skipped);
                }
                NodeStatus::Running => return NodeStatus::Running,
                NodeStatus::Idle => return self.idle_child(uid),
            }
            loops += 1;
            if loops >= MAX_LOOPS_PER_TICK {
                self.retick |= attempts >= 0;
                return NodeStatus::Running;
            }
        }
        self.states[uid].counter = 0;
        NodeStatus::Failure
    }

    fn timeout(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        let msec = self.port_int(uid, "msec").unwrap_or(0).max(0) as f64;
        let start = *self.states[uid].started_at.get_or_insert(self.now);
        let deadline = start + msec / 1000.0;
        if self.now + EPS >= deadline {
            self.message(
                Some(uid),
                format!("Timeout after {msec} ms: halting the child"),
            );
            self.states[uid].started_at = None;
            return self.done_with(uid, NodeStatus::Failure);
        }
        self.wake(uid, deadline);
        let status = self.execute_tick(child);
        if status != NodeStatus::Running {
            self.states[uid].started_at = None;
            self.reset_children(uid);
        }
        status
    }

    fn delay(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        // cell-runner's `port_f64`: a number, or text that reads as one (fractional
        // milliseconds count); anything else, a bool included, is 0.
        let msec = match self.port_value(uid, "delay_msec") {
            Some(Value::Number(n)) => n.as_f64(),
            Some(Value::String(s)) => s.trim().parse().ok(),
            _ => None,
        }
        .unwrap_or(0.0)
        .max(0.0);
        let start = *self.states[uid].started_at.get_or_insert(self.now);
        let release = start + msec / 1000.0;
        if self.now + EPS < release {
            self.wake(uid, release);
            return NodeStatus::Running;
        }
        let status = self.execute_tick(child);
        if status != NodeStatus::Running {
            self.states[uid].started_at = None;
            self.reset_children(uid);
        }
        status
    }

    fn run_once(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        let then_skip = self
            .port_value(uid, "then_skip")
            .is_none_or(|v| script::truthy(&v));
        if let Some(stored) = self.states[uid].stored {
            return if then_skip {
                NodeStatus::Skipped
            } else {
                stored
            };
        }
        let status = self.execute_tick(child);
        if status.is_completed() {
            self.states[uid].stored = Some(status);
            self.reset_children(uid);
        }
        status
    }

    fn precondition(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        if !self.states[uid].child_running {
            let code = self.tree.nodes[uid]
                .port_value("if")
                .unwrap_or("")
                .to_string();
            if !self.eval_bool(uid, &code) {
                let otherwise = self.tree.nodes[uid]
                    .port_value("else")
                    .unwrap_or("FAILURE")
                    .to_string();
                return parse_status(&otherwise).unwrap_or(NodeStatus::Failure);
            }
        }
        let status = self.execute_tick(child);
        self.states[uid].child_running = status == NodeStatus::Running;
        if status != NodeStatus::Running {
            self.reset_children(uid);
        }
        status
    }

    fn blackboard_check(&mut self, uid: Uid, child: Uid) -> NodeStatus {
        let a = self.port_value(uid, "value_A");
        let b = self.port_value(uid, "value_B");
        let equal = match (&a, &b) {
            (Some(a), Some(b)) => match (script::number(a), script::number(b)) {
                (Some(x), Some(y)) => x == y,
                _ => script::display(a) == script::display(b),
            },
            _ => false,
        };
        if !equal {
            self.halt(child);
            let otherwise = self.tree.nodes[uid]
                .port_value("return_on_mismatch")
                .unwrap_or("FAILURE")
                .to_string();
            return parse_status(&otherwise).unwrap_or(NodeStatus::Failure);
        }
        let status = self.execute_tick(child);
        if status != NodeStatus::Running {
            self.reset_children(uid);
        }
        status
    }

    // ────────────────────────────────── leaves ──────────────────────────────────

    fn tick_leaf(&mut self, uid: Uid, id: &str, kind: NodeKind) -> NodeStatus {
        match id {
            "AlwaysSuccess" => NodeStatus::Success,
            "AlwaysFailure" => NodeStatus::Failure,
            "Script" => {
                let code = self.tree.nodes[uid]
                    .port_value("code")
                    .unwrap_or("")
                    .to_string();
                if self.run_script(uid, &code) {
                    NodeStatus::Success
                } else {
                    NodeStatus::Failure
                }
            }
            "ScriptCondition" => {
                let code = self.tree.nodes[uid]
                    .port_value("code")
                    .unwrap_or("")
                    .to_string();
                if self.eval_bool(uid, &code) {
                    NodeStatus::Success
                } else {
                    NodeStatus::Failure
                }
            }
            "SetBlackboard" => {
                let node = &self.tree.nodes[uid];
                let key = node.port_value("output_key").map(strip_braces);
                let value = node.port_value("value").map(str::to_string);
                let (Some(key), Some(value)) = (key, value) else {
                    self.message(
                        Some(uid),
                        "SetBlackboard needs value and output_key".to_string(),
                    );
                    return NodeStatus::Failure;
                };
                let scope = self.blackboards.scope_of[uid];
                let value = match blackboard_pointer(&value) {
                    Some(src) => self.blackboards.get(scope, src).unwrap_or(Value::Null),
                    None => Value::String(value),
                };
                self.blackboards.set(scope, &key, value);
                NodeStatus::Success
            }
            "UnsetBlackboard" => {
                if let Some(key) = self.tree.nodes[uid].port_value("key").map(strip_braces) {
                    let scope = self.blackboards.scope_of[uid];
                    self.blackboards.remove(scope, &key);
                }
                NodeStatus::Success
            }
            "Sleep" => {
                let msec = self.port_int(uid, "msec").unwrap_or(0).max(0) as f64;
                self.timed(uid, msec / 1000.0, NodeStatus::Success)
            }
            _ => {
                if self.handled[uid] && self.policies[uid] == LeafPolicy::Native {
                    return self.tick_native(uid);
                }
                // Switched away from `Native` while it ran: the handler lets go of it first.
                if self.states[uid].native {
                    self.halt_native(uid, HaltCause::PolicyChanged);
                }
                match (kind, self.policies[uid]) {
                    (NodeKind::Condition, LeafPolicy::Fail { .. }) => NodeStatus::Failure,
                    (NodeKind::Condition, _) => NodeStatus::Success,
                    (_, LeafPolicy::Succeed { after_s }) => {
                        self.timed(uid, after_s, NodeStatus::Success)
                    }
                    (_, LeafPolicy::Fail { after_s }) => {
                        self.timed(uid, after_s, NodeStatus::Failure)
                    }
                    (_, LeafPolicy::Manual) => {
                        self.states[uid].started_at.get_or_insert(self.now);
                        NodeStatus::Running
                    }
                    // Not claimed by a handler: the default stub.
                    (_, LeafPolicy::Native) => self.timed(uid, 1.0, NodeStatus::Success),
                }
            }
        }
    }

    /// A leaf the handler runs: `on_start` when it starts, `on_running` while it runs.
    fn tick_native(&mut self, uid: Uid) -> NodeStatus {
        let starting = !self.states[uid].native;
        if starting {
            self.states[uid].started_tick = self.tick;
            self.states[uid].started_at = None;
        }
        let status = self
            .call_leaf(uid, |h, cx| {
                if starting {
                    h.on_start(cx)
                } else {
                    h.on_running(cx)
                }
            })
            .unwrap_or_else(|| {
                self.message(Some(uid), "no leaf handler: FAILURE".to_string());
                NodeStatus::Failure
            });
        self.states[uid].native = status == NodeStatus::Running;
        status
    }

    /// RUNNING for `duration` seconds from the first tick, then `result`.
    fn timed(&mut self, uid: Uid, duration: f64, result: NodeStatus) -> NodeStatus {
        if duration <= 0.0 {
            return result;
        }
        let start = *self.states[uid].started_at.get_or_insert(self.now);
        let end = start + duration;
        if self.now + EPS >= end {
            self.states[uid].started_at = None;
            result
        } else {
            self.wake(uid, end);
            NodeStatus::Running
        }
    }

    // ──────────────────────────────── helpers ────────────────────────────────

    fn port_int(&self, uid: Uid, port: &str) -> Option<i64> {
        self.port_value(uid, port)
            .and_then(|v| script::number(&v))
            .map(|n| n as i64)
    }

    /// Run a script in the node's scope; logs and returns `false` on error.
    fn run_script(&mut self, uid: Uid, code: &str) -> bool {
        let scope = self.blackboards.scope_of[uid];
        let mut env = ScopeEnv {
            blackboards: &mut self.blackboards,
            scope,
        };
        match script::run(code, &mut env) {
            Ok(_) => true,
            Err(e) => {
                self.message(Some(uid), format!("script `{code}`: {e}"));
                false
            }
        }
    }

    fn eval_bool(&mut self, uid: Uid, code: &str) -> bool {
        let scope = self.blackboards.scope_of[uid];
        let mut env = ScopeEnv {
            blackboards: &mut self.blackboards,
            scope,
        };
        match script::eval_bool(code, &mut env) {
            Ok(b) => b,
            Err(e) => {
                self.message(Some(uid), format!("condition `{code}`: {e}"));
                false
            }
        }
    }
}

fn strip_braces(value: &str) -> String {
    blackboard_pointer(value)
        .unwrap_or(value.trim())
        .to_string()
}

fn parse_status(text: &str) -> Option<NodeStatus> {
    Some(match text.trim() {
        "SUCCESS" => NodeStatus::Success,
        "FAILURE" => NodeStatus::Failure,
        "RUNNING" => NodeStatus::Running,
        "SKIPPED" => NodeStatus::Skipped,
        _ => return None,
    })
}

/// Leaves that BT.CPP does not implement, so a policy decides them.
pub fn is_stub(id: &str, kind: NodeKind) -> bool {
    kind.is_leaf()
        && !matches!(
            id,
            "AlwaysSuccess"
                | "AlwaysFailure"
                | "Script"
                | "ScriptCondition"
                | "SetBlackboard"
                | "UnsetBlackboard"
                | "Sleep"
        )
}

/// Script access to the blackboard scope of one node.
struct ScopeEnv<'a> {
    blackboards: &'a mut Blackboards,
    scope: usize,
}

impl Env for ScopeEnv<'_> {
    fn get(&self, key: &str) -> Option<Value> {
        self.blackboards.get(self.scope, key)
    }
    fn set(&mut self, key: &str, value: Value) {
        self.blackboards.set(self.scope, key, value);
    }
}
