//! The cell leaves as bt-exec native leaves: [`CellLeaves`] places their motion on a
//! [`Schedule`] while the tree runs.
//!
//! Each leaf does what cell-runner's does (`btree.rs:244-493`), with the same warnings:
//!
//! | leaf                | when it starts                                                   | while RUNNING |
//! |---------------------|------------------------------------------------------------------|---------------|
//! | `ExecuteTrajectory` | places its motion (unreadable ports: a warning, SUCCESS, nothing) | until motion, hold and settle are over |
//! | `MoveHome`          | places the planned return path, else a slow move home            | the same |
//! | `WaitForRobot`      | polls its joint gate                                             | polls again when the gate could open |
//! | `CheckSafety`       | SUCCESS                                                          | — |
//!
//! Placing motion asks for a re-tick at the same instant (cell-runner's `Clock.moved`), so a
//! gate sees motion a sibling placed in the same tick. A gate nothing can open waits until
//! the tree stalls and [`Executor::release_stalled`](bt_exec::Executor::release_stalled) lets
//! it go, with cell-runner's "never reaches" warning; until then it shows up in
//! [`Executor::blocked`](bt_exec::Executor::blocked) with a [waiting reason](LeafHandler::waiting_reason).
//!
//! While they are installed, the builtins follow behaviortree-rs where it differs from BT.CPP
//! ([`Dialect::BehaviortreeRs`]).
//!
//! cell-runner's motion leaves have no halt: a motion its parent stops early (a `Parallel`
//! that has enough successes, say) still plays to its end. That is [`HaltedMotion::Keep`], the
//! default; [`HaltedMotion::Cut`] stops it where it is instead. A motion stopped by the user
//! (an override on it or above it, or a policy switch) is always cut at that instant.

use std::collections::BTreeMap;

use bt_core::NodeStatus;
use bt_core::model::{NodeKind, Uid};
use bt_exec::{Dialect, Executor, HaltCause, LeafCtx, LeafHandler};
use serde_json::Value;

use crate::CELL_LEAVES;
use crate::joints::{
    COMPLETION_MARGIN_S, DEFAULT_INTERLOCK_TOL_RAD, MAX_SEGMENTS, max_joint_error, to_six,
};
use crate::plan::JointDependency;
use crate::ports::{self, AtRest, PortSource};
use crate::schedule::{Placed, PlacedBy, RobotState, Schedule, WarningKind};

/// What happens to a motion when its parent halts it (a tree halt, not an override).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HaltedMotion {
    /// It plays to its end, like cell-runner, whose motion leaves have no halt.
    #[default]
    Keep,
    /// It is cut at the instant of the halt.
    Cut,
}

/// How the cell is set up.
#[derive(Clone, Debug, Default)]
pub struct CellOptions {
    /// Robots known up front (more are added as they move).
    pub robots: Vec<String>,
    /// Home configurations; unplanned `MoveHome`s go there, and a robot sits there before
    /// its first motion as far as gates and interlock pre-moves are concerned.
    pub home: BTreeMap<String, [f64; 6]>,
    /// Where to show each robot before its first motion is placed (display only).
    pub preroll: BTreeMap<String, [f64; 6]>,
    /// What a tree halt does to a running motion (overrides always cut).
    pub halted_motion: HaltedMotion,
}

impl CellOptions {
    /// For an interactive run of `ex`'s tree: the robots its motion leaves name, each shown
    /// where its first trajectory starts, homes unknown (cell-runner's fallback).
    pub fn for_executor(ex: &Executor) -> Self {
        let mut options = Self::default();
        for node in &ex.tree().nodes {
            if !node.kind.is_leaf() || !matches!(node.id.as_str(), "ExecuteTrajectory" | "MoveHome")
            {
                continue;
            }
            let src = AtRest { ex, uid: node.uid };
            let Some(robot) = ports::port_str(&src, "robot_ns") else {
                continue;
            };
            if !options.robots.contains(&robot) {
                options.robots.push(robot.clone());
            }
            if !options.preroll.contains_key(&robot)
                && let Ok(Some(traj)) = ports::port_trajectory(&src, "trajectory")
                && let Some(first) = traj.first()
            {
                options.preroll.insert(robot, to_six(&first.positions));
            }
        }
        options
    }

    /// What cell-runner's `Schedule::build(plan, robots, home)` runs with.
    pub fn cell_runner(robots: &[String], home: BTreeMap<String, [f64; 6]>) -> Self {
        Self {
            robots: robots.to_vec(),
            home,
            ..Self::default()
        }
    }
}

/// A dependency of a waiting gate, as of its latest poll.
#[derive(Clone, Debug, PartialEq)]
pub struct PendingDep {
    /// The robot waited for.
    pub robot: String,
    /// The configuration it must reach.
    pub target: [f64; 6],
    /// The tolerance in effect, rad (the default when the plan gives none).
    pub tol: f64,
    /// How far the robot was from `target` (largest joint error, rad).
    pub error: f64,
    /// When the motion placed so far brings the robot there.
    pub opens_at: Option<f64>,
}

/// A waiting `WaitForRobot`.
#[derive(Clone, Debug, PartialEq)]
pub struct GateView {
    /// Its label in warnings (its name, else its `robot_ns`).
    pub label: String,
    /// When it started waiting.
    pub since: f64,
    /// What it still waits for.
    pub deps: Vec<PendingDep>,
}

/// A cell leaf that is RUNNING.
#[derive(Clone, Debug)]
enum Run {
    Motion {
        placed: Placed,
        robot: String,
        op: String,
    },
    Gate {
        label: String,
        pending: Vec<JointDependency>,
        since: f64,
        last: Vec<PendingDep>,
    },
}

/// The [`LeafHandler`] for the cell leaves (see the module docs).
#[derive(Clone, Debug)]
pub struct CellLeaves {
    options: CellOptions,
    schedule: Schedule,
    active: BTreeMap<Uid, Run>,
    finished: Option<(NodeStatus, f64)>,
}

impl CellLeaves {
    /// The cell leaves for a cell set up as `options`, nothing placed yet.
    pub fn new(options: CellOptions) -> Self {
        let mut leaves = Self {
            schedule: Schedule::new(&options.robots, options.home.clone()),
            options,
            active: BTreeMap::new(),
            finished: None,
        };
        leaves.reset();
        leaves
    }

    /// The motion placed so far.
    pub fn schedule(&self) -> &Schedule {
        &self.schedule
    }

    /// How the cell is set up.
    pub fn options(&self) -> &CellOptions {
        &self.options
    }

    /// `halted_motion` applies from the next halt; robots, homes and pre-roll poses from the
    /// next reset.
    pub fn options_mut(&mut self) -> &mut CellOptions {
        &mut self.options
    }

    /// Every robot: those known up front, then those that moved.
    pub fn robots(&self) -> &[String] {
        &self.schedule.robots
    }

    /// Every robot at `t`, knowing only the motion placed up to tick `upto`.
    pub fn robot_states(&self, t: f64, upto: u64) -> Vec<RobotState> {
        self.schedule.robot_states(t, upto)
    }

    /// The motion a RUNNING `ExecuteTrajectory` or `MoveHome` placed.
    pub fn motion(&self, uid: Uid) -> Option<&Placed> {
        match self.active.get(&uid)? {
            Run::Motion { placed, .. } => Some(placed),
            Run::Gate { .. } => None,
        }
    }

    /// A waiting `WaitForRobot`.
    pub fn gate(&self, uid: Uid) -> Option<GateView> {
        match self.active.get(&uid)? {
            Run::Gate {
                label, since, last, ..
            } => Some(GateView {
                label: label.clone(),
                since: *since,
                deps: last.clone(),
            }),
            Run::Motion { .. } => None,
        }
    }

    /// How far a motion leaf is through its run at `t` (motion, hold and settle), as of tick
    /// `upto`; `None` before it starts moving and after it is done.
    ///
    /// It is worked out from the segments, so a past tick shows what it showed then. The
    /// scale is the run as planned; a cut ends it early (at the cut, once the cut happened
    /// as of `upto`).
    pub fn progress(&self, uid: Uid, t: f64, upto: u64) -> Option<f32> {
        // The latest placement by `uid` that started by `t`: its segments (an interlock
        // pre-move and the motion) are adjacent and share the tick they were placed in.
        let segments = &self.schedule.segments;
        let mut group: Option<(usize, usize)> = None;
        for (i, s) in segments.iter().enumerate() {
            if s.uid != Some(uid) || s.placed_tick > upto {
                continue;
            }
            match group {
                Some((first, last))
                    if last + 1 == i && segments[first].placed_tick == s.placed_tick =>
                {
                    group = Some((first, i));
                }
                _ if s.t0 <= t => group = Some((i, i)),
                _ => {}
            }
        }
        let (first, last) = group?;
        let placed = &segments[first..=last];
        let start = placed[0].t0;
        let end = &placed[placed.len() - 1];
        let done_at = end.cut.map_or(end.t_done(), |c| c.planned_t_done) + COMPLETION_MARGIN_S;
        let stopped = placed
            .iter()
            .filter_map(|s| s.cut.filter(|c| c.tick <= upto).map(|c| c.at))
            .reduce(f64::min);
        if t > stopped.unwrap_or(done_at) + bt_exec::EPS {
            return None;
        }
        let span = done_at - start;
        let p = if span > 0.0 {
            ((t - start) / span).clamp(0.0, 1.0)
        } else {
            1.0
        };
        Some(p as f32)
    }

    /// How the tree finished, and when.
    pub fn finished(&self) -> Option<(NodeStatus, f64)> {
        self.finished
    }

    fn warn(&mut self, cx: &LeafCtx<'_>, kind: WarningKind, message: String) {
        let uid = (cx.uid() != 0).then_some(cx.uid());
        self.schedule.warn(kind, cx.now(), uid, message);
    }

    /// Mirror the warnings raised since `before` into the executor log.
    fn mirror(&self, cx: &mut LeafCtx<'_>, before: usize) {
        for warning in &self.schedule.warnings[before..] {
            cx.log(warning.message.clone());
        }
    }

    /// Whether placing `n` more segments would pass [`MAX_SEGMENTS`] (warned about once).
    fn at_limit(&mut self, cx: &LeafCtx<'_>, n: usize) -> bool {
        if self.schedule.segments.len() + n <= MAX_SEGMENTS {
            return false;
        }
        if !self
            .schedule
            .warnings
            .iter()
            .any(|w| w.kind == WarningKind::Limit)
        {
            self.warn(
                cx,
                WarningKind::Limit,
                format!("motion limit reached ({MAX_SEGMENTS} segments) — runaway tree?"),
            );
        }
        true
    }

    fn placed_by(cx: &LeafCtx<'_>) -> PlacedBy {
        PlacedBy {
            uid: Some(cx.uid()),
            tick: cx.tick(),
        }
    }

    /// Wait for `placed` to be over (cell-runner `await_until(done_at)`), remembering it.
    fn run_motion(
        &mut self,
        cx: &mut LeafCtx<'_>,
        placed: Placed,
        robot: String,
        op: String,
    ) -> NodeStatus {
        cx.retick();
        let status = cx.await_until(placed.done_at);
        if status == NodeStatus::Running {
            self.active
                .insert(cx.uid(), Run::Motion { placed, robot, op });
        }
        status
    }

    /// `ExecuteTrajectory::on_start` (`btree.rs:279-295`).
    fn start_execute(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let op = match ports::read_execute(&*cx) {
            Ok(op) => op,
            Err(warning) => {
                self.warn(cx, WarningKind::Runner, warning);
                return NodeStatus::Success;
            }
        };
        let n = 1 + usize::from(op.interlock_joints.is_some());
        if self.at_limit(cx, n) {
            return NodeStatus::Failure;
        }
        let placed = self
            .schedule
            .start_execute(&op, cx.now(), Self::placed_by(cx));
        self.run_motion(cx, placed, op.robot, op.op_id)
    }

    /// `MoveHome::on_start` (`btree.rs:327-348`).
    fn start_home(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let robot = ports::port_str(&*cx, "robot_ns");
        let planned = ports::port_trajectory(&*cx, "trajectory");
        let Some(robot) = robot else {
            self.warn(
                cx,
                WarningKind::Runner,
                "MoveHome without robot_ns — skipped".to_string(),
            );
            return NodeStatus::Success;
        };
        let planned = planned.unwrap_or_else(|e| {
            self.warn(
                cx,
                WarningKind::Runner,
                format!("MoveHome {robot}: {e} — moving home unplanned"),
            );
            None
        });
        if self.at_limit(cx, 1) {
            return NodeStatus::Failure;
        }
        let placed = self.schedule.start_home(
            &robot,
            planned.as_deref().unwrap_or(&[]),
            cx.now(),
            Self::placed_by(cx),
        );
        self.run_motion(cx, placed, robot, "home".to_string())
    }

    /// `WaitForRobot::on_start` (`btree.rs:386-418`).
    fn start_wait(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let label = wait_label(&*cx);
        let joints = ports::port(&*cx, "depends_on_joints");
        let positions = ports::port(&*cx, "depends_on_positions");
        let pending: Vec<JointDependency> = match joints {
            None => Vec::new(),
            Some(value) => serde_json::from_value(value).unwrap_or_else(|e| {
                self.warn(
                    cx,
                    WarningKind::Runner,
                    format!("WaitForRobot {label}: unreadable depends_on_joints ({e}) — ignored"),
                );
                Vec::new()
            }),
        };
        if let Some(count) = positions.as_ref().and_then(Value::as_array).map(Vec::len)
            && count > 0
        {
            self.warn(
                cx,
                WarningKind::Runner,
                format!(
                    "WaitForRobot {label}: {count} Cartesian gate(s) ignored (playback synthesizes no FK)"
                ),
            );
        }
        let pending = pending
            .into_iter()
            .filter(|dep| !dep.joint_positions.is_empty())
            .collect();
        self.active.insert(
            cx.uid(),
            Run::Gate {
                label,
                pending,
                since: cx.now(),
                last: Vec::new(),
            },
        );
        self.poll_gate(cx)
    }

    /// One evaluation of a joint gate at the clock's current instant (`btree.rs:435-475`).
    ///
    /// Satisfied dependencies are dropped; for the rest, the earliest future
    /// instant each could open (given the motion placed so far) becomes a wake-up.
    /// A gate nothing can ever open is released once the whole tree has stalled —
    /// by then every motion that could have satisfied it has already been placed.
    fn poll_gate(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let (now, uid) = (cx.now(), Some(cx.uid()));
        let Some(Run::Gate {
            label,
            pending,
            last,
            ..
        }) = self.active.get_mut(&cx.uid())
        else {
            return NodeStatus::Success;
        };
        let schedule = &mut self.schedule;
        pending.retain(|dep| {
            max_joint_error(
                &to_six(&dep.joint_positions),
                &schedule.pose_at(&dep.robot, now),
            ) > tolerance(dep)
        });
        if pending.is_empty() {
            return NodeStatus::Success;
        }
        if cx.releasing() {
            for dep in pending.drain(..) {
                let tol = tolerance(&dep);
                schedule.warn(
                    WarningKind::Runner,
                    now,
                    uid,
                    format!(
                        "WaitForRobot {label}: {} never reaches the awaited configuration \
                         within {tol} rad — released once nothing else could move",
                        dep.robot
                    ),
                );
            }
            return NodeStatus::Success;
        }
        *last = pending
            .iter()
            .map(|dep| {
                let target = to_six(&dep.joint_positions);
                let tol = tolerance(dep);
                PendingDep {
                    robot: dep.robot.clone(),
                    target,
                    tol,
                    error: max_joint_error(&target, &schedule.pose_at(&dep.robot, now)),
                    opens_at: schedule.gate_time(&dep.robot, &target, tol, now),
                }
            })
            .collect();
        for t in last.iter().filter_map(|dep| dep.opens_at) {
            cx.wake_at(t);
        }
        NodeStatus::Running
    }
}

impl LeafHandler for CellLeaves {
    fn handles(&self, id: &str, kind: NodeKind) -> bool {
        kind.is_leaf() && CELL_LEAVES.contains(&id)
    }

    /// cell-runner's runtime, so `SequenceStar` fails once every child succeeded.
    fn dialect(&self) -> Dialect {
        Dialect::BehaviortreeRs
    }

    fn on_start(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let before = self.schedule.warnings.len();
        let status = match cx.node().id.as_str() {
            "ExecuteTrajectory" => self.start_execute(cx),
            "MoveHome" => self.start_home(cx),
            "WaitForRobot" => self.start_wait(cx),
            // CheckSafety: nothing to gate during playback (`btree.rs:477-493`).
            _ => NodeStatus::Success,
        };
        if status != NodeStatus::Running {
            self.active.remove(&cx.uid());
        }
        self.mirror(cx, before);
        status
    }

    fn on_running(&mut self, cx: &mut LeafCtx<'_>) -> NodeStatus {
        let before = self.schedule.warnings.len();
        let status = match self.active.get(&cx.uid()) {
            Some(Run::Motion { placed, .. }) => cx.await_until(placed.done_at),
            Some(Run::Gate { .. }) => self.poll_gate(cx),
            None => {
                cx.log("cell leaf lost its run (reset while RUNNING?): FAILURE");
                NodeStatus::Failure
            }
        };
        if status != NodeStatus::Running {
            self.active.remove(&cx.uid());
        }
        self.mirror(cx, before);
        status
    }

    fn on_halted(&mut self, cx: &mut LeafCtx<'_>, cause: HaltCause) {
        let before = self.schedule.warnings.len();
        let now = cx.now();
        match self.active.remove(&cx.uid()) {
            Some(Run::Motion { placed, robot, op }) => {
                let cut =
                    cause != HaltCause::Tree || self.options.halted_motion == HaltedMotion::Cut;
                if cut && self.schedule.cut(&placed, now, cx.tick()) {
                    self.warn(
                        cx,
                        WarningKind::Cut,
                        format!("{robot} {op}: cut at t={now:.3}s"),
                    );
                    // A gate may open at this instant now.
                    cx.retick();
                }
            }
            Some(Run::Gate { label, pending, .. }) => {
                // Only an override decides a gate instead of its dependencies; a tree halt or a
                // policy switch says nothing. Every dependency still pending is warned about,
                // even one its robot happens to meet at this instant: gates are polled only at
                // events, so cell-runner may well have kept this one waiting.
                let verb = match cause {
                    HaltCause::Forced(NodeStatus::Success) => "released",
                    HaltCause::Forced(NodeStatus::Failure) => "failed",
                    _ => return,
                };
                for dep in &pending {
                    let error = max_joint_error(
                        &to_six(&dep.joint_positions),
                        &self.schedule.pose_at(&dep.robot, now),
                    );
                    self.warn(
                        cx,
                        WarningKind::Override,
                        format!(
                            "WaitForRobot {label}: {verb} by an override ({} {error:.3} rad \
                             from the awaited joints)",
                            dep.robot
                        ),
                    );
                }
            }
            None => {}
        }
        self.mirror(cx, before);
    }

    fn on_finished(&mut self, cx: &mut LeafCtx<'_>, root: NodeStatus) {
        let before = self.schedule.warnings.len();
        let now = cx.now();
        if root == NodeStatus::Failure {
            self.warn(
                cx,
                WarningKind::Runner,
                format!("BehaviorTree returned FAILURE at t={now:.2}s"),
            );
        }
        self.schedule.duration = now;
        self.finished = Some((root, now));
        self.mirror(cx, before);
    }

    fn reset(&mut self) {
        self.schedule = Schedule::new(&self.options.robots, self.options.home.clone());
        self.schedule.set_preroll(self.options.preroll.clone());
        self.active.clear();
        self.finished = None;
    }

    fn waiting_reason(&self, uid: Uid) -> Option<String> {
        match self.active.get(&uid)? {
            Run::Motion { placed, robot, op } if placed.done_at.is_finite() => {
                Some(format!("{robot} {op} until t={:.2}s", placed.done_at))
            }
            Run::Motion { robot, op, .. } => Some(format!(
                "{robot} {op} never ends (an infinite hold or duration)"
            )),
            Run::Gate { label, last, .. } => {
                let reasons: Vec<String> = last
                    .iter()
                    .map(|dep| match dep.opens_at {
                        Some(t) => {
                            format!("{label}: opens at t={t:.2}s when {} arrives", dep.robot)
                        }
                        None => format!(
                            "{label}: no motion placed so far brings {} within {} rad of the \
                             awaited joints ({:.3} rad away)",
                            dep.robot, dep.tol, dep.error
                        ),
                    })
                    .collect();
                (!reasons.is_empty()).then(|| reasons.join("; "))
            }
        }
    }
}

/// A gate's tolerance; ≤ 0 means the default (`btree.rs:427-433`).
fn tolerance(dep: &JointDependency) -> f64 {
    if dep.tolerance_rad > 0.0 {
        dep.tolerance_rad
    } else {
        DEFAULT_INTERLOCK_TOL_RAD
    }
}

/// A `WaitForRobot`'s label in warnings: its `name` port, else its `robot_ns`, else `?`
/// (`btree.rs:387-389`).
pub(crate) fn wait_label<S: PortSource + ?Sized>(src: &S) -> String {
    ports::name_port(src)
        .or_else(|| ports::port_str(src, "robot_ns"))
        .unwrap_or_else(|| "?".to_string())
}
