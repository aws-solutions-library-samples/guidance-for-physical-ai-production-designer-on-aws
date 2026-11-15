//! The motion timeline the cell leaves place their trajectories on.
//!
//! Ported from cell-runner's `Schedule` (`trajectory.rs:411-780`): placement
//! ([`Schedule::start_execute`], [`Schedule::start_home`]), the pose a robot holds at any
//! instant ([`Schedule::pose_at`]) and when a joint gate opens ([`Schedule::gate_time`]) are
//! verbatim, with `HashMap` replaced by `BTreeMap`.
//!
//! btgui adds what an interactive run needs and a compile-once run does not:
//!
//! - Each [`Segment`] knows the leaf that placed it and the tick it was placed in, so the
//!   past can be shown as it was ([`Schedule::display_pose_at`], [`Schedule::robot_states`]).
//! - A motion stopped early is [cut](Schedule::cut) at that instant; what already happened
//!   stays as it was.
//! - Warnings are typed ([`CellWarning`]) and placement checks continuity: a motion that does
//!   not start where its robot is ([`WarningKind::Jump`]), or starts while the robot's last
//!   motion still runs ([`WarningKind::Overlap`]).
//!
//! Playback (`trajectory.rs:782-1021`) is not ported: btgui scrubs the executor instead.

use std::collections::BTreeMap;
use std::ops::Range;

use bt_core::model::Uid;
use bt_exec::EPS;

use crate::joints::{
    COMPLETION_MARGIN_S, FALLBACK_HOME_JOINTS, HOME_RETURN_TIME_S, HOME_TIME_S, JUMP_TOL_RAD,
    Waypoint, interpolate, max_joint_error, to_six,
};
use crate::plan::{BtPlan, PlanWaypoint};

/// Who placed a motion, and when.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PlacedBy {
    /// The leaf (none when placed directly on a schedule).
    pub uid: Option<Uid>,
    /// The executor tick it was placed in.
    pub tick: u64,
}

/// How a motion was stopped early.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cut {
    /// When it was stopped.
    pub at: f64,
    /// Where the trajectory was planned to end, before the cut (`t_end`).
    pub planned_t_end: f64,
    /// Where the hold was planned to end, before the cut (`t_done`).
    pub planned_t_done: f64,
    /// It had not started: nothing of it is played.
    pub dropped: bool,
    /// The executor tick the cut happened in.
    pub tick: u64,
}

/// One robot motion occupying an absolute window on the compiled timeline.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    /// The robot that moves.
    pub robot: String,
    /// When it starts (executor time).
    pub t0: f64,
    /// The motion, segment-local times; ends where a cut stopped it.
    pub traj: Vec<Waypoint>,
    /// The operation (`home` for a `MoveHome`, `{op}:interlock` for a pre-move).
    pub op_id: String,
    /// `weld`, `move`, …
    pub op_type: String,
    /// How long it holds the last pose after the motion.
    pub hold_s: f64,
    /// The leaf that placed it.
    pub uid: Option<Uid>,
    /// The tick it was placed in.
    pub placed_tick: u64,
    /// Set when it was stopped early.
    pub cut: Option<Cut>,
}

impl Segment {
    /// How long the motion lasts.
    pub fn duration(&self) -> f64 {
        self.traj.last().map(|w| w.t).unwrap_or(0.0)
    }
    /// When the motion ends.
    pub fn t_end(&self) -> f64 {
        self.t0 + self.duration()
    }
    /// When the hold ends.
    pub fn t_done(&self) -> f64 {
        self.t_end() + self.hold_s
    }

    /// Dropped before it started.
    pub fn is_dropped(&self) -> bool {
        self.cut.is_some_and(|c| c.dropped)
    }

    /// Whether the segment existed as of tick `upto` (placed by then, and not dropped yet).
    pub fn visible_at(&self, upto: u64) -> bool {
        self.placed_tick <= upto && !self.cut.is_some_and(|c| c.dropped && c.tick <= upto)
    }

    /// `(t_end, t_done)` as planned as of tick `upto`: a later cut is not known yet.
    pub fn extent_at(&self, upto: u64) -> (f64, f64) {
        match self.cut.filter(|c| c.tick > upto) {
            Some(c) => (c.planned_t_end, c.planned_t_done),
            None => (self.t_end(), self.t_done()),
        }
    }
}

/// An `ExecuteTrajectory` leaf's resolved ports.
#[derive(Debug, Clone, PartialEq)]
pub struct ExecuteOp {
    /// `robot_ns`.
    pub robot: String,
    /// `operation_id` (`op` when unset).
    pub op_id: String,
    /// `operation_type`.
    pub op_type: String,
    /// `trajectory`, as planned (before `velocity_scale`).
    pub traj: Vec<PlanWaypoint>,
    /// `start_delay_s`: waits this long before moving.
    pub start_delay_s: f64,
    /// `hold_duration_s`: holds the last pose this long.
    pub hold_s: f64,
    /// `velocity_scale`: divides the trajectory's times (≤ 0 means 1).
    pub velocity_scale: f64,
    /// Safe intermediate configuration to pre-move to (`has_interlock`).
    pub interlock_joints: Option<Vec<f64>>,
}

/// The segments one placement added, and when the leaf completes (trajectory + hold + settle).
#[derive(Debug, Clone, PartialEq)]
pub struct Placed {
    /// Indices into [`Schedule::segments`]: an interlock pre-move, then the motion.
    pub segments: Range<usize>,
    /// When the leaf completes.
    pub done_at: f64,
}

/// What a warning is about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarningKind {
    /// cell-runner gives this warning too (same text).
    Runner,
    /// A motion starts away from where its robot is.
    Jump,
    /// A motion starts while the robot's previous one still runs.
    Overlap,
    /// A motion was stopped early.
    Cut,
    /// An override decided a leaf cell-runner would have run.
    Override,
    /// A guard stopped a runaway tree.
    Limit,
}

/// A warning on the timeline.
#[derive(Debug, Clone, PartialEq)]
pub struct CellWarning {
    /// What it is about.
    pub kind: WarningKind,
    /// Executor time when it was raised.
    pub t: f64,
    /// The leaf it is about.
    pub uid: Option<Uid>,
    /// The text (cell-runner's own for [`WarningKind::Runner`]).
    pub message: String,
}

/// One robot at an instant (cell-runner's `RobotPlaybackState`, plus the leaf behind it).
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState {
    /// The robot.
    pub robot_id: String,
    /// Where it is shown ([`Schedule::display_pose_at`]).
    pub joints: [f64; 6],
    /// The segment's `op_type`, then `hold`, then `idle`.
    pub phase: String,
    /// The segment's operation (none before the robot's first motion).
    pub op_id: Option<String>,
    /// The segment's `op_type`.
    pub op_type: Option<String>,
    /// Through the trajectory as planned, 0..=1; a cut stops it where the robot stopped.
    pub progress: f64,
    /// The leaf whose segment this is.
    pub uid: Option<Uid>,
    /// The segment was cut (as of the tick shown).
    pub cut: bool,
}

/// A plan compiled into an absolute, seekable timeline.
#[derive(Debug, Clone)]
pub struct Schedule {
    /// Every robot: those known up front, then each as it first moves.
    pub robots: Vec<String>,
    /// Every segment in the order placed (cut and dropped ones included).
    pub segments: Vec<Segment>,
    /// Per-robot segment indices, ascending by `t0` (dropped segments are left out).
    by_robot: BTreeMap<String, Vec<usize>>,
    home: BTreeMap<String, [f64; 6]>,
    /// Each robot's configuration at the end of its latest placed motion.
    last_q: BTreeMap<String, [f64; 6]>,
    /// Where each robot is shown before its first motion (display only).
    preroll: BTreeMap<String, [f64; 6]>,
    /// When the tree finished (0 until then).
    pub duration: f64,
    /// Every warning, in the order raised.
    pub warnings: Vec<CellWarning>,
}

impl Schedule {
    /// An empty timeline for `robots`, using each robot's `home` configuration as
    /// its pose before its first segment.
    pub fn new(robots: &[String], home: BTreeMap<String, [f64; 6]>) -> Self {
        Self {
            robots: robots.to_vec(),
            segments: Vec::new(),
            by_robot: BTreeMap::new(),
            home,
            last_q: BTreeMap::new(),
            preroll: BTreeMap::new(),
            duration: 0.0,
            warnings: Vec::new(),
        }
    }

    /// Where to show each robot before its first motion is placed. Display only: execution
    /// uses the home configuration, like cell-runner while it compiles.
    pub fn set_preroll(&mut self, preroll: BTreeMap<String, [f64; 6]>) {
        self.preroll = preroll;
    }

    /// Compile `plan` for `robots` (cell-runner `Schedule::build`, through
    /// [`compile`](crate::compile::compile)).
    ///
    /// Fails only when the tree itself is unusable (malformed XML, unknown node
    /// IDs, undeclared ports) — problems with individual leaves become warnings.
    pub fn build(
        plan: &BtPlan,
        robots: &[String],
        home: BTreeMap<String, [f64; 6]>,
    ) -> Result<Self, String> {
        crate::compile::compile(plan, robots, home).map(|compiled| compiled.schedule)
    }

    /// Park each robot, before its first segment, in the configuration that
    /// segment STARTS from — not in the cell's home pose.
    ///
    /// Robots rarely all start moving at t=0 (a `WaitForRobot` commonly holds
    /// one arm back), and a plan's trajectories begin from whatever pose the
    /// planner homed the arms to, which is not necessarily this cell's KRL/scene
    /// home. Using the cell home for the gap meant a waiting arm sat in one pose
    /// and then TELEPORTED to the plan's starting pose the instant its first
    /// segment began — visible as a jump, and misleading in the 3D view for as
    /// long as the arm was waiting.
    ///
    /// The supplied home is still used for a robot the plan never moves, since
    /// there is no trajectory to take a pose from.
    pub fn adopt_preroll_poses(&mut self) {
        let preroll: Vec<(String, [f64; 6])> = self
            .robots
            .iter()
            .filter_map(|robot| {
                let q = self
                    .by_robot
                    .get(robot)
                    .and_then(|indices| indices.first())
                    .and_then(|index| self.segments.get(*index))
                    .and_then(|segment| segment.traj.first())
                    .map(|waypoint| waypoint.q)?;
                Some((robot.clone(), q))
            })
            .collect();
        for (robot, q) in preroll {
            self.home.insert(robot, q);
        }
    }

    /// A robot's home configuration ([`FALLBACK_HOME_JOINTS`] if it has none).
    pub fn home_of(&self, robot: &str) -> [f64; 6] {
        *self.home.get(robot).unwrap_or(&FALLBACK_HOME_JOINTS)
    }

    /// Where a robot's latest placed motion ends, else its home.
    pub fn last_pose_of(&self, robot: &str) -> [f64; 6] {
        *self.last_q.get(robot).unwrap_or(&self.home_of(robot))
    }

    /// Add a warning.
    pub fn warn(&mut self, kind: WarningKind, t: f64, uid: Option<Uid>, message: String) {
        self.warnings.push(CellWarning {
            kind,
            t,
            uid,
            message,
        });
    }

    fn emit(&mut self, segment: Segment) {
        if let Some(last) = segment.traj.last() {
            self.last_q.insert(segment.robot.clone(), last.q);
        }
        let robot = segment.robot.clone();
        if !self.robots.contains(&robot) {
            self.robots.push(robot.clone());
        }
        let t0 = segment.t0;
        let index = self.segments.len();
        self.segments.push(segment);
        // Insertion point computed under SHARED borrows of `by_robot` and
        // `segments` first: taking the mutable `entry()` borrow up front would
        // conflict with reading `self.segments` to compare `t0`.
        // Kept ascending by `t0` — `pose_at` relies on that ordering.
        let at = match self.by_robot.get(&robot) {
            Some(slot) => slot
                .iter()
                .position(|i| self.segments[*i].t0 > t0)
                .unwrap_or(slot.len()),
            None => 0,
        };
        self.by_robot.entry(robot).or_default().insert(at, index);
    }

    /// Before a motion of `robot` from `start` at `t0`: does it start where the robot is, and
    /// after its previous motion? Only robots that have moved already are checked.
    fn check_continuity(&mut self, robot: &str, op: &str, start: [f64; 6], t0: f64, by: PlacedBy) {
        let Some(indices) = self.by_robot.get(robot).filter(|v| !v.is_empty()) else {
            return;
        };
        let err = max_joint_error(&self.pose_at(robot, t0), &start);
        let previous = indices
            .iter()
            .rev()
            .map(|&i| &self.segments[i])
            .find(|s| s.t0 <= t0)
            .filter(|s| s.t_done() > t0 + EPS)
            .map(|s| (s.op_id.clone(), s.t_done()));
        if err > JUMP_TOL_RAD {
            self.warn(
                WarningKind::Jump,
                t0,
                by.uid,
                format!(
                    "{robot}: {op} starts {err:.3} rad from where the robot is at t={t0:.3}s \
                     — playback jumps"
                ),
            );
        }
        if let Some((prev, t_done)) = previous {
            self.warn(
                WarningKind::Overlap,
                t0,
                by.uid,
                format!("{robot}: {op} starts at t={t0:.3}s while {prev} runs until {t_done:.3}s"),
            );
        }
    }

    /// Place an `ExecuteTrajectory` starting at `t0`.
    pub fn start_execute(&mut self, op: &ExecuteOp, t0: f64, by: PlacedBy) -> Placed {
        let first = self.segments.len();
        let mut t = t0 + op.start_delay_s.max(0.0);

        // Optional pre-move to a safe intermediate configuration.
        if let Some(target) = &op.interlock_joints {
            let from = self.last_pose_of(&op.robot);
            let op_id = format!("{}:interlock", op.op_id);
            // It starts where the robot's latest motion ends, which a kept halted motion may
            // not have reached yet.
            self.check_continuity(&op.robot, &op_id, from, t, by);
            self.emit(Segment {
                robot: op.robot.clone(),
                t0: t,
                traj: vec![
                    Waypoint { q: from, t: 0.0 },
                    Waypoint {
                        q: to_six(target),
                        t: HOME_TIME_S,
                    },
                ],
                op_id,
                op_type: "move".to_string(),
                hold_s: 0.0,
                uid: by.uid,
                placed_tick: by.tick,
                cut: None,
            });
            t += HOME_TIME_S;
        }

        let scale = if op.velocity_scale > 0.0 {
            op.velocity_scale
        } else {
            1.0
        };
        let traj: Vec<Waypoint> = op
            .traj
            .iter()
            .map(|w| Waypoint {
                q: to_six(&w.positions),
                t: w.time_s / scale,
            })
            .collect();
        let hold_s = op.hold_s.max(0.0);
        let duration = traj.last().map(|w| w.t).unwrap_or(0.0);
        if let Some(start) = traj.first() {
            self.check_continuity(&op.robot, &op.op_id, start.q, t, by);
        }
        self.emit(Segment {
            robot: op.robot.clone(),
            t0: t,
            traj,
            op_id: op.op_id.clone(),
            op_type: op.op_type.clone(),
            hold_s,
            uid: by.uid,
            placed_tick: by.tick,
            cut: None,
        });
        Placed {
            segments: first..self.segments.len(),
            done_at: t + duration + hold_s + COMPLETION_MARGIN_S,
        }
    }

    /// Place a `MoveHome` starting at `t0`: the planned return path when given,
    /// else a slow move to the robot's home.
    pub fn start_home(
        &mut self,
        robot: &str,
        planned: &[PlanWaypoint],
        t0: f64,
        by: PlacedBy,
    ) -> Placed {
        let first = self.segments.len();
        let traj: Vec<Waypoint> = if planned.is_empty() {
            vec![
                Waypoint {
                    q: self.last_pose_of(robot),
                    t: 0.0,
                },
                Waypoint {
                    q: self.home_of(robot),
                    t: HOME_RETURN_TIME_S,
                },
            ]
        } else {
            planned
                .iter()
                .map(|w| Waypoint {
                    q: to_six(&w.positions),
                    t: w.time_s,
                })
                .collect()
        };
        let duration = traj.last().map(|w| w.t).unwrap_or(0.0);
        // An unplanned home starts where the robot's latest motion ends, which a kept halted
        // motion may not have reached yet.
        if let Some(start) = traj.first() {
            self.check_continuity(robot, "home", start.q, t0, by);
        }
        self.emit(Segment {
            robot: robot.to_string(),
            t0,
            traj,
            op_id: "home".to_string(),
            op_type: "move".to_string(),
            hold_s: 0.0,
            uid: by.uid,
            placed_tick: by.tick,
            cut: None,
        });
        Placed {
            segments: first..self.segments.len(),
            done_at: t0 + duration + COMPLETION_MARGIN_S,
        }
    }

    /// Stop `placed` at `t` (tick `tick`): a segment that has not started is dropped, one that
    /// runs ends at `t` where it is, a hold ends at `t`. What happened before `t` is kept.
    /// Returns whether anything changed.
    pub fn cut(&mut self, placed: &Placed, t: f64, tick: u64) -> bool {
        let mut changed = false;
        for index in placed.segments.clone() {
            let Some(s) = self.segments.get_mut(index) else {
                continue;
            };
            if s.is_dropped() {
                continue;
            }
            let planned_t_end = s.cut.map_or(s.t_end(), |c| c.planned_t_end);
            let planned_t_done = s.cut.map_or(s.t_done(), |c| c.planned_t_done);
            let dropped = t <= s.t0 + EPS;
            if dropped {
                let robot = s.robot.clone();
                if let Some(slot) = self.by_robot.get_mut(&robot) {
                    slot.retain(|&i| i != index);
                }
            } else if t < s.t_end() - EPS {
                let local = t - s.t0;
                let q = interpolate(&s.traj, local);
                s.traj.retain(|w| w.t < local - EPS);
                s.traj.push(Waypoint { q, t: local });
                s.hold_s = 0.0;
            } else if t < s.t_done() - EPS {
                s.hold_s = t - s.t_end();
            } else {
                continue;
            }
            let s = &mut self.segments[index];
            s.cut = Some(Cut {
                at: t,
                planned_t_end,
                planned_t_done,
                dropped,
                tick,
            });
            let robot = s.robot.clone();
            self.recompute_last_q(&robot);
            changed = true;
        }
        changed
    }

    fn recompute_last_q(&mut self, robot: &str) {
        let last = self
            .segments
            .iter()
            .rev()
            .filter(|s| s.robot == robot && !s.is_dropped())
            .find_map(|s| s.traj.last().map(|w| w.q));
        match last {
            Some(q) => self.last_q.insert(robot.to_string(), q),
            None => self.last_q.remove(robot),
        };
    }

    /// A robot's configuration at absolute time `t`.
    ///
    /// Between segments the arm holds the previous segment's final pose (what
    /// [`interpolate`]'s clamping yields); before its first segment it sits at
    /// its home configuration.
    pub fn pose_at(&self, robot: &str, t: f64) -> [f64; 6] {
        let indices = match self.by_robot.get(robot) {
            Some(v) => v,
            None => return self.home_of(robot),
        };
        let mut active: Option<&Segment> = None;
        for index in indices {
            let segment = &self.segments[*index];
            if segment.t0 <= t {
                active = Some(segment);
            } else {
                break;
            }
        }
        match active {
            Some(segment) => interpolate(&segment.traj, t - segment.t0),
            None => self.home_of(robot),
        }
    }

    /// The latest segment of `robot` that started by `t`, as of tick `upto`.
    fn active_at(&self, robot: &str, t: f64, upto: u64) -> Option<&Segment> {
        let mut active: Option<&Segment> = None;
        for segment in self
            .segments
            .iter()
            .filter(|s| s.robot == robot && s.visible_at(upto))
        {
            if segment.t0 <= t && active.is_none_or(|a| segment.t0 >= a.t0) {
                active = Some(segment);
            }
        }
        active
    }

    /// Where to show `robot` at `t`, knowing only the motion placed up to tick `upto`.
    ///
    /// Like [`pose_at`](Self::pose_at), except before the robot's first motion: there it is
    /// shown where that motion starts (or at its pre-roll pose if none is placed yet), as
    /// cell-runner's playback does after `adopt_preroll_poses`.
    pub fn display_pose_at(&self, robot: &str, t: f64, upto: u64) -> [f64; 6] {
        if let Some(segment) = self.active_at(robot, t, upto) {
            return interpolate(&segment.traj, t - segment.t0);
        }
        let first = self
            .segments
            .iter()
            .filter(|s| s.robot == robot && s.visible_at(upto))
            .fold(None::<&Segment>, |first, s| match first {
                Some(f) if f.t0 <= s.t0 => Some(f),
                _ => Some(s),
            });
        match first.and_then(|s| s.traj.first()) {
            Some(w) => w.q,
            None => self
                .preroll
                .get(robot)
                .copied()
                .unwrap_or_else(|| self.home_of(robot)),
        }
    }

    /// Earliest time at or after `not_before` at which `robot` is within `tol`
    /// of `target`, given the motion placed so far.
    ///
    /// Mirrors a live joint gate, which starts polling when the wait begins and
    /// returns on the first sample within tolerance. Two cases matter: the arm
    /// may already be *holding* a matching pose when the gate opens (it finished
    /// the awaited motion earlier and has not moved since), or it may pass
    /// through the pose later.
    pub fn gate_time(
        &self,
        robot: &str,
        target: &[f64; 6],
        tol: f64,
        not_before: f64,
    ) -> Option<f64> {
        if max_joint_error(target, &self.pose_at(robot, not_before)) <= tol {
            return Some(not_before);
        }
        let indices = self.by_robot.get(robot)?;
        let mut candidates: Vec<f64> = Vec::new();
        for index in indices {
            let segment = &self.segments[*index];
            if segment.t_done() < not_before {
                continue;
            }
            for waypoint in &segment.traj {
                let t_abs = segment.t0 + waypoint.t;
                if t_abs > not_before {
                    candidates.push(t_abs);
                }
            }
            if segment.t_end() > not_before {
                candidates.push(segment.t_end()); // start of the hold window
            }
        }
        candidates.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        candidates
            .into_iter()
            .find(|t| max_joint_error(target, &self.pose_at(robot, *t)) <= tol)
    }

    /// Every robot at `t`, as of tick `upto` (cell-runner's `Playback::robot_states` with the
    /// playhead as a parameter).
    pub fn robot_states(&self, t: f64, upto: u64) -> Vec<RobotState> {
        self.robots
            .iter()
            .map(|robot| {
                let joints = self.display_pose_at(robot, t, upto);
                match self.active_at(robot, t, upto) {
                    None => RobotState {
                        robot_id: robot.clone(),
                        joints,
                        phase: "idle".to_string(),
                        op_id: None,
                        op_type: None,
                        progress: 0.0,
                        uid: None,
                        cut: false,
                    },
                    Some(segment) => {
                        let local = t - segment.t0;
                        let (t_end, t_done) = segment.extent_at(upto);
                        let duration = t_end - segment.t0;
                        let phase = if local < duration {
                            segment.op_type.clone()
                        } else if local < t_done - segment.t0 {
                            "hold".to_string()
                        } else {
                            "idle".to_string()
                        };
                        // Through the trajectory as planned: a cut stops the progress where
                        // the robot stopped, like `CellLeaves::progress`.
                        let cut = segment.cut.filter(|c| c.tick <= upto);
                        let planned = cut.map_or(duration, |c| c.planned_t_end - segment.t0);
                        let reached = cut.map_or(local, |c| local.min(c.at - segment.t0));
                        RobotState {
                            robot_id: robot.clone(),
                            joints,
                            phase,
                            op_id: Some(segment.op_id.clone()),
                            op_type: Some(segment.op_type.clone()),
                            progress: if planned <= 0.0 {
                                1.0
                            } else {
                                (reached / planned).clamp(0.0, 1.0)
                            },
                            uid: segment.uid,
                            cut: cut.is_some(),
                        }
                    }
                }
            })
            .collect()
    }

    /// Every segment of `robot`, dropped ones included, in the order they were placed.
    pub fn segments_of<'a>(&'a self, robot: &'a str) -> impl Iterator<Item = &'a Segment> {
        self.segments.iter().filter(move |s| s.robot == robot)
    }

    /// The end of the timeline: the run's duration, or the end of the latest motion planned
    /// (cut ones as planned).
    pub fn horizon(&self) -> f64 {
        self.segments
            .iter()
            .map(|s| {
                s.cut
                    .map_or(s.t_done(), |c| c.planned_t_done.max(s.t_done()))
            })
            .fold(self.duration, f64::max)
    }

    /// Distinct non-`move` operation ids, i.e. real process operations (dropped ones not
    /// counted).
    pub fn operation_count(&self) -> usize {
        let mut ops: Vec<&str> = self
            .segments
            .iter()
            .filter(|s| s.op_type != "move" && !s.is_dropped())
            .map(|s| s.op_id.as_str())
            .collect();
        ops.sort_unstable();
        ops.dedup();
        ops.len()
    }

    /// Distinct `ExecuteTrajectory` operations placed so far (homes, interlock pre-moves and
    /// dropped segments not counted), to compare with a plan's `num_operations`.
    pub fn placed_count(&self) -> usize {
        let mut ops: Vec<&str> = self
            .segments
            .iter()
            .filter(|s| !s.is_dropped() && s.op_id != "home" && !s.op_id.ends_with(":interlock"))
            .map(|s| s.op_id.as_str())
            .collect();
        ops.sort_unstable();
        ops.dedup();
        ops.len()
    }

    /// A human-readable timeline, for logs (dropped segments left out).
    pub fn describe(&self) -> String {
        let placed: Vec<&Segment> = self.segments.iter().filter(|s| !s.is_dropped()).collect();
        let mut lines = vec![format!(
            "{} segments, {:.2}s total, robots={}",
            placed.len(),
            self.duration,
            self.robots.join(", ")
        )];
        let mut ordered = placed;
        ordered.sort_by(|a, b| {
            a.t0.partial_cmp(&b.t0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.robot.cmp(&b.robot))
        });
        for segment in ordered {
            lines.push(format!(
                "  {:8.2} -> {:8.2}s  {:<10} {:<6} {:<20} ({} wp, hold {:.1}s)",
                segment.t0,
                segment.t_done(),
                segment.robot,
                segment.op_type,
                segment.op_id,
                segment.traj.len(),
                segment.hold_s
            ));
        }
        for warning in &self.warnings {
            lines.push(format!("  WARN {}", warning.message));
        }
        lines.join("\n")
    }
}
