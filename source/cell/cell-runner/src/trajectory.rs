//! Trajectory motion source — replay planned joint trajectories instead of
//! interpreting KRL.
//!
//! The cell-runner's original motion source is the KRL interpreter: Cartesian
//! `PTP`/`LIN` targets are resolved through IK + OTG into a [`JointPath`], and
//! the interpreter's `current_joints` become the cell's telemetry. This module
//! is the second motion source. It consumes an already-planned,
//! collision-checked BehaviorTree (as produced by an external motion planner)
//! and replays its precomputed joint trajectories verbatim.
//!
//! Because playback publishes through the same `krl[i].joints` telemetry field
//! the KRL path uses, the 3D viewer renders trajectory motion with no changes to
//! the viewer or the RRD stream — only the *source* of the joint angles differs.
//!
//! # Plan format
//!
//! Plans are BehaviorTree.CPP v4 trees ([`BtPlan`]): the XML the planner writes,
//! plus the blackboard its `trajectory="{traj_…}"` ports point into. Every file
//! layout is normalized to that pair on load:
//!
//! | layout                                          | tree from           | blackboard from        |
//! |-------------------------------------------------|---------------------|------------------------|
//! | `{behaviour_tree: {xml, blackboard, ..}}`       | `xml`               | `blackboard`           |
//! | `{behaviour_tree: {root, ..}}` / `{root}`       | `root`, converted   | `blackboard` + inline `precomputed_trajectory` |
//! | bare `.xml`                                     | the file            | sibling `.json`        |
//!
//! The converted `root` form also accepts the older upper-case dialect
//! (`SEQUENCE`, `EXECUTE`, `INTERLOCK_WAIT`, …) so previously recorded plans
//! still load.
//!
//! # Model
//!
//! A plan is compiled **once** into an absolute timeline ([`Schedule`]), then a
//! playhead samples it ([`Playback`]). Compilation ticks the tree with the
//! `behaviortree-rs` runtime against a virtual clock (see [`crate::btree`]), so
//! control-flow semantics are BehaviorTree.CPP's own. Compiling up front makes
//! playback deterministic and seekable, and lets a whole plan be
//! collision-checked before any robot moves.

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use serde::Deserialize;
use serde_json::{json, Map, Value};

use crate::path::JointPath;

/// Duration of the pre-move to an interlock's safe intermediate configuration.
pub const HOME_TIME_S: f64 = 2.5;
/// Settle slack appended after a trajectory's own duration elapses.
pub const COMPLETION_MARGIN_S: f64 = 1.0;
/// Duration of the fallback return-to-home when a `MoveHome` node carries no path.
pub const HOME_RETURN_TIME_S: f64 = 7.0;
/// Default tolerance for a joint-space interlock gate.
pub const DEFAULT_INTERLOCK_TOL_RAD: f64 = 0.15;
/// Fallback home configuration when a robot has none of its own.
pub const FALLBACK_HOME_JOINTS: [f64; 6] = [0.0, -1.57, 2.2, 0.0, -0.8, 0.0];

/// Sampling interval used when resampling a robot's timeline into the
/// time-uniform [`JointPath::Dense`] the collision checker consumes.
const COLLISION_SAMPLE_DT_S: f64 = 0.05;
/// Upper bound on synchronized samples per collision check, so checking a long
/// plan stays interactive.
const COLLISION_MAX_SAMPLES: usize = 400;

// ───────────────────────────── plan documents ─────────────────────────────

/// One trajectory waypoint as written in a plan file. Extra fields (e.g. the
/// planner's `phase`) are ignored.
#[derive(Debug, Clone, Deserialize)]
pub struct PlanWaypoint {
    #[serde(default)]
    pub positions: Vec<f64>,
    #[serde(default)]
    pub time_s: f64,
}

/// One joint-space dependency of a `WaitForRobot` node.
#[derive(Debug, Clone, Deserialize)]
pub struct JointDependency {
    pub robot: String,
    #[serde(default)]
    pub joint_positions: Vec<f64>,
    #[serde(default = "default_tolerance")]
    pub tolerance_rad: f64,
}

fn default_tolerance() -> f64 {
    DEFAULT_INTERLOCK_TOL_RAD
}

/// A plan as BehaviorTree.CPP sees it: v4 XML plus the blackboard its `{key}`
/// ports resolve against.
#[derive(Debug, Clone)]
pub struct BtPlan {
    pub xml: String,
    pub blackboard: Map<String, Value>,
}

/// A plan's identity plus its tree, normalized across every file layout.
pub struct ParsedPlan {
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    pub num_operations: Option<usize>,
    pub tree: BtPlan,
}

fn is_xml(text: &str) -> bool {
    text.trim_start().starts_with('<')
}

/// Parse a plan from its text: BehaviorTree.CPP XML, or any JSON layout.
pub fn parse_plan(text: &str) -> Result<ParsedPlan, String> {
    if is_xml(text) {
        return Ok(ParsedPlan {
            tree_id: None,
            cell_id: None,
            num_operations: None,
            tree: BtPlan {
                xml: text.to_string(),
                blackboard: Map::new(),
            },
        });
    }
    let value: Value =
        serde_json::from_str(text).map_err(|e| format!("invalid plan JSON: {e}"))?;
    parse_plan_value(value)
}

/// Parse a plan file.
///
/// A bare `.xml` carries only the tree; its trajectories live in the blackboard
/// of the `.json` the planner writes next to it, which is picked up here.
pub fn parse_plan_file(path: &Path) -> Result<ParsedPlan, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut parsed = parse_plan(&text)?;
    if is_xml(&text) {
        let sibling = path.with_extension("json");
        if sibling.is_file() {
            let text = std::fs::read_to_string(&sibling)
                .map_err(|e| format!("cannot read {}: {e}", sibling.display()))?;
            let side = parse_plan(&text)?;
            parsed.tree.blackboard = side.tree.blackboard;
            parsed.tree_id = side.tree_id;
            parsed.cell_id = side.cell_id;
            parsed.num_operations = side.num_operations;
        }
        if parsed.tree_id.is_none() {
            parsed.tree_id = path.file_stem().and_then(|s| s.to_str()).map(str::to_string);
        }
    }
    Ok(parsed)
}

/// Parse a plan from an already-decoded JSON value (inline `plan` parameter).
pub fn parse_plan_value(value: Value) -> Result<ParsedPlan, String> {
    let obj = value
        .as_object()
        .ok_or_else(|| "invalid plan object: expected a JSON object".to_string())?;
    let str_of = |o: &Map<String, Value>, k: &str| o.get(k).and_then(Value::as_str).map(str::to_string);

    let mut tree_id = str_of(obj, "tree_id");
    let mut cell_id = str_of(obj, "cell_id");
    let num_operations = obj
        .get("num_operations")
        .and_then(Value::as_u64)
        .map(|n| n as usize);

    let wrapped = obj
        .get("behaviour_tree")
        .or_else(|| obj.get("behavior_tree"))
        .and_then(Value::as_object);
    let (holder, root) = match wrapped {
        Some(bt) => {
            tree_id = tree_id.or_else(|| str_of(bt, "tree_id"));
            cell_id = cell_id.or_else(|| str_of(bt, "cell_id"));
            (bt, bt.get("root"))
        }
        None => (obj, obj.get("root")),
    };
    let mut blackboard = holder
        .get("blackboard")
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();

    // The XML is the BehaviorTree.CPP representation proper; `root` is the
    // planner's JSON mirror of it and only used when no XML is present.
    let xml = match (holder.get("xml").and_then(Value::as_str), root) {
        (Some(xml), _) => xml.to_string(),
        (None, Some(root)) => json_tree_to_xml(root, &mut blackboard)?,
        (None, None) if wrapped.is_some() => {
            return Err("`behaviour_tree` has neither `xml` nor `root`".to_string())
        }
        (None, None) => return Err("plan has neither `behaviour_tree` nor `root`".to_string()),
    };
    Ok(ParsedPlan {
        tree_id,
        cell_id,
        num_operations,
        tree: BtPlan { xml, blackboard },
    })
}

/// Node type in the older upper-case JSON dialect → BehaviorTree.CPP ID.
fn btcpp_node_id(node_type: &str) -> &str {
    match node_type {
        "SEQUENCE" => "Sequence",
        "PARALLEL" => "Parallel",
        "FALLBACK" => "Fallback",
        "EXECUTE" => "ExecuteTrajectory",
        "HOME" => "MoveHome",
        "INTERLOCK_WAIT" => "WaitForRobot",
        "SAFETY_GATE" => "CheckSafety",
        other => other,
    }
}

/// Render a JSON port value as a BehaviorTree.CPP attribute string.
///
/// Structured values (trajectories, dependency lists, interlock targets) go
/// onto the blackboard and the port points at them, as a BehaviorTree.CPP
/// planner writes them.
fn port_text(
    port: &str,
    value: &Value,
    blackboard: &mut Map<String, Value>,
    counter: &mut usize,
) -> String {
    match value {
        Value::String(s) => s.clone(),
        Value::Null => String::new(),
        Value::Array(_) | Value::Object(_) => {
            *counter += 1;
            let key = format!("{port}_{counter}");
            blackboard.insert(key.clone(), value.clone());
            format!("{{{key}}}")
        }
        other => other.to_string(),
    }
}

fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Convert a JSON tree (`{type, name, ports, children}`, or the older dialect
/// with flat fields and inline `precomputed_trajectory`) into v4 XML.
///
/// Inline trajectories are moved onto the blackboard and referenced by key, as
/// a BehaviorTree.CPP planner would have written them.
fn json_tree_to_xml(root: &Value, blackboard: &mut Map<String, Value>) -> Result<String, String> {
    fn write(
        node: &Value,
        blackboard: &mut Map<String, Value>,
        depth: usize,
        counter: &mut usize,
        out: &mut String,
    ) -> Result<(), String> {
        let obj = node
            .as_object()
            .ok_or_else(|| "tree node is not an object".to_string())?;
        let node_type = obj
            .get("type")
            .and_then(Value::as_str)
            .ok_or_else(|| "tree node without `type`".to_string())?;
        let tag = btcpp_node_id(node_type);

        let mut attrs: Vec<(String, String)> = Vec::new();
        if let Some(name) = obj.get("name").and_then(Value::as_str) {
            attrs.push(("name".into(), name.into()));
        }
        if let Some(ports) = obj.get("ports").and_then(Value::as_object) {
            for (key, value) in ports {
                attrs.push((key.clone(), port_text(key, value, blackboard, counter)));
            }
        }
        // Older dialect: ports as flat fields. Only fields the node declares are
        // carried over, so planner metadata alongside them stays ignorable.
        for key in crate::btree::leaf_ports(tag) {
            if let Some(value) = obj.get(*key) {
                if !attrs.iter().any(|(k, _)| k == key) {
                    attrs.push((key.to_string(), port_text(key, value, blackboard, counter)));
                }
            }
        }
        if let Some(traj) = obj.get("precomputed_trajectory").filter(|t| {
            t.as_array().is_some_and(|a| !a.is_empty())
        }) {
            attrs.retain(|(k, _)| k != "trajectory");
            let reference = port_text("traj", traj, blackboard, counter);
            attrs.push(("trajectory".into(), reference));
        }

        let indent = "  ".repeat(depth);
        out.push_str(&indent);
        out.push('<');
        out.push_str(tag);
        for (key, value) in &attrs {
            out.push_str(&format!(" {key}=\"{}\"", xml_escape(value)));
        }
        let children = obj.get("children").and_then(Value::as_array);
        match children.filter(|c| !c.is_empty()) {
            None => out.push_str("/>\n"),
            Some(children) => {
                out.push_str(">\n");
                for child in children {
                    write(child, blackboard, depth + 1, counter, out)?;
                }
                out.push_str(&format!("{indent}</{tag}>\n"));
            }
        }
        Ok(())
    }

    // behaviortree-rs cannot parse a bare leaf as a tree's root; a one-child
    // Sequence is equivalent.
    let is_leaf = root
        .get("children")
        .and_then(Value::as_array)
        .is_none_or(|c| c.is_empty());
    let wrapped;
    let root = if is_leaf {
        wrapped = json!({ "type": "Sequence", "children": [root] });
        &wrapped
    } else {
        root
    };
    let mut body = String::new();
    let mut counter = 0usize;
    write(root, blackboard, 2, &mut counter, &mut body)?;
    Ok(format!(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n  \
         <BehaviorTree ID=\"MainTree\">\n{body}  </BehaviorTree>\n</root>\n"
    ))
}

// ─────────────────────────── joint-space helpers ───────────────────────────

/// Widen a plan's positions vector to a fixed 6-axis configuration, padding
/// with zeros and dropping any excess axes.
pub fn to_six(positions: &[f64]) -> [f64; 6] {
    let mut q = [0.0f64; 6];
    for (slot, value) in q.iter_mut().zip(positions.iter()) {
        *slot = *value;
    }
    q
}

/// Smallest absolute difference between two joint angles, treating values a full
/// turn apart as the same physical configuration.
///
/// Weld poses put wrist joints near ±π, where the planner and the solver may
/// disagree on the sign; without wrapping that reads as a spurious 2π error and
/// an interlock gate would never open.
pub fn angular_diff(a: f64, b: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let d = (a - b).abs() % two_pi;
    d.min(two_pi - d)
}

/// Largest per-axis angular difference between two configurations.
pub fn max_joint_error(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| angular_diff(*x, *y))
        .fold(0.0f64, f64::max)
}

/// A resolved waypoint: a 6-axis configuration at a segment-local time.
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub q: [f64; 6],
    pub t: f64,
}

/// Configuration at segment-local time `t`, linearly interpolated and clamped at
/// both ends so a finished segment holds its final pose until the next begins.
pub fn interpolate(traj: &[Waypoint], t: f64) -> [f64; 6] {
    match (traj.first(), traj.last()) {
        (None, _) | (_, None) => FALLBACK_HOME_JOINTS,
        (Some(first), Some(last)) => {
            if t <= first.t {
                return first.q;
            }
            if t >= last.t {
                return last.q;
            }
            for pair in traj.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                if a.t <= t && t <= b.t {
                    let span = b.t - a.t;
                    let alpha = if span > 0.0 { (t - a.t) / span } else { 0.0 };
                    let mut q = [0.0f64; 6];
                    for i in 0..6 {
                        q[i] = a.q[i] + alpha * (b.q[i] - a.q[i]);
                    }
                    return q;
                }
            }
            last.q
        }
    }
}

// ───────────────────────────────── schedule ─────────────────────────────────

/// One robot motion occupying an absolute window on the compiled timeline.
#[derive(Debug, Clone)]
pub struct Segment {
    pub robot: String,
    pub t0: f64,
    pub traj: Vec<Waypoint>,
    pub op_id: String,
    pub op_type: String,
    pub hold_s: f64,
}

impl Segment {
    pub fn duration(&self) -> f64 {
        self.traj.last().map(|w| w.t).unwrap_or(0.0)
    }
    pub fn t_end(&self) -> f64 {
        self.t0 + self.duration()
    }
    pub fn t_done(&self) -> f64 {
        self.t_end() + self.hold_s
    }
}

/// An `ExecuteTrajectory` leaf's resolved ports.
#[derive(Debug, Clone)]
pub struct ExecuteOp {
    pub robot: String,
    pub op_id: String,
    pub op_type: String,
    pub traj: Vec<PlanWaypoint>,
    pub start_delay_s: f64,
    pub hold_s: f64,
    pub velocity_scale: f64,
    /// Safe intermediate configuration to pre-move to (`has_interlock`).
    pub interlock_joints: Option<Vec<f64>>,
}

/// A plan compiled into an absolute, seekable timeline.
pub struct Schedule {
    pub robots: Vec<String>,
    pub segments: Vec<Segment>,
    /// Per-robot segment indices, ascending by `t0`.
    by_robot: HashMap<String, Vec<usize>>,
    home: HashMap<String, [f64; 6]>,
    /// Each robot's configuration at the end of its latest placed motion.
    last_q: HashMap<String, [f64; 6]>,
    pub duration: f64,
    pub warnings: Vec<String>,
}

impl Schedule {
    /// Compile `plan` for `robots`, using each robot's `home` configuration as
    /// its pose before its first segment.
    ///
    /// Fails only when the tree itself is unusable (malformed XML, unknown node
    /// IDs, undeclared ports) — problems with individual leaves become warnings.
    pub fn build(
        plan: &BtPlan,
        robots: &[String],
        home: HashMap<String, [f64; 6]>,
    ) -> Result<Self, String> {
        let schedule = Schedule {
            robots: robots.to_vec(),
            segments: Vec::new(),
            by_robot: HashMap::new(),
            home,
            last_q: HashMap::new(),
            duration: 0.0,
            warnings: Vec::new(),
        };
        let mut schedule = crate::btree::execute(plan, schedule)?;
        schedule.adopt_preroll_poses();
        Ok(schedule)
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
    fn adopt_preroll_poses(&mut self) {
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

    fn home_of(&self, robot: &str) -> [f64; 6] {
        *self.home.get(robot).unwrap_or(&FALLBACK_HOME_JOINTS)
    }

    fn last_pose_of(&self, robot: &str) -> [f64; 6] {
        *self.last_q.get(robot).unwrap_or(&self.home_of(robot))
    }

    pub(crate) fn warn(&mut self, warning: String) {
        self.warnings.push(warning);
    }

    fn emit(&mut self, segment: Segment) {
        if let Some(last) = segment.traj.last() {
            self.last_q.insert(segment.robot.clone(), last.q);
        }
        let robot = segment.robot.clone();
        if !self.robots.iter().any(|r| *r == robot) {
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

    /// Place an `ExecuteTrajectory` starting at `t0`; returns when it completes
    /// (trajectory + hold + settle).
    pub(crate) fn start_execute(&mut self, op: &ExecuteOp, t0: f64) -> f64 {
        let mut t = t0 + op.start_delay_s.max(0.0);

        // Optional pre-move to a safe intermediate configuration.
        if let Some(target) = &op.interlock_joints {
            let from = self.last_pose_of(&op.robot);
            self.emit(Segment {
                robot: op.robot.clone(),
                t0: t,
                traj: vec![
                    Waypoint { q: from, t: 0.0 },
                    Waypoint { q: to_six(target), t: HOME_TIME_S },
                ],
                op_id: format!("{}:interlock", op.op_id),
                op_type: "move".to_string(),
                hold_s: 0.0,
            });
            t += HOME_TIME_S;
        }

        let scale = if op.velocity_scale > 0.0 { op.velocity_scale } else { 1.0 };
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
        self.emit(Segment {
            robot: op.robot.clone(),
            t0: t,
            traj,
            op_id: op.op_id.clone(),
            op_type: op.op_type.clone(),
            hold_s,
        });
        t + duration + hold_s + COMPLETION_MARGIN_S
    }

    /// Place a `MoveHome` starting at `t0`: the planned return path when given,
    /// else a slow move to the robot's home. Returns when it completes.
    pub(crate) fn start_home(&mut self, robot: &str, planned: &[PlanWaypoint], t0: f64) -> f64 {
        let traj: Vec<Waypoint> = if planned.is_empty() {
            vec![
                Waypoint { q: self.last_pose_of(robot), t: 0.0 },
                Waypoint { q: self.home_of(robot), t: HOME_RETURN_TIME_S },
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
        self.emit(Segment {
            robot: robot.to_string(),
            t0,
            traj,
            op_id: "home".to_string(),
            op_type: "move".to_string(),
            hold_s: 0.0,
        });
        t0 + duration + COMPLETION_MARGIN_S
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

    /// Earliest time at or after `not_before` at which `robot` is within `tol`
    /// of `target`, given the motion placed so far.
    ///
    /// Mirrors a live joint gate, which starts polling when the wait begins and
    /// returns on the first sample within tolerance. Two cases matter: the arm
    /// may already be *holding* a matching pose when the gate opens (it finished
    /// the awaited motion earlier and has not moved since), or it may pass
    /// through the pose later.
    pub(crate) fn gate_time(
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

    /// Distinct non-`move` operation ids, i.e. real process operations.
    pub fn operation_count(&self) -> usize {
        let mut ops: Vec<&str> = self
            .segments
            .iter()
            .filter(|s| s.op_type != "move")
            .map(|s| s.op_id.as_str())
            .collect();
        ops.sort_unstable();
        ops.dedup();
        ops.len()
    }

    /// Resample every robot's whole timeline into a time-uniform dense path, for
    /// the collision checker.
    ///
    /// Returns the per-robot paths plus the sample count, so the caller can build
    /// a `SceneTrajectory` without re-deriving either.
    pub fn to_joint_paths(&self) -> (HashMap<String, JointPath>, usize) {
        let samples = if self.duration <= 0.0 {
            1
        } else {
            ((self.duration / COLLISION_SAMPLE_DT_S).ceil() as usize).max(2)
        };
        let mut paths = HashMap::new();
        for robot in &self.robots {
            let qs: Vec<[f64; 6]> = (0..samples)
                .map(|k| {
                    let t = if samples <= 1 {
                        0.0
                    } else {
                        self.duration * (k as f64) / ((samples - 1) as f64)
                    };
                    self.pose_at(robot, t)
                })
                .collect();
            paths.insert(
                robot.clone(),
                JointPath::Dense {
                    qs,
                    takt_s: COLLISION_SAMPLE_DT_S,
                },
            );
        }
        (paths, samples)
    }

    /// Synchronized sample count to check, bounded so long plans stay fast.
    pub fn collision_sample_count(total_knots: usize) -> usize {
        total_knots.min(COLLISION_MAX_SAMPLES).max(1)
    }

    /// A human-readable timeline, for logs.
    pub fn describe(&self) -> String {
        let mut lines = vec![format!(
            "{} segments, {:.2}s total, robots={}",
            self.segments.len(),
            self.duration,
            self.robots.join(", ")
        )];
        let mut ordered: Vec<&Segment> = self.segments.iter().collect();
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
            lines.push(format!("  WARN {warning}"));
        }
        lines.join("\n")
    }
}

// ───────────────────────────────── playback ─────────────────────────────────

/// Per-robot state at the current playhead.
#[derive(Debug, Clone)]
pub struct RobotPlaybackState {
    pub robot_id: String,
    pub joints: [f64; 6],
    pub phase: String,
    pub op_id: Option<String>,
    pub op_type: Option<String>,
    pub progress: f64,
}

/// A playhead over a compiled [`Schedule`], plus the identity of the loaded plan.
///
/// `speed` is NOT stored here: the cell already has one speed concept
/// (`CellState.speed`, driven by the existing `set_speed` verb), and it is passed
/// into [`Playback::advance`] so there is a single source of truth.
pub struct Playback {
    pub schedule: Schedule,
    /// Plan path relative to the cell directory; `None` for an inline plan.
    pub plan: Option<String>,
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    /// Collision findings from load time, replayed verbatim into `state`.
    pub collision: Value,
    playing: bool,
    position: f64,
    last_tick: Instant,
    looping: bool,
    cycle: u64,
    /// Operation ids playback pauses at (when one of their segments starts).
    breakpoints: Vec<String>,
    /// The breakpoint playback paused at last: `(op_id, robot_id, t)`; cleared by play / seek.
    break_hit: Option<(String, String, f64)>,
}

impl Playback {
    pub fn new(
        schedule: Schedule,
        plan: Option<String>,
        tree_id: Option<String>,
        cell_id: Option<String>,
    ) -> Self {
        Self {
            schedule,
            plan,
            tree_id,
            cell_id,
            collision: json!({ "checked": false }),
            playing: false,
            position: 0.0,
            last_tick: Instant::now(),
            looping: false,
            cycle: 0,
            breakpoints: Vec::new(),
            break_hit: None,
        }
    }

    /// Pause at these operations (their `operation_id`s; `home`, `<op>:interlock` too) whenever one
    /// of their segments starts. Returns the ids the schedule does not have (still kept).
    pub fn set_breakpoints(&mut self, ops: Vec<String>) -> Vec<String> {
        let unknown = ops.iter().filter(|op| !self.schedule.segments.iter().any(|s| &s.op_id == *op)).cloned().collect();
        self.breakpoints = ops;
        unknown
    }

    /// The first breakpoint segment start in `(from, to]`.
    fn breakpoint_in(&self, from: f64, to: f64) -> Option<(String, String, f64)> {
        self.schedule
            .segments
            .iter()
            .filter(|s| s.t0 > from + 1e-9 && s.t0 <= to && self.breakpoints.contains(&s.op_id))
            .min_by(|a, b| a.t0.partial_cmp(&b.t0).unwrap_or(std::cmp::Ordering::Equal))
            .map(|s| (s.op_id.clone(), s.robot.clone(), s.t0))
    }

    /// Paused, move to the next segment start (any robot; `robot`: only its) after the playhead, or
    /// to the end. Returns the segment `(op_id, robot_id, t0)` moved to.
    pub fn step_next(&mut self, robot: Option<&str>) -> Option<(String, String, f64)> {
        self.pause();
        self.break_hit = None;
        let next = self
            .schedule
            .segments
            .iter()
            .filter(|s| s.t0 > self.position + 1e-6 && robot.is_none_or(|r| s.robot == r))
            .min_by(|a, b| a.t0.partial_cmp(&b.t0).unwrap_or(std::cmp::Ordering::Equal))
            .map(|s| (s.op_id.clone(), s.robot.clone(), s.t0));
        self.position = next.as_ref().map_or(self.schedule.duration, |n| n.2);
        next
    }

    /// Paused, move back to the previous segment start before the playhead (or the start).
    pub fn step_back(&mut self, robot: Option<&str>) -> Option<(String, String, f64)> {
        self.pause();
        self.break_hit = None;
        let previous = self
            .schedule
            .segments
            .iter()
            .filter(|s| s.t0 < self.position - 1e-6 && robot.is_none_or(|r| s.robot == r))
            .max_by(|a, b| a.t0.partial_cmp(&b.t0).unwrap_or(std::cmp::Ordering::Equal))
            .map(|s| (s.op_id.clone(), s.robot.clone(), s.t0));
        self.position = previous.as_ref().map_or(0.0, |p| p.2);
        previous
    }

    /// Paused, to the start of operation `op` (its first segment; `robot`: that robot's).
    pub fn seek_operation(&mut self, op: &str, robot: Option<&str>) -> Option<f64> {
        let t0 = self
            .schedule
            .segments
            .iter()
            .filter(|s| s.op_id == op && robot.is_none_or(|r| s.robot == r))
            .map(|s| s.t0)
            .fold(None, |a: Option<f64>, t| Some(a.map_or(t, |a| a.min(t))))?;
        self.pause();
        self.break_hit = None;
        self.position = t0.clamp(0.0, self.schedule.duration);
        Some(t0)
    }

    /// Advance the playhead by the wall time elapsed since the last call, scaled
    /// by `speed`. Called from the cell's background thread.
    pub fn advance(&mut self, speed: f64) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_tick).as_secs_f64();
        self.last_tick = now;
        if !self.playing {
            return;
        }
        let from = self.position;
        self.position += elapsed * speed.max(0.0);
        if let Some(hit) = self.breakpoint_in(from, self.position) {
            self.position = hit.2;
            self.playing = false;
            self.break_hit = Some(hit);
            return;
        }
        let end = self.schedule.duration;
        if self.position >= end {
            if self.looping && end > 0.0 {
                self.position %= end;
                self.cycle += 1;
            } else {
                self.position = end;
                self.playing = false;
            }
        }
    }

    pub fn is_playing(&self) -> bool {
        self.playing
    }

    /// Start (or resume) playback, rewinding first when parked at the end.
    pub fn play(&mut self) {
        self.last_tick = Instant::now();
        if self.position >= self.schedule.duration {
            self.position = 0.0;
        }
        self.break_hit = None;
        self.playing = true;
    }

    pub fn pause(&mut self) {
        self.last_tick = Instant::now();
        self.playing = false;
    }

    /// Pause and rewind to the start.
    pub fn stop(&mut self) {
        self.pause();
        self.position = 0.0;
        self.cycle = 0;
    }

    pub fn seek(&mut self, t: f64) {
        self.last_tick = Instant::now();
        self.break_hit = None;
        self.position = t.clamp(0.0, self.schedule.duration);
    }

    pub fn set_loop(&mut self, looping: bool) {
        self.looping = looping;
    }

    /// The robot's configuration right now, or `None` when the plan does not
    /// mention it (so the caller can leave that robot's telemetry alone).
    pub fn joints_for(&self, robot: &str) -> Option<[f64; 6]> {
        if !self.schedule.robots.iter().any(|r| r == robot) {
            return None;
        }
        Some(self.schedule.pose_at(robot, self.position))
    }

    /// Per-robot state at the playhead.
    pub fn robot_states(&self) -> Vec<RobotPlaybackState> {
        self.schedule
            .robots
            .iter()
            .map(|robot| {
                let t = self.position;
                let mut active: Option<&Segment> = None;
                for segment in self.schedule.segments.iter().filter(|s| &s.robot == robot) {
                    if segment.t0 <= t && active.map_or(true, |a| segment.t0 >= a.t0) {
                        active = Some(segment);
                    }
                }
                match active {
                    None => RobotPlaybackState {
                        robot_id: robot.clone(),
                        joints: self.schedule.pose_at(robot, t),
                        phase: "idle".to_string(),
                        op_id: None,
                        op_type: None,
                        progress: 0.0,
                    },
                    Some(segment) => {
                        let local = t - segment.t0;
                        let duration = segment.duration();
                        let phase = if local < duration {
                            segment.op_type.clone()
                        } else if local < duration + segment.hold_s {
                            "hold".to_string()
                        } else {
                            "idle".to_string()
                        };
                        RobotPlaybackState {
                            robot_id: robot.clone(),
                            joints: interpolate(&segment.traj, local),
                            phase,
                            op_id: Some(segment.op_id.clone()),
                            op_type: Some(segment.op_type.clone()),
                            progress: if duration <= 0.0 {
                                1.0
                            } else {
                                (local / duration).clamp(0.0, 1.0)
                            },
                        }
                    }
                }
            })
            .collect()
    }

    /// The `trajectory_load` response: what was compiled, and whether it collides.
    ///
    /// `timeline` lists every placed segment in start order (ties by robot), for viewers that
    /// map the cell's motion back onto the tree: `op_id` is the leaf's `operation_id` (`home`
    /// for a `MoveHome`, `<op>:interlock` for an interlock pre-move), `robot_id` its `robot_ns`.
    pub fn summary_json(&self) -> Value {
        let mut ordered: Vec<&Segment> = self.schedule.segments.iter().collect();
        ordered.sort_by(|a, b| {
            a.t0.partial_cmp(&b.t0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.robot.cmp(&b.robot))
        });
        let timeline: Vec<Value> = ordered
            .into_iter()
            .map(|segment| {
                json!({
                    "op_id": segment.op_id,
                    "robot_id": segment.robot,
                    "op_type": segment.op_type,
                    "t0": round3(segment.t0),
                    "t_end": round3(segment.t_end()),
                    "hold_end": round3(segment.t_done()),
                })
            })
            .collect();
        json!({
            "loaded": true,
            "plan": self.plan,
            "tree_id": self.tree_id,
            "cell_id": self.cell_id,
            "duration_s": round3(self.schedule.duration),
            "segments": self.schedule.segments.len(),
            "operations": self.schedule.operation_count(),
            "robots": self.schedule.robots,
            "warnings": self.schedule.warnings,
            "collision": self.collision,
            "timeline": timeline,
        })
    }

    /// The `trajectory_state` response the controller panel polls.
    pub fn state_json(&self, speed: f64) -> Value {
        let robots: Vec<Value> = self
            .robot_states()
            .into_iter()
            .map(|state| {
                json!({
                    "robot_id": state.robot_id,
                    "phase": state.phase,
                    "op_id": state.op_id,
                    "op_type": state.op_type,
                    "progress": round4(state.progress),
                    "joints": state.joints,
                })
            })
            .collect();
        json!({
            "loaded": true,
            "playing": self.playing,
            "t": round3(self.position),
            "duration": round3(self.schedule.duration),
            "speed": speed,
            "loop": self.looping,
            "cycle": self.cycle,
            "plan": self.plan,
            "tree_id": self.tree_id,
            "collision": self.collision,
            "robots": robots,
            "breakpoints": self.breakpoints,
            "break_hit": self.break_hit.as_ref().map(|(op, robot, t)| json!({"op_id": op, "robot_id": robot, "t": round3(*t)})),
        })
    }
}

/// The `trajectory_state` response when no plan is loaded.
pub fn unloaded_state_json(speed: f64) -> Value {
    json!({
        "loaded": false,
        "playing": false,
        "t": 0.0,
        "duration": 0.0,
        "speed": speed,
        "loop": false,
        "cycle": 0,
        "plan": Value::Null,
        "tree_id": Value::Null,
        "robots": Vec::<Value>::new(),
    })
}

fn round3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn round4(v: f64) -> f64 {
    (v * 10000.0).round() / 10000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(q: [f64; 6], t: f64) -> Value {
        json!({ "positions": q, "time_s": t })
    }

    /// Two sequential welds on one robot, in the older upper-case dialect.
    fn sample_plan() -> Value {
        let a = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0];
        let b = [1.0, -1.0, 1.0, 0.0, 0.0, 0.0];
        json!({
            "tree_id": "t1",
            "cell_id": "c1",
            "behaviour_tree": {
                "tree_id": "t1",
                "root": {
                    "type": "SEQUENCE",
                    "children": [
                        { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "W01",
                          "operation_type": "weld", "hold_duration_s": 0.5,
                          "precomputed_trajectory": [wp(a, 0.0), wp(b, 2.0)] },
                        { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "W02",
                          "operation_type": "weld", "hold_duration_s": 0.0,
                          "precomputed_trajectory": [wp(b, 0.0), wp(a, 1.0)] }
                    ]
                }
            }
        })
    }

    fn build(plan: Value) -> Schedule {
        let parsed = parse_plan_value(plan).expect("plan parses");
        Schedule::build(&parsed.tree, &["r1".to_string()], HashMap::new()).expect("plan compiles")
    }

    fn build_bare(plan: Value) -> Schedule {
        let parsed = parse_plan_value(plan).expect("plan parses");
        Schedule::build(&parsed.tree, &[], HashMap::new()).expect("plan compiles")
    }

    /// Two robots in a BT.CPP `Parallel`, trajectories on the blackboard.
    const BTCPP_XML: &str = r#"<?xml version="1.0" ?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Parallel success_count="-1" failure_count="1">
        <ExecuteTrajectory name="weld_r1_A" robot_ns="r1" operation_id="A" operation_type="weld" trajectory="{traj_A}" hold_duration_s="0.5" velocity_scale="1.0" has_interlock="false"/>
        <ExecuteTrajectory name="move_r2_B" robot_ns="r2" operation_id="B" operation_type="move" trajectory="{traj_B}" hold_duration_s="0.5" velocity_scale="2.0" has_interlock="false"/>
      </Parallel>
      <Parallel success_count="-1" failure_count="1">
        <MoveHome name="home_r1" robot_ns="r1" trajectory="{traj_H}"/>
        <MoveHome name="home_r2" robot_ns="r2"/>
      </Parallel>
    </Sequence>
  </BehaviorTree>
  <TreeNodesModel>
    <Action ID="ExecuteTrajectory">
      <input_port name="robot_ns"/>
    </Action>
  </TreeNodesModel>
</root>"#;

    fn btcpp_plan() -> Value {
        let z = [0.0; 6];
        let q = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
        json!({
            "tree_id": "bt_1",
            "cell_id": "c1",
            "num_operations": 2,
            "behaviour_tree": {
                "BTCPP_format": "4",
                "xml": BTCPP_XML,
                "blackboard": {
                    "traj_A": [wp(z, 0.0), wp(q, 2.0)],
                    "traj_B": [wp(z, 0.0), wp(q, 4.0)],
                    "traj_H": [wp(q, 0.0), wp(z, 1.0)]
                }
            }
        })
    }

    #[test]
    fn parses_every_file_layout() {
        assert!(parse_plan_value(sample_plan()).is_ok());
        assert!(parse_plan_value(btcpp_plan()).is_ok());
        let bare = json!({ "tree_id": "t", "root": { "type": "SEQUENCE", "children": [] } });
        assert!(parse_plan_value(bare).is_ok());
        let neither = json!({ "tree_id": "t" });
        assert!(parse_plan_value(neither).is_err());
        assert!(parse_plan(BTCPP_XML).is_ok());
    }

    #[test]
    fn legacy_dialect_converts_to_btcpp_xml() {
        let parsed = parse_plan_value(sample_plan()).unwrap();
        assert!(parsed.tree.xml.contains("<Sequence>"));
        assert!(parsed.tree.xml.contains("<ExecuteTrajectory"));
        assert!(parsed.tree.xml.contains("trajectory=\"{traj_1}\""));
        assert!(parsed.tree.blackboard.contains_key("traj_1"));
        assert!(parsed.tree.blackboard.contains_key("traj_2"));
    }

    #[test]
    fn inline_json_ports_in_xml_are_decoded() {
        let start = [0.0; 6];
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree"><Sequence>
            <WaitForRobot robot_ns="r2" depends_on_joints="[{&quot;robot&quot;:&quot;r1&quot;,&quot;joint_positions&quot;:[0.0,-1.57,2.2,0.0,-0.8,0.0]}]"/>
            <ExecuteTrajectory robot_ns="r2" operation_id="B" trajectory="{t}"/>
        </Sequence></BehaviorTree></root>"#;
        let plan = json!({ "behaviour_tree": { "xml": xml, "blackboard": { "t": [wp(start, 0.0), wp(start, 1.0)] } } });
        let schedule = build_bare(plan);
        assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
    }

    #[test]
    fn btcpp_parallel_runs_children_from_a_common_start() {
        let schedule = build_bare(btcpp_plan());
        assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
        let seg = |id: &str, robot: &str| {
            schedule
                .segments
                .iter()
                .find(|s| s.op_id == id && s.robot == robot)
                .unwrap_or_else(|| panic!("{id}/{robot} scheduled"))
                .clone()
        };
        let (a, b) = (seg("A", "r1"), seg("B", "r2"));
        assert!(a.t0.abs() < 1e-9 && b.t0.abs() < 1e-9);
        // velocity_scale 2.0 halves B's 4s trajectory.
        assert!((b.duration() - 2.0).abs() < 1e-9);
        // Both finish at 2.0 + 0.5 hold + 1.0 settle = 3.5; homes start there.
        let (h1, h2) = (seg("home", "r1"), seg("home", "r2"));
        assert!((h1.t0 - 3.5).abs() < 1e-9, "home r1 at {}", h1.t0);
        assert!((h2.t0 - 3.5).abs() < 1e-9, "home r2 at {}", h2.t0);
        // r2 has no planned return: slow fallback move, 7.0 + settle.
        assert!((schedule.duration - (3.5 + HOME_RETURN_TIME_S + COMPLETION_MARGIN_S)).abs() < 1e-9);
    }

    #[test]
    fn undeclared_port_is_rejected() {
        let mut plan = btcpp_plan();
        plan["behaviour_tree"]["xml"] =
            json!(BTCPP_XML.replace("has_interlock=\"false\"/>", "bogus=\"1\"/>"));
        let parsed = parse_plan_value(plan).unwrap();
        let err = Schedule::build(&parsed.tree, &[], HashMap::new()).err().expect("rejected");
        assert!(err.contains("bogus"), "{err}");
    }

    #[test]
    fn missing_blackboard_entry_warns_and_skips() {
        let mut plan = btcpp_plan();
        plan["behaviour_tree"]["blackboard"]
            .as_object_mut()
            .unwrap()
            .remove("traj_A");
        let schedule = build_bare(plan);
        assert!(schedule.segments.iter().all(|s| s.op_id != "A"));
        assert!(schedule.warnings.iter().any(|w| w.contains("ExecuteTrajectory A")));
    }

    #[test]
    fn delay_decorator_holds_its_child_back() {
        let z = [0.0; 6];
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree">
            <Delay delay_msec="1500"><ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{t}"/></Delay>
        </BehaviorTree></root>"#;
        let plan = json!({ "behaviour_tree": { "xml": xml, "blackboard": { "t": [wp(z, 0.0), wp(z, 1.0)] } } });
        let schedule = build_bare(plan);
        assert!((schedule.segments[0].t0 - 1.5).abs() < 1e-9);
    }

    #[test]
    fn sequence_places_segments_back_to_back() {
        let schedule = build(sample_plan());
        assert_eq!(schedule.segments.len(), 2);
        // W01: 2.0s + 0.5 hold + 1.0 settle = 3.5 -> W02 starts there.
        assert!((schedule.segments[0].t0 - 0.0).abs() < 1e-9);
        assert!((schedule.segments[1].t0 - 3.5).abs() < 1e-9);
        assert!((schedule.duration - 5.5).abs() < 1e-9);
        assert!(schedule.warnings.is_empty());
    }

    #[test]
    fn velocity_scale_shortens_a_segment() {
        let mut plan = sample_plan();
        plan["behaviour_tree"]["root"]["children"][0]["velocity_scale"] = json!(2.0);
        let schedule = build(plan);
        assert!((schedule.segments[0].duration() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn pose_holds_final_configuration_after_a_segment_ends() {
        let schedule = build(sample_plan());
        let end_of_w01 = schedule.segments[0].t_end();
        let held = schedule.pose_at("r1", end_of_w01 + 0.25);
        let last = schedule.segments[0].traj.last().unwrap().q;
        assert!(max_joint_error(&held, &last) < 1e-9);
    }

    #[test]
    fn parallel_ends_with_its_latest_child() {
        let a = [0.0; 6];
        let plan = json!({
            "root": { "type": "PARALLEL", "children": [
                { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A", "hold_duration_s": 0.0,
                  "precomputed_trajectory": [wp(a, 0.0), wp(a, 1.0)] },
                { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B", "hold_duration_s": 0.0,
                  "precomputed_trajectory": [wp(a, 0.0), wp(a, 4.0)] }
            ]}
        });
        let schedule = build_bare(plan);
        assert!((schedule.duration - 5.0).abs() < 1e-9); // 4.0 + settle
        assert_eq!(schedule.robots.len(), 2);
    }

    #[test]
    fn hold_duration_defaults_to_the_btcpp_model_value() {
        let a = [0.0; 6];
        let plan = json!({
            "root": { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
                      "precomputed_trajectory": [wp(a, 0.0), wp(a, 1.0)] }
        });
        let schedule = build_bare(plan);
        assert!((schedule.segments[0].hold_s - 0.5).abs() < 1e-9);
    }

    #[test]
    fn interlock_gate_opens_while_the_awaited_arm_holds_the_pose() {
        // r1 finishes at `target` and holds; r2's gate opens only after r1's
        // motion is already over, which the hold window must still satisfy.
        let start = [0.0; 6];
        let target = [1.5, -1.0, 1.0, 0.0, 0.0, 0.0];
        let plan = json!({
            "root": { "type": "SEQUENCE", "children": [
                { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
                  "precomputed_trajectory": [wp(start, 0.0), wp(target, 2.0)] },
                { "type": "SEQUENCE", "children": [
                    { "type": "INTERLOCK_WAIT", "id": "g1", "robot_ns": "r2",
                      "depends_on_joints": [
                        { "robot": "r1", "joint_positions": target, "tolerance_rad": 0.2 }
                      ] },
                    { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B",
                      "precomputed_trajectory": [wp(start, 0.0), wp(target, 1.0)] }
                ]}
            ]}
        });
        let schedule = build_bare(plan);
        assert!(
            schedule.warnings.is_empty(),
            "gate should resolve, got {:?}",
            schedule.warnings
        );
        let b = schedule
            .segments
            .iter()
            .find(|s| s.op_id == "B")
            .expect("B scheduled");
        assert!(b.t0 >= 2.0 - 1e-9, "B started too early: {}", b.t0);
    }

    #[test]
    fn wait_in_a_parallel_opens_when_the_other_arm_passes_the_pose() {
        // r2 waits (in parallel) for r1 to pass through `mid`, which r1 does at
        // t=1.0 on its way to `end` — well before r1's motion completes.
        let start = [0.0; 6];
        let mid = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let plan = json!({
            "root": { "type": "PARALLEL", "children": [
                { "type": "SEQUENCE", "children": [
                    { "type": "INTERLOCK_WAIT", "robot_ns": "r2",
                      "depends_on_joints": [ { "robot": "r1", "joint_positions": mid, "tolerance_rad": 0.01 } ] },
                    { "type": "EXECUTE", "robot_ns": "r2", "operation_id": "B",
                      "precomputed_trajectory": [wp(start, 0.0), wp(start, 1.0)] }
                ]},
                { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
                  "precomputed_trajectory": [wp(start, 0.0), wp(mid, 1.0), wp(end, 2.0)] }
            ]}
        });
        let schedule = build_bare(plan);
        assert!(schedule.warnings.is_empty(), "{:?}", schedule.warnings);
        let b = schedule.segments.iter().find(|s| s.op_id == "B").unwrap();
        assert!((b.t0 - 1.0).abs() < 1e-9, "B started at {}", b.t0);
    }

    #[test]
    fn unreachable_gate_warns_and_falls_back() {
        let start = [0.0; 6];
        let unreachable = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let plan = json!({
            "root": { "type": "SEQUENCE", "children": [
                { "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A",
                  "precomputed_trajectory": [wp(start, 0.0), wp(start, 1.0)] },
                { "type": "INTERLOCK_WAIT", "id": "g1", "robot_ns": "r2",
                  "depends_on_joints": [
                    { "robot": "r1", "joint_positions": unreachable, "tolerance_rad": 0.05 }
                  ] }
            ]}
        });
        let schedule = build_bare(plan);
        assert_eq!(schedule.warnings.len(), 1);
        assert!(schedule.warnings[0].contains("never reaches"));
    }

    /// The umarell planner's real output, when the sample checkout is present.
    #[test]
    fn umarell_plans_compile() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join(
            "../../../../../starter/starter-test1/sample-umarell-main/cells/welding_cell_2x/agentic_planner/plans",
        );
        let Ok(entries) = std::fs::read_dir(&dir) else {
            eprintln!("skipping: {} not present", dir.display());
            return;
        };
        let mut checked = 0;
        for entry in entries.flatten() {
            let path = entry.path();
            if !matches!(path.extension().and_then(|e| e.to_str()), Some("json" | "xml")) {
                continue;
            }
            let parsed = parse_plan_file(&path).expect("plan parses");
            let schedule = Schedule::build(&parsed.tree, &[], HashMap::new())
                .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            assert!(schedule.warnings.is_empty(), "{}: {:?}", path.display(), schedule.warnings);
            assert!(schedule.operation_count() > 0);
            assert!(schedule.duration > 0.0);
            eprintln!("{}: {}", path.display(), schedule.describe().lines().next().unwrap_or(""));
            checked += 1;
        }
        assert!(checked > 0);
    }

    #[test]
    fn angle_wrapping_treats_a_full_turn_as_identical() {
        assert!(angular_diff(std::f64::consts::PI, -std::f64::consts::PI) < 1e-9);
        assert!((angular_diff(0.0, std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn playback_advances_pauses_and_stops() {
        let schedule = build(sample_plan());
        let mut playback = Playback::new(schedule, None, None, None);
        assert!(!playback.is_playing());
        playback.play();
        assert!(playback.is_playing());
        playback.seek(2.0);
        assert!((playback.position - 2.0).abs() < 1e-9);
        playback.pause();
        assert!(!playback.is_playing());
        let held = playback.position;
        playback.advance(1.0);
        assert!((playback.position - held).abs() < 1e-9, "paused must not advance");
        playback.stop();
        assert!((playback.position - 0.0).abs() < 1e-9);
    }

    #[test]
    fn breakpoints_pause_and_steps_move_between_operations() {
        let schedule = build(sample_plan());
        let mut starts: Vec<(f64, String)> = schedule.segments.iter().map(|s| (s.t0, s.op_id.clone())).collect();
        starts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let mut playback = Playback::new(schedule, None, None, None);
        // Step forward visits each start in order, then the end.
        let first = playback.step_next(None).expect("a segment after 0");
        assert!(first.2 > 0.0 || starts[0].0 > 0.0);
        let back = playback.step_back(None);
        assert!(playback.position < first.2 + 1e-9, "{back:?}");
        // A breakpoint on a later operation stops playback right at its start.
        let (t_break, op) = starts.iter().find(|(t, _)| *t > 0.0).cloned().expect("a later segment");
        assert!(playback.set_breakpoints(vec![op.clone(), "nope".into()]).contains(&"nope".to_string()));
        playback.seek(0.0);
        playback.play();
        playback.last_tick = Instant::now() - std::time::Duration::from_secs_f64(t_break + 1000.0);
        playback.advance(1.0);
        assert!(!playback.is_playing());
        assert!((playback.position - t_break).abs() < 1e-9);
        assert_eq!(playback.state_json(1.0)["break_hit"]["op_id"], op);
        // Seeking to an operation pauses at its first start.
        assert_eq!(playback.seek_operation(&op, None), Some(t_break));
        assert!(playback.seek_operation("missing", None).is_none());
    }

    #[test]
    fn seek_is_clamped_to_the_schedule() {
        let schedule = build(sample_plan());
        let duration = schedule.duration;
        let mut playback = Playback::new(schedule, None, None, None);
        playback.seek(-5.0);
        assert!((playback.position - 0.0).abs() < 1e-9);
        playback.seek(1.0e6);
        assert!((playback.position - duration).abs() < 1e-9);
    }

    #[test]
    fn summary_lists_the_timeline_in_start_order() {
        let schedule = build(sample_plan());
        let summary = Playback::new(schedule, None, Some("t1".into()), None).summary_json();
        assert_eq!(summary["segments"], 2, "existing fields stay");
        assert_eq!(summary["tree_id"], "t1");
        let timeline = summary["timeline"].as_array().expect("a timeline");
        assert_eq!(timeline.len(), 2);
        assert_eq!(
            timeline[0],
            json!({ "op_id": "W01", "robot_id": "r1", "op_type": "weld",
                    "t0": 0.0, "t_end": 2.0, "hold_end": 2.5 })
        );
        assert_eq!(timeline[1]["op_id"], "W02");
        let t0 = timeline[1]["t0"].as_f64().unwrap();
        assert!(t0 >= 2.5, "W02 starts after W01's hold, at {t0}");
        assert!((timeline[1]["t_end"].as_f64().unwrap() - (t0 + 1.0)).abs() < 1e-3);
        assert_eq!(timeline[1]["hold_end"], timeline[1]["t_end"], "no hold");
    }

    #[test]
    fn joints_are_reported_only_for_robots_in_the_plan() {
        let schedule = build(sample_plan());
        let playback = Playback::new(schedule, None, None, None);
        assert!(playback.joints_for("r1").is_some());
        assert!(playback.joints_for("nobody").is_none());
    }

    #[test]
    fn dense_paths_are_time_uniform_and_shared_length() {
        let schedule = build(sample_plan());
        let (paths, samples) = schedule.to_joint_paths();
        assert!(samples >= 2);
        for path in paths.values() {
            match path {
                JointPath::Dense { qs, takt_s } => {
                    assert_eq!(qs.len(), samples);
                    assert!((*takt_s - COLLISION_SAMPLE_DT_S).abs() < 1e-12);
                }
                other => panic!("expected a dense path, got {other:?}"),
            }
        }
    }
}
