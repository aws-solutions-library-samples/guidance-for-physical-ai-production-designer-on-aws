//! BehaviorTree.CPP executor for trajectory plans, built on `behaviortree-rs`.
//!
//! Plans arrive as BehaviorTree.CPP v4 XML plus a blackboard
//! ([`BtPlan`]). Control flow — `Sequence`, `Parallel` and its
//! `success_count`/`failure_count` thresholds, `Fallback`, the standard
//! decorators — is the crate's; this module only supplies the cell's leaf
//! nodes and a clock for them to run against.
//!
//! # Virtual time
//!
//! The tree is ticked against a *virtual* clock rather than the wall clock, so
//! a whole plan compiles into a [`Schedule`] in milliseconds. Leaves never
//! block: an action places its motion on the schedule when it starts, reports
//! `Running` until the clock reaches its completion time, and registers that
//! time as a wake-up. Between ticks the clock jumps straight to the earliest
//! wake-up, so no tick is spent on a tree in which nothing can change.
//!
//! | node                | behaviour                                                              |
//! |---------------------|------------------------------------------------------------------------|
//! | `ExecuteTrajectory` | `start_delay_s`, optional interlock pre-move, trajectory scaled by `velocity_scale`, then `hold_duration_s` + settle |
//! | `MoveHome`          | the planned return path when present, else a slow move to home          |
//! | `WaitForRobot`      | `Running` until every awaited robot is within tolerance of its target   |
//! | `CheckSafety`       | `Success` (nothing to gate during playback)                             |
//! | `Delay`             | holds its child back by `delay_msec` of virtual time                    |

use std::sync::{Arc, Mutex, MutexGuard};

use behaviortree_rs::basic_types::{FromString, NodeStatus, PortsList};
use behaviortree_rs::blackboard::Blackboard;
use behaviortree_rs::bt_node;
use behaviortree_rs::macros::{
    define_ports, input_port, register_action_node, register_decorator_node,
};
use behaviortree_rs::nodes::{NodeConfig, NodeError, NodeResult};
use behaviortree_rs::tree::Factory;
use serde_json::Value;

use crate::trajectory::{
    max_joint_error, to_six, BtPlan, ExecuteOp, JointDependency, PlanWaypoint, Schedule,
    DEFAULT_INTERLOCK_TOL_RAD,
};

/// Two instants closer than this are the same instant.
const EPS: f64 = 1e-9;
/// Guard against a tree that never settles (e.g. an unbounded `Repeat`).
const MAX_TICKS: usize = 1_000_000;
/// Consecutive ticks with nothing scheduled to wake before giving up.
const MAX_STALLS: usize = 64;

const EXECUTE_PORTS: &[&str] = &[
    "name",
    "robot_ns",
    "operation_id",
    "operation_type",
    "trajectory",
    "start_delay_s",
    "hold_duration_s",
    "velocity_scale",
    "has_interlock",
    "interlock_target",
    "depends_on_robots",
    "depends_on_positions",
];
const HOME_PORTS: &[&str] = &["name", "robot_ns", "trajectory"];
const WAIT_PORTS: &[&str] = &[
    "name",
    "robot_ns",
    "depends_on_robots",
    "depends_on_joints",
    "depends_on_positions",
];
const SAFETY_PORTS: &[&str] = &["name", "condition"];

/// Ports a cell leaf declares, for carrying flat JSON fields over into XML.
/// Empty for anything that is not one of this module's leaves.
pub fn leaf_ports(node_id: &str) -> &'static [&'static str] {
    match node_id {
        "ExecuteTrajectory" => EXECUTE_PORTS,
        "MoveHome" => HOME_PORTS,
        "WaitForRobot" => WAIT_PORTS,
        "CheckSafety" => SAFETY_PORTS,
        _ => &[],
    }
}

// ───────────────────────────── values and clock ─────────────────────────────

/// A port or blackboard value.
///
/// Blackboard entries are stored as-is; a literal port string is read as JSON
/// when it parses (numbers, booleans, inline arrays) and as a plain string
/// otherwise, so one type serves every port.
#[derive(Debug, Clone)]
pub struct BbValue(pub Arc<Value>);

impl FromString for BbValue {
    type Err = std::convert::Infallible;

    fn from_string(value: impl AsRef<str>) -> Result<Self, Self::Err> {
        // behaviortree-rs hands over attribute values still XML-escaped.
        let text = xml_unescape(value.as_ref());
        let parsed = serde_json::from_str(&text).unwrap_or(Value::String(text));
        Ok(BbValue(Arc::new(parsed)))
    }
}

fn xml_unescape(text: &str) -> String {
    if !text.contains('&') {
        return text.to_string();
    }
    text.replace("&quot;", "\"")
        .replace("&apos;", "'")
        .replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&amp;", "&")
}

struct Clock {
    now: f64,
    /// Ticks issued so far; the current tick's number while ticking.
    tick: u64,
    /// Instants some node asked to be ticked again at, this tick.
    wakes: Vec<f64>,
    /// This tick placed new motion, so gates must re-evaluate at this instant.
    moved: bool,
    /// Set when the tree stalls: waits started before this tick give up.
    release_before: Option<u64>,
    schedule: Option<Schedule>,
}

impl Clock {
    fn schedule(&mut self) -> &mut Schedule {
        self.schedule.as_mut().expect("schedule is present while ticking")
    }

    /// `Success` once the clock has reached `t`, else `Running` with a wake-up.
    fn await_until(&mut self, t: f64) -> NodeStatus {
        if self.now + EPS >= t {
            NodeStatus::Success
        } else {
            self.wakes.push(t);
            NodeStatus::Running
        }
    }
}

/// The clock shared by every leaf of one tree.
#[derive(Clone)]
pub struct SharedClock(Arc<Mutex<Clock>>);

impl std::fmt::Debug for SharedClock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("SharedClock")
    }
}

impl SharedClock {
    fn lock(&self) -> MutexGuard<'_, Clock> {
        self.0.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

// ─────────────────────────────── port access ───────────────────────────────

fn port(config: &mut NodeConfig, name: &str) -> Option<Value> {
    config
        .get_input::<BbValue>(name)
        .ok()
        .map(|v| (*v.0).clone())
        .filter(|v| !v.is_null())
}

fn port_str(config: &mut NodeConfig, name: &str) -> Option<String> {
    match port(config, name)? {
        Value::String(s) if s.is_empty() => None,
        Value::String(s) => Some(s),
        other => Some(other.to_string()),
    }
}

fn port_f64(config: &mut NodeConfig, name: &str) -> Option<f64> {
    match port(config, name)? {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse().ok(),
        _ => None,
    }
}

fn port_bool(config: &mut NodeConfig, name: &str) -> Option<bool> {
    match port(config, name)? {
        Value::Bool(b) => Some(b),
        Value::Number(n) => Some(n.as_f64() != Some(0.0)),
        Value::String(s) => match s.trim() {
            "1" | "true" | "True" | "TRUE" => Some(true),
            "0" | "false" | "False" | "FALSE" => Some(false),
            _ => None,
        },
        _ => None,
    }
}

/// A trajectory port: `Ok(None)` when the port is unset, an error when it is
/// set but its blackboard entry is missing or is not a waypoint list.
fn port_trajectory(config: &mut NodeConfig, name: &str) -> Result<Option<Vec<PlanWaypoint>>, String> {
    match config.get_input::<BbValue>(name) {
        Ok(value) if value.0.is_null() => Ok(None),
        Ok(value) => serde_json::from_value((*value.0).clone())
            .map(Some)
            .map_err(|e| format!("`{name}` is not a waypoint list: {e}")),
        Err(NodeError::PortError(_)) => Ok(None),
        Err(NodeError::BlackboardError(key)) => Err(format!("blackboard has no `{key}`")),
        Err(e) => Err(e.to_string()),
    }
}

fn read_execute(config: &mut NodeConfig) -> Result<ExecuteOp, String> {
    let robot = port_str(config, "robot_ns")
        .ok_or_else(|| "ExecuteTrajectory without robot_ns — skipped".to_string())?;
    let op_id = port_str(config, "operation_id").unwrap_or_else(|| "op".to_string());
    let traj = match port_trajectory(config, "trajectory") {
        Ok(Some(traj)) if !traj.is_empty() => traj,
        Ok(_) => return Err(format!("ExecuteTrajectory {op_id} has no trajectory — skipped")),
        Err(e) => return Err(format!("ExecuteTrajectory {op_id} has no trajectory ({e}) — skipped")),
    };
    let interlock_joints = if port_bool(config, "has_interlock").unwrap_or(false) {
        port(config, "interlock_target")
            .and_then(|t| t.get("joint_positions").cloned())
            .and_then(|j| serde_json::from_value::<Vec<f64>>(j).ok())
    } else {
        None
    };
    Ok(ExecuteOp {
        robot,
        op_id,
        op_type: port_str(config, "operation_type").unwrap_or_else(|| "weld".to_string()),
        traj,
        start_delay_s: port_f64(config, "start_delay_s").unwrap_or(0.0),
        hold_s: port_f64(config, "hold_duration_s").unwrap_or(0.0),
        velocity_scale: port_f64(config, "velocity_scale").unwrap_or(1.0),
        interlock_joints,
    })
}

// ─────────────────────────────── leaf nodes ────────────────────────────────

/// Replays one planned trajectory.
#[bt_node(StatefulActionNode)]
pub struct ExecuteTrajectory {
    clock: SharedClock,
    #[bt(default)]
    done_at: f64,
}

#[bt_node(
    StatefulActionNode,
    ports = ports,
    on_start = on_start,
    on_running = on_running,
)]
impl ExecuteTrajectory {
    fn ports() -> PortsList {
        // Defaults as declared in the planner's TreeNodesModel.
        define_ports!(
            input_port!("name"),
            input_port!("robot_ns"),
            input_port!("operation_id"),
            input_port!("operation_type", String::from("weld")),
            input_port!("trajectory"),
            input_port!("start_delay_s"),
            input_port!("hold_duration_s", 0.5),
            input_port!("velocity_scale", 1.0),
            input_port!("has_interlock", false),
            input_port!("interlock_target"),
            input_port!("depends_on_robots"),
            input_port!("depends_on_positions")
        )
    }

    async fn on_start(&mut self) -> NodeResult {
        let op = read_execute(&mut node_.config);
        let handle = self.clock.clone();
        let mut clock = handle.lock();
        let now = clock.now;
        match op {
            Ok(op) => {
                self.done_at = clock.schedule().start_execute(&op, now);
                clock.moved = true;
                Ok(clock.await_until(self.done_at))
            }
            Err(warning) => {
                clock.schedule().warn(warning);
                Ok(NodeStatus::Success)
            }
        }
    }

    async fn on_running(&mut self) -> NodeResult {
        let handle = self.clock.clone();
        let mut clock = handle.lock();
        Ok(clock.await_until(self.done_at))
    }
}

/// Returns a robot to home.
#[bt_node(StatefulActionNode)]
pub struct MoveHome {
    clock: SharedClock,
    #[bt(default)]
    done_at: f64,
}

#[bt_node(
    StatefulActionNode,
    ports = ports,
    on_start = on_start,
    on_running = on_running,
)]
impl MoveHome {
    fn ports() -> PortsList {
        define_ports!(
            input_port!("name"),
            input_port!("robot_ns"),
            input_port!("trajectory")
        )
    }

    async fn on_start(&mut self) -> NodeResult {
        let robot = port_str(&mut node_.config, "robot_ns");
        let planned = port_trajectory(&mut node_.config, "trajectory");
        let handle = self.clock.clone();
        let mut clock = handle.lock();
        let now = clock.now;
        let Some(robot) = robot else {
            clock.schedule().warn("MoveHome without robot_ns — skipped".to_string());
            return Ok(NodeStatus::Success);
        };
        let planned = planned.unwrap_or_else(|e| {
            clock
                .schedule()
                .warn(format!("MoveHome {robot}: {e} — moving home unplanned"));
            None
        });
        self.done_at = clock
            .schedule()
            .start_home(&robot, planned.as_deref().unwrap_or(&[]), now);
        clock.moved = true;
        Ok(clock.await_until(self.done_at))
    }

    async fn on_running(&mut self) -> NodeResult {
        let handle = self.clock.clone();
        let mut clock = handle.lock();
        Ok(clock.await_until(self.done_at))
    }
}

/// Holds its branch until other robots reach awaited joint configurations.
#[bt_node(StatefulActionNode)]
pub struct WaitForRobot {
    clock: SharedClock,
    #[bt(default)]
    label: String,
    #[bt(default)]
    pending: Vec<JointDependency>,
    #[bt(default)]
    started_tick: u64,
}

#[bt_node(
    StatefulActionNode,
    ports = ports,
    on_start = on_start,
    on_running = on_running,
)]
impl WaitForRobot {
    fn ports() -> PortsList {
        define_ports!(
            input_port!("name"),
            input_port!("robot_ns"),
            input_port!("depends_on_robots"),
            input_port!("depends_on_joints"),
            input_port!("depends_on_positions")
        )
    }

    async fn on_start(&mut self) -> NodeResult {
        let label = port_str(&mut node_.config, "name")
            .or_else(|| port_str(&mut node_.config, "robot_ns"))
            .unwrap_or_else(|| "?".to_string());
        let joints = port(&mut node_.config, "depends_on_joints");
        let positions = port(&mut node_.config, "depends_on_positions");

        let handle = self.clock.clone();
        let mut clock = handle.lock();
        let pending: Vec<JointDependency> = match joints {
            None => Vec::new(),
            Some(value) => serde_json::from_value(value).unwrap_or_else(|e| {
                clock
                    .schedule()
                    .warn(format!("WaitForRobot {label}: unreadable depends_on_joints ({e}) — ignored"));
                Vec::new()
            }),
        };
        if let Some(count) = positions.as_ref().and_then(Value::as_array).map(Vec::len) {
            if count > 0 {
                clock.schedule().warn(format!(
                    "WaitForRobot {label}: {count} Cartesian gate(s) ignored (playback synthesizes no FK)"
                ));
            }
        }
        self.pending = pending
            .into_iter()
            .filter(|dep| !dep.joint_positions.is_empty())
            .collect();
        self.label = label;
        self.started_tick = clock.tick;
        Ok(poll_gate(&mut clock, &mut self.pending, self.started_tick, &self.label))
    }

    async fn on_running(&mut self) -> NodeResult {
        let handle = self.clock.clone();
        let mut clock = handle.lock();
        Ok(poll_gate(&mut clock, &mut self.pending, self.started_tick, &self.label))
    }
}

fn tolerance(dep: &JointDependency) -> f64 {
    if dep.tolerance_rad > 0.0 {
        dep.tolerance_rad
    } else {
        DEFAULT_INTERLOCK_TOL_RAD
    }
}

/// One evaluation of a joint gate at the clock's current instant.
///
/// Satisfied dependencies are dropped; for the rest, the earliest future
/// instant each could open (given the motion placed so far) becomes a wake-up.
/// A gate nothing can ever open is released once the whole tree has stalled —
/// by then every motion that could have satisfied it has already been placed.
fn poll_gate(
    clock: &mut Clock,
    pending: &mut Vec<JointDependency>,
    started_tick: u64,
    label: &str,
) -> NodeStatus {
    let now = clock.now;
    let schedule = clock.schedule.as_ref().expect("schedule is present while ticking");
    pending.retain(|dep| {
        max_joint_error(&to_six(&dep.joint_positions), &schedule.pose_at(&dep.robot, now))
            > tolerance(dep)
    });
    if pending.is_empty() {
        return NodeStatus::Success;
    }
    if clock.release_before.is_some_and(|tick| started_tick < tick) {
        for dep in pending.drain(..) {
            let tol = tolerance(&dep);
            clock.schedule().warn(format!(
                "WaitForRobot {label}: {} never reaches the awaited configuration \
                 within {tol} rad — released once nothing else could move",
                dep.robot
            ));
        }
        return NodeStatus::Success;
    }
    let opens: Vec<f64> = pending
        .iter()
        .filter_map(|dep| {
            schedule.gate_time(&dep.robot, &to_six(&dep.joint_positions), tolerance(dep), now)
        })
        .collect();
    clock.wakes.extend(opens);
    NodeStatus::Running
}

/// Safety condition; nothing to gate during playback.
#[bt_node(SyncActionNode)]
pub struct CheckSafety {}

#[bt_node(SyncActionNode, ports = ports, tick = tick)]
impl CheckSafety {
    fn ports() -> PortsList {
        define_ports!(
            input_port!("name"),
            input_port!("condition", String::from("estop_clear"))
        )
    }

    async fn tick(&mut self) -> NodeResult {
        Ok(NodeStatus::Success)
    }
}

/// BT.CPP `Delay`, in virtual time.
#[bt_node(DecoratorNode)]
pub struct Delay {
    clock: SharedClock,
    #[bt(default)]
    release_at: Option<f64>,
}

#[bt_node(DecoratorNode, ports = ports, tick = tick, halt = halt)]
impl Delay {
    fn ports() -> PortsList {
        define_ports!(input_port!("delay_msec", 0))
    }

    async fn tick(&mut self) -> NodeResult {
        let handle = self.clock.clone();
        let delay_s = port_f64(&mut node_.config, "delay_msec").unwrap_or(0.0).max(0.0) / 1000.0;
        let release_at = *self
            .release_at
            .get_or_insert_with(|| handle.lock().now + delay_s);
        if handle.lock().await_until(release_at) == NodeStatus::Running {
            return Ok(NodeStatus::Running);
        }
        node_.set_status(NodeStatus::Running);
        let child = node_
            .child()
            .ok_or_else(|| NodeError::NodeStructureError("Delay needs a child".to_string()))?;
        let status = child.execute_tick().await?;
        if matches!(status, NodeStatus::Success | NodeStatus::Failure) {
            self.release_at = None;
            node_.reset_child().await;
        }
        Ok(status)
    }

    async fn halt(&mut self) {
        self.release_at = None;
        node_.reset_child().await;
    }
}

// ──────────────────────────────── execution ────────────────────────────────

/// Tick `plan` to completion in virtual time, placing its motion on `schedule`.
pub fn execute(plan: &BtPlan, schedule: Schedule) -> Result<Schedule, String> {
    let clock = SharedClock(Arc::new(Mutex::new(Clock {
        now: 0.0,
        tick: 0,
        wakes: Vec::new(),
        moved: false,
        release_before: None,
        schedule: Some(schedule),
    })));

    let mut factory = Factory::new();
    register_action_node!(factory, "ExecuteTrajectory", ExecuteTrajectory, clock.clone());
    register_action_node!(factory, "MoveHome", MoveHome, clock.clone());
    register_action_node!(factory, "WaitForRobot", WaitForRobot, clock.clone());
    register_action_node!(factory, "CheckSafety", CheckSafety);
    register_decorator_node!(factory, "Delay", Delay, clock.clone());

    let mut blackboard = Blackboard::create();
    for (key, value) in &plan.blackboard {
        blackboard.set(key, BbValue(Arc::new(value.clone())));
    }
    let mut tree = factory
        .create_sync_tree_from_text(plan.xml.clone(), &blackboard)
        .map_err(|e| format!("invalid BehaviorTree: {e}"))?;

    let mut ticks = 0usize;
    let mut stalls = 0usize;
    let outcome = loop {
        {
            let mut c = clock.lock();
            c.tick += 1;
            c.wakes.clear();
            c.moved = false;
        }
        let status = tree.tick_exactly_once().map_err(|e| {
            format!("BehaviorTree tick failed at t={:.2}s: {e}", clock.lock().now)
        })?;
        if status != NodeStatus::Running {
            break status;
        }
        ticks += 1;
        let mut c = clock.lock();
        if ticks > MAX_TICKS {
            return Err(format!("BehaviorTree did not finish within {MAX_TICKS} ticks (t={:.2}s)", c.now));
        }
        if c.moved {
            // New motion may open a gate earlier than any wake-up already
            // registered, so re-evaluate at this same instant first.
            continue;
        }
        let now = c.now;
        let next = c
            .wakes
            .iter()
            .copied()
            .filter(|t| *t > now + EPS)
            .fold(f64::INFINITY, f64::min);
        if next.is_finite() {
            c.now = next;
            c.release_before = None;
            stalls = 0;
        } else {
            stalls += 1;
            if stalls > MAX_STALLS {
                return Err(format!(
                    "BehaviorTree stalled at t={now:.2}s: nodes are RUNNING but none can progress"
                ));
            }
            c.release_before = Some(c.tick + 1);
        }
    };
    drop(tree);

    let mut c = clock.lock();
    let now = c.now;
    let mut schedule = c.schedule.take().expect("schedule is present after ticking");
    if outcome == NodeStatus::Failure {
        schedule.warn(format!("BehaviorTree returned FAILURE at t={now:.2}s"));
    }
    schedule.duration = now;
    Ok(schedule)
}
