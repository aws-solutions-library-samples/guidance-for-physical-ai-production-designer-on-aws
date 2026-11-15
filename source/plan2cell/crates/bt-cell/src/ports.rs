//! Reading a cell leaf's ports the way cell-runner (behaviortree-rs) does.
//!
//! That differs from [`Executor::port_value`](bt_exec::Executor::port_value) in three ways
//! (cell-runner `btree.rs:50-242`, behaviortree-rs `NodeConfig::get_input`):
//!
//! - A missing or empty attribute falls back to cell-runner's Rust defaults
//!   ([`default_of`]), never to `<TreeNodesModel>` defaults, which behaviortree-rs ignores.
//! - `{key}` is a blackboard pointer whenever the value starts with `{` and ends with `}`;
//!   a missing entry is an error, not an empty value. So an inline JSON object written into
//!   an attribute reads as a pointer to nothing.
//! - Any other literal is parsed as JSON (numbers, booleans, inline arrays), falling back to
//!   a string.
//!
//! [`PortSource`] lets the same code read a running leaf ([`LeafCtx`]) and a node at rest
//! ([`AtRest`]), for the inspector and for deciding whether a tree is a cell plan at all.

use bt_core::model::{Node, Uid};
use bt_exec::{Executor, LeafCtx};
use serde_json::Value;

use crate::plan::PlanWaypoint;
use crate::schedule::ExecuteOp;

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

/// Ports a cell leaf declares (cell-runner `btree.rs:50-84`), `name` included: behaviortree-rs
/// treats `name` as a port. Empty for anything that is not a cell leaf.
pub fn leaf_ports(node_id: &str) -> &'static [&'static str] {
    match node_id {
        "ExecuteTrajectory" => EXECUTE_PORTS,
        "MoveHome" => HOME_PORTS,
        "WaitForRobot" => WAIT_PORTS,
        "CheckSafety" => SAFETY_PORTS,
        _ => &[],
    }
}

/// cell-runner's default for a port left out or empty (`btree.rs:263-276`, `:486`).
pub fn default_of(node_id: &str, port: &str) -> Option<&'static str> {
    match (node_id, port) {
        ("ExecuteTrajectory", "operation_type") => Some("weld"),
        ("ExecuteTrajectory", "hold_duration_s") => Some("0.5"),
        ("ExecuteTrajectory", "velocity_scale") => Some("1"),
        ("ExecuteTrajectory", "has_interlock") => Some("false"),
        ("CheckSafety", "condition") => Some("estop_clear"),
        _ => None,
    }
}

/// Where a leaf's ports come from.
pub trait PortSource {
    /// The node.
    fn node(&self) -> &Node;
    /// The attribute as written (or edited), not the model default.
    fn attribute(&self, port: &str) -> Option<&str>;
    /// A blackboard entry, as the node's scope sees it.
    fn entry(&self, key: &str) -> Option<Value>;
}

impl PortSource for LeafCtx<'_> {
    fn node(&self) -> &Node {
        LeafCtx::node(self)
    }
    fn attribute(&self, port: &str) -> Option<&str> {
        LeafCtx::attribute(self, port)
    }
    fn entry(&self, key: &str) -> Option<Value> {
        LeafCtx::entry(self, key)
    }
}

/// A node of an executor's tree, read outside a tick.
#[derive(Clone, Copy)]
pub struct AtRest<'a> {
    /// The executor whose tree and blackboard are read.
    pub ex: &'a Executor,
    /// The node.
    pub uid: Uid,
}

impl PortSource for AtRest<'_> {
    fn node(&self) -> &Node {
        self.ex.tree().node(self.uid)
    }
    fn attribute(&self, port: &str) -> Option<&str> {
        self.node().port(port)?.value.as_deref()
    }
    fn entry(&self, key: &str) -> Option<Value> {
        self.ex.blackboard_entry(self.uid, key)
    }
}

/// What a port reads.
#[derive(Clone, Debug, PartialEq)]
pub enum Read {
    /// Set: the literal, or the blackboard entry it points to.
    Value(Value),
    /// Not set, and no default.
    Unset,
    /// `{key}` with no such entry.
    MissingEntry(String),
}

/// A port's value, behaviortree-rs style (see the module docs).
pub fn read<S: PortSource + ?Sized>(src: &S, name: &str) -> Read {
    let raw = match src.attribute(name).filter(|v| !v.is_empty()) {
        Some(raw) => raw,
        None => match default_of(&src.node().id, name) {
            Some(default) => default,
            None => return Read::Unset,
        },
    };
    read_raw(src, raw)
}

/// A value as written, read the way behaviortree-rs reads a port (see the module docs).
pub fn read_raw<S: PortSource + ?Sized>(src: &S, raw: &str) -> Read {
    // behaviortree-rs `strip_bb_pointer`: no trimming, any `{…}`.
    if raw.len() >= 2 && raw.starts_with('{') && raw.ends_with('}') {
        let key = &raw[1..raw.len() - 1];
        return match src.entry(key) {
            Some(value) => Read::Value(value),
            None => Read::MissingEntry(key.to_string()),
        };
    }
    Read::Value(serde_json::from_str(raw).unwrap_or_else(|_| Value::String(raw.to_string())))
}

/// A set value; `null`, unset and a missing entry all count as unset (cell-runner `port`).
fn set(read: Read) -> Option<Value> {
    match read {
        Read::Value(v) if !v.is_null() => Some(v),
        _ => None,
    }
}

/// A value as text; an empty string counts as unset, other values are stringified.
fn text(value: Value) -> Option<String> {
    match value {
        Value::String(s) if s.is_empty() => None,
        Value::String(s) => Some(s),
        other => Some(other.to_string()),
    }
}

/// A set port; `null`, unset and a missing entry all count as unset (cell-runner `port`).
pub fn port<S: PortSource + ?Sized>(src: &S, name: &str) -> Option<Value> {
    set(read(src, name))
}

/// A port as text; an empty string counts as unset, other values are stringified.
pub fn port_str<S: PortSource + ?Sized>(src: &S, name: &str) -> Option<String> {
    text(port(src, name)?)
}

/// The `name` port as text, read like [`port_str`]. behaviortree-rs reads `name` like any
/// other port of a cell leaf; bt-core keeps it out of the ports as the instance name, so it is
/// read from the attribute as written (`None` when not written; a written name counts even when
/// it equals the ID).
pub fn name_port<S: PortSource + ?Sized>(src: &S) -> Option<String> {
    let node = src.node();
    if !node.attributes.iter().any(|a| a == "name") {
        return None;
    }
    text(set(read_raw(src, &node.name))?)
}

/// A number, or a string that parses as one.
pub fn port_f64<S: PortSource + ?Sized>(src: &S, name: &str) -> Option<f64> {
    match port(src, name)? {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse().ok(),
        _ => None,
    }
}

/// A bool, a number (≠ 0 is true), or `1`/`true`/`True`/`TRUE` and the `0`/`false` spellings.
pub fn port_bool<S: PortSource + ?Sized>(src: &S, name: &str) -> Option<bool> {
    match port(src, name)? {
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
pub fn port_trajectory<S: PortSource + ?Sized>(
    src: &S,
    name: &str,
) -> Result<Option<Vec<PlanWaypoint>>, String> {
    match read(src, name) {
        Read::Unset => Ok(None),
        Read::MissingEntry(key) => Err(format!("blackboard has no `{key}`")),
        Read::Value(value) if value.is_null() => Ok(None),
        Read::Value(value) => serde_json::from_value(value)
            .map(Some)
            .map_err(|e| format!("`{name}` is not a waypoint list: {e}")),
    }
}

/// An `ExecuteTrajectory` leaf's ports, or the warning cell-runner gives when it skips the
/// leaf (`btree.rs:216-242`).
pub fn read_execute<S: PortSource + ?Sized>(src: &S) -> Result<ExecuteOp, String> {
    let robot = port_str(src, "robot_ns")
        .ok_or_else(|| "ExecuteTrajectory without robot_ns — skipped".to_string())?;
    let op_id = port_str(src, "operation_id").unwrap_or_else(|| "op".to_string());
    let traj = match port_trajectory(src, "trajectory") {
        Ok(Some(traj)) if !traj.is_empty() => traj,
        Ok(_) => {
            return Err(format!(
                "ExecuteTrajectory {op_id} has no trajectory — skipped"
            ));
        }
        Err(e) => {
            return Err(format!(
                "ExecuteTrajectory {op_id} has no trajectory ({e}) — skipped"
            ));
        }
    };
    let interlock_joints = if port_bool(src, "has_interlock").unwrap_or(false) {
        port(src, "interlock_target")
            .and_then(|t| t.get("joint_positions").cloned())
            .and_then(|j| serde_json::from_value::<Vec<f64>>(j).ok())
    } else {
        None
    };
    Ok(ExecuteOp {
        robot,
        op_id,
        op_type: port_str(src, "operation_type").unwrap_or_else(|| "weld".to_string()),
        traj,
        start_delay_s: port_f64(src, "start_delay_s").unwrap_or(0.0),
        hold_s: port_f64(src, "hold_duration_s").unwrap_or(0.0),
        velocity_scale: port_f64(src, "velocity_scale").unwrap_or(1.0),
        interlock_joints,
    })
}
