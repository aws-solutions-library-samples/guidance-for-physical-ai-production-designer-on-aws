//! factoryos cell-runner plans on bt-exec: the cell leaves run natively, placing their motion
//! on a timeline as the tree runs.
//!
//! cell-runner (factoryos-orchestrator `cell/cell-runner`) compiles a planner's
//! BehaviorTree.CPP plan in one go, in virtual time, into a motion schedule it plays back.
//! This crate ports that — the plan formats ([`plan`]), the schedule ([`schedule`]), how its
//! leaves read their ports ([`ports`]) and what they do ([`leaves`]) — onto bt-exec, so the
//! same plan runs step by step: every node's status is visible, the tree can be paused, and
//! leaves can be forced.
//!
//! [`install`] puts the cell leaves into an executor; [`compile`](compile::compile) runs a
//! plan the way cell-runner does, for the parity tests. [`compat`] lists what cell-runner's
//! runtime (behaviortree-rs) would refuse in a tree that btgui runs anyway.
//!
//! Pure Rust with no I/O and no clock, so it builds for wasm32.

pub mod compat;
pub mod compile;
pub mod joints;
pub mod leaves;
pub mod plan;
pub mod ports;
pub mod schedule;

use bt_core::model::{Tree, Uid};
use bt_exec::Executor;

pub use compile::{Compiled, compile};
pub use leaves::{CellLeaves, CellOptions, GateView, HaltedMotion, PendingDep};
pub use plan::{BtPlan, JointDependency, ParsedPlan, PlanWaypoint, parse_plan};
pub use schedule::{
    CellWarning, Cut, ExecuteOp, Placed, PlacedBy, RobotState, Schedule, Segment, WarningKind,
};

use ports::AtRest;

/// The leaves cell-runner registers (besides the `Delay` decorator, which bt-exec has).
pub const CELL_LEAVES: [&str; 4] = [
    "ExecuteTrajectory",
    "MoveHome",
    "WaitForRobot",
    "CheckSafety",
];

/// Whether `tree` has any cell leaf.
pub fn uses_cell_leaves(tree: &Tree) -> bool {
    tree.nodes
        .iter()
        .any(|n| n.kind.is_leaf() && CELL_LEAVES.contains(&n.id.as_str()))
}

/// Whether `ex` runs a cell plan: some cell leaf's `trajectory` reads as a non-empty waypoint
/// list (from the blackboard or inline). Trees that only use the cell leaves' names stay on
/// stubs.
pub fn is_cell_plan(ex: &Executor) -> bool {
    ex.tree().nodes.iter().any(|n| {
        n.kind.is_leaf()
            && CELL_LEAVES.contains(&n.id.as_str())
            && matches!(
                ports::port_trajectory(&AtRest { ex, uid: n.uid }, "trajectory"),
                Ok(Some(traj)) if !traj.is_empty()
            )
    })
}

/// Run the cell leaves of `ex` natively (the run starts over).
pub fn install(ex: &mut Executor, options: CellOptions) {
    ex.set_leaf_handler(Box::new(CellLeaves::new(options)));
}

/// What a cell leaf will do when it starts, read from its ports as they are now.
#[derive(Clone, Debug, PartialEq)]
pub enum Preview {
    /// The motion an `ExecuteTrajectory` places, or why it is skipped.
    Execute(Result<ExecuteOp, String>),
    /// A `MoveHome`'s robot and planned path (`Ok(None)`: no path, a slow move home).
    Home {
        robot: Option<String>,
        planned: Result<Option<Vec<PlanWaypoint>>, String>,
    },
    /// A `WaitForRobot`'s joint dependencies.
    Wait {
        label: String,
        deps: Result<Vec<JointDependency>, String>,
    },
    /// A `CheckSafety` (always SUCCESS, like cell-runner's).
    Safety { condition: String },
}

/// [`Preview`] of the cell leaf `uid` (`None` for anything else).
pub fn preview(ex: &Executor, uid: Uid) -> Option<Preview> {
    let node = ex.tree().nodes.get(uid)?;
    if !node.kind.is_leaf() {
        return None;
    }
    let src = AtRest { ex, uid };
    Some(match node.id.as_str() {
        "ExecuteTrajectory" => Preview::Execute(ports::read_execute(&src)),
        "MoveHome" => Preview::Home {
            robot: ports::port_str(&src, "robot_ns"),
            planned: ports::port_trajectory(&src, "trajectory"),
        },
        "WaitForRobot" => Preview::Wait {
            label: leaves::wait_label(&src),
            deps: match ports::port(&src, "depends_on_joints") {
                None => Ok(Vec::new()),
                Some(value) => serde_json::from_value::<Vec<JointDependency>>(value)
                    .map(|deps| {
                        deps.into_iter()
                            .filter(|d| !d.joint_positions.is_empty())
                            .collect()
                    })
                    .map_err(|e| format!("unreadable depends_on_joints ({e})")),
            },
        },
        "CheckSafety" => Preview::Safety {
            condition: ports::port_str(&src, "condition").unwrap_or_default(),
        },
        _ => return None,
    })
}
