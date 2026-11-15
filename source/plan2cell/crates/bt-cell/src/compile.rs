//! A whole plan compiled in one go, the way cell-runner does it (`btree.rs:536-620`,
//! `trajectory.rs:463-486`): every cell leaf native, stalled gates released, then each robot
//! parked where its first motion starts.
//!
//! The parity tests compare this against cell-runner's numbers; the GUI runs the same leaves
//! step by step instead.

use std::collections::BTreeMap;

use bt_core::Document;
use bt_exec::{Executor, RunOptions, Stop};

use crate::compat::{self, Fails};
use crate::install;
use crate::joints::MAX_TICKS;
use crate::leaves::{CellLeaves, CellOptions};
use crate::plan::BtPlan;
use crate::schedule::Schedule;

/// A compiled plan: the timeline, and the executor that produced it (statuses, log, trace).
pub struct Compiled {
    /// The motion placed, each robot parked where its first motion starts.
    pub schedule: Schedule,
    /// The finished run, with the cell leaves still installed.
    pub executor: Executor,
}

/// Compile `plan` for `robots`, using each robot's `home` configuration until it moves.
///
/// Fails, with cell-runner's error text, only when the tree itself is unusable: malformed XML,
/// a file or tree behaviortree-rs would not build ([`compat::check_xml`], [`compat::check`]),
/// a tick it would fail (R4, R6), a stall, or a tree that never finishes. Problems with
/// individual leaves become warnings.
pub fn compile(
    plan: &BtPlan,
    robots: &[String],
    home: BTreeMap<String, [f64; 6]>,
) -> Result<Compiled, String> {
    let invalid = |e: String| format!("invalid BehaviorTree: {e}");
    let document = Document::parse(&plan.xml).map_err(invalid)?;
    if let Some(issue) = compat::check_xml(&plan.xml) {
        return Err(invalid(issue.message));
    }
    let tree = document
        .instantiate(document.main_tree_id())
        .map_err(invalid)?;
    let issues = compat::check(&tree);
    if let Some(issue) = issues.iter().find(|i| i.fails == Fails::Build) {
        return Err(invalid(issue.message.clone()));
    }
    let mut executor = Executor::new(tree, plan.initial_blackboard());
    install(&mut executor, CellOptions::cell_runner(robots, home));
    let report = executor.run(RunOptions {
        max_ticks: MAX_TICKS,
        release_stalls: true,
        ..RunOptions::VIRTUAL
    });
    // cell-runner stops at the first tick behaviortree-rs fails; up to there the runs agree.
    let failed = issues
        .iter()
        .filter_map(|issue| Some((issue.fails_at(executor.tree(), executor.trace())?, issue)))
        .min_by_key(|(tick, _)| *tick);
    if let Some((tick, issue)) = failed {
        let t = executor.time_at_tick(tick).unwrap_or_default();
        return Err(format!(
            "BehaviorTree tick failed at t={t:.2}s: {}",
            issue.message
        ));
    }
    let now = executor.now();
    match report.stop {
        Stop::Finished(_) => {}
        Stop::TickLimit => {
            return Err(format!(
                "BehaviorTree did not finish within {MAX_TICKS} ticks (t={now:.2}s)"
            ));
        }
        _ => {
            return Err(format!(
                "BehaviorTree stalled at t={now:.2}s: nodes are RUNNING but none can progress"
            ));
        }
    }
    let mut schedule = executor
        .leaf_handler::<CellLeaves>()
        .expect("the cell leaves are installed")
        .schedule()
        .clone();
    schedule.adopt_preroll_poses();
    Ok(Compiled { schedule, executor })
}
