//! Driving the clock: tick at events only, and say why a run stopped.
//!
//! Like cell-runner's `btree::execute`, [`Executor::run`] ticks again at the same instant while
//! something asks for it (new motion, a user change that matters) and otherwise jumps straight
//! to the next wakeup. Between events nothing is ticked, so a run in chunks of real time places
//! the same motion as one virtual run. When nothing is scheduled the run stops and reports
//! what holds the tree: a breakpoint, a manual leaf, or a stalled wait.

use bt_core::NodeStatus;
use bt_core::model::Uid;

use super::{EPS, Executor, LeafPolicy};

/// Stall releases in a row without the clock moving before a run gives up
/// (cell-runner's `MAX_STALLS`).
pub const MAX_STALL_RELEASES: u32 = 64;

/// How far [`Executor::run`] may go.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunOptions {
    /// Stop after this many ticks.
    pub max_ticks: u32,
    /// Do not move the clock past this instant (the clock ends exactly here if nothing
    /// happens before).
    pub until: f64,
    /// Stop before the clock moves while a manual leaf waits.
    pub stop_on_manual: bool,
    /// Release stalled waits like cell-runner does, instead of stopping at them.
    pub release_stalls: bool,
}

impl RunOptions {
    /// Run to the end, jumping from event to event.
    pub const VIRTUAL: Self = Self {
        max_ticks: 100_000,
        until: f64::INFINITY,
        stop_on_manual: true,
        release_stalls: false,
    };

    /// Run up to instant `t`.
    pub fn until(t: f64) -> Self {
        Self {
            until: t,
            ..Self::VIRTUAL
        }
    }
}

impl Default for RunOptions {
    fn default() -> Self {
        Self::VIRTUAL
    }
}

/// Why a run stopped.
#[derive(Clone, Debug, PartialEq)]
pub enum Stop {
    /// The tree finished with this status.
    Finished(NodeStatus),
    /// Nodes held at their breakpoints.
    Breakpoint(Vec<Uid>),
    /// RUNNING manual leaves that only an override finishes.
    WaitingManual(Vec<Uid>),
    /// Nothing is scheduled: these RUNNING leaves cannot progress on their own (empty: a busy
    /// tree, e.g. a `KeepRunningUntilFailure` loop, with no timer).
    Stalled(Vec<Uid>),
    /// [`RunOptions::max_ticks`] was reached.
    TickLimit,
    /// The clock reached [`RunOptions::until`].
    Until,
}

/// What [`Executor::run`] did.
#[derive(Clone, Debug, PartialEq)]
pub struct RunReport {
    /// The root's status afterwards.
    pub status: NodeStatus,
    /// Ticks run.
    pub ticks: u32,
    /// Why the run stopped.
    pub stop: Stop,
}

impl Executor {
    /// Tick from event to event until something in `options` or the tree stops it.
    pub fn run(&mut self, options: RunOptions) -> RunReport {
        let (mut ticks, mut releases) = (0u32, 0u32);
        loop {
            if !self.due() {
                if !self.paused.is_empty() {
                    return self.report(ticks, Stop::Breakpoint(self.paused.clone()));
                }
                let waiting = self.waiting_on();
                if options.stop_on_manual && !waiting.is_empty() {
                    return self.report(ticks, Stop::WaitingManual(waiting));
                }
                match self.next_wakeup() {
                    Some(t) if t > options.until + EPS => {
                        self.advance_clock(options.until);
                        return self.report(ticks, Stop::Until);
                    }
                    Some(t) => {
                        if ticks >= options.max_ticks {
                            return self.report(ticks, Stop::TickLimit);
                        }
                        self.advance_clock(t);
                        releases = 0;
                    }
                    None if !waiting.is_empty() => {
                        return self.report(ticks, Stop::WaitingManual(waiting));
                    }
                    None => {
                        let blocked = self.blocked();
                        if !options.release_stalls
                            || blocked.is_empty()
                            || releases >= MAX_STALL_RELEASES
                        {
                            return self.report(ticks, Stop::Stalled(blocked));
                        }
                        if ticks >= options.max_ticks {
                            return self.report(ticks, Stop::TickLimit);
                        }
                        releases += 1;
                        self.release_stalled();
                    }
                }
            } else if ticks >= options.max_ticks {
                return self.report(ticks, Stop::TickLimit);
            }
            let status = self.tick();
            ticks += 1;
            if status != NodeStatus::Running {
                // An empty tree fails without its root ever changing: report what the tick said.
                return RunReport {
                    status,
                    ticks,
                    stop: Stop::Finished(status),
                };
            }
        }
    }

    /// Tick until the tree finishes or waits on a manual leaf, jumping the clock straight to each
    /// next timer. Returns the root status and the ticks used.
    ///
    /// It stops *before* advancing the clock once a manual leaf is RUNNING, so a `Timeout` above
    /// that leaf does not fire while the user decides what to force. A busy tree with no timer
    /// (`Stalled([])`) only moves when ticked, so it gets one tick per call.
    pub fn run_virtual(&mut self, max_ticks: u32) -> (NodeStatus, u32) {
        let report = self.run(RunOptions {
            max_ticks,
            ..RunOptions::VIRTUAL
        });
        if report.ticks == 0 && max_ticks > 0 && report.stop == Stop::Stalled(Vec::new()) {
            return (self.tick(), 1);
        }
        (report.status, report.ticks)
    }

    /// cell-runner's way out of a stall: native waits that started before the next tick give
    /// up (they see [`LeafCtx::releasing`](crate::LeafCtx::releasing)) until the clock moves.
    pub fn release_stalled(&mut self) {
        let before = self.tick + 1;
        self.release_before = Some(before);
        self.dirty = true;
        self.message(
            None,
            format!("stalled: waits started before tick {before} give up (like cell-runner)"),
        );
    }

    /// Where the tree halts, if the clock cannot move without the user: paused at a
    /// breakpoint, or nothing scheduled while manual leaves wait or leaves are blocked.
    /// A tree that still has a timer, or a busy tree with no blocked leaf, is not halted.
    pub fn halted(&self) -> Option<Stop> {
        if self.tick == 0 || self.root_status() != NodeStatus::Running || self.due() {
            return None;
        }
        if !self.paused.is_empty() {
            return Some(Stop::Breakpoint(self.paused.clone()));
        }
        if self.next_wakeup().is_some() {
            return None;
        }
        let waiting = self.waiting_on();
        if !waiting.is_empty() {
            return Some(Stop::WaitingManual(waiting));
        }
        let blocked = self.blocked();
        (!blocked.is_empty()).then_some(Stop::Stalled(blocked))
    }

    /// RUNNING leaves with no wakeup of their own that are not manual leaves and have no
    /// override queued: what a stall waits on.
    pub fn blocked(&self) -> Vec<Uid> {
        self.tree
            .nodes
            .iter()
            .filter(|n| {
                n.kind.is_leaf()
                    && self.states[n.uid].status == NodeStatus::Running
                    && self.overrides[n.uid].is_none()
                    && !self.waits_for_user(n.uid)
                    && self.wakeup_of(n.uid).is_none()
            })
            .map(|n| n.uid)
            .collect()
    }

    /// Why `uid` has not finished, for the "where it halts" list.
    pub fn waiting_reason(&self, uid: Uid) -> Option<String> {
        if self.paused.contains(&uid) {
            return Some("paused before it starts".to_string());
        }
        if let Some(o) = self.overrides[uid] {
            return Some(format!("forced {} on its next tick", o.status));
        }
        if !self.tree.nodes[uid].kind.is_leaf() || self.states[uid].status != NodeStatus::Running {
            return None;
        }
        match self.policies[uid] {
            LeafPolicy::Manual if self.waits_for_user(uid) => {
                Some("policy 'wait': RUNNING until you force it".to_string())
            }
            LeafPolicy::Native if self.handled[uid] => self
                .leaves
                .as_ref()
                .and_then(|h| h.waiting_reason(uid))
                .or_else(|| {
                    self.wakeup_of(uid)
                        .is_none()
                        .then(|| "RUNNING with nothing scheduled to finish it".to_string())
                }),
            _ => None,
        }
    }

    /// The tree must be ticked at this instant: something asked for it, it has not started
    /// (or finished), or a wakeup is due.
    fn due(&self) -> bool {
        self.dirty
            || self.retick
            || self.root_status() != NodeStatus::Running
            || self.wakeups.iter().any(|w| w.t <= self.now + EPS)
    }

    fn report(&self, ticks: u32, stop: Stop) -> RunReport {
        RunReport {
            status: self.root_status(),
            ticks,
            stop,
        }
    }
}
