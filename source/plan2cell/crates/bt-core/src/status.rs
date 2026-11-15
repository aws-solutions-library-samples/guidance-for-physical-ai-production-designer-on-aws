//! Node statuses and the record of how they changed.

use serde::{Deserialize, Serialize};

use crate::model::Uid;

/// BehaviorTree.CPP's `NodeStatus` (v4 adds `Skipped`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NodeStatus {
    #[default]
    Idle,
    Running,
    Success,
    Failure,
    Skipped,
}

impl NodeStatus {
    pub fn is_completed(self) -> bool {
        matches!(self, Self::Success | Self::Failure)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::Running => "RUNNING",
            Self::Success => "SUCCESS",
            Self::Failure => "FAILURE",
            Self::Skipped => "SKIPPED",
        }
    }
}

impl std::fmt::Display for NodeStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One status change, as BT.CPP's `StatusChangeLogger` reports it to Groot.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transition {
    /// The tick (1-based) during which the change happened.
    pub tick: u64,
    /// Executor time in seconds.
    pub t: f64,
    pub uid: Uid,
    pub prev: NodeStatus,
    pub status: NodeStatus,
}

/// Every transition of one run, in order, with per-node lookup for scrubbing.
#[derive(Clone, Debug, Default)]
pub struct Trace {
    transitions: Vec<Transition>,
    /// Indices into `transitions`, per node uid.
    by_node: Vec<Vec<usize>>,
}

/// What a node looks like at some point of a trace.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NodeView {
    pub status: NodeStatus,
    /// For an IDLE node: what it was before being reset in the same tick, which Groot draws muted.
    pub was: Option<NodeStatus>,
}

impl Trace {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            transitions: Vec::new(),
            by_node: vec![Vec::new(); num_nodes],
        }
    }

    pub fn push(&mut self, transition: Transition) {
        if transition.uid >= self.by_node.len() {
            self.by_node.resize(transition.uid + 1, Vec::new());
        }
        self.by_node[transition.uid].push(self.transitions.len());
        self.transitions.push(transition);
    }

    pub fn transitions(&self) -> &[Transition] {
        &self.transitions
    }

    pub fn last_tick(&self) -> u64 {
        self.transitions.last().map_or(0, |t| t.tick)
    }

    /// Transitions of one node, oldest first.
    pub fn of_node(&self, uid: Uid) -> impl Iterator<Item = &Transition> {
        self.by_node
            .get(uid)
            .into_iter()
            .flatten()
            .map(|&i| &self.transitions[i])
    }

    /// The node at the end of tick `tick` (0 = before the first tick).
    ///
    /// Like Groot's monitor, a node that finished (or was halted) and was then reset to IDLE
    /// keeps what it was in [`NodeView::was`], drawn muted, until the root (uid 0) starts over.
    pub fn view_at(&self, uid: Uid, tick: u64) -> NodeView {
        let Some(last) = self.last_index_at(uid, tick) else {
            return NodeView::default();
        };
        let t = self.transitions[last];
        let mut view = NodeView {
            status: t.status,
            was: None,
        };
        if t.status == NodeStatus::Idle && t.prev != NodeStatus::Idle {
            let run_started = self.by_node.first().and_then(|root| {
                root.iter()
                    .copied()
                    .take_while(|&i| self.transitions[i].tick <= tick)
                    .filter(|&i| {
                        let r = self.transitions[i];
                        r.prev == NodeStatus::Idle && r.status == NodeStatus::Running
                    })
                    .last()
            });
            if run_started.is_none_or(|start| last > start) {
                view.was = Some(t.prev);
            }
        }
        view
    }

    /// Index into `transitions` of the last change of `uid` up to the end of `tick`.
    fn last_index_at(&self, uid: Uid, tick: u64) -> Option<usize> {
        let indices = self.by_node.get(uid)?;
        let end = indices.partition_point(|&i| self.transitions[i].tick <= tick);
        end.checked_sub(1).map(|e| indices[e])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tr(tick: u64, uid: Uid, prev: NodeStatus, status: NodeStatus) -> Transition {
        Transition {
            tick,
            t: tick as f64,
            uid,
            prev,
            status,
        }
    }

    #[test]
    fn view_at_scrubs_and_remembers_resets() {
        use NodeStatus::*;
        let mut trace = Trace::new(2);
        trace.push(tr(1, 1, Idle, Running));
        trace.push(tr(3, 1, Running, Success));
        trace.push(tr(3, 1, Success, Idle));
        assert_eq!(trace.view_at(1, 0).status, Idle);
        assert_eq!(trace.view_at(1, 2).status, Running);
        assert_eq!(
            trace.view_at(1, 3),
            NodeView {
                status: Idle,
                was: Some(Success)
            }
        );
        // Still muted later, until the root starts a new run.
        assert_eq!(trace.view_at(1, 4).was, Some(Success));
        trace.push(tr(5, 0, Idle, Running));
        assert_eq!(trace.view_at(1, 5).was, None);
        assert_eq!(trace.view_at(1, 4).was, Some(Success));
    }
}
