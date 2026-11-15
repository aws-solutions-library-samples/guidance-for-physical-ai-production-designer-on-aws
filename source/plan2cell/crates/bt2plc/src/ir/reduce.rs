//! Cross-robot edge reduction (DESIGN §3.4).
//!
//! For op `v` of robot R and another robot Q, `pred_Q(v)` is the latest Q op that happens
//! before `v`. The edge `pred_Q(v) → v` is kept only when it differs from `pred_Q(prev_R(v))`
//! (otherwise program order plus the earlier edge already implies it). With three or more
//! robots, an edge implied through a third robot's predecessor is pruned.
//! Lemma R: closure(program order ∪ edges) == HB on ops.

use crate::ir::hb::Hb;
use crate::ir::lower::EventGraph;
use crate::ir::{Edge, OpId, Robot};

pub fn reduce(hb: &Hb, _g: &EventGraph, robots: &[Robot], prune_transitive: bool) -> Vec<Edge> {
    let mut edges = Vec::new();
    // preds[v][q] = pred_Q(v)
    let n = hb.n_ops();
    let mut preds: Vec<Vec<Option<OpId>>> = vec![vec![None; robots.len()]; n];
    for r in robots {
        for q in robots {
            if r.index == q.index {
                continue;
            }
            let chain_q = &q.ops;
            let mut p = 0usize;
            let mut prev: Option<OpId> = None;
            for &v in &r.ops {
                while p < chain_q.len() && hb.before(chain_q[p], v) {
                    p += 1;
                }
                let cur = (p > 0).then(|| chain_q[p - 1]);
                preds[v][q.index] = cur;
                if let Some(u) = cur
                    && Some(u) != prev
                {
                    edges.push(Edge { from: u, to: v });
                }
                prev = cur;
            }
        }
    }
    if prune_transitive && robots.len() >= 3 {
        let robot_of = |o: OpId| {
            robots
                .iter()
                .find(|r| r.ops.contains(&o))
                .map(|r| r.index)
                .unwrap()
        };
        edges.retain(|e| {
            let (ru, rv) = (robot_of(e.from), robot_of(e.to));
            !robots.iter().any(|s| {
                s.index != ru
                    && s.index != rv
                    && preds[e.to][s.index].is_some_and(|sp| hb.before(e.from, sp))
            })
        });
    }
    edges.sort_by_key(|e| (e.to, e.from));
    edges.dedup();
    edges
}
