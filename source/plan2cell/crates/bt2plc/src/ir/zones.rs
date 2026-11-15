//! Zones: greedy chain cover of the reduced cross edges (DESIGN §3.5).
//!
//! A zone is a list of ops that are pairwise HB-ordered, kept in HB order (its ticket order).
//! Each reduced edge `u → v` ends up with `u` immediately before `v` in some zone, so granting
//! zones in ticket order enforces the BT ordering exactly; because members are already
//! HB-ordered, a zone never serialises ops the BT allows to run concurrently.
//!
//! Capacity: edges that would need a zone beyond `max_zones` (e.g. a leader/follower
//! coordinated tree with many gates, whose edges are pairwise incomparable) are enforced
//! by the permit bit instead: the PLC grants it when the listed predecessor ops are done.

use crate::error::Result;
use crate::ir::hb::Hb;
use crate::ir::lower::EventGraph;
use crate::ir::{Edge, OpId};

struct Draft {
    members: Vec<OpId>,
    created: usize,
}

/// Zones (member lists in ticket order, numbered by the position of their first member) and
/// the edges that did not fit into `max_zones` zones; those are enforced by the permit bit.
pub struct Cover {
    pub zones: Vec<Vec<OpId>>,
    pub permit_edges: Vec<Edge>,
}

pub fn cover(hb: &Hb, edges: &[Edge], g: &EventGraph, max_zones: usize) -> Result<Cover> {
    let robot = |o: OpId| g.ops[o].robot_ns.as_str();
    let mut zones: Vec<Draft> = Vec::new();
    let mut permit_edges = Vec::new();
    let mut sorted = edges.to_vec();
    sorted.sort_by_key(|e| (e.to, e.from));
    for e in &sorted {
        let (u, v) = (e.from, e.to);
        let exact: Vec<usize> = (0..zones.len())
            .filter(|&z| *zones[z].members.last().unwrap() == u)
            .collect();
        let mut cand = if exact.is_empty() {
            (0..zones.len())
                .filter(|&z| hb.before(*zones[z].members.last().unwrap(), u))
                .collect()
        } else {
            exact
        };
        let pair = [robot(u), robot(v)];
        cand.sort_by_key(|&z| {
            let same_pair = zones[z].members.iter().all(|&m| pair.contains(&robot(m)));
            (!same_pair, zones[z].created)
        });
        match cand.first() {
            Some(&z) => {
                if *zones[z].members.last().unwrap() != u {
                    zones[z].members.push(u);
                }
                zones[z].members.push(v);
            }
            None if zones.len() < max_zones => {
                let created = zones.len();
                zones.push(Draft {
                    members: vec![u, v],
                    created,
                });
            }
            None => permit_edges.push(*e),
        }
    }
    zones.sort_by_key(|z| z.members[0]);
    Ok(Cover {
        zones: zones.into_iter().map(|z| z.members).collect(),
        permit_edges,
    })
}
