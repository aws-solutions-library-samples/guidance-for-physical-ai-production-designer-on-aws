//! IR invariants checked before code generation (and exhaustively in tests).
//!
//! 1. Each robot program is a strict HB chain; op ids equal positions.
//! 2. Every zone is a strict HB chain in ticket order with consistent masks and op indices.
//! 3. Every reduced cross edge `u → v` has `u` immediately before `v` in some zone.
//! 4. No two HB-concurrent ops share a zone.
//! 5. closure(program order ∪ ticket adjacency ∪ permit edges) == HB on ops (exact equivalence).

use crate::error::{DiagCode, Error, Result};
use crate::ir::CellPlan;
use crate::ir::bitset::BitSet;

pub fn validate(p: &CellPlan) -> Result<()> {
    let fail = |m: String| {
        Err(Error::plan(
            DiagCode::E005,
            None,
            format!("internal invariant violated: {m}"),
        ))
    };
    for (i, op) in p.ops.iter().enumerate() {
        if op.id != i || op.pos != i {
            return fail(format!("op {i} has id {} pos {}", op.id, op.pos));
        }
    }
    for r in &p.robots {
        for w in r.ops.windows(2) {
            if !p.hb.before(w[0], w[1]) {
                return fail(format!("robot {} program not HB-ordered", r.ns));
            }
        }
        for (k, &o) in r.ops.iter().enumerate() {
            if p.ops[o].robot != r.index || p.ops[o].index as usize != k + 1 {
                return fail(format!("robot {} op {o} index mismatch", r.ns));
            }
        }
    }
    for z in &p.zones {
        for w in z.tickets.windows(2) {
            if !p.hb.before(w[0].op, w[1].op) {
                return fail(format!("zone {} is not an HB chain", z.name));
            }
        }
        if z.index >= 31 {
            return fail(format!("zone {} uses the permit bit", z.name));
        }
        for t in &z.tickets {
            let op = &p.ops[t.op];
            if op.zone_mask & (1 << z.index) == 0 || t.robot != op.robot || t.op_index != op.index {
                return fail(format!(
                    "zone {} ticket for op {} inconsistent",
                    z.name, t.op
                ));
            }
        }
    }
    for e in &p.cross_edges {
        let covered = p.zones.iter().any(|z| {
            z.tickets
                .windows(2)
                .any(|w| w[0].op == e.from && w[1].op == e.to)
        }) || p.ops[e.to].permit_preds.contains(&e.from);
        if !covered {
            return fail(format!(
                "edge {} -> {} not covered by a zone",
                p.op_ref(e.from),
                p.op_ref(e.to)
            ));
        }
    }
    if enforced_closure(p) != p.hb.anc {
        return fail("zone protocol order differs from the BT happens-before".into());
    }
    Ok(())
}

/// Ancestor sets of the order the zone protocol enforces: program order + ticket adjacency.
pub fn enforced_closure(p: &CellPlan) -> Vec<BitSet> {
    let n = p.ops.len();
    let mut pred: Vec<Vec<usize>> = vec![Vec::new(); n];
    for r in &p.robots {
        for w in r.ops.windows(2) {
            pred[w[1]].push(w[0]);
        }
    }
    for z in &p.zones {
        for w in z.tickets.windows(2) {
            pred[w[1].op].push(w[0].op);
        }
    }
    for op in &p.ops {
        pred[op.id].extend(op.permit_preds.iter().copied());
    }
    // Op ids are a linear extension, and every enforced edge goes forward.
    let mut anc = vec![BitSet::new(n); n];
    for v in 0..n {
        let mut acc = BitSet::new(n);
        for &u in &pred[v] {
            acc.union_with(&anc[u]);
            acc.insert(u);
        }
        anc[v] = acc;
    }
    anc
}
