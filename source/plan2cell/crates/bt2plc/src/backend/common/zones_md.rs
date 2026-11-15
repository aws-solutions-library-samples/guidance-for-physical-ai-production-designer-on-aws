//! `ZONES.md`: the interlock zones, the robot programs and how every BT dependency is enforced.

use std::fmt::Write as _;

use crate::config::GrantPolicy;
use crate::error::Diagnostics;
use crate::ir::CellPlan;

pub fn render(plan: &CellPlan, d: &Diagnostics) -> String {
    let mut o = String::new();
    let _ = writeln!(o, "# Interlock zones - {}\n", plan.cell_id);
    let _ = writeln!(
        o,
        "Plan `{}` (from `{}`), plan id 16#{:04X}. {} robot(s), {} op(s), {} cross-robot dependencies, {} zone(s){}.\n",
        plan.tree_id,
        plan.source_file,
        plan.plan_id,
        plan.robots.len(),
        plan.ops.len(),
        plan.cross_edges.len(),
        plan.zones.len(),
        if plan.permit_edges.is_empty() {
            String::new()
        } else {
            format!(", {} via the permit bit", plan.permit_edges.len())
        }
    );
    o.push_str("Every robot runs its own op list in one production Folge. Before an op that depends on another robot it raises the REQUEST bits of the op's zones, waits for the head PLC to GRANT all of them, runs the op, drops REQUEST and waits for GRANT to drop (4-phase handshake). ");
    match plan.grant_policy {
        GrantPolicy::Planned => o.push_str("The PLC grants each zone to one robot at a time **in ticket order**, which reproduces the behaviour tree's ordering exactly while letting BT-parallel ops run together.\n\n"),
        GrantPolicy::Fcfs => o.push_str("**FCFS mode**: the PLC grants a free zone to whichever robot asks first. This keeps mutual exclusion but NOT the behaviour tree's order.\n\n"),
    }
    o.push_str("## Zones (ticket order)\n\n");
    if plan.zones.is_empty() {
        o.push_str("_none_\n");
    }
    for z in &plan.zones {
        let t: Vec<String> = z
            .tickets
            .iter()
            .map(|t| {
                format!(
                    "{}.{}",
                    plan.robots[t.robot].ns, plan.ops[t.op].operation_id
                )
            })
            .collect();
        let _ = writeln!(o, "- **{}** (bit {}): {}", z.name, z.index, t.join(" > "));
    }
    o.push_str("\n## Robot programs\n\n");
    for r in &plan.robots {
        let items: Vec<String> = r
            .ops
            .iter()
            .map(|&oi| {
                let op = &plan.ops[oi];
                let mut tags: Vec<String> = op
                    .zones
                    .iter()
                    .map(|&z| plan.zones[z].name.clone())
                    .collect();
                if !op.permit_preds.is_empty() {
                    tags.push("PERMIT".into());
                }
                if tags.is_empty() {
                    op.operation_id.clone()
                } else {
                    format!("{}[{}]", op.operation_id, tags.join(","))
                }
            })
            .collect();
        let _ = writeln!(
            o,
            "- **{}** ({}, Folge {}): {}",
            r.ns,
            r.addr.tag,
            r.addr.prod_folge,
            if items.is_empty() {
                "_idle_".to_string()
            } else {
                items.join(" ")
            }
        );
    }
    o.push_str("\n## Dependencies\n\n| Before | After | Enforced by |\n|---|---|---|\n");
    for e in &plan.cross_edges {
        let via = plan
            .zones
            .iter()
            .find(|z| {
                z.tickets
                    .windows(2)
                    .any(|w| w[0].op == e.from && w[1].op == e.to)
            })
            .map(|z| z.name.clone())
            .unwrap_or_else(|| "PERMIT".into());
        let _ = writeln!(
            o,
            "| {} | {} | {via} |",
            plan.op_ref(e.from),
            plan.op_ref(e.to)
        );
    }
    o.push_str("\n## Schedule (nominal durations)\n\n| Robot | # | Op | Zones | Grant (s) | End (s) | Expected (s) | Watchdog (s) |\n|---|---|---|---|---|---|---|---|\n");
    for r in &plan.robots {
        for &oi in &r.ops {
            let op = &plan.ops[oi];
            let z: Vec<String> = op
                .zones
                .iter()
                .map(|&z| plan.zones[z].name.clone())
                .collect();
            let _ = writeln!(
                o,
                "| {} | {} | {} | {} | {:.3} | {:.3} | {:.3} | {:.3} |",
                r.ns,
                op.index,
                op.operation_id,
                if z.is_empty() {
                    "-".into()
                } else {
                    z.join(",")
                },
                op.asap_start_ms as f64 / 1000.0,
                op.asap_end_ms as f64 / 1000.0,
                op.expected_ms as f64 / 1000.0,
                op.limit_ms as f64 / 1000.0
            );
        }
    }
    let _ = writeln!(
        o,
        "\nMakespan: **{:.3} s**. Critical path: {}.",
        plan.makespan_ms as f64 / 1000.0,
        super::cell_plan_json::critical_path(plan).join(" > ")
    );
    if !d.is_empty() {
        o.push_str("\n## Diagnostics\n\n");
        for x in d.items() {
            let _ = writeln!(o, "- {x}");
        }
    }
    o
}
