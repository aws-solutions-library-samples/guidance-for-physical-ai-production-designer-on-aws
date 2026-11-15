//! `cell_plan.json` (schema `bt2plc/cell-plan@1`): the IR dump.

use serde::Serialize;

use crate::config::Config;
use crate::error::{Diag, Diagnostics};
use crate::ir::{CellPlan, OpKind};

#[derive(Serialize)]
struct Doc<'a> {
    schema: &'static str,
    generator: &'static str,
    source: Source<'a>,
    config: &'a Config,
    robots: Vec<RobotOut<'a>>,
    ops: Vec<OpOut<'a>>,
    cross_edges: Vec<EdgeOut>,
    zones: Vec<ZoneOut>,
    schedule: Sched,
    diagnostics: &'a [Diag],
}

#[derive(Serialize)]
struct Source<'a> {
    file: &'a str,
    tree_id: &'a str,
    cell_id: &'a str,
    fingerprint: String,
    plan_id: String,
}

#[derive(Serialize)]
struct RobotOut<'a> {
    index: usize,
    ns: &'a str,
    tag: &'a str,
    station: &'a str,
    device_name: &'a str,
    ip: String,
    idle: bool,
    prod_folge: u16,
    home_pose_deg: Vec<f64>,
    home_source: &'a crate::ir::HomeSource,
    ops: Vec<String>,
}

#[derive(Serialize)]
struct OpOut<'a> {
    pos: usize,
    robot: &'a str,
    index: u16,
    operation_id: &'a str,
    kind: &'a OpKind,
    bt_path: &'a str,
    bt_name: Option<&'a str>,
    zones: Vec<String>,
    zone_mask: String,
    permit_preds: Vec<String>,
    start_delay_s: f64,
    delay_exact: bool,
    hold_s: f64,
    velocity_scale: f64,
    motion_s: f64,
    expected_s: f64,
    limit_s: f64,
    zone_deadline_s: f64,
    asap_start_s: f64,
    asap_end_s: f64,
    depth: u32,
    waypoints: usize,
}

#[derive(Serialize)]
struct EdgeOut {
    from: String,
    to: String,
    via: String,
}

#[derive(Serialize)]
struct ZoneOut {
    name: String,
    bit: usize,
    tickets: Vec<String>,
}

#[derive(Serialize)]
struct Sched {
    makespan_s: f64,
    critical_path: Vec<String>,
}

fn sec(ms: u64) -> f64 {
    ms as f64 / 1000.0
}

pub fn render(plan: &CellPlan, cfg: &Config, d: &Diagnostics) -> String {
    let opref = |o: usize| plan.op_ref(o);
    let robots = plan
        .robots
        .iter()
        .map(|r| RobotOut {
            index: r.index + 1,
            ns: &r.ns,
            tag: &r.addr.tag,
            station: &r.addr.station,
            device_name: &r.addr.device_name,
            ip: r.addr.ip.to_string(),
            idle: r.idle,
            prod_folge: r.addr.prod_folge,
            home_pose_deg: r
                .home_pose_rad
                .iter()
                .map(|q| round4(q.to_degrees()))
                .collect(),
            home_source: &r.home_source,
            ops: r
                .ops
                .iter()
                .map(|&o| plan.ops[o].operation_id.clone())
                .collect(),
        })
        .collect();
    let ops = plan
        .ops
        .iter()
        .map(|o| OpOut {
            pos: o.pos,
            robot: &plan.robots[o.robot].ns,
            index: o.index,
            operation_id: &o.operation_id,
            kind: &o.kind,
            bt_path: &o.bt.path,
            bt_name: o.bt.name.as_deref(),
            zones: o
                .zones
                .iter()
                .map(|&z| plan.zones[z].name.clone())
                .collect(),
            zone_mask: format!("16#{:08X}", o.zone_mask),
            permit_preds: o.permit_preds.iter().map(|&p| opref(p)).collect(),
            start_delay_s: sec(o.start_delay_ms),
            delay_exact: o.delay_exact,
            hold_s: sec(o.hold_ms),
            velocity_scale: o.velocity_scale,
            motion_s: sec(o.motion_ms),
            expected_s: sec(o.expected_ms),
            limit_s: sec(o.limit_ms),
            zone_deadline_s: sec(o.zone_deadline_ms),
            asap_start_s: sec(o.asap_start_ms),
            asap_end_s: sec(o.asap_end_ms),
            depth: o.depth,
            waypoints: o.trajectory.as_ref().map_or(0, |t| t.waypoints.len()),
        })
        .collect();
    let cross_edges = plan
        .cross_edges
        .iter()
        .map(|e| {
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
            EdgeOut {
                from: opref(e.from),
                to: opref(e.to),
                via,
            }
        })
        .collect();
    let zones = plan
        .zones
        .iter()
        .map(|z| ZoneOut {
            name: z.name.clone(),
            bit: z.index,
            tickets: z.tickets.iter().map(|t| opref(t.op)).collect(),
        })
        .collect();
    let doc = Doc {
        schema: "bt2plc/cell-plan@1",
        generator: crate::backend::codesys::GENERATOR,
        source: Source {
            file: &plan.source_file,
            tree_id: &plan.tree_id,
            cell_id: &plan.cell_id,
            fingerprint: format!("fnv1a64:{:016x}", plan.fingerprint),
            plan_id: format!("16#{:04X}", plan.plan_id),
        },
        config: cfg,
        robots,
        ops,
        cross_edges,
        zones,
        schedule: Sched {
            makespan_s: sec(plan.makespan_ms),
            critical_path: critical_path(plan),
        },
        diagnostics: d.items(),
    };
    serde_json::to_string_pretty(&doc).expect("cell plan serialises") + "\n"
}

fn round4(x: f64) -> f64 {
    let r = (x * 10000.0).round() / 10000.0;
    if r == 0.0 { 0.0 } else { r }
}

/// Ops on a longest chain (by ASAP end) ending at the last finishing op.
pub fn critical_path(plan: &CellPlan) -> Vec<String> {
    let Some(mut cur) = plan
        .ops
        .iter()
        .max_by_key(|o| (o.asap_end_ms, o.pos))
        .map(|o| o.id)
    else {
        return Vec::new();
    };
    let mut path = vec![cur];
    loop {
        let start = plan.ops[cur].asap_start_ms;
        let pred = plan.hb.anc[cur]
            .iter()
            .filter(|&a| plan.ops[a].asap_end_ms == start)
            .max_by_key(|&a| plan.ops[a].pos);
        match pred {
            Some(p) if start > 0 => {
                path.push(p);
                cur = p;
            }
            _ => break,
        }
    }
    path.reverse();
    path.into_iter().map(|o| plan.op_ref(o)).collect()
}
