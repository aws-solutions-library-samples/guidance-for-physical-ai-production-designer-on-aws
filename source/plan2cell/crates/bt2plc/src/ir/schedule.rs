//! ASAP schedule, start delays, watchdogs and zone deadlines (DESIGN §3.8), in integer ms.
//!
//! Per op x (all measured on the robot from the grant):
//! - `expected = start_delay + motion + hold + completion_margin`, motion = trajectory end /
//!   velocity_scale (blind MoveHome: `home_return_time_s`)
//! - `limit = min(f·expected + m, Timeout)`
//! - `zone_deadline = f·grant(x) + (m + hop)·depth(x) + m_zone` since the cycle start, where
//!   grant(x) is the PLC-side ASAP grant (start delays are waited after the grant).

use crate::config::Config;
use crate::error::DiagCode;
use crate::ir::CellPlan;
use crate::ir::lower::{EventGraph, EventKind};

fn ms(s: f64) -> u64 {
    (s * 1000.0).round().max(0.0) as u64
}

pub fn schedule(plan: &mut CellPlan, g: &EventGraph, cfg: &Config) {
    let t = &cfg.timing;
    let margin = ms(t.completion_margin_s);
    for op in &mut plan.ops {
        op.motion_ms = match &op.trajectory {
            Some(tr) => ms(tr.duration_s() / op.velocity_scale),
            None => ms(t.home_return_time_s),
        };
    }
    let n = g.ev.len();
    let mut dmax = vec![0u64; n];
    let mut dmin = vec![0u64; n];
    let mut end = vec![0u64; n];
    // PLC-side schedule: Delay events add nothing (the robot waits its start delay after the
    // grant), an op is granted when all its predecessors are done.
    let mut end_plc = vec![0u64; n];
    let mut depth = vec![0u32; n];
    for &e in &g.order {
        let mut ready = 0u64;
        let mut grant = 0u64;
        let (mut cmax, mut cmin, mut any) = (0u64, u64::MAX, false);
        let mut dep = 0u32;
        for &p in &g.pred[e] {
            ready = ready.max(end[p]);
            grant = grant.max(end_plc[p]);
            // A direct op predecessor is a zero-delay path (it must count for dmin).
            let is_op = g.op_of(p).is_some();
            let c = if is_op { 0 } else { dmax[p] };
            let cm = if is_op { 0 } else { dmin[p] };
            cmax = cmax.max(c);
            cmin = cmin.min(cm);
            any = true;
            dep = dep.max(depth[p] + u32::from(is_op));
        }
        if !any {
            cmin = 0;
        }
        depth[e] = dep;
        match &g.ev[e].kind {
            EventKind::Delay { ms: d } => {
                dmax[e] = cmax + d;
                dmin[e] = cmin + d;
                end[e] = ready + d;
                end_plc[e] = grant;
            }
            EventKind::Op(o) => {
                let op = &mut plan.ops[*o];
                op.start_delay_ms = cmax;
                op.delay_exact = cmax == cmin;
                op.expected_ms = cmax + op.motion_ms + op.hold_ms + margin;
                // Grant time and end as the PLC/robot protocol executes them (equal to the BT
                // schedule whenever the start delay is exact).
                op.asap_start_ms = grant;
                op.asap_end_ms = grant + op.expected_ms;
                op.depth = dep;
                end[e] = ready + op.motion_ms + op.hold_ms + margin;
                end_plc[e] = op.asap_end_ms;
            }
            EventKind::Gate { .. } | EventKind::Barrier => {
                dmax[e] = cmax;
                dmin[e] = cmin;
                end[e] = ready;
                end_plc[e] = grant;
            }
        }
    }
    let f = t.watchdog_factor;
    let m = ms(t.watchdog_margin_s);
    let mz = ms(t.zone_wait_margin_s);
    let hop = ms(t.hop_latency_s);
    for op in &mut plan.ops {
        let computed = ms(f * op.expected_ms as f64 / 1000.0) + m;
        // A BT Timeout bounds the op from its tick, i.e. after delays outside it: allow the start delay on top.
        op.limit_ms = op
            .timeout_override_ms
            .map_or(computed, |to| computed.min(op.start_delay_ms + to));
        op.zone_deadline_ms =
            ms(f * op.asap_start_ms as f64 / 1000.0) + (m + hop) * u64::from(op.depth) + mz;
    }
    plan.makespan_ms = plan.ops.iter().map(|o| o.asap_end_ms).max().unwrap_or(0);
    let _ = DiagCode::W007;
}

/// A BT Timeout shorter than the op's own motion + hold can never be met (E021); every TIME
/// value must fit the PLC's 32-bit millisecond TIME (E023).
pub fn check_timeouts(plan: &CellPlan) -> crate::error::Result<()> {
    for op in &plan.ops {
        for (what, v) in [
            ("deadline", op.zone_deadline_ms),
            ("limit", op.limit_ms),
            ("end", op.asap_end_ms),
        ] {
            if v > u64::from(u32::MAX) {
                return Err(crate::error::Error::plan(
                    DiagCode::E023,
                    Some(&op.bt.path),
                    format!(
                        "{}.{}: {what} {v} ms exceeds the PLC TIME range",
                        plan.robots[op.robot].ns, op.operation_id
                    ),
                ));
            }
        }
        if let Some(to) = op.timeout_override_ms
            && to < op.expected_ms - op.start_delay_ms
        {
            return Err(crate::error::Error::plan(
                DiagCode::E021,
                Some(&op.bt.path),
                format!(
                    "{}.{}: Timeout {to} ms is shorter than the op itself ({} ms); the watchdog would always fire",
                    plan.robots[op.robot].ns,
                    op.operation_id,
                    op.expected_ms - op.start_delay_ms
                ),
            ));
        }
    }
    Ok(())
}

/// Warn about inexact start delays (W007); called by the pipeline after scheduling.
pub fn delay_warnings(plan: &CellPlan, d: &mut crate::error::Diagnostics) {
    for op in plan.ops.iter().filter(|o| !o.delay_exact) {
        d.warn(
            DiagCode::W007,
            Some(&op.bt.path),
            format!(
                "{}.{}: Delay paths of different length lead into this op; the robot waits the longest ({} ms) after its grants",
                plan.robots[op.robot].ns, op.operation_id, op.start_delay_ms
            ),
        );
    }
}
