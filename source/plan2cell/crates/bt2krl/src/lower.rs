//! program.json + bt2krl.toml -> IR (production Folge, HOME Folge), with the checks that make a
//! program unsafe or unexecutable (limits, `$H_POS`, head watchdog) and the time estimates.

use serde::Serialize;

use crate::config::{Config, EAxes, Profile};
use crate::error::{Error, Result};
use crate::ir::{Folge, FolgeKind, Instr, OpBlock, Point, ms, r4};
use crate::model::{self, RobotModel};
use crate::motion;
use crate::program::Program;
use crate::vw::{self, Interlocks};

/// Minimum time a step value stays visible to the head PLC.
pub const MIN_STEP_MS: u64 = 50;
/// Warn when the estimated op time exceeds this factor of the planned `expected_s`.
pub const WARN_FACTOR: f64 = 1.2;

/// Per-robot settings resolved from bt2krl.toml and program.json.
#[derive(Clone, Debug)]
pub struct RobotSetup {
    pub ns: String,
    pub controller: String,
    pub model: &'static RobotModel,
    pub e_axes: EAxes,
    pub e_values: Option<[f64; 6]>,
    pub h_pos_deg: Option<[f64; 6]>,
    pub h_pos_tol_deg: f64,
    pub tool_no: u16,
    pub base_no: u16,
}

impl RobotSetup {
    /// `strict_e_axes = false` (explain only) treats an undeclared `e_axes` as "none".
    pub fn resolve(p: &Program, cfg: &Config, strict_e_axes: bool) -> Result<RobotSetup> {
        let ns = &p.robot.ns;
        let rc = cfg
            .robot(ns)
            .cloned()
            .unwrap_or_else(|| crate::config::RobotCfg {
                ns: ns.clone(),
                ..Default::default()
            });
        let e_axes = match rc.e_axes {
            Some(e) => e,
            None if !strict_e_axes => EAxes::None,
            None => {
                return Err(Error::Config(format!(
                    "robot {ns}: declare e_axes in a [[robot]] table of bt2krl.toml: \"none\" if \
                     the robot has no external axes, \"fixed\" with e_values otherwise \
                     (bt2krl does not guess; external axes are never commanded by the plan)"
                )));
            }
        };
        let name = match (&rc.model, p.robot.model.as_str()) {
            (Some(c), "unknown") => c.clone(),
            (Some(c), pm) => {
                let (a, b) = (model::lookup(c), model::lookup(pm));
                if a.is_some() && b.is_some() && a != b {
                    return Err(Error::Config(format!(
                        "robot {ns}: model {c} in bt2krl.toml differs from {pm} in program.json"
                    )));
                }
                c.clone()
            }
            (None, pm) => pm.to_string(),
        };
        let model = model::lookup(&name).ok_or_else(|| {
            Error::Config(format!(
                "robot {ns}: unknown model {name} (known: {}, unknown)",
                model::MODELS
                    .iter()
                    .map(|m| m.name)
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
        })?;
        Ok(RobotSetup {
            ns: ns.clone(),
            controller: rc.controller.clone().unwrap_or_else(|| ns.clone()),
            model,
            e_axes,
            e_values: rc.e_values,
            h_pos_deg: rc.h_pos_deg,
            h_pos_tol_deg: rc.h_pos_tol_deg,
            tool_no: rc.tool_no,
            base_no: rc.base_no,
        })
    }
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct OpEstimate {
    pub index: u16,
    pub operation_id: String,
    pub zone_mask: String,
    pub points: usize,
    pub waypoints: usize,
    /// Start delay + motion + hold, as the robot will execute it.
    pub est_s: f64,
    /// Motion only (segment times plus one acceleration ramp), to compare with `duration_s`.
    pub est_motion_s: f64,
    pub planned_motion_s: f64,
    pub expected_s: f64,
    pub limit_s: f64,
}

#[derive(Clone, Debug)]
pub struct Lowered {
    pub prod: Folge,
    /// `None` in profile `vw-ks24`: VW `cell.src` refuses to start a Folge unless the robot is
    /// already at home, so a blind home Folge cannot work there (TASK 7.3, Home).
    pub home: Option<Folge>,
    pub estimates: Vec<OpEstimate>,
    pub warnings: Vec<String>,
    /// Zone -> VW interlock index (profile `vw-ks24`).
    pub interlocks: Option<Interlocks>,
}

fn point(name: String, q: &[f64; 6], s: &RobotSetup) -> Point {
    Point {
        name,
        q: q.map(r4),
        e: match s.e_axes {
            EAxes::Fixed => s.e_values.map(|e| e.map(r4)),
            _ => None,
        },
    }
}

pub fn lower(p: &Program, s: &RobotSetup, cfg: &Config, profile: Profile) -> Result<Lowered> {
    let ns = &s.ns;
    let input = |m: String| Error::Input(format!("robot {ns}: {m}"));
    if profile == Profile::Sim && p.protocol.profile != "sim" {
        return Err(input(format!(
            "program.json protocol.profile is {}, not sim",
            p.protocol.profile
        )));
    }
    let vwp = profile == Profile::VwKs24;
    let interlocks = if vwp {
        if p.ops.iter().any(|o| o.mask() & (1 << 31) != 0) {
            return Err(vw::permit_error(p));
        }
        Some(Interlocks::resolve(p, cfg)?)
    } else {
        None
    };
    let mut warnings = Vec::new();
    let home = p.home_pose_deg.map(r4);
    let m = s.model;
    if m.known() {
        if let Some((a, v, lo, hi)) = m.violation(&home) {
            return Err(input(format!(
                "home_pose_deg A{a} = {v} is outside the {} software limits {lo}..{hi}",
                m.name
            )));
        }
        for op in &p.ops {
            for (i, w) in op.waypoints.iter().enumerate() {
                if let Some((a, v, lo, hi)) = m.violation(&w.q_deg) {
                    return Err(input(format!(
                        "op {} {} waypoint {i}: A{a} = {v} is outside the {} software limits \
                         {lo}..{hi} (check the URDF->KUKA joint mapping)",
                        op.index, op.operation_id, m.name
                    )));
                }
            }
        }
    } else {
        warnings.push(format!(
            "robot {ns}: model unknown: software limits are not checked and the time estimate \
             assumes the fastest axis speeds of the model table"
        ));
    }
    if let Some(h) = s.h_pos_deg {
        let d = motion::max_abs_diff(&h, &home);
        if d > s.h_pos_tol_deg {
            return Err(input(format!(
                "home_pose_deg differs from the controller $H_POS (h_pos_deg) by {d:.3} deg > \
                 h_pos_tol_deg {}",
                s.h_pos_tol_deg
            )));
        }
    }

    let mc = &cfg.motion;
    let vmax = &m.vmax;
    let mut points: Vec<Point> = Vec::new();
    if vwp {
        // P0 = home, as in every VW sample Folge (AUE_WAIT sets $H_POS here).
        points.push(point("P0".into(), &home, s));
    }
    let mut home_point: Option<usize> = None;
    let mut prev_end = home;
    let mut ops = Vec::new();
    let mut estimates = Vec::new();
    let mut held = 0u32;
    let all_mask = p.ops.iter().fold(0u32, |m, o| m | o.mask());
    let n_ops = p.ops.len();
    for (oi, op) in p.ops.iter().enumerate() {
        let is_last = oi + 1 == n_ops;
        let k = op.index;
        let mask = op.mask();
        let mut ins = Vec::new();
        if vwp {
            ins.push(Instr::SetStep(k));
            if mask != 0 {
                ins.push(Instr::VwLock(mask));
                held |= mask;
                if cfg.vw_ks24.online_stop {
                    ins.push(Instr::VwOnlineStop(held));
                }
            }
        } else {
            ins.push(Instr::AbortGuard);
            ins.push(Instr::SetStep(k));
            if mask != 0 {
                ins.push(Instr::Request(mask));
                ins.push(Instr::WaitGrant(mask));
                ins.push(Instr::AbortGuard);
            }
        }
        let delay = ms(op.start_delay_s);
        if delay > 0 {
            ins.push(Instr::Delay(delay));
        }
        let mut motion_s = 0.0;
        let n_points_before = points.len();
        let scale = op.velocity_scale;
        if op.blind_home && vwp {
            return Err(input(format!(
                "op {k} {} is a blind MoveHome: profile vw-ks24 has no blind home move (VW \
                 cell.src starts a Folge only when the robot is already at home). Re-plan with a \
                 planned path home, or use profile sim",
                op.operation_id
            )));
        }
        if op.blind_home {
            let hp = *home_point.get_or_insert_with(|| {
                points.push(point("PHOME".into(), &home, s));
                points.len() - 1
            });
            let dq = motion::delta(&prev_end, &home);
            let t = p.protocol.home_return_time_s / scale;
            let v = motion::vel_pct(&dq, t, vmax, mc);
            motion_s += motion::seg_time(&dq, v, vmax);
            ins.push(Instr::Move {
                point: hp,
                vel_pct: v,
                approx: false,
            });
            prev_end = home;
        } else {
            let w = &op.waypoints;
            let w0 = w[0].q_deg.map(r4);
            let dev = motion::max_abs_diff(&w0, &prev_end);
            if dev > p.protocol.start_tol_deg {
                warnings.push(format!(
                    "robot {ns} op {k} {}: first waypoint is {dev:.3} deg from the previous end \
                     pose (start deviation, the head robot sim warns 16#0201)",
                    op.operation_id
                ));
            }
            let keep = motion::decimate(w, mc.decimate_tol_deg);
            // The first waypoint is where the previous op ended: only op 1 moves there
            // (block coincidence and start-deviation reference) or a waypoint that differs.
            let mut from = (prev_end, 0.0);
            let last = *keep.last().expect("non-empty trajectory");
            for &i in &keep {
                let q = w[i].q_deg.map(r4);
                if i == 0 && (k > 1 || vwp) && dev <= 1e-4 {
                    from = (q, w[0].t_s);
                    continue;
                }
                let dq = motion::delta(&from.0, &q);
                let dt = (w[i].t_s - from.1) / scale;
                let v = motion::vel_pct(&dq, dt, vmax, mc);
                motion_s += motion::seg_time(&dq, v, vmax);
                let pn = if vwp { points.len() } else { points.len() + 1 };
                points.push(point(format!("P{pn}"), &q, s));
                ins.push(Instr::Move {
                    point: points.len() - 1,
                    vel_pct: v,
                    approx: i != last,
                });
                from = (q, w[i].t_s);
            }
            prev_end = w[last].q_deg.map(r4);
        }
        if motion_s > 0.0 {
            motion_s += mc.ramp_time_s;
        }
        let hold = ms(op.hold_s);
        if hold > 0 {
            ins.push(Instr::Hold(hold));
        }
        if vwp
            && !cfg.vw_ks24.weld_call.is_empty()
            && op.operation_type() == cfg.vw_ks24.weld_op_type
        {
            ins.push(Instr::VwTechCall(cfg.vw_ks24.weld_call.clone()));
        }
        let est_ms = delay + ms(motion_s) + hold;
        if est_ms < MIN_STEP_MS {
            ins.push(Instr::Dwell(MIN_STEP_MS));
        }
        if vwp {
            if is_last && cfg.vw_ks24.makro15 > 0 {
                ins.push(Instr::VwMakro15);
            }
            if mask != 0 {
                ins.push(Instr::VwUnlock(mask));
                held &= !mask;
                if cfg.vw_ks24.online_stop {
                    ins.push(Instr::VwOnlineStop(held));
                }
                // The VW scheme has no 4-phase handshake: hold the release level long enough
                // for the head PLC to see it and drop its grant before the next step.
                ins.push(Instr::Dwell(cfg.vw_ks24.release_dwell_ms));
            }
        } else if mask != 0 {
            ins.push(Instr::Release(mask));
            ins.push(Instr::WaitReleased(mask));
        }
        let est_s = est_ms.max(MIN_STEP_MS) as f64 / 1000.0;
        if est_s > op.limit_s {
            return Err(input(format!(
                "op {k} {}: estimated {est_s:.3} s exceeds the head watchdog limit_s {} \
                 (motion too slow for this model / velocity limits)",
                op.operation_id, op.limit_s
            )));
        }
        if est_s > WARN_FACTOR * op.expected_s {
            warnings.push(format!(
                "robot {ns} op {k} {}: estimated {est_s:.3} s > {WARN_FACTOR} x expected_s {}",
                op.operation_id, op.expected_s
            ));
        }
        estimates.push(OpEstimate {
            index: k,
            est_motion_s: (motion_s * 1000.0).round() / 1000.0,
            operation_id: op.operation_id.clone(),
            zone_mask: op.zone_mask.clone(),
            points: points.len() - n_points_before,
            waypoints: op.waypoints.len(),
            est_s,
            planned_motion_s: op.duration_s,
            expected_s: op.expected_s,
            limit_s: op.limit_s,
        });
        ops.push(OpBlock {
            index: k,
            operation_id: crate::krl::token(&op.operation_id),
            mask,
            instrs: ins,
        });
    }
    let end_dev = motion::max_abs_diff(&prev_end, &home);
    if !p.ops.is_empty() && end_dev > p.protocol.at_home_tol_deg {
        warnings.push(format!(
            "robot {ns}: the last op ends {end_dev:.3} deg from home_pose_deg (AtHome needs {}): \
             the head will home the robot blind after every cycle",
            p.protocol.at_home_tol_deg
        ));
    }
    let init = Instr::Init {
        approx_pct: mc.approx_ptp_pct,
        tool: s.tool_no,
        base: s.base_no,
    };
    let (prologue, epilogue, home_folge) = if vwp {
        // The VW template: AUE_INIT, P0 (= home, 10 %), "wait for Folge start", cycle start,
        // the ops, AUE_END. P0 is the first point of the first op (TASK 7.3).
        let mut pro = vec![
            Instr::VwAueInit,
            Instr::VwP0 { point: 0 },
            Instr::VwAueWait,
            Instr::VwCycleStart {
                mask: all_mask,
                makro50: cfg.vw_ks24.makro50,
            },
        ];
        if cfg.vw_ks24.plan_id_byte != 0 {
            pro.push(Instr::CheckPlan(p.plan_id()));
        }
        if cfg.vw_ks24.online_stop {
            pro.push(Instr::VwOnlineStop(0));
        }
        (pro, vec![Instr::SetStep(0), Instr::VwAueEnd], None)
    } else {
        let mut pro = vec![init.clone(), Instr::CheckPlan(p.plan_id()), Instr::Running];
        if p.ops.first().is_some_and(|o| !o.blind_home) {
            pro.push(Instr::CheckStart {
                point: 0,
                tol_mdeg: ms(p.protocol.start_tol_deg) as u32,
            });
        }
        let home = Folge {
            number: p.folgen.home,
            kind: FolgeKind::Home,
            points: vec![point("PHOME".into(), &home, s)],
            prologue: vec![init, Instr::Running],
            ops: Vec::new(),
            epilogue: vec![
                Instr::HomeMove {
                    point: 0,
                    time_ms: ms(p.protocol.home_return_time_s),
                },
                Instr::Done,
            ],
        };
        (pro, vec![Instr::Finish], Some(home))
    };
    let prod = Folge {
        number: p.folgen.production,
        kind: FolgeKind::Production,
        points,
        prologue,
        ops,
        epilogue,
    };
    Ok(Lowered {
        prod,
        home: home_folge,
        estimates,
        warnings,
        interlocks,
    })
}
