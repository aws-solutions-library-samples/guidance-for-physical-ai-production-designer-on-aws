//! `robots/<ns>/program.json` (schema `bt2plc/robot-program@1`, `schema_minor` 1): the robot
//! program spec (ops, zones, handshake, timing, trajectories in degrees) for the KRL generator
//! `bt2krl`.
//!
//! Minor 1 (additive): `schema_minor`, `robot.model`, `zones[]`, the robot-side `protocol`
//! constants (`profile`, `job_number`, `error_codes`, tolerances, home time, head timeouts),
//! per op `expected_s`/`limit_s`/`delay_exact`, per waypoint `phase`.

use serde::Serialize;

use crate::config::{Config, JobNumber};
use crate::input::Phase;
use crate::ir::{
    CellPlan, ERR_PLAN_MISMATCH, ERR_UNKNOWN_FOLGE, OpKind, PERMIT_BIT, WARN_START_DEVIATION,
};

/// Bumped on every additive change of the `bt2plc/robot-program@1` schema.
pub const SCHEMA_MINOR: u32 = 1;

#[derive(Serialize)]
struct Doc<'a> {
    schema: &'static str,
    schema_minor: u32,
    plan: PlanRef<'a>,
    robot: RobotRef<'a>,
    folgen: Folgen,
    home_pose_deg: Vec<f64>,
    joint_mapping: JointMap<'a>,
    protocol: Protocol,
    zones: Vec<ZoneRef<'a>>,
    ops: Vec<OpSpec<'a>>,
}

#[derive(Serialize)]
struct ZoneRef<'a> {
    name: &'a str,
    /// REQUEST/GRANT bit (0-based); the permit bit is `protocol.permit_bit`.
    bit: u8,
    /// Profile `vw-ks24`: the VW interlock index of this zone for this robot (1..16, 0 = the
    /// robot is not a member). Absent in profile `sim`.
    #[serde(skip_serializing_if = "Option::is_none")]
    interlock: Option<u8>,
}

#[derive(Serialize)]
struct PlanRef<'a> {
    tree_id: &'a str,
    cell_id: &'a str,
    plan_id: String,
}

#[derive(Serialize)]
struct RobotRef<'a> {
    ns: &'a str,
    tag: &'a str,
    index: usize,
    station: &'a str,
    ip: String,
    model: &'a str,
}

#[derive(Serialize)]
struct Folgen {
    production: u16,
    home: u16,
}

#[derive(Serialize)]
struct JointMap<'a> {
    /// `"identity"`, a model preset (UNVALIDATED calibration) or `"custom"`.
    preset: &'a str,
    sign: [i8; 6],
    offset_deg: [f64; 6],
    source_unit: &'static str,
}

#[derive(Serialize)]
struct Field {
    byte: u8,
    len: u8,
    endian: &'static str,
    dir: &'static str,
}

#[derive(Serialize)]
struct Protocol {
    step: Field,
    request: Field,
    grant: Field,
    plan_id: Field,
    permit_bit: u8,
    handshake: &'static str,
    grant_policy: String,
    /// Robot-side protocol profile (`"sim"`: the bt2plc device simulator telegram).
    profile: &'static str,
    job_number: JobNumber,
    error_codes: ErrorCodes,
    /// AtHome = every axis within this of `home_pose_deg`.
    at_home_tol_deg: f64,
    /// Warning `start_deviation` when an op starts further than this from its first waypoint.
    start_tol_deg: f64,
    /// Duration of a blind MoveHome op and of the HOME Folge.
    home_return_time_s: f64,
    /// Head-PLC timeout of every job-handshake step (Ready, JobAck, ProgRunning after
    /// ProgStart, step 1 after ProgRunning).
    handshake_s: f64,
    /// Head-PLC limit: ProgRunning must drop this long after Stop/Cancel.
    abort_limit_s: f64,
    /// Head-PLC limit: the HOME Folge must end (ProgFinished and AtHome) within this.
    home_limit_s: f64,
}

#[derive(Serialize)]
struct ErrorCodes {
    unknown_folge: u16,
    plan_mismatch: u16,
    start_deviation: u16,
}

#[derive(Serialize)]
struct Wp<'a> {
    t_s: f64,
    q_deg: [f64; 6],
    phase: &'a str,
}

#[derive(Serialize)]
struct OpSpec<'a> {
    index: u16,
    operation_id: &'a str,
    kind: &'a OpKind,
    bt_path: &'a str,
    zones: Vec<String>,
    zone_mask: String,
    waits_for: Vec<String>,
    start_delay_s: f64,
    hold_s: f64,
    velocity_scale: f64,
    duration_s: f64,
    /// Planned grant -> done on the robot (start delay + motion + hold + margin).
    expected_s: f64,
    /// Head per-op watchdog, from the grant (or the step change for ops without zones).
    limit_s: f64,
    /// The start delay is the BT Delay exactly (not a shared-path approximation).
    delay_exact: bool,
    blind_home: bool,
    trajectory_key: Option<&'a str>,
    waypoints: Vec<Wp<'a>>,
}

fn phase_str(p: &Phase) -> &str {
    match p {
        Phase::Transit => "transit",
        Phase::Approach => "approach",
        Phase::Other(s) => s,
    }
}

fn r4(x: f64) -> f64 {
    let r = (x * 10000.0).round() / 10000.0;
    if r == 0.0 { 0.0 } else { r }
}

pub fn render(plan: &CellPlan, cfg: &Config, ri: usize) -> String {
    let r = &plan.robots[ri];
    let il = cfg
        .protocol
        .profile
        .is_vw()
        .then(|| crate::ir::interlock_map(plan, cfg));
    let p = &cfg.protocol;
    let map = |q: &[f64; 6]| {
        let mut o = [0.0; 6];
        for i in 0..6 {
            o[i] =
                r4(f64::from(r.addr.joint_sign[i]) * q[i].to_degrees()
                    + r.addr.joint_offset_deg[i]);
        }
        o
    };
    let ops = r
        .ops
        .iter()
        .map(|&oi| {
            let op = &plan.ops[oi];
            let mut waits: Vec<String> = Vec::new();
            for z in &op.zones {
                let zone = &plan.zones[*z];
                if let Some(k) = zone.tickets.iter().position(|t| t.op == oi)
                    && k > 0
                {
                    waits.push(format!(
                        "{} after {}",
                        zone.name,
                        plan.op_ref(zone.tickets[k - 1].op)
                    ));
                }
            }
            for &u in &op.permit_preds {
                waits.push(format!("PERMIT after {}", plan.op_ref(u)));
            }
            OpSpec {
                index: op.index,
                operation_id: &op.operation_id,
                kind: &op.kind,
                bt_path: &op.bt.path,
                zones: op
                    .zones
                    .iter()
                    .map(|&z| plan.zones[z].name.clone())
                    .collect(),
                zone_mask: format!("16#{:08X}", op.zone_mask),
                waits_for: waits,
                start_delay_s: op.start_delay_ms as f64 / 1000.0,
                hold_s: op.hold_ms as f64 / 1000.0,
                velocity_scale: op.velocity_scale,
                duration_s: op.motion_ms as f64 / 1000.0,
                expected_s: op.expected_ms as f64 / 1000.0,
                limit_s: op.limit_ms as f64 / 1000.0,
                delay_exact: op.delay_exact,
                blind_home: op.trajectory.is_none(),
                trajectory_key: op.trajectory.as_ref().and_then(|t| t.key.as_deref()),
                waypoints: op
                    .trajectory
                    .as_ref()
                    .map(|t| {
                        t.waypoints
                            .iter()
                            .map(|w| Wp {
                                t_s: r4(w.t_s),
                                q_deg: map(&w.q_rad),
                                phase: phase_str(&w.phase),
                            })
                            .collect()
                    })
                    .unwrap_or_default(),
            }
        })
        .collect();
    let doc = Doc {
        schema: "bt2plc/robot-program@1",
        schema_minor: SCHEMA_MINOR,
        plan: PlanRef {
            tree_id: &plan.tree_id,
            cell_id: &plan.cell_id,
            plan_id: format!("16#{:04X}", plan.plan_id),
        },
        robot: RobotRef {
            ns: &r.ns,
            tag: &r.addr.tag,
            index: ri + 1,
            station: &r.addr.station,
            ip: r.addr.ip.to_string(),
            model: cfg
                .robot_cfg(&r.ns)
                .and_then(|c| c.model.as_deref())
                .unwrap_or("unknown"),
        },
        folgen: Folgen {
            production: r.addr.prod_folge,
            home: cfg.cell.home_folge,
        },
        home_pose_deg: map(&r.home_pose_rad).to_vec(),
        joint_mapping: JointMap {
            preset: &r.addr.joint_map,
            sign: r.addr.joint_sign,
            offset_deg: r.addr.joint_offset_deg,
            source_unit: "rad",
        },
        protocol: Protocol {
            step: Field {
                byte: p.step_byte,
                len: 2,
                endian: "little",
                dir: "robot_to_plc",
            },
            request: Field {
                byte: p.request_byte,
                len: 4,
                endian: "little",
                dir: "robot_to_plc",
            },
            grant: Field {
                byte: p.grant_byte,
                len: 4,
                endian: "little",
                dir: "plc_to_robot",
            },
            plan_id: Field {
                byte: p.plan_id_byte,
                len: 2,
                endian: "little",
                dir: "plc_to_robot",
            },
            permit_bit: PERMIT_BIT.trailing_zeros() as u8,
            handshake: "4-phase: step := k; REQUEST := mask; wait GRANT&mask = mask; [delay] move; hold; REQUEST := 0; wait GRANT&mask = 0",
            grant_policy: format!("{:?}", plan.grant_policy).to_lowercase(),
            profile: cfg.protocol.profile.name(),
            job_number: cfg.cell.job_number,
            error_codes: ErrorCodes {
                unknown_folge: ERR_UNKNOWN_FOLGE,
                plan_mismatch: ERR_PLAN_MISMATCH,
                start_deviation: WARN_START_DEVIATION,
            },
            at_home_tol_deg: cfg.robot_sim.at_home_tol_deg,
            start_tol_deg: cfg.robot_sim.start_tol_deg,
            home_return_time_s: cfg.timing.home_return_time_s,
            handshake_s: f64::from(p.handshake_timeout_ms) / 1000.0,
            abort_limit_s: f64::from(p.abort_limit_ms) / 1000.0,
            home_limit_s: r4(cfg.timing.home_limit_s()),
        },
        zones: plan
            .zones
            .iter()
            .map(|z| ZoneRef {
                name: &z.name,
                bit: z.index as u8,
                interlock: il.as_ref().map(|m| m[ri][z.index]),
            })
            .collect(),
        ops,
    };
    serde_json::to_string_pretty(&doc).expect("program serialises") + "\n"
}
