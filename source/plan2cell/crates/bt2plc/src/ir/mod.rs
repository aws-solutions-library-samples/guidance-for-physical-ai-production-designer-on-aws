//! Flavour-independent cell plan: robots, their op programs, cross-robot ordering and zones.
//!
//! Pipeline (DESIGN §3): [`lower`] BT → event graph, [`hb`] happens-before closure and
//! linear extension, [`reduce`] cross-robot edges, [`zones`] chain cover, [`schedule`] ASAP
//! timing and watchdogs, [`validate`] invariants. [`plan`] runs all of it.

pub mod bitset;
pub mod decimate;
pub mod hb;
pub mod lower;
pub mod reduce;
pub mod schedule;
pub mod validate;
pub mod zones;

use serde::Serialize;

use crate::config::{Config, GrantPolicy, RobotAddressing};
use crate::error::{DiagCode, Diagnostics, Error, Result};
use crate::input::{InputDoc, OpRef, Trajectory};
use crate::util::hash::fold16;

pub type OpId = usize;
pub type RobotIdx = usize;
pub type ZoneIdx = usize;

/// REQUEST/GRANT bit used for permit predecessors (zones use bits 0..30).
pub const PERMIT_BIT: u32 = 1 << 31;

/// Profile `vw-ks24`: the VW interlock index of zone `z` for robot `r` (1..16, 0 = not a
/// member). Configured per robot (`[[robot]] interlocks`), else the zones a robot uses are
/// numbered in zone order; the same map is written to program.json so bt2krl uses it too.
pub fn interlock_map(plan: &CellPlan, cfg: &Config) -> Vec<Vec<u8>> {
    plan.robots
        .iter()
        .map(|r| {
            let cfgmap = cfg
                .robot_cfg(&r.ns)
                .map(|c| c.interlocks.clone())
                .unwrap_or_default();
            let used: Vec<usize> = plan
                .zones
                .iter()
                .filter(|z| z.tickets.iter().any(|t| t.robot == r.index))
                .map(|z| z.index)
                .collect();
            let mut out = vec![0u8; plan.zones.len()];
            for (i, &z) in used.iter().enumerate() {
                out[z] = match cfgmap.get(&plan.zones[z].name) {
                    Some(&n) => n,
                    None => (i + 1) as u8,
                };
            }
            out
        })
        .collect()
}

/// Robot error code (bytes 10-11): job number is neither the production nor the HOME Folge.
pub const ERR_UNKNOWN_FOLGE: u16 = 0x0101;
/// Robot error code: PLAN_ID from the head differs from the robot program's plan id.
pub const ERR_PLAN_MISMATCH: u16 = 0x0103;
/// Robot warning code: the pose at the op start deviates from the first waypoint.
pub const WARN_START_DEVIATION: u16 = 0x0201;

#[derive(Clone, Debug, Serialize)]
pub struct CellPlan {
    pub tree_id: String,
    pub cell_id: String,
    pub source_file: String,
    pub fingerprint: u64,
    /// Non-zero 16-bit id written to the robots with the production job.
    pub plan_id: u16,
    /// BT order, then idle robots from the config.
    pub robots: Vec<Robot>,
    /// Sorted by global linear-extension position (`ops[i].pos == i`).
    pub ops: Vec<Op>,
    /// Reduced cross-robot edges, sorted by (pos(to), pos(from)).
    pub cross_edges: Vec<Edge>,
    pub zones: Vec<Zone>,
    /// Reduced edges enforced by the permit bit instead of a zone.
    pub permit_edges: Vec<Edge>,
    pub grant_policy: GrantPolicy,
    /// A CheckSafety node was present (mapped to continuous supervision).
    pub safety_check: bool,
    pub makespan_ms: u64,
    /// Happens-before over ops (ancestor sets), for validation and tests.
    #[serde(skip)]
    pub hb: hb::Hb,
}

#[derive(Clone, Debug, Serialize)]
pub struct Robot {
    pub index: RobotIdx,
    pub ns: String,
    /// Program order; op `ops[k]` has 1-based index `k + 1`.
    pub ops: Vec<OpId>,
    pub home_pose_rad: [f64; 6],
    pub home_source: HomeSource,
    pub addr: RobotAddressing,
    pub idle: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HomeSource {
    Config,
    FinalMoveHome,
    FirstWaypoint,
    Fallback,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum OpKind {
    Execute { operation_type: String },
    MoveHome,
}

#[derive(Clone, Debug, Serialize)]
pub struct BtRef {
    pub node_id: Option<String>,
    pub name: Option<String>,
    pub path: String,
    pub dfs: usize,
}

#[derive(Clone, Debug, Serialize)]
pub struct Op {
    pub id: OpId,
    pub robot: RobotIdx,
    /// 1-based index in the robot program.
    pub index: u16,
    pub operation_id: String,
    pub kind: OpKind,
    pub bt: BtRef,
    pub pos: usize,
    /// `None` = blind MoveHome.
    pub trajectory: Option<Trajectory>,
    pub hold_ms: u64,
    pub velocity_scale: f64,
    pub start_delay_ms: u64,
    pub delay_exact: bool,
    pub timeout_override_ms: Option<u64>,
    pub zones: Vec<ZoneIdx>,
    /// Zone bits plus [`PERMIT_BIT`] when `permit_preds` is not empty.
    pub zone_mask: u32,
    /// Cross-robot predecessors enforced by the permit bit (zone capacity overflow).
    pub permit_preds: Vec<OpId>,
    /// Motion time: trajectory / velocity_scale (or blind home time), without hold and delay.
    pub motion_ms: u64,
    /// Grant → done on the robot: start delay + motion + hold + completion margin.
    pub expected_ms: u64,
    /// Per-op watchdog (from grant, or from the step change for ops without zones).
    pub limit_ms: u64,
    /// Latest acceptable grant time, measured from the cycle start.
    pub zone_deadline_ms: u64,
    /// ASAP grant / end times at expected durations (from the cycle start).
    pub asap_start_ms: u64,
    pub asap_end_ms: u64,
    pub depth: u32,
}

impl Op {
    pub fn label(&self) -> String {
        self.operation_id.clone()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct Edge {
    pub from: OpId,
    pub to: OpId,
}

#[derive(Clone, Debug, Serialize)]
pub struct Zone {
    pub index: ZoneIdx,
    /// "IL01", ...
    pub name: String,
    pub tickets: Vec<Ticket>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Ticket {
    pub op: OpId,
    pub robot: RobotIdx,
    pub op_index: u16,
}

impl CellPlan {
    pub fn robot_ops(&self, r: RobotIdx) -> impl Iterator<Item = &Op> {
        self.robots[r].ops.iter().map(move |&o| &self.ops[o])
    }

    pub fn op_ref(&self, o: OpId) -> String {
        let op = &self.ops[o];
        format!("{}.{}", self.robots[op.robot].ns, op.operation_id)
    }

    pub fn active_robots(&self) -> impl Iterator<Item = &Robot> {
        self.robots.iter().filter(|r| !r.idle)
    }
}

/// Run the full IR pipeline.
pub fn plan(doc: &InputDoc, cfg: &Config, d: &mut Diagnostics) -> Result<CellPlan> {
    let mut g = lower::lower(doc, cfg, d)?;
    if g.ops.is_empty() {
        return Err(Error::plan(
            DiagCode::E014,
            None,
            "the tree contains no ExecuteTrajectory or MoveHome",
        ));
    }
    hb::linearize(&mut g)?;
    let hb = hb::closure(&g);
    let robots = hb::robot_chains(&g, &hb, cfg, d)?;
    let edges = reduce::reduce(&hb, &g, &robots, cfg.cell.prune_transitive);
    let cover = zones::cover(&hb, &edges, &g, cfg.protocol.max_zones as usize)?;
    if !cover.permit_edges.is_empty() {
        d.warn(
            DiagCode::W021,
            None,
            format!(
                "{} cross-robot dependencies exceed the {} zones and are enforced by the permit bit",
                cover.permit_edges.len(),
                cfg.protocol.max_zones
            ),
        );
    }
    if cfg.cell.grant_policy == GrantPolicy::Fcfs {
        d.warn(
            DiagCode::W011,
            None,
            "grant policy FCFS: zones are granted first-come-first-served, so the BT's cross-robot \
             ORDER is no longer enforced (only mutual exclusion). Robots may combine motions the \
             planner never collision-checked. Use only for zones that are true symmetric physical regions.",
        );
    }
    let mut ops: Vec<Op> = g
        .ops
        .iter()
        .enumerate()
        .map(|(id, o)| o.to_op(id))
        .collect();
    for r in &robots {
        for (k, &o) in r.ops.iter().enumerate() {
            ops[o].robot = r.index;
            ops[o].index = (k + 1) as u16;
        }
    }
    let zones: Vec<Zone> = cover
        .zones
        .into_iter()
        .enumerate()
        .map(|(zi, members)| Zone {
            index: zi,
            name: format!("IL{:02}", zi + 1),
            tickets: members
                .iter()
                .map(|&o| Ticket {
                    op: o,
                    robot: ops[o].robot,
                    op_index: ops[o].index,
                })
                .collect(),
        })
        .collect();
    for z in &zones {
        for t in &z.tickets {
            ops[t.op].zones.push(z.index);
            ops[t.op].zone_mask |= 1u32 << z.index;
        }
    }
    for e in &cover.permit_edges {
        ops[e.to].permit_preds.push(e.from);
        ops[e.to].zone_mask |= PERMIT_BIT;
    }
    // The robot programs and the head tables depend on the plan text, the effective blackboard
    // (e.g. a --blackboard file for XML input) and the configuration: hash all of them.
    let fingerprint = {
        let bb: String = doc
            .blackboard
            .0
            .iter()
            .map(|(k, v)| format!("{k}={}\n", crate::util::hash::canonical_json(v)))
            .collect();
        // The OPC UA symbol configuration changes no robot program and no head table: left out,
        // so switching it on keeps the plan id (and the KRL already on the controllers).
        let c = {
            let mut c = cfg.clone();
            c.opcua = Default::default();
            toml::to_string(&c).unwrap_or_default()
        };
        let generator = env!("CARGO_PKG_VERSION");
        crate::util::hash::fnv1a64(
            format!("{:016x}\n{bb}\n{c}\n{generator}", doc.fingerprint).as_bytes(),
        )
    };
    let mut plan = CellPlan {
        tree_id: doc.tree_id.clone(),
        cell_id: doc.cell_id.clone(),
        source_file: doc.file_name.clone(),
        fingerprint,
        plan_id: fold16(fingerprint),
        robots,
        ops,
        cross_edges: edges,
        zones,
        permit_edges: cover.permit_edges,
        grant_policy: cfg.cell.grant_policy,
        safety_check: g.safety_check,
        makespan_ms: 0,
        hb,
    };
    schedule::schedule(&mut plan, &g, cfg);
    schedule::delay_warnings(&plan, d);
    schedule::check_timeouts(&plan)?;
    hb::home_poses(&mut plan, cfg, d);
    validate::validate(&plan)?;
    Ok(plan)
}

/// `robot.operation_id` of a `depends_on_ops` reference, for messages.
pub fn opref_str(r: &OpRef) -> String {
    format!("{}.{}", r.robot, r.operation_id)
}
