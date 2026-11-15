//! BT → event graph (DESIGN §3.1).
//!
//! Every node returns its first/last frontier; a Sequence connects the last frontier of a
//! child to the first frontier of the next (through a barrier event when both sides have more
//! than one event, keeping the edge count linear). Parallel unions frontiers. WaitForRobot is a
//! gate event whose predecessors are the referenced ops (`depends_on_ops`). Delay is an event
//! carrying milliseconds; Timeout narrows the watchdog of the ops below it.

use std::collections::{BTreeMap, BTreeSet};

use crate::config::Config;
use crate::error::{DiagCode, Diagnostics, Error, Result};
use crate::input::{InputDoc, NodeType, NormNode, OpRef, Trajectory, blackboard};
use crate::ir::{BtRef, Op, OpId, OpKind};

#[derive(Clone, Debug)]
pub enum EventKind {
    Op(OpId),
    Gate { robot: String, refs: Vec<OpRef> },
    Barrier,
    Delay { ms: u64 },
}

#[derive(Clone, Debug)]
pub struct Event {
    pub kind: EventKind,
    /// DFS index of the creating node (deterministic tie-break).
    pub key: usize,
    pub path: String,
}

#[derive(Clone, Debug)]
pub struct OpDraft {
    pub robot_ns: String,
    pub operation_id: String,
    pub kind: OpKind,
    pub trajectory: Option<Trajectory>,
    pub hold_s: f64,
    pub velocity_scale: f64,
    pub timeout_override_ms: Option<u64>,
    pub bt: BtRef,
    pub event: usize,
}

impl OpDraft {
    pub fn to_op(&self, id: OpId) -> Op {
        Op {
            id,
            robot: 0,
            index: 0,
            operation_id: self.operation_id.clone(),
            kind: self.kind.clone(),
            bt: self.bt.clone(),
            pos: id,
            trajectory: self.trajectory.clone(),
            hold_ms: (self.hold_s * 1000.0).round() as u64,
            velocity_scale: self.velocity_scale,
            start_delay_ms: 0,
            delay_exact: true,
            timeout_override_ms: self.timeout_override_ms,
            zones: Vec::new(),
            zone_mask: 0,
            permit_preds: Vec::new(),
            motion_ms: 0,
            expected_ms: 0,
            limit_ms: 0,
            zone_deadline_ms: 0,
            asap_start_ms: 0,
            asap_end_ms: 0,
            depth: 0,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct EventGraph {
    pub ev: Vec<Event>,
    pub succ: Vec<Vec<usize>>,
    pub pred: Vec<Vec<usize>>,
    pub ops: Vec<OpDraft>,
    /// Topological order of events (filled by `hb::linearize`).
    pub order: Vec<usize>,
    pub safety_check: bool,
}

impl EventGraph {
    fn add(&mut self, kind: EventKind, key: usize, path: &str) -> usize {
        self.ev.push(Event {
            kind,
            key,
            path: path.to_string(),
        });
        self.succ.push(Vec::new());
        self.pred.push(Vec::new());
        self.ev.len() - 1
    }

    pub fn edge(&mut self, a: usize, b: usize) {
        if !self.succ[a].contains(&b) {
            self.succ[a].push(b);
            self.pred[b].push(a);
        }
    }

    pub fn op_of(&self, e: usize) -> Option<OpId> {
        match self.ev[e].kind {
            EventKind::Op(o) => Some(o),
            _ => None,
        }
    }
}

#[derive(Default)]
struct Frontier {
    first: Vec<usize>,
    last: Vec<usize>,
}

impl Frontier {
    fn single(e: usize) -> Self {
        Frontier {
            first: vec![e],
            last: vec![e],
        }
    }

    fn is_empty(&self) -> bool {
        self.first.is_empty()
    }
}

struct Ctx<'a> {
    doc: &'a InputDoc,
    allow_ignored: bool,
    timeout_ms: Option<u64>,
    gates: Vec<usize>,
    home_ids: BTreeMap<String, usize>,
}

pub fn lower(doc: &InputDoc, cfg: &Config, d: &mut Diagnostics) -> Result<EventGraph> {
    let mut g = EventGraph::default();
    let mut ctx = Ctx {
        doc,
        allow_ignored: cfg.cell.ignore_unsupported_gates,
        timeout_ms: None,
        gates: Vec::new(),
        home_ids: BTreeMap::new(),
    };
    lower_node(&doc.root, &mut g, &mut ctx, d)?;
    // Resolve gate references to ExecuteTrajectory ops.
    let mut index: BTreeMap<(String, String), OpId> = BTreeMap::new();
    for (i, o) in g.ops.iter().enumerate() {
        if matches!(o.kind, OpKind::Execute { .. }) {
            let k = (o.robot_ns.clone(), o.operation_id.clone());
            if let Some(&prev) = index.get(&k) {
                return Err(Error::plan(
                    DiagCode::E007,
                    Some(&o.bt.path),
                    format!(
                        "duplicate op {}.{} (also at {}); depends_on_ops references would be ambiguous",
                        k.0, k.1, g.ops[prev].bt.path
                    ),
                ));
            }
            index.insert(k, i);
        }
    }
    // MoveHome ids must not collide with ExecuteTrajectory ids of the same robot.
    for i in 0..g.ops.len() {
        if g.ops[i].kind != OpKind::MoveHome {
            continue;
        }
        let ns = g.ops[i].robot_ns.clone();
        let mut id = g.ops[i].operation_id.clone();
        while g
            .ops
            .iter()
            .enumerate()
            .any(|(j, o)| j != i && o.robot_ns == ns && o.operation_id == id)
        {
            id.push('_');
        }
        g.ops[i].operation_id = id;
    }
    for gi in ctx.gates.clone() {
        let EventKind::Gate { refs, .. } = g.ev[gi].kind.clone() else {
            unreachable!()
        };
        for r in refs {
            let Some(&o) = index.get(&(r.robot.clone(), r.operation_id.clone())) else {
                return Err(Error::plan(
                    DiagCode::E006,
                    Some(&g.ev[gi].path),
                    format!(
                        "depends_on_ops refers to {}.{} which is not an ExecuteTrajectory in this tree",
                        r.robot, r.operation_id
                    ),
                ));
            };
            let oe = g.ops[o].event;
            g.edge(oe, gi);
        }
    }
    Ok(g)
}

fn connect(g: &mut EventGraph, last: &[usize], first: &[usize], key: usize, path: &str) {
    if last.len() == 1 || first.len() == 1 {
        for &a in last {
            for &b in first {
                g.edge(a, b);
            }
        }
    } else {
        let b = g.add(EventKind::Barrier, key, path);
        for &a in last {
            g.edge(a, b);
        }
        for &f in first {
            g.edge(b, f);
        }
    }
}

fn lower_node(
    n: &NormNode,
    g: &mut EventGraph,
    ctx: &mut Ctx,
    d: &mut Diagnostics,
) -> Result<Frontier> {
    let path = n.path.0.as_str();
    let leaf = matches!(
        n.ty,
        NodeType::ExecuteTrajectory
            | NodeType::MoveHome
            | NodeType::WaitForRobot
            | NodeType::CheckSafety
    );
    if leaf && !n.children.is_empty() {
        return Err(Error::plan(
            DiagCode::E015,
            Some(path),
            format!(
                "{} is a leaf but has {} child node(s)",
                n.raw_type,
                n.children.len()
            ),
        ));
    }
    if n.ty == NodeType::SubTree && n.children.is_empty() {
        let id = n.port_str("ID").unwrap_or_else(|| n.label());
        return Err(Error::plan(
            DiagCode::E002,
            Some(path),
            format!("SubTree {id} is not expanded (missing or recursive tree)"),
        ));
    }
    match &n.ty {
        NodeType::ExecuteTrajectory => lower_execute(n, g, ctx, d),
        NodeType::MoveHome => lower_home(n, g, ctx, d),
        NodeType::WaitForRobot => {
            for (port, code, what) in [
                ("depends_on_joints", DiagCode::W001, "joint-pose waits"),
                (
                    "depends_on_positions",
                    DiagCode::W002,
                    "Cartesian position waits",
                ),
            ] {
                if n.port(port).is_some() {
                    gate_unsupported(
                        ctx,
                        d,
                        code,
                        path,
                        format!("{port}: {what} are not supported by the VASS zone protocol"),
                    )?;
                }
            }
            if n.port("depends_on_robots").is_some() {
                d.warn(
                    DiagCode::W004,
                    Some(path),
                    "depends_on_robots ignored (unused by the reference executor)",
                );
            }
            let mut refs: Vec<OpRef> = n.port_list("depends_on_ops", &ctx.doc.blackboard)?;
            refs.sort();
            refs.dedup();
            if refs.is_empty() {
                return Ok(Frontier::default());
            }
            let robot = n.port_str("robot_ns").unwrap_or_default();
            let e = g.add(EventKind::Gate { robot, refs }, n.dfs, path);
            ctx.gates.push(e);
            Ok(Frontier::single(e))
        }
        NodeType::CheckSafety => {
            g.safety_check = true;
            let cond = n
                .port_str("condition")
                .unwrap_or_else(|| "estop_clear".into());
            d.warn(
                DiagCode::W005,
                Some(path),
                format!("CheckSafety({cond}) is mapped to continuous safety supervision of all robots, not a point check"),
            );
            Ok(Frontier::default())
        }
        NodeType::Sequence | NodeType::SequenceWithMemory | NodeType::SubTree => {
            let mut acc = Frontier::default();
            for c in &n.children {
                let f = lower_node(c, g, ctx, d)?;
                if f.is_empty() {
                    continue;
                }
                if acc.is_empty() {
                    acc = f;
                } else {
                    connect(g, &acc.last, &f.first, c.dfs, &c.path.0);
                    acc.last = f.last;
                }
            }
            Ok(acc)
        }
        NodeType::Parallel => {
            let sc = int_port(n, &["success_count", "success_threshold"])?.unwrap_or(-1);
            let fc = int_port(n, &["failure_count", "failure_threshold"])?.unwrap_or(1);
            let nchild = n.children.len() as i64;
            if !((sc == -1 || sc == nchild) && fc == 1) {
                return Err(Error::plan(
                    DiagCode::E003,
                    Some(path),
                    format!(
                        "Parallel success_count={sc} failure_count={fc}: only 'all must succeed, first failure fails' \
                         (success_count=-1, failure_count=1) maps to the zone protocol"
                    ),
                ));
            }
            let mut acc = Frontier::default();
            for c in &n.children {
                let f = lower_node(c, g, ctx, d)?;
                acc.first.extend(f.first);
                acc.last.extend(f.last);
            }
            Ok(acc)
        }
        NodeType::Delay => {
            let [child] = n.children.as_slice() else {
                return Err(Error::plan(
                    DiagCode::E015,
                    Some(path),
                    format!("Delay needs exactly one child, has {}", n.children.len()),
                ));
            };
            let ms = n.port_f64("delay_msec")?.unwrap_or(0.0).max(0.0);
            if ms > MAX_SECONDS * 1000.0 {
                return Err(Error::plan(
                    DiagCode::E023,
                    Some(path),
                    format!("delay_msec {ms} exceeds {MAX_SECONDS} s"),
                ));
            }
            let ms = ms.round() as u64;
            let de = g.add(EventKind::Delay { ms }, n.dfs, path);
            let f = lower_node(child, g, ctx, d)?;
            if f.is_empty() {
                return Ok(Frontier::single(de));
            }
            for &x in &f.first {
                g.edge(de, x);
            }
            Ok(Frontier {
                first: vec![de],
                last: f.last,
            })
        }
        NodeType::Timeout => {
            let [child] = n.children.as_slice() else {
                return Err(Error::plan(
                    DiagCode::E015,
                    Some(path),
                    format!("Timeout needs exactly one child, has {}", n.children.len()),
                ));
            };
            let ms = n.port_f64("msec")?.unwrap_or(0.0).max(0.0);
            if ms > MAX_SECONDS * 1000.0 {
                return Err(Error::plan(
                    DiagCode::E023,
                    Some(path),
                    format!("Timeout msec {ms} exceeds {MAX_SECONDS} s"),
                ));
            }
            let ms = ms.round() as u64;
            let before = g.ops.len();
            let saved = ctx.timeout_ms;
            ctx.timeout_ms = Some(saved.map_or(ms, |s| s.min(ms)));
            let f = lower_node(child, g, ctx, d)?;
            ctx.timeout_ms = saved;
            if g.ops.len() == before {
                d.warn(
                    DiagCode::W006,
                    Some(path),
                    format!("Timeout({ms} ms) contains no op and has no effect"),
                );
            }
            if g.ops.len() - before > 1 {
                d.warn(
                    DiagCode::W006,
                    Some(path),
                    format!(
                        "Timeout({ms} ms) over {} ops is applied as a per-op watchdog limit",
                        g.ops.len() - before
                    ),
                );
            }
            Ok(f)
        }
        NodeType::Unsupported(t) => Err(Error::plan(
            DiagCode::E002,
            Some(path),
            format!(
                "unsupported node type {t}; supported: Sequence, SequenceWithMemory, Parallel, Delay, Timeout, \
                 SubTree, ExecuteTrajectory, MoveHome, WaitForRobot, CheckSafety"
            ),
        )),
    }
}

/// Longest accepted duration of a single op, delay or timeout (keeps every PLC TIME value far
/// below the 32-bit millisecond range of about 49.7 days).
pub const MAX_SECONDS: f64 = 86_400.0;

fn interlock_target_resolves(n: &NormNode, bb: &crate::input::Blackboard) -> bool {
    let Some(v) = n.port("interlock_target") else {
        return false;
    };
    let json = match v.pointer() {
        Some(k) => bb.get(k).cloned(),
        None => Some(v.to_json()),
    };
    matches!(json, Some(serde_json::Value::Object(_)))
}

/// Unsupported gating drops an ordering the reference executor enforces: an error unless
/// `cell.ignore_unsupported_gates` downgrades it to a warning.
fn gate_unsupported(
    ctx: &Ctx,
    d: &mut Diagnostics,
    warn: DiagCode,
    path: &str,
    msg: String,
) -> Result<()> {
    if ctx.allow_ignored {
        d.warn(
            warn,
            Some(path),
            format!("{msg}; ignored (cell.ignore_unsupported_gates)"),
        );
        Ok(())
    } else {
        let code = if warn == DiagCode::W003 {
            DiagCode::E020
        } else {
            DiagCode::E019
        };
        Err(Error::plan(
            code,
            Some(path),
            format!("{msg} (set cell.ignore_unsupported_gates = true to ignore it)"),
        ))
    }
}

fn int_port(n: &NormNode, names: &[&str]) -> Result<Option<i64>> {
    for name in names {
        if let Some(v) = n.port_f64(name)? {
            return Ok(Some(v.round() as i64));
        }
    }
    Ok(None)
}

fn required(n: &NormNode, port: &str) -> Result<String> {
    n.port_str(port)
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| {
            Error::plan(
                DiagCode::E010,
                Some(&n.path.0),
                format!("{} needs port {port}", n.raw_type),
            )
        })
}

fn bt_ref(n: &NormNode) -> BtRef {
    BtRef {
        node_id: n.id.clone(),
        name: n.name.clone(),
        path: n.path.0.clone(),
        dfs: n.dfs,
    }
}

pub(crate) fn validate_ns(ns: &str, path: &str) -> Result<()> {
    if ns.eq_ignore_ascii_case("head") {
        return Err(Error::plan(
            DiagCode::E018,
            Some(path),
            "robot_ns 'head' is reserved (it names the head PLC project)",
        ));
    }
    if ns.is_empty()
        || ns.len() > 32
        || !ns
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
    {
        return Err(Error::plan(
            DiagCode::E018,
            Some(path),
            format!(
                "robot_ns '{ns}' must match [A-Za-z0-9_-]{{1,32}} (it names files and PLC objects)"
            ),
        ));
    }
    Ok(())
}

fn lower_execute(
    n: &NormNode,
    g: &mut EventGraph,
    ctx: &mut Ctx,
    d: &mut Diagnostics,
) -> Result<Frontier> {
    let path = n.path.0.as_str();
    let robot_ns = required(n, "robot_ns")?;
    validate_ns(&robot_ns, path)?;
    let operation_id = required(n, "operation_id")?;
    let operation_type = n
        .port_str("operation_type")
        .unwrap_or_else(|| "weld".into());
    let trajectory = blackboard::trajectory(n, &ctx.doc.blackboard, d)?.ok_or_else(|| {
        Error::plan(
            DiagCode::E008,
            Some(path),
            format!("ExecuteTrajectory {robot_ns}.{operation_id} has no trajectory"),
        )
    })?;
    let hold_s = n.port_f64("hold_duration_s")?.unwrap_or(0.5).max(0.0);
    if hold_s > MAX_SECONDS {
        return Err(Error::plan(
            DiagCode::E023,
            Some(path),
            format!("hold_duration_s {hold_s} exceeds {MAX_SECONDS} s"),
        ));
    }
    let velocity_scale = n.port_f64("velocity_scale")?.unwrap_or(1.0);
    if !(0.01..=100.0).contains(&velocity_scale) {
        return Err(Error::plan(
            DiagCode::E010,
            Some(path),
            format!("velocity_scale {velocity_scale} must be within 0.01..100"),
        ));
    }
    if trajectory.duration_s() > MAX_SECONDS {
        return Err(Error::plan(
            DiagCode::E023,
            Some(path),
            format!(
                "trajectory lasts {} s (> {MAX_SECONDS} s)",
                trajectory.duration_s()
            ),
        ));
    }
    if (velocity_scale - 1.0).abs() > 1e-9 {
        d.warn(
            DiagCode::I008,
            Some(path),
            format!("velocity_scale {velocity_scale}: playback and watchdogs are scaled"),
        );
    }
    // Same condition as the reference executor: only has_interlock with a resolvable target moves.
    if n.port_bool("has_interlock")?.unwrap_or(false)
        && interlock_target_resolves(n, &ctx.doc.blackboard)
    {
        gate_unsupported(
            ctx,
            d,
            DiagCode::W003,
            path,
            "has_interlock/interlock_target: the interlock pre-move is not supported".into(),
        )?;
    }
    if n.port("depends_on_robots").is_some() {
        d.warn(
            DiagCode::W004,
            Some(path),
            "depends_on_robots ignored (unused by the reference executor)",
        );
    }
    if n.port("depends_on_positions").is_some() {
        gate_unsupported(
            ctx,
            d,
            DiagCode::W002,
            path,
            "depends_on_positions: Cartesian waits are not supported".into(),
        )?;
    }
    let id = g.ops.len();
    let e = g.add(EventKind::Op(id), n.dfs, path);
    g.ops.push(OpDraft {
        robot_ns,
        operation_id,
        kind: OpKind::Execute { operation_type },
        trajectory: Some(trajectory),
        hold_s,
        velocity_scale,
        timeout_override_ms: ctx.timeout_ms,
        bt: bt_ref(n),
        event: e,
    });
    Ok(Frontier::single(e))
}

fn lower_home(
    n: &NormNode,
    g: &mut EventGraph,
    ctx: &mut Ctx,
    d: &mut Diagnostics,
) -> Result<Frontier> {
    let path = n.path.0.as_str();
    let robot_ns = required(n, "robot_ns")?;
    validate_ns(&robot_ns, path)?;
    let trajectory = blackboard::trajectory(n, &ctx.doc.blackboard, d)?;
    if trajectory.is_none() {
        d.warn(
            DiagCode::W010,
            Some(path),
            format!("MoveHome for {robot_ns} has no planned trajectory: blind joint move home"),
        );
    }
    let count = ctx.home_ids.entry(robot_ns.clone()).or_insert(0);
    *count += 1;
    // Provisional; made unique against ExecuteTrajectory ids after the walk.
    let operation_id = if *count == 1 {
        "HOME".to_string()
    } else {
        format!("HOME_{count}")
    };
    let id = g.ops.len();
    let e = g.add(EventKind::Op(id), n.dfs, path);
    g.ops.push(OpDraft {
        robot_ns,
        operation_id,
        kind: OpKind::MoveHome,
        trajectory,
        hold_s: 0.0,
        velocity_scale: 1.0,
        timeout_override_ms: ctx.timeout_ms,
        bt: bt_ref(n),
        event: e,
    });
    Ok(Frontier::single(e))
}

/// Robots in BT order (first appearance in DFS).
pub fn robot_order(g: &EventGraph) -> Vec<String> {
    let mut seen = BTreeSet::new();
    let mut by_dfs: Vec<&OpDraft> = g.ops.iter().collect();
    by_dfs.sort_by_key(|o| o.bt.dfs);
    by_dfs
        .into_iter()
        .filter(|o| seen.insert(o.robot_ns.clone()))
        .map(|o| o.robot_ns.clone())
        .collect()
}
