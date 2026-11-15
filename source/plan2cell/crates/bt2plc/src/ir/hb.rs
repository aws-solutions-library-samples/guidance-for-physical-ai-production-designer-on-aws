//! Happens-before: deterministic topological order, op renumbering, ancestor bitsets and the
//! per-robot program order (DESIGN §3.2–3.3).

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::config::Config;
use crate::error::{DiagCode, Diagnostics, Error, Result};
use crate::ir::bitset::BitSet;
use crate::ir::lower::{EventGraph, EventKind, robot_order};
use crate::ir::{CellPlan, HomeSource, OpId, OpKind, Robot};

/// Ancestor sets over ops (`anc[b]` contains `a` iff `a` happens before `b`).
#[derive(Clone, Debug, Default)]
pub struct Hb {
    pub anc: Vec<BitSet>,
}

impl Hb {
    pub fn n_ops(&self) -> usize {
        self.anc.len()
    }

    pub fn before(&self, a: OpId, b: OpId) -> bool {
        self.anc[b].contains(a)
    }

    pub fn concurrent(&self, a: OpId, b: OpId) -> bool {
        a != b && !self.before(a, b) && !self.before(b, a)
    }
}

/// Kahn's algorithm with a (creation key, event id) min-heap; then renumber the ops so that
/// op id == position in the linear extension.
pub fn linearize(g: &mut EventGraph) -> Result<()> {
    let n = g.ev.len();
    let mut indeg: Vec<usize> = g.pred.iter().map(Vec::len).collect();
    let mut heap: BinaryHeap<Reverse<(usize, usize)>> = (0..n)
        .filter(|&e| indeg[e] == 0)
        .map(|e| Reverse((g.ev[e].key, e)))
        .collect();
    let mut order = Vec::with_capacity(n);
    while let Some(Reverse((_, e))) = heap.pop() {
        order.push(e);
        for &s in &g.succ[e] {
            indeg[s] -= 1;
            if indeg[s] == 0 {
                heap.push(Reverse((g.ev[s].key, s)));
            }
        }
    }
    if order.len() < n {
        return Err(cycle_error(g, &indeg));
    }
    // Renumber ops by their position in the order.
    let mut new_id = vec![usize::MAX; g.ops.len()];
    let mut next = 0;
    for &e in &order {
        if let EventKind::Op(o) = g.ev[e].kind {
            new_id[o] = next;
            next += 1;
        }
    }
    let mut ops: Vec<_> = std::mem::take(&mut g.ops).into_iter().enumerate().collect();
    ops.sort_by_key(|(old, _)| new_id[*old]);
    g.ops = ops.into_iter().map(|(_, o)| o).collect();
    for ev in &mut g.ev {
        if let EventKind::Op(o) = &mut ev.kind {
            *o = new_id[*o];
        }
    }
    g.order = order;
    Ok(())
}

fn cycle_error(g: &EventGraph, indeg: &[usize]) -> Error {
    // Walk predecessors inside the stuck set until a node repeats.
    let stuck: Vec<usize> = (0..g.ev.len()).filter(|&e| indeg[e] > 0).collect();
    let mut cur = stuck[0];
    let mut seen = vec![usize::MAX; g.ev.len()];
    let mut walk = Vec::new();
    while seen[cur] == usize::MAX {
        seen[cur] = walk.len();
        walk.push(cur);
        cur = *g.pred[cur]
            .iter()
            .find(|&&p| indeg[p] > 0)
            .expect("stuck events have stuck predecessors");
    }
    let cycle = &walk[seen[cur]..];
    let describe = |e: usize| match &g.ev[e].kind {
        EventKind::Op(o) => format!("{}.{}", g.ops[*o].robot_ns, g.ops[*o].operation_id),
        EventKind::Gate { robot, .. } => format!("WaitForRobot({robot})@{}", g.ev[e].path),
        EventKind::Barrier => format!("step boundary@{}", g.ev[e].path),
        EventKind::Delay { .. } => format!("Delay@{}", g.ev[e].path),
    };
    let text: Vec<String> = cycle.iter().rev().map(|&e| describe(e)).collect();
    Error::plan(
        DiagCode::E005,
        Some(&g.ev[cycle[0]].path),
        format!("dependency cycle: {} -> (back to start)", text.join(" -> ")),
    )
}

/// Ancestor bitsets over ops, in the renumbered op space.
pub fn closure(g: &EventGraph) -> Hb {
    let n_ops = g.ops.len();
    let mut anc_ev: Vec<BitSet> = vec![BitSet::new(n_ops); g.ev.len()];
    for &e in &g.order {
        let mut acc = BitSet::new(n_ops);
        for &p in &g.pred[e] {
            acc.union_with(&anc_ev[p]);
            if let Some(o) = g.op_of(p) {
                acc.insert(o);
            }
        }
        anc_ev[e] = acc;
    }
    Hb {
        anc: g.ops.iter().map(|o| anc_ev[o.event].clone()).collect(),
    }
}

/// Robots in BT order with their program (ops sorted by position); every robot's ops must be
/// totally ordered by HB (E004), i.e. a robot never appears in two Parallel branches.
pub fn robot_chains(
    g: &EventGraph,
    hb: &Hb,
    cfg: &Config,
    d: &mut Diagnostics,
) -> Result<Vec<Robot>> {
    let mut robots = Vec::new();
    for (i, ns) in robot_order(g).into_iter().enumerate() {
        let ops: Vec<OpId> = (0..g.ops.len())
            .filter(|&o| g.ops[o].robot_ns == ns)
            .collect();
        for w in ops.windows(2) {
            if !hb.before(w[0], w[1]) {
                let (a, b) = (&g.ops[w[0]], &g.ops[w[1]]);
                return Err(Error::plan(
                    DiagCode::E004,
                    Some(&b.bt.path),
                    format!(
                        "robot {ns} would run {} ({}) and {} ({}) concurrently: a robot cannot be in two Parallel branches",
                        a.operation_id, a.bt.path, b.operation_id, b.bt.path
                    ),
                ));
            }
        }
        robots.push(Robot {
            index: i,
            addr: cfg.addressing(&ns, i + 1)?,
            ns,
            ops,
            home_pose_rad: [0.0; 6],
            home_source: HomeSource::Fallback,
            idle: false,
        });
    }
    for rc in cfg.robots.iter().filter(|r| r.include_if_idle) {
        if robots.iter().any(|r| r.ns == rc.ns) {
            continue;
        }
        crate::ir::lower::validate_ns(&rc.ns, "[[robot]]")?;
        let i = robots.len();
        d.warn(
            DiagCode::W013,
            None,
            format!(
                "robot {} has no ops in the tree; included for bring-up and homing only",
                rc.ns
            ),
        );
        robots.push(Robot {
            index: i,
            addr: cfg.addressing(&rc.ns, i + 1)?,
            ns: rc.ns.clone(),
            ops: Vec::new(),
            home_pose_rad: [0.0; 6],
            home_source: HomeSource::Fallback,
            idle: true,
        });
    }
    check_addressing(&robots, cfg)?;
    Ok(robots)
}

/// Robot names, tags, stations, device names and IPs end up in file names, IEC identifiers
/// and the PROFINET configuration: they must be valid and unique.
fn check_addressing(robots: &[Robot], cfg: &Config) -> Result<()> {
    let bad = |m: String| Err(Error::Config(m));
    let n = &cfg.network;
    for (i, a) in robots.iter().enumerate() {
        let ad = &a.addr;
        if ad.tag.is_empty()
            || ad.tag.len() > 16
            || !ad
                .tag
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b == b'_')
        {
            return bad(format!(
                "robot {}: tag '{}' must match [A-Za-z0-9_]{{1,16}} (it is part of PLC identifiers)",
                a.ns, ad.tag
            ));
        }
        let dn = &ad.device_name;
        if let Some(p) = crate::hardware::device_name_problem(dn) {
            return bad(format!("robot {}: device_name '{dn}' {p}", a.ns));
        }
        if ad.ip == n.head_ip
            || u32::from(ad.ip) & u32::from(n.netmask)
                != u32::from(n.head_ip) & u32::from(n.netmask)
        {
            return bad(format!(
                "robot {}: IP {} must be in the head PLC subnet and differ from the head IP",
                a.ns, ad.ip
            ));
        }
        if ad.prod_folge == 0 || ad.prod_folge == cfg.cell.home_folge {
            return bad(format!(
                "robot {}: production Folge {} must be > 0 and differ from the home Folge",
                a.ns, ad.prod_folge
            ));
        }
        for b in &robots[..i] {
            let bd = &b.addr;
            let clash = if a.ns.eq_ignore_ascii_case(&b.ns) {
                Some("ns (case-insensitive)")
            } else if ad.tag.eq_ignore_ascii_case(&bd.tag) {
                Some("tag")
            } else if ad.device_name.eq_ignore_ascii_case(&bd.device_name) {
                Some("device_name")
            } else if ad.station == bd.station {
                Some("station name")
            } else if ad.ip == bd.ip {
                Some("IP address")
            } else {
                None
            };
            if let Some(what) = clash {
                return bad(format!("robots {} and {} have the same {what}", b.ns, a.ns));
            }
        }
    }
    // The PROFINET rules (NameOfStation, subnet host addresses, gateway): the same check as
    // `plan2cell hardware` and the front ends.
    let devices: Vec<crate::hardware::DeviceAddr<'_>> = robots
        .iter()
        .map(|r| crate::hardware::DeviceAddr {
            ns: &r.ns,
            station: &r.addr.station,
            ip: r.addr.ip,
        })
        .collect();
    if let Some(i) = crate::hardware::check(n, &devices).into_iter().next() {
        return bad(format!("hardware: {}: {}", i.field, i.message));
    }
    Ok(())
}

const FALLBACK_HOME: [f64; 6] = [0.0, -1.57, 2.2, 0.0, -0.8, 0.0];

/// Resolve home poses and check program start/end/chaining against them (W009, W015, W016).
pub fn home_poses(plan: &mut CellPlan, cfg: &Config, d: &mut Diagnostics) {
    let rs = &cfg.robot_sim;
    for ri in 0..plan.robots.len() {
        let ns = plan.robots[ri].ns.clone();
        let ops: Vec<OpId> = plan.robots[ri].ops.clone();
        let (pose, src) = if let Some(deg) = cfg.robot_cfg(&ns).and_then(|r| r.home_pose_deg) {
            (deg.map(f64::to_radians), HomeSource::Config)
        } else if let Some(t) = ops
            .last()
            .map(|&o| &plan.ops[o])
            .filter(|o| o.kind == OpKind::MoveHome)
            .and_then(|o| o.trajectory.as_ref())
        {
            (*t.last(), HomeSource::FinalMoveHome)
        } else if let Some(t) = ops.iter().find_map(|&o| plan.ops[o].trajectory.as_ref()) {
            (*t.first(), HomeSource::FirstWaypoint)
        } else {
            d.warn(
                DiagCode::W015,
                None,
                format!(
                    "robot {ns}: no home pose in config or trajectories; using the fallback pose"
                ),
            );
            (FALLBACK_HOME, HomeSource::Fallback)
        };
        plan.robots[ri].home_pose_rad = pose;
        plan.robots[ri].home_source = src;
        if ops.is_empty() {
            continue;
        }
        if let Some(t) = plan.ops[ops[0]].trajectory.as_ref() {
            let dev = max_dev_deg(t.first(), &pose);
            if dev > rs.start_tol_deg {
                d.warn(
                    DiagCode::W016,
                    Some(&plan.ops[ops[0]].bt.path),
                    format!("robot {ns}: first waypoint is {dev:.2} deg from the home pose"),
                );
            }
        }
        let last = &plan.ops[*ops.last().unwrap()];
        let ends_home = match &last.trajectory {
            None => last.kind == OpKind::MoveHome,
            Some(t) => max_dev_deg(t.last(), &pose) <= rs.at_home_tol_deg,
        };
        if !ends_home {
            d.warn(
                DiagCode::W016,
                Some(&last.bt.path),
                format!("robot {ns}: program does not end at the home pose; every auto-cycle starts with a blind home move"),
            );
        }
        for w in ops.windows(2) {
            let (a, b) = (&plan.ops[w[0]], &plan.ops[w[1]]);
            if let (Some(ta), Some(tb)) = (&a.trajectory, &b.trajectory) {
                let dev = max_dev_deg(ta.last(), tb.first());
                if dev > rs.chain_tol_deg {
                    d.warn(
                        DiagCode::W009,
                        Some(&b.bt.path),
                        format!(
                            "robot {ns}: {} starts {dev:.2} deg away from where {} ends",
                            b.operation_id, a.operation_id
                        ),
                    );
                }
            }
        }
    }
}

pub fn max_dev_deg(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).to_degrees().abs())
        .fold(0.0, f64::max)
}
