//! Co-simulation of the GENERATED ST with trust-runtime: one harness for the head PLC and one
//! per robot simulator, exchanging the PROFINET process images (VASS 32 bytes + PROFIsafe
//! 8 bytes each way) between scans, each PLC on its own task interval in virtual time. The
//! intervals are the ones the code is generated for (`codesys.main_task_ms`,
//! `codesys.device_main_task_ms`; the robot player integrates `GVL_Program.R_TASK_S`, which is
//! derived from the latter, so the two must never differ).
//!
//! The protocol trace is decoded black-box from the exchanged frames (step, REQUEST, GRANT,
//! running/finished/error bits, joint angles), never from ST internals, and checked against
//! the plan: zone mutual exclusion (per scan and per op interval), every cross-robot
//! dependency (planned grant policy), program order, BT parallelism, cycle windows and
//! duration, no motion without a grant, and cycle completion.
//!
//! A robot can also be a Rust model instead of the generated device ST ([`RobotModel`],
//! [`run_with`]): bt2krl plugs its KRL instruction executor in there, so the same head PLC is
//! checked against the robot programs it will really talk to.
//!
//! Op runs are numbered by the CELL production cycle (a robot's job leaving step 0 belongs to
//! the cycle the cell is running; home jobs never leave step 0). A cycle the cell aborts is
//! discarded: its runs move to [`Report::aborted_runs`] and the next cycle reuses its number.

use std::fmt;

use trust_runtime::harness::TestHarness;
use trust_runtime::value::{Duration, Value};

use crate::backend::model::{PlcProject, ProjectRole};
use crate::backend::trust_render::render;
use crate::config::{Config, GrantPolicy};
use crate::error::{Error, Result};
use crate::ir::{CellPlan, OpId, PERMIT_BIT};

/// Cross-robot ops that are concurrent in the BT and whose nominal ASAP intervals overlap by
/// more than this must overlap in every simulated cycle (handshake latency stays far below).
const PARALLEL_SLACK_MS: u64 = 2_000;
/// Cycle duration bound: `makespan * 1.25 + this`.
const CYCLE_SLACK_MS: u64 = 10_000;
/// Joint angles A1..A6 in the robot -> PLC frame (FB_JointAngleEncoder: 6 x INT, big-endian).
const POSE_BYTE: usize = 16;
/// Violations of one kind listed in full by [`Report::verify`].
const MAX_LISTED: usize = 5;

// ───────────────────────────── options ─────────────────────────────

#[derive(Clone, Debug)]
pub struct SimOptions {
    /// Production cycles to complete.
    pub cycles: u32,
    /// Give up after this much virtual time (0 = derived from the plan).
    pub max_time_s: f64,
    /// Inject a robot error: (robot index, time ms) → `PLC_PRG.bTest_InjectError` on that robot.
    pub inject_error: Option<(usize, u64)>,
    /// After an injected error: send reset at this time, then confirm recovery.
    pub reset_at_ms: Option<u64>,
    /// Fault injection for the motion check: from (robot index, time ms) on, that robot's
    /// simulator receives its own REQUEST as GRANT (the head's grant is bypassed on the wire;
    /// the trace still sees the head's real GRANT).
    pub bypass_grant: Option<(usize, u64)>,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self {
            cycles: 1,
            max_time_s: 0.0,
            inject_error: None,
            reset_at_ms: None,
            bypass_grant: None,
        }
    }
}

// ───────────────────────────── robot models ─────────────────────────────

/// A robot simulated in Rust instead of by the generated device ST.
///
/// One call is one device scan at the device task interval: consume the PLC -> robot frames
/// (VASS 32 bytes, PROFIsafe 8 bytes) and return the robot -> PLC frames. The model must keep
/// the protocol of `FB_VASS_DeviceSimulator` (DESIGN section 4): status bits, job handshake,
/// step, REQUEST, and the joint angles in bytes 16-27 (6 x INT16 big-endian, 0.02 deg/count)
/// that the motion check reads.
pub trait RobotModel {
    fn scan(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]);

    /// Equivalent of the device `PLC_PRG.bTest_InjectError` debug input.
    fn inject_error(&mut self, _on: bool) {}
}

/// Robot index (0-based, plan order) and its model.
pub type RobotModels = Vec<(usize, Box<dyn RobotModel>)>;

#[allow(clippy::large_enum_variant)] // one Dev per robot; boxing the harness buys nothing
enum Dev {
    /// The generated device ST in trust-runtime.
    St(TestHarness),
    /// A Rust model plus its last robot -> PLC frames.
    Model {
        m: Box<dyn RobotModel>,
        inp: [u8; 32],
        safe_in: [u8; 8],
    },
}

struct Device {
    ri: usize,
    tag: String,
    dev: Dev,
}

// ───────────────────────────── trace ─────────────────────────────

#[derive(Clone, Debug, PartialEq)]
pub enum EvKind {
    Step(u16),
    Request(u32),
    Grant(u32),
    Running(bool),
    Finished(bool),
    RobotError(bool),
    CellState(String),
    CellError(String),
}

#[derive(Clone, Debug)]
pub struct Event {
    pub t_ms: u64,
    /// Robot index (0-based); `usize::MAX` for cell events.
    pub robot: usize,
    pub kind: EvKind,
}

impl fmt::Display for Event {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let who = if self.robot == usize::MAX {
            "cell".to_string()
        } else {
            format!("R{}", self.robot + 1)
        };
        let what = match &self.kind {
            EvKind::Step(s) => format!("step {s}"),
            EvKind::Request(m) => format!("REQUEST 16#{m:08X}"),
            EvKind::Grant(m) => format!("GRANT   16#{m:08X}"),
            EvKind::Running(b) => format!("ProgRunning {b}"),
            EvKind::Finished(b) => format!("ProgFinished {b}"),
            EvKind::RobotError(b) => format!("Error {b}"),
            EvKind::CellState(s) => format!("state {s}"),
            EvKind::CellError(s) => format!("error {s}"),
        };
        write!(f, "{:>9.3} s  {who:<4} {what}", self.t_ms as f64 / 1000.0)
    }
}

/// One executed op occurrence as seen on the wire.
#[derive(Clone, Debug)]
pub struct OpRun {
    /// 1-based cell production cycle (aborted cycles are not counted).
    pub cycle: u32,
    pub robot: usize,
    pub index: u16,
    /// Grant time (masked ops) or step change (unmasked ops).
    pub start_ms: u64,
    /// REQUEST drop (masked ops) or next step / finish (unmasked ops); the abort time for an
    /// op cut short by an abort (only in [`Report::aborted_runs`]).
    pub end_ms: u64,
}

/// A change of the cell error output (`PLC_PRG.fbCell.eError`, incl. back to ERR_NONE).
#[derive(Clone, Debug)]
pub struct CellFault {
    pub t_ms: u64,
    pub error: String,
    /// `PLC_PRG.fbCell.nErrorRobot`: 1-based robot, 0 = cell, -1 = unreadable.
    pub robot: i64,
    /// `PLC_PRG.fbCell.nErrorOp`: op index of the fault, 0 = none.
    pub op: i64,
}

#[derive(Clone, Debug, Default)]
pub struct Report {
    pub events: Vec<Event>,
    /// Runs of the cycles that were not aborted.
    pub runs: Vec<OpRun>,
    /// Runs of aborted cycles (tagged with the discarded cycle number), incl. the ops that
    /// were running at the abort.
    pub aborted_runs: Vec<OpRun>,
    pub cycle_end_ms: Vec<u64>,
    pub cycle_start_ms: Vec<u64>,
    /// (start, abort) of every cycle the cell aborted.
    pub aborted_cycles: Vec<(u64, u64)>,
    pub cell_errors: Vec<CellFault>,
    pub end_ms: u64,
    pub runtime_errors: Vec<String>,
    /// Per scan: zone bit granted to more than one robot.
    pub exclusion_violations: Vec<String>,
    /// Joint motion while the robot's current op has zones and is not granted (or released).
    pub motion_violations: Vec<String>,
    /// Inconsistent trace, e.g. an op still running at the cycle end.
    pub trace_violations: Vec<String>,
}

fn capped(errs: &mut Vec<String>, what: &str, list: &[String]) {
    errs.extend(list.iter().take(MAX_LISTED).cloned());
    if list.len() > MAX_LISTED {
        errs.push(format!(
            "... {} more {what} violations",
            list.len() - MAX_LISTED
        ));
    }
}

fn secs(ms: u64) -> f64 {
    ms as f64 / 1000.0
}

/// Plan op of robot `r` at 1-based program index `k`.
fn op_at(plan: &CellPlan, r: usize, k: u16) -> Option<OpId> {
    plan.robots
        .get(r)?
        .ops
        .get(usize::from(k).checked_sub(1)?)
        .copied()
}

fn op_name(plan: &CellPlan, r: usize, k: u16) -> String {
    op_at(plan, r, k).map_or_else(|| format!("R{} op {k}", r + 1), |o| plan.op_ref(o))
}

/// Concurrent cross-robot op pairs whose nominal ASAP intervals overlap by more than the slack.
fn parallel_pairs(plan: &CellPlan) -> Vec<(OpId, OpId, u64)> {
    let mut v = Vec::new();
    if plan.hb.n_ops() != plan.ops.len() {
        return v;
    }
    for (i, a) in plan.ops.iter().enumerate() {
        for b in &plan.ops[i + 1..] {
            if a.robot != b.robot && plan.hb.concurrent(a.id, b.id) {
                let ov = a
                    .asap_end_ms
                    .min(b.asap_end_ms)
                    .saturating_sub(a.asap_start_ms.max(b.asap_start_ms));
                if ov > PARALLEL_SLACK_MS {
                    v.push((a.id, b.id, ov));
                }
            }
        }
    }
    v
}

impl Report {
    pub fn summary(&self) -> String {
        let cyc: Vec<String> = self
            .cycle_start_ms
            .iter()
            .zip(&self.cycle_end_ms)
            .map(|(s, e)| format!("{:.3} s", secs(e - s)))
            .collect();
        let aborted = if self.aborted_cycles.is_empty() {
            String::new()
        } else {
            format!(", {} aborted", self.aborted_cycles.len())
        };
        format!(
            "co-simulation: {} cycle(s) completed [{}]{aborted}, {} op runs, {} events, {:.3} s virtual",
            self.cycle_end_ms.len(),
            cyc.join(", "),
            self.runs.len(),
            self.events.len(),
            secs(self.end_ms)
        )
    }

    /// The run of plan op `o` in (completed or open) cycle `cycle`.
    pub fn run_of(&self, plan: &CellPlan, cycle: u32, o: OpId) -> Option<&OpRun> {
        let op = plan.ops.get(o)?;
        self.runs
            .iter()
            .find(|r| r.cycle == cycle && r.robot == op.robot && r.index == op.index)
    }

    /// Check the trace against the plan. Per-cycle checks cover the completed cycles only.
    pub fn verify(&self, plan: &CellPlan) -> std::result::Result<(), String> {
        let mut errs = Vec::new();
        errs.extend(self.runtime_errors.iter().cloned());
        capped(&mut errs, "zone exclusion", &self.exclusion_violations);
        capped(&mut errs, "motion", &self.motion_violations);
        capped(&mut errs, "trace", &self.trace_violations);
        // The ASAP schedule (order, parallelism, makespan) is the planned ticket order; FCFS
        // only guarantees mutual exclusion and program order.
        let planned = plan.grant_policy == GrantPolicy::Planned;
        let parallel = if planned {
            parallel_pairs(plan)
        } else {
            Vec::new()
        };
        let bound = plan.makespan_ms + plan.makespan_ms / 4 + CYCLE_SLACK_MS;
        for (ci, (&cs, &ce)) in self
            .cycle_start_ms
            .iter()
            .zip(&self.cycle_end_ms)
            .enumerate()
        {
            let cycle = ci as u32 + 1;
            let run = |o: OpId| self.run_of(plan, cycle, o);
            for op in &plan.ops {
                if run(op.id).is_none() {
                    errs.push(format!("cycle {cycle}: {} never ran", plan.op_ref(op.id)));
                }
            }
            for r in self.runs.iter().filter(|r| r.cycle == cycle) {
                if r.start_ms < cs || r.end_ms > ce {
                    errs.push(format!(
                        "cycle {cycle}: {} ran [{:.3}, {:.3}] s outside the cycle [{:.3}, {:.3}] s",
                        op_name(plan, r.robot, r.index),
                        secs(r.start_ms),
                        secs(r.end_ms),
                        secs(cs),
                        secs(ce)
                    ));
                }
            }
            for r in &plan.robots {
                for w in r.ops.windows(2) {
                    if let (Some(a), Some(b)) = (run(w[0]), run(w[1]))
                        && b.start_ms < a.end_ms
                    {
                        errs.push(format!(
                            "cycle {cycle}: {} started before {} ended",
                            plan.op_ref(w[1]),
                            plan.op_ref(w[0])
                        ));
                    }
                }
            }
            if planned {
                for e in &plan.cross_edges {
                    if let (Some(u), Some(v)) = (run(e.from), run(e.to))
                        && v.start_ms < u.end_ms
                    {
                        errs.push(format!(
                            "cycle {cycle}: dependency violated: {} started at {:.3} s before {} ended at {:.3} s",
                            plan.op_ref(e.to),
                            secs(v.start_ms),
                            plan.op_ref(e.from),
                            secs(u.end_ms)
                        ));
                    }
                }
            }
            for &(a, b, nominal) in &parallel {
                if let (Some(x), Some(y)) = (run(a), run(b))
                    && x.end_ms.min(y.end_ms) <= x.start_ms.max(y.start_ms)
                {
                    errs.push(format!(
                        "cycle {cycle}: {} [{:.3}, {:.3}] s and {} [{:.3}, {:.3}] s are parallel in the BT \
                         (nominal overlap {:.3} s) but did not overlap",
                        plan.op_ref(a),
                        secs(x.start_ms),
                        secs(x.end_ms),
                        plan.op_ref(b),
                        secs(y.start_ms),
                        secs(y.end_ms),
                        secs(nominal)
                    ));
                }
            }
            if planned && ce - cs > bound {
                errs.push(format!(
                    "cycle {cycle}: took {:.3} s, bound {:.3} s (planned makespan {:.3} s)",
                    secs(ce - cs),
                    secs(bound),
                    secs(plan.makespan_ms)
                ));
            }
        }
        capped(&mut errs, "zone interval", &self.interval_exclusion(plan));
        if errs.is_empty() {
            Ok(())
        } else {
            Err(errs.join("\n"))
        }
    }

    /// Runs of different robots whose ops share a zone must not overlap in time (both policies).
    fn interval_exclusion(&self, plan: &CellPlan) -> Vec<String> {
        let zones = |r: &OpRun| {
            op_at(plan, r.robot, r.index).map_or(0, |o| plan.ops[o].zone_mask & !PERMIT_BIT)
        };
        let mut errs = Vec::new();
        for (i, x) in self.runs.iter().enumerate() {
            let mx = zones(x);
            if mx == 0 {
                continue;
            }
            for y in &self.runs[i + 1..] {
                let shared = mx & zones(y);
                if y.robot != x.robot
                    && shared != 0
                    && x.start_ms < y.end_ms
                    && y.start_ms < x.end_ms
                {
                    let z = shared.trailing_zeros() as usize;
                    errs.push(format!(
                        "zone {} held by {} [{:.3}, {:.3}] s and {} [{:.3}, {:.3}] s at the same time",
                        plan.zones.get(z).map_or_else(|| format!("bit {z}"), |z| z.name.clone()),
                        op_name(plan, x.robot, x.index),
                        secs(x.start_ms),
                        secs(x.end_ms),
                        op_name(plan, y.robot, y.index),
                        secs(y.start_ms),
                        secs(y.end_ms)
                    ));
                }
            }
        }
        errs
    }

    /// Simulated time two ops overlapped in a cycle (ms), for parallelism checks.
    pub fn overlap_ms(&self, plan: &CellPlan, cycle: u32, a: usize, b: usize) -> Option<u64> {
        let (x, y) = (self.run_of(plan, cycle, a)?, self.run_of(plan, cycle, b)?);
        let s = x.start_ms.max(y.start_ms);
        let e = x.end_ms.min(y.end_ms);
        Some(e.saturating_sub(s))
    }

    /// The cell aborted: discard the open cycle (its start, its runs) and every op in progress.
    fn abort_cycle(&mut self, tr: &mut [RobotTrace], t: u64) {
        if self.cycle_start_ms.len() > self.cycle_end_ms.len() {
            let c = self.cycle_start_ms.len() as u32;
            if let Some(s) = self.cycle_start_ms.pop() {
                self.aborted_cycles.push((s, t));
            }
            let (gone, keep): (Vec<OpRun>, Vec<OpRun>) = std::mem::take(&mut self.runs)
                .into_iter()
                .partition(|r| r.cycle == c);
            self.runs = keep;
            self.aborted_runs.extend(gone);
        }
        for (ri, x) in tr.iter_mut().enumerate() {
            if let Some(o) = x.open.take() {
                self.aborted_runs.push(OpRun {
                    cycle: o.cycle,
                    robot: ri,
                    index: o.index,
                    start_ms: o.start_ms,
                    end_ms: t,
                });
            }
            x.cycle = None;
            x.must_hold = false;
        }
    }
}

// ───────────────────────────── frames ─────────────────────────────

fn harness(p: &PlcProject) -> Result<TestHarness> {
    let files = render(p);
    let texts: Vec<&str> = files.iter().map(|f| f.text.as_str()).collect();
    TestHarness::from_sources(&texts)
        .map_err(|e| Error::Sim(format!("{}: compile failed: {e}", p.key)))
}

fn get(h: &TestHarness, name: &str) -> Result<Value> {
    h.runtime()
        .storage()
        .get_global(name)
        .cloned()
        .ok_or_else(|| Error::Sim(format!("global {name} not found")))
}

fn set(h: &mut TestHarness, name: &str, v: Value) {
    h.runtime_mut().storage_mut().set_global(name, v);
}

fn bytes(v: &Value) -> Vec<u8> {
    match v {
        Value::Array(a) => a
            .elements()
            .iter()
            .map(|e| match e {
                Value::Byte(b) => *b,
                Value::USInt(b) => *b,
                _ => 0,
            })
            .collect(),
        Value::Byte(b) => vec![*b],
        _ => Vec::new(),
    }
}

/// Overwrite the DWORD (little-endian) at byte `i` of a BYTE array value.
fn patch_dword(v: &mut Value, i: usize, d: u32) {
    if let Value::Array(a) = v
        && let Some(el) = a.elements_mut().get_mut(i..i + 4)
    {
        for (k, e) in el.iter_mut().enumerate() {
            *e = Value::Byte((d >> (8 * k)) as u8);
        }
    }
}

/// Overwrite the elements of a BYTE array value.
fn patch_bytes(v: &mut Value, b: &[u8]) {
    if let Value::Array(a) = v {
        for (e, &x) in a.elements_mut().iter_mut().zip(b) {
            *e = Value::Byte(x);
        }
    }
}

fn to_frame(b: &[u8]) -> [u8; 32] {
    let mut f = [0u8; 32];
    f.copy_from_slice(&b[..32]);
    f
}

fn word(b: &[u8], i: usize) -> u16 {
    u16::from(b[i]) | (u16::from(b[i + 1]) << 8)
}

fn dword(b: &[u8], i: usize) -> u32 {
    u32::from(b[i])
        | (u32::from(b[i + 1]) << 8)
        | (u32::from(b[i + 2]) << 16)
        | (u32::from(b[i + 3]) << 24)
}

fn enum_text(v: Option<Value>) -> String {
    match v {
        Some(Value::Enum(e)) => e.variant_name().to_string(),
        Some(other) => format!("{other:?}"),
        None => "?".into(),
    }
}

fn int(v: Option<Value>) -> i64 {
    match v {
        Some(Value::SInt(i)) => i64::from(i),
        Some(Value::Int(i)) => i64::from(i),
        Some(Value::DInt(i)) => i64::from(i),
        Some(Value::LInt(i)) => i,
        Some(Value::USInt(i) | Value::Byte(i)) => i64::from(i),
        Some(Value::UInt(i) | Value::Word(i)) => i64::from(i),
        Some(Value::UDInt(i) | Value::DWord(i)) => i64::from(i),
        _ => -1,
    }
}

/// One robot's VASS frames (robot -> PLC `inp`, PLC -> robot `out`) as seen by the head.
#[derive(Clone, Copy, Default, PartialEq)]
struct Frame {
    step: u16,
    req: u32,
    grant: u32,
    running: bool,
    finished: bool,
    error: bool,
    /// Raw joint counts A1..A6 (0.02 deg per count).
    pose: [i16; 6],
}

/// Profile `vw-ks24`: the VW signal map the trace decodes instead of the simulator telegram.
#[derive(Clone)]
struct VwMap {
    /// Robot index -> zone index -> VW interlock (1..16, 0 = not a member).
    il: Vec<Vec<u8>>,
    n_zones: usize,
}

fn bit_at(b: &[u8], idx: u16) -> bool {
    let i = usize::from(idx) - 1;
    b.get(i / 8).is_some_and(|x| x & (1 << (i % 8)) != 0)
}

impl Frame {
    /// Profile `vw-ks24`: step from `P_PktNr_SPS`, REQUEST synthesised as
    /// `A(80+n) OR NOT A(40+n)` ("requesting or occupying"), GRANT from `E(80+n)`, running from
    /// the echoed Folge number with PF0 and Folge end.
    fn decode_vw(inp: &[u8], out: &[u8], m: &VwMap, ri: usize) -> Self {
        let mut pose = [0i16; 6];
        for (j, q) in pose.iter_mut().enumerate() {
            let i = POSE_BYTE + 2 * j;
            *q = i16::from_be_bytes([inp[i], inp[i + 1]]);
        }
        let sb = (usize::from(crate::config::vw_ks24::STEP_OUT) - 1) / 8;
        let (mut req, mut grant) = (0u32, 0u32);
        for z in 0..m.n_zones {
            let n = m.il.get(ri).and_then(|r| r.get(z)).copied().unwrap_or(0);
            if !(1..=16).contains(&n) {
                continue;
            }
            let (rel, rq) = (
                crate::config::vw_ks24::RELEASE_OUT + u16::from(n) - 1,
                crate::config::vw_ks24::REQUEST_OUT + u16::from(n) - 1,
            );
            // The "occupied" term counts only while the robot is in a job (an all-zero frame
            // would otherwise read as "inside every interlock"); same rule as FB_VassRobot.
            if bit_at(inp, rq) || (inp[0] != 0 && !bit_at(inp, rel)) {
                req |= 1 << z;
            }
            if bit_at(out, rq) {
                grant |= 1 << z;
            }
        }
        let echo = u16::from(inp[0]);
        let at_home = bit_at(inp, crate::config::vw_ks24::PF0_OUT);
        let fin = bit_at(inp, crate::config::vw_ks24::FOLGE_END_OUT);
        let ecode = inp[(usize::from(crate::config::vw_ks24::ERROR_OUT) - 1) / 8];
        Self {
            step: word(inp, sb),
            req,
            grant,
            running: echo != 0 && !at_home && !fin,
            finished: fin,
            error: ecode != 0,
            pose,
        }
    }

    fn decode(inp: &[u8], out: &[u8], sb: usize, rb: usize, gb: usize) -> Self {
        let mut pose = [0i16; 6];
        for (j, q) in pose.iter_mut().enumerate() {
            let i = POSE_BYTE + 2 * j;
            *q = i16::from_be_bytes([inp[i], inp[i + 1]]);
        }
        Self {
            step: word(inp, sb),
            req: dword(inp, rb),
            grant: dword(out, gb),
            running: inp[0] & 0x02 != 0,
            finished: inp[1] & 0x04 != 0,
            error: inp[0] & 0x40 != 0,
            pose,
        }
    }
}

#[derive(Clone, Copy)]
struct OpenOp {
    index: u16,
    start_ms: u64,
    cycle: u32,
}

#[derive(Default, Clone)]
struct RobotTrace {
    /// Frame of the previous head scan.
    prev: Frame,
    /// The op currently running.
    open: Option<OpenOp>,
    /// Cell cycle of the production job in progress (None: no job, a home job, or discarded).
    cycle: Option<u32>,
    /// After the previous scan: production op with zones that is not running (waiting for its
    /// GRANT, or released) - the robot must stand still.
    must_hold: bool,
}

impl RobotTrace {
    /// Advance the trace of robot `ri` by one head scan. `cell_cycle` is the cell cycle in
    /// progress (None outside START_JOBS .. CYCLE_DONE).
    fn scan(
        &mut self,
        f: Frame,
        t: u64,
        ri: usize,
        plan: &CellPlan,
        cell_cycle: Option<u32>,
        rep: &mut Report,
    ) {
        let p = self.prev;
        let mut ev = |k: EvKind| {
            rep.events.push(Event {
                t_ms: t,
                robot: ri,
                kind: k,
            })
        };
        if f.running != p.running {
            ev(EvKind::Running(f.running));
        }
        if f.step != p.step {
            ev(EvKind::Step(f.step));
        }
        if f.req != p.req {
            ev(EvKind::Request(f.req));
        }
        if f.grant != p.grant {
            ev(EvKind::Grant(f.grant));
        }
        if f.finished != p.finished {
            ev(EvKind::Finished(f.finished));
        }
        if f.error != p.error {
            ev(EvKind::RobotError(f.error));
        }
        let mask_of = |k: u16| op_at(plan, ri, k).map_or(0, |o| plan.ops[o].zone_mask);
        // Motion while the robot had to stand still. Same op only: the first frame of a new op
        // may still carry the last motion of the previous one (zero hold falls through).
        if self.must_hold && f.step == p.step && f.pose != p.pose {
            rep.motion_violations.push(format!(
                "{:.3} s: R{} moved without a GRANT during {} (step {}, REQUEST 16#{:08X}, GRANT 16#{:08X})",
                secs(t),
                ri + 1,
                op_name(plan, ri, f.step),
                f.step,
                f.req,
                f.grant
            ));
        }
        // close the open op
        if let Some(o) = self.open {
            let m = mask_of(o.index);
            let closed = if m != 0 {
                p.req & m == m && f.req & m == 0
            } else {
                f.step != o.index || (f.finished && !f.running)
            };
            if closed || f.step != o.index {
                rep.runs.push(OpRun {
                    cycle: o.cycle,
                    robot: ri,
                    index: o.index,
                    start_ms: o.start_ms,
                    end_ms: t,
                });
                self.open = None;
            }
        }
        // a production job leaves step 0 in the cell's cycle; back at 0 it is over
        if f.step != p.step {
            if f.step == 0 {
                self.cycle = None;
            } else if p.step == 0 {
                self.cycle = cell_cycle;
            }
        }
        // open the current op
        if let (None, Some(c)) = (self.open, self.cycle)
            && f.step > 0
            && f.running
        {
            let m = mask_of(f.step);
            let already = rep
                .runs
                .iter()
                .any(|r| r.robot == ri && r.index == f.step && r.cycle == c);
            let started = m == 0 || (f.grant & m == m && f.req & m == m);
            if started && !already {
                self.open = Some(OpenOp {
                    index: f.step,
                    start_ms: t,
                    cycle: c,
                });
            }
        }
        self.must_hold = self.cycle.is_some()
            && f.running
            && f.step > 0
            && mask_of(f.step) != 0
            && self.open.is_none();
        self.prev = f;
    }
}

// ───────────────────────────── co-simulation ─────────────────────────────

/// Co-simulate the generated head PLC with the generated device ST of every robot.
pub fn run(
    plan: &CellPlan,
    cfg: &Config,
    projects: &[PlcProject],
    opts: &SimOptions,
) -> Result<Report> {
    run_with(plan, cfg, projects, opts, Vec::new())
}

/// Co-simulate with a Rust [`RobotModel`] for the listed robots; the others need their
/// generated device ST (`robot_sim.enabled`).
pub fn run_with(
    plan: &CellPlan,
    cfg: &Config,
    projects: &[PlcProject],
    opts: &SimOptions,
    models: RobotModels,
) -> Result<Report> {
    let head_p = projects
        .iter()
        .find(|p| p.role == ProjectRole::HeadPlc)
        .ok_or_else(|| Error::Sim("no head project".into()))?;
    let mut head = harness(head_p)?;
    let tag = |ri: usize| -> Result<String> {
        plan.robots
            .get(ri)
            .map(|r| r.addr.tag.clone())
            .ok_or_else(|| Error::Sim(format!("robot {ri} is not in the plan")))
    };
    let mut devs: Vec<Device> = Vec::new();
    for (ri, m) in models {
        if devs.iter().any(|d| d.ri == ri) {
            return Err(Error::Sim(format!("two models for robot {ri}")));
        }
        devs.push(Device {
            ri,
            tag: tag(ri)?,
            dev: Dev::Model {
                m,
                inp: [0; 32],
                safe_in: [0; 8],
            },
        });
    }
    for p in projects.iter().filter(|p| p.role == ProjectRole::RobotSim) {
        let ri = p
            .robot
            .ok_or_else(|| Error::Sim(format!("{}: robot simulator without a robot", p.key)))?;
        if devs.iter().any(|d| d.ri == ri) {
            continue; // a model stands in for this robot
        }
        devs.push(Device {
            ri,
            tag: tag(ri)?,
            dev: Dev::St(harness(p)?),
        });
    }
    devs.sort_by_key(|d| d.ri);
    if devs.len() != plan.robots.len() {
        return Err(Error::Sim(
            "co-simulation needs a robot simulator or a model for every robot (robot_sim.enabled)"
                .into(),
        ));
    }
    // Exactly the task intervals the code is generated for (GVL_Program.R_TASK_S).
    let hdt = u64::from(cfg.codesys.main_task_ms).max(1);
    let ddt = u64::from(cfg.codesys.device_main_task_ms).max(1);
    let max_ms = if opts.max_time_s > 0.0 {
        (opts.max_time_s * 1000.0) as u64
    } else {
        // Bring-up + homing + cycles at a generous factor of the planned makespan.
        30_000 + u64::from(opts.cycles) * (plan.makespan_ms * 3 + 30_000)
    };
    let p = &cfg.protocol;
    let (sb, rb, gb) = (
        p.step_byte as usize,
        p.request_byte as usize,
        p.grant_byte as usize,
    );
    if !p.profile.is_vw()
        && [sb + 2, rb + 4, gb + 4, POSE_BYTE + 12]
            .iter()
            .any(|&end| end > 32)
    {
        return Err(Error::Sim(
            "protocol byte offsets exceed the 32-byte VASS frame".into(),
        ));
    }
    let vwmap = p.profile.is_vw().then(|| VwMap {
        il: crate::ir::interlock_map(plan, cfg),
        n_zones: plan.zones.len(),
    });
    let mut rep = Report::default();
    let mut tr: Vec<RobotTrace> = vec![RobotTrace::default(); plan.robots.len()];
    let mut last_state = String::new();
    let mut last_error = String::new();
    let mut t = 0u64;
    let step = gcd(hdt, ddt);
    let mut injected = false;
    let mut reset_sent = false;
    while t <= max_ms {
        t += step;
        // ── robot simulators ──
        if t.is_multiple_of(ddt) {
            for d in devs.iter_mut() {
                let (ri, tag) = (d.ri, d.tag.clone());
                let mut cmd = get(&head, &format!("GVL_IO.aVassOutputData_{tag}"))?;
                let own_req = match &d.dev {
                    Dev::St(h) => bytes(&get(h, "GVL_IO.aVassInputData")?),
                    Dev::Model { inp, .. } => inp.to_vec(),
                };
                if opts
                    .bypass_grant
                    .is_some_and(|(br, at)| br == ri && t >= at)
                    && own_req.len() >= rb + 4
                {
                    patch_dword(&mut cmd, gb, dword(&own_req, rb));
                }
                let err_on = opts
                    .inject_error
                    .is_some_and(|(er, at)| er == ri && t >= at)
                    && opts.reset_at_ms.is_none_or(|r| t < r);
                let mut safe_out = [0u8; 8];
                for (k, b) in safe_out.iter_mut().enumerate() {
                    *b = bytes(&get(&head, &format!("GVL_IO.bySafeOut{k}_{tag}"))?)
                        .first()
                        .copied()
                        .unwrap_or(0);
                }
                match &mut d.dev {
                    Dev::St(h) => {
                        set(h, "GVL_IO.aVassOutputData", cmd);
                        for (k, b) in safe_out.iter().enumerate() {
                            set(h, &format!("GVL_IO.bySafeOut{k}"), Value::Byte(*b));
                        }
                        if err_on && !injected {
                            let _ = h.try_set_input("PLC_PRG.bTest_InjectError", Value::Bool(true));
                            injected = true;
                        }
                        if injected && !err_on && opts.inject_error.is_some_and(|(er, _)| er == ri)
                        {
                            let _ =
                                h.try_set_input("PLC_PRG.bTest_InjectError", Value::Bool(false));
                        }
                        h.advance_time(Duration::from_millis(ddt as i64));
                        let r = h.cycle();
                        if !r.errors.is_empty() {
                            rep.runtime_errors.push(format!(
                                "{:.3} s robot {tag}: {:?}",
                                secs(t),
                                r.errors
                            ));
                            rep.end_ms = t;
                            return Ok(rep);
                        }
                    }
                    Dev::Model { m, inp, safe_in } => {
                        let cmd = bytes(&cmd);
                        if cmd.len() < 32 {
                            return Err(Error::Sim(format!(
                                "robot {tag}: process image is not 32 bytes"
                            )));
                        }
                        if err_on && !injected {
                            m.inject_error(true);
                            injected = true;
                        }
                        if injected && !err_on && opts.inject_error.is_some_and(|(er, _)| er == ri)
                        {
                            m.inject_error(false);
                        }
                        let (i, s) = m.scan(ddt, &to_frame(&cmd), &safe_out);
                        *inp = i;
                        *safe_in = s;
                    }
                }
            }
        }
        // ── head PLC ──
        if t.is_multiple_of(hdt) {
            for d in &devs {
                let tag = &d.tag;
                match &d.dev {
                    Dev::St(h) => {
                        set(
                            &mut head,
                            &format!("GVL_IO.aVassInputData_{tag}"),
                            get(h, "GVL_IO.aVassInputData")?,
                        );
                        for k in 0..8 {
                            set(
                                &mut head,
                                &format!("GVL_IO.bySafeIn{k}_{tag}"),
                                get(h, &format!("GVL_IO.bySafeIn{k}"))?,
                            );
                        }
                    }
                    Dev::Model { inp, safe_in, .. } => {
                        let name = format!("GVL_IO.aVassInputData_{tag}");
                        let mut v = get(&head, &name)?;
                        patch_bytes(&mut v, inp);
                        set(&mut head, &name, v);
                        for (k, b) in safe_in.iter().enumerate() {
                            set(
                                &mut head,
                                &format!("GVL_IO.bySafeIn{k}_{tag}"),
                                Value::Byte(*b),
                            );
                        }
                    }
                }
            }
            if let Some(r) = opts.reset_at_ms {
                if t >= r && !reset_sent {
                    let _ = head.try_set_input("PLC_PRG.bReset", Value::Bool(true));
                    reset_sent = true;
                } else if reset_sent && t >= r + 100 && t < r + 2_000 {
                    let _ = head.try_set_input("PLC_PRG.bReset", Value::Bool(false));
                } else if reset_sent && t >= r + 2_000 && t < r + 2_100 {
                    let _ = head.try_set_input("PLC_PRG.bRecoveryConfirm", Value::Bool(true));
                } else if reset_sent && t >= r + 2_100 {
                    let _ = head.try_set_input("PLC_PRG.bRecoveryConfirm", Value::Bool(false));
                }
            }
            head.advance_time(Duration::from_millis(hdt as i64));
            let r = head.cycle();
            if !r.errors.is_empty() {
                rep.runtime_errors
                    .push(format!("{:.3} s head: {:?}", secs(t), r.errors));
                rep.end_ms = t;
                return Ok(rep);
            }
            // ── trace from the frames ──
            let open = rep.cycle_start_ms.len() > rep.cycle_end_ms.len();
            let cell_cycle = open.then_some(rep.cycle_start_ms.len() as u32);
            let mut zone_holder: Vec<Option<usize>> = vec![None; plan.zones.len()];
            for d in &devs {
                let (ri, tag) = (&d.ri, &d.tag);
                let inp = bytes(&get(&head, &format!("GVL_IO.aVassInputData_{tag}"))?);
                let out = bytes(&get(&head, &format!("GVL_IO.aVassOutputData_{tag}"))?);
                if inp.len() < 32 || out.len() < 32 {
                    return Err(Error::Sim(format!(
                        "robot {tag}: process image is not 32 bytes"
                    )));
                }
                let f = match &vwmap {
                    Some(m) => Frame::decode_vw(&inp, &out, m, *ri),
                    None => Frame::decode(&inp, &out, sb, rb, gb),
                };
                tr[*ri].scan(f, t, *ri, plan, cell_cycle, &mut rep);
                for (zi, holder) in zone_holder.iter_mut().enumerate() {
                    if f.grant & (1 << zi) != 0 {
                        if let Some(other) = *holder {
                            rep.exclusion_violations.push(format!(
                                "{:.3} s: zone {} granted to R{} and R{}",
                                secs(t),
                                plan.zones[zi].name,
                                other + 1,
                                ri + 1
                            ));
                        }
                        *holder = Some(*ri);
                    }
                }
            }
            let st = enum_text(head.get_output("PLC_PRG.fbCell.eState"));
            if st != last_state {
                rep.events.push(Event {
                    t_ms: t,
                    robot: usize::MAX,
                    kind: EvKind::CellState(st.clone()),
                });
                if (st.contains("CS_RUNNING") || st.contains("CS_START_JOBS")) && !open {
                    rep.cycle_start_ms.push(t);
                }
                if st.contains("CS_ABORTING") {
                    rep.abort_cycle(&mut tr, t);
                }
                if st.contains("CS_CYCLE_DONE") && open {
                    rep.cycle_end_ms.push(t);
                    let c = rep.cycle_end_ms.len();
                    for (ri, x) in tr.iter().enumerate() {
                        if let Some(o) = x.open {
                            rep.trace_violations.push(format!(
                                "cycle {c}: {} still running at the cycle end ({:.3} s)",
                                op_name(plan, ri, o.index),
                                secs(t)
                            ));
                        }
                    }
                    if rep.cycle_end_ms.len() as u32 >= opts.cycles {
                        rep.end_ms = t;
                        return Ok(rep);
                    }
                }
                last_state = st;
            }
            let er = enum_text(head.get_output("PLC_PRG.fbCell.eError"));
            if er != last_error {
                rep.events.push(Event {
                    t_ms: t,
                    robot: usize::MAX,
                    kind: EvKind::CellError(er.clone()),
                });
                rep.cell_errors.push(CellFault {
                    t_ms: t,
                    error: er.clone(),
                    robot: int(head.get_output("PLC_PRG.fbCell.nErrorRobot")),
                    op: int(head.get_output("PLC_PRG.fbCell.nErrorOp")),
                });
                last_error = er;
            }
        }
    }
    rep.end_ms = t;
    rep.runtime_errors.push(format!(
        "timeout: {} of {} cycle(s) completed after {:.1} s virtual (last cell state {last_state}, error {last_error})",
        rep.cycle_end_ms.len(),
        opts.cycles,
        secs(t)
    ));
    Ok(rep)
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}
