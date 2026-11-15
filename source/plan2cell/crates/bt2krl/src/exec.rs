//! Executor of the generated IR: behaves like the KRL on the wire (TASK 8.4).
//!
//! One [`Executor::scan`] is one controller cycle. It plays the role of the submit interpreter
//! (`bt_sps`: job handshake, status bits, abort latch, step/REQUEST clearing, joint encoder) and
//! of the robot interpreter (`bt_cell` plus the Folge: the instruction list of
//! [`crate::ir::Folge`]). The frames are the bt2plc VASS telegram, so it can stand in for the
//! generated device ST in the bt2plc co-simulation (`bt2plc::trust::cosim::RobotModel`).
//!
//! Motion: constant speed per segment, exactly the time the KRL renderer computed the velocity
//! for, plus `ramp_time_s` per op. Approximation (`C_PTP`) is NOT modelled: the executor stops
//! at every point, so a real robot is never slower than this model. The pose is interpolated
//! linearly in joint space, so the co-simulation's "no motion without a grant" check sees it.

use crate::config::{Config, EAxes, Profile};
use crate::io::{self, SimIo};
use crate::ir::{Folge, Instr};
use crate::lower::{Lowered, RobotSetup};
use crate::motion;
use crate::program::Program;

/// Instruction fall-through bound per scan (as `FB_ProgramPlayer`'s N_MAX_TRANSITIONS).
const MAX_TRANSITIONS: usize = 16;
/// `$PERI_RDY` after DrivesOn, as the device simulator (`_tonDriveStartup`).
const DRIVE_STARTUP_MS: u64 = 2_000;
/// Reset must be held this long to clear an error (`bt_sps`).
const RESET_HOLD_MS: u64 = 500;
/// Profile `vw-ks24`: VW signal indices the executor drives (see `crate::vw`).
const VW_READY_OUT: u16 = 9;
const VW_RK100_OUT: u16 = 10;
const VW_AUTO_OUT: u16 = 12;
const VW_ESTOP_OUT: u16 = 13;
const VW_END_OUT: u16 = 14;
const VW_DRIVES_IN: u16 = 10;
const VW_SRB_IN: u16 = 9;
const VW_RESET_IN: u16 = 26;

/// Telegram offsets the executor reads and writes (all little-endian).
#[derive(Clone, Copy, Debug)]
struct Offsets {
    step: usize,
    request: usize,
    grant: usize,
    plan_id: usize,
    job_byte: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Run {
    /// No Folge selected.
    Idle,
    /// Executing `flat[pc]` of the production (`prod = true`) or HOME Folge.
    Active { prod: bool, pc: usize },
}

#[derive(Clone, Debug)]
struct Motion {
    from: [f64; 6],
    to: [f64; 6],
    dur_ms: u64,
    done_ms: u64,
}

pub struct Executor {
    // ── static ──
    profile: Profile,
    /// Profile `vw-ks24`: zone bit -> VW interlock index.
    il: Vec<u8>,
    /// First `$OUT` of the op index signal (vw).
    vw_step_out: u16,
    /// Release level held for at least this long after an unlock (vw), so the head sees it.
    off: Offsets,
    prod_folge: u16,
    home_folge: u16,
    plan_id: u16,
    err_unknown_folge: u16,
    err_plan_mismatch: u16,
    warn_start_dev: u16,
    at_home_tol: f64,
    ramp_ms: u64,
    vmax: [f64; 6],
    min_vel: f64,
    max_vel: f64,
    time_margin: f64,
    home_pose: [f64; 6],
    prod: Folge,
    home: Folge,
    prod_flat: Vec<Instr>,
    home_flat: Vec<Instr>,
    /// Every REQUEST bit the production Folge uses (cleared when no Folge runs).
    used_mask: u32,

    // ── robot state ──
    pose: [f64; 6],
    drives_ms: u64,
    drives: bool,
    ext: bool,
    error: bool,
    ecode: u16,
    wcode: u16,
    running: bool,
    finished: bool,
    ack: bool,
    abort: bool,
    job: u16,
    plan_latched: u16,
    step: u16,
    request: u32,
    run: Run,
    motion: Option<Motion>,
    /// Remaining time of a blocking WAIT SEC.
    wait_ms: u64,
    reset_ms: u64,
    inject_error: bool,
    // edge memories (bt_sps)
    m_job: bool,
    m_start: bool,
    m_reset: bool,
    start: bool,
    /// Profile `vw-ks24`: the Folge number mirrored on PRG_NR_REFL (0 = waiting for a number).
    vw_echo: u16,
    /// Profile `vw-ks24`: the Folge reached its end (A14) and waits for PRG_NR = 0.
    vw_end: bool,
    /// Profile `vw-ks24`: interlock release levels A41-56 (TRUE = area free).
    vw_release: u32,
    /// Profile `vw-ks24`: interlock request levels A81-96.
    vw_request: u32,
    /// Profile `vw-ks24`: SRB (Folge start) from the PLC.
    vw_srb: bool,
    /// Profile `vw-ks24`: the PLC's interlock grants E81-96.
    vw_grant: u32,
    /// Profile `vw-ks24`: the partner releases E41-56 routed by the PLC.
    vw_partner: u32,
    /// Profile `vw-ks24`: interlocks whose partner release the driving condition watches.
    vw_watch: u32,
    /// Profile `vw-ks24`: the robot stands at its home point P0 (PF0, A15).
    vw_at_home: bool,
    /// Profile `vw-ks24`: the safe channel is closed (RK100).
    vw_safety: bool,
    /// Profile `vw-ks24`: PLC -> robot byte of PLAN_ID (0 = no check).
    vw_plan_byte: u8,
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

fn put_word(b: &mut [u8], i: usize, v: u16) {
    b[i] = v as u8;
    b[i + 1] = (v >> 8) as u8;
}

fn put_dword(b: &mut [u8], i: usize, v: u32) {
    for k in 0..4 {
        b[i + k] = (v >> (8 * k)) as u8;
    }
}

impl Executor {
    /// `cfg` and `l` must be the ones the KRL was generated from. The `sim` IO bases must be 0:
    /// the executor works in telegram bytes, which is what base 0 maps to.
    pub fn new(p: &Program, s: &RobotSetup, cfg: &Config, l: &Lowered) -> crate::Result<Executor> {
        if cfg.sim.in_base != 0 || cfg.sim.out_base != 0 {
            return Err(crate::Error::Usage(
                "the IR executor needs sim.in_base = sim.out_base = 0 (telegram mapping)".into(),
            ));
        }
        // E axes are not part of the protocol and are not modelled: the executor plays A1..A6
        // (with e_axes = "fixed" the real robot also moves E1..E6 to their configured values).
        let _ = EAxes::Fixed;
        // The HOME Folge only exists in profile sim; in vw-ks24 the production Folge is the
        // only program and the robot is homed by the operator.
        let home = l.home.clone().unwrap_or_else(|| Folge {
            number: p.folgen.home,
            kind: crate::ir::FolgeKind::Home,
            points: Vec::new(),
            prologue: Vec::new(),
            ops: Vec::new(),
            epilogue: Vec::new(),
        });
        let profile = if l.interlocks.is_some() {
            Profile::VwKs24
        } else {
            Profile::Sim
        };
        let mut il = vec![0u8; 32];
        if let Some(m) = &l.interlocks {
            for z in &p.zones {
                if let Some(&n) = m.map.get(&z.name) {
                    il[usize::from(z.bit)] = n;
                }
            }
        }
        let pr = &p.protocol;
        Ok(Executor {
            off: Offsets {
                step: pr.step.byte as usize,
                request: pr.request.byte as usize,
                grant: pr.grant.byte as usize,
                plan_id: pr.plan_id.byte as usize,
                job_byte: pr.job_number_byte(),
            },
            prod_folge: p.folgen.production,
            home_folge: p.folgen.home,
            plan_id: p.plan_id(),
            err_unknown_folge: pr.error_codes.unknown_folge,
            err_plan_mismatch: pr.error_codes.plan_mismatch,
            warn_start_dev: pr.error_codes.start_deviation,
            at_home_tol: pr.at_home_tol_deg,
            ramp_ms: crate::ir::ms(cfg.motion.ramp_time_s),
            vmax: s.model.vmax,
            min_vel: f64::from(cfg.motion.min_vel_pct),
            max_vel: f64::from(cfg.motion.max_vel_pct),
            time_margin: cfg.motion.time_margin,
            home_pose: p.home_pose_deg.map(crate::ir::r4),
            profile,
            il,
            vw_step_out: crate::vw::step_signal(&cfg.vw_ks24).0,
            prod_flat: l.prod.instrs().cloned().collect(),
            home_flat: home.instrs().cloned().collect(),
            prod: l.prod.clone(),
            home,
            used_mask: l.prod.ops.iter().fold(0, |m, o| m | o.mask),
            pose: p.home_pose_deg.map(crate::ir::r4),
            drives_ms: 0,
            drives: false,
            ext: false,
            error: false,
            ecode: 0,
            wcode: 0,
            running: false,
            finished: false,
            ack: false,
            abort: false,
            job: 0,
            plan_latched: 0,
            step: 0,
            request: 0,
            run: Run::Idle,
            motion: None,
            wait_ms: 0,
            reset_ms: 0,
            inject_error: false,
            m_job: false,
            m_start: false,
            m_reset: false,
            start: false,
            vw_echo: 0,
            vw_end: false,
            vw_release: u32::MAX,
            vw_request: 0,
            vw_srb: false,
            vw_grant: 0,
            vw_partner: 0,
            vw_watch: 0,
            vw_at_home: true,
            vw_safety: false,
            vw_plan_byte: cfg.vw_ks24.plan_id_byte,
        })
    }

    pub fn pose(&self) -> [f64; 6] {
        self.pose
    }

    pub fn at_home(&self) -> bool {
        motion::max_abs_diff(&self.pose, &self.home_pose) <= self.at_home_tol
    }

    /// PLAN_ID the production Folge requires.
    pub fn plan_id(&self) -> u16 {
        self.plan_id
    }

    pub fn step(&self) -> u16 {
        self.step
    }

    pub fn request(&self) -> u32 {
        self.request
    }

    /// Program counter and Folge for tests: `(production?, pc)` or `None` when idle.
    pub fn program(&self) -> Option<(bool, usize)> {
        match self.run {
            Run::Idle => None,
            Run::Active { prod, pc } => Some((prod, pc)),
        }
    }

    /// The instruction the program is at (tests and diagnostics).
    pub fn current(&self) -> Option<&Instr> {
        match self.run {
            Run::Idle => None,
            Run::Active { prod, pc } => self.flat(prod).get(pc),
        }
    }

    pub fn error_code(&self) -> u16 {
        self.ecode
    }

    pub fn warning_code(&self) -> u16 {
        self.wcode
    }

    /// `PLC_PRG.bTest_InjectError` equivalent: a robot fault while the program runs.
    pub fn inject_error(&mut self, on: bool) {
        self.inject_error = on;
    }

    /// VW interlock indices of a zone mask (ascending).
    fn il_of(&self, mask: u32) -> Vec<u8> {
        let mut v: Vec<u8> = (0..31)
            .filter(|b| mask & (1 << b) != 0)
            .map(|b| self.il[b as usize])
            .filter(|&n| (1..=16).contains(&n))
            .collect();
        v.sort_unstable();
        v
    }

    fn flat(&self, prod: bool) -> &[Instr] {
        if prod {
            &self.prod_flat
        } else {
            &self.home_flat
        }
    }

    fn folge(&self, prod: bool) -> &Folge {
        if prod { &self.prod } else { &self.home }
    }

    /// Abort (Stop/Cancel, drives off, E-stop, fault): brake, clear step/REQUEST, ProgRunning
    /// off without ProgFinished, leave the Folge (as `BT_ABORT` + `bt_cell`).
    fn do_abort(&mut self) {
        self.run = Run::Idle;
        self.motion = None;
        self.wait_ms = 0;
        self.step = 0;
        self.request = 0;
        self.running = false;
        self.finished = false;
        self.abort = false;
    }

    fn end_program(&mut self) {
        self.run = Run::Idle;
        self.motion = None;
        self.wait_ms = 0;
    }

    fn start_motion(&mut self, to: [f64; 6], vel_pct: u8, extra_ms: u64) {
        let dq = motion::delta(&self.pose, &to);
        let dur = crate::ir::ms(motion::seg_time(&dq, vel_pct, &self.vmax)) + extra_ms;
        self.motion = Some(Motion {
            from: self.pose,
            to,
            dur_ms: dur,
            done_ms: 0,
        });
    }

    /// Velocity of the HOME Folge move, computed at run time from the current pose (the KRL does
    /// the same with `$AXIS_ACT`).
    fn home_vel(&self, to: &[f64; 6], time_ms: u64) -> u8 {
        let dq = motion::delta(&self.pose, to);
        let need = (0..6)
            .map(|i| dq[i].abs() / self.vmax[i])
            .fold(0.0, f64::max);
        let t = (time_ms as f64 / 1000.0 * self.time_margin).max(0.001);
        (100.0 * need / t).clamp(self.min_vel, self.max_vel) as u8
    }

    /// Advance the motion; returns true when it is finished.
    fn advance_motion(&mut self, dt_ms: u64) -> bool {
        let Some(m) = self.motion.as_mut() else {
            return true;
        };
        m.done_ms += dt_ms;
        if m.done_ms >= m.dur_ms {
            self.pose = m.to;
            self.motion = None;
            return true;
        }
        let a = m.done_ms as f64 / m.dur_ms as f64;
        let (from, to) = (m.from, m.to);
        self.pose = std::array::from_fn(|i| from[i] + a * (to[i] - from[i]));
        false
    }

    /// One controller cycle: decode the commands, run the submit logic and the interpreter,
    /// encode the status.
    pub fn scan(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]) {
        if self.profile == Profile::VwKs24 {
            return self.scan_vw(dt_ms, cmd, safe_out);
        }
        let c_drives = cmd[0] & 0x01 != 0;
        let c_start = cmd[0] & 0x02 != 0;
        let c_stop = cmd[0] & 0x04 != 0;
        let c_cancel = cmd[0] & 0x08 != 0;
        let c_ext = cmd[0] & 0x10 != 0;
        let c_reset = cmd[0] & 0x20 != 0;
        let c_jobreq = cmd[1] & 0x01 != 0;
        let grant = dword(cmd, self.off.grant);
        let plan_in = word(cmd, self.off.plan_id);
        let job_in = if self.off.job_byte {
            u16::from(cmd[io::JOB_BYTE as usize])
        } else {
            word(cmd, io::JOB_BYTE as usize)
        };
        // The safe channel is the controller's PROFIsafe configuration, not KRL: mirror the
        // device simulator (drive enable bit 0, external E-stop bit 1, SS1 bit 2).
        let safe_enable = safe_out[0] & 0x01 != 0;
        let estop = safe_out[0] & 0x02 != 0;
        let ss1 = safe_out[0] & 0x04 != 0;
        let safety = safe_enable && !estop;

        // ── drives, mode, errors ($PERI_RDY, $MODE_OP) ──
        if c_drives && safety {
            self.drives_ms = self.drives_ms.saturating_add(dt_ms);
            if self.drives_ms >= DRIVE_STARTUP_MS {
                self.drives = true;
            }
        } else {
            self.drives_ms = 0;
            self.drives = false;
        }
        self.ext = c_ext && self.drives && safety;
        if self.inject_error && self.running {
            self.error = true;
            if self.ecode == 0 {
                self.ecode = 0x0999; // test fault, like the device simulator's debug input
            }
        }

        // ── reset (bt_sps) ──
        if c_reset {
            if !self.m_reset {
                self.wcode = 0;
                self.reset_ms = 0;
            }
            self.reset_ms += dt_ms;
            if self.error && self.reset_ms >= RESET_HOLD_MS && !self.inject_error {
                self.error = false;
                self.ecode = 0;
                self.wcode = 0;
            }
        } else {
            self.reset_ms = 0;
        }
        self.m_reset = c_reset;

        // ── abort latch (bt_sps) ──
        if self.running && (c_stop || c_cancel || !self.drives || !safety || ss1 || self.error) {
            self.abort = true;
        }
        if self.abort {
            self.do_abort();
        }

        let ready = self.drives && self.ext && !self.error && !self.running && safety;

        // ── job handshake (bt_sps) ──
        if c_jobreq && !self.m_job && ready {
            self.job = job_in;
            self.plan_latched = plan_in;
            self.ack = true;
            self.finished = false;
        }
        if !c_jobreq {
            self.ack = false;
        }
        self.m_job = c_jobreq;
        if c_start && !self.m_start && ready && self.job != 0 {
            self.start = true;
        }
        self.m_start = c_start;

        // ── bt_cell: dispatch ──
        if self.start && self.run == Run::Idle {
            self.start = false;
            self.finished = false;
            if self.job == self.prod_folge {
                self.run = Run::Active { prod: true, pc: 0 };
            } else if self.job == self.home_folge {
                self.run = Run::Active { prod: false, pc: 0 };
            } else {
                self.error = true;
                self.ecode = self.err_unknown_folge;
            }
        }

        // ── the Folge ──
        self.run_folge(dt_ms, grant);

        // ── no Folge active: clear step and REQUEST (bt_sps) ──
        if self.run == Run::Idle {
            self.step = 0;
            self.request &= !self.used_mask;
        }

        (self.encode(), self.encode_safe(safety, ss1))
    }

    fn run_folge(&mut self, dt_ms: u64, grant: u32) {
        let Run::Active { prod, mut pc } = self.run else {
            return;
        };
        // Timed instruction in progress: one of WAIT SEC / motion.
        if self.wait_ms > 0 {
            self.wait_ms = self.wait_ms.saturating_sub(dt_ms);
            if self.wait_ms > 0 {
                return;
            }
            pc += 1;
        } else if self.motion.is_some() {
            if !self.advance_motion(dt_ms) {
                return;
            }
            pc += 1;
        }
        let mut step_written = false;
        for _ in 0..MAX_TRANSITIONS {
            let Some(ins) = self.flat(prod).get(pc).cloned() else {
                self.end_program();
                return;
            };
            match ins {
                Instr::Init { .. } => {}
                Instr::CheckPlan(id) => {
                    if self.plan_latched != id {
                        self.error = true;
                        self.ecode = self.err_plan_mismatch;
                        self.running = false;
                        self.finished = false;
                        self.end_program();
                        return;
                    }
                }
                Instr::Running => self.running = true,
                Instr::CheckStart { point, tol_mdeg } => {
                    let q = self.folge(prod).points[point].q;
                    if motion::max_abs_diff(&self.pose, &q) > f64::from(tol_mdeg) / 1000.0 {
                        self.wcode = self.warn_start_dev;
                    }
                }
                Instr::AbortGuard => {
                    if self.abort {
                        self.do_abort();
                        return;
                    }
                }
                Instr::SetStep(k) => {
                    if step_written {
                        self.run = Run::Active { prod, pc };
                        return; // one step change per scan (the head checks +1 per telegram)
                    }
                    self.step = k;
                    step_written = true;
                }
                Instr::Request(m) => self.request |= m,
                Instr::WaitGrant(m) => {
                    if grant & m != m {
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                }
                Instr::Delay(ms) | Instr::Hold(ms) | Instr::Dwell(ms) => {
                    if ms > 0 {
                        self.wait_ms = ms;
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                }
                Instr::Move {
                    point,
                    vel_pct,
                    approx,
                } => {
                    let q = self.folge(prod).points[point].q;
                    // ramp_time_s is charged once per op, at its first motion.
                    let first = !matches!(
                        self.flat(prod).get(pc.wrapping_sub(1)),
                        Some(Instr::Move { .. })
                    );
                    let extra = if first { self.ramp_ms } else { 0 };
                    let _ = approx;
                    self.start_motion(q, vel_pct, extra);
                    self.run = Run::Active { prod, pc };
                    return;
                }
                Instr::HomeMove { point, time_ms } => {
                    let q = self.folge(prod).points[point].q;
                    let v = self.home_vel(&q, time_ms);
                    self.start_motion(q, v, 0);
                    self.run = Run::Active { prod, pc };
                    return;
                }
                Instr::Release(m) => self.request &= !m,
                Instr::WaitReleased(m) => {
                    if grant & m != 0 {
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                }
                Instr::Finish => {
                    self.step = 0;
                    self.finished = true;
                    self.running = false;
                    self.end_program();
                    return;
                }
                Instr::Done => {
                    self.finished = true;
                    self.running = false;
                    self.end_program();
                    return;
                }
                // ── profile vw-ks24 ──
                Instr::VwAueInit => {}
                Instr::VwAueWait => {
                    // "Warte auf Folgenstart": mirror PRG_NR, then wait for SRB.
                    self.vw_echo = self.job;
                    if !self.vw_srb {
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                }
                Instr::VwCycleStart { mask, .. } => {
                    self.running = true;
                    self.vw_at_home = false;
                    for n in self.il_of(mask) {
                        self.vw_release |= 1 << n;
                    }
                }
                Instr::VwLock(m) => {
                    // MAKRO20: request, wait for the PLC grant, wait for the partner release,
                    // drop the own release ("occupied"), clear the request.
                    let ix = self.il_of(m);
                    for n in &ix {
                        self.vw_request |= 1 << n;
                    }
                    let granted = ix.iter().all(|n| self.vw_grant & (1 << n) != 0);
                    let partners = ix.iter().all(|n| self.vw_partner & (1 << n) != 0);
                    if !(granted && partners) {
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                    for n in &ix {
                        self.vw_release &= !(1 << n);
                        self.vw_request &= !(1 << n);
                    }
                }
                Instr::VwUnlock(m) => {
                    for n in self.il_of(m) {
                        self.vw_release |= 1 << n;
                    }
                }
                Instr::VwOnlineStop(held) => {
                    // The online driving condition: while an interlock is held, the partner
                    // release must stay set, else the robot stops on its path (VW(#FB_STOP)).
                    self.vw_watch = held;
                }
                Instr::VwP0 { point } => {
                    let q = self.folge(prod).points[point].q;
                    if self.motion.is_none() && motion::max_abs_diff(&self.pose, &q) > 1e-4 {
                        self.start_motion(q, 10, self.ramp_ms);
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                    self.vw_at_home = true;
                    self.vw_release = u32::MAX;
                    self.vw_request = 0;
                }
                Instr::VwMakro15 | Instr::VwTechCall(_) => {}
                Instr::VwAueEnd => {
                    // AUE_END: Folge end A14, wait for PRG_NR = 0, then back to P0.
                    self.vw_end = true;
                    self.running = false;
                    if self.job != 0 {
                        self.run = Run::Active { prod, pc };
                        return;
                    }
                    self.vw_end = false;
                    self.vw_echo = 0;
                    self.finished = true;
                    self.end_program();
                    return;
                }
            }
            pc += 1;
        }
        self.run = Run::Active { prod, pc };
    }

    /// One controller cycle in profile `vw-ks24`: the VW tech package's own handshake
    /// (`cell.src` + `AUE_WAIT`/`AUE_END`) and the standard robot interlocks.
    fn scan_vw(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]) {
        let bit_in = |i: u16| -> bool {
            let i = usize::from(i) - 1;
            cmd[i / 8] & (1 << (i % 8)) != 0
        };
        let c_drives = bit_in(VW_DRIVES_IN);
        let c_reset = bit_in(VW_RESET_IN);
        self.vw_srb = bit_in(VW_SRB_IN);
        let prg_nr = u16::from(cmd[0]);
        let safe_enable = safe_out[0] & 0x01 != 0;
        let estop = safe_out[0] & 0x02 != 0;
        let safety = safe_enable && !estop;
        self.vw_safety = safety;
        if self.vw_plan_byte != 0 {
            self.plan_latched = word(cmd, usize::from(self.vw_plan_byte));
        }
        self.vw_grant = 0;
        self.vw_partner = 0;
        for n in 1..=16u32 {
            if bit_in(crate::vw::grant_in(n as u8)) {
                self.vw_grant |= 1 << n;
            }
            if bit_in(crate::vw::partner_in(n as u8)) {
                self.vw_partner |= 1 << n;
            }
        }

        // Drives and mode (the KS24 robot reports "ready" only with drives on, in auto).
        if c_drives && safety {
            self.drives_ms = self.drives_ms.saturating_add(dt_ms);
            if self.drives_ms >= DRIVE_STARTUP_MS {
                self.drives = true;
            }
        } else {
            self.drives_ms = 0;
            self.drives = false;
        }
        self.ext = self.drives && safety;
        if self.inject_error && self.running {
            self.error = true;
            if self.ecode == 0 {
                self.ecode = 0x99;
            }
        }
        if c_reset && !self.m_reset {
            self.error = false;
            self.ecode = 0;
        }
        self.m_reset = c_reset;

        // The job number: cell.src picks up PRG_NR; AUE_WAIT leaves the Folge when it changes.
        self.job = prg_nr;
        if prg_nr != 0 && self.run == Run::Idle && !self.error && self.drives && self.at_home() {
            if prg_nr == self.prod_folge {
                self.run = Run::Active { prod: true, pc: 0 };
                self.finished = false;
            } else {
                // Any other Folge is the cell's own (home, maintenance): mirror it and report
                // the end straight away - bt2krl does not generate those programs.
                self.vw_echo = prg_nr;
                self.vw_end = true;
            }
        }
        if prg_nr == 0 && self.run == Run::Idle {
            self.vw_end = false;
            self.vw_echo = 0;
        }

        // Abort: the drives drop, the safety opens, or a partner withdrew its release while an
        // interlock is held (the online driving condition stops the robot on its path).
        let watch_lost = self
            .il_of(self.vw_watch)
            .iter()
            .any(|n| self.vw_partner & (1 << n) == 0);
        if self.run != Run::Idle && (!self.drives || !safety || self.error) {
            self.run = Run::Idle;
            self.motion = None;
            self.wait_ms = 0;
            self.running = false;
            self.finished = false;
            self.step = 0;
            self.vw_echo = 0;
            self.vw_end = false;
            self.vw_watch = 0;
        } else if watch_lost {
            // FB_STOP: brake and wait, keep the step and the interlocks.
            self.wait_ms = self.wait_ms.max(dt_ms);
        }

        if !watch_lost {
            self.run_folge(dt_ms, 0);
        }
        (self.encode_vw(), self.encode_safe(safety, false))
    }

    fn encode_vw(&self) -> [u8; 32] {
        let mut b = [0u8; 32];
        let mut set = |i: u16, v: bool| {
            if v {
                let i = usize::from(i) - 1;
                b[i / 8] |= 1 << (i % 8);
            }
        };
        set(
            VW_READY_OUT,
            self.drives && self.ext && !self.error && self.run == Run::Idle,
        );
        set(VW_RK100_OUT, self.vw_safety && !self.error);
        set(VW_AUTO_OUT, self.ext);
        set(VW_ESTOP_OUT, true);
        set(VW_END_OUT, self.vw_end);
        set(
            crate::vw::PF0_OUT,
            if self.run == Run::Idle {
                self.at_home()
            } else {
                self.vw_at_home
            },
        );
        for n in 1..=16u16 {
            set(
                crate::vw::release_out(n as u8),
                self.vw_release & (1 << n) != 0,
            );
            set(
                crate::vw::request_out(n as u8),
                self.vw_request & (1 << n) != 0,
            );
        }
        // PRG_NR_REFL A1-8 and the op index BT_PKTNR (the outputs of P_PktNr_SPS).
        b[0] = self.vw_echo as u8;
        let si = usize::from(self.vw_step_out) - 1;
        b[si / 8] = self.step as u8;
        b[si / 8 + 1] = (self.step >> 8) as u8;
        let ei = usize::from(crate::vw::ERROR_OUT) - 1;
        b[ei / 8] = self.ecode as u8;
        for (j, q) in self.pose.iter().enumerate() {
            let v = (q * 50.0)
                .round()
                .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
            let [hi, lo] = v.to_be_bytes();
            b[io::JOINT_BYTE as usize + 2 * j] = hi;
            b[io::JOINT_BYTE as usize + 2 * j + 1] = lo;
        }
        b
    }

    fn encode(&self) -> [u8; 32] {
        let mut b = [0u8; 32];
        // byte 0
        b[0] = u8::from(self.drives)
            | (u8::from(self.running) << 1)
            | (u8::from(self.at_home()) << 2)
            | (u8::from(self.motion.is_none()) << 3)
            | (u8::from(self.ext) << 4)
            | (u8::from(self.drives && self.ext && !self.error && !self.running) << 5)
            | (u8::from(self.error) << 6)
            | (u8::from(self.wcode != 0) << 7);
        // byte 1: gates, E-stop OK, ProgFinished, JobAck, periphery ready
        b[1] = 0x01 | 0x02 | (u8::from(self.finished) << 2) | (u8::from(self.ack) << 3) | 0x40;
        // job echo
        b[io::ECHO_BYTE as usize] = self.job as u8;
        if !self.off.job_byte {
            b[io::ECHO_BYTE as usize + 1] = (self.job >> 8) as u8;
        }
        put_word(&mut b, io::ERROR_BYTE as usize, self.ecode);
        put_word(&mut b, io::WARNING_BYTE as usize, self.wcode);
        // joints: 6 x INT16 big-endian, 0.02 deg per count (bt_sps joint encoder)
        for (j, q) in self.pose.iter().enumerate() {
            let v = (q * 50.0)
                .round()
                .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
            let [hi, lo] = v.to_be_bytes();
            b[io::JOINT_BYTE as usize + 2 * j] = hi;
            b[io::JOINT_BYTE as usize + 2 * j + 1] = lo;
        }
        put_word(&mut b, self.off.step, self.step);
        put_dword(&mut b, self.off.request, self.request);
        b
    }

    fn encode_safe(&self, safety: bool, ss1: bool) -> [u8; 8] {
        let mut s = [0u8; 8];
        // The head needs SafeIn0.0 (drives OK) and SafeIn1.4 (E-stop OK).
        s[0] = u8::from(safety) | (u8::from(!self.running) << 1) | 0x08 | 0x10 | 0x20;
        s[1] = 0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x40;
        s[2] = u8::from(ss1) << 1;
        s
    }
}

/// Build the executor of one robot from its program.json.
pub fn executor(p: &Program, cfg: &Config) -> crate::Result<Executor> {
    let s = RobotSetup::resolve(p, cfg, true)?;
    let l = crate::lower::lower(p, &s, cfg, crate::Profile::Sim)?;
    Executor::new(p, &s, cfg, &l)
}

/// The IO mapping of a robot (for tests that decode frames).
pub fn io_of(p: &Program, cfg: &Config) -> SimIo {
    SimIo::new(p, &cfg.sim)
}
