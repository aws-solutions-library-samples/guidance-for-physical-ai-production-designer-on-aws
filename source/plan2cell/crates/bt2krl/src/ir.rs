//! Robot-program IR: one instruction list per Folge (TASK section 7).
//!
//! The renderer turns it into KRL per profile, [`crate::parse`] reads the rendered KRL back
//! into it (round trip), and the co-simulation executor runs it. Times are integer
//! milliseconds and joint values are rounded to 4 decimals, so the round trip is exact.

/// A motion target in the Folge's data list (`DECL AXIS`/`E6AXIS <name>`).
#[derive(Clone, Debug, PartialEq)]
pub struct Point {
    pub name: String,
    pub q: [f64; 6],
    /// E1..E6 for `e_axes = "fixed"` (E6AXIS); `None` = AXIS point.
    pub e: Option<[f64; 6]>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Instr {
    /// `BAS(#INITMOV,0)`, advance run, `$APO.CPTP`, optional tool/base.
    Init {
        approx_pct: u8,
        tool: u16,
        base: u16,
    },
    /// Production start: PLAN_ID from the head must equal this, else Error `plan_mismatch`
    /// and return without ProgRunning or motion.
    CheckPlan(u16),
    /// ProgRunning := TRUE (Folge-level flag).
    Running,
    /// Warning `start_deviation` when the pose is more than `tol_mdeg` / 1000 deg away from
    /// point `point` on any axis.
    CheckStart { point: usize, tol_mdeg: u32 },
    /// `IF BT_ABRT THEN RETURN`: no step change and no motion after an abort was latched
    /// (defense in depth behind the abort interrupt).
    AbortGuard,
    /// STEP := k (one atomic 16-bit write, before the request).
    SetStep(u16),
    /// REQUEST := mask (exactly the op's bits incl. the permit bit).
    Request(u32),
    /// Wait for every GRANT bit of the mask (advance-run stop before the motion).
    WaitGrant(u32),
    /// Start delay after the grants.
    Delay(u64),
    /// PTP to `point` at `vel_pct`; `approx` = `C_PTP` (never on the op's last point).
    Move {
        point: usize,
        vel_pct: u8,
        approx: bool,
    },
    /// PTP to `point` with the velocity computed at run time from the current pose so that
    /// the move takes about `time_ms` (HOME Folge; the start pose is unknown).
    HomeMove { point: usize, time_ms: u64 },
    /// Hold at the end pose (the process happens here).
    Hold(u64),
    /// Keep the step value visible for at least this long (ops shorter than that).
    Dwell(u64),
    /// REQUEST := 0 for the mask (after the exact stop and the hold).
    Release(u32),
    /// 4-phase: wait until no GRANT bit of the mask is set.
    WaitReleased(u32),
    /// Production end: advance-run stop, STEP := 0, ProgFinished, then ProgRunning := FALSE.
    Finish,
    /// HOME Folge end: advance-run stop, ProgFinished, then ProgRunning := FALSE.
    Done,

    // ───────── profile vw-ks24 (the VW tech package does the job handshake) ─────────
    /// `VW(#VW_AUE_INIT,TRUE)`, `PENTER:` and the online-stop interrupt.
    VwAueInit,
    /// P0 = home at 10 %: `$OUT[15]=TRUE` (PF0), MAKRO0, `VW(#VW_INIT,TRUE)` (runs `BAS(#INITMOV)`).
    VwP0 { point: usize },
    /// `VW(#VW_AUE_WAIT,TRUE)` ("Warte auf Folgenstart") and `IF N_VW THEN GOTO PEXIT`.
    /// It also sets `$H_POS = $AXIS_ACT`, so P0 defines home.
    VwAueWait,
    /// Cycle start (P1 point SPS): `$OUT[15]=FALSE`, MAKRO50, release every interlock used.
    VwCycleStart { mask: u32, makro50: bool },
    /// Enter the interlocks of a mask: request `$OUT[80+n]`, then the MAKRO20 handshake.
    VwLock(u32),
    /// Leave them again: `$OUT[40+n]=TRUE` (no 4-phase in VW).
    VwUnlock(u32),
    /// Online driving condition `$CYCFLAG[254]`: the partner releases of the held interlocks.
    VwOnlineStop(u32),
    /// Gesamtfertigmeldung `SPSMAKRO15` (config-gated).
    VwMakro15,
    /// A technology call during the hold of an op (`vw_ks24.weld_call`, config-gated).
    VwTechCall(String),
    /// `VW(#VW_AUE_END,TRUE)`, `GOTO PENTER`, `PEXIT:`.
    VwAueEnd,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpBlock {
    pub index: u16,
    pub operation_id: String,
    pub mask: u32,
    pub instrs: Vec<Instr>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FolgeKind {
    Production,
    Home,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Folge {
    pub number: u16,
    pub kind: FolgeKind,
    pub points: Vec<Point>,
    pub prologue: Vec<Instr>,
    pub ops: Vec<OpBlock>,
    pub epilogue: Vec<Instr>,
}

impl Folge {
    /// `FOLGE11`
    pub fn name(&self) -> String {
        format!("FOLGE{}", self.number)
    }

    pub fn instrs(&self) -> impl Iterator<Item = &Instr> {
        self.prologue
            .iter()
            .chain(self.ops.iter().flat_map(|o| o.instrs.iter()))
            .chain(self.epilogue.iter())
    }
}

/// Round to 4 decimals (the precision of program.json and of the rendered points).
pub fn r4(x: f64) -> f64 {
    let r = (x * 10000.0).round() / 10000.0;
    if r == 0.0 { 0.0 } else { r }
}

/// Seconds -> integer milliseconds.
pub fn ms(s: f64) -> u64 {
    (s * 1000.0).round().max(0.0) as u64
}
