//! `bt2krl.toml`: output style, motion, profile IO bases and per-robot data.
//!
//! Every field has a default except the per-robot `e_axes`, which must be declared: bt2krl
//! does not guess whether a robot carries external axes (TASK decision 4). Unknown fields are
//! rejected so typos surface.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::error::{Error, Result};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum Profile {
    /// bt2plc device-simulator telegram (generated head PLC, docker simulation, OfficeLite).
    Sim,
    /// VW KS24 robots (KSS V8.2, TPVW 8.1.8): cell.src, AUE handshake, A41-56/A81-96 interlocks.
    VwKs24,
}

impl Profile {
    pub fn name(self) -> &'static str {
        match self {
            Profile::Sim => "sim",
            Profile::VwKs24 => "vw-ks24",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "snake_case")]
pub enum MotionMode {
    /// Approximated joint PTP with per-segment velocity (default).
    Ptp,
    /// `PTP_SPLINE` with `TIME_BLOCK` (refused: unverified on the target KSS V8.2.29, see
    /// `docs/design.md`).
    Spline,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LineEnding {
    Crlf,
    Lf,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NameStyle {
    /// `folge11.src` (DEF FOLGE11)
    Lower,
    /// `FOLGE11.src`
    Upper,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EAxes {
    /// The robot has no external axes: AXIS points (A1-A6).
    None,
    /// E6AXIS points with the configured `e_values` for E1..E6.
    Fixed,
    /// AXIS points on a robot WITH external axes. Refused: that PTP to an AXIS target leaves
    /// E1 where it is is unverified on KSS 8.2.
    Keep,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub output: OutputCfg,
    pub motion: MotionCfg,
    pub sim: SimCfg,
    pub vw_ks24: VwKs24Cfg,
    #[serde(rename = "robot")]
    pub robots: Vec<RobotCfg>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OutputCfg {
    pub line_ending: LineEnding,
    pub program_name_style: NameStyle,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotionCfg {
    pub mode: MotionMode,
    /// Joint-space RDP tolerance in degrees (first/last point and phase boundaries are kept).
    pub decimate_tol_deg: f64,
    /// `$APO.CPTP` for the intermediate points of an op.
    pub approx_ptp_pct: u8,
    pub max_vel_pct: u8,
    pub min_vel_pct: u8,
    /// >= 1; > 1 makes the motion slower than planned (never faster).
    pub time_margin: f64,
    /// Time estimate only: the acceleration and deceleration of one op, added once per op
    /// (the points inside an op are approximated, so the robot does not stop between them).
    /// The KUKA machine data of the sample robots carries no usable per-axis acceleration
    /// (only the jog reduction factors `$RED_ACC_AXC`), so this is a constant; the default
    /// keeps the estimates of both example cells within 10 % of the planned `duration_s`.
    pub ramp_time_s: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SimCfg {
    /// Telegram byte b, bit i <-> `$IN[in_base + 8b + i + 1]`.
    pub in_base: u16,
    /// Telegram byte b, bit i <-> `$OUT[out_base + 8b + i + 1]`.
    pub out_base: u16,
    /// Interrupt priority of the abort interrupt in `bt_cell` (free user priority).
    pub interrupt_no: u8,
    /// `$TIMER[n]` used by `bt_sps` for the 500 ms reset.
    pub reset_timer: u8,
    /// Publish A1..A6 in bytes 16-27 (INT16 big-endian, 0.02 deg/count) from `bt_sps`.
    pub joint_encoder: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VwKs24Cfg {
    pub max_interlocks: u8,
    /// Op index -> `BT_PKTNR` (`$OUT[97..112]` = the outputs of the VW `P_PktNr_SPS`).
    pub step_signal: String,
    /// Add the partner releases E(40+n) to the online driving condition while inside.
    pub online_stop: bool,
    /// Hold the interlock release A(40+n) at least this long before the next op requests an
    /// interlock again. The VW scheme has no 4-phase handshake, so this is what lets the head
    /// PLC see the release and drop its grant: it must exceed one PROFINET cycle (8 ms on the
    /// sample robots) plus one head scan, with margin.
    pub release_dwell_ms: u64,
    /// Enter an interlock with `SEL_RES=SELECT(#MAKRO,20,...)` instead of the inline MAKRO20
    /// semantics (depends on the cell's macro numbering).
    pub use_makro20: bool,
    /// Call `SPSMAKRO0` ("RUECKSETZEN_EA") at the home point P0, as every VW sample Folge
    /// does. It resets A33-A96, which includes the tool releases A57-80.
    pub makro0: bool,
    /// Call `SPSMAKRO50` ("Ini Merker", cyclic flags of the driving conditions) at the cycle
    /// start point, as every VW sample Folge does.
    pub makro50: bool,
    /// Gesamtfertigmeldung `SPSMAKRO<n>` at the last point before home (0 = none; VW uses 15).
    pub makro15: u16,
    /// Technology call emitted during the hold of ops with this `operation_type`
    /// (e.g. `weld_call = "SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)"`).
    pub weld_call: String,
    /// `operation_type` the weld call applies to.
    pub weld_op_type: String,
    /// Write PLAN_ID into this PLC -> robot byte and check it (0 = no check; VW has no
    /// standard signal for it). Valid start bytes (two free bytes): 12, 13, 28, 29.
    pub plan_id_byte: u8,
    /// Also write patched copies of `cell.src` and `$config.dat` (marked optional in INSTALL.md).
    pub emit_patches: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RobotCfg {
    pub ns: String,
    /// Controller id for headers, INSTALL.md and the manifest (default: ns).
    pub controller: Option<String>,
    /// Robot model (soft limits, axis speeds); default: program.json `robot.model`.
    pub model: Option<String>,
    /// Required: `"none"` | `"fixed"` (needs `e_values`). `"keep"` is refused.
    pub e_axes: Option<EAxes>,
    pub e_values: Option<[f64; 6]>,
    /// Controller `$H_POS`; checked against `home_pose_deg` when set.
    pub h_pos_deg: Option<[f64; 6]>,
    pub h_pos_tol_deg: f64,
    /// `BAS(#TOOL,n)`; 0 = keep the current tool.
    pub tool_no: u16,
    /// `BAS(#BASE,n)`; 0 = keep the current base.
    pub base_no: u16,
    /// vw-ks24 only: zone name -> VW interlock index n (1..16).
    pub interlocks: BTreeMap<String, u8>,
}

impl Default for OutputCfg {
    fn default() -> Self {
        Self {
            line_ending: LineEnding::Crlf,
            program_name_style: NameStyle::Lower,
        }
    }
}

impl Default for MotionCfg {
    fn default() -> Self {
        Self {
            mode: MotionMode::Ptp,
            decimate_tol_deg: 0.5,
            approx_ptp_pct: 100,
            max_vel_pct: 100,
            min_vel_pct: 1,
            time_margin: 1.0,
            ramp_time_s: 0.25,
        }
    }
}

impl Default for SimCfg {
    fn default() -> Self {
        Self {
            in_base: 0,
            out_base: 0,
            interrupt_no: 20,
            reset_timer: 20,
            joint_encoder: true,
        }
    }
}

impl Default for VwKs24Cfg {
    fn default() -> Self {
        Self {
            max_interlocks: 16,
            step_signal: "out:97:16".into(),
            online_stop: true,
            use_makro20: false,
            release_dwell_ms: 100,
            makro0: true,
            makro50: true,
            makro15: 15,
            weld_call: String::new(),
            weld_op_type: "weld".into(),
            plan_id_byte: 0,
            emit_patches: true,
        }
    }
}

impl Default for RobotCfg {
    fn default() -> Self {
        Self {
            ns: String::new(),
            controller: None,
            model: None,
            e_axes: None,
            e_values: None,
            h_pos_deg: None,
            h_pos_tol_deg: 2.0,
            tool_no: 0,
            base_no: 0,
            interlocks: BTreeMap::new(),
        }
    }
}

/// Start bytes for PLAN_ID (16 bits: the byte and the next). PLC -> robot bytes 12, 13, 14, 17,
/// 19, 23, 28, 29, 30 are unused by the VW KS24 standard on all 8 sample robots
/// (`samples-config.md`; byte 31 carries E249/E250); only these start a pair of free bytes.
/// Same list as `bt2plc::config::vw_ks24::FREE_IN_BYTES`.
pub const VW_FREE_IN_BYTES: [u8; 4] = [12, 13, 28, 29];

/// Free user interrupt priorities on KSS 8 (3 and 40..80 are reserved by the system).
fn free_interrupt(n: u8) -> bool {
    matches!(n, 1 | 2 | 4..=39 | 81..=128)
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        Self::parse(&text)
    }

    pub fn parse(text: &str) -> Result<Self> {
        let c: Config = toml::from_str(text).map_err(|e| Error::Config(e.to_string()))?;
        c.validate()?;
        Ok(c)
    }

    pub fn default_toml() -> String {
        let mut s = String::from(
            "# bt2krl.toml: every field shown has its default. Per-robot data goes into\n\
             # [[robot]] tables; e_axes must be declared for every robot.\n\
             #\n\
             # [[robot]]\n\
             # ns = \"robot1\"\n\
             # controller = \"000001R01\"     # default: ns\n\
             # model = \"KR240R2900_ULTRA\"   # default: program.json robot.model\n\
             # e_axes = \"none\"              # \"none\" (no external axes) | \"fixed\" (+ e_values)\n\
             # e_values = [95.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n\
             # h_pos_deg = [-90.0, -120.0, 90.0, -5.0, 40.0, -5.0]\n\
             # h_pos_tol_deg = 2.0\n\
             # tool_no = 0                  # 0 = keep\n\
             # base_no = 0                  # 0 = keep\n\
             # interlocks = { IL01 = 1, IL02 = 11 }   # vw-ks24 only: zone -> VW interlock index 1..16\n\n",
        );
        s.push_str(&toml::to_string_pretty(&Config::default()).expect("default config serialises"));
        s
    }

    pub fn robot(&self, ns: &str) -> Option<&RobotCfg> {
        self.robots.iter().find(|r| r.ns == ns)
    }

    pub fn validate(&self) -> Result<()> {
        let bad = |m: String| Err(Error::Config(m));
        let m = &self.motion;
        if !(m.decimate_tol_deg.is_finite() && m.decimate_tol_deg >= 0.0) {
            return bad("motion.decimate_tol_deg must be >= 0".into());
        }
        if m.approx_ptp_pct > 100 {
            return bad("motion.approx_ptp_pct must be 0..=100".into());
        }
        if m.min_vel_pct == 0 || m.min_vel_pct > m.max_vel_pct || m.max_vel_pct > 100 {
            return bad("motion: need 1 <= min_vel_pct <= max_vel_pct <= 100".into());
        }
        if !(m.time_margin.is_finite() && m.time_margin >= 1.0) {
            return bad("motion.time_margin must be >= 1.0 (never faster than the plan)".into());
        }
        if !(m.ramp_time_s.is_finite() && m.ramp_time_s >= 0.0) {
            return bad("motion.ramp_time_s must be >= 0".into());
        }
        let s = &self.sim;
        if s.in_base as u32 + 256 > 4096 || s.out_base as u32 + 256 > 4096 {
            return bad("sim: in_base/out_base + 256 must stay within $IN/$OUT[1..4096]".into());
        }
        if !free_interrupt(s.interrupt_no) {
            return bad(format!(
                "sim.interrupt_no {} is not a free user priority (1, 2, 4..39, 81..128)",
                s.interrupt_no
            ));
        }
        if s.reset_timer == 0 || s.reset_timer > 64 {
            return bad("sim.reset_timer must be 1..=64".into());
        }
        let v = &self.vw_ks24;
        if v.max_interlocks == 0 || v.max_interlocks > 16 {
            return bad("vw_ks24.max_interlocks must be 1..=16".into());
        }
        if !v.step_signal.is_empty() {
            let p: Vec<&str> = v.step_signal.split(':').collect();
            let ok = p.len() == 3
                && p[0] == "out"
                && p[1].parse::<u16>().is_ok_and(|a| a >= 1 && a + 15 <= 256)
                && p[2] == "16";
            if !ok {
                return bad(format!(
                    "vw_ks24.step_signal {:?} must be \"out:<first $OUT>:16\"",
                    v.step_signal
                ));
            }
        }
        if !v.weld_call.is_empty() && !v.weld_call.is_ascii() {
            return bad("vw_ks24.weld_call must be ASCII".into());
        }
        if v.weld_call.contains('\n') || v.weld_call.contains('\r') {
            return bad("vw_ks24.weld_call must be a single line".into());
        }
        if v.makro15 > 999 {
            return bad("vw_ks24.makro15 must be 0..=999".into());
        }
        if v.release_dwell_ms < 20 || v.release_dwell_ms > 5000 {
            return bad(
                "vw_ks24.release_dwell_ms must be 20..=5000 (one PROFINET cycle plus one head                  scan, with margin)"
                    .into(),
            );
        }
        if v.plan_id_byte != 0 && !VW_FREE_IN_BYTES.contains(&v.plan_id_byte) {
            return bad(format!(
                "vw_ks24.plan_id_byte {} is used by the VW standard; free PLC -> robot bytes: {:?}",
                v.plan_id_byte, VW_FREE_IN_BYTES
            ));
        }
        let mut seen = std::collections::BTreeSet::new();
        for r in &self.robots {
            if r.ns.is_empty() {
                return bad("[[robot]] needs ns".into());
            }
            if !seen.insert(&r.ns) {
                return bad(format!("duplicate [[robot]] ns {}", r.ns));
            }
            match r.e_axes {
                Some(EAxes::Keep) => {
                    return bad(format!(
                        "robot {}: e_axes = \"keep\" is refused: that a PTP to an AXIS target \
                         leaves the external axes where they are is unverified on KSS 8.2. \
                         Use \"fixed\" with e_values, or \"none\" if the robot has no external axes",
                        r.ns
                    ));
                }
                Some(EAxes::Fixed) => match r.e_values {
                    Some(e) if e.iter().all(|x| x.is_finite()) => {}
                    _ => {
                        return bad(format!(
                            "robot {}: e_axes = \"fixed\" needs e_values (6 finite values)",
                            r.ns
                        ));
                    }
                },
                Some(EAxes::None) if r.e_values.is_some() => {
                    return bad(format!(
                        "robot {}: e_values given but e_axes = \"none\"",
                        r.ns
                    ));
                }
                _ => {}
            }
            if !(r.h_pos_tol_deg.is_finite() && r.h_pos_tol_deg >= 0.0) {
                return bad(format!("robot {}: h_pos_tol_deg must be >= 0", r.ns));
            }
            if let Some(h) = r.h_pos_deg
                && h.iter().any(|x| !x.is_finite())
            {
                return bad(format!("robot {}: h_pos_deg must be finite", r.ns));
            }
            if let Some(c) = &r.controller
                && (c.is_empty() || !c.is_ascii())
            {
                return bad(format!(
                    "robot {}: controller must be non-empty ASCII",
                    r.ns
                ));
            }
            for (z, &n) in &r.interlocks {
                if n == 0 || n > v.max_interlocks {
                    return bad(format!(
                        "robot {}: interlock {z} = {n} is outside 1..={}",
                        r.ns, v.max_interlocks
                    ));
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_round_trips_and_validates() {
        let back = Config::parse(&Config::default_toml()).unwrap();
        assert_eq!(back, Config::default());
    }

    #[test]
    fn unknown_field_is_rejected() {
        assert!(Config::parse("[motion]\nmod = \"ptp\"\n").is_err());
    }

    #[test]
    fn keep_is_refused_and_fixed_needs_values() {
        assert!(Config::parse("[[robot]]\nns = \"r\"\ne_axes = \"keep\"\n").is_err());
        assert!(Config::parse("[[robot]]\nns = \"r\"\ne_axes = \"fixed\"\n").is_err());
        Config::parse(
            "[[robot]]\nns = \"r\"\ne_axes = \"fixed\"\ne_values = [95, 0, 0, 0, 0, 0]\n",
        )
        .unwrap();
        Config::parse("[[robot]]\nns = \"r\"\ne_axes = \"none\"\n").unwrap();
    }

    #[test]
    fn time_margin_below_one_is_rejected() {
        assert!(Config::parse("[motion]\ntime_margin = 0.9\n").is_err());
    }
}
