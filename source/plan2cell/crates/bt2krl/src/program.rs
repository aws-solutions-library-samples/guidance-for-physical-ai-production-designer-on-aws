//! The bt2plc output this crate consumes: `robots/<ns>/program.json`
//! (`bt2plc/robot-program@1`, `schema_minor >= 1`) and the plan id of `cell_plan.json`.
//!
//! Unknown fields are ignored (minor versions only add fields); everything bt2krl relies on is
//! checked in [`Program::check`].

use std::path::Path;

use serde::Deserialize;

use crate::error::{Error, Result};

pub const SCHEMA: &str = "bt2plc/robot-program@1";
pub const MIN_SCHEMA_MINOR: u32 = 1;

#[derive(Clone, Debug, Deserialize)]
pub struct Program {
    pub schema: String,
    #[serde(default)]
    pub schema_minor: u32,
    pub plan: PlanRef,
    pub robot: RobotRef,
    pub folgen: Folgen,
    pub home_pose_deg: [f64; 6],
    pub joint_mapping: JointMapping,
    pub protocol: Protocol,
    #[serde(default)]
    pub zones: Vec<ZoneRef>,
    pub ops: Vec<Op>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct PlanRef {
    pub tree_id: String,
    pub cell_id: String,
    /// `"16#AB3A"`
    pub plan_id: String,
}

#[derive(Clone, Debug, Deserialize)]
pub struct RobotRef {
    pub ns: String,
    pub tag: String,
    pub index: usize,
    pub station: String,
    pub ip: String,
    #[serde(default = "unknown")]
    pub model: String,
}

fn unknown() -> String {
    "unknown".into()
}

#[derive(Clone, Debug, Deserialize)]
pub struct Folgen {
    pub production: u16,
    pub home: u16,
}

#[derive(Clone, Debug, Deserialize)]
pub struct JointMapping {
    #[serde(default)]
    pub preset: Option<String>,
    pub sign: [i8; 6],
    pub offset_deg: [f64; 6],
}

impl JointMapping {
    pub fn describe(&self) -> String {
        match self.preset.as_deref() {
            Some(p) => p.to_string(),
            None if self.sign == [1; 6] && self.offset_deg == [0.0; 6] => "identity".into(),
            None => "custom".into(),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct Field {
    pub byte: u8,
    pub len: u8,
}

#[derive(Clone, Debug, Deserialize)]
pub struct ErrorCodes {
    pub unknown_folge: u16,
    pub plan_mismatch: u16,
    pub start_deviation: u16,
}

#[derive(Clone, Debug, Deserialize)]
pub struct Protocol {
    pub step: Field,
    pub request: Field,
    pub grant: Field,
    pub plan_id: Field,
    pub permit_bit: u8,
    pub grant_policy: String,
    pub profile: String,
    /// `"word_le"` | `"byte"`
    pub job_number: String,
    pub error_codes: ErrorCodes,
    pub at_home_tol_deg: f64,
    pub start_tol_deg: f64,
    pub home_return_time_s: f64,
    pub handshake_s: f64,
    pub abort_limit_s: f64,
    pub home_limit_s: f64,
}

impl Protocol {
    pub fn job_number_byte(&self) -> bool {
        self.job_number == "byte"
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct ZoneRef {
    pub name: String,
    pub bit: u8,
    /// Profile `vw-ks24`: the VW interlock index of this zone for this robot (1..16).
    #[serde(default)]
    pub interlock: Option<u8>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum OpKind {
    Execute { operation_type: String },
    MoveHome,
}

#[derive(Clone, Debug, Deserialize)]
pub struct Op {
    pub index: u16,
    pub operation_id: String,
    pub kind: OpKind,
    #[serde(default)]
    pub bt_path: String,
    #[serde(default)]
    pub zones: Vec<String>,
    /// `"16#XXXXXXXX"`, includes the permit bit.
    pub zone_mask: String,
    #[serde(default)]
    pub waits_for: Vec<String>,
    pub start_delay_s: f64,
    pub hold_s: f64,
    pub velocity_scale: f64,
    pub duration_s: f64,
    pub expected_s: f64,
    pub limit_s: f64,
    pub delay_exact: bool,
    pub blind_home: bool,
    #[serde(default)]
    pub trajectory_key: Option<String>,
    #[serde(default)]
    pub waypoints: Vec<Waypoint>,
}

impl Op {
    pub fn mask(&self) -> u32 {
        parse_hex(&self.zone_mask).unwrap_or(0) as u32
    }

    pub fn operation_type(&self) -> &str {
        match &self.kind {
            OpKind::Execute { operation_type } => operation_type,
            OpKind::MoveHome => "move_home",
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct Waypoint {
    pub t_s: f64,
    pub q_deg: [f64; 6],
    #[serde(default = "transit")]
    pub phase: String,
}

fn transit() -> String {
    "transit".into()
}

/// `"16#AB3A"` -> 0xAB3A.
pub fn parse_hex(s: &str) -> Option<u64> {
    u64::from_str_radix(s.strip_prefix("16#")?, 16).ok()
}

impl Program {
    pub fn from_str(text: &str, origin: &str) -> Result<Program> {
        let p: Program = serde_json::from_str(text)
            .map_err(|e| Error::Input(format!("{origin}: not a bt2plc program.json: {e}")))?;
        p.check(origin)?;
        Ok(p)
    }

    pub fn load(path: &Path) -> Result<Program> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        Program::from_str(&text, &path.display().to_string())
    }

    pub fn plan_id(&self) -> u16 {
        parse_hex(&self.plan.plan_id).unwrap_or(0) as u16
    }

    /// Structural checks of everything the generator relies on.
    pub fn check(&self, origin: &str) -> Result<()> {
        let bad = |m: String| Err(Error::Input(format!("{origin}: {m}")));
        if self.schema != SCHEMA {
            return bad(format!("schema {} is not {SCHEMA}", self.schema));
        }
        if self.schema_minor < MIN_SCHEMA_MINOR {
            return bad(format!(
                "schema_minor {} < {MIN_SCHEMA_MINOR}: regenerate with the current bt2plc",
                self.schema_minor
            ));
        }
        match parse_hex(&self.plan.plan_id) {
            Some(v) if v > 0 && v <= 0xFFFF => {}
            _ => {
                return bad(format!(
                    "plan_id {} is not a non-zero WORD",
                    self.plan.plan_id
                ));
            }
        }
        let pr = &self.protocol;
        for (what, f, len) in [
            ("step", &pr.step, 2),
            ("request", &pr.request, 4),
            ("grant", &pr.grant, 4),
            ("plan_id", &pr.plan_id, 2),
        ] {
            if f.len != len || f.byte as usize + len as usize > 32 {
                return bad(format!(
                    "protocol.{what} must be {len} bytes inside the 32-byte telegram"
                ));
            }
        }
        if pr.permit_bit != 31 {
            return bad(format!("permit_bit {} is not 31", pr.permit_bit));
        }
        if !matches!(pr.job_number.as_str(), "word_le" | "byte") {
            return bad(format!("unknown job_number mode {}", pr.job_number));
        }
        if self.folgen.production == 0
            || self.folgen.home == 0
            || self.folgen.production == self.folgen.home
        {
            return bad("Folge numbers must be > 0 and differ".into());
        }
        let zone_bit = |name: &str| self.zones.iter().find(|z| z.name == name).map(|z| z.bit);
        for (k, op) in self.ops.iter().enumerate() {
            let at = format!("op {} ({})", k + 1, op.operation_id);
            if op.index as usize != k + 1 {
                return bad(format!("{at}: index {} is not {}", op.index, k + 1));
            }
            let Some(mask) = parse_hex(&op.zone_mask).filter(|&m| m <= u32::MAX as u64) else {
                return bad(format!("{at}: zone_mask {} is not a DWORD", op.zone_mask));
            };
            let mut want = mask as u32 & !(1u32 << 31);
            for z in &op.zones {
                let Some(b) = zone_bit(z) else {
                    return bad(format!("{at}: zone {z} is not in zones[]"));
                };
                if b > 30 || want & (1 << b) == 0 {
                    return bad(format!("{at}: zone {z} (bit {b}) not in zone_mask"));
                }
                want &= !(1 << b);
            }
            if want != 0 {
                return bad(format!("{at}: zone_mask has bits without a zone"));
            }
            for (what, v) in [
                ("start_delay_s", op.start_delay_s),
                ("hold_s", op.hold_s),
                ("expected_s", op.expected_s),
                ("limit_s", op.limit_s),
            ] {
                if !v.is_finite() || v < 0.0 {
                    return bad(format!("{at}: {what} {v} must be finite and >= 0"));
                }
            }
            if !(op.velocity_scale.is_finite() && op.velocity_scale > 0.0) {
                return bad(format!("{at}: velocity_scale must be > 0"));
            }
            if op.blind_home != op.waypoints.is_empty() {
                return bad(format!(
                    "{at}: blind_home must be set exactly when there are no waypoints"
                ));
            }
            let mut last_t = f64::NEG_INFINITY;
            for (i, w) in op.waypoints.iter().enumerate() {
                if !(w.t_s.is_finite() && w.t_s >= last_t) || w.q_deg.iter().any(|q| !q.is_finite())
                {
                    return bad(format!(
                        "{at}: waypoint {i} has a non-finite value or goes back in time"
                    ));
                }
                last_t = w.t_s;
            }
        }
        if self.home_pose_deg.iter().any(|q| !q.is_finite()) {
            return bad("home_pose_deg is not finite".into());
        }
        Ok(())
    }
}

/// `cell_plan.json` (only what is cross-checked).
#[derive(Clone, Debug, Deserialize)]
pub struct CellPlanRef {
    pub source: CellPlanSource,
}

#[derive(Clone, Debug, Deserialize)]
pub struct CellPlanSource {
    pub plan_id: String,
}
