use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Serialize)]
pub struct Frame {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub name: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct Frames {
    pub bases: BTreeMap<u32, Frame>,
    pub tools: BTreeMap<u32, Frame>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MotionParams {
    pub tool: u32,
    pub base: u32,
    pub vb: f64,
    pub ve: f64,
    pub motion: String,
    pub s: i32,
    pub t: i32,
}

#[derive(Debug, Clone, Serialize)]
pub struct Point {
    pub coords: [f64; 3],
    pub ori: [f64; 3],
    pub params: MotionParams,
    pub desc: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct IoOperation {
    pub io_type: String,
    pub channel: u32,
    pub action: String,
    pub signal_name: Option<String>,
    pub context: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct MovementProgram {
    pub program_name: String,
    pub frames: Frames,
    pub points: BTreeMap<String, Point>,
    pub sequence: Vec<String>,
    pub ios: Vec<IoOperation>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct KrlPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub s: i32,
    pub t: i32,
}

#[derive(Debug, Clone)]
pub struct MotionCommand {
    pub motion_type: MotionType,
    pub target: String,
    pub approximate: bool,
    pub context: MotionContext,
    pub pdat_name: Option<String>,
    pub fdat_name: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct PdatEntry {
    pub vel: f64,
    pub acc: f64,
    pub apo_dist: f64,
}

#[derive(Debug, Clone, Default)]
pub struct FdatEntry {
    pub tool_no: u32,
    pub base_no: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub enum MotionType {
    Ptp,
    PtpRel,
    Lin,
    LinRel,
    Circ,
    CircRel,
}

impl std::fmt::Display for MotionType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MotionType::Ptp => write!(f, "PTP"),
            MotionType::PtpRel => write!(f, "PTP_REL"),
            MotionType::Lin => write!(f, "LIN"),
            MotionType::LinRel => write!(f, "LIN_REL"),
            MotionType::Circ => write!(f, "CIRC"),
            MotionType::CircRel => write!(f, "CIRC_REL"),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct MotionContext {
    pub tool_no: u32,
    pub base_no: u32,
    pub vel_ptp: f64,
    pub vel_cp: f64,
}
