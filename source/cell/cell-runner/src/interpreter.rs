use std::collections::HashMap;

use anyhow::Result;
use serde::Serialize;

use kuka_krl_parser::types::{IoOperation, KrlPosition, MotionContext, MotionType, MovementProgram};

use crate::path::JointPath;
use crate::traj::MotionKind;

#[derive(Debug, Clone, PartialEq)]
pub enum IoValue {
    Bool(bool),
    Int(i32),
    Float(f32),
}

#[derive(Debug)]
pub struct IoState {
    pub inputs: HashMap<u32, bool>,
    pub outputs: HashMap<u32, bool>,
    pub analog_outputs: HashMap<u32, f32>,
}

impl IoState {
    pub fn new() -> Self {
        Self {
            inputs: HashMap::new(),
            outputs: HashMap::new(),
            analog_outputs: HashMap::new(),
        }
    }
    pub fn set_output(&mut self, channel: u32, value: bool) { self.outputs.insert(channel, value); }
    pub fn get_output(&self, channel: u32) -> bool { self.outputs.get(&channel).copied().unwrap_or(false) }
    pub fn set_input(&mut self, channel: u32, value: bool) { self.inputs.insert(channel, value); }
    pub fn get_input(&self, channel: u32) -> bool { self.inputs.get(&channel).copied().unwrap_or(false) }
}

#[derive(Debug, Clone, Serialize)]
pub enum Instruction {
    Motion { motion_type: MotionType, target: String, position: KrlPosition, context: MotionContext, approximate: bool, desc: String, tool_name: String, base_name: String },
    SetOutput { channel: u32, value: bool },
    WaitForInput { channel: u32, value: bool },
    WaitTime { seconds: f64 },
    SetAnalogOutput { channel: u32, value: f32 },
    Pulse { channel: u32, value: bool, duration: f64 },
    TriggerOutput { channel: u32, value: bool },
}

impl std::fmt::Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::Motion { motion_type, target, position, context, approximate, .. } => {
                let approx = if *approximate { " C_DIS" } else { "" };
                write!(f, "{} {} ({:.1},{:.1},{:.1},{:.1},{:.1},{:.1}) S{} T{} V{} T{} B{}{}",
                    motion_type, target,
                    position.x, position.y, position.z,
                    position.a, position.b, position.c,
                    position.s, position.t,
                    context.vel_ptp, context.tool_no, context.base_no,
                    approx)
            }
            Instruction::SetOutput { channel, value } => write!(f, "$OUT[{}]={}", channel, if *value { "TRUE" } else { "FALSE" }),
            Instruction::WaitForInput { channel, value } => write!(f, "WAIT $IN[{}]=={}", channel, if *value { "TRUE" } else { "FALSE" }),
            Instruction::WaitTime { seconds } => write!(f, "WAIT SEC {}", seconds),
            Instruction::SetAnalogOutput { channel, value } => write!(f, "ANOUT[{}]={}", channel, value),
            Instruction::Pulse { channel, value, .. } => write!(f, "PULSE $OUT[{}]={}", channel, if *value { "TRUE" } else { "FALSE" }),
            Instruction::TriggerOutput { channel, value } => write!(f, "TRIGGER $OUT[{}]={}", channel, if *value { "TRUE" } else { "FALSE" }),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct MotionState {
    /// Total duration of this motion in seconds
    pub duration_s: f64,
    /// Time elapsed so far in seconds
    pub elapsed_s: f64,
    /// Pending trigger outputs to fire at specific progress fractions
    pub pending_triggers: Vec<(f64, u32, bool)>, // (progress_fraction, channel, value)
    /// Number of trigger instructions consumed after the motion instruction
    pub consumed_triggers: usize,
    /// Start position when this motion began (for interpolation)
    pub start_position: KrlPosition,
    /// Joint-space trajectory for this motion (set by the trajectory provider).
    /// When set, joint angles are sampled per micro-step via `JointPath::sample`
    /// (Dense interpolates the nearest 12 ms knots; Endpoints lerps; None yields
    /// no joints) and Cartesian position is interpolated as before.
    #[serde(skip)]
    pub joint_trajectory: Option<JointPath>,
}

#[derive(Debug, Serialize)]
pub enum StepResult {
    Executed,
    WaitingForInput { channel: u32, value: bool },
    MotionInProgress,
    ProgramComplete,
}

pub struct KrlInterpreter {
    pub io: IoState,
    pub instructions: Vec<Instruction>,
    pub pc: usize,
    pub program_name: String,
    pub motion_state: Option<MotionState>,
    pub plc_cycle_time: f64,
    pub current_position: KrlPosition,
    /// Interpolated joint angles during motion (set when joint_trajectory is available)
    pub current_joints: Option<[f64; 6]>,
    /// Counts how many cycles we've been blocked on the current WaitForInput
    pub wait_cycles: u32,
    /// After this many blocked cycles, auto-satisfy the wait (simulates external signal arrival)
    pub wait_timeout_cycles: u32,
}

fn cartesian_distance(from: &KrlPosition, to: &KrlPosition) -> f64 {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let dz = to.z - from.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute realistic PTP motion duration based on both translational and rotational travel.
/// KUKA PTP motions are joint-space moves — the slowest axis determines the total time.
/// We estimate this from Cartesian + angular distance since we don't have joint-space info.
fn compute_ptp_duration(from: &KrlPosition, to: &KrlPosition, vel_percent: f64, approximate: bool) -> f64 {
    let vel_frac = (vel_percent / 100.0).max(0.01);

    // Translational component
    let dist_mm = cartesian_distance(from, to);
    let max_tcp_speed = 2000.0; // mm/s at 100%
    let trans_time = dist_mm / (vel_frac * max_tcp_speed);

    // Rotational component — angular distance in degrees
    let da = angle_diff(from.a, to.a);
    let db = angle_diff(from.b, to.b);
    let dc = angle_diff(from.c, to.c);
    let ang_dist_deg = (da * da + db * db + dc * dc).sqrt();
    // KR240 wrist axes: ~300-400 °/s max; at vel%, limited further
    let max_rot_speed = 360.0; // °/s at 100%
    let rot_time = ang_dist_deg / (vel_frac * max_rot_speed);

    // PTP duration is the maximum of translation and rotation time
    let move_time = trans_time.max(rot_time);

    // Acceleration/deceleration time — real KUKA PTP takes 0.3-0.5s for accel+decel
    // Approximate (C_DIS) motions blend into the next but still have accel phases
    let accel_time = if approximate { 0.2 } else { 0.5 };
    // Minimum motion time: even the smallest PTP move takes at least 0.4s on a real robot
    (move_time + accel_time).max(0.4)
}

fn angle_diff(from: f64, to: f64) -> f64 {
    let mut d = to - from;
    while d > 180.0 { d -= 360.0; }
    while d < -180.0 { d += 360.0; }
    d.abs()
}

/// KR240 R2900 joint position limits from URDF (radians)
pub const JOINT_POS_LIMITS: [(f64, f64); 6] = [
    (-3.2289, 3.2289),   // J1: ±185°
    (-2.4435, -0.0873),  // J2: -140° to -5°
    (-2.0944, 2.9322),   // J3: -120° to 168°
    (-6.1087, 6.1087),   // J4: ±350°
    (-2.1380, 2.1380),   // J5: ±122.5°
    (-6.1087, 6.1087),   // J6: ±350°
];

/// Normalize wide-range joints (J1, J4, J6 with ±350° range) into [-π, π].
/// IK solvers can produce equivalent solutions outside [-π, π] that are still
/// physically valid for these revolute joints.
pub fn normalize_joints(joints: &mut [f64; 6]) {
    const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
    for i in [0, 3, 5] {
        while joints[i] > std::f64::consts::PI {
            joints[i] -= TWO_PI;
        }
        while joints[i] < -std::f64::consts::PI {
            joints[i] += TWO_PI;
        }
    }
}

/// Check if joint angles are within URDF limits (after normalization)
pub fn joints_within_limits(joints: &[f64; 6]) -> bool {
    let mut j = *joints;
    normalize_joints(&mut j);
    for i in 0..6 {
        if j[i] < JOINT_POS_LIMITS[i].0 - 0.05 || j[i] > JOINT_POS_LIMITS[i].1 + 0.05 {
            return false;
        }
    }
    true
}

/// Normalize and validate IK result. Returns normalized joints if within limits.
pub fn normalize_ik_result(joints: [f64; 6]) -> Option<[f64; 6]> {
    let mut j = joints;
    normalize_joints(&mut j);
    for i in 0..6 {
        if j[i] < JOINT_POS_LIMITS[i].0 - 0.05 || j[i] > JOINT_POS_LIMITS[i].1 + 0.05 {
            return None;
        }
    }
    Some(j)
}

/// Map a KRL motion type to the trajectory provider's [`MotionKind`].
///
/// `LIN` / `LIN_REL` are Cartesian-space moves (`MotionKind::Lin`); all other
/// KRL motion types (`PTP`, `PTP_REL`, `CIRC`, `CIRC_REL`) are resolved in
/// joint space (`MotionKind::Ptp`), which is also the safe default when the kind
/// is otherwise unknown.
pub fn motion_kind_of(motion_type: &MotionType) -> MotionKind {
    match motion_type {
        MotionType::Lin | MotionType::LinRel => MotionKind::Lin,
        _ => MotionKind::Ptp,
    }
}

/// KR240 R2900 joint velocity limits from URDF (rad/s)
const JOINT_VEL_LIMITS_RAD: [f64; 6] = [
    1.9199,  // A1: 110 °/s
    1.7453,  // A2: 100 °/s
    1.9199,  // A3: 110 °/s
    3.7525,  // A4: 215 °/s
    3.2289,  // A5: 185 °/s
    4.5379,  // A6: 260 °/s
];

/// Compute PTP duration using Pinocchio IK to get joint-space deltas.
/// Falls back to heuristic if IK fails.
pub fn compute_ptp_duration_ik(
    pin: &mut crate::pinocchio::PinocchioModel,
    from: &KrlPosition,
    to: &KrlPosition,
    vel_percent: f64,
    approximate: bool,
    seed_joints: &mut [f64; 6],
) -> f64 {
    let vel_frac = (vel_percent / 100.0).max(0.01);

    let target_from = [from.x, from.y, from.z, from.a, from.b, from.c];
    let target_to = [to.x, to.y, to.z, to.a, to.b, to.c];

    // Solve IK for start position
    let q_from = match pin.inverse_kinematics(seed_joints, &target_from, 200, 1e-3) {
        Ok(q) => q,
        Err(_) => return compute_ptp_duration(from, to, vel_percent, approximate),
    };

    // Solve IK for end position using start as seed (warm-start tracks smoothly)
    let q_to = match pin.inverse_kinematics(&q_from, &target_to, 200, 1e-3) {
        Ok(q) => q,
        Err(_) => return compute_ptp_duration(from, to, vel_percent, approximate),
    };

    // Update seed for next call
    *seed_joints = q_to;

    // Compute per-axis time: |Δq| / (vel_frac * max_vel)
    let mut max_axis_time = 0.0f64;
    for i in 0..6 {
        let dq = (q_to[i] - q_from[i]).abs();
        let axis_time = dq / (vel_frac * JOINT_VEL_LIMITS_RAD[i]);
        max_axis_time = max_axis_time.max(axis_time);
    }

    // Add accel/decel time
    let accel_time = if approximate { 0.05 } else { 0.3 };
    (max_axis_time + accel_time).max(0.1)
}

fn interpolate_position(from: &KrlPosition, to: &KrlPosition, t: f64) -> KrlPosition {
    KrlPosition {
        x: from.x + (to.x - from.x) * t,
        y: from.y + (to.y - from.y) * t,
        z: from.z + (to.z - from.z) * t,
        a: interpolate_angle(from.a, to.a, t),
        b: interpolate_angle(from.b, to.b, t),
        c: interpolate_angle(from.c, to.c, t),
        // Always use target S/T — the IK solver warm-starts from previous joints
        // and should track smoothly without flipping
        s: to.s,
        t: to.t,
    }
}

#[allow(dead_code)]
fn interpolate_joints(from: &[f64; 6], to: &[f64; 6], t: f64) -> [f64; 6] {
    let mut result = [0.0f64; 6];
    for i in 0..6 {
        result[i] = from[i] + (to[i] - from[i]) * t;
    }
    result
}

fn interpolate_angle(from: f64, to: f64, t: f64) -> f64 {
    let mut diff = to - from;
    // Shortest path on ±180° wraparound
    while diff > 180.0 { diff -= 360.0; }
    while diff < -180.0 { diff += 360.0; }
    from + diff * t
}

impl KrlInterpreter {
    pub fn from_instructions(instructions: Vec<Instruction>, name: String) -> Self {
        Self::with_state(instructions, name, IoState::new(), KrlPosition::default())
    }

    pub fn with_state(instructions: Vec<Instruction>, name: String, io: IoState, position: KrlPosition) -> Self {
        Self {
            io,
            instructions,
            pc: 0,
            program_name: name,
            motion_state: None,
            plc_cycle_time: 0.01,
            current_position: position,
            current_joints: None,
            wait_cycles: 0,
            wait_timeout_cycles: 2,
        }
    }

    pub fn is_complete(&self) -> bool { self.pc >= self.instructions.len() }

    pub fn current_instruction(&self) -> Option<&Instruction> { self.instructions.get(self.pc) }

    /// The [`MotionKind`] of the current Motion instruction (PC position), used
    /// to forward the KRL motion kind into `MotionRequest.kind`. Returns `None`
    /// when the PC is not on a `Motion` instruction.
    pub fn current_motion_kind(&self) -> Option<MotionKind> {
        match self.current_instruction() {
            Some(Instruction::Motion { motion_type, .. }) => Some(motion_kind_of(motion_type)),
            _ => None,
        }
    }

    pub fn step(&mut self) -> Result<StepResult> {
        if self.is_complete() { return Ok(StepResult::ProgramComplete); }

        // If a motion is in progress, advance its time
        if let Some(ref mut state) = self.motion_state {
            state.elapsed_s += self.plc_cycle_time;
            let progress = (state.elapsed_s / state.duration_s).min(1.0);

            // Fire any pending triggers whose progress fraction has been reached
            let mut i = 0;
            while i < state.pending_triggers.len() {
                if state.pending_triggers[i].0 <= progress {
                    let (_, channel, value) = state.pending_triggers.remove(i);
                    self.io.set_output(channel, value);
                } else {
                    i += 1;
                }
            }

            // Interpolate current_position based on progress
            if let Instruction::Motion { position, .. } = &self.instructions[self.pc] {
                // Cartesian position is always interpolated (FK-based recompute is
                // a later/server concern), preserving the base behavior for every tier.
                let start = &state.start_position.clone();
                self.current_position = interpolate_position(start, position, progress);
                // Sample joints from the path when one is present. Dense interpolates
                // the nearest 12 ms knots; Endpoints lerps; None yields no joints.
                if let Some(ref path) = state.joint_trajectory {
                    if let Some(j) = path.sample(progress) {
                        if joints_within_limits(&j) {
                            self.current_joints = Some(j);
                        }
                    }
                }
            }

            if state.elapsed_s >= state.duration_s {
                // Motion complete — fire any remaining triggers
                let remaining: Vec<(f64, u32, bool)> = state.pending_triggers.drain(..).collect();
                for (_, channel, value) in remaining {
                    self.io.set_output(channel, value);
                }
                let consumed_triggers = state.consumed_triggers;
                // Final position is exactly the target
                if let Instruction::Motion { position, .. } = &self.instructions[self.pc] {
                    self.current_position = position.clone();
                }
                // Advance PC past motion + consumed trigger instructions
                self.pc += 1 + consumed_triggers;
                self.motion_state = None;
                return Ok(StepResult::Executed);
            }
            return Ok(StepResult::MotionInProgress);
        }

        let instr = self.instructions[self.pc].clone();
        match &instr {
            Instruction::Motion { position, context, approximate, .. } => {
                self.wait_cycles = 0;
                // Start a new motion — compute duration from translation + rotation
                let duration_s = compute_ptp_duration(&self.current_position, position, context.vel_ptp, *approximate);

                // Look ahead for TriggerOutput instructions to consume
                let mut pending_triggers = Vec::new();
                let mut consumed_triggers = 0;
                let mut look_idx = self.pc + 1;
                while look_idx < self.instructions.len() {
                    if let Instruction::TriggerOutput { channel, value } = &self.instructions[look_idx] {
                        // Fire at end of motion (DISTANCE=1 → progress 1.0)
                        pending_triggers.push((1.0, *channel, *value));
                        consumed_triggers += 1;
                        look_idx += 1;
                    } else {
                        break;
                    }
                }

                self.motion_state = Some(MotionState {
                    duration_s,
                    elapsed_s: self.plc_cycle_time, // first cycle already consumed
                    pending_triggers,
                    consumed_triggers,
                    start_position: self.current_position.clone(),
                    joint_trajectory: None,
                });
                // Interpolate position for this first step
                let progress = (self.plc_cycle_time / duration_s).min(1.0);
                self.current_position = interpolate_position(&self.current_position.clone(), position, progress);
                Ok(StepResult::MotionInProgress)
            }
            Instruction::SetOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; Ok(StepResult::Executed) }
            Instruction::WaitForInput { channel, value } => {
                if self.io.get_input(*channel) == *value {
                    self.wait_cycles = 0;
                    self.pc += 1;
                    Ok(StepResult::Executed)
                } else {
                    self.wait_cycles += 1;
                    if self.wait_cycles >= self.wait_timeout_cycles {
                        // Timeout: simulate external signal arrival
                        self.io.set_input(*channel, *value);
                        self.wait_cycles = 0;
                        self.pc += 1;
                        Ok(StepResult::Executed)
                    } else {
                        Ok(StepResult::WaitingForInput { channel: *channel, value: *value })
                    }
                }
            }
            Instruction::WaitTime { .. } => { self.pc += 1; Ok(StepResult::Executed) }
            Instruction::SetAnalogOutput { .. } => { self.pc += 1; Ok(StepResult::Executed) }
            Instruction::Pulse { channel, value, .. } => { self.io.set_output(*channel, *value); self.io.set_output(*channel, !*value); self.pc += 1; Ok(StepResult::Executed) }
            Instruction::TriggerOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; Ok(StepResult::Executed) }
        }
    }

    /// Step to the next motion instruction (PTP/LIN/CIRC).
    /// If a motion is in progress, waits for the background to complete it (returns MotionInProgress).
    /// If no motion is in progress, executes non-motion instructions and STARTS the next motion.
    pub fn step_to_motion(&mut self) -> Result<StepResult> {
        if self.is_complete() { return Ok(StepResult::ProgramComplete); }

        // If a motion is already in progress, DON'T complete it — let background animate it
        if self.motion_state.is_some() {
            return Ok(StepResult::MotionInProgress);
        }

        // Execute non-motion instructions until we land on the next motion (or hit a block/end)
        for _ in 0..10000 {
            if self.is_complete() {
                return Ok(StepResult::ProgramComplete);
            }
            match &self.instructions[self.pc] {
                Instruction::Motion { position, context, approximate, .. } => {
                    // START the motion (don't complete it) — let the render loop animate
                    let duration_s = compute_ptp_duration(&self.current_position, position, context.vel_ptp, *approximate);

                    // Look ahead for TriggerOutput instructions to consume
                    let mut pending_triggers = Vec::new();
                    let mut consumed_triggers = 0;
                    let mut look_idx = self.pc + 1;
                    while look_idx < self.instructions.len() {
                        if let Instruction::TriggerOutput { channel, value } = &self.instructions[look_idx] {
                            pending_triggers.push((1.0, *channel, *value));
                            consumed_triggers += 1;
                            look_idx += 1;
                        } else {
                            break;
                        }
                    }

                    self.motion_state = Some(MotionState {
                        duration_s,
                        elapsed_s: 0.0,
                        pending_triggers,
                        consumed_triggers,
                        start_position: self.current_position.clone(),
                        joint_trajectory: None,
                    });
                    return Ok(StepResult::MotionInProgress);
                }
                Instruction::SetOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; }
                Instruction::WaitForInput { channel, value } => {
                    if self.io.get_input(*channel) == *value {
                        self.pc += 1;
                    } else {
                        return Ok(StepResult::WaitingForInput { channel: *channel, value: *value });
                    }
                }
                Instruction::WaitTime { .. } => { self.pc += 1; }
                Instruction::SetAnalogOutput { .. } => { self.pc += 1; }
                Instruction::Pulse { channel, value, .. } => { self.io.set_output(*channel, *value); self.io.set_output(*channel, !*value); self.pc += 1; }
                Instruction::TriggerOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; }
            }
        }
        Ok(StepResult::Executed)
    }

    /// Step to the next WAIT or block. Executes all instructions (including motions) instantly.
    pub fn step_to_wait(&mut self) -> Result<StepResult> {
        if self.is_complete() { return Ok(StepResult::ProgramComplete); }

        // If a motion is in progress, complete it
        if self.motion_state.is_some() {
            if let Instruction::Motion { position, .. } = &self.instructions[self.pc] {
                self.current_position = position.clone();
            }
            let consumed = self.motion_state.as_ref().map(|s| s.consumed_triggers).unwrap_or(0);
            self.motion_state = None;
            self.pc += 1 + consumed;
        }

        // Execute instructions until we hit a blocking WAIT or end
        for _ in 0..10000 {
            if self.is_complete() { return Ok(StepResult::ProgramComplete); }
            match &self.instructions[self.pc] {
                Instruction::Motion { position, .. } => {
                    self.current_position = position.clone();
                    // Skip over any consumed triggers
                    self.pc += 1;
                    while self.pc < self.instructions.len() {
                        if let Instruction::TriggerOutput { channel, value } = &self.instructions[self.pc] {
                            self.io.set_output(*channel, *value);
                            self.pc += 1;
                        } else {
                            break;
                        }
                    }
                }
                Instruction::SetOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; }
                Instruction::WaitForInput { channel, value } => {
                    if self.io.get_input(*channel) == *value {
                        self.pc += 1;
                        return Ok(StepResult::Executed);
                    } else {
                        return Ok(StepResult::WaitingForInput { channel: *channel, value: *value });
                    }
                }
                Instruction::WaitTime { .. } => { self.pc += 1; return Ok(StepResult::Executed); }
                Instruction::SetAnalogOutput { .. } => { self.pc += 1; }
                Instruction::Pulse { channel, value, .. } => { self.io.set_output(*channel, *value); self.io.set_output(*channel, !*value); self.pc += 1; }
                Instruction::TriggerOutput { channel, value } => { self.io.set_output(*channel, *value); self.pc += 1; }
            }
        }
        Ok(StepResult::Executed)
    }

    /// Returns the motion progress as a fraction (0.0 to 1.0), or None if no motion is active
    pub fn motion_progress(&self) -> Option<f64> {
        self.motion_state.as_ref().map(|s| (s.elapsed_s / s.duration_s).min(1.0))
    }

    /// Override the duration of the current in-progress motion (called by server when Pinocchio is available)
    pub fn set_motion_duration(&mut self, duration_s: f64) {
        if let Some(ref mut state) = self.motion_state {
            state.duration_s = duration_s.max(0.1);
        }
    }
}

pub fn build_instruction_stream(program: &MovementProgram) -> Vec<Instruction> {
    let mut instructions = Vec::new();
    let mut io_index = 0;
    let io_positions = compute_io_positions(program);

    for (seq_idx, point_name) in program.sequence.iter().enumerate() {
        while io_index < program.ios.len() {
            if let Some(&insert_before) = io_positions.get(io_index) {
                if insert_before <= seq_idx {
                    if let Some(instr) = io_to_instruction(&program.ios[io_index]) { instructions.push(instr); }
                    io_index += 1;
                } else { break; }
            } else { break; }
        }
        if let Some(point) = program.points.get(point_name) {
            let motion_type = match point.params.motion.to_uppercase().as_str() {
                "PTP" => MotionType::Ptp, "LIN" => MotionType::Lin, "CIRC" => MotionType::Circ,
                "PTP_REL" => MotionType::PtpRel, "LIN_REL" => MotionType::LinRel, "CIRC_REL" => MotionType::CircRel,
                _ => MotionType::Ptp,
            };
            let tool_name = program.frames.tools.get(&point.params.tool)
                .map(|f| f.name.clone()).unwrap_or_default();
            let base_name = program.frames.bases.get(&point.params.base)
                .map(|f| f.name.clone()).unwrap_or_default();

            // Transform position from KRL base frame to robot base frame (base=0)
            let (world_xyz, world_abc) = transform_to_world(
                point.coords, point.ori, point.params.base, &program.frames,
            );

            // Remove tool offset: TCP → flange position (Pinocchio targets the flange frame)
            // Skip for tools > 1m — these are external fixtures where tool removal
            // produces unreachable flange positions
            let tool_length = program.frames.tools.get(&point.params.tool)
                .map(|t| (t.x*t.x + t.y*t.y + t.z*t.z).sqrt())
                .unwrap_or(0.0);
            let (mut world_xyz, world_abc) = if tool_length < 1000.0 {
                remove_tool_offset(world_xyz, world_abc, point.params.tool, &program.frames)
            } else {
                (world_xyz, world_abc)
            };

            // Clamp to robot reach (2900mm) — positions beyond reach get scaled
            // to max reach distance while preserving direction. This handles robots
            // with external tools/fixtures where the KRL coordinates exceed arm reach.
            let dist = (world_xyz[0]*world_xyz[0] + world_xyz[1]*world_xyz[1] + world_xyz[2]*world_xyz[2]).sqrt();
            if dist > 2800.0 {
                let scale = 2800.0 / dist;
                world_xyz[0] *= scale;
                world_xyz[1] *= scale;
                world_xyz[2] *= scale;
            }

            instructions.push(Instruction::Motion {
                motion_type, target: point_name.clone(),
                position: KrlPosition { x: world_xyz[0], y: world_xyz[1], z: world_xyz[2], a: world_abc[0], b: world_abc[1], c: world_abc[2], s: point.params.s, t: point.params.t },
                context: MotionContext { tool_no: point.params.tool, base_no: point.params.base, vel_ptp: point.params.vb, vel_cp: point.params.vb },
                approximate: point.params.ve > 0.0,
                desc: point.desc.clone(),
                tool_name,
                base_name,
            });
        }
    }
    while io_index < program.ios.len() {
        if let Some(instr) = io_to_instruction(&program.ios[io_index]) { instructions.push(instr); }
        io_index += 1;
    }
    instructions
}

/// Transform a point from a KUKA base frame to the robot's world frame (base=0).
/// KUKA convention: rotation = Rz(A) * Ry(B) * Rx(C)
fn transform_to_world(
    coords: [f64; 3],
    ori: [f64; 3],
    base_no: u32,
    frames: &kuka_krl_parser::types::Frames,
) -> ([f64; 3], [f64; 3]) {
    if base_no == 0 {
        return (coords, ori);
    }
    let base = match frames.bases.get(&base_no) {
        Some(b) => b,
        None => return (coords, ori),
    };

    // Base frame rotation matrix: Rz(A) * Ry(B) * Rx(C)
    let r_base = kuka_abc_to_mat3(base.a, base.b, base.c);
    let t_base = [base.x, base.y, base.z];

    // Point in base frame
    let p = [coords[0], coords[1], coords[2]];

    // Transform position: world_pos = R_base * p + t_base
    let world_pos = [
        r_base[0][0] * p[0] + r_base[0][1] * p[1] + r_base[0][2] * p[2] + t_base[0],
        r_base[1][0] * p[0] + r_base[1][1] * p[1] + r_base[1][2] * p[2] + t_base[1],
        r_base[2][0] * p[0] + r_base[2][1] * p[1] + r_base[2][2] * p[2] + t_base[2],
    ];

    // If transformed Z goes significantly below ground, the base frame describes
    // a tilted coordinate system where the robot reaches a workpiece from above.
    // For visualization, clamp Z to a minimum to prevent underground rendering.
    let world_pos = if world_pos[2] < -200.0 {
        [world_pos[0], world_pos[1], -200.0]
    } else {
        world_pos
    };

    // Transform orientation: world_R = R_base * R_point
    let r_point = kuka_abc_to_mat3(ori[0], ori[1], ori[2]);
    let r_world = mat3_multiply(&r_base, &r_point);
    let world_abc = mat3_to_kuka_abc(&r_world);

    (world_pos, world_abc)
}

/// Public version for use from main.rs with system frames
pub fn remove_tool_offset_ext(
    tcp_xyz: [f64; 3],
    tcp_abc: [f64; 3],
    tool_no: u32,
    frames: &kuka_krl_parser::types::Frames,
) -> ([f64; 3], [f64; 3]) {
    remove_tool_offset(tcp_xyz, tcp_abc, tool_no, frames)
}

/// Remove tool offset from TCP position to get flange position.
/// KUKA convention: TCP_pose = Flange_pose * Tool_frame
/// Therefore: Flange_pose = TCP_pose * Tool_frame^(-1)
///
/// In terms of position/rotation:
///   R_tcp = R_flange * R_tool → R_flange = R_tcp * R_tool^T
///   P_tcp = P_flange + R_flange * tool_xyz → P_flange = P_tcp - R_flange * tool_xyz
fn remove_tool_offset(
    tcp_xyz: [f64; 3],
    tcp_abc: [f64; 3],
    tool_no: u32,
    frames: &kuka_krl_parser::types::Frames,
) -> ([f64; 3], [f64; 3]) {
    let tool = match frames.tools.get(&tool_no) {
        Some(t) => t,
        None => return (tcp_xyz, tcp_abc),
    };
    if tool.x.abs() < 0.1 && tool.y.abs() < 0.1 && tool.z.abs() < 0.1
        && tool.a.abs() < 0.1 && tool.b.abs() < 0.1 && tool.c.abs() < 0.1 {
        return (tcp_xyz, tcp_abc);
    }



    // Compute R_flange = R_tcp * R_tool^T
    let r_tcp = kuka_abc_to_mat3(tcp_abc[0], tcp_abc[1], tcp_abc[2]);
    let r_tool = kuka_abc_to_mat3(tool.a, tool.b, tool.c);
    let r_tool_inv = transpose3(&r_tool);
    let r_flange = mat3_multiply(&r_tcp, &r_tool_inv);

    // Flange position = TCP position - R_flange * tool_offset
    let tool_vec = [tool.x, tool.y, tool.z];
    let flange_xyz = [
        tcp_xyz[0] - (r_flange[0][0] * tool_vec[0] + r_flange[0][1] * tool_vec[1] + r_flange[0][2] * tool_vec[2]),
        tcp_xyz[1] - (r_flange[1][0] * tool_vec[0] + r_flange[1][1] * tool_vec[1] + r_flange[1][2] * tool_vec[2]),
        tcp_xyz[2] - (r_flange[2][0] * tool_vec[0] + r_flange[2][1] * tool_vec[1] + r_flange[2][2] * tool_vec[2]),
    ];

    // Flange orientation
    let flange_abc = mat3_to_kuka_abc(&r_flange);

    (flange_xyz, flange_abc)
}

fn transpose3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn kuka_abc_to_mat3(a_deg: f64, b_deg: f64, c_deg: f64) -> [[f64; 3]; 3] {
    let a = a_deg.to_radians();
    let b = b_deg.to_radians();
    let c = c_deg.to_radians();
    let (sa, ca) = a.sin_cos();
    let (sb, cb) = b.sin_cos();
    let (sc, cc) = c.sin_cos();
    // Rz(A) * Ry(B) * Rx(C)
    [
        [ca*cb, ca*sb*sc - sa*cc, ca*sb*cc + sa*sc],
        [sa*cb, sa*sb*sc + ca*cc, sa*sb*cc - ca*sc],
        [-sb,   cb*sc,            cb*cc           ],
    ]
}

fn mat3_multiply(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut c = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j];
        }
    }
    c
}

fn mat3_to_kuka_abc(r: &[[f64; 3]; 3]) -> [f64; 3] {
    // Extract A, B, C from rotation matrix (Rz(A)*Ry(B)*Rx(C))
    let b = (-r[2][0]).asin();
    let (a, c) = if b.cos().abs() > 1e-6 {
        let a = r[1][0].atan2(r[0][0]);
        let c = r[2][1].atan2(r[2][2]);
        (a, c)
    } else {
        // Gimbal lock
        let a = (-r[0][1]).atan2(r[1][1]);
        (a, 0.0)
    };
    [a.to_degrees(), b.to_degrees(), c.to_degrees()]
}

fn io_to_instruction(io: &IoOperation) -> Option<Instruction> {
    match io.io_type.as_str() {
        "OUT" => {
            let value = io.action.contains("TRUE");
            if io.action.starts_with("trigger") { Some(Instruction::TriggerOutput { channel: io.channel, value }) }
            else if io.action.starts_with("pulse") { Some(Instruction::Pulse { channel: io.channel, value, duration: 0.5 }) }
            else { Some(Instruction::SetOutput { channel: io.channel, value }) }
        }
        "IN" => {
            if io.action.starts_with("wait") { Some(Instruction::WaitForInput { channel: io.channel, value: io.action.contains("TRUE") }) }
            else { None }
        }
        "ANOUT" => Some(Instruction::SetAnalogOutput { channel: io.channel, value: 0.0 }),
        _ => None,
    }
}

fn compute_io_positions(program: &MovementProgram) -> Vec<usize> {
    let num_points = program.sequence.len();
    let num_ios = program.ios.len();
    if num_ios == 0 || num_points == 0 { return vec![]; }
    let step = if num_ios > 1 { (num_points - 1).max(1) as f64 / num_ios as f64 } else { 1.0 };
    (0..num_ios).map(|i| (1.0 + step * i as f64).min(num_points as f64) as usize).collect()
}
