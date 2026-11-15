pub use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Serialize;

use serde_json::Value;

use crate::interpreter::{build_instruction_stream, Instruction, KrlInterpreter, StepResult as KrlStepResult};

#[derive(Debug, Clone, PartialEq)]
pub enum StepStatus {
    Executed,
    MotionInProgress,
    Blocked,
    Done,
}

#[derive(Serialize)]
pub struct KrlState {
    pub robot_id: String,
    pub active_folge: u8,
    pub pc: usize,
    pub total: usize,
    pub complete: bool,
    pub waiting_for: Option<String>,
    pub last_instruction: Option<String>,
    pub motion_progress: Option<f64>,
}

pub struct RobotKrl {
    pub robot_id: String,
    pub programs: HashMap<u8, Vec<Instruction>>,
    pub interpreter: Option<KrlInterpreter>,
    pub active_folge: u8,
    pub program_complete: bool,
    /// Persistent IO process image — survives across program changes
    pub io: crate::interpreter::IoState,
    /// Persistent robot position — survives across program changes
    pub position: kuka_krl_parser::types::KrlPosition,
    /// Pre-solved joint angles for current position (for visualization)
    pub home_joints: Option<[f64; 6]>,
}

impl RobotKrl {
    pub fn load(robot_id: &str, krl_source: &Path) -> Result<Self> {
        let robot_dir = krl_source.join(robot_id).join("KRC").join("R1").join("Folgen");
        let mut programs: HashMap<u8, Vec<Instruction>> = HashMap::new();

        if robot_dir.exists() {
            // Read system $config.dat for BASE_DATA / TOOL_DATA
            let config_dat_path = krl_source.join(robot_id).join("KRC").join("R1").join("System").join("$config.dat");
            let system_frames = if config_dat_path.exists() {
                let content = std::fs::read_to_string(&config_dat_path).unwrap_or_default();
                kuka_krl_parser::parser::parse_system_frames(&content)
            } else {
                kuka_krl_parser::types::Frames {
                    bases: std::collections::BTreeMap::new(),
                    tools: std::collections::BTreeMap::new(),
                }
            };

            let mut parsed = kuka_krl_parser::parser::parse_krl_project(&robot_dir)
                .with_context(|| format!("parsing KRL for {}", robot_id))?;

            // Merge system frames into each program's frames (system provides defaults)
            for prog in &mut parsed {
                for (idx, frame) in &system_frames.bases {
                    prog.frames.bases.entry(*idx).or_insert_with(|| frame.clone());
                }
                for (idx, frame) in &system_frames.tools {
                    prog.frames.tools.entry(*idx).or_insert_with(|| frame.clone());
                }
            }

            for prog in &parsed {
                let name_lower = prog.program_name.to_lowercase();
                if let Some(num_str) = name_lower.strip_prefix("folge") {
                    if let Ok(num) = num_str.parse::<u8>() {
                        let instructions = build_instruction_stream(prog);
                        if !instructions.is_empty() {
                            programs.insert(num, instructions);
                        }
                    }
                }
            }
        }

        Ok(Self {
            robot_id: robot_id.to_string(),
            programs,
            interpreter: None,
            active_folge: 0,
            program_complete: false,
            io: crate::interpreter::IoState::new(),
            position: kuka_krl_parser::types::KrlPosition::default(),
            home_joints: None,
        })
    }

    pub fn start_program(&mut self, folge: u8) {
        self.active_folge = folge;
        self.program_complete = false;

        // Save current IO, position, and JOINT configuration from the running
        // interpreter before replacing it. Carrying the joints forward keeps the
        // IK seed continuous across folge boundaries so the wrist branch does not
        // flip when one folge hands off to the next (e.g. folge11 -> folge125).
        let prev_joints = self.interpreter.as_ref().and_then(|i| i.current_joints);
        if let Some(ref interp) = self.interpreter {
            self.io = crate::interpreter::IoState {
                inputs: interp.io.inputs.clone(),
                outputs: interp.io.outputs.clone(),
                analog_outputs: interp.io.analog_outputs.clone(),
            };
            self.position = interp.current_position.clone();
        }

        if let Some(instructions) = self.programs.get(&folge) {
            // Create new interpreter with persistent IO state and position
            self.interpreter = Some(KrlInterpreter::with_state(
                instructions.clone(),
                format!("Folge{}", folge),
                crate::interpreter::IoState {
                    inputs: self.io.inputs.clone(),
                    outputs: self.io.outputs.clone(),
                    analog_outputs: self.io.analog_outputs.clone(),
                },
                self.position.clone(),
            ));
            if let Some(ref mut interp) = self.interpreter {
                // Seed from the previous folge's final joints (continuity);
                // fall back to home joints only on the very first load.
                interp.current_joints = prev_joints.or(self.home_joints);
            }
        } else {
            self.interpreter = None;
            self.program_complete = true;
        }
    }

    /// Execute one step. Returns a status indicating what happened.
    pub fn step(&mut self, auto_satisfy: bool) -> StepStatus {
        let interp = match &mut self.interpreter {
            Some(i) => i,
            None => return StepStatus::Done,
        };

        if interp.is_complete() {
            self.program_complete = true;
            return StepStatus::Done;
        }

        match interp.step() {
            Ok(KrlStepResult::Executed) => StepStatus::Executed,
            Ok(KrlStepResult::MotionInProgress) => StepStatus::MotionInProgress,
            Ok(KrlStepResult::WaitingForInput { channel, value }) => {
                if auto_satisfy {
                    interp.io.set_input(channel, value);
                    // Step again to pass through the now-satisfied wait
                    let _ = interp.step();
                    StepStatus::Executed
                } else {
                    StepStatus::Blocked
                }
            }
            Ok(KrlStepResult::ProgramComplete) => {
                self.program_complete = true;
                StepStatus::Done
            }
            Err(_) => StepStatus::Done,
        }
    }

    /// Step to the next motion instruction (PTP/LIN/CIRC).
    pub fn step_to_motion(&mut self) -> StepStatus {
        let interp = match &mut self.interpreter {
            Some(i) => i,
            None => return StepStatus::Done,
        };
        if interp.is_complete() { self.program_complete = true; return StepStatus::Done; }
        match interp.step_to_motion() {
            Ok(KrlStepResult::Executed) => StepStatus::Executed,
            Ok(KrlStepResult::MotionInProgress) => StepStatus::MotionInProgress,
            Ok(KrlStepResult::WaitingForInput { .. }) => StepStatus::Blocked,
            Ok(KrlStepResult::ProgramComplete) => { self.program_complete = true; StepStatus::Done }
            _ => StepStatus::Done,
        }
    }

    /// Step to the next WAIT instruction.
    pub fn step_to_wait(&mut self) -> StepStatus {
        let interp = match &mut self.interpreter {
            Some(i) => i,
            None => return StepStatus::Done,
        };
        if interp.is_complete() { self.program_complete = true; return StepStatus::Done; }
        match interp.step_to_wait() {
            Ok(KrlStepResult::Executed) => StepStatus::Executed,
            Ok(KrlStepResult::WaitingForInput { .. }) => StepStatus::Blocked,
            Ok(KrlStepResult::ProgramComplete) => { self.program_complete = true; StepStatus::Done }
            _ => StepStatus::Done,
        }
    }

    pub fn run_to_completion(&mut self, max_steps: usize) {
        for _ in 0..max_steps {
            if self.program_complete {
                break;
            }
            match self.step(true) {
                StepStatus::Done => break,
                _ => continue,
            }
        }
    }

    pub fn state(&self) -> KrlState {
        let (pc, total, complete, waiting, last_instr, motion_progress) = if let Some(ref interp) = self.interpreter {
            let waiting = if interp.motion_state.is_some() {
                let progress = interp.motion_progress().unwrap_or(0.0);
                if let Some(Instruction::Motion { target, desc, .. }) = interp.current_instruction() {
                    Some(format!("MOTION {} {:.0}% ({})", target, progress * 100.0, desc))
                } else {
                    Some(format!("MOTION {:.0}%", progress * 100.0))
                }
            } else if !interp.is_complete() {
                match interp.current_instruction() {
                    Some(Instruction::WaitForInput { channel, value }) => {
                        Some(format!("$IN[{}]=={}", channel, if *value { "TRUE" } else { "FALSE" }))
                    }
                    _ => None,
                }
            } else {
                None
            };
            let last = interp.current_instruction().map(|i| format!("{}", i));
            let progress = interp.motion_progress();
            (interp.pc, interp.instructions.len(), interp.is_complete(), waiting, last, progress)
        } else {
            (0, 0, self.program_complete, None, None, None)
        };

        KrlState {
            robot_id: self.robot_id.clone(),
            active_folge: self.active_folge,
            pc,
            total,
            complete,
            waiting_for: waiting,
            last_instruction: last_instr,
            motion_progress,
        }
    }

    pub fn full_state(&self, input_names: &HashMap<u32, String>, output_names: &HashMap<u32, String>) -> Value {
        let mut state = serde_json::to_value(self.state()).unwrap_or(Value::Null);

        // Folge numbers this robot has loadable programs for (sorted), so the
        // controller UI can offer a Folge-override selector.
        let mut available_folgen: Vec<u8> = self.programs.keys().copied().collect();
        available_folgen.sort_unstable();
        state["available_folgen"] = serde_json::json!(available_folgen);

        if let Some(ref interp) = self.interpreter {
            let instructions: Vec<Value> = interp.instructions.iter().enumerate().map(|(i, instr)| {
                let mut entry = serde_json::json!({
                    "index": i,
                    "text": format!("{}", instr),
                    "is_current": i == interp.pc && !interp.is_complete(),
                });
                // Add structured parts for tooltips on motion instructions
                if let Instruction::Motion { motion_type, target, position, context, approximate, desc, tool_name, base_name } = instr {
                    entry["parts"] = serde_json::json!([
                        {"text": format!("{}", motion_type), "tip": "Motion type"},
                        {"text": target.clone(), "tip": "Point name"},
                        {"text": format!("{:.1}", position.x), "tip": "X position"},
                        {"text": format!("{:.1}", position.y), "tip": "Y position"},
                        {"text": format!("{:.1}", position.z), "tip": "Z position"},
                        {"text": format!("{:.1}", position.a), "tip": "A orientation"},
                        {"text": format!("{:.1}", position.b), "tip": "B orientation"},
                        {"text": format!("{:.1}", position.c), "tip": "C orientation"},
                        {"text": format!("{}", position.s), "tip": "S (Status)"},
                        {"text": format!("{}", position.t), "tip": "T (Turn)"},
                        {"text": format!("{}", context.vel_ptp), "tip": "Velocity"},
                        {"text": format!("{}", context.tool_no), "tip": "Tool number"},
                        {"text": format!("{}", context.base_no), "tip": "Base number"},
                        {"text": if *approximate { "C_DIS" } else { "" }, "tip": "Approximate (continuous path)"},
                        {"text": desc.clone(), "tip": "Description"},
                        {"text": tool_name.clone(), "tip": "Tool name"},
                        {"text": base_name.clone(), "tip": "Base name"}
                    ]);
                }
                entry
            }).collect();
            state["instructions"] = Value::Array(instructions);

            let mut inputs: Vec<Value> = interp.io.inputs.iter()
                .map(|(ch, val)| {
                    let mut e = serde_json::json!({"channel": ch, "value": val});
                    if let Some(name) = input_names.get(ch) { e["name"] = Value::String(name.clone()); }
                    e
                })
                .collect();
            inputs.sort_by_key(|v| v["channel"].as_u64().unwrap_or(0));

            let mut outputs: Vec<Value> = interp.io.outputs.iter()
                .map(|(ch, val)| {
                    let mut e = serde_json::json!({"channel": ch, "value": val});
                    if let Some(name) = output_names.get(ch) { e["name"] = Value::String(name.clone()); }
                    e
                })
                .collect();
            outputs.sort_by_key(|v| v["channel"].as_u64().unwrap_or(0));

            state["io"] = serde_json::json!({ "inputs": inputs, "outputs": outputs });

            let pos = &interp.current_position;
            state["position"] = serde_json::json!({
                "x": pos.x, "y": pos.y, "z": pos.z,
                "a": pos.a, "b": pos.b, "c": pos.c,
                "s": pos.s, "t": pos.t
            });

            let joints_val = interp.current_joints.or(self.home_joints).unwrap_or([0.0; 6]);
            state["joints"] = serde_json::json!(joints_val);
        } else {
            let pos = &self.position;
            state["position"] = serde_json::json!({
                "x": pos.x, "y": pos.y, "z": pos.z,
                "a": pos.a, "b": pos.b, "c": pos.c,
                "s": pos.s, "t": pos.t
            });
            let joints_val = self.home_joints.unwrap_or([0.0; 6]);
            state["joints"] = serde_json::json!(joints_val);
        }

        state
    }
}
