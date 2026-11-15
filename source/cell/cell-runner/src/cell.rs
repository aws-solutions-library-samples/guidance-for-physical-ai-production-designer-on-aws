use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum Phase {
    Idle,
    EnableDrives,
    WaitReady,
    SelectSequence,
    WaitEcho,
    StartSequence,
    Running,
    WaitFM,
    Stopped,
}

impl std::fmt::Display for Phase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Phase::Idle => write!(f, "IDLE"),
            Phase::EnableDrives => write!(f, "ENABLE_DRIVES"),
            Phase::WaitReady => write!(f, "WAIT_READY"),
            Phase::SelectSequence => write!(f, "SELECT_SEQUENCE"),
            Phase::WaitEcho => write!(f, "WAIT_ECHO"),
            Phase::StartSequence => write!(f, "START_SEQUENCE"),
            Phase::Running => write!(f, "RUNNING"),
            Phase::WaitFM => write!(f, "WAIT_FM"),
            Phase::Stopped => write!(f, "STOPPED"),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(untagged)]
pub enum SignalValue {
    Bool(bool),
    Byte(u8),
}

#[derive(Clone)]
pub struct RobotState {
    pub name: String,
    pub robot_id: String,
    pub model: String,
    /// Optional URDF path (relative to the cell directory) used for IK/FK.
    /// When set, it overrides the default env/model/<model>.urdf lookup so a
    /// self-contained cell can point at the robot's own dressed URDF.
    pub urdf: Option<String>,
    /// robot_id used by the upstream scene generator (scene.yaml). Lets the
    /// runner read the home pose from scene.yaml's `home_pose_deg`.
    pub scene_id: Option<String>,
    pub phase: Phase,
    pub sequence: u8,
    pub production_folgen: Vec<u8>,
    pub current_folge_idx: usize,
    pub is_home_cycle: bool,
    pub cycles_completed: u64,
    pub role: String,
    pub depends_on: Vec<String>,
    pub inputs: HashMap<String, SignalValue>,
    pub outputs: HashMap<String, SignalValue>,
}

#[derive(Clone)]
pub struct CellSimulator {
    pub robots: Vec<RobotState>,
    pub cell_name: String,
    pub cycle_count: u64,
}

impl CellSimulator {
    pub fn from_run_config(path: &Path) -> Result<Self> {
        let content =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let yaml: RunConfigYaml =
            serde_yaml::from_str(&content).with_context(|| format!("parsing {}", path.display()))?;

        let robots = yaml
            .robots
            .iter()
            .map(|r| {
                let mut inputs = HashMap::new();
                let mut outputs = HashMap::new();

                inputs.insert("RoboterBereit".into(), SignalValue::Bool(false));
                inputs.insert("PF0".into(), SignalValue::Bool(true));
                inputs.insert("SpiegelungFolge".into(), SignalValue::Byte(0));
                inputs.insert("FM".into(), SignalValue::Bool(false));

                outputs.insert("AntriebeEin".into(), SignalValue::Bool(false));
                outputs.insert("Frei_RC".into(), SignalValue::Bool(false));
                outputs.insert("MaSich".into(), SignalValue::Bool(false));
                outputs.insert("AnwahlFolge".into(), SignalValue::Byte(0));
                outputs.insert("AnwahlFolgeStart".into(), SignalValue::Bool(false));

                RobotState {
                    name: r.name.clone(),
                    robot_id: r.id.clone(),
                    model: r.model.clone().unwrap_or_else(|| "kr240_r2900_ultra".to_string()),
                    urdf: r.urdf.clone(),
                    scene_id: r.scene_id.clone(),
                    phase: Phase::Idle,
                    sequence: 0,
                    production_folgen: r.production_folgen.clone(),
                    current_folge_idx: 0,
                    is_home_cycle: false,
                    cycles_completed: 0,
                    role: r.role.clone().unwrap_or_default(),
                    depends_on: r.depends_on.clone(),
                    inputs,
                    outputs,
                }
            })
            .collect();

        let cell_name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();

        Ok(Self {
            robots,
            cell_name,
            cycle_count: 0,
        })
    }

    pub fn step(&mut self) -> StepResult {
        self.cycle_count += 1;
        let mut waiting = Vec::new();

        let dep_satisfied: Vec<bool> = self
            .robots
            .iter()
            .map(|r| self.check_dependencies(r))
            .collect();

        for (i, robot) in self.robots.iter_mut().enumerate() {
            if !dep_satisfied[i] && robot.phase == Phase::SelectSequence {
                waiting.push(format!(
                    "{}: Waiting for deps {:?}",
                    robot.robot_id, robot.depends_on
                ));
                continue;
            }
            if let Some(msg) = step_robot(robot) {
                waiting.push(format!("{}: {}", robot.robot_id, msg));
            }
        }

        if !waiting.is_empty() {
            StepResult::Waiting(waiting)
        } else {
            StepResult::Stepped
        }
    }

    pub fn check_dependencies(&self, robot: &RobotState) -> bool {
        if robot.depends_on.is_empty() {
            return true;
        }
        for dep_id in &robot.depends_on {
            let dep_running = self.robots.iter().any(|r| {
                r.robot_id == *dep_id
                    && (r.phase == Phase::Running || r.phase == Phase::WaitFM)
            });
            if !dep_running {
                return false;
            }
        }
        true
    }

    pub fn reset(&mut self) {
        self.cycle_count = 0;
        for robot in &mut self.robots {
            robot.phase = Phase::Idle;
            robot.sequence = 0;
            robot.current_folge_idx = 0;
            robot.is_home_cycle = false;
            robot.cycles_completed = 0;
            robot.inputs.insert("RoboterBereit".into(), SignalValue::Bool(false));
            robot.inputs.insert("PF0".into(), SignalValue::Bool(true));
            robot.inputs.insert("SpiegelungFolge".into(), SignalValue::Byte(0));
            robot.inputs.insert("FM".into(), SignalValue::Bool(false));
            robot.outputs.insert("AntriebeEin".into(), SignalValue::Bool(false));
            robot.outputs.insert("Frei_RC".into(), SignalValue::Bool(false));
            robot.outputs.insert("MaSich".into(), SignalValue::Bool(false));
            robot.outputs.insert("AnwahlFolge".into(), SignalValue::Byte(0));
            robot.outputs.insert("AnwahlFolgeStart".into(), SignalValue::Bool(false));
        }
    }
}

pub enum StepResult {
    Stepped,
    Waiting(Vec<String>),
}

fn step_robot(robot: &mut RobotState) -> Option<String> {
    match robot.phase {
        Phase::Idle => {
            robot.outputs.insert("AntriebeEin".into(), SignalValue::Bool(true));
            robot.outputs.insert("Frei_RC".into(), SignalValue::Bool(true));
            robot.outputs.insert("MaSich".into(), SignalValue::Bool(true));
            robot.phase = Phase::EnableDrives;
            None
        }
        Phase::EnableDrives => {
            robot.phase = Phase::WaitReady;
            Some("Waiting for RoboterBereit".into())
        }
        Phase::WaitReady => {
            if !get_bool(&robot.inputs, "RoboterBereit") {
                return Some("Waiting for RoboterBereit".into());
            }
            robot.is_home_cycle = false;
            robot.phase = Phase::SelectSequence;
            None
        }
        Phase::SelectSequence => {
            let folge = if robot.is_home_cycle {
                125u8
            } else {
                robot.production_folgen.get(robot.current_folge_idx).copied().unwrap_or(11)
            };
            robot.sequence = folge;
            robot.outputs.insert("AnwahlFolge".into(), SignalValue::Byte(folge));
            robot.phase = Phase::WaitEcho;
            None
        }
        Phase::WaitEcho => {
            let echo = get_byte(&robot.inputs, "SpiegelungFolge");
            if echo != robot.sequence {
                return Some(format!("Waiting for SpiegelungFolge={} (got {})", robot.sequence, echo));
            }
            robot.phase = Phase::StartSequence;
            None
        }
        Phase::StartSequence => {
            robot.outputs.insert("AnwahlFolgeStart".into(), SignalValue::Bool(true));
            robot.phase = Phase::Running;
            None
        }
        Phase::Running => {
            robot.outputs.insert("AnwahlFolgeStart".into(), SignalValue::Bool(false));
            robot.inputs.insert("FM".into(), SignalValue::Bool(false));
            robot.phase = Phase::WaitFM;
            let label = if robot.is_home_cycle { "Home (125)".to_string() } else { format!("Folge {}", robot.sequence) };
            Some(format!("{} running — waiting for FM", label))
        }
        Phase::WaitFM => {
            if !get_bool(&robot.inputs, "FM") {
                let label = if robot.is_home_cycle { "Home (125)".to_string() } else { format!("Folge {}", robot.sequence) };
                return Some(format!("{} running — waiting for FM", label));
            }
            if robot.is_home_cycle {
                robot.current_folge_idx += 1;
                if robot.current_folge_idx >= robot.production_folgen.len() {
                    robot.current_folge_idx = 0;
                    robot.cycles_completed += 1;
                }
                robot.is_home_cycle = false;
            } else {
                robot.is_home_cycle = true;
            }
            robot.phase = Phase::SelectSequence;
            None
        }
        Phase::Stopped => None,
    }
}

fn get_bool(map: &HashMap<String, SignalValue>, key: &str) -> bool {
    match map.get(key) {
        Some(SignalValue::Bool(v)) => *v,
        _ => false,
    }
}

fn get_byte(map: &HashMap<String, SignalValue>, key: &str) -> u8 {
    match map.get(key) {
        Some(SignalValue::Byte(v)) => *v,
        _ => 0,
    }
}

#[derive(Deserialize)]
struct RunConfigYaml {
    #[serde(default)]
    robots: Vec<RunConfigRobot>,
}

#[derive(Deserialize)]
struct RunConfigRobot {
    id: String,
    name: String,
    #[serde(default)]
    production_folgen: Vec<u8>,
    #[serde(default)]
    model: Option<String>,
    #[serde(default)]
    urdf: Option<String>,
    #[serde(default)]
    scene_id: Option<String>,
    #[serde(default)]
    role: Option<String>,
    #[serde(default)]
    depends_on: Vec<String>,
}
