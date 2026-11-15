use serde::{Deserialize, Serialize};

use crate::interpreter::IoState;

#[derive(Debug, Clone, Deserialize)]
pub struct ToolConfig {
    pub name: String,
    #[serde(default)]
    pub desc: String,
    pub trigger_out: u32,
    pub confirm_in: u32,
    #[serde(default = "default_duration")]
    pub duration_steps: u32,
    #[serde(default = "default_true")]
    pub confirm_on_value: bool,
}

fn default_duration() -> u32 { 1 }
fn default_true() -> bool { true }

#[derive(Debug, Clone, Serialize)]
pub struct ToolState {
    pub name: String,
    pub active: bool,
    pub remaining: u32,
}

pub struct ToolSimulator {
    pub configs: Vec<ToolConfig>,
    pub states: Vec<Vec<ToolState>>,  // per robot, per tool
}

impl ToolSimulator {
    pub fn new(configs: Vec<ToolConfig>, num_robots: usize) -> Self {
        let states = (0..num_robots)
            .map(|_| {
                configs
                    .iter()
                    .map(|c| ToolState {
                        name: c.name.clone(),
                        active: false,
                        remaining: 0,
                    })
                    .collect()
            })
            .collect();
        Self { configs, states }
    }

    pub fn step_robot(&mut self, robot_idx: usize, krl_io: &mut IoState) {
        if robot_idx >= self.states.len() {
            return;
        }

        for (tool_idx, config) in self.configs.iter().enumerate() {
            let state = &mut self.states[robot_idx][tool_idx];
            let trigger = krl_io.get_output(config.trigger_out);

            if trigger && !state.active {
                // Tool just triggered — start timer
                state.active = true;
                state.remaining = config.duration_steps;
            } else if !trigger && state.active {
                // Tool deactivated — immediate confirm off
                state.active = false;
                state.remaining = 0;
                if config.confirm_on_value {
                    krl_io.set_input(config.confirm_in, false);
                } else {
                    krl_io.set_input(config.confirm_in, true);
                }
            }

            if state.active && state.remaining > 0 {
                state.remaining -= 1;
                if state.remaining == 0 {
                    // Timer done — set confirmation
                    krl_io.set_input(config.confirm_in, config.confirm_on_value);
                }
            }
        }
    }

    pub fn reset(&mut self) {
        for robot_states in &mut self.states {
            for state in robot_states.iter_mut() {
                state.active = false;
                state.remaining = 0;
            }
        }
    }

    pub fn robot_states(&self, robot_idx: usize) -> &[ToolState] {
        if robot_idx < self.states.len() {
            &self.states[robot_idx]
        } else {
            &[]
        }
    }
}
