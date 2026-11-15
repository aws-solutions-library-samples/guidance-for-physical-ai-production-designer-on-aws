//! Test helpers: build planner-style JSON plans in code and run the pipeline.
#![allow(dead_code)]

use serde_json::{Map, Value, json};

use bt2plc::input::{SourceSel, load_str};
use bt2plc::ir::CellPlan;
use bt2plc::{Config, Diagnostics, Result};

pub fn fixture(name: &str) -> String {
    std::fs::read_to_string(format!(
        "{}/../../test_data/{name}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap()
}

pub const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
pub const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";

const HOME: [f64; 6] = [
    0.0,
    -std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
    0.0,
    0.0,
    0.0,
];

#[derive(Default)]
pub struct Tree {
    pub bb: Map<String, Value>,
}

impl Tree {
    fn traj(&mut self, key: &str, dur: f64) -> String {
        self.bb.insert(
            key.to_string(),
            json!([
                {"positions": HOME, "time_s": 0.0, "phase": "transit"},
                {"positions": HOME, "time_s": dur, "phase": "transit"}
            ]),
        );
        format!("{{{key}}}")
    }

    pub fn exec(&mut self, robot: &str, op: &str, dur: f64) -> Value {
        let t = self.traj(&format!("traj_{robot}_{op}"), dur);
        json!({"type": "ExecuteTrajectory", "name": format!("weld_{robot}_{op}"),
               "ports": {"robot_ns": robot, "operation_id": op, "operation_type": "weld", "trajectory": t,
                         "hold_duration_s": 0.5, "velocity_scale": 1.0, "has_interlock": false}})
    }

    pub fn home(&mut self, robot: &str) -> Value {
        let t = self.traj(&format!("traj_{robot}_home"), 2.0);
        json!({"type": "MoveHome", "name": format!("home_{robot}"), "ports": {"robot_ns": robot, "trajectory": t}})
    }

    pub fn wait(&self, robot: &str, deps: &[(&str, &str)]) -> Value {
        let d: Vec<Value> = deps
            .iter()
            .map(|(r, o)| json!({"robot": r, "operation_id": o}))
            .collect();
        json!({"type": "WaitForRobot", "name": format!("wait_{robot}"), "ports": {"robot_ns": robot, "depends_on_ops": d}})
    }

    pub fn envelope(&self, root: Value) -> String {
        json!({"tree_id": "bt_test", "cell_id": "test_cell", "status": "ok",
               "behaviour_tree": {"tree_id": "bt_test", "cell_id": "test_cell", "BTCPP_format": "4",
                                  "main_tree_to_execute": "MainTree", "root": root, "blackboard": self.bb}})
        .to_string()
    }
}

pub fn seq(children: Vec<Value>) -> Value {
    json!({"type": "Sequence", "ports": {}, "children": children})
}

pub fn par(children: Vec<Value>) -> Value {
    json!({"type": "Parallel", "ports": {"success_count": -1, "failure_count": 1}, "children": children})
}

pub fn delay(ms: u64, child: Value) -> Value {
    json!({"type": "Delay", "ports": {"delay_msec": ms}, "children": [child]})
}

pub fn plan_json(text: &str, cfg: &Config) -> Result<(CellPlan, Diagnostics)> {
    let mut d = Diagnostics::default();
    let doc = load_str(text, "test.json", SourceSel::Auto, None, &mut d)?;
    let p = bt2plc::plan(&doc, cfg, &mut d)?;
    Ok((p, d))
}

pub fn plan_fixture(name: &str) -> CellPlan {
    plan_json(&fixture(name), &Config::default()).unwrap().0
}

pub fn zone_strings(p: &CellPlan) -> Vec<String> {
    p.zones
        .iter()
        .map(|z| {
            format!(
                "{} = {}",
                z.name,
                z.tickets
                    .iter()
                    .map(|t| p.op_ref(t.op))
                    .collect::<Vec<_>>()
                    .join(" > ")
            )
        })
        .collect()
}
