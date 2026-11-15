//! Test helpers: bt2plc plans built in code (like ../bt2plc/tests/common) -> program.json -> bt2krl.
#![allow(dead_code)]

use serde_json::{Map, Value, json};

use bt2krl::Config;
use bt2krl::program::Program;

pub const HOME: [f64; 6] = [
    0.0,
    -std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
    0.0,
    0.0,
    0.0,
];

/// bt2plc output of an example cell (`examples/<cell>/plc`, written by `plan2cell build`).
pub fn plc_dir(sample: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .join(sample)
        .join("plc")
}

/// A plan of `test_data/`.
pub fn test_data(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../test_data")
        .join(name)
}

pub fn test_cfg() -> Config {
    Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/bt2krl.toml"),
    )
    .unwrap()
}

pub fn vw_cfg() -> Config {
    Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/bt2krl-vw.toml"),
    )
    .unwrap()
}

/// Config for robots r0..r3 (proptest trees): KR240, no external axes.
pub fn cfg_for(robots: &[&str]) -> Config {
    let mut s = String::new();
    for r in robots {
        s.push_str(&format!(
            "[[robot]]\nns = \"{r}\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n"
        ));
    }
    Config::parse(&s).unwrap()
}

pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    /// Uniform in [-1, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

/// Plan builder (BT.CPP JSON mirror) with random joint trajectories that start and end at home.
#[derive(Default)]
pub struct Tree {
    pub bb: Map<String, Value>,
    pub seed: u64,
}

impl Tree {
    fn traj(&mut self, key: &str, dur: f64) -> String {
        let mut rng = Rng(self.seed ^ (self.bb.len() as u64).wrapping_mul(0x1234_5678_9abc));
        let n = 2 + rng.below(12) as usize;
        let mut pts = Vec::new();
        let mut q = HOME;
        for i in 0..=n {
            let t = dur * i as f64 / n as f64;
            if i == n {
                q = HOME;
            } else if i > 0 {
                // A small random walk around home (well inside the KR240 limits).
                for (j, v) in q.iter_mut().enumerate() {
                    *v = HOME[j] + (rng.unit() * 0.3) * (i as f64 / n as f64).min(1.0);
                }
            }
            let phase = if i * 3 > n * 2 { "approach" } else { "transit" };
            pts.push(json!({"positions": q, "time_s": t, "phase": phase}));
        }
        self.bb.insert(key.to_string(), Value::Array(pts));
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

    pub fn blind_home(&self, robot: &str) -> Value {
        json!({"type": "MoveHome", "name": format!("home_{robot}"), "ports": {"robot_ns": robot}})
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

/// bt2plc in memory: plan text -> program.json of every robot.
pub fn programs(text: &str, plc_cfg: &bt2plc::Config) -> Vec<Program> {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        text,
        "test.json",
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .expect("bt2plc loads the plan");
    let g = bt2plc::generate(
        &doc,
        plc_cfg,
        &bt2plc::GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .expect("bt2plc plans the tree");
    g.plan
        .robots
        .iter()
        .map(|r| {
            let f = format!("robots/{}/program.json", r.ns);
            Program::from_str(g.output.get(&f).unwrap(), &f).unwrap()
        })
        .collect()
}
