//! Co-simulation of a build (feature `sim`): the generated head-PLC ST in trust-runtime against
//! the robots, exchanging the PROFINET frames in virtual time, checked against the plan
//! (dependencies, zone exclusion, parallelism, no motion without a grant, cycle time).
//!
//! The robots are either the generated device-simulator ST (`Robots::DeviceSt`, profile `sim`
//! only) or the bt2krl IR executor, i.e. the behaviour of the generated KRL (`Robots::Krl`).

use serde::{Deserialize, Serialize};

use bt2plc::trust::cosim::{Report, RobotModel, RobotModels, SimOptions};

use crate::{Build, Error, Profile, Result};

/// Who plays the robots.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum Robots {
    /// The bt2krl executor of the generated KRL (needs `krl = true`).
    #[default]
    Krl,
    /// The generated CODESYS device simulators (profile sim, `robot_sim = true`).
    DeviceSt,
}

/// The KRL executor as a co-simulation robot.
struct KrlRobot(bt2krl::exec::Executor);

impl RobotModel for KrlRobot {
    fn scan(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]) {
        self.0.scan(dt_ms, cmd, safe_out)
    }

    fn inject_error(&mut self, on: bool) {
        self.0.inject_error(on);
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimResult {
    pub summary: String,
    pub cycles_completed: usize,
    pub cycle_s: Vec<f64>,
    pub ok: bool,
    /// `Report::verify` failures (empty when ok).
    pub errors: Vec<String>,
    /// The last events of the trace when the check failed.
    pub tail: Vec<String>,
}

/// Run `cycles` production cycles of a build.
pub fn simulate(
    build: &Build,
    cfg: &crate::BuildConfig,
    robots: Robots,
    cycles: u32,
) -> Result<(Report, SimResult)> {
    let eff = crate::effective(cfg, &build.options)?;
    let models: RobotModels = match robots {
        Robots::DeviceSt => {
            if build.options.profile != Profile::Sim || !eff.plc.robot_sim.enabled {
                return Err(Error::Usage(
                    "robots = device-st needs profile sim with robot_sim = true".into(),
                ));
            }
            Vec::new()
        }
        Robots::Krl => {
            let g = build
                .krl
                .as_ref()
                .ok_or_else(|| Error::Usage("robots = krl needs a build with krl = true".into()))?;
            let mut m: RobotModels = Vec::new();
            for (ri, r) in build.plan.robots.iter().enumerate() {
                let out = g.robots.iter().find(|x| x.ns == r.ns).ok_or_else(|| {
                    Error::Usage(format!(
                        "robot {} has no KRL in this build (all robots must be simulated)",
                        r.ns
                    ))
                })?;
                let f = format!("robots/{}/program.json", r.ns);
                let text = build.plc.output.get(&f).unwrap_or_default();
                let p = bt2krl::program::Program::from_str(text, &f)?;
                let e = bt2krl::exec::Executor::new(&p, &out.setup, &eff.krl, &out.lowered)?;
                m.push((ri, Box::new(KrlRobot(e)) as Box<dyn RobotModel>));
            }
            m
        }
    };
    let report = bt2plc::trust::cosim::run_with(
        &build.plan,
        &eff.plc,
        &build.plc.projects,
        &SimOptions {
            cycles,
            ..Default::default()
        },
        models,
    )?;
    let verdict = report.verify(&build.plan);
    let res = SimResult {
        summary: report.summary(),
        cycles_completed: report.cycle_end_ms.len(),
        cycle_s: report
            .cycle_start_ms
            .iter()
            .zip(&report.cycle_end_ms)
            .map(|(s, e)| (e - s) as f64 / 1000.0)
            .collect(),
        ok: verdict.is_ok(),
        errors: verdict
            .as_ref()
            .err()
            .map(|e| e.lines().map(str::to_string).collect())
            .unwrap_or_default(),
        tail: if verdict.is_ok() {
            Vec::new()
        } else {
            report
                .events
                .iter()
                .rev()
                .take(40)
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
                .into_iter()
                .rev()
                .collect()
        },
    };
    Ok((report, res))
}
