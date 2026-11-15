//! The real check: the cell planner's weld IK (`trajgen.ik.SceneIK`, Pinocchio), run by
//! `deploy/planner-router/reach_check.py` in the planner's venv, one process per robot.
//!
//! Each process gets its robot and welds as JSON on stdin and prints one NDJSON line per weld
//! (`{"robot","weld","reachable","q_deg","reasons","err":{"pos_m","tool_axis_deg"},"elapsed_s"}`),
//! then `{"done":true}`; on failure `{"error":…}` and exit code 1.

use std::io::{BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use factoryos_protocol::fitter::{ReachMode, ReachReason, ToolTcp};

/// Where the IK runs.
#[derive(Clone, Debug)]
pub struct IkSetup {
    pub python: PathBuf,
    pub script: PathBuf,
    pub planner_src: PathBuf,
    pub timeout: Duration,
}

impl IkSetup {
    /// Why it can't run, if it can't: "IK unavailable: …".
    pub fn unavailable(&self) -> Option<String> {
        let missing = |what: &str, path: &PathBuf| format!("IK unavailable: {what} {} not found", path.display());
        if !self.python.is_file() {
            return Some(missing("the cell planner's Python", &self.python));
        }
        if !self.script.is_file() {
            return Some(missing("the reach check script", &self.script));
        }
        if !self.planner_src.is_dir() {
            return Some(missing("the cell planner's sources", &self.planner_src));
        }
        None
    }
}

/// One robot as the script takes it (scene frame, `scene_origin` subtracted).
#[derive(Clone, Debug, serde::Serialize)]
pub struct IkRobot {
    pub id: String,
    pub urdf: PathBuf,
    /// The URDF root's position, pedestal included.
    pub base_xyz: [f64; 3],
    pub yaw_deg: f64,
    pub tilt_bc_deg: [f64; 2],
    pub home_deg: Vec<f64>,
    pub tcp: IkTcp,
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct IkTcp {
    pub tip_link: String,
    pub offset: [f64; 3],
    pub axis: [f64; 3],
}

impl From<&ToolTcp> for IkTcp {
    fn from(tcp: &ToolTcp) -> Self {
        Self { tip_link: tcp.tip_link.clone(), offset: tcp.offset, axis: tcp.axis }
    }
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct IkWeld {
    pub id: String,
    pub position: [f64; 3],
    pub approach: [f64; 3],
}

/// One robot's result on one weld.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PairResult {
    pub robot: String,
    pub weld: String,
    pub reachable: bool,
    pub q_deg: Vec<f64>,
    pub reasons: Vec<ReachReason>,
    pub pos_m: Option<f64>,
    pub axis_deg: Option<f64>,
    pub detail: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum IkEvent {
    Pair(PairResult),
    /// A robot's process failed; its remaining welds are not checked.
    Failed { robot: String, error: String },
}

#[derive(Debug, PartialEq)]
pub enum IkStop {
    Cancelled,
    TimedOut,
}

#[derive(serde::Serialize)]
struct Input<'a> {
    mode: ReachMode,
    robots: [&'a IkRobot; 1],
    welds: &'a [IkWeld],
    floor_z: f64,
    tolerances: Tolerances,
}

#[derive(serde::Serialize)]
struct Tolerances {
    position_m: f64,
    tool_axis_deg: f64,
}

/// The script's reason names (and trajgen's raw ones, e.g. `self_collision(0.0012)`).
pub fn map_reason(reason: &str) -> ReachReason {
    let name = reason.split('(').next().unwrap_or(reason).trim();
    match name {
        "joint_limits" => ReachReason::JointLimits,
        "self_collision" => ReachReason::SelfCollision,
        "floor" | "umarell_floor" => ReachReason::Floor,
        "pose_tolerance" => ReachReason::PoseTolerance,
        "no_solution" | "no_ik_solution" => ReachReason::NoSolution,
        "out_of_reach" => ReachReason::OutOfReach,
        "inside_inner_void" => ReachReason::InsideInnerVoid,
        "base_tilt_unsupported" => ReachReason::BaseTiltUnsupported,
        "weld_inside_product" => ReachReason::WeldInsideProduct,
        "tool_hits_product" => ReachReason::ToolHitsProduct,
        _ => ReachReason::Error,
    }
}

/// A pair line of the script, or `None` for other lines.
fn parse_pair(line: &serde_json::Value) -> Option<PairResult> {
    let robot = line["robot"].as_str()?.to_owned();
    let weld = line["weld"].as_str()?.to_owned();
    let raw: Vec<String> = line["reasons"].as_array().into_iter().flatten().filter_map(|r| r.as_str().map(str::to_owned)).collect();
    let mut reasons: Vec<ReachReason> = Vec::new();
    for r in raw.iter().map(|r| map_reason(r)) {
        if !reasons.contains(&r) {
            reasons.push(r);
        }
    }
    let reachable = line["reachable"].as_bool().unwrap_or(false);
    if !reachable && reasons.is_empty() {
        reasons.push(ReachReason::NoSolution);
    }
    let mut detail = line["detail"].as_str().map(str::to_owned).unwrap_or_else(|| raw.join(", "));
    if let Some(t) = line["elapsed_s"].as_f64() {
        detail = if detail.is_empty() { format!("IK {t:.1} s") } else { format!("{detail}; IK {t:.1} s") };
    }
    Some(PairResult {
        robot,
        weld,
        reachable,
        q_deg: line["q_deg"].as_array().into_iter().flatten().filter_map(serde_json::Value::as_f64).collect(),
        reasons,
        pos_m: line["err"]["pos_m"].as_f64(),
        axis_deg: line["err"]["tool_axis_deg"].as_f64(),
        detail,
    })
}

enum Line {
    Out(usize, String),
    Closed(usize),
}

/// Run the IK on `jobs` (each robot with its welds), one process per robot, in parallel. Calls
/// `on_event` as results arrive. Stops (killing the processes) when `cancel` is set or after
/// `setup.timeout`.
pub fn run(
    setup: &IkSetup,
    mode: ReachMode,
    floor_z: f64,
    jobs: &[(IkRobot, Vec<IkWeld>)],
    mut on_event: impl FnMut(IkEvent),
    cancel: &AtomicBool,
) -> Result<(), IkStop> {
    let started = Instant::now();
    let (tx, rx) = mpsc::channel::<Line>();
    let mut children: Vec<(String, Option<Child>, String, bool)> = Vec::new(); // robot, child, stderr, finished
    let mut stderr_threads = Vec::new();
    for (i, (robot, welds)) in jobs.iter().enumerate() {
        if welds.is_empty() {
            children.push((robot.id.clone(), None, String::new(), true));
            continue;
        }
        let input = Input {
            mode,
            robots: [robot],
            welds,
            floor_z,
            tolerances: Tolerances { position_m: 0.01, tool_axis_deg: 5.0 },
        };
        let spawned = Command::new(&setup.python)
            .arg("-I")
            .arg(&setup.script)
            .arg("--planner-src")
            .arg(&setup.planner_src)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn();
        let mut child = match spawned {
            Ok(child) => child,
            Err(err) => {
                on_event(IkEvent::Failed { robot: robot.id.clone(), error: format!("starting {}: {err}", setup.python.display()) });
                children.push((robot.id.clone(), None, String::new(), true));
                continue;
            }
        };
        if let Some(mut stdin) = child.stdin.take() {
            let json = serde_json::to_vec(&input).unwrap_or_default();
            std::thread::spawn(move || {
                stdin.write_all(&json).ok();
            });
        }
        if let Some(stdout) = child.stdout.take() {
            let tx = tx.clone();
            std::thread::spawn(move || {
                for line in BufReader::new(stdout).lines() {
                    let Ok(line) = line else { break };
                    if tx.send(Line::Out(i, line)).is_err() {
                        return;
                    }
                }
                tx.send(Line::Closed(i)).ok();
            });
        }
        if let Some(mut stderr) = child.stderr.take() {
            stderr_threads.push((
                i,
                std::thread::spawn(move || {
                    let mut text = String::new();
                    stderr.read_to_string(&mut text).ok();
                    text
                }),
            ));
        }
        children.push((robot.id.clone(), Some(child), String::new(), false));
    }
    drop(tx);
    let mut errored = vec![false; children.len()];
    let kill_all = |children: &mut Vec<(String, Option<Child>, String, bool)>| {
        for (_, child, _, _) in children.iter_mut() {
            if let Some(child) = child.as_mut() {
                child.kill().ok();
                child.wait().ok();
            }
        }
    };
    loop {
        if children.iter().all(|c| c.3) {
            break;
        }
        if cancel.load(Ordering::Relaxed) {
            kill_all(&mut children);
            return Err(IkStop::Cancelled);
        }
        if started.elapsed() > setup.timeout {
            kill_all(&mut children);
            return Err(IkStop::TimedOut);
        }
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(Line::Out(i, line)) => {
                let Ok(value) = serde_json::from_str::<serde_json::Value>(&line) else {
                    tracing::debug!("reach check ({}): {line}", children[i].0);
                    continue;
                };
                if let Some(pair) = parse_pair(&value) {
                    on_event(IkEvent::Pair(pair));
                } else if let Some(error) = value["error"].as_str() {
                    errored[i] = true;
                    on_event(IkEvent::Failed { robot: children[i].0.clone(), error: error.to_owned() });
                }
            }
            Ok(Line::Closed(i)) => {
                let status = children[i].1.as_mut().and_then(|c| c.wait().ok());
                children[i].3 = true;
                if !status.is_some_and(|s| s.success()) && !errored[i] {
                    errored[i] = true;
                    let stderr = stderr_threads
                        .iter()
                        .position(|(j, _)| *j == i)
                        .map(|k| stderr_threads.remove(k).1.join().unwrap_or_default())
                        .unwrap_or_default();
                    let tail: Vec<&str> = stderr.lines().rev().take(6).collect::<Vec<_>>().into_iter().rev().collect();
                    let code = status.and_then(|s| s.code()).map_or("killed".to_owned(), |c| format!("exit code {c}"));
                    on_event(IkEvent::Failed { robot: children[i].0.clone(), error: format!("{code}: {}", tail.join(" | ")) });
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                // Every reader is gone: reap what's left.
                for (i, c) in children.iter_mut().enumerate() {
                    if !c.3 {
                        let ok = c.1.as_mut().and_then(|c| c.wait().ok()).is_some_and(|s| s.success());
                        c.3 = true;
                        if !ok && !errored[i] {
                            on_event(IkEvent::Failed { robot: c.0.clone(), error: "the IK process ended".to_owned() });
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A stub IK script: prints a pair line per weld (reachable when the weld's x > 0), sleeping
    /// `delay` seconds before each.
    pub fn stub(dir: &std::path::Path, delay: f64) -> IkSetup {
        std::fs::create_dir_all(dir).unwrap();
        let script = dir.join("reach_check.py");
        std::fs::write(
            &script,
            format!(
                r#"import json, sys, time
job = json.load(sys.stdin)
r = job["robots"][0]
assert "--planner-src" in sys.argv
for w in job["welds"]:
    time.sleep({delay})
    ok = w["position"][0] > 0
    print(json.dumps({{"robot": r["id"], "weld": w["id"], "reachable": ok, "q_deg": [1, 2, 3, 4, 5, 6] if ok else [],
                      "reasons": [] if ok else ["self_collision(0.0010)", "joint_limits"],
                      "err": {{"pos_m": 0.001, "tool_axis_deg": 0.5}}, "elapsed_s": 0.1}}), flush=True)
print(json.dumps({{"done": True}}))
"#
            ),
        )
        .unwrap();
        IkSetup { python: which_python(), script, planner_src: dir.to_owned(), timeout: Duration::from_secs(60) }
    }

    pub fn which_python() -> PathBuf {
        for p in ["/usr/bin/python3", "/opt/homebrew/bin/python3", "/usr/local/bin/python3"] {
            if std::path::Path::new(p).is_file() {
                return p.into();
            }
        }
        "python3".into()
    }

    pub fn robot(id: &str) -> IkRobot {
        IkRobot {
            id: id.into(),
            urdf: "r.urdf".into(),
            base_xyz: [0.0; 3],
            yaw_deg: 0.0,
            tilt_bc_deg: [0.0; 2],
            home_deg: vec![0.0; 6],
            tcp: IkTcp { tip_link: "l6".into(), offset: [-0.9, 0.0, 0.0], axis: [-1.0, 0.0, 0.0] },
        }
    }

    pub fn weld(id: &str, x: f64) -> IkWeld {
        IkWeld { id: id.into(), position: [x, 0.0, 1.0], approach: [0.0, 0.0, -1.0] }
    }

    #[test]
    fn reasons_map_to_the_protocol() {
        assert_eq!(map_reason("self_collision(0.0012)"), ReachReason::SelfCollision);
        assert_eq!(map_reason("floor(-0.010)"), ReachReason::Floor);
        assert_eq!(map_reason("umarell_floor"), ReachReason::Floor);
        assert_eq!(map_reason("pose_tolerance"), ReachReason::PoseTolerance);
        assert_eq!(map_reason("base_tilt_unsupported"), ReachReason::BaseTiltUnsupported);
        assert_eq!(map_reason("what"), ReachReason::Error);
    }

    #[test]
    fn the_runner_streams_pairs() {
        let dir = std::env::temp_dir().join(format!("factoryos-reach-ik-{}", std::process::id()));
        let setup = stub(&dir, 0.0);
        assert!(setup.unavailable().is_none());
        let jobs = vec![(robot("r1"), vec![weld("a", 1.0), weld("b", -1.0)]), (robot("r2"), vec![weld("a", 1.0)])];
        let mut events = Vec::new();
        run(&setup, ReachMode::Quick, 0.0, &jobs, |e| events.push(e), &AtomicBool::new(false)).unwrap();
        assert_eq!(events.len(), 3, "{events:?}");
        let b = events.iter().find_map(|e| match e {
            IkEvent::Pair(p) if p.weld == "b" => Some(p.clone()),
            _ => None,
        });
        let b = b.unwrap();
        assert!(!b.reachable);
        assert_eq!(b.reasons, [ReachReason::SelfCollision, ReachReason::JointLimits]);
        assert_eq!(b.pos_m, Some(0.001));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn cancel_and_timeout_kill_the_processes() {
        let dir = std::env::temp_dir().join(format!("factoryos-reach-ik-slow-{}", std::process::id()));
        let mut setup = stub(&dir, 30.0);
        let jobs = vec![(robot("r1"), vec![weld("a", 1.0)])];
        let cancel = AtomicBool::new(true);
        let t = Instant::now();
        assert_eq!(run(&setup, ReachMode::Quick, 0.0, &jobs, |_| {}, &cancel), Err(IkStop::Cancelled));
        setup.timeout = Duration::from_millis(300);
        assert_eq!(run(&setup, ReachMode::Quick, 0.0, &jobs, |_| {}, &AtomicBool::new(false)), Err(IkStop::TimedOut));
        assert!(t.elapsed() < Duration::from_secs(10));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_failing_script_reports_its_error() {
        let dir = std::env::temp_dir().join(format!("factoryos-reach-ik-fail-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let script = dir.join("reach_check.py");
        std::fs::write(&script, "import sys\nprint('{\"error\": \"no trajgen\"}')\nsys.exit(1)\n").unwrap();
        let setup = IkSetup { python: which_python(), script, planner_src: dir.clone(), timeout: Duration::from_secs(30) };
        let mut events = Vec::new();
        run(&setup, ReachMode::Full, 0.0, &[(robot("r1"), vec![weld("a", 1.0)])], |e| events.push(e), &AtomicBool::new(false)).unwrap();
        assert_eq!(events, [IkEvent::Failed { robot: "r1".into(), error: "no trajgen".into() }]);
        let missing = IkSetup { python: dir.join("nope/python"), ..setup };
        assert!(missing.unavailable().unwrap().starts_with("IK unavailable: the cell planner's Python"));
        std::fs::remove_dir_all(&dir).ok();
    }
}
