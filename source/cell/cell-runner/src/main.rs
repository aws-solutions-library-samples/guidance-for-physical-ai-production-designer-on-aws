// Vendored build: retained/legacy public API, enum variants, and serde-populated
// fields are kept intentionally (used by other build configs / future features /
// round-tripping), so silence dead-code noise rather than delete them.
#![allow(dead_code)]

mod bridge;
mod btree;
mod cell;
#[cfg(feature = "collision")]
mod cell_environment;
#[cfg(feature = "collision")]
pub mod collision;
mod export;
mod interpreter;
mod krl;
mod limits;
mod path;
mod pinocchio;
mod ruckig_otg;
mod server;
mod tools;
mod traj;
mod trajectory;

use anyhow::{Context, Result};
use clap::Parser;
use std::collections::HashMap;
use std::path::PathBuf;

use cell::CellSimulator;
use krl::RobotKrl;
use tools::{ToolConfig, ToolSimulator};
use traj::TrajectoryProvider;

#[derive(Parser, Debug)]
#[command(name = "cell-runner")]
#[command(about = "Combined PLC + KRL cell runner — executes robot programs with PLC coordination")]
struct Args {
    /// Path to cell directory (containing cell.yaml, io_names.json, krl/)
    #[arg(long)]
    cell: PathBuf,

    /// Path to environment directory (model, scenes, meshes) — served as static files at /env.
    /// Default: the cell directory (a self-contained cell carries its own).
    #[arg(long)]
    env: Option<PathBuf>,

    /// Run as TCP server for VS Code viewer
    #[arg(long)]
    server: bool,

    /// Address the server listens on. Loopback unless asked for: its clients (factoryos-backend
    /// through cell-rerun-server, which passes its environment on, and the cell's scripts) connect
    /// on 127.0.0.1, so an address for other hosts has to include it (`0.0.0.0`).
    #[arg(long, env = "CELL_RUNNER_HOST", default_value = "127.0.0.1")]
    host: std::net::IpAddr,

    /// TCP port for server mode
    #[arg(long, default_value = "9902")]
    port: u16,

    /// KRL instructions to execute per PLC cycle (higher = faster KRL execution)
    #[arg(long, default_value = "10")]
    krl_steps: usize,

    /// Run headless and export ALL robots' trajectories to ONE combined JSON file.
    /// When set, the live server and interactive loop are skipped and no network
    /// socket is opened (Req 1.1, 15.4).
    #[arg(long, value_name = "FILE")]
    export: Option<PathBuf>,

    /// Optional scene YAML providing robot world positions (meters) for
    /// base_position (D3). Only used in export mode.
    #[arg(long, value_name = "FILE")]
    scene: Option<PathBuf>,

    /// Name of the trajectory group in the output (D2). Must be non-empty and at
    /// most 255 characters.
    #[arg(long, default_value = "structural")]
    group_name: String,

    /// Points emitted per trajectory, evenly spaced in time (D5).
    /// Valid range: 2..=10000.
    #[arg(long, default_value = "10", value_parser = clap::builder::RangedU64ValueParser::<usize>::new().range(2..=10000))]
    samples_per_trajectory: usize,

    /// Width of each `positions` vector; axes beyond 6 are padded (D1).
    /// Valid range: 6..=64.
    #[arg(long, default_value = "7", value_parser = clap::builder::RangedU64ValueParser::<usize>::new().range(6..=64))]
    joint_dim: usize,

    /// Hard cap on PLC cycles to guarantee termination (Req 13.1).
    /// Valid range: 1..=100000000.
    #[arg(long, default_value = "100000", value_parser = clap::builder::RangedU64ValueParser::<usize>::new().range(1..=100000000))]
    export_max_cycles: usize,

    /// When set, omit trajectories whose JointPath is `None` (Req 14.3).
    #[arg(long)]
    skip_jointless: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Validate the group name up front, before loading or running the cell, so an
    // invalid value terminates non-zero with no side effects (Req 5.2). The numeric
    // ranges (--samples-per-trajectory, --joint-dim, --export-max-cycles) are
    // enforced by clap during `Args::parse()` above (Req 4.1/4.5/4.6, 8.1/8.7,
    // 13.1/13.2).
    if args.export.is_some() {
        let name_len = args.group_name.chars().count();
        if args.group_name.is_empty() || name_len > 255 {
            anyhow::bail!(
                "invalid --group-name: must be non-empty and at most 255 characters (got {} characters)",
                name_len
            );
        }
    }

    let cell_dir = &args.cell;
    let env = args.env.clone().unwrap_or_else(|| cell_dir.clone());
    let config_path = cell_dir.join("cell.yaml");
    let krl_dir = cell_dir.join("krl");
    let io_names_path = cell_dir.join("io_names.json");

    eprintln!("Loading cell from: {}", cell_dir.display());

    let sim = CellSimulator::from_run_config(&config_path)
        .with_context(|| format!("Failed to load {}", config_path.display()))?;

    eprintln!("  Cell '{}': {} robots", sim.cell_name, sim.robots.len());

    let mut krls: Vec<RobotKrl> = Vec::new();
    for robot in &sim.robots {
        let krl = RobotKrl::load(&robot.robot_id, &krl_dir)
            .with_context(|| format!("Loading KRL for {}", robot.robot_id))?;
        eprintln!("  {} — {} KRL programs: {:?}", robot.robot_id, krl.programs.len(), krl.programs.keys().collect::<Vec<_>>());
        krls.push(krl);
    }

    let io_names = std::fs::read_to_string(&io_names_path).unwrap_or_default();

    let tool_configs = load_tool_configs(&config_path);
    let tool_sim = ToolSimulator::new(tool_configs, sim.robots.len());
    eprintln!("  Tool simulators: {}", tool_sim.configs.len());

    // Load and apply home poses from each robot's movement directory
    // Home poses are TCP positions — remove tool offset for flange-based IK
    let movements_dir = env.join("movements");
    let home_poses = load_home_poses(&movements_dir, &sim.robots);
    for krl in &mut krls {
        if let Some(pos) = home_poses.get(&krl.robot_id) {
            let config_dat_path = krl_dir.join(&krl.robot_id).join("KRC").join("R1").join("System").join("$config.dat");
            let flange_pos = if config_dat_path.exists() {
                let content = std::fs::read_to_string(&config_dat_path).unwrap_or_default();
                let frames = kuka_krl_parser::parser::parse_system_frames(&content);
                let xyz = [pos.x, pos.y, pos.z];
                let abc = [pos.a, pos.b, pos.c];
                let (f_xyz, f_abc) = interpreter::remove_tool_offset_ext(xyz, abc, 1, &frames);
                kuka_krl_parser::types::KrlPosition {
                    x: f_xyz[0], y: f_xyz[1], z: f_xyz[2],
                    a: f_abc[0], b: f_abc[1], c: f_abc[2],
                    s: pos.s, t: pos.t,
                }
            } else {
                pos.clone()
            };
            krl.position = flange_pos.clone();
            eprintln!("  {} — home flange: ({:.1}, {:.1}, {:.1})", krl.robot_id, flange_pos.x, flange_pos.y, flange_pos.z);
        }
    }

    // Load sync_map from cell.yaml
    let sync_map = load_sync_map(&config_path);
    eprintln!("  Sync map: {} robots", sync_map.len());

    // Load per-robot trajectory providers (Ruckig OTG → Pinocchio IK → heuristic),
    // one per distinct robot model (deduped below), mirroring the original
    // per-model Pinocchio map. `LayeredTrajectoryProvider::load` reports the
    // active top tier (RuckigOtg / PinocchioIk / Heuristic) once at startup for
    // each model (Requirements 1.2, 1.3).
    let model_dir = env.join("model");
    let mut providers: HashMap<String, traj::LayeredTrajectoryProvider> = HashMap::new();
    for robot in &sim.robots {
        if providers.contains_key(&robot.model) {
            continue;
        }
        // Prefer the robot's own URDF (relative to the cell directory) when the
        // cell.yaml specifies one — this lets a self-contained cell drive IK/FK
        // straight from robots/<id>/<id>.urdf and drop the separate model/ dir.
        // Otherwise fall back to env/model/<model>.urdf.
        let urdf_path = match &robot.urdf {
            Some(rel) => cell_dir.join(rel),
            None => model_dir.join(format!("{}.urdf", robot.model)),
        };
        eprintln!("  IK model for {} [{}]: {}", robot.robot_id, robot.model, urdf_path.display());
        // LayeredTrajectoryProvider::load handles the missing-URDF case internally
        // (downgrades to the heuristic tier and logs a warning).
        let provider = traj::LayeredTrajectoryProvider::load(&urdf_path, &robot.model);
        providers.insert(robot.model.clone(), provider);
    }

    // Home joints from the scene (home_pose_deg, degrees) keyed by cell robot id.
    // This is the primary source now: a self-contained cell carries its home
    // pose in scene.yaml, so no separate movements/ directory is required.
    let scene_home_joints = load_scene_home_joints(args.scene.as_deref(), cell_dir, &env, &sim.robots);

    // Pre-solved home joints from movements/home-pose.yaml, kept as a fallback
    // for cells that still ship a movements directory (the legacy KRL cells).
    let pre_solved_joints = load_home_joints(&movements_dir, &sim.robots);

    // Resolve each robot's home joints for smooth initial visualization and as
    // the IK warm-start seed. Priority: scene.yaml home_pose_deg > cached
    // movements joints > one-shot IK from the home flange pose.
    if !providers.is_empty() {
        let mut seed = [0.0f64; 6];
        for (i, krl) in krls.iter_mut().enumerate() {
            let model = sim.robots[i].model.clone();

            // 1. scene.yaml home_pose_deg (converted to radians). Derive the
            //    flange cartesian via FK so the readout and the first motion's
            //    start pose are consistent.
            if let Some(joints) = scene_home_joints.get(&krl.robot_id) {
                krl.home_joints = Some(*joints);
                seed = *joints;
                if let Some(provider) = providers.get_mut(&model) {
                    if let Some(pose) = provider.forward_kinematics(joints) {
                        krl.position = kuka_krl_parser::types::KrlPosition {
                            x: pose[0], y: pose[1], z: pose[2],
                            a: pose[3], b: pose[4], c: pose[5],
                            s: 0, t: 0,
                        };
                    }
                }
                eprintln!("  {} [{}] — home joints (scene.yaml): [{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}]",
                    krl.robot_id, model, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
                continue;
            }

            // 2. cached joints from movements/home-pose.yaml
            if let Some(joints) = pre_solved_joints.get(&krl.robot_id) {
                eprintln!("  {} [{}] — home joints (cached): [{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}]",
                    krl.robot_id, model, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
                krl.home_joints = Some(*joints);
                seed = *joints;
                continue;
            }

            // 3. one-shot IK from the home flange pose (from movements, if any)
            let provider = match providers.get_mut(&model) {
                Some(p) => p,
                None => continue,
            };
            let pos = &krl.position;
            let target = [pos.x, pos.y, pos.z, pos.a, pos.b, pos.c];
            match provider.solve_pose(&target, &seed) {
                Some(joints) => {
                    eprintln!("  {} [{}] — home joints: [{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}]",
                        krl.robot_id, model, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
                    krl.home_joints = Some(joints);
                    seed = joints;
                }
                None => {
                    eprintln!("  {} [{}] — home IK: no valid solution found", krl.robot_id, model);
                }
            }
        }
    }

    if let Some(export_path) = args.export.as_ref() {
        // Headless export mode: skip the server and interactive loop, open no
        // network socket (Req 1.1, 15.4). Resolve base positions from the optional
        // scene file (Req 6.1, 6.5), then run the deterministic export driver.
        let base_positions = export::load_base_positions(args.scene.as_deref(), &sim.robots)
            .with_context(|| "loading base positions for export")?;

        let opts = export::ExportOptions {
            group_name: args.group_name.clone(),
            samples_per_trajectory: args.samples_per_trajectory,
            joint_dim: args.joint_dim,
            external_axis_fill: 0.0,
            max_cycles: args.export_max_cycles,
            skip_jointless: args.skip_jointless,
            min_duration_floor: 0.001,
        };

        eprintln!("Exporting trajectories to: {}", export_path.display());
        let summary = export::run_export(sim, krls, providers, base_positions, &opts, export_path)
            .with_context(|| format!("exporting trajectories to {}", export_path.display()))?;

        eprintln!(
            "  Export complete: {} robots, {} trajectories, {} downgraded, {} cycles{}",
            summary.robot_count,
            summary.trajectory_count,
            summary.downgraded_count,
            summary.cycles_run,
            if summary.max_cycles_reached {
                format!(
                    " (max_cycles cap reached; incomplete: {:?})",
                    summary.incomplete_robots
                )
            } else {
                String::new()
            }
        );
        Ok(())
    } else if args.server {
        let env_dir = std::fs::canonicalize(&env)
            .unwrap_or_else(|_| env.clone());
        eprintln!("  Serving env from: {}", env_dir.display());
        server::run_server(
            sim,
            krls,
            tool_sim,
            args.host,
            args.port,
            args.krl_steps,
            &io_names,
            sync_map,
            env_dir,
            home_poses,
            providers,
            args.cell.clone(),
            #[cfg(feature = "collision")]
            args.scene.clone(),
        )
    } else {
        run_interactive(sim, krls, tool_sim, args.krl_steps, home_poses)
    }
}

fn run_interactive(mut sim: CellSimulator, mut krls: Vec<RobotKrl>, mut tool_sim: ToolSimulator, krl_steps: usize, home_poses: HashMap<String, kuka_krl_parser::types::KrlPosition>) -> Result<()> {
    use std::io::{BufRead, Write};

    println!("Cell Runner — Combined PLC + KRL");
    println!("=================================");
    println!("Cell: {}", sim.cell_name);
    for (i, r) in sim.robots.iter().enumerate() {
        let krl_info = if krls[i].programs.is_empty() { "no KRL".into() }
        else { format!("KRL: {:?}", krls[i].programs.keys().collect::<Vec<_>>()) };
        println!("  [{}] {} — {} — {}", i, r.robot_id, r.role, krl_info);
    }
    println!();
    println!("Commands: step, run, reset, status, set_all_ready, quit");
    println!();

    loop {
        print!("[cycle {}] cell> ", sim.cycle_count);
        std::io::stdout().flush()?;

        let mut line = String::new();
        if std::io::stdin().lock().read_line(&mut line)? == 0 { break; }
        let cmd = line.trim();

        match cmd {
            "" | "s" | "step" => {
                sim.step();
                for (i, krl) in krls.iter_mut().enumerate() {
                    bridge::sync_step(&mut sim.robots[i], krl, krl_steps, &mut tool_sim, i);
                }
                print_status(&sim, &krls);
            }
            "r" | "run" => {
                for _ in 0..100 {
                    sim.step();
                    for (i, krl) in krls.iter_mut().enumerate() {
                        bridge::sync_step(&mut sim.robots[i], krl, krl_steps, &mut tool_sim, i);
                    }
                }
                print_status(&sim, &krls);
            }
            "reset" => {
                sim.reset();
                for krl in krls.iter_mut() {
                    krl.interpreter = None;
                    krl.active_folge = 0;
                    krl.program_complete = false;
                    krl.io = crate::interpreter::IoState::new();
                    krl.position = home_poses.get(&krl.robot_id)
                        .cloned()
                        .unwrap_or_default();
                }
                println!("  Reset.");
            }
            "set_all_ready" | "ready" => {
                for robot in &mut sim.robots {
                    robot.inputs.insert("RoboterBereit".into(), cell::SignalValue::Bool(true));
                }
                println!("  All robots: RoboterBereit=TRUE");
            }
            "status" | "st" => print_status(&sim, &krls),
            "q" | "quit" | "exit" => break,
            _ => println!("Unknown: {}", cmd),
        }
    }
    Ok(())
}

fn print_status(sim: &CellSimulator, krls: &[RobotKrl]) {
    println!("  Cycle: {}", sim.cycle_count);
    for (i, r) in sim.robots.iter().enumerate() {
        let krl_info = if krls[i].active_folge > 0 {
            let st = krls[i].state();
            if st.complete { format!("KRL Folge{} DONE", st.active_folge) }
            else { format!("KRL Folge{} {}/{}", st.active_folge, st.pc, st.total) }
        } else { String::new() };
        println!("  [{}] {} — {} Seq={} {} {}", i, r.robot_id, r.phase, r.sequence, krl_info,
            if r.is_home_cycle { "(home)" } else { "" });
    }
}

fn load_tool_configs(config_path: &std::path::Path) -> Vec<ToolConfig> {
    let content = match std::fs::read_to_string(config_path) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };
    #[derive(serde::Deserialize)]
    struct ConfigWithTools {
        #[serde(default)]
        tools: Vec<ToolConfig>,
    }
    match serde_yaml::from_str::<ConfigWithTools>(&content) {
        Ok(c) => c.tools,
        Err(_) => Vec::new(),
    }
}

pub type SyncMap = std::collections::HashMap<String, Vec<Vec<String>>>;

fn load_home_poses(movements_dir: &std::path::Path, robots: &[cell::RobotState]) -> HashMap<String, kuka_krl_parser::types::KrlPosition> {
    #[derive(serde::Deserialize)]
    struct HomePoseEntry {
        coords: [f64; 3],
        ori: [f64; 3],
        #[serde(default)]
        s: i32,
        #[serde(default)]
        t: i32,
    }

    let mut poses = HashMap::new();
    for robot in robots {
        let path = movements_dir.join(format!("R{}", robot.robot_id)).join("home-pose.yaml");
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(_) => continue,
        };
        if let Ok(entry) = serde_yaml::from_str::<HomePoseEntry>(&content) {
            poses.insert(robot.robot_id.clone(), kuka_krl_parser::types::KrlPosition {
                x: entry.coords[0],
                y: entry.coords[1],
                z: entry.coords[2],
                a: entry.ori[0],
                b: entry.ori[1],
                c: entry.ori[2],
                s: entry.s,
                t: entry.t,
            });
        }
    }
    poses
}

/// Load home joints from scene.yaml's `home_pose_deg` (degrees), keyed by cell
/// robot id. The scene names robots with the upstream generator ids, so each
/// cell robot is matched by its `scene_id` (falling back to its own id). The
/// scene is `--scene` when given (a FactoryOS reference cell has no scene.yaml
/// of its own: its scene is the Robo Fitter's), else `<cell>/scene.yaml`, then
/// `<env>/scene.yaml`.
fn load_scene_home_joints(
    scene: Option<&std::path::Path>,
    cell_dir: &std::path::Path,
    env_dir: &std::path::Path,
    robots: &[cell::RobotState],
) -> HashMap<String, [f64; 6]> {
    #[derive(serde::Deserialize)]
    struct SceneRobot {
        robot_id: String,
        #[serde(default)]
        home_pose_deg: Option<Vec<f64>>,
    }
    #[derive(serde::Deserialize)]
    struct SceneDoc {
        #[serde(default)]
        robots: Vec<SceneRobot>,
    }

    let mut result = HashMap::new();

    // Find and parse the scene file.
    let candidates: Vec<std::path::PathBuf> = scene
        .map(std::path::Path::to_path_buf)
        .into_iter()
        .chain([cell_dir.join("scene.yaml"), env_dir.join("scene.yaml")])
        .collect();
    let content = candidates
        .iter()
        .find_map(|p| std::fs::read_to_string(p).ok());
    let doc = match content.and_then(|c| serde_yaml::from_str::<SceneDoc>(&c).ok()) {
        Some(d) => d,
        None => return result,
    };

    // scene robot_id -> home joints (radians)
    let mut by_scene_id: HashMap<String, [f64; 6]> = HashMap::new();
    for r in &doc.robots {
        if let Some(deg) = &r.home_pose_deg {
            if deg.len() == 6 {
                let mut rad = [0.0f64; 6];
                for k in 0..6 {
                    rad[k] = deg[k].to_radians();
                }
                by_scene_id.insert(r.robot_id.clone(), rad);
            }
        }
    }

    for robot in robots {
        let key = robot.scene_id.clone().unwrap_or_else(|| robot.robot_id.clone());
        if let Some(joints) = by_scene_id.get(&key) {
            result.insert(robot.robot_id.clone(), *joints);
        }
    }
    result
}

fn load_home_joints(movements_dir: &std::path::Path, robots: &[cell::RobotState]) -> HashMap<String, [f64; 6]> {
    #[derive(serde::Deserialize)]
    struct HomePoseWithJoints {
        #[serde(default)]
        home_joints_rad: Option<Vec<f64>>,
    }

    let mut joints = HashMap::new();
    for robot in robots {
        let path = movements_dir.join(format!("R{}", robot.robot_id)).join("home-pose.yaml");
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(_) => continue,
        };
        if let Ok(entry) = serde_yaml::from_str::<HomePoseWithJoints>(&content) {
            if let Some(j) = entry.home_joints_rad {
                if j.len() == 6 {
                    joints.insert(robot.robot_id.clone(), [j[0], j[1], j[2], j[3], j[4], j[5]]);
                }
            }
        }
    }
    joints
}

fn load_sync_map(config_path: &std::path::Path) -> SyncMap {
    let content = match std::fs::read_to_string(config_path) {
        Ok(c) => c,
        Err(_) => return SyncMap::new(),
    };
    #[derive(serde::Deserialize)]
    struct ConfigWithSync {
        #[serde(default)]
        sync_map: SyncMap,
    }
    match serde_yaml::from_str::<ConfigWithSync>(&content) {
        Ok(c) => c.sync_map,
        Err(_) => SyncMap::new(),
    }
}

#[cfg(test)]
mod args_tests {
    use super::Args;
    use clap::Parser;

    /// The server listens on loopback unless asked for another address (`--host`, or
    /// `CELL_RUNNER_HOST` in the environment); without `--env` the cell is its own environment.
    #[test]
    fn the_server_listens_on_loopback_unless_asked() {
        let args = Args::try_parse_from(["cell-runner", "--cell", "cell", "--server"]).unwrap();
        if std::env::var_os("CELL_RUNNER_HOST").is_none() {
            assert_eq!(args.host, std::net::Ipv4Addr::LOCALHOST);
        }
        assert_eq!(args.env, None);
        let args = Args::try_parse_from(["cell-runner", "--cell", "cell", "--server", "--host", "0.0.0.0"]).unwrap();
        assert!(args.host.is_unspecified());
        let args = Args::try_parse_from(["cell-runner", "--cell", "cell", "--host", "::1", "--env", "env"]).unwrap();
        assert_eq!((args.host, args.env.as_deref()), (std::net::Ipv6Addr::LOCALHOST.into(), Some(std::path::Path::new("env"))));
        assert!(Args::try_parse_from(["cell-runner", "--cell", "cell", "--host", "localhost"]).is_err(), "an address, not a name");
    }
}

#[cfg(test)]
mod home_joint_tests {
    use super::load_scene_home_joints;

    /// A FactoryOS reference cell has no scene.yaml of its own: its home poses come from `--scene`
    /// (the Robo Fitter's scene), not silently from nowhere; without `--scene` the cell's own wins.
    #[test]
    fn home_poses_come_from_the_scene_argument_first() {
        let dir = std::env::temp_dir().join(format!("cell-runner-home-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        let (cell, fitted) = (dir.join("cell"), dir.join("fitted-robots"));
        std::fs::create_dir_all(&cell).unwrap();
        std::fs::create_dir_all(&fitted).unwrap();
        let cell_yaml = cell.join("cell.yaml");
        std::fs::write(&cell_yaml, "name: t\nrobots:\n  - id: robot1\n    scene_id: kr\n    name: KR\n").unwrap();
        let sim = crate::cell::CellSimulator::from_run_config(&cell_yaml).expect("the cell config parses");
        let scene = fitted.join("jam2.yaml");
        std::fs::write(&scene, "robots:\n- robot_id: kr\n  home_pose_deg: [0, -90, 90, 0, 45, 0]\n").unwrap();

        let homes = load_scene_home_joints(Some(&scene), &cell, &cell, &sim.robots);
        let joints = homes.get("robot1").expect("robot1's home from --scene");
        assert!((joints[1] + std::f64::consts::FRAC_PI_2).abs() < 1e-9 && (joints[4] - std::f64::consts::FRAC_PI_4).abs() < 1e-9);
        assert!(load_scene_home_joints(None, &cell, &cell, &sim.robots).is_empty(), "the cell has no scene of its own");

        // A legacy cell's own scene.yaml when no --scene is given.
        std::fs::write(cell.join("scene.yaml"), "robots:\n- robot_id: kr\n  home_pose_deg: [10, 0, 0, 0, 0, 0]\n").unwrap();
        let own = load_scene_home_joints(None, &cell, &cell, &sim.robots);
        assert!((own["robot1"][0] - 10f64.to_radians()).abs() < 1e-9);
        std::fs::remove_dir_all(&dir).ok();
    }
}
