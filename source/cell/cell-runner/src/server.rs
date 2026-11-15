use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use axum::extract::ws::{Message, WebSocket};
use axum::extract::{State, WebSocketUpgrade};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures::stream::StreamExt;
use futures::{FutureExt, SinkExt};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tower_http::cors::CorsLayer;
use tower_http::services::ServeDir;

use crate::bridge;
use crate::cell::{CellSimulator, Phase, SignalValue};
use crate::krl::{RobotKrl, StepStatus};
use crate::tools::ToolSimulator;
use crate::traj::TrajectoryProvider;
use crate::trajectory::Playback;

pub struct IoNames {
    pub inputs: HashMap<u32, String>,
    pub outputs: HashMap<u32, String>,
}

impl IoNames {
    pub fn load(json_str: &str) -> Self {
        let mut inputs = HashMap::new();
        let mut outputs = HashMap::new();
        if let Ok(doc) = serde_json::from_str::<Value>(json_str) {
            if let Some(obj) = doc.get("inputs").and_then(|v| v.as_object()) {
                for (k, v) in obj {
                    if let (Ok(ch), Some(name)) = (k.parse::<u32>(), v.as_str()) {
                        inputs.insert(ch, name.to_string());
                    }
                }
            }
            if let Some(obj) = doc.get("outputs").and_then(|v| v.as_object()) {
                for (k, v) in obj {
                    if let (Ok(ch), Some(name)) = (k.parse::<u32>(), v.as_str()) {
                        outputs.insert(ch, name.to_string());
                    }
                }
            }
        }
        Self { inputs, outputs }
    }
}

#[derive(Deserialize)]
struct Request {
    id: u64,
    #[serde(rename = "type")]
    request_type: String,
    #[serde(default)]
    params: Value,
}

#[derive(Serialize)]
struct Response {
    id: u64,
    ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

impl Response {
    fn success(id: u64, result: Value) -> Self { Self { id, ok: true, result: Some(result), error: None } }
    fn error(id: u64, msg: impl Into<String>) -> Self { Self { id, ok: false, result: None, error: Some(msg.into()) } }
}

struct CellState {
    sim: CellSimulator,
    krls: Vec<RobotKrl>,
    tools: ToolSimulator,
    running: bool,
    speed: f64,
    krl_steps: usize,
    home_poses: std::collections::HashMap<String, kuka_krl_parser::types::KrlPosition>,
    providers: HashMap<String, crate::traj::LayeredTrajectoryProvider>,
    /// The cell directory, used to resolve `trajectory_load` paths (and to
    /// confine them: a plan path may never escape this directory).
    cell_dir: PathBuf,
    /// The active TRAJECTORY motion source, or `None` when the cell is driven by
    /// the KRL interpreter (the original behaviour).
    ///
    /// While this is `Some`, the scheduled joint angles REPLACE the interpreter's
    /// in the telemetry every consumer already reads — including the 3D viewer —
    /// so trajectory playback needs no separate publishing path. The KRL state is
    /// left untouched, so `trajectory_unload` restores it exactly.
    trajectory: Option<Playback>,
    /// Scene collision checker built once at startup (Req 3.1), consumed by the
    /// `preview_trajectory` handler (task 9.2). `None` when the checker could
    /// not be built — collision checking then degrades gracefully rather than
    /// failing the server (never-fail-hard). Feature-gated on `collision` so the
    /// default build carries no collision types.
    #[cfg(feature = "collision")]
    #[allow(dead_code)] // read by preview_trajectory in task 9.2
    scene_checker: Option<crate::collision::SceneCollisionChecker>,
}

#[derive(Clone)]
pub struct AppState {
    cell: Arc<Mutex<CellState>>,
    io_names: Arc<IoNames>,
    sync_map_json: Arc<Value>,
}

pub fn run_server(
    sim: CellSimulator,
    krls: Vec<RobotKrl>,
    tool_sim: ToolSimulator,
    // The address to listen on (`--host`: loopback unless asked for).
    host: IpAddr,
    port: u16,
    krl_steps: usize,
    io_names_json: &str,
    sync_map: crate::SyncMap,
    env_dir: PathBuf,
    home_poses: std::collections::HashMap<String, kuka_krl_parser::types::KrlPosition>,
    providers: HashMap<String, crate::traj::LayeredTrajectoryProvider>,
    // The cell directory. Used to resolve `trajectory_load` plan paths, and (in
    // `collision` builds) to build the collision scene checker.
    cell_dir: PathBuf,
    // Optional scene file, used only to build the collision scene checker.
    #[cfg(feature = "collision")] scene: Option<PathBuf>,
) -> Result<()> {
    let io_names = Arc::new(IoNames::load(io_names_json));
    let sync_map_json = Arc::new(serde_json::to_value(&sync_map).unwrap_or(Value::Null));

    // Build the collision scene checker alongside the trajectory providers
    // (Req 3.1). Borrow `sim` before it is moved into `CellState` below. The
    // helper never panics and returns `None` on any failure so startup is never
    // aborted (never-fail-hard contract).
    #[cfg(feature = "collision")]
    let scene_checker = build_scene_checker(&sim, &cell_dir, &env_dir, scene.as_deref());

    let state = Arc::new(Mutex::new(CellState {
        sim,
        krls,
        tools: tool_sim,
        running: false,
        speed: 1.0,
        krl_steps,
        home_poses,
        providers,
        cell_dir,
        trajectory: None,
        #[cfg(feature = "collision")]
        scene_checker,
    }));

    // Background simulation thread
    let sim_state = Arc::clone(&state);
    thread::spawn(move || {
        let cycle_duration = Duration::from_millis(10);
        loop {
            let start = Instant::now();

            let should_run = {
                let mut s = sim_state.lock().unwrap();
                // The TRAJECTORY motion source owns the robots while a plan is
                // loaded: its playhead advances on wall time (scaled by the
                // cell's existing `speed`) and the KRL cycle is left alone, so
                // unloading restores the interpreter exactly as it was.
                if s.trajectory.is_some() {
                    let speed = s.speed;
                    if let Some(playback) = s.trajectory.as_mut() {
                        playback.advance(speed);
                    }
                    false
                } else if s.running {
                    run_one_cycle(&mut s);
                    true
                } else {
                    // Even when stopped, advance any in-progress motions
                    // (triggered by step_to_motion) so the viewer animates them
                    advance_pending_motions(&mut s);
                    false
                }
            };

            if should_run {
                let speed = sim_state.lock().unwrap().speed;
                let target = Duration::from_secs_f64(cycle_duration.as_secs_f64() / speed);
                let elapsed = start.elapsed();
                if elapsed < target {
                    thread::sleep(target - elapsed);
                }
            } else {
                thread::sleep(Duration::from_millis(10));
            }
        }
    });

    let app_state = AppState {
        cell: state,
        io_names,
        sync_map_json,
    };

    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(async move {
        let app = Router::new()
            .route("/ws", get(ws_handler))
            .nest_service("/env", ServeDir::new(&env_dir))
            .layer(loopback_cors())
            .with_state(app_state);

        let listener = tokio::net::TcpListener::bind((host, port)).await?;
        let addr = listener.local_addr()?;
        let local = local_address(addr);
        eprintln!("Cell runner server listening on http://{}", addr);
        eprintln!("  WebSocket: ws://{}/ws", local);
        eprintln!("  Static files: http://{}/env/", local);
        eprintln!("  Simulation runs in background. Connect a viewer to control.");

        axum::serve(listener, app).await?;
        Ok::<(), anyhow::Error>(())
    })?;

    Ok(())
}

/// CORS for pages served from this machine only, e.g. a viewer on another loopback port reading `/env`.
/// Any method, request header and exposed header, as the permissive layer before it allowed, but only for an
/// `Origin` whose host is loopback. Another site's page open in the same browser gets no CORS headers,
/// so it can neither read a response nor send a preflighted request (a JSON `PUT` or `POST`). No
/// credentials either way.
fn loopback_cors() -> CorsLayer {
    use tower_http::cors::{AllowOrigin, Any};
    CorsLayer::new()
        .allow_origin(AllowOrigin::predicate(|origin, _| loopback_origin(origin.as_bytes())))
        .allow_methods(Any)
        .allow_headers(Any)
        .expose_headers(Any)
}

/// Whether an `Origin` header (`http[s]://host[:port]`) names this machine: `localhost`, 127.0.0.0/8 or `[::1]`.
fn loopback_origin(origin: &[u8]) -> bool {
    let Ok(origin) = std::str::from_utf8(origin) else { return false };
    let Some(rest) = origin.strip_prefix("http://").or_else(|| origin.strip_prefix("https://")) else { return false };
    let (host, port) = match rest.strip_prefix('[') {
        Some(v6) => match v6.split_once(']') {
            Some(split) => split,
            None => return false,
        },
        None => rest.split_at(rest.find(':').unwrap_or(rest.len())),
    };
    let port_ok = port.is_empty()
        || port.strip_prefix(':').is_some_and(|p| !p.is_empty() && p.len() <= 5 && p.bytes().all(|b| b.is_ascii_digit()));
    port_ok && (host.eq_ignore_ascii_case("localhost") || host.parse::<std::net::IpAddr>().is_ok_and(|ip| ip.is_loopback()))
}

/// Where a client on this machine reaches a listener on `addr`: that address, or loopback for a
/// wildcard one (`0.0.0.0`, `[::]`).
fn local_address(addr: SocketAddr) -> SocketAddr {
    match addr.ip() {
        IpAddr::V4(ip) if ip.is_unspecified() => (Ipv4Addr::LOCALHOST, addr.port()).into(),
        IpAddr::V6(ip) if ip.is_unspecified() => (Ipv6Addr::LOCALHOST, addr.port()).into(),
        _ => addr,
    }
}

/// Build the scene collision checker from the loaded robots (Req 3.1).
///
/// Constructs one [`RobotCollisionSpec`] per robot, resolving:
/// * **URDF** — mirrors the trajectory-provider load in `main.rs`: prefer the
///   robot's own dressed URDF (path relative to the cell directory) when the
///   cell declares one, else fall back to `<env>/model/<model>.urdf`.
/// * **SRDF** — `RobotState` carries no SRDF path, so a sibling `<stem>.srdf`
///   next to the resolved URDF is used when present; otherwise no SRDF is
///   supplied and all self-collision pairs are retained (Req 1.4).
/// * **Base transform** — `world <- robot base` from `scene.yaml`: the robot's
///   position raised by its pedestal and turned by its orientation
///   ([`crate::cell_environment::load_robot_bases`]); a robot absent or
///   malformed in the scene defaults to the identity.
/// * **Environment** — the scene's `static_models:` meshes, plus the cell's
///   fixtures, pedestals and floor as boxes
///   ([`crate::cell_environment::load_environment_boxes`]).
///
/// This never panics and returns `None` when the checker cannot be built, so
/// the server continues without collision checking rather than aborting
/// startup (never-fail-hard contract).
#[cfg(feature = "collision")]
fn build_scene_checker(
    sim: &CellSimulator,
    cell_dir: &std::path::Path,
    env_dir: &std::path::Path,
    scene: Option<&std::path::Path>,
) -> Option<crate::collision::SceneCollisionChecker> {
    use crate::collision::{CollisionConfig, EnvironmentRole, RobotCollisionSpec, SceneCollisionChecker};
    use nalgebra::Isometry3;

    // World base poses from scene.yaml: position raised by the robot's pedestal,
    // turned by its orientation. A robot the scene lacks stays at the origin.
    let bases = crate::cell_environment::load_robot_bases(scene, &sim.robots);

    let model_dir = env_dir.join("model");
    let specs: Vec<RobotCollisionSpec> = sim
        .robots
        .iter()
        .map(|robot| {
            let urdf = match &robot.urdf {
                Some(rel) => cell_dir.join(rel),
                None => model_dir.join(format!("{}.urdf", robot.model)),
            };
            // Beside the URDF; else (a FactoryOS reference cell, whose URDFs are the
            // catalogue's) the one Prepare Cell wrote into the cell's agentic_planner/srdf/.
            let srdf = {
                let candidate = urdf.with_extension("srdf");
                let in_cell = urdf
                    .file_stem()
                    .map(|stem| cell_dir.join("agentic_planner/srdf").join(stem).with_extension("srdf"));
                std::iter::once(candidate).chain(in_cell).find(|p| p.is_file())
            };
            let base_transform = bases
                .get(&robot.robot_id)
                .map_or_else(Isometry3::identity, |b| b.pose);

            RobotCollisionSpec::new(robot.robot_id.clone(), urdf, srdf, base_transform)
        })
        .collect();

    // Parse the scene's `static_models:` list into real environment specs.
    // On failure, warn and continue with an empty `Vec` — never abort startup
    // (never-fail-hard contract).
    let env_specs: Vec<crate::collision::EnvironmentCollisionSpec> =
        match crate::export::load_static_models(scene) {
            Ok(specs) => specs,
            Err(e) => {
                eprintln!(
                    "WARN: collision: failed to load static environment models ({e}); \
                     continuing with zero environment objects"
                );
                Vec::new()
            }
        };

    // The cell's fixtures (obstacles.yaml boxes, else their URDF collision
    // geometry), the robots' pedestals and the floor, as boxes.
    let boxes = crate::cell_environment::load_environment_boxes(scene, cell_dir, &bases);

    match SceneCollisionChecker::build_with_boxes(&specs, &env_specs, &boxes, CollisionConfig::default()) {
        Ok(checker) => {
            let (mut workpieces, mut supports) = (0, 0);
            for (_, role) in checker.environment_objects() {
                match role {
                    EnvironmentRole::Workpiece => workpieces += 1,
                    EnvironmentRole::Support(_) => supports += 1,
                    EnvironmentRole::Obstacle => {}
                }
            }
            eprintln!(
                "  Collision checker: built for {} robot(s), {} environment object(s) \
                 ({} workpiece, {} pedestal/floor)",
                checker.robot_count(),
                checker.environment_object_count(),
                workpieces,
                supports,
            );
            for error in checker.environment_errors() {
                eprintln!(
                    "WARN: collision: {}: {}",
                    error.model_id,
                    error.error.as_deref().unwrap_or("failed to load")
                );
            }
            Some(checker)
        }
        Err(e) => {
            eprintln!(
                "WARN: collision: failed to build scene checker ({e}); \
                 continuing without collision checking"
            );
            None
        }
    }
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_ws_client(socket, state))
}

async fn handle_ws_client(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();

    while let Some(Ok(msg)) = receiver.next().await {
        match msg {
            Message::Text(text) => {
                let request: Request = match serde_json::from_str(&text) {
                    Ok(r) => r,
                    Err(e) => {
                        let resp = Response::error(0, e.to_string());
                        let json = serde_json::to_string(&resp).unwrap_or_default();
                        let _ = sender.send(Message::Text(json.into())).await;
                        continue;
                    }
                };

                if request.request_type == "subscribe_positions" {
                    let fps = request.params.get("fps")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(30.0)
                        .clamp(1.0, 120.0);
                    let resp = Response::success(request.id, serde_json::json!({"subscribed": true, "fps": fps}));
                    let json = serde_json::to_string(&resp).unwrap_or_default();
                    let _ = sender.send(Message::Text(json.into())).await;
                    stream_positions(sender, receiver, state, fps).await;
                    return;
                }

                let mut response = dispatch(&request, &state.cell, &state.io_names);
                if let Some(ref mut result) = response.result {
                    if let Some(s) = result.get_mut("state") {
                        s["sync_map"] = (*state.sync_map_json).clone();
                    } else if result.get("robots").is_some() {
                        result["sync_map"] = (*state.sync_map_json).clone();
                    }
                }
                let json = serde_json::to_string(&response).unwrap_or_default();
                if sender.send(Message::Text(json.into())).await.is_err() {
                    return;
                }
            }
            Message::Close(_) => return,
            Message::Ping(data) => { let _ = sender.send(Message::Pong(data)).await; }
            _ => {}
        }
    }
}

async fn stream_positions(
    mut sender: futures::stream::SplitSink<WebSocket, Message>,
    mut receiver: futures::stream::SplitStream<WebSocket>,
    state: AppState,
    fps: f64,
) {
    let interval = Duration::from_secs_f64(1.0 / fps);

    loop {
        let start = Instant::now();

        // Check for close/unsubscribe without blocking
        match receiver.next().now_or_never() {
            Some(Some(Ok(Message::Close(_)))) => break,
            Some(Some(Ok(Message::Text(text)))) => {
                if let Ok(req) = serde_json::from_str::<Request>(&text) {
                    if req.request_type == "unsubscribe" {
                        let resp = Response::success(req.id, serde_json::json!({"unsubscribed": true}));
                        let json = serde_json::to_string(&resp).unwrap_or_default();
                        let _ = sender.send(Message::Text(json.into())).await;
                        break;
                    }
                }
            }
            Some(Some(Ok(Message::Ping(data)))) => { let _ = sender.send(Message::Pong(data)).await; }
            Some(Some(Err(_))) | Some(None) => break,
            _ => {}
        }

        let positions = {
            let s = state.cell.lock().unwrap();
            build_positions_frame(&s)
        };

        let json = serde_json::to_string(&positions).unwrap_or_default();
        if sender.send(Message::Text(json.into())).await.is_err() {
            break;
        }

        let elapsed = start.elapsed();
        if elapsed < interval {
            tokio::time::sleep(interval - elapsed).await;
        }
    }
}

fn run_one_cycle(s: &mut CellState) {
    s.sim.step();
    for (i, krl) in s.krls.iter_mut().enumerate() {
        let had_motion = krl.interpreter.as_ref().map_or(false, |interp| interp.motion_state.is_some());
        bridge::sync_step(&mut s.sim.robots[i], krl, s.krl_steps, &mut s.tools, i);
        // Fallback seed for the motion-start IK below: the robot's home joints.
        // Captured before borrowing `krl.interpreter` mutably. Without this the
        // first motion (before `current_joints` is populated) seeds IK from an
        // all-zeros configuration, which diverges for a KUKA arm and forces a
        // permanent heuristic downgrade. Mirrors presolve_motion_joints_for_robot
        // and solve_missing_joints, which already fall back to home_joints.
        let home_joints = krl.home_joints;
        if !had_motion {
            if let Some(ref mut interp) = krl.interpreter {
                if interp.motion_state.is_some() {
                    if let Some(provider) = s.providers.get_mut(&s.sim.robots[i].model) {
                        if let Some(crate::interpreter::Instruction::Motion { position, context, approximate, .. }) = interp.current_instruction().cloned() {
                            let kind = interp.current_motion_kind().unwrap_or(crate::traj::MotionKind::Ptp);
                            let start_pos = interp.motion_state.as_ref().unwrap().start_position.clone();
                            let start_pose = [start_pos.x, start_pos.y, start_pos.z, start_pos.a, start_pos.b, start_pos.c];
                            let target_pose = [position.x, position.y, position.z, position.a, position.b, position.c];
                            let seed_joints = interp.current_joints.or(home_joints).unwrap_or([0.0f64; 6]);
                            let req = crate::traj::MotionRequest {
                                kind,
                                start_pose,
                                target_pose,
                                vel_percent: context.vel_ptp,
                                approximate,
                                seed_joints,
                            };
                            let sol = provider.resolve_motion(&req);
                            // Log the downgrade note locally when a tier below
                            // RuckigOtg was used (Req 12.2). The telemetry wire
                            // payload is intentionally left unchanged (Req 12.4).
                            if sol.tier != crate::traj::SolverTier::RuckigOtg {
                                if let Some(ref note) = sol.note {
                                    eprintln!(
                                        "  [{}] motion downgraded to {}: {}",
                                        s.sim.robots[i].model, sol.tier, note
                                    );
                                }
                            }
                            interp.set_motion_duration(sol.duration_s);
                            // Seed current_joints from the resolved path start so
                            // visualization tracks the produced trajectory.
                            if let Some(start_joints) = sol.path.sample(0.0) {
                                interp.current_joints = Some(start_joints);
                            }
                            if let Some(ref mut ms) = interp.motion_state {
                                ms.joint_trajectory = Some(sol.path);
                            }
                        }
                    }
                }
            }
        }
    }
    solve_missing_joints(s);
}

fn solve_missing_joints(s: &mut CellState) {
    for (i, krl) in s.krls.iter_mut().enumerate() {
        let provider = match s.providers.get_mut(&s.sim.robots[i].model) {
            Some(p) => p,
            None => continue,
        };
        if let Some(ref mut interp) = krl.interpreter {
            // Case 1: no joints at all — cold solve
            let no_joints = interp.current_joints.is_none()
                || interp.current_joints == Some([0.0; 6]);
            // Case 2: motion in progress but no joint trajectory — track position with IK
            let motion_without_trajectory = interp.motion_state.as_ref()
                .map_or(false, |ms| ms.joint_trajectory.is_none());

            if no_joints || motion_without_trajectory {
                let pos = &interp.current_position;
                let target = [pos.x, pos.y, pos.z, pos.a, pos.b, pos.c];
                // Prefer a warm seed (existing joints or home joints); the
                // provider's solve_pose runs a warm attempt followed by its own
                // cold multi-seed sweep, so a zero seed still resolves cold.
                let seed = interp.current_joints
                    .filter(|j| *j != [0.0; 6])
                    .or(krl.home_joints)
                    .unwrap_or([0.0; 6]);
                if let Some(joints) = provider.solve_pose(&target, &seed) {
                    interp.current_joints = Some(joints);
                }
            }
        }
    }
}

fn presolve_motion_joints_for_robot(s: &mut CellState, robot_idx: usize) {
    let home_j = s.krls[robot_idx].home_joints;
    let model_name = s.sim.robots[robot_idx].model.clone();
    let provider = match s.providers.get_mut(&model_name) { Some(p) => p, None => return };
    let interp = match s.krls[robot_idx].interpreter.as_mut() { Some(i) => i, None => return };
    if interp.motion_state.is_none() { return; }
    if let Some(crate::interpreter::Instruction::Motion { position, context, approximate, .. }) = interp.current_instruction().cloned() {
        let kind = interp.current_motion_kind().unwrap_or(crate::traj::MotionKind::Ptp);
        let start_pos = interp.motion_state.as_ref().unwrap().start_position.clone();
        let start_pose = [start_pos.x, start_pos.y, start_pos.z, start_pos.a, start_pos.b, start_pos.c];
        let target_pose = [position.x, position.y, position.z, position.a, position.b, position.c];
        let seed_joints = interp.current_joints.or(home_j).unwrap_or([0.0f64; 6]);
        let req = crate::traj::MotionRequest {
            kind,
            start_pose,
            target_pose,
            vel_percent: context.vel_ptp,
            approximate,
            seed_joints,
        };
        let sol = provider.resolve_motion(&req);
        if sol.tier != crate::traj::SolverTier::RuckigOtg {
            if let Some(ref note) = sol.note {
                eprintln!("  [{}] motion downgraded to {}: {}", model_name, sol.tier, note);
            }
        }
        interp.set_motion_duration(sol.duration_s);
        // Track the motion end configuration (parity with the prior presolve,
        // which seeded current_joints from the solved end endpoint).
        if let Some(end_joints) = sol.path.sample(1.0) {
            interp.current_joints = Some(end_joints);
        }
        if let Some(ref mut ms) = interp.motion_state {
            ms.joint_trajectory = Some(sol.path);
        }
    }
}


fn advance_pending_motions(s: &mut CellState) {
    for krl in s.krls.iter_mut() {
        if let Some(ref mut interp) = krl.interpreter {
            if interp.motion_state.is_some() {
                let _ = interp.step();
            }
        }
    }
}

fn build_positions_frame(s: &CellState) -> Value {
    let robots: Vec<Value> = s.sim.robots.iter().enumerate().map(|(idx, r)| {
        let (position, joints) = if let Some(ref krl) = s.krls.get(idx) {
            if let Some(ref interp) = krl.interpreter {
                let pos = &interp.current_position;
                let j = interp.current_joints.or(krl.home_joints).unwrap_or([0.0; 6]);
                (serde_json::json!({
                    "x": pos.x, "y": pos.y, "z": pos.z,
                    "a": pos.a, "b": pos.b, "c": pos.c,
                    "s": pos.s, "t": pos.t
                }), j)
            } else {
                let pos = &krl.position;
                let j = krl.home_joints.unwrap_or([0.0; 6]);
                (serde_json::json!({
                    "x": pos.x, "y": pos.y, "z": pos.z,
                    "a": pos.a, "b": pos.b, "c": pos.c,
                    "s": pos.s, "t": pos.t
                }), j)
            }
        } else {
            (serde_json::json!({"x": 0, "y": 0, "z": 0, "a": 0, "b": 0, "c": 0, "s": 0, "t": 0}), [0.0; 6])
        };
        // A playing behaviour tree's trajectory moves the robots it plans, as the state's
        // `overlay_trajectory_joints`: the stream would otherwise stand still during playback.
        let joints = s.trajectory.as_ref().and_then(|playback| playback.joints_for(&r.robot_id)).unwrap_or(joints);
        serde_json::json!({
            "index": idx,
            "name": r.name,
            "robot_id": r.robot_id,
            "model": r.model,
            "position": position,
            "joints": joints
        })
    }).collect();

    serde_json::json!({
        "type": "positions",
        "cycle": s.sim.cycle_count,
        "robots": robots
    })
}


/// Overlay the active trajectory's joint angles onto a built `state` result, and
/// attach the playback block the Trajectory Controller polls.
///
/// A robot the plan does not mention is left exactly as the KRL path reported
/// it, so a partially-planned cell still shows its other arms correctly.
fn overlay_trajectory_joints(s: &CellState, result: &mut Value) {
    let playback = match s.trajectory.as_ref() {
        Some(playback) => playback,
        None => {
            result["trajectory"] = crate::trajectory::unloaded_state_json(s.speed);
            return;
        }
    };
    if let Some(entries) = result.get_mut("krl").and_then(|v| v.as_array_mut()) {
        for entry in entries.iter_mut() {
            let robot_id = entry
                .get("robot_id")
                .and_then(|v| v.as_str())
                .map(str::to_string);
            if let Some(joints) = robot_id.and_then(|id| playback.joints_for(&id)) {
                entry["joints"] = json!(joints);
            }
        }
    }
    // `robots` carries the same joints for consumers that read it instead.
    if let Some(entries) = result.get_mut("robots").and_then(|v| v.as_array_mut()) {
        for entry in entries.iter_mut() {
            let robot_id = entry
                .get("robot_id")
                .and_then(|v| v.as_str())
                .map(str::to_string);
            if let Some(joints) = robot_id.and_then(|id| playback.joints_for(&id)) {
                entry["joints"] = json!(joints);
            }
        }
    }
    result["running"] = Value::Bool(playback.is_playing());
    result["trajectory"] = playback.state_json(s.speed);
}

/// The cell's plan folders a cell directory may link to elsewhere: a FactoryOS reference cell's
/// view (`<cell>/.view`) links `agentic_planner` (and `trajectories`) to the cell's own folder.
const PLAN_DIRS: [&str; 2] = ["agentic_planner", "trajectories"];

/// Resolve a `trajectory_load` path inside the cell directory (or one of its plan folders,
/// [`PLAN_DIRS`], wherever they link to).
///
/// Refuses anything that escapes them, so a panel (or any WS client) cannot use
/// this verb to read arbitrary files off the host.
fn resolve_plan_path(cell_dir: &std::path::Path, rel: &str) -> Result<PathBuf, String> {
    let joined = if std::path::Path::new(rel).is_absolute() {
        PathBuf::from(rel)
    } else {
        cell_dir.join(rel)
    };
    let canonical_root = std::fs::canonicalize(cell_dir)
        .map_err(|e| format!("cell directory is unreadable: {e}"))?;
    let canonical = std::fs::canonicalize(&joined)
        .map_err(|_| format!("plan not found: {rel}"))?;
    let in_plan_dir = PLAN_DIRS
        .iter()
        .filter_map(|dir| std::fs::canonicalize(cell_dir.join(dir)).ok())
        .any(|dir| canonical.starts_with(dir));
    if !canonical.starts_with(&canonical_root) && !in_plan_dir {
        return Err(format!("plan path escapes the cell directory: {rel}"));
    }
    if !canonical.is_file() {
        return Err(format!("not a file: {rel}"));
    }
    Ok(canonical)
}

/// The `trajectory_list` result: every `*.json` under `<cell>/trajectories`.
fn list_trajectory_plans(s: &CellState) -> Value {
    let dir = s.cell_dir.join("trajectories");
    let mut plans: Vec<Value> = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&dir) {
        let mut names: Vec<String> = entries
            .filter_map(|e| e.ok())
            .filter(|e| e.path().is_file())
            .filter_map(|e| e.file_name().to_str().map(str::to_string))
            .filter(|name| name.ends_with(".json"))
            .collect();
        names.sort();
        for name in names {
            let full = dir.join(&name);
            let size = std::fs::metadata(&full).map(|m| m.len()).unwrap_or(0);
            let mut entry = json!({
                "name": name,
                "path": format!("trajectories/{name}"),
                "size_bytes": size,
            });
            match std::fs::read_to_string(&full)
                .map_err(|e| e.to_string())
                .and_then(|text| crate::trajectory::parse_plan(&text))
            {
                Ok(parsed) => {
                    entry["tree_id"] = json!(parsed.tree_id);
                    entry["cell_id"] = json!(parsed.cell_id);
                    entry["operations"] = json!(parsed.num_operations);
                }
                Err(e) => {
                    entry["error"] = json!(e);
                }
            }
            plans.push(entry);
        }
    }
    json!({
        "plans": plans,
        "loaded": s.trajectory.as_ref().and_then(|p| p.plan.clone()),
    })
}

/// Per-robot home configuration, so a schedule knows where each arm sits before
/// its first segment. Sourced from the KRL home poses when available.
fn home_joints_by_robot(s: &CellState) -> HashMap<String, [f64; 6]> {
    let mut home = HashMap::new();
    for (idx, robot) in s.sim.robots.iter().enumerate() {
        if let Some(joints) = s.krls.get(idx).and_then(|krl| krl.home_joints) {
            home.insert(robot.robot_id.clone(), joints);
        }
    }
    home
}

/// Compile a plan and install it as the active motion source.
fn load_trajectory(req: &Request, s: &mut CellState) -> Response {
    // Either a path relative to the cell directory, or an inline plan object.
    let parsed = if let Some(rel) = req.params.get("path").and_then(|v| v.as_str()) {
        let full = match resolve_plan_path(&s.cell_dir, rel) {
            Ok(p) => p,
            Err(e) => return Response::error(req.id, e),
        };
        match crate::trajectory::parse_plan_file(&full) {
            Ok(parsed) => (parsed, Some(rel.to_string())),
            Err(e) => return Response::error(req.id, e),
        }
    } else if let Some(plan) = req.params.get("plan") {
        match crate::trajectory::parse_plan_value(plan.clone()) {
            Ok(parsed) => (parsed, None),
            Err(e) => return Response::error(req.id, e),
        }
    } else {
        return Response::error(req.id, "either `path` or `plan` is required");
    };
    let (parsed, plan_rel) = parsed;

    let robot_ids: Vec<String> = s.sim.robots.iter().map(|r| r.robot_id.clone()).collect();
    let schedule = match crate::trajectory::Schedule::build(
        &parsed.tree,
        &robot_ids,
        home_joints_by_robot(s),
    ) {
        Ok(schedule) => schedule,
        Err(e) => return Response::error(req.id, e),
    };
    eprintln!(
        "[trajectory] loaded {} — {}",
        plan_rel.clone().unwrap_or_else(|| "<inline>".into()),
        schedule.describe()
    );

    let mut playback = Playback::new(schedule, plan_rel, parsed.tree_id, parsed.cell_id);
    // Bound separately: `playback.collision = f(&playback)` would borrow
    // `playback` immutably while assigning to one of its fields.
    let collision = check_trajectory_collisions(s, &playback);
    playback.collision = collision;
    let summary = playback.summary_json();
    s.trajectory = Some(playback);
    Response::success(req.id, summary)
}

/// Collision-check a compiled trajectory before anything moves.
///
/// In a `collision` build this runs the scene checker over every robot's whole
/// resampled timeline. Without that feature (or without a checker) the result
/// records that no check was performed, rather than implying the plan is safe.
#[cfg(feature = "collision")]
fn check_trajectory_collisions(s: &mut CellState, playback: &Playback) -> Value {
    use crate::collision::SceneTrajectory;

    let checker = match s.scene_checker.as_mut() {
        Some(checker) => checker,
        None => {
            return json!({
                "checked": false,
                "reason": "no scene collision checker could be built for this cell",
            })
        }
    };

    let (paths, samples) = playback.schedule.to_joint_paths();
    let sample_count = crate::trajectory::Schedule::collision_sample_count(samples);
    let mut scene = SceneTrajectory::new(samples, sample_count);
    for (robot_id, path) in paths {
        scene.insert(robot_id, path);
    }

    let report = checker.check_trajectory(&scene);
    if let Some(reason) = report.rejected {
        return json!({ "checked": false, "reason": reason });
    }

    // Report every colliding waypoint when available, else just the first.
    let waypoints = if !report.all.is_empty() {
        report.all.clone()
    } else {
        report.first.clone().into_iter().collect()
    };
    let mut collisions: Vec<Value> = Vec::new();
    let mut model_errors: Vec<String> = Vec::new();
    for waypoint in &waypoints {
        for pair in &waypoint.report.pairs {
            collisions.push(json!({
                "progress": (waypoint.progress * 100000.0).round() / 100000.0,
                "waypoint": waypoint.waypoint,
                "object_a": pair.object_a,
                "object_b": pair.object_b,
                "kind": format!("{:?}", pair.kind),
                "distance_m": pair.distance_m,
            }));
        }
        for model in &waypoint.report.model_status {
            if let Some(error) = &model.error {
                let entry = format!("{}: {}", model.model_id, error);
                if !model_errors.contains(&entry) {
                    model_errors.push(entry);
                }
            }
        }
    }

    json!({
        "checked": true,
        "in_collision": report.collided,
        "samples": sample_count,
        "collisions": collisions,
        "unchecked_robots": report.unchecked_robots,
        "model_errors": model_errors,
    })
}

/// Non-`collision` builds cannot check anything; say so explicitly.
#[cfg(not(feature = "collision"))]
fn check_trajectory_collisions(_s: &mut CellState, _playback: &Playback) -> Value {
    json!({
        "checked": false,
        "reason": "this cell-runner was built without the `collision` feature",
    })
}

/// The `trajectory_state` result for the current mode.
fn trajectory_state(s: &CellState) -> Value {
    match s.trajectory.as_ref() {
        Some(playback) => playback.state_json(s.speed),
        None => crate::trajectory::unloaded_state_json(s.speed),
    }
}

fn dispatch(req: &Request, state: &Arc<Mutex<CellState>>, names: &IoNames) -> Response {
    match req.request_type.as_str() {
        "state" => {
            let s = state.lock().unwrap();
            let mut result = build_full_state(&s.sim, &s.krls, names);
            result["running"] = Value::Bool(s.running);
            // While a trajectory is loaded IT is the motion source, so overlay
            // its joints onto the telemetry every consumer already reads. This
            // is the single reason trajectory playback shows up in the 3D
            // viewer without touching the viewer or the RRD stream: the viewer
            // renders `krl[i].joints`, and this is where those come from.
            //
            // Patching the built result follows the same pattern as the
            // `running` override above rather than threading an override
            // through `build_full_state`/`RobotKrl::state`, keeping the KRL
            // module unaware of the second motion source.
            overlay_trajectory_joints(&s, &mut result);
            result["speed"] = serde_json::json!(s.speed);
            Response::success(req.id, result)
        }
        "start" => {
            let mut s = state.lock().unwrap();
            s.running = true;
            let mut result = serde_json::json!({"action": "started"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(true);
            result["state"]["speed"] = serde_json::json!(s.speed);
            Response::success(req.id, result)
        }
        "stop" => {
            let mut s = state.lock().unwrap();
            s.running = false;
            let mut result = serde_json::json!({"action": "stopped"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            result["state"]["speed"] = serde_json::json!(s.speed);
            Response::success(req.id, result)
        }
        "set_speed" => {
            let speed = req.params.get("speed").and_then(|v| v.as_f64()).unwrap_or(1.0);
            let speed = speed.clamp(0.1, 100.0);
            let mut s = state.lock().unwrap();
            s.speed = speed;
            Response::success(req.id, serde_json::json!({"speed": speed}))
        }
        "step" => {
            let mut s = state.lock().unwrap();
            if s.running {
                return Response::error(req.id, "cannot step while running — stop first");
            }
            run_one_cycle(&mut s);
            let mut result = serde_json::json!({"step_result": "stepped"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            Response::success(req.id, result)
        }
        "step_robot" | "step_robot_motion" => {
            let mut s = state.lock().unwrap();
            if s.running {
                return Response::error(req.id, "cannot step while running — stop first");
            }
            let robot_idx = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            if robot_idx >= s.krls.len() {
                return Response::error(req.id, "invalid robot index");
            }
            {
                let CellState { ref mut sim, ref mut krls, ref mut tools, .. } = *s;
                bridge::plc_to_krl(&sim.robots[robot_idx], &mut krls[robot_idx]);
                if krls[robot_idx].interpreter.is_some() && !krls[robot_idx].program_complete {
                    let mut status = StepStatus::Blocked;
                    for _ in 0..10 {
                        if let Some(ref mut interp) = krls[robot_idx].interpreter {
                            tools.step_robot(robot_idx, &mut interp.io);
                        }
                        status = krls[robot_idx].step_to_motion();
                        if status != StepStatus::Blocked {
                            break;
                        }
                    }
                    if status == StepStatus::Blocked {
                        if let Some(ref mut interp) = krls[robot_idx].interpreter {
                            if let Some(crate::interpreter::Instruction::WaitForInput { channel, value }) =
                                interp.current_instruction().cloned()
                            {
                                interp.io.set_input(channel, value);
                            }
                        }
                        krls[robot_idx].step_to_motion();
                    }
                }
                bridge::krl_to_plc(&krls[robot_idx], &mut sim.robots[robot_idx]);
            }
            // Pre-solve joints for the newly started motion
            presolve_motion_joints_for_robot(&mut s, robot_idx);
            let mut result = serde_json::json!({"step_result": "stepped_robot"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            Response::success(req.id, result)
        }
        "step_robot_wait" => {
            let mut s = state.lock().unwrap();
            if s.running {
                return Response::error(req.id, "cannot step while running — stop first");
            }
            let robot_idx = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            if robot_idx >= s.krls.len() {
                return Response::error(req.id, "invalid robot index");
            }
            let CellState { ref mut sim, ref mut krls, ref mut tools, .. } = *s;
            bridge::plc_to_krl(&sim.robots[robot_idx], &mut krls[robot_idx]);
            if krls[robot_idx].interpreter.is_some() && !krls[robot_idx].program_complete {
                let mut status = StepStatus::Blocked;
                for _ in 0..10 {
                    if let Some(ref mut interp) = krls[robot_idx].interpreter {
                        tools.step_robot(robot_idx, &mut interp.io);
                    }
                    status = krls[robot_idx].step_to_wait();
                    if status != StepStatus::Blocked {
                        break;
                    }
                }
                if status == StepStatus::Blocked {
                    if let Some(ref mut interp) = krls[robot_idx].interpreter {
                        if let Some(crate::interpreter::Instruction::WaitForInput { channel, value }) =
                            interp.current_instruction().cloned()
                        {
                            interp.io.set_input(channel, value);
                        }
                    }
                    krls[robot_idx].step_to_wait();
                }
            }
            bridge::krl_to_plc(&krls[robot_idx], &mut sim.robots[robot_idx]);
            let mut result = serde_json::json!({"step_result": "stepped_robot_wait"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            Response::success(req.id, result)
        }
        "run" => {
            let mut s = state.lock().unwrap();
            if s.running {
                let mut result = build_full_state(&s.sim, &s.krls, names);
                result["running"] = Value::Bool(true);
                result["speed"] = serde_json::json!(s.speed);
                return Response::success(req.id, result);
            }
            let cycles = req.params.get("cycles").and_then(|v| v.as_u64()).unwrap_or(100) as usize;
            let cycles = cycles.min(10000);
            for _ in 0..cycles {
                run_one_cycle(&mut s);
            }
            let mut result = serde_json::json!({"cycles_run": cycles});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            Response::success(req.id, result)
        }
        "reset" => {
            let mut s = state.lock().unwrap();
            s.running = false;
            s.sim.reset();
            s.tools.reset();
            let home_poses = s.home_poses.clone();
            for krl in s.krls.iter_mut() {
                krl.interpreter = None;
                krl.active_folge = 0;
                krl.program_complete = false;
                krl.io = crate::interpreter::IoState::new();
                krl.position = home_poses.get(&krl.robot_id)
                    .cloned()
                    .unwrap_or_default();
            }
            let mut result = serde_json::json!({"action": "reset"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            result["state"]["running"] = Value::Bool(false);
            Response::success(req.id, result)
        }
        "set_input" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            let signal = req.params.get("signal").and_then(|v| v.as_str()).unwrap_or("");
            let value = if let Some(b) = req.params.get("value").and_then(|v| v.as_bool()) { SignalValue::Bool(b) }
            else if let Some(n) = req.params.get("value").and_then(|v| v.as_u64()) { SignalValue::Byte(n as u8) }
            else { return Response::error(req.id, "invalid value"); };
            if robot < s.sim.robots.len() { s.sim.robots[robot].inputs.insert(signal.to_string(), value); }
            let result = serde_json::json!({"state": build_full_state(&s.sim, &s.krls, names)});
            Response::success(req.id, result)
        }
        "set_all_inputs" => {
            let mut s = state.lock().unwrap();
            let signal = req.params.get("signal").and_then(|v| v.as_str()).unwrap_or("");
            let value = if let Some(b) = req.params.get("value").and_then(|v| v.as_bool()) { SignalValue::Bool(b) }
            else if let Some(n) = req.params.get("value").and_then(|v| v.as_u64()) { SignalValue::Byte(n as u8) }
            else { return Response::error(req.id, "invalid value"); };
            for robot in &mut s.sim.robots { robot.inputs.insert(signal.to_string(), value.clone()); }
            let result = serde_json::json!({"state": build_full_state(&s.sim, &s.krls, names)});
            Response::success(req.id, result)
        }
        "set_output" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            let signal = req.params.get("signal").and_then(|v| v.as_str()).unwrap_or("");
            let value = if let Some(b) = req.params.get("value").and_then(|v| v.as_bool()) { SignalValue::Bool(b) }
            else if let Some(n) = req.params.get("value").and_then(|v| v.as_u64()) { SignalValue::Byte(n as u8) }
            else { return Response::error(req.id, "invalid value"); };
            if robot < s.sim.robots.len() { s.sim.robots[robot].outputs.insert(signal.to_string(), value); }
            let result = serde_json::json!({"state": build_full_state(&s.sim, &s.krls, names)});
            Response::success(req.id, result)
        }
        "set_krl_output" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            let channel = req.params.get("channel").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
            let value = req.params.get("value").and_then(|v| v.as_bool()).unwrap_or(false);
            if robot < s.krls.len() {
                if let Some(ref mut interp) = s.krls[robot].interpreter {
                    interp.io.set_output(channel, value);
                } else {
                    s.krls[robot].io.set_output(channel, value);
                }
            }
            let result = serde_json::json!({"state": build_full_state(&s.sim, &s.krls, names)});
            Response::success(req.id, result)
        }
        "set_krl_input" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            let channel = req.params.get("channel").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
            let value = req.params.get("value").and_then(|v| v.as_bool()).unwrap_or(false);
            if robot < s.krls.len() {
                if let Some(ref mut interp) = s.krls[robot].interpreter {
                    interp.io.set_input(channel, value);
                } else {
                    s.krls[robot].io.set_input(channel, value);
                }
            }
            let result = serde_json::json!({"state": build_full_state(&s.sim, &s.krls, names)});
            Response::success(req.id, result)
        }
        "set_all_ready" => {
            let mut s = state.lock().unwrap();
            for robot in &mut s.sim.robots { robot.inputs.insert("RoboterBereit".into(), SignalValue::Bool(true)); }
            for krl in s.krls.iter_mut() {
                if let Some(ref mut interp) = krl.interpreter { interp.io.set_output(2, true); }
            }
            let mut result = serde_json::json!({"set": "all_ready"});
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            Response::success(req.id, result)
        }
        "load_folge" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
            let folge = req.params.get("folge").and_then(|v| v.as_u64()).unwrap_or(0) as u8;
            if robot >= s.krls.len() {
                return Response::error(req.id, "invalid robot index");
            }
            if !s.krls[robot].programs.contains_key(&folge) {
                let avail: Vec<u8> = {
                    let mut v: Vec<u8> = s.krls[robot].programs.keys().copied().collect();
                    v.sort_unstable();
                    v
                };
                return Response::error(
                    req.id,
                    format!("Folge {} not available (have: {:?})", folge, avail),
                );
            }
            // Load the chosen program into the interpreter so the KRL viewer
            // shows it immediately (even while stopped).
            s.krls[robot].start_program(folge);
            // Override the coordinator's sequence selection so a subsequent
            // Start keeps running the chosen Folge instead of the default
            // production folge / home alternation.
            if robot < s.sim.robots.len() {
                s.sim.robots[robot].production_folgen = vec![folge];
                s.sim.robots[robot].current_folge_idx = 0;
                s.sim.robots[robot].is_home_cycle = false;
                s.sim.robots[robot].sequence = folge;
            }
            let mut result = serde_json::json!({
                "action": "load_folge", "robot": robot, "folge": folge
            });
            result["state"] = build_full_state(&s.sim, &s.krls, names);
            Response::success(req.id, result)
        }
        // Kinematics/trajectory diagnostics: dry-run a motion through the
        // existing provider and report collisions along the resolved path
        // (add-mcp.md §4 `preview_trajectory`, Req 11.1–11.6). The whole arm is
        // feature-gated on `collision`: the default build carries no collision
        // types and treats `preview_trajectory` as an unknown request type,
        // exactly as before this feature was added.
        #[cfg(feature = "collision")]
        "preview_trajectory" => {
            let mut s = state.lock().unwrap();
            preview_trajectory(req, &mut s)
        }
        // ── trajectory motion source ─────────────────────────────────────
        // An alternative to the KRL interpreter: replay an already-planned
        // trajectory. These verbs are deliberately NOT feature-gated (unlike
        // `preview_trajectory` above) — the `collision` feature is off in
        // default builds, and gating them would omit the whole motion source
        // from exactly the artifact that ships. Only the collision REPORT
        // attached at load time depends on that feature.
        "trajectory_list" => {
            let s = state.lock().unwrap();
            Response::success(req.id, list_trajectory_plans(&s))
        }
        "trajectory_load" => {
            let mut s = state.lock().unwrap();
            load_trajectory(req, &mut s)
        }
        "trajectory_unload" => {
            let mut s = state.lock().unwrap();
            let was = s.trajectory.take().is_some();
            if was {
                eprintln!("[trajectory] unloaded — motion source is the KRL interpreter again");
            }
            Response::success(req.id, serde_json::json!({ "loaded": false, "was_loaded": was }))
        }
        // The mutation is scoped so the `as_mut()` borrow ends before
        // `trajectory_state(&s)` takes a shared borrow of the same state.
        "trajectory_play" | "trajectory_pause" | "trajectory_stop" => {
            let mut s = state.lock().unwrap();
            let verb = req.request_type.clone();
            let loaded = match s.trajectory.as_mut() {
                None => false,
                Some(playback) => {
                    match verb.as_str() {
                        "trajectory_play" => playback.play(),
                        "trajectory_pause" => playback.pause(),
                        _ => playback.stop(),
                    }
                    true
                }
            };
            if !loaded {
                Response::error(req.id, "no trajectory loaded")
            } else {
                let result = trajectory_state(&s);
                Response::success(req.id, result)
            }
        }
        "trajectory_seek" => {
            let mut s = state.lock().unwrap();
            let t = req.params.get("t").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let loaded = match s.trajectory.as_mut() {
                None => false,
                Some(playback) => {
                    playback.seek(t);
                    true
                }
            };
            if !loaded {
                Response::error(req.id, "no trajectory loaded")
            } else {
                let result = trajectory_state(&s);
                Response::success(req.id, result)
            }
        }
        // Breakpoints and stepping by operation: playback pauses where an operation starts.
        "trajectory_set_breakpoints" => {
            let mut s = state.lock().unwrap();
            let ops: Vec<String> = req
                .params
                .get("operations")
                .and_then(|v| v.as_array())
                .map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_owned)).collect())
                .unwrap_or_default();
            let Some(playback) = s.trajectory.as_mut() else {
                return Response::error(req.id, "no trajectory loaded");
            };
            let unknown = playback.set_breakpoints(ops);
            let mut result = trajectory_state(&s);
            if !unknown.is_empty() {
                result["unknown_operations"] = serde_json::json!(unknown);
            }
            Response::success(req.id, result)
        }
        "trajectory_step" => {
            let mut s = state.lock().unwrap();
            let robot = req.params.get("robot_id").and_then(|v| v.as_str()).map(str::to_owned);
            let back = req.params.get("direction").and_then(|v| v.as_str()) == Some("back");
            let Some(playback) = s.trajectory.as_mut() else {
                return Response::error(req.id, "no trajectory loaded");
            };
            let moved = if back { playback.step_back(robot.as_deref()) } else { playback.step_next(robot.as_deref()) };
            let mut result = trajectory_state(&s);
            result["stepped_to"] = match moved {
                Some((op, robot, t)) => serde_json::json!({"op_id": op, "robot_id": robot, "t": t}),
                None => serde_json::json!(if back { "start" } else { "end" }),
            };
            Response::success(req.id, result)
        }
        "trajectory_seek_operation" => {
            let mut s = state.lock().unwrap();
            let op = req.params.get("op_id").and_then(|v| v.as_str()).unwrap_or_default().to_owned();
            let robot = req.params.get("robot_id").and_then(|v| v.as_str()).map(str::to_owned);
            let Some(playback) = s.trajectory.as_mut() else {
                return Response::error(req.id, "no trajectory loaded");
            };
            if playback.seek_operation(&op, robot.as_deref()).is_none() {
                return Response::error(req.id, &format!("the loaded plan has no operation {op}"));
            }
            Response::success(req.id, trajectory_state(&s))
        }
        "trajectory_timeline" => {
            let s = state.lock().unwrap();
            match s.trajectory.as_ref() {
                Some(playback) => Response::success(req.id, playback.summary_json()),
                None => Response::error(req.id, "no trajectory loaded"),
            }
        }
        "trajectory_set_loop" => {
            let mut s = state.lock().unwrap();
            let looping = req.params.get("loop").and_then(|v| v.as_bool()).unwrap_or(false);
            let loaded = match s.trajectory.as_mut() {
                None => false,
                Some(playback) => {
                    playback.set_loop(looping);
                    true
                }
            };
            if !loaded {
                Response::error(req.id, "no trajectory loaded")
            } else {
                let result = trajectory_state(&s);
                Response::success(req.id, result)
            }
        }
        // Playback speed IS the cell speed — one concept, already exposed by
        // `set_speed`; this alias exists so the panel can drive it without
        // knowing that, and returns trajectory state rather than a bare speed.
        "trajectory_set_speed" => {
            let mut s = state.lock().unwrap();
            let speed = req.params.get("speed").and_then(|v| v.as_f64()).unwrap_or(1.0);
            s.speed = speed.clamp(0.1, 100.0);
            let result = trajectory_state(&s);
            Response::success(req.id, result)
        }
        "trajectory_state" => {
            let s = state.lock().unwrap();
            Response::success(req.id, trajectory_state(&s))
        }
        other => Response::error(req.id, format!("unknown request type: {}", other)),
    }
}

/// Handle the `preview_trajectory` MCP tool (Req 11.1–11.6).
///
/// Resolves the requested motion through the existing trajectory provider
/// **unchanged** (Req 11.1) and returns the provider outcome (duration, tier,
/// resolved-path kind). When a scene collision checker is available it then runs
/// a discrete trajectory collision check over the resolved [`JointPath`]
/// (Req 11.2) and augments the response with a top-level `in_collision` boolean
/// and a `collisions` array ordered by ascending progress (Req 11.3–11.5), plus
/// a `model_errors` array naming any per-model FFI errors without failing the
/// preview (Req 11.6).
///
/// ## What is wired vs. what is left to the provider
///
/// * **Target semantics.** The provider's [`crate::traj::MotionRequest`] is
///   expressed entirely in Cartesian poses `[x, y, z, a, b, c]` (mm / deg) for
///   both `PTP` and `LIN`; `motion_type` only selects the [`MotionKind`]. This
///   handler therefore interprets the `target` argument as a 6-value Cartesian
///   pose (consistent with the provider), rather than joint angles. A
///   joint-angle PTP target is out of scope and rejected.
/// * **Start pose / seed.** Taken from the robot's current interpreter state
///   (or home fallback), mirroring how `run_one_cycle` builds a motion request.
/// * **Scene composition.** The preview supplies only the previewed robot's
///   path, so the checker evaluates that robot's self-collision (and any class
///   involving only it); other robots are absent and treated as *unchecked*
///   (Req 6.7), so robot-robot pairs are not reported from a single-robot
///   preview. This is an honest consequence of the single-robot tool contract.
/// * **Model errors.** `check_trajectory` records per-model errors on the
///   reports of *colliding* waypoints; this handler aggregates those. A model
///   that errors while nothing collides yields no retained waypoint and so is
///   not named — matching the "records one or more per-model errors" wording of
///   Req 11.6.
#[cfg(feature = "collision")]
fn preview_trajectory(req: &Request, s: &mut CellState) -> Response {
    use crate::collision::{CollisionKind, ModelStatus, SceneTrajectory};
    use crate::path::JointPath;
    use crate::traj::{MotionKind, MotionRequest};

    // --- Parse arguments -----------------------------------------------------
    let robot_idx = req.params.get("robot").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
    if robot_idx >= s.krls.len() || robot_idx >= s.sim.robots.len() {
        return Response::error(req.id, "invalid robot index");
    }

    let kind = match req
        .params
        .get("motion_type")
        .and_then(|v| v.as_str())
        .unwrap_or("PTP")
        .to_ascii_uppercase()
        .as_str()
    {
        "PTP" => MotionKind::Ptp,
        "LIN" => MotionKind::Lin,
        other => return Response::error(req.id, format!("invalid motion_type: {other}")),
    };

    // `target` is a Cartesian pose [x, y, z, a, b, c] (mm / deg), matching the
    // provider's MotionRequest for both PTP and LIN (see doc comment).
    let target_pose: [f64; 6] = match req.params.get("target").and_then(|v| v.as_array()) {
        Some(arr) if arr.len() == 6 => {
            let mut pose = [0.0f64; 6];
            for (i, v) in arr.iter().enumerate() {
                match v.as_f64() {
                    Some(f) => pose[i] = f,
                    None => return Response::error(req.id, "target must be 6 finite numbers"),
                }
            }
            pose
        }
        _ => {
            return Response::error(
                req.id,
                "target must be a 6-element Cartesian pose [x,y,z,a,b,c]",
            )
        }
    };

    let vel_percent = req
        .params
        .get("vel_percent")
        .and_then(|v| v.as_f64())
        .unwrap_or(100.0);
    let approximate = req
        .params
        .get("approximate")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    // --- Start pose + seed from current robot state --------------------------
    let (start_pose, seed_joints) = {
        let krl = &s.krls[robot_idx];
        if let Some(ref interp) = krl.interpreter {
            let p = &interp.current_position;
            let seed = interp.current_joints.or(krl.home_joints).unwrap_or([0.0; 6]);
            ([p.x, p.y, p.z, p.a, p.b, p.c], seed)
        } else {
            let p = &krl.position;
            let seed = krl.home_joints.unwrap_or([0.0; 6]);
            ([p.x, p.y, p.z, p.a, p.b, p.c], seed)
        }
    };

    // --- Resolve through the existing provider, unchanged (Req 11.1) ---------
    let model_name = s.sim.robots[robot_idx].model.clone();
    let robot_id = s.sim.robots[robot_idx].robot_id.clone();
    let provider = match s.providers.get_mut(&model_name) {
        Some(p) => p,
        None => return Response::error(req.id, format!("no trajectory provider for model {model_name}")),
    };
    let solution = provider.resolve_motion(&MotionRequest {
        kind,
        start_pose,
        target_pose,
        vel_percent,
        approximate,
        seed_joints,
    });

    let (path_kind, knots) = match &solution.path {
        JointPath::Dense { qs, .. } => ("dense", qs.len()),
        JointPath::Endpoints { .. } => ("endpoints", 2),
        JointPath::None => ("none", 0),
    };

    // Provider outcome — identical to what the provider produces without the
    // collision step (Req 11.1).
    let mut result = serde_json::json!({
        "robot": robot_idx,
        "robot_id": robot_id.clone(),
        "motion_type": match kind { MotionKind::Ptp => "PTP", MotionKind::Lin => "LIN" },
        "duration_s": solution.duration_s,
        "tier": solution.tier.to_string(),
        "note": solution.note,
        "reachable": solution.path.is_some(),
        "path_kind": path_kind,
        "knots": knots,
    });

    // --- Discrete trajectory collision check over the resolved path ----------
    // Build a single-robot scene trajectory keyed by this robot's id. Choose a
    // sample count that visits every knot at stride 1:
    //   * Dense     — N = knots-1 so s = k/N lands on each knot (N>=1).
    //   * Endpoints — 2 knots ⇒ N = 2 (samples s = 0, 0.5, 1.0).
    //   * None      — no joint info; the checker reports "no checkable robots".
    let (total_knots, sample_count) = match &solution.path {
        JointPath::Dense { qs, .. } => {
            let k = qs.len().max(1);
            (k, k.saturating_sub(1).max(1))
        }
        JointPath::Endpoints { .. } => (2, 2),
        JointPath::None => (1, 1),
    };

    match s.scene_checker.as_mut() {
        Some(checker) => {
            // The preview tool opts into report-all so every colliding waypoint
            // is surfaced ordered by progress (Req 11.5).
            checker.set_report_all(true);
            let trajectory = SceneTrajectory::new(total_knots, sample_count)
                .with_path(robot_id.clone(), solution.path.clone());
            let traj_report = checker.check_trajectory(&trajectory);

            // Flatten colliding waypoints (already ascending by progress, Req
            // 6.4) into a single ordered `collisions` array (Req 11.3, 11.5).
            // distance_m is clamped to be finite and >= 0: internal penetration
            // distances are negative, and are reported as 0.0 (Req 11.3).
            let mut collisions: Vec<Value> = Vec::new();
            let mut model_errors: std::collections::BTreeMap<String, String> =
                std::collections::BTreeMap::new();

            // Seed model_errors with every environment object that failed to
            // load, independent of whether any waypoint collided (Req 8.9).
            // `traj_report.all` only carries *colliding* waypoints, so an
            // environment load failure on an otherwise clear trajectory would
            // never reach the loop below without this.
            for mc in checker.environment_errors() {
                model_errors
                    .entry(mc.model_id.clone())
                    .or_insert_with(|| mc.error.clone().unwrap_or_else(|| "environment object failed to load".into()));
            }

            for wp in &traj_report.all {
                for pair in &wp.report.pairs {
                    let kind_str = match pair.kind {
                        CollisionKind::SelfCollision => "SelfCollision",
                        CollisionKind::RobotRobot => "RobotRobot",
                        CollisionKind::RobotEnvironment => "RobotEnvironment",
                    };
                    let distance_m = if pair.distance_m.is_finite() {
                        pair.distance_m.max(0.0)
                    } else {
                        0.0
                    };
                    collisions.push(serde_json::json!({
                        "kind": kind_str,
                        "object_a": pair.object_a,
                        "object_b": pair.object_b,
                        "distance_m": distance_m,
                        "progress": wp.progress,
                    }));
                }
                // Aggregate per-model FFI errors recorded on this waypoint
                // (Req 11.6) — named, but never fatal to the preview.
                for mc in &wp.report.model_status {
                    if mc.status == ModelStatus::Errored {
                        model_errors
                            .entry(mc.model_id.clone())
                            .or_insert_with(|| {
                                mc.error.clone().unwrap_or_else(|| "collision query failed".into())
                            });
                    }
                }
            }

            // Req 11.4: in_collision iff the collisions array is non-empty.
            result["in_collision"] = Value::Bool(!collisions.is_empty());
            result["collisions"] = Value::Array(collisions);
            if !model_errors.is_empty() {
                let errors: Vec<Value> = model_errors
                    .into_iter()
                    .map(|(model, error)| serde_json::json!({"model": model, "error": error}))
                    .collect();
                result["model_errors"] = Value::Array(errors);
            }
            if let Some(reason) = traj_report.rejected {
                result["collision_check_rejected"] = Value::String(reason);
            }
            if !traj_report.unchecked_robots.is_empty() {
                result["unchecked_robots"] = serde_json::json!(traj_report.unchecked_robots);
            }
        }
        None => {
            // No scene checker (never-fail-hard): report an empty, non-colliding
            // collision result so the response shape is stable.
            result["in_collision"] = Value::Bool(false);
            result["collisions"] = Value::Array(Vec::new());
            result["collision_check_unavailable"] = Value::Bool(true);
        }
    }

    Response::success(req.id, result)
}

fn build_full_state(sim: &CellSimulator, krls: &[RobotKrl], names: &IoNames) -> Value {
    let robots: Vec<Value> = sim.robots.iter().enumerate().map(|(idx, r)| {
        let mut inputs: Vec<Value> = r.inputs.iter()
            .map(|(k, v)| serde_json::json!({"signal": k, "value": v}))
            .collect();
        inputs.sort_by(|a, b| a["signal"].as_str().unwrap_or("").cmp(b["signal"].as_str().unwrap_or("")));

        let mut outputs: Vec<Value> = r.outputs.iter()
            .map(|(k, v)| serde_json::json!({"signal": k, "value": v}))
            .collect();
        outputs.sort_by(|a, b| a["signal"].as_str().unwrap_or("").cmp(b["signal"].as_str().unwrap_or("")));

        let waiting_for: Option<String> = match r.phase {
            Phase::WaitReady => Some("RoboterBereit".into()),
            Phase::WaitEcho => Some(format!("SpiegelungFolge={}", r.sequence)),
            Phase::WaitFM => Some(format!("FM (Folge {} running)", r.sequence)),
            Phase::EnableDrives => Some("Enabling drives".into()),
            Phase::SelectSequence if !sim.check_dependencies(r) => {
                Some(format!("Dependencies: {:?}", r.depends_on))
            }
            _ => None,
        };

        let folgen_str: Vec<String> = r.production_folgen.iter().map(|f| f.to_string()).collect();

        serde_json::json!({
            "index": idx,
            "name": r.name,
            "robot_id": r.robot_id,
            "model": r.model,
            "phase": format!("{}", r.phase),
            "sequence": r.sequence,
            "is_home": r.is_home_cycle,
            "cycles_completed": r.cycles_completed,
            "production_folgen": folgen_str.join(","),
            "role": r.role,
            "depends_on": r.depends_on,
            "inputs": inputs,
            "outputs": outputs,
            "waiting_for": waiting_for,
        })
    }).collect();

    let krl_states: Vec<Value> = krls.iter().map(|k| k.full_state(&names.inputs, &names.outputs)).collect();

    serde_json::json!({
        "cell_name": sim.cell_name,
        "cycle": sim.cycle_count,
        "complete": false,
        "robots": robots,
        "krl": krl_states,
    })
}

#[cfg(test)]
mod listen_tests {
    use super::local_address;
    use std::net::SocketAddr;

    /// The URLs the server prints: loopback for a wildcard listener, else its own address.
    #[test]
    fn a_wildcard_listener_is_reached_on_loopback() {
        let at = |s: &str| s.parse::<SocketAddr>().unwrap();
        assert_eq!(local_address(at("0.0.0.0:9902")), at("127.0.0.1:9902"));
        assert_eq!(local_address(at("[::]:9902")), at("[::1]:9902"));
        assert_eq!(local_address(at("127.0.0.1:9902")), at("127.0.0.1:9902"));
        assert_eq!(local_address(at("192.0.2.10:9902")), at("192.0.2.10:9902"));
    }
}

#[cfg(all(test, unix))]
mod plan_path_tests {
    use super::resolve_plan_path;

    #[test]
    fn a_plan_in_a_linked_plan_folder_resolves_others_do_not() {
        let root = std::env::temp_dir().join(format!("cell-runner-plan-path-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cell");
        let view = cell.join(".view");
        std::fs::create_dir_all(cell.join("agentic_planner/plans")).unwrap();
        std::fs::create_dir_all(&view).unwrap();
        std::fs::write(cell.join("agentic_planner/plans/bt_1.json"), "{}").unwrap();
        std::fs::write(root.join("secret.json"), "{}").unwrap();
        std::os::unix::fs::symlink("../agentic_planner", view.join("agentic_planner")).unwrap();
        std::os::unix::fs::symlink(root.join("secret.json"), view.join("secret.json")).unwrap();
        let plan = resolve_plan_path(&view, "agentic_planner/plans/bt_1.json").unwrap();
        assert_eq!(plan, std::fs::canonicalize(cell.join("agentic_planner/plans/bt_1.json")).unwrap());
        let err = resolve_plan_path(&view, "secret.json").unwrap_err();
        assert!(err.contains("escapes"), "{err}");
        assert!(resolve_plan_path(&view, "agentic_planner/../../secret.json").unwrap_err().contains("escapes"));
        assert!(resolve_plan_path(&cell, "agentic_planner/plans/bt_1.json").is_ok(), "a legacy cell as before");
        std::fs::remove_dir_all(&root).ok();
    }
}

#[cfg(test)]
mod loopback_cors_tests {
    #[test]
    fn only_loopback_origins_get_cors() {
        for ok in ["http://localhost:8080", "http://127.0.0.1:3030", "http://127.0.0.2", "https://localhost", "http://[::1]:9902"] {
            assert!(super::loopback_origin(ok.as_bytes()), "{ok}");
        }
        for bad in ["http://localhost.example.com", "http://127.0.0.1.example.com", "https://example.com", "null",
                    "http://example.com@127.0.0.1", "http://[::1", "http://localhost:", "http://10.0.0.1:8080", "http://0.0.0.0:8080"] {
            assert!(!super::loopback_origin(bad.as_bytes()), "{bad}");
        }
    }
}
