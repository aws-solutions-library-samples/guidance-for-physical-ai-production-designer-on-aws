//! The simulated cell: the cell-runner of this repo's `cell/` workspace, embedded in
//! `cell-rerun-server` and run by factoryos-backend for the behaviour-tree executor (step 1 Load &
//! Play).
//!
//! Ported from the retired Kiro extension's `cell-rerun-server-manager.ts`:
//! * `cell-rerun-server stdio-serve`: NDJSON control channel; `start` launches the cell-runner, on
//!   `cellRunnerWsPort` when that is free, else on a free port ([`sim_port`]), so a port already
//!   in use (another backend's simulation, a cell-runner left running) doesn't fail Load & Play.
//! * The cell-runner's WebSocket (`/ws`): polled for `state` (→ `factoryos/sim/state`), pushed
//!   `positions` frames (→ `factoryos/sim/positions`: a stream of their own at `positions_fps`, stamped `t`), and commands from the viewer
//!   (`factoryos/sim/cmd`).
//!
//! The simulation is shown in the Robo Fitter's scene (its robots follow `factoryos/sim/state`, see
//! `FitterService::follow_cell`). `factoryos/cell/*` and `factoryos/scene/cell` are the LIVE
//! cell's (fos-live-bridge, fos-live-scene); this service neither writes nor answers them.

use std::collections::HashMap;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context as _;
use axum::extract::State;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::{
    BackendState, BackendStatus, CellCommand, CellResponse, CellsInfo, CreateCell, LoadPlan, PlansInfo, SampleCell,
    SetActiveCell, TaskProgram, api, keys,
};
use futures_util::{SinkExt as _, StreamExt as _};
use serde_json::{Value, json};
use tokio::io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader};
use tokio::process::{Child, ChildStdin, Command};
use tokio::sync::{mpsc, oneshot};
use tokio_tungstenite::tungstenite::Message;
use tokio_util::sync::CancellationToken;

use crate::bus::{Bus, reply_json};
use crate::{AppState, paths};

const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(10);
/// How long a simulation of another cell may take to stop before Load & Play gives up.
const RESTART_TIMEOUT: Duration = Duration::from_secs(8);
const START_TIMEOUT: Duration = Duration::from_secs(60);
const COMMAND_TIMEOUT: Duration = Duration::from_secs(10);
/// Fast enough for the Robo Fitter's 3D view to follow the robots smoothly (see `FitterService::follow_cell`).
const STATE_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// What to run: resolved from the config when the cell is started.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellPaths {
    /// The cell's folder name (the active cell).
    pub cell: String,
    pub cell_dir: String,
    pub env_dir: String,
    pub scene: String,
    /// `low`, `visual` or `collision`.
    pub mesh_lod: String,
    /// The cell-runner's HTTP/WebSocket port: the preferred one (`cellRunnerWsPort`) until
    /// [`CellService::start`] resolves it with [`sim_port`].
    pub cell_port: u16,
    /// A reference cell's inputs (`InputHashes::combined`: its scene, URDFs, products) its view was
    /// built for; empty for a legacy cell. Saving its scene changes it, so Load & Play restarts the
    /// simulation on the scene as saved.
    pub inputs: String,
}

impl CellPaths {
    /// The same cell, scene and paths (the port is not compared: it is picked at each start).
    fn same_target(&self, other: &Self) -> bool {
        (&self.cell, &self.cell_dir, &self.env_dir, &self.scene, &self.mesh_lod, &self.inputs)
            == (&other.cell, &other.cell_dir, &other.env_dir, &other.scene, &other.mesh_lod, &other.inputs)
    }
}

/// Whether a simulation running `running` must be restarted to run `wanted`.
fn needs_restart(running: Option<&CellPaths>, wanted: &CellPaths) -> bool {
    running.is_some_and(|running| !running.same_target(wanted))
}

/// Poses per second on `factoryos/sim/positions` (`FACTORY_OS_SIM_POSITIONS_FPS`, default 60, at most
/// the cell-runner's 120; 0: no stream).
pub fn positions_fps() -> f64 {
    std::env::var("FACTORY_OS_SIM_POSITIONS_FPS").ok().and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(60.0).clamp(0.0, 120.0)
}

/// The port for the simulated cell-runner: `preferred` when nothing holds it, else a free one.
pub fn sim_port(preferred: u16) -> u16 {
    let addr = std::net::SocketAddr::from(([127, 0, 0, 1], preferred));
    // Someone listening (on any address) accepts the connection; binding alone can succeed next
    // to a wildcard listener on BSD/macOS.
    let listening = std::net::TcpStream::connect_timeout(&addr, Duration::from_millis(200)).is_ok();
    if !listening && std::net::TcpListener::bind(addr).is_ok() {
        return preferred;
    }
    std::net::TcpListener::bind(("127.0.0.1", 0))
        .and_then(|listener| listener.local_addr())
        .map(|addr| addr.port())
        .unwrap_or(preferred)
}

type CommandRequest = (CellCommand, oneshot::Sender<CellResponse>);

pub struct CellService {
    bus: Bus,
    shared: Mutex<Shared>,
    /// Serialises what (re)starts or stops the simulation for the guided flow: Load & Play (its
    /// scene opened in the Fitter, the simulation started) and the Fitter's Stop simulation and edit.
    flow: tokio::sync::Mutex<()>,
    /// Bumped by Stop simulation and edit: a Load & Play still waiting for its simulation then
    /// gives up at once (and says why) instead of waiting for a start that was cancelled.
    edits: std::sync::atomic::AtomicU64,
}

/// Held while Load & Play opens its scene and starts the simulation, or while Stop simulation and
/// edit stops it: the two never interleave.
pub type FlowGuard<'a> = tokio::sync::MutexGuard<'a, ()>;

#[derive(Default)]
struct Shared {
    status: BackendStatus,
    /// Latest cell-runner `state` result, verbatim.
    state: Option<Value>,
    running: Option<Running>,
    /// What the running cell-runner runs (its port resolved), while it runs.
    running_paths: Option<CellPaths>,
}

struct Running {
    cancel: CancellationToken,
    commands: mpsc::Sender<CommandRequest>,
}

impl CellService {
    pub fn new(bus: Bus) -> Arc<Self> {
        Arc::new(Self {
            bus,
            shared: Mutex::default(),
            flow: tokio::sync::Mutex::new(()),
            edits: std::sync::atomic::AtomicU64::new(0),
        })
    }

    /// Wait for the guided flow's turn (see [`CellService::flow`]).
    pub async fn flow_lock(&self) -> FlowGuard<'_> {
        self.flow.lock().await
    }

    /// How many times Stop simulation and edit ran.
    pub fn edits(&self) -> u64 {
        self.edits.load(std::sync::atomic::Ordering::SeqCst)
    }

    /// Stop simulation and edit: cancel a Load & Play waiting for its simulation, then stop it.
    /// The caller holds the [`FlowGuard`].
    pub async fn stop_for_edit(&self, _flow: &FlowGuard<'_>) {
        self.edits.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        if self.running_paths().is_some() {
            self.bus.log("Stopping the simulation to edit the scene in the Robo Fitter");
            self.stop_and_wait(RESTART_TIMEOUT).await;
        }
    }

    /// Saving a Robo Fitter scene rebuilds the cells made from it: stop the simulation if it runs
    /// one of `cells` (and cancel a Load & Play waiting for it, as [`Self::stop_for_edit`] does), so
    /// the cell-runner does not run a cell whose files are replaced. The caller holds the
    /// [`FlowGuard`]. Returns whether it stopped.
    pub async fn stop_for_rebuild(&self, _flow: &FlowGuard<'_>, cells: &[String]) -> bool {
        let Some(paths) = self.running_paths().filter(|paths| cells.contains(&paths.cell)) else {
            return false;
        };
        self.edits.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        self.bus.log(format!("Stopping the simulation of {}: saving its Robo Fitter scene rebuilds the cell", paths.cell));
        self.stop_and_wait(RESTART_TIMEOUT).await;
        true
    }

    pub fn status(&self) -> BackendStatus {
        self.shared.lock().unwrap().status.clone()
    }

    /// What the running cell-runner runs, if it runs.
    pub fn running_paths(&self) -> Option<CellPaths> {
        self.shared.lock().unwrap().running_paths.clone()
    }

    /// Start the cell-runner for `paths`, on `paths.cell_port` if it is free, else on a free port.
    pub fn start(self: &Arc<Self>, mut paths: CellPaths) -> Result<(), String> {
        let (commands_tx, commands_rx) = mpsc::channel(64);
        let cancel = CancellationToken::new();
        {
            let mut shared = self.shared.lock().unwrap();
            if shared.running.is_some() {
                return Err("the simulation is already running".to_owned());
            }
            let port = sim_port(paths.cell_port);
            if port != paths.cell_port {
                self.bus.log(format!(
                    "Port {} is taken; the simulation uses 127.0.0.1:{port}",
                    paths.cell_port
                ));
                paths.cell_port = port;
            }
            shared.running = Some(Running {
                cancel: cancel.clone(),
                commands: commands_tx,
            });
            shared.running_paths = Some(paths.clone());
            shared.state = None;
        }
        self.set_status(|status| *status = BackendStatus {
            state: BackendState::Starting,
            cell: Some(paths.cell.clone()),
            scene: Some(paths.scene.clone()),
            port: Some(paths.cell_port),
            ..Default::default()
        });

        let this = self.clone();
        tokio::spawn(async move {
            let result = this.run(&paths, &cancel, commands_rx).await;
            {
                let mut shared = this.shared.lock().unwrap();
                shared.running = None;
                shared.running_paths = None;
            }
            match result {
                Ok(()) => {
                    this.set_status(|status| {
                        status.state = BackendState::Stopped;
                        status.connected = false;
                    });
                    this.bus.log("Backend stopped.");
                }
                Err(err) => {
                    let message = format!("{err:#}");
                    this.set_status(|status| {
                        status.state = BackendState::Failed;
                        status.connected = false;
                        status.error = Some(message.clone());
                    });
                    this.bus.log(format!("Backend failed: {message}"));
                }
            }
        });
        Ok(())
    }

    pub fn stop(&self) {
        if let Some(running) = &self.shared.lock().unwrap().running {
            running.cancel.cancel();
        }
    }

    /// Stop the cell and wait (up to `timeout`) until its processes are gone.
    pub async fn stop_and_wait(&self, timeout: Duration) {
        self.stop();
        let deadline = tokio::time::Instant::now() + timeout;
        while self.shared.lock().unwrap().running.is_some() && tokio::time::Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    /// Send a verb to the cell-runner and wait for its reply.
    pub async fn command(&self, command: CellCommand) -> CellResponse {
        let sender = self.shared.lock().unwrap().running.as_ref().map(|r| r.commands.clone());
        let Some(sender) = sender else {
            return CellResponse::error("the simulation is not running (Load & Play in step 1 starts it)");
        };
        let (reply_tx, reply_rx) = oneshot::channel();
        if sender.send((command, reply_tx)).await.is_err() {
            return CellResponse::error("the simulation is shutting down");
        }
        match tokio::time::timeout(COMMAND_TIMEOUT, reply_rx).await {
            Ok(Ok(response)) => response,
            Ok(Err(_)) => CellResponse::error("the cell runner connection was lost"),
            Err(_) => CellResponse::error("the cell runner did not reply in time"),
        }
    }

    fn set_status(&self, update: impl FnOnce(&mut BackendStatus)) {
        let status = {
            let mut shared = self.shared.lock().unwrap();
            let before = shared.status.clone();
            update(&mut shared.status);
            if shared.status == before {
                return;
            }
            shared.status.clone()
        };
        self.bus.put_json(keys::SIM_STATUS, &status);
    }

    async fn run(
        self: &Arc<Self>,
        paths: &CellPaths,
        cancel: &CancellationToken,
        mut commands: mpsc::Receiver<CommandRequest>,
    ) -> anyhow::Result<()> {
        let mut supervisor = Supervisor::spawn(&self.bus).await?;
        let health = supervisor.request("health.check", json!({}), HANDSHAKE_TIMEOUT).await?;
        anyhow::ensure!(
            health["backendsEmbedded"] != false,
            "cell-rerun-server was built without embedded backends"
        );

        self.bus.log(format!(
            "Starting the simulation of {} (cell {}, env {}, scene {}, port {})",
            paths.cell, paths.cell_dir, paths.env_dir, paths.scene, paths.cell_port
        ));
        let start = supervisor.request(
            "start",
            json!({
                "cellDir": paths.cell_dir,
                "envDir": paths.env_dir,
                "scene": paths.scene,
                "cellPort": paths.cell_port,
            }),
            START_TIMEOUT,
        );
        let info = tokio::select! {
            info = start => info?,
            () = cancel.cancelled() => {
                supervisor.shutdown().await;
                return Ok(());
            }
        };
        let cell_port = info["cellPort"].as_u64().unwrap_or(u64::from(paths.cell_port));
        self.bus.log(format!(
            "Simulation of {} ready — controller {}",
            paths.cell,
            info["cellWsUrl"].as_str().unwrap_or("?")
        ));
        self.set_status(|status| {
            status.state = BackendState::Ready;
            status.cell = Some(paths.cell.clone());
            status.scene = Some(paths.scene.clone());
            status.port = u16::try_from(cell_port).ok();
        });

        let client = {
            let this = self.clone();
            let cancel = cancel.clone();
            tokio::spawn(async move { this.cell_runner_client(cell_port, &cancel, &mut commands).await })
        };
        let positions = {
            let this = self.clone();
            let cancel = cancel.clone();
            tokio::spawn(async move { this.positions_stream(cell_port, &cancel).await })
        };

        cancel.cancelled().await;
        client.await.ok();
        positions.await.ok();
        supervisor.shutdown().await;
        Ok(())
    }

    /// Talk to the cell-runner's `/ws` until cancelled, reconnecting as needed.
    async fn cell_runner_client(
        &self,
        cell_port: u64,
        cancel: &CancellationToken,
        commands: &mut mpsc::Receiver<CommandRequest>,
    ) {
        let url = format!("ws://127.0.0.1:{cell_port}/ws");
        while !cancel.is_cancelled() {
            match tokio_tungstenite::connect_async(&url).await {
                Ok((socket, _)) => {
                    self.set_status(|status| status.connected = true);
                    self.bus.log(format!("Connected to {url}"));
                    if let Err(err) = self.drive_cell_runner(socket, cancel, commands).await {
                        self.bus.log(format!("Cell runner connection lost: {err:#}"));
                    }
                    self.set_status(|status| status.connected = false);
                }
                Err(_) => {
                    // Not up yet: answer queued commands instead of letting them time out.
                    let wait = tokio::time::sleep(Duration::from_secs(1));
                    tokio::pin!(wait);
                    loop {
                        tokio::select! {
                            () = &mut wait => break,
                            () = cancel.cancelled() => return,
                            Some((_, reply)) = commands.recv() => {
                                reply.send(CellResponse::error("not connected to the cell runner")).ok();
                            }
                        }
                    }
                }
            }
        }
    }

    /// The cell-runner's pose stream on `factoryos/sim/positions`: its `subscribe_positions` at
    /// [`positions_fps`] on a connection of its own (the subscription takes the connection over),
    /// each frame stamped `t` (this machine's clock, seconds since the epoch, at its arrival: the
    /// cell-runner is local) for players that buffer to a fixed lag (Isaac Sim, fos-isaac-bridge).
    /// Reconnects as needed until cancelled; `FACTORY_OS_SIM_POSITIONS_FPS=0`: off.
    async fn positions_stream(&self, cell_port: u64, cancel: &CancellationToken) {
        let fps = positions_fps();
        if fps <= 0.0 {
            return;
        }
        let url = format!("ws://127.0.0.1:{cell_port}/ws");
        while !cancel.is_cancelled() {
            if let Ok((socket, _)) = tokio_tungstenite::connect_async(&url).await {
                let (mut tx, mut rx) = socket.split();
                let subscribe = json!({ "id": 1, "type": "subscribe_positions", "params": { "fps": fps } });
                if tx.send(Message::text(subscribe.to_string())).await.is_ok() {
                    loop {
                        tokio::select! {
                            () = cancel.cancelled() => {
                                tx.close().await.ok();
                                return;
                            }
                            msg = rx.next() => {
                                let Some(Ok(Message::Text(text))) = msg else { break };
                                let Ok(mut value) = serde_json::from_str::<Value>(&text) else { continue };
                                if value["type"] == "positions" {
                                    let t = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
                                    value["t"] = json!(t);
                                    self.bus.put_json(keys::SIM_POSITIONS, &value);
                                }
                            }
                        }
                    }
                }
            }
            tokio::select! {
                () = tokio::time::sleep(Duration::from_secs(1)) => {}
                () = cancel.cancelled() => return,
            }
        }
    }

    async fn drive_cell_runner(
        &self,
        socket: tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>,
        cancel: &CancellationToken,
        commands: &mut mpsc::Receiver<CommandRequest>,
    ) -> anyhow::Result<()> {
        enum Pending {
            State,
            Command(oneshot::Sender<CellResponse>),
        }

        let (mut tx, mut rx) = socket.split();
        let mut pending = HashMap::<u64, Pending>::new();
        let mut next_id = 1_u64;
        let mut poll = tokio::time::interval(STATE_POLL_INTERVAL);
        poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        loop {
            tokio::select! {
                () = cancel.cancelled() => {
                    tx.close().await.ok();
                    return Ok(());
                }
                _ = poll.tick() => {
                    // One state request in flight at a time.
                    if !pending.values().any(|p| matches!(p, Pending::State)) {
                        let id = next_id;
                        next_id += 1;
                        tx.send(Message::text(json!({ "id": id, "type": "state" }).to_string())).await?;
                        pending.insert(id, Pending::State);
                    }
                }
                Some((command, reply)) = commands.recv() => {
                    let id = next_id;
                    next_id += 1;
                    let mut request = serde_json::to_value(&command)?;
                    request["id"] = id.into();
                    tx.send(Message::text(request.to_string())).await?;
                    pending.insert(id, Pending::Command(reply));
                }
                msg = rx.next() => {
                    let Some(msg) = msg else { anyhow::bail!("closed by the cell runner") };
                    let Message::Text(text) = msg? else { continue };
                    let value: Value = match serde_json::from_str(&text) {
                        Ok(value) => value,
                        Err(err) => {
                            tracing::warn!("cell runner sent invalid JSON: {err}");
                            continue;
                        }
                    };
                    if value["type"] == "positions" {
                        self.bus.put_json(keys::SIM_POSITIONS, &value);
                        continue;
                    }
                    let Some(entry) = value["id"].as_u64().and_then(|id| pending.remove(&id)) else {
                        continue;
                    };
                    let response: CellResponse = serde_json::from_value(value).unwrap_or_default();
                    match entry {
                        Pending::State => {
                            if response.ok && let Some(state) = response.result {
                                self.shared.lock().unwrap().state = Some(state.clone());
                                self.bus.put_json(keys::SIM_STATE, &state);
                            }
                        }
                        Pending::Command(reply) => {
                            reply.send(response).ok();
                        }
                    }
                }
            }
        }
    }

    /// Serve the simulation's queryables: commands, and the latest state/status for late joiners.
    /// Not `factoryos/cell/*` (the live cell's: on a server connected to it, fos-live-scene
    /// answers those).
    pub async fn serve_queries(self: Arc<Self>) -> anyhow::Result<()> {
        let session = self.bus.session();
        let err = |e: zenoh::Error| anyhow::anyhow!("{e}");
        let commands = session.declare_queryable(keys::SIM_COMMAND).await.map_err(err)?;
        let state = session.declare_queryable(keys::SIM_STATE).await.map_err(err)?;
        let status = session.declare_queryable(keys::SIM_STATUS).await.map_err(err)?;

        loop {
            tokio::select! {
                Ok(query) = commands.recv_async() => {
                    let this = self.clone();
                    tokio::spawn(async move {
                        let command = query
                            .payload()
                            .map(|payload| serde_json::from_slice::<CellCommand>(&payload.to_bytes()));
                        let response = match command {
                            Some(Ok(command)) => this.command(command).await,
                            Some(Err(err)) => CellResponse::error(format!("invalid command: {err}")),
                            None => CellResponse::error("missing command payload"),
                        };
                        reply_json(&query, &response).await;
                    });
                }
                Ok(query) = state.recv_async() => {
                    let latest = self.shared.lock().unwrap().state.clone();
                    if let Some(latest) = latest {
                        reply_json(&query, &latest).await;
                    }
                }
                Ok(query) = status.recv_async() => {
                    reply_json(&query, &self.status()).await;
                }
                else => return Ok(()),
            }
        }
    }
}

/// Forward a child's output lines to the backend log.
fn pipe_to_log(bus: &Bus, prefix: &'static str, reader: impl tokio::io::AsyncRead + Send + Unpin + 'static) {
    let bus = bus.clone();
    tokio::spawn(async move {
        let mut lines = BufReader::new(reader).lines();
        while let Ok(Some(line)) = lines.next_line().await {
            bus.log(format!("[{prefix}] {line}"));
        }
    });
}

/// NDJSON request/response client for `cell-rerun-server stdio-serve`.
struct Supervisor {
    child: Child,
    stdin: Option<ChildStdin>,
    responses: mpsc::UnboundedReceiver<Value>,
    next_id: u64,
}

impl Supervisor {
    async fn spawn(bus: &Bus) -> anyhow::Result<Self> {
        let binary = paths::cell_server_binary();
        anyhow::ensure!(binary.exists(), "cell-rerun-server not found: {}", binary.display());
        bus.log(format!("Spawning {} stdio-serve", binary.display()));

        let mut command = Command::new(&binary);
        command.arg("stdio-serve").stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped()).kill_on_drop(true);
        // Retried while the binary is Text file busy (a freshly written binary; Linux).
        let mut child = super::codegen::spawn_when_not_busy(&mut command).await?;
        let stdin = child.stdin.take();
        let stdout = child.stdout.take().expect("piped");
        pipe_to_log(bus, "backend", child.stderr.take().expect("piped"));

        let (tx, responses) = mpsc::unbounded_channel();
        let bus = bus.clone();
        tokio::spawn(async move {
            let mut lines = BufReader::new(stdout).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                match serde_json::from_str::<Value>(&line) {
                    Ok(value) => {
                        if tx.send(value).is_err() {
                            break;
                        }
                    }
                    Err(_) => bus.log(format!("[backend] {line}")),
                }
            }
        });

        Ok(Self {
            child,
            stdin,
            responses,
            next_id: 1,
        })
    }

    async fn request(&mut self, method: &str, params: Value, timeout: Duration) -> anyhow::Result<Value> {
        let id = self.next_id;
        self.next_id += 1;
        let stdin = self.stdin.as_mut().context("backend stdin closed")?;
        let line = json!({ "id": id, "method": method, "params": params }).to_string() + "\n";
        stdin.write_all(line.as_bytes()).await?;
        stdin.flush().await?;

        let wait = async {
            loop {
                let response = self.responses.recv().await.context("backend exited")?;
                if response["id"] != id {
                    continue;
                }
                if let Some(error) = response.get("error") {
                    anyhow::bail!(
                        "backend '{method}' failed: {}",
                        error["message"].as_str().unwrap_or("unknown error")
                    );
                }
                return Ok(response["result"].clone());
            }
        };
        tokio::time::timeout(timeout, wait)
            .await
            .map_err(|_| anyhow::anyhow!("backend request '{method}' timed out"))?
    }

    async fn shutdown(mut self) {
        self.request("stop", json!({}), Duration::from_secs(5)).await.ok();
        drop(self.stdin.take()); // EOF stops the backends
        if tokio::time::timeout(Duration::from_secs(5), self.child.wait()).await.is_err() {
            self.child.kill().await.ok();
        }
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::CELL_STATUS, get(status))
        .route(api::CELL_START, post(start))
        .route(api::CELL_STOP, post(stop))
        .route(api::CELL_COMMAND, post(command))
        .route(api::CELL_SAMPLES, get(samples))
        .route(api::CELLS, get(cells))
        .route(api::CELL_ACTIVE, post(set_active_cell))
        .route(api::CELL_CREATE, post(create_cell))
        .route(api::CELL_PLANS, get(plans))
        // Plans with their blackboards run to megabytes of JSON: gzipped when the client asks.
        .route(api::CELL_PLAN, get(get_plan).layer(tower_http::compression::CompressionLayer::new().gzip(true)))
        .route(api::CELL_PLAN_LOAD, post(load_plan))
        .route(api::CELL_TASK, get(get_task).put(put_task))
        .route(api::CELL_TASK_SAMPLE, get(sample_task))
}

#[derive(serde::Deserialize)]
struct TaskQuery {
    name: String,
}

async fn get_task(
    State(state): State<AppState>,
    axum::extract::Query(query): axum::extract::Query<TaskQuery>,
) -> Result<Json<TaskProgram>, HttpError> {
    super::cells::read_task(&state.config, &query.name)
        .map(Json)
        .map_err(|err| (axum::http::StatusCode::NOT_FOUND, format!("{err:#}")))
}

/// Save a task program; returns the refreshed plans/tasks listing.
async fn put_task(State(state): State<AppState>, Json(task): Json<TaskProgram>) -> Result<Json<PlansInfo>, HttpError> {
    super::cells::write_task(&state.config, &task).map_err(|err| (axum::http::StatusCode::BAD_REQUEST, format!("{err:#}")))?;
    state.bus.log(format!("Saved task program {}", task.name));
    Ok(Json(super::cells::plans(&state.config)))
}

async fn sample_task(State(state): State<AppState>) -> Json<TaskProgram> {
    Json(super::cells::sample_task(&state.config))
}

async fn status(State(state): State<AppState>) -> Json<BackendStatus> {
    Json(state.cell.status())
}

/// The active cell's paths; `cell_port` is the preferred port (see [`sim_port`]). A reference cell
/// runs in its view (`<cell>/.view/`: its scene and the catalogue's robots and products laid out
/// under one folder, see `cell_resolve::materialise_view`), brought up to date with its inputs.
async fn cell_paths(state: &AppState) -> CellPaths {
    let store = state.config.clone();
    let view = tokio::task::spawn_blocking(move || reference_view(&store)).await.ok().flatten();
    let mut paths = configured_cell_paths(state);
    if let Some((view, inputs)) = view {
        paths.cell_dir = view.display().to_string();
        paths.env_dir.clone_from(&paths.cell_dir);
        paths.scene = super::cells::SCENE_MARKER.to_owned();
        paths.inputs = inputs;
    }
    paths
}

/// The active cell's view and the inputs it was built for, if it is a reference cell.
fn reference_view(store: &crate::config_store::ConfigStore) -> Option<(std::path::PathBuf, String)> {
    let resolved = super::cells::active_resolved(store).filter(|r| r.kind == super::cell_resolve::CellKind::Reference)?;
    match super::cell_resolve::materialise_view(&resolved) {
        Ok(view) => Some((view, resolved.inputs.combined)),
        Err(err) => {
            tracing::warn!("Cell {}: {err:#}", resolved.name);
            None
        }
    }
}

/// The active cell's paths as the config has them.
fn configured_cell_paths(state: &AppState) -> CellPaths {
    let config = state.config.get();
    CellPaths {
        cell: super::cells::active_cell(&state.config).map(|(name, _)| name).unwrap_or_default(),
        cell_dir: state.config.resolve(&config.cell_dir),
        env_dir: state.config.resolve(&config.env_dir),
        scene: config.scene.trim().to_owned(),
        mesh_lod: if config.mesh_lod.trim().is_empty() { "low".to_owned() } else { config.mesh_lod.trim().to_owned() },
        // Was never sent before, so the setting had no effect and the cell always took 9902.
        cell_port: u16::try_from(config.cell_runner_ws_port).ok().filter(|port| *port != 0).unwrap_or(9902),
        inputs: String::new(),
    }
}

async fn start(State(state): State<AppState>) -> Result<Json<BackendStatus>, (axum::http::StatusCode, String)> {
    state
        .cell
        .start(cell_paths(&state).await)
        .map_err(|err| (axum::http::StatusCode::CONFLICT, err))?;
    Ok(Json(state.cell.status()))
}

/// Run the simulation of `wanted` (restarting one of another cell or scene). The caller holds the
/// [`FlowGuard`]; [`wait_cell_ready`] then waits for it without.
async fn ensure_cell_started(state: &AppState, wanted: CellPaths) -> Result<(), String> {
    let running = state.cell.running_paths();
    if needs_restart(running.as_ref(), &wanted) {
        let was = running.map(|r| format!("{} ({})", r.cell, r.scene)).unwrap_or_default();
        state.bus.log(format!("Restarting the simulation for {} ({}); it ran {was}", wanted.cell, wanted.scene));
        state.cell.stop_and_wait(RESTART_TIMEOUT).await;
        if state.cell.running_paths().is_some() {
            return Err(format!("the simulation of {was} did not stop within {} s", RESTART_TIMEOUT.as_secs()));
        }
    }
    if state.cell.running_paths().is_none() {
        let cell = wanted.cell.clone();
        state
            .cell
            .start(wanted)
            .map_err(|err| format!("could not start the simulation of {cell}: {err}"))?;
        state.bus.log(format!("Started the simulated cell-runner of {cell} for the behaviour-tree executor"));
    }
    Ok(())
}

/// Why waiting for a started simulation ends: `None` while it is still coming up.
#[derive(Debug, PartialEq, Eq)]
enum StartWait {
    Ready,
    /// Stop simulation and edit ran meanwhile.
    Edited,
    Stopped,
    Failed(String),
}

fn start_wait(status: &BackendStatus, running: bool, edited: bool) -> Option<StartWait> {
    if edited {
        return Some(StartWait::Edited);
    }
    match status.state {
        BackendState::Ready if status.connected => Some(StartWait::Ready),
        BackendState::Failed => Some(StartWait::Failed(
            status.error.clone().unwrap_or_else(|| "unknown error (see the backend logs)".to_owned()),
        )),
        BackendState::Stopped if !running => Some(StartWait::Stopped),
        _ => None,
    }
}

/// Wait until the simulation's WebSocket is connected; give up when it fails, is stopped, or Stop
/// simulation and edit ran since `edits` (see [`CellService::edits`]).
async fn wait_cell_ready(state: &AppState, edits: u64) -> Result<(), HttpError> {
    use axum::http::StatusCode;
    let deadline = tokio::time::Instant::now() + START_TIMEOUT + Duration::from_secs(10);
    loop {
        let edited = state.cell.edits() != edits;
        match start_wait(&state.cell.status(), state.cell.running_paths().is_some(), edited) {
            Some(StartWait::Ready) => return Ok(()),
            Some(StartWait::Edited) => {
                return Err((
                    StatusCode::CONFLICT,
                    "Load & Play cancelled: the Robo Fitter's Stop simulation and edit stopped the simulation".to_owned(),
                ));
            }
            Some(StartWait::Stopped) => {
                return Err((
                    StatusCode::CONFLICT,
                    "Load & Play cancelled: the simulation was stopped before it came up".to_owned(),
                ));
            }
            Some(StartWait::Failed(error)) => {
                return Err((StatusCode::SERVICE_UNAVAILABLE, format!("the simulated cell-runner failed to start: {error}")));
            }
            None => {}
        }
        if tokio::time::Instant::now() > deadline {
            return Err((
                StatusCode::SERVICE_UNAVAILABLE,
                "the simulated cell-runner did not come up in time (see the backend logs)".to_owned(),
            ));
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

async fn plans(State(state): State<AppState>) -> Json<PlansInfo> {
    let store = state.config.clone();
    Json(tokio::task::spawn_blocking(move || super::cells::plans(&store)).await.unwrap_or_default())
}

/// One saved plan, whole, for the viewer's behaviour-tree pane; `304` while its copy is current.
async fn get_plan(
    State(state): State<AppState>,
    axum::extract::Path(name): axum::extract::Path<String>,
    headers: axum::http::HeaderMap,
) -> Result<axum::response::Response, HttpError> {
    use axum::http::{StatusCode, header};
    use axum::response::IntoResponse as _;
    use super::cells::PlanFile;

    let name = name.strip_suffix(".json").unwrap_or(&name).to_owned();
    if !super::cells::valid_plan_name(&name) {
        return Err((StatusCode::BAD_REQUEST, format!("invalid plan name: {name}")));
    }
    let if_none_match = headers.get(header::IF_NONE_MATCH).and_then(|value| value.to_str().ok()).map(str::to_owned);
    let store = state.config.clone();
    let plan = tokio::task::spawn_blocking(move || super::cells::read_plan(&store, &name, if_none_match.as_deref()))
        .await
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
        .map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?;
    // `no-cache`: the browser keeps its copy but asks every time (the ETag makes that cheap).
    let cache = (header::CACHE_CONTROL, "no-cache");
    Ok(match plan {
        PlanFile::NotModified { etag } => (StatusCode::NOT_MODIFIED, [(header::ETAG, etag)], [cache]).into_response(),
        PlanFile::Plan { etag, bytes } => (
            [(header::ETAG, etag)],
            [cache, (header::CONTENT_TYPE, "application/json")],
            bytes,
        )
            .into_response(),
    })
}

/// Load a plan into the simulated cell-runner's behaviour-tree executor (and play it), in the
/// scene `request.cell` (default: the active cell): made the active cell and opened in the Robo
/// Fitter, whose 3D view shows the simulation.
async fn load_plan(State(state): State<AppState>, Json(request): Json<LoadPlan>) -> Result<Json<CellResponse>, HttpError> {
    let bad_request = |err: String| (axum::http::StatusCode::BAD_REQUEST, err);
    // Opening the scene and starting its simulation never interleave with Stop simulation and
    // edit: an edit before re-links nothing behind it, one after cancels this Load & Play.
    let flow = state.cell.flow_lock().await;
    let edits = state.cell.edits();
    let active = super::cells::active_cell(&state.config).map(|(name, _)| name);
    let Some(cell) = request.cell.clone().filter(|cell| !cell.is_empty()).or(active.clone()) else {
        return Err(bad_request(
            "no scene to simulate: pick one in step 1 Behaviour Tree (or Robo Fitter → Open cell)".to_owned(),
        ));
    };
    // A Robo Fitter scene without a cell yet: Load & Play makes its cell (only its manifest).
    let cell = {
        let store = state.config.clone();
        tokio::task::spawn_blocking(move || cell_or_scene(&store, &cell))
            .await
            .map_err(|err| (axum::http::StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
            .map_err(|err| bad_request(format!("{err:#}")))?
    };
    if state.fitter.linked_cell().as_deref() != Some(cell.as_str()) {
        // Shows the cell's scene in the Robo Fitter, linked to it; makes it the active cell too.
        let fitter = state.fitter.clone();
        let name = cell.clone();
        let opened = tokio::task::spawn_blocking(move || fitter.open_cell(&name))
            .await
            .map_err(|err| (axum::http::StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
            .map_err(|err| bad_request(format!("could not open the scene {cell}: {err:#}")))?;
        state.bus.put_json(keys::CONFIG, &opened);
    } else if active.as_deref() != Some(cell.as_str()) {
        let config = super::cells::set_active(&state.config, &cell).map_err(|err| bad_request(format!("{err:#}")))?;
        state.bus.put_json(keys::CONFIG, &config);
    }
    if state.fitter.linked_cell().as_deref() != Some(cell.as_str()) {
        state.bus.log(format!("Simulating {cell}, but the cell has no Robo Fitter scene to show it"));
    }
    let params = super::cells::plan_load_params(&state.config, &request.path).map_err(|err| bad_request(format!("{err:#}")))?;
    ensure_cell_started(&state, cell_paths(&state).await)
        .await
        .map_err(|err| (axum::http::StatusCode::SERVICE_UNAVAILABLE, err))?;
    drop(flow);
    wait_cell_ready(&state, edits).await?;
    let response = state.cell.command(CellCommand::with_params("trajectory_load", params)).await;
    state.bus.log(if response.ok {
        format!("Loaded behaviour tree {}", request.path)
    } else {
        format!("Failed to load behaviour tree: {}\nFile: {}", response.error_text(), request.path)
    });
    if response.ok && request.play {
        let played = state.cell.command(CellCommand::new("trajectory_play")).await;
        if !played.ok {
            return Ok(Json(played));
        }
    }
    Ok(Json(response))
}

/// `cell`, a cell folder name; or a Robo Fitter scene (`jam2.yaml`, `jam2`) that has no cell folder
/// of that name: its cell, made if it has none (see `cell_resolve::ensure_cell`).
fn cell_or_scene(store: &crate::config_store::ConfigStore, cell: &str) -> anyhow::Result<String> {
    let info = super::cells::info(store);
    if info.cells.iter().any(|c| c == cell) || super::cells::active_cell(store).is_some_and(|(name, _)| name == cell) {
        return Ok(cell.to_owned());
    }
    match super::scene_store::name_of(cell).filter(|s| info.fitted_scenes.contains(s)) {
        Some(scene) => Ok(super::cell_resolve::ensure_cell(store, &scene)?.name),
        None => Ok(cell.to_owned()),
    }
}

async fn stop(State(state): State<AppState>) -> Json<BackendStatus> {
    state.cell.stop();
    Json(state.cell.status())
}

async fn command(State(state): State<AppState>, Json(command): Json<CellCommand>) -> Json<CellResponse> {
    Json(state.cell.command(command).await)
}

async fn samples() -> Json<Vec<SampleCell>> {
    Json(paths::sample_cells())
}

async fn cells(State(state): State<AppState>) -> Json<CellsInfo> {
    Json(super::cells::info(&state.config))
}

type HttpError = (axum::http::StatusCode, String);

async fn set_active_cell(State(state): State<AppState>, Json(request): Json<SetActiveCell>) -> Result<Json<CellsInfo>, HttpError> {
    let config = super::cells::set_active(&state.config, &request.cell)
        .map_err(|err| (axum::http::StatusCode::BAD_REQUEST, format!("{err:#}")))?;
    state.bus.put_json(keys::CONFIG, &config);
    Ok(Json(super::cells::info(&state.config)))
}

async fn create_cell(State(state): State<AppState>, Json(request): Json<CreateCell>) -> Result<Json<CellsInfo>, HttpError> {
    let store = state.config.clone();
    let config = tokio::task::spawn_blocking(move || super::cells::create_from_fitted(&store, &request.scene, &request.name))
        .await
        .map_err(|err| (axum::http::StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
        .map_err(|err| (axum::http::StatusCode::BAD_REQUEST, format!("{err:#}")))?;
    state.bus.log(format!("Created cell {}", config.env_dir));
    state.bus.put_json(keys::CONFIG, &config);
    Ok(Json(super::cells::info(&state.config)))
}

#[cfg(test)]
mod tests {
    #[test]
    fn waiting_for_a_start_ends_on_ready_failure_stop_or_edit() {
        use super::{BackendState, BackendStatus, StartWait, start_wait};
        let status = |state, connected| BackendStatus { state, connected, ..Default::default() };
        assert_eq!(start_wait(&status(BackendState::Starting, false), true, false), None);
        assert_eq!(start_wait(&status(BackendState::Ready, false), true, false), None, "not connected yet");
        assert_eq!(start_wait(&status(BackendState::Ready, true), true, false), Some(StartWait::Ready));
        assert_eq!(start_wait(&status(BackendState::Stopped, false), false, false), Some(StartWait::Stopped));
        assert_eq!(start_wait(&status(BackendState::Starting, false), true, true), Some(StartWait::Edited));
        let failed = BackendStatus { state: BackendState::Failed, error: Some("boom".into()), ..Default::default() };
        assert_eq!(start_wait(&failed, false, false), Some(StartWait::Failed("boom".into())));
    }

    use super::*;

    fn paths(cell: &str, port: u16) -> CellPaths {
        CellPaths {
            cell: cell.to_owned(),
            cell_dir: format!("/cells/{cell}/cellrunner"),
            env_dir: format!("/cells/{cell}"),
            scene: "scene.yaml".to_owned(),
            mesh_lod: "low".to_owned(),
            cell_port: port,
            inputs: String::new(),
        }
    }

    #[test]
    fn sim_port_avoids_a_taken_port() {
        let held = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let taken = held.local_addr().unwrap().port();
        let port = sim_port(taken);
        assert_ne!(port, taken, "someone holds it");
        assert_ne!(port, 0);
        drop(held);
        // Free again: the preferred port.
        let free = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        assert_eq!(sim_port(free), free);
    }

    #[test]
    fn restarts_only_for_another_cell_or_scene() {
        let running = paths("a", 9902);
        assert!(!needs_restart(None, &paths("a", 9902)), "nothing runs: just start");
        assert!(!needs_restart(Some(&running), &paths("a", 9902)));
        assert!(!needs_restart(Some(&running), &paths("a", 40000)), "the port is picked at each start");
        assert!(needs_restart(Some(&running), &paths("b", 9902)));
        let other_scene = CellPaths { scene: "scene.plan2cell-live.yaml".to_owned(), ..paths("a", 9902) };
        assert!(needs_restart(Some(&running), &other_scene));
        // A reference cell whose scene (or a robot, a product) changed since its view was built.
        let resaved = CellPaths { inputs: "after".to_owned(), ..paths("a", 9902) };
        assert!(needs_restart(Some(&CellPaths { inputs: "before".to_owned(), ..paths("a", 9902) }), &resaved));
    }

    /// Load & Play on a Robo Fitter scene without a cell makes its cell; a cell's name is a cell.
    #[test]
    fn load_and_play_on_a_scene_makes_its_cell() {
        let h = super::super::cell_resolve::tests::host("load-play");
        assert_eq!(cell_or_scene(&h.store, "jam2.yaml").unwrap(), "jam2");
        assert!(h.cells.join("jam2/factoryos-cell.json").is_file());
        assert_eq!(cell_or_scene(&h.store, "jam2").unwrap(), "jam2", "now a cell");
        assert_eq!(cell_or_scene(&h.store, "new-scene").unwrap(), "new_scene", "by the scene's stem");
        assert_eq!(cell_or_scene(&h.store, "nothing").unwrap(), "nothing", "left to the caller's error");
    }
}
