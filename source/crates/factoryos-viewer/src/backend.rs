//! The viewer's connection to factoryos-backend, which runs the cell (and, over time, every other
//! subsystem) on its behalf.
//!
//! * HTTP: config, loading behaviour trees into the simulation.
//! * Zenoh: the simulation's state and status and its commands (`factoryos/sim/*`: the backend's
//!   simulated cell-runner, shown in the Robo Fitter's 3D view), backend log lines, and the LIVE
//!   cell (`factoryos/cell/*`, on a server connected to the virtual cell): its state (the head,
//!   which cell), and its 3D scene (`factoryos/scene/cell`, fos-live-scene) through
//!   [`crate::scene3d`] (Watch the cell).
//!
//! Natively the backend is `FACTORY_OS_BACKEND` or the one picked at startup (see
//! [`crate::backend_choice`]). For this machine's, if none is running, the viewer starts the
//! `factoryos-backend` binary next to its own executable and stops it again on exit.

use std::sync::{Arc, Mutex};
#[cfg(not(target_arch = "wasm32"))]
use std::time::Duration;

use factoryos_client::protocol::agentic::AgenticStatus;
use factoryos_client::protocol::codegen::CodegenStatus;
use factoryos_client::protocol::infra::InfraStatus;
use factoryos_client::protocol::{
    self, BackendStatus, CellCommand, CellResponse, CellsInfo, CreateCell, Health, LoadPlan, PlansInfo, SetActiveCell, api, keys,
};
use factoryos_client::{Api, Bus};
use crate::rr::re_log_channel::LogReceiver;
use crate::rr::re_log_types::StoreId;
#[cfg(not(target_arch = "wasm32"))]
use crate::backend_choice;
use crate::rr::{egui, re_log};
use crate::scene3d::Scene3d;
use serde_json::Value;

pub use factoryos_client::protocol::BackendState;

#[cfg(not(target_arch = "wasm32"))]
const SPAWN_TIMEOUT: Duration = Duration::from_secs(20);

#[derive(Default)]
pub struct Shared {
    /// The simulation's status (`factoryos/sim/status`).
    pub status: BackendStatus,
    pub log: Vec<String>,

    /// Latest `state` snapshot from the simulated cell-runner (`factoryos/sim/state`).
    pub cell_state: Option<Value>,
    /// The last head state fos-live-bridge published (`plan2cell.head` of the live cell's state)
    /// and when it came; kept when the state stops carrying it, so step 5 can say it is stale.
    pub live_head: Option<(Value, web_time::Instant)>,
    /// The live cell (`factoryos/cell/state`), if one streams here.
    pub live: Option<LiveCell>,
    /// The live cell's status (`factoryos/cell/status`, fos-live-bridge / fos-live-scene).
    pub live_status: BackendStatus,
    /// Connected to the simulated cell-runner.
    pub connected: bool,

    /// The live cell's recording (its scene `factoryos/scene/cell`, once one has arrived).
    pub recording_id: Option<StoreId>,
    /// Live cell recordings replaced by a new scene (a restarted fos-live-scene), for the app to
    /// close (and to route to `recording_id` instead if one is shown).
    pub replaced_recordings: Vec<StoreId>,
    /// A freshly created log stream for the viewer to pick up (the live cell's, once).
    pub new_receiver: Option<LogReceiver>,

    /// The cells to pick from and the fitted scenes to create cells from.
    pub cells: CellsInfo,
    /// Why the last cell selection/creation failed.
    pub cells_error: Option<String>,

    /// The Agentic Planning stack.
    pub agentic: AgenticStatus,
    /// The Control Code Export build.
    pub codegen: CodegenStatus,
    /// The Virtual Cell (`factoryos/infra/status`), with when it last came.
    pub infra: Option<(InfraStatus, u64)>,

    /// The behaviour trees planned for the active cell.
    pub plans: PlansInfo,
    /// A Load & Play request is in flight (the backend starts the simulation for it).
    pub plan_in_flight: bool,
    /// The last plan load: `Ok(tree)` or `Err(why)`.
    pub plan_result: Option<Result<String, String>>,
    /// How many plan loads succeeded (the behaviour-tree pane fetches the plan again after one).
    pub plan_loads: u64,

    commands: Vec<CellCommand>,
    /// The live cell's 3D scene (fos-live-scene), Rerun application `robotic_cell`.
    pub(crate) cell_scene: Option<Scene3d>,
    egui_ctx: Option<egui::Context>,
}

/// The live cell, as its state names it.
#[derive(Clone, Debug)]
pub struct LiveCell {
    /// Its folder name (`cell_name` before ` (live, …)`).
    pub cell: Option<String>,
    /// When its state last came.
    pub at: web_time::Instant,
}

impl Shared {
    pub fn state(&self) -> BackendState {
        self.status.state
    }

    /// Queue a cell-runner command (`start`, `stop`, `reset`, `set_speed`, …).
    pub fn command(&mut self, kind: &str, params: Option<Value>) {
        self.commands.push(CellCommand {
            kind: kind.to_owned(),
            params,
        });
    }

    pub(crate) fn push_log(&mut self, line: impl Into<String>) {
        let line = line.into();
        re_log::debug!("{line}");
        self.log.push(line);
        if self.log.len() > 2000 {
            self.log.drain(..500);
        }
    }

    pub(crate) fn repaint(&self) {
        if let Some(ctx) = &self.egui_ctx {
            ctx.request_repaint();
        }
    }

    /// The simulation's status.
    fn set_status(&mut self, status: BackendStatus) {
        let restarted = status.state == BackendState::Starting && self.status.state != BackendState::Starting;
        if restarted || !matches!(status.state, BackendState::Starting | BackendState::Ready) {
            // A new run, or none: forget the last run's state.
            self.cell_state = None;
        }
        self.connected = status.connected;
        self.status = status;
    }

    /// The live cell's status. (A restarted fos-live-scene announces a new scene, which replaces
    /// the one shown: [`crate::scene3d`].)
    fn set_live_status(&mut self, status: BackendStatus) {
        self.live_status = status;
    }

    /// The live cell's state: the head (step 5) and which cell it is.
    fn set_live_state(&mut self, state: &Value) {
        if let Some(head) = Some(&state["plan2cell"]["head"]).filter(|head| head.is_object()) {
            self.live_head = Some((head.clone(), web_time::Instant::now()));
        }
        let cell = state["cell_name"].as_str().and_then(crate::view_mode::live_cell_name);
        self.live = Some(LiveCell { cell, at: web_time::Instant::now() });
    }

    /// Hand the live cell's scene to Rerun once it has ingested the last batch, so a backlog
    /// (hidden page, busy viewer) shows the cell's latest pose instead of replaying it. Call every
    /// frame, before Rerun's `logic`, with the recording the viewer shows (`active`): a new live
    /// scene (fos-live-scene restarted, another cell) is framed again once shown, like the tools'
    /// scenes ([`Scene3d::set_active_recording`]); poses and joints never re-frame it. Returns
    /// whether messages still wait (then repaint again).
    pub fn forward_stream(&mut self, active: Option<&StoreId>) -> bool {
        let Some(scene) = &self.cell_scene else { return false };
        scene.set_active_recording(active);
        let waiting = scene.forward();
        self.recording_id = scene.current();
        // The app routes to the live cell itself (Watch the cell, `recording_id`), and closes what
        // a new scene replaced.
        if let Some(request) = scene.take_route_request() {
            self.replaced_recordings.extend(request.close);
        }
        waiting
    }
}

pub type SharedBackend = Arc<Mutex<Shared>>;

pub struct Backend {
    pub api: Api,
    pub bus: Bus,
    /// Whether the backend speaks this viewer's protocol; if not, no 3D scene is connected and the
    /// UI says so ([`ProtocolCheck::banner`]).
    pub protocol: ProtocolCheck,
    pub shared: SharedBackend,
    /// Geometry of every 3D scene, fetched by content hash (see [`crate::scene3d`]).
    pub assets: Arc<crate::scene3d::assets::AssetCache>,
    /// The config as loaded from the backend at startup.
    pub initial_config: protocol::config::ConfigResponse,
    /// The backend process's boot id at connecting (`/api/v1/health`; empty for an older
    /// backend): another one later means it restarted (see [`crate::reconnect`]).
    pub boot_id: String,
    /// The backend process, if we started it ourselves.
    #[cfg(not(target_arch = "wasm32"))]
    spawned: Option<std::process::Child>,
}

/// Whether the viewer and the backend speak the same protocol (`/api/v1/health`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProtocolCheck {
    Compatible,
    /// Another protocol: the 3D scenes are not connected (their messages may not even decode), and
    /// the UI shows [`ProtocolCheck::banner`] until the viewer or the backend is updated.
    Mismatch { viewer: u32, backend: u32, backend_version: String, release: Option<String> },
}

impl ProtocolCheck {
    /// The check for a viewer of protocol `viewer` and the backend's `health`.
    pub fn of(viewer: u32, health: &Health) -> Self {
        if health.protocol_version == viewer {
            return Self::Compatible;
        }
        Self::Mismatch {
            viewer,
            backend: health.protocol_version,
            backend_version: health.backend_version.clone(),
            release: health.release.clone(),
        }
    }

    /// Whether the 3D scenes may be connected.
    pub fn scenes_enabled(&self) -> bool {
        *self == Self::Compatible
    }

    /// What the UI says about a mismatch, if there is one.
    pub fn banner(&self) -> Option<String> {
        let Self::Mismatch { viewer, backend, backend_version, release } = self else { return None };
        let release = release.as_deref().map(|release| format!(", release {release}")).unwrap_or_default();
        let update = if viewer < backend { "update the viewer" } else { "update the backend" };
        Some(format!(
            "Viewer protocol v{viewer}, backend v{backend} (factoryos-backend {backend_version}{release}): {update} \
             to the same release; 3D disabled"
        ))
    }
}

/// The backend is behind the edge's login and there is nothing to log in with silently
/// (`factoryos_client::auth::login_silently`): [`Backend::login_with_browser`], or what the
/// message says.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug)]
pub struct LoginNeeded(pub String);

#[cfg(not(target_arch = "wasm32"))]
impl std::fmt::Display for LoginNeeded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl std::error::Error for LoginNeeded {}

/// The workspace exists (or is being created) but its backend is not up yet: wait and retry.
#[derive(Debug, Clone)]
pub struct WorkspaceNotReady(pub protocol::workspace::WorkspaceStatus);

impl std::fmt::Display for WorkspaceNotReady {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "workspace {}: {}", self.0.name, self.0.describe())
    }
}

impl std::error::Error for WorkspaceNotReady {}

/// How long to wait between connection attempts to a workspace that is coming up.
#[cfg(not(target_arch = "wasm32"))]
pub const WORKSPACE_POLL: std::time::Duration = std::time::Duration::from_secs(3);

/// The health check's error as [`WorkspaceNotReady`] if that's what it is.
fn not_reachable(api: &Api, err: String) -> anyhow::Error {
    match factoryos_client::workspace_status(&err) {
        Some(status) => WorkspaceNotReady(status).into(),
        None => anyhow::anyhow!("factoryos-backend at {} is not reachable: {err}", api.base()),
    }
}

/// For an `https://` backend behind the edge's login: make sure there is a session (saved, or
/// silently where the build can), and talk to the origin itself, with the workspace `url` names
/// (`…/workspace/<name>`) selected by the edge's cookie, or else the user's own. Other backends,
/// https ones without that login, and any without a session where the build has no silent login
/// (the public one), are used as they are.
#[cfg(not(target_arch = "wasm32"))]
async fn with_session(url: &str) -> anyhow::Result<String> {
    use factoryos_client::auth::{self, LoginError};

    let Some(origin) = auth::origin(url).filter(|_| url.starts_with("https://")) else { return Ok(url.to_owned()) };
    let session = match auth::session(&origin) {
        Some(session) => session,
        None => {
            let login_origin = origin.clone();
            match crate::rr::tokio::task::spawn_blocking(move || auth::login_silently(&login_origin)).await? {
                Ok(session) => session,
                Err(LoginError::NoSilentSession(why)) => return Err(LoginNeeded(why).into()),
                Err(LoginError::Failed(why)) => {
                    re_log::debug!("{origin}: {why}; connecting without a session");
                    return Ok(url.to_owned());
                }
            }
        }
    };
    let selected = protocol::workspace::of_url(url).filter(|name| Some(*name) != auth::alias(&session));
    auth::set_workspace(&origin, selected);
    Ok(origin)
}

impl Backend {
    /// Log in to the backend at `url` through the system browser (see [`LoginNeeded`]); then
    /// [`Self::connect_to`] finds the session.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn login_with_browser(url: &str) -> anyhow::Result<()> {
        let origin = factoryos_client::auth::origin(url).ok_or_else(|| anyhow::anyhow!("not a URL: {url}"))?;
        re_log::info!("Signing in to {origin} in the browser…");
        factoryos_client::auth::login_with_browser(&origin, |login| {
            webbrowser::open(login).map_err(|err| format!("could not open the browser: {err} (open {login})"))
        })
        .await?;
        Ok(())
    }

    /// Connect to the backend at `url`. For [`backend_choice::LOCAL`], start one next to this
    /// executable if none is running there yet. Behind the edge's login, log in first
    /// ([`LoginNeeded`] if that takes the user).
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn connect_to(url: &str) -> anyhow::Result<Self> {
        let url = with_session(url).await?;
        let api = Api::new(url);
        let mut spawned = None;
        let health = match api.get_async::<Health>(api::HEALTH).await {
            Ok(health) => health,
            Err(_) if api.base() == backend_choice::LOCAL => {
                spawned = Some(spawn_local_backend()?);
                wait_for_health(&api).await?
            }
            Err(err) => return Err(not_reachable(&api, err)),
        };
        match Self::connect_with(api, health).await {
            Ok(mut backend) => {
                backend.spawned = spawned;
                Ok(backend)
            }
            Err(err) => {
                if let Some(mut child) = spawned {
                    child.kill().ok();
                }
                Err(err)
            }
        }
    }

    /// Whether this viewer started the backend (which then stops with it).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn spawned(&self) -> bool {
        self.spawned.is_some()
    }

    /// Connect to the backend at `url` (the web viewer's: the backend that served it).
    #[cfg(target_arch = "wasm32")]
    pub async fn connect(url: String) -> anyhow::Result<Self> {
        let api = Api::new(url);
        let health = api.get_async::<Health>(api::HEALTH).await.map_err(|err| not_reachable(&api, err))?;
        Self::connect_with(api, health).await
    }

    async fn connect_with(api: Api, health: Health) -> anyhow::Result<Self> {
        let check = ProtocolCheck::of(protocol::PROTOCOL_VERSION, &health);
        if let Some(banner) = check.banner() {
            re_log::warn!("{banner}");
        }

        let endpoint = std::env::var("FACTORY_OS_ZENOH")
            .ok()
            .or_else(|| factoryos_client::zenoh_endpoint(api.base(), &health))
            .ok_or_else(|| {
                if health.zenoh_ws {
                    anyhow::anyhow!("factoryos-backend at {} advertises no Zenoh endpoint reachable from here", api.base())
                } else {
                    anyhow::anyhow!(
                        "factoryos-backend {} at {} only takes Zenoh on {}, not over its HTTP port \
                         (--zenoh-ws-port 0), which viewers on other machines need",
                        health.backend_version,
                        api.base(),
                        health.zenoh_endpoints.join(", ")
                    )
                }
            })?;
        let bus = Bus::connect(&endpoint).await.map_err(|err| anyhow::anyhow!(err))?;
        re_log::info!("Connected to factoryos-backend {} (zenoh {endpoint})", api.base());

        let initial_config = api.get_async(api::CONFIG).await.map_err(|err| match check.banner() {
            // The UI can't come up to say so: the error does.
            Some(banner) => anyhow::anyhow!("{banner} ({err})"),
            None => anyhow::anyhow!(err),
        })?;
        let backend = Self {
            assets: crate::scene3d::Scene3d::asset_cache(&api),
            api,
            bus,
            protocol: check,
            shared: SharedBackend::default(),
            initial_config,
            boot_id: health.boot_id.clone(),
            #[cfg(not(target_arch = "wasm32"))]
            spawned: None,
        };
        backend.subscribe()?;
        if backend.protocol.scenes_enabled() {
            // The live cell's scene, on its own log channel (handed to Rerun by the app's `poll_backend`).
            let mut scene = Scene3d::connect(&backend.bus, backend.assets.clone(), keys::SCENE_CELL, "robotic_cell");
            let mut shared = backend.shared.lock().unwrap();
            shared.new_receiver = scene.take_receiver();
            shared.cell_scene = Some(scene);
        }
        Ok(backend)
    }

    pub fn set_egui_ctx(&self, ctx: &egui::Context) {
        let mut shared = self.shared.lock().unwrap();
        if let Some(scene) = &shared.cell_scene {
            scene.set_egui_ctx(ctx);
        }
        shared.egui_ctx = Some(ctx.clone());
    }

    /// Request a repaint (once the UI is up), e.g. from a Zenoh callback.
    pub fn repaint_handle(&self) -> impl Fn() + Send + Sync + 'static {
        let shared = self.shared.clone();
        move || shared.lock().unwrap().repaint()
    }

    fn subscribe(&self) -> anyhow::Result<()> {
        let on = |key: &str, handle: fn(&mut Shared, &[u8])| {
            let shared = self.shared.clone();
            self.bus
                .subscribe(key, move |payload| {
                    let mut shared = shared.lock().unwrap();
                    handle(&mut shared, &payload);
                    shared.repaint();
                })
                .map_err(|err| anyhow::anyhow!(err))
        };
        // The simulation (the backend's simulated cell-runner).
        on(keys::SIM_STATUS, |shared, payload| {
            if let Ok(status) = serde_json::from_slice(payload) {
                shared.set_status(status);
            }
        })?;
        on(keys::SIM_STATE, |shared, payload| {
            if shared.status.state == BackendState::Ready {
                shared.cell_state = serde_json::from_slice(payload).ok();
            }
        })?;
        // The live cell (fos-live-bridge / fos-live-scene), whatever the simulation does.
        on(keys::CELL_STATUS, |shared, payload| {
            if let Ok(status) = serde_json::from_slice(payload) {
                shared.set_live_status(status);
            }
        })?;
        on(keys::CELL_STATE, |shared, payload| {
            if let Ok(state) = serde_json::from_slice::<Value>(payload) {
                shared.set_live_state(&state);
            }
        })?;
        on(keys::LOG, |shared, payload| {
            shared.push_log(String::from_utf8_lossy(payload));
        })?;
        // The shared catalogue changed: the panels listing it list it again.
        on(keys::CATALOGUE_CHANGED, |_, payload| crate::catalogue_events::received(payload))?;
        // The chat agent moves the 3D view (view_* tools): applied by the app each frame.
        on(keys::VIEW_COMMAND, |shared, payload| {
            crate::view_commands::received(payload);
            shared.repaint();
        })?;
        // Published when they change; queried now (a late joiner) and after every re-join (a
        // restarted backend, a lost connection): see `Bus::follow`.
        let follow = |key: &str, handle: fn(&mut Shared, &[u8])| {
            let shared = self.shared.clone();
            self.bus
                .follow(key, move |payload| {
                    let mut shared = shared.lock().unwrap();
                    handle(&mut shared, &payload);
                    shared.repaint();
                })
                .map_err(|err| anyhow::anyhow!(err))
        };
        follow(keys::AGENTIC_STATUS, |shared, payload| {
            if let Ok(status) = serde_json::from_slice(payload) {
                shared.agentic = status;
            }
        })?;
        follow(keys::CODEGEN_STATUS, |shared, payload| {
            if let Ok(status) = serde_json::from_slice(payload) {
                shared.codegen = status;
            }
        })?;
        follow(keys::INFRA_STATUS, |shared, payload| {
            if let Ok(status) = serde_json::from_slice(payload) {
                let n = shared.infra.as_ref().map_or(0, |(_, n)| n + 1);
                shared.infra = Some((status, n));
            }
        })?;

        let catch_up = self.catch_up();
        catch_up();
        self.bus.on_rejoin(catch_up);

        self.cells_refresher()();
        Ok(())
    }

    /// Catch up with a simulation that is already running (status, then state) and a live cell's
    /// status (fos-live-scene answers; its scene comes through `cell_scene`, snapshot then live):
    /// on connecting, and after every re-join, when what was published meanwhile was missed.
    fn catch_up(&self) -> impl factoryos_client::RejoinFn {
        let (shared, bus) = (self.shared.clone(), self.bus.clone());
        move || {
            let (sim, bus2) = (shared.clone(), bus.clone());
            bus.query(keys::SIM_STATUS, None, move |reply| {
                let Ok(Some(payload)) = reply else { return };
                let Ok(status) = serde_json::from_slice::<BackendStatus>(&payload) else { return };
                let running = status.state == BackendState::Ready;
                sim.lock().unwrap().set_status(status);
                if running {
                    let sim = sim.clone();
                    bus2.query(keys::SIM_STATE, None, move |reply| {
                        let mut shared = sim.lock().unwrap();
                        if let Ok(Some(payload)) = reply
                            && shared.status.state == BackendState::Ready
                        {
                            shared.cell_state = serde_json::from_slice(&payload).ok();
                        }
                        shared.repaint();
                    });
                } else {
                    sim.lock().unwrap().repaint();
                }
            });
            let live = shared.clone();
            bus.query(keys::CELL_STATUS, None, move |reply| {
                let Ok(Some(payload)) = reply else { return };
                let Ok(status) = serde_json::from_slice::<BackendStatus>(&payload) else { return };
                let mut shared = live.lock().unwrap();
                shared.set_live_status(status);
                shared.repaint();
            });
            true
        }
    }

    /// Re-read the cells folder and fitted scenes (e.g. after a config change).
    pub fn cells_refresher(&self) -> impl Fn() + Send + Sync + 'static {
        let (api, shared) = (self.api.clone(), self.shared.clone());
        move || {
            let shared = shared.clone();
            api.get(api::CELLS, move |cells: Result<CellsInfo, String>| apply_cells(&shared, cells));
        }
    }

    /// Agentic Planning: start (`start`), stop or prepare the active cell for the planner stack.
    /// The resulting status arrives on `factoryos/agentic/status`.
    pub fn agentic(&self, path: &'static str) {
        let shared = self.shared.clone();
        self.api.post(path, move |status: Result<AgenticStatus, String>| {
            if let Err(err) = status {
                shared.lock().unwrap().push_log(format!("Agentic Planning: {err}"));
            }
        });
    }

    /// Make `cell` (in the cells folder) the active cell.
    pub fn set_active_cell(&self, cell: &str) {
        let shared = self.shared.clone();
        let request = SetActiveCell { cell: cell.to_owned() };
        self.api
            .post_json(api::CELL_ACTIVE, &request, move |cells: Result<CellsInfo, String>| apply_cells(&shared, cells));
    }

    /// Use the saved Robo Fitter scene `scene` as a cell: the backend makes its cell folder (or
    /// reuses the one that references it; nothing is copied) and makes it the active cell. `then`
    /// gets the active cell once it is.
    pub fn use_scene(&self, scene: &str, then: impl FnOnce(&str) + Send + 'static) {
        let shared = self.shared.clone();
        let request = CreateCell { scene: scene.to_owned(), name: String::new() };
        self.api.post_json(api::CELL_CREATE, &request, move |cells: Result<CellsInfo, String>| {
            let active = cells.as_ref().ok().map(|cells| cells.active.clone()).filter(|cell| !cell.is_empty());
            apply_cells(&shared, cells);
            if let Some(active) = active {
                then(&active);
            }
        });
    }

    /// Re-list the active cell's behaviour trees (and, when they changed, the cells: a Prepare or
    /// Plan Cell also changes the cell's stale badge).
    pub fn refresh_plans(&self) {
        let shared = self.shared.clone();
        let refresh_cells = self.cells_refresher();
        self.api.get(api::CELL_PLANS, move |plans: Result<PlansInfo, String>| {
            // A failed request keeps what was listed: an empty list would read as a cell switch
            // (and reset what the code export and the task editor show).
            let plans = match plans {
                Ok(plans) => plans,
                Err(err) => {
                    re_log::debug!("listing the plans: {err}");
                    return;
                }
            };
            let changed = {
                let mut shared = shared.lock().unwrap();
                let changed = shared.plans != plans;
                if changed {
                    shared.plans = plans;
                    shared.repaint();
                }
                changed
            };
            if changed {
                refresh_cells();
            }
        });
    }

    /// Load a behaviour tree into the simulation's executor in the scene `plan.cell` and play it;
    /// the backend starts (or restarts, for another cell) the simulated cell-runner for it.
    pub fn load_plan(&self, plan: LoadPlan) {
        {
            let mut shared = self.shared.lock().unwrap();
            shared.plan_result = None;
            shared.plan_in_flight = true;
        }
        let shared = self.shared.clone();
        let tree = plan.path.rsplit('/').next().unwrap_or(&plan.path).trim_end_matches(".json").to_owned();
        self.api.post_json(api::CELL_PLAN_LOAD, &plan, move |response: Result<CellResponse, String>| {
            let mut shared = shared.lock().unwrap();
            shared.plan_in_flight = false;
            let result = match response {
                Ok(response) if response.ok => Ok(tree),
                Ok(response) => Err(response.error_text()),
                Err(err) => Err(err),
            };
            match &result {
                Ok(_) => shared.plan_loads += 1,
                Err(err) => shared.push_log(format!("Failed to load behaviour tree: {err}")),
            }
            shared.plan_result = Some(result);
            shared.repaint();
        });
    }

    /// Start the simulation of the active cell (`FACTORY_OS_AUTOSTART`).
    pub fn start(&self) {
        {
            let mut shared = self.shared.lock().unwrap();
            if matches!(shared.state(), BackendState::Starting | BackendState::Ready) {
                return;
            }
            shared.set_status(BackendStatus {
                state: BackendState::Starting,
                ..Default::default()
            });
        }
        let shared = self.shared.clone();
        self.api.post(api::CELL_START, move |status: Result<BackendStatus, String>| {
            if let Err(err) = status {
                let mut shared = shared.lock().unwrap();
                shared.push_log(format!("Failed to start the simulation: {err}"));
                shared.set_status(BackendStatus {
                    state: BackendState::Failed,
                    error: Some(err),
                    ..Default::default()
                });
                shared.repaint();
            }
        });
    }

    /// Send queued commands to the simulation. Call once per frame.
    pub fn flush_commands(&self) {
        let commands = std::mem::take(&mut self.shared.lock().unwrap().commands);
        for command in commands {
            let Ok(payload) = serde_json::to_vec(&command) else { continue };
            let shared = self.shared.clone();
            let kind = command.kind;
            self.bus.query(keys::SIM_COMMAND, Some(payload), move |reply| {
                let error = match reply {
                    Ok(Some(payload)) => match serde_json::from_slice::<CellResponse>(&payload) {
                        Ok(response) if response.ok => None,
                        Ok(response) => Some(response.error_text()),
                        Err(err) => Some(format!("invalid reply: {err}")),
                    },
                    Ok(None) => Some("no reply from the backend".to_owned()),
                    Err(err) => Some(err),
                };
                if let Some(error) = error {
                    let mut shared = shared.lock().unwrap();
                    shared.push_log(format!("Command {kind} failed: {error}"));
                    shared.repaint();
                }
            });
        }
    }

    /// Stop the backend if we started it (which also stops the cell). A shared backend keeps running.
    #[cfg(target_arch = "wasm32")]
    pub fn shutdown(&mut self) {}

    /// Stop the backend if we started it (which also stops the cell). A shared backend keeps running.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn shutdown(&mut self) {
        let Some(mut child) = self.spawned.take() else { return };
        #[cfg(unix)]
        if let Ok(pid) = i32::try_from(child.id()) {
            // SAFETY: plain syscall to our own child; SIGTERM lets it stop the cell cleanly.
            #[expect(unsafe_code)]
            unsafe {
                libc::kill(pid, libc::SIGTERM);
            }
        }
        let deadline = web_time::Instant::now() + Duration::from_secs(10);
        while web_time::Instant::now() < deadline {
            if matches!(child.try_wait(), Ok(Some(_))) {
                return;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        child.kill().ok();
    }
}

fn apply_cells(shared: &SharedBackend, cells: Result<CellsInfo, String>) {
    let mut shared = shared.lock().unwrap();
    match cells {
        Ok(cells) => {
            shared.cells = cells;
            shared.cells_error = None;
        }
        Err(err) => {
            shared.push_log(format!("Cells: {err}"));
            shared.cells_error = Some(err);
        }
    }
    shared.repaint();
}

/// Start `factoryos-backend` from next to our own executable (`target/<profile>/`).
#[cfg(not(target_arch = "wasm32"))]
fn spawn_local_backend() -> anyhow::Result<std::process::Child> {
    let exe = std::env::current_exe()?;
    let binary = exe.with_file_name(format!("factoryos-backend{}", std::env::consts::EXE_SUFFIX));
    anyhow::ensure!(
        binary.exists(),
        "no factoryos-backend running at {} and none found at {} \
         (build it with `cargo build -p factoryos-backend`, or set FACTORY_OS_BACKEND)",
        backend_choice::LOCAL,
        binary.display()
    );
    re_log::info!("Starting {}", binary.display());
    Ok(std::process::Command::new(binary)
        .args(["--bind", "127.0.0.1:8080", "--zenoh-listen", "tcp/127.0.0.1:7447"])
        .stdin(std::process::Stdio::null())
        .spawn()?)
}

#[cfg(not(target_arch = "wasm32"))]
async fn wait_for_health(api: &Api) -> anyhow::Result<Health> {
    let deadline = web_time::Instant::now() + SPAWN_TIMEOUT;
    loop {
        match api.get_async::<Health>(api::HEALTH).await {
            Ok(health) => return Ok(health),
            Err(err) if web_time::Instant::now() > deadline => {
                anyhow::bail!("factoryos-backend did not come up at {}: {err}", api.base())
            }
            Err(_) => crate::rr::tokio::time::sleep(Duration::from_millis(200)).await,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn health(protocol_version: u32, release: Option<&str>) -> Health {
        serde_json::from_value(serde_json::json!({
            "protocolVersion": protocol_version,
            "backendVersion": "0.9.0",
            "zenohEndpoints": [],
            "release": release,
        }))
        .unwrap()
    }

    #[test]
    fn the_same_protocol_connects_the_scenes() {
        let check = ProtocolCheck::of(4, &health(4, None));
        assert_eq!(check, ProtocolCheck::Compatible);
        assert!(check.scenes_enabled());
        assert_eq!(check.banner(), None);
    }

    #[test]
    fn another_protocol_disables_3d_and_says_what_to_update() {
        let older_backend = ProtocolCheck::of(4, &health(3, Some("p1234")));
        assert!(!older_backend.scenes_enabled());
        assert_eq!(
            older_backend.banner().unwrap(),
            "Viewer protocol v4, backend v3 (factoryos-backend 0.9.0, release p1234): update the backend to the same \
             release; 3D disabled"
        );
        let newer_backend = ProtocolCheck::of(4, &health(5, None));
        assert!(!newer_backend.scenes_enabled());
        assert!(newer_backend.banner().unwrap().contains("v4, backend v5 (factoryos-backend 0.9.0): update the viewer"));
    }
}
