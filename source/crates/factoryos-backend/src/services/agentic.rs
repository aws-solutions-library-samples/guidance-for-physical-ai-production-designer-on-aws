//! Agentic Planning: the umarell multi-robot planner (`multi_robots_agents` checkout), mirroring
//! the Kiro extension's `multi-robots-agents-*` integration:
//! * Docker available: `make sim-detached` / `make down` in the checkout, ready once the
//!   orchestrator's `/health` answers; "Prepare Cell" runs `make convert` for the active cell.
//! * No Docker: only the planner MCP server (:8890) runs, natively from `.venv-planner`
//!   (like the checkout's `run-planner-native.sh`), for the active cell.
//!
//! Planning is driven from the viewer's Kiro chat, through the stack's MCP servers.
//! AWS credentials (the planner calls Bedrock) come from the backend's own environment.

use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Context as _;
use axum::extract::State;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::agentic::{AgenticPrepare, AgenticStatus, PrepareState, SimMode, SimState};
use factoryos_protocol::config::Config;
use factoryos_protocol::{api, keys};
use tokio::io::{AsyncBufReadExt as _, BufReader};
use tokio::process::{Child, Command};

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::cell_resolve::{self, CellKind, ResolvedCell};
use crate::services::cells;
use crate::services::planner_model;
use crate::services::product_tasks;
use crate::services::tcp::ToolCatalogues;

/// A cold `make sim-detached` (image pulls, ROS start-up) can take minutes.
const DOCKER_READY_TIMEOUT: Duration = Duration::from_secs(600);
const NATIVE_READY_TIMEOUT: Duration = Duration::from_secs(120);
const HEALTH_POLL_INTERVAL: Duration = Duration::from_secs(2);
const NATIVE_PLANNER_PORT: u16 = 8890;

/// GUI-launched processes often lack Docker Desktop's and Homebrew's bin directories.
const PATH_AUGMENT_DIRS: [&str; 6] = ["/opt/homebrew/bin", "/usr/local/bin", "/usr/bin", "/bin", "/usr/sbin", "/sbin"];

pub struct AgenticService {
    bus: Bus,
    config: Arc<ConfigStore>,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    status: AgenticStatus,
    /// The native planner process.
    native: Option<Child>,
    /// Prepare Cell requests made while one ran (`None`: the active cell), in order, each once.
    prepare_queue: Vec<Option<String>>,
}

/// What to start, resolved from the config when "Start Simulation" is pressed.
#[derive(Clone, Debug)]
struct Target {
    repo_dir: PathBuf,
    cell: String,
    cells_dir: String,
}

impl AgenticService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>) -> Arc<Self> {
        let this = Arc::new(Self {
            bus: bus.clone(),
            config,
            inner: Mutex::default(),
        });
        {
            let this = this.clone();
            crate::bus::serve_latest(bus, keys::AGENTIC_STATUS, move || Some(this.status()));
        }
        this
    }

    pub fn status(&self) -> AgenticStatus {
        self.inner.lock().unwrap().status.clone()
    }

    fn update(&self, update: impl FnOnce(&mut AgenticStatus)) {
        let status = {
            let mut inner = self.inner.lock().unwrap();
            update(&mut inner.status);
            inner.status.clone()
        };
        self.bus.put_json(keys::AGENTIC_STATUS, &status);
    }

    fn log(&self, line: impl AsRef<str>) {
        self.bus.log(format!("[agentic] {}", line.as_ref()));
    }

    fn target(&self) -> anyhow::Result<Target> {
        self.target_for(None)
    }

    /// The target for `cell` (a folder in the cells folder), default the active cell.
    fn target_for(&self, cell: Option<&str>) -> anyhow::Result<Target> {
        let config = self.config.get();
        let repo_dir = PathBuf::from(self.config.resolve(&config.agentic_repo_dir));
        anyhow::ensure!(
            repo_dir.join("Makefile").is_file(),
            "set Configuration → Agentic Planning → Checkout dir to your multi_robots_agents checkout (no Makefile in {})",
            repo_dir.display()
        );
        let cells = cells::info(&self.config);
        let cell = match cell.map(str::trim).filter(|c| !c.is_empty()) {
            Some(cell) if cells.cells.iter().any(|c| c == cell) => cell.to_owned(),
            Some(cell) => {
                // A Robo Fitter scene (`jam2.yaml`, or `jam2`) without a cell yet: Prepare makes its
                // cell (a folder for its outputs that references the scene).
                let scene = super::scene_store::name_of(cell).filter(|s| cells.fitted_scenes.contains(s));
                let scene = scene.ok_or_else(|| anyhow::anyhow!("no cell {cell} in the cells folder {}", cells.cells_dir))?;
                cell_resolve::ensure_cell(&self.config, &scene)?.name
            }
            None => {
                anyhow::ensure!(!cells.active.is_empty(), "select an active cell first (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)");
                cells.active
            }
        };
        Ok(Target {
            repo_dir,
            cell,
            cells_dir: cells.cells_dir,
        })
    }

    pub fn start(self: &Arc<Self>) {
        if matches!(self.status().state, SimState::Starting | SimState::Ready) {
            return;
        }
        let target = match self.target() {
            Ok(target) => target,
            Err(err) => {
                self.fail(format!("{err:#}"));
                return;
            }
        };
        let mode = if find_in_path("docker").is_some() {
            SimMode::Docker
        } else if native_python(&target.repo_dir).is_file() {
            SimMode::Native
        } else {
            self.fail(format!(
                "neither Docker nor a native planner ({}) is available",
                native_python(&target.repo_dir).display()
            ));
            return;
        };
        self.update(|status| {
            status.state = SimState::Starting;
            status.mode = Some(mode);
            status.repo_dir = target.repo_dir.display().to_string();
            status.cell.clone_from(&target.cell);
            status.error = None;
        });

        let this = self.clone();
        tokio::spawn(async move {
            // The stack reads the cell's catalogue files while it comes up: no catalogue sync deletes meanwhile.
            let _catalogue = crate::storage::sync::in_use();
            let config = this.config.get();
            let result = match mode {
                SimMode::Docker => this.start_docker(&config, &target).await,
                SimMode::Native => this.start_native(&config, &target).await,
            };
            match result {
                Ok(()) => {
                    this.log(format!("Planner stack ready for cell {} ({mode:?})", target.cell));
                    this.update(|status| status.state = SimState::Ready);
                }
                Err(err) => this.fail(format!("{err:#}")),
            }
        });
    }

    async fn start_docker(&self, config: &Config, target: &Target) -> anyhow::Result<()> {
        self.run_make(&target.repo_dir, &sim_make_args(config, &target.cell, &target.cells_dir), config)
            .await?;
        let health = format!("{}/health", config.agentic_orchestrator_url.trim_end_matches('/'));
        self.wait_healthy(&health, DOCKER_READY_TIMEOUT, None).await
    }

    async fn start_native(&self, config: &Config, target: &Target) -> anyhow::Result<()> {
        let health = format!("http://127.0.0.1:{NATIVE_PLANNER_PORT}/health");
        if is_healthy(&health).await {
            // Started outside the backend (e.g. `run-planner-native.sh`): use it, and leave it running on Stop.
            self.log(format!(
                "A planner is already running on :{NATIVE_PLANNER_PORT}; using it (its cell is whatever it was started for)"
            ));
            return Ok(());
        }
        let scratch = std::env::temp_dir().join("umarell-scratch");
        std::fs::create_dir_all(&scratch)?;
        let mut command = Command::new(native_python(&target.repo_dir));
        command
            .arg("agents/trajectory/mcp_server.py")
            .current_dir(&target.repo_dir)
            .env("PATH", augmented_path())
            .env("CELL_BASE_PATH", &target.cells_dir)
            .env("CELL_ID", &target.cell)
            .env("TRAJ_MCP_PORT", NATIVE_PLANNER_PORT.to_string())
            .env("TRAJECTORY_BACKEND", &config.agentic_trajectory_backend)
            .env("PLANNER_SCRATCH_PATH", &scratch)
            .env("PYTHONPATH", &target.repo_dir)
            .env("PYTHONWARNINGS", "ignore")
            .envs(llm_env(config))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        let mut child = command.spawn().context("failed to start the native planner")?;
        self.pipe_to_log(child.stdout.take());
        self.pipe_to_log(child.stderr.take());
        self.log(format!("Started native planner MCP on :{NATIVE_PLANNER_PORT} for cell {}", target.cell));
        self.inner.lock().unwrap().native = Some(child);

        self.wait_healthy(&health, NATIVE_READY_TIMEOUT, Some(())).await
    }

    /// Poll `url` until it answers 2xx. With `native`, also fail as soon as the planner exits.
    async fn wait_healthy(&self, url: &str, timeout: Duration, native: Option<()>) -> anyhow::Result<()> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            if is_healthy(url).await {
                return Ok(());
            }
            if native.is_some() {
                let mut inner = self.inner.lock().unwrap();
                if let Some(child) = inner.native.as_mut()
                    && let Ok(Some(exit)) = child.try_wait()
                {
                    inner.native = None;
                    anyhow::bail!("the native planner exited ({exit}); see View Logs");
                }
            }
            anyhow::ensure!(tokio::time::Instant::now() < deadline, "{url} did not become healthy in {timeout:?}");
            tokio::time::sleep(HEALTH_POLL_INTERVAL).await;
        }
    }

    pub fn stop(self: &Arc<Self>) {
        let (mode, repo_dir, native) = {
            let mut inner = self.inner.lock().unwrap();
            (inner.status.mode, inner.status.repo_dir.clone(), inner.native.take())
        };
        let this = self.clone();
        tokio::spawn(async move {
            let result = match (mode, native) {
                (_, Some(mut child)) => {
                    child.kill().await.ok();
                    this.log("Stopped the native planner");
                    Ok(())
                }
                (Some(SimMode::Docker), None) => this.run_make(Path::new(&repo_dir), &["down".to_owned()], &this.config.get()).await,
                _ => Ok(()),
            };
            match result {
                Ok(()) => this.update(|status| {
                    status.state = SimState::Stopped;
                    status.error = None;
                }),
                Err(err) => this.fail(format!("{err:#}")),
            }
        });
    }

    /// Stop what we started (on backend shutdown). The Docker stack is left to `make down`.
    pub async fn shutdown(&self) {
        let native = self.inner.lock().unwrap().native.take();
        if let Some(mut child) = native {
            child.kill().await.ok();
        }
    }

    /// "Prepare Cell": `make convert` the active cell into an agentic-planner cell, in place.
    pub fn prepare(self: &Arc<Self>) {
        self.prepare_cell(None);
    }

    /// Prepare Cell for `cell` (a folder in the cells folder; default the active cell), e.g. the planner router's
    /// job on a cell that is not the active one. One at a time: a request while one runs is queued (once; not
    /// for the cell being prepared) and runs after it (the status' `prepareCell` says whose is running).
    pub fn prepare_cell(self: &Arc<Self>, cell: Option<&str>) {
        {
            let mut inner = self.inner.lock().unwrap();
            if inner.status.prepare == PrepareState::Preparing {
                let cell = cell.map(str::trim).filter(|c| !c.is_empty()).map(str::to_owned);
                let running = cell.as_deref().is_some_and(|c| c == inner.status.prepare_cell);
                if !running && !inner.prepare_queue.contains(&cell) {
                    inner.prepare_queue.push(cell);
                }
                return;
            }
            // Claimed (published below): a request at the same time is queued.
            inner.status.prepare = PrepareState::Preparing;
        }
        let target = match self.target_for(cell) {
            Ok(target) => target,
            Err(err) => {
                let message = format!("{err:#}");
                let cell = cell.unwrap_or_default().to_owned();
                self.update(|status| {
                    status.prepare = PrepareState::Failed;
                    status.prepare_error = Some(message);
                    status.prepare_cell = cell;
                });
                self.prepare_next();
                return;
            }
        };
        self.update(|status| {
            status.prepare = PrepareState::Preparing;
            status.prepare_error = None;
            status.prepare_cell.clone_from(&target.cell);
        });
        let this = self.clone();
        tokio::spawn(async move {
            let result = async {
                // Where the cell's inputs are: a legacy cell's own scene.yaml and robots/, or (a
                // reference cell) the Robo Fitter scene and the catalogue's URDFs, read in place.
                let resolved = {
                    let (config, cell) = (this.config.clone(), target.cell.clone());
                    tokio::task::spawn_blocking(move || cell_resolve::resolve(&config, &cell)).await??
                };
                let cell_dir = resolved.root.clone();
                let scene_yaml = resolved.scene_path.clone();
                anyhow::ensure!(scene_yaml.is_file(), "cell {} has no scene.yaml to prepare", target.cell);
                let reference = resolved.kind == CellKind::Reference;
                // A reference cell's USDA bundle is extracted into its view (`.view/usda`).
                let view = if reference {
                    let r = resolved.clone();
                    Some(tokio::task::spawn_blocking(move || cell_resolve::materialise_view(&r)).await??)
                } else {
                    None
                };
                let usda = match &view {
                    Some(view) => find_usda(&view.join(cell_resolve::VIEW_USDA)),
                    None => find_usda(&cell_dir),
                };
                let cell_id = sanitize_cell_id(&target.cell);
                // What the converter does not derive: each robot's TCP (from its dressed URDF) and
                // the pedestal it stands on. Written into its output once it ran.
                let tools = ToolCatalogues::from_config(&this.config);
                let robots = {
                    let scene_yaml = scene_yaml.clone();
                    tokio::task::spawn_blocking(move || planner_model::read_scene(&scene_yaml, &tools)).await??
                };
                for line in planner_model::describe(&robots) {
                    this.log(line);
                }
                let tcp = planner_model::shared_tcp(&robots);
                // A reference cell's SRDFs: in the cell, never beside the catalogue's URDFs.
                let srdfs = if reference { reference_srdfs(&resolved) } else { Vec::new() };
                let native = native_python(&target.repo_dir);
                let docker = find_in_path("docker").is_some() && !(reference && native.is_file());
                // Near the product's welds the converter models the fixture with a fine voxel
                // shell (when the deployed one can: an older converter rejects the flags).
                let refine = {
                    let (config, root, repo) = (this.config.clone(), cell_dir.clone(), target.repo_dir.clone());
                    tokio::task::spawn_blocking(move || prepare_refine_points(&root, &cells::weld_points(&config, &root), &repo, docker))
                        .await??
                };
                match &refine {
                    RefinePoints::None => {}
                    RefinePoints::Unsupported(n) => this.log(format!(
                        "The deployed converter has no --refine-points: the coarse voxel shell stays near the {n} weld(s)"
                    )),
                    RefinePoints::Written { path, count } => this.log(format!(
                        "Near-weld refinement: {count} weld(s) in {} ({} m voxels within {} m)",
                        path.display(),
                        product_tasks::REFINE_VOXEL_SIZE_M,
                        product_tasks::REFINE_RADIUS_M
                    )),
                }
                if docker {
                    // The container sees the cells folder only: a reference cell converts from its
                    // view (symlinks into the catalogue; best effort, the native run is preferred).
                    let scene_yaml = view.as_ref().map_or_else(|| scene_yaml.clone(), |v| v.join("scene.yaml"));
                    let args = convert_make_args(&cell_dir, &scene_yaml, usda.as_deref(), &cell_id, tcp, refine.path());
                    this.run_make(&target.repo_dir, &args, &this.config.get()).await?;
                } else {
                    // No Docker: the converter and SRDF generator are plain Python; run them from the planner's venv.
                    anyhow::ensure!(native.is_file(), "Prepare Cell needs Docker or the checkout's .venv-planner");
                    let missing: Vec<(PathBuf, PathBuf)> = if reference {
                        srdfs.iter().filter(|s| srdf_out_of_date(&s.0, &s.1)).cloned().collect()
                    } else {
                        robot_urdfs_without_srdf(&cell_dir).into_iter().map(|urdf| (urdf.clone(), urdf.with_extension("srdf"))).collect()
                    };
                    for (urdf, srdf) in missing {
                        if let Some(parent) = srdf.parent() {
                            std::fs::create_dir_all(parent)?;
                        }
                        let args = ["scripts/generate_srdf.py".into(), "--urdf".into(), urdf.display().to_string(), "--out".into(), srdf.display().to_string()];
                        this.run_python(&target.repo_dir, &args).await?;
                    }
                    let args = convert_script_args(&cell_dir, &scene_yaml, usda.as_deref(), &cell_id, tcp, refine.path());
                    this.run_python(&target.repo_dir, &args).await?;
                }
                planner_model::apply(&cell_dir, &robots)?;
                if reference {
                    let rel: Vec<(PathBuf, String)> = srdfs
                        .iter()
                        .filter(|(_, srdf)| srdf.is_file())
                        .filter_map(|(urdf, srdf)| Some((urdf.clone(), srdf.strip_prefix(&cell_dir).ok()?.display().to_string())))
                        .collect();
                    planner_model::apply_srdfs(&cell_dir, &rel)?;
                    // The view was made before the SRDFs: rebuilt with them beside their URDFs (the
                    // planner router's jobs link its robots/).
                    let r = resolved.clone();
                    tokio::task::spawn_blocking(move || cell_resolve::materialise_view(&r)).await??;
                }
                let pedestals = robots.iter().filter(|r| r.pedestal.is_some()).count();
                this.log(format!(
                    "Set each robot's TCP{} in {}",
                    if pedestals > 0 { format!(" and {pedestals} pedestal(s) (base height, obstacle)") } else { String::new() },
                    planner_model::ROBOTS_YAML
                ));
                // The converter rewrote obstacles.yaml: mark the boxes the product welds are on as
                // is_workpiece again, and bring the task programs up to date with the products.
                let (config, root) = (this.config.clone(), cell_dir.clone());
                if let Some(line) = tokio::task::spawn_blocking(move || super::cells::derive_tasks(&config, &root)).await? {
                    this.log(line);
                }
                // What the prepared model was made from: it is stale once these change.
                cell_resolve::record_prepared(&resolved)?;
                // The prepared model, the SRDFs, the task programs (and the plans so far) to the
                // user's store (S3 mode); the plans the planner saves later go by storage::watch.
                crate::storage::saved_tree(cell_dir.join("agentic_planner"));
                if reference {
                    this.log(format!(
                        "Prepared cell {} from the Robo Fitter scene {} (referenced, nothing copied)",
                        target.cell,
                        scene_yaml.display()
                    ));
                }
                Ok(())
            }
            .await;
            this.update(|status| match result {
                Ok(()) => status.prepare = PrepareState::Done,
                Err(err) => {
                    status.prepare = PrepareState::Failed;
                    status.prepare_error = Some(format!("{err:#}"));
                }
            });
            this.prepare_next();
        });
    }

    /// Prepare Cell for `cells`, one after another (see [`Self::prepare_cell`]).
    pub fn prepare_cells(self: &Arc<Self>, cells: &[String]) {
        for cell in cells {
            self.prepare_cell(Some(cell));
        }
    }

    /// Start the next queued Prepare Cell, if any.
    fn prepare_next(self: &Arc<Self>) {
        let next = {
            let mut inner = self.inner.lock().unwrap();
            if inner.prepare_queue.is_empty() { None } else { Some(inner.prepare_queue.remove(0)) }
        };
        if let Some(cell) = next {
            self.prepare_cell(cell.as_deref());
        }
    }

    fn fail(&self, message: String) {
        self.log(format!("Failed: {message}"));
        self.update(|status| {
            status.state = SimState::Failed;
            status.error = Some(message);
        });
    }

    async fn run_make(&self, repo_dir: &Path, args: &[String], config: &Config) -> anyhow::Result<()> {
        self.log(format!("make {} (in {})", args.join(" "), repo_dir.display()));
        let mut child = Command::new(find_in_path("make").unwrap_or_else(|| PathBuf::from("make")))
            .args(args)
            .current_dir(repo_dir)
            .env("PATH", augmented_path())
            .envs(llm_env(config))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true)
            .spawn()
            .context("failed to run make")?;
        self.pipe_to_log(child.stdout.take());
        self.pipe_to_log(child.stderr.take());
        let exit = child.wait().await?;
        anyhow::ensure!(exit.success(), "make {} failed ({exit}); see View Logs", args.first().map_or("", String::as_str));
        Ok(())
    }

    async fn run_python(&self, repo_dir: &Path, args: &[String]) -> anyhow::Result<()> {
        self.log(format!("python {}", args.join(" ")));
        let mut child = Command::new(native_python(repo_dir))
            .args(args)
            .current_dir(repo_dir)
            .env("PATH", augmented_path())
            .env("PYTHONPATH", repo_dir)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true)
            .spawn()
            .context("failed to run the planner's python")?;
        self.pipe_to_log(child.stdout.take());
        self.pipe_to_log(child.stderr.take());
        let exit = child.wait().await?;
        anyhow::ensure!(exit.success(), "{} failed ({exit}); see View Logs", args.first().map_or("", String::as_str));
        Ok(())
    }

    fn pipe_to_log(&self, reader: Option<impl tokio::io::AsyncRead + Send + Unpin + 'static>) {
        let Some(reader) = reader else { return };
        let bus = self.bus.clone();
        tokio::spawn(async move {
            let mut lines = BufReader::new(reader).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                bus.log(format!("[agentic] {line}"));
            }
        });
    }
}

async fn is_healthy(url: &str) -> bool {
    let Ok(client) = reqwest::Client::builder().timeout(Duration::from_secs(2)).build() else {
        return false;
    };
    client.get(url).send().await.is_ok_and(|r| r.status().is_success())
}

fn native_python(repo_dir: &Path) -> PathBuf {
    repo_dir.join(".venv-planner/bin/python")
}

/// The planner's LLM settings (the AWS credentials themselves come from our environment).
fn llm_env(config: &Config) -> Vec<(String, String)> {
    let mut env = Vec::new();
    if !config.agentic_region.trim().is_empty() {
        env.push(("BEDROCK_REGION".to_owned(), config.agentic_region.trim().to_owned()));
    }
    if !config.agentic_model.trim().is_empty() {
        env.push(("LLM_MODEL".to_owned(), config.agentic_model.trim().to_owned()));
    }
    env
}

/// argv for `make sim-detached`; optional variables only when set, so the Makefile's defaults hold.
fn sim_make_args(config: &Config, cell: &str, cells_dir: &str) -> Vec<String> {
    let mut args = vec!["sim-detached".to_owned(), format!("SIM={cell}")];
    if !cells_dir.trim().is_empty() {
        args.push(format!("CELL_BASE={}", cells_dir.trim()));
    }
    args.push(format!("RVIZ={}", if config.agentic_rviz_enabled { "1" } else { "0" }));
    args.push(format!("TRAJECTORY_BACKEND={}", config.agentic_trajectory_backend));
    args.push(format!("RMW={}", config.agentic_rmw));
    if !config.agentic_model.trim().is_empty() {
        args.push(format!("LLM_MODEL={}", config.agentic_model.trim()));
    }
    // ZENOH_ROUTER only matters (and is only required) on the zenoh transport.
    if config.agentic_rmw == "zenoh" && !config.agentic_zenoh_router.trim().is_empty() {
        args.push(format!("ZENOH_ROUTER={}", config.agentic_zenoh_router.trim()));
    }
    args
}

/// The converter's `--tcp-offset` value.
fn tcp_arg(tcp: [f64; 3]) -> String {
    tcp.map(|v| format!("{v}")).join(" ")
}

/// The converter script, in the umarell checkout.
const CONVERTER: &str = "scripts/convert_isaac_scene.py";
/// The cell's weld points for the converter's near-weld refinement (`--refine-points`).
pub const REFINE_POINTS: &str = "agentic_planner/refine_points.json";

/// What Prepare Cell passes the converter for the near-weld refinement.
#[derive(Clone, Debug, PartialEq)]
enum RefinePoints {
    /// No welds (or nothing to pass).
    None,
    /// The cell has this many welds, but the converter (or, `make convert`, the Makefile) has no
    /// `--refine-points`.
    Unsupported(usize),
    /// The weld points file.
    Written { path: PathBuf, count: usize },
}

impl RefinePoints {
    fn path(&self) -> Option<&Path> {
        match self {
            Self::Written { path, .. } => Some(path),
            _ => None,
        }
    }
}

/// Whether the umarell checkout `repo_dir`'s converter takes `--refine-points` (the cell
/// planner's upstream patch 25; umarell eccb1bc's rejects unknown flags), and for `make convert`
/// (`make`) whether its Makefile passes `REFINE_POINTS=` on: read from the files, every Prepare Cell.
fn refine_supported(repo_dir: &Path, make: bool) -> bool {
    let has = |file: &str, what: &str| std::fs::read_to_string(repo_dir.join(file)).is_ok_and(|text| text.contains(what));
    has(CONVERTER, "--refine-points") && (!make || has("Makefile", "REFINE_POINTS"))
}

/// Write cell `cell_dir`'s weld `points` (world frame) as [`REFINE_POINTS`] for the converter of
/// umarell checkout `repo_dir` (run by `make convert`: `make`), if it has any and the converter
/// can use them; a stale file is removed otherwise.
fn prepare_refine_points(cell_dir: &Path, points: &[[f64; 3]], repo_dir: &Path, make: bool) -> anyhow::Result<RefinePoints> {
    let path = cell_dir.join(REFINE_POINTS);
    if points.is_empty() || !refine_supported(repo_dir, make) {
        if path.exists() {
            std::fs::remove_file(&path).with_context(|| format!("removing {}", path.display()))?;
            crate::storage::saved(&path);
        }
        return Ok(if points.is_empty() { RefinePoints::None } else { RefinePoints::Unsupported(points.len()) });
    }
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&path, serde_json::to_string(points)? + "\n").with_context(|| format!("writing {}", path.display()))?;
    Ok(RefinePoints::Written { path, count: points.len() })
}

/// The converter's refinement values (m), as `make` and the script take them.
fn refine_values() -> [String; 2] {
    [product_tasks::REFINE_VOXEL_SIZE_M.to_string(), product_tasks::REFINE_RADIUS_M.to_string()]
}

/// argv for `make convert` of a cell, in place; `tcp`: the TCP offset all its robots share;
/// `refine`: the weld points file for the near-weld refinement.
fn convert_make_args(
    cell_dir: &Path,
    scene_yaml: &Path,
    usda: Option<&Path>,
    cell_id: &str,
    tcp: Option<[f64; 3]>,
    refine: Option<&Path>,
) -> Vec<String> {
    let mut args = vec![
        "convert".to_owned(),
        format!("OUT={}", cell_dir.display()),
        format!("SCENE_YAML={}", scene_yaml.display()),
    ];
    if let Some(usda) = usda {
        args.push(format!("SCENE={}", usda.display()));
    }
    if !cell_id.is_empty() {
        args.push(format!("CELL_ID={cell_id}"));
    }
    if let Some(tcp) = tcp {
        args.push(format!("TCP_OFFSET={}", tcp_arg(tcp)));
    }
    if let Some(points) = refine {
        let [voxel, radius] = refine_values();
        args.extend([format!("REFINE_POINTS={}", points.display()), format!("REFINE_VOXEL_SIZE={voxel}"), format!("REFINE_RADIUS={radius}")]);
    }
    args
}

/// `convert_isaac_scene.py` arguments, the same as `make convert` passes.
fn convert_script_args(
    cell_dir: &Path,
    scene_yaml: &Path,
    usda: Option<&Path>,
    cell_id: &str,
    tcp: Option<[f64; 3]>,
    refine: Option<&Path>,
) -> Vec<String> {
    let mut args = vec![
        CONVERTER.to_owned(),
        "--scene-yaml".to_owned(),
        scene_yaml.display().to_string(),
        "--out-dir".to_owned(),
        cell_dir.display().to_string(),
    ];
    if let Some(usda) = usda {
        args.extend(["--scene".to_owned(), usda.display().to_string()]);
    }
    if !cell_id.is_empty() {
        args.extend(["--cell-id".to_owned(), cell_id.to_owned()]);
    }
    if let Some(tcp) = tcp {
        // One argument: a value starting with '-' would read as an option.
        args.push(format!("--tcp-offset={}", tcp_arg(tcp)));
    }
    if let Some(points) = refine {
        let [voxel, radius] = refine_values();
        args.extend([
            "--refine-points".to_owned(),
            points.display().to_string(),
            "--refine-voxel-size".to_owned(),
            voxel,
            "--refine-radius".to_owned(),
            radius,
        ]);
    }
    args
}

/// The cell's dressed robot URDFs (`robots/<pkg>/<pkg>.urdf`) with no SRDF beside them.
/// Without one, the planner's collision checker keeps every self-collision pair.
fn robot_urdfs_without_srdf(cell_dir: &Path) -> Vec<PathBuf> {
    let mut urdfs: Vec<PathBuf> = std::fs::read_dir(cell_dir.join("robots"))
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            let urdf = entry.path().join(format!("{name}.urdf"));
            let has_srdf = std::fs::read_dir(entry.path())
                .into_iter()
                .flatten()
                .flatten()
                .any(|f| f.path().extension().is_some_and(|e| e == "srdf"));
            (urdf.is_file() && !has_srdf).then_some(urdf)
        })
        .collect();
    urdfs.sort();
    urdfs
}

/// A reference cell's robots' SRDFs: `(URDF, <cell>/agentic_planner/srdf/<URDF stem>.srdf)`, one
/// per URDF (two arms may share a dressed model).
fn reference_srdfs(r: &ResolvedCell) -> Vec<(PathBuf, PathBuf)> {
    let mut out: Vec<(PathBuf, PathBuf)> = Vec::new();
    for robot in &r.robots {
        if out.iter().any(|(urdf, _)| *urdf == robot.urdf) {
            continue;
        }
        let stem = robot.urdf.file_stem().unwrap_or_default().to_string_lossy().into_owned();
        out.push((robot.urdf.clone(), r.root.join(cell_resolve::SRDF_DIR).join(format!("{stem}.srdf"))));
    }
    out
}

/// Whether `srdf` must be (re)generated for `urdf`: it is missing or older than the URDF (re-dressed).
fn srdf_out_of_date(urdf: &Path, srdf: &Path) -> bool {
    let modified = |p: &Path| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    match (modified(urdf), modified(srdf)) {
        (_, None) => true,
        (Some(urdf), Some(srdf)) => srdf < urdf,
        (None, Some(_)) => false,
    }
}

/// A cell id usable as a ROS 2 namespace: `[A-Za-z0-9_]`, not starting with a digit.
fn sanitize_cell_id(name: &str) -> String {
    cells::planner_cell_id(name)
}

/// The cell's USD stage, from its extracted USDA bundle or its root.
fn find_usda(cell_dir: &Path) -> Option<PathBuf> {
    [cell_dir.join("usda-extracted"), cell_dir.to_path_buf()].into_iter().find_map(|dir| {
        let mut found: Vec<PathBuf> = std::fs::read_dir(dir)
            .ok()?
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|ext| ext == "usda"))
            .collect();
        found.sort();
        found.into_iter().next()
    })
}

fn augmented_path() -> String {
    let existing = std::env::var("PATH").unwrap_or_default();
    let mut dirs: Vec<&str> = PATH_AUGMENT_DIRS.to_vec();
    for dir in existing.split(':').filter(|d| !d.is_empty()) {
        if !dirs.contains(&dir) {
            dirs.push(dir);
        }
    }
    dirs.join(":")
}

fn find_in_path(command: &str) -> Option<PathBuf> {
    augmented_path()
        .split(':')
        .map(|dir| Path::new(dir).join(command))
        .find(|path| path.is_file())
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::AGENTIC_STATUS, get(status))
        .route(api::AGENTIC_START, post(start))
        .route(api::AGENTIC_STOP, post(stop))
        .route(api::AGENTIC_PREPARE, post(prepare))
}

async fn status(State(state): State<AppState>) -> Json<AgenticStatus> {
    Json(state.agentic.status())
}

async fn start(State(state): State<AppState>) -> Json<AgenticStatus> {
    state.agentic.start();
    Json(state.agentic.status())
}

async fn stop(State(state): State<AppState>) -> Json<AgenticStatus> {
    state.agentic.stop();
    Json(state.agentic.status())
}

/// The body is optional (`{"cell": "<folder>"}`): without one, or unreadable, the active cell.
async fn prepare(State(state): State<AppState>, body: axum::body::Bytes) -> Json<AgenticStatus> {
    let request = prepare_request(&body);
    state.agentic.prepare_cell(request.cell.as_deref());
    Json(state.agentic.status())
}

fn prepare_request(body: &[u8]) -> AgenticPrepare {
    serde_json::from_slice(body).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_args_omit_unset_variables() {
        let config = Config::default();
        assert_eq!(
            sim_make_args(&config, "welding_cell_2x", ""),
            ["sim-detached", "SIM=welding_cell_2x", "RVIZ=0", "TRAJECTORY_BACKEND=ompl", "RMW=cyclonedds"]
        );
        let config = Config {
            agentic_rviz_enabled: true,
            agentic_model: " opus-5 ".to_owned(),
            agentic_rmw: "zenoh".to_owned(),
            agentic_zenoh_router: "tcp/router:7447".to_owned(),
            ..Config::default()
        };
        assert_eq!(
            sim_make_args(&config, "c", "/cells"),
            [
                "sim-detached",
                "SIM=c",
                "CELL_BASE=/cells",
                "RVIZ=1",
                "TRAJECTORY_BACKEND=ompl",
                "RMW=zenoh",
                "LLM_MODEL=opus-5",
                "ZENOH_ROUTER=tcp/router:7447"
            ]
        );
        // The router is ignored on cyclonedds.
        let config = Config {
            agentic_zenoh_router: "tcp/router:7447".to_owned(),
            ..Config::default()
        };
        assert!(!sim_make_args(&config, "c", "").iter().any(|a| a.starts_with("ZENOH_ROUTER")));
    }

    #[test]
    fn convert_args() {
        let cell = Path::new("/cells/my-cell");
        assert_eq!(
            convert_make_args(cell, &cell.join("scene.yaml"), None, "my_cell", None, None),
            ["convert", "OUT=/cells/my-cell", "SCENE_YAML=/cells/my-cell/scene.yaml", "CELL_ID=my_cell"]
        );
        let usda = cell.join("usda-extracted/scene.usda");
        assert!(convert_make_args(cell, &cell.join("scene.yaml"), Some(&usda), "x", None, None).contains(&format!("SCENE={}", usda.display())));
    }

    #[test]
    fn native_convert_args_match_make_convert() {
        let cell = Path::new("/cells/c");
        assert_eq!(
            convert_script_args(cell, &cell.join("scene.yaml"), None, "c", None, None),
            ["scripts/convert_isaac_scene.py", "--scene-yaml", "/cells/c/scene.yaml", "--out-dir", "/cells/c", "--cell-id", "c"]
        );
        // The robots' shared TCP, as one argument (it starts with '-').
        let tcp = Some([-0.9092, 0.0056, 0.0005]);
        assert!(convert_script_args(cell, &cell.join("scene.yaml"), None, "c", tcp, None).contains(&"--tcp-offset=-0.9092 0.0056 0.0005".to_owned()));
        assert!(convert_make_args(cell, &cell.join("scene.yaml"), None, "c", tcp, None).contains(&"TCP_OFFSET=-0.9092 0.0056 0.0005".to_owned()));
    }

    #[test]
    fn near_weld_refinement_args() {
        let cell = Path::new("/cells/c");
        let points = cell.join(REFINE_POINTS);
        let script = convert_script_args(cell, &cell.join("scene.yaml"), None, "c", None, Some(&points));
        assert_eq!(
            script[script.len() - 6..],
            ["--refine-points", "/cells/c/agentic_planner/refine_points.json", "--refine-voxel-size", "0.02", "--refine-radius", "0.25"]
        );
        let make = convert_make_args(cell, &cell.join("scene.yaml"), None, "c", None, Some(&points));
        assert_eq!(
            make[make.len() - 3..],
            ["REFINE_POINTS=/cells/c/agentic_planner/refine_points.json", "REFINE_VOXEL_SIZE=0.02", "REFINE_RADIUS=0.25"]
        );
        assert!(!convert_script_args(cell, &cell.join("scene.yaml"), None, "c", None, None).iter().any(|a| a.contains("refine")));
    }

    /// The points file is written only for welds and a converter that takes the flags (an older
    /// umarell's rejects unknown ones); `make convert` also needs the Makefile to pass them on.
    #[test]
    fn refine_points_only_for_a_converter_that_takes_them() {
        let dir = crate::services::tcp::tests::TestDir::new();
        let (cell, repo) = (dir.path().join("cell"), dir.path().join("umarell"));
        std::fs::create_dir_all(repo.join("scripts")).unwrap();
        std::fs::write(repo.join(CONVERTER), "parser.add_argument('--voxel-size')\n").unwrap();
        std::fs::write(repo.join("Makefile"), "convert:\n\tpython3 scripts/convert_isaac_scene.py\n").unwrap();
        let welds = [[0.5, 0.0, 1.0], [0.6, 0.1, 1.0]];
        assert_eq!(prepare_refine_points(&cell, &welds, &repo, false).unwrap(), RefinePoints::Unsupported(2));
        assert!(!cell.join(REFINE_POINTS).exists());

        std::fs::write(repo.join(CONVERTER), "parser.add_argument('--refine-points')\n").unwrap();
        let written = prepare_refine_points(&cell, &welds, &repo, false).unwrap();
        assert_eq!(written, RefinePoints::Written { path: cell.join(REFINE_POINTS), count: 2 });
        let file: Vec<[f64; 3]> = serde_json::from_str(&std::fs::read_to_string(cell.join(REFINE_POINTS)).unwrap()).unwrap();
        assert_eq!(file, welds);
        assert_eq!(prepare_refine_points(&cell, &welds, &repo, true).unwrap(), RefinePoints::Unsupported(2), "the Makefile does not pass it");
        assert!(!cell.join(REFINE_POINTS).exists(), "a stale file is removed");
        std::fs::write(repo.join("Makefile"), "convert:\n\t$(if $(REFINE_POINTS),--refine-points \"$(REFINE_POINTS)\",)\n").unwrap();
        assert!(matches!(prepare_refine_points(&cell, &welds, &repo, true).unwrap(), RefinePoints::Written { .. }));
        assert_eq!(prepare_refine_points(&cell, &[], &repo, true).unwrap(), RefinePoints::None);
        assert!(!cell.join(REFINE_POINTS).exists());
    }

    #[test]
    fn finds_robots_missing_an_srdf() {
        let cell = std::env::temp_dir().join(format!("factoryos-srdf-{}", std::process::id()));
        for (pkg, srdf) in [("a", false), ("b", true)] {
            let dir = cell.join("robots").join(pkg);
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join(format!("{pkg}.urdf")), "<robot/>").unwrap();
            if srdf {
                std::fs::write(dir.join(format!("{pkg}.srdf")), "<robot/>").unwrap();
            }
        }
        assert_eq!(robot_urdfs_without_srdf(&cell), vec![cell.join("robots/a/a.urdf")]);
        std::fs::remove_dir_all(&cell).ok();
    }

    /// Prepare Cell on a reference cell: the converter reads the Robo Fitter scene itself (absolute
    /// path; its URDF paths resolve from scenes/ into the catalogue) and writes into the cell;
    /// the SRDFs go into the cell's agentic_planner/srdf/, never beside the catalogue's URDFs.
    #[test]
    fn a_reference_cell_prepares_from_its_fitted_scene() {
        let h = cell_resolve::tests::host("prepare-reference");
        let r = cell_resolve::ensure_cell(&h.store, "jam2.yaml").unwrap();
        let args = convert_script_args(&r.root, &r.scene_path, None, "jam2", None, None);
        assert_eq!(args[2], h.assets.join("scenes/jam2/scene.yaml").display().to_string());
        assert_eq!(args[4], r.root.display().to_string());
        let urdf = h.assets.join("urdf-dressed-robots/kr240__weld/kr240__weld.urdf");
        let srdfs = reference_srdfs(&r);
        assert_eq!(srdfs, [(urdf.clone(), r.root.join("agentic_planner/srdf/kr240__weld.srdf"))]);
        assert!(srdf_out_of_date(&urdf, &srdfs[0].1), "none yet");
        std::fs::create_dir_all(srdfs[0].1.parent().unwrap()).unwrap();
        std::fs::write(&srdfs[0].1, "<robot/>").unwrap();
        assert!(!srdf_out_of_date(&urdf, &srdfs[0].1));
        // Re-dressed after the SRDF was made: made again.
        let later = std::time::SystemTime::now() + std::time::Duration::from_secs(5);
        std::fs::File::options().write(true).open(&urdf).unwrap().set_modified(later).unwrap();
        assert!(srdf_out_of_date(&urdf, &srdfs[0].1));
        // Nothing is ever written into the catalogue.
        assert!(!urdf.with_extension("srdf").exists());
    }

    #[test]
    fn cell_ids_are_ros_namespaces() {
        assert_eq!(sanitize_cell_id("welding-cell.2"), "welding_cell_2");
        assert_eq!(sanitize_cell_id("2cell"), "cell_2cell");
    }

    #[test]
    fn prepare_body_is_optional() {
        assert_eq!(prepare_request(b"").cell, None);
        assert_eq!(prepare_request(b"{}").cell, None);
        assert_eq!(prepare_request(b"not json").cell, None);
        assert_eq!(prepare_request(br#"{"cell": "jam2"}"#).cell.as_deref(), Some("jam2"));
    }

    #[test]
    fn llm_env_only_when_set() {
        assert_eq!(llm_env(&Config::default()), [("BEDROCK_REGION".to_owned(), "us-east-1".to_owned())]);
        let config = Config {
            agentic_region: String::new(),
            agentic_model: "sonnet-5".to_owned(),
            ..Config::default()
        };
        assert_eq!(llm_env(&config), [("LLM_MODEL".to_owned(), "sonnet-5".to_owned())]);
    }
}
