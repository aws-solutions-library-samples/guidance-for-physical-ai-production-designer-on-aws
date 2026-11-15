//! Control Code Export: a saved plan of the active cell → head-PLC code and robot programs, with
//! the `plan2cell` CLI.
//!
//! plan2cell is a workspace of its own (its trust-platform dependencies pin another zenoh than
//! ours), so the backend runs its binary (`bin/plan2cell` in a release, see
//! [`crate::paths::plan2cell_binary`]) rather than linking it:
//! * `plan2cell options --json`: what can be built, cached per binary (and its mtime); the
//!   defaults the form starts from are its defaults with Factory OS's (profile vw-ks24,
//!   [`DEFAULT_PROFILE`]; on a plan2cell virtual cell `sim`, [`VIRTUAL_CELL_PROFILE`]) over them and
//!   the active cell's plan2cell.toml on top;
//! * `plan2cell build <plan> -c <dir>/plan2cell.toml -o <dir> --force --json ...`: one JSON report
//!   on stdout, which becomes the build's [`BuildInfo`].
//!
//! A build lives in `<cell>/plan2cell/builds/<plan>/<UTC time>/`: the generated files, the
//! `plan2cell.toml` it was built with (the cell's own `plan2cell/plan2cell.toml` if it has one,
//! on a plan2cell virtual cell with its KUKA VCs' hardware values where that sets none, with the
//! request's options and the cell's saved Hardware Configuration on top, see
//! [`crate::services::hardware`]) and `build.json`. `<plan>/latest` names the newest. One build
//! runs at a time; its progress is published on [`keys::CODEGEN_STATUS`]. A saved build (and
//! `latest`) is written through to the user's store (S3 mode): the Virtual Cell deploys it, and a
//! new instance lists it as it was.

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use anyhow::Context as _;
use axum::extract::{Path as UrlPath, State};
use axum::http::{StatusCode, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::codegen::{
    BuildAccepted, BuildEntry, BuildFile, BuildInfo, BuildList, BuildRequest, BuildSummary, Choice, CodegenDefaults,
    CodegenOptions, CodegenState, CodegenStatus, Diagnostic, RobotSummary, Severity, ZoneSummary,
};
use factoryos_protocol::{api, keys};
use tokio::io::{AsyncBufReadExt as _, AsyncReadExt as _, BufReader};
use tokio::process::Command;

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::cell_resolve::{self, InputHashes, ResolvedCell};
use crate::services::files::contained_file;
use crate::services::{cells, hardware};
use factoryos_protocol::hardware::{HardwareConfig, HardwareSource};

/// A build of a plan takes a second or two; the ST type check of a big cell somewhat longer.
const BUILD_TIMEOUT: Duration = Duration::from_secs(300);
const OPTIONS_TIMEOUT: Duration = Duration::from_secs(30);
/// How long the CLI's stderr may stay open once it has exited (or was killed): a descendant that
/// inherited it is killed with the CLI's process group after this.
const LOG_DRAIN: Duration = Duration::from_secs(2);
/// At most this much of the CLI's JSON report is read.
const MAX_REPORT_BYTES: u64 = 64 << 20;
/// `build.json` files larger than this are not read.
const MAX_INFO_BYTES: u64 = 16 << 20;
/// The largest single file served raw.
const MAX_FILE_BYTES: u64 = 8 << 20;
/// The largest build zipped.
const MAX_ARCHIVE_BYTES: u64 = 256 << 20;

const BUILDS_DIR: &str = "plan2cell/builds";
/// The cell's own plan2cell.toml (hardware, robots, ...), the base of every build's.
pub(crate) const BASE_CONFIG: &str = "plan2cell/plan2cell.toml";
/// The export form's profile unless the cell's plan2cell.toml names one (plan2cell's own is `sim`).
pub const DEFAULT_PROFILE: &str = "vw-ks24";
/// The export form's profile on a plan2cell virtual cell: its KUKA VCs run bt2krl's `sim` programs
/// (bt_cell, bt_sps; the deploy resets sps.sub to the stock file, dropping the VW port's loop, which
/// is what writes a vw-ks24 robot's status bits such as A13 O_R_Notaus: Start production ended in
/// ERR_SAFETY_TIMEOUT).
pub const VIRTUAL_CELL_PROFILE: &str = "sim";
const BUILD_INFO: &str = "build.json";
const BUILD_CONFIG: &str = "plan2cell.toml";
const LATEST: &str = "latest";
/// `YYYYMMDDTHHMMSSmmmZ`.
const STAMP_LEN: usize = 19;

type HttpError = (StatusCode, String);

/// Where status and log lines go: the Zenoh bus, or a test's recorder.
pub trait Sink: Send + Sync {
    fn status(&self, status: &CodegenStatus);
    fn log(&self, line: String);
}

impl Sink for Bus {
    fn status(&self, status: &CodegenStatus) {
        self.put_json(keys::CODEGEN_STATUS, status);
    }

    fn log(&self, line: String) {
        Bus::log(self, line);
    }
}

pub struct CodegenService {
    sink: Arc<dyn Sink>,
    config: Arc<ConfigStore>,
    /// The CLI; looked up again with `lookup` while there is none (built after the backend started).
    binary: Mutex<Option<PathBuf>>,
    lookup: Option<fn() -> Option<PathBuf>>,
    timeout: Duration,
    /// Whether builds run for a plan2cell virtual cell (its KUKA VCs' values in every base).
    virtual_cell: hardware::VirtualCell,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    status: CodegenStatus,
    running: bool,
    /// `plan2cell options --json` of this binary at this mtime.
    options: Option<(PathBuf, Option<SystemTime>, CodegenOptions)>,
}

/// A build about to run.
struct Job {
    id: String,
    cell: String,
    plan_rel: String,
    plan_file: PathBuf,
    dir: PathBuf,
    request: BuildRequest,
    created: SystemTime,
    plan_etag: String,
    base_config_etag: Option<String>,
    /// The cell's saved hardware configuration (in the build's plan2cell.toml), and its ETag.
    hardware: Option<(HardwareConfig, String)>,
    /// What the plan was made from (its `planner_meta.inputs`; else the cell's inputs now):
    /// recorded in `build.json` as `inputs`, the build is stale once the cell's inputs differ.
    inputs: Option<InputHashes>,
}

impl CodegenService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>) -> Arc<Self> {
        let mut this = Self::with_sink(Arc::new(bus.clone()), config, crate::paths::plan2cell_binary());
        {
            let this = Arc::get_mut(&mut this).expect("not shared yet");
            this.lookup = Some(crate::paths::plan2cell_binary);
            this.virtual_cell = hardware::VirtualCell::from_env();
        }
        {
            let this = this.clone();
            crate::bus::serve_latest(bus, keys::CODEGEN_STATUS, move || Some(this.status()));
        }
        if let Some(binary) = &*this.binary.lock().unwrap() {
            tracing::info!("Control Code Export: {}", binary.display());
        } else {
            tracing::warn!("Control Code Export: no plan2cell binary (set FACTORY_OS_PLAN2CELL)");
        }
        this
    }

    /// With this `binary` only (no lookup when it is `None`).
    pub fn with_sink(sink: Arc<dyn Sink>, config: Arc<ConfigStore>, binary: Option<PathBuf>) -> Arc<Self> {
        Arc::new(Self {
            sink,
            config,
            binary: Mutex::new(binary),
            lookup: None,
            timeout: BUILD_TIMEOUT,
            virtual_cell: hardware::VirtualCell::none(),
            inner: Mutex::default(),
        })
    }

    /// For builds on `virtual_cell` (tests; before it is shared).
    pub fn on_virtual_cell(mut self: Arc<Self>, virtual_cell: hardware::VirtualCell) -> Arc<Self> {
        Arc::get_mut(&mut self).expect("not shared yet").virtual_cell = virtual_cell;
        self
    }

    pub fn status(&self) -> CodegenStatus {
        self.inner.lock().unwrap().status.clone()
    }

    /// Store and publish `status`, under the lock: what is published comes in the order it was
    /// stored (the sink only queues it).
    fn update(&self, status: CodegenStatus) {
        let mut inner = self.inner.lock().unwrap();
        inner.status = status;
        self.sink.status(&inner.status);
    }

    /// A build's end: free for the next one and its final status, in one step, so a build started
    /// right after cannot have its Running replaced by this Done/Failed.
    fn finish(&self, status: CodegenStatus) {
        let mut inner = self.inner.lock().unwrap();
        inner.running = false;
        inner.status = status;
        self.sink.status(&inner.status);
    }

    fn log(&self, line: impl AsRef<str>) {
        self.sink.log(format!("[codegen] {}", line.as_ref()));
    }

    fn binary(&self) -> Result<PathBuf, HttpError> {
        let binary = {
            let mut binary = self.binary.lock().unwrap();
            if binary.is_none()
                && let Some(lookup) = self.lookup
            {
                *binary = lookup();
            }
            binary.clone()
        };
        let binary = binary.ok_or_else(|| {
            (
                StatusCode::SERVICE_UNAVAILABLE,
                "no plan2cell binary: install it next to factoryos-backend (bin/plan2cell) or set FACTORY_OS_PLAN2CELL".to_owned(),
            )
        })?;
        if !binary.is_file() {
            return Err((
                StatusCode::SERVICE_UNAVAILABLE,
                format!("the plan2cell binary {} does not exist (FACTORY_OS_PLAN2CELL)", binary.display()),
            ));
        }
        Ok(binary)
    }

    /// What can be built, with the defaults of the active cell (see [`CodegenOptions`]): Factory
    /// OS's ([`factoryos_defaults`]: profile vw-ks24, `sim` on a virtual cell) over plan2cell's, the
    /// cell's plan2cell.toml on top.
    pub async fn options(&self) -> Result<CodegenOptions, HttpError> {
        let mut options = self.cli_options().await?;
        let mut defaults = options.defaults.clone();
        factoryos_defaults(&options, &mut defaults, self.virtual_cell.active());
        options.defaults = defaults;
        if let Some((cell, root)) = cells::active_cell(&self.config) {
            if let Ok(base) = std::fs::read_to_string(root.join(BASE_CONFIG)) {
                let mut defaults = options.defaults.clone();
                cell_defaults(&base, &options, &mut defaults);
                options.defaults = defaults;
            }
            options.cell = cell;
        }
        Ok(options)
    }

    /// `plan2cell options --json`, once per binary (again when it is rebuilt).
    async fn cli_options(&self) -> Result<CodegenOptions, HttpError> {
        let binary = self.binary()?;
        let mtime = std::fs::metadata(&binary).and_then(|m| m.modified()).ok();
        if let Some((path, at, options)) = &self.inner.lock().unwrap().options
            && *path == binary
            && *at == mtime
        {
            return Ok(options.clone());
        }
        let unavailable = |err: String| (StatusCode::SERVICE_UNAVAILABLE, err);
        let mut command = Command::new(&binary);
        command.args(["options", "--json"]).stdin(Stdio::null()).kill_on_drop(true);
        let output = tokio::time::timeout(OPTIONS_TIMEOUT, output_when_not_busy(&mut command))
            .await
            .map_err(|_| unavailable(format!("{} options --json timed out", binary.display())))?
            .map_err(|err| unavailable(format!("running {}: {err}", binary.display())))?;
        if !output.status.success() {
            return Err(unavailable(format!(
                "{} options --json failed ({}): {} (an older plan2cell without `options`?)",
                binary.display(),
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        let cli: cli::Options = serde_json::from_slice(&output.stdout)
            .map_err(|err| unavailable(format!("{} options --json: unexpected output: {err}", binary.display())))?;
        let options = cli.into_protocol();
        self.inner.lock().unwrap().options = Some((binary, mtime, options.clone()));
        Ok(options)
    }

    /// Check and start a build: `202` with its id, `409` while another one runs.
    pub async fn start(self: &Arc<Self>, request: BuildRequest) -> Result<BuildAccepted, HttpError> {
        let binary = self.binary()?;
        let options = self.cli_options().await?;
        validate_request(&request, &options).map_err(|err| (StatusCode::BAD_REQUEST, err))?;
        let (cell, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        let (plan_rel, plan_file) = resolve_plan(&root, &request.plan).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?;
        {
            let mut inner = self.inner.lock().unwrap();
            if inner.running {
                let running = inner.status.build_id.clone().unwrap_or_default();
                return Err((StatusCode::CONFLICT, format!("a build is running ({running}); wait for it to finish")));
            }
            inner.running = true;
        }
        // The saved hardware, checked against this plan when it was saved for another one.
        let hardware = match hardware::for_export(&binary, &self.virtual_cell, &root, &plan_rel, &plan_file).await {
            Ok(hardware) => hardware,
            Err((code, message)) => {
                self.inner.lock().unwrap().running = false;
                self.log(format!("Failed to start the build: {message}"));
                return Err((code, message));
            }
        };
        // The cell's plan2cell.toml, on a virtual cell with its KUKA VCs' values where it has none.
        let base = match hardware::export_base(&binary, &self.virtual_cell, &root, &plan_file).await {
            Ok(base) => base,
            Err((code, message)) => {
                self.inner.lock().unwrap().running = false;
                self.log(format!("Failed to start the build: {message}"));
                return Err((code, message));
            }
        };
        let inputs = {
            let (config, plan_file) = (self.config.clone(), plan_file.clone());
            tokio::task::spawn_blocking(move || build_inputs(&config, &plan_file)).await.ok().flatten()
        };
        let job = match prepare(cell, &root, plan_rel, plan_file, request, base, hardware) {
            Ok(job) => Job { inputs, ..job },
            Err((code, message)) => {
                self.inner.lock().unwrap().running = false;
                self.log(format!("Failed to start the build: {message}"));
                return Err((code, message));
            }
        };
        let accepted = BuildAccepted { build_id: job.id.clone() };
        self.log(format!("Building {} ({})", job.plan_rel, job.id));
        self.update(CodegenStatus {
            state: CodegenState::Running,
            build_id: Some(job.id.clone()),
            cell: Some(job.cell.clone()),
            plan: Some(job.plan_rel.clone()),
            message: Some(format!("Building {}…", plan_stem(&job.plan_rel))),
        });
        let this = self.clone();
        tokio::spawn(async move {
            let info = this.run(&binary, &job).await;
            let saved = write_build(&job, &info);
            if saved.is_ok() {
                // The build is what the Virtual Cell deploys and what build.json records: kept in the
                // user's store (S3 mode), the plan's `latest` with it.
                crate::storage::saved_tree(&job.dir);
                if let Some(plan_dir) = job.dir.parent() {
                    crate::storage::saved(plan_dir.join(LATEST));
                }
            }
            let (state, message) = match (&saved, info.ok) {
                (Err(err), _) => (CodegenState::Failed, format!("could not save the build: {err:#}")),
                (Ok(()), true) => (
                    CodegenState::Done,
                    format!(
                        "{} files, {} warning(s)",
                        info.files.len(),
                        info.count(Severity::Warning)
                    ),
                ),
                (Ok(()), false) => (CodegenState::Failed, info.error.clone().unwrap_or_else(|| "build failed".to_owned())),
            };
            this.log(format!("{} {}: {message}", if state == CodegenState::Done { "Built" } else { "Failed" }, job.id));
            this.finish(CodegenStatus {
                state,
                build_id: Some(job.id.clone()),
                cell: Some(job.cell.clone()),
                plan: Some(job.plan_rel.clone()),
                message: Some(message),
            });
        });
        Ok(accepted)
    }

    /// Run the CLI and turn its report into the build's info. Never fails: a failure is a failed build.
    async fn run(&self, binary: &Path, job: &Job) -> BuildInfo {
        let started = std::time::Instant::now();
        let mut info = BuildInfo {
            id: job.id.clone(),
            plan: job.plan_rel.clone(),
            created_s: unix_s(job.created),
            request: job.request.clone(),
            plan_etag: job.plan_etag.clone(),
            base_config_etag: job.base_config_etag.clone(),
            hardware_source: if job.hardware.is_some() { HardwareSource::Saved } else { HardwareSource::Defaults },
            hardware: job.hardware.as_ref().map(|(config, _)| config.clone()),
            hardware_etag: job.hardware.as_ref().map(|(_, etag)| etag.clone()),
            ..Default::default()
        };
        let args = cli_args(&job.plan_file, &job.dir, &job.request);
        let result = self.run_cli(binary, &args).await;
        info.duration_s = started.elapsed().as_secs_f64();
        match result {
            Ok((code, stdout)) => match serde_json::from_slice::<cli::Report>(&stdout) {
                Ok(report) => report.fill(&mut info),
                Err(err) => {
                    info.exit_code = code;
                    info.error = Some(format!(
                        "plan2cell exited ({}) without a report ({err}); see View Logs",
                        code.map_or("killed".to_owned(), |c| c.to_string())
                    ));
                }
            },
            Err(err) => info.error = Some(format!("{err:#}")),
        }
        if info.error.is_some() && info.diagnostics.iter().all(|d| d.severity != Severity::Error) {
            info.diagnostics.insert(
                0,
                Diagnostic {
                    severity: Severity::Error,
                    message: info.error.clone().unwrap_or_default(),
                    source: "backend".to_owned(),
                    ..Default::default()
                },
            );
        }
        let config_bytes = std::fs::metadata(job.dir.join(BUILD_CONFIG)).map_or(0, |m| m.len());
        if !info.files.iter().any(|f| f.path == BUILD_CONFIG) {
            info.files.insert(0, BuildFile { path: BUILD_CONFIG.to_owned(), bytes: config_bytes });
        }
        info
    }

    /// `binary args…` with the timeout: its exit code and stdout; stderr goes to the log.
    /// The CLI runs in a process group of its own, so a timeout (or a descendant still holding its
    /// stderr) kills everything it started, not just the CLI (a wrapper script around it, say).
    async fn run_cli(&self, binary: &Path, args: &[String]) -> anyhow::Result<(Option<i32>, Vec<u8>)> {
        let mut command = Command::new(binary);
        command
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        #[cfg(unix)]
        command.process_group(0);
        let mut child = command.spawn().with_context(|| format!("starting {}", binary.display()))?;
        let group = child.id();
        let stdout = child.stdout.take().context("no stdout")?;
        let stderr = child.stderr.take().context("no stderr")?;
        let sink = self.sink.clone();
        let mut log = tokio::spawn(async move {
            let mut lines = BufReader::new(stderr).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                sink.log(format!("[codegen] {line}"));
            }
        });
        let work = async {
            let mut out = Vec::new();
            stdout.take(MAX_REPORT_BYTES).read_to_end(&mut out).await?;
            let status = child.wait().await?;
            anyhow::Ok((status.code(), out))
        };
        let result = match tokio::time::timeout(self.timeout, work).await {
            Ok(result) => result,
            Err(_) => {
                kill_group(group);
                child.kill().await.ok();
                Err(anyhow::anyhow!("plan2cell timed out after {:.1} s", self.timeout.as_secs_f64()))
            }
        };
        if tokio::time::timeout(LOG_DRAIN, &mut log).await.is_err() {
            // Something the CLI started still holds its stderr: it goes with the group.
            self.log("plan2cell left a process behind; killing its process group");
            kill_group(group);
            if tokio::time::timeout(LOG_DRAIN, &mut log).await.is_err() {
                log.abort();
            }
        }
        result
    }

    /// The active cell's builds, newest first.
    pub fn list(&self) -> BuildList {
        let Some((cell, root)) = cells::active_cell(&self.config) else {
            return BuildList::default();
        };
        let builds_dir = root.join(BUILDS_DIR);
        let resolved = cells::active_resolved(&self.config);
        let mut builds = Vec::new();
        for plan in read_dirs(&builds_dir) {
            let Some(stem) = plan.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
            if !cells::valid_plan_name(&stem) {
                continue;
            }
            let latest = std::fs::read_to_string(plan.join(LATEST)).unwrap_or_default();
            for build in read_dirs(&plan) {
                let Some(stamp) = build.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
                if !valid_stamp(&stamp) {
                    continue;
                }
                let Some(info) = read_info(&build) else { continue };
                builds.push(BuildEntry {
                    id: format!("{stamp}-{stem}"),
                    stale: is_stale(&root, &build, &info, resolved.as_ref()),
                    errors: info.count(Severity::Error) as u64,
                    warnings: info.count(Severity::Warning) as u64,
                    plan: info.plan,
                    created_s: info.created_s,
                    ok: info.ok,
                    latest: latest.trim() == stamp,
                    plan_id: info.summary.as_ref().map(|s| s.plan_id.clone()).filter(|p| !p.is_empty()),
                    hardware_etag: info.hardware_etag,
                });
            }
        }
        builds.sort_by(|a, b| b.id.cmp(&a.id));
        BuildList { cell, builds }
    }

    /// Build `id` of the active cell: its folder (checked to lie in the builds folder).
    fn build_dir(&self, id: &str) -> Result<(PathBuf, PathBuf), HttpError> {
        let (stamp, stem) = parse_id(id).ok_or_else(|| (StatusCode::BAD_REQUEST, format!("invalid build id: {id}")))?;
        let (_, root) = cells::active_cell(&self.config).ok_or_else(|| (StatusCode::NOT_FOUND, "no active cell".to_owned()))?;
        let not_found = || (StatusCode::NOT_FOUND, format!("no build {id} in the active cell"));
        let builds = std::fs::canonicalize(root.join(BUILDS_DIR)).map_err(|_| not_found())?;
        let dir = std::fs::canonicalize(builds.join(stem).join(stamp)).map_err(|_| not_found())?;
        if !dir.starts_with(&builds) || !dir.is_dir() {
            return Err(not_found());
        }
        Ok((root, dir))
    }

    pub fn info(&self, id: &str) -> Result<BuildInfo, HttpError> {
        let (root, dir) = self.build_dir(id)?;
        let mut info = read_info(&dir).ok_or_else(|| (StatusCode::NOT_FOUND, format!("build {id} has no readable {BUILD_INFO}")))?;
        info.stale = is_stale(&root, &dir, &info, cells::active_resolved(&self.config).as_ref());
        Ok(info)
    }

    /// A file of build `id`: its bytes and whether it is text.
    pub fn file(&self, id: &str, path: &str) -> Result<(Vec<u8>, bool), HttpError> {
        let (_, dir) = self.build_dir(id)?;
        let file = contained_file(&dir, path)?;
        let len = std::fs::metadata(&file).map_err(|err| (StatusCode::NOT_FOUND, err.to_string()))?.len();
        if len > MAX_FILE_BYTES {
            return Err((
                StatusCode::PAYLOAD_TOO_LARGE,
                format!("{path} is {len} bytes; download the archive for files over {MAX_FILE_BYTES}"),
            ));
        }
        let bytes = std::fs::read(&file).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
        let text = std::str::from_utf8(&bytes).is_ok();
        Ok((bytes, text))
    }

    /// Build `id` as a zip, every file under `<id>/`.
    pub fn archive(&self, id: &str) -> Result<Vec<u8>, HttpError> {
        let (_, dir) = self.build_dir(id)?;
        zip_dir(&dir, id).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}")))
    }
}

/// The build's folder, with its plan2cell.toml (`base`, [`hardware::export_base`], with `saved`,
/// [`hardware::for_export`]). The config is made first: a cell plan2cell.toml that does not parse is
/// the user's to fix (`422`), and leaves no folder behind.
fn prepare(
    cell: String,
    root: &Path,
    plan_rel: String,
    plan_file: PathBuf,
    request: BuildRequest,
    (base, vc_robots): (Option<String>, Vec<String>),
    saved: Option<(HardwareConfig, String)>,
) -> Result<Job, HttpError> {
    let internal = |err: anyhow::Error| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}"));
    let base_file = root.join(BASE_CONFIG);
    let toml = build_config(base.as_deref(), base_file.is_file(), &vc_robots, &request, saved.as_ref().map(|(config, _)| config))
        .map_err(|err| (StatusCode::UNPROCESSABLE_ENTITY, format!("the cell's {BASE_CONFIG}: {err:#}")))?;
    let stem = plan_stem(&plan_rel).to_owned();
    let created = SystemTime::now();
    let mut stamp = utc_stamp(created);
    let plan_dir = root.join(BUILDS_DIR).join(&stem);
    // Two builds in one millisecond (only in tests, really): the next free stamp.
    while plan_dir.join(&stamp).exists() {
        stamp = bump_stamp(&stamp);
    }
    let dir = plan_dir.join(&stamp);
    std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display())).map_err(internal)?;
    if let Err(err) = std::fs::write(dir.join(BUILD_CONFIG), toml) {
        std::fs::remove_dir_all(&dir).ok();
        return Err(internal(anyhow::Error::new(err).context(format!("writing {}", dir.join(BUILD_CONFIG).display()))));
    }
    let plan_etag = std::fs::metadata(&plan_file).map(|m| cells::plan_etag(&m)).unwrap_or_default();
    let base_config_etag = std::fs::metadata(&base_file).ok().map(|m| cells::plan_etag(&m));
    Ok(Job {
        id: format!("{stamp}-{stem}"),
        cell,
        plan_rel,
        plan_file,
        dir,
        request,
        created,
        plan_etag,
        base_config_etag,
        hardware: saved,
        inputs: None,
    })
}

/// The inputs a build of plan `plan_file` records: the plan's own (`planner_meta.inputs`), else
/// the active cell's as they are now.
fn build_inputs(config: &ConfigStore, plan_file: &Path) -> Option<InputHashes> {
    let plan = std::fs::read(plan_file).ok().and_then(|bytes| serde_json::from_slice::<serde_json::Value>(&bytes).ok());
    plan.as_ref().and_then(cell_resolve::plan_inputs).or_else(|| cells::active_resolved(config).map(|r| r.inputs))
}

/// The inputs build folder `dir` recorded (`build.json`'s `inputs`), if any.
fn read_inputs(dir: &Path) -> Option<InputHashes> {
    let file = std::fs::File::open(dir.join(BUILD_INFO)).ok()?;
    let mut bytes = Vec::new();
    file.take(MAX_INFO_BYTES).read_to_end(&mut bytes).ok()?;
    let value: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    serde_json::from_value(value.get("inputs")?.clone()).ok()
}

/// Factory OS's defaults over plan2cell's (`defaults`): profile vw-ks24 (the VW KS24 robots of the
/// cells it builds for, and the KSS port of the plan2cell KUKA VCs), with what plan2cell makes of it
/// anyway (job number `byte`, no robot simulator projects), if the binary offers it. On a plan2cell
/// virtual cell: profile `sim` ([`VIRTUAL_CELL_PROFILE`]) without robot simulator projects (its
/// robots are the KUKA VCs), with the OPC UA symbols the live view reads.
fn factoryos_defaults(options: &CodegenOptions, defaults: &mut CodegenDefaults, virtual_cell: bool) {
    if virtual_cell && options.profiles.iter().any(|c| c.id == VIRTUAL_CELL_PROFILE && c.implemented) {
        defaults.profile = VIRTUAL_CELL_PROFILE.to_owned();
        defaults.robot_sim = false;
        // The live view (fos-live-bridge) reads the head's state over OPC UA: without the symbol
        // configuration it shows "head PLC offline".
        defaults.opcua_symbols = true;
        return;
    }
    if !options.profiles.iter().any(|c| c.id == DEFAULT_PROFILE && c.implemented) {
        return;
    }
    defaults.profile = DEFAULT_PROFILE.to_owned();
    defaults.robot_sim = false;
    if options.job_numbers.iter().any(|c| c.id == "byte") {
        defaults.job_number = "byte".to_owned();
    }
}

/// The values of the cell's own plan2cell.toml (`base`) for the form's fields, over `defaults`;
/// ones the binary does not offer are left out (a build then reports them). A base that does not
/// parse changes nothing (a build refuses it).
fn cell_defaults(base: &str, options: &CodegenOptions, defaults: &mut CodegenDefaults) {
    use toml::Value;

    let Ok(doc) = base.parse::<toml::Table>() else { return };
    let get = |path: &[&str]| -> Option<&Value> {
        let (first, rest) = path.split_first()?;
        rest.iter().try_fold(doc.get(*first)?, |value, key| value.get(*key))
    };
    let choice = |path: &[&str], choices: &[Choice]| {
        get(path).and_then(Value::as_str).filter(|id| choices.iter().any(|c| c.id == *id)).map(str::to_owned)
    };
    let flag = |path: &[&str]| get(path).and_then(Value::as_bool);
    if let Some(profile) = choice(&["build", "profile"], &options.profiles) {
        defaults.profile = profile;
    }
    if let Some(job_number) = choice(&["plc", "cell", "job_number"], &options.job_numbers) {
        defaults.job_number = job_number;
    }
    if let Some(on) = flag(&["build", "robot_sim"]) {
        defaults.robot_sim = on;
    }
    if let Some(on) = flag(&["build", "st_check"]) {
        defaults.st_check = on && options.st_check_available;
    }
    if let Some(on) = flag(&["build", "strict"]) {
        defaults.strict = on;
    }
    if let Some(on) = flag(&["plc", "opcua", "symbols"]) {
        defaults.opcua_symbols = on;
    }
}

/// The CLI's arguments: never through a shell, and every user value checked before.
fn cli_args(plan: &Path, dir: &Path, request: &BuildRequest) -> Vec<String> {
    let mut args = vec![
        "build".to_owned(),
        plan.display().to_string(),
        "-c".to_owned(),
        dir.join(BUILD_CONFIG).display().to_string(),
        "-o".to_owned(),
        dir.display().to_string(),
        // The folder already holds plan2cell.toml (which --force leaves alone: it only replaces
        // the files of an earlier build).
        "--force".to_owned(),
        "--json".to_owned(),
        "--plc-dialect".to_owned(),
        request.plc_dialect.clone(),
        "--robot-language".to_owned(),
        request.robot_language.clone(),
        // Until the cell's hardware configuration says otherwise, robots have no external axes
        // (each one gets a note in the build).
        "--krl-e-axes-default".to_owned(),
        "none".to_owned(),
    ];
    if !request.st_check {
        args.push("--no-check".to_owned());
    }
    args
}

/// SIGKILL to the process group `group` leads (the CLI's, see [`CodegenService::run_cli`]).
pub(crate) fn kill_group(group: Option<u32>) {
    #[cfg(unix)]
    if let Some(pid) = group.and_then(|pid| i32::try_from(pid).ok()) {
        // SAFETY: plain syscall; a negative pid addresses the process group we created.
        #[expect(unsafe_code)]
        unsafe {
            libc::kill(-pid, libc::SIGKILL);
        }
    }
    #[cfg(not(unix))]
    let _ = group;
}

/// Check the request's choices against what the binary offers (planned ones are let through: the
/// CLI refuses them with its "not implemented" exit code, which the build then shows).
fn validate_request(request: &BuildRequest, options: &CodegenOptions) -> Result<(), String> {
    let known = |what: &str, value: &str, choices: &[Choice]| {
        if choices.iter().any(|c| c.id == value) {
            Ok(())
        } else {
            let ids: Vec<&str> = choices.iter().map(|c| c.id.as_str()).collect();
            Err(format!("unknown {what} {value:?} (one of {})", ids.join(", ")))
        }
    };
    known("profile", &request.profile, &options.profiles)?;
    known("PLC dialect", &request.plc_dialect, &options.plc_dialects)?;
    known("robot language", &request.robot_language, &options.robot_languages)?;
    known("job number mode", &request.job_number, &options.job_numbers)?;
    Ok(())
}

/// The plan `plan` names in the cell at `root`: its path relative to the cell (as
/// [`factoryos_protocol::PlanInfo::path`]) and its file.
pub(crate) fn resolve_plan(root: &Path, plan: &str) -> anyhow::Result<(String, PathBuf)> {
    let name = plan.strip_prefix(cells::PLANS_DIR).map(|rest| rest.trim_start_matches('/')).unwrap_or(plan);
    let name = name.strip_suffix(".json").unwrap_or(name);
    anyhow::ensure!(cells::valid_plan_name(name), "not a plan of the active cell: {plan}");
    let rel = format!("{}/{name}.json", cells::PLANS_DIR);
    let file = root.join(&rel);
    anyhow::ensure!(file.is_file(), "no plan {name} in the active cell");
    Ok((rel, file))
}

fn plan_stem(plan_rel: &str) -> &str {
    let name = plan_rel.rsplit('/').next().unwrap_or(plan_rel);
    name.strip_suffix(".json").unwrap_or(name)
}

/// The build's plan2cell.toml: `base` (the cell's own, `cell_file`: it has one; with the virtual
/// cell's values for `vc_robots`) with the request's options on top, then the cell's saved hardware
/// configuration (`[plc.network]`, `[[plc.robot]]` station / device_name / ip / safety flags).
fn build_config(
    base: Option<&str>,
    cell_file: bool,
    vc_robots: &[String],
    request: &BuildRequest,
    hardware: Option<&HardwareConfig>,
) -> anyhow::Result<String> {
    use toml::{Table, Value};

    let mut doc: Table = match base {
        Some(text) => text.parse().context("not a valid plan2cell.toml")?,
        None => Table::new(),
    };
    fn section<'a>(table: &'a mut Table, key: &str) -> anyhow::Result<&'a mut Table> {
        table
            .entry(key)
            .or_insert_with(|| Value::Table(Table::new()))
            .as_table_mut()
            .with_context(|| format!("[{key}] is not a table"))
    }
    let build = section(&mut doc, "build")?;
    build.insert("profile".into(), Value::String(request.profile.clone()));
    build.insert("plc".into(), Value::String("codesys".into()));
    build.insert("robot_sim".into(), Value::Boolean(request.robot_sim));
    build.insert("krl".into(), Value::Boolean(true));
    build.insert("st_check".into(), Value::Boolean(request.st_check));
    build.insert("strict".into(), Value::Boolean(request.strict));
    let plc = section(&mut doc, "plc")?;
    section(plc, "cell")?.insert("job_number".into(), Value::String(request.job_number.clone()));
    section(plc, "opcua")?.insert("symbols".into(), Value::Boolean(request.opcua_symbols));
    if let Some(hardware) = hardware {
        hardware::apply_to_plan2cell_toml(&mut doc, hardware)?;
    }
    let header = format!(
        "# Written by Physical AI Production Designer (Control Code Export) for {}: {}, the request's options on top{}.\n\
         # Rebuild by hand: plan2cell build <plan> -c plan2cell.toml -o <dir> --plc-dialect {} \
         --robot-language {} --krl-e-axes-default none\n\n",
        request.plan,
        match (cell_file, vc_robots.is_empty()) {
            (true, true) => "the cell's plan2cell/plan2cell.toml".to_owned(),
            (true, false) => format!("the cell's plan2cell/plan2cell.toml with the virtual cell's values where it has none ({})", vc_robots.join(", ")),
            (false, true) => "plan2cell's defaults".to_owned(),
            (false, false) => format!("plan2cell's defaults with the virtual cell's values ({})", vc_robots.join(", ")),
        },
        if hardware.is_some() { format!(", then its Hardware Configuration ({})", hardware::HARDWARE_FILE) } else { String::new() },
        request.plc_dialect,
        request.robot_language,
    );
    Ok(header + &toml::to_string_pretty(&doc)?)
}

/// `build.json`, and the plan's `latest` pointer.
fn write_build(job: &Job, info: &BuildInfo) -> anyhow::Result<()> {
    let mut value = serde_json::to_value(info)?;
    if let (Some(inputs), Some(object)) = (&job.inputs, value.as_object_mut()) {
        object.insert("inputs".to_owned(), serde_json::to_value(inputs)?);
    }
    let json = serde_json::to_vec_pretty(&value)?;
    write_atomic(&job.dir.join(BUILD_INFO), &json)?;
    if let (Some(plan_dir), Some(stamp)) = (job.dir.parent(), job.dir.file_name()) {
        write_atomic(&plan_dir.join(LATEST), stamp.to_string_lossy().as_bytes())?;
    }
    Ok(())
}

fn write_atomic(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, bytes).with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

fn read_info(dir: &Path) -> Option<BuildInfo> {
    let file = std::fs::File::open(dir.join(BUILD_INFO)).ok()?;
    let mut bytes = Vec::new();
    file.take(MAX_INFO_BYTES).read_to_end(&mut bytes).ok()?;
    serde_json::from_slice(&bytes).ok()
}

/// Whether the plan, the cell's plan2cell.toml or its hardware configuration changed since `info`
/// (in build folder `dir`) was built, or the cell's inputs (`cell`: its scene, URDFs, products)
/// differ from those the build recorded.
fn is_stale(root: &Path, dir: &Path, info: &BuildInfo, cell: Option<&ResolvedCell>) -> bool {
    let etag = |rel: &str| std::fs::metadata(root.join(rel)).ok().map(|m| cells::plan_etag(&m));
    etag(&info.plan).as_deref() != Some(info.plan_etag.as_str())
        || etag(BASE_CONFIG) != info.base_config_etag
        || hardware::saved_etag(root) != info.hardware_etag
        || cell.zip(read_inputs(dir)).is_some_and(|(cell, inputs)| cell_resolve::staleness(cell, &inputs).is_some())
}

/// `build.json` of the build in `dir`.
pub fn read_build_info(dir: &Path) -> Option<BuildInfo> {
    read_info(dir)
}

/// Whether the build in `dir` of the cell at `root` is stale (its plan, the cell's plan2cell.toml
/// or Hardware Configuration changed since).
pub fn is_stale_at(root: &Path, dir: &Path, info: &BuildInfo) -> bool {
    is_stale(root, dir, info, None)
}

/// The newest successful build of the cell at `root` (any plan): its id and folder.
pub fn newest_build(root: &Path) -> Option<(String, PathBuf)> {
    newest_build_where(root, |_| true)
}

/// The newest successful build of the cell at `root` whose PLAN_ID is `plan_id` (`16#9573` and
/// `38259` are the same).
pub fn newest_build_of_plan(root: &Path, plan_id: &str) -> Option<(String, PathBuf)> {
    newest_build_where(root, |info| info.summary.as_ref().is_some_and(|s| same_plan_id(&s.plan_id, plan_id)))
}

/// The folder of the successful build `id` (`<stamp>-<plan>`) of the cell at `root`, if it has one
/// (checked to lie in the builds folder).
pub fn export_dir(root: &Path, id: &str) -> Option<PathBuf> {
    let (stamp, stem) = parse_id(id)?;
    let builds = std::fs::canonicalize(root.join(BUILDS_DIR)).ok()?;
    let dir = std::fs::canonicalize(builds.join(stem).join(stamp)).ok()?;
    (dir.starts_with(&builds) && dir.is_dir() && read_info(&dir).is_some_and(|info| info.ok)).then(|| root.join(BUILDS_DIR).join(stem).join(stamp))
}

/// The PLAN_ID of the build in `dir` (its summary).
pub fn build_plan_id(dir: &Path) -> Option<String> {
    read_info(dir)?.summary.map(|s| s.plan_id).filter(|p| !p.is_empty())
}

pub use factoryos_protocol::codegen::{plain_plan_id, same_plan_id};
#[cfg(test)]
pub use factoryos_protocol::codegen::plan_id_number;

/// `plan2cell cell`'s report in a build directory (written by every stage run, dry runs excepted).
pub const CELL_REPORT: &str = "plan2cell-cell-report.json";

/// The export the head was last deployed from, as `p2c-cell` finds it: of the successful builds
/// whose `plan2cell-cell-report.json` has a successful `deploy` stage, the one whose report was
/// written last.
pub fn last_deployed_build(root: &Path) -> Option<(String, PathBuf)> {
    let mut best: Option<(SystemTime, String, PathBuf)> = None;
    for plan in read_dirs(&root.join(BUILDS_DIR)) {
        let Some(stem) = plan.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
        if !cells::valid_plan_name(&stem) {
            continue;
        }
        for build in read_dirs(&plan) {
            let Some(stamp) = build.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
            if !valid_stamp(&stamp) {
                continue;
            }
            let Some(at) = deployed_at(&build) else { continue };
            if best.as_ref().is_some_and(|(t, _, _)| *t >= at) {
                continue;
            }
            if read_info(&build).is_some_and(|info| info.ok) {
                best = Some((at, format!("{stamp}-{stem}"), build));
            }
        }
    }
    best.map(|(_, id, dir)| (id, dir))
}

/// When the cell report in `dir` was written, if it has a successful `deploy` stage.
fn deployed_at(dir: &Path) -> Option<SystemTime> {
    let path = dir.join(CELL_REPORT);
    let modified = std::fs::metadata(&path).ok()?.modified().ok()?;
    let mut bytes = Vec::new();
    std::fs::File::open(&path).ok()?.take(MAX_INFO_BYTES).read_to_end(&mut bytes).ok()?;
    let report: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    let deployed = report["stages"].as_array()?.iter().any(|s| s["stage"] == "deploy" && s["ok"] == true);
    deployed.then_some(modified)
}

fn newest_build_where(root: &Path, keep: impl Fn(&BuildInfo) -> bool) -> Option<(String, PathBuf)> {
    let mut best: Option<(String, PathBuf)> = None;
    for plan in read_dirs(&root.join(BUILDS_DIR)) {
        let Some(stem) = plan.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
        if !cells::valid_plan_name(&stem) {
            continue;
        }
        for build in read_dirs(&plan) {
            let Some(stamp) = build.file_name().and_then(|n| n.to_str()).map(str::to_owned) else { continue };
            if !valid_stamp(&stamp) || best.as_ref().is_some_and(|(id, _)| id[..STAMP_LEN] >= *stamp) {
                continue;
            }
            if read_info(&build).is_some_and(|info| info.ok && keep(&info)) {
                best = Some((format!("{stamp}-{stem}"), build));
            }
        }
    }
    best
}

fn read_dirs(dir: &Path) -> Vec<PathBuf> {
    std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .map(|e| e.path())
        .collect()
}

/// `<stamp>-<plan>` → `(stamp, plan)`, both checked.
fn parse_id(id: &str) -> Option<(&str, &str)> {
    let (stamp, stem) = (id.get(..STAMP_LEN)?, id.get(STAMP_LEN..)?.strip_prefix('-')?);
    (valid_stamp(stamp) && cells::valid_plan_name(stem)).then_some((stamp, stem))
}

/// `YYYYMMDDTHHMMSSmmmZ`.
fn valid_stamp(stamp: &str) -> bool {
    let b = stamp.as_bytes();
    b.len() == STAMP_LEN
        && b[8] == b'T'
        && b[STAMP_LEN - 1] == b'Z'
        && b.iter().enumerate().all(|(i, c)| i == 8 || i == STAMP_LEN - 1 || c.is_ascii_digit())
}

fn zip_dir(dir: &Path, prefix: &str) -> anyhow::Result<Vec<u8>> {
    use std::io::Write as _;
    use zip::write::SimpleFileOptions;

    let mut files = Vec::new();
    let mut total = 0u64;
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for entry in std::fs::read_dir(&d)?.flatten() {
            let kind = entry.file_type()?;
            // No symlinks: nothing outside the build gets in.
            if kind.is_dir() {
                stack.push(entry.path());
            } else if kind.is_file() {
                total += entry.metadata()?.len();
                anyhow::ensure!(total <= MAX_ARCHIVE_BYTES, "the build is larger than {MAX_ARCHIVE_BYTES} bytes");
                files.push(entry.path());
            }
        }
    }
    files.sort();
    let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
    for file in files {
        let rel = file.strip_prefix(dir)?;
        let name: Vec<String> = rel.components().map(|c| c.as_os_str().to_string_lossy().into_owned()).collect();
        if name.last().is_some_and(|n| n.ends_with(".tmp")) {
            continue;
        }
        zip.start_file(format!("{prefix}/{}", name.join("/")), options)?;
        zip.write_all(&std::fs::read(&file)?)?;
    }
    Ok(zip.finish()?.into_inner())
}

fn unix_s(time: SystemTime) -> f64 {
    time.duration_since(SystemTime::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// `YYYYMMDDTHHMMSSmmmZ` in UTC.
fn utc_stamp(time: SystemTime) -> String {
    let since = time.duration_since(SystemTime::UNIX_EPOCH).unwrap_or_default();
    let (days, secs) = (since.as_secs() / 86_400, since.as_secs() % 86_400);
    let (y, m, d) = civil_from_days(days as i64);
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}{:03}Z",
        secs / 3600,
        secs / 60 % 60,
        secs % 60,
        since.subsec_millis()
    )
}

/// The next millisecond's stamp (no calendar carry needed: only used to step past a taken one,
/// and `999` rolls into the seconds field's last digit staying a valid stamp).
fn bump_stamp(stamp: &str) -> String {
    let ms: u32 = stamp[15..18].parse().unwrap_or(0);
    if ms < 999 {
        format!("{}{:03}Z", &stamp[..15], ms + 1)
    } else {
        format!("{}000Z", &stamp[..15])
    }
}

/// Days since 1970-01-01 → (year, month, day) (Howard Hinnant's `civil_from_days`).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

/// The CLI's JSON (snake_case, as plan2cell writes it).
mod cli {
    use super::*;

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Choice {
        id: String,
        label: String,
        implemented: bool,
        description: String,
    }

    #[derive(serde::Deserialize)]
    #[serde(default)]
    pub struct Defaults {
        profile: String,
        plc_dialect: String,
        robot_language: String,
        robot_sim: bool,
        st_check: bool,
        strict: bool,
        job_number: String,
        opcua_symbols: bool,
    }

    impl Default for Defaults {
        fn default() -> Self {
            let d = CodegenDefaults::default();
            Self {
                profile: d.profile,
                plc_dialect: d.plc_dialect,
                robot_language: d.robot_language,
                robot_sim: d.robot_sim,
                st_check: d.st_check,
                strict: d.strict,
                job_number: d.job_number,
                opcua_symbols: d.opcua_symbols,
            }
        }
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Options {
        generator: String,
        plc_dialects: Vec<Choice>,
        robot_languages: Vec<Choice>,
        profiles: Vec<Choice>,
        job_numbers: Vec<Choice>,
        defaults: Defaults,
        st_check_available: bool,
    }

    impl Options {
        pub fn into_protocol(self) -> CodegenOptions {
            let choices = |list: Vec<Choice>| {
                list.into_iter()
                    .map(|c| super::Choice {
                        id: c.id,
                        label: c.label,
                        implemented: c.implemented,
                        description: c.description,
                    })
                    .collect()
            };
            let d = self.defaults;
            CodegenOptions {
                plc_dialects: choices(self.plc_dialects),
                robot_languages: choices(self.robot_languages),
                profiles: choices(self.profiles),
                job_numbers: choices(self.job_numbers),
                defaults: CodegenDefaults {
                    profile: d.profile,
                    plc_dialect: d.plc_dialect,
                    robot_language: d.robot_language,
                    robot_sim: d.robot_sim,
                    job_number: d.job_number,
                    opcua_symbols: d.opcua_symbols,
                    st_check: d.st_check && self.st_check_available,
                    strict: d.strict,
                },
                generator: self.generator,
                st_check_available: self.st_check_available,
                cell: String::new(),
            }
        }
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Diagnostic {
        severity: String,
        code: Option<String>,
        message: String,
        file: Option<String>,
        line: Option<u64>,
        location: Option<String>,
        source: String,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Robot {
        ns: String,
        tag: String,
        ops: u64,
        folgen: Vec<u32>,
        krl_points: u64,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Zone {
        name: String,
        tickets: Vec<String>,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Summary {
        tree_id: String,
        cell_id: String,
        plan_id: String,
        makespan_s: f64,
        st_checked: bool,
        robots: Vec<Robot>,
        zones: Vec<Zone>,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct File {
        path: String,
        bytes: u64,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Report {
        generator: String,
        ok: bool,
        exit_code: i32,
        error: Option<String>,
        summary_line: String,
        summary: Option<Summary>,
        diagnostics: Vec<Diagnostic>,
        files: Vec<File>,
        notes: Vec<String>,
    }

    impl Report {
        pub fn fill(self, info: &mut BuildInfo) {
            info.ok = self.ok;
            info.exit_code = Some(self.exit_code);
            info.error = self.error.filter(|_| !self.ok).or_else(|| (!self.ok).then(|| "build failed".to_owned()));
            info.generator = self.generator;
            info.notes = self.notes;
            info.summary = self.summary.map(|s| BuildSummary {
                line: self.summary_line,
                tree_id: s.tree_id,
                cell_id: s.cell_id,
                plan_id: s.plan_id,
                makespan_s: s.makespan_s,
                st_checked: s.st_checked,
                robots: s
                    .robots
                    .into_iter()
                    .map(|r| RobotSummary {
                        ns: r.ns,
                        tag: r.tag,
                        ops: r.ops,
                        folgen: r.folgen,
                        krl_points: r.krl_points,
                    })
                    .collect(),
                zones: s.zones.into_iter().map(|z| ZoneSummary { name: z.name, tickets: z.tickets }).collect(),
            });
            info.diagnostics = self
                .diagnostics
                .into_iter()
                .map(|d| super::Diagnostic {
                    severity: match d.severity.as_str() {
                        "error" => Severity::Error,
                        "warning" => Severity::Warning,
                        _ => Severity::Info,
                    },
                    code: d.code,
                    message: d.message,
                    file: d.file,
                    line: d.line,
                    location: d.location,
                    source: d.source,
                })
                .collect();
            info.files = self.files.into_iter().map(|f| BuildFile { path: f.path, bytes: f.bytes }).collect();
        }
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::CODEGEN_OPTIONS, get(options))
        .route(api::CODEGEN_BUILD, post(build))
        .route(api::CODEGEN_BUILDS, get(builds))
        .route(api::CODEGEN_BUILD_INFO, get(build_info))
        // Generated sources are text: gzipped when the client asks.
        .route(api::CODEGEN_BUILD_FILE, get(build_file).layer(tower_http::compression::CompressionLayer::new().gzip(true)))
        .route(api::CODEGEN_BUILD_ARCHIVE, get(build_archive))
}

async fn options(State(state): State<AppState>) -> Result<Json<CodegenOptions>, HttpError> {
    state.codegen.options().await.map(Json)
}

async fn build(State(state): State<AppState>, Json(request): Json<BuildRequest>) -> Result<(StatusCode, Json<BuildAccepted>), HttpError> {
    state.codegen.start(request).await.map(|accepted| (StatusCode::ACCEPTED, Json(accepted)))
}

async fn blocking<T: Send + 'static>(work: impl FnOnce() -> Result<T, HttpError> + Send + 'static) -> Result<T, HttpError> {
    tokio::task::spawn_blocking(work)
        .await
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
}

async fn builds(State(state): State<AppState>) -> Result<Json<BuildList>, HttpError> {
    let codegen = state.codegen.clone();
    blocking(move || Ok(codegen.list())).await.map(Json)
}

async fn build_info(State(state): State<AppState>, UrlPath(id): UrlPath<String>) -> Result<Json<BuildInfo>, HttpError> {
    let codegen = state.codegen.clone();
    blocking(move || codegen.info(&id)).await.map(Json)
}

async fn build_file(State(state): State<AppState>, UrlPath((id, path)): UrlPath<(String, String)>) -> Result<Response, HttpError> {
    let codegen = state.codegen.clone();
    let (bytes, text) = blocking(move || codegen.file(&id, &path)).await?;
    let kind = if text { "text/plain; charset=utf-8" } else { "application/octet-stream" };
    Ok((
        [(header::CONTENT_TYPE, kind), (header::X_CONTENT_TYPE_OPTIONS, "nosniff"), (header::CACHE_CONTROL, "no-cache")],
        bytes,
    )
        .into_response())
}

async fn build_archive(State(state): State<AppState>, UrlPath(id): UrlPath<String>) -> Result<Response, HttpError> {
    let codegen = state.codegen.clone();
    let name = id.clone();
    let bytes = blocking(move || codegen.archive(&id)).await?;
    // The id is checked (`[0-9TZ]`, `-` and a plan name), so it is safe in the header.
    let disposition = format!("attachment; filename=\"{name}.zip\"");
    Ok((
        [(header::CONTENT_TYPE, "application/zip".to_owned()), (header::CONTENT_DISPOSITION, disposition)],
        bytes,
    )
        .into_response())
}

/// `command.output()`, retried briefly while its binary is `Text file busy` (ETXTBSY): a binary
/// just written (plan2cell rebuilt, a test's fake) can still be open for writing in a process
/// forked meanwhile, which Linux refuses to execute until that copy of the descriptor is closed.
pub(crate) async fn output_when_not_busy(command: &mut Command) -> std::io::Result<std::process::Output> {
    let mut tries = 0;
    loop {
        match command.output().await {
            Err(err) if err.kind() == std::io::ErrorKind::ExecutableFileBusy && tries < 20 => {
                tries += 1;
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            result => return result,
        }
    }
}

/// `command.spawn()`, retried briefly while its binary is `Text file busy` (ETXTBSY), like
/// [`output_when_not_busy`].
pub(crate) async fn spawn_when_not_busy(command: &mut Command) -> std::io::Result<tokio::process::Child> {
    let mut tries = 0;
    loop {
        match command.spawn() {
            Err(err) if err.kind() == std::io::ErrorKind::ExecutableFileBusy && tries < 20 => {
                tries += 1;
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            result => return result,
        }
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    /// A stand-in for the plan2cell CLI: `options --json`, and `build` that writes two files and a
    /// report, fails like the real one for planned dialects (exit 5), plans containing `FAIL`
    /// (exit 3) and `TRUST` (exit 8: the ST check rejected the code, nothing written), and takes
    /// 2 s for plans containing `SLOW`. `HANG`: it never ends (`sleep 30` in a child); `LINGER`: it
    /// ends, but leaves a child that holds its stderr. Like the real one, only a failed ST check
    /// has diagnostics with a file.
    const FAKE: &str = r#"#!/bin/sh
cmd="$1"; shift
if [ "$cmd" = "hardware" ]; then
cat <<'JSON'
{"schema":"plan2cell/hardware@1","ok":true,
 "network":{"headIp":"192.168.100.10","netmask":"255.255.255.0","prefix":24,"gateway":"0.0.0.0","controllerStation":"controller"},
 "networkDefaults":{"headIp":"192.168.100.10","netmask":"255.255.255.0","gateway":"0.0.0.0","controllerStation":"controller"},
 "robots":[{"ns":"robot1","tag":"R1","station":"kuka-robi1","deviceName":"kuka_robi1","ip":"192.168.100.11","model":"KR240","external":true},
           {"ns":"robot2","tag":"R2","station":"kuka-robi2","deviceName":"kuka_robi2","ip":"192.168.100.12","model":"KR240","external":true}],
 "kukaVc":{"kliPrefix":24,"kliGw":null,"controllers":[]},"issues":[]}
JSON
exit 0
fi
if [ "$cmd" = "options" ]; then
cat <<'JSON'
{"schema":"plan2cell/options@1","generator":"plan2cell fake","st_check_available":true,
 "plc_dialects":[{"id":"codesys","label":"CODESYS V3.5","implemented":true},{"id":"twincat3","label":"Beckhoff TwinCAT 3","implemented":false}],
 "robot_languages":[{"id":"krl","label":"KUKA KRL","implemented":true},{"id":"rapid","label":"ABB RAPID","implemented":false}],
 "profiles":[{"id":"sim","label":"Simulation","implemented":true},{"id":"vw-ks24","label":"VW KS24","implemented":true}],
 "job_numbers":[{"id":"word_le","label":"WORD","implemented":true},{"id":"byte","label":"BYTE","implemented":true}],
 "defaults":{"profile":"sim","plc_dialect":"codesys","robot_language":"krl","robot_sim":true,"krl":true,"st_check":true,"strict":false,"job_number":"word_le","opcua_symbols":false}}
JSON
exit 0
fi
plan="$1"; shift
out=""; dialect="codesys"
while [ $# -gt 0 ]; do
  case "$1" in
    -o) out="$2"; shift;;
    --plc-dialect) dialect="$2"; shift;;
  esac
  shift
done
echo "fake: building $plan" >&2
if [ "$dialect" != "codesys" ]; then
  echo '{"ok":false,"exit_code":5,"error":"flavour twincat3 is not implemented yet","diagnostics":[{"severity":"error","message":"flavour twincat3 is not implemented yet","source":"plc"}],"files":[],"notes":[],"written":false}'
  exit 5
fi
if grep -q FAIL "$plan"; then
  echo '{"ok":false,"exit_code":3,"error":"config: robot robot1: declare e_axes","diagnostics":[{"severity":"error","message":"robot robot1: declare e_axes","source":"plan"}],"files":[],"notes":[],"written":false}'
  exit 3
fi
if grep -q TRUST "$plan"; then
  echo '{"ok":false,"exit_code":8,"error":"generated ST rejected","diagnostics":[{"severity":"error","code":"E101","message":"undefined","file":"plc/codesys/head/POU/FB_Cell_Body.st","line":120,"source":"st"}],"files":[],"notes":[],"written":false}'
  exit 8
fi
if grep -q SLOW "$plan"; then sleep 2; fi
if grep -q HANG "$plan"; then sleep 30; fi
if grep -q LINGER "$plan"; then (sleep 30 >/dev/null &); fi
mkdir -p "$out/plc/codesys"
printf 'PROGRAM PLC_PRG\nEND_PROGRAM\n' > "$out/plc/codesys/PLC_PRG.st"
printf '# Build\n' > "$out/BUILD.md"
cat <<'JSON'
{"ok":true,"exit_code":0,"generator":"plan2cell fake","summary_line":"t (PLAN_ID 16#0001)","summary":{"tree_id":"t","cell_id":"c","plan_id":"16#0001","makespan_s":1.5,"st_checked":true,"robots":[{"ns":"robot1","tag":"R1","ops":2,"folgen":[11,125],"krl_points":3}],"zones":[{"name":"IL01","tickets":["robot1.W01"]}]},
 "diagnostics":[{"severity":"warning","code":"W003","message":"approximated","location":"/root/0","source":"plc"},{"severity":"info","message":"robot1: 2 ops","source":"krl"}],
 "files":[{"path":"plc/codesys/PLC_PRG.st","bytes":28},{"path":"BUILD.md","bytes":8}],"notes":["robot1: e_axes = \"none\" assumed"],"written":true}
JSON
"#;

    #[derive(Default)]
    struct Recorder {
        statuses: Mutex<Vec<CodegenStatus>>,
        logs: Mutex<Vec<String>>,
    }

    impl Sink for Recorder {
        fn status(&self, status: &CodegenStatus) {
            self.statuses.lock().unwrap().push(status.clone());
        }

        fn log(&self, line: String) {
            self.logs.lock().unwrap().push(line);
        }
    }

    struct Fixture {
        root: PathBuf,
        cell: PathBuf,
        service: Arc<CodegenService>,
        recorder: Arc<Recorder>,
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.root).ok();
        }
    }

    fn fixture(name: &str) -> Fixture {
        use std::os::unix::fs::PermissionsExt as _;

        let root = std::env::temp_dir().join(format!("factoryos-codegen-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cells/weld");
        let plans = cell.join(cells::PLANS_DIR);
        std::fs::create_dir_all(&plans).unwrap();
        for (plan, text) in
            [("bt_ok", "{}"), ("bt_fail", "FAIL"), ("bt_slow", "SLOW"), ("bt_trust", "TRUST"), ("bt_hang", "HANG"), ("bt_linger", "LINGER")]
        {
            std::fs::write(plans.join(format!("{plan}.json")), text).unwrap();
        }
        let fake = root.join("plan2cell");
        std::fs::write(&fake, FAKE).unwrap();
        std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o755)).unwrap();
        let store = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let mut config = store.get();
        config.cell_dir = cell.display().to_string();
        store.set(config);
        let recorder = Arc::new(Recorder::default());
        let service = CodegenService::with_sink(recorder.clone(), store, Some(fake));
        Fixture { root, cell, service, recorder }
    }

    fn request(plan: &str) -> BuildRequest {
        BuildRequest::new(plan, &CodegenDefaults::default())
    }

    async fn finished(service: &CodegenService) -> CodegenStatus {
        for _ in 0..200 {
            let status = service.status();
            if !service.inner.lock().unwrap().running && status.state != CodegenState::Running {
                return status;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!("the build did not finish");
    }

    #[tokio::test]
    async fn a_build_runs_the_cli_and_is_stored() {
        let f = fixture("ok");
        let options = f.service.options().await.unwrap();
        assert_eq!(options.plc_dialects.len(), 2);
        assert!(!options.plc_dialects[1].implemented);
        // Factory OS's defaults over plan2cell's (profile sim, word_le, robot simulators).
        let d = &options.defaults;
        assert_eq!((d.profile.as_str(), d.job_number.as_str(), d.robot_sim), ("vw-ks24", "byte", false));

        let accepted = f.service.start(request("agentic_planner/plans/bt_ok.json")).await.unwrap();
        assert!(accepted.build_id.ends_with("-bt_ok"), "{}", accepted.build_id);
        let status = finished(&f.service).await;
        assert_eq!(status.state, CodegenState::Done, "{status:?}");
        assert_eq!(status.build_id.as_deref(), Some(accepted.build_id.as_str()));
        assert_eq!(status.cell.as_deref(), Some("weld"));
        let statuses = f.recorder.statuses.lock().unwrap().clone();
        assert_eq!(statuses.first().map(|s| s.state), Some(CodegenState::Running));
        assert!(f.recorder.logs.lock().unwrap().iter().any(|l| l.contains("fake: building")), "stderr goes to the log");

        let info = f.service.info(&accepted.build_id).unwrap();
        assert!(info.ok && !info.stale);
        assert_eq!(info.plan, "agentic_planner/plans/bt_ok.json");
        assert_eq!(info.summary.as_ref().unwrap().plan_id, "16#0001");
        assert_eq!(info.count(Severity::Warning), 1);
        assert_eq!(info.count(Severity::Error), 0);
        assert_eq!(info.files[0].path, "plan2cell.toml", "the config it was built with is part of the build");
        let stamp = &accepted.build_id[..STAMP_LEN];
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(stamp);
        let toml = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap();
        assert!(toml.contains("job_number = \"word_le\"") && toml.contains("profile = \"sim\""), "{toml}");
        assert_eq!(std::fs::read_to_string(f.cell.join(BUILDS_DIR).join("bt_ok/latest")).unwrap(), stamp);

        let list = f.service.list();
        assert_eq!(list.cell, "weld");
        assert_eq!(list.builds.len(), 1);
        assert!(list.builds[0].latest && list.builds[0].ok);
        let (bytes, text) = f.service.file(&accepted.build_id, "plc/codesys/PLC_PRG.st").unwrap();
        assert!(text && bytes.starts_with(b"PROGRAM"));
        let zip = f.service.archive(&accepted.build_id).unwrap();
        let mut archive = zip::ZipArchive::new(std::io::Cursor::new(zip)).unwrap();
        let names: Vec<String> = archive.file_names().map(|n| n.unwrap().into_owned()).collect();
        assert!(names.contains(&format!("{}/plc/codesys/PLC_PRG.st", accepted.build_id)), "{names:?}");
        assert!(names.contains(&format!("{}/build.json", accepted.build_id)), "{names:?}");
        assert!(archive.by_name(&format!("{}/BUILD.md", accepted.build_id)).is_ok());

        // The plan changes: the build is stale.
        std::fs::write(f.cell.join(cells::PLANS_DIR).join("bt_ok.json"), "{\"v\": 2}").unwrap();
        assert!(f.service.info(&accepted.build_id).unwrap().stale);
        assert!(f.service.list().builds[0].stale);
    }

    #[tokio::test]
    async fn a_saved_build_is_written_through() {
        use crate::storage::sync::tests_support::{eventually, storage_at};
        use factoryos_protocol::storage::Scope;
        let f = fixture("through");
        let (s3, storage) = storage_at(&f.root);
        let _registered = crate::storage::register(&storage);
        let accepted = f.service.start(request("agentic_planner/plans/bt_ok.json")).await.unwrap();
        assert_eq!(finished(&f.service).await.state, CodegenState::Done);
        let stamp = &accepted.build_id[..STAMP_LEN];
        let build = format!("cells/weld/{BUILDS_DIR}/bt_ok/{stamp}");
        let (info, latest) = (format!("{build}/{BUILD_INFO}"), format!("cells/weld/{BUILDS_DIR}/bt_ok/{LATEST}"));
        let code = format!("{build}/plc/codesys/PLC_PRG.st");
        eventually("the build", || [&info, &latest, &code].iter().all(|k| s3.bytes(Scope::User, k).is_some())).await;
        assert_eq!(s3.bytes(Scope::User, &latest).unwrap(), stamp.as_bytes());
        assert!(s3.bytes(Scope::User, &format!("{build}/{BUILD_CONFIG}")).is_some(), "the config it was built with");
    }

    /// A reference cell: the build records the inputs its plan was made from, and is stale once
    /// the cell's scene (or a robot, a product) changed.
    #[tokio::test]
    async fn a_reference_cells_build_goes_stale_with_its_inputs() {
        let f = fixture("inputs");
        let assets = f.root.join("assets");
        std::fs::create_dir_all(assets.join("scenes/weld")).unwrap();
        std::fs::create_dir_all(assets.join("urdf-dressed-robots/kr")).unwrap();
        std::fs::write(assets.join("urdf-dressed-robots/kr/kr.urdf"), "<robot name='kr'/>").unwrap();
        let scene = assets.join("scenes/weld/scene.yaml");
        let text = "name: weld\nrobots:\n- robot_id: r1\n  dressed_urdf: ../../urdf-dressed-robots/kr/kr.urdf\n  position: {x: 0, y: 0, z: 0}\n";
        std::fs::write(&scene, text).unwrap();
        std::fs::write(f.cell.join("factoryos-cell.json"), r#"{"format":2,"kind":"reference","fitted_scene":"weld"}"#).unwrap();
        let mut config = f.service.config.get();
        config.asset_root = assets.display().to_string();
        f.service.config.set(config);
        let resolved = cells::active_resolved(&f.service.config).unwrap();
        assert_eq!(resolved.kind, cell_resolve::CellKind::Reference);

        // The plan records what it was planned from: the build takes that.
        let plan = serde_json::json!({ "behaviour_tree": {}, "planner_meta": { "inputs": resolved.inputs } });
        std::fs::write(f.cell.join(cells::PLANS_DIR).join("bt_ok.json"), plan.to_string()).unwrap();
        let accepted = f.service.start(request("agentic_planner/plans/bt_ok.json")).await.unwrap();
        assert_eq!(finished(&f.service).await.state, CodegenState::Done);
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        assert_eq!(read_inputs(&dir), Some(resolved.inputs.clone()));
        assert!(!f.service.info(&accepted.build_id).unwrap().stale);
        assert!(!f.service.list().builds[0].stale);

        // The robot is re-dressed: stale; nothing was rebuilt or deleted.
        std::fs::write(assets.join("urdf-dressed-robots/kr/kr.urdf"), "<robot name='kr' re-dressed='1'/>").unwrap();
        assert!(f.service.info(&accepted.build_id).unwrap().stale);
        assert!(f.service.list().builds[0].stale);
        assert!(dir.join(BUILD_CONFIG).is_file());

        // A plan without recorded inputs: the cell's inputs at build time.
        std::fs::write(f.cell.join(cells::PLANS_DIR).join("bt_ok.json"), "{}").unwrap();
        let second = f.service.start(request("agentic_planner/plans/bt_ok.json")).await.unwrap();
        assert_eq!(finished(&f.service).await.state, CodegenState::Done);
        assert!(!f.service.info(&second.build_id).unwrap().stale);
        std::fs::write(&scene, text.replace("x: 0", "x: 1")).unwrap();
        assert!(f.service.info(&second.build_id).unwrap().stale, "the scene was saved");
    }

    #[tokio::test]
    async fn failures_are_failed_builds() {
        let f = fixture("fail");
        let accepted = f.service.start(request("bt_fail")).await.unwrap();
        let status = finished(&f.service).await;
        assert_eq!(status.state, CodegenState::Failed);
        assert!(status.message.unwrap().contains("e_axes"));
        let info = f.service.info(&accepted.build_id).unwrap();
        assert!(!info.ok);
        assert_eq!(info.exit_code, Some(3));
        assert_eq!(info.diagnostics[0].severity, Severity::Error);

        // The ST check rejected the code: diagnostics with files, but none of them written.
        let accepted = f.service.start(request("bt_trust")).await.unwrap();
        assert_eq!(finished(&f.service).await.state, CodegenState::Failed);
        let info = f.service.info(&accepted.build_id).unwrap();
        assert_eq!(info.exit_code, Some(8));
        assert_eq!(info.diagnostics[0].file.as_deref(), Some("plc/codesys/head/POU/FB_Cell_Body.st"));
        assert_eq!(info.files.iter().map(|f| f.path.as_str()).collect::<Vec<_>>(), [BUILD_CONFIG]);

        // A planned dialect: the CLI's "not implemented" (5), reported as such.
        let accepted = f.service.start(BuildRequest { plc_dialect: "twincat3".to_owned(), ..request("bt_ok") }).await.unwrap();
        assert_eq!(finished(&f.service).await.state, CodegenState::Failed);
        let info = f.service.info(&accepted.build_id).unwrap();
        assert_eq!(info.exit_code, Some(5));
        assert!(info.error.unwrap().contains("not implemented"));

        // Unknown choices and plans never reach the CLI.
        let err = f.service.start(BuildRequest { plc_dialect: "s7; rm -rf /".to_owned(), ..request("bt_ok") }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::BAD_REQUEST);
        let err = f.service.start(BuildRequest { job_number: "dword".to_owned(), ..request("bt_ok") }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::BAD_REQUEST);
        for plan in ["../../etc/passwd", "agentic_planner/plans/../x", "missing", "a/b"] {
            assert_eq!(f.service.start(request(plan)).await.unwrap_err().0, StatusCode::NOT_FOUND, "{plan}");
        }

        // No binary: 503 on everything that needs it.
        let store = Arc::new(ConfigStore::open(f.root.join("other.config.json")));
        let none = CodegenService::with_sink(f.recorder.clone(), store.clone(), None);
        assert_eq!(none.options().await.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        let missing = CodegenService::with_sink(f.recorder.clone(), store, Some(f.root.join("nope")));
        let err = missing.options().await.unwrap_err();
        assert_eq!(err.0, StatusCode::SERVICE_UNAVAILABLE);
        assert!(err.1.contains("does not exist"), "{}", err.1);
    }

    #[tokio::test]
    async fn one_build_at_a_time() {
        let f = fixture("busy");
        let first = f.service.start(request("bt_slow")).await.unwrap();
        let err = f.service.start(request("bt_ok")).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        assert!(err.1.contains(&first.build_id));
        assert_eq!(finished(&f.service).await.state, CodegenState::Done);
        assert!(f.service.start(request("bt_ok")).await.is_ok(), "free again");
        finished(&f.service).await;
    }

    /// A build started the moment the previous one ends: its Running is never replaced by the
    /// previous build's Done, and the 409s name the build that runs.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn a_build_right_after_another_keeps_its_status() {
        let f = fixture("race");
        for _ in 0..15 {
            let first = f.service.start(request("bt_ok")).await.unwrap();
            let second = loop {
                match f.service.start(request("bt_ok")).await {
                    Ok(accepted) => break accepted,
                    Err((StatusCode::CONFLICT, message)) => {
                        assert!(message.contains(&first.build_id), "{message}");
                        tokio::task::yield_now().await;
                    }
                    Err(err) => panic!("{err:?}"),
                }
            };
            let status = finished(&f.service).await;
            assert_eq!(status.build_id.as_deref(), Some(second.build_id.as_str()), "{status:?}");
            // Once the second build's Running is out, nothing about the first one follows.
            let statuses = f.recorder.statuses.lock().unwrap().clone();
            let running = statuses
                .iter()
                .position(|s| s.state == CodegenState::Running && s.build_id.as_deref() == Some(second.build_id.as_str()))
                .unwrap();
            for later in &statuses[running..] {
                assert_eq!(later.build_id.as_deref(), Some(second.build_id.as_str()), "{statuses:#?}");
            }
            assert_eq!(statuses.last(), Some(&status), "published in the order stored");
        }
    }

    fn with_timeout(f: &Fixture, timeout: Duration) -> Arc<CodegenService> {
        Arc::new(CodegenService {
            sink: f.recorder.clone(),
            config: f.service.config.clone(),
            binary: Mutex::new(f.service.binary.lock().unwrap().clone()),
            lookup: None,
            timeout,
            virtual_cell: hardware::VirtualCell::none(),
            inner: Mutex::default(),
        })
    }

    #[tokio::test]
    async fn a_hanging_cli_times_out() {
        let f = fixture("timeout");
        let service = with_timeout(&f, Duration::from_millis(300));
        // The fake's `sleep 30` holds its stderr: it is killed with the fake's process group.
        let started = std::time::Instant::now();
        let accepted = service.start(request("bt_hang")).await.unwrap();
        let status = finished(&service).await;
        assert!(started.elapsed() < Duration::from_secs(2), "{:?}", started.elapsed());
        assert_eq!(status.state, CodegenState::Failed);
        assert!(status.message.unwrap().contains("timed out"));
        assert!(!service.info(&accepted.build_id).unwrap().ok);
        assert!(service.start(request("bt_ok")).await.is_ok(), "free again");
        finished(&service).await;
    }

    #[tokio::test]
    async fn a_process_left_behind_does_not_hold_the_build() {
        let f = fixture("linger");
        let started = std::time::Instant::now();
        f.service.start(request("bt_linger")).await.unwrap();
        let status = finished(&f.service).await;
        assert_eq!(status.state, CodegenState::Done, "{status:?}");
        assert!(started.elapsed() < LOG_DRAIN * 2, "{:?}", started.elapsed());
        assert!(f.recorder.logs.lock().unwrap().iter().any(|l| l.contains("left a process behind")));
    }

    #[tokio::test]
    async fn a_virtual_cell_exports_profile_sim() {
        let f = fixture("virtual");
        let config = f.root.join("cell.toml");
        std::fs::write(&config, "").unwrap();
        let binary = f.service.binary.lock().unwrap().clone();
        let service = CodegenService::with_sink(f.recorder.clone(), f.service.config.clone(), binary)
            .on_virtual_cell(hardware::VirtualCell::at(config));
        let d = service.options().await.unwrap().defaults;
        assert_eq!((d.profile.as_str(), d.robot_sim, d.opcua_symbols), ("sim", false, true));
        // The cell's own profile still wins.
        let base = f.cell.join(BASE_CONFIG);
        std::fs::create_dir_all(base.parent().unwrap()).unwrap();
        std::fs::write(&base, "[build]\nprofile = \"vw-ks24\"\n").unwrap();
        assert_eq!(service.options().await.unwrap().defaults.profile, "vw-ks24");
    }

    #[tokio::test]
    async fn the_cells_config_is_the_forms_default_and_must_parse() {
        let f = fixture("base");
        let base = f.cell.join(BASE_CONFIG);
        std::fs::create_dir_all(base.parent().unwrap()).unwrap();
        std::fs::write(
            &base,
            "[build]\nprofile = \"vw-ks24\"\nstrict = true\nrobot_sim = false\n[plc.cell]\njob_number = \"byte\"\nprod_folge = 12\n[plc.opcua]\nsymbols = true\n",
        )
        .unwrap();
        let options = f.service.options().await.unwrap();
        assert_eq!(options.cell, "weld");
        let d = &options.defaults;
        assert_eq!((d.profile.as_str(), d.job_number.as_str()), ("vw-ks24", "byte"));
        assert!(d.strict && d.opcua_symbols && !d.robot_sim && d.st_check);

        // The form left alone: the cell's values survive into the build's config.
        let accepted = f.service.start(BuildRequest::new("bt_ok", &options.defaults)).await.unwrap();
        finished(&f.service).await;
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        let toml: toml::Table = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap().parse().unwrap();
        assert_eq!(toml["plc"]["cell"]["job_number"].as_str(), Some("byte"));
        assert_eq!(toml["plc"]["cell"]["prod_folge"].as_integer(), Some(12));
        assert_eq!(toml["plc"]["opcua"]["symbols"].as_bool(), Some(true));
        assert_eq!(toml["build"]["strict"].as_bool(), Some(true));
        assert_eq!(toml["build"]["profile"].as_str(), Some("vw-ks24"));

        // Values the binary does not offer stay the defaults.
        std::fs::write(&base, "[plc.cell]\njob_number = \"dword\"\n").unwrap();
        assert_eq!(f.service.options().await.unwrap().defaults.job_number, "byte");
        // The cell's own profile wins.
        std::fs::write(&base, "[build]\nprofile = \"sim\"\n").unwrap();
        assert_eq!(f.service.options().await.unwrap().defaults.profile, "sim");

        // A broken one: the user's to fix (422), and no build folder is left behind.
        let before = read_dirs(&f.cell.join(BUILDS_DIR).join("bt_ok")).len();
        for broken in ["[build", "build = 1", "[plc]\nopcua = 3\n"] {
            std::fs::write(&base, broken).unwrap();
            assert!(f.service.options().await.is_ok(), "the options still load");
            let err = f.service.start(request("bt_ok")).await.unwrap_err();
            assert_eq!(err.0, StatusCode::UNPROCESSABLE_ENTITY, "{broken}: {err:?}");
        }
        assert_eq!(read_dirs(&f.cell.join(BUILDS_DIR).join("bt_ok")).len(), before);
        assert!(f.service.start(request("bt_slow")).await.is_err(), "still refused");
        assert!(!f.service.inner.lock().unwrap().running, "and free");
    }

    #[tokio::test]
    async fn the_saved_hardware_goes_into_every_build() {
        use factoryos_protocol::hardware::{NetworkConfig, RobotHardware};

        let f = fixture("hardware");
        let accepted = f.service.start(request("bt_ok")).await.unwrap();
        finished(&f.service).await;
        let info = f.service.info(&accepted.build_id).unwrap();
        assert_eq!(info.hardware_source, HardwareSource::Defaults);
        assert!(info.hardware.is_none() && !info.stale);

        let config = HardwareConfig {
            network: NetworkConfig {
                head_ip: "192.168.100.20".into(),
                netmask: "255.255.255.0".into(),
                gateway: String::new(),
                controller_station: "head".into(),
            },
            robots: vec![RobotHardware {
                ns: "robot1".into(),
                station: "weld-r1".into(),
                device_name: "weld_r1".into(),
                ip: "192.168.100.21".into(),
                ..Default::default()
            }],
            kuka_vc: Vec::new(),
        };
        let saved =
            hardware::Saved { schema: "factoryos/hardware@1".into(), saved_s: 1.0, plan: format!("{}/bt_ok.json", cells::PLANS_DIR), config };
        let file = f.cell.join(hardware::HARDWARE_FILE);
        hardware::write_atomic(&file, &serde_json::to_vec(&saved).unwrap()).unwrap();
        assert!(f.service.info(&accepted.build_id).unwrap().stale, "a build before the hardware was saved");

        let accepted = f.service.start(request("bt_ok")).await.unwrap();
        finished(&f.service).await;
        let info = f.service.info(&accepted.build_id).unwrap();
        assert_eq!(info.hardware_source, HardwareSource::Saved);
        assert_eq!(info.hardware.as_ref().unwrap().robots[0].station, "weld-r1");
        assert!(info.hardware_etag.is_some() && !info.stale);
        let entry = f.service.list().builds.into_iter().find(|b| b.id == accepted.build_id).unwrap();
        assert_eq!(entry.hardware_etag, info.hardware_etag, "the list names the hardware it was built with");
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        let text = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap();
        assert!(text.contains("then its Hardware Configuration"), "{text}");
        let toml: toml::Table = text.parse().unwrap();
        assert_eq!(toml["plc"]["network"]["head_ip"].as_str(), Some("192.168.100.20"));
        assert_eq!(toml["plc"]["network"]["gateway"].as_str(), Some("0.0.0.0"));
        assert_eq!(toml["plc"]["robot"][0]["ip"].as_str(), Some("192.168.100.21"));
        assert_eq!(newest_build(&f.cell).map(|(id, _)| id), Some(accepted.build_id.clone()));

        // Saved again (other values): stale.
        let mut saved = saved;
        saved.config.robots[0].ip = "192.168.100.121".into();
        hardware::write_atomic(&file, &serde_json::to_vec(&saved).unwrap()).unwrap();
        assert!(f.service.info(&accepted.build_id).unwrap().stale);

        // Saved for another plan: over this plan's defaults and checked again. robot1 at robot2's
        // default IP clashes: refused before the build, pointing at the Hardware Configuration.
        let mut other = saved.clone();
        other.plan = format!("{}/bt_other.json", cells::PLANS_DIR);
        other.config.robots[0].ip = "192.168.100.12".into();
        hardware::write_atomic(&file, &serde_json::to_vec(&other).unwrap()).unwrap();
        let before = read_dirs(&f.cell.join(BUILDS_DIR).join("bt_ok")).len();
        let err = f.service.start(request("bt_ok")).await.unwrap_err();
        assert_eq!(err.0, StatusCode::UNPROCESSABLE_ENTITY, "{}", err.1);
        assert!(err.1.contains("Hardware Config") && err.1.contains("bt_other"), "{}", err.1);
        assert_eq!(read_dirs(&f.cell.join(BUILDS_DIR).join("bt_ok")).len(), before, "no build folder");
        assert!(!f.service.inner.lock().unwrap().running, "and free");
        // A robot this plan does not have is left out; this plan's own robots keep their defaults.
        other.config.robots[0].ip = "192.168.100.21".into();
        other.config.robots.push(RobotHardware {
            ns: "robot9".into(),
            station: "r9".into(),
            device_name: "r9".into(),
            ip: "192.168.100.99".into(),
            ..Default::default()
        });
        hardware::write_atomic(&file, &serde_json::to_vec(&other).unwrap()).unwrap();
        let accepted = f.service.start(request("bt_ok")).await.unwrap();
        finished(&f.service).await;
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        let toml: toml::Table = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap().parse().unwrap();
        let robots = toml["plc"]["robot"].as_array().unwrap();
        let ns: Vec<&str> = robots.iter().map(|r| r["ns"].as_str().unwrap()).collect();
        assert_eq!(ns, ["robot1", "robot2"], "{toml:#?}");
        assert_eq!(robots[0]["ip"].as_str(), Some("192.168.100.21"));
        assert_eq!(robots[1]["ip"].as_str(), Some("192.168.100.12"));
    }

    /// On a virtual cell, nothing saved: the build gets its KUKA VCs' values (and keeps the cell's own).
    #[tokio::test]
    async fn a_build_for_a_virtual_cell_gets_its_robots_values() {
        let f = fixture("vc");
        let server_cell = f.root.join("cell.toml");
        std::fs::write(&server_cell, "[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\n\n[[kuka_vc.controller]]\nrobot_ns = \"robot2\"\n").unwrap();
        std::fs::create_dir_all(f.cell.join("plan2cell")).unwrap();
        std::fs::write(f.cell.join(BASE_CONFIG), "[[plc.robot]]\nns = \"robot2\"\nmodel = \"KR240\"\n").unwrap();
        let binary = f.service.binary.lock().unwrap().clone();
        let service = CodegenService::with_sink(f.recorder.clone(), f.service.config.clone(), binary)
            .on_virtual_cell(hardware::VirtualCell::at(server_cell));
        let accepted = service.start(request("bt_ok")).await.unwrap();
        assert_eq!(finished(&service).await.state, CodegenState::Done);
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        let text = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap();
        assert!(text.contains("with the virtual cell's values where it has none (robot1, robot2)"), "{text}");
        let toml: toml::Table = text.parse().unwrap();
        assert_eq!(toml["plc"]["network"]["gateway"].as_str(), Some(factoryos_protocol::hardware::VIRTUAL_CELL_GATEWAY));
        let robots = toml["plc"]["robot"].as_array().unwrap();
        let flags: Vec<_> = robots
            .iter()
            .map(|r| (r["ns"].as_str().unwrap(), r["external"].as_bool(), r["safety_bypass"].as_bool(), r["safety_module"].as_bool()))
            .collect();
        assert_eq!(flags, [("robot2", Some(true), Some(true), Some(false)), ("robot1", Some(true), Some(true), Some(false))]);
        assert_eq!(robots[0]["model"].as_str(), Some("KR240"), "the cell's own values stay");

        // Real hardware: the cell's file as it is.
        let accepted = f.service.start(request("bt_ok")).await.unwrap();
        finished(&f.service).await;
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&accepted.build_id[..STAMP_LEN]);
        let toml: toml::Table = std::fs::read_to_string(dir.join(BUILD_CONFIG)).unwrap().parse().unwrap();
        assert!(toml["plc"].get("network").is_none() && toml["plc"]["robot"][0].get("safety_module").is_none(), "{toml:#?}");
    }

    static LOOKUP: Mutex<Option<PathBuf>> = Mutex::new(None);

    #[tokio::test]
    async fn the_binary_is_looked_up_until_found_and_rebuilds_are_noticed() {
        use std::os::unix::fs::PermissionsExt as _;

        let f = fixture("lookup");
        let fake = f.service.binary.lock().unwrap().clone().unwrap();
        let service = Arc::new(CodegenService {
            sink: f.recorder.clone(),
            config: f.service.config.clone(),
            binary: Mutex::new(None),
            lookup: Some(|| LOOKUP.lock().unwrap().clone()),
            timeout: BUILD_TIMEOUT,
            virtual_cell: hardware::VirtualCell::none(),
            inner: Mutex::default(),
        });
        assert_eq!(service.options().await.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        *LOOKUP.lock().unwrap() = Some(fake.clone());
        assert_eq!(service.options().await.unwrap().generator, "plan2cell fake", "found once built");

        // Rebuilt (a new mtime): asked again.
        std::thread::sleep(Duration::from_millis(20));
        let tmp = fake.with_extension("new");
        std::fs::write(&tmp, FAKE.replace("\"generator\":\"plan2cell fake\"", "\"generator\":\"plan2cell fake 2\"")).unwrap();
        std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o755)).unwrap();
        std::fs::rename(&tmp, &fake).unwrap();
        assert_eq!(service.options().await.unwrap().generator, "plan2cell fake 2");
    }

    #[tokio::test]
    async fn paths_and_ids_stay_inside_the_builds() {
        let f = fixture("paths");
        let accepted = f.service.start(request("bt_ok")).await.unwrap();
        finished(&f.service).await;
        let id = accepted.build_id.as_str();
        std::fs::write(f.root.join("secret.txt"), "secret").unwrap();
        let dir = f.cell.join(BUILDS_DIR).join("bt_ok").join(&id[..STAMP_LEN]);
        std::os::unix::fs::symlink(f.root.join("secret.txt"), dir.join("link.txt")).unwrap();
        for bad in [
            "../../../../secret.txt",
            "/etc/passwd",
            "plc/../../secret.txt",
            "./BUILD.md",
            "plc//codesys/PLC_PRG.st",
            "C:\\x",
            "",
            "plc\\..\\x",
        ] {
            let err = f.service.file(id, bad).unwrap_err();
            assert_eq!(err.0, StatusCode::BAD_REQUEST, "{bad}: {err:?}");
        }
        assert_eq!(f.service.file(id, "link.txt").unwrap_err().0, StatusCode::NOT_FOUND, "a symlink out of the build");
        assert_eq!(f.service.file(id, "plc/missing.st").unwrap_err().0, StatusCode::NOT_FOUND);
        assert!(f.service.file(id, "BUILD.md").is_ok());
        let archive = f.service.archive(id).unwrap();
        let archive = zip::ZipArchive::new(std::io::Cursor::new(archive)).unwrap();
        assert!(!archive.file_names().any(|n| n.unwrap().ends_with("link.txt")), "symlinks stay out of the zip");
        for bad in [
            "..",
            "../x",
            "20261012T153045123Z-..",
            "20261012T153045123Z-a/b",
            "2026101xT153045123Z-bt_ok",
            "20261012T153045123Z",
            "x-bt_ok",
        ] {
            assert_eq!(f.service.info(bad).unwrap_err().0, StatusCode::BAD_REQUEST, "{bad}");
        }
        assert_eq!(f.service.info("20000101T000000000Z-bt_ok").unwrap_err().0, StatusCode::NOT_FOUND);
    }

    #[test]
    fn stamps_and_config() {
        let t = SystemTime::UNIX_EPOCH + Duration::from_millis(1_791_000_000_123);
        let stamp = utc_stamp(t);
        assert_eq!(stamp, "20261003T040000123Z");
        assert!(valid_stamp(&stamp));
        assert_eq!(bump_stamp(&stamp), "20261003T040000124Z");
        assert!(valid_stamp(&bump_stamp("20261003T040000999Z")));
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(19_782), (2024, 2, 29));
        assert_eq!(parse_id("20261003T040000123Z-bt_1.x"), Some(("20261003T040000123Z", "bt_1.x")));

        let request = BuildRequest { opcua_symbols: true, job_number: "byte".to_owned(), ..request("p") };
        let base = "[build]\nkrl = false\n[[krl.robot]]\nns = \"robot1\"\ne_axes = \"fixed\"\ne_values = [95.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n[plc.cell]\nprod_folge = 12\n";
        let toml: toml::Table = build_config(Some(base), true, &[], &request, None).unwrap().parse().unwrap();
        assert_eq!(toml["build"]["krl"].as_bool(), Some(true));
        assert_eq!(toml["plc"]["cell"]["prod_folge"].as_integer(), Some(12), "the cell's settings stay");
        assert_eq!(toml["plc"]["cell"]["job_number"].as_str(), Some("byte"));
        assert_eq!(toml["plc"]["opcua"]["symbols"].as_bool(), Some(true));
        assert_eq!(toml["krl"]["robot"][0]["e_axes"].as_str(), Some("fixed"));
        assert!(build_config(Some("[build"), true, &[], &request, None).is_err());
        assert!(build_config(Some("build = 1"), true, &[], &request, None).is_err());
        let args = cli_args(Path::new("/c/p.json"), Path::new("/c/b"), &BuildRequest { st_check: false, ..request });
        assert!(args.contains(&"--no-check".to_owned()) && args.contains(&"--json".to_owned()));
    }

    /// End to end through the real CLI: `cd plan2cell && cargo build -p plan2cell-cli`, then
    /// `cargo test -p factoryos-backend -- --ignored real_plan2cell` (or point
    /// `FACTORY_OS_PLAN2CELL` at another build; CI does, at its release build). It checks the
    /// contract the fake above stands in for: that plan2cell accepts the config the backend writes
    /// (its `deny_unknown_fields`) with every form option set, and the report's fields.
    #[tokio::test]
    #[ignore = "needs the plan2cell binary (cd plan2cell && cargo build -p plan2cell-cli)"]
    async fn real_plan2cell_builds_the_sample_plan() {
        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let binary = std::env::var_os("FACTORY_OS_PLAN2CELL")
            .map(PathBuf::from)
            .unwrap_or_else(|| repo.join("plan2cell/target/debug/plan2cell"));
        assert!(binary.is_file(), "build it first: {}", binary.display());
        let f = fixture("real");
        let plan = "bt_1790934549_welding_cell_2_small";
        std::fs::copy(
            repo.join("plan2cell/test_data").join(format!("{plan}.json")),
            f.cell.join(cells::PLANS_DIR).join(format!("{plan}.json")),
        )
        .unwrap();
        let service = CodegenService::with_sink(f.recorder.clone(), f.service.config.clone(), Some(binary));
        let options = service.options().await.unwrap();
        assert_eq!(options.plc_dialects.iter().filter(|c| c.implemented).count(), 1);
        assert_eq!(options.robot_languages.len(), 5);

        // The cell's own plan2cell.toml under the backend's options, none of them at its default.
        let base = f.cell.join(BASE_CONFIG);
        std::fs::create_dir_all(base.parent().unwrap()).unwrap();
        std::fs::write(
            &base,
            "[plc.cell]\nprod_folge = 11\n\n[[krl.robot]]\nns = \"robot1\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n\n\
             [[krl.robot]]\nns = \"robot2\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n",
        )
        .unwrap();
        let options_set = BuildRequest {
            job_number: "byte".to_owned(),
            opcua_symbols: true,
            robot_sim: false,
            st_check: false,
            ..BuildRequest::new(format!("agentic_planner/plans/{plan}.json"), &service.options().await.unwrap().defaults)
        };
        let accepted = service.start(options_set.clone()).await.unwrap();
        let status = finished(&service).await;
        assert_eq!(status.state, CodegenState::Done, "{status:?}\n{:#?}", f.recorder.logs.lock().unwrap());
        let info = service.info(&accepted.build_id).unwrap();
        let summary = info.summary.as_ref().unwrap();
        assert!(!summary.line.is_empty(), "the report's summary line");
        assert!(!summary.st_checked, "--no-check reached the CLI");
        assert_eq!(summary.tree_id, plan);
        assert_eq!(summary.robots.len(), 2);
        assert!(!info.notes.iter().any(|n| n.contains("e_axes")), "the cell declares its robots' e_axes: {:?}", info.notes);
        assert!(info.files.iter().any(|f| f.path == "plc/codesys/head/POU/FB_Cell_Body.st"));
        assert!(info.files.iter().any(|f| f.path == "krl/robot1/KRC/R1/Folgen/folge11.src"));
        let (bytes, text) = service.file(&accepted.build_id, "krl/robot1/KRC/R1/Folgen/folge11.src").unwrap();
        assert!(text && !bytes.is_empty());
        let zip = service.archive(&accepted.build_id).unwrap();
        let archive = zip::ZipArchive::new(std::io::Cursor::new(zip)).unwrap();
        assert!(archive.len() > info.files.len(), "{} entries", archive.len());
        eprintln!(
            "real plan2cell: {} in {:.2} s, {} files, {} warnings, {} notes, zip {} entries",
            summary.line,
            info.duration_s,
            info.files.len(),
            info.count(Severity::Warning),
            info.notes.len(),
            archive.len()
        );

        // Strict: whatever it makes of the warnings, never a config (3) or usage (2) error.
        let accepted = service.start(BuildRequest { strict: true, ..options_set }).await.unwrap();
        finished(&service).await;
        let info = service.info(&accepted.build_id).unwrap();
        assert!(!matches!(info.exit_code, Some(2 | 3)), "{:?}: {:?}", info.exit_code, info.error);

        // A planned dialect through the real CLI: a failed build with its "not implemented".
        let accepted = service
            .start(BuildRequest { plc_dialect: "tia-scl".to_owned(), ..request(plan) })
            .await
            .unwrap();
        assert_eq!(finished(&service).await.state, CodegenState::Failed);
        let info = service.info(&accepted.build_id).unwrap();
        assert_eq!(info.exit_code, Some(5));
        assert!(info.error.unwrap().contains("not implemented"));
    }
}
