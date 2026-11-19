//! The stages. Each one can run on its own and is idempotent; with `dry_run` every planned ssh,
//! SSM and MCP call is recorded in [`Report::calls`] and nothing is sent.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::codesys::{self, Call, Check, Deferred, Expect};
use crate::config::{CellConfig, KukaController, KukaVc, OtherCodesys};
use crate::krl::{self, Bundle, ControllerReport, Remotes, VcRemotes};
use crate::mcp::{self, ToolInfo, ToolOutput, Tools};
use crate::project::{DeviceId, Manifest, Project, Role};
use crate::remote::Remote;
use crate::{Error, Result};

pub const REPORT_SCHEMA: &str = "plan2cell/cell-report@1";
/// Where the report is written in the build directory.
pub const REPORT_FILE: &str = "plan2cell-cell-report.json";
const OVERLAY: &str = "plc/docker/docker-compose.robots.yml";
const OVERLAY_TMP: &str = "/tmp/plan2cell-docker-compose.robots.yml";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Stage {
    Preflight,
    Runtimes,
    Projects,
    Deploy,
    /// KRL onto the KUKA virtual controllers of `[kuka_vc]` (skipped in a run of several stages
    /// when the cell has none).
    Krl,
    Verify,
    /// Bring the deployed cell into production: runtimes up, KUKA VCs acknowledged and in EXT,
    /// the head out of error / recovery until it runs (not part of `up`).
    Start,
    /// Abort the cell (`PLC_PRG.bStop`).
    Stop,
    /// The MCP server's CODESYS restarted: logouts and the build's projects closed without
    /// saving (with a build directory), `shutdown_codesys` (unsaved changes discarded),
    /// `launch_codesys`, `get_codesys_status` (exit 0 when it takes calls). Not part of `up`.
    ResetIde,
}

impl Stage {
    pub const ALL: [Stage; 6] = [
        Stage::Preflight,
        Stage::Runtimes,
        Stage::Projects,
        Stage::Deploy,
        Stage::Krl,
        Stage::Verify,
    ];

    /// Every stage in report order.
    pub const ORDER: [Stage; 9] = [
        Stage::Preflight,
        Stage::Runtimes,
        Stage::Projects,
        Stage::Deploy,
        Stage::Krl,
        Stage::Verify,
        Stage::Start,
        Stage::Stop,
        Stage::ResetIde,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Stage::Preflight => "preflight",
            Stage::Runtimes => "runtimes",
            Stage::Projects => "projects",
            Stage::Deploy => "deploy",
            Stage::Krl => "krl",
            Stage::Verify => "verify",
            Stage::Start => "start",
            Stage::Stop => "stop",
            Stage::ResetIde => "reset-ide",
        }
    }

    fn needs_build(self) -> bool {
        !matches!(self, Stage::Preflight | Stage::ResetIde)
    }
}

/// A build directory (`<out>/plc/...`), on disk or still in memory (a dry-run `up`).
pub enum Tree {
    Dir(PathBuf),
    Mem {
        dir: PathBuf,
        files: BTreeMap<String, String>,
    },
}

impl Tree {
    pub fn dir(&self) -> &Path {
        match self {
            Tree::Dir(d) | Tree::Mem { dir: d, .. } => d,
        }
    }

    pub fn read(&self, rel: &str) -> Result<String> {
        match self {
            Tree::Dir(d) => {
                let p = d.join(rel);
                std::fs::read_to_string(&p).map_err(|e| Error::io(&p, e))
            }
            Tree::Mem { dir, files } => files.get(rel).cloned().ok_or_else(|| {
                Error::io(
                    &dir.join(rel),
                    std::io::Error::from(std::io::ErrorKind::NotFound),
                )
            }),
        }
    }

    pub fn exists(&self, rel: &str) -> bool {
        match self {
            Tree::Dir(d) => d.join(rel).is_file(),
            Tree::Mem { files, .. } => files.contains_key(rel),
        }
    }

    /// The names of the files directly in directory `rel`, sorted (none when it does not exist).
    pub fn list(&self, rel: &str) -> Result<Vec<String>> {
        let rel = rel.trim_end_matches('/');
        let mut v: Vec<String> = match self {
            Tree::Dir(d) => {
                let p = d.join(rel);
                match std::fs::read_dir(&p) {
                    Ok(rd) => rd
                        .filter_map(|e| e.ok())
                        .filter(|e| e.path().is_file())
                        .filter_map(|e| e.file_name().to_str().map(str::to_string))
                        .collect(),
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
                    Err(e) => return Err(Error::io(&p, e)),
                }
            }
            Tree::Mem { files, .. } => files
                .keys()
                .filter_map(|k| k.strip_prefix(rel)?.strip_prefix('/'))
                .filter(|k| !k.contains('/'))
                .map(str::to_string)
                .collect(),
        };
        v.sort();
        Ok(v)
    }
}

#[derive(Clone, Debug)]
pub struct Settings {
    pub dry_run: bool,
    /// `verify` retries until every check passes or this much time has passed (PROFINET needs a
    /// while to connect after a start).
    pub verify_timeout: Duration,
    pub verify_interval: Duration,
    /// `start`: how long the head may take to reach production, and the time between reads.
    pub start_timeout: Duration,
    pub start_interval: Duration,
    /// `start`: how long the head's login is tried again (gateway, connection, refused login),
    /// counted from the first attempt and again after a runtime restart; separate from
    /// `start_timeout`, which counts from the login. A build error is never tried again.
    pub login_timeout: Duration,
    /// `start`: at most this many operator pulses (`bReset`, `bRecoveryConfirm`).
    pub start_pulses: u32,
    /// `start`: restart the runtime containers even if they answer (a fresh demo period).
    pub restart_runtime: bool,
    /// `start`: how long a restarted runtime may take to open its gateway port.
    pub runtime_wait: Duration,
    /// `start`: after a runtime restart, the KUKA VCs' stop messages (`kvc-ack.sh`) are
    /// acknowledged no sooner than this after the restart (the head's PROFINET AR first has to
    /// come up, the M_2858 it leaves comes with it).
    pub ack_settle: Duration,
    /// A CODESYS an ended MCP session started that the server cannot take over yet (its watcher
    /// does not answer while CODESYS is still loading): the call is sent again every
    /// `others_interval` for up to `others_timeout`, until the server has taken it over.
    pub others_timeout: Duration,
    pub others_interval: Duration,
    /// `factoryos/hook@1`: the pause before the one retry of a verb that changes nothing (exit 4).
    pub hook_retry: Duration,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            dry_run: false,
            verify_timeout: Duration::from_secs(120),
            verify_interval: Duration::from_secs(10),
            start_timeout: Duration::from_secs(300),
            start_interval: Duration::from_secs(3),
            login_timeout: Duration::from_secs(60),
            start_pulses: 8,
            restart_runtime: false,
            runtime_wait: Duration::from_secs(120),
            ack_settle: Duration::from_secs(25),
            others_timeout: Duration::from_secs(300),
            others_interval: Duration::from_secs(15),
            hook_retry: crate::hook::RETRY_WAIT,
        }
    }
}

/// `MCP_PROMPT.md` §4 (`projects[]`), plus the stages, the online checks and the calls made.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Report {
    pub schema: String,
    pub cell_id: Option<String>,
    pub plan_id: Option<String>,
    pub dry_run: bool,
    pub ok: bool,
    pub stages: Vec<StageReport>,
    pub projects: Vec<ProjectReport>,
    /// The KUKA virtual controllers (`krl`, `verify`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub controllers: Vec<ControllerReport>,
    pub calls: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StageReport {
    pub stage: Stage,
    pub ok: bool,
    #[serde(default)]
    pub notes: Vec<String>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ProjectReport {
    pub name: String,
    pub file: String,
    pub built: bool,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
    pub deviations: Vec<String>,
    pub human_steps: Vec<String>,
    pub deployed: Option<bool>,
    pub state: Option<String>,
    pub verified: Option<bool>,
    #[serde(default)]
    pub checks: Vec<CheckResult>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CheckResult {
    pub expr: String,
    pub expect: String,
    pub value: Option<String>,
    pub ok: bool,
}

impl Report {
    pub fn load(path: &Path) -> Option<Self> {
        let t = std::fs::read_to_string(path).ok()?;
        serde_json::from_str::<Report>(&t)
            .ok()
            .filter(|r| r.schema == REPORT_SCHEMA)
    }

    pub fn write(&self, path: &Path) -> Result<()> {
        let tmp = path.with_extension("json.tmp");
        let text = serde_json::to_string_pretty(self).expect("report serialises") + "\n";
        std::fs::write(&tmp, text).map_err(|e| Error::io(&tmp, e))?;
        std::fs::rename(&tmp, path).map_err(|e| Error::io(path, e))
    }
}

/// Single-quoted for a POSIX shell.
pub fn sh(s: &str) -> String {
    if !s.is_empty()
        && s.chars()
            .all(|c| c.is_ascii_alphanumeric() || "/._-:=@".contains(c))
    {
        s.into()
    } else {
        format!("'{}'", s.replace('\'', r"'\''"))
    }
}

fn failed(stage: Stage, code: i32, msg: impl Into<String>) -> Error {
    Error::Failed {
        stage: stage.name(),
        code,
        msg: msg.into(),
    }
}

/// What the calls of one project (or of preflight) produced.
#[derive(Debug, Default)]
struct Outcome {
    errors: Vec<String>,
    warnings: Vec<String>,
    deviations: Vec<String>,
    notes: Vec<String>,
    /// What a person has to do on the IDE (the server has no tool for it).
    human: Vec<String>,
    /// Modules `ensure_module` could not add (their IO mapping rows are left out).
    absent: Vec<String>,
    /// Devices whose identification read back (last path part).
    devices: Vec<String>,
    /// Objects `get_all_pou_code` listed.
    objects: Vec<String>,
    /// DeviceIDs `find_device_description` did not find.
    missing: Vec<DeviceId>,
    compiled: bool,
    state: Option<String>,
    values: BTreeMap<String, String>,
    /// `list_configured_symbols`: `Signature.variable` -> effective access (`None`: no answer
    /// to read).
    symbols: Option<Vec<(String, String)>>,
    /// The answers of `export_project_libraries` (for the copy on the workspace).
    libraries: Vec<String>,
    /// References `list_project_libraries` showed without a library (for the build's message).
    unresolved: Vec<String>,
}

const INTERACTIVE: &str = "cannot be taken over";
/// How the server ends the line of a CODESYS without unsaved changes in its question.
const NO_UNSAVED: &str = "no unsaved changes";
/// The kind the server gives a CODESYS an MCP session started whose server has ended.
const LEFT_BY_MCP: &str = "left by an ended MCP session";

/// The CODESYS the server's question lists (`- PID 2540, opened by hand: CODESYS; no unsaved
/// changes`), without the dash.
fn other_codesys(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(|l| l.trim().strip_prefix("- PID "))
        .map(|l| format!("PID {}", l.trim()))
        .collect()
}

/// Leave out the `robot_sim` projects of robots a KUKA VC of the cell plays (`[[kuka_vc.controller]]`
/// `robot_ns`): the VC is that robot's PROFINET device, so a CODESYS simulator project for it would
/// be deployed to the VC's address, where no CODESYS runtime answers (2026-10-17: deploy
/// `connect_to_device` of `RobotSim_*_R1` on a per-workspace cell). Returns every robot_sim
/// project's robot in project.json's order (the head numbers its robots so) and a note per
/// project left out.
fn leave_out_vc_sims(m: &mut Manifest, cfg: &CellConfig) -> (Vec<Option<String>>, Vec<String>) {
    let sims = m
        .projects
        .iter()
        .filter(|p| p.role == Role::RobotSim)
        .map(|p| p.robot.clone())
        .collect();
    let vcs: Vec<&str> = cfg
        .kuka_vc
        .iter()
        .flat_map(|k| &k.controllers)
        .map(|c| c.robot_ns.as_str())
        .collect();
    let mut notes = Vec::new();
    m.projects.retain(|p| {
        let vc = p.role == Role::RobotSim
            && p.robot.as_deref().is_some_and(|r| vcs.contains(&r));
        if vc {
            notes.push(format!(
                "{} left out: {} is a KUKA VC of the cell ([kuka_vc]), not a CODESYS robot simulator",
                p.name,
                p.robot.as_deref().unwrap_or("")
            ));
        }
        !vc
    });
    (sims, notes)
}

/// The sub-elements `get_device_parameter` listed, by (device path, parameter id).
type Labels = HashMap<(String, u32), Vec<(i64, String, String)>>;

/// How often a call is sent while the server answers [`transient`].
const RETRIES: u32 = 4;

/// The CODESYS-side watcher could not read the command file (another process held it, e.g. a
/// virus scan): the script did not run, so sending the call again is safe.
pub fn transient(text: &str) -> bool {
    text.contains("Read error: [Errno 32]") || text.contains("Read error: [Errno 13]")
}

/// The online tools' answer when the IDE has no gateway at all.
const NO_GATEWAY: &str = "No gateway is configured in this CODESYS";

/// An answer of the IDE without a gateway: the rebind/scan scripts say so, a login of a project
/// bound to no gateway says "Gateway not configured properly".
pub fn no_gateway(text: &str) -> bool {
    text.contains(NO_GATEWAY) || text.contains("Gateway not configured properly")
}
/// CODESYS creates its default gateway (Gateway-1, localhost:1217) only when a device editor's
/// Communication Settings page is opened; scripting alone never does.
const GATEWAY_STEP: &str = "IDE: open any project's Device > Communication Settings once in the \
     MCP server's CODESYS (over DCV) so CODESYS creates Gateway-1 (localhost:1217), or add that \
     gateway there; no tool of the server configures gateways";

/// The human steps `projects` raises (a module or a struct member the server cannot set): a
/// later `projects` run replaces them.
const MODULE_STEP: &str = ": Add Device > module id ";
const MEMBER_STEP: &str = " in the device editor";

fn projects_step(h: &str) -> bool {
    h.contains(MODULE_STEP) || h.ends_with(MEMBER_STEP)
}

/// The last line of an answer with text (without the server's `DEBUG:` lines), for a note.
fn last_line(r: &ToolOutput) -> String {
    r.short()
        .lines()
        .rev()
        .find(|l| !l.trim().is_empty())
        .unwrap_or("no answer")
        .trim()
        .to_string()
}

/// The overlay's library tools: the summary and `ERROR:` lines before the JSON between the
/// markers (`; `-joined); any other answer: its last line.
fn library_answer(r: &ToolOutput) -> String {
    let text = r.short();
    match text.find("\n### ") {
        Some(i) => text[..i]
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty())
            .collect::<Vec<_>>()
            .join("; "),
        None => last_line(r),
    }
}

/// `connect_to_device` refused for want of a device user (a fresh runtime with user management).
fn login_refused(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    t.contains("authentication") || t.contains("user management") || t.contains("no user")
}

/// The orchestrator: one cell, one build directory.
pub struct Cell<'a> {
    cfg: CellConfig,
    tree: Option<Tree>,
    manifest: Option<Manifest>,
    settings: Settings,
    remote: &'a mut dyn Remote,
    tools: &'a mut dyn Tools,
    /// The hosts of the KUKA virtual controllers.
    vc: Option<Box<dyn Remotes + 'a>>,
    /// Display names of the device descriptions, by DeviceID.
    names: HashMap<DeviceId, String>,
    /// The optional arguments the server has, once `tools/list` was read.
    caps: Option<codesys::Caps>,
    /// The robot of each `robot_sim` project of project.json, in its order (also those left out).
    sim_robots: Vec<Option<String>>,
    /// The `robot_sim` projects left out: their robot is a KUKA VC of the cell (a note per stage).
    left_out: Vec<String>,
    /// The CODESYS in the server's way were closed once this run (`other_codesys = "close"`).
    others_closed: bool,
    /// Until when a call waits for the server to take over a CODESYS an ended MCP session left.
    others_until: Option<Instant>,
    /// Notes for the running stage's report from outside its own notes (`exec`).
    pending_notes: Vec<String>,
    /// The hook pods `describe` answered for this session (`factoryos/hook@1`, first contact).
    described: std::collections::HashSet<String>,
    pub report: Report,
}

impl<'a> Cell<'a> {
    /// `previous` is the report of an earlier run on the same build directory; it is updated.
    pub fn new(
        cfg: CellConfig,
        tree: Option<Tree>,
        settings: Settings,
        remote: &'a mut dyn Remote,
        tools: &'a mut dyn Tools,
        previous: Option<Report>,
    ) -> Result<Self> {
        let mut manifest = match &tree {
            Some(t) => Some(Manifest::parse(&t.read("plc/project.json")?)?),
            None => None,
        };
        let (sim_robots, left_out) = manifest
            .as_mut()
            .map(|m| leave_out_vc_sims(m, &cfg))
            .unwrap_or_default();
        let mut report = previous.unwrap_or_default();
        report.schema = REPORT_SCHEMA.into();
        report.dry_run = settings.dry_run;
        report.calls.clear();
        let vc = cfg
            .kuka_vc
            .clone()
            .map(|k| Box::new(VcRemotes::new(k)) as Box<dyn Remotes + 'a>);
        let mut c = Self {
            cfg,
            tree,
            manifest,
            settings,
            remote,
            tools,
            vc,
            names: HashMap::new(),
            caps: None,
            sim_robots,
            left_out,
            others_closed: false,
            others_until: None,
            pending_notes: Vec::new(),
            described: Default::default(),
            report,
        };
        if let Some(m) = c.manifest.clone() {
            if c.report.plan_id.as_deref() != Some(m.plan.plan_id.as_str()) {
                // Another build: nothing of the old report applies.
                c.report.stages.clear();
                c.report.projects.clear();
                c.report.controllers.clear();
            }
            c.report.cell_id = Some(m.plan.cell_id.clone());
            c.report.plan_id = Some(m.plan.plan_id.clone());
            let opts = c.options();
            let mut projects = Vec::new();
            for p in m.ordered() {
                let mut r = c
                    .report
                    .projects
                    .iter()
                    .find(|r| r.name == p.name)
                    .cloned()
                    .unwrap_or_default();
                r.name = p.name.clone();
                r.file = opts.project_path(p);
                projects.push(r);
            }
            c.report.projects = projects;
        }
        Ok(c)
    }

    /// Reach the KUKA VC hosts through `r` instead of SSM / ssh (tests).
    pub fn with_controllers(mut self, r: Box<dyn Remotes + 'a>) -> Self {
        self.vc = Some(r);
        self
    }

    /// The tool options for this cell.
    pub fn options(&self) -> codesys::Options {
        let m = &self.cfg.codesys_mcp;
        let cell = self
            .manifest
            .as_ref()
            .map(|m| m.plan.cell_id.as_str())
            .unwrap_or("cell");
        codesys::Options {
            dir: m.project_dir(cell),
            install: m.install.clone(),
            io_csv: m.io_csv,
            credentials: m.device_credentials(),
            caps: self.caps.unwrap_or_default(),
            address_map: self.cfg.address_map.clone(),
        }
    }

    /// The optional arguments of the server, from `tools/list` (once per run). A dry run
    /// assumes our overlay's server.
    fn load_caps(&mut self) -> Result<codesys::Caps> {
        if let Some(c) = self.caps {
            return Ok(c);
        }
        let tools = self.list_tools()?;
        Ok(self.set_caps(&tools))
    }

    fn set_caps(&mut self, tools: &[ToolInfo]) -> codesys::Caps {
        let c = if self.settings.dry_run {
            codesys::Caps::ALL
        } else {
            codesys::Caps::from_tools(tools)
        };
        self.caps = Some(c);
        c
    }

    /// Run the stages in order; the first failing one stops the run.
    pub fn run(&mut self, stages: &[Stage]) -> Result<()> {
        let mut result = Ok(());
        let no_vc = self.cfg.controllers().is_empty();
        for &st in stages {
            if st == Stage::Krl && no_vc && stages.len() > 1 {
                continue;
            }
            let mut notes = Vec::new();
            if matches!(
                st,
                Stage::Runtimes | Stage::Projects | Stage::Deploy | Stage::Verify
            ) {
                notes.extend(self.left_out.iter().cloned());
            }
            let r = self.stage(st, &mut notes);
            notes.append(&mut self.pending_notes);
            let rep = StageReport {
                stage: st,
                ok: r.is_ok(),
                notes,
                error: r.as_ref().err().map(|e| e.to_string()),
            };
            match self.report.stages.iter_mut().find(|s| s.stage == st) {
                Some(s) => *s = rep,
                None => self.report.stages.push(rep),
            }
            // Start and stop each describe the head's current state, and only the last one that
            // ran still does: after a start, an earlier stop's `CS_ERROR ERR_OPERATOR_STOP` (and
            // after a stop, an earlier start's `CS_RUNNING`) is history. Left in, it would sort
            // after the start and readers of the report would take it for this run's state.
            if r.is_ok() && !self.settings.dry_run {
                let superseded = match st {
                    Stage::Start => Some(Stage::Stop),
                    Stage::Stop => Some(Stage::Start),
                    _ => None,
                };
                if let Some(old) = superseded {
                    self.report.stages.retain(|s| s.stage != old);
                }
            }
            if r.is_err() {
                result = r;
                break;
            }
        }
        self.report
            .stages
            .sort_by_key(|s| Stage::ORDER.iter().position(|x| *x == s.stage));
        self.report.ok = result.is_ok() && self.report.stages.iter().all(|s| s.ok);
        result
    }

    fn stage(&mut self, st: Stage, notes: &mut Vec<String>) -> Result<()> {
        if st.needs_build() && self.manifest.is_none() {
            return Err(Error::Usage(format!(
                "{} needs a build directory",
                st.name()
            )));
        }
        if self.cfg.plc_host.is_hook() {
            // The head is a factoryos/hook@1 pod: no PLC host, no CODESYS (stages/hooks.rs).
            return match st {
                Stage::Preflight => self.hook_preflight(notes),
                Stage::Runtimes | Stage::Projects => {
                    notes.push(format!(
                        "nothing to do: the head is a factoryos/hook@1 pod (plc_host driver = \"hook\"): no \
                         CODESYS {}; it builds at deploy",
                        if st == Stage::Runtimes { "runtimes" } else { "projects" }
                    ));
                    Ok(())
                }
                Stage::Deploy => self.hook_deploy(notes),
                Stage::Krl => self.krl(notes),
                Stage::Verify => self.hook_verify(notes),
                Stage::Start => self.hook_start(notes),
                Stage::Stop => self.hook_stop(notes),
                Stage::ResetIde => Err(Error::Config(
                    "reset-ide: the head is a factoryos/hook@1 pod (plc_host driver = \"hook\"): there is no \
                     CODESYS IDE"
                        .into(),
                )),
            };
        }
        match st {
            Stage::Preflight => {
                self.preflight(notes)?;
                self.hook_robots_ready(notes)
            }
            Stage::Runtimes => self.runtimes(notes),
            Stage::Projects => self.projects(notes),
            Stage::Deploy => self.deploy(notes),
            Stage::Krl => self.krl(notes),
            Stage::Verify => self.verify(notes),
            Stage::Start => self.start(notes),
            Stage::Stop => self.stop(notes),
            Stage::ResetIde => self.reset_ide(notes),
        }
    }

    fn manifest(&self) -> &Manifest {
        self.manifest.as_ref().expect("checked by stage()")
    }

    fn ssh(&mut self, cmd: &str) -> Result<String> {
        self.report
            .calls
            .push(format!("ssh {}: {cmd}", self.cfg.plc_host.ssh_host));
        if self.settings.dry_run {
            return Ok(String::new());
        }
        self.remote.run(cmd)
    }

    fn scp(&mut self, rel: &str, remote: &str) -> Result<()> {
        let local = self
            .tree
            .as_ref()
            .expect("checked by stage()")
            .dir()
            .join(rel);
        self.report.calls.push(format!(
            "scp {} {}:{remote}",
            local.display(),
            self.cfg.plc_host.ssh_host
        ));
        if self.settings.dry_run {
            return Ok(());
        }
        self.remote.copy(&local, remote)
    }

    fn list_tools(&mut self) -> Result<Vec<ToolInfo>> {
        self.report
            .calls
            .push(format!("mcp {}: tools/list", self.cfg.codesys_mcp.url));
        if self.settings.dry_run {
            return Ok(codesys::TOOLS
                .iter()
                .map(|t| ToolInfo {
                    name: t.to_string(),
                    args: Vec::new(),
                })
                .collect());
        }
        self.tools.list()
    }

    /// Record and send one call. A password the call sent is masked in the answer, which ends
    /// up in error messages and the report. A CODESYS dialog nobody answered (the bridge says so,
    /// [`mcp::dialog`]) is [`Error::Dialog`]: it blocks every later call too, so no stage retries it.
    fn send(&mut self, c: &Call) -> Result<ToolOutput> {
        self.report
            .calls
            .push(format!("mcp {}: {}", self.cfg.codesys_mcp.url, c.show()));
        let dialog = |msg: String| Error::Dialog {
            tool: c.tool.to_string(),
            msg,
        };
        let mut attempt = 0;
        let mut r = loop {
            let r = if c.tool == codesys::STRUCTURE {
                let uri = c.args.get("uri").and_then(|v| v.as_str()).unwrap_or("");
                self.tools.read_resource(uri)
            } else {
                self.tools.call(c.tool, &c.args)
            };
            let r = match r {
                Err(Error::Mcp(m)) => return Err(mcp::dialog(&m).map_or(Error::Mcp(m), dialog)),
                r => r?,
            };
            attempt += 1;
            if attempt < RETRIES && transient(&r.text) {
                std::thread::sleep(Duration::from_secs(2));
                continue;
            }
            break r;
        };
        for s in c.secrets() {
            r.text = r.text.replace(s, "<secret>");
        }
        if let Some(d) = mcp::dialog(&r.text) {
            return Err(dialog(d));
        }
        Ok(r)
    }

    /// Run calls in order and read their answers into `out`. A failed call stops with
    /// [`Error::Tool`]; an interactive CODESYS in the way stops the stage (exit 3).
    fn exec(
        &mut self,
        stage: Stage,
        project: Option<&Project>,
        calls: &[Call],
        out: &mut Outcome,
    ) -> Result<()> {
        let dry = self.settings.dry_run;
        let mut labels: Labels = HashMap::new();
        let mut tasks: Vec<(String, Vec<String>)> = Vec::new();
        // Struct members not written (and so not read back).
        let mut unset: Vec<(String, u32, String)> = Vec::new();
        for c in calls {
            if dry {
                self.report
                    .calls
                    .push(format!("mcp {}: {}", self.cfg.codesys_mcp.url, c.show()));
                continue;
            }
            if c.when.as_ref().is_some_and(|w| !w.holds(&tasks)) {
                continue;
            }
            if c.check == Check::ExportLibraries && !out.errors.is_empty() {
                out.warnings.push(format!(
                    "libraries not stored with the project: the build has {} error(s)",
                    out.errors.len()
                ));
                continue;
            }
            let mut call = c.clone();
            call.deferred.clear();
            let mut skip = false;
            for (k, d) in &c.deferred {
                let v = match d {
                    Deferred::DeviceName(id) => match self.names.get(id) {
                        Some(n) => serde_json::Value::String(n.clone()),
                        None => {
                            return Err(Error::Config(format!(
                                "device description ({}, '{}', '{}') is not in the IDE's \
                                 repository (see `plan2cell cell preflight`)",
                                id.ty, id.id, id.version
                            )));
                        }
                    },
                    Deferred::MemberIndex {
                        device,
                        param,
                        member,
                    } => {
                        let subs = labels.get(&(device.clone(), *param));
                        match subs.and_then(|s| codesys::member_index(s, member)) {
                            Some(i) => serde_json::Value::from(i),
                            None => {
                                let value = c.args.get("value").and_then(|v| v.as_str());
                                let value = value.unwrap_or("");
                                match subs.and_then(|s| codesys::nested_member(s, member)) {
                                    Some(st) => {
                                        out.deviations.push(format!(
                                            "{device}: parameter {param} member {member} not \
                                             set: {st} is a struct, and set_device_parameter \
                                             writes one level of sub-elements"
                                        ));
                                        out.human.push(format!(
                                            "{device}: set parameter {param} {member} = \
                                             {value}{MEMBER_STEP}"
                                        ));
                                    }
                                    None => {
                                        let have: Vec<&str> = subs
                                            .map(|s| s.iter().map(|(_, l, _)| l.as_str()).collect())
                                            .unwrap_or_default();
                                        out.deviations.push(format!(
                                            "{device}: parameter {param} has no member {member} \
                                             (sub-elements: {}); not set",
                                            have.join(", ")
                                        ));
                                    }
                                }
                                unset.push((device.clone(), *param, member.clone()));
                                skip = true;
                                break;
                            }
                        }
                    }
                };
                call.args.insert(k.to_string(), v);
            }
            if skip {
                continue;
            }
            if c.check == Check::IoCarrier
                && !out.absent.is_empty()
                && let Some(csv) = call.args.get("declarationCode").and_then(|v| v.as_str())
            {
                let (kept, n) = codesys::drop_rows(csv, &out.absent);
                if n > 0 {
                    out.deviations.push(format!(
                        "{n} IO mapping row(s) left out: their modules are not in the project ({})",
                        out.absent.join(", ")
                    ));
                }
                call.args
                    .insert("declarationCode".into(), serde_json::Value::String(kept));
            }
            let mut r = self.send(&call)?;
            while r.text.contains(INTERACTIVE) && self.other_codesys(&r)? {
                r = self.send(&call)?;
            }
            if r.text.contains(INTERACTIVE) {
                return Err(failed(
                    stage,
                    3,
                    format!(
                        "a CODESYS opened by hand on the IDE is in the way of the MCP server's ({}): \
                         close it over DCV (or take it over with attach_codesys) and run again",
                        r.short().lines().nth(1).unwrap_or("").trim()
                    ),
                ));
            }
            if no_gateway(&r.text) {
                // The IDE's gateway list is empty: nothing online works, for any project.
                for p in &mut self.report.projects {
                    if !p.human_steps.iter().any(|h| h == GATEWAY_STEP) {
                        p.human_steps.push(GATEWAY_STEP.into());
                    }
                }
                // "Gateway not configured properly" alone is CODESYS's login failing: the IDE may
                // have its gateway (a rebind through it just worked) while no runtime answers at
                // the device's address (2026-10-17: a RobotSim project bound to a KUKA VC's).
                let why = if r.text.contains(NO_GATEWAY) {
                    NO_GATEWAY.to_string()
                } else {
                    format!(
                        "CODESYS cannot log in to {}: \"Gateway not configured properly\" (no \
                         CODESYS runtime answers at the device's address, or the IDE has no gateway)",
                        project.map_or("the device", |p| p.name.as_str())
                    )
                };
                return Err(failed(
                    stage,
                    3,
                    format!("{}: {why} (see human_steps)", c.tool),
                ));
            }
            // A symbol-config edit landed and the build the overlay runs after it failed only on
            // library-open errors (live, 2026-11-18: set_signature_access_bulk, then '#CmpUserMgr
            // Implementation 3.5.21.0', '#CmpVisuHandler 3.5.19.0', '#SysTask 3.5.17.0'): as
            // BuildClean, not a stop; the login builds again and fails deploy if it cannot. Any
            // other build error, or an edit that failed, stays the project's error.
            if r.failed()
                && let Some(e) = codesys::auto_compile_library_errors(&r.text)
            {
                let shown: Vec<&str> = e.iter().take(3).map(String::as_str).collect();
                out.warnings.push(format!(
                    "{}: {} library error(s) in its auto-compile (the login builds again): {}",
                    c.tool,
                    e.len(),
                    shown.join("; ")
                ));
                r = ToolOutput::ok(r.text.clone());
            }
            let tool_err = |r: &ToolOutput| Error::Tool {
                tool: c.tool.to_string(),
                msg: r.short(),
            };
            match &c.check {
                Check::Compile => {
                    let (e, w) = codesys::parse_compile(&r.text);
                    if r.failed() && e.is_empty() {
                        return Err(tool_err(&r));
                    }
                    out.compiled = true;
                    out.errors.extend(e);
                    out.warnings.extend(w);
                    continue;
                }
                Check::Connect if r.failed() => {
                    let add = project.and_then(|p| codesys::add_device_user(p, &self.options()));
                    match add {
                        Some(add) if login_refused(&r.text) => {
                            let a = self.send(&add)?;
                            if a.failed() {
                                return Err(Error::Tool {
                                    tool: add.tool.into(),
                                    msg: a.short(),
                                });
                            }
                            out.notes.push("first login: device user created".into());
                            let again = self.send(&call)?;
                            if again.failed() {
                                return Err(tool_err(&again));
                            }
                        }
                        _ => return Err(tool_err(&r)),
                    }
                    continue;
                }
                Check::Optional(what) if r.failed() => {
                    let why = r
                        .short()
                        .lines()
                        .find(|l| l.contains("Error") || l.contains("not found"))
                        .unwrap_or("failed")
                        .trim()
                        .to_string();
                    out.warnings.push(format!("{what} not set: {why}"));
                    continue;
                }
                Check::ExportLibraries => {
                    if r.failed() {
                        out.warnings.push(format!(
                            "libraries not stored with the project ({}): {}",
                            c.tool,
                            library_answer(&r)
                        ));
                    } else {
                        out.notes.push(format!(
                            "{}: libraries stored in {}",
                            project.map_or("project", |p| p.name.as_str()),
                            self.options().libraries_dir()
                        ));
                        out.libraries.push(r.text.clone());
                    }
                    continue;
                }
                Check::InstallLibraries if r.failed() => {
                    if !codesys::no_libraries_json(&r.text) {
                        return Err(failed(
                            stage,
                            8,
                            format!("{}: {}", c.tool, library_answer(&r)),
                        ));
                    }
                    out.notes.push(format!(
                        "no {} with the project (an older archive): download_missing_libraries",
                        codesys::LIBRARIES_JSON
                    ));
                    if let Some(p) = project {
                        let d = codesys::download_missing_libraries(p, &self.options());
                        self.exec(stage, project, &[d], out)?;
                    }
                    continue;
                }
                Check::InstallLibraries => {
                    out.notes
                        .push(format!("{}: {}", c.tool, library_answer(&r)));
                    continue;
                }
                Check::DownloadLibraries => {
                    out.notes.push(if r.failed() {
                        format!("{} failed (the build decides): {}", c.tool, last_line(&r))
                    } else {
                        format!("{}: {}", c.tool, last_line(&r))
                    });
                    continue;
                }
                Check::LibrariesResolved => {
                    if r.failed() {
                        return Err(failed(stage, 8, format!("{}: {}", c.tool, r.short())));
                    }
                    let (unresolved, errors) = codesys::unresolved_libraries(&r.text);
                    if !errors.is_empty() {
                        return Err(failed(
                            stage,
                            8,
                            format!("{}: {}", c.tool, errors.join("; ")),
                        ));
                    }
                    out.unresolved = unresolved;
                    continue;
                }
                Check::BuildClean => {
                    let (mut e, mut w) = codesys::parse_compile(&r.text);
                    let mut last = r.clone();
                    // The first build right after the project is opened again (start: closed, the
                    // libraries installed, listed) can report library-open errors of versions the
                    // repository does not have, which the next build of the same project resolves
                    // (live, 2026-11-02: '#SysTask 3.5.17.0' etc., then 0 errors). Only library-open
                    // errors: built once more; a library really missing fails as before.
                    if codesys::library_open_only(&e) {
                        out.notes.push(format!(
                            "{}: {} library error(s) on the first build after the open; libraries \
                             refreshed (download_missing_libraries, list_project_libraries), built again",
                            c.tool,
                            e.len()
                        ));
                        // The same session builds the same errors again (live, 2026-11-03, a fresh
                        // compile node): what cleared them by hand was download_missing_libraries and
                        // the listing before the build (CODESYS's library view of the session, stale
                        // after install_libraries installed into the repository).
                        if let Some(p) = project {
                            let o = self.options();
                            self.send(&codesys::download_missing_libraries(p, &o))?;
                            if let Some(list) = codesys::libraries_ready(p, &o).get(1) {
                                self.send(list)?;
                            }
                        }
                        last = self.send(&call)?;
                        (e, w) = codesys::parse_compile(&last.text);
                        // Still only library-open errors: the offline build resolves the libraries'
                        // own placeholders to versions no IDE here has (CmpUserMgr Implementation
                        // 3.5.21.0, SysTask 3.5.17.0, ...), which the login's build, after the device
                        // is bound to the running PLC, does not need (deploy's login built and ran the
                        // same project). Not a stop: the login builds again and fails start if it
                        // cannot.
                        if codesys::library_open_only(&e) {
                            let shown: Vec<&str> = e.iter().take(3).map(String::as_str).collect();
                            out.notes.push(format!(
                                "warning: {}: {} library error(s) before the login (the login builds again): {}",
                                project.map_or("the project", |p| p.name.as_str()),
                                e.len(),
                                shown.join("; ")
                            ));
                            continue;
                        }
                    }
                    if e.is_empty() && last.failed() {
                        return Err(failed(stage, 8, format!("{}: {}", c.tool, last.short())));
                    }
                    let name = project.map_or("the project", |p| p.name.as_str());
                    if !e.is_empty() {
                        let shown: Vec<&str> = e.iter().take(5).map(String::as_str).collect();
                        let mut msg = format!(
                            "{name}: {} build error(s) before the login: {}",
                            e.len(),
                            shown.join("; ")
                        );
                        if !out.unresolved.is_empty() {
                            msg.push_str(&format!(
                                " (library references that resolve to nothing: {})",
                                out.unresolved.join(", ")
                            ));
                        }
                        out.errors.extend(e);
                        return Err(failed(stage, 8, msg));
                    }
                    out.notes
                        .push(format!("{name} builds ({} warning(s))", w.len()));
                    continue;
                }
                _ if r.failed() => return Err(tool_err(&r)),
                Check::Module(m) => {
                    self.module(stage, project, m, &r.text, out)?;
                    continue;
                }
                Check::EmptySlot(e) => {
                    self.empty_slot(stage, project, e, &r.text, out)?;
                    continue;
                }
                _ => {}
            }
            self.read_back(&c.check, &r.text, out, &mut labels, &mut tasks, &unset);
        }
        Ok(())
    }

    /// A CODESYS the server cannot take over is in its way ([`INTERACTIVE`]); `true`: send the
    /// call again. One an ended MCP session started (a server that gave up while it was coming
    /// up, 2026-10-17) is waited for: the server takes it over once its watcher answers, which it
    /// does only when CODESYS has finished loading. Else, with `codesys_mcp.other_codesys =
    /// "close"` (a cell nobody works in by hand) and none of them with unsaved changes, the server
    /// closes them (`launch_codesys` `otherInstances: close`, once per run). Else (`false`) the
    /// stage refuses as before.
    fn other_codesys(&mut self, r: &ToolOutput) -> Result<bool> {
        let others = other_codesys(&r.text);
        if !others.is_empty() && others.iter().all(|o| o.contains(LEFT_BY_MCP)) {
            if self.others_until.is_none() {
                let msg = format!(
                    "waiting up to {} s for the MCP server to take over the CODESYS an ended MCP \
                     session started ({}): it answers once CODESYS has finished loading",
                    self.settings.others_timeout.as_secs(),
                    others.join("; ")
                );
                eprintln!("note: {msg}");
                self.pending_notes.push(msg);
            }
            let until = *self
                .others_until
                .get_or_insert_with(|| Instant::now() + self.settings.others_timeout);
            if Instant::now() < until {
                if self.settings.dry_run {
                    return Ok(false);
                }
                std::thread::sleep(self.settings.others_interval);
                return Ok(true);
            }
        }
        if self.cfg.codesys_mcp.other_codesys != OtherCodesys::Close || self.others_closed {
            return Ok(false);
        }
        if others.is_empty() || others.iter().any(|o| !o.ends_with(NO_UNSAVED)) {
            self.pending_notes.push(format!(
                "a CODESYS the MCP server did not start is in its way and was not closed: unsaved \
                 changes, or not known ({})",
                others.join("; ")
            ));
            return Ok(false);
        }
        self.others_closed = true;
        let a = self.send(&codesys::close_other_codesys(&self.options()))?;
        let said = a.short().split_whitespace().collect::<Vec<_>>().join(" ");
        let msg = format!(
            "a CODESYS the MCP server did not start was in its way ({}): closed, as \
             codesys_mcp.other_codesys = \"close\": {said}",
            others.join("; ")
        );
        eprintln!("note: {msg}");
        self.pending_notes.push(msg);
        Ok(!a.failed())
    }

    /// `ensure_module` once the device's children are known: plug, rename or a human step.
    fn module(
        &mut self,
        stage: Stage,
        project: Option<&Project>,
        m: &codesys::Module,
        structure: &str,
        out: &mut Outcome,
    ) -> Result<()> {
        let Some(p) = project else {
            return Ok(());
        };
        let action = m.action(structure);
        match &action {
            codesys::ModuleAction::NoDevice => {
                out.deviations
                    .push(format!("{}: not in the project structure", m.device));
                out.absent.push(m.name.clone());
                return Ok(());
            }
            codesys::ModuleAction::FreeList(n) if m.add => out.notes.push(format!(
                "{}: module {} (module id {}) added to its {n} child(ren)",
                m.device, m.name, m.module_id
            )),
            codesys::ModuleAction::FreeList(n) => {
                out.deviations.push(format!(
                    "{}: module {} (module id {}) not added: the device has {n} child(ren) and \
                     no slot {}; it takes its modules as a free child list, and no tool of the \
                     server adds a module by module id (add_device has no moduleId)",
                    m.device, m.name, m.module_id, m.slot
                ));
                out.human.push(format!(
                    "{}{MODULE_STEP}{} ({}) as {}",
                    m.device,
                    m.module_id,
                    m.ident.as_deref().unwrap_or("?"),
                    m.name
                ));
                out.absent.push(m.name.clone());
                return Ok(());
            }
            codesys::ModuleAction::Rename(from) => out.notes.push(format!(
                "{}: slot {} ({from}) renamed to {}",
                m.device, m.slot, m.name
            )),
            _ => {}
        }
        let opts = self.options();
        let calls = m.calls(&action, &opts, &opts.project_path(p));
        self.exec(stage, project, &calls, out)?;
        out.devices.push(m.name.clone());
        Ok(())
    }

    /// `ensure_empty_slot` once the device's children are known: unplug the module in the slot
    /// (the default one its description plugs there), then read back that it is empty.
    fn empty_slot(
        &mut self,
        stage: Stage,
        project: Option<&Project>,
        e: &codesys::EmptySlot,
        structure: &str,
        out: &mut Outcome,
    ) -> Result<()> {
        let Some(p) = project else {
            return Ok(());
        };
        let action = e.action(structure);
        match &action {
            codesys::EmptySlotAction::Empty => {
                out.notes
                    .push(format!("{}: slot {} is empty", e.device, e.slot));
                return Ok(());
            }
            codesys::EmptySlotAction::NoDevice => {
                out.deviations
                    .push(format!("{}: not in the project structure", e.device));
                return Ok(());
            }
            codesys::EmptySlotAction::NoSlot(n) => {
                out.deviations.push(format!(
                    "{}: slot {} must be empty, but the device has {n} child(ren) and no such slot",
                    e.device, e.slot
                ));
                return Ok(());
            }
            codesys::EmptySlotAction::Unplug(m) => out.notes.push(format!(
                "{}: slot {} ({m}) unplugged: it must be empty",
                e.device, e.slot
            )),
        }
        let opts = self.options();
        let calls = e.calls(&action, &opts, &opts.project_path(p));
        self.exec(stage, project, &calls, out)
    }

    fn read_back(
        &mut self,
        check: &Check,
        text: &str,
        out: &mut Outcome,
        labels: &mut Labels,
        tasks: &mut Vec<(String, Vec<String>)>,
        unset: &[(String, u32, String)],
    ) {
        let last = |p: &str| p.rsplit('/').next().unwrap_or(p).to_string();
        match check {
            Check::Done
            | Check::Compile
            | Check::ExportLibraries
            | Check::InstallLibraries
            | Check::DownloadLibraries
            | Check::LibrariesResolved
            | Check::BuildClean
            | Check::Connect
            | Check::Module(_)
            | Check::EmptySlot(_)
            | Check::IoCarrier
            | Check::Optional(_) => {}
            Check::Symbols(_) => {
                out.symbols = Some(codesys::parse_symbols(text).unwrap_or_default())
            }
            Check::SlotEmpty(e) => match e.action(text) {
                codesys::EmptySlotAction::Empty => {}
                codesys::EmptySlotAction::Unplug(m) => out.deviations.push(format!(
                    "{}: slot {} still holds {m} after unplug_device",
                    e.device, e.slot
                )),
                other => out.deviations.push(format!(
                    "{}: slot {} not read back as empty ({other:?})",
                    e.device, e.slot
                )),
            },
            Check::ModuleIdent { device, ident } => {
                let rows = codesys::parse_params(text);
                let got =
                    rows.iter()
                        .find(|(_, n, _)| n == "Module parameters")
                        .map(|(_, _, v)| {
                            v.trim_start_matches(['{', '['])
                                .split(',')
                                .next()
                                .unwrap_or("")
                                .trim()
                                .to_string()
                        });
                // A module of the robot simulator's own PN device has no `Module parameters`:
                // its first parameter is the module ({1, 1, 16#0000008F, 16#00000001, 12, ..}:
                // slot, subslot, module ident, submodule ident, ..).
                let own = || {
                    rows.iter().any(|(_, _, v)| {
                        v.trim_start().starts_with('{')
                            && v.trim_start_matches(['{', '['])
                                .split(',')
                                .take(4)
                                .any(|x| codesys::same_value(x.trim(), ident))
                    })
                };
                match got {
                    Some(g) if codesys::same_value(&g, ident) => {}
                    Some(g) => out
                        .deviations
                        .push(format!("{device}: module ident {g}, want {ident}")),
                    None if own() => {}
                    None => out
                        .warnings
                        .push(format!("{device}: no Module parameters to check the ident")),
                }
            }
            Check::Status => {
                if let Some(exe) = text.lines().find_map(|l| l.trim().strip_prefix("Exe:")) {
                    out.notes.push(format!("IDE {}", exe.trim()));
                }
            }
            Check::Installed(id) => match codesys::parse_installed(text) {
                Some(n) => {
                    self.names.insert(id.clone(), n);
                }
                None => out.missing.push(id.clone()),
            },
            Check::Ident { device, want } => match codesys::parse_ident(text) {
                Some(id) => {
                    if let Some(w) = want
                        && id != *w
                    {
                        out.deviations.push(format!(
                            "{device}: DeviceID ({}, '{}', '{}'), want ({}, '{}', '{}')",
                            id.ty, id.id, id.version, w.ty, w.id, w.version
                        ));
                    }
                    out.devices.push(last(device));
                }
                None => out
                    .deviations
                    .push(format!("{device}: no identification read back")),
            },
            Check::SubElements { device, param } => {
                labels.insert((device.clone(), *param), codesys::parse_sub_elements(text));
            }
            Check::Params { device, want } => {
                let rows = codesys::parse_params(text);
                for (id, v) in want {
                    match rows.iter().find(|(i, _, _)| i == id) {
                        None => out
                            .deviations
                            .push(format!("{device}: no parameter {id} read back")),
                        Some((_, n, got)) if codesys::norm_value(got) != codesys::norm_value(v) => {
                            out.deviations.push(format!(
                                "{device}: parameter {id} ({n}) reads back {got}, want {v}"
                            ))
                        }
                        Some(_) => {}
                    }
                }
            }
            Check::Members {
                device,
                param,
                want,
            } => {
                let subs = codesys::parse_sub_elements(text);
                for (m, v) in want {
                    if unset
                        .iter()
                        .any(|(d, p, x)| d == device && p == param && x == m)
                    {
                        continue;
                    }
                    let got = codesys::member_index(&subs, m)
                        .and_then(|i| subs.iter().find(|(j, _, _)| *j == i))
                        .map(|(_, _, g)| g.clone());
                    match got {
                        Some(g) if codesys::norm_value(&g) == codesys::norm_value(v) => {}
                        Some(g) => out.deviations.push(format!(
                            "{device}: parameter {param}.{m} reads back {g}, want {v}"
                        )),
                        None => out
                            .deviations
                            .push(format!("{device}: parameter {param}.{m} not read back")),
                    }
                }
            }
            Check::Libraries { ensure, auto } => {
                let have = codesys::parse_libraries(text);
                let has = |n: &str| have.iter().any(|h| h.eq_ignore_ascii_case(n));
                for l in ensure.iter().filter(|l| !has(l)) {
                    out.deviations
                        .push(format!("library {l} missing after adding it"));
                }
                for l in auto.iter().filter(|l| !has(l)) {
                    out.warnings.push(format!(
                        "library {l} (normally added with the device tree) is missing"
                    ));
                }
            }
            Check::Code(want) => {
                let have = codesys::parse_code(text);
                for (n, decl, imp) in want {
                    match have.iter().find(|(h, _, _)| h == n) {
                        None => out.deviations.push(format!("object {n} missing")),
                        Some((_, d, i)) => {
                            if codesys::norm_text(d) != codesys::norm_text(decl) {
                                out.deviations
                                    .push(format!("{n}: declaration differs after writing it"));
                            }
                            if let Some(want_i) = imp
                                && codesys::norm_text(i.as_deref().unwrap_or(""))
                                    != codesys::norm_text(want_i)
                            {
                                out.deviations
                                    .push(format!("{n}: implementation differs after writing it"));
                            }
                        }
                    }
                }
                out.objects = have.into_iter().map(|(n, _, _)| n).collect();
            }
            Check::Tasks(want) => {
                *tasks = codesys::parse_tasks(text);
                for w in want {
                    let Some((_, pous)) = tasks.iter().find(|(n, _)| n == &w.name) else {
                        out.deviations.push(format!("task {} missing", w.name));
                        continue;
                    };
                    for pou in &w.pous {
                        if !pous.iter().any(|p| codesys::same_pou(p, pou)) {
                            out.deviations
                                .push(format!("task {} does not call {pou}", w.name));
                        }
                    }
                    if w.exclusive {
                        for p in pous {
                            if !w.pous.iter().any(|x| codesys::same_pou(p, x)) {
                                out.deviations
                                    .push(format!("task {} also calls {p}", w.name));
                            }
                        }
                    }
                }
            }
            Check::BusCycle { device, task } => {
                let have = text
                    .lines()
                    .find_map(|l| l.trim().strip_prefix("Bus cycle task:"))
                    .map(str::trim);
                if have != Some(task.as_str()) {
                    out.deviations.push(format!(
                        "{device}: bus cycle task {}, want {task}",
                        have.unwrap_or("?")
                    ));
                }
            }
            Check::AppState => out.state = codesys::parse_state(text),
            Check::Values => out.values = codesys::parse_values(text).into_iter().collect(),
        }
    }

    /// A project went online: the gateway human step of an earlier run is done.
    fn gateway_works(&mut self) {
        for p in &mut self.report.projects {
            p.human_steps.retain(|h| h != GATEWAY_STEP);
        }
    }

    fn project_report(&mut self, name: &str) -> &mut ProjectReport {
        self.report
            .projects
            .iter_mut()
            .find(|r| r.name == name)
            .expect("one report per project")
    }

    /// Send a call whose failure does not matter (recorded; a dry run sends nothing).
    fn best_effort(&mut self, c: &Call) -> Option<ToolOutput> {
        if self.settings.dry_run {
            self.report
                .calls
                .push(format!("mcp {}: {}", self.cfg.codesys_mcp.url, c.show()));
            return None;
        }
        self.send(c).ok()
    }

    /// The head project closed in the PLC IDE without saving ([`codesys::discard_ide_edits`]),
    /// before a stage binds, saves, logs in to or recreates it: an edit made in the IDE during
    /// the review never reaches the head PLC. Best effort (a note either way).
    fn discard_ide_edits(&mut self, head: &Project, notes: &mut Vec<String>) {
        let call = codesys::discard_ide_edits(head, &self.options());
        match self.best_effort(&call) {
            Some(r) if !r.failed() => notes.push(format!(
                "{} closed in the PLC IDE without saving first (edits made there are discarded)",
                head.name
            )),
            Some(r) => notes.push(format!(
                "{} not closed in the PLC IDE: {}",
                head.name,
                r.text
                    .lines()
                    .rev()
                    .find(|l| !l.trim().is_empty())
                    .unwrap_or("no answer")
                    .trim()
            )),
            None if self.settings.dry_run => {}
            None => notes.push(format!(
                "{} not closed in the PLC IDE (no answer)",
                head.name
            )),
        }
    }

    /// `verify` of a project with a Symbol Configuration: every `require[]` symbol exported
    /// readable (`list_configured_symbols`), and the runtime's OPC UA port open as seen from
    /// the PLC host (skipped with a note if the PLC host does not answer).
    fn opc_ua_checks(
        &mut self,
        p: &Project,
        sc: &crate::project::SymbolConfig,
        symbols: Option<&[(String, String)]>,
        notes: &mut Vec<String>,
    ) -> Vec<CheckResult> {
        let mut v: Vec<CheckResult> = symbol_checks(&sc.require, symbols)
            .into_iter()
            .map(|(x, ok)| CheckResult {
                value: symbols.and_then(|s| {
                    s.iter()
                        .find(|(n, _)| n.eq_ignore_ascii_case(&x))
                        .map(|(_, a)| a.clone())
                }),
                expr: format!("symbol {x}"),
                expect: "read".into(),
                ok,
            })
            .collect();
        if !sc.support_opc_ua {
            return v;
        }
        let (ip, port) = (p.runtime.ip.clone(), p.runtime.opcua_port);
        match self.ssh(&opc_ua_probe(&ip, port)) {
            Ok(out) => {
                let open = out.lines().any(|l| l.trim() == "open");
                v.push(CheckResult {
                    expr: format!("opc.tcp://{ip}:{port}"),
                    expect: "open".into(),
                    value: Some(if open { "open" } else { "closed" }.into()),
                    ok: open,
                });
            }
            Err(e) => notes.push(format!("OPC UA port of {} not checked: {e}", p.name)),
        }
        v
    }

    fn preflight(&mut self, notes: &mut Vec<String>) -> Result<()> {
        for r in &mut self.report.projects {
            r.human_steps.clear();
        }
        let h = self.cfg.plc_host.clone();
        let out = self.ssh(&format!(
            "if test -x {} && sudo -n true; then echo ok; fi",
            sh(&h.setup)
        ))?;
        if !self.settings.dry_run && out.trim() != "ok" {
            return Err(failed(
                Stage::Preflight,
                3,
                format!(
                    "{}: {} is missing or sudo needs a password",
                    h.ssh_host, h.setup
                ),
            ));
        }
        let tools = self.list_tools()?;
        let caps = self.set_caps(&tools);
        let missing: Vec<&str> = codesys::TOOLS
            .iter()
            .copied()
            .filter(|t| !tools.iter().any(|x| x.name == *t))
            .collect();
        if !missing.is_empty() {
            return Err(failed(
                Stage::Preflight,
                3,
                format!(
                    "the CODESYS MCP server lacks tools plan2cell calls: {} (it has {} tools; \
                     Codesys-MCP-Master 0.19.0 expected)",
                    missing.join(", "),
                    tools.len()
                ),
            ));
        }
        if !self.settings.dry_run {
            notes.push(format!("{} tools", tools.len()));
            for m in caps.missing() {
                notes.push(format!(
                    "the server has no {m}; infra/windows/codesys-mcp-overlay adds it"
                ));
            }
        }
        let devices = match &self.manifest {
            Some(m) => m.device_ids(),
            None => vec![DeviceId {
                ty: 4102,
                id: "0000 0005".into(),
                version: "4.20.0.0".into(),
            }],
        };
        let calls = codesys::preflight(&devices, &self.options());
        let mut o = Outcome::default();
        self.exec(Stage::Preflight, None, &calls, &mut o)?;
        notes.append(&mut o.notes);
        if o.missing.is_empty() {
            return Ok(());
        }
        let mut steps: Vec<(String, Vec<Role>)> = Vec::new();
        let mut text = Vec::new();
        for d in &o.missing {
            text.push(format!("({}, '{}', '{}')", d.ty, d.id, d.version));
            let pre = self
                .manifest
                .as_ref()
                .and_then(|m| m.prerequisites.iter().find(|p| p.provides == *d));
            if let Some(p) = pre {
                let how = match (&p.how, &p.file) {
                    (Some(h), _) => format!("{}: {h}", p.id),
                    (None, Some(f)) => format!("{}: install {f} into the device repository", p.id),
                    _ => format!("{}: install the device description", p.id),
                };
                let roles = p
                    .needed_by
                    .iter()
                    .filter_map(|r| match r.as_str() {
                        "head_plc" => Some(Role::HeadPlc),
                        "robot_sim" => Some(Role::RobotSim),
                        _ => None,
                    })
                    .collect();
                steps.push((how, roles));
            }
        }
        if let Some(m) = self.manifest.clone() {
            for (how, roles) in &steps {
                for p in m.projects.iter().filter(|p| roles.contains(&p.role)) {
                    let r = self.project_report(&p.name);
                    if !r.human_steps.contains(how) {
                        r.human_steps.push(how.clone());
                    }
                }
            }
        }
        Err(failed(
            Stage::Preflight,
            3,
            format!(
                "device descriptions missing in the IDE's repository: {} (see human_steps)",
                text.join(", ")
            ),
        ))
    }

    fn runtimes(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let h = self.cfg.plc_host.clone();
        let (bucket, region) = match h.bucket_region() {
            Ok(x) => x,
            Err(_) if self.settings.dry_run => ("<bucket>".into(), "<region>".into()),
            Err(e) => return Err(e),
        };
        let mut setup = format!(
            "sudo {} --bucket {} --region {}",
            sh(&h.setup),
            sh(&bucket),
            sh(&region)
        );
        if self.tree.as_ref().is_some_and(|t| t.exists(OVERLAY)) {
            let dest = format!(
                "{}/docker-compose.robots.yml",
                h.compose_dir.trim_end_matches('/')
            );
            self.scp(OVERLAY, OVERLAY_TMP)?;
            self.ssh(&format!("sudo install -m 0644 {OVERLAY_TMP} {}", sh(&dest)))?;
            setup.push_str(&format!(" --robots-overlay {}", sh(&dest)));
        }
        self.ssh(&setup)?;
        let ps = self.ssh("sudo docker ps --format '{{.Names}}'")?;
        let want: Vec<String> = self
            .manifest()
            .ordered()
            .iter()
            .map(|p| p.runtime.container.clone())
            .collect();
        if self.settings.dry_run {
            return Ok(());
        }
        let running: Vec<&str> = ps.lines().map(str::trim).collect();
        let missing: Vec<&String> = want
            .iter()
            .filter(|c| !running.contains(&c.as_str()))
            .collect();
        if !missing.is_empty() {
            return Err(failed(
                Stage::Runtimes,
                7,
                format!(
                    "containers not running on {}: {}",
                    h.ssh_host,
                    missing
                        .iter()
                        .map(|s| s.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
            ));
        }
        notes.push(format!("running: {}", want.join(", ")));
        Ok(())
    }

    /// Run a project's calls; a failed tool is the project's error, anything else stops.
    fn exec_project(
        &mut self,
        stage: Stage,
        p: &Project,
        calls: &[Call],
        out: &mut Outcome,
    ) -> Result<bool> {
        match self.exec(stage, Some(p), calls, out) {
            Ok(()) => Ok(true),
            Err(e @ Error::Tool { .. }) => {
                out.errors.push(e.to_string());
                Ok(false)
            }
            Err(e) => Err(e),
        }
    }

    fn projects(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let m = self.manifest().clone();
        let caps = self.load_caps()?;
        if !self.settings.dry_run {
            for x in caps.missing() {
                notes.push(format!(
                    "the server has no {x}; infra/windows/codesys-mcp-overlay adds it"
                ));
            }
        }
        let opts = self.options();
        // A login left from an earlier session (a manual connect_to_device, an interrupted
        // run) makes create_project hang: log out first, head first (the likeliest one).
        let mut head_exists = false;
        for p in m.ordered().into_iter().rev() {
            let r = self.best_effort(&codesys::disconnect(p, &opts));
            if r.as_ref().is_some_and(|r| !r.failed()) {
                notes.push(format!("{}: logged out first", p.name));
                head_exists |= p.role == Role::HeadPlc;
            }
        }
        // Then edits made in the PLC IDE to the head project (open there for the review) are
        // dropped; create_project would refuse it with unsaved changes. Only a head that exists
        // (its logout passed): opening a missing project only fails.
        if let Some(head) = m
            .projects
            .iter()
            .find(|p| p.role == Role::HeadPlc && head_exists)
        {
            self.discard_ide_edits(head, notes);
        }
        let (mut rejected, mut deviating) = (Vec::new(), Vec::new());
        let mut exported = Vec::new();
        for p in m.ordered() {
            let tree = self.tree.as_ref().expect("checked by stage()");
            let calls = codesys::project(p, &|rel| tree.read(&format!("plc/{rel}")), &opts)?;
            let mut o = Outcome::default();
            let ran = self.exec_project(Stage::Projects, p, &calls, &mut o)?;
            if self.settings.dry_run {
                continue;
            }
            exported.append(&mut o.libraries);
            let rep = self.project_report(&p.name);
            rep.errors = o.errors;
            rep.warnings = o.warnings;
            rep.deviations = o.deviations;
            rep.human_steps.retain(|h| !projects_step(h));
            for h in o.human {
                if !rep.human_steps.contains(&h) {
                    rep.human_steps.push(h);
                }
            }
            notes.append(&mut o.notes);
            rep.built = ran && o.compiled && rep.errors.len() as u32 <= p.acceptance.build_errors;
            rep.deployed = None;
            rep.state = None;
            rep.verified = None;
            rep.checks.clear();
            for ob in &p.acceptance.objects {
                if !o.objects.contains(ob) {
                    rep.deviations.push(format!("object {ob} missing"));
                }
            }
            for d in &p.acceptance.devices {
                if !o.devices.contains(d) {
                    rep.deviations.push(format!("device {d} missing"));
                }
            }
            if let (Some(sc), true) = (&p.symbol_config, ran) {
                for (x, ok) in symbol_checks(&sc.require, o.symbols.as_deref()) {
                    if !ok {
                        rep.deviations
                            .push(format!("symbol {x} not exported readable"));
                    }
                }
            }
            rep.deviations.dedup();
            if !rep.built {
                rejected.push(p.name.clone());
            } else if !rep.deviations.is_empty() {
                deviating.push(p.name.clone());
            } else {
                notes.push(format!("{} built", p.name));
            }
        }
        if !exported.is_empty() {
            self.copy_libraries(&exported, notes);
        }
        if !rejected.is_empty() {
            return Err(failed(
                Stage::Projects,
                8,
                format!("not built: {} (errors in the report)", rejected.join(", ")),
            ));
        }
        if !deviating.is_empty() {
            return Err(failed(
                Stage::Projects,
                7,
                format!(
                    "acceptance failed: {} (deviations in the report)",
                    deviating.join(", ")
                ),
            ));
        }
        Ok(())
    }

    /// The libraries `export_project_libraries` stored on the IDE, next to the build's PLC code
    /// on the workspace (`plc/libraries/` with `libraries.json`, every project's merged): a cell
    /// without an archive, or a new cell from the same build, can install them too. Only the
    /// files the answers carry (`includeData`: `data`, up to the overlay's size budget per
    /// answer; past it `dataOmitted`) are copied; the list is written either way. A failed copy
    /// is a note (the projects are built).
    fn copy_libraries(&mut self, answers: &[String], notes: &mut Vec<String>) {
        let Some(Tree::Dir(d)) = &self.tree else {
            return;
        };
        let dir = d.join("plc").join(codesys::LIBRARIES_DIR);
        let ide = self.options().libraries_dir();
        let mut all: Vec<codesys::Library> = Vec::new();
        for a in answers {
            for l in codesys::exported_libraries(a).unwrap_or_default() {
                let same = |x: &codesys::Library| {
                    x.name == l.name && x.version == l.version && x.vendor == l.vendor
                };
                match all.iter_mut().find(|x| same(x)) {
                    Some(x) if x.data.is_none() && l.data.is_some() => {
                        x.data = l.data;
                        x.data_omitted = None;
                    }
                    Some(_) => {}
                    None => all.push(l),
                }
            }
        }
        if all.is_empty() {
            notes.push(format!(
                "libraries stored with the projects in {ide}; the answer lists none to copy to the \
                 workspace"
            ));
            return;
        }
        let r = (|| -> Result<(usize, Vec<String>)> {
            std::fs::create_dir_all(&dir).map_err(|e| Error::io(&dir, e))?;
            let (mut copied, mut left) = (0, Vec::new());
            for l in &all {
                let name = l.file.rsplit(['\\', '/']).next().unwrap_or("");
                let bytes = l.data.as_deref().and_then(crate::remote::base64_decode);
                match bytes {
                    Some(b) if !name.is_empty() && name != "." && name != ".." => {
                        let f = dir.join(name);
                        std::fs::write(&f, b).map_err(|e| Error::io(&f, e))?;
                        copied += 1;
                    }
                    _ => {
                        let f = if l.file.is_empty() { &l.name } else { &l.file };
                        left.push(match &l.data_omitted {
                            Some(why) => format!("{f} ({why})"),
                            None => f.clone(),
                        })
                    }
                }
            }
            let f = dir.join(codesys::LIBRARIES_JSON);
            let text = serde_json::to_string_pretty(&all).expect("libraries serialise") + "\n";
            std::fs::write(&f, text).map_err(|e| Error::io(&f, e))?;
            Ok((copied, left))
        })();
        notes.push(match r {
            Ok((copied, left)) if left.is_empty() => format!(
                "libraries: {} listed, {copied} file(s) copied to {}",
                all.len(),
                dir.display()
            ),
            Ok((copied, left)) => format!(
                "libraries: {} listed in {}, {copied} file(s) copied; not in the answer (still \
                 only in {ide} on the IDE): {}",
                all.len(),
                dir.join(codesys::LIBRARIES_JSON).display(),
                left.join(", ")
            ),
            Err(e) => format!("libraries not copied to the workspace: {e}"),
        });
    }

    fn deploy(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let m = self.manifest().clone();
        let opts = self.options();
        if opts.credentials.is_none() && !self.settings.dry_run {
            let c = &self.cfg.codesys_mcp;
            notes.push(format!(
                "${} / ${} not set: the MCP server's device login is used",
                c.device_user_env, c.device_password_env
            ));
        }
        // The head project as its file has it, not with edits made in the PLC IDE (bind saves).
        if let Some(head) = m.projects.iter().find(|p| p.role == Role::HeadPlc) {
            self.discard_ide_edits(head, notes);
        }
        let mut bad = Vec::new();
        for p in m.ordered() {
            let mut o = Outcome::default();
            let ran = self.exec_project(Stage::Deploy, p, &codesys::deploy(p, &opts), &mut o)?;
            if self.settings.dry_run {
                continue;
            }
            notes.append(&mut o.notes);
            if ran {
                self.gateway_works();
            }
            let rep = self.project_report(&p.name);
            let running = o
                .state
                .as_deref()
                .is_some_and(|s| s.to_ascii_lowercase().contains("run"));
            let ok = ran && running;
            rep.deployed = Some(ok);
            rep.state = o.state.clone();
            rep.verified = None;
            rep.errors.extend(o.errors);
            if ok {
                notes.push(format!("{} running on {}", p.name, p.runtime.ip));
            } else {
                if ran {
                    rep.errors.push(format!(
                        "application state {} after start",
                        o.state.as_deref().unwrap_or("unknown")
                    ));
                }
                bad.push(p.name.clone());
            }
        }
        // The head project last, logged out (the next `projects` would hang on a login): the
        // project the PLC IDE (NICE DCV) shows for a review before Start production.
        if let Some(head) = m.projects.iter().find(|p| p.role == Role::HeadPlc) {
            let call = codesys::open_for_review(head, &opts);
            let path = opts.project_path(head);
            match self.best_effort(&call) {
                Some(r) if !r.failed() => {
                    notes.push(format!("{} open in the PLC IDE ({path})", head.name))
                }
                Some(r) => notes.push(format!(
                    "{} not opened in the PLC IDE: {}",
                    head.name,
                    r.text
                        .lines()
                        .rev()
                        .find(|l| !l.trim().is_empty())
                        .unwrap_or("no answer")
                        .trim()
                )),
                None if self.settings.dry_run => {}
                None => notes.push(format!(
                    "{} not opened in the PLC IDE (no answer)",
                    head.name
                )),
            }
        }
        if bad.is_empty() {
            Ok(())
        } else {
            Err(failed(
                Stage::Deploy,
                7,
                format!("not running: {}", bad.join(", ")),
            ))
        }
    }

    fn verify(&mut self, notes: &mut Vec<String>) -> Result<()> {
        if self
            .cfg
            .kuka_vc
            .as_ref()
            .is_some_and(|k| !k.controllers.is_empty())
        {
            self.verify_vc(notes)?;
        }
        let m = self.manifest().clone();
        let opts = self.options();
        let deadline = Instant::now() + self.settings.verify_timeout;
        let mut pending: Vec<&Project> = m.ordered();
        let mut attempt = 0;
        loop {
            attempt += 1;
            let mut still = Vec::new();
            for p in pending {
                let mut o = Outcome::default();
                let ran =
                    self.exec_project(Stage::Verify, p, &codesys::verify(&m, p, &opts), &mut o)?;
                if self.settings.dry_run {
                    continue;
                }
                let mut checks = evaluate(
                    o.state.as_deref(),
                    &o.values,
                    &codesys::verify_checks(&m, p),
                );
                if let Some(sc) = &p.symbol_config {
                    checks.extend(self.opc_ua_checks(p, sc, o.symbols.as_deref(), notes));
                }
                let ok = ran && checks.iter().all(|c| c.ok);
                if ran {
                    self.gateway_works();
                }
                let rep = self.project_report(&p.name);
                rep.state = o.state.or(rep.state.take());
                rep.checks = checks;
                rep.verified = Some(ok);
                if !ran {
                    rep.errors.extend(o.errors);
                    rep.errors.dedup();
                }
                if !ok {
                    still.push(p);
                }
            }
            if still.is_empty() {
                notes.push(format!("verified after {attempt} attempt(s)"));
                return Ok(());
            }
            if Instant::now() + self.settings.verify_interval > deadline {
                let names: Vec<&str> = still.iter().map(|p| p.name.as_str()).collect();
                return Err(failed(
                    Stage::Verify,
                    7,
                    format!(
                        "checks failed after {attempt} attempt(s): {} (see checks)",
                        names.join(", ")
                    ),
                ));
            }
            std::thread::sleep(self.settings.verify_interval);
            pending = still;
        }
    }
}

impl Cell<'_> {
    fn vc_config(&self) -> Result<KukaVc> {
        match &self.cfg.kuka_vc {
            Some(k) if !k.controllers.is_empty() => Ok(k.clone()),
            _ => Err(Error::Config(
                "cell.toml has no [[kuka_vc.controller]] (see docs/CELL.md)".into(),
            )),
        }
    }

    fn vc_run(&mut self, c: &KukaController, cmd: &str) -> Result<String> {
        self.report.calls.push(format!("{}: {cmd}", c.access()));
        if self.settings.dry_run {
            return Ok(String::new());
        }
        let vc = self
            .vc
            .as_mut()
            .ok_or_else(|| Error::Config("no remote for the KUKA VC hosts".into()))?;
        vc.remote(c)?.run(cmd)
    }

    /// Run one command on each of these controllers' hosts at the same time: all are sent
    /// first ([`crate::remote::Remote::start`]), then each is waited for. Recorded like
    /// [`Cell::vc_run`]; the outputs in the order of `cmds`.
    fn vc_all(&mut self, cmds: &[(&KukaController, String)]) -> Vec<Result<String>> {
        let mut sent = Vec::new();
        for (c, cmd) in cmds {
            self.report.calls.push(format!("{}: {cmd}", c.access()));
            if self.settings.dry_run {
                sent.push(Ok(None));
                continue;
            }
            sent.push(match self.vc.as_mut() {
                Some(vc) => vc.remote(c).and_then(|r| r.start(cmd)).map(Some),
                None => Err(Error::Config("no remote for the KUKA VC hosts".into())),
            });
        }
        sent.into_iter()
            .zip(cmds)
            .map(|(p, (c, _))| match p? {
                None => Ok(String::new()),
                Some(p) => self
                    .vc
                    .as_mut()
                    .ok_or_else(|| Error::Config("no remote for the KUKA VC hosts".into()))?
                    .remote(c)?
                    .wait(p),
            })
            .collect()
    }

    fn vc_copy(&mut self, c: &KukaController, local: &Path, remote: &str) -> Result<()> {
        self.report
            .calls
            .push(format!("{}: copy {} {remote}", c.access(), local.display()));
        if self.settings.dry_run {
            return Ok(());
        }
        let vc = self
            .vc
            .as_mut()
            .ok_or_else(|| Error::Config("no remote for the KUKA VC hosts".into()))?;
        vc.remote(c)?.copy(local, remote)
    }

    fn controller_report(&mut self, c: &KukaController) -> &mut ControllerReport {
        let i = match self
            .report
            .controllers
            .iter()
            .position(|r| r.robot_ns == c.robot_ns)
        {
            Some(i) => i,
            None => {
                self.report.controllers.push(ControllerReport::default());
                self.report.controllers.len() - 1
            }
        };
        let r = &mut self.report.controllers[i];
        r.robot_ns = c.robot_ns.clone();
        r.instance = c.label().to_string();
        r.host = c.access().to_string();
        r
    }

    fn krl(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let k = self.vc_config()?;
        let plan = self.manifest().plan.plan_id.clone();
        // One directory per run (runs in one process, e.g. tests, must not share it).
        static RUNS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = RUNS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let stage_dir =
            std::env::temp_dir().join(format!("plan2cell-krl-{}-{n}", std::process::id()));
        let mut first_err = None;
        for c in &k.controllers {
            let r = if c.is_hook() {
                self.krl_hook(c, &plan, notes)
            } else {
                self.krl_one(&k, c, &plan, &stage_dir, notes)
            };
            if let Err(e) = r {
                let rep = self.controller_report(c);
                rep.deployed = Some(false);
                if !rep.errors.contains(&e.to_string()) {
                    rep.errors.push(e.to_string());
                }
                first_err.get_or_insert(e);
            }
        }
        let _ = std::fs::remove_dir_all(&stage_dir);
        first_err.map_or(Ok(()), Err)
    }

    fn krl_one(
        &mut self,
        k: &KukaVc,
        c: &KukaController,
        plan: &str,
        stage_dir: &Path,
        notes: &mut Vec<String>,
    ) -> Result<()> {
        let ns = c.robot_ns.clone();
        let tree = self.tree.as_ref().expect("checked by stage()");
        if !tree.exists(&krl::manifest_path(&ns)) {
            return Err(Error::Config(format!(
                "the build has no {} (build with [krl] enabled for {ns})",
                krl::manifest_path(&ns)
            )));
        }
        let b = Bundle::load(tree, &ns)?;
        if b.manifest.plan_id != plan {
            return Err(failed(
                Stage::Krl,
                7,
                format!(
                    "krl/{ns} is for plan {}, the build is plan {plan} (rebuild)",
                    b.manifest.plan_id
                ),
            ));
        }
        {
            let rep = self.controller_report(c);
            rep.plan_id = Some(b.manifest.plan_id.clone());
            rep.deployed = None;
            rep.verified = None;
            rep.changed.clear();
            rep.errors.clear();
            rep.checks.clear();
        }
        let p = krl::host_paths(k, &ns);
        let files = [
            (
                "p2c-kvc.py",
                krl::HELPER.to_string(),
                p.helper.clone(),
                "0755",
            ),
            (
                "p2c-headstub.py",
                krl::HEADSTUB.to_string(),
                p.headstub.clone(),
                "0755",
            ),
            (
                "kvc-t1-test.sh",
                krl::T1TEST.to_string(),
                p.t1test.clone(),
                "0755",
            ),
            ("bundle.json", b.json.clone(), p.bundle.clone(), "0644"),
        ];
        let mut install = Vec::new();
        for (name, text, dest, mode) in &files {
            let local = stage_dir.join(format!("{ns}-{name}"));
            if !self.settings.dry_run {
                std::fs::create_dir_all(stage_dir).map_err(|e| Error::io(stage_dir, e))?;
                std::fs::write(&local, text).map_err(|e| Error::io(&local, e))?;
            }
            let tmp = format!("/tmp/plan2cell-{ns}-{name}");
            self.vc_copy(c, &local, &tmp)?;
            install.push(format!(
                "sudo -n install -D -m {mode} {} {} && rm -f {}",
                sh(&tmp),
                sh(dest),
                sh(&tmp)
            ));
        }
        self.vc_run(c, &install.join(" && "))?;
        let mut extra = Vec::new();
        if let Some(s) = &c.sps_base {
            extra.push("--sps-base".to_string());
            extra.push(s.clone());
        }
        let out = self.vc_run(c, &krl::helper_cmd(k, c, "deploy", &extra))?;
        if self.settings.dry_run {
            return Ok(());
        }
        let a = krl::parse_answer(&out)?;
        let changed = krl::strings(&a.json["changed"]);
        let errors = krl::compile_errors(&a.json["errors"]);
        let msgs = krl::messages(&a.json["messages"]);
        let error = a.json["error"].as_str().unwrap_or("").to_string();
        let rep = self.controller_report(c);
        rep.changed = changed.clone();
        rep.errors = errors.clone();
        if !error.is_empty() {
            rep.errors.push(error.clone());
        }
        rep.messages = msgs;
        rep.deployed = Some(a.code == 0);
        let what = format!("{ns} on {} ({})", c.instance, c.access());
        match a.code {
            0 => {
                notes.push(format!(
                    "{what}: {} file(s) changed, {} unchanged{}",
                    changed.len(),
                    a.json["unchanged"],
                    krl::strings(&a.json["confirmed"])
                        .first()
                        .map(|_| ", messages confirmed")
                        .unwrap_or("")
                ));
                Ok(())
            }
            4 => Err(failed(
                Stage::Krl,
                8,
                format!("{what}: {} compile error(s): {}", errors.len(), {
                    let mut e = errors.clone();
                    e.truncate(5);
                    e.join("; ")
                }),
            )),
            3 => Err(failed(Stage::Krl, 3, format!("{what}: {error}"))),
            n => Err(failed(Stage::Krl, 7, format!("{what}: exit {n}: {error}"))),
        }
    }

    fn verify_vc(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let k = self.vc_config()?;
        let mut bad = Vec::new();
        for c in &k.controllers {
            if c.is_hook() {
                if !self.verify_hook_robot(Stage::Verify, c, None, notes)? {
                    bad.push(c.robot_ns.clone());
                }
                continue;
            }
            let out = self.vc_run(c, &krl::helper_cmd(&k, c, "verify", &[]))?;
            if self.settings.dry_run {
                continue;
            }
            let a = krl::parse_answer(&out)?;
            let checks = check_results(&a.json["checks"]);
            let ok = a.code == 0 && a.json["ok"].as_bool() == Some(true) && !checks.is_empty();
            let msgs = krl::messages(&a.json["messages"]);
            let errors = krl::compile_errors(&a.json["errors"]);
            let rep = self.controller_report(c);
            rep.checks = checks;
            rep.verified = Some(ok);
            rep.messages = msgs;
            if !errors.is_empty() {
                rep.errors = errors;
            }
            if ok {
                notes.push(format!("{} on {} verified", c.robot_ns, c.instance));
            } else {
                bad.push(c.robot_ns.clone());
            }
        }
        if bad.is_empty() {
            Ok(())
        } else {
            Err(failed(
                Stage::Verify,
                7,
                format!(
                    "KUKA VC checks failed: {} (see controllers)",
                    bad.join(", ")
                ),
            ))
        }
    }
}

/// The `checks` of a p2c-kvc / robot hook `verify` answer (`expect` and `value` are strings).
pub fn check_results(v: &serde_json::Value) -> Vec<CheckResult> {
    v.as_array()
        .map(|v| {
            v.iter()
                .map(|c| CheckResult {
                    expr: c["expr"].as_str().unwrap_or("").into(),
                    expect: c["expect"].as_str().unwrap_or("").into(),
                    value: c["value"].as_str().map(str::to_string),
                    ok: c["ok"].as_bool() == Some(true),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// A TCP connect to `ip:port` from the PLC host: prints `open` or `closed`.
pub fn opc_ua_probe(ip: &str, port: u16) -> String {
    format!(
        "if timeout 3 bash -c 'exec 3<>/dev/tcp/{ip}/{port}' 2>/dev/null; then echo open; else \
         echo closed; fi"
    )
}

/// Each required `Signature.variable` and whether `list_configured_symbols` has it readable
/// (no answer: none is).
pub fn symbol_checks(
    require: &[String],
    symbols: Option<&[(String, String)]>,
) -> Vec<(String, bool)> {
    require
        .iter()
        .map(|x| {
            let ok = symbols.is_some_and(|s| {
                s.iter()
                    .any(|(n, a)| n.eq_ignore_ascii_case(x) && codesys::readable(a))
            });
            (x.clone(), ok)
        })
        .collect()
}

/// The application state plus every check against the values `read_variables` returned.
pub fn evaluate(
    state: Option<&str>,
    values: &BTreeMap<String, String>,
    checks: &[Expect],
) -> Vec<CheckResult> {
    let mut out = vec![CheckResult {
        expr: "application_state".into(),
        expect: "run".into(),
        ok: state.is_some_and(|s| s.to_ascii_lowercase().contains("run")),
        value: state.map(str::to_string),
    }];
    for c in checks {
        let value = values.get(&c.expr).cloned();
        out.push(CheckResult {
            expr: c.expr.clone(),
            expect: c.expect.clone(),
            ok: value
                .as_deref()
                .is_some_and(|v| codesys::same_value(v, &c.expect)),
            value,
        });
    }
    out
}

mod hooks;
mod production;
pub mod operate;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shell_quoting() {
        assert_eq!(
            sh("/opt/plan2cell/plc-host/setup.sh"),
            "/opt/plan2cell/plc-host/setup.sh"
        );
        assert_eq!(sh("a b"), "'a b'");
        assert_eq!(sh("it's"), r"'it'\''s'");
        assert_eq!(sh(""), "''");
    }

    #[test]
    fn checks_against_values() {
        let values: BTreeMap<String, String> = [("A.x", "16#8024"), ("A.y", "FALSE")]
            .into_iter()
            .map(|(a, b)| (a.to_string(), b.to_string()))
            .collect();
        let e = |x: &str, v: &str| Expect {
            expr: x.into(),
            expect: v.into(),
        };
        let c = evaluate(
            Some("run"),
            &values,
            &[e("A.x", "16#8024"), e("A.y", "TRUE"), e("A.z", "TRUE")],
        );
        assert_eq!(
            c.iter().map(|c| c.ok).collect::<Vec<_>>(),
            [true, true, false, false]
        );
    }

    #[test]
    fn gateway_answers() {
        assert!(no_gateway(
            "SCRIPT_ERROR: RuntimeError: No gateway is configured in this CODESYS (Tools > Options > Gateway)."
        ));
        assert!(no_gateway(
            "Unable to connect to the device for item 'Device.Application' due to the following reason:\nGateway not configured properlyDo you want to continue with the items?"
        ));
        assert!(!no_gateway("Invalid user authentication on the target"));
    }

    #[test]
    fn transient_answers() {
        assert!(transient(
            "Operation failed. Output:\n\nError: Read error: [Errno 32] The process cannot access the file 'C:\\x.command.json' because it is being used by another process."
        ));
        assert!(!transient("Operation failed. Output:\nError in plug"));
    }

    #[test]
    fn first_login_refusals() {
        assert!(login_refused(
            "Operation failed. Output:\nInvalid user authentication on the target"
        ));
        assert!(!login_refused("Operation failed. Output:\ntimeout"));
    }
}
