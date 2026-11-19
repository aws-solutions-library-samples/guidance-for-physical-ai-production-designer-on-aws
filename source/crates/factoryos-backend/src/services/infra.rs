//! The Virtual Cell: the plan2cell cell's instances (PLC host, Windows CODESYS IDE, one KUKA VC
//! host per controller), through the cell-control Lambda (`plan2cell/infra/lib/cell-control-stack.ts`), so this
//! machine needs `lambda:InvokeFunction` on it and no EC2 or SSM permissions of its own.
//!
//! * The function is `FACTORY_OS_CELL_CONTROL_FUNCTION` (and `FACTORY_OS_CELL_CONTROL_REGION`), from
//!   the environment or `/etc/factoryos/cell-control.env` (written by the test server's user data /
//!   `connect-cell.sh`; the systemd unit reads it as an `EnvironmentFile` too). Unset: the status
//!   says unavailable and every action is `503`.
//! * It is invoked with the `aws` CLI (`aws lambda invoke ... --cli-binary-format raw-in-base64-out`),
//!   arguments only (no shell), with timeouts; no AWS SDK in the backend.
//! * The status ([`InfraStatus`]) is published on [`keys::INFRA_STATUS`] (also a queryable) and
//!   read again about every 10 s while a viewer asks for it (`GET`), a start/stop is under way,
//!   or a bring-up / production job runs.
//! * Stop is passed through (an EC2 stop: nothing is deleted); it remembers whether the cell was in
//!   production ([`InfraStatus::stopped_in_production`]). Start (Start cell): the Lambda starts
//!   each instance on its own (a stopped one without capacity for its type starts as a fallback
//!   type of its role; [`InfraStatus::start`] has each instance's result). A start of the whole
//!   cell then (unless `bringUp: false`) waits until every instance runs and is SSM-online
//!   ([`Settings::start_timeout`], about 10 min, per-instance progress in `start.message`), runs
//!   the bring-up below and, when the request says `production: true` or the cell was in
//!   production when it was stopped, Start production: one click to a producing cell. Stop cell
//!   ends the wait. Bring-up waits until every instance runs and is SSM-online,
//!   runs the Lambda's `bringup` (on every KUKA VC host: the health check, `kvc-cell-setup.sh up` only
//!   if it fails, then `kvc-ack.sh`) and polls `bringupResult` (done when every host is, ok only if
//!   every host passed). Production (Start production, Restart PLC runtime, auto-restart) runs
//!   `plan2cell cell start <export> --restart-runtime --pulses 16 --cell-config <cell.toml>
//!   --json`, as `p2c-cell start` does, with the CODESYS MCP token from Secrets Manager in the
//!   child's environment only, on the export the head runs ([`head_export`]): of the PLAN_ID it
//!   runs (Run Control Code's, the last production start's, or the one its PLAN_ID guard named;
//!   kept in the auto-restart file across backend restarts: the export last deployed if it is of
//!   that plan, else the newest of it); with that PLAN_ID unknown, the export last deployed (its
//!   `plan2cell-cell-report.json` has a successful `deploy` stage, the newest report, as
//!   `p2c-cell` finds it), else the newest. If the head's PLAN_ID guard still refuses, the export
//!   of the PLAN_ID it names is started once more, without `--restart-runtime` (none: `head runs
//!   <id>; no export of it: deploy one (Run Control Code)`). `build` in the request: that export,
//!   no retry. One job (bring-up, production, Run Control Code) at a time.
//! * Run Control Code (`POST /api/v1/infra/deploy`, default the newest export): `plan2cell cell
//!   projects`, `deploy`, `krl` (only with `[[kuka_vc.controller]]` in the cell.toml; alone it
//!   exits 3 without), then `start --restart-runtime --pulses 16`, each `DIR --cell-config
//!   <cell.toml> --json` (each command runs its one stage only), the first failure stops it
//!   ([`InfraStatus::deploy`]: the stage and a message, live). Once `deploy` passed, the export's
//!   PLAN_ID is the one the head runs (`deployedPlanId`; a failed `deploy` makes it unknown) and
//!   the export is [`InfraStatus::deployed_export`]; the cell is out of production until a start
//!   succeeds. Auto-restart pauses while it runs and waits 10 min after it.
//! * Deploy only (`{"start": false}`): the same without `start`. What it leaves: the head PLC and
//!   the robot PLCs run the new application (`deploy` downloads, creates the boot application and
//!   starts it; the stage fails unless each one runs), the KUKA VCs hold the new KRL, and the head
//!   sits in `CS_INIT` / `CS_RECOVERY` waiting for the operator pulses only Start production sends:
//!   deployed, not producing ([`InfraStatus::in_production`] `false`, auto-restart idle). The
//!   head project stays open in the PLC IDE's CODESYS (`plan2cell cell deploy` opens it last,
//!   logged out: [`InfraStatus::ide_project`]). The demo runtimes are not restarted: Start
//!   production does that.
//! * The CODESYS demo runtimes (no licence: each stops ~2 h after it starts) are read with
//!   `plan2cell cell runtime <export> --cell-config <cell.toml> --json` (read-only: docker inspect
//!   and the runtime logs on the PLC host) about every minute while a viewer asks or auto-restart
//!   is on ([`InfraStatus::plc_runtime`]). `<export>`: the last production start's (else the
//!   newest of the deployed PLAN_ID, else the newest), so only the containers a restart restarts
//!   count; without any export, every `codesys-*` container of the host.
//! * Auto-restart (per cell, `<cell>/plan2cell/auto-restart.json`), only while the cell is in
//!   production (the last production start succeeded, Stop cell has not run since): when the
//!   first runtime stops in less than the threshold (300 s), the head's runtime is not running, or
//!   docker restarted it by its restart policy (its `RestartCount` went up: a stop of its own; a
//!   `docker restart` from elsewhere does not count), the production job with `matchDeployed`.
//!   Never while another job runs, at most one attempt per 10 min after any production job, and a
//!   refusal of the PLAN_ID guard or 3 failed attempts in a row hold it (blocked) until it is
//!   switched on again or a production start succeeds.
//! * One cell, several backends (a test server and personal workspaces): `FACTORY_OS_CELL_ROLE`
//!   (`/etc/factoryos/cell-control.env`). The one `supervisor` (the default) runs auto-restart and
//!   the production resume of Start cell; an `operator` never does (`PUT` auto-restart on is
//!   `409`). User actions (Start / Stop cell, bring-up, production, Stop production, Run Control
//!   Code, Deploy only) and auto-restart hold the cell's lease in the cell-control Lambda (its
//!   `lease` action: `acquire`, `renew` every [`Settings::lease_ttl`] / 3 while the job runs,
//!   `release` with `lastChange`), named `FACTORY_OS_CELL_HOLDER` (else `FACTORY_OS_WORKSPACE`):
//!   another backend's action meanwhile is `409 cell busy: <holder> is <purpose> since hh:mm`. A
//!   Lambda without leases (an unknown action) or a lease error: the action runs as before. The
//!   supervisor reads the lease (at most every 30 s): another holder skips auto-restart (no
//!   failure counted); another backend's production change after its own production start takes
//!   the cell out of its production (`production changed by <holder> at hh:mm; auto-restart
//!   idle`). Open PLC IDE switches the IDE's project only while the lease is free or this backend's.
//! * The power of the whole cell (`GET` / `POST` [`api::INFRA_CELL_POWER`], [`InfraStatus::cell_power`]):
//!   the Lambda's `power_status` replaces `status` in every read (one call for both; an older Lambda
//!   without it: `status`, and no power, asked again after [`POWER_RETRY`]). Start virtual cell
//!   (`on`): the Lambda starts every instance and carries the boot, the KUKA bring-up and the
//!   readiness checks on by itself (its EventBridge tick), so the request returns at once and the
//!   phases show in the status; with `production` (or, on the supervisor, a cell that was in
//!   production when it was stopped) this backend starts production once the cell is ready. Stop
//!   virtual cell (`off`): `409` in production or during a job unless `force`; the Lambda refuses too
//!   while another backend reports production. Both the supervisor and operators may. This backend
//!   tells the Lambda it uses the cell (`touch`, every [`TOUCH_EVERY`] while a viewer asks, a job
//!   runs or the cell is in production): the Lambda's idle stop counts from the last one.
//!   Auto-restart pauses while the cell is off, stopping or booting, and a cell found off (stopped
//!   by another backend or the idle stop) leaves production (remembered for the next start).
//! * A workspace's own virtual cell (`FACTORY_OS_CELL_CONTROL_QUALIFIER=ws-<alias>`, [`CellBinding::Own`]):
//!   the function (`factoryos-cell`) is invoked with `--qualifier ws-<alias>`, and the Lambda takes
//!   the cell from the alias. Every read is its `cell_status` (the cell, its power, the wiring of the
//!   launch, cost, images, limits). `GET` / `POST` [`api::INFRA_CELLS`] list / create the cell,
//!   `DELETE` [`api::INFRA_CELL_ITEM`] deletes it (the id typed again). Such a cell is immutable:
//!   Stop terminates its instances (the Lambda archives the PLC data first), Start launches fresh
//!   ones from the latest images. Once it is `ready`, the backend writes the launch's wiring
//!   ([`cell_wiring`]: `cell.toml` with `[address_map]`, the live bridge's config, `dcv.env`) and
//!   restores the last deployed control code ([`CellPowerRequest::restore`]: `auto` runs `krl`
//!   (and `start` if it was in production) when the Lambda restored the head's archived boot
//!   applications of that export, else Deploy only; `full`: Deploy only; `none`), then reports it
//!   (`restore_report`). The shared cell ([`CellBinding::Shared`]) works as before.
//! * An own cell's off mode ([`VirtualCell::off_mode`], `PATCH` [`api::INFRA_CELL_ITEM`],
//!   [`UpdateCellRequest`]): `terminate` (the default, as above) or `stop`: Stop stops the
//!   instances (the Lambda archives first in both modes), Start resumes them
//!   ([`CellPowerRequest::fresh`]: `true` Start fresh; the Lambda launches fresh anyway when one is
//!   due, [`VirtualCell::resume`]). A resume (its power session's `resume`) keeps the launch: no
//!   restore (the code is on the disks); if the cell was in production when it stopped, `start`
//!   only, then `krl` + `start`, then Run Control Code (the restore's mode `resume`). Its `start`
//!   runs without `--restart-runtime` ([`fresh_boot`]: the start is a fresh boot).
//! * Cell templates (docs/tasks/cell-templates.md §13): an own cell boots from a template the Lambda
//!   resolves (`cell_status.template`: its capabilities, consumer block and host semantics). `GET`
//!   [`api::INFRA_TEMPLATES`] lists what the workspace may create (the Lambda's `templates`, cached
//!   30 s); create / update pass `template` and `params` through (the Lambda is authoritative). The
//!   cell's capabilities gate the smartPADs, DCV (Open PLC IDE), the bring-up action, auto-restart
//!   (`runtime.codesys`) and the KUKA VC operations (the template's `consumer.ops`, never looser than
//!   plan2cell's own list for its KSS ops); the restore after a fresh launch is not gated (yet: the
//!   contract's eks-codesys-kuka has no `restore`, and EKS cells are restored today). A cell without a template (an older Lambda, templates off, the shared cell)
//!   works exactly as before. The head PLC and the IDE are found by their semantic instead of their
//!   role names only with `FACTORY_OS_CELL_SEMANTIC_HOSTS=1` (it makes auto-restart see an EKS
//!   cell's `n1/plcHost`).
//! * Engine 2 (docs/tasks/cell-templates.md E7-E10): a template whose head and/or robots are `pod@1`
//!   images driven through `factoryos/hook@1` ([`CellTemplateRef::hook`]). The wiring's files say
//!   `driver = "hook"` ([`cell_wiring::render_hook_cell_toml`]) and plan2cell drives the hooks
//!   (deploy check `hook globals`, `krl`, `start`, `kvc` ops `hook op`, `runtime` `hook health`);
//!   here: Run Control Code runs the template's `consumer.deploy.stages` only, a hook head needs no
//!   CODESYS MCP token (and has no PLC IDE to reset or open), its hosts are found by their semantics,
//!   and hook robots offer no smartPAD.

use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant, SystemTime};

use anyhow::Context as _;
use axum::body::Bytes;
use axum::extract::{Path as UrlPath, State};
use axum::http::StatusCode;
use axum::routing::{delete, get, post};
use serde::{Deserialize, Serialize};
use axum::{Json, Router};
use factoryos_protocol::infra::{
    AUTO_RESTART_BACKOFF_S, AUTO_RESTART_MAX_FAILURES, AUTO_RESTART_THRESHOLD_S, AutoRestartSettings, AutoRestartStatus, BringupRequest, CODESYS_DEMO_S,
    CellCost, CellImage, CellKind, CellLease, CellLifecycle, CellPower, CellPowerRequest, CellRestore, CellResume, CellRole, CellStartStatus, CellsList,
    OFF_MODE_STOP, OFF_MODE_TERMINATE, OFF_MODES, UpdateCellRequest, IDES, RUNTIME_EC2, RUNTIMES, CellNetwork, MAX_NETWORKS, MAX_NETWORK_ROBOTS,
    CreateCellRequest, DEPLOY_STAGES, DeleteCellRequest, DeployRequest, PowerHost, PowerState, RESTORE_MODES, VirtualCell, DeployStatus, ExportRef, IdeProject, InfraInstance, InfraStatus, InstanceStart, JobState, PlcRuntime,
    ProductionRequest, ROLE_PLC_HOST, ROLE_WINDOWS_IDE, ROLES, RuntimeBrief, StartStopRequest, deploy_stage_label, known_role, role_label, utc_hhmm,
    CONSUMER_LEVEL, CellTemplateInfo, CellTemplateRef, CellTemplates, KvcRequest, RUNTIME_EKS, TEMPLATE_AND_LEGACY_FIELDS, TEMPLATE_EC2_CODESYS_KUKA,
    TEMPLATE_EKS_CODESYS_KUKA, TemplateSelector, valid_param_name, valid_template_id, valid_template_version,
};
use factoryos_protocol::deploy_check::{DeployCheck, DeployFixRequest, DeployFixResult};
use factoryos_protocol::{api, keys};
use serde_json::{Value, json};
use axum::response::{IntoResponse as _, Response};
use tokio::io::{AsyncBufReadExt as _, AsyncReadExt as _, BufReader};
use tokio::process::Command;

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::hardware::{HardwareService, TempFile};
use crate::services::cell_wiring::{self, WireFiles, Wiring};
use crate::services::{cells, codegen};

/// The env file the test server writes (see the module docs).
pub const CELL_CONTROL_ENV: &str = "/etc/factoryos/cell-control.env";
/// The CODESYS MCP bearer token (as `p2c-cell` reads it).
pub const MCP_TOKEN_SECRET: &str = "plan2cell/codesys-mcp-token";
/// Deploy only's outcome (after the build and PLAN_ID; [`ide_outcome`] follows).
const DEPLOYED_NOT_STARTED: &str = "deployed, not started: the head PLC and the robots run the new code, the head waits in \
CS_INIT / CS_RECOVERY for Start production";
/// `[codesys_mcp] windows_project_dir` when the cell.toml has none (plan2cell's default).
const WINDOWS_PROJECT_DIR: &str = r"C:\ProgramData\plan2cell\projects\{cell}";
/// The variable `[codesys_mcp] token_env` of the cell.toml names.
const MCP_TOKEN_ENV: &str = "PLAN2CELL_CODESYS_MCP_TOKEN";
/// Open PLC IDE on the CODESYS MCP bridge (plan2cell/infra/windows/mcp-bridge.cjs), next to the
/// cell.toml's `[codesys_mcp] url`.
const SHOW_IDE_PATH: &str = "/show-ide";
/// The bridge answers at once (it opens a project in the background).
const SHOW_IDE_TIMEOUT: Duration = Duration::from_secs(20);
/// A status younger than this is answered as it is.
const FRESH: Duration = Duration::from_secs(5);
/// A viewer that asked within this keeps the polling on.
const INTEREST: Duration = Duration::from_secs(30);
/// A start / stop counts as under way at most this long.
const TRANSITION: Duration = Duration::from_secs(600);
const TAIL: usize = 1500;
/// The auto-restart setting and its state, in the cell's folder.
pub const AUTO_RESTART_FILE: &str = "plan2cell/auto-restart.json";
/// How often this backend tells the Lambda it uses the cell (its idle stop counts from the last).
pub const TOUCH_EVERY: Duration = Duration::from_secs(5 * 60);
/// A Lambda without power actions is asked again after this (it may have been updated).
pub const POWER_RETRY: Duration = Duration::from_secs(10 * 60);
/// A Start virtual cell's production follow-up gives up after this.
const POWER_FOLLOW: Duration = Duration::from_secs(50 * 60);
/// A PLC host that booted less than this ago runs fresh CODESYS runtimes (a new 2 h demo period
/// each): Run Control Code's `start` does not restart them (no `--restart-runtime`).
const FRESH_BOOT_S: f64 = 30.0 * 60.0;
/// The Lambda's `templates` is read again after this.
const TEMPLATES_FRESH: Duration = Duration::from_secs(30);
/// The Lambda's `restore_report` takes strings up to this many characters.
const RESTORE_REPORT_MAX: usize = 500;

type HttpError = (StatusCode, String);

mod deploy_check;
pub use deploy_check::fix as deploy_fix;

/// Where status and log lines go: the Zenoh bus, or a test's recorder.
pub trait Sink: Send + Sync {
    fn status(&self, status: &InfraStatus);
    fn log(&self, line: String);
}

impl Sink for Bus {
    fn status(&self, status: &InfraStatus) {
        self.put_json(keys::INFRA_STATUS, status);
    }

    fn log(&self, line: String) {
        Bus::log(self, line);
    }
}

/// What the service runs and how long it waits.
#[derive(Clone, Debug)]
pub struct Settings {
    /// The cell-control Lambda (name or ARN); `None`: unavailable.
    pub function: Option<String>,
    /// Its qualifier (`FACTORY_OS_CELL_CONTROL_QUALIFIER`, `ws-<alias>`): the workspace's own cell
    /// ([`CellBinding::Own`]); `None`: the shared cell.
    pub qualifier: Option<String>,
    /// Where an own cell's wiring is written ([`cell_wiring`]; `None`: nowhere).
    pub wire: Option<WireFiles>,
    pub region: Option<String>,
    /// The `aws` CLI.
    pub aws: PathBuf,
    pub mcp_token_secret: String,
    /// How often the status is read while something needs it.
    pub poll: Duration,
    /// Until the instances run and are SSM-online (a boot takes a few minutes).
    pub wait_timeout: Duration,
    /// Start cell: until every instance runs and is SSM-online, before the bring-up.
    pub start_timeout: Duration,
    /// The bring-up script (an `up` that restarts both controllers takes ~10 min).
    pub bringup_timeout: Duration,
    /// `plan2cell cell start` (its own `--timeout` is 300 s; restarting runtimes adds a little).
    pub production_timeout: Duration,
    /// `plan2cell cell projects` (creates and builds every CODESYS project: about 15 min).
    pub projects_timeout: Duration,
    /// `plan2cell cell deploy` and `krl`.
    pub stage_timeout: Duration,
    /// One `aws` call (and one `plan2cell cell runtime`).
    pub call_timeout: Duration,
    /// How often the CODESYS runtimes are read while a viewer asks or auto-restart is on.
    pub runtime_poll: Duration,
    /// This backend's part in the cell (`FACTORY_OS_CELL_ROLE`).
    pub role: CellRole,
    /// Its name in the cell's lease.
    pub holder: String,
    /// The supervisor's name, shown by an operator (`FACTORY_OS_CELL_SUPERVISOR`).
    pub supervisor: Option<String>,
    /// The lease's time to live (renewed at a third of it while the job runs).
    pub lease_ttl: Duration,
    /// A lease status read within this is used as it is.
    pub lease_cache: Duration,
    /// Settings ignored or taken differently (logged at start).
    pub warnings: Vec<String>,
    /// `FACTORY_OS_CELL_SEMANTIC_HOSTS=1`: a cell with a template finds its head PLC and IDE by
    /// their semantic (`cell_status.template.semantics`), not by the EC2 role names (default off:
    /// the role names, as before; with it, auto-restart reads an EKS cell's head).
    pub semantic_hosts: bool,
}

/// `FACTORY_OS_CELL_ROLE`: absent, a supervisor; not `supervisor` / `operator`, an operator (no
/// auto action from a misspelt setting), with a warning.
fn cell_role(value: Option<&str>) -> (CellRole, Option<String>) {
    match value.map(str::trim).filter(|v| !v.is_empty()) {
        None => (CellRole::Supervisor, None),
        Some(v) => match CellRole::parse(v) {
            Some(role) => (role, None),
            None => (
                CellRole::Operator,
                Some(format!("FACTORY_OS_CELL_ROLE {v:?} is neither supervisor nor operator: an operator (no auto-restart)")),
            ),
        },
    }
}

impl Settings {
    /// From the environment and [`CELL_CONTROL_ENV`] (`FACTORY_OS_CELL_CONTROL_ENV` overrides its
    /// path); `FACTORY_OS_AWS` is the `aws` CLI, `FACTORY_OS_CODESYS_MCP_SECRET` the token's secret.
    pub fn from_env() -> Self {
        let file = std::env::var_os("FACTORY_OS_CELL_CONTROL_ENV").map_or_else(|| PathBuf::from(CELL_CONTROL_ENV), PathBuf::from);
        let from_file = std::fs::read_to_string(&file).map(|text| parse_env_file(&text)).unwrap_or_default();
        let var = |name: &str| {
            std::env::var(name)
                .ok()
                .filter(|v| !v.trim().is_empty())
                .or_else(|| from_file.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone()))
                .map(|v| v.trim().to_owned())
        };
        let (role, warning) = cell_role(var("FACTORY_OS_CELL_ROLE").as_deref());
        let holder = var("FACTORY_OS_CELL_HOLDER")
            .or_else(|| var("FACTORY_OS_WORKSPACE"))
            .or_else(|| std::fs::read_to_string("/etc/hostname").ok().map(|h| h.trim().to_owned()))
            .filter(|h| !h.is_empty() && h.len() <= 64 && h.chars().all(|c| c.is_ascii_alphanumeric() || "._-".contains(c)))
            .unwrap_or_else(|| Self::default().holder);
        let qualifier = var("FACTORY_OS_CELL_CONTROL_QUALIFIER");
        let region = var("FACTORY_OS_CELL_CONTROL_REGION").or_else(|| var("AWS_REGION")).or_else(|| var("AWS_DEFAULT_REGION"));
        let wire = qualifier.as_ref().map(|_| WireFiles {
            dir: var("FACTORY_OS_CELL_WIRE_DIR").map_or_else(|| PathBuf::from(cell_wiring::WIRE_DIR), PathBuf::from),
            cell_template: var("FACTORY_OS_CELL_WIRE_CELL_TEMPLATE").map_or_else(|| PathBuf::from(cell_wiring::CELL_TEMPLATE), PathBuf::from),
            bridge_template: var("FACTORY_OS_CELL_WIRE_BRIDGE_TEMPLATE").map_or_else(|| PathBuf::from(cell_wiring::BRIDGE_TEMPLATE), PathBuf::from),
            region: region.clone(),
        });
        Self {
            function: var("FACTORY_OS_CELL_CONTROL_FUNCTION"),
            qualifier,
            wire,
            region,
            aws: var("FACTORY_OS_AWS").map_or_else(|| PathBuf::from("aws"), PathBuf::from),
            mcp_token_secret: var("FACTORY_OS_CODESYS_MCP_SECRET").unwrap_or_else(|| MCP_TOKEN_SECRET.to_owned()),
            role,
            holder,
            supervisor: var("FACTORY_OS_CELL_SUPERVISOR").filter(|s| !s.is_empty()),
            warnings: warning.into_iter().collect(),
            semantic_hosts: var("FACTORY_OS_CELL_SEMANTIC_HOSTS").as_deref() == Some("1"),
            ..Self::default()
        }
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            function: None,
            qualifier: None,
            wire: None,
            region: None,
            aws: PathBuf::from("aws"),
            mcp_token_secret: MCP_TOKEN_SECRET.to_owned(),
            poll: Duration::from_secs(10),
            wait_timeout: Duration::from_secs(15 * 60),
            start_timeout: Duration::from_secs(10 * 60),
            bringup_timeout: Duration::from_secs(35 * 60),
            production_timeout: Duration::from_secs(20 * 60),
            projects_timeout: Duration::from_secs(45 * 60),
            stage_timeout: Duration::from_secs(30 * 60),
            call_timeout: Duration::from_secs(120),
            runtime_poll: Duration::from_secs(60),
            role: CellRole::Supervisor,
            holder: "factoryos".to_owned(),
            supervisor: None,
            lease_ttl: Duration::from_secs(180),
            lease_cache: Duration::from_secs(30),
            warnings: Vec::new(),
            semantic_hosts: false,
        }
    }
}

/// `KEY=VALUE` lines (systemd EnvironmentFile style: `#` comments, optional quotes).
pub(crate) fn parse_env_file(text: &str) -> Vec<(String, String)> {
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| {
            let (k, v) = l.strip_prefix("export ").unwrap_or(l).split_once('=')?;
            let v = v.trim();
            let v = v.strip_prefix('"').and_then(|v| v.strip_suffix('"')).or_else(|| v.strip_prefix('\'').and_then(|v| v.strip_suffix('\''))).unwrap_or(v);
            Some((k.trim().to_owned(), v.to_owned()))
        })
        .collect()
}

/// Which cell the backend's cell control acts on.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CellBinding {
    /// The shared `plan2cell` cell (`plan2cell-cell-control`).
    Shared { function: String },
    /// The workspace's own cell: `function:qualifier` (`factoryos-cell:ws-<alias>`), cell id `qualifier`.
    Own { function: String, qualifier: String },
}

impl CellBinding {
    pub fn kind(&self) -> CellKind {
        match self {
            Self::Shared { .. } => CellKind::Shared,
            Self::Own { .. } => CellKind::Own,
        }
    }

    pub fn function(&self) -> &str {
        match self {
            Self::Shared { function } | Self::Own { function, .. } => function,
        }
    }

    pub fn qualifier(&self) -> Option<&str> {
        match self {
            Self::Shared { .. } => None,
            Self::Own { qualifier, .. } => Some(qualifier),
        }
    }
}

impl Settings {
    /// The binding (`None`: no function).
    pub fn binding(&self) -> Option<CellBinding> {
        let function = self.function.clone()?;
        Some(match &self.qualifier {
            Some(qualifier) => CellBinding::Own { function, qualifier: qualifier.clone() },
            None => CellBinding::Shared { function },
        })
    }

    fn own(&self) -> bool {
        self.function.is_some() && self.qualifier.is_some()
    }
}

/// A Lambda alias as the provisioner names them (`ws-<alias>`; not all digits).
fn valid_qualifier(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 128
        && !name.chars().all(|c| c.is_ascii_digit())
        && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_'))
}

/// A function name or ARN, as Lambda allows them.
fn valid_function(name: &str) -> bool {
    !name.is_empty() && name.len() <= 170 && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | ':' | '.' | '$'))
}

pub struct InfraService {
    sink: Arc<dyn Sink>,
    settings: Settings,
    config: Arc<ConfigStore>,
    hardware: Arc<HardwareService>,
    plan2cell: Option<PathBuf>,
    plan2cell_lookup: Option<fn() -> Option<PathBuf>>,
    inner: Mutex<Inner>,
    /// One status read at a time.
    refreshing: tokio::sync::Mutex<()>,
    /// One runtime read at a time.
    runtime_reading: tokio::sync::Mutex<()>,
    /// One writer of the auto-restart file at a time.
    auto_file: Mutex<()>,
    /// The CODESYS MCP bearer token, read once for [`InfraService::show_ide`] (cleared on a `401`).
    mcp_token: Mutex<Option<String>>,
    /// The head as fos-live-bridge last read it (`factoryos/cell/state`, `plan2cell.head`): auto-restart's
    /// [`HeadLive`] trigger.
    head: Mutex<Option<HeadLive>>,
}

/// The head PLC's state machine as fos-live-bridge reads it over OPC UA.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HeadLive {
    /// `CS_RUNNING`, `CS_ERROR`, ...
    pub state: String,
    /// `ERR_DRIVES_LOST`, ... (`ERR_NONE`: none).
    pub error: Option<String>,
    pub cycles: u64,
    /// When the head entered this state (Unix seconds; as this backend saw it).
    pub since_s: f64,
    /// When it was last read.
    pub seen_s: f64,
}

/// How long the head must stay in `CS_ERROR` before auto-restart recovers production (a reset pulse of an
/// operator, or of a running `plan2cell cell start`, gets there first).
pub const HEAD_ERROR_GRACE_S: f64 = 20.0;
/// A head read older than this is no read (the bridge stopped).
const HEAD_FRESH_S: f64 = 60.0;

#[derive(Default)]
struct Inner {
    status: InfraStatus,
    refreshed: Option<Instant>,
    interest: Option<Instant>,
    /// A job holds the cell: `bring-up`, `production` or `deploy` (Run Control Code).
    job: Option<&'static str>,
    /// `start` / `stop` asked for, and when.
    transition: Option<(String, Instant)>,
    /// Counts Stop cell requests: a Start cell chain started before one stops waiting.
    stops: u64,
    /// When the CODESYS runtimes were last read (`None`: read at the next tick).
    runtime_checked: Option<Instant>,
    /// The cell root whose jobs `status.deploy` and `status.cell` describe (the active cell's,
    /// published last while no job ran): another active cell resets them to idle.
    jobs_root: Option<PathBuf>,
    /// When the cell's lease was last read (`status.lease`).
    lease_read: Option<Instant>,
    /// A task renews the lease this backend holds and releases it once no job runs.
    lease_keeper: bool,
    /// The last leased action that changed the cell's production (reported on release).
    lease_change: Option<&'static str>,
    /// The Lambda answered `power_status` with an unknown action (an older one): `status` until then.
    power_unsupported: Option<Instant>,
    /// When this backend last told the Lambda it uses the cell (`touch`).
    touched: Option<Instant>,
    /// A Start virtual cell of this backend that waits for the cell to be ready.
    power_follow: Option<PowerFollow>,
    /// The workspace's own cell, as last read.
    own: OwnCell,
    /// The Lambda's `templates`, as last read and when.
    templates: Option<(Instant, TemplatesRead)>,
}

/// The Lambda's `templates` answer.
#[derive(Clone, Debug)]
struct TemplatesRead {
    list: CellTemplates,
    /// An older Lambda: no `templates` action (the backend maps the legacy ids itself).
    older: bool,
}

/// What the backend knows of its own cell ([`CellBinding::Own`]).
#[derive(Default)]
struct OwnCell {
    /// The Lambda's `CellDoc` (`None`: no cell).
    doc: Option<Value>,
    /// The launch's wiring (from `configuring` on).
    wiring: Option<Wiring>,
    /// The Lambda's restore plan of the launch (`{mode: fast|full|none, build, planId, firstBoot?}`).
    restore_info: Option<Value>,
    /// The restore a Start virtual cell of this backend asked for (taken once the cell is ready).
    restore_plan: Option<RestorePlan>,
    /// A restore runs.
    restoring: bool,
    restore: Option<CellRestore>,
    /// The launch the last restore was for (also in `<wire dir>/restored`).
    restored_launch: Option<u32>,
    /// The resume (its power session's `requestedAt`) the last resume path was for (also in
    /// `<wire dir>/resumed`).
    resumed: Option<String>,
    /// The last error writing the wiring (logged once).
    wire_error: Option<String>,
}

/// After Start virtual cell of an own cell: restore the control code (`mode`), then maybe production.
#[derive(Clone, Debug)]
struct RestorePlan {
    mode: String,
    production: bool,
}

/// After Start virtual cell: once the Lambda reports the cell ready, Start production (`production`).
#[derive(Clone, Copy, Debug)]
struct PowerFollow {
    production: bool,
    since: Instant,
}

/// The cell's power keeps every job off it (off, stopping, booting).
fn power_holds(power: Option<&CellPower>) -> Option<&CellPower> {
    power.filter(|p| matches!(p.state, PowerState::Off | PowerState::Stopping | PowerState::Booting))
}

/// `off`, `stopping`, `booting (bring-up)`.
fn power_word(power: &CellPower) -> String {
    match power.state {
        PowerState::Booting => power.booting_text(),
        PowerState::Stopping => "stopping".to_owned(),
        _ => "off".to_owned(),
    }
}

/// The cell-control Lambda's answer to a `lease` call.
#[derive(Debug)]
enum Lease {
    /// Done (`acquire`: held now); the lease as it answered.
    Ok(Option<CellLease>),
    /// Another backend holds it.
    Busy(CellLease),
    /// No leases (an older Lambda) or an error: go on without.
    Unsupported(String),
}

/// The lease in a `lease` answer (`lease`, else the answer itself), `mine` for `me`.
fn parse_lease(body: &Value, me: &str) -> Option<CellLease> {
    let value = if body["lease"].is_object() { &body["lease"] } else { body };
    let mut lease: CellLease = serde_json::from_value(value.clone()).ok()?;
    if lease.holder.is_none() {
        lease.holder = body["error"]["holder"].as_str().map(str::to_owned);
        lease.purpose = lease.purpose.or_else(|| body["error"]["purpose"].as_str().map(str::to_owned));
    }
    lease.mine = lease.holder.as_deref() == Some(me);
    Some(lease)
}

/// A `lease` answer refusing because another backend holds it.
fn lease_busy(body: &Value) -> bool {
    matches!(body["error"]["code"].as_str(), Some("busy" | "conflict" | "held" | "lease-held" | "not-holder" | "locked"))
        || body["error"]["name"] == "ConditionalCheckFailedException"
}

/// `cell busy: <holder> is <purpose> since hh:mm UTC`.
fn busy_message(lease: &CellLease) -> String {
    format!(
        "cell busy: {} is {}{}; try again when it has finished",
        lease.holder.as_deref().unwrap_or("another backend"),
        lease.purpose.as_deref().unwrap_or("running a job"),
        lease.since_s.map(|s| format!(" since {}", utc_hhmm(s))).unwrap_or_default()
    )
}

impl InfraService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>, hardware: Arc<HardwareService>) -> Arc<Self> {
        let settings = Settings::from_env();
        match &settings.function {
            Some(function) => tracing::info!("Virtual Cell: cell control function {function} ({} {})", settings.role.as_str(), settings.holder),
            None => tracing::info!("Virtual Cell: no cell control function (FACTORY_OS_CELL_CONTROL_FUNCTION)"),
        }
        for warning in &settings.warnings {
            tracing::warn!("Virtual Cell: {warning}");
        }
        let mut this = Self::with(Arc::new(bus.clone()), settings, config, hardware, None);
        Arc::get_mut(&mut this).expect("not shared yet").plan2cell_lookup = Some(crate::paths::plan2cell_binary);
        {
            let this = this.clone();
            crate::bus::serve_latest(bus, keys::INFRA_STATUS, move || Some(this.status()));
        }
        this.spawn_poller();
        this.spawn_head_follower(bus);
        this
    }

    /// Follow the head's live state (fos-live-bridge on `factoryos/cell/state`) for auto-restart.
    fn spawn_head_follower(self: &Arc<Self>, bus: &Bus) {
        let this = Arc::downgrade(self);
        let session = bus.session().clone();
        tokio::spawn(async move {
            let subscriber = match session.declare_subscriber(keys::CELL_STATE).await {
                Ok(s) => s,
                Err(err) => return tracing::warn!("Virtual Cell: no head state for auto-restart ({err})"),
            };
            while let Ok(sample) = subscriber.recv_async().await {
                let Some(this) = this.upgrade() else { return };
                let Ok(doc) = serde_json::from_slice::<Value>(&sample.payload().to_bytes()) else { continue };
                let head = &doc["plan2cell"]["head"];
                if let Some(state) = head["state"].as_str() {
                    this.note_head(state, head["error"].as_str(), head["cycles"].as_u64().unwrap_or_default(), unix_s(SystemTime::now()));
                }
            }
        });
    }

    /// The head was read in `state` (`since_s` moves only when the state changes).
    pub fn note_head(&self, state: &str, error: Option<&str>, cycles: u64, now: f64) {
        let mut head = self.head.lock().unwrap();
        let since_s = head.as_ref().filter(|h| h.state == state && h.error.as_deref() == error).map_or(now, |h| h.since_s);
        if head.as_ref().is_none_or(|h| h.state != state) {
            tracing::info!("Virtual Cell: head {state} {} cycles {cycles}", error.unwrap_or("-"));
        }
        *head = Some(HeadLive { state: state.to_owned(), error: error.map(str::to_owned), cycles, since_s, seen_s: now });
    }

    pub fn with(
        sink: Arc<dyn Sink>,
        settings: Settings,
        config: Arc<ConfigStore>,
        hardware: Arc<HardwareService>,
        plan2cell: Option<PathBuf>,
    ) -> Arc<Self> {
        let bad_qualifier = settings.qualifier.clone().filter(|q| !valid_qualifier(q));
        // A bad qualifier never falls back to the unqualified function (the shared cell, or a refusal).
        let function = settings.function.clone().filter(|f| valid_function(f) && bad_qualifier.is_none());
        let binding = Settings { function: function.clone(), ..settings.clone() }.binding();
        let mut status = InfraStatus {
            available: function.is_some(),
            binding: binding.as_ref().map(CellBinding::kind),
            function: function.clone(),
            cell_role: settings.role,
            cell_supervisor: settings.supervisor.clone(),
            ..Default::default()
        };
        status.message = match (&settings.function, &function) {
            (None, _) => Some(
                "No cell control function on this server: set FACTORY_OS_CELL_CONTROL_FUNCTION (a test server connected \
                 to the cell has it in /etc/factoryos/cell-control.env; Lambda: plan2cell/infra Plan2cellCellControl)"
                    .to_owned(),
            ),
            (Some(_), None) if bad_qualifier.is_some() => {
                Some(format!("FACTORY_OS_CELL_CONTROL_QUALIFIER {:?} is not a Lambda alias", bad_qualifier.unwrap_or_default()))
            }
            (Some(f), None) => Some(format!("FACTORY_OS_CELL_CONTROL_FUNCTION {f:?} is not a Lambda function name or ARN")),
            _ => None,
        };
        status.cell.demo_note = Some(demo_note(&status, settings.semantic_hosts));
        Arc::new(Self {
            sink,
            settings: Settings { function, ..settings },
            config,
            hardware,
            plan2cell,
            plan2cell_lookup: None,
            inner: Mutex::new(Inner { status, ..Default::default() }),
            refreshing: tokio::sync::Mutex::new(()),
            runtime_reading: tokio::sync::Mutex::new(()),
            auto_file: Mutex::new(()),
            mcp_token: Mutex::new(None),
            head: Mutex::new(None),
        })
    }

    fn spawn_poller(self: &Arc<Self>) {
        let weak: Weak<Self> = Arc::downgrade(self);
        let poll = self.settings.poll;
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(poll).await;
                let Some(this) = weak.upgrade() else { break };
                if this.should_poll() {
                    this.refresh().await;
                }
            }
        });
        // The runtimes and auto-restart: their own loop (a runtime read takes a few seconds over SSM).
        let weak: Weak<Self> = Arc::downgrade(self);
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(poll).await;
                let Some(this) = weak.upgrade() else { break };
                this.runtime_tick().await;
            }
        });
    }

    fn should_poll(&self) -> bool {
        let inner = self.inner.lock().unwrap();
        // A power-on to follow, or the cell on its way on / off: read on (the follow-up and auto-restart need it).
        let power = inner.power_follow.is_some()
            || inner.status.cell_power.as_ref().is_some_and(|p| matches!(p.state, PowerState::Booting | PowerState::Stopping));
        inner.status.available
            && (inner.job.is_some() || inner.transition.is_some() || power || inner.interest.is_some_and(|at| at.elapsed() < INTEREST))
    }

    pub fn status(&self) -> InfraStatus {
        self.inner.lock().unwrap().status.clone()
    }

    /// Change the status and publish it, under the lock (published in the order changed).
    fn update(&self, change: impl FnOnce(&mut InfraStatus)) {
        let mut inner = self.inner.lock().unwrap();
        change(&mut inner.status);
        inner.status.transition = inner.transition.as_ref().map(|(t, _)| t.clone());
        inner.status.cell.demo_note = Some(demo_note(&inner.status, self.settings.semantic_hosts));
        self.sink.status(&inner.status);
    }

    fn log(&self, line: impl AsRef<str>) {
        self.sink.log(format!("[infra] {}", line.as_ref()));
    }

    fn function(&self) -> Result<&str, HttpError> {
        self.settings.function.as_deref().ok_or_else(|| {
            (StatusCode::SERVICE_UNAVAILABLE, self.status().message.unwrap_or_else(|| "no cell control function".to_owned()))
        })
    }

    /// Whether the backend serves the Windows IDE's DCV at `/dcv/` (services/dcv.rs), in the status.
    /// `fallback`: it has a DCV server of its own for a cell without a Windows IDE.
    /// `targets`: those served at `/dcv/<target>/`.
    pub fn set_dcv_gateway(&self, on: bool, fallback: bool, targets: Vec<String>) {
        self.update(|s| {
            s.dcv_gateway = on;
            s.dcv_server_fallback = on && fallback;
            s.dcv_targets = if on { targets } else { Vec::new() };
        });
    }

    // --- the cell's lease (several backends on one cell) ---

    /// `{"action": "lease", "op": …, "holder": <me>, …}`.
    async fn lease_call(&self, mut payload: Value) -> Lease {
        payload["action"] = json!("lease");
        payload["holder"] = json!(self.settings.holder);
        match self.invoke(payload).await {
            Err(err) => Lease::Unsupported(format!("{err:#}")),
            Ok(body) => {
                let lease = parse_lease(&body, &self.settings.holder);
                if body["ok"] == true {
                    Lease::Ok(lease.filter(|l| l.holder.is_some() || l.last_change.is_some()))
                } else if lease_busy(&body) {
                    Lease::Busy(lease.unwrap_or_default())
                } else {
                    Lease::Unsupported(lambda_error(&body))
                }
            }
        }
    }

    fn set_lease(&self, lease: Option<CellLease>) {
        self.inner.lock().unwrap().lease_read = Some(Instant::now());
        if self.status().lease != lease {
            self.update(|s| s.lease = lease);
        }
    }

    /// The cell's lease, read again when older than [`Settings::lease_cache`] (`None`: free, or no leases).
    async fn lease_status(&self) -> Option<CellLease> {
        self.settings.function.as_ref()?;
        let fresh = self.inner.lock().unwrap().lease_read.is_some_and(|at| at.elapsed() < self.settings.lease_cache);
        if fresh {
            return self.status().lease;
        }
        let lease = match self.lease_call(json!({"op": "status"})).await {
            Lease::Ok(lease) => lease,
            Lease::Busy(lease) => Some(lease),
            Lease::Unsupported(_) => None,
        };
        self.set_lease(lease.clone());
        lease
    }

    /// Hold the cell's lease for `purpose`: `Ok(true)` held, `Ok(false)` no leases here (go on),
    /// `409` another backend holds it.
    async fn acquire(&self, purpose: &str) -> Result<bool, HttpError> {
        if self.settings.function.is_none() {
            return Ok(false);
        }
        let ttl = self.settings.lease_ttl.as_secs();
        match self.lease_call(json!({"op": "acquire", "purpose": purpose, "ttlS": ttl})).await {
            Lease::Ok(lease) => {
                self.set_lease(lease);
                Ok(true)
            }
            Lease::Busy(lease) => {
                let message = busy_message(&lease);
                self.set_lease(Some(lease));
                self.log(format!("{purpose}: {message}"));
                Err((StatusCode::CONFLICT, message))
            }
            Lease::Unsupported(why) => {
                tracing::debug!("Virtual Cell: no cell lease ({why}); {purpose} goes on without");
                Ok(false)
            }
        }
    }

    /// Run `action` (a user action, or auto-restart) holding the cell's lease for `purpose`; a
    /// task renews it while the job it started runs, then releases it (with `lastChange` when
    /// `changes` and the action was accepted: the cell's production changed).
    pub async fn leased<T>(
        self: &Arc<Self>,
        purpose: &'static str,
        changes: bool,
        action: impl Future<Output = Result<T, HttpError>>,
    ) -> Result<T, HttpError> {
        let held = self.acquire(purpose).await?;
        let result = action.await;
        if held {
            if changes && result.is_ok() {
                self.inner.lock().unwrap().lease_change = Some(purpose);
            }
            self.keep_lease();
        }
        result
    }

    /// A job (or Start cell's chain) runs.
    fn busy_now(inner: &Inner) -> bool {
        inner.job.is_some() || inner.status.start.state.busy()
    }

    fn busy(&self) -> bool {
        Self::busy_now(&self.inner.lock().unwrap())
    }

    /// Renew the lease while a job runs, then release it (once per backend).
    fn keep_lease(self: &Arc<Self>) {
        {
            let mut inner = self.inner.lock().unwrap();
            if inner.lease_keeper {
                return;
            }
            inner.lease_keeper = true;
        }
        let this = self.clone();
        tokio::spawn(async move {
            let ttl = this.settings.lease_ttl.as_secs();
            let every = this.settings.lease_ttl / 3;
            let mut renewed = Instant::now();
            loop {
                while this.busy() {
                    if renewed.elapsed() >= every {
                        renewed = Instant::now();
                        if let Lease::Busy(lease) = this.lease_call(json!({"op": "renew", "ttlS": ttl})).await {
                            this.log(format!("the cell's lease went to {} meanwhile", lease.holder.as_deref().unwrap_or("another backend")));
                        }
                    }
                    tokio::time::sleep(this.settings.poll.min(Duration::from_secs(5))).await;
                }
                let change = this.inner.lock().unwrap().lease_change.take();
                let mut payload = json!({"op": "release"});
                if let Some(action) = change {
                    let plan_id = cells::active_cell(&this.config).and_then(|(_, root)| load_auto(&root).status.deployed_plan_id);
                    payload["lastChange"] =
                        json!({"holder": this.settings.holder, "action": action, "planId": plan_id, "atS": unix_s(SystemTime::now())});
                }
                let answer = this.lease_call(payload).await;
                if let Lease::Ok(lease) = answer {
                    this.set_lease(lease);
                }
                let done = {
                    let mut inner = this.inner.lock().unwrap();
                    let done = !Self::busy_now(&inner);
                    if done {
                        inner.lease_keeper = false;
                        inner.lease_read = None;
                    }
                    done
                };
                if done {
                    return;
                }
                // Another action of this backend started meanwhile: hold the lease for it again.
                let _ = this.lease_call(json!({"op": "acquire", "purpose": "a job", "ttlS": ttl})).await;
                renewed = Instant::now();
            }
        });
    }

    /// The Windows IDE's private address while it runs (for the DCV gateway's resolver): the status,
    /// read again if it has none or is old.
    /// A cell whose template has no DCV IDE (`ide.dcv`) has none (the gateway falls back to its own
    /// server, if any); with [`Settings::semantic_hosts`], the template's `ide` host.
    pub async fn windows_ide_address(&self) -> Option<String> {
        if self.lacks("ide.dcv", "DCV").is_some() {
            return None;
        }
        let semantic = self.settings.semantic_hosts;
        let address = |status: &InfraStatus| {
            host_of(status, semantic, cell_wiring::SEMANTIC_IDE, ROLE_WINDOWS_IDE).filter(|i| i.running()).and_then(|i| i.private_ip.clone())
        };
        match address(&self.status()) {
            Some(ip) if self.inner.lock().unwrap().refreshed.is_some_and(|at| at.elapsed() < Duration::from_secs(60)) => Some(ip),
            _ => address(&self.current().await),
        }
    }

    /// Open PLC IDE: CODESYS in front on the Windows IDE's desktop, maximized, with the head project
    /// ([`InfraStatus::ide_project`]) open. The MCP bridge's `POST /show-ide` (next to the server
    /// cell.toml's `[codesys_mcp] url`, with the MCP's bearer token) runs Show-Codesys.ps1 in the
    /// autologon session; when no CODESYS window shows the project it opens it through the MCP
    /// (starting CODESYS if none runs). While a job (bring-up, production, deploy; of this backend,
    /// or of another holding the cell's lease) holds the cell it drives CODESYS itself: the window
    /// is only shown then. Best effort: the bridge's answer.
    pub async fn show_ide(&self) -> anyhow::Result<Value> {
        let config = self.hardware.server_cell_config_path().context("no cell config on this server")?;
        let text = std::fs::read_to_string(config).with_context(|| format!("reading {}", config.display()))?;
        let url = show_ide_url(&text).with_context(|| format!("no http [codesys_mcp] url in {}", config.display()))?;
        // Another backend's job on the cell drives CODESYS: only show the window then.
        let foreign = self.lease_status().await.is_some_and(|l| l.held_by_other(unix_s(SystemTime::now())));
        let project = {
            let inner = self.inner.lock().unwrap();
            inner.status.ide_project.as_ref().map(|p| p.path.clone()).filter(|_| inner.job.is_none() && !foreign)
        };
        let cached = self.mcp_token.lock().unwrap().clone();
        let token = match cached {
            Some(token) => token,
            None => {
                let token = self.secret(&self.mcp_secret()).await?;
                *self.mcp_token.lock().unwrap() = Some(token.clone());
                token
            }
        };
        let client = reqwest::Client::builder().timeout(SHOW_IDE_TIMEOUT).build()?;
        let response = client
            .post(url.clone())
            .bearer_auth(&token)
            .header(reqwest::header::CONTENT_TYPE, "application/json")
            .body(json!({ "project": project }).to_string())
            .send()
            .await
            .with_context(|| format!("POST {url}"))?;
        let status = response.status();
        if status == reqwest::StatusCode::UNAUTHORIZED {
            // The secret may have been rotated: read it again next time.
            self.mcp_token.lock().unwrap().take();
        }
        let body = response.bytes().await.unwrap_or_default();
        anyhow::ensure!(status.is_success(), "POST {url}: {status} {}", String::from_utf8_lossy(&body).chars().take(300).collect::<String>());
        serde_json::from_slice(&body).with_context(|| format!("POST {url}: not JSON"))
    }

    /// For `GET`: keeps the polling on, and reads the status if it is older than a few seconds.
    pub async fn current(&self) -> InfraStatus {
        let stale = {
            let mut inner = self.inner.lock().unwrap();
            inner.interest = Some(Instant::now());
            inner.status.available && inner.refreshed.is_none_or(|at| at.elapsed() > FRESH)
        };
        if stale {
            self.refresh().await;
        }
        self.status()
    }

    /// The Lambda has the power actions (not known otherwise until it answers).
    fn power_supported(&self) -> bool {
        self.inner.lock().unwrap().power_unsupported.is_none_or(|at| at.elapsed() >= POWER_RETRY)
    }

    /// Read the instances (and the cell's power) through the Lambda's `power_status`; an older
    /// Lambda: `status`.
    pub async fn refresh(&self) {
        if self.settings.function.is_none() {
            return;
        }
        if self.settings.own() {
            return self.refresh_own().await;
        }
        let _one = self.refreshing.lock().await;
        let power = self.power_supported();
        let mut result = self.invoke(json!({"action": if power { "power_status" } else { "status" }})).await;
        if power
            && let Ok(body) = &result
            && body["ok"] != true
            && matches!(body["error"]["code"].as_str(), Some("unknown-action" | "not-configured"))
        {
            tracing::info!("Virtual Cell: the cell-control Lambda has no power ({}); status only", lambda_error(body));
            self.inner.lock().unwrap().power_unsupported = Some(Instant::now());
            result = self.invoke(json!({"action": "status"})).await;
        } else if power && result.as_ref().is_ok_and(|b| b["ok"] == true) {
            self.inner.lock().unwrap().power_unsupported = None;
        }
        let now = unix_s(SystemTime::now());
        {
            let mut inner = self.inner.lock().unwrap();
            inner.refreshed = Some(Instant::now());
            if let Ok(body) = &result
                && body["ok"] == true
            {
                let instances = parse_instances(&body["instances"]);
                end_transition(&mut inner, &instances);
                let status = &mut inner.status;
                status.instances = instances;
                status.all_running = body["allRunning"] == true;
                status.ready = body["ready"] == true;
                status.dcv_url = body["dcvUrl"].as_str().map(str::to_owned);
                status.cell_power = serde_json::from_value(body["power"].clone()).ok().filter(|_| body["power"].is_object());
                status.updated_s = now;
                status.message = None;
            }
        }
        match result {
            Ok(body) if body["ok"] == true => self.update(|_| {}),
            Ok(body) => {
                let message = lambda_error(&body);
                self.update(|s| s.message = Some(format!("cell control status: {message}")));
            }
            Err(err) => self.update(|s| s.message = Some(format!("cell control status: {err:#}"))),
        }
    }

    /// [`Self::refresh`] of an own cell: the Lambda's `cell_status`.
    async fn refresh_own(&self) {
        let _one = self.refreshing.lock().await;
        let result = self.invoke(json!({"action": "cell_status"})).await;
        let now = unix_s(SystemTime::now());
        self.inner.lock().unwrap().refreshed = Some(Instant::now());
        let body = match result {
            Ok(body) if body["ok"] == true => body,
            Ok(body) => {
                let message = lambda_error(&body);
                return self.update(|s| s.message = Some(format!("cell control status: {message}")));
            }
            Err(err) => return self.update(|s| s.message = Some(format!("cell control status: {err:#}"))),
        };
        let read = OwnRead::of(&body);
        {
            let mut inner = self.inner.lock().unwrap();
            end_transition(&mut inner, &read.instances);
            let restore = inner.own.restore.clone();
            inner.own.doc = read.doc.clone();
            inner.own.wiring = read.wiring.clone();
            inner.own.restore_info = Some(body["restore"].clone()).filter(Value::is_object);
            let status = &mut inner.status;
            status.virtual_cell = read.doc.as_ref().map(|doc| virtual_cell(doc, &body, read.power.clone(), restore, now));
            let off = if read.doc.is_some() { "Its instances are not launched: Start virtual cell" } else { "No virtual cell yet: Create virtual cell" };
            status.cell_power = Some(read.power.clone().unwrap_or_else(|| CellPower {
                state: PowerState::Off,
                phase: "off".to_owned(),
                message: Some(off.to_owned()),
                ..Default::default()
            }));
            status.instances = read.instances;
            status.all_running = read.all_running;
            status.ready = read.ready;
            status.dcv_url = None;
            status.updated_s = now;
            status.message = None;
        }
        self.update(|_| {});
        self.wire_if_ready();
    }

    /// The own cell's wiring into its files once it is ready (and its addresses changed).
    fn wire_if_ready(&self) {
        let Some(files) = &self.settings.wire else { return };
        let (wiring, ready, capabilities) = {
            let inner = self.inner.lock().unwrap();
            let template = inner.status.virtual_cell.as_ref().and_then(|c| c.template.as_ref());
            (inner.own.wiring.clone(), inner.status.cell_power.as_ref().is_some_and(|p| p.phase == "ready"), template.map(|t| t.capabilities.clone()))
        };
        let Some(wiring) = wiring.filter(|_| ready) else { return };
        match files.write(&wiring, capabilities.as_deref()) {
            Ok(true) if wiring.has_hook() => {
                self.inner.lock().unwrap().own.wire_error = None;
                let hooks: Vec<&str> = wiring.components.iter().filter(|c| c.hook()).map(|c| c.name.as_str()).collect();
                self.log(format!("virtual cell launch {}: wired ({} hook components: {}) into {}", wiring.launch, hooks.len(), hooks.join(", "), files.dir.display()));
            }
            Ok(true) => {
                self.inner.lock().unwrap().own.wire_error = None;
                self.log(format!(
                    "virtual cell launch {}: wired (PLC host {}, Windows IDE {}, {} KUKA VC hosts) into {}",
                    wiring.launch,
                    wiring.plc_host.ip,
                    wiring.windows_ide.ip,
                    wiring.kuka_vc.len(),
                    files.dir.display()
                ));
            }
            Ok(false) => {}
            Err(err) => {
                let message = format!("{err:#}");
                let new = self.inner.lock().unwrap().own.wire_error.replace(message.clone()) != Some(message.clone());
                if new {
                    self.log(format!("virtual cell launch {}: writing the wiring: {message}", wiring.launch));
                }
            }
        }
    }

    /// The launch's wiring is in its files (or nothing is written on this backend).
    fn wired(&self, wiring: &Wiring) -> bool {
        self.settings.wire.as_ref().is_none_or(|files| files.current().as_deref() == Some(wiring.generation().as_str()))
    }

    /// The CODESYS MCP token's secret: the own cell's (its wiring), else the configured one.
    fn mcp_secret(&self) -> String {
        let inner = self.inner.lock().unwrap();
        inner
            .own
            .wiring
            .as_ref()
            .and_then(|w| w.mcp_token_secret.clone())
            .or_else(|| inner.own.doc.as_ref().and_then(|d| d["mcpTokenSecret"].as_str().map(str::to_owned)))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| self.settings.mcp_token_secret.clone())
    }

    // --- the workspace's own cells ---

    /// The own cell's template (`None`: no own cell, an older Lambda, templates off, the shared cell).
    fn template(&self) -> Option<CellTemplateRef> {
        self.inner.lock().unwrap().status.virtual_cell.as_ref().and_then(|c| c.template.clone())
    }

    /// Why `what` is not offered: the cell's template has no `capability` (`None`: it has, or the
    /// cell has no template, which keeps today's behaviour).
    fn lacks(&self, capability: &str, what: &str) -> Option<String> {
        let t = self.template()?;
        (!t.has(capability)).then(|| format!("{what}: this virtual cell's template {} has no {capability}", t.label()))
    }

    /// Why the smartPADs are not offered (the template has no `smartpad`), for the smartPAD proxy.
    pub fn smartpad_refusal(&self) -> Option<String> {
        self.lacks("smartpad", "smartPAD")
    }

    /// Whether `request` changes the controller: by the template's operations (`400` for one it does
    /// not have or a bad argument; plan2cell's own KSS operations never less than today's), else
    /// today's list ([`KvcRequest::changes`]).
    pub fn kvc_changes(&self, request: &KvcRequest) -> Result<bool, HttpError> {
        match self.template() {
            Some(t) => request.changes_for(&t.consumer.ops).map_err(|err| (StatusCode::BAD_REQUEST, err)),
            None => Ok(request.changes()),
        }
    }

    fn own_binding(&self) -> Result<String, HttpError> {
        self.function()?;
        self.settings.qualifier.clone().ok_or_else(|| {
            (
                StatusCode::CONFLICT,
                "this workspace is bound to the shared virtual cell plan2cell: its own cell needs FactoryosCell=own:ws-<alias> \
                 (the workspace provisioner sets it)"
                    .to_owned(),
            )
        })
    }

    /// `GET` [`api::INFRA_CELLS`].
    pub async fn cells(&self) -> CellsList {
        let shared = || CellsList {
            available: false,
            max_per_workspace: 1,
            can_create: false,
            reason: Some("bound to the shared virtual cell plan2cell".to_owned()),
            shared: Some("plan2cell".to_owned()),
            ..Default::default()
        };
        match self.settings.binding() {
            None => {
                return CellsList {
                    reason: Some(self.status().message.unwrap_or_else(|| "no cell control function".to_owned())),
                    max_per_workspace: 1,
                    ..Default::default()
                };
            }
            Some(CellBinding::Shared { .. }) => return shared(),
            Some(CellBinding::Own { .. }) => {}
        }
        self.inner.lock().unwrap().interest = Some(Instant::now());
        let body = match self.invoke(json!({"action": "cell_status"})).await {
            Ok(body) if body["ok"] == true => body,
            Ok(body) => return CellsList { max_per_workspace: 1, reason: Some(format!("cell control: {}", lambda_error(&body))), ..Default::default() },
            Err(err) => return CellsList { max_per_workspace: 1, reason: Some(format!("cell control: {err:#}")), ..Default::default() },
        };
        // The status too (one read for both).
        self.refresh_own().await;
        let cell = self.status().virtual_cell;
        let limits = &body["limits"];
        let max_per = limits["maxPerWorkspace"].as_u64().unwrap_or(1) as u32;
        // The account's cell count: `cells` (the Lambda's, with templates on); `count` from an older one.
        let count = limits["cells"].as_u64().or_else(|| limits["count"].as_u64());
        let max = limits["max"].as_u64();
        let reason = if cell.is_some() {
            Some(format!("this workspace has its virtual cell already (at most {max_per})"))
        } else if let (Some(count), Some(max)) = (count, max)
            && count >= max
        {
            Some(format!("the account's cap of {max} virtual cells is reached ({count})"))
        } else {
            None
        };
        let runtimes = body["runtimes"].as_array().map(|r| r.iter().filter_map(|x| x.as_str().map(str::to_owned)).collect()).unwrap_or_default();
        // The templates to create from (none from an older Lambda: the runtime pickers).
        let templates = self.templates_read().await.map(|t| t.list).unwrap_or_default();
        CellsList {
            available: true,
            max_per_workspace: max_per,
            can_create: reason.is_none(),
            reason,
            cells: cell.into_iter().collect(),
            shared: None,
            runtimes,
            templates_enabled: templates.enabled,
            templates: templates.templates,
        }
    }

    /// `GET` [`api::INFRA_TEMPLATES`]: the templates this workspace may create a cell from.
    pub async fn templates(&self) -> CellTemplates {
        match self.settings.binding() {
            None => {
                return CellTemplates { reason: Some(self.status().message.unwrap_or_else(|| "no cell control function".to_owned())), ..Default::default() };
            }
            Some(CellBinding::Shared { .. }) => {
                return CellTemplates { reason: Some("bound to the shared virtual cell plan2cell".to_owned()), ..Default::default() };
            }
            Some(CellBinding::Own { .. }) => {}
        }
        match self.templates_read().await {
            Ok(read) => read.list,
            Err(err) => CellTemplates { reason: Some(format!("cell control templates: {err}")), ..Default::default() },
        }
    }

    /// The Lambda's `templates` (cached [`TEMPLATES_FRESH`]; an error is not cached). A template that
    /// needs a wiring level beyond [`CONSUMER_LEVEL`] is listed as unavailable (`consumer`).
    async fn templates_read(&self) -> Result<TemplatesRead, String> {
        if let Some((at, read)) = &self.inner.lock().unwrap().templates
            && at.elapsed() < TEMPLATES_FRESH
        {
            return Ok(read.clone());
        }
        let body = self.invoke(json!({"action": "templates"})).await.map_err(|err| format!("{err:#}"))?;
        let read = if body["ok"] == true {
            let mut list = CellTemplates {
                available: true,
                enabled: body["enabled"] == true,
                engine: body["engine"].as_u64().unwrap_or(0) as u32,
                revision: body["revision"].as_u64(),
                source: body["source"].as_str().unwrap_or_default().to_owned(),
                // An entry this backend cannot read is left out (and logged), not the whole list.
                templates: body["templates"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|t| {
                        serde_json::from_value::<CellTemplateInfo>(t.clone())
                            .inspect_err(|err| tracing::warn!("Virtual Cell: templates: left out {}: {err}", t["id"].as_str().unwrap_or("an entry without id")))
                            .ok()
                    })
                    .collect(),
                reason: None,
            };
            for t in list.templates.iter_mut().filter(|t| t.available && t.beyond_consumer()) {
                t.available = false;
                t.reason = Some("consumer".to_owned());
            }
            TemplatesRead { list, older: false }
        } else if older_lambda(&body) {
            let reason = "the cell Lambda has no templates (an older one): create by runtime".to_owned();
            TemplatesRead { list: CellTemplates { reason: Some(reason), ..Default::default() }, older: true }
        } else {
            return Err(lambda_error(&body));
        };
        self.inner.lock().unwrap().templates = Some((Instant::now(), read.clone()));
        Ok(read)
    }

    /// `template` and `params` of a create or update into the Lambda's `payload`, checked for their
    /// shape only (the Lambda checks them against the template). A template beyond this backend's
    /// wiring level is `501`. An older Lambda (no templates): a legacy id is mapped to today's
    /// fields (docs/tasks/cell-templates.md §8), any other is `501`.
    async fn template_fields(&self, payload: &mut Value, template: Option<&TemplateSelector>, params: Option<&serde_json::Map<String, Value>>) -> Result<(), HttpError> {
        if let Some(t) = template {
            if !valid_template_id(t.id()) {
                return Err((StatusCode::BAD_REQUEST, format!("template: an id like {TEMPLATE_EC2_CODESYS_KUKA}, not {:?}", t.id())));
            }
            if let Some(v) = t.version().filter(|v| !valid_template_version(v)) {
                return Err((StatusCode::BAD_REQUEST, format!("template.version: MAJOR.MINOR.PATCH, not {v:?}")));
            }
        }
        if let Some(name) = params.into_iter().flat_map(|p| p.keys()).find(|k| !valid_param_name(k)) {
            return Err((StatusCode::BAD_REQUEST, format!("params: no parameter {name:?}")));
        }
        let read = self.templates_read().await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("cell control templates: {err}")))?;
        if !read.older {
            if let Some(t) = template
                && let Some(info) = read.list.templates.iter().find(|i| i.id == t.id())
                && info.beyond_consumer()
            {
                return Err((
                    StatusCode::NOT_IMPLEMENTED,
                    format!("template {}: needs wiring level {} (this backend renders {CONSUMER_LEVEL}): update the workspace", t.id(), info.requires.as_ref().map_or(0, |r| r.consumer)),
                ));
            }
            if let Some(t) = template {
                payload["template"] = json!(t);
            }
            if let Some(p) = params {
                payload["params"] = Value::Object(p.clone());
            }
            return Ok(());
        }
        legacy_template_fields(payload, template, params)
    }

    /// `POST` [`api::INFRA_CELLS`]: create the cell (no instances; lifecycle `off`), and start it if asked.
    pub async fn create_cell(self: &Arc<Self>, request: CreateCellRequest) -> Result<VirtualCell, HttpError> {
        let id = self.own_binding()?;
        let templated = request.template.is_some() || request.params.is_some();
        let mut payload = json!({"action": "cell_create"});
        let what = if templated {
            if request.has_legacy_fields() {
                return Err((StatusCode::BAD_REQUEST, TEMPLATE_AND_LEGACY_FIELDS.to_owned()));
            }
            self.template_fields(&mut payload, request.template.as_ref(), request.params.as_ref()).await?;
            format!("template {}", request.template.as_ref().map_or("default", TemplateSelector::id))
        } else {
            // Today's request: the Lambda maps it to the legacy template itself.
            let robots = request.robots.unwrap_or(2);
            if !(1..=8).contains(&robots) {
                return Err((StatusCode::BAD_REQUEST, format!("robots must be 1 to 8, not {robots}")));
            }
            payload["robots"] = json!(robots);
            if let Some(mode) = &request.off_mode {
                if !OFF_MODES.contains(&mode.as_str()) {
                    return Err((StatusCode::BAD_REQUEST, format!("offMode must be one of {}, not {mode:?}", OFF_MODES.join(", "))));
                }
                payload["offMode"] = json!(mode);
            }
            runtime_fields(&mut payload, request.runtime.as_deref(), request.ide.as_deref())?;
            networks_field(&mut payload, request.networks.as_deref())?;
            format!("{robots} robots")
        };
        let body = self.invoke(payload).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
        if body["ok"] != true {
            return Err(cell_error("cell_create", &body));
        }
        self.log(format!("Create virtual cell {id} ({what})"));
        if request.start {
            if let Err((_, err)) = self.leased("power-on", true, self.power(CellPowerRequest { action: "on".into(), ..Default::default() })).await {
                self.log(format!("Create virtual cell {id}: Start: {err}"));
            }
        } else {
            self.refresh().await;
        }
        let now = unix_s(SystemTime::now());
        Ok(self.status().virtual_cell.unwrap_or_else(|| virtual_cell(&body["cell"], &body, None, None, now)))
    }

    /// `PATCH` [`api::INFRA_CELL_ITEM`]: the cell's off mode (what Stop does from now on) and its
    /// robots (only while off), the Lambda's `cell_update`.
    pub async fn update_cell(&self, id: &str, request: UpdateCellRequest) -> Result<VirtualCell, HttpError> {
        let own = self.own_binding()?;
        if id != own {
            return Err((StatusCode::NOT_FOUND, format!("no virtual cell {id} in this workspace (its cell is {own})")));
        }
        let mut payload = json!({"action": "cell_update"});
        if request.template.is_some() || request.params.is_some() {
            if request.has_legacy_fields() {
                return Err((StatusCode::BAD_REQUEST, TEMPLATE_AND_LEGACY_FIELDS.to_owned()));
            }
            self.template_fields(&mut payload, request.template.as_ref(), request.params.as_ref()).await?;
        }
        if let Some(mode) = &request.off_mode {
            if !OFF_MODES.contains(&mode.as_str()) {
                return Err((StatusCode::BAD_REQUEST, format!("offMode must be one of {}, not {mode:?}", OFF_MODES.join(", "))));
            }
            payload["offMode"] = json!(mode);
        }
        if let Some(robots) = request.robots {
            payload["robots"] = json!(robots);
        }
        runtime_fields(&mut payload, request.runtime.as_deref(), request.ide.as_deref())?;
        networks_field(&mut payload, request.networks.as_deref())?;
        if request.template.is_none() && request.params.as_ref().is_none_or(|p| p.is_empty()) && !request.has_legacy_fields() {
            return Err((StatusCode::BAD_REQUEST, "nothing to change (offMode, robots, runtime, ide, networks; or template, params)".to_owned()));
        }
        let body = self.invoke(payload).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
        if body["ok"] != true {
            return Err(cell_error("cell_update", &body));
        }
        if let Some(mode) = &request.off_mode {
            self.log(format!(
                "Virtual cell {own}: Stop {} from now on",
                if mode == OFF_MODE_STOP { "stops the instances (Start resumes them; the disks are paid while stopped)" } else { "terminates the instances" }
            ));
        }
        if let Some(robots) = request.robots {
            self.log(format!("Virtual cell {own}: {robots} robots from the next fresh Start"));
        }
        if let Some(t) = &request.template {
            self.log(format!("Virtual cell {own}: template {} from the next Start", t.id()));
        }
        if let Some(params) = request.params.as_ref().filter(|p| !p.is_empty()) {
            let names: Vec<&str> = params.keys().map(String::as_str).collect();
            self.log(format!("Virtual cell {own}: {} changed", names.join(", ")));
        }
        if request.runtime.is_some() || request.ide.is_some() {
            let cell = &body["cell"];
            self.log(format!(
                "Virtual cell {own}: runtime {}{} from the next Start",
                cell["runtime"].as_str().unwrap_or(RUNTIME_EC2),
                cell["ide"].as_str().map(|i| format!(", IDE {i}")).unwrap_or_default()
            ));
        }
        self.refresh().await;
        let now = unix_s(SystemTime::now());
        Ok(self.status().virtual_cell.unwrap_or_else(|| virtual_cell(&body["cell"], &body, None, None, now)))
    }

    /// `DELETE` [`api::INFRA_CELL_ITEM`]: delete the cell (`confirm` must be its id; an on cell is
    /// powered off first, by the Lambda).
    pub async fn delete_cell(&self, id: &str, request: DeleteCellRequest) -> Result<VirtualCell, HttpError> {
        let own = self.own_binding()?;
        if id != own {
            return Err((StatusCode::NOT_FOUND, format!("no virtual cell {id} in this workspace (its cell is {own})")));
        }
        if request.confirm != own {
            return Err((StatusCode::BAD_REQUEST, format!("type the cell's id ({own}) to delete it")));
        }
        let in_production = self.status().in_production;
        let body = self
            .invoke(json!({"action": "cell_delete", "confirm": request.confirm, "inProduction": in_production}))
            .await
            .map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
        if body["ok"] != true {
            return Err(cell_error("cell_delete", &body));
        }
        self.log(format!("Delete virtual cell {own}"));
        // Its wiring and production are gone with it.
        if let Some(files) = &self.settings.wire {
            for file in [cell_wiring::GENERATION, cell_wiring::CELL_TOML, RESTORED_FILE, RESUMED_FILE] {
                let _ = std::fs::remove_file(files.dir.join(file));
            }
        }
        if let Some((_, root)) = cells::active_cell(&self.config) {
            let _ = self.change_auto(&root, |a| {
                a.in_production = false;
                a.stopped_in_production = false;
            });
        }
        {
            let mut inner = self.inner.lock().unwrap();
            inner.own = OwnCell::default();
            inner.power_follow = None;
        }
        let now = unix_s(SystemTime::now());
        let mut cell = match body["cell"].is_object() {
            true => virtual_cell(&body["cell"], &body, None, None, now),
            false => VirtualCell { id: own.clone(), kind: CellKind::Own, message: Some("deleted".to_owned()), ..Default::default() },
        };
        if cell.lifecycle != CellLifecycle::Deleting && body["cell"].is_object() {
            cell.lifecycle = CellLifecycle::Deleting;
        }
        self.refresh().await;
        Ok(cell)
    }

    /// The restore after a launch of the own cell, once it is ready and wired (see the module docs).
    async fn follow_restore(self: &Arc<Self>) {
        let (doc, wiring, ready, off, restoring, plan, restored, info, resume) = {
            let inner = self.inner.lock().unwrap();
            let power = inner.status.cell_power.as_ref();
            // A resume of the stopped instances (the Lambda's session): its key, the session's start.
            let resume = power
                .and_then(|p| p.session.as_ref())
                .filter(|s| s.op == "on" && s.resume)
                .map(|s| s.requested_at.clone().unwrap_or_default());
            (
                inner.own.doc.clone(),
                inner.own.wiring.clone(),
                power.is_some_and(|p| p.state == PowerState::Ready),
                power.is_none_or(|p| matches!(p.state, PowerState::Off | PowerState::Failed)),
                inner.own.restoring,
                inner.own.restore_plan.clone(),
                inner.own.restored_launch,
                inner.own.restore_info.clone(),
                resume,
            )
        };
        if restoring {
            return;
        }
        // A launch that did not get ready: no restore; production stays due for the next Start (it was
        // cleared when this one was asked for).
        let drop_plan = |plan: &Option<RestorePlan>| {
            self.inner.lock().unwrap().own.restore_plan = None;
            if plan.as_ref().is_some_and(|p| p.production)
                && let Some((_, root)) = cells::active_cell(&self.config)
            {
                let _ = self.change_auto(&root, |a| a.stopped_in_production = true);
            }
        };
        let (Some(doc), Some(wiring)) = (doc, wiring) else {
            if off && plan.is_some() {
                drop_plan(&plan);
            }
            return;
        };
        if !ready || !self.wired(&wiring) {
            if off && plan.is_some() {
                drop_plan(&plan);
                self.log("Start virtual cell: the cell did not get ready; no restore");
            }
            return;
        }
        let launch = doc["launch"].as_u64().map_or(wiring.launch, |l| l as u32);
        // A resume (S4): the launch is unchanged and its code is on the disks, so no restore; only
        // production again if it was in production when it stopped.
        if let Some(key) = resume {
            let seen = self.inner.lock().unwrap().own.resumed.clone().or_else(|| self.resumed_file());
            if seen.as_deref() == Some(key.as_str()) {
                return;
            }
            let plan = plan.unwrap_or_else(|| RestorePlan {
                mode: "auto".to_owned(),
                production: cells::active_cell(&self.config).is_some_and(|(_, root)| load_auto(&root).stopped_in_production),
            });
            {
                let mut inner = self.inner.lock().unwrap();
                inner.own.restore_plan = None;
                inner.own.restoring = true;
                inner.own.resumed = Some(key.clone());
                // A fresh launch after this one restores again (its launch differs).
                inner.own.restored_launch = Some(launch);
            }
            self.write_resumed(&key);
            self.write_restored(launch);
            let this = self.clone();
            tokio::spawn(async move {
                let restore = this.run_resume(&plan, launch, &doc).await;
                this.restore_report(launch, &restore).await;
                this.inner.lock().unwrap().own.restoring = false;
            });
            return;
        }
        let restored = restored.or_else(|| self.restored_file());
        if restored == Some(launch) {
            return;
        }
        // Not started by this backend (or before its restart): as Start virtual cell would.
        // Not gated by the template's `restore` (yet): the contract's eks-codesys-kuka lacks it, and an
        // EKS cell is restored today (a Deploy only), which its legacy template keeps.
        let plan = plan.unwrap_or_else(|| RestorePlan {
            mode: "auto".to_owned(),
            production: cells::active_cell(&self.config).is_some_and(|(_, root)| load_auto(&root).stopped_in_production),
        });
        {
            let mut inner = self.inner.lock().unwrap();
            inner.own.restore_plan = None;
            inner.own.restoring = true;
            inner.own.restored_launch = Some(launch);
        }
        self.write_restored(launch);
        let this = self.clone();
        tokio::spawn(async move {
            let restore = this.run_restore(&plan, launch, &doc, info.as_ref()).await;
            this.restore_report(launch, &restore).await;
            this.inner.lock().unwrap().own.restoring = false;
        });
    }

    /// Tell the cell-control Lambda how the restore of `launch` ended (`restore_report`; its
    /// `restoreReport`, and on success its `lastBuild` / `lastPlanId`).
    async fn restore_report(&self, launch: u32, restore: &CellRestore) {
        match self.invoke(restore_report_payload(launch, restore)).await {
            Ok(body) if body["ok"] == true => {}
            Ok(body) => tracing::warn!("Virtual Cell: restore_report: {}", lambda_error(&body)),
            Err(err) => tracing::warn!("Virtual Cell: restore_report: {err:#}"),
        }
    }

    fn restored_file(&self) -> Option<u32> {
        let files = self.settings.wire.as_ref()?;
        std::fs::read_to_string(files.dir.join(RESTORED_FILE)).ok()?.trim().parse().ok()
    }

    fn write_restored(&self, launch: u32) {
        if let Some(files) = &self.settings.wire {
            let _ = std::fs::create_dir_all(&files.dir);
            let _ = std::fs::write(files.dir.join(RESTORED_FILE), format!("{launch}\n"));
        }
    }

    fn resumed_file(&self) -> Option<String> {
        let files = self.settings.wire.as_ref()?;
        Some(std::fs::read_to_string(files.dir.join(RESUMED_FILE)).ok()?.trim().to_owned()).filter(|s| !s.is_empty())
    }

    fn write_resumed(&self, key: &str) {
        if let Some(files) = &self.settings.wire {
            let _ = std::fs::create_dir_all(&files.dir);
            let _ = std::fs::write(files.dir.join(RESUMED_FILE), format!("{key}\n"));
        }
    }

    /// After a resume of the stopped cell (S4): the control code stayed on its disks (the head's
    /// boot application, the IDE's projects and libraries), so nothing is restored. In production
    /// when it stopped: `start` only (the PLC host just booted: no forced runtime restart, R7),
    /// then `krl` + `start`, then a full Run Control Code.
    async fn run_resume(self: &Arc<Self>, plan: &RestorePlan, launch: u32, doc: &Value) -> CellRestore {
        let root = cells::active_cell(&self.config).map(|(_, r)| r);
        let auto = root.as_deref().map(load_auto).unwrap_or_default();
        let build = auto.deployed_build.clone().or_else(|| doc["lastBuild"].as_str().map(str::to_owned));
        let plan_id = auto.status.deployed_plan_id.clone().or_else(|| doc["lastPlanId"].as_str().map(str::to_owned));
        let mut restore = CellRestore { state: JobState::Running, mode: "resume".to_owned(), build: build.clone(), plan_id: plan_id.clone(), message: None };
        let done = |this: &Self, mut r: CellRestore, state: JobState, message: String| {
            this.log(format!("virtual cell launch {launch}: resumed: {message}"));
            r.state = state;
            r.message = Some(message);
            this.set_restore(r.clone());
            r
        };
        let (Some(root), Some(build)) = (root, build) else {
            return done(self, restore, JobState::Done, "the stopped instances run again (no control code deployed yet)".to_owned());
        };
        if !plan.production {
            return done(
                self,
                restore,
                JobState::Done,
                format!("the stopped instances run again; the head runs {build} from its disk (not in production: Start production)"),
            );
        }
        if Self::export(&root, &build).is_err() {
            return done(self, restore, JobState::Failed, format!("the export {build} is not in the active cell: Run Control Code"));
        }
        self.set_restore(restore.clone());
        let kuka = self.hardware.cell_config_for(&root).ok().flatten().is_some_and(|c| has_kuka_vc(&c));
        let mut tries: Vec<Vec<String>> = vec![vec!["start".to_owned()]];
        if kuka {
            tries.push(vec!["krl".to_owned(), "start".to_owned()]);
        }
        let mut last = String::new();
        for stages in tries {
            let what = stages.join(" + ");
            restore.message = Some(format!("in production when it stopped: {what}…"));
            self.set_restore(restore.clone());
            match self.restore_job(DeployRequest { build: Some(build.clone()), start: true, stages: Some(stages), force: true }).await {
                Ok(message) => return done(self, restore, JobState::Done, format!("{what}: {message}")),
                Err(err) => {
                    self.log(format!("virtual cell launch {launch}: resume: {what} failed ({err}); next"));
                    last = err;
                }
            }
        }
        restore.message = Some(format!("{last}; Run Control Code…"));
        self.set_restore(restore.clone());
        match self.restore_job(DeployRequest { build: Some(build.clone()), start: true, stages: None, force: true }).await {
            Ok(message) => done(self, restore, JobState::Done, format!("Run Control Code: {message}")),
            Err(err) => done(self, restore, JobState::Failed, format!("{err} (Start fresh launches new instances)")),
        }
    }

    fn set_restore(&self, restore: CellRestore) {
        self.inner.lock().unwrap().own.restore = Some(restore.clone());
        self.update(|s| {
            if let Some(cell) = &mut s.virtual_cell {
                cell.restore = Some(restore);
            }
        });
    }

    /// Restore the last deployed control code onto a fresh launch: its outcome.
    async fn run_restore(self: &Arc<Self>, plan: &RestorePlan, launch: u32, doc: &Value, info: Option<&Value>) -> CellRestore {
        let root = cells::active_cell(&self.config).map(|(_, r)| r);
        let auto = root.as_deref().map(load_auto).unwrap_or_default();
        let build = auto.deployed_build.clone().or_else(|| doc["lastBuild"].as_str().map(str::to_owned));
        let plan_id = auto.status.deployed_plan_id.clone().or_else(|| doc["lastPlanId"].as_str().map(str::to_owned));
        let mut restore = CellRestore { state: JobState::Running, mode: plan.mode.clone(), build: build.clone(), plan_id: plan_id.clone(), message: None };
        let done = |this: &Self, mut r: CellRestore, state: JobState, message: String| {
            this.log(format!("virtual cell launch {launch}: restore ({}): {message}", r.mode));
            r.state = state;
            r.message = Some(message);
            this.set_restore(r.clone());
            r
        };
        if plan.mode == "none" {
            return done(self, restore, JobState::Done, "not asked for".to_owned());
        }
        let (Some(root), Some(build)) = (root, build) else {
            restore.mode = "none".to_owned();
            return done(self, restore, JobState::Done, "no control code deployed yet: nothing to restore".to_owned());
        };
        if Self::export(&root, &build).is_err() {
            restore.mode = "none".to_owned();
            return done(self, restore, JobState::Failed, format!("the export {build} is not in the active cell: Run Control Code"));
        }
        let fast = plan.mode == "auto" && boot_apps_restored(info, doc, &build, plan_id.as_deref(), launch);
        restore.mode = if fast { "fast" } else { "full" }.to_owned();
        self.set_restore(restore.clone());
        if fast {
            // The Lambda restored the head's and the robots' boot applications of this export.
            let _ = self.change_auto(&root, |a| {
                a.deployed_build = Some(build.clone());
                a.deploy_unknown = false;
                a.status.deployed_plan_id = plan_id.clone();
                a.production_build = Some(build.clone());
            });
            let mut stages = Vec::new();
            if self.hardware.cell_config_for(&root).ok().flatten().is_some_and(|c| has_kuka_vc(&c)) {
                stages.push("krl".to_owned());
            }
            if plan.production {
                stages.push("start".to_owned());
            }
            if stages.is_empty() {
                return done(self, restore, JobState::Done, format!("{build}: the head's boot application was restored"));
            }
            self.log(format!("virtual cell launch {launch}: restore {build}: boot applications restored, then {}", stages.join(", ")));
            match self.restore_job(DeployRequest { build: Some(build.clone()), start: plan.production, stages: Some(stages), force: true }).await {
                Ok(message) => return done(self, restore, JobState::Done, message),
                Err(err) => {
                    // Exit 8: a CODESYS tool failed or the build was rejected (a library the fresh
                    // IDE lacks): deterministic, the fallback's `projects` downloads it.
                    let kind = if err.contains("(exit 8)") { "a CODESYS tool / build failure" } else { "failed" };
                    self.log(format!("virtual cell launch {launch}: fast restore: {kind} ({err}); CODESYS IDE reset, then Deploy only"));
                    // The fallback runs whatever the reset says (today's behaviour without it).
                    if let Err(why) = self.restore_reset_ide(&root, &mut restore).await {
                        self.log(format!("virtual cell launch {launch}: CODESYS IDE reset failed ({why}); Deploy only anyway"));
                    }
                }
            }
            restore.mode = "full".to_owned();
            self.set_restore(restore.clone());
        }
        let full = DeployRequest { build: Some(build.clone()), start: plan.production, stages: None, force: true };
        let mut result = self.restore_job(full.clone()).await;
        // One bounded retry: a CODESYS tool error (IDE state after many failed logins) gets an IDE
        // reset and one more run; project errors and a partly downloaded head do not.
        if let Err(err) = &result
            && retry_after_ide_reset(err, load_auto(&root).deploy_unknown)
        {
            self.log(format!("virtual cell launch {launch}: restore failed with a CODESYS tool error ({err}); CODESYS IDE reset, then once more"));
            result = match self.restore_reset_ide(&root, &mut restore).await {
                Ok(_) => self.restore_job(full).await,
                Err(why) => Err(format!("{err}; the CODESYS IDE reset failed too: {why}")),
            };
        }
        match result {
            Ok(message) => done(self, restore, JobState::Done, message),
            Err(err) => done(self, restore, JobState::Failed, err),
        }
    }

    /// [`Self::reset_ide`] during a restore, with the restore's status saying so.
    async fn restore_reset_ide(self: &Arc<Self>, root: &Path, restore: &mut CellRestore) -> Result<String, String> {
        if !self.needs_mcp_token() {
            return Ok("no CODESYS IDE in this virtual cell (its head is deployed through its hook)".to_owned());
        }
        restore.message = Some("resetting the CODESYS IDE (plan2cell cell reset-ide)…".to_owned());
        self.set_restore(restore.clone());
        let result = self.reset_ide(root).await;
        restore.message = None;
        result
    }

    /// `plan2cell cell reset-ide`: disconnect, close the projects unsaved, shut CODESYS down and
    /// launch it again, until its status is healthy (exit 0). Holds the cell (a job, waiting for
    /// one that runs) and its lease: it drops an operator's DCV view of the IDE.
    async fn reset_ide(self: &Arc<Self>, root: &Path) -> Result<String, String> {
        let binary = self.plan2cell().map_err(|(_, err)| err)?;
        let cell_config = self.cell_config(root).map_err(|(_, err)| err)?;
        let mut tries = 0;
        loop {
            match self.claim("reset-ide", "resetting the CODESYS IDE") {
                Ok(()) => break,
                Err((_, err)) if tries < 60 => {
                    tries += 1;
                    tracing::debug!("Virtual Cell: the IDE reset waits: {err}");
                    tokio::time::sleep(self.settings.poll.max(Duration::from_millis(50))).await;
                }
                Err((_, err)) => return Err(err),
            }
        }
        let this = self.clone();
        let result = self
            .leased("reset-ide", false, async move {
                let token = this.secret(&this.mcp_secret()).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
                this.run_reset_ide(&binary, &cell_config, &token).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))
            })
            .await;
        self.release();
        match result {
            Ok(note) => {
                self.log(format!("CODESYS IDE reset: {note}"));
                Ok(note)
            }
            Err((_, err)) => Err(err),
        }
    }

    /// `plan2cell cell kvc <robot> <op> [arg]` on the active cell's virtual cell: its JSON. An
    /// operation that changes the controller waits for no job: it is refused (`409`) while one runs,
    /// and holds the cell while it runs.
    pub async fn kvc_op(self: &Arc<Self>, request: KvcRequest) -> Result<Value, HttpError> {
        let name_ok = |s: &str| !s.is_empty() && s.len() <= 64 && s.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '=' | ':' | '\\'));
        if !name_ok(&request.robot) || !name_ok(&request.op) || request.arg.as_deref().is_some_and(|a| !name_ok(a)) {
            return Err((StatusCode::BAD_REQUEST, "robot, op and arg: names only (see the tool's description)".to_owned()));
        }
        let (_, root) = cells::active_cell(&self.config).ok_or((StatusCode::CONFLICT, "no active cell".to_owned()))?;
        let binary = self.plan2cell()?;
        let cell_config = self.cell_config(&root)?;
        let changes = self.kvc_changes(&request)?;
        if changes {
            self.claim("kvc", &format!("kvc {} {}", request.op, request.arg.as_deref().unwrap_or_default()))?;
        }
        let mut args: Vec<String> = vec!["cell".into(), "--cell-config".into(), cell_config.display().to_string(), "kvc".into(), request.robot.clone(), request.op.clone()];
        args.extend(request.arg.clone());
        let mut env = Vec::new();
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        self.log(format!("KUKA VC {}: {} {}", request.robot, request.op, request.arg.as_deref().unwrap_or_default()));
        let out = run(&binary, &args, &env, self.settings.stage_timeout, Some(&*self.sink)).await;
        if changes {
            self.release();
        }
        let out = out.map_err(|err| (StatusCode::BAD_GATEWAY, format!("plan2cell cell kvc: {err:#}")))?;
        if out.code == Some(2) {
            let why = out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("usage error").to_owned();
            return Err((StatusCode::BAD_REQUEST, why));
        }
        let mut answer: Value = serde_json::from_slice(&out.stdout).unwrap_or_else(|_| json!({"ok": false}));
        if answer.get("error").is_none() && out.code != Some(0) {
            answer["stderr"] = json!(out.stderr_tail.lines().rev().take(10).collect::<Vec<_>>());
        }
        Ok(answer)
    }

    /// `plan2cell cell reset-ide --cell-config <cell.toml> --json`: its last note, or why it failed.
    async fn run_reset_ide(&self, binary: &Path, cell_config: &Path, token: &str) -> anyhow::Result<String> {
        let args: Vec<String> =
            vec!["cell".into(), "reset-ide".into(), "--cell-config".into(), cell_config.display().to_string(), "--json".into()];
        let mut env = vec![(MCP_TOKEN_ENV.to_owned(), token.to_owned())];
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        let out = run(binary, &args, &env, self.settings.stage_timeout, Some(&*self.sink)).await.context("plan2cell cell reset-ide")?;
        let report: Value = serde_json::from_slice(&out.stdout).unwrap_or(Value::Null);
        let stages = || report["stages"].as_array().into_iter().flatten();
        if out.code == Some(0) && report["ok"] != false {
            let note = stages().flat_map(|s| s["notes"].as_array().into_iter().flatten()).filter_map(Value::as_str).next_back();
            return Ok(note.unwrap_or("the CODESYS IDE runs again").to_owned());
        }
        let error = stages()
            .find_map(|s| s["error"].as_str().map(str::to_owned))
            .or_else(|| report["error"].as_str().map(str::to_owned))
            .or_else(|| out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).map(str::to_owned))
            .unwrap_or_else(|| "no report".to_owned());
        let code = out.code.map_or("killed".to_owned(), |c| c.to_string());
        let why = match out.code {
            Some(2) => " - a usage error (plan2cell too old?)",
            Some(3) => " - an unanswered CODESYS dialog, answer it in the IDE",
            Some(4) => " - the MCP bridge is unreachable",
            Some(8) => " - the IDE is not ready",
            _ => "",
        };
        anyhow::bail!("plan2cell cell reset-ide failed (exit {code}{why}): {error}")
    }

    /// Run Control Code / Deploy only for a restore and wait for it: its message.
    async fn restore_job(self: &Arc<Self>, request: DeployRequest) -> Result<String, String> {
        let purpose = if request.start { "deploy" } else { "deploy-only" };
        let mut tries = 0;
        loop {
            match self.leased(purpose, true, self.deploy(request.clone())).await {
                Ok(_) => break,
                // Another job (e.g. auto-restart) holds the cell: wait for it.
                Err((StatusCode::CONFLICT, err)) if tries < 60 && self.busy() => {
                    tries += 1;
                    tracing::debug!("Virtual Cell: restore waits: {err}");
                    tokio::time::sleep(self.settings.poll.max(Duration::from_millis(50))).await;
                }
                Err((_, err)) => return Err(err),
            }
        }
        self.follow_job("deploy").await;
        let deploy = self.status().deploy;
        let message = deploy.message.unwrap_or_default();
        match deploy.state {
            JobState::Done => Ok(message),
            _ => Err(message),
        }
    }

    /// `start` / `stop` (all roles, or those named). A start of the whole cell then waits for the
    /// instances, brings up the controllers and maybe starts production (see the module docs).
    pub async fn start_stop(self: &Arc<Self>, action: &str, request: StartStopRequest) -> Result<InfraStatus, HttpError> {
        self.function()?;
        let mut payload = json!({ "action": action });
        let request_roles = request.roles.clone();
        if let Some(roles) = &request.roles {
            if roles.is_empty() || !roles.iter().all(|r| known_role(r)) {
                return Err((StatusCode::BAD_REQUEST, format!("roles must be a non-empty list of {}, kukaVcN", ROLES.join(", "))));
            }
            payload["roles"] = json!(roles);
        }
        let plc_host = request_roles.as_ref().is_none_or(|r| r.iter().any(|r| r == ROLE_PLC_HOST));
        // The PLC host was stopped already: a second Stop cell keeps what the first one remembered.
        let host_was_stopped = self.status().instance(ROLE_PLC_HOST).is_some_and(|h| h.state == "stopped");
        if action == "stop" {
            let mut inner = self.inner.lock().unwrap();
            inner.stops += 1;
            inner.power_follow = None;
        }
        let body = self.invoke(payload).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
        if body["ok"] != true {
            return Err((StatusCode::BAD_GATEWAY, format!("cell control {action}: {}", lambda_error(&body))));
        }
        if action == "start" {
            let starts = instance_starts(&body);
            for i in &starts {
                self.log(format!("start: {}", i.summary()));
            }
            self.inner.lock().unwrap().transition = Some((action.to_owned(), Instant::now()));
            self.after_start(request, starts).await;
        } else {
            let changes: Vec<String> = body["instances"]
                .as_array()
                .into_iter()
                .flatten()
                .map(|i| {
                    format!(
                        "{} {} → {}",
                        i["role"].as_str().unwrap_or("?"),
                        i["previousState"].as_str().unwrap_or("?"),
                        i["currentState"].as_str().unwrap_or("?")
                    )
                })
                .collect();
            self.log(format!("{action}: {}", changes.join(", ")));
            // A stopped PLC host is not meant to be in production any more: auto-restart leaves it.
            // Whether it was is remembered: the next Start cell starts production again.
            if plc_host && let Some((_, root)) = cells::active_cell(&self.config) {
                let _ = self.change_auto(&root, |a| {
                    if !host_was_stopped {
                        a.stopped_in_production = a.in_production;
                    }
                    a.in_production = false;
                });
            }
            self.inner.lock().unwrap().transition = Some((action.to_owned(), Instant::now()));
        }
        self.refresh().await;
        Ok(self.status())
    }

    /// `GET` [`api::INFRA_CELL_POWER`]: the cell's power, read from the Lambda now.
    pub async fn cell_power(&self) -> Result<CellPower, HttpError> {
        self.function()?;
        self.inner.lock().unwrap().interest = Some(Instant::now());
        self.refresh().await;
        let status = self.status();
        status.cell_power.ok_or_else(|| match status.message {
            Some(message) => (StatusCode::BAD_GATEWAY, message),
            None => (
                StatusCode::NOT_IMPLEMENTED,
                "the cell-control Lambda has no power actions: deploy Plan2cellCellControl (plan2cell/infra)".to_owned(),
            ),
        })
    }

    /// `POST` [`api::INFRA_CELL_POWER`]: Start / Stop virtual cell through the Lambda (see the
    /// module docs).
    pub async fn power(self: &Arc<Self>, request: CellPowerRequest) -> Result<InfraStatus, HttpError> {
        self.function()?;
        let unsupported = || {
            (StatusCode::NOT_IMPLEMENTED, "the cell-control Lambda has no power actions: deploy Plan2cellCellControl (plan2cell/infra)".to_owned())
        };
        let root = cells::active_cell(&self.config).map(|(_, r)| r);
        let own = self.settings.own();
        match request.action.as_str() {
            "on" => {
                let mode = request.restore.clone().unwrap_or_else(|| "auto".to_owned());
                if !RESTORE_MODES.contains(&mode.as_str()) {
                    return Err((StatusCode::BAD_REQUEST, format!("restore must be one of {}, not {mode:?}", RESTORE_MODES.join(", "))));
                }
                let mut payload = json!({"action": "power_on", "holder": self.settings.holder});
                if own {
                    // The export to restore: the one last deployed here, else the Lambda's last report.
                    let auto = root.as_deref().map(load_auto).unwrap_or_default();
                    let doc = self.inner.lock().unwrap().own.doc.clone().unwrap_or(Value::Null);
                    let build = auto.deployed_build.clone().or_else(|| doc["lastBuild"].as_str().map(str::to_owned));
                    let plan_id = auto.status.deployed_plan_id.clone().or_else(|| doc["lastPlanId"].as_str().map(str::to_owned));
                    // Only what is known (a first start has neither; the Lambda wants strings).
                    let mut restore = json!({"mode": mode});
                    if let Some(build) = build {
                        restore["build"] = json!(build);
                    }
                    if let Some(plan_id) = plan_id {
                        restore["planId"] = json!(plan_id);
                    }
                    payload["restore"] = restore;
                    if let Some(fresh) = request.fresh {
                        payload["fresh"] = json!(fresh);
                    }
                }
                let body = self.invoke(payload).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
                match body["error"]["code"].as_str() {
                    _ if body["ok"] == true => {}
                    Some("unknown-action" | "not-configured") => return Err(unsupported()),
                    Some("stopping" | "not-usable") => return Err((StatusCode::CONFLICT, lambda_error(&body))),
                    Some(_) if own => return Err(cell_error("power_on", &body)),
                    // Some instances did not start: the others did; the status says which.
                    Some("start-failed") => self.log(format!("Start virtual cell: {}", lambda_error(&body))),
                    _ => return Err((StatusCode::BAD_GATEWAY, format!("cell control power_on: {}", lambda_error(&body)))),
                }
                // Only the supervisor resumes production by itself (an operator when asked to).
                let was_producing =
                    self.settings.role == CellRole::Supervisor && root.as_deref().is_some_and(|r| load_auto(r).stopped_in_production);
                let production = request.production.unwrap_or(was_producing);
                if let Some(root) = &root {
                    let _ = self.change_auto(root, |a| a.stopped_in_production = false);
                }
                {
                    let mut inner = self.inner.lock().unwrap();
                    if own {
                        // The restore after the launch starts production when it is due.
                        inner.power_follow = None;
                        inner.own.restore_plan = (body["ok"] == true).then(|| RestorePlan { mode: mode.clone(), production });
                        inner.own.restore = None;
                    } else {
                        inner.power_follow = (body["ok"] == true).then_some(PowerFollow { production, since: Instant::now() });
                    }
                    inner.touched = Some(Instant::now());
                    inner.power_unsupported = None;
                }
                if own && body["resumed"] == true {
                    self.log(format!(
                        "Start virtual cell: the cell-control Lambda resumes the stopped instances (no first boot, no restore: the \
                         control code is on their disks), then the KUKA bring-up and the readiness checks{}",
                        if production { "; then Start production" } else { "" }
                    ));
                } else if own {
                    let why = body["fresh"]["reason"].as_str().map(|r| format!(" ({r})")).unwrap_or_default();
                    self.log(format!(
                        "Start virtual cell: the cell-control Lambda launches fresh instances from the latest images{why}, then the \
                         KUKA bring-up and the readiness checks; then the restore of the control code ({mode}){}",
                        if production { " and Start production" } else { "" }
                    ));
                } else {
                    self.log(format!(
                        "Start virtual cell: the cell-control Lambda starts every instance, then the KUKA bring-up and the readiness checks{}",
                        if production { "; then Start production" } else { "" }
                    ));
                }
            }
            "off" => {
                let status = self.status();
                if status.in_production && !request.force {
                    return Err((
                        StatusCode::CONFLICT,
                        "the cell is in production: Stop production first (step 5), or stop the virtual cell anyway (force)".to_owned(),
                    ));
                }
                let running = self.inner.lock().unwrap().job;
                if let Some(job) = running.filter(|_| !request.force) {
                    return Err((StatusCode::CONFLICT, format!("{job} is running on the cell; wait for it, or stop the virtual cell anyway (force)")));
                }
                let host_was_stopped = status.instance(ROLE_PLC_HOST).is_some_and(|h| h.state == "stopped");
                {
                    let mut inner = self.inner.lock().unwrap();
                    inner.stops += 1;
                    inner.power_follow = None;
                }
                let mut payload = json!({"action": "power_off", "holder": self.settings.holder, "force": request.force, "inProduction": status.in_production});
                if own {
                    // The Lambda archives its PLC data first (in both modes: a later fresh start restores it).
                    payload["archive"] = json!(true);
                    if let Some(mode) = &request.mode {
                        if !OFF_MODES.contains(&mode.as_str()) {
                            return Err((StatusCode::BAD_REQUEST, format!("mode must be one of {}, not {mode:?}", OFF_MODES.join(", "))));
                        }
                        payload["mode"] = json!(mode);
                    }
                    self.inner.lock().unwrap().own.restore_plan = None;
                }
                let body = self.invoke(payload).await.map_err(|err| (StatusCode::BAD_GATEWAY, format!("{err:#}")))?;
                if body["ok"] != true {
                    return Err(match body["error"]["code"].as_str() {
                        Some("unknown-action" | "not-configured") => unsupported(),
                        Some("in-production") => (StatusCode::CONFLICT, lambda_error(&body)),
                        Some(_) if own => cell_error("power_off", &body),
                        _ => (StatusCode::BAD_GATEWAY, format!("cell control power_off: {}", lambda_error(&body))),
                    });
                }
                // As Stop cell: out of production, remembered for the next start.
                if let Some(root) = &root {
                    let _ = self.change_auto(root, |a| {
                        if !host_was_stopped {
                            a.stopped_in_production = a.in_production;
                        }
                        a.in_production = false;
                    });
                }
                self.inner.lock().unwrap().transition = Some(("stop".to_owned(), Instant::now()));
                self.log(format!("Stop virtual cell{}", if request.force { " (force)" } else { "" }));
            }
            other => return Err((StatusCode::BAD_REQUEST, format!("action must be on or off, not {other:?}"))),
        }
        self.refresh().await;
        Ok(self.status())
    }

    /// Tell the Lambda this backend uses the cell (its idle stop), every [`TOUCH_EVERY`] while a
    /// viewer asks, a job runs or the cell is in production.
    async fn touch_if_due(&self) {
        let due = {
            let mut inner = self.inner.lock().unwrap();
            let using = inner.interest.is_some_and(|at| at.elapsed() < INTEREST) || inner.job.is_some() || inner.status.in_production;
            let due = using && inner.status.cell_power.is_some() && inner.touched.is_none_or(|at| at.elapsed() >= TOUCH_EVERY);
            if due {
                inner.touched = Some(Instant::now());
            }
            due
        };
        if !due {
            return;
        }
        let in_production = self.status().in_production;
        let mut payload = json!({"action": "touch", "holder": self.settings.holder, "inProduction": in_production, "role": self.settings.role.as_str()});
        // An own cell's Lambda keeps the export last deployed for the next launch's restore.
        if self.settings.own()
            && let Some((_, root)) = cells::active_cell(&self.config)
        {
            let auto = load_auto(&root);
            if let Some(build) = &auto.deployed_build {
                payload["build"] = json!(build);
            }
            if let Some(plan_id) = &auto.status.deployed_plan_id {
                payload["planId"] = json!(plan_id);
            }
        }
        match self.invoke(payload).await {
            Ok(body) if body["ok"] == true => {}
            Ok(body) => tracing::debug!("Virtual Cell: touch: {}", lambda_error(&body)),
            Err(err) => tracing::debug!("Virtual Cell: touch: {err:#}"),
        }
    }

    /// A Start virtual cell's follow-up: once the cell is ready, Start production if asked for.
    async fn follow_power(self: &Arc<Self>) {
        if self.settings.own() {
            return self.follow_restore().await;
        }
        let (follow, power) = {
            let inner = self.inner.lock().unwrap();
            (inner.power_follow, inner.status.cell_power.clone())
        };
        let Some(follow) = follow else { return };
        let phase = power.as_ref().map(|p| p.state);
        let give_up = |this: &Self, why: String| {
            this.inner.lock().unwrap().power_follow = None;
            this.log(format!("Start virtual cell: {why}"));
        };
        match phase {
            Some(PowerState::Ready) => {
                self.inner.lock().unwrap().power_follow = None;
                if !follow.production {
                    self.log("Start virtual cell: the cell is ready");
                    return;
                }
                self.log("Start virtual cell: the cell is ready; Start production");
                if let Err((_, err)) = self.leased("production", true, self.production(ProductionRequest::default())).await {
                    self.log(format!("Start virtual cell: Start production: {err}"));
                }
            }
            Some(PowerState::Failed | PowerState::Off | PowerState::Stopping) => {
                let message = power.and_then(|p| p.message).unwrap_or_default();
                give_up(self, format!("the cell did not get ready ({message}); no Start production"));
            }
            _ if follow.since.elapsed() > POWER_FOLLOW => give_up(self, "the cell was not ready in time; no Start production".to_owned()),
            _ => {}
        }
    }

    /// After the Lambda's start: the per-instance results into [`InfraStatus::start`] and, for the
    /// whole cell, the chain (wait, bring-up, production) as a job.
    async fn after_start(self: &Arc<Self>, request: StartStopRequest, starts: Vec<InstanceStart>) {
        let whole = request.roles.is_none();
        let root = cells::active_cell(&self.config).map(|(_, r)| r);
        // Only the supervisor resumes production by itself (an operator when asked to).
        let was_producing =
            whole && self.settings.role == CellRole::Supervisor && root.as_deref().is_some_and(|r| load_auto(r).stopped_in_production);
        let bring_up = request.bring_up.unwrap_or(whole) || request.production == Some(true);
        let production = bring_up && request.production.unwrap_or(was_producing);
        // Used up by this start (also when it does not follow it: the user said so).
        if whole && let Some(root) = &root {
            let _ = self.change_auto(root, |a| a.stopped_in_production = false);
        }
        let started = unix_s(SystemTime::now());
        let failed: Vec<String> = starts.iter().filter(|i| i.failed()).map(InstanceStart::summary).collect();
        let fallbacks: Vec<String> = starts.iter().filter(|i| i.result == "fallback").map(InstanceStart::summary).collect();
        let ok = starts.iter().filter(|i| matches!(i.result.as_str(), "started" | "fallback")).count();
        let mut summary = format!("{ok} of {} instances started", starts.len());
        if !fallbacks.is_empty() {
            summary.push_str(&format!(" ({})", fallbacks.join("; ")));
        }
        let mut job = CellStartStatus {
            state: JobState::Done,
            message: None,
            instances: starts,
            bring_up,
            production,
            started_s: Some(started),
            finished_s: None,
        };
        let claimed = if !bring_up {
            if failed.is_empty() {
                job.message = Some(summary.clone());
            } else {
                job.state = JobState::Failed;
                job.message = Some(format!("{summary}; {}", failed.join("; ")));
            }
            None
        } else if !failed.is_empty() {
            job.state = JobState::Failed;
            job.message = Some(format!("{}; not bringing up the controllers: start it again, or Bring up controllers once it runs", failed.join("; ")));
            None
        } else {
            match self.claim("cell-start", "Start cell's bring-up") {
                Ok(()) => {
                    job.state = JobState::Waiting;
                    job.message = Some(format!("{summary}; waiting until every instance runs and is SSM-online…"));
                    Some(self.inner.lock().unwrap().stops)
                }
                Err((_, err)) => {
                    job.message = Some(format!("{summary}; not bringing up the controllers: {err}"));
                    None
                }
            }
        };
        if claimed.is_none() {
            job.finished_s = Some(started);
        }
        let line = job.message.clone().unwrap_or_default();
        self.update(|s| s.start = job);
        self.log(format!("Start cell: {line}"));
        let Some(stops) = claimed else { return };
        let this = self.clone();
        tokio::spawn(async move {
            let result = this.start_chain(stops, production).await;
            let mut inner = this.inner.lock().unwrap();
            if inner.job == Some("cell-start") {
                inner.job = None;
            }
            let st = &mut inner.status.start;
            st.finished_s = Some(unix_s(SystemTime::now()));
            match result {
                Ok(message) => {
                    st.state = JobState::Done;
                    st.message = Some(message);
                }
                Err(err) => {
                    st.state = JobState::Failed;
                    st.message = Some(format!("{err:#}"));
                }
            }
            let line = format!("[infra] Start cell: {}", st.message.clone().unwrap_or_default());
            inner.status.cell.demo_note = Some(demo_note(&inner.status, this.settings.semantic_hosts));
            this.sink.status(&inner.status);
            this.sink.log(line);
        });
    }

    /// Start cell's chain, holding the cell (`cell-start`) while it waits: every instance running and
    /// SSM-online, then the bring-up, then (`production`) Start production, each followed to its end.
    async fn start_chain(self: &Arc<Self>, stops: u64, production: bool) -> anyhow::Result<String> {
        let begun = Instant::now();
        let deadline = begun + self.settings.start_timeout;
        let stopped = || self.inner.lock().unwrap().stops != stops;
        loop {
            if stopped() {
                anyhow::bail!("Stop cell was asked for: Start cell stopped waiting for the instances");
            }
            self.refresh().await;
            let status = self.status();
            if status.ready {
                break;
            }
            let gone: Vec<String> = status
                .instances
                .iter()
                .filter(|i| matches!(i.state.as_str(), "terminated" | "shutting-down" | "not-found"))
                .map(|i| format!("{} is {}", role_label(&i.role), i.state))
                .collect();
            // Right after StartInstances EC2 may still say stopped for a moment.
            let stopped_now: Vec<String> = status
                .instances
                .iter()
                .filter(|i| matches!(i.state.as_str(), "stopped" | "stopping"))
                .map(|i| format!("{} is {}", role_label(&i.role), i.state))
                .collect();
            if !gone.is_empty() || (!stopped_now.is_empty() && begun.elapsed() > Duration::from_secs(90)) {
                anyhow::bail!("{}: Start cell again", [gone, stopped_now].concat().join(", "));
            }
            let ready = status.instances.iter().filter(|i| i.running() && i.online()).count();
            let waiting: Vec<String> = status
                .instances
                .iter()
                .filter(|i| !i.running() || !i.online())
                .map(|i| {
                    if i.running() { format!("{} running, SSM {}", role_label(&i.role), i.ssm_ping) } else { format!("{} {}", role_label(&i.role), i.state) }
                })
                .collect();
            let secs = begun.elapsed().as_secs();
            if Instant::now() > deadline {
                anyhow::bail!(
                    "not every instance was running and SSM-online after {} min ({}); Bring up controllers once they are",
                    self.settings.start_timeout.as_secs().div_ceil(60),
                    waiting.join(", ")
                );
            }
            self.update(|s| {
                s.start.message = Some(format!(
                    "Waiting for the instances ({ready} of {} ready, {}:{:02}): {}…",
                    status.instances.len(),
                    secs / 60,
                    secs % 60,
                    waiting.join(", ")
                ));
            });
            tokio::time::sleep(self.settings.poll).await;
        }
        let up = begun.elapsed().as_secs();
        self.update(|s| {
            s.start.state = JobState::Running;
            s.start.message = Some(format!("Every instance runs and is SSM-online ({}:{:02}); bringing up the controllers…", up / 60, up % 60));
        });
        // The cell goes from Start cell's hold to the bring-up's.
        self.release();
        self.bringup(BringupRequest::default()).await.map_err(|(_, err)| anyhow::anyhow!("bring-up: {err}"))?;
        self.follow_job("bring-up").await;
        let bringup = self.status().bringup;
        if bringup.state != JobState::Done {
            anyhow::bail!("instances up, bring-up failed: {}", bringup.message.unwrap_or_default());
        }
        if !production {
            return Ok(format!("cell up, controllers brought up: {}", bringup.message.unwrap_or_default()));
        }
        if stopped() {
            anyhow::bail!("Stop cell was asked for: no Start production");
        }
        self.update(|s| s.start.message = Some("Controllers up; starting production…".to_owned()));
        self.production(ProductionRequest::default()).await.map_err(|(_, err)| anyhow::anyhow!("controllers up, Start production: {err}"))?;
        self.follow_job("production").await;
        let cell = self.status().cell;
        if cell.state != JobState::Done {
            anyhow::bail!("controllers up, Start production failed: {}", cell.message.unwrap_or_default());
        }
        Ok(format!("cell up and in production: {}", cell.message.unwrap_or_default()))
    }

    /// Until the job `kind` (started just now) has finished.
    async fn follow_job(&self, kind: &str) {
        while self.inner.lock().unwrap().job == Some(kind) {
            tokio::time::sleep(self.settings.poll).await;
        }
    }

    /// Hold the cell for job `kind`; `409` while another one runs.
    fn claim(&self, kind: &'static str, what: &str) -> Result<(), HttpError> {
        let mut inner = self.inner.lock().unwrap();
        if let Some(running) = inner.job {
            let running = match running {
                "bring-up" => "the controller bring-up",
                "deploy" => "Run Control Code (deploy)",
                "production-stop" => "Stop production",
                "reset-ide" => "the CODESYS IDE reset",
                "kvc" => "a KUKA VC operation",
                "cell-start" => "Start cell (waiting for the instances)",
                _ => "production start",
            };
            return Err((StatusCode::CONFLICT, format!("{running} is running; wait for it before {what}")));
        }
        inner.job = Some(kind);
        Ok(())
    }

    fn release(&self) {
        self.inner.lock().unwrap().job = None;
    }

    /// With the cell held: read the status; `409` (and the cell released) unless every instance
    /// runs and is SSM-online.
    async fn ready_or_release(&self) -> Result<(), HttpError> {
        self.refresh().await;
        let status = self.status();
        if status.ready {
            return Ok(());
        }
        self.release();
        let waiting: Vec<String> = status
            .instances
            .iter()
            .filter(|i| !i.running() || !i.online())
            .map(|i| format!("{} {}/{}", factoryos_protocol::infra::role_label(&i.role), i.state, i.ssm_ping))
            .collect();
        Err((StatusCode::CONFLICT, format!("the cell is not ready ({}): Start cell, then Bring up controllers", waiting.join(", "))))
    }

    /// Start the KUKA VC bring-up: `202` (see [`InfraStatus::bringup`]).
    pub async fn bringup(self: &Arc<Self>, request: BringupRequest) -> Result<InfraStatus, HttpError> {
        self.function()?;
        if let Some(why) = self.lacks("bringup.action", "bring-up") {
            return Err((StatusCode::BAD_REQUEST, format!("{why}: Stop and Start the cell to repeat its bring-up")));
        }
        self.claim("bring-up", "bringing up the controllers")?;
        self.refresh().await;
        let stopped: Vec<String> = self
            .status()
            .instances
            .iter()
            .filter(|i| matches!(i.state.as_str(), "stopped" | "stopping" | "shutting-down" | "terminated" | "not-found"))
            .map(|i| format!("{} is {}", factoryos_protocol::infra::role_label(&i.role), i.state))
            .collect();
        if !stopped.is_empty() {
            self.release();
            return Err((StatusCode::CONFLICT, format!("{}: Start cell first (it brings up the controllers too)", stopped.join(", "))));
        }
        let started = unix_s(SystemTime::now());
        self.update(|s| {
            s.bringup = factoryos_protocol::infra::BringupStatus {
                state: JobState::Waiting,
                message: Some("Waiting until every instance runs and is SSM-online…".to_owned()),
                started_s: Some(started),
                ..Default::default()
            }
        });
        self.log(format!("bring-up (restart controllers: {})", request.restart_controllers));
        let this = self.clone();
        tokio::spawn(async move {
            let result = this.run_bringup(request).await;
            let mut inner = this.inner.lock().unwrap();
            inner.job = None;
            let b = &mut inner.status.bringup;
            match result {
                Ok(message) => {
                    b.state = JobState::Done;
                    b.message = Some(message);
                }
                Err(err) => {
                    b.state = JobState::Failed;
                    b.message = Some(format!("{err:#}"));
                }
            }
            let line = format!("[infra] bring-up: {}", b.message.clone().unwrap_or_default());
            inner.status.cell.demo_note = Some(demo_note(&inner.status, this.settings.semantic_hosts));
            this.sink.status(&inner.status);
            this.sink.log(line);
        });
        Ok(self.status())
    }

    async fn run_bringup(&self, request: BringupRequest) -> anyhow::Result<String> {
        let started = Instant::now();
        let deadline = started + self.settings.wait_timeout;
        let command_id = loop {
            if !self.status().ready {
                self.refresh().await;
            }
            let status = self.status();
            if let Some(i) = status.instances.iter().find(|i| matches!(i.state.as_str(), "stopped" | "stopping" | "terminated")) {
                anyhow::bail!("{} is {}: Start cell first", factoryos_protocol::infra::role_label(&i.role), i.state);
            }
            if status.ready {
                self.update(|s| {
                    s.bringup.state = JobState::Running;
                    s.bringup.message = Some(if request.restart_controllers {
                        "Restarting the KUKA controllers (kvc-restart.sh on every VC host), then the health check…".to_owned()
                    } else {
                        "KUKA VC health check on every host (kvc-cell-setup.sh check; up only if it fails; kvc-ack.sh)…".to_owned()
                    });
                });
                let body = self.invoke(json!({"action": "bringup", "restartControllers": request.restart_controllers})).await?;
                if body["ok"] == true {
                    break body["commandId"].as_str().context("bringup answered without a commandId")?.to_owned();
                }
                // One bring-up at a time on the VC host (e.g. one started before this backend restarted):
                // follow that one instead of starting a second.
                if body["error"]["code"] == "busy"
                    && let Some(running) = body["commandId"].as_str()
                {
                    self.log(format!("a bring-up is already running on the KUKA VC host (SSM command {running}); following it"));
                    self.update(|s| s.bringup.message = Some("A bring-up is already running on the KUKA VC host; following it…".to_owned()));
                    break running.to_owned();
                }
                if body["error"]["code"] != "not-ready" {
                    anyhow::bail!("cell control bringup: {}", lambda_error(&body));
                }
            } else {
                let waiting: Vec<String> = status
                    .instances
                    .iter()
                    .filter(|i| !i.running() || !i.online())
                    .map(|i| format!("{} {}/{}", factoryos_protocol::infra::role_label(&i.role), i.state, i.ssm_ping))
                    .collect();
                self.update(|s| s.bringup.message = Some(format!("Waiting for {}…", waiting.join(", "))));
            }
            if Instant::now() > deadline {
                anyhow::bail!("the instances were not all running and SSM-online after {} s", self.settings.wait_timeout.as_secs());
            }
            tokio::time::sleep(self.settings.poll).await;
        };
        self.update(|s| s.bringup.command_id = Some(command_id.clone()));
        let deadline = Instant::now() + self.settings.bringup_timeout;
        loop {
            tokio::time::sleep(self.settings.poll).await;
            let body = self.invoke(json!({"action": "bringupResult", "commandId": command_id})).await?;
            if body["ok"] != true {
                anyhow::bail!("cell control bringupResult: {}", lambda_error(&body));
            }
            let ssm = body["status"].as_str().unwrap_or("?").to_owned();
            let tail = body["stdoutTail"].as_str().map(|t| tail_chars(t, TAIL));
            let result = body["result"].as_str().map(str::to_owned);
            self.update(|s| {
                s.bringup.output_tail = tail.clone();
                s.bringup.result = result.clone();
                if body["done"] != true {
                    s.bringup.message = Some(format!("KUKA VC bring-up: SSM {ssm}…"));
                }
            });
            if body["done"] == true {
                let code = body["responseCode"].as_i64();
                return match (ssm.as_str(), result.as_deref()) {
                    ("Success", Some("healthy")) => Ok("KUKA controllers healthy (check passed)".to_owned()),
                    ("Success", Some("repaired")) => {
                        Ok("KUKA controllers brought up (restart / kvc-cell-setup.sh up / stop messages acknowledged in T1); check passed".to_owned())
                    }
                    // Exit 0 without the marker (SSM keeps only the first 24,000 characters of stdout).
                    ("Success", Some("ok")) | ("Success", None) if code.unwrap_or(0) == 0 => {
                        Ok("KUKA VC bring-up finished (exit 0; its result line was not in the SSM output)".to_owned())
                    }
                    (_, Some("busy")) => anyhow::bail!(
                        "KUKA VC bring-up busy: another bring-up or kvc-cell-setup.sh is running on the VC host; try again when it has finished"
                    ),
                    (_, result) => {
                        let last = tail.as_deref().and_then(|t| t.lines().rev().find(|l| !l.trim().is_empty() && !l.starts_with("BRINGUP_RESULT")));
                        anyhow::bail!(
                            "KUKA VC bring-up {} (SSM {ssm}){}",
                            result.unwrap_or("failed"),
                            last.map(|l| format!(": {l}")).unwrap_or_default()
                        )
                    }
                };
            }
            if Instant::now() > deadline {
                anyhow::bail!("the bring-up did not finish within {} s (SSM command {command_id})", self.settings.bringup_timeout.as_secs());
            }
        }
    }

    fn plan2cell(&self) -> Result<PathBuf, HttpError> {
        let binary = self.plan2cell.clone().or_else(|| self.plan2cell_lookup.and_then(|lookup| lookup())).ok_or_else(|| {
            (StatusCode::SERVICE_UNAVAILABLE, "no plan2cell binary (bin/plan2cell or FACTORY_OS_PLAN2CELL)".to_owned())
        })?;
        if !binary.is_file() {
            return Err((StatusCode::SERVICE_UNAVAILABLE, format!("the plan2cell binary {} does not exist", binary.display())));
        }
        Ok(binary)
    }

    /// Bring the export the head runs ([`head_export`]; `build`: that one) into production:
    /// `202` (see [`InfraStatus::cell`]).
    pub async fn production(self: &Arc<Self>, request: ProductionRequest) -> Result<InfraStatus, HttpError> {
        self.start_production(request, None).await
    }

    /// The production job; `auto`: auto-restart's reason (its outcome goes to [`InfraStatus::auto_restart`]).
    async fn start_production(self: &Arc<Self>, request: ProductionRequest, auto: Option<String>) -> Result<InfraStatus, HttpError> {
        self.function()?;
        let binary = self.plan2cell()?;
        let (_, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        let (build, dir, pick) = match &request.build {
            Some(id) => (id.clone(), Self::export(&root, id)?, Pick::Explicit),
            None => head_export(&root, &load_auto(&root)).map_err(|err| (StatusCode::CONFLICT, err))?,
        };
        let cell_config = self.cell_config(&root)?;
        self.claim("production", "starting production")?;
        self.ready_or_release().await?;
        let started = unix_s(SystemTime::now());
        if auto.is_some() {
            // The attempt, once the job holds the cell (its outcome follows from the job).
            let _ = self.change_auto(&root, |a| {
                a.status.last_attempt_s = Some(started);
                a.status.last_result = None;
                a.status.last_error = None;
            });
        }
        let what = if auto.is_some() { "Auto-restart: restarting the CODESYS runtimes of" } else { "Starting production of" };
        self.update(|s| {
            let keep = (s.cell.runtime_started_s, s.cell.head_state.clone());
            s.cell = factoryos_protocol::infra::CellRunStatus {
                state: JobState::Running,
                message: Some(format!("{what} {build}…")),
                build: Some(build.clone()),
                started_s: Some(started),
                runtime_started_s: keep.0,
                head_state: keep.1,
                demo_note: None,
            }
        });
        let plan_id = codegen::build_plan_id(&dir);
        self.log(format!(
            "{}: plan2cell cell start {} --cell-config {}{}{}",
            if auto.is_some() { "auto-restart" } else { "production" },
            dir.display(),
            cell_config.display(),
            plan_id.as_deref().map(|p| format!(" (PLAN_ID {p})")).unwrap_or_default(),
            match pick {
                Pick::Report => " (PLAN_ID the head runs unknown: the export last deployed, by its plan2cell-cell-report.json)",
                Pick::Newest => " (PLAN_ID the head runs unknown, no deploy report: the newest export)",
                Pick::Explicit => " (by choice)",
                Pick::Deployed => " (the PLAN_ID the head runs)",
            }
        ));
        let this = self.clone();
        tokio::spawn(async move {
            let (mut build, mut dir, mut plan_id) = (build, dir, plan_id);
            let mut result = this.run_production(&binary, &dir, &cell_config, started, true).await;
            let mut refused = result.as_ref().err().and_then(|(err, _)| guard_plan_id(&format!("{err:#}")));
            // The head runs another plan (its PLAN_ID guard refused the export): once more on the
            // export of that plan, without restarting the runtimes again (not for an export
            // chosen by hand).
            if let Some(head) = refused.clone().filter(|_| pick != Pick::Explicit) {
                match export_of_plan(&root, &head).filter(|(_, d)| *d != dir) {
                    Some((other, other_dir)) => {
                        this.log(format!(
                            "{}: the head runs PLAN_ID {head}: plan2cell cell start {} (once more, without --restart-runtime)",
                            if auto.is_some() { "auto-restart" } else { "production" },
                            other_dir.display()
                        ));
                        this.update(|s| {
                            s.cell.build = Some(other.clone());
                            s.cell.message = Some(format!("The head runs PLAN_ID {head}: starting {other} instead…"));
                        });
                        (build, dir) = (other, other_dir);
                        plan_id = codegen::build_plan_id(&dir).or(Some(head));
                        result = this.run_production(&binary, &dir, &cell_config, started, false).await;
                        refused = result.as_ref().err().and_then(|(err, _)| guard_plan_id(&format!("{err:#}")));
                    }
                    None => {
                        result = result.map_err(|(err, h)| (err.context(format!("head runs {head}; no export of it: deploy one (Run Control Code)")), h));
                    }
                }
            }
            let finished = unix_s(SystemTime::now());
            // The auto-restart file: what the head runs now, and auto-restart's outcome.
            let _ = this.change_auto(&root, |a| {
                // Whatever the outcome, the export's runtimes may have been restarted: its
                // containers are the ones read from now on, the next attempt waits 10 min, and
                // the head's restart count is taken anew at the next read.
                a.production_build = Some(build.clone());
                a.production_dir = Some(dir.display().to_string());
                a.last_job_s = Some(finished);
                a.head_restart_count = None;
                match &result {
                    Ok((_, head)) => {
                        a.in_production = true;
                        a.foreign_change = None;
                        a.production_done_s = Some(finished);
                        a.status.blocked = None;
                        a.status.failures = 0;
                        if plan_id.is_some() {
                            a.status.deployed_plan_id = plan_id.clone();
                        }
                        if auto.is_some() {
                            a.status.last_result = Some(head.clone().unwrap_or_else(|| "in production".to_owned()));
                            a.status.last_error = None;
                        }
                    }
                    Err((err, _)) => {
                        if let Some(head_plan) = &refused {
                            a.status.deployed_plan_id = Some(head_plan.clone());
                        }
                        if auto.is_some() {
                            let err = format!("{err:#}");
                            a.status.failures += 1;
                            if refused.is_some() {
                                a.status.blocked = Some(format!("the head refused the export (PLAN_ID guard): {err}"));
                            } else if a.status.failures >= AUTO_RESTART_MAX_FAILURES {
                                a.status.blocked = Some(format!("{} attempts in a row failed", a.status.failures));
                            }
                            a.status.last_result = None;
                            a.status.last_error = Some(err);
                        }
                    }
                }
            });
            let mut inner = this.inner.lock().unwrap();
            inner.job = None;
            // Read the runtimes again at the next tick (restarted: a new demo period).
            inner.runtime_checked = None;
            let c = &mut inner.status.cell;
            match result {
                Ok((message, head)) => {
                    c.state = JobState::Done;
                    c.message = Some(message);
                    c.head_state = head.or(c.head_state.take());
                }
                Err((err, head)) => {
                    c.state = JobState::Failed;
                    c.message = Some(format!("{err:#}"));
                    if head.is_some() {
                        c.head_state = head;
                    }
                }
            }
            let line = match &auto {
                Some(_) => format!("[infra] auto-restart: {}", inner.status.auto_restart.message.clone().unwrap_or_default()),
                None => format!("[infra] production: {}", c.message.clone().unwrap_or_default()),
            };
            inner.status.cell.demo_note = Some(demo_note(&inner.status, this.settings.semantic_hosts));
            this.sink.status(&inner.status);
            this.sink.log(line);
        });
        Ok(self.status())
    }

    /// The folder of export `id` of the cell at `root` (`404`: no successful export of that id).
    fn export(root: &Path, id: &str) -> Result<PathBuf, HttpError> {
        codegen::export_dir(root, id)
            .ok_or_else(|| (StatusCode::NOT_FOUND, format!("no successful Control Code Export {id} in the active cell")))
    }

    /// The cell.toml for the cell at `root` (`409` without a cell config on this server).
    fn cell_config(&self, root: &Path) -> Result<PathBuf, HttpError> {
        self.hardware
            .cell_config_for(root)
            .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("rendering the cell.toml: {err:#}")))?
            .ok_or_else(|| {
                (
                    StatusCode::CONFLICT,
                    "this server has no plan2cell cell config (/etc/plan2cell/cell.toml): it is not connected to a virtual cell".to_owned(),
                )
            })
    }

    // --- the CODESYS demo runtimes and auto-restart ---

    /// The active cell's auto-restart (read from its file).
    pub fn auto_restart(&self) -> AutoRestartStatus {
        match cells::active_cell(&self.config) {
            Some((_, root)) => {
                let a = load_auto(&root);
                self.publish_auto(&root, &a);
                self.status().auto_restart
            }
            None => AutoRestartStatus::default(),
        }
    }

    /// `PUT`: switch it on / off (on again: a hold of the PLAN_ID guard is lifted).
    pub fn set_auto_restart(&self, request: AutoRestartSettings) -> Result<AutoRestartStatus, HttpError> {
        let (_, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        if request.enabled && self.settings.role == CellRole::Operator {
            return Err((StatusCode::CONFLICT, format!("{} (this backend is an operator of the cell: FACTORY_OS_CELL_ROLE)", self.supervisor_note())));
        }
        let threshold = request.threshold_s.filter(|t| *t > 0);
        if threshold.is_some_and(|t| t > 3600) {
            return Err((StatusCode::BAD_REQUEST, "thresholdS: at most 3600 (the demo period is 2 h)".to_owned()));
        }
        self.change_auto(&root, |a| {
            if request.enabled {
                a.status.blocked = None;
                a.status.failures = 0;
            }
            a.status.enabled = request.enabled;
            if let Some(t) = threshold {
                a.status.threshold_s = t;
            }
        })
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}")))?;
        let status = self.status().auto_restart;
        self.log(format!(
            "auto-restart {} (restart {} s before the CODESYS demo stop)",
            if status.enabled { "on" } else { "off" },
            status.threshold_s
        ));
        // Read the runtimes soon.
        self.inner.lock().unwrap().runtime_checked = None;
        Ok(status)
    }

    /// `Auto-restart runs on <supervisor>`.
    fn supervisor_note(&self) -> String {
        format!("Auto-restart runs on {}", self.settings.supervisor.as_deref().unwrap_or("the cell's supervisor backend"))
    }

    /// Load, change and save the auto-restart file of the cell at `root`; the status follows when
    /// it is the active cell's.
    fn change_auto(&self, root: &Path, change: impl FnOnce(&mut AutoFile)) -> anyhow::Result<()> {
        let saved = {
            let _one = self.auto_file.lock().unwrap();
            let mut a = load_auto(root);
            change(&mut a);
            save_auto(root, &a).map(|()| a)
        };
        match saved {
            Ok(a) => {
                if cells::active_cell(&self.config).is_some_and(|(_, r)| r == root) {
                    self.publish_auto(root, &a);
                }
                Ok(())
            }
            Err(err) => {
                self.log(format!("auto-restart: {err:#}"));
                Err(err)
            }
        }
    }

    /// `a` (of the active cell at `root`) and the exports production and Run Control Code would
    /// take into the status (published when it changed).
    fn publish_auto(&self, root: &Path, a: &AutoFile) {
        // A job that finishes after the active cell changed publishes nothing of its cell.
        if cells::active_cell(&self.config).is_none_or(|(_, r)| r != root) {
            return;
        }
        let mut status = a.status.clone();
        status.next_allowed_s = a.last_try_s().map(|t| t + AUTO_RESTART_BACKOFF_S);
        let (job, inactive) = {
            let inner = self.inner.lock().unwrap();
            // The runtimes cannot be read while the PLC host runs: auto-restart cannot act.
            let host_up = plc_host(&inner.status, self.settings.semantic_hosts).is_some_and(|h| h.running() && h.online());
            let error = inner.status.plc_runtime.as_ref().and_then(|r| r.error.clone()).filter(|_| host_up);
            (inner.job, error.filter(|_| status.enabled && status.blocked.is_none()))
        };
        status.inactive = inactive.map(|e| format!("cannot read the CODESYS runtimes: {e}"));
        status.message = match job {
            Some("deploy") if status.enabled && status.blocked.is_none() => {
                Some("auto-restart paused while Run Control Code deploys".to_owned())
            }
            _ => auto_message(&status, job.is_some()),
        };
        if let Some(note) = a.foreign_change.as_ref().filter(|_| !a.in_production && status.enabled && job.is_none()) {
            status.message = Some(note.clone());
        }
        let power = power_holds(self.inner.lock().unwrap().status.cell_power.as_ref()).map(power_word);
        if let Some(word) = power.filter(|_| status.enabled && status.blocked.is_none()) {
            status.message = Some(format!("auto-restart paused while the virtual cell is {word}"));
            status.inactive = None;
        }
        if self.settings.role == CellRole::Operator {
            status.message = Some(self.supervisor_note());
            status.inactive = None;
        }
        if let Some(why) = self.lacks("runtime.codesys", "auto-restart").filter(|_| status.enabled) {
            status.inactive = Some(why);
        }
        let exports = Exports::of(root, a, &windows_project_dir(self.hardware.server_cell_config_path()));
        let (changed, switched) = {
            let mut inner = self.inner.lock().unwrap();
            // The last Deploy / production job is another cell's (the active cell changed since):
            // its outcome is not this cell's. Not while a job runs (it is the cell it started on).
            let switched = inner.job.is_none() && inner.jobs_root.as_deref().is_some_and(|r| r != root);
            if inner.job.is_none() {
                inner.jobs_root = Some(root.to_owned());
            }
            (inner.status.auto_restart != status || Exports::from_status(&inner.status) != exports, switched)
        };
        if changed || switched {
            self.update(|s| {
                s.auto_restart = status;
                exports.into_status(s);
                if switched {
                    s.deploy = DeployStatus::default();
                    let cell = std::mem::take(&mut s.cell);
                    // The head PLC and its runtime are the same hardware: what was read of them stays.
                    s.cell = factoryos_protocol::infra::CellRunStatus {
                        runtime_started_s: cell.runtime_started_s,
                        head_state: cell.head_state,
                        ..Default::default()
                    };
                    s.cell.demo_note = Some(demo_note(s, self.settings.semantic_hosts));
                }
            });
        }
    }

    /// Every poll: the active cell's auto-restart file, then (about every
    /// [`Settings::runtime_poll`], while a viewer asks or auto-restart is on) the runtimes and
    /// auto-restart's decision.
    pub async fn runtime_tick(self: &Arc<Self>) {
        if self.settings.function.is_none() {
            return;
        }
        self.touch_if_due().await;
        self.follow_power().await;
        let Some((_, root)) = cells::active_cell(&self.config) else { return };
        let supervisor = self.settings.role == CellRole::Supervisor;
        let mut auto = load_auto(&root);
        // Found off (another backend's Stop virtual cell, the Lambda's idle stop): out of production, as
        // a Stop cell of this backend would leave it, so auto-restart does not fight it.
        let power = self.status().cell_power;
        if auto.in_production
            && let Some(p) = power.as_ref().filter(|p| matches!(p.state, PowerState::Off | PowerState::Stopping))
        {
            let by = p.session.as_ref().and_then(|s| s.by.clone()).filter(|b| b != &self.settings.holder);
            self.log(format!(
                "the virtual cell is {}{}: out of production (the next Start virtual cell resumes it)",
                power_word(p),
                by.map(|b| format!(" (stopped by {b})")).unwrap_or_default()
            ));
            let _ = self.change_auto(&root, |a| {
                a.stopped_in_production = true;
                a.in_production = false;
            });
            auto = load_auto(&root);
        }
        // The cell's lease: for a viewer's "in use by", and the supervisor's auto-restart.
        let interest = self.inner.lock().unwrap().interest.is_some_and(|at| at.elapsed() < INTEREST);
        let lease = if interest || (supervisor && auto.status.enabled) { self.lease_status().await } else { self.status().lease };
        if supervisor
            && auto.in_production
            && let Some(change) = lease.as_ref().and_then(|l| l.last_change.as_ref())
            && change.holder != self.settings.holder
            && auto.production_done_s.is_none_or(|own| change.at_s > own)
        {
            let note = format!("production changed by {} at {} ({}); auto-restart idle", change.holder, utc_hhmm(change.at_s), change.action);
            self.log(&note);
            let _ = self.change_auto(&root, |a| {
                a.in_production = false;
                a.stopped_in_production = false;
                a.foreign_change = Some(note.clone());
            });
            auto = load_auto(&root);
        }
        self.publish_auto(&root, &auto);
        let due = {
            let inner = self.inner.lock().unwrap();
            let wanted = auto.status.enabled || inner.interest.is_some_and(|at| at.elapsed() < INTEREST);
            wanted && inner.runtime_checked.is_none_or(|at| at.elapsed() >= self.settings.runtime_poll)
        };
        // A template without CODESYS demo runtimes: nothing to read or restart (publish_auto says so).
        if !due || no_codesys_runtime(&self.status()) {
            return;
        }
        let runtime = self.read_runtime(&root, &auto).await;
        // A read error shows in auto-restart's line (inactive).
        let auto = load_auto(&root);
        self.publish_auto(&root, &auto);
        let Some(runtime) = runtime else { return };
        let (job, ready) = {
            let inner = self.inner.lock().unwrap();
            (inner.job.is_some(), inner.status.ready)
        };
        let now = unix_s(SystemTime::now());
        // Only the supervisor restarts, and not while another backend holds the cell.
        let foreign = lease.as_ref().is_some_and(|l| l.held_by_other(now));
        // Never while the cell is off, stopping or booting (the Lambda's power-on brings it up).
        let powered = power_holds(self.status().cell_power.as_ref()).is_none();
        let head = self.head.lock().unwrap().clone();
        let reason = auto_reason(&auto, &runtime, head.as_ref(), now).filter(|_| !job && ready && supervisor && !foreign && powered);
        // The head's restart count: taken as it is now, unless a restart of its own waits for an
        // attempt (on, not held, in production; during the backoff or another job).
        let pending = auto.status.enabled && auto.status.blocked.is_none() && auto.in_production && own_restart(&auto, &runtime) && supervisor;
        if reason.is_none() && !pending && runtime.restart_count.is_some() && auto.head_restart_count != runtime.restart_count {
            let _ = self.change_auto(&root, |a| a.head_restart_count = runtime.restart_count);
        }
        let Some(reason) = reason else { return };
        self.log(format!("auto-restart: {reason}: Restart PLC runtime (plan2cell cell start --restart-runtime)"));
        let request = ProductionRequest { match_deployed: true, build: None };
        let started = self.leased("auto-restart", true, self.start_production(request, Some(reason))).await;
        if let Err((code, err)) = started {
            // Another backend took the cell meanwhile: not a failure of auto-restart.
            if code == StatusCode::CONFLICT && err.starts_with("cell busy") {
                self.log(format!("auto-restart: {err}"));
                return;
            }
            // Refused before the job started (no export of the head's PLAN_ID, not ready, ...).
            let refused = code == StatusCode::CONFLICT && err.contains("PLAN_ID");
            self.log(format!("auto-restart: {err}"));
            let _ = self.change_auto(&root, |a| {
                a.status.last_attempt_s = Some(now);
                a.status.last_result = None;
                a.status.failures += 1;
                if refused {
                    a.status.blocked = Some(err.clone());
                } else if a.status.failures >= AUTO_RESTART_MAX_FAILURES {
                    a.status.blocked = Some(format!("{} attempts in a row failed", a.status.failures));
                }
                a.status.last_error = Some(err);
            });
        }
    }

    /// The export whose runtime containers count: the last production start's, else the newest
    /// of the deployed PLAN_ID, else (that unknown) the export last deployed, else the newest
    /// (`None`: no export with a `plc/project.json`).
    fn runtime_export(root: &Path, a: &AutoFile) -> Option<(String, PathBuf)> {
        let usable = |(id, dir): (String, PathBuf)| dir.join("plc/project.json").is_file().then_some((id, dir));
        a.production_dir
            .as_ref()
            .map(|d| (a.production_build.clone().unwrap_or_default(), PathBuf::from(d)))
            .and_then(usable)
            .or_else(|| a.status.deployed_plan_id.as_deref().and_then(|p| codegen::newest_build_of_plan(root, p)).and_then(usable))
            .or_else(|| codegen::last_deployed_build(root).filter(|_| a.status.deployed_plan_id.is_none()).and_then(usable))
            .or_else(|| codegen::newest_build(root).and_then(usable))
    }

    /// `plan2cell cell runtime --json` into [`InfraStatus::plc_runtime`]; `None` if it was not
    /// read (no PLC host running, no plan2cell, an error: the status says why).
    async fn read_runtime(&self, root: &Path, auto: &AutoFile) -> Option<PlcRuntime> {
        let _one = self.runtime_reading.lock().await;
        let stale = self.inner.lock().unwrap().refreshed.is_none_or(|at| at.elapsed() >= self.settings.runtime_poll);
        if stale {
            self.refresh().await;
        }
        self.inner.lock().unwrap().runtime_checked = Some(Instant::now());
        let now = unix_s(SystemTime::now());
        let fail = |this: &Self, error: String| {
            this.update(|s| {
                let mut rt = s.plc_runtime.take().unwrap_or_default();
                rt.error = Some(error);
                rt.checked_s = now;
                s.plc_runtime = Some(rt);
            });
            None
        };
        let host = plc_host(&self.status(), self.settings.semantic_hosts).cloned();
        match &host {
            Some(h) if h.running() && h.online() => {}
            Some(h) => {
                // A stopped cell: nothing runs.
                self.update(|s| {
                    s.plc_runtime = Some(PlcRuntime {
                        checked_s: now,
                        error: Some(format!("the PLC host is {} (SSM {}): CODESYS is not running", h.state, h.ssm_ping)),
                        ..Default::default()
                    })
                });
                return None;
            }
            None => return fail(self, "no PLC host in the cell's status".to_owned()),
        }
        let binary = match self.plan2cell() {
            Ok(b) => b,
            Err((_, err)) => return fail(self, err),
        };
        let cell_config = match self.cell_config(root) {
            Ok(c) => c,
            Err((_, err)) => return fail(self, err),
        };
        let export = Self::runtime_export(root, auto);
        let mut args: Vec<String> = vec!["cell".into(), "runtime".into()];
        if let Some((_, dir)) = &export {
            args.push(dir.display().to_string());
        }
        args.extend(["--cell-config".into(), cell_config.display().to_string(), "--json".into()]);
        let mut env = Vec::new();
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        let out = match run(&binary, &args, &env, self.settings.call_timeout, None).await {
            Ok(out) => out,
            Err(err) => return fail(self, format!("plan2cell cell runtime: {err:#}")),
        };
        let report: Value = serde_json::from_slice(&out.stdout).unwrap_or(Value::Null);
        if out.code != Some(0) || report["ok"] != true {
            let why = report["error"]
                .as_str()
                .map(str::to_owned)
                .or_else(|| out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).map(str::to_owned))
                .unwrap_or_else(|| "no report".to_owned());
            let code = out.code.map_or("killed".to_owned(), |c| c.to_string());
            return fail(self, format!("plan2cell cell runtime (exit {code}): {why}"));
        }
        let runtime = PlcRuntime { checked_s: now, build: export.map(|(id, _)| id).filter(|id| !id.is_empty()), ..plc_runtime(&report) };
        self.update(|s| s.plc_runtime = Some(runtime.clone()));
        Some(runtime)
    }

    /// `plan2cell cell start`; `Ok((message, head state))`, or the error and the head state read.
    async fn run_production(
        &self,
        binary: &Path,
        dir: &Path,
        cell_config: &Path,
        started: f64,
        restart_runtime: bool,
    ) -> Result<(String, Option<String>), (anyhow::Error, Option<String>)> {
        if self.needs_mcp_token() {
            self.update(|s| s.cell.message = Some("Reading the CODESYS MCP token (Secrets Manager)…".to_owned()));
        }
        let token = self.mcp_token().await.map_err(|err| (err, None))?;
        self.run_start(binary, dir, cell_config, started, restart_runtime, token).await
    }

    /// `plan2cell cell start` with the MCP token `token`.
    async fn run_start(
        &self,
        binary: &Path,
        dir: &Path,
        cell_config: &Path,
        started: f64,
        restart_runtime: bool,
        token: String,
    ) -> Result<(String, Option<String>), (anyhow::Error, Option<String>)> {
        self.update(|s| {
            s.cell.message = Some(if restart_runtime {
                "plan2cell cell start: restarting the CODESYS runtimes, KUKA VCs into EXT, reset pulses…".to_owned()
            } else {
                "plan2cell cell start: KUKA VCs into EXT, reset pulses…".to_owned()
            })
        });
        let mut args: Vec<String> = vec!["cell".into(), "start".into(), dir.display().to_string()];
        args.extend(["--cell-config".into(), cell_config.display().to_string()]);
        if restart_runtime {
            args.push("--restart-runtime".into());
        }
        args.extend(["--pulses".into(), "16".into(), "--json".into()]);
        let mut env: Vec<(String, String)> = Some(token).filter(|t| !t.is_empty()).map(|t| (MCP_TOKEN_ENV.to_owned(), t)).into_iter().collect();
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        let out = run(binary, &args, &env, self.settings.production_timeout, Some(&*self.sink)).await.map_err(|err| (err, None))?;
        let report: Value = serde_json::from_slice(&out.stdout).unwrap_or(Value::Null);
        // The report is the build's plan2cell-cell-report.json, merged over runs: the stages of
        // earlier runs (deploy, an earlier stop's `CS_ERROR ERR_OPERATOR_STOP`, after this start
        // in stage order) are in it too. Only this run's start stage counts.
        let start = report["stages"].as_array().into_iter().flatten().find(|s| s["stage"] == "start");
        let notes: Vec<String> =
            start.and_then(|s| s["notes"].as_array()).into_iter().flatten().filter_map(|n| n.as_str().map(str::to_owned)).collect();
        let head = notes.iter().rev().find_map(|n| cell_state_in(n)).or_else(|| {
            report["projects"].as_array().into_iter().flatten().find_map(|p| p["state"].as_str().map(str::to_owned))
        });
        let restarted = notes.iter().any(|n| n.starts_with("restarted "));
        if restarted {
            self.update(|s| s.cell.runtime_started_s = Some(started));
        }
        let ok = match start {
            Some(s) => s["ok"] != false,
            None => report["ok"] != false,
        };
        if out.code == Some(0) && ok {
            let last = notes.iter().rev().find(|n| n.contains("in production")).cloned();
            Ok((last.unwrap_or_else(|| "in production".to_owned()), head))
        } else {
            let error = start
                .and_then(|s| s["error"].as_str().map(str::to_owned))
                .or_else(|| out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).map(str::to_owned))
                .unwrap_or_else(|| "no report".to_owned());
            let code = out.code.map_or("killed".to_owned(), |c| c.to_string());
            Err((anyhow::anyhow!("plan2cell cell start failed (exit {code}): {error}"), head))
        }
    }

    /// Stop production: `plan2cell cell stop` on the export the head runs (`PLC_PRG.bStop`; the
    /// head ends in `CS_ERROR ERR_OPERATOR_STOP`): `202`. The cell leaves production (auto-restart
    /// stays out); the instances and runtimes keep running, Start production starts it again.
    pub async fn stop_production(self: &Arc<Self>) -> Result<InfraStatus, HttpError> {
        self.function()?;
        let binary = self.plan2cell()?;
        let (_, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        let (build, dir, _) = head_export(&root, &load_auto(&root)).map_err(|err| (StatusCode::CONFLICT, err))?;
        let cell_config = self.cell_config(&root)?;
        self.claim("production-stop", "Stop production")?;
        self.ready_or_release().await?;
        let started = unix_s(SystemTime::now());
        self.update(|s| {
            s.cell.state = JobState::Running;
            s.cell.message = Some(format!("Stopping production of {build} (PLC_PRG.bStop)…"));
            s.cell.build = Some(build.clone());
            s.cell.started_s = Some(started);
        });
        self.log(format!("Stop production: plan2cell cell stop {} --cell-config {}", dir.display(), cell_config.display()));
        let this = self.clone();
        tokio::spawn(async move {
            let result = match this.secret(&this.mcp_secret()).await {
                Ok(token) => this.run_stage(&binary, "stop", &dir, &cell_config, &token, this.settings.stage_timeout).await,
                Err(err) => Err(err),
            };
            // Whatever the outcome: no auto-restart into production behind the user's back.
            let _ = this.change_auto(&root, |a| a.in_production = false);
            {
                let mut inner = this.inner.lock().unwrap();
                inner.job = None;
                let cell = &mut inner.status.cell;
                match &result {
                    Ok(notes) => {
                        // `PLC_PRG.bStop pulsed`, then the state read: `CS_ERROR ERR_OPERATOR_STOP robot 0, udiCycles 7`.
                        let state = notes.last().filter(|n| n.starts_with("CS_")).cloned();
                        cell.state = JobState::Done;
                        cell.head_state = state.as_deref().and_then(|s| s.split([' ', ',']).next()).map(str::to_owned);
                        cell.message = Some(format!("production stopped: {}", state.unwrap_or_else(|| "PLC_PRG.bStop pulsed".to_owned())));
                    }
                    Err(err) => {
                        cell.state = JobState::Failed;
                        cell.message = Some(format!("Stop production: {err:#}"));
                    }
                }
                let line = format!("[infra] Stop production: {}", cell.message.clone().unwrap_or_default());
                inner.status.cell.demo_note = Some(demo_note(&inner.status, this.settings.semantic_hosts));
                this.sink.status(&inner.status);
                this.sink.log(line);
            }
            this.publish_auto(&root, &load_auto(&root));
        });
        Ok(self.status())
    }

    // --- Run Control Code ---

    /// Run Control Code: deploy an export (`build`, else the newest) to the cell and start it:
    /// `202` (see [`InfraStatus::deploy`]).
    pub async fn deploy(self: &Arc<Self>, request: DeployRequest) -> Result<InfraStatus, HttpError> {
        self.function()?;
        // A cell that is off / stopping / booting has no IDE or PLC to deploy to (the cell config still
        // names the last launch's hosts).
        if let Some(why) = self.status().power_blocks() {
            return Err((StatusCode::CONFLICT, why));
        }
        let binary = self.plan2cell()?;
        let (_, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        let (build, dir) = match &request.build {
            Some(id) => (id.clone(), Self::export(&root, id)?),
            None => codegen::newest_build(&root).ok_or_else(|| {
                (StatusCode::CONFLICT, "no successful Control Code Export of this cell yet: Export PLC + robot code first".to_owned())
            })?,
        };
        if !dir.join("plc/project.json").is_file() {
            return Err((StatusCode::CONFLICT, format!("the export {build} has no CODESYS projects (plc/project.json): export it with PLC code")));
        }
        let cell_config = self.cell_config(&root)?;
        // Each `plan2cell cell` command runs one stage (none runs another); `krl` alone exits 3
        // without [kuka_vc].
        let kuka = has_kuka_vc(&cell_config);
        // The cell's template's stages (`consumer.deploy.stages`: a hook head has no `projects`), else all.
        let offered = self.template().map(|t| t.consumer.deploy.stages).filter(|s| !s.is_empty());
        let offers = |stage: &str| offered.as_ref().is_none_or(|o| o.iter().any(|s| s == stage));
        let stages: Vec<&'static str> = match &request.stages {
            Some(asked) => {
                if let Some(bad) = asked.iter().find(|s| !DEPLOY_STAGES.contains(&s.as_str())) {
                    return Err((StatusCode::BAD_REQUEST, format!("unknown stage {bad:?} (stages: {})", DEPLOY_STAGES.join(", "))));
                }
                if let (Some(bad), Some(o)) = (asked.iter().find(|s| !offers(s)), &offered) {
                    return Err((StatusCode::BAD_REQUEST, format!("stage {bad:?}: not a stage of this virtual cell's template (stages: {})", o.join(", "))));
                }
                // In their order; `krl` only with KUKA VCs.
                let stages: Vec<&'static str> =
                    DEPLOY_STAGES.into_iter().filter(|s| asked.iter().any(|a| a == s) && (*s != "krl" || kuka)).collect();
                if stages.is_empty() {
                    return Err((StatusCode::BAD_REQUEST, "no stage to run (krl needs KUKA VCs in the cell.toml)".to_owned()));
                }
                stages
            }
            None => {
                let mut stages = vec!["projects", "deploy"];
                if kuka {
                    stages.push("krl");
                }
                if request.start {
                    stages.push("start");
                }
                stages.retain(|s| offers(s));
                if stages.is_empty() {
                    return Err((StatusCode::CONFLICT, "this virtual cell's template has no deploy stage to run".to_owned()));
                }
                stages
            }
        };
        let starts = stages.contains(&"start");
        let what = if starts { "Run Control Code" } else { "Deploy" };
        // The deployability check first (seconds, read-only): a failing rule is `409` with the
        // report, unless the user deploys anyway.
        if !request.force {
            self.gate(&build, &root, &dir).await?;
        }
        self.claim("deploy", what)?;
        self.ready_or_release().await?;
        let plan_id = codegen::build_plan_id(&dir);
        let started = unix_s(SystemTime::now());
        let plan = plan_id.as_deref().map(|p| format!(" (PLAN_ID {p})")).unwrap_or_default();
        // Not inside `update` (it holds the state the template is read from).
        let first = if self.needs_mcp_token() { ": reading the CODESYS MCP token (Secrets Manager)…" } else { "…" };
        self.update(|s| {
            s.deploy = DeployStatus {
                state: JobState::Running,
                stage: None,
                stages: stages.iter().map(|s| (*s).to_owned()).collect(),
                done: Vec::new(),
                message: Some(format!("{what}: {build}{plan}{first}")),
                build: Some(build.clone()),
                plan_id: plan_id.clone(),
                started_s: Some(started),
                ..Default::default()
            }
        });
        // Auto-restart is paused while it runs (one job at a time): its line says so.
        self.publish_auto(&root, &load_auto(&root));
        self.log(format!(
            "{what}: {build}{plan}: plan2cell cell {} {} --cell-config {}",
            stages.join(", "),
            dir.display(),
            cell_config.display()
        ));
        let this = self.clone();
        tokio::spawn(async move {
            let result = this.run_deploy(&binary, &root, &build, &dir, &cell_config, &stages, plan_id.as_deref(), started).await;
            this.finish_deploy(&root, &build, &dir, plan_id, result).await;
        });
        Ok(self.status())
    }

    /// The stages of Run Control Code, in order; the first failure stops it.
    #[allow(clippy::too_many_arguments)]
    /// An own cell's live bridge shows the export being deployed: its cell, zones and robots'
    /// operations ([`cell_wiring::WireFiles::apply_export`]; the template's are another cell's).
    fn follow_export_live(&self, root: &Path, dir: &Path) {
        let (Some(files), Some(cell)) = (&self.settings.wire, root.file_name().and_then(|n| n.to_str())) else { return };
        let cell_plan = dir.join("plc/cell_plan.json");
        if !cell_plan.is_file() {
            return;
        }
        let export = cell_wiring::BridgeExport { cell: cell.to_owned(), cell_plan };
        match files.apply_export(&export) {
            Ok(true) => self.log(format!("the live view follows {cell}'s export (fos-live-bridge restarted)")),
            Ok(false) => {}
            Err(err) => tracing::warn!("Virtual Cell: the live bridge does not follow the export: {err:#}"),
        }
    }

    async fn run_deploy(
        &self,
        binary: &Path,
        root: &Path,
        build: &str,
        dir: &Path,
        cell_config: &Path,
        stages: &[&'static str],
        plan_id: Option<&str>,
        started: f64,
    ) -> Result<(String, Option<String>), DeployError> {
        let token = self.mcp_token().await.map_err(|error| DeployError { stage: None, error, head: None })?;
        self.follow_export_live(root, dir);
        for &stage in stages {
            self.update(|s| {
                s.deploy.stage = Some(stage.to_owned());
                s.deploy.message = Some(format!("{stage}: {}…", deploy_stage_label(stage)));
            });
            self.log(format!("Run Control Code: plan2cell cell {stage} {}", dir.display()));
            if stage == "start" {
                self.update(|s| {
                    let keep = (s.cell.runtime_started_s, s.cell.head_state.clone());
                    s.cell = factoryos_protocol::infra::CellRunStatus {
                        state: JobState::Running,
                        message: Some(format!("Starting production of {build} (Run Control Code)…")),
                        build: Some(build.to_owned()),
                        started_s: Some(started),
                        runtime_started_s: keep.0,
                        head_state: keep.1,
                        demo_note: None,
                    }
                });
                // A fresh boot's runtimes started minutes ago (a new demo period): no forced
                // restart. `start` still restarts a runtime that does not answer.
                let restart = !fresh_boot(&self.status(), self.settings.semantic_hosts, unix_s(SystemTime::now()));
                if !restart {
                    self.log(format!("Run Control Code: the PLC host booted under {} min ago: start without --restart-runtime", FRESH_BOOT_S / 60.0));
                }
                let (message, head) = self
                    .run_start(binary, dir, cell_config, started, restart, token.clone())
                    .await
                    .map_err(|(error, head)| DeployError { stage: Some(stage), error, head })?;
                self.update(|s| s.deploy.done.push(stage.to_owned()));
                return Ok((message, head));
            }
            let timeout = if stage == "projects" { self.settings.projects_timeout } else { self.settings.stage_timeout };
            let notes = self
                .run_stage(binary, stage, dir, cell_config, &token, timeout)
                .await
                .map_err(|error| DeployError { stage: Some(stage), error, head: None })?;
            self.update(|s| {
                s.deploy.done.push(stage.to_owned());
                if let Some(note) = notes.last() {
                    s.deploy.message = Some(format!("{stage}: {note}"));
                }
            });
            if stage == "deploy" {
                let (opened, note) = ide_outcome(&notes);
                self.update(|s| {
                    s.deploy.ide_opened = opened;
                    s.deploy.ide_note = note;
                });
                // The head PLC runs this export's code from now on (Restart PLC runtime and
                // auto-restart follow it), whatever the later stages do. The download restarted
                // the head's application: it waits for the operator pulses (CS_INIT /
                // CS_RECOVERY), so the cell is not in production until a start succeeds.
                let _ = self.change_auto(root, |a| {
                    a.in_production = false;
                    a.deploy_unknown = false;
                    a.deployed_build = Some(build.to_owned());
                    a.status.deployed_plan_id = plan_id.map(str::to_owned);
                    a.production_build = Some(build.to_owned());
                    a.production_dir = Some(dir.display().to_string());
                    a.head_restart_count = None;
                });
            }
        }
        if !self.needs_mcp_token() {
            // A hook head (`deploy.head.hook`): no PLC IDE.
            return Ok((format!("{DEPLOYED_NOT_STARTED}; the head was deployed through its hook (no PLC IDE)"), None));
        }
        let ide = match (self.status().deploy.ide_opened, self.status().deploy.ide_note) {
            (Some(true), _) => "the head project is open in the PLC IDE".to_owned(),
            (Some(false), Some(note)) => format!("{note}: open it in CODESYS by hand (File > Open)"),
            _ => "this plan2cell did not report opening the head project in the PLC IDE: open it in CODESYS by hand (File > Open)".to_owned(),
        };
        Ok((format!("{DEPLOYED_NOT_STARTED}; {ide}"), None))
    }

    /// The outcome of Run Control Code into the status and the auto-restart file; frees the cell.
    /// A failed restore it supersedes is reported to the Lambda as done (`restore_report ok:true`).
    async fn finish_deploy(
        &self,
        root: &Path,
        build: &str,
        dir: &Path,
        plan_id: Option<String>,
        result: Result<(String, Option<String>), DeployError>,
    ) {
        let finished = unix_s(SystemTime::now());
        let starts = self.status().deploy.starts();
        let refused = result.as_ref().err().filter(|e| e.stage == Some("start")).and_then(|e| guard_plan_id(&format!("{:#}", e.error)));
        let _ = self.change_auto(root, |a| {
            // The runtimes were restarted or changed: auto-restart waits 10 min, and the head's
            // restart count is taken anew.
            a.last_job_s = Some(finished);
            a.head_restart_count = None;
            match &result {
                Ok(_) if starts => {
                    a.in_production = true;
                    a.foreign_change = None;
                    a.production_done_s = Some(finished);
                    a.production_build = Some(build.to_owned());
                    a.production_dir = Some(dir.display().to_string());
                    a.deployed_build = Some(build.to_owned());
                    a.status.deployed_plan_id = plan_id.clone();
                    a.status.blocked = None;
                    a.status.failures = 0;
                }
                // Deploy only: the head runs the new code but waits for Start production.
                Ok(_) => {
                    a.in_production = false;
                    a.production_build = Some(build.to_owned());
                    a.production_dir = Some(dir.display().to_string());
                    a.deployed_build = Some(build.to_owned());
                    a.status.deployed_plan_id = plan_id.clone();
                }
                // Partly downloaded: what the head runs is not known (the deploy reports and the
                // PLAN_ID guard tell).
                Err(e) if e.stage == Some("deploy") => {
                    a.in_production = false;
                    a.deploy_unknown = true;
                    a.deployed_build = None;
                    a.status.deployed_plan_id = None;
                }
                Err(_) => {
                    if let Some(head) = &refused {
                        a.status.deployed_plan_id = Some(head.clone());
                    }
                }
            }
        });
        let plan = plan_id.as_deref().map(|p| format!(" (PLAN_ID {p})")).unwrap_or_default();
        let mut superseded = None;
        {
            let mut inner = self.inner.lock().unwrap();
            inner.job = None;
            inner.runtime_checked = None;
            let runtime_down = inner.status.plc_runtime.as_ref().is_some_and(|r| r.error.is_none() && !r.running);
            // A restore that failed is over once Run Control Code put an export on the cell: the
            // export it deployed is what the cell runs (and the Lambda's lastBuild from now on).
            let launch = inner.own.restored_launch;
            if result.is_ok()
                && let Some(restore) = inner.own.restore.as_mut().filter(|r| r.state == JobState::Failed)
            {
                restore.state = JobState::Done;
                restore.message = Some(format!("superseded: Run Control Code deployed {build}"));
                restore.build = Some(build.to_owned());
                restore.plan_id = plan_id.clone();
                let restore = restore.clone();
                superseded = launch.map(|l| (l, restore.clone()));
                if let Some(cell) = &mut inner.status.virtual_cell {
                    cell.restore = Some(restore);
                }
            }
            let status = &mut inner.status;
            status.deploy.finished_s = Some(finished);
            match result {
                Ok((message, head)) if starts => {
                    status.deploy.state = JobState::Done;
                    status.deploy.stage = None;
                    status.deploy.message = Some(format!("{build}{plan} deployed and in production: {message}"));
                    status.cell.state = JobState::Done;
                    status.cell.message = Some(message);
                    status.cell.head_state = head.or(status.cell.head_state.take());
                }
                Ok((message, _)) => {
                    status.deploy.state = JobState::Done;
                    status.deploy.stage = None;
                    status.deploy.message = Some(format!("{build}{plan} {message}"));
                    // The last production start's outcome no longer holds: the head's
                    // application restarted with the download and waits for its pulses.
                    status.cell.state = JobState::Idle;
                    status.cell.build = Some(build.to_owned());
                    status.cell.message = Some(format!("{build} deployed, not in production yet: Start production"));
                    status.cell.head_state = None;
                }
                Err(DeployError { stage, error, head }) => {
                    status.deploy.state = JobState::Failed;
                    status.deploy.stage = stage.map(str::to_owned);
                    let hint = match stage {
                        Some("deploy") if runtime_down => {
                            " (the head's CODESYS runtime is not running: Restart PLC runtime, then Run Control Code again)"
                        }
                        Some("projects") => " (nothing was changed on the PLCs)",
                        _ => "",
                    };
                    status.deploy.message = Some(match stage {
                        Some(stage) => format!("{stage} failed: {error:#}{hint}"),
                        None => format!("{error:#}"),
                    });
                    if stage == Some("start") {
                        status.cell.state = JobState::Failed;
                        status.cell.message = Some(format!("{error:#}"));
                        if head.is_some() {
                            status.cell.head_state = head;
                        }
                    }
                }
            }
            let what = if starts { "Run Control Code" } else { "Deploy" };
            let line = format!("[infra] {what}: {}", status.deploy.message.clone().unwrap_or_default());
            status.cell.demo_note = Some(demo_note(status, self.settings.semantic_hosts));
            self.sink.status(status);
            self.sink.log(line);
        }
        // Auto-restart's line without the pause.
        self.publish_auto(root, &load_auto(root));
        if let Some((launch, restore)) = superseded {
            self.restore_report(launch, &restore).await;
        }
    }

    /// `plan2cell cell <stage> DIR --json`: the stage's notes, or why it failed.
    async fn run_stage(&self, binary: &Path, stage: &str, dir: &Path, cell_config: &Path, token: &str, timeout: Duration) -> anyhow::Result<Vec<String>> {
        let args: Vec<String> = vec![
            "cell".into(),
            stage.into(),
            dir.display().to_string(),
            "--cell-config".into(),
            cell_config.display().to_string(),
            "--json".into(),
        ];
        let mut env: Vec<(String, String)> = Some(token).filter(|t| !t.is_empty()).map(|t| (MCP_TOKEN_ENV.to_owned(), t.to_owned())).into_iter().collect();
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        let out = run(binary, &args, &env, timeout, Some(&*self.sink)).await.with_context(|| format!("plan2cell cell {stage}"))?;
        let report: Value = serde_json::from_slice(&out.stdout).unwrap_or(Value::Null);
        let entry = report["stages"].as_array().into_iter().flatten().find(|s| s["stage"] == stage);
        let notes: Vec<String> =
            entry.and_then(|e| e["notes"].as_array()).into_iter().flatten().filter_map(|n| n.as_str().map(str::to_owned)).collect();
        if out.code == Some(0) && entry.is_none_or(|e| e["ok"] != false) {
            return Ok(notes);
        }
        let error = entry
            .and_then(|e| e["error"].as_str().map(str::to_owned))
            .or_else(|| out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).map(str::to_owned))
            .unwrap_or_else(|| "no report".to_owned());
        // The first errors the projects / controllers name (a build error, a compile error).
        let details: Vec<String> = ["projects", "controllers"]
            .iter()
            .flat_map(|k| report[*k].as_array().into_iter().flatten())
            .flat_map(|p| {
                let who = p["name"].as_str().or_else(|| p["robot_ns"].as_str()).unwrap_or("?").to_owned();
                p["errors"].as_array().into_iter().flatten().filter_map(move |e| e.as_str().map(|e| format!("{who}: {e}")))
            })
            .take(3)
            .collect();
        let code = out.code.map_or("killed".to_owned(), |c| c.to_string());
        let details = if details.is_empty() { String::new() } else { format!(" ({})", details.join("; ")) };
        anyhow::bail!("plan2cell cell {stage} failed (exit {code}): {error}{details}")
    }

    /// The cell's head is deployed through the CODESYS MCP (`deploy.head.codesys-mcp`; a cell without
    /// a template: always). A hook head (`deploy.head.hook`) needs no token.
    fn needs_mcp_token(&self) -> bool {
        self.lacks("deploy.head.codesys-mcp", "the CODESYS MCP").is_none()
    }

    /// The CODESYS MCP token for `plan2cell cell` stages (empty: [`Self::needs_mcp_token`] says none).
    async fn mcp_token(&self) -> anyhow::Result<String> {
        if !self.needs_mcp_token() {
            return Ok(String::new());
        }
        self.secret(&self.mcp_secret()).await
    }

    /// A Secrets Manager string, for a child's environment only (never logged).
    async fn secret(&self, id: &str) -> anyhow::Result<String> {
        let mut args: Vec<String> = ["secretsmanager", "get-secret-value", "--secret-id", id, "--query", "SecretString", "--output", "text"]
            .map(str::to_owned)
            .to_vec();
        if let Some(region) = &self.settings.region {
            args.extend(["--region".to_owned(), region.clone()]);
        }
        let out = run(&self.settings.aws, &args, &[], self.settings.call_timeout, None).await?;
        anyhow::ensure!(
            out.code == Some(0),
            "aws secretsmanager get-secret-value {id} failed: {}",
            out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("no output")
        );
        let secret = String::from_utf8(out.stdout).context("the secret is not UTF-8")?.trim().to_owned();
        anyhow::ensure!(!secret.is_empty() && secret != "None", "the secret {id} is empty");
        Ok(secret)
    }

    /// Invoke the cell-control function with `payload`: its answer (an `{ok: false, error}` answer
    /// is returned as it is; a failed invocation or a function error is an `Err`).
    async fn invoke(&self, payload: Value) -> anyhow::Result<Value> {
        let function = self.settings.function.as_deref().context("no cell control function")?;
        let response = TempFile::new("cell-control", "json", b"")?;
        let mut args: Vec<String> = vec![
            "lambda".into(),
            "invoke".into(),
            "--function-name".into(),
            function.to_owned(),
            "--cli-binary-format".into(),
            "raw-in-base64-out".into(),
            "--cli-read-timeout".into(),
            self.settings.call_timeout.as_secs().to_string(),
            "--payload".into(),
            payload.to_string(),
            "--output".into(),
            "json".into(),
        ];
        if let Some(qualifier) = &self.settings.qualifier {
            args.extend(["--qualifier".into(), qualifier.clone()]);
        }
        if let Some(region) = &self.settings.region {
            args.extend(["--region".into(), region.clone()]);
        }
        args.push(response.path.display().to_string());
        let out = run(&self.settings.aws, &args, &[], self.settings.call_timeout + Duration::from_secs(10), None).await?;
        anyhow::ensure!(
            out.code == Some(0),
            "aws lambda invoke {function} failed (exit {}): {}",
            out.code.map_or("killed".to_owned(), |c| c.to_string()),
            out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("no output")
        );
        let meta: Value = serde_json::from_slice(&out.stdout).unwrap_or(Value::Null);
        let body = std::fs::read(&response.path).with_context(|| format!("reading {}", response.path.display()))?;
        if let Some(error) = meta["FunctionError"].as_str() {
            anyhow::bail!("{function} failed ({error}): {}", String::from_utf8_lossy(&body).chars().take(500).collect::<String>());
        }
        serde_json::from_slice(&body).with_context(|| format!("{function} answered no JSON"))
    }
}

/// The launch whose restore ran (in the wire directory: a backend restart does not run it again).
const RESTORED_FILE: &str = "restored";
/// The resume ([`OwnCell::resumed`]) this backend followed last: one line, the session's `requestedAt`.
const RESUMED_FILE: &str = "resumed";

/// The Lambda's instances (`status`, `power_status`).
fn parse_instances(list: &Value) -> Vec<InfraInstance> {
    list.as_array()
        .into_iter()
        .flatten()
        .map(|i| {
            let s = |k: &str| i[k].as_str().map(str::to_owned);
            InfraInstance {
                role: s("role").unwrap_or_default(),
                instance_id: s("instanceId").unwrap_or_default(),
                robot_ns: s("robotNs"),
                name: s("name"),
                state: s("state").unwrap_or_else(|| "unknown".to_owned()),
                ssm_ping: s("ssmPing").unwrap_or_else(|| "Unknown".to_owned()),
                private_ip: s("privateIp"),
                public_ip: s("publicIp"),
                launch_time: s("launchTime"),
                instance_type: s("instanceType"),
            }
        })
        .collect()
}

/// A start / stop is over once every instance got there.
fn end_transition(inner: &mut Inner, instances: &[InfraInstance]) {
    if let Some((what, at)) = &inner.transition {
        let target = if what == "start" { "running" } else { "stopped" };
        // An own cell's stop terminates: no instances left is stopped too.
        let there = if what == "start" { !instances.is_empty() && instances.iter().all(|i| i.state == target) } else {
            instances.iter().all(|i| matches!(i.state.as_str(), "stopped" | "terminated"))
        };
        if at.elapsed() > TRANSITION || there {
            inner.transition = None;
        }
    }
}

/// Whether the first boot put the archived boot applications of `build` back: the Lambda's restore
/// plan says `fast` (its archive matched) and the PLC host's first boot did not report otherwise
/// (`firstBoot.plcData`: `ok...` / `restored...`); without a plan (an older Lambda), the record's
/// archive of an earlier launch for this build and PLAN_ID.
fn boot_apps_restored(info: Option<&Value>, doc: &Value, build: &str, plan_id: Option<&str>, launch: u32) -> bool {
    if let Some(info) = info {
        let first = &info["firstBoot"]["plcData"];
        return info["mode"] == "fast"
            && info["build"].as_str().is_none_or(|b| b == build)
            && first.as_str().is_none_or(|r| r.starts_with("ok") || r.starts_with("restored"));
    }
    let archive = &doc["archive"];
    archive["build"].as_str() == Some(build)
        && (plan_id.is_none() || archive["planId"].as_str() == plan_id)
        && archive["launch"].as_u64().is_none_or(|l| (l as u32) < launch)
}

/// An own cell's `cell_status`, read.
struct OwnRead {
    doc: Option<Value>,
    power: Option<CellPower>,
    instances: Vec<InfraInstance>,
    all_running: bool,
    ready: bool,
    wiring: Option<Wiring>,
}

impl OwnRead {
    /// `power` is the power (`CellPower`) or a whole `power_status` answer (`{power, instances, ...}`);
    /// without instances, they come from the power's hosts.
    fn of(body: &Value) -> Self {
        let doc = Some(body["cell"].clone()).filter(Value::is_object);
        let p = &body["power"];
        let (power, outer) = if p["power"].is_object() { (&p["power"], p) } else { (p, body) };
        let power: Option<CellPower> = doc.as_ref().and_then(|_| serde_json::from_value(power.clone()).ok().filter(|_| power.is_object()));
        let mut instances = parse_instances(&outer["instances"]);
        if instances.is_empty()
            && let Some(power) = &power
        {
            instances = power.hosts.iter().map(host_instance).collect();
        }
        let live = |i: &InfraInstance| i.state != "terminated";
        instances.retain(live);
        let all_running = outer["allRunning"].as_bool().unwrap_or(!instances.is_empty() && instances.iter().all(InfraInstance::running));
        let ready = outer["ready"].as_bool().unwrap_or_else(|| {
            power.as_ref().is_some_and(|p| p.state == PowerState::Ready) || (all_running && instances.iter().all(InfraInstance::online))
        });
        let wiring = serde_json::from_value::<Wiring>(body["wiring"].clone()).ok().filter(|_| body["wiring"].is_object());
        Self { doc, power, instances, all_running, ready, wiring }
    }
}

fn host_instance(h: &PowerHost) -> InfraInstance {
    InfraInstance {
        role: h.role.clone(),
        instance_id: h.instance_id.clone(),
        robot_ns: h.robot_ns.clone(),
        state: h.state.clone(),
        ssm_ping: if h.ssm_ping.is_empty() { (if h.online { "Online" } else { "Unknown" }).to_owned() } else { h.ssm_ping.clone() },
        private_ip: h.private_ip.clone(),
        launch_time: h.launch_time.clone(),
        ..Default::default()
    }
}

/// The Lambda's `CellDoc` (and the answer's cost and images) as the viewer's [`VirtualCell`].
fn virtual_cell(doc: &Value, body: &Value, power: Option<CellPower>, restore: Option<CellRestore>, now: f64) -> VirtualCell {
    let text = |v: &Value| match v {
        Value::String(s) => Some(s.clone()),
        Value::Number(n) => Some(n.to_string()),
        _ => None,
    };
    let mut cost: CellCost = serde_json::from_value(body["cost"].clone()).unwrap_or_default();
    if cost.running_usd_per_hour == 0.0 {
        cost.running_usd_per_hour = 2.65;
    }
    if cost.note.is_empty() {
        cost.note = if doc["offMode"] == OFF_MODE_STOP {
            "~$2.65/h while on; while stopped only its disks: Stop stops the instances, Start resumes them".to_owned()
        } else {
            "~$2.65/h while on, $0 while off: Stop removes the instances, Start builds them fresh from the latest images (~6-8 min)".to_owned()
        };
    }
    // The session so far, from when its power-on was asked for.
    if cost.session_usd.is_none()
        && let Some(power) = power.as_ref().filter(|p| p.state.on())
        && let Some(since) = power.session.as_ref().filter(|s| s.op == "on").and_then(|s| s.requested_at.as_deref()).and_then(parse_iso8601)
    {
        cost.session_usd = Some(((now - since).max(0.0) / 3600.0 * cost.running_usd_per_hour * 100.0).round() / 100.0);
    }
    VirtualCell {
        id: doc["id"].as_str().unwrap_or_default().to_owned(),
        workspace: doc["workspace"].as_str().unwrap_or_default().to_owned(),
        kind: CellKind::Own,
        lifecycle: serde_json::from_value(doc["lifecycle"].clone()).unwrap_or_default(),
        created_at: text(&doc["createdAt"]),
        launch: doc["launch"].as_u64().unwrap_or(0) as u32,
        robots: doc["robots"].as_u64().unwrap_or(2) as u8,
        power,
        cost,
        images: serde_json::from_value::<Vec<CellImage>>(body["images"].clone()).unwrap_or_default(),
        restore,
        was_on_at_stop: doc["wasOnAtStop"] == true,
        message: doc["message"].as_str().map(str::to_owned),
        off_mode: doc["offMode"].as_str().unwrap_or(OFF_MODE_TERMINATE).to_owned(),
        stopped: doc["stopped"] == true,
        resume: serde_json::from_value::<CellResume>(body["resume"].clone()).ok().filter(|_| body["resume"].is_object() && doc["stopped"] == true),
        runtime: doc["runtime"].as_str().unwrap_or(RUNTIME_EC2).to_owned(),
        ide: doc["ide"].as_str().map(str::to_owned),
        networks: serde_json::from_value(doc["networks"].clone()).unwrap_or_default(),
        // The template it runs (none from an older Lambda or with templates off: today's rules).
        template: serde_json::from_value::<CellTemplateRef>(body["template"].clone()).ok().filter(|t| body["template"].is_object() && !t.id.is_empty()),
        params: doc["params"].as_object().cloned(),
    }
}

/// `networks` of a create or update request into the Lambda's payload: 1 to [`MAX_NETWORKS`], names `n1`..`n9`
/// unique, robots 1..[`MAX_NETWORK_ROBOTS`] (the Lambda checks again and allocates their VNIs and subnets).
fn networks_field(payload: &mut Value, networks: Option<&[CellNetwork]>) -> Result<(), HttpError> {
    let Some(nets) = networks else { return Ok(()) };
    if nets.is_empty() || nets.len() > MAX_NETWORKS {
        return Err((StatusCode::BAD_REQUEST, format!("networks: 1 to {MAX_NETWORKS}")));
    }
    let mut seen = std::collections::BTreeSet::new();
    for n in nets {
        let ok = n.name.len() == 2 && n.name.starts_with('n') && n.name[1..].chars().all(|c| ('1'..='9').contains(&c));
        if !ok || !seen.insert(n.name.as_str()) {
            return Err((StatusCode::BAD_REQUEST, format!("networks: unique names n1..n9, not {:?}", n.name)));
        }
        if !(1..=MAX_NETWORK_ROBOTS).contains(&n.robots) {
            return Err((StatusCode::BAD_REQUEST, format!("networks.{}: robots 1 to {MAX_NETWORK_ROBOTS}", n.name)));
        }
    }
    payload["networks"] = json!(nets.iter().map(|n| json!({"name": n.name, "robots": n.robots})).collect::<Vec<_>>());
    Ok(())
}

/// `runtime` / `ide` of a create or update request into the Lambda's payload, checked.
fn runtime_fields(payload: &mut Value, runtime: Option<&str>, ide: Option<&str>) -> Result<(), HttpError> {
    if let Some(r) = runtime {
        if !RUNTIMES.contains(&r) {
            return Err((StatusCode::BAD_REQUEST, format!("runtime must be one of {}, not {r:?}", RUNTIMES.join(", "))));
        }
        payload["runtime"] = json!(r);
    }
    if let Some(i) = ide {
        if !IDES.contains(&i) {
            return Err((StatusCode::BAD_REQUEST, format!("ide must be one of {}, not {i:?}", IDES.join(", "))));
        }
        payload["ide"] = json!(i);
    }
    Ok(())
}

/// The Lambda has no such action (an older one): the cell Lambda's `bad-request` "action must be one
/// of", or `unknown-action`.
fn older_lambda(body: &Value) -> bool {
    match body["error"]["code"].as_str() {
        Some("unknown-action") => true,
        Some("bad-request") => body["error"]["message"].as_str().is_some_and(|m| m.starts_with("action must be one of")),
        _ => false,
    }
}

/// [`InfraService::template_fields`] for an older Lambda: a legacy template's parameters as today's
/// fields (docs/tasks/cell-templates.md §8: `ec2-codesys-kuka` robots and offMode; `eks-codesys-kuka`
/// runtime eks, ide, networks and offMode). Without a template (an update), the parameters of either.
fn legacy_template_fields(payload: &mut Value, template: Option<&TemplateSelector>, params: Option<&serde_json::Map<String, Value>>) -> Result<(), HttpError> {
    let older = |what: String| (StatusCode::NOT_IMPLEMENTED, format!("the cell Lambda has no templates (an older one): {what}; deploy FactoryosCells (plan2cell/infra)"));
    let names: &[&str] = match template.map(TemplateSelector::id) {
        None => &["robots", "offMode", "ide", "networks"],
        Some(TEMPLATE_EC2_CODESYS_KUKA) => &["robots", "offMode"],
        Some(TEMPLATE_EKS_CODESYS_KUKA) => &["ide", "networks", "offMode"],
        Some(other) => return Err(older(format!("only {TEMPLATE_EC2_CODESYS_KUKA} and {TEMPLATE_EKS_CODESYS_KUKA}, not {other}"))),
    };
    if let Some(v) = template.and_then(TemplateSelector::version).filter(|v| *v != "1.0.0") {
        return Err(older(format!("only version 1.0.0 of the legacy templates, not {v}")));
    }
    let params = params.cloned().unwrap_or_default();
    if let Some(name) = params.keys().find(|k| !names.contains(&k.as_str())) {
        let of = template.map_or_else(|| "this cell".to_owned(), |t| t.id().to_owned());
        return Err((StatusCode::BAD_REQUEST, format!("params: {of} has no parameter {name} (only {})", names.join(", "))));
    }
    let bad = |name: &str| (StatusCode::BAD_REQUEST, format!("params.{name}: not a valid value"));
    if let Some(t) = template {
        payload["runtime"] = json!(if t.id() == TEMPLATE_EKS_CODESYS_KUKA { RUNTIME_EKS } else { RUNTIME_EC2 });
    }
    if let Some(robots) = params.get("robots") {
        let robots = robots.as_u64().filter(|r| (1..=8).contains(r)).ok_or_else(|| bad("robots"))?;
        payload["robots"] = json!(robots);
    }
    if let Some(mode) = params.get("offMode") {
        let mode = mode.as_str().filter(|m| OFF_MODES.contains(m)).ok_or_else(|| bad("offMode"))?;
        payload["offMode"] = json!(mode);
    }
    runtime_fields(payload, None, params.get("ide").map(|i| i.as_str().ok_or_else(|| bad("ide"))).transpose()?)?;
    let networks = params.get("networks").map(|n| serde_json::from_value::<Vec<CellNetwork>>(n.clone()).map_err(|_| bad("networks"))).transpose()?;
    networks_field(payload, networks.as_deref())
}

/// An own cell's Lambda error as an HTTP error.
fn cell_error(action: &str, body: &Value) -> HttpError {
    let message = lambda_error(body);
    let code = match body["error"]["code"].as_str() {
        Some("no-cell") => StatusCode::NOT_FOUND,
        // choose: a stopped cell with newer images (resume, fresh: false, or Start fresh, fresh: true).
        Some("exists" | "in-production" | "busy" | "not-off" | "stopping" | "not-usable" | "choose") => StatusCode::CONFLICT,
        Some("cap-reached") => StatusCode::TOO_MANY_REQUESTS,
        Some("confirm-mismatch" | "bad-request") => StatusCode::BAD_REQUEST,
        Some("capacity") => StatusCode::SERVICE_UNAVAILABLE,
        Some("unqualified") => StatusCode::FORBIDDEN,
        // A template's: not loadable from any allowed source, invalid, or beyond the Lambda's engine
        // (docs/tasks/cell-templates.md §8); the Lambda's message says which.
        Some("template-unavailable") => StatusCode::SERVICE_UNAVAILABLE,
        Some("template-invalid") => StatusCode::BAD_REQUEST,
        Some("template-engine") => StatusCode::NOT_IMPLEMENTED,
        Some("unknown-action" | "not-configured") => {
            return (StatusCode::NOT_IMPLEMENTED, format!("the cell Lambda has no {action}: deploy FactoryosCells (plan2cell/infra): {message}"));
        }
        _ => return (StatusCode::BAD_GATEWAY, format!("cell control {action}: {message}")),
    };
    (code, message)
}

/// A failed Run Control Code: the stage (`None`: before the first), the error, the head state read.
struct DeployError {
    stage: Option<&'static str>,
    error: anyhow::Error,
    head: Option<String>,
}

/// How the export of a production start was chosen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pick {
    /// `build` in the request.
    Explicit,
    /// Of the PLAN_ID the head runs.
    Deployed,
    /// That unknown: the export last deployed (its cell report).
    Report,
    /// Neither: the newest.
    Newest,
}

impl Pick {
    fn source(self) -> &'static str {
        match self {
            Pick::Explicit => "explicit",
            Pick::Deployed => "deployed",
            Pick::Report => "report",
            Pick::Newest => "newest",
        }
    }
}

/// The export the head runs, which Start production and Restart PLC runtime start: of the PLAN_ID
/// it runs ([`export_of_plan`]); with that unknown, the export last deployed (its
/// `plan2cell-cell-report.json` has a successful `deploy` stage, as `p2c-cell` finds it), else the
/// newest. `Err`: why there is none.
fn head_export(root: &Path, a: &AutoFile) -> Result<(String, PathBuf, Pick), String> {
    match a.status.deployed_plan_id.as_deref() {
        Some(plan) => export_of_plan(root, plan).map(|(b, d)| (b, d, Pick::Deployed)).ok_or_else(|| {
            format!(
                "no successful Control Code Export of PLAN_ID {plan} (the plan the head runs): export that plan, or deploy \
                 another export (Run Control Code)"
            )
        }),
        None => codegen::last_deployed_build(root)
            .map(|(b, d)| (b, d, Pick::Report))
            .or_else(|| codegen::newest_build(root).map(|(b, d)| (b, d, Pick::Newest)))
            .ok_or_else(|| "no successful Control Code Export of this cell yet: Export PLC + robot code first".to_owned()),
    }
}

/// The export of PLAN_ID `plan`: the one last deployed if it is of that plan, else the newest.
fn export_of_plan(root: &Path, plan: &str) -> Option<(String, PathBuf)> {
    codegen::last_deployed_build(root)
        .filter(|(_, d)| codegen::build_plan_id(d).is_some_and(|p| codegen::same_plan_id(&p, plan)))
        .or_else(|| codegen::newest_build_of_plan(root, plan))
}

/// The exports of the status: what production starts, what the head runs, the newest.
#[derive(Clone, Debug, Default, PartialEq)]
struct Exports {
    head_export: Option<ExportRef>,
    head_export_error: Option<String>,
    head_plan_id: Option<String>,
    newest_export: Option<ExportRef>,
    deployed_export: Option<ExportRef>,
    in_production: bool,
    stopped_in_production: bool,
    ide_project: Option<IdeProject>,
}

impl Exports {
    /// `windows_dir`: the cell.toml's `[codesys_mcp] windows_project_dir` (`{cell}` not filled in).
    fn of(root: &Path, a: &AutoFile, windows_dir: &str) -> Self {
        let export = |(build, dir): (String, PathBuf), pick: Pick| ExportRef { build, plan_id: codegen::build_plan_id(&dir), source: pick.source().to_owned() };
        let (head_export, head_export_error) = match head_export(root, a) {
            Ok((build, dir, pick)) => (Some(export((build, dir), pick)), None),
            Err(err) => (None, Some(err)),
        };
        // After a failed deploy stage the reports' export is not what the head holds.
        let reported = || codegen::last_deployed_build(root).filter(|_| !a.deploy_unknown);
        let head_plan_id =
            a.status.deployed_plan_id.clone().or_else(|| reported().and_then(|(_, dir)| codegen::build_plan_id(&dir)));
        let newest = codegen::newest_build(root);
        let deployed = a
            .deployed_build
            .as_deref()
            .and_then(|id| codegen::export_dir(root, id).map(|dir| ((id.to_owned(), dir), "deploy")))
            .or_else(|| reported().map(|b| (b, "report")));
        let ide_project = deployed.as_ref().map(|((_, dir), _)| dir).or(newest.as_ref().map(|(_, dir)| dir)).and_then(|dir| ide_project(dir, windows_dir));
        let deployed_export = deployed.map(|((build, dir), source)| ExportRef { plan_id: codegen::build_plan_id(&dir), build, source: source.to_owned() });
        Self {
            head_export,
            head_export_error,
            head_plan_id,
            newest_export: newest.map(|b| export(b, Pick::Newest)),
            deployed_export,
            in_production: a.in_production,
            stopped_in_production: a.stopped_in_production,
            ide_project,
        }
    }

    fn from_status(s: &InfraStatus) -> Self {
        Self {
            head_export: s.head_export.clone(),
            head_export_error: s.head_export_error.clone(),
            head_plan_id: s.head_plan_id.clone(),
            newest_export: s.newest_export.clone(),
            deployed_export: s.deployed_export.clone(),
            in_production: s.in_production,
            stopped_in_production: s.stopped_in_production,
            ide_project: s.ide_project.clone(),
        }
    }

    fn into_status(self, s: &mut InfraStatus) {
        s.head_export = self.head_export;
        s.head_export_error = self.head_export_error;
        s.head_plan_id = self.head_plan_id;
        s.newest_export = self.newest_export;
        s.deployed_export = self.deployed_export;
        s.in_production = self.in_production;
        s.stopped_in_production = self.stopped_in_production;
        s.ide_project = self.ide_project;
    }
}

/// The head project of the export `dir` on the Windows IDE: its name and file from
/// `plc/project.json` (the `head_plc` project), in `windows_dir` with `{cell}` the plan's cell id
/// (as `plan2cell cell` puts it there).
fn ide_project(dir: &Path, windows_dir: &str) -> Option<IdeProject> {
    let manifest: Value = serde_json::from_slice(&std::fs::read(dir.join("plc/project.json")).ok()?).ok()?;
    let head = manifest["projects"].as_array()?.iter().find(|p| p["role"] == "head_plc")?;
    let name = head["name"].as_str()?.to_owned();
    let file = head["file"].as_str().map_or_else(|| format!("{name}.project"), str::to_owned);
    let cell = manifest["plan"]["cell_id"].as_str().unwrap_or_default();
    let folder = windows_dir.replace("{cell}", cell);
    Some(IdeProject { name, path: format!("{}\\{file}", folder.trim_end_matches(['\\', '/'])) })
}

/// The MCP bridge's [`SHOW_IDE_PATH`] next to the cell.toml's `[codesys_mcp] url`
/// (`http://10.0.2.20:8766/mcp` → `http://10.0.2.20:8766/show-ide`); `None`: no http(s) URL.
fn show_ide_url(cell_toml: &str) -> Option<reqwest::Url> {
    let table = toml::from_str::<toml::Table>(cell_toml).ok()?;
    let mut url = reqwest::Url::parse(table.get("codesys_mcp")?.get("url")?.as_str()?.trim()).ok()?;
    if !matches!(url.scheme(), "http" | "https") || url.host_str().is_none() {
        return None;
    }
    url.set_path(SHOW_IDE_PATH);
    url.set_query(None);
    url.set_fragment(None);
    Some(url)
}

/// `[codesys_mcp] windows_project_dir` of the cell.toml at `path` (plan2cell's default without).
fn windows_project_dir(path: Option<&Path>) -> String {
    path.and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|text| toml::from_str::<toml::Table>(&text).ok())
        .and_then(|t| t.get("codesys_mcp")?.get("windows_project_dir")?.as_str().map(str::to_owned))
        .unwrap_or_else(|| WINDOWS_PROJECT_DIR.to_owned())
}

/// Whether the cell.toml at `path` lists KUKA virtual controllers (`[[kuka_vc.controller]]`).
fn has_kuka_vc(path: &Path) -> bool {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|text| toml::from_str::<toml::Table>(&text).ok())
        .and_then(|t| t.get("kuka_vc")?.get("controller")?.as_array().map(|c| !c.is_empty()))
        .unwrap_or(false)
}

/// What the `deploy` stage's notes say of the head project in the PLC IDE (plan2cell's
/// `<name> open in the PLC IDE (<path>)` / `<name> not opened in the PLC IDE...`): opened, and
/// the note (`None`: no such note, an older plan2cell).
fn ide_outcome(notes: &[String]) -> (Option<bool>, Option<String>) {
    match notes.iter().rev().find(|n| n.contains(" open in the PLC IDE") || n.contains(" not opened in the PLC IDE")) {
        Some(n) if n.contains(" not opened in the PLC IDE") => (Some(false), Some(n.clone())),
        Some(n) if n.contains(" open in the PLC IDE") => (Some(true), Some(n.clone())),
        _ => (None, None),
    }
}

/// The message of an `{ok: false, error: {code, message}}` answer.
/// The Lambda's per-instance start results (`instances[]` of `start`).
fn instance_starts(body: &Value) -> Vec<InstanceStart> {
    body["instances"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|i| {
            let s = |k: &str| i[k].as_str().map(str::to_owned);
            let error = i["error"]["name"].as_str().map(|name| match i["error"]["message"].as_str() {
                Some(m) if !m.is_empty() => format!("{name}: {m}"),
                _ => name.to_owned(),
            });
            // What made it fall back: the first attempt's error (its own type), else the failure's.
            let first_error = i["attempts"][0]["error"]["name"].as_str().or(i["error"]["name"].as_str()).map(str::to_owned);
            let result = s("result").unwrap_or_else(|| if i["skipped"] == true { "skipped".to_owned() } else { "started".to_owned() });
            InstanceStart {
                role: s("role").unwrap_or_default(),
                instance_id: s("instanceId").unwrap_or_default(),
                previous_type: s("previousType"),
                new_type: s("newType"),
                first_error: first_error.filter(|_| result != "started"),
                error: error.filter(|_| result == "failed"),
                result,
            }
        })
        .collect()
}

fn lambda_error(body: &Value) -> String {
    let e = &body["error"];
    match (e["code"].as_str(), e["message"].as_str()) {
        (Some(code), Some(message)) => format!("{message} ({code})"),
        (None, Some(message)) => message.to_owned(),
        _ => format!("unexpected answer: {}", body.to_string().chars().take(300).collect::<String>()),
    }
}

/// The `restore_report` payload for the cell-control Lambda (strings cut to what it takes).
fn restore_report_payload(launch: u32, restore: &CellRestore) -> Value {
    let cut = |s: &Option<String>| s.as_deref().map(|s| s.chars().take(RESTORE_REPORT_MAX).collect::<String>());
    json!({
        "action": "restore_report", "launch": launch, "build": cut(&restore.build), "planId": cut(&restore.plan_id),
        "ok": restore.state == JobState::Done, "mode": restore.mode, "message": cut(&restore.message),
    })
}

/// After a full restore (Deploy only / Run Control Code) failed with `message`: reset the CODESYS
/// IDE and run it once more? Only for a CODESYS tool error (the IDE's state, e.g. after many
/// failed logins), never for a project error (`C0046`, compile / build errors: deterministic) or
/// when the head is partly downloaded (`deploy_unknown`).
fn retry_after_ide_reset(message: &str, deploy_unknown: bool) -> bool {
    !deploy_unknown && codesys_tool_error(message) && !project_error(message)
}

/// plan2cell's `codesys tool <name>: …` (`Error::Tool`), not `… (codesys tool <name>)`: a dialog
/// that waits for a person.
fn codesys_tool_error(message: &str) -> bool {
    message.match_indices("codesys tool ").any(|(i, _)| !message[..i].ends_with('('))
}

/// A project's compile / build error: a CODESYS code (`C0046:`), or the build's own words.
fn project_error(message: &str) -> bool {
    let bytes = message.as_bytes();
    let code = (0..bytes.len().saturating_sub(5)).any(|i| {
        bytes[i] == b'C'
            && (i == 0 || !bytes[i - 1].is_ascii_alphanumeric())
            && bytes[i + 1..i + 5].iter().all(u8::is_ascii_digit)
            && bytes[i + 5] == b':'
    });
    let lower = message.to_ascii_lowercase();
    code || ["not built:", "compile error", "build error"].iter().any(|w| lower.contains(w))
}

/// The PLC host booted (EC2 launch / start) less than [`FRESH_BOOT_S`] ago: its CODESYS
/// runtimes started with it. Not for a long-lived cell (its demo runtimes stop after ~2 h).
///
/// A resume of a stopped own cell counts as a fresh boot, on purpose: EC2's `LaunchTime` is the
/// last start's, and the runtimes (docker, restart policy) start with that boot, so their 2 h demo
/// period is as new as after a launch and the boot application on the disk is loaded again. The
/// resume's `start` therefore runs without `--restart-runtime`; a runtime that does not answer is
/// still restarted by `start` itself.
fn fresh_boot(status: &InfraStatus, semantic_hosts: bool, now: f64) -> bool {
    plc_host(status, semantic_hosts)
        .filter(|i| i.running())
        .and_then(|i| i.launch_time.as_deref())
        .and_then(parse_iso8601)
        .is_some_and(|boot| (now - boot) < FRESH_BOOT_S && (now - boot) > -60.0)
}

/// The head PLC state (`CS_RUNNING`, ...) a note names, the last one if several.
fn cell_state_in(note: &str) -> Option<String> {
    note.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .rev()
        .find(|w| w.len() > 3 && w.starts_with("CS_"))
        .map(str::to_owned)
}

/// The host of `semantic` in the status: with `semantic_hosts` and a template, by the template's
/// semantics (an EKS cell's head is `n1/plcHost`); else by today's role name `role`. A hook
/// template's cell (engine 2, [`CellTemplateRef::hook`]) has no legacy role names: always by its
/// semantics.
fn host_of<'a>(status: &'a InfraStatus, semantic_hosts: bool, semantic: &str, role: &str) -> Option<&'a InfraInstance> {
    let template = status.virtual_cell.as_ref().and_then(|c| c.template.as_ref()).filter(|t| semantic_hosts || t.hook());
    status.instance(template.and_then(|t| t.role_of(semantic)).unwrap_or(role))
}

/// The head PLC's host ([`host_of`]; today's [`ROLE_PLC_HOST`]).
fn plc_host(status: &InfraStatus, semantic_hosts: bool) -> Option<&InfraInstance> {
    host_of(status, semantic_hosts, cell_wiring::SEMANTIC_HEAD_PLC, ROLE_PLC_HOST)
}

/// The cell's template does not have CODESYS demo runtimes (`runtime.codesys`): nothing to restart.
fn no_codesys_runtime(status: &InfraStatus) -> bool {
    status.virtual_cell.as_ref().and_then(|c| c.template.as_ref()).is_some_and(|t| !t.has("runtime.codesys"))
}

/// The status line about the CODESYS demo period.
fn demo_note(status: &InfraStatus, semantic_hosts: bool) -> String {
    if no_codesys_runtime(status) {
        return "No CODESYS demo runtime in this cell (its template has no runtime.codesys).".to_owned();
    }
    let restart = "Start production restarts them (--restart-runtime)";
    if let Some(rt) = status.plc_runtime.as_ref().filter(|r| r.error.is_none()) {
        let now = unix_s(SystemTime::now());
        return match rt.stops_s {
            _ if !rt.running => "CODESYS demo runtime: stopped; Restart PLC runtime starts a new 2 h period.".to_owned(),
            Some(end) if end > now => format!(
                "CODESYS demo runtime: stops at {} (in {} min; {}); Restart PLC runtime starts a new 2 h period.",
                utc_hhmm(end),
                ((end - now) / 60.0).ceil() as u64,
                rt.source.as_deref().unwrap_or("estimate")
            ),
            Some(end) => format!("CODESYS demo runtime: its 2 h period ended at {}; Restart PLC runtime.", utc_hhmm(end)),
            None => "CODESYS runtime: running, no demo stop found in its log.".to_owned(),
        };
    }
    if let Some(at) = status.cell.runtime_started_s {
        let end = at + CODESYS_DEMO_S;
        let now = unix_s(SystemTime::now());
        if now < end {
            return format!(
                "CODESYS demo runtimes restarted at {}: they stop around {} (~{} min left); {restart}.",
                utc_hhmm(at),
                utc_hhmm(end),
                ((end - now) / 60.0).ceil() as u64
            );
        }
        return format!("CODESYS demo runtimes restarted at {} have stopped (2 h demo period): {restart}.", utc_hhmm(at));
    }
    if let Some(boot) = plc_host(status, semantic_hosts)
        .filter(|i| i.running())
        .and_then(|i| i.launch_time.as_deref())
        .and_then(parse_iso8601)
    {
        return format!(
            "CODESYS runs in demo mode: runtimes stop ~2 h after they start (PLC host up since {}, so around {} at the latest after a boot); {restart}.",
            utc_hhmm(boot),
            utc_hhmm(boot + CODESYS_DEMO_S)
        );
    }
    format!("CODESYS runs in demo mode: the runtimes stop after about 2 h; {restart}.")
}

/// `<cell>/plan2cell/auto-restart.json`: the setting, the last attempt, and what the head runs.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
struct AutoFile {
    #[serde(flatten)]
    status: AutoRestartStatus,
    /// The last production start succeeded and the PLC host was not stopped since: the head's
    /// runtime is meant to run.
    in_production: bool,
    /// The cell was in production when Stop cell stopped it (used up by the next Start cell,
    /// which then starts production again).
    stopped_in_production: bool,
    /// When that start finished (Unix seconds).
    production_done_s: Option<f64>,
    /// When the last production job (any outcome, also by hand) finished: the backoff counts
    /// from it as well.
    last_job_s: Option<f64>,
    /// The export of the last production job (build id, directory): its containers are read.
    production_build: Option<String>,
    production_dir: Option<String>,
    /// The head container's `RestartCount` as last taken (`None`: take it at the next read).
    head_restart_count: Option<u64>,
    /// The export a deploy job of this backend last downloaded to the head (cleared by a failed
    /// `deploy` stage).
    deployed_build: Option<String>,
    /// A `deploy` stage of this backend failed (part of an export may be on the head) and none
    /// passed since: what the head holds is not known, so the deploy reports (an earlier
    /// export's successful deploy) do not count as deployed.
    deploy_unknown: bool,
    /// Another backend changed the cell's production after this one's last production start
    /// (the lease's `lastChange`): why auto-restart idles (until a production start of its own).
    foreign_change: Option<String>,
}

impl AutoFile {
    /// The last attempt or production job, whichever is later.
    fn last_try_s(&self) -> Option<f64> {
        match (self.status.last_attempt_s, self.last_job_s) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        }
    }
}

fn load_auto(root: &Path) -> AutoFile {
    let mut a: AutoFile =
        std::fs::read(root.join(AUTO_RESTART_FILE)).ok().and_then(|b| serde_json::from_slice(&b).ok()).unwrap_or_default();
    if a.status.threshold_s == 0 {
        a.status.threshold_s = AUTO_RESTART_THRESHOLD_S;
    }
    // Derived, not stored.
    a.status.message = None;
    a.status.next_allowed_s = None;
    a.status.inactive = None;
    a
}

fn save_auto(root: &Path, a: &AutoFile) -> anyhow::Result<()> {
    let path = root.join(AUTO_RESTART_FILE);
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    let mut a = a.clone();
    a.status.message = None;
    a.status.next_allowed_s = None;
    a.status.inactive = None;
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, serde_json::to_vec_pretty(&a)?).with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, &path).with_context(|| format!("writing {}", path.display()))
}

/// The status line of auto-restart; `job`: a bring-up or production job runs now.
fn auto_message(a: &AutoRestartStatus, job: bool) -> Option<String> {
    let at = a.last_attempt_s.map(utc_hhmm);
    if let Some(blocked) = &a.blocked {
        return Some(format!("auto-restart holds: {blocked} (switch it on again to retry)"));
    }
    if let Some(why) = &a.inactive {
        return Some(format!("auto-restart inactive: {why}"));
    }
    match (&at, &a.last_result, &a.last_error) {
        (Some(at), Some(result), _) => Some(format!("auto-restart at {at}: {result}")),
        (Some(at), None, Some(err)) => Some(format!(
            "auto-restart at {at} failed: {err}{}",
            a.last_attempt_s.filter(|_| a.enabled).map(|t| format!("; next attempt not before {}", utc_hhmm(t + AUTO_RESTART_BACKOFF_S))).unwrap_or_default()
        )),
        (Some(at), None, None) if job => Some(format!("auto-restart at {at}: running…")),
        // No outcome and no job: the backend stopped during the attempt.
        (Some(at), None, None) => Some(format!("auto-restart at {at}: interrupted (no outcome; the backend restarted during the job?)")),
        _ if a.enabled => Some(format!("auto-restart on: {} min before the CODESYS demo stop", a.threshold_s.div_ceil(60))),
        _ => None,
    }
}

/// Why auto-restart should run now, if it should: on, not held, the cell in production, the last
/// attempt or production job 10 min ago or more, and the first runtime of the export stops within
/// the threshold, or the head's runtime is not running or restarted by itself, or the head (`head`, live)
/// stopped production with an error of its own (not Stop production's `ERR_OPERATOR_STOP`) since the last
/// production start and stayed in it for [`HEAD_ERROR_GRACE_S`] (e.g. `ERR_DRIVES_LOST` after a PROFINET hiccup).
fn auto_reason(a: &AutoFile, rt: &PlcRuntime, head: Option<&HeadLive>, now: f64) -> Option<String> {
    let s = &a.status;
    if !s.enabled || s.blocked.is_some() || rt.error.is_some() || !a.in_production {
        return None;
    }
    if a.last_try_s().is_some_and(|t| now < t + AUTO_RESTART_BACKOFF_S) {
        return None;
    }
    if let (Some(left), Some(at)) = (rt.seconds_left, rt.stops_s)
        && left < s.threshold_s as f64
    {
        let who = rt.container.as_deref().unwrap_or("the runtime");
        return Some(if left <= 0.0 {
            format!("the demo period of {who} ended at {}", utc_hhmm(at))
        } else {
            format!("{who} stops at {} (in {} s)", utc_hhmm(at), left.round() as i64)
        });
    }
    if !rt.running {
        return Some("the head's CODESYS runtime is not running (the cell was in production)".to_owned());
    }
    if let Some(h) = head.filter(|h| {
        h.state == "CS_ERROR"
            && h.error.as_deref() != Some("ERR_OPERATOR_STOP")
            && now - h.seen_s < HEAD_FRESH_S
            && now - h.since_s >= HEAD_ERROR_GRACE_S
            && h.since_s > a.production_done_s.unwrap_or_default()
    }) {
        return Some(format!(
            "the head stopped production: CS_ERROR {} after {} cycle(s), since {}",
            h.error.as_deref().unwrap_or("?"),
            h.cycles,
            utc_hhmm(h.since_s)
        ));
    }
    if own_restart(a, rt) {
        let at = rt.started_s.map(|t| format!(" at {}", utc_hhmm(t))).unwrap_or_default();
        return Some(format!(
            "docker restarted the head's CODESYS runtime by itself{at} (restart count {} → {}; the cell was in production)",
            a.head_restart_count.unwrap_or_default(),
            rt.restart_count.unwrap_or_default()
        ));
    }
    None
}

/// The head container's `RestartCount` went up since it was taken: its restart policy restarted
/// it (the runtime stopped by itself). `docker restart` and a recreated container do not count.
fn own_restart(a: &AutoFile, rt: &PlcRuntime) -> bool {
    matches!((a.head_restart_count, rt.restart_count), (Some(was), Some(now)) if now > was)
}

/// The PLAN_ID the head runs, from a refusal of `plan2cell cell start`'s PLAN_ID guard
/// (`the head runs PLAN_ID 16#00AA (GVL_Cell.PLAN_ID), this export is plan ...`).
fn guard_plan_id(error: &str) -> Option<String> {
    let rest = &error[error.find("the head runs PLAN_ID ")? + "the head runs PLAN_ID ".len()..];
    let id: String = rest.chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '#' || *c == '_').collect();
    // `WORD#16#9573` (a typed online value) → `16#9573`, as the exports write it.
    let id = codegen::plain_plan_id(&id).to_owned();
    (!id.is_empty()).then_some(id)
}

/// [`PlcRuntime`] of a `plan2cell cell runtime --json` report (`plan2cell/cell-runtime@1`).
fn plc_runtime(report: &Value) -> PlcRuntime {
    let runtimes: Vec<&Value> = report["runtimes"].as_array().into_iter().flatten().collect();
    let num = |v: &Value, k: &str| v[k].as_f64();
    let head = runtimes.iter().find(|r| r["head"] == true);
    let first = runtimes
        .iter()
        .filter(|r| r["running"] == true && r["stopsS"].is_number())
        .min_by(|a, b| num(a, "stopsS").unwrap_or(f64::MAX).total_cmp(&num(b, "stopsS").unwrap_or(f64::MAX)));
    PlcRuntime {
        running: head.is_some_and(|h| h["running"] == true && num(h, "secondsLeft").is_none_or(|l| l > 0.0)),
        started_s: head.and_then(|h| num(h, "startedS")),
        restart_count: head.and_then(|h| h["restartCount"].as_u64()),
        stops_s: first.and_then(|r| num(r, "stopsS")),
        seconds_left: first.and_then(|r| num(r, "secondsLeft")),
        source: first.and_then(|r| r["source"].as_str().map(str::to_owned)),
        container: first.and_then(|r| r["container"].as_str().map(str::to_owned)),
        warning: first.and_then(|r| r["warning"].as_str().map(str::to_owned)),
        runtimes: runtimes
            .iter()
            .map(|r| RuntimeBrief {
                container: r["container"].as_str().unwrap_or_default().to_owned(),
                head: r["head"] == true,
                running: r["running"] == true,
                stops_s: num(r, "stopsS"),
            })
            .collect(),
        checked_s: 0.0,
        build: None,
        error: (head.is_none()).then(|| "no head PLC runtime (codesys-master) on the PLC host".to_owned()),
    }
}

/// `2026-10-12T08:00:00.000Z` → Unix seconds.
fn parse_iso8601(text: &str) -> Option<f64> {
    let (date, time) = text.split_once('T')?;
    let mut d = date.split('-').map(|p| p.parse::<i64>());
    let (y, m, day) = (d.next()?.ok()?, d.next()?.ok()?, d.next()?.ok()?);
    let time = time.trim_end_matches('Z');
    let time = time.split(['+']).next()?;
    let mut t = time.split(':');
    let (h, min) = (t.next()?.parse::<i64>().ok()?, t.next()?.parse::<i64>().ok()?);
    let s: f64 = t.next().unwrap_or("0").parse().ok()?;
    // Howard Hinnant's days_from_civil.
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    Some((days * 86_400 + h * 3600 + min * 60) as f64 + s)
}

fn tail_chars(text: &str, n: usize) -> String {
    let count = text.chars().count();
    text.chars().skip(count.saturating_sub(n)).collect()
}

fn unix_s(time: SystemTime) -> f64 {
    time.duration_since(SystemTime::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// What a command printed.
struct Output {
    code: Option<i32>,
    stdout: Vec<u8>,
    /// Its last stderr lines.
    stderr_tail: String,
}

/// `program args…` (no shell) with `env` added, in a process group of its own that a timeout
/// kills whole. With `log`, each stderr line goes there as well.
async fn run(program: &Path, args: &[String], env: &[(String, String)], timeout: Duration, log: Option<&dyn Sink>) -> anyhow::Result<Output> {
    let mut command = Command::new(program);
    command
        .args(args)
        .envs(env.iter().map(|(k, v)| (k.as_str(), v.as_str())))
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true);
    #[cfg(unix)]
    command.process_group(0);
    // A fake `aws` just written by a test (or a CLI being replaced) can be briefly busy.
    let mut child = codegen::spawn_when_not_busy(&mut command).await.with_context(|| format!("starting {}", program.display()))?;
    let group = child.id();
    let mut stdout = child.stdout.take().context("no stdout")?;
    let stderr = child.stderr.take().context("no stderr")?;
    let work = async {
        let read_err = async {
            let mut tail: std::collections::VecDeque<String> = std::collections::VecDeque::new();
            let mut lines = BufReader::new(stderr).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                if let Some(log) = log {
                    log.log(format!("[infra] {line}"));
                }
                tail.push_back(line);
                if tail.len() > 20 {
                    tail.pop_front();
                }
            }
            tail.into_iter().collect::<Vec<_>>().join("\n")
        };
        let read_out = async {
            let mut out = Vec::new();
            (&mut stdout).take(64 << 20).read_to_end(&mut out).await.map(|_| out)
        };
        let (out, err) = tokio::join!(read_out, read_err);
        let status = child.wait().await?;
        anyhow::Ok(Output { code: status.code(), stdout: out?, stderr_tail: err })
    };
    match tokio::time::timeout(timeout, work).await {
        Ok(result) => result,
        Err(_) => {
            codegen::kill_group(group);
            anyhow::bail!("{} timed out after {} s", program.display(), timeout.as_secs())
        }
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::INFRA_STATUS, get(status))
        .route(api::INFRA_START, post(start))
        .route(api::INFRA_STOP, post(stop))
        .route(api::INFRA_BRINGUP, post(bringup))
        .route(api::INFRA_PRODUCTION, post(production))
        .route(api::INFRA_PRODUCTION_STOP, post(stop_production))
        .route(api::INFRA_DEPLOY, post(deploy))
        .route(api::INFRA_DEPLOY_CHECK, post(deploy_check))
        .route(api::INFRA_DEPLOY_FIX, post(deploy_fix_route))
        .route(api::INFRA_KVC, post(kvc_route))
        .route(api::INFRA_KVC_READ, post(kvc_read_route))
        .route(api::INFRA_AUTO_RESTART, get(auto_restart).put(set_auto_restart))
        .route(api::INFRA_CELL_POWER, get(cell_power).post(set_cell_power))
        .route(api::INFRA_CELLS, get(list_cells).post(create_cell))
        .route(api::INFRA_TEMPLATES, get(list_templates))
        .route(api::INFRA_CELL_ITEM, delete(delete_cell).patch(update_cell).put(update_cell))
}

async fn list_cells(State(state): State<AppState>) -> Json<CellsList> {
    Json(state.infra.cells().await)
}

async fn list_templates(State(state): State<AppState>) -> Json<CellTemplates> {
    Json(state.infra.templates().await)
}

async fn create_cell(State(state): State<AppState>, bytes: Bytes) -> Result<(StatusCode, Json<VirtualCell>), HttpError> {
    state.infra.create_cell(body(&bytes)?).await.map(|c| (StatusCode::ACCEPTED, Json(c)))
}

async fn update_cell(State(state): State<AppState>, UrlPath(id): UrlPath<String>, bytes: Bytes) -> Result<Json<VirtualCell>, HttpError> {
    state.infra.update_cell(&id, body(&bytes)?).await.map(Json)
}

async fn delete_cell(State(state): State<AppState>, UrlPath(id): UrlPath<String>, bytes: Bytes) -> Result<(StatusCode, Json<VirtualCell>), HttpError> {
    state.infra.delete_cell(&id, body(&bytes)?).await.map(|c| (StatusCode::ACCEPTED, Json(c)))
}

/// An optional JSON body (empty: the defaults).
fn body<T: serde::de::DeserializeOwned + Default>(bytes: &Bytes) -> Result<T, HttpError> {
    if bytes.iter().all(u8::is_ascii_whitespace) {
        return Ok(T::default());
    }
    serde_json::from_slice(bytes).map_err(|err| (StatusCode::BAD_REQUEST, format!("invalid request: {err}")))
}

async fn status(State(state): State<AppState>) -> Json<InfraStatus> {
    Json(state.infra.current().await)
}

// The user's actions hold the cell's lease (see the module docs).

async fn start(State(state): State<AppState>, bytes: Bytes) -> Result<Json<InfraStatus>, HttpError> {
    let infra = &state.infra;
    infra.leased("start", true, infra.start_stop("start", body(&bytes)?)).await.map(Json)
}

async fn stop(State(state): State<AppState>, bytes: Bytes) -> Result<Json<InfraStatus>, HttpError> {
    let infra = &state.infra;
    infra.leased("stop", true, infra.start_stop("stop", body(&bytes)?)).await.map(Json)
}

async fn bringup(State(state): State<AppState>, bytes: Bytes) -> Result<(StatusCode, Json<InfraStatus>), HttpError> {
    let infra = &state.infra;
    infra.leased("bring-up", false, infra.bringup(body(&bytes)?)).await.map(|s| (StatusCode::ACCEPTED, Json(s)))
}

async fn production(State(state): State<AppState>, bytes: Bytes) -> Result<(StatusCode, Json<InfraStatus>), HttpError> {
    let infra = &state.infra;
    infra.leased("production", true, infra.production(body(&bytes)?)).await.map(|s| (StatusCode::ACCEPTED, Json(s)))
}

async fn stop_production(State(state): State<AppState>) -> Result<(StatusCode, Json<InfraStatus>), HttpError> {
    let infra = &state.infra;
    infra.leased("production-stop", true, infra.stop_production()).await.map(|s| (StatusCode::ACCEPTED, Json(s)))
}

async fn deploy(State(state): State<AppState>, bytes: Bytes) -> Response {
    let infra = &state.infra;
    let request: DeployRequest = match body(&bytes) {
        Ok(r) => r,
        Err(err) => return err.into_response(),
    };
    let starts = request.stages.as_ref().map_or(request.start, |s| s.iter().any(|s| s == "start"));
    let purpose = if starts { "deploy" } else { "deploy-only" };
    match infra.leased(purpose, true, infra.deploy(request)).await {
        Ok(s) => (StatusCode::ACCEPTED, Json(s)).into_response(),
        // A refusal of the deployability check: its report, as JSON.
        Err((code, body)) if body.starts_with('{') => (code, [(axum::http::header::CONTENT_TYPE, "application/json")], body).into_response(),
        Err(err) => err.into_response(),
    }
}

async fn deploy_check(State(state): State<AppState>, bytes: Bytes) -> Result<Json<DeployCheck>, HttpError> {
    state.infra.deploy_check(body(&bytes)?).await.map(Json)
}

async fn deploy_fix_route(State(state): State<AppState>, bytes: Bytes) -> Result<Json<DeployFixResult>, HttpError> {
    let request: DeployFixRequest = body(&bytes)?;
    // Under the cell's lease (it changes the Hardware Configuration and exports).
    let infra = state.infra.clone();
    infra.leased("deploy-fix", false, deploy_fix(&state, request)).await.map(Json)
}

async fn kvc_route(State(state): State<AppState>, bytes: Bytes) -> Result<Json<Value>, HttpError> {
    state.infra.kvc_op(body(&bytes)?).await.map(Json)
}

async fn kvc_read_route(State(state): State<AppState>, bytes: Bytes) -> Result<Json<Value>, HttpError> {
    let request: KvcRequest = body(&bytes)?;
    if state.infra.kvc_changes(&request)? {
        return Err((StatusCode::BAD_REQUEST, format!("{} {} changes the controller: use vcell_kvc", request.op, request.arg.unwrap_or_default())));
    }
    state.infra.kvc_op(request).await.map(Json)
}

async fn cell_power(State(state): State<AppState>) -> Result<Json<CellPower>, HttpError> {
    state.infra.cell_power().await.map(Json)
}

/// Start / Stop virtual cell: the supervisor and operators alike, under the cell's lease.
async fn set_cell_power(State(state): State<AppState>, bytes: Bytes) -> Result<Json<InfraStatus>, HttpError> {
    let infra = &state.infra;
    let request: CellPowerRequest = body(&bytes)?;
    let purpose = if request.action == "off" { "power-off" } else { "power-on" };
    infra.leased(purpose, true, infra.power(request)).await.map(Json)
}

async fn auto_restart(State(state): State<AppState>) -> Json<AutoRestartStatus> {
    Json(state.infra.auto_restart())
}

async fn set_auto_restart(State(state): State<AppState>, Json(request): Json<AutoRestartSettings>) -> Result<Json<AutoRestartStatus>, HttpError> {
    state.infra.set_auto_restart(request).map(Json)
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    /// A stand-in for the `aws` CLI: `secretsmanager get-secret-value` prints `tok-123`; `lambda
    /// invoke` answers like the cell-control Lambda, with the instances' state in `state` (start /
    /// stop change it at once) and a bring-up that is done on its second `bringupResult` (failed if
    /// `fail-bringup` exists, exit 0 without a marker with `cut-bringup`, busy with `busy-result`;
    /// `busy-bringup`: another bring-up still runs). `lease`: an unknown action (an older Lambda)
    /// unless `lease-on` exists; then `acquire` is busy with the lease in `lease-busy` (if it
    /// exists), `status` answers `lease-status`, the rest ok; each payload goes to `lease.calls`.
    /// Power (`power_status`, `power_on`, `power_off`, `touch`): an unknown action (an older
    /// Lambda) unless `power-on` exists; then the phase is `power-phase` (default from `state`),
    /// `power_on` starts (phase `bring-up`; `power-stopping`: refused), `power_off` stops (refused
    /// as in production with `prod-busy` unless `"force":true`; `power-by` names who stopped);
    /// payloads go to `power.calls` (`touch` to `touch.calls`).
    /// Every call's arguments go to `aws.calls`.
    const FAKE_AWS: &str = r#"#!/bin/sh
dir=$(dirname "$0")
echo "$*" >> "$dir/aws.calls"
if [ "$1" = "secretsmanager" ]; then echo "tok-123"; exit 0; fi
[ "$1" = "lambda" ] || { echo "unknown command $1" >&2; exit 2; }
payload=""; last=""
while [ $# -gt 0 ]; do
  case "$1" in --payload) payload="$2"; shift;; esac
  last="$1"; shift
done
state=$(cat "$dir/state" 2>/dev/null || echo stopped)
# booting: that many status reads still say pending after a start.
if [ "$state" = "running" ] && [ -f "$dir/booting" ] && [ "$(cat "$dir/booting")" -gt 0 ] && echo "$payload" | grep -q '"action":"status"'; then
  echo $(( $(cat "$dir/booting") - 1 )) > "$dir/booting"; state=pending
fi
if [ "$state" = "running" ]; then ping=Online; dcv='"https://ec2-1-2-3-4.compute.amazonaws.com:8443/"'; ready=true; else ping=ConnectionLost; dcv=null; ready=false; fi
case "$payload" in
  *'"action":"lease"'*)
    echo "$payload" >> "$dir/lease.calls"
    if [ ! -f "$dir/lease-on" ]; then echo '{"ok":false,"action":"lease","error":{"code":"unknown-action","message":"action must be one of status, start, stop, bringup, bringupResult"}}' > "$last"
    elif [ -f "$dir/lease-busy" ] && echo "$payload" | grep -q '"op":"acquire"'; then echo "{\"ok\":false,\"action\":\"lease\",\"error\":{\"code\":\"busy\",\"message\":\"held\"},\"lease\":$(cat "$dir/lease-busy")}" > "$last"
    elif echo "$payload" | grep -q '"op":"status"'; then echo "{\"ok\":true,\"action\":\"lease\",\"lease\":$(cat "$dir/lease-status" 2>/dev/null || echo '{"holder":null}')}" > "$last"
    else echo '{"ok":true,"action":"lease"}' > "$last"; fi ;;
  *'"action":"power_'*|*'"action":"touch"'*)
    if [ ! -f "$dir/power-on" ]; then echo '{"ok":false,"error":{"code":"unknown-action","message":"action must be one of status, start, stop, bringup, bringupResult"}}' > "$last"
    elif echo "$payload" | grep -q '"action":"touch"'; then echo "$payload" >> "$dir/touch.calls"; echo '{"ok":true,"action":"touch"}' > "$last"
    else
      echo "$payload" >> "$dir/power.calls"
      case "$payload" in
        *'"action":"power_on"'*)
          if [ -f "$dir/power-stopping" ]; then echo '{"ok":false,"action":"power_on","error":{"code":"stopping","message":"kukaVc1 still stopping"}}' > "$last"; payload=done
          else echo running > "$dir/state"; state=running; ping=Online; ready=true; echo bring-up > "$dir/power-phase"; fi ;;
        *'"action":"power_off"'*)
          if [ -f "$dir/prod-busy" ] && ! echo "$payload" | grep -q '"force":true'; then
            echo '{"ok":false,"action":"power_off","error":{"code":"in-production","message":"the cell is in production (plan2cell-test): Stop production first, or power off with force"}}' > "$last"; payload=done
          else echo stopped > "$dir/state"; state=stopped; ping=ConnectionLost; ready=false; dcv=null; echo off > "$dir/power-phase"; fi ;;
      esac
      if [ "$payload" != done ]; then
        phase=$(cat "$dir/power-phase" 2>/dev/null || { [ "$state" = running ] && echo ready || echo off; })
        case "$phase" in off) pst=off ;; ready) pst=ready ;; stopping) pst=stopping ;; failed) pst=failed ;; not-ready) pst=degraded ;; *) pst=booting ;; esac
        by=$(cat "$dir/power-by" 2>/dev/null || echo factoryos)
        cat > "$last" <<JSON
{"ok":true,"action":"power_status","power":{"state":"$pst","phase":"$phase","since":null,"message":"phase $phase","hosts":[],"checks":[],
 "session":{"op":"on","by":"$by","done":false,"history":[]},"idle":null,"production":null},"instances":[
 {"role":"plcHost","instanceId":"i-plcHost","name":"plc","state":"$state","ssmPing":"$ping","launchTime":"2026-10-12T08:00:00.000Z"},
 {"role":"windowsIde","instanceId":"i-windowsIde","state":"$state","ssmPing":"$ping","publicIp":"203.0.113.4"},
 {"role":"kukaVc1","instanceId":"i-kukaVc1","robotNs":"robot1","state":"$state","ssmPing":"$ping"},
 {"role":"kukaVc2","instanceId":"i-kukaVc2","robotNs":"robot2","state":"$state","ssmPing":"$ping"}],
 "allRunning":$ready,"allOnline":$ready,"ready":$ready,"dcvUrl":$dcv}
JSON
      fi
    fi ;;
  *'"action":"status"'*)
    cat > "$last" <<JSON
{"ok":true,"action":"status","instances":[
 {"role":"plcHost","instanceId":"i-plcHost","name":"plc","state":"$state","ssmPing":"$ping","launchTime":"2026-10-12T08:00:00.000Z"},
 {"role":"windowsIde","instanceId":"i-windowsIde","state":"$state","ssmPing":"$ping","publicIp":"203.0.113.4"},
 {"role":"kukaVc1","instanceId":"i-kukaVc1","robotNs":"robot1","state":"$state","ssmPing":"$ping"},
 {"role":"kukaVc2","instanceId":"i-kukaVc2","robotNs":"robot2","state":"$state","ssmPing":"$ping"}],
 "allRunning":$ready,"allOnline":$ready,"ready":$ready,"dcvUrl":$dcv}
JSON
    ;;
  *'"action":"start"'*) echo running > "$dir/state"
    ide='{"role":"windowsIde","instanceId":"i-windowsIde","previousState":"stopped","currentState":"pending","previousType":"c5a.4xlarge","result":"started"}'
    [ -f "$dir/start-fallback" ] && ide='{"role":"windowsIde","instanceId":"i-windowsIde","previousState":"stopped","currentState":"pending","previousType":"c5a.4xlarge","newType":"c5.4xlarge","result":"fallback","attempts":[{"type":"c5a.4xlarge","error":{"name":"InsufficientInstanceCapacity","message":"no capacity"}}]}'
    [ -f "$dir/start-fail-ide" ] && ide='{"role":"windowsIde","instanceId":"i-windowsIde","previousState":"stopped","currentState":"stopped","previousType":"c5a.4xlarge","result":"failed","error":{"name":"InsufficientInstanceCapacity","message":"no type started (c5a.4xlarge: InsufficientInstanceCapacity, c5.4xlarge: InsufficientInstanceCapacity)"}}'
    echo "{\"ok\":true,\"action\":\"start\",\"instances\":[{\"role\":\"plcHost\",\"instanceId\":\"i-plcHost\",\"previousState\":\"$state\",\"currentState\":\"pending\",\"previousType\":\"c5.2xlarge\",\"result\":\"started\"},$ide]}" > "$last" ;;
  *'"action":"stop"'*) echo stopped > "$dir/state"
    echo "{\"ok\":true,\"action\":\"stop\",\"instances\":[{\"role\":\"plcHost\",\"instanceId\":\"i-plcHost\",\"previousState\":\"$state\",\"currentState\":\"stopping\"}]}" > "$last" ;;
  *'"action":"bringup"'*)
    if [ "$state" != "running" ]; then echo '{"ok":false,"action":"bringup","error":{"code":"not-ready","message":"not ready"}}' > "$last"
    elif [ -f "$dir/busy-bringup" ]; then echo '{"ok":false,"action":"bringup","error":{"code":"busy","message":"running"},"commandId":"0c0c0c0c-1111-2222-3333-444455556666"}' > "$last"
    else echo '{"ok":true,"action":"bringup","commandId":"0b5f9c1e-1111-2222-3333-444455556666","instanceIds":["i-kukaVc1","i-kukaVc2"]}' > "$last"; fi ;;
  *'"action":"bringupResult"'*)
    n=$(cat "$dir/polls" 2>/dev/null || echo 0); n=$((n+1)); echo $n > "$dir/polls"
    if [ $n -lt 2 ]; then printf '%s\n' '{"ok":true,"commandId":"x","status":"InProgress","done":false,"result":null,"stdoutTail":"== check\n"}' > "$last"
    elif [ -f "$dir/cut-bringup" ]; then printf '%s\n' '{"ok":true,"commandId":"x","status":"Success","done":true,"result":"ok","responseCode":0,"truncated":true,"stdoutTail":"res line 99\n"}' > "$last"
    elif [ -f "$dir/busy-result" ]; then printf '%s\n' '{"ok":true,"commandId":"x","status":"Failed","done":true,"result":"busy","responseCode":3,"stdoutTail":"BRINGUP_RESULT=busy\n"}' > "$last"
    elif [ -f "$dir/fail-bringup" ]; then printf '%s\n' '{"ok":true,"commandId":"x","status":"Failed","done":true,"result":"failed","stdoutTail":"== check\nR2 did not start\nBRINGUP_RESULT=failed\n"}' > "$last"
    else printf '%s\n' '{"ok":true,"commandId":"x","status":"Success","done":true,"result":"healthy","stdoutTail":"check passed\nBRINGUP_RESULT=healthy\n"}' > "$last"; fi ;;
  *'"action":"boom"'*) echo '{"errorMessage":"boom"}' > "$last"; echo '{"StatusCode":200,"FunctionError":"Unhandled"}'; exit 0 ;;
  *) echo '{"ok":false,"error":{"code":"unknown-action","message":"?"}}' > "$last" ;;
esac
echo '{"StatusCode":200,"ExecutedVersion":"$LATEST"}'
"#;

    /// `plan2cell cell start DIR ... --json`: refuses without the token in its environment; its
    /// arguments go to `plan2cell.args` (and a line to `start.calls`); with `guard` the head
    /// refuses the export (PLAN_ID: the file's text, else WORD#16#00AA, typed as an online value)
    /// unless DIR is the one in `guard-ok`. `plan2cell cell runtime ... --json` prints
    /// `runtime.json` (exit 4 with `runtime-fail`); its arguments go to `runtime.calls`.
    /// `plan2cell cell projects|deploy|krl DIR ... --json`: its arguments go to `stage.args`;
    /// `slow-<stage>` holds it that many seconds, `fail-<stage>` fails it (exit 8, a project
    /// error), `tool-fail-<stage>` fails it that many times more (exit 8, a CODESYS tool error);
    /// a successful `deploy` makes DIR the export the head's guard accepts (`guard-ok`).
    /// Every stage and start goes to `stage.calls` (`<stage> DIR`). `plan2cell cell reset-ide`
    /// goes to `reset.calls` (and `reset-ide` to `stage.calls`); `fail-reset-ide` fails it.
    const FAKE_PLAN2CELL: &str = r#"#!/bin/sh
dir=$(dirname "$0")
# A hook cell's stages (file hook-cell) get no CODESYS MCP token; every other cell's the token.
token_ok() { if [ -f "$dir/hook-cell" ]; then [ -z "${PLAN2CELL_CODESYS_MCP_TOKEN:-}" ]; else [ "$PLAN2CELL_CODESYS_MCP_TOKEN" = "tok-123" ]; fi; }
if [ "$1 $2" = "cell check" ]; then
  echo "$*" >> "$dir/check.calls"
  if [ -f "$dir/check-fail" ]; then cat "$dir/check-fail"; exit 7; fi
  echo '{"schema":"plan2cell/deploy-check@1","export":"'"$3"'","cell_id":"weld","plan_id":"16#0001","profile":"sim","ok":true,"fails":0,"warns":0,"rules":[{"id":"export.files","title":"Export complete","scope":"export","status":"pass","message":"ok"}]}'
  exit 0
fi
if [ "$1 $2" = "cell runtime" ]; then
  echo "$*" >> "$dir/runtime.calls"
  if [ -f "$dir/runtime-fail" ]; then
    echo '{"schema":"plan2cell/cell-runtime@1","ok":false,"runtimes":[],"error":"plc host: ssm i-0ebe: ConnectionLost"}'; exit 4
  fi
  cat "$dir/runtime.json"; exit 0
fi
if [ "$1 $2" = "cell reset-ide" ]; then
  echo "$*" >> "$dir/reset.calls"
  echo "reset-ide" >> "$dir/stage.calls"
  token_ok || { echo "no token" >&2; exit 9; }
  if [ -f "$dir/fail-reset-ide" ]; then
    echo '{"schema":"plan2cell/cell-report@1","ok":false,"stages":[{"stage":"reset-ide","ok":false,"notes":[],"error":"codesys tool get_codesys_status: not running"}],"projects":[]}'; exit 8
  fi
  echo '{"schema":"plan2cell/cell-report@1","ok":true,"stages":[{"stage":"reset-ide","ok":true,"notes":["CODESYS restarted","CODESYS IDE healthy"],"error":null}],"projects":[]}'
  exit 0
fi
echo "$2 $3" >> "$dir/stage.calls"
if [ "$2" = "stop" ]; then
  echo "$*" >> "$dir/stop.calls"
  token_ok || { echo "no token" >&2; exit 9; }
  echo '{"schema":"plan2cell/cell-report@1","ok":true,"stages":[{"stage":"stop","ok":true,"notes":["PLC_PRG.bStop pulsed","CS_ERROR ERR_OPERATOR_STOP robot 0, udiCycles 7"],"error":null}],"projects":[]}'
  exit 0
fi
case "$2" in projects|deploy|krl)
  echo "$*" >> "$dir/stage.args"
  token_ok || { echo "no token" >&2; exit 9; }
  echo "$2: working" >&2
  [ -f "$dir/slow-$2" ] && sleep "$(cat "$dir/slow-$2")"
  if [ -f "$dir/fail-$2" ]; then
    echo '{"schema":"plan2cell/cell-report@1","ok":false,"stages":[{"stage":"'"$2"'","ok":false,"notes":[],"error":"not built: HeadPLC (errors in the report)"}],"projects":[{"name":"HeadPLC","errors":["PLC_PRG(12): C0046: Identifier x not defined"]}]}'
    exit 8
  fi
  n=$(cat "$dir/tool-fail-$2" 2>/dev/null || echo 0)
  if [ "$n" -gt 0 ]; then
    echo $((n - 1)) > "$dir/tool-fail-$2"
    echo '{"schema":"plan2cell/cell-report@1","ok":false,"stages":[{"stage":"'"$2"'","ok":false,"notes":[],"error":"codesys tool get_project_structure: Error retrieving structure."}],"projects":[]}'
    exit 8
  fi
  note='"'"$2"' of HeadPLC ok"'
  if [ "$2" = "deploy" ]; then
    echo "$3" > "$dir/guard-ok"
    if [ -f "$dir/ide-refused" ]; then note="$note"',"HeadPLC not opened in the PLC IDE: Refusing to switch projects"'
    elif [ ! -f "$dir/ide-old" ]; then note="$note"',"HeadPLC open in the PLC IDE (C:/p/HeadPLC.project)"'; fi
  fi
  echo '{"schema":"plan2cell/cell-report@1","ok":true,"stages":[{"stage":"'"$2"'","ok":true,"notes":['"$note"'],"error":null}],"projects":[]}'
  exit 0;;
esac
echo "$*" > "$dir/plan2cell.args"
echo "$*" >> "$dir/start.calls"
token_ok || { echo "no token" >&2; exit 9; }
echo "start: pulsing" >&2
if [ -f "$dir/guard" ] && [ "$3" != "$(cat "$dir/guard-ok" 2>/dev/null)" ]; then
  head=$(cat "$dir/guard"); [ -n "$head" ] || head='WORD#16#00AA'
  echo '{"schema":"plan2cell/cell-report@1","ok":false,"stages":[{"stage":"start","ok":false,"notes":[],"error":"start: the head runs PLAN_ID '"$head"' (GVL_Cell.PLAN_ID), this export is plan 16#0001: start the export the head runs, or deploy this one first (cell projects, deploy, krl DIR)"}],"projects":[]}'
  exit 7
fi
# `cell start --json` prints the build's report merged over runs (plan2cell-cell-report.json):
# with `stale-report`, an earlier deploy and an earlier stop (CS_ERROR, after start in stage order).
stale_deploy='' stale_stop=''
if [ -f "$dir/stale-report" ]; then
  stale_deploy='{"stage":"deploy","ok":true,"notes":["HeadPLC running on 192.168.100.10"],"error":null},'
  stale_stop=',{"stage":"stop","ok":true,"notes":["PLC_PRG.bStop pulsed","CS_ERROR ERR_OPERATOR_STOP robot INT#0, udiCycles UDINT#10"],"error":null}'
fi
if [ -f "$dir/fail-start" ]; then
  echo '{"schema":"plan2cell/cell-report@1","ok":false,"stages":[{"stage":"start","ok":false,"notes":["restarted codesys-master (as asked)"],"error":"not in production after 16 pulse(s): CS_ERROR ERR_ROBOT robot 2"}'"$stale_stop"'],"projects":[]}'
  exit 7
fi
echo '{"schema":"plan2cell/cell-report@1","ok":true,"stages":['"$stale_deploy"'{"stage":"start","ok":true,"notes":["restarted codesys-master (as asked)","CS_ERROR: PLC_PRG.bReset pulsed","in production after 4 read(s) and 3 pulse(s): CS_RUNNING, udiCycles 0"],"error":null}'"$stale_stop"'],"projects":[{"name":"HeadPLC","state":"RUN"}]}'
"#;

    #[derive(Default)]
    struct Recorder {
        statuses: Mutex<Vec<InfraStatus>>,
        logs: Mutex<Vec<String>>,
    }

    impl Sink for Recorder {
        fn status(&self, status: &InfraStatus) {
            self.statuses.lock().unwrap().push(status.clone());
        }

        fn log(&self, line: String) {
            self.logs.lock().unwrap().push(line);
        }
    }

    struct Fixture {
        root: PathBuf,
        cell: PathBuf,
        service: Arc<InfraService>,
        recorder: Arc<Recorder>,
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.root).ok();
        }
    }

    fn script(path: &Path, text: &str) {
        use std::os::unix::fs::PermissionsExt as _;
        std::fs::write(path, text).unwrap();
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    fn fixture(name: &str, function: Option<&str>) -> Fixture {
        fixture_with(name, function, |_| {})
    }

    fn fixture_with(name: &str, function: Option<&str>, tweak: impl FnOnce(&mut Settings)) -> Fixture {
        let root = std::env::temp_dir().join(format!("factoryos-infra-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cells/weld");
        std::fs::create_dir_all(cell.join(cells::PLANS_DIR)).unwrap();
        script(&root.join("aws"), FAKE_AWS);
        script(&root.join("plan2cell"), FAKE_PLAN2CELL);
        let server_cell = root.join("etc-cell.toml");
        std::fs::write(&server_cell, "[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\nkli_ip = \"192.168.100.11\"\n").unwrap();
        let store = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let mut config = store.get();
        config.cell_dir = cell.display().to_string();
        store.set(config);
        let hardware = HardwareService::with(store.clone(), None, server_cell);
        let settings = Settings {
            function: function.map(str::to_owned),
            region: Some("eu-central-1".into()),
            aws: root.join("aws"),
            poll: Duration::from_millis(20),
            wait_timeout: Duration::from_secs(5),
            bringup_timeout: Duration::from_secs(5),
            production_timeout: Duration::from_secs(10),
            call_timeout: Duration::from_secs(10),
            ..Settings::default()
        };
        let mut settings = settings;
        tweak(&mut settings);
        let recorder = Arc::new(Recorder::default());
        let service = InfraService::with(recorder.clone(), settings, store, hardware, Some(root.join("plan2cell")));
        Fixture { root, cell, service, recorder }
    }

    async fn settled(service: &InfraService) -> InfraStatus {
        for _ in 0..400 {
            if service.inner.lock().unwrap().job.is_none() {
                return service.status();
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!("the job did not finish: {:#?}", service.status());
    }

    /// Until Start cell's chain (wait, bring-up, production) has finished.
    async fn start_settled(service: &InfraService) -> InfraStatus {
        for _ in 0..400 {
            let status = service.status();
            if !status.start.state.busy() && service.inner.lock().unwrap().job.is_none() {
                return status;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!("Start cell did not finish: {:#?}", service.status().start);
    }

    fn export(f: &Fixture) {
        let build = f.cell.join("plan2cell/builds/bt_a/20261012T080000000Z");
        std::fs::create_dir_all(&build).unwrap();
        std::fs::write(build.join("build.json"), r#"{"ok":true}"#).unwrap();
    }

    #[tokio::test]
    async fn start_cell_brings_up_and_produces_in_one_click() {
        let f = fixture("startcell", Some("plan2cell-cell-control"));
        export(&f);
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        // In production when Stop cell runs: remembered for the next Start cell.
        f.service.change_auto(&f.cell, |a| a.in_production = true).unwrap();
        let status = f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        assert!(status.stopped_in_production && !status.in_production, "{status:#?}");
        // A second Stop cell (the PLC host stopped already) keeps it.
        let status = f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        assert!(status.stopped_in_production);

        // The IDE's type has no capacity: it starts as a fallback; the instances boot for two reads.
        std::fs::write(f.root.join("start-fallback"), "").unwrap();
        std::fs::write(f.root.join("booting"), "2").unwrap();
        let accepted = f.service.start_stop("start", StartStopRequest::default()).await.unwrap();
        assert!(accepted.start.bring_up && accepted.start.production, "{:#?}", accepted.start);
        assert_eq!(accepted.start.instances[1].new_type.as_deref(), Some("c5.4xlarge"));
        assert_eq!(accepted.start.instances[1].summary(), "Windows IDE: c5a.4xlarge had no capacity, started as c5.4xlarge");
        assert!(!accepted.stopped_in_production, "used up by this start");
        // No other job while it waits.
        let err = f.service.production(ProductionRequest::default()).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        let status = start_settled(&f.service).await;
        assert_eq!(status.start.state, JobState::Done, "{:#?}", status.start);
        assert!(status.start.message.as_deref().unwrap().contains("in production"), "{:?}", status.start.message);
        assert_eq!(status.bringup.state, JobState::Done);
        assert_eq!(status.cell.state, JobState::Done);
        assert!(status.in_production);
        let messages: Vec<String> =
            f.recorder.statuses.lock().unwrap().iter().filter_map(|s| s.start.message.clone()).collect();
        assert!(messages.iter().any(|m| m.starts_with("Waiting for the instances (0 of 4 ready")), "{messages:#?}");
        assert!(messages.iter().any(|m| m.contains("Windows IDE pending")), "per-instance progress: {messages:#?}");
        let logs = f.recorder.logs.lock().unwrap().join("\n");
        assert!(logs.contains("start: Windows IDE: c5a.4xlarge had no capacity, started as c5.4xlarge"), "{logs}");
        std::fs::remove_file(f.root.join("start-fallback")).unwrap();

        // In production again, but this start says production: false: bring-up only.
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        let starts = std::fs::read_to_string(f.root.join("start.calls")).unwrap().lines().count();
        let accepted = f.service.start_stop("start", StartStopRequest { production: Some(false), ..Default::default() }).await.unwrap();
        assert!(accepted.start.bring_up && !accepted.start.production);
        let status = start_settled(&f.service).await;
        assert_eq!(status.start.state, JobState::Done, "{:#?}", status.start);
        assert!(status.start.message.as_deref().unwrap().contains("controllers brought up"), "{:?}", status.start.message);
        assert_eq!(std::fs::read_to_string(f.root.join("start.calls")).unwrap().lines().count(), starts, "no production start");

        // Not in production at the stop: Start cell brings up only; `production: true` starts it anyway.
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        let accepted = f.service.start_stop("start", StartStopRequest::default()).await.unwrap();
        assert!(!accepted.start.production);
        start_settled(&f.service).await;
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        let accepted = f.service.start_stop("start", StartStopRequest { production: Some(true), ..Default::default() }).await.unwrap();
        assert!(accepted.start.production);
        let status = start_settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done);
        assert!(std::fs::read_to_string(f.root.join("start.calls")).unwrap().lines().count() > starts);
    }

    #[tokio::test]
    async fn start_cell_reports_an_instance_that_did_not_start_and_stop_ends_the_wait() {
        let f = fixture("startfail", Some("plan2cell-cell-control"));
        std::fs::write(f.root.join("start-fail-ide"), "").unwrap();
        let status = f.service.start_stop("start", StartStopRequest::default()).await.unwrap();
        assert_eq!(status.start.state, JobState::Failed, "{:#?}", status.start);
        let message = status.start.message.as_deref().unwrap();
        assert!(message.contains("Windows IDE: did not start (c5a.4xlarge): InsufficientInstanceCapacity"), "{message}");
        assert!(message.contains("not bringing up"), "{message}");
        assert!(status.start.instances[0].result == "started", "the others started anyway");
        assert!(f.service.inner.lock().unwrap().job.is_none());
        std::fs::remove_file(f.root.join("start-fail-ide")).unwrap();
        // Stop cell while Start cell waits for the instances.
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        std::fs::write(f.root.join("booting"), "100000").unwrap();
        f.service.start_stop("start", StartStopRequest::default()).await.unwrap();
        tokio::time::sleep(Duration::from_millis(60)).await;
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        let status = start_settled(&f.service).await;
        assert_eq!(status.start.state, JobState::Failed);
        assert!(status.start.message.as_deref().unwrap().contains("Stop cell was asked for"), "{:?}", status.start.message);
        assert_eq!(status.bringup.state, JobState::Idle, "no bring-up");
        // A start of some roles: no chain unless asked for.
        let status = f.service.start_stop("start", StartStopRequest { roles: Some(vec!["plcHost".into()]), ..Default::default() }).await.unwrap();
        assert!(!status.start.bring_up && status.start.state == JobState::Done, "{:#?}", status.start);
    }

    #[tokio::test]
    async fn without_a_function_it_is_unavailable() {
        let f = fixture("none", None);
        let status = f.service.current().await;
        assert!(!status.available);
        assert!(status.message.unwrap().contains("FACTORY_OS_CELL_CONTROL_FUNCTION"));
        assert!(status.cell.demo_note.is_some());
        assert_eq!(f.service.start_stop("start", StartStopRequest::default()).await.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        assert_eq!(f.service.bringup(BringupRequest::default()).await.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        assert_eq!(f.service.production(ProductionRequest::default()).await.unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        assert!(!f.root.join("aws.calls").exists(), "nothing invoked");
        let bad = fixture("bad", Some("x; rm -rf /"));
        assert!(!bad.service.status().available);
    }

    #[tokio::test]
    async fn bring_up_follows_a_running_one_and_reads_the_exit_code() {
        let f = fixture("follow", Some("plan2cell-cell-control"));
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        // Another bring-up still runs on the VC host (e.g. from before a backend restart): followed, not doubled.
        std::fs::write(f.root.join("busy-bringup"), "").unwrap();
        f.service.bringup(BringupRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.bringup.state, JobState::Done, "{:#?}", status.bringup);
        assert_eq!(status.bringup.command_id.as_deref(), Some("0c0c0c0c-1111-2222-3333-444455556666"));
        std::fs::remove_file(f.root.join("busy-bringup")).unwrap();
        // Exit 0, but the result line was past SSM's 24,000 characters: done, not failed.
        std::fs::write(f.root.join("cut-bringup"), "").unwrap();
        std::fs::remove_file(f.root.join("polls")).ok();
        f.service.bringup(BringupRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.bringup.state, JobState::Done, "{:#?}", status.bringup);
        assert!(status.bringup.message.as_deref().unwrap().contains("exit 0"), "{:?}", status.bringup.message);
        std::fs::remove_file(f.root.join("cut-bringup")).unwrap();
        // A kvc-cell-setup.sh someone else runs: busy, not "check failed".
        std::fs::write(f.root.join("busy-result"), "").unwrap();
        std::fs::remove_file(f.root.join("polls")).ok();
        f.service.bringup(BringupRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.bringup.state, JobState::Failed);
        assert!(status.bringup.message.as_deref().unwrap().contains("busy"), "{:?}", status.bringup.message);
    }

    #[tokio::test]
    async fn start_bring_up_produce_and_stop() {
        let f = fixture("cycle", Some("plan2cell-cell-control"));
        let status = f.service.current().await;
        assert!(status.available && !status.ready, "{status:#?}");
        // One KUKA VC host per controller.
        assert_eq!(status.instances.len(), 4);
        assert_eq!(status.instances[3].role, "kukaVc2");
        assert_eq!(status.instances[3].robot_ns.as_deref(), Some("robot2"));
        assert!(status.dcv_url.is_none());
        assert_eq!(f.service.bringup(BringupRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT, "stopped: start first");
        let err = f.service.start_stop("start", StartStopRequest { roles: Some(vec!["db".into()]), ..Default::default() }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::BAD_REQUEST);
        // A single KUKA VC host by its role: passed through to the Lambda.
        f.service.start_stop("stop", StartStopRequest { roles: Some(vec!["kukaVc2".into()]), ..Default::default() }).await.unwrap();
        let calls = std::fs::read_to_string(f.root.join("aws.calls")).unwrap();
        assert!(calls.contains(r#""roles":["kukaVc2"]"#), "{calls}");

        let status = f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        assert!(status.ready && status.all_running);
        assert_eq!(status.dcv_url.as_deref(), Some("https://ec2-1-2-3-4.compute.amazonaws.com:8443/"));
        assert!(status.cell.demo_note.as_deref().unwrap().contains("08:00 UTC"), "{:?}", status.cell.demo_note);
        let calls = std::fs::read_to_string(f.root.join("aws.calls")).unwrap();
        assert!(calls.contains("lambda invoke --function-name plan2cell-cell-control --cli-binary-format raw-in-base64-out"), "{calls}");
        assert!(calls.contains("--region eu-central-1"));

        // Bring-up: done on the second poll.
        let accepted = f.service.bringup(BringupRequest { restart_controllers: true }).await.unwrap();
        assert!(accepted.bringup.state.busy());
        let err = f.service.production(ProductionRequest::default()).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT, "one job at a time, or no build: {}", err.1);
        let status = settled(&f.service).await;
        assert_eq!(status.bringup.state, JobState::Done, "{:#?}", status.bringup);
        assert_eq!(status.bringup.result.as_deref(), Some("healthy"));
        assert_eq!(status.bringup.command_id.as_deref(), Some("0b5f9c1e-1111-2222-3333-444455556666"));
        assert!(std::fs::read_to_string(f.root.join("aws.calls")).unwrap().contains("\"restartControllers\":true"));

        // Production: no export yet.
        let err = f.service.production(ProductionRequest::default()).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        assert!(err.1.contains("Export"), "{}", err.1);
        let build = f.cell.join("plan2cell/builds/bt_a/20261012T080000000Z");
        std::fs::create_dir_all(&build).unwrap();
        std::fs::write(build.join("build.json"), r#"{"ok":true}"#).unwrap();
        let older = f.cell.join("plan2cell/builds/bt_b/20261011T080000000Z");
        std::fs::create_dir_all(&older).unwrap();
        std::fs::write(older.join("build.json"), r#"{"ok":true}"#).unwrap();
        let failed = f.cell.join("plan2cell/builds/bt_b/20261013T080000000Z");
        std::fs::create_dir_all(&failed).unwrap();
        std::fs::write(failed.join("build.json"), r#"{"ok":false}"#).unwrap();

        let accepted = f.service.production(ProductionRequest::default()).await.unwrap();
        assert_eq!(accepted.cell.state, JobState::Running);
        assert_eq!(accepted.cell.build.as_deref(), Some("20261012T080000000Z-bt_a"), "the newest successful build");
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_RUNNING"));
        assert!(status.cell.message.as_deref().unwrap().contains("in production"));
        assert!(status.cell.runtime_started_s.is_some());
        assert!(status.cell.demo_note.as_deref().unwrap().contains("restarted at"), "{:?}", status.cell.demo_note);
        let args = std::fs::read_to_string(f.root.join("plan2cell.args")).unwrap();
        assert!(args.starts_with(&format!("cell start {} --cell-config {}", build.display(), f.root.join("etc-cell.toml").display())), "{args}");
        assert!(args.contains("--restart-runtime --pulses 16 --json"), "{args}");
        let logs = f.recorder.logs.lock().unwrap().join("\n");
        assert!(logs.contains("start: pulsing"), "the CLI's stderr goes to the log");
        assert!(!logs.contains("tok-123"), "the token is never logged");
        assert!(!serde_json::to_string(&*f.recorder.statuses.lock().unwrap()).unwrap().contains("tok-123"));

        // The start's report is the build's, merged over runs: an earlier stop's CS_ERROR (after
        // the start stage in stage order) is not the head's state now.
        std::fs::write(f.root.join("stale-report"), "").unwrap();
        f.service.stop_production().await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_ERROR"), "{:#?}", status.cell);
        f.service.production(ProductionRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_RUNNING"), "the start's own state, not the earlier stop's");
        assert!(status.cell.message.as_deref().unwrap().starts_with("in production after 4 read(s)"), "{:?}", status.cell.message);

        // A failed start keeps what it read (an earlier stop's notes are not a read).
        std::fs::write(f.root.join("fail-start"), "").unwrap();
        f.service.production(ProductionRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Failed);
        assert!(status.cell.message.as_deref().unwrap().contains("CS_ERROR ERR_ROBOT robot 2"), "{:?}", status.cell.message);
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_RUNNING"), "no new state read: the last one stays");
        std::fs::remove_file(f.root.join("stale-report")).unwrap();

        // A failed bring-up.
        std::fs::write(f.root.join("fail-bringup"), "").unwrap();
        std::fs::remove_file(f.root.join("polls")).ok();
        f.service.bringup(BringupRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.bringup.state, JobState::Failed);
        assert!(status.bringup.message.as_deref().unwrap().contains("R2 did not start"), "{:?}", status.bringup.message);

        let status = f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        assert!(status.all_stopped());
        assert!(status.dcv_url.is_none());
        assert_eq!(f.service.production(ProductionRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT, "not ready");
        assert!(f.service.inner.lock().unwrap().job.is_none(), "a refused job frees the cell");
    }

    #[tokio::test]
    async fn function_errors_and_polling() {
        let f = fixture("errors", Some("plan2cell-cell-control"));
        let err = f.service.invoke(json!({"action": "boom"})).await.unwrap_err();
        assert!(format!("{err:#}").contains("Unhandled"), "{err:#}");
        let body = f.service.invoke(json!({"action": "nope"})).await.unwrap();
        assert_eq!(lambda_error(&body), "? (unknown-action)");
        // The viewer asked: the poller keeps reading.
        f.service.spawn_poller();
        f.service.current().await;
        let before = f.recorder.statuses.lock().unwrap().len();
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!(f.recorder.statuses.lock().unwrap().len() > before, "polled while a viewer is interested");
        f.service.inner.lock().unwrap().interest = None;
        // A refresh already under way (a fake `aws` process, slow on a loaded machine) still records: settle
        // until the count holds before asserting that polling stopped.
        let mut quiet = f.recorder.statuses.lock().unwrap().len();
        for _ in 0..20 {
            tokio::time::sleep(Duration::from_millis(100)).await;
            let now = f.recorder.statuses.lock().unwrap().len();
            if now == quiet {
                break;
            }
            quiet = now;
        }
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert_eq!(f.recorder.statuses.lock().unwrap().len(), quiet, "and stops when nobody is");
    }

    /// `runtime.json` as `plan2cell cell runtime --json` prints it: the head (docker restarted it
    /// `restarts` times by its policy) and one simulator.
    fn runtime_json(root: &Path, head_running: bool, head_left: i64, sim_left: i64, restarts: u64) {
        let now = unix_s(SystemTime::now()) as i64;
        let head = if head_running {
            json!({"container": "codesys-master", "head": true, "exists": true, "running": true, "startedS": now - 7200 + head_left,
                   "restartCount": restarts, "demo": true, "stopsS": now + head_left, "secondsLeft": head_left,
                   "source": "shutdown warning", "warning": "!!!! Warning: WARNING: performing shutdown in 10 minutes"})
        } else {
            json!({"container": "codesys-master", "head": true, "exists": true, "running": false, "state": "exited", "restartCount": restarts})
        };
        let report = json!({"schema": "plan2cell/cell-runtime@1", "ok": true, "nowS": now, "runtimes": [head,
            {"container": "codesys-device-robot1", "head": false, "exists": true, "running": true, "stopsS": now + sim_left,
             "secondsLeft": sim_left, "source": "demo line"}]});
        std::fs::write(root.join("runtime.json"), report.to_string()).unwrap();
    }

    fn calls(f: &Fixture, file: &str) -> Vec<String> {
        std::fs::read_to_string(f.root.join(file)).unwrap_or_default().lines().map(str::to_owned).collect()
    }

    /// Read the runtimes now (and let auto-restart decide), then wait for a job it started.
    async fn tick(f: &Fixture) -> InfraStatus {
        f.service.inner.lock().unwrap().runtime_checked = None;
        f.service.runtime_tick().await;
        settled(&f.service).await
    }

    fn build(f: &Fixture, plan: &str, stamp: &str, plan_id: &str) -> PathBuf {
        let dir = f.cell.join(format!("plan2cell/builds/{plan}/{stamp}"));
        std::fs::create_dir_all(dir.join("plc")).unwrap();
        std::fs::write(dir.join("build.json"), json!({"ok": true, "summary": {"planId": plan_id}}).to_string()).unwrap();
        std::fs::write(dir.join("plc/project.json"), "{}").unwrap();
        dir
    }

    /// The last attempt and the last production job 11 min earlier (past the backoff).
    fn back(f: &Fixture) {
        f.service
            .change_auto(&f.cell, |a| {
                a.status.last_attempt_s = a.status.last_attempt_s.map(|t| t - 660.0);
                a.last_job_s = a.last_job_s.map(|t| t - 660.0);
            })
            .unwrap();
    }

    #[tokio::test]
    async fn runtimes_are_read_and_auto_restart_keeps_the_demo_runtime_going() {
        let f = fixture("auto", Some("plan2cell-cell-control"));
        let newest = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        let deployed = build(&f, "bt_b", "20261011T080000000Z", "16#00AA");
        let config = f.root.join("etc-cell.toml");
        runtime_json(&f.root, true, 3000, 5000, 0);

        // Stopped cell: the runtimes are not read.
        f.service.current().await;
        let status = tick(&f).await;
        assert!(calls(&f, "runtime.calls").is_empty());
        let rt = status.plc_runtime.expect("a status line");
        assert!(!rt.running && rt.error.as_deref().unwrap().contains("PLC host is stopped"), "{rt:?}");

        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        // A viewer asks, auto-restart is off: read (the newest export's containers), nothing started.
        f.service.current().await;
        let status = tick(&f).await;
        let rt = status.plc_runtime.clone().unwrap();
        assert!(rt.running && rt.error.is_none(), "{rt:?}");
        assert_eq!(rt.container.as_deref(), Some("codesys-master"), "the runtime that stops first");
        assert!((rt.seconds_left.unwrap() - 3000.0).abs() < 1.0);
        assert_eq!(rt.source.as_deref(), Some("shutdown warning"));
        assert_eq!(rt.runtimes.len(), 2);
        assert_eq!(rt.build.as_deref(), Some("20261012T080000000Z-bt_a"));
        assert!(status.cell.demo_note.as_deref().unwrap().contains("CODESYS demo runtime: stops at"), "{:?}", status.cell.demo_note);
        assert_eq!(calls(&f, "runtime.calls"), [format!("cell runtime {} --cell-config {} --json", newest.display(), config.display())]);
        // Not again within the poll interval.
        f.service.runtime_tick().await;
        assert_eq!(calls(&f, "runtime.calls").len(), 1);
        assert!(calls(&f, "start.calls").is_empty());
        assert!(!status.auto_restart.enabled && status.auto_restart.message.is_none());

        // On, 2 min left, but the cell was never put into production: left alone.
        let auto = f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        assert!(auto.enabled);
        assert_eq!(auto.threshold_s, AUTO_RESTART_THRESHOLD_S);
        assert!(f.cell.join(AUTO_RESTART_FILE).is_file());
        assert_eq!(f.service.auto_restart(), auto, "GET reads the file");
        f.service.inner.lock().unwrap().interest = None;
        runtime_json(&f.root, true, 120, 5000, 0);
        tick(&f).await;
        assert_eq!(calls(&f, "runtime.calls").len(), 2, "auto-restart on: read without a viewer");
        assert!(calls(&f, "start.calls").is_empty(), "not in production: no production start");

        // Start production by hand; right after it, auto-restart waits 10 min.
        f.service.production(ProductionRequest::default()).await.unwrap();
        settled(&f.service).await;
        assert_eq!(calls(&f, "start.calls").len(), 1);
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 1, "within 10 min of a production job");
        assert_eq!(load_auto(&f.cell).head_restart_count, Some(0), "taken anew after the job");

        // 11 min later, 2 min left: production with --restart-runtime on the export in production.
        back(&f);
        let status = tick(&f).await;
        let starts = calls(&f, "start.calls");
        assert_eq!(starts.len(), 2, "{starts:?}");
        assert!(starts[1].starts_with(&format!("cell start {} ", newest.display())), "{starts:?}");
        assert!(starts[1].contains("--restart-runtime --pulses 16 --json"));
        let auto = &status.auto_restart;
        assert_eq!(auto.last_result.as_deref(), Some("CS_RUNNING"), "{auto:?}");
        assert!(auto.message.as_deref().unwrap().starts_with("auto-restart at ") && auto.message.as_deref().unwrap().ends_with(": CS_RUNNING"));
        assert_eq!(auto.deployed_plan_id.as_deref(), Some("16#0001"));
        assert!(auto.next_allowed_s.unwrap() > unix_s(SystemTime::now()) + 500.0);
        assert_eq!(status.cell.state, JobState::Done);
        let logs = f.recorder.logs.lock().unwrap().join("\n");
        assert!(logs.contains("[infra] auto-restart: codesys-master stops at"), "{logs}");
        assert!(logs.contains("[infra] auto-restart: auto-restart at"), "{logs}");

        // The runtime still reads 2 min left (say the restart did not take): one attempt per 10 min.
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 2, "backoff");

        // Someone restarts the runtimes outside the backend (CLI, docker restart): a newer start,
        // the same restart count: not a restart of its own, nothing done.
        runtime_json(&f.root, true, 7000, 7000, 0);
        back(&f);
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 2, "an out-of-band restart is left alone");
        // docker restarted the head by its restart policy (the runtime stopped by itself).
        runtime_json(&f.root, true, 7100, 7000, 1);
        let status = tick(&f).await;
        let starts = calls(&f, "start.calls");
        assert_eq!(starts.len(), 3, "{starts:?}");
        assert!(f.recorder.logs.lock().unwrap().join("\n").contains("restarted the head's CODESYS runtime by itself"));
        assert_eq!(status.auto_restart.last_result.as_deref(), Some("CS_RUNNING"));
        // The next read takes the count as it is: no loop.
        back(&f);
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 3);
        assert_eq!(load_auto(&f.cell).head_restart_count, Some(1));

        // A start that fails after restarting the runtimes: no "restarted on its own" loop; 3
        // failed attempts in a row hold it.
        std::fs::write(f.root.join("fail-start"), "").unwrap();
        runtime_json(&f.root, true, 120, 5000, 1);
        for n in 1..=3 {
            back(&f);
            let status = tick(&f).await;
            assert_eq!(calls(&f, "start.calls").len(), 3 + n, "attempt {n}");
            assert_eq!(status.auto_restart.failures, n as u32);
            assert_eq!(status.auto_restart.blocked.is_some(), n == 3, "{:?}", status.auto_restart);
        }
        back(&f);
        let status = tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 6, "held after 3 failures");
        assert!(status.auto_restart.message.as_deref().unwrap().contains("3 attempts in a row failed"), "{:?}", status.auto_restart);
        std::fs::remove_file(f.root.join("fail-start")).unwrap();
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        assert_eq!(load_auto(&f.cell).status.failures, 0);

        // The head's runtime is down while in production; the head refuses the export (PLAN_ID
        // guard, its typed online value, decimal) and no export has its PLAN_ID: reported, held,
        // no loop.
        back(&f);
        runtime_json(&f.root, false, 0, 5000, 1);
        std::fs::write(f.root.join("guard"), "WORD#3054").unwrap();
        let status = tick(&f).await;
        assert!(!status.plc_runtime.as_ref().unwrap().running);
        assert_eq!(calls(&f, "start.calls").len(), 7, "no export to retry with");
        let auto = &status.auto_restart;
        assert!(auto.blocked.as_deref().unwrap().contains("PLAN_ID guard"), "{auto:?}");
        assert!(auto.last_error.as_deref().unwrap().starts_with("head runs 3054; no export of it: deploy one"), "{auto:?}");
        assert!(auto.last_error.as_deref().unwrap().contains("the head runs PLAN_ID WORD#3054"), "{auto:?}");
        assert!(auto.message.as_deref().unwrap().starts_with("auto-restart holds"), "{auto:?}");
        assert_eq!(auto.deployed_plan_id.as_deref(), Some("3054"), "the plan the head named, without its type");
        assert_eq!(status.cell.state, JobState::Failed);
        back(&f);
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 7, "held: no loop");

        // On again, the file still names the plan of the newest export, but the head runs
        // 16#00AA: refused there, then once more on the newest export of 16#00AA (the older one),
        // without --restart-runtime.
        std::fs::write(f.root.join("guard"), "").unwrap();
        std::fs::write(f.root.join("guard-ok"), deployed.display().to_string()).unwrap();
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = Some("16#0001".into())).unwrap();
        let auto = f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: Some(600) }).unwrap();
        assert!(auto.blocked.is_none());
        assert_eq!(auto.threshold_s, 600);
        back(&f);
        let status = tick(&f).await;
        let starts = calls(&f, "start.calls");
        assert_eq!(starts.len(), 9, "{starts:?}");
        assert!(starts[7].starts_with(&format!("cell start {} ", newest.display())) && starts[7].contains("--restart-runtime"), "{starts:?}");
        assert!(starts[8].starts_with(&format!("cell start {} ", deployed.display())), "{starts:?}");
        assert!(!starts[8].contains("--restart-runtime"), "the runtimes are not restarted again: {starts:?}");
        let auto = &status.auto_restart;
        assert_eq!(auto.last_result.as_deref(), Some("CS_RUNNING"), "{auto:?}");
        assert!(auto.blocked.is_none() && auto.failures == 0, "{auto:?}");
        assert_eq!(auto.deployed_plan_id.as_deref(), Some("16#00AA"));
        assert_eq!(status.cell.build.as_deref(), Some("20261011T080000000Z-bt_b"));
        std::fs::remove_file(f.root.join("guard")).unwrap();
        std::fs::remove_file(f.root.join("guard-ok")).unwrap();
        // From now on the runtimes of that export are read.
        back(&f);
        runtime_json(&f.root, true, 7000, 7000, 1);
        tick(&f).await;
        let runtime_calls = calls(&f, "runtime.calls");
        assert!(runtime_calls.last().unwrap().starts_with(&format!("cell runtime {} ", deployed.display())), "{runtime_calls:?}");

        // Restart PLC runtime and Start production by hand: the same export; the newest only by
        // choice (`build`).
        f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap();
        settled(&f.service).await;
        assert!(calls(&f, "start.calls")[9].starts_with(&format!("cell start {} ", deployed.display())));
        f.service.production(ProductionRequest::default()).await.unwrap();
        settled(&f.service).await;
        assert!(calls(&f, "start.calls")[10].starts_with(&format!("cell start {} ", deployed.display())));
        let explicit = ProductionRequest { build: Some("20261012T080000000Z-bt_a".into()), ..Default::default() };
        f.service.production(explicit).await.unwrap();
        settled(&f.service).await;
        assert!(calls(&f, "start.calls")[11].starts_with(&format!("cell start {} ", newest.display())));
        // Back to the export of the PLAN_ID the head runs.
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = Some("16#00AA".into())).unwrap();
        f.service.production(ProductionRequest::default()).await.unwrap();
        settled(&f.service).await;
        assert!(calls(&f, "start.calls")[12].starts_with(&format!("cell start {} ", deployed.display())));
        // A PLAN_ID without an export: refused before anything runs.
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = Some("16#0BAD".into())).unwrap();
        let err = f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        assert!(err.1.contains("PLAN_ID 16#0BAD"), "{}", err.1);
        assert!(f.service.inner.lock().unwrap().job.is_none());

        // An unreadable PLC host: the error in the status and in auto-restart's line, no restart.
        std::fs::write(f.root.join("runtime-fail"), "").unwrap();
        back(&f);
        let status = tick(&f).await;
        let rt = status.plc_runtime.unwrap();
        assert!(rt.error.as_deref().unwrap().contains("ConnectionLost"), "{rt:?}");
        assert_eq!(calls(&f, "start.calls").len(), 13);
        let auto = &status.auto_restart;
        assert!(auto.inactive.is_some(), "{auto:?}");
        assert!(auto.message.as_deref().unwrap().starts_with("auto-restart inactive: cannot read the CODESYS runtimes"), "{auto:?}");

        // Stop cell: no longer meant to be in production.
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        assert!(!load_auto(&f.cell).in_production);
        let off = f.service.set_auto_restart(AutoRestartSettings { enabled: false, threshold_s: None }).unwrap();
        assert!(!off.enabled);
        assert!(f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: Some(7200) }).is_err());
    }

    /// A cell report (`plan2cell cell` stages) in `dir`, written at `at` (Unix seconds).
    fn cell_report(dir: &Path, deploy_ok: bool, at: u64) {
        let report = json!({"schema": "plan2cell/cell-report@1", "ok": deploy_ok, "stages": [
            {"stage": "projects", "ok": true, "notes": [], "error": null},
            {"stage": "deploy", "ok": deploy_ok, "notes": [], "error": if deploy_ok { Value::Null } else { json!("download failed") }}], "projects": []});
        let path = dir.join(codegen::CELL_REPORT);
        std::fs::write(&path, report.to_string()).unwrap();
        let file = std::fs::File::options().write(true).open(&path).unwrap();
        file.set_modified(SystemTime::UNIX_EPOCH + Duration::from_secs(at)).unwrap();
    }

    #[tokio::test]
    async fn restart_plc_runtime_finds_the_export_the_head_runs() {
        let f = fixture("match", Some("plan2cell-cell-control"));
        // The incident: the head runs 16#9201 (WORD#37377), the newest export is another plan.
        let head = build(&f, "bt_a", "20261012T075502149Z", "16#9201");
        let newest = build(&f, "bt_a", "20261012T131019320Z", "16#4195");
        let older = build(&f, "bt_c", "20261011T080000000Z", "16#0003");
        let failed_deploy = build(&f, "bt_d", "20261010T080000000Z", "16#0004");
        cell_report(&older, true, 1_000);
        cell_report(&head, true, 2_000);
        cell_report(&failed_deploy, false, 3_000);
        cell_report(&newest, true, 500);
        assert_eq!(codegen::last_deployed_build(&f.cell).map(|(id, _)| id).as_deref(), Some("20261012T075502149Z-bt_a"), "the newest report");
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();

        // PLAN_ID unknown: the export last deployed (by its report), not the newest.
        assert!(load_auto(&f.cell).status.deployed_plan_id.is_none());
        f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        let starts = calls(&f, "start.calls");
        assert!(starts[0].starts_with(&format!("cell start {} ", head.display())) && starts[0].contains("--restart-runtime"), "{starts:?}");
        // Kept in the auto-restart file (a backend restart reads it from there).
        assert_eq!(status.auto_restart.deployed_plan_id.as_deref(), Some("16#9201"));
        let saved: Value = serde_json::from_slice(&std::fs::read(f.cell.join(AUTO_RESTART_FILE)).unwrap()).unwrap();
        assert_eq!(saved["deployedPlanId"], "16#9201", "{saved}");

        // No report and no PLAN_ID known: the newest; the head's guard names WORD#37377: once
        // more on the newest export of 16#9201, without restarting the runtimes again.
        for dir in [&head, &newest, &older, &failed_deploy] {
            std::fs::remove_file(dir.join(codegen::CELL_REPORT)).unwrap();
        }
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = None).unwrap();
        std::fs::write(f.root.join("guard"), "WORD#37377").unwrap();
        std::fs::write(f.root.join("guard-ok"), head.display().to_string()).unwrap();
        f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        let starts = calls(&f, "start.calls");
        assert_eq!(starts.len(), 3, "{starts:?}");
        assert!(starts[1].starts_with(&format!("cell start {} ", newest.display())) && starts[1].contains("--restart-runtime"), "{starts:?}");
        assert!(starts[2].starts_with(&format!("cell start {} ", head.display())) && !starts[2].contains("--restart-runtime"), "{starts:?}");
        assert_eq!(status.cell.build.as_deref(), Some("20261012T075502149Z-bt_a"));
        assert_eq!(status.auto_restart.deployed_plan_id.as_deref(), Some("16#9201"), "the export's PLAN_ID");
        assert!(f.recorder.logs.lock().unwrap().join("\n").contains("the head runs PLAN_ID 37377"));

        // The head runs a plan without an export: one start, a clear error, no loop.
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = None).unwrap();
        std::fs::write(f.root.join("guard"), "UINT#16#0BEE").unwrap();
        f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Failed);
        assert_eq!(calls(&f, "start.calls").len(), 4);
        let message = status.cell.message.as_deref().unwrap();
        assert!(message.starts_with("head runs 16#0BEE; no export of it: deploy one (Run Control Code): plan2cell cell start failed (exit 7)"), "{message}");
        assert_eq!(status.auto_restart.deployed_plan_id.as_deref(), Some("16#0BEE"));
        // Known now: Restart PLC runtime refuses before anything runs.
        let err = f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        assert_eq!(calls(&f, "start.calls").len(), 4);

        // Start production on the newest, by choice: a refusal is not retried with another plan.
        std::fs::write(f.root.join("guard"), "WORD#37377").unwrap();
        f.service.production(ProductionRequest { build: Some("20261012T131019320Z-bt_a".into()), ..Default::default() }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Failed);
        assert_eq!(calls(&f, "start.calls").len(), 5);
        assert_eq!(status.auto_restart.deployed_plan_id.as_deref(), Some("37377"));
    }

    #[tokio::test]
    async fn run_control_code_deploys_an_export_and_starts_it() {
        let f = fixture("deploy", Some("plan2cell-cell-control"));
        // The incident: the head runs 16#9201, the user has just exported 16#8E35.
        let head = build(&f, "bt_a", "20261012T075502149Z", "16#9201");
        let newest = build(&f, "bt_a", "20261013T090000000Z", "16#8E35");
        cell_report(&head, true, 2_000);
        std::fs::write(f.root.join("guard"), "WORD#37377").unwrap();
        std::fs::write(f.root.join("guard-ok"), head.display().to_string()).unwrap();
        let config = f.root.join("etc-cell.toml");
        assert_eq!(f.service.deploy(DeployRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT, "not ready");
        assert!(f.service.inner.lock().unwrap().job.is_none());
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();

        // What the viewer shows: Start production takes the export the head runs, the newest differs.
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        let status = f.service.status();
        assert_eq!(status.head_export.as_ref().map(|e| (e.build.as_str(), e.source.as_str())), Some(("20261012T075502149Z-bt_a", "report")));
        assert_eq!(status.head_plan_id.as_deref(), Some("16#9201"));
        let newest_ref = status.newest_export.clone().unwrap();
        assert_eq!((newest_ref.build.as_str(), newest_ref.plan_id.as_deref()), ("20261013T090000000Z-bt_a", Some("16#8E35")));

        // Start production (no matchDeployed): the export the head runs, not the newest; no refusal.
        f.service.production(ProductionRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        assert_eq!(calls(&f, "stage.calls"), [format!("start {}", head.display())]);

        // Unknown or unusable exports: refused before anything runs.
        let err = f.service.deploy(DeployRequest { build: Some("20261013T090000000Z-nope".into()), ..Default::default() }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::NOT_FOUND, "{}", err.1);
        let no_plc = f.cell.join("plan2cell/builds/bt_x/20261001T080000000Z");
        std::fs::create_dir_all(&no_plc).unwrap();
        std::fs::write(no_plc.join("build.json"), r#"{"ok":true}"#).unwrap();
        let err = f.service.deploy(DeployRequest { build: Some("20261001T080000000Z-bt_x".into()), ..Default::default() }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        assert!(err.1.contains("plc/project.json"), "{}", err.1);
        assert_eq!(calls(&f, "stage.calls").len(), 1);

        // Run Control Code on the newest export: one job at a time, auto-restart paused.
        f.recorder.statuses.lock().unwrap().clear();
        std::fs::write(f.root.join("slow-projects"), "1").unwrap();
        let accepted = f.service.deploy(DeployRequest::default()).await.unwrap();
        assert_eq!(accepted.deploy.state, JobState::Running);
        assert_eq!(accepted.deploy.build.as_deref(), Some("20261013T090000000Z-bt_a"));
        assert_eq!(accepted.deploy.plan_id.as_deref(), Some("16#8E35"));
        assert_eq!(accepted.deploy.stages, ["projects", "deploy", "krl", "start"]);
        assert!(accepted.busy());
        assert_eq!(accepted.auto_restart.message.as_deref(), Some("auto-restart paused while Run Control Code deploys"));
        for err in [
            f.service.production(ProductionRequest::default()).await.unwrap_err(),
            f.service.bringup(BringupRequest::default()).await.unwrap_err(),
            f.service.deploy(DeployRequest::default()).await.unwrap_err(),
        ] {
            assert_eq!(err.0, StatusCode::CONFLICT);
            assert!(err.1.starts_with("Run Control Code (deploy) is running"), "{}", err.1);
        }
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("slow-projects")).unwrap();
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert_eq!(status.deploy.done, ["projects", "deploy", "krl", "start"]);
        assert!(status.deploy.message.as_deref().unwrap().starts_with("20261013T090000000Z-bt_a (PLAN_ID 16#8E35) deployed and in production"), "{:?}", status.deploy.message);
        assert_eq!(status.cell.state, JobState::Done);
        assert_eq!(status.cell.build.as_deref(), Some("20261013T090000000Z-bt_a"));
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_RUNNING"));
        // The stage order, each once, all on that export.
        let d = newest.display();
        assert_eq!(calls(&f, "stage.calls")[1..], [format!("projects {d}"), format!("deploy {d}"), format!("krl {d}"), format!("start {d}")]);
        let args = calls(&f, "stage.args");
        assert_eq!(args[0], format!("cell projects {d} --cell-config {} --json", config.display()));
        let start = calls(&f, "plan2cell.args");
        assert_eq!(start, [format!("cell start {d} --cell-config {} --restart-runtime --pulses 16 --json", config.display())]);
        // Live: the stage running, in order.
        let seen: Vec<String> = f.recorder.statuses.lock().unwrap().iter().filter(|s| s.deploy.state == JobState::Running).filter_map(|s| s.deploy.stage.clone()).collect();
        let mut order = seen.clone();
        order.dedup();
        assert_eq!(order, ["projects", "deploy", "krl", "start"], "{seen:?}");
        // The head runs the new code: Restart PLC runtime and auto-restart follow it.
        let a = load_auto(&f.cell);
        assert_eq!(a.status.deployed_plan_id.as_deref(), Some("16#8E35"));
        assert_eq!(a.production_dir.as_deref(), Some(d.to_string().as_str()));
        assert!(a.in_production && a.last_job_s.is_some() && a.head_restart_count.is_none());
        assert!(status.in_production);
        assert_eq!(status.deployed_export.as_ref().map(|e| (e.build.as_str(), e.source.as_str())), Some(("20261013T090000000Z-bt_a", "deploy")));
        assert_eq!(status.head_plan_id.as_deref(), Some("16#8E35"));
        assert_eq!(status.head_export.as_ref().map(|e| e.build.as_str()), Some("20261013T090000000Z-bt_a"));
        assert!(status.auto_restart.message.as_deref().is_some_and(|m| !m.contains("paused")), "{:?}", status.auto_restart.message);
        let logs = f.recorder.logs.lock().unwrap().join("\n");
        assert!(logs.contains("projects: working") && !logs.contains("tok-123"), "{logs}");
        f.service.production(ProductionRequest { match_deployed: true, build: None }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        assert_eq!(calls(&f, "stage.calls").last().unwrap(), &format!("start {d}"));

        // A failed projects stage: a clear status, nothing changed on the PLCs, no later stage.
        std::fs::write(f.root.join("fail-projects"), "").unwrap();
        let before = calls(&f, "stage.calls").len();
        f.service.deploy(DeployRequest { build: Some("20261012T075502149Z-bt_a".into()), ..Default::default() }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("fail-projects")).unwrap();
        assert_eq!(status.deploy.state, JobState::Failed);
        assert_eq!(status.deploy.stage.as_deref(), Some("projects"));
        assert!(status.deploy.done.is_empty());
        let message = status.deploy.message.as_deref().unwrap();
        assert!(message.starts_with("projects failed: plan2cell cell projects failed (exit 8): not built: HeadPLC"), "{message}");
        assert!(message.contains("HeadPLC: PLC_PRG(12): C0046") && message.contains("nothing was changed"), "{message}");
        assert_eq!(calls(&f, "stage.calls")[before..], [format!("projects {}", head.display())]);
        assert_eq!(load_auto(&f.cell).status.deployed_plan_id.as_deref(), Some("16#8E35"), "unchanged");
        assert_eq!(status.cell.state, JobState::Done, "production untouched");

        // A failed deploy stage: what the head runs is unknown now; no krl, no start.
        std::fs::write(f.root.join("fail-deploy"), "").unwrap();
        let before = calls(&f, "stage.calls").len();
        f.service.deploy(DeployRequest { build: Some("20261012T075502149Z-bt_a".into()), ..Default::default() }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("fail-deploy")).unwrap();
        assert_eq!((status.deploy.state, status.deploy.stage.as_deref()), (JobState::Failed, Some("deploy")));
        assert_eq!(status.deploy.done, ["projects"]);
        assert!(status.deploy.message.as_deref().unwrap().starts_with("deploy failed: plan2cell cell deploy failed (exit 8)"));
        assert_eq!(calls(&f, "stage.calls")[before..], [format!("projects {}", head.display()), format!("deploy {}", head.display())]);
        assert_eq!(load_auto(&f.cell).status.deployed_plan_id, None);
        assert!(f.service.inner.lock().unwrap().job.is_none(), "the cell is free again");

        // A failed start: deployed (the PLAN_ID is the export's), the production line failed too.
        std::fs::write(f.root.join("fail-start"), "").unwrap();
        f.service.deploy(DeployRequest { build: Some("20261012T075502149Z-bt_a".into()), ..Default::default() }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("fail-start")).unwrap();
        assert_eq!((status.deploy.state, status.deploy.stage.as_deref()), (JobState::Failed, Some("start")));
        assert_eq!(status.deploy.done, ["projects", "deploy", "krl"]);
        assert!(status.deploy.message.as_deref().unwrap().contains("CS_ERROR ERR_ROBOT robot 2"), "{:?}", status.deploy.message);
        assert_eq!(status.cell.state, JobState::Failed);
        assert_eq!(load_auto(&f.cell).status.deployed_plan_id.as_deref(), Some("16#9201"));
        assert!(!status.in_production, "deployed, the start failed: not in production (auto-restart stays out)");

        // A cell without KUKA VCs: no krl stage.
        std::fs::write(&config, "[plc_host]\nssh_host = \"plc-host\"\n").unwrap();
        let before = calls(&f, "stage.calls").len();
        f.service.deploy(DeployRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert_eq!(calls(&f, "stage.calls")[before..], [format!("projects {d}"), format!("deploy {d}"), format!("start {d}")]);
        assert!(!has_kuka_vc(&config) && !has_kuka_vc(&f.root.join("missing.toml")));
    }

    #[tokio::test]
    async fn deploy_only_leaves_the_head_deployed_not_producing() {
        let f = fixture("deploy-only", Some("plan2cell-cell-control"));
        let running = build(&f, "bt_a", "20261012T075502149Z", "16#9201");
        let newest = build(&f, "bt_a", "20261013T090000000Z", "16#8E35");
        std::fs::write(
            newest.join("plc/project.json"),
            json!({"plan": {"cell_id": "weld"}, "projects": [
                {"key": "robot1", "role": "robot_sim", "name": "RobotSim_weld_R1", "file": "RobotSim_weld_R1.project"},
                {"key": "head", "role": "head_plc", "name": "HeadPLC_weld", "file": "HeadPLC_weld.project"}]})
            .to_string(),
        )
        .unwrap();
        let config = f.root.join("etc-cell.toml");
        std::fs::write(&config, "[codesys_mcp]\nwindows_project_dir = 'D:\\p2c\\{cell}\\'\n[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\n").unwrap();
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        // In production on the older export first.
        f.service.production(ProductionRequest { match_deployed: false, build: Some("20261012T075502149Z-bt_a".into()) }).await.unwrap();
        let status = settled(&f.service).await;
        assert!(status.in_production, "{:#?}", status.cell);
        assert_eq!(status.deployed_export, None, "nothing deployed by this backend, no deploy report");

        let accepted = f.service.deploy(DeployRequest { build: None, start: false, stages: None, force: false }).await.unwrap();
        assert_eq!(accepted.deploy.stages, ["projects", "deploy", "krl"], "no start");
        assert!(!accepted.deploy.starts());
        assert!(accepted.deploy.message.as_deref().unwrap().starts_with("Deploy: 20261013T090000000Z-bt_a (PLAN_ID 16#8E35)"), "{:?}", accepted.deploy.message);
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert_eq!(status.deploy.done, ["projects", "deploy", "krl"]);
        let message = status.deploy.message.as_deref().unwrap();
        assert!(message.starts_with("20261013T090000000Z-bt_a (PLAN_ID 16#8E35) deployed, not started"), "{message}");
        assert!(message.ends_with("; the head project is open in the PLC IDE"), "{message}");
        assert_eq!(status.deploy.ide_opened, Some(true));
        let d = newest.display();
        assert_eq!(calls(&f, "stage.calls")[1..], [format!("projects {d}"), format!("deploy {d}"), format!("krl {d}")]);
        assert_eq!(calls(&f, "start.calls").len(), 1, "no plan2cell cell start");

        // The definite state: deployed (the head runs the new PLAN_ID), not in production.
        assert!(!status.in_production);
        assert_eq!((status.cell.state, status.cell.head_state.as_deref()), (JobState::Idle, None));
        assert!(status.cell.message.as_deref().unwrap().contains("not in production yet: Start production"));
        let deployed = status.deployed_export.clone().unwrap();
        assert_eq!((deployed.build.as_str(), deployed.plan_id.as_deref(), deployed.source.as_str()), ("20261013T090000000Z-bt_a", Some("16#8E35"), "deploy"));
        assert_eq!(status.head_plan_id.as_deref(), Some("16#8E35"));
        assert_eq!(status.head_export.as_ref().map(|e| e.build.as_str()), Some("20261013T090000000Z-bt_a"));
        // Where to look in the PLC IDE.
        let ide = status.ide_project.clone().unwrap();
        assert_eq!((ide.name.as_str(), ide.path.as_str()), ("HeadPLC_weld", r"D:\p2c\weld\HeadPLC_weld.project"));
        let a = load_auto(&f.cell);
        assert!(!a.in_production && a.deployed_build.as_deref() == Some("20261013T090000000Z-bt_a"));
        assert_eq!(a.production_dir.as_deref(), Some(d.to_string().as_str()), "the runtimes read are the new export's");
        // Auto-restart does not act on a cell that is not in production.
        assert!(f.service.inner.lock().unwrap().job.is_none());

        // Start production then starts what was deployed.
        f.service.production(ProductionRequest::default()).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.cell.state, JobState::Done, "{:#?}", status.cell);
        assert_eq!(calls(&f, "start.calls").last().unwrap(), &format!("cell start {d} --cell-config {} --restart-runtime --pulses 16 --json", config.display()));
        assert!(status.in_production);

        // Stop production: plan2cell cell stop on that export; out of production, the head stopped.
        f.service.stop_production().await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(calls(&f, "stop.calls"), [format!("cell stop {d} --cell-config {} --json", config.display())]);
        assert_eq!((status.cell.state, status.cell.head_state.as_deref()), (JobState::Done, Some("CS_ERROR")));
        assert!(status.cell.message.as_deref().unwrap().starts_with("production stopped: CS_ERROR ERR_OPERATOR_STOP"), "{:?}", status.cell.message);
        assert!(!status.in_production && !load_auto(&f.cell).in_production);

        // The deploy stage's open_project refused (or an older plan2cell without it): the
        // message says so instead of claiming the head project is open.
        std::fs::write(f.root.join("ide-refused"), "").unwrap();
        f.service.deploy(DeployRequest { build: None, start: false, stages: None, force: false }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("ide-refused")).unwrap();
        let message = status.deploy.message.as_deref().unwrap();
        assert!(message.ends_with("; HeadPLC not opened in the PLC IDE: Refusing to switch projects: open it in CODESYS by hand (File > Open)"), "{message}");
        assert_eq!(status.deploy.ide_opened, Some(false));
        std::fs::write(f.root.join("ide-old"), "").unwrap();
        f.service.deploy(DeployRequest { build: None, start: false, stages: None, force: false }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("ide-old")).unwrap();
        assert!(status.deploy.message.as_deref().unwrap().contains("did not report opening the head project"), "{:?}", status.deploy.message);
        assert_eq!((status.deploy.ide_opened, status.deploy.ide_note.as_deref()), (None, None));

        // A failed deploy stage of a Deploy only: nothing known deployed, not in production, even
        // with an earlier export's successful deploy report (the reports no longer count).
        std::fs::write(
            running.join(codegen::CELL_REPORT),
            json!({"schema": "plan2cell/cell-report@1", "ok": true, "stages": [{"stage": "deploy", "ok": true, "notes": [], "error": null}], "projects": []}).to_string(),
        )
        .unwrap();
        tick(&f).await;
        assert_eq!(f.service.status().deployed_export.map(|e| e.source), Some("deploy".to_owned()), "this backend's deploy first");
        std::fs::write(f.root.join("fail-deploy"), "").unwrap();
        f.service.deploy(DeployRequest { build: Some("20261012T075502149Z-bt_a".into()), start: false, stages: None, force: false }).await.unwrap();
        let status = settled(&f.service).await;
        std::fs::remove_file(f.root.join("fail-deploy")).unwrap();
        assert_eq!((status.deploy.state, status.deploy.stage.as_deref()), (JobState::Failed, Some("deploy")));
        assert!(!status.in_production && status.deployed_export.is_none(), "{:?}", status.deployed_export);
        assert_eq!(status.head_plan_id, None, "what the head holds is unknown");
        assert!(load_auto(&f.cell).deploy_unknown);
        assert!(running.is_dir());
        // A deploy that passes counts again.
        f.service.deploy(DeployRequest { build: None, start: false, stages: None, force: false }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.deployed_export.map(|e| e.build), Some("20261013T090000000Z-bt_a".to_owned()));
        assert!(!load_auto(&f.cell).deploy_unknown);

        // Another active cell: the last Deploy and production start are not its (reset to idle);
        // what was read of the head PLC stays.
        f.service.inner.lock().unwrap().status.cell.head_state = Some("CS_INIT".into());
        let other = f.root.join("cells/other");
        std::fs::create_dir_all(other.join(cells::PLANS_DIR)).unwrap();
        let mut config = f.service.config.get();
        config.cell_dir = other.display().to_string();
        f.service.config.set(config);
        let status = tick(&f).await;
        assert_eq!((status.deploy.state, status.deploy.build.as_deref()), (JobState::Idle, None), "{:#?}", status.deploy);
        assert_eq!((status.cell.state, status.cell.build.as_deref(), status.cell.message.as_deref()), (JobState::Idle, None, None));
        assert_eq!(status.cell.head_state.as_deref(), Some("CS_INIT"));
        assert_eq!(status.deployed_export, None);
    }

    #[test]
    fn plan_ids_of_the_guard() {
        let guard = |id: &str| {
            guard_plan_id(&format!("plan2cell cell start failed (exit 7): start: the head runs PLAN_ID {id} (GVL_Cell.PLAN_ID), this export is plan 16#4195"))
        };
        for (literal, plain) in [("WORD#37377", "37377"), ("16#9201", "16#9201"), ("UINT#16#9201", "16#9201"), ("WORD#16#9201", "16#9201"), ("37377", "37377"), ("0x9201", "0x9201")] {
            let id = guard(literal).unwrap();
            assert_eq!(id, plain);
            assert_eq!(codegen::plan_id_number(&id), Some(0x9201), "{literal}");
            assert!(codegen::same_plan_id(&id, "16#9201"), "{literal}");
        }
        assert_eq!(codegen::plan_id_number("92A1"), Some(0x92A1), "plain hex");
        assert_eq!(codegen::plan_id_number("9201"), Some(9201), "digits only: decimal");
        assert!(!codegen::same_plan_id("WORD#37377", "16#4195"));
        assert_eq!(guard_plan_id("start: no head"), None);
    }

    #[test]
    fn auto_restart_decisions() {
        let now = 1_000_000.0;
        let on = AutoFile {
            status: AutoRestartStatus { enabled: true, threshold_s: 300, ..Default::default() },
            in_production: true,
            production_done_s: Some(now - 3600.0),
            head_restart_count: Some(2),
            ..Default::default()
        };
        let rt = PlcRuntime {
            running: true,
            started_s: Some(now - 3700.0),
            restart_count: Some(2),
            stops_s: Some(now + 3500.0),
            seconds_left: Some(3500.0),
            container: Some("codesys-master".into()),
            ..Default::default()
        };
        assert_eq!(auto_reason(&on, &rt, None, now), None);
        let soon = PlcRuntime { seconds_left: Some(200.0), stops_s: Some(now + 200.0), ..rt.clone() };
        assert!(auto_reason(&on, &soon, None, now).unwrap().starts_with("codesys-master stops at"));
        let idle = AutoFile { in_production: false, ..on.clone() };
        assert_eq!(auto_reason(&idle, &soon, None, now), None, "not in production: not even before the demo stop");
        let over = PlcRuntime { seconds_left: Some(0.0), stops_s: Some(now - 5.0), running: false, ..rt.clone() };
        assert!(auto_reason(&on, &over, None, now).unwrap().contains("ended at"));
        let down = PlcRuntime { running: false, seconds_left: None, stops_s: None, ..rt.clone() };
        assert!(auto_reason(&on, &down, None, now).unwrap().contains("not running"));
        assert_eq!(auto_reason(&idle, &down, None, now), None, "not in production");
        // A newer start alone (docker restart from elsewhere) is not a restart of its own.
        let restarted = PlcRuntime { started_s: Some(now - 60.0), ..rt.clone() };
        assert_eq!(auto_reason(&on, &restarted, None, now), None);
        let own = PlcRuntime { restart_count: Some(3), ..restarted.clone() };
        assert!(auto_reason(&on, &own, None, now).unwrap().contains("by itself"), "{:?}", auto_reason(&on, &own, None, now));
        assert_eq!(auto_reason(&AutoFile { head_restart_count: None, ..on.clone() }, &own, None, now), None, "not taken yet");
        assert_eq!(auto_reason(&on, &PlcRuntime { restart_count: Some(0), ..restarted }, None, now), None, "recreated");
        for held in [
            AutoFile { status: AutoRestartStatus { enabled: false, ..on.status.clone() }, ..on.clone() },
            AutoFile { status: AutoRestartStatus { blocked: Some("guard".into()), ..on.status.clone() }, ..on.clone() },
            AutoFile { status: AutoRestartStatus { last_attempt_s: Some(now - 599.0), ..on.status.clone() }, ..on.clone() },
            AutoFile { last_job_s: Some(now - 599.0), ..on.clone() },
        ] {
            assert_eq!(auto_reason(&held, &soon, None, now), None, "{held:?}");
        }
        let later = AutoFile { status: AutoRestartStatus { last_attempt_s: Some(now - 601.0), ..on.status.clone() }, ..on.clone() };
        assert!(auto_reason(&later, &soon, None, now).is_some());
        assert_eq!(auto_reason(&on, &PlcRuntime { error: Some("x".into()), ..soon }, None, now), None, "not read: no decision");

        // The head stopped production by itself (live): after the grace, once per backoff; not Stop production's.
        let on_done = AutoFile { production_done_s: Some(now - 1500.0), ..on.clone() };
        let err = |error: &str, since: f64| HeadLive { state: "CS_ERROR".into(), error: Some(error.into()), cycles: 39, since_s: since, seen_s: now - 2.0 };
        let lost = err("ERR_DRIVES_LOST", now - 30.0);
        assert!(auto_reason(&on_done, &rt, Some(&lost), now).unwrap().contains("CS_ERROR ERR_DRIVES_LOST after 39 cycle(s)"));
        assert_eq!(auto_reason(&on_done, &rt, Some(&err("ERR_DRIVES_LOST", now - 5.0)), now), None, "within the grace");
        assert_eq!(auto_reason(&on_done, &rt, Some(&err("ERR_OPERATOR_STOP", now - 30.0)), now), None, "Stop production");
        assert_eq!(auto_reason(&on_done, &rt, Some(&err("ERR_DRIVES_LOST", now - 1600.0)), now), None, "before the last start");
        assert_eq!(auto_reason(&on_done, &rt, Some(&HeadLive { seen_s: now - 120.0, ..lost.clone() }), now), None, "stale read");
        assert_eq!(auto_reason(&AutoFile { in_production: false, ..on_done.clone() }, &rt, Some(&lost), now), None, "not in production");
        assert_eq!(auto_reason(&AutoFile { last_job_s: Some(now - 300.0), ..on_done.clone() }, &rt, Some(&lost), now), None, "backoff");
        let running = HeadLive { state: "CS_RUNNING".into(), error: Some("ERR_NONE".into()), ..lost.clone() };
        assert_eq!(auto_reason(&on_done, &rt, Some(&running), now), None);

        assert_eq!(guard_plan_id("start: the head runs PLAN_ID 16#00AA (GVL_Cell.PLAN_ID), this export").as_deref(), Some("16#00AA"));
        assert_eq!(guard_plan_id("start: the head runs PLAN_ID WORD#16#9573 (GVL_Cell.PLAN_ID)").as_deref(), Some("16#9573"));
        assert_eq!(guard_plan_id("not in production after 16 pulse(s)"), None);
        assert!(codegen::same_plan_id("16#00AA", "170") && codegen::same_plan_id("'16#00aa'", "16#AA") && !codegen::same_plan_id("16#1", "2"));
        assert!(codegen::same_plan_id("WORD#16#9573", "16#9573") && codegen::same_plan_id("'WORD#38259'", "16#9573"));
        assert!(!codegen::same_plan_id("1#11", "3") && !codegen::same_plan_id("99#1", "1"), "no radix outside 2..=36 (and no panic)");
        let failed = AutoRestartStatus { enabled: true, last_attempt_s: Some(3600.0), last_error: Some("boom".into()), ..Default::default() };
        assert_eq!(auto_message(&failed, false).unwrap(), "auto-restart at 01:00 UTC failed: boom; next attempt not before 01:10 UTC");
        assert_eq!(
            auto_message(&AutoRestartStatus { enabled: true, threshold_s: 300, ..Default::default() }, false).unwrap(),
            "auto-restart on: 5 min before the CODESYS demo stop"
        );
        assert_eq!(auto_message(&AutoRestartStatus::default(), false), None);
        let attempt = AutoRestartStatus { enabled: true, last_attempt_s: Some(3600.0), ..Default::default() };
        assert_eq!(auto_message(&attempt, true).unwrap(), "auto-restart at 01:00 UTC: running…");
        assert!(auto_message(&attempt, false).unwrap().contains("interrupted"), "no job: not running any more");
        let inactive = AutoRestartStatus { inactive: Some("cannot read".into()), ..attempt };
        assert_eq!(auto_message(&inactive, false).unwrap(), "auto-restart inactive: cannot read");

        let report = json!({"ok": true, "runtimes": [
            {"container": "codesys-master", "head": true, "running": true, "startedS": 10.0, "stopsS": 7210.0, "secondsLeft": 0.0,
             "source": "demo line", "restartCount": 4},
            {"container": "codesys-device", "head": false, "running": false}]});
        let rt = plc_runtime(&report);
        assert!(!rt.running, "the demo period is over");
        assert_eq!((rt.started_s, rt.stops_s, rt.source.as_deref()), (Some(10.0), Some(7210.0), Some("demo line")));
        assert_eq!(rt.restart_count, Some(4));
        assert_eq!(rt.runtimes.len(), 2);
        assert!(plc_runtime(&json!({"ok": true, "runtimes": []})).error.is_some(), "no head");
    }
    #[test]
    fn env_files_times_and_states() {
        let vars = parse_env_file("# x\nFACTORY_OS_CELL_CONTROL_FUNCTION=plan2cell-cell-control\nexport FACTORY_OS_CELL_CONTROL_REGION=\"eu-central-1\"\n\nB='q'\n");
        assert_eq!(vars[0], ("FACTORY_OS_CELL_CONTROL_FUNCTION".into(), "plan2cell-cell-control".into()));
        assert_eq!(vars[1].1, "eu-central-1");
        assert_eq!(vars[2].1, "q");
        assert!(valid_function("arn:aws:lambda:eu-central-1:111122223333:function:plan2cell-cell-control"));
        assert!(!valid_function("a b") && !valid_function(""));
        assert_eq!(parse_iso8601("1970-01-02T00:00:01.500Z"), Some(86_401.5));
        assert_eq!(parse_iso8601("2026-10-12T08:00:00.000Z").map(utc_hhmm).as_deref(), Some("08:00 UTC"));
        assert_eq!(cell_state_in("in production after 4 read(s): CS_RUNNING, udiCycles 0").as_deref(), Some("CS_RUNNING"));
        assert_eq!(cell_state_in("restarted codesys-master"), None);
        assert_eq!(tail_chars("abcdef", 3), "def");
    }


    #[test]
    fn cell_roles() {
        assert_eq!(cell_role(None), (CellRole::Supervisor, None));
        assert_eq!(cell_role(Some("  ")), (CellRole::Supervisor, None));
        assert_eq!(cell_role(Some("operator")), (CellRole::Operator, None));
        assert_eq!(cell_role(Some("Supervisor")), (CellRole::Supervisor, None));
        let (role, warning) = cell_role(Some("operater"));
        assert_eq!(role, CellRole::Operator, "a misspelt role never auto-restarts");
        assert!(warning.unwrap().contains("operater"));
        let busy = json!({"ok": false, "error": {"code": "busy"}, "lease": {"holder": "jdoe", "purpose": "deploy", "sinceS": 29100}});
        assert!(lease_busy(&busy) && !lease_busy(&json!({"ok": false, "error": {"code": "unknown-action"}})));
        let lease = parse_lease(&busy, "plan2cell-test").unwrap();
        assert!(!lease.mine);
        assert_eq!(busy_message(&lease), "cell busy: jdoe is deploy since 08:05 UTC; try again when it has finished");
        assert!(parse_lease(&json!({"ok": true, "holder": "me"}), "me").unwrap().mine);
    }

    /// Until the lease's keeper has released it (its `release` call is in `lease.calls`).
    /// The phase the fake Lambda answers next.
    fn phase(f: &Fixture, phase: &str) {
        std::fs::write(f.root.join("power-phase"), phase).unwrap();
    }

    #[tokio::test]
    async fn start_and_stop_the_virtual_cell_through_the_lambda() {
        let f = fixture("power", Some("plan2cell-cell-control"));
        std::fs::write(f.root.join("power-on"), "").unwrap();
        // One call reads the instances and the power.
        let status = f.service.current().await;
        let power = status.cell_power.clone().expect("power");
        assert_eq!((power.state, power.phase.as_str()), (PowerState::Off, "off"));
        assert_eq!(status.instances.len(), 4);
        assert_eq!(calls(&f, "aws.calls").iter().filter(|c| c.contains("lambda invoke")).count(), 1, "power_status only, no status");
        assert!(status.power_blocks().unwrap().contains("Start virtual cell"));

        // Start virtual cell: answered at once with the Lambda's phase; nothing waits here.
        let status = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        let power = status.cell_power.clone().unwrap();
        assert_eq!((power.state, power.phase.as_str()), (PowerState::Booting, "bring-up"));
        assert!(status.power_blocks().unwrap().contains("booting (bring-up)"));
        let on = calls(&f, "power.calls").into_iter().find(|c| c.contains("power_on")).unwrap();
        assert!(on.contains(r#""holder":"factoryos""#), "{on}");
        assert!(f.service.inner.lock().unwrap().job.is_none(), "no job holds the cell");
        // The cell boots: reads go on without a viewer (the follow-up needs them).
        f.service.inner.lock().unwrap().interest = None;
        assert!(f.service.should_poll());
        phase(&f, "ready");
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert!(f.service.inner.lock().unwrap().power_follow.is_none(), "ready: the follow-up is done");
        assert!(calls(&f, "start.calls").is_empty(), "not asked for production, not in production when stopped");
        assert!(f.recorder.logs.lock().unwrap().iter().any(|l| l.contains("Start virtual cell: the cell is ready")));

        // Stop virtual cell in production: 409 unless force; the Lambda is told it is in production.
        f.service.change_auto(&f.cell, |a| a.in_production = true).unwrap();
        let (code, err) = f.service.power(CellPowerRequest { action: "off".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::CONFLICT);
        assert!(err.contains("in production"), "{err}");
        assert!(!calls(&f, "power.calls").iter().any(|c| c.contains("power_off")), "refused here, before the Lambda");
        let status = f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        let off = calls(&f, "power.calls").into_iter().find(|c| c.contains("power_off")).unwrap();
        assert!(off.contains(r#""force":true"#) && off.contains(r#""inProduction":true"#), "{off}");
        assert_eq!(status.cell_power.as_ref().unwrap().state, PowerState::Off);
        assert!(!status.in_production && status.stopped_in_production, "remembered for the next start");

        // Another backend in production (the Lambda refuses): 409 with its reason.
        std::fs::write(f.root.join("prod-busy"), "").unwrap();
        let (code, err) = f.service.power(CellPowerRequest { action: "off".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::CONFLICT);
        assert!(err.contains("plan2cell-test"), "{err}");
        std::fs::remove_file(f.root.join("prod-busy")).unwrap();
        // Still stopping: Start virtual cell is 409.
        std::fs::write(f.root.join("power-stopping"), "").unwrap();
        let (code, err) = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!((code, err.contains("still stopping")), (StatusCode::CONFLICT, true), "{err}");
        std::fs::remove_file(f.root.join("power-stopping")).unwrap();
        let (code, _) = f.service.power(CellPowerRequest { action: "reboot".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST);
        // GET: the power alone.
        let power = f.service.cell_power().await.unwrap();
        assert_eq!(power.phase, "off");
    }

    #[tokio::test]
    async fn start_virtual_cell_resumes_production_once_ready() {
        let f = fixture("powerprod", Some("plan2cell-cell-control"));
        std::fs::write(f.root.join("power-on"), "").unwrap();
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.change_auto(&f.cell, |a| a.stopped_in_production = true).unwrap();
        let status = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        assert!(!status.stopped_in_production, "used up by this start");
        // Booting: no production yet.
        f.service.runtime_tick().await;
        assert!(calls(&f, "start.calls").is_empty());
        phase(&f, "ready");
        f.service.refresh().await;
        f.service.runtime_tick().await;
        let status = settled(&f.service).await;
        assert_eq!(calls(&f, "start.calls").len(), 1, "Start production once ready");
        assert!(status.in_production, "{:?}", status.cell);
        // A power-on that fails: no production.
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), production: Some(true), ..Default::default() }).await.unwrap();
        phase(&f, "failed");
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert!(f.service.inner.lock().unwrap().power_follow.is_none());
        assert_eq!(calls(&f, "start.calls").len(), 1);
        assert!(f.recorder.logs.lock().unwrap().iter().any(|l| l.contains("did not get ready (phase failed)")), "{:?}", f.recorder.logs.lock().unwrap());
    }

    #[tokio::test]
    async fn an_older_lambda_without_power_still_reads_the_instances() {
        let f = fixture("nopower", Some("plan2cell-cell-control"));
        let status = f.service.current().await;
        assert!(status.cell_power.is_none() && status.message.is_none(), "{:?}", status.message);
        assert_eq!(status.instances.len(), 4, "read with status");
        assert_eq!(status.power_blocks(), None, "nothing blocks without power");
        let (code, err) = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::NOT_IMPLEMENTED, "{err}");
        assert_eq!(f.service.cell_power().await.unwrap_err().0, StatusCode::NOT_IMPLEMENTED);
        // Not asked again on every read.
        let before = calls(&f, "aws.calls").len();
        f.service.refresh().await;
        assert_eq!(calls(&f, "aws.calls").len(), before + 1, "status only");
    }

    #[tokio::test]
    async fn auto_restart_pauses_while_the_cell_is_off_or_booting_and_the_backend_touches() {
        let f = fixture_with("powerauto", Some("plan2cell-cell-control"), |s| s.holder = "plan2cell-test".into());
        std::fs::write(f.root.join("power-on"), "").unwrap();
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        f.service.change_auto(&f.cell, |a| a.in_production = true).unwrap();
        runtime_json(&f.root, true, 120, 5000, 0);
        // Running, but still in bring-up (the Lambda's power-on): no restart, the line says why.
        phase(&f, "bring-up");
        f.service.current().await;
        f.service.refresh().await;
        let status = tick(&f).await;
        assert!(calls(&f, "start.calls").is_empty(), "paused while booting");
        assert_eq!(status.auto_restart.message.as_deref(), Some("auto-restart paused while the virtual cell is booting (bring-up)"));
        // The viewer asks and the cell is in production: the Lambda hears of this backend's use.
        let touches = calls(&f, "touch.calls");
        assert_eq!(touches.len(), 1, "{touches:?}");
        assert!(touches[0].contains(r#""holder":"plan2cell-test""#) && touches[0].contains(r#""inProduction":true"#) && touches[0].contains(r#""role":"supervisor""#));
        tick(&f).await;
        assert_eq!(calls(&f, "touch.calls").len(), 1, "every 5 min, not every tick");
        // Ready: auto-restart acts again.
        phase(&f, "ready");
        f.service.refresh().await;
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 1, "restarted once the cell is ready");
        // Another backend (or the idle stop) powered the cell off: out of production, no fight.
        std::fs::write(f.root.join("power-by"), "idle-stop").unwrap();
        phase(&f, "off");
        std::fs::write(f.root.join("state"), "stopped").unwrap();
        f.service.refresh().await;
        back(&f);
        let status = tick(&f).await;
        assert!(!status.in_production && status.stopped_in_production, "{:?}", status.auto_restart);
        assert_eq!(calls(&f, "start.calls").len(), 1);
        assert!(f.recorder.logs.lock().unwrap().iter().any(|l| l.contains("the virtual cell is off (stopped by idle-stop): out of production")));
    }

    async fn released(f: &Fixture) -> String {
        for _ in 0..200 {
            if let Some(line) = calls(f, "lease.calls").into_iter().find(|l| l.contains(r#""op":"release""#)) {
                return line;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        panic!("the lease was not released: {:?}", calls(f, "lease.calls"));
    }

    #[tokio::test]
    async fn user_actions_hold_the_cells_lease() {
        let f = fixture_with("lease", Some("plan2cell-cell-control"), |s| s.holder = "jdoe".into());
        // An older Lambda without leases: the action runs as before.
        let status = f.service.leased("start", true, f.service.start_stop("start", StartStopRequest::instances_only())).await.unwrap();
        assert!(status.instances.iter().all(|i| i.state == "running"));
        assert_eq!(calls(&f, "lease.calls").len(), 1, "one acquire, no keeper");
        assert!(f.service.status().lease.is_none());

        // With leases: held for the job, released with what changed.
        std::fs::write(f.root.join("lease-on"), "").unwrap();
        std::fs::remove_file(f.root.join("lease.calls")).unwrap();
        f.service.leased("stop", true, f.service.start_stop("stop", StartStopRequest::default())).await.unwrap();
        let release = released(&f).await;
        let calls_ = calls(&f, "lease.calls");
        assert!(calls_[0].contains(r#""op":"acquire""#) && calls_[0].contains(r#""holder":"jdoe""#) && calls_[0].contains(r#""purpose":"stop""#), "{calls_:?}");
        let release: Value = serde_json::from_str(&release).unwrap();
        assert_eq!((release["lastChange"]["holder"].as_str(), release["lastChange"]["action"].as_str()), (Some("jdoe"), Some("stop")));
        assert!(release["lastChange"]["atS"].as_f64().unwrap() > 0.0);

        // Another backend holds the cell: 409, and nothing runs.
        std::fs::write(f.root.join("lease-busy"), r#"{"holder":"plan2cell-test","purpose":"production","sinceS":29100}"#).unwrap();
        let before = calls(&f, "aws.calls").len();
        let (code, err) = f.service.leased("start", true, f.service.start_stop("start", StartStopRequest::instances_only())).await.unwrap_err();
        assert_eq!(code, StatusCode::CONFLICT);
        assert_eq!(err, "cell busy: plan2cell-test is production since 08:05 UTC; try again when it has finished");
        assert_eq!(calls(&f, "aws.calls").len(), before + 1, "only the acquire");
        let lease = f.service.status().lease.unwrap();
        assert_eq!(lease.describe().as_deref(), Some("Cell in use by plan2cell-test (production, since 08:05 UTC)"));
        assert!(!lease.mine);
    }

    #[tokio::test]
    async fn an_operator_never_auto_restarts() {
        let f = fixture_with("operator", Some("plan2cell-cell-control"), |s| {
            s.role = CellRole::Operator;
            s.holder = "jdoe".into();
            s.supervisor = Some("plan2cell-test".into());
        });
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        assert_eq!((f.service.status().cell_role, f.service.status().cell_supervisor.as_deref()), (CellRole::Operator, Some("plan2cell-test")));
        let (code, err) = f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap_err();
        assert_eq!(code, StatusCode::CONFLICT);
        assert!(err.starts_with("Auto-restart runs on plan2cell-test"), "{err}");
        assert!(f.service.set_auto_restart(AutoRestartSettings { enabled: false, threshold_s: None }).is_ok(), "off is fine");
        // Even with an old file that has it on, in production, 2 min left: no restart.
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        f.service
            .change_auto(&f.cell, |a| {
                a.status.enabled = true;
                a.in_production = true;
                a.stopped_in_production = true;
            })
            .unwrap();
        runtime_json(&f.root, true, 120, 5000, 0);
        let status = tick(&f).await;
        assert!(calls(&f, "start.calls").is_empty(), "an operator never restarts");
        assert_eq!(status.auto_restart.message.as_deref(), Some("Auto-restart runs on plan2cell-test"));
        // Start cell does not resume production on an operator either.
        f.service.start_stop("stop", StartStopRequest::default()).await.unwrap();
        f.service.change_auto(&f.cell, |a| a.stopped_in_production = true).unwrap();
        let status = f.service.start_stop("start", StartStopRequest::default()).await.unwrap();
        assert!(!status.start.production, "{:?}", status.start);
        start_settled(&f.service).await;
        assert!(calls(&f, "start.calls").is_empty());
    }

    #[tokio::test]
    async fn the_supervisor_leaves_production_another_backend_changed() {
        let f = fixture_with("supervisor", Some("plan2cell-cell-control"), |s| s.holder = "plan2cell-test".into());
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        let now = unix_s(SystemTime::now());
        f.service
            .change_auto(&f.cell, |a| {
                a.in_production = true;
                a.production_done_s = Some(now - 100.0);
            })
            .unwrap();
        std::fs::write(f.root.join("lease-on"), "").unwrap();
        // Its own change: nothing happens.
        let change = |holder: &str, at: f64| json!({"holder": null, "lastChange": {"holder": holder, "action": "deploy", "planId": "16#9201", "atS": at}});
        std::fs::write(f.root.join("lease-status"), change("plan2cell-test", now).to_string()).unwrap();
        runtime_json(&f.root, true, 3000, 5000, 0);
        tick(&f).await;
        assert!(load_auto(&f.cell).in_production);
        // Another backend's, before its own production start: nothing either.
        f.service.inner.lock().unwrap().lease_read = None;
        std::fs::write(f.root.join("lease-status"), change("jdoe", now - 200.0).to_string()).unwrap();
        tick(&f).await;
        assert!(load_auto(&f.cell).in_production);
        // After it: out of production, auto-restart idle, says why.
        f.service.inner.lock().unwrap().lease_read = None;
        std::fs::write(f.root.join("lease-status"), change("jdoe", now).to_string()).unwrap();
        let status = tick(&f).await;
        let a = load_auto(&f.cell);
        assert!(!a.in_production && !a.stopped_in_production);
        let message = status.auto_restart.message.unwrap();
        assert!(message.starts_with("production changed by jdoe at ") && message.ends_with("(deploy); auto-restart idle"), "{message}");
        // Another backend holds the cell: no auto-restart meanwhile, and no failure counted.
        f.service.change_auto(&f.cell, |a| a.in_production = true).unwrap();
        back(&f);
        f.service.inner.lock().unwrap().lease_read = None;
        std::fs::write(f.root.join("lease-status"), json!({"holder": "jdoe", "purpose": "deploy", "sinceS": now, "expiresS": now + 180.0}).to_string()).unwrap();
        runtime_json(&f.root, true, 120, 5000, 0);
        tick(&f).await;
        assert!(calls(&f, "start.calls").is_empty());
        assert_eq!(load_auto(&f.cell).status.failures, 0);
        // Free again: it restarts, holding the lease for it.
        f.service.inner.lock().unwrap().lease_read = None;
        std::fs::write(f.root.join("lease-status"), r#"{"holder":null}"#).unwrap();
        tick(&f).await;
        assert_eq!(calls(&f, "start.calls").len(), 1);
        assert!(calls(&f, "lease.calls").iter().any(|l| l.contains(r#""purpose":"auto-restart""#)));
    }

    #[test]
    fn show_ide_next_to_the_mcp_url() {
        let url = |toml: &str| show_ide_url(toml).map(|u| u.to_string());
        assert_eq!(url("[codesys_mcp]\nurl = \"http://10.0.2.20:8766/mcp\"\n").as_deref(), Some("http://10.0.2.20:8766/show-ide"));
        assert_eq!(url("[codesys_mcp]\nurl = \"https://ide:9/x/mcp?a=1#f\"\n").as_deref(), Some("https://ide:9/show-ide"));
        assert_eq!(url("[codesys_mcp]\nwindows_project_dir = 'C:\\p'\n"), None);
        assert_eq!(url("[codesys_mcp]\nurl = \"stdio:codesys\"\n"), None);
        assert_eq!(url("not toml ["), None);
    }

    /// Open PLC IDE: the bridge's `POST /show-ide` with the MCP token and the head project (none
    /// while a job holds the cell); the secret is read once, again after a `401`.
    #[tokio::test]
    async fn show_ide_asks_the_mcp_bridge() {
        type Seen = Arc<Mutex<Vec<(String, String, Value)>>>;
        let seen: Seen = Arc::default();
        let refuse = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let app = axum::Router::new().route(
            "/{*path}",
            post({
                let (seen, refuse) = (seen.clone(), refuse.clone());
                move |uri: axum::http::Uri, headers: axum::http::HeaderMap, body: Bytes| async move {
                    let auth = headers.get("authorization").and_then(|v| v.to_str().ok()).unwrap_or_default().to_owned();
                    seen.lock().unwrap().push((uri.path().to_owned(), auth, serde_json::from_slice(&body).unwrap_or(Value::Null)));
                    if refuse.load(std::sync::atomic::Ordering::SeqCst) {
                        return (StatusCode::UNAUTHORIZED, "unauthorized".to_owned());
                    }
                    (StatusCode::OK, json!({"found": true, "shown": true, "title": "HeadPLC_weld.project - CODESYS", "opening": false}).to_string())
                }
            }),
        );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });

        let f = fixture("show-ide", Some("plan2cell-cell-control"));
        let none = f.service.show_ide().await.unwrap_err().to_string();
        assert!(none.contains("no http [codesys_mcp] url"), "{none}");
        std::fs::write(f.root.join("etc-cell.toml"), format!("[codesys_mcp]\nurl = \"http://127.0.0.1:{port}/mcp\"\n")).unwrap();
        let head = r"C:\ProgramData\plan2cell\projects\weld\HeadPLC_weld.project";
        f.service.update(|s| s.ide_project = Some(IdeProject { name: "HeadPLC_weld".into(), path: head.into() }));

        let answer = f.service.show_ide().await.unwrap();
        assert_eq!(answer["shown"], true);
        f.service.inner.lock().unwrap().job = Some("deploy");
        f.service.show_ide().await.unwrap();
        f.service.inner.lock().unwrap().job = None;
        {
            let seen = seen.lock().unwrap();
            assert_eq!(seen.len(), 2);
            assert_eq!((seen[0].0.as_str(), seen[0].1.as_str()), ("/show-ide", "Bearer tok-123"));
            assert_eq!(seen[0].2, json!({"project": head}));
            assert_eq!(seen[1].2, json!({"project": null}), "a job drives CODESYS: only show it");
        }
        let secret_reads = || std::fs::read_to_string(f.root.join("aws.calls")).unwrap_or_default().matches("secretsmanager").count();
        assert_eq!(secret_reads(), 1, "the token is read once");

        refuse.store(true, std::sync::atomic::Ordering::SeqCst);
        let err = f.service.show_ide().await.unwrap_err().to_string();
        assert!(err.contains("401"), "{err}");
        refuse.store(false, std::sync::atomic::Ordering::SeqCst);
        f.service.show_ide().await.unwrap();
        assert_eq!(secret_reads(), 2, "read again after a 401");
    }

    // --- a workspace's own cell (factoryos-cell:ws-<alias>) ---

    /// The `factoryos-cell` Lambda for the alias `ws-jdoe`: `cell_status`, `cell_create` (`exists`;
    /// `cap`: cap-reached), `cell_delete` (`confirm-mismatch`), `power_on` (launch + 1, phase
    /// `launching`; `no-cell`), `power_off` (phase off), `touch`, `restore_report` (to
    /// `restore.calls`); `cell` exists while there is one, `cell-phase` its phase, `launch` the
    /// launch, `archive` its archive manifest, `last-build` its lastBuild, `restore-plan` its restore. Every payload goes to
    /// `own.calls`, every call's arguments to `aws.calls`. Stop / start mode: `cell_update` writes `offmode`; with
    /// `offmode` stop, `power_off` (not mode terminate) leaves it `stopped`, and a `power_on` without `"fresh":true`
    /// resumes it (`resume`: the session's `resume`, the launch kept, `launch-time` now, `"resumed":true`).
    /// Templates: `templates` answers the file `templates` (none: an unknown action, an older Lambda); `template` is
    /// `cell_status.template`; `limits` the limits without a cell; `create-fail` a code `cell_create` /
    /// `cell_update` fail with.
    const FAKE_AWS_OWN: &str = r#"#!/bin/sh
dir=$(dirname "$0")
echo "$*" >> "$dir/aws.calls"
if [ "$1" = "secretsmanager" ]; then echo "tok-123"; exit 0; fi
payload=""; last=""
while [ $# -gt 0 ]; do
  case "$1" in --payload) payload="$2"; shift;; esac
  last="$1"; shift
done
echo "$payload" >> "$dir/own.calls"
action=$(echo "$payload" | sed -n 's/.*"action":"\([a-z_]*\)".*/\1/p')
fail() { echo "{\"ok\":false,\"action\":\"$action\",\"error\":{\"code\":\"$1\",\"message\":\"$2\"}}" > "$last"; }
status() {
  launch=$(cat "$dir/launch" 2>/dev/null || echo 0)
  phase=$(cat "$dir/cell-phase" 2>/dev/null || echo off)
  lifecycle=$(cat "$dir/lifecycle" 2>/dev/null || echo off)
  archive=$(cat "$dir/archive" 2>/dev/null || echo null)
  lb=null; [ -f "$dir/last-build" ] && lb="\"$(cat "$dir/last-build")\""
  rp=$(cat "$dir/restore-plan" 2>/dev/null || echo null)
  om=$(cat "$dir/offmode" 2>/dev/null || echo terminate)
  st=false; [ -f "$dir/stopped" ] && st=true
  res=false; [ -f "$dir/resume" ] && res=true
  lt=null; [ -f "$dir/launch-time" ] && lt="\"$(cat "$dir/launch-time")\""
  req=$(cat "$dir/requested" 2>/dev/null || echo 2026-10-15T08:00:00.000Z)
  tmpl=$(cat "$dir/template" 2>/dev/null || echo null)
  limits=$(cat "$dir/limits" 2>/dev/null || echo '{"maxPerWorkspace":1,"max":3,"maxRunning":2,"running":0}')
  if [ ! -f "$dir/cell" ]; then
    echo "{\"ok\":true,\"action\":\"$action\",\"cell\":null,\"cost\":{\"runningUsdPerHour\":2.65,\"offUsdPerMonth\":0,\"note\":\"n\"},\"images\":[{\"role\":\"plcHost\",\"amiId\":\"ami-1\",\"recipeVersion\":\"1.0.0\"}],\"limits\":$limits}" > "$last"
    return
  fi
  case "$phase" in off) pst=off ;; ready) pst=ready ;; archiving|terminating) pst=stopping ;; failed) pst=failed ;; *) pst=booting ;; esac
  hosts=""; wiring=null
  if [ "$phase" != off ]; then
    on=false; ping=Unknown
    case "$phase" in configuring|bring-up|ready) on=true; ping=Online ;; esac
    h() { echo "{\"role\":\"$1\",\"instanceId\":\"i-$1-$launch\",\"state\":\"running\",\"ssmPing\":\"$ping\",\"online\":$on,\"privateIp\":\"10.0.$launch.$2\",\"tagged\":true,\"launchTime\":$lt}"; }
    hosts="$(h plcHost 10),$(h windowsIde 20),$(h kukaVc1 31),$(h kukaVc2 32)"
    case "$phase" in configuring|bring-up|ready)
      wiring="{\"launch\":$launch,\"plcHost\":{\"instanceId\":\"i-plcHost-$launch\",\"ip\":\"10.0.$launch.10\"},\"windowsIde\":{\"instanceId\":\"i-windowsIde-$launch\",\"ip\":\"10.0.$launch.20\",\"mcpUrl\":\"http://10.0.$launch.20:8766/mcp\",\"dcvServer\":\"10.0.$launch.20:8443\"},\"kukaVc\":[{\"role\":\"kukaVc1\",\"instanceId\":\"i-kukaVc1-$launch\",\"ip\":\"10.0.$launch.31\",\"station\":\"kuka-robi1\",\"kliIp\":\"192.168.100.11\"},{\"role\":\"kukaVc2\",\"instanceId\":\"i-kukaVc2-$launch\",\"ip\":\"10.0.$launch.32\",\"station\":\"kuka-robi2\",\"kliIp\":\"192.168.100.12\"}],\"profinet\":{\"cidr\":\"192.168.100.0/24\",\"gateway\":\"192.168.100.1\",\"vni\":1101},\"addressMap\":{\"192.168.100.10\":\"10.0.$launch.11\",\"192.168.100.11\":\"10.0.$launch.12\",\"192.168.100.12\":\"10.0.$launch.13\"},\"mcpTokenSecret\":\"factoryos/cell/ws-jdoe-1\"}" ;;
    esac
    if [ "$wiring" != null ] && [ -f "$dir/wiring" ]; then wiring=$(cat "$dir/wiring"); fi
  fi
  cat > "$last" <<JSON
{"ok":true,"action":"$action","cell":{"id":"ws-jdoe","workspace":"jdoe","lifecycle":"$lifecycle","createdAt":"2026-10-15T08:00:00Z","launch":$launch,"robots":2,
 "deviceSim":false,"vni":1101,"sgId":"sg-1","mcpTokenSecret":"factoryos/cell/ws-jdoe-1","wasOnAtStop":false,"lastBuild":$lb,"archive":$archive,
 "offMode":"$om","stopped":$st},
 "power":{"state":"$pst","phase":"$phase","message":"phase $phase","hosts":[$hosts],"checks":[],
  "session":{"op":"on","by":"factoryos","requestedAt":"$req","done":false,"history":[],"resume":$res}},
 "wiring":$wiring,"template":$tmpl,"restore":$rp,"cost":{"runningUsdPerHour":2.65,"offUsdPerMonth":0,"note":"n"},
 "images":[{"role":"plcHost","amiId":"ami-1","recipeVersion":"1.0.0","buildDate":"2026-10-15"}],
 "limits":{"maxPerWorkspace":1,"max":3,"maxRunning":2,"running":1}}
JSON
}
case "$action" in
  cell_status) status ;;
  templates)
    if [ -f "$dir/templates" ]; then cat "$dir/templates" > "$last"; else fail unknown-action "?"; fi ;;
  cell_create)
    if [ -f "$dir/create-fail" ]; then fail "$(cat "$dir/create-fail")" "template x: the Lambda's words"
    elif [ -f "$dir/cell" ]; then fail exists "ws-jdoe exists"
    elif [ -f "$dir/cap" ]; then fail cap-reached "3 of 3 cells"
    else touch "$dir/cell"; echo off > "$dir/lifecycle"; status; fi ;;
  cell_delete)
    if ! echo "$payload" | grep -q '"confirm":"ws-jdoe"'; then fail confirm-mismatch "confirm must be ws-jdoe"
    else rm -f "$dir/cell"; echo '{"ok":true,"action":"cell_delete","cell":{"id":"ws-jdoe","workspace":"jdoe","lifecycle":"deleting"}}' > "$last"; fi ;;
  power_on)
    if [ ! -f "$dir/cell" ]; then fail no-cell "no virtual cell"
    elif [ -f "$dir/stopped" ] && ! echo "$payload" | grep -q '"fresh":true'; then
      rm -f "$dir/stopped"; touch "$dir/resume"; date -u +%Y-%m-%dT%H:%M:%S.000Z > "$dir/launch-time"
      echo on > "$dir/lifecycle"; echo starting-instances > "$dir/cell-phase"; status
      sed 's/^{"ok":true,/{"ok":true,"resumed":true,/' "$last" > "$last.t" && mv "$last.t" "$last"
    else rm -f "$dir/stopped" "$dir/resume" "$dir/launch-time"; echo $(( $(cat "$dir/launch" 2>/dev/null || echo 0) + 1 )) > "$dir/launch"; echo on > "$dir/lifecycle"; echo launching > "$dir/cell-phase"; status; fi ;;
  power_off)
    if [ "$(cat "$dir/offmode" 2>/dev/null)" = stop ] && ! echo "$payload" | grep -q '"mode":"terminate"'; then touch "$dir/stopped"; else rm -f "$dir/stopped"; fi
    echo off > "$dir/cell-phase"; echo off > "$dir/lifecycle"; status ;;
  cell_update)
    if [ -f "$dir/create-fail" ]; then fail "$(cat "$dir/create-fail")" "template x: the Lambda's words"; echo '{"StatusCode":200}'; exit 0; fi
    om=$(echo "$payload" | sed -n 's/.*"offMode":"\([a-z]*\)".*/\1/p'); [ -n "$om" ] && echo "$om" > "$dir/offmode"; status ;;
  touch) echo "$payload" >> "$dir/touch.calls"; echo '{"ok":true,"action":"touch"}' > "$last" ;;
  restore_report) echo "$payload" >> "$dir/restore.calls"; echo '{"ok":true,"action":"restore_report"}' > "$last" ;;
  *) fail unknown-action "?" ;;
esac
echo '{"StatusCode":200,"ExecutedVersion":"$LATEST"}'
"#;

    /// A workspace bound to its own cell: the function `factoryos-cell`, qualifier `ws-jdoe`; its
    /// wiring goes to `<root>/wire`, which is also the server's cell.toml.
    fn own_fixture(name: &str) -> Fixture {
        let root = std::env::temp_dir().join(format!("factoryos-infra-own-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cells/weld");
        std::fs::create_dir_all(cell.join(cells::PLANS_DIR)).unwrap();
        script(&root.join("aws"), FAKE_AWS_OWN);
        script(&root.join("plan2cell"), FAKE_PLAN2CELL);
        std::fs::write(
            root.join("cell-template.toml"),
            "[codesys_mcp]\nurl = \"http://10.0.2.20:8766/mcp\"\n[plc_host]\nssm_instance = \"i-plcHost\"\n\
             [[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"iiQKA.OS2_KSS_9.2.0_vass\"\nssm_instance = \"i-kukaVc1\"\n",
        )
        .unwrap();
        std::fs::write(root.join("bridge-template.toml"), "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n").unwrap();
        let store = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let mut config = store.get();
        config.cell_dir = cell.display().to_string();
        store.set(config);
        let wire = WireFiles {
            dir: root.join("wire"),
            cell_template: root.join("cell-template.toml"),
            bridge_template: root.join("bridge-template.toml"),
            region: Some("eu-central-1".into()),
        };
        let hardware = HardwareService::with(store.clone(), None, wire.cell_toml());
        let settings = Settings {
            function: Some("factoryos-cell".into()),
            qualifier: Some("ws-jdoe".into()),
            wire: Some(wire),
            region: Some("eu-central-1".into()),
            aws: root.join("aws"),
            poll: Duration::from_millis(20),
            wait_timeout: Duration::from_secs(5),
            bringup_timeout: Duration::from_secs(5),
            production_timeout: Duration::from_secs(10),
            call_timeout: Duration::from_secs(10),
            holder: "jdoe".into(),
            ..Settings::default()
        };
        let recorder = Arc::new(Recorder::default());
        let service = InfraService::with(recorder.clone(), settings, store, hardware, Some(root.join("plan2cell")));
        Fixture { root, cell, service, recorder }
    }

    fn cell_phase(f: &Fixture, phase: &str) {
        std::fs::write(f.root.join("cell-phase"), phase).unwrap();
    }

    /// Until the restore of the current launch has finished.
    async fn restored(f: &Fixture) -> CellRestore {
        for _ in 0..400 {
            f.service.refresh().await;
            f.service.runtime_tick().await;
            let restore = f.service.inner.lock().unwrap().own.restore.clone();
            if let Some(r) = restore.filter(|r| !r.state.busy())
                && !f.service.inner.lock().unwrap().own.restoring
            {
                return r;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!("the restore did not finish: {:#?}", f.service.status().deploy);
    }

    #[tokio::test]
    async fn own_cell_create_start_wire_restore_stop_and_delete() {
        let f = own_fixture("lifecycle");
        let export = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        let id = "20261012T080000000Z-bt_a";
        // No cell by default: nothing to power, Create is offered.
        let list = f.service.cells().await;
        assert!(list.available && list.can_create && list.cells.is_empty() && list.shared.is_none(), "{list:#?}");
        let status = f.service.status();
        assert_eq!(status.binding, Some(CellKind::Own));
        assert_eq!(status.virtual_cell, None);
        assert!(status.power_blocks().unwrap().contains("off"));
        let invokes: Vec<String> = calls(&f, "aws.calls").into_iter().filter(|c| c.contains("lambda invoke")).collect();
        assert!(invokes.iter().all(|c| c.contains("--function-name factoryos-cell") && c.contains("--qualifier ws-jdoe")), "{invokes:#?}");
        let (code, _) = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::NOT_FOUND, "no cell to start");

        // Create: off, no instances; once only.
        let cell = f.service.create_cell(CreateCellRequest::default()).await.unwrap();
        assert_eq!((cell.id.as_str(), cell.lifecycle, cell.launch, cell.kind), ("ws-jdoe", CellLifecycle::Off, 0, CellKind::Own));
        assert_eq!(cell.images[0].ami_id, "ami-1");
        assert!(calls(&f, "own.calls").iter().any(|c| c.contains(r#""action":"cell_create""#) && c.contains(r#""robots":2"#)));
        assert_eq!(f.service.create_cell(CreateCellRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT);
        let list = f.service.cells().await;
        assert!(!list.can_create && list.cells.len() == 1 && list.reason.is_some(), "{list:#?}");
        assert!(f.service.status().power_blocks().unwrap().contains("off"));

        // The last export deployed (the Lambda's lastBuild): Start restores it, Deploy only.
        std::fs::write(f.root.join("last-build"), id).unwrap();
        f.service.refresh().await;
        let status = f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        let on = calls(&f, "own.calls").into_iter().rfind(|c| c.contains("power_on")).unwrap();
        assert!(on.contains(r#""mode":"auto""#) && on.contains(&format!(r#""build":"{id}""#)), "{on}");
        // What is not known is left out (the Lambda wants strings): no PLAN_ID yet.
        assert!(!on.contains("planId") && !on.contains("null"), "{on}");
        assert_eq!(status.cell_power.as_ref().unwrap().phase, "launching");
        assert_eq!(status.virtual_cell.as_ref().unwrap().launch, 1);
        // Configuring: wired only once ready; nothing restored before.
        cell_phase(&f, "configuring");
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert!(!f.root.join("wire/cell.toml").exists(), "not ready: not wired yet");
        assert!(calls(&f, "stage.calls").is_empty());
        assert_eq!(f.service.deploy(DeployRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT, "no cell.toml yet");

        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!((restore.state, restore.mode.as_str(), restore.build.as_deref()), (JobState::Done, "full", Some(id)), "{restore:#?}");
        let toml = std::fs::read_to_string(f.root.join("wire/cell.toml")).unwrap();
        assert!(toml.contains("http://10.0.1.20:8766/mcp") && toml.contains("[address_map]") && toml.contains("i-kukaVc2-1"), "{toml}");
        assert!(std::fs::read_to_string(f.root.join("wire/fos-live-bridge.toml")).unwrap().contains("opc.tcp://10.0.1.11:4840"));
        assert_eq!(std::fs::read_to_string(f.root.join("wire/restored")).unwrap().trim(), "1");
        let shown = format!("{}", export.display());
        assert_eq!(calls(&f, "stage.calls"), [format!("projects {shown}"), format!("deploy {shown}"), format!("krl {shown}")]);
        assert!(calls(&f, "aws.calls").iter().any(|c| c.contains("get-secret-value --secret-id factoryos/cell/ws-jdoe-1")), "the cell's MCP token");
        let report = calls(&f, "restore.calls");
        assert_eq!(report.len(), 1);
        assert!(report[0].contains(r#""ok":true"#) && report[0].contains(r#""mode":"full""#) && report[0].contains(r#""launch":1"#), "{report:?}");
        let status = f.service.status();
        let cell = status.virtual_cell.as_ref().unwrap();
        assert_eq!(cell.restore.as_ref().unwrap().state, JobState::Done);
        assert!(cell.cost.session_usd.is_some_and(|c| c > 0.0));
        assert_eq!(status.instances.len(), 4);
        assert!(status.ready);
        // Read again (a backend restart): neither rewired nor restored again.
        f.service.inner.lock().unwrap().own.restored_launch = None;
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert_eq!(calls(&f, "restore.calls").len(), 1);
        // The touch tells the Lambda what was deployed.
        {
            let mut inner = f.service.inner.lock().unwrap();
            inner.interest = Some(Instant::now());
            inner.touched = None;
        }
        f.service.touch_if_due().await;
        assert!(calls(&f, "touch.calls").last().unwrap().contains(&format!(r#""build":"{id}""#)));

        // Stop: the Lambda archives and terminates.
        let status = f.service.power(CellPowerRequest { action: "off".into(), ..Default::default() }).await.unwrap();
        let off = calls(&f, "own.calls").into_iter().find(|c| c.contains("power_off")).unwrap();
        assert!(off.contains(r#""archive":true"#), "{off}");
        assert_eq!(status.cell_power.as_ref().unwrap().state, PowerState::Off);
        assert!(status.instances.is_empty());

        // Delete: the id typed again.
        let wrong = DeleteCellRequest { confirm: "ws-jdo".into() };
        assert_eq!(f.service.delete_cell("ws-jdoe", wrong).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        assert_eq!(f.service.delete_cell("ws-other", DeleteCellRequest { confirm: "ws-other".into() }).await.unwrap_err().0, StatusCode::NOT_FOUND);
        assert!(!calls(&f, "own.calls").iter().any(|c| c.contains("cell_delete")), "refused here");
        let gone = f.service.delete_cell("ws-jdoe", DeleteCellRequest { confirm: "ws-jdoe".into() }).await.unwrap();
        assert_eq!(gone.lifecycle, CellLifecycle::Deleting);
        assert!(!f.root.join("wire/cell.toml").exists() && !f.root.join("wire/restored").exists());
        assert!(f.service.cells().await.can_create);
        // The account's cap.
        std::fs::write(f.root.join("cap"), "").unwrap();
        assert_eq!(f.service.create_cell(CreateCellRequest::default()).await.unwrap_err().0, StatusCode::TOO_MANY_REQUESTS);
    }

    #[tokio::test]
    async fn own_cell_stop_mode_resume_restores_nothing_and_starts_production_only() {
        let f = own_fixture("resume");
        let export = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        let id = "20261012T080000000Z-bt_a";
        let shown = format!("{}", export.display());
        std::fs::write(f.root.join("cell"), "").unwrap();
        std::fs::write(f.root.join("launch"), "3").unwrap();
        f.service.refresh().await;
        // The off mode: per cell, the Lambda's cell_update.
        let bad = UpdateCellRequest { off_mode: Some("pause".into()), ..Default::default() };
        assert_eq!(f.service.update_cell("ws-jdoe", bad).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        let stop = UpdateCellRequest { off_mode: Some(OFF_MODE_STOP.into()), ..Default::default() };
        assert_eq!(f.service.update_cell("ws-other", stop.clone()).await.unwrap_err().0, StatusCode::NOT_FOUND);
        let cell = f.service.update_cell("ws-jdoe", stop).await.unwrap();
        assert!(cell.stop_mode(), "{cell:#?}");
        assert!(calls(&f, "own.calls").iter().any(|c| c.contains(r#""action":"cell_update""#) && c.contains(r#""offMode":"stop""#)));
        // Launch 3 ran this export in production; Stop (force) stops the instances.
        f.service
            .change_auto(&f.cell, |a| {
                a.deployed_build = Some(id.to_owned());
                a.status.deployed_plan_id = Some("16#0001".into());
                a.in_production = true;
            })
            .unwrap();
        f.service.write_restored(3);
        let status = f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        assert!(status.virtual_cell.as_ref().unwrap().stopped);
        assert!(load_auto(&f.cell).stopped_in_production);

        // Start: resumed (launch 3 kept), ready: no restore, `start` only, without --restart-runtime (a fresh boot).
        std::fs::write(f.root.join("requested"), "2026-10-15T09:00:00.000Z").unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        let on = calls(&f, "own.calls").into_iter().rfind(|c| c.contains("power_on")).unwrap();
        assert!(!on.contains("fresh"), "resume unless asked otherwise: {on}");
        let logs = f.recorder.logs.lock().unwrap().join("\n");
        assert!(logs.contains("resumes the stopped instances"), "{logs}");
        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!((restore.state, restore.mode.as_str(), restore.build.as_deref()), (JobState::Done, "resume", Some(id)), "{restore:#?}");
        assert_eq!(calls(&f, "stage.calls"), [format!("start {shown}")]);
        let args = std::fs::read_to_string(f.root.join("plan2cell.args")).unwrap();
        assert!(!args.contains("--restart-runtime"), "{args}");
        let report = calls(&f, "restore.calls");
        assert!(report[0].contains(r#""mode":"resume""#) && report[0].contains(r#""launch":3"#) && report[0].contains(r#""ok":true"#), "{report:?}");
        assert!(f.service.status().in_production);
        assert_eq!(std::fs::read_to_string(f.root.join("wire/resumed")).unwrap().trim(), "2026-10-15T09:00:00.000Z");
        // Read again (or after a backend restart): not again.
        f.service.inner.lock().unwrap().own.resumed = None;
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert_eq!(calls(&f, "restore.calls").len(), 1);

        // Stopped in production again, `start` fails: krl + start, then Run Control Code; it fails too: Failed.
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        std::fs::write(f.root.join("fail-start"), "").unwrap();
        std::fs::write(f.root.join("requested"), "2026-10-15T10:00:00.000Z").unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!((restore.state, restore.mode.as_str()), (JobState::Failed, "resume"), "{restore:#?}");
        assert!(restore.message.as_deref().unwrap().contains("Start fresh"), "{restore:#?}");
        assert_eq!(
            calls(&f, "stage.calls")[1..],
            [
                format!("start {shown}"),
                format!("krl {shown}"),
                format!("start {shown}"),
                format!("projects {shown}"),
                format!("deploy {shown}"),
                format!("krl {shown}"),
                format!("start {shown}"),
            ]
        );
        assert!(calls(&f, "restore.calls")[1].contains(r#""ok":false"#));

        // Start fresh (one click): fresh: true, a new launch with the usual restore.
        std::fs::remove_file(f.root.join("fail-start")).unwrap();
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), fresh: Some(true), production: Some(false), ..Default::default() }).await.unwrap();
        let on = calls(&f, "own.calls").into_iter().rfind(|c| c.contains("power_on")).unwrap();
        assert!(on.contains(r#""fresh":true"#), "{on}");
        assert_eq!(f.service.status().virtual_cell.unwrap().launch, 4);
        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!((restore.state, restore.mode.as_str()), (JobState::Done, "full"), "{restore:#?}");
        // Stop with a terminate this once.
        f.service.power(CellPowerRequest { action: "off".into(), force: true, mode: Some(OFF_MODE_TERMINATE.into()), ..Default::default() }).await.unwrap();
        let off = calls(&f, "own.calls").into_iter().rfind(|c| c.contains("power_off")).unwrap();
        assert!(off.contains(r#""mode":"terminate""#), "{off}");
        assert!(!f.service.status().virtual_cell.unwrap().stopped);
        let bad = CellPowerRequest { action: "off".into(), force: true, mode: Some("pause".into()), ..Default::default() };
        assert_eq!(f.service.power(bad).await.unwrap_err().0, StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn own_cell_a_failed_launch_keeps_production_due_for_the_next_start() {
        let f = own_fixture("failed-launch");
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        std::fs::write(f.root.join("cell"), "").unwrap();
        f.service.change_auto(&f.cell, |a| a.stopped_in_production = true).unwrap();
        f.service.refresh().await;
        f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        assert!(!load_auto(&f.cell).stopped_in_production, "cleared while the launch runs");
        cell_phase(&f, "failed");
        f.service.refresh().await;
        f.service.runtime_tick().await;
        assert!(load_auto(&f.cell).stopped_in_production, "due again after the failed launch");
        assert!(f.service.inner.lock().unwrap().own.restore_plan.is_none());
    }

    #[tokio::test]
    async fn own_cell_fast_restore_runs_krl_and_start_only() {
        let f = own_fixture("fast");
        let export = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        let id = "20261012T080000000Z-bt_a";
        std::fs::write(f.root.join("cell"), "").unwrap();
        std::fs::write(f.root.join("launch"), "4").unwrap();
        // The archive of launch 4 holds the boot applications of this export; it was in production.
        std::fs::write(f.root.join("archive"), format!(r#"{{"launch":4,"build":"{id}","planId":"16#0001","runtimeVersion":"4.20.0.0"}}"#)).unwrap();
        f.service.change_auto(&f.cell, |a| {
            a.deployed_build = Some(id.into());
            a.status.deployed_plan_id = Some("16#0001".into());
            a.stopped_in_production = true;
        })
        .unwrap();
        f.service.refresh().await;
        // Supervisor: production resumes (it was in production when stopped).
        f.service.power(CellPowerRequest { action: "on".into(), ..Default::default() }).await.unwrap();
        let on = calls(&f, "own.calls").into_iter().rfind(|c| c.contains("power_on")).unwrap();
        assert!(on.contains(r#""planId":"16#0001""#), "{on}");
        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!((restore.state, restore.mode.as_str()), (JobState::Done, "fast"), "{restore:#?}");
        let shown = format!("{}", export.display());
        assert_eq!(calls(&f, "stage.calls"), [format!("krl {shown}"), format!("start {shown}")]);
        let status = f.service.status();
        assert!(status.in_production, "{:#?}", status.deploy);
        assert!(calls(&f, "restore.calls")[0].contains(r#""mode":"fast""#));
        // Without a matching archive, `full` is asked for: Deploy only, and Start production.
        cell_phase(&f, "off");
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), restore: Some("full".into()), production: Some(false), ..Default::default() }).await.unwrap();
        cell_phase(&f, "ready");
        let restore = restored(&f).await;
        assert_eq!(restore.mode, "full");
        assert_eq!(calls(&f, "stage.calls")[2..], [format!("projects {shown}"), format!("deploy {shown}"), format!("krl {shown}")]);
        // The Lambda's plan says fast, but the PLC host's first boot did not restore (another runtime): Deploy only.
        std::fs::write(
            f.root.join("restore-plan"),
            format!(r#"{{"mode":"fast","build":"{id}","planId":"16#0001","firstBoot":{{"plcData":"skipped: runtime 4.19.0.0"}}}}"#),
        )
        .unwrap();
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), production: Some(false), ..Default::default() }).await.unwrap();
        cell_phase(&f, "ready");
        assert_eq!(restored(&f).await.mode, "full");
        assert_eq!(calls(&f, "stage.calls").len(), 8);
        // ... and it did: krl only (not in production).
        std::fs::write(f.root.join("restore-plan"), format!(r#"{{"mode":"fast","build":"{id}","firstBoot":{{"plcData":"ok 3 apps"}}}}"#)).unwrap();
        f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
        f.service.power(CellPowerRequest { action: "on".into(), production: Some(false), ..Default::default() }).await.unwrap();
        cell_phase(&f, "ready");
        assert_eq!(restored(&f).await.mode, "fast");
        assert_eq!(calls(&f, "stage.calls")[8..], [format!("krl {shown}")]);
        let (code, _) = f.service.power(CellPowerRequest { action: "on".into(), restore: Some("later".into()), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST);
    }

    #[test]
    fn a_restore_is_retried_after_an_ide_reset_only_for_codesys_tool_errors() {
        let tool = "projects failed: plan2cell cell projects failed (exit 8): codesys tool get_project_structure: Error retrieving structure.";
        assert!(retry_after_ide_reset(tool, false));
        assert!(retry_after_ide_reset("start failed: plan2cell cell start failed (exit 8): codesys tool connect_to_device: timed out", false));
        // A partly downloaded head (deploy_unknown) is never run again behind the user's back.
        assert!(!retry_after_ide_reset(tool, true));
        // Project errors are deterministic.
        let c0046 = "projects failed: plan2cell cell projects failed (exit 8): not built: HeadPLC (errors in the report) (HeadPLC: PLC_PRG(12): C0046: Identifier x not defined) (nothing was changed on the PLCs)";
        assert!(!retry_after_ide_reset(c0046, false));
        assert!(!retry_after_ide_reset("plan2cell cell projects failed (exit 8): codesys tool build_project: C0046: Identifier x not defined", false));
        assert!(!retry_after_ide_reset("plan2cell cell start failed (exit 8): codesys tool connect_to_device: Compile errors occurred in Device.Application", false));
        assert!(!retry_after_ide_reset("codesys tool compile_project: Build errors: 2", false));
        // A dialog waits for a person; other failures are not the IDE's state.
        assert!(!retry_after_ide_reset("plan2cell cell projects failed (exit 3): a dialog is open: Save changes? (codesys tool create_project)", false));
        assert!(!retry_after_ide_reset("deploy failed: plan2cell cell deploy failed (exit 4): plc host: ssm ConnectionLost", false));
        assert!(!retry_after_ide_reset("Run Control Code (deploy) is running; wait for it before Deploy", false));
        assert!(!retry_after_ide_reset("", false));
        // `C0046` only as a code (not inside a word, a colon after it).
        assert!(project_error("C0046: x") && project_error("x (C1234: y)"));
        assert!(!project_error("ABC0046: x") && !project_error("C0046 x") && !project_error("C004"));
    }

    #[test]
    fn a_fresh_boot_keeps_its_runtimes() {
        let boot = parse_iso8601("2026-10-12T08:00:00.000Z").unwrap();
        let status = |state: &str, launch: Option<&str>| InfraStatus {
            instances: vec![InfraInstance {
                role: ROLE_PLC_HOST.into(),
                state: state.into(),
                launch_time: launch.map(str::to_owned),
                ..Default::default()
            }],
            ..Default::default()
        };
        let up = status("running", Some("2026-10-12T08:00:00.000Z"));
        assert!(fresh_boot(&up, false, boot + 60.0));
        assert!(fresh_boot(&up, false, boot + 29.0 * 60.0));
        // A long-lived cell (demo runtimes stop after ~2 h): restarted as before.
        assert!(!fresh_boot(&up, false, boot + 30.0 * 60.0));
        assert!(!fresh_boot(&up, false, boot + 3.0 * 3600.0));
        assert!(!fresh_boot(&status("stopped", Some("2026-10-12T08:00:00.000Z")), false, boot + 60.0));
        assert!(!fresh_boot(&status("running", None), false, boot + 60.0));
        assert!(!fresh_boot(&InfraStatus::default(), false, boot + 60.0));
    }

    #[test]
    fn restore_reports_fit_the_lambda() {
        let restore = CellRestore {
            state: JobState::Done,
            mode: "full".into(),
            build: Some("b".into()),
            plan_id: None,
            message: Some("é".repeat(600)),
        };
        let payload = restore_report_payload(3, &restore);
        assert_eq!(payload["action"], "restore_report");
        assert_eq!((payload["launch"].as_u64(), payload["ok"].as_bool(), payload["planId"].is_null()), (Some(3), Some(true), true));
        assert_eq!(payload["message"].as_str().unwrap().chars().count(), RESTORE_REPORT_MAX);
        let failed = CellRestore { state: JobState::Failed, ..restore };
        assert_eq!(restore_report_payload(3, &failed)["ok"], false);
    }

    #[tokio::test]
    async fn own_cell_restore_resets_the_ide_and_retries_once() {
        let f = own_fixture("reset-ide");
        let export = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        let id = "20261012T080000000Z-bt_a";
        let shown = format!("{}", export.display());
        std::fs::write(f.root.join("cell"), "").unwrap();
        std::fs::write(f.root.join("last-build"), id).unwrap();
        let relaunch = |restore: &str| {
            let request = CellPowerRequest { action: "on".into(), restore: Some(restore.into()), production: Some(false), ..Default::default() };
            let f = &f;
            async move {
                f.service.power(CellPowerRequest { action: "off".into(), force: true, ..Default::default() }).await.unwrap();
                f.service.power(request).await.unwrap();
                cell_phase(f, "ready");
                restored(f).await
            }
        };
        f.service.refresh().await;

        // A CODESYS tool error: IDE reset, then the full restore once more.
        std::fs::write(f.root.join("tool-fail-projects"), "1").unwrap();
        let restore = relaunch("full").await;
        assert_eq!((restore.state, restore.mode.as_str()), (JobState::Done, "full"), "{restore:#?}");
        assert_eq!(
            calls(&f, "stage.calls"),
            [format!("projects {shown}"), "reset-ide".into(), format!("projects {shown}"), format!("deploy {shown}"), format!("krl {shown}")]
        );
        let reset = calls(&f, "reset.calls");
        assert_eq!(reset.len(), 1);
        assert!(reset[0].starts_with("cell reset-ide --cell-config ") && reset[0].ends_with(" --json"), "{reset:?}");
        assert!(calls(&f, "restore.calls").last().unwrap().contains(r#""ok":true"#));

        // Bounded: a second CODESYS tool error ends the restore.
        std::fs::write(f.root.join("tool-fail-projects"), "2").unwrap();
        let restore = relaunch("full").await;
        assert_eq!(restore.state, JobState::Failed, "{restore:#?}");
        assert!(restore.message.as_deref().unwrap().contains("codesys tool get_project_structure"), "{restore:#?}");
        assert_eq!(calls(&f, "stage.calls")[5..], [format!("projects {shown}"), "reset-ide".into(), format!("projects {shown}")]);
        assert_eq!(calls(&f, "reset.calls").len(), 2);

        // A project error (C0046) is not retried.
        std::fs::write(f.root.join("fail-projects"), "").unwrap();
        let restore = relaunch("full").await;
        assert_eq!(restore.state, JobState::Failed);
        assert_eq!(calls(&f, "stage.calls")[8..], [format!("projects {shown}")]);
        assert_eq!(calls(&f, "reset.calls").len(), 2);
        std::fs::remove_file(f.root.join("fail-projects")).unwrap();

        // Partly downloaded (deploy_unknown): not retried either.
        std::fs::write(f.root.join("tool-fail-deploy"), "1").unwrap();
        let restore = relaunch("full").await;
        assert_eq!(restore.state, JobState::Failed);
        assert!(load_auto(&f.cell).deploy_unknown);
        assert_eq!(calls(&f, "stage.calls")[9..], [format!("projects {shown}"), format!("deploy {shown}")]);
        assert_eq!(calls(&f, "reset.calls").len(), 2);
        let failed = calls(&f, "restore.calls").last().unwrap().clone();
        assert!(failed.contains(r#""ok":false"#), "{failed}");

        // Run Control Code supersedes the failed restore: done, and the Lambda is told (ok:true).
        let launch = f.service.inner.lock().unwrap().own.restored_launch.unwrap();
        f.service.deploy(DeployRequest { build: Some(id.into()), start: false, stages: None, force: true }).await.unwrap();
        f.service.follow_job("deploy").await;
        let mut reports = Vec::new();
        for _ in 0..200 {
            reports = calls(&f, "restore.calls");
            if reports.last() != Some(&failed) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let last = reports.last().unwrap();
        assert!(last.contains(r#""ok":true"#) && last.contains("superseded") && last.contains(&format!(r#""launch":{launch}"#)), "{last}");
        assert!(last.contains(&format!(r#""build":"{id}""#)), "{last}");
        let restore = f.service.status().virtual_cell.unwrap().restore.unwrap();
        assert_eq!(restore.state, JobState::Done);

        // The fast path: a failure (exit 8) gets an IDE reset before the fallback.
        f.service.change_auto(&f.cell, |a| a.status.deployed_plan_id = Some("16#0001".into())).unwrap();
        std::fs::write(f.root.join("archive"), format!(r#"{{"launch":{},"build":"{id}","planId":"16#0001","runtimeVersion":"4.20.0.0"}}"#, launch)).unwrap();
        std::fs::write(f.root.join("tool-fail-krl"), "1").unwrap();
        let before = calls(&f, "stage.calls").len();
        let restore = relaunch("auto").await;
        assert_eq!((restore.state, restore.mode.as_str()), (JobState::Done, "full"), "{restore:#?}");
        assert_eq!(
            calls(&f, "stage.calls")[before..],
            [format!("krl {shown}"), "reset-ide".into(), format!("projects {shown}"), format!("deploy {shown}"), format!("krl {shown}")]
        );
        assert_eq!(calls(&f, "reset.calls").len(), 3);

        // A failed reset (exit 8, the IDE not ready): no second run, the restore says why.
        std::fs::write(f.root.join("tool-fail-projects"), "1").unwrap();
        std::fs::write(f.root.join("fail-reset-ide"), "").unwrap();
        let before = calls(&f, "stage.calls").len();
        let restore = relaunch("full").await;
        assert_eq!(restore.state, JobState::Failed, "{restore:#?}");
        let message = restore.message.as_deref().unwrap();
        assert!(message.contains("the CODESYS IDE reset failed too") && message.contains("exit 8 - the IDE is not ready"), "{message}");
        assert_eq!(calls(&f, "stage.calls")[before..], [format!("projects {shown}"), "reset-ide".into()]);
        assert_eq!(calls(&f, "reset.calls").len(), 4);
    }

    #[tokio::test]
    async fn deploy_runs_the_stages_asked_for() {
        let f = fixture("stages", Some("plan2cell-cell-control"));
        let dir = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        let bad = DeployRequest { stages: Some(vec!["compile".into()]), ..Default::default() };
        assert_eq!(f.service.deploy(bad).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        // In their order, whatever the request's.
        let request = DeployRequest { stages: Some(vec!["start".into(), "krl".into()]), ..Default::default() };
        f.service.deploy(request).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert_eq!(status.deploy.stages, ["krl", "start"]);
        let shown = dir.display();
        assert_eq!(calls(&f, "stage.calls"), [format!("krl {shown}"), format!("start {shown}")]);
    }

    /// plan2cell's report for scene_fit's export (plan2cell/crates/plan2cell/tests/deploy_check.rs).
    const SCENE_FIT_CHECK: &str = r#"{"schema":"plan2cell/deploy-check@1","export":"x","cell_id":"weld","plan_id":"16#A1BE","profile":"vw-ks24","ok":false,"fails":2,"warns":0,"rules":[
{"id":"cell.safety","title":"Safety settings the KUKA VCs can satisfy","scope":"cell","status":"fail","message":"robot1: a safety module: Start production will fail with ERR_SAFETY_TIMEOUT","hint":"Hardware Configuration: robot1 → Safety module off","robots":["robot1"],"fixes":[{"kind":"hardware","field":"robots.robot1.safetyModule","value":false}]},
{"id":"cell.network","title":"Head PLC network","scope":"cell","status":"fail","message":"gateway none (0.0.0.0)","fixes":[{"kind":"hardware","field":"network.gateway","value":"192.168.100.1"}]}]}"#;

    #[tokio::test]
    async fn deploy_checks_first_and_refuses_a_failing_export_unless_forced() {
        let f = fixture("gate", Some("plan2cell-cell-control"));
        let dir = build(&f, "bt_a", "20261012T080000000Z", "16#A1BE");
        f.service.start_stop("start", StartStopRequest::instances_only()).await.unwrap();
        // The check alone: plan2cell's rules against the server's cell.toml, plus the backend's.
        let report = f.service.deploy_check(Default::default()).await.unwrap();
        assert!(report.ok, "{}", report.text());
        assert_eq!((report.cell.as_str(), report.build.as_str()), ("weld", "20261012T080000000Z-bt_a"));
        let ids: Vec<&str> = report.rules.iter().map(|r| r.id.as_str()).collect();
        assert_eq!(ids, ["export.files", "cell.state", "plan.status", "export.stale"], "{}", report.text());
        assert_eq!(report.rules[1].status, factoryos_protocol::deploy_check::CheckStatus::Pass, "{}", report.text());
        let args = std::fs::read_to_string(f.root.join("check.calls")).unwrap();
        assert!(args.contains(&format!("cell check {} --json --cell-config {}", dir.display(), f.root.join("etc-cell.toml").display())), "{args}");
        assert!(dir.join(deploy_check::CHECK_FILE).is_file());
        // A failing check: Deploy is 409 with the report, nothing runs.
        std::fs::write(f.root.join("check-fail"), SCENE_FIT_CHECK).unwrap();
        let err = f.service.deploy(DeployRequest { start: false, ..Default::default() }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::CONFLICT);
        let refused = factoryos_protocol::deploy_check::DeployRefused::from_error(&format!("409 Conflict: {}", err.1)).expect(&err.1);
        assert_eq!(refused.report.fails, 2);
        assert!(refused.error.contains("fix 2 problem(s) first (or Deploy anyway)"), "{}", refused.error);
        assert_eq!(refused.report.fixes(&[]).len(), 2);
        assert!(!f.root.join("stage.calls").exists());
        assert!(f.service.inner.lock().unwrap().job.is_none());
        // Deploy anyway.
        f.service.deploy(DeployRequest { start: false, force: true, ..Default::default() }).await.unwrap();
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert_eq!(calls(&f, "stage.calls")[0], format!("projects {}", dir.display()));
    }

    #[tokio::test]
    async fn the_check_says_when_the_cell_is_off() {
        let f = fixture("gate-off", Some("plan2cell-cell-control"));
        build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.update(|s| {
            s.cell_power = Some(CellPower { state: PowerState::Off, ..Default::default() });
        });
        let report = f.service.deploy_check(Default::default()).await.unwrap();
        let state = report.rules.iter().find(|r| r.id == "cell.state").unwrap();
        assert_eq!(state.status, factoryos_protocol::deploy_check::CheckStatus::Fail, "{}", report.text());
        assert!(state.hint.as_deref().unwrap().contains("Start virtual cell"), "{state:?}");
        assert!(!report.ok);
        // No such export / cell.
        let err = f.service.deploy_check(factoryos_protocol::deploy_check::DeployCheckRequest { build: Some("20261012T080000000Z-nope".into()), cell: None }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::NOT_FOUND);
        let err = f.service.deploy_check(factoryos_protocol::deploy_check::DeployCheckRequest { build: None, cell: Some("../x".into()) }).await.unwrap_err();
        assert_eq!(err.0, StatusCode::BAD_REQUEST);
    }

    #[test]
    fn bindings_and_qualifiers() {
        let shared = Settings { function: Some("plan2cell-cell-control".into()), ..Settings::default() };
        assert_eq!(shared.binding(), Some(CellBinding::Shared { function: "plan2cell-cell-control".into() }));
        let own = Settings { qualifier: Some("ws-jdoe".into()), ..shared.clone() };
        let binding = own.binding().unwrap();
        assert_eq!((binding.kind(), binding.function(), binding.qualifier()), (CellKind::Own, "plan2cell-cell-control", Some("ws-jdoe")));
        assert_eq!(Settings::default().binding(), None);
        assert!(valid_qualifier("ws-jdoe") && !valid_qualifier("123") && !valid_qualifier("ws jdoe") && !valid_qualifier(""));
        // A bad qualifier: unavailable, never the unqualified function.
        let recorder = Arc::new(Recorder::default());
        let store = Arc::new(ConfigStore::open(std::env::temp_dir().join(format!("factoryos-infra-q-{}.json", std::process::id()))));
        let hardware = HardwareService::with(store.clone(), None, PathBuf::from("/nonexistent/cell.toml"));
        let service = InfraService::with(recorder, Settings { qualifier: Some("ws jdoe".into()), ..shared }, store, hardware, None);
        let status = service.status();
        assert!(!status.available && status.message.unwrap().contains("QUALIFIER"));
        // The Lambda's errors as HTTP.
        let e = |code: &str| cell_error("x", &json!({"ok": false, "error": {"code": code, "message": "m"}})).0;
        assert_eq!(
            [e("no-cell"), e("exists"), e("cap-reached"), e("confirm-mismatch"), e("capacity"), e("unqualified"), e("boom")],
            [
                StatusCode::NOT_FOUND,
                StatusCode::CONFLICT,
                StatusCode::TOO_MANY_REQUESTS,
                StatusCode::BAD_REQUEST,
                StatusCode::SERVICE_UNAVAILABLE,
                StatusCode::FORBIDDEN,
                StatusCode::BAD_GATEWAY
            ]
        );
    }

    #[tokio::test]
    async fn a_shared_binding_lists_no_own_cells() {
        let f = fixture("shared-cells", Some("plan2cell-cell-control"));
        let list = f.service.cells().await;
        assert_eq!((list.available, list.can_create, list.shared.as_deref()), (false, false, Some("plan2cell")));
        assert_eq!(f.service.create_cell(CreateCellRequest::default()).await.unwrap_err().0, StatusCode::CONFLICT);
        assert_eq!(f.service.status().binding, Some(CellKind::Shared));
        assert!(!calls(&f, "aws.calls").iter().any(|c| c.contains("--qualifier")));
        let none = fixture("no-cells", None);
        assert!(!none.service.cells().await.available);
    }

    // --- cell templates (docs/tasks/cell-templates.md §13) ---

    /// The Lambda's `templates` (§8): the two legacy templates and one that needs a wiring level beyond this backend's.
    const TEMPLATES: &str = r#"{"ok":true,"action":"templates","engine":1,"enabled":true,"revision":1,"source":"s3","templates":[
        {"id":"ec2-codesys-kuka","version":"1.0.0","channel":"stable","source":"s3","title":"EC2","runtime":"ec2","status":"active",
         "parameters":{"robots":{"type":"integer","min":1,"max":3,"default":2,"mutable":"off","label":"Robots"}},
         "capabilities":[{"name":"smartpad"}],"consumer":{"deploy":{"stages":["projects","deploy","krl","start"]},"ops":[]},
         "cost":{"runningUsdPerHour":2.65,"note":"n"},"agentNotes":"a","available":true},
        {"id":"eks-codesys-kuka","version":"1.0.0","channel":"stable","source":"s3","runtime":"eks","available":false,"reason":"runtime"},
        {"id":"far-future","version":"2.0.0","runtime":"eks","requires":{"engine":1,"kinds":[],"consumer":4},"available":true},
        {"id":"unreadable","parameters":"not an object"}]}"#;

    /// `cell_status.template` of a template with none of the optional capabilities and an operation of its own.
    const BARE_TEMPLATE: &str = r#"{"id":"bare-cell","version":"0.1.0","sha256":"ab","source":"s3","parameters":{},
        "capabilities":["deploy.head.codesys-mcp","deploy.robot.kss","live.head.opcua"],
        "consumer":{"deploy":{"stages":["projects","deploy","krl","start"]},"ops":[{"name":"status","label":"Status","destructive":false}]},
        "semantics":{"plcHost":"head-plc","kukaVc1":"robot-controller","windowsIde":"compile"}}"#;

    fn payloads(f: &Fixture, action: &str) -> Vec<Value> {
        calls(f, "own.calls").iter().filter(|c| c.contains(&format!(r#""action":"{action}""#))).map(|c| serde_json::from_str(c).unwrap()).collect()
    }

    fn forget_templates(f: &Fixture) {
        f.service.inner.lock().unwrap().templates = None;
    }

    #[tokio::test]
    async fn own_cell_templates_with_an_older_and_a_new_lambda() {
        let f = own_fixture("templates");
        // An older Lambda (no templates): none listed, the runtimes as before; the legacy ids mapped by this backend.
        let t = f.service.templates().await;
        assert!(!t.available && t.templates.is_empty() && t.reason.as_deref().is_some_and(|r| r.contains("older")), "{t:#?}");
        let list = f.service.cells().await;
        assert!(list.templates.is_empty() && !list.templates_enabled);
        assert!(!serde_json::to_string(&list).unwrap().contains("templates"), "an older viewer's list, as before");
        let params = |v: Value| Some(v.as_object().unwrap().clone());
        let ec2 = CreateCellRequest { template: Some(TemplateSelector::Id("ec2-codesys-kuka".into())), params: params(json!({"robots": 3})), ..Default::default() };
        f.service.create_cell(ec2).await.unwrap();
        let sent = payloads(&f, "cell_create").pop().unwrap();
        assert_eq!(sent, json!({"action": "cell_create", "runtime": "ec2", "robots": 3}), "today's fields");
        std::fs::remove_file(f.root.join("cell")).unwrap();
        let eks = CreateCellRequest {
            template: Some(TemplateSelector::Pinned { id: "eks-codesys-kuka".into(), version: "1.0.0".into() }),
            params: params(json!({"networks": [{"name": "n1", "robots": 4}], "ide": "ec2"})),
            ..Default::default()
        };
        f.service.create_cell(eks).await.unwrap();
        let sent = payloads(&f, "cell_create").pop().unwrap();
        assert_eq!(sent, json!({"action": "cell_create", "runtime": "eks", "ide": "ec2", "networks": [{"name": "n1", "robots": 4}]}));
        std::fs::remove_file(f.root.join("cell")).unwrap();
        let other = |id: &str, version: Option<&str>| CreateCellRequest {
            template: Some(match version {
                Some(v) => TemplateSelector::Pinned { id: id.into(), version: v.into() },
                None => TemplateSelector::Id(id.into()),
            }),
            ..Default::default()
        };
        assert_eq!(f.service.create_cell(other("krl-sim", None)).await.unwrap_err().0, StatusCode::NOT_IMPLEMENTED);
        assert_eq!(f.service.create_cell(other("ec2-codesys-kuka", Some("1.1.0"))).await.unwrap_err().0, StatusCode::NOT_IMPLEMENTED);
        let unknown = CreateCellRequest { params: params(json!({"robots": 2, "ide": "ec2"})), ..other("ec2-codesys-kuka", None) };
        assert_eq!(f.service.create_cell(unknown).await.unwrap_err().0, StatusCode::BAD_REQUEST, "ide is not ec2-codesys-kuka's");
        let bad = CreateCellRequest { params: params(json!({"robots": 9})), ..other("ec2-codesys-kuka", None) };
        assert_eq!(f.service.create_cell(bad).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        // Shape only, before any call: legacy fields beside a template, odd ids, versions and names.
        let mixed = CreateCellRequest { robots: Some(2), ..other("ec2-codesys-kuka", None) };
        let (code, message) = f.service.create_cell(mixed).await.unwrap_err();
        assert_eq!((code, message.as_str()), (StatusCode::BAD_REQUEST, TEMPLATE_AND_LEGACY_FIELDS));
        for bad in [other("EC2", None), other("ec2-codesys-kuka", Some("1.0")), CreateCellRequest { params: params(json!({"__proto__": 1})), ..other("ec2-codesys-kuka", None) }] {
            assert_eq!(f.service.create_cell(bad).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        }
        assert_eq!(payloads(&f, "cell_create").len(), 2);

        // A new Lambda: its list (an entry this backend cannot read left out; one beyond its wiring level unavailable).
        std::fs::write(f.root.join("templates"), TEMPLATES).unwrap();
        forget_templates(&f);
        let t = f.service.templates().await;
        assert!(t.available && t.enabled && t.revision == Some(1), "{t:#?}");
        assert_eq!(t.templates.iter().map(|t| (t.id.as_str(), t.available, t.reason.as_deref())).collect::<Vec<_>>(), [
            ("ec2-codesys-kuka", true, None),
            ("eks-codesys-kuka", false, Some("runtime")),
            ("far-future", false, Some("consumer")),
        ]);
        let list = f.service.cells().await;
        assert_eq!((list.templates.len(), list.templates_enabled), (3, true));
        // Passed through as given (the Lambda is authoritative); a template beyond this backend is 501.
        let ec2 = CreateCellRequest { params: params(json!({"robots": 3})), ..other("ec2-codesys-kuka", None) };
        f.service.create_cell(ec2).await.unwrap();
        let sent = payloads(&f, "cell_create").pop().unwrap();
        assert_eq!(sent, json!({"action": "cell_create", "template": "ec2-codesys-kuka", "params": {"robots": 3}}));
        assert_eq!(f.service.create_cell(other("far-future", None)).await.unwrap_err().0, StatusCode::NOT_IMPLEMENTED);
        // A legacy request still goes as today.
        std::fs::remove_file(f.root.join("cell")).unwrap();
        f.service.create_cell(CreateCellRequest::default()).await.unwrap();
        assert_eq!(payloads(&f, "cell_create").pop().unwrap(), json!({"action": "cell_create", "robots": 2}));
        // Update: params and a template, through; the same refusals.
        let update = UpdateCellRequest { params: params(json!({"offMode": "stop"})), ..Default::default() };
        f.service.update_cell("ws-jdoe", update).await.unwrap();
        assert_eq!(payloads(&f, "cell_update").pop().unwrap(), json!({"action": "cell_update", "params": {"offMode": "stop"}}));
        let switch = UpdateCellRequest { template: Some(TemplateSelector::Id("eks-codesys-kuka".into())), ..Default::default() };
        f.service.update_cell("ws-jdoe", switch).await.unwrap();
        assert_eq!(payloads(&f, "cell_update").pop().unwrap(), json!({"action": "cell_update", "template": "eks-codesys-kuka"}));
        let mixed = UpdateCellRequest { robots: Some(3), params: params(json!({"robots": 3})), ..Default::default() };
        assert_eq!(f.service.update_cell("ws-jdoe", mixed).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        let empty = UpdateCellRequest { params: params(json!({})), ..Default::default() };
        assert_eq!(f.service.update_cell("ws-jdoe", empty).await.unwrap_err().0, StatusCode::BAD_REQUEST);
        // The template errors, with the Lambda's message.
        for (code, status) in [
            ("template-unavailable", StatusCode::SERVICE_UNAVAILABLE),
            ("template-invalid", StatusCode::BAD_REQUEST),
            ("template-engine", StatusCode::NOT_IMPLEMENTED),
        ] {
            std::fs::write(f.root.join("create-fail"), code).unwrap();
            let (got, message) = f.service.update_cell("ws-jdoe", UpdateCellRequest { template: Some(TemplateSelector::Id("ec2-codesys-kuka".into())), ..Default::default() }).await.unwrap_err();
            assert_eq!((got, message.contains("the Lambda's words")), (status, true), "{code}: {message}");
        }
        // A shared binding or none: no templates.
        let shared = fixture("shared-templates", Some("plan2cell-cell-control"));
        assert!(!shared.service.templates().await.available);
    }

    #[tokio::test]
    async fn own_cell_capabilities_gate_its_features() {
        let f = own_fixture("gates");
        std::fs::write(f.root.join("cell"), "").unwrap();
        std::fs::write(f.root.join("launch"), "1").unwrap();
        cell_phase(&f, "ready");
        let kvc = |op: &str, arg: Option<&str>| KvcRequest { robot: "robot1".into(), op: op.into(), arg: arg.map(str::to_owned) };
        // No template (an older Lambda): today's behaviour.
        f.service.refresh().await;
        let cell = f.service.status().virtual_cell.unwrap();
        assert!(cell.template.is_none() && cell.has_dcv());
        assert_eq!(f.service.smartpad_refusal(), None);
        assert_eq!(f.service.kvc_changes(&kvc("format", None)), Ok(true), "today's list passes it to plan2cell");
        assert_eq!(f.service.windows_ide_address().await.as_deref(), Some("10.0.1.20"));
        assert!(demo_note(&f.service.status(), false).contains("demo mode"));

        // The legacy template (as the Lambda reports it with templates on): the same.
        let mut legacy: Value = serde_json::from_str(BARE_TEMPLATE).unwrap();
        legacy["capabilities"] = json!(factoryos_protocol::infra::legacy_capabilities("ec2", None));
        legacy["consumer"]["ops"] = json!([{"name": "status", "label": "Status", "destructive": false},
            {"name": "mode", "label": "Mode", "argSchema": {"required": true, "values": ["show", "T1"]}, "destructive": true, "readArgs": ["show"]}]);
        std::fs::write(f.root.join("template"), legacy.to_string()).unwrap();
        f.service.refresh().await;
        assert!(f.service.status().virtual_cell.unwrap().template.is_some());
        assert_eq!(f.service.smartpad_refusal(), None);
        assert_eq!(f.service.windows_ide_address().await.as_deref(), Some("10.0.1.20"));
        assert_eq!((f.service.kvc_changes(&kvc("mode", Some("show"))), f.service.kvc_changes(&kvc("mode", Some("T1")))), (Ok(false), Ok(true)));
        assert_eq!(f.service.kvc_changes(&kvc("format", None)).unwrap_err().0, StatusCode::BAD_REQUEST, "not one of the template's ops");
        assert!(demo_note(&f.service.status(), false).contains("demo mode"));

        // A template without smartPADs, DCV, bring-up, restore or CODESYS demo runtimes.
        std::fs::write(f.root.join("template"), BARE_TEMPLATE).unwrap();
        f.service.refresh().await;
        let cell = f.service.status().virtual_cell.unwrap();
        assert!(!cell.has_dcv() && !cell.has("smartpad"));
        assert!(f.service.smartpad_refusal().unwrap().contains("bare-cell@0.1.0 has no smartpad"));
        assert_eq!(f.service.windows_ide_address().await, None);
        let (code, message) = f.service.bringup(BringupRequest::default()).await.unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST, "{message}");
        assert!(message.contains("bringup.action"), "{message}");
        assert!(f.service.inner.lock().unwrap().job.is_none(), "refused before it claims the cell");
        assert!(demo_note(&f.service.status(), false).contains("no runtime.codesys"));
        f.service.set_auto_restart(AutoRestartSettings { enabled: true, threshold_s: None }).unwrap();
        f.service.inner.lock().unwrap().interest = Some(Instant::now());
        f.service.runtime_tick().await;
        assert!(f.service.status().auto_restart.inactive.unwrap_or_default().contains("runtime.codesys"));
        assert!(calls(&f, "runtime.calls").is_empty(), "no runtime to read");
        // Its fresh launch is restored as today's, `restore` or not (the eks-codesys-kuka template has
        // none, and EKS cells are restored today); the wiring is complete by its capabilities. Launch 1 was
        // already followed by the first refresh (nothing deployed then): a new launch restores again.
        std::fs::write(f.root.join("launch"), "2").unwrap();
        export(&f);
        std::fs::write(f.root.join("last-build"), "20261012T080000000Z-bt_a").unwrap();
        let restore = restored(&f).await;
        assert_eq!(restore.mode.as_str(), "full", "{restore:#?}");
    }

    /// `cell_status.template` of an engine-2 template (docs/tasks/cell-templates.md E12): a hook head and hook robots.
    const HOOK_TEMPLATE: &str = r#"{"id":"eks-generic-smoke","version":"0.1.0","sha256":"cd","source":"s3","channel":"preview","parameters":{},
        "capabilities":["check.hook","deploy.head.hook","deploy.robot.hook","live.head.exec","live.robot.exec","runtime.hook"],
        "consumer":{"deploy":{"stages":["deploy","krl","start"]},"ops":[{"name":"status","label":"Status","destructive":false}]},
        "semantics":{"n1/head":"head-plc","n1/robot-1":"robot-controller"}}"#;

    /// Its wiring (`factoryos/wiring@2`, E8): no projection, the pods' exec targets.
    const HOOK_WIRING: &str = r#"{"schema":"factoryos/wiring@2","launch":1,"runtime":"eks","namespace":"cell-jdoe",
        "profinet":{"cidr":"10.77.1.0/24","gateway":"10.77.1.1","head":"10.77.1.10","vni":1101},"addressMap":{},
        "networks":[{"name":"n1","vni":1101,"profinet":{"cidr":"10.77.1.0/24","gateway":"10.77.1.1","head":"10.77.1.10","vni":1101},"addressMap":{}}],
        "kubernetes":{"cluster":"factoryos-cells","endpoint":"https://ABC123.gr7.eu-central-1.eks.amazonaws.com","ca":"Q0E="},
        "components":[
          {"name":"n1/head","semantic":"head-plc","driver":"hook","network":"n1","address":"10.77.1.10","kubectlPod":"cell-jdoe/n1-head/head","hookContainer":"head","ip":"172.31.60.11"},
          {"name":"n1/robot-1","semantic":"robot-controller","driver":"hook","network":"n1","robotNs":"robot1","station":"kuka-robi1","address":"10.77.1.11",
           "kubectlPod":"cell-jdoe/n1-robot-1/robot","hookContainer":"robot","ip":"172.31.60.21"}]}"#;

    /// An engine-2 cell (E7-E9): its wiring is written with hook drivers, Run Control Code runs the template's
    /// stages only and reads no CODESYS MCP token, the head is found by its semantic.
    #[tokio::test]
    async fn own_cell_hook_template_runs_its_stages_without_codesys() {
        let f = own_fixture("hook");
        std::fs::write(f.root.join("cell"), "").unwrap();
        std::fs::write(f.root.join("launch"), "1").unwrap();
        std::fs::write(f.root.join("hook-cell"), "").unwrap();
        std::fs::write(f.root.join("template"), HOOK_TEMPLATE).unwrap();
        std::fs::write(f.root.join("wiring"), HOOK_WIRING).unwrap();
        cell_phase(&f, "ready");
        let dir = build(&f, "bt_a", "20261012T080000000Z", "16#0001");
        f.service.refresh().await;
        restored(&f).await;
        let cell = f.service.status().virtual_cell.unwrap();
        assert!(cell.template.as_ref().unwrap().hook() && !cell.has("smartpad"));
        let wire = f.root.join("wire");
        let toml: toml::Table = std::fs::read_to_string(wire.join(cell_wiring::CELL_TOML)).unwrap().parse().unwrap();
        assert_eq!((toml["plc_host"]["driver"].as_str(), toml["plc_host"]["kubectl_pod"].as_str()), (Some("hook"), Some("cell-jdoe/n1-head/head")));
        assert_eq!(toml["kuka_vc"]["controller"][0]["driver"].as_str(), Some("hook"));
        assert!(toml.get("codesys_mcp").is_none());
        let bridge: toml::Table = std::fs::read_to_string(wire.join(cell_wiring::BRIDGE_TOML)).unwrap().parse().unwrap();
        assert_eq!(bridge["opcua"]["enabled"].as_bool(), Some(false));
        assert_eq!(bridge["exec"]["head"]["pod"].as_str(), Some("cell-jdoe/n1-head"));
        assert!(!wire.join(cell_wiring::DCV_ENV).exists(), "no IDE: no DCV server");

        // A stage the template does not have: refused.
        let (code, message) = f.service.deploy(DeployRequest { stages: Some(vec!["projects".into()]), ..Default::default() }).await.unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST);
        assert!(message.contains("not a stage of this virtual cell's template (stages: deploy, krl, start)"), "{message}");
        // Deploy only: the template's stages without start, no Secrets Manager read, no PLC IDE.
        let accepted = f.service.deploy(DeployRequest { start: false, ..Default::default() }).await.unwrap();
        assert_eq!(accepted.deploy.stages, ["deploy", "krl"]);
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert!(status.deploy.message.as_deref().unwrap_or_default().contains("deployed through its hook"), "{:#?}", status.deploy);
        let shown = dir.display();
        assert_eq!(calls(&f, "stage.calls"), [format!("deploy {shown}"), format!("krl {shown}")]);
        assert!(calls(&f, "aws.calls").iter().all(|c| !c.contains("secretsmanager")), "no CODESYS MCP token");
        // Run Control Code: deploy, krl, start.
        let accepted = f.service.deploy(DeployRequest { start: true, ..Default::default() }).await.unwrap();
        assert_eq!(accepted.deploy.stages, ["deploy", "krl", "start"]);
        let status = settled(&f.service).await;
        assert_eq!(status.deploy.state, JobState::Done, "{:#?}", status.deploy);
        assert!(calls(&f, "aws.calls").iter().all(|c| !c.contains("secretsmanager")));
        // The head by its semantic (no plcHost role): here the fake Lambda reports none of that name.
        let mut status = f.service.status();
        status.instances.push(InfraInstance { role: "n1/head".into(), state: "running".into(), ..Default::default() });
        assert_eq!(plc_host(&status, false).map(|h| h.role.as_str()), Some("n1/head"), "a hook template: semantics, flag or not");
    }

    /// The semantic host lookups (FACTORY_OS_CELL_SEMANTIC_HOSTS): an EKS cell's head is `n1/plcHost`, which the
    /// role names miss (auto-restart never read its runtimes); off, or without a template, the role names.
    #[test]
    fn semantic_hosts_find_an_eks_cells_head() {
        let boot = parse_iso8601("2026-10-12T08:00:00.000Z").unwrap();
        let host = |role: &str| InfraInstance {
            role: role.into(),
            state: "running".into(),
            ssm_ping: "Online".into(),
            private_ip: Some("10.0.0.1".into()),
            launch_time: Some("2026-10-12T08:00:00.000Z".into()),
            ..Default::default()
        };
        let template: CellTemplateRef = serde_json::from_value(json!({"id": "eks-codesys-kuka", "version": "1.0.0", "capabilities": ["runtime.codesys"],
            "semantics": {"n1/plcHost": "head-plc", "n2/plcHost": "head-plc", "n1/kukaVc1": "robot-controller", "compile": "compile", "windowsIde": "ide"}})).unwrap();
        let mut status = InfraStatus {
            instances: vec![host("n1/kukaVc1"), host("n2/plcHost"), host("n1/plcHost"), host("compile"), host("windowsIde")],
            virtual_cell: Some(VirtualCell { runtime: RUNTIME_EKS.into(), template: Some(template), ..Default::default() }),
            ..Default::default()
        };
        assert_eq!(plc_host(&status, false), None, "today: no plcHost on EKS");
        assert_eq!(plc_host(&status, true).map(|h| h.role.as_str()), Some("n1/plcHost"));
        assert!(!fresh_boot(&status, false, boot + 60.0) && fresh_boot(&status, true, boot + 60.0));
        assert!(demo_note(&status, true).contains("PLC host up since 08:00 UTC") && !demo_note(&status, false).contains("up since"));
        assert_eq!(host_of(&status, true, cell_wiring::SEMANTIC_IDE, ROLE_WINDOWS_IDE).map(|h| h.role.as_str()), Some("windowsIde"));
        // An EC2 cell: the same hosts either way.
        status.instances = vec![host("plcHost"), host("windowsIde")];
        status.virtual_cell.as_mut().unwrap().template.as_mut().unwrap().semantics =
            [("plcHost".to_owned(), "head-plc".to_owned()), ("windowsIde".to_owned(), "ide".to_owned())].into();
        assert_eq!(plc_host(&status, true), plc_host(&status, false));
        status.virtual_cell.as_mut().unwrap().template = None;
        assert_eq!(plc_host(&status, true).map(|h| h.role.as_str()), Some("plcHost"), "no template: the role names");
    }

    #[test]
    fn template_errors_and_older_lambdas() {
        let e = |code: &str| cell_error("cell_create", &json!({"ok": false, "error": {"code": code, "message": "template eks-codesys-kuka@1.0.0: m"}}));
        assert_eq!(e("template-unavailable"), (StatusCode::SERVICE_UNAVAILABLE, "template eks-codesys-kuka@1.0.0: m (template-unavailable)".to_owned()));
        assert_eq!(e("template-invalid").0, StatusCode::BAD_REQUEST);
        assert_eq!(e("template-engine").0, StatusCode::NOT_IMPLEMENTED);
        assert!(e("template-engine").1.starts_with("template"), "the Lambda's message, not the deploy hint");
        let err = |code: &str, message: &str| json!({"ok": false, "error": {"code": code, "message": message}});
        assert!(older_lambda(&err("bad-request", "action must be one of cell_status, cell_create")));
        assert!(older_lambda(&err("unknown-action", "templates")));
        assert!(!older_lambda(&err("bad-request", "params.robots: 1 to 3")) && !older_lambda(&err("template-unavailable", "x")));
        // The legacy ids for an older Lambda: today's fields.
        let map = |template: Option<TemplateSelector>, params: Value| {
            let mut payload = json!({});
            legacy_template_fields(&mut payload, template.as_ref(), params.as_object()).map(|_| payload)
        };
        let id = |s: &str| Some(TemplateSelector::Id(s.into()));
        assert_eq!(map(id("ec2-codesys-kuka"), json!({"offMode": "stop"})).unwrap(), json!({"runtime": "ec2", "offMode": "stop"}));
        assert_eq!(map(id("eks-codesys-kuka"), json!({})).unwrap(), json!({"runtime": "eks"}));
        assert_eq!(map(None, json!({"robots": 1})).unwrap(), json!({"robots": 1}), "an update's params");
        assert_eq!(map(id("eks-codesys-kuka"), json!({"networks": [{"name": "n0", "robots": 1}]})).unwrap_err().0, StatusCode::BAD_REQUEST);
        assert_eq!(map(id("eks-codesys-kuka"), json!({"ide": "dcv"})).unwrap_err().0, StatusCode::BAD_REQUEST);
        assert_eq!(map(id("eks-codesys-kuka"), json!({"offMode": "pause"})).unwrap_err().0, StatusCode::BAD_REQUEST);
    }

    /// The Lambda's limits name the account's cell count `cells` (an older one: `count`).
    #[tokio::test]
    async fn the_accounts_cap_reads_the_lambdas_cell_count() {
        let f = own_fixture("limits");
        std::fs::write(f.root.join("limits"), r#"{"maxPerWorkspace":1,"max":3,"maxRunning":2,"running":1,"cells":3}"#).unwrap();
        let list = f.service.cells().await;
        assert!(!list.can_create && list.reason.as_deref().is_some_and(|r| r.contains("cap of 3")), "{list:#?}");
        std::fs::write(f.root.join("limits"), r#"{"maxPerWorkspace":1,"max":3,"maxRunning":2,"running":1,"count":3}"#).unwrap();
        assert!(!f.service.cells().await.can_create);
        std::fs::write(f.root.join("limits"), r#"{"maxPerWorkspace":1,"max":3,"maxRunning":2,"running":2}"#).unwrap();
        assert!(f.service.cells().await.can_create, "today's limits: no count, no cap");
    }
}
