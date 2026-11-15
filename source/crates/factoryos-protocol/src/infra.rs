//! The Virtual Cell: the instances of the plan2cell cell (PLC host, Windows CODESYS IDE, one KUKA
//! VC host per controller: roles `kukaVc1`, `kukaVc2`, ...), controlled through the cell-control
//! Lambda (`plan2cell-cell-control`), so the backend's machine needs `lambda:InvokeFunction` only,
//! no EC2 or SSM permissions.
//!
//! * `GET` [`crate::api::INFRA_STATUS`]: [`InfraStatus`] (also on [`crate::keys::INFRA_STATUS`],
//!   refreshed about every 10 s while a viewer asks for it or something is in progress).
//! * `POST` [`crate::api::INFRA_START`] / [`crate::api::INFRA_STOP`] ([`StartStopRequest`], optional):
//!   start / stop the instances (stop never terminates: an EC2 stop, nothing is deleted). A start
//!   is per instance ([`InfraStatus::start`]: one that cannot start never stops the others; a
//!   stopped instance without capacity for its type starts as a fallback type of its role,
//!   [`InstanceStart::new_type`]); a start of the whole cell then waits until every instance runs
//!   and is SSM-online, brings up the controllers and, if the cell was in production when it was
//!   stopped (or [`StartStopRequest::production`]), starts production: one click to a producing cell.
//! * `POST` [`crate::api::INFRA_BRINGUP`] ([`BringupRequest`], optional): `202`; waits until every
//!   instance runs and is SSM-online, then every KUKA VC host's health check / repair and the
//!   acknowledgement of its stop messages ([`InfraStatus::bringup`]).
//! * `POST` [`crate::api::INFRA_PRODUCTION`]: `202`; `plan2cell cell start <export>
//!   --restart-runtime` (fresh CODESYS demo runtimes, KUKA VCs into EXT, reset pulses until the
//!   head runs; [`InfraStatus::cell`]) on the export the head runs ([`InfraStatus::head_export`]:
//!   of the PLAN_ID it runs, else the export last deployed, else the newest; a refusal of the
//!   head's PLAN_ID guard is retried once on the newest export of the PLAN_ID it names), or on
//!   [`ProductionRequest::build`]. One job (bring-up, production, deploy) at a time (`409`).
//! * `POST` [`crate::api::INFRA_DEPLOY`] ([`DeployRequest`], optional): `202`; Run Control Code:
//!   `plan2cell cell projects`, `deploy`, `krl` (cells with `[kuka_vc]`), then `start
//!   --restart-runtime --pulses 16` on an export (default: the newest; [`InfraStatus::deploy`]).
//!   Its PLAN_ID is then the one the head runs ([`AutoRestartStatus::deployed_plan_id`]). With
//!   `start: false` (Deploy only) without the `start` stage: the head runs the new application but
//!   is not in production ([`InfraStatus::in_production`]) until Start production.
//! * [`InfraStatus::plc_runtime`]: the CODESYS demo runtimes (`plan2cell cell runtime --json`, about
//!   every minute while a viewer asks or auto-restart is on).
//! * `GET` / `PUT` [`crate::api::INFRA_AUTO_RESTART`] ([`AutoRestartSettings`]): [`AutoRestartStatus`],
//!   a production start with `matchDeployed` shortly before the 2 h demo stop (or when the head's
//!   runtime stopped while in production); one job at a time, one attempt per 10 min.
//!
//! * `POST` [`crate::api::INFRA_DCV_SESSION`] ([`DcvSessionRequest`], optional): [`DcvSession`], the
//!   Windows IDE's CODESYS in the browser: the backend's [`crate::api::DCV`]`/<target>/` proxy to
//!   its DCV Connection Gateway, with a token its verifier accepts, so no Windows password
//!   ([`InfraStatus::dcv_gateway`], [`InfraStatus::dcv_targets`]; target [`DCV_TARGET_IDE`]).
//! * One cell, several backends: exactly one is the cell's [`CellRole::Supervisor`] (auto-restart,
//!   the production resume after Start cell); the others are [`CellRole::Operator`]s. User actions
//!   on any of them hold the cell's lease in the cell-control Lambda ([`InfraStatus::lease`]):
//!   another backend's action meanwhile is `409 cell busy`.
//!
//! * `GET` / `POST` [`crate::api::INFRA_CELL_POWER`] ([`CellPowerRequest`]): the power of the whole
//!   virtual cell ([`InfraStatus::cell_power`], [`CellPower`]): Start virtual cell (`on`: the Lambda
//!   starts every instance, waits for the boot, brings up the KUKA controllers and checks the PLC
//!   runtime, the routing, DCV and the MCP bridge; the phases in [`CellPower::phase`]), Stop virtual
//!   cell (`off`; refused in production unless `force`). An idle cell may be stopped by the Lambda
//!   (its idle stop, from the backends' `touch`es). Auto-restart pauses while it is off or booting.
//!
//! `503` when the backend has no cell-control function (`FACTORY_OS_CELL_CONTROL_FUNCTION`).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// The Windows IDE's `Administrator` password (never fetched by the backend or shown).
pub const WINDOWS_ADMIN_SECRET: &str = "plan2cell/windows-admin";
/// The Windows IDE's automatic DCV console session (`create-session`, owner `Administrator`).
pub const DCV_CONSOLE_SESSION: &str = "console";
/// The DCV target of the Windows IDE's console (`/dcv/ide/`): the default target.
pub const DCV_TARGET_IDE: &str = "ide";
/// How long a CODESYS runtime runs without a licence (demo mode).
pub const CODESYS_DEMO_S: f64 = 2.0 * 3600.0;

/// The instance roles, as the Lambda names them. The KUKA VC hosts are `kukaVc1`, `kukaVc2`, ...
/// (one per controller); [`ROLE_KUKA_VC`] alone (an older Lambda's single host, or in a start /
/// stop request: every KUKA VC host).
pub const ROLE_PLC_HOST: &str = "plcHost";
pub const ROLE_WINDOWS_IDE: &str = "windowsIde";
pub const ROLE_KUKA_VC: &str = "kukaVc";
/// The roles shown before the first status (and accepted in every start / stop request).
pub const ROLES: [&str; 3] = [ROLE_PLC_HOST, ROLE_WINDOWS_IDE, ROLE_KUKA_VC];

/// `kukaVc`, `kukaVc1`, `kukaVc2`, ...: a KUKA VC host.
pub fn is_kuka_vc(role: &str) -> bool {
    role.strip_prefix(ROLE_KUKA_VC).is_some_and(|n| n.chars().all(|c| c.is_ascii_digit()))
}

/// A role a start / stop request may name: [`ROLES`] or a `kukaVcN`.
pub fn known_role(role: &str) -> bool {
    ROLES.contains(&role) || is_kuka_vc(role)
}

/// `plcHost` → `PLC host`, `kukaVc2` → `KUKA VC host 2`, ...
pub fn role_label(role: &str) -> String {
    match role {
        ROLE_PLC_HOST => "PLC host".to_owned(),
        ROLE_WINDOWS_IDE => "Windows IDE".to_owned(),
        ROLE_KUKA_VC => "KUKA VC host".to_owned(),
        r if is_kuka_vc(r) => format!("KUKA VC host {}", &r[ROLE_KUKA_VC.len()..]),
        other => other.to_owned(),
    }
}

/// One instance of the cell.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct InfraInstance {
    /// [`ROLE_PLC_HOST`], [`ROLE_WINDOWS_IDE`] or a KUKA VC host (`kukaVc1`, ...; [`is_kuka_vc`]).
    pub role: String,
    pub instance_id: String,
    /// A KUKA VC host's robot (`robot1`, ...).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_ns: Option<String>,
    pub name: Option<String>,
    /// EC2: `pending`, `running`, `stopping`, `stopped`, ... (`not-found`, `unknown`).
    pub state: String,
    /// SSM: `Online`, `ConnectionLost`, `Inactive`, `Unknown`.
    pub ssm_ping: String,
    pub private_ip: Option<String>,
    pub public_ip: Option<String>,
    /// ISO 8601.
    pub launch_time: Option<String>,
    /// EC2 instance type (`c5.4xlarge`, ...); a fallback start may have changed it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instance_type: Option<String>,
}

impl InfraInstance {
    pub fn running(&self) -> bool {
        self.state == "running"
    }

    pub fn online(&self) -> bool {
        self.ssm_ping == "Online"
    }

    /// Between states (`pending`, `stopping`, `shutting-down`): no job can use it now.
    pub fn changing(&self) -> bool {
        matches!(self.state.as_str(), "pending" | "stopping" | "shutting-down")
    }
}

/// One instance of a start (the Lambda's per-instance result).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct InstanceStart {
    pub role: String,
    pub instance_id: String,
    /// Its instance type before the start.
    pub previous_type: Option<String>,
    /// The fallback type it started as (its own had no capacity); it keeps that type.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub new_type: Option<String>,
    /// `started`, `fallback` (started as `newType`), `failed`, `skipped` (no such instance).
    pub result: String,
    /// Why it failed: `InsufficientInstanceCapacity: ...`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// The error that made it fall back (or fail): `InsufficientInstanceCapacity`, ...
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_error: Option<String>,
}

impl InstanceStart {
    pub fn failed(&self) -> bool {
        self.result == "failed"
    }

    /// `Windows IDE: c5a.4xlarge had no capacity, started as c5.4xlarge`, `PLC host: started`, ...
    pub fn summary(&self) -> String {
        let who = role_label(&self.role);
        let own = self.previous_type.as_deref().unwrap_or("its type");
        let why = match self.first_error.as_deref() {
            Some("InsufficientInstanceCapacity") | None => "had no capacity",
            Some("Unsupported") => "is not offered in its zone",
            Some(_) => "did not start",
        };
        match self.result.as_str() {
            "fallback" => format!("{who}: {own} {why}, started as {}", self.new_type.as_deref().unwrap_or("another type")),
            "failed" => format!("{who}: did not start ({own}): {}", self.error.as_deref().unwrap_or("unknown error")),
            "skipped" => format!("{who}: not found (skipped)"),
            _ => format!("{who}: started ({own})"),
        }
    }
}

/// Start cell ([`crate::api::INFRA_START`]): the per-instance start, then (the whole cell) the wait
/// until every instance runs and is SSM-online, the bring-up and, if asked, Start production.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellStartStatus {
    /// `Waiting` for the instances, `Running` the bring-up / production start, `Done`, `Failed`.
    pub state: JobState,
    /// The step now (or the outcome), with per-instance progress while waiting.
    pub message: Option<String>,
    /// What the start of each instance did.
    pub instances: Vec<InstanceStart>,
    /// The bring-up follows once every instance is ready.
    pub bring_up: bool,
    /// Start production follows the bring-up (asked for, or the cell was in production when it was stopped).
    pub production: bool,
    pub started_s: Option<f64>,
    pub finished_s: Option<f64>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JobState {
    #[default]
    Idle,
    /// Waiting for the instances (running and SSM-online).
    Waiting,
    Running,
    Done,
    Failed,
}

impl JobState {
    pub fn busy(self) -> bool {
        matches!(self, Self::Waiting | Self::Running)
    }
}

/// The KUKA VC bring-up (the Lambda's `bringup` on the VC host).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BringupStatus {
    pub state: JobState,
    pub message: Option<String>,
    /// The SSM command.
    pub command_id: Option<String>,
    /// `healthy`, `repaired`, `failed` or `busy` (the script's `BRINGUP_RESULT`), or `ok` for exit 0 without it.
    pub result: Option<String>,
    /// The end of its output.
    pub output_tail: Option<String>,
    pub started_s: Option<f64>,
}

/// The cell's production (`plan2cell cell start`) and its CODESYS demo period.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellRunStatus {
    pub state: JobState,
    pub message: Option<String>,
    /// The export it runs (a build id of the active cell).
    pub build: Option<String>,
    /// The head PLC's state the last start read (`CS_RUNNING`, ...).
    pub head_state: Option<String>,
    /// When the runtimes were last restarted by this backend (Unix seconds).
    pub runtime_started_s: Option<f64>,
    /// The CODESYS demo period, for the status line.
    pub demo_note: Option<String>,
    pub started_s: Option<f64>,
}

/// The CODESYS demo runtimes of the PLC host, from `plan2cell cell runtime --json` (read about
/// every minute while a viewer shows the Virtual Cell or auto-restart is on).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PlcRuntime {
    /// The head PLC's runtime (`codesys-master`) runs and its demo period is not over.
    pub running: bool,
    /// When the head's container started (Unix seconds).
    pub started_s: Option<f64>,
    /// The head container's docker `RestartCount` (restarts by its restart policy: a stop of its
    /// own; `docker restart` does not count).
    pub restart_count: Option<u64>,
    /// When the first running runtime's demo period ends (an estimate, see `source`), and how
    /// long until then as read (`0`: over).
    pub stops_s: Option<f64>,
    pub seconds_left: Option<f64>,
    /// What `stopsS` comes from: `shutdown warning`, `demo line`, `container start`, ...
    pub source: Option<String>,
    /// The container that stops first.
    pub container: Option<String>,
    /// Its latest `performing shutdown in ...` log line.
    pub warning: Option<String>,
    /// Every runtime container of the export in production (head and robot simulators).
    pub runtimes: Vec<RuntimeBrief>,
    /// The export whose containers were read (its build id), if one is known.
    pub build: Option<String>,
    /// When it was read (Unix seconds).
    pub checked_s: f64,
    /// Why it could not be read (the rest is then the previous read's).
    pub error: Option<String>,
}

/// One runtime container of [`PlcRuntime`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct RuntimeBrief {
    pub container: String,
    pub head: bool,
    pub running: bool,
    pub stops_s: Option<f64>,
}

/// Restart the CODESYS runtimes before the demo stop (or when the head's runtime stopped while
/// the cell was in production), as [`crate::api::INFRA_PRODUCTION`] with `matchDeployed`.
/// Kept per cell (`plan2cell/auto-restart.json` of the cell).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct AutoRestartStatus {
    pub enabled: bool,
    /// Restart when the first runtime stops in less than this (only a cell in production).
    pub threshold_s: u64,
    /// The last attempt (Unix seconds); the next one not before `nextAllowedS` (one per 10 min).
    pub last_attempt_s: Option<f64>,
    pub next_allowed_s: Option<f64>,
    /// The last attempt's outcome (`CS_RUNNING`, ...) or error.
    pub last_result: Option<String>,
    pub last_error: Option<String>,
    /// Auto-restart holds (e.g. the head refused the export: PLAN_ID guard) until it is switched
    /// on again or a production start succeeds.
    pub blocked: Option<String>,
    /// The PLAN_ID the head runs, as the last production start found it.
    pub deployed_plan_id: Option<String>,
    /// Failed attempts in a row (3 hold it).
    pub failures: u32,
    /// Why it cannot act now although it is on (the runtimes cannot be read, ...); derived.
    pub inactive: Option<String>,
    /// The status line, e.g. `auto-restart at 14:02 UTC: CS_RUNNING`.
    pub message: Option<String>,
}

/// Auto-restart holds after this many failed attempts in a row.
pub const AUTO_RESTART_MAX_FAILURES: u32 = 3;

/// `PUT` [`crate::api::INFRA_AUTO_RESTART`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct AutoRestartSettings {
    pub enabled: bool,
    /// Default (`None` / `0`): [`AUTO_RESTART_THRESHOLD_S`].
    pub threshold_s: Option<u64>,
}

/// Auto-restart's default margin before the demo stop.
pub const AUTO_RESTART_THRESHOLD_S: u64 = 300;
/// At most one auto-restart attempt this often.
pub const AUTO_RESTART_BACKOFF_S: f64 = 600.0;

/// `POST` [`crate::api::INFRA_PRODUCTION`] (optional body).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct ProductionRequest {
    /// Restart PLC runtime (auto-restart sends it too). The export the head runs is the default
    /// now (see [`InfraStatus::head_export`]); kept for older clients.
    pub match_deployed: bool,
    /// This export (a build id of the active cell) instead, by choice: a refusal of the head's
    /// PLAN_ID guard is not retried with another export.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
}

/// `POST` [`crate::api::INFRA_DEPLOY`] (optional body).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployRequest {
    /// The export to deploy (a build id of the active cell); default: the newest successful one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Put the cell into production afterwards (the `start` stage; default `true`: Run Control
    /// Code). `false`: Deploy only (`projects`, `deploy`, `krl`): the head PLC and the robots hold
    /// the new code, their applications run, and the head waits in `CS_INIT` / `CS_RECOVERY` for
    /// the operator pulses Start production sends: deployed, not producing.
    #[serde(skip_serializing_if = "is_true")]
    pub start: bool,
    /// Only these stages of [`DEPLOY_STAGES`], in that order (the restore after a fresh launch of a
    /// workspace's own cell: `["krl", "start"]` when the head already runs the export's PLAN_ID);
    /// `None`: as [`Self::start`] says. An unknown stage is `400`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stages: Option<Vec<String>>,
    /// Deploy anyway: without the deployability check first ([`crate::api::INFRA_DEPLOY_CHECK`];
    /// a failing rule is `409` [`crate::deploy_check::DeployRefused`] otherwise).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub force: bool,
}

impl Default for DeployRequest {
    fn default() -> Self {
        Self { build: None, start: true, stages: None, force: false }
    }
}

fn is_true(b: &bool) -> bool {
    *b
}

/// The stages of Run Control Code, in order (`krl` only for a cell with `[kuka_vc]`; no `start`
/// for Deploy only, [`DeployRequest::start`] `false`).
pub const DEPLOY_STAGES: [&str; 4] = ["projects", "deploy", "krl", "start"];

/// What a stage of Run Control Code does, for the status line.
pub fn deploy_stage_label(stage: &str) -> &'static str {
    match stage {
        "projects" => "creating and building the CODESYS projects (about 15 min)",
        "deploy" => "downloading to the head PLC and the robot PLCs, starting them",
        "krl" => "putting the KRL onto the KUKA virtual controllers",
        "start" => "restarting the CODESYS runtimes, KUKA VCs into EXT, pulsing the head into production",
        _ => "",
    }
}

/// Run Control Code ([`crate::api::INFRA_DEPLOY`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployStatus {
    pub state: JobState,
    /// The stage running now, or the one that failed (`projects`, `deploy`, `krl`, `start`).
    pub stage: Option<String>,
    /// The stages this run has, in order, and those that passed.
    pub stages: Vec<String>,
    pub done: Vec<String>,
    pub message: Option<String>,
    /// The export (a build id of the active cell) and its PLAN_ID.
    pub build: Option<String>,
    pub plan_id: Option<String>,
    pub started_s: Option<f64>,
    pub finished_s: Option<f64>,
    /// Whether the `deploy` stage left the head project open in the PLC IDE (plan2cell's
    /// best-effort `open_project` at its end); `None`: not reported (no deploy stage passed in
    /// this run, or a plan2cell that does not open it).
    pub ide_opened: Option<bool>,
    /// The stage's note on it: `<name> open in the PLC IDE (<path>)`, `<name> not opened in the
    /// PLC IDE: <why>`.
    pub ide_note: Option<String>,
}

impl DeployStatus {
    /// The run ends with the `start` stage (Run Control Code), not Deploy only.
    pub fn starts(&self) -> bool {
        self.stages.iter().any(|s| s == "start")
    }
}

/// The head PLC's CODESYS project as the Windows IDE has it: the project the deploy stages leave
/// open in the CODESYS the Codesys-MCP drives (visible over NICE DCV, window title `<file> -
/// CODESYS ... [MCP]`).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct IdeProject {
    /// The project name (`HeadPLC_<cell>`, from the export's `plc/project.json`).
    pub name: String,
    /// Its file on the Windows IDE (`<windows_project_dir>\<file>`).
    pub path: String,
}

/// An export of the active cell and its PLAN_ID.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct ExportRef {
    /// Its build id.
    pub build: String,
    pub plan_id: Option<String>,
    /// How it was chosen: `deployed` (of the PLAN_ID the head runs), `report` (the export last
    /// deployed, by its `plan2cell-cell-report.json`), `newest`.
    pub source: String,
}

/// `GET /api/v1/infra/status`, and on `factoryos/infra/status`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct InfraStatus {
    /// The backend has a cell-control function.
    pub available: bool,
    /// Why it is unavailable, or the last error talking to it.
    pub message: Option<String>,
    pub function: Option<String>,
    pub instances: Vec<InfraInstance>,
    pub all_running: bool,
    /// Every instance running and SSM-online.
    pub ready: bool,
    /// The Windows IDE's NICE DCV web client (`https://<host>:8443/`), while it runs.
    pub dcv_url: Option<String>,
    /// The backend serves the Windows IDE's DCV at [`crate::api::DCV`] (a DCV Connection Gateway on
    /// its machine): open it with [`crate::api::INFRA_DCV_SESSION`], no Windows password.
    pub dcv_gateway: bool,
    /// The backend has a DCV server of its own to use when the cell has no Windows IDE
    /// (`FACTORY_OS_DCV_SERVER`): Open PLC IDE works without one in the status.
    pub dcv_server_fallback: bool,
    /// When the instances were last read (Unix seconds; 0: never).
    pub updated_s: f64,
    /// A start or stop just asked for (`start` / `stop`), until the instances get there.
    pub transition: Option<String>,
    pub bringup: BringupStatus,
    pub cell: CellRunStatus,
    /// The CODESYS demo runtimes (`None`: not read yet).
    pub plc_runtime: Option<PlcRuntime>,
    pub auto_restart: AutoRestartStatus,
    /// Run Control Code.
    pub deploy: DeployStatus,
    /// The export Start production / Restart PLC runtime start (`None`: none; see
    /// `headExportError`).
    pub head_export: Option<ExportRef>,
    /// Why there is none (e.g. no export of the PLAN_ID the head runs).
    pub head_export_error: Option<String>,
    /// The PLAN_ID the head runs, if known ([`AutoRestartStatus::deployed_plan_id`], else the
    /// export last deployed's).
    pub head_plan_id: Option<String>,
    /// The newest successful export of the active cell (what Run Control Code deploys by default).
    pub newest_export: Option<ExportRef>,
    /// The export last downloaded to the head PLC (`source`: `deploy`, by a deploy job of this
    /// backend; `report`, by the newest `plan2cell-cell-report.json` with a successful `deploy`).
    pub deployed_export: Option<ExportRef>,
    /// The cell is in production: the last production start succeeded and neither Stop cell nor a
    /// deploy ran since (a Deploy only leaves the head waiting for Start production).
    pub in_production: bool,
    /// The head project of the deployed export (else of the newest) on the Windows IDE.
    pub ide_project: Option<IdeProject>,
    /// The last Start cell: per-instance results and its bring-up / production chain.
    pub start: CellStartStatus,
    /// The cell was in production when Stop cell last stopped it: the next Start cell starts production again.
    pub stopped_in_production: bool,
    /// The DCV targets this backend serves at `/dcv/<target>/` (e.g. [`DCV_TARGET_IDE`]); empty
    /// from an older server (then `ide` while [`Self::dcv_gateway`]).
    pub dcv_targets: Vec<String>,
    /// This backend's part in the cell (`FACTORY_OS_CELL_ROLE`).
    pub cell_role: CellRole,
    /// Who runs auto-restart when this backend is an operator (`FACTORY_OS_CELL_SUPERVISOR`), if known.
    pub cell_supervisor: Option<String>,
    /// The cell's lease as last read (`None`: not read, or a cell-control Lambda without leases).
    pub lease: Option<CellLease>,
    /// The power of the whole cell (`None`: not read yet, or a cell-control Lambda without power).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_power: Option<CellPower>,
    /// The workspace's own virtual cell, if it has one ([`CellsList::cells`]; `None`: none, bound
    /// to the shared cell, or an older server). Named `virtualCell` because [`Self::cell`] is the
    /// production status.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub virtual_cell: Option<VirtualCell>,
    /// Which cell this backend's cell control acts on (`None`: no cell-control function, or an
    /// older server: the shared cell).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub binding: Option<CellKind>,
}

impl InfraStatus {
    /// Whether DCV target `target` can be opened here (an older server: `ide` with a gateway).
    pub fn has_dcv_target(&self, target: &str) -> bool {
        self.dcv_gateway && (self.dcv_targets.iter().any(|t| t == target) || (self.dcv_targets.is_empty() && target == DCV_TARGET_IDE))
    }

    pub fn instance(&self, role: &str) -> Option<&InfraInstance> {
        self.instances.iter().find(|i| i.role == role)
    }

    /// Start cell's chain, bring-up, production or Run Control Code in progress.
    pub fn busy(&self) -> bool {
        self.start.state.busy() || self.bringup.state.busy() || self.cell.state.busy() || self.deploy.state.busy()
    }

    /// Some instance is between states (`pending`, `stopping`, ...) or a start / stop is under way.
    pub fn changing(&self) -> bool {
        self.transition.is_some() || self.instances.iter().any(InfraInstance::changing)
    }

    /// Every instance stopped (or none known).
    pub fn all_stopped(&self) -> bool {
        self.instances.iter().all(|i| i.state == "stopped")
    }

    /// Why nothing can use the cell now because of its power (off, stopping, booting), for a
    /// disabled button; `None`: it is on (or no power is known).
    pub fn power_blocks(&self) -> Option<String> {
        let power = self.cell_power.as_ref()?;
        match power.state {
            PowerState::Off => Some("The virtual cell is off: Start virtual cell first (in the Virtual cell box)".to_owned()),
            PowerState::Stopping => Some("The virtual cell is stopping".to_owned()),
            PowerState::Booting => Some(format!("The virtual cell is {}: wait until it is ready", power.booting_text())),
            _ => None,
        }
    }
}

/// [`CellPower::state`]: the coarse state, for the status dot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PowerState {
    Off,
    /// Starting instances, booting, bring-up.
    Booting,
    Ready,
    /// Running, but a readiness check fails (or was not read yet), and no power-on is under way.
    Degraded,
    Stopping,
    /// Some instances run, some do not.
    Partial,
    /// The last power-on / power-off did not get there ([`CellPower::message`]: why).
    Failed,
    /// A state this viewer does not know (a newer Lambda).
    #[default]
    #[serde(other)]
    Unknown,
}

impl PowerState {
    /// The cell is (on its way) on: not off, not stopping.
    pub fn on(self) -> bool {
        !matches!(self, Self::Off | Self::Stopping)
    }
}

/// The phases of [`CellPower::phase`], as the Lambda names them.
pub const POWER_PHASES: [&str; 13] = [
    "off",
    "launching",
    "starting-instances",
    "booting",
    "configuring",
    "bring-up",
    "ready",
    "not-ready",
    "archiving",
    "terminating",
    "stopping",
    "partial",
    "failed",
];

/// The phases a fresh launch of an own cell goes through, in order (for the stepper; `restoring`
/// is the backend's restore of the last control code after `ready`, [`VirtualCell::restore`]).
pub const OWN_CELL_START_PHASES: [&str; 5] = ["launching", "booting", "configuring", "bring-up", "ready"];
/// The phases of an own cell's stop (Stop terminates its instances).
pub const OWN_CELL_STOP_PHASES: [&str; 2] = ["archiving", "terminating"];

/// The power of the whole virtual cell (the cell-control Lambda's `power_status`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellPower {
    pub state: PowerState,
    /// [`POWER_PHASES`]: `off`, `starting-instances`, `booting` (running, SSM agents not back yet),
    /// `bring-up` (KUKA bring-up and readiness checks), `ready`, `not-ready`, `stopping`,
    /// `partial`, `failed`.
    pub phase: String,
    /// When this phase began (ISO 8601), if known.
    pub since: Option<String>,
    /// What it does now, or why it failed / is not ready.
    pub message: Option<String>,
    pub hosts: Vec<PowerHost>,
    /// The readiness checks (PLC runtime, routing, each KUKA VC, DCV, MCP bridge), as last read.
    pub checks: Vec<PowerCheck>,
    /// When the checks were last read (ISO 8601).
    pub checked_at: Option<String>,
    /// The last power-on / power-off.
    pub session: Option<PowerSession>,
    /// The Lambda's idle stop (`None`: off).
    pub idle: Option<IdleStop>,
    /// A backend reported the cell in production (recently).
    pub production: Option<PowerProduction>,
    /// An own cell of [`OFF_MODE_STOP`] that is stopped (its instances and disks kept; phase `off`).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub stopped: bool,
}

impl CellPower {
    /// `booting (bring-up)`, `ready`, ...: the phase in words.
    pub fn phase_label(&self) -> &str {
        match self.phase.as_str() {
            "starting-instances" => "starting the instances",
            "launching" => "launching the instances",
            "configuring" => "configuring the instances",
            "archiving" => "archiving the cell's state",
            "terminating" => "terminating the instances",
            "booting" => "booting",
            "bring-up" => "bring-up",
            "not-ready" => "not ready",
            "" => "unknown",
            other => other,
        }
    }

    /// `booting (bring-up)`, `booting (starting the instances)`, `booting`.
    pub fn booting_text(&self) -> String {
        match self.phase.as_str() {
            "booting" | "" => "booting".to_owned(),
            _ => format!("booting ({})", self.phase_label()),
        }
    }

    /// `Virtual cell: off`, `Virtual cell: booting (bring-up)`, `Virtual cell: ready`, ...
    pub fn line(&self) -> String {
        let state = match self.state {
            PowerState::Off => "off".to_owned(),
            PowerState::Booting => self.booting_text(),
            PowerState::Ready => "ready".to_owned(),
            PowerState::Degraded => "running, not ready".to_owned(),
            PowerState::Stopping => "stopping".to_owned(),
            PowerState::Partial => "partly running".to_owned(),
            PowerState::Failed => "failed".to_owned(),
            PowerState::Unknown => self.phase_label().to_owned(),
        };
        format!("Virtual cell: {state}")
    }

    /// The checks that did not pass (failed, or not read).
    pub fn failing(&self) -> impl Iterator<Item = &PowerCheck> {
        self.checks.iter().filter(|c| c.ok != Some(true))
    }
}

/// One instance in [`CellPower::hosts`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PowerHost {
    pub role: String,
    pub instance_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_ns: Option<String>,
    pub state: String,
    pub ssm_ping: String,
    /// Its SSM agent answered since this boot.
    pub online: bool,
    pub private_ip: Option<String>,
    pub launch_time: Option<String>,
    /// It carries the cell's tag (`Plan2cellCell`), which the Lambda's start / stop needs.
    pub tagged: bool,
}

/// One readiness check in [`CellPower::checks`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PowerCheck {
    /// `plcRuntime`, `routing`, `kukaVc1`, ..., `dcv`, `mcpBridge`.
    pub name: String,
    pub label: String,
    /// `None`: not read yet.
    pub ok: Option<bool>,
    pub detail: Option<String>,
    pub host: Option<String>,
}

/// [`CellPower::session`]: the last power-on / power-off.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PowerSession {
    /// `on` / `off`.
    pub op: String,
    /// The backend that asked (`idle-stop`: the Lambda's idle stop).
    pub by: Option<String>,
    pub reason: Option<String>,
    pub requested_at: Option<String>,
    pub finished_at: Option<String>,
    pub done: bool,
    /// Every phase it went through and when.
    pub history: Vec<PowerPhaseAt>,
    /// The KUKA VC bring-up's result (`healthy`, `repaired`, ...).
    pub bringup_result: Option<String>,
    /// An own cell's power-on that resumes its stopped instances (no first boot, no restore).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub resume: bool,
    /// An own cell's power-off: `terminate` or `stop`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mode: Option<String>,
    /// An own cell's power-on that launched fresh instead of resuming: why.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fresh_reason: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PowerPhaseAt {
    pub phase: String,
    pub at: Option<String>,
}

/// [`CellPower::idle`]: the Lambda stops the cell after `hours` without use.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct IdleStop {
    pub hours: f64,
    pub last_activity_at: Option<String>,
    pub last_activity_by: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct PowerProduction {
    pub by: Option<String>,
    pub at: Option<String>,
}

/// `POST` [`crate::api::INFRA_CELL_POWER`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellPowerRequest {
    /// `on` / `off`.
    pub action: String,
    /// `off` while the cell is in production (production stops).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub force: bool,
    /// `on`: then Start production once it is ready (`true`), never (`false`), or (default) if
    /// the cell was in production when it was stopped (the supervisor only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub production: Option<bool>,
    /// `on` of an own cell (a fresh launch): restore the last deployed control code once it is
    /// ready: `auto` (default: the fast path if the archived boot applications match, else Deploy
    /// only), `full` (Deploy only), `none`. Ignored for the shared cell.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub restore: Option<String>,
    /// `on` of a stopped own cell ([`OFF_MODE_STOP`]): `true` Start fresh (new instances from the
    /// latest images), `false` resume even with newer images, `None` resume unless newer images
    /// wait (`409`: choose). A fresh launch that is due anyway ([`CellResume::fresh_reasons`])
    /// happens whatever this says. Also `true` on a cell whose resume failed (Start fresh).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fresh: Option<bool>,
    /// `off` of an own cell: [`OFF_MODE_TERMINATE`] or [`OFF_MODE_STOP`] this once (default: the
    /// cell's [`VirtualCell::off_mode`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mode: Option<String>,
}

/// The restore modes of [`CellPowerRequest::restore`].
pub const RESTORE_MODES: [&str; 3] = ["auto", "full", "none"];

/// An own cell's off mode ([`VirtualCell::off_mode`]): Stop terminates the instances (the default:
/// nothing is paid while off, every Start is a fresh launch from the latest images).
pub const OFF_MODE_TERMINATE: &str = "terminate";
/// Stop stops the instances (EC2 stop: their disks are paid while stopped), Start resumes them.
pub const OFF_MODE_STOP: &str = "stop";
pub const OFF_MODES: [&str; 2] = [OFF_MODE_TERMINATE, OFF_MODE_STOP];

/// An own cell's runtime ([`VirtualCell::runtime`]): EC2 instances (the default: PLC host, Windows IDE, KUKA VC hosts).
pub const RUNTIME_EC2: &str = "ec2";
/// Pods on the factoryos cells EKS cluster (plan2cell/infra/eks): the PLC and the KUKA VCs as Linux containers on a
/// PROFINET overlay across the nodes, the IDE as a Windows container ([`IDE_CONTAINER`]) or the EC2 IDE ([`IDE_EC2`]).
pub const RUNTIME_EKS: &str = "eks";
pub const RUNTIMES: [&str; 2] = [RUNTIME_EC2, RUNTIME_EKS];
/// An EKS cell's IDE: the compile node only (one headless CODESYS + MCP server pod per cell; no DCV, no Open PLC IDE).
pub const IDE_CONTAINER: &str = "container";
/// An EKS cell's IDE: the compile node and, beside it, the EC2 Windows IDE with DCV (Open PLC IDE).
pub const IDE_EC2: &str = "ec2";
pub const IDES: [&str; 2] = [IDE_CONTAINER, IDE_EC2];
/** Most PROFINET networks of an EKS cell (each a cell of its own: head PLC, KUKA VCs, segment). */
pub const MAX_NETWORKS: usize = 4;
/// KUKA VCs per EKS network (one kvc node's worth; the Lambda's eks.js MAX_NETWORK_ROBOTS).
pub const MAX_NETWORK_ROBOTS: u8 = 6;

/// One PROFINET network of an EKS cell: its own head PLC, KUKA VCs and L2 segment (VXLAN `vni`, `subnet`).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellNetwork {
    /// `n1` .. `n9`.
    pub name: String,
    pub robots: u8,
    /// The Lambda's (unique in the cluster; not set in a request).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vni: Option<u32>,
    /// `10.77.<k>.0/24` (the Lambda's).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subnet: Option<String>,
}

// ---------------------------------------------------------------------------------------------
// Per-workspace virtual cells (the `factoryos-cell` Lambda, qualifier `ws-<alias>`).
// ---------------------------------------------------------------------------------------------

/// Which cell a backend's cell control is bound to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum CellKind {
    /// The workspace's own cell (`factoryos-cell:ws-<alias>`): Stop terminates it (or, off mode `stop`, stops it).
    #[default]
    Own,
    /// The shared `plan2cell` cell (`plan2cell-cell-control`): stop / start in place.
    Shared,
}

/// [`VirtualCell::lifecycle`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CellLifecycle {
    /// Created, no instances (nothing is paid).
    #[default]
    Off,
    /// Launched (see its power's phase).
    On,
    /// Being deleted.
    Deleting,
    Failed,
    /// A lifecycle this viewer does not know (a newer Lambda).
    #[serde(other)]
    Unknown,
}

/// [`VirtualCell::cost`]: what the cell costs (eu-central-1 on-demand).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellCost {
    pub running_usd_per_hour: f64,
    pub off_usd_per_month: f64,
    /// The current (or last) session so far.
    pub session_usd: Option<f64>,
    pub note: String,
    /// The cell's EBS volumes (gp3), what a stopped cell still pays for.
    pub ebs_gib: u32,
    /// Those volumes per month / per day ([`OFF_MODE_STOP`] while stopped).
    pub stopped_usd_per_month: f64,
    pub stopped_usd_per_day: f64,
}

/// The AMI a role launches from ([`VirtualCell::images`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellImage {
    pub role: String,
    pub ami_id: String,
    pub recipe_version: String,
    pub build_date: Option<String>,
}

/// The backend's restore of the last control code after a fresh launch ([`VirtualCell::restore`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellRestore {
    pub state: JobState,
    /// `fast` (boot applications restored: `krl` and `start`), `full` (Deploy only), `none`.
    pub mode: String,
    pub build: Option<String>,
    pub plan_id: Option<String>,
    pub message: Option<String>,
}

/// A newer image than a stopped instance's ([`CellResume::newer_images`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct NewerImage {
    pub role: String,
    /// The recipe version the stopped instance runs.
    pub running: String,
    pub latest: String,
    pub build_date: Option<String>,
}

/// What the next Start of a stopped own cell does ([`VirtualCell::resume`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellResume {
    /// It resumes (no fresh launch is due).
    pub possible: bool,
    /// Why the next Start is a fresh launch anyway (stopped too long, another robot count or
    /// workspace address, an instance gone).
    pub fresh_reasons: Vec<String>,
    /// Images newer than the stopped instances': resume or Start fresh.
    pub newer_images: Vec<NewerImage>,
    pub stopped_at: Option<String>,
    /// The oldest image the stopped instances run (its build date).
    pub image_date: Option<String>,
    pub max_stopped_days: u32,
    /// The next fresh launch resolves another template version than the stopped one runs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub newer_template: Option<NewerTemplate>,
}

// ---------------------------------------------------------------------------------------------
// Cell templates (docs/tasks/cell-templates.md §8, §13): a cell boots from a template the
// `factoryos-cell` Lambda resolves; consumers switch on its capabilities and semantics, never on
// role names. Every type is a tolerant reader, so an older Lambda (no templates) reads as today.
// ---------------------------------------------------------------------------------------------

/// The legacy templates (a cell without `cell_status.template` is one of these, by its runtime).
pub const TEMPLATE_EC2_CODESYS_KUKA: &str = "ec2-codesys-kuka";
pub const TEMPLATE_EKS_CODESYS_KUKA: &str = "eks-codesys-kuka";
/// The wiring level this backend renders (a template's `requires.consumer` must not exceed it):
/// 3 renders the hook drivers of engine-2 templates (docs/tasks/cell-templates.md E0, E8-E10).
pub const CONSUMER_LEVEL: u32 = 3;
/// The contract of a `pod@1` component's hook (`requires.hook`, E7).
pub const HOOK_CONTRACT: &str = "factoryos/hook@1";
/// The capabilities of a cell driven through [`HOOK_CONTRACT`] (E9), sorted.
pub const HOOK_CAPABILITIES: [&str; 6] = ["check.hook", "deploy.head.hook", "deploy.robot.hook", "live.head.exec", "live.robot.exec", "runtime.hook"];
/// The capability vocabulary (docs/tasks/cell-templates.md §4.2), sorted.
pub const CAPABILITIES: [&str; 18] = [
    "archive",
    "bringup.action",
    "check.hook",
    "check.kss-config",
    "deploy.head.codesys-mcp",
    "deploy.head.hook",
    "deploy.robot.hook",
    "deploy.robot.kss",
    "ide.dcv",
    "live.head.exec",
    "live.head.opcua",
    "live.robot.exec",
    "live.robot.vass-zenoh",
    "offmode.stop",
    "restore",
    "runtime.codesys",
    "runtime.hook",
    "smartpad",
];

/// The capabilities of a cell whose Lambda reports no template (an older Lambda or templates off),
/// sorted: the legacy templates' (§4.2). An EKS cell has no archive, restore, stop mode or bring-up
/// action, and DCV only with the EC2 IDE.
pub fn legacy_capabilities(runtime: &str, ide: Option<&str>) -> Vec<&'static str> {
    if runtime != RUNTIME_EKS {
        return vec![
            "archive",
            "bringup.action",
            "check.kss-config",
            "deploy.head.codesys-mcp",
            "deploy.robot.kss",
            "ide.dcv",
            "live.head.opcua",
            "live.robot.vass-zenoh",
            "offmode.stop",
            "restore",
            "runtime.codesys",
            "smartpad",
        ];
    }
    let mut caps = vec!["check.kss-config", "deploy.head.codesys-mcp", "deploy.robot.kss"];
    if ide == Some(IDE_EC2) {
        caps.push("ide.dcv");
    }
    caps.extend(["live.head.opcua", "live.robot.vass-zenoh", "runtime.codesys", "smartpad"]);
    caps
}

/// The legacy template of a runtime ([`RUNTIME_EKS`]: [`TEMPLATE_EKS_CODESYS_KUKA`], anything else EC2).
pub fn legacy_template(runtime: &str) -> &'static str {
    if runtime == RUNTIME_EKS { TEMPLATE_EKS_CODESYS_KUKA } else { TEMPLATE_EC2_CODESYS_KUKA }
}

/// A template id as the Lambda takes it: `^[a-z][a-z0-9-]{2,40}$`.
pub fn valid_template_id(id: &str) -> bool {
    (3..=41).contains(&id.len())
        && id.starts_with(|c: char| c.is_ascii_lowercase())
        && id.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
}

/// A plain semver `MAJOR.MINOR.PATCH` (no pre-release, no build part, no leading zeros).
pub fn valid_template_version(version: &str) -> bool {
    let parts: Vec<&str> = version.split('.').collect();
    parts.len() == 3
        && parts.iter().all(|p| !p.is_empty() && p.len() <= 9 && p.chars().all(|c| c.is_ascii_digit()) && (p.len() == 1 || !p.starts_with('0')))
}

/// A template parameter's name: `^[a-z][A-Za-z0-9]{0,31}$`.
pub fn valid_param_name(name: &str) -> bool {
    (1..=32).contains(&name.len()) && name.starts_with(|c: char| c.is_ascii_lowercase()) && name.chars().all(|c| c.is_ascii_alphanumeric())
}

/// One parameter of a template (§4.1), as the template has it.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateParameter {
    /// `integer`, `enum`, `boolean` or `networks`.
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<i64>,
    pub default: serde_json::Value,
    /// An `enum`'s values.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub values: Vec<String>,
    /// A `networks` parameter's robots per network.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robots: Option<NetworkRobots>,
    /// `always`, `off` (only while the cell is off) or `never` (create only).
    pub mutable: String,
    pub label: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct NetworkRobots {
    pub min: u8,
    pub max: u8,
    pub default: u8,
}

/// A capability or component only with a parameter's value.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct OnlyIf {
    pub param: String,
    pub equals: serde_json::Value,
}

/// A capability of a template (`only_if`: only with that parameter's value).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateCapability {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub only_if: Option<OnlyIf>,
}

/// An operation's argument (§4.6): required or not, one of `values` or (with `prefix`) starting with it.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateOpArgs {
    pub required: bool,
    pub values: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prefix: Option<String>,
}

/// A robot-controller operation of a template (`vcell_kvc`).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateOp {
    pub name: String,
    pub label: String,
    /// `None`: it takes no argument.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arg_schema: Option<TemplateOpArgs>,
    /// It changes the controller, except with one of `read_args`.
    pub destructive: bool,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub read_args: Vec<String>,
}

/// The Control Code Export defaults on a template's cell.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateBuild {
    pub plc_dialect: String,
    pub robot_language: String,
    pub profile: String,
    pub robot_sim: bool,
    pub opcua_symbols: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateDeploy {
    /// A subsequence of [`DEPLOY_STAGES`].
    pub stages: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateLimits {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robots: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_running: Option<u32>,
}

/// A template's consumer block (§4.6): what the backend, the viewer and the agents do with its cell.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateConsumer {
    #[serde(deserialize_with = "null_default")]
    pub deploy: TemplateDeploy,
    #[serde(deserialize_with = "null_default")]
    pub build: TemplateBuild,
    #[serde(deserialize_with = "null_default")]
    pub ops: Vec<TemplateOp>,
    pub e2e: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limits: Option<TemplateLimits>,
}

/// `null` as `T`'s default (`#[serde(default)]` only covers a missing key): the Lambda sends `null`
/// for what it does not know (a version it has not loaded, no cost).
fn null_default<'de, D, T>(deserializer: D) -> Result<T, D::Error>
where
    D: serde::Deserializer<'de>,
    T: Default + Deserialize<'de>,
{
    Ok(Option::<T>::deserialize(deserializer)?.unwrap_or_default())
}

/// What a template's cell costs at its default parameters (a hint).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateCostHint {
    pub running_usd_per_hour: f64,
    pub note: String,
}

/// What a template needs of its readers: the engine level, the kinds, the wiring level.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct TemplateRequires {
    pub engine: u32,
    pub kinds: Vec<String>,
    /// The wiring level the backend must render ([`CONSUMER_LEVEL`]).
    pub consumer: u32,
    /// [`HOOK_CONTRACT`] when a `pod@1` component has a hook (engine 2).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hook: Option<String>,
}

/// One entry of the Lambda's `templates` (§8): the version a new cell would get.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellTemplateInfo {
    pub id: String,
    #[serde(deserialize_with = "null_default")]
    pub version: String,
    /// `stable`, `preview` or `pin`.
    #[serde(deserialize_with = "null_default")]
    pub channel: String,
    /// `s3`, `cache` or `builtin`.
    #[serde(deserialize_with = "null_default")]
    pub source: String,
    #[serde(deserialize_with = "null_default")]
    pub title: String,
    #[serde(deserialize_with = "null_default")]
    pub description: String,
    /// `active` or `deprecated`.
    #[serde(deserialize_with = "null_default")]
    pub status: String,
    /// [`RUNTIME_EC2`] or [`RUNTIME_EKS`].
    #[serde(deserialize_with = "null_default")]
    pub runtime: String,
    #[serde(deserialize_with = "null_default")]
    pub parameters: BTreeMap<String, TemplateParameter>,
    #[serde(deserialize_with = "null_default")]
    pub capabilities: Vec<TemplateCapability>,
    #[serde(deserialize_with = "null_default")]
    pub consumer: TemplateConsumer,
    #[serde(deserialize_with = "null_default")]
    pub cost: TemplateCostHint,
    #[serde(deserialize_with = "null_default")]
    pub agent_notes: String,
    /// `None`: a Lambda that does not say (then it runs at this backend's level).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires: Option<TemplateRequires>,
    pub available: bool,
    /// Why not: `engine`, `runtime`, `denied`, `unavailable`, `consumer` (this backend's wiring level).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl CellTemplateInfo {
    /// It needs a wiring level this backend does not render.
    pub fn beyond_consumer(&self) -> bool {
        self.requires.as_ref().is_some_and(|r| r.consumer > CONSUMER_LEVEL)
    }
}

/// `GET` [`crate::api::INFRA_TEMPLATES`]: the templates this workspace may create a cell from.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellTemplates {
    /// `false`: no own cell, or a Lambda without `templates` (then the runtime pickers, [`CellsList::runtimes`]).
    pub available: bool,
    /// Template-resolved launches are on for this workspace (else only the builtin legacy templates).
    pub enabled: bool,
    /// The Lambda's engine level.
    pub engine: u32,
    /// The index revision the list comes from (`None`: none read, the builtin templates).
    pub revision: Option<u64>,
    /// `s3`, `cache` or `builtin`.
    pub source: String,
    pub templates: Vec<CellTemplateInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// `cell_status.template` (§8): the template a cell runs (its launch record), or the one its next
/// launch would resolve to.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellTemplateRef {
    pub id: String,
    #[serde(deserialize_with = "null_default")]
    pub version: String,
    #[serde(deserialize_with = "null_default")]
    pub sha256: String,
    /// `s3`, `cache` or `builtin`.
    #[serde(deserialize_with = "null_default")]
    pub source: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channel: Option<String>,
    /// The launch it was resolved for (`None`: what the next launch would resolve to).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub launch: Option<u32>,
    #[serde(deserialize_with = "null_default")]
    pub parameters: serde_json::Map<String, serde_json::Value>,
    /// Resolved for the cell's parameters, sorted.
    #[serde(deserialize_with = "null_default")]
    pub capabilities: Vec<String>,
    #[serde(deserialize_with = "null_default")]
    pub consumer: TemplateConsumer,
    /// Each host role the power view reports (`plcHost`, `n1/kukaVc1`, ...) to its semantic
    /// (`head-plc`, `robot-controller`, `compile`, `ide`, `aux`).
    #[serde(deserialize_with = "null_default")]
    pub semantics: BTreeMap<String, String>,
}

impl CellTemplateRef {
    pub fn has(&self, capability: &str) -> bool {
        self.capabilities.iter().any(|c| c == capability)
    }

    /// The semantic of host role `role`.
    pub fn semantic(&self, role: &str) -> Option<&str> {
        self.semantics.get(role).map(String::as_str)
    }

    /// Some host is driven through [`HOOK_CONTRACT`] (an engine-2 template: any of
    /// [`HOOK_CAPABILITIES`]). Such a cell has no legacy role names: its hosts are found by their
    /// semantics.
    pub fn hook(&self) -> bool {
        self.capabilities.iter().any(|c| HOOK_CAPABILITIES.contains(&c.as_str()))
    }

    /// The first host role (in role order: `n1/...` before `n2/...`) of `semantic`.
    pub fn role_of(&self, semantic: &str) -> Option<&str> {
        self.semantics.iter().find(|(_, s)| *s == semantic).map(|(r, _)| r.as_str())
    }

    /// `ec2-codesys-kuka@1.0.0`.
    pub fn label(&self) -> String {
        format!("{}@{}", self.id, self.version)
    }
}

/// cell_create / cell_update `template`: an id (track the major it resolves to now) or a pinned version.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum TemplateSelector {
    Id(String),
    Pinned { id: String, version: String },
}

impl TemplateSelector {
    pub fn id(&self) -> &str {
        match self {
            Self::Id(id) | Self::Pinned { id, .. } => id,
        }
    }

    pub fn version(&self) -> Option<&str> {
        match self {
            Self::Id(_) => None,
            Self::Pinned { version, .. } => Some(version),
        }
    }
}

/// A stopped cell runs `running`, its next fresh launch would get `latest` ([`CellResume::newer_template`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct NewerTemplate {
    pub id: String,
    pub running: String,
    pub latest: String,
}

/// A virtual cell of a workspace.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct VirtualCell {
    /// `ws-<alias>`.
    pub id: String,
    pub workspace: String,
    pub kind: CellKind,
    pub lifecycle: CellLifecycle,
    pub created_at: Option<String>,
    /// The launch counter (`FactoryosCellLaunch`).
    pub launch: u32,
    pub robots: u8,
    pub power: Option<CellPower>,
    pub cost: CellCost,
    pub images: Vec<CellImage>,
    pub restore: Option<CellRestore>,
    /// It was on when its workspace stopped (the GUI offers to start it again).
    pub was_on_at_stop: bool,
    pub message: Option<String>,
    /// [`OFF_MODE_TERMINATE`] (also when empty: an older Lambda) or [`OFF_MODE_STOP`].
    pub off_mode: String,
    /// Stopped ([`OFF_MODE_STOP`]): its instances kept, Start resumes them.
    pub stopped: bool,
    /// While stopped: what the next Start does.
    pub resume: Option<CellResume>,
    /// [`RUNTIME_EC2`] (also when empty: an older Lambda) or [`RUNTIME_EKS`].
    pub runtime: String,
    /// An EKS cell's IDE ([`IDES`]).
    pub ide: Option<String>,
    /// An EKS cell's PROFINET networks (empty: an EC2 cell, one network).
    pub networks: Vec<CellNetwork>,
    /// The template it runs (`cell_status.template`; `None`: an older Lambda, or templates off).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub template: Option<CellTemplateRef>,
    /// Its template parameters as the record has them (`None`: a record without them).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub params: Option<serde_json::Map<String, serde_json::Value>>,
}

impl VirtualCell {
    /// Its pods run on the EKS cluster (else EC2 instances).
    pub fn eks(&self) -> bool {
        self.runtime == RUNTIME_EKS
    }

    /// It has a DCV-capable IDE: its template's `ide.dcv`; without a template, an EC2 cell or an
    /// EKS cell with the EC2 IDE.
    pub fn has_dcv(&self) -> bool {
        match &self.template {
            Some(t) => t.has("ide.dcv"),
            None => !self.eks() || self.ide.as_deref() == Some(IDE_EC2),
        }
    }

    /// Whether it has `capability`: its template's, else the legacy template's of its runtime.
    pub fn has(&self, capability: &str) -> bool {
        match &self.template {
            Some(t) => t.has(capability),
            None => legacy_capabilities(&self.runtime, self.ide.as_deref()).contains(&capability),
        }
    }

    /// Its Stop stops the instances (else it terminates them).
    pub fn stop_mode(&self) -> bool {
        self.off_mode == OFF_MODE_STOP
    }

    /// Its power's phase (`off` while it has none).
    pub fn phase(&self) -> &str {
        match &self.power {
            Some(p) if !p.phase.is_empty() => p.phase.as_str(),
            _ if self.lifecycle == CellLifecycle::Deleting => "deleting",
            _ => "off",
        }
    }
}

/// `GET` [`crate::api::INFRA_CELLS`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellsList {
    /// The backend can create / delete a cell of its own (it has the `factoryos-cell` function).
    pub available: bool,
    pub max_per_workspace: u32,
    pub can_create: bool,
    /// Why not (no function, one exists, the account's cap).
    pub reason: Option<String>,
    /// The own cell (0 or 1).
    pub cells: Vec<VirtualCell>,
    /// `plan2cell` while bound to the shared cell.
    pub shared: Option<String>,
    /// The runtimes the function offers ([`RUNTIMES`]; empty: an older Lambda, EC2 only).
    pub runtimes: Vec<String>,
    /// The templates a cell may be created from ([`CellTemplates::templates`]; empty: an older
    /// Lambda, use [`Self::runtimes`]).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub templates: Vec<CellTemplateInfo>,
    /// Template-resolved launches are on for this workspace.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub templates_enabled: bool,
}

/// `POST` [`crate::api::INFRA_CELLS`]. With `template` or `params`, the legacy fields (`robots`,
/// `offMode`, `runtime`, `ide`, `networks`) are refused: the template's parameters say it.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CreateCellRequest {
    /// A template id ([`CellTemplates`]) or a pinned `{id, version}` (default: the legacy fields).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub template: Option<TemplateSelector>,
    /// The template's parameters (omitted ones take its defaults).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub params: Option<serde_json::Map<String, serde_json::Value>>,
    /// KUKA robots (default 2).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robots: Option<u8>,
    /// Start it right after creating it.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub start: bool,
    /// [`OFF_MODES`] (default [`OFF_MODE_TERMINATE`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub off_mode: Option<String>,
    /// [`RUNTIMES`] (default [`RUNTIME_EC2`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub runtime: Option<String>,
    /// [`IDES`] (EKS only; default [`IDE_CONTAINER`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ide: Option<String>,
    /// EKS only: its networks (`name`, `robots`; default one network `n1` of `robots`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub networks: Option<Vec<CellNetwork>>,
}

/// `PATCH` [`crate::api::INFRA_CELL_ITEM`]: the own cell's settings.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct UpdateCellRequest {
    /// Another template (only while off and not stopped; its defaults fill what `params` does not say).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub template: Option<TemplateSelector>,
    /// Template parameters to change (each when its `mutable` allows).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub params: Option<serde_json::Map<String, serde_json::Value>>,
    /// [`OFF_MODES`]: what Stop does from now on (any time).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub off_mode: Option<String>,
    /// KUKA robots (only while off; a stopped cell's next Start is then fresh).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robots: Option<u8>,
    /// [`RUNTIMES`] (only while off and not stopped).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub runtime: Option<String>,
    /// [`IDES`] (EKS only; only while off).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ide: Option<String>,
    /// EKS only, only while off: its networks from now on.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub networks: Option<Vec<CellNetwork>>,
}

impl CreateCellRequest {
    /// It names a legacy field (`robots`, `offMode`, `runtime`, `ide`, `networks`).
    pub fn has_legacy_fields(&self) -> bool {
        self.robots.is_some() || self.off_mode.is_some() || self.runtime.is_some() || self.ide.is_some() || self.networks.is_some()
    }
}

impl UpdateCellRequest {
    /// It names a legacy field (`offMode`, `robots`, `runtime`, `ide`, `networks`).
    pub fn has_legacy_fields(&self) -> bool {
        self.off_mode.is_some() || self.robots.is_some() || self.runtime.is_some() || self.ide.is_some() || self.networks.is_some()
    }
}

/// The refusal of a request that gives a template and legacy fields (the Lambda's words).
pub const TEMPLATE_AND_LEGACY_FIELDS: &str = "template: give params, not runtime/ide/robots/networks/offMode";

/// `DELETE` [`crate::api::INFRA_CELL_ITEM`] (the cell id typed again).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeleteCellRequest {
    pub confirm: String,
}

/// `POST /api/v1/infra/start` and `/stop`: which instances (default: all; `kukaVc`: every KUKA VC host).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct StartStopRequest {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub roles: Option<Vec<String>>,
    /// Start: wait until every instance runs and is SSM-online, then bring up the controllers
    /// (default: for a start of the whole cell, not of some roles).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bring_up: Option<bool>,
    /// Start: then Start production (`true`; implies the bring-up), never (`false`), or (default)
    /// if the cell was in production when it was stopped ([`InfraStatus::stopped_in_production`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub production: Option<bool>,
}

impl StartStopRequest {
    /// Start the instances only, nothing after.
    pub fn instances_only() -> Self {
        Self { roles: None, bring_up: Some(false), production: Some(false) }
    }
}

/// `POST /api/v1/infra/bringup`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BringupRequest {
    /// Restart every KUKA controller first (`kvc-restart.sh` on each host; a controller stuck with M_27002).
    pub restart_controllers: bool,
}

/// A backend's part in a cell several backends share (`FACTORY_OS_CELL_ROLE`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CellRole {
    /// Runs auto-restart and resumes production after Start cell (one per cell; an older server).
    #[default]
    Supervisor,
    /// User actions only (under the cell's lease); never auto-restart.
    Operator,
}

impl CellRole {
    /// `supervisor` / `operator` (any case); anything else is `None`.
    pub fn parse(text: &str) -> Option<Self> {
        match text.trim().to_ascii_lowercase().as_str() {
            "supervisor" => Some(Self::Supervisor),
            "operator" => Some(Self::Operator),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Supervisor => "supervisor",
            Self::Operator => "operator",
        }
    }
}

/// The cell's lease (the cell-control Lambda's `lease` `status`): who runs a job on the cell now,
/// and the last production change.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellLease {
    /// The backend (its workspace) holding it; `None`: free.
    pub holder: Option<String>,
    /// What it does (`production`, `deploy`, `start`, ...).
    pub purpose: Option<String>,
    pub since_s: Option<f64>,
    pub expires_s: Option<f64>,
    pub last_change: Option<LeaseChange>,
    /// The holder is this backend (set by the backend, not the Lambda).
    pub mine: bool,
}

impl CellLease {
    /// Held by another backend, not expired at `now` (Unix seconds).
    pub fn held_by_other(&self, now: f64) -> bool {
        self.holder.is_some() && !self.mine && self.expires_s.is_none_or(|e| e > now)
    }

    /// `Cell in use by <holder> (<purpose>, since hh:mm UTC)`.
    pub fn describe(&self) -> Option<String> {
        let holder = self.holder.as_deref()?;
        let purpose = self.purpose.as_deref().unwrap_or("a job");
        let since = self.since_s.map(|s| format!(", since {}", utc_hhmm(s))).unwrap_or_default();
        Some(format!("Cell in use by {holder} ({purpose}{since})"))
    }
}

/// The last change of the cell's production a lease holder reported on release.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct LeaseChange {
    pub holder: String,
    pub action: String,
    pub plan_id: Option<String>,
    pub at_s: f64,
}

/// `POST /api/v1/infra/dcv/session` (optional body): the DCV target (default [`DCV_TARGET_IDE`]).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DcvSessionRequest {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
}

/// `POST /api/v1/infra/dcv/session`: open the Windows IDE's DCV console session in a new window.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DcvSession {
    /// Relative to the backend's base URL, e.g. `/dcv/ide/?authToken=<token>#console`: the DCV
    /// web client connects with the token, no login. Open it under [`Self::workspace`]'s prefix
    /// (`crate::workspace::pinned_url`): the browser it opens in may select another workspace.
    pub url: String,
    /// Until when the token can start a connection (Unix seconds). Its first connection binds it
    /// to that client's address, which may then reconnect with it for some hours.
    pub expires_s: f64,
    /// When the server issued it (Unix seconds, the server's clock: the viewer compares it with
    /// [`DeployStatus::finished_s`]). 0 from an older server.
    pub issued_s: f64,
    /// The workspace of the backend that issued it (its token is redeemed through that backend's
    /// gateway only); `None`: a backend without a workspace, or an older server.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workspace: Option<String>,
    /// The DCV target (empty from an older server: [`DCV_TARGET_IDE`]).
    pub target: String,
}

/// `DELETE /api/v1/infra/dcv/session`: every DCV session token revoked (running connections stay).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DcvRevoked {
    pub revoked: usize,
}

/// `HH:MM UTC` of Unix seconds `s`.
pub fn utc_hhmm(s: f64) -> String {
    let secs = s.max(0.0) as u64 % 86_400;
    format!("{:02}:{:02} UTC", secs / 3600, secs / 60 % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_format_is_camel_case() {
        let status = InfraStatus {
            available: true,
            instances: vec![InfraInstance { role: ROLE_WINDOWS_IDE.into(), ssm_ping: "Online".into(), ..Default::default() }],
            dcv_url: Some("https://h:8443/".into()),
            ..Default::default()
        };
        let json = serde_json::to_value(&status).unwrap();
        assert_eq!(json["instances"][0]["ssmPing"], "Online");
        assert_eq!(json["dcvUrl"], "https://h:8443/");
        assert_eq!(json["bringup"]["state"], "idle");
        let back: InfraStatus = serde_json::from_value(json).unwrap();
        assert_eq!(back, status);
        assert_eq!(serde_json::to_string(&BringupRequest { restart_controllers: true }).unwrap(), r#"{"restartControllers":true}"#);
        assert_eq!(role_label(ROLE_KUKA_VC), "KUKA VC host");
        assert_eq!(role_label("kukaVc2"), "KUKA VC host 2");
        assert_eq!(role_label(ROLE_PLC_HOST), "PLC host");
        assert!(is_kuka_vc("kukaVc1") && is_kuka_vc(ROLE_KUKA_VC) && !is_kuka_vc("kukaVcX") && !is_kuka_vc(ROLE_PLC_HOST));
        assert!(known_role("kukaVc12") && known_role(ROLE_WINDOWS_IDE) && !known_role("database"));
        // The Lambda's per-host instances (robotNs) and an older one's single host both read.
        let two: InfraStatus = serde_json::from_str(
            r#"{"instances":[{"role":"kukaVc1","instanceId":"i-1","robotNs":"robot1","state":"running"},{"role":"kukaVc","instanceId":"i-0"}]}"#,
        )
        .unwrap();
        assert_eq!(two.instances[0].robot_ns.as_deref(), Some("robot1"));
        assert_eq!(two.instances[1].robot_ns, None);
        assert!(!serde_json::to_string(&two.instances[1]).unwrap().contains("robotNs"));
        assert_eq!(utc_hhmm(8.0 * 3600.0 + 5.0 * 60.0), "08:05 UTC");
        let session = DcvSession { url: "/dcv/?authToken=t#console".into(), expires_s: 1.0, issued_s: 0.5, ..Default::default() };
        assert_eq!(
            serde_json::to_value(&session).unwrap(),
            serde_json::json!({"url": "/dcv/?authToken=t#console", "expiresS": 1.0, "issuedS": 0.5, "target": ""})
        );
        let session = DcvSession { workspace: Some("jdoe".into()), target: DCV_TARGET_IDE.into(), ..session };
        assert_eq!(serde_json::to_value(&session).unwrap()["workspace"], "jdoe");
        assert_eq!(serde_json::to_string(&DcvSessionRequest { target: Some("ide".into()) }).unwrap(), r#"{"target":"ide"}"#);
        assert_eq!(serde_json::from_str::<DcvSessionRequest>("{}").unwrap().target, None);
        // Older servers: no targets listed, `ide` while they have a gateway; supervisors.
        let old: InfraStatus = serde_json::from_str(r#"{"dcvGateway":true}"#).unwrap();
        assert!(old.has_dcv_target(DCV_TARGET_IDE) && !old.has_dcv_target("robot1"));
        assert_eq!(old.cell_role, CellRole::Supervisor);
        let new: InfraStatus = serde_json::from_str(r#"{"dcvGateway":true,"dcvTargets":["robot1"],"cellRole":"operator"}"#).unwrap();
        assert!(!new.has_dcv_target(DCV_TARGET_IDE) && new.has_dcv_target("robot1"));
        assert_eq!(new.cell_role, CellRole::Operator);
        assert_eq!((CellRole::parse(" Operator "), CellRole::parse("x")), (Some(CellRole::Operator), None));
        let lease: CellLease = serde_json::from_str(
            r#"{"holder":"plan2cell-test","purpose":"production","sinceS":29100,"expiresS":2e9,"lastChange":{"holder":"jdoe","action":"deploy","planId":"16#9201","atS":1}}"#,
        )
        .unwrap();
        assert_eq!(lease.describe().as_deref(), Some("Cell in use by plan2cell-test (production, since 08:05 UTC)"));
        assert!(lease.held_by_other(1.0) && !CellLease { mine: true, ..lease.clone() }.held_by_other(1.0));
        assert_eq!(lease.last_change.unwrap().plan_id.as_deref(), Some("16#9201"));
        assert_eq!(CellLease::default().describe(), None);
        assert_eq!(serde_json::from_str::<DcvSession>(r#"{"url":"/dcv/","expiresS":1.0}"#).unwrap().issued_s, 0.0, "an older server");
        assert!(!serde_json::from_str::<InfraStatus>("{}").unwrap().dcv_gateway);
        let rt = PlcRuntime { running: true, stops_s: Some(1.0), seconds_left: Some(2.0), source: Some("demo line".into()), ..Default::default() };
        let json = serde_json::to_value(InfraStatus { plc_runtime: Some(rt), ..Default::default() }).unwrap();
        assert_eq!(json["plcRuntime"]["secondsLeft"], 2.0);
        assert_eq!(json["plcRuntime"]["stopsS"], 1.0);
        assert_eq!(json["autoRestart"]["enabled"], false);
        assert_eq!(serde_json::to_string(&ProductionRequest { match_deployed: true, build: None }).unwrap(), r#"{"matchDeployed":true}"#);
        assert_eq!(serde_json::from_str::<ProductionRequest>(r#"{"build":"b"}"#).unwrap().build.as_deref(), Some("b"));
        assert_eq!(serde_json::to_string(&DeployRequest::default()).unwrap(), "{}");
        // Older clients send no `start`: Run Control Code (deploy and start).
        assert!(serde_json::from_str::<DeployRequest>(r#"{"build":"b"}"#).unwrap().start);
        let only = DeployRequest { build: None, start: false, stages: None, force: false };
        assert_eq!(serde_json::to_string(&only).unwrap(), r#"{"start":false}"#);
        assert_eq!(serde_json::from_str::<DeployRequest>(r#"{"start":false}"#).unwrap(), only);
        let stages = |s: &[&str]| DeployStatus { stages: s.iter().map(|s| (*s).to_owned()).collect(), ..Default::default() };
        assert!(stages(&DEPLOY_STAGES).starts() && !stages(&["projects", "deploy", "krl"]).starts());
        let deploying = InfraStatus {
            deploy: DeployStatus { state: JobState::Running, stage: Some("projects".into()), ..Default::default() },
            head_plan_id: Some("16#9201".into()),
            ..Default::default()
        };
        assert!(deploying.busy());
        let json = serde_json::to_value(&deploying).unwrap();
        assert_eq!(json["deploy"]["stage"], "projects");
        assert_eq!(json["deploy"]["state"], "running");
        assert_eq!(json["headPlanId"], "16#9201");
        assert_eq!(serde_json::from_value::<InfraStatus>(json).unwrap(), deploying);
        assert!(deploy_stage_label("projects").contains("15 min"));
        assert_eq!(serde_json::from_str::<AutoRestartSettings>(r#"{"enabled":true}"#).unwrap().threshold_s, None);
        // Start cell: older clients send `{}` or `{roles}`.
        assert_eq!(serde_json::to_string(&StartStopRequest::default()).unwrap(), "{}");
        assert_eq!(serde_json::from_str::<StartStopRequest>(r#"{"roles":["plcHost"]}"#).unwrap().bring_up, None);
        assert_eq!(
            serde_json::to_string(&StartStopRequest { roles: None, bring_up: Some(true), production: Some(true) }).unwrap(),
            r#"{"bringUp":true,"production":true}"#
        );
        let start: InstanceStart = serde_json::from_str(
            r#"{"role":"windowsIde","instanceId":"i-1","previousType":"c5a.4xlarge","newType":"c5.4xlarge","result":"fallback","firstError":"InsufficientInstanceCapacity"}"#,
        )
        .unwrap();
        assert_eq!(start.summary(), "Windows IDE: c5a.4xlarge had no capacity, started as c5.4xlarge");
        let failed = InstanceStart { result: "failed".into(), error: Some("VcpuLimitExceeded: x".into()), ..start.clone() };
        assert!(failed.failed() && failed.summary().starts_with("Windows IDE: did not start (c5a.4xlarge): VcpuLimitExceeded"));
        let starting = InfraStatus { start: CellStartStatus { state: JobState::Waiting, ..Default::default() }, ..Default::default() };
        assert!(starting.busy());
        let pending = InfraStatus { instances: vec![InfraInstance { state: "stopping".into(), ..Default::default() }], ..Default::default() };
        assert!(pending.changing() && !InfraStatus::default().changing());
    }

    #[test]
    fn cell_power_reads_the_lambdas_power_status() {
        let power: CellPower = serde_json::from_str(
            r#"{"state":"booting","phase":"bring-up","since":"2026-10-15T08:01:40.000Z","message":"bringing up the KUKA controllers",
                "hosts":[{"role":"kukaVc1","instanceId":"i-1","robotNs":"robot1","state":"running","ssmPing":"Online","online":true,
                          "privateIp":"192.0.2.30","launchTime":"2026-10-15T08:00:00.000Z","tagged":true,"name":"kuka-vc-p2c-1"}],
                "checks":[{"name":"plcRuntime","label":"PLC runtime (codesys-master)","ok":true,"detail":"running","host":"plcHost"},
                          {"name":"mcpBridge","label":"CODESYS MCP bridge","ok":null,"detail":null,"host":null}],
                "checkedAt":null,"session":{"op":"on","by":"plan2cell-test","requestedAt":"2026-10-15T08:00:00.000Z","done":false,
                "history":[{"phase":"starting-instances","at":"2026-10-15T08:00:00.000Z"}]},"idle":{"hours":4,"lastActivityAt":null,
                "lastActivityBy":null},"production":null}"#,
        )
        .unwrap();
        assert_eq!(power.state, PowerState::Booting);
        assert_eq!(power.line(), "Virtual cell: booting (bring-up)");
        assert!(power.hosts[0].online && power.hosts[0].tagged);
        assert_eq!(power.failing().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["mcpBridge"]);
        assert_eq!(power.session.as_ref().unwrap().history[0].phase, "starting-instances");
        assert_eq!(power.idle.as_ref().unwrap().hours, 4.0);
        let status = InfraStatus { cell_power: Some(power.clone()), ..Default::default() };
        assert!(status.power_blocks().unwrap().contains("booting (bring-up)"));
        let json = serde_json::to_value(&status).unwrap();
        assert_eq!(json["cellPower"]["phase"], "bring-up");
        assert_eq!(serde_json::from_value::<InfraStatus>(json).unwrap(), status);
        // An older server: no cellPower, nothing blocks.
        assert!(!serde_json::to_string(&InfraStatus::default()).unwrap().contains("cellPower"));
        assert_eq!(InfraStatus::default().power_blocks(), None);
        let off = InfraStatus { cell_power: Some(CellPower { state: PowerState::Off, phase: "off".into(), ..Default::default() }), ..Default::default() };
        assert!(off.power_blocks().unwrap().contains("Start virtual cell"));
        assert!(!PowerState::Off.on() && !PowerState::Stopping.on() && PowerState::Booting.on() && PowerState::Ready.on());
        // A state from a newer Lambda.
        assert_eq!(serde_json::from_str::<CellPower>(r#"{"state":"hibernating"}"#).unwrap().state, PowerState::Unknown);
        assert_eq!(serde_json::to_string(&CellPowerRequest { action: "off".into(), ..Default::default() }).unwrap(), r#"{"action":"off"}"#);
        assert_eq!(
            serde_json::from_str::<CellPowerRequest>(r#"{"action":"on","production":true}"#).unwrap(),
            CellPowerRequest { action: "on".into(), production: Some(true), ..Default::default() }
        );
        assert!(POWER_PHASES.contains(&"bring-up"));
    }

    #[test]
    fn own_cells_wire_format() {
        assert!(OWN_CELL_START_PHASES.iter().chain(&OWN_CELL_STOP_PHASES).all(|p| POWER_PHASES.contains(p)));
        let cell: VirtualCell = serde_json::from_str(
            r#"{"id":"ws-jdoe","workspace":"jdoe","kind":"own","lifecycle":"on","createdAt":"2026-10-15T08:00:00Z","launch":3,"robots":2,
                "power":{"state":"booting","phase":"configuring"},"cost":{"runningUsdPerHour":2.65,"offUsdPerMonth":0,"sessionUsd":0.4,"note":"n"},
                "images":[{"role":"plcHost","amiId":"ami-1","recipeVersion":"1.0.0","buildDate":null}],
                "restore":{"state":"running","mode":"fast","build":"b","planId":"16#1"},"wasOnAtStop":true}"#,
        )
        .unwrap();
        assert_eq!((cell.kind, cell.lifecycle, cell.launch, cell.phase()), (CellKind::Own, CellLifecycle::On, 3, "configuring"));
        assert_eq!(cell.power.as_ref().unwrap().line(), "Virtual cell: booting (configuring the instances)");
        assert_eq!(cell.restore.as_ref().unwrap().state, JobState::Running);
        assert_eq!(serde_json::from_value::<VirtualCell>(serde_json::to_value(&cell).unwrap()).unwrap(), cell);
        assert_eq!(serde_json::from_str::<VirtualCell>(r#"{"lifecycle":"hibernating"}"#).unwrap().lifecycle, CellLifecycle::Unknown);
        assert_eq!(VirtualCell { lifecycle: CellLifecycle::Deleting, ..Default::default() }.phase(), "deleting");
        assert_eq!(serde_json::to_string(&CellKind::Shared).unwrap(), r#""shared""#);
        let list = CellsList { available: true, max_per_workspace: 1, can_create: true, ..Default::default() };
        let json = serde_json::to_value(&list).unwrap();
        assert_eq!((json["maxPerWorkspace"].as_u64(), json["canCreate"].as_bool()), (Some(1), Some(true)));
        assert_eq!(serde_json::to_string(&CreateCellRequest::default()).unwrap(), "{}");
        assert_eq!(serde_json::from_str::<CreateCellRequest>(r#"{"robots":2,"start":true}"#).unwrap(), CreateCellRequest { robots: Some(2), start: true, ..Default::default() });
        assert_eq!(serde_json::from_str::<DeleteCellRequest>(r#"{"confirm":"ws-jdoe"}"#).unwrap().confirm, "ws-jdoe");
        let status = InfraStatus { virtual_cell: Some(cell), binding: Some(CellKind::Own), ..Default::default() };
        let json = serde_json::to_value(&status).unwrap();
        assert_eq!((json["virtualCell"]["id"].as_str(), json["binding"].as_str()), (Some("ws-jdoe"), Some("own")));
        assert!(!serde_json::to_string(&InfraStatus::default()).unwrap().contains("virtualCell"));
        let req: CellPowerRequest = serde_json::from_str(r#"{"action":"on","restore":"full"}"#).unwrap();
        assert_eq!(req.restore.as_deref(), Some("full"));
        assert!(RESTORE_MODES.contains(&"auto"));
        let stages = DeployRequest { stages: Some(vec!["krl".into(), "start".into()]), ..Default::default() };
        assert_eq!(serde_json::to_string(&stages).unwrap(), r#"{"stages":["krl","start"]}"#);
        assert_eq!(serde_json::from_str::<DeployRequest>("{}").unwrap().stages, None);
    }
}

/// `POST` [`crate::api::INFRA_KVC`]: one operation on one KUKA virtual controller of the active
/// cell's virtual cell (`plan2cell cell kvc`): `status`, `mode` (`arg`: `show`, `T1`, `T2`, `AUT`,
/// `EXT`), `ack`, `restart`, `program` (`arg`: `show`, `cancel`, `select` or `select=<module>`),
/// `braketest-need`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct KvcRequest {
    /// The robot's namespace (`robot1`, …).
    pub robot: String,
    pub op: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arg: Option<String>,
}

impl KvcRequest {
    /// Whether the operation changes the controller (mode, ack, restart, program cancel / select).
    pub fn changes(&self) -> bool {
        match self.op.as_str() {
            "status" | "braketest-need" => false,
            "mode" | "program" => self.arg.as_deref().is_some_and(|a| a != "show"),
            _ => true,
        }
    }

    /// Whether the operation changes the controller, by a template's operations (§4.6): `Err` for an
    /// operation the template does not have, or an argument it does not take (`values` compared ASCII
    /// case folded, as plan2cell takes `mode t1`). For the legacy templates' operations this equals
    /// [`Self::changes`]; for plan2cell's own KSS operations ([`KSS_OPS`]) it is never less than
    /// [`Self::changes`]: a template cannot make `ack` or `mode T1` read-only.
    pub fn changes_for(&self, ops: &[TemplateOp]) -> Result<bool, String> {
        let Some(op) = ops.iter().find(|o| o.name == self.op) else {
            let names: Vec<&str> = ops.iter().map(|o| o.name.as_str()).collect();
            return Err(format!("{}: not an operation of this cell (one of {})", self.op, names.join(", ")));
        };
        let arg = self.arg.as_deref();
        match (&op.arg_schema, arg) {
            (None, Some(a)) => return Err(format!("{} takes no argument, not {a:?}", op.name)),
            (Some(schema), None) if schema.required => return Err(format!("{} needs an argument ({})", op.name, arg_words(schema))),
            (Some(schema), Some(a)) => {
                let prefixed = schema.prefix.as_deref().is_some_and(|p| a.len() > p.len() && a.starts_with(p));
                if !schema.values.iter().any(|v| v.eq_ignore_ascii_case(a)) && !prefixed {
                    return Err(format!("{} {a}: the argument is one of {}", op.name, arg_words(schema)));
                }
            }
            _ => {}
        }
        let template = op.destructive && !arg.is_some_and(|a| op.read_args.iter().any(|r| r == a));
        Ok(template || (KSS_OPS.contains(&self.op.as_str()) && self.changes()))
    }
}

/// The KUKA VC operations plan2cell's KSS driver implements (`vcell_kvc`, [`KvcRequest::changes`]).
pub const KSS_OPS: [&str; 6] = ["status", "mode", "ack", "restart", "program", "braketest-need"];

/// `show, cancel, select or select=<...>`.
fn arg_words(schema: &TemplateOpArgs) -> String {
    let mut words: Vec<String> = schema.values.clone();
    if let Some(p) = &schema.prefix {
        words.push(format!("{p}<...>"));
    }
    words.join(", ")
}

#[cfg(test)]
mod template_tests {
    use super::*;

    /// The legacy templates' operations (docs/tasks/cell-templates.md §5).
    fn legacy_ops() -> Vec<TemplateOp> {
        serde_json::from_str(
            r#"[{"destructive":false,"label":"Status","name":"status"},
                {"argSchema":{"required":true,"values":["show","T1","T2","AUT","EXT"]},"destructive":true,"label":"Operating mode","name":"mode","readArgs":["show"]},
                {"destructive":true,"label":"Acknowledge messages","name":"ack"},
                {"destructive":true,"label":"Restart controller","name":"restart"},
                {"argSchema":{"prefix":"select=","required":true,"values":["show","cancel","select"]},"destructive":true,"label":"Program","name":"program","readArgs":["show"]},
                {"destructive":false,"label":"Brake test due","name":"braketest-need"}]"#,
        )
        .unwrap()
    }

    fn kvc(op: &str, arg: Option<&str>) -> KvcRequest {
        KvcRequest { robot: "robot1".into(), op: op.into(), arg: arg.map(str::to_owned) }
    }

    #[test]
    fn changes_for_the_legacy_ops_equals_changes() {
        let ops = legacy_ops();
        for (op, arg) in [
            ("status", None),
            ("braketest-need", None),
            ("ack", None),
            ("restart", None),
            ("mode", Some("show")),
            ("mode", Some("T1")),
            ("mode", Some("t1")),
            ("mode", Some("aut")),
            ("mode", Some("EXT")),
            ("program", Some("show")),
            ("program", Some("cancel")),
            ("program", Some("select")),
            ("program", Some("select=bt_cell")),
        ] {
            let r = kvc(op, arg);
            assert_eq!(r.changes_for(&ops), Ok(r.changes()), "{op} {arg:?}");
        }
        // What the template does not have is refused (today's path passes it to plan2cell).
        assert!(kvc("format", None).changes_for(&ops).unwrap_err().contains("not an operation of this cell"));
        assert!(kvc("mode", None).changes_for(&ops).unwrap_err().contains("needs an argument"));
        assert!(kvc("mode", Some("T3")).changes_for(&ops).unwrap_err().contains("one of show, T1"));
        assert!(kvc("program", Some("select=")).changes_for(&ops).is_err(), "a prefix without a module");
        assert!(kvc("status", Some("x")).changes_for(&ops).unwrap_err().contains("takes no argument"));
        // A template op of its own: read-only unless destructive, the readArgs read.
        let own: Vec<TemplateOp> = serde_json::from_str(r#"[{"name":"io","label":"I/O","argSchema":{"required":false,"values":["read","pulse"]},"destructive":true,"readArgs":["read"]}]"#).unwrap();
        assert_eq!(kvc("io", Some("read")).changes_for(&own), Ok(false));
        assert_eq!(kvc("io", Some("pulse")).changes_for(&own), Ok(true));
        assert_eq!(kvc("io", None).changes_for(&own), Ok(true), "no argument is not a read argument");
        // plan2cell's own operations: a template cannot make one that changes the controller read-only.
        let lax: Vec<TemplateOp> = serde_json::from_str(
            r#"[{"name":"ack","label":"Ack","destructive":false},{"name":"restart","label":"Restart","destructive":false},
                {"name":"mode","label":"Mode","argSchema":{"required":true,"values":["show","T1"]},"destructive":true,"readArgs":["show","T1"]},
                {"name":"status","label":"Status","destructive":false}]"#,
        )
        .unwrap();
        for (op, arg) in [("ack", None), ("restart", None), ("mode", Some("T1")), ("mode", Some("t1"))] {
            assert_eq!(kvc(op, arg).changes_for(&lax), Ok(true), "{op} {arg:?}");
        }
        assert_eq!((kvc("mode", Some("show")).changes_for(&lax), kvc("status", None).changes_for(&lax)), (Ok(false), Ok(false)));
    }

    #[test]
    fn legacy_capabilities_are_the_legacy_templates() {
        let ec2 = legacy_capabilities(RUNTIME_EC2, None);
        assert_eq!(ec2.len(), 12);
        assert_eq!(legacy_capabilities("", None), ec2, "an older Lambda: EC2");
        let eks = legacy_capabilities(RUNTIME_EKS, Some(IDE_CONTAINER));
        assert!(!eks.contains(&"ide.dcv") && !eks.contains(&"archive") && !eks.contains(&"bringup.action") && eks.contains(&"smartpad"));
        assert!(legacy_capabilities(RUNTIME_EKS, Some(IDE_EC2)).contains(&"ide.dcv"));
        for caps in [&ec2, &eks] {
            assert!(caps.windows(2).all(|w| w[0] < w[1]), "sorted: {caps:?}");
            assert!(caps.iter().all(|c| CAPABILITIES.contains(c)));
        }
        assert!(CAPABILITIES.windows(2).all(|w| w[0] < w[1]));
        assert_eq!((legacy_template(RUNTIME_EKS), legacy_template("")), (TEMPLATE_EKS_CODESYS_KUKA, TEMPLATE_EC2_CODESYS_KUKA));
        assert!(valid_template_id("ec2-codesys-kuka") && !valid_template_id("Ec2") && !valid_template_id("ab") && !valid_template_id("1ab"));
        assert!(valid_template_version("1.0.0") && valid_template_version("0.12.3") && !valid_template_version("1.0") && !valid_template_version("01.0.0"));
        assert!(!valid_template_version("1.0.0-rc1") && !valid_template_version("1.0.0+b"));
        assert!(valid_param_name("offMode") && !valid_param_name("OffMode") && !valid_param_name("off_mode") && !valid_param_name(""));
    }

    /// The contract's examples (§8) read; an older Lambda's (no template) still does, and writes as before.
    #[test]
    fn templates_wire_format() {
        let list: CellTemplates = serde_json::from_str(
            r#"{"ok":true,"action":"templates","engine":1,"enabled":true,"revision":1,"source":"s3","templates":[
                {"id":"eks-codesys-kuka","version":"1.0.0","channel":"stable","source":"s3","title":"CODESYS head PLC and KUKA VCs on EKS",
                 "description":"The EKS virtual cell","status":"active","runtime":"eks",
                 "parameters":{"ide":{"default":"container","label":"IDE","mutable":"off","type":"enum","values":["container","ec2"]},
                   "networks":{"default":[{"name":"n1","robots":2}],"label":"PROFINET networks","max":4,"min":1,"mutable":"off","robots":{"default":2,"max":6,"min":1},"type":"networks"}},
                 "capabilities":[{"name":"check.kss-config"},{"name":"ide.dcv","onlyIf":{"param":"ide","equals":"ec2"}}],
                 "consumer":{"deploy":{"stages":["projects","deploy","krl","start"]},"ops":[]},
                 "cost":{"runningUsdPerHour":2.72,"note":"eu-central-1 on-demand"},"agentNotes":"Head PLC per network","available":true},
                {"id":"m1-probe","version":"0.0.1","available":false,"reason":"engine"}]}"#,
        )
        .unwrap();
        assert_eq!((list.enabled, list.engine, list.revision, list.source.as_str()), (true, 1, Some(1), "s3"));
        let eks = &list.templates[0];
        assert_eq!(eks.parameters["networks"].robots, Some(NetworkRobots { min: 1, max: 6, default: 2 }));
        assert_eq!(eks.parameters["ide"].values, ["container", "ec2"]);
        assert_eq!(eks.capabilities[1].only_if.as_ref().unwrap().equals, "ec2");
        assert_eq!(eks.consumer.deploy.stages, DEPLOY_STAGES);
        assert!(!eks.beyond_consumer());
        assert_eq!((list.templates[1].available, list.templates[1].reason.as_deref()), (false, Some("engine")));
        let json = serde_json::to_value(&list.templates[0]).unwrap();
        assert_eq!(json["capabilities"][1]["onlyIf"]["param"], "ide");
        assert_eq!(json["parameters"]["ide"]["type"], "enum");
        assert!(json["parameters"]["ide"].get("robots").is_none() && json.get("reason").is_none());
        let newer = CellTemplateInfo { requires: Some(TemplateRequires { engine: 1, kinds: vec![], consumer: CONSUMER_LEVEL + 1, hook: None }), ..Default::default() };
        assert!(newer.beyond_consumer());
        // What the Lambda does not know is `null` (a version it has not loaded, no cost): read as empty.
        let unloaded: CellTemplateInfo = serde_json::from_str(
            r#"{"id":"m1-probe","version":null,"channel":null,"source":null,"title":"m1-probe","description":"","status":null,
                "runtime":null,"parameters":{},"capabilities":[],"consumer":null,"cost":null,"agentNotes":null,"requires":null,
                "available":false,"reason":"engine"}"#,
        )
        .unwrap();
        assert_eq!((unloaded.id.as_str(), unloaded.version.as_str(), unloaded.reason.as_deref()), ("m1-probe", "", Some("engine")));
        assert_eq!((unloaded.consumer, unloaded.cost, unloaded.requires), (TemplateConsumer::default(), TemplateCostHint::default(), None));
        let no_cost: CellTemplateInfo =
            serde_json::from_value(serde_json::json!({"id": "ec2-codesys-kuka", "version": "1.0.0", "cost": null, "consumer": {"deploy": null, "build": null, "ops": null}, "available": true}))
                .unwrap();
        assert!(no_cost.available && no_cost.consumer.ops.is_empty() && no_cost.cost.running_usd_per_hour == 0.0);

        let status = serde_json::json!({
            "id": "eks-codesys-kuka", "version": "1.0.0", "sha256": "5a1b25bfe8e1f22af82945b9d35752a379d947474033c4571b63d1ddc0870658",
            "source": "s3", "channel": "stable", "launch": 7,
            "parameters": {"ide": "container", "networks": [{"name": "n1", "robots": 4}], "offMode": "terminate"},
            "capabilities": ["check.kss-config", "deploy.head.codesys-mcp", "deploy.robot.kss", "live.head.opcua",
                "live.robot.vass-zenoh", "runtime.codesys", "smartpad"],
            "consumer": {"deploy": {"stages": ["projects", "deploy", "krl", "start"]}, "build": {}, "ops": [], "e2e": {}},
            "semantics": {"n2/plcHost": "head-plc", "n1/plcHost": "head-plc", "n1/kukaVc1": "robot-controller", "compile": "compile"}
        });
        let t: CellTemplateRef = serde_json::from_value(status).unwrap();
        assert!(t.has("smartpad") && !t.has("ide.dcv") && !t.has("bringup.action"));
        assert_eq!((t.semantic("n1/kukaVc1"), t.semantic("windowsIde")), (Some("robot-controller"), None));
        assert_eq!((t.role_of("head-plc"), t.role_of("ide")), (Some("n1/plcHost"), None), "the first network's head");
        assert_eq!((t.label(), t.launch), ("eks-codesys-kuka@1.0.0".to_owned(), Some(7)));

        // A cell with a template: DCV and the rest by its capabilities; without, today's rules.
        let ec2 = VirtualCell { runtime: RUNTIME_EC2.into(), ..Default::default() };
        assert!(ec2.has_dcv() && ec2.has("restore") && ec2.has("offmode.stop"));
        let headless = VirtualCell { runtime: RUNTIME_EKS.into(), ide: Some(IDE_CONTAINER.into()), ..Default::default() };
        assert!(!headless.has_dcv() && !headless.has("archive"));
        let templated = VirtualCell { template: Some(t.clone()), ..ec2.clone() };
        assert!(!templated.has_dcv() && templated.has("smartpad") && !templated.has("restore"));
        // An older Lambda's cell, and an older backend's list: nothing new is written.
        let old: VirtualCell = serde_json::from_str(r#"{"id":"ws-jdoe","runtime":"ec2"}"#).unwrap();
        assert_eq!((old.template.as_ref(), old.params.as_ref()), (None, None));
        let json = serde_json::to_string(&old).unwrap();
        assert!(!json.contains("template") && !json.contains("params"), "{json}");
        let json = serde_json::to_string(&CellsList::default()).unwrap();
        assert!(!json.contains("templates"), "{json}");
        assert!(!serde_json::to_string(&CellResume::default()).unwrap().contains("newerTemplate"));
        let resume: CellResume = serde_json::from_str(r#"{"possible":true,"newerTemplate":{"id":"ec2-codesys-kuka","running":"1.0.0","latest":"1.1.0"}}"#).unwrap();
        assert_eq!(resume.newer_template.unwrap().latest, "1.1.0");
        let back: VirtualCell = serde_json::from_value(serde_json::to_value(&templated).unwrap()).unwrap();
        assert_eq!(back, templated);
    }

    #[test]
    fn create_and_update_take_a_template_and_params() {
        let create: CreateCellRequest =
            serde_json::from_str(r#"{"template":"eks-codesys-kuka","params":{"networks":[{"name":"n1","robots":4}],"ide":"ec2"}}"#).unwrap();
        assert_eq!(create.template, Some(TemplateSelector::Id("eks-codesys-kuka".into())));
        assert_eq!(create.params.as_ref().unwrap()["ide"], "ec2");
        assert!(!create.has_legacy_fields());
        let pinned: CreateCellRequest = serde_json::from_str(r#"{"template":{"id":"ec2-codesys-kuka","version":"1.0.0"},"params":{"robots":3}}"#).unwrap();
        let selector = pinned.template.clone().unwrap();
        assert_eq!((selector.id(), selector.version()), ("ec2-codesys-kuka", Some("1.0.0")));
        assert_eq!(
            serde_json::to_value(&pinned).unwrap(),
            serde_json::json!({"template": {"id": "ec2-codesys-kuka", "version": "1.0.0"}, "params": {"robots": 3}})
        );
        assert_eq!(serde_json::to_string(&CreateCellRequest::default()).unwrap(), "{}", "an older backend's body");
        assert!(CreateCellRequest { robots: Some(2), ..Default::default() }.has_legacy_fields());
        let update: UpdateCellRequest = serde_json::from_str(r#"{"params":{"offMode":"stop"}}"#).unwrap();
        assert!(update.template.is_none() && !update.has_legacy_fields());
        assert_eq!(serde_json::to_string(&UpdateCellRequest { template: Some(TemplateSelector::Id("ec2-codesys-kuka".into())), ..Default::default() }).unwrap(), r#"{"template":"ec2-codesys-kuka"}"#);
        assert!(serde_json::from_str::<CreateCellRequest>(r#"{"template":3}"#).is_err());
    }

    /// Engine 2 (docs/tasks/cell-templates.md E0, E12): this backend renders the hook drivers (level 3), an
    /// engine-2 template's `requires.hook` reads and writes back, and its cell is a hook cell.
    #[test]
    fn an_engine_2_template_is_within_this_consumer_level() {
        assert_eq!(CONSUMER_LEVEL, 3);
        assert!(HOOK_CAPABILITIES.iter().all(|c| CAPABILITIES.contains(c)) && HOOK_CAPABILITIES.is_sorted());
        let smoke: CellTemplateInfo = serde_json::from_str(
            r#"{"id":"eks-generic-smoke","version":"0.1.0","channel":"preview","source":"s3","runtime":"eks",
                "capabilities":[{"name":"check.hook"},{"name":"deploy.head.hook"},{"name":"deploy.robot.hook"},{"name":"live.head.exec"},{"name":"live.robot.exec"},{"name":"runtime.hook"}],
                "consumer":{"deploy":{"stages":["deploy","krl","start"]},"e2e":{"expect":{"head":"CS_RUNNING"}}},
                "requires":{"consumer":3,"engine":2,"hook":"factoryos/hook@1","kinds":["pod@1"]},"available":true}"#,
        )
        .unwrap();
        let requires = smoke.requires.as_ref().unwrap();
        assert_eq!((requires.engine, requires.consumer, requires.hook.as_deref()), (2, 3, Some(HOOK_CONTRACT)));
        assert!(!smoke.beyond_consumer());
        assert_eq!(serde_json::to_value(&smoke).unwrap()["requires"]["hook"], HOOK_CONTRACT);
        // An engine-1 template's requires serialises as before (no `hook`).
        let legacy = TemplateRequires { engine: 1, kinds: vec!["builtin:ec2-cell@1".into()], consumer: 1, hook: None };
        assert_eq!(serde_json::to_string(&legacy).unwrap(), r#"{"engine":1,"kinds":["builtin:ec2-cell@1"],"consumer":1}"#);
        let cell = |caps: &[&str]| CellTemplateRef { capabilities: caps.iter().map(|c| (*c).to_owned()).collect(), ..Default::default() };
        assert!(cell(&["check.hook", "deploy.head.hook"]).hook());
        assert!(cell(&["deploy.head.codesys-mcp", "deploy.robot.hook"]).hook(), "a codesys head with hook robots");
        let legacy_eks: Vec<&str> = legacy_capabilities(RUNTIME_EKS, Some(IDE_EC2));
        assert!(!cell(&legacy_eks).hook() && !cell(&legacy_capabilities(RUNTIME_EC2, None)).hook());
    }
}
