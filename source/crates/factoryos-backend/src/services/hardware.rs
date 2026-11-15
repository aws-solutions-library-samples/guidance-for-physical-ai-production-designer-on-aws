//! Hardware Configuration of the active cell: the head PLC's network, each robot's PROFINET
//! station name, device name and IP, and the KUKA virtual controllers' KLI addresses.
//!
//! * The defaults are plan2cell's own: `plan2cell hardware <plan> -c <cell>/plan2cell/plan2cell.toml
//!   --json` (bt2plc's addressing for that plan, with the cell's config on top), for the plan the
//!   export form has chosen or the newest one.
//! * Saved values live in `<cell>/plan2cell/hardware.json` ([`HARDWARE_FILE`]). Every Control Code
//!   Export writes them into its build's plan2cell.toml ([`apply_to_plan2cell_toml`]: `[plc.network]`
//!   and `[[plc.robot]]` station / device_name / ip), and `build.json` records them.
//! * On a server connected to a virtual cell (`/etc/plan2cell/cell.toml`, see
//!   `plan2cell/infra/test-server/`), saving also renders `<cell>/plan2cell/cell.toml` ([`CELL_CONFIG`]):
//!   that file with each `[[kuka_vc.controller]]`'s `kli_ip` / `station` from the configuration. The
//!   Virtual Cell's "Start production" uses it. Without `/etc/plan2cell/cell.toml` nothing is rendered.
//!   Both files are written through to the user's store (S3 mode, [`crate::storage::saved`]).
//!
//! * Defaults on a plan2cell virtual cell ([`VirtualCell`]): the robots there are KUKA VCs, whose
//!   GSDML DAP `DIM 3` has no safety slot. Each of them defaults to `external = true`,
//!   `safety_bypass = true`, `safety_module = false`, and the head's gateway to
//!   [`VIRTUAL_CELL_GATEWAY`] (the PLC host's PROFINET bridge), wherever the cell's plan2cell.toml
//!   sets none of these ([`with_virtual_cell_defaults`]; the same base every export of the cell gets).
//!   Without a virtual cell (real hardware) plan2cell's own defaults stay: `external = false`,
//!   `safety_bypass = false`, `safety_module = true`, no gateway. A saved configuration is never
//!   changed: [`HardwareConfig::virtual_cell_warnings`] says what will not work there (the form
//!   offers the virtual cell's values in one click).
//!
//! The virtual cell is wired for the default addresses: other values change the generated code
//! (and the rendered cell.toml), but the cell itself is not re-addressed; the view's warnings say
//! where the values differ from the cell's wiring.

use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use anyhow::Context as _;
use axum::extract::{Query, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse as _, Response};
use axum::routing::get;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::hardware::{
    CellWiring, FieldIssue, HardwareConfig, HardwareRejected, HardwareSave, HardwareSource, HardwareView, KukaVcController,
    NetworkConfig, RobotHardware, VIRTUAL_CELL_GATEWAY, VIRTUAL_CELL_NOTE,
};
use tokio::process::Command;

use crate::AppState;
use crate::config_store::ConfigStore;
use crate::services::{cells, codegen};

/// The saved configuration, relative to the cell.
pub const HARDWARE_FILE: &str = "plan2cell/hardware.json";
/// The `plan2cell cell` config rendered from the server's and the hardware, relative to the cell.
pub const CELL_CONFIG: &str = "plan2cell/cell.toml";
/// The server's own `plan2cell cell` config (a test server connected to the virtual cell).
pub const SERVER_CELL_CONFIG: &str = "/etc/plan2cell/cell.toml";
const SCHEMA: &str = "factoryos/hardware@1";
/// `plan2cell hardware` reads the plan and the config: a second at most.
const CLI_TIMEOUT: Duration = Duration::from_secs(30);
const MAX_SAVED_BYTES: u64 = 1 << 20;

type HttpError = (StatusCode, String);

/// `hardware.json`.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Saved {
    pub schema: String,
    pub saved_s: f64,
    /// The plan it was checked against.
    pub plan: String,
    pub config: HardwareConfig,
}

/// The cell's saved hardware configuration and its ETag, if it has one that reads.
pub fn read_saved(root: &Path) -> Option<(Saved, String)> {
    let file = root.join(HARDWARE_FILE);
    let meta = std::fs::metadata(&file).ok().filter(|m| m.len() <= MAX_SAVED_BYTES)?;
    let saved: Saved = serde_json::from_slice(&std::fs::read(&file).ok()?).ok()?;
    Some((saved, cells::plan_etag(&meta)))
}

/// The ETag of the cell's `hardware.json`, if it has one (for the builds' stale badge).
pub fn saved_etag(root: &Path) -> Option<String> {
    std::fs::metadata(root.join(HARDWARE_FILE)).ok().map(|m| cells::plan_etag(&m))
}

/// The server's `plan2cell cell` config: `FACTORY_OS_PLAN2CELL_CELL_CONFIG`, else [`SERVER_CELL_CONFIG`].
pub fn server_cell_config() -> PathBuf {
    std::env::var_os("FACTORY_OS_PLAN2CELL_CELL_CONFIG")
        .filter(|p| !p.is_empty())
        .map_or_else(|| PathBuf::from(SERVER_CELL_CONFIG), PathBuf::from)
}

/// Whether this server's cell is a plan2cell virtual cell, and which robots of a plan are its KUKA
/// VCs (the rule of the virtual cell's hardware defaults): it is one when
/// `FACTORY_OS_PLAN2CELL_CELL_CONFIG` is set (a workspace with its own cell, whose wiring is written
/// once the cell is up) or the server's `plan2cell cell` config exists (`/etc/plan2cell/cell.toml`,
/// the shared test server). Its KUKA VCs: the plan's robots that config lists as
/// `[[kuka_vc.controller]] robot_ns`, every robot of the plan while it lists none (or cannot be read).
#[derive(Clone, Debug, Default)]
pub struct VirtualCell {
    config: PathBuf,
    configured: bool,
}

impl VirtualCell {
    /// This server's: [`server_cell_config`], and whether the environment names one.
    pub fn from_env() -> Self {
        let configured = std::env::var_os("FACTORY_OS_PLAN2CELL_CELL_CONFIG").is_some_and(|p| !p.is_empty());
        Self { config: server_cell_config(), configured }
    }

    /// One there while `config` exists (tests; `/etc/plan2cell/cell.toml`).
    pub fn at(config: PathBuf) -> Self {
        Self { config, configured: false }
    }

    /// No virtual cell: real hardware.
    pub fn none() -> Self {
        Self::default()
    }

    pub fn active(&self) -> bool {
        self.configured || (!self.config.as_os_str().is_empty() && self.config.is_file())
    }

    /// The robots of `plan_robots` that are KUKA VCs of the cell (empty: no virtual cell).
    pub fn robots(&self, plan_robots: &[String]) -> Vec<String> {
        if !self.active() {
            return Vec::new();
        }
        let listed: Vec<String> = std::fs::read_to_string(&self.config)
            .ok()
            .and_then(|text| text.parse::<toml::Table>().ok())
            .and_then(|doc| {
                let controllers = doc.get("kuka_vc")?.get("controller")?.as_array()?.clone();
                Some(controllers.iter().filter_map(|c| Some(c.get("robot_ns")?.as_str()?.to_owned())).collect())
            })
            .unwrap_or_default();
        plan_robots.iter().filter(|ns| listed.is_empty() || listed.contains(ns)).cloned().collect()
    }
}

/// What the virtual cell's own config says about its PROFINET addressing: `[profinet]` (`head_ip`,
/// `gateway`, `netmask`: an EKS network's 10.77.<k>.0/24, else unset) and each `[[kuka_vc.controller]]`'s
/// `robot_ns`, `station`, `kli_ip`: the station and address that controller (the one that runs that
/// robot's KRL) has, so the head must address that robot so.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Addressing {
    pub head_ip: Option<String>,
    pub gateway: Option<String>,
    pub netmask: Option<String>,
    /// `(robot_ns, station, kli_ip)`.
    pub controllers: Vec<(String, String, String)>,
}

impl VirtualCell {
    /// The config's [`Addressing`] (empty without a virtual cell or a readable config).
    pub fn addressing(&self) -> Addressing {
        if !self.active() {
            return Addressing::default();
        }
        let Some(doc) = std::fs::read_to_string(&self.config).ok().and_then(|t| t.parse::<toml::Table>().ok()) else {
            return Addressing::default();
        };
        let get = |t: &toml::Table, k: &str| t.get(k).and_then(|v| v.as_str()).filter(|v| !v.is_empty()).map(str::to_owned);
        let profinet = doc.get("profinet").and_then(|p| p.as_table()).cloned().unwrap_or_default();
        let controllers = doc
            .get("kuka_vc")
            .and_then(|k| k.get("controller"))
            .and_then(|c| c.as_array())
            .map(|list| {
                list.iter()
                    .filter_map(|c| c.as_table())
                    .filter_map(|c| Some((get(c, "robot_ns")?, get(c, "station")?, get(c, "kli_ip")?)))
                    .collect()
            })
            .unwrap_or_default();
        Addressing {
            head_ip: get(&profinet, "head_ip"),
            gateway: get(&profinet, "gateway"),
            netmask: get(&profinet, "netmask"),
            controllers,
        }
    }
}

pub struct HardwareService {
    config: Arc<ConfigStore>,
    binary: Option<PathBuf>,
    lookup: Option<fn() -> Option<PathBuf>>,
    server_cell_config: PathBuf,
    virtual_cell: VirtualCell,
}

impl HardwareService {
    pub fn new(config: Arc<ConfigStore>) -> Arc<Self> {
        Arc::new(Self {
            config,
            binary: None,
            lookup: Some(crate::paths::plan2cell_binary),
            server_cell_config: server_cell_config(),
            virtual_cell: VirtualCell::from_env(),
        })
    }

    /// With this `binary` and server cell config (tests).
    pub fn with(config: Arc<ConfigStore>, binary: Option<PathBuf>, server_cell_config: PathBuf) -> Arc<Self> {
        let virtual_cell = VirtualCell::at(server_cell_config.clone());
        Arc::new(Self { config, binary, lookup: None, server_cell_config, virtual_cell })
    }

    fn binary(&self) -> Result<PathBuf, HttpError> {
        let binary = self.binary.clone().or_else(|| self.lookup.and_then(|lookup| lookup())).ok_or_else(|| {
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

    /// The active cell, and the plan `plan` names (`None`/empty: the newest).
    fn cell_and_plan(&self, plan: Option<&str>) -> Result<(String, PathBuf, String, PathBuf), HttpError> {
        let (cell, root) = cells::active_cell(&self.config)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?;
        let plan = match plan.filter(|p| !p.trim().is_empty()) {
            Some(plan) => plan.trim().to_owned(),
            None => cells::plans(&self.config).plans.first().map(|p| p.path.clone()).ok_or_else(|| {
                (
                    StatusCode::NOT_FOUND,
                    format!("cell {cell} has no plans yet: the robots come from a plan (Agentic Planning → Plan Cell)"),
                )
            })?,
        };
        let (rel, file) = codegen::resolve_plan(&root, &plan).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?;
        Ok((cell, root, rel, file))
    }

    /// The effective configuration for `plan`, with the defaults and what differs from the cell.
    pub async fn view(&self, plan: Option<&str>) -> Result<HardwareView, HttpError> {
        let (cell, root, plan_rel, plan_file) = self.cell_and_plan(plan)?;
        let base = read_base(&root)?;
        let (report, _, vc_robots) = defaults_report(&self.binary()?, &self.virtual_cell, &plan_file, base.as_deref()).await?;
        let defaults = report.config();
        let saved = read_saved(&root);
        let (source, saved_s, config) = match &saved {
            Some((saved, _)) => (HardwareSource::Saved, Some(saved.saved_s), overlay(&defaults, &saved.config)),
            None => (HardwareSource::Defaults, None, defaults.clone()),
        };
        let mut errors = config.validate();
        if source == HardwareSource::Defaults {
            // What plan2cell itself finds about the cell's own config (a device name, say).
            for issue in report.issues {
                if !errors.iter().any(|e| e.field == issue.field) {
                    errors.push(issue);
                }
            }
        }
        let wiring = self.wiring(&report.network_defaults);
        let mut warnings = config.wiring_warnings(wiring.as_ref());
        warnings.extend(config.virtual_cell_warnings(&vc_robots));
        let rendered = root.join(CELL_CONFIG);
        Ok(HardwareView {
            cell,
            plan: plan_rel,
            source,
            saved_s,
            etag: saved.as_ref().and_then(|_| saved_etag(&root)),
            config,
            defaults,
            errors,
            warnings,
            wiring,
            cell_config: rendered.is_file().then(|| rendered.display().to_string()),
            virtual_cell_robots: vc_robots,
            note: VIRTUAL_CELL_NOTE.to_owned(),
        })
    }

    /// Check and save `request`: `422` with every problem (the form's rules, then plan2cell's own
    /// on the build config it makes), else `hardware.json` (and the cell.toml) and the new view.
    pub async fn save(&self, request: HardwareSave) -> Result<HardwareView, Response> {
        let http = |(code, message): HttpError| (code, message).into_response();
        let rejected = |errors: Vec<FieldIssue>| (StatusCode::UNPROCESSABLE_ENTITY, Json(HardwareRejected { errors })).into_response();
        let (_, root, plan_rel, plan_file) = self.cell_and_plan(Some(&request.plan)).map_err(http)?;
        let base = read_base(&root).map_err(http)?;
        let binary = self.binary().map_err(http)?;
        let (report, base, _) = defaults_report(&binary, &self.virtual_cell, &plan_file, base.as_deref()).await.map_err(http)?;
        let defaults = report.config();
        let unknown: Vec<FieldIssue> = request
            .config
            .robots
            .iter()
            .filter(|r| defaults.robot(&r.ns).is_none())
            .map(|r| FieldIssue::new(format!("robots.{}", r.ns), format!("{} is not a robot of {plan_rel}", r.ns)))
            .collect();
        if !unknown.is_empty() {
            return Err(rejected(unknown));
        }
        let config = overlay(&defaults, &trimmed(&request.config));
        let errors = config.validate();
        if !errors.is_empty() {
            return Err(rejected(errors));
        }
        // plan2cell's own verdict on the config a build will get.
        let mut doc = parse_base(base.as_deref()).map_err(http)?;
        apply_to_plan2cell_toml(&mut doc, &config).map_err(|err| http((StatusCode::UNPROCESSABLE_ENTITY, format!("{err:#}"))))?;
        let merged = toml::to_string_pretty(&doc).map_err(|err| http((StatusCode::INTERNAL_SERVER_ERROR, err.to_string())))?;
        let report = self.cli(&plan_file, &merged).await.map_err(http)?;
        if !report.issues.is_empty() {
            return Err(rejected(report.issues));
        }
        let saved = Saved { schema: SCHEMA.to_owned(), saved_s: unix_s(SystemTime::now()), plan: plan_rel.clone(), config };
        let json = serde_json::to_vec_pretty(&saved).map_err(|err| http((StatusCode::INTERNAL_SERVER_ERROR, err.to_string())))?;
        write_atomic(&root.join(HARDWARE_FILE), &json).map_err(|err| http((StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}"))))?;
        crate::storage::saved(root.join(HARDWARE_FILE));
        if let Err(err) = self.render_cell_config(&root, &saved.config) {
            tracing::warn!("Hardware Configuration: rendering {CELL_CONFIG}: {err:#}");
        }
        self.view(Some(&plan_rel)).await.map_err(http)
    }

    /// The connected virtual cell's wiring: the server's cell.toml controllers and plan2cell's
    /// default head address (the cell's PLC host is set up for it). `None` without that file.
    fn wiring(&self, network_defaults: &cli::Network) -> Option<CellWiring> {
        let text = std::fs::read_to_string(&self.server_cell_config).ok()?;
        let doc: toml::Table = text.parse().ok()?;
        let controllers = doc
            .get("kuka_vc")
            .and_then(|k| k.get("controller"))
            .and_then(|c| c.as_array())
            .into_iter()
            .flatten()
            .filter_map(|c| {
                Some(KukaVcController {
                    robot_ns: c.get("robot_ns")?.as_str()?.to_owned(),
                    kli_ip: c.get("kli_ip").and_then(|v| v.as_str()).unwrap_or_default().to_owned(),
                    station: c.get("station").and_then(|v| v.as_str()).unwrap_or_default().to_owned(),
                })
            })
            .collect();
        Some(CellWiring {
            source: self.server_cell_config.display().to_string(),
            head_ip: network_defaults.head_ip.clone(),
            netmask: network_defaults.netmask.clone(),
            controllers,
        })
    }

    /// `<cell>/plan2cell/cell.toml` from the server's with `hardware`'s controllers, if the server
    /// has one; returns the file written.
    pub fn render_cell_config(&self, root: &Path, hardware: &HardwareConfig) -> anyhow::Result<Option<PathBuf>> {
        let Ok(text) = std::fs::read_to_string(&self.server_cell_config) else { return Ok(None) };
        let rendered = render_cell_config(&text, &self.server_cell_config, hardware)?;
        let file = root.join(CELL_CONFIG);
        write_atomic(&file, rendered.as_bytes())?;
        crate::storage::saved(&file);
        Ok(Some(file))
    }

    /// The server's `plan2cell cell` config, if this server is connected to a virtual cell (as it
    /// is: a cell's rendered one differs in its KUKA VC addresses only).
    pub fn server_cell_config_path(&self) -> Option<&Path> {
        self.server_cell_config.is_file().then_some(self.server_cell_config.as_path())
    }

    /// The `plan2cell cell` config for the active cell `root`: rendered again from the server's
    /// (it may have changed) when the cell has saved hardware, else the server's own. `None`: there
    /// is no virtual cell on this server.
    pub fn cell_config_for(&self, root: &Path) -> anyhow::Result<Option<PathBuf>> {
        if !self.server_cell_config.is_file() {
            return Ok(None);
        }
        match read_saved(root) {
            Some((saved, _)) => self.render_cell_config(root, &saved.config),
            None => Ok(Some(self.server_cell_config.clone())),
        }
    }

    /// `plan2cell hardware <plan> -c <config> --json`: its report (exit 0 or 3 both print one).
    async fn cli(&self, plan: &Path, config_toml: &str) -> Result<cli::Report, HttpError> {
        report(&self.binary()?, plan, config_toml).await
    }
}

/// The saved hardware configuration for an export of `plan_rel` (`plan_file`), and its ETag;
/// `None` when nothing is saved. Saved for this plan: as saved (checked then). Saved for another
/// plan: over this plan's own defaults (its robots only; the others get plan2cell's values) and
/// checked again, form rules and plan2cell's, so a clash (say a saved IP that is another robot's
/// default here) is a `422` naming the Hardware Configuration, not a failed build.
pub async fn for_export(
    binary: &Path,
    virtual_cell: &VirtualCell,
    root: &Path,
    plan_rel: &str,
    plan_file: &Path,
) -> Result<Option<(HardwareConfig, String)>, HttpError> {
    let Some((saved, etag)) = read_saved(root) else { return Ok(None) };
    if saved.plan == plan_rel {
        return Ok(Some((saved.config, etag)));
    }
    let base = read_base(root)?;
    let (first, base, _) = defaults_report(binary, virtual_cell, plan_file, base.as_deref()).await?;
    let defaults = first.config();
    let config = overlay(&defaults, &saved.config);
    let mut problems: Vec<FieldIssue> = config.validate();
    if problems.is_empty() {
        let mut doc = parse_base(base.as_deref())?;
        apply_to_plan2cell_toml(&mut doc, &config).map_err(|err| (StatusCode::UNPROCESSABLE_ENTITY, format!("{err:#}")))?;
        let merged = toml::to_string_pretty(&doc).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
        problems = report(binary, plan_file, &merged).await?.issues;
    }
    if !problems.is_empty() {
        let list: Vec<String> = problems.iter().map(|i| format!("{}: {}", i.field, i.message)).collect();
        return Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            format!(
                "the saved Hardware Configuration (checked for {}) does not fit {plan_rel}: {}. \
                 Open Generate Hardware Config for this plan, fix and save it",
                saved.plan,
                list.join("; ")
            ),
        ));
    }
    Ok(Some((config, etag)))
}

/// The base of an export of `plan_file` from the cell at `root`: the cell's plan2cell.toml (if any),
/// on a virtual cell with its KUKA VCs' values where it sets none ([`with_virtual_cell_defaults`]),
/// and those robots (empty: the cell's file as it is).
pub async fn export_base(
    binary: &Path,
    virtual_cell: &VirtualCell,
    root: &Path,
    plan_file: &Path,
) -> Result<(Option<String>, Vec<String>), HttpError> {
    let base = read_base(root)?;
    if !virtual_cell.active() {
        return Ok((base, Vec::new()));
    }
    let (_, base, robots) = defaults_report(binary, virtual_cell, plan_file, base.as_deref()).await?;
    Ok((base, robots))
}

/// plan2cell's report for `plan` with the cell's `base`; on a virtual cell, again with its KUKA
/// VCs' values filled in ([`with_virtual_cell_defaults`]). Also the base it used and those robots.
async fn defaults_report(
    binary: &Path,
    virtual_cell: &VirtualCell,
    plan: &Path,
    base: Option<&str>,
) -> Result<(cli::Report, Option<String>, Vec<String>), HttpError> {
    let first = report(binary, plan, base.unwrap_or("")).await?;
    let robots = virtual_cell.robots(&first.robot_ns());
    if robots.is_empty() {
        return Ok((first, base.map(str::to_owned), robots));
    }
    let filled = with_virtual_cell_defaults(base, &robots, &virtual_cell.addressing())?;
    let again = report(binary, plan, &filled).await?;
    Ok((again, Some(filled), robots))
}

/// The cell's plan2cell.toml `base` with what a plan2cell virtual cell needs, where it sets none:
/// each of `robots` (its KUKA VCs) gets a `[[plc.robot]]` `external = true`, `safety_bypass = true`,
/// `safety_module = false`, and the `station` / `device_name` / `ip` of the controller that runs its
/// KRL (`addressing`: robotN's controller, not the robot's position in the plan); `[plc.network]` the
/// cell's `head_ip` / `netmask` / `gateway` (else the gateway [`VIRTUAL_CELL_GATEWAY`]). A value the
/// file has stays (the cell's own choice).
pub fn with_virtual_cell_defaults(base: Option<&str>, robots: &[String], addressing: &Addressing) -> Result<String, HttpError> {
    use toml::{Table, Value};

    let invalid = |what: &str| (StatusCode::UNPROCESSABLE_ENTITY, format!("the cell's {}: {what}", codegen::BASE_CONFIG));
    let mut doc = parse_base(base)?;
    let plc = doc.entry("plc").or_insert_with(|| Value::Table(Table::new())).as_table_mut().ok_or_else(|| invalid("[plc] is not a table"))?;
    let network = plc
        .entry("network")
        .or_insert_with(|| Value::Table(Table::new()))
        .as_table_mut()
        .ok_or_else(|| invalid("[plc.network] is not a table"))?;
    network
        .entry("gateway")
        .or_insert_with(|| Value::String(addressing.gateway.clone().unwrap_or_else(|| VIRTUAL_CELL_GATEWAY.to_owned())));
    if let Some(v) = &addressing.head_ip {
        network.entry("head_ip").or_insert_with(|| Value::String(v.clone()));
    }
    if let Some(v) = &addressing.netmask {
        network.entry("netmask").or_insert_with(|| Value::String(v.clone()));
    }
    let list = plc
        .entry("robot")
        .or_insert_with(|| Value::Array(Vec::new()))
        .as_array_mut()
        .ok_or_else(|| invalid("[[plc.robot]] is not an array of tables"))?;
    for ns in robots {
        let index = match list.iter().position(|t| t.get("ns").and_then(Value::as_str) == Some(ns.as_str())) {
            Some(index) => index,
            None => {
                let mut table = Table::new();
                table.insert("ns".into(), Value::String(ns.clone()));
                list.push(Value::Table(table));
                list.len() - 1
            }
        };
        let table = list[index].as_table_mut().ok_or_else(|| invalid("[[plc.robot]] is not an array of tables"))?;
        table.entry("external").or_insert(Value::Boolean(true));
        table.entry("safety_bypass").or_insert(Value::Boolean(true));
        table.entry("safety_module").or_insert(Value::Boolean(false));
        if let Some((_, station, ip)) = addressing.controllers.iter().find(|(c, _, _)| c == ns) {
            table.entry("station").or_insert_with(|| Value::String(station.clone()));
            table.entry("device_name").or_insert_with(|| Value::String(station.replace('-', "_")));
            table.entry("ip").or_insert_with(|| Value::String(ip.clone()));
        }
    }
    toml::to_string_pretty(&doc).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))
}

/// `plan2cell hardware <plan> -c <config> --json` with `binary`: its report (exit 0 or 3 both print one).
async fn report(binary: &Path, plan: &Path, config_toml: &str) -> Result<cli::Report, HttpError> {
    let unavailable = |err: String| (StatusCode::SERVICE_UNAVAILABLE, err);
    let tmp = TempFile::new("hardware", "toml", config_toml.as_bytes())
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}")))?;
    let mut command = Command::new(binary);
    command
        .arg("hardware")
        .arg(plan)
        .arg("-c")
        .arg(&tmp.path)
        .arg("--json")
        .stdin(Stdio::null())
        .kill_on_drop(true);
    // Retried while the binary is Text file busy (a test writes its fake plan2cell just before; Linux CI).
    let output = tokio::time::timeout(CLI_TIMEOUT, super::codegen::output_when_not_busy(&mut command))
        .await
        .map_err(|_| unavailable(format!("{} hardware timed out", binary.display())))?
        .map_err(|err| unavailable(format!("running {}: {err}", binary.display())))?;
    let report: cli::Report = serde_json::from_slice(&output.stdout).map_err(|err| {
        unavailable(format!(
            "{} hardware --json ({}): no report ({err}; an older plan2cell without `hardware`?): {}",
            binary.display(),
            output.status,
            String::from_utf8_lossy(&output.stderr).trim()
        ))
    })?;
    if let Some(error) = &report.error {
        return Err((StatusCode::UNPROCESSABLE_ENTITY, format!("plan2cell hardware: {}", error.message)));
    }
    Ok(report)
}

/// The cell's own plan2cell.toml, if it has one.
fn read_base(root: &Path) -> Result<Option<String>, HttpError> {
    let file = root.join(codegen::BASE_CONFIG);
    match std::fs::read_to_string(&file) {
        Ok(text) => Ok(Some(text)),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(err) => Err((StatusCode::INTERNAL_SERVER_ERROR, format!("reading {}: {err}", file.display()))),
    }
}

fn parse_base(base: Option<&str>) -> Result<toml::Table, HttpError> {
    base.map_or(Ok(toml::Table::new()), |text| {
        text.parse().map_err(|err| (StatusCode::UNPROCESSABLE_ENTITY, format!("the cell's {}: {err}", codegen::BASE_CONFIG)))
    })
}

/// `values` over `defaults`: the network, and per robot / controller (by ns) the editable fields
/// (a robot's station, device name, IP, external, safety bypass and safety module);
/// robots and controllers of `defaults` only (the plan's), the rest (model, tag, ...) from it.
pub fn overlay(defaults: &HardwareConfig, values: &HardwareConfig) -> HardwareConfig {
    let mut out = defaults.clone();
    out.network = values.network.clone();
    for robot in &mut out.robots {
        if let Some(v) = values.robot(&robot.ns) {
            robot.station = v.station.clone();
            robot.device_name = v.device_name.clone();
            robot.ip = v.ip.clone();
            robot.external = v.external;
            robot.safety_bypass = v.safety_bypass;
            robot.safety_module = v.safety_module;
        }
    }
    for controller in &mut out.kuka_vc {
        if let Some(v) = values.controller(&controller.robot_ns) {
            controller.kli_ip = v.kli_ip.clone();
            controller.station = v.station.clone();
        }
    }
    out
}

/// Every editable value without surrounding whitespace.
fn trimmed(config: &HardwareConfig) -> HardwareConfig {
    let t = |s: &str| s.trim().to_owned();
    HardwareConfig {
        network: NetworkConfig {
            head_ip: t(&config.network.head_ip),
            netmask: t(&config.network.netmask),
            gateway: t(&config.network.gateway),
            controller_station: t(&config.network.controller_station),
        },
        robots: config
            .robots
            .iter()
            .map(|r| RobotHardware { station: t(&r.station), device_name: t(&r.device_name), ip: t(&r.ip), ..r.clone() })
            .collect(),
        kuka_vc: config
            .kuka_vc
            .iter()
            .map(|c| KukaVcController { robot_ns: c.robot_ns.clone(), kli_ip: t(&c.kli_ip), station: t(&c.station) })
            .collect(),
    }
}

/// `hardware` into a plan2cell.toml: `[plc.network]` head_ip / netmask / gateway /
/// controller_station (and without an `ip_pool_first` / `ip_pool_last` outside the new head
/// subnet, so plan2cell derives the pool from it), and each robot's `[[plc.robot]]` (by ns; a new entry if it has none)
/// station / device_name / ip / external / safety_bypass / safety_module. Everything else in `doc` stays.
pub fn apply_to_plan2cell_toml(doc: &mut toml::Table, hardware: &HardwareConfig) -> anyhow::Result<()> {
    use toml::{Table, Value};

    let plc = doc
        .entry("plc")
        .or_insert_with(|| Value::Table(Table::new()))
        .as_table_mut()
        .context("[plc] is not a table")?;
    let network = plc
        .entry("network")
        .or_insert_with(|| Value::Table(Table::new()))
        .as_table_mut()
        .context("[plc.network] is not a table")?;
    let n = &hardware.network;
    network.insert("head_ip".into(), Value::String(n.head_ip.clone()));
    network.insert("netmask".into(), Value::String(n.netmask.clone()));
    network.insert("gateway".into(), Value::String(hardware.gateway()));
    network.insert("controller_station".into(), Value::String(n.controller_station.clone()));
    // The PN controller's DCP pool: plan2cell derives it from the head subnet unless it is set.
    // A pool set for another subnet would not fit the new one: drop it (derived again).
    if let (Ok(head), Ok(mask)) = (n.head_ip.parse::<std::net::Ipv4Addr>(), n.netmask.parse::<std::net::Ipv4Addr>()) {
        let mask = u32::from(mask);
        let fits = |key: &str| {
            network
                .get(key)
                .and_then(Value::as_str)
                .and_then(|v| v.parse::<std::net::Ipv4Addr>().ok())
                .is_some_and(|ip| u32::from(ip) & mask == u32::from(head) & mask)
        };
        if !(fits("ip_pool_first") && fits("ip_pool_last")) {
            network.remove("ip_pool_first");
            network.remove("ip_pool_last");
        }
    }
    let robots = plc
        .entry("robot")
        .or_insert_with(|| Value::Array(Vec::new()))
        .as_array_mut()
        .context("[[plc.robot]] is not an array of tables")?;
    for r in &hardware.robots {
        let index = match robots.iter().position(|t| t.get("ns").and_then(Value::as_str) == Some(r.ns.as_str())) {
            Some(index) => index,
            None => {
                let mut table = Table::new();
                table.insert("ns".into(), Value::String(r.ns.clone()));
                robots.push(Value::Table(table));
                robots.len() - 1
            }
        };
        let table = robots[index].as_table_mut().context("[[plc.robot]] is not an array of tables")?;
        table.insert("station".into(), Value::String(r.station.clone()));
        table.insert("device_name".into(), Value::String(r.device_name.clone()));
        table.insert("ip".into(), Value::String(r.ip.clone()));
        table.insert("external".into(), Value::Boolean(r.external));
        table.insert("safety_bypass".into(), Value::Boolean(r.safety_bypass));
        table.insert("safety_module".into(), Value::Boolean(r.safety_module));
    }
    Ok(())
}

/// The server's cell.toml (`server`, read from `source`) with each `[[kuka_vc.controller]]`'s
/// `kli_ip` and `station` from `hardware` (by robot_ns); the rest as it is.
pub fn render_cell_config(server: &str, source: &Path, hardware: &HardwareConfig) -> anyhow::Result<String> {
    let mut doc: toml::Table = server.parse().with_context(|| format!("{} is not valid TOML", source.display()))?;
    if let Some(controllers) = doc.get_mut("kuka_vc").and_then(|k| k.get_mut("controller")).and_then(|c| c.as_array_mut()) {
        for c in controllers.iter_mut().filter_map(|c| c.as_table_mut()) {
            let Some(ns) = c.get("robot_ns").and_then(|v| v.as_str()).map(str::to_owned) else { continue };
            if let Some(h) = hardware.controller(&ns) {
                c.insert("kli_ip".into(), toml::Value::String(h.kli_ip.clone()));
                c.insert("station".into(), toml::Value::String(h.station.clone()));
            }
        }
    }
    let header = format!(
        "# Written by Physical AI Production Designer (Hardware Configuration): {} with the KUKA VC controllers' kli_ip / station\n\
         # from {HARDWARE_FILE}. Rewritten on every save and before every Start production; edit those instead.\n\n",
        source.display()
    );
    Ok(header + &toml::to_string_pretty(&doc)?)
}

pub(crate) fn write_atomic(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, bytes).with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

fn unix_s(time: SystemTime) -> f64 {
    time.duration_since(SystemTime::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// A file in the temp folder, removed when dropped.
pub(crate) struct TempFile {
    pub path: PathBuf,
}

impl TempFile {
    pub fn new(what: &str, extension: &str, bytes: &[u8]) -> anyhow::Result<Self> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("factoryos-{what}-{}-{n}.{extension}", std::process::id()));
        std::fs::write(&path, bytes).with_context(|| format!("writing {}", path.display()))?;
        Ok(Self { path })
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        std::fs::remove_file(&self.path).ok();
    }
}

/// `plan2cell hardware --json` (schema `plan2cell/hardware@1`, camelCase).
mod cli {
    use super::*;

    #[derive(serde::Deserialize, Default)]
    #[serde(default, rename_all = "camelCase")]
    pub struct Network {
        pub head_ip: String,
        pub netmask: String,
        pub gateway: String,
        pub controller_station: String,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default, rename_all = "camelCase")]
    pub struct Robot {
        ns: String,
        tag: String,
        station: String,
        device_name: String,
        ip: String,
        model: String,
        external: bool,
        safety_bypass: bool,
        /// bt2plc's default (an older plan2cell without it in the report).
        #[serde(default = "yes")]
        safety_module: bool,
    }

    fn yes() -> bool {
        true
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default, rename_all = "camelCase")]
    pub struct Controller {
        robot_ns: String,
        kli_ip: String,
        station: String,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default, rename_all = "camelCase")]
    pub struct KukaVc {
        controllers: Vec<Controller>,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default)]
    pub struct Error {
        pub message: String,
    }

    #[derive(serde::Deserialize, Default)]
    #[serde(default, rename_all = "camelCase")]
    pub struct Report {
        pub network: Network,
        pub network_defaults: Network,
        robots: Vec<Robot>,
        kuka_vc: KukaVc,
        pub issues: Vec<FieldIssue>,
        pub error: Option<Error>,
    }

    impl Report {
        /// The plan's robots.
        pub fn robot_ns(&self) -> Vec<String> {
            self.robots.iter().map(|r| r.ns.clone()).collect()
        }

        /// The values as the form shows them.
        pub fn config(&self) -> HardwareConfig {
            let n = &self.network;
            HardwareConfig {
                network: NetworkConfig {
                    head_ip: n.head_ip.clone(),
                    netmask: n.netmask.clone(),
                    gateway: n.gateway.clone(),
                    controller_station: n.controller_station.clone(),
                },
                robots: self
                    .robots
                    .iter()
                    .map(|r| RobotHardware {
                        ns: r.ns.clone(),
                        tag: r.tag.clone(),
                        station: r.station.clone(),
                        device_name: r.device_name.clone(),
                        ip: r.ip.clone(),
                        model: r.model.clone(),
                        external: r.external,
                        safety_bypass: r.safety_bypass,
                        safety_module: r.safety_module,
                    })
                    .collect(),
                kuka_vc: self
                    .kuka_vc
                    .controllers
                    .iter()
                    .map(|c| KukaVcController { robot_ns: c.robot_ns.clone(), kli_ip: c.kli_ip.clone(), station: c.station.clone() })
                    .collect(),
            }
        }
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new().route(api::CELL_HARDWARE, get(get_hardware).put(put_hardware))
}

#[derive(serde::Deserialize, Default)]
#[serde(default)]
struct HardwareQuery {
    plan: Option<String>,
}

async fn get_hardware(State(state): State<AppState>, Query(query): Query<HardwareQuery>) -> Result<Json<HardwareView>, HttpError> {
    state.hardware.view(query.plan.as_deref()).await.map(Json)
}

async fn put_hardware(State(state): State<AppState>, Json(request): Json<HardwareSave>) -> Result<Json<HardwareView>, Response> {
    state.hardware.save(request).await.map(Json)
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    /// A stand-in for `plan2cell hardware --json`: two external robots at their defaults, with the
    /// config's `[plc.network] head_ip` and `gateway` (grep'd) as the network, and both robots
    /// without the safety module (and with the bypass) when the config has a `safety_module = false`;
    /// `issues` when the config contains `BAD`; an `error` for a plan containing `BROKEN`.
    const FAKE: &str = r#"#!/bin/sh
[ "$1" = "hardware" ] || exit 2
plan="$2"; cfg="$4"
if grep -q BROKEN "$plan"; then echo '{"schema":"plan2cell/hardware@1","ok":false,"error":{"code":3,"message":"plan unreadable"},"issues":[]}'; exit 3; fi
head=$(sed -n 's/^head_ip = "\(.*\)"/\1/p' "$cfg"); [ -n "$head" ] || head=192.168.100.10
gw=$(sed -n 's/^gateway = "\(.*\)"/\1/p' "$cfg"); [ -n "$gw" ] || gw=0.0.0.0
flags='"external":true'
if grep -q '^safety_module = false' "$cfg"; then flags='"external":true,"safetyBypass":true,"safetyModule":false'; fi
issues='[]'; code=0
if grep -q BAD "$cfg"; then issues='[{"field":"robots.robot1.deviceName","message":"bt2plc says no"}]'; code=3; fi
cat <<JSON
{"schema":"plan2cell/hardware@1","ok":true,
 "network":{"headIp":"$head","netmask":"255.255.255.0","prefix":24,"gateway":"$gw","controllerStation":"controller"},
 "networkDefaults":{"headIp":"192.168.100.10","netmask":"255.255.255.0","gateway":"0.0.0.0","controllerStation":"controller"},
 "robots":[{"ns":"robot1","tag":"R1","station":"kuka-robi1","deviceName":"kuka_robi1","ip":"192.168.100.11","model":"KR240",$flags},
           {"ns":"robot2","tag":"R2","station":"kuka-robi2","deviceName":"kuka_robi2","ip":"192.168.100.12","model":"KR240",$flags}],
 "kukaVc":{"kliPrefix":24,"kliGw":null,"controllers":[{"robotNs":"robot1","kliIp":"192.168.100.11","station":"kuka-robi1"},{"robotNs":"robot2","kliIp":"192.168.100.12","station":"kuka-robi2"}]},
 "issues":$issues}
JSON
exit $code
"#;

    const SERVER_CELL: &str = "[codesys_mcp]\nurl = \"http://x/mcp\"\n\n[kuka_vc]\nregion = \"eu-central-1\"\n\n\
        [[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"r1\"\nnetns = \"krc1\"\nkli_ip = \"192.168.100.11\"\nstation = \"kuka-robi1\"\n\n\
        [[kuka_vc.controller]]\nrobot_ns = \"robot2\"\ninstance = \"r2\"\nnetns = \"krc2\"\nkli_ip = \"192.168.100.12\"\nstation = \"kuka-robi2\"\n";

    struct Fixture {
        root: PathBuf,
        cell: PathBuf,
        server_cell: PathBuf,
        service: Arc<HardwareService>,
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.root).ok();
        }
    }

    fn fixture(name: &str, with_server_cell: bool) -> Fixture {
        use std::os::unix::fs::PermissionsExt as _;

        let root = std::env::temp_dir().join(format!("factoryos-hardware-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let cell = root.join("cells/weld");
        let plans = cell.join(cells::PLANS_DIR);
        std::fs::create_dir_all(&plans).unwrap();
        std::fs::write(plans.join("bt_a.json"), r#"{"behaviour_tree":{},"tree_id":"bt_a","created_s":1}"#).unwrap();
        std::fs::write(plans.join("bt_broken.json"), r#"{"behaviour_tree":{},"tree_id":"BROKEN","created_s":0}"#).unwrap();
        let fake = root.join("plan2cell");
        std::fs::write(&fake, FAKE).unwrap();
        std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o755)).unwrap();
        let server_cell = root.join("etc-cell.toml");
        if with_server_cell {
            std::fs::write(&server_cell, SERVER_CELL).unwrap();
        }
        let store = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let mut config = store.get();
        config.cell_dir = cell.display().to_string();
        store.set(config);
        let service = HardwareService::with(store, Some(fake), server_cell.clone());
        Fixture { root, cell, server_cell, service }
    }

    async fn rejected(response: Response) -> (StatusCode, Vec<FieldIssue>) {
        let status = response.status();
        let body = axum::body::to_bytes(response.into_body(), 1 << 20).await.unwrap();
        (status, serde_json::from_slice::<HardwareRejected>(&body).map(|r| r.errors).unwrap_or_default())
    }

    #[tokio::test]
    async fn defaults_come_from_plan2cell_and_saves_are_checked() {
        let f = fixture("save", true);
        let view = f.service.view(None).await.unwrap();
        assert_eq!(view.cell, "weld");
        assert_eq!(view.plan, "agentic_planner/plans/bt_a.json", "the newest plan");
        assert_eq!(view.source, HardwareSource::Defaults);
        assert_eq!(view.config, view.defaults);
        assert_eq!(view.config.robots.len(), 2);
        assert_eq!(view.config.kuka_vc[1].kli_ip, "192.168.100.12");
        assert_eq!((view.errors.len(), view.warnings.len()), (0, 0), "{view:#?}");
        assert_eq!(view.wiring.as_ref().unwrap().controllers.len(), 2);
        assert!(view.cell_config.is_none());
        // A virtual cell: its KUKA VCs' values are the defaults.
        assert_eq!(view.virtual_cell_robots, ["robot1", "robot2"]);
        assert_eq!(view.defaults.network.gateway, VIRTUAL_CELL_GATEWAY);
        assert!(view.defaults.robots.iter().all(|r| r.external && r.safety_bypass && !r.safety_module), "{view:#?}");

        // Invalid: every problem, nothing saved.
        let mut config = view.config.clone();
        config.robots[0].station = "Kuka R1".into();
        config.robots[1].ip = "192.168.100.11".into();
        let (status, errors) = rejected(f.service.save(HardwareSave { plan: String::new(), config }).await.unwrap_err()).await;
        assert_eq!(status, StatusCode::UNPROCESSABLE_ENTITY);
        let fields: Vec<&str> = errors.iter().map(|e| e.field.as_str()).collect();
        assert_eq!(fields, ["robots.robot1.station", "robots.robot2.ip"]);
        assert!(!f.cell.join(HARDWARE_FILE).exists());

        // A robot the plan does not have.
        let mut config = view.config.clone();
        config.robots.push(RobotHardware { ns: "robot9".into(), ..Default::default() });
        let (status, errors) = rejected(f.service.save(HardwareSave { plan: "bt_a".into(), config }).await.unwrap_err()).await;
        assert_eq!((status, errors[0].field.as_str()), (StatusCode::UNPROCESSABLE_ENTITY, "robots.robot9"));

        // plan2cell's own check refuses it.
        let mut config = view.config.clone();
        config.robots[0].device_name = "BAD".into();
        let (status, errors) = rejected(f.service.save(HardwareSave { plan: "bt_a".into(), config }).await.unwrap_err()).await;
        assert_eq!(status, StatusCode::UNPROCESSABLE_ENTITY);
        assert_eq!(errors[0].message, "bt2plc says no");

        // Valid: saved, the cell.toml rendered, the view says what differs from the cell.
        let mut config = view.config.clone();
        config.network.head_ip = " 192.168.100.20 ".into();
        config.robots[0].station = "weld-r1".into();
        config.robots[0].model = "client values for read-only fields are ignored".into();
        config.kuka_vc[0].station = "weld-r1".into();
        let saved = f.service.save(HardwareSave { plan: "bt_a".into(), config }).await.unwrap();
        assert_eq!(saved.source, HardwareSource::Saved);
        assert!(saved.saved_s.is_some());
        assert!(saved.etag.is_some() && saved.etag == saved_etag(&f.cell), "the ETag a build records");
        assert_eq!(saved.config.network.head_ip, "192.168.100.20", "trimmed");
        assert_eq!(saved.config.robots[0].model, "KR240");
        assert_eq!(saved.defaults.network.head_ip, "192.168.100.10");
        let warned: Vec<&str> = saved.warnings.iter().map(|w| w.field.as_str()).collect();
        assert_eq!(warned, ["network.headIp", "robots.robot1.station", "kukaVc.robot1.station"], "{:#?}", saved.warnings);
        let rendered = std::fs::read_to_string(f.cell.join(CELL_CONFIG)).unwrap();
        let doc: toml::Table = rendered.parse().unwrap();
        let controllers = doc["kuka_vc"]["controller"].as_array().unwrap();
        assert_eq!(controllers[0]["station"].as_str(), Some("weld-r1"));
        assert_eq!(controllers[0]["netns"].as_str(), Some("krc1"), "the rest stays");
        assert_eq!(controllers[1]["station"].as_str(), Some("kuka-robi2"));
        assert_eq!(doc["codesys_mcp"]["url"].as_str(), Some("http://x/mcp"));
        assert_eq!(saved.cell_config.as_deref(), Some(f.cell.join(CELL_CONFIG).display().to_string().as_str()));
        assert_eq!(f.service.cell_config_for(&f.cell).unwrap(), Some(f.cell.join(CELL_CONFIG)));

        // Read back from hardware.json.
        let again = f.service.view(Some("agentic_planner/plans/bt_a.json")).await.unwrap();
        assert_eq!(again.config, saved.config);
        let (file, _) = read_saved(&f.cell).unwrap();
        assert_eq!(file.schema, SCHEMA);
        assert_eq!(file.plan, "agentic_planner/plans/bt_a.json");

        // Unreadable plans and other paths.
        assert_eq!(f.service.view(Some("bt_broken")).await.unwrap_err().0, StatusCode::UNPROCESSABLE_ENTITY);
        assert_eq!(f.service.view(Some("../../etc/passwd")).await.unwrap_err().0, StatusCode::NOT_FOUND);
    }

    #[tokio::test]
    async fn a_save_and_its_cell_toml_are_written_through() {
        use crate::storage::sync::tests_support::{eventually, storage_at};
        use factoryos_protocol::storage::Scope;
        let f = fixture("through", true);
        let (s3, storage) = storage_at(&f.root);
        let _registered = crate::storage::register(&storage);
        let view = f.service.view(None).await.unwrap();
        f.service.save(HardwareSave { plan: "bt_a".into(), config: view.config }).await.unwrap();
        let (hardware, cell_toml) = ("cells/weld/plan2cell/hardware.json", "cells/weld/plan2cell/cell.toml");
        eventually("hardware.json and cell.toml", || s3.bytes(Scope::User, hardware).is_some() && s3.bytes(Scope::User, cell_toml).is_some()).await;
        assert_eq!(s3.bytes(Scope::User, hardware).unwrap(), std::fs::read(f.cell.join(HARDWARE_FILE)).unwrap());
        assert_eq!(s3.bytes(Scope::User, cell_toml).unwrap(), std::fs::read(f.cell.join(CELL_CONFIG)).unwrap());
    }

    #[tokio::test]
    async fn without_a_virtual_cell_nothing_is_rendered() {
        let f = fixture("nocell", false);
        assert!(!f.server_cell.exists());
        let view = f.service.view(None).await.unwrap();
        assert!(view.wiring.is_none());
        // Real hardware: plan2cell's own defaults (the safety module, no gateway), nothing to warn.
        assert!(view.virtual_cell_robots.is_empty());
        assert_eq!(view.defaults.network.gateway, "0.0.0.0");
        assert!(view.defaults.robots.iter().all(|r| !r.safety_bypass && r.safety_module), "{view:#?}");
        assert_eq!(view.warnings, vec![]);
        let saved = f.service.save(HardwareSave { plan: String::new(), config: view.config }).await.unwrap();
        assert_eq!(saved.source, HardwareSource::Saved);
        assert!(!f.cell.join(CELL_CONFIG).exists());
        assert_eq!(f.service.cell_config_for(&f.cell).unwrap(), None);
    }

    /// The test server's files the real-CLI test below reads, as this crate's fixtures
    /// (`tests/fixtures/test-server/`): plan2cell/infra/test-server's with placeholder ids, address and
    /// account, so the test needs no infra files (the public export carries none). Where the infra files
    /// are (this repo), each copy must be its file but for those values: a change there fails here until
    /// the copy has it too.
    #[test]
    fn the_test_server_fixtures_are_the_test_servers_files() {
        let infra = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../plan2cell/infra/test-server");
        if !infra.is_dir() {
            eprintln!("skipped: no {} (the fixtures stand in for it)", infra.display());
            return;
        }
        let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/test-server");
        let read = |p: PathBuf| -> toml::Table { std::fs::read_to_string(&p).unwrap().parse().unwrap() };
        assert_eq!(read(fixtures.join("plan2cell.toml")), read(infra.join("plan2cell.toml")));
        let (mut server, fixture) = (read(infra.join("cell.toml")), read(fixtures.join("cell.toml")));
        // The placeholders: the CODESYS MCP's address, the PLC host's instance and bucket, each KUKA VC's instance.
        for (table, key) in [("codesys_mcp", "url"), ("plc_host", "ssm_instance"), ("plc_host", "bucket")] {
            server[table].as_table_mut().unwrap()[key] = fixture[table][key].clone();
        }
        let controllers = fixture["kuka_vc"]["controller"].as_array().unwrap();
        for (i, c) in server["kuka_vc"]["controller"].as_array_mut().unwrap().iter_mut().enumerate() {
            c.as_table_mut().unwrap()["ssm_instance"] = controllers[i]["ssm_instance"].clone();
        }
        assert_eq!(server, fixture, "the fixture is the test server's cell.toml but for its placeholders");
    }

    /// End to end through the real CLI (`cd plan2cell && cargo build -p plan2cell-cli`, then
    /// `cargo test -p factoryos-backend -- --ignored real_plan2cell`; or `FACTORY_OS_PLAN2CELL`):
    /// the defaults of the sample plan with the test server's plan2cell.toml, a save that plan2cell
    /// accepts (its `deny_unknown_fields` and its own rules), a refused one, and a build that gets
    /// the saved addresses. The test server's files are the fixtures' copies (above).
    #[tokio::test]
    #[ignore = "needs the plan2cell binary (cd plan2cell && cargo build -p plan2cell-cli)"]
    async fn real_plan2cell_hardware_defaults_and_save() {
        use factoryos_protocol::codegen::{BuildRequest, CodegenState};

        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let test_server = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/test-server");
        let binary = std::env::var_os("FACTORY_OS_PLAN2CELL")
            .map(PathBuf::from)
            .unwrap_or_else(|| repo.join("plan2cell/target/debug/plan2cell"));
        assert!(binary.is_file(), "build it first: {}", binary.display());
        let f = fixture("real", true);
        let plan = "bt_1790934549_welding_cell_2_small";
        let plans = f.cell.join(cells::PLANS_DIR);
        std::fs::copy(repo.join("plan2cell/test_data").join(format!("{plan}.json")), plans.join(format!("{plan}.json"))).unwrap();
        std::fs::remove_file(plans.join("bt_a.json")).unwrap();
        std::fs::remove_file(plans.join("bt_broken.json")).unwrap();
        std::fs::copy(test_server.join("cell.toml"), &f.server_cell).unwrap();
        let service = HardwareService::with(f.service.config.clone(), Some(binary.clone()), f.server_cell.clone());

        // A cell without a plan2cell.toml (scene_fit) on the virtual cell: its KUKA VCs' values,
        // which plan2cell accepts.
        let view = service.view(None).await.unwrap();
        assert_eq!(view.virtual_cell_robots, ["robot1", "robot2"]);
        assert_eq!(view.defaults.network.gateway, VIRTUAL_CELL_GATEWAY);
        assert!(view.defaults.robots.iter().all(|r| r.external && r.safety_bypass && !r.safety_module), "{:#?}", view.defaults);
        assert_eq!((view.errors.len(), view.warnings.len()), (0, 0), "{:#?} {:#?}", view.errors, view.warnings);
        // Without the virtual cell: plan2cell's own.
        let real = HardwareService::with(f.service.config.clone(), Some(binary.clone()), f.root.join("none.toml"));
        let view = real.view(None).await.unwrap();
        assert!(view.virtual_cell_robots.is_empty());
        assert!(view.defaults.robots.iter().all(|r| !r.external && !r.safety_bypass && r.safety_module), "{:#?}", view.defaults);

        std::fs::create_dir_all(f.cell.join("plan2cell")).unwrap();
        std::fs::copy(test_server.join("plan2cell.toml"), f.cell.join(codegen::BASE_CONFIG)).unwrap();

        let view = service.view(None).await.unwrap();
        assert_eq!(view.plan, format!("{}/{plan}.json", cells::PLANS_DIR));
        let n = &view.defaults.network;
        assert_eq!((n.head_ip.as_str(), n.netmask.as_str(), n.controller_station.as_str()), ("192.168.100.10", "255.255.255.0", "controller"));
        let robots: Vec<(&str, &str, &str, &str)> =
            view.defaults.robots.iter().map(|r| (r.ns.as_str(), r.station.as_str(), r.device_name.as_str(), r.ip.as_str())).collect();
        assert_eq!(
            robots,
            [("robot1", "kuka-robi1", "kuka_robi1", "192.168.100.11"), ("robot2", "kuka-robi2", "kuka_robi2", "192.168.100.12")]
        );
        assert!(view.defaults.robots.iter().all(|r| r.external && r.model == "KR240R2900_ULTRA"));
        assert_eq!(view.defaults.kuka_vc.len(), 2);
        assert_eq!((view.errors.len(), view.warnings.len()), (0, 0), "{:#?} {:#?}", view.errors, view.warnings);
        assert_eq!(view.wiring.as_ref().unwrap().controllers[1].kli_ip, "192.168.100.12", "the test server's cell.toml");

        // Saved with another address: plan2cell accepts the config, the cell's wiring differs.
        let mut config = view.config.clone();
        config.robots[0].ip = "192.168.100.21".into();
        config.robots[0].station = "weld-r1".into();
        config.network.gateway = "192.168.100.1".into();
        let saved = service.save(HardwareSave { plan: String::new(), config }).await.unwrap();
        assert_eq!(saved.source, HardwareSource::Saved);
        assert!(saved.errors.is_empty(), "{:#?}", saved.errors);
        assert!(saved.warnings.iter().any(|w| w.field == "robots.robot1.ip"), "{:#?}", saved.warnings);
        assert!(saved.warnings.iter().any(|w| w.field == "kukaVc.robot1.kliIp"), "the VC still follows .11: {:#?}", saved.warnings);
        let rendered: toml::Table = std::fs::read_to_string(f.cell.join(CELL_CONFIG)).unwrap().parse().unwrap();
        let server: toml::Table = std::fs::read_to_string(&f.server_cell).unwrap().parse().unwrap();
        let plc_host = server["plc_host"]["ssm_instance"].as_str().expect("the test server's PLC host");
        assert_eq!(rendered["plc_host"]["ssm_instance"].as_str(), Some(plc_host));

        // A device name only plan2cell's build checks catch would be refused too; the form's rules first.
        let mut config = saved.config.clone();
        config.robots[1].ip = "192.168.100.21".into();
        let response = service.save(HardwareSave { plan: String::new(), config }).await.unwrap_err();
        assert_eq!(response.status(), StatusCode::UNPROCESSABLE_ENTITY);

        // A build gets the saved addresses. The form's default on a virtual cell is sim (its KUKA VCs run
        // bt2krl's sim programs); vw-ks24 asked for, with robot sims, still builds the VW port's KRL.
        let codegen = codegen::CodegenService::with_sink(
            Arc::new(NoSink),
            f.service.config.clone(),
            Some(binary),
        )
        .on_virtual_cell(VirtualCell::at(f.server_cell.clone()));
        let options = codegen.options().await.unwrap();
        assert_eq!((options.defaults.profile.as_str(), options.defaults.robot_sim), ("sim", false));
        assert!(options.defaults.opcua_symbols, "the live view reads the head over OPC UA");
        let request = BuildRequest {
            st_check: false,
            robot_sim: true,
            profile: "vw-ks24".into(),
            job_number: "byte".into(),
            ..BuildRequest::new(plan, &options.defaults)
        };
        let accepted = codegen.start(request).await.unwrap();
        let status = loop {
            let status = codegen.status();
            if status.state != CodegenState::Running {
                break status;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        };
        assert_eq!(status.state, CodegenState::Done, "{status:?}");
        let info = codegen.info(&accepted.build_id).unwrap();
        assert_eq!(info.hardware_source, HardwareSource::Saved);
        let program = info.files.iter().find(|file| file.path.ends_with("program.json")).expect("a program.json");
        let (bytes, _) = codegen.file(&accepted.build_id, &program.path).unwrap();
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.contains("192.168.100.21") && text.contains("weld-r1"), "{}: {}", program.path, &text[..text.len().min(2000)]);
        let (bytes, _) = codegen.file(&accepted.build_id, "plan2cell.toml").unwrap();
        let config: toml::Table = String::from_utf8(bytes).unwrap().parse().unwrap();
        assert_eq!(config["build"]["profile"].as_str(), Some("vw-ks24"));
        assert!(config["plc"]["robot"].as_array().unwrap().iter().all(|r| r["safety_module"].as_bool() == Some(false)));
        // The KRL of the VW port (a2717d1): the op index is the global BT_PKTNR.
        let krl: Vec<_> = info.files.iter().filter(|file| file.path.ends_with(".src") || file.path.ends_with(".dat")).collect();
        assert!(!krl.is_empty(), "{:#?}", info.files);
        let all: String = krl.iter().map(|file| String::from_utf8(codegen.file(&accepted.build_id, &file.path).unwrap().0).unwrap()).collect();
        let code = |l: &&str| l.split(';').next().unwrap_or_default().to_owned();
        assert!(all.lines().map(|l| code(&l)).any(|c| c.contains("BT_PKTNR=1")), "the vw-ks24 KRL sets BT_PKTNR");
        assert!(!all.lines().map(|l| code(&l)).any(|c| c.contains("P_PktNr_SPS")), "nor the vw_usr_r local P_PktNr_SPS (KSS 2137)");
        assert!(!info.files.iter().any(|file| file.path.contains("RobotSim")), "vw-ks24: no robot simulator projects");
    }

    /// scene_fit's case: a configuration saved with plan2cell's defaults (the safety module, no
    /// gateway) on a virtual cell stays as saved, with warnings; the virtual cell's values fix it.
    #[tokio::test]
    async fn a_saved_safety_module_on_a_virtual_cell_is_kept_and_warned_about() {
        let f = fixture("vc-saved", true);
        let view = f.service.view(None).await.unwrap();
        let mut config = view.config.clone();
        config.network.gateway = "0.0.0.0".into();
        for r in &mut config.robots {
            (r.external, r.safety_bypass, r.safety_module) = (false, false, true);
        }
        let saved = f.service.save(HardwareSave { plan: String::new(), config: config.clone() }).await.unwrap();
        assert_eq!(saved.config, config, "saved as given");
        let again = f.service.view(None).await.unwrap();
        assert_eq!(again.config, config, "not changed when read back");
        let warned: Vec<&str> = again.warnings.iter().map(|w| w.field.as_str()).collect();
        assert!(warned.contains(&"robots.robot1.safetyModule") && warned.contains(&"network.gateway"), "{warned:?}");
        assert!(again.warnings.iter().any(|w| w.message.contains("ERR_SAFETY_TIMEOUT")));
        // The export gets what is saved.
        let (export, _) = for_export(&f.root.join("plan2cell"), &f.service.virtual_cell, &f.cell, &again.plan, &f.cell.join(&again.plan))
            .await
            .unwrap()
            .unwrap();
        assert!(export.robots.iter().all(|r| r.safety_module));

        // "Use the virtual cell's values".
        let fixed = again.config.with_virtual_cell_values(&again.virtual_cell_robots);
        let saved = f.service.save(HardwareSave { plan: String::new(), config: fixed }).await.unwrap();
        assert_eq!(saved.warnings, vec![]);
        assert!(saved.config.robots.iter().all(|r| !r.safety_module));
        assert_eq!(saved.config.network.gateway, VIRTUAL_CELL_GATEWAY);

        // Without the bypass, no safety module is refused (bt2plc's rule).
        let mut bad = saved.config.clone();
        bad.robots[0].safety_bypass = false;
        let (status, errors) = rejected(f.service.save(HardwareSave { plan: String::new(), config: bad }).await.unwrap_err()).await;
        assert_eq!((status, errors[0].field.as_str()), (StatusCode::UNPROCESSABLE_ENTITY, "robots.robot1.safetyModule"));
    }

    #[test]
    fn the_virtual_cells_defaults_fill_only_what_the_cell_leaves_open() {
        let robots = vec!["robot1".to_owned(), "robot2".to_owned()];
        let doc: toml::Table = with_virtual_cell_defaults(None, &robots, &Addressing::default()).unwrap().parse().unwrap();
        assert_eq!(doc["plc"]["network"]["gateway"].as_str(), Some(VIRTUAL_CELL_GATEWAY));
        let list = doc["plc"]["robot"].as_array().unwrap();
        assert_eq!(list.len(), 2);
        for r in list {
            assert_eq!((r["external"].as_bool(), r["safety_bypass"].as_bool(), r["safety_module"].as_bool()), (Some(true), Some(true), Some(false)));
        }
        // The cell's own choices stay.
        let base = "[plc.network]\ngateway = \"0.0.0.0\"\n\n[[plc.robot]]\nns = \"robot1\"\nsafety_module = true\nip = \"192.168.100.21\"\n";
        let doc: toml::Table = with_virtual_cell_defaults(Some(base), &robots[..1], &Addressing::default()).unwrap().parse().unwrap();
        assert_eq!(doc["plc"]["network"]["gateway"].as_str(), Some("0.0.0.0"));
        let list = doc["plc"]["robot"].as_array().unwrap();
        assert_eq!(list.len(), 1, "robot2 is no VC here");
        assert_eq!(list[0]["safety_module"].as_bool(), Some(true));
        assert_eq!(list[0]["external"].as_bool(), Some(true));
        assert_eq!(list[0]["ip"].as_str(), Some("192.168.100.21"));
        assert_eq!(with_virtual_cell_defaults(Some("plc = 1"), &robots, &Addressing::default()).unwrap_err().0, StatusCode::UNPROCESSABLE_ENTITY);
    }

    #[test]
    fn the_virtual_cells_defaults_address_each_robot_as_its_controller() {
        // An EKS network: robotN runs on the controller kuka-robiN at 10.77.1.(10+N), whatever its place in
        // the plan (here the plan lists robot2 first).
        let robots = vec!["robot2".to_owned(), "robot1".to_owned(), "robot3".to_owned(), "robot4".to_owned()];
        let addressing = Addressing {
            head_ip: Some("10.77.1.10".into()),
            gateway: Some("10.77.1.1".into()),
            netmask: Some("255.255.255.0".into()),
            controllers: (1..=4).map(|n| (format!("robot{n}"), format!("kuka-robi{n}"), format!("10.77.1.{}", 10 + n))).collect(),
        };
        let doc: toml::Table = with_virtual_cell_defaults(None, &robots, &addressing).unwrap().parse().unwrap();
        let net = &doc["plc"]["network"];
        assert_eq!((net["head_ip"].as_str(), net["gateway"].as_str(), net["netmask"].as_str()), (Some("10.77.1.10"), Some("10.77.1.1"), Some("255.255.255.0")));
        let list = doc["plc"]["robot"].as_array().unwrap();
        let of = |ns: &str| list.iter().find(|r| r["ns"].as_str() == Some(ns)).unwrap();
        assert_eq!((of("robot2")["station"].as_str(), of("robot2")["ip"].as_str()), (Some("kuka-robi2"), Some("10.77.1.12")));
        assert_eq!((of("robot1")["device_name"].as_str(), of("robot4")["ip"].as_str()), (Some("kuka_robi1"), Some("10.77.1.14")));
        // The cell's own values stay.
        let base = "[[plc.robot]]\nns = \"robot1\"\nip = \"10.77.1.99\"\n";
        let doc: toml::Table = with_virtual_cell_defaults(Some(base), &robots, &addressing).unwrap().parse().unwrap();
        let list = doc["plc"]["robot"].as_array().unwrap();
        assert_eq!(list.iter().find(|r| r["ns"].as_str() == Some("robot1")).unwrap()["ip"].as_str(), Some("10.77.1.99"));
    }

    #[test]
    fn the_virtual_cells_addressing() {
        let dir = std::env::temp_dir().join(format!("factoryos-vca-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("cell.toml");
        std::fs::write(&file, "[profinet]\nhead_ip = \"10.77.1.10\"\ngateway = \"10.77.1.1\"\n\n[kuka_vc]\n[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\nstation = \"kuka-robi1\"\nkli_ip = \"10.77.1.11\"\n").unwrap();
        let a = VirtualCell::at(file).addressing();
        assert_eq!((a.head_ip.as_deref(), a.gateway.as_deref(), a.netmask), (Some("10.77.1.10"), Some("10.77.1.1"), None));
        assert_eq!(a.controllers, vec![("robot1".into(), "kuka-robi1".into(), "10.77.1.11".into())]);
        assert_eq!(VirtualCell::none().addressing(), Addressing::default());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_virtual_cells_robots() {
        let dir = std::env::temp_dir().join(format!("factoryos-vc-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let plan = vec!["robot1".to_owned(), "robot2".to_owned(), "robot3".to_owned()];
        let file = dir.join("cell.toml");
        assert!(VirtualCell::at(file.clone()).robots(&plan).is_empty(), "no cell config: real hardware");
        assert!(VirtualCell::none().robots(&plan).is_empty());
        // A workspace whose cell has not been up yet: every robot.
        let configured = VirtualCell { config: file.clone(), configured: true };
        assert_eq!(configured.robots(&plan), plan);
        std::fs::write(&file, SERVER_CELL).unwrap();
        assert_eq!(VirtualCell::at(file.clone()).robots(&plan), ["robot1", "robot2"], "the controllers it lists");
        std::fs::write(&file, "[kuka_vc]\nregion = \"x\"\n").unwrap();
        assert_eq!(VirtualCell::at(file).robots(&plan), plan, "it lists none");
        std::fs::remove_dir_all(&dir).ok();
    }

    struct NoSink;

    impl codegen::Sink for NoSink {
        fn status(&self, _: &factoryos_protocol::codegen::CodegenStatus) {}
        fn log(&self, _: String) {}
    }

    #[test]
    fn the_build_config_gets_the_hardware() {
        let base = "[plc.cell]\nprod_folge = 12\n\n[[plc.robot]]\nns = \"robot1\"\nexternal = true\nsafety_bypass = true\n";
        let mut doc: toml::Table = base.parse().unwrap();
        let hardware = HardwareConfig {
            network: NetworkConfig {
                head_ip: "10.1.0.10".into(),
                netmask: "255.255.0.0".into(),
                gateway: String::new(),
                controller_station: "head".into(),
            },
            robots: ["robot1", "robot2"]
                .iter()
                .enumerate()
                .map(|(i, ns)| RobotHardware {
                    ns: (*ns).into(),
                    station: format!("r{}", i + 1),
                    device_name: format!("dev{}", i + 1),
                    ip: format!("10.1.0.{}", 11 + i),
                    ..Default::default()
                })
                .collect(),
            kuka_vc: Vec::new(),
        };
        apply_to_plan2cell_toml(&mut doc, &hardware).unwrap();
        assert_eq!(doc["plc"]["network"]["head_ip"].as_str(), Some("10.1.0.10"));
        assert_eq!(doc["plc"]["network"]["gateway"].as_str(), Some("0.0.0.0"));
        assert_eq!(doc["plc"]["cell"]["prod_folge"].as_integer(), Some(12));
        let robots = doc["plc"]["robot"].as_array().unwrap();
        assert_eq!(robots.len(), 2, "robot1's entry is updated, robot2 gets one");
        assert_eq!(robots[0]["safety_bypass"].as_bool(), Some(false), "the configuration's flags");
        assert_eq!(robots[0]["safety_module"].as_bool(), Some(true));
        assert_eq!(robots[0]["station"].as_str(), Some("r1"));
        assert_eq!(robots[1]["ns"].as_str(), Some("robot2"));
        assert_eq!(robots[1]["ip"].as_str(), Some("10.1.0.12"));
        let mut bad: toml::Table = "plc = 3".parse().unwrap();
        assert!(apply_to_plan2cell_toml(&mut bad, &hardware).is_err());
        // A DCP pool of the old subnet is dropped (plan2cell derives it again); one that fits stays.
        let mut doc: toml::Table =
            "[plc.network]\nip_pool_first = \"192.168.100.2\"\nip_pool_last = \"192.168.100.254\"\n".parse().unwrap();
        apply_to_plan2cell_toml(&mut doc, &hardware).unwrap();
        assert!(
            doc["plc"]["network"].get("ip_pool_first").is_none()
                && doc["plc"]["network"].get("ip_pool_last").is_none()
        );
        let mut doc: toml::Table =
            "[plc.network]\nip_pool_first = \"10.1.0.100\"\nip_pool_last = \"10.1.0.200\"\n"
                .parse()
                .unwrap();
        apply_to_plan2cell_toml(&mut doc, &hardware).unwrap();
        assert_eq!(
            doc["plc"]["network"]["ip_pool_first"].as_str(),
            Some("10.1.0.100")
        );
    }
}
