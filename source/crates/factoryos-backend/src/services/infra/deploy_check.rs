//! The deployability check before Deploy / Run Control Code (see
//! [`factoryos_protocol::deploy_check`]): `plan2cell cell check <export> --json` against the
//! server's virtual cell (its wiring cell.toml: the KUKA VCs as they really are, their
//! `$config.dat` read over SSM), plus the backend's rules: the cell's state, the plan's status and
//! whether the export is stale. [`fix`] applies the failing rules' fixes (the Hardware
//! Configuration saved the normal way, after a backup of `hardware.json`; export options),
//! exports the plan again with the same options and checks the new export.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use axum::http::StatusCode;
use factoryos_protocol::codegen::{CodegenState, Severity};
use factoryos_protocol::deploy_check::{
    CheckFix, CheckRule, CheckScope, CheckStatus, DeployCheck, DeployCheckRequest, DeployFixRequest, DeployFixResult, DeployRefused,
};
use factoryos_protocol::hardware::{HardwareConfig, HardwareSave};
use serde_json::Value;

use super::{HttpError, InfraService, run, unix_s};
use crate::AppState;
use crate::services::{cells, codegen, hardware};

/// `plan2cell cell check` reads two `$config.dat` over SSM in parallel: seconds, at most ~2 min.
const CHECK_TIMEOUT: Duration = Duration::from_secs(150);
/// A re-export (the ST type check of a big cell takes a while).
const EXPORT_TIMEOUT: Duration = Duration::from_secs(330);
/// The report next to the export.
pub const CHECK_FILE: &str = "plan2cell-deploy-check.json";

/// An export to check: its cell, folder and build id.
pub(crate) struct Subject {
    pub cell: String,
    pub root: PathBuf,
    pub build: String,
    pub dir: PathBuf,
}

impl InfraService {
    /// The export `request` names (default: the newest successful one of the active cell).
    pub(crate) fn subject(&self, cell: Option<&str>, build: Option<&str>) -> Result<Subject, HttpError> {
        let (cell, root) = match cell.filter(|c| !c.is_empty()) {
            Some(name) => {
                if !name.chars().all(|c| c.is_ascii_alphanumeric() || "_-.".contains(c)) || name.starts_with('.') {
                    return Err((StatusCode::BAD_REQUEST, format!("invalid cell name {name:?}")));
                }
                let root = cells::cell_folder(&self.config, name)
                    .filter(|r| r.is_dir())
                    .ok_or_else(|| (StatusCode::NOT_FOUND, format!("no cell {name}")))?;
                (name.to_owned(), root)
            }
            None => cells::active_cell(&self.config)
                .ok_or_else(|| (StatusCode::BAD_REQUEST, "no active cell (pick the scene in step 1 Behaviour Tree, or Robo Fitter → Open cell)".to_owned()))?,
        };
        let (build, dir) = match build.filter(|b| !b.is_empty()) {
            Some(id) => (
                id.to_owned(),
                codegen::export_dir(&root, id).ok_or_else(|| (StatusCode::NOT_FOUND, format!("no successful Control Code Export {id} in cell {cell}")))?,
            ),
            None => codegen::newest_build(&root).ok_or_else(|| {
                (StatusCode::CONFLICT, format!("no successful Control Code Export of cell {cell} yet: Export PLC + robot code first (step 2)"))
            })?,
        };
        Ok(Subject { cell, root, build, dir })
    }

    /// The deployability check of an export: read-only, seconds.
    pub async fn deploy_check(&self, request: DeployCheckRequest) -> Result<DeployCheck, HttpError> {
        let subject = self.subject(request.cell.as_deref(), request.build.as_deref())?;
        self.check_subject(&subject).await
    }

    pub(crate) async fn check_subject(&self, s: &Subject) -> Result<DeployCheck, HttpError> {
        let binary = self.plan2cell()?;
        let started = std::time::Instant::now();
        // The cell as it is wired (not the cell.toml rendered with the Hardware Configuration's
        // KLI values): what the export has to match.
        let cell_config = self.hardware.server_cell_config_path().map(Path::to_path_buf);
        let mut args: Vec<String> = vec!["cell".into(), "check".into(), s.dir.display().to_string(), "--json".into()];
        match &cell_config {
            Some(c) => args.extend(["--cell-config".into(), c.display().to_string()]),
            None => args.push("--export-only".into()),
        }
        let mut env = Vec::new();
        if let Some(region) = &self.settings.region {
            env.push(("AWS_REGION".to_owned(), region.clone()));
        }
        let out = run(&binary, &args, &env, CHECK_TIMEOUT, None)
            .await
            .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("plan2cell cell check: {err:#}")))?;
        let mut report: DeployCheck = serde_json::from_slice(&out.stdout).map_err(|err| {
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!(
                    "plan2cell cell check (exit {}) gave no report ({err}): {}",
                    out.code.map_or("killed".to_owned(), |c| c.to_string()),
                    out.stderr_tail.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("")
                ),
            )
        })?;
        report.rules.extend(self.backend_rules(s, cell_config.is_some()));
        report.cell = s.cell.clone();
        report.build = s.build.clone();
        report.export = s.dir.display().to_string();
        report.checked_s = unix_s(SystemTime::now());
        report.elapsed_s = started.elapsed().as_secs_f64();
        report.count();
        if let Ok(text) = serde_json::to_vec_pretty(&report) {
            let _ = std::fs::write(s.dir.join(CHECK_FILE), text);
        }
        self.log(format!("Deployability of {} ({}): {}", s.build, s.cell, report.summary()));
        Ok(report)
    }

    /// The rules only the backend can judge: the cell's state, the plan's status, the export's age.
    fn backend_rules(&self, s: &Subject, has_cell: bool) -> Vec<CheckRule> {
        let status = self.status();
        let mut out = Vec::new();
        let mut state = CheckRule {
            id: "cell.state".into(),
            title: "Virtual cell ready".into(),
            scope: CheckScope::Cell,
            ..Default::default()
        };
        if !status.available || !has_cell {
            state.status = CheckStatus::Warn;
            state.message = "this server has no virtual cell (no cell control or no plan2cell cell config): only the export's rules ran".into();
        } else if let Some(why) = status.power_blocks() {
            state.status = CheckStatus::Fail;
            state.message = why;
            state.hint = Some("Start virtual cell (the Virtual cell box below step 4), wait until it is ready, then check again".into());
        } else if status.busy() {
            state.status = CheckStatus::Warn;
            state.message = "a job runs on the cell (deploy, production or bring-up): Deploy waits for it".into();
        } else {
            state.message = "on, no job running".into();
        }
        out.push(state);
        let info = codegen::read_build_info(&s.dir);
        let plan = info.as_ref().map(|i| i.plan.clone()).unwrap_or_default();
        let mut p = CheckRule { id: "plan.status".into(), title: "Plan verified".into(), scope: CheckScope::Export, ..Default::default() };
        let plan_status = (!plan.is_empty())
            .then(|| std::fs::read_to_string(s.root.join(&plan)).ok())
            .flatten()
            .and_then(|t| serde_json::from_str::<Value>(&t).ok())
            .and_then(|v| v["status"].as_str().map(str::to_owned));
        match plan_status.as_deref() {
            None => {
                p.status = CheckStatus::Skip;
                p.message = format!("{plan}: no status");
            }
            Some("ok") => p.message = format!("{plan}: ok (the planner's export checks passed)"),
            Some("failed") => {
                p.status = CheckStatus::Fail;
                p.message = format!("{plan}: the planner marked it failed");
                p.hint = Some("plan the cell again (Agentic Planning: Plan Cell), then Export it (step 2)".into());
            }
            Some(other) => {
                p.status = CheckStatus::Warn;
                p.message = format!(
                    "{plan}: {other} (an export check of the planner failed: collisions or reach are not proven; the cell may stop in a zone)"
                );
                p.hint = Some("plan again with another planner (Agentic Planning), or deploy knowing it is unverified".into());
            }
        }
        out.push(p);
        if let Some(info) = &info {
            let mut st = CheckRule { id: "export.stale".into(), title: "Export up to date".into(), scope: CheckScope::Export, ..Default::default() };
            if codegen::is_stale_at(&s.root, &s.dir, info) {
                st.status = CheckStatus::Warn;
                st.message = "the plan, the cell's plan2cell.toml or its Hardware Configuration changed since this export".into();
                st.hint = Some("Export PLC + robot code again (step 2) so the deploy carries the current settings".into());
            } else {
                st.message = "built from the current plan, plan2cell.toml and Hardware Configuration".into();
            }
            let errors = info.count(Severity::Error);
            if errors > 0 {
                st.status = CheckStatus::Fail;
                st.message = format!("the export has {errors} error diagnostic(s)");
                st.hint = Some("see the export's diagnostics (step 2 results)".into());
            }
            out.push(st);
        }
        out
    }

    /// Deploy's gate: `Err(409 DeployRefused as JSON)` when a rule fails.
    pub(crate) async fn gate(&self, build: &str, root: &Path, dir: &Path) -> Result<(), HttpError> {
        let cell = root.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let s = Subject { cell, root: root.to_path_buf(), build: build.to_owned(), dir: dir.to_path_buf() };
        let report = self.check_subject(&s).await?;
        if report.ok {
            return Ok(());
        }
        let first = report.failing().next().map(|r| format!(": {}", r.line())).unwrap_or_default();
        let refused = DeployRefused {
            error: format!(
                "not deployable: fix {} problem(s) first (or Deploy anyway){first}",
                report.fails
            ),
            report,
        };
        self.log(format!("Deploy of {build} refused: {}", refused.error));
        Err((StatusCode::CONFLICT, serde_json::to_string(&refused).unwrap_or(refused.error)))
    }
}

/// Set `field` (`network.gateway`, `robots.<ns>.safetyModule`, `kukaVc.<ns>.kliIp`) of `config`
/// to `value`: whether it changed.
pub(crate) fn set_field(config: &mut HardwareConfig, field: &str, value: &Value) -> Result<bool, String> {
    let mut v = serde_json::to_value(&*config).map_err(|e| e.to_string())?;
    let parts: Vec<&str> = field.split('.').collect();
    let slot = match parts.as_slice() {
        ["network", key] => v.get_mut("network").and_then(|n| n.get_mut(*key)),
        ["robots", ns, key] => v
            .get_mut("robots")
            .and_then(Value::as_array_mut)
            .and_then(|a| a.iter_mut().find(|r| r["ns"] == *ns))
            .and_then(|r| r.get_mut(*key)),
        ["kukaVc", ns, key] => v
            .get_mut("kukaVc")
            .and_then(Value::as_array_mut)
            .and_then(|a| a.iter_mut().find(|r| r["robotNs"] == *ns))
            .and_then(|r| r.get_mut(*key)),
        _ => None,
    }
    .ok_or_else(|| format!("no Hardware Configuration field {field}"))?;
    if slot == value {
        return Ok(false);
    }
    *slot = value.clone();
    *config = serde_json::from_value(v).map_err(|e| format!("{field}: {e}"))?;
    Ok(true)
}

/// The hardware the fixes give: the virtual cell's values for the robots whose safety rules fail
/// ([`HardwareConfig::with_virtual_cell_values`]), then every field fix as the rules say. The
/// changes, as [`CheckFix::describe`] says them.
pub(crate) fn apply_hardware(config: &HardwareConfig, report: &DeployCheck, rules: &[String]) -> Result<(HardwareConfig, Vec<String>), String> {
    let safety_rules = ["cell.safety", "cell.simulators", "plc.safety"];
    let robots: Vec<String> = report
        .failing()
        .filter(|r| (rules.is_empty() || rules.contains(&r.id)) && safety_rules.contains(&r.id.as_str()))
        .flat_map(|r| r.robots.iter().cloned())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .collect();
    let mut out = config.with_virtual_cell_values(&robots);
    if !robots.is_empty() {
        // with_virtual_cell_values sets the virtual cell's gateway as well: only where a rule says so.
        out.network.gateway = config.network.gateway.clone();
    }
    let mut changes = Vec::new();
    for fix in report.fixes(rules) {
        if let CheckFix::Hardware { field, value } = &fix {
            // The value with_virtual_cell_values gave counts as a change too.
            let before = {
                let mut probe = config.clone();
                set_field(&mut probe, field, value)?
            };
            set_field(&mut out, field, value)?;
            if before {
                changes.push(fix.describe());
            }
        }
    }
    Ok((out, changes))
}

/// `POST` [`factoryos_protocol::api::INFRA_DEPLOY_FIX`]: fix, re-export, check again.
pub async fn fix(state: &AppState, request: DeployFixRequest) -> Result<DeployFixResult, HttpError> {
    let infra = &state.infra;
    let s = infra.subject(request.cell.as_deref(), request.build.as_deref())?;
    if cells::active_cell(&state.config).map(|(c, _)| c) != Some(s.cell.clone()) {
        return Err((StatusCode::CONFLICT, format!("cell {} is not the active cell: make it the active cell first (its Hardware Configuration and exports are the active cell's)", s.cell)));
    }
    let before = infra.check_subject(&s).await?;
    let rules = request.rules.clone().unwrap_or_default();
    let fixes = before.fixes(&rules);
    let mut result = DeployFixResult { before: before.clone(), ..Default::default() };
    if fixes.is_empty() {
        result.error = Some(if before.ok {
            "nothing to fix: the export is deployable".into()
        } else {
            format!(
                "no mechanical fix for {}: {}",
                before.unfixable().iter().map(|r| r.id.as_str()).collect::<Vec<_>>().join(", "),
                before.unfixable().first().and_then(|r| r.hint.clone()).unwrap_or_default()
            )
        });
        result.report = Some(before);
        return Ok(result);
    }
    let info = codegen::read_build_info(&s.dir).ok_or_else(|| (StatusCode::CONFLICT, format!("{}: no readable build.json", s.build)))?;
    // The Hardware Configuration: saved the normal way, the previous file kept.
    if fixes.iter().any(|f| matches!(f, CheckFix::Hardware { .. })) {
        let view = state.hardware.view(Some(&info.plan)).await?;
        let (config, changes) = apply_hardware(&view.config, &before, &rules).map_err(|e| (StatusCode::UNPROCESSABLE_ENTITY, e))?;
        if config != view.config {
            let file = s.root.join(hardware::HARDWARE_FILE);
            if file.is_file() {
                let backup = s.root.join(format!("{}.bak-{}", hardware::HARDWARE_FILE, unix_s(SystemTime::now()) as u64));
                std::fs::copy(&file, &backup)
                    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, format!("backing up {}: {e}", file.display())))?;
                result.backup = Some(backup.display().to_string());
            }
            if let Err(resp) = state.hardware.save(HardwareSave { plan: info.plan.clone(), config }).await {
                let code = resp.status();
                let body = axum::body::to_bytes(resp.into_body(), 1 << 20).await.unwrap_or_default();
                return Err((code, format!("the Hardware Configuration refused the fix: {}", String::from_utf8_lossy(&body))));
            }
            infra.log(format!("Deployability fix: Hardware Configuration of {} saved ({})", s.cell, changes.join("; ")));
        }
        result.changes.extend(changes);
    }
    // The export: the same plan and options, with the option fixes.
    let mut options = serde_json::to_value(&info.request).map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e.to_string()))?;
    options["plan"] = Value::String(info.plan.clone());
    for fix in &fixes {
        if let CheckFix::Export { option, value } = fix {
            if options.get(option).is_some_and(|v| v != value) {
                options[option.as_str()] = value.clone();
                result.changes.push(fix.describe());
            }
        }
    }
    let request = serde_json::from_value(options).map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, format!("export options: {e}")))?;
    let reasons: Vec<String> = fixes
        .iter()
        .filter_map(|f| match f {
            CheckFix::Reexport { reason } => Some(reason.clone()),
            _ => None,
        })
        .collect();
    result.changes.push(if reasons.is_empty() {
        format!("Export {} again with the new settings", info.plan)
    } else {
        format!("Export {} again ({})", info.plan, reasons.join("; "))
    });
    let accepted = state.codegen.start(request).await?;
    let id = accepted.build_id;
    let deadline = std::time::Instant::now() + EXPORT_TIMEOUT;
    loop {
        tokio::time::sleep(Duration::from_millis(300)).await;
        let st = state.codegen.status();
        if st.build_id.as_deref() == Some(id.as_str()) && st.state != CodegenState::Running {
            if st.state != CodegenState::Done {
                result.build = Some(id.clone());
                result.error = Some(format!("the re-export failed: {}", st.message.unwrap_or_default()));
                return Ok(result);
            }
            break;
        }
        if std::time::Instant::now() > deadline {
            result.error = Some(format!("the re-export {id} did not finish in {} s", EXPORT_TIMEOUT.as_secs()));
            return Ok(result);
        }
    }
    result.build = Some(id.clone());
    let s2 = infra.subject(Some(&s.cell), Some(&id))?;
    let report = infra.check_subject(&s2).await?;
    infra.log(format!("Deployability fix of {}: re-exported as {id}: {}", s.build, report.summary()));
    result.report = Some(report);
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_protocol::hardware::{KukaVcController, NetworkConfig, RobotHardware};

    /// scene_fit's hardware.json of 2026-10-16 (plan2cell/crates/plan2cell/tests/fixtures).
    fn scene_fit() -> HardwareConfig {
        let robot = |ns: &str, n: u8| RobotHardware {
            ns: ns.into(),
            tag: format!("R{n}"),
            station: format!("kuka-robi{n}"),
            device_name: format!("kuka_robi{n}"),
            ip: format!("192.168.100.1{n}"),
            model: "unknown".into(),
            external: false,
            safety_bypass: false,
            safety_module: true,
        };
        HardwareConfig {
            network: NetworkConfig {
                head_ip: "192.168.100.10".into(),
                netmask: "255.255.255.0".into(),
                gateway: "0.0.0.0".into(),
                controller_station: "controller".into(),
            },
            robots: vec![robot("robot1", 1), robot("robot2", 2)],
            kuka_vc: vec![KukaVcController { robot_ns: "robot1".into(), kli_ip: "192.168.100.11".into(), station: "kuka-robi1".into() }],
        }
    }

    fn rule(id: &str, robots: &[&str], fixes: Vec<CheckFix>) -> CheckRule {
        CheckRule {
            id: id.into(),
            status: CheckStatus::Fail,
            robots: robots.iter().map(|r| r.to_string()).collect(),
            fixes,
            ..Default::default()
        }
    }

    fn hw(field: &str, value: Value) -> CheckFix {
        CheckFix::Hardware { field: field.into(), value }
    }

    /// The report plan2cell gives for scene_fit (tests/fixtures/deploy_check/scene_fit).
    fn scene_fit_report() -> DeployCheck {
        let mut safety = Vec::new();
        for ns in ["robot1", "robot2"] {
            safety.push(hw(&format!("robots.{ns}.safetyModule"), Value::Bool(false)));
            safety.push(hw(&format!("robots.{ns}.safetyBypass"), Value::Bool(true)));
            safety.push(hw(&format!("robots.{ns}.external"), Value::Bool(true)));
        }
        let mut r = DeployCheck {
            rules: vec![
                rule("cell.simulators", &["robot1", "robot2"], vec![hw("robots.robot1.external", Value::Bool(true)), hw("robots.robot2.external", Value::Bool(true))]),
                rule("cell.safety", &["robot1", "robot2"], safety),
                rule("cell.network", &[], vec![hw("network.gateway", Value::String("192.168.100.1".into()))]),
            ],
            ..Default::default()
        };
        r.count();
        r
    }

    #[test]
    fn the_scene_fit_fixes_give_the_virtual_cells_values() {
        let (fixed, changes) = apply_hardware(&scene_fit(), &scene_fit_report(), &[]).unwrap();
        assert_eq!(fixed, scene_fit().with_virtual_cell_values(&["robot1".into(), "robot2".into()]));
        assert!(fixed.virtual_cell_warnings(&["robot1".into(), "robot2".into()]).is_empty(), "{fixed:?}");
        assert_eq!(fixed.network.gateway, "192.168.100.1");
        assert_eq!(changes.len(), 7, "{changes:?}");
        assert!(changes.contains(&"Hardware Configuration: robot1 Safety module → off".to_owned()), "{changes:?}");
        assert!(changes.contains(&"Hardware Configuration: Network Gateway → 192.168.100.1".to_owned()), "{changes:?}");
        // Only the network rule: the robots stay as they are.
        let (only, changes) = apply_hardware(&scene_fit(), &scene_fit_report(), &["cell.network".into()]).unwrap();
        assert_eq!(only.robots, scene_fit().robots);
        assert_eq!((only.network.gateway.as_str(), changes.len()), ("192.168.100.1", 1));
    }

    #[test]
    fn fields_are_set_by_name() {
        let mut c = scene_fit();
        assert!(set_field(&mut c, "robots.robot2.ip", &Value::String("192.168.100.22".into())).unwrap());
        assert!(!set_field(&mut c, "robots.robot2.ip", &Value::String("192.168.100.22".into())).unwrap());
        assert!(set_field(&mut c, "kukaVc.robot1.kliIp", &Value::String("192.168.100.21".into())).unwrap());
        assert!(set_field(&mut c, "network.headIp", &Value::String("192.168.100.20".into())).unwrap());
        assert_eq!((c.robots[1].ip.as_str(), c.kuka_vc[0].kli_ip.as_str(), c.network.head_ip.as_str()), ("192.168.100.22", "192.168.100.21", "192.168.100.20"));
        assert!(set_field(&mut c, "robots.robot9.ip", &Value::Null).is_err());
        assert!(set_field(&mut c, "network.nope", &Value::Null).is_err());
    }
}
