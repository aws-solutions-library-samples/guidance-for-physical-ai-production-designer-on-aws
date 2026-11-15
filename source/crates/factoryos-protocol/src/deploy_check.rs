//! The deployability check of an export (`plan2cell cell check`, plan2cell's `deploy_check`):
//! can it be deployed to, and run on, the configured target, before Deploy / Run Control Code?
//!
//! * `POST` [`crate::api::INFRA_DEPLOY_CHECK`] ([`DeployCheckRequest`], optional): [`DeployCheck`],
//!   read-only, in seconds: plan2cell's rules on the export (default: the newest of the active
//!   cell) and the cell's facts (its KUKA VCs, their PROFINET addresses, the globals of their
//!   `$config.dat` read over SSM), plus the backend's own (the cell's state, the plan's status,
//!   whether the export is stale). Each rule passes, warns or fails, with a hint and, where it is
//!   mechanical, [`CheckFix`]es.
//! * `POST` [`crate::api::INFRA_DEPLOY_FIX`] ([`DeployFixRequest`]): [`DeployFixResult`]: the fixes of
//!   the failing rules applied (Hardware Configuration values saved the normal way, the previous
//!   `hardware.json` kept as a backup; export options), the plan exported again with the same
//!   options, and the new export checked.
//! * `POST` [`crate::api::INFRA_DEPLOY`] checks first: a failing rule is `409` [`DeployRefused`]
//!   (the report), unless [`crate::infra::DeployRequest::force`].
//!
//! The chat agent has the same as the planner router's `check_deployability` / `fix_deployability`.

use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CheckStatus {
    #[default]
    Pass,
    Skip,
    Warn,
    Fail,
}

impl CheckStatus {
    /// `✓`, `·`, `⚠`, `✗`.
    pub fn mark(self) -> &'static str {
        match self {
            Self::Pass => "✓",
            Self::Skip => "·",
            Self::Warn => "⚠",
            Self::Fail => "✗",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CheckScope {
    /// The export alone (also run at every export).
    #[default]
    Export,
    /// The cell too.
    Cell,
}

/// A mechanical fix (every one is followed by a re-export).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CheckFix {
    /// A Hardware Configuration value (`network.gateway`, `robots.robot1.safetyModule`, ... as
    /// [`crate::hardware::FieldIssue::field`]).
    Hardware { field: String, value: Value },
    /// An export option ([`crate::codegen::BuildRequest`]'s field, camelCase: `profile`, `jobNumber`, ...).
    Export { option: String, value: Value },
    /// Export the plan again (a fixed generator; settings already right).
    Reexport { reason: String },
}

impl CheckFix {
    /// What it changes, for a confirmation: `robot1: Safety module off`, `Gateway 192.168.100.1`,
    /// `re-export (generator bug fixed in a2717d1)`.
    pub fn describe(&self) -> String {
        match self {
            Self::Hardware { field, value } => {
                let v = match value {
                    Value::Bool(true) => "on".to_owned(),
                    Value::Bool(false) => "off".to_owned(),
                    Value::String(s) => s.clone(),
                    v => v.to_string(),
                };
                let label = |k: &str| match k {
                    "safetyModule" => "Safety module",
                    "safetyBypass" => "Safety bypass",
                    "external" => "External",
                    "ip" => "IP",
                    "station" => "Station",
                    "deviceName" => "Device name",
                    "headIp" => "Head IP",
                    "netmask" => "Netmask",
                    "gateway" => "Gateway",
                    "controllerStation" => "Controller station",
                    "kliIp" => "KLI IP",
                    _ => "",
                };
                let parts: Vec<&str> = field.split('.').collect();
                match parts.as_slice() {
                    ["robots", ns, key] => format!("Hardware Configuration: {ns} {} → {v}", nonempty(label(key), key)),
                    ["network", key] => format!("Hardware Configuration: Network {} → {v}", nonempty(label(key), key)),
                    ["kukaVc", ns, key] => format!("Hardware Configuration: KUKA VC of {ns} {} → {v}", nonempty(label(key), key)),
                    _ => format!("Hardware Configuration: {field} → {v}"),
                }
            }
            Self::Export { option, value } => format!("Export option {option} → {}", value.as_str().map(str::to_owned).unwrap_or_else(|| value.to_string())),
            Self::Reexport { reason } => format!("Export again ({reason})"),
        }
    }
}

fn nonempty<'a>(a: &'a str, b: &'a str) -> &'a str {
    if a.is_empty() { b } else { a }
}

/// One rule's outcome.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CheckRule {
    /// `krl.names`, `cell.safety`, `cell.state`, ...
    pub id: String,
    pub title: String,
    pub scope: CheckScope,
    pub status: CheckStatus,
    pub message: String,
    /// What to change, and where in Factory OS.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hint: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub robots: Vec<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub fixes: Vec<CheckFix>,
    /// The generated code is wrong for the target (not a setting), e.g. "generator bug fixed in
    /// a2717d1; re-export".
    #[serde(alias = "generator_bug", skip_serializing_if = "Option::is_none")]
    pub generator_bug: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub details: Vec<String>,
}

impl CheckRule {
    /// `✗ Title: message`.
    pub fn line(&self) -> String {
        format!("{} {}: {}", self.status.mark(), self.title, self.message)
    }
}

/// `POST` [`crate::api::INFRA_DEPLOY_CHECK`] (optional body).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployCheckRequest {
    /// An export of the cell (a build id); default: the newest successful one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// A cell folder name; default: the active cell.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell: Option<String>,
}

/// The report.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployCheck {
    /// The cell (folder name) and the export (build id) checked.
    pub cell: String,
    pub build: String,
    /// The export's directory.
    pub export: String,
    #[serde(alias = "plan_id")]
    pub plan_id: String,
    pub profile: String,
    /// What it was checked against (`None`: the export's rules only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    /// No rule failed: Deploy may go ahead.
    pub ok: bool,
    pub fails: usize,
    pub warns: usize,
    pub rules: Vec<CheckRule>,
    /// Unix seconds.
    pub checked_s: f64,
    #[serde(alias = "elapsed_s")]
    pub elapsed_s: f64,
}

impl DeployCheck {
    /// Recount [`Self::fails`] / [`Self::warns`] / [`Self::ok`].
    pub fn count(&mut self) {
        self.fails = self.rules.iter().filter(|r| r.status == CheckStatus::Fail).count();
        self.warns = self.rules.iter().filter(|r| r.status == CheckStatus::Warn).count();
        self.ok = self.fails == 0;
    }

    pub fn failing(&self) -> impl Iterator<Item = &CheckRule> {
        self.rules.iter().filter(|r| r.status == CheckStatus::Fail)
    }

    /// The fixes of the failing rules (`rules`: only these; empty: all), deduplicated, in order.
    pub fn fixes(&self, rules: &[String]) -> Vec<CheckFix> {
        let mut out: Vec<CheckFix> = Vec::new();
        for r in self.failing().filter(|r| rules.is_empty() || rules.contains(&r.id)) {
            for f in &r.fixes {
                if !out.contains(f) {
                    out.push(f.clone());
                }
            }
        }
        out
    }

    /// Failing rules without a fix (a person has to act).
    pub fn unfixable(&self) -> Vec<&CheckRule> {
        self.failing().filter(|r| r.fixes.is_empty()).collect()
    }

    /// One line: `deployable` / `3 problems (2 fixable)`.
    pub fn summary(&self) -> String {
        if self.ok {
            match self.warns {
                0 => "deployable".to_owned(),
                n => format!("deployable, {n} warning(s)"),
            }
        } else {
            let fixable = self.failing().filter(|r| !r.fixes.is_empty()).count();
            format!(
                "{} problem(s){}{}",
                self.fails,
                if fixable > 0 { format!(" ({fixable} fixable)") } else { String::new() },
                if self.warns > 0 { format!(", {} warning(s)", self.warns) } else { String::new() }
            )
        }
    }

    /// The report as text (one line per rule, hints and details of the problems): for the chat.
    pub fn text(&self) -> String {
        let mut s = format!(
            "Deployability of export {} (cell {}, PLAN_ID {}, profile {}){}: {}\n",
            self.build,
            self.cell,
            self.plan_id,
            self.profile,
            self.target.as_deref().map(|t| format!(" on {t}")).unwrap_or_default(),
            self.summary()
        );
        for r in &self.rules {
            s.push_str(&format!("{} [{}]\n", r.line(), r.id));
            if r.status >= CheckStatus::Warn {
                if let Some(h) = &r.hint {
                    s.push_str(&format!("    fix: {h}\n"));
                }
                if let Some(g) = &r.generator_bug {
                    s.push_str(&format!("    generator: {g}\n"));
                }
                for f in &r.fixes {
                    s.push_str(&format!("    auto-fix: {}\n", f.describe()));
                }
                for d in &r.details {
                    s.push_str(&format!("    {d}\n"));
                }
            }
        }
        s
    }
}

/// `POST` [`crate::api::INFRA_DEPLOY_FIX`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployFixRequest {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell: Option<String>,
    /// Only the fixes of these rules ([`CheckRule::id`]); `None`: of every failing rule.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rules: Option<Vec<String>>,
}

/// The answer of [`crate::api::INFRA_DEPLOY_FIX`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployFixResult {
    /// What was changed ([`CheckFix::describe`]).
    pub changes: Vec<String>,
    /// Where the previous `hardware.json` was kept, when it was changed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub backup: Option<String>,
    /// The new export (`None`: nothing re-exported).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// The check before, and of the new export.
    pub before: DeployCheck,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub report: Option<DeployCheck>,
    /// Why it stopped (the re-export failed, ...).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// The `409` body of a Deploy / Run Control Code a failing check refused.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct DeployRefused {
    pub error: String,
    pub report: DeployCheck,
}

impl DeployRefused {
    /// The refusal in an `Api` error (`409 Conflict: {json}`).
    pub fn from_error(err: &str) -> Option<Self> {
        let json = &err[err.find('{')?..];
        serde_json::from_str::<Self>(json).ok().filter(|r| !r.report.rules.is_empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan2cells_report_reads_as_a_deploy_check() {
        let report = serde_json::json!({
            "schema": "plan2cell/deploy-check@1", "export": "/x", "cell_id": "scene_fit", "plan_id": "16#A1BE",
            "profile": "vw-ks24", "ok": false, "fails": 1, "warns": 0, "elapsed_s": 0.1,
            "rules": [{"id": "cell.network", "title": "Head PLC network", "scope": "cell", "status": "fail",
                       "message": "gateway none", "hint": "Hardware Configuration: Network → Gateway 192.168.100.1",
                       "fixes": [{"kind": "hardware", "field": "network.gateway", "value": "192.168.100.1"}]},
                      {"id": "krl.names", "title": "KRL names", "scope": "export", "status": "fail", "message": "P_PktNr_SPS",
                       "generator_bug": "generator bug fixed in a2717d1; re-export",
                       "fixes": [{"kind": "reexport", "reason": "generator bug fixed in a2717d1"}]}]
        });
        let mut c: DeployCheck = serde_json::from_value(report).unwrap();
        c.count();
        assert_eq!((c.plan_id.as_str(), c.fails, c.ok), ("16#A1BE", 2, false));
        assert_eq!(c.rules[1].generator_bug.as_deref(), Some("generator bug fixed in a2717d1; re-export"));
        let fixes = c.fixes(&[]);
        assert_eq!(fixes.len(), 2);
        assert_eq!(fixes[0].describe(), "Hardware Configuration: Network Gateway → 192.168.100.1");
        assert_eq!(fixes[1].describe(), "Export again (generator bug fixed in a2717d1)");
        assert_eq!(c.fixes(&["krl.names".into()]).len(), 1);
        assert_eq!(c.summary(), "2 problem(s) (2 fixable)");
        assert!(c.text().contains("auto-fix: Hardware Configuration: Network Gateway → 192.168.100.1"));
        let f = CheckFix::Hardware { field: "robots.robot1.safetyModule".into(), value: Value::Bool(false) };
        assert_eq!(f.describe(), "Hardware Configuration: robot1 Safety module → off");
    }

    #[test]
    fn a_refusal_comes_back_out_of_the_api_error() {
        let body = serde_json::to_string(&DeployRefused {
            error: "not deployable".into(),
            report: DeployCheck { rules: vec![CheckRule { id: "cell.safety".into(), ..Default::default() }], ..Default::default() },
        })
        .unwrap();
        let r = DeployRefused::from_error(&format!("409 Conflict: {body}")).unwrap();
        assert_eq!(r.report.rules[0].id, "cell.safety");
        assert!(DeployRefused::from_error("409 Conflict: cell busy").is_none());
    }
}
