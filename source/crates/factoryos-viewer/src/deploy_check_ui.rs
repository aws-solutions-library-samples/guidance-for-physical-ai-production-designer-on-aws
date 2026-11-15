//! Step 3's deployability check ([`api::INFRA_DEPLOY_CHECK`], see
//! [`factoryos_client::protocol::deploy_check`]): "Check deployability" (also by itself for the
//! export step 3 deploys, once per export), the report as a list (✓ / ⚠ / ✗ with the hint of each
//! problem), "Fix and re-export" ([`api::INFRA_DEPLOY_FIX`], after a confirmation that lists what
//! will change) and "Ask the agent" (a new chat tab "Check deploy <cell>" with the report and where
//! the export and the Hardware Configuration are). Deploy is disabled while the check of its export
//! fails ("fix N problems first (or Deploy anyway)").

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::deploy_check::{CheckStatus, DeployCheck, DeployCheckRequest, DeployFixRequest, DeployFixResult};

use crate::theme;
use crate::code_export::export_label;
use crate::rr::egui::{self, Color32, RichText};

enum Msg {
    Check(String, Result<DeployCheck, String>),
    Fix(Result<DeployFixResult, String>),
}

/// What the user asked for in the check's UI.
#[derive(Debug, PartialEq, Eq)]
pub enum CheckAction {
    /// Deploy anyway (`force`), after its confirmation.
    DeployAnyway,
}

pub struct DeployCheckUi {
    api: Api,
    inbox: Arc<Mutex<Vec<Msg>>>,
    report: Option<DeployCheck>,
    /// `check` / `fix` while it runs (for that build).
    pending: Option<(&'static str, String)>,
    error: Option<String>,
    confirm_fix: bool,
    /// The last fix's outcome.
    fixed: Option<DeployFixResult>,
    /// The export last checked by itself.
    auto_for: Option<String>,
    /// A chat tab to open: (title, prompt).
    chat: Option<(String, String)>,
}

impl DeployCheckUi {
    pub fn new(api: Api) -> Self {
        Self {
            api,
            inbox: Arc::default(),
            report: None,
            pending: None,
            error: None,
            confirm_fix: false,
            fixed: None,
            auto_for: None,
            chat: None,
        }
    }

    fn poll(&mut self) {
        for msg in std::mem::take(&mut *self.inbox.lock().unwrap()) {
            match msg {
                Msg::Check(build, result) => {
                    self.pending = None;
                    match result {
                        Ok(r) => {
                            self.error = None;
                            self.report = Some(r);
                        }
                        Err(err) => {
                            self.error = Some(format!("Check deployability of {}: {}", export_label(&build), body_of(&err)));
                        }
                    }
                }
                Msg::Fix(result) => {
                    self.pending = None;
                    match result {
                        Ok(f) => {
                            self.error = f.error.clone();
                            if let Some(r) = &f.report {
                                self.report = Some(r.clone());
                            }
                            // The new export was checked already.
                            self.auto_for = f.build.clone().or(self.auto_for.take());
                            self.fixed = Some(f);
                        }
                        Err(err) => self.error = Some(format!("Fix and re-export: {}", body_of(&err))),
                    }
                }
            }
        }
    }

    /// The report of `build`, if it is the one checked last.
    pub fn report_for(&self, build: &str) -> Option<&DeployCheck> {
        self.report.as_ref().filter(|r| r.build == build)
    }

    /// Why Deploy of `build` waits: its check failed.
    pub fn blocks(&self, build: &str) -> Option<String> {
        self.report_for(build).and_then(deploy_block)
    }

    /// A Deploy the backend refused: its report.
    pub fn refused(&mut self, report: DeployCheck) {
        self.report = Some(report);
    }

    pub fn take_chat(&mut self) -> Option<(String, String)> {
        self.chat.take()
    }

    fn check(&mut self, ctx: &egui::Context, build: &str) {
        self.pending = Some(("check", build.to_owned()));
        self.error = None;
        let (inbox, ctx2, b) = (self.inbox.clone(), ctx.clone(), build.to_owned());
        let body = DeployCheckRequest { build: Some(build.to_owned()), cell: None };
        self.api.post_json(api::INFRA_DEPLOY_CHECK, &body, move |r| {
            inbox.lock().unwrap().push(Msg::Check(b, r));
            ctx2.request_repaint();
        });
    }

    fn fix(&mut self, ctx: &egui::Context, build: &str) {
        self.pending = Some(("fix", build.to_owned()));
        self.error = None;
        self.fixed = None;
        let (inbox, ctx2) = (self.inbox.clone(), ctx.clone());
        let body = DeployFixRequest { build: Some(build.to_owned()), cell: None, rules: None };
        self.api.post_json(api::INFRA_DEPLOY_FIX, &body, move |r| {
            inbox.lock().unwrap().push(Msg::Fix(r));
            ctx2.request_repaint();
        });
    }

    /// The check of `build` (the export step 3 deploys); `can_check`: the server has a cell to
    /// check against (else only on request). Returns Deploy anyway when it was clicked.
    pub fn ui(&mut self, ui: &mut egui::Ui, build: Option<&str>, can_check: bool) -> Option<CheckAction> {
        self.poll();
        let ctx = ui.ctx().clone();
        let Some(build) = build else { return None };
        if can_check && self.pending.is_none() && self.auto_for.as_deref() != Some(build) {
            self.auto_for = Some(build.to_owned());
            self.check(&ctx, build);
        }
        let mut action = None;
        let busy = self.pending.is_some();
        ui.horizontal_wrapped(|ui| {
            let label = match &self.pending {
                Some(("check", _)) => "Checking…",
                _ => "Check deployability",
            };
            if ui
                .add_enabled(!busy, egui::Button::new(RichText::new(label).small()).corner_radius(4))
                .on_hover_text(
                    "Read-only, a few seconds: can this export be deployed to, and run on, the virtual cell? The KRL names against \
                     the KUKA VCs' $config.dat, the safety settings and addresses against the controllers, the head's network, \
                     the job number and profile, the PLAN_ID, the cell's state and the plan's status. Deploy runs it too.",
                )
                .clicked()
            {
                self.check(&ctx, build);
            }
            let report = self.report_for(build).cloned();
            if let Some(r) = report.as_ref().filter(|r| !r.ok && !r.fixes(&[]).is_empty()) {
                let label = if matches!(self.pending, Some(("fix", _))) { "Fixing…" } else { "Fix and re-export" };
                if ui
                    .add_enabled(!busy, egui::Button::new(RichText::new(label).small().color(theme::current().warn)).corner_radius(4))
                    .on_hover_text(format!("{} fix(es): see the list before anything changes", r.fixes(&[]).len()))
                    .clicked()
                {
                    self.confirm_fix = true;
                }
            }
            if let Some(r) = &report {
                if ui
                    .add_enabled(true, egui::Button::new(RichText::new("Ask the agent").small()).corner_radius(4))
                    .on_hover_text("A new chat tab with this report: the agent explains each problem, proposes fixes and applies them with your consent")
                    .clicked()
                {
                    self.chat = Some((format!("Check deploy {}", r.cell), chat_prompt(r)));
                }
                // Deploying past a failed check: Advanced only.
                if !r.ok
                    && crate::ui_mode::is_advanced(ui)
                    && ui
                        .add_enabled(!busy, egui::Button::new(RichText::new("Deploy anyway…").small()).corner_radius(4))
                        .on_hover_text("Deploy without the check (it will most likely fail on the cell, see the problems)")
                        .clicked()
                {
                    action = Some(CheckAction::DeployAnyway);
                }
            }
        });
        if let Some(r) = self.report_for(build).cloned() {
            report_ui(ui, &r);
        } else if let Some(r) = self.report.as_ref().filter(|_| !busy) {
            ui.small(RichText::new(format!("Last checked: {} ({})", export_label(&r.build), r.summary())).weak());
        }
        if let Some(f) = &self.fixed {
            for line in fix_lines(f) {
                ui.small(RichText::new(line).color(Color32::GRAY));
            }
        }
        if let Some(err) = &self.error {
            ui.small(RichText::new(err).color(theme::current().danger));
        }
        if self.confirm_fix {
            self.fix_dialog(&ctx, build);
        }
        action
    }

    fn fix_dialog(&mut self, ctx: &egui::Context, build: &str) {
        let Some(report) = self.report_for(build).cloned() else {
            self.confirm_fix = false;
            return;
        };
        let mut go = false;
        let modal = egui::Modal::new(egui::Id::new("deploy_check_fix")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(460.0);
            ui.heading("Fix and re-export?");
            ui.label(format!("For {} ({}), this changes:", export_label(build), report.summary()));
            for f in report.fixes(&[]) {
                ui.label(format!("• {}", f.describe()));
            }
            ui.label(
                "Then the plan is exported again with the same options and the new export checked. The previous \
                 hardware.json is kept as a backup; the cell itself is not touched.",
            );
            let rest = report.unfixable();
            if !rest.is_empty() {
                ui.label(RichText::new(format!("Not fixed by this: {}", rest.iter().map(|r| r.title.as_str()).collect::<Vec<_>>().join(", "))).color(theme::current().warn));
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button(RichText::new("Fix and re-export").color(theme::current().warn)).clicked() {
                    go = true;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_fix = false;
                }
            });
        });
        if modal.should_close() {
            self.confirm_fix = false;
        }
        if go {
            self.confirm_fix = false;
            self.fix(ctx, build);
        }
    }
}

/// "fix N problem(s) first (or Deploy anyway)" for a failing check.
pub fn deploy_block(r: &DeployCheck) -> Option<String> {
    (!r.ok).then(|| format!("fix {} problem(s) first (or Deploy anyway): {}", r.fails, r.failing().map(|x| x.title.as_str()).collect::<Vec<_>>().join(", ")))
}

/// The report as list lines: (mark, text, colour); the problems' hints as their own lines.
pub fn report_lines(r: &DeployCheck) -> Vec<(String, Color32)> {
    let mut out = vec![(
        format!("Deployability: {}", r.summary()),
        if r.ok { if r.warns > 0 { theme::current().warn } else { theme::current().ok } } else { theme::current().danger },
    )];
    for rule in &r.rules {
        let color = match rule.status {
            CheckStatus::Fail => theme::current().danger,
            CheckStatus::Warn => theme::current().warn,
            CheckStatus::Pass => Color32::GRAY,
            CheckStatus::Skip => Color32::DARK_GRAY,
        };
        let text = match rule.status {
            CheckStatus::Pass | CheckStatus::Skip => format!("{} {}", rule.status.mark(), rule.title),
            _ => rule.line(),
        };
        out.push((text, color));
        if rule.status >= CheckStatus::Warn {
            if let Some(g) = &rule.generator_bug {
                out.push((format!("    {g}"), color));
            }
            if let Some(h) = &rule.hint {
                out.push((format!("    Fix: {h}"), Color32::LIGHT_GRAY));
            }
        }
    }
    out
}

fn report_ui(ui: &mut egui::Ui, r: &DeployCheck) {
    let lines = report_lines(r);
    let (head, rest) = lines.split_first().expect("the summary line");
    ui.small(RichText::new(&head.0).color(head.1));
    if !crate::ui_mode::is_advanced(ui) {
        // Simple: the summary and the problems; the passed rules, hints and details are Advanced.
        for rule in r.rules.iter().filter(|x| x.status >= CheckStatus::Warn) {
            let color = if rule.status == CheckStatus::Fail { theme::current().danger } else { theme::current().warn };
            ui.small(RichText::new(rule.line()).color(color));
        }
        return;
    }
    for (text, color) in rest {
        ui.small(RichText::new(text).color(*color));
    }
    let details: Vec<&String> = r.rules.iter().filter(|x| x.status >= CheckStatus::Warn).flat_map(|x| &x.details).collect();
    if !details.is_empty() || r.target.is_some() {
        egui::CollapsingHeader::new(RichText::new("Details").small()).id_salt("deploy_check_details").show(ui, |ui| {
            if let Some(t) = &r.target {
                ui.small(RichText::new(format!("Target: {t}")).weak());
            }
            for d in details {
                ui.small(RichText::new(d).monospace().weak());
            }
            ui.small(RichText::new(format!("{} in {:.1} s", export_label(&r.build), r.elapsed_s)).weak());
        });
    }
}

/// What a fix did, as lines.
pub fn fix_lines(f: &DeployFixResult) -> Vec<String> {
    let mut out: Vec<String> = f.changes.iter().map(|c| format!("Fixed: {c}")).collect();
    if let Some(b) = &f.backup {
        out.push(format!("Previous Hardware Configuration kept as {b}"));
    }
    match (&f.build, &f.report) {
        (Some(b), Some(r)) => out.push(format!("Re-exported as {}: {}", export_label(b), r.summary())),
        (Some(b), None) => out.push(format!("Re-export {}", export_label(b))),
        _ => {}
    }
    out
}

/// The chat agent's first message for "Ask the agent": the report, where the export and the
/// Hardware Configuration are, and what it may do.
pub fn chat_prompt(r: &DeployCheck) -> String {
    let cell_root = r.export.split("/plan2cell/builds/").next().unwrap_or("");
    format!(
        "Check whether export {build} of cell {cell} can be deployed to the virtual cell and run there.\n\n\
         The deployability check says:\n{text}\n\
         Context: the export is {export} (plan2cell.toml: the settings it was built with; plc/project.json: the head's PROFINET \
         devices; krl/<robot>/: the KRL). The cell's Hardware Configuration is {cell_root}/plan2cell/hardware.json, its base \
         plan2cell.toml {cell_root}/plan2cell/plan2cell.toml.\n\n\
         Explain each problem in plain words and what it would do on the cell. Propose the fix for each (which setting, where in \
         Physical AI Production Designer). Check again with check_deployability (cell {cell}, build {build}) if anything may have changed. Apply fixes \
         only with my consent, with fix_deployability (it saves the Hardware Configuration with a backup, re-exports and checks \
         the new export), then check again and tell me the new export's id. Do not deploy, start or stop the cell.",
        build = r.build,
        cell = r.cell,
        text = r.text(),
        export = r.export,
    )
}

fn body_of(err: &str) -> &str {
    match err.split_once(": ") {
        Some((status, body)) if status.len() >= 3 && status.as_bytes()[..3].iter().all(u8::is_ascii_digit) => body,
        _ => err,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_client::protocol::deploy_check::{CheckFix, CheckRule};

    fn report(ok: bool) -> DeployCheck {
        let mut r = DeployCheck {
            cell: "scene_fit".into(),
            build: "20261016T174822044Z-bt_1792156523_scene_fit".into(),
            export: "/var/lib/factoryos/cells/scene_fit/plan2cell/builds/bt_1792156523_scene_fit/20261016T174822044Z".into(),
            rules: vec![
                CheckRule { id: "export.files".into(), title: "Export complete".into(), message: "ok".into(), ..Default::default() },
                CheckRule {
                    id: "cell.network".into(),
                    title: "Head PLC network".into(),
                    status: if ok { CheckStatus::Warn } else { CheckStatus::Fail },
                    message: "gateway none (0.0.0.0)".into(),
                    hint: Some("Hardware Configuration: Network → Gateway 192.168.100.1".into()),
                    fixes: vec![CheckFix::Hardware { field: "network.gateway".into(), value: "192.168.100.1".into() }],
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        r.count();
        r
    }

    #[test]
    fn a_failing_check_blocks_deploy_and_lists_its_problems() {
        let r = report(false);
        assert_eq!(deploy_block(&r).as_deref(), Some("fix 1 problem(s) first (or Deploy anyway): Head PLC network"));
        assert!(deploy_block(&report(true)).is_none());
        let lines = report_lines(&r);
        assert_eq!(lines[0].0, "Deployability: 1 problem(s) (1 fixable)");
        assert_eq!(lines[1].0, "✓ Export complete");
        assert_eq!(lines[2].0, "✗ Head PLC network: gateway none (0.0.0.0)");
        assert_eq!(lines[3].0, "    Fix: Hardware Configuration: Network → Gateway 192.168.100.1");
        assert_eq!(lines[2].1, theme::current().danger);
    }

    #[test]
    fn the_agent_gets_the_report_and_the_files() {
        let p = chat_prompt(&report(false));
        assert!(p.contains("export 20261016T174822044Z-bt_1792156523_scene_fit of cell scene_fit"), "{p}");
        assert!(p.contains("✗ Head PLC network: gateway none (0.0.0.0) [cell.network]"), "{p}");
        assert!(p.contains("/var/lib/factoryos/cells/scene_fit/plan2cell/hardware.json"), "{p}");
        assert!(p.contains("fix_deployability") && p.contains("Do not deploy, start or stop the cell"), "{p}");
    }

    #[test]
    fn a_fix_says_what_changed() {
        let f = DeployFixResult {
            changes: vec!["Hardware Configuration: Network Gateway → 192.168.100.1".into()],
            backup: Some("/c/plan2cell/hardware.json.bak-1".into()),
            build: Some("20261016T190000000Z-bt_1792156523_scene_fit".into()),
            report: Some(DeployCheck { ok: true, ..Default::default() }),
            ..Default::default()
        };
        let lines = fix_lines(&f);
        assert_eq!(lines[0], "Fixed: Hardware Configuration: Network Gateway → 192.168.100.1");
        assert!(lines[2].starts_with("Re-exported as") && lines[2].ends_with(": deployable"), "{lines:?}");
    }
}
