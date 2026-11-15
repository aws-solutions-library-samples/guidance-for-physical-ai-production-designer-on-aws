//! Hardware Configuration, part of the guided flow's step 2 "Generate PLC + robot code": its button
//! and one-line summary sit in that box under the export (the export says which robots there are);
//! the editor is a side panel at the right, like the export results.
//!
//! The backend lists plan2cell's values (`plan2cell hardware <plan>`) for the plan of the export
//! the next steps use, else the plan the export form shows
//! ([`api::cell_hardware`]): the head PLC's network, each robot's PROFINET station name, device
//! name and IP, and the KUKA virtual controllers' KLI addresses. The form starts from the saved
//! configuration (else those defaults), checks every value as it is typed
//! ([`HardwareConfig::validate`], the same rules as plan2cell's), and saves it for every later
//! export ([`api::CELL_HARDWARE`] `PUT`); "Reset to defaults" goes back to plan2cell's values.
//!
//! On a plan2cell virtual cell ([`HardwareView::virtual_cell_robots`]) the defaults are its KUKA
//! VCs' (External, Safety bypass, no safety module, gateway 192.168.100.1). A saved configuration
//! with other values is left as it is, under a warning ([`HardwareConfig::virtual_cell_warnings`])
//! with "Use the virtual cell's values" (sets them and saves).

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::hardware::{FieldIssue, HardwareConfig, HardwareRejected, HardwareSave, HardwareSource, HardwareView};
use factoryos_client::protocol::infra::utc_hhmm;
use web_time::{Duration, Instant};

use crate::theme;
use crate::rr::egui::{self, RichText};
use crate::sidebar;

/// How long to wait before asking again after a failed load (no plan yet, no binary, ...).
const RETRY: Duration = Duration::from_secs(15);

enum Msg {
    View { key: Key, view: Box<Result<HardwareView, String>> },
    /// The reply to a save made for `key` (dropped when the panel shows another cell or plan now).
    Saved { key: Option<Key>, result: Box<Result<HardwareView, String>> },
}

/// What a view was asked for: the cell and the plan.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Key {
    cell: String,
    plan: Option<String>,
}

pub struct HardwareUi {
    api: Api,
    inbox: Arc<Mutex<Vec<Msg>>>,
    /// The editor panel is shown.
    pub open: bool,
    key: Option<Key>,
    asked: Option<(Key, Instant)>,
    view: Option<Result<HardwareView, String>>,
    /// The form's values.
    edit: Option<HardwareConfig>,
    /// Per KUKA VC controller: its KLI IP and station follow its robot's.
    follow: Vec<bool>,
    saving: bool,
    /// The server's verdict on the last save (its `422` problems).
    rejected: Vec<FieldIssue>,
    save_result: Option<Result<String, String>>,
    /// A save succeeded since [`Self::take_saved`] last looked.
    saved_since: bool,
}

impl HardwareUi {
    pub fn new(api: Api) -> Self {
        Self {
            api,
            inbox: Arc::default(),
            open: false,
            key: None,
            asked: None,
            view: None,
            edit: None,
            follow: Vec::new(),
            saving: false,
            rejected: Vec::new(),
            save_result: None,
            saved_since: false,
        }
    }

    fn sender(&self, ctx: &egui::Context) -> impl Fn(Msg) + Send + Sync + 'static {
        let (inbox, ctx) = (self.inbox.clone(), ctx.clone());
        move |msg| {
            inbox.lock().unwrap().push(msg);
            ctx.request_repaint();
        }
    }

    /// Call once per frame with the active cell and the plan the export form has chosen.
    pub fn poll(&mut self, ctx: &egui::Context, cell: &str, plan: Option<&str>) {
        let key = Key { cell: cell.to_owned(), plan: plan.map(str::to_owned) };
        if self.key.as_ref() != Some(&key) {
            let cell_changed = self.key.as_ref().is_none_or(|k| k.cell != key.cell);
            self.key = Some(key.clone());
            // Another plan: other robots, but the edits of this cell's network are kept until the
            // view comes (then the saved values win).
            if cell_changed {
                self.edit = None;
                self.view = None;
                self.rejected.clear();
                self.save_result = None;
            }
            self.asked = None;
        }
        let messages = std::mem::take(&mut *self.inbox.lock().unwrap());
        for msg in messages {
            self.receive(msg);
        }
        if key.cell.is_empty() {
            return;
        }
        let ask = match (&self.view, &self.asked) {
            (_, None) => true,
            (Some(Err(_)), Some((_, at))) => at.elapsed() > RETRY,
            _ => false,
        };
        if ask {
            self.asked = Some((key.clone(), Instant::now()));
            let send = self.sender(ctx);
            self.api.get(&api::cell_hardware(key.plan.as_deref()), move |view| {
                send(Msg::View { key, view: Box::new(view) })
            });
        }
    }

    fn receive(&mut self, msg: Msg) {
        match msg {
            Msg::View { key, view } => {
                if self.key.as_ref() != Some(&key) {
                    return;
                }
                if let Ok(view) = view.as_ref() {
                    let dirty = self.dirty();
                    let same_robots = self.edit.as_ref().is_some_and(|e| robots_of(e) == robots_of(&view.config));
                    if !dirty || !same_robots {
                        self.set_edit(view.config.clone());
                    }
                }
                self.view = Some(*view);
            }
            Msg::Saved { key, result } => {
                self.saving = false;
                if key != self.key {
                    // Saved for another cell or plan than the one shown now: drop it, and read the
                    // current one again (the save may have changed what the server stores for it).
                    self.asked = None;
                    return;
                }
                match *result {
                    Ok(view) => {
                        self.rejected.clear();
                        self.saved_since = true;
                        self.save_result = Some(Ok(format!("Saved: every export of cell {} uses it from now on", view.cell)));
                        self.set_edit(view.config.clone());
                        self.view = Some(Ok(view));
                    }
                    Err(err) => {
                        match rejected_errors(&err) {
                            Some(errors) => {
                                self.save_result = Some(Err(format!("Not saved: {} problem(s)", errors.len())));
                                self.rejected = errors;
                            }
                            None => self.save_result = Some(Err(err)),
                        }
                    }
                }
            }
        }
    }

    fn set_edit(&mut self, config: HardwareConfig) {
        self.follow = config
            .kuka_vc
            .iter()
            .map(|c| config.robot(&c.robot_ns).is_some_and(|r| r.ip == c.kli_ip && r.station == c.station))
            .collect();
        self.edit = Some(config);
    }

    fn loaded(&self) -> Option<&HardwareView> {
        self.view.as_ref().and_then(|v| v.as_ref().ok())
    }

    /// The form differs from what is saved (or from the defaults when nothing is).
    fn dirty(&self) -> bool {
        match (self.loaded(), &self.edit) {
            (Some(view), Some(edit)) => *edit != view.config,
            _ => false,
        }
    }

    /// For the export form: what its builds use.
    pub fn export_note(&self) -> Option<String> {
        let view = self.loaded()?;
        let source = match view.source {
            HardwareSource::Saved => "saved",
            HardwareSource::Defaults => "defaults",
        };
        let unsaved = if self.dirty() { ", unsaved edits not included" } else { "" };
        Some(format!("Uses hardware config ({source}{unsaved})"))
    }

    /// Whether a save succeeded since the last call (the exports' STALE flags then change).
    pub fn take_saved(&mut self) -> bool {
        std::mem::take(&mut self.saved_since)
    }

    /// The saved configuration as step 2 compares an export with it (`None`: not loaded).
    pub fn saved_hardware(&self) -> Option<crate::flow::SavedHardware> {
        let view = self.loaded()?;
        Some(crate::flow::SavedHardware { saved: view.source == HardwareSource::Saved, etag: view.etag.clone(), saved_s: view.saved_s })
    }

    /// The plan whose robots are shown (its file stem), once loaded.
    pub fn plan(&self) -> Option<&str> {
        self.loaded().map(|v| plan_stem(&v.plan))
    }

    /// What the summary under the Hardware Configuration button in step 2 shows.
    pub fn flow_facts(&self) -> crate::flow::HardwareFacts {
        let Some(view) = self.loaded() else {
            return crate::flow::HardwareFacts {
                error: match &self.view {
                    Some(Err(err)) => Some(short(err)),
                    _ => None,
                },
                ..Default::default()
            };
        };
        crate::flow::HardwareFacts {
            loaded: true,
            error: None,
            saved: view.source == HardwareSource::Saved,
            saved_at: view.saved_s.map(utc_hhmm),
            head_ip: view.config.network.head_ip.clone(),
            robots: view.config.robots.len(),
            dirty: self.dirty(),
            problems: self.edit.as_ref().map_or(0, |e| e.validate().len()),
            differs: !view.warnings.is_empty(),
        }
    }

    /// The button in step 2's box, under the export (the summary is [`Self::flow_facts`]).
    pub fn sidebar_ui(&mut self, ui: &mut egui::Ui) {
        let label = if self.open { "Hide Hardware Configuration" } else { "Hardware Configuration" };
        if sidebar::button_response(ui, label, true)
            .on_hover_text(
                "Head PLC network, PROFINET station names, device names and IPs, KUKA VC addresses of the export's robots: \
                 saved, every later export uses them (saving after an export: export again)",
            )
            .clicked()
        {
            self.open = !self.open;
        }
    }

    /// The editor panel.
    pub fn panel_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        ui.horizontal(|ui| {
            ui.strong("Hardware Configuration");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("✕").on_hover_text("Hide the hardware configuration").clicked() {
                    self.open = false;
                }
                if ui.button("↻").on_hover_text("Load it again (unsaved edits are kept for the same robots)").clicked() {
                    self.asked = None;
                }
            });
        });
        ui.separator();
        let view = match &self.view {
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.weak("Loading plan2cell's hardware values…");
                });
                return;
            }
            Some(Err(err)) => {
                ui.colored_label(theme::current().danger, err);
                return;
            }
            Some(Ok(view)) => view.clone(),
        };
        let Some(mut edit) = self.edit.clone() else { return };
        let source = match view.source {
            HardwareSource::Saved => format!("saved{}", view.saved_s.map(|s| format!(" at {}", utc_hhmm(s))).unwrap_or_default()),
            HardwareSource::Defaults => "plan2cell defaults, not saved yet".to_owned(),
        };
        ui.small(RichText::new(format!("Cell {} · robots of {} · {source}", view.cell, plan_stem(&view.plan))).weak());

        let mut errors = edit.validate();
        // The server's own verdict on the last save, for fields the form's rules pass.
        for issue in &self.rejected {
            if !errors.iter().any(|e| e.field == issue.field) {
                errors.push(issue.clone());
            }
        }
        let vc_warnings = edit.virtual_cell_warnings(&view.virtual_cell_robots);
        let mut warnings = edit.wiring_warnings(view.wiring.as_ref());
        warnings.extend(vc_warnings.iter().cloned());
        let mut save: Option<HardwareConfig> = None;

        // Save, Reset to defaults and Revert, pinned to the panel's bottom (before the form, which
        // gets the room above), with the last save's outcome above them.
        theme::action_row(ui, "hardware_actions", |ui| {
            match &self.save_result {
                Some(Ok(message)) => {
                    ui.label(RichText::new(format!("✓ {message}")).small().color(theme::current().ok));
                }
                Some(Err(message)) => {
                    ui.label(RichText::new(format!("✗ {message}")).small().color(theme::current().danger));
                }
                None => {}
            }
            theme::action_buttons(ui, |ui| {
                let dirty = edit != view.config;
                let can_save = !self.saving && edit.validate().is_empty() && (dirty || view.source == HardwareSource::Defaults);
                let label = if self.saving { "Saving…" } else { "Save" };
                let save_button = ui
                    .add_enabled(can_save, theme::primary(ui, label))
                    .on_hover_text("Check with plan2cell and save it for every export of this cell")
                    .on_disabled_hover_text("Fix the problems first (or change something)");
                if save_button.clicked() {
                    save = Some(edit.clone());
                }
                if ui
                    .add_enabled(edit != view.defaults, egui::Button::new("Reset to defaults"))
                    .on_hover_text("plan2cell's values for this plan (Save to keep them)")
                    .clicked()
                {
                    edit = view.defaults.clone();
                    self.rejected.clear();
                    self.save_result = None;
                    self.set_follow(&edit);
                }
                if ui.add_enabled(dirty, egui::Button::new("Revert")).on_hover_text("Back to what is saved").clicked() {
                    edit = view.config.clone();
                    self.rejected.clear();
                    self.set_follow(&edit);
                }
            });
        });

        egui::ScrollArea::vertical().id_salt("hardware_form").auto_shrink([false, false]).show(ui, |ui| {
            ui.add_space(4.0);
            ui.label(RichText::new("Head PLC network").strong());
            egui::Grid::new("hardware_network").num_columns(2).spacing([8.0, 4.0]).show(ui, |ui| {
                let n = &mut edit.network;
                row(ui, "Head PLC IP", &mut n.head_ip, "network.headIp", &errors, &warnings, true);
                row(ui, "Netmask", &mut n.netmask, "network.netmask", &errors, &warnings, true);
                row(ui, "Gateway", &mut n.gateway, "network.gateway", &errors, &warnings, true)
                    .on_hover_text("0.0.0.0 (or empty) = none");
                row(ui, "Controller station", &mut n.controller_station, "network.controllerStation", &errors, &warnings, true)
                    .on_hover_text("The head PLC's own PROFINET station name");
            });

            ui.add_space(8.0);
            ui.label(RichText::new("Robots (PROFINET devices of the head)").strong());
            egui::Grid::new("hardware_robots").num_columns(8).striped(true).spacing([8.0, 4.0]).show(ui, |ui| {
                for title in ["Robot", "Station name", "Device name", "IP", "Model", "External", "Safety bypass", "Safety module"] {
                    ui.small(RichText::new(title).strong());
                }
                ui.end_row();
                for r in &mut edit.robots {
                    let what = if r.external { "virtual / real controller" } else { "CODESYS robot simulator" };
                    ui.label(format!("{} ({})", r.ns, r.tag)).on_hover_text(what);
                    let ns = r.ns.clone();
                    cell(ui, &mut r.station, &format!("robots.{ns}.station"), &errors, &warnings, 130.0, true);
                    cell(ui, &mut r.device_name, &format!("robots.{ns}.deviceName"), &errors, &warnings, 120.0, true);
                    cell(ui, &mut r.ip, &format!("robots.{ns}.ip"), &errors, &warnings, 110.0, true);
                    ui.label(RichText::new(&r.model).small().weak()).on_hover_text("read-only (the plan2cell.toml's robot model)");
                    flag(ui, &mut r.external, &format!("robots.{ns}.external"), &errors, &warnings)
                        .on_hover_text("A real or virtual robot controller (KUKA VC), not a CODESYS robot simulator");
                    flag(ui, &mut r.safety_bypass, &format!("robots.{ns}.safetyBypass"), &errors, &warnings)
                        .on_hover_text("SIMULATION ONLY: the head does not wait for this robot's PROFIsafe bits. Never on a real cell");
                    flag(ui, &mut r.safety_module, &format!("robots.{ns}.safetyModule"), &errors, &warnings).on_hover_text(
                        "The head's PN device has the PROFIsafe module (FDIO64, slot 1). Off (simulation only, needs External and \
                         Safety bypass): DAP DIM 3 with slot 1 empty, as a KUKA VC needs",
                    );
                    ui.end_row();
                }
            });
            ui.small(RichText::new("Station names: a-z, 0-9, '-' and '.'; device names: IEC identifiers (a letter, then letters, digits, '_').").weak());

            if !edit.kuka_vc.is_empty() {
                ui.add_space(8.0);
                ui.label(RichText::new("KUKA virtual controllers").strong());
                if self.follow.len() != edit.kuka_vc.len() {
                    self.follow.resize(edit.kuka_vc.len(), true);
                }
                egui::Grid::new("hardware_kuka_vc").num_columns(4).striped(true).spacing([8.0, 4.0]).show(ui, |ui| {
                    for title in ["Robot", "Same as robot", "KLI IP", "Station"] {
                        ui.small(RichText::new(title).strong());
                    }
                    ui.end_row();
                    for (i, c) in edit.kuka_vc.iter_mut().enumerate() {
                        ui.label(&c.robot_ns);
                        ui.checkbox(&mut self.follow[i], "")
                            .on_hover_text("The controller answers at its robot's IP and station (what the head PLC addresses)");
                        if self.follow[i]
                            && let Some(r) = edit.robots.iter().find(|r| r.ns == c.robot_ns)
                        {
                            c.kli_ip = r.ip.clone();
                            c.station = r.station.clone();
                        }
                        let editable = !self.follow[i];
                        let ns = c.robot_ns.clone();
                        cell(ui, &mut c.kli_ip, &format!("kukaVc.{ns}.kliIp"), &errors, &warnings, 110.0, editable);
                        cell(ui, &mut c.station, &format!("kukaVc.{ns}.station"), &errors, &warnings, 130.0, editable);
                        ui.end_row();
                    }
                });
            }

            // Problems of no field shown above (`config`, a robot the plan does not have, ...).
            let shown = shown_fields(&edit);
            for e in errors.iter().filter(|e| !shown.contains(&e.field)) {
                ui.label(RichText::new(format!("✗ {}", e.message)).small().color(theme::current().danger));
            }

            ui.add_space(8.0);
            if !vc_warnings.is_empty() {
                egui::Frame::new().stroke(egui::Stroke::new(1.0, theme::current().warn)).inner_margin(6.0).corner_radius(4).show(ui, |ui| {
                    ui.label(
                        RichText::new(format!("⚠ {} value(s) will not work on the virtual cell (its KUKA VCs)", vc_warnings.len()))
                            .color(theme::current().warn),
                    );
                    for w in &vc_warnings {
                        ui.small(RichText::new(format!("{}: {}", w.field, w.message)).color(theme::current().warn));
                    }
                    if let Some(fixed) = virtual_cell_fix(&view, &edit)
                        && ui
                            .add_enabled(!self.saving, egui::Button::new("Use the virtual cell's values"))
                            .on_hover_text(
                                "External and Safety bypass on, Safety module off for its robots, gateway 192.168.100.1; and save",
                            )
                            .clicked()
                    {
                        edit = fixed.clone();
                        self.set_follow(&edit);
                        save = Some(fixed);
                    }
                });
                ui.add_space(4.0);
            }
            let wiring_warnings: Vec<&FieldIssue> = warnings.iter().filter(|w| !vc_warnings.contains(w)).collect();
            if !wiring_warnings.is_empty() {
                egui::Frame::new().stroke(egui::Stroke::new(1.0, theme::current().warn)).inner_margin(6.0).corner_radius(4).show(ui, |ui| {
                    ui.label(
                        RichText::new(format!("⚠ {} value(s) differ from the virtual cell's wiring", wiring_warnings.len())).color(theme::current().warn),
                    );
                    for w in wiring_warnings {
                        ui.small(RichText::new(format!("{}: {}", w.field, w.message)).color(theme::current().warn));
                    }
                });
                ui.add_space(4.0);
            }
            ui.small(RichText::new(&view.note).weak());
            if let Some(wiring) = &view.wiring {
                ui.small(RichText::new(format!("Virtual cell wiring: {}", wiring.source)).weak());
            }
            if let Some(file) = &view.cell_config {
                ui.small(RichText::new(format!("Start production uses {file} (rendered from it with these KUKA VC addresses)")).weak());
            }

        });
        if let Some(config) = save {
            self.saving = true;
            self.save_result = None;
            let request = HardwareSave { plan: view.plan.clone(), config };
            let send = self.sender(&ctx);
            let key = self.key.clone();
            self.api.put_json(api::CELL_HARDWARE, &request, move |result| send(Msg::Saved { key, result: Box::new(result) }));
        }
        if self.edit.as_ref() != Some(&edit) {
            // An edit makes the server's last verdict stale for the fields it changed.
            if let Some(old) = &self.edit {
                let changed = changed_fields(old, &edit);
                self.rejected.retain(|r| !changed.iter().any(|f| r.field.starts_with(f.as_str())));
            }
            self.edit = Some(edit);
        }
    }

    fn set_follow(&mut self, config: &HardwareConfig) {
        self.follow = config
            .kuka_vc
            .iter()
            .map(|c| config.robot(&c.robot_ns).is_some_and(|r| r.ip == c.kli_ip && r.station == c.station))
            .collect();
    }
}

/// The form with the virtual cell's values for its robots (and saved as such), if that changes it
/// and passes the form's rules; `None` off a virtual cell.
fn virtual_cell_fix(view: &HardwareView, edit: &HardwareConfig) -> Option<HardwareConfig> {
    let fixed = edit.with_virtual_cell_values(&view.virtual_cell_robots);
    (fixed != *edit && fixed.validate().is_empty()).then_some(fixed)
}

/// The fields the form has a text field or checkbox for.
fn shown_fields(config: &HardwareConfig) -> Vec<String> {
    let mut out: Vec<String> =
        ["network.headIp", "network.netmask", "network.gateway", "network.controllerStation"].map(str::to_owned).to_vec();
    for r in &config.robots {
        out.extend(["station", "deviceName", "ip", "external", "safetyBypass", "safetyModule"].map(|k| format!("robots.{}.{k}", r.ns)));
    }
    for c in &config.kuka_vc {
        out.extend(["kliIp", "station"].map(|k| format!("kukaVc.{}.{k}", c.robot_ns)));
    }
    out
}

/// The robots' ns, to tell whether edits still fit a view.
fn robots_of(config: &HardwareConfig) -> Vec<&str> {
    config.robots.iter().map(|r| r.ns.as_str()).collect()
}

/// The field names (as in [`FieldIssue::field`]) whose values differ.
fn changed_fields(a: &HardwareConfig, b: &HardwareConfig) -> Vec<String> {
    let mut out = Vec::new();
    let (n, m) = (&a.network, &b.network);
    for (name, x, y) in [
        ("network.headIp", &n.head_ip, &m.head_ip),
        ("network.netmask", &n.netmask, &m.netmask),
        ("network.gateway", &n.gateway, &m.gateway),
        ("network.controllerStation", &n.controller_station, &m.controller_station),
    ] {
        if x != y {
            out.push(name.to_owned());
        }
    }
    for r in &b.robots {
        if a.robot(&r.ns) != Some(r) {
            out.push(format!("robots.{}", r.ns));
        }
    }
    for c in &b.kuka_vc {
        if a.controller(&c.robot_ns) != Some(c) {
            out.push(format!("kukaVc.{}", c.robot_ns));
        }
    }
    out
}

/// The problems of a `422` answer (`422 Unprocessable Entity: {"errors": [...]}`).
fn rejected_errors(err: &str) -> Option<Vec<FieldIssue>> {
    let rest = err.strip_prefix("422")?;
    let json = &rest[rest.find('{')?..];
    serde_json::from_str::<HardwareRejected>(json).ok().map(|r| r.errors).filter(|e| !e.is_empty())
}

/// A labelled grid row with a text field and its problems below it.
fn row(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut String,
    field: &str,
    errors: &[FieldIssue],
    warnings: &[FieldIssue],
    editable: bool,
) -> egui::Response {
    ui.label(RichText::new(label).small());
    let response = cell(ui, value, field, errors, warnings, 160.0, editable);
    ui.end_row();
    response
}

/// A checkbox with this field's errors (red) and warnings (yellow) under it.
fn flag(ui: &mut egui::Ui, value: &mut bool, field: &str, errors: &[FieldIssue], warnings: &[FieldIssue]) -> egui::Response {
    ui.vertical(|ui| {
        let response = ui.checkbox(value, "");
        for e in errors.iter().filter(|e| e.field == field) {
            ui.label(RichText::new(&e.message).small().color(theme::current().danger));
        }
        if warnings.iter().any(|w| w.field == field) {
            // The message is in the virtual cell's box below (it is long, and the same per robot).
            ui.label(RichText::new("⚠").small().color(theme::current().warn));
        }
        response
    })
    .inner
}

/// A text field of `width` with this field's errors (red) and warnings (yellow) under it.
fn cell(
    ui: &mut egui::Ui,
    value: &mut String,
    field: &str,
    errors: &[FieldIssue],
    warnings: &[FieldIssue],
    width: f32,
    editable: bool,
) -> egui::Response {
    ui.vertical(|ui| {
        let mine: Vec<&FieldIssue> = errors.iter().filter(|e| e.field == field).collect();
        let warned: Vec<&FieldIssue> = warnings.iter().filter(|w| w.field == field).collect();
        let mut edit = egui::TextEdit::singleline(value).desired_width(width).font(egui::TextStyle::Monospace);
        if !mine.is_empty() {
            edit = edit.text_color(theme::current().danger);
        }
        let response = ui.add_enabled(editable, edit);
        for e in mine {
            ui.label(RichText::new(&e.message).small().color(theme::current().danger));
        }
        for w in warned {
            ui.label(RichText::new(format!("⚠ {}", w.message)).small().color(theme::current().warn));
        }
        response
    })
    .inner
}

fn plan_stem(path: &str) -> &str {
    let name = path.rsplit('/').next().unwrap_or(path);
    name.strip_suffix(".json").unwrap_or(name)
}

fn short(err: &str) -> String {
    let line = err.lines().next().unwrap_or(err);
    if line.chars().count() > 140 { format!("{}…", line.chars().take(140).collect::<String>()) } else { line.to_owned() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_client::protocol::hardware::{KukaVcController, RobotHardware};

    #[test]
    fn rejections_are_read_back() {
        let err = r#"422 Unprocessable Entity: {"errors":[{"field":"robots.robot1.ip","message":"m"}]}"#;
        assert_eq!(rejected_errors(err).unwrap()[0].field, "robots.robot1.ip");
        assert_eq!(rejected_errors("500 Internal Server Error: boom"), None);
        assert_eq!(rejected_errors("422 Unprocessable Entity: plan2cell hardware: unreadable"), None);
    }

    #[test]
    fn a_save_reply_for_another_cell_or_plan_is_dropped() {
        let mut ui = HardwareUi::new(Api::new("http://localhost:0"));
        let b = Key { cell: "b".into(), plan: Some("plans/b.json".into()) };
        ui.key = Some(b.clone());
        ui.asked = Some((b.clone(), Instant::now()));
        let view = |cell: &str, plan: &str| HardwareView { cell: cell.into(), plan: plan.into(), ..Default::default() };
        ui.receive(Msg::View { key: b.clone(), view: Box::new(Ok(view("b", "plans/b.json"))) });
        ui.saving = true;
        let a = Some(Key { cell: "a".into(), plan: Some("plans/a.json".into()) });
        ui.receive(Msg::Saved { key: a, result: Box::new(Ok(view("a", "plans/a.json"))) });
        assert!(!ui.saving);
        assert_eq!(ui.loaded().unwrap().cell, "b", "B's view stays");
        assert!(ui.save_result.is_none() && !ui.take_saved());
        assert!(ui.asked.is_none(), "B is read again");
        ui.saving = true;
        ui.receive(Msg::Saved { key: Some(b), result: Box::new(Ok(view("b", "plans/b.json"))) });
        assert!(ui.save_result.as_ref().is_some_and(Result::is_ok));
        assert!(ui.take_saved() && !ui.take_saved(), "the exports are read again once");
    }

    /// scene_fit: saved with plan2cell's real-hardware defaults on a virtual cell.
    #[test]
    fn the_virtual_cells_values_in_one_click() {
        let robot = |ns: &str, ip: &str| RobotHardware {
            ns: ns.into(),
            tag: "R".into(),
            station: format!("kuka-{ns}"),
            device_name: format!("kuka_{ns}"),
            ip: ip.into(),
            ..Default::default()
        };
        let config = HardwareConfig {
            network: factoryos_client::protocol::hardware::NetworkConfig {
                head_ip: "192.168.100.10".into(),
                netmask: "255.255.255.0".into(),
                gateway: "0.0.0.0".into(),
                controller_station: "controller".into(),
            },
            robots: vec![robot("robot1", "192.168.100.11"), robot("robot2", "192.168.100.12")],
            kuka_vc: Vec::new(),
        };
        let mut view = HardwareView { config: config.clone(), source: HardwareSource::Saved, ..Default::default() };
        assert_eq!(virtual_cell_fix(&view, &config), None, "real hardware: no button");
        view.virtual_cell_robots = vec!["robot1".into(), "robot2".into()];
        assert!(!config.virtual_cell_warnings(&view.virtual_cell_robots).is_empty());
        let fixed = virtual_cell_fix(&view, &config).expect("the button");
        assert_eq!(fixed.network.gateway, "192.168.100.1");
        assert!(fixed.robots.iter().all(|r| r.external && r.safety_bypass && !r.safety_module));
        assert!(fixed.virtual_cell_warnings(&view.virtual_cell_robots).is_empty());
        assert_eq!(virtual_cell_fix(&view, &fixed), None, "nothing left to fix");
        assert!(shown_fields(&config).contains(&"robots.robot1.safetyModule".to_owned()));
    }

    #[test]
    fn changed_fields_name_what_moved() {
        let a = HardwareConfig {
            robots: vec![RobotHardware { ns: "robot1".into(), ip: "1".into(), ..Default::default() }],
            kuka_vc: vec![KukaVcController { robot_ns: "robot1".into(), ..Default::default() }],
            ..Default::default()
        };
        let mut b = a.clone();
        b.network.head_ip = "x".into();
        b.robots[0].ip = "2".into();
        assert_eq!(changed_fields(&a, &b), ["network.headIp", "robots.robot1"]);
        assert_eq!(plan_stem("agentic_planner/plans/bt_1.json"), "bt_1");
    }
}
