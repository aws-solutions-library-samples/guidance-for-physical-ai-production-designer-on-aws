//! Robo Dresser panels. The dressing itself (catalogue scan, validation, collision hulls, merge)
//! runs in factoryos-backend (`services/dresser.rs`); previews come from there as renderer-neutral
//! scenes (`factoryos/scene/dresser`), and this draws the catalogue and inspector.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use factoryos_client::protocol::dresser::{
    AssetKind as Kind, DressRequest, DresserState, HULLS, Joint, JointUpdate, OpenRequest, PreviewRequest, ViewRequest,
};
use factoryos_client::protocol::{api, keys};
use factoryos_client::{Api, Bus};
use crate::theme;
use crate::rr::egui::{self, RichText};
use crate::rr::re_log;
use crate::rr::re_log_types::StoreId;

use crate::backend::{Backend, SharedBackend};
use crate::dresser_tcp::TcpEditor;
use crate::scene3d::RouteRequest;
use crate::scene3d::Scene3d;

/// The previewed URDF's node in the backend's dresser scene.
const PREVIEW_ROBOT_NODE: &str = "robot";

pub struct Dresser {
    api: Api,
    bus: Bus,
    /// The backend's dresser state, as last published.
    state: Arc<Mutex<DresserState>>,
    pub stream: Scene3d,
    /// The catalogue last opened.
    opened: Option<String>,

    robot: Option<String>,
    tool: Option<String>,
    selected: Option<(Kind, String)>,
    hull: usize,
    orient: [u16; 3],

    /// Slider values for the current preview's joints, and which preview they belong to.
    joints: Vec<Joint>,
    joints_for: Option<String>,
    /// When the last dress outcome arrived (by its `seq`), for the "new URDF written" note.
    dress_seen: Option<(u64, Instant)>,

    /// The selected tool's TCP.
    tcp: TcpEditor,
    /// For the active cell's name.
    shared: SharedBackend,
    /// "Publish to shared catalogue" of the dressed pair (catalogue admins, S3).
    publish: crate::shared_publish::SharedPublish,
}

impl Dresser {
    pub fn new(backend: &Backend) -> Self {
        let state = Arc::new(Mutex::new(DresserState::default()));
        let sink = state.clone();
        let repaint = backend.repaint_handle();
        // Also queried now and after a re-join: it is only published when it changes.
        let subscribed = backend.bus.follow(keys::DRESSER_STATE, move |payload| {
            if let Ok(update) = serde_json::from_slice(&payload) {
                *sink.lock().unwrap() = update;
                repaint();
            }
        });
        if let Err(err) = subscribed {
            re_log::warn!("Robo Dresser: {err}");
        }
        Self {
            publish: crate::shared_publish::SharedPublish::new(backend.api.clone()),
            api: backend.api.clone(),
            bus: backend.bus.clone(),
            state,
            stream: Scene3d::for_backend(backend, keys::SCENE_DRESSER, "robo_dresser"),
            opened: None,
            robot: None,
            tool: None,
            selected: None,
            hull: 1,
            orient: [0, 0, 0],
            joints: Vec::new(),
            joints_for: None,
            dress_seen: None,
            tcp: TcpEditor::new(backend.api.clone()),
            shared: backend.shared.clone(),
        }
    }

    /// The backend restarted: it has not scanned the catalogue, nor a 3D scene for the dresser.
    /// Open it again if it was open here.
    pub fn backend_restarted(&mut self) {
        if let Some(catalogue_root) = self.opened.take() {
            self.open(&catalogue_root);
            self.stream.resync();
        }
    }

    /// Have the backend (re)scan the catalogue when it changed or has never been scanned.
    pub fn open(&mut self, catalogue_root: &str) {
        if self.opened.as_deref() != Some(catalogue_root) {
            self.opened = Some(catalogue_root.to_owned());
            self.post(api::DRESSER_OPEN, &OpenRequest { rescan: false });
        }
    }

    /// Run one action given as `preview:<robot|tool|dressed>:<name>` or `dress:<robot>:<tool>`
    /// (from `FACTORY_OS_DRESSER`, for demos and testing).
    pub fn run_command(&mut self, command: &str) {
        let parts: Vec<&str> = command.split(':').collect();
        match parts.as_slice() {
            ["preview", kind, name] => {
                let kind = match *kind {
                    "robot" => Kind::Robot,
                    "tool" => Kind::Tool,
                    _ => Kind::Dressed,
                };
                self.load_preview(kind, name);
                self.selected = Some((kind, (*name).to_owned()));
            }
            ["dress", robot, tool] => {
                self.robot = Some((*robot).to_owned());
                self.tool = Some((*tool).to_owned());
                self.dress();
            }
            _ => re_log::warn!("Unknown FACTORY_OS_DRESSER command: {command}"),
        }
    }

    /// The recording currently previewed, if any.
    pub fn preview_recording(&self) -> Option<StoreId> {
        self.stream.current()
    }

    /// Navigation to perform after a new preview was loaded.
    pub fn take_route_request(&mut self) -> Option<RouteRequest> {
        self.stream.take_route_request()
    }

    /// POST `body` to `path`; the resulting state arrives on `factoryos/dresser/state`.
    fn post(&self, path: &'static str, body: &impl serde::Serialize) {
        let state = self.state.clone();
        self.api.post_json(path, body, move |response: Result<DresserState, String>| {
            if let Err(err) = response {
                state.lock().unwrap().status = format!("Failed: {err}");
            }
        });
    }

    fn rescan(&self) {
        self.post(api::DRESSER_OPEN, &OpenRequest { rescan: true });
    }

    fn load_preview(&self, kind: Kind, name: &str) {
        self.post(api::DRESSER_PREVIEW, &PreviewRequest { kind, name: name.to_owned() });
    }

    fn dress(&self) {
        let (Some(robot), Some(tool)) = (self.robot.clone(), self.tool.clone()) else {
            return;
        };
        self.post(
            api::DRESSER_DRESS,
            &DressRequest {
                robot,
                tool,
                hull: HULLS[self.hull].to_owned(),
                orient: self.orient,
                spin_deg: 0.0,
                attach_to: None,
            },
        );
    }

    fn set_view(&self, show_collision: bool, show_reach: bool) {
        self.post(api::DRESSER_VIEW, &ViewRequest { show_collision, show_reach });
    }

    /// Keep the sliders in step with the backend's preview.
    fn sync_joints(&mut self, state: &DresserState) {
        let label = state.preview.as_ref().map(|p| p.label.clone());
        if label != self.joints_for {
            self.joints = state.preview.as_ref().map(|p| p.joints.clone()).unwrap_or_default();
            self.joints_for = label;
        }
        if let Some(dress) = &state.dress
            && self.dress_seen.is_none_or(|(seq, _)| seq != dress.seq)
        {
            self.dress_seen = Some((dress.seq, Instant::now()));
        }
    }

    // ------------------------------------------------------------------------
    // UI

    /// Header, view toggles and the inspector, shown as the right-hand panel.
    /// The pane's main action, pinned to its bottom ([`theme::action_row`]): Dress the selected pair.
    pub fn actions_ui(&mut self, ui: &mut egui::Ui) {
        let busy = self.state.lock().unwrap().busy.is_some();
        let can_dress = self.robot.is_some() && self.tool.is_some() && !busy;
        theme::action_buttons(ui, |ui| {
            if ui
                .add_enabled(can_dress, theme::primary(ui, "Dress"))
                .on_disabled_hover_text("Select a robot and a tool first.")
                .clicked()
            {
                self.dress();
            }
        });
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let state = self.state.lock().unwrap().clone();
        self.sync_joints(&state);
        let (busy, status) = (state.busy.clone(), state.status.clone());

        ui.horizontal_wrapped(|ui| {
            if busy.is_some() {
                ui.spinner();
            }
            ui.small(RichText::new(status).weak());
        });
        // The debug views: Advanced only.
        crate::ui_mode::advanced(ui, |ui| ui.horizontal(|ui| {
            let (mut show_collision, mut show_reach) = (state.show_collision, state.show_reach);
            if ui.checkbox(&mut show_collision, "Collision hulls").changed() {
                self.set_view(show_collision, show_reach);
            }
            let has_reach = state.preview.as_ref().is_some_and(|p| p.has_reach);
            if ui
                .add_enabled(has_reach, egui::Checkbox::new(&mut show_reach, "Reach envelope"))
                .on_disabled_hover_text("Only dressed robots have a reach envelope.")
                .changed()
            {
                self.set_view(show_collision, show_reach);
            }
        }));
        ui.separator();

        self.inspector_ui(ui, &state);
        // The tool's TCP editor: Advanced only.
        crate::ui_mode::advanced(ui, |ui| {
            ui.separator();
            self.tcp_ui(ui, &state);
        });
    }

    /// The selected tool's TCP: edit, snap, save. The backend draws edits into the tool's preview.
    fn tcp_ui(&mut self, ui: &mut egui::Ui, state: &DresserState) {
        heading(ui, "Tool TCP");
        self.tcp.set_tool(self.tool.as_deref());
        let tool_shown = self.tool.as_ref().is_some_and(|tool| {
            self.selected.as_ref() == Some(&(Kind::Tool, tool.clone()))
                && state.preview.as_ref().is_some_and(|p| p.label == format!("tool {tool}"))
        });
        let note = if tool_shown {
            "The 3D view follows the edits; Save TCP writes them."
        } else {
            "Click the tool in the catalogue to see its TCP and the edits in the 3D view."
        };
        let active_cell = self.shared.lock().unwrap().cells.active.clone();
        self.tcp.ui(ui, note, &active_cell);
    }

    /// The catalogue (robots, tools, dressed robots), shown as a pane in the left panel.
    pub fn catalogue_ui(&mut self, ui: &mut egui::Ui) {
        // The backend rescans the catalogue on a change and publishes its state: just say so.
        crate::catalogue_events::notice_ui(ui);
        let s = self.state.lock().unwrap().clone();
        let busy = s.busy.is_some();
        ui.horizontal(|ui| {
            crate::ui_mode::advanced(ui, |ui| {
                ui.small(RichText::new(&s.root).weak())
                    .on_hover_text("Set in Configuration → Directories → Catalogue directory");
            });
            if ui.small_button("↻").on_hover_text("Rescan catalogue").clicked() {
                self.rescan();
            }
        });
        if let Some(err) = &s.catalogue_error {
            ui.colored_label(theme::current().danger, err);
        }

        let mut clicked = None;
        {
            for (kind, title, entries, empty) in [
                (Kind::Robot, "Robots", &s.robots, "No robots found."),
                (Kind::Tool, "Tools", &s.tools, "No tools found."),
                (Kind::Dressed, "Dressed", &s.dressed, "None yet — dress a pair."),
            ] {
                egui::CollapsingHeader::new(title).default_open(true).show(ui, |ui| {
                    if entries.is_empty() {
                        ui.weak(empty);
                    }
                    for e in entries {
                        let selected = self.selected.as_ref() == Some(&(kind, e.name.clone()));
                        ui.horizontal(|ui| {
                            let (color, tip) = match &e.problem {
                                None => (theme::current().ok, "valid".to_owned()),
                                Some(problem) if problem.error => (theme::current().danger, format!("error: {}", problem.message)),
                                Some(problem) => (theme::current().warn, format!("incomplete: {}", problem.message)),
                            };
                            if kind != Kind::Dressed {
                                ui.label(RichText::new("●").color(color)).on_hover_text(tip);
                            }
                            let label = egui::Button::selectable(selected, RichText::new(&e.name).monospace());
                            if ui.add_enabled(!busy, label).clicked() {
                                clicked = Some((kind, e.name.clone()));
                            }
                        });
                    }
                });
            }
        }

        if let Some((kind, name)) = clicked {
            match kind {
                Kind::Robot => self.robot = Some(name.clone()),
                Kind::Tool => self.tool = Some(name.clone()),
                Kind::Dressed => {}
            }
            self.load_preview(kind, &name);
            self.selected = Some((kind, name));
        }
    }

    fn inspector_ui(&mut self, ui: &mut egui::Ui, state: &DresserState) {
        heading(ui, "Inspector");

        sub_heading(ui, "Selected pair");
        ui.monospace(format!("Robot: {}", self.robot.as_deref().unwrap_or("—")));
        ui.monospace(format!("Tool:  {}", self.tool.as_deref().unwrap_or("—")));

        // The dress options: Advanced only (Simple dresses with them as set: auto-orient by default).
        crate::ui_mode::advanced(ui, |ui| {
        sub_heading(ui, "Dress options");
        ui.horizontal(|ui| {
            ui.label("hull");
            egui::ComboBox::from_id_salt("dresser_hull")
                .selected_text(HULLS[self.hull])
                .show_ui(ui, |ui| {
                    for (i, hull) in HULLS.iter().enumerate() {
                        ui.selectable_value(&mut self.hull, i, *hull);
                    }
                });
        });
        ui.small(RichText::new("TOOL ORIENTATION (90° STEPS)").weak());
        for (axis, label) in ["X", "Y", "Z"].iter().enumerate() {
            ui.horizontal(|ui| {
                ui.monospace(*label);
                if ui.small_button("−90").clicked() {
                    self.orient[axis] = (self.orient[axis] + 270) % 360;
                }
                ui.monospace(format!("{:>4}°", self.orient[axis]));
                if ui.small_button("+90").clicked() {
                    self.orient[axis] = (self.orient[axis] + 90) % 360;
                }
            });
        }
        if self.orient == [0, 0, 0] {
            ui.small(RichText::new("0/0/0 = auto-orient the tool").weak());
        }
        });
        // Dress is in the pane's action row (`actions_ui`).
        sub_heading(ui, "Output");
        let finished = self.dress_seen.map(|(_, at)| at);
        match &state.dress {
            Some(outcome) if outcome.output_rel.is_some() => {
                let rel = outcome.output_rel.as_deref().unwrap_or_default();
                if finished.is_some_and(|at| at.elapsed() < Duration::from_secs(10)) {
                    ui.colored_label(theme::current().ok, "✓ new URDF written");
                    ui.ctx().request_repaint_after(Duration::from_secs(1));
                }
                crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(rel).monospace()));
                // `urdf-dressed-robots/<pair>/<pair>.urdf`: the pair's folder, from the user's catalogue additions.
                if let Some((folder, _)) = rel.rsplit_once('/') {
                    let (from, to) = (format!("catalogue/{folder}/"), format!("{folder}/"));
                    self.publish.button_ui(ui, true, &from, &to);
                    self.publish.status_ui(ui);
                }
            }
            Some(outcome) if outcome.error.is_some() => {
                ui.colored_label(theme::current().danger, outcome.error.as_deref().unwrap_or_default());
            }
            _ => {
                ui.weak("—");
            }
        }

        if !crate::ui_mode::is_advanced(ui) {
            // Simple: only the errors (they block the dress); warnings and the joints are Advanced.
            for diagnostic in state.dress.iter().flat_map(|o| &o.diagnostics).filter(|d| d.error) {
                ui.label(RichText::new(&diagnostic.text).small().color(theme::current().danger));
            }
            return;
        }
        egui::CollapsingHeader::new("Diagnostics").default_open(true).show(ui, |ui| {
            ui.small(RichText::new("URDF validation of the robot + tool. Errors block the dress; warnings are advisory.").weak());
            match &state.dress {
                Some(outcome) if !outcome.diagnostics.is_empty() => {
                    for diagnostic in &outcome.diagnostics {
                        let color = if diagnostic.error { theme::current().danger } else { theme::current().warn };
                        ui.label(RichText::new(&diagnostic.text).small().color(color));
                    }
                }
                Some(_) => {
                    ui.small(RichText::new("No issues.").color(theme::current().ok));
                }
                None => {
                    ui.weak("Dress a pair to validate it.");
                }
            }
        });

        egui::CollapsingHeader::new("Joints").default_open(true).show(ui, |ui| {
            let Some(preview) = &state.preview else {
                ui.weak("Select or dress a robot to see its joints.");
                return;
            };
            if self.joints.is_empty() {
                ui.weak("No articulated joints.");
                return;
            }
            ui.small(RichText::new(&preview.label).weak());
            for joint in &mut self.joints {
                ui.small(joint.name.as_str());
                let slider = egui::Slider::new(&mut joint.value, joint.lower..=joint.upper).max_decimals(3);
                if ui.add(slider).changed() {
                    let update = JointUpdate {
                        name: joint.name.clone(),
                        value: joint.value,
                    };
                    if let Ok(payload) = serde_json::to_vec(&update) {
                        self.bus.put(keys::DRESSER_JOINT, payload);
                    }
                    // Moved at once; the backend's echo of it changes nothing.
                    self.stream.set_local_joints(PREVIEW_ROBOT_NODE, &BTreeMap::from([(update.name, update.value)]));
                }
            }
        });
    }
}

fn heading(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title.to_uppercase()).small().strong());
}

fn sub_heading(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.label(RichText::new(title.to_uppercase()).small().weak());
}
