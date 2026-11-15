//! The Robo Dresser's TCP editor: the selected tool's TCP (relative to the tool's root link),
//! edited in mm and degrees, snapped to the tool tip, previewed live and saved into the tool URDF.
//!
//! [`TcpEdit`] is the panel's state and [`PreviewThrottle`] paces the live preview, both free of UI
//! and I/O so they can be tested; [`TcpEditor`] wires them to the backend
//! (`/api/v1/dresser/tool_tcp`, `…/tool_tcp/preview`, `…/tool_tip`) and draws the section. The 3D
//! view is the backend's: it draws the saved TCP, and the unsaved one this sends as a preview.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::dresser::{SetToolTcp, ToolTcp, ToolTcpSaved, ToolTipEstimate};

use web_time::{Duration, Instant};

use crate::theme;
use crate::rr::egui::{self, Color32, RichText};

/// The live preview is sent at most this often while dragging (nudges go at once).
const PREVIEW_INTERVAL: Duration = Duration::from_millis(150);

/// Differences below this (in m, resp. degrees) are not an edit.
const EPS_M: f64 = 1e-7;
const EPS_DEG: f64 = 1e-6;

pub fn m_to_mm(m: f64) -> f64 {
    m * 1000.0
}

pub fn mm_to_m(mm: f64) -> f64 {
    mm / 1000.0
}

/// `deg` in (-180, 180].
pub fn wrap_deg(deg: f64) -> f64 {
    let wrapped = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped <= -180.0 { wrapped + 360.0 } else { wrapped }
}

/// The TCP being edited for one tool: the values last loaded or saved, and the edited ones.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TcpEdit {
    pub tool: String,
    /// As last loaded from (or saved to) the backend.
    pub base: ToolTcp,
    /// Edited offset from the tool's root link, in mm.
    pub xyz_mm: [f64; 3],
    /// Edited roll, pitch, yaw in degrees.
    pub rpy_deg: [f64; 3],
}

impl TcpEdit {
    pub fn from_loaded(tcp: ToolTcp) -> Self {
        Self {
            tool: tcp.tool.clone(),
            xyz_mm: tcp.xyz_m.map(m_to_mm),
            rpy_deg: tcp.rpy_deg,
            base: tcp,
        }
    }

    /// Discard the edits.
    pub fn reset(&mut self) {
        *self = Self::from_loaded(self.base.clone());
    }

    pub fn nudge_mm(&mut self, axis: usize, delta_mm: f64) {
        self.xyz_mm[axis] += delta_mm;
    }

    pub fn nudge_deg(&mut self, axis: usize, delta_deg: f64) {
        self.rpy_deg[axis] = wrap_deg(self.rpy_deg[axis] + delta_deg);
    }

    /// Move the TCP onto the tool tip; the orientation stays.
    pub fn snap_to(&mut self, tip: &ToolTipEstimate) {
        self.xyz_mm = tip.xyz_m.map(m_to_mm);
    }

    pub fn xyz_m(&self) -> [f64; 3] {
        self.xyz_mm.map(mm_to_m)
    }

    /// Whether the edited TCP differs from the loaded one.
    pub fn dirty(&self) -> bool {
        let xyz = self.xyz_m();
        (0..3).any(|i| (xyz[i] - self.base.xyz_m[i]).abs() > EPS_M || (self.rpy_deg[i] - self.base.rpy_deg[i]).abs() > EPS_DEG)
    }

    /// Whether the edited TCP is the tool's declared one (else: an estimate, until saved).
    pub fn declared(&self) -> bool {
        self.base.declared && !self.dirty()
    }

    pub fn request(&self) -> SetToolTcp {
        SetToolTcp {
            tool: self.tool.clone(),
            xyz_m: self.xyz_m(),
            rpy_deg: self.rpy_deg,
        }
    }
}

/// What [`PreviewThrottle::poll`] asks for.
#[derive(Clone, Debug, PartialEq)]
pub enum PreviewPoll {
    /// The 3D view shows the edited TCP already.
    Idle,
    /// Send it now.
    Send,
    /// Sent recently: ask again after this long.
    Wait(Duration),
}

/// Paces the live preview: the edited TCP goes to the backend when it changed, at most every
/// [`PREVIEW_INTERVAL`] while dragging (the last value always follows), nudges at once.
#[derive(Clone, Debug, Default)]
pub struct PreviewThrottle {
    /// What the 3D view shows: last sent, or loaded/saved (the backend draws that itself).
    shown: Option<SetToolTcp>,
    sent_at: Option<Instant>,
}

impl PreviewThrottle {
    /// The backend draws `tcp` already (just loaded or saved).
    pub fn mark_shown(&mut self, tcp: SetToolTcp) {
        self.shown = Some(tcp);
    }

    /// Whether to send `current` now. `immediate`: a nudge, sent without waiting.
    pub fn poll(&mut self, now: Instant, current: &SetToolTcp, immediate: bool) -> PreviewPoll {
        if self.shown.as_ref() == Some(current) {
            return PreviewPoll::Idle;
        }
        let since = self.sent_at.map(|at| now.saturating_duration_since(at));
        match since {
            Some(since) if !immediate && since < PREVIEW_INTERVAL => PreviewPoll::Wait(PREVIEW_INTERVAL - since),
            _ => {
                self.shown = Some(current.clone());
                self.sent_at = Some(now);
                PreviewPoll::Send
            }
        }
    }
}

/// Backend replies, filled in by the HTTP callbacks and taken by the UI.
#[derive(Default)]
struct Inbox {
    loaded: Option<ToolTcp>,
    tip: Option<ToolTipEstimate>,
    saved: Option<ToolTcpSaved>,
    error: Option<String>,
    /// Requests in flight.
    pending: usize,
}

/// The "Tool TCP" section of the Robo Dresser inspector.
pub struct TcpEditor {
    api: Api,
    inbox: Arc<Mutex<Inbox>>,
    /// The tool the editor is for (the Dresser's selected tool).
    tool: Option<String>,
    pub edit: Option<TcpEdit>,
    /// The outcome of the last save, for the tool it was for.
    saved: Option<ToolTcpSaved>,
    error: Option<String>,
    preview: PreviewThrottle,
}

impl TcpEditor {
    pub fn new(api: Api) -> Self {
        Self {
            api,
            inbox: Arc::default(),
            tool: None,
            edit: None,
            saved: None,
            error: None,
            preview: PreviewThrottle::default(),
        }
    }

    /// Follow the Dresser's selected tool: a new tool's TCP is loaded (edits to the old are dropped).
    pub fn set_tool(&mut self, tool: Option<&str>) {
        if self.tool.as_deref() == tool {
            return;
        }
        self.tool = tool.map(str::to_owned);
        self.edit = None;
        self.saved = None;
        self.error = None;
        self.preview = PreviewThrottle::default();
        if self.tool.is_some() {
            self.load();
        }
    }

    fn query(tool: &str) -> String {
        format!("?tool={}", encode_query_value(tool))
    }

    fn load(&self) {
        let Some(tool) = &self.tool else { return };
        let inbox = self.inbox.clone();
        inbox.lock().unwrap().pending += 1;
        self.api.get(&format!("{}{}", api::DRESSER_TOOL_TCP, Self::query(tool)), move |reply: Result<ToolTcp, String>| {
            let mut inbox = inbox.lock().unwrap();
            inbox.pending -= 1;
            match reply {
                Ok(tcp) => inbox.loaded = Some(tcp),
                Err(err) => inbox.error = Some(format!("Loading the TCP failed: {err}")),
            }
        });
    }

    fn fetch_tip(&self) {
        let Some(tool) = &self.tool else { return };
        let inbox = self.inbox.clone();
        inbox.lock().unwrap().pending += 1;
        self.api.get(&format!("{}{}", api::DRESSER_TOOL_TIP, Self::query(tool)), move |reply: Result<ToolTipEstimate, String>| {
            let mut inbox = inbox.lock().unwrap();
            inbox.pending -= 1;
            match reply {
                Ok(tip) => inbox.tip = Some(tip),
                Err(err) => inbox.error = Some(format!("Finding the tool tip failed: {err}")),
            }
        });
    }

    fn save(&self) {
        let Some(edit) = &self.edit else { return };
        let inbox = self.inbox.clone();
        inbox.lock().unwrap().pending += 1;
        self.api.post_json(api::DRESSER_TOOL_TCP, &edit.request(), move |reply: Result<ToolTcpSaved, String>| {
            let mut inbox = inbox.lock().unwrap();
            inbox.pending -= 1;
            match reply {
                Ok(saved) => inbox.saved = Some(saved),
                Err(err) => inbox.error = Some(format!("Saving the TCP failed: {err}")),
            }
        });
    }

    /// Have the backend draw the edited (unsaved) TCP in the 3D view.
    fn send_preview(&self, request: &SetToolTcp) {
        let inbox = self.inbox.clone();
        self.api.post_json(api::DRESSER_TOOL_TCP_PREVIEW, request, move |reply: Result<(), String>| {
            if let Err(err) = reply {
                inbox.lock().unwrap().error = Some(format!("Previewing the TCP failed: {err}"));
            }
        });
    }

    /// Send the live preview when due; `nudged`: a nudge button was clicked this frame.
    fn update_preview(&mut self, ctx: &egui::Context, nudged: bool) {
        let Some(edit) = &self.edit else { return };
        let request = edit.request();
        match self.preview.poll(Instant::now(), &request, nudged) {
            PreviewPoll::Idle => {}
            PreviewPoll::Send => self.send_preview(&request),
            PreviewPoll::Wait(after) => ctx.request_repaint_after(after),
        }
    }

    /// Apply the backend's replies that arrived. Returns whether requests are still in flight.
    fn take_replies(&mut self) -> bool {
        let mut inbox = self.inbox.lock().unwrap();
        let current = |tool: &str| self.tool.as_deref() == Some(tool);
        if let Some(tcp) = inbox.loaded.take()
            && current(&tcp.tool)
        {
            let edit = TcpEdit::from_loaded(tcp);
            // A reset (re-load) brings an edited preview back to the saved TCP: that one is sent.
            if self.edit.is_none() {
                self.preview.mark_shown(edit.request());
            }
            self.edit = Some(edit);
        }
        if let Some(tip) = inbox.tip.take()
            && let Some(edit) = &mut self.edit
        {
            edit.snap_to(&tip);
        }
        if let Some(saved) = inbox.saved.take()
            && current(&saved.tcp.tool)
        {
            let edit = TcpEdit::from_loaded(saved.tcp.clone());
            self.preview.mark_shown(edit.request()); // the backend redraws the saved TCP
            self.edit = Some(edit);
            self.saved = Some(saved);
        }
        if let Some(err) = inbox.error.take() {
            self.error = Some(err);
        }
        inbox.pending > 0
    }

    /// The section's UI. `preview_note`: how the edited TCP is shown in the 3D view; `active_cell`:
    /// the active cell's folder name (after a save the backend prepares that one itself).
    pub fn ui(&mut self, ui: &mut egui::Ui, preview_note: &str, active_cell: &str) {
        let busy = self.take_replies();
        if busy {
            ui.ctx().request_repaint_after(web_time::Duration::from_millis(100));
        }
        let Some(tool) = self.tool.clone() else {
            ui.weak("Select a tool to see and edit its TCP.");
            return;
        };
        ui.horizontal(|ui| {
            ui.monospace(&tool);
            if busy {
                ui.spinner();
            }
        });
        if let Some(err) = &self.error {
            ui.colored_label(theme::current().danger, err);
        }
        let Some(edit) = &mut self.edit else {
            if !busy {
                ui.weak("No TCP loaded.");
            }
            return;
        };

        if edit.declared() {
            ui.colored_label(theme::current().ok, "● declared")
                .on_hover_text("The tool URDF declares this TCP (a `tcp` link on a fixed joint from its root).");
        } else if edit.dirty() {
            ui.colored_label(theme::current().warn, "● edited, not saved");
        } else {
            ui.colored_label(theme::current().warn, "● estimate")
                .on_hover_text("The tool declares no TCP: this is estimated from its geometry. Save it to declare it.");
        }
        ui.small(RichText::new("Relative to the tool's root link (the flange interface).").weak());

        let (mut changed, mut nudged) = (false, false);
        egui::Grid::new("dresser_tcp_grid").num_columns(2).spacing([4.0, 2.0]).show(ui, |ui| {
            for (axis, label) in ["x", "y", "z"].iter().enumerate() {
                ui.monospace(format!("{label} mm"));
                ui.horizontal(|ui| {
                    for (text, delta) in [("−10", -10.0), ("−1", -1.0)] {
                        if ui.small_button(text).clicked() {
                            edit.nudge_mm(axis, delta);
                            nudged = true;
                        }
                    }
                    changed |= ui
                        .add(egui::DragValue::new(&mut edit.xyz_mm[axis]).speed(0.5).fixed_decimals(2))
                        .changed();
                    for (text, delta) in [("+1", 1.0), ("+10", 10.0)] {
                        if ui.small_button(text).clicked() {
                            edit.nudge_mm(axis, delta);
                            nudged = true;
                        }
                    }
                });
                ui.end_row();
            }
            for (axis, label) in ["roll", "pitch", "yaw"].iter().enumerate() {
                ui.monospace(format!("{label} °"));
                ui.horizontal(|ui| {
                    if ui.small_button("−5°").clicked() {
                        edit.nudge_deg(axis, -5.0);
                        nudged = true;
                    }
                    let drag = egui::DragValue::new(&mut edit.rpy_deg[axis]).speed(0.5).fixed_decimals(1).range(-180.0..=180.0);
                    changed |= ui.add(drag).changed();
                    if ui.small_button("+5°").clicked() {
                        edit.nudge_deg(axis, 5.0);
                        nudged = true;
                    }
                });
                ui.end_row();
            }
        });
        if changed || nudged {
            self.saved = None;
        }
        ui.small(RichText::new(preview_note).weak());

        let dirty = edit.dirty();
        let (mut snap, mut reset) = (false, false);
        ui.horizontal(|ui| {
            snap = ui
                .add_enabled(!busy, egui::Button::new("Snap to tool tip"))
                .on_hover_text("Move the TCP to the farthest point of the tool along its axis (orientation stays).")
                .clicked();
            reset = ui
                .add_enabled(!busy, egui::Button::new("Reset"))
                .on_hover_text("Reload the TCP from the tool URDF, dropping the edits.")
                .clicked();
        });
        let save_button = egui::Button::new(RichText::new("Save TCP").color(Color32::WHITE))
            .fill(theme::current().accent)
            .min_size(egui::vec2(ui.available_width(), 24.0));
        // An estimate can be saved unchanged: that declares it.
        let save = ui
            .add_enabled(!busy && (dirty || !edit.base.declared), save_button)
            .on_hover_text("Write the TCP into the tool URDF and the robots dressed with it.")
            .on_disabled_hover_text("Nothing to save: this is the declared TCP.")
            .clicked();

        if let Some(saved) = &self.saved {
            ui.colored_label(theme::current().ok, "✓ TCP saved");
            ui.small(format!("updated robots: {}", list_or_none(&saved.updated_robots)));
            let (being_prepared, needing) = split_active(&saved.cells_needing_prepare, active_cell);
            if let Some(cell) = being_prepared {
                ui.small(format!("being prepared: {cell} (the active cell)"));
            }
            let text = RichText::new(format!("cells needing Prepare Cell: {}", list_or_none(&needing))).small();
            ui.label(if needing.is_empty() { text } else { text.color(theme::current().warn) })
                .on_hover_text("Their planner model still has the old TCP: run Prepare Cell for each.");
        }

        if snap {
            self.error = None;
            self.fetch_tip();
        }
        if reset {
            self.error = None;
            self.saved = None;
            if let Some(edit) = &mut self.edit {
                edit.reset(); // at once; the re-load then picks up changes made elsewhere
            }
            self.load();
        }
        self.update_preview(ui.ctx(), nudged);
        if save {
            self.error = None;
            self.save();
        }
    }
}

/// `cells` split into the active cell (if among them; the backend prepares it again itself) and
/// the others.
pub fn split_active<'a>(cells: &'a [String], active_cell: &str) -> (Option<&'a str>, Vec<String>) {
    let active = cells.iter().find(|c| !active_cell.is_empty() && c.as_str() == active_cell);
    let others = cells.iter().filter(|c| Some(*c) != active).cloned().collect();
    (active.map(String::as_str), others)
}

fn list_or_none(items: &[String]) -> String {
    if items.is_empty() { "none".to_owned() } else { items.join(", ") }
}

/// Percent-encode a query parameter value (tool names are usually plain, but need not be).
fn encode_query_value(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for byte in value.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(byte as char),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn loaded(declared: bool) -> TcpEdit {
        TcpEdit::from_loaded(ToolTcp {
            tool: "gun".to_owned(),
            xyz_m: [0.0, 0.0125, 0.25],
            rpy_deg: [0.0, 90.0, 0.0],
            declared,
        })
    }

    fn assert_close(a: [f64; 3], b: [f64; 3]) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < 1e-9, "{a:?} != {b:?}");
        }
    }

    #[test]
    fn loading_converts_metres_to_millimetres_and_back() {
        let edit = loaded(true);
        assert_close(edit.xyz_mm, [0.0, 12.5, 250.0]);
        assert_close(edit.xyz_m(), [0.0, 0.0125, 0.25]);
        assert_eq!(m_to_mm(mm_to_m(3.7)), 3.7);
        assert!(!edit.dirty());
        assert!(edit.declared());
    }

    #[test]
    fn nudges_move_by_millimetres_and_degrees() {
        let mut edit = loaded(true);
        edit.nudge_mm(2, 10.0);
        edit.nudge_mm(2, 1.0);
        edit.nudge_mm(0, -1.0);
        assert_close(edit.xyz_mm, [-1.0, 12.5, 261.0]);
        assert_close(edit.request().xyz_m, [-0.001, 0.0125, 0.261]);
        edit.nudge_deg(1, 5.0);
        edit.nudge_deg(2, -5.0);
        assert_close(edit.rpy_deg, [0.0, 95.0, -5.0]);
    }

    #[test]
    fn angle_nudges_wrap_into_plus_minus_180() {
        let mut edit = loaded(false);
        edit.rpy_deg[2] = 178.0;
        edit.nudge_deg(2, 5.0);
        assert!((edit.rpy_deg[2] - -177.0).abs() < 1e-9);
        edit.nudge_deg(2, -5.0);
        assert!((edit.rpy_deg[2] - 178.0).abs() < 1e-9);
        assert_eq!(wrap_deg(-180.0), 180.0);
        assert_eq!(wrap_deg(540.0), 180.0);
        assert_eq!(wrap_deg(-190.0), 170.0);
    }

    #[test]
    fn dirty_until_reset_or_back_at_the_loaded_value() {
        let mut edit = loaded(true);
        edit.nudge_mm(1, 1.0);
        assert!(edit.dirty());
        assert!(!edit.declared(), "an edited TCP is not the declared one");
        edit.nudge_mm(1, -1.0);
        assert!(!edit.dirty(), "float round trip of a nudge and back is not an edit");
        edit.nudge_deg(0, 5.0);
        assert!(edit.dirty());
        edit.reset();
        assert_eq!(edit, loaded(true));
    }

    #[test]
    fn snap_moves_the_position_and_keeps_the_orientation() {
        let mut edit = loaded(false);
        edit.snap_to(&ToolTipEstimate { xyz_m: [0.001, -0.002, 0.3105] });
        assert_close(edit.xyz_mm, [1.0, -2.0, 310.5]);
        assert_close(edit.rpy_deg, [0.0, 90.0, 0.0]);
        assert!(edit.dirty());
        assert!(!edit.declared());
        let request = edit.request();
        assert_eq!(request.tool, "gun");
        assert_close(request.xyz_m, [0.001, -0.002, 0.3105]);
    }

    #[test]
    fn an_estimate_is_not_declared_even_unedited() {
        let edit = loaded(false);
        assert!(!edit.dirty());
        assert!(!edit.declared());
    }

    #[test]
    fn the_preview_is_sent_on_change_at_most_every_interval_and_nudges_at_once() {
        let mut edit = loaded(false);
        let mut throttle = PreviewThrottle::default();
        throttle.mark_shown(edit.request());
        let t0 = Instant::now();
        assert_eq!(throttle.poll(t0, &edit.request(), false), PreviewPoll::Idle, "loaded: drawn by the backend");

        edit.xyz_mm[0] += 0.5; // a drag
        assert_eq!(throttle.poll(t0, &edit.request(), false), PreviewPoll::Send);
        assert_eq!(throttle.poll(t0, &edit.request(), false), PreviewPoll::Idle);
        edit.xyz_mm[0] += 0.5;
        let t1 = t0 + Duration::from_millis(50);
        assert_eq!(throttle.poll(t1, &edit.request(), false), PreviewPoll::Wait(Duration::from_millis(100)));
        edit.nudge_mm(0, 1.0);
        assert_eq!(throttle.poll(t1, &edit.request(), true), PreviewPoll::Send, "a nudge goes at once");
        edit.xyz_mm[0] += 0.5;
        let t2 = t1 + PREVIEW_INTERVAL;
        assert_eq!(throttle.poll(t2, &edit.request(), false), PreviewPoll::Send, "the last value always follows");
        assert_eq!(throttle.poll(t2, &edit.request(), false), PreviewPoll::Idle);

        // Reset: the saved TCP is sent back, so the preview returns to it.
        edit.reset();
        let t3 = t2 + PREVIEW_INTERVAL;
        assert_eq!(throttle.poll(t3, &edit.request(), false), PreviewPoll::Send);
    }

    #[test]
    fn the_active_cell_is_being_prepared_the_others_need_it() {
        let cells = vec!["weld_a".to_owned(), "weld_b".to_owned(), "weld_c".to_owned()];
        let (active, others) = split_active(&cells, "weld_b");
        assert_eq!(active, Some("weld_b"));
        assert_eq!(others, ["weld_a", "weld_c"]);
        let (active, others) = split_active(&cells, "");
        assert_eq!((active, others.len()), (None, 3));
        assert_eq!(split_active(&cells, "elsewhere").0, None);
    }

    #[test]
    fn query_values_are_percent_encoded() {
        assert_eq!(encode_query_value("weld_gun-2.v1"), "weld_gun-2.v1");
        assert_eq!(encode_query_value("a b&c"), "a%20b%26c");
    }
}
