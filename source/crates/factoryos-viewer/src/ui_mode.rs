//! Simple / Advanced: how much of each panel is shown.
//!
//! One mode for the whole UI, kept in egui's memory (so every panel reaches it through its `Ui`)
//! and persisted with eframe's storage (a file natively, localStorage on the web). Simple is the
//! default. Simple mode only hides: hidden widgets keep their values (they live in the panels'
//! state, not in the widgets) and whatever they set still applies.

use crate::rr::{eframe, egui};

/// How much of the UI is shown.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UiMode {
    /// Only what the main workflow needs: pick, plan, run, save.
    #[default]
    Simple,
    /// Everything: tuning parameters, diagnostics, raw IDs and paths, expert toggles.
    Advanced,
}

/// Where the choice is kept in eframe's storage.
const STORAGE_KEY: &str = "factoryos.ui_mode";

fn id() -> egui::Id {
    egui::Id::new("factoryos.ui_mode")
}

/// The stored choice; Simple when there is none.
pub fn load(storage: Option<&dyn eframe::Storage>) -> UiMode {
    match storage.and_then(|s| s.get_string(STORAGE_KEY)).as_deref() {
        Some("advanced") => UiMode::Advanced,
        _ => UiMode::Simple,
    }
}

pub fn save(storage: &mut dyn eframe::Storage, ctx: &egui::Context) {
    let name = match get(ctx) {
        UiMode::Simple => "simple",
        UiMode::Advanced => "advanced",
    };
    storage.set_string(STORAGE_KEY, name.to_owned());
}

pub fn get(ctx: &egui::Context) -> UiMode {
    ctx.data(|d| d.get_temp(id())).unwrap_or_default()
}

pub fn set(ctx: &egui::Context, mode: UiMode) {
    ctx.data_mut(|d| d.insert_temp(id(), mode));
}

/// Whether the UI shows everything.
pub fn is_advanced(ui: &egui::Ui) -> bool {
    get(ui.ctx()) == UiMode::Advanced
}

/// `add_contents` in Advanced mode only; `None` in Simple mode.
///
/// Wrap only UI in it: anything the closure computes or applies is skipped in Simple mode.
pub fn advanced<R>(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui) -> R) -> Option<R> {
    is_advanced(ui).then(|| add_contents(ui))
}

/// The top bar's segmented "Simple | Advanced" switch.
pub fn toggle_ui(ui: &mut egui::Ui) {
    let current = get(ui.ctx());
    let visuals = ui.visuals().clone();
    let accent = visuals.selection.bg_fill;
    let accent_fg = visuals.selection.stroke.color;
    let border = visuals.widgets.inactive.bg_stroke.color;
    let segment = |ui: &mut egui::Ui, mode: UiMode, label: &str, tip: &str, corner: egui::CornerRadius| {
        let selected = current == mode;
        let text = egui::RichText::new(label).color(if selected { accent_fg } else { visuals.text_color() });
        let button = egui::Button::new(text)
            .fill(if selected { accent } else { egui::Color32::TRANSPARENT })
            .stroke(egui::Stroke::new(1.0, if selected { accent } else { border }))
            .corner_radius(corner);
        let response = ui.add(button).on_hover_text(tip);
        if response.clicked() && !selected {
            set(ui.ctx(), mode);
        }
    };
    let simple = |ui: &mut egui::Ui| {
        segment(
            ui,
            UiMode::Simple,
            "Simple",
            "Simple: only what the main workflow needs. Hidden settings keep their values.",
            egui::CornerRadius { nw: 8, sw: 8, ne: 0, se: 0 },
        );
    };
    let advanced = |ui: &mut egui::Ui| {
        segment(
            ui,
            UiMode::Advanced,
            "Advanced",
            "Advanced: every option, tuning parameter and diagnostic",
            egui::CornerRadius { nw: 0, sw: 0, ne: 8, se: 8 },
        );
    };
    ui.scope(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.spacing_mut().button_padding = egui::vec2(8.0, 2.0);
        // Reads "Simple | Advanced" in either direction (the top bar lays out right to left).
        if ui.layout().prefer_right_to_left() {
            advanced(ui);
            simple(ui);
        } else {
            simple(ui);
            advanced(ui);
        }
    });
}
