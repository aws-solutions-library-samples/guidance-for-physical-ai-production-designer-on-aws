//! The "Labels" toggle painted over the 3D view's bottom-right corner, on every tool that shows
//! one (Robo Fitter, Product & Task, Robo Dresser, Live cell, Training).
//!
//! One setting for all views, kept between runs like the theme: off, the views draw no text
//! labels at all (weld names, robot names, the TCP's, also those shown only on hover). The views
//! apply it themselves, at render time (`scene3d::set_labels_shown`, read by each
//! `Scene3d::forward`): no backend round trip. On the Robo Fitter it is the same setting as its
//! "Labels" / "Weld labels" checkboxes (`FitterView::show_labels`): the app keeps them in step.

use crate::rr::{eframe, egui};
use crate::theme;

const STORAGE_KEY: &str = "factoryos_labels";

/// Kept clear between the button and the view's edges (points).
const MARGIN: f32 = 10.0;

/// The setting as saved last (on when never saved).
pub fn load(storage: Option<&dyn eframe::Storage>) -> bool {
    !matches!(storage.and_then(|s| s.get_string(STORAGE_KEY)).as_deref(), Some("off"))
}

pub fn save(storage: &mut dyn eframe::Storage) {
    let value = if crate::scene3d::labels_shown() { "on" } else { "off" };
    storage.set_string(STORAGE_KEY, value.to_owned());
}

/// The Robo Fitter's weld labels (`FitterView::show_labels`) are `weld_labels` now: when they
/// changed since the last call (its checkboxes, the HTTP/MCP API), the views follow. Call every
/// frame. Returns whether the setting changed.
pub fn follow_fitter(ctx: &egui::Context, weld_labels: bool) -> bool {
    let id = egui::Id::new("factoryos_labels_fitter_seen");
    let seen = ctx.data_mut(|d| {
        let seen = d.get_temp::<bool>(id);
        d.insert_temp(id, weld_labels);
        seen
    });
    let changed = seen.is_some_and(|seen| seen != weld_labels) && crate::scene3d::labels_shown() != weld_labels;
    if changed {
        crate::scene3d::set_labels_shown(weld_labels);
    }
    changed
}

/// The toggle over the bottom-right corner of the 3D view at `view` (points). Returns the new
/// setting when it was clicked (already applied: [`crate::scene3d::set_labels_shown`]).
pub fn ui(ctx: &egui::Context, view: egui::Rect) -> Option<bool> {
    let on = crate::scene3d::labels_shown();
    let mut toggled = None;
    egui::Area::new(egui::Id::new("factoryos_labels_overlay"))
        .order(egui::Order::Middle)
        .pivot(egui::Align2::RIGHT_BOTTOM)
        .fixed_pos(view.right_bottom() - egui::vec2(MARGIN, MARGIN))
        .constrain_to(view)
        .show(ctx, |ui| {
            let p = theme::palette(ui);
            let (fill, text, stroke) = if on { (p.accent, p.accent_fg, p.accent) } else { (p.card, p.text, p.border_strong) };
            let button = egui::Button::new(egui::RichText::new("🏷 Labels").color(text))
                .fill(fill)
                .stroke(egui::Stroke::new(1.0, stroke))
                .corner_radius(6);
            let tip = if on {
                "Labels shown: weld spots, robots, … Click to hide them in the 3D view"
            } else {
                "Labels hidden. Click to show the weld spots', robots', … names in the 3D view"
            };
            if ui.add(button).on_hover_text(tip).clicked() {
                crate::scene3d::set_labels_shown(!on);
                toggled = Some(!on);
            }
        });
    toggled
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rr::eframe::Storage as _;

    #[derive(Default)]
    struct Storage(std::collections::HashMap<String, String>);

    impl eframe::Storage for Storage {
        fn get_string(&self, key: &str) -> Option<String> {
            self.0.get(key).cloned()
        }
        fn set_string(&mut self, key: &str, value: String) {
            self.0.insert(key.to_owned(), value);
        }
        fn remove_string(&mut self, key: &str) {
            self.0.remove(key);
        }
        fn flush(&mut self) {}
    }

    #[test]
    fn on_unless_saved_off() {
        assert!(load(None));
        let mut storage = Storage::default();
        assert!(load(Some(&storage)));
        storage.set_string(STORAGE_KEY, "off".into());
        assert!(!load(Some(&storage)));
        storage.set_string(STORAGE_KEY, "on".into());
        assert!(load(Some(&storage)));
    }
}
