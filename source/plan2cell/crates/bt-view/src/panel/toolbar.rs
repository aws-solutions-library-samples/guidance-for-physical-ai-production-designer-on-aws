//! The toolbar above the canvas: the file and its trees, running the tree (Run, Step, Run to
//! wait, Run to t, Reset and the clock), Overrides ▾, Cell sim, the layout and the view (Fit,
//! Follow, Compact), and which panels show.
//!
//! The run buttons only ask for a run ([`Action::Step`] and the like): the panel runs it once
//! the inspector has drawn, so a value typed there and committed by the same click (the field
//! loses focus) is in effect when the run starts.

use bt_core::NodeKind;
use bt_core::layout::Orientation;
use bt_exec::LeafPolicy;

use super::{Action, BtPanel, CellMode, PlanInfo, label};
use crate::style;

impl BtPanel {
    pub(super) fn toolbar(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        ui.horizontal_wrapped(|ui| {
            let source = ui.strong(if self.source.is_empty() {
                "no tree"
            } else {
                &self.source
            });
            if let Some(plan) = self.plan.as_ref().filter(|p| **p != PlanInfo::default()) {
                source.on_hover_ui(|ui| {
                    if let Some(id) = &plan.tree_id {
                        ui.label(format!("tree_id: {id}"));
                    }
                    if let Some(id) = &plan.cell_id {
                        ui.label(format!("cell_id: {id}"));
                    }
                    if let Some(n) = plan.num_operations {
                        ui.label(format!("{n} operations planned"));
                    }
                });
            }
            let tree_ids = self
                .document
                .as_ref()
                .map(|d| d.tree_ids.clone())
                .unwrap_or_default();
            if tree_ids.len() > 1 {
                let current = self.tree_id().unwrap_or_default().to_string();
                let mut chosen = current.clone();
                egui::ComboBox::from_id_salt(self.id.with("tree"))
                    .selected_text(&chosen)
                    .show_ui(ui, |ui| {
                        for id in &tree_ids {
                            ui.selectable_value(&mut chosen, id.clone(), id);
                        }
                    });
                if chosen != current {
                    // An error shows in the timeline; the current tree stays.
                    let _ = self.show_tree(&chosen);
                }
            }
            let Some((now, done)) = self.executor.as_ref().map(|ex| (ex.now(), ex.is_done()))
            else {
                return;
            };
            ui.separator();
            if self.running {
                if ui.button(label(ui, "⏸", "||", "Pause")).clicked() {
                    actions.push(Action::SetRunning(false));
                }
            } else if ui
                .button(label(ui, "▶", ">", "Run"))
                .on_hover_text("Run in real time, ticking whenever something changes")
                .clicked()
            {
                actions.push(Action::SetRunning(true));
            }
            if ui
                .button(label(ui, "⏵", ">", "Step"))
                .on_hover_text(
                    "One tick at the next instant something changes, or now if nothing is \
                     scheduled",
                )
                .clicked()
            {
                actions.push(Action::Step);
            }
            if ui
                .button(label(ui, "⏭", ">>", "Run to wait"))
                .on_hover_text(
                    "Tick in virtual time until the tree finishes or halts (a breakpoint, a \
                     manual leaf, a stalled gate)",
                )
                .clicked()
            {
                actions.push(Action::RunToWait);
            }
            // A finished run starts over, so any instant can be run to; else only a later one.
            let from = if done { 0.0 } else { now };
            self.run_to_t = self.run_to_t.max(from);
            ui.label("Run to t =");
            // It follows the clock while the tree runs, so it is as wide as its widest value
            // with as many whole seconds' digits as now (at least three): a field that narrows
            // and widens with the digits moves the rest of the toolbar, and its wrapping, with
            // every frame. It widens once per digit the clock gains, past 1000 s, 10000 s, ...
            let digits = (self.run_to_t.max(now).max(0.0).floor() as u64)
                .to_string()
                .len()
                .max(3);
            let widest = format!("{}.000 s", "0".repeat(digits));
            let widest = egui::WidgetText::from(widest).into_galley(
                ui,
                Some(egui::TextWrapMode::Extend),
                f32::INFINITY,
                ui.style().drag_value_text_style.clone(),
            );
            // The padding, and room for the frame.
            let width = widest.size().x + 2.0 * ui.spacing().button_padding.x + 4.0;
            // Not in a scope of its own, which would not wrap with the rest of the row.
            let interact = ui.spacing().interact_size;
            ui.spacing_mut().interact_size.x = interact.x.max(width);
            ui.add(
                egui::DragValue::new(&mut self.run_to_t)
                    .range(from..=f64::INFINITY)
                    .speed(0.1)
                    .max_decimals(3)
                    .suffix(" s"),
            );
            ui.spacing_mut().interact_size = interact;
            if ui
                .button(style::glyph(ui.ctx(), "⏵", ">"))
                .on_hover_text(
                    "Tick in virtual time up to this instant, unless the tree finishes or halts \
                     first; manual leaves keep waiting while the clock moves",
                )
                .clicked()
            {
                actions.push(Action::RunToTime(self.run_to_t));
            }
            if ui.button(label(ui, "⟲", "↺", "Reset")).clicked() {
                actions.push(Action::Reset);
            }
            ui.add(
                egui::Slider::new(&mut self.speed, 0.1..=20.0)
                    .logarithmic(true)
                    .text("× speed"),
            );
            ui.add(
                egui::DragValue::new(&mut self.tick_hz)
                    .range(1.0..=200.0)
                    .suffix(" Hz"),
            )
            .on_hover_text("Heartbeat for trees that are busy without a timer");
            ui.checkbox(&mut self.looping, "Loop");
            ui.separator();
            self.overrides_menu(ui, actions);
            if self.cell_capable {
                let mut on = self.cell().is_some();
                if ui
                    .toggle_value(&mut on, "Cell sim")
                    .on_hover_text(
                        "Run ExecuteTrajectory, MoveHome, WaitForRobot and CheckSafety with \
                         cell-runner's semantics, or as stubs (switching starts the run over)",
                    )
                    .changed()
                {
                    actions.push(Action::SetCellMode(if on {
                        CellMode::On
                    } else {
                        CellMode::Off
                    }));
                }
            }
            ui.separator();
            let before = self.orientation;
            let (down, right) = (
                label(ui, "⬇", "v", "Vertical"),
                label(ui, "➡", ">", "Horizontal"),
            );
            ui.selectable_value(&mut self.orientation, Orientation::Vertical, down);
            ui.selectable_value(&mut self.orientation, Orientation::Horizontal, right);
            if before != self.orientation {
                self.fit = true;
            }
            if ui
                .button(label(ui, "⌂", "⤢", "Fit"))
                .on_hover_text(
                    "Frame the whole tree. Drag or scroll to pan; ⌘/Ctrl + scroll or pinch to \
                     zoom",
                )
                .clicked()
            {
                self.fit = true;
            }
            let mut follow = self.follow;
            if ui
                .toggle_value(&mut follow, "Follow")
                .on_hover_text(
                    "Keep the running leaves in view, zoomed in to read them when the tree is \
                     too big to read whole; each new running leaf moves the view",
                )
                .changed()
            {
                self.set_follow(follow);
            }
            let mut compact = self.compact;
            if ui
                .toggle_value(&mut compact, "Compact")
                .on_hover_text(
                    "Cards without their port rows, so more of the tree fits; hover or select a \
                     node for its ports",
                )
                .changed()
            {
                self.set_compact(compact);
            }
            if ui.button("Collapse subtrees").clicked()
                && let Some(ex) = &self.executor
            {
                self.collapsed = ex
                    .tree()
                    .nodes
                    .iter()
                    .filter(|n| n.kind == NodeKind::SubTree)
                    .map(|n| n.uid)
                    .collect();
                self.fit = true;
            }
            if ui.button("Expand all").clicked() {
                self.collapsed.clear();
                self.fit = true;
            }
            if self.cell().is_some() {
                ui.toggle_value(&mut self.show_cell, "Cell").on_hover_text(
                    "The cell panel: the robots, the motion placed so far and the warnings",
                );
            }
            ui.toggle_value(&mut self.show_inspector, "Inspector");
        });
    }

    /// Overrides ▾: policies for every leaf, and clearing what the user set.
    fn overrides_menu(&self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let Some(ex) = &self.executor else {
            return;
        };
        let forced = (0..ex.tree().len())
            .filter(|&uid| ex.override_of(uid).is_some())
            .count();
        let breakpoints = ex.breakpoints().len();
        let edits = ex.port_edits().len() + ex.blackboard_edits().len();
        let auto_release = self.auto_release;
        let down = style::glyph(ui.ctx(), "▾", "⏷");
        ui.menu_button(format!("Overrides {down}"), |ui| {
            for (text, policy) in [
                ("All actions manual (wait for override)", LeafPolicy::Manual),
                ("All actions succeed after 1 s", LeafPolicy::DEFAULT_ACTION),
                (
                    "All actions succeed at once",
                    LeafPolicy::Succeed { after_s: 0.0 },
                ),
            ] {
                if ui.button(text).clicked() {
                    actions.push(Action::AllPolicies(NodeKind::Action, policy));
                    ui.close();
                }
            }
            if ui
                .add_enabled(
                    ex.has_leaf_handler(),
                    egui::Button::new("Cell leaves: simulate"),
                )
                .on_hover_text("Every cell leaf runs with cell-runner's semantics again")
                .on_disabled_hover_text("Cell sim is off")
                .clicked()
            {
                for kind in [NodeKind::Action, NodeKind::Condition] {
                    actions.push(Action::AllPolicies(kind, LeafPolicy::Native));
                }
                ui.close();
            }
            ui.separator();
            for (text, n, action) in [
                ("Clear overrides", forced, Action::ClearAllForces),
                ("Clear breakpoints", breakpoints, Action::ClearBreakpoints),
                ("Revert all edits", edits, Action::RevertAllEdits),
            ] {
                if ui
                    .add_enabled(n > 0, egui::Button::new(format!("{text} ({n})")))
                    .clicked()
                {
                    actions.push(action);
                    ui.close();
                }
            }
            ui.separator();
            let mut on = auto_release;
            if ui
                .checkbox(&mut on, "Auto-release stalled gates (like cell-runner)")
                .on_hover_text(
                    "When nothing can move, waits that started earlier give up with \
                     cell-runner's \"never reaches\" warning, instead of halting the run",
                )
                .changed()
            {
                actions.push(Action::SetAutoRelease(on));
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use crate::BtPanel;
    use egui::accesskit::Role;

    /// Where the toolbar's Reset button is, drawn with `run_to_t` in the Run to t field, in a
    /// context kept across frames like an app's.
    fn reset_at(ctx: &egui::Context, panel: &mut BtPanel, run_to_t: f64) -> egui::Pos2 {
        let mut at = None;
        for _ in 0..2 {
            panel.run_to_t = run_to_t;
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(520.0, 980.0),
                )),
                ..Default::default()
            };
            let mut output = ctx.run_ui(input, |ui| {
                egui::CentralPanel::default().show(ui, |ui| panel.tree_ui(ui));
            });
            output.textures_delta.clear();
            let nodes = output.platform_output.accesskit_update.unwrap().nodes;
            let reset = nodes
                .iter()
                .find(|(_, n)| {
                    n.role() == Role::Button && n.label().is_some_and(|l| l.ends_with("Reset"))
                })
                .and_then(|(_, n)| n.bounds())
                .expect("a Reset button");
            at = Some(egui::pos2(reset.x0 as f32, reset.y0 as f32));
        }
        at.unwrap()
    }

    /// The Run to t field follows the clock while the tree runs: its width, and so what comes
    /// after it on the toolbar's wrapping rows, does not change with the digits shown.
    #[test]
    fn the_toolbar_holds_still_while_the_clock_runs() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../test_data/bt_1790934549_welding_cell_2_small.json"
        );
        let mut panel = BtPanel::new("welding");
        panel
            .load_text("welding.json", &std::fs::read_to_string(path).unwrap())
            .unwrap();
        let ctx = egui::Context::default();
        crate::fonts::install(&ctx);
        ctx.enable_accesskit();
        let first = reset_at(&ctx, &mut panel, 23.716);
        for t in [24.0, 24.1, 24.12, 24.125, 25.5, 99.999, 123.456] {
            assert_eq!(reset_at(&ctx, &mut panel, t), first, "t = {t}");
        }
        // Past 1000 s (a long cell plan, a plain tree on Loop): once a digit wider, then still.
        let past = reset_at(&ctx, &mut panel, 1000.0);
        for t in [1000.1, 1000.12, 1000.125, 1234.567, 9999.999] {
            assert_eq!(reset_at(&ctx, &mut panel, t), past, "t = {t}");
        }
    }
}
