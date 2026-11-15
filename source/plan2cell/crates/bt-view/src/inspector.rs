//! The side panel (the selected node with its execution, ports and cell motion; the blackboard;
//! edits; the log), hover tooltips and the right-click menu. The values (ports, blackboard,
//! edits) are in the `values` child module.
//!
//! Everything here only reads the executor and returns [`Action`]s, which the panel applies once
//! the frame is drawn; text being typed is kept in the panel's drafts ([`edit`](crate::edit)).
//! While the timeline shows the past, decisions still apply at the live instant, and their labels
//! say so.

use bt_cell::{CELL_LEAVES, CellLeaves, Preview};
use bt_core::status::NodeView;
use bt_core::{NodeKind, NodeStatus, Uid};
use bt_exec::{Executor, LeafPolicy};
use egui::{Id, RichText, Ui};

use crate::edit::{Drafts, short, summary};
use crate::panel::{Action, BtPanel};
use crate::style;

mod values;

use values::{blackboard, edits, ports};

/// What the inspector shows: the run, as of the instant on screen.
struct View<'a> {
    /// The panel's id, salting every grid.
    id: Id,
    ex: &'a Executor,
    cell: Option<&'a CellLeaves>,
    /// The tick and instant on screen (scrubbed, or the latest).
    tick: u64,
    t: f64,
    /// The past is shown; decisions still apply at `now`.
    scrubbed: bool,
}

/// " at t = …" on a decision's label while the past is shown: it applies at `now`.
fn at_now(ex: &Executor, scrubbed: bool) -> String {
    if scrubbed {
        format!(" at t = {:.2} s", ex.now())
    } else {
        String::new()
    }
}

fn status_text(status: NodeStatus) -> RichText {
    RichText::new(status.as_str())
        .strong()
        .color(style::status(status).unwrap_or(style::TEXT_DIM))
}

fn dim(text: impl Into<String>) -> RichText {
    RichText::new(text).small().color(style::TEXT_DIM)
}

/// A cell leaf (`ExecuteTrajectory`, `MoveHome`, `WaitForRobot`, `CheckSafety`).
fn is_cell_leaf(ex: &Executor, uid: Uid) -> bool {
    let node = ex.tree().node(uid);
    node.kind.is_leaf() && CELL_LEAVES.contains(&node.id.as_str())
}

/// A RUNNING motion leaf the cell leaves placed motion for.
fn running_motion(ex: &Executor, uid: Uid) -> bool {
    ex.status(uid) == NodeStatus::Running
        && ex
            .leaf_handler::<CellLeaves>()
            .is_some_and(|cell| cell.motion(uid).is_some())
}

pub(crate) fn tooltip(ui: &mut Ui, ex: &Executor, uid: Uid, view: NodeView, live: bool) {
    let node = ex.tree().node(uid);
    ui.horizontal(|ui| {
        ui.strong(&node.id);
        ui.label(RichText::new(node.kind.as_str()).color(style::header(node.kind)));
        ui.label(status_text(view.status));
        if let Some(was) = view.was {
            ui.label(
                RichText::new(format!("(was {was})"))
                    .color(style::was(was).unwrap_or(style::TEXT_DIM)),
            );
        }
    });
    if node.name != node.id {
        ui.label(format!("name: {}", node.name));
    }
    if node.line > 0 {
        ui.label(dim(format!("{} · line {}", node.tree_id, node.line)));
    }
    if live && let Some(reason) = ex.waiting_reason(uid) {
        ui.colored_label(style::WARN, reason);
    }
    let arrow = style::glyph(ui.ctx(), "→", "->");
    for port in &node.ports {
        let mut line = format!("{} {}", port.direction.label(), port.name);
        if let Some(raw) = port.effective() {
            line += &format!(" = {}", short(raw, 40));
        }
        if port.blackboard_key().is_some() {
            let live = ex.port_value(uid, &port.name);
            let live = live.map_or_else(|| "(unset)".to_string(), |v| summary(&v, 60));
            line += &format!("  {arrow} {live}");
        }
        let edited = ex.port_original(uid, &port.name).is_some();
        let color = if edited { style::EDITED } else { style::TEXT };
        ui.label(RichText::new(line).monospace().color(color));
        if !port.description.is_empty() {
            ui.label(dim(&port.description));
        }
    }
    if node.unknown_model {
        ui.colored_label(
            style::WARN,
            "Not builtin and not in <TreeNodesModel>; kind inferred.",
        );
    }
    if node.kind != NodeKind::Root {
        ui.label(dim("Right-click to force it or break before it starts"));
    }
    if node.kind == NodeKind::SubTree {
        ui.label(dim("Double-click to collapse / expand"));
    }
}

/// One menu entry: `action` when clicked.
fn item(ui: &mut Ui, text: impl Into<String>, hover: &str, actions: &mut Vec<Action>, a: Action) {
    let mut response = ui.button(text.into());
    if !hover.is_empty() {
        response = response.on_hover_text(hover);
    }
    if response.clicked() {
        actions.push(a);
        ui.close();
    }
}

/// The right-click menu. `scrubbed`: the past is shown (decisions apply at `now`).
pub(crate) fn context_menu(
    ui: &mut Ui,
    ex: &Executor,
    uid: Uid,
    scrubbed: bool,
    actions: &mut Vec<Action>,
) {
    let node = ex.tree().node(uid);
    actions.push(Action::Select(Some(uid)));
    if node.kind == NodeKind::SubTree && ui.button("Collapse / expand").clicked() {
        actions.push(Action::ToggleCollapse(uid));
        ui.close();
    }
    if node.kind == NodeKind::Root {
        ui.label(dim("The root runs the tree; force a node below it"));
        return;
    }
    let at = at_now(ex, scrubbed);
    let pause = style::glyph(ui.ctx(), "⏸", "||");
    if ex.breakpoint(uid) {
        item(
            ui,
            "Remove breakpoint",
            "",
            actions,
            Action::Breakpoint(uid, false),
        );
    } else {
        item(
            ui,
            format!("{pause} Break before it starts"),
            "Pauses once each time its parent starts",
            actions,
            Action::Breakpoint(uid, true),
        );
    }
    if ex.paused_at().contains(&uid) {
        let play = style::glyph(ui.ctx(), "▶", ">");
        item(
            ui,
            format!("{play} Continue (let it start){at}"),
            "",
            actions,
            Action::ResumeNode(uid),
        );
    }
    ui.separator();
    let leaf = node.kind.is_leaf();
    let halts = if leaf { "" } else { " (halts its children)" };
    let arrow = style::glyph(ui.ctx(), "→", "->");
    let motion = running_motion(ex, uid);
    for status in [NodeStatus::Success, NodeStatus::Failure] {
        let (text, hover) = if motion {
            (
                format!("Stop motion now {arrow} {status}{at}"),
                "The motion is cut where the robot is now",
            )
        } else {
            (format!("Force {status}{halts}{at}"), "")
        };
        item(ui, text, hover, actions, Action::Force(uid, status, false));
    }
    for status in [NodeStatus::Success, NodeStatus::Failure] {
        item(
            ui,
            format!("Always {status} (pinned){halts}{at}"),
            "",
            actions,
            Action::Force(uid, status, true),
        );
    }
    if ex.override_of(uid).is_some() {
        item(
            ui,
            format!("Clear override{at}"),
            "",
            actions,
            Action::ClearForce(uid),
        );
    }
    if bt_exec::is_stub(&node.id, node.kind) {
        ui.separator();
        if ex.is_handled(uid) {
            item(
                ui,
                format!("Policy: simulate (cell){at}"),
                "Run it with cell-runner's semantics",
                actions,
                Action::Policy(uid, LeafPolicy::Native),
            );
        }
        let options: &[(&str, LeafPolicy)] = if node.kind == NodeKind::Action {
            &[
                ("Policy: wait for override", LeafPolicy::Manual),
                ("Policy: succeed after 1 s", LeafPolicy::DEFAULT_ACTION),
                ("Policy: fail after 1 s", LeafPolicy::Fail { after_s: 1.0 }),
            ]
        } else {
            &[
                ("Policy: SUCCESS", LeafPolicy::DEFAULT_CONDITION),
                ("Policy: FAILURE", LeafPolicy::Fail { after_s: 0.0 }),
            ]
        };
        for (text, policy) in options {
            item(
                ui,
                format!("{text}{at}"),
                "",
                actions,
                Action::Policy(uid, *policy),
            );
        }
    }
}

pub(crate) fn ui(panel: &mut BtPanel, ui: &mut Ui) -> Vec<Action> {
    let mut actions = Vec::new();
    let (tick, t, scrubbed) = (
        panel.shown_tick(),
        panel.shown_time(),
        panel.scrub.is_some(),
    );
    let BtPanel {
        id,
        executor,
        drafts,
        selected,
        ..
    } = panel;
    let Some(ex) = executor.as_ref() else {
        return actions;
    };
    let view = View {
        id: *id,
        ex,
        cell: ex.leaf_handler::<CellLeaves>(),
        tick,
        t,
        scrubbed,
    };
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            match *selected {
                Some(uid) if uid < ex.tree().len() => {
                    selected_node(ui, &view, uid, drafts, &mut actions);
                }
                _ => {
                    ui.label(RichText::new("Click a node to inspect it.").color(style::TEXT_DIM));
                }
            }
            ui.separator();
            egui::CollapsingHeader::new(format!("Blackboard ({})", ex.blackboard().len()))
                .id_salt(view.id.with("blackboard"))
                .default_open(true)
                .show(ui, |ui| blackboard(ui, &view, drafts, &mut actions));
            edits(ui, &view, &mut actions);
            let warnings = &ex.tree().warnings;
            if !warnings.is_empty() {
                let sign = style::glyph(ui.ctx(), "⚠", "!");
                egui::CollapsingHeader::new(
                    RichText::new(format!("{sign} Warnings ({})", warnings.len()))
                        .color(style::WARN),
                )
                .id_salt(view.id.with("warnings"))
                .default_open(false)
                .show(ui, |ui| {
                    for w in warnings {
                        ui.label(RichText::new(w).small());
                    }
                });
            }
            egui::CollapsingHeader::new(format!("Log ({})", ex.log().len()))
                .id_salt(view.id.with("log"))
                .default_open(false)
                .show(ui, |ui| {
                    for entry in ex.log().iter().rev().take(200) {
                        let who = entry
                            .uid
                            .map(|u| ex.tree().node(u).name.clone())
                            .unwrap_or_default();
                        let text =
                            format!("#{} {:.2}s {} {}", entry.tick, entry.t, who, entry.message);
                        if ui
                            .selectable_label(false, RichText::new(text).small().monospace())
                            .clicked()
                        {
                            actions.push(Action::Select(entry.uid));
                        }
                    }
                });
        });
    actions
}

fn selected_node(
    ui: &mut Ui,
    view: &View<'_>,
    uid: Uid,
    drafts: &mut Drafts,
    actions: &mut Vec<Action>,
) {
    let ex = view.ex;
    let node = ex.tree().node(uid);
    let status = ex.trace().view_at(uid, view.tick);
    ui.horizontal(|ui| {
        ui.heading(&node.id);
        ui.label(RichText::new(node.kind.as_str()).color(style::header(node.kind)));
    });
    egui::Grid::new(view.id.with("selected"))
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| {
            ui.label("name");
            ui.label(&node.name);
            ui.end_row();
            ui.label("status");
            ui.horizontal(|ui| {
                ui.label(status_text(status.status));
                if let Some(was) = status.was {
                    ui.label(format!("(was {was})"));
                }
            });
            ui.end_row();
            ui.label("tree");
            ui.label(if node.line > 0 {
                format!("{} · line {}", node.tree_id, node.line)
            } else {
                node.tree_id.clone()
            });
            ui.end_row();
            if let Some(sub) = &node.subtree {
                ui.label("subtree");
                ui.label(format!(
                    "{}{}{}",
                    sub.tree_id,
                    if sub.autoremap { " · _autoremap" } else { "" },
                    if sub.resolved { "" } else { " · missing!" }
                ));
                ui.end_row();
            }
            if let Some(parent) = node.parent {
                ui.label("parent");
                if ui.link(&ex.tree().node(parent).name).clicked() {
                    actions.push(Action::Select(Some(parent)));
                }
                ui.end_row();
            }
        });

    if node.kind != NodeKind::Root {
        execution(ui, view, uid, actions);
    }
    ports(ui, view, uid, drafts, actions);
    cell(ui, view, uid);
    if !node.conditions.is_empty() {
        ui.add_space(4.0);
        ui.strong("Conditions");
        for (k, v) in &node.conditions {
            ui.label(
                RichText::new(format!("{k} = {v}"))
                    .monospace()
                    .color(style::WARN),
            );
        }
    }

    let history: Vec<_> = ex
        .trace()
        .of_node(uid)
        .filter(|t| t.tick <= view.tick)
        .collect();
    ui.add_space(6.0);
    let arrow = style::glyph(ui.ctx(), "→", "->");
    egui::CollapsingHeader::new(format!("History ({})", history.len()))
        .id_salt(view.id.with(("history", uid)))
        .default_open(true)
        .show(ui, |ui| {
            for t in history.iter().rev().take(100) {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("#{:<4} {:>7.2}s", t.tick, t.t))
                            .monospace()
                            .small(),
                    );
                    ui.label(status_text(t.prev));
                    ui.label(arrow);
                    ui.label(status_text(t.status));
                });
            }
        });
}

/// Forcing, pinning, the breakpoint and the policy of a node (any but the root).
fn execution(ui: &mut Ui, view: &View<'_>, uid: Uid, actions: &mut Vec<Action>) {
    let ex = view.ex;
    let node = ex.tree().node(uid);
    let at = at_now(ex, view.scrubbed);
    ui.add_space(6.0);
    ui.strong("Execution");
    match ex.override_of(uid) {
        Some(o) => {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "forced: {}{}",
                    o.status,
                    if o.pinned {
                        " (pinned)"
                    } else {
                        " (next tick)"
                    }
                ));
                if ui.small_button(format!("clear{at}")).clicked() {
                    actions.push(Action::ClearForce(uid));
                }
            });
        }
        None => {
            if !view.scrubbed
                && let Some(reason) = ex.waiting_reason(uid)
            {
                ui.label(RichText::new(format!("waiting: {reason}")).color(style::WARN));
            }
        }
    }
    let motion = running_motion(ex, uid);
    ui.horizontal_wrapped(|ui| {
        let arrow = style::glyph(ui.ctx(), "→", "->");
        for status in [NodeStatus::Success, NodeStatus::Failure] {
            let text = if motion {
                format!("Stop motion now {arrow} {status}{at}")
            } else {
                format!("Force {status}{at}")
            };
            let hover = match (motion, node.kind.is_leaf()) {
                (true, _) => "The motion is cut where the robot is now",
                (false, true) => "Finish it with this status the next time it is ticked",
                (false, false) => "Halts its children and finishes with this status",
            };
            let color = style::status(status).unwrap_or(style::TEXT);
            if ui
                .button(RichText::new(text).color(color))
                .on_hover_text(hover)
                .clicked()
            {
                actions.push(Action::Force(uid, status, false));
            }
        }
    });
    let pinned = ex.override_of(uid).is_some_and(|o| o.pinned);
    let mut pin = pinned;
    ui.checkbox(&mut pin, format!("pin (apply on every tick){at}"));
    if pin != pinned {
        let status = ex
            .override_of(uid)
            .map_or(NodeStatus::Success, |o| o.status);
        actions.push(if pin {
            Action::Force(uid, status, true)
        } else {
            Action::ClearForce(uid)
        });
    }
    ui.horizontal(|ui| {
        let had = ex.breakpoint(uid);
        let mut on = had;
        let pause = style::glyph(ui.ctx(), "⏸", "||");
        ui.checkbox(&mut on, format!("{pause} break before it starts"))
            .on_hover_text("Pauses once each time its parent starts");
        if on != had {
            actions.push(Action::Breakpoint(uid, on));
        }
        if ex.paused_at().contains(&uid) {
            ui.colored_label(style::BREAK, "paused here");
            let play = style::glyph(ui.ctx(), "▶", ">");
            if ui.small_button(format!("{play} Continue{at}")).clicked() {
                actions.push(Action::ResumeNode(uid));
            }
        }
    });
    if node.kind.is_leaf() {
        if bt_exec::is_stub(&node.id, node.kind) {
            policy_editor(ui, ex, uid, node.kind, actions);
            if view.scrubbed {
                ui.label(dim(format!("a new policy applies{at}")));
            }
        } else {
            ui.label(dim(format!(
                "{} is a BT.CPP builtin and runs as such.",
                node.id
            )));
        }
    }
}

fn policy_editor(ui: &mut Ui, ex: &Executor, uid: Uid, kind: NodeKind, actions: &mut Vec<Action>) {
    let policy = ex.policy(uid);
    let mut changed = policy;
    ui.horizontal_wrapped(|ui| {
        ui.label("runs as:");
        if ex.is_handled(uid)
            && ui
                .selectable_label(policy == LeafPolicy::Native, "simulate")
                .on_hover_text("cell-runner's semantics (the cell leaves)")
                .clicked()
        {
            changed = LeafPolicy::Native;
        }
        if kind == NodeKind::Action {
            let after = match policy {
                LeafPolicy::Succeed { after_s } | LeafPolicy::Fail { after_s } => after_s,
                LeafPolicy::Manual | LeafPolicy::Native => 1.0,
            };
            if ui
                .selectable_label(matches!(policy, LeafPolicy::Succeed { .. }), "succeed")
                .clicked()
            {
                changed = LeafPolicy::Succeed { after_s: after };
            }
            if ui
                .selectable_label(matches!(policy, LeafPolicy::Fail { .. }), "fail")
                .clicked()
            {
                changed = LeafPolicy::Fail { after_s: after };
            }
            if ui
                .selectable_label(policy == LeafPolicy::Manual, "wait")
                .on_hover_text("RUNNING until forced")
                .clicked()
            {
                changed = LeafPolicy::Manual;
            }
            if let LeafPolicy::Succeed { after_s } | LeafPolicy::Fail { after_s } = &mut changed {
                ui.add(
                    egui::DragValue::new(after_s)
                        .range(0.0..=3600.0)
                        .speed(0.05)
                        .suffix(" s"),
                );
            }
        } else {
            let fails = matches!(policy, LeafPolicy::Fail { .. });
            let succeeds = !fails && policy != LeafPolicy::Native;
            if ui.selectable_label(succeeds, "SUCCESS").clicked() {
                changed = LeafPolicy::DEFAULT_CONDITION;
            }
            if ui.selectable_label(fails, "FAILURE").clicked() {
                changed = LeafPolicy::Fail { after_s: 0.0 };
            }
        }
    });
    if changed != policy {
        actions.push(Action::Policy(uid, changed));
    }
}

/// What a cell leaf does (or will do) with the cell: its motion, gate or safety check.
fn cell(ui: &mut Ui, view: &View<'_>, uid: Uid) {
    let ex = view.ex;
    let Some(preview) = bt_cell::preview(ex, uid) else {
        return;
    };
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        ui.strong("Cell");
        if !ex.is_handled(uid) {
            ui.label(dim("(runs as a stub: Cell sim is off)"));
        } else if ex.policy(uid) != LeafPolicy::Native {
            ui.label(dim("(runs as a stub: its policy is not simulate)"));
        }
    });
    match preview {
        Preview::Execute(Ok(op)) => {
            let scale = if op.velocity_scale > 0.0 {
                op.velocity_scale
            } else {
                1.0
            };
            let duration = op.traj.last().map_or(0.0, |w| w.time_s) / scale;
            ui.label(format!(
                "{} {} ({}): {} waypoints in {duration:.3} s, hold {:.2} s",
                op.robot,
                op.op_id,
                op.op_type,
                op.traj.len(),
                op.hold_s
            ));
            let mut extras = Vec::new();
            if op.start_delay_s > 0.0 {
                extras.push(format!("starts {:.2} s late", op.start_delay_s));
            }
            if scale != 1.0 {
                extras.push(format!("velocity × {scale}"));
            }
            if op.interlock_joints.is_some() {
                extras.push("pre-moves to its interlock target first".to_string());
            }
            if !extras.is_empty() {
                ui.label(dim(extras.join(" · ")));
            }
        }
        Preview::Execute(Err(warning)) => {
            ui.colored_label(style::WARN, warning);
        }
        Preview::Home { robot, planned } => match robot {
            None => {
                ui.colored_label(style::WARN, "no robot_ns: skipped");
            }
            Some(robot) => {
                let how = match planned {
                    Ok(Some(path)) if !path.is_empty() => format!(
                        "planned path, {} waypoints in {:.3} s",
                        path.len(),
                        path.last().map_or(0.0, |w| w.time_s)
                    ),
                    Ok(_) => format!(
                        "no path: moves home unplanned in {} s",
                        bt_cell::joints::HOME_RETURN_TIME_S
                    ),
                    Err(e) => format!("{e}: moves home unplanned"),
                };
                ui.label(format!("{robot} home: {how}"));
            }
        },
        Preview::Wait { label, deps } => match deps {
            Ok(deps) if deps.is_empty() => {
                ui.label(format!("{label}: waits for nothing"));
            }
            Ok(deps) => {
                ui.label(format!("{label} waits for:"));
                for dep in deps {
                    let tol = if dep.tolerance_rad > 0.0 {
                        dep.tolerance_rad
                    } else {
                        bt_cell::joints::DEFAULT_INTERLOCK_TOL_RAD
                    };
                    ui.label(dim(format!(
                        "{} within {tol} rad of {}",
                        dep.robot,
                        summary(&serde_json::json!(dep.joint_positions), 60)
                    )));
                }
            }
            Err(e) => {
                ui.colored_label(style::WARN, format!("{label}: {e}"));
            }
        },
        Preview::Safety { condition } => {
            ui.label(format!(
                "condition {condition}: SUCCESS (cell-runner checks nothing during playback)"
            ));
        }
    }
    let Some(cell) = view.cell else {
        return;
    };
    if let Some(progress) = cell.progress(uid, view.t, view.tick) {
        ui.add(
            egui::ProgressBar::new(progress)
                .desired_width(200.0)
                .text(format!("{:.0} %", progress * 100.0)),
        );
    }
    if view.scrubbed {
        return;
    }
    if let Some(placed) = cell.motion(uid) {
        ui.label(format!(
            "moving: done at t = {:.2} s ({} segment{})",
            placed.done_at,
            placed.segments.len(),
            if placed.segments.len() == 1 { "" } else { "s" }
        ));
    }
    if let Some(gate) = cell.gate(uid) {
        ui.label(format!("waiting since t = {:.2} s", gate.since));
        for dep in &gate.deps {
            let when = match dep.opens_at {
                Some(t) => format!("opens at t = {t:.2} s"),
                None => "no motion placed so far gets it there".to_string(),
            };
            ui.label(dim(format!(
                "{}: {:.3} rad away (tolerance {}) — {when}",
                dep.robot, dep.error, dep.tol
            )));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edit::{self, DraftKey};

    const WELDING: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test_data/bt_1790934549_welding_cell_2_small.json"
    );

    fn welding() -> BtPanel {
        let mut panel = BtPanel::new("inspector");
        let text = std::fs::read_to_string(WELDING).unwrap();
        panel.load_text("welding.json", &text).unwrap();
        panel
    }

    fn draw(ctx: &egui::Context, f: impl FnMut(&mut Ui)) {
        let mut f = f;
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            egui::CentralPanel::default().show(ui, |ui| f(ui));
        });
        output.textures_delta.clear();
    }

    #[test]
    fn menus_and_tooltips_draw_for_every_node() {
        let ctx = egui::Context::default();
        let mut panel = welding();
        panel.run_to_time(5.0);
        let ex = panel.executor().unwrap();
        let w01 = ex
            .tree()
            .nodes
            .iter()
            .find(|n| n.name == "weld_robot1_W01")
            .unwrap()
            .uid;
        assert!(running_motion(ex, w01));
        for uid in 0..ex.tree().len() {
            let view = ex.trace().view_at(uid, ex.ticks());
            for scrubbed in [false, true] {
                let mut actions = Vec::new();
                draw(&ctx, |ui| {
                    tooltip(ui, ex, uid, view, !scrubbed);
                    context_menu(ui, ex, uid, scrubbed, &mut actions);
                });
                // Drawing selects the node; nothing else is decided without a click.
                assert_eq!(actions, [Action::Select(Some(uid))]);
            }
        }
        for (_, _, value) in ex.blackboard() {
            draw(&ctx, |ui| edit::hover(ui, value));
        }
    }

    #[test]
    fn open_editors_draw_headless() {
        let ctx = egui::Context::default();
        let mut panel = welding();
        let ex = panel.executor().unwrap();
        let w01 = ex
            .tree()
            .nodes
            .iter()
            .find(|n| n.name == "weld_robot1_W01")
            .unwrap()
            .uid;
        panel.select(Some(w01));
        // An open blackboard editor with valid and with broken JSON, and a port being typed.
        for text in ["[1, 2]", "[1, 2", "estop_clear", ""] {
            panel
                .drafts
                .insert(DraftKey::Blackboard("traj_robot1_W01".into()), text.into());
            panel
                .drafts
                .insert(DraftKey::Port(w01, "robot_ns".into()), "robot".into());
            draw(&ctx, |ui| {
                let actions = super::ui(&mut panel, ui);
                assert!(actions.is_empty(), "{actions:?}");
            });
        }
        // The port draft is dropped once its field is not focused; the editor stays open.
        assert!(
            !panel
                .drafts
                .contains_key(&DraftKey::Port(w01, "robot_ns".into()))
        );
        assert!(
            panel
                .drafts
                .contains_key(&DraftKey::Blackboard("traj_robot1_W01".into()))
        );
    }
}
