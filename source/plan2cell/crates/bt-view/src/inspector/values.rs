//! The inspector's values: the selected node's ports, the blackboard, and every edit, each
//! editable ([`edit`]) and revertible.
//!
//! Big values (trajectories) are summed up in one line and only listed while hovered.

use bt_core::{NodeKind, NodeStatus, Uid};
use bt_exec::Executor;
use egui::{RichText, Ui};

use super::{View, dim, is_cell_leaf};
use crate::edit::{self, DraftKey, Drafts, short, summary};
use crate::panel::Action;
use crate::style;

/// What an unset port of `uid` reads: cell-runner's default for a cell leaf (it ignores the
/// model's), the model's default otherwise.
fn port_default<'a>(ex: &'a Executor, uid: Uid, port: &'a bt_core::model::Port) -> Option<&'a str> {
    if is_cell_leaf(ex, uid) {
        bt_cell::ports::default_of(&ex.tree().node(uid).id, &port.name)
    } else {
        port.default.as_deref()
    }
}

/// The ports, editable: what the node reads the next time it reads them.
pub(super) fn ports(
    ui: &mut Ui,
    view: &View<'_>,
    uid: Uid,
    drafts: &mut Drafts,
    actions: &mut Vec<Action>,
) {
    let ex = view.ex;
    let node = ex.tree().node(uid);
    // A SubTree's ports remap the blackboard; edit the entry instead.
    let editable = !matches!(node.kind, NodeKind::Root | NodeKind::SubTree);
    let addable: Vec<&str> = if editable && is_cell_leaf(ex, uid) {
        bt_cell::ports::leaf_ports(&node.id)
            .iter()
            .copied()
            // `name` is the instance name, not a port to add.
            .filter(|p| *p != "name" && node.port(p).is_none())
            .collect()
    } else {
        Vec::new()
    };
    if node.ports.is_empty() && addable.is_empty() {
        return;
    }
    ui.add_space(4.0);
    ui.horizontal(|ui| {
        ui.strong("Ports");
        if editable && node.kind.is_leaf() && ex.status(uid) == NodeStatus::Running {
            ui.label(dim("takes effect the next time it starts"));
        }
    });
    egui::Grid::new(view.id.with(("ports", uid)))
        .num_columns(4)
        .striped(true)
        .show(ui, |ui| {
            for port in &node.ports {
                let original = ex.port_original(uid, &port.name);
                let name = RichText::new(format!("{} {}", port.direction.label(), port.name))
                    .monospace()
                    .color(if original.is_some() {
                        style::EDITED
                    } else {
                        style::TEXT
                    });
                let response = ui.label(name);
                if !port.description.is_empty() {
                    response.on_hover_text(&port.description);
                }
                if editable {
                    let default = port_default(ex, uid, port);
                    edit::port_editor(ui, uid, port, default, drafts, actions);
                } else {
                    ui.label(RichText::new(short(port.effective().unwrap_or("—"), 30)).monospace());
                }
                // What a `{key}` port reads now.
                let live = port
                    .blackboard_key()
                    .map(|_| ex.port_value(uid, &port.name));
                match live {
                    Some(Some(value)) => {
                        ui.label(
                            RichText::new(summary(&value, 30))
                                .monospace()
                                .color(style::CHIP_BLACKBOARD),
                        )
                        .on_hover_ui(|ui| edit::hover(ui, &value));
                    }
                    Some(None) => {
                        ui.label(dim("(unset)"));
                    }
                    None => {
                        ui.label("");
                    }
                }
                match original {
                    Some(original) => {
                        let file = original.map_or_else(
                            || "file: (not set)".to_string(),
                            |v| format!("file: {}", short(v, 60)),
                        );
                        let revert = style::glyph(ui.ctx(), "↺", "undo");
                        if ui.small_button(revert).on_hover_text(file).clicked() {
                            actions.push(Action::SetPort {
                                uid,
                                port: port.name.clone(),
                                value: None,
                            });
                        }
                    }
                    None => {
                        ui.label("");
                    }
                }
                ui.end_row();
            }
        });
    if !addable.is_empty() {
        let plus = style::glyph(ui.ctx(), "＋", "+");
        ui.menu_button(format!("{plus} port"), |ui| {
            for port in addable {
                if ui.button(port).clicked() {
                    let value = bt_cell::ports::default_of(&node.id, port).unwrap_or("");
                    actions.push(Action::SetPort {
                        uid,
                        port: port.to_string(),
                        value: Some(value.to_string()),
                    });
                    ui.close();
                }
            }
        });
    }
}

pub(super) fn blackboard(
    ui: &mut Ui,
    view: &View<'_>,
    drafts: &mut Drafts,
    actions: &mut Vec<Action>,
) {
    let ex = view.ex;
    let entries = ex.blackboard();
    if entries.is_empty() {
        ui.label(RichText::new("empty").color(style::TEXT_DIM));
        return;
    }
    egui::Grid::new(view.id.with("blackboard_grid"))
        .num_columns(4)
        .striped(true)
        .show(ui, |ui| {
            for &(owner, key, value) in &entries {
                let scope = owner.map_or("main".to_string(), |o| ex.tree().node(o).name.clone());
                ui.label(dim(scope));
                let edited = owner.is_none() && ex.blackboard_edited(key);
                let color = if edited { style::EDITED } else { style::TEXT };
                ui.label(RichText::new(key).monospace().color(color));
                ui.label(RichText::new(summary(value, 40)).monospace())
                    .on_hover_ui(|ui| edit::hover(ui, value));
                ui.horizontal(|ui| {
                    if owner.is_some() {
                        return;
                    }
                    let draft = DraftKey::Blackboard(key.to_string());
                    let pen = style::glyph(ui.ctx(), "✎", "edit");
                    if ui
                        .small_button(pen)
                        .on_hover_text("Edit (JSON, or bare text for a string)")
                        .clicked()
                        && !drafts.contains_key(&draft)
                    {
                        drafts.insert(draft, edit::draft_of(value));
                    }
                    if edited {
                        let file = ex.initial_blackboard().get(key).map_or_else(
                            || "file: (no entry)".to_string(),
                            |v| format!("file: {}", summary(v, 60)),
                        );
                        let revert = style::glyph(ui.ctx(), "↺", "undo");
                        if ui.small_button(revert).on_hover_text(file).clicked() {
                            actions.push(Action::SetBlackboard {
                                key: key.to_string(),
                                value: None,
                            });
                        }
                    }
                });
                ui.end_row();
            }
        });
    let mut open: Vec<String> = drafts
        .keys()
        .filter_map(|k| match k {
            DraftKey::Blackboard(key) => Some(key.clone()),
            DraftKey::Port(..) => None,
        })
        .collect();
    open.sort();
    for key in open {
        blackboard_editor(ui, view, &key, drafts, actions);
    }
}

/// The open editor of main-blackboard entry `key` (its draft holds the text).
fn blackboard_editor(
    ui: &mut Ui,
    view: &View<'_>,
    key: &str,
    drafts: &mut Drafts,
    actions: &mut Vec<Action>,
) {
    let draft = DraftKey::Blackboard(key.to_string());
    let Some(text) = drafts.get_mut(&draft) else {
        return;
    };
    ui.add_space(4.0);
    let pen = style::glyph(ui.ctx(), "✎", "edit");
    ui.strong(format!("{pen} {key}"));
    ui.add(
        egui::TextEdit::multiline(text)
            .id_salt(view.id.with(("bb_edit", key)))
            .code_editor()
            .desired_rows(4)
            .desired_width(f32::INFINITY),
    );
    let parsed = edit::parse_edit(text);
    let mut close = false;
    ui.horizontal_wrapped(|ui| {
        if ui
            .add_enabled(parsed.is_ok(), egui::Button::new("Apply"))
            .clicked()
            && let Ok(value) = &parsed
        {
            actions.push(Action::SetBlackboard {
                key: key.to_string(),
                value: Some(value.clone()),
            });
            close = true;
        }
        if ui.button("Cancel").clicked() {
            close = true;
        }
        match &parsed {
            Err(e) => {
                ui.colored_label(style::ERROR, e);
            }
            Ok(value) => {
                ui.label(dim(format!(
                    "{}: read when a node next reads it; Step re-ticks now",
                    summary(value, 30)
                )));
            }
        }
    });
    if close {
        drafts.remove(&draft);
    }
}

/// Every port and blackboard edit, each with ↺, and Revert all.
pub(super) fn edits(ui: &mut Ui, view: &View<'_>, actions: &mut Vec<Action>) {
    let ex = view.ex;
    let ports = ex.port_edits();
    let entries = ex.blackboard_edits();
    let n = ports.len() + entries.len();
    if n == 0 {
        return;
    }
    let pen = style::glyph(ui.ctx(), "✎", "*");
    egui::CollapsingHeader::new(RichText::new(format!("{pen} Edits ({n})")).color(style::EDITED))
        .id_salt(view.id.with("edits"))
        .default_open(true)
        .show(ui, |ui| {
            let revert = style::glyph(ui.ctx(), "↺", "undo");
            egui::Grid::new(view.id.with("edits_grid"))
                .num_columns(3)
                .striped(true)
                .show(ui, |ui| {
                    for (uid, port) in &ports {
                        let node = ex.tree().node(*uid);
                        if ui.link(&node.name).clicked() {
                            actions.push(Action::Focus(*uid));
                        }
                        let now = node.port(port).and_then(|p| p.value.as_deref());
                        let file = ex.port_original(*uid, port).flatten();
                        ui.label(
                            RichText::new(format!(
                                "{port} = {} (file: {})",
                                short(now.unwrap_or(""), 30),
                                short(file.unwrap_or("not set"), 30)
                            ))
                            .monospace(),
                        );
                        if ui.small_button(revert).clicked() {
                            actions.push(Action::SetPort {
                                uid: *uid,
                                port: port.clone(),
                                value: None,
                            });
                        }
                        ui.end_row();
                    }
                    for (key, value) in entries {
                        ui.label(dim("blackboard"));
                        ui.label(
                            RichText::new(format!("{key} = {}", summary(value, 40))).monospace(),
                        )
                        .on_hover_ui(|ui| edit::hover(ui, value));
                        if ui.small_button(revert).clicked() {
                            actions.push(Action::SetBlackboard {
                                key: key.clone(),
                                value: None,
                            });
                        }
                        ui.end_row();
                    }
                });
            if ui.button("Revert all").clicked() {
                actions.push(Action::RevertAllEdits);
            }
        });
}
