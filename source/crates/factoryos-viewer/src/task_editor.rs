//! The "Task Program" pane: the active cell's task programs (the operations the agentic planner
//! plans, `agentic_planner/tasks/<name>.json`) as raw JSON. Secondary: task programs come from the
//! cell's products (their welds are edited in Product & Task, and the backend derives
//! `<fixture>__<task>.json` from them). A derived one is shown read-only here; "Make an editable
//! copy" saves it as a hand-written one. Hand-written ones are edited here, starting from a
//! sample if need be.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::{PlansInfo, TaskProgram, TaskSource, api};
use crate::theme;
use crate::rr::egui::{self, RichText};

use crate::sidebar::{Action, task_source};


/// What came back from the backend.
enum Reply {
    Loaded(TaskProgram),
    Sample(TaskProgram),
    Saved(String, PlansInfo),
    Failed(String),
}

pub struct TaskEditor {
    api: Api,
    replies: Arc<Mutex<Vec<Reply>>>,
    /// The cell the text belongs to; the editor starts over when the active cell changes.
    cell: String,
    name: String,
    text: String,
    /// The text as last loaded or saved, to tell unsaved edits apart.
    saved: Option<String>,
    status: Option<Result<String, String>>,
    /// The task to open once the backend has listed the cell's task programs.
    open_requested: Option<String>,
    /// Newer task lists to hand to the sidebar after a save.
    pub saved_listing: Option<PlansInfo>,
}

impl TaskEditor {
    pub fn new(api: Api) -> Self {
        Self {
            api,
            replies: Arc::default(),
            cell: String::new(),
            name: String::new(),
            text: String::new(),
            saved: None,
            status: None,
            open_requested: None,
            saved_listing: None,
        }
    }

    /// Show task program `name` (or, with `None`, a new empty one) next time the pane is drawn.
    pub fn open(&mut self, name: Option<String>) {
        match name.filter(|n| !n.is_empty()) {
            Some(name) => self.open_requested = Some(name),
            None => {
                self.name.clear();
                self.text.clear();
                self.saved = None;
                self.status = None;
            }
        }
    }

    fn load(&self, name: &str) {
        let replies = self.replies.clone();
        self.api.get(&format!("{}?name={name}", api::CELL_TASK), move |task: Result<TaskProgram, String>| {
            replies.lock().unwrap().push(match task {
                Ok(task) => Reply::Loaded(task),
                Err(err) => Reply::Failed(err),
            });
        });
    }

    fn load_sample(&self) {
        let replies = self.replies.clone();
        self.api.get(api::CELL_TASK_SAMPLE, move |task: Result<TaskProgram, String>| {
            replies.lock().unwrap().push(match task {
                Ok(task) => Reply::Sample(task),
                Err(err) => Reply::Failed(err),
            });
        });
    }

    fn save(&self) {
        self.put(TaskProgram {
            name: self.name.trim().to_owned(),
            text: self.text.clone(),
        });
    }

    fn put(&self, task: TaskProgram) {
        let replies = self.replies.clone();
        let name = task.name.clone();
        self.api.put_json(api::CELL_TASK, &task, move |listing: Result<PlansInfo, String>| {
            replies.lock().unwrap().push(match listing {
                Ok(listing) => Reply::Saved(name, listing),
                Err(err) => Reply::Failed(err),
            });
        });
    }

    fn apply_replies(&mut self) {
        let replies = std::mem::take(&mut *self.replies.lock().unwrap());
        for reply in replies {
            match reply {
                Reply::Loaded(task) => {
                    self.status = Some(Ok(format!("Loaded {}.", task.name)));
                    self.name = task.name;
                    self.saved = Some(task.text.clone());
                    self.text = task.text;
                }
                Reply::Sample(task) => {
                    self.status = Some(Ok("Sample loaded: edit it, then Save.".to_owned()));
                    if self.name.is_empty() {
                        self.name = task.name;
                    }
                    self.text = task.text;
                }
                Reply::Saved(name, listing) => {
                    self.status = Some(Ok(format!("Saved {name}.")));
                    self.saved = Some(self.text.clone());
                    self.saved_listing = Some(listing);
                }
                Reply::Failed(err) => self.status = Some(Err(err)),
            }
        }
    }

    /// The pane. `plans` is the active cell's listing (its task programs). Returns what the
    /// user asked for elsewhere (Open Product & Task).
    /// Replies, another active cell, a task program to open: before drawing (either half).
    fn prepare(&mut self, plans: &PlansInfo) {
        self.apply_replies();
        if plans.cell != self.cell {
            // Another cell: start over with its first task program, if any.
            self.cell.clone_from(&plans.cell);
            self.open(None);
            self.open_requested = self.open_requested.take().or_else(|| plans.tasks.first().cloned());
        }
        if let Some(name) = self.open_requested.take() {
            self.load(&name);
        }
    }

    /// Whether the pane has an action row: only with an active cell.
    pub fn has_actions(plans: &PlansInfo) -> bool {
        !plans.cell.is_empty()
    }

    /// The pane's main actions, pinned to its bottom ([`theme::action_row`]): Save, Revert and
    /// Load sample, or, for a task program made from a product, Make an editable copy.
    pub fn actions_ui(&mut self, ui: &mut egui::Ui, plans: &PlansInfo) {
        self.prepare(plans);
        let source = task_source(plans, self.name.trim()).filter(|_| self.saved.is_some());
        let read_only = read_only(source, &self.text);
        let operations = serde_json::from_str::<serde_json::Value>(&self.text)
            .ok()
            .and_then(|v| v["operations"].as_array().map(Vec::len));
        let valid_name = !self.name.trim().is_empty()
            && self.name.trim().chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
        let dirty = self.saved.as_deref() != Some(self.text.as_str());
        theme::action_buttons(ui, |ui| {
            if read_only {
                if ui
                    .button("Make an editable copy")
                    .on_hover_text("Save it under a new name, without its source")
                    .clicked()
                {
                    match editable_copy(self.name.trim(), &self.text, &plans.tasks) {
                        Ok(copy) => {
                            self.name.clone_from(&copy.name);
                            self.text.clone_from(&copy.text);
                            self.saved = None;
                            self.put(copy);
                        }
                        Err(err) => self.status = Some(Err(err)),
                    }
                }
                return;
            }
            let can_save = dirty && valid_name && operations.is_some();
            if ui
                .add_enabled(can_save, theme::primary(ui, "Save"))
                .on_hover_text("Write it into the cell; Plan Cell can then plan it")
                .clicked()
            {
                self.save();
            }
            let can_revert = dirty && self.saved.is_some();
            if ui.add_enabled(can_revert, egui::Button::new("Revert")).clicked()
                && let Some(saved) = &self.saved
            {
                self.text.clone_from(saved);
            }
            if ui
                .button("Load sample")
                .on_hover_text("12 spot welds on the top edges of a 1 m cube at the origin (the weld_cube fixture)")
                .clicked()
            {
                self.load_sample();
            }
        });
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, plans: &PlansInfo) -> Option<Action> {
        self.prepare(plans);

        let source = task_source(plans, self.name.trim()).filter(|_| self.saved.is_some());
        let mut action = None;
        ui.label(RichText::new("Task programs come from the products: edit welds in Product & Task.").color(theme::current().warn));
        if ui
            .button("Open Product & Task")
            .on_hover_text("Where the products' tasks and welds are defined; the cells using a product follow it")
            .clicked()
        {
            action = Some(product_action(source));
        }
        ui.add_space(4.0);

        if plans.cell.is_empty() {
            ui.weak("No active cell: create or open one in the Robo Fitter.");
            return action;
        }
        crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(format!("Cell {} · agentic_planner/tasks/", plans.cell)).weak()));
        let read_only = read_only(source, &self.text);

        ui.horizontal(|ui| {
            let label = if self.name.is_empty() { "New task program…".to_owned() } else { self.name.clone() };
            egui::ComboBox::from_id_salt("task_editor_pick")
                .selected_text(label)
                .width(ui.available_width() * 0.6)
                .show_ui(ui, |ui| {
                    if ui.selectable_label(false, "New task program…").clicked() {
                        self.open(None);
                    }
                    for task in &plans.tasks {
                        let label = crate::sidebar::task_label(task, task_source(plans, task));
                        if ui.selectable_label(*task == self.name, label).clicked() {
                            self.open_requested = Some(task.clone());
                        }
                    }
                });
            ui.add_enabled(
                !read_only,
                egui::TextEdit::singleline(&mut self.name).hint_text("name").desired_width(ui.available_width()),
            )
            .on_hover_text("The file name in agentic_planner/tasks/ (letters, digits, _ and -)");
        });

        let parsed = serde_json::from_str::<serde_json::Value>(&self.text);
        let operations = parsed.as_ref().ok().and_then(|v| v["operations"].as_array().map(Vec::len));
        let valid_name = !self.name.trim().is_empty()
            && self.name.trim().chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
        let dirty = self.saved.as_deref() != Some(self.text.as_str());

        if read_only {
            let from = source
                .and_then(|s| s.product.as_deref())
                .map_or_else(|| "a product".to_owned(), |product| format!("product {product}"));
            ui.small(
                RichText::new(format!(
                    "Read-only: made from {from}. Edit its welds in Product & Task, or make an editable copy \
                     (a hand-written task program the product no longer updates)."
                ))
                .color(theme::current().warn),
            );
        }
        // Make an editable copy, or Save, Revert and Load sample: the pane's action row (`actions_ui`).

        match (&parsed, operations) {
            _ if self.text.trim().is_empty() => {
                ui.small(RichText::new("Empty: load the sample to start from.").weak());
            }
            (Err(err), _) => {
                ui.small(RichText::new(format!("✗ JSON: {err}")).color(theme::current().danger));
            }
            (Ok(_), None) => {
                ui.small(RichText::new("✗ A task program needs an \"operations\" list.").color(theme::current().danger));
            }
            (Ok(_), Some(n)) => {
                let unsaved = if dirty { " · unsaved" } else { "" };
                ui.small(
                    RichText::new(format!("✓ {n} operations{unsaved}")).color(if dirty { theme::current().warn } else { theme::current().ok }),
                );
            }
        }
        if !valid_name && !self.text.trim().is_empty() {
            ui.small(RichText::new("Give it a name (letters, digits, _ and -).").color(theme::current().warn));
        }
        match &self.status {
            Some(Ok(text)) => {
                ui.small(RichText::new(text).weak());
            }
            Some(Err(err)) => {
                ui.small(RichText::new(err).color(theme::current().danger));
            }
            None => {}
        }

        egui::CollapsingHeader::new("Format").default_open(self.text.trim().is_empty()).show(ui, |ui| {
            ui.small(
                "Each operation: \"id\", \"type\" (e.g. \"weld\"), \"position\" [x, y, z] in metres in the cell's \
                 world frame, \"approach_vector\" (the tool's approach direction), \"orientation\" / \
                 \"weld_orientation\" (quaternions x, y, z, w), \"sequence_group\" and \"priority\". \
                 \"extensions\" holds the strategy (group order, concurrent robots), cycle-time targets and \
                 weld parameters. The planner allocates the operations to the robots.",
            );
        });

        egui::ScrollArea::vertical().id_salt("task_editor_text").show(ui, |ui| {
            ui.add(
                egui::TextEdit::multiline(&mut self.text)
                    .interactive(!read_only)
                    .code_editor()
                    .desired_rows(18)
                    .desired_width(f32::INFINITY)
                    .hint_text("{ \"program\": …, \"operations\": [ … ] }"),
            );
        });
        action
    }
}

/// "Open Product & Task" from the editor: on the program's product and task when it has one.
fn product_action(source: Option<&TaskSource>) -> Action {
    match source.and_then(|s| s.product.clone()) {
        Some(product) => Action::EditProductTask { product, task: source.and_then(|s| s.task.clone()) },
        None => Action::OpenProduct,
    }
}

/// A task program made from a product is read-only here: the backend says so (`source`), or its
/// JSON names the product it came from (a top-level `"source"`).
pub(crate) fn read_only(source: Option<&TaskSource>, text: &str) -> bool {
    source.is_some_and(|s| s.product.is_some())
        || serde_json::from_str::<serde_json::Value>(text).is_ok_and(|v| v.get("source").is_some())
}

/// A hand-written copy of task program `name`: its JSON without `source` (so the backend neither
/// updates nor replaces it), under a name not in `taken` and without the derived files' `__`.
pub(crate) fn editable_copy(name: &str, text: &str, taken: &[String]) -> Result<TaskProgram, String> {
    let mut json = serde_json::from_str::<serde_json::Value>(text).map_err(|err| format!("✗ JSON: {err}"))?;
    let object = json.as_object_mut().ok_or("✗ A task program is a JSON object.")?;
    object.remove("source");
    let mut base = name.to_owned();
    while base.contains("__") {
        base = base.replace("__", "_");
    }
    let base = format!("{}_copy", if base.is_empty() { "task" } else { &base });
    let copy = std::iter::once(base.clone())
        .chain((2..).map(|n| format!("{base}{n}")))
        .find(|candidate| !taken.contains(candidate))
        .unwrap_or(base);
    if object.contains_key("program") {
        object.insert("program".to_owned(), serde_json::Value::String(copy.clone()));
    }
    let text = serde_json::to_string_pretty(&json).map_err(|err| err.to_string())?;
    Ok(TaskProgram { name: copy, text })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn derived() -> TaskSource {
        TaskSource {
            file: "cube__spot_weld_1".to_owned(),
            product: Some("bug_cube".to_owned()),
            task: Some("spot_weld_1".to_owned()),
            ..TaskSource::default()
        }
    }

    #[test]
    fn derived_programs_are_read_only() {
        let plain = r#"{"operations": []}"#;
        let sourced = r#"{"source": {"product": "bug_cube"}, "operations": []}"#;
        assert!(read_only(Some(&derived()), plain));
        assert!(read_only(None, sourced));
        assert!(!read_only(None, plain));
        assert!(!read_only(Some(&TaskSource { file: "4welds".to_owned(), ..TaskSource::default() }), plain));
        assert!(!read_only(None, "not json"));
    }

    #[test]
    fn an_editable_copy_drops_the_source() {
        let text = r#"{"program": "cube__spot_weld_1", "source": {"product": "bug_cube"}, "operations": [{"id": "W01"}]}"#;
        let taken = vec!["cube__spot_weld_1".to_owned(), "cube_spot_weld_1_copy".to_owned()];
        let copy = editable_copy("cube__spot_weld_1", text, &taken).unwrap();
        assert_eq!(copy.name, "cube_spot_weld_1_copy2");
        let json: serde_json::Value = serde_json::from_str(&copy.text).unwrap();
        assert!(json.get("source").is_none());
        assert_eq!(json["program"], "cube_spot_weld_1_copy2");
        assert_eq!(json["operations"][0]["id"], "W01");
        assert!(!read_only(None, &copy.text));
        assert!(editable_copy("x", "[1]", &[]).is_err());
    }

    #[test]
    fn open_product_goes_to_the_programs_product() {
        assert_eq!(
            product_action(Some(&derived())),
            Action::EditProductTask { product: "bug_cube".to_owned(), task: Some("spot_weld_1".to_owned()) }
        );
        assert_eq!(product_action(None), Action::OpenProduct);
    }
}
