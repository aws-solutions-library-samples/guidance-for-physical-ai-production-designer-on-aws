//! The "Behaviour Tree" pane: the plan the cell-runner plays, as bt-view's Groot-style tree, in a
//! bottom panel while a plan is loaded (`trajectory.loaded`).
//!
//! It has two sources of status. bt-view's own interpreter runs the plan with cell-runner's
//! semantics (fetched whole, blackboard included, from [`api::CELL_PLAN`]), and the cell's state
//! says which operation each robot runs now.
//! * **Live** (the default): each cell state moves the interpreter to the cell's `t` (from the
//!   start again when `t` goes back or a new loop cycle starts), and the leaves the robots run get
//!   bt-view's live ring; the panel lists any drift between the two.
//! * **Inspect**: entered when the user scrubs, steps, runs or selects a node (or with the
//!   toggle). The interpreter is the user's then: seeking and stepping are local, and so are
//!   forces, breakpoints and edits. The live ring still shows the cell. "Follow cell" goes back.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use bt_view::{Action, BtPanel, Uid};
use factoryos_client::Api;
use factoryos_client::protocol::api;
use serde_json::{Value, json};

use crate::theme;
use crate::backend::Shared;
use crate::rr::egui::{self, Color32, RichText};
use crate::rr::re_ui::UiExt as _;

/// bt-view's live ring.
const LIVE: Color32 = Color32::from_rgb(0, 200, 235);
/// The cell's `t` is rounded to the millisecond: closer than this is the same instant.
const SAME_T: f64 = 1e-3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// The interpreter follows the cell.
    Live,
    /// The interpreter is the user's.
    Inspect,
}

/// A plan's name and what fetching it gave.
type Fetched = (String, Result<Value, String>);

pub struct BtPane {
    api: Api,
    panel: BtPanel,
    /// Shown, or folded to its title bar.
    pub open: bool,
    mode: Mode,
    /// The plan the panel shows or is fetching.
    plan: Option<String>,
    /// [`Shared::plan_loads`] when it was fetched: a new load of the same plan fetches it again.
    plan_load: u64,
    /// The fetched plan, handed over by the HTTP callback.
    fetched: Arc<Mutex<Option<Fetched>>>,
    /// The cell's `(cycle, t)` the interpreter was last moved to.
    followed: Option<(u64, f64)>,
    /// "Watch in Isaac Sim" in the title bar.
    isaac: Rc<RefCell<crate::isaac_ui::IsaacSim>>,
}

impl BtPane {
    pub fn new(api: Api, isaac: Rc<RefCell<crate::isaac_ui::IsaacSim>>) -> Self {
        let mut panel = BtPanel::new("factory_os_bt_pane");
        // cell-runner releases a gate nothing will open (with a warning) instead of halting.
        panel.set_auto_release(true);
        Self {
            api,
            panel,
            open: true,
            mode: Mode::Live,
            plan: None,
            plan_load: 0,
            fetched: Arc::default(),
            followed: None,
            isaac,
        }
    }

    /// Whether the cell plays a plan, i.e. the pane is drawn at all.
    pub fn wanted(shared: &Shared) -> bool {
        shared.cell_state.as_ref().is_some_and(|state| state["trajectory"]["loaded"] == true)
    }

    /// Each frame, before [`tree_ui`](Self::tree_ui) and [`bottom_ui`](Self::bottom_ui): fetch
    /// the plan the cell plays and, while open, move the tree to the cell's time. Call only while
    /// [`wanted`](Self::wanted).
    pub fn prepare(&mut self, ctx: &egui::Context, shared: &Shared) {
        let trajectory = trajectory(shared);
        self.fetch(ctx, &trajectory, shared);
        self.receive();
        if self.open {
            self.follow(&trajectory);
        }
    }

    /// The tree, its toolbar and inspector, in the right-hand panel (while [`open`](Self::open)).
    pub fn tree_ui(&mut self, ui: &mut egui::Ui) {
        ui.panel_content(|ui| {
            ui.panel_title_bar("Behaviour Tree", Some("The plan the cell plays, as a tree: live, or inspected on its own"));
        });
        self.panel.tree_ui(ui);
    }

    /// The bottom panel: title bar, then (while open) the cell's timeline. After
    /// [`tree_ui`](Self::tree_ui) in the frame.
    pub fn bottom_ui(&mut self, ui: &mut egui::Ui, shared: &mut Shared) {
        let trajectory = trajectory(shared);
        self.title_bar(ui, &trajectory, shared);
        if !self.open {
            return;
        }
        ui.separator();
        self.panel.cell_ui(ui);
        if self.mode == Mode::Live && takes_over(&self.panel) {
            self.mode = Mode::Inspect;
        }
    }

    /// Fetch the plan the cell plays when it is another one, or was loaded again.
    fn fetch(&mut self, ctx: &egui::Context, trajectory: &Value, shared: &Shared) {
        let last_loaded = match &shared.plan_result {
            Some(Ok(tree)) => Some(tree.as_str()),
            _ => None,
        };
        let Some(name) = plan_name(trajectory, last_loaded) else {
            return;
        };
        if self.plan.as_deref() == Some(name.as_str()) && self.plan_load == shared.plan_loads {
            return;
        }
        self.plan = Some(name.clone());
        self.plan_load = shared.plan_loads;
        let (fetched, ctx) = (self.fetched.clone(), ctx.clone());
        self.api.get(&api::cell_plan(&name), move |plan: Result<Value, String>| {
            *fetched.lock().unwrap() = Some((name, plan));
            ctx.request_repaint();
        });
    }

    /// Load a fetched plan into the panel, which then follows the cell from its start.
    fn receive(&mut self) {
        let Some((name, plan)) = self.fetched.lock().unwrap().take() else {
            return;
        };
        if self.plan.as_deref() != Some(name.as_str()) {
            return; // another plan was asked for since
        }
        let loaded = plan.and_then(bt_view::bt_cell::plan::parse_plan_value).and_then(|plan| self.panel.load_plan(&name, plan));
        match loaded {
            Ok(()) => {
                self.mode = Mode::Live;
                self.followed = None;
            }
            Err(err) => self.panel.set_error(format!("{name}: {err}")),
        }
    }

    fn title_bar(&mut self, ui: &mut egui::Ui, trajectory: &Value, shared: &mut Shared) {
        ui.horizontal(|ui| {
            let toggle = if self.open { "Hide" } else { "Show" };
            if ui.small_button(toggle).on_hover_text(if self.open { "Hide the tree" } else { "Show the tree" }).clicked() {
                self.open = !self.open;
            }
            ui.strong("Behaviour Tree");
            if let Some(plan) = &self.plan {
                ui.weak(plan);
            }
            // Isaac Sim's own view of the cell playing this tree; why it is off on hover (crate::isaac_ui).
            self.isaac.borrow_mut().ui(ui, false, |ui, label, enabled| ui.add_enabled(enabled, egui::Button::new(label).small()));
            if !self.open {
                return;
            }
            ui.separator();
            let live = self.mode == Mode::Live;
            // Live / Inspect (the tree's debugger): Advanced only, unless already inspecting.
            if live && !crate::ui_mode::is_advanced(ui) {
                ui.weak(format!("t = {:.2} s", trajectory["t"].as_f64().unwrap_or(0.0)));
                return;
            }
            if ui
                .add(crate::theme::segment(live, RichText::new("● Live").color(LIVE)))
                .on_hover_text("The interpreter follows the cell's time; the cell's robots' leaves get a live ring")
                .clicked()
                && !live
            {
                self.follow_cell();
            }
            if ui
                .add(crate::theme::segment(!live, "Inspect"))
                .on_hover_text("Run, step and scrub the interpreter on its own (the cell plays on)")
                .clicked()
            {
                self.mode = Mode::Inspect;
            }
            if live {
                ui.weak(format!("t = {:.2} s", trajectory["t"].as_f64().unwrap_or(0.0)));
                return;
            }
            if ui
                .button("Follow cell")
                .on_hover_text("Back to the cell's time; drops local forces, breakpoints and edits")
                .clicked()
            {
                self.follow_cell();
            }
            let shown = self.panel.shown_time();
            if ui
                .button(format!("Seek the cell to {shown:.2} s"))
                .on_hover_text("Move the cell-runner's playback to the instant the tree shows")
                .clicked()
            {
                shared.command("trajectory_seek", Some(json!({ "t": shown })));
            }
            ui.label(RichText::new("local only: steps, forces, breakpoints and edits change this tree, not the cell").small().color(theme::current().warn));
        });
    }

    /// Back to live: the interpreter starts over, without the user's forces, breakpoints and
    /// edits, and follows the cell again from the next frame.
    fn follow_cell(&mut self) {
        self.mode = Mode::Live;
        self.followed = None;
        self.panel.apply([
            Action::SetRunning(false),
            Action::ClearAllForces,
            Action::ClearBreakpoints,
            Action::RevertAllEdits,
            Action::Reset,
        ]);
    }

    /// Show what the cell runs, and in live mode move the interpreter to the cell's time.
    fn follow(&mut self, trajectory: &Value) {
        let Some(ex) = self.panel.executor() else {
            return;
        };
        let (now, done) = (ex.now(), ex.is_done());
        let active = external_active(&self.panel, trajectory);
        let mut actions = Vec::new();
        if self.panel.external_active() != Some(active.as_slice()) {
            actions.push(Action::SetExternalActive(active));
        }
        if self.mode == Mode::Live {
            let cycle = trajectory["cycle"].as_u64().unwrap_or(0);
            let t = trajectory["t"].as_f64().unwrap_or(0.0);
            if self.followed != Some((cycle, t)) {
                actions.extend(follow_actions(now, done, self.followed, cycle, t));
                self.followed = Some((cycle, t));
            }
        }
        self.panel.apply(actions);
    }
}

/// The cell state's `trajectory` block (null when there is none).
fn trajectory(shared: &Shared) -> Value {
    shared.cell_state.as_ref().map(|state| state["trajectory"].clone()).unwrap_or_default()
}

/// Whether the user took the interpreter over in the frame just drawn: scrubbed, ran it, or asked
/// for anything but looking around (bringing a node into view, folding a SubTree).
fn takes_over(panel: &BtPanel) -> bool {
    !panel.is_live()
        || panel.is_running()
        || panel.user_actions().iter().any(|action| {
            !matches!(
                action,
                Action::Focus(_) | Action::ToggleCollapse(_) | Action::Select(None) | Action::DismissError | Action::RequestOpen
            )
        })
}

/// The plan the cell plays, by its file name in the cell's plans folder without `.json`: from the
/// cell's `plan` path, else the viewer's last load, else the plan's `tree_id` (an inline plan).
pub(crate) fn plan_name(trajectory: &Value, last_loaded: Option<&str>) -> Option<String> {
    let from_path = trajectory["plan"].as_str().and_then(|path| path.rsplit('/').next()).map(|file| file.trim_end_matches(".json"));
    from_path
        .or(last_loaded)
        .or_else(|| trajectory["tree_id"].as_str())
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
}

/// The leaves the cell's robots run now, by the operation each reports (none while idle).
///
/// A robot's `robot_id` is the plan's `robot_ns`: cell-runner places each motion under the leaf's
/// `robot_ns` and reports a robot's motion only when its id is that name.
fn external_active(panel: &BtPanel, trajectory: &Value) -> Vec<(Uid, String)> {
    let mut active = Vec::new();
    for robot in trajectory["robots"].as_array().into_iter().flatten() {
        let (Some(robot_id), Some(op_id)) = (robot["robot_id"].as_str(), robot["op_id"].as_str()) else {
            continue;
        };
        if robot["phase"] == "idle" {
            continue;
        }
        for uid in panel.uid_for_operation(robot_id, op_id) {
            active.push((uid, robot_id.to_owned()));
        }
    }
    active
}

/// What moves an interpreter at `now` (finished: `done`) to the cell's instant `t` of loop `cycle`,
/// after it last followed `followed`: from the start when the cell went back or looped, then up
/// to `t`. A finished run is left alone (running it to `t` would start it over).
fn follow_actions(now: f64, done: bool, followed: Option<(u64, f64)>, cycle: u64, t: f64) -> Vec<Action> {
    let restart = followed.is_some_and(|(last_cycle, _)| last_cycle != cycle) || t + SAME_T < now;
    let (now, done) = if restart { (0.0, false) } else { (now, done) };
    let mut actions = Vec::new();
    if restart {
        actions.push(Action::Reset);
    }
    if t > now + SAME_T && !done {
        actions.push(Action::RunToTime(t));
    }
    actions
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plan_is_named_after_its_file() {
        let state = json!({ "plan": "agentic_planner/plans/bt_1_cell.json", "tree_id": "other" });
        assert_eq!(plan_name(&state, Some("bt_2_cell")).as_deref(), Some("bt_1_cell"));
        // An inline plan (a cell folder that does not hold the plans).
        let state = json!({ "plan": null, "tree_id": "bt_3_cell" });
        assert_eq!(plan_name(&state, Some("bt_2_cell")).as_deref(), Some("bt_2_cell"));
        assert_eq!(plan_name(&state, None).as_deref(), Some("bt_3_cell"));
        assert_eq!(plan_name(&json!({}), None), None);
    }

    #[test]
    fn following_runs_forward_and_starts_over_when_the_cell_does() {
        let run = |t| vec![Action::RunToTime(t)];
        assert_eq!(follow_actions(0.0, false, None, 0, 1.5), run(1.5));
        assert_eq!(follow_actions(1.5, false, Some((0, 1.5)), 0, 2.0), run(2.0));
        assert_eq!(follow_actions(2.0, false, Some((0, 2.0)), 0, 2.0), vec![], "same instant");
        // The cell was stopped or seeked back, or looped.
        assert_eq!(follow_actions(2.0, false, Some((0, 2.0)), 0, 0.5), vec![Action::Reset, Action::RunToTime(0.5)]);
        assert_eq!(follow_actions(9.0, true, Some((0, 9.0)), 1, 0.2), vec![Action::Reset, Action::RunToTime(0.2)]);
        assert_eq!(follow_actions(2.0, false, Some((0, 2.0)), 0, 0.0), vec![Action::Reset]);
        // Finished before the cell's end: left alone, not started over.
        assert_eq!(follow_actions(8.9, true, Some((0, 8.9)), 0, 9.0), vec![]);
    }

    #[test]
    fn the_cell_robots_light_their_leaves() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../plan2cell/test_data/bt_1790934549_welding_cell_2_small.json");
        let mut panel = BtPanel::new("test");
        panel.load_text("welding", &std::fs::read_to_string(path).unwrap()).unwrap();
        let trajectory = json!({ "robots": [
            { "robot_id": "robot1", "phase": "weld", "op_id": "W01" },
            { "robot_id": "robot2", "phase": "hold", "op_id": "W13" },
            { "robot_id": "robot3", "phase": "idle", "op_id": "W02" },
            { "robot_id": "robot1", "phase": "idle", "op_id": null },
        ]});
        let active = external_active(&panel, &trajectory);
        let names: Vec<_> = active.iter().map(|(uid, robot)| (panel.executor().unwrap().tree().node(*uid).name.as_str(), robot.as_str())).collect();
        assert_eq!(names, [("weld_robot1_W01", "robot1"), ("weld_robot2_W13", "robot2")]);
    }
}
