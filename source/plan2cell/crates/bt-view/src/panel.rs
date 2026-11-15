//! [`BtPanel`]: toolbar, tree canvas, timeline and inspector around one [`Executor`].
//!
//! Loading builds the document, the executor and the blackboard before anything changes, so a
//! file that fails to load leaves the panel showing what it showed. A loaded tree whose cell
//! leaves carry trajectories (a factoryos plan) runs them natively ([`CellMode`]). Running it
//! (Step, Run to wait, Run to t, real time) is in the `run` child module; the `toolbar`, the
//! tree `view` and the `timeline` with its halt banner have one module each. While the cell
//! leaves run natively, the cell panel ([`cell_panel`](crate::cell_panel)) sits between the
//! canvas and the timeline.
//!
//! Whatever the user asks for while a frame is drawn is collected as [`Action`]s and applied
//! once the frame is done ([`BtPanel::apply`]): first what the cell panel, the inspector and
//! the canvas ask for, then the toolbar's and the timeline's runs and decisions, so a value
//! typed in the inspector and committed by the click that asks for a run is in effect when the
//! run starts.
//!
//! An embedder that also watches the real cell (factoryos' viewer) tells the panel which leaves
//! the cell runs now ([`Action::SetExternalActive`], found with [`BtPanel::uid_for_operation`]):
//! they get a ring of their own, the interpreter's statuses stay its own, and [`BtPanel::drift`]
//! lists where the two disagree.

use std::collections::{BTreeMap, HashMap, HashSet};

use bt_cell::compat::{self, CompatIssue};
use bt_cell::{BtPlan, CellLeaves, CellOptions, HaltedMotion, ParsedPlan};
use bt_core::layout::Orientation;
use bt_core::{Document, NodeKind, NodeStatus, Uid};
use bt_exec::{Executor, LeafPolicy, Override, Stop};
use egui::{Rect, Vec2};
use serde_json::Value;

use crate::edit::Drafts;
use crate::{cell_panel, inspector, style};

mod run;
mod timeline;
mod toolbar;
mod view;

use run::RunMode;

/// Something the user can ask the panel for. The panel collects them while it draws and
/// applies them once the frame is done; an embedder can [`apply`](BtPanel::apply) them too (from
/// a menu, a shortcut or another view of the same run).
///
/// Decisions about the run (forcing, policies, releasing, resuming) apply at the live instant:
/// a panel scrubbed into the past goes back to live first.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// Select a node, or nothing.
    Select(Option<Uid>),
    /// Select a node and bring it into view, opening its collapsed ancestors.
    Focus(Uid),
    /// Collapse or expand a SubTree.
    ToggleCollapse(Uid),
    /// Force a node to finish with this status the next time it is ticked; `true` keeps
    /// forcing it (pinned). A control node or decorator halts its children.
    Force(Uid, NodeStatus, bool),
    /// Drop a node's override.
    ClearForce(Uid),
    /// Drop every override.
    ClearAllForces,
    /// How a stub leaf runs ([`LeafPolicy::Native`]: with the cell leaves' semantics).
    Policy(Uid, LeafPolicy),
    /// How every stub leaf of this kind runs (`Native`: every leaf the cell leaves handle).
    AllPolicies(NodeKind, LeafPolicy),
    /// Release every stalled gate once, the way cell-runner does (with its "never reaches"
    /// warning).
    ReleaseAllLikeCellRunner,
    /// Release stalled gates like cell-runner whenever a run stalls, instead of halting.
    SetAutoRelease(bool),
    /// Pause before a node starts, or not.
    Breakpoint(Uid, bool),
    /// Remove every breakpoint.
    ClearBreakpoints,
    /// Let every node paused at its breakpoint start.
    Resume,
    /// Let one paused node start.
    ResumeNode(Uid),
    /// Set a port's value as if the file said so (`None`: back to the file's). A port the
    /// node does not have yet is added. A rejected edit shows in the error row.
    SetPort {
        uid: Uid,
        port: String,
        value: Option<String>,
    },
    /// Set a main-blackboard entry, now and on every reset (`None`: back to the file's).
    SetBlackboard { key: String, value: Option<Value> },
    /// Revert every port and blackboard edit.
    RevertAllEdits,
    /// Show the run as it was at this instant (at or after `now`: live).
    SeekTime(f64),
    /// One event ([`BtPanel::step`]).
    Step,
    /// Tick until the tree finishes or halts ([`BtPanel::run_to_wait`]).
    RunToWait,
    /// Tick up to this instant ([`BtPanel::run_to_time`]).
    RunToTime(f64),
    /// Run in real time, or pause ([`BtPanel::set_running`]).
    SetRunning(bool),
    /// Back to the start ([`BtPanel::reset`]).
    Reset,
    /// Whether the cell leaves run natively, for the tree shown (the run starts over if that
    /// changes).
    SetCellMode(CellMode),
    /// What a tree halt does to a running motion, from the next halt on.
    SetHaltedMotion(HaltedMotion),
    /// Go on the way the last run went, after a decision where it stopped.
    ContinueRun,
    /// Hide the error row.
    DismissError,
    /// Ask the embedder for a file to open ([`BtPanel::take_open_request`]).
    RequestOpen,
    /// The leaves an external source (the live cell) runs now, each with the robot it reported
    /// it for: drawn with a ring of their own, without touching the interpreter's statuses, and
    /// compared with them ([`BtPanel::drift`]). Leaves not in the tree are dropped.
    SetExternalActive(Vec<(Uid, String)>),
}

impl Action {
    /// The node it is about, if any.
    fn uid(&self) -> Option<Uid> {
        match self {
            Self::Select(uid) => *uid,
            Self::Focus(uid)
            | Self::ToggleCollapse(uid)
            | Self::Force(uid, ..)
            | Self::ClearForce(uid)
            | Self::Policy(uid, _)
            | Self::Breakpoint(uid, _)
            | Self::ResumeNode(uid)
            | Self::SetPort { uid, .. } => Some(*uid),
            _ => None,
        }
    }

    /// A decision about the live run, taken at `now` even while the past is shown.
    fn decides(&self) -> bool {
        matches!(
            self,
            Self::Force(..)
                | Self::ClearForce(_)
                | Self::ClearAllForces
                | Self::Policy(..)
                | Self::AllPolicies(..)
                | Self::ReleaseAllLikeCellRunner
                | Self::Resume
                | Self::ResumeNode(_)
        )
    }
}

/// Whether the cell leaves (`ExecuteTrajectory`, `MoveHome`, `WaitForRobot`, `CheckSafety`)
/// run with cell-runner's semantics ([`bt_cell::CellLeaves`]) or as stubs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CellMode {
    /// When some cell leaf has a trajectory to follow, i.e. the tree is a factoryos plan.
    #[default]
    Auto,
    /// Whenever the tree has cell leaves.
    On,
    /// Never: every cell leaf is a stub.
    Off,
}

/// What a factoryos plan says about itself (all `None` for a bare XML file).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlanInfo {
    /// The plan's `tree_id`.
    pub tree_id: Option<String>,
    /// The cell it was planned for.
    pub cell_id: Option<String>,
    /// How many operations the planner placed.
    pub num_operations: Option<usize>,
}

/// Where the interpreter and an external source ([`Action::SetExternalActive`]) disagree about the
/// cell leaves that report motion (`ExecuteTrajectory`, `MoveHome`), in tree order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Drift {
    /// Running by the external source, not by the interpreter.
    pub external_only: Vec<Uid>,
    /// Running by the interpreter, not by the external source.
    pub interpreter_only: Vec<Uid>,
}

impl Drift {
    pub fn is_empty(&self) -> bool {
        self.external_only.is_empty() && self.interpreter_only.is_empty()
    }
}

/// A Groot2-style view of one behaviour tree that can also run it.
///
/// Embed it with [`BtPanel::ui`] in any `egui::Ui` (it lays out its own toolbar, timeline and
/// inspector inside), and feed it with [`BtPanel::load_text`] or [`BtPanel::load_plan`].
pub struct BtPanel {
    pub(crate) id: egui::Id,
    pub(crate) source: String,
    pub(crate) document: Option<Document>,
    pub(crate) executor: Option<Executor>,
    pub(crate) blackboard: BTreeMap<String, Value>,
    /// What the loaded plan says about itself.
    pub(crate) plan: Option<PlanInfo>,
    /// What cell-runner would refuse in the loaded file around its trees.
    pub(crate) file_issue: Option<CompatIssue>,
    /// The last load error (the panel keeps showing what it showed before), or a rejected edit.
    pub(crate) error: Option<String>,
    /// `error` is about the tree that is shown (a rejected edit, or a note), not a failed load
    /// that kept it.
    pub(crate) error_is_note: bool,
    pub(crate) orientation: Orientation,
    pub(crate) collapsed: HashSet<Uid>,
    pub(crate) selected: Option<Uid>,
    scene_rect: Rect,
    /// The canvas size `scene_rect` was last shown on: when the canvas changes size (a footer
    /// row comes or goes, the pane is resized) the view keeps its zoom and top-left corner
    /// ([`view::keep_zoom`]) rather than egui's Scene fitting the same rect into it again.
    view_outer: Vec2,
    /// The view Fit chose, while nothing has moved it since: it goes on framing the whole tree.
    fitted: Option<Rect>,
    fit: bool,
    /// Keep the leaves running at the instant shown in view, readable.
    pub(crate) follow: bool,
    /// The running leaves last brought into view (only a change moves the view again).
    followed: Vec<Uid>,
    /// Cards without their port rows.
    pub(crate) compact: bool,
    pub(crate) running: bool,
    pub(crate) tick_hz: f32,
    pub(crate) speed: f32,
    pub(crate) looping: bool,
    last_frame: Option<web_time::Instant>,
    accumulator: f64,
    /// Tick shown when scrubbing the timeline; `None` follows the executor.
    pub(crate) scrub: Option<u64>,
    /// Instant shown when scrubbing to a time rather than a tick (it lies in `scrub`).
    pub(crate) scrub_t: Option<f64>,
    pub(crate) show_inspector: bool,
    /// Show the cell panel (while the cell leaves run natively).
    pub(crate) show_cell: bool,
    /// Show the cell panel's list of warnings.
    pub(crate) show_cell_warnings: bool,
    pub(crate) cell_mode: CellMode,
    /// The tree has cell leaves, so [`CellMode::On`] can run them.
    pub(crate) cell_capable: bool,
    /// What a tree halt does to a running motion, for the cell leaves installed next.
    pub(crate) halted_motion: HaltedMotion,
    /// Release stalled gates like cell-runner does, instead of halting at them.
    pub(crate) auto_release: bool,
    /// Why the last run or Step stopped.
    pub(crate) last_stop: Option<Stop>,
    /// The tick that stop was at (a manual stop is shown until the tree ticks again).
    pub(crate) last_stop_tick: u64,
    /// How the last run went, so a decision at a halt can go on the same way.
    pub(crate) last_mode: RunMode,
    /// The instant "Run to t" runs to.
    pub(crate) run_to_t: f64,
    /// Text being typed into the inspector's fields.
    pub(crate) drafts: Drafts,
    /// A node to bring into view on the next frame (where a run stopped, or one the user asked
    /// for with [`Action::Focus`]).
    pub(crate) focus: Option<Uid>,
    /// Offer "Open…" on the empty canvas; it was clicked.
    pub(crate) open_button: bool,
    open_requested: bool,
    /// The motion leaves of the tree shown by `(robot_ns, operation_id)`; a `MoveHome` is
    /// `(robot_ns, "home")`, as cell-runner names its motion.
    operations: HashMap<(String, String), Vec<Uid>>,
    /// What an external source says runs now, with the robot it said it for; `None` without one.
    pub(crate) external: Option<Vec<(Uid, String)>>,
    /// What the user asked for in the last frame drawn.
    user_actions: Vec<Action>,
}

impl Default for BtPanel {
    fn default() -> Self {
        Self::new("bt_panel")
    }
}

impl BtPanel {
    /// `id_salt` keeps several panels apart.
    pub fn new(id_salt: impl std::hash::Hash + std::fmt::Debug) -> Self {
        Self {
            id: egui::Id::new(id_salt),
            source: String::new(),
            document: None,
            executor: None,
            blackboard: BTreeMap::new(),
            plan: None,
            file_issue: None,
            error: None,
            error_is_note: false,
            orientation: Orientation::Vertical,
            collapsed: HashSet::new(),
            selected: None,
            scene_rect: Rect::ZERO,
            view_outer: Vec2::ZERO,
            fitted: None,
            fit: true,
            follow: true,
            followed: Vec::new(),
            compact: false,
            running: false,
            tick_hz: 10.0,
            speed: 1.0,
            looping: false,
            last_frame: None,
            accumulator: 0.0,
            scrub: None,
            scrub_t: None,
            show_inspector: true,
            show_cell: true,
            show_cell_warnings: true,
            cell_mode: CellMode::Auto,
            cell_capable: false,
            halted_motion: HaltedMotion::Keep,
            auto_release: false,
            last_stop: None,
            last_stop_tick: 0,
            last_mode: RunMode::default(),
            run_to_t: 0.0,
            drafts: Drafts::new(),
            focus: None,
            open_button: false,
            open_requested: false,
            operations: HashMap::new(),
            external: None,
            user_actions: Vec::new(),
        }
    }

    /// Load a BehaviorTree.CPP XML file, or a factoryos plan JSON in any layout cell-runner
    /// accepts (see [`bt_cell::plan`]). `name` is shown in the toolbar.
    ///
    /// On error the panel keeps the tree it showed, and [`error`](Self::error) says what went
    /// wrong with `name`.
    pub fn load_text(&mut self, name: &str, text: &str) -> Result<(), String> {
        let plan = bt_cell::parse_plan(text).map_err(|e| self.fail(name, e))?;
        self.load_plan(name, plan)
    }

    /// Load a parsed plan and show its main tree; its cell leaves run natively if it is a cell
    /// plan ([`CellMode`]).
    ///
    /// Everything is built before anything changes: on error the panel keeps the tree it
    /// showed, with its run, and [`error`](Self::error) says what went wrong with `name`.
    pub fn load_plan(&mut self, name: &str, plan: ParsedPlan) -> Result<(), String> {
        let (document, blackboard, executor) = build(&plan.tree).map_err(|e| self.fail(name, e))?;
        self.source = name.to_string();
        self.document = Some(document);
        self.blackboard = blackboard;
        self.error = None;
        self.file_issue = compat::check_xml(&plan.tree.xml);
        self.plan = Some(PlanInfo {
            tree_id: plan.tree_id,
            cell_id: plan.cell_id,
            num_operations: plan.num_operations,
        });
        self.adopt(executor);
        Ok(())
    }

    /// Load XML with an initial blackboard; shows the main tree.
    pub fn load_xml(
        &mut self,
        name: &str,
        xml: &str,
        blackboard: BTreeMap<String, Value>,
    ) -> Result<(), String> {
        let tree = BtPlan {
            xml: xml.to_string(),
            blackboard: blackboard.into_iter().collect(),
        };
        self.load_plan(
            name,
            ParsedPlan {
                tree_id: None,
                cell_id: None,
                num_operations: None,
                tree,
            },
        )
    }

    /// Switch to another `<BehaviorTree>` of the loaded file. On error the current tree stays.
    pub fn show_tree(&mut self, tree_id: &str) -> Result<(), String> {
        let document = self.document.as_ref().ok_or("nothing loaded")?;
        match document.instantiate(tree_id) {
            Ok(tree) => {
                let executor = Executor::new(tree, self.blackboard.clone());
                self.adopt(executor);
                Ok(())
            }
            Err(e) => {
                self.set_error(e.clone());
                Err(e)
            }
        }
    }

    /// Show `executor`'s tree from the start, with the cell leaves installed as its tree calls
    /// for ([`CellMode::Auto`]: a Cell sim choice applies to the tree it was made on).
    fn adopt(&mut self, executor: Executor) {
        self.cell_capable = bt_cell::uses_cell_leaves(executor.tree());
        self.cell_mode = CellMode::Auto;
        self.executor = Some(executor);
        self.collapsed.clear();
        self.selected = None;
        self.focus = None;
        self.running = false;
        self.last_stop = None;
        self.last_mode = RunMode::default();
        self.run_to_t = 0.0;
        self.drafts.clear();
        self.go_live();
        self.fit = true;
        self.followed.clear();
        self.install_cell();
        self.operations = self
            .executor
            .as_ref()
            .map(operation_index)
            .unwrap_or_default();
        // About the tree shown before.
        self.external = None;
    }

    /// Note a load error; the panel keeps showing what it showed (or names `name` if nothing).
    fn fail(&mut self, name: &str, e: String) -> String {
        if self.executor.is_none() {
            self.source = name.to_string();
        }
        self.error = Some(format!("{name}: {e}"));
        self.error_is_note = false;
        e
    }

    /// Show an error, e.g. a file the embedder could not read. Over a loaded tree it says that
    /// tree is still showing.
    pub fn set_error(&mut self, message: impl Into<String>) {
        self.error = Some(message.into());
        self.error_is_note = false;
    }

    /// Show a problem with the tree that is shown, e.g. what it loaded without. It goes in the
    /// error row like [`set_error`](Self::set_error)'s, but does not say an older tree is still
    /// showing.
    pub fn set_note(&mut self, message: impl Into<String>) {
        self.error = Some(message.into());
        self.error_is_note = true;
    }

    /// The last error, until the next successful load (or the user dismisses it).
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    /// What the loaded plan says about itself.
    pub fn plan_info(&self) -> Option<&PlanInfo> {
        self.plan.as_ref()
    }

    /// The loaded file.
    pub fn document(&self) -> Option<&Document> {
        self.document.as_ref()
    }

    /// What cell-runner (behaviortree-rs) would refuse to run in the file and the tree shown,
    /// in the order it runs into them ([`bt_cell::compat`]); btgui runs the tree anyway.
    pub fn compat_issues(&self) -> Vec<CompatIssue> {
        let tree = self.executor.as_ref().map(|ex| compat::check(ex.tree()));
        self.file_issue
            .iter()
            .cloned()
            .chain(tree.into_iter().flatten())
            .collect()
    }

    // ───────────────────────────── cell leaves ─────────────────────────────

    /// Choose whether the cell leaves run natively, for the tree shown (loading a file or
    /// showing another tree goes back to [`CellMode::Auto`]). Installing or removing them starts
    /// the run over.
    pub fn set_cell_mode(&mut self, mode: CellMode) {
        self.cell_mode = mode;
        if self.install_cell() {
            self.running = false;
            self.last_stop = None;
            self.go_live();
        }
    }

    /// Whether the cell leaves run natively ([`set_cell_mode`](Self::set_cell_mode)).
    pub fn cell_mode(&self) -> CellMode {
        self.cell_mode
    }

    /// The cell leaves, when they run natively: the motion schedule, robot states, gates.
    pub fn cell(&self) -> Option<&CellLeaves> {
        self.executor.as_ref()?.leaf_handler::<CellLeaves>()
    }

    /// Release stalled gates like cell-runner does (with its "never reaches" warning) instead
    /// of halting at them. Off by default, so the panel shows where the tree halts.
    pub fn set_auto_release(&mut self, on: bool) {
        self.auto_release = on;
    }

    /// Install or remove the cell leaves as [`CellMode`] says; whether that changed anything
    /// (the run then starts over).
    fn install_cell(&mut self) -> bool {
        let Some(ex) = self.executor.as_mut() else {
            return false;
        };
        let on = match self.cell_mode {
            CellMode::Off => false,
            CellMode::On => self.cell_capable,
            CellMode::Auto => bt_cell::is_cell_plan(ex),
        };
        if on == ex.leaf_handler::<CellLeaves>().is_some() {
            return false;
        }
        if on {
            let options = CellOptions {
                halted_motion: self.halted_motion,
                ..CellOptions::for_executor(ex)
            };
            bt_cell::install(ex, options);
        } else {
            ex.take_leaf_handler();
        }
        true
    }

    // ───────────────────────────── an external source ─────────────────────────────

    /// The motion leaves (`ExecuteTrajectory`, `MoveHome`) of the tree shown for a motion the
    /// cell reports: by their `robot_ns` and `operation_id` ports as the tree was loaded, and
    /// `home` for a `MoveHome`. cell-runner's interlock pre-move (`<op>:interlock`) is its leaf's.
    /// Empty for a motion the tree has no leaf for; several for an operation run more than once.
    pub fn uid_for_operation(&self, robot_ns: &str, operation_id: &str) -> Vec<Uid> {
        let operation_id = operation_id
            .strip_suffix(":interlock")
            .unwrap_or(operation_id);
        self.operations
            .get(&(robot_ns.to_string(), operation_id.to_string()))
            .cloned()
            .unwrap_or_default()
    }

    /// What the external source said last ([`Action::SetExternalActive`]); `None` without one.
    pub fn external_active(&self) -> Option<&[(Uid, String)]> {
        self.external.as_deref()
    }

    /// Forget the external source: no rings, no drift.
    pub fn clear_external(&mut self) {
        self.external = None;
    }

    /// Where the interpreter's live run and the external source disagree about the motion
    /// leaves; `None` without an external source.
    pub fn drift(&self) -> Option<Drift> {
        let external = self.external.as_ref()?;
        let ex = self.executor.as_ref()?;
        let external: HashSet<Uid> = external.iter().map(|(uid, _)| *uid).collect();
        let mut motion: Vec<Uid> = self.operations.values().flatten().copied().collect();
        motion.sort_unstable();
        motion.dedup();
        let running: HashSet<Uid> = motion
            .into_iter()
            .filter(|&uid| ex.status(uid) == NodeStatus::Running)
            .collect();
        let sorted = |set: Vec<Uid>| {
            let mut set = set;
            set.sort_unstable();
            set
        };
        Some(Drift {
            external_only: sorted(external.difference(&running).copied().collect()),
            interpreter_only: sorted(running.difference(&external).copied().collect()),
        })
    }

    // ───────────────────────────── embedding ─────────────────────────────

    /// Offer an "Open…" button on the empty canvas. bt-view cannot open a file dialog itself:
    /// the embedder asks [`take_open_request`](Self::take_open_request) each frame.
    pub fn set_open_button(&mut self, on: bool) {
        self.open_button = on;
    }

    /// Whether "Open…" was clicked since the last call.
    pub fn take_open_request(&mut self) -> bool {
        std::mem::take(&mut self.open_requested)
    }

    pub fn executor(&self) -> Option<&Executor> {
        self.executor.as_ref()
    }

    pub fn executor_mut(&mut self) -> Option<&mut Executor> {
        self.executor.as_mut()
    }

    /// The selected node, if any.
    pub fn selected(&self) -> Option<Uid> {
        self.selected
    }

    /// Select a node (what clicking it does), e.g. to follow a selection made elsewhere.
    pub fn select(&mut self, uid: Option<Uid>) {
        self.selected = uid.filter(|uid| {
            self.executor
                .as_ref()
                .is_some_and(|ex| *uid < ex.tree().len())
        });
    }

    /// Select the first node with this instance name.
    pub fn select_by_name(&mut self, name: &str) -> Option<Uid> {
        let uid = self
            .executor
            .as_ref()?
            .tree()
            .nodes
            .iter()
            .find(|n| n.name == name)?
            .uid;
        self.selected = Some(uid);
        Some(uid)
    }

    /// Keep the leaves running at the instant shown in view, zoomed in to read them whenever the
    /// view is too small to (on by default). Each new set of running leaves moves the view;
    /// in between, pan and zoom are the user's.
    pub fn set_follow(&mut self, on: bool) {
        if on && !self.follow {
            self.followed.clear();
        }
        self.follow = on;
    }

    /// Draw the cards without their port rows (they stay in the tooltip and the inspector).
    pub fn set_compact(&mut self, on: bool) {
        if on != self.compact {
            self.compact = on;
            self.fit = true;
        }
    }

    /// Hide or show a SubTree's children.
    pub fn collapse(&mut self, uid: Uid, collapsed: bool) {
        if collapsed {
            self.collapsed.insert(uid);
        } else {
            self.collapsed.remove(&uid);
        }
        self.fit = true;
    }

    pub fn tree_id(&self) -> Option<&str> {
        self.executor.as_ref().map(|e| e.tree().tree_id.as_str())
    }

    /// What the user asked for in the last frame [`ui`](Self::ui) drew (before it was applied),
    /// e.g. for an embedder that stops driving the panel once the user takes over. Scrubbing the
    /// timeline is not an action: see [`is_live`](Self::is_live).
    pub fn user_actions(&self) -> &[Action] {
        &self.user_actions
    }

    /// Whether the panel shows the executor's latest instant (not one scrubbed to).
    pub fn is_live(&self) -> bool {
        self.scrub.is_none()
    }

    /// Draw the panel, run it in real time while it runs, and do what the user asked for.
    pub fn ui(&mut self, ui: &mut egui::Ui) {
        self.drive(ui.ctx());
        // Runs and halt decisions wait for the inspector: a click on them takes the focus from
        // a field being typed in, which commits its value while the inspector draws.
        let mut runs = Vec::new();
        egui::Panel::top(self.id.with("toolbar")).show(ui, |ui| self.toolbar(ui, &mut runs));
        egui::Panel::bottom(self.id.with("timeline")).show(ui, |ui| self.timeline(ui, &mut runs));
        let mut actions = Vec::new();
        if self.cell().is_some() {
            // The closure borrows all of `self`, so the flag the panel may flip is a copy.
            let mut show = self.show_cell;
            egui::Panel::bottom(self.id.with("cell"))
                .resizable(true)
                .default_size(210.0)
                .min_size(90.0)
                .show_collapsible(ui, &mut show, |ui| actions = cell_panel::ui(self, ui));
            self.show_cell = show;
        }
        if self.show_inspector && self.executor.is_some() {
            egui::Panel::right(self.id.with("inspector"))
                .resizable(true)
                .default_size(340.0)
                .show(ui, |ui| actions.append(&mut inspector::ui(self, ui)));
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(style::CANVAS))
            .show(ui, |ui| self.canvas(ui, &mut actions));
        actions.append(&mut runs);
        self.finish_frame(ui, actions, false);
    }

    /// The panel split in two, for an embedder that shows them apart (e.g. the tree beside the
    /// 3D view and the cell's timeline under it): the tree with its toolbar, scrubber and
    /// inspector (under the tree here, for a tall and narrow place). Call it before
    /// [`cell_ui`](Self::cell_ui) in each frame; [`user_actions`](Self::user_actions) then holds
    /// what the user asked for in both.
    pub fn tree_ui(&mut self, ui: &mut egui::Ui) {
        self.drive(ui.ctx());
        let mut runs = Vec::new();
        egui::Panel::top(self.id.with("toolbar")).show(ui, |ui| self.toolbar(ui, &mut runs));
        egui::Panel::bottom(self.id.with("timeline")).show(ui, |ui| self.timeline(ui, &mut runs));
        let mut actions = Vec::new();
        if self.show_inspector && self.executor.is_some() {
            let height = ui.available_height();
            egui::Panel::bottom(self.id.with("inspector_below"))
                .resizable(true)
                .default_size(height * 0.4)
                .min_size(80.0)
                .max_size((height - 120.0).max(80.0))
                .show(ui, |ui| actions.append(&mut inspector::ui(self, ui)));
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(style::CANVAS))
            .show(ui, |ui| self.canvas(ui, &mut actions));
        actions.append(&mut runs);
        self.finish_frame(ui, actions, false);
    }

    /// The other half of the split ([`tree_ui`](Self::tree_ui)): the cell panel (the robots and
    /// the motion as a Gantt chart), filling `ui`; nothing when there is no cell or the toolbar's
    /// "Cell" is off.
    pub fn cell_ui(&mut self, ui: &mut egui::Ui) {
        if self.cell().is_none() || !self.show_cell {
            return;
        }
        let actions = cell_panel::ui(self, ui);
        self.finish_frame(ui, actions, true);
    }

    /// Record and apply what the user asked for in a frame (`append`: to what an earlier part of
    /// the same frame recorded).
    fn finish_frame(&mut self, ui: &egui::Ui, mut actions: Vec<Action>, append: bool) {
        // An open context menu selects its node on every frame: that alone changes nothing.
        actions.retain(|action| *action != Action::Select(self.selected));
        if append {
            self.user_actions.extend(actions.iter().cloned());
        } else {
            self.user_actions.clone_from(&actions);
        }
        if !actions.is_empty() {
            // This frame shows the panel as it was before them.
            ui.ctx().request_repaint();
        }
        self.apply(actions);
    }

    /// Do what the user asked for, in order (see [`Action`]). Actions about a node that is not
    /// in the tree are ignored.
    pub fn apply(&mut self, actions: impl IntoIterator<Item = Action>) {
        for action in actions {
            self.apply_one(action);
        }
    }

    fn apply_one(&mut self, action: Action) {
        let len = self.executor.as_ref().map_or(0, |ex| ex.tree().len());
        if action.uid().is_some_and(|uid| uid >= len) {
            return;
        }
        if action.decides() {
            self.go_live();
        }
        match action {
            Action::Select(uid) => self.select(uid),
            Action::Focus(uid) => {
                self.selected = Some(uid);
                self.focus = Some(uid);
            }
            Action::ToggleCollapse(uid) => {
                if !self.collapsed.remove(&uid) {
                    self.collapsed.insert(uid);
                }
                self.fit = true;
            }
            Action::SetAutoRelease(on) => self.set_auto_release(on),
            Action::SeekTime(t) => self.scrub_to_time(Some(t)),
            Action::Step => self.step(),
            Action::RunToWait => self.run_to_wait(),
            Action::RunToTime(t) => self.run_to_time(t),
            Action::SetRunning(on) => self.set_running(on),
            Action::Reset => self.reset(),
            Action::SetCellMode(mode) => self.set_cell_mode(mode),
            Action::SetHaltedMotion(halted_motion) => {
                self.halted_motion = halted_motion;
                if let Some(cell) = self
                    .executor
                    .as_mut()
                    .and_then(|ex| ex.leaf_handler_mut::<CellLeaves>())
                {
                    cell.options_mut().halted_motion = halted_motion;
                }
            }
            Action::ContinueRun => self.continue_run(),
            Action::DismissError => self.error = None,
            Action::RequestOpen => self.open_requested = true,
            Action::SetExternalActive(mut active) => {
                active.retain(|(uid, _)| *uid < len);
                self.external = Some(active);
            }
            action => {
                let Some(ex) = &mut self.executor else {
                    return;
                };
                if let Err(e) = apply_to_executor(ex, action) {
                    self.error = Some(e);
                    self.error_is_note = true;
                }
            }
        }
    }
}

/// The actions that change the executor's run: overrides, policies, breakpoints and edits.
/// A rejected port edit is the error.
fn apply_to_executor(ex: &mut Executor, action: Action) -> Result<(), String> {
    match action {
        Action::Force(uid, status, pinned) => {
            ex.set_override(uid, Some(Override { status, pinned }));
        }
        Action::ClearForce(uid) => ex.set_override(uid, None),
        Action::ClearAllForces => ex.clear_overrides(),
        Action::Policy(uid, policy) => ex.set_policy(uid, policy),
        Action::AllPolicies(kind, policy) => ex.set_all_policies(kind, policy),
        Action::ReleaseAllLikeCellRunner => ex.release_stalled(),
        Action::Breakpoint(uid, on) => ex.set_breakpoint(uid, on),
        Action::ClearBreakpoints => ex.clear_breakpoints(),
        Action::Resume => ex.resume(),
        Action::ResumeNode(uid) => ex.resume_node(uid),
        Action::SetPort { uid, port, value } => match value {
            Some(value) => ex.set_port_value(uid, &port, &value).map_err(|e| {
                let node = &ex.tree().node(uid).name;
                format!("{node}.{port} = {value}: {e}")
            })?,
            None => ex.revert_port_value(uid, &port),
        },
        Action::SetBlackboard { key, value } => match value {
            Some(value) => ex.set_blackboard_value(&key, value),
            None => ex.revert_blackboard_value(&key),
        },
        Action::RevertAllEdits => ex.revert_all_edits(),
        // The panel's own actions are applied by `apply_one`.
        _ => {}
    }
    Ok(())
}

/// A button label whose icon falls back when the bundled fonts lack the glyph.
fn label(ui: &egui::Ui, preferred: &'static str, plain: &'static str, text: &str) -> String {
    format!("{} {text}", style::glyph(ui.ctx(), preferred, plain))
}

/// The motion leaves of `ex`'s tree by `(robot_ns, operation_id)` (`home` for a `MoveHome`),
/// their ports read as the run starts (a `{key}` through the blackboard).
fn operation_index(ex: &Executor) -> HashMap<(String, String), Vec<Uid>> {
    let text = |uid: Uid, port: &str| -> Option<String> {
        match ex.port_value(uid, port)? {
            Value::String(text) => Some(text),
            Value::Null => None,
            other => Some(other.to_string()),
        }
    };
    let mut index: HashMap<(String, String), Vec<Uid>> = HashMap::new();
    for node in ex.tree().nodes.iter().filter(|n| n.kind.is_leaf()) {
        let operation = match node.id.as_str() {
            "ExecuteTrajectory" => text(node.uid, "operation_id"),
            "MoveHome" => Some("home".to_string()),
            _ => continue,
        };
        if let (Some(robot), Some(operation)) = (text(node.uid, "robot_ns"), operation) {
            index.entry((robot, operation)).or_default().push(node.uid);
        }
    }
    index
}

/// The document, blackboard and executor for `plan`'s main tree, or why there are none.
fn build(plan: &BtPlan) -> Result<(Document, BTreeMap<String, Value>, Executor), String> {
    let document = Document::parse(&plan.xml)?;
    // The main tree always exists (an unknown `main_tree_to_execute` falls back to the first
    // tree), so a file that loads at all fails in `parse`, not here.
    let tree = document.instantiate(document.main_tree_id())?;
    let blackboard = plan.initial_blackboard();
    let executor = Executor::new(tree, blackboard.clone());
    Ok((document, blackboard, executor))
}

#[cfg(test)]
mod tests {
    use super::*;

    const NESTED: &str = r#"<root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <Sequence><AlwaysSuccess name="first"/><SubTree ID="Inner" name="inner_tree"/></Sequence>
      </BehaviorTree>
      <BehaviorTree ID="Inner"><AlwaysSuccess name="inner"/></BehaviorTree>
    </root>"#;

    #[test]
    fn a_stop_opens_its_node_and_keeps_the_selection() {
        let ctx = egui::Context::default();
        let mut panel = BtPanel::new("focus");
        panel.load_text("nested.xml", NESTED).unwrap();
        let find = |panel: &BtPanel, name: &str| {
            panel
                .executor()
                .unwrap()
                .tree()
                .nodes
                .iter()
                .find(|n| n.name == name)
                .unwrap()
                .uid
        };
        let (first, subtree, inner) = (
            find(&panel, "first"),
            find(&panel, "inner_tree"),
            find(&panel, "inner"),
        );
        panel.collapse(subtree, true);
        panel.select(Some(first));
        panel.executor_mut().unwrap().set_breakpoint(inner, true);
        panel.run_to_wait();
        assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![inner])));
        assert_eq!(panel.focus, Some(inner));

        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
        });
        output.textures_delta.clear();
        assert_eq!(panel.focus, None);
        assert!(!panel.collapsed.contains(&subtree));
        assert_eq!(panel.selected(), Some(first));
    }
}
