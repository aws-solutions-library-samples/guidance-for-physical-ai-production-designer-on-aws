//! The "Designer Workflow" sidebar, mirroring the Kiro extension's `consolidated-sidebar.ts`
//! (Sept 2026 bundle): one bordered box per concern, stacked pill buttons inside, dashed
//! connectors between boxes, top-down along the pipeline: the product and its task → dress → fit
//! (and create the cell) → plan; then the guided flow to the running cell, five numbered steps
//! with a state and a one-line summary each ([`crate::flow`]): 1 Behaviour Tree (the scene to
//! simulate it in, Load & Play: the simulation, shown in the Robo Fitter's 3D view), 2 Generate
//! PLC + robot code (the export, then the Hardware Configuration of its robots), 3 Deploy to the
//! virtual cell (no start), 4 Review in the PLC IDE, the "Virtual cell" box (the workspace's own
//! cell: Create / Start / Stop / Delete; the shared one: Start / Stop cell,
//! Bring up controllers), 5 Start production (with Watch the cell: the live cell's own 3D view);
//! below them, Build training job (the cells a learned planner policy trains on, around the active
//! cell, saved as a plan: [`crate::training_ui`]).
//! The top bar's chip says which of these the 3D view shows ([`crate::view_mode`]).

use factoryos_client::protocol::{CellsInfo, PlansInfo, TaskSource};
use factoryos_client::protocol::agentic::{AgenticStatus, PrepareState, SimState};
use crate::theme;
use crate::rr::egui::{self, Color32, CornerRadius, RichText, Stroke, Vec2};

use crate::{cell_list, flow};

const BUTTON_HEIGHT: f32 = 30.0;

/// What the user clicked.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Action {
    OpenProduct,
    /// Ask the agent to do a step's work for the current product (the steps' ✦ buttons).
    Agent(AgentStep),
    OpenDresser,
    OpenFitter,
    /// Make an existing cell the active cell and show it in the Robo Fitter's 3D view.
    OpenCell(String),
    /// The linked cell's scene editable again: stop the simulation and open the scene in the Robo
    /// Fitter (its panel's "Stop simulation and edit").
    EditScene,
    ViewLogs,
    /// Step 5: show the live cell's own 3D recording (`factoryos/rrd/cell`), without the Robo
    /// Fitter's panes.
    WatchCell,
    SetActiveCell(String),
    /// A saved Robo Fitter scene without a cell folder: make (or reuse) its cell and make it the
    /// active cell; `open`: then show it in the Robo Fitter's 3D view (as [`Action::OpenCell`]).
    UseScene { scene: String, open: bool },
    /// Open the planner chat and send it the "Plan prompt" template for the active cell, planning
    /// `task` (empty: the cell has none yet) with `planner` (`auto`: the planner router's choice).
    PlanCell { task: String, planner: String },
    /// Open the Task Program editor on a task program (`None`: a new one): a hand-written one.
    EditTask(Option<String>),
    /// Open Product & Task on `product` (loading it) and its task `task`, if named: where a
    /// product-derived task program is edited.
    EditProductTask { product: String, task: Option<String> },
    RefreshCells,
    RefreshPlans,
    /// The Robo Fitter's weld spot labels on or off (`FitterView::show_labels`).
    SetWeldLabels(bool),
    /// Load the plan (a path as in `PlanInfo::path`) into the simulation's behaviour-tree executor
    /// in the scene `cell` (`None`: the active cell) and play it.
    LoadPlan { path: String, cell: Option<String> },
    /// A `trajectory_*` cell-runner verb for the loaded behaviour tree.
    Trajectory(&'static str, Option<serde_json::Value>),
    /// Open the chat in a new tab titled `title`, the agent starting on `prompt` (step 3's "Ask
    /// the agent" about the deployability check).
    AskAgent { title: String, prompt: String },
    /// Build training job: the active cell in its own 3D view, with the job's panel.
    OpenTrainingJob,
}

/// What a step's agent button (at the bottom of its right-hand pane) asks for; the prompts are in
/// the settings.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgentStep {
    /// Product & Task: the product's tasks worked out into operations, in its own frame.
    TaskDescription,
    /// Robo Dresser: the robot + tool pairs the tasks need.
    Dressing,
    /// Robo Fitter: a cell layout where every operation is reachable.
    CellLayout,
    /// Robo Fitter: check that the open scene's robots reach every weld (IK), and fix the layout
    /// in a new scene if they don't. Needs a saved scene, not a product.
    ReachCheck,
}

/// Choices in progress in the sidebar.
#[derive(Default)]
pub struct SidebarForm {
    /// The cell picked to open in the Robo Fitter; `None` = the active cell.
    pub open_cell: Option<String>,
    /// The picked plan; `None` = the newest.
    pub plan: Option<String>,
    /// The picked task program; `None` = the only/first one.
    pub task: Option<String>,
    /// The planner picked for Plan Cell; `None` = `auto` (the planner router recommends one).
    pub planner: Option<String>,
    /// The playback speed set in the panel, until the simulation's state reports it.
    pub speed: SpeedEdit,
}

/// The playback speeds the panel offers (×); the cell-runner itself accepts up to 100×.
pub const SPEED_RANGE: std::ops::RangeInclusive<f64> = 0.1..=20.0;
/// One-click playback speeds (×).
const SPEED_PRESETS: [f64; 5] = [0.5, 1.0, 2.0, 5.0, 10.0];
/// How long a set speed is shown without the simulation's state confirming it (a lost or failed
/// command): then the panel shows the simulation's speed again.
const SPEED_CONFIRM_S: f64 = 3.0;

/// The playback speed being set. The panel's speed widget shows the simulation's state, which
/// lags the command by a state poll: without this, every frame in between reset the widget to the
/// old speed (a drag snapped back, and its release sent the old speed again).
#[derive(Default)]
pub struct SpeedEdit {
    pending: Option<(f64, web_time::Instant)>,
}

impl SpeedEdit {
    /// The speed to show: the one set, until the simulation reports it (or gives up on it), else
    /// the simulation's (`reported`, 1× when it says none).
    pub fn shown(&mut self, reported: Option<f64>, now: web_time::Instant) -> f64 {
        if let Some((speed, at)) = self.pending {
            let confirmed = reported.is_some_and(|reported| (reported - speed).abs() < 1e-6);
            if confirmed || now.duration_since(at).as_secs_f64() > SPEED_CONFIRM_S {
                self.pending = None;
            }
        }
        self.pending.map_or_else(|| reported.unwrap_or(1.0), |(speed, _)| speed)
    }

    /// Set `speed` (clamped to [`SPEED_RANGE`]) and return the command that applies it.
    pub fn set(&mut self, speed: f64, now: web_time::Instant) -> Action {
        let speed = clamp_speed(speed);
        self.pending = Some((speed, now));
        Action::Trajectory("trajectory_set_speed", Some(serde_json::json!({ "speed": speed })))
    }
}

/// `speed` within [`SPEED_RANGE`] (1× for a non-number).
pub fn clamp_speed(speed: f64) -> f64 {
    if speed.is_finite() { speed.clamp(*SPEED_RANGE.start(), *SPEED_RANGE.end()) } else { 1.0 }
}

/// A speed as the panel labels it: `0.5×`, `2×`.
fn speed_label(speed: f64) -> String {
    format!("{}×", (speed * 100.0).round() / 100.0)
}

/// The planners Plan Cell offers (the planner router's, deploy/planner-router/planners.json), with
/// their hover text. `auto` lets the router recommend one and the agent ask when it is unclear.
pub const PLANNERS: [(&str, &str); 11] = [
    ("auto", "The planner router recommends one for the cell and task; the agent asks you when it is unclear"),
    ("umarell_u3", "umarell's own planning tools with the upstream fixes, driven by a deterministic orchestrator (no LLM): any task program, 15/15 welds in the validated cells, ~2-3 min here"),
    ("umarell_jam", "umarell from a cache of certified weld paths (no LLM): seconds once the cell's cache is prepared or pulled from the catalogue; plans weld and robot subsets; a cache miss falls back to umarell_u3"),
    ("umarell", "The LLM planner: any cell, any task program and free-text constraints; takes minutes"),
    ("hier_la_fu", "Learned policy (hierarchical RL, learned lookahead): the canonical program in its training cell, ~54 s, every check passes"),
    ("hier_q", "Learned policy (hierarchical RL, Q head): fastest, ~46 s in the full profile"),
    ("ppo_pure", "Learned PPO policy: ~66 s, largest clearance"),
    ("ppo_v3", "Learned PPO v3 policy: ~68 s"),
    ("ppo_v4", "Learned PPO v4 policy: ~80 s"),
    ("ppo_2t", "Learned PPO policy for the two-robot fixture cell welding_cell_2t"),
    ("trajgen", "trajgen's precomputed sample for the cell (scripted generator, instant)"),
];

/// The state the sidebar shows.
pub struct CellRunner<'a> {
    pub cells: &'a CellsInfo,
    pub error: Option<&'a str>,
    pub agentic: &'a AgenticStatus,
    pub plans: &'a PlansInfo,
    /// The simulation's `trajectory` state block (the loaded behaviour tree's playback), if it runs.
    pub trajectory: Option<&'a serde_json::Value>,
    /// A Load & Play is in flight (the backend may be starting the simulation for it).
    pub plan_pending: bool,
    pub plan_result: Option<&'a Result<String, String>>,
    /// The cell the Robo Fitter's 3D view follows.
    pub linked_cell: Option<&'a str>,
    /// The Planning Perspective waits for the simulation before the planner starts.
    pub planning_waiting: bool,
    /// The product the agent buttons work on (saved in Product & Task), if any.
    pub product: Option<&'a str>,
    /// The plan the behaviour-tree executor plays (its name), if any.
    pub loaded_plan: Option<&'a str>,
    /// The live cell's 3D stream has arrived (its recording exists): Watch the cell can show it.
    pub cell_stream: bool,
    /// The live cell's name, if one streams here.
    pub live_cell: Option<&'a str>,
    /// The head's live state (fos-live-bridge), for step 5.
    pub live_head: flow::LiveHead,
    /// The cell the simulation runs (starting or up), if any: step 1's Scene may name another.
    pub sim_cell: Option<&'a str>,
    /// The Robo Fitter's weld spots carry their text labels (`FitterView::show_labels`).
    pub weld_labels: bool,
}

pub fn ui(
    ui: &mut egui::Ui,
    cell: &CellRunner<'_>,
    form: &mut SidebarForm,
    export: &mut crate::code_export::CodeExport,
    hardware: &mut crate::hardware_ui::HardwareUi,
    infra: &mut crate::infra_ui::InfraUi,
    isaac: &mut crate::isaac_ui::IsaacSim,
) -> Option<Action> {
    let mut action = None;

    group(ui, "Product & Task", |ui| {
        if button(ui, "Open Product & Task") {
            action = Some(Action::OpenProduct);
        }
        let note = match cell.product {
            Some(product) => format!("Product: {product}"),
            None => "The part to produce and its tasks, e.g. weld spots.".to_owned(),
        };
        ui.small(RichText::new(note).weak());
    });
    connector(ui);
    group(ui, "Robo Dresser", |ui| {
        if button(ui, "Open Dresser") {
            action = Some(Action::OpenDresser);
        }
    });
    connector(ui);
    group(ui, "Robo Fitter", |ui| {
        if button(ui, "Open Fitter") {
            action = Some(Action::OpenFitter);
        }
        let mut labels = cell.weld_labels;
        if ui
            .checkbox(&mut labels, "Weld labels")
            .on_hover_text("The weld spots' names in 3D (W02~: dome estimate; W02 r1: reached by r1 in the IK check); off leaves the coloured spots")
            .changed()
        {
            action = Some(Action::SetWeldLabels(labels));
        }

        match cell.linked_cell {
            Some(linked) => {
                ui.small(
                    RichText::new(format!("Cell {linked} open (read-only: Load & Play simulates it; Generate USDA still exports it)"))
                        .color(theme::current().ok),
                );
                if ui
                    .button("Edit scene")
                    .on_hover_text(format!(
                        "Stop the simulation and open cell {linked}'s scene in the Robo Fitter for editing (saving it marks the \
                         cell's plans and exports stale)"
                    ))
                    .clicked()
                {
                    action = Some(Action::EditScene);
                }
            }
            None => {
                ui.small(RichText::new("Place the robots, then Use as cell in the Robo Fitter panel (or pick a scene below).").weak());
            }
        }
        if let Some(open) = open_cell_row(ui, cell.cells, form) {
            action = Some(open);
        }
    });
    connector(ui);
    group(ui, "Agentic Planning", |ui| {
        if let Some(agentic) = agentic_ui(ui, cell.agentic, cell.plans, form) {
            action = Some(agentic);
        }
        if cell.planning_waiting {
            ui.small(RichText::new("The planner starts once the simulation is up…").color(theme::current().warn));
        }
    });

    // The guided flow: behaviour tree → code (+ hardware) → deploy → review → production.
    connector(ui);
    let playing = cell.trajectory.is_some_and(|t| t["loaded"].as_bool() == Some(true) && t["playing"].as_bool() == Some(true));
    let scene = Some(cell.cells.active.as_str()).filter(|scene| !scene.is_empty());
    let bt = flow::bt_step(&flow::BtFacts {
        plans: cell.plans.plans.len(),
        loaded: cell.loaded_plan,
        playing,
        pending: cell.plan_pending,
        scene,
        simulated: cell.sim_cell,
    });
    step_group(ui, 1, "Behaviour Tree", &bt, |ui| {
        if let Some(executor) = executor_ui(ui, cell, form) {
            action = Some(executor);
        }
    });
    connector(ui);
    let export_facts = export.flow_facts(hardware.saved_hardware().as_ref());
    step_group(ui, 2, "Generate PLC + robot code", &flow::generate_step(&export_facts), |ui| {
        export.form_ui(ui, cell.plans, cell.loaded_plan, hardware.export_note().as_deref());
        // The Hardware Configuration of the export's robots: after the export, which says what
        // there is to configure.
        // Network addresses and station names: the plan2cell defaults apply until saved.
        crate::ui_mode::advanced(ui, |ui| {
            ui.separator();
            hardware.sidebar_ui(ui);
        });
        let summary = flow::hardware_summary(&hardware.flow_facts());
        let color = match summary.state {
            flow::StepState::Done | flow::StepState::UpToDate | flow::StepState::Waiting => Color32::GRAY,
            state => state.color(),
        };
        let line = ui.small(RichText::new(&summary.summary).color(color));
        if let Some(plan) = hardware.plan() {
            line.on_hover_text(format!("The robots of {plan} (plan2cell hardware {plan})"));
        }
    });
    // Steps 3–5 work on the export step 2 has: picked in the Control Code Export results, else the
    // newest; while step 2 is stale, none of them is done with it.
    let generated = export.deploy_choice();
    let stale = export_facts.stale();
    let status = infra.status().cloned();
    connector(ui);
    let deploy = flow::deploy_step(status.as_ref(), generated.as_ref(), stale);
    step_group(ui, 3, "Deploy to the virtual cell", &deploy, |ui| {
        infra.deploy_ui(ui, generated.clone());
        if let Some((title, prompt)) = infra.take_chat_request() {
            action = Some(Action::AskAgent { title, prompt });
        }
        if let Some(s) = status.as_ref().filter(|s| !s.all_running) {
            let why = crate::virtual_cell::blocks(s)
                .filter(|_| s.binding == Some(factoryos_client::protocol::infra::CellKind::Own))
                .unwrap_or_else(|| "The virtual cell is not running: Start virtual cell in the Virtual cell box below.".to_owned());
            ui.small(RichText::new(why).weak());
        }
    });
    connector(ui);
    let review = flow::review_step(status.as_ref(), generated.as_ref(), infra.ide_opened_s(), stale);
    step_group(ui, 4, "Review in the PLC IDE", &review, |ui| infra.review_ui(ui));
    connector(ui);
    // The running cell: its instances (Start / Stop cell, Bring up controllers), then production.
    group(ui, "Virtual cell", |ui| infra.header_ui(ui));
    connector(ui);
    let production = flow::production_step(status.as_ref(), generated.as_ref(), stale, &cell.live_head);
    step_group(ui, 5, "Start production", &production, |ui| {
        infra.production_ui(ui, generated.clone(), &cell.live_head);
        ui.separator();
        let what = match (cell.live_cell, cell.cell_stream) {
            (Some(live), _) => {
                format!("Shows the live cell {live}: its own scene as the virtual cell runs it (not the Robo Fitter's scene)")
            }
            (None, true) => "Shows the live cell: its own scene as the virtual cell runs it (not the Robo Fitter's scene)".to_owned(),
            (None, false) => "No live cell on this workspace".to_owned(),
        };
        ui.small(RichText::new(what).weak());
        if button_response(ui, "Watch the cell", cell.cell_stream)
            .on_hover_text(
                "The live cell's own 3D view (its recording, factoryos/rrd/cell), without the Robo Fitter's panes. \
                 Start production, Restart PLC runtime, Run Control Code and Start cell's chain switch to it once the \
                 cell is in production, unless another tool is open. The simulation of step 1 keeps running.",
            )
            .on_disabled_hover_text(
                "No live cell streams here (factoryos/rrd/cell). A server connected to the virtual cell (connect-cell.sh) has one.",
            )
            .clicked()
        {
            action = Some(Action::WatchCell);
        }
        // Isaac Sim's own view of the cell (its web viewer), when a stack is linked to this workspace.
        // The "no Isaac Sim linked" setup note: Advanced only (the button's hover still says why).
        let note = crate::ui_mode::is_advanced(ui);
        isaac.ui(ui, note, button_response);
    });
    // A secondary tool: Advanced only.
    crate::ui_mode::advanced(ui, |ui| {
    connector(ui);
    group(ui, "Build training job", |ui| {
        let active = Some(cell.cells.active.as_str()).filter(|active| !active.is_empty());
        if button_response(ui, "Build training job", active.is_some())
            .on_hover_text(
                "Define the cells a learned planner policy trains on, around the active cell: robots and their placement, \
                 product sizes, welds, scenes and training settings; shown in 3D, saved as a plan for the cell planner",
            )
            .on_disabled_hover_text("Open a cell first (Robo Fitter → Use as cell, or a scene below Robo Fitter)")
            .clicked()
        {
            action = Some(Action::OpenTrainingJob);
        }
        let note = match active {
            Some(active) => format!("Around cell {active}: vary robots, products and welds."),
            None => "No active cell.".to_owned(),
        };
        ui.small(RichText::new(note).weak());
    });
    });

    action
}

/// The planner's state, the task program to plan and Plan Cell (which prepares the cell itself; its state is shown
/// while it runs or when it failed).
fn agentic_ui(
    ui: &mut egui::Ui,
    status: &AgenticStatus,
    plans: &PlansInfo,
    form: &mut SidebarForm,
) -> Option<Action> {
    let mut action = None;
    match status.prepare {
        PrepareState::Preparing => {
            ui.small(RichText::new("Preparing the cell for planning…").color(theme::current().warn));
        }
        PrepareState::Failed => {
            ui.small(RichText::new("Preparing the cell failed; Plan Cell tries again").color(theme::current().danger));
        }
        PrepareState::Idle | PrepareState::Done => {}
    }
    if let Some(error) = &status.prepare_error {
        ui.label(RichText::new(error).small().color(theme::current().danger));
    }

    // The planner stack (umarell's planner MCP) is started on demand by the planner router; only its state shows.
    match status.state {
        SimState::Starting => {
            ui.small(RichText::new("Starting the planner…").color(theme::current().warn));
        }
        SimState::Failed => {
            ui.small(RichText::new("The planner failed to start; Plan Cell starts it again").color(theme::current().danger));
        }
        SimState::Stopped | SimState::Ready => {}
    }
    if let Some(mode) = status.mode.filter(|_| status.state != SimState::Stopped && crate::ui_mode::is_advanced(ui)) {
        let what = match mode {
            factoryos_client::protocol::agentic::SimMode::Docker => "full stack (Docker)",
            factoryos_client::protocol::agentic::SimMode::Native => "planner only (native, no Docker)",
        };
        ui.small(RichText::new(format!("planner running for {} · {what}", status.cell)).weak());
    }
    if let Some(error) = &status.error {
        ui.label(RichText::new(error).small().color(theme::current().danger));
    }

    if let Some(note) = prepared_note(plans) {
        ui.small(RichText::new(note).color(theme::current().warn));
    }
    if let Some(plan) = plan_cell_row(ui, plans, form) {
        action = Some(plan);
    }
    action
}

/// The task program to plan, and "Plan Cell".
fn plan_cell_row(ui: &mut egui::Ui, plans: &PlansInfo, form: &mut SidebarForm) -> Option<Action> {
    let tasks = &plans.tasks;
    if form.task.as_ref().is_some_and(|t| !tasks.contains(t)) {
        form.task = None;
    }
    let selected = form.task.clone().or_else(|| tasks.first().cloned());
    ui.small(RichText::new("Task program (the operations to plan)").weak());
    let mut action = None;
    if tasks.is_empty() {
        match plans.products.first() {
            Some(product) => {
                ui.small(RichText::new(no_task_note(plans)).color(theme::current().warn));
                if button(ui, "Define the welds in Product & Task") {
                    action = Some(Action::EditProductTask { product: product.clone(), task: None });
                }
            }
            None => {
                ui.small(RichText::new("No product in the cell: add a product in the Robo Fitter.").color(theme::current().warn));
                if secondary_button(ui, "Add a product in the Robo Fitter") {
                    action = Some(Action::OpenFitter);
                }
            }
        }
        if crate::ui_mode::is_advanced(ui)
            && ui
            .small_button("Write one by hand…")
            .on_hover_text("The raw Task Program editor (JSON); task programs normally come from the products")
            .clicked()
        {
            action = Some(Action::EditTask(None));
        }
    } else {
        ui.horizontal(|ui| {
            let combo_width = ui.available_width() - 60.0 - ui.spacing().item_spacing.x;
            let label = |task: &str| task_label(task, task_source(plans, task));
            theme::truncated_combo(ui, "task_program", selected.as_deref().map(label).unwrap_or_default(), combo_width, |ui| {
                for task in tasks {
                    ui.selectable_value(&mut form.task, Some(task.clone()), label(task));
                }
            });
            let source = selected.as_deref().and_then(|task| task_source(plans, task));
            let hover = if source.is_some_and(|s| s.product.is_some()) {
                "Edit its welds in Product & Task (it comes from the product)"
            } else {
                "Open it in the Task Program editor beside the 3D view"
            };
            if ui
                .add(egui::Button::new("Edit").min_size(Vec2::new(60.0, 0.0)))
                .on_hover_text(hover)
                .clicked()
                && let Some(task) = &selected
            {
                action = Some(edit_action(task, source));
            }
        });
        if let Some(source) = selected.as_deref().and_then(|task| task_source(plans, task)) {
            if source.stale {
                ui.small(RichText::new("The product changed since: saving it in Product & Task updates the cell.").color(theme::current().warn));
            } else if source.edited && source.product.is_some() {
                ui.small(RichText::new("Edited by hand since it was made from the product.").color(theme::current().warn));
            }
        }
    }
    let planner = form.planner.clone().unwrap_or_else(|| PLANNERS[0].0.to_owned());
    // Simple mode keeps the picked planner (the default unless one was picked in Advanced).
    crate::ui_mode::advanced(ui, |ui| {
    ui.small(RichText::new("Planner").weak());
    let width = ui.available_width();
    theme::truncated_combo(ui, "planner", planner.as_str(), width, |ui| {
        for (name, hint) in PLANNERS {
            let picked = planner == name;
            if ui.selectable_label(picked, name).on_hover_text(hint).clicked() {
                form.planner = (name != PLANNERS[0].0).then(|| name.to_owned());
            }
        }
    })
    .response
        .on_hover_text(PLANNERS.iter().find(|(n, _)| *n == planner).map_or("", |(_, h)| *h));
    });
    if button(ui, "Plan Cell") {
        action = Some(Action::PlanCell { task: selected.unwrap_or_default(), planner });
    }
    action
}

/// Where the task program `task` (a `PlansInfo::tasks` entry) comes from, if the backend says.
pub(crate) fn task_source<'a>(plans: &'a PlansInfo, task: &str) -> Option<&'a TaskSource> {
    plans.task_sources.iter().find(|s| s.file.strip_suffix(".json").unwrap_or(&s.file) == task)
}

/// A task program's name in the dropdown: by its product and task when it comes from one,
/// with whether the product changed since or the file was edited.
pub(crate) fn task_label(task: &str, source: Option<&TaskSource>) -> String {
    let Some(source) = source.filter(|s| s.product.is_some()) else {
        return format!("{task} (hand-written)");
    };
    let product = source.product.as_deref().unwrap_or_default();
    let mut label = match source.task.as_deref() {
        Some(name) => format!("{product} · {name} (from product)"),
        None => format!("{product} (from product)"),
    };
    if source.stale {
        label.push_str(" · product changed");
    }
    if source.edited {
        label.push_str(" · edited");
    }
    label
}

/// Where "Edit" goes for a task program: a product-derived one to its product and task in
/// Product & Task, a hand-written one to the raw Task Program editor.
pub(crate) fn edit_action(task: &str, source: Option<&TaskSource>) -> Action {
    match source.and_then(|s| s.product.clone()) {
        Some(product) => Action::EditProductTask { product, task: source.and_then(|s| s.task.clone()) },
        None => Action::EditTask(Some(task.to_owned())),
    }
}

/// What Plan Cell does for a cell without a task program.
fn no_task_note(plans: &PlansInfo) -> String {
    if plans.product_welds > 0 {
        format!(
            "None yet: Plan Cell writes it from the {} welds of product {}: define them in Product & Task.",
            plans.product_welds,
            plans.products.join(", ")
        )
    } else {
        format!("None yet: define the welds of product {} in Product & Task.", plans.products.join(", "))
    }
}

/// The scene to simulate in (the active cell), its planned behaviour trees, and the simulation's
/// executor playing one.
fn executor_ui(ui: &mut egui::Ui, cell: &CellRunner<'_>, form: &mut SidebarForm) -> Option<Action> {
    let mut action = None;
    if let Some(select) = scene_row(ui, cell.cells, cell.sim_cell) {
        action = Some(select);
    }
    if let Some(error) = cell.error {
        ui.label(RichText::new(error).small().color(theme::current().danger));
    }
    let plans = &cell.plans.plans;
    if form.plan.as_ref().is_some_and(|p| !plans.iter().any(|plan| plan.path == *p)) {
        form.plan = None;
    }
    let selected = form.plan.clone().or_else(|| plans.first().map(|plan| plan.path.clone()));

    ui.small(RichText::new(format!("Behaviour trees of cell {}", or_dash(&cell.plans.cell))).weak());
    ui.horizontal(|ui| {
        let combo_width = ui.available_width() - BUTTON_HEIGHT - 2.0 * ui.spacing().item_spacing.x;
        let label = |path: &str| plans.iter().find(|p| p.path == path).map(plan_label).unwrap_or_default();
        let text = selected.as_deref().map_or_else(|| "No behaviour trees yet".to_owned(), label);
        ui.add_enabled_ui(!plans.is_empty(), |ui| {
            theme::truncated_combo(ui, "bt_plan", text, combo_width, |ui| {
                for plan in plans {
                    if ui.selectable_label(selected.as_deref() == Some(plan.path.as_str()), plan_label(plan)).clicked() {
                        form.plan = Some(plan.path.clone());
                    }
                }
            });
        });
        if refresh_button(ui).on_hover_text("Re-list the planned behaviour trees").clicked() {
            action = Some(Action::RefreshPlans);
        }
    });
    if plans.is_empty() {
        ui.small(
            RichText::new("Plan the cell first: Agentic Planning → Plan Cell (or ask the planner in the chat).")
                .weak(),
        );
    }
    for note in scene_version_notes(cell.plans, selected.as_deref()) {
        ui.label(RichText::new(note).small().color(theme::current().warn));
    }

    let label = if cell.plan_pending { "Starting the simulation…" } else { "Load & Play" };
    let enabled = selected.is_some() && !cell.plan_pending;
    if ui.add_enabled_ui(enabled, |ui| button(ui, label)).inner {
        let scene = Some(cell.cells.active.clone()).filter(|scene| !scene.is_empty());
        action = selected.clone().map(|path| Action::LoadPlan { path, cell: scene });
    }
    let failed = matches!(cell.plan_result, Some(Err(_)));
    if let Some(Err(err)) = cell.plan_result {
        ui.label(RichText::new(err).small().color(theme::current().danger));
    }

    if let Some(playback) = playback_ui(ui, cell, &mut form.speed) {
        action = Some(playback);
    }
    let logs = RichText::new("Backend logs").small();
    if crate::ui_mode::is_advanced(ui)
        && ui
        .link(if failed { logs.color(theme::current().danger) } else { logs })
        .on_hover_text("The backend's log: the simulation's start, the tree's load, errors")
        .clicked()
    {
        action = Some(Action::ViewLogs);
    }
    action
}

/// The loaded behaviour tree's playback controls, while the simulation has one loaded.
fn playback_ui(ui: &mut egui::Ui, cell: &CellRunner<'_>, speed_edit: &mut SpeedEdit) -> Option<Action> {
    let mut action = None;
    let trajectory = cell.trajectory.filter(|t| t["loaded"].as_bool() == Some(true))?;
    let playing = trajectory["playing"].as_bool().unwrap_or(false);
    let (t, duration) = (trajectory["t"].as_f64().unwrap_or(0.0), trajectory["duration"].as_f64().unwrap_or(0.0));
    let tree = trajectory["tree_id"].as_str().or(trajectory["plan"].as_str()).unwrap_or("behaviour tree");
    ui.label(
        RichText::new(format!("{} {tree}", if playing { "▶" } else { "⏸" }))
            .small()
            .color(if playing { theme::current().ok } else { theme::current().warn }),
    );
    ui.add(
        egui::ProgressBar::new(if duration > 0.0 { (t / duration) as f32 } else { 0.0 })
            .text(format!("{t:.1} / {duration:.1} s")),
    );
    ui.horizontal(|ui| {
        if ui.button(if playing { "⏸ Pause" } else { "▶ Play" }).clicked() {
            action = Some(Action::Trajectory(if playing { "trajectory_pause" } else { "trajectory_play" }, None));
        }
        if ui.button("■ Stop").on_hover_text("Stop and rewind").clicked() {
            action = Some(Action::Trajectory("trajectory_stop", None));
        }
        let mut looping = trajectory["loop"].as_bool().unwrap_or(false);
        if ui.checkbox(&mut looping, "Loop").changed() {
            action = Some(Action::Trajectory("trajectory_set_loop", Some(serde_json::json!({ "loop": looping }))));
        }
    });
    crate::ui_mode::advanced(ui, |ui| ui.horizontal_wrapped(|ui| {
        ui.small("Speed");
        let now = web_time::Instant::now();
        let shown = speed_edit.shown(trajectory["speed"].as_f64(), now);
        let mut speed = shown;
        // Applied as it changes (dragging too), so it takes effect while playing; the drag step
        // grows with the speed so the whole range is a short drag.
        let response = ui
            .add(
                egui::DragValue::new(&mut speed)
                    .range(SPEED_RANGE)
                    .speed((shown * 0.01).max(0.002))
                    .max_decimals(2)
                    .suffix("×"),
            )
            .on_hover_text("Playback speed: drag, or click to type one (0.1× to 20×)");
        if response.changed() && speed != shown {
            action = Some(speed_edit.set(speed, now));
        }
        for preset in SPEED_PRESETS {
            if ui.add(theme::segment((shown - preset).abs() < 1e-6, RichText::new(speed_label(preset)).small())).clicked() {
                action = Some(speed_edit.set(preset, now));
            }
        }
        if speed_edit.pending.is_some() {
            // Show the simulation's speed again if it never confirms this one.
            ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(SPEED_CONFIRM_S + 0.1));
        }
    }));
    // The robots' phases: Advanced only.
    let robots = if crate::ui_mode::is_advanced(ui) { trajectory["robots"].as_array() } else { None };
    for robot in robots.into_iter().flatten() {
        let progress = robot["progress"].as_f64().map(|p| format!(" {:.0}%", p * 100.0)).unwrap_or_default();
        ui.small(
            RichText::new(format!(
                "{}: {} {}{progress}",
                robot["robot_id"].as_str().unwrap_or("?"),
                robot["phase"].as_str().unwrap_or("-"),
                robot["op_id"].as_str().unwrap_or(""),
            ))
            .weak(),
        );
    }
    if crate::ui_mode::is_advanced(ui) && secondary_button(ui, "Unload (back to KRL)") {
        action = Some(Action::Trajectory("trajectory_unload", None));
    }
    action
}

fn plan_label(plan: &factoryos_client::protocol::PlanInfo) -> String {
    crate::code_export::plan_label(plan)
}

fn or_dash(text: &str) -> &str {
    if text.is_empty() { "—" } else { text }
}

/// Pick an existing cell and open it in the Robo Fitter (it becomes the active cell).
fn open_cell_row(ui: &mut egui::Ui, cells: &CellsInfo, form: &mut SidebarForm) -> Option<Action> {
    let rows = cell_list::cell_rows(cells);
    if form.open_cell.as_ref().is_some_and(|c| !rows.iter().any(|row| row.name == *c)) {
        form.open_cell = None;
    }
    let selected = form
        .open_cell
        .clone()
        .or_else(|| (!cells.active.is_empty()).then(|| cells.active.clone()))
        .or_else(|| rows.first().map(|row| row.name.clone()));
    let picked = selected.as_deref().and_then(|name| rows.iter().find(|row| row.name == name));
    ui.small(RichText::new("Or open a cell (every saved scene is one)").weak());
    let mut action = None;
    ui.horizontal(|ui| {
        let combo_width = ui.available_width() - 70.0 - ui.spacing().item_spacing.x;
        let text = picked.map_or_else(|| "No cells — set a cells folder in Configuration".to_owned(), cell_list::CellRow::label);
        ui.add_enabled_ui(!rows.is_empty(), |ui| {
            theme::truncated_combo(ui, "open_cell", text, combo_width, |ui| {
                for row in &rows {
                    if cell_label(ui, selected.as_deref() == Some(row.name.as_str()), row).clicked() {
                        form.open_cell = Some(row.name.clone());
                    }
                }
            });
        })
        .response
        .on_hover_text(picked.map_or_else(|| cells.cells_dir.clone(), cell_list::CellRow::hover));
        if ui
            .add_enabled(picked.is_some(), egui::Button::new("Open").min_size(Vec2::new(70.0, 0.0)))
            .on_hover_text("Make it the active cell and show it in the Robo Fitter's 3D view")
            .clicked()
        {
            action = picked.map(|row| match row.target() {
                cell_list::CellTarget::Cell(cell) => Action::OpenCell(cell),
                cell_list::CellTarget::Scene(scene) => Action::UseScene { scene, open: true },
            });
        }
    });
    if let Some(badge) = picked.and_then(|row| row.badge.as_deref()) {
        ui.small(RichText::new(format!("{badge}: Plan Cell prepares it again")).color(theme::current().warn));
    }
    action
}

/// A cell picker's entry: its label (legacy / not used yet / stale tags) and hover text.
fn cell_label(ui: &mut egui::Ui, selected: bool, row: &cell_list::CellRow) -> egui::Response {
    let text = RichText::new(row.label());
    let text = if row.badge.is_some() {
        text.color(theme::current().warn)
    } else if row.is_legacy() {
        text.weak()
    } else {
        text
    };
    ui.selectable_label(selected, text).on_hover_text(row.hover())
}

const SCENE_HOVER: &str = "The cell whose scene the behaviour tree is loaded into and played in. Every saved Robo Fitter \
    scene is a cell here: its folder (plans, task programs, exports) is made the first time it is used. It is the \
    active cell, so steps 2-5 follow it too.";

/// Step 1's "Scene": the cell to simulate in (the active cell; a picker with more than one), plus a
/// rescan button.
fn scene_row(ui: &mut egui::Ui, cells: &CellsInfo, sim_cell: Option<&str>) -> Option<Action> {
    let mut action = None;
    let rows = cell_list::cell_rows(cells);
    let active = cell_list::row(&rows, &cells.active);
    ui.small(RichText::new("Scene").weak()).on_hover_text(SCENE_HOVER);
    ui.horizontal(|ui| {
        let width = ui.available_width() - BUTTON_HEIGHT - 2.0 * ui.spacing().item_spacing.x;
        let selected = if rows.is_empty() {
            "No cells — set a cells folder in Configuration".to_owned()
        } else if cells.active.is_empty() {
            "Select a scene".to_owned()
        } else {
            active.map_or_else(|| cells.active.clone(), cell_list::CellRow::label)
        };
        let hover = active.map_or_else(|| SCENE_HOVER.to_owned(), |row| format!("{SCENE_HOVER}\n\n{}", row.hover()));
        if rows.len() > 1 || cells.active.is_empty() {
            ui.add_enabled_ui(!rows.is_empty(), |ui| {
                theme::truncated_combo(ui, "active_cell", selected, width, |ui| {
                    for row in &rows {
                        let current = row.folder && row.name == cells.active;
                        if cell_label(ui, current, row).clicked() && !current {
                            action = Some(match row.target() {
                                cell_list::CellTarget::Cell(cell) => Action::SetActiveCell(cell),
                                cell_list::CellTarget::Scene(scene) => Action::UseScene { scene, open: false },
                            });
                        }
                    }
                });
            })
            .response
            .on_hover_text(hover);
        } else {
            ui.add_sized([width, BUTTON_HEIGHT], egui::Label::new(RichText::new(selected).strong()).truncate())
                .on_hover_text(hover);
        }
        if refresh_button(ui).on_hover_text(format!("Re-scan the cells folder ({})", cells.cells_dir)).clicked() {
            action = Some(Action::RefreshCells);
        }
    });
    if !cells.active.is_empty() {
        ui.small(RichText::new(scene_note(&cells.active, sim_cell)).weak());
    }
    action
}

/// Step 1's warnings about the scene's version: a legacy cell differs from the Robo Fitter scene
/// it was copied from, and the `selected` plan is stale (planned for other inputs: scene, URDFs,
/// products).
fn scene_version_notes(plans: &PlansInfo, selected: Option<&str>) -> Vec<String> {
    let mut notes = Vec::new();
    if let Some(scene) = &plans.out_of_date_with {
        notes.push(format!(
            "⚠ Cell {} is a legacy copy and differs from its scene {scene}: it is simulated with its own copy.",
            plans.cell
        ));
    }
    if let Some(plan) = plans.plans.iter().find(|plan| plan.stale && Some(plan.path.as_str()) == selected) {
        notes.push(match cell_list::stale_badge(false, plan.stale_reason.as_deref()) {
            Some(badge) => format!("{badge}: this behaviour tree was planned before; plan the cell again."),
            None => "⚠ This behaviour tree was planned for an earlier version of the scene: plan the cell again.".to_owned(),
        });
    }
    notes
}

/// Agentic Planning's note when the active cell's prepared planner model is stale.
fn prepared_note(plans: &PlansInfo) -> Option<String> {
    let badge = cell_list::stale_badge(false, plans.prepared_stale_reason.as_deref())?;
    Some(format!("{badge}: Plan Cell prepares the cell again"))
}

/// The line under step 1's Scene: whether the picked scene is the one being simulated.
fn scene_note(picked: &str, sim_cell: Option<&str>) -> String {
    match sim_cell {
        Some(sim) if sim == picked => format!("{picked} · simulated in the Robo Fitter's 3D view"),
        Some(sim) => format!("{sim} is simulated now · Load & Play simulates {picked}"),
        None => format!("{picked} · Load & Play simulates it in the Robo Fitter's 3D view"),
    }
}

/// A numbered step of the guided flow: the group box with its number, title and state, and the
/// step's one-line summary above its contents.
fn step_group(ui: &mut egui::Ui, n: u8, title: &str, step: &flow::Step, add_contents: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .stroke(Stroke::new(2.0, theme::current().accent_hover))
        .corner_radius(CornerRadius::same(14))
        .inner_margin(egui::Margin::symmetric(12, 10))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("{n}")).small().strong().color(Color32::WHITE).background_color(theme::current().accent));
                // The title takes what the state at the right leaves, cut with an ellipsis (full on hover).
                let state = egui::WidgetText::from(RichText::new(format!("● {}", step.state.label())).small().color(step.state.color()));
                let state_width = state
                    .clone()
                    .into_galley(ui, Some(egui::TextWrapMode::Extend), f32::INFINITY, egui::TextStyle::Small)
                    .size()
                    .x;
                let title_width = (ui.available_width() - state_width - ui.spacing().item_spacing.x).max(0.0);
                ui.allocate_ui_with_layout(Vec2::new(title_width, 0.0), egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    ui.add(egui::Label::new(RichText::new(title.to_uppercase()).small().strong()).truncate());
                });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add(egui::Label::new(state).truncate());
                });
            });
            ui.add_space(2.0);
            ui.label(RichText::new(&step.summary).small().color(if step.state == flow::StepState::Failed { theme::current().danger } else { Color32::GRAY }));
            ui.add_space(4.0);
            ui.spacing_mut().item_spacing.y = 8.0;
            add_contents(ui);
        });
}

fn group(ui: &mut egui::Ui, title: &str, add_contents: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .stroke(Stroke::new(2.0, theme::current().accent_hover))
        .corner_radius(CornerRadius::same(14))
        .inner_margin(egui::Margin::symmetric(12, 10))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(title.to_uppercase()).small().strong());
            ui.add_space(4.0);
            ui.spacing_mut().item_spacing.y = 8.0;
            add_contents(ui);
        });
}

/// Short dashed line linking two stacked group boxes.
fn connector(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 20.0), egui::Sense::hover());
    ui.painter().extend(egui::Shape::dashed_line(
        &[rect.center_top(), rect.center_bottom()],
        Stroke::new(2.0, theme::current().accent_hover),
        4.0,
        3.0,
    ));
}

fn button_size(ui: &egui::Ui) -> Vec2 {
    Vec2::new(ui.available_width(), BUTTON_HEIGHT)
}

pub(crate) fn button(ui: &mut egui::Ui, label: &str) -> bool {
    ui.add_sized(
        button_size(ui),
        egui::Button::new(RichText::new(label).color(Color32::WHITE))
            .fill(theme::current().accent)
            .corner_radius(4)
            .truncate(),
    )
    .clicked()
}

/// [`button`], possibly disabled, with its response (for hover texts).
pub(crate) fn button_response(ui: &mut egui::Ui, label: &str, enabled: bool) -> egui::Response {
    ui.add_enabled(
        enabled,
        egui::Button::new(RichText::new(label).color(Color32::WHITE))
            .fill(theme::current().accent)
            .corner_radius(4)
            .truncate()
            .min_size(button_size(ui)),
    )
}

pub(crate) fn secondary_button(ui: &mut egui::Ui, label: &str) -> bool {
    ui.add(egui::Button::new(RichText::new(label).small()).corner_radius(4).truncate())
        .clicked()
}

fn refresh_button(ui: &mut egui::Ui) -> egui::Response {
    ui.add(
        egui::Button::new(RichText::new("↻").color(Color32::WHITE))
            .fill(theme::current().accent)
            .corner_radius(4)
            .min_size(Vec2::splat(BUTTON_HEIGHT)),
    )
}

#[cfg(test)]
mod tests {
    use super::{Action, PlansInfo, SpeedEdit, TaskSource, clamp_speed, edit_action, speed_label, no_task_note, prepared_note, scene_note, scene_version_notes, task_label, task_source};

    fn derived(stale: bool, edited: bool) -> TaskSource {
        TaskSource {
            file: "cube__spot_weld_1.json".to_owned(),
            product: Some("bug_cube".to_owned()),
            task: Some("spot_weld_1".to_owned()),
            fixture: Some("cube".to_owned()),
            stale,
            edited,
        }
    }

    #[test]
    fn task_labels_name_their_source() {
        assert_eq!(task_label("cube__spot_weld_1", Some(&derived(false, false))), "bug_cube · spot_weld_1 (from product)");
        assert_eq!(
            task_label("cube__spot_weld_1", Some(&derived(true, false))),
            "bug_cube · spot_weld_1 (from product) · product changed"
        );
        assert_eq!(task_label("cube__spot_weld_1", Some(&derived(false, true))), "bug_cube · spot_weld_1 (from product) · edited");
        assert_eq!(task_label("4welds", None), "4welds (hand-written)");
        let hand = TaskSource { file: "4welds.json".to_owned(), ..TaskSource::default() };
        assert_eq!(task_label("4welds", Some(&hand)), "4welds (hand-written)");
    }

    #[test]
    fn task_sources_match_by_file_stem() {
        let plans = PlansInfo { task_sources: vec![derived(false, false)], ..PlansInfo::default() };
        assert!(task_source(&plans, "cube__spot_weld_1").is_some());
        assert!(task_source(&plans, "4welds").is_none());
    }

    #[test]
    fn edit_routes_derived_programs_to_the_product() {
        assert_eq!(
            edit_action("cube__spot_weld_1", Some(&derived(false, false))),
            Action::EditProductTask { product: "bug_cube".to_owned(), task: Some("spot_weld_1".to_owned()) }
        );
        assert_eq!(edit_action("4welds", None), Action::EditTask(Some("4welds".to_owned())));
        let hand = TaskSource { file: "4welds".to_owned(), edited: true, ..TaskSource::default() };
        assert_eq!(edit_action("4welds", Some(&hand)), Action::EditTask(Some("4welds".to_owned())));
    }

    #[test]
    fn scene_note_names_the_simulated_cell() {
        assert_eq!(scene_note("weld", Some("weld")), "weld · simulated in the Robo Fitter's 3D view");
        assert_eq!(scene_note("glue", Some("weld")), "weld is simulated now · Load & Play simulates glue");
        assert_eq!(scene_note("weld", None), "weld · Load & Play simulates it in the Robo Fitter's 3D view");
    }

    #[test]
    fn no_task_note_names_the_product_welds() {
        let mut plans = PlansInfo { products: vec!["bug_cube".to_owned()], ..PlansInfo::default() };
        assert_eq!(no_task_note(&plans), "None yet: define the welds of product bug_cube in Product & Task.");
        plans.product_welds = 4;
        assert_eq!(no_task_note(&plans), "None yet: Plan Cell writes it from the 4 welds of product bug_cube: define them in Product & Task.");
    }

    #[test]
    fn step_1_warns_about_an_earlier_scene() {
        use factoryos_client::protocol::PlanInfo;
        let plan = |path: &str, stale: bool| PlanInfo { path: path.to_owned(), stale, ..PlanInfo::default() };
        let mut plans = PlansInfo {
            cell: "cell_new_scene".to_owned(),
            plans: vec![plan("p/new.json", false), plan("p/old.json", true)],
            ..PlansInfo::default()
        };
        assert!(scene_version_notes(&plans, Some("p/new.json")).is_empty());
        assert_eq!(
            scene_version_notes(&plans, Some("p/old.json")),
            ["⚠ This behaviour tree was planned for an earlier version of the scene: plan the cell again."]
        );
        plans.plans[1].stale_reason = Some("the scene new-scene.yaml changed".to_owned());
        assert_eq!(
            scene_version_notes(&plans, Some("p/old.json")),
            ["⚠ stale: the scene new-scene.yaml changed: this behaviour tree was planned before; plan the cell again."]
        );
        plans.out_of_date_with = Some("new-scene.yaml".to_owned());
        assert_eq!(
            scene_version_notes(&plans, Some("p/new.json")),
            ["⚠ Cell cell_new_scene is a legacy copy and differs from its scene new-scene.yaml: it is simulated with its own copy."]
        );
        assert_eq!(prepared_note(&plans), None);
        plans.prepared_stale_reason = Some("robot r1's URDF changed".to_owned());
        assert_eq!(prepared_note(&plans).as_deref(), Some("⚠ stale: robot r1's URDF changed: Plan Cell prepares the cell again"));
    }
    #[test]
    fn a_set_speed_shows_until_the_simulation_reports_it() {
        let start = web_time::Instant::now();
        let mut edit = SpeedEdit::default();
        assert_eq!(edit.shown(None, start), 1.0);
        assert_eq!(edit.shown(Some(1.5), start), 1.5);
        assert_eq!(
            edit.set(5.0, start),
            Action::Trajectory("trajectory_set_speed", Some(serde_json::json!({ "speed": 5.0 })))
        );
        // The state still says the old speed: the widget keeps the new one (no snap back).
        assert_eq!(edit.shown(Some(1.0), start), 5.0);
        // Confirmed: from then on the simulation's speed shows.
        assert_eq!(edit.shown(Some(5.0), start), 5.0);
        assert_eq!(edit.shown(Some(2.0), start), 2.0);
    }

    #[test]
    fn an_unconfirmed_speed_gives_way_to_the_simulations() {
        let start = web_time::Instant::now();
        let mut edit = SpeedEdit::default();
        edit.set(10.0, start);
        assert_eq!(edit.shown(Some(1.0), start + std::time::Duration::from_secs(1)), 10.0);
        assert_eq!(edit.shown(Some(1.0), start + std::time::Duration::from_secs(4)), 1.0);
    }

    /// The width a resizable left pane, dragged to `want`, ends up with when it shows the whole
    /// sidebar (cards with long names, Simple and Advanced): content wider than the pane widens it.
    fn sidebar_pane_width(want: f32, mode: crate::ui_mode::UiMode) -> f32 {
        use crate::rr::egui;
        use factoryos_client::protocol::{CellsInfo, PlanInfo};
        let long = "body_weld_4x8_with_a_rather_long_scene_name";
        let cells = CellsInfo {
            cells_dir: "/var/lib/factoryos/cells/with/a/long/path".to_owned(),
            cells: vec![long.to_owned(), "body_weld_3".to_owned()],
            active: long.to_owned(),
            ..CellsInfo::default()
        };
        let plans = PlansInfo {
            cell: long.to_owned(),
            plans: vec![PlanInfo {
                path: "agentic_planner/plans/bt_1793975983_body_weld_4x8.json".to_owned(),
                tree_id: "bt_1793975983_body_weld_4x8_with_a_rather_long_scene_name".to_owned(),
                num_operations: 12,
                planner: "umarell_u3".to_owned(),
                status: "ok".to_owned(),
                ..PlanInfo::default()
            }],
            tasks: vec!["body_weld_4x8__spot_weld_1".to_owned()],
            products: vec!["body_weld_4x8".to_owned()],
            task_sources: vec![TaskSource {
                file: "body_weld_4x8__spot_weld_1.json".to_owned(),
                product: Some("body_weld_4x8_product_with_a_long_name".to_owned()),
                task: Some("spot_weld_1".to_owned()),
                ..TaskSource::default()
            }],
            ..PlansInfo::default()
        };
        let agentic = factoryos_client::protocol::agentic::AgenticStatus::default();
        let runner = super::CellRunner {
            cells: &cells,
            error: None,
            agentic: &agentic,
            plans: &plans,
            trajectory: None,
            plan_pending: false,
            plan_result: None,
            linked_cell: Some(long),
            planning_waiting: true,
            product: Some("body_weld_4x8_product_with_a_long_name"),
            loaded_plan: None,
            cell_stream: false,
            live_cell: None,
            live_head: crate::flow::LiveHead::new(None),
            sim_cell: Some("another_cell_that_is_simulated_now"),
            weld_labels: true,
        };
        let api = factoryos_client::Api::new("http://127.0.0.1:1");
        let mut form = super::SidebarForm::default();
        let mut export = crate::code_export::CodeExport::new(api.clone());
        let mut hardware = crate::hardware_ui::HardwareUi::new(api.clone());
        let mut infra = crate::infra_ui::InfraUi::new(api.clone());
        let mut isaac = crate::isaac_ui::IsaacSim::new(api, std::rc::Rc::default());
        let ctx = egui::Context::default();
        // The viewer's fonts, so its text takes up room (egui has none of its own).
        ctx.set_fonts(crate::theme::font_definitions());
        crate::ui_mode::set(&ctx, mode);
        let input = || egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1600.0, 1000.0))),
            ..Default::default()
        };
        let mut width = 0.0;
        for _ in 0..6 {
            let mut output = ctx.run_ui(input(), |ui| {
                let panel = egui::Panel::left("sidebar_width_test")
                    .resizable(true)
                    .min_size(120.0)
                    .default_size(want)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                            crate::theme::controls_style(ui);
                            super::ui(ui, &runner, &mut form, &mut export, &mut hardware, &mut infra, &mut isaac);
                        });
                    });
                width = panel.response.rect.width();
            });
            output.textures_delta.clear();
        }
        width
    }

    #[test]
    fn the_sidebar_fits_a_narrow_pane() {
        for mode in [crate::ui_mode::UiMode::Simple, crate::ui_mode::UiMode::Advanced] {
            let width = sidebar_pane_width(240.0, mode);
            assert!(width <= 241.0, "{mode:?}: the pane is {width} px wide, not 240");
        }
    }

    #[test]
    fn speeds_are_clamped_to_the_panels_range() {
        let start = web_time::Instant::now();
        let mut edit = SpeedEdit::default();
        assert_eq!(
            edit.set(50.0, start),
            Action::Trajectory("trajectory_set_speed", Some(serde_json::json!({ "speed": 20.0 })))
        );
        assert_eq!(clamp_speed(0.0), 0.1);
        assert_eq!(clamp_speed(f64::NAN), 1.0);
        assert_eq!(clamp_speed(2.5), 2.5);
        assert_eq!(speed_label(0.5), "0.5×");
        assert_eq!(speed_label(10.0), "10×");
    }
}
