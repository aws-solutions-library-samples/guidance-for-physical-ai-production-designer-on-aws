//! The guided flow of the sidebar, from a behaviour tree to the running cell: each numbered step's
//! state and one-line summary, worked out from what the viewer knows (pure functions, so the
//! rules are tested here; the sidebar draws them).
//!
//! 1. Behaviour Tree: a planned tree, loaded into the executor (Load & Play).
//! 2. Generate PLC + robot code: the Control Code Export of the plan, then its Hardware
//!    Configuration (the head network, station names, IPs of the export's robots; saved for every
//!    later export: saving it after the export makes the step stale, [`hardware_changed`]).
//! 3. Deploy to the virtual cell: `projects`, `deploy`, `krl`, no `start` (deployed, not producing).
//! 4. Review in the PLC IDE: the head project open in CODESYS over NICE DCV.
//! 5. Start production: the head pulsed into `CS_RUNNING`; restart, auto-restart, stop. Its state is
//!    the live head's (fos-live-bridge on `factoryos/cell/state`), the last job's only without one.

use factoryos_client::protocol::codegen::same_plan_id;
use factoryos_client::protocol::infra::{InfraStatus, JobState, ROLE_WINDOWS_IDE};

use crate::theme;
use crate::code_export::{DeployChoice, export_label};
use crate::rr::egui::Color32;

/// Where a step stands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepState {
    /// Its work is done (and nothing later invalidated it).
    Done,
    /// Its result matches its inputs (the export of the plan and config; the deployed code).
    UpToDate,
    /// It has a result, but an input changed since (or it is not the one the next step uses).
    Stale,
    /// The user's turn here.
    Action,
    /// A job of this step runs.
    Running,
    /// It waits for an earlier step (or the virtual cell).
    Waiting,
    /// Its last job failed.
    Failed,
}

impl StepState {
    pub fn label(self) -> &'static str {
        match self {
            StepState::Done => "done",
            StepState::UpToDate => "up to date",
            StepState::Stale => "stale",
            StepState::Action => "needs action",
            StepState::Running => "running",
            StepState::Waiting => "waiting",
            StepState::Failed => "failed",
        }
    }

    pub fn color(self) -> Color32 {
        match self {
            StepState::Done | StepState::UpToDate => theme::current().ok,
            StepState::Stale | StepState::Action | StepState::Running => theme::current().warn,
            StepState::Waiting => Color32::GRAY,
            StepState::Failed => theme::current().danger,
        }
    }
}

/// A step's state and its one-line summary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    pub state: StepState,
    pub summary: String,
}

fn step(state: StepState, summary: impl Into<String>) -> Step {
    Step { state, summary: summary.into() }
}

/// Step 1: the planned behaviour trees and the one the executor plays.
#[derive(Clone, Debug, Default)]
pub struct BtFacts<'a> {
    pub plans: usize,
    /// The tree the executor has loaded (its name), if any.
    pub loaded: Option<&'a str>,
    pub playing: bool,
    /// Load & Play waits for the simulation (the backend starts it).
    pub pending: bool,
    /// The scene (cell) picked in step 1: the next Load & Play simulates it.
    pub scene: Option<&'a str>,
    /// The cell the running simulation runs (`factoryos/sim/status`), if known.
    pub simulated: Option<&'a str>,
}

pub fn bt_step(f: &BtFacts<'_>) -> Step {
    let picked = f.scene.unwrap_or("the scene");
    // The tree plays in the simulated cell, which is not the picked one after a Scene change.
    let scene = f.simulated.or(f.scene).unwrap_or("the scene");
    let other = f.simulated.zip(f.scene).is_some_and(|(sim, picked)| sim != picked);
    let next = |text: String| {
        if other {
            step(StepState::Action, format!("{text} · Load & Play to simulate {picked}"))
        } else {
            step(StepState::Done, text)
        }
    };
    match (f.loaded, f.plans) {
        _ if f.pending => step(StepState::Running, format!("Starting the simulation of {picked}…")),
        (Some(tree), _) if f.playing => next(format!("▶ Simulating {tree} in {scene}")),
        (Some(tree), _) => next(format!("⏸ {tree} in {scene}, paused")),
        (None, 0) => step(StepState::Action, "No behaviour tree yet: plan the cell (Agentic Planning)"),
        (None, n) => step(StepState::Action, format!("{n} planned: pick one, Load & Play to check it")),
    }
}

/// Step 2's Hardware Configuration: of the plan of the export the next steps use (else the plan
/// the export form shows).
#[derive(Clone, Debug, Default)]
pub struct HardwareFacts {
    /// Loaded at all (else still loading, or `error`).
    pub loaded: bool,
    pub error: Option<String>,
    /// Saved (`hardware.json`), else plan2cell's defaults.
    pub saved: bool,
    pub saved_at: Option<String>,
    pub head_ip: String,
    pub robots: usize,
    pub dirty: bool,
    pub problems: usize,
    /// It differs from the connected virtual cell's wiring.
    pub differs: bool,
}

/// The one-line summary under step 2's Hardware Configuration button (its state colours it).
pub fn hardware_summary(f: &HardwareFacts) -> Step {
    if let Some(err) = &f.error {
        return step(StepState::Failed, format!("Unavailable: {err}"));
    }
    if !f.loaded {
        return step(StepState::Waiting, "Loading plan2cell's hardware values…");
    }
    let what = if f.saved {
        format!("Saved{}", f.saved_at.as_deref().map(|t| format!(" {t}")).unwrap_or_default())
    } else {
        "defaults".to_owned()
    };
    let text = format!("{what} · head {} · {} robot(s)", f.head_ip, f.robots);
    if f.problems > 0 {
        step(StepState::Action, format!("{text} · {} problem(s)", f.problems))
    } else if f.dirty {
        step(StepState::Action, format!("{text} · unsaved edits: Save them for the export"))
    } else if f.differs {
        step(StepState::Stale, format!("{text} · differs from the virtual cell"))
    } else if !f.saved {
        step(StepState::Action, format!("{text}: check them, Save (or keep the defaults)"))
    } else {
        step(StepState::Done, text)
    }
}

/// Step 2: the export the next steps use, and the plan the form shows.
#[derive(Clone, Debug, Default)]
pub struct ExportFacts {
    pub running: bool,
    /// The last export of this viewer failed (its message).
    pub failed: Option<String>,
    /// The export Deploy takes ([`crate::code_export::CodeExport::deploy_choice`]).
    pub chosen: Option<ExportBrief>,
    /// The plan the form shows (its file stem).
    pub shown_plan: Option<String>,
}

#[derive(Clone, Debug, Default)]
pub struct ExportBrief {
    pub id: String,
    /// Its plan's file stem.
    pub plan: String,
    pub plan_id: Option<String>,
    /// The plan, the cell's plan2cell.toml or the hardware configuration changed since.
    pub stale: bool,
    pub warnings: u64,
    /// Picked in the results (else the newest).
    pub picked: bool,
    /// The Hardware Configuration was saved after it ([`hardware_changed`]).
    pub hardware_changed: bool,
}

/// The cell's saved Hardware Configuration as an export is compared with it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SavedHardware {
    /// `hardware.json` exists (else the plan2cell defaults).
    pub saved: bool,
    /// Its ETag (newer backends), as a build's `hardwareEtag` records it.
    pub etag: Option<String>,
    /// When it was saved (Unix seconds).
    pub saved_s: Option<f64>,
}

/// Whether the hardware configuration changed since an export that recorded `export_etag` (its
/// `build.json` `hardwareEtag`) and was made at `export_created_s`: the ETags differ, or, from a
/// backend that sends none, it was saved later. `None`: not loaded yet (unknown: no).
pub fn hardware_changed(export_etag: Option<&str>, export_created_s: f64, hardware: Option<&SavedHardware>) -> bool {
    let Some(h) = hardware else { return false };
    match (h.saved, h.etag.as_deref()) {
        (true, Some(etag)) => export_etag != Some(etag),
        // An older backend: no ETags, compare the times.
        (true, None) => h.saved_s.is_some_and(|s| s > export_created_s),
        // Nothing saved now; the export had a saved configuration.
        (false, _) => export_etag.is_some(),
    }
}

impl ExportFacts {
    /// The export the next steps use is outdated (the plan, plan2cell.toml or the hardware
    /// configuration changed since), or of another plan than the form shows: steps 3–5 are not
    /// done with it ([`deploy_step`]'s `stale`).
    pub fn stale(&self) -> bool {
        self.chosen.as_ref().is_some_and(|e| e.stale || e.hardware_changed || self.shown_plan.as_deref().is_some_and(|p| p != e.plan))
    }
}

pub fn generate_step(f: &ExportFacts) -> Step {
    if f.running {
        return step(StepState::Running, "Exporting PLC + robot code…");
    }
    if let Some(err) = &f.failed {
        return step(StepState::Failed, err.clone());
    }
    let Some(e) = &f.chosen else {
        return step(StepState::Action, "No export yet: Export PLC + robot code");
    };
    let plan_id = e.plan_id.as_deref().map(|p| format!(" · PLAN_ID {p}")).unwrap_or_default();
    let what = format!("{}{plan_id}", export_label(&e.id));
    if e.hardware_changed {
        return step(StepState::Stale, format!("{what}: hardware changed since the export, export again"));
    }
    if e.stale {
        return step(StepState::Stale, format!("{what}: the plan or hardware config changed since: export again"));
    }
    if let Some(plan) = f.shown_plan.as_deref().filter(|p| *p != e.plan) {
        let which = if e.picked { "the picked export" } else { "the newest export" };
        return step(StepState::Stale, format!("{which} is of {}, not {plan}: export {plan}", e.plan));
    }
    let warnings = if e.warnings > 0 { format!(" · {} warning(s)", e.warnings) } else { String::new() };
    step(StepState::UpToDate, format!("{what}{warnings}"))
}

/// The deployed export against the generated one: `= generated code`, `same PLAN_ID`, `differs`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Match {
    Same,
    SamePlan,
    Differs,
    /// Nothing known deployed, or nothing generated.
    Unknown,
}

pub fn deployed_match(status: &InfraStatus, generated: Option<&DeployChoice>) -> Match {
    match (status.deployed_export.as_ref(), generated) {
        (Some(d), Some(g)) if d.build == g.build => Match::Same,
        (Some(d), Some(g)) => match (d.plan_id.as_deref(), g.plan_id.as_deref()) {
            (Some(a), Some(b)) if same_plan_id(a, b) => Match::SamePlan,
            _ => Match::Differs,
        },
        _ => Match::Unknown,
    }
}

/// `deployed: <export> (PLAN_ID x) = generated code` / `… differs from the generated <export>`.
pub fn deployed_text(status: &InfraStatus, generated: Option<&DeployChoice>) -> Option<String> {
    let d = status.deployed_export.as_ref()?;
    let plan = d.plan_id.as_deref().map(|p| format!(" (PLAN_ID {p})")).unwrap_or_default();
    let tail = match (deployed_match(status, generated), generated) {
        (Match::Same, _) => " = generated code".to_owned(),
        (Match::SamePlan, Some(g)) => format!(": same PLAN_ID as the generated {}, another export", export_label(&g.build)),
        (Match::Differs, Some(g)) => format!(" differs from the generated {}", export_label(&g.build)),
        _ => String::new(),
    };
    Some(format!("deployed: {}{plan}{tail}", export_label(&d.build)))
}

/// Why the cell's jobs cannot run (`None`: ready).
fn cell_waiting(status: Option<&InfraStatus>) -> Option<Step> {
    let Some(s) = status else { return Some(step(StepState::Waiting, "Reading the virtual cell…")) };
    if !s.available {
        return Some(step(StepState::Waiting, "No cell control on this server"));
    }
    if let Some(why) = crate::virtual_cell::blocks(s) {
        return Some(step(StepState::Waiting, why));
    }
    (!s.ready).then(|| step(StepState::Waiting, "The virtual cell is not ready: Start virtual cell (or Bring up controllers)"))
}

/// Step 3: Deploy (or Run Control Code) and what the head holds. `stale`: step 2 is stale (the
/// plan, the cell's plan2cell.toml or the hardware configuration changed since the export, or it
/// is of another plan than the one shown), so even the generated code on the head is not current.
pub fn deploy_step(status: Option<&InfraStatus>, generated: Option<&DeployChoice>, stale: bool) -> Step {
    if let Some(s) = status {
        let d = &s.deploy;
        if d.state.busy() {
            let what = if d.starts() { "Run Control Code" } else { "Deploying" };
            let at = d.stage.as_deref().map(|st| format!(": {st}")).unwrap_or_default();
            let done = format!(" ({}/{})", d.done.len(), d.stages.len());
            return step(StepState::Running, format!("{what}{at}…{done}"));
        }
        // A failed start of Run Control Code is step 5's; the deploy itself passed.
        if d.state == JobState::Failed && d.stage.as_deref() != Some("start") {
            let mut summary = d.message.clone().unwrap_or_else(|| "the deploy failed".to_owned());
            if let Some(text) = deployed_text(s, generated) {
                summary.push_str(&format!(" · {text}"));
            }
            return step(StepState::Failed, summary);
        }
    }
    let deployed = status.and_then(|s| deployed_text(s, generated));
    let matched = status.map_or(Match::Unknown, |s| deployed_match(s, generated));
    if matched == Match::Same {
        if stale {
            return step(StepState::Stale, format!("{}, but step 2 is stale: export again, then Deploy", deployed.unwrap_or_default()));
        }
        return step(StepState::UpToDate, deployed.unwrap_or_default());
    }
    if let Some(wait) = cell_waiting(status) {
        return Step { summary: deployed.unwrap_or(wait.summary), ..wait };
    }
    match (generated, deployed) {
        (None, Some(text)) => step(StepState::Done, text),
        (None, None) => step(StepState::Waiting, "Generate the code first (step 2)"),
        (Some(_), Some(text)) if stale => step(StepState::Stale, format!("{text}: export again (step 2 is stale), then Deploy")),
        (Some(_), Some(text)) => step(StepState::Stale, format!("{text}: Deploy")),
        (Some(_), None) if stale => step(StepState::Waiting, "Nothing deployed by this server yet; step 2 is stale: export again first"),
        (Some(g), None) => step(StepState::Action, format!("Nothing deployed by this server yet: Deploy {}", export_label(&g.build))),
    }
}

/// Step 4: the PLC IDE; `opened_s`: when this viewer last opened it (Unix seconds, the server's
/// clock: the DCV session's `issuedS`); `stale`: as [`deploy_step`].
pub fn review_step(status: Option<&InfraStatus>, generated: Option<&DeployChoice>, opened_s: Option<f64>, stale: bool) -> Step {
    let Some(s) = status else { return step(StepState::Waiting, "Reading the virtual cell…") };
    let project = s.ide_project.as_ref().map_or_else(|| "the head project".to_owned(), |p| p.name.clone());
    if !s.dcv_gateway {
        return step(StepState::Waiting, "No NICE DCV on this server: the PLC IDE cannot open");
    }
    if let Some(why) = crate::virtual_cell::blocks(s) {
        return step(StepState::Waiting, why);
    }
    match s.instance(ROLE_WINDOWS_IDE) {
        Some(ide) if ide.running() => {}
        Some(ide) => return step(StepState::Waiting, format!("The Windows IDE is {}: Start virtual cell", ide.state)),
        None if s.dcv_server_fallback => {}
        None => return step(StepState::Waiting, "No Windows IDE in this cell"),
    }
    if s.deploy.state.busy() {
        return step(StepState::Waiting, format!("{project} opens in the IDE when the deploy is done"));
    }
    let deployed_s = s.deploy.finished_s.filter(|_| s.deploy.state != JobState::Failed);
    match deployed_match(s, generated) {
        Match::Same | Match::SamePlan | Match::Unknown if s.deployed_export.is_some() && stale => {
            step(StepState::Stale, format!("{project} in the IDE is the deployed code; step 2 is stale: export again, Deploy"))
        }
        Match::Same | Match::SamePlan | Match::Unknown if s.deployed_export.is_some() => {
            if opened_s.is_some_and(|o| deployed_s.is_none_or(|d| o >= d)) {
                step(StepState::Done, format!("Opened: {project} in CODESYS (window \"… [MCP]\")"))
            } else if s.deploy.state == JobState::Done && s.deploy.starts() {
                step(StepState::Done, format!("Run Control Code started it without a review: Open PLC IDE to look at {project}"))
            } else if s.deploy.state == JobState::Done && s.deploy.ide_opened == Some(false) {
                let why = s.deploy.ide_note.clone().unwrap_or_else(|| format!("{project} not opened in the PLC IDE"));
                step(StepState::Action, format!("{why}: Open PLC IDE, then File > Open it in CODESYS"))
            } else {
                step(StepState::Action, format!("Open PLC IDE: review {project} in CODESYS (view only) before starting"))
            }
        }
        _ if s.deployed_export.is_some() => {
            step(StepState::Stale, format!("{project} in the IDE is the deployed code, not the generated one: Deploy first"))
        }
        _ => step(StepState::Waiting, "Deploy first: the IDE then shows the head project"),
    }
}

/// How long the live head state counts as current (fos-live-bridge publishes it at least every
/// second while it reads the head).
pub const LIVE_HEAD_STALE_S: f64 = 10.0;

/// The head as fos-live-bridge reads it over OPC UA: `plan2cell.head` of `factoryos/cell/state`
/// (`head_view` in plan2cell/infra/viewer/fos_live_bridge.py).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HeadView {
    /// `CS_RUNNING`, `CS_ERROR`, … (the number, for a state the bridge has no name for).
    pub state: Option<String>,
    /// `ERR_ROBOT_STOPPED`, … (`ERR_NONE` without an error).
    pub error: Option<String>,
    /// The robot the error is about (1-based; 0 or none: the cell).
    pub error_robot: Option<i64>,
    pub cycles: Option<u64>,
}

impl HeadView {
    pub fn from_json(head: &serde_json::Value) -> Option<Self> {
        let text = |v: &serde_json::Value| match v {
            serde_json::Value::String(s) => Some(s.clone()),
            serde_json::Value::Number(n) => Some(n.to_string()),
            _ => None,
        };
        head.is_object().then(|| HeadView {
            state: text(&head["state"]),
            error: text(&head["error"]).filter(|e| e != "ERR_NONE" && e != "0"),
            error_robot: head["error_robot"].as_i64().filter(|r| *r > 0),
            cycles: head["cycles"].as_u64(),
        })
    }

    fn state(&self) -> Option<&str> {
        self.state.as_deref()
    }

    /// The head produces: its jobs run, or a cycle just ended.
    pub fn producing(&self) -> bool {
        matches!(self.state(), Some("CS_RUNNING" | "CS_CYCLE_DONE"))
    }

    /// `CS_ERROR ERR_ROBOT_STOPPED (robot 2)`.
    pub fn label(&self) -> String {
        let mut label = self.state.clone().unwrap_or_else(|| "state unknown".to_owned());
        if let Some(error) = &self.error {
            label.push(' ');
            label.push_str(error);
        }
        if let Some(robot) = self.error_robot.filter(|_| self.error.is_some()) {
            label.push_str(&format!(" (robot {robot})"));
        }
        label
    }
}

/// What the viewer knows of the head right now, for step 5.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LiveHead {
    /// No live head state (no fos-live-bridge on the cell state, or it never read the head).
    Offline,
    /// The last one is older than [`LIVE_HEAD_STALE_S`].
    Stale { head: HeadView, age_s: u64 },
    Live(HeadView),
}

impl LiveHead {
    /// From the last head state seen and its age.
    pub fn new(last: Option<(&serde_json::Value, f64)>) -> Self {
        match last.and_then(|(head, age)| Some((HeadView::from_json(head)?, age))) {
            None => LiveHead::Offline,
            Some((head, age)) if age > LIVE_HEAD_STALE_S => LiveHead::Stale { head, age_s: age as u64 },
            Some((head, _)) => LiveHead::Live(head),
        }
    }

    fn live(&self) -> Option<&HeadView> {
        match self {
            LiveHead::Live(head) => Some(head),
            _ => None,
        }
    }

    /// The head stopped producing and waits for Start production (CS_ERROR, CS_RECOVERY), even
    /// if the backend still counts the cell as in production.
    pub fn needs_start(&self) -> bool {
        self.live().is_some_and(|h| matches!(h.state(), Some("CS_ERROR" | "CS_RECOVERY")))
    }

    /// The head's line in step 5 (above the job's).
    pub fn line(&self) -> (String, Color32) {
        match self {
            LiveHead::Offline => ("Head: no live state (fos-live-bridge, factoryos/cell/state)".to_owned(), Color32::GRAY),
            LiveHead::Stale { head, age_s } => {
                (format!("Head: no live state for {age_s} s (last {})", head.label()), theme::current().warn)
            }
            LiveHead::Live(head) => {
                let color = match head.state() {
                    Some("CS_ERROR") => theme::current().danger,
                    _ if head.producing() => theme::current().ok,
                    _ => theme::current().warn,
                };
                let cycles = head.cycles.map(|c| format!(" · {c} cycles")).unwrap_or_default();
                (format!("Head: {}{cycles}", head.label()), color)
            }
        }
    }
}

/// The step the live head alone decides (errors, recovery, the head on its way), if any.
fn live_head_step(head: &HeadView) -> Option<Step> {
    let robot = head.error_robot.map(|r| format!(" (robot {r})")).unwrap_or_default();
    match head.state()? {
        "CS_ERROR" if head.error.as_deref() == Some("ERR_OPERATOR_STOP") => {
            Some(step(StepState::Action, "Stopped (ERR_OPERATOR_STOP) · Start production"))
        }
        "CS_ERROR" => {
            let error = head.error.as_deref().unwrap_or("unknown");
            Some(step(StepState::Failed, format!("Error: {error}{robot} · Start production to recover")))
        }
        "CS_RECOVERY" => Some(step(StepState::Action, "Recovery required · Start production")),
        state @ ("CS_BRINGUP" | "CS_HOMING_SELECT" | "CS_HOMING_RUN" | "CS_START_JOBS" | "CS_ABORTING" | "CS_RESETTING") => {
            Some(step(StepState::Running, format!("Head in {state}…")))
        }
        _ => None,
    }
}

/// Step 5: production, by the live head (`live`) where there is one, else by the last job;
/// `stale`: as [`deploy_step`].
pub fn production_step(status: Option<&InfraStatus>, generated: Option<&DeployChoice>, stale: bool, live: &LiveHead) -> Step {
    let Some(s) = status else { return step(StepState::Waiting, "Reading the virtual cell…") };
    let cell = &s.cell;
    if cell.state.busy() && !s.deploy.state.busy() {
        return step(StepState::Running, cell.message.clone().unwrap_or_else(|| "Starting production…".to_owned()));
    }
    if s.deploy.state.busy() {
        let what = if s.deploy.starts() { "Run Control Code starts it after the deploy" } else { "Start production once the deploy is done" };
        return step(StepState::Waiting, what);
    }
    if let Some(head_step) = live.live().and_then(live_head_step) {
        return head_step;
    }
    let head = match live.live() {
        Some(h) => format!(" ({})", h.label()),
        None => cell.head_state.as_deref().map(|h| format!(" ({h})")).unwrap_or_default(),
    };
    // The live head says whether it produces; without it, the last job does.
    if live.live().map_or(s.in_production, HeadView::producing) {
        let summary = match live {
            LiveHead::Live(h) => format!("In production · cycle {}", h.cycles.unwrap_or(0)),
            LiveHead::Stale { age_s, .. } => format!("In production{head} by the last job · no live head state for {age_s} s"),
            LiveHead::Offline => format!("In production{head} by the last job · no live head state"),
        };
        return match deployed_match(s, generated) {
            Match::Differs | Match::SamePlan => step(StepState::Stale, format!("{summary}, not with the generated code: Deploy it")),
            _ if stale => step(StepState::Stale, format!("{summary}; step 2 is stale: export again, Deploy")),
            _ if live.live().is_none() => step(StepState::Stale, summary),
            _ => step(StepState::Done, summary),
        };
    }
    if cell.state == JobState::Failed {
        return step(StepState::Failed, cell.message.clone().unwrap_or_else(|| "the start failed".to_owned()));
    }
    if let Some(wait) = cell_waiting(Some(s)) {
        return wait;
    }
    // Not producing: Start production starts what the head runs (`headExport`, which falls back
    // to the newest export even with nothing deployed), so it is only the next step once the
    // generated code is on the head.
    if s.deployed_export.is_none() {
        return step(StepState::Waiting, "Deploy first (step 3)");
    }
    match (deployed_match(s, generated), generated) {
        (Match::Differs | Match::SamePlan, Some(g)) => {
            return step(StepState::Waiting, format!("The generated {} is not deployed: Deploy it (step 3)", export_label(&g.build)));
        }
        _ if stale => return step(StepState::Waiting, "Step 2 is stale: export again, Deploy (step 3)"),
        _ => {}
    }
    if live.live().is_none() && cell.head_state.as_deref().is_some_and(|h| h.starts_with("CS_ERROR")) {
        return step(StepState::Action, format!("Stopped{head}: Start production"));
    }
    if live.live().is_some() {
        return step(StepState::Action, format!("Deployed, not producing{head}: Start production"));
    }
    step(StepState::Action, "Deployed, not producing (the head waits in CS_INIT / CS_RECOVERY): Start production")
}

#[cfg(test)]
mod tests {
    use factoryos_client::protocol::infra::{DeployStatus, ExportRef, IdeProject, InfraInstance};

    use super::*;

    #[test]
    fn steps_wait_for_a_powered_off_or_booting_cell() {
        use factoryos_client::protocol::infra::{CellPower, PowerState};
        let ide = InfraInstance { role: ROLE_WINDOWS_IDE.into(), state: "running".into(), ssm_ping: "Online".into(), ..Default::default() };
        let mut s = InfraStatus { available: true, dcv_gateway: true, instances: vec![ide], ..Default::default() };
        s.cell_power = Some(CellPower { state: PowerState::Off, phase: "off".into(), ..Default::default() });
        let deploy = deploy_step(Some(&s), None, false);
        assert_eq!(deploy.state, StepState::Waiting);
        let review = review_step(Some(&s), None, None, false);
        assert_eq!(review.state, StepState::Waiting);
        assert!(review.summary.contains("Start virtual cell first"), "{review:?}");
        s.cell_power = Some(CellPower { state: PowerState::Booting, phase: "booting".into(), ..Default::default() });
        assert!(review_step(Some(&s), None, None, false).summary.contains("The virtual cell is booting: wait"));
    }

    #[test]
    fn steps_wait_for_the_own_cell() {
        use factoryos_client::protocol::infra::{CellKind, CellLifecycle, CellPower, PowerState, VirtualCell};
        let ide = InfraInstance { role: ROLE_WINDOWS_IDE.into(), state: "running".into(), ssm_ping: "Online".into(), ..Default::default() };
        let mut s = InfraStatus { available: true, ready: true, dcv_gateway: true, instances: vec![ide], binding: Some(CellKind::Own), ..Default::default() };
        // No own cell yet: every step waits for Create virtual cell.
        assert!(deploy_step(Some(&s), None, false).summary.contains("Create virtual cell"));
        assert!(review_step(Some(&s), None, None, false).summary.contains("Create virtual cell"));
        let cell = |lifecycle, state, phase: &str| VirtualCell {
            id: "ws-jdoe".into(),
            lifecycle,
            power: Some(CellPower { state, phase: phase.into(), ..Default::default() }),
            ..Default::default()
        };
        s.virtual_cell = Some(cell(CellLifecycle::Off, PowerState::Off, "off"));
        assert!(review_step(Some(&s), None, None, false).summary.contains("Start virtual cell first"));
        s.virtual_cell = Some(cell(CellLifecycle::On, PowerState::Booting, "configuring"));
        assert_eq!(review_step(Some(&s), None, None, false).state, StepState::Waiting);
        s.virtual_cell = Some(cell(CellLifecycle::On, PowerState::Ready, "ready"));
        // Ready: the step's own reasons only (nothing deployed yet).
        assert_eq!(review_step(Some(&s), None, None, false).summary, "Deploy first: the IDE then shows the head project");
    }

    const NEW: &str = "20261013T090000000Z-bt_a";
    const OLD: &str = "20261012T184135962Z-bt_a";

    fn generated(build: &str, plan: &str) -> DeployChoice {
        DeployChoice { build: build.into(), plan_id: Some(plan.into()), picked: false }
    }

    fn ready(deployed: Option<(&str, &str)>) -> InfraStatus {
        InfraStatus {
            available: true,
            ready: true,
            dcv_gateway: true,
            instances: vec![InfraInstance { role: ROLE_WINDOWS_IDE.into(), state: "running".into(), ssm_ping: "Online".into(), ..Default::default() }],
            deployed_export: deployed.map(|(b, p)| ExportRef { build: b.into(), plan_id: Some(p.into()), source: "deploy".into() }),
            // As the backend sends it: with nothing deployed, the newest export (any export exists).
            head_export: Some(match deployed {
                Some((b, p)) => ExportRef { build: b.into(), plan_id: Some(p.into()), source: "deployed".into() },
                None => ExportRef { build: NEW.into(), plan_id: Some("16#8E35".into()), source: "newest".into() },
            }),
            ide_project: Some(IdeProject { name: "HeadPLC_weld".into(), path: r"C:\p\weld\HeadPLC_weld.project".into() }),
            ..Default::default()
        }
    }

    #[test]
    fn behaviour_tree() {
        assert_eq!(bt_step(&BtFacts::default()).state, StepState::Action);
        assert!(bt_step(&BtFacts { plans: 2, ..Default::default() }).summary.starts_with("2 planned"));
        let loaded = bt_step(&BtFacts { plans: 2, loaded: Some("bt_a"), playing: true, scene: Some("weld"), ..Default::default() });
        assert_eq!(loaded, step(StepState::Done, "▶ Simulating bt_a in weld"));
        let same = bt_step(&BtFacts { plans: 2, loaded: Some("bt_a"), playing: true, scene: Some("weld"), simulated: Some("weld"), ..Default::default() });
        assert_eq!(same, loaded);
        let paused = bt_step(&BtFacts { plans: 2, loaded: Some("bt_a"), scene: Some("weld"), ..Default::default() });
        assert_eq!(paused, step(StepState::Done, "⏸ bt_a in weld, paused"));
        // Another scene picked while weld is simulated: step 1 names the simulated one.
        let other = bt_step(&BtFacts { plans: 2, loaded: Some("bt_a"), playing: true, scene: Some("glue"), simulated: Some("weld"), ..Default::default() });
        assert_eq!(other, step(StepState::Action, "▶ Simulating bt_a in weld · Load & Play to simulate glue"));
        let starting = bt_step(&BtFacts { pending: true, scene: Some("weld"), ..Default::default() });
        assert_eq!(starting, step(StepState::Running, "Starting the simulation of weld…"));
    }

    #[test]
    fn hardware() {
        assert_eq!(hardware_summary(&HardwareFacts::default()).state, StepState::Waiting);
        let saved = HardwareFacts { loaded: true, saved: true, saved_at: Some("08:00 UTC".into()), head_ip: "192.168.100.10".into(), robots: 2, ..Default::default() };
        assert_eq!(hardware_summary(&saved), step(StepState::Done, "Saved 08:00 UTC · head 192.168.100.10 · 2 robot(s)"));
        assert_eq!(hardware_summary(&HardwareFacts { dirty: true, ..saved.clone() }).state, StepState::Action);
        assert_eq!(hardware_summary(&HardwareFacts { problems: 1, ..saved.clone() }).state, StepState::Action);
        assert_eq!(hardware_summary(&HardwareFacts { differs: true, ..saved.clone() }).state, StepState::Stale);
        let defaults = hardware_summary(&HardwareFacts { saved: false, ..saved.clone() });
        assert_eq!(defaults.state, StepState::Action);
        assert!(defaults.summary.starts_with("defaults · head 192.168.100.10 · 2 robot(s)"), "{}", defaults.summary);
        assert_eq!(hardware_summary(&HardwareFacts { error: Some("no plan".into()), ..saved }).state, StepState::Failed);
    }

    #[test]
    fn hardware_saved_after_the_export() {
        let at = |etag: Option<&str>, saved_s: Option<f64>| SavedHardware { saved: true, etag: etag.map(str::to_owned), saved_s };
        // By the ETag the export's build.json recorded.
        assert!(!hardware_changed(Some("a"), 100.0, Some(&at(Some("a"), Some(200.0)))), "same file: not changed, whatever the times");
        assert!(hardware_changed(Some("a"), 100.0, Some(&at(Some("b"), Some(50.0)))));
        assert!(hardware_changed(None, 100.0, Some(&at(Some("b"), None))), "exported with the defaults, saved since");
        // An older backend without ETags: the times.
        assert!(hardware_changed(None, 100.0, Some(&at(None, Some(101.0)))));
        assert!(!hardware_changed(None, 100.0, Some(&at(None, Some(99.0)))));
        // Defaults now.
        let defaults = SavedHardware::default();
        assert!(!hardware_changed(None, 100.0, Some(&defaults)));
        assert!(hardware_changed(Some("a"), 100.0, Some(&defaults)), "the saved file is gone");
        assert!(!hardware_changed(Some("a"), 100.0, None), "not loaded yet");
    }

    #[test]
    fn generate() {
        assert_eq!(generate_step(&ExportFacts::default()).state, StepState::Action);
        let e = ExportBrief { id: NEW.into(), plan: "bt_a".into(), plan_id: Some("16#8E35".into()), ..Default::default() };
        let up = ExportFacts { chosen: Some(e.clone()), shown_plan: Some("bt_a".into()), ..Default::default() };
        let s = generate_step(&up);
        assert_eq!(s.state, StepState::UpToDate);
        assert!(s.summary.ends_with("bt_a · PLAN_ID 16#8E35"), "{}", s.summary);
        let stale = ExportFacts { chosen: Some(ExportBrief { stale: true, ..e.clone() }), ..up.clone() };
        assert_eq!(generate_step(&stale).state, StepState::Stale);
        assert!(stale.stale() && !up.stale());
        // The hardware config saved after the export: stale, and steps 3–5 follow.
        let hw = ExportFacts { chosen: Some(ExportBrief { hardware_changed: true, stale: true, ..e.clone() }), ..up.clone() };
        let s = generate_step(&hw);
        assert_eq!(s.state, StepState::Stale);
        assert!(s.summary.ends_with("PLAN_ID 16#8E35: hardware changed since the export, export again"), "{}", s.summary);
        assert!(hw.stale());
        assert!(ExportFacts { chosen: Some(ExportBrief { hardware_changed: true, ..e.clone() }), ..up.clone() }.stale(), "before the list says so");
        let g = generated(NEW, "16#8E35");
        let same = ready(Some((NEW, "16#8E35")));
        assert_eq!(deploy_step(Some(&same), Some(&g), hw.stale()).state, StepState::Stale);
        let mut producing = same.clone();
        producing.in_production = true;
        assert_eq!(production_step(Some(&producing), Some(&g), hw.stale(), &LiveHead::Offline).state, StepState::Stale);
        let other = ExportFacts { shown_plan: Some("bt_b".into()), ..up.clone() };
        assert_eq!(generate_step(&other), step(StepState::Stale, "the newest export is of bt_a, not bt_b: export bt_b"));
        assert!(other.stale());
        assert_eq!(generate_step(&ExportFacts { running: true, ..up.clone() }).state, StepState::Running);
        assert_eq!(generate_step(&ExportFacts { failed: Some("✗ plan2cell failed".into()), ..up }).state, StepState::Failed);
    }

    #[test]
    fn deploy() {
        let g = generated(NEW, "16#8E35");
        assert_eq!(deploy_step(None, Some(&g), false).state, StepState::Waiting);
        // Nothing deployed yet: the user's turn.
        let none = ready(None);
        assert_eq!(deploy_step(Some(&none), Some(&g), false).state, StepState::Action);
        assert_eq!(deploy_step(Some(&none), None, false).state, StepState::Waiting, "generate first");
        // The generated code is on the head.
        let same = ready(Some((NEW, "16#8E35")));
        let s = deploy_step(Some(&same), Some(&g), false);
        assert_eq!(s.state, StepState::UpToDate);
        assert!(s.summary.starts_with("deployed: ") && s.summary.ends_with("bt_a (PLAN_ID 16#8E35) = generated code"), "{}", s.summary);
        // Older code on the head.
        let old = ready(Some((OLD, "16#1234")));
        let s = deploy_step(Some(&old), Some(&g), false);
        assert_eq!(s.state, StepState::Stale);
        assert!(s.summary.contains("(PLAN_ID 16#1234) differs from the generated"), "{}", s.summary);
        assert_eq!(deployed_match(&ready(Some((OLD, "WORD#16#8E35"))), Some(&g)), Match::SamePlan);
        // Up to date even with the cell stopped; differs and not ready: waiting, with what is deployed.
        let stopped = InfraStatus { ready: false, ..old.clone() };
        let s = deploy_step(Some(&stopped), Some(&g), false);
        assert_eq!(s.state, StepState::Waiting);
        assert!(s.summary.contains("differs"), "{}", s.summary);
        assert_eq!(deploy_step(Some(&InfraStatus { ready: false, ..same.clone() }), Some(&g), false).state, StepState::UpToDate);
        // Step 2 stale (the hardware config or the plan changed): the generated code on the head is not current.
        let s = deploy_step(Some(&same), Some(&g), true);
        assert_eq!(s.state, StepState::Stale);
        assert!(s.summary.ends_with("= generated code, but step 2 is stale: export again, then Deploy"), "{}", s.summary);
        assert_eq!(deploy_step(Some(&none), Some(&g), true).state, StepState::Waiting);
        // Live, and failed.
        let stages: Vec<String> = ["projects", "deploy", "krl"].map(str::to_owned).to_vec();
        let running = InfraStatus {
            deploy: DeployStatus { state: JobState::Running, stage: Some("deploy".into()), stages: stages.clone(), done: vec!["projects".into()], ..Default::default() },
            ..old.clone()
        };
        assert_eq!(deploy_step(Some(&running), Some(&g), false), step(StepState::Running, "Deploying: deploy… (1/3)"));
        let failed = InfraStatus {
            deploy: DeployStatus { state: JobState::Failed, stage: Some("deploy".into()), message: Some("deploy failed: x".into()), stages, ..Default::default() },
            ..old
        };
        let s = deploy_step(Some(&failed), Some(&g), false);
        assert_eq!(s.state, StepState::Failed);
        assert!(s.summary.starts_with("deploy failed: x · deployed: "), "{}", s.summary);
        let none_failed = InfraStatus { deployed_export: None, ..failed };
        assert_eq!(deploy_step(Some(&none_failed), Some(&g), false), step(StepState::Failed, "deploy failed: x"));
    }

    #[test]
    fn review() {
        let g = generated(NEW, "16#8E35");
        let mut same = ready(Some((NEW, "16#8E35")));
        same.deploy = DeployStatus { state: JobState::Done, finished_s: Some(1000.0), ..Default::default() };
        let s = review_step(Some(&same), Some(&g), None, false);
        assert_eq!(s.state, StepState::Action);
        assert!(s.summary.contains("review HeadPLC_weld in CODESYS (view only)"), "{}", s.summary);
        assert_eq!(review_step(Some(&same), Some(&g), Some(900.0), false).state, StepState::Action, "opened before the deploy");
        assert_eq!(review_step(Some(&same), Some(&g), Some(1001.0), false).state, StepState::Done);
        assert_eq!(review_step(Some(&same), Some(&g), Some(1001.0), true).state, StepState::Stale, "step 2 stale");
        let mut refused = same.clone();
        refused.deploy.ide_opened = Some(false);
        refused.deploy.ide_note = Some("HeadPLC_weld not opened in the PLC IDE: Refusing to switch projects".into());
        let s = review_step(Some(&refused), Some(&g), None, false);
        assert_eq!(s.state, StepState::Action);
        assert!(s.summary.starts_with("HeadPLC_weld not opened in the PLC IDE: Refusing"), "{}", s.summary);
        let mut run = same.clone();
        run.deploy.stages = vec!["projects".into(), "deploy".into(), "start".into()];
        assert_eq!(review_step(Some(&run), Some(&g), None, false).state, StepState::Done, "Run Control Code: no review step");
        assert_eq!(review_step(Some(&ready(None)), Some(&g), None, false).state, StepState::Waiting);
        assert_eq!(review_step(Some(&ready(Some((OLD, "16#1234")))), Some(&g), None, false).state, StepState::Stale);
        let mut ide_off = same.clone();
        ide_off.instances[0].state = "stopped".into();
        assert_eq!(review_step(Some(&ide_off), Some(&g), None, false), step(StepState::Waiting, "The Windows IDE is stopped: Start virtual cell"));
        assert_eq!(review_step(Some(&InfraStatus { dcv_gateway: false, ..same }), Some(&g), None, false).state, StepState::Waiting);
    }

    #[test]
    fn production() {
        let g = generated(NEW, "16#8E35");
        // Deployed, not producing: the definite state Deploy leaves.
        let deployed = ready(Some((NEW, "16#8E35")));
        let s = production_step(Some(&deployed), Some(&g), false, &LiveHead::Offline);
        assert_eq!(s.state, StepState::Action);
        assert!(s.summary.starts_with("Deployed, not producing"), "{}", s.summary);
        let mut producing = deployed.clone();
        producing.in_production = true;
        producing.cell.head_state = Some("CS_RUNNING".into());
        // The last job's word only, no live head: in production, but not known to be.
        assert_eq!(
            production_step(Some(&producing), Some(&g), false, &LiveHead::Offline),
            step(StepState::Stale, "In production (CS_RUNNING) by the last job · no live head state")
        );
        let older = generated(OLD, "16#1234");
        assert_eq!(production_step(Some(&producing), Some(&older), false, &LiveHead::Offline).state, StepState::Stale, "producing other code than generated");
        let mut stopped = deployed.clone();
        stopped.cell.head_state = Some("CS_ERROR".into());
        assert_eq!(production_step(Some(&stopped), Some(&g), false, &LiveHead::Offline), step(StepState::Action, "Stopped (CS_ERROR): Start production"));
        let mut starting = deployed.clone();
        starting.cell.state = JobState::Running;
        assert_eq!(production_step(Some(&starting), Some(&g), false, &LiveHead::Offline).state, StepState::Running);
        let mut failed = deployed.clone();
        failed.cell.state = JobState::Failed;
        assert_eq!(production_step(Some(&failed), Some(&g), false, &LiveHead::Offline).state, StepState::Failed);
        // Nothing deployed (headExport is the newest export): not "Deployed, not producing".
        assert_eq!(production_step(Some(&ready(None)), Some(&g), false, &LiveHead::Offline), step(StepState::Waiting, "Deploy first (step 3)"));
        // Stopped with OLD on the head, NEW generated and not deployed: Deploy, not Start production.
        let old = ready(Some((OLD, "16#1234")));
        let s = production_step(Some(&old), Some(&g), false, &LiveHead::Offline);
        assert_eq!(s.state, StepState::Waiting);
        assert!(s.summary.contains("is not deployed: Deploy it (step 3)"), "{}", s.summary);
        assert_eq!(production_step(Some(&ready(Some((OLD, "16#8E35")))), Some(&g), false, &LiveHead::Offline).state, StepState::Waiting, "same PLAN_ID, another export");
        // Step 2 stale: not done, and not the next step.
        assert_eq!(production_step(Some(&producing), Some(&g), true, &LiveHead::Offline).state, StepState::Stale);
        assert_eq!(production_step(Some(&deployed), Some(&g), true, &LiveHead::Offline).state, StepState::Waiting);
        assert_eq!(production_step(Some(&InfraStatus { ready: false, ..deployed }), Some(&g), false, &LiveHead::Offline).state, StepState::Waiting);
    }

    fn head(state: &str, error: &str, robot: i64, cycles: u64) -> serde_json::Value {
        serde_json::json!({"state": state, "state_code": 99, "error": error, "error_robot": robot, "cycles": cycles, "robots": []})
    }

    fn live(state: &str, error: &str, robot: i64, cycles: u64) -> LiveHead {
        LiveHead::new(Some((&head(state, error, robot, cycles), 0.5)))
    }

    #[test]
    fn live_head_state() {
        let error = head("CS_ERROR", "ERR_ROBOT_STOPPED", 2, 67);
        let view = HeadView::from_json(&error).unwrap();
        assert_eq!(view.label(), "CS_ERROR ERR_ROBOT_STOPPED (robot 2)");
        assert_eq!(LiveHead::new(Some((&error, 3.0))), LiveHead::Live(view.clone()));
        assert_eq!(LiveHead::new(Some((&error, 12.4))), LiveHead::Stale { head: view.clone(), age_s: 12 });
        assert_eq!(LiveHead::new(None), LiveHead::Offline);
        assert_eq!(LiveHead::new(Some((&serde_json::Value::Null, 0.0))), LiveHead::Offline, "the bridge without the head");
        // No error: no ERR_NONE, no robot.
        let running = HeadView::from_json(&head("CS_RUNNING", "ERR_NONE", 0, 3)).unwrap();
        assert_eq!((running.error.clone(), running.error_robot, running.label()), (None, None, "CS_RUNNING".to_owned()));
        assert!(running.producing() && !view.producing());
        // An unnamed state code.
        let unnamed = HeadView::from_json(&serde_json::json!({"state": 77, "error": null})).unwrap();
        assert_eq!(unnamed.label(), "77");
        // Start production again from an error or the recovery, not while it runs or is unknown.
        assert!(live("CS_ERROR", "ERR_ROBOT_STOPPED", 2, 67).needs_start());
        assert!(live("CS_RECOVERY", "ERR_NONE", 0, 67).needs_start());
        assert!(!live("CS_RUNNING", "ERR_NONE", 0, 67).needs_start());
        assert!(!LiveHead::Stale { head: view, age_s: 30 }.needs_start());
        assert!(!LiveHead::Offline.needs_start());
        assert_eq!(live("CS_ERROR", "ERR_ROBOT_STOPPED", 2, 67).line().1, theme::current().danger);
        assert_eq!(live("CS_RUNNING", "ERR_NONE", 0, 67).line(), ("Head: CS_RUNNING · 67 cycles".to_owned(), theme::current().ok));
    }

    #[test]
    fn production_by_the_live_head() {
        let g = generated(NEW, "16#8E35");
        let deployed = ready(Some((NEW, "16#8E35")));
        // The backend still says in production from the last job (CS_RUNNING then).
        let mut producing = deployed.clone();
        producing.in_production = true;
        producing.cell.state = JobState::Done;
        producing.cell.head_state = Some("CS_RUNNING".into());
        let at = |state: &str, error: &str, robot: i64| production_step(Some(&producing), Some(&g), false, &live(state, error, robot, 67));
        assert_eq!(at("CS_RUNNING", "ERR_NONE", 0), step(StepState::Done, "In production · cycle 67"));
        assert_eq!(at("CS_CYCLE_DONE", "ERR_NONE", 0), step(StepState::Done, "In production · cycle 67"));
        assert_eq!(
            at("CS_ERROR", "ERR_ROBOT_STOPPED", 2),
            step(StepState::Failed, "Error: ERR_ROBOT_STOPPED (robot 2) · Start production to recover")
        );
        assert_eq!(at("CS_ERROR", "ERR_SAFETY_LOST", 0), step(StepState::Failed, "Error: ERR_SAFETY_LOST · Start production to recover"));
        assert_eq!(at("CS_ERROR", "ERR_OPERATOR_STOP", 0), step(StepState::Action, "Stopped (ERR_OPERATOR_STOP) · Start production"));
        assert_eq!(at("CS_RECOVERY", "ERR_NONE", 0), step(StepState::Action, "Recovery required · Start production"));
        assert_eq!(at("CS_START_JOBS", "ERR_NONE", 0), step(StepState::Running, "Head in CS_START_JOBS…"));
        // The head waits (CS_INIT), whatever the last job said.
        assert_eq!(at("CS_INIT", "ERR_NONE", 0), step(StepState::Action, "Deployed, not producing (CS_INIT): Start production"));
        // Producing other code than generated, or with step 2 stale: still stale.
        let older = generated(OLD, "16#1234");
        let running = live("CS_RUNNING", "ERR_NONE", 0, 67);
        assert_eq!(production_step(Some(&producing), Some(&older), false, &running).state, StepState::Stale);
        assert_eq!(production_step(Some(&producing), Some(&g), true, &running).state, StepState::Stale);
        // A running head the backend does not count as in production (started elsewhere).
        assert_eq!(production_step(Some(&deployed), Some(&g), false, &running), step(StepState::Done, "In production · cycle 67"));
        // No live data for more than 10 s: stale, with the job's word.
        let quiet = LiveHead::new(Some((&head("CS_RUNNING", "ERR_NONE", 0, 67), 25.0)));
        assert_eq!(
            production_step(Some(&producing), Some(&g), false, &quiet),
            step(StepState::Stale, "In production (CS_RUNNING) by the last job · no live head state for 25 s")
        );
        // A stale error does not decide: the job's state does.
        let old_error = LiveHead::new(Some((&head("CS_ERROR", "ERR_ROBOT_STOPPED", 2, 67), 25.0)));
        assert_eq!(production_step(Some(&producing), Some(&g), false, &old_error).state, StepState::Stale);
        // A job still running comes first.
        let mut starting = producing.clone();
        starting.cell.state = JobState::Running;
        assert_eq!(production_step(Some(&starting), Some(&g), false, &live("CS_ERROR", "ERR_ROBOT_STOPPED", 2, 67)).state, StepState::Running);
    }
}
