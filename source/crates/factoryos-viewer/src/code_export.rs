//! Control Code Export: the guided flow's step 2 "Generate PLC + robot code" form (in the sidebar)
//! and its code page (the main area, in place of the 3D view: see [`crate::code_layout`]).
//!
//! The backend builds a saved plan of the active cell with the plan2cell CLI ([`api::CODEGEN_BUILD`])
//! and publishes the progress on [`keys::CODEGEN_STATUS`](factoryos_client::protocol::keys); the page
//! then shows the build: its summary, the diagnostics by severity, the files (searchable), the zip,
//! and the selected file in a large read-only code viewer ([`crate::code_view`]), served by
//! `GET` [`api::CODEGEN_BUILD_FILE`]. Planned PLC dialects and robot languages are listed but disabled.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::codegen::{
    BuildAccepted, BuildFile, BuildInfo, BuildList, BuildRequest, Choice, CodegenOptions, CodegenState, CodegenStatus,
    Diagnostic, Severity,
};
use factoryos_client::protocol::{PlanInfo, PlansInfo, api};
use web_time::{Duration, Instant};

use crate::theme;
use crate::code_view::Code;
use crate::rr::egui::{self, Color32, RichText};
use crate::rr::re_ui::{self, UiExt as _};
use crate::sidebar;

/// Files larger than this are not previewed (the archive has them).
const PREVIEW_MAX: u64 = 512 * 1024;
/// How long to wait before asking for the options again after they failed (no binary yet, …).
const OPTIONS_RETRY: Duration = Duration::from_secs(15);
/// … or came for another cell than the one shown (the plans, which name it, lag behind a switch).
const OPTIONS_CELL_RETRY: Duration = Duration::from_secs(1);

enum Msg {
    Options(Result<CodegenOptions, String>),
    Builds(Result<BuildList, String>),
    Info(String, Box<Result<BuildInfo, String>>),
    File { id: String, path: String, text: Result<String, String> },
    Submitted(Result<BuildAccepted, String>),
}

/// The export Run Control Code deploys ([`CodeExport::deploy_choice`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeployChoice {
    /// Its build id.
    pub build: String,
    pub plan_id: Option<String>,
    /// Picked by hand in the results (else the newest).
    pub picked: bool,
}

/// A file of a build being viewed, and its code once fetched.
struct Preview {
    id: String,
    path: String,
    code: Option<Result<Code, String>>,
    /// The line it was opened at (a diagnostic's), marked in the viewer …
    line: Option<usize>,
    /// … and scrolled to once its code is shown.
    scroll_to: Option<usize>,
}

pub struct CodeExport {
    api: Api,
    inbox: Arc<Mutex<Vec<Msg>>>,
    /// The code page is shown (in place of the 3D view, see [`crate::code_layout`]).
    pub open: bool,
    /// The search over the files' paths.
    filter: String,
    options: Option<Result<CodegenOptions, String>>,
    options_asked: Option<Instant>,
    /// The picked plan (a [`PlanInfo::path`]); `None`: the one the executor plays, else the newest.
    plan: Option<String>,
    /// The form's options, from the backend's defaults for the active cell (its plan2cell.toml on
    /// plan2cell's defaults) once they arrive; again for each cell.
    request: Option<BuildRequest>,
    builds: BuildList,
    /// The active cell the builds were listed for.
    cell: Option<String>,
    selected: Option<String>,
    /// The automation's `export_code`: the export button is pressed as soon as it is enabled.
    export_requested: bool,
    /// The build picked by hand in the results (Run Control Code deploys it instead of the newest).
    picked: Option<String>,
    info: Option<(String, Result<BuildInfo, String>)>,
    /// The files open as tabs (Kiro Crew's file explorer), and the one shown.
    previews: Vec<Preview>,
    active: usize,
    /// "Ask the agent about this file", for the app to open in the chat: (title, prompt).
    chat_request: Option<(String, String)>,
    status: CodegenStatus,
    /// The build this viewer started: its results open when it ends.
    awaiting: Option<String>,
    submitting: bool,
    submit_error: Option<String>,
    /// The plan the form shows (a [`PlanInfo::path`]), for the Hardware Configuration's robots
    /// before there is an export.
    shown_plan: Option<String>,
}

impl CodeExport {
    pub fn new(api: Api) -> Self {
        Self {
            api,
            inbox: Arc::default(),
            open: false,
            filter: String::new(),
            options: None,
            options_asked: None,
            plan: None,
            request: None,
            builds: BuildList::default(),
            cell: None,
            selected: None,
            export_requested: false,
            picked: None,
            info: None,
            previews: Vec::new(),
            active: 0,
            chat_request: None,
            status: CodegenStatus::default(),
            awaiting: None,
            submitting: false,
            submit_error: None,
            shown_plan: None,
        }
    }

    /// The export Run Control Code deploys: the successful build picked in the results, else the
    /// newest successful one (`None`: no builds listed yet).
    pub fn deploy_choice(&self) -> Option<DeployChoice> {
        let builds = &self.builds.builds;
        let (entry, picked) = match self.picked.as_deref().and_then(|id| builds.iter().find(|b| b.id == id && b.ok)) {
            Some(entry) => (entry, true),
            None => (builds.iter().find(|b| b.ok)?, false),
        };
        let plan_id = entry.plan_id.clone().or_else(|| match &self.info {
            Some((id, Ok(info))) if *id == entry.id => info.summary.as_ref().map(|s| s.plan_id.clone()).filter(|p| !p.is_empty()),
            _ => None,
        });
        Some(DeployChoice { build: entry.id.clone(), plan_id, picked })
    }

    /// What the guided flow's step 2 shows; `hardware`: the cell's saved Hardware Configuration
    /// (`None`: not loaded yet), to tell whether it changed since the chosen export.
    pub fn flow_facts(&self, hardware: Option<&crate::flow::SavedHardware>) -> crate::flow::ExportFacts {
        let choice = self.deploy_choice();
        let chosen = choice.as_ref().and_then(|c| {
            let entry = self.builds.builds.iter().find(|b| b.id == c.build)?;
            Some(crate::flow::ExportBrief {
                id: entry.id.clone(),
                plan: plan_stem(&entry.plan).to_owned(),
                plan_id: c.plan_id.clone(),
                stale: entry.stale,
                warnings: entry.warnings,
                picked: c.picked,
                hardware_changed: crate::flow::hardware_changed(entry.hardware_etag.as_deref(), entry.created_s, hardware),
            })
        });
        let failed = (self.status.state == CodegenState::Failed && self.of_this_cell(&self.status))
            .then(|| format!("✗ {}", self.status.message.clone().unwrap_or_else(|| "the export failed".to_owned())))
            .or_else(|| self.submit_error.clone());
        crate::flow::ExportFacts {
            running: self.status.state == CodegenState::Running || self.submitting,
            failed,
            chosen,
            shown_plan: self.shown_plan.as_deref().map(|p| plan_stem(p).to_owned()),
        }
    }

    /// The plan whose robots the Hardware Configuration lists: the plan of the export the next
    /// steps use (picked in the results, else the newest), else the plan the form shows.
    pub fn hardware_plan(&self) -> Option<String> {
        let choice = self.deploy_choice();
        choice
            .and_then(|c| self.builds.builds.iter().find(|b| b.id == c.build))
            .map(|b| b.plan.clone())
            .filter(|p| !p.is_empty())
            .or_else(|| self.shown_plan.clone())
    }

    /// Read the builds again (their STALE flags), e.g. after the hardware configuration was saved.
    pub fn refresh(&self, ctx: &egui::Context) {
        self.refresh_builds(ctx);
    }

    fn sender(&self, ctx: &egui::Context) -> impl Fn(Msg) + Send + Sync + 'static {
        let (inbox, ctx) = (self.inbox.clone(), ctx.clone());
        move |msg| {
            inbox.lock().unwrap().push(msg);
            ctx.request_repaint();
        }
    }

    /// Call once per frame: answers, the build status and the active cell.
    pub fn poll(&mut self, ctx: &egui::Context, status: &CodegenStatus, cell: &str) {
        let ask = match (&self.options, self.options_asked) {
            (None, None) => true,
            (Some(Err(_)), Some(at)) => at.elapsed() > OPTIONS_RETRY,
            (Some(Ok(_)), Some(at)) => self.request.is_none() && at.elapsed() > OPTIONS_CELL_RETRY,
            _ => false,
        };
        if ask {
            self.options_asked = Some(Instant::now());
            let send = self.sender(ctx);
            self.api.get(api::CODEGEN_OPTIONS, move |options| send(Msg::Options(options)));
        }
        let messages = std::mem::take(&mut *self.inbox.lock().unwrap());
        for msg in messages {
            self.receive(ctx, msg);
        }
        if self.cell.as_deref() != Some(cell) {
            // Builds are looked up in the active cell: nothing of the previous one stays.
            self.cell = Some(cell.to_owned());
            self.builds = BuildList::default();
            self.selected = None;
            self.picked = None;
            self.info = None;
            self.previews.clear();
            self.plan = None;
            self.awaiting = None;
            // The form starts from this cell's own settings.
            self.request = None;
            self.options_asked = None;
            if matches!(self.options, Some(Ok(_))) {
                self.options = None;
            }
            self.refresh_builds(ctx);
        }
        if *status != self.status {
            let ended = matches!(status.state, CodegenState::Done | CodegenState::Failed);
            if ended && status.build_id.is_some() {
                self.refresh_builds(ctx);
                if status.build_id == self.awaiting {
                    self.awaiting = None;
                    if let Some(id) = status.build_id.clone()
                        && self.of_this_cell(status)
                    {
                        self.open = true;
                        // The new export is the newest: Run Control Code takes it.
                        self.picked = None;
                        self.select(ctx, id);
                    }
                }
            }
            self.status = status.clone();
        }
    }

    /// Whether `status` is about a build of the cell shown (older backends do not say: yes).
    fn of_this_cell(&self, status: &CodegenStatus) -> bool {
        status.cell.is_none() || status.cell == self.cell
    }

    fn receive(&mut self, ctx: &egui::Context, msg: Msg) {
        match msg {
            Msg::Options(options) => {
                // Defaults for another cell (asked for before a switch) do not seed the form; it
                // asks again (see `poll`).
                if let (Ok(options), None) = (&options, &self.request)
                    && self.cell.as_deref().is_none_or(|cell| cell == options.cell)
                {
                    self.request = Some(BuildRequest::new("", &options.defaults));
                }
                self.options = Some(options);
            }
            // A list asked for before a cell switch.
            Msg::Builds(Ok(list)) if self.cell.as_deref().is_some_and(|cell| cell != list.cell) => {}
            Msg::Builds(Ok(list)) => {
                if self.selected.is_none()
                    && let Some(first) = list.builds.first()
                {
                    let id = first.id.clone();
                    self.builds = list;
                    self.select(ctx, id);
                } else {
                    self.builds = list;
                }
            }
            Msg::Builds(Err(err)) => re_log_warn(&format!("Control Code Export: listing the builds: {err}")),
            Msg::Info(id, info) => {
                if self.selected.as_deref() == Some(id.as_str()) {
                    self.info = Some((id, *info));
                }
            }
            Msg::File { id, path, text } => {
                if let Some(preview) = self.previews.iter_mut().find(|p| p.id == id && p.path == path) {
                    preview.scroll_to = preview.line;
                    preview.code = Some(text.map(|text| Code::new(&path, &text)));
                }
            }
            Msg::Submitted(result) => {
                self.submitting = false;
                match result {
                    Ok(accepted) => {
                        self.submit_error = None;
                        // Its end can come over the bus before this answer does.
                        let ended = matches!(self.status.state, CodegenState::Done | CodegenState::Failed)
                            && self.status.build_id.as_deref() == Some(accepted.build_id.as_str());
                        if !ended {
                            self.awaiting = Some(accepted.build_id);
                        } else if self.of_this_cell(&self.status.clone()) {
                            self.open = true;
                            self.picked = None;
                            self.select(ctx, accepted.build_id);
                        }
                    }
                    Err(err) => self.submit_error = Some(err),
                }
            }
        }
    }

    fn refresh_builds(&self, ctx: &egui::Context) {
        let send = self.sender(ctx);
        self.api.get(api::CODEGEN_BUILDS, move |list| send(Msg::Builds(list)));
    }

    fn select(&mut self, ctx: &egui::Context, id: String) {
        self.selected = Some(id.clone());
        self.info = None;
        self.previews.clear();
        let send = self.sender(ctx);
        self.api.get(&api::codegen_build(&id), move |info| send(Msg::Info(id, Box::new(info))));
    }

    /// View `file` of build `id`; `line`: the line to show (a diagnostic's). Open already: its tab
    /// is shown; else it opens in a new tab with `new_tab`, in place of the shown one without.
    fn open_file(&mut self, ctx: &egui::Context, id: &str, file: &BuildFile, line: Option<usize>, new_tab: bool) {
        if let Some(index) = self.previews.iter().position(|p| p.id == id && p.path == file.path) {
            self.active = index;
            let preview = &mut self.previews[index];
            if line.is_some() {
                preview.line = line;
                preview.scroll_to = line;
            }
            return;
        }
        let preview = |code| Preview { id: id.to_owned(), path: file.path.clone(), code, line, scroll_to: None };
        let too_large = (file.bytes > PREVIEW_MAX)
            .then(|| format!("{} is {} KiB: too large to show here; download the archive", file.path, file.bytes / 1024));
        let new = preview(too_large.clone().map(Err));
        if new_tab || self.previews.is_empty() {
            self.previews.push(new);
            self.active = self.previews.len() - 1;
        } else {
            self.active = self.active.min(self.previews.len() - 1);
            self.previews[self.active] = new;
        }
        if too_large.is_some() {
            return;
        }
        let send = self.sender(ctx);
        let (id, path) = (id.to_owned(), file.path.clone());
        self.api.get_text(&api::codegen_file(&id, &path), (PREVIEW_MAX as usize) * 2, move |text| {
            send(Msg::File { id, path, text });
        });
    }

    /// For the automation's `state`: the export's status as the sidebar shows it, the build the
    /// code page shows, and whether an `export_code` still waits for the button.
    pub fn automation_facts(&self) -> serde_json::Value {
        serde_json::json!({
            "status": self.status,
            "submitting": self.submitting,
            "requested": self.export_requested,
            "open": self.open,
            "selected": self.selected,
            "plan": self.shown_plan,
        })
    }

    /// Press "Export PLC + robot code" (the automation's `export_code`), on `plan` (a
    /// `PlanInfo::path`) if given, else on the form's plan; as soon as the button is enabled.
    pub fn request_export(&mut self, plan: Option<String>) {
        if plan.is_some() {
            self.plan = plan;
        }
        self.export_requested = true;
    }

    fn submit(&mut self, ctx: &egui::Context, request: BuildRequest) {
        self.submitting = true;
        self.submit_error = None;
        let send = self.sender(ctx);
        self.api.post_json(api::CODEGEN_BUILD, &request, move |accepted| send(Msg::Submitted(accepted)));
    }

    /// The zip of build `id`, for the browser. On the web the API base has the viewer's workspace
    /// prefix; natively, behind the edge, the base is the bare origin and the workspace a cookie of
    /// the viewer's own requests, which the system browser does not have: the URL names it.
    fn archive_url(&self, id: &str) -> String {
        let base = self.api.base();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let workspace = factoryos_client::auth::origin(base).and_then(|origin| factoryos_client::auth::workspace(&origin));
            archive_url(base, workspace.as_deref(), id)
        }
        #[cfg(target_arch = "wasm32")]
        format!("{base}{}", api::codegen_archive(id))
    }

    /// The form in the sidebar's "Generate PLC + robot code" box (step 2). `loaded` is the plan the
    /// behaviour-tree executor plays (its name), if any; `hardware` what the Hardware
    /// Configuration says about the builds ("Uses hardware config (saved)").
    pub fn form_ui(&mut self, ui: &mut egui::Ui, plans: &PlansInfo, loaded: Option<&str>, hardware: Option<&str>) {
        let plans = &plans.plans;
        if self.plan.as_ref().is_some_and(|p| !plans.iter().any(|plan| plan.path == *p)) {
            self.plan = None;
        }
        let default_plan = loaded
            .and_then(|name| plans.iter().find(|p| plan_stem(&p.path) == name || p.tree_id == name))
            .or_else(|| plans.first())
            .map(|p| p.path.clone());
        let selected = self.plan.clone().or(default_plan);
        self.shown_plan = selected.clone();

        ui.small(RichText::new("Control Code Export: plan → head PLC + robot programs").weak());
        let label = |path: &str| plans.iter().find(|p| p.path == path).map(plan_label).unwrap_or_default();
        ui.add_enabled_ui(!plans.is_empty(), |ui| {
            let text = selected.as_deref().map_or_else(|| "No behaviour trees yet".to_owned(), label);
            let width = ui.available_width();
            theme::truncated_combo(ui, "codegen_plan", text, width, |ui| {
                for plan in plans {
                    let mut text = plan_label(plan);
                    if loaded.is_some_and(|name| plan_stem(&plan.path) == name) {
                        text.push_str(" (in the executor)");
                    }
                    if ui.selectable_label(selected.as_deref() == Some(plan.path.as_str()), text).clicked() {
                        self.plan = Some(plan.path.clone());
                    }
                }
            });
        });

        let options = match &self.options {
            None => {
                ui.small(RichText::new("Loading the export options…").weak());
                return;
            }
            Some(Err(err)) => {
                ui.label(RichText::new(format!("Control Code Export is unavailable: {err}")).small().color(theme::current().danger));
                return;
            }
            Some(Ok(options)) => options.clone(),
        };
        let Some(request) = self.request.as_mut() else { return };
        // The export options: Advanced only (Simple exports with their current values).
        crate::ui_mode::advanced(ui, |ui| {
        egui::Grid::new("codegen_options").num_columns(2).spacing([6.0, 4.0]).show(ui, |ui| {
            choice_row(ui, "codegen_profile", "Profile", &options.profiles, &mut request.profile);
            choice_row(ui, "codegen_dialect", "PLC dialect", &options.plc_dialects, &mut request.plc_dialect);
            choice_row(ui, "codegen_language", "Robot language", &options.robot_languages, &mut request.robot_language);
            choice_row(ui, "codegen_job_number", "Job number", &options.job_numbers, &mut request.job_number);
        });
        ui.checkbox(&mut request.robot_sim, "Robot simulator projects")
            .on_hover_text("A CODESYS device-simulator project per robot next to the head PLC (profile sim)");
        ui.checkbox(&mut request.opcua_symbols, "OPC UA symbols")
            .on_hover_text("A Symbol Configuration in the head project that exports the cell status read-only over OPC UA");
        ui.add_enabled(options.st_check_available, egui::Checkbox::new(&mut request.st_check, "ST type check (trust)"))
            .on_hover_text("Type-check the generated Structured Text before writing it")
            .on_disabled_hover_text("This plan2cell build has no ST check (feature `check`)");
        ui.checkbox(&mut request.strict, "Strict (warnings are errors)");
        if request.profile == "vw-ks24" {
            ui.small(RichText::new("vw-ks24: job number BYTE and no robot simulators (plan2cell adjusts them).").color(theme::current().warn));
        }

        if let Some(note) = hardware {
            ui.small(RichText::new(note).weak())
                .on_hover_text("The head network, station names, device names and IPs of the build (Hardware Configuration, below)");
        }
        });
        let implemented = |choices: &[Choice], id: &str| choices.iter().any(|c| c.id == id && c.implemented);
        let ready = implemented(&options.plc_dialects, &request.plc_dialect) && implemented(&options.robot_languages, &request.robot_language);
        let running = self.status.state == CodegenState::Running || self.submitting;
        let enabled = selected.is_some() && ready && !running;
        let label = if running { "Exporting…" } else { "Export PLC + robot code" };
        let clicked = ui.add_enabled_ui(enabled, |ui| sidebar::button(ui, label)).inner;
        let requested = enabled && std::mem::take(&mut self.export_requested);
        if (clicked || requested)
            && let Some(plan) = selected.clone()
        {
            let request = BuildRequest { plan, ..request.clone() };
            self.submit(ui.ctx(), request);
        }

        let (text, color) = match self.status.state {
            CodegenState::Idle => (String::new(), theme::current().warn),
            CodegenState::Running => (self.status.message.clone().unwrap_or_else(|| "Building…".to_owned()), theme::current().warn),
            CodegenState::Done => (format!("✓ {}", self.status.message.clone().unwrap_or_default()), theme::current().ok),
            CodegenState::Failed => (format!("✗ {}", self.status.message.clone().unwrap_or_default()), theme::current().danger),
        };
        if !text.is_empty() {
            ui.label(RichText::new(text).small().color(color));
        }
        if let Some(err) = &self.submit_error {
            ui.label(RichText::new(err).small().color(theme::current().danger));
        }
        let results = if self.open { "Close the code (back to the 3D view)" } else { "Show the code" };
        if !self.builds.builds.is_empty() && sidebar::secondary_button(ui, results) {
            self.open = !self.open;
        }
    }

    /// The code page, in the main area, laid out like Kiro Crew's file explorer: the build at the
    /// top; at the left its summary, diagnostics and file tree (searchable); at the right the open
    /// files as tabs, the shown one's path and its code.
    pub fn page_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let p = theme::palette(ui);
        egui::Frame::new().fill(p.bg).inner_margin(egui::Margin::symmetric(10, 8)).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Control Code Export").size(16.0).strong().color(p.text_strong));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let back = egui::Button::new(RichText::new("✕  Back to the 3D view").color(p.text))
                        .fill(p.card)
                        .stroke(egui::Stroke::new(1.0, p.border))
                        .corner_radius(6);
                    if ui
                        .add(back)
                        .on_hover_text("Close the code: the 3D view, its panes and the timeline come back")
                        .clicked()
                    {
                        self.open = false;
                    }
                    if ui.add(icon_button("↻", p)).on_hover_text("Re-list the builds").clicked() {
                        self.refresh_builds(&ctx);
                        if let Some(id) = self.selected.clone() {
                            self.select(&ctx, id);
                        }
                    }
                });
            });
            ui.add_space(6.0);
            if self.builds.builds.is_empty() {
                ui.label(RichText::new("No builds of this cell yet: Export PLC + robot code in the sidebar.").color(p.muted));
                return;
            }
            let mut pick = None;
            let selected_text = self
                .selected
                .as_ref()
                .and_then(|id| self.builds.builds.iter().find(|b| b.id == *id))
                .map_or_else(|| "Pick a build".to_owned(), entry_label);
            egui::ComboBox::from_id_salt("codegen_builds")
                .selected_text(selected_text)
                .width(ui.available_width().min(560.0))
                .truncate()
                .show_ui(ui, |ui| {
                    for build in &self.builds.builds {
                        if ui.selectable_label(self.selected.as_deref() == Some(build.id.as_str()), entry_label(build)).clicked() {
                            pick = Some(build.id.clone());
                        }
                    }
                });
            if let Some(id) = pick {
                self.picked = Some(id.clone());
                self.select(&ctx, id);
            }
            let info = match &self.info {
                None => {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label(RichText::new("Loading the build…").color(p.muted));
                    });
                    return;
                }
                Some((_, Err(err))) => {
                    ui.colored_label(p.danger, err);
                    return;
                }
                Some((_, Ok(info))) => info.clone(),
            };
            ui.add_space(6.0);
            if self.previews.is_empty()
                && let Some(file) = default_file(&info.files).cloned()
            {
                self.open_file(&ctx, &info.id, &file, None, false);
            }

            let mut action = None;
            let archive_url = self.archive_url(&info.id);
            let viewing = self.previews.get(self.active).map(|p| p.path.clone());
            let max_width = (ui.available_width() * 0.6).max(260.0);
            egui::Panel::left("codegen_page_files")
                .resizable(true)
                .default_size(340.0_f32.min(max_width))
                .min_size(240.0_f32.min(max_width))
                .max_size(max_width)
                .frame(egui::Frame::new().inner_margin(egui::Margin { right: 8, ..Default::default() }))
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().id_salt("codegen_results").auto_shrink([false, false]).show(ui, |ui| {
                        card(ui, p, |ui| build_header_ui(ui, &info, &archive_url));
                        ui.add_space(6.0);
                        if let Some((file, line)) = diagnostics_ui(ui, &info) {
                            action = Some(FileAction::Open { path: file, line, new_tab: false });
                        }
                        ui.add_space(6.0);
                        search_ui(ui, &mut self.filter, p);
                        let matches = filter_files(&info.files, &self.filter);
                        let title = if self.filter.trim().is_empty() {
                            format!("FILES  {}", info.files.len())
                        } else {
                            format!("FILES  {} of {}", matches.len(), info.files.len())
                        };
                        ui.add_space(4.0);
                        ui.label(RichText::new(title).small().color(p.muted));
                        let clicked = if self.filter.trim().is_empty() {
                            file_tree_ui(ui, &info.files, viewing.as_deref(), p)
                        } else {
                            file_list_ui(ui, &matches, viewing.as_deref(), p)
                        };
                        if clicked.is_some() {
                            action = clicked;
                        }
                    });
                });
            egui::CentralPanel::no_frame().show(ui, |ui| {
                if let Some(next) = self.viewer_ui(ui, &info, &archive_url) {
                    action = Some(next);
                }
            });
            match action {
                Some(FileAction::Open { path, line, new_tab }) => {
                    if let Some(file) = info.files.iter().find(|f| f.path == path).cloned() {
                        self.open_file(&ctx, &info.id, &file, line, new_tab);
                    }
                }
                Some(FileAction::Ask(path)) => {
                    let name = path.rsplit('/').next().unwrap_or(&path).to_owned();
                    let prompt = format!(
                        "In the Control Code Export build `{}` (plan `{}`) of the active cell, explain `{path}`: what it \
                         does, and how it ties into the rest of the generated PLC and robot code.",
                        info.id,
                        plan_stem(&info.plan),
                    );
                    self.chat_request = Some((format!("Code {name}"), prompt));
                }
                Some(FileAction::Filter(folder)) => self.filter = folder,
                None => {}
            }
        });
    }

    /// "Ask the agent about this file" from the page, for the app to open in the chat.
    pub fn take_chat_request(&mut self) -> Option<(String, String)> {
        self.chat_request.take()
    }

    /// The open files' tabs, the shown one's path and actions, and its code filling the space.
    fn viewer_ui(&mut self, ui: &mut egui::Ui, info: &BuildInfo, archive_url: &str) -> Option<FileAction> {
        let p = theme::palette(ui);
        let mut action = None;
        if self.previews.is_empty() {
            ui.centered_and_justified(|ui| ui.label(RichText::new("Pick a file at the left to see its code.").color(p.muted)));
            return None;
        }
        self.active = self.active.min(self.previews.len() - 1);
        if let Some(close) = self.tabs_ui(ui, p) {
            self.previews.remove(close);
            if self.active >= close && self.active > 0 {
                self.active -= 1;
            }
            if self.previews.is_empty() {
                return None;
            }
        }
        let preview = &mut self.previews[self.active];

        // The path as breadcrumbs (a click filters the tree to that folder), and the build.
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            egui::Frame::new().fill(p.card).stroke(egui::Stroke::new(1.0, p.border)).corner_radius(6).inner_margin(egui::Margin::symmetric(8, 3)).show(ui, |ui| {
                ui.spacing_mut().item_spacing.x = 2.0;
                let parts: Vec<&str> = preview.path.split('/').collect();
                for (i, part) in parts.iter().enumerate() {
                    if i > 0 {
                        ui.label(RichText::new("/").monospace().color(p.muted_strong));
                    }
                    let last = i + 1 == parts.len();
                    let text = RichText::new(*part).monospace().color(if last { p.text } else { p.muted });
                    if last {
                        ui.label(text);
                    } else if ui.add(egui::Label::new(text).sense(egui::Sense::click())).on_hover_text("Show this folder's files").clicked() {
                        action = Some(FileAction::Filter(format!("{}/", parts[..=i].join("/"))));
                    }
                }
            });
            egui::Frame::new().fill(p.accent_subtle).corner_radius(10).inner_margin(egui::Margin::symmetric(8, 2)).show(ui, |ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                ui.label(RichText::new(plan_stem(&info.plan)).monospace().small().color(p.accent));
                let warnings = info.diagnostics.iter().filter(|d| d.severity == Severity::Warning).count();
                if warnings > 0 {
                    ui.label(RichText::new(format!("{warnings} ⚠")).monospace().small().color(p.warn));
                }
            })
            .response
            .on_hover_text(format!("Build {}", stamp_text(&info.id)));
        });

        // The file's name and what it is; its actions.
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let name = preview.path.rsplit('/').next().unwrap_or(&preview.path);
            ui.label(RichText::new(name).size(14.0).strong().color(p.text_strong));
            let lang = match crate::code_view::language(&preview.path) {
                crate::code_view::Lang::St => "Structured Text",
                crate::code_view::Lang::Krl => "KRL",
                crate::code_view::Lang::Plain => "",
            };
            let mut meta = Vec::new();
            if !lang.is_empty() {
                meta.push(lang.to_owned());
            }
            if let Some(Ok(code)) = &preview.code {
                meta.push(format!("{} lines", code.lines.len()));
            }
            if let Some(line) = preview.line {
                meta.push(format!("diagnostic at line {line}"));
            }
            ui.label(RichText::new(meta.join(" · ")).small().color(p.muted));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.small_icon_button(&re_ui::icons::DOWNLOAD, "Download the build (.zip)").on_hover_text("Download the build (.zip)").clicked() {
                    crate::workspace_ui::open_url(archive_url);
                }
                if let Some(Ok(code)) = &preview.code
                    && ui.small_icon_button(&re_ui::icons::COPY, "Copy the file").on_hover_text("Copy the whole file").clicked()
                {
                    ui.ctx().copy_text(code.text.clone());
                }
                if ui.small_icon_button(&re_ui::icons::AGENT, "Ask the agent").on_hover_text("Ask the agent about this file").clicked() {
                    action = Some(FileAction::Ask(preview.path.clone()));
                }
            });
        });
        let rect = ui.available_rect_before_wrap();
        ui.painter().hline(rect.x_range(), rect.top() + 2.0, egui::Stroke::new(1.0, p.border));
        ui.add_space(6.0);
        let scroll_to = preview.scroll_to.take();
        match &preview.code {
            None => {
                ui.spinner();
            }
            Some(Err(err)) => {
                ui.colored_label(p.warn, err);
            }
            Some(Ok(code)) => code.ui(ui, egui::Id::new(("codegen_code", &preview.id, &preview.path)), preview.line, scroll_to),
        }
        action
    }

    /// The open files as folder tabs (Kiro Crew's file explorer); returns the one closed.
    fn tabs_ui(&mut self, ui: &mut egui::Ui, p: &theme::Palette) -> Option<usize> {
        let mut close = None;
        let mut active_rect = None;
        let strip = ui.available_rect_before_wrap();
        egui::ScrollArea::horizontal().id_salt("codegen_tabs").auto_shrink([false, true]).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 2.0;
                for (index, preview) in self.previews.iter().enumerate() {
                    let active = index == self.active;
                    let name = preview.path.rsplit('/').next().unwrap_or(&preview.path);
                    let inner = egui::Frame::new()
                        .fill(if active { p.bg } else { p.card })
                        .stroke(egui::Stroke::new(1.0, p.border))
                        .corner_radius(egui::CornerRadius { nw: 6, ne: 6, sw: 0, se: 0 })
                        .inner_margin(egui::Margin { left: 10, right: 6, top: 5, bottom: 5 })
                        .show(ui, |ui| {
                            ui.set_max_width(200.0);
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 6.0;
                                ui.add(egui::Label::new(RichText::new(name).small().color(if active { p.text } else { p.muted })).truncate());
                                let x = egui::Button::new(RichText::new("×").small().color(p.muted)).frame(false);
                                if ui.add(x).on_hover_text("Close").clicked() {
                                    close = Some(index);
                                }
                            });
                        });
                    let rect = inner.response.rect;
                    if active {
                        active_rect = Some(rect);
                    }
                    let response = ui.interact(rect, ui.id().with(("codegen_tab", index)), egui::Sense::click());
                    if response.clicked() {
                        self.active = index;
                    }
                    if response.middle_clicked() {
                        close = Some(index);
                    }
                    response.on_hover_text(&preview.path);
                }
            });
        });
        // The strip's bottom edge, open under the shown tab (into its code).
        let bottom = ui.min_rect().bottom();
        ui.painter().hline(strip.x_range(), bottom, egui::Stroke::new(1.0, p.border));
        if let Some(rect) = active_rect {
            ui.painter().hline(rect.x_range().shrink(1.0), bottom, egui::Stroke::new(1.0, p.bg));
        }
        close
    }
}

/// `base` + the archive path; with `workspace` (selected by cookie) as a path prefix when `base` is
/// the bare origin.
#[cfg(not(target_arch = "wasm32"))]
fn archive_url(base: &str, workspace: Option<&str>, id: &str) -> String {
    use factoryos_client::protocol::workspace;

    workspace::pinned_url(base, workspace, &api::codegen_archive(id))
}

fn re_log_warn(message: &str) {
    crate::rr::re_log::warn!("{message}");
}

/// A combo box row of a grid; planned (not implemented) entries are shown but disabled.
fn choice_row(ui: &mut egui::Ui, id: &str, label: &str, choices: &[Choice], value: &mut String) {
    ui.label(RichText::new(label).small());
    let text = choices.iter().find(|c| c.id == *value).map_or_else(|| value.clone(), |c| c.label.clone());
    let width = ui.available_width();
    theme::truncated_combo(ui, id, text, width, |ui| {
        for choice in choices {
            if choice.implemented {
                let response = ui.selectable_value(value, choice.id.clone(), &choice.label);
                if !choice.description.is_empty() {
                    response.on_hover_text(&choice.description);
                }
            } else {
                ui.add_enabled(false, egui::Button::selectable(false, format!("{} (planned)", choice.label)))
                    .on_disabled_hover_text("not implemented yet");
            }
        }
    });
    ui.end_row();
}

/// `bt_… · hier_la_fu · 20 ops` (`· unverified` when an export check failed, `· earlier scene` when
/// planned before the cell was rebuilt from its scene); the planner is left out for a backend that
/// does not report it.
pub(crate) fn plan_label(plan: &PlanInfo) -> String {
    let mut label = plan.tree_id.clone();
    if !plan.planner.is_empty() {
        label.push_str(&format!(" · {}", plan.planner));
    }
    label.push_str(&format!(" · {} ops", plan.num_operations));
    if !plan.status.is_empty() && plan.status != "ok" {
        label.push_str(&format!(" · {}", plan.status));
    }
    if plan.stale {
        // A reason: its recorded inputs (scene, URDFs, products) changed; none: an older backend's
        // "planned before the cell was rebuilt".
        label.push_str(if plan.stale_reason.is_some() { " · stale" } else { " · earlier scene" });
    }
    label
}

fn plan_stem(path: &str) -> &str {
    let name = path.rsplit('/').next().unwrap_or(path);
    name.strip_suffix(".json").unwrap_or(name)
}

/// `2026-10-12 15:30:45 UTC · weld_plan` from a build id (`20261012T153045123Z-weld_plan`).
pub(crate) fn export_label(id: &str) -> String {
    match id.get(20..).filter(|_| id.as_bytes().get(19) == Some(&b'-')) {
        Some(plan) => format!("{} · {plan}", stamp_text(id)),
        None => id.to_owned(),
    }
}

/// `2026-10-12 15:30:45 UTC` from a build id's stamp (`20261012T153045123Z-…`).
fn stamp_text(id: &str) -> String {
    let s = id.as_bytes();
    if s.len() < 19 || s[8] != b'T' {
        return id.to_owned();
    }
    format!("{}-{}-{} {}:{}:{} UTC", &id[0..4], &id[4..6], &id[6..8], &id[9..11], &id[11..13], &id[13..15])
}

fn entry_label(build: &factoryos_client::protocol::codegen::BuildEntry) -> String {
    let plan = plan_stem(&build.plan);
    let state = if build.ok { "✓" } else { "✗" };
    let stale = if build.stale { " · stale" } else { "" };
    let warnings = if build.warnings > 0 { format!(" · {} warning(s)", build.warnings) } else { String::new() };
    format!("{state} {} · {plan}{warnings}{stale}", stamp_text(&build.id))
}

/// Result, stale badge, summary, the options it was built with, notes and the zip.
fn build_header_ui(ui: &mut egui::Ui, info: &BuildInfo, archive_url: &str) {
    ui.horizontal_wrapped(|ui| {
        if info.ok {
            ui.colored_label(theme::current().ok, "✓ Built");
        } else {
            let code = info.exit_code.map(|c| format!(" (exit {c})")).unwrap_or_default();
            ui.colored_label(theme::current().danger, format!("✗ Failed{code}"));
        }
        if info.stale {
            ui.label(RichText::new(" STALE ").small().strong().color(Color32::BLACK).background_color(theme::current().warn))
                .on_hover_text("The plan, the cell's plan2cell/plan2cell.toml or its hardware configuration changed after this build: export it again");
        }
        ui.weak(format!("{} · {:.1} s", stamp_text(&info.id), info.duration_s));
    });
    if let Some(error) = info.error.as_ref().filter(|_| !info.ok) {
        ui.label(RichText::new(error).small().color(theme::current().danger));
    }
    let r = &info.request;
    ui.small(
        RichText::new(format!(
            "{} · profile {} · {} + {} · job number {} · robot sims {} · OPC UA {} · ST check {}{}",
            plan_stem(&info.plan),
            r.profile,
            r.plc_dialect,
            r.robot_language,
            r.job_number,
            on_off(r.robot_sim),
            on_off(r.opcua_symbols),
            on_off(r.st_check),
            if r.strict { " · strict" } else { "" },
        ))
        .weak(),
    );
    if let Some(summary) = &info.summary {
        ui.label(RichText::new(format!(
            "PLAN_ID {} · {} robot(s) · {} zone(s) · makespan {:.1} s · ST {}",
            summary.plan_id,
            summary.robots.len(),
            summary.zones.len(),
            summary.makespan_s,
            if summary.st_checked { "trust OK" } else { "not checked" }
        )));
        egui::CollapsingHeader::new("Robots and zones").id_salt("codegen_summary").show(ui, |ui| {
            for robot in &summary.robots {
                let folgen: Vec<String> = robot.folgen.iter().map(u32::to_string).collect();
                ui.small(format!(
                    "{} ({}): {} ops, Folgen {}, {} KRL points",
                    robot.ns,
                    robot.tag,
                    robot.ops,
                    if folgen.is_empty() { "-".to_owned() } else { folgen.join(", ") },
                    robot.krl_points
                ));
            }
            for zone in &summary.zones {
                ui.small(format!("{} = {}", zone.name, zone.tickets.join(" > ")));
            }
        });
    }
    {
        use factoryos_client::protocol::hardware::HardwareSource;
        let text = match (&info.hardware_source, &info.hardware) {
            (HardwareSource::Saved, Some(hw)) => {
                let robots: Vec<String> = hw.robots.iter().map(|r| format!("{} {} {}", r.ns, r.station, r.ip)).collect();
                format!("Hardware config (saved): head {} · {}", hw.network.head_ip, robots.join(" · "))
            }
            _ => "Hardware config: plan2cell defaults (none saved)".to_owned(),
        };
        ui.small(RichText::new(text).weak());
    }
    if !info.notes.is_empty() {
        egui::CollapsingHeader::new(format!("Notes ({})", info.notes.len())).id_salt("codegen_notes").show(ui, |ui| {
            for note in &info.notes {
                ui.small(note);
            }
        });
    }
    if ui
        .add_enabled(!info.files.is_empty(), egui::Button::new("⬇ Download archive (.zip)"))
        .on_hover_text("Every file of this build, with its plan2cell.toml and build.json")
        .clicked()
    {
        crate::workspace_ui::open_url(archive_url);
    }
}

fn on_off(on: bool) -> &'static str {
    if on { "on" } else { "off" }
}

/// The diagnostics by severity; a click on a file:line returns that file and line.
fn diagnostics_ui(ui: &mut egui::Ui, info: &BuildInfo) -> Option<(String, Option<usize>)> {
    let mut open = None;
    if info.diagnostics.is_empty() {
        ui.small(RichText::new("No diagnostics.").weak());
        return None;
    }
    for (severity, title, color) in [
        (Severity::Error, "Errors", theme::current().danger),
        (Severity::Warning, "Warnings", theme::current().warn),
        (Severity::Info, "Info", Color32::GRAY),
    ] {
        let items: Vec<&Diagnostic> = info.diagnostics.iter().filter(|d| d.severity == severity).collect();
        if items.is_empty() {
            continue;
        }
        egui::CollapsingHeader::new(RichText::new(format!("{title} ({})", items.len())).color(color))
            .id_salt(("codegen_diagnostics", title))
            .default_open(severity != Severity::Info)
            .show(ui, |ui| {
                for d in items {
                    ui.horizontal_wrapped(|ui| {
                        let code = d.code.as_deref().map(|c| format!("[{c}] ")).unwrap_or_default();
                        ui.label(RichText::new(format!("{code}{}", d.source)).small().monospace().color(color));
                        if let Some(file) = &d.file {
                            let at = d.line.map_or_else(|| file.clone(), |line| format!("{file}:{line}"));
                            // Only failed ST checks name files, and such builds write none.
                            if info.files.iter().any(|f| f.path == *file) {
                                if ui.link(RichText::new(at).small().monospace()).on_hover_text("Show the file at that line").clicked() {
                                    open = Some((file.clone(), d.line.map(|l| l as usize)));
                                }
                            } else {
                                ui.label(RichText::new(at).small().monospace().weak())
                                    .on_hover_text("Not written: the build stopped before writing its files");
                            }
                        }
                        ui.label(RichText::new(&d.message).small());
                        if let Some(location) = &d.location {
                            ui.label(RichText::new(location).small().weak());
                        }
                    });
                }
            });
    }
    open
}

/// A directory of the file tree: subdirectories and files.
#[derive(Default)]
struct Dir<'a> {
    dirs: BTreeMap<&'a str, Dir<'a>>,
    files: Vec<&'a BuildFile>,
}

fn tree(files: &[BuildFile]) -> Dir<'_> {
    let mut root = Dir::default();
    for file in files {
        let mut parts: Vec<&str> = file.path.split('/').collect();
        parts.pop();
        let mut dir = &mut root;
        for part in parts {
            dir = dir.dirs.entry(part).or_default();
        }
        dir.files.push(file);
    }
    root
}

fn size_text(bytes: u64) -> String {
    if bytes < 1024 { format!("{bytes} B") } else { format!("{:.1} KiB", bytes as f64 / 1024.0) }
}

/// The file a build's page opens first: the head PLC's Structured Text, else a robot program,
/// else the first file.
fn default_file(files: &[BuildFile]) -> Option<&BuildFile> {
    use crate::code_view::{Lang, language};
    files
        .iter()
        .find(|f| f.path.starts_with("plc/") && language(&f.path) == Lang::St && f.bytes <= PREVIEW_MAX)
        .or_else(|| files.iter().find(|f| language(&f.path) == Lang::Krl && f.bytes <= PREVIEW_MAX))
        .or_else(|| files.first())
}

/// The files whose path has every word of `filter` (any case); all of them for an empty one.
fn filter_files<'a>(files: &'a [BuildFile], filter: &str) -> Vec<&'a BuildFile> {
    let words: Vec<String> = filter.split_whitespace().map(str::to_lowercase).collect();
    files
        .iter()
        .filter(|f| {
            let path = f.path.to_lowercase();
            words.iter().all(|w| path.contains(w))
        })
        .collect()
}

/// What a click on the page asks for.
enum FileAction {
    /// Show a file (at a line), in a new tab or the shown one.
    Open { path: String, line: Option<usize>, new_tab: bool },
    /// Ask the agent about a file, in a new chat.
    Ask(String),
    /// Show only a folder's files (a click on a breadcrumb).
    Filter(String),
}

/// The files as a tree of 22 px rows (Kiro Crew's file explorer): folders fold, a click shows a
/// file, Cmd/Ctrl+click opens it in a new tab, right-click has the rest.
fn file_tree_ui(ui: &mut egui::Ui, files: &[BuildFile], previewing: Option<&str>, p: &theme::Palette) -> Option<FileAction> {
    #[allow(clippy::too_many_arguments)]
    fn show(
        ui: &mut egui::Ui,
        dir: &Dir<'_>,
        prefix: &str,
        depth: usize,
        previewing: Option<&str>,
        p: &theme::Palette,
        action: &mut Option<FileAction>,
    ) {
        for (name, sub) in &dir.dirs {
            let path = format!("{prefix}{name}/");
            let id = ui.id().with(("codegen_dir", &path));
            let open = ui
                .data(|d| d.get_temp::<bool>(id))
                .unwrap_or_else(|| previewing.is_some_and(|v| v.starts_with(&path)));
            let label = format!("{name}  {}", count(sub));
            if tree_row(ui, depth, Some(open), None, &label, false, p).clicked() {
                ui.data_mut(|d| d.insert_temp(id, !open));
            }
            if open {
                show(ui, sub, &path, depth + 1, previewing, p, action);
            }
        }
        for file in &dir.files {
            let name = file.path.rsplit('/').next().unwrap_or(&file.path);
            let selected = previewing == Some(file.path.as_str());
            let response = tree_row(ui, depth, None, Some((&file.path, file.bytes)), name, selected, p);
            file_response(ui, &response, file, action);
        }
    }
    fn count(dir: &Dir<'_>) -> usize {
        dir.files.len() + dir.dirs.values().map(count).sum::<usize>()
    }
    let mut action = None;
    ui.spacing_mut().item_spacing.y = 0.0;
    show(ui, &tree(files), "", 0, previewing, p, &mut action);
    action
}

/// The files a search found, by their paths.
fn file_list_ui(ui: &mut egui::Ui, files: &[&BuildFile], viewing: Option<&str>, p: &theme::Palette) -> Option<FileAction> {
    if files.is_empty() {
        ui.label(RichText::new("No file matches.").small().color(p.muted));
    }
    let mut action = None;
    ui.spacing_mut().item_spacing.y = 0.0;
    for file in files {
        let response = tree_row(ui, 0, None, Some((&file.path, file.bytes)), &file.path, viewing == Some(file.path.as_str()), p);
        file_response(ui, &response, file, &mut action);
    }
    action
}

/// A click on a file row: show it (in a new tab with Cmd/Ctrl); its context menu.
fn file_response(ui: &egui::Ui, response: &egui::Response, file: &BuildFile, action: &mut Option<FileAction>) {
    if response.clicked() {
        let new_tab = ui.input(|i| i.modifiers.command);
        *action = Some(FileAction::Open { path: file.path.clone(), line: None, new_tab });
    }
    if response.middle_clicked() {
        *action = Some(FileAction::Open { path: file.path.clone(), line: None, new_tab: true });
    }
    response.context_menu(|ui| {
        if ui.button("Open in a new tab").clicked() {
            *action = Some(FileAction::Open { path: file.path.clone(), line: None, new_tab: true });
            ui.close();
        }
        if ui.button("Copy the path").clicked() {
            ui.ctx().copy_text(file.path.clone());
            ui.close();
        }
        ui.separator();
        if ui.button("Ask the agent about this file").clicked() {
            *action = Some(FileAction::Ask(file.path.clone()));
            ui.close();
        }
    });
}

/// One row of the tree: indented by `depth`, a chevron for a folder (`open`), the language for a
/// file (its path and size), its size at the right.
pub(crate) fn tree_row(
    ui: &mut egui::Ui,
    depth: usize,
    open: Option<bool>,
    file: Option<(&str, u64)>,
    label: &str,
    selected: bool,
    p: &theme::Palette,
) -> egui::Response {
    let height = 22.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(ui.available_width(), height), egui::Sense::click());
    let painter = ui.painter_at(rect);
    if selected {
        painter.rect_filled(rect, 4, p.accent.gamma_multiply(0.22));
    } else if response.hovered() {
        painter.rect_filled(rect, 4, p.text.gamma_multiply(0.07));
    }
    let mut x = rect.left() + 6.0 + depth as f32 * 12.0;
    let y = rect.center().y;
    match (open, file) {
        (Some(open), _) => {
            let c = egui::pos2(x + 4.0, y);
            let points = if open {
                vec![c + egui::vec2(-3.5, -2.0), c + egui::vec2(3.5, -2.0), c + egui::vec2(0.0, 2.5)]
            } else {
                vec![c + egui::vec2(-2.0, -3.5), c + egui::vec2(2.5, 0.0), c + egui::vec2(-2.0, 3.5)]
            };
            painter.add(egui::Shape::convex_polygon(points, p.muted, egui::Stroke::NONE));
            x += 14.0;
        }
        (None, Some((path, _))) => {
            let (tag, color) = match crate::code_view::language(path) {
                crate::code_view::Lang::St => ("ST", p.info),
                crate::code_view::Lang::Krl => ("KRL", p.accent),
                crate::code_view::Lang::Plain => ("·", p.muted),
            };
            let galley = painter.layout_no_wrap(tag.to_owned(), egui::FontId::monospace(9.0), color);
            painter.galley(egui::pos2(x, y - galley.size().y / 2.0), galley, color);
            x += 26.0;
        }
        (None, None) => {}
    }
    let size = file.map(|(_, bytes)| size_text(bytes));
    let size_width = if let Some(size) = &size {
        let galley = painter.layout_no_wrap(size.clone(), egui::FontId::proportional(10.5), p.muted);
        let width = galley.size().x;
        painter.galley(egui::pos2(rect.right() - 6.0 - width, y - galley.size().y / 2.0), galley, p.muted);
        width + 12.0
    } else {
        0.0
    };
    let color = if selected { p.text_strong } else { p.text };
    let font = egui::FontId::proportional(12.0);
    let max = (rect.right() - size_width - x).max(10.0);
    let mut job = egui::text::LayoutJob::simple_singleline(label.to_owned(), font, color);
    job.wrap = egui::text::TextWrapping::truncate_at_width(max);
    let galley = ui.fonts_mut(|f| f.layout_job(job));
    painter.galley(egui::pos2(x, y - galley.size().y / 2.0), galley, color);
    response.on_hover_cursor(egui::CursorIcon::PointingHand)
}

/// A bordered card on the page (Kiro Crew's `card`).
fn card(ui: &mut egui::Ui, p: &theme::Palette, add: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(p.card)
        .stroke(egui::Stroke::new(1.0, p.border))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(10, 8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui);
        });
}

/// The search over the files' paths, in a card-coloured field that turns accent when focused.
pub(crate) fn search_ui(ui: &mut egui::Ui, filter: &mut String, p: &theme::Palette) {
    let id = ui.id().with("codegen_search");
    let focused = ui.memory(|m| m.focused()) == Some(id);
    egui::Frame::new()
        .fill(p.card)
        .stroke(egui::Stroke::new(1.0, if focused { p.accent } else { p.border }))
        .corner_radius(6)
        .inner_margin(egui::Margin::symmetric(8, 4))
        .show(ui, |ui| {
            ui.add(
                egui::TextEdit::singleline(filter)
                    .id(id)
                    .frame(egui::Frame::NONE)
                    .hint_text("🔍 Search the files")
                    .desired_width(f32::INFINITY),
            );
        });
}

/// A frameless icon button in the muted colour.
pub(crate) fn icon_button(icon: &str, p: &theme::Palette) -> egui::Button<'static> {
    egui::Button::new(RichText::new(icon.to_owned()).size(13.0).color(p.muted))
        .frame(false)
        .min_size(egui::vec2(22.0, 22.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_labels_name_the_planner() {
        let plan = |planner: &str, status: &str| PlanInfo {
            tree_id: "bt_1_c".to_owned(),
            num_operations: 20,
            planner: planner.to_owned(),
            status: status.to_owned(),
            ..PlanInfo::default()
        };
        assert_eq!(plan_label(&plan("hier_la_fu", "ok")), "bt_1_c · hier_la_fu · 20 ops");
        assert_eq!(plan_label(&plan("hier_q", "unverified")), "bt_1_c · hier_q · 20 ops · unverified");
        assert_eq!(plan_label(&plan("", "")), "bt_1_c · 20 ops", "an older backend");
        let stale = PlanInfo { stale: true, ..plan("umarell", "ok") };
        assert_eq!(plan_label(&stale), "bt_1_c · umarell · 20 ops · earlier scene");
        let changed = PlanInfo { stale_reason: Some("robot r1's URDF changed".into()), ..stale };
        assert_eq!(plan_label(&changed), "bt_1_c · umarell · 20 ops · stale");
    }

    #[test]
    fn files_become_a_tree() {
        let file = |path: &str| BuildFile { path: path.to_owned(), bytes: 1 };
        let files = [file("BUILD.md"), file("plc/codesys/a.st"), file("plc/codesys/b.st"), file("krl/robot1/x.src")];
        let root = tree(&files);
        assert_eq!(root.files.len(), 1);
        assert_eq!(root.dirs.keys().copied().collect::<Vec<_>>(), ["krl", "plc"]);
        assert_eq!(root.dirs["plc"].dirs["codesys"].files.len(), 2);
    }

    #[test]
    fn the_page_opens_the_plc_code_first() {
        let file = |path: &str, bytes: u64| BuildFile { path: path.to_owned(), bytes };
        let files = [file("BUILD.md", 10), file("krl/robot1/Folge1.src", 10), file("plc/codesys/PRG_Cell.st", 10)];
        assert_eq!(default_file(&files).unwrap().path, "plc/codesys/PRG_Cell.st");
        assert_eq!(default_file(&files[..2]).unwrap().path, "krl/robot1/Folge1.src");
        let huge = [file("plc/codesys/big.st", PREVIEW_MAX + 1), file("BUILD.md", 10)];
        assert_eq!(default_file(&huge).unwrap().path, "plc/codesys/big.st", "the first file: it says it is too large");
        assert!(default_file(&[]).is_none());
    }

    #[test]
    fn the_file_search_matches_every_word_in_the_path() {
        let file = |path: &str| BuildFile { path: path.to_owned(), bytes: 1 };
        let files = [file("krl/robot1/Folge1.src"), file("krl/robot2/Folge1.src"), file("plc/codesys/PRG_Cell.st")];
        let found = |filter: &str| filter_files(&files, filter).iter().map(|f| f.path.as_str()).collect::<Vec<_>>();
        assert_eq!(found("").len(), 3);
        assert_eq!(found("folge"), ["krl/robot1/Folge1.src", "krl/robot2/Folge1.src"]);
        assert_eq!(found("robot2 FOLGE"), ["krl/robot2/Folge1.src"]);
        assert_eq!(found(" .st "), ["plc/codesys/PRG_Cell.st"]);
        assert!(found("nothing").is_empty());
    }

    #[test]
    fn labels() {
        assert_eq!(stamp_text("20261012T153045123Z-bt_1"), "2026-10-12 15:30:45 UTC");
        assert_eq!(stamp_text("x"), "x");
        assert_eq!(plan_stem("agentic_planner/plans/bt_1.json"), "bt_1");
        assert_eq!(size_text(2048), "2.0 KiB");
        assert_eq!(export_label("20261012T153045123Z-bt_1"), "2026-10-12 15:30:45 UTC · bt_1");
        assert_eq!(export_label("bt_1"), "bt_1");
    }

    #[test]
    fn run_control_code_takes_the_picked_build_else_the_newest() {
        let entry = |id: &str, ok: bool, plan_id: Option<&str>| factoryos_client::protocol::codegen::BuildEntry {
            id: id.into(),
            ok,
            plan_id: plan_id.map(str::to_owned),
            ..Default::default()
        };
        let mut export = CodeExport::new(Api::new("http://localhost:1"));
        assert_eq!(export.deploy_choice(), None);
        export.builds.builds = vec![
            entry("20261013T090000000Z-bt_a", false, None),
            entry("20261012T090000000Z-bt_a", true, Some("16#8E35")),
            entry("20261011T090000000Z-bt_b", true, Some("16#9201")),
        ];
        let newest = DeployChoice { build: "20261012T090000000Z-bt_a".into(), plan_id: Some("16#8E35".into()), picked: false };
        assert_eq!(export.deploy_choice(), Some(newest.clone()), "the newest successful one");
        export.picked = Some("20261011T090000000Z-bt_b".into());
        assert_eq!(export.deploy_choice().map(|c| (c.build, c.picked)), Some(("20261011T090000000Z-bt_b".into(), true)));
        export.picked = Some("20261013T090000000Z-bt_a".into());
        assert_eq!(export.deploy_choice(), Some(newest), "a failed build is never deployed");
    }

    #[test]
    fn the_hardware_follows_the_plan_of_the_export() {
        let entry = |id: &str, plan: &str| factoryos_client::protocol::codegen::BuildEntry {
            id: id.into(),
            plan: plan.into(),
            ok: true,
            ..Default::default()
        };
        let mut export = CodeExport::new(Api::new("http://localhost:1"));
        export.shown_plan = Some("agentic_planner/plans/bt_b.json".into());
        assert_eq!(export.hardware_plan().as_deref(), Some("agentic_planner/plans/bt_b.json"), "no export yet: the form's plan");
        export.builds.builds = vec![
            entry("20261013T090000000Z-bt_a", "agentic_planner/plans/bt_a.json"),
            entry("20261012T090000000Z-bt_b", "agentic_planner/plans/bt_b.json"),
        ];
        assert_eq!(export.hardware_plan().as_deref(), Some("agentic_planner/plans/bt_a.json"), "the newest export's robots");
        export.picked = Some("20261012T090000000Z-bt_b".into());
        assert_eq!(export.hardware_plan().as_deref(), Some("agentic_planner/plans/bt_b.json"), "the picked export's");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn archive_urls_name_the_workspace_the_cookie_selects() {
        let path = api::codegen_archive("20261012T153045123Z-bt_1");
        assert_eq!(archive_url("https://host", Some("ci"), "20261012T153045123Z-bt_1"), format!("https://host/workspace/ci{path}"));
        assert_eq!(archive_url("https://host/", None, "20261012T153045123Z-bt_1"), format!("https://host{path}"));
        // A base that has the workspace already (web, plain http) is used as it is.
        let base = "http://host/workspace/ci";
        assert_eq!(archive_url(base, Some("other"), "20261012T153045123Z-bt_1"), format!("{base}{path}"));
        assert_eq!(archive_url("https://host", Some("../x"), "20261012T153045123Z-bt_1"), format!("https://host{path}"));
    }
}
