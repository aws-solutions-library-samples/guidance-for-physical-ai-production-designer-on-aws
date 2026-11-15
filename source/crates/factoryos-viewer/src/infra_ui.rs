//! The virtual cell in the sidebar's guided flow ([`crate::flow`]):
//!
//! * a workspace's own cell ([`crate::virtual_cell`], [`InfraStatus::virtual_cell`] and
//!   [`api::INFRA_CELLS`]): "No virtual cell" with Create virtual cell (and the cost hint); off:
//!   Start virtual cell (how to restore the last control code), Delete virtual cell… (the cell id
//!   typed again); starting: the phase stepper launching → booting → configuring → bring-up →
//!   restoring control code → ready, with the time so far; ready: Stop virtual cell (which
//!   terminates the instances); stopping: archiving → terminating; its images and the session's
//!   cost. Bound to the shared cell: its controls below, plus Create own virtual cell. A backend
//!   that lists cell templates ([`CellsList::templates`]): Create picks a template and fills the
//!   form of its parameters (instead of the runtime pickers), an own cell shows its template and
//!   changes it or its parameters, and its controls follow the template's capabilities;
//! * the "Virtual cell" header ([`InfraUi::header_ui`]): `Virtual cell: ● off / booting (phase) /
//!   ready` ([`InfraStatus::cell_power`]: the Lambda boots, brings up and checks the cell by itself;
//!   the checks that fail), the plan2cell cell's instances (PLC host, Windows CODESYS IDE, one chip
//!   per KUKA VC host) as state chips, Start / Stop virtual cell ([`api::INFRA_CELL_POWER`]; Start
//!   with "then start production": one click to a producing cell; Stop with a confirmation that
//!   warns in production; an older server: Start / Stop cell), Bring up controllers. Deploy, Open
//!   PLC IDE and Start production are disabled while the cell is off or booting, with a hint;
//! * step 3, Deploy ([`InfraUi::deploy_ui`]): [`api::INFRA_DEPLOY`] with `start: false` (the export
//!   step 2 has: picked in the Control Code Export results, else the newest) onto the head PLC and
//!   the robots, its stages live; afterwards deployed, not producing;
//! * step 4, Review ([`InfraUi::review_ui`]): Open PLC IDE (NICE DCV in a new window, through the
//!   backend's `/dcv/ide/` with a session token: no Windows password; the link pinned to the
//!   backend's workspace, `/workspace/<name>/dcv/ide/`), Open smartPAD per KUKA controller (KDM's iiQKA.UI, each in
//!   a browser window of its own at `https://<robot>--<workspace>.<site>/`, [`api::SMARTPADS`]) and which project to
//!   look at;
//! * step 5, Start production ([`InfraUi::production_ui`]): Start production (the code the head
//!   runs), Restart PLC runtime, Stop production ([`api::INFRA_PRODUCTION_STOP`]), the CODESYS demo
//!   runtime's stop time (live, from [`InfraStatus::plc_runtime`]), Auto-restart before the 2 h
//!   demo stop ([`api::INFRA_AUTO_RESTART`]), and Run Control Code (deploy + start in one go, the
//!   shortcut without the review).
//!
//! The backend does the work through the cell-control Lambda ([`api::INFRA_STATUS`] and the
//! `POST`s next to it); its status comes over `factoryos/infra/status` and, while the sidebar is
//! shown, from a `GET` about every 10 s (which keeps the backend reading it).

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::codegen::same_plan_id;
use factoryos_client::protocol::smartpad::SmartPads;
use factoryos_client::protocol::infra::{
    AutoRestartSettings, AutoRestartStatus, BringupRequest, CellPower, CellPowerRequest, CellRole, CellsList, CreateCellRequest, DCV_TARGET_IDE,
    DcvRevoked, DcvSession, DcvSessionRequest, DeleteCellRequest, DeployRequest, DeployStatus, InfraInstance, InfraStatus, JobState, PlcRuntime,
    CellKind, PowerState, ProductionRequest, RESTORE_MODES, ROLE_KUKA_VC, ROLE_WINDOWS_IDE, ROLES, StartStopRequest, VirtualCell, role_label, utc_hhmm, OFF_MODE_STOP, OFF_MODE_TERMINATE, OFF_MODES, UpdateCellRequest, IDE_CONTAINER, IDE_EC2, IDES, RUNTIME_EC2, RUNTIME_EKS, RUNTIMES, CellNetwork, MAX_NETWORKS, MAX_NETWORK_ROBOTS,
    CellTemplateInfo, TemplateParameter, TemplateSelector,
};
use web_time::{Duration, Instant, SystemTime};

use crate::theme;
use crate::code_export::{DeployChoice, export_label};
use crate::deploy_check_ui::{CheckAction, DeployCheckUi};
use crate::rr::egui::{self, Color32, RichText};
use crate::sidebar;
use crate::virtual_cell::{self, Mode, OwnState, START_STEPS, USUAL_RESUME, USUAL_START};

/// How often the sidebar asks while it is shown.
const POLL: Duration = Duration::from_secs(10);
/// … and while a start, stop or job is under way.
const POLL_BUSY: Duration = Duration::from_secs(4);
/// The demo runtime's line turns to the warning colour with less than this left.
const DEMO_WARN_S: f64 = 600.0;
/// A production start from this viewer that has not reached production by then is no longer watched.
const PRODUCTION_WATCH: Duration = Duration::from_secs(30 * 60);
/// How often the own cells' list is read while the box is shown (the status brings the own cell
/// over the bus as well).
const CELLS_POLL: Duration = Duration::from_secs(30);

enum Msg {
    Status(Result<InfraStatus, String>),
    Action(&'static str, Result<InfraStatus, String>),
    /// `POST /api/v1/infra/dcv/session`.
    Dcv(Result<DcvSession, String>),
    /// `DELETE /api/v1/infra/dcv/session`.
    DcvRevoked(Result<DcvRevoked, String>),
    /// `PUT /api/v1/infra/auto-restart`.
    AutoRestart(Result<AutoRestartStatus, String>),
    /// `GET /api/v1/infra/cells`.
    Cells(Result<CellsList, String>),
    /// `POST /api/v1/infra/cells` (`create`), `DELETE /api/v1/infra/cells/{id}` (`delete`).
    CellAction(&'static str, Result<VirtualCell, String>),
    /// `GET /api/v1/smartpads`.
    SmartPads(Result<SmartPads, String>),
}

pub struct InfraUi {
    api: Api,
    inbox: Arc<Mutex<Vec<Msg>>>,
    status: Option<InfraStatus>,
    /// The bus update last taken (its counter).
    bus_seen: Option<u64>,
    asked: Option<Instant>,
    /// The sidebar drew the group (polling only while it is shown).
    drawn: Option<Instant>,
    pending: Option<&'static str>,
    /// The last status `GET` failed (cleared by the next status, from the `GET` or the bus).
    status_error: Option<String>,
    /// The last action failed (which, its message; cleared by the next action).
    action_error: Option<(String, String)>,
    confirm_stop: bool,
    /// Start (virtual) cell: then start production once it is ready (default on).
    start_follow: bool,
    /// Deploy / Run Control Code asked for an export and waits for the confirmation.
    confirm_deploy: Option<ConfirmDeploy>,
    /// When Open PLC IDE last opened the IDE (Unix seconds, the server's clock: the session's
    /// `issuedS`); set only once the window opened, or its link was clicked.
    ide_opened_s: Option<f64>,
    /// One-off recovery (M_27002): cleared once a bring-up with it was accepted.
    restart_controllers: bool,
    /// A DCV session is being asked for (ahead of the click, see [`Self::ide_button`]).
    dcv_pending: bool,
    /// The DCV session link (absolute URL), when it was issued (the server's clock), when this
    /// viewer asked for it (ours) and whether Open PLC IDE has used it yet.
    dcv_link: Option<(String, f64)>,
    dcv_asked_s: f64,
    dcv_used: bool,
    dcv_error: Option<String>,
    /// A production start from this viewer (Start production, Restart PLC runtime, Run Control
    /// Code, Start cell with "then bring up and start production") on its way into production.
    production_watch: Option<ProductionWatch>,
    /// It got there (see [`Self::take_reached_production`]).
    reached_production: bool,
    /// The own cells' list ([`api::INFRA_CELLS`]) as last read, when it was asked for, and whether
    /// the server has it at all (an older server answers `404`: not asked again).
    cells: Option<CellsList>,
    cells_asked: Option<Instant>,
    cells_unsupported: bool,
    /// Create virtual cell: start it right away (default on).
    create_start: bool,
    /// Create virtual cell: its runtime ([`RUNTIMES`]) and, for EKS, its IDE ([`IDES`]).
    create_runtime: &'static str,
    create_ide: &'static str,
    /// Create virtual cell (EKS): how many PROFINET networks (each a cell of its own: head PLC, 2 robots, segment).
    create_networks: usize,
    /// KUKA VCs of each network of a new EKS cell.
    create_robots: u8,
    /// Create virtual cell from a template (a backend that lists them): the one picked (`None`: the
    /// default, [`virtual_cell::create_template`]) and the parameters chosen, of the template named
    /// (another one starts at its defaults).
    create_template: Option<String>,
    create_params: (String, serde_json::Map<String, serde_json::Value>),
    /// Create own virtual cell while bound to the shared cell: waiting for the confirmation.
    confirm_create_own: bool,
    /// Delete…: the cell id and what has been typed so far.
    confirm_delete: Option<(String, String)>,
    /// Start of an own cell: how to restore the last control code ([`RESTORE_MODES`]).
    restore_mode: &'static str,
    /// Step 3's deployability check of the export it deploys.
    check: DeployCheckUi,
    /// The cell's smartPADs ([`api::SMARTPADS`]) as last read, when they were asked for, the last error, and whether
    /// the server has them at all (an older server answers `404`: not asked again).
    smartpads: Option<SmartPads>,
    smartpads_asked: Option<Instant>,
    smartpads_error: Option<String>,
    smartpads_unsupported: bool,
}

/// A production start from this viewer, watched until the cell is in production.
#[derive(Clone, Copy, Debug)]
struct ProductionWatch {
    since: Instant,
    /// Its job (or the instances' start) has shown up in the status.
    seen_working: bool,
}

/// Where a watched production start is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WatchStep {
    Wait,
    /// The job ended with the cell in production.
    Reached,
    /// The job ended without it.
    Ended,
}

/// The next step of a watched production start: it waits for the request's reply (`pending`),
/// then for its job (a start, bring-up, production start or deploy, or instances changing state)
/// to show up and end.
fn watch_step(seen_working: &mut bool, pending: bool, status: Option<&InfraStatus>) -> WatchStep {
    let Some(status) = status.filter(|_| !pending) else { return WatchStep::Wait };
    if status.busy() || status.changing() {
        *seen_working = true;
        return WatchStep::Wait;
    }
    match (*seen_working, status.in_production) {
        (false, _) => WatchStep::Wait,
        (true, true) => WatchStep::Reached,
        (true, false) => WatchStep::Ended,
    }
}

/// The actions that bring the cell into production (`start` / `power-on` only with their follow-up).
fn starts_production(what: &str, start_follow: bool) -> bool {
    matches!(what, "production" | "restart" | "run") || (matches!(what, "start" | "power-on" | "power-fresh") && start_follow)
}

impl InfraUi {
    pub fn new(api: Api) -> Self {
        Self {
            check: DeployCheckUi::new(api.clone()),
            api,
            inbox: Arc::default(),
            status: None,
            bus_seen: None,
            asked: None,
            drawn: None,
            pending: None,
            status_error: None,
            action_error: None,
            confirm_stop: false,
            start_follow: true,
            confirm_deploy: None,
            ide_opened_s: None,
            restart_controllers: false,
            dcv_pending: false,
            dcv_link: None,
            dcv_asked_s: 0.0,
            dcv_used: false,
            dcv_error: None,
            production_watch: None,
            reached_production: false,
            cells: None,
            cells_asked: None,
            cells_unsupported: false,
            smartpads: None,
            smartpads_asked: None,
            smartpads_error: None,
            smartpads_unsupported: false,
            create_start: true,
            create_runtime: RUNTIME_EC2,
            create_ide: IDE_CONTAINER,
            create_networks: 1,
            create_robots: 2,
            create_template: None,
            create_params: (String::new(), serde_json::Map::new()),
            confirm_create_own: false,
            confirm_delete: None,
            restore_mode: RESTORE_MODES[0],
        }
    }

    /// The cell reached production after a start from this viewer (once per start): the app
    /// shows the live cell then.
    pub fn take_reached_production(&mut self) -> bool {
        std::mem::take(&mut self.reached_production)
    }

    fn sender(&self, ctx: &egui::Context) -> impl Fn(Msg) + Send + Sync + 'static {
        let (inbox, ctx) = (self.inbox.clone(), ctx.clone());
        move |msg| {
            inbox.lock().unwrap().push(msg);
            ctx.request_repaint();
        }
    }

    /// Call once per frame with the last status from the bus (and its counter).
    pub fn poll(&mut self, ctx: &egui::Context, bus: Option<&(InfraStatus, u64)>) {
        if let Some((status, n)) = bus
            && self.bus_seen != Some(*n)
        {
            self.bus_seen = Some(*n);
            self.status = Some(status.clone());
            self.status_error = None;
        }
        let messages = std::mem::take(&mut *self.inbox.lock().unwrap());
        for msg in messages {
            match msg {
                Msg::Status(Ok(status)) => {
                    self.status = Some(status);
                    self.status_error = None;
                }
                Msg::Status(Err(err)) => self.status_error = Some(body_of(&err).to_owned()),
                Msg::Action(what, result) => {
                    self.pending = None;
                    match result {
                        Ok(status) => {
                            self.action_error = None;
                            self.status = Some(status);
                            if what == "bring-up" {
                                self.restart_controllers = false;
                            }
                        }
                        Err(err) => {
                            if starts_production(what, true) {
                                self.production_watch = None;
                            }
                            // The deployability check refused it: its report into step 3.
                            let refused = matches!(what, "deploy" | "run")
                                .then(|| factoryos_client::protocol::deploy_check::DeployRefused::from_error(&err))
                                .flatten();
                            let text = match refused {
                                Some(r) => {
                                    let e = r.error.clone();
                                    self.check.refused(r.report);
                                    e
                                }
                                None => body_of(&err).to_owned(),
                            };
                            self.action_error = Some((what.to_owned(), format!("{}: {}", action_label(what), text)));
                        }
                    }
                }
                Msg::Dcv(result) => {
                    self.dcv_pending = false;
                    match result {
                        Ok(session) => {
                            let url = dcv_link(self.api.base(), &session);
                            self.dcv_error = None;
                            // The server's clock (the deploy's finishedS is too); an older server: ours.
                            let issued = if session.issued_s > 0.0 { session.issued_s } else { now_s() };
                            // Ready for the next click: Open PLC IDE opens it in the click itself.
                            self.dcv_link = Some((url, issued));
                            self.dcv_used = false;
                        }
                        Err(err) => self.dcv_error = Some(body_of(&err).to_owned()),
                    }
                }
                Msg::AutoRestart(result) => {
                    self.pending = None;
                    match result {
                        Ok(auto) => {
                            self.action_error = None;
                            if let Some(s) = &mut self.status {
                                s.auto_restart = auto;
                            }
                        }
                        Err(err) => self.action_error = Some(("auto-restart".to_owned(), format!("Auto-restart: {}", body_of(&err)))),
                    }
                }
                Msg::Cells(Ok(list)) => self.cells = Some(list),
                // An older server: no own cells.
                Msg::Cells(Err(err)) => self.cells_unsupported = err.starts_with("404"),
                Msg::SmartPads(Ok(pads)) => {
                    self.smartpads = Some(pads);
                    self.smartpads_error = None;
                }
                // An older server: no smartPADs.
                Msg::SmartPads(Err(err)) => {
                    self.smartpads_unsupported = err.starts_with("404");
                    self.smartpads_error = Some(body_of(&err).to_owned());
                }
                Msg::CellAction(what, result) => {
                    self.pending = None;
                    match result {
                        Ok(cell) => {
                            self.action_error = None;
                            if let Some(list) = &mut self.cells {
                                list.cells = vec![cell.clone()];
                                list.can_create = false;
                            }
                            if let Some(s) = self.status.as_mut().filter(|s| s.binding == Some(CellKind::Own)) {
                                s.virtual_cell = Some(cell);
                            }
                        }
                        Err(err) => self.action_error = Some((what.to_owned(), format!("{}: {}", action_label(what), body_of(&err)))),
                    }
                    // Read both again right away.
                    self.asked = None;
                    self.cells_asked = None;
                }
                Msg::DcvRevoked(result) => match result {
                    Ok(_) => {
                        self.dcv_link = None;
                        self.dcv_error = None;
                    }
                    Err(err) => self.dcv_error = Some(format!("Sign out: {}", body_of(&err))),
                },
            }
        }
        if let Some(watch) = &mut self.production_watch {
            match watch_step(&mut watch.seen_working, self.pending.is_some(), self.status.as_ref()) {
                _ if watch.since.elapsed() > PRODUCTION_WATCH => self.production_watch = None,
                WatchStep::Wait => {}
                WatchStep::Reached => {
                    self.production_watch = None;
                    self.reached_production = true;
                }
                WatchStep::Ended => self.production_watch = None,
            }
        }
        let own_moving = virtual_cell::own_state(virtual_cell::own_cell(self.status.as_ref(), self.cells.as_ref())).transitional();
        let busy = self.pending.is_some()
            || own_moving
            || self.status.as_ref().is_some_and(|s| {
                s.busy() || s.transition.is_some() || s.cell_power.as_ref().is_some_and(|p| matches!(p.state, PowerState::Booting | PowerState::Stopping))
            });
        let every = if busy { POLL_BUSY } else { POLL };
        // Shown: drawn within the last poll interval (+ slack). The frame this schedules runs before the
        // sidebar redraws, so in an idle viewer `drawn` is about `every` old here.
        let shown = self.drawn.is_some_and(|at| at.elapsed() < every + Duration::from_secs(2));
        if shown && self.asked.is_none_or(|at| at.elapsed() > every) {
            self.asked = Some(Instant::now());
            let send = self.sender(ctx);
            self.api.get(api::INFRA_STATUS, move |status| send(Msg::Status(status)));
            ctx.request_repaint_after(every);
        } else if shown {
            ctx.request_repaint_after(every);
        }
        let cells_every = if busy { POLL_BUSY } else { CELLS_POLL };
        if shown && !self.cells_unsupported && self.cells_asked.is_none_or(|at| at.elapsed() > cells_every) {
            self.cells_asked = Some(Instant::now());
            let send = self.sender(ctx);
            self.api.get(api::INFRA_CELLS, move |list| send(Msg::Cells(list)));
        }
        // The controllers change with the cell's config only (a launch of a workspace's own cell).
        if shown && !self.smartpads_unsupported && self.smartpads_asked.is_none_or(|at| at.elapsed() > CELLS_POLL) {
            self.smartpads_asked = Some(Instant::now());
            let send = self.sender(ctx);
            self.api.get(api::SMARTPADS, move |pads| send(Msg::SmartPads(pads)));
        }
    }

    fn act(&mut self, ctx: &egui::Context, what: &'static str, path: &'static str, body: serde_json::Value) {
        self.pending = Some(what);
        self.action_error = None;
        if starts_production(what, self.start_follow) {
            self.production_watch = Some(ProductionWatch { since: Instant::now(), seen_working: false });
        }
        let send = self.sender(ctx);
        self.api.post_json(path, &body, move |result| send(Msg::Action(what, result)));
    }

    /// A chat tab step 3's "Ask the agent" asked for: (title, prompt).
    pub fn take_chat_request(&mut self) -> Option<(String, String)> {
        self.check.take_chat()
    }

    /// The last status read (the guided flow's steps 3–5 derive their state from it).
    pub fn status(&self) -> Option<&InfraStatus> {
        self.status.as_ref()
    }

    /// When this viewer last opened the PLC IDE (Unix seconds).
    pub fn ide_opened_s(&self) -> Option<f64> {
        self.ide_opened_s
    }

    /// The status, or (`None`) a line why there is none / no cell control.
    fn usable_status(&self, ui: &mut egui::Ui) -> Option<InfraStatus> {
        match &self.status {
            None => {
                match &self.status_error {
                    Some(err) => ui.small(RichText::new(format!("Can't read the virtual cell: {err}")).color(theme::current().danger)),
                    None => ui.small(RichText::new("Reading the virtual cell…").weak()),
                };
                None
            }
            Some(status) if !status.available => {
                ui.small(RichText::new(status.message.as_deref().unwrap_or("No cell control on this server.")).color(theme::current().warn));
                None
            }
            Some(status) => Some(status.clone()),
        }
    }

    /// The "Virtual cell" box above step 5: the workspace's own cell (Create / Start / Stop /
    /// Delete, [`crate::virtual_cell`]), or the shared cell's instances as chips, Start / Stop cell
    /// and Bring up controllers (plus Create own virtual cell when the server can).
    pub fn header_ui(&mut self, ui: &mut egui::Ui) {
        self.drawn = Some(Instant::now());
        match virtual_cell::mode(self.status.as_ref(), self.cells.as_ref()) {
            Mode::Own => self.own_cell_ui(ui),
            Mode::Shared => {
                self.shared_header_ui(ui);
                self.create_own_offer(ui);
            }
            Mode::Legacy => self.shared_header_ui(ui),
        }
    }

    /// The shared cell's (or an older server's) controls.
    fn shared_header_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let Some(status) = self.usable_status(ui) else {
            self.errors(ui, &["start", "stop", "bring-up", "power-on", "power-off"]);
            return;
        };
        if let Some(power) = &status.cell_power {
            power_ui(ui, power);
        }
        // One chip per instance the Lambda reports (a KUKA VC host per controller); before any, the roles. Advanced only.
        if status.instances.is_empty() && crate::ui_mode::is_advanced(ui) {
            for role in ROLES {
                chip(ui, role, None);
            }
        }
        let chips = crate::ui_mode::is_advanced(ui);
        for i in status.instances.iter().filter(|_| chips) {
            chip(ui, &i.role, Some(i));
        }
        if let Some(message) = &status.message {
            ui.small(RichText::new(message).color(theme::current().danger));
        }
        if let Some(t) = &status.transition {
            ui.small(RichText::new(if t == "start" { "Starting the instances…" } else { "Stopping the instances…" }).color(theme::current().warn));
        }
        if let Some(line) = lease_line(&status, now_s()) {
            ui.small(RichText::new(line).color(theme::current().warn));
        }

        let pending = self.pending.is_some();
        let power = status.cell_power.as_ref();
        let any_running = status.instances.iter().any(|i| i.state == "running" || i.state == "pending");
        let (can_start, can_stop) = power_buttons(&status);
        ui.horizontal(|ui| {
            let half = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let label = if matches!(self.pending, Some("start" | "power-on")) { "Starting…" } else { "Start virtual cell" };
                if sidebar::button_response(ui, label, !pending && can_start)
                    .on_hover_text(match (power.is_some(), self.start_follow) {
                        (true, true) => {
                            "Start the PLC host, the KUKA VC hosts and the Windows IDE (one without capacity for its type starts as a \
                             fallback type). The cell control Lambda then waits for the boot, brings up the KUKA controllers (EXT) and \
                             checks the PLC runtime, the routing, DCV and the MCP bridge (about 3–6 min); then Start production."
                        }
                        (true, false) => {
                            "Start the PLC host, the KUKA VC hosts and the Windows IDE; the cell control Lambda then waits for the boot, \
                             brings up the KUKA controllers (EXT) and checks the PLC runtime, the routing, DCV and the MCP bridge."
                        }
                        (false, true) => {
                            "Start the PLC host, the Windows IDE and the KUKA VC hosts (each on its own; one without capacity for its \
                             type starts as a fallback type), wait until all run and are SSM-online (about 5 min), bring up the \
                             controllers, then Start production"
                        }
                        (false, false) => {
                            "Start the PLC host, the Windows IDE and the KUKA VC hosts (each on its own; one without capacity for its \
                             type starts as a fallback type)"
                        }
                    })
                    .on_disabled_hover_text(if power.is_some() {
                        "The virtual cell is on already, on its way, or stopping"
                    } else {
                        "Every instance runs already (or is changing state)"
                    })
                    .clicked()
                {
                    if power.is_some() {
                        let body = CellPowerRequest { action: "on".into(), production: Some(self.start_follow), ..Default::default() };
                        self.act(&ctx, "power-on", api::INFRA_CELL_POWER, serde_json::to_value(body).unwrap_or_default());
                    } else {
                        let follow = Some(self.start_follow);
                        let body = serde_json::to_value(StartStopRequest { roles: None, bring_up: follow, production: follow }).unwrap_or_default();
                        self.act(&ctx, "start", api::INFRA_START, body);
                    }
                }
            });
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let label = if matches!(self.pending, Some("stop" | "power-off")) { "Stopping…" } else { "Stop virtual cell" };
                if sidebar::button_response(ui, label, !pending && can_stop)
                    .on_hover_text(
                        "Shuts down the EC2 instances that run the virtual PLCs, the CODESYS IDE and the KUKA controllers \
                         (an EC2 stop: nothing is deleted, Start virtual cell resumes it). To stop only the cell's motion: Stop \
                         production (step 5).",
                    )
                    .on_disabled_hover_text("Nothing runs")
                    .clicked()
                {
                    self.confirm_stop = true;
                }
            });
        });
        if any_running {
            ui.small(
                RichText::new(
                    "⚠ Stop virtual cell shuts down the machines of the virtual PLCs, the PLC IDE and the KUKA controllers. \
                     To pause the cell only, use Stop production.",
                )
                .color(theme::current().warn),
            );
        }
        if crate::ui_mode::is_advanced(ui) {
        ui.checkbox(&mut self.start_follow, RichText::new("then start production").small()).on_hover_text(if power.is_some() {
            "Once the cell is ready (booted, KUKA controllers brought up, every check passed), Start production (the code the head \
             runs). Off: the cell is only started (a cell that was in production when it was stopped is not started either)."
        } else {
            "Start cell waits until every instance runs and is SSM-online (up to 10 min), runs Bring up controllers and then \
             Start production (the code the head runs). Off: only the instances start (a cell that was in production at Stop \
             cell is not started)."
        });
        }
        let start = &status.start;
        if start.state != JobState::Idle {
            job_line(ui, start.state, start.message.as_deref(), None);
            let advanced = crate::ui_mode::is_advanced(ui);
            for i in start.instances.iter().filter(|i| i.failed() || (i.result == "fallback" && advanced)) {
                let color = if i.failed() { theme::current().danger } else { theme::current().warn };
                ui.small(RichText::new(i.summary()).color(color));
            }
            if start.instances.iter().any(|i| i.result == "fallback") && crate::ui_mode::is_advanced(ui) {
                ui.small(
                    RichText::new("A fallback type stays on the instance: record it in the stack (plan2cell/docs/CELL.md, \"Fallback instance types\").")
                        .weak(),
                );
            }
        }
        if self.confirm_stop {
            self.stop_dialog(&ctx, &status);
        }
        self.bringup_ui(ui, &status);
        if let Some(err) = &self.status_error {
            ui.label(RichText::new(format!("Can't read the virtual cell: {err}")).small().color(theme::current().danger));
        }
        self.errors(ui, &["start", "stop", "bring-up", "power-on", "power-off"]);
    }

    /// Bring up controllers (and its outcome).
    fn bringup_ui(&mut self, ui: &mut egui::Ui, status: &InfraStatus) {
        let ctx = ui.ctx().clone();
        let pending = self.pending.is_some();
        let bringup = &status.bringup;
        let job = status.busy() || pending;
        let wait = wait_reason(status);
        let label = match bringup.state {
            JobState::Waiting => "Waiting for the instances…",
            JobState::Running => "Bringing up the controllers…",
            _ => "Bring up controllers",
        };
        if sidebar::button_response(ui, label, !job && wait.is_none() && !status.all_stopped() && !status.instances.is_empty())
            .on_hover_text(
                "Every KUKA VC host: health check (kvc-cell-setup.sh check), `up` only if it fails, stop messages \
                 acknowledged in T1 (kvc-ack.sh, back to EXT), and the vass-io units. Waits until every instance runs and is SSM-online.",
            )
            .on_disabled_hover_text(wait.clone().unwrap_or_else(|| "Start the cell first (or wait for the running job)".to_owned()))
            .clicked()
        {
            let body = serde_json::to_value(BringupRequest { restart_controllers: self.restart_controllers }).unwrap_or_default();
            self.act(&ctx, "bring-up", api::INFRA_BRINGUP, body);
        }
        crate::ui_mode::advanced(ui, |ui| {
            ui.checkbox(&mut self.restart_controllers, RichText::new("Restart the controllers first").small())
                .on_hover_text("For a controller stuck with M_27002 (docs/CELL.md): kvc-restart.sh on every KUKA VC host");
        });
        job_line(ui, bringup.state, bringup.message.as_deref(), bringup.result.as_deref());
        if let Some(tail) = bringup.output_tail.as_deref().filter(|t| !t.trim().is_empty() && crate::ui_mode::is_advanced(ui)) {
            egui::CollapsingHeader::new(RichText::new("Bring-up output").small()).id_salt("infra_bringup_output").show(ui, |ui| {
                egui::ScrollArea::vertical().max_height(160.0).show(ui, |ui| {
                    ui.label(RichText::new(tail).small().monospace());
                });
            });
        }
    }

    /// Step 3: Deploy (no start): which export, against what the head runs, the confirmation and
    /// the live stages. `choice`: the export step 2 has (picked in the results, else the newest;
    /// `None`: the backend's newest).
    pub fn deploy_ui(&mut self, ui: &mut egui::Ui, choice: Option<DeployChoice>) {
        self.drawn = Some(Instant::now());
        let ctx = ui.ctx().clone();
        let Some(status) = self.usable_status(ui) else {
            sidebar::button_response(ui, "Deploy", false).on_disabled_hover_text("No virtual cell to deploy to");
            self.errors(ui, &["deploy"]);
            return;
        };
        let job = status.busy() || self.pending.is_some();
        let deploy = &status.deploy;
        let target = deploy_target(&status, choice);
        let label = if deploy.state.busy() && !deploy.starts() {
            "Deploying…"
        } else if self.pending == Some("deploy") {
            "Starting the deploy…"
        } else {
            "Deploy (no start)"
        };
        let hover = match &target {
            Some((build, plan, picked)) => format!(
                "Deploy {} (PLAN_ID {}){}: plan2cell cell projects (about 3–15 min: creates and builds the CODESYS projects on \
                 the Windows IDE), deploy (download, boot application, start the applications), krl (the KRL onto the KUKA \
                 virtual controllers). No start: the head then runs the new code but waits in CS_INIT / CS_RECOVERY for Start \
                 production (step 5), and its project is open in the PLC IDE for a review (step 4).",
                export_label(build),
                plan.as_deref().unwrap_or("unknown"),
                if *picked { " (picked in the Control Code Export results)" } else { " (the newest export)" }
            ),
            None => String::new(),
        };
        let why = if target.is_none() {
            "No successful Control Code Export of this cell yet: Export PLC + robot code first (step 2)".to_owned()
        } else {
            wait_reason(&status)
                .unwrap_or_else(|| "Every instance must run and be SSM-online (Start virtual cell, Bring up controllers), and no other job may run".to_owned())
        };
        let off = crate::virtual_cell::blocks(&status);
        // A failing deployability check of this export.
        let block = target.as_ref().and_then(|(build, _, _)| self.check.blocks(build));
        let why = off.clone().or(block.clone()).unwrap_or(why);
        if sidebar::button_response(ui, label, !job && off.is_none() && block.is_none() && status.ready && !status.changing() && target.is_some())
            .on_hover_text(hover)
            .on_disabled_hover_text(why)
            .clicked()
            && let Some((build, plan, _)) = &target
        {
            self.confirm_deploy = Some(ConfirmDeploy { build: build.clone(), plan: plan.clone(), start: false, force: false });
        }
        if let Some((build, plan, _)) = target.as_ref().filter(|_| !deploy.state.busy()) {
            let plan_text = plan.as_deref().map(|p| format!("PLAN_ID {p}")).unwrap_or_else(|| "PLAN_ID unknown".to_owned());
            let head = status.head_plan_id.as_deref();
            let head_text = head.map(|h| format!("the head runs {h}")).unwrap_or_else(|| "the head's PLAN_ID unknown".to_owned());
            let same = matches!((plan.as_deref(), head), (Some(p), Some(h)) if same_plan_id(p, h));
            ui.small(RichText::new(format!("Deploys {} ({plan_text}; {head_text})", export_label(build))).color(if same { Color32::GRAY } else { theme::current().warn }));
        }
        if let Some(block) = &block {
            ui.small(RichText::new(format!("Deploy: {block}")).color(theme::current().danger));
        }
        let checked = target.as_ref().map(|(b, _, _)| b.clone());
        if self.check.ui(ui, checked.as_deref(), off.is_none() && !deploy.state.busy()) == Some(CheckAction::DeployAnyway)
            && let Some((build, plan, _)) = &target
        {
            self.confirm_deploy = Some(ConfirmDeploy { build: build.clone(), plan: plan.clone(), start: false, force: true });
        }
        self.job_ui(ui, &status);
        if self.confirm_deploy.is_some() {
            self.deploy_dialog(&ctx, &status);
        }
        self.errors(ui, &["deploy"]);
    }

    /// The deploy job's stages and outcome (Deploy or Run Control Code), while there is one.
    fn job_ui(&self, ui: &mut egui::Ui, status: &InfraStatus) {
        let deploy = &status.deploy;
        if deploy.state == JobState::Idle {
            return;
        }
        let what = if deploy.starts() { "Run Control Code: " } else { "Deploy: " };
        let color = if deploy.state == JobState::Failed { theme::current().danger } else { Color32::GRAY };
        ui.small(RichText::new(format!("{what}{}", stages_text(deploy))).color(color));
        let build = deploy.build.as_deref().map(export_label);
        job_line(ui, deploy.state, deploy.message.as_deref(), build.as_deref());
    }

    fn deploy_dialog(&mut self, ctx: &egui::Context, status: &InfraStatus) {
        let Some(ConfirmDeploy { build, plan, start, force }) = self.confirm_deploy.clone() else { return };
        let mut go = false;
        let problems = self.check.report_for(&build).map(|r| r.failing().map(|x| x.line()).collect::<Vec<_>>()).unwrap_or_default();
        let modal = egui::Modal::new(egui::Id::new("infra_deploy_confirm")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(440.0);
            ui.heading(match (force, start) {
                (true, _) => "Deploy anyway?",
                (false, true) => "Run Control Code?",
                (false, false) => "Deploy to the virtual cell?",
            });
            if force {
                ui.label(RichText::new("The deployability check failed: the deploy will most likely fail on the cell.").color(theme::current().danger));
                for p in &problems {
                    ui.label(RichText::new(p).small().color(theme::current().danger));
                }
            }
            ui.label(format!(
                "Deploys {} (PLAN_ID {}): this replaces the code on the head PLC and the robots.",
                export_label(&build),
                plan.as_deref().unwrap_or("unknown")
            ));
            if let Some(head) = &status.head_plan_id {
                ui.label(format!("The head runs PLAN_ID {head} now."));
            }
            if start {
                ui.label(
                    "plan2cell cell projects, deploy, krl, then start --restart-runtime --pulses 16, without a stop for the review \
                     in the PLC IDE. A running cell stops; auto-restart is paused until it is done.",
                );
            } else {
                ui.label(
                    "plan2cell cell projects, deploy, krl; no start. A running cell stops (the download restarts the \
                     applications). Afterwards the head runs the new code but waits for Start production (CS_INIT / \
                     CS_RECOVERY), and its project is open in the PLC IDE.",
                );
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button(RichText::new(if start { "Deploy and start" } else { "Deploy" }).color(theme::current().warn)).clicked() {
                    go = true;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_deploy = None;
                }
            });
        });
        if modal.should_close() {
            self.confirm_deploy = None;
        }
        if go {
            self.confirm_deploy = None;
            let body = serde_json::to_value(DeployRequest { build: Some(build), start, force, ..Default::default() }).unwrap_or_default();
            self.act(ctx, if start { "run" } else { "deploy" }, api::INFRA_DEPLOY, body);
        }
    }

    /// Step 4: Open PLC IDE and which project to look at.
    pub fn review_ui(&mut self, ui: &mut egui::Ui) {
        self.drawn = Some(Instant::now());
        let status = self.status.clone();
        self.ide_button(ui, status.as_ref());
        if crate::ui_mode::is_advanced(ui) {
            self.smartpad_buttons(ui, status.as_ref());
        }
        if let Some(project) = status.as_ref().and_then(|s| s.ide_project.as_ref()) {
            ui.small(RichText::new(format!("Look at {} in the CODESYS window titled \"{}.project - CODESYS … [MCP]\" (it comes to the front when the window connects; else restore it from the taskbar).", project.name, project.name)).weak())
                .on_hover_text(format!(
                    "{}\nThe CODESYS the Codesys-MCP drives: the deploy left the head project open in it, logged out. \
                     Opening the .project in a second CODESYS fails (the project is locked).",
                    project.path
                ));
            ui.small(
                RichText::new(
                    "View only: Start production, Restart PLC runtime, auto-restart, Stop production and Deploy discard unsaved edits \
                     in that window first. Don't save (a saved edit would go to the head PLC with the next start), don't log in, \
                     and close any dialog you open (an open dialog blocks every automated step).",
                )
                .color(theme::current().warn),
            );
        }
    }

    /// Step 5: Start production, Restart PLC runtime, the demo countdown, auto-restart, Stop
    /// production, and Run Control Code (deploy + start in one go).
    /// `live`: the head's live state, shown above the last job's line; with the head in error or
    /// recovery, Start production is enabled again even while the backend counts the cell as in
    /// production.
    pub fn production_ui(&mut self, ui: &mut egui::Ui, choice: Option<DeployChoice>, live: &crate::flow::LiveHead) {
        self.drawn = Some(Instant::now());
        let ctx = ui.ctx().clone();
        let Some(status) = self.usable_status(ui) else {
            sidebar::button_response(ui, "Start production", false).on_disabled_hover_text("No virtual cell");
            self.errors(ui, &["production", "restart", "auto-restart", "stop-production", "run"]);
            return;
        };
        let job = status.busy() || self.pending.is_some();
        let cell = &status.cell;
        let label = if cell.state.busy() && !status.deploy.state.busy() { "Starting production…" } else { "Start production" };
        let head_plan = status.head_plan_id.as_deref().or_else(|| status.head_export.as_ref().and_then(|e| e.plan_id.as_deref()));
        let start_hover = match (&status.head_export, &status.head_export_error) {
            (Some(export), _) => format!(
                "Starts the code the head runs (PLAN_ID {}): plan2cell cell start {} --restart-runtime: fresh CODESYS demo runtimes, \
                 KUKA VCs into EXT, reset pulses until the head runs (CS_RUNNING).{}",
                head_plan.unwrap_or("unknown"),
                export_label(&export.build),
                match export.source.as_str() {
                    "report" => " (the export last deployed: the head's PLAN_ID is not known yet)",
                    "newest" => " (the newest export: no deploy known; the head's PLAN_ID guard checks it)",
                    _ => "",
                }
            ),
            (None, Some(err)) => format!("Starts the code the head runs: {err}"),
            (None, None) => "Starts the code the head runs (plan2cell cell start --restart-runtime)".to_owned(),
        };
        let why = wait_reason(&status)
            .unwrap_or_else(|| "Every instance must run and be SSM-online (Start virtual cell, Bring up controllers), and no other job may run".to_owned());
        let why = why.as_str();
        // What is bringing the cell into production right now (Start cell's chain, a bring-up, a start).
        let progress = if status.start.state.busy() {
            Some(format!("Start cell is bringing the cell into production: {}", status.start.message.as_deref().unwrap_or("…")))
        } else if status.bringup.state.busy() {
            Some(format!("Bringing up the controllers: {}", status.bringup.message.as_deref().unwrap_or("…")))
        } else if cell.state.busy() {
            Some(format!("Starting production: {}", cell.message.as_deref().unwrap_or("…")))
        } else {
            None
        };
        if let Some(progress) = &progress {
            ui.horizontal_wrapped(|ui| {
                ui.spinner();
                ui.label(RichText::new(progress).small().color(theme::current().warn));
            });
        }
        let (head_line, head_color) = live.line();
        ui.label(RichText::new(head_line).small().color(head_color));
        let producing = status.in_production && !job && !live.needs_start();
        let start_why = if producing {
            "Already in production: Stop production first, or Restart PLC runtime for a fresh demo period"
        } else {
            why
        };
        if sidebar::button_response(ui, label, !job && !producing && status.ready && !status.changing())
            .on_hover_text(start_hover)
            .on_disabled_hover_text(start_why)
            .clicked()
        {
            self.act(&ctx, "production", api::INFRA_PRODUCTION, serde_json::json!({}));
        }
        ui.horizontal(|ui| {
            let half = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let label = if self.pending == Some("restart") { "Restarting…" } else { "Restart PLC runtime" };
                if sidebar::button_response(ui, label, !job && status.ready && !status.changing())
                    .on_hover_text(
                        "A new 2 h CODESYS demo period: the production start with --restart-runtime on the newest export of the \
                         PLAN_ID the head runs (restarts every runtime container, KUKA VCs into EXT, pulses the head into CS_RUNNING)",
                    )
                    .on_disabled_hover_text(why)
                    .clicked()
                {
                    let body = serde_json::to_value(ProductionRequest { match_deployed: true, build: None }).unwrap_or_default();
                    self.act(&ctx, "restart", api::INFRA_PRODUCTION, body);
                }
            });
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let label = if self.pending == Some("stop-production") { "Stopping…" } else { "Stop production" };
                if sidebar::button_response(ui, label, !job && status.ready && !status.changing() && status.head_export.is_some() && status.in_production)
                    .on_hover_text(
                        "plan2cell cell stop: pulses PLC_PRG.bStop, the head ends in CS_ERROR ERR_OPERATOR_STOP. The instances and \
                         runtimes keep running; auto-restart stays out; Start production starts it again.",
                    )
                    .on_disabled_hover_text(why)
                    .clicked()
                {
                    self.act(&ctx, "stop-production", api::INFRA_PRODUCTION_STOP, serde_json::json!({}));
                }
            });
        });
        // The last job, secondary to the live head above.
        let head = cell.head_state.as_deref().map(|h| format!("head {h} then"));
        job_line(ui, cell.state, cell.message.as_deref(), head.as_deref());
        match &status.plc_runtime {
            _ if !crate::ui_mode::is_advanced(ui) => {}
            Some(rt) => runtime_line(ui, rt, now_s()),
            None => {
                if let Some(note) = &cell.demo_note {
                    ui.small(RichText::new(note).weak());
                }
            }
        }
        let auto = &status.auto_restart;
        let mut enabled = auto.enabled;
        let minutes = auto.threshold_s.div_ceil(60).max(1);
        if !crate::ui_mode::is_advanced(ui) {
            // Auto-restart: Advanced only (it keeps its setting).
        } else if !virtual_cell::status_allows(Some(&status), "runtime.codesys") {
            // Its template's head runs no CODESYS demo runtime: nothing stops after 2 h (the backend keeps it inactive).
            ui.small(RichText::new("Auto-restart: not needed (this cell's template has no 2 h demo runtime)").color(Color32::GRAY));
        } else if status.cell_role == CellRole::Operator {
            // Another backend (the cell's supervisor) runs it; this one never does.
            ui.small(RichText::new(supervisor_line(&status)).color(Color32::GRAY));
        } else if ui
            .add_enabled(self.pending.is_none(), egui::Checkbox::new(&mut enabled, RichText::new("Auto-restart (2 h demo stop, head errors): run for ever").small()))
            .on_hover_text(format!(
                "Only while the cell is in production (Start production succeeded, no Stop cell, Stop production or deploy since): \
                 Restart PLC runtime by itself {minutes} min before the first CODESYS runtime of the export stops, or when the \
                 head's runtime stopped or docker restarted it, or when the head stopped production with an error of its own \
                 (CS_ERROR for 20 s, e.g. ERR_DRIVES_LOST after a PROFINET hiccup; not Stop production). That restarts every runtime and pulses the head into CS_RUNNING \
                 again. Never during another job, at most once per 10 min; a refused PLAN_ID or 3 failures in a row hold it \
                 until you switch it on again."
            ))
            .changed()
        {
            self.pending = Some("auto-restart");
            self.action_error = None;
            let send = self.sender(&ctx);
            let body = AutoRestartSettings { enabled, threshold_s: Some(auto.threshold_s).filter(|t| *t > 0) };
            self.api.put_json(api::INFRA_AUTO_RESTART, &body, move |result| send(Msg::AutoRestart(result)));
        }
        if let Some(message) = auto.message.as_ref().filter(|_| {
            status.cell_role != CellRole::Operator && crate::ui_mode::is_advanced(ui) && virtual_cell::status_allows(Some(&status), "runtime.codesys")
        }) {
            let color =
                if auto.blocked.is_some() || auto.last_error.is_some() || auto.inactive.is_some() { theme::current().danger } else { Color32::GRAY };
            ui.small(RichText::new(message).color(color));
        }

        // The shortcut: deploy and start in one go, without the review.
        let target = deploy_target(&status, choice);
        let label = if status.deploy.state.busy() && status.deploy.starts() { "Running Control Code…" } else { "Run Control Code (deploy + start)" };
        let hover = target.as_ref().map_or_else(String::new, |(build, plan, _)| {
            format!(
                "Steps 3 and 5 in one go, without the review: plan2cell cell projects, deploy, krl, start --restart-runtime \
                 --pulses 16 of {} (PLAN_ID {}).",
                export_label(build),
                plan.as_deref().unwrap_or("unknown")
            )
        });
        let block = target.as_ref().and_then(|(build, _, _)| self.check.blocks(build));
        let why = block.clone().unwrap_or_else(|| why.to_owned());
        // The shortcut is Advanced only: Simple follows steps 3 and 5.
        if crate::ui_mode::is_advanced(ui)
            && ui
            .add_enabled(!job && block.is_none() && status.ready && !status.changing() && target.is_some(), egui::Button::new(RichText::new(label).small()).corner_radius(4))
            .on_hover_text(hover)
            .on_disabled_hover_text(why)
            .clicked()
            && let Some((build, plan, _)) = &target
        {
            self.confirm_deploy = Some(ConfirmDeploy { build: build.clone(), plan: plan.clone(), start: true, force: false });
        }
        if status.deploy.starts() {
            self.job_ui(ui, &status);
        }
        // Its confirmation is drawn by step 3 (one modal per frame).
        self.errors(ui, &["production", "restart", "auto-restart", "stop-production", "run"]);
    }

    /// The last action's error, if it was one of `whats`.
    fn errors(&self, ui: &mut egui::Ui, whats: &[&str]) {
        if let Some((what, err)) = &self.action_error
            && whats.contains(&what.as_str())
        {
            ui.label(RichText::new(err).small().color(theme::current().danger));
        }
    }

    /// "Open PLC IDE": the Windows IDE's NICE DCV web client in a new window, through the backend's
    /// `/dcv/` with a fresh session token (the DCV server asks the backend about it: no Windows
    /// password).
    fn ide_button(&mut self, ui: &mut egui::Ui, status: Option<&InfraStatus>) {
        let ide = status.and_then(|s| s.instance(ROLE_WINDOWS_IDE));
        let gateway = status.is_some_and(|s| s.has_dcv_target(DCV_TARGET_IDE));
        // Without a Windows IDE in the status, the backend's own DCV server (FACTORY_OS_DCV_SERVER).
        let fallback = status.is_some_and(|s| s.dcv_server_fallback) && ide.is_none();
        let power_why = status.and_then(virtual_cell::blocks);
        // An EKS cell with the container IDE: CODESYS runs headless in a Windows pod, there is no desktop to show
        // (a template's cell: whenever its template has no `ide.dcv`).
        let own = virtual_cell::own_cell(status, self.cells.as_ref());
        let headless = own.is_some_and(|c| !c.has_dcv());
        let ready = gateway && (ide.is_some_and(InfraInstance::running) || fallback) && !self.dcv_pending && power_why.is_none() && !headless;
        let why = match (status, ide) {
            _ if headless && own.is_some_and(|c| !c.eks()) => format!(
                "This cell's template ({}) has no PLC IDE desktop (NICE DCV)",
                own.and_then(|c| c.template.as_ref()).map_or_else(|| "unknown".to_owned(), |t| t.label())
            ),
            _ if headless => "This cell has the compile node only (EKS, headless: no desktop). For Open PLC IDE add the EC2 IDE \
                              (Advanced, IDE: compile node + EC2 IDE, while the cell is off)"
                .to_owned(),
            _ if power_why.is_some() => power_why.clone().unwrap_or_default(),
            (None, _) => match &self.status_error {
                Some(err) => format!("Can't read the virtual cell: {err}"),
                None => "Reading the virtual cell…".to_owned(),
            },
            (Some(s), _) if !s.dcv_gateway => "NICE DCV is not set up on this server (no DCV Connection Gateway)".to_owned(),
            (Some(s), _) if !s.has_dcv_target(DCV_TARGET_IDE) => format!(
                "This server's NICE DCV has no PLC IDE target (it serves {})",
                if s.dcv_targets.is_empty() { "none".to_owned() } else { s.dcv_targets.join(", ") }
            ),
            (Some(s), None) if s.dcv_server_fallback => "Opening…".to_owned(),
            (Some(s), _) if !s.available => "No cell control on this server".to_owned(),
            (_, Some(i)) if !i.running() => format!("The Windows IDE is {}: Start virtual cell first", i.state),
            (_, None) => "No Windows IDE in this cell".to_owned(),
            _ => "Opening…".to_owned(),
        };
        // The session link is asked for ahead of the click, so the click opens it right away, as a
        // link does (a window opened empty and pointed at the URL later stays blank in some
        // browsers). An unused link expires after 5 minutes: renewed every 4; a used one keeps
        // working from this computer for hours.
        let now = now_s();
        let stale = match &self.dcv_link {
            None => true,
            Some(_) => !self.dcv_used && now - self.dcv_asked_s > 240.0,
        };
        if ready && stale && !self.dcv_pending && self.dcv_error.is_none() {
            self.dcv_pending = true;
            self.dcv_asked_s = now;
            let send = self.sender(ui.ctx());
            // JSON, so another site's page could never send it without a CORS preflight.
            let request = DcvSessionRequest { target: Some(DCV_TARGET_IDE.to_owned()) };
            self.api.post_json(api::INFRA_DCV_SESSION, &request, move |session| send(Msg::Dcv(session)));
        }
        if ready && self.dcv_link.is_some() {
            ui.ctx().request_repaint_after(std::time::Duration::from_secs(30));
        }
        let link = self.dcv_link.clone().filter(|_| ready);
        let label = if link.is_none() && ready { "Preparing PLC IDE…" } else { "Open PLC IDE" };
        let response = sidebar::button_response(ui, label, link.is_some())
            .on_hover_text(format!(
                "CODESYS on the Windows IDE through NICE DCV ({}), in a window of its own (signed in already)",
                api::dcv_target(DCV_TARGET_IDE)
            ))
            .on_disabled_hover_text(if ready { "Getting a PLC IDE session…".to_owned() } else { why });
        if let Some((url, issued)) = link {
            if response.clicked() || response.middle_clicked() {
                // Web: a window of its own, the smartPADs' size (in the click's user gesture). Native: the
                // system browser (webbrowser), as every other link of the native viewer.
                #[cfg(target_arch = "wasm32")]
                crate::web::open_popup(&url, "plc-ide");
                #[cfg(not(target_arch = "wasm32"))]
                crate::workspace_ui::open_url(&url);
                self.dcv_used = true;
                self.ide_opened_s = Some(issued);
            }
        }
        if let Some(err) = self.dcv_error.clone() {
            let retry = ui
                .horizontal_wrapped(|ui| {
                    ui.label(RichText::new(format!("Open PLC IDE: {err}")).small().color(theme::current().danger));
                    ui.small_button("Retry").clicked()
                })
                .inner;
            if retry {
                self.dcv_error = None;
            }
        }
        if self.dcv_link.is_some()
            && crate::ui_mode::is_advanced(ui)
            && ui
                .small_button("Sign out")
                .on_hover_text("Revoke every PLC IDE link of this server (open windows stay connected)")
                .clicked()
        {
            let send = self.sender(ui.ctx());
            self.api.delete(api::INFRA_DCV_SESSION, move |revoked| send(Msg::DcvRevoked(revoked)));
        }
        if status.is_some_and(|s| !s.dcv_gateway) && crate::ui_mode::is_advanced(ui) {
            ui.small(
                RichText::new(
                    "The PLC IDE opens through this server's DCV Connection Gateway, which it doesn't have (plan2cell/infra/test-server/dcv-gateway.sh).",
                )
                .weak(),
            );
        }
    }

    /// "Open smartPAD <robot>": per KUKA controller of the cell, its smartPAD (KDM's iiQKA.UI) in a browser window of
    /// its own (named after the robot: a second click brings that window back), at the controller's own hostname on
    /// the workspace edge, which the backend proxies to the controller's KDM.
    fn smartpad_buttons(&mut self, ui: &mut egui::Ui, status: Option<&InfraStatus>) {
        // A cell whose template has no smartPADs (the backend lists none for it either).
        if self.smartpads_unsupported || !virtual_cell::status_allows(status, "smartpad") {
            return;
        }
        let Some(list) = self.smartpads.clone() else {
            if let Some(err) = &self.smartpads_error {
                ui.small(RichText::new(format!("smartPADs: {err}")).color(theme::current().danger));
            }
            return;
        };
        if list.pads.is_empty() {
            return;
        }
        let power_why = status.and_then(virtual_cell::blocks);
        ui.horizontal_wrapped(|ui| {
            for pad in &list.pads {
                // robot<N> runs on KUKA VC host kukaVc<N> (an older Lambda's single host: kukaVc).
                let role = pad.robot.strip_prefix("robot").map(|n| format!("kukaVc{n}"));
                let host = status.and_then(|s| role.as_deref().and_then(|r| s.instance(r)).or_else(|| s.instance(ROLE_KUKA_VC)));
                let why = if let Some(why) = &power_why {
                    Some(why.clone())
                } else if !list.may_open {
                    Some("The smartPADs of this workspace are its owner's".to_owned())
                } else if pad.url.is_none() {
                    Some(list.reason.clone().unwrap_or_else(|| "No smartPAD address".to_owned()))
                } else {
                    host.filter(|i| !i.running()).map(|i| format!("The KUKA VC host of {} is {}: Start virtual cell first", pad.robot, i.state))
                };
                let label = format!("Open smartPAD {}", pad.robot);
                let response = sidebar::button_response(ui, &label, why.is_none())
                    .on_hover_text(format!(
                        "The smartPAD of {} (KDM's iiQKA.UI on its controller, KLI {}), in a browser window of its own: {}",
                        pad.robot,
                        pad.kli_ip,
                        pad.url.as_deref().unwrap_or("-")
                    ))
                    .on_disabled_hover_text(why.unwrap_or_default());
                if let Some(url) = &pad.url
                    && (response.clicked() || response.middle_clicked())
                {
                    #[cfg(target_arch = "wasm32")]
                    crate::web::open_popup(url, &format!("smartpad-{}", pad.robot));
                    #[cfg(not(target_arch = "wasm32"))]
                    crate::workspace_ui::open_url(url);
                }
            }
        });
    }

    /// The workspace's own cell: none (Create virtual cell), off (Start, Delete…), starting (the
    /// phase stepper), ready (Stop), stopping (archiving → terminating), its images and cost.
    fn own_cell_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let status = self.status.clone();
        let cell = virtual_cell::own_cell(status.as_ref(), self.cells.as_ref()).cloned();
        let state = virtual_cell::own_state(cell.as_ref());
        let can_create = self.cells.as_ref().is_some_and(|l| l.can_create);
        let controls = virtual_cell::controls(state, can_create);
        let pending = self.pending.is_some();
        let errors = ["create", "delete", "power-on", "power-fresh", "power-off", "off-mode", "runtime", "template", "params", "bring-up"];
        let Some(cell) = cell else {
            if status.is_none() && self.cells.is_none() {
                match &self.status_error {
                    Some(err) => ui.small(RichText::new(format!("Can't read the virtual cell: {err}")).color(theme::current().danger)),
                    None => ui.small(RichText::new("Reading the virtual cell…").weak()),
                };
                return;
            }
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("Virtual cell:").strong());
                ui.label(RichText::new("● none").color(Color32::GRAY).strong());
            });
            ui.small(RichText::new("No virtual cell: this workspace has none yet.").weak());
            let label = if self.pending == Some("create") { "Creating…" } else { "Create virtual cell" };
            let reason = self.cells.as_ref().and_then(|l| l.reason.clone()).unwrap_or_else(|| "Reading the cells…".to_owned());
            let template = self.create_choice();
            let unavailable = template.as_ref().and_then(virtual_cell::unavailable_text);
            let hover = match &template {
                Some(t) => format!(
                    "A virtual cell of this workspace's own from the template {} ({}@{}), tagged with this workspace: {} Creating it \
                     costs nothing; it runs (and costs) only while it is on, and it stops with the workspace.",
                    template_title(t),
                    t.id,
                    t.version,
                    t.description
                ),
                None => "A virtual cell of this workspace's own: PLC host, Windows CODESYS IDE and the KUKA VC hosts, tagged with \
                         this workspace. Creating it costs nothing; its instances run (and cost) only while it is on, and it stops \
                         with the workspace."
                    .to_owned(),
            };
            if sidebar::button_response(ui, label, !pending && controls.create && unavailable.is_none())
                .on_hover_text(hover)
                .on_disabled_hover_text(unavailable.filter(|_| controls.create).unwrap_or_else(|| reason.clone()))
                .clicked()
            {
                self.create_cell(&ctx);
            }
            let eks = self.cells.as_ref().is_some_and(|l| l.runtimes.iter().any(|r| r == RUNTIME_EKS));
            crate::ui_mode::advanced(ui, |ui| {
                ui.checkbox(&mut self.create_start, RichText::new("then start it").small())
                    .on_hover_text("Start virtual cell right after creating it (launches its instances)");
                if template.is_some() {
                    self.template_choice(ui);
                } else if eks {
                    self.runtime_choice(ui);
                }
            });
            let hint = template.as_ref().map_or_else(|| virtual_cell::cost_hint(None), virtual_cell::template_cost_hint);
            ui.small(RichText::new(hint).weak());
            if !can_create && self.cells.is_some() {
                ui.small(RichText::new(reason).color(theme::current().warn));
            }
            self.errors(ui, &errors);
            return;
        };

        let color = own_color(state);
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Virtual cell:").strong());
            ui.label(RichText::new("●").color(color));
            ui.label(RichText::new(virtual_cell::own_label(Some(&cell), state)).color(color).strong());
            if state.transitional() {
                ui.spinner();
            }
            ui.add_visible(crate::ui_mode::is_advanced(ui), egui::Label::new(RichText::new(&cell.id).small().weak())).on_hover_text(format!(
                "Your own virtual cell (workspace {}, launch {}, {} robots){}{}",
                cell.workspace,
                cell.launch,
                cell.robots,
                cell.created_at.as_deref().map(|c| format!(", created {c}")).unwrap_or_default(),
                cell.template.as_ref().map(|t| format!(", template {} from {}", t.label(), t.source)).unwrap_or_default()
            ));
        });
        // What it boots from (an older Lambda: nothing).
        if let Some(line) = virtual_cell::template_line(&cell).filter(|_| crate::ui_mode::is_advanced(ui)) {
            let hover = cell.template.as_ref().map(|t| format!("sha256 {}\ncapabilities: {}", t.sha256, t.capabilities.join(", "))).unwrap_or_default();
            ui.small(RichText::new(line).weak()).on_hover_text(hover);
        }
        match state {
            OwnState::Starting(i) => {
                ui.small(RichText::new(virtual_cell::stepper_text(&START_STEPS, i, false)).color(theme::current().warn));
                let elapsed = virtual_cell::session_since(&cell).map(|since| format!("{} so far, ", virtual_cell::elapsed_text(since, now_s())));
                ui.small(RichText::new(format!("{}{USUAL_START}", elapsed.unwrap_or_default())).weak());
            }
            OwnState::Stopping(i) => {
                ui.small(RichText::new(virtual_cell::stepper_text(virtual_cell::stop_steps(&cell), i, false)).color(theme::current().warn));
            }
            _ => {}
        }
        if let Some(power) = cell.power.as_ref().filter(|_| matches!(state, OwnState::Starting(_) | OwnState::Ready | OwnState::Trouble)) {
            power_details(ui, power, false);
        }
        if let Some((job, text)) = virtual_cell::restore_text(&cell) {
            job_line(ui, job, Some(&text), None);
        }
        if let Some(message) = cell.message.as_deref().filter(|m| !m.is_empty()) {
            let color = if state == OwnState::Failed { theme::current().danger } else { Color32::GRAY };
            ui.small(RichText::new(message).color(color));
        }
        let instances = status.as_ref().map(|s| s.instances.clone()).unwrap_or_default();
        if matches!(state, OwnState::Starting(_) | OwnState::Ready | OwnState::Trouble | OwnState::Stopping(_)) && crate::ui_mode::is_advanced(ui) {
            for i in &instances {
                chip(ui, &i.role, Some(i));
            }
        }

        for line in virtual_cell::stopped_lines(&cell, now_s()) {
            ui.small(RichText::new(line).color(Color32::GRAY));
        }
        let resumes = virtual_cell::resumes(&cell);
        // A template without archive: its Stop keeps nothing. The restore is not gated by the template (the
        // backend restores every fresh launch, as today: the eks-codesys-kuka template has no `restore`).
        let archives = virtual_cell::allows(&cell, "archive");
        ui.horizontal(|ui| {
            let half = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let idle = if resumes { "Resume virtual cell" } else { "Start virtual cell" };
                let label = if self.pending == Some("power-on") { "Starting…" } else { idle };
                let hover = if resumes {
                    format!(
                        "Starts the stopped instances again (no first boot, no restore: the control code is on their disks), brings \
                         up the KUKA controllers and checks the cell ({USUAL_RESUME} to ready; a due patch run adds minutes); in \
                         production when it stopped: Start production. {}",
                        virtual_cell::mode_cost_hint(&cell)
                    )
                } else {
                    format!(
                        "Launches the PLC host, the KUKA VC hosts and the Windows IDE fresh from the latest images, configures \
                         them, brings up the KUKA controllers and checks the cell; then restores the last deployed control code \
                         ({USUAL_START}). {}",
                        virtual_cell::mode_cost_hint(&cell)
                    )
                };
                if sidebar::button_response(ui, label, !pending && controls.start)
                    .on_hover_text(hover)
                    .on_disabled_hover_text(match state {
                        OwnState::Stopping(_) => "Wait: the cell is stopping",
                        OwnState::Deleting => "The cell is being deleted",
                        _ => "The virtual cell is on already, or on its way",
                    })
                    .clicked()
                {
                    let body = CellPowerRequest {
                        action: "on".into(),
                        production: Some(self.start_follow),
                        restore: Some(self.restore_mode.to_owned()),
                        // Resume chosen here (the newer images are shown above; Start fresh takes them).
                        fresh: cell.stopped.then_some(false),
                        ..Default::default()
                    };
                    self.act(&ctx, "power-on", api::INFRA_CELL_POWER, serde_json::to_value(body).unwrap_or_default());
                }
            });
            ui.allocate_ui(egui::vec2(half, 30.0), |ui| {
                let label = if self.pending == Some("power-off") { "Stopping…" } else { "Stop virtual cell" };
                if sidebar::button_response(ui, label, !pending && controls.stop)
                    .on_hover_text(match (cell.stop_mode(), archives) {
                        (true, true) => {
                            "Archives the boot applications and the PLC IDE's projects, then stops the cell's instances: their disks \
                             are kept (and paid), and the next Start resumes them. To stop only the cell's motion: Stop production \
                             (step 5)."
                        }
                        (false, true) => {
                            "Archives the boot applications and the PLC IDE's projects, then terminates the cell's instances: \
                             nothing is paid while it is off, and the next Start builds them fresh from the latest images. To \
                             stop only the cell's motion: Stop production (step 5)."
                        }
                        (true, false) => {
                            "Stops the cell's instances (its template archives nothing): their disks are kept (and paid), and the \
                             next Start resumes them. To stop only the cell's motion: Stop production (step 5)."
                        }
                        (false, false) => {
                            "Removes the cell's instances or pods (its template archives nothing): nothing is paid while it is off, \
                             and the next Start builds it fresh. To stop only the cell's motion: Stop production (step 5)."
                        }
                    })
                    .on_disabled_hover_text("Nothing runs")
                    .clicked()
                {
                    self.confirm_stop = true;
                }
            });
        });
        // Start fresh: a stopped cell (instead of resuming it), or a resume that did not get there (one click).
        let resuming = matches!(state, OwnState::Starting(_)) && cell.power.as_ref().and_then(|p| p.session.as_ref()).is_some_and(|s| s.resume);
        if virtual_cell::offer_fresh(&cell) || resuming {
            let label = if self.pending == Some("power-fresh") { "Starting fresh…" } else { "Start fresh" };
            if sidebar::button_response(ui, label, !pending && (controls.start || resuming))
                .on_hover_text(
                    "Terminates the cell's instances and launches new ones from the latest images (a first boot), then restores \
                     the last deployed control code from the archive of the last stop (usually 6–8 min)",
                )
                .on_disabled_hover_text("Stop the cell first")
                .clicked()
            {
                let body = CellPowerRequest {
                    action: "on".into(),
                    production: Some(self.start_follow),
                    restore: Some(self.restore_mode.to_owned()),
                    fresh: Some(true),
                    ..Default::default()
                };
                self.act(&ctx, "power-fresh", api::INFRA_CELL_POWER, serde_json::to_value(body).unwrap_or_default());
            }
        }
        // What Stop does, then start production, the restore: Advanced only (they keep their values).
        // A cell that runs a template the backend lists: its template and the form of its parameters, instead of the
        // runtime, networks and off-mode pickers below (Advanced, as those).
        let template_ui = virtual_cell::templates_offered(self.cells.as_ref()) && cell.template.is_some();
        if template_ui && crate::ui_mode::is_advanced(ui) && !matches!(state, OwnState::Deleting | OwnState::None) {
            self.cell_template_ui(ui, &cell);
        }
        // The runtime (EC2 instances / EKS pods) and an EKS cell's IDE: shown always, changed only while it is off.
        let eks_offered = self.cells.as_ref().is_some_and(|l| l.runtimes.iter().any(|r| r == RUNTIME_EKS));
        if crate::ui_mode::is_advanced(ui) && !template_ui && (cell.eks() || eks_offered) && !matches!(state, OwnState::Deleting | OwnState::None) {
            let off = cell.phase() == "off" && !cell.stopped;
            let before = (if cell.eks() { RUNTIME_EKS } else { RUNTIME_EC2 }, if cell.ide.as_deref() == Some(IDE_EC2) { IDE_EC2 } else { IDE_CONTAINER });
            let (mut runtime, mut ide) = before;
            ui.add_enabled_ui(off && !pending, |ui| {
                // One below the other, each as wide as the pane (their labels are long).
                let width = ui.available_width();
                theme::truncated_combo(ui, "own_cell_runtime", RichText::new(runtime_label(runtime)).small(), width, |ui| {
                    for r in RUNTIMES {
                        ui.selectable_value(&mut runtime, r, runtime_label(r));
                    }
                })
                .response
                .on_hover_text(RUNTIME_HINT)
                .on_disabled_hover_text("The runtime changes only while the cell is off (not stopped)");
                if runtime == RUNTIME_EKS {
                    theme::truncated_combo(ui, "own_cell_ide", RichText::new(ide_label(ide)).small(), width, |ui| {
                        for i in IDES {
                            ui.selectable_value(&mut ide, i, ide_label(i));
                        }
                    })
                    .response
                    .on_hover_text(IDE_HINT);
                }
            });
            if (runtime, ide) != before && off && !pending {
                self.pending = Some("runtime");
                self.action_error = None;
                let body = UpdateCellRequest {
                    runtime: (runtime != before.0).then(|| runtime.to_owned()),
                    ide: (runtime == RUNTIME_EKS && (ide != before.1 || runtime != before.0)).then(|| ide.to_owned()),
                    ..Default::default()
                };
                let send = self.sender(&ctx);
                self.api.put_json(&api::INFRA_CELL_ITEM.replace("{id}", &cell.id), &body, move |result| send(Msg::CellAction("runtime", result)));
            }
        }
        // An EKS cell's networks: listed; their number changed only while it is off.
        if crate::ui_mode::is_advanced(ui) && !template_ui && cell.eks() && !matches!(state, OwnState::Deleting | OwnState::None) {
            let off = cell.phase() == "off" && !cell.stopped;
            let mut changed: Option<Vec<CellNetwork>> = None;
            for (i, n) in cell.networks.iter().enumerate() {
                ui.horizontal_wrapped(|ui| {
                    ui.small(RichText::new(format!("{}: {}", n.name, n.subnet.as_deref().unwrap_or("?"))).weak());
                    let mut robots = n.robots;
                    ui.add_enabled_ui(off && !pending, |ui| robots_combo(ui, &format!("own_cell_robots_{}", n.name), &mut robots))
                        .response
                        .on_disabled_hover_text("The robots change only while the cell is off");
                    if robots != n.robots && changed.is_none() {
                        let mut nets: Vec<CellNetwork> =
                            cell.networks.iter().map(|m| CellNetwork { name: m.name.clone(), robots: m.robots, ..Default::default() }).collect();
                        nets[i].robots = robots;
                        changed = Some(nets);
                    }
                });
            }
            if let Some(nets) = changed.filter(|_| off && !pending) {
                self.pending = Some("runtime");
                self.action_error = None;
                let body = UpdateCellRequest { networks: Some(nets), ..Default::default() };
                let send = self.sender(&ctx);
                self.api.put_json(&api::INFRA_CELL_ITEM.replace("{id}", &cell.id), &body, move |result| send(Msg::CellAction("runtime", result)));
            }
            let off = cell.phase() == "off" && !cell.stopped;
            let before = cell.networks.len().max(1);
            let mut count = before;
            ui.add_enabled_ui(off && !pending, |ui| networks_combo(ui, "own_cell_networks", &mut count))
                .response
                .on_disabled_hover_text("The networks change only while the cell is off");
            if count != before && off && !pending {
                self.pending = Some("runtime");
                self.action_error = None;
                let body = UpdateCellRequest { networks: Some(networks_of(count, &cell.networks, 2)), ..Default::default() };
                let send = self.sender(&ctx);
                self.api.put_json(&api::INFRA_CELL_ITEM.replace("{id}", &cell.id), &body, move |result| send(Msg::CellAction("runtime", result)));
            }
        }
        // An EKS cell's Stop always deletes its pods (no stop mode).
        if !matches!(state, OwnState::Deleting | OwnState::None) && crate::ui_mode::is_advanced(ui) && !template_ui && !cell.eks() {
            let mut mode = if cell.stop_mode() { OFF_MODE_STOP } else { OFF_MODE_TERMINATE };
            let before = mode;
            let width = ui.available_width();
            theme::truncated_combo(ui, "own_cell_off_mode", RichText::new(virtual_cell::off_mode_label(mode)).small(), width, |ui| {
                for m in OFF_MODES {
                    ui.selectable_value(&mut mode, m, virtual_cell::off_mode_label(m));
                }
            })
            .response
                .on_hover_text(format!(
                    "What Stop does with this cell. Terminate (the default): nothing is paid while it is off, every Start is a fresh \
                     launch from the latest images ({USUAL_START}). Keep (stop): the instances are stopped, their disks paid \
                     (~${:.2}/day), and Start resumes them ({USUAL_RESUME} to ready); after a long stop, newer images or another \
                     robot count it starts fresh.",
                    virtual_cell::stopped_usd_per_day(Some(&cell.cost))
                ));
            if mode != before && !pending {
                self.pending = Some("off-mode");
                self.action_error = None;
                let body = UpdateCellRequest { off_mode: Some(mode.to_owned()), ..Default::default() };
                let send = self.sender(&ctx);
                self.api.put_json(&api::INFRA_CELL_ITEM.replace("{id}", &cell.id), &body, move |result| send(Msg::CellAction("off-mode", result)));
            }
        }
        if controls.start && crate::ui_mode::is_advanced(ui) {
            ui.checkbox(&mut self.start_follow, RichText::new("then start production").small()).on_hover_text(
                "Once the cell is ready and the control code is restored, Start production (the code the head runs). Off: a \
                 cell that was in production when it stopped is started again anyway.",
            );
            let width = ui.available_width();
            theme::truncated_combo(ui, "own_cell_restore", RichText::new(restore_label(self.restore_mode)).small(), width, |ui| {
                for mode in RESTORE_MODES {
                    ui.selectable_value(&mut self.restore_mode, mode, restore_label(mode));
                }
            })
            .response
            .on_hover_text(
                "What the backend puts onto the fresh cell once it is ready. Fast: the archived boot applications if they \
                 are of the export last deployed (then only KRL and start), else a Deploy only. Full: always a Deploy only \
                 (about 20 min more). None: an empty cell.",
            );
        }
        if state == OwnState::Off && cell.was_on_at_stop {
            let again = if resumes { "Resume virtual cell starts it again." } else { "Start virtual cell builds it again." };
            ui.small(RichText::new(format!("It was on when the workspace stopped: {again}")).color(theme::current().warn));
        }
        match virtual_cell::session_line(&cell.cost).filter(|_| state != OwnState::Off) {
            Some(line) => ui.small(RichText::new(line).weak()),
            None => ui.small(RichText::new(virtual_cell::mode_cost_hint(&cell)).weak()),
        };
        if !cell.images.is_empty() && crate::ui_mode::is_advanced(ui) {
            egui::CollapsingHeader::new(RichText::new("Images").small()).id_salt("own_cell_images").show(ui, |ui| {
                for image in &cell.images {
                    ui.small(RichText::new(virtual_cell::image_line(image)).color(Color32::GRAY)).on_hover_text(&image.ami_id);
                }
                ui.small(RichText::new(if cell.stopped { "A resume keeps the stopped instances' images; Start fresh launches these." } else { "The next fresh Start launches the latest images." }).weak());
            });
        }
        if let Some(status) = status.as_ref().filter(|s| s.available && matches!(state, OwnState::Ready | OwnState::Trouble)) {
            if virtual_cell::allows(&cell, "bringup.action") {
                self.bringup_ui(ui, status);
            } else if crate::ui_mode::is_advanced(ui) {
                ui.small(RichText::new("Its controllers bring themselves up at the start (its template has no Bring up controllers): Stop and Start the cell to repeat it.").weak());
            }
        }
        if crate::ui_mode::is_advanced(ui)
            && sidebar::button_response(ui, "Delete virtual cell…", !pending && controls.delete)
            .on_hover_text("Deletes the cell (its security group, its secret and its archives); Create virtual cell makes a new one")
            .on_disabled_hover_text("Stop the cell first")
            .clicked()
        {
            self.confirm_delete = Some((cell.id.clone(), String::new()));
        }
        if self.confirm_stop {
            let status = status.clone().unwrap_or_else(|| InfraStatus { binding: Some(CellKind::Own), ..Default::default() });
            self.stop_dialog(&ctx, &status);
        }
        if self.confirm_delete.is_some() {
            self.delete_dialog(&ctx);
        }
        if let Some(err) = &self.status_error {
            ui.label(RichText::new(format!("Can't read the virtual cell: {err}")).small().color(theme::current().danger));
        }
        self.errors(ui, &errors);
    }

    /// Bound to the shared cell: Create own virtual cell (it replaces the shared cell's access
    /// once this server is connected to it), or the own cell made already.
    fn create_own_offer(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let Some(list) = self.cells.clone().filter(|l| l.available) else { return };
        ui.separator();
        let pending = self.pending.is_some();
        if let Some(cell) = list.cells.first() {
            let state = virtual_cell::own_state(Some(cell));
            ui.small(
                RichText::new(format!(
                    "Own virtual cell {}: {}. This server still uses the shared cell until it is connected to its own (connect-cell).",
                    cell.id,
                    state.label()
                ))
                .color(Color32::GRAY),
            );
            if crate::ui_mode::is_advanced(ui)
                && sidebar::button_response(ui, "Delete own virtual cell…", !pending && virtual_cell::controls(state, false).delete)
                .on_disabled_hover_text("It is not off")
                .clicked()
            {
                self.confirm_delete = Some((cell.id.clone(), String::new()));
            }
        } else {
            let label = if self.pending == Some("create") { "Creating…" } else { "Create own virtual cell" };
            if sidebar::button_response(ui, label, !pending && list.can_create)
                .on_hover_text("Replaces the shared cell access: this workspace gets a virtual cell of its own")
                .on_disabled_hover_text(list.reason.clone().unwrap_or_else(|| "Not possible on this server".to_owned()))
                .clicked()
            {
                self.confirm_create_own = true;
            }
            ui.small(RichText::new("(replaces the shared cell access)").weak());
            if let Some(reason) = list.reason.as_ref().filter(|_| !list.can_create) {
                ui.small(RichText::new(reason).color(theme::current().warn));
            }
        }
        if self.confirm_create_own {
            self.create_own_dialog(&ctx, list.shared.as_deref().unwrap_or("plan2cell"));
        }
        if self.confirm_delete.is_some() {
            self.delete_dialog(&ctx);
        }
        self.errors(ui, &["create", "delete"]);
    }

    /// Create virtual cell's runtime (EC2 instances or EKS pods) and an EKS cell's IDE.
    fn runtime_choice(&mut self, ui: &mut egui::Ui) {
        // The long-labelled pickers one below the other, as wide as the pane; the counts in a row
        // that wraps in a narrow pane.
        let width = ui.available_width();
        theme::truncated_combo(ui, "own_cell_create_runtime", RichText::new(runtime_label(self.create_runtime)).small(), width, |ui| {
            for r in RUNTIMES {
                ui.selectable_value(&mut self.create_runtime, r, runtime_label(r));
            }
        })
        .response
        .on_hover_text(RUNTIME_HINT);
        if self.create_runtime == RUNTIME_EKS {
            theme::truncated_combo(ui, "own_cell_create_ide", RichText::new(ide_label(self.create_ide)).small(), width, |ui| {
                for i in IDES {
                    ui.selectable_value(&mut self.create_ide, i, ide_label(i));
                }
            })
            .response
            .on_hover_text(IDE_HINT);
            ui.horizontal_wrapped(|ui| {
                networks_combo(ui, "own_cell_create_networks", &mut self.create_networks);
                robots_combo(ui, "own_cell_create_robots", &mut self.create_robots);
            });
        }
    }

    /// The template Create virtual cell takes (`None`: the backend lists none, the runtime pickers).
    fn create_choice(&self) -> Option<CellTemplateInfo> {
        let list = self.cells.as_ref().filter(|_| virtual_cell::templates_offered(self.cells.as_ref()))?;
        virtual_cell::create_template(list, self.create_template.as_deref()).cloned()
    }

    /// Create virtual cell's template (a backend that lists them, instead of the runtime pickers):
    /// the picker (each template's title, an unavailable one with why), its description, and the
    /// form of its parameters.
    fn template_choice(&mut self, ui: &mut egui::Ui) {
        let Some(list) = self.cells.clone() else { return };
        let Some(chosen) = self.create_choice() else { return };
        let width = ui.available_width();
        let mut pick = chosen.id.clone();
        theme::truncated_combo(ui, "own_cell_create_template", RichText::new(template_title(&chosen)).small(), width, |ui| {
            template_items(ui, &list.templates, &mut pick);
        })
        .response
        .on_hover_text(format!("{}@{}: {}", chosen.id, chosen.version, chosen.description));
        if pick != chosen.id {
            // Its form (at its defaults) from the next frame on.
            self.create_template = Some(pick);
            ui.ctx().request_repaint();
            return;
        }
        if !chosen.description.is_empty() {
            ui.small(RichText::new(&chosen.description).weak());
        }
        if let Some(why) = virtual_cell::unavailable_text(&chosen) {
            ui.small(RichText::new(why).color(theme::current().warn));
        }
        if self.create_params.0 != chosen.id {
            self.create_params = (chosen.id.clone(), virtual_cell::template_defaults(&chosen));
        }
        for (name, p) in &chosen.parameters {
            let value = self.create_params.1.get(name).cloned().unwrap_or_else(|| p.default.clone());
            if let Some(v) = param_ui(ui, &format!("own_cell_create_{name}"), name, p, &value, Ok(())) {
                self.create_params.1.insert(name.clone(), v);
            }
        }
    }

    /// An own cell's template and its parameters (a backend that lists templates, a cell that runs
    /// one): another template only while the cell is off (not stopped), each parameter when its
    /// `mutable` allows. The form is the listed template's (a cell of a template no longer listed:
    /// the picker only).
    fn cell_template_ui(&mut self, ui: &mut egui::Ui, cell: &VirtualCell) {
        let ctx = ui.ctx().clone();
        let (Some(list), Some(current)) = (self.cells.clone(), cell.template.as_ref()) else { return };
        let pending = self.pending.is_some();
        let listed = list.templates.iter().find(|t| t.id == current.id);
        let lock = virtual_cell::template_changeable(cell);
        let mut pick = current.id.clone();
        let width = ui.available_width();
        let title = listed.map_or_else(|| current.label(), template_title);
        ui.add_enabled_ui(lock.is_ok() && !pending, |ui| {
            theme::truncated_combo(ui, "own_cell_template", RichText::new(title).small(), width, |ui| {
                template_items(ui, &list.templates, &mut pick);
            })
            .response
            .on_hover_text(
                "The template the cell boots from: what it runs, its parameters and its controls. Changed only while the cell is \
                 off (not stopped); the next Start builds it from the new one (its parameters at their defaults).",
            )
            .on_disabled_hover_text(lock.err().unwrap_or("Wait for the last change"));
        });
        if pick != current.id && lock.is_ok() && !pending {
            let body = UpdateCellRequest { template: Some(TemplateSelector::Id(pick)), ..Default::default() };
            self.update_cell(&ctx, &cell.id, "template", body);
            return;
        }
        let Some(listed) = listed else { return };
        let values = virtual_cell::cell_params(cell);
        for (name, p) in &listed.parameters {
            let lock = if pending { Err("Wait for the last change") } else { virtual_cell::param_changeable(p, cell) };
            let value = values.get(name).unwrap_or(&p.default);
            if let Some(v) = param_ui(ui, &format!("own_cell_param_{name}"), name, p, value, lock) {
                let params = serde_json::Map::from_iter([(name.clone(), v)]);
                self.update_cell(&ctx, &cell.id, "params", UpdateCellRequest { params: Some(params), ..Default::default() });
                break;
            }
        }
    }

    /// Changes the own cell's settings; `what`: the pending action (and its error line).
    fn update_cell(&mut self, ctx: &egui::Context, id: &str, what: &'static str, body: UpdateCellRequest) {
        self.pending = Some(what);
        self.action_error = None;
        let send = self.sender(ctx);
        self.api.put_json(&api::INFRA_CELL_ITEM.replace("{id}", id), &body, move |result| send(Msg::CellAction(what, result)));
    }

    fn create_cell(&mut self, ctx: &egui::Context) {
        self.pending = Some("create");
        self.action_error = None;
        let body = match self.create_choice() {
            // The template's parameters: its defaults, with what the form changed.
            Some(t) => {
                let mut params = virtual_cell::template_defaults(&t);
                if self.create_params.0 == t.id {
                    params.extend(self.create_params.1.clone().into_iter().filter(|(name, _)| t.parameters.contains_key(name)));
                }
                CreateCellRequest { template: Some(TemplateSelector::Id(t.id)), params: Some(params), start: self.create_start, ..Default::default() }
            }
            None => {
                let eks = self.create_runtime == RUNTIME_EKS;
                CreateCellRequest {
                    start: self.create_start,
                    runtime: eks.then(|| RUNTIME_EKS.to_owned()),
                    ide: eks.then(|| self.create_ide.to_owned()),
                    networks: eks.then(|| networks_of(self.create_networks, &[], self.create_robots)),
                    ..Default::default()
                }
            }
        };
        let send = self.sender(ctx);
        self.api.post_json(api::INFRA_CELLS, &body, move |result| send(Msg::CellAction("create", result)));
    }

    fn create_own_dialog(&mut self, ctx: &egui::Context, shared: &str) {
        let mut go = false;
        let modal = egui::Modal::new(egui::Id::new("own_cell_create_confirm")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(420.0);
            ui.heading("Create your own virtual cell?");
            ui.label(format!(
                "This workspace uses the shared cell {shared} now. Its own cell replaces that access: once this server is \
                 connected to it, Start / Stop, Deploy, the PLC IDE and production act on the own cell only."
            ));
            let template = self.create_choice();
            ui.label(template.as_ref().map_or_else(|| virtual_cell::cost_hint(None), virtual_cell::template_cost_hint));
            ui.checkbox(&mut self.create_start, "then start it");
            if template.is_some() {
                self.template_choice(ui);
            } else if self.cells.as_ref().is_some_and(|l| l.runtimes.iter().any(|r| r == RUNTIME_EKS)) {
                self.runtime_choice(ui);
            }
            let unavailable = template.as_ref().and_then(virtual_cell::unavailable_text);
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(unavailable.is_none(), egui::Button::new(RichText::new("Create own virtual cell").color(theme::current().warn)))
                    .on_disabled_hover_text(unavailable.unwrap_or_default())
                    .clicked()
                {
                    go = true;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_create_own = false;
                }
            });
        });
        if modal.should_close() {
            self.confirm_create_own = false;
        }
        if go {
            self.confirm_create_own = false;
            self.create_cell(ctx);
        }
    }

    /// Delete…: the cell id has to be typed.
    fn delete_dialog(&mut self, ctx: &egui::Context) {
        let Some((id, mut typed)) = self.confirm_delete.clone() else { return };
        let mut go = false;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("own_cell_delete_confirm")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(420.0);
            ui.heading("Delete the virtual cell?");
            ui.label(format!(
                "Deletes {id}: its security group, its secret and the archived boot applications and PLC IDE projects (they \
                 expire within 14 days). The control code exports stay. Create virtual cell makes a new, empty one."
            ));
            ui.label(format!("Type {id} to confirm:"));
            let edit = ui.add(egui::TextEdit::singleline(&mut typed).hint_text(id.as_str()).desired_width(f32::INFINITY));
            let ok = virtual_cell::delete_confirmed(&typed, &id);
            if ok && edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                go = true;
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.add_enabled(ok, egui::Button::new(RichText::new("Delete virtual cell").color(theme::current().danger))).clicked() {
                    go = true;
                }
                if ui.button("Cancel").clicked() {
                    cancel = true;
                }
            });
        });
        if modal.should_close() || cancel {
            self.confirm_delete = None;
            return;
        }
        if go {
            self.confirm_delete = None;
            self.pending = Some("delete");
            self.action_error = None;
            let body = DeleteCellRequest { confirm: typed.trim().to_owned() };
            let send = self.sender(ctx);
            self.api.delete_json(&api::INFRA_CELL_ITEM.replace("{id}", &id), &body, move |result| send(Msg::CellAction("delete", result)));
        } else {
            self.confirm_delete = Some((id, typed));
        }
    }

    fn stop_dialog(&mut self, ctx: &egui::Context, status: &InfraStatus) {
        let own = status.binding == Some(CellKind::Own);
        let running: Vec<String> = status
            .instances
            .iter()
            .filter(|i| i.state != "stopped")
            .map(|i| format!("{} ({})", role_label(&i.role), i.instance_id))
            .collect();
        let mut stop = false;
        let modal = egui::Modal::new(egui::Id::new("infra_stop_confirm")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(380.0);
            ui.heading("Stop the virtual cell?");
            ui.colored_label(
                theme::current().warn,
                "This shuts down the machines: the virtual PLCs (head PLC), the Windows CODESYS IDE and the KUKA \
                 controllers stop running. To stop only the robots' motion, use Stop production instead.",
            );
            let stop_mode = status.virtual_cell.as_ref().is_some_and(|c| c.stop_mode());
            if own && stop_mode {
                if !running.is_empty() {
                    ui.label(format!("Stops {} (an EC2 stop).", running.join(", ")));
                }
                ui.label(format!(
                    "This cell keeps its instances: Stop archives the boot applications and the PLC IDE's projects, then stops the \
                     instances. Their disks stay (~${:.2}/day). Start virtual cell resumes them with the deployed code ({USUAL_RESUME} \
                     to ready); Start fresh launches new ones from the latest images.",
                    virtual_cell::stopped_usd_per_day(status.virtual_cell.as_ref().map(|c| &c.cost))
                ));
            } else if own && !virtual_cell::status_allows(Some(status), "archive") {
                if !running.is_empty() {
                    ui.label(format!("Terminates {}.", running.join(", ")));
                }
                ui.label(
                    "Your own cell is rebuilt on every start: Stop removes its instances or pods and archives nothing (its \
                     template has no archive; nothing is paid while it is off). Start virtual cell builds it fresh; deploy the \
                     control code again then. Retain values, logs and anything changed by hand on it are gone.",
                );
            } else if own {
                if !running.is_empty() {
                    ui.label(format!("Terminates {}.", running.join(", ")));
                }
                ui.label(
                    "Your own cell is rebuilt on every start: Stop archives the boot applications and the PLC IDE's projects, \
                     then terminates the instances (nothing is paid while it is off). Start virtual cell launches them fresh \
                     from the latest images and restores the last deployed control code (usually 6–8 min). Retain values, \
                     logs and anything changed by hand on the instances are gone.",
                );
            } else {
                ui.label(format!("Stops {} (an EC2 stop).", running.join(", ")));
                ui.label(
                    "Nothing is deleted or terminated: disks, configuration, the deployed code and the KUKA controllers stay. Start \
                     virtual cell resumes the cell in about 5 min (and, with \"then start production\", brings it back into \
                     production).",
                );
            }
            let producing = status.in_production || status.cell_power.as_ref().is_some_and(|p| p.production.is_some());
            if producing {
                let by = status.cell_power.as_ref().and_then(|p| p.production.as_ref()).and_then(|p| p.by.clone());
                ui.colored_label(
                    theme::current().danger,
                    format!(
                        "⚠ The cell is in production now{}: production stops, and the next Start virtual cell starts it again.",
                        by.map(|b| format!(" ({b})")).unwrap_or_default()
                    ),
                );
            }
            if status.busy() {
                ui.colored_label(theme::current().warn, "A bring-up or production start is running; it will fail.");
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let label = if producing { "Stop production and the virtual cell" } else { "Stop virtual cell" };
                if ui.button(RichText::new(label).color(theme::current().danger)).clicked() {
                    stop = true;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_stop = false;
                }
            });
        });
        if modal.should_close() {
            self.confirm_stop = false;
        }
        if stop {
            self.confirm_stop = false;
            if status.cell_power.is_some() || own {
                // Confirmed: in production (or during a job) too.
                let force = status.in_production || status.busy() || status.cell_power.as_ref().is_some_and(|p| p.production.is_some());
                let body = CellPowerRequest { action: "off".into(), force, ..Default::default() };
                self.act(ctx, "power-off", api::INFRA_CELL_POWER, serde_json::to_value(body).unwrap_or_default());
            } else {
                let body = serde_json::to_value(StartStopRequest::default()).unwrap_or_default();
                self.act(ctx, "stop", api::INFRA_STOP, body);
            }
        }
    }
}

const RUNTIME_HINT: &str = "Where the cell runs. EC2: the PLC host, the Windows IDE and a KUKA VC host per robot as EC2 instances (the \
default). EKS: the same cell as pods on the factoryos cells EKS cluster: the CODESYS runtime and the KUKA VCs as Linux \
containers on a PROFINET overlay network across the nodes (VXLAN), the IDE as a Windows container or the EC2 IDE. Changed only while \
the cell is off; the next Start builds it in the new runtime.";
const IDE_HINT: &str = "An EKS cell always has a compile node: CODESYS and its MCP server headless in a Windows pod, which Deploy and \
Run Control Code use for every network. Compile node + EC2 IDE: also the EC2 Windows IDE with DCV (Open PLC IDE), reaching each \
network's head PLC over TCP.";

/// The PROFINET networks of an EKS cell: 1 to [`MAX_NETWORKS`] (each a cell of its own on the shared cluster).
fn networks_combo(ui: &mut egui::Ui, id: &str, count: &mut usize) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(RichText::new(format!("Networks: {count}")).small())
        .show_ui(ui, |ui| {
            for n in 1..=MAX_NETWORKS {
                ui.selectable_value(count, n, format!("{n} PROFINET network{}", if n == 1 { "" } else { "s" }));
            }
        })
        .response
        .on_hover_text(
            "PROFINET networks of this EKS cell: each is a cell of its own (head PLC, its KUKA VCs, its own L2 segment and subnet) \
             on the shared cluster. More networks: more pods and nodes while on.",
        );
}

/// The KUKA VCs of one EKS network: 1 to [`MAX_NETWORK_ROBOTS`] (one pod each, 5 pinned cores).
fn robots_combo(ui: &mut egui::Ui, id: &str, robots: &mut u8) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(RichText::new(format!("{robots} robot{}", if *robots == 1 { "" } else { "s" })).small())
        .show_ui(ui, |ui| {
            for n in 1..=MAX_NETWORK_ROBOTS {
                ui.selectable_value(robots, n, format!("{n} robot{}", if n == 1 { "" } else { "s" }));
            }
        })
        .response
        .on_hover_text("KUKA VCs of this network: one pod each (5 pinned cores), up to 6 on one node.");
}

/// `count` networks n1..: the ones there keep their robots, new ones get `robots`.
fn networks_of(count: usize, have: &[CellNetwork], robots: u8) -> Vec<CellNetwork> {
    (1..=count)
        .map(|i| {
            let name = format!("n{i}");
            let robots = have.iter().find(|n| n.name == name).map_or(robots, |n| n.robots);
            CellNetwork { name, robots, ..Default::default() }
        })
        .collect()
}

fn runtime_label(r: &str) -> &'static str {
    if r == RUNTIME_EKS { "Runtime: EKS pods" } else { "Runtime: EC2 instances" }
}

fn ide_label(i: &str) -> &'static str {
    if i == IDE_EC2 { "IDE: compile node + EC2 IDE (DCV)" } else { "IDE: compile node only" }
}

/// A template in the picker: its title (else its id).
fn template_title(t: &CellTemplateInfo) -> String {
    if t.title.is_empty() { t.id.clone() } else { t.title.clone() }
}

/// The picker's entries: every listed template, an unavailable one disabled with why.
fn template_items(ui: &mut egui::Ui, templates: &[CellTemplateInfo], pick: &mut String) {
    for t in templates {
        let why = virtual_cell::unavailable_text(t);
        let text = match &why {
            Some(why) => format!("{} ({why})", template_title(t)),
            None => template_title(t),
        };
        let about = format!("{}@{}: {}\n{}", t.id, t.version, t.description, virtual_cell::template_cost_hint(t));
        ui.add_enabled_ui(why.is_none(), |ui| ui.selectable_value(pick, t.id.clone(), text))
            .inner
            .on_hover_text(about.as_str())
            .on_disabled_hover_text(about.as_str());
    }
}

/// When a template parameter may change, for its hover.
fn mutable_hint(p: &TemplateParameter) -> &'static str {
    match p.mutable.as_str() {
        "always" => "Changes any time",
        "never" => "Set when the cell is created",
        _ => "Changes only while the cell is off; the next Start builds the cell with it",
    }
}

/// An enum value in its picker: the legacy templates' Stop and IDE as the runtime pickers word
/// them, anything else `<label>: <value>`.
fn enum_text(name: &str, label: &str, value: &str) -> String {
    match (name, value) {
        ("offMode", OFF_MODE_TERMINATE | OFF_MODE_STOP) => virtual_cell::off_mode_label(value).to_owned(),
        ("ide", IDE_CONTAINER | IDE_EC2) => ide_label(value).to_owned(),
        _ => format!("{label}: {value}"),
    }
}

/// One template parameter's control (`id`: unique per form): an integer or enum picker, a
/// checkbox, the networks editor; an enum of one value (nothing to choose) or a type this viewer
/// does not know: none. `lock`: why it cannot change now (drawn disabled). `Some`: the value picked.
fn param_ui(
    ui: &mut egui::Ui,
    id: &str,
    name: &str,
    p: &TemplateParameter,
    value: &serde_json::Value,
    lock: Result<(), &str>,
) -> Option<serde_json::Value> {
    let label = if p.label.is_empty() { name } else { p.label.as_str() };
    let why = lock.err().unwrap_or_default();
    let hint = mutable_hint(p);
    let width = ui.available_width();
    ui.add_enabled_ui(lock.is_ok(), |ui| match p.kind.as_str() {
        "integer" => {
            let min = p.min.unwrap_or(0);
            let max = p.max.unwrap_or(min).max(min);
            let mut v = value.as_i64().unwrap_or(min);
            let before = v;
            int_combo(ui, id, label, &mut v, min, max).on_hover_text(hint).on_disabled_hover_text(why);
            (v != before).then(|| serde_json::json!(v))
        }
        "enum" if p.values.len() > 1 => {
            let mut v = value.as_str().unwrap_or_default().to_owned();
            let before = v.clone();
            theme::truncated_combo(ui, id, RichText::new(enum_text(name, label, &v)).small(), width, |ui| {
                for value in &p.values {
                    ui.selectable_value(&mut v, value.clone(), enum_text(name, label, value));
                }
            })
            .response
            .on_hover_text(hint)
            .on_disabled_hover_text(why);
            (v != before).then(|| serde_json::Value::String(v))
        }
        "boolean" => {
            let mut v = value.as_bool().unwrap_or_default();
            let before = v;
            ui.checkbox(&mut v, RichText::new(label).small()).on_hover_text(hint).on_disabled_hover_text(why);
            (v != before).then_some(serde_json::Value::Bool(v))
        }
        "networks" => networks_ui(ui, id, label, p, value, hint, why),
        _ => None,
    })
    .inner
}

/// An integer picker `<label>: <n>` over `min..=max` (at most 64 entries; the kinds' caps are far below).
fn int_combo(ui: &mut egui::Ui, id: &str, label: &str, value: &mut i64, min: i64, max: i64) -> egui::Response {
    egui::ComboBox::from_id_salt(id)
        .selected_text(RichText::new(format!("{label}: {value}")).small())
        .show_ui(ui, |ui| {
            for n in min..=max.min(min.saturating_add(63)) {
                ui.selectable_value(value, n, format!("{label}: {n}"));
            }
        })
        .response
}

/// A `networks` parameter: how many (n1.., `min..=max`), and each one's robots (`robots.min..=max`;
/// a new one gets `robots.default`). `Some`: the networks picked.
fn networks_ui(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    p: &TemplateParameter,
    value: &serde_json::Value,
    hint: &str,
    why: &str,
) -> Option<serde_json::Value> {
    let have = virtual_cell::networks_value(value);
    let robots = p.robots.clone().unwrap_or_default();
    let (rmin, rdefault) = (i64::from(robots.min.max(1)), i64::from(robots.default.max(1)));
    let rmax = i64::from(robots.max).max(rmin);
    let min = p.min.unwrap_or(1).max(1);
    let max = p.max.unwrap_or(min).max(min);
    let mut count = (have.len() as i64).max(min);
    let before = count;
    int_combo(ui, &format!("{id}_count"), label, &mut count, min, max).on_hover_text(hint).on_disabled_hover_text(why);
    if count != before {
        return Some(virtual_cell::networks_json(count as usize, &have, rdefault));
    }
    let mut changed = None;
    for (i, (net, r)) in have.iter().enumerate() {
        let mut v = *r;
        ui.horizontal_wrapped(|ui| {
            ui.small(RichText::new(format!("{net}:")).weak());
            int_combo(ui, &format!("{id}_{net}"), "robots", &mut v, rmin, rmax).on_hover_text(hint).on_disabled_hover_text(why);
        });
        if v != *r && changed.is_none() {
            let mut nets = have.clone();
            nets[i].1 = v;
            changed = Some(virtual_cell::networks_json(nets.len(), &nets, rdefault));
        }
    }
    changed
}

/// The Open PLC IDE link of `session`, pinned to its backend's workspace: natively, behind the
/// edge, the base is the bare origin and the workspace a cookie the system browser may not have
/// (or have for another workspace); a token is redeemed only through the gateway of the backend
/// that made it.
fn dcv_link(base: &str, session: &DcvSession) -> String {
    factoryos_client::protocol::workspace::pinned_url(base, session.workspace.as_deref(), &session.url)
}

/// `Cell in use by <holder> (<purpose>, since hh:mm UTC)` while another backend holds the cell.
fn lease_line(status: &InfraStatus, now: f64) -> Option<String> {
    status.lease.as_ref().filter(|l| l.held_by_other(now)).and_then(|l| l.describe())
}

/// An operator's line instead of the auto-restart toggle.
fn supervisor_line(status: &InfraStatus) -> String {
    format!("Auto-restart runs on {}", status.cell_supervisor.as_deref().unwrap_or("the cell's supervisor backend"))
}

/// Why no job can use the cell now although it is not stopped: an instance between states, or
/// Start cell still bringing it up (`None`: neither).
fn wait_reason(status: &InfraStatus) -> Option<String> {
    if let Some(why) = virtual_cell::blocks(status) {
        return Some(why);
    }
    let changing: Vec<String> = status.instances.iter().filter(|i| i.changing()).map(|i| format!("{} {}", role_label(&i.role), i.state)).collect();
    if !changing.is_empty() {
        return Some(format!("Wait: the instances are changing state ({})", changing.join(", ")));
    }
    if status.start.state.busy() {
        return Some("Wait: Start cell is bringing the cell up".to_owned());
    }
    match status.transition.as_deref() {
        Some("stop") => Some("Wait: the cell is stopping".to_owned()),
        Some(_) => Some("Wait: the cell is starting".to_owned()),
        None => None,
    }
}

/// A Deploy / Run Control Code waiting for its confirmation.
#[derive(Clone, Debug)]
struct ConfirmDeploy {
    build: String,
    plan: Option<String>,
    /// Run Control Code (deploy and start), else Deploy only.
    start: bool,
    /// Deploy anyway: without the deployability check.
    force: bool,
}

/// The export a deploy takes: step 2's choice, else the backend's newest (build, PLAN_ID, picked).
fn deploy_target(status: &InfraStatus, choice: Option<DeployChoice>) -> Option<(String, Option<String>, bool)> {
    choice
        .map(|c| (c.build, c.plan_id, c.picked))
        .or_else(|| status.newest_export.as_ref().map(|e| (e.build.clone(), e.plan_id.clone(), false)))
}

/// An action's name in its error line.
fn action_label(what: &str) -> &str {
    match what {
        "start" => "Start cell",
        "stop" => "Stop instances",
        "power-on" => "Start virtual cell",
        "power-fresh" => "Start fresh",
        "power-off" => "Stop virtual cell",
        "off-mode" => "Off mode",
        "runtime" => "Runtime",
        "template" => "Template",
        "params" => "Parameters",
        "create" => "Create virtual cell",
        "delete" => "Delete virtual cell",
        "bring-up" => "Bring up controllers",
        "deploy" => "Deploy",
        "run" => "Run Control Code",
        "production" => "Start production",
        "restart" => "Restart PLC runtime",
        "stop-production" => "Stop production",
        other => other,
    }
}

/// Start / Stop virtual cell enabled: by the cell's power (with it), else by the instances.
fn power_buttons(status: &InfraStatus) -> (bool, bool) {
    let any_stopped = status.instances.iter().any(|i| i.state == "stopped");
    let any_running = status.instances.iter().any(|i| i.state == "running" || i.state == "pending");
    match &status.cell_power {
        // Start also brings up a cell that runs but is not ready (or failed); not while on its way or stopping.
        Some(p) => (
            matches!(p.state, PowerState::Off | PowerState::Degraded | PowerState::Partial | PowerState::Failed | PowerState::Unknown),
            any_running && p.state != PowerState::Stopping,
        ),
        None => (any_stopped, any_running),
    }
}

/// `hh:mm UTC` of an ISO 8601 time (`2026-10-15T08:01:40.000Z`).
fn iso_hhmm(iso: &str) -> Option<String> {
    let time = iso.split('T').nth(1)?;
    (time.len() >= 5 && time.as_bytes()[2] == b':').then(|| format!("{} UTC", &time[..5]))
}

/// The status dot's colour of the own cell.
fn own_color(state: OwnState) -> Color32 {
    match state {
        OwnState::Ready => theme::current().ok,
        OwnState::Starting(_) | OwnState::Stopping(_) | OwnState::Trouble | OwnState::Deleting => theme::current().warn,
        OwnState::Failed => theme::current().danger,
        OwnState::None | OwnState::Off | OwnState::Unknown => Color32::GRAY,
    }
}

/// [`RESTORE_MODES`] in words.
fn restore_label(mode: &str) -> &str {
    match mode {
        "auto" => "Restore the last control code (fast if possible)",
        "full" => "Restore the last control code by a Deploy",
        "none" => "Don't restore any control code",
        other => other,
    }
}

/// The status dot's colour of the cell's power.
fn power_color(state: PowerState) -> Color32 {
    match state {
        PowerState::Ready => theme::current().ok,
        PowerState::Booting | PowerState::Stopping | PowerState::Degraded | PowerState::Partial => theme::current().warn,
        PowerState::Failed => theme::current().danger,
        PowerState::Off | PowerState::Unknown => Color32::GRAY,
    }
}

/// `Virtual cell: ● booting (bring-up) since 08:01 UTC`, what it does or why it is not ready, the
/// checks that fail, and the idle stop.
fn power_ui(ui: &mut egui::Ui, power: &CellPower) {
    let color = power_color(power.state);
    let line = power.line();
    let state = line.trim_start_matches("Virtual cell: ").to_owned();
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("Virtual cell:").strong());
        ui.label(RichText::new("●").color(color));
        ui.label(RichText::new(state).color(color).strong());
        if matches!(power.state, PowerState::Booting | PowerState::Stopping) {
            ui.spinner();
        }
        if let Some(since) = power.since.as_deref().and_then(iso_hhmm) {
            ui.label(RichText::new(format!("since {since}")).small().weak());
        }
    });
    power_details(ui, power, true);
}

/// What the power does or why it is not ready, the readiness checks, the idle stop; `shared`: the
/// shared cell (its instances need the `Plan2cellCell` tag).
fn power_details(ui: &mut egui::Ui, power: &CellPower, shared: bool) {
    if let Some(message) = power.message.as_deref().filter(|m| !m.is_empty()) {
        let color = if power.state == PowerState::Failed { theme::current().danger } else { Color32::GRAY };
        ui.small(RichText::new(message).color(color));
    }
    if matches!(power.state, PowerState::Ready | PowerState::Degraded | PowerState::Failed | PowerState::Booting) && !power.checks.is_empty() {
        let failing: Vec<_> = power.failing().filter(|c| c.ok == Some(false)).collect();
        let title = if failing.is_empty() {
            format!("Readiness: {} of {} checks passed", power.checks.iter().filter(|c| c.ok == Some(true)).count(), power.checks.len())
        } else {
            format!("Readiness: {} failing", failing.len())
        };
        egui::CollapsingHeader::new(RichText::new(title).small()).id_salt("infra_power_checks").show(ui, |ui| {
            for c in &power.checks {
                let (mark, color) = match c.ok {
                    Some(true) => ("ok", theme::current().ok),
                    Some(false) => ("failed", theme::current().danger),
                    None => ("not read", Color32::GRAY),
                };
                let text = match &c.detail {
                    Some(d) => format!("{}: {mark} ({d})", c.label),
                    None => format!("{}: {mark}", c.label),
                };
                ui.small(RichText::new(text).color(color));
            }
        });
    }
    if let Some(idle) = &power.idle {
        let last = idle.last_activity_at.as_deref().and_then(iso_hhmm).map(|t| match &idle.last_activity_by {
            Some(by) => format!(" (last use: {by}, {t})"),
            None => format!(" (last use {t})"),
        });
        ui.small(RichText::new(format!("Stops by itself after {} h without use{}", idle.hours, last.unwrap_or_default())).weak());
    }
    if shared && power.hosts.iter().any(|h| !h.tagged) {
        ui.small(RichText::new("Some instances lack the Plan2cellCell tag: the cell control Lambda cannot start or stop them (deploy Plan2cellCellControl).").color(theme::current().warn));
    }
}

/// `● PLC host  running · SSM Online` (`● KUKA VC host 1 · robot1 …`), coloured by how usable it is.
fn chip(ui: &mut egui::Ui, role: &str, instance: Option<&InfraInstance>) {
    let (text, color) = match instance {
        None => ("unknown".to_owned(), Color32::GRAY),
        Some(i) => {
            let color = match (i.state.as_str(), i.online()) {
                ("running", true) => theme::current().ok,
                ("running" | "pending" | "stopping", _) => theme::current().warn,
                ("stopped", _) => Color32::GRAY,
                _ => theme::current().danger,
            };
            let ssm = if i.state == "running" { format!(" · SSM {}", i.ssm_ping) } else { String::new() };
            (format!("{}{ssm}", i.state), color)
        }
    };
    let response = ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("●").color(color));
        let label = match instance.and_then(|i| i.robot_ns.as_deref()) {
            Some(ns) => format!("{} · {ns}", role_label(role)),
            None => role_label(role),
        };
        ui.label(RichText::new(label).small().strong());
        ui.label(RichText::new(text).small().color(color));
    });
    if let Some(i) = instance {
        let mut hover = i.instance_id.clone();
        if let Some(name) = &i.name {
            hover = format!("{name} ({hover})");
        }
        if let Some(ip) = &i.private_ip {
            hover.push_str(&format!("\nprivate {ip}"));
        }
        response.response.on_hover_text(hover);
    }
}

/// Unix seconds now.
fn now_s() -> f64 {
    SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

/// `CODESYS demo runtime: stops at 14:02 UTC (in 37 min)`, `… stopped`, or `… unknown` when the
/// last read failed (the rest is then an older read's: not shown as current).
fn runtime_text(rt: &PlcRuntime, now: f64) -> (String, bool) {
    if rt.error.is_some() {
        return ("CODESYS demo runtime: unknown (not read)".to_owned(), true);
    }
    let left = rt.stops_s.map(|end| end - now);
    match left {
        _ if !rt.running => ("CODESYS demo runtime: stopped".to_owned(), true),
        Some(left) if left > 0.0 => (
            format!("CODESYS demo runtime: stops at {} (in {} min)", utc_hhmm(rt.stops_s.unwrap_or_default()), (left / 60.0).ceil() as u64),
            left < DEMO_WARN_S,
        ),
        Some(_) => (format!("CODESYS demo runtime: stopped at {}", utc_hhmm(rt.stops_s.unwrap_or_default())), true),
        None => ("CODESYS runtime: running (no demo stop in its log)".to_owned(), false),
    }
}

fn runtime_line(ui: &mut egui::Ui, rt: &PlcRuntime, now: f64) {
    let (text, warn) = runtime_text(rt, now);
    let mut hover = String::new();
    if let Some(c) = &rt.container {
        hover.push_str(&format!("first to stop: {c}"));
    }
    if let Some(source) = &rt.source {
        hover.push_str(&format!(" (from its {source})"));
    }
    if let Some(w) = &rt.warning {
        hover.push_str(&format!("\n{w}"));
    }
    for r in &rt.runtimes {
        let at = r.stops_s.map(|t| format!(", stops {}", utc_hhmm(t))).unwrap_or_default();
        hover.push_str(&format!("\n{}{}: {}{at}", r.container, if r.head { " (head)" } else { "" }, if r.running { "running" } else { "not running" }));
    }
    if rt.checked_s > 0.0 {
        hover.push_str(&format!("\nread {}", utc_hhmm(rt.checked_s)));
    }
    let color = if warn { theme::current().warn } else { Color32::GRAY };
    let response = ui.small(RichText::new(text).color(color));
    if !hover.is_empty() {
        response.on_hover_text(hover.trim_start());
    }
    if let Some(err) = &rt.error {
        ui.small(RichText::new(err).color(theme::current().warn));
    }
}

/// `projects ✓ · deploy … · krl · start`: the stages of Run Control Code.
fn stages_text(d: &DeployStatus) -> String {
    d.stages
        .iter()
        .map(|stage| {
            let mark = if d.done.contains(stage) {
                " ✓"
            } else if d.stage.as_deref() == Some(stage.as_str()) {
                match d.state {
                    JobState::Failed => " ✗",
                    JobState::Waiting | JobState::Running => " …",
                    _ => "",
                }
            } else {
                ""
            };
            format!("{stage}{mark}")
        })
        .collect::<Vec<_>>()
        .join(" · ")
}

/// A job's state as one coloured line.
fn job_line(ui: &mut egui::Ui, state: JobState, message: Option<&str>, extra: Option<&str>) {
    let (mark, color) = match state {
        JobState::Idle => return,
        JobState::Waiting | JobState::Running => ("…", theme::current().warn),
        JobState::Done => ("✓", theme::current().ok),
        JobState::Failed => ("✗", theme::current().danger),
    };
    let extra = extra.map(|e| format!(" ({e})")).unwrap_or_default();
    ui.label(RichText::new(format!("{mark} {}{extra}", message.unwrap_or_default())).small().color(color));
}

/// The message of an API error (`409 Conflict: <message>` → `<message>`).
fn body_of(err: &str) -> &str {
    match err.split_once(": ") {
        Some((status, body)) if status.len() >= 3 && status.as_bytes()[..3].iter().all(u8::is_ascii_digit) => body,
        _ => err,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plc_ide_link_names_its_workspace() {
        let session = |url: &str, workspace: Option<&str>| DcvSession {
            url: url.into(),
            workspace: workspace.map(str::to_owned),
            target: "ide".into(),
            ..Default::default()
        };
        let jd = session("/dcv/ide/?authToken=jdoe.ab#console", Some("jdoe"));
        // A native viewer behind the edge: its base is the origin.
        assert_eq!(dcv_link("https://d1.cloudfront.net", &jd), "https://d1.cloudfront.net/workspace/jdoe/dcv/ide/?authToken=jdoe.ab#console");
        // The test server directly: its own prefix (its backend strips it).
        let test = session("/dcv/ide/?authToken=plan2cell-test.cd#console", Some("plan2cell-test"));
        assert_eq!(dcv_link("http://198.51.100.7", &test), "http://198.51.100.7/workspace/plan2cell-test/dcv/ide/?authToken=plan2cell-test.cd#console");
        // The web viewer under a prefix, and an older server (no workspace, `/dcv/`): as they are.
        assert_eq!(dcv_link("https://h/workspace/jdoe", &jd), "https://h/workspace/jdoe/dcv/ide/?authToken=jdoe.ab#console");
        assert_eq!(dcv_link("http://127.0.0.1:8080", &session("/dcv/?authToken=t#console", None)), "http://127.0.0.1:8080/dcv/?authToken=t#console");
    }

    #[test]
    fn another_backend_on_the_cell() {
        use factoryos_client::protocol::infra::CellLease;
        let mut status = InfraStatus::default();
        assert_eq!(lease_line(&status, 0.0), None);
        status.lease = Some(CellLease { holder: Some("plan2cell-test".into()), purpose: Some("production".into()), since_s: Some(29100.0), ..Default::default() });
        assert_eq!(lease_line(&status, 0.0).as_deref(), Some("Cell in use by plan2cell-test (production, since 08:05 UTC)"));
        status.lease.as_mut().unwrap().mine = true;
        assert_eq!(lease_line(&status, 0.0), None, "its own");
        assert_eq!(supervisor_line(&status), "Auto-restart runs on the cell's supervisor backend");
        status.cell_supervisor = Some("plan2cell-test".into());
        assert_eq!(supervisor_line(&status), "Auto-restart runs on plan2cell-test");
    }

    #[test]
    fn jobs_wait_while_instances_change_state() {
        let instance = |role: &str, state: &str| InfraInstance { role: role.into(), state: state.into(), ..Default::default() };
        let mut status = InfraStatus { instances: vec![instance("plcHost", "running"), instance("windowsIde", "stopping")], ..Default::default() };
        assert_eq!(wait_reason(&status).as_deref(), Some("Wait: the instances are changing state (Windows IDE stopping)"));
        status.instances[1].state = "running".into();
        assert_eq!(wait_reason(&status), None);
        status.start.state = JobState::Waiting;
        assert!(wait_reason(&status).unwrap().contains("Start cell"));
    }

    #[test]
    fn the_cells_power_gates_the_buttons_and_the_jobs() {
        let instance = |state: &str| InfraInstance { role: "plcHost".into(), state: state.into(), ..Default::default() };
        let power = |state: PowerState, phase: &str| Some(CellPower { state, phase: phase.into(), ..Default::default() });
        // An older server (no power): by the instances, as before.
        let mut status = InfraStatus { instances: vec![instance("stopped")], ..Default::default() };
        assert_eq!(power_buttons(&status), (true, false));
        assert_eq!(wait_reason(&status), None);
        // Off: Start only; every job waits with the hint.
        status.cell_power = power(PowerState::Off, "off");
        assert_eq!(power_buttons(&status), (true, false));
        assert!(wait_reason(&status).unwrap().contains("Start virtual cell first"));
        // Booting: neither (Stop is still possible while instances run).
        status.instances = vec![instance("running")];
        status.cell_power = power(PowerState::Booting, "bring-up");
        assert_eq!(power_buttons(&status), (false, true));
        assert_eq!(wait_reason(&status).as_deref(), Some("The virtual cell is booting (bring-up): wait until it is ready"));
        // Ready: Stop; nothing in the way of a job.
        status.cell_power = power(PowerState::Ready, "ready");
        assert_eq!(power_buttons(&status), (false, true));
        assert_eq!(wait_reason(&status), None);
        // Running but not ready (or failed): Start brings it up again.
        status.cell_power = power(PowerState::Degraded, "not-ready");
        assert_eq!(power_buttons(&status), (true, true));
        status.cell_power = power(PowerState::Stopping, "stopping");
        assert_eq!(power_buttons(&status), (false, false));
        assert!(starts_production("power-on", true) && !starts_production("power-on", false));
        assert_eq!(action_label("power-off"), "Stop virtual cell");
        assert_eq!(iso_hhmm("2026-10-15T08:01:40.000Z").as_deref(), Some("08:01 UTC"));
        assert_eq!(iso_hhmm("soon"), None);
        assert_eq!(power_color(PowerState::Ready), theme::current().ok);
        assert_eq!(power_color(PowerState::Failed), theme::current().danger);
    }

    #[test]
    fn a_production_start_is_watched_until_its_job_ends() {
        let mut seen = false;
        let idle = InfraStatus::default();
        assert_eq!(watch_step(&mut seen, true, Some(&idle)), WatchStep::Wait, "the reply is still out");
        assert_eq!(watch_step(&mut seen, false, Some(&idle)), WatchStep::Wait, "its job has not shown up yet");
        assert!(!seen);
        let mut running = InfraStatus::default();
        running.cell.state = JobState::Running;
        assert_eq!(watch_step(&mut seen, false, Some(&running)), WatchStep::Wait);
        assert!(seen);
        let producing = InfraStatus { in_production: true, ..Default::default() };
        assert_eq!(watch_step(&mut seen, false, Some(&producing)), WatchStep::Reached);
        assert_eq!(watch_step(&mut seen, false, Some(&idle)), WatchStep::Ended, "ended without production");

        // Restart PLC runtime: in production before, during and after.
        let mut seen = false;
        let restarting = InfraStatus { in_production: true, cell: running.cell.clone(), ..Default::default() };
        assert_eq!(watch_step(&mut seen, false, Some(&producing)), WatchStep::Wait, "not the job's outcome yet");
        assert_eq!(watch_step(&mut seen, false, Some(&restarting)), WatchStep::Wait);
        assert_eq!(watch_step(&mut seen, false, Some(&producing)), WatchStep::Reached);

        // Start cell's chain: the instances start first.
        let mut seen = false;
        let starting = InfraStatus { transition: Some("start".into()), ..Default::default() };
        assert_eq!(watch_step(&mut seen, false, Some(&starting)), WatchStep::Wait);
        assert!(seen);
    }

    #[test]
    fn what_brings_the_cell_into_production() {
        for what in ["production", "restart", "run"] {
            assert!(starts_production(what, false));
        }
        assert!(starts_production("start", true));
        assert!(!starts_production("start", false), "Start cell without its chain");
        for what in ["stop", "bring-up", "deploy", "stop-production"] {
            assert!(!starts_production(what, true));
        }
    }

    #[test]
    fn a_templates_form_words_its_values() {
        // The legacy templates' Stop and IDE as the runtime pickers had them; anything else by its label.
        assert_eq!(enum_text("offMode", "Stop", "stop"), "Stop keeps the instances (resume)");
        assert_eq!(enum_text("ide", "IDE", "ec2"), "IDE: compile node + EC2 IDE (DCV)");
        assert_eq!(enum_text("ide", "IDE", "web"), "IDE: web");
        assert_eq!(enum_text("runtime", "Runtime", "ec2"), "Runtime: ec2");
        let t = CellTemplateInfo { id: "m1-probe".into(), ..Default::default() };
        assert_eq!(template_title(&t), "m1-probe");
        assert_eq!(template_title(&CellTemplateInfo { title: "CODESYS on EC2".into(), ..t }), "CODESYS on EC2");
        assert_eq!(mutable_hint(&TemplateParameter { mutable: "always".into(), ..Default::default() }), "Changes any time");
        assert_eq!(action_label("params"), "Parameters");
    }

    #[test]
    fn errors_lose_their_status_line() {
        assert_eq!(body_of("409 Conflict: Start cell first"), "Start cell first");
        assert_eq!(body_of("network down"), "network down");
    }

    #[test]
    fn the_stages_of_run_control_code() {
        let stages = ["projects", "deploy", "krl", "start"].map(str::to_owned).to_vec();
        let running = DeployStatus {
            state: JobState::Running,
            stage: Some("deploy".into()),
            stages: stages.clone(),
            done: vec!["projects".into()],
            ..Default::default()
        };
        assert_eq!(stages_text(&running), "projects ✓ · deploy … · krl · start");
        let failed = DeployStatus { state: JobState::Failed, ..running };
        assert_eq!(stages_text(&failed), "projects ✓ · deploy ✗ · krl · start");
        let done = DeployStatus { state: JobState::Done, stage: None, done: stages.clone(), stages, ..Default::default() };
        assert_eq!(stages_text(&done), "projects ✓ · deploy ✓ · krl ✓ · start ✓");
    }

    #[test]
    fn the_demo_runtime_line() {
        let now = 14.0 * 3600.0;
        let rt = PlcRuntime { running: true, stops_s: Some(now + 37.0 * 60.0 - 5.0), ..Default::default() };
        assert_eq!(runtime_text(&rt, now), ("CODESYS demo runtime: stops at 14:36 UTC (in 37 min)".to_owned(), false));
        let soon = PlcRuntime { stops_s: Some(now + 300.0), ..rt.clone() };
        assert!(runtime_text(&soon, now).1, "under 10 min: the warning colour");
        assert_eq!(runtime_text(&PlcRuntime { running: false, ..rt.clone() }, now).0, "CODESYS demo runtime: stopped");
        assert!(runtime_text(&PlcRuntime { stops_s: Some(now - 1.0), ..rt.clone() }, now).0.contains("stopped at"));
        // A failed read: neither "stopped" nor the old stop time.
        let unread = PlcRuntime { running: false, error: Some("ssm: ConnectionLost".into()), ..rt };
        assert_eq!(runtime_text(&unread, now), ("CODESYS demo runtime: unknown (not read)".to_owned(), true));
    }
}
