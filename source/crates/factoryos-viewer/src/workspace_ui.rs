//! Workspaces behind the workspace edge (CloudFront and its login): the switcher in the top bar
//! (the user's own workspace, and shared ones such as `ci`), the Workspace section of the settings
//! (backup, replace the instance, start fresh) and signing the planner agent in.
//!
//! Workspaces are the provisioner's ([`workspace::LIST`] …), next to the backend on the same
//! origin. Without one (a local backend, a plain server) only the agent sign-in is shown.
//!
//! The agent signs in with the build's Identity Center where it has one ([`DEFAULT_START_URL`]:
//! its start URL and region editable in Advanced mode); without one the user chooses an AWS
//! Builder ID or their own Identity Center ([`LoginMode`]). A server that fixes the sign-in
//! itself ([`AgentLogin::preset`]) is not asked how.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::agent::{AgentLogin, AgentLoginRequest, DEFAULT_START_URL, LoginMode};
use factoryos_client::protocol::api;
use factoryos_client::protocol::workspace::{
    self, Reprovision, ReprovisionMode, WorkspaceKind, WorkspaceList, WorkspaceState, WorkspaceStatus,
};
use web_time::{Duration, Instant};

use crate::theme;
use crate::rr::egui::{self, Color32, RichText};

const LIST_REFRESH: Duration = Duration::from_secs(60);
const AGENT_POLL: Duration = Duration::from_secs(3);
/// Up to this many workspaces, the top bar shows them as chips.
const CHIPS_MAX: usize = 4;

/// The agent sign-in's Defaults button (Advanced, where the build has an Identity Center): what they are.
#[cfg(feature = "internal")]
const AGENT_DEFAULTS: &str = crate::internal::AGENT_SIGN_IN_DEFAULTS;
#[cfg(not(feature = "internal"))]
const AGENT_DEFAULTS: &str = "The build's Identity Center and region";

#[derive(Default)]
struct Inbox {
    list: Option<Result<WorkspaceList, String>>,
    status: Option<Result<WorkspaceStatus, String>>,
    agent: Option<Result<AgentLogin, String>>,
    /// An action finished: what to tell the user, and whether to reconnect (the instance is new).
    done: Option<(Result<String, String>, bool)>,
}

type Wake = Arc<dyn Fn() + Send + Sync>;

pub struct Workspaces {
    /// The provisioner's origin: the backend's (behind the edge they share it).
    provisioner: Api,
    backend: Api,
    list: Option<WorkspaceList>,
    /// The list could not be fetched: no provisioner here.
    no_provisioner: bool,
    status: Option<WorkspaceStatus>,
    agent: Option<AgentLogin>,
    agent_error: Option<String>,
    message: Option<Result<String, String>>,
    busy: bool,
    confirm_fresh: bool,
    listed: Option<Instant>,
    agent_polled: Option<Instant>,
    agent_requested: bool,
    /// Open the approval page once the device code arrives (from the click that asked for it).
    open_when_pending: bool,
    /// Where the agent signs in (editable before signing in).
    agent_request: AgentLoginRequest,
    inbox: Arc<Mutex<Inbox>>,
    wake: Wake,
}

impl Workspaces {
    /// For the backend at `backend_base`; the provisioner is looked for on the same origin.
    pub fn new(backend_base: &str, wake: impl Fn() + Send + Sync + 'static) -> Self {
        let origin = origin_of(backend_base);
        let local = !backend_base.starts_with("https://") && !cfg!(target_arch = "wasm32");
        Self {
            provisioner: Api::new(origin),
            backend: Api::new(backend_base),
            list: None,
            // Natively only https backends can be behind the edge; on the web, ask the origin.
            no_provisioner: local,
            status: None,
            agent: None,
            agent_error: None,
            message: None,
            busy: false,
            confirm_fresh: false,
            listed: None,
            agent_polled: None,
            agent_requested: false,
            open_when_pending: false,
            agent_request: AgentLoginRequest::default(),
            inbox: Arc::default(),
            wake: Arc::new(wake),
        }
    }

    /// The selected workspace, if there is a provisioner.
    pub fn selected(&self) -> Option<&str> {
        self.list.as_ref().map(|list| list.selected.as_str())
    }

    fn selected_kind(&self) -> Option<WorkspaceKind> {
        let list = self.list.as_ref()?;
        list.workspaces.iter().find(|entry| entry.name == list.selected).map(|entry| entry.kind)
    }

    pub fn is_shared(&self) -> bool {
        self.selected_kind() == Some(WorkspaceKind::Shared)
    }

    /// Apply results and refresh what is due. Call every frame.
    pub fn poll(&mut self, ctx: &egui::Context) {
        let inbox = std::mem::take(&mut *self.inbox.lock().unwrap());
        if let Some(list) = inbox.list {
            match list {
                Ok(list) => self.list = Some(list),
                Err(_) => self.no_provisioner = self.list.is_none(),
            }
        }
        if let Some(status) = inbox.status {
            self.status = status.ok();
        }
        if let Some(agent) = inbox.agent {
            self.agent_requested = false;
            match agent {
                Ok(agent) => {
                    if self.open_when_pending
                        && let Some(pending) = &agent.pending
                    {
                        open_url(&pending.verification_uri);
                        self.open_when_pending = false;
                    }
                    self.agent_error = agent.error.clone();
                    self.agent = Some(agent);
                }
                Err(err) => self.agent_error = Some(err),
            }
        }
        if let Some((result, reconnect)) = inbox.done {
            self.busy = false;
            let ok = result.is_ok();
            self.message = Some(result);
            if ok && reconnect {
                reconnect_to(ctx, self.selected());
            }
            self.refresh_status();
        }

        if !self.no_provisioner && self.listed.is_none_or(|at| at.elapsed() > LIST_REFRESH) {
            self.listed = Some(Instant::now());
            self.refresh_list();
            self.refresh_status();
        }
        let pending = self.agent.as_ref().is_some_and(|agent| agent.pending.is_some());
        if pending && self.agent_polled.is_none_or(|at| at.elapsed() > AGENT_POLL) {
            self.refresh_agent();
        }
        if pending {
            ctx.request_repaint_after(AGENT_POLL);
        }
    }

    fn refresh_list(&self) {
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.provisioner.get(workspace::LIST, move |result| {
            inbox.lock().unwrap().list = Some(result);
            wake();
        });
    }

    fn refresh_status(&self) {
        if self.no_provisioner {
            return;
        }
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        // Personal status (backups) only; a shared workspace has none.
        self.provisioner.get(workspace::STATUS, move |result: Result<WorkspaceStatus, String>| {
            let result = result.or_else(|err| factoryos_client::workspace_status(&err).ok_or(err));
            inbox.lock().unwrap().status = Some(result);
            wake();
        });
    }

    fn refresh_agent(&mut self) {
        if self.agent_requested {
            return;
        }
        self.agent_requested = true;
        self.agent_polled = Some(Instant::now());
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.backend.get(api::AGENT_LOGIN, move |result| {
            inbox.lock().unwrap().agent = Some(result);
            wake();
        });
    }

    /// Whether the planner agent is signed in (`None`: not known yet), checked again when older
    /// than `max_age`. For the chat, which offers the sign-in before the first prompt fails.
    pub fn agent_signed_in(&mut self, max_age: Duration) -> Option<bool> {
        if self.agent_polled.is_none_or(|at| at.elapsed() > max_age) {
            self.refresh_agent();
        }
        self.agent.as_ref().map(|agent| agent.logged_in)
    }

    fn sign_in_agent(&mut self) {
        self.agent_requested = true;
        self.open_when_pending = true;
        self.agent_polled = Some(Instant::now());
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.backend.post_json(api::AGENT_LOGIN, &self.agent_request, move |result| {
            inbox.lock().unwrap().agent = Some(result);
            wake();
        });
    }

    fn sign_out_agent(&mut self) {
        self.agent_requested = true;
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.backend.delete(api::AGENT_LOGIN, move |result| {
            inbox.lock().unwrap().agent = Some(result);
            wake();
        });
    }

    fn backup(&mut self) {
        self.busy = true;
        self.message = None;
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.provisioner.post(workspace::BACKUP, move |result: Result<serde_json::Value, String>| {
            let result = result.map(|_| "Backup started; it runs on the instance and takes a minute or two.".to_owned());
            inbox.lock().unwrap().done = Some((result, false));
            wake();
        });
    }

    fn reprovision(&mut self, mode: ReprovisionMode) {
        self.busy = true;
        self.message = None;
        let (inbox, wake) = (self.inbox.clone(), self.wake.clone());
        self.provisioner.post_json(
            workspace::REPROVISION,
            &Reprovision { mode },
            move |result: Result<WorkspaceStatus, String>| {
                let result = result
                    .or_else(|err| factoryos_client::workspace_status(&err).ok_or(err))
                    .map(|status| status.describe());
                inbox.lock().unwrap().done = Some((result, true));
                wake();
            },
        );
    }

    /// The switcher at the right of the top bar, hidden with only one workspace: chips for a few,
    /// `● <workspace> ▾` for more.
    pub fn top_bar_ui(&mut self, ui: &mut egui::Ui) {
        let Some(list) = &self.list else { return };
        if list.workspaces.len() < 2 {
            return;
        }
        let selected = list.selected.clone();
        let mut choice = selected.clone();
        let entries = list.workspaces.clone();
        let alias = list.alias.clone();
        // A few: as chips, the active one in the accent (Kiro Crew's instance tabs); more: a list.
        if entries.len() <= CHIPS_MAX {
            let p = theme::palette(ui);
            ui.spacing_mut().item_spacing.x = 4.0;
            for entry in &entries {
                let active = entry.name == selected;
                let (fill, text) = if active { (p.accent_subtle, p.accent) } else { (Color32::TRANSPARENT, p.muted) };
                let chip = egui::Button::new(RichText::new(&entry.name).color(text))
                    .fill(fill)
                    .stroke(egui::Stroke::new(1.0, if active { p.accent.gamma_multiply(0.5) } else { p.border }))
                    .corner_radius(8);
                let dot = state_color(entry.state);
                let response = ui.add(chip).on_hover_text(match entry.kind {
                    WorkspaceKind::Shared => "A shared workspace: its cells, config, chat and agent are everyone's",
                    WorkspaceKind::Personal => "Your own workspace",
                });
                // Its state as a dot in the chip's corner.
                ui.painter().circle_filled(response.rect.right_top() + egui::vec2(-4.0, 4.0), 2.5, dot);
                if response.clicked() && !active {
                    choice = entry.name.clone();
                }
            }
        } else {
        theme::with_controls(ui, |ui| egui::ComboBox::from_id_salt("factory_os_workspace")
            .selected_text(RichText::new(format!("● {selected}")).color(state_color(self.entry_state(&selected))))
            .show_ui(ui, |ui| {
                for entry in &entries {
                    let kind = match entry.kind {
                        WorkspaceKind::Personal if entry.name == alias => "personal",
                        WorkspaceKind::Personal => "",
                        WorkspaceKind::Shared => "shared",
                    };
                    let text = RichText::new(format!("● {}  {kind}", entry.name)).color(state_color(entry.state));
                    ui.selectable_value(&mut choice, entry.name.clone(), text);
                }
            })
            .response
            .on_hover_text("The workspace this viewer works in: your own, or a shared one"));
        }
        if self.is_shared() {
            ui.label(RichText::new("shared").small().color(theme::current().warn))
                .on_hover_text("A shared workspace: its cells, config, chat and agent are everyone's");
        }
        if choice != selected {
            switch_to(ui.ctx(), &choice);
        }
    }

    fn entry_state(&self, name: &str) -> WorkspaceState {
        self.list
            .as_ref()
            .and_then(|list| list.workspaces.iter().find(|entry| entry.name == name))
            .map_or(WorkspaceState::None, |entry| entry.state)
    }

    /// A banner for a shared workspace, under the top bar.
    pub fn shared_banner_ui(&self, ui: &mut egui::Ui) {
        if self.is_shared() {
            ui.label(
                RichText::new(format!(
                    "Shared workspace \"{}\": its state is shared with everyone, and CI deploys restart it.",
                    self.selected().unwrap_or_default()
                ))
                .small()
                .color(theme::current().warn),
            );
        }
    }

    /// The Workspace section of the settings.
    pub fn settings_ui(&mut self, ui: &mut egui::Ui) {
        if let Some(list) = self.list.clone() {
            ui.add_space(10.0);
            ui.label(RichText::new("WORKSPACE").small().strong());
            let shared = self.is_shared();
            ui.horizontal(|ui| {
                ui.monospace(&list.selected);
                ui.small(RichText::new(if shared { "shared" } else { "your own" }).weak());
            });
            self.shared_banner_ui(ui);
            if !shared {
                if let Some(status) = &self.status {
                    ui.small(format!("State: {}", status.describe()));
                    if let Some(release) = status.release.as_ref().filter(|_| crate::ui_mode::is_advanced(ui)) {
                        ui.small(RichText::new(format!("Release: {release}")).weak());
                    }
                    ui.small(match status.last_backup {
                        Some(at) => format!("Last backup: {}", ago(at)),
                        None => "No backup yet".to_owned(),
                    });
                }
                ui.add_enabled_ui(!self.busy, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("Back up now").on_hover_text("Save the whole workspace to S3 now").clicked() {
                            self.backup();
                        }
                        // Replace instance and Start fresh: Advanced only.
                        if !crate::ui_mode::is_advanced(ui) {
                            return;
                        }
                        if ui
                            .button("Replace instance")
                            .on_hover_text("Back up, then continue on a fresh instance that restores the backup")
                            .clicked()
                        {
                            self.reprovision(ReprovisionMode::Keep);
                        }
                        if ui
                            .button("Start fresh…")
                            .on_hover_text("Back up and archive the workspace, then start over from the starter content")
                            .clicked()
                        {
                            self.confirm_fresh = true;
                        }
                    });
                });
                if self.confirm_fresh {
                    ui.label(
                        RichText::new(
                            "Start from the starter content? The current workspace is backed up and archived \
                             (not deleted); the viewer reconnects to the new instance.",
                        )
                        .small()
                        .color(Color32::ORANGE),
                    );
                    ui.horizontal(|ui| {
                        if ui.button("Archive and start fresh").clicked() {
                            self.confirm_fresh = false;
                            self.reprovision(ReprovisionMode::Fresh);
                        }
                        if ui.button("Cancel").clicked() {
                            self.confirm_fresh = false;
                        }
                    });
                }
                if self.busy {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.small("Working…");
                    });
                }
                match &self.message {
                    Some(Ok(message)) => {
                        ui.small(RichText::new(message).weak());
                    }
                    Some(Err(err)) => {
                        ui.small(RichText::new(err).color(Color32::LIGHT_RED));
                    }
                    None => {}
                }
            }
        }
        ui.add_space(10.0);
        ui.label(RichText::new("PLANNER AGENT (KIRO)").small().strong());
        // Fresh whenever the settings are open (the agent may have been signed in or out elsewhere).
        self.agent_signed_in(Duration::from_secs(10));
        self.agent_ui(ui);
    }

    /// Whether the planner agent is signed in, and the sign-in (a device code approved in a browser).
    pub fn agent_ui(&mut self, ui: &mut egui::Ui) {
        if self.agent.is_none() && !self.agent_requested && self.agent_error.is_none() {
            self.refresh_agent();
        }
        let shared_note = if self.is_shared() { " (the shared workspace's agent, for everyone)" } else { "" };
        match self.agent.clone() {
            None if self.agent_requested => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.small("Checking the agent's sign-in…");
                });
            }
            Some(AgentLogin { logged_in: true, user, .. }) => {
                ui.horizontal(|ui| {
                    ui.small(format!("Signed in{}{shared_note}", user.map(|user| format!(" as {user}")).unwrap_or_default()));
                    if ui.small_button("Sign out").clicked() {
                        self.sign_out_agent();
                    }
                });
            }
            Some(AgentLogin { pending: Some(code), .. }) => {
                ui.small("Approve the sign-in in the browser window that opened (or open the page below):");
                ui.horizontal(|ui| {
                    if !code.user_code.is_empty() {
                        ui.small("Code:");
                        ui.label(RichText::new(&code.user_code).monospace().strong());
                        if ui.small_button("Copy").clicked() {
                            ui.ctx().copy_text(code.user_code.clone());
                        }
                    }
                    ui.spinner();
                });
                ui.horizontal_wrapped(|ui| {
                    ui.add(egui::Hyperlink::from_label_and_url(RichText::new(&code.verification_uri).monospace().small(), &code.verification_uri).open_in_new_tab(true))
                        .on_hover_text("Open the approval page");
                    if ui.small_button("Copy URL").clicked() {
                        ui.ctx().copy_text(code.verification_uri.clone());
                    }
                });
            }
            _ => {
                ui.small(format!("Not signed in{shared_note}"));
                // The server fixes how (its FACTORY_OS_KIRO_LOGIN_ARGS): nothing to choose or enter.
                let preset = self.agent.as_ref().is_some_and(|agent| agent.preset);
                if !preset {
                    self.sign_in_options_ui(ui);
                }
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!self.agent_requested, egui::Button::new("Sign in to the agent"))
                        .on_hover_text(sign_in_hover(preset, &self.agent_request))
                        .clicked()
                    {
                        match sign_in_check(preset, &self.agent_request) {
                            Ok(()) => self.sign_in_agent(),
                            Err(err) => self.agent_error = Some(err),
                        }
                    }
                    if !preset
                        && DEFAULT_START_URL.is_some()
                        && crate::ui_mode::is_advanced(ui)
                        && ui.small_button("Defaults").on_hover_text(AGENT_DEFAULTS).clicked()
                    {
                        self.agent_request = AgentLoginRequest::default();
                    }
                });
            }
        }
        if let Some(err) = &self.agent_error {
            ui.small(RichText::new(err).color(Color32::LIGHT_RED));
        }
    }

    /// How the agent is to sign in: the build's Identity Center, whose start URL and region only
    /// Advanced mode shows (the defaults apply); or, in a build without one, the user's choice of
    /// an AWS Builder ID or an Identity Center of their own (its start URL and region).
    fn sign_in_options_ui(&mut self, ui: &mut egui::Ui) {
        if DEFAULT_START_URL.is_some() {
            crate::ui_mode::advanced(ui, |ui| self.start_url_ui(ui, None));
            return;
        }
        ui.horizontal(|ui| {
            for mode in LoginMode::ALL {
                ui.radio_value(&mut self.agent_request.mode, mode, mode.label()).on_hover_text(match mode {
                    LoginMode::BuilderId => "A personal AWS Builder ID (kiro-cli login --license free)",
                    LoginMode::IdentityCenter => {
                        "Your organization's IAM Identity Center: its start URL and region (kiro-cli login --license pro)"
                    }
                });
            }
        });
        if self.agent_request.mode == LoginMode::IdentityCenter {
            self.start_url_ui(ui, Some("https://…/start"));
        }
    }

    /// The Identity Center's start URL (with `hint` while empty) and region.
    fn start_url_ui(&mut self, ui: &mut egui::Ui, hint: Option<&str>) {
        egui::Grid::new("agent_sign_in").num_columns(2).spacing([8.0, 4.0]).show(ui, |ui| {
            ui.small("Start URL");
            let mut start_url = egui::TextEdit::singleline(&mut self.agent_request.start_url).desired_width(260.0);
            if let Some(hint) = hint {
                start_url = start_url.hint_text(hint);
            }
            ui.add(start_url);
            ui.end_row();
            ui.small("Region");
            ui.add(egui::TextEdit::singleline(&mut self.agent_request.region).desired_width(120.0));
            ui.end_row();
        });
    }
}

/// Whether the sign-in `request` may be sent: always where the server fixes the sign-in itself
/// (`preset`: the request's mode, start URL and region are not used), else if kiro-cli can run it.
fn sign_in_check(preset: bool, request: &AgentLoginRequest) -> Result<(), String> {
    if preset { Ok(()) } else { request.validate() }
}

/// The sign-in button's hover: what it runs.
fn sign_in_hover(preset: bool, request: &AgentLoginRequest) -> &'static str {
    match (preset, DEFAULT_START_URL, request.mode) {
        (true, ..) => "Runs the server's kiro-cli login (its configuration says how), and opens the approval page in a new browser window",
        (false, Some(_), _) => "Runs kiro-cli login with these, and opens the approval page in a new browser window",
        (false, None, LoginMode::BuilderId) => {
            "Runs kiro-cli login with an AWS Builder ID, and opens the approval page in a new browser window"
        }
        (false, None, LoginMode::IdentityCenter) => {
            "Runs kiro-cli login with this Identity Center, and opens the approval page in a new browser window"
        }
    }
}

fn state_color(state: WorkspaceState) -> Color32 {
    match state {
        WorkspaceState::Ready => theme::current().ok,
        WorkspaceState::Error => theme::current().danger,
        WorkspaceState::None | WorkspaceState::Stopped => Color32::GRAY,
        _ => theme::current().warn,
    }
}

/// `scheme://host[:port]` of a URL.
fn origin_of(url: &str) -> String {
    let Some((scheme, rest)) = url.split_once("://") else { return url.to_owned() };
    let host = rest.split(['/', '?', '#']).next().unwrap_or(rest);
    format!("{scheme}://{host}")
}

fn ago(unix: u64) -> String {
    // web_time: std's SystemTime::now panics on wasm32 (which froze the web viewer's settings).
    let now = web_time::SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map_or(0, |since| since.as_secs());
    let secs = now.saturating_sub(unix);
    match secs {
        0..60 => "just now".to_owned(),
        60..3600 => format!("{} min ago", secs / 60),
        3600..86400 => format!("{} h ago", secs / 3600),
        _ => format!("{} days ago", secs / 86400),
    }
}

/// Open `url` in a (new) browser window.
pub fn open_url(url: &str) {
    #[cfg(not(target_arch = "wasm32"))]
    if let Err(err) = webbrowser::open(url) {
        // Not the query: it may hold a session token (Open PLC IDE).
        crate::rr::re_log::warn!("Could not open the browser ({err}); open {}", without_query(url));
    }
    #[cfg(target_arch = "wasm32")]
    crate::web::open_window(url);
}

/// `url` without its query and fragment (for logs: a query may carry a token).
pub fn without_query(url: &str) -> &str {
    url.split(['?', '#']).next().unwrap_or(url)
}

/// Continue in workspace `name`: the web viewer reloads with the selection cookie set, the
/// native one restarts into it.
fn switch_to(ctx: &egui::Context, name: &str) {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = ctx;
        crate::web::select_workspace(name);
    }
    #[cfg(not(target_arch = "wasm32"))]
    crate::backend_choice::switch_workspace(ctx, Some(name));
}

/// Reconnect to the (re-provisioned) workspace, waiting for it to come up.
fn reconnect_to(ctx: &egui::Context, name: Option<&str>) {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (ctx, name);
        crate::web::reload();
    }
    #[cfg(not(target_arch = "wasm32"))]
    crate::backend_choice::switch_workspace(ctx, name);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origins() {
        assert_eq!(origin_of("https://h.example.com/workspace/a"), "https://h.example.com");
        assert_eq!(origin_of("http://127.0.0.1:8080"), "http://127.0.0.1:8080");
    }

    #[test]
    fn a_server_preset_skips_the_sign_in_checks() {
        let unusable = AgentLoginRequest { mode: LoginMode::IdentityCenter, start_url: String::new(), region: String::new() };
        assert!(sign_in_check(false, &unusable).is_err());
        assert!(sign_in_check(true, &unusable).is_ok(), "the server's own sign-in is used");
        assert!(sign_in_check(false, &AgentLoginRequest { mode: LoginMode::BuilderId, ..unusable }).is_ok());
    }

    #[test]
    fn the_sign_in_starts_from_the_builds_default() {
        let request = AgentLoginRequest::default();
        assert!(sign_in_check(false, &request).is_ok(), "the default needs nothing entered");
        assert!(sign_in_hover(true, &request).contains("the server's kiro-cli login"));
        match DEFAULT_START_URL {
            Some(_) => assert!(sign_in_hover(false, &request).contains("with these")),
            // No Identity Center of the build's: a Builder ID unless the user enters their own.
            None => {
                assert_eq!(request.mode, LoginMode::BuilderId);
                assert!(sign_in_hover(false, &request).contains("AWS Builder ID"));
                let own = AgentLoginRequest { mode: LoginMode::IdentityCenter, ..request };
                assert!(sign_in_hover(false, &own).contains("this Identity Center"));
                assert!(sign_in_check(false, &own).is_err(), "until its start URL is entered");
            }
        }
    }
}
