//! Which factoryos-backend the native viewer uses. Unless `FACTORY_OS_BACKEND` says, a small
//! window asks first ([`choose`]): this machine's backend or a remembered/entered address, with
//! an optional workspace (`<address>/workspace/<name>`). The viewer then opens on that backend; config, cells, the 3D stream, previews and plans all come
//! from it. "Switch backend…" in Configuration restarts the viewer into the chooser again.
//!
//! An https backend may be behind the workspace edge's login (CloudFront; the internal build lists
//! its dev server as a built-in entry, `CLOUD`): the viewer signs in, silently where the build
//! can, else through the browser, and opens the user's workspace, or another one they may open
//! (e.g. the shared `ci`), picked from the provisioner's list. A workspace that is not up yet
//! (being created, stopped while idle) is waited for.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use factoryos_client::protocol::workspace::{self, WorkspaceKind, WorkspaceList};

use crate::theme;
use crate::backend::WORKSPACE_POLL;
use crate::{Backend, LoginNeeded, WorkspaceNotReady};
use crate::rr::egui::{self, RichText};
use crate::rr::{eframe, re_log, re_viewer, tokio};

/// The backend on this machine, started by the viewer if none is running.
pub const LOCAL: &str = "http://127.0.0.1:8080";
const MAX_RECENT: usize = 8;

/// A built-in entry of the chooser.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Preset {
    pub url: &'static str,
    pub label: &'static str,
    pub note: &'static str,
}

/// The built-in cloud backend, if the build has one: the internal build's dev server
/// (its internal module). The public build has none; an address is entered instead.
#[cfg(feature = "internal")]
const CLOUD: Option<Preset> = Some(crate::internal::CLOUD_DEV);
#[cfg(not(feature = "internal"))]
const CLOUD: Option<Preset> = None;

/// Whether `url` is a built-in entry (never in the recent list).
pub(crate) fn is_preset(url: &str) -> bool {
    url == LOCAL || CLOUD.is_some_and(|cloud| cloud.url == url)
}

/// What the chooser says under a login the viewer could not do silently ([`LoginNeeded`]).
#[derive(Clone, Copy, Debug)]
pub(crate) struct LoginTexts {
    /// Retry's hover.
    pub retry: &'static str,
    /// Sign in with browser's hover.
    pub browser: &'static str,
    /// What else would let the viewer sign in, if anything.
    pub hint: Option<&'static str>,
}

/// The internal build's texts name its silent login (its internal module); the public build has none.
#[cfg(feature = "internal")]
const LOGIN_TEXTS: LoginTexts = crate::internal::LOGIN_TEXTS;
#[cfg(not(feature = "internal"))]
const LOGIN_TEXTS: LoginTexts = LoginTexts {
    retry: "Connect again",
    browser: "Opens the edge's login in your browser; the viewer continues when it is done",
    hint: None,
};

static RESTART: AtomicBool = AtomicBool::new(false);
/// The restart should connect straight away (to the saved backend and workspace).
static AUTOCONNECT: AtomicBool = AtomicBool::new(false);
static BACKEND_CHAT: AtomicBool = AtomicBool::new(false);

#[derive(Default, serde::Serialize, serde::Deserialize)]
struct Saved {
    /// The backend used last; preselected in the chooser.
    current: Option<String>,
    /// Other backends used before (never [`LOCAL`]), most recent first.
    #[serde(default)]
    recent: Vec<String>,
    /// With this machine's backend too, plan in the backend's chat (see [`backend_chat`]).
    #[serde(default)]
    backend_chat: bool,
    /// The workspace entered last; prefilled in the chooser.
    #[serde(default)]
    workspace: String,
}

fn file() -> Option<PathBuf> {
    eframe::storage_dir("factory_os").map(|dir| dir.join("backend.json"))
}

/// Keep login sessions (see `factoryos_client::auth`) next to the backend list, so a restart
/// reuses them. Call once at startup.
pub fn init_sessions() {
    if let Some(dir) = eframe::storage_dir("factory_os") {
        factoryos_client::auth::set_store(dir.join("session.json"));
    }
}

fn load() -> Saved {
    file()
        .and_then(|path| std::fs::read(path).ok())
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or_default()
}

/// Whether Agentic Planning uses the backend's planner chat even though the backend is on this
/// machine, as it always does with one on another machine: ticked in the chooser, or
/// `FACTORY_OS_BACKEND_CHAT=1`. Otherwise a local backend means Rerun's agent panel runs
/// `kiro-cli` here.
pub fn backend_chat() -> bool {
    BACKEND_CHAT.load(Ordering::SeqCst) || std::env::var_os("FACTORY_OS_BACKEND_CHAT").is_some_and(|value| value == "1")
}

/// `http://host/workspace/<name>` → (`http://host`, `name`); other URLs as they are, without a name.
fn split_workspace(url: &str) -> (&str, &str) {
    match (workspace::of_url(url), url.find(workspace::PREFIX)) {
        (Some(name), Some(at)) => (&url[..at], name),
        _ => (url, ""),
    }
}

/// Remember `url` as the backend used last (behind the edge: its origin, and the workspace apart).
fn remember(url: &str, backend_chat: bool) {
    let mut saved = load();
    saved.backend_chat = backend_chat;
    let (base, name) = if url.starts_with("https://") { split_workspace(url) } else { (url, "") };
    if !name.is_empty() || base != url {
        saved.workspace = name.to_owned();
    }
    let url = base;
    saved.recent.retain(|recent| recent != url);
    if !is_preset(url) {
        saved.recent.insert(0, url.to_owned());
        saved.recent.truncate(MAX_RECENT);
    }
    saved.current = Some(url.to_owned());
    save(&saved);
}

fn save(saved: &Saved) {
    let Some(path) = file() else { return };
    let result = path
        .parent()
        .map_or(Ok(()), std::fs::create_dir_all)
        .and_then(|()| std::fs::write(&path, serde_json::to_vec_pretty(saved).unwrap_or_default()));
    if let Err(err) = result {
        re_log::warn!("Failed to save {}: {err}", path.display());
    }
}

/// `host`, `host:port` or a URL → `http://host[:port]`, without a trailing slash.
pub fn normalize(input: &str) -> Option<String> {
    let input = input.trim().trim_end_matches('/');
    if input.is_empty() || input.contains(char::is_whitespace) {
        return None;
    }
    let url = if input.contains("://") { input.to_owned() } else { format!("http://{input}") };
    (url.starts_with("http://") || url.starts_with("https://")).then_some(url)
}

/// `address` (see [`normalize`]) in workspace `name`, unless it names one itself already.
pub fn with_workspace(address: &str, name: &str) -> Result<String, String> {
    let url = normalize(address).ok_or_else(|| format!("not a backend address: {}", address.trim()))?;
    let name = name.trim();
    if name.is_empty() || workspace::of_url(&url).is_some() {
        return Ok(url);
    }
    if !workspace::is_valid_name(name) {
        return Err(format!("not a workspace name: {name} (letters, digits, '.', '_', '-')"));
    }
    Ok(workspace::url(&url, name))
}

/// `http://host/workspace/<name>` → `http://host · <name>`; other URLs as they are.
fn label(url: &str) -> String {
    match (workspace::of_url(url), url.find(workspace::PREFIX)) {
        (Some(name), Some(at)) => format!("{} · {name}", &url[..at]),
        _ => url.to_owned(),
    }
}

/// Show the chooser until a backend is connected; `None` if the window was closed instead.
/// Call from the main thread, inside the tokio runtime.
pub fn choose() -> anyhow::Result<Option<Backend>> {
    let chosen = Rc::new(RefCell::new(None));
    let mut options = re_viewer::native::eframe_options(None);
    options.viewport = egui::ViewportBuilder::default()
        .with_app_id("factory_os")
        .with_title("Physical AI Production Designer: choose a backend")
        .with_inner_size([520.0, 460.0])
        .with_resizable(false);
    let result = Rc::clone(&chosen);
    eframe::run_native(
        "Physical AI Production Designer backend",
        options,
        Box::new(move |cc| {
            crate::theme::setup_plain_window(cc);
            Ok(Box::new(Chooser::new(result)))
        }),
    )
    .map_err(|err| anyhow::anyhow!("backend chooser: {err}"))?;
    Ok(chosen.take())
}

type Attempt = Arc<Mutex<Option<(u64, anyhow::Result<Backend>)>>>;
type Listing = Arc<Mutex<Option<(String, Result<WorkspaceList, String>)>>>;

struct Chooser {
    saved: Saved,
    /// The selected entry: [`LOCAL`] or an address.
    selected: String,
    input: String,
    /// The workspace for an entered address; empty for none.
    workspace: String,
    /// The connection in progress: its number and address.
    connecting: Option<(u64, String)>,
    /// See [`backend_chat`].
    backend_chat: bool,
    attempts: u64,
    finished: Attempt,
    error: Option<String>,
    /// The address whose connection needs a login (see [`LoginNeeded`]), and why.
    login: Option<(String, String)>,
    /// The workspace is coming up: its latest status, and since when we wait.
    provisioning: Option<(workspace::WorkspaceStatus, web_time::Instant)>,
    /// The workspaces of the selected https backend (its provisioner's list), once signed in.
    workspaces: Option<(String, WorkspaceList)>,
    listing: Listing,
    listed_origin: Option<String>,
    /// Connect at the first frame (after a workspace switch).
    autoconnect: bool,
    chosen: Rc<RefCell<Option<Backend>>>,
}

impl Chooser {
    fn new(chosen: Rc<RefCell<Option<Backend>>>) -> Self {
        let saved = load();
        let selected = saved.current.clone().unwrap_or_else(|| LOCAL.to_owned());
        Self {
            backend_chat: saved.backend_chat,
            workspace: saved.workspace.clone(),
            saved,
            selected,
            input: String::new(),
            connecting: None,
            attempts: 0,
            finished: Attempt::default(),
            error: None,
            login: None,
            provisioning: None,
            workspaces: None,
            listing: Listing::default(),
            listed_origin: None,
            autoconnect: std::env::var_os("FACTORY_OS_AUTOCONNECT").is_some_and(|value| value == "1"),
            chosen,
        }
    }

    /// The URL to connect to for the selected entry: an https backend in the chosen workspace.
    fn target(&self) -> Result<String, String> {
        if self.input.trim().is_empty() {
            let url = Some(self.selected.clone()).filter(|url| !url.is_empty()).ok_or_else(|| "choose a backend".to_owned())?;
            if url.starts_with("https://") { with_workspace(&url, &self.workspace) } else { Ok(url) }
        } else {
            with_workspace(&self.input, &self.workspace)
        }
    }

    /// List the selected https backend's workspaces once there is a session for it.
    fn list_workspaces(&mut self, ctx: &egui::Context) {
        if let Some((origin, Ok(list))) = self.listing.lock().unwrap().take() {
            if self.workspace.is_empty() || !list.workspaces.iter().any(|entry| entry.name == self.workspace) {
                self.workspace = list.alias.clone();
            }
            self.workspaces = Some((origin, list));
        }
        let Some(origin) = factoryos_client::auth::origin(&self.selected).filter(|_| self.selected.starts_with("https://"))
        else {
            return;
        };
        if self.listed_origin.as_deref() == Some(origin.as_str()) || factoryos_client::auth::session(&origin).is_none() {
            return;
        }
        self.listed_origin = Some(origin.clone());
        let (listing, ctx) = (self.listing.clone(), ctx.clone());
        factoryos_client::Api::new(origin.clone()).get(workspace::LIST, move |result| {
            *listing.lock().unwrap() = Some((origin, result));
            ctx.request_repaint();
        });
    }

    fn connect(&mut self, ctx: &egui::Context, url: String) {
        self.start(ctx, url, false);
    }

    /// Connect to `url`, after logging in through the browser if `browser`.
    fn start(&mut self, ctx: &egui::Context, url: String, browser: bool) {
        self.provisioning = None;
        self.start_after(ctx, url, browser, None);
    }

    /// [`Self::start`], after `delay` (polling a workspace that is coming up).
    fn start_after(&mut self, ctx: &egui::Context, url: String, browser: bool, delay: Option<std::time::Duration>) {
        self.attempts += 1;
        let attempt = self.attempts;
        self.selected = if url.starts_with("https://") { split_workspace(&url).0.to_owned() } else { url.clone() };
        self.connecting = Some((attempt, url.clone()));
        self.error = None;
        self.login = None;
        let (finished, ctx) = (self.finished.clone(), ctx.clone());
        tokio::spawn(async move {
            if let Some(delay) = delay {
                tokio::time::sleep(delay).await;
            }
            let result = match browser {
                true => match Backend::login_with_browser(&url).await {
                    Ok(()) => Backend::connect_to(&url).await,
                    Err(err) => Err(err),
                },
                false => Backend::connect_to(&url).await,
            };
            *finished.lock().unwrap() = Some((attempt, result));
            ctx.request_repaint();
        });
    }

    /// Take a finished connection: the current attempt's result; older ones are dropped (a
    /// backend they started stops with them).
    fn poll(&mut self, ctx: &egui::Context) {
        let Some((attempt, result)) = self.finished.lock().unwrap().take() else { return };
        let Some((_, url)) = self.connecting.take_if(|(current, _)| *current == attempt) else {
            if let Ok(mut stale) = result {
                stale.shutdown();
            }
            return;
        };
        if let Err(err) = &result
            && let Some(WorkspaceNotReady(status)) = err.downcast_ref::<WorkspaceNotReady>()
            // Ready too: the load balancer may still be switching over to it.
            && (status.state.is_coming_up()
                || matches!(status.state, workspace::WorkspaceState::None | workspace::WorkspaceState::Stopped | workspace::WorkspaceState::Ready))
        {
            // Being created or started: show how far it is and try again shortly.
            let since = self.provisioning.as_ref().map_or_else(web_time::Instant::now, |(_, since)| *since);
            self.provisioning = Some((status.clone(), since));
            self.start_after(ctx, url, false, Some(WORKSPACE_POLL));
            return;
        }
        self.provisioning = None;
        match result {
            Ok(backend) => {
                remember(&url, self.backend_chat);
                BACKEND_CHAT.store(self.backend_chat && url == LOCAL, Ordering::SeqCst);
                *self.chosen.borrow_mut() = Some(backend);
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            Err(err) => match err.downcast_ref::<LoginNeeded>() {
                Some(LoginNeeded(why)) => self.login = Some((url, why.clone())),
                None => self.error = Some(format!("{err:#}")),
            },
        }
    }

    fn entry(&mut self, ui: &mut egui::Ui, url: &str, label: &str, note: &str) {
        let selected = self.selected == url;
        let response = ui
            .add_sized(
                [ui.available_width(), 0.0],
                egui::Button::selectable(selected, RichText::new(label).monospace()).right_text(RichText::new(note).weak()),
            )
            .on_hover_text(url);
        if response.clicked() {
            self.selected = url.to_owned();
        }
        if response.double_clicked() {
            self.connect(ui.ctx(), url.to_owned());
        }
    }
}

impl eframe::App for Chooser {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        crate::theme::begin_frame(ui.ctx());
        self.poll(ui.ctx());
        self.list_workspaces(ui.ctx());
        if std::mem::take(&mut self.autoconnect) {
            match self.target() {
                Ok(url) => self.connect(ui.ctx(), url),
                Err(err) => self.error = Some(err),
            }
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::central_panel(ui.style()).inner_margin(20.0))
            .show(ui, |ui| {
                crate::theme::controls_style(ui);
                ui.heading("Physical AI Production Designer");
                ui.label(RichText::new("Choose the backend that runs the cells, tools and planner.").weak());
                ui.add_space(14.0);

                ui.label(RichText::new("BACKENDS").small().strong());
                egui::ScrollArea::vertical().max_height(170.0).show(ui, |ui| {
                    self.entry(ui, LOCAL, "This machine", "starts a local backend if needed");
                    if let Some(cloud) = CLOUD {
                        self.entry(ui, cloud.url, cloud.label, cloud.note);
                    }
                    for url in self.saved.recent.clone() {
                        self.entry(ui, &url, &label(&url), "");
                    }
                });
                ui.add_enabled_ui(self.selected == LOCAL, |ui| {
                    ui.checkbox(&mut self.backend_chat, "Planner chat in the backend (as on the server)")
                        .on_hover_text(
                            "With this machine's backend, plan in the backend's chat pane (kiro-cli run by the \
                             backend) instead of Rerun's agent panel. A backend on another machine always \
                             uses its own chat.",
                        );
                });

                ui.add_space(10.0);
                ui.label(RichText::new("OTHER").small().strong());
                let response = ui.add(
                    egui::TextEdit::singleline(&mut self.input)
                        .hint_text("host[:port] or http://…, e.g. 192.0.2.10")
                        .desired_width(ui.available_width()),
                );
                let mut entered = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if response.changed() && !self.input.trim().is_empty() {
                    self.selected.clear();
                }
                let listed = self
                    .workspaces
                    .as_ref()
                    .filter(|(origin, _)| self.input.trim().is_empty() && self.selected.starts_with(origin.as_str()))
                    .map(|(_, list)| list.clone());
                if let Some(list) = listed {
                    ui.add_space(10.0);
                    ui.label(RichText::new("WORKSPACE").small().strong());
                    egui::ComboBox::from_id_salt("chooser_workspace")
                        .width(ui.available_width())
                        .selected_text(self.workspace.clone())
                        .show_ui(ui, |ui| {
                            for entry in &list.workspaces {
                                let kind = match entry.kind {
                                    WorkspaceKind::Personal => "your own",
                                    WorkspaceKind::Shared => "shared with everyone",
                                };
                                ui.selectable_value(&mut self.workspace, entry.name.clone(), format!("{}  ({kind})", entry.name));
                            }
                        });
                } else {
                    let response = ui
                        .add(
                            egui::TextEdit::singleline(&mut self.workspace)
                                .hint_text("workspace (optional): yours if empty, or e.g. ci")
                                .desired_width(ui.available_width()),
                        )
                        .on_hover_text("The workspace to open (behind the cloud login: yours if empty, or a shared one)");
                    entered |= response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                }

                ui.add_space(14.0);
                ui.horizontal(|ui| {
                    let busy = self.connecting.is_some();
                    let label = if busy { "Connecting…" } else { "Connect" };
                    if (ui.add_enabled(!busy, egui::Button::new(label)).clicked() || entered) && !busy {
                        match self.target() {
                            Ok(url) => self.connect(ui.ctx(), url),
                            Err(err) => self.error = Some(err),
                        }
                    }
                    if let Some((_, url)) = &self.connecting {
                        ui.spinner();
                        let what = if url.starts_with("https://") { format!("{url} (signing in…)") } else { url.clone() };
                        ui.label(RichText::new(what).monospace().weak());
                    }
                });
                if let Some((status, since)) = &self.provisioning {
                    ui.add_space(6.0);
                    let waited = since.elapsed().as_secs();
                    ui.label(
                        RichText::new(format!(
                            "Workspace {}: {} ({}:{:02})",
                            status.name,
                            status.describe(),
                            waited / 60,
                            waited % 60
                        ))
                        .small()
                        .color(theme::current().warn),
                    );
                    ui.label(
                        RichText::new(
                            "A new workspace takes a few minutes (instance, restore, install); a stopped one about a minute.",
                        )
                        .small()
                        .weak(),
                    );
                    if ui.button("Cancel").clicked() {
                        self.attempts += 1; // drops the pending retry's result
                        self.connecting = None;
                        self.provisioning = None;
                    }
                }
                if let Some((url, why)) = self.login.clone() {
                    ui.add_space(6.0);
                    ui.label(RichText::new(why).small().color(theme::current().warn));
                    ui.horizontal(|ui| {
                        if ui.button("Retry").on_hover_text(LOGIN_TEXTS.retry).clicked() {
                            self.start(ui.ctx(), url.clone(), false);
                        }
                        if ui.button("Sign in with browser").on_hover_text(LOGIN_TEXTS.browser).clicked() {
                            self.start(ui.ctx(), url, true);
                        }
                    });
                    if let Some(hint) = LOGIN_TEXTS.hint {
                        ui.label(RichText::new(hint).small().weak());
                    }
                }
                if let Some(err) = &self.error {
                    ui.add_space(6.0);
                    ui.label(RichText::new(err).small().color(theme::current().danger));
                }
            });
    }
}

/// Close the viewer and start it again at the chooser.
fn restart(ctx: &egui::Context) {
    RESTART.store(true, Ordering::SeqCst);
    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
}

/// Whether the viewer closed to switch backends (see `main.rs`).
pub fn restart_requested() -> bool {
    RESTART.load(Ordering::SeqCst)
}

/// Whether the restart should connect straight away (see [`switch_workspace`]).
pub fn autoconnect_requested() -> bool {
    AUTOCONNECT.load(Ordering::SeqCst)
}

/// Continue in workspace `name` (`None`: the user's own) of the saved backend: save it and restart
/// the viewer, which connects to it at once (waiting while it comes up).
pub fn switch_workspace(ctx: &egui::Context, name: Option<&str>) {
    let mut saved = load();
    saved.workspace = name.unwrap_or_default().to_owned();
    save(&saved);
    AUTOCONNECT.store(true, Ordering::SeqCst);
    restart(ctx);
}

/// The Backend section of the Configuration pane.
pub fn ui(ui: &mut egui::Ui, current: &str, spawned: bool) {
    ui.add_space(10.0);
    ui.label(RichText::new("BACKEND").small().strong());
    ui.horizontal(|ui| {
        ui.monospace(current);
        if spawned {
            ui.small(RichText::new("started by this viewer").weak());
        }
    });
    if let Some(name) = workspace::of_url(current)
        .map(str::to_owned)
        .or_else(|| factoryos_client::auth::origin(current).and_then(|origin| factoryos_client::auth::workspace(&origin)))
    {
        ui.small(format!("Workspace: {name}"));
    }
    let chat = if !factoryos_client::is_local(current) || backend_chat() {
        "Planner chat: in the backend"
    } else {
        "Planner chat: the agent panel (kiro-cli on this machine)"
    };
    ui.small(RichText::new(chat).weak());
    if ui
        .button("Switch backend…")
        .on_hover_text("Restart the viewer and choose another backend")
        .clicked()
    {
        restart(ui.ctx());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalizes_addresses() {
        assert_eq!(normalize("203.0.113.10").as_deref(), Some("http://203.0.113.10"));
        assert_eq!(normalize(" cell-host:8080/ ").as_deref(), Some("http://cell-host:8080"));
        assert_eq!(normalize("https://example.com/").as_deref(), Some("https://example.com"));
        assert_eq!(normalize(""), None);
        assert_eq!(normalize("ftp://x"), None);
        assert_eq!(normalize("a b"), None);
    }

    #[test]
    fn adds_workspaces() {
        assert_eq!(with_workspace("203.0.113.10", " jdoe ").as_deref(), Ok("http://203.0.113.10/workspace/jdoe"));
        assert_eq!(with_workspace("203.0.113.10", "").as_deref(), Ok("http://203.0.113.10"));
        assert_eq!(with_workspace("http://h/workspace/a", "b").as_deref(), Ok("http://h/workspace/a"));
        assert!(with_workspace("h", "a/b").is_err());
        assert_eq!(label("http://h:8080/workspace/a"), "http://h:8080 · a");
        assert_eq!(label("http://h:8080"), "http://h:8080");
    }

    #[test]
    fn built_in_entries() {
        assert!(is_preset(LOCAL));
        assert!(!is_preset("http://203.0.113.10"));
        #[cfg(not(feature = "internal"))]
        {
            assert!(CLOUD.is_none(), "the public build has no built-in cloud backend");
            assert!(LOGIN_TEXTS.hint.is_none(), "nor a silent login to hint at");
        }
    }
}
