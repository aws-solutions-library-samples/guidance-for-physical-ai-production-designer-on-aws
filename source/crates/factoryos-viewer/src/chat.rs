//! The Agentic Planning chat pane, for a planner that runs in the backend.
//!
//! The agent itself runs in the backend (`kiro-cli acp`, see its `services/chat.rs`); this is the
//! transcript, the input box and the permission prompts. The wasm viewer always uses it; the native
//! one when the backend is on another machine (with a local one, Rerun's own agent panel runs
//! `kiro-cli` instead).
//!
//! It is laid out like Kiro Crew's chat: a header (the shown chat's title, + New), a sessions column
//! (☰), tabs for the open sessions (shown with two or more), the transcript (markdown, tool calls
//! as cards), and a composer card with the agent's state, Restart / Stop and Send.
//!
//! Every agent session in the backend is one [`ChatTab`] ([`Chats`]): every action that starts an
//! agent (Plan Cell, a step's Agent button, a diagnosis, a file on the code page) opens it in a new
//! tab with a fresh agent, so several planners run side by side. A tab's × only puts it away (it
//! stays in the sessions column, its agent keeps running); the column's × closes the session and
//! stops its agent. Each session keeps its own WebSocket, so the ones in the background keep up
//! while another is shown. The sessions live in the backend: a reloaded viewer lists them and
//! attaches to them again (the newest shown). With an older backend (no sessions API) there is one
//! tab, its single chat.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::chat::{ChatSocket, SocketState};
use factoryos_client::protocol::api;
use factoryos_client::protocol::chat::{
    ChatCommand, ChatEntry, ChatEvent, ChatRole, ChatSessionInfo, ChatSessions, ChatState, NewChatSession,
    PermissionOption, SessionStatus, ToolStatus, session_status,
};

use egui_commonmark::{CommonMarkCache, CommonMarkViewer};

use crate::rr::egui::{self, Color32, RichText};
use crate::theme;
use crate::workspace_ui::Workspaces;

type Wake = std::sync::Arc<dyn Fn() + Send + Sync>;
type SharedWorkspaces = std::rc::Rc<std::cell::RefCell<Workspaces>>;

/// How often the tabs are compared with the backend's sessions (another viewer may open or close
/// some).
const RELIST_SECS: u64 = 10;

/// What came back from the sessions API.
enum Reply {
    Listed { result: Result<ChatSessions, String>, sent: web_time::Instant },
    Created(Result<ChatSessionInfo, String>),
    Closed { id: String, result: Result<(), String> },
}

/// Every chat tab, one per agent session.
#[derive(Default)]
pub struct Chats {
    tabs: Vec<ChatTab>,
    active: usize,
    api: Option<Api>,
    wake: Option<Wake>,
    workspaces: Option<SharedWorkspaces>,
    replies: Arc<Mutex<Vec<Reply>>>,
    /// The backend has no sessions API: one tab on its single chat (`/chat`).
    legacy: bool,
    /// When the sessions were last asked for (`None`: never), and whether an answer is due.
    listed: Option<web_time::Instant>,
    listing: bool,
    /// Sessions asked for and not created yet (the tab appears with the backend's answer).
    creating: usize,
    max_sessions: Option<usize>,
    /// Errors of the pane itself, e.g. the session limit.
    errors: Vec<String>,
    /// The sessions column shown or hidden by the user (`None`: shown when the pane is wide).
    sessions_open: Option<bool>,
    /// The sessions column's search.
    search: String,
    /// The older finished sessions are listed too.
    show_older: bool,
}

impl Chats {
    pub fn set_workspaces(&mut self, workspaces: SharedWorkspaces) {
        self.workspaces = Some(workspaces);
    }

    /// Attach to the backend's sessions, if not attached already. `wake` repaints the UI.
    pub fn attach(&mut self, api: &Api, wake: impl Fn() + Send + Sync + 'static) {
        if self.api.as_ref().is_some_and(|known| known.base() == api.base()) {
            return;
        }
        self.api = Some(api.clone());
        self.wake = Some(std::sync::Arc::new(wake));
        self.list();
    }

    fn list(&mut self) {
        let (Some(api), Some(wake)) = (self.api.clone(), self.wake.clone()) else { return };
        if self.listing {
            return;
        }
        self.listing = true;
        let sent = web_time::Instant::now();
        self.listed = Some(sent);
        let replies = self.replies.clone();
        api.get(api::CHAT_SESSIONS, move |result| {
            replies.lock().unwrap().push(Reply::Listed { result, sent });
            wake();
        });
    }

    /// Open a new tab with a fresh agent, titled `title`; with `prompt`, the agent starts on it.
    pub fn start(&mut self, title: String, prompt: Option<String>) {
        self.errors.clear();
        if self.legacy {
            // One chat only: the prompt goes there, unless it is still busy with the last one.
            let Some(tab) = self.tabs.first_mut() else { return };
            self.active = 0;
            if let Some(prompt) = prompt {
                if tab.is_busy() {
                    tab.note(
                        "The planner is still working on the last request: answer its questions below, \
                         or press Stop first. (This backend runs one chat at a time; update it for tabs.)",
                    );
                    return;
                }
                tab.send_prompt(prompt);
            }
            return;
        }
        let (Some(api), Some(wake)) = (self.api.clone(), self.wake.clone()) else {
            self.errors.push("not connected to the backend".to_owned());
            return;
        };
        self.creating += 1;
        let replies = self.replies.clone();
        let request = NewChatSession { title, prompt };
        api.post_json(api::CHAT_SESSIONS, &request, move |result| {
            replies.lock().unwrap().push(Reply::Created(result));
            wake();
        });
    }

    /// Close tab `index` and stop its agent.
    fn close(&mut self, index: usize) {
        if index >= self.tabs.len() {
            return;
        }
        let tab = self.tabs.remove(index);
        if self.active > index || self.active >= self.tabs.len() {
            self.active = self.active.saturating_sub(1);
        }
        let (Some(id), Some(api), Some(wake)) = (tab.id.clone(), self.api.clone(), self.wake.clone()) else {
            return;
        };
        let replies = self.replies.clone();
        api.delete(&api::chat_session(&id), move |result: Result<(), String>| {
            replies.lock().unwrap().push(Reply::Closed { id, result });
            wake();
        });
    }

    fn add_tab(&mut self, info: &ChatSessionInfo, focus: bool) {
        if let Some(index) = self.tabs.iter().position(|tab| tab.id.as_deref() == Some(info.id.as_str())) {
            if focus {
                self.active = index;
            }
            return;
        }
        let mut tab = ChatTab::new(Some(info.id.clone()), info.title.clone(), self.workspaces.clone());
        tab.created_ms = info.created_ms;
        if let (Some(api), Some(wake)) = (&self.api, &self.wake) {
            let wake = wake.clone();
            tab.connect(api.base(), move || wake());
        }
        self.tabs.push(tab);
        if focus {
            self.active = self.tabs.len() - 1;
        }
    }

    /// Apply the sessions API's answers and every tab's events.
    fn poll(&mut self) {
        let replies = std::mem::take(&mut *self.replies.lock().unwrap());
        for reply in replies {
            match reply {
                Reply::Listed { result, sent } => {
                    self.listing = false;
                    match result {
                        Ok(list) => self.apply_list(&list, sent),
                        // An older backend: its one chat, as before.
                        Err(err) if err.starts_with("404") || err.starts_with("405") => {
                            if !self.legacy {
                                self.legacy = true;
                                let mut tab = ChatTab::new(None, "Chat".to_owned(), self.workspaces.clone());
                                if let (Some(api), Some(wake)) = (&self.api, &self.wake) {
                                    let wake = wake.clone();
                                    tab.connect(api.base(), move || wake());
                                }
                                self.tabs = vec![tab];
                                self.active = 0;
                            }
                        }
                        Err(err) => self.errors.push(format!("could not list the agent chats: {err}")),
                    }
                }
                Reply::Created(result) => {
                    self.creating = self.creating.saturating_sub(1);
                    match result {
                        Ok(info) => self.add_tab(&info, true),
                        Err(err) => self.errors.push(match err.split_once(": ") {
                            Some((status, reason)) if status.starts_with("409") => reason.to_owned(),
                            _ => format!("could not open an agent chat: {err}"),
                        }),
                    }
                }
                Reply::Closed { id, result } => {
                    if let Err(err) = result
                        && !err.starts_with("404")
                    {
                        self.errors.push(format!("closing chat {id}: {err}"));
                    }
                }
            }
        }
        for tab in &mut self.tabs {
            tab.poll();
        }
        // Closed elsewhere (another viewer, or the backend restarted).
        if self.tabs.iter().any(|tab| tab.closed) {
            let active_id = self.tabs.get(self.active).and_then(|tab| tab.id.clone());
            self.tabs.retain(|tab| !tab.closed);
            self.active = active_id
                .and_then(|id| self.tabs.iter().position(|tab| tab.id.as_deref() == Some(&id)))
                .unwrap_or(self.active)
                .min(self.tabs.len().saturating_sub(1));
        }
        if !self.legacy && self.listed.is_some_and(|at| at.elapsed() > web_time::Duration::from_secs(RELIST_SECS)) {
            self.list();
        }
    }

    /// Tabs for the sessions the backend has (opened before a reload, or by another viewer); tabs
    /// of sessions it no longer has, and had when it was asked, go.
    fn apply_list(&mut self, list: &ChatSessions, sent: web_time::Instant) {
        self.max_sessions = Some(list.max_sessions);
        // Shown: the tab shown so far, else (a reload) the newest session.
        let active_id = self.tabs.get(self.active).and_then(|tab| tab.id.clone());
        let first_listing = self.tabs.is_empty();
        for info in &list.sessions {
            self.add_tab(info, false);
        }
        self.tabs.retain(|tab| {
            tab.added > sent || tab.id.as_ref().is_none_or(|id| list.sessions.iter().any(|s| &s.id == id))
        });
        self.active = active_id
            .and_then(|id| self.tabs.iter().position(|tab| tab.id.as_deref() == Some(&id)))
            .unwrap_or(if first_listing { self.tabs.len().saturating_sub(1) } else { self.active })
            .min(self.tabs.len().saturating_sub(1));
    }

    /// The pane: the sessions column (Kiro Crew's sidebar; shown by default when the pane is wide
    /// enough) and the shown chat with its header; `true` when the user hid the pane.
    pub fn ui(&mut self, ui: &mut egui::Ui) -> bool {
        self.poll();
        if self.tabs.iter().any(|tab| tab.status() == SessionStatus::Running) {
            // The busy sessions' dots pulse.
            ui.ctx().request_repaint_after(web_time::Duration::from_millis(50));
        }
        self.shortcuts(ui);
        let list = !self.legacy && self.sessions_open.unwrap_or(ui.available_width() >= SESSIONS_AUTO_WIDTH);
        if list {
            let p = theme::palette(ui);
            egui::Panel::left("chat_sessions")
                .resizable(false)
                .show_separator_line(false)
                .exact_size(SESSIONS_WIDTH.min(ui.available_width() * 0.5))
                .frame(
                    egui::Frame::new()
                        .fill(p.card)
                        .stroke(egui::Stroke::new(1.0, p.border))
                        .corner_radius(12)
                        .outer_margin(egui::Margin { left: 6, right: 2, top: 6, bottom: 6 })
                        .inner_margin(egui::Margin::symmetric(8, 8)),
                )
                .show(ui, |ui| self.sessions_ui(ui));
        }
        let mut hide = false;
        egui::CentralPanel::no_frame().show(ui, |ui| {
            hide = self.header_ui(ui, list);
            for message in &self.errors {
                notice_ui(ui, theme::palette(ui).danger, theme::palette(ui).danger_subtle, message);
            }
            match self.tabs.get_mut(self.active) {
                Some(tab) => tab.ui(ui),
                None if self.creating > 0 => {
                    ui.add_space(12.0);
                    ui.horizontal(|ui| {
                        ui.add_space(12.0);
                        ui.spinner();
                        ui.label(RichText::new("Starting a new agent…").color(theme::palette(ui).muted));
                    });
                }
                None => {
                    ui.add_space(24.0);
                    ui.vertical_centered(|ui| {
                        ui.label(RichText::new("No agent chat open").size(14.0).color(theme::palette(ui).text_strong));
                        ui.label(
                            RichText::new("Plan Cell and the steps' Agent buttons each open one; so does + New.")
                                .color(theme::palette(ui).muted),
                        );
                    });
                }
            }
        });
        hide
    }

    /// Cmd/Ctrl+1…9 shows the session with that number in the list.
    fn shortcuts(&mut self, ui: &mut egui::Ui) {
        const KEYS: [egui::Key; 9] = [
            egui::Key::Num1,
            egui::Key::Num2,
            egui::Key::Num3,
            egui::Key::Num4,
            egui::Key::Num5,
            egui::Key::Num6,
            egui::Key::Num7,
            egui::Key::Num8,
            egui::Key::Num9,
        ];
        let order = self.display_order();
        for (n, key) in KEYS.into_iter().enumerate() {
            if ui.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, key))
                && let Some(&index) = order.get(n)
            {
                self.active = index;
            }
        }
    }

    /// The sessions as the list shows them: newest first.
    fn display_order(&self) -> Vec<usize> {
        (0..self.tabs.len()).rev().collect()
    }

    /// The chat's title row: the shown chat's title and hide; with the sessions column hidden, its
    /// toggle and + New too.
    fn header_ui(&mut self, ui: &mut egui::Ui, list: bool) -> bool {
        let p = theme::palette(ui);
        let mut hide = false;
        let mut new = false;
        egui::Frame::new().inner_margin(egui::Margin { left: 8, right: 8, top: 6, bottom: 6 }).show(ui, |ui| {
            ui.horizontal(|ui| {
                if !self.legacy && !list && ui.add(icon_button("☰", false, p)).on_hover_text("Show the sessions").clicked() {
                    self.sessions_open = Some(true);
                }
                let title = self.tabs.get(self.active).map_or("Agentic Planning", |tab| tab.title.as_str()).to_owned();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.add(icon_button("×", false, p)).on_hover_text("Hide the chat (the agents keep running)").clicked() {
                        hide = true;
                    }
                    if !self.legacy && !list {
                        new = self.new_button(ui, p);
                    }
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.add(egui::Label::new(RichText::new(title).size(14.0).strong().color(p.text_strong)).truncate());
                    });
                });
            });
        });
        separator(ui, p);
        if new {
            self.start("Chat".to_owned(), None);
        }
        hide
    }

    /// The accent "+ New" button; whether it was clicked.
    fn new_button(&self, ui: &mut egui::Ui, p: &theme::Palette) -> bool {
        let full = self.max_sessions.is_some_and(|max| self.tabs.len() + self.creating >= max);
        let hover = match self.max_sessions {
            Some(max) => format!("A new chat with a fresh agent ({} of {max} open)", self.tabs.len()),
            None => "A new chat with a fresh agent".to_owned(),
        };
        let button = egui::Button::new(RichText::new("+ New").color(p.accent_fg).strong())
            .fill(p.accent)
            .corner_radius(8)
            .min_size(egui::vec2(64.0, 26.0));
        ui.add_enabled(!full, button)
            .on_hover_text(hover)
            .on_disabled_hover_text("The backend's limit of agent chats is reached: close one first")
            .clicked()
    }

    /// Kiro Crew's session sidebar: its header (collapse, "Sessions", + New), a search, and every
    /// session newest first; the long-finished ones folded under "older".
    fn sessions_ui(&mut self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        let mut new = false;
        ui.horizontal(|ui| {
            if ui.add(icon_button("☰", true, p)).on_hover_text("Hide the sessions").clicked() {
                self.sessions_open = Some(false);
            }
            ui.label(RichText::new("Sessions").size(15.0).strong().color(p.text_strong));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                new = self.new_button(ui, p);
            });
        });
        ui.add_space(6.0);
        let search_id = ui.id().with("chat_sessions_search");
        let focused = ui.memory(|m| m.focused()) == Some(search_id);
        egui::Frame::new()
            .fill(p.bg)
            .stroke(egui::Stroke::new(1.0, if focused { p.accent } else { p.border }))
            .corner_radius(8)
            .inner_margin(egui::Margin::symmetric(8, 4))
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.search)
                        .id(search_id)
                        .frame(egui::Frame::NONE)
                        .hint_text("🔍 Search sessions…")
                        .desired_width(f32::INFINITY),
                );
            });
        ui.add_space(6.0);

        let workspace = self
            .workspaces
            .as_ref()
            .and_then(|workspaces| workspaces.borrow().selected().map(str::to_owned))
            .unwrap_or_else(|| "this backend".to_owned());
        let numbers = repeat_numbers(self.tabs.iter().map(|tab| tab.title.as_str()));
        let words: Vec<String> = self.search.split_whitespace().map(str::to_lowercase).collect();
        let now = unix_ms();
        let time = ui.input(|i| i.time);
        let order = self.display_order();
        // Older: past the first few, finished or never started, and an hour old.
        let older = |index: usize, position: usize| {
            let tab = &self.tabs[index];
            position >= RECENT_SESSIONS
                && matches!(tab.status(), SessionStatus::Done | SessionStatus::Idle)
                && now.saturating_sub(tab.created_ms) > 3_600_000
        };
        let older_count = order.iter().enumerate().filter(|&(position, &index)| older(index, position)).count();
        let mut show = None;
        let mut close = None;
        egui::ScrollArea::vertical().id_salt("chat_sessions_list").auto_shrink([false, false]).show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            for (position, &index) in order.iter().enumerate() {
                let tab = &self.tabs[index];
                let title = match numbers[index] {
                    1 => tab.title.clone(),
                    n => format!("{} ({n})", tab.title),
                };
                let title_lower = title.to_lowercase();
                if !words.iter().all(|word| title_lower.contains(word)) {
                    continue;
                }
                if words.is_empty() && !self.show_older && older(index, position) {
                    continue;
                }
                let row = SessionRow {
                    title: &title,
                    workspace: &workspace,
                    status: tab.status(),
                    age: age_text(now.saturating_sub(tab.created_ms)),
                    number: (position < 9).then_some(position + 1),
                    selected: index == self.active,
                };
                let (clicked, closed) = row.ui(ui, time, p);
                if closed {
                    close = Some(index);
                } else if clicked {
                    show = Some(index);
                }
            }
            if words.is_empty() && older_count > 0 {
                ui.add_space(6.0);
                let text = if self.show_older { "Hide older sessions".to_owned() } else { format!("Show {older_count} older sessions") };
                if ui.add(egui::Button::new(RichText::new(text).color(p.muted)).small()).clicked() {
                    self.show_older = !self.show_older;
                }
            }
        });
        if let Some(index) = close {
            self.close(index);
        } else if let Some(index) = show {
            self.active = index;
        }
        if new {
            self.start("Chat".to_owned(), None);
        }
    }
}

/// Width of the sessions column …
const SESSIONS_WIDTH: f32 = 230.0;
/// … shown by default when the pane is at least this wide (else behind ☰).
const SESSIONS_AUTO_WIDTH: f32 = 560.0;
/// The sessions always listed; older finished ones past these fold away.
const RECENT_SESSIONS: usize = 6;

/// One session of the sidebar, laid out as Kiro Crew's: a meta line (state dot, workspace, what it
/// does, age), its title, and its Cmd+number.
struct SessionRow<'a> {
    title: &'a str,
    workspace: &'a str,
    status: SessionStatus,
    age: String,
    number: Option<usize>,
    selected: bool,
}

impl SessionRow<'_> {
    /// Returns (clicked, its × clicked).
    fn ui(&self, ui: &mut egui::Ui, time: f64, p: &theme::Palette) -> (bool, bool) {
        let (dot, what) = status_look(self.status, p);
        let dot = if self.status == SessionStatus::Running {
            dot.gamma_multiply(0.55 + 0.45 * (time * 4.0).sin().abs() as f32)
        } else {
            dot
        };
        let width = ui.available_width();
        let (rect, response) = ui.allocate_exact_size(egui::vec2(width, 50.0), egui::Sense::click());
        let painter = ui.painter_at(rect);
        if self.selected {
            painter.rect_filled(rect, 8, p.hover);
            let bar = egui::Rect::from_min_size(rect.min + egui::vec2(0.0, 10.0), egui::vec2(3.0, rect.height() - 20.0));
            painter.rect_filled(bar, 2, p.accent);
        } else if response.hovered() {
            painter.rect_filled(rect, 8, p.hover.gamma_multiply(0.6));
        }
        let left = rect.left() + 12.0;
        let right = rect.right() - 10.0;
        let text = |painter: &egui::Painter, s: &str, font: egui::FontId, color: Color32, max: f32| {
            let mut job = egui::text::LayoutJob::simple_singleline(s.to_owned(), font, color);
            job.wrap = egui::text::TextWrapping::truncate_at_width(max.max(10.0));
            painter.layout_job(job)
        };
        // Meta line: ● workspace · what … age.
        let meta_y = rect.top() + 14.0;
        let age = text(&painter, &self.age, egui::FontId::proportional(10.5), p.muted, 60.0);
        let age_width = age.size().x;
        painter.galley(egui::pos2(right - age_width, meta_y - age.size().y / 2.0), age, p.muted);
        painter.circle_filled(egui::pos2(left + 3.0, meta_y), 3.0, dot);
        let meta = text(
            &painter,
            &format!("{} · {what}", self.workspace),
            egui::FontId::proportional(10.5),
            p.muted,
            right - age_width - 8.0 - (left + 12.0),
        );
        painter.galley(egui::pos2(left + 12.0, meta_y - meta.size().y / 2.0), meta, p.muted);
        // Title, and the number (× on hover).
        let title_y = rect.top() + 34.0;
        let title_color = if self.selected { p.text_strong } else { p.text };
        let title = text(&painter, self.title, egui::FontId::proportional(13.5), title_color, right - left - 26.0);
        painter.galley(egui::pos2(left, title_y - title.size().y / 2.0), title, title_color);
        let badge = egui::Rect::from_center_size(egui::pos2(right - 8.0, title_y), egui::vec2(18.0, 18.0));
        let mut closed = false;
        if response.hovered() || ui.rect_contains_pointer(badge) {
            let x = ui.place(badge, egui::Button::new(RichText::new("×").color(p.muted)).frame(false));
            closed = x.on_hover_text("Close this session and stop its agent").clicked();
        } else if let Some(number) = self.number {
            painter.rect(badge, 4, p.bg, egui::Stroke::new(1.0, p.border), egui::StrokeKind::Inside);
            let digit = text(&painter, &number.to_string(), egui::FontId::monospace(10.5), p.muted, 18.0);
            painter.galley(badge.center() - digit.size() / 2.0, digit, p.muted);
        }
        let shortcut = self.number.map(|n| format!("\n{}+{n} shows it", if cfg!(target_os = "macos") { "Cmd" } else { "Ctrl" }));
        let clicked = response
            .on_hover_text(format!("{} — {what}{}", self.title, shortcut.unwrap_or_default()))
            .clicked();
        (clicked, closed)
    }
}

/// A session's dot colour and what it means.
fn status_look(status: SessionStatus, p: &theme::Palette) -> (Color32, &'static str) {
    match status {
        SessionStatus::Idle => (p.muted_strong, "not started"),
        SessionStatus::Running => (p.accent, "running"),
        SessionStatus::Waiting => (p.warn, "waiting for you"),
        SessionStatus::Done => (p.ok, "done"),
        SessionStatus::Error => (p.danger, "error"),
    }
}

/// "now", "5m", "3h", "2d": how long ago, from milliseconds.
fn age_text(ms: u64) -> String {
    let minutes = ms / 60_000;
    match minutes {
        0 => "now".to_owned(),
        1..60 => format!("{minutes}m"),
        60..1440 => format!("{}h", minutes / 60),
        _ => format!("{}d", minutes / 1440),
    }
}

fn unix_ms() -> u64 {
    web_time::SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64)
}

/// A frameless icon-sized button, accent when `on`.
fn icon_button(icon: &str, on: bool, p: &theme::Palette) -> egui::Button<'static> {
    let color = if on { p.accent } else { p.muted };
    egui::Button::new(RichText::new(icon.to_owned()).size(14.0).color(color))
        .frame(false)
        .min_size(egui::vec2(22.0, 22.0))
}

fn separator(ui: &mut egui::Ui, p: &theme::Palette) {
    let rect = ui.available_rect_before_wrap();
    ui.painter().hline(rect.x_range(), rect.top(), egui::Stroke::new(1.0, p.border));
    ui.add_space(1.0);
}

/// A tinted, bordered line of text: errors, notes.
fn notice_ui(ui: &mut egui::Ui, color: Color32, fill: Color32, text: &str) {
    egui::Frame::new()
        .fill(fill)
        .stroke(egui::Stroke::new(1.0, color.gamma_multiply(0.5)))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(10, 6))
        .outer_margin(egui::Margin::symmetric(8, 3))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.add(egui::Label::new(RichText::new(text).color(color)).wrap());
        });
}

/// Per tab, the how-manieth with its title it is (1 for the first): repeats show `title (2)`, ….
fn repeat_numbers<'a>(titles: impl Iterator<Item = &'a str>) -> Vec<usize> {
    let mut seen: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
    titles
        .map(|title| {
            let n = seen.entry(title).or_insert(0);
            *n += 1;
            *n
        })
        .collect()
}

/// A permission request waiting for the user.
struct Permission {
    id: String,
    title: String,
    options: Vec<PermissionOption>,
}

/// One tab: one agent session in the backend.
pub struct ChatTab {
    /// The backend's session; `None`: its single chat (an older backend).
    id: Option<String>,
    title: String,
    added: web_time::Instant,
    /// The backend closed the session (here or in another viewer).
    closed: bool,
    socket: Option<ChatSocket>,
    /// Why there is no socket, if connecting failed.
    connect_error: Option<String>,
    state: ChatState,
    agent: Option<String>,
    entries: Vec<ChatEntry>,
    permissions: Vec<Permission>,
    input: String,
    /// Scroll to the end while new text streams in, unless the user scrolled up.
    follow: bool,
    errors: Vec<String>,
    /// Where the socket connects, to reconnect after it closed.
    base: Option<(String, Wake)>,
    /// For the agent's sign-in, offered right here when it needs one.
    workspaces: Option<SharedWorkspaces>,
    /// The agent was signed in after it had stopped for want of a sign-in: restarted once.
    restarted_after_sign_in: bool,
    /// When the backend created it (Unix milliseconds).
    created_ms: u64,
    /// The agent's messages, parsed as markdown.
    markdown: CommonMarkCache,
}

impl ChatTab {
    fn new(id: Option<String>, title: String, workspaces: Option<SharedWorkspaces>) -> Self {
        Self {
            id,
            title,
            added: web_time::Instant::now(),
            closed: false,
            socket: None,
            connect_error: None,
            state: ChatState::Idle,
            agent: None,
            entries: Vec::new(),
            permissions: Vec::new(),
            input: String::new(),
            follow: true,
            errors: Vec::new(),
            base: None,
            workspaces,
            restarted_after_sign_in: false,
            created_ms: 0,
            markdown: CommonMarkCache::default(),
        }
    }

    /// Connect to the backend's session, if not connected already. `wake` repaints the UI.
    fn connect(&mut self, base: &str, wake: impl Fn() + Send + Sync + 'static) {
        if self.socket.is_some() {
            return;
        }
        let wake: Wake = std::sync::Arc::new(wake);
        self.base = Some((base.to_owned(), wake.clone()));
        match ChatSocket::connect_session(base, self.id.as_deref(), move || wake()) {
            Ok(socket) => {
                self.socket = Some(socket);
                self.connect_error = None;
            }
            Err(err) => self.connect_error = Some(err),
        }
    }

    /// Drop a closed socket and open a new one (a fresh snapshot of the chat comes with it).
    fn reconnect(&mut self) {
        let Some((base, wake)) = self.base.clone() else { return };
        self.socket = None;
        self.connect(&base, move || wake());
    }

    /// Apply everything that arrived since the last frame.
    fn poll(&mut self) {
        let Some(socket) = self.socket.as_mut() else { return };
        for event in socket.drain() {
            self.apply(event);
        }
    }

    fn status(&self) -> SessionStatus {
        if self.socket.as_ref().map(ChatSocket::state) == Some(SocketState::Closed) || self.connect_error.is_some() {
            return SessionStatus::Error;
        }
        session_status(self.state, !self.permissions.is_empty(), &self.entries)
    }

    fn apply(&mut self, event: ChatEvent) {
        match event {
            ChatEvent::State { state, agent } => {
                self.state = state;
                self.agent = agent;
            }
            ChatEvent::Transcript { entries } => {
                self.entries = entries;
                self.follow = true;
            }
            ChatEvent::Entry { entry } => {
                // The backend numbers entries, so a reconnect cannot duplicate them.
                let index = entry.index;
                if index < self.entries.len() {
                    self.entries[index] = entry;
                } else {
                    self.entries.push(entry);
                }
            }
            ChatEvent::Chunk { index, text } => {
                if let Some(entry) = self.entries.get_mut(index) {
                    entry.text.push_str(&text);
                }
            }
            ChatEvent::ToolUpdate { index, status, text } => {
                if let Some(entry) = self.entries.get_mut(index) {
                    entry.tool_status = Some(status);
                    if let Some(text) = text {
                        entry.text = text;
                    }
                }
            }
            ChatEvent::TurnEnd { .. } => {}
            ChatEvent::Permission { id, title, options } => {
                self.permissions.retain(|pending| pending.id != id);
                self.permissions.push(Permission { id, title, options });
            }
            ChatEvent::PermissionResolved { id } => self.permissions.retain(|pending| pending.id != id),
            ChatEvent::Error { message } => self.errors.push(message),
            ChatEvent::SessionClosed => self.closed = true,
        }
    }

    fn send(&mut self, command: &ChatCommand) {
        let Some(socket) = self.socket.as_ref() else {
            self.errors.push("not connected to the backend's chat".to_owned());
            return;
        };
        if let Err(err) = socket.send(command) {
            self.errors.push(err);
        }
    }

    /// Whether the agent is already starting or answering a prompt.
    fn is_busy(&self) -> bool {
        matches!(self.state, ChatState::Starting | ChatState::Thinking)
    }

    /// A line from the viewer itself, shown with the errors (e.g. why a click did nothing).
    fn note(&mut self, text: impl Into<String>) {
        self.errors.push(text.into());
    }

    /// Send `text` as if the user had typed it (the Plan prompt does this).
    fn send_prompt(&mut self, text: String) {
        self.send(&ChatCommand::Prompt { text });
        self.follow = true;
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        if let Some(err) = &self.connect_error {
            notice_ui(ui, p.danger, p.danger_subtle, &format!("Chat unavailable: {err}"));
            return;
        }
        if self.socket.as_ref().map(ChatSocket::state) == Some(SocketState::Closed) {
            let mut reconnect = false;
            ui.horizontal(|ui| {
                ui.add_space(8.0);
                ui.label(RichText::new("The chat connection closed.").color(p.danger));
                reconnect = ui.button("Reconnect").clicked();
            });
            if reconnect {
                self.reconnect();
            }
        }
        // Whether the agent is signed in, asked as soon as the chat is open (not only after a prompt
        // failed), re-checked every 30 s, and every 3 s while it needs one.
        let max_age = if self.state == ChatState::AuthRequired { 3 } else { 30 };
        let signed_in = self
            .workspaces
            .as_ref()
            .and_then(|workspaces| workspaces.borrow_mut().agent_signed_in(web_time::Duration::from_secs(max_age)));
        if self.state == ChatState::AuthRequired && signed_in == Some(true) {
            // Signed in meanwhile: start the agent again, rather than asking for Restart.
            if !self.restarted_after_sign_in {
                self.restarted_after_sign_in = true;
                self.entries.clear();
                self.permissions.clear();
                self.errors.clear();
                self.send(&ChatCommand::Restart);
            }
        } else if self.state != ChatState::AuthRequired {
            self.restarted_after_sign_in = false;
        }
        if self.state == ChatState::AuthRequired || signed_in == Some(false) {
            egui::Frame::new()
                .fill(p.warn_subtle)
                .stroke(egui::Stroke::new(1.0, p.warn.gamma_multiply(0.5)))
                .corner_radius(8)
                .inner_margin(egui::Margin::symmetric(10, 8))
                .outer_margin(egui::Margin::symmetric(8, 4))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(RichText::new("⚠ The planner agent (kiro-cli) is not signed in. Sign it in to plan:").color(p.warn));
                    if let Some(workspaces) = &self.workspaces {
                        workspaces.borrow_mut().agent_ui(ui);
                    }
                });
        }

        // Questions and the composer are pinned to the bottom, so they stay visible however long
        // the transcript or a question (e.g. a whole script to run) is.
        let max_bottom = ui.available_height() * 0.6;
        egui::Panel::bottom("chat_input")
            .frame(egui::Frame::new().inner_margin(egui::Margin { left: 8, right: 8, top: 4, bottom: 6 }))
            .resizable(false)
            .show_separator_line(false)
            .show(ui, |ui| {
                ui.set_max_height(max_bottom);
                self.permissions_ui(ui);
                if self.entries.is_empty() {
                    self.suggestions_ui(ui);
                }
                self.composer_ui(ui);
                // The workspace and agent binary: Advanced only.
                crate::ui_mode::advanced(ui, |ui| self.footer_ui(ui));
            });
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .stick_to_bottom(self.follow)
            .show(ui, |ui| {
                egui::Frame::new().inner_margin(egui::Margin { left: 12, right: 12, top: 8, bottom: 8 }).show(ui, |ui| {
                    // Everything wraps: a line wider than the pane (a long path in a tool call)
                    // would hold the panel at its widest, undoing any resize, and be clipped on its
                    // left edge.
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Wrap);
                    // Kiro Crew's message text: 13.5 px, a little looser than the panels.
                    for (style, size) in [(egui::TextStyle::Body, 13.5), (egui::TextStyle::Monospace, 12.5)] {
                        if let Some(font) = ui.style_mut().text_styles.get_mut(&style) {
                            font.size = size;
                        }
                    }
                    ui.spacing_mut().item_spacing.y = 6.0;
                    if self.entries.is_empty() {
                        ui.add_space(16.0);
                        ui.label(RichText::new("Ask the planner about the active cell").size(15.0).color(p.text_strong));
                        ui.label(RichText::new("e.g. “plan the welding cell”. It runs in the backend, with the cell's tools.").color(p.muted));
                    }
                    let markdown = &mut self.markdown;
                    for row in fold(&self.entries) {
                        match row {
                            Row::One(entry) => entry_ui(ui, entry, markdown, p),
                            Row::Repeat { entries, calls, last, note } => repeat_ui(ui, entries, calls, last, note, markdown, p),
                        }
                    }
                    if self.state == ChatState::Thinking {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label(RichText::new("Thinking…").color(p.accent));
                        });
                    }
                    for message in &self.errors {
                        ui.add(egui::Label::new(RichText::new(message).color(p.danger)).wrap());
                    }
                });
            });
    }

    /// Permission requests block the agent: a card each, what it asks and its answers.
    fn permissions_ui(&mut self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        let mut answered = None;
        for (index, pending) in self.permissions.iter().enumerate() {
            egui::Frame::new()
                .fill(p.warn_subtle)
                .stroke(egui::Stroke::new(1.0, p.warn.gamma_multiply(0.6)))
                .corner_radius(8)
                .inner_margin(egui::Margin::symmetric(10, 8))
                .outer_margin(egui::Margin { bottom: 6, ..Default::default() })
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(RichText::new("⚠ The agent asks for permission").strong().color(p.warn));
                    egui::ScrollArea::vertical()
                        .id_salt(("permission", index))
                        .max_height(90.0)
                        .show(ui, |ui| {
                            ui.add(egui::Label::new(RichText::new(&pending.title).monospace().color(p.text)).wrap());
                        });
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().button_padding = egui::vec2(10.0, 4.0);
                        let mut first_allow = true;
                        for option in &pending.options {
                            let allow = option.kind.as_deref().is_some_and(|kind| kind.starts_with("allow"));
                            let reject = option.kind.as_deref().is_some_and(|kind| kind.starts_with("reject"));
                            let button = if allow && std::mem::take(&mut first_allow) {
                                egui::Button::new(RichText::new(&option.name).color(p.accent_fg)).fill(p.accent)
                            } else {
                                let color = if reject { p.danger } else { p.text };
                                egui::Button::new(RichText::new(&option.name).color(color))
                                    .fill(Color32::TRANSPARENT)
                                    .stroke(egui::Stroke::new(1.0, if reject { p.danger.gamma_multiply(0.6) } else { p.border_strong }))
                            };
                            if ui.add(button.corner_radius(6)).clicked() {
                                answered = Some((pending.id.clone(), option.id.clone()));
                            }
                        }
                    });
                });
        }
        if let Some((id, option_id)) = answered {
            self.send(&ChatCommand::Permission { id, option_id });
        }
    }

    /// Starting points for an empty chat, as chips above the composer (Kiro Crew's follow-ups).
    fn suggestions_ui(&mut self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        let mut chosen = None;
        egui::ScrollArea::horizontal().id_salt("chat_suggestions").auto_shrink([false, true]).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().button_padding = egui::vec2(10.0, 5.0);
                for prompt in SUGGESTIONS {
                    let chip = egui::Button::new(RichText::new(format!("{prompt}  ↑")).color(p.text))
                        .fill(p.card)
                        .stroke(egui::Stroke::new(1.0, p.border))
                        .corner_radius(8);
                    if ui.add(chip).on_hover_text("Send this").clicked() {
                        chosen = Some(prompt.to_string());
                    }
                }
            });
        });
        ui.add_space(4.0);
        if let Some(prompt) = chosen {
            self.send_prompt(prompt);
        }
    }

    /// The message box in a rounded card, with the agent's state, Restart / Stop and Send.
    fn composer_ui(&mut self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        let mut send = false;
        let focused = ui.memory(|m| m.focused()) == Some(ui.id().with("chat_composer"));
        egui::Frame::new()
            .fill(p.card)
            .stroke(egui::Stroke::new(1.0, if focused { p.accent.gamma_multiply(0.7) } else { p.border }))
            .corner_radius(12)
            .inner_margin(egui::Margin { left: 12, right: 8, top: 8, bottom: 6 })
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                let response = ui.add(
                    egui::TextEdit::multiline(&mut self.input)
                        .id(ui.id().with("chat_composer"))
                        .hint_text("Message the planner…  (Enter sends, Shift+Enter: new line)")
                        .frame(egui::Frame::NONE)
                        .desired_rows(2)
                        .desired_width(f32::INFINITY),
                );
                // Enter sends; Shift+Enter makes a new line.
                send |= response.has_focus() && ui.input(|input| input.key_pressed(egui::Key::Enter) && !input.modifiers.shift);
                ui.horizontal(|ui| {
                    let (label, color) = match self.state {
                        ChatState::Idle => ("Idle", p.muted),
                        ChatState::Starting => ("Starting…", p.warn),
                        ChatState::Ready => ("Ready", p.ok),
                        ChatState::Thinking => ("Thinking…", p.accent),
                        ChatState::AuthRequired => ("Sign-in needed", p.warn),
                        ChatState::Failed => ("Failed", p.danger),
                    };
                    egui::Frame::new()
                        .stroke(egui::Stroke::new(1.0, p.border))
                        .corner_radius(10)
                        .inner_margin(egui::Margin::symmetric(8, 2))
                        .show(ui, |ui| {
                            ui.label(RichText::new(format!("● {label}")).small().color(color));
                        });
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let can_send = !self.input.trim().is_empty();
                        let arrow = egui::Button::new(RichText::new("↑").strong().color(p.accent_fg))
                            .fill(if can_send { p.accent } else { p.accent.gamma_multiply(0.4) })
                            .corner_radius(14)
                            .min_size(egui::vec2(28.0, 28.0));
                        send |= ui.add_enabled(can_send, arrow).on_hover_text("Send (Enter)").clicked();
                        if self.state == ChatState::Thinking
                            && ui.add(icon_button("■", false, p)).on_hover_text("Stop the agent's turn").clicked()
                        {
                            self.send(&ChatCommand::Cancel);
                        }
                        if ui.add(icon_button("⟲", false, p)).on_hover_text("Stop this tab's agent and clear its chat").clicked() {
                            self.entries.clear();
                            self.permissions.clear();
                            self.errors.clear();
                            self.send(&ChatCommand::Restart);
                        }
                    });
                });
            });
        if send && !self.input.trim().is_empty() {
            let text = std::mem::take(&mut self.input).trim().to_owned();
            self.send_prompt(text);
        }
    }

    /// Under the composer, in mono: the workspace and the agent.
    fn footer_ui(&self, ui: &mut egui::Ui) {
        let p = theme::palette(ui);
        let workspace = self
            .workspaces
            .as_ref()
            .and_then(|workspaces| workspaces.borrow().selected().map(str::to_owned))
            .unwrap_or_else(|| "this backend".to_owned());
        let agent = self.agent.as_deref().unwrap_or("kiro-cli");
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            ui.add_space(4.0);
            ui.add(egui::Label::new(RichText::new(format!("{workspace}  ·  {agent}")).monospace().small().color(p.muted)).truncate());
        });
    }
}

/// What an empty chat offers to start with.
const SUGGESTIONS: [&str; 3] = ["Plan the active cell", "Explain the active cell's task program", "What can you do here?"];

/// The transcript as shown: a run of the same tool call (an agent polling `plan_status`), with the
/// agent's one-line remarks between the calls, is one row.
#[derive(Debug)]
enum Row<'a> {
    One(&'a ChatEntry),
    Repeat { entries: &'a [ChatEntry], calls: usize, last: &'a ChatEntry, note: Option<&'a ChatEntry> },
}

/// A remark short enough to be polling chatter ("Still planning. I'll keep polling.").
const REMARK_MAX: usize = 200;

fn fold(entries: &[ChatEntry]) -> Vec<Row<'_>> {
    let same_tool = |e: &ChatEntry, text: &str| e.role == ChatRole::Tool && e.text == text;
    let mut rows = Vec::new();
    let mut i = 0;
    while i < entries.len() {
        let first = &entries[i];
        if first.role != ChatRole::Tool {
            rows.push(Row::One(first));
            i += 1;
            continue;
        }
        let (mut calls, mut last, mut note, mut j) = (1, i, None, i + 1);
        while j < entries.len() {
            let e = &entries[j];
            if same_tool(e, &first.text) {
                calls += 1;
                last = j;
                j += 1;
            } else if matches!(e.role, ChatRole::Agent | ChatRole::Thought)
                && e.text.trim().len() <= REMARK_MAX
                && entries.get(j + 1).is_some_and(|n| same_tool(n, &first.text))
            {
                if e.role == ChatRole::Agent {
                    note = Some(j);
                }
                j += 1;
            } else {
                break;
            }
        }
        if calls < 2 {
            rows.push(Row::One(first));
            i += 1;
        } else {
            rows.push(Row::Repeat {
                entries: &entries[i..=last],
                calls,
                last: &entries[last],
                note: note.map(|n| &entries[n]),
            });
            i = last + 1;
        }
    }
    rows
}

fn repeat_ui(
    ui: &mut egui::Ui,
    entries: &[ChatEntry],
    calls: usize,
    last: &ChatEntry,
    note: Option<&ChatEntry>,
    markdown: &mut CommonMarkCache,
    p: &theme::Palette,
) {
    let id = ui.id().with(("chat-repeat", entries[0].index));
    tool_card(ui, id, last, Some(calls), p, |ui| {
        if let Some(note) = note {
            ui.add(egui::Label::new(RichText::new(format!("latest: {}", note.text.trim())).italics().color(p.muted)).wrap());
        }
        for entry in entries {
            entry_ui(ui, entry, markdown, p);
        }
    });
}

/// A tool call as a card (Kiro Crew's ToolCallLine): ▸, its state, what it does and on what (in
/// mono, cut to the line), ×N for a repeated call; a click unfolds the whole text and `body`.
fn tool_card(
    ui: &mut egui::Ui,
    id: egui::Id,
    entry: &ChatEntry,
    calls: Option<usize>,
    p: &theme::Palette,
    body: impl FnOnce(&mut egui::Ui),
) {
    let status = entry.tool_status.unwrap_or(ToolStatus::Pending);
    let (mark, color) = match status {
        ToolStatus::Pending | ToolStatus::InProgress => ("○", p.muted),
        ToolStatus::Completed => ("✓", p.ok),
        ToolStatus::Failed => ("×", p.danger),
    };
    // "Running: @router/plan_status" → "Running", "@router/plan_status".
    let text = entry.text.trim();
    let (verb, target) = match text.split_once(": ") {
        Some((verb, target)) if verb.len() <= 24 && !verb.contains('\n') => (verb, target),
        _ => (text, ""),
    };
    let open = ui.data(|d| d.get_temp::<bool>(id)).unwrap_or(false);
    let inner = egui::Frame::new()
        .fill(p.card)
        .stroke(egui::Stroke::new(1.0, p.border))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(10, 5))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                chevron(ui, open, p.muted);
                if status == ToolStatus::InProgress {
                    ui.add(egui::Spinner::new().size(12.0).color(p.accent));
                } else {
                    ui.label(RichText::new(mark).color(color));
                }
                ui.label(RichText::new(first_line(verb)).strong().color(p.text_strong));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(calls) = calls {
                        egui::Frame::new().fill(p.accent_subtle).corner_radius(8).inner_margin(egui::Margin::symmetric(6, 0)).show(ui, |ui| {
                            ui.label(RichText::new(format!("×{calls}")).small().color(p.accent));
                        });
                    }
                    if !target.is_empty() {
                        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                            egui::Frame::new().fill(p.bg).stroke(egui::Stroke::new(1.0, p.border)).corner_radius(4).inner_margin(egui::Margin::symmetric(6, 1)).show(ui, |ui| {
                                ui.add(egui::Label::new(RichText::new(first_line(target)).monospace().small().color(p.muted)).truncate());
                            });
                        });
                    }
                });
            });
            if open {
                ui.add_space(2.0);
                ui.add(egui::Label::new(RichText::new(text).monospace().small().color(p.text)).wrap());
                body(ui);
            }
        });
    let header = egui::Rect::from_min_size(inner.response.rect.min, egui::vec2(inner.response.rect.width(), 26.0));
    let response = ui.interact(header, id.with("header"), egui::Sense::click());
    if response.hovered() {
        ui.painter().rect_stroke(inner.response.rect, 8, egui::Stroke::new(1.0, p.border_strong), egui::StrokeKind::Inside);
    }
    if response.on_hover_cursor(egui::CursorIcon::PointingHand).clicked() {
        ui.data_mut(|d| d.insert_temp(id, !open));
    }
}

/// ▸ / ▾, painted (the fonts lack the glyphs).
fn chevron(ui: &mut egui::Ui, open: bool, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 12.0), egui::Sense::hover());
    let c = rect.center();
    let points = if open {
        vec![c + egui::vec2(-4.0, -2.0), c + egui::vec2(4.0, -2.0), c + egui::vec2(0.0, 3.0)]
    } else {
        vec![c + egui::vec2(-2.0, -4.0), c + egui::vec2(3.0, 0.0), c + egui::vec2(-2.0, 4.0)]
    };
    ui.painter().add(egui::Shape::convex_polygon(points, color, egui::Stroke::NONE));
}

fn first_line(text: &str) -> &str {
    text.lines().next().unwrap_or("")
}

fn entry_ui(ui: &mut egui::Ui, entry: &ChatEntry, markdown: &mut CommonMarkCache, p: &theme::Palette) {
    match entry.role {
        ChatRole::User => {
            ui.add_space(4.0);
            // Right-aligned, in a card (as Kiro Crew shows the user's messages).
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                egui::Frame::new()
                    .fill(p.card)
                    .stroke(egui::Stroke::new(1.0, p.border))
                    .corner_radius(12)
                    .inner_margin(egui::Margin::symmetric(12, 8))
                    .show(ui, |ui| {
                        ui.set_max_width(ui.available_width() * 0.85);
                        // The bubble is at the right; its text reads from the left.
                        ui.with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
                            ui.add(egui::Label::new(RichText::new(&entry.text).color(p.text_strong)).wrap());
                        });
                    });
            });
            ui.add_space(2.0);
        }
        ChatRole::Agent => {
            ui.scope(|ui| {
                ui.visuals_mut().override_text_color = Some(p.text);
                CommonMarkViewer::new().show(ui, markdown, &entry.text);
            });
        }
        ChatRole::Thought => {
            egui::CollapsingHeader::new(RichText::new("Thinking").italics().color(p.muted))
                .id_salt(("chat-thought", entry.index))
                .default_open(false)
                .show(ui, |ui| {
                    ui.add(egui::Label::new(RichText::new(&entry.text).italics().color(p.muted)).wrap());
                });
        }
        ChatRole::Tool => tool_card(ui, ui.id().with(("chat-tool", entry.index)), entry, None, p, |_| {}),
        ChatRole::Status => {
            ui.vertical_centered(|ui| {
                ui.add(egui::Label::new(RichText::new(&entry.text).small().color(p.muted)).wrap());
            });
        }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn repeated_tab_titles_are_numbered() {
        let titles = ["Plan a · u3", "Plan b · u3", "Plan a · u3", "Plan a · u3"];
        assert_eq!(repeat_numbers(titles.into_iter()), vec![1, 1, 2, 3]);
    }

    use super::*;

    fn e(index: usize, role: ChatRole, text: &str) -> ChatEntry {
        ChatEntry { index, role, text: text.into(), tool_status: Some(ToolStatus::Completed) }
    }

    fn info(id: &str, title: &str) -> ChatSessionInfo {
        ChatSessionInfo {
            id: id.into(),
            title: title.into(),
            state: ChatState::Idle,
            status: SessionStatus::Idle,
            agent: None,
            created_ms: 0,
        }
    }

    fn ids(chats: &Chats) -> Vec<&str> {
        chats.tabs.iter().map(|tab| tab.id.as_deref().unwrap_or("-")).collect()
    }

    #[test]
    fn a_reload_reattaches_to_the_backends_sessions() {
        let mut chats = Chats::default();
        let list = ChatSessions { sessions: vec![info("a", "Plan jam2 · umarell_u3"), info("b", "Layout jam2")], max_sessions: 6 };
        chats.apply_list(&list, web_time::Instant::now());
        assert_eq!(ids(&chats), ["a", "b"]);
        assert_eq!(chats.active, 1, "the newest is shown");
        assert_eq!(chats.max_sessions, Some(6));

        // Created here after the list was asked for: kept although the answer lacks it.
        let sent = web_time::Instant::now();
        chats.add_tab(&info("c", "Chat"), true);
        chats.active = 0;
        let list = ChatSessions { sessions: vec![info("a", "Plan jam2 · umarell_u3")], max_sessions: 6 };
        chats.apply_list(&list, sent);
        assert_eq!(ids(&chats), ["a", "c"], "b was closed elsewhere");
        assert_eq!(chats.active, 0, "the shown tab stays shown");
    }

    #[test]
    fn the_list_is_newest_first_and_a_reload_shows_the_newest() {
        let mut chats = Chats::default();
        let list = ChatSessions { sessions: vec![info("a", "A"), info("b", "B"), info("c", "C")], max_sessions: 6 };
        chats.apply_list(&list, web_time::Instant::now());
        assert_eq!(chats.display_order(), [2, 1, 0], "Cmd+1 is c, the newest");
        assert_eq!(chats.active, 2);
    }

    #[test]
    fn ages() {
        assert_eq!(age_text(5_000), "now");
        assert_eq!(age_text(16 * 60_000), "16m");
        assert_eq!(age_text(3 * 3_600_000), "3h");
        assert_eq!(age_text(50 * 3_600_000), "2d");
    }

    #[test]
    fn a_tab_closed_elsewhere_goes() {
        let mut chats = Chats::default();
        for id in ["a", "b", "c"] {
            chats.add_tab(&info(id, id), true);
        }
        chats.active = 2;
        chats.tabs[1].apply(ChatEvent::SessionClosed);
        chats.poll();
        assert_eq!(ids(&chats), ["a", "c"]);
        assert_eq!(chats.active, 1, "still c");
    }

    #[test]
    fn tab_status_and_title() {
        let mut tab = ChatTab::new(Some("a".into()), "Plan".into(), None);
        assert_eq!(tab.status(), SessionStatus::Idle);
        tab.apply(ChatEvent::State { state: ChatState::Thinking, agent: None });
        assert_eq!(tab.status(), SessionStatus::Running);
        tab.apply(ChatEvent::Permission { id: "p".into(), title: "run?".into(), options: Vec::new() });
        assert_eq!(tab.status(), SessionStatus::Waiting);
        tab.apply(ChatEvent::PermissionResolved { id: "p".into() });
        tab.apply(ChatEvent::Entry { entry: e(0, ChatRole::Agent, "Saved the plan.") });
        tab.apply(ChatEvent::State { state: ChatState::Ready, agent: None });
        assert_eq!(tab.status(), SessionStatus::Done);
    }

    #[test]
    fn polling_folds_into_one_row() {
        let poll = "Running: @factoryos-planner-router/plan_status";
        let entries = vec![
            e(0, ChatRole::User, "plan"),
            e(1, ChatRole::Tool, "Running: @factoryos-planner-router/plan"),
            e(2, ChatRole::Tool, poll),
            e(3, ChatRole::Agent, "Still planning."),
            e(4, ChatRole::Tool, poll),
            e(5, ChatRole::Tool, poll),
            e(6, ChatRole::Agent, "Still planning, I'll keep polling."),
            e(7, ChatRole::Tool, poll),
            e(8, ChatRole::Agent, "Done. The plan is saved."),
        ];
        let rows = fold(&entries);
        assert_eq!(rows.len(), 4);
        assert!(matches!(rows[1], Row::One(e) if e.index == 1));
        match &rows[2] {
            Row::Repeat { entries: run, calls, last, note } => {
                assert_eq!((run.len(), *calls, last.index), (6, 4, 7));
                assert_eq!(note.map(|n| n.index), Some(6));
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(rows[3], Row::One(e) if e.index == 8));
    }

    #[test]
    fn a_single_call_and_long_text_stay_apart() {
        let poll = "Running: x/plan_status";
        let long = "a".repeat(REMARK_MAX + 1);
        let entries = vec![e(0, ChatRole::Tool, poll), e(1, ChatRole::Agent, &long), e(2, ChatRole::Tool, poll)];
        assert_eq!(fold(&entries).len(), 3);
    }
}
