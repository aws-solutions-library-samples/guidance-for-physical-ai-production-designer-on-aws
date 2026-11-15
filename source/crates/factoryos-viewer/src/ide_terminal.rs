//! The Workspace IDE's terminal panel, at the bottom of the page: the shown session's screen
//! ([`crate::term_view`]) and, in a column at the right, the sessions with their host and current
//! folder. Shells run on the backend's host ([`api::TERMINALS`]); each session has a WebSocket of
//! its own ([`TerminalSocket`]), reopened (and its output replayed) when it drops.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::terminal::{BACKEND_HOST, NewTerminal, TerminalHost, TerminalInfo, TerminalNotice};
use factoryos_client::terminal::{SocketState, TermEvent, TerminalSocket};
use web_time::{Duration, Instant};

use crate::rr::egui::{self, RichText};
use crate::term_view::TermView;
use crate::theme;

const RECONNECT: Duration = Duration::from_secs(2);

enum Msg {
    Hosts(Result<Vec<TerminalHost>, String>),
    Listed(Result<Vec<TerminalInfo>, String>),
    Opened(Result<TerminalInfo, String>),
    Closed(Result<bool, String>),
}

struct Term {
    info: TerminalInfo,
    socket: Option<TerminalSocket>,
    view: TermView,
    /// When to open a new socket after the last one closed.
    reconnect_at: Option<Instant>,
}

pub struct TerminalPanel {
    pub open: bool,
    api: Api,
    inbox: Arc<Mutex<Vec<Msg>>>,
    hosts: Vec<TerminalHost>,
    terms: Vec<Term>,
    active: usize,
    /// The sessions were asked for (once per opening of the panel).
    listed: bool,
    /// A session is being made; and whether the first one was (when there were none).
    opening: bool,
    error: Option<String>,
    maximized: bool,
}

impl TerminalPanel {
    pub fn new(api: Api) -> Self {
        Self {
            open: true,
            api,
            inbox: Arc::default(),
            hosts: Vec::new(),
            terms: Vec::new(),
            active: 0,
            listed: false,
            opening: false,
            error: None,
            maximized: false,
        }
    }

    fn sender(&self, ctx: &egui::Context) -> impl Fn(Msg) + Send + Sync + 'static {
        let (inbox, ctx) = (self.inbox.clone(), ctx.clone());
        move |msg| {
            inbox.lock().unwrap().push(msg);
            ctx.request_repaint();
        }
    }

    fn list(&mut self, ctx: &egui::Context) {
        self.listed = true;
        let send = self.sender(ctx);
        self.api.get(api::TERMINAL_HOSTS, move |hosts| send(Msg::Hosts(hosts)));
        let send = self.sender(ctx);
        self.api.get(api::TERMINALS, move |list| send(Msg::Listed(list)));
    }

    fn new_terminal(&mut self, ctx: &egui::Context, host: &str, root: Option<&str>) {
        if self.opening {
            return;
        }
        self.opening = true;
        let (cols, rows) = self.terms.get(self.active).map_or((100, 20), |t| t.view.size());
        let request = NewTerminal { host: host.to_owned(), root: root.map(str::to_owned), cols, rows };
        let send = self.sender(ctx);
        self.api.post_json(api::TERMINALS, &request, move |info| send(Msg::Opened(info)));
    }

    fn kill(&mut self, ctx: &egui::Context, index: usize) {
        let Some(term) = self.terms.get(index) else { return };
        let id = term.info.id.clone();
        let send = self.sender(ctx);
        self.api.delete(&api::terminal(&id), move |done| send(Msg::Closed(done)));
        self.terms.remove(index);
        self.active = self.active.min(self.terms.len().saturating_sub(1));
    }

    fn add(&mut self, ctx: &egui::Context, info: TerminalInfo) -> usize {
        if let Some(index) = self.terms.iter().position(|t| t.info.id == info.id) {
            return index;
        }
        let mut term = Term { info, socket: None, view: TermView::default(), reconnect_at: None };
        term.socket = self.connect(ctx, &term.info.id);
        self.terms.push(term);
        self.terms.len() - 1
    }

    fn connect(&mut self, ctx: &egui::Context, id: &str) -> Option<TerminalSocket> {
        let ctx = ctx.clone();
        match TerminalSocket::connect(self.api.base(), id, move || ctx.request_repaint()) {
            Ok(socket) => Some(socket),
            Err(err) => {
                self.error = Some(err);
                None
            }
        }
    }

    fn poll(&mut self, ctx: &egui::Context, root: Option<&str>) {
        let messages = std::mem::take(&mut *self.inbox.lock().unwrap());
        for msg in messages {
            match msg {
                Msg::Hosts(Ok(hosts)) => self.hosts = hosts,
                Msg::Hosts(Err(err)) | Msg::Listed(Err(err)) => self.error = Some(refusal(&err)),
                Msg::Listed(Ok(list)) => {
                    self.error = None;
                    let empty = list.is_empty() && self.terms.is_empty();
                    for info in list {
                        self.add(ctx, info);
                    }
                    if empty {
                        self.new_terminal(ctx, BACKEND_HOST, root);
                    }
                }
                Msg::Opened(result) => {
                    self.opening = false;
                    match result {
                        Ok(info) => {
                            self.error = None;
                            self.active = self.add(ctx, info);
                        }
                        Err(err) => self.error = Some(refusal(&err)),
                    }
                }
                // Already gone (a restarted backend): nothing to say.
                Msg::Closed(Err(err)) if !err.starts_with("404") => self.error = Some(refusal(&err)),
                Msg::Closed(_) => {}
            }
        }
        let now = Instant::now();
        for index in 0..self.terms.len() {
            let mut reconnect = false;
            let term = &mut self.terms[index];
            if let Some(socket) = &mut term.socket {
                for event in socket.drain() {
                    match event {
                        TermEvent::Output(bytes) => term.view.feed(&bytes),
                        TermEvent::Notice(TerminalNotice::Cwd(cwd)) => term.info.cwd = cwd,
                        TermEvent::Notice(TerminalNotice::Exit(code)) => {
                            term.info.alive = false;
                            term.info.exit_code = code;
                        }
                        TermEvent::Notice(TerminalNotice::Replayed) => {}
                    }
                }
                if socket.state() == SocketState::Closed {
                    term.socket = None;
                    term.reconnect_at = term.info.alive.then_some(now + RECONNECT);
                }
            } else if term.reconnect_at.is_some_and(|at| at <= now) {
                term.reconnect_at = None;
                reconnect = true;
            } else if let Some(at) = term.reconnect_at {
                ctx.request_repaint_after(at - now);
            }
            if reconnect {
                let id = term.info.id.clone();
                // The new socket replays the output so far onto an empty screen.
                self.terms[index].view.reset();
                self.terms[index].socket = self.connect(ctx, &id);
            }
        }
    }

    /// The panel at the bottom of `ui` (call before the panels above it); `root`: the IDE's folder,
    /// for new shells.
    pub fn ui(&mut self, ui: &mut egui::Ui, root: Option<&str>) {
        let ctx = ui.ctx().clone();
        if ui.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::Backtick)) {
            self.open = !self.open;
        }
        if !self.open {
            self.listed = false;
            return;
        }
        if !self.listed {
            self.list(&ctx);
        }
        self.poll(&ctx, root);
        let p = theme::palette(ui);
        let available = ui.available_height();
        let mut panel = egui::Panel::bottom("workspace_ide_terminal")
            .frame(egui::Frame::new().fill(p.bg).stroke(egui::Stroke::new(1.0, p.border)).inner_margin(egui::Margin { left: 0, right: 0, top: 4, bottom: 0 }));
        panel = if self.maximized {
            panel.resizable(false).exact_size((available * 0.8).max(160.0))
        } else {
            panel.resizable(true).default_size(240.0).min_size(110.0).max_size((available * 0.8).max(160.0))
        };
        panel.show(ui, |ui| {
            egui::Panel::right("workspace_ide_terminal_sessions")
                .resizable(true)
                .default_size(220.0)
                .min_size(170.0)
                .max_size(320.0)
                .frame(egui::Frame::new().fill(p.card).inner_margin(egui::Margin::symmetric(6, 4)))
                .show(ui, |ui| self.sessions_ui(ui, &ctx, root));
            egui::CentralPanel::no_frame().show(ui, |ui| {
                if let Some(err) = &self.error {
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.add_space(10.0);
                        ui.colored_label(p.warn, err.as_str());
                    });
                }
                let Some(term) = self.terms.get_mut(self.active) else {
                    if self.error.is_none() {
                        ui.add_space(8.0);
                        ui.horizontal(|ui| {
                            ui.add_space(10.0);
                            ui.spinner();
                            ui.label(RichText::new("Starting a shell…").color(p.muted));
                        });
                    }
                    return;
                };
                let out = term.view.ui(ui, egui::Id::new(("workspace_ide_term", &term.info.id)));
                if let Some(socket) = &term.socket {
                    if let Some((cols, rows)) = out.resize {
                        socket.resize(cols, rows);
                    }
                    if !out.input.is_empty() && term.info.alive {
                        socket.send_bytes(out.input);
                    }
                }
            });
        });
    }

    fn sessions_ui(&mut self, ui: &mut egui::Ui, ctx: &egui::Context, root: Option<&str>) {
        let p = theme::palette(ui);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            ui.label(RichText::new("TERMINALS").small().strong().color(p.muted));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.add(tool_button("✕", p)).on_hover_text("Hide the panel (Ctrl+`)").clicked() {
                    self.open = false;
                }
                let max = if self.maximized { "Restore the panel" } else { "Maximize the panel" };
                if ui.add(tool_button(if self.maximized { "▭" } else { "⛶" }, p)).on_hover_text(max).clicked() {
                    self.maximized = !self.maximized;
                }
                if !self.terms.is_empty() && ui.add(tool_button("🗑", p)).on_hover_text("Kill the shown terminal").clicked() {
                    self.kill(ctx, self.active);
                }
                let hosts = self.hosts.clone();
                ui.menu_button(RichText::new("▾").size(12.0).color(p.muted), |ui| {
                    ui.label(RichText::new("New terminal on").small().color(p.muted));
                    for host in &hosts {
                        let label = format!("{}  ({})", host.label, host.os);
                        if ui.add_enabled(host.online, egui::Button::new(label)).clicked() {
                            let root = (host.id == BACKEND_HOST).then_some(root).flatten();
                            self.new_terminal(ctx, &host.id, root);
                            ui.close();
                        }
                    }
                });
                if ui.add(tool_button("+", p)).on_hover_text("New terminal in the IDE's folder").clicked() {
                    self.new_terminal(ctx, BACKEND_HOST, root);
                }
            });
        });
        ui.add_space(2.0);
        let mut pick = None;
        let mut kill = None;
        egui::ScrollArea::vertical().id_salt("workspace_ide_terminal_list").auto_shrink([false, false]).show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            for (index, term) in self.terms.iter().enumerate() {
                let selected = index == self.active;
                let (rect, response) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 38.0), egui::Sense::click());
                let hovered = ui.rect_contains_pointer(rect);
                let painter = ui.painter_at(rect);
                if selected {
                    painter.rect_filled(rect, 4, p.accent.gamma_multiply(0.22));
                } else if hovered {
                    painter.rect_filled(rect, 4, p.hover);
                }
                let left = rect.left() + 8.0;
                painter.text(egui::pos2(left, rect.top() + 11.0), egui::Align2::LEFT_CENTER, ">_", egui::FontId::monospace(10.5), if selected { p.accent } else { p.muted });
                let mut title = term.info.shell.clone();
                if term.info.host != BACKEND_HOST {
                    title = format!("{title} · {}", term.info.host);
                }
                painter.text(egui::pos2(left + 22.0, rect.top() + 11.0), egui::Align2::LEFT_CENTER, &title, egui::FontId::proportional(12.5), if selected { p.text_strong } else { p.text });
                let path = short_path(&term.info.cwd, ((rect.width() - 40.0) / 6.5) as usize);
                painter.text(egui::pos2(left + 22.0, rect.top() + 27.0), egui::Align2::LEFT_CENTER, &path, egui::FontId::monospace(10.5), p.muted);
                let status = if !term.info.alive {
                    Some(("⏻", p.danger, format!("exited ({})", term.info.exit_code.map_or_else(|| "?".to_owned(), |c| c.to_string()))))
                } else if term.socket.is_none() || term.socket.as_ref().is_some_and(|s| s.state() != SocketState::Open) {
                    Some(("⚠", p.warn, "connecting…".to_owned()))
                } else {
                    None
                };
                let close_at = egui::Rect::from_center_size(egui::pos2(rect.right() - 14.0, rect.top() + 11.0), egui::vec2(18.0, 18.0));
                if hovered {
                    let x = egui::Button::new(RichText::new("✕").size(12.0).color(p.muted)).frame(false);
                    if ui.place(close_at, x).on_hover_text("Kill this terminal").clicked() {
                        kill = Some(index);
                    }
                } else if let Some((icon, color, _)) = &status {
                    painter.text(close_at.center(), egui::Align2::CENTER_CENTER, *icon, egui::FontId::proportional(11.0), *color);
                }
                let hover = match &status {
                    Some((_, _, what)) => format!("{}\n{what}", term.info.cwd),
                    None => term.info.cwd.clone(),
                };
                if response.on_hover_text(hover).clicked() {
                    pick = Some(index);
                }
            }
        });
        if let Some(index) = kill {
            self.kill(ctx, index);
        } else if let Some(index) = pick {
            self.active = index;
        }
    }
}

/// An error from the backend, in words for the panel.
fn refusal(err: &str) -> String {
    if err.starts_with("403") {
        "Terminals are only open to this workspace's owner.".to_owned()
    } else if err.starts_with("404") {
        "This backend has no terminals (an older release?).".to_owned()
    } else {
        format!("Terminal: {err}")
    }
}

/// `path` in at most `max` characters: its last folders, `…/` in front of what doesn't fit.
pub fn short_path(path: &str, max: usize) -> String {
    let max = max.max(8);
    if path.chars().count() <= max {
        return path.to_owned();
    }
    let parts: Vec<&str> = path.split('/').filter(|p| !p.is_empty()).collect();
    let mut kept = String::new();
    for part in parts.iter().rev() {
        let candidate = if kept.is_empty() { (*part).to_owned() } else { format!("{part}/{kept}") };
        if candidate.chars().count() + 2 > max {
            break;
        }
        kept = candidate;
    }
    if kept.is_empty() {
        let last = parts.last().copied().unwrap_or(path);
        let tail: String = last.chars().rev().take(max - 1).collect::<Vec<_>>().into_iter().rev().collect();
        return format!("…{tail}");
    }
    format!("…/{kept}")
}

fn tool_button(icon: &str, p: &theme::Palette) -> egui::Button<'static> {
    egui::Button::new(RichText::new(icon.to_owned()).size(12.5).color(p.muted)).frame(false).min_size(egui::vec2(20.0, 20.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_keep_their_last_folders() {
        assert_eq!(short_path("/opt/biw", 30), "/opt/biw");
        assert_eq!(short_path("/Users/jdoe/Projects/cells/welding/biw/out", 20), "…/welding/biw/out");
        assert_eq!(short_path("/a/averyveryverylongfoldername", 12), "…gfoldername");
    }

    #[test]
    fn refusals_say_why() {
        assert!(refusal("403 Forbidden: workspace files: only the owner").contains("owner"));
        assert!(refusal("500 Internal").starts_with("Terminal: "));
    }
}
