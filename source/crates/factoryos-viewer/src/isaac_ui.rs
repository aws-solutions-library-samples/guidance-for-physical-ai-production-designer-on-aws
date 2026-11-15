//! "Watch in Isaac Sim": the workspace's Isaac Sim web viewer ([`api::rtc_target`]) in a new tab, next to
//! step 5's "Watch the cell" and in the Behaviour Tree pane. The targets come from [`api::RTC_TARGETS`],
//! polled; with none, the button says to boot an Isaac Sim stack (Configuration → Isaac Sim, where the build
//! has the launch link: [`crate::config::ISAAC_LAUNCH_URL`]). With several (more than one instance linked to
//! the workspace), the button asks which, or opens all.

use std::rc::Rc;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::rtc::RtcTargets;

use crate::rr::egui::{self, RichText};

/// A stack booted meanwhile shows up within this.
const POLL: Duration = Duration::from_secs(20);

const NONE_LINKED: &str = "No Isaac Sim is linked to this workspace: boot an Isaac Sim stack in \
                           Configuration → Isaac Sim (Launch Isaac Sim stack), then register its instance.";
/// [`NONE_LINKED`] in a build without the launch link (no Isaac Sim section to point at).
const NONE_LINKED_NO_LAUNCH: &str = "No Isaac Sim is linked to this workspace.";

/// Why there is nothing to watch when no Isaac Sim is linked, and how to boot one where the build can.
fn none_linked_text() -> &'static str {
    if crate::config::ISAAC_LAUNCH_URL.is_some() { NONE_LINKED } else { NONE_LINKED_NO_LAUNCH }
}

pub struct IsaacSim {
    api: Api,
    fetched: Arc<Mutex<Option<Result<RtcTargets, String>>>>,
    targets: Option<Result<RtcTargets, String>>,
    next_poll: Option<Instant>,
    /// The Configuration window's toggle (its "Open Configuration").
    settings_open: Rc<std::cell::Cell<bool>>,
}

impl IsaacSim {
    pub fn new(api: Api, settings_open: Rc<std::cell::Cell<bool>>) -> Self {
        Self { api, fetched: Arc::default(), targets: None, next_poll: None, settings_open }
    }

    fn poll(&mut self, ctx: &egui::Context) {
        if let Some(fetched) = self.fetched.lock().unwrap().take() {
            self.targets = Some(fetched);
        }
        if self.next_poll.is_some_and(|t| Instant::now() < t) {
            return;
        }
        self.next_poll = Some(Instant::now() + POLL);
        ctx.request_repaint_after(POLL);
        let (fetched, ctx) = (self.fetched.clone(), ctx.clone());
        self.api.get(api::RTC_TARGETS, move |targets: Result<RtcTargets, String>| {
            *fetched.lock().unwrap() = Some(targets);
            ctx.request_repaint();
        });
    }

    /// The viewer's URL of `target`.
    fn url(&self, target: &str) -> String {
        format!("{}{}", self.api.base(), api::rtc_target(target))
    }

    /// Isaac Sim's web viewer of `target` in a window of its own (the smartPADs' size; one per target).
    fn open(&self, ui: &egui::Ui, target: &str) {
        let url = self.url(target);
        let _ = ui;
        // Web: in the click's user gesture. Native: the system browser.
        #[cfg(target_arch = "wasm32")]
        crate::web::open_popup(&url, &format!("isaac-{target}"));
        #[cfg(not(target_arch = "wasm32"))]
        crate::workspace_ui::open_url(&url);
    }

    /// Why the button is off, if it is.
    fn blocked(&self) -> Option<String> {
        match &self.targets {
            None => Some("Asking the backend for its Isaac Sim streams…".to_owned()),
            Some(Err(err)) => Some(format!("The backend's Isaac Sim streams: {err}")),
            Some(Ok(t)) if t.targets.is_empty() => Some(match &t.error {
                Some(err) => format!("Isaac Sim streams are off: {err}"),
                None => none_linked_text().to_owned(),
            }),
            Some(Ok(t)) if !t.may_watch => Some("The Isaac Sim streams of this workspace are its owner's".to_owned()),
            Some(Ok(_)) => None,
        }
    }

    fn none_linked(&self) -> bool {
        matches!(&self.targets, Some(Ok(t)) if t.targets.is_empty() && t.error.is_none())
    }

    /// "Watch in Isaac Sim", as `button` draws it (one stream: opens it; several: a menu of them, and all);
    /// `note`: the reason it is off, also as a line below (else only on hover).
    pub fn ui(&mut self, ui: &mut egui::Ui, note: bool, button: impl Fn(&mut egui::Ui, &str, bool) -> egui::Response) {
        self.poll(ui.ctx());
        let blocked = self.blocked();
        let targets = match &self.targets {
            Some(Ok(t)) if blocked.is_none() => t.targets.clone(),
            _ => Vec::new(),
        };
        let label = if targets.len() > 1 { format!("Watch in Isaac Sim ({}) ⏷", targets.len()) } else { "Watch in Isaac Sim".to_owned() };
        let response = button(ui, &label, blocked.is_none());
        let response = match (&blocked, targets.as_slice()) {
            (Some(why), _) => response.on_disabled_hover_text(why),
            (None, [one]) => response.on_hover_text(format!(
                "Isaac Sim's own view of the cell ({}), streamed over WebRTC, in a new tab",
                api::rtc_target(one)
            )),
            _ => response.on_hover_text(format!(
                "{} Isaac Sim instances are linked to this workspace: pick one, or open all (a tab each)",
                targets.len()
            )),
        };
        match targets.as_slice() {
            [] => {}
            [one] => {
                if response.clicked() {
                    self.open(ui, one);
                }
            }
            several => {
                egui::Popup::menu(&response).show(|ui| {
                    ui.label(RichText::new("Watch which Isaac Sim?").small().weak());
                    for target in several {
                        if ui.button(target).on_hover_text(api::rtc_target(target)).clicked() {
                            self.open(ui, target);
                        }
                    }
                    ui.separator();
                    if ui
                        .button(format!("Open all {}", several.len()))
                        .on_hover_text("A tab each (the browser may ask to allow this site's pop-ups)")
                        .clicked()
                    {
                        for target in several {
                            self.open(ui, target);
                        }
                    }
                });
            }
        }
        // Where to boot one: only where the build has the launch link (the disabled button says the rest).
        if note && self.none_linked() && crate::config::ISAAC_LAUNCH_URL.is_some() {
            ui.horizontal_wrapped(|ui| {
                ui.small(RichText::new("No Isaac Sim linked: boot a stack in Configuration → Isaac Sim.").weak());
                if ui.small_button("Open Configuration").clicked() {
                    self.settings_open.set(true);
                }
            });
        }
    }
}
