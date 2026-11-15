//! "Publish to shared catalogue": copy a product or a dressed robot from the user's own files into
//! the shared catalogue ([`factoryos_client::protocol::storage::api::STORAGE_PUBLISH`]).
//!
//! Offered to catalogue admins (the backend's [`StorageInfo::roles`]) when the backend keeps its files
//! in S3; with a local backend its catalogue folder already is the shared catalogue, which "Save to
//! catalogue" / Dress write. The backend checks the role again on every publish.

use std::sync::{Arc, Mutex};

use factoryos_client::protocol::storage::{StorageInfo, StorageMode};
use factoryos_client::{Api, DataSource};

use crate::rr::egui::{self, RichText};
use crate::theme;

#[derive(Default)]
struct Shared {
    info: Option<StorageInfo>,
    /// The last publish's outcome; `Err` starting with `409`: it is there already.
    result: Option<Result<String, String>>,
    busy: bool,
}

pub struct SharedPublish {
    source: DataSource,
    shared: Arc<Mutex<Shared>>,
    asked: bool,
    /// The last publish's `(from, to)`, to replace it when asked.
    last: Option<(String, String)>,
}

impl SharedPublish {
    pub fn new(api: Api) -> Self {
        Self { source: DataSource::new(api), shared: Arc::default(), asked: false, last: None }
    }

    /// Whether the button is offered (asks the backend once).
    pub fn available(&mut self, ctx: &egui::Context) -> bool {
        if !self.asked {
            self.asked = true;
            let (shared, ctx) = (self.shared.clone(), ctx.clone());
            self.source.info(Box::new(move |info| {
                shared.lock().unwrap().info = info.ok();
                ctx.request_repaint();
            }));
        }
        self.shared.lock().unwrap().info.as_ref().is_some_and(|info| info.mode == StorageMode::S3 && info.roles.catalogue_admin)
    }

    fn publish(&mut self, ctx: &egui::Context, from: &str, to: &str, overwrite: bool) {
        self.last = Some((from.to_owned(), to.to_owned()));
        let (shared, ctx, to_label) = (self.shared.clone(), ctx.clone(), to.to_owned());
        {
            let mut s = shared.lock().unwrap();
            s.busy = true;
            s.result = None;
        }
        self.source.publish(from, to, overwrite, Box::new(move |result| {
            let mut s = shared.lock().unwrap();
            s.busy = false;
            s.result = Some(result.map(|p| format!("Published {} file(s) to the shared catalogue's {to_label}", p.objects)));
            ctx.request_repaint();
        }));
    }

    /// The button (for `from` → `to`, both folders ending in `/`), and the last outcome beside it.
    pub fn button_ui(&mut self, ui: &mut egui::Ui, enabled: bool, from: &str, to: &str) {
        let ctx = ui.ctx().clone();
        if !self.available(&ctx) {
            return;
        }
        let busy = self.shared.lock().unwrap().busy;
        theme::with_controls(ui, |ui| {
            let label = if busy { "Publishing…" } else { "Publish to shared catalogue" };
            if ui
                .add_enabled(enabled && !busy, egui::Button::new(label))
                .on_hover_text(format!("Copy it into the shared catalogue ({to}), for every workspace"))
                .on_disabled_hover_text("Save it first")
                .clicked()
            {
                self.publish(&ctx, from, to, false);
            }
        });
    }

    /// The last outcome (a line under the buttons), with "Replace" when it was there already.
    pub fn status_ui(&mut self, ui: &mut egui::Ui) {
        let result = self.shared.lock().unwrap().result.clone();
        let p = theme::palette(ui);
        match result {
            Some(Ok(text)) => {
                ui.label(RichText::new(text).small().color(p.ok));
            }
            Some(Err(err)) if err.starts_with("409") => {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("It is in the shared catalogue already.").small().color(p.warn));
                    if theme::with_controls(ui, |ui| ui.button("Replace it")).clicked()
                        && let Some((from, to)) = self.last.clone()
                    {
                        let ctx = ui.ctx().clone();
                        self.publish(&ctx, &from, &to, true);
                    }
                });
            }
            Some(Err(err)) => {
                ui.label(RichText::new(format!("Not published: {err}")).small().color(p.danger));
            }
            None => {}
        }
    }
}
