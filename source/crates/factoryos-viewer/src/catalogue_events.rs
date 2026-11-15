//! The shared catalogue changed ([`keys::CATALOGUE_CHANGED`], from the backend: a change made here,
//! or one another workspace made that the backend synced): the panels that list the catalogue list
//! it again ([`changed_since`]), and show a short "Shared catalogue updated" ([`notice_ui`]).
//!
//! [`keys::CATALOGUE_CHANGED`]: factoryos_client::protocol::keys::CATALOGUE_CHANGED

use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use factoryos_client::protocol::storage::CatalogueChanged;
use web_time::{Duration, Instant};

use crate::rr::egui::{self, RichText};
use crate::theme;

/// How long the notice stays.
const NOTICE_FOR: Duration = Duration::from_secs(8);

static GENERATION: AtomicU64 = AtomicU64::new(0);
static LAST: Mutex<Option<(CatalogueChanged, Instant)>> = Mutex::new(None);

/// A [`CatalogueChanged`] arrived (backend.rs).
pub fn received(payload: &[u8]) {
    let changed = serde_json::from_slice::<CatalogueChanged>(payload).unwrap_or_default();
    *LAST.lock().unwrap() = Some((changed, Instant::now()));
    GENERATION.fetch_add(1, Ordering::SeqCst);
}

/// Whether the catalogue changed since `seen` (a panel's own count, updated).
pub fn changed_since(seen: &mut u64) -> bool {
    let now = GENERATION.load(Ordering::SeqCst);
    let changed = now != *seen;
    *seen = now;
    changed
}

/// The count now: a panel starts from here (it lists the catalogue when it opens anyway).
pub fn generation() -> u64 {
    GENERATION.load(Ordering::SeqCst)
}

/// "Shared catalogue updated" for a few seconds after a change from another workspace.
pub fn notice_ui(ui: &mut egui::Ui) {
    let last = LAST.lock().unwrap().clone();
    let Some((changed, at)) = last else { return };
    if !changed.remote || at.elapsed() >= NOTICE_FOR {
        return;
    }
    let what = match changed.keys.as_slice() {
        [one] => format!(": {one}"),
        [] => String::new(),
        many => format!(": {} files", many.len()),
    };
    ui.label(RichText::new(format!("⟳ Shared catalogue updated{what}")).small().color(theme::palette(ui).accent));
    ui.ctx().request_repaint_after(Duration::from_secs(1));
}
