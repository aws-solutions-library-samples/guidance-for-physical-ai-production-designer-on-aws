//! Noticing that the backend went away or restarted, so the viewer can pick up again without a
//! page reload.
//!
//! Two signals, either is enough:
//! * the bus's view of the backend's liveliness token ([`Bus::backend_away`], [`Bus::on_rejoin`]);
//! * a health poll ([`api::HEALTH`]) every [`POLL_UP`] ([`POLL_DOWN`] while away), each with a
//!   [`HEALTH_TIMEOUT`]: [`LOST_AFTER`] failures in a row mean the backend is away, and another
//!   `bootId` than the last one means it restarted ([`BootWatch`]).
//!
//! A restart bumps [`Reconnect::restarts`]; the app then has the panels open again on the new
//! backend (it starts with none of them open), asks the storage mode again and retries failed
//! assets (see `FactoryOsApp::on_backend_restart`). The bus re-queries its followed states and
//! scene snapshots on its own re-join. If the health poll sees the new backend and the bus has
//! not re-joined it [`BUS_GRACE`] later (a WebSocket left half-open), the bus is reconnected.
//! While the backend is away, [`notice_ui`] says so.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use factoryos_client::protocol::{Health, api};
use factoryos_client::{Api, Bus};
use web_time::Instant;

use crate::rr::egui;

/// Health poll interval while the backend answers.
pub const POLL_UP: Duration = Duration::from_secs(5);
/// Health poll interval while it is away.
pub const POLL_DOWN: Duration = Duration::from_secs(2);
/// A health check without an answer by then failed.
pub const HEALTH_TIMEOUT: Duration = Duration::from_secs(5);
/// Failed health checks in a row before the backend counts as away (one is a blip).
pub const LOST_AFTER: u32 = 2;
/// After a restart, how long the bus may take to re-join the new backend before it is reconnected.
pub const BUS_GRACE: Duration = Duration::from_secs(10);

/// What a health check said about the backend, compared with the ones before.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    /// Nothing new.
    Same,
    /// It stopped answering.
    Lost,
    /// It answers again: the same process (the network was away).
    Back,
    /// Another process answers (another boot id): it restarted.
    Restarted,
}

/// The health checks' state machine: the backend's boot id, whether it is away, and when to
/// check next. Pure: the caller does the requests ([`Self::due`], [`Self::sent`],
/// [`Self::observe`]).
#[derive(Debug)]
pub struct BootWatch {
    /// The boot id last answered (empty: an older backend without one).
    boot: Option<String>,
    failures: u32,
    away: bool,
    in_flight: bool,
    /// Asked for at once while a check was in flight: the next one follows its answer.
    asap: bool,
    next_at: Instant,
}

impl BootWatch {
    /// Watching the backend that answered `boot` (at connecting), first checked [`POLL_UP`] after `now`.
    pub fn new(boot: &str, now: Instant) -> Self {
        Self { boot: Some(boot.to_owned()), failures: 0, away: false, in_flight: false, asap: false, next_at: now + POLL_UP }
    }

    /// The backend is away (it stopped answering, and has not answered since).
    pub fn away(&self) -> bool {
        self.away
    }

    /// The boot id last answered, if any (empty for a backend without one).
    #[cfg(test)]
    pub fn boot(&self) -> Option<&str> {
        self.boot.as_deref()
    }

    /// Whether to send a health check now.
    pub fn due(&self, now: Instant) -> bool {
        !self.in_flight && now >= self.next_at
    }

    /// A health check was sent.
    pub fn sent(&mut self) {
        self.in_flight = true;
        self.asap = false;
    }

    /// Check now, or as soon as the one in flight has answered (whose answer may predate
    /// whatever asks, e.g. the bus re-joined).
    pub fn check_now(&mut self, now: Instant) {
        if self.in_flight {
            self.asap = true;
        } else {
            self.next_at = now;
        }
    }

    /// When the next check is due.
    pub fn next_at(&self) -> Instant {
        self.next_at
    }

    /// A health check's answer: the backend's boot id, or `Err` (no answer, an error status, not
    /// in time).
    pub fn observe(&mut self, answer: Result<&str, ()>, now: Instant) -> Change {
        self.in_flight = false;
        let change = match answer {
            Ok(boot) => {
                self.failures = 0;
                let was_away = std::mem::take(&mut self.away);
                let restarted = match &self.boot {
                    // A backend that names no boot id can't tell; the bus's re-join does.
                    Some(last) => !boot.is_empty() && !last.is_empty() && last != boot,
                    None => false,
                };
                self.boot = Some(boot.to_owned());
                if restarted {
                    Change::Restarted
                } else if was_away {
                    Change::Back
                } else {
                    Change::Same
                }
            }
            Err(()) => {
                self.failures += 1;
                if self.failures >= LOST_AFTER && !self.away {
                    self.away = true;
                    Change::Lost
                } else {
                    Change::Same
                }
            }
        };
        self.next_at = if std::mem::take(&mut self.asap) { now } else { now + self.interval() };
        change
    }

    fn interval(&self) -> Duration {
        if self.away || self.failures > 0 { POLL_DOWN } else { POLL_UP }
    }
}

/// After a restart the health checks saw: whether the bus has re-joined the new backend, and
/// whether to reconnect it. Pure.
#[derive(Debug, Default)]
pub struct BusFollow {
    /// The boot id the bus should re-join, and since when it has not.
    waiting: Option<(String, Instant)>,
}

impl BusFollow {
    /// The health checks saw backend `boot` start.
    pub fn restarted(&mut self, boot: &str, now: Instant) {
        if !boot.is_empty() {
            self.waiting = Some((boot.to_owned(), now));
        }
    }

    /// Whether to reconnect the bus now, given the boot id it sees alive. At most once per
    /// [`BUS_GRACE`] until it has re-joined.
    pub fn reconnect(&mut self, bus_boot: Option<&str>, now: Instant) -> bool {
        let Some((boot, since)) = &mut self.waiting else { return false };
        if bus_boot == Some(boot.as_str()) {
            self.waiting = None;
            return false;
        }
        if now.duration_since(*since) < BUS_GRACE {
            return false;
        }
        *since = now;
        true
    }
}

struct Inner {
    watch: BootWatch,
    bus: BusFollow,
    restarts: u64,
    /// The bus re-joined since the last health answer (for a backend without a boot id, the
    /// only sign of a restart).
    rejoined: bool,
    ctx: Option<egui::Context>,
}

/// Watches the backend for the app ([`Self::poll`] every frame).
pub struct Reconnect {
    api: Api,
    bus: Bus,
    inner: Arc<Mutex<Inner>>,
}

impl Reconnect {
    /// Watch the backend `api`/`bus` connected to, which answered `boot_id`.
    pub fn new(api: &Api, bus: &Bus, boot_id: &str) -> Self {
        let inner = Arc::new(Mutex::new(Inner {
            watch: BootWatch::new(boot_id, Instant::now()),
            bus: BusFollow::default(),
            restarts: 0,
            rejoined: false,
            ctx: None,
        }));
        let weak = Arc::downgrade(&inner);
        bus.on_rejoin(move || {
            let Some(inner) = weak.upgrade() else { return false };
            let mut inner = inner.lock().unwrap();
            // Restarted, or the same backend back? The health check tells.
            inner.rejoined = true;
            inner.watch.check_now(Instant::now());
            if let Some(ctx) = &inner.ctx {
                ctx.request_repaint();
            }
            true
        });
        Self { api: api.with_timeout(HEALTH_TIMEOUT), bus: bus.clone(), inner }
    }

    /// Send a health check when one is due, reconnect the bus if it is left behind. Call every frame.
    pub fn poll(&self, ctx: &egui::Context) {
        let now = Instant::now();
        let check = {
            let mut inner = self.inner.lock().unwrap();
            inner.ctx.get_or_insert_with(|| ctx.clone());
            let bus_boot = self.bus.backend_boot();
            if !inner.watch.away() && inner.bus.reconnect(bus_boot.as_deref(), now) {
                crate::rr::re_log::warn!("The bus has not re-joined the restarted backend: reconnecting it");
                self.bus.reconnect();
            }
            let due = inner.watch.due(now);
            if due {
                inner.watch.sent();
            }
            due
        };
        if check {
            let inner = self.inner.clone();
            let ctx = ctx.clone();
            self.api.get(api::HEALTH, move |health: Result<Health, String>| {
                let now = Instant::now();
                let mut inner = inner.lock().unwrap();
                let change = inner.watch.observe(health.as_ref().map(|h| h.boot_id.as_str()).map_err(|_| ()), now);
                let rejoined = std::mem::take(&mut inner.rejoined) && health.is_ok();
                let old_backend = health.as_ref().is_ok_and(|h| h.boot_id.is_empty());
                match change {
                    Change::Lost => crate::rr::re_log::warn!("The backend does not answer; reconnecting…"),
                    Change::Back => crate::rr::re_log::info!("The backend answers again"),
                    Change::Restarted => crate::rr::re_log::info!("The backend restarted; picking up again"),
                    Change::Same => {}
                }
                if change == Change::Restarted || (rejoined && old_backend) {
                    inner.restarts += 1;
                    if let Ok(health) = &health {
                        inner.bus.restarted(&health.boot_id, now);
                    }
                }
                drop(inner);
                ctx.request_repaint();
            });
        }
        let next = self.inner.lock().unwrap().watch.next_at();
        ctx.request_repaint_after(next.saturating_duration_since(now).max(Duration::from_millis(100)));
    }

    /// How many restarts were seen (the app picks up again when this changes).
    pub fn restarts(&self) -> u64 {
        self.inner.lock().unwrap().restarts
    }

    /// The backend is away: the health checks fail, or the bus lost it and has not re-joined.
    pub fn reconnecting(&self) -> bool {
        self.inner.lock().unwrap().watch.away() || self.bus.backend_away()
    }
}

/// "Reconnecting to the backend…", small, at the top of the window below Rerun's top bar.
pub fn notice_ui(ctx: &egui::Context, below_top_bar: Option<egui::Rect>) {
    let top = below_top_bar.map_or(40.0, |rect| rect.top() + 8.0);
    let palette = crate::theme::current();
    egui::Area::new(egui::Id::new("factoryos_reconnecting"))
        .order(egui::Order::Foreground)
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, top))
        .interactable(false)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(palette.warn_subtle)
                .stroke(egui::Stroke::new(1.0, palette.warn))
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(10, 4))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label(egui::RichText::new("Reconnecting to the backend…").color(palette.text_strong));
                    });
                });
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(start: Instant, secs: u64) -> Instant {
        start + Duration::from_secs(secs)
    }

    #[test]
    fn the_same_boot_is_nothing_new() {
        let t0 = Instant::now();
        let mut watch = BootWatch::new("a", t0);
        assert!(!watch.due(t0), "first checked a poll interval after connecting");
        assert!(watch.due(t0 + POLL_UP));
        watch.sent();
        assert!(!watch.due(t0 + POLL_UP), "one at a time");
        assert_eq!(watch.observe(Ok("a"), t0 + POLL_UP), Change::Same);
        assert_eq!(watch.next_at(), t0 + POLL_UP + POLL_UP);
        assert!(!watch.away());
    }

    #[test]
    fn another_boot_id_is_a_restart() {
        let t0 = Instant::now();
        let mut watch = BootWatch::new("a", t0);
        assert_eq!(watch.observe(Ok("b"), at(t0, 5)), Change::Restarted);
        assert_eq!(watch.boot(), Some("b"));
        assert_eq!(watch.observe(Ok("b"), at(t0, 10)), Change::Same, "once");
    }

    #[test]
    fn lost_after_failures_in_a_row_then_back_or_restarted() {
        let t0 = Instant::now();
        let mut watch = BootWatch::new("a", t0);
        assert_eq!(watch.observe(Err(()), at(t0, 5)), Change::Same, "one failure is a blip");
        assert!(!watch.away());
        assert_eq!(watch.next_at(), at(t0, 5) + POLL_DOWN, "checked sooner after a failure");
        assert_eq!(watch.observe(Ok("a"), at(t0, 7)), Change::Same);
        assert_eq!(watch.observe(Err(()), at(t0, 12)), Change::Same);
        assert_eq!(watch.observe(Err(()), at(t0, 14)), Change::Lost);
        assert!(watch.away());
        assert_eq!(watch.observe(Err(()), at(t0, 16)), Change::Same, "lost once");
        // The same process again: the network was away.
        assert_eq!(watch.observe(Ok("a"), at(t0, 18)), Change::Back);
        assert!(!watch.away());
        assert_eq!(watch.next_at(), at(t0, 18) + POLL_UP);
        // Away again, and another process answers: a restart (not just "back").
        watch.observe(Err(()), at(t0, 23));
        watch.observe(Err(()), at(t0, 25));
        assert_eq!(watch.observe(Ok("b"), at(t0, 27)), Change::Restarted);
        assert!(!watch.away());
    }

    #[test]
    fn a_backend_without_boot_ids_never_restarts_by_health() {
        let t0 = Instant::now();
        let mut watch = BootWatch::new("", t0);
        assert_eq!(watch.observe(Ok(""), at(t0, 5)), Change::Same);
        watch.observe(Err(()), at(t0, 10));
        assert_eq!(watch.observe(Err(()), at(t0, 12)), Change::Lost);
        assert_eq!(watch.observe(Ok(""), at(t0, 14)), Change::Back);
        // An upgrade to a backend that has one: no restart to compare with yet, it is just back.
        assert_eq!(watch.observe(Ok("c"), at(t0, 19)), Change::Same);
        assert_eq!(watch.observe(Ok("d"), at(t0, 24)), Change::Restarted);
    }

    #[test]
    fn check_now_while_in_flight_checks_right_after() {
        let t0 = Instant::now();
        let mut watch = BootWatch::new("a", t0);
        watch.sent();
        // The bus re-joined while the check was in flight: its answer may predate that.
        watch.check_now(at(t0, 1));
        assert!(!watch.due(at(t0, 1)), "one at a time");
        watch.observe(Ok("a"), at(t0, 2));
        assert!(watch.due(at(t0, 2)), "asked again at once");
    }

    #[test]
    fn the_bus_is_reconnected_if_it_does_not_rejoin_in_time() {
        let t0 = Instant::now();
        let mut bus = BusFollow::default();
        assert!(!bus.reconnect(Some("a"), t0), "nothing to wait for");
        bus.restarted("b", t0);
        assert!(!bus.reconnect(None, at(t0, 5)), "within the grace");
        assert!(bus.reconnect(Some("a"), t0 + BUS_GRACE), "still on the old backend");
        assert!(!bus.reconnect(None, t0 + BUS_GRACE + Duration::from_secs(1)), "once per grace");
        assert!(bus.reconnect(None, t0 + BUS_GRACE * 2));
        // Re-joined: done.
        assert!(!bus.reconnect(Some("b"), t0 + BUS_GRACE * 3));
        assert!(!bus.reconnect(None, t0 + BUS_GRACE * 5));
    }

    #[test]
    fn a_restart_the_bus_saw_first_needs_no_reconnect() {
        let t0 = Instant::now();
        let mut bus = BusFollow::default();
        bus.restarted("b", t0);
        assert!(!bus.reconnect(Some("b"), t0 + BUS_GRACE * 2));
        bus.restarted("", t0);
        assert!(!bus.reconnect(None, t0 + BUS_GRACE * 2), "no boot id: nothing to compare with");
    }
}
