//! Re-joins: the bus session sees the backend's liveliness token
//! ([`crate::protocol::keys::backend_alive`]) appear on joining, and again after every re-join,
//! a restarted backend (another boot id) or a reconnect to the same one. Samples published while
//! the session was away are lost, so whatever follows "latest state" queries it again then.

use std::sync::{Arc, Mutex};

/// A [`crate::Bus::on_rejoin`] callback; `false` once it is no longer interested.
pub trait RejoinFn: Fn() -> bool + Send + Sync + 'static {}
impl<F: Fn() -> bool + Send + Sync + 'static> RejoinFn for F {}

/// The backend's liveliness as the session sees it, and who to tell when it comes back.
#[derive(Clone, Default)]
pub(crate) struct Rejoins(Arc<Mutex<State>>);

#[derive(Default)]
struct State {
    /// The first token seen is the join itself, not a re-join.
    seen: bool,
    /// The boot id whose token is alive now.
    alive: Option<String>,
    callbacks: Vec<Box<dyn RejoinFn>>,
}

impl Rejoins {
    pub fn on_rejoin(&self, callback: impl RejoinFn) {
        self.0.lock().unwrap().callbacks.push(Box::new(callback));
    }

    /// The backend's token with `boot_id` appeared. Tells the callbacks unless it is the first
    /// one seen, or the one alive already (a duplicate).
    pub fn appeared(&self, boot_id: &str) {
        let rejoin = {
            let mut state = self.0.lock().unwrap();
            let rejoin = state.seen && state.alive.as_deref() != Some(boot_id);
            state.seen = true;
            state.alive = Some(boot_id.to_owned());
            rejoin
        };
        if rejoin {
            self.notify();
        }
    }

    /// The token with `boot_id` went away (`None`: the session lost them all).
    pub fn vanished(&self, boot_id: Option<&str>) {
        let mut state = self.0.lock().unwrap();
        if boot_id.is_none() || state.alive.as_deref() == boot_id {
            state.alive = None;
        }
    }

    /// The boot id of the backend alive now, as its token says.
    pub fn alive(&self) -> Option<String> {
        self.0.lock().unwrap().alive.clone()
    }

    /// A backend was seen, and none is alive now: a re-join is still to come.
    pub fn away(&self) -> bool {
        let state = self.0.lock().unwrap();
        state.seen && state.alive.is_none()
    }

    fn notify(&self) {
        // Without the lock: a callback may register another.
        let callbacks = std::mem::take(&mut self.0.lock().unwrap().callbacks);
        let kept: Vec<_> = callbacks.into_iter().filter(|callback| callback()).collect();
        let mut state = self.0.lock().unwrap();
        let added = std::mem::replace(&mut state.callbacks, kept);
        state.callbacks.extend(added);
    }
}

/// The boot id in a token's key, `factoryos/backend/alive/<boot_id>`.
pub(crate) fn boot_id_of(key: &str) -> &str {
    key.rsplit('/').next().unwrap_or(key)
}

impl crate::Bus {
    /// Call `callback` whenever the session re-joined the backend (it restarted, or the
    /// connection was lost and is back): samples published meanwhile were missed. Not on the
    /// first join. Kept while it returns `true`.
    pub fn on_rejoin(&self, callback: impl RejoinFn) {
        self.rejoins().on_rejoin(callback);
    }

    /// The boot id of the backend the session sees alive now (`None` before its liveliness
    /// token arrived, or while the session is away).
    pub fn backend_boot(&self) -> Option<String> {
        self.rejoins().alive()
    }

    /// The session saw the backend, and has lost it (it went down, or the connection did): what
    /// follows "latest state" is out of date until the re-join ([`Self::on_rejoin`]). Not before
    /// the first join.
    pub fn backend_away(&self) -> bool {
        self.rejoins().away()
    }

    /// Follow `key`'s latest state: `on_payload` gets every sample, the reply to a query now (a
    /// late joiner's catch-up), and the reply to one after every re-join (what was published while
    /// away), for the rest of the session.
    pub fn follow(&self, key: &str, on_payload: impl Fn(Vec<u8>) + Send + Sync + 'static) -> Result<(), String> {
        let on_payload = Arc::new(on_payload);
        let on_sample = on_payload.clone();
        self.subscribe(key, move |payload| on_sample(payload))?;
        let catch_up = {
            let (bus, key) = (self.clone(), key.to_owned());
            move || {
                let on_payload = on_payload.clone();
                bus.query(&key, None, move |reply| {
                    if let Ok(Some(payload)) = reply {
                        on_payload(payload);
                    }
                });
                true
            }
        };
        catch_up();
        self.on_rejoin(catch_up);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    fn counting(rejoins: &Rejoins) -> Arc<AtomicUsize> {
        let count = Arc::new(AtomicUsize::new(0));
        let c = count.clone();
        rejoins.on_rejoin(move || {
            c.fetch_add(1, Ordering::SeqCst);
            true
        });
        count
    }

    #[test]
    fn the_first_token_is_the_join() {
        let rejoins = Rejoins::default();
        let count = counting(&rejoins);
        assert!(!rejoins.away(), "not joined yet: not away");
        rejoins.appeared("a");
        rejoins.appeared("a");
        assert_eq!(count.load(Ordering::SeqCst), 0);
        assert_eq!(rejoins.alive().as_deref(), Some("a"));
        assert!(!rejoins.away());
    }

    #[test]
    fn a_restart_and_a_reconnect_are_rejoins() {
        let rejoins = Rejoins::default();
        let count = counting(&rejoins);
        rejoins.appeared("a");
        // Restarted: the old token may go before or after the new one comes.
        rejoins.appeared("b");
        rejoins.vanished(Some("a"));
        assert_eq!(count.load(Ordering::SeqCst), 1);
        assert_eq!(rejoins.alive().as_deref(), Some("b"));
        // The connection lost and back, same backend.
        rejoins.vanished(Some("b"));
        assert_eq!(rejoins.alive(), None);
        assert!(rejoins.away(), "until it re-joins");
        rejoins.appeared("b");
        assert!(!rejoins.away());
        assert_eq!(count.load(Ordering::SeqCst), 2);
        // The session lost (the web bus reopening).
        rejoins.vanished(None);
        rejoins.appeared("b");
        assert_eq!(count.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn callbacks_drop_out_and_may_register_more() {
        let rejoins = Rejoins::default();
        let count = counting(&rejoins);
        let once = Arc::new(AtomicUsize::new(0));
        {
            let (once, inner) = (once.clone(), rejoins.clone());
            rejoins.on_rejoin(move || {
                once.fetch_add(1, Ordering::SeqCst);
                inner.on_rejoin(|| false);
                false
            });
        }
        rejoins.appeared("a");
        rejoins.appeared("b");
        rejoins.appeared("c");
        assert_eq!(count.load(Ordering::SeqCst), 2);
        assert_eq!(once.load(Ordering::SeqCst), 1);
        assert_eq!(rejoins.0.lock().unwrap().callbacks.len(), 1, "the ones that said false are gone");
    }

    #[test]
    fn boot_ids_from_keys() {
        assert_eq!(boot_id_of(&crate::protocol::keys::backend_alive("19a-1f")), "19a-1f");
    }
}
