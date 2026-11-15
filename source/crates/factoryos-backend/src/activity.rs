//! When a viewer last used this backend, for the host's idle shutdown (`factoryos-workspace
//! idle-check` reads it from `/api/v1/health`). Health checks (the load balancer's, the idle
//! check's own) don't count; an open WebSocket (Zenoh, chat) counts as in use however quiet it is.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::Instant;

use axum::extract::{Request, State};
use axum::middleware::Next;
use axum::response::Response;
use factoryos_protocol::api;

#[derive(Debug)]
pub struct Activity {
    started: Instant,
    /// Milliseconds after `started` of the last request.
    last_ms: AtomicU64,
    open: AtomicU32,
}

impl Activity {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            started: Instant::now(),
            last_ms: AtomicU64::new(0),
            open: AtomicU32::new(0),
        })
    }

    pub fn touch(&self) {
        self.last_ms.store(self.started.elapsed().as_millis() as u64, Ordering::Relaxed);
    }

    pub fn idle_secs(&self) -> u64 {
        let now = self.started.elapsed().as_millis() as u64;
        now.saturating_sub(self.last_ms.load(Ordering::Relaxed)) / 1000
    }

    pub fn open_connections(&self) -> u32 {
        self.open.load(Ordering::Relaxed)
    }

    /// Count a WebSocket as open until the guard drops.
    pub fn connection(self: &Arc<Self>) -> ConnectionGuard {
        self.open.fetch_add(1, Ordering::Relaxed);
        self.touch();
        ConnectionGuard(self.clone())
    }
}

pub struct ConnectionGuard(Arc<Activity>);

impl Drop for ConnectionGuard {
    fn drop(&mut self) {
        self.0.open.fetch_sub(1, Ordering::Relaxed);
        self.0.touch();
    }
}

/// Middleware: every request but a health check is activity. Runs inside the workspace-prefix
/// stripping, so the path is the backend's own.
pub async fn middleware(State(activity): State<Arc<Activity>>, request: Request, next: Next) -> Response {
    if request.uri().path() != api::HEALTH {
        activity.touch();
    }
    next.run(request).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_connections() {
        let activity = Activity::new();
        assert_eq!(activity.open_connections(), 0);
        let guard = activity.connection();
        assert_eq!(activity.open_connections(), 1);
        drop(guard);
        assert_eq!(activity.open_connections(), 0);
        assert_eq!(activity.idle_secs(), 0);
    }
}
