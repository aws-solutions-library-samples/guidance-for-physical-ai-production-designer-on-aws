//! Client for factoryos-backend, for native and wasm viewers alike.
//!
//! * [`Api`]: HTTP request/response (`/api/v1/…`), callback based so it fits an egui frame loop.
//! * [`Bus`]: Zenoh pub/sub and queries: live state, commands, RRD streams.
//!
//! Callbacks run on a background task (native) or the JS event loop (wasm); hand results to the
//! UI through shared state and request a repaint.

mod api;
#[cfg(not(target_arch = "wasm32"))]
pub mod auth;
/// The internal build's silent login ([`auth::login_silently`]); not in the public export.
#[cfg(all(feature = "internal", not(target_arch = "wasm32")))]
mod internal;
pub mod chat;
mod rejoin;
pub mod storage;
pub mod terminal;
mod timer;

#[cfg(not(target_arch = "wasm32"))]
mod bus_native;
#[cfg(target_arch = "wasm32")]
mod bus_web;

pub use api::{Api, WORKSPACE_NOT_READY, workspace_status};
#[cfg(not(target_arch = "wasm32"))]
pub use bus_native::Bus;
#[cfg(target_arch = "wasm32")]
pub use bus_web::Bus;
pub use factoryos_protocol as protocol;
pub use rejoin::RejoinFn;
pub use storage::DataSource;

/// Select rustls' crypto provider (`ring`) for this process, once, before any TLS connection.
/// Without it, a build with more than one provider enabled (the whole workspace at once) panics.
#[cfg(not(target_arch = "wasm32"))]
pub fn install_crypto_provider() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        // Err: one is installed already, which is just as good.
        let _ = rustls::crypto::ring::default_provider().install_default();
    });
}

pub type Callback<T> = Box<dyn FnOnce(Result<T, String>) + Send + 'static>;

/// Starts the error [`with_deadline`] answers with when the real answer did not come in time.
pub const TIMED_OUT: &str = "timed out";

/// `done`, called at most once: with the real answer, or with an error starting with
/// [`TIMED_OUT`] once `after` has passed without one (a request that would otherwise wait
/// forever, e.g. to a backend that went away mid-request). An answer after that is dropped.
pub fn with_deadline<T: 'static>(after: std::time::Duration, what: impl Into<String>, done: Callback<T>) -> Callback<T> {
    let slot = std::sync::Arc::new(std::sync::Mutex::new(Some(done)));
    let expired = slot.clone();
    let what = what.into();
    timer::after(after, move || {
        let done = expired.lock().unwrap().take();
        if let Some(done) = done {
            done(Err(format!("{TIMED_OUT}: {what}: no answer within {} s", after.as_secs_f32())));
        }
    });
    Box::new(move |result| {
        let done = slot.lock().unwrap().take();
        if let Some(done) = done {
            done(result);
        }
    })
}

/// The Zenoh endpoint to use for the backend at `backend_url`, given its `/api/v1/health`.
/// Wasm viewers always use `ws(s)://…/zenoh`. Native viewers get a Zenoh locator, or a
/// `ws(s)://…/zenoh-native` URL, which [`Bus::connect`] reaches through a local bridge:
/// * under a path (a workspace's `/workspace/<name>`, behind a load balancer): that URL only;
/// * otherwise, in this order:
///   1. an advertised TCP/TLS endpoint they can reach: bound to an unspecified host (`0.0.0.0`,
///      `[::]`, replaced by the backend's host), to the backend's host itself, or to loopback
///      when the backend is on this machine;
///   2. that URL, if the backend takes Zenoh over its HTTP port; this is how a server that only
///      exposes port 80 is reached;
///   3. any other advertised TCP/TLS endpoint.
pub fn zenoh_endpoint(backend_url: &str, health: &protocol::Health) -> Option<String> {
    if cfg!(target_arch = "wasm32") {
        return Some(websocket_url(backend_url, protocol::api::ZENOH_WS));
    }
    let websocket = || health.zenoh_ws.then(|| websocket_url(backend_url, protocol::api::ZENOH_NATIVE_WS));
    if has_path(backend_url) {
        return websocket();
    }
    let host = backend_host(backend_url)?;
    let advertised: Vec<(&str, &str, &str)> = health
        .zenoh_endpoints
        .iter()
        .filter_map(|endpoint| {
            let (proto, addr) = endpoint.split_once('/')?;
            let (bind_host, port) = addr.rsplit_once(':')?;
            matches!(proto, "tcp" | "tls").then_some((proto, bind_host, port))
        })
        .collect();
    let reachable = advertised.iter().find_map(|&(proto, bind_host, port)| {
        let target = if is_unspecified(bind_host) {
            host
        } else if bind_host == host || (is_loopback(bind_host) && is_loopback(host)) {
            bind_host
        } else {
            return None;
        };
        Some(format!("{proto}/{target}:{port}"))
    });
    let any_other = || {
        advertised
            .iter()
            .find(|(_, bind_host, _)| !is_loopback(bind_host))
            .map(|(proto, bind_host, port)| format!("{proto}/{bind_host}:{port}"))
    };
    reachable.or_else(websocket).or_else(any_other)
}

/// `http://host/…` + `path` → `ws://host/…{path}`, `https://` → `wss://`.
pub(crate) fn websocket_url(base: &str, path: &str) -> String {
    let base = base.trim_end_matches('/');
    let rest = base
        .strip_prefix("https://")
        .map(|rest| ("wss://", rest))
        .or_else(|| base.strip_prefix("http://").map(|rest| ("ws://", rest)));
    match rest {
        Some((scheme, rest)) => format!("{scheme}{rest}{path}"),
        // Already a ws URL, or something unexpected: let the WebSocket client judge it.
        None => format!("{base}{path}"),
    }
}

/// Whether `url` has a path beyond `/`, e.g. a workspace's `http://host/workspace/<name>`.
fn has_path(url: &str) -> bool {
    let rest = url.split_once("://").map_or(url, |(_, rest)| rest);
    rest.find('/').is_some_and(|slash| !rest[slash..].trim_end_matches('/').is_empty())
}

/// Whether the backend at `backend_url` runs on this machine.
pub fn is_local(backend_url: &str) -> bool {
    backend_host(backend_url).is_some_and(is_loopback)
}

fn is_unspecified(host: &str) -> bool {
    matches!(host, "0.0.0.0" | "[::]" | "::")
}

fn is_loopback(host: &str) -> bool {
    host == "localhost" || host == "[::1]" || host.starts_with("127.")
}

/// `http://host:8080/…` → `host`.
fn backend_host(url: &str) -> Option<&str> {
    let rest = url.split_once("://")?.1;
    let authority = rest.split('/').next()?;
    if authority.starts_with('[') {
        return authority.split_once(']').map(|(h, _)| &authority[..=h.len()]);
    }
    Some(authority.rsplit_once(':').map_or(authority, |(host, _)| host))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn health(endpoints: &[&str], zenoh_ws: bool) -> protocol::Health {
        protocol::Health {
            protocol_version: protocol::PROTOCOL_VERSION,
            backend_version: String::new(),
            zenoh_endpoints: endpoints.iter().map(|e| (*e).to_owned()).collect(),
            zenoh_ws,
            workspace: None,
            release: None,
            idle_secs: None,
            open_connections: None,
            boot_id: String::new(),
        }
    }

    #[test]
    fn native_endpoint_uses_backend_host() {
        let unspecified = health(&["tcp/0.0.0.0:7447"], true);
        assert_eq!(
            zenoh_endpoint("http://cell-host:8080", &unspecified).as_deref(),
            Some("tcp/cell-host:7447")
        );
        let other_interface = health(&["udp/0.0.0.0:7447", "tls/192.0.2.5:7448"], false);
        assert_eq!(
            zenoh_endpoint("https://x:8443/", &other_interface).as_deref(),
            Some("tls/192.0.2.5:7448")
        );
        assert_eq!(zenoh_endpoint("http://h:1", &health(&[], false)), None);
    }

    #[test]
    fn local_backend_uses_loopback_tcp() {
        let local = health(&["tcp/127.0.0.1:7447"], true);
        assert_eq!(zenoh_endpoint("http://127.0.0.1:8080", &local).as_deref(), Some("tcp/127.0.0.1:7447"));
        assert_eq!(zenoh_endpoint("http://localhost:8080", &local).as_deref(), Some("tcp/127.0.0.1:7447"));
    }

    #[test]
    fn remote_backend_with_loopback_zenoh_uses_websocket() {
        let server = health(&["tcp/127.0.0.1:7447"], true);
        assert_eq!(
            zenoh_endpoint("http://203.0.113.10", &server).as_deref(),
            Some("ws://203.0.113.10/zenoh-native")
        );
        assert_eq!(
            zenoh_endpoint("https://cell-host:8443/", &server).as_deref(),
            Some("wss://cell-host:8443/zenoh-native")
        );
        assert_eq!(zenoh_endpoint("http://cell-host", &health(&["tcp/127.0.0.1:7447"], false)), None);
    }

    #[test]
    fn workspace_uses_websocket_only() {
        // Even a reachable TCP endpoint would bypass the load balancer's routing.
        let server = health(&["tcp/0.0.0.0:7447"], true);
        assert_eq!(
            zenoh_endpoint("http://lb.example.com/workspace/jdoe", &server).as_deref(),
            Some("ws://lb.example.com/workspace/jdoe/zenoh-native")
        );
        assert_eq!(zenoh_endpoint("http://lb/workspace/jdoe", &health(&["tcp/0.0.0.0:7447"], false)), None);
    }

    #[test]
    fn builds_websocket_urls() {
        assert_eq!(websocket_url("http://host:8080", "/chat"), "ws://host:8080/chat");
        assert_eq!(websocket_url("https://host/", "/chat"), "wss://host/chat");
        assert_eq!(websocket_url("http://host/workspace/a/", "/zenoh"), "ws://host/workspace/a/zenoh");
    }

    /// Built with the backend (`cargo test -p factoryos-client -p factoryos-backend`), both rustls
    /// providers are enabled: without the explicit choice this panics.
    #[test]
    fn tls_needs_no_provider_guess() {
        install_crypto_provider();
        let _ = rustls::ClientConfig::builder();
    }

    #[test]
    fn deadlines_answer_once() {
        use std::sync::mpsc;
        use std::time::Duration;

        // No answer: the deadline's error.
        let (tx, rx) = mpsc::channel();
        let _pending = with_deadline::<u32>(Duration::from_millis(20), "health", Box::new(move |r| tx.send(r).unwrap()));
        let err = rx.recv_timeout(Duration::from_secs(5)).unwrap().unwrap_err();
        assert!(err.starts_with(TIMED_OUT) && err.contains("health"), "{err}");
        assert!(rx.recv_timeout(Duration::from_millis(50)).is_err(), "once");

        // The answer in time: that, and nothing when the deadline passes.
        let (tx, rx) = mpsc::channel();
        let done = with_deadline::<u32>(Duration::from_millis(20), "health", Box::new(move |r| tx.send(r).unwrap()));
        done(Ok(7));
        assert_eq!(rx.recv_timeout(Duration::from_secs(1)).unwrap(), Ok(7));
        assert!(rx.recv_timeout(Duration::from_millis(100)).is_err(), "the deadline says nothing after the answer");
    }

    #[test]
    fn host_parsing() {
        assert_eq!(backend_host("http://127.0.0.1:8080"), Some("127.0.0.1"));
        assert_eq!(backend_host("https://example.com/viewer"), Some("example.com"));
        assert_eq!(backend_host("http://[::1]:8080"), Some("[::1]"));
    }
}
