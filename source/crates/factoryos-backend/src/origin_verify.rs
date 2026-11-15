//! The origin check of the public deployment (deployment/CONTRACT.md "Origin secret"): CloudFront's sign-in gate
//! (Lambda@Edge, deployment/lambda/edge-auth) adds `x-origin-verify: <secret>` to every request it lets through,
//! and with `FACTORY_OS_ORIGIN_SECRET` set this backend refuses (403) every request from a peer that is not loopback
//! without that header, WebSocket upgrades included. So a request reaches the routes only through this deployment's
//! own distribution (whose gate verified a session), not through any other CloudFront distribution with a VPC origin
//! for the instance. Loopback peers (the SSM port forward, the planner router, the health check, internal callers)
//! are exempt. Unset (internal deployments, local runs): nothing changes.
//!
//! The variable holds one value, or during a rotation two separated by a comma (the new one and the one before it,
//! both accepted until the installer writes the new one alone). Values are compared in constant time. The header is
//! removed from a request before the routes see it (the DCV, WebRTC and smartPAD proxies never pass it on).

use std::net::SocketAddr;
use std::sync::Arc;

use axum::extract::connect_info::ConnectInfo;
use axum::extract::{Request, State};
use axum::http::{HeaderValue, StatusCode, header};
use axum::middleware::Next;
use axum::response::{IntoResponse as _, Response};

/// The header the edge sets.
pub const HEADER: &str = "x-origin-verify";
/// The variable naming the accepted value(s).
pub const ENV: &str = "FACTORY_OS_ORIGIN_SECRET";
const MIN_LEN: usize = 32;
const MAX_LEN: usize = 256;
const MAX_VALUES: usize = 2;

/// The accepted values (never printed: no `Debug` of the values).
pub struct OriginSecret {
    values: Vec<Vec<u8>>,
}

impl std::fmt::Debug for OriginSecret {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "OriginSecret({} value(s))", self.values.len())
    }
}

impl OriginSecret {
    /// `FACTORY_OS_ORIGIN_SECRET`'s value(s): `None` when unset; an error (the backend does not start) when set but
    /// not one or two comma-separated base64url values of 32 to 256 characters. The error never shows the value.
    pub fn from_env() -> anyhow::Result<Option<Arc<Self>>> {
        match std::env::var_os(ENV) {
            None => Ok(None),
            Some(raw) => {
                let raw = raw.into_string().map_err(|_| anyhow::anyhow!("{ENV} is not UTF-8"))?;
                Ok(Some(Arc::new(Self::parse(&raw)?)))
            }
        }
    }

    pub fn parse(raw: &str) -> anyhow::Result<Self> {
        let values: Vec<&str> = raw.trim().split(',').map(str::trim).collect();
        anyhow::ensure!(values.len() <= MAX_VALUES, "{ENV} holds {} values; at most {MAX_VALUES} (the new one, the one before)", values.len());
        for value in &values {
            anyhow::ensure!(
                (MIN_LEN..=MAX_LEN).contains(&value.len())
                    && value.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_'),
                "{ENV} is set but is not one or two comma-separated base64url values of {MIN_LEN} to {MAX_LEN} characters"
            );
        }
        Ok(Self { values: values.iter().map(|v| v.as_bytes().to_vec()).collect() })
    }

    /// The header's value is one of the accepted ones (every one compared, each in constant time).
    pub fn accepts(&self, given: &[u8]) -> bool {
        let mut ok = false;
        for value in &self.values {
            ok |= constant_time_eq(value, given);
        }
        ok
    }
}

/// Equal, in time independent of where `a` and `b` differ (the lengths are not secret: every value is as long).
fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b) {
        diff |= x ^ y;
    }
    std::hint::black_box(diff) == 0
}

/// A loopback peer: 127.0.0.0/8, ::1, or ::ffff:127.x (an IPv4 peer on a dual-stack socket).
fn is_loopback(peer: &SocketAddr) -> bool {
    peer.ip().to_canonical().is_loopback()
}

/// [`check`] as middleware around the whole router (outside the workspace prefix); the state is the secret.
pub async fn middleware(State(secret): State<Option<Arc<OriginSecret>>>, request: Request, next: Next) -> Response {
    check(secret.as_deref(), request, async |request| next.run(request).await).await
}

/// Refuse `request` (403) unless `secret` is unset, its peer is loopback, or it carries the header with an accepted
/// value; then run `next` without the header. A request without a peer address counts as not loopback.
pub async fn check(secret: Option<&OriginSecret>, mut request: Request, next: impl AsyncFnOnce(Request) -> Response) -> Response {
    let Some(secret) = secret else {
        return next(request).await;
    };
    let peer = request.extensions().get::<ConnectInfo<SocketAddr>>().map(|ConnectInfo(peer)| *peer);
    let loopback = peer.as_ref().is_some_and(is_loopback);
    let mut given = request.headers().get_all(HEADER).iter();
    let verified = match (given.next(), given.next()) {
        (Some(value), None) => secret.accepts(value.as_bytes()),
        _ => false,
    };
    if !loopback && !verified {
        tracing::warn!(
            "origin check: refused {} {} from {} (no valid {HEADER}: not through this deployment's CloudFront)",
            request.method(),
            request.uri().path(),
            peer.map_or_else(|| "an unknown peer".to_owned(), |p| p.to_string()),
        );
        return forbidden();
    }
    request.headers_mut().remove(HEADER);
    next(request).await
}

fn forbidden() -> Response {
    (
        StatusCode::FORBIDDEN,
        [
            (header::CONTENT_TYPE, HeaderValue::from_static("text/plain; charset=utf-8")),
            (header::CACHE_CONTROL, HeaderValue::from_static("no-store")),
        ],
        "Forbidden: not through this deployment's sign-in\n",
    )
        .into_response()
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::Router;
    use axum::body::Body;
    use axum::routing::get;
    use tower::ServiceExt as _;

    // The tests' own values, no deployment's.
    const SECRET: &str = "s3cr3t-0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUV"; // pragma: allowlist secret
    const OLD: &str = "old-0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWX"; // pragma: allowlist secret

    /// The middleware around a router whose routes echo whether they saw the header.
    fn app(secret: Option<&str>) -> Router {
        let secret = secret.map(|s| Arc::new(OriginSecret::parse(s).unwrap()));
        let seen = |request: Request| async move { if request.headers().contains_key(HEADER) { "header seen" } else { "ok" } };
        let routes = Router::new().route("/api/v1/health", get(seen)).route("/zenoh", get(seen));
        routes.layer(axum::middleware::from_fn_with_state(secret, middleware))
    }

    async fn call(app: Router, peer: Option<&str>, uri: &str, headers: &[(&str, &str)]) -> (StatusCode, String) {
        let mut builder = Request::builder().uri(uri);
        for (name, value) in headers {
            builder = builder.header(*name, *value);
        }
        let mut request = builder.body(Body::empty()).unwrap();
        if let Some(peer) = peer {
            request.extensions_mut().insert(ConnectInfo(peer.parse::<SocketAddr>().unwrap()));
        }
        let response = app.oneshot(request).await.unwrap();
        let status = response.status();
        let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
        (status, String::from_utf8(body.to_vec()).unwrap())
    }

    #[tokio::test]
    async fn a_remote_peer_without_the_header_is_refused() {
        let (status, body) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/api/v1/health", &[]).await;
        assert_eq!(status, StatusCode::FORBIDDEN);
        assert!(body.starts_with("Forbidden"), "{body}");
        assert!(!body.contains(SECRET));
    }

    #[tokio::test]
    async fn a_wrong_or_doubled_header_is_refused() {
        for headers in [
            vec![(HEADER, "guess")],
            vec![(HEADER, &SECRET[..SECRET.len() - 1])],
            vec![(HEADER, "")],
            vec![(HEADER, SECRET), (HEADER, SECRET)],
        ] {
            let (status, _) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/api/v1/health", &headers).await;
            assert_eq!(status, StatusCode::FORBIDDEN, "{headers:?}");
        }
    }

    #[tokio::test]
    async fn the_right_header_passes_and_is_removed() {
        let (status, body) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/api/v1/health", &[(HEADER, SECRET)]).await;
        assert_eq!((status, body.as_str()), (StatusCode::OK, "ok"));
    }

    #[tokio::test]
    async fn loopback_without_the_header_passes() {
        for peer in ["127.0.0.1:5000", "127.0.0.53:5000", "[::1]:5000", "[::ffff:127.0.0.1]:5000"] {
            let (status, body) = call(app(Some(SECRET)), Some(peer), "/api/v1/health", &[]).await;
            assert_eq!((status, body.as_str()), (StatusCode::OK, "ok"), "{peer}");
        }
        // A loopback caller's header goes too (never passed on).
        let (_, body) = call(app(Some(SECRET)), Some("127.0.0.1:5000"), "/api/v1/health", &[(HEADER, "x")]).await;
        assert_eq!(body, "ok");
    }

    #[tokio::test]
    async fn no_peer_address_counts_as_remote() {
        let (status, _) = call(app(Some(SECRET)), None, "/api/v1/health", &[]).await;
        assert_eq!(status, StatusCode::FORBIDDEN);
    }

    #[tokio::test]
    async fn unset_changes_nothing() {
        let (status, body) = call(app(None), Some("10.0.0.5:41000"), "/api/v1/health", &[]).await;
        assert_eq!((status, body.as_str()), (StatusCode::OK, "ok"));
        // Not even the header is touched.
        let (_, body) = call(app(None), Some("10.0.0.5:41000"), "/api/v1/health", &[(HEADER, "x")]).await;
        assert_eq!(body, "header seen");
    }

    #[tokio::test]
    async fn websocket_upgrades_are_checked_too() {
        let ws = [
            ("connection", "upgrade"),
            ("upgrade", "websocket"),
            ("sec-websocket-version", "13"),
            ("sec-websocket-key", "dGhlIHNhbXBsZSBub25jZQ=="),
        ];
        let (status, _) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/zenoh", &ws).await;
        assert_eq!(status, StatusCode::FORBIDDEN);
        let with: Vec<_> = ws.iter().copied().chain([(HEADER, SECRET)]).collect();
        let (status, body) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/zenoh", &with).await;
        assert_eq!((status, body.as_str()), (StatusCode::OK, "ok"));
    }

    #[tokio::test]
    async fn during_a_rotation_both_values_pass() {
        let both = format!("{SECRET},{OLD}");
        for value in [SECRET, OLD] {
            let (status, _) = call(app(Some(&both)), Some("10.0.0.5:41000"), "/api/v1/health", &[(HEADER, value)]).await;
            assert_eq!(status, StatusCode::OK);
        }
        let (status, _) = call(app(Some(SECRET)), Some("10.0.0.5:41000"), "/api/v1/health", &[(HEADER, OLD)]).await;
        assert_eq!(status, StatusCode::FORBIDDEN);
    }

    #[test]
    fn a_malformed_value_is_an_error_that_does_not_show_it() {
        for raw in ["", "short", &format!("{SECRET},{OLD},{SECRET}"), &format!("{SECRET},"), &format!("{} x", &SECRET[..40])] {
            let err = OriginSecret::parse(raw).unwrap_err().to_string();
            assert!(!err.contains(&SECRET[..32]) && !err.contains("short"), "{err}");
        }
        assert!(OriginSecret::parse(SECRET).is_ok());
        assert!(OriginSecret::parse(&format!(" {SECRET} , {OLD} \n")).is_ok());
        assert_eq!(format!("{:?}", OriginSecret::parse(SECRET).unwrap()), "OriginSecret(1 value(s))");
    }

    #[test]
    fn constant_time_eq_is_eq() {
        assert!(constant_time_eq(b"abc", b"abc"));
        assert!(!constant_time_eq(b"abc", b"abd"));
        assert!(!constant_time_eq(b"abc", b"abcd"));
        assert!(constant_time_eq(b"", b""));
    }
}
