//! In-binary HTTP server hosting the embedded 2D scene-layout editor + a
//! versioned REST API over the existing compose pipeline.
//!
//! Gated behind the `server` Cargo feature so default builds stay
//! dependency-free. The HTTP surface is documented in `docs/api.md`; the
//! authoritative wire schema is `openapi.json` (also served at
//! `/openapi.json`). API routes live under `/api/v1/...`. The unversioned
//! `/api/...` aliases are kept for one release as a deprecation cushion.
//! Liveness/readiness probes live at the root (`/healthz`, `/readyz`).

use std::net::SocketAddr;
use std::path::PathBuf;

use axum::http::{header::HeaderName, HeaderValue};
use axum::routing::{get, post};
use axum::Router;
use tower_http::cors::CorsLayer;
use tower_http::set_header::SetResponseHeaderLayer;

pub mod dto;
pub mod error;
pub mod handlers;
pub mod openapi;
pub mod state;

pub use error::ApiError;
pub use state::AppState;

use crate::error::FittingRoomError;

/// Build the axum router. `state` carries the asset/web roots and is
/// shared across all handlers via `with_state`.
pub fn router(state: AppState) -> Router {
    // Versioned API surface — every new endpoint goes here.
    let api_v1 = Router::new()
        .route("/assets/robots", get(handlers::assets::list_robots))
        .route("/assets/fixtures", get(handlers::assets::list_fixtures))
        .route("/assets/floor-plans", get(handlers::assets::list_floor_plans))
        .route(
            "/assets/floor-plans/upload-obj",
            post(handlers::assets::upload_obj),
        )
        .route("/urdf/joints", post(handlers::urdf::joints))
        .route("/floor-plan", post(handlers::floor_plan::segments))
        .route("/validate", post(handlers::compose::validate))
        .route("/compose", post(handlers::compose::compose_zip))
        .route(
            "/compose-urdf-bundle",
            post(handlers::compose::compose_urdf_bundle_zip),
        )
        .route("/scenes/load", post(handlers::scene::load))
        .route("/scenes/save", post(handlers::scene::save));

    // Deprecated unversioned aliases — same handlers, with a `Deprecation: true`
    // response header so clients see the migration signal.
    let api_legacy = Router::new()
        .route("/assets/robots", get(handlers::assets::list_robots))
        .route("/assets/fixtures", get(handlers::assets::list_fixtures))
        .route("/assets/floor-plans", get(handlers::assets::list_floor_plans))
        .route(
            "/assets/floor-plans/upload-obj",
            post(handlers::assets::upload_obj),
        )
        .route("/urdf/joints", post(handlers::urdf::joints))
        .route("/floor-plan", post(handlers::floor_plan::segments))
        .route("/validate", post(handlers::compose::validate))
        .route("/compose", post(handlers::compose::compose_zip))
        .route(
            "/compose-urdf-bundle",
            post(handlers::compose::compose_urdf_bundle_zip),
        )
        .route("/scenes/load", post(handlers::scene::load))
        .route("/scenes/save", post(handlers::scene::save))
        .layer(SetResponseHeaderLayer::if_not_present(
            HeaderName::from_static("deprecation"),
            HeaderValue::from_static("true"),
        ))
        .layer(SetResponseHeaderLayer::if_not_present(
            HeaderName::from_static("sunset"),
            HeaderValue::from_static("use /api/v1"),
        ));

    Router::new()
        .nest("/api/v1", api_v1)
        .nest("/api", api_legacy)
        // Liveness + readiness — k8s probe convention puts these at the root.
        .route("/healthz", get(handlers::health::healthz))
        .route("/readyz", get(handlers::health::readyz))
        .merge(openapi::docs_router())
        .fallback(handlers::static_files::handler)
        .layer(loopback_cors())
        .with_state(state)
}

/// CORS for pages served from this machine only, e.g. a page or tool on another loopback port.
/// Any method, request header and exposed header, as the permissive layer before it allowed, but only for an
/// `Origin` whose host is loopback. Another site's page open in the same browser gets no CORS headers,
/// so it can neither read a response nor send a preflighted request (a JSON `PUT` or `POST`). No
/// credentials either way.
fn loopback_cors() -> CorsLayer {
    use tower_http::cors::{AllowOrigin, Any};
    CorsLayer::new()
        .allow_origin(AllowOrigin::predicate(|origin, _| loopback_origin(origin.as_bytes())))
        .allow_methods(Any)
        .allow_headers(Any)
        .expose_headers(Any)
}

/// Whether an `Origin` header (`http[s]://host[:port]`) names this machine: `localhost`, 127.0.0.0/8 or `[::1]`.
fn loopback_origin(origin: &[u8]) -> bool {
    let Ok(origin) = std::str::from_utf8(origin) else { return false };
    let Some(rest) = origin.strip_prefix("http://").or_else(|| origin.strip_prefix("https://")) else { return false };
    let (host, port) = match rest.strip_prefix('[') {
        Some(v6) => match v6.split_once(']') {
            Some(split) => split,
            None => return false,
        },
        None => rest.split_at(rest.find(':').unwrap_or(rest.len())),
    };
    let port_ok = port.is_empty()
        || port.strip_prefix(':').is_some_and(|p| !p.is_empty() && p.len() <= 5 && p.bytes().all(|b| b.is_ascii_digit()));
    port_ok && (host.eq_ignore_ascii_case("localhost") || host.parse::<std::net::IpAddr>().is_ok_and(|ip| ip.is_loopback()))
}

/// Bind to `host:port` and run until the process is killed. With the `mcp`
/// feature and a stdin that is not a terminal (an MCP client spawned the
/// process), MCP runs on stdin/stdout beside the REST API, through the same
/// router (`crate::mcp`); the end of a session that a client used (stdin
/// EOF) shuts REST down gracefully and `run` returns.
pub async fn run(
    host: &str,
    port: u16,
    asset_root: PathBuf,
    web_root: Option<PathBuf>,
) -> Result<(), FittingRoomError> {
    let asset_root_canon = std::fs::canonicalize(&asset_root).map_err(|e| {
        FittingRoomError::Validation(format!(
            "asset_root {} not found: {}",
            asset_root.display(),
            e
        ))
    })?;
    let web_root_owned = web_root.clone();
    let state = AppState::new(asset_root_canon.clone(), web_root);
    let app = router(state);
    let addr: SocketAddr = format!("{}:{}", host, port)
        .parse()
        .map_err(|e: std::net::AddrParseError| {
            FittingRoomError::Validation(format!("invalid bind address {}:{} — {}", host, port, e))
        })?;
    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .map_err(|e| FittingRoomError::Validation(format!("bind {}: {}", addr, e)))?;

    // Always-on startup banner. log::info! is silenced by default (no
    // RUST_LOG configured), and missing this on first launch is the
    // single most common "did the server even start?" footgun.
    let url_host = if host == "0.0.0.0" { "localhost" } else { host };
    eprintln!();
    eprintln!("  robo-fitting-room  ▸  scene editor + REST API");
    eprintln!("  ───────────────────────────────────────────────");
    eprintln!("  Listening on   http://{}", addr);
    eprintln!("  Open in browser → http://{}:{}/", url_host, port);
    eprintln!("  Asset root     {}", asset_root_canon.display());
    eprintln!(
        "  SPA source     {}",
        match web_root_owned {
            Some(p) => format!("{} (live)", p.display()),
            None => "embedded (rebuild to update)".to_string(),
        }
    );
    #[cfg(feature = "mcp")]
    let mcp = crate::mcp::on_stdio(&app, &asset_root_canon);
    eprintln!("  Press Ctrl-C to stop.");
    eprintln!();
    log::info!("robo-fitting-room serve listening on http://{}", addr);
    let serve_failed =
        |e: std::io::Error| FittingRoomError::Validation(format!("axum::serve: {}", e));
    #[cfg(feature = "mcp")]
    if let Some(facade) = mcp {
        let (session, stop) = crate::mcp::serve_stdio(facade);
        axum::serve(listener, app)
            .with_graceful_shutdown(stop)
            .await
            .map_err(serve_failed)?;
        let mcp_failed = |e: String| FittingRoomError::Io(std::io::Error::other(e));
        return session
            .await
            .map_err(|e| mcp_failed(format!("MCP: {e}")))?
            .map_err(|e| mcp_failed(e.to_string()));
    }
    axum::serve(listener, app).await.map_err(serve_failed)?;
    Ok(())
}

#[cfg(test)]
mod loopback_cors_tests {
    #[test]
    fn only_loopback_origins_get_cors() {
        for ok in ["http://localhost:8080", "http://127.0.0.1:3030", "http://127.0.0.2", "https://localhost", "http://[::1]:9902"] {
            assert!(super::loopback_origin(ok.as_bytes()), "{ok}");
        }
        for bad in ["http://localhost.example.com", "http://127.0.0.1.example.com", "https://example.com", "null",
                    "http://example.com@127.0.0.1", "http://[::1", "http://localhost:", "http://10.0.0.1:8080", "http://0.0.0.0:8080"] {
            assert!(!super::loopback_origin(bad.as_bytes()), "{bad}");
        }
    }
}
