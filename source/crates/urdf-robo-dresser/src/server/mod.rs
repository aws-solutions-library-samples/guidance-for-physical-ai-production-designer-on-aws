//! In-binary HTTP server exposing the dresser's `dress` / `validate` /
//! `batch` / `list` operations as a versioned REST API. Gated on the
//! `server` Cargo feature so default builds stay CLI-only and pull in
//! zero axum/tokio dependencies.
//!
//! Both transports — CLI (`urdf-robo-dresser dress …`) and REST
//! (`POST /api/v1/dress`) — call the same library functions
//! (`merge`, `validate`, `parse_urdf`, `serialize_urdf`,
//! `catalogue::scan`). Single source of truth.
//!
//! Routes:
//!   * GET  /api/v1/catalogue                    catalogue listing
//!   * GET  /api/v1/catalogue/robots             robot listing only
//!   * GET  /api/v1/catalogue/tools              tool listing only
//!   * POST /api/v1/dress                        merge robot + tool
//!   * POST /api/v1/validate                     validate a URDF body
//!   * POST /api/v1/batch                        kick off a batch job
//!   * GET  /api/v1/jobs/{id}                    poll a batch job
//!   * GET  /api/v1/jobs/{id}/artifact           download the batch zip
//!   * GET  /assets/{*path}                      raw catalogue bytes (URDF + meshes)
//!   * GET  /healthz                             liveness
//!   * GET  /readyz                              readiness
//!   * GET  /openapi.json                        spec
//!   * GET  /docs                                Swagger UI
//!   * GET  /  (+ fallback)                      embedded SPA (3D URDF viewer)

use std::net::SocketAddr;
use std::path::PathBuf;

use axum::routing::{get, post};
use axum::Router;
use tower_http::compression::CompressionLayer;
use tower_http::cors::CorsLayer;

pub mod dto;
pub mod error;
pub mod handlers;
pub mod openapi;
pub mod state;

pub use error::ApiError;
pub use state::{AppState, JobRegistry};

use crate::error::DresserError;

/// Build the axum router for tests + production. State carries the
/// catalogue root + an `Arc<RwLock<JobRegistry>>` for async batches.
pub fn router(state: AppState) -> Router {
    let api_v1 = Router::new()
        .route("/catalogue", get(handlers::catalogue::list_all))
        .route("/catalogue/robots", get(handlers::catalogue::list_robots))
        .route("/catalogue/tools", get(handlers::catalogue::list_tools))
        .route("/catalogue/dressed", get(handlers::catalogue::list_dressed))
        .route("/dress", post(handlers::dress::dress))
        .route("/validate", post(handlers::validate::validate))
        .route("/batch", post(handlers::batch::start))
        .route("/jobs/{id}", get(handlers::jobs::status))
        .route("/jobs/{id}/artifact", get(handlers::jobs::artifact))
        // Viewer preload: an index over every loadable catalogue entry (robots,
        // tools and dressed combos) plus one bundled response per entry, so
        // warming the browser cache costs one request instead of the ~2,900 a
        // real gripper's mesh set would otherwise need.
        .route("/preload", get(handlers::preload::index))
        .route("/preload/{collection}/{name}", get(handlers::preload::bundle));

    Router::new()
        .nest("/api/v1", api_v1)
        // FR-056: raw catalogue bytes for the browser 3D viewer.
        .route("/assets/{*path}", get(handlers::assets::serve_asset))
        .route("/healthz", get(handlers::health::healthz))
        .route("/readyz", get(handlers::health::readyz))
        .merge(openapi::docs_router())
        // FR-052: the SPA owns "/" and every other unmatched path. axum tries
        // the explicit routes above first, so /api/v1, /assets, /docs,
        // /openapi.json, /healthz, /readyz all still win.
        .fallback(handlers::static_files::handler)
        // Content-negotiated gzip. Decisive when the viewer runs in a REMOTE Kiro
        // window: the webview renders on the client, so every mesh byte crosses the
        // SSH/SSM tunnel, and the catalogue is ASCII OBJ/MTL text that gzips ~5-6x.
        // Transparent to the three.js loaders, which parse by loader, not by MIME.
        // Outside the handlers, so the ETags they set describe the unencoded bytes;
        // the layer adds `vary: accept-encoding` itself.
        .layer(CompressionLayer::new())
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

/// Bind to `host:port` and run until the process is killed. Mirrors the
/// fitting-room banner so users see the URL on stderr on first launch. With
/// the `mcp` feature and a stdin that is not a terminal (an MCP client
/// spawned the process), MCP runs on stdin/stdout beside the REST API,
/// through the same router (`crate::mcp`); the end of a session that a
/// client used (stdin EOF) shuts REST down gracefully and `run` returns.
pub async fn run(
    host: &str,
    port: u16,
    catalogue: PathBuf,
    web_root: Option<PathBuf>,
) -> Result<(), DresserError> {
    let canon = std::fs::canonicalize(&catalogue).map_err(DresserError::from)?;
    let state = AppState::new(canon.clone()).with_web_root(web_root.clone());
    let app = router(state);

    let addr: SocketAddr = format!("{}:{}", host, port)
        .parse()
        .map_err(|e: std::net::AddrParseError| DresserError::Io(std::io::Error::other(format!(
            "invalid bind address {}:{} — {}",
            host, port, e
        ))))?;
    let listener = tokio::net::TcpListener::bind(addr).await.map_err(DresserError::from)?;

    let url_host = if host == "0.0.0.0" { "localhost" } else { host };
    let spa_src = match web_root.as_ref() {
        Some(dir) => format!("disk ({})", dir.display()),
        None => "embedded".to_string(),
    };
    eprintln!();
    eprintln!("  urdf-robo-dresser  ▸  3D viewer + REST API");
    eprintln!("  ─────────────────────────────────────────────");
    eprintln!("  Listening on   http://{}", addr);
    eprintln!("  Open in browser → http://{}:{}/", url_host, port);
    eprintln!("  API docs        http://{}:{}/docs", url_host, port);
    eprintln!("  Catalogue       {}", canon.display());
    eprintln!("  SPA source      {}", spa_src);
    #[cfg(feature = "mcp")]
    let mcp = crate::mcp::on_stdio(&app, &canon);
    eprintln!("  Press Ctrl-C to stop.");
    eprintln!();
    log::info!("urdf-robo-dresser serve listening on http://{}", addr);
    let serve_failed =
        |e: std::io::Error| DresserError::Io(std::io::Error::other(format!("axum::serve: {}", e)));
    #[cfg(feature = "mcp")]
    if let Some(facade) = mcp {
        let (session, stop) = crate::mcp::serve_stdio(facade);
        axum::serve(listener, app)
            .with_graceful_shutdown(stop)
            .await
            .map_err(serve_failed)?;
        let mcp_failed = |e: String| DresserError::Io(std::io::Error::other(e));
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
