//! plan2cell as an MCP server.
//!
//! The tools wrap the plan2cell pipeline (plan -> head-PLC code + robot programs). Two transports:
//! - **stdio** ([`serve_stdio`]): for an MCP client that starts the server as a child process;
//! - **streamable HTTP** ([`serve_http`]): network-exposed, at `http://<addr>/mcp`.
//!
//! Security model (the HTTP transport can be reached by other machines):
//! - Every file the tools read or write lies under one **root directory** ([`Settings::root`]).
//!   Paths are relative to it; absolute paths and `..` are rejected, symlinks out of it too.
//! - Writing is off unless [`Settings::allow_write`] is set; it never overwrites a non-empty
//!   directory unless the call asks for it.
//! - HTTP binds to loopback by default. Any other address needs a bearer token (checked on
//!   every request, constant time) and an explicit list of allowed `Host` names (DNS rebinding).

mod paths;
mod tools;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

pub use paths::Sandbox;
pub use tools::Plan2cellServer;

/// What the server may touch and how it builds.
#[derive(Clone, Debug)]
pub struct Settings {
    /// All plan, config and output paths are relative to this directory.
    pub root: PathBuf,
    /// plan2cell.toml used when a call names no config (relative to `root`, or absolute here).
    pub config: Option<PathBuf>,
    /// Allow `plan2cell_build` to write into `root`.
    pub allow_write: bool,
}

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("{0}")]
    Config(String),
    #[error("{0}")]
    Io(#[from] std::io::Error),
    #[error("{0}")]
    Mcp(String),
}

/// Serve over stdin/stdout until the client disconnects.
pub async fn serve_stdio(settings: Settings) -> Result<(), Error> {
    use rmcp::ServiceExt;
    let server = Plan2cellServer::new(settings)?;
    let running = server
        .serve(rmcp::transport::stdio())
        .await
        .map_err(|e| Error::Mcp(e.to_string()))?;
    running
        .waiting()
        .await
        .map_err(|e| Error::Mcp(e.to_string()))?;
    Ok(())
}

/// Network options of the HTTP transport.
#[derive(Clone, Debug)]
pub struct HttpOptions {
    pub addr: SocketAddr,
    /// Required for a non-loopback address; `Authorization: Bearer <token>` on every request.
    pub token: Option<String>,
    /// Extra `Host` header values to accept (loopback names are always accepted). Needed when
    /// clients reach the server by a DNS name or a non-loopback IP.
    pub allowed_hosts: Vec<String>,
}

impl HttpOptions {
    /// Refuse configurations that would expose the build tools without authentication.
    pub fn check(&self) -> Result<(), Error> {
        let loopback = self.addr.ip().is_loopback();
        match &self.token {
            Some(t) if t.len() < 16 => Err(Error::Config(
                "the bearer token must be at least 16 characters".into(),
            )),
            None if !loopback => Err(Error::Config(format!(
                "refusing to listen on {} without a token: set --token (or PLAN2CELL_MCP_TOKEN), \
                 or bind to 127.0.0.1",
                self.addr
            ))),
            _ => Ok(()),
        }
    }
}

/// A running HTTP server.
pub struct HttpServer {
    pub local_addr: SocketAddr,
    cancel: tokio_util::sync::CancellationToken,
    task: tokio::task::JoinHandle<()>,
}

impl HttpServer {
    pub async fn shutdown(self) {
        self.cancel.cancel();
        let _ = self.task.await;
    }

    /// Run until Ctrl-C.
    pub async fn run_until_ctrl_c(self) {
        let _ = tokio::signal::ctrl_c().await;
        self.shutdown().await;
    }
}

/// Start the streamable-HTTP server (`/mcp`, plus `/healthz` without authentication).
pub async fn serve_http(settings: Settings, opts: HttpOptions) -> Result<HttpServer, Error> {
    use rmcp::transport::streamable_http_server::{
        StreamableHttpServerConfig, StreamableHttpService, session::local::LocalSessionManager,
    };
    opts.check()?;
    let server = Plan2cellServer::new(settings)?;
    let cancel = tokio_util::sync::CancellationToken::new();
    let mut hosts: Vec<String> = vec!["localhost".into(), "127.0.0.1".into(), "::1".into()];
    hosts.extend(opts.allowed_hosts.iter().cloned());
    let config = StreamableHttpServerConfig::default()
        .with_allowed_hosts(hosts)
        .with_cancellation_token(cancel.child_token());
    let service: StreamableHttpService<Plan2cellServer, LocalSessionManager> =
        StreamableHttpService::new(move || Ok(server.clone()), Default::default(), config);
    let token = opts.token.clone().map(Arc::new);
    let mcp = axum::Router::new()
        .nest_service("/mcp", service)
        .layer(axum::middleware::from_fn(move |req, next| {
            let token = token.clone();
            async move { auth(token, req, next).await }
        }));
    let app = axum::Router::new()
        .route("/healthz", axum::routing::get(|| async { "ok" }))
        .merge(mcp);
    let listener = tokio::net::TcpListener::bind(opts.addr).await?;
    let local_addr = listener.local_addr()?;
    let stop = cancel.clone();
    let task = tokio::spawn(async move {
        let _ = axum::serve(listener, app)
            .with_graceful_shutdown(async move { stop.cancelled_owned().await })
            .await;
    });
    Ok(HttpServer {
        local_addr,
        cancel,
        task,
    })
}

async fn auth(
    token: Option<Arc<String>>,
    req: axum::extract::Request,
    next: axum::middleware::Next,
) -> axum::response::Response {
    use axum::response::IntoResponse;
    let Some(token) = token else {
        return next.run(req).await;
    };
    let presented = req
        .headers()
        .get(axum::http::header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("Bearer "))
        .unwrap_or("");
    if constant_time_eq(presented.as_bytes(), token.as_bytes()) {
        next.run(req).await
    } else {
        (
            axum::http::StatusCode::UNAUTHORIZED,
            [(axum::http::header::WWW_AUTHENTICATE, "Bearer")],
            "missing or wrong bearer token",
        )
            .into_response()
    }
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_loopback_needs_a_token() {
        let o = |addr: &str, token: Option<&str>| HttpOptions {
            addr: addr.parse().unwrap(),
            token: token.map(str::to_string),
            allowed_hosts: vec![],
        };
        assert!(o("127.0.0.1:8765", None).check().is_ok());
        assert!(o("[::1]:8765", None).check().is_ok());
        assert!(o("0.0.0.0:8765", None).check().is_err());
        assert!(o("0.0.0.0:8765", Some("short")).check().is_err());
        assert!(o("0.0.0.0:8765", Some("0123456789abcdef")).check().is_ok());
    }

    #[test]
    fn token_compare() {
        assert!(constant_time_eq(b"abc", b"abc"));
        assert!(!constant_time_eq(b"abc", b"abd"));
        assert!(!constant_time_eq(b"abc", b"abcd"));
    }
}
