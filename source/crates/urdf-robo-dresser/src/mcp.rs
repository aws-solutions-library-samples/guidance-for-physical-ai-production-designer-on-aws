//! MCP beside the REST API. When an MCP client spawns `serve` (stdin is not a terminal), the
//! process also speaks MCP on stdin/stdout: every operation of [`ApiDoc`] is a tool, answered in
//! process by the router `serve` serves over REST, so both share one `AppState` (a batch job
//! started over MCP shows over REST too) and the same handlers, validation and sandbox. MCP
//! opens no socket. The facade is the `openapi-mcp` crate; it saves zips in a new 0700
//! temporary directory per process.

use std::future::Future;
use std::io::IsTerminal as _;
use std::path::Path;

use axum::Router;
use openapi_mcp::{Facade, Options};
use tokio::task::JoinHandle;
use utoipa::OpenApi as _;

use crate::server::openapi::ApiDoc;

/// The POST operations that only read.
const READ_ONLY: [&str; 1] = ["validate_urdf"];

/// Every operation of [`ApiDoc`] as an MCP tool, answered by `app`: give it the router `serve`
/// serves, and MCP and REST share one `AppState`. The instructions name `catalogue`.
pub fn facade(app: Router, catalogue: &Path) -> Result<Facade<Router>, openapi_mcp::Error> {
    let spec = serde_json::to_value(ApiDoc::openapi())
        .map_err(|e| openapi_mcp::Error::Spec(e.to_string()))?;
    let mut options = Options::new(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    options.read_only = READ_ONLY.map(String::from).to_vec();
    options.instructions = Some(format!(
        "The catalogue root is {}: urdf-robot-collection/<id>/<id>.urdf holds the robots, \
         urdf-tool-collection/<id>/<id>.urdf the tools, and dress_robot writes \
         urdf-dressed-robots/<robot_id>__<tool_id>/. robot_id and tool_id are single entry \
         names, as list_catalogue_robots and list_catalogue_tools return them, never paths; \
         validate_urdf's `path` is relative to the catalogue root, without `..`. Typical flow: \
         list_catalogue, then dress_robot for one pair; or start_batch for every pair, then \
         get_job_status with its job_id as `id` (poll until `state` is completed or failed), \
         then get_job_artifact, which saves the zip. Batch jobs live in this process only.",
        catalogue.display()
    ));
    Facade::new(app, &spec, options)
}

/// The facade `serve` runs on stdio beside REST, after printing the banner's MCP line. `None`
/// when stdin is a terminal (a person started `serve`, not an MCP client) or the facade can't
/// be built; REST runs the same either way.
pub(crate) fn on_stdio(app: &Router, catalogue: &Path) -> Option<Facade<Router>> {
    if std::io::stdin().is_terminal() {
        eprintln!("  MCP             off (stdin is a terminal)");
        return None;
    }
    match facade(app.clone(), catalogue) {
        Ok(facade) => {
            eprintln!(
                "  MCP             stdio, artifacts in {}",
                facade.artifact_dir().display()
            );
            Some(facade)
        }
        Err(e) => {
            eprintln!("  MCP             off ({e})");
            None
        }
    }
}

/// Serves `facade` on stdin/stdout in a task of its own. Returns the task, which ends with the
/// session's outcome, and REST's shutdown signal, which fires once a session that a client used
/// has ended (stdin EOF). If stdin closes before any MCP request (stdin is `/dev/null`, say),
/// MCP stops quietly and the signal never fires: REST keeps running.
pub(crate) fn serve_stdio(
    facade: Facade<Router>,
) -> (
    JoinHandle<Result<(), openapi_mcp::Error>>,
    impl Future<Output = ()> + Send + 'static,
) {
    let (ended, stop) = tokio::sync::oneshot::channel::<()>();
    let session = tokio::spawn(async move {
        let used = facade.serve_stdio().await;
        match used {
            Ok(false) => {
                log::debug!("stdin closed before any MCP request; the REST API keeps running");
                std::future::pending::<()>().await;
            }
            Ok(true) => log::info!("the MCP client closed stdin; shutting down"),
            Err(_) => {}
        }
        drop(ended);
        used.map(|_| ())
    });
    let stop = async move {
        let _ = stop.await;
    };
    (session, stop)
}
