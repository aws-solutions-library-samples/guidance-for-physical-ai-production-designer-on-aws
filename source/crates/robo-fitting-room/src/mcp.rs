//! MCP beside the REST API. When an MCP client spawns `serve` (stdin is not a terminal), the
//! process also speaks MCP on stdin/stdout: every operation of [`ApiDoc`] is a tool, answered in
//! process by the router `serve` serves over REST, so both share one `AppState` and the same
//! handlers, validation and asset-root sandbox. MCP opens no socket. The facade is the
//! `openapi-mcp` crate; it saves zips in a new 0700 temporary directory per process.

use std::future::Future;
use std::io::IsTerminal as _;
use std::path::Path;

use axum::Router;
use openapi_mcp::{Facade, Options};
use tokio::task::JoinHandle;
use utoipa::OpenApi as _;

use crate::server::openapi::ApiDoc;

/// The POST operations that only read.
const READ_ONLY: [&str; 4] = [
    "validate_scene",
    "load_scene",
    "list_urdf_joints",
    "get_floor_plan_segments",
];

/// Every operation of [`ApiDoc`] as an MCP tool, answered by `app`: give it the router `serve`
/// serves, and MCP and REST share one `AppState`. The instructions name `asset_root`.
pub fn facade(app: Router, asset_root: &Path) -> Result<Facade<Router>, openapi_mcp::Error> {
    let spec = serde_json::to_value(ApiDoc::openapi())
        .map_err(|e| openapi_mcp::Error::Spec(e.to_string()))?;
    let mut options = Options::new(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    options.read_only = READ_ONLY.map(String::from).to_vec();
    options.instructions = Some(format!(
        "Every `path` and `manifest_path` is relative to the asset root {}: fitted-robots/ holds \
         the scene manifests (YAML), urdf-dressed-robots/<id>/<id>.urdf the robots, \
         urdf-fixture-collection/<id>/<id>.urdf the fixtures and floor-plans/ the OBJ floor \
         plans. Absolute paths and `..` are rejected. Inside a manifest, paths are relative to \
         the manifest's folder (../urdf-dressed-robots/<id>/<id>.urdf from fitted-robots/); \
         validate_scene reads them as if the manifest were in fitted-robots/. Typical flow: \
         list_robots, list_fixtures and list_floor_plans; write the manifest with save_scene \
         under fitted-robots/; check it with validate_scene; then compose_scene with its path \
         as manifest_path for the USDA scene zip, or compose_urdf_bundle for its source URDFs.",
        asset_root.display()
    ));
    Facade::new(app, &spec, options)
}

/// The facade `serve` runs on stdio beside REST, after printing the banner's MCP line. `None`
/// when stdin is a terminal (a person started `serve`, not an MCP client) or the facade can't
/// be built; REST runs the same either way.
pub(crate) fn on_stdio(app: &Router, asset_root: &Path) -> Option<Facade<Router>> {
    if std::io::stdin().is_terminal() {
        eprintln!("  MCP            off (stdin is a terminal)");
        return None;
    }
    match facade(app.clone(), asset_root) {
        Ok(facade) => {
            eprintln!(
                "  MCP            stdio, artifacts in {}",
                facade.artifact_dir().display()
            );
            Some(facade)
        }
        Err(e) => {
            eprintln!("  MCP            off ({e})");
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
