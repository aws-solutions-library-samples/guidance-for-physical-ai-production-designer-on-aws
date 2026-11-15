//! MCP beside the REST API (`src/mcp.rs`): the facade over the router in
//! process, and `serve` spawned with piped stdin and stdout, driven by rmcp's
//! client while REST answers on the same port. Gated behind the `mcp` feature
//! (in the default set). Run with: `cargo test -p robo-fitting-room --test mcp`.

#![cfg(feature = "mcp")]

mod common;

use std::collections::BTreeSet;
use std::future::Future;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::Duration;

use common::populate_assets;
use openapi_mcp::rmcp::model::{CallToolRequestParams, CallToolResult};
use openapi_mcp::rmcp::service::{RoleClient, RunningService};
use openapi_mcp::rmcp::ServiceExt as _;
use openapi_mcp::Facade;
use robo_fitting_room::server::openapi::ApiDoc;
use robo_fitting_room::server::{router, AppState};
use serde_json::{json, Value};
use tokio::io::AsyncReadExt as _;
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;
use utoipa::OpenApi as _;

// -----------------------------------------------------------------------------
// Test scaffolding
// -----------------------------------------------------------------------------

/// The spec's operationIds.
fn operation_ids() -> BTreeSet<String> {
    let spec = serde_json::to_value(ApiDoc::openapi()).unwrap();
    spec["paths"]
        .as_object()
        .unwrap()
        .values()
        .flat_map(|item| item.as_object().unwrap().values())
        .filter_map(|op| op["operationId"].as_str().map(String::from))
        .collect()
}

/// The facade over a router on `tests/common`'s synthetic asset tree, and the
/// tempdir holding the tree. The facade's artifact directory is a new
/// temporary one: remove it with [`remove_artifact_dir`].
fn facade() -> (Facade<axum::Router>, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    populate_assets(dir.path());
    let root = dir.path().canonicalize().unwrap();
    let app = router(AppState::new(root.clone(), None));
    (robo_fitting_room::mcp::facade(app, &root).unwrap(), dir)
}

fn remove_artifact_dir(facade: &Facade<axum::Router>) {
    std::fs::remove_dir(facade.artifact_dir()).unwrap();
}

/// A TCP port that was free a moment ago.
fn free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// `serve --port <port> --asset-root <root>` with `stdin`, stdout piped, and
/// stderr collected by the returned task. Temporary files, the artifact
/// directory among them, go to `tmp`.
fn spawn_serve(root: &Path, port: u16, tmp: &Path, stdin: Stdio) -> (Child, JoinHandle<String>) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_robo-fitting-room"))
        .arg("serve")
        .args(["--port", &port.to_string()])
        .arg("--asset-root")
        .arg(root)
        .env("TMPDIR", tmp)
        .env("TMP", tmp)
        .env("TEMP", tmp)
        .stdin(stdin)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .expect("spawn serve");
    let mut stderr = child.stderr.take().unwrap();
    let stderr = tokio::spawn(async move {
        let mut text = String::new();
        let _ = stderr.read_to_string(&mut text).await;
        text
    });
    (child, stderr)
}

/// `future`, or a panic after 30 s.
async fn within<F: Future>(future: F) -> F::Output {
    tokio::time::timeout(Duration::from_secs(30), future)
        .await
        .expect("timed out")
}

/// Calls `name` with `arguments` over MCP and expects a result that is no error.
async fn call(
    client: &RunningService<RoleClient, ()>,
    name: &'static str,
    arguments: Value,
) -> CallToolResult {
    let Value::Object(arguments) = arguments else {
        panic!("arguments must be an object")
    };
    let params = CallToolRequestParams::new(name).with_arguments(arguments);
    let r = within(client.call_tool(params)).await.unwrap();
    assert_eq!(r.is_error, Some(false), "{name}: {r:?}");
    r
}

// -----------------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------------

#[tokio::test]
async fn every_operation_is_a_tool() {
    let (facade, _dir) = facade();
    let names: BTreeSet<String> = facade.tools().iter().map(|t| t.name.to_string()).collect();
    assert_eq!(names, operation_ids());
    assert_eq!(names.len(), 13);
    for tool in facade.tools() {
        assert_eq!(
            tool.schema_as_json_value()["type"],
            "object",
            "{}",
            tool.name
        );
    }
    remove_artifact_dir(&facade);
}

#[tokio::test]
async fn traversal_arguments_are_http_400_errors() {
    let (facade, _dir) = facade();
    for (tool, arguments, code) in [
        (
            "list_urdf_joints",
            json!({"path": "../etc/passwd"}),
            "invalid_path",
        ),
        (
            "get_floor_plan_segments",
            json!({"path": "/etc/passwd"}),
            "invalid_path",
        ),
        (
            "load_scene",
            json!({"path": "fitted-robots/../../x.yaml"}),
            "invalid_path",
        ),
        (
            "save_scene",
            json!({"path": "../x.yaml", "yaml": "name: x\n"}),
            "invalid_path",
        ),
        (
            "compose_scene",
            json!({"manifest_path": "../scene.yaml"}),
            "invalid_manifest_path",
        ),
        (
            "compose_urdf_bundle",
            json!({"manifest_path": "/etc/passwd"}),
            "invalid_manifest_path",
        ),
    ] {
        let r = facade.call(tool, arguments).await.unwrap();
        assert_eq!(r.is_error, Some(true), "{tool}");
        let s = r.structured_content.unwrap();
        assert_eq!(s["http_status"], 400, "{tool}: {s}");
        assert_eq!(s["body"]["code"], code, "{tool}: {s}");
    }
    remove_artifact_dir(&facade);
}

#[tokio::test]
async fn serve_speaks_mcp_on_stdio_beside_rest() {
    let assets = tempfile::tempdir().unwrap();
    populate_assets(assets.path());
    let tmp = tempfile::tempdir().unwrap();
    let port = free_port();
    let (mut child, stderr) = spawn_serve(assets.path(), port, tmp.path(), Stdio::piped());
    let transport = (child.stdout.take().unwrap(), child.stdin.take().unwrap());
    let client = within(().serve(transport)).await.unwrap();
    let instructions = client.peer_info().unwrap().instructions.clone().unwrap();

    let tools = within(client.list_all_tools()).await.unwrap();
    let names: BTreeSet<String> = tools.iter().map(|t| t.name.to_string()).collect();
    assert_eq!(names, operation_ids());

    // A JSON array, wrapped as `items`.
    let r = call(&client, "list_robots", json!({})).await;
    assert_eq!(r.structured_content.unwrap()["items"][0]["id"], "r1");

    // A zip, saved in the artifact directory: a new one in TMPDIR, named in the
    // instructions and in the banner.
    let r = call(
        &client,
        "compose_scene",
        json!({"manifest_path": "fitted-robots/scene.yaml"}),
    )
    .await;
    let saved = r.structured_content.unwrap();
    assert_eq!(saved["content_type"], "application/zip");
    assert_eq!(saved["filename"], "scene.zip");
    let path = PathBuf::from(saved["path"].as_str().unwrap());
    let zip = std::fs::read(&path).unwrap();
    assert!(zip.starts_with(b"PK"), "{}", path.display());
    assert_eq!(saved["bytes"], zip.len());
    let artifacts = path.parent().unwrap().to_path_buf();
    assert!(artifacts.starts_with(tmp.path().canonicalize().unwrap()));
    let dir_name = artifacts
        .file_name()
        .unwrap()
        .to_string_lossy()
        .into_owned();
    assert!(dir_name.starts_with("robo-fitting-room-mcp-"), "{dir_name}");
    assert!(instructions.contains(&artifacts.display().to_string()));

    // REST answers on the same port meanwhile.
    let health: Value = reqwest::get(format!("http://127.0.0.1:{port}/healthz"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(health["status"], "ok");

    // Cancelling closes the client's end: serve's stdin hits EOF, the session
    // ends, and REST shuts down with it.
    client.cancel().await.unwrap();
    let status = within(child.wait()).await.unwrap();
    let stderr = within(stderr).await.unwrap();
    assert!(status.success(), "{status}\n{stderr}");
    let banner = format!("MCP            stdio, artifacts in {}", artifacts.display());
    assert!(stderr.contains(&banner), "{stderr}");
}

#[tokio::test]
async fn with_stdin_at_eof_rest_keeps_running() {
    let assets = tempfile::tempdir().unwrap();
    populate_assets(assets.path());
    let tmp = tempfile::tempdir().unwrap();
    let port = free_port();
    let (mut child, stderr) = spawn_serve(assets.path(), port, tmp.path(), Stdio::null());
    let url = format!("http://127.0.0.1:{port}/healthz");
    within(async {
        while !reqwest::get(&url)
            .await
            .is_ok_and(|r| r.status().is_success())
        {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await;

    // By now MCP has read the end of stdin, without a request, and stopped.
    tokio::time::sleep(Duration::from_secs(1)).await;
    assert!(child.try_wait().unwrap().is_none(), "serve exited");
    assert!(reqwest::get(&url).await.unwrap().status().is_success());

    child.kill().await.unwrap();
    // stdout carries MCP messages only, and there were none.
    let mut stdout = Vec::new();
    child
        .stdout
        .take()
        .unwrap()
        .read_to_end(&mut stdout)
        .await
        .unwrap();
    assert!(stdout.is_empty(), "{}", String::from_utf8_lossy(&stdout));
    let stderr = within(stderr).await.unwrap();
    assert!(
        stderr.contains("MCP            stdio, artifacts in "),
        "{stderr}"
    );
}
