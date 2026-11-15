//! MCP beside the REST API (`src/mcp.rs`): the facade over the router in
//! process, and `serve` spawned with piped stdin and stdout, driven by rmcp's
//! client while REST answers on the same port. Gated behind the `mcp` feature
//! (in the default set). Run with: `cargo test -p urdf-robo-dresser --test mcp`.

#![cfg(feature = "mcp")]

use std::collections::BTreeSet;
use std::future::Future;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::Duration;

use openapi_mcp::rmcp::model::{CallToolRequestParams, CallToolResult};
use openapi_mcp::rmcp::service::{RoleClient, RunningService};
use openapi_mcp::rmcp::ServiceExt as _;
use openapi_mcp::Facade;
use serde_json::{json, Value};
use tokio::io::AsyncReadExt as _;
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;
use urdf_robo_dresser::server::openapi::ApiDoc;
use urdf_robo_dresser::server::{router, AppState};
use utoipa::OpenApi as _;

// ---------- scaffolding ----------

/// The catalogue of `tests/server.rs`: robot `r1` and tool `t1`, one link each.
fn seed_catalogue(root: &Path) {
    for (collection, name, link) in [
        ("urdf-robot-collection", "r1", "base"),
        ("urdf-tool-collection", "t1", "tcp"),
    ] {
        let dir = root.join(collection).join(name);
        std::fs::create_dir_all(&dir).unwrap();
        let urdf = format!(
            "<?xml version=\"1.0\"?>\n<robot name=\"{name}\">\n  <link name=\"{link}\"/>\n</robot>\n"
        );
        std::fs::write(dir.join(format!("{name}.urdf")), urdf).unwrap();
    }
}

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

/// The facade over a router on the seeded catalogue, and the tempdir holding
/// it. The facade's artifact directory is a new temporary one: remove it with
/// [`remove_artifact_dir`].
fn facade() -> (Facade<axum::Router>, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    seed_catalogue(dir.path());
    let root = dir.path().canonicalize().unwrap();
    let app = router(AppState::new(root.clone()));
    (urdf_robo_dresser::mcp::facade(app, &root).unwrap(), dir)
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

/// `serve --port <port> --catalogue <root>` with `stdin`, stdout piped, and
/// stderr collected by the returned task. Temporary files, the artifact
/// directory and the batch jobs' among them, go to `tmp`.
fn spawn_serve(root: &Path, port: u16, tmp: &Path, stdin: Stdio) -> (Child, JoinHandle<String>) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_urdf-robo-dresser"))
        .arg("serve")
        .args(["--port", &port.to_string()])
        .arg("--catalogue")
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

// ---------- in process ----------

#[tokio::test]
async fn every_operation_is_a_tool() {
    let (facade, _dir) = facade();
    let names: BTreeSet<String> = facade.tools().iter().map(|t| t.name.to_string()).collect();
    assert_eq!(names, operation_ids());
    assert_eq!(names.len(), 11);
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
            "dress_robot",
            json!({"robot_id": "../r1", "tool_id": "t1"}),
            "invalid_id",
        ),
        (
            "dress_robot",
            json!({"robot_id": "r1", "tool_id": "/etc"}),
            "invalid_id",
        ),
        (
            "validate_urdf",
            json!({"path": "../etc/passwd"}),
            "invalid_path",
        ),
        // One encoded segment (`a%2Fb`): the handler sees `a/b`, no UUID.
        ("get_job_status", json!({"id": "a/b"}), "invalid_job_id"),
    ] {
        let r = facade.call(tool, arguments).await.unwrap();
        assert_eq!(r.is_error, Some(true), "{tool}");
        let s = r.structured_content.unwrap();
        assert_eq!(s["http_status"], 400, "{tool}: {s}");
        assert_eq!(s["body"]["code"], code, "{tool}: {s}");
    }
    // A dot segment never leaves the facade.
    let r = facade
        .call("get_job_artifact", json!({"id": ".."}))
        .await
        .unwrap();
    assert_eq!(r.is_error, Some(true));
    assert_eq!(r.structured_content.unwrap()["code"], "invalid_arguments");
    remove_artifact_dir(&facade);
}

// ---------- over stdio ----------

#[tokio::test]
async fn serve_speaks_mcp_on_stdio_beside_rest() {
    let catalogue = tempfile::tempdir().unwrap();
    seed_catalogue(catalogue.path());
    let tmp = tempfile::tempdir().unwrap();
    let port = free_port();
    let base = format!("http://127.0.0.1:{port}");
    let (mut child, stderr) = spawn_serve(catalogue.path(), port, tmp.path(), Stdio::piped());
    let transport = (child.stdout.take().unwrap(), child.stdin.take().unwrap());
    let client = within(().serve(transport)).await.unwrap();
    let instructions = client.peer_info().unwrap().instructions.clone().unwrap();

    let tools = within(client.list_all_tools()).await.unwrap();
    let names: BTreeSet<String> = tools.iter().map(|t| t.name.to_string()).collect();
    assert_eq!(names, operation_ids());

    // A JSON array, wrapped as `items`.
    let r = call(&client, "list_catalogue_robots", json!({})).await;
    assert_eq!(r.structured_content.unwrap()["items"][0]["name"], "r1");

    // A batch started over MCP, polled until it completes.
    let r = call(&client, "start_batch", json!({})).await;
    let job = r.structured_content.unwrap()["job_id"]
        .as_str()
        .unwrap()
        .to_string();
    within(async {
        loop {
            let r = call(&client, "get_job_status", json!({"id": job})).await;
            let state = r.structured_content.unwrap()["state"].clone();
            if state == "completed" {
                break;
            }
            assert_ne!(state, "failed");
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await;

    // REST answers on the same port meanwhile, from the same state: it knows the job.
    let status: Value = reqwest::get(format!("{base}/api/v1/jobs/{job}"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(status["job_id"], job.as_str());
    assert_eq!(status["state"], "completed");
    let health = reqwest::get(format!("{base}/healthz")).await.unwrap();
    assert!(health.status().is_success());

    // The job's zip, saved in the artifact directory: a new one in TMPDIR, named
    // in the instructions and in the banner.
    let r = call(&client, "get_job_artifact", json!({"id": job})).await;
    let saved = r.structured_content.unwrap();
    assert_eq!(saved["content_type"], "application/zip");
    assert_eq!(saved["filename"], format!("batch-{job}.zip"));
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
    assert!(dir_name.starts_with("urdf-robo-dresser-mcp-"), "{dir_name}");
    assert!(instructions.contains(&artifacts.display().to_string()));

    // Cancelling closes the client's end: serve's stdin hits EOF, the session
    // ends, and REST shuts down with it.
    client.cancel().await.unwrap();
    let status = within(child.wait()).await.unwrap();
    let stderr = within(stderr).await.unwrap();
    assert!(status.success(), "{status}\n{stderr}");
    let banner = format!(
        "MCP             stdio, artifacts in {}",
        artifacts.display()
    );
    assert!(stderr.contains(&banner), "{stderr}");
}

#[tokio::test]
async fn with_stdin_at_eof_rest_keeps_running() {
    let catalogue = tempfile::tempdir().unwrap();
    seed_catalogue(catalogue.path());
    let tmp = tempfile::tempdir().unwrap();
    let port = free_port();
    let (mut child, stderr) = spawn_serve(catalogue.path(), port, tmp.path(), Stdio::null());
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
        stderr.contains("MCP             stdio, artifacts in "),
        "{stderr}"
    );
}
