//! End-to-end integration test driving `compose.run` against the *built*
//! `robo-fitting-room` binary over its `stdio-serve` transport.
//!
//! Gated behind the `stdio` feature. Run with:
//! `cargo test --features stdio --test stdio_compose_e2e`.
//!
//! ## Property 6: Artifact existence
//! **Validates: Requirement 4.1**
//!
//! Unlike `tests/stdio_compose_artifact.rs` (which exercises `dispatch()`
//! in-process), this test spawns the real binary with
//! `stdio-serve --asset-root <fixture>`, feeds a single `compose.run`
//! request line on stdin, closes stdin, and inspects the NDJSON response
//! line on stdout. It asserts the returned `ComposeArtifact.zipPath`
//! exists on disk, is non-empty, its on-disk size equals the reported
//! `bytes`, and the file opens as a valid zip archive.

#![cfg(feature = "stdio")]

mod common;

use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Command, Stdio};

use common::populate_assets;
use serde_json::Value;

// ---------------------------------------------------------------------------
// End-to-end: spawn the built binary and drive `compose.run` over stdio, on
// `tests/common`'s synthetic asset tree (a small arm, a fixture, a floor plan
// and the one-robot `fitted-robots/scene.yaml`).
// ---------------------------------------------------------------------------

#[test]
fn compose_run_e2e_returns_valid_zip_artifact() {
    let dir = tempfile::tempdir().expect("create asset tempdir");
    populate_assets(dir.path());

    let bin = env!("CARGO_BIN_EXE_robo-fitting-room");
    let mut child = Command::new(bin)
        .arg("stdio-serve")
        .arg("--asset-root")
        .arg(dir.path())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn robo-fitting-room stdio-serve");

    // Send one compose.run request, then close stdin so the loop exits on EOF.
    let request =
        "{\"id\":1,\"method\":\"compose.run\",\"params\":{\"manifestPath\":\"fitted-robots/scene.yaml\",\"fullRobots\":false,\"fullTools\":true}}\n";
    {
        let mut stdin = child.stdin.take().expect("child stdin piped");
        stdin
            .write_all(request.as_bytes())
            .expect("write compose.run request");
        stdin.flush().expect("flush request");
        // Dropping `stdin` here closes it, signalling EOF to the child.
    }

    // Read exactly one NDJSON response line from stdout.
    let stdout = child.stdout.take().expect("child stdout piped");
    let mut reader = BufReader::new(stdout);
    let mut line = String::new();
    let n = reader.read_line(&mut line).expect("read response line");
    assert!(n > 0, "expected one NDJSON response line, got EOF");

    // The child should exit cleanly (code 0) once stdin closes.
    let status = child.wait().expect("wait for child exit");

    let response: Value = serde_json::from_str(line.trim_end())
        .unwrap_or_else(|e| panic!("response line must be valid JSON ({e}): {line}"));

    assert_eq!(response["id"].as_u64(), Some(1), "response echoes request id");

    let result = response
        .get("result")
        .unwrap_or_else(|| panic!("expected a success `result`, got: {response}"));

    // Property 6 / Requirement 4.1: the artifact names a real, non-empty,
    // valid zip whose on-disk size equals the reported `bytes`.
    let zip_path = result["zipPath"]
        .as_str()
        .unwrap_or_else(|| panic!("ComposeArtifact missing zipPath: {result}"));
    let bytes = result["bytes"]
        .as_u64()
        .unwrap_or_else(|| panic!("ComposeArtifact missing bytes: {result}"));
    assert_eq!(
        result["downloadName"].as_str(),
        Some("scene.zip"),
        "unexpected downloadName in {result}"
    );

    let path = PathBuf::from(zip_path);
    assert!(
        path.exists(),
        "returned zipPath must exist on disk: {}",
        path.display()
    );

    let actual = std::fs::metadata(&path).expect("temp zip metadata").len();
    assert_eq!(
        actual, bytes,
        "reported bytes ({bytes}) must equal real on-disk size ({actual})"
    );
    assert!(bytes > 0, "composed zip must be non-empty");

    let f = std::fs::File::open(&path).expect("open returned temp zip");
    let archive = zip::ZipArchive::new(f).expect("returned file must be a valid zip archive");
    assert!(archive.len() > 0, "zip archive must contain entries");

    // Clean up the temp zip (ownership transferred to us, the caller).
    std::fs::remove_file(&path).ok();

    assert!(
        status.success(),
        "stdio-serve should exit 0 on stdin EOF, got: {status}"
    );
}
