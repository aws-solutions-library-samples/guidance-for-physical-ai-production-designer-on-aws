//! Unit/integration tests for the `stdio` transport's compose artifact
//! temp-file behavior.
//!
//! Gated behind the `stdio` feature. Run with:
//! `cargo test --features stdio --test stdio_compose_artifact`.
//!
//! ## Property 6: Artifact existence
//! **Validates: Requirements 4.1, 4.2, 4.3, 4.4**
//!
//! On a successful `compose.run` / `compose.urdfBundle` dispatch, the
//! returned `ComposeArtifact` names a file that exists on disk whose real
//! size equals the reported `bytes` exactly (Req 4.1), the file is not
//! deleted before the response is returned (Req 4.3), and it opens as a
//! valid zip archive. On a validation/parse failure, dispatch returns an
//! error and no temp zip file path is referenced anywhere in the error
//! response (Req 4.4).

#![cfg(feature = "stdio")]

mod common;

use std::path::PathBuf;

use common::populate_assets;
use robo_fitting_room::stdio::dispatch::dispatch;
use robo_fitting_room::transport_common::AppState;
use serde_json::{json, Value};

// ---------------------------------------------------------------------------
// Fixture asset tree: `tests/common`'s synthetic one (a small arm, a fixture,
// a floor plan and the one-robot `fitted-robots/scene.yaml`)
// ---------------------------------------------------------------------------

/// Stand up an `AppState` over a freshly-seeded asset tempdir. The returned
/// `TempDir` guard must be kept alive for the duration of the test.
fn state_over_fixture() -> (AppState, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    populate_assets(dir.path());
    let state = AppState::new(dir.path().to_path_buf(), None);
    (state, dir)
}

/// Extract and validate a `ComposeArtifact` JSON `Value`: the file at
/// `zipPath` must exist with an on-disk size exactly equal to `bytes`, and
/// it must open as a valid zip archive. Returns the `zipPath` so the caller
/// can clean it up.
fn assert_artifact_on_disk(result: &Value, expected_download: &str) -> PathBuf {
    let zip_path = result["zipPath"]
        .as_str()
        .unwrap_or_else(|| panic!("ComposeArtifact missing zipPath: {result}"));
    let bytes = result["bytes"]
        .as_u64()
        .unwrap_or_else(|| panic!("ComposeArtifact missing bytes: {result}"));
    assert_eq!(
        result["downloadName"].as_str(),
        Some(expected_download),
        "unexpected downloadName in {result}"
    );

    let path = PathBuf::from(zip_path);
    assert!(
        path.exists(),
        "temp zip must exist on disk (Req 4.3): {}",
        path.display()
    );

    // Req 4.1: the reported `bytes` equals the file's real on-disk size.
    let actual = std::fs::metadata(&path)
        .expect("temp zip metadata")
        .len();
    assert_eq!(
        actual, bytes,
        "reported bytes ({bytes}) must equal real on-disk size ({actual})"
    );

    // The artifact opens as a valid, non-empty zip archive.
    assert!(bytes > 0, "composed zip must be non-empty");
    let f = std::fs::File::open(&path).expect("open temp zip");
    let archive = zip::ZipArchive::new(f).expect("temp zip must be a valid zip archive");
    assert!(archive.len() > 0, "zip archive must contain entries");

    path
}

/// Assert that a dispatch error carries no reference to a temp zip file
/// path (Req 4.4). We serialize the whole error and confirm none of its
/// fields name a `.zip` file or a `zipPath`/`path`-shaped key.
fn assert_no_temp_file_referenced(err: &robo_fitting_room::stdio::protocol::RpcError) {
    let serialized = serde_json::to_value(err).expect("error serializes");
    // No `zipPath` field anywhere in the error response.
    assert!(
        serialized.get("zipPath").is_none(),
        "error response must not carry a zipPath: {serialized}"
    );
    if let Some(data) = serialized.get("data") {
        assert!(
            data.get("zipPath").is_none() && data.get("path").is_none(),
            "error data must not carry a path/zipPath: {data}"
        );
    }
    // Defensive: no substring naming a composed temp artifact leaks through
    // the message/hint either.
    let text = serialized.to_string();
    assert!(
        !text.contains("rfr-compose-"),
        "error must not reference a composed temp file name: {text}"
    );
}

// ---------------------------------------------------------------------------
// SUCCESS — compose.run
// ---------------------------------------------------------------------------

#[test]
fn compose_run_success_writes_temp_zip_with_exact_size() {
    let (state, _dir) = state_over_fixture();
    let params = json!({
        "manifestPath": "fitted-robots/scene.yaml",
        "fullRobots": false,
        "fullTools": true,
    });

    let result = dispatch(&state, "compose.run", params)
        .expect("compose.run over a valid manifest must succeed");

    let zip_path = assert_artifact_on_disk(&result, "scene.zip");

    // Cleanup: ownership of the temp file transfers to the caller.
    std::fs::remove_file(&zip_path).ok();
}

// ---------------------------------------------------------------------------
// SUCCESS — compose.urdfBundle
// ---------------------------------------------------------------------------

#[test]
fn compose_urdf_bundle_success_writes_temp_zip_with_exact_size() {
    let (state, _dir) = state_over_fixture();
    let params = json!({ "manifestPath": "fitted-robots/scene.yaml" });

    let result = dispatch(&state, "compose.urdfBundle", params)
        .expect("compose.urdfBundle over a valid manifest must succeed");

    let zip_path = assert_artifact_on_disk(&result, "scene-urdf.zip");

    std::fs::remove_file(&zip_path).ok();
}

// ---------------------------------------------------------------------------
// FAILURE — compose.run references no temp file
// ---------------------------------------------------------------------------

#[test]
fn compose_run_failure_references_no_temp_file() {
    let (state, _dir) = state_over_fixture();
    // A well-formed relative path that resolves under asset_root but does
    // not exist → parse_scene fails before any temp file is created.
    let params = json!({ "manifestPath": "fitted-robots/does-not-exist.yaml" });

    let err = dispatch(&state, "compose.run", params)
        .expect_err("compose.run over a missing manifest must fail");

    assert_no_temp_file_referenced(&err);
}

#[test]
fn compose_run_failure_on_traversal_references_no_temp_file() {
    let (state, _dir) = state_over_fixture();
    // Sandbox-escaping path → rejected by resolve_asset before any work.
    let params = json!({ "manifestPath": "../secret.yaml" });

    let err = dispatch(&state, "compose.run", params)
        .expect_err("compose.run over a traversal path must fail");

    assert_no_temp_file_referenced(&err);
}

// ---------------------------------------------------------------------------
// FAILURE — compose.urdfBundle references no temp file
// ---------------------------------------------------------------------------

#[test]
fn compose_urdf_bundle_failure_references_no_temp_file() {
    let (state, _dir) = state_over_fixture();
    let params = json!({ "manifestPath": "fitted-robots/does-not-exist.yaml" });

    let err = dispatch(&state, "compose.urdfBundle", params)
        .expect_err("compose.urdfBundle over a missing manifest must fail");

    assert_no_temp_file_referenced(&err);
}
