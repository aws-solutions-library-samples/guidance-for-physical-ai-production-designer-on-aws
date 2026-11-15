//! End-to-end integration test for the `stdio-serve` subcommand.
//!
//! Spawns the actual built binary with
//! `stdio-serve --asset-root <fixture dir>`, feeds a batch of
//! newline-delimited JSON (NDJSON) requests on stdin, closes stdin (EOF),
//! and asserts the NDJSON responses that come back on stdout.
//!
//! Gated behind the `stdio` feature (in the default set), so
//! `CARGO_BIN_EXE_robo-fitting-room` — which Cargo provides to integration
//! tests — points at a binary that has the `stdio-serve` subcommand wired
//! in. Run with either:
//!   `cargo test --features stdio --test stdio_integration`
//!   `cargo test --test stdio_integration`   (stdio is a default feature)
//!
//! Coverage:
//! - handshake            → `health.check`           (Req 1.1, 1.2, 3.1)
//! - a normal call        → `assets.listRobots`      (Req 3.1)
//! - malformed JSON line  → parse error, id null     (Req 2.2, 3.2)
//! - unknown method       → method_not_found, id 3   (Req 3.3)
//! - invalid params       → invalid_params, id 4     (Req 3.4)
//! - stdin EOF            → process exits 0           (Req 2.4)
//! - framing              → one JSON value per line   (Req 2.1)
//!
//! **Validates: Requirements 1.1, 1.2, 2.1, 2.2, 2.3, 2.4, 3.1, 3.2, 3.3, 3.4**

#![cfg(feature = "stdio")]

mod common;

use std::io::{BufRead, BufReader, Read, Write};
use std::process::{Command, Stdio};

use common::populate_assets;
use serde_json::Value;

// Protocol error codes (mirrors src/stdio/protocol.rs).
const CODE_PARSE_ERROR: i64 = -32700;
const CODE_METHOD_NOT_FOUND: i64 = -32601;
const CODE_INVALID_PARAMS: i64 = -32602;

// The fixture asset tree is `tests/common`'s synthetic one: a small arm, a
// fixture, a floor plan and `fitted-robots/scene.yaml`.

// ---------------------------------------------------------------------------
// Test
// ---------------------------------------------------------------------------

/// Drive the whole request menu through one spawned `stdio-serve` process
/// and assert the framed responses plus a clean exit on stdin EOF.
#[test]
fn stdio_serve_handles_request_menu_and_exits_on_eof() {
    let dir = tempfile::tempdir().unwrap();
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
        .expect("spawn stdio-serve child process");

    // Feed the full request batch, then close stdin to signal EOF.
    // Blank lines interleaved to confirm they are discarded without a
    // response (Req 2.3): they must NOT produce extra output lines.
    let input = concat!(
        "{\"id\":1,\"method\":\"health.check\",\"params\":{}}\n",
        "\n",
        "{\"id\":2,\"method\":\"assets.listRobots\",\"params\":{}}\n",
        "   \n",
        "not json\n",
        "{\"id\":3,\"method\":\"does.notExist\",\"params\":{}}\n",
        "{\"id\":4,\"method\":\"urdf.joints\",\"params\":123}\n",
    );
    {
        let mut stdin = child.stdin.take().expect("child stdin");
        stdin.write_all(input.as_bytes()).expect("write stdin");
        // Dropping `stdin` here closes the pipe → the child sees EOF.
    }

    // Read every byte of stdout so we can assert on exact framing.
    let mut raw = String::new();
    child
        .stdout
        .take()
        .expect("child stdout")
        .read_to_string(&mut raw)
        .expect("read stdout");

    // Req 2.4: the process terminates on stdin EOF with exit code 0.
    let status = child.wait().expect("wait for child");
    let mut stderr = String::new();
    if let Some(mut e) = child.stderr.take() {
        let _ = e.read_to_string(&mut stderr);
    }
    assert!(
        status.success(),
        "stdio-serve must exit 0 on stdin EOF; status={status:?} stderr={stderr}"
    );

    // Framing (Req 2.1): split on '\n'; every non-empty chunk must be
    // exactly one complete JSON value with no embedded newline.
    let lines: Vec<&str> = raw.lines().filter(|l| !l.trim().is_empty()).collect();
    assert_eq!(
        lines.len(),
        5,
        "expected exactly 5 response lines (blank input lines produce none), got {}: {raw:?}",
        lines.len()
    );

    let responses: Vec<Value> = lines
        .iter()
        .map(|l| {
            assert!(
                !l.contains('\n'),
                "a single response line must not contain an embedded newline: {l:?}"
            );
            serde_json::from_str::<Value>(l)
                .unwrap_or_else(|e| panic!("each response line must be one JSON value: {e} in {l:?}"))
        })
        .collect();

    // Responses are emitted in request-arrival order (blank lines skipped):
    // 1) health.check  2) assets.listRobots  3) parse error  4) unknown
    // method  5) invalid params.

    // (1) Handshake — health.check → result {status:"ok", version:...}, id 1.
    let r0 = &responses[0];
    assert_eq!(r0["id"], 1, "handshake response must echo id 1: {r0}");
    assert!(r0.get("error").is_none(), "handshake must not error: {r0}");
    assert_eq!(
        r0["result"]["status"], "ok",
        "health.check result.status must be \"ok\": {r0}"
    );
    assert!(
        r0["result"]["version"].is_string(),
        "health.check result.version must be a string: {r0}"
    );

    // (2) Normal call — assets.listRobots → id 2, result array with r1.
    let r1 = &responses[1];
    assert_eq!(r1["id"], 2, "listRobots response must echo id 2: {r1}");
    assert!(r1.get("error").is_none(), "listRobots must not error: {r1}");
    let robots = r1["result"]
        .as_array()
        .unwrap_or_else(|| panic!("listRobots result must be an array: {r1}"));
    assert_eq!(robots.len(), 1, "expected the one seeded robot: {r1}");
    assert_eq!(robots[0]["id"], "r1", "seeded robot id must be r1: {r1}");

    // (3) Malformed JSON line → parse error (-32700), id null/absent.
    let r2 = &responses[2];
    assert!(
        r2["id"].is_null(),
        "parse-error response id must be null/absent: {r2}"
    );
    assert_eq!(
        r2["error"]["code"], CODE_PARSE_ERROR,
        "malformed JSON must map to parse_error {CODE_PARSE_ERROR}: {r2}"
    );

    // (4) Unknown method → method_not_found (-32601), id 3.
    let r3 = &responses[3];
    assert_eq!(r3["id"], 3, "unknown-method response must echo id 3: {r3}");
    assert_eq!(
        r3["error"]["code"], CODE_METHOD_NOT_FOUND,
        "unknown method must map to method_not_found {CODE_METHOD_NOT_FOUND}: {r3}"
    );

    // (5) Invalid params → invalid_params (-32602), id 4.
    let r4 = &responses[4];
    assert_eq!(r4["id"], 4, "invalid-params response must echo id 4: {r4}");
    assert_eq!(
        r4["error"]["code"], CODE_INVALID_PARAMS,
        "non-object params must map to invalid_params {CODE_INVALID_PARAMS}: {r4}"
    );
}

/// A manifest with keys the 2D editor carries along without working with
/// them (a pedestal, a payload, footprint layers) and keys nothing knows.
const MANIFEST_WITH_OTHER_KEYS: &str = r#"name: "keys"
layout_rev: 7
robots:
  - robot_id: "R1"
    dressed_urdf: "../urdf-dressed-robots/r1/r1.urdf"
    position: { x: 1.25, y: -0.5, z: 0.0 }
    orientation: { a: 12.345678, b: 0.0, c: 0.0 }
    home_pose_deg: [0.0, 12.345678]
    pedestal: { height_mm: 600.0, diameter_mm: 900.0, material: concrete }
    payload: { mass_kg: 42.5, center_of_mass_m: { x: 0.1, y: 0.0, z: 0.25 }, inertia_kgm2: { ixx: 1.5, iyy: 2.0, izz: 2.5 }, source: datasheet }
    controller: { address: 10.0.0.7, cycle_ms: 4 }
floor_plan:
  geometry_file: "../floor-plans/empty.obj"
  scale: 1.0
  footprint_layers: [ FOOTPRINTS ]
  line_width_m: 0.02
"#;

/// Every stdio method that writes or rewrites a manifest keeps the keys it
/// doesn't work with: `scene.save` writes the YAML verbatim, `scene.load`
/// returns it byte for byte, `compose.validate` accepts it, and the URDF
/// bundle's copy changes only the asset paths.
#[test]
fn stdio_save_load_and_bundle_keep_every_manifest_key() {
    let dir = tempfile::tempdir().unwrap();
    populate_assets(dir.path());

    let path = "fitted-robots/keys.yaml";
    let requests = [
        serde_json::json!({"id": 1, "method": "scene.save", "params": {"path": path, "yaml": MANIFEST_WITH_OTHER_KEYS}}),
        serde_json::json!({"id": 2, "method": "scene.load", "params": {"path": path}}),
        serde_json::json!({"id": 3, "method": "compose.validate", "params": {"yaml": MANIFEST_WITH_OTHER_KEYS}}),
        serde_json::json!({"id": 4, "method": "compose.urdfBundle", "params": {"manifestPath": path}}),
    ];
    let mut child = Command::new(env!("CARGO_BIN_EXE_robo-fitting-room"))
        .arg("stdio-serve")
        .arg("--asset-root")
        .arg(dir.path())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn stdio-serve child process");
    {
        let mut stdin = child.stdin.take().expect("child stdin");
        for request in &requests {
            writeln!(stdin, "{request}").expect("write stdin");
        }
    }
    let mut raw = String::new();
    child.stdout.take().expect("child stdout").read_to_string(&mut raw).expect("read stdout");
    assert!(child.wait().expect("wait for child").success());
    let responses: Vec<Value> = raw.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
    assert_eq!(responses.len(), 4, "{raw}");
    for (i, r) in responses.iter().enumerate() {
        assert_eq!(r["id"], i + 1, "{r}");
        assert!(r.get("error").is_none(), "{r}");
    }

    assert_eq!(responses[0]["result"]["bytes"], MANIFEST_WITH_OTHER_KEYS.len() as u64);
    assert_eq!(responses[1]["result"]["yaml"], MANIFEST_WITH_OTHER_KEYS, "load returns the saved bytes");
    assert_eq!(responses[2]["result"]["ok"], true, "{}", responses[2]);

    let zip_path = responses[3]["result"]["zipPath"].as_str().unwrap().to_owned();
    let mut z = zip::ZipArchive::new(std::fs::File::open(&zip_path).unwrap()).unwrap();
    let mut bundled = String::new();
    z.by_name("keys.yaml").unwrap().read_to_string(&mut bundled).unwrap();
    std::fs::remove_file(&zip_path).ok();
    let bundled: serde_yaml::Value = serde_yaml::from_str(&bundled).unwrap();
    let mut expected: serde_yaml::Value = serde_yaml::from_str(MANIFEST_WITH_OTHER_KEYS).unwrap();
    let rewritten = bundled["robots"][0]["dressed_urdf"].as_str().unwrap();
    assert_ne!(rewritten, "../urdf-dressed-robots/r1/r1.urdf", "the bundle's own path");
    expected["robots"][0]["dressed_urdf"] = rewritten.into();
    expected["floor_plan"]["geometry_file"] = bundled["floor_plan"]["geometry_file"].clone();
    assert_eq!(bundled, expected, "the bundle's scene keeps every other key");
}

/// Guard against a subtle regression: reading stdout line-by-line (as the
/// extension host does) must yield each response as its own complete line.
/// This mirrors the framing invariant from a consumer's perspective.
#[test]
fn stdio_serve_responses_are_line_delimited() {
    let dir = tempfile::tempdir().unwrap();
    populate_assets(dir.path());

    let bin = env!("CARGO_BIN_EXE_robo-fitting-room");
    let mut child = Command::new(bin)
        .arg("stdio-serve")
        .arg("--asset-root")
        .arg(dir.path())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn stdio-serve child process");

    {
        let mut stdin = child.stdin.take().expect("child stdin");
        stdin
            .write_all(b"{\"id\":7,\"method\":\"health.check\",\"params\":{}}\n")
            .unwrap();
    }

    let stdout = child.stdout.take().expect("child stdout");
    let mut reader = BufReader::new(stdout);
    let mut first = String::new();
    let n = reader.read_line(&mut first).expect("read first line");
    assert!(n > 0, "expected at least one response line");
    let v: Value = serde_json::from_str(first.trim_end()).expect("first line is one JSON value");
    assert_eq!(v["id"], 7);
    assert_eq!(v["result"]["status"], "ok");

    let status = child.wait().expect("wait for child");
    assert!(status.success(), "must exit 0 on EOF; status={status:?}");
}
