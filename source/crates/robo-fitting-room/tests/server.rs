//! Happy-path integration tests for the in-binary HTTP server. Gated behind
//! the `server` feature so default-feature builds don't pull in axum/tokio/
//! reqwest. Run with: `cargo test --features server --test server`.

#![cfg(feature = "server")]

mod common;

use std::net::SocketAddr;

use common::{populate_assets, ARM_JOINTS};
use robo_fitting_room::server::{router, AppState};
use serde_json::Value;

// -----------------------------------------------------------------------------
// Test scaffolding
// -----------------------------------------------------------------------------

/// Stand up the router on a random port and return its base URL plus the
/// tempdir that holds the asset_root, seeded with the synthetic asset tree of
/// `tests/common` (a small arm, a fixture, a floor plan, `fitted-robots/scene.yaml`).
async fn spawn_server() -> (String, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    let asset_root = dir.path().to_path_buf();
    populate_assets(&asset_root);

    let state = AppState::new(asset_root, None);
    let app = router(state);

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });
    (format!("http://{}", addr), dir)
}

fn rq() -> reqwest::Client {
    reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(15))
        .build()
        .unwrap()
}

// -----------------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------------

#[tokio::test]
async fn list_robots_returns_seeded_entry() {
    let (base, _dir) = spawn_server().await;
    let resp = rq().get(format!("{}/api/v1/assets/robots", base)).send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let v: Value = resp.json().await.unwrap();
    let arr = v.as_array().unwrap();
    assert_eq!(arr.len(), 1);
    assert_eq!(arr[0]["id"], "r1");
    assert_eq!(arr[0]["urdf_rel"], "urdf-dressed-robots/r1/r1.urdf");
}

#[tokio::test]
async fn list_fixtures_returns_seeded_entry() {
    let (base, _dir) = spawn_server().await;
    let v: Value = rq()
        .get(format!("{}/api/v1/assets/fixtures", base))
        .send().await.unwrap()
        .json().await.unwrap();
    let arr = v.as_array().unwrap();
    assert_eq!(arr.len(), 1);
    assert_eq!(arr[0]["id"], "f1");
}

#[tokio::test]
async fn list_floor_plans_returns_obj() {
    let (base, _dir) = spawn_server().await;
    let v: Value = rq()
        .get(format!("{}/api/v1/assets/floor-plans", base))
        .send().await.unwrap()
        .json().await.unwrap();
    let arr = v.as_array().unwrap();
    assert!(arr.iter().any(|e| e["name"] == "empty.obj"));
}

#[tokio::test]
async fn urdf_joints_returns_actuated_count() {
    let (base, _dir) = spawn_server().await;
    let v: Value = rq()
        .post(format!("{}/api/v1/urdf/joints", base))
        .json(&serde_json::json!({ "path": "urdf-dressed-robots/r1/r1.urdf" }))
        .send().await.unwrap()
        .json().await.unwrap();
    let joints = v["joints"].as_array().unwrap();
    // The arm's revolute joints, in declaration order; the world_to_base fixed joint must be
    // filtered.
    let names: Vec<&str> = joints.iter().map(|j| j["name"].as_str().unwrap()).collect();
    assert_eq!(names, ARM_JOINTS);
    assert!(joints.iter().all(|j| j["kind"] == "revolute"), "{joints:?}");
}

#[tokio::test]
async fn floor_plan_segments_returns_geometry() {
    let (base, _dir) = spawn_server().await;
    let v: Value = rq()
        .post(format!("{}/api/v1/floor-plan", base))
        .json(&serde_json::json!({ "path": "floor-plans/empty.obj" }))
        .send().await.unwrap()
        .json().await.unwrap();
    let segs = v["segments"].as_array().unwrap();
    assert_eq!(segs.len(), 1);
}

/// A plan with a `# Layer:` layer and a `g` group, one polyline each.
const LAYERED_PLAN: &str = "v 0 0 0\nv 1 0 0\nv 1 1 0\n# Layer: Zone-B\nl 1 2 3\ng walls\nl 3 1\n";

#[tokio::test]
async fn floor_plan_segments_split_the_listed_footprint_layers() {
    let (base, dir) = spawn_server().await;
    std::fs::write(dir.path().join("floor-plans/layered.obj"), LAYERED_PLAN).unwrap();
    let counts = |v: &Value| {
        (
            v["segments"].as_array().unwrap().len(),
            v["footprint_segments"].as_array().unwrap().len(),
        )
    };
    // Without layers: every line is a line (the polyline makes two segments).
    let v: Value = rq()
        .post(format!("{}/api/v1/floor-plan", base))
        .json(&serde_json::json!({ "path": "floor-plans/layered.obj" }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(counts(&v), (3, 0), "{v}");
    // The manifest's footprint_layers, as compose splits them (any case).
    let v: Value = rq()
        .post(format!("{}/api/v1/floor-plan", base))
        .json(&serde_json::json!({ "path": "floor-plans/layered.obj", "footprint_layers": ["zone-b"] }))
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(counts(&v), (1, 2), "{v}");
}

#[tokio::test]
async fn floor_plan_refuses_a_dxf_file() {
    let (base, dir) = spawn_server().await;
    std::fs::write(dir.path().join("floor-plans/cell.dxf"), "0\nSECTION\n").unwrap();
    let resp = rq()
        .post(format!("{}/api/v1/floor-plan", base))
        .json(&serde_json::json!({ "path": "floor-plans/cell.dxf" }))
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 400);
    let v: Value = resp.json().await.unwrap();
    assert_eq!(v["code"], "floor_plan_unsupported", "{v}");
    assert!(v["message"].as_str().unwrap().contains("BL-22"), "{v}");
}

#[tokio::test]
async fn urdf_joints_give_prismatic_limits_in_metres() {
    let (base, dir) = spawn_server().await;
    let robot_dir = dir.path().join("urdf-dressed-robots/s1");
    std::fs::create_dir_all(&robot_dir).unwrap();
    std::fs::write(
        robot_dir.join("s1.urdf"),
        "<robot name=\"s\"><link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>\
         <joint name=\"track\" type=\"prismatic\"><parent link=\"a\"/><child link=\"b\"/><axis xyz=\"1 0 0\"/><limit lower=\"0.5\" upper=\"2\" effort=\"1\" velocity=\"1\"/></joint>\
         <joint name=\"turn\" type=\"revolute\"><parent link=\"b\"/><child link=\"c\"/><axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>\
         </robot>",
    )
    .unwrap();
    let v: Value = rq()
        .post(format!("{}/api/v1/urdf/joints", base))
        .json(&serde_json::json!({ "path": "urdf-dressed-robots/s1/s1.urdf" }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let joints = v["joints"].as_array().unwrap();
    // The slide's limits in metres, not as if they were radians; the turn's in degrees.
    assert_eq!(joints[0]["kind"], "prismatic");
    assert_eq!(
        (joints[0]["lower_m"].as_f64(), joints[0]["upper_m"].as_f64()),
        (Some(0.5), Some(2.0))
    );
    assert!(
        joints[0]["lower_deg"].is_null() && joints[0]["upper_deg"].is_null(),
        "{v}"
    );
    assert!(
        (joints[1]["upper_deg"].as_f64().unwrap() - 57.29578).abs() < 1e-4,
        "{v}"
    );
    assert!(joints[1].get("lower_m").is_none(), "{v}");
}

#[tokio::test]
async fn validate_reports_warnings_without_failing() {
    let (base, dir) = spawn_server().await;
    let robot_dir = dir.path().join("urdf-dressed-robots/v1");
    std::fs::create_dir_all(&robot_dir).unwrap();
    std::fs::write(
        robot_dir.join("v1.urdf"),
        "<robot name=\"v\"><link name=\"a\"><visual><geometry><mesh filename=\"./meshes/a.stl\"/></geometry></visual></link></robot>",
    )
    .unwrap();
    let yaml = "name: \"t\"\nrobots:\n  - robot_id: \"V1\"\n    dressed_urdf: \"../urdf-dressed-robots/v1/v1.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n";
    let v: Value = rq()
        .post(format!("{}/api/v1/validate", base))
        .header("Content-Type", "application/x-yaml")
        .body(yaml)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(v["ok"], true, "{v}");
    let warnings = v["warnings"].as_array().unwrap();
    assert_eq!(warnings.len(), 1, "{v}");
    assert!(
        warnings[0].as_str().unwrap().contains("aren't OBJ files"),
        "{v}"
    );
}

#[tokio::test]
async fn validate_round_trips_yaml() {
    let (base, _dir) = spawn_server().await;
    // Craft a valid manifest in-memory referencing the URDF that's on disk.
    // The validate handler resolves manifest paths against
    // `<asset_root>/fitted-robots/`, so reach back via `..` to hit the URDF.
    let yaml = "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"../urdf-dressed-robots/r1/r1.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n";
    let resp = rq()
        .post(format!("{}/api/v1/validate", base))
        .header("Content-Type", "application/x-yaml")
        .body(yaml)
        .send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let v: Value = resp.json().await.unwrap();
    assert_eq!(v["ok"], true, "{:?}", v);
    // Nothing to warn about: the key is absent.
    assert!(v.get("warnings").is_none(), "{v}");
}

#[tokio::test]
async fn validate_rejects_bad_home_pose_length() {
    let (base, _dir) = spawn_server().await;
    // Manifest paths resolve relative to <asset_root>/fitted-robots/. Three angles for the
    // arm's two actuated joints.
    let yaml = "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"../urdf-dressed-robots/r1/r1.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n    home_pose_deg: [0.0, 0.0, 0.0]\n";
    let v: Value = rq()
        .post(format!("{}/api/v1/validate", base))
        .header("Content-Type", "application/x-yaml")
        .body(yaml)
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(v["ok"], false);
    let errs = v["errors"].as_array().unwrap();
    assert!(errs.iter().any(|e| e.as_str().unwrap_or("").contains("home_pose_deg")));
}

#[tokio::test]
async fn save_then_load_round_trips() {
    let (base, _dir) = spawn_server().await;
    let body = serde_json::json!({
        "path": "fitted-robots/edited.yaml",
        "yaml": "name: \"edited\"\n",
    });
    let resp = rq()
        .post(format!("{}/api/v1/scenes/save", base))
        .json(&body)
        .send().await.unwrap();
    let status = resp.status();
    let text = resp.text().await.unwrap();
    assert!(status.is_success(), "save failed: {} body={}", status, text);
    let v: Value = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("save body not json: {} (body={})", e, text));
    assert_eq!(v["ok"], true);

    let loaded = rq()
        .post(format!("{}/api/v1/scenes/load", base))
        .json(&serde_json::json!({ "path": "fitted-robots/edited.yaml" }))
        .send().await.unwrap()
        .text().await.unwrap();
    assert!(loaded.contains("edited"));
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
fixtures:
  - id: "F1"
    urdf: "../urdf-fixture-collection/f1/f1.urdf"
    position: { x: 3.0, y: 0.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }
    clamps: [ { name: c1, closed: false } ]
floor_plan:
  geometry_file: "../floor-plans/empty.obj"
  scale: 1.0
  footprint_layers: [ FOOTPRINTS ]
  line_width_m: 0.02
"#;

/// Every server path that writes or rewrites a manifest keeps the keys it
/// doesn't work with: save writes the YAML verbatim, load returns it
/// byte for byte, validate accepts it, and the URDF bundle's copy changes
/// only the asset paths.
#[tokio::test]
async fn save_load_and_bundle_keep_every_manifest_key() {
    let (base, _dir) = spawn_server().await;
    let path = "fitted-robots/keys.yaml";
    let saved: Value = rq()
        .post(format!("{}/api/v1/scenes/save", base))
        .json(&serde_json::json!({ "path": path, "yaml": MANIFEST_WITH_OTHER_KEYS }))
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(saved["bytes"], MANIFEST_WITH_OTHER_KEYS.len() as u64, "{saved}");
    let loaded = rq()
        .post(format!("{}/api/v1/scenes/load", base))
        .json(&serde_json::json!({ "path": path }))
        .send().await.unwrap()
        .text().await.unwrap();
    assert_eq!(loaded, MANIFEST_WITH_OTHER_KEYS, "load returns the saved bytes");

    let v: Value = rq()
        .post(format!("{}/api/v1/validate", base))
        .header("Content-Type", "application/x-yaml")
        .body(MANIFEST_WITH_OTHER_KEYS)
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(v["ok"], true, "{v}");

    let resp = rq()
        .post(format!("{}/api/v1/compose-urdf-bundle?manifest_path={}", base, path))
        .send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let bytes = resp.bytes().await.unwrap();
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(bytes.to_vec())).unwrap();
    let mut bundled = String::new();
    std::io::Read::read_to_string(&mut z.by_name("keys.yaml").unwrap(), &mut bundled).unwrap();
    let bundled: serde_yaml::Value = serde_yaml::from_str(&bundled).unwrap();
    let mut expected: serde_yaml::Value = serde_yaml::from_str(MANIFEST_WITH_OTHER_KEYS).unwrap();
    for (list, key) in [("robots", "dressed_urdf"), ("fixtures", "urdf")] {
        let rewritten = bundled[list][0][key].as_str().unwrap();
        assert_ne!(rewritten, expected[list][0][key].as_str().unwrap(), "{list}: the bundle's own path");
        expected[list][0][key] = rewritten.into();
    }
    expected["floor_plan"]["geometry_file"] = bundled["floor_plan"]["geometry_file"].clone();
    assert_eq!(bundled, expected, "the bundle's scene keeps every other key");
}

#[tokio::test]
async fn compose_returns_zip() {
    let (base, _dir) = spawn_server().await;
    let resp = rq()
        .post(format!(
            "{}/api/v1/compose?manifest_path=fitted-robots/scene.yaml",
            base
        ))
        .send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let ct = resp
        .headers()
        .get("content-type")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        .to_string();
    assert_eq!(ct, "application/zip", "wrong content-type");
    let bytes = resp.bytes().await.unwrap();
    // PK\x03\x04 zip local-file header
    assert_eq!(&bytes[..4], b"PK\x03\x04", "first 4 bytes should be PK header");
}

#[tokio::test]
async fn compose_urdf_bundle_returns_zip() {
    let (base, _dir) = spawn_server().await;
    let resp = rq()
        .post(format!(
            "{}/api/v1/compose-urdf-bundle?manifest_path=fitted-robots/scene.yaml",
            base
        ))
        .send()
        .await
        .unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let ct = resp
        .headers()
        .get("content-type")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        .to_string();
    assert_eq!(ct, "application/zip", "wrong content-type");
    let bytes = resp.bytes().await.unwrap();
    assert_eq!(&bytes[..4], b"PK\x03\x04", "first 4 bytes should be PK header");

    // Unzip in memory: expect the scene YAML + at least one bundled URDF.
    let reader = std::io::Cursor::new(bytes.to_vec());
    let mut z = zip::ZipArchive::new(reader).unwrap();
    let names: Vec<String> = (0..z.len()).map(|i| z.by_index(i).unwrap().name().unwrap().into_owned()).collect();
    assert!(names.iter().any(|n| n.ends_with(".yaml")), "names: {:?}", names);
    assert!(
        names.iter().any(|n| n.starts_with("robots/") && n.ends_with(".urdf")),
        "expected a bundled robot URDF, names: {:?}",
        names
    );
    // The URDF's whole `meshes/` subtree comes along, the dressed tool's too.
    assert!(
        names
            .iter()
            .any(|n| n.starts_with("robots/") && n.ends_with("meshes/tool/tool.obj")),
        "expected the bundled tool mesh, names: {:?}",
        names
    );
}

#[tokio::test]
async fn upload_obj_round_trips_and_appears_in_listing() {
    let (base, _dir) = spawn_server().await;
    let form = reqwest::multipart::Form::new()
        .text("filename", "uploaded.obj")
        .part(
            "file",
            reqwest::multipart::Part::bytes(
                b"v 0 0 0\nv 1 0 0\nl 1 2\n".to_vec(),
            )
            .file_name("uploaded.obj"),
        );
    let v: Value = rq()
        .post(format!("{}/api/v1/assets/floor-plans/upload-obj", base))
        .multipart(form)
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(v["stored_path"], "floor-plans/uploaded.obj");

    let list: Value = rq()
        .get(format!("{}/api/v1/assets/floor-plans", base))
        .send().await.unwrap()
        .json().await.unwrap();
    let arr = list.as_array().unwrap();
    assert!(arr.iter().any(|e| e["name"] == "uploaded.obj"));
}

#[tokio::test]
async fn path_traversal_is_blocked() {
    let (base, _dir) = spawn_server().await;
    let resp = rq()
        .post(format!("{}/api/v1/urdf/joints", base))
        .json(&serde_json::json!({ "path": "../etc/passwd" }))
        .send().await.unwrap();
    // Either 400 (rejected by resolver) or 404; never 200.
    assert!(!resp.status().is_success(), "{}", resp.status());
}

/// FR-205: OpenAPI spec is served at /openapi.json (and Swagger UI at /docs).
#[tokio::test]
async fn openapi_spec_is_served() {
    let (base, _dir) = spawn_server().await;
    let resp = rq().get(format!("{}/openapi.json", base)).send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let v: Value = resp.json().await.unwrap();
    // Minimal sanity: it really is an OpenAPI 3.x doc.
    assert!(v.get("openapi").is_some(), "missing top-level `openapi`");
    assert!(v.get("paths").is_some(), "missing `paths`");
    // Every documented path is under /api/v1 (or the root probes).
    let paths = v["paths"].as_object().unwrap();
    for k in paths.keys() {
        assert!(
            k.starts_with("/api/v1/") || k == "/healthz" || k == "/readyz",
            "spec path {} not under /api/v1 contract",
            k,
        );
    }
}

/// Liveness + readiness probes at the root (k8s convention).
#[tokio::test]
async fn healthz_and_readyz_are_served() {
    let (base, _dir) = spawn_server().await;
    let h: Value = rq()
        .get(format!("{}/healthz", base))
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(h["status"], "ok");
    assert!(h["version"].is_string());

    let r: Value = rq()
        .get(format!("{}/readyz", base))
        .send().await.unwrap()
        .json().await.unwrap();
    assert_eq!(r["ready"], true, "readyz should be true once asset_root is seeded: {:?}", r);
    assert!(r["urdf_count"].as_u64().unwrap() >= 1);
}

/// Legacy `/api/...` aliases continue to work but carry `Deprecation: true`.
#[tokio::test]
async fn legacy_api_alias_is_marked_deprecated() {
    let (base, _dir) = spawn_server().await;
    let resp = rq()
        .get(format!("{}/api/assets/robots", base))
        .send().await.unwrap();
    assert!(resp.status().is_success(), "{}", resp.status());
    let dep = resp
        .headers()
        .get("deprecation")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    assert_eq!(dep, "true", "legacy /api alias must carry `Deprecation: true`");
}

/// Error envelope is a typed body, not a bare string.
#[tokio::test]
async fn error_envelope_is_typed() {
    let (base, _dir) = spawn_server().await;
    let resp = rq()
        .post(format!("{}/api/v1/urdf/joints", base))
        .json(&serde_json::json!({ "path": "does/not/exist.urdf" }))
        .send().await.unwrap();
    assert!(!resp.status().is_success());
    let v: Value = resp.json().await.unwrap();
    assert!(v["code"].is_string(), "envelope must carry `code`: {:?}", v);
    assert!(v["message"].is_string(), "envelope must carry `message`: {:?}", v);
}
