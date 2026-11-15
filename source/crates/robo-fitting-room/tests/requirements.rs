//! Integration tests mapped to REQUIREMENTS.md functional requirements.
//! Each test function is named after its requirement ID for direct traceability.
//! Run with: cargo test --test requirements
//!
//! Conventions:
//! - Tests for FRs marked DONE / PARTIAL exercise the implementation end-to-end
//!   via the public API (parse_scene, validate_scene, compose_stage_with_options).
//! - Tests for FRs marked NOT STARTED / BACKLOG remain `todo!()` so the
//!   requirements list stays auditable: green = implemented, red = pending.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use robo_fitting_room::{
    compose_stage, compose_stage_with_options, compose_urdf_bundle, parse_scene, scene_warnings,
    validate_scene, ComposeOptions, FixturePlacement, Pedestal, RobotPlacement, SceneDefinition,
};

// =============================================================================
// Test helpers
// =============================================================================

/// Write a tiny valid OBJ triangle so URDF mesh references resolve at compose time.
fn write_obj_triangle(path: &Path) {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    let mut f = std::fs::File::create(path).unwrap();
    writeln!(f, "v 0 0 0").unwrap();
    writeln!(f, "v 1 0 0").unwrap();
    writeln!(f, "v 0 1 0").unwrap();
    writeln!(f, "f 1 2 3").unwrap();
}

/// The links [`write_six_axis_urdf`] gives URDF inertials, after its `World`: name, mass (kg) and
/// diagonal inertia (kg·m²), at the link origin. Synthetic values.
const SIX_AXIS_LINKS: [(&str, f64, [f64; 3]); 7] = [
    ("arm_base_link", 400.0, [20.0, 20.0, 10.0]),
    ("arm_link_1", 300.0, [12.0, 12.0, 6.0]),
    ("arm_link_2", 200.0, [10.0, 10.0, 2.0]),
    ("arm_link_3", 100.0, [5.0, 5.0, 1.0]),
    ("arm_link_4", 40.0, [2.0, 2.0, 0.8]),
    ("arm_link_5", 20.0, [0.6, 0.6, 0.3]),
    ("arm_link_6", 10.0, [0.2, 0.2, 0.1]),
];

/// Write a minimal 6-axis revolute robot URDF: `World` (no inertial) and the
/// [`SIX_AXIS_LINKS`] with their inertials, each link with a triangle visual and collision
/// (which encloses no volume, so the inertials are used as authored, FR-113). All meshes live
/// under `meshes/robot/`.
fn write_six_axis_urdf(urdf_path: &Path) {
    let dir = urdf_path.parent().unwrap();
    write_obj_triangle(&dir.join("meshes/robot/m.obj"));

    let mut s = String::from("<?xml version=\"1.0\"?>\n<robot name=\"six_axis\">\n");
    let mesh = "    <visual>\n      <geometry>\n        <mesh filename=\"./meshes/robot/m.obj\"/>\n      </geometry>\n    </visual>\n    <collision>\n      <geometry>\n        <mesh filename=\"./meshes/robot/m.obj\"/>\n      </geometry>\n    </collision>\n";
    s.push_str(&format!("  <link name=\"World\">\n{mesh}  </link>\n"));
    for (name, mass, [ixx, iyy, izz]) in SIX_AXIS_LINKS {
        s.push_str(&format!(
            "  <link name=\"{name}\">\n    <inertial>\n      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n      <mass value=\"{mass}\"/>\n      <inertia ixx=\"{ixx}\" ixy=\"0\" ixz=\"0\" iyy=\"{iyy}\" iyz=\"0\" izz=\"{izz}\"/>\n    </inertial>\n{mesh}  </link>\n"
        ));
    }
    // World → base via fixed; base → link_1..6 revolute.
    s.push_str(
        "  <joint name=\"world_to_base\" type=\"fixed\">\n    <parent link=\"World\"/>\n    <child link=\"arm_base_link\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n",
    );
    for i in 1..=6 {
        let parent = if i == 1 {
            "arm_base_link".to_string()
        } else {
            format!("arm_link_{}", i - 1)
        };
        s.push_str(&format!(
            "  <joint name=\"joint_{}\" type=\"revolute\">\n    <parent link=\"{}\"/>\n    <child link=\"arm_link_{}\"/>\n    <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"100\" velocity=\"1\"/>\n  </joint>\n",
            i, parent, i
        ));
    }
    s.push_str("</robot>\n");
    std::fs::File::create(urdf_path)
        .unwrap()
        .write_all(s.as_bytes())
        .unwrap();
}

/// Write a fixture URDF with a single link and one fixed joint.
/// Kept as a helper for future fixture-emission tests; currently unused
/// in the suite, hence the dead_code allow.
#[allow(dead_code)]
fn write_fixture_urdf(urdf_path: &Path) {
    let dir = urdf_path.parent().unwrap();
    write_obj_triangle(&dir.join("meshes/m.obj"));
    let s = "<?xml version=\"1.0\"?>\n<robot name=\"fix\">\n  <link name=\"World\">\n    <visual>\n      <geometry>\n        <mesh filename=\"./meshes/m.obj\"/>\n      </geometry>\n    </visual>\n    <collision>\n      <geometry>\n        <mesh filename=\"./meshes/m.obj\"/>\n      </geometry>\n    </collision>\n  </link>\n  <link name=\"plate\"/>\n  <joint name=\"j_fix\" type=\"fixed\">\n    <parent link=\"World\"/>\n    <child link=\"plate\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n</robot>\n";
    std::fs::File::create(urdf_path)
        .unwrap()
        .write_all(s.as_bytes())
        .unwrap();
}

/// Read a single entry from a zip archive into a String (panics on missing/IO).
fn read_zip_entry(zip_path: &Path, entry: &str) -> String {
    let f = std::fs::File::open(zip_path).unwrap();
    let mut z = zip::ZipArchive::new(f).unwrap();
    let mut e = z.by_name(entry).unwrap();
    let mut s = String::new();
    e.read_to_string(&mut s).unwrap();
    s
}

/// List every entry name in a zip archive.
fn zip_entry_names(zip_path: &Path) -> Vec<String> {
    let f = std::fs::File::open(zip_path).unwrap();
    let mut z = zip::ZipArchive::new(f).unwrap();
    (0..z.len())
        .map(|i| z.by_index(i).unwrap().name().to_string())
        .collect()
}

/// Compose a one-robot scene from a six_axis URDF and return (zip_path, usda_text).
fn compose_one_six_axis_robot(opts: ComposeOptions) -> (PathBuf, String, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();

    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [1.0, 2.0, 3.0],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");
    (out, usda, dir)
}

// =============================================================================
// 2.1 Scene Manifest & Composition
// =============================================================================

/// FR-100: Parse scene manifest YAML declaring URDF assets and world-space placements
#[test]
fn fr_100_parse_scene_manifest() {
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    let mut f = std::fs::File::create(&scene_path).unwrap();
    write!(
        f,
        r#"name: "Test Cell"
description: "Integration test scene"
robots:
  - robot_id: "R1"
    dressed_urdf: "robot.urdf"
    position: {{ x: 1.0, y: 2.0, z: 0.0 }}
    orientation: {{ a: 90.0, b: 0.0, c: 0.0 }}
fixtures:
  - id: "F1"
    urdf: "fixture.urdf"
    position: {{ x: 3.0, y: 4.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
"#
    )
    .unwrap();

    let scene = parse_scene(&scene_path).unwrap();
    assert_eq!(scene.name, "Test Cell");
    assert_eq!(scene.robots.len(), 1);
    assert_eq!(scene.robots[0].robot_id, "R1");
    assert!((scene.robots[0].position[0] - 1.0).abs() < 1e-6);
    assert!((scene.robots[0].orientation_deg[0] - 90.0).abs() < 1e-6);
    assert_eq!(scene.fixtures.len(), 1);
    assert_eq!(scene.fixtures[0].id, "F1");
}

/// FR-101: Place multiple dressed robots in a single scene
#[test]
fn fr_101_multiple_robots() {
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    let mut f = std::fs::File::create(&scene_path).unwrap();
    write!(
        f,
        r#"name: "Multi-robot"
robots:
  - robot_id: "R1"
    dressed_urdf: "r1.urdf"
    position: {{ x: 0.0, y: 0.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
  - robot_id: "R2"
    dressed_urdf: "r2.urdf"
    position: {{ x: 5.0, y: 0.0, z: 0.0 }}
    orientation: {{ a: 180.0, b: 0.0, c: 0.0 }}
  - robot_id: "R3"
    dressed_urdf: "r3.urdf"
    position: {{ x: 10.0, y: 0.0, z: 0.0 }}
    orientation: {{ a: 270.0, b: 0.0, c: 0.0 }}
  - robot_id: "R4"
    dressed_urdf: "r4.urdf"
    position: {{ x: 15.0, y: 0.0, z: 0.0 }}
    orientation: {{ a: 45.0, b: 0.0, c: 0.0 }}
  - robot_id: "R5"
    dressed_urdf: "r5.urdf"
    position: {{ x: 20.0, y: 0.0, z: 0.0 }}
    orientation: {{ a: 315.0, b: 0.0, c: 0.0 }}
"#
    )
    .unwrap();

    let scene = parse_scene(&scene_path).unwrap();
    assert_eq!(scene.robots.len(), 5);
    assert_eq!(scene.robots[0].robot_id, "R1");
    assert_eq!(scene.robots[4].robot_id, "R5");
    assert!((scene.robots[4].position[0] - 20.0).abs() < 1e-6);
}

/// FR-102: Place static fixture URDFs as static colliders: their links with their visual and
/// collision meshes only, no rigid body, mass, contact reporting, sensor, anchor or joints
#[test]
fn fr_102_static_fixtures() {
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    let mut f = std::fs::File::create(&scene_path).unwrap();
    write!(
        f,
        r#"name: "Fixtures"
fixtures:
  - id: "conveyor_01"
    urdf: "conveyor.urdf"
    position: {{ x: 10.0, y: 5.0, z: 0.0 }}
    orientation: {{ a: 90.0, b: 0.0, c: 0.0 }}
  - id: "fence_north"
    urdf: "fence.urdf"
    position: {{ x: 0.0, y: 20.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
"#
    )
    .unwrap();

    let scene = parse_scene(&scene_path).unwrap();
    assert_eq!(scene.fixtures.len(), 2);
    assert_eq!(scene.fixtures[0].id, "conveyor_01");
    assert_eq!(scene.fixtures[1].id, "fence_north");

    // Composed: static colliders at the placement.
    let urdf = dir.path().join("fx/f.urdf");
    write_fixture_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let mut scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![],
        fixtures: vec![FixturePlacement {
            id: "F1".into(),
            urdf: urdf.clone(),
            position: [10.0, 5.0, 0.0],
            orientation_deg: [90.0, 0.0, 0.0],
        }],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");
    let start = usda.find("    def Scope \"fixtures\" (").unwrap();
    let fixtures = &usda[start..start + usda[start..].find("\n    }\n").unwrap()];
    assert!(fixtures.contains("            double3 xformOp:translate = (10, 5, 0)\n"));
    // The links are plain Xforms (no applied schemas); the colliders are PhysicsCollisionAPI.
    assert!(fixtures.contains("            def Xform \"world_link\"\n            {\n"));
    assert!(fixtures.contains("            def Xform \"plate\"\n            {\n"));
    assert!(fixtures.contains("def Mesh \"visual_0\""));
    assert!(fixtures.contains("prepend apiSchemas = [\"PhysicsCollisionAPI\", \"PhysicsMeshCollisionAPI\"]"));
    for gone in [
        "PhysicsRigidBodyAPI",
        "PhysicsMassAPI",
        "physics:mass",
        "PhysxContactReportAPI",
        "ContactSensor",
        "world_anchor",
        "Joint",
    ] {
        assert!(!fixtures.contains(gone), "{gone}");
    }
    assert!(!usda.contains("/World/fixtures/"), "nothing may refer to a fixture link");

    // A fixture with a non-fixed joint is rejected at compose time.
    let bad = dir.path().join("bad/arm.urdf");
    write_six_axis_urdf(&bad);
    scene.fixtures[0].urdf = bad;
    let err = compose_stage(&scene, &manifest, &out).unwrap_err();
    assert!(err.to_string().contains("non-fixed joint"), "{err}");
}

/// FR-103: Validate manifest completeness (paths exist, positions finite, URDF valid)
#[test]
fn fr_103_manifest_validation() {
    // Missing URDF file
    let scene_missing = SceneDefinition {
        name: "test".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: PathBuf::from("/nonexistent/robot.urdf"),
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errors = validate_scene(&scene_missing);
    assert!(errors.iter().any(|e| e.to_string().contains("not found")));

    // Non-finite position
    let scene_nan = SceneDefinition {
        name: "test".into(),
        description: String::new(),
        robots: vec![],
        fixtures: vec![FixturePlacement {
            id: "F1".into(),
            urdf: PathBuf::from("/nonexistent/f.urdf"),
            position: [f64::NAN, 0.0, 0.0],
            orientation_deg: [0.0; 3],
        }],
        floor_plan: None,
    };
    let errors = validate_scene(&scene_nan);
    assert!(errors.iter().any(|e| e.to_string().contains("non-finite")));

    // Invalid URDF content
    let dir = tempfile::tempdir().unwrap();
    let bad_urdf = dir.path().join("bad.urdf");
    std::fs::write(&bad_urdf, "not a urdf file").unwrap();
    let scene_invalid = SceneDefinition {
        name: "test".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: bad_urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errors = validate_scene(&scene_invalid);
    assert!(errors.iter().any(|e| e.to_string().contains("robot")));

    // Valid URDF passes
    let good_urdf = dir.path().join("good.urdf");
    std::fs::write(
        &good_urdf,
        r#"<?xml version="1.0"?><robot name="test"><link name="base"/></robot>"#,
    )
    .unwrap();
    let scene_ok = SceneDefinition {
        name: "test".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: good_urdf,
            position: [1.0, 2.0, 3.0],
            orientation_deg: [90.0, 0.0, 0.0],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errors = validate_scene(&scene_ok);
    assert!(errors.is_empty());
    assert!(scene_warnings(&scene_ok).is_empty());

    // What composes but not as meant is a warning, not an error: a mesh that isn't OBJ (it would
    // compose as an empty prim), a mesh given as a URI.
    let warned_urdf = dir.path().join("warned.urdf");
    std::fs::write(
        &warned_urdf,
        r#"<?xml version="1.0"?><robot name="w"><link name="base"><visual><geometry><mesh filename="./meshes/base.stl"/></geometry></visual><collision><geometry><mesh filename="package://w/meshes/base.obj"/></geometry></collision></link></robot>"#,
    )
    .unwrap();
    let mut scene_warned = scene_ok.clone();
    scene_warned.robots[0].dressed_urdf = warned_urdf;
    assert!(validate_scene(&scene_warned).is_empty());
    let warnings = scene_warnings(&scene_warned);
    assert_eq!(warnings.len(), 2, "{warnings:#?}");
    assert!(
        warnings[0].contains("given as a URI") && warnings[1].contains("aren't OBJ files"),
        "{warnings:#?}"
    );
}

/// FR-104: Scene inheritance (base + overlay manifests) — NOT STARTED
#[test]
fn fr_104_scene_inheritance() {
    todo!("Scene inheritance not yet implemented");
}

/// FR-105: Load 2D CAD floor plan (OBJ line geometry) with offset/scale and layer separation
#[test]
fn fr_105_floor_plan_loading() {
    let dir = tempfile::tempdir().unwrap();

    // Create OBJ with floor segments, a group the manifest lists as a footprint layer and a
    // layer named like footprints
    let obj_path = dir.path().join("plan.obj");
    std::fs::write(
        &obj_path,
        "v 0.0 0.0 0.0\nv 10.0 0.0 0.0\nl 1 2\no ZONE_7\nv 2.0 2.0 0.0\nv 4.0 4.0 0.0\nl 3 4\n# Layer: Robot Footprints\nl 1 3\n",
    )
    .unwrap();

    let scene_path = dir.path().join("cell.yaml");
    let manifest = |footprint_layers: &str| {
        let mut f = std::fs::File::create(&scene_path).unwrap();
        write!(
            f,
            "name: \"Floor plan test\"\nfloor_plan:\n  geometry_file: \"plan.obj\"\n  offset: {{ x: 1.0, y: 2.0, z: 0.0 }}\n  scale: 2.0\n{footprint_layers}"
        )
        .unwrap();
        parse_scene(&scene_path).unwrap().floor_plan.unwrap()
    };

    // Without `footprint_layers` only the layer named like footprints holds footprints.
    let fp = manifest("");
    assert_eq!((fp.segments.len(), fp.footprint_segments.len()), (2, 1));
    let fp = manifest("  footprint_layers: [zone_7]\n");

    // Floor segment: (0*2+1, 0*2+2, 0) to (10*2+1, 0*2+2, 0)
    assert_eq!(fp.segments.len(), 1);
    assert!((fp.segments[0][0][0] - 1.0).abs() < 1e-6);
    assert!((fp.segments[0][1][0] - 21.0).abs() < 1e-6);

    // Footprint segments: the listed group (case-insensitively) and the footprint layer
    assert_eq!(fp.footprint_segments.len(), 2);
    assert!((fp.footprint_segments[0][0][0] - 5.0).abs() < 1e-6); // 2*2+1

    // Other exporters: `l` polylines (a segment per vertex pair), `g` groups as layers, `v/vt`
    // index tokens and negative indices.
    std::fs::write(
        &obj_path,
        "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\ng outline\nl 1/1 2/2 3/3 4/4 -4\ng ZONE_7\nl -1 -3\n",
    )
    .unwrap();
    let fp = manifest("  footprint_layers: [zone_7]\n");
    assert_eq!((fp.segments.len(), fp.footprint_segments.len()), (4, 1));
    assert_eq!(fp.layers, ["outline", "ZONE_7"]);
}

/// FR-106: Optional `scene_origin` field is subtracted from all placements + floor-plan offset
#[test]
fn fr_106_scene_origin_centering() {
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    std::fs::write(
        &scene_path,
        "name: \"o\"\nscene_origin: { x: 100.0, y: 50.0, z: 0.0 }\nrobots:\n  - robot_id: \"R\"\n    dressed_urdf: \"r.urdf\"\n    position: { x: 110.0, y: 60.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\nfixtures:\n  - id: \"F\"\n    urdf: \"f.urdf\"\n    position: { x: 105.0, y: 55.0, z: 1.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n",
    )
    .unwrap();
    let scene = parse_scene(&scene_path).unwrap();
    // scene_origin subtracted from every position
    assert!((scene.robots[0].position[0] - 10.0).abs() < 1e-6);
    assert!((scene.robots[0].position[1] - 10.0).abs() < 1e-6);
    assert!((scene.fixtures[0].position[0] - 5.0).abs() < 1e-6);
    assert!((scene.fixtures[0].position[1] - 5.0).abs() < 1e-6);
    assert!((scene.fixtures[0].position[2] - 1.0).abs() < 1e-6);
}

/// FR-107: the floor plan is drawn on the ground: /World/floor_plan, 2 mm up, its lines and
/// footprints as flat meshes of 2.5 cm strips (two triangles each, facing up) in one matte slate
/// material, rendered (no guide purpose) and without a collider.
#[test]
fn fr_107_floor_plan_emission() {
    let dir = tempfile::tempdir().unwrap();
    let obj_path = dir.path().join("plan.obj");
    std::fs::write(
        &obj_path,
        "v 0 0 0\nv 2 0 0\nv 2 3 0\nl 1 2\nl 2 3\n# Layer: footprints\nv 5 5 0\nv 5 6 0\nl 4 5\n",
    )
    .unwrap();
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(
        &manifest,
        "name: \"fp\"\nfloor_plan:\n  geometry_file: \"plan.obj\"\n",
    )
    .unwrap();
    let scene = parse_scene(&manifest).unwrap();
    assert!(scene.floor_plan.is_some());
    let out = dir.path().join("scene.zip");
    let report = compose_stage(&scene, &manifest, &out).unwrap();
    assert_eq!(report.floor_plan_segments, 3);
    let usda = read_zip_entry(&out, "scene.usda");

    let start = usda.find("\n    def Xform \"floor_plan\"\n").expect("floor_plan prim") + 1;
    let floor_plan = &usda[start..start + usda[start..].find("\n    }\n").unwrap()];
    assert!(usda.find("def Xform \"World\"").unwrap() < start);
    assert!(floor_plan.contains("        double3 xformOp:translate = (0, 0, 0.002)\n"));
    // One matte slate material on both layers.
    assert!(floor_plan.contains("            def Material \"M_floorplan\"\n"));
    assert!(floor_plan.contains("color3f inputs:diffuseColor = (0.046665, 0.061246, 0.0865)\n"));
    assert!(floor_plan.contains("float inputs:roughness = 0.9\n"));
    assert_eq!(
        floor_plan
            .matches("rel material:binding = </World/floor_plan/Looks/M_floorplan>\n")
            .count(),
        2
    );
    for (layer, segments) in [("lines", 2), ("footprints", 1)] {
        let mesh = &floor_plan[floor_plan.find(&format!("def Mesh \"{layer}\" (")).unwrap()..];
        let mesh = &mesh[..mesh.find("\n        }").unwrap()];
        assert!(mesh.contains("uniform bool doubleSided = 1\n"));
        assert!(mesh.contains("float3[] extent = ["));
        let line = |key: &str| mesh.lines().find(|l| l.contains(key)).unwrap().to_owned();
        // Two triangles per segment, four corners each facing up.
        assert_eq!(line("int[] faceVertexCounts").matches('3').count(), 2 * segments);
        assert_eq!(line("normal3f[] normals").matches("(0, 0, 1)").count(), 4 * segments);
        assert!(mesh.contains("interpolation = \"vertex\""));
        let points: Vec<Vec<f64>> = line("point3f[] points")
            .split(" = ")
            .nth(1)
            .unwrap()
            .trim_matches(|c| c == '[' || c == ']')
            .split("), (")
            .map(|p| {
                p.trim_matches(|c| c == '(' || c == ')')
                    .split(", ")
                    .map(|v| v.parse().unwrap())
                    .collect()
            })
            .collect();
        assert_eq!(points.len(), 4 * segments);
        // 2.5 cm wide: the first strip's corners 0 and 1 straddle its start point.
        let width = ((points[0][0] - points[1][0]).powi(2) + (points[0][1] - points[1][1]).powi(2)).sqrt();
        assert!((width - 0.025).abs() < 1e-9, "{layer}: {width}");
    }
    // Drawn, not a guide (Kit hides guides); a drawing, not a collider.
    for gone in ["purpose", "Collision", "physics:"] {
        assert!(!floor_plan.contains(gone), "{gone}");
    }
}

/// One emitted `def Camera` prim.
#[derive(Debug, Default)]
struct EmittedCamera {
    name: String,
    translate: [f64; 3],
    /// (w, x, y, z)
    orient: [f64; 4],
    clipping_range: [f64; 2],
    /// Focal length, horizontal and vertical aperture.
    lens: [f64; 3],
    camera_lock: bool,
    /// The applied schemas' line, e.g. `prepend apiSchemas = [...]`.
    schemas: String,
    /// exposure:fStop, exposure:responsivity, exposure:time.
    exposure: [f64; 3],
}

/// The numbers in the parenthesised tuple of an attribute line.
fn tuple_values(line: &str) -> Vec<f64> {
    let inner = &line[line.find('(').unwrap() + 1..line.rfind(')').unwrap()];
    inner
        .split(',')
        .map(|v| v.trim().parse().unwrap())
        .collect()
}

/// The value right of the `=` of an attribute line.
fn scalar_value(line: &str) -> f64 {
    line.split('=').nth(1).unwrap().trim().parse().unwrap()
}

/// Every `def Camera` prim in a composed USDA, in order.
fn emitted_cameras(usda: &str) -> Vec<EmittedCamera> {
    let mut cameras: Vec<EmittedCamera> = Vec::new();
    let mut inside = false;
    for line in usda.lines() {
        let l = line.trim();
        if let Some(name) = l.strip_prefix("def Camera \"") {
            cameras.push(EmittedCamera {
                name: name[..name.find('"').unwrap()].to_string(),
                ..EmittedCamera::default()
            });
            inside = true;
            continue;
        }
        let Some(camera) = cameras.last_mut().filter(|_| inside) else {
            continue;
        };
        if l.starts_with("prepend apiSchemas = ") {
            camera.schemas = l.to_string();
        } else if l.starts_with("float exposure:fStop = ") {
            camera.exposure[0] = scalar_value(l);
        } else if l.starts_with("float exposure:responsivity = ") {
            camera.exposure[1] = scalar_value(l);
        } else if l.starts_with("float exposure:time = ") {
            camera.exposure[2] = scalar_value(l);
        } else if l.starts_with("float2 clippingRange = ") {
            camera.clipping_range.copy_from_slice(&tuple_values(l));
        } else if l.starts_with("float focalLength = ") {
            camera.lens[0] = scalar_value(l);
        } else if l.starts_with("float horizontalAperture = ") {
            camera.lens[1] = scalar_value(l);
        } else if l.starts_with("float verticalAperture = ") {
            camera.lens[2] = scalar_value(l);
        } else if l == "custom bool omni:kit:cameraLock = 1" {
            camera.camera_lock = true;
        } else if l.starts_with("quatd xformOp:orient = ") {
            camera.orient.copy_from_slice(&tuple_values(l));
        } else if l.starts_with("double3 xformOp:translate = ") {
            camera.translate.copy_from_slice(&tuple_values(l));
        } else if l == "}" {
            inside = false;
        }
    }
    cameras
}

/// `p` in the camera space of `camera`: right, up, and the depth along its view direction.
fn camera_space(camera: &EmittedCamera, p: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = camera.orient;
    let axes = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ],
        [
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ],
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ];
    let d: Vec<f64> = (0..3).map(|i| p[i] - camera.translate[i]).collect();
    let along = |axis: &[f64; 3]| axis[0] * d[0] + axis[1] * d[1] + axis[2] * d[2];
    [along(&axes[0]), along(&axes[1]), -along(&axes[2])]
}

/// FR-108: a six-camera rig under /World/Cameras frames the scene's bounds — every corner in
/// every view, one on a frame edge, nothing clipped — and Kit's opening view is seeded onto it.
#[test]
fn fr_108_camera_rig() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());

    // Inside /World, after the robots and graphs, before /Environment.
    let rig = usda
        .find("    def Scope \"Cameras\"\n")
        .expect("camera rig");
    assert!(usda.find("def Xform \"World\"").unwrap() < rig);
    assert!(usda.rfind("def OmniGraph ").unwrap() < rig);
    assert!(rig < usda.find("def Xform \"Environment\"").unwrap());

    let cameras = emitted_cameras(&usda);
    let names: Vec<&str> = cameras.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        names,
        ["ISO_NE", "ISO_NW", "PLAN", "ISO_SE", "ISO_SW", "ELEVATION"]
    );
    // RTX's exposure APIs with Kit's own exposure (scale 1 / 1133.6 at ISO 100).
    for camera in &cameras {
        assert_eq!(
            camera.schemas,
            "prepend apiSchemas = [\"OmniRtxCameraAutoExposureAPI_1\", \"OmniRtxCameraExposureAPI_1\"]",
            "{}",
            camera.name
        );
        assert_eq!(camera.exposure, [5.0, 1.1026709, 0.02], "{}", camera.name);
    }

    // The six_axis arm at (1, 2, 3), stowed (link_2 turned -90°), spans x 1..2, y 1..3, z 3..6
    // with its one-triangle hulls; the rig aims at the centre.
    let (lo, hi) = ([1.0, 1.0, 3.0], [2.0, 3.0, 6.0]);
    let center = [1.5, 2.0, 4.5];
    let corners: Vec<[f64; 3]> = (0..8)
        .map(|i| [0, 1, 2].map(|a| if i & (1 << a) == 0 { lo[a] } else { hi[a] }))
        .collect();
    let distance = camera_space(&cameras[0], center)[2];
    let mut tightest = 0.0f64;
    for camera in &cameras {
        assert_eq!(
            camera.lens,
            [32.992786, 20.955, 11.787187],
            "{}",
            camera.name
        );
        assert!(camera.camera_lock, "{}", camera.name);
        // Every camera looks at the centre from the same distance.
        let [x, y, depth] = camera_space(camera, center);
        assert!(
            x.abs() < 1e-9 && y.abs() < 1e-9,
            "{} misses the centre",
            camera.name
        );
        assert!((depth - distance).abs() < 1e-9, "{}", camera.name);
        let [near, far] = camera.clipping_range;
        assert!((near - 0.20471 * distance).abs() < 1e-6 && far == 10000.0);
        let tan_h = camera.lens[1] / (2.0 * camera.lens[0]);
        let tan_v = camera.lens[2] / (2.0 * camera.lens[0]);
        for &corner in &corners {
            let [x, y, depth] = camera_space(camera, corner);
            assert!(
                near < depth && depth < far,
                "{}: {corner:?} clipped",
                camera.name
            );
            // 1e-6: the apertures are written to 6 decimals.
            let edge = (x.abs() / (tan_h * depth)).max(y.abs() / (tan_v * depth));
            assert!(
                edge < 1.0 + 1e-6,
                "{}: {corner:?} out of frame",
                camera.name
            );
            tightest = tightest.max(edge);
        }
    }
    assert!((tightest - 1.0).abs() < 1e-6, "loose fit: {tightest}");

    // Kit's own camera stays the opening one, seeded to look at the centre from ISO_NE's side.
    assert!(usda.contains("string boundCamera = \"/OmniverseKit_Persp\""));
    assert!(usda.contains("rel camera = </OmniverseKit_Persp>"));
    let perspective = &usda[usda.find("dictionary Perspective = {").unwrap()..];
    let seed = |key: &str| tuple_values(perspective.lines().find(|l| l.contains(key)).unwrap());
    let (position, target) = (seed("double3 position = ("), seed("double3 target = ("));
    assert!(position != [5.0, 5.0, 5.0]);
    let toward = |p: &[f64]| -> Vec<f64> {
        let d: Vec<f64> = (0..3).map(|i| p[i] - center[i]).collect();
        let n = d.iter().map(|v| v * v).sum::<f64>().sqrt();
        d.iter().map(|v| v / n).collect()
    };
    let (seed_dir, iso_ne_dir) = (toward(&position), toward(&cameras[0].translate));
    for i in 0..3 {
        assert!((target[i] - center[i]).abs() < 1e-9, "target {target:?}");
        assert!(
            (seed_dir[i] - iso_ne_dir[i]).abs() < 1e-9,
            "{seed_dir:?} vs {iso_ne_dir:?}"
        );
    }

    // The box leaves out the floor plan and the ground: a plan reaching far beyond the arm
    // changes neither the rig nor the opening view.
    let (_zip, usda_with_plan, _plan_dir) = compose_one_six_axis_robot_with_floor_plan(
        "v -40 -30 0\nv 60 50 0\nl 1 2\n",
        ComposeOptions::default(),
    );
    assert!(usda_with_plan.contains("    def Xform \"floor_plan\"\n"));
    assert!(usda_with_plan.contains("    def Mesh \"Ground\" (\n"));
    let rig_text = |usda: &str| {
        let start = usda.find("    def Scope \"Cameras\"\n").unwrap();
        usda[start..start + usda[start..].find("\n    }\n").unwrap()].to_owned()
    };
    assert_eq!(rig_text(&usda_with_plan), rig_text(&usda));
    assert_eq!(perspective_seed(&usda_with_plan), perspective_seed(&usda));

    // Nothing to frame: no rig, and the template's opening view.
    let dir = tempfile::tempdir().unwrap();
    let manifest = dir.path().join("empty.yaml");
    std::fs::write(&manifest, "name: \"e\"\n").unwrap();
    let scene = parse_scene(&manifest).unwrap();
    let out = dir.path().join("empty.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let empty = read_zip_entry(&out, "empty.usda");
    assert!(!empty.contains("Cameras") && !empty.contains("def Camera"));
    assert!(empty.contains("double3 position = (5, 5, 5)"));
}

/// The `cameraSettings.Perspective` dictionary of a composed USDA's header.
fn perspective_seed(usda: &str) -> String {
    let start = usda.find("dictionary Perspective = {").unwrap();
    usda[start..start + usda[start..].find('}').unwrap()].to_owned()
}

/// [`compose_one_six_axis_robot`] with a floor plan from the OBJ text `plan_obj`.
fn compose_one_six_axis_robot_with_floor_plan(
    plan_obj: &str,
    opts: ComposeOptions,
) -> (PathBuf, String, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    std::fs::write(dir.path().join("plan.obj"), plan_obj).unwrap();
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(
        &manifest,
        "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"arm.urdf\"\n    position: { x: 1.0, y: 2.0, z: 3.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\nfloor_plan:\n  geometry_file: \"plan.obj\"\n",
    )
    .unwrap();
    let scene = parse_scene(&manifest).unwrap();
    let out = dir.path().join("scene.zip");
    compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");
    (out, usda, dir)
}

/// FR-109: layer metadata — frame rate, `<name>: <description>` doc, provenance — and the model
/// kinds: /World an assembly, the robots and fixtures scopes groups of component models.
#[test]
fn fr_109_layer_metadata_and_model_kinds() {
    let dir = tempfile::tempdir().unwrap();
    write_six_axis_urdf(&dir.path().join("arm.urdf"));
    let fixture = dir.path().join("fx/f.urdf");
    std::fs::create_dir_all(fixture.parent().unwrap()).unwrap();
    write_fixture_urdf(&fixture);
    let manifest = dir.path().join("cell.yaml");
    std::fs::write(
        &manifest,
        "name: \"Demo Cell\"\n\
         description: \"two \\\"quoted\\\" words\\nand\\ta tab\"\n\
         scene_origin: { x: 12.75, y: -3.5, z: 0 }\n\
         robots:\n\
         \x20 - robot_id: \"R1\"\n\
         \x20   dressed_urdf: \"arm.urdf\"\n\
         \x20   position: { x: 14.0, y: -2.5, z: 0.0 }\n\
         \x20   orientation: { a: 0.0, b: 0.0, c: 0.0 }\n\
         fixtures:\n\
         \x20 - id: \"F1\"\n\
         \x20   urdf: \"fx/f.urdf\"\n\
         \x20   position: { x: 12.0, y: -4.0, z: 0.0 }\n\
         \x20   orientation: { a: 0.0, b: 0.0, c: 0.0 }\n",
    )
    .unwrap();
    let scene = parse_scene(&manifest).unwrap();
    let out = dir.path().join("cell.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "cell.usda");
    let header = &usda[..usda.find("\n)\n").unwrap() + 3];

    assert!(header.contains("    framesPerSecond = 60\n"));
    assert!(header.contains("    timeCodesPerSecond = 60\n"));
    assert!(header.contains(&format!(
        "string \"robo_fitting_room:generator\" = \"robo-fitting-room {}\"\n",
        env!("CARGO_PKG_VERSION")
    )));
    assert!(
        header.contains("double3 \"robo_fitting_room:scene_origin_m\" = (12.75, -3.5, 0)\n")
    );
    // The description's quotes, newline and tab stay escaped on the one doc line.
    assert!(
        header.contains("    doc = \"Demo Cell: two \\\"quoted\\\" words\\nand\\ta tab\"\n)\n"),
        "{header}"
    );

    for model in [
        "def Xform \"World\" (\n    kind = \"assembly\"\n)\n",
        "    def Scope \"robots\" (\n        kind = \"group\"\n    )\n",
        "    def Scope \"fixtures\" (\n        kind = \"group\"\n    )\n",
        "    def Xform \"R1\" (\n        prepend apiSchemas = [\"PhysicsArticulationRootAPI\", \"PhysxArticulationAPI\"]\n        kind = \"component\"\n    )\n",
        "        def Xform \"F1\" (\n            kind = \"component\"\n        )\n",
    ] {
        assert!(usda.contains(model), "missing {model}");
    }
    assert_eq!(usda.matches("kind = \"component\"").count(), 2);
    assert_eq!(usda.matches("kind = \"group\"").count(), 2);
    assert_eq!(usda.matches("kind = \"assembly\"").count(), 1);
}

// =============================================================================
// 2.2 URDF-to-USD Conversion
// =============================================================================

/// FR-110: Convert URDF joint/link trees into USD PhysX articulations
#[test]
fn fr_110_urdf_to_usd_articulations() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions {
        full_robots: true,
        full_tools: true,
    });
    // Articulation root with PhysX schema
    assert!(usda.contains("PhysicsArticulationRootAPI"));
    assert!(usda.contains("PhysxArticulationAPI"));
    // Six revolute joints emitted in URDF declaration order
    for n in 1..=6 {
        assert!(
            usda.contains(&format!("def PhysicsRevoluteJoint \"joint_{}\"", n)),
            "joint_{} missing",
            n
        );
    }
    // Joint limits in degrees (URDF -3.14..3.14 rad → ~-179.9..179.9 deg)
    assert!(usda.contains("physics:lowerLimit = -179."));
    assert!(usda.contains("physics:upperLimit = 179."));
}

/// FR-111: Convert URDF visual meshes into USD mesh prims (inlined geometry), their MTL colours
/// (sRGB) linear
#[test]
fn fr_111_visual_mesh_to_usd() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions {
        full_robots: true,
        full_tools: true,
    });
    // Visual prim emitted, geometry inlined (no external `references = @...@`)
    assert!(usda.contains("def Mesh \"visual_0\""));
    assert!(usda.contains("point3f[] points = ["));
    assert!(usda.contains("int[] faceVertexIndices = ["));
    assert!(usda.contains("int[] faceVertexCounts = ["));
    // No external reference anywhere in the stage (the environment is inline too, NFR-108),
    // so none inside a def Mesh block either.
    assert!(!usda.contains("references = @"));
    let mut depth = 0i32;
    let mut in_mesh = false;
    for line in usda.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("def Mesh ") {
            in_mesh = true;
        }
        if in_mesh {
            assert!(
                !trimmed.starts_with("prepend references = @")
                    && !trimmed.starts_with("references = @"),
                "mesh data must be inlined; found external reference in def Mesh: {}",
                line
            );
            depth += line.matches('{').count() as i32;
            depth -= line.matches('}').count() as i32;
            if depth <= 0 {
                in_mesh = false;
                depth = 0;
            }
        }
    }

    // The visual's MTL colour, Kd 0.5 (sRGB), is bound linear: 0.214041.
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("meshes")).unwrap();
    std::fs::write(
        dir.path().join("meshes/m.obj"),
        "mtllib m.mtl\nusemtl Grey\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .unwrap();
    std::fs::write(dir.path().join("meshes/m.mtl"), "newmtl Grey\nKd 0.5 0.5 0.5\n").unwrap();
    std::fs::write(
        dir.path().join("grey.urdf"),
        "<?xml version=\"1.0\"?>\n<robot name=\"grey\">\n  <link name=\"base\">\n    <visual><geometry><mesh filename=\"./meshes/m.obj\"/></geometry></visual>\n  </link>\n</robot>\n",
    )
    .unwrap();
    let manifest = dir.path().join("grey.yaml");
    std::fs::write(&manifest, "name: \"g\"\nfixtures:\n  - id: \"F1\"\n    urdf: \"grey.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n").unwrap();
    let scene = parse_scene(&manifest).unwrap();
    let out = dir.path().join("grey.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "grey.usda");
    assert!(usda.contains("rel material:binding = </World/Looks/grey__Grey>"));
    let material = &usda[usda.find("def Material \"grey__Grey\"").unwrap()..];
    assert!(
        material[..material.find("token outputs:surface\n").unwrap()]
            .contains("color3f inputs:diffuseColor = (0.214041, 0.214041, 0.214041)\n"),
        "{material}"
    );
}

/// FR-112: Convert URDF collision meshes into USD collision geometry (convex hull)
#[test]
fn fr_112_collision_to_usd() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions {
        full_robots: true,
        full_tools: true,
    });
    assert!(usda.contains("def Mesh \"collision_0\""));
    assert!(usda.contains("PhysicsCollisionAPI"));
    assert!(usda.contains("PhysicsMeshCollisionAPI"));
    assert!(usda.contains("uniform token physics:approximation = \"convexHull\""));

    // URDF primitives are colliders too, as their UsdGeom prims: a box the unit Cube scaled to
    // its size, a cylinder a Z Cylinder, a sphere a Sphere; PhysX collides with them as they are.
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("prims.urdf");
    std::fs::write(
        &urdf,
        r#"<?xml version="1.0"?><robot name="p"><link name="base">
  <collision><geometry><box size="0.5 1 2"/></geometry></collision>
  <collision><origin xyz="0 0 1" rpy="0 0 0"/><geometry><cylinder radius="0.1" length="0.4"/></geometry></collision>
  <collision><geometry><sphere radius="0.25"/></geometry></collision>
</link></robot>"#,
    )
    .unwrap();
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");
    for (prim, attribute) in [
        (
            "def Cube \"collision_0\" (",
            "double3 xformOp:scale = (0.5, 1, 2)",
        ),
        ("def Cylinder \"collision_1\" (", "double height = 0.4"),
        ("def Sphere \"collision_2\" (", "double radius = 0.25"),
    ] {
        let at = usda.find(prim).unwrap_or_else(|| panic!("no {prim}"));
        let block = &usda[at..at + usda[at..].find("\n            }\n").unwrap()];
        assert!(
            block
                .contains("prepend apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]"),
            "{block}"
        );
        assert!(block.contains(attribute), "{block}");
        assert!(!block.contains("approximation"), "{block}");
    }
    assert!(!usda.contains("# inline"));
    // The link weighs its primitives (FR-113) and reports their contacts (FR-160).
    assert!(usda.contains("def IsaacContactSensor \"ContactSensor_node\""));
}

/// FR-113: one mass source (`drives::link_mass_properties`) for the emission and the drive tuning
/// (FR-158): a URDF inertial as authored unless its implied density over the link's robot geometry
/// marks it a placeholder (below a quarter of the link density), else that geometry at the link
/// density (the manifest's `link_density_kg_m3`, else the default); tool hulls at the same density
/// without a payload; a 10 kg fallback for links welded to the root with neither, 0.1 kg for a
/// frame link a joint moves; the principal axes with every diagonal inertia.
#[test]
fn fr_113_inertial_to_usd() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions {
        full_robots: true,
        full_tools: true,
    });
    // The six-axis links' triangles enclose no volume: their URDF inertials as authored.
    for (name, mass, [ixx, iyy, izz]) in SIX_AXIS_LINKS {
        let prim = &usda[usda.find(&format!("def Xform \"{name}\" (")).unwrap()..];
        let prim = &prim[..prim.find("def Mesh").unwrap()];
        assert!(
            prim.contains(&format!("float physics:mass = {mass}\n")),
            "{prim}"
        );
        assert!(
            prim.contains(&format!(
                "float3 physics:diagonalInertia = ({ixx}, {iyy}, {izz})\n"
            )),
            "{prim}"
        );
    }
    // world_link (no inertial, no volume) gets the 10 kg fallback.
    let world = &usda[usda.find("def Xform \"world_link\" (").unwrap()..];
    assert!(world[..world.find("def Mesh").unwrap()].contains("float physics:mass = 10\n"));
    // Every rigid body carries PhysicsMassAPI with a mass, and its principal axes with its
    // diagonal inertia: here the links' own axes, the identity.
    let bodies = usda.matches("PhysicsRigidBodyAPI").count();
    assert_eq!(bodies, 8);
    assert_eq!(usda.matches("PhysicsMassAPI").count(), bodies);
    assert_eq!(usda.matches("float physics:mass = ").count(), bodies);
    assert_eq!(
        usda.matches("float3 physics:diagonalInertia = ").count(),
        bodies
    );
    assert_eq!(
        usda.matches("quatf physics:principalAxes = (1, 0, 0, 0)\n")
            .count(),
        bodies
    );

    // Closed geometry: `plate` a 1 × 0.5 × 0.2 m robot box (0.1 m³) at (0.3, 0, 0.1) with an
    // exporter's 2 kg placeholder; `casting` the same box at its origin with a 60 kg inertial at
    // (0.1, 0.2, 0), turned by rpy (0, 0, 0.5), with a product of inertia; `gripper` a
    // 0.4 × 0.2 × 0.6 m tool box (0.048 m³) and no inertial.
    let dir = tempfile::tempdir().unwrap();
    let write_box = |path: &str, [hx, hy, hz]: [f64; 3]| {
        let mut obj = String::new();
        for i in 0..8 {
            let s = |bit: usize, h: f64| if i & (1 << bit) == 0 { -h } else { h };
            obj.push_str(&format!("v {} {} {}\n", s(0, hx), s(1, hy), s(2, hz)));
        }
        for [a, b, c, d] in [
            [1, 3, 4, 2],
            [5, 6, 8, 7],
            [1, 2, 6, 5],
            [3, 7, 8, 4],
            [1, 5, 7, 3],
            [2, 4, 8, 6],
        ] {
            obj.push_str(&format!("f {a} {b} {c} {d}\n"));
        }
        let path = dir.path().join(path);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, obj).unwrap();
    };
    write_box("meshes/robot/slab.obj", [0.5, 0.25, 0.1]);
    write_box("meshes/tool/grip.obj", [0.2, 0.1, 0.3]);
    let collision = |xyz: &str, file: &str| {
        format!("    <collision>\n      <origin xyz=\"{xyz}\" rpy=\"0 0 0\"/>\n      <geometry>\n        <mesh filename=\"./meshes/{file}\"/>\n      </geometry>\n    </collision>\n")
    };
    let inertial = |mass: f64, xyz: &str, rpy: &str, tensor: &str| {
        format!("    <inertial>\n      <origin xyz=\"{xyz}\" rpy=\"{rpy}\"/>\n      <mass value=\"{mass}\"/>\n      <inertia {tensor}/>\n    </inertial>\n")
    };
    let urdf = dir.path().join("boxes.urdf");
    std::fs::write(
        &urdf,
        format!(
            "<?xml version=\"1.0\"?>\n<robot name=\"boxes\">\n  <link name=\"World\"/>\n  <link name=\"plate\">\n{}{}  </link>\n  <link name=\"casting\">\n{}{}  </link>\n  <link name=\"gripper\">\n{}  </link>\n  <joint name=\"root_joint\" type=\"fixed\">\n    <parent link=\"World\"/>\n    <child link=\"plate\"/>\n  </joint>\n  <joint name=\"j1\" type=\"revolute\">\n    <parent link=\"plate\"/>\n    <child link=\"casting\"/>\n    <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-3\" upper=\"3\" effort=\"100\" velocity=\"1\"/>\n  </joint>\n  <joint name=\"tool_joint\" type=\"fixed\">\n    <parent link=\"casting\"/>\n    <child link=\"gripper\"/>\n    <origin xyz=\"1 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n</robot>\n",
            inertial(2.0, "0 0 0", "0 0 0", "ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\""),
            collision("0.3 0 0.1", "robot/slab.obj"),
            inertial(60.0, "0.1 0.2 0", "0 0 0.5", "ixx=\"2\" ixy=\"0.5\" ixz=\"0\" iyy=\"3\" iyz=\"0\" izz=\"4\""),
            collision("0 0 0", "robot/slab.obj"),
            collision("0 0 0", "tool/grip.obj"),
        ),
    )
    .unwrap();
    let manifest = dir.path().join("cell.yaml");
    let compose = |density: &str| -> String {
        std::fs::write(
            &manifest,
            format!("name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"boxes.urdf\"\n    position: {{ x: 0.0, y: 0.0, z: 0.0 }}\n    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}\n    home_pose_deg: [0]\n{density}"),
        )
        .unwrap();
        let scene = parse_scene(&manifest).unwrap();
        assert!(validate_scene(&scene).is_empty());
        let out = dir.path().join("cell.zip");
        compose_stage(&scene, &manifest, &out).unwrap();
        read_zip_entry(&out, "cell.usda")
    };
    let block = |usda: &str, link: &str| -> String {
        let start = usda
            .find(&format!("        def Xform \"{link}\" ("))
            .unwrap();
        let rest = &usda[start..];
        rest[..rest.find("def Mesh").unwrap()].to_string()
    };
    let numbers = |block: &str, key: &str| -> Vec<f64> {
        let line = block
            .lines()
            .find(|l| l.contains(key))
            .unwrap_or_else(|| panic!("no {key} in {block}"));
        let inner = line
            .split_once(" = ")
            .unwrap()
            .1
            .trim_matches(|c| c == '(' || c == ')');
        inner
            .split(',')
            .map(|v| v.trim().parse().unwrap())
            .collect()
    };
    // The tensor the emitted principal moments and axes make: A·diag(I)·Aᵀ.
    let tensor = |block: &str| -> [[f64; 3]; 3] {
        let d = numbers(block, "physics:diagonalInertia");
        let q = numbers(block, "physics:principalAxes");
        let a = frame([0.0; 3], [q[0], q[1], q[2], q[3]]);
        let mut t = [[0.0; 3]; 3];
        for (i, row) in t.iter_mut().enumerate() {
            for (j, entry) in row.iter_mut().enumerate() {
                *entry = (0..3).map(|k| a[i][k] * d[k] * a[j][k]).sum();
            }
        }
        t
    };
    let close = |got: f64, want: f64, tol: f64, what: &str| {
        assert!((got - want).abs() <= tol, "{what}: {got} != {want}");
    };
    // A box's inertia per kilogram about its centre: diag((b² + c²)/12, (a² + c²)/12, (a² + b²)/12).
    let box_moments = |[a, b, c]: [f64; 3], mass: f64| {
        [
            mass * (b * b + c * c) / 12.0,
            mass * (a * a + c * c) / 12.0,
            mass * (a * a + b * b) / 12.0,
        ]
    };

    // At the default link density (1200 kg/m³; placeholders below a quarter of it, 300 kg/m³):
    let usda = compose("");
    // `plate`: 2 kg over 0.1 m³ (20 kg/m³) is a placeholder, so the box at 1200 kg/m³: 120 kg
    // at its centre with its tensor, on the link's axes.
    let plate = block(&usda, "plate");
    close(
        numbers(&plate, "physics:mass")[0],
        120.0,
        1e-6,
        "plate mass",
    );
    for (g, w) in numbers(&plate, "physics:centerOfMass")
        .iter()
        .zip([0.3, 0.0, 0.1])
    {
        close(*g, w, 1e-6, "plate centre of mass");
    }
    for (g, w) in numbers(&plate, "physics:diagonalInertia")
        .iter()
        .zip(box_moments([1.0, 0.5, 0.2], 120.0))
    {
        close(*g, w, 1e-5, "plate inertia");
    }
    assert!(
        plate.contains("quatf physics:principalAxes = (1, 0, 0, 0)\n"),
        "{plate}"
    );
    // `casting`: 60 kg over 0.1 m³ (600 kg/m³) clears the line: as authored, its tensor turned
    // into the link's axes by the inertial's rpy (R·I·Rᵀ) and diagonalised.
    let casting = block(&usda, "casting");
    assert!(casting.contains("float physics:mass = 60\n"), "{casting}");
    assert!(
        casting.contains("point3f physics:centerOfMass = (0.1, 0.2, 0)\n"),
        "{casting}"
    );
    let (s, c) = 0.5f64.sin_cos();
    let r = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
    let authored = [[2.0, 0.5, 0.0], [0.5, 3.0, 0.0], [0.0, 0.0, 4.0]];
    let got = tensor(&casting);
    for i in 0..3 {
        for j in 0..3 {
            let want: f64 = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| r[i][k] * authored[k][l] * r[j][l])
                .sum();
            close(got[i][j], want, 5e-5, "casting tensor");
        }
    }
    assert!(
        !casting.contains("quatf physics:principalAxes = (1, 0, 0, 0)\n"),
        "{casting}"
    );
    // `gripper`: no inertial, no payload, so its tool box at the link density: 57.6 kg.
    let gripper = block(&usda, "gripper");
    close(
        numbers(&gripper, "physics:mass")[0],
        57.6,
        1e-6,
        "gripper mass",
    );
    for (g, w) in numbers(&gripper, "physics:diagonalInertia")
        .iter()
        .zip(box_moments([0.4, 0.2, 0.6], 57.6))
    {
        close(*g, w, 1e-5, "gripper inertia");
    }
    // `world_link`: neither, so the fallback.
    assert!(block(&usda, "world_link").contains("float physics:mass = 10\n"));
    assert_eq!(usda.matches("float3 physics:diagonalInertia = ").count(), 4);
    assert_eq!(usda.matches("quatf physics:principalAxes = ").count(), 4);

    // The manifest's link density: 2000 kg/m³ scales the derived masses; the casting's 60 kg
    // still clears the line (a quarter, 500 kg/m³).
    let usda = compose("    link_density_kg_m3: 2000\n");
    close(
        numbers(&block(&usda, "plate"), "physics:mass")[0],
        200.0,
        1e-6,
        "plate at 2000",
    );
    close(
        numbers(&block(&usda, "gripper"), "physics:mass")[0],
        96.0,
        1e-6,
        "gripper at 2000",
    );
    assert!(block(&usda, "casting").contains("float physics:mass = 60\n"));
    // At 2500 kg/m³ it doesn't (600 < 625 kg/m³): the casting box at the density, 250 kg.
    let usda = compose("    link_density_kg_m3: 2500\n");
    let casting = block(&usda, "casting");
    close(
        numbers(&casting, "physics:mass")[0],
        250.0,
        1e-6,
        "casting at 2500",
    );
    for (g, w) in numbers(&casting, "physics:centerOfMass")
        .iter()
        .zip([0.0, 0.0, 0.0])
    {
        close(*g, w, 1e-6, "casting box centre");
    }
    // A density that isn't finite and > 0 fails validation.
    std::fs::write(
        &manifest,
        "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"boxes.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n    home_pose_deg: [0]\n    link_density_kg_m3: 0\n",
    )
    .unwrap();
    let errors: Vec<String> = validate_scene(&parse_scene(&manifest).unwrap())
        .iter()
        .map(|e| e.to_string())
        .collect();
    assert!(
        errors.len() == 1 && errors[0].contains("link_density_kg_m3 must be finite and > 0"),
        "{errors:?}"
    );

    // Frame links (neither geometry nor an inertial): the root and what is welded to it 10 kg
    // (nothing moves them), a frame a joint moves 0.1 kg as a 5 cm sphere — a flange or tool0
    // must not load the wrist.
    std::fs::write(
        dir.path().join("frames.urdf"),
        "<robot name=\"f\"><link name=\"World\"/><link name=\"base\"/><link name=\"flange\"/><link name=\"tool0\"/>\
         <joint name=\"root_joint\" type=\"fixed\"><parent link=\"World\"/><child link=\"base\"/></joint>\
         <joint name=\"j1\" type=\"revolute\"><parent link=\"base\"/><child link=\"flange\"/><axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>\
         <joint name=\"tip\" type=\"fixed\"><parent link=\"flange\"/><child link=\"tool0\"/><origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/></joint></robot>",
    )
    .unwrap();
    std::fs::write(
        &manifest,
        "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"frames.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n    home_pose_deg: [0]\n",
    )
    .unwrap();
    let out = dir.path().join("cell.zip");
    compose_stage(&parse_scene(&manifest).unwrap(), &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "cell.usda");
    let mass = |link: &str| {
        let at = usda
            .find(&format!("        def Xform \"{link}\" ("))
            .unwrap();
        numbers(&usda[at..], "physics:mass")[0]
    };
    assert_eq!((mass("world_link"), mass("base")), (10.0, 10.0));
    assert_eq!((mass("flange"), mass("tool0")), (0.1, 0.1));
    let at = usda.find("        def Xform \"tool0\" (").unwrap();
    assert_eq!(numbers(&usda[at..], "physics:diagonalInertia"), [0.0001; 3]);
}

/// FR-114: Deterministic USD prim paths from robot ID and link/joint names
#[test]
fn fr_114_deterministic_prim_paths() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // Robot Xform path uses the manifest robot_id ("R1")
    assert!(usda.contains("def Xform \"R1\""));
    // URDF link "World" sanitised to "world_link" (avoids stage-root shadowing)
    assert!(usda.contains("def Xform \"world_link\""));
    // Other link names preserved
    assert!(usda.contains("def Xform \"arm_link_3\""));
}

/// FR-115: Anchor URDF root link to placement pose via PhysicsFixedJoint "world_anchor"
#[test]
fn fr_115_world_link_anchoring() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.contains("def PhysicsFixedJoint \"world_anchor\""));
    // Anchor's localPos0 = manifest position [1, 2, 3]
    assert!(
        usda.contains("point3f physics:localPos0 = (1, 2, 3)"),
        "anchor must carry the placement xyz"
    );
    // body1 rel points at the URDF root link (sanitised)
    assert!(usda.contains("rel physics:body1 = </World/robots/R1/world_link>"));
}

/// FR-116: Forward-kinematics bind poses pre-computed at home configuration. (They show the home
/// pose and keep the joint frames together; the start state is FR-157's joint state.)
#[test]
fn fr_116_fk_bind_poses() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // Each link Xform uses a single matrix4d xformOp:transform (not split translate+orient)
    assert!(usda.contains("matrix4d xformOp:transform = ("));
    assert!(
        usda.contains("uniform token[] xformOpOrder = [\"xformOp:transform\"]"),
        "links must use single matrix4d op"
    );
    // The home pose places joint_2 at -90° around its z-axis. Link_2 sits two joint origins
    // (0, 0, 0.5) above the base and accumulates the -90° rotation in its bind pose: X maps to −Y.
    let poses = usd_link_poses(&usda);
    let link_2 = poses["arm_link_2"];
    let expected = [
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    for (row, want) in link_2.iter().zip(expected) {
        for (g, w) in row.iter().zip(want) {
            assert!((g - w).abs() < 1e-12, "{link_2:?}");
        }
    }
}

/// Helper: the text right of the `=` of attribute `key` in joint `joint_name`'s block of a
/// composed USDA (e.g. `drive:angular:physics:targetPosition`, degrees). `None` if the joint or
/// attribute isn't found.
fn joint_attr_text<'a>(usda: &'a str, joint_name: &str, key: &str) -> Option<&'a str> {
    let mut in_block = false;
    for l in usda.lines() {
        let t = l.trim_start();
        if t.starts_with("def Physics") && t.contains(&format!("Joint \"{}\"", joint_name)) {
            in_block = true;
        } else if in_block && l.starts_with("        }") {
            break;
        } else if in_block && t.contains(&format!(" {} = ", key)) {
            return Some(t.split(" = ").nth(1)?.trim());
        }
    }
    None
}

/// Helper: [`joint_attr_text`] as a number.
fn joint_attr(usda: &str, joint_name: &str, key: &str) -> Option<f64> {
    joint_attr_text(usda, joint_name, key)?.parse::<f64>().ok()
}

/// Helper: the joint's drive target and initial state are the very same text (FR-157), and the
/// joint starts at rest.
fn assert_starts_on_target(usda: &str, joint_name: &str) {
    let target = joint_attr_text(usda, joint_name, "drive:angular:physics:targetPosition");
    let state = joint_attr_text(usda, joint_name, "state:angular:physics:position");
    assert!(target.is_some(), "{joint_name}: no target");
    assert_eq!(state, target, "{joint_name}: state != target");
    assert_eq!(
        joint_attr_text(usda, joint_name, "state:angular:physics:velocity"),
        Some("0"),
        "{joint_name}"
    );
}

/// FR-117: default fallback path. When `home_pose_deg` is omitted on a
/// robot, the KUKA stowed pose `[0, -90°, +90°, 0, 0, 0]` drives the
/// per-joint `targetPosition` and the initial joint state. FR-118 covers the per-robot override.
#[test]
fn fr_117_home_pose_drive_targets() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // Targets emitted in degrees on revolute drives.
    let key = "drive:angular:physics:targetPosition";
    let j1 = joint_attr(&usda, "joint_1", key).expect("joint_1 drive target");
    let j2 = joint_attr(&usda, "joint_2", key).expect("joint_2 drive target");
    let j3 = joint_attr(&usda, "joint_3", key).expect("joint_3 drive target");
    assert!(j1.abs() < 1e-6);
    assert!((j2 + 90.0).abs() < 1e-3);
    assert!((j3 - 90.0).abs() < 1e-3);
    for n in 1..=6 {
        assert_starts_on_target(&usda, &format!("joint_{n}"));
    }
}

/// FR-118: per-robot `home_pose_deg` override drives the per-joint
/// `targetPosition` and initial joint state directly (not the KUKA fallback).
#[test]
fn fr_118_per_robot_home_pose() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();

    let scene = SceneDefinition {
        name: "override".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: Some(vec![10.0, -45.0, 30.0, 0.0, 0.0, 0.0]),
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage_with_options(&scene, &manifest, &out, &ComposeOptions::default()).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");

    let key = "drive:angular:physics:targetPosition";
    let j1 = joint_attr(&usda, "joint_1", key).expect("joint_1");
    let j2 = joint_attr(&usda, "joint_2", key).expect("joint_2");
    let j3 = joint_attr(&usda, "joint_3", key).expect("joint_3");
    assert!((j1 - 10.0).abs() < 1e-3, "j1 = {}", j1);
    assert!((j2 + 45.0).abs() < 1e-3, "j2 = {}", j2);
    assert!((j3 - 30.0).abs() < 1e-3, "j3 = {}", j3);
    for n in 1..=6 {
        assert_starts_on_target(&usda, &format!("joint_{n}"));
    }
}

/// FR-118: every revolute joint's home angle, given or the KUKA stowed default, must lie within
/// its URDF limits.
#[test]
fn fr_118_home_pose_within_limits() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    // Limits ±3.14 rad = ±179.909°.
    write_six_axis_urdf(&urdf);
    let errors = |home_pose_deg: Option<Vec<f64>>| {
        let scene = SceneDefinition {
            name: "limits".into(),
            description: String::new(),
            robots: vec![RobotPlacement {
                robot_id: "R1".into(),
                dressed_urdf: urdf.clone(),
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        validate_scene(&scene)
            .iter()
            .map(|e| e.to_string())
            .collect::<Vec<_>>()
    };
    assert!(errors(None).is_empty(), "the KUKA default is within limits");
    assert!(errors(Some(vec![179.9, -90.0, 90.0, -179.9, 0.0, 0.0])).is_empty());
    let out = errors(Some(vec![190.0, -90.0, 90.0, 0.0, 0.0, -185.0]));
    assert_eq!(out.len(), 2, "{out:?}");
    assert!(
        out[0].contains("home angle 190 deg of joint 'joint_1' is outside its URDF limits [-179.9087, 179.9087] deg"),
        "{out:?}"
    );
    assert!(out[1].contains("joint 'joint_6'"), "{out:?}");
}

/// FR-118 / FR-157 / FR-158: a prismatic joint's `home_pose_deg` entry is metres: its drive
/// target and initial state, the slide in its link's bind pose, validated against its limits in
/// metres; its drive's stiffness is the weight it moves over a 1 mm sag (N/m).
#[test]
fn fr_118_prismatic_home_in_metres() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("track.urdf");
    std::fs::write(
        &urdf,
        r#"<?xml version="1.0"?><robot name="track">
  <link name="World"/>
  <link name="base"/>
  <link name="carriage"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="50"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <link name="arm"><inertial><origin xyz="1 0 0" rpy="0 0 0"/><mass value="10"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <joint name="root_joint" type="fixed"><parent link="World"/><child link="base"/></joint>
  <joint name="track" type="prismatic"><parent link="base"/><child link="carriage"/><axis xyz="1 0 0"/><limit lower="0.5" upper="3" effort="1e9" velocity="1.5"/></joint>
  <joint name="turn" type="revolute"><parent link="carriage"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1e9" velocity="1"/></joint>
</robot>"#,
    )
    .unwrap();
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let scene = |home: Option<Vec<f64>>| SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf.clone(),
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: home,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errors = |home: Option<Vec<f64>>| -> Vec<String> {
        validate_scene(&scene(home))
            .iter()
            .map(|e| e.to_string())
            .collect()
    };
    assert!(errors(Some(vec![1.25, 30.0])).is_empty());
    let out = errors(Some(vec![3.5, 30.0]));
    assert!(
        out.len() == 1
            && out[0].contains("home position 3.5 m of joint 'track' is outside its URDF limits [0.5000, 3.0000] m"),
        "{out:?}"
    );
    // Without a home pose the slide sits at 0 m — outside [0.5, 3] m, so reported too.
    let out = errors(None);
    assert!(
        out.iter()
            .any(|e| e.contains("home position 0 m of joint 'track'")),
        "{out:?}"
    );

    let out = dir.path().join("scene.zip");
    compose_stage(&scene(Some(vec![1.25, 30.0])), &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");
    assert_eq!(
        joint_attr_text(&usda, "track", "drive:linear:physics:targetPosition"),
        Some("1.25")
    );
    assert_eq!(
        joint_attr_text(&usda, "track", "state:linear:physics:position"),
        Some("1.25")
    );
    assert_eq!(joint_attr(&usda, "track", "physics:lowerLimit"), Some(0.5));
    assert_eq!(joint_attr(&usda, "track", "physics:upperLimit"), Some(3.0));
    assert_eq!(
        joint_attr_text(&usda, "track", "physxJoint:maxJointVelocity"),
        Some("1.5")
    );
    let poses = usd_link_poses(&usda);
    assert!(
        (poses["carriage"][0][3] - 1.25).abs() < 1e-9,
        "{:?}",
        poses["carriage"]
    );
    // FR-158: k = (50 + 10 kg)·9.81 m/s² / 1 mm = 588600 N/m; damping 2·0.7·√(k·m) N·s/m.
    let k = 60.0 * 9.81 / 1e-3;
    assert_eq!(
        joint_attr(&usda, "track", "drive:linear:physics:stiffness"),
        Some(588600.0)
    );
    let damping = joint_attr(&usda, "track", "drive:linear:physics:damping").unwrap();
    assert!(
        (damping - 2.0 * 0.7 * (k * 60.0f64).sqrt()).abs() < 1e-3,
        "{damping}"
    );
    // The revolute joint keeps its per-degree stiffness.
    assert_eq!(
        joint_attr(&usda, "turn", "drive:angular:physics:stiffness"),
        Some(150000.0)
    );
}

/// FR-118: home_pose_deg length must match the URDF's actuated-joint count.
#[test]
fn fr_118_home_pose_length_validation() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);

    // URDF has 6 actuated joints (joint_1..6); supply only 3 → mismatch error.
    let scene = SceneDefinition {
        name: "bad".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: Some(vec![0.0, 0.0, 0.0]),
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errs = validate_scene(&scene);
    assert!(
        errs.iter().any(|e| {
            let s = e.to_string();
            s.contains("home_pose_deg") && s.contains("3 entries") && s.contains("6 actuated")
        }),
        "expected length-mismatch validation error, got: {:?}",
        errs
    );
}

/// FR-119: a per-robot pedestal raises the robot and emits a static
/// collision cylinder beneath it, spanning the robot's placement height (the
/// floor at z = 0, or a platform) to its raised foot.
#[test]
fn fr_119_pedestal_raises_robot_and_emits_cylinder() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();

    let scene = SceneDefinition {
        name: "ped".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [1.0, 2.0, 3.0],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: Some(Pedestal {
                height_mm: 500.0,
                diameter_mm: 800.0,
            }),
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage_with_options(&scene, &manifest, &out, &ComposeOptions::default()).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");

    // 800mm dia → r=0.4, 500mm tall → h=0.5, spanning the robot's placement height (z = 3) to
    // its raised foot: centred at z = 3.25.
    assert!(usda.contains("def Scope \"pedestals\""));
    assert!(usda.contains("def Cylinder \"socket\""));
    assert!(usda.contains("double radius = 0.4"));
    assert!(usda.contains("double height = 0.5"));
    assert!(usda.contains("double3 xformOp:translate = (1, 2, 3.25)"));
    // Robot foot raised z 3.0 → 3.5 on both Xform and world_anchor.
    assert!(usda.contains("double3 xformOp:translate = (1, 2, 3.5)"));
    assert!(usda.contains("point3f physics:localPos0 = (1, 2, 3.5)"));
    // The welded root links (world_link, and base_link on its fixed joint) don't collide with
    // the pedestal they stand in, nor with the ground (NFR-108); the moving links still do.
    let filter = "rel physics:filteredPairs = [</World/pedestals/pedestal_R1/socket>, </Environment/GroundCollider>]";
    assert_eq!(usda.matches(filter).count(), 2);
    assert_eq!(usda.matches("\"PhysicsFilteredPairsAPI\"").count(), 2);
    let base = &usda[usda.find("def Xform \"arm_base_link\" (").unwrap()..];
    assert!(base[..base.find("def Mesh").unwrap()].contains(filter));
    // Both targets exist: the pedestal's cylinder and the ground's collider.
    assert!(usda.contains("    def Plane \"GroundCollider\" (\n        prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n"));
    // The pedestal is matte: the default material, not the renderer's glossy default.
    let socket = &usda[usda.find("def Cylinder \"socket\" (").unwrap()..];
    let socket = &socket[..socket.find('}').unwrap()];
    assert!(socket.contains("prepend apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]"));
    assert!(socket.contains("rel material:binding = </World/Looks/factoryos_default>"));

    // On the floor (no pedestal), the ground alone.
    let (_out, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    let filter = "rel physics:filteredPairs = [</Environment/GroundCollider>]";
    assert_eq!(usda.matches(filter).count(), 2);
    assert_eq!(usda.matches("filteredPairs").count(), 2);
}

/// FR-119: omitting the pedestal (or height 0) emits no pedestal scope and
/// leaves the robot at its floor z. Diameter defaults to 800mm when only a
/// height is given.
#[test]
fn fr_119_pedestal_defaults_and_absence() {
    // No pedestal → no scope, unchanged z (compose_one_six_axis_robot places at z=3).
    let (_out, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(!usda.contains("def Scope \"pedestals\""));

    // Manifest with only height_mm → diameter defaults to 800mm (r=0.4).
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    std::fs::write(
        &scene_path,
        "name: \"d\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"r.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n    pedestal:\n      height_mm: 1000.0\n",
    )
    .unwrap();
    let scene = parse_scene(&scene_path).unwrap();
    let ped = scene.robots[0].pedestal.unwrap();
    assert!((ped.diameter_mm - 800.0).abs() < 1e-6);
    assert!((ped.height_mm - 1000.0).abs() < 1e-6);
}

// =============================================================================
// 2.3 Runtime Tool Swapping  (BACKLOG)
// =============================================================================

#[test]
fn fr_120_runtime_tool_swap() {
    todo!("Runtime tool swap — backlog");
}
#[test]
fn fr_121_preserve_joint_state() {
    todo!("Joint state preservation across swap — backlog");
}
#[test]
fn fr_122_swap_compatibility_check() {
    todo!("Swap compatibility validation — backlog");
}

// =============================================================================
// 2.4 Spatial Validation  (NOT STARTED)
// =============================================================================

#[test]
fn fr_130_forward_kinematics() {
    todo!("FK reachability not yet implemented");
}
#[test]
fn fr_131_static_collision_detection() {
    todo!("Static collision detection not yet implemented");
}
#[test]
fn fr_132_station_boundary_check() {
    todo!("Station boundary validation not yet implemented");
}
#[test]
fn fr_133_target_reachability() {
    todo!("Target reachability validation not yet implemented");
}

// =============================================================================
// 2.5 CLI & API Interface
// =============================================================================

/// FR-140: compose subcommand emits a zip with an inlined .usda
#[test]
fn fr_140_compose_subcommand() {
    let (zip_path, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(zip_path.exists());
    // Zip contents: manifest + usda (no `meshes/` folder — geometry inlined)
    let f = std::fs::File::open(&zip_path).unwrap();
    let mut z = zip::ZipArchive::new(f).unwrap();
    let mut entries: Vec<String> = (0..z.len()).map(|i| z.by_index(i).unwrap().name().to_string()).collect();
    entries.sort();
    assert!(entries.iter().any(|n| n.ends_with(".yaml")));
    assert!(entries.iter().any(|n| n.ends_with(".usda")));
    // USDA header signals USD ASCII format
    assert!(usda.starts_with("#usda 1.0"));
}

/// FR-141: validate subcommand exits OK on a clean manifest
#[test]
fn fr_141_validate_subcommand() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("robot.urdf");
    std::fs::write(&urdf, r#"<?xml version="1.0"?><robot name="r"><link name="base"/></robot>"#).unwrap();

    let scene_path = dir.path().join("cell.yaml");
    let mut f = std::fs::File::create(&scene_path).unwrap();
    write!(
        f,
        "name: \"Valid\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"robot.urdf\"\n    position: {{ x: 0.0, y: 0.0, z: 0.0 }}\n    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}\n"
    )
    .unwrap();
    let scene = parse_scene(&scene_path).unwrap();
    let errors = validate_scene(&scene);
    assert!(errors.is_empty());
}

#[test]
fn fr_142_launch_subcommand() {
    todo!("launch subcommand not yet implemented");
}
#[test]
fn fr_143_swap_subcommand() {
    todo!("swap subcommand — backlog");
}

/// FR-144: Library API exposes parse_scene, validate_scene, compose_stage(_with_options), ComposeOptions
#[test]
fn fr_144_library_api() {
    let dir = tempfile::tempdir().unwrap();
    let scene_path = dir.path().join("cell.yaml");
    std::fs::write(&scene_path, "name: \"API\"\n").unwrap();
    let scene = parse_scene(&scene_path).unwrap();
    assert_eq!(scene.name, "API");
    let _errors = validate_scene(&scene);

    // Both compose entry points exist and accept the same scene
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let manifest = dir.path().join("m.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let out_a = dir.path().join("a.zip");
    let out_b = dir.path().join("b.zip");
    compose_stage(&scene, &manifest, &out_a).unwrap();
    compose_stage_with_options(&scene, &manifest, &out_b, &ComposeOptions::default()).unwrap();
    assert!(out_a.exists() && out_b.exists());
}

/// FR-145: --full-robots / --full-tools toggles (mesh-emission only, articulation untouched)
#[test]
fn fr_145_full_robots_full_tools_flags() {
    // Default (full_robots=false, full_tools=true): no robot visual meshes
    let (_zip_lean, usda_lean, _d1) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(!usda_lean.contains("def Mesh \"visual_"));
    assert!(usda_lean.contains("def Mesh \"collision_"));

    // full_robots=true: visuals re-appear
    let (_zip_full, usda_full, _d2) = compose_one_six_axis_robot(ComposeOptions {
        full_robots: true,
        full_tools: true,
    });
    assert!(usda_full.contains("def Mesh \"visual_"));

    // Joints are unaffected by the visual toggle in either mode
    for usda in [&usda_lean, &usda_full] {
        for n in 1..=6 {
            assert!(usda.contains(&format!("def PhysicsRevoluteJoint \"joint_{}\"", n)));
        }
    }
}

/// FR-146: Deterministic zip — same manifest produces byte-identical USDA
#[test]
fn fr_146_deterministic_zip() {
    let (_z1, u1, _d1) = compose_one_six_axis_robot(ComposeOptions::default());
    let (_z2, u2, _d2) = compose_one_six_axis_robot(ComposeOptions::default());
    assert_eq!(u1.len(), u2.len());
    assert_eq!(u1, u2);
}

/// Build a two-robot (sharing one dressed URDF) + one-fixture + floor-plan
/// scene on disk and return (scene, manifest_path, tempdir). The manifest YAML
/// carries scene_origin + home_pose_deg so the rewrite must preserve them.
fn urdf_bundle_fixture() -> (SceneDefinition, PathBuf, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    // Author the sources under distinct dirs (src-robots/, src-fixtures/,
    // src-floor/) so the bundled top-level layout (robots/ fixtures/
    // floorplans/) is provably different from the authored paths.
    let robot_urdf = dir.path().join("src-robots/arm/arm.urdf");
    std::fs::create_dir_all(robot_urdf.parent().unwrap()).unwrap();
    write_six_axis_urdf(&robot_urdf);
    let fixture_urdf = dir.path().join("src-fixtures/f/f.urdf");
    std::fs::create_dir_all(fixture_urdf.parent().unwrap()).unwrap();
    write_fixture_urdf(&fixture_urdf);
    // Floor plan OBJ.
    let obj = dir.path().join("src-floor/plan.obj");
    std::fs::create_dir_all(obj.parent().unwrap()).unwrap();
    std::fs::write(&obj, "v 0 0 0\nv 1 0 0\nl 1 2\n").unwrap();

    let manifest = dir.path().join("scene.yaml");
    let yaml = "name: \"bundle\"\n\
        description: \"desc\"\n\
        scene_origin: { x: 1.0, y: 2.0, z: 0.0 }\n\
        robots:\n\
        \x20 - robot_id: \"R1\"\n\
        \x20   dressed_urdf: \"src-robots/arm/arm.urdf\"\n\
        \x20   position: { x: 0.0, y: 0.0, z: 0.0 }\n\
        \x20   orientation: { a: 0.0, b: 0.0, c: 0.0 }\n\
        \x20   home_pose_deg: [0, -90, 90, 0, 0, 0]\n\
        \x20 - robot_id: \"R2\"\n\
        \x20   dressed_urdf: \"src-robots/arm/arm.urdf\"\n\
        \x20   position: { x: 5.0, y: 0.0, z: 0.0 }\n\
        \x20   orientation: { a: 0.0, b: 0.0, c: 0.0 }\n\
        fixtures:\n\
        \x20 - id: \"F1\"\n\
        \x20   urdf: \"src-fixtures/f/f.urdf\"\n\
        \x20   position: { x: 2.0, y: 2.0, z: 0.0 }\n\
        \x20   orientation: { a: 0.0, b: 0.0, c: 0.0 }\n\
        floor_plan:\n\
        \x20 geometry_file: \"src-floor/plan.obj\"\n\
        \x20 scale: 1.0\n";
    std::fs::write(&manifest, yaml).unwrap();

    let scene = parse_scene(&manifest).unwrap();
    (scene, manifest, dir)
}

/// FR-147: compose_urdf_bundle produces a self-contained zip — rewritten scene
/// YAML + deduped source URDFs (own folders, meshes/ preserved) + floor-plan
/// OBJ; deterministic across runs; extracted YAML validates standalone.
#[test]
fn fr_147_urdf_source_bundle() {
    let (scene, manifest, dir) = urdf_bundle_fixture();
    let out = dir.path().join("bundle.zip");
    let report = compose_urdf_bundle(&scene, &manifest, &out).unwrap();
    assert!(out.exists());

    let names = zip_entry_names(&out);
    // Scene YAML present.
    assert!(names.iter().any(|n| n == "scene.yaml"), "names: {:?}", names);
    // Two robots share one URDF dir → exactly one robot bundle folder.
    assert_eq!(report.robot_urdf_count, 1);
    assert_eq!(report.fixture_urdf_count, 1);
    // The shared robot URDF + its mesh are copied under robots/<folder>/.
    assert!(names.iter().any(|n| n.starts_with("robots/") && n.ends_with(".urdf")));
    assert!(names.iter().any(|n| n.starts_with("robots/") && n.contains("meshes/robot/")));
    // Fixture under fixtures/<folder>/.
    assert!(names.iter().any(|n| n.starts_with("fixtures/") && n.ends_with(".urdf")));
    // Floor plan bundled under floorplans/<stem>/<basename>.
    assert!(report.floor_plan_bundled);
    assert!(names.iter().any(|n| n == "floorplans/plan/plan.obj"));

    // Rewritten YAML references bundled relative paths and preserves extras.
    let yaml = read_zip_entry(&out, "scene.yaml");
    assert!(yaml.contains("robots/"), "expected rewritten robot path: {}", yaml);
    assert!(yaml.contains("fixtures/"), "expected rewritten fixture path");
    assert!(yaml.contains("floorplans/plan/plan.obj"), "expected rewritten floor path");
    assert!(!yaml.contains("src-robots/arm/arm.urdf"), "old path must be rewritten");
    assert!(yaml.contains("scene_origin"), "scene_origin must be preserved");
    assert!(yaml.contains("home_pose_deg"), "home_pose_deg must be preserved");

    // Determinism: a second run into a fresh path is byte-identical.
    let out2 = dir.path().join("bundle2.zip");
    compose_urdf_bundle(&scene, &manifest, &out2).unwrap();
    assert_eq!(std::fs::read(&out).unwrap(), std::fs::read(&out2).unwrap());

    // Extracted bundle validates standalone (the decisive self-containedness check).
    let extract = dir.path().join("extracted");
    std::fs::create_dir_all(&extract).unwrap();
    {
        let f = std::fs::File::open(&out).unwrap();
        let mut z = zip::ZipArchive::new(f).unwrap();
        z.extract(&extract).unwrap();
    }
    let extracted_scene = parse_scene(&extract.join("scene.yaml")).unwrap();
    let errors = validate_scene(&extracted_scene);
    assert!(errors.is_empty(), "extracted bundle should validate: {:?}", errors);
}

/// FR-148: a URDF referencing a mesh outside its own directory is rejected.
#[test]
fn fr_148_bundle_rejects_external_mesh() {
    let dir = tempfile::tempdir().unwrap();
    // A mesh that lives OUTSIDE the URDF dir, referenced via `..`.
    let outside_mesh = dir.path().join("shared/m.obj");
    write_obj_triangle(&outside_mesh);
    let urdf = dir.path().join("robot/arm.urdf");
    std::fs::create_dir_all(urdf.parent().unwrap()).unwrap();
    let s = "<?xml version=\"1.0\"?>\n<robot name=\"arm\">\n  <link name=\"base\">\n    <visual>\n      <geometry>\n        <mesh filename=\"../shared/m.obj\"/>\n      </geometry>\n    </visual>\n  </link>\n</robot>\n";
    std::fs::write(&urdf, s).unwrap();

    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"x\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"robot/arm.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n").unwrap();
    let scene = parse_scene(&manifest).unwrap();
    let out = dir.path().join("bundle.zip");
    let err = compose_urdf_bundle(&scene, &manifest, &out).unwrap_err();
    let msg = err.to_string();
    assert!(msg.contains("outside its directory") || msg.contains("absolute path"), "got: {}", msg);
}

/// FR-149: `compose --urdf-bundle` writes a second <stem>-urdf.zip. Exercised
/// here via the library entry points that back the flag (the CLI wiring only
/// derives the path + calls compose_urdf_bundle after compose_stage).
#[test]
fn fr_149_cli_urdf_bundle_flag() {
    let (scene, manifest, dir) = urdf_bundle_fixture();
    // Mirror the CLI: compose the USDA zip, then the derived -urdf.zip.
    let usda_out = dir.path().join("cell.zip");
    compose_stage(&scene, &manifest, &usda_out).unwrap();
    let bundle_out = dir.path().join("cell-urdf.zip");
    compose_urdf_bundle(&scene, &manifest, &bundle_out).unwrap();
    assert!(usda_out.exists());
    assert!(bundle_out.exists());
    // The two artifacts are distinct: USDA zip has a .usda; bundle has URDFs.
    let usda_names = zip_entry_names(&usda_out);
    assert!(usda_names.iter().any(|n| n.ends_with(".usda")));
    let bundle_names = zip_entry_names(&bundle_out);
    assert!(bundle_names.iter().any(|n| n.ends_with(".urdf")));
    assert!(!bundle_names.iter().any(|n| n.ends_with(".usda")));
}

// =============================================================================
// 2.6 ActionGraph & ROS2 Preparation
// =============================================================================

/// FR-150: Per-robot ActionGraph including a FollowJointTrajectory action
/// server — moved to BACKLOG. The publish/subscribe + ArticulationController
/// pieces are emitted today; FR-150 stays red until BL-11 lands the action
/// server node.
#[test]
fn fr_150_actiongraph_generation() {
    todo!("FollowJointTrajectory action server emission — BL-11");
}

/// FR-151: ROS 2 namespace is `<cell_id>/<robot_id>`; digit-leading values
/// in either segment get an `r_` prefix to satisfy rmw_validate_namespace.
#[test]
fn fr_151_ros2_namespace() {
    // Plain ids in both segments → both pass through verbatim.
    // compose_one_six_axis_robot builds a SceneDefinition { name: "t", robot_id: "R1" }.
    let (_z1, u1, _d1) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(
        u1.contains("inputs:nodeNamespace = \"t/R1\""),
        "expected two-segment <cell_id>/<robot_id> namespace, got: {}",
        u1.lines().find(|l| l.contains("inputs:nodeNamespace")).unwrap_or("<none>"),
    );

    // Digit-leading robot_id → r_-prefixed second segment.
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("m.yaml");
    std::fs::write(&manifest, "name: \"CELL01\"\n").unwrap();
    let scene = SceneDefinition {
        name: "CELL01".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "1234R01".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "m.usda");
    assert!(
        usda.contains("inputs:nodeNamespace = \"CELL01/r_1234R01\""),
        "expected CELL01/r_1234R01 namespace, got: {}",
        usda.lines().find(|l| l.contains("inputs:nodeNamespace")).unwrap_or("<none>"),
    );
}

/// FR-152: Scene-level /clock publisher, namespaced under the cell.
/// FQT becomes `/<cell_id>/clock` so multi-cell deployments don't collide.
#[test]
fn fr_152_clock_publisher() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.contains("def OmniGraph \"ActionGraph_clock\""));
    assert!(usda.contains("ROS2PublishClock"));
    assert!(usda.contains("inputs:topicName = \"clock\""));
    // Cell-scoped namespace — the test fixture's manifest name is "t".
    assert!(
        usda.contains("inputs:nodeNamespace = \"t\""),
        "clock publisher must carry the cell namespace",
    );
}

/// FR-153: Joint mappings auto-derived from URDF actuated-joint declaration order
#[test]
fn fr_153_joint_mapping() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // The ArticulationController inputs:jointNames token[] must list joint_1..6 in order
    let needle = "inputs:jointNames = [\"joint_1\", \"joint_2\", \"joint_3\", \"joint_4\", \"joint_5\", \"joint_6\"]";
    assert!(usda.contains(needle), "joint name array out of order or missing");
}

/// FR-154: PhysicsScene tuning (120 Hz step, solver 64/4 that the scene-wide caps let through,
/// drive stiffness 1.5e5 per degree; damping and force/velocity limits per FR-158)
#[test]
fn fr_154_physx_scene_tuning() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.contains("uint physxScene:timeStepsPerSecond = 120"));
    assert!(usda.contains("physxArticulation:solverPositionIterationCount = 64"));
    assert!(usda.contains("physxArticulation:solverVelocityIterationCount = 4"));
    assert!(usda.contains("uniform uint physxScene:maxPositionIterationCount = 64"));
    assert!(usda.contains("uniform uint physxScene:maxVelocityIterationCount = 4"));
    assert_eq!(usda.matches("drive:angular:physics:stiffness = 150000\n").count(), 6);
    for n in 1..=6 {
        let joint = format!("joint_{n}");
        let damping = joint_attr(&usda, &joint, "drive:angular:physics:damping").unwrap();
        let max_force = joint_attr(&usda, &joint, "drive:angular:physics:maxForce").unwrap();
        let velocity = joint_attr(&usda, &joint, "physxJoint:maxJointVelocity").unwrap();
        assert!(damping > 0.0 && damping.is_finite(), "{joint}: {damping}");
        assert!(max_force > 0.0 && max_force < 1e30, "{joint}: {max_force}");
        assert!(velocity > 0.0 && velocity.is_finite(), "{joint}: {velocity}");
    }
}

/// FR-155: GPU broadphase capacities (none below PhysX's defaults)
#[test]
fn fr_155_gpu_broadphase_capacities() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.contains("uint physxScene:gpuFoundLostPairsCapacity = 262144"));
    assert!(usda.contains("uint physxScene:gpuFoundLostAggregatePairsCapacity = 8192"));
    assert!(usda.contains("uint physxScene:gpuTotalAggregatePairsCapacity = 4096"));
    assert!(usda.contains("uint physxScene:gpuMaxRigidContactCount = 1048576"));
    assert!(usda.contains("uint physxScene:gpuMaxRigidPatchCount = 81920"));
}

/// FR-156: the PhysicsScene is written to the schemas — PhysX's applied-schema name, UsdPhysics
/// gravity, only schema attributes with the schema's types — and its scene-wide iteration caps
/// don't undercut the articulations' own counts.
#[test]
fn fr_156_physics_scene_schema_conformance() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    let start = usda
        .find("    def PhysicsScene \"PhysicsScene\" (\n")
        .unwrap();
    let scene = &usda[start..start + usda[start..].find("\n    }\n").unwrap()];

    assert!(scene.contains("        prepend apiSchemas = [\"PhysxSceneAPI\"]\n"));
    assert!(scene.contains("        vector3f physics:gravityDirection = (0, 0, -1)\n"));
    assert!(scene.contains("        float physics:gravityMagnitude = 9.81\n"));
    // Every physxScene attribute is one of PhysxSceneAPI's, with its type.
    let schema = [
        ("bool", "enableGPUDynamics"),
        ("uint", "gpuFoundLostAggregatePairsCapacity"),
        ("uint", "gpuFoundLostPairsCapacity"),
        ("uint", "gpuMaxRigidContactCount"),
        ("uint", "gpuMaxRigidPatchCount"),
        ("uint", "gpuTotalAggregatePairsCapacity"),
        ("uniform uint", "maxPositionIterationCount"),
        ("uniform uint", "maxVelocityIterationCount"),
        ("uint", "timeStepsPerSecond"),
    ];
    let attributes: Vec<&str> = scene
        .lines()
        .map(str::trim)
        .filter(|l| l.contains("physxScene:"))
        .collect();
    assert_eq!(attributes.len(), schema.len(), "{attributes:#?}");
    for (ty, name) in schema {
        let declaration = format!("{ty} physxScene:{name} = ");
        assert!(
            attributes.iter().any(|l| l.starts_with(&declaration)),
            "{declaration}"
        );
    }
    for gone in [
        "PhysxSchemaPhysxSceneAPI",
        "basic:gravity",
        "enableGpuSdfApproximation",
        "physxScene:positionIterations",
        "physxScene:velocityIterations",
        "NewtonSceneAPI",
    ] {
        assert!(!usda.contains(gone), "{gone}");
    }

    let value = |text: &str, key: &str| -> f64 {
        scalar_value(text.lines().find(|l| l.contains(key)).unwrap())
    };
    assert!(
        value(scene, "physxScene:maxPositionIterationCount")
            >= value(&usda, "physxArticulation:solverPositionIterationCount")
    );
    assert!(
        value(scene, "physxScene:maxVelocityIterationCount")
            >= value(&usda, "physxArticulation:solverVelocityIterationCount")
    );
}

/// A 4×4 row-major matrix.
type M4 = [[f64; 4]; 4];

/// Every `def Xform` prim's `matrix4d xformOp:transform` in a composed USDA, by prim name, as a
/// row-major matrix (USD writes the transpose: row vectors, translation in the last row).
fn usd_link_poses(usda: &str) -> std::collections::HashMap<String, M4> {
    let mut poses = std::collections::HashMap::new();
    let mut xform = String::new();
    for line in usda.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("def Xform \"") {
            xform = rest.split('"').next().unwrap().to_string();
        } else if let Some(value) = t.strip_prefix("matrix4d xformOp:transform = ") {
            let v: Vec<f64> = value
                .replace(['(', ')'], " ")
                .split(',')
                .map(|s| s.trim().parse().unwrap())
                .collect();
            let mut m = [[0.0; 4]; 4];
            for (r, row) in m.iter_mut().enumerate() {
                for (c, entry) in row.iter_mut().enumerate() {
                    *entry = v[c * 4 + r];
                }
            }
            poses.insert(xform.clone(), m);
        }
    }
    poses
}

/// A joint of a composed USDA, parsed back as PhysX reads it.
#[derive(Debug, Default)]
struct UsdJoint {
    name: String,
    schemas: String,
    /// The link prim names (empty for the world).
    body0: String,
    body1: String,
    axis: String,
    local_pos0: [f64; 3],
    /// (w, x, y, z)
    local_rot0: [f64; 4],
    local_pos1: [f64; 3],
    /// (w, x, y, z)
    local_rot1: [f64; 4],
    /// The initial joint state: degrees (revolute joints), metres (prismatic ones).
    state: Option<f64>,
    prismatic: bool,
}

fn usd_joints(usda: &str) -> Vec<UsdJoint> {
    let tuple = |v: &str| -> Vec<f64> {
        v.trim_start_matches('(')
            .trim_end_matches(')')
            .split(',')
            .map(|s| s.trim().parse().unwrap())
            .collect()
    };
    let body = |v: &str| v.trim_end_matches('>').rsplit('/').next().unwrap().to_string();
    let mut joints = Vec::new();
    let mut current: Option<UsdJoint> = None;
    for line in usda.lines() {
        let t = line.trim();
        let head = ["def PhysicsRevoluteJoint \"", "def PhysicsPrismaticJoint \"", "def PhysicsFixedJoint \""]
            .iter()
            .find_map(|p| t.strip_prefix(p));
        if let Some(rest) = head {
            current = Some(UsdJoint {
                name: rest.split('"').next().unwrap().to_string(),
                local_rot0: [1.0, 0.0, 0.0, 0.0],
                local_rot1: [1.0, 0.0, 0.0, 0.0],
                prismatic: t.starts_with("def PhysicsPrismaticJoint"),
                ..UsdJoint::default()
            });
            continue;
        }
        let Some(joint) = current.as_mut() else {
            continue;
        };
        if t.starts_with("prepend apiSchemas = ") {
            joint.schemas = t.to_string();
        } else if let Some(v) = t.strip_prefix("rel physics:body0 = <") {
            joint.body0 = body(v);
        } else if let Some(v) = t.strip_prefix("rel physics:body1 = <") {
            joint.body1 = body(v);
        } else if let Some(v) = t.strip_prefix("uniform token physics:axis = ") {
            joint.axis = v.trim_matches('"').to_string();
        } else if let Some(v) = t.strip_prefix("point3f physics:localPos0 = ") {
            joint.local_pos0.copy_from_slice(&tuple(v));
        } else if let Some(v) = t.strip_prefix("quatf physics:localRot0 = ") {
            joint.local_rot0.copy_from_slice(&tuple(v));
        } else if let Some(v) = t.strip_prefix("point3f physics:localPos1 = ") {
            joint.local_pos1.copy_from_slice(&tuple(v));
        } else if let Some(v) = t.strip_prefix("quatf physics:localRot1 = ") {
            joint.local_rot1.copy_from_slice(&tuple(v));
        } else if let Some(v) = t
            .strip_prefix("float state:angular:physics:position = ")
            .or_else(|| t.strip_prefix("float state:linear:physics:position = "))
        {
            joint.state = Some(v.parse().unwrap());
        } else if t == "}" {
            joints.push(current.take().unwrap());
        }
    }
    joints
}

fn mat_mul(a: &M4, b: &M4) -> M4 {
    robo_fitting_room::math3::multiply(a, b)
}

/// The rigid transform translating by `p` after rotating by the unit quaternion `q` (w, x, y, z).
fn frame(p: [f64; 3], q: [f64; 4]) -> M4 {
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    let [w, x, y, z] = q.map(|v| v / n);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y), p[0]],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x), p[1]],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y), p[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// FR-157: every actuated joint starts at its home value, at rest — `PhysicsJointStateAPI:angular`
/// with the state equal to the drive target (unwrapped on a revolute joint, wrapped into
/// (−180°, 180°] on a continuous one; `:linear` in metres on a prismatic one) — and forward
/// kinematics at those states through the joint frames PhysX reads reproduces every authored link
/// pose: Play starts where the stage shows. A negative or skewed URDF axis keeps its sign and
/// direction: it is folded into both joint frames, so the states are the URDF's.
#[test]
fn fr_157_initial_joint_state() {
    let dir = tempfile::tempdir().unwrap();
    // A dresser-shaped arm: turned joint frames, X/Y/Z axes, offsets off the axes, a fixed root
    // joint and a continuous wrist; then a vendor-style tail: a joint about −Z, one about a skewed
    // axis and a slide along −Y.
    let urdf = dir.path().join("fk.urdf");
    let joint = |name: &str, kind: &str, parent: &str, child: &str, xyz: &str, rpy: &str, axis: &str| {
        format!(
            "  <joint name=\"{name}\" type=\"{kind}\">\n    <origin xyz=\"{xyz}\" rpy=\"{rpy}\"/>\n    <parent link=\"{parent}\"/>\n    <child link=\"{child}\"/>\n    <axis xyz=\"{axis}\"/>\n    <limit lower=\"-6\" upper=\"6\" effort=\"340282346638528859811704183484516925440\" velocity=\"2.5\"/>\n  </joint>\n"
        )
    };
    let mut s = String::from("<?xml version=\"1.0\"?>\n<robot name=\"fk\">\n");
    for link in [
        "World", "base", "l1", "l2", "l3", "l4", "l5", "l6", "l7", "l8",
    ] {
        s.push_str(&format!("  <link name=\"{link}\"/>\n"));
    }
    s.push_str(&joint(
        "root_joint",
        "fixed",
        "World",
        "base",
        "0 0 0",
        "0 0 0",
        "0 0 1",
    ));
    s.push_str(&joint(
        "j1",
        "revolute",
        "base",
        "l1",
        "0 0 0.75",
        "3.1415927 0 3.1415927",
        "0 0 1",
    ));
    s.push_str(&joint(
        "j2",
        "revolute",
        "l1",
        "l2",
        "-0.4 0 0",
        "3.1415927 0 3.1415927",
        "0 1 0",
    ));
    s.push_str(&joint(
        "j3", "revolute", "l2", "l3", "1.3 0 0", "0 0 0", "0 1 0",
    ));
    s.push_str(&joint(
        "j4",
        "revolute",
        "l3",
        "l4",
        "1.1 0 -0.05",
        "0 0 3.1415927",
        "1 0 0",
    ));
    s.push_str(&joint(
        "j5",
        "continuous",
        "l4",
        "l5",
        "0 0 0",
        "0 0 3.1415927",
        "0 1 0",
    ));
    s.push_str(&joint(
        "j6", "revolute", "l5", "l6", "0.2 0 0", "0.4 0 0", "0 0 -1",
    ));
    s.push_str(&joint(
        "j7",
        "revolute",
        "l6",
        "l7",
        "0 0.1 0",
        "0 0.3 0",
        "0.36 0.48 0.8",
    ));
    s.push_str(&joint(
        "j8",
        "prismatic",
        "l7",
        "l8",
        "0.1 0 0",
        "0 0 0",
        "0 -1 0",
    ));
    s.push_str("</robot>\n");
    std::fs::write(&urdf, s).unwrap();
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [2.0, -2.3, 0.6],
            orientation_deg: [30.0, 0.0, 0.0],
            home_pose_deg: Some(vec![-60.0, -100.0, 75.0, 190.0, 270.0, 30.0, -45.0, 0.3]),
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    assert!(validate_scene(&scene).is_empty());
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");

    let joints: Vec<UsdJoint> = usd_joints(&usda)
        .into_iter()
        .filter(|j| !j.body0.is_empty())
        .collect();
    assert_eq!(joints.len(), 9);
    let states: Vec<(&str, f64)> = joints
        .iter()
        .filter_map(|j| Some((j.name.as_str(), j.state?)))
        .collect();
    // 190° stays 190° on the revolute wrist; 270° wraps to −90° on the continuous joint; the
    // −Z, skewed and −Y joints start at the URDF's values, not mirrored; the slide in metres.
    assert_eq!(
        states,
        [
            ("j1", -60.0),
            ("j2", -100.0),
            ("j3", 75.0),
            ("j4", 190.0),
            ("j5", -90.0),
            ("j6", 30.0),
            ("j7", -45.0),
            ("j8", 0.3),
        ]
    );
    // +X/+Y/+Z axes keep their token and the URDF's frames; the others are folded (token X, a
    // body1 frame turning +X onto the axis).
    let tokens: Vec<(&str, &str, bool)> = joints
        .iter()
        .filter(|j| j.state.is_some())
        .map(|j| {
            (
                j.name.as_str(),
                j.axis.as_str(),
                j.local_rot1 != [1.0, 0.0, 0.0, 0.0],
            )
        })
        .collect();
    assert_eq!(
        tokens,
        [
            ("j1", "Z", false),
            ("j2", "Y", false),
            ("j3", "Y", false),
            ("j4", "X", false),
            ("j5", "Y", false),
            ("j6", "X", true),
            ("j7", "X", true),
            ("j8", "X", true),
        ]
    );
    // The slide: limits in metres as written, target and state 0.3 m.
    assert_eq!(joint_attr(&usda, "j8", "physics:lowerLimit"), Some(-6.0));
    assert_eq!(
        joint_attr_text(&usda, "j8", "drive:linear:physics:targetPosition"),
        Some("0.3")
    );
    for j in joints.iter().filter(|j| j.state.is_some() && !j.prismatic) {
        assert!(
            j.schemas.contains("\"PhysicsDriveAPI:angular\", \"PhysicsJointStateAPI:angular\""),
            "{}: {}",
            j.name,
            j.schemas
        );
        assert_starts_on_target(&usda, &j.name);
    }

    // FK from the root through every joint: parent · frame0(localPos0, localRot0) · motion(axis,
    // state) · frame1(localPos1, localRot1)⁻¹ must land on the authored child pose.
    let poses = usd_link_poses(&usda);
    let mut fk: std::collections::HashMap<String, M4> = std::collections::HashMap::new();
    fk.insert("world_link".into(), poses["world_link"]);
    for j in &joints {
        let parent = fk[&j.body0];
        let axis = match j.axis.as_str() {
            "X" => [1.0, 0.0, 0.0],
            "Y" => [0.0, 1.0, 0.0],
            _ => [0.0, 0.0, 1.0],
        };
        let state = j.state.unwrap_or(0.0);
        let motion = if j.prismatic {
            frame(axis.map(|a| a * state), [1.0, 0.0, 0.0, 0.0])
        } else {
            robo_fitting_room::math3::rotation_about_axis(axis, state.to_radians())
        };
        let frame1 = robo_fitting_room::math3::rigid_inverse(&frame(j.local_pos1, j.local_rot1));
        let child = mat_mul(
            &mat_mul(
                &mat_mul(&parent, &frame(j.local_pos0, j.local_rot0)),
                &motion,
            ),
            &frame1,
        );
        // Every pose and joint frame is written to 6 decimals (`quatf` frames: float precision),
        // two frames for a folded axis, so one joint from its authored parent lands within a few
        // 1e-6 of the authored child, and the chain FK of the nine joints within 1e-5.
        let authored = poses[&j.body1];
        let from_authored = mat_mul(
            &mat_mul(
                &mat_mul(&poses[&j.body0], &frame(j.local_pos0, j.local_rot0)),
                &motion,
            ),
            &frame1,
        );
        for r in 0..4 {
            for c in 0..4 {
                assert!(
                    (child[r][c] - authored[r][c]).abs() < 1e-5,
                    "{}: FK {child:?} != authored {authored:?}",
                    j.body1
                );
                assert!(
                    (from_authored[r][c] - authored[r][c]).abs() < 5e-6,
                    "{}: joint FK {from_authored:?} != authored {authored:?}",
                    j.body1
                );
            }
        }
        fk.insert(j.body1.clone(), child);
    }
    assert_eq!(fk.len(), 10);
    // And the authored poses are the URDF's own FK, its axes as signed: l6 turned +30° about its
    // joint's −Z, i.e. −30° about +Z of the joint frame.
    let m = robo_fitting_room::math3::multiply;
    let urdf_frame =
        |xyz: [f64; 3], rpy: [f64; 3]| robo_fitting_room::math3::from_translation_rpy(xyz, rpy);
    let l6 = m(
        &m(&poses["l5"], &urdf_frame([0.2, 0.0, 0.0], [0.4, 0.0, 0.0])),
        &robo_fitting_room::math3::rotation_about_axis([0.0, 0.0, 1.0], (-30f64).to_radians()),
    );
    for r in 0..4 {
        for c in 0..4 {
            assert!(
                (l6[r][c] - poses["l6"][r][c]).abs() < 1e-6,
                "l6: {l6:?} != {:?}",
                poses["l6"]
            );
        }
    }
}

/// A six_axis robot ([`write_six_axis_urdf`]) whose joints carry the dresser's placeholder effort
/// (FLT_MAX) and a velocity limit of 2.5 rad/s.
fn write_six_axis_placeholder_urdf(urdf_path: &Path) {
    write_six_axis_urdf(urdf_path);
    let s = std::fs::read_to_string(urdf_path).unwrap().replace(
        "effort=\"100\" velocity=\"1\"",
        "effort=\"340282346638528859811704183484516925440\" velocity=\"2.5\"",
    );
    std::fs::write(urdf_path, s).unwrap();
}

/// FR-158: every drive's damping is ζ = 0.7 on the composite inertia it moves at home; its force
/// cap is the URDF effort, or — for the placeholder effort — the derived cap
/// max(1, 2·τ_hold + I·v/0.25 s + D·v); its velocity limit the URDF's, in deg/s.
#[test]
fn fr_158_drive_limits() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_placeholder_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");

    // By hand. The six_axis links are stacked 0.5 m apart on Z with Z axes, their centres of mass
    // at their origins (their URDF inertials, SIX_AXIS_LINKS: link_1..6 300/200/100/40/20/10 kg,
    // Izz 6/2/1/0.8/0.3/0.1 kg·m²), so the stowed pose (turns about Z) keeps every centre on
    // every axis: joint_a_k moves I = Σ_{j≥k} Izz_j, and gravity's torque is at most
    // g·Σ_{j≥k} m_j·0.5·(j − k).
    let mass: Vec<f64> = SIX_AXIS_LINKS[1..].iter().map(|(_, m, _)| *m).collect();
    let izz: Vec<f64> = SIX_AXIS_LINKS[1..].iter().map(|(_, _, i)| i[2]).collect();
    let k_rad = 1.5e5 * 180.0 / std::f64::consts::PI;
    let v = 2.5;
    for k in 0..6 {
        let joint = format!("joint_{}", k + 1);
        let inertia: f64 = izz[k..].iter().sum();
        let holding: f64 = 9.81 * (k..6).map(|j| mass[j] * 0.5 * (j - k) as f64).sum::<f64>();
        let d_rad = 2.0 * 0.7 * (k_rad * inertia).sqrt();
        let damping = joint_attr(&usda, &joint, "drive:angular:physics:damping").unwrap();
        assert!(
            (damping - d_rad * std::f64::consts::PI / 180.0).abs() < 1e-5,
            "{joint}: damping {damping}"
        );
        let cap = 2.0 * holding + inertia * v / 0.25 + d_rad * v;
        let max_force = joint_attr(&usda, &joint, "drive:angular:physics:maxForce").unwrap();
        assert!((max_force - cap).abs() < 1e-5, "{joint}: maxForce {max_force} != {cap}");
        assert_eq!(
            joint_attr_text(&usda, &joint, "physxJoint:maxJointVelocity"),
            Some("143.239449"),
            "{joint}"
        );
        let schemas = format!(
            "def PhysicsRevoluteJoint \"{joint}\" (\n            prepend apiSchemas = [\"PhysicsDriveAPI:angular\", \"PhysicsJointStateAPI:angular\", \"PhysxJointAPI\"]\n"
        );
        assert!(usda.contains(&schemas), "{joint}");
    }
    // joint_1's cap by number: 2·9.81·325 + 10.2·v/0.25 + D·v ≈ 6376.5 + 102 + 13107.96·v.
    let cap = joint_attr(&usda, "joint_1", "drive:angular:physics:maxForce").unwrap();
    assert!((cap - 39248.4).abs() < 0.1, "{cap}");
    assert!(!usda.contains("3.4028234663852886e38"));

    // A real effort is the cap (compose_one_six_axis_robot: effort 100, velocity 1 rad/s).
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    for n in 1..=6 {
        let joint = format!("joint_{n}");
        assert_eq!(joint_attr_text(&usda, &joint, "drive:angular:physics:maxForce"), Some("100"));
        assert_eq!(
            joint_attr_text(&usda, &joint, "physxJoint:maxJointVelocity"),
            Some("57.29578")
        );
    }
}

/// FR-159: a robot's tool payload is merged into the link carrying its tool collisions (or the
/// link `payload.link` names) — spread over their hulls at uniform density, or at an explicit
/// centre of mass and inertia — by the parallel-axis theorem, with principal axes; the drives move
/// it; bad payloads (and a link the URDF hasn't got) are rejected.
#[test]
fn fr_159_tool_payload() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("robot/arm.urdf");
    std::fs::create_dir_all(urdf.parent().unwrap()).unwrap();
    write_six_axis_urdf(&urdf);
    // A 0.4 × 0.2 × 0.6 m tool box hull on link_6, centred at (−0.5, 0, 0.2).
    let mut obj = String::new();
    for i in 0..8 {
        let s = |bit: usize, h: f64| if i & (1 << bit) == 0 { -h } else { h };
        obj.push_str(&format!("v {} {} {}\n", s(0, 0.2), s(1, 0.1), s(2, 0.3)));
    }
    for [a, b, c, d] in [[1, 3, 4, 2], [5, 6, 8, 7], [1, 2, 6, 5], [3, 7, 8, 4], [1, 5, 7, 3], [2, 4, 8, 6]] {
        obj.push_str(&format!("f {a} {b} {c} {d}\n"));
    }
    std::fs::create_dir_all(dir.path().join("robot/meshes/tool")).unwrap();
    std::fs::write(dir.path().join("robot/meshes/tool/box.obj"), obj).unwrap();
    let s = std::fs::read_to_string(&urdf).unwrap().replace(
        "  <link name=\"arm_link_6\">\n",
        "  <link name=\"arm_link_6\">\n    <collision>\n      <origin xyz=\"-0.5 0 0.2\" rpy=\"0 0 0\"/>\n      <geometry>\n        <mesh filename=\"./meshes/tool/box.obj\"/>\n      </geometry>\n    </collision>\n",
    );
    std::fs::write(&urdf, s).unwrap();
    let manifest = dir.path().join("cell.yaml");
    let compose = |payload: &str| -> String {
        std::fs::write(
            &manifest,
            format!(
                "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"robot/arm.urdf\"\n    position: {{ x: 0.0, y: 0.0, z: 0.0 }}\n    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}\n    payload: {payload}\n"
            ),
        )
        .unwrap();
        let scene = parse_scene(&manifest).unwrap();
        assert!(validate_scene(&scene).is_empty());
        let out = dir.path().join("cell.zip");
        compose_stage(&scene, &manifest, &out).unwrap();
        read_zip_entry(&out, "cell.usda")
    };
    let link_6 = |usda: &str| -> String {
        let start = usda.find("        def Xform \"arm_link_6\" (").unwrap();
        let block = &usda[start..];
        block[..block.find("def Mesh").unwrap()].to_string()
    };
    let numbers = |block: &str, key: &str| -> Vec<f64> {
        let line = block.lines().find(|l| l.contains(key)).unwrap_or_else(|| panic!("no {key}"));
        line[line.find('(').unwrap() + 1..line.rfind(')').unwrap()]
            .split(',')
            .map(|v| v.trim().parse().unwrap())
            .collect()
    };
    // The tensor the emitted principal moments and axes make: A·diag(I)·Aᵀ.
    let tensor = |block: &str| -> [[f64; 3]; 3] {
        let d = numbers(block, "physics:diagonalInertia");
        let q = if block.contains("principalAxes") {
            numbers(block, "physics:principalAxes")
        } else {
            vec![1.0, 0.0, 0.0, 0.0]
        };
        let a = frame([0.0; 3], [q[0], q[1], q[2], q[3]]);
        let mut t = [[0.0; 3]; 3];
        for (i, row) in t.iter_mut().enumerate() {
            for (j, entry) in row.iter_mut().enumerate() {
                *entry = (0..3).map(|k| a[i][k] * d[k] * a[j][k]).sum();
            }
        }
        t
    };
    // 5e-5 kg·m²: the moments and the axes' quaternion are written to 6 decimals.
    let assert_tensor = |got: [[f64; 3]; 3], want: [[f64; 3]; 3]| {
        for i in 0..3 {
            for j in 0..3 {
                assert!((got[i][j] - want[i][j]).abs() < 5e-5, "{got:?} != {want:?}");
            }
        }
    };

    // Spread over the box: 100 kg at (−0.5, 0, 0.2) with the box's m·(b² + c²)/12, … plus
    // arm_link_6's own URDF inertial, 10 kg with diag(0.2, 0.2, 0.1) at its origin, both moved to
    // the common centre of mass. The tool box isn't weighed at the link density on top.
    let usda = compose("{ mass_kg: 100 }");
    // Naming the carrier (`link`) that the tool meshes already mark changes nothing.
    assert_eq!(compose("{ mass_kg: 100, link: arm_link_6 }"), usda);
    let block = link_6(&usda);
    assert!(block.contains("float physics:mass = 110\n"), "{block}");
    let com = [-50.0 / 110.0, 0.0, 20.0 / 110.0];
    for (g, w) in numbers(&block, "physics:centerOfMass").iter().zip(com) {
        assert!((g - w).abs() < 1e-6, "{block}");
    }
    let parallel = |m: f64, d: [f64; 3]| -> [[f64; 3]; 3] {
        let d2: f64 = d.iter().map(|v| v * v).sum();
        let mut t = [[0.0; 3]; 3];
        for (i, row) in t.iter_mut().enumerate() {
            for (j, entry) in row.iter_mut().enumerate() {
                *entry = m * (if i == j { d2 } else { 0.0 } - d[i] * d[j]);
            }
        }
        t
    };
    let (box_xx, box_yy, box_zz) = (
        100.0 * (0.2f64 * 0.2 + 0.6 * 0.6) / 12.0,
        100.0 * (0.4f64 * 0.4 + 0.6 * 0.6) / 12.0,
        100.0 * (0.4f64 * 0.4 + 0.2 * 0.2) / 12.0,
    );
    let link_shift = parallel(10.0, [-com[0], -com[1], -com[2]]);
    let box_shift = parallel(100.0, [-0.5 - com[0], -com[1], 0.2 - com[2]]);
    let own = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.1]];
    let solid = [[box_xx, 0.0, 0.0], [0.0, box_yy, 0.0], [0.0, 0.0, box_zz]];
    let mut want = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            want[i][j] = own[i][j] + link_shift[i][j] + solid[i][j] + box_shift[i][j];
        }
    }
    // Off the X and Z axes: an XZ product (0.909 kg·m²), so the principal axes turn about Y.
    assert!(want[0][2].abs() > 0.5);
    assert!(block.contains("quatf physics:principalAxes = ("), "{block}");
    assert!(!block.contains("quatf physics:principalAxes = (1, 0, 0, 0)"), "{block}");
    assert_tensor(tensor(&block), want);
    // joint_6 (about link_6's Z) moves the merged link: Izz + m·|d⊥|², d⊥ = (com_x, com_y).
    let i_a6 = want[2][2] + 110.0 * (com[0] * com[0] + com[1] * com[1]);
    let d_a6 = 2.0 * 0.7 * (1.5e5 * 180.0 / std::f64::consts::PI * i_a6).sqrt() * std::f64::consts::PI / 180.0;
    let damping = joint_attr(&usda, "joint_6", "drive:angular:physics:damping").unwrap();
    assert!((damping - d_a6).abs() < 1e-4, "{damping} != {d_a6}");
    // Only arm_link_6 carries it: every other body is on its own axes, whose principal axes are
    // authored as the identity (with every diagonal inertia).
    let bodies = usda.matches("PhysicsRigidBodyAPI").count();
    assert_eq!(usda.matches("quatf physics:principalAxes = ").count(), bodies);
    assert_eq!(usda.matches("quatf physics:principalAxes = (1, 0, 0, 0)\n").count(), bodies - 1);
    assert!(usda.contains("float physics:mass = 200\n"), "arm_link_2 keeps its own");

    // An explicit centre of mass and inertia: on arm_link_6's Z axis, so the sum stays diagonal —
    // diag(0.2 + 1 + δ, 0.2 + 2 + δ, 0.1 + 3), δ = 10·100/110·0.2².
    let usda = compose(
        "{ mass_kg: 100, center_of_mass_m: { x: 0, y: 0, z: 0.2 }, inertia_kgm2: { ixx: 1, iyy: 2, izz: 3 } }",
    );
    let block = link_6(&usda);
    assert!(block.contains("float physics:mass = 110\n"), "{block}");
    let delta = 10.0 * 100.0 / 110.0 * 0.04;
    let moments = numbers(&block, "physics:diagonalInertia");
    for (g, w) in moments.iter().zip([1.2 + delta, 2.2 + delta, 3.1]) {
        assert!((g - w).abs() < 1e-6, "{block}");
    }
    assert!(block.contains("quatf physics:principalAxes = (1, 0, 0, 0)\n"), "{block}");

    // Rejected: a non-positive mass, a non-finite centre, impossible moments.
    for (payload, problem) in [
        ("{ mass_kg: 0 }", "payload mass_kg must be finite and > 0"),
        ("{ mass_kg: -3 }", "payload mass_kg must be finite and > 0"),
        (
            "{ mass_kg: 5, center_of_mass_m: { x: .nan, y: 0, z: 0 } }",
            "payload center_of_mass_m must be finite",
        ),
        (
            "{ mass_kg: 5, inertia_kgm2: { ixx: 1, iyy: 1, izz: 0 } }",
            "payload inertia_kgm2 must be finite and > 0",
        ),
        (
            "{ mass_kg: 5, inertia_kgm2: { ixx: 1, iyy: 1, izz: 3 } }",
            "at most the sum of the other two",
        ),
        (
            "{ mass_kg: 5, link: no_such_link }",
            "payload link 'no_such_link' is no link of its URDF",
        ),
    ] {
        std::fs::write(
            &manifest,
            format!(
                "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"robot/arm.urdf\"\n    position: {{ x: 0.0, y: 0.0, z: 0.0 }}\n    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}\n    payload: {payload}\n"
            ),
        )
        .unwrap();
        let errors: Vec<String> = validate_scene(&parse_scene(&manifest).unwrap())
            .iter()
            .map(|e| e.to_string())
            .collect();
        assert!(errors.len() == 1 && errors[0].contains(problem), "{payload}: {errors:?}");
    }
    // mass_kg is required.
    std::fs::write(
        &manifest,
        "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"robot/arm.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n    payload: { center_of_mass_m: { x: 0, y: 0, z: 0 } }\n",
    )
    .unwrap();
    assert!(parse_scene(&manifest).unwrap_err().to_string().contains("mass_kg"));
}

// =============================================================================
// 2.7 Collision Reporting
// =============================================================================

/// FR-160: every collision-enabled link emits an IsaacReadContactSensor +
/// ROS2PublishContactSensor pair inside the existing per-robot ActionGraph.
#[test]
fn fr_160_contact_sensor_nodes() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // The six_axis URDF has 8 links (World + base + link_1..6), each with one
    // <collision>. World sanitises to "world_link"; the rest pass through.
    let sensor_count = usda
        .matches("\"isaacsim.sensors.physics.IsaacReadContactSensor\"")
        .count();
    // The publisher is the generic `ROS2Publisher` (not the
    // `ROS2PublishContactSensor` node type, which doesn't exist in
    // Isaac Sim 5.1). The matching emission flag is the
    // `messageName = "Bool"` line — count those.
    let publisher_count = usda.matches("inputs:messageName = \"Bool\"").count();
    assert_eq!(sensor_count, 8, "expected 8 contact sensors (one per collision link)");
    assert_eq!(publisher_count, 8, "expected 8 ROS2Publisher Bool emissions (one per sensor)");
    // Per-link IsaacContactSensor prim must be emitted with `enabled = 1`
    // — without it Isaac drops events with `no valid sensor reading`.
    assert_eq!(
        usda.matches("def IsaacContactSensor \"ContactSensor_node\"").count(),
        8,
        "expected 8 IsaacContactSensor prims (one per collision link)",
    );

    // Sensors live inside the same per-robot graph as motion (ActionGraph_R1),
    // not in a separate ActionGraph_contacts_*. Verify by absence.
    assert!(
        !usda.contains("ActionGraph_contacts_"),
        "contact sensors must inline into the existing per-robot graph",
    );
    assert!(usda.contains("def OmniGraph \"ActionGraph_R1\""));

    // Each ContactSensor's csPrim rel resolves to the per-link
    // IsaacContactSensor USD prim emitted as a child of the link Xform.
    // Pointing at the link itself triggers Isaac's "no valid sensor
    // reading, is the sensor enabled?" warning at runtime.
    assert!(
        usda.contains("add rel inputs:csPrim = </World/robots/R1/world_link/ContactSensor_node>"),
        "csPrim rel must point at the per-link IsaacContactSensor child prim",
    );
    assert!(usda.contains("def Xform \"world_link\""));
    // The IsaacContactSensor prim itself must be present and enabled.
    assert!(
        usda.contains("def IsaacContactSensor \"ContactSensor_node\""),
        "every collision-bearing link must emit a child IsaacContactSensor prim",
    );
    assert!(usda.contains("bool enabled = 1"));
}

/// FR-161: silence guarantee at compose time. The ROS2PublishContactSensor's
/// execIn is wired to the matching IsaacReadContactSensor's execOut, NOT the
/// OnPlaybackTick — so the publisher only fires when the sensor reports a
/// contact event, never per frame. The namespace on every publisher is the
/// two-segment <cell_id>/<robot_id> contract from the topic schema.
#[test]
fn fr_161_contact_ros2_topic() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());

    // Find every PublishContact_<link> block and check its execIn target.
    let mut checked_at_least_one = false;
    let lines: Vec<&str> = usda.lines().collect();
    for (i, line) in lines.iter().enumerate() {
        if let Some(rest) = line.trim().strip_prefix("def OmniGraphNode \"PublishContact_") {
            let link = rest.trim_end_matches('"');
            // Scan forward for the execIn.connect line (within ~30 lines).
            let exec_in = lines[i..i.saturating_add(30).min(lines.len())]
                .iter()
                .find(|l| l.contains("inputs:execIn.connect"))
                .copied()
                .unwrap_or("<missing>");
            assert!(
                exec_in.contains(&format!("ContactSensor_{}.outputs:execOut", link)),
                "PublishContact_{} must wire execIn to its sensor's execOut, got: {}",
                link, exec_in,
            );
            assert!(
                !exec_in.contains("OnPlaybackTick.outputs:tick"),
                "PublishContact_{} must NOT tie execIn to OnPlaybackTick (would break the silence guarantee), got: {}",
                link, exec_in,
            );
            // Namespace shape: <cell_id>/<robot_id> for this fixture.
            let ns = lines[i..i.saturating_add(30).min(lines.len())]
                .iter()
                .find(|l| l.contains("inputs:nodeNamespace"))
                .copied()
                .unwrap_or("<missing>");
            assert!(
                ns.contains("\"t/R1\""),
                "PublishContact_{} must carry namespace t/R1, got: {}",
                link, ns,
            );
            checked_at_least_one = true;
        }
    }
    assert!(checked_at_least_one, "no PublishContact_* nodes found");
}

/// FR-162: contact-sensor wiring is per-robot and URDF-derived. Adding a
/// second robot (same URDF) must double the contact-sensor count — no
/// manifest knob, no manual config.
#[test]
fn fr_162_automatic_contact_wiring() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();

    let one_robot = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf.clone(),
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out_one = dir.path().join("one.zip");
    compose_stage_with_options(&one_robot, &manifest, &out_one, &ComposeOptions::default()).unwrap();
    let usda_one = read_zip_entry(&out_one, "scene.usda");
    let count_one = usda_one
        .matches("\"isaacsim.sensors.physics.IsaacReadContactSensor\"")
        .count();
    assert!(count_one > 0, "1-robot scene must produce contact sensors");

    let two_robots = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![
            RobotPlacement {
                robot_id: "R1".into(),
                dressed_urdf: urdf.clone(),
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            },
            RobotPlacement {
                robot_id: "R2".into(),
                dressed_urdf: urdf,
                position: [5.0, 0.0, 0.0],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            },
        ],
        fixtures: vec![],
        floor_plan: None,
    };
    let out_two = dir.path().join("two.zip");
    compose_stage_with_options(&two_robots, &manifest, &out_two, &ComposeOptions::default()).unwrap();
    let usda_two = read_zip_entry(&out_two, "scene.usda");
    let count_two = usda_two
        .matches("\"isaacsim.sensors.physics.IsaacReadContactSensor\"")
        .count();
    assert_eq!(count_two, count_one * 2, "adding a robot must double the contact-sensor count");
}

/// FR-163: USD collision geometry uses the dresser-generated convex hulls
#[test]
fn fr_163_dresser_collision_hulls() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    // Convex hull approximation drives PhysX contact generation off the
    // collision meshes from the URDF's <collision> elements.
    assert!(usda.contains("uniform token physics:approximation = \"convexHull\""));
    assert!(usda.contains("PhysicsMeshCollisionAPI"));
}

/// Wire-contract guarantee: every emitted `inputs:nodeNamespace` and
/// `inputs:topicName` is a string of slash-separated tokens, where each
/// token matches the rmw regex `[A-Za-z_][A-Za-z0-9_]*`. This is the
/// shape both FastDDS (Isaac default) AND `rmw_cyclonedds_cpp` accept,
/// so users running Isaac Sim with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
/// see no validation errors at bridge bring-up.
#[test]
fn ros2_topics_are_cyclone_dds_compatible() {
    // Use a digit-prefixed manifest name + robot_id so we exercise the
    // r_-prefix sanitisation in both segments — the most demanding case.
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"CELL01\"\n").unwrap();

    let scene = SceneDefinition {
        name: "2025-line-A".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "1234R02".into(),
            dressed_urdf: urdf,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();
    let usda = read_zip_entry(&out, "scene.usda");

    // rmw token: leading [A-Za-z_], rest [A-Za-z0-9_].
    fn rmw_token_ok(tok: &str) -> bool {
        let mut chars = tok.chars();
        let Some(first) = chars.next() else { return false };
        if !(first.is_ascii_alphabetic() || first == '_') {
            return false;
        }
        chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
    }

    let mut checked = 0usize;
    for line in usda.lines() {
        let line = line.trim();
        for (key, label) in [
            ("custom string inputs:nodeNamespace = \"", "namespace"),
            ("custom string inputs:topicName = \"", "topic"),
        ] {
            if let Some(rest) = line.strip_prefix(key) {
                let value = rest.trim_end_matches('"');
                assert!(!value.is_empty(), "empty {} string", label);
                assert!(!value.starts_with('/'), "{} must NOT begin with `/`: {:?}", label, value);
                assert!(!value.ends_with('/'), "{} must NOT end with `/`: {:?}", label, value);
                assert!(!value.contains("//"), "{} must NOT contain `//`: {:?}", label, value);
                for tok in value.split('/') {
                    assert!(
                        rmw_token_ok(tok),
                        "{} segment {:?} (in {:?}) violates rmw token regex \
                         [A-Za-z_][A-Za-z0-9_]* — would be rejected by \
                         rmw_validate_namespace / rmw_validate_topic_name under \
                         rmw_cyclonedds_cpp",
                        label, tok, value,
                    );
                }
                checked += 1;
            }
        }
    }
    assert!(checked >= 4, "expected at least 4 namespace/topic emissions, got {}", checked);
}

#[test]
fn fr_164_monitor_subcommand() {
    todo!("Monitor subcommand not yet implemented");
}
#[test]
fn fr_165_monitor_topic_derivation() {
    todo!("Monitor topic derivation not yet implemented");
}
#[test]
fn fr_166_monitor_exit_code() {
    todo!("Monitor exit code not yet implemented");
}
#[test]
fn fr_167_monitor_persistence() {
    todo!("Monitor persistence not yet implemented");
}
#[test]
fn fr_168_force_threshold() {
    todo!("Force threshold filter not yet implemented");
}
#[test]
fn fr_169_event_debouncing() {
    todo!("Event debouncing not yet implemented");
}

// =============================================================================
// 2.8 USD Deploy  (NOT STARTED)
// =============================================================================

#[test]
fn fr_170_streaming_client_connect() {
    todo!("Streaming client connection not yet implemented");
}
#[test]
fn fr_171_stage_deploy() {
    todo!("Stage deploy not yet implemented");
}
#[test]
fn fr_172_live_reload() {
    todo!("Live stage reload not yet implemented");
}
#[test]
fn fr_173_deploy_status() {
    todo!("Deploy status reporting not yet implemented");
}
#[test]
fn fr_174_asset_path_resolution() {
    todo!("Asset path resolution not yet implemented");
}
#[test]
fn fr_175_connectivity_check() {
    todo!("Pre-deploy connectivity validation not yet implemented");
}
#[test]
fn fr_176_ros_domain_id() {
    todo!("ROS_DOMAIN_ID matching not yet implemented");
}
#[test]
fn fr_177_extension_verification() {
    todo!("Target extension verification not yet implemented");
}

// =============================================================================
// 2.9 Server Mode  (subset DONE/PARTIAL behind --features server)
// =============================================================================
//
// FR-180/181/182/184/204 happy-path coverage lives in `tests/server.rs`,
// gated behind the `server` Cargo feature. Run those with:
//   cargo test --features server --test server
// The placeholders below remain so REQUIREMENTS.md → tests mapping stays
// 1:1; default-feature builds report them as `todo!()` until the server
// feature is enabled.

/// FR-180: in-binary HTTP server behind `server` feature → covered by
/// `tests/server.rs::list_robots_returns_seeded_entry` and friends.
#[test]
fn fr_180_server_startup() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled — run `cargo test --features server --test server`");
    }
}

/// FR-181: `POST /api/compose` returns a streaming zip → covered by
/// `tests/server.rs::compose_returns_zip`.
#[test]
fn fr_181_api_compose() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-182: `POST /api/validate` returns structured diagnostics →
/// `tests/server.rs::validate_round_trips_yaml`.
#[test]
fn fr_182_api_validate() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

#[test]
fn fr_183_api_monitor() {
    todo!("API monitor (SSE collision stream) — BL-17");
}

/// FR-184: zip stream covers the binary-download path → see FR-181 test.
/// Stage-ID registry is BL-19.
#[test]
fn fr_184_api_get_scene() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

#[test]
fn fr_185_api_deploy() {
    todo!("API deploy — BL-20");
}
#[test]
fn fr_186_api_swap() {
    todo!("API swap — BL-03");
}
#[test]
fn fr_187_api_status() {
    todo!("API status — depends on FR-185");
}
/// FR-188: `POST /api/compose-urdf-bundle` streams the URDF-source bundle zip →
/// covered by `tests/server.rs::compose_urdf_bundle_returns_zip`.
#[test]
fn fr_188_api_compose_urdf_bundle() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled — run `cargo test --features server --test server`");
    }
}
#[test]
fn fr_190_sse_broadcast_fanout() {
    todo!("SSE broadcast — BL-17");
}
#[test]
fn fr_191_event_db_sink() {
    todo!("Event DB sink — BL-18");
}

// =============================================================================
// 2.10 2D Scene-Layout Editor
// =============================================================================
//
// The editor SPA lives in `web/`, embedded into the binary via
// `include_dir!`. End-to-end coverage is the manual workflow in the README
// (load YAML → drag → save → compose → open in Isaac); the server-side
// supporting endpoints are covered by `tests/server.rs`.

/// FR-200: top-down 2D layout with drag-drop. Browser-side; manual smoke.
#[test]
fn fr_200_2d_layout_canvas() {
    // Compile-time evidence: when the `server` feature is on, the
    // embedded SPA bundle compiles in via include_dir!.
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled — editor SPA only ships with --features server");
    }
}

/// FR-201: floor-plan upload-OBJ endpoint → covered by
/// `tests/server.rs::upload_obj_round_trips_and_appears_in_listing`.
#[test]
fn fr_201_floor_plan_upload() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-202: form-based inspector. Browser-side; manual smoke. Server-side
/// `/api/urdf/joints` (used by the inspector) is tested in
/// `tests/server.rs::urdf_joints_returns_actuated_count`.
#[test]
fn fr_202_form_based_inspector() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-203: the Generate-USDA modal exposes an optional "Also download URDF
/// source bundle" checkbox that triggers a second `/api/compose-urdf-bundle`
/// download. Browser-side; the server endpoint it calls is covered by
/// `tests/server.rs::compose_urdf_bundle_returns_zip`.
#[test]
fn fr_203_editor_urdf_bundle_checkbox() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled — editor SPA only ships with --features server");
    }
}

/// FR-204: serve subcommand boots both the API and the embedded SPA.
#[test]
fn fr_204_in_binary_serve() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-205: OpenAPI spec + Swagger UI shipped behind the `server` feature.
/// Wire-level coverage lives in tests/server.rs (or is reachable via curl);
/// here we just gate on the feature so the requirement is auditable.
#[test]
fn fr_205_openapi_docs() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-206: scene save/load round-trip. Wire-level test:
/// `tests/server.rs::save_then_load_round_trips`.
#[test]
fn fr_206_api_scenes_save_load() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-207: path-traversal sandboxing. Wire-level test:
/// `tests/server.rs::path_traversal_is_blocked`.
#[test]
fn fr_207_asset_path_sandbox() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-208: actuated-joint listing endpoint. Wire-level test:
/// `tests/server.rs::urdf_joints_returns_actuated_count`.
#[test]
fn fr_208_api_urdf_joints() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-209: `cell_origin_dxf` derivation in floor-plan parsing. When the
/// manifest sets only `cell_origin_dxf` (no explicit `offset`),
/// parse_scene must derive `offset = -cell_origin_dxf` so the chosen DXF
/// point lands at the world origin.
#[test]
fn fr_209_cell_origin_dxf_derivation() {
    let dir = tempfile::tempdir().unwrap();
    let obj_path = dir.path().join("plan.obj");
    // Single segment from (100, 50, 0) to (101, 50, 0).
    std::fs::write(
        &obj_path,
        "v 100.0 50.0 0.0\nv 101.0 50.0 0.0\nl 1 2\n",
    )
    .unwrap();

    let scene_path = dir.path().join("cell.yaml");
    std::fs::write(
        &scene_path,
        "name: \"fp\"\nfloor_plan:\n  geometry_file: \"plan.obj\"\n  cell_origin_dxf: { x: 100.0, y: 50.0, z: 0.0 }\n",
    )
    .unwrap();

    let scene = parse_scene(&scene_path).unwrap();
    let fp = scene.floor_plan.expect("floor plan parsed");
    // After derivation the (100, 50) DXF point lands at the world origin.
    assert!((fp.segments[0][0][0] - 0.0).abs() < 1e-6);
    assert!((fp.segments[0][0][1] - 0.0).abs() < 1e-6);
    assert!((fp.segments[0][1][0] - 1.0).abs() < 1e-6);
    assert!((fp.segments[0][1][1] - 0.0).abs() < 1e-6);
}

/// FR-210: liveness + readiness probes at the root. Wire-level test:
/// `tests/server.rs::healthz_and_readyz_are_served`.
#[test]
fn fr_210_health_readiness_probes() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-211: typed `ErrorEnvelope` body on every 4xx/5xx response. Wire-level
/// test: `tests/server.rs::error_envelope_is_typed`.
#[test]
fn fr_211_error_envelope_is_typed() {
    #[cfg(feature = "server")]
    {
        return;
    }
    #[allow(unreachable_code)]
    {
        todo!("Server feature not enabled");
    }
}

/// FR-212: every emitted ROS 2 namespace + topic passes the rmw token regex,
/// guaranteeing the composed USDA works under both FastDDS and Cyclone DDS.
/// Asserted by `ros2_topics_are_cyclone_dds_compatible` above.
#[test]
fn fr_212_cyclone_dds_compatible_topics() {
    // The exhaustive assertion lives in `ros2_topics_are_cyclone_dds_compatible`
    // — this entry just keeps the FR ↔ test mapping 1:1 for auditability.
}

/// FR-213: cell-level `/<cell_id>/contacts/v1` topic with enriched
/// `cell_sim_contact_msgs/msg/CollisionEvent` records. Implemented by the
/// aggregator pod (BL-24) — separate from the composer.
#[test]
fn fr_213_cell_collision_event_topic() {
    todo!("cell-level CollisionEvent aggregator — BL-24");
}

/// FR-214: collision-event message schema versioning policy
/// (`schema_version` field, versioned topic `/v<n>`, IDL-hash snapshot).
#[test]
fn fr_214_collision_event_schema_versioning() {
    todo!("collision-event schema versioning policy — BL-24");
}

// =============================================================================
// 3. Non-Functional Requirements
// =============================================================================

/// NFR-100: Pure-Rust pipeline — no native USD/PhysX deps
#[test]
fn nfr_100_pure_rust_pipeline() {
    // Pure-Rust ASCII USDA emission: a successful compose without any C++/cmake
    // bindings is itself the proof.
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.starts_with("#usda 1.0"));
}

#[test]
fn nfr_101_composition_performance() {
    todo!("Performance benchmark — needs criterion harness");
}
#[test]
fn nfr_102_swap_performance() {
    todo!("Swap performance benchmark — backlog");
}

/// NFR-103: Compose runs on Linux / macOS / Windows
#[test]
fn nfr_103_platform_support() {
    assert!(cfg!(target_os = "macos") || cfg!(target_os = "linux") || cfg!(target_os = "windows"));
    // Sanity: compose actually runs on whichever platform is hosting the tests.
    let _ = compose_one_six_axis_robot(ComposeOptions::default());
}

/// NFR-104: validate_scene rejects non-URDF 3D inputs
#[test]
fn nfr_104_urdf_only_input() {
    let dir = tempfile::tempdir().unwrap();
    let stl = dir.path().join("not-a-urdf.stl");
    std::fs::write(&stl, b"solid foo\nendsolid\n").unwrap();
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: stl,
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let errors = validate_scene(&scene);
    assert!(
        !errors.is_empty(),
        "non-URDF 3D file must be rejected at validate time"
    );
}

/// NFR-105: Deterministic USD output — identical manifests → byte-identical USDA
#[test]
fn nfr_105_deterministic_output() {
    let (_z1, u1, _d1) = compose_one_six_axis_robot(ComposeOptions::default());
    let (_z2, u2, _d2) = compose_one_six_axis_robot(ComposeOptions::default());
    assert_eq!(u1, u2);
}

/// NFR-106: Compose is read-only on source URDFs and meshes
#[test]
fn nfr_106_read_only_inputs() {
    let dir = tempfile::tempdir().unwrap();
    let urdf = dir.path().join("arm.urdf");
    write_six_axis_urdf(&urdf);
    let mesh = dir.path().join("meshes/robot/m.obj");
    let urdf_before = std::fs::read(&urdf).unwrap();
    let mesh_before = std::fs::read(&mesh).unwrap();
    let urdf_mtime_before = std::fs::metadata(&urdf).unwrap().modified().unwrap();
    let mesh_mtime_before = std::fs::metadata(&mesh).unwrap().modified().unwrap();

    let manifest = dir.path().join("scene.yaml");
    std::fs::write(&manifest, "name: \"t\"\n").unwrap();
    let scene = SceneDefinition {
        name: "t".into(),
        description: String::new(),
        robots: vec![RobotPlacement {
            robot_id: "R1".into(),
            dressed_urdf: urdf.clone(),
            position: [0.0; 3],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }],
        fixtures: vec![],
        floor_plan: None,
    };
    let out = dir.path().join("scene.zip");
    compose_stage(&scene, &manifest, &out).unwrap();

    assert_eq!(std::fs::read(&urdf).unwrap(), urdf_before);
    assert_eq!(std::fs::read(&mesh).unwrap(), mesh_before);
    assert_eq!(
        std::fs::metadata(&urdf).unwrap().modified().unwrap(),
        urdf_mtime_before
    );
    assert_eq!(
        std::fs::metadata(&mesh).unwrap().modified().unwrap(),
        mesh_mtime_before
    );
}

#[test]
fn nfr_107_port_error_messages() {
    todo!("Port unreachable error messages — depends on FR-170/175");
}

/// The attribute lines (trimmed) of the prim whose head line is `head`, up to its first
/// nested prim or its end.
fn prim_lines<'a>(usda: &'a str, head: &str) -> Vec<&'a str> {
    let start = usda.find(head).unwrap_or_else(|| panic!("no {head}"));
    usda[start + head.len()..]
        .lines()
        .map(str::trim)
        .skip_while(|l| *l != "{")
        .skip(1)
        .take_while(|l| *l != "}" && !l.starts_with("def "))
        .filter(|l| !l.is_empty())
        .collect()
}

/// NFR-108: the USDA's layout follows NVIDIA's flat-scene template (header, /World, an inline
/// /Environment, /Render), lit by a replica of Kit's Default light rig over a matte ground with
/// its collider, with no FlatGrid or other network asset, and leaves the render settings and mode
/// to Isaac Sim (its quality default).
#[test]
fn nfr_108_flat_scene_template_layout() {
    let (_zip, usda, _dir) = compose_one_six_axis_robot(ComposeOptions::default());
    assert!(usda.starts_with("#usda 1.0\n"));
    assert!(usda.contains("customLayerData"));
    // /World, then /Environment, then /Render, the last prim.
    let order = ["\ndef Xform \"World\" (", "\ndef Xform \"Environment\"\n", "\ndef \"Render\" ("];
    let at: Vec<usize> = order.iter().map(|h| usda.find(h).unwrap()).collect();
    assert!(at.windows(2).all(|p| p[0] < p[1]), "{at:?}");
    assert!(usda.contains("def PhysicsScene \"PhysicsScene\""));
    assert!(usda.trim_end().ends_with("uniform token sourceType = \"raw\"\n        }\n    }\n}"));
    // No FlatGrid, its 100,000-nit sphere light or emissive grid, nor any other reference.
    for gone in ["FlatGrid", "SphereLight", "default_environment", "references", "https://"] {
        assert!(!usda.contains(gone), "{gone}");
    }

    // The lights: Kit's Default rig (sun 1 · 2^10, 7250 K, 0.53°, 35° up in the north-east;
    // sky 1 · 2^9, 6150 K, unseen by the camera).
    let sun = prim_lines(&usda, "    def DistantLight \"defaultLight\" (");
    for line in [
        "float inputs:angle = 0.53",
        "float inputs:colorTemperature = 7250",
        "bool inputs:enableColorTemperature = 1",
        "float inputs:exposure = 10",
        "float inputs:intensity = 1",
        "quatd xformOp:orient = (0.33944435018566016, 0.17670354420260315, 0.42660009293331813, 0.8194911538891423)",
    ] {
        assert!(sun.contains(&line), "{line} not in {sun:?}");
    }
    let [w, x, y, z] = [0.33944435018566016f64, 0.17670354420260315, 0.42660009293331813, 0.8194911538891423];
    // −Z turned by the quaternion: the light's direction of travel.
    let along = [2.0 * (-x * z - w * y), 2.0 * (w * x - y * z), -(1.0 - 2.0 * (x * x + y * y))];
    assert!(((-along[2]).asin().to_degrees() - 35.0).abs() < 1e-9, "{along:?}");
    assert!(((-along[1]).atan2(-along[0]).to_degrees() - 45.0).abs() < 1e-9, "{along:?}");
    let sky = prim_lines(&usda, "    def DomeLight \"domeLight\" (");
    for line in [
        "bool visibleInPrimaryRay = 0",
        "float inputs:colorTemperature = 6150",
        "bool inputs:enableColorTemperature = 1",
        "float inputs:exposure = 9",
        "float inputs:intensity = 1",
    ] {
        assert!(sky.contains(&line), "{line} not in {sky:?}");
    }
    assert!(!sky.iter().any(|l| l.contains("texture:file")));
    assert!(usda.contains("token omni:rtx:background:source:type = \"domeLight\""));
    assert_eq!(usda.matches("Light \"").count(), 2);

    // The ground: a matte mid-grey square under the cell and its collider at z = 0.
    let ground = prim_lines(&usda, "    def Mesh \"Ground\" (");
    assert!(ground.contains(&"rel material:binding = </Environment/Looks/M_ground>"));
    assert!(ground.iter().any(|l| l.starts_with("point3f[] points = [(-") && l.ends_with(", 0)]")));
    let material = &usda[usda.find("def Material \"M_ground\"").unwrap()..];
    let material = &material[..material.find("token outputs:surface\n").unwrap()];
    for line in [
        "color3f inputs:diffuseColor = (0.18, 0.18, 0.18)",
        "float inputs:metallic = 0",
        "float inputs:roughness = 0.85",
    ] {
        assert!(material.contains(line), "{line}");
    }
    let collider = prim_lines(&usda, "    def Plane \"GroundCollider\" (");
    assert_eq!(collider, ["uniform token axis = \"Z\"", "uniform token purpose = \"guide\""]);
    assert!(usda.contains("    def Plane \"GroundCollider\" (\n        prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n"));

    // Isaac Sim's own render settings and mode (RealTimePathTracing in 6.x).
    assert!(usda.contains("        dictionary renderSettings = {\n        }\n"));
    assert!(!usda.contains("rendermode") && !usda.contains("rtx:post:aa:op:enabled"));
    assert!(usda.contains("uint omni:rtx:pt:maxSamplesPerLaunch = 2073600\n"));
    assert!(usda.contains("uint omni:rtx:pt:mgpu:maxPixelsPerRegionExponent = 12\n"));
    assert!(usda.contains("    def \"Vars\" (\n        hide_in_stage_window = true\n        no_delete = true\n    )\n"));
}
