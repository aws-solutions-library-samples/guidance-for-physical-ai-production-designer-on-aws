//! Real-asset integration tests. Every scene manifest in an asset tree's
//! `fitted-robots/` (the layout `serve --asset-root` works on) is parsed and
//! validated, and the smallest one is composed in lean mode, so real URDFs,
//! meshes, materials and manifests are exercised next to the synthetic
//! fixtures of the other tests (mesh-cache lookups, the material registry,
//! the mass model on real inertials, real `home_pose_deg`). Every expectation
//! is derived from the manifest at hand — its name, robots, fixtures and their
//! URDFs — so nothing here knows a particular cell.
//!
//! **Opt-in via the `real-assets` Cargo feature.** The asset root is
//! `$RFR_ASSET_ROOT`, else `<crate>/../assets/`. A manifest that names a URDF
//! or floor plan that doesn't exist is skipped with a message, and so is every
//! test when no manifest is left, so a partial checkout doesn't masquerade as
//! a failing implementation.
//!
//! Run with:
//!     RFR_ASSET_ROOT=<asset tree> cargo test -p robo-fitting-room \
//!         --features real-assets --test real_assets -- --nocapture

#![cfg(feature = "real-assets")]

use std::path::{Path, PathBuf};

use robo_fitting_room::{compose_stage_with_options, parse_scene, validate_scene, ComposeOptions};
use serde_yaml::Value;
use urdf_robo_dresser::{parse_urdf, Geometry, JointType, UrdfRobot};

/// The asset root: `$RFR_ASSET_ROOT`, else `<crate>/../assets/`.
fn asset_root() -> PathBuf {
    std::env::var_os("RFR_ASSET_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../assets"))
}

/// A placement as the manifest states it: its id and its URDF, resolved
/// against the manifest's directory.
#[derive(Debug)]
struct Entry {
    id: String,
    urdf: PathBuf,
}

/// A scene manifest as its YAML states it, read without the crate's parser.
#[derive(Debug)]
struct Manifest {
    path: PathBuf,
    name: String,
    robots: Vec<Entry>,
    fixtures: Vec<Entry>,
    floor_plan: Option<PathBuf>,
    /// The manifest's `scene_origin`, (0, 0, 0) without one.
    scene_origin: [f64; 3],
    /// Each robot's `position` and `orientation` (a, b, c), as written.
    robot_poses: Vec<([f64; 3], [f64; 3])>,
}

impl Manifest {
    fn read(path: &Path) -> Self {
        let at = path.display();
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{at}: {e}"));
        let yaml: Value = serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{at}: {e}"));
        let dir = path.parent().unwrap();
        let list = |key: &str| -> Vec<Value> {
            yaml.get(key)
                .and_then(Value::as_sequence)
                .cloned()
                .unwrap_or_default()
        };
        let entries = |key: &str, id: &str, file: &str| -> Vec<Entry> {
            list(key)
                .iter()
                .map(|item| Entry {
                    id: scalar(&item[id], path),
                    urdf: dir.join(scalar(&item[file], path)),
                })
                .collect()
        };
        Self {
            path: path.to_path_buf(),
            name: scalar(&yaml["name"], path),
            robots: entries("robots", "robot_id", "dressed_urdf"),
            fixtures: entries("fixtures", "id", "urdf"),
            floor_plan: yaml
                .get("floor_plan")
                .filter(|fp| !fp.is_null())
                .map(|fp| dir.join(scalar(&fp["geometry_file"], path))),
            scene_origin: triple(yaml.get("scene_origin"), ["x", "y", "z"], path),
            robot_poses: list("robots")
                .iter()
                .map(|robot| {
                    (
                        triple(robot.get("position"), ["x", "y", "z"], path),
                        triple(robot.get("orientation"), ["a", "b", "c"], path),
                    )
                })
                .collect(),
        }
    }

    /// The files the manifest names that don't exist.
    fn missing_files(&self) -> Vec<&Path> {
        self.robots
            .iter()
            .chain(&self.fixtures)
            .map(|entry| entry.urdf.as_path())
            .chain(self.floor_plan.as_deref())
            .filter(|file| !file.exists())
            .collect()
    }
}

/// A YAML scalar as text (a number as written).
fn scalar(value: &Value, manifest: &Path) -> String {
    match value {
        Value::String(s) => s.clone(),
        Value::Number(n) => n.to_string(),
        other => panic!("{}: expected a string, got {other:?}", manifest.display()),
    }
}

/// The numbers under `keys` of a YAML mapping, 0 for an absent mapping or key.
fn triple(map: Option<&Value>, keys: [&str; 3], manifest: &Path) -> [f64; 3] {
    keys.map(|key| match map.and_then(|m| m.get(key)) {
        None => 0.0,
        Some(v) => v
            .as_f64()
            .unwrap_or_else(|| panic!("{}: {key} is not a number: {v:?}", manifest.display())),
    })
}

/// The manifests (`*.yaml`, `*.yml`) directly in `<asset root>/fitted-robots/`
/// whose URDFs and floor plan all exist, in path order; the others are
/// reported and skipped.
fn manifests() -> Vec<Manifest> {
    let dir = asset_root().join("fitted-robots");
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("Skipping real-asset tests — no {}", dir.display());
        return vec![];
    };
    let mut paths: Vec<PathBuf> = entries
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            path.is_file()
                && !path.file_name().unwrap().to_string_lossy().starts_with('.')
                && matches!(
                    path.extension().and_then(|e| e.to_str()),
                    Some("yaml" | "yml")
                )
        })
        .collect();
    paths.sort();
    let mut found = Vec::new();
    for path in paths {
        let manifest = Manifest::read(&path);
        let missing = manifest.missing_files();
        if missing.is_empty() {
            found.push(manifest);
        } else {
            eprintln!(
                "Skipping {} — it names files that don't exist: {missing:?}",
                path.display()
            );
        }
    }
    if found.is_empty() {
        eprintln!(
            "Skipping real-asset tests — no manifest with all its files under {}",
            dir.display()
        );
    }
    found
}

/// FR-151's sanitising of one namespace segment, written from the
/// requirement: a digit-leading (or empty) value gets an `r_` prefix and
/// every character outside `[A-Za-z0-9_]` becomes `_`.
fn ros_segment(raw: &str) -> String {
    let prefix = match raw.chars().next() {
        Some(c) if !c.is_ascii_digit() => "",
        _ => "r_",
    };
    let body: String = raw
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    format!("{prefix}{body}")
}

/// FR-114's prim name of a robot id, written from the requirement: `World` is
/// `world_link`; every character outside `[A-Za-z0-9_]` becomes `_`, and a
/// digit-leading name gets a leading `_`.
fn prim_name(raw: &str) -> String {
    if raw == "World" {
        return "world_link".into();
    }
    let mut name: String = raw
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if name.starts_with(|c: char| c.is_ascii_digit()) {
        name.insert(0, '_');
    }
    name
}

/// The actuated (non-fixed) joints of `robot`.
fn actuated_joints(robot: &UrdfRobot) -> usize {
    robot
        .joints
        .iter()
        .filter(|joint| joint.joint_type != JointType::Fixed)
        .count()
}

/// Whether `geometry` is a mesh of the dressed tool (under `meshes/tool/`).
fn is_tool_mesh(geometry: &Geometry) -> bool {
    matches!(geometry, Geometry::Mesh { filename, .. }
        if filename.to_ascii_lowercase().contains("meshes/tool/")
            || filename.to_ascii_lowercase().contains("meshes\\tool\\"))
}

/// FR-100/103/106/144: every manifest parses into the robots, fixtures and
/// floor plan its YAML lists — each robot where the YAML puts it, less the
/// scene origin — and validates cleanly.
#[test]
fn every_manifest_parses_and_validates() {
    for manifest in manifests() {
        let at = manifest.path.display();
        let scene = parse_scene(&manifest.path).unwrap_or_else(|e| panic!("{at}: {e}"));
        assert_eq!(scene.name, manifest.name, "{at}");
        let robots: Vec<(&str, &Path)> = scene
            .robots
            .iter()
            .map(|r| (r.robot_id.as_str(), r.dressed_urdf.as_path()))
            .collect();
        let expected: Vec<(&str, &Path)> = manifest
            .robots
            .iter()
            .map(|e| (e.id.as_str(), e.urdf.as_path()))
            .collect();
        assert_eq!(robots, expected, "{at}");
        let fixtures: Vec<(&str, &Path)> = scene
            .fixtures
            .iter()
            .map(|f| (f.id.as_str(), f.urdf.as_path()))
            .collect();
        let expected: Vec<(&str, &Path)> = manifest
            .fixtures
            .iter()
            .map(|e| (e.id.as_str(), e.urdf.as_path()))
            .collect();
        assert_eq!(fixtures, expected, "{at}");
        assert_eq!(
            scene.floor_plan.is_some(),
            manifest.floor_plan.is_some(),
            "{at}"
        );
        for (robot, (position, orientation)) in scene.robots.iter().zip(&manifest.robot_poses) {
            for k in 0..3 {
                let want = position[k] - manifest.scene_origin[k];
                assert!(
                    (robot.position[k] - want).abs() <= 1e-9 * want.abs().max(1.0),
                    "{at}: {} position {:?}, want {position:?} less {:?}",
                    robot.robot_id,
                    robot.position,
                    manifest.scene_origin
                );
            }
            assert_eq!(
                robot.orientation_deg, *orientation,
                "{at}: {}",
                robot.robot_id
            );
        }

        let errors = validate_scene(&scene);
        assert!(
            errors.is_empty(),
            "{at} must validate cleanly, got: {errors:?}"
        );
        eprintln!(
            "Validated {at}: {} robots, {} fixtures, floor plan: {}",
            scene.robots.len(),
            scene.fixtures.len(),
            scene.floor_plan.is_some()
        );
    }
}

/// FR-101/110/115/145/146/151/152/160/162: the smallest manifest (fewest
/// robots but at least one, then fewest fixtures) composes in lean mode, the
/// default, into a zip of the manifest as given and `<manifest stem>.usda`:
/// one articulation per robot, the cell's clock graph and one ActionGraph per
/// robot with actuated joints, its topics under the `<cell_id>/<robot_id>`
/// namespace computed from the manifest's `name` and `robot_id`, a contact
/// sensor per collision link, and of the robots' visuals only the tools'.
#[test]
fn the_smallest_manifest_composes_in_lean_mode() {
    let Some(manifest) = manifests()
        .into_iter()
        .min_by_key(|m| (m.robots.is_empty(), m.robots.len(), m.fixtures.len()))
    else {
        return;
    };
    let at = manifest.path.display().to_string();
    eprintln!("Composing {at} in lean mode");
    let scene = parse_scene(&manifest.path).unwrap_or_else(|e| panic!("{at}: {e}"));
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("lean.zip");
    let options = ComposeOptions::default();
    assert!(
        !options.full_robots && options.full_tools,
        "lean mode is the default"
    );
    let report = compose_stage_with_options(&scene, &manifest.path, &out, &options)
        .unwrap_or_else(|e| panic!("{at}: {e}"));
    assert_eq!(report.robot_count, manifest.robots.len(), "{at}");
    assert_eq!(report.fixture_count, manifest.fixtures.len(), "{at}");
    assert!(report.zip_bytes > 0, "{at}");

    // The manifest rides along as given, next to the stage named after it.
    let stem = manifest.path.file_stem().unwrap().to_string_lossy();
    assert_eq!(report.usda_name, format!("{stem}.usda"), "{at}");
    assert_eq!(
        read_zip_entry(&out, &report.manifest_name),
        std::fs::read_to_string(&manifest.path).unwrap(),
        "{at}"
    );
    let usda = read_zip_entry(&out, &report.usda_name);

    // One articulation per robot.
    assert_eq!(
        usda.matches(
            "prepend apiSchemas = [\"PhysicsArticulationRootAPI\", \"PhysxArticulationAPI\"]"
        )
        .count(),
        manifest.robots.len(),
        "{at}"
    );

    let cell = ros_segment(&manifest.name);
    // The clock graph is cell-scoped; a scene without robots has none.
    let clock = format!("inputs:nodeNamespace = \"{cell}\"\n");
    assert_eq!(
        usda.contains(&clock),
        !manifest.robots.is_empty(),
        "{at}: {clock}"
    );
    let mut graphs = usize::from(!manifest.robots.is_empty());
    let mut sensors = 0;
    for robot in &manifest.robots {
        let urdf = parse_urdf(&robot.urdf).unwrap_or_else(|e| panic!("{at}: {e}"));
        let namespace = format!(
            "inputs:nodeNamespace = \"{cell}/{}\"\n",
            ros_segment(&robot.id)
        );
        if actuated_joints(&urdf) > 0 {
            graphs += 1;
            sensors += urdf
                .links
                .iter()
                .filter(|l| !l.collisions.is_empty())
                .count();
            assert!(
                usda.contains(&namespace),
                "{at}: {} not under {namespace}",
                robot.id
            );
        } else {
            assert!(
                !usda.contains(&namespace),
                "{at}: {} has no actuated joint",
                robot.id
            );
        }
        // Lean (FR-145): the robot's hulls are emitted, and of its visuals only the tool's.
        let prim = block(
            &usda,
            &format!("    def Xform \"{}\" (", prim_name(&robot.id)),
        );
        assert!(prim.contains("def Mesh \"collision_"), "{at}: {}", robot.id);
        let tool_visuals = urdf
            .links
            .iter()
            .flat_map(|l| &l.visuals)
            .filter(|v| is_tool_mesh(&v.geometry))
            .count();
        assert_eq!(
            prim.matches("def Mesh \"visual_").count(),
            tool_visuals,
            "{at}: {}'s visuals",
            robot.id
        );
    }
    assert_eq!(
        usda.matches("def OmniGraph \"ActionGraph_").count(),
        graphs,
        "{at}"
    );
    // Contact sensors (FR-160/162): one sensor and Bool publisher per collision link.
    assert_eq!(
        usda.matches("token node:type = \"isaacsim.sensors.physics.IsaacReadContactSensor\"")
            .count(),
        sensors,
        "{at}"
    );
    assert_eq!(
        usda.matches("custom string inputs:messageName = \"Bool\"")
            .count(),
        sensors,
        "{at}"
    );
    eprintln!(
        "Composed {at}: {} robots, {} fixtures, {graphs} graphs, {sensors} contact sensors, \
         {} inline meshes",
        report.robot_count, report.fixture_count, report.mesh_count
    );
}

/// The text of the first block whose head line starts with `head`
/// (indentation included), through its closing brace.
fn block<'a>(usda: &'a str, head: &str) -> &'a str {
    let start = usda.find(head).unwrap_or_else(|| panic!("no {head}"));
    let indent = &head[..head.len() - head.trim_start().len()];
    let close = format!("\n{indent}}}\n");
    let end = usda[start..].find(&close).unwrap() + close.len();
    &usda[start..start + end]
}

fn read_zip_entry(zip_path: &Path, entry: &str) -> String {
    use std::io::Read;
    let f = std::fs::File::open(zip_path).unwrap();
    let mut z = zip::ZipArchive::new(f).unwrap();
    let mut e = z.by_name(entry).unwrap();
    let mut s = String::new();
    e.read_to_string(&mut s).unwrap();
    s
}
