//! The task shown and checked in the Robo Fitter, in the scene frame (scene YAML coordinates):
//! the welds of the products placed as fixtures (moved with them), or the linked cell's program.

use std::path::{Path, PathBuf};

use factoryos_protocol::fitter::{FixtureYaml, SceneFile, TaskSource, TaskWeld};
use factoryos_protocol::product::{Geometry, Product, Target};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

/// Path operations are checked at points about this far apart.
const PATH_STEP_M: f64 = 0.05;
/// Program and product welds this close are the same weld.
const SAME_WELD_M: f64 = 0.01;

/// A product's box in the scene frame.
#[derive(Clone, Debug)]
pub struct ProductBox {
    pub name: String,
    /// The box centre's pose (scene frame).
    pub pose: Isometry3<f64>,
    pub half: [f64; 3],
}

impl ProductBox {
    /// Signed distance of `p` (scene frame) from the box surface: negative inside.
    pub fn signed_distance(&self, p: [f64; 3]) -> f64 {
        let l = self.pose.inverse_transform_point(&Point3::from(p));
        let q = [l.x.abs() - self.half[0], l.y.abs() - self.half[1], l.z.abs() - self.half[2]];
        let outside = q.iter().map(|v| v.max(0.0).powi(2)).sum::<f64>().sqrt();
        outside + q[0].max(q[1]).max(q[2]).min(0.0)
    }
}

/// A product placed in the scene, with its welds (scene frame).
#[derive(Clone, Debug)]
pub struct PlacedProduct {
    pub name: String,
    pub welds: Vec<TaskWeld>,
    /// Path operations as polylines (scene frame), for drawing.
    pub paths: Vec<Vec<[f64; 3]>>,
    pub geometry: Option<ProductBox>,
}

/// Everything the scene's task can come from.
#[derive(Clone, Debug, Default)]
pub struct SceneTask {
    pub products: Vec<PlacedProduct>,
    /// The linked cell's program (`None`: not linked, or it has none).
    pub program: Option<(String, Vec<TaskWeld>)>,
}

impl SceneTask {
    pub fn load(scene: &SceneFile, scene_base: &Path, cell_root: Option<&Path>) -> Self {
        let products = scene.fixtures.iter().filter_map(|f| placed_product(f, scene_base)).collect();
        let program = cell_root.and_then(task_program);
        Self { products, program }
    }

    /// `Auto`: the cell program when linked to a cell that has one, else the products. `CellProgram`
    /// without one (a fresh scene, the choice kept from a linked cell) also falls back to the products.
    pub fn resolve(&self, source: TaskSource) -> TaskSource {
        let has_program = self.program.as_ref().is_some_and(|(_, w)| !w.is_empty());
        match source {
            TaskSource::Auto | TaskSource::CellProgram if has_program => TaskSource::CellProgram,
            TaskSource::Auto | TaskSource::CellProgram => TaskSource::Product,
            other => other,
        }
    }

    /// The welds of `source` (resolved) and the notes on them.
    pub fn welds(&self, source: TaskSource) -> (TaskSource, Vec<TaskWeld>, Vec<String>) {
        let source = self.resolve(source);
        let product: Vec<TaskWeld> = self.products.iter().flat_map(|p| p.welds.clone()).collect();
        let program = self.program.as_ref().map(|(_, w)| w.clone()).unwrap_or_default();
        let mut notes = Vec::new();
        if let Some(note) = self.mismatch() {
            notes.push(note);
        }
        let welds = match source {
            TaskSource::CellProgram => {
                if self.program.is_none() {
                    notes.push("no cell program: the scene is not linked to a cell with agentic_planner/tasks/*.json".to_owned());
                }
                program
            }
            _ => {
                if self.products.is_empty() {
                    notes.push("no product in the scene: place a Product & Task product as a fixture".to_owned());
                }
                product
            }
        };
        (source, welds, notes)
    }

    /// "cell program has 12 welds, product bug_cube has 4; program does not match product", when
    /// both exist and differ.
    pub fn mismatch(&self) -> Option<String> {
        let (_, program) = self.program.as_ref()?;
        if self.products.is_empty() || program.is_empty() {
            return None;
        }
        let product: Vec<&TaskWeld> = self.products.iter().flat_map(|p| &p.welds).collect();
        let matched = program.len() == product.len()
            && program.iter().all(|w| product.iter().any(|p| distance(p.position, w.position) <= SAME_WELD_M));
        if matched {
            return None;
        }
        let products: Vec<String> = self.products.iter().map(|p| format!("product {} has {}", p.name, p.welds.len())).collect();
        Some(format!("cell program has {} welds, {}; program does not match product", program.len(), products.join(", ")))
    }

    pub fn boxes(&self) -> Vec<&ProductBox> {
        self.products.iter().filter_map(|p| p.geometry.as_ref()).collect()
    }

    pub fn paths(&self) -> Vec<Vec<[f64; 3]>> {
        self.products.iter().flat_map(|p| p.paths.clone()).collect()
    }
}

/// A fixture's pose in the scene frame (`Rz(a)·Ry(b)·Rx(c)`, degrees).
pub fn fixture_pose(fixture: &FixtureYaml) -> Isometry3<f64> {
    let o = fixture.orientation;
    Isometry3::from_parts(
        Translation3::new(fixture.position.x, fixture.position.y, fixture.position.z),
        UnitQuaternion::from_euler_angles(o.c.to_radians(), o.b.to_radians(), o.a.to_radians()),
    )
}

/// The fixture as a product, if its URDF's folder has a `product.json`.
fn placed_product(fixture: &FixtureYaml, scene_base: &Path) -> Option<PlacedProduct> {
    let urdf = scene_base.join(&fixture.urdf);
    let file = urdf.parent()?.join("product.json");
    let product: Product = serde_json::from_str(&std::fs::read_to_string(&file).ok()?)
        .map_err(|err| tracing::warn!("Robo Fitter: {}: {err}", file.display()))
        .ok()?;
    Some(place(&product, fixture))
}

/// `product`'s welds and box placed by `fixture`: `p_w = R·p + position`, `approach = −R·normal`.
pub fn place(product: &Product, fixture: &FixtureYaml) -> PlacedProduct {
    let pose = fixture_pose(fixture);
    let at = |p: [f32; 3]| {
        let w = pose * Point3::new(p[0] as f64, p[1] as f64, p[2] as f64);
        [w.x, w.y, w.z]
    };
    let approach = |n: [f32; 3]| {
        let v = -(pose.rotation * Vector3::new(n[0] as f64, n[1] as f64, n[2] as f64));
        let v = if v.norm() < 1e-9 { Vector3::new(0.0, 0.0, -1.0) } else { v.normalize() };
        [v.x, v.y, v.z]
    };
    let mut welds = Vec::new();
    let mut paths = Vec::new();
    for task in &product.tasks {
        for op in &task.operations {
            let weld = |id: String, label: String, position: [f64; 3]| TaskWeld {
                id,
                label,
                fixture_id: Some(fixture.id.clone()),
                task: task.name.clone(),
                position,
                approach: approach(op.target.normal()),
            };
            match &op.target {
                Target::Point { position, .. } => {
                    welds.push(weld(format!("{}/{}", fixture.id, op.id), op.id.clone(), at(*position)));
                }
                Target::Path { points, .. } => {
                    let line: Vec<[f64; 3]> = points.iter().map(|p| at(*p)).collect();
                    for (k, p) in sample(&line, PATH_STEP_M).into_iter().enumerate() {
                        welds.push(weld(format!("{}/{}.{}", fixture.id, op.id, k + 1), format!("{}.{}", op.id, k + 1), p));
                    }
                    paths.push(line);
                }
            }
        }
    }
    let geometry = match product.geometry {
        Geometry::Box { size } => {
            let half = size.map(|v| v as f64 / 2.0);
            // A box stands on its origin.
            Some(ProductBox {
                name: product.name.clone(),
                pose: pose * Translation3::new(0.0, 0.0, half[2]),
                half,
            })
        }
        Geometry::Mesh { .. } | Geometry::Urdf { .. } => None,
    };
    PlacedProduct { name: product.name.clone(), welds, paths, geometry }
}

/// Points along `line` about `step` apart, both ends included.
fn sample(line: &[[f64; 3]], step: f64) -> Vec<[f64; 3]> {
    let Some(first) = line.first() else { return Vec::new() };
    let mut out = vec![*first];
    for pair in line.windows(2) {
        let d = distance(pair[0], pair[1]);
        let n = (d / step - 1e-6).ceil().max(1.0) as usize;
        for i in 1..=n {
            let t = i as f64 / n as f64;
            out.push([0, 1, 2].map(|k| pair[0][k] + (pair[1][k] - pair[0][k]) * t));
        }
    }
    out
}

pub fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// A weld of a task program: id, position, approach direction (unit, from the tool towards the weld).
pub type Weld = (String, [f64; 3], Option<[f64; 3]>);

/// The task program files of cell `root`: `agentic_planner/tasks/*.json`, then `tasks/*.json`, by name.
fn program_files(root: &Path) -> Vec<PathBuf> {
    let json_files = |dir: PathBuf| {
        let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
            .into_iter()
            .flatten()
            .flatten()
            .map(|entry| entry.path())
            .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
            .collect();
        files.sort();
        files
    };
    let mut files = json_files(root.join("agentic_planner/tasks"));
    files.extend(json_files(root.join("tasks")));
    files
}

/// The welds of cell `root`'s first task program that has any, and its name.
fn program_welds(root: &Path) -> Option<(String, Vec<Weld>)> {
    let vec3 = |v: &serde_json::Value| -> Option<[f64; 3]> {
        let a = v.as_array()?;
        (a.len() == 3).then_some(())?;
        Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
    };
    for file in program_files(root) {
        let Some(task) = std::fs::read_to_string(&file).ok().and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        else {
            continue;
        };
        let welds: Vec<Weld> = (task["operations"].as_array().into_iter().flatten())
            .enumerate()
            .filter_map(|(i, op)| {
                let position = vec3(&op["position"])?;
                let id = op["id"].as_str().map_or_else(|| format!("W{:02}", i + 1), str::to_owned);
                Some((id, position, vec3(&op["approach_vector"])))
            })
            .collect();
        if !welds.is_empty() {
            let name = task["program"].as_str().map(str::to_owned);
            let name = name.unwrap_or_else(|| file.file_stem().unwrap_or_default().to_string_lossy().into_owned());
            return Some((name, welds));
        }
    }
    None
}

/// The welds of cell `root`'s first task program that has any (`agentic_planner/tasks/*.json`,
/// else the cell's own `tasks/*.json`, by name).
pub fn task_welds(root: &Path) -> Vec<Weld> {
    program_welds(root).map(|(_, welds)| welds).unwrap_or_default()
}

/// The cell's program as task welds (scene frame; a weld without an approach is approached from above).
pub fn task_program(root: &Path) -> Option<(String, Vec<TaskWeld>)> {
    let (name, welds) = program_welds(root)?;
    let welds = welds
        .into_iter()
        .map(|(id, position, approach)| {
            let a = approach.map(Vector3::from).filter(|v| v.norm() > 1e-9).map_or(Vector3::new(0.0, 0.0, -1.0), |v| v.normalize());
            TaskWeld { label: id.clone(), id, fixture_id: None, task: name.clone(), position, approach: [a.x, a.y, a.z] }
        })
        .collect();
    Some((name, welds))
}

#[cfg(test)]
pub(crate) mod tests {

    #[test]
    fn a_scene_without_a_cell_program_shows_the_products_welds() {
        let weld = TaskWeld {
            id: "bug_cube/W01".into(),
            label: "W01".into(),
            fixture_id: Some("bug_cube".into()),
            task: "spot_weld_1".into(),
            position: [0.5, 0.5, 2.0],
            approach: [0.0, 0.0, -1.0],
        };
        let mut task = SceneTask::default();
        task.products.push(PlacedProduct { name: "bug_cube".into(), welds: vec![weld], paths: Vec::new(), geometry: None });
        let (source, welds, _) = task.welds(TaskSource::CellProgram);
        assert_eq!((source, welds.len()), (TaskSource::Product, 1));
    }

    use super::*;
    use factoryos_protocol::fitter::{Abc, Xyz};
    use factoryos_protocol::product::{Operation, Task};

    pub fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/jam2")
    }

    /// jam2 as on the user's workspace, with the paths of the fixtures folder.
    pub fn jam2() -> SceneFile {
        serde_yaml::from_str(&std::fs::read_to_string(fixtures().join("scenes/jam2/scene.yaml")).unwrap()).unwrap()
    }

    fn fixture(x: f64, y: f64, a: f64) -> FixtureYaml {
        FixtureYaml { id: "f".into(), urdf: "f.urdf".into(), position: Xyz { x, y, z: 0.0 }, orientation: Abc { a, b: 0.0, c: 0.0 }, ..Default::default() }
    }

    #[test]
    fn product_welds_move_with_the_fixture() {
        let op = |id: &str, target| Operation { id: id.into(), target, group: String::new(), params: Default::default() };
        let product = Product {
            name: "p".into(),
            tasks: vec![Task {
                name: "spot".into(),
                process: "spotWeld".into(),
                operations: vec![
                    op("W1", Target::Point { position: [0.5, 0.0, 2.0], normal: [0.0, 0.0, 1.0] }),
                    op("S1", Target::Path { points: vec![[0.0, 0.0, 1.0], [0.0, 0.2, 1.0]], normal: [1.0, 0.0, 0.0] }),
                ],
                ..Default::default()
            }],
            ..Default::default()
        };
        let placed = place(&product, &fixture(1.0, 2.0, 90.0));
        let w = &placed.welds[0];
        assert_eq!((w.id.as_str(), w.label.as_str(), w.fixture_id.as_deref()), ("f/W1", "W1", Some("f")));
        for (got, want) in w.position.iter().zip([1.0, 2.5, 2.0]) {
            assert!((got - want).abs() < 1e-9, "{:?}", w.position);
        }
        for (got, want) in w.approach.iter().zip([0.0, 0.0, -1.0]) {
            assert!((got - want).abs() < 1e-9, "{:?}", w.approach);
        }
        // The path: 0.2 m sampled every 0.05 m: 5 points; its normal +x turned to +y, approached along −y.
        let path: Vec<&TaskWeld> = placed.welds.iter().filter(|w| w.label.starts_with("S1.")).collect();
        assert_eq!(path.len(), 5);
        assert_eq!(path[4].id, "f/S1.5");
        assert!((path[0].approach[1] + 1.0).abs() < 1e-9, "{:?}", path[0].approach);
        assert!((path[4].position[0] - 0.8).abs() < 1e-6, "{:?}", path[4].position);
        assert_eq!(placed.paths.len(), 1);
        // The default box (0.6 × 0.4 × 0.3) stands on the fixture's origin.
        let b = placed.geometry.unwrap();
        assert!(b.signed_distance([1.0, 2.0, 0.15]) < -0.1);
        assert!((b.signed_distance([1.0, 2.0, 0.4]) - 0.1).abs() < 1e-6);
    }

    #[test]
    fn jam2_product_and_program() {
        let base = fixtures().join("scenes/jam2");
        let task = SceneTask::load(&jam2(), &base, Some(&fixtures().join("cell")));
        let (source, welds, notes) = task.welds(TaskSource::Auto);
        assert_eq!(source, TaskSource::CellProgram, "linked to a cell with a program");
        assert_eq!(welds.len(), 4);
        assert!(notes.is_empty(), "the program matches the product: {notes:?}");
        let (source, product, _) = task.welds(TaskSource::Product);
        assert_eq!(source, TaskSource::Product);
        assert_eq!(product.iter().map(|w| w.id.as_str()).collect::<Vec<_>>(), ["bug_cube/W01", "bug_cube/W02", "bug_cube/W03", "bug_cube/W04"]);
        assert_eq!(product[1].position, [0.5, -0.5, 2.0]);

        // An agent-written program with welds on the walls, 6 of them 5 cm inside the cube.
        let walls = SceneTask::load(&jam2(), &base, Some(&fixtures().join("walls")));
        let (_, welds, notes) = walls.welds(TaskSource::Auto);
        assert_eq!(welds.len(), 12);
        assert_eq!(notes, ["cell program has 12 welds, product bug_cube has 4; program does not match product"]);
        let cube = walls.boxes()[0];
        let inside: Vec<&str> = welds.iter().filter(|w| cube.signed_distance(w.position) < -0.005).map(|w| w.id.as_str()).collect();
        assert_eq!(inside, ["W07", "W08", "W09", "W10", "W11", "W12"]);
        assert!(SceneTask::load(&jam2(), &base, None).welds(TaskSource::Auto).0 == TaskSource::Product);
    }
}
