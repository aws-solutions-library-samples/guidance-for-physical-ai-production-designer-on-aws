//! Task from product: a cell's task programs come from the Product & Task definitions of the
//! products placed in it, so the user defines the welds in the product, not in the raw task
//! program JSON.
//!
//! For every scene.yaml fixture that is a product (its URDF's folder, `fixtures/<product>/`, has a
//! `product.json`, as the Robo Fitter copies it) the backend writes one task program per product
//! task that has point operations: `agentic_planner/tasks/<fixture>__<task>.json`, in the task
//! program format of `assets/sample-task-cube-top-edges.json`, with a `source` key:
//! `{product, task, fixture, product_sha256, program_sha256}`. Before that the cell's copy of the
//! product is refreshed from the products folder.
//!
//! The transform is the Robo Fitter's (`reach::task::fixture_pose`) and the planner router's
//! (`deploy/planner-router/server.py`, `place_product`): `R = Rz(a)·Ry(b)·Rx(c)` (degrees),
//! `p_world = R·p + position`, `approach = −R·normal`; the orientation is the top-down one
//! `(0, −0.7071, 0, 0.7071)` turned by the shortest rotation from −z to the approach. Positions and
//! approach vectors are rounded to 6 decimals, quaternions to 4, as the router does.
//!
//! Path operations are not derived yet: they are listed in the program
//! (`extensions.product_task.skipped_paths`) and in the [`Derivation`].
//!
//! The mapping of the product task onto the task program:
//!
//! | Product & Task                         | task program                                              |
//! |----------------------------------------|-----------------------------------------------------------|
//! | task `params.sequence_groups_order`    | `extensions.strategy.sequence_groups_order` (default: the operations' groups, first seen first) |
//! | task `params.max_concurrent_robots`    | `extensions.strategy.max_concurrent_robots` (default: the scene's robot count, 2 without robots) |
//! | task `params.parallel_within_group`    | `extensions.strategy.parallel_within_group` (default true)|
//! | task `params.cycle_time_target_s`      | `extensions.cycle_time.target_s` (default 20)             |
//! | task `params.cycle_time_max_s`         | `extensions.cycle_time.max_s` (default 30)                |
//! | spotWeld `params.weldTime`             | `extensions.weld_params.welding_duration_s` and the `weld` phase's `duration_s` |
//! | spotWeld `params.approachOffset`       | `extensions.weld_params.approach_offset_z` and `retract_offset_z` |
//! | task `process`, `tool`, all `params`   | `extensions.product_task` (verbatim; other processes have no `weld_params`) |
//! | operation `id`                         | operation `id`                                            |
//! | operation `group`                      | operation `sequence_group` (default `structural`)         |
//! | operation `params.priority`            | operation `priority` (default 1)                          |
//! | other operation `params`               | operation `process_params`                                |
//! | process `spotWeld` / other             | operation `type` `weld` / the process key                 |
//!
//! Hand-written task programs (no `source`) are never overwritten. A derived program is re-derived
//! when what it would be changes (the product, the fixture's pose); one the user edited (its
//! `program_sha256` no longer matches its content) is kept with a warning, and listed as edited
//! (and stale, once its product changed). A derived program whose product task or fixture is gone
//! is removed, unless it was edited.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use factoryos_protocol::TaskSource;
use factoryos_protocol::fitter::{FixtureYaml, SceneFile};
use factoryos_protocol::product::{Product, Target, Task};
use nalgebra::{Point3, Vector3};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use super::planner_model::OBSTACLES_YAML;
use super::reach::task::fixture_pose;

pub const TASKS_DIR: &str = "agentic_planner/tasks";
const PRODUCT_FILE: &str = "product.json";
/// Between the fixture id and the task name in a derived file's name.
pub const SEPARATOR: &str = "__";
/// Obstacle boxes within this of a derived weld are marked `is_workpiece` (as the router does).
const WORKPIECE_TOL_M: f64 = 0.01;
/// ... and the boxes of that fixture's voxel shell within this: the planner keeps the tool's
/// 0.05 m spheres + 0.05 m margin clear of every other box, so its neighbouring voxels would block the weld.
const WORKPIECE_NEAR_M: f64 = 0.12;
/// Prepare Cell's near-weld refinement (the converter's `--refine-points`): within this of a weld
/// the converter replaces the coarse voxel shell (`<fixture>_v<n>`) by a fine one (`<fixture>_f<n>`)
/// of this voxel size, and the fine boxes of the welded fixture within the radius are workpiece.
pub const REFINE_VOXEL_SIZE_M: f64 = 0.02;
pub const REFINE_RADIUS_M: f64 = 0.25;
/// The task programs' top-down orientation, as written in them (4 decimals, not FRAC_1_SQRT_2).
#[allow(clippy::approx_constant)]
const TOP_DOWN: [f64; 4] = [0.0, -0.7071, 0.0, 0.7071];
/// The task program layout the derived programs follow (its operations are replaced).
const TEMPLATE: &str = include_str!("../../assets/sample-task-cube-top-edges.json");

/// What [`derive_cell`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Derivation {
    /// Task programs written (new or re-derived), by file stem.
    pub written: Vec<String>,
    /// Derived task programs already up to date.
    pub unchanged: Vec<String>,
    /// Derived task programs removed: their product task or fixture is gone.
    pub removed: Vec<String>,
    /// Fixtures whose `product.json` copy was refreshed from the products folder.
    pub refreshed: Vec<String>,
    /// Obstacle boxes marked `is_workpiece`.
    pub workpieces: Vec<String>,
    /// Not derived: path operations, tasks without point operations.
    pub skipped: Vec<String>,
    /// Kept files (edited, hand-written) and unreadable products.
    pub warnings: Vec<String>,
}

impl Derivation {
    pub fn changed(&self) -> bool {
        !(self.written.is_empty() && self.removed.is_empty() && self.refreshed.is_empty() && self.workpieces.is_empty())
    }

    /// One line for the log, or `None` when nothing happened.
    pub fn summary(&self, cell: &str) -> Option<String> {
        if !self.changed() && self.warnings.is_empty() {
            return None;
        }
        let mut parts = Vec::new();
        let list = |label: &str, items: &[String]| (!items.is_empty()).then(|| format!("{label} {}", items.join(", ")));
        parts.extend(list("wrote", &self.written));
        parts.extend(list("removed", &self.removed));
        parts.extend(list("refreshed the product copy of", &self.refreshed));
        parts.extend(list("marked is_workpiece:", &self.workpieces));
        parts.extend(list("skipped", &self.skipped));
        parts.extend(self.warnings.iter().cloned());
        Some(format!("Task programs of cell {cell} from its products: {}", parts.join("; ")))
    }
}

/// A scene.yaml fixture that is a product.
struct FixtureProduct {
    fixture: FixtureYaml,
    /// The product's name (its folder under the products folder).
    name: String,
    /// The cell's copy: `fixtures/<product>/product.json`.
    copy: PathBuf,
    /// The products folder's file, if it has one.
    original: Option<PathBuf>,
}

/// The scene of cell `root` when the caller does not say (see [`derive_cell_with`]): a legacy
/// cell's own `scene.yaml`, else a reference cell's view (`.view/scene.yaml`, its paths into the
/// view's links to the catalogue and the products), if it has one.
pub fn cell_scene(root: &Path) -> PathBuf {
    let own = root.join("scene.yaml");
    let view = root.join(super::cell_resolve::VIEW_DIR).join("scene.yaml");
    if !own.is_file() && view.is_file() { view } else { own }
}

/// The fixtures of the scene `scene_path` that are products (`None`: no readable scene). Fixture
/// paths are relative to the scene's folder: a legacy cell's root, or `scenes/<name>/` for the
/// scene a reference cell points at (whose products are the products folder's own files).
fn fixture_products(scene_path: &Path, products_dir: &Path) -> Option<Vec<FixtureProduct>> {
    let text = std::fs::read_to_string(scene_path).ok()?;
    let scene: SceneFile = super::scene_paths::parse_scene_file(&text).ok()?;
    let root = scene_path.parent().unwrap_or(Path::new("."));
    Some(
        scene
            .fixtures
            .into_iter()
            .filter_map(|fixture| {
                let folder = super::scene_store::asset_path(root, &fixture.urdf).parent()?.to_path_buf();
                let copy = folder.join(PRODUCT_FILE);
                let folder_name = folder.file_name()?.to_string_lossy().into_owned();
                let named = |name: &str| {
                    let file = products_dir.join(name).join(PRODUCT_FILE);
                    (factoryos_protocol::product::valid_name(name) && file.is_file()).then_some(file)
                };
                let original = named(&folder_name).or_else(|| {
                    let product: Product = serde_json::from_str(&std::fs::read_to_string(&copy).ok()?).ok()?;
                    named(&product.name)
                });
                (copy.is_file() || original.is_some()).then(|| {
                    let name = original
                        .as_ref()
                        .and_then(|f| f.parent()?.file_name())
                        .map_or(folder_name, |n| n.to_string_lossy().into_owned());
                    FixtureProduct { fixture, name, copy, original }
                })
            })
            .collect(),
    )
}

/// How many robots the scene `scene_path` has (0 if unreadable).
fn scene_robot_count(scene_path: &Path) -> usize {
    std::fs::read_to_string(scene_path)
        .ok()
        .and_then(|text| super::scene_paths::parse_scene_file(&text).ok())
        .map_or(0, |scene: SceneFile| scene.robots.len())
}

/// The world positions of the point operations of every product task of the scene `scene_path`'s
/// products (the products folder's file, else the cell's copy: what [`derive_cell_with`] derives
/// from), for the converter's near-weld refinement before the task programs are derived.
pub fn weld_points(scene_path: &Path, products_dir: &Path) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for fp in fixture_products(scene_path, products_dir).unwrap_or_default() {
        let file = fp.original.as_ref().unwrap_or(&fp.copy);
        let Some(product) = std::fs::read(file).ok().and_then(|bytes| serde_json::from_slice::<Product>(&bytes).ok()) else {
            continue;
        };
        for op in product.tasks.iter().flat_map(|t| &t.operations) {
            if let Target::Point { position, normal } = &op.target {
                points.push(place_point(&fp.fixture, *position, *normal).0);
            }
        }
    }
    points
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

/// `name` usable in a file stem: letters, digits, `_` and `-`.
fn file_part(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '-') { c } else { '_' }).collect()
}

/// The derived task program's file stem for `task` of the product at `fixture`.
pub fn program_name(fixture: &str, task: &str, index: usize) -> String {
    let task = if task.trim().is_empty() { format!("task{}", index + 1) } else { file_part(task) };
    format!("{}{SEPARATOR}{task}", file_part(fixture))
}

/// JSON with the object keys sorted, compact: what `program_sha256` hashes (key order and
/// formatting are not edits).
fn canonical(value: &Value) -> String {
    match value {
        Value::Object(map) => {
            let mut keys: Vec<&String> = map.keys().collect();
            keys.sort();
            let items: Vec<String> =
                keys.iter().map(|k| format!("{}:{}", Value::String((*k).clone()), canonical(&map[k.as_str()]))).collect();
            format!("{{{}}}", items.join(","))
        }
        Value::Array(items) => format!("[{}]", items.iter().map(canonical).collect::<Vec<_>>().join(",")),
        other => other.to_string(),
    }
}

/// The hash of a program's content, its `source.program_sha256` left out.
fn program_sha(program: &Value) -> String {
    let mut program = program.clone();
    if let Some(source) = program.get_mut("source").and_then(Value::as_object_mut) {
        source.remove("program_sha256");
    }
    sha256_hex(canonical(&program).as_bytes())
}

/// Whether a derived program was edited since the backend wrote it.
fn edited(program: &Value) -> bool {
    program["source"]["program_sha256"].as_str() != Some(program_sha(program).as_str())
}

fn round(v: f64, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (v * f).round() / f + 0.0
}

type M3 = [[f64; 3]; 3];

fn matmul(a: &M3, b: &M3) -> M3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn quat_to_matrix([x, y, z, w]: [f64; 4]) -> M3 {
    let n = (x * x + y * y + z * z + w * w).sqrt();
    let (x, y, z, w) = (x / n, y / n, z / n, w / n);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

/// As the router's `matrix_to_quat`: `w ≥ 0`, rounded to 4 decimals.
fn matrix_to_quat(r: &M3) -> [f64; 4] {
    let t = r[0][0] + r[1][1] + r[2][2];
    let (w, x, y, z);
    if t > 0.0 {
        let s = (t + 1.0).sqrt() * 2.0;
        (w, x, y, z) = (s / 4.0, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s);
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        (w, x, y, z) = ((r[2][1] - r[1][2]) / s, s / 4.0, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s);
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        (w, x, y, z) = ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, s / 4.0, (r[1][2] + r[2][1]) / s);
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        (w, x, y, z) = ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4.0);
    }
    let q = if w < 0.0 { [-x, -y, -z, -w] } else { [x, y, z, w] };
    q.map(|v| round(v, 4))
}

/// The tool orientation `{x, y, z, w}` whose tool axis (−R[:,0]) is `approach`: the top-down one
/// turned by the shortest rotation from −z to `approach` (the router's `orientation_for`).
pub fn orientation_for(approach: [f64; 3]) -> [f64; 4] {
    let v = Vector3::from(approach);
    let a = if v.norm() > 1e-12 { v.normalize() } else { Vector3::new(0.0, 0.0, -1.0) };
    let down = Vector3::new(0.0, 0.0, -1.0);
    let (axis, cos) = (down.cross(&a), down.dot(&a));
    let sin = axis.norm();
    let turn: M3 = if sin < 1e-9 {
        if cos > 0.0 { [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] } else { [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]] }
    } else {
        let k = axis / sin;
        let ang = sin.atan2(cos);
        let kx: M3 = [[0.0, -k.z, k.y], [k.z, 0.0, -k.x], [-k.y, k.x, 0.0]];
        let kx2 = matmul(&kx, &kx);
        let mut m = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = if i == j { 1.0 } else { 0.0 } + ang.sin() * kx[i][j] + (1.0 - ang.cos()) * kx2[i][j];
            }
        }
        m
    };
    matrix_to_quat(&matmul(&turn, &quat_to_matrix(TOP_DOWN)))
}

fn quat_json(q: [f64; 4]) -> Value {
    json!({ "x": q[0], "y": q[1], "z": q[2], "w": q[3] })
}

/// A derived point operation in the world frame: id, position, approach.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedOp {
    pub id: String,
    pub position: [f64; 3],
    pub approach: [f64; 3],
}

/// `position` and `normal` (product frame) placed by `fixture`, rounded as the router does.
pub fn place_point(fixture: &FixtureYaml, position: [f32; 3], normal: [f32; 3]) -> ([f64; 3], [f64; 3]) {
    let pose = fixture_pose(fixture);
    let p = pose * Point3::new(position[0] as f64, position[1] as f64, position[2] as f64);
    let n = Vector3::new(normal[0] as f64, normal[1] as f64, normal[2] as f64);
    let n = if n.norm() > 1e-12 { n.normalize() } else { Vector3::new(0.0, 0.0, 1.0) };
    let a = -(pose.rotation * n);
    ([p.x, p.y, p.z].map(|v| round(v, 6)), [a.x, a.y, a.z].map(|v| round(v, 6)))
}

/// The task program for `task` of `product` placed at `fixture`, with the point operations placed
/// (`None` if the task has none), and the path operations it skipped.
pub fn derive_program(
    cell_id: &str,
    product: &Product,
    product_sha256: &str,
    fixture: &FixtureYaml,
    task: &Task,
    name: &str,
    robots: usize,
) -> (Option<(Value, Vec<PlacedOp>)>, Vec<String>) {
    let mut ops = Vec::new();
    let mut placed = Vec::new();
    let mut skipped = Vec::new();
    let mut groups: Vec<String> = Vec::new();
    let op_type = if task.process == "spotWeld" { "weld".to_owned() } else { task.process.clone() };
    for op in &task.operations {
        let Target::Point { position, normal } = &op.target else {
            skipped.push(format!("{}/{}/{} (path)", fixture.id, task.name, op.id));
            continue;
        };
        let (position, approach) = place_point(fixture, *position, *normal);
        let orientation = quat_json(orientation_for(approach));
        let group = if op.group.trim().is_empty() { "structural".to_owned() } else { op.group.clone() };
        if !groups.contains(&group) {
            groups.push(group.clone());
        }
        let priority = op.params.get("priority").and_then(Value::as_i64).unwrap_or(1);
        let mut value = json!({
            "id": op.id,
            "type": op_type,
            "position": position,
            "approach_vector": approach,
            "orientation": orientation,
            "weld_orientation": orientation,
            "sequence_group": group,
            "priority": priority,
            "description": format!("{} {} {} (Product & Task)", product.name, task.name, op.id),
        });
        let overrides: serde_json::Map<String, Value> =
            op.params.iter().filter(|(k, _)| k.as_str() != "priority").map(|(k, v)| (k.clone(), v.clone())).collect();
        if !overrides.is_empty() {
            value["process_params"] = Value::Object(overrides);
        }
        ops.push(value);
        placed.push(PlacedOp { id: op.id.clone(), position, approach });
    }
    if ops.is_empty() {
        return (None, skipped);
    }
    let mut program: Value = serde_json::from_str(TEMPLATE).expect("bundled sample task is valid JSON");
    let ext = &mut program["extensions"];
    let params = &task.params;
    ext["strategy"]["sequence_groups_order"] = params
        .get("sequence_groups_order")
        .filter(|v| v.as_array().is_some_and(|a| a.iter().all(Value::is_string)))
        .cloned()
        .unwrap_or_else(|| json!(groups));
    // All the cell's robots may weld at once unless the task says otherwise (the template's 2
    // when the scene has none).
    if let Some(v) = params.get("max_concurrent_robots").and_then(Value::as_u64) {
        ext["strategy"]["max_concurrent_robots"] = v.into();
    } else if robots > 0 {
        ext["strategy"]["max_concurrent_robots"] = (robots as u64).into();
    }
    if let Some(v) = params.get("parallel_within_group").and_then(Value::as_bool) {
        ext["strategy"]["parallel_within_group"] = v.into();
    }
    if let Some(v) = params.get("cycle_time_target_s").and_then(Value::as_f64) {
        ext["cycle_time"]["target_s"] = v.into();
    }
    if let Some(v) = params.get("cycle_time_max_s").and_then(Value::as_f64) {
        ext["cycle_time"]["max_s"] = v.into();
    }
    if task.process == "spotWeld" {
        let weld = &mut ext["weld_params"];
        if let Some(t) = params.get("weldTime").and_then(Value::as_f64) {
            weld["welding_duration_s"] = t.into();
            for phase in weld["weld_phases"].as_array_mut().into_iter().flatten() {
                if phase["phase"] == "weld" {
                    phase["duration_s"] = t.into();
                }
            }
        }
        if let Some(d) = params.get("approachOffset").and_then(Value::as_f64) {
            weld["approach_offset_z"] = d.into();
            weld["retract_offset_z"] = d.into();
        }
    } else if let Some(ext) = ext.as_object_mut() {
        ext.remove("weld_params");
    }
    ext["product_task"] = json!({
        "process": task.process,
        "tool": task.tool,
        "params": task.params,
        "skipped_paths": skipped,
    });
    let n = ops.len();
    program["program"] = name.into();
    program["cell_id"] = cell_id.into();
    program["version"] = "1.0.0".into();
    program["description"] = format!(
        "From Product & Task: product {}, task {} ({}), at fixture {}: {n} point operation{}{}, world frame by the \
         fixture's scene.yaml pose. Edit the welds in Product & Task; this file is re-derived when the product changes.",
        product.name,
        task.name,
        task.process,
        fixture.id,
        if n == 1 { "" } else { "s" },
        if skipped.is_empty() { String::new() } else { format!(" ({} path operations not derived yet)", skipped.len()) },
    )
    .into();
    program["operations"] = Value::Array(ops);
    program["source"] = json!({
        "product": product.name,
        "task": task.name,
        "fixture": fixture.id,
        "product_sha256": product_sha256,
    });
    let sha = program_sha(&program);
    program["source"]["program_sha256"] = sha.into();
    (Some((program, placed)), skipped)
}

fn read_json(path: &Path) -> Option<Value> {
    serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()
}

fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}

/// The cell's id for the planner (as `cells::planner_cell_id`).
fn cell_name(root: &Path) -> String {
    root.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
}

/// Derive cell `root`'s task programs from its products (see the module docs): refresh the
/// fixtures' product copies from `products_dir`, write `agentic_planner/tasks/<fixture>__<task>.json`
/// per product task, remove the derived ones whose task is gone, and mark the obstacle boxes the
/// welds are on `is_workpiece`. Idempotent. The cell's scene is [`cell_scene`].
#[cfg(test)]
pub fn derive_cell(root: &Path, products_dir: &Path) -> Derivation {
    derive_cell_with(root, &cell_scene(root), products_dir)
}

/// [`derive_cell`] of cell `root` (its outputs) placed by the scene `scene_path` (a reference
/// cell's `scenes/<scene>/scene.yaml`, a legacy cell's own `scene.yaml`). A reference cell's
/// fixtures are the products' own files: there is no copy to refresh.
pub fn derive_cell_with(root: &Path, scene_path: &Path, products_dir: &Path) -> Derivation {
    let mut out = Derivation::default();
    let Some(fixtures) = fixture_products(scene_path, products_dir) else { return out };
    let robots = scene_robot_count(scene_path);
    let cell_id = super::cells::planner_cell_id(&cell_name(root));
    let tasks_dir = root.join(TASKS_DIR);
    // The derived files the products make now, and the fixtures whose product could not be read
    // (their derived files stay).
    let mut wanted: BTreeSet<String> = BTreeSet::new();
    let mut unreadable: BTreeSet<String> = BTreeSet::new();
    let mut all_ops: Vec<PlacedOp> = Vec::new();
    let mut refreshed: BTreeSet<PathBuf> = BTreeSet::new();
    for fp in &fixtures {
        if let Some(original) = &fp.original
            && !refreshed.contains(&fp.copy)
            && let Ok(bytes) = std::fs::read(original)
        {
            refreshed.insert(fp.copy.clone());
            if std::fs::read(&fp.copy).ok().as_deref() != Some(bytes.as_slice()) {
                match write_atomic(&fp.copy, &bytes) {
                    Ok(()) => out.refreshed.push(fp.fixture.id.clone()),
                    Err(err) => out.warnings.push(format!("refreshing {}: {err}", fp.copy.display())),
                }
            }
        }
        let bytes = std::fs::read(&fp.copy).unwrap_or_default();
        let product: Product = match serde_json::from_slice(&bytes) {
            Ok(product) => product,
            Err(err) => {
                out.warnings.push(format!("fixture {}: {}: {err}", fp.fixture.id, fp.copy.display()));
                unreadable.insert(fp.fixture.id.clone());
                continue;
            }
        };
        let product = Product { name: if product.name.is_empty() { fp.name.clone() } else { product.name }, ..product };
        let sha = sha256_hex(&bytes);
        for (index, task) in product.tasks.iter().enumerate() {
            let name = program_name(&fp.fixture.id, &task.name, index);
            let (derived, skipped) = derive_program(&cell_id, &product, &sha, &fp.fixture, task, &name, robots);
            out.skipped.extend(skipped);
            let Some((program, ops)) = derived else {
                out.skipped.push(format!("{}/{}: no point operations", fp.fixture.id, task.name));
                continue;
            };
            all_ops.extend(ops);
            if !wanted.insert(name.clone()) {
                out.warnings.push(format!("{name}: two product tasks make the same file; the first is kept"));
                continue;
            }
            let file = tasks_dir.join(format!("{name}.json"));
            match file.exists().then(|| read_json(&file)) {
                None => {}
                Some(Some(existing)) if existing.get("source").is_some_and(Value::is_object) => {
                    if edited(&existing) {
                        out.warnings.push(format!(
                            "{name} was edited by hand: kept (not re-derived from product {}); edit the welds in Product & Task, \
                             or delete the file to derive it again",
                            product.name
                        ));
                        continue;
                    }
                    if canonical(&existing) == canonical(&program) {
                        out.unchanged.push(name);
                        continue;
                    }
                }
                Some(_) => {
                    out.warnings.push(format!(
                        "{name} is a hand-written task program (no source): kept; product {} task {} is not derived into it",
                        product.name, task.name
                    ));
                    continue;
                }
            }
            let text = serde_json::to_string_pretty(&program).expect("JSON") + "\n";
            match write_atomic(&file, text.as_bytes()) {
                Ok(()) => out.written.push(name),
                Err(err) => out.warnings.push(format!("writing {}: {err}", file.display())),
            }
        }
    }
    // Derived files whose product task or fixture is gone.
    for (stem, path, program) in task_files(&tasks_dir) {
        let Some(source) = program.as_ref().and_then(|p| p.get("source")).filter(|s| s["product"].is_string()) else {
            continue;
        };
        let fixture = source["fixture"].as_str().unwrap_or_default();
        if wanted.contains(&stem) || unreadable.contains(fixture) {
            continue;
        }
        if program.as_ref().is_some_and(edited) {
            out.warnings.push(format!("{stem} was edited by hand and its product task is gone: kept"));
            continue;
        }
        match std::fs::remove_file(&path) {
            Ok(()) => out.removed.push(stem),
            Err(err) => out.warnings.push(format!("removing {}: {err}", path.display())),
        }
    }
    let obstacles = root.join(OBSTACLES_YAML);
    if !all_ops.is_empty()
        && let Ok(text) = std::fs::read_to_string(&obstacles)
    {
        let points: Vec<[f64; 3]> = all_ops.iter().map(|o| o.position).collect();
        let mark = workpiece_ids(&obstacle_boxes(&text), &points);
        if !mark.is_empty() {
            match write_atomic(&obstacles, mark_workpieces(&text, &mark).as_bytes()) {
                Ok(()) => out.workpieces = mark.into_iter().collect(),
                Err(err) => out.warnings.push(format!("marking workpieces in {}: {err}", obstacles.display())),
            }
        }
    }
    out
}

/// The `*.json` files of `dir` by stem, parsed (`None`: not JSON).
fn task_files(dir: &Path) -> Vec<(String, PathBuf, Option<Value>)> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .filter_map(|p| Some((p.file_stem()?.to_string_lossy().into_owned(), p.clone(), read_json(&p))))
        .collect();
    files.sort_by(|a, b| a.0.cmp(&b.0));
    files
}

/// Where each of cell `root`'s task programs `tasks` (file stems) comes from (its scene: [`cell_scene`]).
#[cfg(test)]
pub fn task_sources(root: &Path, products_dir: &Path, tasks: &[String]) -> Vec<TaskSource> {
    task_sources_with(root, &cell_scene(root), products_dir, tasks)
}

/// [`task_sources`], the cell placed by the scene `scene_path`.
pub fn task_sources_with(root: &Path, scene_path: &Path, products_dir: &Path, tasks: &[String]) -> Vec<TaskSource> {
    let fixtures = fixture_products(scene_path, products_dir).unwrap_or_default();
    let current_sha = |fixture: &str| {
        let fp = fixtures.iter().find(|fp| fp.fixture.id == fixture)?;
        let file = fp.original.as_ref().unwrap_or(&fp.copy);
        std::fs::read(file).ok().map(|b| sha256_hex(&b))
    };
    tasks
        .iter()
        .map(|file| {
            let program = read_json(&root.join(TASKS_DIR).join(format!("{file}.json")));
            let source = program.as_ref().and_then(|p| p.get("source")).filter(|s| s.is_object());
            let Some(source) = source else {
                return TaskSource { file: file.clone(), ..TaskSource::default() };
            };
            let text = |key: &str| source[key].as_str().map(str::to_owned);
            let fixture = text("fixture");
            let sha = fixture.as_deref().and_then(current_sha);
            TaskSource {
                file: file.clone(),
                product: text("product"),
                task: text("task"),
                stale: sha.is_none() || sha.as_deref() != source["product_sha256"].as_str(),
                edited: program.as_ref().is_some_and(edited),
                fixture,
            }
        })
        .collect()
}

/// Whether cell `root` places product `name` as a fixture (its scene: [`cell_scene`]).
#[cfg(test)]
pub fn uses_product(root: &Path, products_dir: &Path, name: &str) -> bool {
    uses_product_in(&cell_scene(root), products_dir, name)
}

/// Whether the scene `scene_path` places product `name` as a fixture.
pub fn uses_product_in(scene_path: &Path, products_dir: &Path, name: &str) -> bool {
    fixture_products(scene_path, products_dir).is_some_and(|f| f.iter().any(|fp| fp.name == name))
}

/// A box obstacle of `obstacles.yaml`.
#[derive(Clone, Debug, PartialEq)]
pub struct ObstacleBox {
    pub id: String,
    pub center: [f64; 3],
    pub half_extents: [f64; 3],
    pub is_workpiece: bool,
}

impl ObstacleBox {
    /// How deep `p` is inside the (axis-aligned) box: > 0 inside, ≤ 0 on or outside.
    pub fn depth(&self, p: [f64; 3]) -> f64 {
        (0..3).map(|i| self.half_extents[i] - (p[i] - self.center[i]).abs()).fold(f64::INFINITY, f64::min)
    }
}

/// The fixture of a voxel-shell box id: the converter's coarse `<fixture>_v<n>` or its near-weld
/// refinement's fine `<fixture>_f<n>`.
fn shell_of(id: &str) -> Option<&str> {
    shell_kind(id).map(|(fixture, _)| fixture)
}

/// [`shell_of`], and whether the box is of the fine shell.
fn shell_kind(id: &str) -> Option<(&str, bool)> {
    let at = id.rfind(['v', 'f'])?;
    let (fixture, n) = (id[..at].strip_suffix('_')?, &id[at + 1..]);
    (!fixture.is_empty() && !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit())).then_some((fixture, id.as_bytes()[at] == b'f'))
}

/// How far from a weld the boxes of its fixture's shell `id` is in are workpiece: the coarse
/// shell within [`WORKPIECE_NEAR_M`], the fine one within [`REFINE_RADIUS_M`].
fn shell_near_m(id: &str) -> Option<f64> {
    shell_kind(id).map(|(_, fine)| if fine { REFINE_RADIUS_M } else { WORKPIECE_NEAR_M })
}

/// The ids of the boxes (not yet `is_workpiece`) the welds at `points` are on: within
/// [`WORKPIECE_TOL_M`] of one, and of the same fixture's voxel shell within [`WORKPIECE_NEAR_M`]
/// (coarse `_v` boxes) or [`REFINE_RADIUS_M`] (fine `_f` boxes). As the router's `workpiece_ids`.
fn workpiece_ids(boxes: &[ObstacleBox], points: &[[f64; 3]]) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for &p in points {
        let shells: BTreeSet<&str> = boxes.iter().filter(|b| b.depth(p) >= -WORKPIECE_TOL_M).filter_map(|b| shell_of(&b.id)).collect();
        let near = |b: &ObstacleBox| {
            shell_of(&b.id).is_some_and(|s| shells.contains(s)) && shell_near_m(&b.id).is_some_and(|near| b.depth(p) >= -near)
        };
        out.extend(
            boxes
                .iter()
                .filter(|b| !b.is_workpiece && (b.depth(p) >= -WORKPIECE_TOL_M || near(b)))
                .map(|b| b.id.clone()),
        );
    }
    out
}

pub fn obstacle_boxes(text: &str) -> Vec<ObstacleBox> {
    let Ok(doc) = serde_yaml::from_str::<serde_yaml::Value>(text) else { return Vec::new() };
    let vec3 = |v: &serde_yaml::Value| -> Option<[f64; 3]> {
        let s = v.as_sequence().filter(|s| s.len() == 3)?;
        Some([s[0].as_f64()?, s[1].as_f64()?, s[2].as_f64()?])
    };
    doc["obstacles"]
        .as_sequence()
        .into_iter()
        .flatten()
        .filter(|o| o["type"].as_str().unwrap_or("box") == "box")
        .filter_map(|o| {
            let id = match &o["id"] {
                serde_yaml::Value::String(s) => s.clone(),
                serde_yaml::Value::Number(n) => n.to_string(),
                _ => String::new(),
            };
            Some(ObstacleBox {
                id,
                center: vec3(&o["center"])?,
                half_extents: vec3(&o["half_extents"])?.map(f64::abs),
                is_workpiece: o["is_workpiece"].as_bool() == Some(true),
            })
        })
        .collect()
}

fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start_matches(' ').len()
}

/// The obstacle id of a `- id: <id>` line, and its indent.
fn item_id(line: &str) -> Option<(usize, String)> {
    let body = line.trim_start_matches([' ', '\t']);
    let rest = body.strip_prefix('-')?;
    let rest = rest.strip_prefix(char::is_whitespace)?.trim_start();
    let value = rest.strip_prefix("id:")?;
    let value = value.split('#').next().unwrap_or_default().trim().trim_matches(['"', '\'']);
    (!value.is_empty() && !value.contains(char::is_whitespace)).then(|| (line.len() - body.len(), value.to_owned()))
}

/// `obstacles.yaml` with `is_workpiece: true` on the obstacles `ids`, text-edited so comments and
/// layout stay (as the router's `mark_workpieces`).
pub fn mark_workpieces(text: &str, ids: &BTreeSet<String>) -> String {
    const NOTE: &str = "   # Product & Task: the product's welds are on it (tool tip may touch)";
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    let mut out = String::new();
    let mut i = 0;
    while i < lines.len() {
        let Some((indent, _)) = item_id(lines[i].trim_end_matches('\n')).filter(|(_, id)| ids.contains(id)) else {
            out.push_str(lines[i]);
            i += 1;
            continue;
        };
        let mut j = i + 1;
        while j < lines.len() {
            let l = lines[j].trim_end_matches('\n');
            let blank_or_comment = l.trim().is_empty() || l.trim_start().starts_with('#');
            if !(blank_or_comment || indent_of(l) > indent) || item_id(l).is_some() {
                break;
            }
            j += 1;
        }
        let mut block: Vec<String> = lines[i..j].iter().map(|s| (*s).to_owned()).collect();
        let last = (0..block.len())
            .rev()
            .find(|&k| {
                let t = block[k].trim();
                !t.is_empty() && !t.starts_with('#')
            })
            .unwrap_or(0);
        if let Some(k) = block.iter().position(|l| l.trim_start().starts_with("is_workpiece:")) {
            let pad = &block[k][..block[k].len() - block[k].trim_start().len()];
            block[k] = format!("{pad}is_workpiece: true{NOTE}\n");
        } else {
            if !block[last].ends_with('\n') {
                block[last].push('\n');
            }
            block[last].push_str(&format!("{}is_workpiece: true{NOTE}\n", " ".repeat(indent + 2)));
        }
        out.extend(block);
        i = j;
    }
    out
}

#[cfg(test)]
#[allow(clippy::approx_constant)]
mod tests {
    use super::*;

    #[test]
    fn marks_the_welded_fixtures_voxels_near_a_weld() {
        let b = |id: &str, center: [f64; 3]| ObstacleBox { id: id.into(), center, half_extents: [0.025; 3], is_workpiece: false };
        let boxes = [
            b("door_v0001", [0.0, 0.0, 0.0]),  // the weld is on it
            b("door_v0002", [0.1, 0.0, 0.0]),  // a neighbour of the same shell
            b("door_v0003", [0.5, 0.0, 0.0]),  // same shell, far
            b("clamp_v0001", [0.0, 0.1, 0.0]), // another fixture, near: an obstacle still
            b("table", [0.0, 0.0, -0.1]),
        ];
        let ids: Vec<String> = workpiece_ids(&boxes, &[[0.0, 0.0, 0.025]]).into_iter().collect();
        assert_eq!(ids, ["door_v0001", "door_v0002"]);
        assert_eq!(shell_of("body_weld_1_v0233"), Some("body_weld_1"));
        assert_eq!(shell_of("floor"), None);
    }

    const OBSTACLES: &str = "# Static collision obstacles\nobstacles:\n\n  # Cell floor\n  - id: floor\n    type: plane\n    \
        normal: [0.0, 0.0, 1.0]\n    offset: -0.05\n\n  - id: bug_cube_v0\n    type: box\n    center: [0.025, 0.025, 0.025]\n    \
        half_extents: [1.025, 1.025, 0.025]\n  - id: bug_cube_v3\n    type: box\n    description: \"top\"\n    \
        center: [0.05, 0.05, 2.025]\n    half_extents: [1, 1, 0.025]\n    collision_margin: 0.05\n\n  - id: pedestal_robot1\n    \
        type: box\n    center: [2.5, 0.0, 0.4]\n    half_extents: [0.75, 0.75, 0.4]\n\ndefault_collision_margin: 0.05\n";

    /// With the near-weld refinement the weld is on the fine shell (`<fixture>_f<n>`): the welded
    /// fixture's fine boxes within REFINE_RADIUS_M are workpiece, another fixture's are not, and
    /// the coarse rule is unchanged.
    #[test]
    fn marks_the_welded_fixtures_fine_shell_within_the_refine_radius() {
        let b = |id: &str, center: [f64; 3], h: f64| ObstacleBox { id: id.into(), center, half_extents: [h; 3], is_workpiece: false };
        let boxes = [
            b("door_f12", [0.0, 0.0, 0.0], 0.01),    // the weld is on it
            b("door_f13", [0.2, 0.0, 0.0], 0.01),    // same fine shell, 0.19 m away
            b("door_f14", [0.3, 0.0, 0.0], 0.01),    // beyond the radius
            b("door_v0007", [0.0, 0.2, 0.0], 0.025), // coarse, 0.175 m: beyond WORKPIECE_NEAR_M
            b("door_v0008", [0.0, 0.1, 0.0], 0.025), // coarse, within it
            b("clamp_f1", [0.0, -0.05, 0.0], 0.01),  // another fixture's fine box
        ];
        let ids: Vec<String> = workpiece_ids(&boxes, &[[0.0, 0.0, 0.01]]).into_iter().collect();
        assert_eq!(ids, ["door_f12", "door_f13", "door_v0008"]);
        assert_eq!(shell_kind("body_weld_1_f233"), Some(("body_weld_1", true)));
        assert_eq!(shell_kind("body_weld_1_v0233"), Some(("body_weld_1", false)));
        assert_eq!(shell_of("floor"), None);
        assert_eq!(shell_of("door_f"), None);
        assert_eq!(shell_of("_f1"), None);
        assert_eq!(shell_of("x_fv1"), None);
    }

    /// A derived program lets all the scene's robots weld at once unless the task says otherwise.
    #[test]
    fn derived_programs_default_to_all_the_scenes_robots() {
        let cell = Cell::new("robots", 0.0, 0.0, 0.0);
        let robot = |i: usize| format!("- robot_id: r{i}\n  dressed_urdf: robots/r/r.urdf\n  position: {{x: {i}.0, y: 0.0, z: 0.0}}\n");
        let scene = std::fs::read_to_string(cell.root().join("scene.yaml")).unwrap();
        let four: String = (1..=4).map(robot).collect();
        std::fs::write(cell.root().join("scene.yaml"), scene.replace("robots: []\n", &format!("robots:\n{four}"))).unwrap();
        cell.derive();
        assert_eq!(cell.program(NAME)["extensions"]["strategy"]["max_concurrent_robots"], json!(4));
        cell.save_product(|p| p["tasks"][0]["params"]["max_concurrent_robots"] = json!(3));
        cell.derive();
        assert_eq!(cell.program(NAME)["extensions"]["strategy"]["max_concurrent_robots"], json!(3), "the task's own value wins");
        // No robots in the scene: the template's 2.
        let other = Cell::new("no-robots", 0.0, 0.0, 0.0);
        other.derive();
        assert_eq!(other.program(NAME)["extensions"]["strategy"]["max_concurrent_robots"], json!(2));
    }

    /// The weld points for the converter's refinement, before anything is derived: the same world
    /// positions the task program gets.
    #[test]
    fn weld_points_are_the_derived_positions() {
        let cell = Cell::new("weld-points", 1.0, 2.0, 90.0);
        let points = weld_points(&cell.root().join("scene.yaml"), &cell.products());
        cell.derive();
        let ops = cell.program(NAME)["operations"].as_array().unwrap().clone();
        let derived: Vec<[f64; 3]> = ops.iter().map(|o| serde_json::from_value(o["position"].clone()).unwrap()).collect();
        assert_eq!(points, derived);
        assert_eq!(points.len(), 4);
    }

    fn jam2_product() -> String {
        std::fs::read_to_string(super::super::reach::task::tests::fixtures().join("bug_cube/product.json")).unwrap()
    }

    struct Cell {
        dir: PathBuf,
    }

    impl Cell {
        /// A jam2-like cell `jam2` in a temp folder, its bug_cube fixture at (x, y, 0) turned `a`°,
        /// and a products folder with bug_cube.
        fn new(tag: &str, x: f64, y: f64, a: f64) -> Self {
            let dir = std::env::temp_dir().join(format!("factoryos-product-tasks-{tag}-{}", std::process::id()));
            std::fs::remove_dir_all(&dir).ok();
            let cell = Self { dir };
            std::fs::create_dir_all(cell.products().join("bug_cube")).unwrap();
            std::fs::write(cell.products().join("bug_cube/product.json"), jam2_product()).unwrap();
            std::fs::create_dir_all(cell.root().join("fixtures/bug_cube")).unwrap();
            std::fs::write(cell.root().join("fixtures/bug_cube/bug_cube.urdf"), "<robot/>").unwrap();
            cell.place(x, y, a);
            std::fs::create_dir_all(cell.root().join("agentic_planner/config")).unwrap();
            std::fs::write(cell.root().join(OBSTACLES_YAML), OBSTACLES).unwrap();
            cell
        }
        fn place(&self, x: f64, y: f64, a: f64) {
            std::fs::write(
                self.root().join("scene.yaml"),
                format!(
                    "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n  \
                     position: {{x: {x}, y: {y}, z: 0.0}}\n  orientation: {{a: {a}, b: 0.0, c: 0.0}}\n"
                ),
            )
            .unwrap();
        }
        fn root(&self) -> PathBuf {
            self.dir.join("cells/jam2")
        }
        fn products(&self) -> PathBuf {
            self.dir.join("products")
        }
        fn derive(&self) -> Derivation {
            derive_cell(&self.root(), &self.products())
        }
        fn program(&self, name: &str) -> Value {
            read_json(&self.root().join(TASKS_DIR).join(format!("{name}.json"))).unwrap()
        }
        fn save_product(&self, edit: impl FnOnce(&mut Value)) {
            let file = self.products().join("bug_cube/product.json");
            let mut product = read_json(&file).unwrap();
            edit(&mut product);
            std::fs::write(file, serde_json::to_string_pretty(&product).unwrap()).unwrap();
        }
        fn sources(&self) -> Vec<TaskSource> {
            let tasks: Vec<String> = task_files(&self.root().join(TASKS_DIR)).into_iter().map(|(s, ..)| s).collect();
            task_sources(&self.root(), &self.products(), &tasks)
        }
    }

    impl Drop for Cell {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.dir).ok();
        }
    }

    const NAME: &str = "bug_cube__spot_weld_1";

    #[test]
    fn orientations_match_the_router() {
        // deploy/planner-router: orientation_for
        assert_eq!(orientation_for([0.0, 0.0, -1.0]), TOP_DOWN);
        assert_eq!(orientation_for([0.0, -1.0, 0.0]), [-0.5, -0.5, 0.5, 0.5]);
        assert_eq!(orientation_for([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0, 0.0]);
        assert_eq!(orientation_for([0.0, 0.0, 1.0]), [-0.7071, 0.0, 0.7071, 0.0]);
    }

    #[test]
    fn jam2_cell_derives_its_product_task() {
        let cell = Cell::new("jam2", 0.0, 0.0, 0.0);
        let d = cell.derive();
        assert_eq!(d.written, [NAME]);
        assert_eq!(d.workpieces, ["bug_cube_v3"], "the top shell the welds are on");
        let p = cell.program(NAME);
        assert_eq!(p["program"], NAME);
        assert_eq!(p["cell_id"], "jam2");
        let ops = p["operations"].as_array().unwrap();
        let positions: Vec<Value> = ops.iter().map(|o| o["position"].clone()).collect();
        assert_eq!(positions, [json!([-0.5, -0.5, 2.0]), json!([0.5, -0.5, 2.0]), json!([-0.5, 0.5, 2.0]), json!([0.5, 0.5, 2.0])]);
        assert_eq!(ops.iter().map(|o| o["id"].as_str().unwrap()).collect::<Vec<_>>(), ["W01", "W02", "W03", "W04"]);
        for o in ops {
            assert_eq!(o["approach_vector"], json!([0.0, 0.0, -1.0]));
            assert_eq!(o["orientation"], json!({"x": 0.0, "y": -0.7071, "z": 0.0, "w": 0.7071}));
            assert_eq!(o["weld_orientation"], o["orientation"]);
            assert_eq!((o["type"].as_str(), o["sequence_group"].as_str(), o["priority"].as_i64()), (Some("weld"), Some("structural"), Some(1)));
        }
        let ext = &p["extensions"];
        assert_eq!(ext["strategy"]["sequence_groups_order"], json!(["structural"]));
        assert_eq!(ext["weld_params"]["welding_duration_s"], json!(0.3));
        assert_eq!(ext["weld_params"]["approach_offset_z"], json!(0.05));
        assert_eq!(ext["product_task"]["process"], "spotWeld");
        let sha = sha256_hex(&std::fs::read(cell.root().join("fixtures/bug_cube/product.json")).unwrap());
        assert_eq!(p["source"]["product"], "bug_cube");
        assert_eq!(p["source"]["task"], "spot_weld_1");
        assert_eq!(p["source"]["fixture"], "bug_cube");
        assert_eq!(p["source"]["product_sha256"], sha);
        assert_eq!(d.refreshed, ["bug_cube"], "the cell got the product's copy");
        let obstacles = std::fs::read_to_string(cell.root().join(OBSTACLES_YAML)).unwrap();
        assert_eq!(obstacles.matches("is_workpiece: true").count(), 1);
        assert!(obstacles.contains("    collision_margin: 0.05\n    is_workpiece: true"), "{obstacles}");
        assert!(obstacles.contains("# Static collision obstacles"));
        let boxes = obstacle_boxes(&obstacles);
        assert_eq!(boxes.iter().filter(|b| b.is_workpiece).map(|b| b.id.as_str()).collect::<Vec<_>>(), ["bug_cube_v3"]);
        assert_eq!(
            cell.sources(),
            [TaskSource {
                file: NAME.into(),
                product: Some("bug_cube".into()),
                task: Some("spot_weld_1".into()),
                fixture: Some("bug_cube".into()),
                stale: false,
                edited: false
            }]
        );
    }

    /// A reference cell: no scene.yaml or fixtures/ of its own; its scene is the Robo Fitter's
    /// `scenes/jam2/scene.yaml`, its product the products folder's own file (never written).
    #[test]
    fn a_reference_cell_derives_from_its_fitted_scene_and_the_products_folder() {
        let cell = Cell::new("reference", 0.0, 0.0, 0.0);
        std::fs::remove_file(cell.root().join("scene.yaml")).unwrap();
        std::fs::remove_dir_all(cell.root().join("fixtures")).unwrap();
        std::fs::write(cell.root().join("factoryos-cell.json"), r#"{"format": 2, "kind": "reference", "fitted_scene": "jam2.yaml"}"#).unwrap();
        let fitted = cell.dir.join("assets/scenes");
        std::fs::create_dir_all(fitted.join("jam2")).unwrap();
        std::fs::write(cell.products().join("bug_cube/bug_cube.urdf"), "<robot/>").unwrap();
        let scene = fitted.join("jam2/scene.yaml");
        std::fs::write(
            &scene,
            "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: ../../../products/bug_cube/bug_cube.urdf\n  \
             position: {x: 0.0, y: 0.0, z: 0.0}\n  orientation: {a: 0.0, b: 0.0, c: 0.0}\n",
        )
        .unwrap();
        assert_eq!(derive_cell(&cell.root(), &cell.products()), Derivation::default(), "the scene is not the cell's");
        let product = std::fs::read(cell.products().join("bug_cube/product.json")).unwrap();
        let d = derive_cell_with(&cell.root(), &scene, &cell.products());
        assert_eq!(d.written, [NAME]);
        assert!(d.refreshed.is_empty(), "no copy to refresh");
        assert_eq!(std::fs::read(cell.products().join("bug_cube/product.json")).unwrap(), product, "the product is not written");
        assert!(!cell.root().join("fixtures").exists());
        let p = cell.program(NAME);
        assert_eq!(p["source"]["product_sha256"], sha256_hex(&product));
        assert_eq!(p["operations"][0]["position"], json!([-0.5, -0.5, 2.0]));
        let tasks = [NAME.to_owned()];
        let sources = task_sources_with(&cell.root(), &scene, &cell.products(), &tasks);
        assert_eq!((sources[0].product.as_deref(), sources[0].stale), (Some("bug_cube"), false));
        assert!(uses_product_in(&scene, &cell.products(), "bug_cube"));
        // The product's welds change: the program is stale until derived again.
        cell.save_product(|p| p["tasks"][0]["operations"][0]["target"]["position"] = json!([-0.25, -0.5, 2.0]));
        assert!(task_sources_with(&cell.root(), &scene, &cell.products(), &tasks)[0].stale);
        assert_eq!(derive_cell_with(&cell.root(), &scene, &cell.products()).written, [NAME]);
        assert_eq!(cell.program(NAME)["operations"][0]["position"], json!([-0.25, -0.5, 2.0]));
        // Through its view, too (what the defaults read when the caller has no scene).
        let view = cell.root().join(".view");
        std::fs::create_dir_all(view.join("fixtures")).unwrap();
        std::os::unix::fs::symlink(cell.products().join("bug_cube"), view.join("fixtures/bug_cube")).unwrap();
        std::fs::write(
            view.join("scene.yaml"),
            "name: jam2\nrobots: []\nfixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n  \
             position: {x: 0.0, y: 0.0, z: 0.0}\n  orientation: {a: 0.0, b: 0.0, c: 0.0}\n",
        )
        .unwrap();
        assert_eq!(cell_scene(&cell.root()), view.join("scene.yaml"));
        assert!(uses_product(&cell.root(), &cell.products(), "bug_cube"));
        assert_eq!(derive_cell(&cell.root(), &cell.products()).unchanged, [NAME]);
    }

    #[test]
    fn a_turned_fixture_turns_the_welds() {
        let cell = Cell::new("turned", 1.0, 2.0, 90.0);
        cell.save_product(|p| {
            let ops = p["tasks"][0]["operations"].as_array_mut().unwrap();
            ops.push(json!({"id": "S01", "group": "side", "params": {"priority": 2, "force": 4.0},
                "target": {"kind": "point", "position": [1.0, 0.0, 1.0], "normal": [1.0, 0.0, 0.0]}}));
            ops.push(json!({"id": "P01", "target": {"kind": "path", "points": [[0, 0, 2], [0.5, 0, 2]], "normal": [0, 0, 1]}}));
            p["tasks"][0]["params"]["sequence_groups_order"] = json!(["side", "structural"]);
            p["tasks"][0]["params"]["max_concurrent_robots"] = json!(1);
            p["tasks"][0]["params"]["cycle_time_target_s"] = json!(12.5);
        });
        let d = cell.derive();
        assert_eq!(d.skipped, ["bug_cube/spot_weld_1/P01 (path)"]);
        let p = cell.program(NAME);
        let op = |id: &str| p["operations"].as_array().unwrap().iter().find(|o| o["id"] == id).unwrap().clone();
        // Rz(90): (x, y) -> (-y, x), then moved by (1, 2): the router's numbers.
        assert_eq!(op("W02")["position"], json!([1.5, 2.5, 2.0]));
        assert_eq!(op("W02")["approach_vector"], json!([0.0, 0.0, -1.0]));
        assert_eq!(op("S01")["position"], json!([1.0, 3.0, 1.0]));
        assert_eq!(op("S01")["approach_vector"], json!([0.0, -1.0, 0.0]));
        assert_eq!(op("S01")["orientation"], json!({"x": -0.5, "y": -0.5, "z": 0.5, "w": 0.5}));
        assert_eq!((op("S01")["sequence_group"].as_str(), op("S01")["priority"].as_i64()), (Some("side"), Some(2)));
        assert_eq!(op("S01")["process_params"], json!({"force": 4.0}));
        assert!(op("W02").get("process_params").is_none());
        assert_eq!(p["extensions"]["strategy"]["sequence_groups_order"], json!(["side", "structural"]));
        assert_eq!(p["extensions"]["strategy"]["max_concurrent_robots"], json!(1));
        assert_eq!(p["extensions"]["cycle_time"]["target_s"], json!(12.5));
        assert_eq!(p["extensions"]["product_task"]["skipped_paths"], json!(["bug_cube/spot_weld_1/P01 (path)"]));
        assert_eq!(p["operations"].as_array().unwrap().len(), 5);
    }

    #[test]
    fn deriving_again_changes_nothing() {
        let cell = Cell::new("idem", 0.0, 0.0, 0.0);
        cell.derive();
        let file = cell.root().join(TASKS_DIR).join(format!("{NAME}.json"));
        let before = (std::fs::read(&file).unwrap(), std::fs::metadata(&file).unwrap().modified().unwrap());
        let obstacles = std::fs::read_to_string(cell.root().join(OBSTACLES_YAML)).unwrap();
        let d = cell.derive();
        assert!(!d.changed(), "{d:?}");
        assert_eq!(d.unchanged, [NAME]);
        assert_eq!(before, (std::fs::read(&file).unwrap(), std::fs::metadata(&file).unwrap().modified().unwrap()));
        assert_eq!(std::fs::read_to_string(cell.root().join(OBSTACLES_YAML)).unwrap(), obstacles);
        // Moving the fixture re-derives it.
        cell.place(1.0, 0.0, 0.0);
        assert_eq!(cell.derive().written, [NAME]);
        assert_eq!(cell.program(NAME)["operations"][0]["position"], json!([0.5, -0.5, 2.0]));
    }

    #[test]
    fn hand_written_and_edited_programs_stay() {
        let cell = Cell::new("hand", 0.0, 0.0, 0.0);
        let tasks = cell.root().join(TASKS_DIR);
        std::fs::create_dir_all(&tasks).unwrap();
        // A hand-written program with the derived name, and another one.
        let hand = r#"{"program": "mine", "operations": [{"id": "X", "position": [0, 0, 3]}]}"#;
        std::fs::write(tasks.join(format!("{NAME}.json")), hand).unwrap();
        std::fs::write(tasks.join("bug_cube_spot_welding.json"), hand).unwrap();
        let d = cell.derive();
        assert!(d.written.is_empty());
        assert!(d.warnings[0].contains("hand-written"), "{:?}", d.warnings);
        assert_eq!(std::fs::read_to_string(tasks.join(format!("{NAME}.json"))).unwrap(), hand);
        assert_eq!(std::fs::read_to_string(tasks.join("bug_cube_spot_welding.json")).unwrap(), hand);
        assert!(cell.sources().iter().all(|s| s.product.is_none() && !s.edited && !s.stale));

        // Derived, then edited by hand: kept, and stale once the product changes.
        std::fs::remove_file(tasks.join(format!("{NAME}.json"))).unwrap();
        cell.derive();
        let file = tasks.join(format!("{NAME}.json"));
        // Reformatting alone is no edit.
        std::fs::write(&file, serde_json::to_string(&cell.program(NAME)).unwrap()).unwrap();
        assert!(cell.sources().iter().all(|s| !s.edited), "{:?}", cell.sources());
        let mut p = cell.program(NAME);
        p["operations"][0]["priority"] = json!(5);
        let edited_text = serde_json::to_string_pretty(&p).unwrap();
        std::fs::write(&file, &edited_text).unwrap();
        let d = cell.derive();
        assert!(d.warnings.iter().any(|w| w.contains("edited by hand")), "{d:?}");
        assert_eq!(std::fs::read_to_string(&file).unwrap(), edited_text);
        let derived = cell.sources().into_iter().find(|s| s.file == NAME).unwrap();
        assert!(derived.edited && !derived.stale, "{derived:?}");
        cell.save_product(|p| p["tasks"][0]["params"]["weldTime"] = json!(0.4));
        cell.derive();
        assert_eq!(std::fs::read_to_string(&file).unwrap(), edited_text);
        let derived = cell.sources().into_iter().find(|s| s.file == NAME).unwrap();
        assert!(derived.edited && derived.stale, "{derived:?}");
        assert_eq!(std::fs::read_to_string(tasks.join("bug_cube_spot_welding.json")).unwrap(), hand);
    }

    #[test]
    fn saving_the_product_updates_the_cell() {
        let cell = Cell::new("save", 0.0, 0.0, 0.0);
        cell.derive();
        assert!(uses_product(&cell.root(), &cell.products(), "bug_cube"));
        assert!(!uses_product(&cell.root(), &cell.products(), "other"));
        // A moved weld, and a second task.
        cell.save_product(|p| {
            p["tasks"][0]["operations"][0]["target"]["position"] = json!([-0.25, -0.5, 2.0]);
            let mut second = p["tasks"][0].clone();
            second["name"] = json!("spot weld 2");
            p["tasks"].as_array_mut().unwrap().push(second);
        });
        assert!(cell.sources()[0].stale, "the product changed, the cell not yet");
        let d = cell.derive();
        assert_eq!(d.refreshed, ["bug_cube"]);
        assert_eq!(d.written, [NAME, "bug_cube__spot_weld_2"]);
        assert_eq!(cell.program(NAME)["operations"][0]["position"], json!([-0.25, -0.5, 2.0]));
        let copy = std::fs::read(cell.root().join("fixtures/bug_cube/product.json")).unwrap();
        assert_eq!(copy, std::fs::read(cell.products().join("bug_cube/product.json")).unwrap());
        assert_eq!(cell.program(NAME)["source"]["product_sha256"], sha256_hex(&copy));
        assert!(cell.sources().iter().all(|s| !s.stale && !s.edited));
        // The second task removed again: its derived file goes.
        cell.save_product(|p| {
            p["tasks"].as_array_mut().unwrap().pop();
        });
        assert_eq!(cell.derive().removed, ["bug_cube__spot_weld_2"]);
        assert_eq!(cell.sources().len(), 1);
    }

    #[test]
    fn file_names_of_any_fixture_and_task() {
        assert_eq!(program_name("my cube", "a/b", 0), "my_cube__a_b");
        assert_eq!(program_name("f", " ", 2), "f__task3");
    }
}
