//! The cell's collision environment for the scene collision checker, as boxes:
//!
//! * **Fixtures** (`scene.yaml` `fixtures:`): the boxes of the planner's
//!   `obstacles.yaml` (what Prepare Cell writes, `agentic_planner/config/obstacles.yaml`, or the
//!   scene's `obstacles:` path), which also say which box is the workpiece (`is_workpiece`). A
//!   fixture that file does not cover (no box with its id, or `<id>_…`) gets one box per
//!   `<collision>` of its URDF (box, cylinder and sphere primitives, and the bounding box of a
//!   mesh). Other `obstacles.yaml` boxes and spheres are obstacles too.
//! * **Pedestals** (`scene.yaml` `robots[].pedestal: {height_mm, diameter_mm}`): the cylinder's
//!   bounding box from the floor up to the robot base. `obstacles.yaml`'s own pedestal boxes are
//!   lowered for the planner's base spheres, so they are replaced by these.
//! * **The floor**: a slab whose top is z = 0.
//!
//! Plus each robot's base pose: the scene position raised by its pedestal, turned by the scene
//! orientation (KUKA A/B/C, degrees, about z/y/x).
//!
//! Everything here is never-fail-hard: a missing or malformed file or entry is logged and
//! skipped.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use crate::cell::RobotState;
use crate::collision::{EnvironmentBoxSpec, EnvironmentRole};

/// Where Prepare Cell writes the planner's obstacles, relative to the cell directory.
pub const OBSTACLES_YAML: &str = "agentic_planner/config/obstacles.yaml";
/// The Robo Fitter's pedestal diameter when the scene gives only a height.
const DEFAULT_PEDESTAL_DIAMETER_MM: f64 = 800.0;
/// Half the floor slab's size across (metres) and its half-thickness.
const FLOOR_HALF_SIZE_M: f64 = 100.0;
const FLOOR_HALF_THICKNESS_M: f64 = 0.05;

/// A pedestal a robot stands on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pedestal {
    pub height_m: f64,
    pub diameter_m: f64,
}

/// Where a cell robot stands.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotBase {
    /// `world <- robot base` (the URDF root): position + pedestal height, scene orientation.
    pub pose: Isometry3<f64>,
    pub pedestal: Option<Pedestal>,
}

/// KUKA A/B/C (degrees; intrinsic Z-Y-X) as a rotation.
fn kuka_rotation(abc: [f64; 3]) -> UnitQuaternion<f64> {
    let [a, b, c] = abc;
    UnitQuaternion::from_euler_angles(c.to_radians(), b.to_radians(), a.to_radians())
}

fn read_yaml(path: &Path) -> Option<serde_yaml::Value> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) => {
            eprintln!("WARN: collision: cannot read {} ({e})", path.display());
            return None;
        }
    };
    match serde_yaml::from_str(&text) {
        Ok(doc) => Some(doc),
        Err(e) => {
            eprintln!("WARN: collision: cannot parse {} ({e})", path.display());
            None
        }
    }
}

fn num(value: Option<&serde_yaml::Value>) -> Option<f64> {
    value.and_then(serde_yaml::Value::as_f64).filter(|v| v.is_finite())
}

fn xyz(value: Option<&serde_yaml::Value>, keys: [&str; 3]) -> Option<[f64; 3]> {
    let map = value?;
    Some([num(map.get(keys[0]))?, num(map.get(keys[1]))?, num(map.get(keys[2]))?])
}

fn triple(value: Option<&serde_yaml::Value>) -> Option<[f64; 3]> {
    let seq = value?.as_sequence()?;
    match seq.as_slice() {
        [x, y, z] => Some([num(Some(x))?, num(Some(y))?, num(Some(z))?]),
        _ => None,
    }
}

/// The scene pose of an entry: `position {x,y,z}` (required) and `orientation {a,b,c}` (default 0).
fn scene_pose(entry: &serde_yaml::Value) -> Option<([f64; 3], UnitQuaternion<f64>)> {
    let position = xyz(entry.get("position"), ["x", "y", "z"])?;
    let orientation = match entry.get("orientation") {
        None => [0.0; 3],
        Some(o) => xyz(Some(o), ["a", "b", "c"])?,
    };
    Some((position, kuka_rotation(orientation)))
}

/// Every cell robot's base, from `scene.yaml` (looked up by `scene_id`, else `robot_id`). Robots
/// the scene lacks (or a missing scene) are absent from the map.
pub fn load_robot_bases(scene: Option<&Path>, robots: &[RobotState]) -> HashMap<String, RobotBase> {
    let mut out = HashMap::new();
    let Some(doc) = scene.and_then(read_yaml) else {
        return out;
    };
    let entries = doc.get("robots").and_then(|r| r.as_sequence()).cloned().unwrap_or_default();
    for robot in robots {
        let lookup = robot.scene_id.as_deref().unwrap_or(&robot.robot_id);
        let Some(entry) = entries.iter().find(|e| e.get("robot_id").and_then(|v| v.as_str()) == Some(lookup)) else {
            eprintln!("WARN: collision: robot '{lookup}' is not in the scene; its base stays at the origin");
            continue;
        };
        let Some(([x, y, z], rotation)) = scene_pose(entry) else {
            eprintln!("WARN: collision: robot '{lookup}' has no valid scene position/orientation; its base stays at the origin");
            continue;
        };
        let pedestal = entry.get("pedestal").and_then(|p| {
            let height_mm = num(p.get("height_mm")).unwrap_or(0.0);
            let diameter_mm = num(p.get("diameter_mm")).unwrap_or(DEFAULT_PEDESTAL_DIAMETER_MM);
            (height_mm > 0.0 && diameter_mm > 0.0)
                .then_some(Pedestal { height_m: height_mm / 1000.0, diameter_m: diameter_mm / 1000.0 })
        });
        let lift = pedestal.map_or(0.0, |p| p.height_m);
        out.insert(
            robot.robot_id.clone(),
            RobotBase { pose: Isometry3::from_parts(Translation3::new(x, y, z + lift), rotation), pedestal },
        );
    }
    out
}

/// The cell's environment boxes: fixtures, pedestals, floor (see the module docs).
pub fn load_environment_boxes(
    scene: Option<&Path>,
    cell_dir: &Path,
    bases: &HashMap<String, RobotBase>,
) -> Vec<EnvironmentBoxSpec> {
    let scene_doc = scene.and_then(read_yaml);
    let scene_dir = scene.and_then(Path::parent).unwrap_or(cell_dir);
    let has_pedestals = bases.values().any(|b| b.pedestal.is_some());
    let mut boxes = Vec::new();

    // The planner's obstacles: the scene's `obstacles:` path, else Prepare Cell's.
    let obstacles_path = scene_doc
        .as_ref()
        .and_then(|d| d.get("obstacles"))
        .and_then(|v| v.as_str())
        .map(|rel| scene_dir.join(rel))
        .unwrap_or_else(|| cell_dir.join(OBSTACLES_YAML));
    if obstacles_path.is_file() {
        boxes.extend(obstacle_boxes(&obstacles_path, has_pedestals));
    }

    // Fixtures the obstacles do not cover: from their URDF's collision geometry.
    let fixtures = scene_doc
        .as_ref()
        .and_then(|d| d.get("fixtures"))
        .and_then(|f| f.as_sequence())
        .cloned()
        .unwrap_or_default();
    for fixture in &fixtures {
        let Some(id) = fixture.get("id").and_then(|v| v.as_str()).filter(|s| !s.is_empty()) else {
            eprintln!("WARN: collision: a scene fixture has no id; skipping it");
            continue;
        };
        let prefix = format!("{id}_");
        if boxes.iter().any(|b: &EnvironmentBoxSpec| b.id == id || b.id.starts_with(&prefix)) {
            continue;
        }
        let Some(urdf) = fixture.get("urdf").and_then(|v| v.as_str()) else {
            eprintln!("WARN: collision: fixture '{id}' has no urdf; skipping it");
            continue;
        };
        let Some(([x, y, z], rotation)) = scene_pose(fixture) else {
            eprintln!("WARN: collision: fixture '{id}' has no valid position/orientation; skipping it");
            continue;
        };
        let pose = Isometry3::from_parts(Translation3::new(x, y, z), rotation);
        match urdf_boxes(&scene_dir.join(urdf)) {
            Ok(local) if local.is_empty() => {
                eprintln!("WARN: collision: fixture '{id}' ({urdf}) has no collision geometry")
            }
            Ok(local) => {
                for (n, (box_pose, half_extents)) in local.into_iter().enumerate() {
                    boxes.push(EnvironmentBoxSpec {
                        id: format!("{id}_{n}"),
                        pose: pose * box_pose,
                        half_extents,
                        role: EnvironmentRole::Obstacle,
                    });
                }
            }
            Err(e) => eprintln!("WARN: collision: fixture '{id}' ({urdf}): {e}; skipping it"),
        }
    }

    // Pedestals: from the floor (the scene z) up to the robot base.
    let mut robot_ids: Vec<&String> = bases.keys().collect();
    robot_ids.sort();
    for robot_id in robot_ids {
        let base = &bases[robot_id];
        let Some(pedestal) = base.pedestal else { continue };
        let top = base.pose.translation.vector;
        let r = pedestal.diameter_m / 2.0;
        let h = pedestal.height_m / 2.0;
        boxes.push(EnvironmentBoxSpec::aligned(
            format!("{robot_id}_pedestal"),
            Vector3::new(top.x, top.y, top.z - h),
            Vector3::new(r, r, h),
            EnvironmentRole::Support(Some(robot_id.clone())),
        ));
    }

    // The floor.
    boxes.push(EnvironmentBoxSpec::aligned(
        "floor",
        Vector3::new(0.0, 0.0, -FLOOR_HALF_THICKNESS_M),
        Vector3::new(FLOOR_HALF_SIZE_M, FLOOR_HALF_SIZE_M, FLOOR_HALF_THICKNESS_M),
        EnvironmentRole::Support(None),
    ));
    boxes
}

/// The boxes and spheres (as their bounding boxes) of an `obstacles.yaml`. Planes are left out
/// (the floor is added separately); so are its pedestal boxes when the scene has pedestals.
fn obstacle_boxes(path: &Path, skip_pedestals: bool) -> Vec<EnvironmentBoxSpec> {
    let Some(doc) = read_yaml(path) else {
        return Vec::new();
    };
    let entries = doc.get("obstacles").and_then(|o| o.as_sequence()).cloned().unwrap_or_default();
    let mut out = Vec::new();
    for (n, entry) in entries.iter().enumerate() {
        let id = entry
            .get("id")
            .and_then(|v| v.as_str())
            .filter(|s| !s.is_empty())
            .map_or_else(|| format!("obstacle_{n}"), str::to_owned);
        if skip_pedestals && id.contains("pedestal") {
            continue;
        }
        let kind = entry.get("type").and_then(|v| v.as_str()).unwrap_or_default();
        let center = triple(entry.get("center"));
        let half_extents = match kind {
            "box" => triple(entry.get("half_extents")),
            "sphere" => num(entry.get("radius")).map(|r| [r, r, r]),
            "plane" => continue,
            other => {
                eprintln!("WARN: collision: {}: obstacle '{id}' has unsupported type {other:?}; skipping it", path.display());
                continue;
            }
        };
        let (Some(c), Some(h)) = (center, half_extents) else {
            eprintln!("WARN: collision: {}: obstacle '{id}' lacks a valid center/size; skipping it", path.display());
            continue;
        };
        let workpiece = entry.get("is_workpiece").and_then(|v| v.as_bool()).unwrap_or(false);
        out.push(EnvironmentBoxSpec::aligned(
            id,
            Vector3::from(c),
            Vector3::from(h),
            if workpiece { EnvironmentRole::Workpiece } else { EnvironmentRole::Obstacle },
        ));
    }
    out
}

/* ------------------------------------------------------------------------- */
/* Fixture URDF collision geometry, as boxes                                 */
/* ------------------------------------------------------------------------- */

#[derive(Default)]
struct UrdfCollision {
    link: String,
    origin: Isometry3<f64>,
    geometry: Option<UrdfGeometry>,
}

enum UrdfGeometry {
    Box([f64; 3]),
    Cylinder { radius: f64, length: f64 },
    Sphere(f64),
    Mesh { filename: String, scale: [f64; 3] },
}

struct UrdfJoint {
    parent: String,
    child: String,
    origin: Isometry3<f64>,
}

fn parse_floats<const N: usize>(text: &str) -> Option<[f64; N]> {
    let values: Vec<f64> = text.split_whitespace().map(str::parse).collect::<Result<_, _>>().ok()?;
    let array: [f64; N] = values.try_into().ok()?;
    array.iter().all(|v| v.is_finite()).then_some(array)
}

/// A URDF `<origin xyz rpy>` (fixed-axis roll, pitch, yaw).
fn urdf_origin(xyz: Option<&str>, rpy: Option<&str>) -> Isometry3<f64> {
    let [x, y, z] = xyz.and_then(parse_floats::<3>).unwrap_or([0.0; 3]);
    let [r, p, w] = rpy.and_then(parse_floats::<3>).unwrap_or([0.0; 3]);
    Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::from_euler_angles(r, p, w))
}

/// Every `<collision>` of the URDF at `path` as a box `(pose in the URDF root frame, half-extents)`.
/// Links are placed by their joints' origins (joints at zero).
fn urdf_boxes(path: &Path) -> Result<Vec<(Isometry3<f64>, Vector3<f64>)>, String> {
    use quick_xml::events::Event;

    let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut reader = quick_xml::Reader::from_str(&text);
    let mut stack: Vec<String> = Vec::new();
    let mut link = String::new();
    let mut collisions: Vec<UrdfCollision> = Vec::new();
    let mut current: Option<UrdfCollision> = None;
    let mut joints: Vec<UrdfJoint> = Vec::new();
    let mut joint: Option<UrdfJoint> = None;

    loop {
        let event = reader.read_event().map_err(|e| format!("{}: invalid XML: {e}", path.display()))?;
        let (element, empty) = match &event {
            Event::Start(e) => (e, false),
            Event::Empty(e) => (e, true),
            Event::End(_) => {
                match stack.pop().as_deref() {
                    Some("collision") => collisions.extend(current.take()),
                    Some("joint") => joints.extend(joint.take()),
                    _ => {}
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        let name = element.local_name().as_ref().to_owned();
        let attr = |key: &str| -> Option<String> {
            element
                .try_get_attribute(key)
                .ok()
                .flatten()
                // Entities replaced, as unescape_value() did (it also normalizes tab/CR/LF to spaces now,
                // as XML's attribute-value normalization does).
                .and_then(|a| a.normalized_value(quick_xml::XmlVersion::Implicit1_0).ok().map(|v| v.into_owned()))
        };
        let parent = stack.last().map(String::as_str);
        match (name.as_str(), parent) {
            ("link", _) => link = attr("name").unwrap_or_default(),
            ("collision", _) => {
                current = Some(UrdfCollision { link: link.clone(), ..Default::default() })
            }
            ("origin", Some("collision")) => {
                if let Some(c) = current.as_mut() {
                    c.origin = urdf_origin(attr("xyz").as_deref(), attr("rpy").as_deref());
                }
            }
            ("box" | "cylinder" | "sphere" | "mesh", Some("geometry")) => {
                let geometry = match name.as_str() {
                    "box" => attr("size").as_deref().and_then(parse_floats::<3>).map(UrdfGeometry::Box),
                    "cylinder" => {
                        let radius = attr("radius").and_then(|v| v.trim().parse().ok());
                        let length = attr("length").and_then(|v| v.trim().parse().ok());
                        radius.zip(length).map(|(radius, length)| UrdfGeometry::Cylinder { radius, length })
                    }
                    "sphere" => attr("radius").and_then(|v| v.trim().parse().ok()).map(UrdfGeometry::Sphere),
                    _ => attr("filename").map(|filename| UrdfGeometry::Mesh {
                        filename,
                        scale: attr("scale").as_deref().and_then(parse_floats::<3>).unwrap_or([1.0; 3]),
                    }),
                };
                if let (Some(c), Some(g)) = (current.as_mut(), geometry) {
                    c.geometry = Some(g);
                }
            }
            ("joint", _) => {
                joint = Some(UrdfJoint { parent: String::new(), child: String::new(), origin: Isometry3::identity() })
            }
            ("parent" | "child" | "origin", Some("joint")) => {
                if let Some(j) = joint.as_mut() {
                    match name.as_str() {
                        "parent" => j.parent = attr("link").unwrap_or_default(),
                        "child" => j.child = attr("link").unwrap_or_default(),
                        _ => j.origin = urdf_origin(attr("xyz").as_deref(), attr("rpy").as_deref()),
                    }
                }
            }
            _ => {}
        }
        if !empty {
            stack.push(name);
        } else if name == "collision" {
            collisions.extend(current.take());
        } else if name == "joint" {
            joints.extend(joint.take());
        }
    }

    // Link poses in the root frame: compose joint origins up to a link no joint has as child.
    let by_child: HashMap<&str, &UrdfJoint> = joints.iter().map(|j| (j.child.as_str(), j)).collect();
    let link_pose = |link: &str| -> Isometry3<f64> {
        let mut pose = Isometry3::identity();
        let mut at = link;
        for _ in 0..=joints.len() {
            let Some(j) = by_child.get(at) else { break };
            pose = j.origin * pose;
            at = j.parent.as_str();
        }
        pose
    };

    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    let mut out = Vec::new();
    for collision in &collisions {
        let Some(geometry) = &collision.geometry else { continue };
        let frame = link_pose(&collision.link) * collision.origin;
        let (center, half) = match geometry {
            UrdfGeometry::Box([x, y, z]) => (Vector3::zeros(), Vector3::new(x / 2.0, y / 2.0, z / 2.0)),
            UrdfGeometry::Cylinder { radius, length } => (Vector3::zeros(), Vector3::new(*radius, *radius, length / 2.0)),
            UrdfGeometry::Sphere(r) => (Vector3::zeros(), Vector3::new(*r, *r, *r)),
            UrdfGeometry::Mesh { filename, scale } => match mesh_bounds(&resolve_mesh(base_dir, filename)) {
                Ok((lo, hi)) => {
                    let s = Vector3::from(*scale);
                    let (a, b) = (lo.component_mul(&s), hi.component_mul(&s));
                    let (lo, hi) = (a.inf(&b), a.sup(&b));
                    ((lo + hi) / 2.0, (hi - lo) / 2.0)
                }
                Err(e) => {
                    eprintln!("WARN: collision: {}: mesh {filename}: {e}; skipping it", path.display());
                    continue;
                }
            },
        };
        let pose = frame * Isometry3::from_parts(Translation3::from(center), UnitQuaternion::identity());
        out.push((pose, half.abs()));
    }
    Ok(out)
}

/// A URDF mesh `filename` as a path: relative to the URDF, `file://` stripped. `package://` URIs
/// are tried relative to the URDF, without the package name.
fn resolve_mesh(base_dir: &Path, filename: &str) -> PathBuf {
    if let Some(rest) = filename.strip_prefix("file://") {
        return PathBuf::from(rest);
    }
    if let Some(rest) = filename.strip_prefix("package://") {
        let rest = rest.split_once('/').map_or(rest, |(_, r)| r);
        return base_dir.join(rest);
    }
    base_dir.join(filename)
}

/// The local bounding box `(min, max)` of an OBJ, STL or GLB mesh.
fn mesh_bounds(path: &Path) -> Result<(Vector3<f64>, Vector3<f64>), String> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or_default().to_ascii_lowercase();
    if ext == "glb" || ext == "gltf" {
        return match crate::collision::glb::load_collision_proxy(path, Default::default())? {
            crate::collision::glb::CollisionProxy::Box { center, half_extents } => {
                Ok((center - half_extents, center + half_extents))
            }
            crate::collision::glb::CollisionProxy::ConvexHull { vertices } => bounds(vertices.into_iter()),
        };
    }
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    match ext.as_str() {
        "obj" => {
            let text = String::from_utf8_lossy(&bytes);
            bounds(text.lines().filter_map(|line| {
                let rest = line.trim_start().strip_prefix("v ")?;
                let mut it = rest.split_whitespace().map(|v| v.parse::<f64>().ok());
                Some(Vector3::new(it.next()??, it.next()??, it.next()??))
            }))
        }
        "stl" => {
            let binary_len = bytes
                .get(80..84)
                .map(|n| 84 + 50 * u32::from_le_bytes([n[0], n[1], n[2], n[3]]) as usize);
            if binary_len == Some(bytes.len()) {
                bounds(bytes[84..].chunks_exact(50).flat_map(|tri| {
                    (0..3).map(move |v| {
                        let f = |k: usize| {
                            let at = 12 + v * 12 + k * 4;
                            f32::from_le_bytes([tri[at], tri[at + 1], tri[at + 2], tri[at + 3]]) as f64
                        };
                        Vector3::new(f(0), f(1), f(2))
                    })
                }))
            } else {
                let text = String::from_utf8_lossy(&bytes);
                bounds(text.lines().filter_map(|line| {
                    let rest = line.trim_start().strip_prefix("vertex")?;
                    let mut it = rest.split_whitespace().map(|v| v.parse::<f64>().ok());
                    Some(Vector3::new(it.next()??, it.next()??, it.next()??))
                }))
            }
        }
        other => Err(format!("unsupported mesh format {other:?}")),
    }
}

fn bounds(points: impl Iterator<Item = Vector3<f64>>) -> Result<(Vector3<f64>, Vector3<f64>), String> {
    let mut out: Option<(Vector3<f64>, Vector3<f64>)> = None;
    for p in points.filter(|p| p.iter().all(|v| v.is_finite())) {
        out = Some(match out {
            None => (p, p),
            Some((lo, hi)) => (lo.inf(&p), hi.sup(&p)),
        });
    }
    out.ok_or_else(|| "the mesh has no vertices".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::{
        CollisionConfig, CollisionKind, RobotCollisionSpec, SceneCollisionChecker, SceneTrajectory,
    };
    use crate::path::JointPath;

    /// A 6-joint arm: a base block (no joint moves it), a 2 m arm on joint 2 (pitch about y,
    /// pointing along +x at zero), and a small tool block at the arm's end on joint 6.
    const ARM_URDF: &str = r#"<?xml version="1.0"?>
<robot name="arm">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <link name="l1"/>
  <link name="l2"><collision><origin xyz="1.0 0 0"/><geometry><box size="2.0 0.1 0.1"/></geometry></collision></link>
  <link name="l3"/><link name="l4"/><link name="l5"/>
  <link name="l6"><collision><origin xyz="0 0 -0.05"/><geometry><box size="0.04 0.04 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="2.0 0 0"/><axis xyz="0 1 0"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
  <joint name="j4" type="revolute"><parent link="l3"/><child link="l4"/><origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
  <joint name="j5" type="revolute"><parent link="l4"/><child link="l5"/><origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
  <joint name="j6" type="revolute"><parent link="l5"/><child link="l6"/><origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-6" upper="6" effort="1" velocity="1"/></joint>
</robot>
"#;

    /// A fixture: a 1 m cube standing on the floor at its origin.
    const CUBE_URDF: &str = r#"<?xml version="1.0"?>
<robot name="cube">
  <link name="World"><collision><origin xyz="0 0 0.5"/><geometry><box size="1 1 1"/></geometry></collision></link>
</robot>
"#;

    /// The cell: the cube at the origin, and the arm at x = 2 on a 1 m pedestal, turned 180° to
    /// face the cube (its +x points along world -x).
    const SCENE: &str = "\
robots:
  - robot_id: arm_scene
    position: {x: 2.0, y: 0.0, z: 0.0}
    orientation: {a: 180.0, b: 0.0, c: 0.0}
    pedestal: {height_mm: 1000.0, diameter_mm: 1000.0}
fixtures:
  - id: cube
    urdf: fixtures/cube.urdf
    position: {x: 0.0, y: 0.0, z: 0.0}
";

    fn cell_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("cell_runner_env_{}_{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("fixtures")).unwrap();
        std::fs::write(dir.join("arm.urdf"), ARM_URDF).unwrap();
        std::fs::write(dir.join("fixtures/cube.urdf"), CUBE_URDF).unwrap();
        std::fs::write(dir.join("scene.yaml"), SCENE).unwrap();
        dir
    }

    /// The cell robot `arm`, `arm_scene` in the scene.
    fn robot(dir: &Path) -> RobotState {
        let cell_yaml = dir.join("cell.yaml");
        std::fs::write(&cell_yaml, "name: test\nrobots:\n  - id: arm\n    scene_id: arm_scene\n    name: Arm\n    urdf: arm.urdf\n")
            .unwrap();
        let mut sim = crate::cell::CellSimulator::from_run_config(&cell_yaml).expect("the cell config parses");
        sim.robots.remove(0)
    }

    fn checker(dir: &Path) -> SceneCollisionChecker {
        let scene = dir.join("scene.yaml");
        let robots = [robot(dir)];
        let bases = load_robot_bases(Some(&scene), &robots);
        let boxes = load_environment_boxes(Some(&scene), dir, &bases);
        let spec = RobotCollisionSpec::new("arm", dir.join("arm.urdf"), None, bases["arm"].pose);
        let config = CollisionConfig { check_self: false, report_all: true, ..CollisionConfig::default() };
        SceneCollisionChecker::build_with_boxes(&[spec], &[], &boxes, config).expect("the checker builds")
    }

    /// A plan from `start` to `end` (joint 2 only), sampled densely.
    fn plan(pitch_start: f64, pitch_end: f64) -> SceneTrajectory {
        let qs: Vec<[f64; 6]> = (0..=20)
            .map(|k| {
                let p = pitch_start + (pitch_end - pitch_start) * k as f64 / 20.0;
                [0.0, p, 0.0, 0.0, 0.0, 0.0]
            })
            .collect();
        let n = qs.len();
        SceneTrajectory::new(n, n).with_path("arm", JointPath::Dense { qs, takt_s: 0.012 })
    }

    #[test]
    fn the_fixture_the_pedestal_and_the_floor_are_environment_objects() {
        let dir = cell_dir("objects");
        let checker = checker(&dir);
        let mut names: Vec<(String, EnvironmentRole)> =
            checker.environment_objects().map(|(o, r)| (o.name.clone(), r.clone())).collect();
        names.sort_by(|a, b| a.0.cmp(&b.0));
        assert_eq!(
            names,
            [
                ("environment/arm_pedestal".to_owned(), EnvironmentRole::Support(Some("arm".into()))),
                ("environment/cube_0".to_owned(), EnvironmentRole::Obstacle),
                ("environment/floor".to_owned(), EnvironmentRole::Support(None)),
            ]
        );
        let bases = load_robot_bases(Some(&dir.join("scene.yaml")), &[robot(&dir)]);
        let base = &bases["arm"].pose;
        assert!((base.translation.vector - Vector3::new(2.0, 0.0, 1.0)).norm() < 1e-12, "on the pedestal");
        assert!((base * Vector3::x() + Vector3::x()).norm() < 1e-12, "turned to face the cube");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_plan_through_the_fixture_collides_and_one_above_it_does_not() {
        let dir = cell_dir("plans");
        let mut checker = checker(&dir);

        // Arm level at z = 1.2 over the cube (top at z = 1), sweeping a little up: clear. The
        // base block stands on its pedestal, which is not a collision.
        let above = checker.check_trajectory(&plan(0.0, -0.2));
        assert!(!above.collided, "a plan above the cube is clear: {:?}", above.first);

        // Pitching the arm down 30° drives it into the cube.
        let through = checker.check_trajectory(&plan(0.0, 0.52));
        assert!(through.collided, "a plan through the cube collides");
        let hit = through.first.expect("the first colliding waypoint");
        assert!(
            hit.report.pairs.iter().any(|p| p.kind == CollisionKind::RobotEnvironment
                && (p.object_a == "environment/cube_0" || p.object_b == "environment/cube_0")),
            "the arm hits the cube: {:?}",
            hit.report.pairs
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_tool_end_may_touch_a_workpiece_but_the_arm_may_not() {
        let dir = cell_dir("workpiece");
        // The planner's obstacles: the cube top as a workpiece slab (z 0.95..1.0), the rest of
        // the cube as an obstacle, and its lowered pedestal box (replaced by the scene's).
        std::fs::create_dir_all(dir.join("agentic_planner/config")).unwrap();
        std::fs::write(
            dir.join(OBSTACLES_YAML),
            "obstacles:
  - {id: floor, type: plane, normal: [0, 0, 1], offset: -0.05}
  - {id: cube_top, type: box, center: [0, 0, 0.975], half_extents: [0.5, 0.5, 0.025], is_workpiece: true}
  - {id: cube_body, type: box, center: [0, 0, 0.475], half_extents: [0.5, 0.5, 0.475]}
  - {id: pedestal_robot1, type: box, center: [2, 0, 0.4], half_extents: [0.5, 0.5, 0.4]}
",
        )
        .unwrap();
        let mut checker = checker(&dir);
        let ids: Vec<String> = checker.environment_objects().map(|(o, _)| o.name.clone()).collect();
        assert!(!ids.iter().any(|id| id.contains("pedestal_robot1")), "the planner's pedestal is replaced: {ids:?}");
        assert!(!ids.iter().any(|id| id.contains("cube_0")), "obstacles.yaml covers the fixture: {ids:?}");

        // Joint 2 pitched down a little: at 0.06 rad the arm's box stays above the top (z >= 1.03)
        // while the tool block at its end dips 2 cm into the top slab; at 0.15 rad the arm's own
        // box reaches down into the top.
        let config = |pitch: f64| crate::collision::SceneConfig::new().with_robot("arm", [0.0, pitch, 0.0, 0.0, 0.0, 0.0]);
        // Tool tip just into the workpiece slab, arm clear of it: allowed.
        let touch = checker.check_configuration(&config(0.06));
        assert!(
            !touch.pairs.iter().any(|p| p.object_a.contains("cube_top") || p.object_b.contains("cube_top")),
            "the tool end may touch the workpiece: {:?}",
            touch.pairs
        );
        // The arm itself into the workpiece: a collision.
        let arm = checker.check_configuration(&config(0.15));
        assert!(
            arm.pairs.iter().any(|p| (p.object_a.contains("cube_top") || p.object_b.contains("cube_top"))
                && (p.object_a == "arm/geom1" || p.object_b == "arm/geom1")),
            "the arm may not touch the workpiece: {:?}",
            arm.pairs
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A link mesh off its link frame (as every KUKA link hull is) is checked where the mesh is,
    /// not as a box centred on the frame.
    #[test]
    fn a_link_mesh_away_from_its_frame_is_checked_where_it_is() {
        let dir = cell_dir("offset_mesh");
        // A 0.2 m cube mesh 1.5..1.7 m along x of link l2's frame.
        let mut obj = String::new();
        for x in [1.5, 1.7] {
            for y in [-0.1, 0.1] {
                for z in [-0.1, 0.1] {
                    obj.push_str(&format!("v {x} {y} {z}\n"));
                }
            }
        }
        obj.push_str("f 1 2 3\nf 5 6 7\n");
        std::fs::write(dir.join("offset.obj"), obj).unwrap();
        let urdf = ARM_URDF.replace(
            r#"<link name="l2"><collision><origin xyz="1.0 0 0"/><geometry><box size="2.0 0.1 0.1"/></geometry></collision></link>"#,
            r#"<link name="l2"><collision><geometry><mesh filename="offset.obj"/></geometry></collision></link>"#,
        );
        assert_ne!(urdf, ARM_URDF);
        std::fs::write(dir.join("arm.urdf"), urdf).unwrap();

        let probe = |at: Vector3<f64>| {
            let boxes = [EnvironmentBoxSpec::aligned("probe", at, Vector3::new(0.05, 0.05, 0.05), EnvironmentRole::Obstacle)];
            let spec = RobotCollisionSpec::new("arm", dir.join("arm.urdf"), None, Isometry3::identity());
            let config = CollisionConfig { check_self: false, ..CollisionConfig::default() };
            let mut checker = SceneCollisionChecker::build_with_boxes(&[spec], &[], &boxes, config).unwrap();
            let report = checker.check_configuration(&crate::collision::SceneConfig::new().with_robot("arm", [0.0; 6]));
            report.pairs.iter().any(|p| p.object_b == "environment/probe" || p.object_a == "environment/probe")
        };
        // l2's frame is at z = 0.2 (joint 1).
        assert!(probe(Vector3::new(1.6, 0.0, 0.2)), "the mesh 1.6 m out is hit");
        // Just above the link frame (and clear of the base block, top z = 0.2): a box centred on
        // the frame (half 0.1) would reach it.
        assert!(!probe(Vector3::new(0.0, 0.0, 0.32)), "nothing is at the link frame itself");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
