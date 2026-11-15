//! Model nodes: a URDF from bytes, its geometry like Rerun's `UrdfTree::emit` (whose mesh loading
//! reads files, and is unsupported on the web), its joints as per-joint transforms.
//!
//! Collision geometry is fetched and logged only once a model shows it (`show_collision`); turned
//! off again, it is cleared from the recording, and logged again (from the cached bytes) when
//! shown again. Not hidden by the blueprint's query: Rerun shows a clone of the blueprint made
//! when it was activated, so later writes to it never reach the view (see [`super::blueprint`]).

use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

use factoryos_client::protocol::scene::{ModelRef, Node, NodeKind};
use re_importer::UrdfTree;
use re_importer::external::urdf_rs::{self, Geometry, JointType, Material};
use web_time::Instant;

use crate::rr::re_chunk::EntityPath;
use crate::rr::re_log;
use crate::rr::re_sdk_types::archetypes::{Boxes3D, Capsules3D, CoordinateFrame, Cylinders3D, Ellipsoids3D, InstancePoses3D};
use crate::rr::re_sdk_types::components::Color;
use crate::rr::re_sdk_types::datatypes::{Rgba32, Vec3D};
use crate::rr::re_sdk_types::external::glam;
use crate::rr::re_sdk_types::AsComponents;

use super::super::{AssetSource, AssetState, Priority};
use super::entities::{self, Out, PendingMesh};
use super::framing;

/// A model node's URDF, once loaded, and the joint values shown.
pub struct Model {
    id: String,
    /// The node's frame ([`entities::Frames`]), which its link frames are named after.
    frame: String,
    pub model: ModelRef,
    pub show_collision: bool,
    visible: bool,
    state: Load,
    /// The values wanted, by joint name (missing: 0).
    joints: BTreeMap<String, f64>,
    /// The values logged, by joint name.
    shown: HashMap<String, f64>,
    /// The collision geometry is logged (once loaded, visible and asked to show it).
    collisions: bool,
}

enum Load {
    /// For the URDF; after a fetch that failed for now, not asked again before `retry_at`.
    Waiting { retry_at: Option<Instant> },
    Loaded(Box<Loaded>),
    /// For good: the URDF does not parse, or its bytes don't match its hash.
    Failed,
}

struct Loaded {
    tree: UrdfTree,
    materials: HashMap<String, Material>,
}

impl Model {
    /// `None` unless `node` is a model. `frame`: the node's frame.
    pub fn new(node: &Node, frame: String) -> Option<Self> {
        let NodeKind::Model { model, joints, show_collision } = &node.kind else { return None };
        Some(Self {
            id: node.id.clone(),
            frame,
            model: model.clone(),
            show_collision: *show_collision,
            visible: node.visible,
            state: Load::Waiting { retry_at: None },
            joints: joints.clone(),
            shown: HashMap::new(),
            collisions: false,
        })
    }

    /// The URDF's robot name once it is loaded: its geometry is under `/<id>/<name>/`.
    pub fn robot_name(&self) -> Option<&str> {
        match &self.state {
            Load::Loaded(loaded) => Some(loaded.tree.name()),
            _ => None,
        }
    }

    /// Whether node `node` is still this model: the same URDF and meshes, shown or hidden alike.
    /// Then an upsert of it need not load anything again (see [`Self::update`]).
    pub fn is_same(&self, node: &Node) -> bool {
        matches!(&node.kind, NodeKind::Model { model, .. } if *model == self.model) && node.visible == self.visible
    }

    /// Take what an upsert of the same model ([`Self::is_same`]) changes: `show_collision`
    /// (collision geometry logged when shown, meshes into `pending`; cleared when no longer) and
    /// the joint values (all of them: missing ones are 0 again).
    pub fn update(&mut self, node: &Node, out: &mut Out, pending: &mut Vec<PendingMesh>) {
        let NodeKind::Model { joints, show_collision, .. } = &node.kind else { return };
        self.show_collision = *show_collision;
        self.emit_collisions(out, pending);
        self.clear_collisions(out, pending);
        self.joints = joints.clone();
        self.log_joints(out);
    }

    /// Clear the collision geometry if it is logged and no longer wanted (its meshes still on
    /// their way too).
    fn clear_collisions(&mut self, out: &mut Out, pending: &mut Vec<PendingMesh>) {
        if self.show_collision || !self.collisions {
            return;
        }
        let Some(root) = self.collision_root() else { return };
        pending.retain(|mesh| !(mesh.node == self.id && mesh.entity.is_descendant_of(&root)));
        out.clear(root, true);
        self.collisions = false;
    }

    /// `/<id>/<robot>/collision_geometries`, once the URDF is loaded.
    fn collision_root(&self) -> Option<EntityPath> {
        let robot = self.robot_name()?;
        Some(entities::entity(&self.id) / EntityPath::from_single_string(robot) / EntityPath::from_single_string("collision_geometries"))
    }

    /// Log the collision geometry if it is wanted and not yet logged.
    fn emit_collisions(&mut self, out: &mut Out, pending: &mut Vec<PendingMesh>) {
        if let (Load::Loaded(loaded), true, true, false) = (&self.state, self.show_collision, self.visible, self.collisions) {
            emit_links(&self.id, &self.frame, &self.model, loaded, Geometries::Collision, out, pending);
            self.collisions = true;
        }
    }

    /// Still to be loaded, and due to be tried at `now` (not before a failed fetch's retry time).
    pub fn waiting(&self, now: Instant) -> bool {
        matches!(self.state, Load::Waiting { retry_at } if retry_at.is_none_or(|at| now >= at))
    }

    /// Its URDF is still to be loaded (now or after a retry); not once loaded or failed for good.
    pub fn loading(&self) -> bool {
        matches!(self.state, Load::Waiting { .. })
    }

    /// When a failed fetch will be retried, if one did.
    pub fn retry_at(&self) -> Option<Instant> {
        match self.state {
            Load::Waiting { retry_at } => retry_at,
            _ => None,
        }
    }

    /// Load the URDF if it has arrived: geometry into `out` (meshes into `pending`), then every
    /// joint. After a fetch that failed for now, it is tried again `retry_after` later.
    pub fn try_load(&mut self, assets: &dyn AssetSource, out: &mut Out, pending: &mut Vec<PendingMesh>, now: Instant, retry_after: Duration) {
        let bytes = match assets.get_with(&self.model.urdf, Priority::High) {
            AssetState::Pending => return,
            AssetState::Failed { err, retryable: true } => {
                re_log::warn_once!("Failed to fetch the URDF of {} (retrying): {err}", self.id);
                self.state = Load::Waiting { retry_at: Some(now + retry_after) };
                return;
            }
            AssetState::Failed { err, retryable: false } => {
                re_log::warn!("Failed to fetch the URDF of {}: {err}", self.id);
                self.state = Load::Failed;
                return;
            }
            AssetState::Ready(bytes) => bytes,
        };
        let parsed = std::str::from_utf8(&bytes)
            .map_err(anyhow::Error::from)
            .and_then(|xml| Ok(urdf_rs::read_from_string(xml)?))
            .and_then(|robot| {
                let materials = robot.materials.iter().map(|m| (m.name.clone(), m.clone())).collect();
                let tree = UrdfTree::new(robot, None, Some(entities::entity(&self.id)))?
                    .with_frame_prefix(entities::link_frame_prefix(&self.frame));
                Ok(Loaded { tree, materials })
            });
        let loaded = match parsed {
            Ok(loaded) => loaded,
            Err(err) => {
                re_log::warn!("Failed to load the URDF of {}: {err:#}", self.id);
                self.state = Load::Failed;
                return;
            }
        };
        emit_root(&self.id, &self.frame, &loaded, out);
        if self.visible {
            emit_links(&self.id, &self.frame, &self.model, &loaded, Geometries::Visual, out, pending);
        }
        self.state = Load::Loaded(Box::new(loaded));
        self.collisions = false;
        self.emit_collisions(out, pending);
        self.shown.clear();
        self.log_joints(out);
    }

    /// Show `joints` (others keep their value): logs the transforms of those that changed.
    pub fn set_joints(&mut self, joints: &BTreeMap<String, f64>, out: &mut Out) {
        self.joints.extend(joints.iter().map(|(name, value)| (name.clone(), *value)));
        self.log_joints(out);
    }

    /// Every joint whose value differs from the one logged, at `/<id>/joints/<joint>`. Fixed joints
    /// sit at 0; floating / planar / spherical ones (which Rerun can't pose, e.g. a fixture's `world`
    /// joint) are treated as fixed, so their child links still hang off the root.
    fn log_joints(&mut self, out: &mut Out) {
        let Load::Loaded(loaded) = &self.state else { return };
        for joint in loaded.tree.joints() {
            let value = match joint.joint_type {
                JointType::Revolute | JointType::Continuous | JointType::Prismatic => {
                    self.joints.get(&joint.name).copied().unwrap_or(0.0)
                }
                _ => 0.0,
            };
            if self.shown.get(&joint.name) == Some(&value) {
                continue;
            }
            let fixed;
            let posed = if matches!(joint.joint_type, JointType::Floating | JointType::Planar | JointType::Spherical) {
                fixed = urdf_rs::Joint { joint_type: JointType::Fixed, ..joint.clone() };
                &fixed
            } else {
                joint
            };
            match loaded.tree.compute_joint_transform(posed, value, true) {
                Ok(transform) => {
                    out.transform(joint_entity(&self.id, &joint.name), &transform);
                    self.shown.insert(joint.name.clone(), value);
                }
                Err(err) => re_log::warn_once!("Joint {} of {} failed: {err}", joint.name, self.id),
            }
        }
    }
}

impl Model {
    /// The value joint `joint` is shown at, as [`Self::log_joints`] logs it, and the joint it is
    /// posed as (floating / planar / spherical ones as fixed).
    fn posed(&self, joint: &urdf_rs::Joint) -> (f64, urdf_rs::Joint) {
        match joint.joint_type {
            JointType::Revolute | JointType::Continuous | JointType::Prismatic => {
                (self.joints.get(&joint.name).copied().unwrap_or(0.0), joint.clone())
            }
            JointType::Floating | JointType::Planar | JointType::Spherical => {
                (0.0, urdf_rs::Joint { joint_type: JointType::Fixed, ..joint.clone() })
            }
            _ => (0.0, joint.clone()),
        }
    }

    /// Every link's pose in the model node's frame at the joint values shown (forward kinematics,
    /// as the joint transforms place them), once the URDF is loaded.
    pub fn link_poses(&self) -> Option<HashMap<String, glam::Affine3A>> {
        let Load::Loaded(loaded) = &self.state else { return None };
        let tree = &loaded.tree;
        let mut poses = HashMap::from([(tree.root().name.clone(), glam::Affine3A::IDENTITY)]);
        let mut links = vec![tree.root()];
        while let Some(link) = links.pop() {
            let parent = poses[&link.name];
            for joint in tree.get_children(&link.name).into_iter().flatten() {
                let (value, posed) = self.posed(joint);
                let Ok(t) = re_importer::urdf_joint_transform::internal::compute_joint_transform(&posed, value, true) else { continue };
                let child = tree.get_joint_child(joint);
                if poses.contains_key(&child.name) {
                    continue; // Not a tree.
                }
                poses.insert(child.name.clone(), parent * glam::Affine3A::from_rotation_translation(t.quaternion, t.translation));
                links.push(child);
            }
        }
        Some(poses)
    }

    /// What the model's visual geometry covers in its node's frame, once loaded and if shown: the
    /// corners of each visual's box (meshes by their extents in `meshes`, or their origin until
    /// they are drawn), placed. A link turned by its joint keeps its own box's shape, not that of
    /// the box around it.
    pub fn visual_corners(&self, meshes: &framing::MeshBounds) -> Option<Vec<glam::Vec3>> {
        let Load::Loaded(loaded) = &self.state else { return None };
        if !self.visible {
            return None;
        }
        let poses = self.link_poses()?;
        let mut corners = Vec::new();
        for (name, link_pose) in &poses {
            let Some(link) = loaded.tree.get_link(name) else { continue };
            for visual in &link.visual {
                let origin = glam::Affine3A::from_rotation_translation(
                    quat_from_rpy(&visual.origin.rpy.0),
                    glam::Vec3::from_array(visual.origin.xyz.0.map(|v| v as f32)),
                );
                let half = |x: f64, y: f64, z: f64| {
                    let half = glam::Vec3::new(x as f32, y as f32, z as f32);
                    Some(framing::Aabb { min: -half, max: half })
                };
                let local = match &visual.geometry {
                    Geometry::Box { size: urdf_rs::Vec3([x, y, z]) } => half(x / 2.0, y / 2.0, z / 2.0),
                    Geometry::Cylinder { radius, length } => half(*radius, *radius, length / 2.0),
                    Geometry::Capsule { radius, length } => half(*radius, *radius, length / 2.0 + radius),
                    Geometry::Sphere { radius } => half(*radius, *radius, *radius),
                    Geometry::Mesh { filename, scale } => {
                        let scale = scale.map_or(glam::Vec3::ONE, |urdf_rs::Vec3(s)| glam::Vec3::from_array(s.map(|v| v as f32)));
                        let mesh = self.model.meshes.get(filename).and_then(|mesh| meshes.get(mesh));
                        Some(mesh.map_or(framing::Aabb::point(glam::Vec3::ZERO), |m| {
                            framing::Aabb::from_points(m.corners().map(|c| c * scale)).unwrap_or(m)
                        }))
                    }
                };
                if let Some(local) = local {
                    let placement = *link_pose * origin;
                    corners.extend(local.corners().map(|c| placement.transform_point3(c)));
                }
            }
        }
        (!corners.is_empty()).then_some(corners)
    }
}

/// `/<id>/joints/<joint>`.
pub fn joint_entity(id: &str, joint: &str) -> EntityPath {
    entities::entity(id) / EntityPath::from_single_string("joints") / EntityPath::from_single_string(joint)
}

/// The model's root frame: `/<id>/<robot>`, the root link's frame, attached to the node's frame.
/// Logged for hidden models too: their frames are still there for children.
fn emit_root(id: &str, node_frame: &str, loaded: &Loaded, out: &mut Out) {
    let tree = &loaded.tree;
    let root_entity = entities::entity(id) / EntityPath::from_single_string(tree.name());
    let root_frame = entities::link_frame(node_frame, &tree.root().name);
    out.transform(root_entity.clone(), &entities::pose_transform(&Default::default(), node_frame.to_owned(), root_frame.clone()));
    out.geometry(root_entity, &[&CoordinateFrame::new(root_frame)]);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Geometries {
    Visual,
    Collision,
}

/// The links' visual or collision geometry, in the entity layout of `UrdfTree::emit`:
/// `/<id>/<robot>/visual_geometries/<link>/<visual>`,
/// `/<id>/<robot>/collision_geometries/<type>/<link>/<collision>`, each with an instance pose and
/// the link's frame. Meshes go into `pending`.
fn emit_links(
    id: &str,
    node_frame: &str,
    model: &ModelRef,
    loaded: &Loaded,
    which: Geometries,
    out: &mut Out,
    pending: &mut Vec<PendingMesh>,
) {
    let tree = &loaded.tree;
    let mut links = vec![tree.root()];
    while let Some(link) = links.pop() {
        let frame = entities::link_frame(node_frame, &link.name);
        let mut add = |entity: EntityPath, origin: &urdf_rs::Pose, geometry: &Geometry, material: Option<&Material>| {
            let mut poses = InstancePoses3D::update_fields()
                .with_translations([origin.xyz.0.map(|v| v as f32)])
                .with_quaternions([quat_from_rpy(&origin.rpy.0).to_array()]);
            let albedo = material_albedo(material);
            if let Geometry::Mesh { filename, scale } = geometry {
                if let Some(urdf_rs::Vec3([x, y, z])) = scale {
                    poses = poses.with_scales([Vec3D::new(*x as f32, *y as f32, *z as f32)]);
                }
                match model.meshes.get(filename) {
                    Some(mesh) => pending.push(PendingMesh::new(id, entity, mesh.clone(), filename.clone(), poses, frame.clone(), albedo)),
                    None => re_log::warn_once!("Mesh {filename} of {id} is not in its model's assets"),
                }
                return;
            }
            let color = Color::new(albedo.unwrap_or(Rgba32::WHITE));
            let shape: Box<dyn AsComponents> = match geometry {
                Geometry::Box { size: urdf_rs::Vec3([x, y, z]) } => {
                    Box::new(Boxes3D::from_sizes([Vec3D::new(*x as f32, *y as f32, *z as f32)]).with_colors([color]))
                }
                Geometry::Cylinder { radius, length } => {
                    Box::new(Cylinders3D::from_lengths_and_radii([*length as f32], [*radius as f32]).with_colors([color]))
                }
                Geometry::Capsule { radius, length } => {
                    Box::new(Capsules3D::from_lengths_and_radii([*length as f32], [*radius as f32]).with_colors([color]))
                }
                Geometry::Sphere { radius } => Box::new(Ellipsoids3D::from_radii([*radius as f32]).with_colors([color])),
                Geometry::Mesh { .. } => unreachable!(),
            };
            out.geometry(entity, &[&poses, &CoordinateFrame::new(frame.clone()), shape.as_ref()]);
        };
        match which {
            Geometries::Visual => {
                for (entity, visual) in tree.get_visual_geometries(link).unwrap_or_default() {
                    // An inline colour or texture wins over the named material.
                    let material = visual.material.as_ref().and_then(|m| {
                        if m.color.is_some() || m.texture.is_some() { Some(m) } else { loaded.materials.get(&m.name) }
                    });
                    add(entity, &visual.origin, &visual.geometry, material);
                }
            }
            Geometries::Collision => {
                for (entity, collision) in tree.get_collision_geometries(link).unwrap_or_default() {
                    add(entity, &collision.origin, &collision.geometry, None);
                }
            }
        }
        for joint in tree.get_children(&link.name).into_iter().flatten() {
            links.push(tree.get_joint_child(joint));
        }
    }
}

/// The URDF material's colour, if it gives one (as Rerun's importer reads it).
fn material_albedo(material: Option<&Material>) -> Option<Rgba32> {
    let urdf_rs::Vec4([r, g, b, a]) = material?.color.as_ref()?.rgba;
    Some(Rgba32::from_linear_unmultiplied_rgba_f32(r as f32, g as f32, b as f32, a as f32))
}

fn quat_from_rpy(rpy: &[f64; 3]) -> glam::Quat {
    glam::Quat::from_euler(glam::EulerRot::ZYX, rpy[2] as f32, rpy[1] as f32, rpy[0] as f32)
}
