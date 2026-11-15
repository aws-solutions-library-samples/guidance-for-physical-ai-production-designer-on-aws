//! The 3D view's default camera: it fits the scene's geometry, with a margin, in the view's
//! actual size.
//!
//! Rerun's own default eye (the `EyeControls3D` fallbacks in `re_view_spatial/src/view_3d.rs`)
//! sits at 1.5 half-diagonals of the scene's bounding box from its centre, whatever the field of
//! view (55° vertically) and the view's aspect ratio: a box's corners need about 2.2 half-diagonals
//! in a square view, and well over 3 in a portrait one (the Product & Task view is ~690 points wide
//! and ~1100 high). So scenes ran off the view's edges.
//!
//! Instead, the view's eye is placed from the scene's extent ([`scene_extent`]: node poses, model
//! links by forward kinematics, primitive and mesh extents) and the 3D view's size ([`view_size`]),
//! see [`fit_points`], and written into the blueprint as the view's `EyeControls3D` position, look
//! target and up ([`super::blueprint`]), so Rerun's own orbital eye starts there and orbits, pans
//! and zooms about the look target: the scene's centre. So:
//!
//! * a new scene, a resync, and a rebase into a fresh recording are framed as soon as their first
//!   batch is in, and again whenever the bounds grow substantially ([`Framing::update`]), e.g. once
//!   models and meshes have arrived, or the view is resized;
//! * and again when what is drawn changed (nodes shown, hidden, added, removed, another model or
//!   mesh: [`content_key`]) and the bounds shrank substantially, e.g. a training job drawn with
//!   fewer robots: poses and joints of the same nodes alone never zoom in;
//! * once the user moves the camera (Rerun writes the eye it moved into the active blueprint),
//!   none of this moves it: the user keeps their camera ([`super::blueprint::EyeSync`]). The
//!   view's reset (double-click) goes back to the default blueprint, which holds the latest fit.
//!
//! (The eye used to be a tracked, invisible `Pinhole` in the recording. Rerun's eye stops tracking
//! when the user drags, and then orbits about a point one unit in front of the tracked pose
//! (`EyeController::copy_from_eye` in `re_view_spatial/src/eye.rs`): in mid-air, not the scene.)

use std::collections::{BTreeMap, HashMap};

use factoryos_client::protocol::scene::{MeshAsset, Node, NodeKind, Parent, Pose3, SceneDoc};

use crate::rr::egui;
use crate::rr::re_sdk_types::archetypes::Mesh3D;
use crate::rr::re_sdk_types::components::Position3D;
use crate::rr::re_sdk_types::external::glam::{Affine3A, Quat, Vec3};
use crate::rr::re_sdk_types::FromArrow as _;

use super::urdf;

/// Rerun's default vertical field of view (`Eye::DEFAULT_FOV_Y`), which its eye uses: the view's
/// own eye has no other.
pub const FOV_Y: f32 = 55.0_f32 * std::f32::consts::PI / 180.0;

/// Kept clear at the view's left and right, and at its top and bottom (points): labels are drawn
/// centred on their points, in screen space.
const MARGIN_X: f32 = 60.0;
const MARGIN_Y: f32 = 30.0;
/// The share of what is left that the scene fills at most.
const FILL: f32 = 0.92;

/// Re-frame when the bounds grow out of the framed ones by more than this share of their diagonal…
const GROWTH: f32 = 0.05;
/// …or the view's width or height changes by more than this share…
const RESIZE: f32 = 0.08;
/// …or what is drawn changed and the bounds' diagonal is less than this share of the framed one's.
const SHRINK: f32 = 0.6;

/// Before the 3D view has been seen (points).
const DEFAULT_VIEW: ViewSize = ViewSize { width: 1000.0, height: 800.0 };

/// An axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn point(p: Vec3) -> Self {
        Self { min: p, max: p }
    }

    pub fn from_points(points: impl IntoIterator<Item = Vec3>) -> Option<Self> {
        let mut points = points.into_iter().filter(|p| p.is_finite());
        let first = Self::point(points.next()?);
        Some(points.fold(first, Self::extend))
    }

    pub fn extend(self, p: Vec3) -> Self {
        Self { min: self.min.min(p), max: self.max.max(p) }
    }

    pub fn union(self, other: Self) -> Self {
        Self { min: self.min.min(other.min), max: self.max.max(other.max) }
    }

    /// Grown by `by` on every side.
    pub fn expand(self, by: f32) -> Self {
        Self { min: self.min - Vec3::splat(by), max: self.max + Vec3::splat(by) }
    }

    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn diagonal(&self) -> f32 {
        (self.max - self.min).length()
    }

    pub fn contains(&self, other: &Self) -> bool {
        self.min.cmple(other.min).all() && self.max.cmpge(other.max).all()
    }

    pub fn corners(&self) -> [Vec3; 8] {
        let (a, b) = (self.min, self.max);
        [
            Vec3::new(a.x, a.y, a.z),
            Vec3::new(b.x, a.y, a.z),
            Vec3::new(a.x, b.y, a.z),
            Vec3::new(b.x, b.y, a.z),
            Vec3::new(a.x, a.y, b.z),
            Vec3::new(b.x, a.y, b.z),
            Vec3::new(a.x, b.y, b.z),
            Vec3::new(b.x, b.y, b.z),
        ]
    }

    /// The box around this one in another frame.
    pub fn transformed(&self, transform: &Affine3A) -> Self {
        let corners = self.corners().map(|c| transform.transform_point3(c));
        Self::from_points(corners).unwrap_or(*self)
    }
}

/// The 3D view's size, in points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewSize {
    pub width: f32,
    pub height: f32,
}

impl ViewSize {
    /// Whether `other` differs by more than [`RESIZE`] in width or height.
    fn differs(&self, other: &Self) -> bool {
        let rel = |a: f32, b: f32| (a - b).abs() > RESIZE * a.max(b);
        rel(self.width, other.width) || rel(self.height, other.height)
    }
}

/// Where the camera is and what it looks at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Eye {
    pub position: Vec3,
    pub look_target: Vec3,
}

/// The direction the camera looks in: Rerun's default (`view_3d.rs`), from the front left, a bit
/// from above, Z up.
fn view_direction() -> Vec3 {
    Vec3::new(0.25, 0.75, -0.25).normalize()
}

/// The camera's right, up and forward axes (world), Z up.
fn camera_axes(forward: Vec3) -> (Vec3, Vec3, Vec3) {
    let right = forward.cross(Vec3::Z).try_normalize().unwrap_or(Vec3::X);
    let up = right.cross(forward);
    (right, up, forward)
}

/// The eye that looks at the centre of `bounds` along [`view_direction`] from just far enough for
/// all of the box to be in a view of `view` points (with [`FOV_Y`]), less its margins.
#[cfg(test)]
pub fn fit(bounds: &Aabb, view: ViewSize) -> Eye {
    fit_points(bounds.center(), &bounds.corners(), view)
}

/// The eye that looks at `center` along [`view_direction`] from just far enough for all of
/// `points` to be in a view of `view` points (with [`FOV_Y`]), less its margins.
pub fn fit_points(center: Vec3, points: &[Vec3], view: ViewSize) -> Eye {
    let forward = view_direction();
    let (right, up, forward) = camera_axes(forward);
    let width = view.width.max(1.0);
    let height = view.height.max(1.0);
    let fill_x = ((width - 2.0 * MARGIN_X) / width).max(0.5) * FILL;
    let fill_y = ((height - 2.0 * MARGIN_Y) / height).max(0.5) * FILL;
    let tan_y = (FOV_Y * 0.5).tan() * fill_y;
    let tan_x = (FOV_Y * 0.5).tan() * (width / height) * fill_x;
    // A corner at (x, y, z) in the camera's axes, relative to the centre, is at depth `d + z` from
    // an eye `d` behind the centre: in view if |x| <= (d + z) tan_x and |y| <= (d + z) tan_y.
    let mut distance = 0.1_f32;
    for corner in points {
        let rel = *corner - center;
        let (x, y, z) = (rel.dot(right), rel.dot(up), rel.dot(forward));
        distance = distance.max(x.abs() / tan_x - z).max(y.abs() / tan_y - z).max(0.05 - z);
    }
    Eye { position: center - forward * distance, look_target: center }
}

/// What the camera is fitted to: the scene's bounds, and the points its geometry is inside of
/// (each part's box corners, [`scene_extent`]): fewer empty corners than the bounds' own, so the
/// scene fills more of the view.
#[derive(Clone, Debug, PartialEq)]
pub struct Extent {
    pub bounds: Aabb,
    pub points: Vec<Vec3>,
}

impl From<Aabb> for Extent {
    fn from(bounds: Aabb) -> Self {
        Self { bounds, points: bounds.corners().to_vec() }
    }
}

/// When the camera is to be placed again (see the module docs).
pub struct Framing {
    /// The 3D view's size, as last seen.
    view: ViewSize,
    /// What the camera was last fitted to, in what size of view, and what was drawn then
    /// ([`content_key`]); `None`: not in this recording yet.
    framed: Option<(Aabb, ViewSize, u64)>,
    /// The bounds may have changed since the last [`Self::update`].
    pub dirty: bool,
    /// Mesh extents, by their file's hash.
    pub meshes: MeshBounds,
}

impl Default for Framing {
    fn default() -> Self {
        Self { view: DEFAULT_VIEW, framed: None, dirty: true, meshes: MeshBounds::default() }
    }
}

impl Framing {
    /// A new recording, or the scene logged again: the camera is logged at the next update.
    pub fn restart(&mut self) {
        self.framed = None;
        self.dirty = true;
    }

    /// The bounds the eye was last fitted to (`None`: not since the last [`Self::restart`]).
    pub fn framed_bounds(&self) -> Option<Aabb> {
        self.framed.map(|(bounds, _, _)| bounds)
    }

    /// The 3D view is `view` points large.
    pub fn set_view(&mut self, view: ViewSize) {
        if view.width < 1.0 || view.height < 1.0 {
            return;
        }
        if self.framed.is_some_and(|(_, framed, _)| framed.differs(&view)) {
            self.dirty = true;
        }
        self.view = view;
    }

    /// The eye to log for the scene's current `bounds` with `content` drawn ([`content_key`]),
    /// if the camera is to move: not in the recording yet, the view resized, the bounds grown out
    /// of the framed ones, or shrunk well inside them with other content drawn ([`SHRINK`]).
    ///
    /// The view follows the camera only while it tracks it: once the user has moved the view,
    /// none of this moves it (see the module docs).
    pub fn update(&mut self, extent: Option<Extent>, content: u64) -> Option<(Eye, ViewSize)> {
        self.dirty = false;
        // An empty scene: a metre around the origin.
        let extent = extent.unwrap_or_else(|| Aabb::point(Vec3::ZERO).expand(0.5).into());
        let bounds = extent.bounds;
        let due = match &self.framed {
            None => true,
            Some((framed, view, framed_content)) => {
                view.differs(&self.view)
                    || !framed.expand(GROWTH * framed.diagonal() + 1e-3).contains(&bounds)
                    || (content != *framed_content && bounds.diagonal() < SHRINK * framed.diagonal())
            }
        };
        if !due {
            return None;
        }
        self.framed = Some((bounds, self.view, content));
        Some((fit_points(bounds.center(), &extent.points, self.view), self.view))
    }
}

/// The 3D view's size in points, from the last frame's widgets: Rerun's 3D view is the largest
/// widget that senses clicks and drags, short of the whole window.
pub fn view_size(ctx: &egui::Context) -> Option<ViewSize> {
    let screen = ctx.content_rect();
    let rect = ctx.viewport(|viewport| {
        viewport
            .prev_pass
            .widgets
            .layers()
            .flat_map(|(_, widgets)| widgets.iter())
            .filter(|w| w.enabled && w.sense.senses_click() && w.sense.senses_drag())
            .map(|w| w.rect.intersect(screen))
            .filter(|r| r.is_positive() && r.width() >= 100.0 && r.height() >= 100.0 && r.area() < 0.9 * screen.area())
            .max_by(|a, b| a.area().total_cmp(&b.area()))
    })?;
    Some(ViewSize { width: rect.width(), height: rect.height() })
}

/// Mesh extents in the mesh's own coordinates, by the hash of its file ([`MeshAsset::asset`]:
/// its GLB and decimated variants have the same geometry), as they are drawn.
#[derive(Default)]
pub struct MeshBounds(HashMap<String, Aabb>);

impl MeshBounds {
    pub fn get(&self, mesh: &MeshAsset) -> Option<Aabb> {
        self.0.get(&mesh.asset.sha256).copied()
    }

    /// Mesh `mesh`'s extents, if `bytes` (a file of `media_type`, e.g. `model/gltf-binary`) give
    /// them. A later call (the full mesh after a decimated one) wins.
    pub fn observe_file(&mut self, mesh: &MeshAsset, bytes: &[u8], media_type: &str) {
        let bounds = match media_type {
            "model/gltf-binary" => glb_bounds(bytes),
            "model/gltf+json" => serde_json::from_slice(bytes).ok().and_then(|json| gltf_bounds(&json)),
            "model/obj" => obj_bounds(bytes),
            "model/stl" => stl_bounds(bytes),
            _ => None,
        };
        if let Some(bounds) = bounds {
            self.0.insert(mesh.asset.sha256.clone(), bounds);
        }
    }

    /// Mesh `mesh`'s extents from its converted [`Mesh3D`].
    pub fn observe_mesh3d(&mut self, mesh: &MeshAsset, mesh3d: &Mesh3D) {
        let Some(batch) = &mesh3d.vertex_positions else { return };
        let Ok(positions) = Position3D::from_arrow(batch.array.as_ref()) else { return };
        if let Some(bounds) = Aabb::from_points(positions.iter().map(|p| Vec3::from(p.0.0))) {
            self.0.insert(mesh.asset.sha256.clone(), bounds);
        }
    }
}

/// A binary glTF's extents: its JSON chunk's position accessors' `min`/`max` (required by the
/// spec), through the default scene's node transforms.
pub fn glb_bounds(bytes: &[u8]) -> Option<Aabb> {
    let u32_at = |at: usize| Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?));
    if bytes.get(0..4)? != b"glTF" || u32_at(16)? != 0x4E4F_534A {
        return None;
    }
    let len = u32_at(12)? as usize;
    let json: serde_json::Value = serde_json::from_slice(bytes.get(20..20 + len)?).ok()?;
    gltf_bounds(&json)
}

/// A glTF document's extents (see [`glb_bounds`]).
pub fn gltf_bounds(json: &serde_json::Value) -> Option<Aabb> {
    let empty = Vec::new();
    let array = |v: &serde_json::Value, key: &str| v.get(key).and_then(|a| a.as_array()).unwrap_or(&empty).clone();
    let nodes = array(json, "nodes");
    let meshes = array(json, "meshes");
    let accessors = array(json, "accessors");
    let vec3 = |v: Option<&serde_json::Value>| -> Option<Vec3> {
        let a = v?.as_array()?;
        Some(Vec3::new(a.first()?.as_f64()? as f32, a.get(1)?.as_f64()? as f32, a.get(2)?.as_f64()? as f32))
    };
    let mesh_bounds = |mesh: usize| -> Option<Aabb> {
        let primitives = array(meshes.get(mesh)?, "primitives");
        primitives
            .iter()
            .filter_map(|p| {
                let accessor = accessors.get(p.get("attributes")?.get("POSITION")?.as_u64()? as usize)?;
                Some(Aabb { min: vec3(accessor.get("min"))?, max: vec3(accessor.get("max"))? })
            })
            .reduce(Aabb::union)
    };
    let local = |node: &serde_json::Value| -> Affine3A {
        if let Some(m) = node.get("matrix").and_then(|m| m.as_array()).filter(|m| m.len() == 16) {
            let cols: Vec<f32> = m.iter().map(|v| v.as_f64().unwrap_or(0.0) as f32).collect();
            return Affine3A::from_mat4(crate::rr::re_sdk_types::external::glam::Mat4::from_cols_slice(&cols));
        }
        let t = vec3(node.get("translation")).unwrap_or(Vec3::ZERO);
        let s = vec3(node.get("scale")).unwrap_or(Vec3::ONE);
        let r = node
            .get("rotation")
            .and_then(|r| r.as_array())
            .filter(|r| r.len() == 4)
            .map(|r| Quat::from_array(std::array::from_fn(|i| r[i].as_f64().unwrap_or(0.0) as f32)).normalize())
            .unwrap_or(Quat::IDENTITY);
        Affine3A::from_scale_rotation_translation(s, r, t)
    };

    if nodes.is_empty() {
        return (0..meshes.len()).filter_map(mesh_bounds).reduce(Aabb::union);
    }
    let roots: Vec<usize> = match json.get("scenes").and_then(|s| s.get(json.get("scene").and_then(|s| s.as_u64()).unwrap_or(0) as usize)) {
        Some(scene) => array(scene, "nodes").iter().filter_map(|n| n.as_u64()).map(|n| n as usize).collect(),
        None => {
            let children: std::collections::HashSet<u64> =
                nodes.iter().flat_map(|n| array(n, "children")).filter_map(|c| c.as_u64()).collect();
            (0..nodes.len()).filter(|i| !children.contains(&(*i as u64))).collect()
        }
    };
    let mut bounds: Option<Aabb> = None;
    let mut stack: Vec<(usize, Affine3A, usize)> = roots.into_iter().map(|n| (n, Affine3A::IDENTITY, 0)).collect();
    while let Some((index, parent, depth)) = stack.pop() {
        let Some(node) = nodes.get(index).filter(|_| depth < 64) else { continue };
        let transform = parent * local(node);
        if let Some(mesh) = node.get("mesh").and_then(|m| m.as_u64()).and_then(|m| mesh_bounds(m as usize)) {
            let mesh = mesh.transformed(&transform);
            bounds = Some(bounds.map_or(mesh, |b| b.union(mesh)));
        }
        for child in array(node, "children").iter().filter_map(|c| c.as_u64()) {
            stack.push((child as usize, transform, depth + 1));
        }
    }
    bounds
}

/// An OBJ's vertices' extents.
pub fn obj_bounds(bytes: &[u8]) -> Option<Aabb> {
    let text = std::str::from_utf8(bytes).ok()?;
    Aabb::from_points(text.lines().filter_map(|line| {
        let mut words = line.trim_start().strip_prefix("v ")?.split_whitespace().map(|w| w.parse::<f32>().ok());
        Some(Vec3::new(words.next()??, words.next()??, words.next()??))
    }))
}

/// An STL's vertices' extents (binary or ASCII).
pub fn stl_bounds(bytes: &[u8]) -> Option<Aabb> {
    let count = bytes.get(80..84).map(|c| u32::from_le_bytes(c.try_into().unwrap()) as usize);
    if let Some(count) = count.filter(|n| 84 + 50 * n == bytes.len()) {
        let f = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
        return Aabb::from_points((0..count).flat_map(|i| {
            let tri = 84 + 50 * i + 12;
            (0..3).map(move |v| Vec3::new(f(tri + 12 * v), f(tri + 12 * v + 4), f(tri + 12 * v + 8)))
        }));
    }
    let text = std::str::from_utf8(bytes).ok()?;
    Aabb::from_points(text.lines().filter_map(|line| {
        let mut words = line.trim_start().strip_prefix("vertex")?.split_whitespace().map(|w| w.parse::<f32>().ok());
        Some(Vec3::new(words.next()??, words.next()??, words.next()??))
    }))
}

fn v3(v: &[f64; 3]) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

pub fn pose_affine(pose: &Pose3) -> Affine3A {
    let q = pose.quat_xyzw.map(|v| v as f32);
    Affine3A::from_rotation_translation(Quat::from_array(q).normalize(), v3(&pose.xyz))
}

/// What a primitive or mesh node draws, in its own frame (`mesh`: the mesh's extents, if known).
/// `None` for models and groups (and empty primitives).
pub fn local_bounds(kind: &NodeKind, mesh: Option<Aabb>) -> Option<Aabb> {
    match kind {
        NodeKind::Model { .. } | NodeKind::Group => None,
        NodeKind::Mesh { scale, .. } => {
            let s = v3(scale);
            mesh.map(|m| Aabb::from_points(m.corners().map(|c| c * s)).unwrap_or(m))
        }
        NodeKind::Boxes { centers, half_sizes, quats_xyzw, .. } => {
            let boxes = centers.iter().enumerate().filter_map(|(i, c)| {
                let half = v3(half_sizes.get(i).or(half_sizes.last())?);
                let q = quats_xyzw.get(i).or(quats_xyzw.last()).map(|q| Quat::from_array(q.map(|v| v as f32)).normalize());
                let transform = Affine3A::from_rotation_translation(q.unwrap_or(Quat::IDENTITY), v3(c));
                Some(Aabb { min: -half, max: half }.transformed(&transform))
            });
            boxes.reduce(Aabb::union)
        }
        NodeKind::Cylinder { height, radius, center, .. } => {
            let half = Vec3::new(*radius as f32, *radius as f32, *height as f32 * 0.5);
            Some(Aabb { min: v3(center) - half, max: v3(center) + half })
        }
        NodeKind::Points { points, radius, .. } => {
            Aabb::from_points(points.iter().map(v3)).map(|b| b.expand(radius.unwrap_or(0.0).max(0.0)))
        }
        NodeKind::Lines { strips, .. } => Aabb::from_points(strips.iter().flatten().map(v3)),
        NodeKind::Arrows { origins, vectors, .. } => Aabb::from_points(vectors.iter().enumerate().flat_map(|(i, v)| {
            let origin = origins.get(i).map(v3).unwrap_or(Vec3::ZERO);
            [origin, origin + v3(v)]
        })),
        NodeKind::TriMesh { vertices, .. } => Aabb::from_points(vertices.iter().map(v3)),
        NodeKind::Axes { length, .. } => Some(Aabb { min: Vec3::ZERO, max: Vec3::splat(*length as f32) }),
    }
}

/// The world extents of the scene's working content: its visible nodes' geometry, placed through
/// their parents (models' links by their joint values), less large flat context ([`is_context`]:
/// a floor plan, a ground mesh). A model whose URDF isn't loaded yet and a mesh not drawn yet
/// count as their origin.
#[cfg(test)]
pub fn scene_bounds(doc: &SceneDoc, models: &BTreeMap<String, urdf::Model>, meshes: &MeshBounds) -> Option<Aabb> {
    scene_extent(doc, models, meshes).map(|extent| extent.bounds)
}

/// [`scene_bounds`], and the corners of the boxes of what is in them (each model link's visual,
/// each primitive and mesh), placed: what [`Framing`] fits the camera to.
pub fn scene_extent(doc: &SceneDoc, models: &BTreeMap<String, urdf::Model>, meshes: &MeshBounds) -> Option<Extent> {
    let nodes = node_extents(doc, models, meshes);
    let keep = kept(&nodes.iter().map(|(bounds, _)| *bounds).collect::<Vec<_>>());
    let kept: Vec<_> = nodes.into_iter().zip(keep).filter_map(|(node, keep)| keep.then_some(node)).collect();
    let bounds = kept.iter().map(|(bounds, _)| *bounds).reduce(Aabb::union)?;
    Some(Extent { bounds, points: kept.into_iter().flat_map(|(_, points)| points).collect() })
}

/// What the scene draws, for [`Framing::update`]: its visible nodes, their kinds, and the models
/// and meshes they draw (by file). Poses, joints, colours and labels don't change it.
pub fn content_key(doc: &SceneDoc) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for node in doc.nodes.iter().filter(|node| node.visible) {
        node.id.hash(&mut hasher);
        std::mem::discriminant(&node.kind).hash(&mut hasher);
        match &node.kind {
            NodeKind::Model { model, .. } => model.urdf.sha256.hash(&mut hasher),
            NodeKind::Mesh { mesh, scale, .. } => {
                mesh.asset.sha256.hash(&mut hasher);
                scale.map(f64::to_bits).hash(&mut hasher);
            }
            _ => {}
        }
    }
    hasher.finish()
}

/// A node at most this thick, relative to its larger horizontal extent, is flat…
const FLAT: f32 = 0.1;
/// …and flat context if its footprint's diagonal is more than this many times that of the rest.
const CONTEXT: f32 = 2.0;

/// At most [`FLAT`] of its larger horizontal extent thick (and not a point).
fn is_flat(b: &Aabb) -> bool {
    let size = b.max - b.min;
    let wide = size.x.max(size.y);
    wide > 0.0 && size.z <= FLAT * wide
}

/// Whether `node` is context, not content, next to `content` (the non-flat nodes' bounds): flat
/// ([`is_flat`]) and much larger: a footprint diagonal over [`CONTEXT`] times that of `content`
/// (at least half a metre). A floor plan or ground mesh around a cell is; a small flat part, a
/// table the robots stand around, or a scene of nothing but flat things is not.
pub fn is_context(node: &Aabb, content: Option<&Aabb>) -> bool {
    let footprint = |b: &Aabb| (b.max - b.min).truncate().length();
    content.is_some_and(|content| is_flat(node) && footprint(node) > CONTEXT * footprint(content).max(0.5))
}

/// The union of `bounds` less the context ([`is_context`]); all of it if that is all there is.
#[cfg(test)]
pub fn without_context(bounds: Vec<Aabb>) -> Option<Aabb> {
    let keep = kept(&bounds);
    bounds.into_iter().zip(keep).filter_map(|(b, keep)| keep.then_some(b)).reduce(Aabb::union)
}

/// Which of `bounds` count: not context ([`is_context`]); all of them if that is all there is.
fn kept(bounds: &[Aabb]) -> Vec<bool> {
    let content = bounds.iter().filter(|b| !is_flat(b)).copied().reduce(Aabb::union);
    let keep: Vec<bool> = bounds.iter().map(|b| !is_context(b, content.as_ref())).collect();
    if keep.iter().any(|k| *k) { keep } else { vec![true; bounds.len()] }
}

/// Each visible node's world bounds and the corners of its parts' boxes, placed (see
/// [`scene_extent`]).
fn node_extents(doc: &SceneDoc, models: &BTreeMap<String, urdf::Model>, meshes: &MeshBounds) -> Vec<(Aabb, Vec<Vec3>)> {
    let nodes: HashMap<&str, &Node> = doc.nodes.iter().map(|node| (node.id.as_str(), node)).collect();
    let mut placements = Placements { nodes, models, world: HashMap::new(), links: HashMap::new() };
    let mut extents = Vec::new();
    for node in doc.nodes.iter().filter(|node| node.visible) {
        let world = placements.world(&node.id, 0);
        let local: Option<Vec<Vec3>> = match &node.kind {
            NodeKind::Model { .. } => models.get(&node.id).and_then(|model| model.visual_corners(meshes)),
            NodeKind::Mesh { mesh, .. } => local_bounds(&node.kind, meshes.get(mesh)).map(|b| b.corners().to_vec()),
            kind => local_bounds(kind, None).map(|b| b.corners().to_vec()),
        };
        let points: Vec<Vec3> = match local {
            Some(local) if !local.is_empty() => local.into_iter().map(|p| world.transform_point3(p)).collect(),
            _ => vec![world.translation.into()],
        };
        let Some(bounds) = Aabb::from_points(points.iter().copied()) else { continue };
        extents.push((bounds, points));
    }
    extents
}

/// Node placements in the world, memoised.
struct Placements<'a> {
    nodes: HashMap<&'a str, &'a Node>,
    models: &'a BTreeMap<String, urdf::Model>,
    world: HashMap<String, Affine3A>,
    /// Model links in the model node's frame, by model.
    links: HashMap<String, HashMap<String, Affine3A>>,
}

impl Placements<'_> {
    fn world(&mut self, id: &str, depth: usize) -> Affine3A {
        if let Some(world) = self.world.get(id) {
            return *world;
        }
        let Some(node) = self.nodes.get(id).copied() else { return Affine3A::IDENTITY };
        let parent = match &node.parent {
            // A cycle, or one deeper than any real scene: the world.
            _ if depth > 64 => Affine3A::IDENTITY,
            None => Affine3A::IDENTITY,
            Some(Parent { node: parent, link }) => {
                let parent_world = self.world(parent, depth + 1);
                let link = link.as_ref().and_then(|link| self.link(parent, link));
                parent_world * link.unwrap_or(Affine3A::IDENTITY)
            }
        };
        let world = parent * pose_affine(&node.pose);
        self.world.insert(id.to_owned(), world);
        world
    }

    fn link(&mut self, model: &str, link: &str) -> Option<Affine3A> {
        if !self.links.contains_key(model) {
            let poses = self.models.get(model)?.link_poses()?;
            self.links.insert(model.to_owned(), poses);
        }
        self.links.get(model)?.get(link).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where `p` lands in a view of `view` seen from `eye` with [`FOV_Y`]: x right, y up, in
    /// points from the centre, and its depth.
    fn project(eye: &Eye, view: ViewSize, p: Vec3) -> (f32, f32, f32) {
        let (right, up, forward) = camera_axes((eye.look_target - eye.position).normalize());
        let rel = p - eye.position;
        let depth = rel.dot(forward);
        let focal = view.height * 0.5 / (FOV_Y * 0.5).tan();
        (rel.dot(right) / depth * focal, rel.dot(up) / depth * focal, depth)
    }

    /// The largest share of the half-width and half-height any corner of `bounds` takes.
    fn extent(eye: &Eye, view: ViewSize, bounds: &Aabb) -> (f32, f32) {
        bounds.corners().iter().fold((0.0_f32, 0.0_f32), |(ex, ey), c| {
            let (x, y, depth) = project(eye, view, *c);
            assert!(depth > 0.0, "{c} behind the eye");
            (ex.max(x.abs() / (view.width * 0.5)), ey.max(y.abs() / (view.height * 0.5)))
        })
    }

    fn robot_bounds() -> Aabb {
        // A 3 m arm along X on a 1 m base.
        Aabb { min: Vec3::new(-0.5, -0.5, 0.0), max: Vec3::new(3.0, 0.5, 2.0) }
    }

    #[test]
    fn the_eye_fits_the_box_with_a_margin_in_any_aspect_ratio() {
        for view in [
            ViewSize { width: 1000.0, height: 800.0 },
            ViewSize { width: 686.0, height: 1180.0 },
            ViewSize { width: 1900.0, height: 500.0 },
            ViewSize { width: 300.0, height: 300.0 },
        ] {
            for bounds in [robot_bounds(), Aabb { min: Vec3::splat(-0.01), max: Vec3::splat(0.01) }, Aabb::point(Vec3::ONE).expand(0.3)] {
                let eye = fit(&bounds, view);
                assert_eq!(eye.look_target, bounds.center());
                let (ex, ey) = extent(&eye, view, &bounds);
                let room_x = (1.0 - 2.0 * MARGIN_X / view.width).max(0.5) * FILL;
                let room_y = (1.0 - 2.0 * MARGIN_Y / view.height).max(0.5) * FILL;
                assert!(ex <= room_x + 1e-4 && ey <= room_y + 1e-4, "{view:?} {bounds:?}: {ex} {ey}");
                if bounds.diagonal() > 0.5 {
                    // Not needlessly far: one side is (nearly) filled.
                    assert!((ex / room_x).max(ey / room_y) > 0.9, "{view:?} {bounds:?}: {ex} {ey}");
                }
            }
        }
    }

    #[test]
    fn a_narrow_view_steps_back_further() {
        let wide = fit(&robot_bounds(), ViewSize { width: 1600.0, height: 900.0 });
        let narrow = fit(&robot_bounds(), ViewSize { width: 686.0, height: 1180.0 });
        let distance = |eye: &Eye| eye.position.distance(eye.look_target);
        assert!(distance(&narrow) > 1.5 * distance(&wide), "{} vs {}", distance(&narrow), distance(&wide));
        // Rerun's default (1.5 half-diagonals) is too close for either.
        let rerun = 1.5 * robot_bounds().diagonal() * 0.5;
        assert!(distance(&wide) > rerun);
    }

    #[test]
    fn the_eye_looks_from_the_front_left_and_above() {
        let eye = fit(&robot_bounds(), DEFAULT_VIEW);
        let from = eye.position - eye.look_target;
        assert!(from.x < 0.0 && from.y < 0.0 && from.z > 0.0, "{from}");
    }

    #[test]
    fn framing_follows_growth_and_resizes_only() {
        let mut framing = Framing::default();
        let small = Aabb { min: Vec3::ZERO, max: Vec3::ONE };
        assert!(framing.update(Some(small.into()), 1).is_some(), "not in the recording yet");
        assert!(framing.update(Some(small.into()), 1).is_none());
        let shrunk = Aabb { min: Vec3::ZERO, max: Vec3::splat(0.5) };
        assert!(framing.update(Some(shrunk.into()), 1).is_none(), "shrinking by moving the same content keeps the camera");
        let nudged = Aabb { min: Vec3::ZERO, max: Vec3::new(1.05, 1.0, 1.0) };
        assert!(framing.update(Some(nudged.into()), 1).is_none(), "within the tolerance");
        let grown = Aabb { min: Vec3::ZERO, max: Vec3::new(3.0, 1.0, 1.0) };
        let (eye, _) = framing.update(Some(grown.into()), 1).expect("grown");
        assert_eq!(eye.look_target, grown.center());

        framing.set_view(ViewSize { width: 1010.0, height: 790.0 });
        assert!(framing.update(Some(grown.into()), 1).is_none(), "about the same size");
        framing.set_view(ViewSize { width: 600.0, height: 800.0 });
        assert!(framing.dirty);
        let (_, view) = framing.update(Some(grown.into()), 1).expect("resized");
        assert_eq!(view.width, 600.0);

        framing.restart();
        assert!(framing.update(Some(grown.into()), 1).is_some(), "a new recording");
        assert!(framing.update(None, 1).is_some(), "an empty scene: around the origin");
    }

    #[test]
    fn the_eye_fits_the_parts_not_the_box_around_them() {
        // Two robots' worth of boxes on a diagonal: the box around both is mostly empty.
        let a = Aabb { min: Vec3::new(-4.0, -4.0, 0.0), max: Vec3::new(-3.0, -3.0, 2.0) };
        let b = Aabb { min: Vec3::new(3.0, 3.0, 0.0), max: Vec3::new(4.0, 4.0, 2.0) };
        let union = a.union(b);
        let points: Vec<Vec3> = a.corners().into_iter().chain(b.corners()).collect();
        let parts = fit_points(union.center(), &points, DEFAULT_VIEW);
        let around = fit(&union, DEFAULT_VIEW);
        assert_eq!(parts.look_target, around.look_target);
        assert!(parts.position.distance(parts.look_target) < 0.9 * around.position.distance(around.look_target), "closer: {parts:?} {around:?}");
        for part in [a, b] {
            let (ex, ey) = extent(&parts, DEFAULT_VIEW, &part);
            assert!(ex <= FILL + 1e-3 && ey <= FILL + 1e-3, "{part:?} in view: {ex} {ey}");
        }
        // As framed: by the parts' corners.
        let mut framing = Framing::default();
        let (eye, _) = framing.update(Some(Extent { bounds: union, points }), 1).unwrap();
        assert_eq!(eye, parts);
    }

    #[test]
    fn framing_follows_a_smaller_drawing() {
        let mut framing = Framing::default();
        let big = Aabb { min: Vec3::ZERO, max: Vec3::new(10.0, 10.0, 2.0) };
        assert!(framing.update(Some(big.into()), 1).is_some());
        // Other content in about the same bounds: the camera stays.
        let most = Aabb { min: Vec3::ZERO, max: Vec3::new(8.0, 8.0, 2.0) };
        assert!(framing.update(Some(most.into()), 2).is_none(), "shrunk, but not by much");
        // Other content well inside them: framed again, to what is drawn now.
        let small = Aabb { min: Vec3::ZERO, max: Vec3::new(2.0, 2.0, 2.0) };
        let (eye, _) = framing.update(Some(small.into()), 3).expect("a smaller drawing");
        assert_eq!(eye.look_target, small.center());
        assert!(framing.update(Some(small.into()), 3).is_none(), "once");
        // The same content, posed into smaller bounds still: never.
        let folded = Aabb { min: Vec3::ZERO, max: Vec3::splat(0.5) };
        assert!(framing.update(Some(folded.into()), 3).is_none(), "poses and joints alone don't zoom in");
        // Unchanged content compares with what was drawn when framed, not at the last update.
        assert!(framing.update(Some(small.into()), 4).is_none());
        assert!(framing.update(Some(folded.into()), 4).is_some(), "other content, shrunk well inside");
    }

    #[test]
    fn the_content_key_follows_what_is_drawn_not_where() {
        let doc = SceneDoc {
            scene_id: "s".into(),
            nodes: vec![Node::new("a", NodeKind::Group), Node::new("b", NodeKind::Group)],
            ..SceneDoc::default()
        };
        let key = content_key(&doc);
        let mut moved = doc.clone();
        moved.nodes[1].pose = Pose3::from_xyz([5.0, 0.0, 0.0]);
        assert_eq!(content_key(&moved), key, "a pose");
        let mut hidden = doc.clone();
        hidden.nodes[1].visible = false;
        assert_ne!(content_key(&hidden), key, "a node hidden");
        let mut removed = doc.clone();
        removed.nodes.pop();
        assert_eq!(content_key(&removed), content_key(&hidden), "hidden or removed: the same drawing");
    }

    #[test]
    fn primitives_bound_what_they_draw() {
        let points = NodeKind::Points { points: vec![[1.0, 2.0, 3.0], [-1.0, 0.0, 0.0]], colors: vec![], labels: vec![], radius: Some(0.1), show_labels: None };
        assert_eq!(local_bounds(&points, None), Some(Aabb { min: Vec3::new(-1.1, -0.1, -0.1), max: Vec3::new(1.1, 2.1, 3.1) }));
        let arrows = NodeKind::Arrows { origins: vec![[1.0, 0.0, 0.0]], vectors: vec![[0.0, 0.0, -2.0]], colors: vec![], labels: vec![], radius: None, show_labels: None };
        assert_eq!(local_bounds(&arrows, None), Some(Aabb { min: Vec3::new(1.0, 0.0, -2.0), max: Vec3::new(1.0, 0.0, 0.0) }));
        let rotated = NodeKind::Boxes {
            centers: vec![[0.0; 3]],
            half_sizes: vec![[1.0, 0.1, 0.1]],
            // 90 degrees about Z.
            quats_xyzw: vec![[0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2]],
            colors: vec![],
            labels: vec![],
            solid: true,
            show_labels: None,
        };
        let b = local_bounds(&rotated, None).unwrap();
        assert!((b.max - Vec3::new(0.1, 1.0, 0.1)).abs().max_element() < 1e-5, "{b:?}");
        let mesh = NodeKind::Mesh { mesh: MeshAsset::default(), scale: [2.0, 1.0, 1.0], color: None };
        assert_eq!(local_bounds(&mesh, Some(Aabb { min: Vec3::splat(-1.0), max: Vec3::ONE })).map(|b| b.max), Some(Vec3::new(2.0, 1.0, 1.0)));
        assert_eq!(local_bounds(&mesh, None), None, "not drawn yet");
        assert_eq!(local_bounds(&NodeKind::Group, None), None);
    }

    #[test]
    fn nodes_are_placed_through_their_parents() {
        let cube = |id: &str| NodeKind::Boxes {
            centers: vec![[0.0; 3]],
            half_sizes: vec![[0.5; 3]],
            quats_xyzw: vec![],
            colors: vec![],
            labels: vec![id.to_owned()],
            solid: true,
            show_labels: None,
        };
        let doc = SceneDoc {
            scene_id: "s".into(),
            nodes: vec![
                Node::new("cell", NodeKind::Group).at(Pose3::from_xyz([10.0, 0.0, 0.0])),
                Node::new("cell/part", cube("part")).under("cell", None).at(Pose3::from_xyz([0.0, 0.0, 1.0])),
                Node::new("hidden", cube("hidden")).at(Pose3::from_xyz([100.0, 0.0, 0.0])).hidden(),
            ],
            ..SceneDoc::default()
        };
        let bounds = scene_bounds(&doc, &BTreeMap::new(), &MeshBounds::default()).unwrap();
        assert_eq!(bounds, Aabb { min: Vec3::new(9.5, -0.5, 0.0), max: Vec3::new(10.5, 0.5, 1.5) }, "the group's origin is inside the part");
    }

    #[test]
    fn large_flat_context_does_not_count() {
        let robots = Aabb { min: Vec3::new(0.0, 0.0, 0.0), max: Vec3::new(3.0, 1.0, 2.5) };
        let floor = Aabb { min: Vec3::new(-20.0, -15.0, -0.01), max: Vec3::new(20.0, 15.0, 0.0) };
        let mat = Aabb { min: Vec3::new(0.5, 0.2, 0.0), max: Vec3::new(1.5, 0.8, 0.02) };
        assert_eq!(without_context(vec![robots, floor, mat]), Some(robots), "the floor plan is context, the small mat content");
        assert!(is_context(&floor, Some(&robots)) && !is_context(&mat, Some(&robots)));
        // A floor only a little larger than the cell counts.
        let apron = Aabb { min: Vec3::new(-1.0, -1.0, -0.01), max: Vec3::new(4.0, 2.0, 0.0) };
        assert_eq!(without_context(vec![robots, apron]), Some(robots.union(apron)));
        // Nothing but flat things, or a tall thing as large: all of it.
        assert_eq!(without_context(vec![floor, mat]), Some(floor.union(mat)));
        let hall = Aabb { min: floor.min, max: Vec3::new(20.0, 15.0, 8.0) };
        assert_eq!(without_context(vec![robots, hall]), Some(hall));
        assert_eq!(without_context(vec![]), None);
    }

    #[test]
    fn mesh_files_give_their_extents() {
        let obj = b"v 0 0 0\nv 1 2 3\nvn 9 9 9\nv -1 0 0\nf 1 2 3\n";
        assert_eq!(obj_bounds(obj), Some(Aabb { min: Vec3::new(-1.0, 0.0, 0.0), max: Vec3::new(1.0, 2.0, 3.0) }));

        let mut stl = vec![0u8; 80];
        stl.extend(1u32.to_le_bytes());
        for f in [0.0f32, 0.0, 1.0, /* v */ 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, -3.0, 0.5] {
            stl.extend(f.to_le_bytes());
        }
        stl.extend([0, 0]);
        assert_eq!(stl_bounds(&stl), Some(Aabb { min: Vec3::new(0.0, -3.0, 0.0), max: Vec3::new(2.0, 0.0, 0.5) }));

        // A node translated by +1 in X and scaled by 2 above a mesh from (0,0,0) to (1,1,1).
        let json = serde_json::json!({
            "scene": 0,
            "scenes": [{ "nodes": [0] }],
            "nodes": [{ "translation": [1.0, 0.0, 0.0], "children": [1] }, { "scale": [2.0, 2.0, 2.0], "mesh": 0 }],
            "meshes": [{ "primitives": [{ "attributes": { "POSITION": 0 } }] }],
            "accessors": [{ "min": [0.0, 0.0, 0.0], "max": [1.0, 1.0, 1.0] }],
        });
        let text = serde_json::to_vec(&json).unwrap();
        let mut glb = b"glTF".to_vec();
        glb.extend(2u32.to_le_bytes());
        glb.extend(((20 + text.len()) as u32).to_le_bytes());
        glb.extend((text.len() as u32).to_le_bytes());
        glb.extend(0x4E4F_534Au32.to_le_bytes());
        glb.extend(&text);
        assert_eq!(glb_bounds(&glb), Some(Aabb { min: Vec3::new(1.0, 0.0, 0.0), max: Vec3::new(3.0, 2.0, 2.0) }));
        assert_eq!(glb_bounds(b"not a glb"), None);

        let mut meshes = MeshBounds::default();
        let mesh = MeshAsset::default();
        meshes.observe_file(&mesh, &glb, "model/gltf-binary");
        assert_eq!(meshes.get(&mesh).map(|b| b.max), Some(Vec3::new(3.0, 2.0, 2.0)));
        meshes.observe_file(&mesh, obj, "model/obj");
        assert_eq!(meshes.get(&mesh).map(|b| b.max), Some(Vec3::new(1.0, 2.0, 3.0)), "the full mesh wins");
    }
}
