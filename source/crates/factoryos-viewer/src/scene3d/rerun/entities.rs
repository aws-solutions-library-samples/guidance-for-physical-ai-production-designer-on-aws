//! Names (entity paths, transform frames) and the archetypes of plain nodes: primitives and meshes.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use web_time::Instant;

use factoryos_client::protocol::scene::{AssetRef, MeshAsset, Node, NodeKind, Parent, Pose3, Rgba};

use crate::rr::re_chunk::{Chunk, EntityPath, RowId, TimePoint};
use crate::rr::re_log;
use crate::rr::re_sdk_types::archetypes::{
    Arrows3D, Asset3D, Boxes3D, Clear, CoordinateFrame, Cylinders3D, InstancePoses3D, LineStrips3D, Mesh3D, Points3D,
    Transform3D,
};
use crate::rr::re_sdk_types::components::{Color, FillMode, MediaType};
use crate::rr::re_sdk_types::datatypes::{Rgba32, UVec3D};
use crate::rr::re_sdk_types::AsComponents;

use super::super::{AssetSource, AssetState, Priority, Waker};
use super::framing::MeshBounds;

/// The scene's world frame: Rerun's implicit frame of the root entity `/`.
pub const WORLD_FRAME: &str = "tf#/";

/// A node's entity: `/<node id>`.
pub fn entity(id: &str) -> EntityPath {
    EntityPath::parse_forgiving(id)
}

/// The transform frames of the node instances shown: node `id`'s frame is `id`, or `id@<n>` for
/// its `n`th instance in this recording; a model's link frames are `<node frame>#<link>`.
///
/// Rerun keeps static transforms of cleared entities (`re_tf` ignores static clears), so a frame
/// name, once logged, stays defined at its last pose. A node that is removed, or a model whose
/// URDF is loaded again, therefore takes a new generation: frames of the old instance are left
/// unused (no "two entities define the same child frame" when another URDF has the same link
/// names), and children parented to the node (by `Node.parent`, whose placements are logged
/// again then) follow the new instance, or hang in a frame nobody defines while there is none.
#[derive(Default)]
pub struct Frames {
    generations: HashMap<String, u64>,
}

impl Frames {
    /// Node `id`'s frame: that of its current instance, or of the next if there is none.
    pub fn node(&self, id: &str) -> String {
        match self.generations.get(id).copied().unwrap_or(0) {
            0 => id.to_owned(),
            n => format!("{id}@{n}"),
        }
    }

    /// The frame of link `link` of model node `id`.
    pub fn link(&self, id: &str, link: &str) -> String {
        link_frame(&self.node(id), link)
    }

    /// The frame a node's pose is relative to.
    pub fn parent(&self, parent: Option<&Parent>) -> String {
        match parent {
            None => WORLD_FRAME.to_owned(),
            Some(Parent { node, link: None }) => self.node(node),
            Some(Parent { node, link: Some(link) }) => self.link(node, link),
        }
    }

    /// Retire node `id`'s frames (its instance is gone or replaced).
    pub fn retire(&mut self, id: &str) {
        *self.generations.entry(id.to_owned()).or_default() += 1;
    }

    /// The transform that places `node`'s frame in its parent's.
    pub fn placement(&self, node: &Node) -> Transform3D {
        pose_transform(&node.pose, self.parent(node.parent.as_ref()), self.node(&node.id))
    }
}

/// The frame of link `link` of the model whose node frame is `node_frame`: `<node frame>#<link>`.
pub fn link_frame(node_frame: &str, link: &str) -> String {
    format!("{}{link}", link_frame_prefix(node_frame))
}

/// The prefix of a model's link frames (`UrdfTree::with_frame_prefix`).
pub fn link_frame_prefix(node_frame: &str) -> String {
    format!("{node_frame}#")
}

pub fn pose_transform(pose: &Pose3, parent: String, child: String) -> Transform3D {
    Transform3D::update_fields()
        .with_translation(pose.xyz.map(|v| v as f32))
        .with_quaternion(pose.quat_xyzw.map(|v| v as f32))
        .with_parent_frame(parent)
        .with_child_frame(child)
}

pub fn color(rgba: Rgba) -> Color {
    Color::from_unmultiplied_rgba(rgba[0], rgba[1], rgba[2], rgba[3])
}

fn colors(rgba: &[Rgba]) -> Vec<Color> {
    rgba.iter().copied().map(color).collect()
}

fn f32s<const N: usize>(v: &[[f64; N]]) -> Vec<[f32; N]> {
    v.iter().map(|p| p.map(|x| x as f32)).collect()
}

/// RGB axes, as the Robo Dresser draws a TCP.
const AXIS_COLORS: [u32; 3] = [0xE5_39_35_FF, 0x43_A0_47_FF, 0x1E_88_E5_FF];

/// The archetype that draws a primitive node (in the node's own frame), `None` for meshes, models
/// and groups. Without `show_text` (the view's "Labels" toggle off) its text labels are left out,
/// also those shown only on hover.
pub fn primitive(kind: &NodeKind, show_text: bool) -> Option<Box<dyn AsComponents>> {
    Some(match kind {
        NodeKind::Model { .. } | NodeKind::Mesh { .. } | NodeKind::Group => return None,
        NodeKind::Boxes { centers, half_sizes, quats_xyzw, colors: c, labels, solid, show_labels } => {
            let mut boxes = Boxes3D::from_centers_and_half_sizes(f32s(centers), f32s(half_sizes));
            if !quats_xyzw.is_empty() {
                boxes = boxes.with_quaternions(f32s(quats_xyzw));
            }
            if !c.is_empty() {
                boxes = boxes.with_colors(colors(c));
            }
            if show_text && !labels.is_empty() {
                boxes = boxes.with_labels(labels.clone());
            }
            if *solid {
                boxes = boxes.with_fill_mode(FillMode::Solid);
            }
            if let Some(show) = show_labels.filter(|_| show_text) {
                boxes = boxes.with_show_labels(show);
            }
            Box::new(boxes)
        }
        NodeKind::Cylinder { height, radius, center, color: c, solid } => {
            let mut cylinder = Cylinders3D::from_lengths_and_radii([*height as f32], [*radius as f32])
                .with_centers([center.map(|v| v as f32)])
                .with_fill_mode(if *solid { FillMode::Solid } else { FillMode::MajorWireframe });
            if let Some(c) = c {
                cylinder = cylinder.with_colors([color(*c)]);
            }
            Box::new(cylinder)
        }
        NodeKind::Points { points, colors: c, labels, radius, show_labels } => {
            let mut pts = Points3D::new(f32s(points));
            if !c.is_empty() {
                pts = pts.with_colors(colors(c));
            }
            if show_text && !labels.is_empty() {
                pts = pts.with_labels(labels.clone());
            }
            if let Some(r) = radius {
                pts = pts.with_radii([*r]);
            }
            if let Some(show) = show_labels.filter(|_| show_text) {
                pts = pts.with_show_labels(show);
            }
            Box::new(pts)
        }
        NodeKind::Lines { strips, colors: c, radius, labels, show_labels } => {
            let mut lines = LineStrips3D::new(strips.iter().map(|s| f32s(s)));
            if !c.is_empty() {
                lines = lines.with_colors(colors(c));
            }
            if let Some(r) = radius {
                lines = lines.with_radii([*r]);
            }
            if show_text && !labels.is_empty() {
                lines = lines.with_labels(labels.clone());
            }
            if let Some(show) = show_labels.filter(|_| show_text) {
                lines = lines.with_show_labels(show);
            }
            Box::new(lines)
        }
        NodeKind::Arrows { origins, vectors, colors: c, labels, radius, show_labels } => {
            let mut arrows = Arrows3D::from_vectors(f32s(vectors)).with_origins(f32s(origins));
            if !c.is_empty() {
                arrows = arrows.with_colors(colors(c));
            }
            if show_text && !labels.is_empty() {
                arrows = arrows.with_labels(labels.clone());
            }
            if let Some(r) = radius {
                arrows = arrows.with_radii([*r]);
            }
            if let Some(show) = show_labels.filter(|_| show_text) {
                arrows = arrows.with_show_labels(show);
            }
            Box::new(arrows)
        }
        NodeKind::TriMesh { vertices, triangles, color: c } => {
            let mut mesh = Mesh3D::new(f32s(vertices)).with_triangle_indices(triangles.iter().map(|t| UVec3D::from(*t)));
            if let Some(c) = c {
                mesh = mesh.with_albedo_factor(Rgba32::from_unmultiplied_rgba(c[0], c[1], c[2], c[3]));
            }
            Box::new(mesh)
        }
        NodeKind::Axes { length, radius } => {
            let l = *length as f32;
            Box::new(
                Arrows3D::from_vectors([[l, 0.0, 0.0], [0.0, l, 0.0], [0.0, 0.0, l]])
                    .with_colors(AXIS_COLORS)
                    .with_radii([radius.unwrap_or((l * 0.03).max(0.0005))]),
            )
        }
    })
}

/// A mesh waiting for its bytes (and its sidecars'), drawn in stages once they are there:
///
/// 1. Its decimated variant ([`MeshAsset::lod`]), if any and if it comes first.
/// 2. The full mesh, on the same entity in the same frame: the self-contained GLB
///    ([`MeshAsset::glb`]) if there is one, else the file ([`MeshAsset::asset`]) with its sidecars.
///
/// A colour override (`albedo`: a URDF `<color>`, a node's `color`) replaces the mesh's own colours,
/// which a GLB's baked materials can't do (a tint would multiply them): such a mesh skips both
/// variants and is drawn from the file, as before (an OBJ's vertex colours replaced, anything
/// else an [`Asset3D`] with the albedo).
///
/// An OBJ (only one without a GLB variant) becomes a [`Mesh3D`] through a [`Converter`], off the
/// UI thread natively.
pub struct PendingMesh {
    /// The node it belongs to (dropped with it).
    pub node: String,
    pub entity: EntityPath,
    pub mesh: MeshAsset,
    /// The name it is known by (a URDF's `<mesh filename>`), for the format if the media type is vague.
    pub name: String,
    pub poses: InstancePoses3D,
    pub frame: String,
    /// A colour that wins over the file's own (a URDF `<color>`, a node's `color`).
    pub albedo: Option<Rgba32>,
    /// A fetch failed (for now): not asked again before then.
    pub retry_at: Option<Instant>,
    /// What was drawn so far.
    lod: Lod,
    /// The full mesh being converted ([`Converter`]); drawn once the slot is filled.
    converting: Option<Slot>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Lod {
    /// Not yet drawn (nor known to fail).
    Wanted,
    /// Drawn (an [`Asset3D`]): the full mesh replaces it.
    Shown,
    /// None, or it failed: only the full mesh is left.
    Skipped,
}

impl PendingMesh {
    pub fn new(node: &str, entity: EntityPath, mut mesh: MeshAsset, name: String, poses: InstancePoses3D, frame: String, albedo: Option<Rgba32>) -> Self {
        if albedo.is_some() {
            // The colour replaces the file's (see above): the GLB variants can't take it.
            mesh.glb = None;
            mesh.lod = None;
        }
        let lod = if mesh.lod.is_some() { Lod::Wanted } else { Lod::Skipped };
        Self { node: node.to_owned(), entity, mesh, name, poses, frame, albedo, retry_at: None, lod, converting: None }
    }

    /// Ask for the decimated variant first (at high priority), if it is still wanted.
    pub fn request_lod(&self, assets: &dyn AssetSource) {
        if let (Lod::Wanted, Some(lod)) = (self.lod, &self.mesh.lod) {
            assets.get_with(lod, Priority::High);
        }
    }

    /// Draws what has arrived into `out`: the decimated variant, then the full mesh. Returns
    /// whether it is done (the full mesh drawn, or failed for good, which leaves the decimated one
    /// if it was drawn). After a failure that may pass (the file, or a sidecar such as its `.mtl`),
    /// it is tried again `retry_after` later.
    ///
    /// The extents of what is drawn go into `bounds` (for the camera, [`super::framing`]).
    pub fn try_emit(
        &mut self,
        assets: &dyn AssetSource,
        convert: &Converter,
        out: &mut Out,
        bounds: &mut MeshBounds,
        now: Instant,
        retry_after: Duration,
    ) -> bool {
        if let Some(slot) = &self.converting {
            let Some(mesh) = slot.lock().unwrap().take() else { return false };
            self.converting = None;
            self.log(out, bounds, &mesh);
            return true;
        }
        if let (Lod::Wanted, Some(lod)) = (self.lod, &self.mesh.lod) {
            match assets.get_with(lod, Priority::High) {
                AssetState::Pending => {}
                AssetState::Ready(bytes) => {
                    bounds.observe_file(&self.mesh, &bytes, MediaType::GLB);
                    self.log(out, bounds, &MeshData::Asset3D(glb_asset(&bytes)));
                    self.lod = Lod::Shown;
                }
                AssetState::Failed { err, .. } => {
                    // Not worth a retry: the full mesh is on its way.
                    re_log::warn_once!("Failed to fetch the decimated mesh {}: {err}", self.name);
                    self.lod = Lod::Skipped;
                }
            }
        }

        // A failed full mesh waits for its retry (a decimated one is drawn meanwhile).
        if self.retry_at.is_some_and(|at| now < at) {
            return false;
        }
        let (asset, glb) = match &self.mesh.glb {
            Some(glb) => (glb, true),
            None => (&self.mesh.asset, false),
        };
        let bytes = match assets.get_with(asset, Priority::Normal) {
            AssetState::Pending => return false,
            AssetState::Failed { err, retryable: true } => {
                re_log::warn_once!("Failed to fetch mesh {} (retrying): {err}", self.name);
                self.retry_at = Some(now + retry_after);
                return false;
            }
            AssetState::Failed { err, retryable: false } => {
                re_log::warn_once!("Failed to fetch mesh {}: {err}", self.name);
                return true;
            }
            AssetState::Ready(bytes) => bytes,
        };
        if glb {
            // Materials baked in: nothing else to fetch or convert.
            bounds.observe_file(&self.mesh, &bytes, MediaType::GLB);
            self.log(out, bounds, &MeshData::Asset3D(glb_asset(&bytes)));
            return true;
        }
        let mut sidecars = BTreeMap::new();
        for (name, asset) in &self.mesh.sidecars {
            match assets.get(asset) {
                AssetState::Pending => return false,
                AssetState::Failed { err, retryable: true } => {
                    re_log::warn_once!("Failed to fetch {name} (of mesh {}, retrying): {err}", self.name);
                    self.retry_at = Some(now + retry_after);
                    return false;
                }
                AssetState::Failed { err, retryable: false } => {
                    re_log::warn_once!("Failed to fetch {name} (of mesh {}): {err}", self.name);
                }
                AssetState::Ready(bytes) => {
                    sidecars.insert(name.clone(), bytes);
                }
            }
        }
        let media_type = media_type(&self.mesh.asset, &self.name);
        if media_type.as_ref().is_some_and(|m| m.as_str() == MediaType::OBJ) {
            let slot = convert.obj(bytes, sidecars, self.albedo);
            let converted = slot.lock().unwrap().take();
            match converted {
                Some(mesh) => self.log(out, bounds, &mesh),
                None => {
                    self.converting = Some(slot);
                    return false;
                }
            }
            return true;
        }
        if let Some(media_type) = &media_type {
            bounds.observe_file(&self.mesh, &bytes, media_type.as_str());
        }
        let mut mesh = Asset3D::from_file_contents(bytes.to_vec(), media_type);
        if let Some(albedo) = self.albedo {
            mesh = mesh.with_albedo_factor(albedo);
        }
        self.log(out, bounds, &MeshData::Asset3D(mesh));
        true
    }

    /// Draw `mesh` in place of what was drawn before.
    fn log(&self, out: &mut Out, bounds: &mut MeshBounds, mesh: &MeshData) {
        if let MeshData::Mesh3D(mesh3d) = mesh {
            bounds.observe_mesh3d(&self.mesh, mesh3d);
        }
        if self.lod == Lod::Shown {
            // The decimated GLB's blob (and media type) go; whatever replaces it sets its own.
            out.geometry(self.entity.clone(), &[&Asset3D::clear_fields()]);
        }
        out.geometry(self.entity.clone(), &[&self.poses, &CoordinateFrame::new(self.frame.clone()), mesh]);
    }
}

/// A self-contained GLB, materials baked in (only for meshes without a colour override).
fn glb_asset(bytes: &[u8]) -> Asset3D {
    Asset3D::from_file_contents(bytes.to_vec(), Some(MediaType::glb()))
}

/// A mesh's archetype, as sent between threads.
pub enum MeshData {
    Mesh3D(Mesh3D),
    Asset3D(Asset3D),
}

impl AsComponents for MeshData {
    fn as_serialized_batches(&self) -> Vec<crate::rr::re_sdk_types::SerializedComponentBatch> {
        match self {
            Self::Mesh3D(mesh) => mesh.as_serialized_batches(),
            Self::Asset3D(asset) => asset.as_serialized_batches(),
        }
    }
}

/// An OBJ mesh: `obj` is the file, `sidecars` the files it references (its `mtllib`, the MTL's
/// textures) by the name written in the referencing file, `albedo` a colour that wins over the
/// file's materials.
///
/// A [`Mesh3D`] with the `.mtl`'s colours as vertex colours (factoryos-urdf-materials, as for
/// URDFs opened from disk); `albedo` replaces them. Should the OBJ not parse, Rerun's own loader
/// gets to try (white unless `albedo`).
pub fn obj_mesh(obj: &[u8], sidecars: &BTreeMap<String, Arc<[u8]>>, albedo: Option<Rgba32>) -> MeshData {
    let load_sidecar = |name: &str| sidecars.get(name).map(|bytes| bytes.to_vec());
    match factoryos_urdf_materials::obj_to_mesh3d(obj, &load_sidecar, factoryos_urdf_materials::ColorSpace::Srgb) {
        Ok(mut mesh) => {
            if let Some(albedo) = albedo {
                mesh.vertex_colors = None;
                mesh = mesh.with_albedo_factor(albedo);
            }
            return MeshData::Mesh3D(mesh);
        }
        Err(err) => re_log::warn_once!("Failed to read an OBJ mesh (Rerun's loader tries): {err:#}"),
    }
    let mut mesh = Asset3D::from_file_contents(obj.to_vec(), Some(MediaType::obj()));
    if let Some(albedo) = albedo {
        mesh = mesh.with_albedo_factor(albedo);
    }
    MeshData::Asset3D(mesh)
}

/// Where a conversion's result arrives.
type Slot = Arc<Mutex<Option<MeshData>>>;

/// Turns OBJ+MTL into [`Mesh3D`]s ([`obj_mesh`]), which takes long for big meshes.
///
/// Natively on a few shared worker threads ([`workers`]): the result arrives in a slot that
/// [`PendingMesh::try_emit`] looks at again once the waker has woken the UI. On the web, inline:
/// there are no threads to spare, and once the backend converts meshes to GLB
/// ([`MeshAsset::glb`]) only legacy OBJs without a GLB variant still come this way.
#[derive(Clone, Default)]
pub struct Converter {
    inline: bool,
    waker: Waker,
}

impl Converter {
    /// On the worker threads natively, inline on the web.
    pub fn new() -> Self {
        Self { inline: cfg!(target_arch = "wasm32"), waker: Waker::default() }
    }

    /// Converts on the calling thread.
    #[cfg(test)]
    pub fn inline() -> Self {
        Self { inline: true, waker: Waker::default() }
    }

    #[cfg(test)]
    pub fn is_inline(&self) -> bool {
        self.inline
    }

    /// Woken whenever a conversion is done.
    pub fn set_waker(&mut self, waker: Waker) {
        self.waker = waker;
    }

    /// Start converting; the result comes in the slot (filled at once when inline).
    fn obj(&self, obj: Arc<[u8]>, sidecars: BTreeMap<String, Arc<[u8]>>, albedo: Option<Rgba32>) -> Slot {
        let slot = Slot::default();
        let done = slot.clone();
        let waker = self.waker.clone();
        let job: Job = Box::new(move || {
            *done.lock().unwrap() = Some(obj_mesh(&obj, &sidecars, albedo));
            waker.wake();
        });
        if self.inline {
            job();
        } else {
            run_on_worker(job);
        }
        slot
    }
}

type Job = Box<dyn FnOnce() + Send>;

/// Natively: on one of [`workers`] threads (inline if there are none).
fn run_on_worker(job: Job) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        static WORKERS: std::sync::OnceLock<Option<std::sync::mpsc::Sender<Job>>> = std::sync::OnceLock::new();
        let workers = WORKERS.get_or_init(|| {
            let (tx, rx) = std::sync::mpsc::channel::<Job>();
            let rx = Arc::new(Mutex::new(rx));
            let mut spawned = 0;
            for i in 0..workers() {
                let rx = rx.clone();
                let thread = std::thread::Builder::new().name(format!("scene3d-mesh-{i}")).spawn(move || {
                    loop {
                        // Not holding the lock while a job runs.
                        let job = rx.lock().unwrap().recv();
                        match job {
                            Ok(job) => job(),
                            Err(_) => return,
                        }
                    }
                });
                spawned += usize::from(thread.is_ok());
            }
            (spawned > 0).then_some(tx)
        });
        let job = match workers {
            Some(tx) => match tx.send(job) {
                Ok(()) => return,
                Err(std::sync::mpsc::SendError(job)) => job,
            },
            None => job,
        };
        job();
    }
    #[cfg(target_arch = "wasm32")]
    job();
}

/// Mesh conversion threads: half the cores, 2 to 4.
#[cfg(not(target_arch = "wasm32"))]
fn workers() -> usize {
    std::thread::available_parallelism().map_or(2, |n| n.get() / 2).clamp(2, 4)
}

/// The asset's media type if it names a mesh format, else a guess from `name`'s extension.
fn media_type(asset: &AssetRef, name: &str) -> Option<MediaType> {
    let known = [MediaType::OBJ, MediaType::STL, MediaType::GLB, MediaType::GLTF, MediaType::DAE, MediaType::PLY];
    if known.contains(&asset.media_type.as_str()) {
        return Some(MediaType::from(asset.media_type.clone()));
    }
    let name = name.split(['?', '#']).next().unwrap_or(name);
    MediaType::guess_from_path(name)
}

/// Whether `kind` draws text labels ([`primitive`]): what the "Labels" toggle redraws.
pub fn has_labels(kind: &NodeKind) -> bool {
    match kind {
        NodeKind::Boxes { labels, .. } | NodeKind::Points { labels, .. } | NodeKind::Lines { labels, .. } | NodeKind::Arrows { labels, .. } => {
            !labels.is_empty()
        }
        _ => false,
    }
}

/// Chunks to log, all static. `protected`: never dropped by the [`crate::live_queue::LiveQueue`]
/// (geometry, clears); transforms are not, so that a backlog keeps only the latest pose.
#[derive(Default)]
pub struct Out {
    pub chunks: Vec<(Chunk, bool)>,
}

impl Out {
    fn push(&mut self, entity: EntityPath, archetypes: &[&dyn AsComponents], protected: bool) {
        let batches = archetypes.iter().flat_map(|a| a.as_serialized_batches());
        match Chunk::builder(entity.clone()).with_serialized_batches(RowId::new(), TimePoint::STATIC, batches).build() {
            Ok(chunk) => self.chunks.push((chunk, protected)),
            Err(err) => re_log::warn!("Failed to build a chunk: {err}\nEntity: {entity}"),
        }
    }

    /// Geometry (and its frame/instance poses), kept.
    pub fn geometry(&mut self, entity: EntityPath, archetypes: &[&dyn AsComponents]) {
        self.push(entity, archetypes, true);
    }

    /// A transform, which a newer one of the same entity replaces while queued.
    pub fn transform(&mut self, entity: EntityPath, transform: &Transform3D) {
        self.push(entity, &[transform], false);
    }

    pub fn clear(&mut self, entity: EntityPath, recursive: bool) {
        let clear = if recursive { Clear::recursive() } else { Clear::flat() };
        self.push(entity, &[&clear], true);
    }
}
