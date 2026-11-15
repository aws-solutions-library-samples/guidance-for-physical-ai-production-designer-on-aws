//! The only place a neutral scene becomes Rerun data.
//!
//! Each scene (`scene_id`) is a recording of the application `app_id`, logged into one log channel
//! through a [`LiveQueue`] (a backlog of joint values shows only the latest). All of it is static:
//! a live view, no history.
//!
//! Layout ([`entities`]): node `a/b` is entity `/a/b` and transform frame `a/b`; its pose is a
//! `Transform3D` from its parent's frame (the world, a node, or a model's link `<node>#<link>`), so
//! the entity tree need not follow the node tree. A model's links, geometry and joints are below
//! its entity ([`urdf`]). One 3D view shows all of the recording ([`blueprint`]), through an eye
//! fitted to the scene ([`framing`]): what is not shown is not logged (or is cleared).
//!
//! Every drawing entity names its frame (`CoordinateFrame`) and every `Transform3D` has explicit
//! parent and child frames, so Rerun's implicit entity-path hierarchy never places anything: an
//! id says only what a removal takes along. A hidden node (`visible = false`) logs its frame but
//! none of its own geometry; other nodes, its id-children included, are not affected.
//!
//! Edits: a removed node is cleared with everything below it; a replaced one is cleared (only its
//! own entity, and a model's geometry and joints) and logged again, except a model that keeps its
//! URDF and visibility (a new pose, joint values or `show_collision`): only what changed is logged
//! (or cleared: collision geometry turned off).
//! Children keep their entities and frames, which are named by id ([`entities::Frames`]: with a
//! generation, as Rerun keeps static transforms of cleared entities).
//!
//! Pose frames ([`SceneEvent::Poses`]) only move nodes: their placements are logged again, as for
//! an upsert of the same model.
//!
//! Meshes ([`PendingMesh`]): a decimated variant first if there is one, then on the same entity the
//! full mesh (preferably the producer's GLB). A new scene asks for every URDF and decimated mesh
//! before any full mesh ([`super::Priority`]); a model's meshes are known to be visual only once
//! its URDF is parsed (`ModelRef` does not say), so they are asked for then.
//!
//! Local joint values ([`RerunScene::set_local_joints`], the Robo Dresser's sliders) win over the
//! producer's for that joint until it echoes the last one, at most [`LOCAL_HOLD`]: lagging echoes
//! of earlier values would move the slider's joint back.

pub mod blueprint;
mod compact;
mod entities;
mod framing;
mod urdf;

pub use blueprint::ViewEye;
pub use framing::{Aabb, Eye, FOV_Y, view_size};

use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use factoryos_client::protocol::scene::{JointFrame, ModelJoints, Node, NodeKind, PoseFrame, SceneDelta, SceneDoc, is_under};
use web_time::Instant;

use crate::live_queue::LiveQueue;
use crate::rr::re_chunk::{Chunk, EntityPath};
use crate::rr::re_log;
use crate::rr::re_log_channel::{LogReceiver, LogSender, LogSource, log_channel};
use crate::rr::re_log_types::{ApplicationId, LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource};
use crate::rr::re_sdk_types::archetypes::{CoordinateFrame, InstancePoses3D};
use crate::rr::re_sdk_types::datatypes::{Rgba32, Vec3D};
use crate::rr::re_viewer_context::{StoreHub, blueprint_timepoint_for_writes};

use super::{AssetSource, Priority, RouteRequest, SceneEvent, Waker};
use entities::{Converter, Frames, Out, PendingMesh};

/// How long a local joint value holds at most against the producer's (see the module docs).
pub const LOCAL_HOLD: Duration = Duration::from_secs(1);

pub struct RerunScene {
    app_id: ApplicationId,
    tx: LogSender,
    receiver: Option<LogReceiver>,
    /// Messages not yet handed to Rerun.
    queue: LiveQueue,
    blueprint: blueprint::Blueprint,
    /// The scene shown, as edited so far.
    doc: Option<SceneDoc>,
    /// Its recording.
    store: Option<StoreId>,
    /// The app should route to `store` (a new one).
    open_wanted: bool,
    /// Recordings replaced, to be closed.
    closes: Vec<Close>,
    /// The model nodes, by id.
    models: BTreeMap<String, urdf::Model>,
    /// Meshes waiting for their bytes.
    pending: Vec<PendingMesh>,
    /// The node instances' frame names.
    frames: Frames,
    /// Local joint values, by node and joint, and when they were set.
    local: BTreeMap<String, BTreeMap<String, (f64, Instant)>>,
    /// A failed fetch is asked again after this long.
    retry_after: Duration,
    /// OBJ meshes to `Mesh3D`s, off the UI thread natively.
    convert: Converter,
    /// What the recording keeps beyond the scene, and rebasing into a fresh one ([`compact`]).
    compact: compact::Compact,
    /// The view's eye ([`framing`]).
    framing: framing::Framing,
    /// The active blueprint's eye, kept on the fitted one until the user moves it.
    eye_sync: blueprint::EyeSync,
    /// A new scene replaced another: once it is shown, the blueprint is activated again so the
    /// view shows its fitted eye even if the user had moved the old one ([`Self::set_shown`]).
    retrack: bool,
    /// Text labels are drawn ([`Self::set_labels`], the view's "Labels" toggle).
    labels: bool,
}

/// A replaced recording. Closed only once Rerun has ingested everything logged to it: a message
/// still on its way would open it again (and leave it behind).
struct Close {
    store: StoreId,
    stage: Stage,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Stage {
    /// Messages of it may still be in the [`LiveQueue`].
    Queued,
    /// All handed to the log channel.
    Sent,
    /// The channel was drained since: Rerun has them all.
    Ingested,
}

impl RerunScene {
    pub fn new(app_id: &str) -> Self {
        let (tx, receiver) = log_channel(LogSource::Sdk);
        Self {
            app_id: ApplicationId::new_or_unknown(app_id),
            tx,
            receiver: Some(receiver),
            queue: LiveQueue::default(),
            blueprint: blueprint::Blueprint::new(app_id),
            doc: None,
            store: None,
            open_wanted: false,
            closes: Vec::new(),
            models: BTreeMap::new(),
            pending: Vec::new(),
            frames: Frames::default(),
            local: BTreeMap::new(),
            retry_after: super::assets::RETRY_AFTER,
            convert: Converter::new(),
            compact: compact::Compact::default(),
            framing: framing::Framing::default(),
            eye_sync: blueprint::EyeSync::default(),
            retrack: false,
            labels: true,
        }
    }

    /// Woken when work done off the UI thread (a mesh conversion) is ready to be logged.
    pub fn set_waker(&mut self, waker: Waker) {
        self.convert.set_waker(waker);
    }

    pub fn take_receiver(&mut self) -> Option<LogReceiver> {
        self.receiver.take()
    }

    /// Apply `events` (and retry geometry whose assets have arrived since).
    pub fn apply(&mut self, events: Vec<SceneEvent>, assets: &dyn AssetSource) {
        for event in events {
            let mut out = Out::default();
            match event {
                SceneEvent::Reset(doc) => {
                    self.reset(doc, &mut out);
                    self.prefetch(assets);
                }
                SceneEvent::Delta(delta) => self.edit(&delta, &mut out),
                SceneEvent::Joints(frame) => {
                    let frame = self.without_held(frame, Instant::now());
                    self.joints(&frame, &mut out);
                }
                SceneEvent::Poses(frame) => self.poses(&frame, &mut out),
            }
            self.flush(out);
        }
        self.retry(assets);
        self.rebase_if_due(assets);
        self.switch_when_loaded();
        self.frame_view();
        self.sync_blueprint();
    }

    /// Whether the 3D view shows this scene's recording now. A new scene that replaced another is
    /// framed again once shown, even if the user had moved the camera ([`blueprint::Blueprint::activate`],
    /// only safe while shown); edits, poses and joints never move a camera the user moved.
    pub fn set_shown(&mut self, shown: bool) {
        if !(shown && self.retrack) {
            return;
        }
        if let Some(msg) = self.blueprint.activate() {
            self.queue.push(msg, true);
            self.retrack = false;
        }
    }

    /// Draw the nodes' text labels or not (the view's "Labels" toggle): a node with labels is
    /// cleared and drawn again with or without them, here, without the producer.
    pub fn set_labels(&mut self, on: bool) {
        if self.labels == on {
            return;
        }
        self.labels = on;
        let Some(doc) = &self.doc else { return };
        let labelled: Vec<Node> = doc.nodes.iter().filter(|node| node.visible && entities::has_labels(&node.kind)).cloned().collect();
        let mut out = Out::default();
        for node in &labelled {
            // Its own entity only: a primitive has no geometry below it.
            out.clear(entities::entity(&node.id), false);
            self.add(node, &mut out);
        }
        self.flush(out);
    }

    /// Whether text labels are drawn ([`Self::set_labels`]).
    pub fn labels(&self) -> bool {
        self.labels
    }

    /// The 3D view is `width` x `height` points large: the camera is fitted to that.
    pub fn set_view_size(&mut self, width: f32, height: f32) {
        self.framing.set_view(framing::ViewSize { width, height });
    }

    /// Place the view's eye again if the scene's bounds grew out of what it was fitted to, or
    /// shrank well inside them with other content drawn (or it isn't fitted to this recording
    /// yet, or the view was resized); see [`framing`].
    fn frame_view(&mut self) {
        if !self.framing.dirty || self.store.is_none() {
            return;
        }
        let Some(doc) = &self.doc else { return };
        let extent = framing::scene_extent(doc, &self.models, &self.framing.meshes);
        if let Some((eye, _)) = self.framing.update(extent, framing::content_key(doc)) {
            for msg in self.blueprint.set_eye(eye) {
                self.queue.push(msg, true);
            }
        }
    }

    /// Move the view's eye as `command` says ([`factoryos_client::protocol::view`]), also over a
    /// camera the user moved; `fit` frames the scene again. Returns the eye set (`None`: fitted).
    pub fn command_eye(&mut self, command: &factoryos_client::protocol::view::ViewCommand) -> Result<Option<Eye>, String> {
        let current = self.blueprint.eye();
        let center = self.framing.framed_bounds().map(|b| b.center()).or(current.map(|e| e.look_target)).unwrap_or(crate::rr::re_sdk_types::external::glam::Vec3::ZERO);
        let (eye, target) = current.map_or(([-1.0, -4.0, 2.0], [0.0; 3]), |e| (e.position.to_array().map(f64::from), e.look_target.to_array().map(f64::from)));
        let resolved = factoryos_client::protocol::view::resolve_eye(command, eye, target, center.to_array().map(f64::from))?;
        // Written over whatever the user did to the active blueprint's eye.
        self.eye_sync = blueprint::EyeSync::default();
        let Some((position, look_target)) = resolved else {
            self.framing.restart();
            self.frame_view();
            return Ok(None);
        };
        let to = |v: [f64; 3]| crate::rr::re_sdk_types::external::glam::Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
        let eye = Eye { position: to(position), look_target: to(look_target) };
        for msg in self.blueprint.set_eye(eye) {
            self.queue.push(msg, true);
        }
        Ok(Some(eye))
    }

    /// The eye the view is to have (fitted last), and the bounds it was fitted to.
    pub fn fitted_eye(&self) -> Option<(Eye, Option<Aabb>)> {
        Some((self.blueprint.eye()?, self.framing.framed_bounds()))
    }

    /// The 3D view's eye as Rerun has it (its active blueprint's), and that blueprint.
    pub fn view_eye(&self, hub: &StoreHub) -> Option<(StoreId, ViewEye)> {
        let store = hub.active_blueprint_id_for_app(&self.app_id)?;
        let db = hub.entity_db(store)?;
        Some((store.clone(), ViewEye::read(db, &self.blueprint.eye_entity())))
    }

    /// What to append to the active blueprint (`SystemCommand::AppendToStore`) so that the 3D
    /// view shows the fitted eye, unless the user has moved it ([`blueprint::EyeSync`]). Call
    /// every frame, before Rerun's `logic`.
    pub fn sync_eye(&mut self, hub: &StoreHub) -> Option<(StoreId, Vec<Chunk>)> {
        let wanted = self.blueprint.eye()?;
        let (store, active) = self.view_eye(hub)?;
        if &store == self.blueprint.store() || !self.eye_sync.should_write(wanted, &store, &active) {
            return None;
        }
        let timepoint = blueprint_timepoint_for_writes(hub.entity_db(&store)?);
        let chunk = self.blueprint.eye_chunk(&wanted, timepoint)?;
        Some((store, vec![chunk]))
    }

    /// Returns whether work still waits: messages, or recordings to close.
    pub fn forward(&mut self) -> bool {
        if self.tx.is_empty() {
            for close in &mut self.closes {
                if close.stage == Stage::Sent {
                    close.stage = Stage::Ingested;
                }
            }
        }
        let (garbage, store) = (&mut self.compact.garbage, self.store.as_ref());
        self.queue.forward_each(&self.tx, |msg| garbage.observe(store, msg));
        if self.queue.is_empty() {
            for close in &mut self.closes {
                if close.stage == Stage::Queued {
                    close.stage = Stage::Sent;
                }
            }
        }
        !self.queue.is_empty() || !self.closes.is_empty()
    }

    pub fn current(&self) -> Option<StoreId> {
        self.compact.shown(self.store.as_ref())
    }

    /// Routing to the recording shown (once it is new), and one replaced recording to close
    /// (once Rerun has all of it). Call every frame, after [`Self::forward`], until `None`.
    pub fn take_route_request(&mut self) -> Option<RouteRequest> {
        let close = self.closes.iter().position(|c| c.stage == Stage::Ingested).map(|i| self.closes.remove(i).store);
        if close.is_none() && !self.open_wanted {
            return None;
        }
        self.open_wanted = false;
        Some(RouteRequest { close, open: self.store.clone()? })
    }

    /// What is still on its way to the 3D view (read-only; see [`super::Pending`]).
    pub fn pending(&self) -> super::Pending {
        super::Pending {
            scene: self.doc.is_some(),
            models: self.models.values().filter(|model| model.loading()).count(),
            meshes: self.pending.len(),
            queued: !self.queue.is_empty(),
            unread: !self.tx.is_empty(),
            closes: self.closes.len(),
            rebasing: self.compact.switching(),
            route: self.open_wanted,
            rejoin: false, // The bus's: `Scene3d::pending`.
        }
    }

    /// The scene's title (its view hints), once there is a scene.
    pub fn title(&self) -> Option<String> {
        self.doc.as_ref().map(|doc| doc.view.title.clone())
    }

    /// When a failed fetch is to be retried next, if one is.
    pub fn next_retry(&self) -> Option<Instant> {
        let models = self.models.values().filter_map(urdf::Model::retry_at);
        models.chain(self.pending.iter().filter_map(|mesh| mesh.retry_at)).chain(self.compact.deadline()).min()
    }

    /// Show `joints` of model `node` at once; they hold against the producer's for a while (see
    /// the module docs).
    pub fn set_local_joints(&mut self, node: &str, joints: &BTreeMap<String, f64>, assets: &dyn AssetSource) {
        let Some(scene_id) = self.doc.as_ref().map(|doc| doc.scene_id.clone()) else { return };
        let now = Instant::now();
        let held = self.local.entry(node.to_owned()).or_default();
        for (joint, value) in joints {
            held.insert(joint.clone(), (*value, now));
        }
        let frame = JointFrame {
            scene_id,
            t: None,
            models: vec![ModelJoints { node: node.to_owned(), joints: joints.clone() }],
        };
        let mut out = Out::default();
        self.joints(&frame, &mut out);
        self.flush(out);
        self.retry(assets);
        self.frame_view();
        self.sync_blueprint();
    }

    /// A whole scene: a new recording for a new `scene_id`, else (a resync) the same one, cleared.
    fn reset(&mut self, mut doc: SceneDoc, out: &mut Out) {
        let same_scene = self.doc.as_ref().is_some_and(|shown| shown.scene_id == doc.scene_id);
        if same_scene && self.store.is_some() {
            out.clear(EntityPath::root(), true);
            // All is logged again, in new frames: the old ones stay defined.
            for node in self.doc.iter().flat_map(|shown| &shown.nodes) {
                self.frames.retire(&node.id);
                self.compact.garbage.retired(node);
            }
        } else {
            let store = StoreId::random(StoreKind::Recording, self.app_id.clone());
            self.queue.push(
                LogMsg::SetStoreInfo(SetStoreInfo {
                    row_id: *crate::rr::re_chunk::RowId::new(),
                    info: StoreInfo::new(store.clone(), StoreSource::Other("factoryos scene3d".to_owned())),
                }),
                true,
            );
            if let Some(old) = self.store.replace(store) {
                self.closes.push(Close { store: old, stage: Stage::Queued });
                // A new scene (a rebase takes the store first: it keeps the user's camera).
                self.retrack = true;
            }
            self.open_wanted = true;
            self.frames = Frames::default();
            self.local.clear();
        }
        self.models.clear();
        self.pending.clear();
        // Cleared, or a new recording: the eye is fitted again.
        self.framing.restart();
        let now = Instant::now();
        for node in &mut doc.nodes {
            hold_local(&mut self.local, node, now);
        }
        for node in &doc.nodes {
            self.add(node, out);
        }
        self.doc = Some(doc);
    }

    fn edit(&mut self, delta: &SceneDelta, out: &mut Out) {
        let Some(doc) = &mut self.doc else { return };
        let mut delta = delta.clone();
        let now = Instant::now();
        for node in &mut delta.upsert {
            hold_local(&mut self.local, node, now);
        }
        let before = doc.clone();
        if !doc.apply(&delta) {
            re_log::debug!("Scene delta {} for {} does not follow {}", delta.seq, delta.scene_id, before.seq);
            return;
        }
        // Nodes whose frames are retired: their children by `parent` are placed again.
        let mut retired = BTreeSet::new();
        for removed in &delta.remove {
            out.clear(entities::entity(removed), true);
            for node in before.nodes.iter().filter(|node| is_under(&node.id, removed)) {
                self.frames.retire(&node.id);
                self.compact.garbage.retired(node);
                retired.insert(node.id.clone());
            }
            self.models.retain(|id, _| !is_under(id, removed));
            self.pending.retain(|mesh| !is_under(&mesh.node, removed));
            self.local.retain(|id, _| !is_under(id, removed));
        }
        for node in &delta.upsert {
            let old = before.node(&node.id).filter(|old| !delta.remove.iter().any(|r| is_under(&old.id, r)));
            if let Some(old) = old {
                if let Some(model) = self.models.get_mut(&node.id).filter(|model| model.is_same(node)) {
                    // The same model, moved or reposed: its URDF and meshes stay.
                    out.transform(entities::entity(&node.id), &self.frames.placement(node));
                    model.update(node, out, &mut self.pending);
                    continue;
                }
                self.drop_node(&node.id, out);
                if matches!(old.kind, NodeKind::Model { .. }) {
                    // Its URDF is loaded again (perhaps another one): new link frames.
                    self.frames.retire(&node.id);
                    self.compact.garbage.retired(old);
                    retired.insert(node.id.clone());
                }
            }
            self.add(node, out);
        }
        if retired.is_empty() {
            return;
        }
        let upserted: BTreeSet<&str> = delta.upsert.iter().map(|node| node.id.as_str()).collect();
        for node in self.doc.iter().flat_map(|doc| &doc.nodes) {
            if node.parent.as_ref().is_some_and(|parent| retired.contains(&parent.node)) && !upserted.contains(node.id.as_str()) {
                out.transform(entities::entity(&node.id), &self.frames.placement(node));
            }
        }
    }

    /// The producer's joint values, less those a local one still holds (see the module docs).
    fn without_held(&mut self, mut frame: JointFrame, now: Instant) -> JointFrame {
        for model in &mut frame.models {
            let Some(held) = self.local.get_mut(&model.node) else { continue };
            model.joints.retain(|joint, value| {
                let Some(&(local, at)) = held.get(joint) else { return true };
                if (*value - local).abs() < 1e-9 || now.saturating_duration_since(at) >= LOCAL_HOLD {
                    // Echoed (or given up on): the producer's again from now on.
                    held.remove(joint);
                    true
                } else {
                    false
                }
            });
        }
        self.local.retain(|_, held| !held.is_empty());
        frame
    }

    fn joints(&mut self, frame: &JointFrame, out: &mut Out) {
        let Some(doc) = &mut self.doc else { return };
        if !doc.apply_joints(frame) {
            return;
        }
        for model in &frame.models {
            if let Some(m) = self.models.get_mut(&model.node) {
                m.set_joints(&model.joints, out);
            }
        }
    }

    /// Nodes moved: their placements again, nothing else (the same fast path as an upsert of the
    /// same model). Folded into the scene, for a later edit or resync.
    fn poses(&mut self, frame: &PoseFrame, out: &mut Out) {
        let Some(doc) = &mut self.doc else { return };
        if frame.scene_id != doc.scene_id {
            return;
        }
        let moved: BTreeSet<&str> = frame
            .poses
            .iter()
            .filter(|p| doc.node(&p.node).is_some_and(|node| node.pose != p.pose))
            .map(|p| p.node.as_str())
            .collect();
        if !doc.apply_poses(frame) {
            return;
        }
        for id in moved {
            if let Some(node) = doc.node(id) {
                out.transform(entities::entity(id), &self.frames.placement(node));
            }
        }
    }

    /// A new scene: ask for what its first frame needs (every URDF, every decimated mesh) before
    /// the full meshes, which [`Self::retry`] asks for next.
    fn prefetch(&self, assets: &dyn AssetSource) {
        let Some(doc) = &self.doc else { return };
        for node in &doc.nodes {
            if let NodeKind::Model { model, .. } = &node.kind {
                assets.get_with(&model.urdf, Priority::High);
            }
        }
        for mesh in &self.pending {
            mesh.request_lod(assets);
        }
    }

    /// Clear what node `id` logged itself (not its child nodes).
    fn drop_node(&mut self, id: &str, out: &mut Out) {
        out.clear(entities::entity(id), false);
        if let Some(model) = self.models.remove(id) {
            if let Some(robot) = model.robot_name() {
                out.clear(entities::entity(id) / EntityPath::from_single_string(robot), true);
            }
            out.clear(entities::entity(id) / EntityPath::from_single_string("joints"), true);
        }
        self.pending.retain(|mesh| mesh.node != id);
    }

    /// Log `node`: its placement and, if visible, what it draws (models and meshes once their
    /// assets are there).
    fn add(&mut self, node: &Node, out: &mut Out) {
        let entity = entities::entity(&node.id);
        let frame = self.frames.node(&node.id);
        out.transform(entity.clone(), &self.frames.placement(node));
        match &node.kind {
            NodeKind::Model { .. } => {
                if let Some(model) = urdf::Model::new(node, frame) {
                    self.models.insert(node.id.clone(), model);
                }
            }
            NodeKind::Mesh { mesh, scale, color } => {
                if node.visible {
                    let poses = InstancePoses3D::update_fields()
                        .with_scales([Vec3D::new(scale[0] as f32, scale[1] as f32, scale[2] as f32)]);
                    let albedo = color.map(|c| Rgba32::from_unmultiplied_rgba(c[0], c[1], c[2], c[3]));
                    self.pending.push(PendingMesh::new(&node.id, entity, mesh.clone(), node.id.clone(), poses, frame, albedo));
                }
            }
            kind => {
                if node.visible
                    && let Some(primitive) = entities::primitive(kind, self.labels)
                {
                    out.geometry(entity, &[&CoordinateFrame::new(frame), primitive.as_ref()]);
                }
            }
        }
    }

    /// Load models and draw meshes whose assets have arrived.
    fn retry(&mut self, assets: &dyn AssetSource) {
        let mut out = Out::default();
        let now = Instant::now();
        for model in self.models.values_mut() {
            if model.waiting(now) {
                model.try_load(assets, &mut out, &mut self.pending, now, self.retry_after);
            }
        }
        // Decimated meshes before full ones, also those of URDFs just loaded.
        for mesh in &self.pending {
            mesh.request_lod(assets);
        }
        let (convert, bounds) = (&self.convert, &mut self.framing.meshes);
        self.pending.retain_mut(|mesh| !mesh.try_emit(assets, convert, &mut out, bounds, now, self.retry_after));
        self.flush(out);
    }

    /// Keep the blueprint in step with the scene (its first write activates it, with the eye
    /// fitted by then).
    fn sync_blueprint(&mut self) {
        if let Some(doc) = &self.doc {
            for msg in self.blueprint.sync(&doc.view.title) {
                self.queue.push(msg, true);
            }
        }
    }

    /// Queue `out`'s chunks for the scene's recording.
    fn flush(&mut self, out: Out) {
        let Some(store) = &self.store else { return };
        // Something moved or was drawn: the scene's bounds may have grown.
        self.framing.dirty |= !out.chunks.is_empty();
        for (chunk, protected) in out.chunks {
            debug_assert!(chunk.is_static(), "a live view logs static data only: {}", chunk.entity_path());
            match chunk.to_arrow_msg() {
                Ok(msg) => self.queue.push(LogMsg::ArrowMsg(store.clone(), msg), protected),
                Err(err) => re_log::warn!("Failed to encode a chunk: {err}"),
            }
        }
    }
}

/// Local values still held for model `node` replace the producer's (an upsert, a resync).
fn hold_local(local: &mut BTreeMap<String, BTreeMap<String, (f64, Instant)>>, node: &mut Node, now: Instant) {
    let NodeKind::Model { joints, .. } = &mut node.kind else { return };
    let Some(held) = local.get_mut(&node.id) else { return };
    held.retain(|_, (_, at)| now.saturating_duration_since(*at) < LOCAL_HOLD);
    for (joint, (value, _)) in held.iter() {
        joints.insert(joint.clone(), *value);
    }
}

#[cfg(test)]
mod tests;
