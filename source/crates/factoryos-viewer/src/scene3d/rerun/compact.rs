//! Rerun is only the live view; the recording is the behaviour tree on disk. So a scene's
//! recording should hold its current state and nothing else. It doesn't by itself, because Rerun's
//! store keeps more than the latest static data:
//!
//! * Every static `Transform3D` stays in the store: re-logging one does not replace the old chunk,
//!   because named transforms are interpreted in full, not latest-at
//!   (`preserves_static_transforms` in `re_chunk_store/src/writes.rs`). So every joint value, pose
//!   and generation of frames logged so far is kept.
//! * A `Clear` only hides data at query time (`re_query/src/latest_at.rs`); the cleared chunks and
//!   the entities stay. `re_tf` also ignores static clears, which is why replaced nodes get new
//!   frame names ([`super::entities::Frames`]), and the old frames stay defined.
//! * The GC never drops static data (`re_chunk_store/src/gc.rs`), so a memory limit cannot
//!   reclaim any of it.
//!
//! Other static data does replace its predecessor: a chunk whose components were all re-logged
//! later is dropped. This applies to meshes re-logged on the same entity.
//!
//! [`Garbage`] estimates what the current recording holds beyond its current state: transforms
//! that were logged again, cleared entities, retired frame generations. Once that passes a
//! threshold, [`RerunScene::rebase_if_due`] rebuilds the scene from its [`SceneDoc`] into a fresh
//! recording, with generations starting at 0 again and the cached asset bytes reused, then routes
//! to it and closes the old one. The old recording is closed only once Rerun has ingested the new
//! one, so nothing flickers or reopens.

use std::collections::HashMap;
use std::time::Duration;

use web_time::Instant;

use factoryos_client::protocol::scene::{Node, NodeKind};

use crate::rr::re_chunk::{Chunk, EntityPath};
use crate::rr::re_chunk_store::preserves_static_transforms;
use crate::rr::re_log;
use crate::rr::re_log_types::{LogMsg, StoreId, StoreKind};
use crate::rr::re_sdk_types::archetypes::Clear;

use super::super::AssetSource;
use super::{Close, RerunScene, Stage};
use super::entities::Out;

/// Rebase once this many model instances have been replaced or removed (their frame generations
/// retired). Their frames stay defined in Rerun's transform tree.
pub const MAX_REPLACED_GENERATIONS: u64 = 20;

/// Rebase once this many entities have been cleared. A cleared entity stays in the entity tree,
/// and its data stays in the store.
pub const MAX_REMOVED_ENTITIES: u64 = 1_000;

/// Rebase once the store holds about this much that is not shown (64 MiB), and at least as much
/// as the scene itself. Rebuilding then costs no more than what has piled up, so memory stays
/// within about twice the scene plus this.
pub const MAX_GARBAGE_BYTES: u64 = 64 << 20;

/// What the current recording holds beyond its current state, by what Rerun's store keeps (see the
/// module docs). Only chunks handed to Rerun count (the [`crate::live_queue::LiveQueue`] drops
/// superseded ones before that).
#[derive(Default, Debug)]
pub struct Garbage {
    /// The recording counted. Chunks for other recordings (e.g. the one being replaced) don't count.
    store: Option<StoreId>,
    /// What each entity holds now, by what Rerun keeps of it.
    entities: HashMap<EntityPath, Held>,
    /// Retired frame generations (a removed, or re-loaded, model instance).
    pub replaced_generations: u64,
    /// Entities cleared (removed, or replaced and cleared first).
    pub removed_entities: u64,
    /// Chunks that logged an entity's data again.
    pub relogged_chunks: u64,
    /// Their bytes.
    pub relogged_bytes: u64,
    /// An estimate of what the store keeps that is not shown: superseded transforms, cleared data.
    pub garbage_bytes: u64,
    /// What is shown, by the same estimate.
    pub live_bytes: u64,
}

/// What an entity holds that is shown.
#[derive(Default, Debug, Clone, Copy)]
struct Held {
    /// Its transform (kept even once superseded).
    transform: u64,
    /// Everything else (replaced by a newer chunk of the same components).
    data: u64,
}

impl Garbage {
    /// Count `msg`, handed to Rerun, if it is a chunk of `current` (the scene's recording now).
    pub fn observe(&mut self, current: Option<&StoreId>, msg: &LogMsg) {
        let (Some(current), LogMsg::ArrowMsg(store, arrow_msg)) = (current, msg) else { return };
        if store != current || store.kind() != StoreKind::Recording {
            return;
        }
        if self.store.as_ref() != Some(current) {
            *self = Self { store: Some(current.clone()), ..Self::default() };
        }
        let Ok(chunk) = Chunk::from_arrow_msg(arrow_msg) else { return };
        self.observe_chunk(&chunk, arrow_msg.batch.get_array_memory_size() as u64);
    }

    fn observe_chunk(&mut self, chunk: &Chunk, bytes: u64) {
        let entity = chunk.entity_path();
        if let Some(recursive) = clear_of(chunk) {
            let cleared: Vec<EntityPath> = self
                .entities
                .keys()
                .filter(|e| *e == entity || (recursive && e.is_descendant_of(entity)))
                .cloned()
                .collect();
            for path in cleared {
                if let Some(held) = self.entities.remove(&path) {
                    let bytes = held.transform + held.data;
                    self.live_bytes = self.live_bytes.saturating_sub(bytes);
                    self.garbage_bytes += bytes;
                    self.removed_entities += 1;
                }
            }
            return;
        }
        let is_transform = chunk.component_descriptors().any(|d| preserves_static_transforms(d.archetype));
        let held = self.entities.entry(entity.clone()).or_default();
        let slot = if is_transform { &mut held.transform } else { &mut held.data };
        if *slot > 0 {
            self.relogged_chunks += 1;
            self.relogged_bytes += bytes;
            self.live_bytes = self.live_bytes.saturating_sub(*slot);
            if is_transform {
                self.garbage_bytes += *slot; // Rerun keeps the old one
            }
        }
        *slot = bytes.max(1);
        self.live_bytes += *slot;
    }

    /// Node `node`'s frames were retired (it was removed, or its URDF is loaded again). Counted
    /// for models, whose link frames stay behind in Rerun's transform tree; a plain node leaves
    /// one frame, and its cleared entity counts already.
    pub fn retired(&mut self, node: &Node) {
        if matches!(node.kind, NodeKind::Model { .. }) {
            self.replaced_generations += 1;
        }
    }

    /// Whether the recording should be rebuilt (see the thresholds).
    pub fn due(&self) -> bool {
        self.replaced_generations >= MAX_REPLACED_GENERATIONS
            || self.removed_entities >= MAX_REMOVED_ENTITIES
            || self.garbage_bytes >= MAX_GARBAGE_BYTES.max(self.live_bytes)
    }
}

/// `Some(recursive)` if `chunk` is a `Clear`.
fn clear_of(chunk: &Chunk) -> Option<bool> {
    let component = Clear::descriptor_is_recursive().component;
    chunk.component_descriptors().any(|d| d.component == component).then(|| {
        chunk.iter_slices::<bool>(component).any(|values| values.iter().any(|recursive| recursive))
    })
}

/// How long a rebased scene's new recording may take to load its geometry before it is shown
/// anyway (a mesh still being fetched or converted).
pub const SWITCH_TIMEOUT: Duration = Duration::from_secs(3);

/// The rebasing state of a scene (see the module docs).
#[derive(Default)]
pub struct Compact {
    pub garbage: Garbage,
    /// A rebase is loading its new recording: until then Rerun shows (and the app routes to) the
    /// old one.
    switching: Option<Switching>,
}

struct Switching {
    /// The recording shown until the new one is ready.
    old: StoreId,
    /// The new one.
    new: StoreId,
    since: Instant,
}

impl Compact {
    /// The recording to show: the old one while a rebase loads the new one (`current`).
    pub fn shown(&self, current: Option<&StoreId>) -> Option<StoreId> {
        match &self.switching {
            Some(switching) if Some(&switching.new) == current => Some(switching.old.clone()),
            _ => current.cloned(),
        }
    }

    /// A rebase is loading its new recording.
    pub fn switching(&self) -> bool {
        self.switching.is_some()
    }

    /// When a rebase shows its new recording at the latest.
    pub fn deadline(&self) -> Option<Instant> {
        self.switching.as_ref().map(|switching| switching.since + SWITCH_TIMEOUT)
    }
}

impl RerunScene {
    /// Rebuild the scene into a fresh recording if enough garbage has piled up in the current one
    /// (see the module docs). Returns whether it did.
    pub(super) fn rebase_if_due(&mut self, assets: &dyn AssetSource) -> bool {
        if self.compact.switching.is_some() || !self.compact.garbage.due() {
            return false;
        }
        let (Some(doc), Some(old)) = (self.doc.clone(), self.store.take()) else { return false };
        let garbage = std::mem::take(&mut self.compact.garbage);
        re_log::debug!(
            "Scene {}: a new recording, to drop what the old one keeps ({} replaced generations, {} removed entities, {} MiB not shown)",
            doc.scene_id,
            garbage.replaced_generations,
            garbage.removed_entities,
            garbage.garbage_bytes >> 20,
        );
        // With no recording, `reset` starts a new one: new store info, frames at generation 0, every
        // model and mesh logged again (from the cached bytes, in `retry`). The blueprint is the
        // application's and stays. Local joint values keep holding (the doc has them already).
        let local = std::mem::take(&mut self.local);
        let mut out = Out::default();
        self.reset(doc, &mut out);
        self.local = local;
        self.flush(out);
        self.retry(assets);
        // Not shown (routed to) before its geometry is there: see `switch_when_loaded`.
        self.open_wanted = false;
        let new = self.store.clone().expect("reset starts a recording");
        self.compact.switching = Some(Switching { old, new, since: Instant::now() });
        self.switch_when_loaded();
        true
    }

    /// Show the rebased scene's new recording once its models and meshes are all logged (or after
    /// [`SWITCH_TIMEOUT`]); the old one is closed once Rerun has ingested all of the new one.
    pub(super) fn switch_when_loaded(&mut self) {
        let Some(switching) = &self.compact.switching else { return };
        let now = Instant::now();
        let replaced = self.store.as_ref() != Some(&switching.new); // a new scene came meanwhile
        let loaded = self.pending.is_empty() && !self.models.values().any(|model| model.waiting(now));
        if !replaced && !loaded && now.saturating_duration_since(switching.since) < SWITCH_TIMEOUT {
            return;
        }
        let Some(switching) = self.compact.switching.take() else { return };
        self.open_wanted |= !replaced;
        // Queued after everything logged to the new one so far.
        self.closes.push(Close { store: switching.old, stage: Stage::Queued });
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, HashMap};
    use std::sync::Arc;

    use factoryos_client::protocol::scene::{AssetRef, ModelRef, Node, NodeKind, Pose3, SceneDelta, SceneDoc, ViewHints};

    use super::super::{RerunScene, entities};
    use super::*;
    use crate::rr::re_chunk::{RowId, TimePoint};
    use crate::rr::re_entity_db::EntityDb;
    use crate::rr::re_log_channel::DataSourceMessage;
    use crate::rr::re_log_types::StoreInfo;
    use crate::rr::re_sdk_types::archetypes::Transform3D;
    use crate::scene3d::{AssetState, Priority, SceneEvent};

    const URDF: &str = r#"<robot name="bot">
      <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
      <link name="arm"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>"#;

    /// URDF bytes by hash, as the asset cache has them once fetched.
    struct Assets(HashMap<String, Arc<[u8]>>);

    impl Assets {
        fn new() -> Self {
            Self(HashMap::from([("urdf".to_owned(), URDF.as_bytes().into())]))
        }
    }

    impl AssetSource for Assets {
        fn get_with(&self, asset: &AssetRef, _priority: Priority) -> AssetState {
            match self.0.get(&asset.sha256) {
                Some(bytes) => AssetState::Ready(bytes.clone()),
                None => AssetState::Pending,
            }
        }
    }

    fn robot(id: &str, j1: f64) -> Node {
        Node::new(
            id,
            NodeKind::Model {
                model: ModelRef {
                    urdf: AssetRef { sha256: "urdf".into(), size: 0, media_type: "application/xml".into() },
                    meshes: BTreeMap::new(),
                },
                joints: BTreeMap::from([("j1".to_owned(), j1)]),
                show_collision: false,
            },
        )
        .at(Pose3::from_xyz([1.0, 0.0, 0.0]))
    }

    fn doc(nodes: Vec<Node>) -> SceneDoc {
        SceneDoc { scene_id: "s1".into(), seq: 1, view: ViewHints { title: "Robo Fitter".into() }, nodes }
    }

    /// Rerun's side: the recordings, as the viewer's store would hold them.
    #[derive(Default)]
    struct Viewer(HashMap<StoreId, EntityDb>);

    impl Viewer {
        /// Ingest what `scene` forwarded (and let it see the channel drained).
        fn ingest(&mut self, scene: &mut RerunScene, rx: &crate::rr::re_log_channel::LogReceiver) {
            scene.forward();
            while let Ok(msg) = rx.try_recv() {
                let Some(DataSourceMessage::LogMsg(LogMsg::ArrowMsg(store, arrow_msg))) = msg.into_data() else { continue };
                if store.kind() != StoreKind::Recording {
                    continue;
                }
                let db = self.0.entry(store.clone()).or_insert_with(|| EntityDb::new(store.clone()));
                db.add_chunk(&Arc::new(Chunk::from_arrow_msg(&arrow_msg).unwrap())).unwrap();
            }
        }

        fn static_chunks(&self, store: &StoreId) -> u64 {
            self.0[store].storage_engine().store().stats().static_chunks.num_chunks
        }
    }

    /// The audit, against Rerun's own store: a static transform logged again does not replace the
    /// old chunk, and a static mesh-like component does.
    #[test]
    fn rerun_keeps_every_static_transform() {
        let mut db = EntityDb::new(StoreInfo::testing().store_id);
        for i in 0..10 {
            let transform = Transform3D::from_translation([i as f32, 0.0, 0.0]).with_child_frame("f");
            let chunk = Chunk::builder("a").with_archetype(RowId::new(), TimePoint::STATIC, &transform).build().unwrap();
            db.add_chunk(&Arc::new(chunk)).unwrap();
        }
        assert_eq!(db.storage_engine().store().stats().static_chunks.num_chunks, 10, "all kept");

        let mut db = EntityDb::new(StoreInfo::testing().store_id);
        for i in 0..10 {
            let points = crate::rr::re_sdk_types::archetypes::Points3D::new([[i as f32, 0.0, 0.0]]);
            let chunk = Chunk::builder("a").with_archetype(RowId::new(), TimePoint::STATIC, &points).build().unwrap();
            db.add_chunk(&Arc::new(chunk)).unwrap();
        }
        assert_eq!(db.storage_engine().store().stats().static_chunks.num_chunks, 1, "replaced");
    }

    #[test]
    fn garbage_counts_superseded_transforms_and_cleared_data() {
        let store = StoreInfo::testing().store_id;
        let mut garbage = Garbage::default();
        let msg_to = |store: &StoreId, entity: &str, archetype: &dyn crate::rr::re_sdk_types::AsComponents| {
            let chunk = Chunk::builder(entity).with_archetype(RowId::new(), TimePoint::STATIC, archetype).build().unwrap();
            LogMsg::ArrowMsg(store.clone(), chunk.to_arrow_msg().unwrap())
        };
        let msg = |entity: &str, archetype: &dyn crate::rr::re_sdk_types::AsComponents| msg_to(&store, entity, archetype);
        let points = crate::rr::re_sdk_types::archetypes::Points3D::new([[0.0, 0.0, 0.0]]);
        garbage.observe(Some(&store), &msg("/a", &Transform3D::from_translation([1.0, 0.0, 0.0])));
        garbage.observe(Some(&store), &msg("/a/b", &points));
        assert_eq!(garbage.garbage_bytes, 0);
        let live = garbage.live_bytes;
        garbage.observe(Some(&store), &msg("/a/b", &points));
        assert_eq!((garbage.relogged_chunks, garbage.garbage_bytes, garbage.live_bytes), (1, 0, live), "replaced");
        garbage.observe(Some(&store), &msg("/a", &Transform3D::from_translation([2.0, 0.0, 0.0])));
        assert!(garbage.garbage_bytes > 0, "the old transform stays");
        let before = garbage.garbage_bytes;
        garbage.observe(Some(&store), &msg("/a", &Clear::recursive()));
        assert_eq!(garbage.removed_entities, 2);
        assert_eq!(garbage.live_bytes, 0);
        assert_eq!(garbage.garbage_bytes, before + live);
        // Another recording's chunks don't count.
        let other = StoreId::random(StoreKind::Recording, "x");
        garbage.observe(Some(&other), &msg("/c", &points));
        assert_eq!(garbage.store.as_ref(), Some(&store), "not the one counted");
        garbage.observe(Some(&other), &msg_to(&other, "/c", &points));
        assert_eq!(garbage.store.as_ref(), Some(&other), "counting starts over for a new recording");
        assert_eq!(garbage.garbage_bytes, 0);
    }

    /// Many replacements: the scene moves to a new recording that holds just the current nodes,
    /// with generation-0 frames and no refetch, and the old one is closed once Rerun has it all.
    #[test]
    fn replacements_rebase_into_a_fresh_recording() {
        let mut scene = RerunScene::new("factoryos_test");
        let rx = scene.take_receiver().unwrap();
        let assets = Assets::new();
        let mut viewer = Viewer::default();
        scene.apply(vec![SceneEvent::Reset(doc(vec![robot("r1", 0.0)]))], &assets);
        let first = scene.current().unwrap();
        assert!(scene.take_route_request().is_some());
        viewer.ingest(&mut scene, &rx);

        // Remove and add a model again and again: a new generation each time.
        let mut seq = 1;
        let mut rebased = None;
        for i in 0..MAX_REPLACED_GENERATIONS + 2 {
            seq += 1;
            let delta = SceneDelta { scene_id: "s1".into(), seq, upsert: vec![], remove: vec!["r1".into()], view: None };
            seq += 1;
            let back = SceneDelta { scene_id: "s1".into(), seq, upsert: vec![robot("r1", i as f64 * 0.01)], remove: vec![], view: None };
            scene.apply(vec![SceneEvent::Delta(delta), SceneEvent::Delta(back)], &assets);
            viewer.ingest(&mut scene, &rx);
            if scene.current() != Some(first.clone()) {
                rebased = Some(i);
                break;
            }
        }
        assert!(rebased.is_some(), "rebased after {MAX_REPLACED_GENERATIONS} generations");
        let second = scene.current().unwrap();
        assert_ne!(second, first);
        let route = scene.take_route_request().unwrap();
        assert_eq!((route.close.as_ref(), &route.open), (None, &second), "not closed before Rerun has the new one");

        // The new recording holds the current nodes only, in generation-0 frames.
        let doc_now = scene.doc.clone().unwrap();
        assert_eq!(doc_now.nodes.iter().map(|n| n.id.as_str()).collect::<Vec<_>>(), ["r1"]);
        assert_eq!(scene.frames.node("r1"), "r1", "generations start at 0");
        viewer.ingest(&mut scene, &rx);
        viewer.ingest(&mut scene, &rx);
        let route = scene.take_route_request().unwrap();
        assert_eq!((route.close.as_ref(), &route.open), (Some(&first), &second));
        assert!(scene.take_route_request().is_none());
        let db = &viewer.0[&second];
        let entities: Vec<String> = db.sorted_entity_paths().map(|e| e.to_string()).collect();
        assert!(entities.iter().all(|e| e == "/" || e.starts_with("/r1")), "{entities:?}");
        assert!(viewer.static_chunks(&second) < viewer.static_chunks(&first), "smaller than the old one");
        let frame = Transform3D::descriptor_child_frame().component;
        let results = db.latest_at(&crate::rr::re_chunk_store::LatestAtQuery::latest(crate::rr::re_log_types::TimelineName::log_tick()), &entities::entity("r1"), [frame]);
        assert_eq!(
            results.component_mono::<crate::rr::re_sdk_types::components::TransformFrameId>(frame).map(|f| f.as_str().to_owned()),
            Some("r1".to_owned())
        );
        assert!(!scene.forward(), "nothing left to do");
    }

    /// A long run of joint values (all kept by Rerun) rebases too, once they outweigh the scene.
    #[test]
    fn joint_values_rebase_once_they_outweigh_the_scene() {
        let mut scene = RerunScene::new("factoryos_test");
        let rx = scene.take_receiver().unwrap();
        let assets = Assets::new();
        let mut viewer = Viewer::default();
        scene.apply(vec![SceneEvent::Reset(doc(vec![robot("r1", 0.0)]))], &assets);
        let first = scene.current().unwrap();
        viewer.ingest(&mut scene, &rx);
        let mut n = 0;
        while scene.current() == Some(first.clone()) {
            n += 1;
            assert!(n < 1_000_000, "never rebased");
            let frame = factoryos_client::protocol::scene::JointFrame {
                scene_id: "s1".into(),
                t: None,
                models: vec![factoryos_client::protocol::scene::ModelJoints {
                    node: "r1".into(),
                    joints: BTreeMap::from([("j1".to_owned(), (n % 1000) as f64 * 1e-3)]),
                }],
            };
            scene.apply(vec![SceneEvent::Joints(frame)], &assets);
            scene.forward();
            while rx.try_recv().is_ok() {}
        }
        let Some(NodeKind::Model { joints, .. }) = scene.doc.as_ref().unwrap().node("r1").map(|n| &n.kind) else { panic!() };
        assert_eq!(joints["j1"], (n % 1000) as f64 * 1e-3, "rebuilt at the current joint values");
    }
}
