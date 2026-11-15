//! The 3D scenes ([`crate::scene3d`]) must show the state *now*, never a replay of what happened
//! while nobody looked. Rerun is only the live view here: the recording is the behaviour tree on
//! disk. So every chunk is static (no timeline, no `log_time`/`log_tick`), and nothing temporal is
//! ever handed to Rerun (a debug assertion; dropped in release builds).
//!
//! Scene messages arrive also while the page is hidden (browsers keep delivering WebSocket messages
//! but run no `requestAnimationFrame`, so no frame) or while the viewer is busy. Rerun ingests only
//! ~10 ms worth of messages per frame (`receive_messages` in re_viewer's `app/logic.rs`), so
//! forwarding every message straight into its channel queued every intermediate pose there, and
//! once the page came back Rerun worked through that backlog: the robots replayed their movements.
//! Rerun also keeps every static transform it gets (`preserves_static_transforms` in
//! `re_chunk_store/src/writes.rs`), so each one handed over costs memory until the recording is
//! rebuilt (`scene3d/rerun/compact.rs`).
//!
//! [`LiveQueue`] sits in front of Rerun's channel. It hands messages over only once Rerun has
//! drained the previous batch. While it holds them, a newer transform of the same entity and frame
//! replaces the older one (see [`coalesce_key`]). So after any backlog Rerun gets one transform per
//! frame: the latest. Never dropped: store info and blueprints, static data other than transforms
//! (meshes, geometry, clears), and messages pushed as `protected`.

use std::collections::{HashMap, VecDeque};

use crate::rr::re_chunk::{Chunk, ComponentIdentifier, EntityPath};
use crate::rr::re_log;
use crate::rr::re_log_channel::LogSender;
use crate::rr::re_log_types::{LogMsg, StoreId, StoreKind};
use crate::rr::re_sdk_types::archetypes::{InstancePoses3D, Transform3D};
use crate::rr::re_sdk_types::Archetype as _;

/// What makes two messages interchangeable for the latest state.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
struct Key {
    store: StoreId,
    entity: EntityPath,
    components: Vec<ComponentIdentifier>,
    /// The transform's child frame: an entity may define several frames (Rerun keeps them all).
    child_frame: Option<String>,
}

/// The chunk of a recording's data message (not store info, not blueprints).
fn recording_chunk(msg: &LogMsg) -> Option<(&StoreId, Chunk)> {
    let LogMsg::ArrowMsg(store, arrow_msg) = msg else {
        return None; // store info, blueprint activation
    };
    if store.kind() != StoreKind::Recording {
        return None; // blueprints (on their own `blueprint` timeline) are small and must all apply
    }
    Some((store, Chunk::from_arrow_msg(arrow_msg).ok()?))
}

/// Whether `msg` is a recording's data on a timeline, which a live view never logs.
#[cfg(test)]
fn is_temporal(msg: &LogMsg) -> bool {
    recording_chunk(msg).is_some_and(|(_, chunk)| !chunk.is_static())
}

/// The coalescing identity of `chunk` (of `store`), if a newer one may replace it: a static
/// transform.
fn coalesce_key(store: &StoreId, chunk: &Chunk) -> Option<Key> {
    let transform_archetypes = [Transform3D::name(), InstancePoses3D::name()];
    let transforms_only = chunk
        .component_descriptors()
        .all(|descr| descr.archetype.is_some_and(|archetype| transform_archetypes.contains(&archetype)));
    if !chunk.is_static() || !transforms_only {
        return None; // meshes, URDF geometry, clears
    }
    let mut components: Vec<_> = chunk.component_descriptors().map(|descr| descr.component).collect();
    if components.is_empty() {
        return None;
    }
    components.sort();
    let child_frame = chunk
        .iter_slices::<String>(Transform3D::descriptor_child_frame().component)
        .flatten()
        .next()
        .map(|frame| frame.to_string());
    Some(Key { store: store.clone(), entity: chunk.entity_path().clone(), components, child_frame })
}

/// Scene messages not yet handed to Rerun.
#[derive(Default)]
pub struct LiveQueue {
    /// In arrival order; `None` where a newer message replaced one.
    pending: VecDeque<Option<LogMsg>>,
    /// Sequence number of `pending.front()`.
    base: u64,
    /// The pending message that the next one with the same key replaces.
    newest: HashMap<Key, u64>,
    /// Messages dropped because a newer one replaced them (for logs and tests).
    pub dropped: u64,
}

impl LiveQueue {
    /// Queue `msg`. A `protected` message is never dropped (it may still replace older ones).
    /// Temporal data is refused (see the module docs).
    pub fn push(&mut self, msg: LogMsg, protected: bool) {
        let chunk = recording_chunk(&msg);
        if chunk.as_ref().is_some_and(|(_, chunk)| !chunk.is_static()) {
            debug_assert!(false, "a live view logs static data only");
            re_log::warn_once!("Dropped temporal data for a live 3D view (a bug: scenes are static)");
            return;
        }
        let seq = self.base + self.pending.len() as u64;
        if let Some(key) = chunk.and_then(|(store, chunk)| coalesce_key(store, &chunk)) {
            if let Some(old_seq) = self.newest.remove(&key)
                && let Some(slot) = self.pending.get_mut((old_seq - self.base) as usize)
                && slot.take().is_some()
            {
                self.dropped += 1;
            }
            if !protected {
                self.newest.insert(key, seq);
            }
        }
        self.pending.push_back(Some(msg));
    }

    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Messages waiting for Rerun (dropped ones not counted).
    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.pending.iter().filter(|msg| msg.is_some()).count()
    }

    /// Take everything pending, in order.
    pub fn take_all(&mut self) -> Vec<LogMsg> {
        self.base += self.pending.len() as u64;
        self.newest.clear();
        self.pending.drain(..).flatten().collect()
    }

    /// Hand the pending messages to Rerun, unless it has not ingested the last batch yet (then they
    /// wait here, where newer ones can still replace them). Call once per frame, before Rerun's
    /// `logic`, and repaint again while [`Self::is_empty`] is `false`. Returns how many were sent.
    #[cfg(test)]
    pub fn forward(&mut self, tx: &LogSender) -> usize {
        self.forward_each(tx, |_| {})
    }

    /// [`Self::forward`], showing `each` every message handed over.
    pub fn forward_each(&mut self, tx: &LogSender, mut each: impl FnMut(&LogMsg)) -> usize {
        if self.pending.is_empty() || !tx.is_empty() {
            return 0;
        }
        let msgs = self.take_all();
        let n = msgs.len();
        for msg in msgs {
            each(&msg);
            tx.send(msg.into()).ok();
        }
        n
    }
}

/// Whether the page became visible again since the last call (web: `visibilitychange`); the live
/// recordings then go back to following the latest time. Natively always `false`.
pub fn take_page_shown() -> bool {
    #[cfg(target_arch = "wasm32")]
    {
        web_visibility::SHOWN.swap(false, std::sync::atomic::Ordering::Relaxed)
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        false
    }
}

/// Watch the page's visibility (web only): see [`take_page_shown`].
pub fn watch_page_visibility(_ctx: &crate::rr::egui::Context) {
    #[cfg(target_arch = "wasm32")]
    web_visibility::watch(_ctx);
}

#[cfg(target_arch = "wasm32")]
mod web_visibility {
    use std::sync::atomic::{AtomicBool, Ordering};

    use wasm_bindgen::JsCast as _;
    use wasm_bindgen::closure::Closure;

    use crate::rr::egui;

    pub static SHOWN: AtomicBool = AtomicBool::new(false);

    pub fn watch(ctx: &egui::Context) {
        let Some(document) = web_sys::window().and_then(|window| window.document()) else {
            return;
        };
        let ctx = ctx.clone();
        let doc = document.clone();
        let on_change = Closure::<dyn Fn()>::new(move || {
            if doc.visibility_state() == web_sys::VisibilityState::Visible {
                SHOWN.store(true, Ordering::Relaxed);
                ctx.request_repaint();
            }
        });
        if document
            .add_event_listener_with_callback("visibilitychange", on_change.as_ref().unchecked_ref())
            .is_ok()
        {
            on_change.forget(); // lives as long as the page
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rr::re_chunk::{RowId, TimePoint, Timeline};
    use crate::rr::re_log_types::SetStoreInfo;
    use crate::rr::re_sdk_types::AsComponents;
    use crate::rr::re_sdk_types::archetypes::{Asset3D, Clear};

    fn store() -> StoreId {
        StoreId::random(StoreKind::Recording, "robotic_cell")
    }

    fn msg_at(store: &StoreId, entity: &str, timepoint: TimePoint, archetype: &dyn AsComponents) -> LogMsg {
        let chunk = Chunk::builder(entity).with_archetype(RowId::new(), timepoint, archetype).build().unwrap();
        LogMsg::ArrowMsg(store.clone(), chunk.to_arrow_msg().unwrap())
    }

    fn msg(store: &StoreId, entity: &str, archetype: &dyn AsComponents) -> LogMsg {
        msg_at(store, entity, TimePoint::STATIC, archetype)
    }

    fn pose(store: &StoreId, entity: &str, x: f32) -> LogMsg {
        msg(store, entity, &Transform3D::from_translation([x, 0.0, 0.0]))
    }

    fn translation_x(msg: &LogMsg) -> f32 {
        let LogMsg::ArrowMsg(_, arrow_msg) = msg else { panic!("not a chunk") };
        let chunk = Chunk::from_arrow_msg(arrow_msg).unwrap();
        let component = Transform3D::descriptor_translation().component;
        let translations: Vec<_> = chunk
            .iter_slices::<[f32; 3]>(component)
            .flat_map(|slice| slice.iter().map(|t| t[0]).collect::<Vec<_>>())
            .collect();
        translations[0]
    }

    fn entity(msg: &LogMsg) -> String {
        let LogMsg::ArrowMsg(_, arrow_msg) = msg else { return "<store info>".to_owned() };
        Chunk::from_arrow_msg(arrow_msg).unwrap().entity_path().to_string()
    }

    #[test]
    fn keeps_only_the_newest_pose_per_entity() {
        let store = store();
        let mut queue = LiveQueue::default();
        for i in 0..100 {
            queue.push(pose(&store, "robots/r1/joints/j1", i as f32), false);
            queue.push(pose(&store, "robots/r1/joints/j2", -(i as f32)), false);
        }
        assert_eq!(queue.len(), 2);
        assert_eq!(queue.dropped, 198);
        let msgs = queue.take_all();
        assert_eq!(entity(&msgs[0]), "/robots/r1/joints/j1");
        assert_eq!(translation_x(&msgs[0]), 99.0);
        assert_eq!(translation_x(&msgs[1]), -99.0);
    }

    #[test]
    fn a_joint_that_stopped_moving_keeps_its_last_pose() {
        // Only joints that moved are logged: an entity missing from newer messages must stay.
        let store = store();
        let mut queue = LiveQueue::default();
        queue.push(pose(&store, "a", 1.0), false);
        queue.push(pose(&store, "b", 2.0), false);
        queue.push(pose(&store, "b", 3.0), false);
        let msgs = queue.take_all();
        assert_eq!(msgs.iter().map(entity).collect::<Vec<_>>(), ["/a", "/b"]);
        assert_eq!(translation_x(&msgs[1]), 3.0);
    }

    #[test]
    fn transforms_coalesce_per_frame_geometry_and_clears_are_kept() {
        let store = store();
        let mut queue = LiveQueue::default();
        let mesh = Asset3D::from_file_contents(b"solid x\nendsolid x\n".to_vec(), None::<crate::rr::re_sdk_types::components::MediaType>);
        queue.push(msg(&store, "robot/link_1/mesh", &mesh), false);
        queue.push(msg(&store, "robot/link_1/mesh", &mesh), false);
        queue.push(msg(&store, "robot/link_1", &Clear::recursive()), false);
        for i in 0..10 {
            queue.push(pose(&store, "robot/joints/j1", i as f32), false);
        }
        // Another frame on the same entity: both stay (Rerun keeps both).
        let other_frame = Transform3D::from_translation([7.0, 0.0, 0.0]).with_child_frame("robot@1");
        queue.push(msg(&store, "robot/joints/j1", &other_frame), false);
        let msgs = queue.take_all();
        assert_eq!(msgs.len(), 5);
        assert_eq!(translation_x(&msgs[3]), 9.0);
        assert_eq!(translation_x(&msgs[4]), 7.0);
    }

    #[test]
    fn protected_messages_are_never_dropped() {
        let store = store();
        let mut queue = LiveQueue::default();
        queue.push(pose(&store, "a", 1.0), true);
        queue.push(pose(&store, "a", 2.0), false);
        queue.push(pose(&store, "a", 3.0), false);
        let msgs = queue.take_all();
        assert_eq!(msgs.iter().map(translation_x).collect::<Vec<_>>(), [1.0, 3.0]);
        // ...but a protected message does replace older live ones.
        queue.push(pose(&store, "a", 4.0), false);
        queue.push(pose(&store, "a", 5.0), true);
        let msgs = queue.take_all();
        assert_eq!(msgs.iter().map(translation_x).collect::<Vec<_>>(), [5.0]);
    }

    #[test]
    fn store_info_and_other_stores_are_kept_in_order() {
        let store = store();
        let other = StoreId::random(StoreKind::Recording, "robotic_cell");
        let mut queue = LiveQueue::default();
        let info = LogMsg::SetStoreInfo(SetStoreInfo {
            row_id: *RowId::new(),
            info: crate::rr::re_log_types::StoreInfo::testing_with_recording_id(store.recording_id().clone()),
        });
        queue.push(info, false);
        queue.push(pose(&store, "a", 1.0), false);
        queue.push(pose(&other, "a", 2.0), false);
        queue.push(pose(&store, "a", 3.0), false);
        let msgs = queue.take_all();
        assert!(matches!(msgs[0], LogMsg::SetStoreInfo(_)));
        assert_eq!(msgs[1..].iter().map(translation_x).collect::<Vec<_>>(), [2.0, 3.0]);
    }

    #[test]
    fn forwards_only_once_rerun_drained_the_last_batch() {
        use crate::rr::re_log_channel::{LogSource, log_channel};

        let store = store();
        let (tx, rx) = log_channel(LogSource::Sdk);
        let mut queue = LiveQueue::default();
        queue.push(pose(&store, "a", 1.0), false);
        assert_eq!(queue.forward(&tx), 1);
        // Rerun has not run (hidden page / busy): everything waits here and coalesces.
        for i in 2..50 {
            queue.push(pose(&store, "a", i as f32), false);
            assert_eq!(queue.forward(&tx), 0);
        }
        assert!(rx.try_recv().is_ok()); // Rerun's next frame
        assert_eq!(queue.forward(&tx), 1);
        let latest = rx.try_recv().unwrap().into_data().unwrap();
        let crate::rr::re_log_channel::DataSourceMessage::LogMsg(latest) = latest else { panic!() };
        assert_eq!(translation_x(&latest), 49.0);
        assert!(rx.try_recv().is_err());
    }

    fn temporal(store: &StoreId) -> LogMsg {
        let timepoint = TimePoint::default().with(Timeline::new_duration("execution_time"), 0);
        msg_at(store, "a", timepoint, &Transform3D::from_translation([1.0, 0.0, 0.0]))
    }

    #[test]
    fn only_a_recordings_timeline_data_is_temporal() {
        let store = store();
        assert!(is_temporal(&temporal(&store)));
        assert!(!is_temporal(&pose(&store, "a", 1.0)));
        // Blueprints write on their own `blueprint` timeline: not scene data.
        let blueprint = StoreId::random(StoreKind::Blueprint, "robotic_cell");
        assert!(!is_temporal(&temporal(&blueprint)));
    }

    #[test]
    #[cfg_attr(debug_assertions, should_panic(expected = "static data only"))]
    fn temporal_data_is_never_forwarded() {
        let store = store();
        let mut queue = LiveQueue::default();
        queue.push(temporal(&store), true);
        assert!(queue.is_empty(), "release builds drop it");
    }
}
