//! 3D scenes from renderer-neutral messages ([`factoryos_client::protocol::scene`]): the backend's
//! services and fos-live-scene describe *what* is in a scene; everything Rerun-specific (entities,
//! transforms, blueprints, chunks) is in [`rerun`] and nowhere else.
//!
//! * [`client`]: follows `factoryos/scene/<source>` (snapshot, deltas, joint and pose frames).
//!   Neutral.
//! * [`assets`]: fetches content-addressed URDFs/meshes over HTTP and caches them. Neutral.
//! * [`rerun`]: turns the scene into a Rerun recording, on one log channel per source.
//!
//! [`Scene3d`] puts the three together for a panel; the app routes to its recordings
//! ([`RouteRequest`]).

// `assets::Inner::disk` is the native disk cache: unread on the web.
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub mod assets;
pub mod client;
pub mod rerun;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use factoryos_client::protocol::scene::{AssetRef, JointFrame, PoseFrame, SceneDelta, SceneDoc};
use factoryos_client::{Api, Bus};

use crate::rr::egui;
use crate::rr::re_log_channel::LogReceiver;
use crate::rr::re_log_types::StoreId;

/// Viewer navigation the app should perform on a scene's behalf: show `open`, close `close`.
pub struct RouteRequest {
    pub close: Option<StoreId>,
    pub open: StoreId,
}

/// What [`client::SceneClient`] hands on, in order.
#[derive(Clone, Debug)]
pub enum SceneEvent {
    /// A whole scene: a new one (new `scene_id`), the late-join snapshot, or a resync after a gap.
    /// Replaces everything shown before.
    Reset(SceneDoc),
    /// An edit that follows the last `Reset`/`Delta` (already checked by the client).
    Delta(SceneDelta),
    Joints(JointFrame),
    /// New poses of nodes (they move; nothing else changes).
    Poses(PoseFrame),
}

/// An asset's bytes, as far as they have arrived.
#[derive(Clone, Debug)]
pub enum AssetState {
    /// Being fetched; the source wakes the UI when it is done.
    Pending,
    Ready(Arc<[u8]>),
    /// `retryable`: the fetch may succeed later (asked again after [`assets::RETRY_AFTER`], e.g. a
    /// 404 while a restarted backend's asset index fills up again); else never (not a hash, or
    /// the bytes do not match it).
    Failed { err: String, retryable: bool },
}

/// Which fetches go first when more are asked for than run at once ([`assets::MAX_FETCHES`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// Full-detail meshes.
    #[default]
    Normal,
    /// What a first frame needs: URDFs and decimated meshes (`MeshAsset::lod`).
    High,
}

/// Where [`rerun`] gets geometry from. Never blocks: the first call starts the fetch.
pub trait AssetSource {
    /// `priority`: of the fetch this starts, or of one still waiting to start (raised, never lowered).
    fn get_with(&self, asset: &AssetRef, priority: Priority) -> AssetState;

    fn get(&self, asset: &AssetRef) -> AssetState {
        self.get_with(asset, Priority::Normal)
    }
}

/// Wakes the UI (an egui repaint) once set; callable from any thread.
#[derive(Clone, Default)]
pub struct Waker(Arc<Mutex<Option<egui::Context>>>);

impl Waker {
    pub fn set(&self, ctx: &egui::Context) {
        *self.0.lock().unwrap() = Some(ctx.clone());
    }

    /// The UI's context, once set.
    fn ctx(&self) -> Option<egui::Context> {
        self.0.lock().unwrap().clone()
    }

    pub fn wake(&self) {
        if let Some(ctx) = &*self.0.lock().unwrap() {
            ctx.request_repaint();
        }
    }

    /// Wake the UI once `after` has passed (e.g. to retry a failed fetch).
    pub fn wake_after(&self, after: std::time::Duration) {
        if let Some(ctx) = &*self.0.lock().unwrap() {
            ctx.request_repaint_after(after);
        }
    }
}

/// What a [`Scene3d`] still waits for before its 3D view shows all of the scene (for tests and
/// the native automation's `wait_idle`, see `crate::automation`).
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize)]
pub struct Pending {
    /// A scene has arrived (a snapshot or a new scene).
    pub scene: bool,
    /// Models whose URDF is not loaded yet (fetching, or waiting for a retry).
    pub models: usize,
    /// Meshes not drawn in full yet (fetching, converting, or waiting for a retry).
    pub meshes: usize,
    /// Messages in the [`crate::live_queue::LiveQueue`], not yet handed to Rerun.
    pub queued: bool,
    /// Messages handed to the log channel that Rerun has not read yet.
    pub unread: bool,
    /// Replaced recordings not closed yet.
    pub closes: usize,
    /// A rebuild into a fresh recording is loading (`rerun/compact.rs`).
    pub rebasing: bool,
    /// The app has not been asked to route to a new recording yet.
    pub route: bool,
    /// The backend went away and the bus has not re-joined it yet: the scene shown may be out of
    /// date (e.g. a restarted backend's new scene) until the re-join's resync, which then counts
    /// as `queued` while its snapshot query is in flight.
    pub rejoin: bool,
}

impl Pending {
    /// Why the scene is not fully shown yet, or `None` if it is.
    pub fn busy_reason(&self) -> Option<String> {
        let mut reasons = Vec::new();
        if !self.scene {
            reasons.push("no scene yet".to_owned());
        }
        if self.models > 0 {
            reasons.push(format!("{} model(s) loading", self.models));
        }
        if self.meshes > 0 {
            reasons.push(format!("{} mesh(es) pending", self.meshes));
        }
        if self.queued {
            reasons.push("live queue not empty".to_owned());
        }
        if self.unread {
            reasons.push("Rerun has not ingested the last batch".to_owned());
        }
        if self.closes > 0 {
            reasons.push(format!("{} replaced recording(s) to close", self.closes));
        }
        if self.rebasing {
            reasons.push("rebuilding into a fresh recording".to_owned());
        }
        if self.route {
            reasons.push("route to the new recording pending".to_owned());
        }
        if self.rejoin {
            reasons.push("the backend is away, re-join pending".to_owned());
        }
        (!reasons.is_empty()).then(|| reasons.join(", "))
    }
}

/// Whether the 3D views draw text labels (weld names, robot names, …): one setting for every
/// tool's view, the "Labels" toggle over the view ([`crate::labels_overlay`]). Applied by each
/// [`Scene3d::forward`] at render time, without the producer.
static LABELS_SHOWN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Whether the 3D views draw text labels ([`LABELS_SHOWN`]).
pub fn labels_shown() -> bool {
    LABELS_SHOWN.load(std::sync::atomic::Ordering::Relaxed)
}

/// Show or hide the 3D views' text labels from the next frame on ([`LABELS_SHOWN`]).
pub fn set_labels_shown(on: bool) {
    LABELS_SHOWN.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// One source's 3D scene in the viewer.
pub struct Scene3d {
    client: client::SceneClient,
    assets: Arc<assets::AssetCache>,
    scene: Mutex<rerun::RerunScene>,
    waker: Waker,
}

impl Scene3d {
    /// Follow `source` (`keys::SCENE_*`), shown as recordings of Rerun application `app_id`.
    /// `assets` is shared by every scene.
    pub fn connect(bus: &Bus, assets: Arc<assets::AssetCache>, source: &str, app_id: &str) -> Self {
        let waker = Waker::default();
        let mut scene = rerun::RerunScene::new(app_id);
        // Mesh conversions finishing off the UI thread wake it.
        scene.set_waker(waker.clone());
        Self {
            client: client::SceneClient::connect(bus, source, waker.clone()),
            assets,
            scene: Mutex::new(scene),
            waker,
        }
    }

    /// [`Scene3d::connect`] with `backend`'s bus and assets, or a scene that follows nothing when
    /// `backend` speaks another protocol version (the app shows why).
    pub fn for_backend(backend: &crate::backend::Backend, source: &str, app_id: &str) -> Self {
        if backend.protocol.scenes_enabled() {
            return Self::connect(&backend.bus, backend.assets.clone(), source, app_id);
        }
        let waker = Waker::default();
        Self {
            client: client::SceneClient::idle(&backend.bus, source, waker.clone()),
            assets: backend.assets.clone(),
            scene: Mutex::new(rerun::RerunScene::new(app_id)),
            waker,
        }
    }

    /// A shared asset cache for [`Scene3d::connect`].
    pub fn asset_cache(api: &Api) -> Arc<assets::AssetCache> {
        Arc::new(assets::AssetCache::new(api.clone()))
    }

    pub fn set_egui_ctx(&self, ctx: &egui::Context) {
        self.waker.set(ctx);
        self.assets.set_waker(self.waker.clone());
    }

    /// The log stream, for the Rerun app to read. `Some` only once.
    pub fn take_receiver(&mut self) -> Option<LogReceiver> {
        self.scene.get_mut().unwrap().take_receiver()
    }

    /// Apply what arrived and hand it to Rerun once it has ingested the last batch. Call every
    /// frame, before Rerun's `logic`. Returns whether work still waits (then repaint again).
    pub fn forward(&self) -> bool {
        let scene = &mut *self.scene.lock().unwrap();
        if let Some(view) = self.waker.ctx().as_ref().and_then(rerun::view_size) {
            scene.set_view_size(view.width, view.height);
        }
        scene.set_labels(labels_shown());
        scene.apply(self.client.drain(), &*self.assets);
        if let Some(at) = scene.next_retry() {
            self.waker.wake_after(at.saturating_duration_since(web_time::Instant::now()));
        }
        scene.forward()
    }

    /// Whether the 3D view shows this scene now (its recording is the active one): a new scene
    /// is framed again once shown, see `rerun::RerunScene::set_shown`. Call every frame.
    pub fn set_shown(&self, shown: bool) {
        self.scene.lock().unwrap().set_shown(shown);
    }

    /// Call with the recording the viewer shows: [`Self::set_shown`] for this scene's.
    pub fn set_active_recording(&self, active: Option<&StoreId>) {
        let shown = active.is_some() && self.current().as_ref() == active;
        self.set_shown(shown);
    }

    /// What to append to Rerun's active blueprint (`SystemCommand::AppendToStore`) so that the
    /// 3D view shows the scene's fitted eye, unless the user has moved it (see
    /// `rerun::RerunScene::sync_eye`). Call every frame, before Rerun's `logic`.
    pub fn sync_eye(&self, hub: &crate::rr::re_viewer_context::StoreHub) -> Option<(StoreId, Vec<crate::rr::re_chunk::Chunk>)> {
        self.scene.lock().unwrap().sync_eye(hub)
    }

    /// The 3D view's eye as Rerun has it, and the one fitted to the scene with the bounds it was
    /// fitted to (read-only, for automation).
    pub fn eye(&self, hub: &crate::rr::re_viewer_context::StoreHub) -> (Option<rerun::ViewEye>, Option<(rerun::Eye, Option<rerun::Aabb>)>) {
        let scene = self.scene.lock().unwrap();
        (scene.view_eye(hub).map(|(_, eye)| eye), scene.fitted_eye())
    }

    /// Move the 3D view's eye ([`rerun::RerunScene::command_eye`]).
    pub fn command_camera(&self, command: &factoryos_client::protocol::view::ViewCommand) -> Result<Option<rerun::Eye>, String> {
        let result = self.scene.lock().unwrap().command_eye(command);
        self.waker.wake();
        result
    }

    /// The recording currently shown, if any.
    pub fn current(&self) -> Option<StoreId> {
        self.scene.lock().unwrap().current()
    }

    /// What is still on its way to the 3D view (read-only).
    pub fn pending(&self) -> Pending {
        let mut pending = self.scene.lock().unwrap().pending();
        // Events received but not applied yet (applied in the next `forward`).
        pending.queued |= self.client.has_events();
        pending.rejoin = self.client.backend_away();
        pending
    }

    /// Query the scene's snapshot again (the backend restarted).
    pub fn resync(&self) {
        self.client.resync();
    }

    /// The scene's title, once there is a scene.
    pub fn title(&self) -> Option<String> {
        self.scene.lock().unwrap().title()
    }

    /// Navigation to perform after a new scene started: the new recording, and the replaced ones
    /// to close (one per call, once Rerun has all of it). Call every frame, after [`Self::forward`].
    pub fn take_route_request(&self) -> Option<RouteRequest> {
        self.scene.lock().unwrap().take_route_request()
    }

    /// Show joint values at once, before the producer echoes them (e.g. the Robo Dresser's sliders).
    pub fn set_local_joints(&self, node: &str, joints: &BTreeMap<String, f64>) {
        self.scene.lock().unwrap().set_local_joints(node, joints, &*self.assets);
        self.waker.wake();
    }
}
