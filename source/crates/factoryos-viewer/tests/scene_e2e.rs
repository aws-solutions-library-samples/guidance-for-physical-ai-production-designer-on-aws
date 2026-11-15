//! End to end, without a GUI: the real backend (its binary, Zenoh and HTTP) → the viewer's
//! `scene3d` (scene client, asset cache, Rerun adapter) → a Rerun `EntityDb`, queried the way the
//! viewer's 3D view resolves it (`re_tf`'s `TransformResolutionCache`).
//!
//! Run with `cargo test -p factoryos_viewer --test scene_e2e -- --include-ignored`. Ignored by
//! default: it needs the starter assets (`assets/catalogue`) and builds and starts the backend.
//!
//! **The backend binary**: `$FACTORYOS_BACKEND_BIN` if set (then used as is); else
//! `factoryos-backend` in this test binary's own profile directory (`<target>/<profile>/`, the
//! parent of `deps/` where cargo puts this test), so it follows `CARGO_TARGET_DIR`,
//! `--target-dir` and `--release`. The test first runs `cargo build -p factoryos-backend` (same
//! profile) so the binary is never stale; set `FACTORYOS_E2E_NO_BUILD=1` to skip that.
//!
//! It drives `factoryos_viewer::scene3d` (exported, hidden from the docs, for tests like this).
//!
//! The asset cache is the viewer's own (`AssetCache::new`), including its native disk cache:
//! assets already on disk from an earlier run are not fetched again (they are hash-checked either
//! way), so HTTP is exercised by the first run on a machine and by every asset not cached yet.

#![cfg(not(target_arch = "wasm32"))]
#![allow(dead_code)]

/// The Rerun crates, as `crate::rr` in the viewer.
mod rr {
    pub use re_viewer::external::*;
}

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::process::{Child, Command};
use std::sync::Arc;
use std::time::{Duration, Instant};

use factoryos_client::protocol::scene::{NodeKind, SceneDoc};
use factoryos_client::protocol::{Health, api, keys};
use factoryos_client::{Api, Bus};

use re_importer::external::urdf_rs;
use rr::re_chunk::{Chunk, EntityPath};
use rr::re_chunk_store::LatestAtQuery;
use rr::re_entity_db::EntityDb;
use rr::re_log_channel::{DataSourceMessage, LogReceiver};
use rr::re_log_types::{LogMsg, StoreId, StoreKind, TimelineName};
use rr::re_sdk_types::archetypes::{Asset3D, CoordinateFrame, Mesh3D};
use rr::re_sdk_types::blueprint::archetypes::ViewBlueprint;
use rr::re_sdk_types::components::{Name, TransformFrameId};
use rr::re_sdk_types::external::glam;
use rr::re_tf::{TransformForest, TransformFrameIdHash, TransformResolutionCache};

use factoryos_viewer::scene3d::{self, Scene3d};

/// The whole test, at most (backend build aside).
const TEST_BUDGET: Duration = Duration::from_secs(120);
/// One wait for a state, at most.
const WAIT: Duration = Duration::from_secs(45);

// --- The backend ------------------------------------------------------------------------------

struct Backend {
    child: Child,
    api: Api,
    zenoh: String,
    dir: PathBuf,
}

impl Drop for Backend {
    fn drop(&mut self) {
        // SIGTERM first, so the backend stops whatever it started.
        Command::new("kill").arg(self.child.id().to_string()).status().ok();
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline && self.child.try_wait().ok().flatten().is_none() {
            std::thread::sleep(Duration::from_millis(50));
        }
        self.child.kill().ok();
        self.child.wait().ok();
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

fn workspace_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// The backend binary (see the module docs), built first unless `FACTORYOS_E2E_NO_BUILD` is set.
fn backend_binary() -> PathBuf {
    if let Some(bin) = std::env::var_os("FACTORYOS_BACKEND_BIN") {
        let bin = PathBuf::from(bin);
        assert!(bin.is_file(), "FACTORYOS_BACKEND_BIN={} is not a file", bin.display());
        return bin;
    }
    // <target>/<profile>/deps/scene_e2e-<hash>
    let exe = std::env::current_exe().expect("no current_exe");
    let profile_dir = exe.parent().and_then(|deps| deps.parent()).expect("the test binary is not in <target>/<profile>/deps");
    let bin = profile_dir.join(format!("factoryos-backend{}", std::env::consts::EXE_SUFFIX));
    if std::env::var_os("FACTORYOS_E2E_NO_BUILD").is_none() {
        let profile = profile_dir.file_name().and_then(|n| n.to_str()).unwrap_or("debug");
        let target_dir = profile_dir.parent().expect("no target dir");
        let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
        let mut build = Command::new(cargo);
        build.current_dir(workspace_dir()).args(["build", "-p", "factoryos-backend", "--bin", "factoryos-backend"]);
        build.arg("--target-dir").arg(target_dir);
        match profile {
            "debug" => {}
            "release" => {
                build.arg("--release");
            }
            other => {
                build.args(["--profile", other]);
            }
        }
        let started = Instant::now();
        let status = build.status().expect("failed to run cargo build -p factoryos-backend");
        assert!(status.success(), "cargo build -p factoryos-backend failed ({status}); set FACTORYOS_E2E_NO_BUILD=1 to use {} as is", bin.display());
        eprintln!("[scene_e2e] backend built in {:.1?}", started.elapsed());
    }
    assert!(bin.is_file(), "no backend binary at {}: run `cargo build -p factoryos-backend` (or set FACTORYOS_BACKEND_BIN)", bin.display());
    bin
}

fn free_ports<const N: usize>() -> [u16; N] {
    let listeners: Vec<_> = (0..N).map(|_| std::net::TcpListener::bind("127.0.0.1:0").unwrap()).collect();
    std::array::from_fn(|i| listeners[i].local_addr().unwrap().port())
}

/// The backend on free loopback ports, with a fresh config in a temp dir; the catalogue is its
/// default, this checkout's `assets/catalogue`.
async fn start_backend(bin: &std::path::Path) -> Backend {
    let [port] = free_ports();
    let dir = std::env::temp_dir().join(format!("factoryos-scene-e2e-{}-{port}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // A "free" port can be taken before the backend binds it, and then it exits: try again.
    for _attempt in 0..5 {
        let [http, zenoh, remote_api, zenoh_ws] = free_ports();
        let mut child = Command::new(bin)
            .args(["--bind", &format!("127.0.0.1:{http}")])
            .args(["--zenoh-listen", &format!("tcp/127.0.0.1:{zenoh}")])
            .args(["--remote-api-port", &remote_api.to_string()])
            .args(["--zenoh-ws-port", &zenoh_ws.to_string()])
            .args(["--config", dir.join("factoryos.config.json").to_str().unwrap()])
            .spawn()
            .unwrap_or_else(|err| panic!("failed to start {}: {err}", bin.display()));
        let api = Api::new(format!("http://127.0.0.1:{http}"));
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            match api.get_async::<Health>(api::HEALTH).await {
                Ok(health) => {
                    let zenoh = factoryos_client::zenoh_endpoint(api.base(), &health).expect("no Zenoh endpoint in /health");
                    return Backend { child, api, zenoh, dir };
                }
                Err(_) if child.try_wait().ok().flatten().is_some() => break,
                Err(err) if Instant::now() > deadline => {
                    child.kill().ok();
                    panic!("the backend did not come up within 30 s: {err}");
                }
                Err(_) => tokio::time::sleep(Duration::from_millis(100)).await,
            }
        }
    }
    panic!("the backend did not come up: its ports kept being taken");
}

async fn http<T: serde::de::DeserializeOwned + Send + 'static>(
    what: &str,
    send: impl FnOnce(Box<dyn FnOnce(Result<T, String>) + Send>),
) -> T {
    let (tx, rx) = tokio::sync::oneshot::channel();
    send(Box::new(move |result| {
        tx.send(result).ok();
    }));
    match tokio::time::timeout(WAIT, rx).await {
        Ok(Ok(Ok(value))) => value,
        Ok(Ok(Err(err))) => panic!("{what}: {err}"),
        Ok(Err(_)) => panic!("{what}: no reply"),
        Err(_) => panic!("{what}: no reply within {WAIT:?}"),
    }
}

/// Polls `path` until `done`.
async fn wait_for_state<T: serde::de::DeserializeOwned + Send + std::fmt::Debug + 'static>(
    api: &Api,
    path: &'static str,
    what: &str,
    done: impl Fn(&T) -> bool,
) -> T {
    let deadline = Instant::now() + WAIT;
    loop {
        let api = api.clone();
        let state: T = http(what, |cb| api.get(path, cb)).await;
        if done(&state) {
            return state;
        }
        assert!(Instant::now() < deadline, "timed out ({WAIT:?}) waiting for {what}: {state:?}");
        tokio::time::sleep(Duration::from_millis(150)).await;
    }
}

/// The backend's latest doc of `source` (its snapshot), once `ready`. Observes the backend only,
/// for what to expect; what is asserted is the viewer's store.
async fn snapshot(bus: &Bus, source: &str, what: &str, ready: impl Fn(&SceneDoc) -> bool) -> SceneDoc {
    let deadline = Instant::now() + WAIT;
    loop {
        let (tx, rx) = tokio::sync::oneshot::channel();
        bus.query(&keys::scene_snapshot(source), None, move |reply| {
            tx.send(reply).ok();
        });
        if let Ok(Ok(Ok(Some(bytes)))) = tokio::time::timeout(Duration::from_secs(5), rx).await
            && let Ok(doc) = serde_json::from_slice::<SceneDoc>(&bytes)
            && ready(&doc)
        {
            return doc;
        }
        assert!(Instant::now() < deadline, "timed out ({WAIT:?}) waiting for {what} in the {source} snapshot");
        tokio::time::sleep(Duration::from_millis(150)).await;
    }
}

async fn get_bytes(api: &Api, path: &str) -> Vec<u8> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    api.get_bytes(path, usize::MAX, move |r| {
        tx.send(r).ok();
    });
    tokio::time::timeout(WAIT, rx).await.expect("asset fetch timed out").unwrap().unwrap_or_else(|err| panic!("GET {path}: {err}"))
}

/// The URDF of model node `id` in `doc`.
async fn urdf_of(api: &Api, doc: &SceneDoc, id: &str) -> urdf_rs::Robot {
    let node = doc.node(id).unwrap_or_else(|| panic!("no node {id} in {}", doc.scene_id));
    let NodeKind::Model { model, .. } = &node.kind else { panic!("{id} is not a model: {node:?}") };
    let bytes = get_bytes(api, &api::asset(&model.urdf.sha256)).await;
    urdf_rs::read_from_string(std::str::from_utf8(&bytes).unwrap()).unwrap()
}

// --- The viewer side: scene3d into EntityDbs ----------------------------------------------------

/// One viewer panel's scene, as the Rerun app would ingest it: every store its log channel opens.
struct Viewer {
    what: &'static str,
    scene: Scene3d,
    rx: LogReceiver,
    stores: HashMap<StoreId, EntityDb>,
    /// Recording chunks that are not static: `(entity, timelines)`.
    temporal: Vec<String>,
    activated: Vec<StoreId>,
}

impl Viewer {
    fn connect(what: &'static str, bus: &Bus, assets: Arc<scene3d::assets::AssetCache>, source: &str, app_id: &str) -> Self {
        let mut scene = Scene3d::connect(bus, assets, source, app_id);
        let rx = scene.take_receiver().expect("a receiver");
        Self { what, scene, rx, stores: HashMap::new(), temporal: Vec::new(), activated: Vec::new() }
    }

    /// What the viewer does each frame: `forward`, Rerun drains the channel, `take_route_request`;
    /// repeated until nothing waits.
    fn pump(&mut self) {
        for _ in 0..1000 {
            let busy = self.scene.forward();
            let mut got = false;
            while let Ok(msg) = self.rx.try_recv() {
                got = true;
                if let Some(DataSourceMessage::LogMsg(msg)) = msg.into_data() {
                    self.ingest(&msg);
                }
            }
            while self.scene.take_route_request().is_some() {}
            if !busy && !got {
                return;
            }
        }
    }

    fn ingest(&mut self, msg: &LogMsg) {
        if let LogMsg::BlueprintActivationCommand(cmd) = msg
            && cmd.make_active
        {
            self.activated.push(cmd.blueprint_id.clone());
        }
        if let LogMsg::ArrowMsg(store, arrow) = msg
            && store.kind() == StoreKind::Recording
        {
            let chunk = Chunk::from_arrow_msg(arrow).unwrap();
            if !chunk.is_static() {
                let timelines: Vec<_> = chunk.timelines().keys().map(|t| t.to_string()).collect();
                self.temporal.push(format!("{} on {timelines:?}", chunk.entity_path()));
            }
        }
        let store = msg.store_id().clone();
        let db = self.stores.entry(store.clone()).or_insert_with(|| EntityDb::new(store));
        db.add_log_msg(msg).unwrap_or_else(|err| panic!("{}: Rerun refused a message: {err}", self.what));
    }

    /// Pump until `check` passes on the current recording; panics with its last complaint.
    async fn wait(&mut self, what: &str, check: impl Fn(&Rec<'_>) -> Result<(), String>) {
        let deadline = Instant::now() + WAIT;
        loop {
            self.pump();
            let complaint = match self.rec() {
                Some(rec) => match check(&rec) {
                    Ok(()) => return,
                    Err(err) => err,
                },
                None => "no recording yet".to_owned(),
            };
            assert!(Instant::now() < deadline, "{}: timed out ({WAIT:?}) waiting for {what}: {complaint}", self.what);
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    /// The recording shown.
    fn rec(&self) -> Option<Rec<'_>> {
        let store = self.scene.current()?;
        Some(Rec(self.stores.get(&store)?))
    }

    /// The blueprint (title of the 3D view) of the scene's application, and whether it was activated.
    fn blueprint_title(&self) -> Option<String> {
        let app = self.scene.current()?.application_id().clone();
        let query = LatestAtQuery::latest(TimelineName::from("blueprint"));
        let component = ViewBlueprint::descriptor_display_name().component;
        let (id, db) = self.stores.iter().find(|(id, _)| id.kind() == StoreKind::Blueprint && *id.application_id() == app)?;
        assert!(self.activated.contains(id), "{}: the blueprint {id:?} was never activated", self.what);
        db.sorted_entity_paths().find_map(|entity| {
            let results = db.latest_at(&query, entity, [component]);
            results.component_mono::<Name>(component).map(|name| name.to_string())
        })
    }
}

/// A recording, queried as the 3D view does (latest, static data included).
struct Rec<'a>(&'a EntityDb);

impl Rec<'_> {
    fn query() -> LatestAtQuery {
        LatestAtQuery::latest(TimelineName::log_tick())
    }

    fn has(&self, entity: &EntityPath, component: rr::re_sdk_types::ComponentDescriptor) -> bool {
        let results = self.0.latest_at(&Self::query(), entity, [component.component]);
        results.get(component.component).is_some_and(|c| c.num_instances(component.component) > 0)
    }

    /// Whether `entity` holds mesh data (a `Mesh3D`'s vertices or an `Asset3D`'s file).
    fn has_mesh(&self, entity: &EntityPath) -> bool {
        self.has(entity, Mesh3D::descriptor_vertex_positions()) || self.has(entity, Asset3D::descriptor_blob())
    }

    /// The transform frame `entity` draws in.
    fn frame(&self, entity: &EntityPath) -> Option<String> {
        let component = CoordinateFrame::descriptor_frame().component;
        let results = self.0.latest_at(&Self::query(), entity, [component]);
        results.component_mono::<TransformFrameId>(component).map(|f| f.as_str().to_owned())
    }

    /// World from `frame`, as Rerun resolves it; `None` if it is not connected to the world.
    fn world_from(&self, frame: &str) -> Option<glam::DAffine3> {
        let mut cache = TransformResolutionCache::new(self.0);
        cache.ensure_timeline_is_initialized(self.0.storage_engine().store(), TimelineName::log_tick());
        let forest = TransformForest::new(self.0, &cache, &Self::query());
        let (_, result) = forest
            .transform_from_to(TransformFrameIdHash::entity_path_hierarchy_root(), std::iter::once(TransformFrameIdHash::from_str(frame)), &|_| 1.0)
            .next()?;
        result.ok().map(|t| t.target_from_source)
    }

    /// The visual geometry of the model at node `id`: `link → [(entity, has mesh data)]`, the link
    /// taken from the entity's frame (`<node frame>#<link>`).
    fn visuals(&self, id: &str) -> BTreeMap<String, Vec<(EntityPath, bool)>> {
        let prefix = format!("/{id}/");
        let mut links: BTreeMap<String, Vec<(EntityPath, bool)>> = BTreeMap::new();
        for entity in self.0.sorted_entity_paths() {
            let path = entity.to_string();
            if !path.starts_with(&prefix) || !path.contains("/visual_geometries/") {
                continue;
            }
            if let Some(frame) = self.frame(entity)
                && let Some((_, link)) = frame.split_once('#')
            {
                links.entry(link.to_owned()).or_default().push((entity.clone(), self.has_mesh(entity)));
            }
        }
        links
    }

    /// The frame of link `link` of node `id`, as its visual geometry names it.
    fn link_frame(&self, id: &str, link: &str) -> Option<String> {
        let (entity, _) = self.visuals(id).remove(link)?.into_iter().next()?;
        self.frame(&entity)
    }

    /// Every link of `urdf` with a mesh visual has a mesh entity in the store, in a frame the
    /// world reaches.
    fn check_model(&self, id: &str, urdf: &urdf_rs::Robot) -> Result<(), String> {
        let visuals = self.visuals(id);
        let mesh_links = urdf.links.iter().filter(|l| l.visual.iter().any(|v| matches!(v.geometry, urdf_rs::Geometry::Mesh { .. })));
        for link in mesh_links {
            let Some(entities) = visuals.get(&link.name) else { return Err(format!("no visual geometry of link {}", link.name)) };
            if !entities.iter().any(|(_, mesh)| *mesh) {
                return Err(format!("link {}: no mesh data in {entities:?}", link.name));
            }
            let frame = self.frame(&entities[0].0).unwrap();
            if self.world_from(&frame).is_none() {
                return Err(format!("link {}'s frame {frame} is not reachable from the world", link.name));
            }
        }
        if visuals.is_empty() {
            return Err(format!("no visual geometry under /{id}"));
        }
        Ok(())
    }
}

fn mesh_link_count(urdf: &urdf_rs::Robot) -> usize {
    urdf.links.iter().filter(|l| l.visual.iter().any(|v| matches!(v.geometry, urdf_rs::Geometry::Mesh { .. }))).count()
}

// --- The test --------------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs the starter assets (assets/catalogue) and builds/starts the backend binary"]
async fn backend_scenes_become_rerun_stores() {
    let catalogue = workspace_dir().join("assets/catalogue");
    assert!(catalogue.join("urdf-robot-collection").is_dir(), "no catalogue at {}", catalogue.display());
    factoryos_client::install_crypto_provider();
    let bin = backend_binary();
    let started = Instant::now();
    let budget = tokio::time::timeout(TEST_BUDGET, async {
        let backend = start_backend(&bin).await;
        let bus = Bus::connect(&backend.zenoh).await.unwrap_or_else(|err| panic!("Zenoh {}: {err}", backend.zenoh));
        let assets = Scene3d::asset_cache(&backend.api);
        let mut dresser = Viewer::connect("dresser", &bus, assets.clone(), keys::SCENE_DRESSER, "robo_dresser");
        let mut fitter = Viewer::connect("fitter", &bus, assets.clone(), keys::SCENE_FITTER, "robo_fitter");
        eprintln!("[scene_e2e] backend up, viewer connected after {:.1?}", started.elapsed());

        dresser_slider_moves_the_link(&backend, &bus, &mut dresser).await;
        eprintln!("[scene_e2e] dresser done after {:.1?}", started.elapsed());
        fitter_move_moves_the_robot(&backend, &bus, &mut fitter).await;
        eprintln!("[scene_e2e] fitter done after {:.1?}", started.elapsed());

        for viewer in [&dresser, &fitter] {
            assert!(viewer.temporal.is_empty(), "{}: temporal recording chunks: {:?}", viewer.what, viewer.temporal);
        }
    })
    .await;
    assert!(budget.is_ok(), "the test took longer than {TEST_BUDGET:?}");
    eprintln!("[scene_e2e] total {:.1?}", started.elapsed());
}

/// Robo Dresser: preview a catalogue robot; its meshes and frames are in the viewer's store; a
/// joint slider (`keys::DRESSER_JOINT`, echoed by the backend as a joint frame) turns the joint's
/// child link by exactly the change, relative to its parent link.
async fn dresser_slider_moves_the_link(backend: &Backend, bus: &Bus, viewer: &mut Viewer) {
    use factoryos_client::protocol::dresser::{AssetKind, DresserState, JointUpdate, OpenRequest, PreviewRequest};
    let api = backend.api.clone();
    let _: DresserState = http("dresser open", |cb| api.post_json(api::DRESSER_OPEN, &OpenRequest { rescan: false }, cb)).await;
    let state: DresserState =
        wait_for_state(&api, api::DRESSER_STATE, "the catalogue scan", |s: &DresserState| s.busy.is_none() && !s.robots.is_empty()).await;
    let request = PreviewRequest { kind: AssetKind::Robot, name: state.robots[0].name.clone() };
    let _: DresserState = http("dresser preview", |cb| api.post_json(api::DRESSER_PREVIEW, &request, cb)).await;
    let state: DresserState =
        wait_for_state(&api, api::DRESSER_STATE, "the preview", |s: &DresserState| s.busy.is_none() && s.preview.is_some()).await;
    let preview = state.preview.unwrap();
    let doc = snapshot(bus, keys::SCENE_DRESSER, "the preview's robot", |doc| doc.node("robot").is_some()).await;
    let urdf = urdf_of(&api, &doc, "robot").await;
    assert!(mesh_link_count(&urdf) > 0, "{}: the URDF has no mesh visuals", request.name);

    // Geometry with mesh data, every link reachable from the world.
    viewer.wait("the robot's meshes and frames", |rec| rec.check_model("robot", &urdf)).await;
    let title = viewer.blueprint_title().expect("dresser: no blueprint for the recording's application");
    assert_eq!(title, "Robo Dresser");

    // The slider: a movable joint, its value changed noticeably.
    let joint = preview.joints.iter().find(|j| j.upper - j.lower > 0.2).expect("no movable joint with a range");
    let urdf_joint = urdf.joints.iter().find(|j| j.name == joint.name).unwrap_or_else(|| panic!("joint {} not in the URDF", joint.name));
    let (parent, child) = (urdf_joint.parent.link.clone(), urdf_joint.child.link.clone());
    let NodeKind::Model { joints, .. } = &doc.node("robot").unwrap().kind else { unreachable!() };
    let from = joints.get(&joint.name).copied().unwrap_or(0.0);
    let to = if (joint.lower + (joint.upper - joint.lower) * 0.3 - from).abs() > 0.1 {
        joint.lower + (joint.upper - joint.lower) * 0.3
    } else {
        joint.lower + (joint.upper - joint.lower) * 0.7
    };

    // The link frames, by the geometry's frames (a link without visuals: `robot#<link>`).
    let rec = viewer.rec().unwrap();
    let frame_of = |link: &str| rec.link_frame("robot", link).unwrap_or_else(|| format!("robot#{link}"));
    let (parent_frame, child_frame) = (frame_of(&parent), frame_of(&child));
    let relative = |rec: &Rec<'_>| -> Option<glam::DAffine3> { Some(rec.world_from(&parent_frame)?.inverse() * rec.world_from(&child_frame)?) };
    let before = relative(&rec).unwrap_or_else(|| panic!("dresser: {parent_frame} or {child_frame} is not reachable from the world"));
    let parent_before = rec.world_from(&parent_frame).unwrap();

    let update = JointUpdate { name: joint.name.clone(), value: to };
    bus.put(keys::DRESSER_JOINT, serde_json::to_vec(&update).unwrap());
    let what = format!("joint {} ({:?}) {from:.3} → {to:.3} to move link {child}", joint.name, urdf_joint.joint_type);
    let joint_type = urdf_joint.joint_type.clone();
    let axis = glam::DVec3::from(urdf_joint.axis.xyz.0).normalize();
    viewer
        .wait(&what, |rec| {
            let after = relative(rec).ok_or("the link frames are no longer reachable")?;
            let change = before.inverse() * after;
            let expected = match joint_type {
                urdf_rs::JointType::Prismatic => glam::DAffine3::from_translation(axis * (to - from)),
                _ => glam::DAffine3::from_axis_angle(axis, to - from),
            };
            if change.abs_diff_eq(expected, 1e-4) { Ok(()) } else { Err(format!("child from parent changed by {change:?}, expected {expected:?}")) }
        })
        .await;
    // The parent link stayed where it was.
    let rec = viewer.rec().unwrap();
    assert!(rec.world_from(&parent_frame).unwrap().abs_diff_eq(parent_before, 1e-6), "dresser: the parent link {parent} moved too");
}

/// Robo Fitter: a saved scene (or a new one with a dressed robot); its first item's meshes and
/// frames are in the store; moving it +0.5 m in x (a scene edit over Zenoh) moves its geometry by
/// exactly that in the world.
async fn fitter_move_moves_the_robot(backend: &Backend, bus: &Bus, viewer: &mut Viewer) {
    use factoryos_client::protocol::fitter::{FitterState, FixtureYaml, LoadRequest, RobotYaml, SceneEdit};
    let api = backend.api.clone();
    let state: FitterState = http("fitter open", |cb| api.post(api::FITTER_OPEN, cb)).await;
    let state: FitterState = match state.palette.scenes.first().cloned() {
        Some(file_name) => http("fitter load", |cb| api.post_json(api::FITTER_LOAD, &LoadRequest { file_name }, cb)).await,
        None => {
            let mut scene = state.scene.clone();
            if let Some(name) = state.palette.robots.first() {
                scene.robots.push(RobotYaml {
                    robot_id: name.clone(),
                    dressed_urdf: format!("../urdf-dressed-robots/{name}/{name}.urdf"),
                    ..Default::default()
                });
            } else {
                let name = state.palette.fixtures.first().expect("no robots or fixtures in the catalogue");
                scene.fixtures.push(FixtureYaml { id: name.clone(), urdf: format!("../urdf-fixture-collection/{name}/{name}.urdf"), ..Default::default() });
            }
            let edit = SceneEdit { file_name: state.scene_file_name.clone(), scene: scene.clone() };
            bus.put(keys::FITTER_SCENE, serde_json::to_vec(&edit).unwrap());
            wait_for_state(&api, api::FITTER_STATE, "the item added", |s: &FitterState| s.scene == scene).await
        }
    };
    let robots = state.scene.robots.len();
    let state: FitterState = wait_for_state(&api, api::FITTER_STATE, "the scene's items loaded", |s: &FitterState| {
        s.busy.is_none() && s.robot_dofs.len() == robots && s.status.starts_with("Loaded")
    })
    .await;
    let id = match state.scene.robots.first() {
        Some(robot) => format!("robots/{}", robot.robot_id),
        None => format!("fixtures/{}", state.scene.fixtures[0].id),
    };
    let doc = snapshot(bus, keys::SCENE_FITTER, "the scene's first item", |doc| matches!(doc.node(&id).map(|n| &n.kind), Some(NodeKind::Model { .. }))).await;
    let urdf = urdf_of(&api, &doc, &id).await;

    viewer.wait(&format!("{id}'s meshes and frames"), |rec| rec.check_model(&id, &urdf)).await;
    let title = viewer.blueprint_title().expect("fitter: no blueprint for the recording's application");
    assert!(title.starts_with("Robo Fitter"), "fitter: blueprint title {title:?}");

    // Every link's world pose before the move.
    let rec = viewer.rec().unwrap();
    let before: BTreeMap<String, glam::DAffine3> = rec
        .visuals(&id)
        .into_iter()
        .map(|(link, entities)| {
            let frame = rec.frame(&entities[0].0).unwrap();
            (link, rec.world_from(&frame).unwrap())
        })
        .collect();
    assert_eq!(before.len(), rec.visuals(&id).len());

    let mut scene = state.scene.clone();
    match scene.robots.first_mut() {
        Some(robot) => robot.position.x += 0.5,
        None => scene.fixtures[0].position.x += 0.5,
    }
    let edit = SceneEdit { file_name: state.scene_file_name.clone(), scene: scene.clone() };
    bus.put(keys::FITTER_SCENE, serde_json::to_vec(&edit).unwrap());
    let shift = glam::DAffine3::from_translation(glam::DVec3::new(0.5, 0.0, 0.0));
    viewer
        .wait(&format!("{id} to move +0.5 m in x"), |rec| {
            let visuals = rec.visuals(&id);
            if !before.keys().all(|link| visuals.contains_key(link)) {
                return Err(format!("links {:?} of {:?} drawn", visuals.keys().collect::<Vec<_>>(), before.keys().collect::<Vec<_>>()));
            }
            for (link, entities) in visuals {
                let Some(old) = before.get(&link) else { continue };
                let frame = rec.frame(&entities[0].0).unwrap();
                let now = rec.world_from(&frame).ok_or(format!("link {link} ({frame}) not reachable"))?;
                if !now.abs_diff_eq(shift * *old, 1e-4) {
                    return Err(format!("link {link}: {now:?}, expected {:?}", shift * *old));
                }
            }
            Ok(())
        })
        .await;
}
