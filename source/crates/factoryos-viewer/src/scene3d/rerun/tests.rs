use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::Duration;

use web_time::Instant;

use factoryos_client::protocol::scene::{
    AssetRef, JointFrame, MeshAsset, ModelJoints, ModelRef, Node, NodeKind, NodePose, PoseFrame, Pose3, SceneDelta, SceneDoc, ViewHints,
};

use super::*;
use crate::rr::re_chunk::{Chunk, RowId};
use crate::rr::re_sdk_types::external::glam::Vec3;
use crate::rr::re_sdk_types::Archetype as _;
use crate::rr::re_sdk_types::archetypes::{Asset3D, Boxes3D, Clear, Mesh3D, Transform3D};
use crate::rr::re_sdk_types::blueprint::archetypes::ViewContents;
use crate::rr::re_sdk_types::AsComponents as _;
use crate::scene3d::{AssetState, Priority};

const URDF: &str = r#"<robot name="bot">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry><material name="red"/></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="arm">
    <visual><geometry><mesh filename="package://bot/arm.obj"/></geometry></visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"#;

const OBJ: &str = "mtllib arm.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nusemtl blue\nf 1 2 3\n";
const MTL: &str = "newmtl blue\nKd 0 0 1\n";

fn asset(name: &str, media_type: &str) -> AssetRef {
    AssetRef { sha256: name.to_owned(), size: 0, media_type: media_type.to_owned() }
}

/// A scene that converts OBJs inline (as on the web), so that tests see them at once.
fn new_scene() -> RerunScene {
    let mut scene = RerunScene::new("factoryos_test");
    scene.convert = entities::Converter::inline();
    scene
}

/// Assets by `sha256`; missing ones are pending, unless they fail (retryable or not). Counts the
/// requests per asset, and logs them in order with their priority.
#[derive(Default)]
struct Assets(
    HashMap<String, Arc<[u8]>>,
    HashMap<String, bool>,
    std::cell::RefCell<HashMap<String, usize>>,
    std::cell::RefCell<Vec<(String, Priority)>>,
);

impl Assets {
    fn all() -> Self {
        let mut assets = Self::default();
        assets.add("urdf", URDF);
        assets.add("obj", OBJ);
        assets.add("mtl", MTL);
        assets
    }

    fn add(&mut self, sha: &str, bytes: &str) {
        self.1.remove(sha);
        self.0.insert(sha.to_owned(), bytes.as_bytes().into());
    }

    fn fail(&mut self, sha: &str, retryable: bool) {
        self.0.remove(sha);
        self.1.insert(sha.to_owned(), retryable);
    }

    fn requests(&self, sha: &str) -> usize {
        self.2.borrow().get(sha).copied().unwrap_or(0)
    }
}

impl AssetSource for Assets {
    fn get_with(&self, asset: &AssetRef, priority: Priority) -> AssetState {
        *self.2.borrow_mut().entry(asset.sha256.clone()).or_default() += 1;
        self.3.borrow_mut().push((asset.sha256.clone(), priority));
        if let Some(&retryable) = self.1.get(&asset.sha256) {
            return AssetState::Failed { err: "404".into(), retryable };
        }
        match self.0.get(&asset.sha256) {
            Some(bytes) => AssetState::Ready(bytes.clone()),
            None => AssetState::Pending,
        }
    }
}

fn arm_mesh() -> MeshAsset {
    MeshAsset { asset: asset("obj", "model/obj"), sidecars: BTreeMap::from([("arm.mtl".to_owned(), asset("mtl", "text/plain"))]), ..MeshAsset::default() }
}

fn robot(show_collision: bool) -> Node {
    Node::new(
        "robots/r1",
        NodeKind::Model {
            model: ModelRef {
                urdf: asset("urdf", "application/xml"),
                meshes: BTreeMap::from([("package://bot/arm.obj".to_owned(), arm_mesh())]),
            },
            joints: BTreeMap::from([("j1".to_owned(), 0.5)]),
            show_collision,
        },
    )
    .at(Pose3::from_xyz([1.0, 0.0, 0.0]))
}

fn boxes() -> Node {
    Node::new(
        "robots/r1/tcp",
        NodeKind::Boxes {
            centers: vec![[0.0; 3]],
            half_sizes: vec![[0.1; 3]],
            quats_xyzw: vec![],
            colors: vec![],
            labels: vec![],
            solid: false,
            show_labels: None,
        },
    )
    .under("robots/r1", Some("arm".to_owned()))
}

fn doc(scene_id: &str, nodes: Vec<Node>) -> SceneDoc {
    SceneDoc { scene_id: scene_id.to_owned(), seq: 1, view: ViewHints { title: "Robo Fitter".to_owned() }, nodes }
}

/// The recording's chunks queued so far (taken), and the other messages, less the view's eye
/// (which follows the scene's bounds, see [`framing`]).
fn take(scene: &mut RerunScene) -> (Vec<Chunk>, Vec<LogMsg>) {
    let eye_entity = scene.blueprint.eye_entity();
    let (chunks, mut others) = take_with_eye(scene);
    others.retain(|msg| match msg {
        LogMsg::ArrowMsg(store, arrow_msg) if store.kind() == StoreKind::Blueprint => {
            Chunk::from_arrow_msg(arrow_msg).unwrap().entity_path() != &eye_entity
        }
        _ => true,
    });
    (chunks, others)
}

/// [`take`], the view's eye included.
fn take_with_eye(scene: &mut RerunScene) -> (Vec<Chunk>, Vec<LogMsg>) {
    let mut chunks = Vec::new();
    let mut others = Vec::new();
    for msg in scene.queue.take_all() {
        match &msg {
            LogMsg::ArrowMsg(store, arrow_msg) if store.kind() == StoreKind::Recording => {
                chunks.push(Chunk::from_arrow_msg(arrow_msg).unwrap());
            }
            _ => others.push(msg),
        }
    }
    (chunks, others)
}

fn has(chunks: &[Chunk], entity: &str, archetype: &str) -> bool {
    chunks.iter().any(|chunk| {
        chunk.entity_path().to_string() == entity
            && chunk.component_descriptors().any(|d| d.archetype.is_some_and(|a| a.as_str() == archetype))
    })
}

fn entities_with(chunks: &[Chunk], archetype: &str) -> Vec<String> {
    let mut entities: Vec<_> = chunks
        .iter()
        .filter(|chunk| chunk.component_descriptors().any(|d| d.archetype.is_some_and(|a| a.as_str() == archetype)))
        .map(|chunk| chunk.entity_path().to_string())
        .collect();
    entities.sort();
    entities.dedup();
    entities
}

/// The view's query expressions, as last logged.
fn view_query(msgs: &[LogMsg]) -> Option<Vec<String>> {
    let mut query = None;
    for msg in msgs {
        let LogMsg::ArrowMsg(store, arrow_msg) = msg else { continue };
        if store.kind() != StoreKind::Blueprint {
            continue;
        }
        let chunk = Chunk::from_arrow_msg(arrow_msg).unwrap();
        let component = ViewContents::descriptor_query().component;
        if chunk.components().contains_component(component) {
            query = Some(chunk.iter_slices::<String>(component).flatten().map(|s| s.to_string()).collect());
        }
    }
    query
}

#[test]
fn a_scene_is_a_recording_with_an_active_blueprint() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes()]))], &assets);

    let store = scene.current().expect("a recording");
    let route = scene.take_route_request().expect("a route");
    assert_eq!((route.close, route.open), (None, store.clone()));

    let (chunks, others) = take(&mut scene);
    assert!(matches!(&others[0], LogMsg::SetStoreInfo(info) if info.info.store_id == store));
    assert!(others.iter().any(|m| matches!(m, LogMsg::BlueprintActivationCommand(c) if c.make_active)));
    assert_eq!(view_query(&others).unwrap(), [blueprint::VIEW_QUERY], "the whole recording");

    // Placements, the model's root and joint transforms.
    for entity in ["/robots/r1", "/robots/r1/tcp", "/robots/r1/bot", "/robots/r1/joints/j1"] {
        assert!(has(&chunks, entity, Transform3D::name().as_str()), "{entity}");
    }
    assert!(has(&chunks, "/robots/r1/tcp", Boxes3D::name().as_str()));
    // The URDF's box (visual and collision) and its OBJ, with the MTL's colours.
    let boxes = entities_with(&chunks, Boxes3D::name().as_str());
    assert!(boxes.iter().any(|e| e.starts_with("/robots/r1/bot/visual_geometries/base")), "{boxes:?}");
    assert!(!boxes.iter().any(|e| e.starts_with("/robots/r1/bot/collision_geometries/")), "not shown: not logged {boxes:?}");
    let meshes = entities_with(&chunks, Mesh3D::name().as_str());
    assert_eq!(meshes.len(), 1, "{meshes:?}");
    assert!(meshes[0].starts_with("/robots/r1/bot/visual_geometries/arm"), "{meshes:?}");
    let mesh = chunks.iter().find(|c| c.entity_path().to_string() == meshes[0]).unwrap();
    assert!(mesh.components().contains_component(Mesh3D::descriptor_vertex_colors().component));
    assert!(entities_with(&chunks, Asset3D::name().as_str()).is_empty());
}

#[test]
fn geometry_waits_for_its_assets() {
    let mut scene = new_scene();
    let mut assets = Assets::default();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(true)]))], &assets);
    let (chunks, others) = take(&mut scene);
    assert!(has(&chunks, "/robots/r1", Transform3D::name().as_str()));
    assert!(entities_with(&chunks, Boxes3D::name().as_str()).is_empty());
    assert_eq!(view_query(&others).unwrap(), [blueprint::VIEW_QUERY]);

    // The URDF arrives, its mesh's sidecar not yet: the boxes are drawn, the mesh waits.
    assets.add("urdf", URDF);
    assets.add("obj", OBJ);
    scene.apply(Vec::new(), &assets);
    let (chunks, _) = take(&mut scene);
    assert!(has(&chunks, "/robots/r1/joints/j1", Transform3D::name().as_str()));
    assert!(!entities_with(&chunks, Boxes3D::name().as_str()).is_empty());
    assert!(entities_with(&chunks, Mesh3D::name().as_str()).is_empty());
    assert_eq!(scene.pending.len(), 1);

    assets.add("mtl", MTL);
    scene.apply(Vec::new(), &assets);
    let (chunks, others) = take(&mut scene);
    assert_eq!(entities_with(&chunks, Mesh3D::name().as_str()).len(), 1);
    assert!(scene.pending.is_empty());
    assert!(others.is_empty(), "the blueprint did not change (collision shown)");
}

#[test]
fn a_colour_replaces_the_materials() {
    let mesh = entities::obj_mesh(OBJ.as_bytes(), &BTreeMap::from([("arm.mtl".to_owned(), MTL.as_bytes().into())]), None);
    let colored = mesh.as_serialized_batches();
    assert!(colored.iter().any(|b| b.descriptor == Mesh3D::descriptor_vertex_colors()));
    let mesh = entities::obj_mesh(
        OBJ.as_bytes(),
        &BTreeMap::from([("arm.mtl".to_owned(), MTL.as_bytes().into())]),
        Some(Rgba32::from_unmultiplied_rgba(0, 255, 0, 255)),
    );
    let batches = mesh.as_serialized_batches();
    assert!(!batches.iter().any(|b| b.descriptor == Mesh3D::descriptor_vertex_colors()));
    assert!(batches.iter().any(|b| b.descriptor == Mesh3D::descriptor_albedo_factor()));
}

#[test]
fn edits_clear_what_they_replace() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes()]))], &assets);
    take(&mut scene);

    // Replace the model (now hidden), then remove it with its TCP.
    let delta = SceneDelta { scene_id: "s1".into(), seq: 2, upsert: vec![robot(false).hidden()], remove: vec![], view: None };
    scene.apply(vec![SceneEvent::Delta(delta)], &assets);
    let (chunks, _) = take(&mut scene);
    let clears = entities_with(&chunks, Clear::name().as_str());
    assert_eq!(clears, ["/robots/r1", "/robots/r1/bot", "/robots/r1/joints"]);
    assert!(entities_with(&chunks, Boxes3D::name().as_str()).is_empty(), "hidden: nothing drawn");
    assert!(has(&chunks, "/robots/r1/joints/j1", Transform3D::name().as_str()), "its frames are still there");

    let delta = SceneDelta { scene_id: "s1".into(), seq: 3, upsert: vec![], remove: vec!["robots".into()], view: None };
    scene.apply(vec![SceneEvent::Delta(delta)], &assets);
    let (chunks, others) = take(&mut scene);
    assert_eq!(entities_with(&chunks, Clear::name().as_str()), ["/robots"]);
    assert!(scene.models.is_empty());
    assert!(others.is_empty(), "the blueprint did not change");

    // A delta that does not follow is ignored (the client resyncs).
    let delta = SceneDelta { scene_id: "s1".into(), seq: 9, upsert: vec![boxes()], remove: vec![], view: None };
    scene.apply(vec![SceneEvent::Delta(delta)], &assets);
    assert!(take(&mut scene).0.is_empty());
}

#[test]
fn joints_log_only_what_changed() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    take(&mut scene);

    let frame = |scene_id: &str, value: f64| JointFrame {
        scene_id: scene_id.into(),
        t: None,
        models: vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("j1".into(), value)]) }],
    };
    scene.apply(vec![SceneEvent::Joints(frame("s1", 0.5))], &assets);
    assert!(take(&mut scene).0.is_empty(), "unchanged");
    scene.apply(vec![SceneEvent::Joints(frame("other", 1.0))], &assets);
    assert!(take(&mut scene).0.is_empty(), "another scene's");
    scene.apply(vec![SceneEvent::Joints(frame("s1", 1.0))], &assets);
    let (chunks, _) = take(&mut scene);
    assert_eq!(chunks.iter().map(|c| c.entity_path().to_string()).collect::<Vec<_>>(), ["/robots/r1/joints/j1"]);

    scene.set_local_joints("robots/r1", &BTreeMap::from([("j1".into(), -1.0)]), &assets);
    let (chunks, _) = take(&mut scene);
    assert_eq!(chunks.len(), 1);
    let Some(Node { kind: NodeKind::Model { joints, .. }, .. }) = scene.doc.as_ref().unwrap().node("robots/r1") else { panic!() };
    assert_eq!(joints["j1"], -1.0, "folded into the scene, for a later replacement of the node");
}

#[test]
fn a_new_scene_is_a_new_recording_a_resync_is_not() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![boxes()]))], &assets);
    let first = scene.current().unwrap();
    scene.take_route_request();
    take(&mut scene);

    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![boxes()]))], &assets);
    assert_eq!(scene.current(), Some(first.clone()));
    assert!(scene.take_route_request().is_none());
    let (chunks, others) = take(&mut scene);
    assert_eq!(entities_with(&chunks, Clear::name().as_str()), ["/"]);
    assert!(others.is_empty());

    let mut renamed = doc("s2", vec![boxes()]);
    renamed.view.title = "Robo Dresser".into();
    scene.apply(vec![SceneEvent::Reset(renamed)], &assets);
    let second = scene.current().unwrap();
    assert_ne!(second, first);
    let route = scene.take_route_request().unwrap();
    assert_eq!((route.close, route.open), (None, second.clone()), "closed once Rerun has all of it");
    let (chunks, others) = take(&mut scene);
    scene.forward();
    scene.forward();
    let route = scene.take_route_request().unwrap();
    assert_eq!((route.close, route.open), (Some(first), second.clone()));
    assert!(entities_with(&chunks, Clear::name().as_str()).is_empty());
    assert!(matches!(&others[0], LogMsg::SetStoreInfo(info) if info.info.store_id == second));
    // The new title: the view is logged again, the blueprint not activated again.
    assert!(others.iter().any(|m| matches!(m, LogMsg::ArrowMsg(store, _) if store.kind() == StoreKind::Blueprint)));
    assert!(!others.iter().any(|m| matches!(m, LogMsg::BlueprintActivationCommand(_))));
}

// --- What producers rely on, checked against Rerun's own store and transform resolution ---

use crate::rr::re_chunk_store::LatestAtQuery;
use crate::rr::re_entity_db::EntityDb;
use crate::rr::re_log_types::{StoreInfo, TimelineName};
use crate::rr::re_sdk_types::archetypes::{Arrows3D, CoordinateFrame, Cylinders3D, Points3D};
use crate::rr::re_sdk_types::components::{ShowLabels, Text, TransformFrameId};
use crate::rr::re_sdk_types::external::glam;
use crate::rr::re_sdk_types::ComponentDescriptor;
use crate::rr::re_tf::{TransformForest, TransformFrameIdHash, TransformResolutionCache};

/// What the viewer's store holds of a recording: every chunk the scene logged so far.
struct Store(EntityDb);

impl Store {
    fn new() -> Self {
        Self(EntityDb::new(StoreInfo::testing().store_id))
    }

    /// Add what `scene` queued since.
    fn take(&mut self, scene: &mut RerunScene) {
        for chunk in take(scene).0 {
            self.0.add_chunk(&Arc::new(chunk)).unwrap();
        }
    }

    fn query() -> LatestAtQuery {
        LatestAtQuery::latest(TimelineName::log_tick())
    }

    /// Whether `entity` holds `component` (as of the latest clear).
    fn has(&self, entity: &str, component: ComponentDescriptor) -> bool {
        let results = self.0.latest_at(&Self::query(), &EntityPath::from(entity), [component.component]);
        results.get(component.component).is_some_and(|c| c.num_instances(component.component) > 0)
    }

    /// The transform frame `entity` draws in (its `CoordinateFrame`).
    fn frame(&self, entity: &str) -> Option<String> {
        let component = CoordinateFrame::descriptor_frame().component;
        let results = self.0.latest_at(&Self::query(), &EntityPath::from(entity), [component]);
        results.component_mono::<TransformFrameId>(component).map(|f| f.as_str().to_owned())
    }

    /// World from `frame`, as Rerun resolves it; `None` if `frame` is not connected to the world.
    fn world_from(&self, frame: &str) -> Option<glam::DAffine3> {
        let mut cache = TransformResolutionCache::new(&self.0);
        cache.ensure_timeline_is_initialized(self.0.storage_engine().store(), TimelineName::log_tick());
        let forest = TransformForest::new(&self.0, &cache, &Self::query());
        let (_, result) = forest
            .transform_from_to(
                TransformFrameIdHash::entity_path_hierarchy_root(),
                std::iter::once(TransformFrameIdHash::from_str(frame)),
                &|_| 1.0,
            )
            .next()?;
        result.ok().map(|t| t.target_from_source)
    }

    /// World from the frame `entity` draws in.
    fn world(&self, entity: &str) -> Option<glam::DAffine3> {
        self.world_from(&self.frame(entity).unwrap_or_else(|| panic!("{entity} has no CoordinateFrame")))
    }

    /// Entities with `component` below `prefix`.
    fn below(&self, prefix: &str, component: ComponentDescriptor) -> Vec<String> {
        let mut entities: Vec<String> = self
            .0
            .sorted_entity_paths()
            .map(|e| e.to_string())
            .filter(|e| e.starts_with(prefix) && self.has(e, component.clone()))
            .collect();
        entities.sort();
        entities
    }
}

fn assert_at(world: Option<glam::DAffine3>, xyz: [f64; 3], rot_z: f64, what: &str) {
    let world = world.unwrap_or_else(|| panic!("{what}: not connected to the world"));
    let expected = glam::DAffine3::from_rotation_translation(glam::DQuat::from_rotation_z(rot_z), glam::DVec3::from(xyz));
    assert!(world.abs_diff_eq(expected, 1e-5), "{what}: {world:?}, expected {expected:?}");
}

fn pedestal(slot: &str, center: [f64; 3]) -> Node {
    Node::new(
        format!("robots/{slot}/pedestal"),
        NodeKind::Cylinder { height: 0.5, radius: 0.2, center, color: None, solid: true },
    )
}

/// (a) A node is placed by `Node.parent`, never by its id: Rerun's entity tree must not move it.
/// Dresser: `robot/tcp/approach` hangs off the robot's link (not off `robot/tcp`); training and
/// fitter: `robots/slot0/pedestal` is in the world (not in `robots/slot0`).
#[test]
fn nodes_are_placed_by_parent_not_by_id() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let tcp = Node::new("robots/r1/tcp", NodeKind::Group).under("robots/r1", Some("arm".to_owned())).at(Pose3::from_xyz([0.0, 0.0, 0.3]));
    let approach = Node::new(
        "robots/r1/tcp/approach",
        NodeKind::Arrows {
            origins: vec![[0.0, 0.0, 0.3]],
            vectors: vec![[0.0, 0.0, 0.25]],
            colors: vec![],
            labels: vec!["approach".to_owned()],
            radius: Some(0.005),
            show_labels: Some(true),
        },
    )
    .under("robots/r1", Some("arm".to_owned()));
    let axes = Node::new("robots/r1/tcp/axes", NodeKind::Axes { length: 0.1, radius: None }).under("robots/r1/tcp", None);
    let pedestal = pedestal("r1", [1.0, 0.0, 0.25]);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), tcp, approach, axes, pedestal]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);

    // Every drawing entity names its own frame; frames are node ids and `<id>#<link>`.
    assert_eq!(store.frame("/robots/r1/tcp/approach").as_deref(), Some("robots/r1/tcp/approach"));
    assert_eq!(store.frame("/robots/r1/pedestal").as_deref(), Some("robots/r1/pedestal"));
    assert_eq!(store.frame("/robots/r1/bot").as_deref(), Some("robots/r1#base"));

    // The robot at x = 1, its arm turned by j1 = 0.5 about z.
    assert_at(store.world("/robots/r1/bot"), [1.0, 0.0, 0.0], 0.0, "robot root link");
    assert_at(store.world_from("robots/r1#arm"), [1.0, 0.0, 0.0], 0.5, "arm link");
    // The approach arrow is in the arm's frame, not offset by the TCP's 0.3 m.
    assert_at(store.world("/robots/r1/tcp/approach"), [1.0, 0.0, 0.0], 0.5, "approach");
    assert_at(store.world("/robots/r1/tcp/axes"), [1.0, 0.0, 0.3], 0.5, "TCP axes, in the TCP");
    // The pedestal is in the world, not moved by its id-parent `robots/r1` (at x = 1).
    assert_at(store.world("/robots/r1/pedestal"), [0.0; 3], 0.0, "pedestal");

    // The approach arrow's label, forced visible, and its radius.
    assert!(store.has("/robots/r1/tcp/approach", Arrows3D::descriptor_labels()));
    assert!(store.has("/robots/r1/tcp/approach", Arrows3D::descriptor_radii()));
    let show = Arrows3D::descriptor_show_labels().component;
    let results = store.0.latest_at(&Store::query(), &EntityPath::from("/robots/r1/tcp/approach"), [show]);
    assert_eq!(results.component_mono::<ShowLabels>(show), Some(ShowLabels::from(true)));
    let text = Arrows3D::descriptor_labels().component;
    let results = store.0.latest_at(&Store::query(), &EntityPath::from("/robots/r1/tcp/approach"), [text]);
    assert_eq!(results.component_batch::<Text>(text).unwrap()[0].as_str(), "approach");
}

/// (b) `visible = false` hides only the node's own geometry: a hidden model (an unused training
/// slot) keeps its id-children (its pedestal), and a hidden child keeps its parent.
#[test]
fn visibility_is_per_node() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let mut slot1 = robot(false).hidden();
    slot1.id = "robots/slot1".to_owned();
    let mut slot0 = robot(false);
    slot0.id = "robots/slot0".to_owned();
    let nodes = vec![slot0, pedestal("slot0", [1.0, 0.0, 0.25]).hidden(), slot1, pedestal("slot1", [2.0, 0.0, 0.25])];
    scene.apply(vec![SceneEvent::Reset(doc("s1", nodes))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);

    let cylinder = Cylinders3D::descriptor_lengths();
    let shapes = Boxes3D::descriptor_half_sizes();
    let meshes = Mesh3D::descriptor_vertex_positions();
    assert!(!store.below("/robots/slot0/bot", shapes.clone()).is_empty(), "a visible model under a hidden child");
    assert!(!store.below("/robots/slot0/bot", meshes.clone()).is_empty());
    assert!(!store.has("/robots/slot0/pedestal", cylinder.clone()), "the hidden child");
    assert!(store.below("/robots/slot1", shapes).is_empty(), "the hidden model");
    assert!(store.below("/robots/slot1", meshes).is_empty());
    assert!(store.has("/robots/slot1/pedestal", cylinder.clone()), "a visible child of a hidden model");
    // A hidden model's frames are still there for its children.
    assert_at(store.world_from("robots/slot1#arm"), [1.0, 0.0, 0.0], 0.5, "hidden model's link");

    // Showing the slot again draws it; its pedestal is left alone.
    let mut slot1 = robot(false);
    slot1.id = "robots/slot1".to_owned();
    let delta = SceneDelta { scene_id: "s1".into(), seq: 2, upsert: vec![slot1], remove: vec![], view: None };
    scene.apply(vec![SceneEvent::Delta(delta)], &assets);
    store.take(&mut scene);
    assert!(!store.below("/robots/slot1/bot", Boxes3D::descriptor_half_sizes()).is_empty());
    assert!(store.has("/robots/slot1/pedestal", cylinder));
}

/// A part whose links hang off floating and planar joints (product URDFs, fixtures' `world`).
const PART_URDF: &str = r#"<robot name="part">
  <link name="world"/>
  <link name="body"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="slider"><visual><geometry><sphere radius="0.1"/></geometry></visual></link>
  <link name="ball"><visual><geometry><sphere radius="0.1"/></geometry></visual></link>
  <joint name="float" type="floating"><parent link="world"/><child link="body"/><origin xyz="0 0 1"/></joint>
  <joint name="plane" type="planar"><parent link="body"/><child link="slider"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="sphere" type="spherical"><parent link="body"/><child link="ball"/><origin xyz="0 1 0"/></joint>
</robot>"#;

/// (c) Floating, planar and spherical joints sit at their origin; a value for one is ignored
/// (a scalar can't pose them).
#[test]
fn multi_dof_joints_sit_at_their_origin() {
    let mut scene = new_scene();
    let mut assets = Assets::default();
    assets.add("part", PART_URDF);
    let part = Node::new(
        "part",
        NodeKind::Model { model: ModelRef { urdf: asset("part", "application/xml"), meshes: BTreeMap::new() }, joints: BTreeMap::new(), show_collision: false },
    )
    .at(Pose3::from_xyz([0.0, 2.0, 0.0]));
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![part]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert_at(store.world_from("part#body"), [0.0, 2.0, 1.0], 0.0, "floating");
    assert_at(store.world_from("part#slider"), [1.0, 2.0, 1.0], 0.0, "planar");
    assert_at(store.world_from("part#ball"), [0.0, 3.0, 1.0], 0.0, "spherical");

    let joints = BTreeMap::from([("float".to_owned(), 0.7), ("plane".to_owned(), 0.3), ("sphere".to_owned(), 0.2)]);
    scene.apply(
        vec![SceneEvent::Joints(JointFrame { scene_id: "s1".into(), t: None, models: vec![ModelJoints { node: "part".into(), joints }] })],
        &assets,
    );
    assert!(take(&mut scene).0.is_empty(), "values of multi-DoF joints log nothing");
}

/// (d) A node on a link of a model whose URDF has not arrived is nowhere until it does, then on
/// the link.
#[test]
fn a_child_on_a_link_appears_once_the_model_loads() {
    let mut scene = new_scene();
    let mut assets = Assets::default();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes()]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert!(store.has("/robots/r1/tcp", Boxes3D::descriptor_half_sizes()), "drawn at once");
    assert_eq!(store.world("/robots/r1/tcp"), None, "but in a frame not yet connected");

    assets.add("urdf", URDF);
    scene.apply(Vec::new(), &assets);
    store.take(&mut scene);
    assert_at(store.world("/robots/r1/tcp"), [1.0, 0.0, 0.0], 0.5, "on the arm");
}

/// (e) Removing a node clears it and its id-descendants (the dresser's `robot/tcp` takes
/// `robot/tcp/approach` with it), nothing else.
#[test]
fn removing_a_node_clears_its_id_descendants() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let tcp = Node::new("robots/r1/tcp", NodeKind::Points { points: vec![[0.0; 3]], colors: vec![], labels: vec!["TCP".into()], radius: None, show_labels: Some(true) })
        .under("robots/r1", Some("arm".to_owned()));
    let approach = Node::new(
        "robots/r1/tcp/approach",
        NodeKind::Arrows { origins: vec![[0.0; 3]], vectors: vec![[0.0, 0.0, 0.1]], colors: vec![], labels: vec![], radius: None, show_labels: None },
    )
    .under("robots/r1", Some("arm".to_owned()));
    let other = Node::new("robots/r1/tcp_old", NodeKind::Axes { length: 0.1, radius: None }).under("robots/r1", None);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), tcp, approach, other]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert!(store.has("/robots/r1/tcp/approach", Arrows3D::descriptor_vectors()));

    let delta = SceneDelta { scene_id: "s1".into(), seq: 2, upsert: vec![], remove: vec!["robots/r1/tcp".into()], view: None };
    scene.apply(vec![SceneEvent::Delta(delta)], &assets);
    store.take(&mut scene);
    assert!(!store.has("/robots/r1/tcp", Points3D::descriptor_positions()));
    assert!(!store.has("/robots/r1/tcp/approach", Arrows3D::descriptor_vectors()), "an id-descendant");
    assert!(store.has("/robots/r1/tcp_old", Arrows3D::descriptor_vectors()), "a sibling with a common prefix");
    assert!(!store.below("/robots/r1/bot", Boxes3D::descriptor_half_sizes()).is_empty(), "the parent");
}

/// The view's "Labels" toggle: off, labelled nodes are drawn again without their labels (also
/// those shown on hover only), on with them; the geometry, its frame and other nodes stay. A
/// scene that arrives while it is off has none.
#[test]
fn the_labels_toggle_redraws_labelled_nodes_without_their_labels() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let name = Node::new(
        "robots/r1/name_label",
        NodeKind::Points { points: vec![[0.0, 0.0, 1.0]], colors: vec![], labels: vec!["r1".into()], radius: None, show_labels: Some(true) },
    )
    .under("robots/r1", None);
    let welds = Node::new(
        "task/welds",
        NodeKind::Boxes { centers: vec![[0.0; 3]], half_sizes: vec![[0.1; 3]], quats_xyzw: vec![], colors: vec![], labels: vec!["W01".into()], solid: false, show_labels: Some(false) },
    );
    let floor = Node::new("floor", NodeKind::Lines { strips: vec![vec![[0.0; 3], [1.0, 0.0, 0.0]]], colors: vec![], radius: None, labels: vec![], show_labels: None });
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), name, welds, floor]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert!(store.has("/robots/r1/name_label", Points3D::descriptor_labels()));
    assert!(store.has("/task/welds", Boxes3D::descriptor_labels()));

    scene.set_labels(false);
    let (chunks, _) = take(&mut scene);
    let entities: std::collections::BTreeSet<String> = chunks.iter().map(|c| c.entity_path().to_string()).collect();
    assert_eq!(entities, ["/robots/r1/name_label", "/task/welds"].map(str::to_owned).into(), "only the labelled nodes");
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert!(!store.has("/robots/r1/name_label", Points3D::descriptor_labels()));
    assert!(!store.has("/robots/r1/name_label", Points3D::descriptor_show_labels()));
    assert!(store.has("/robots/r1/name_label", Points3D::descriptor_positions()), "the point stays");
    assert!(!store.has("/task/welds", Boxes3D::descriptor_labels()), "hover-only labels go too");
    assert!(store.has("/task/welds", Boxes3D::descriptor_half_sizes()));
    assert_at(store.world("/robots/r1/name_label"), [1.0, 0.0, 0.0], 0.0, "still on the robot");
    assert!(store.has("/floor", crate::rr::re_sdk_types::archetypes::LineStrips3D::descriptor_strips()));

    scene.set_labels(false);
    assert!(take(&mut scene).0.is_empty(), "nothing to do");

    // A new scene while off: drawn without labels.
    let points = |label: &str| NodeKind::Points { points: vec![[0.0; 3]], colors: vec![], labels: vec![label.into()], radius: None, show_labels: None };
    scene.apply(vec![SceneEvent::Reset(doc("s2", vec![Node::new("spots", points("A"))]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert!(store.has("/spots", Points3D::descriptor_positions()) && !store.has("/spots", Points3D::descriptor_labels()));

    scene.set_labels(true);
    store.take(&mut scene);
    assert!(store.has("/spots", Points3D::descriptor_labels()));
}

/// The fields that restore the RRD-era visuals reach Rerun.
#[test]
fn labels_radii_and_fill_modes() {
    use crate::rr::re_sdk_types::archetypes::LineStrips3D;
    use crate::rr::re_sdk_types::components::FillMode;
    let batches = |kind: &NodeKind| entities::primitive(kind, true).unwrap().as_serialized_batches();
    let lines = NodeKind::Lines {
        strips: vec![vec![[0.0; 3], [1.0, 0.0, 0.0]]],
        colors: vec![],
        radius: None,
        labels: vec!["seam".into()],
        show_labels: Some(false),
    };
    let lines = batches(&lines);
    assert!(lines.iter().any(|b| b.descriptor == LineStrips3D::descriptor_labels()));
    assert!(lines.iter().any(|b| b.descriptor == LineStrips3D::descriptor_show_labels()));

    let axes = batches(&NodeKind::Axes { length: 0.1, radius: Some(0.01) });
    let radii = axes.iter().find(|b| b.descriptor == Arrows3D::descriptor_radii()).unwrap();
    let radii = radii.array.as_any().downcast_ref::<crate::rr::re_sdk_types::external::arrow::array::Float32Array>().unwrap();
    assert_eq!(radii.values().as_ref(), [0.01]);

    let fill = |solid| {
        use crate::rr::re_sdk_types::FromArrow as _;
        let kind = NodeKind::Cylinder { height: 1.0, radius: 0.1, center: [0.0; 3], color: None, solid };
        let batches = batches(&kind);
        let array = &batches.iter().find(|b| b.descriptor == Cylinders3D::descriptor_fill_mode()).unwrap().array;
        FillMode::from_arrow(array).unwrap()
    };
    assert_eq!(fill(true), [FillMode::Solid]);
    assert_eq!(fill(false), [FillMode::MajorWireframe]);
}

fn upsert(seq: u64, nodes: Vec<Node>) -> SceneEvent {
    SceneEvent::Delta(SceneDelta { scene_id: "s1".into(), seq, upsert: nodes, remove: vec![], view: None })
}

fn remove(seq: u64, ids: &[&str]) -> SceneEvent {
    SceneEvent::Delta(SceneDelta { scene_id: "s1".into(), seq, upsert: vec![], remove: ids.iter().map(|id| (*id).to_owned()).collect(), view: None })
}

fn joint_values(node: &mut Node, values: &[(&str, f64)]) {
    let NodeKind::Model { joints, .. } = &mut node.kind else { panic!() };
    *joints = values.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect();
}

/// The Robo Fitter moves a model, the Dresser flips `show_collision`: the URDF is not loaded again.
#[test]
fn a_model_upsert_logs_only_what_changed() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes()]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    let urdf_requests = assets.requests("urdf");

    let moved = robot(false).at(Pose3::from_xyz([2.0, 0.0, 0.0]));
    scene.apply(vec![upsert(2, vec![moved])], &assets);
    let (chunks, others) = take(&mut scene);
    assert_eq!(chunks.iter().map(|c| c.entity_path().to_string()).collect::<Vec<_>>(), ["/robots/r1"], "only the placement");
    assert!(has(&chunks, "/robots/r1", Transform3D::name().as_str()));
    assert!(others.is_empty(), "the blueprint did not change");
    assert_eq!(assets.requests("urdf"), urdf_requests, "the URDF is not asked for again");
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert_at(store.world("/robots/r1/tcp"), [2.0, 0.0, 0.0], 0.5, "the TCP follows");

    // Collision shown, a joint moved (j1 left out: back to 0).
    let mut shown = robot(true).at(Pose3::from_xyz([2.0, 0.0, 0.0]));
    joint_values(&mut shown, &[]);
    scene.apply(vec![upsert(3, vec![shown])], &assets);
    let (chunks, others) = take(&mut scene);
    let mut entities: Vec<_> = chunks.iter().map(|c| c.entity_path().to_string()).collect();
    entities.sort();
    entities.dedup();
    assert_eq!(entities.len(), 3, "{entities:?}");
    assert_eq!([entities[0].as_str(), entities[2].as_str()], ["/robots/r1", "/robots/r1/joints/j1"]);
    assert!(entities[1].starts_with("/robots/r1/bot/collision_geometries/"), "logged the first time it is shown: {entities:?}");
    assert!(others.is_empty(), "the blueprint did not change");
    assert!(entities_with(&chunks, Clear::name().as_str()).is_empty());
}

/// A fetch that fails for now (a restarted backend's asset index) is retried; one that can't
/// succeed is not.
#[test]
fn failed_fetches_are_retried_until_they_succeed() {
    let mut scene = new_scene();
    scene.retry_after = Duration::ZERO;
    let mut assets = Assets::default();
    assets.fail("urdf", true);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    assert!(scene.next_retry().is_some());
    assert!(entities_with(&take(&mut scene).0, Boxes3D::name().as_str()).is_empty());

    // The URDF is there now; the mesh's `.mtl` fails for now.
    assets.add("urdf", URDF);
    assets.add("obj", OBJ);
    assets.fail("mtl", true);
    scene.apply(Vec::new(), &assets);
    let (chunks, _) = take(&mut scene);
    assert!(!entities_with(&chunks, Boxes3D::name().as_str()).is_empty(), "the URDF loaded on a retry");
    assert!(entities_with(&chunks, Mesh3D::name().as_str()).is_empty(), "not drawn white");
    assert_eq!(scene.pending.len(), 1);

    assets.add("mtl", MTL);
    scene.apply(Vec::new(), &assets);
    let (chunks, _) = take(&mut scene);
    let meshes = entities_with(&chunks, Mesh3D::name().as_str());
    assert_eq!(meshes.len(), 1);
    let mesh = chunks.iter().find(|c| c.entity_path().to_string() == meshes[0]).unwrap();
    assert!(mesh.components().contains_component(Mesh3D::descriptor_vertex_colors().component), "with the MTL's colours");
    assert!(scene.pending.is_empty() && scene.next_retry().is_none());

    // Not before the retry time.
    let mut scene = new_scene();
    let mut assets = Assets::default();
    assets.fail("urdf", true);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    let requests = assets.requests("urdf");
    assets.add("urdf", URDF);
    scene.apply(Vec::new(), &assets);
    assert_eq!(assets.requests("urdf"), requests);

    // For good: not asked again.
    let mut scene = new_scene();
    scene.retry_after = Duration::ZERO;
    let mut assets = Assets::default();
    assets.fail("urdf", false);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    let requests = assets.requests("urdf");
    scene.apply(Vec::new(), &assets);
    assert_eq!(assets.requests("urdf"), requests);
    assert!(scene.next_retry().is_none());
}

/// The Dresser's slider: echoes of earlier values don't move the joint back while dragging.
#[test]
fn local_joint_values_hold_against_lagging_echoes() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    take(&mut scene);
    let echo = |value: f64| {
        SceneEvent::Joints(JointFrame {
            scene_id: "s1".into(),
            t: None,
            models: vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("j1".into(), value)]) }],
        })
    };
    let shown = |scene: &RerunScene| {
        let Some(Node { kind: NodeKind::Model { joints, .. }, .. }) = scene.doc.as_ref().unwrap().node("robots/r1") else { panic!() };
        joints["j1"]
    };

    for value in [0.6, 0.7, 0.8] {
        scene.set_local_joints("robots/r1", &BTreeMap::from([("j1".into(), value)]), &assets);
    }
    take(&mut scene);
    scene.apply(vec![echo(0.6), echo(0.7)], &assets);
    assert!(take(&mut scene).0.is_empty(), "lagging echoes are ignored");
    assert_eq!(shown(&scene), 0.8);
    // An upsert that lags too (the Dresser flips `show_collision`) keeps the local value.
    let mut flipped = robot(true);
    joint_values(&mut flipped, &[("j1", 0.6)]);
    scene.apply(vec![upsert(2, vec![flipped])], &assets);
    assert_eq!(shown(&scene), 0.8);
    take(&mut scene);
    // The echo of the last value: the producer's again.
    scene.apply(vec![echo(0.8), echo(0.3)], &assets);
    assert_eq!(shown(&scene), 0.3);
    assert!(scene.local.is_empty());

    // An echo that never matches (the producer clamps) is taken after LOCAL_HOLD.
    scene.set_local_joints("robots/r1", &BTreeMap::from([("j1".into(), 5.0)]), &assets);
    scene.apply(vec![echo(1.0)], &assets);
    assert_eq!(shown(&scene), 5.0);
    for (_, at) in scene.local.get_mut("robots/r1").unwrap().values_mut() {
        *at = Instant::now().checked_sub(LOCAL_HOLD).unwrap();
    }
    scene.apply(vec![echo(1.0)], &assets);
    assert_eq!(shown(&scene), 1.0);
}

/// Scenes replaced within one drain: every replaced recording is closed, once Rerun has it all.
#[test]
fn every_replaced_recording_is_closed() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let receiver = scene.take_receiver().unwrap();
    scene.apply(vec![SceneEvent::Reset(doc("a", vec![boxes()]))], &assets);
    let a = scene.current().unwrap();
    scene.apply(vec![SceneEvent::Reset(doc("b", vec![boxes()])), SceneEvent::Reset(doc("c", vec![boxes()]))], &assets);
    let c = scene.current().unwrap();
    let route = scene.take_route_request().unwrap();
    assert_eq!((route.close, &route.open), (None, &c));

    // Handed over, not yet ingested: nothing closed (B's messages would open it again).
    assert!(scene.forward(), "work waits");
    assert!(scene.take_route_request().is_none());
    assert!(scene.forward());
    assert!(scene.take_route_request().is_none());
    while receiver.try_recv().is_ok() {}
    scene.forward();
    let first = scene.take_route_request().unwrap();
    let second = scene.take_route_request().unwrap();
    assert!(scene.take_route_request().is_none());
    assert_eq!((&first.open, &second.open), (&c, &c));
    let closed = [first.close.unwrap(), second.close.unwrap()];
    assert_eq!(closed[0], a);
    assert!(!closed.contains(&c) && closed[0] != closed[1]);
    assert!(!scene.forward());
}

/// Another URDF with the same link names; Rerun keeps static transforms of cleared entities.
const URDF2: &str = r#"<robot name="bot2">
  <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="arm"/>
  <joint name="j2" type="fixed">
    <parent link="base"/><child link="arm"/><origin xyz="0 0 2"/>
  </joint>
</robot>"#;

/// A model replaced by another URDF: its link frames are new (no two entities defining one child
/// frame), and children by `parent` follow the new instance; a removed parent takes them along.
#[test]
fn a_replaced_model_gets_new_frames_its_children_follow() {
    let mut scene = new_scene();
    let mut assets = Assets::all();
    assets.add("urdf2", URDF2);
    let base = pedestal("r1", [0.0; 3]).under("robots/r1", None);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes(), base]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert_at(store.world("/robots/r1/tcp"), [1.0, 0.0, 0.0], 0.5, "on the first model's arm");

    let NodeKind::Model { model, .. } = &robot(false).kind else { panic!() };
    let mut other = robot(false);
    other.kind = NodeKind::Model {
        model: ModelRef { urdf: asset("urdf2", "application/xml"), ..model.clone() },
        joints: BTreeMap::new(),
        show_collision: false,
    };
    scene.apply(vec![upsert(2, vec![other])], &assets);
    store.take(&mut scene);
    assert_eq!(store.frame("/robots/r1/bot2").as_deref(), Some("robots/r1@1#base"), "new frames");
    assert_at(store.world("/robots/r1/tcp"), [1.0, 0.0, 2.0], 0.0, "on the new model's arm");
    assert_at(store.world("/robots/r1/pedestal"), [1.0, 0.0, 0.0], 0.0, "on the new model");
    // The same robot again: the old frames are not taken up again.
    scene.apply(vec![upsert(3, vec![robot(false)])], &assets);
    store.take(&mut scene);
    assert_at(store.world("/robots/r1/tcp"), [1.0, 0.0, 0.0], 0.5, "back on the first model's arm");

    // The model removed (not its children by `parent`): they hang nowhere, not at the stale pose.
    scene.apply(vec![remove(4, &["robots/r1"])], &assets);
    let mut tcp = boxes();
    tcp.id = "tcp".into();
    let mut kept = pedestal("r1", [0.0; 3]).under("robots/r1", None);
    kept.id = "pedestal".into();
    scene.apply(vec![upsert(5, vec![tcp, kept])], &assets);
    store.take(&mut scene);
    assert_eq!(store.world("/tcp"), None);
    assert_eq!(store.world("/pedestal"), None);
    // And follow it when it is back.
    scene.apply(vec![upsert(6, vec![robot(false).at(Pose3::from_xyz([3.0, 0.0, 0.0]))])], &assets);
    store.take(&mut scene);
    assert_at(store.world("/tcp"), [3.0, 0.0, 0.0], 0.5, "on the new instance's arm");
    assert_at(store.world("/pedestal"), [3.0, 0.0, 0.0], 0.0, "on the new instance");
}

/// A pose frame moves nodes (placements only) and is folded into the scene.
#[test]
fn pose_frames_move_nodes_only() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), boxes()]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    let urdf_requests = assets.requests("urdf");

    let frame = |scene_id: &str, poses: Vec<(&str, [f64; 3])>| {
        SceneEvent::Poses(PoseFrame {
            scene_id: scene_id.into(),
            t: None,
            poses: poses.into_iter().map(|(node, xyz)| NodePose { node: node.into(), pose: Pose3::from_xyz(xyz) }).collect(),
        })
    };
    scene.apply(vec![frame("s1", vec![("robots/r1", [2.0, 0.0, 0.0]), ("nowhere", [1.0; 3]), ("robots/r1/tcp", [0.0; 3])])], &assets);
    let (chunks, others) = take(&mut scene);
    assert_eq!(chunks.iter().map(|c| c.entity_path().to_string()).collect::<Vec<_>>(), ["/robots/r1"], "only what moved, its placement");
    assert!(has(&chunks, "/robots/r1", Transform3D::name().as_str()));
    assert!(others.is_empty());
    assert_eq!(assets.requests("urdf"), urdf_requests);
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert_at(store.world("/robots/r1/tcp"), [2.0, 0.0, 0.0], 0.5, "the TCP follows");
    assert_eq!(scene.doc.as_ref().unwrap().node("robots/r1").unwrap().pose.xyz, [2.0, 0.0, 0.0], "folded into the scene");

    scene.apply(vec![frame("s1", vec![("robots/r1", [2.0, 0.0, 0.0])]), frame("other", vec![("robots/r1", [5.0; 3])])], &assets);
    assert!(take(&mut scene).0.is_empty(), "unchanged, and another scene's");

    // A later upsert of the same model keeps the folded pose as its starting point.
    scene.apply(vec![upsert(2, vec![robot(false).at(Pose3::from_xyz([2.0, 0.0, 0.0]))])], &assets);
    let (chunks, _) = take(&mut scene);
    assert!(entities_with(&chunks, Clear::name().as_str()).is_empty());
}

fn glb_asset(name: &str) -> AssetRef {
    asset(name, "model/gltf-binary")
}

fn mesh_node(mesh: MeshAsset) -> Node {
    Node::new("part", NodeKind::Mesh { mesh, scale: [1.0; 3], color: None })
}

/// The bytes of `entity`'s `Asset3D`, if it has one.
fn asset_blob(store: &Store, entity: &str) -> Option<Vec<u8>> {
    use crate::rr::re_sdk_types::components::Blob;
    let component = Asset3D::descriptor_blob().component;
    let results = store.0.latest_at(&Store::query(), &EntityPath::from(entity), [component]);
    results.component_mono::<Blob>(component).map(|blob| blob.0.0.to_vec())
}

/// The decimated mesh first, then the full one on the same entity (and in the same frame).
#[test]
fn a_decimated_mesh_first_then_the_full_one() {
    let mut scene = new_scene();
    let mut assets = Assets::default();
    assets.add("lod", "glb-lod");
    let mesh = MeshAsset { glb: Some(glb_asset("glb")), lod: Some(glb_asset("lod")), ..arm_mesh() };
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![mesh_node(mesh.clone())]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert_eq!(asset_blob(&store, "/part").as_deref(), Some(&b"glb-lod"[..]));
    assert_eq!(store.frame("/part").as_deref(), Some("part"));
    assert_eq!(scene.pending.len(), 1, "the full mesh is still to come");
    // Asked for: the decimated one first and at high priority, then the GLB (not the OBJ).
    let asked = assets.3.borrow().clone();
    assert_eq!(asked[0], ("lod".to_owned(), Priority::High));
    assert!(asked.contains(&("glb".to_owned(), Priority::Normal)));
    assert_eq!(assets.requests("obj") + assets.requests("mtl"), 0);

    assets.add("glb", "glb-full");
    scene.apply(Vec::new(), &assets);
    store.take(&mut scene);
    assert_eq!(asset_blob(&store, "/part").as_deref(), Some(&b"glb-full"[..]));
    assert_eq!(store.frame("/part").as_deref(), Some("part"));
    assert!(scene.pending.is_empty());

    // Without a GLB, the OBJ (a Mesh3D) replaces the decimated Asset3D.
    let mut scene = new_scene();
    let mut assets = Assets::all();
    assets.0.remove("obj");
    assets.add("lod", "glb-lod");
    let mesh = MeshAsset { lod: Some(glb_asset("lod")), ..arm_mesh() };
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![mesh_node(mesh)]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    assert!(asset_blob(&store, "/part").is_some());
    assets.add("obj", OBJ);
    scene.apply(Vec::new(), &assets);
    store.take(&mut scene);
    assert_eq!(asset_blob(&store, "/part"), None, "the decimated mesh is gone");
    assert!(store.has("/part", Mesh3D::descriptor_vertex_colors()));

    // The full mesh there first: the decimated one is never drawn.
    let mut scene = new_scene();
    let mut assets = Assets::default();
    assets.add("glb", "glb-full");
    let mesh = MeshAsset { glb: Some(glb_asset("glb")), lod: Some(glb_asset("lod")), ..arm_mesh() };
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![mesh_node(mesh)]))], &assets);
    let (chunks, _) = take(&mut scene);
    assert_eq!(chunks.iter().filter(|c| c.entity_path().to_string() == "/part").count(), 2, "its placement and the full mesh");
    assert!(scene.pending.is_empty());
    assets.add("lod", "glb-lod");
    scene.apply(Vec::new(), &assets);
    assert!(take(&mut scene).0.is_empty());
}

/// A model's mesh with a GLB variant is drawn from it, not from the OBJ; without one, from the OBJ.
#[test]
fn a_glb_is_preferred_over_obj_and_mtl() {
    let mut scene = new_scene();
    let mut assets = Assets::all();
    assets.add("glb", "glb-full");
    let mut node = robot(false);
    let NodeKind::Model { model, .. } = &mut node.kind else { panic!() };
    model.meshes.get_mut("package://bot/arm.obj").unwrap().glb = Some(glb_asset("glb"));
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![node]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    let assets3d = store.below("/robots/r1/bot/visual_geometries/arm", Asset3D::descriptor_blob());
    assert_eq!(assets3d.len(), 1, "{assets3d:?}");
    assert_eq!(asset_blob(&store, &assets3d[0]).as_deref(), Some(&b"glb-full"[..]));
    let media = Asset3D::descriptor_media_type().component;
    let results = store.0.latest_at(&Store::query(), &EntityPath::from(assets3d[0].as_str()), [media]);
    let media = results.component_mono::<crate::rr::re_sdk_types::components::MediaType>(media).unwrap();
    assert_eq!(media.as_str(), "model/gltf-binary");
    assert!(store.below("/robots/r1", Mesh3D::descriptor_vertex_positions()).is_empty());
    assert_eq!(assets.requests("obj") + assets.requests("mtl"), 0, "the OBJ and its MTL are not fetched");

    // No variants: the OBJ with the MTL's colours, as before.
    let mut scene = new_scene();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &Assets::all());
    let mut store = Store::new();
    store.take(&mut scene);
    assert_eq!(store.below("/robots/r1", Mesh3D::descriptor_vertex_colors()).len(), 1);
    assert!(store.below("/robots/r1", Asset3D::descriptor_blob()).is_empty());
}

/// A URDF whose collision geometry is a mesh of its own.
const COLLISION_URDF: &str = r#"<robot name="bot">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><mesh filename="package://bot/base_collision.stl"/></geometry></collision>
  </link>
</robot>"#;

fn collision_assets() -> Assets {
    let mut assets = Assets::default();
    assets.add("curdf", COLLISION_URDF);
    assets.add("stl", "solid x\nendsolid x\n");
    assets
}

/// A model of [`COLLISION_URDF`].
fn collision_model(show_collision: bool) -> Node {
    Node::new(
        "robots/r1",
        NodeKind::Model {
            model: ModelRef {
                urdf: asset("curdf", "application/xml"),
                meshes: BTreeMap::from([(
                    "package://bot/base_collision.stl".to_owned(),
                    MeshAsset { asset: asset("stl", "model/stl"), ..MeshAsset::default() },
                )]),
            },
            joints: BTreeMap::new(),
            show_collision,
        },
    )
}

/// What Rerun's 3D view shows of a scene's collision geometry, modelled as Rerun does it: the
/// recording as stored (clears applied), through the view query of the blueprint's clone made at
/// the last activation (`StoreHub::set_cloned_blueprint_active_for_app`): blueprint writes after
/// that do not reach the view.
struct RerunView {
    recordings: HashMap<StoreId, Store>,
    /// The blueprint's view query as logged so far, and as cloned at the last activation.
    logged: Option<Vec<String>>,
    active: Option<Vec<String>>,
}

impl RerunView {
    fn new() -> Self {
        Self { recordings: HashMap::new(), logged: None, active: None }
    }

    fn ingest(&mut self, scene: &mut RerunScene) {
        for msg in scene.queue.take_all() {
            match &msg {
                LogMsg::ArrowMsg(store, arrow_msg) if store.kind() == StoreKind::Recording => {
                    let chunk = Chunk::from_arrow_msg(arrow_msg).unwrap();
                    self.recordings.entry(store.clone()).or_insert_with(Store::new).0.add_chunk(&Arc::new(chunk)).unwrap();
                }
                LogMsg::ArrowMsg(..) => {
                    if let Some(query) = view_query(std::slice::from_ref(&msg)) {
                        self.logged = Some(query);
                    }
                }
                LogMsg::BlueprintActivationCommand(cmd) if cmd.make_active => self.active = self.logged.clone(),
                _ => {}
            }
        }
    }

    /// The collision blobs the view draws of recording `store`.
    fn collisions(&self, store: &StoreId) -> Vec<String> {
        let excluded = |entity: &str| {
            self.active.iter().flatten().any(|q| q.strip_prefix("- ").and_then(|p| p.strip_suffix("/**")).is_some_and(|p| entity.starts_with(p)))
        };
        let Some(recording) = self.recordings.get(store) else { return Vec::new() };
        recording
            .below("/robots/r1/bot/collision_geometries", Asset3D::descriptor_blob())
            .into_iter()
            .filter(|entity| !excluded(entity))
            .collect()
    }
}

/// Collision geometry shown, then turned off, then a new scene of the same application shown
/// (its blueprint activated again, as for a restarted backend's scene): the hulls are gone each
/// time, and come back only where they are turned on.
#[test]
fn collision_turned_off_stays_off_through_a_new_scene() {
    let mut scene = new_scene();
    let assets = collision_assets();
    let mut view = RerunView::new();
    let doc_of = |id: &str, show: bool| doc(id, vec![collision_model(show)]);

    scene.apply(vec![SceneEvent::Reset(doc_of("s1", false))], &assets);
    scene.set_shown(true);
    view.ingest(&mut scene);
    let s1 = scene.current().unwrap();
    assert!(view.active.is_some(), "activated");
    assert!(view.collisions(&s1).is_empty());

    scene.apply(vec![upsert(2, vec![collision_model(true)])], &assets);
    view.ingest(&mut scene);
    assert_eq!(view.collisions(&s1).len(), 1, "turned on: shown");

    scene.apply(vec![upsert(3, vec![collision_model(false)])], &assets);
    view.ingest(&mut scene);
    assert!(view.collisions(&s1).is_empty(), "turned off: gone");

    // A new scene, its blueprint activated again once shown.
    scene.apply(vec![SceneEvent::Reset(doc_of("s2", false))], &assets);
    scene.set_shown(true);
    view.ingest(&mut scene);
    let s2 = scene.current().unwrap();
    assert_ne!(s2, s1);
    assert!(view.collisions(&s2).is_empty(), "the new scene: none");
    assert_eq!(view.active.as_deref(), Some([blueprint::VIEW_QUERY.to_owned()].as_slice()));

    // One that starts with them on, then off.
    scene.apply(vec![SceneEvent::Reset(doc_of("s3", true))], &assets);
    scene.set_shown(true);
    view.ingest(&mut scene);
    let s3 = scene.current().unwrap();
    assert_eq!(view.collisions(&s3).len(), 1);
    scene.apply(vec![SceneEvent::Delta(SceneDelta { scene_id: "s3".into(), seq: 2, upsert: vec![collision_model(false)], remove: vec![], view: None })], &assets);
    view.ingest(&mut scene);
    assert!(view.collisions(&s3).is_empty());
}

/// Collision geometry is fetched and logged only once shown; turned off it is cleared from the
/// recording (the view's query never changes: Rerun shows the blueprint's clone made when it was
/// activated), and logged again from the cached bytes when shown again.
#[test]
fn collision_geometry_is_loaded_when_first_shown() {
    let mut scene = new_scene();
    let assets = collision_assets();
    let model = collision_model;
    let collisions = |store: &Store| store.below("/robots/r1/bot/collision_geometries", Asset3D::descriptor_blob());
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![model(false)]))], &assets);
    let mut store = Store::new();
    let (chunks, others) = take(&mut scene);
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert_eq!(view_query(&others).unwrap(), [blueprint::VIEW_QUERY]);
    assert_eq!(assets.requests("stl"), 0, "not fetched");
    assert!(collisions(&store).is_empty());
    assert!(!store.below("/robots/r1/bot/visual_geometries", Boxes3D::descriptor_half_sizes()).is_empty());

    // Shown: fetched, logged; the blueprint untouched.
    scene.apply(vec![upsert(2, vec![model(true)])], &assets);
    let (chunks, others) = take(&mut scene);
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert!(others.is_empty(), "the blueprint did not change");
    assert_eq!(assets.requests("stl"), 1);
    assert_eq!(collisions(&store).len(), 1, "{:?}", collisions(&store));
    assert_eq!(store.frame(&collisions(&store)[0]).as_deref(), Some("robots/r1#base"));

    // Off: cleared from the recording, the visual geometry stays.
    scene.apply(vec![upsert(3, vec![model(false)])], &assets);
    let (chunks, others) = take(&mut scene);
    assert_eq!(entities_with(&chunks, Clear::name().as_str()), ["/robots/r1/bot/collision_geometries"]);
    assert!(others.is_empty(), "the blueprint did not change");
    for chunk in chunks {
        store.0.add_chunk(&Arc::new(chunk)).unwrap();
    }
    assert!(collisions(&store).is_empty(), "not shown any more");
    assert!(!store.below("/robots/r1/bot/visual_geometries", Boxes3D::descriptor_half_sizes()).is_empty());
    // Off again: nothing to clear.
    scene.apply(vec![upsert(4, vec![model(false)])], &assets);
    assert!(entities_with(&take(&mut scene).0, Clear::name().as_str()).is_empty());

    // On again: logged again (the bytes are cached, not fetched again).
    scene.apply(vec![upsert(5, vec![model(true)])], &assets);
    store.take(&mut scene);
    assert_eq!(collisions(&store).len(), 1, "shown again");
    assert!(assets.requests("stl") >= 2, "asked the cache again");

    // A scene that starts with it shown logs it at once.
    let mut scene = new_scene();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![model(true)]))], &assets);
    assert_eq!(entities_with(&take(&mut scene).0, Asset3D::name().as_str()).len(), 1);
}

/// Natively, OBJs are converted on worker threads: the mesh waits (pending) until the result is
/// back, then is drawn as when converted inline.
#[cfg(not(target_arch = "wasm32"))]
#[test]
fn obj_meshes_are_converted_off_the_calling_thread() {
    let mut scene = RerunScene::new("factoryos_test");
    assert!(!scene.convert.is_inline());
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);
    let start = Instant::now();
    while !scene.pending.is_empty() {
        assert!(start.elapsed() < Duration::from_secs(10), "timed out");
        std::thread::sleep(Duration::from_millis(2));
        scene.apply(Vec::new(), &assets);
        store.take(&mut scene);
    }
    let meshes = store.below("/robots/r1/bot/visual_geometries", Mesh3D::descriptor_vertex_colors());
    assert_eq!(meshes.len(), 1, "{meshes:?}");
    assert_eq!(store.frame(&meshes[0]).as_deref(), Some("robots/r1#arm"));

    // Removed while converting: the result is dropped with it.
    let mut scene = RerunScene::new("factoryos_test");
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    scene.apply(vec![remove(2, &["robots/r1"])], &assets);
    assert!(scene.pending.is_empty());
}

/// A new scene asks for every URDF and decimated mesh before any full mesh; a URDF's own meshes
/// (only known once it is parsed) decimated first too.
#[test]
fn a_new_scene_asks_for_urdfs_and_decimated_meshes_first() {
    let mut scene = new_scene();
    let assets = Assets::default();
    let mut r2 = robot(false);
    r2.id = "robots/r2".into();
    let NodeKind::Model { model, .. } = &mut r2.kind else { panic!() };
    model.urdf = asset("urdf2", "application/xml");
    let part = mesh_node(MeshAsset { glb: Some(glb_asset("glb")), lod: Some(glb_asset("lod")), ..arm_mesh() });
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![part, robot(false), r2]))], &assets);
    let asked = assets.3.borrow().clone();
    let first_normal = asked.iter().position(|(_, p)| *p == Priority::Normal).unwrap();
    let high: BTreeSet<&str> = asked[..first_normal].iter().map(|(sha, _)| sha.as_str()).collect();
    assert_eq!(high, BTreeSet::from(["urdf", "urdf2", "lod"]), "{asked:?}");
    assert!(asked.iter().all(|(sha, p)| (*p == Priority::High) == ["urdf", "urdf2", "lod"].contains(&sha.as_str())), "{asked:?}");

    // The URDF arrives: its mesh's decimated variant before its full one.
    let mut assets = Assets::default();
    assets.add("urdf", URDF);
    let mut node = robot(false);
    let NodeKind::Model { model, .. } = &mut node.kind else { panic!() };
    let arm = model.meshes.get_mut("package://bot/arm.obj").unwrap();
    arm.lod = Some(glb_asset("arm-lod"));
    let mut scene = new_scene();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![node]))], &assets);
    let asked = assets.3.borrow().clone();
    let lod = asked.iter().position(|a| *a == ("arm-lod".to_owned(), Priority::High)).expect("asked for");
    let obj = asked.iter().position(|(sha, _)| sha == "obj").expect("asked for");
    assert!(lod < obj, "{asked:?}");
}

/// A colour override (a URDF `<color>`, a Mesh node's `color`) replaces the mesh's colours: the
/// GLB variants are skipped, the OBJ drawn with the colour (an STL tinted, as before).
#[test]
fn a_colour_override_skips_the_glb_variants() {
    let variants = || MeshAsset { glb: Some(glb_asset("glb")), lod: Some(glb_asset("lod")), ..arm_mesh() };
    let mut assets = Assets::all();
    assets.add("glb", "glb-full");
    assets.add("lod", "glb-lod");
    assets.add("stl", "solid x\nendsolid x\n");

    // A URDF visual with a material colour, its mesh with both variants.
    const COLORED: &str = r#"<robot name="bot">
  <link name="base">
    <visual><geometry><mesh filename="package://bot/arm.obj"/></geometry><material name="c"><color rgba="0 1 0 1"/></material></visual>
  </link>
</robot>"#;
    assets.add("curdf", COLORED);
    let model = Node::new(
        "robots/r1",
        NodeKind::Model {
            model: ModelRef { urdf: asset("curdf", "application/xml"), meshes: BTreeMap::from([("package://bot/arm.obj".to_owned(), variants())]) },
            joints: BTreeMap::new(),
            show_collision: false,
        },
    );
    // A Mesh node with a colour, and one without.
    let colored = Node::new("colored", NodeKind::Mesh { mesh: variants(), scale: [1.0; 3], color: Some([0, 255, 0, 255]) });
    let stl = Node::new(
        "stl",
        NodeKind::Mesh {
            mesh: MeshAsset { asset: asset("stl", "model/stl"), glb: Some(glb_asset("glb")), ..MeshAsset::default() },
            scale: [1.0; 3],
            color: Some([0, 255, 0, 255]),
        },
    );
    let plain = mesh_node(variants());
    let mut scene = new_scene();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![model, colored, stl, plain]))], &assets);
    let mut store = Store::new();
    store.take(&mut scene);

    let urdf_mesh = store.below("/robots/r1/bot/visual_geometries", Mesh3D::descriptor_vertex_positions());
    assert_eq!(urdf_mesh.len(), 1, "{urdf_mesh:?}");
    for entity in ["/colored", urdf_mesh[0].as_str()] {
        assert!(store.has(entity, Mesh3D::descriptor_albedo_factor()), "{entity}: the colour");
        assert!(!store.has(entity, Mesh3D::descriptor_vertex_colors()), "{entity}: replaces the MTL's");
        assert_eq!(asset_blob(&store, entity), None, "{entity}: no GLB");
    }
    assert!(store.has("/stl", Asset3D::descriptor_albedo_factor()));
    assert_eq!(asset_blob(&store, "/stl").as_deref(), Some(&b"solid x\nendsolid x\n"[..]), "the STL, not the GLB");
    assert_eq!(asset_blob(&store, "/part").as_deref(), Some(&b"glb-full"[..]), "no colour: the GLB");
    assert!(!store.has("/part", Asset3D::descriptor_albedo_factor()));
}

/// The view's eyes written into the blueprint, in order (`EyeControls3D`: position, look target,
/// up), and whether anything of them tracks an entity.
fn eye_writes(msgs: &[LogMsg]) -> (Vec<(Vec3, Vec3, Vec3)>, bool) {
    use crate::rr::re_sdk_types::blueprint::archetypes::EyeControls3D;
    use crate::rr::re_sdk_types::components::{Position3D, Vector3D};
    let mut eyes = Vec::new();
    let mut tracking = false;
    for msg in msgs {
        let LogMsg::ArrowMsg(store, arrow_msg) = msg else { continue };
        if store.kind() != StoreKind::Blueprint {
            continue;
        }
        let chunk = Chunk::from_arrow_msg(arrow_msg).unwrap();
        tracking |= chunk.components().contains_component(EyeControls3D::descriptor_tracking_entity().component);
        let first = |c| chunk.iter_component::<Position3D>(c).flat_map(|batch| batch.to_vec()).next().map(Vec3::from);
        let (Some(position), Some(look_target)) =
            (first(EyeControls3D::descriptor_position().component), first(EyeControls3D::descriptor_look_target().component))
        else {
            continue;
        };
        let up = chunk
            .iter_component::<Vector3D>(EyeControls3D::descriptor_eye_up().component)
            .flat_map(|batch| batch.to_vec())
            .next()
            .map(Vec3::from)
            .unwrap();
        eyes.push((position, look_target, up));
    }
    (eyes, tracking)
}

/// The eyes written into the blueprint since the last `take`.
fn eyes(scene: &mut RerunScene) -> Vec<(Vec3, Vec3, Vec3)> {
    eye_writes(&take_with_eye(scene).1).0
}

/// The view's eye is fitted to the scene's bounds (model links by their joint values, meshes by
/// their extents once drawn) and written into the blueprint, with its first write: Rerun's own
/// orbital eye starts there, nothing tracked.
#[test]
fn the_view_eye_is_fitted_to_the_scene() {
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    let (chunks, others) = take_with_eye(&mut scene);
    assert!(chunks.iter().all(|chunk| !chunk.entity_path().to_string().contains("camera")), "no camera in the recording");
    let (written, tracking) = eye_writes(&others);
    assert!(!tracking, "nothing tracked");
    let activation = others.iter().position(|m| matches!(m, LogMsg::BlueprintActivationCommand(_))).unwrap();
    assert_eq!(eye_writes(&others[..activation]).0.len(), 1, "in the blueprint before it is activated (cloned)");
    let (eye, bounds) = scene.fitted_eye().unwrap();
    assert_eq!(written, [(eye.position, eye.look_target, Vec3::Z)]);

    // The base's 1 m box at x = 1, and the arm's mesh (the triangle (0,0,0) (1,0,0) (0,1,0), by
    // its box) turned by j1 = 0.5 about Z: the box's corner (1,1) goes to (c - s, c + s).
    let bounds = bounds.unwrap();
    assert_eq!(Some(bounds), framing::scene_bounds(scene.doc.as_ref().unwrap(), &scene.models, &scene.framing.meshes));
    let (c, s) = (0.5_f32.cos(), 0.5_f32.sin());
    let want = framing::Aabb { min: Vec3::new(0.5, -0.5, -0.5), max: Vec3::new(1.0 + c, c + s, 0.5) };
    assert!((bounds.min - want.min).abs().max_element() < 1e-5 && (bounds.max - want.max).abs().max_element() < 1e-5, "{bounds:?}");
    assert!((eye.look_target - bounds.center()).length() < 1e-6, "it orbits the scene's centre");

    // Unchanged joints, a small move: the eye stays.
    scene.apply(vec![upsert(2, vec![robot(false).at(Pose3::from_xyz([1.01, 0.0, 0.0]))])], &assets);
    assert!(eyes(&mut scene).is_empty(), "within the tolerance");
    // A far move: placed again.
    scene.apply(vec![upsert(3, vec![robot(false).at(Pose3::from_xyz([5.0, 0.0, 0.0]))])], &assets);
    let moved = eyes(&mut scene);
    assert_eq!(moved.len(), 1);
    assert!((moved[0].1 - scene.fitted_eye().unwrap().1.unwrap().center()).length() < 1e-6);

    // A resized view: placed again.
    scene.set_view_size(500.0, 1200.0);
    scene.apply(vec![], &assets);
    assert_eq!(eyes(&mut scene).len(), 1);
    scene.apply(vec![], &assets);
    assert!(take_with_eye(&mut scene).1.is_empty(), "nothing to do");

    // A new scene: fitted again.
    scene.apply(vec![SceneEvent::Reset(doc("s2", vec![robot(false).at(Pose3::from_xyz([-4.0, 0.0, 0.0]))]))], &assets);
    assert_eq!(eyes(&mut scene).len(), 1);
}

/// A new scene that replaced another activates the blueprint again once shown (the view shows its
/// fitted eye again, even if the user had moved the old one); nothing else does.
#[test]
fn a_new_scene_tracks_the_camera_again_once_shown() {
    let activations = |msgs: &[LogMsg]| msgs.iter().filter(|m| matches!(m, LogMsg::BlueprintActivationCommand(_))).count();
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    assert_eq!(activations(&take(&mut scene).1), 1, "the first sync activates it");
    scene.set_shown(true);
    assert_eq!(activations(&take(&mut scene).1), 0, "the first scene: nothing more");

    // Edits, poses and joints of the shown scene: never.
    scene.apply(vec![upsert(2, vec![robot(false).at(Pose3::from_xyz([9.0, 0.0, 0.0]))])], &assets);
    scene.set_shown(true);
    assert_eq!(activations(&take(&mut scene).1), 0);
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    scene.set_shown(true);
    assert_eq!(activations(&take(&mut scene).1), 0, "a resync of the same scene");

    // A new scene: only once shown, and once.
    scene.apply(vec![SceneEvent::Reset(doc("s2", vec![robot(false)]))], &assets);
    scene.set_shown(false);
    assert_eq!(activations(&take(&mut scene).1), 0, "not while another recording is shown");
    scene.set_shown(true);
    assert_eq!(activations(&take(&mut scene).1), 1);
    scene.set_shown(true);
    assert_eq!(activations(&take(&mut scene).1), 0);
}

// --- The view's eye in Rerun's own store hub: what the active blueprint holds ---

use crate::rr::re_viewer_context::{StoreHub, blueprint_timepoint_for_writes};

/// Rerun's store hub as the viewer keeps it for one scene's application: the scene's messages
/// ingested (an activation makes its blueprint the default and a clone of it the active one,
/// `StoreHub::set_cloned_blueprint_active_for_app`), and what the app appends to the active
/// blueprint (`SystemCommand::AppendToStore`, [`RerunScene::sync_eye`]).
struct Hub(StoreHub);

impl Hub {
    fn new() -> Self {
        Self(StoreHub::test_hub())
    }

    /// One frame: the scene's messages, then the eye sync (as the app's `logic` does).
    fn frame(&mut self, scene: &mut RerunScene) {
        for msg in scene.queue.take_all() {
            match msg {
                LogMsg::SetStoreInfo(info) => {
                    self.0.entity_db_entry(&info.info.store_id);
                }
                LogMsg::ArrowMsg(store, arrow_msg) => {
                    let chunk = Chunk::from_arrow_msg(&arrow_msg).unwrap();
                    self.0.entity_db_entry(&store).add_chunk(&Arc::new(chunk)).unwrap();
                }
                LogMsg::BlueprintActivationCommand(cmd) => {
                    if cmd.make_default {
                        self.0.set_default_blueprint_for_app(&cmd.blueprint_id).unwrap();
                    }
                    if cmd.make_active {
                        self.0.set_cloned_blueprint_active_for_app(&cmd.blueprint_id).unwrap();
                    }
                }
            }
        }
        if let Some((store, chunks)) = scene.sync_eye(&self.0) {
            for chunk in chunks {
                self.0.entity_db_entry(&store).add_chunk(&Arc::new(chunk)).unwrap();
            }
        }
    }

    /// The active blueprint's eye, as Rerun's 3D view reads it.
    fn eye(&self, scene: &RerunScene) -> ViewEye {
        scene.view_eye(&self.0).expect("an active blueprint").1
    }

    /// Write `components` of the view's eye into the active blueprint, as Rerun does when the user
    /// moves it (`EyeController::save_to_blueprint`, `re_view_spatial/src/eye.rs`).
    fn user_writes(&mut self, scene: &RerunScene, components: &crate::rr::re_sdk_types::blueprint::archetypes::EyeControls3D) {
        let store = self.0.active_blueprint_id_for_app(&scene.app_id).unwrap().clone();
        let timepoint = blueprint_timepoint_for_writes(self.0.entity_db(&store).unwrap());
        let chunk = Chunk::builder(scene.blueprint.eye_entity()).with_archetype(RowId::new(), timepoint, components).build().unwrap();
        self.0.entity_db_entry(&store).add_chunk(&Arc::new(chunk)).unwrap();
    }

    /// The user orbits by `yaw` radians about the Z axis: Rerun's orbital eye turns its position
    /// about the look target and keeps the look target (`EyeController::rotate` and
    /// `apply_rotation_and_radius`, `re_view_spatial/src/eye.rs`), then saves both.
    fn user_orbits(&mut self, scene: &RerunScene, yaw: f32) {
        use crate::rr::re_sdk_types::blueprint::archetypes::EyeControls3D;
        use crate::rr::re_sdk_types::components::Position3D;
        let eye = self.eye(scene);
        let (position, look_target) = (eye.position.unwrap(), eye.look_target.unwrap());
        let turned = look_target + glam::Quat::from_rotation_z(yaw).mul_vec3(position - look_target);
        self.user_writes(scene, &EyeControls3D::new().with_position(Position3D::from(turned)).with_look_target(Position3D::from(look_target)));
    }

    /// The view's reset (double-click): every eye component back to the default blueprint's
    /// (`View3DState::reset_eye`, `ViewProperty::reset_all_components`).
    fn user_resets(&mut self, scene: &RerunScene) {
        use crate::rr::re_sdk_types::blueprint::archetypes::EyeControls3D;
        use crate::rr::re_sdk_types::components::{Position3D, Vector3D};
        let default = self.0.default_blueprint_for_app(&scene.app_id).unwrap();
        let eye = ViewEye::read(default, &scene.blueprint.eye_entity());
        assert!(eye.tracking.is_none(), "the default blueprint tracks nothing");
        let mut controls = EyeControls3D::new();
        if let Some(p) = eye.position {
            controls = controls.with_position(Position3D::from(p));
        }
        if let Some(p) = eye.look_target {
            controls = controls.with_look_target(Position3D::from(p));
        }
        if let Some(up) = eye.eye_up {
            controls = controls.with_eye_up(Vector3D::from(up));
        }
        self.user_writes(scene, &controls);
    }
}

/// The orbit centre of Rerun's orbital eye: its look target, unless it tracks an entity (then
/// the eye is that entity's pose, and it orbits a point in front of it once the user takes over).
fn orbit_centre(eye: &ViewEye) -> Option<Vec3> {
    if eye.tracking.is_some() { None } else { eye.look_target }
}

fn assert_near(a: Vec3, b: Vec3, what: &str) {
    assert!((a - b).length() < 1e-5, "{what}: {a} vs {b}");
}

/// What the user orbits about is the scene's centre, from the first drag on (the eye starts at the
/// fitted pose: no jump), after a reset (double-click), and after the scene's bounds grew; once the
/// user has moved the camera, re-framing leaves it alone, until a reset or a new scene.
#[test]
fn the_user_orbits_the_scenes_centre() {
    let mut scene = new_scene();
    let assets = Assets::all();
    let mut hub = Hub::new();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &assets);
    hub.frame(&mut scene);
    scene.set_shown(true);
    hub.frame(&mut scene);

    let (fitted, bounds) = scene.fitted_eye().unwrap();
    let eye = hub.eye(&scene);
    assert_eq!(orbit_centre(&eye), Some(fitted.look_target), "the orbit centre is the fitted look target");
    assert_near(fitted.look_target, bounds.unwrap().center(), "the fitted look target");
    assert_eq!(eye.position, Some(fitted.position), "the eye starts at the fitted pose");
    assert_eq!(eye.eye_up, Some(Vec3::Z));

    // The user orbits: about the scene's centre, at the fitted distance.
    hub.user_orbits(&scene, 0.3);
    hub.frame(&mut scene);
    let orbited = hub.eye(&scene);
    assert_near(orbit_centre(&orbited).unwrap(), fitted.look_target, "after an orbit");
    let distance = |eye: &ViewEye| (eye.position.unwrap() - eye.look_target.unwrap()).length();
    assert!((distance(&orbited) - (fitted.position - fitted.look_target).length()).abs() < 1e-4);

    // The scene grows: the user's camera stays, the default blueprint gets the new fit.
    scene.apply(vec![upsert(2, vec![robot(false).at(Pose3::from_xyz([6.0, 0.0, 0.0]))])], &assets);
    hub.frame(&mut scene);
    hub.frame(&mut scene);
    let (grown, grown_bounds) = scene.fitted_eye().unwrap();
    assert_ne!(grown, fitted);
    assert_eq!(hub.eye(&scene), orbited, "the user's camera stays");

    // Double-click: the latest fit, orbiting the scene's centre again.
    hub.user_resets(&scene);
    hub.frame(&mut scene);
    let reset = hub.eye(&scene);
    assert_eq!(orbit_centre(&reset), Some(grown.look_target), "after a reset");
    assert_near(grown.look_target, grown_bounds.unwrap().center(), "the grown scene's centre");
    assert_eq!(reset.position, Some(grown.position));
    hub.user_orbits(&scene, -0.5);
    hub.frame(&mut scene);
    assert_near(orbit_centre(&hub.eye(&scene)).unwrap(), grown.look_target, "orbiting after a reset");

    // Reset again, then the scene grows: the view follows the fit again.
    hub.user_resets(&scene);
    hub.frame(&mut scene);
    scene.apply(vec![upsert(3, vec![robot(false).at(Pose3::from_xyz([-8.0, 0.0, 0.0]))])], &assets);
    hub.frame(&mut scene);
    let (regrown, _) = scene.fitted_eye().unwrap();
    assert_ne!(regrown, grown);
    assert_eq!(orbit_centre(&hub.eye(&scene)), Some(regrown.look_target), "follows the fit after a reset");
    assert_eq!(hub.eye(&scene).position, Some(regrown.position));

    // The user moves it: poses, joints and growth leave it alone.
    hub.user_orbits(&scene, 1.0);
    hub.frame(&mut scene);
    let moved = hub.eye(&scene);
    scene.apply(
        vec![SceneEvent::Poses(PoseFrame { scene_id: "s1".into(), t: None, poses: vec![NodePose { node: "robots/r1".into(), pose: Pose3::from_xyz([-30.0, 0.0, 0.0]) }] })],
        &assets,
    );
    hub.frame(&mut scene);
    let joints = JointFrame { scene_id: "s1".into(), t: None, models: vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("j1".into(), -0.7)]) }] };
    scene.apply(vec![SceneEvent::Joints(joints)], &assets);
    hub.frame(&mut scene);
    assert_ne!(scene.fitted_eye().unwrap().0, regrown, "fitted again (the default blueprint's, for a reset)");
    assert_eq!(hub.eye(&scene), moved, "the user's camera stays");

    // A new scene (activated again once shown) is framed anyway.
    scene.apply(vec![SceneEvent::Reset(doc("s2", vec![robot(false).at(Pose3::from_xyz([20.0, 0.0, 0.0]))]))], &assets);
    hub.frame(&mut scene);
    scene.set_shown(true);
    hub.frame(&mut scene);
    hub.frame(&mut scene);
    let (new_scene_eye, _) = scene.fitted_eye().unwrap();
    assert_eq!(orbit_centre(&hub.eye(&scene)), Some(new_scene_eye.look_target), "a new scene");
    assert_eq!(hub.eye(&scene).position, Some(new_scene_eye.position));
}

/// The first activation's clone may predate the fit (Rerun's fallback eye then): the fitted eye is
/// written into it as soon as there is one.
#[test]
fn a_fit_after_the_activation_reaches_the_active_blueprint() {
    let mut scene = new_scene();
    let mut hub = Hub::new();
    // Nothing loaded yet: the URDF is still on its way.
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false)]))], &Assets::default());
    hub.frame(&mut scene);
    let first = scene.fitted_eye().unwrap().0;
    assert_eq!(orbit_centre(&hub.eye(&scene)), Some(first.look_target));
    scene.apply(vec![], &Assets::all());
    hub.frame(&mut scene);
    let (loaded, _) = scene.fitted_eye().unwrap();
    assert_ne!(loaded, first, "fitted again once the model is in");
    assert_eq!(orbit_centre(&hub.eye(&scene)), Some(loaded.look_target));
}

/// A smaller drawing (a robot hidden) frames the camera again; moving or reposing the same robots
/// into smaller bounds never does.
#[test]
fn a_smaller_drawing_is_framed_again_poses_and_joints_are_not() {
    let camera = |scene: &mut RerunScene| !eyes(scene).is_empty();
    let far = |at: f64| {
        let mut r2 = robot(false).at(Pose3::from_xyz([at, 0.0, 0.0]));
        r2.id = "robots/r2".into();
        r2
    };
    let poses = |at: f64| {
        SceneEvent::Poses(PoseFrame {
            scene_id: "s1".into(),
            t: None,
            poses: vec![NodePose { node: "robots/r2".into(), pose: Pose3::from_xyz([at, 0.0, 0.0]) }],
        })
    };
    let mut scene = new_scene();
    let assets = Assets::all();
    scene.apply(vec![SceneEvent::Reset(doc("s1", vec![robot(false), far(20.0)]))], &assets);
    assert!(camera(&mut scene));

    // The same robots, moved close together: the camera stays.
    scene.apply(vec![poses(1.0)], &assets);
    assert!(!camera(&mut scene), "a pose alone");
    let joints = JointFrame {
        scene_id: "s1".into(),
        t: None,
        models: vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("j1".into(), 2.0)]) }],
    };
    scene.apply(vec![SceneEvent::Joints(joints)], &assets);
    assert!(!camera(&mut scene), "joints alone");
    scene.apply(vec![poses(20.0)], &assets);
    assert!(!camera(&mut scene), "back within the framed bounds");

    // One robot hidden: the drawing is a twentieth as large, framed again; once.
    scene.apply(vec![upsert(2, vec![far(20.0).hidden()])], &assets);
    assert!(camera(&mut scene), "a smaller drawing");
    scene.apply(vec![], &assets);
    assert!(!camera(&mut scene));
}
