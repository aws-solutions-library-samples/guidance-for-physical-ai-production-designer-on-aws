//! Renderer-neutral 3D scenes, the only 3D contract between producers (backend services,
//! fos-live-scene) and the viewer.
//!
//! A producer describes *what* is in a scene: models (URDFs, by content hash), meshes, primitives,
//! poses and joint values. How that is drawn (Rerun entities, transforms, blueprints) is the
//! viewer's business alone. Geometry never travels on Zenoh: [`AssetRef`]s are fetched over HTTP
//! from [`crate::api::asset`], content-addressed and immutable.
//!
//! On Zenoh, per `source` ([`crate::keys::scene`]):
//! * [`SceneMsg`] on `factoryos/scene/<source>`: a whole [`SceneDoc`] when the scene is replaced,
//!   [`SceneDelta`]s for edits.
//! * The latest [`SceneDoc`] on the `…/snapshot` queryable, for late joiners and after a gap in `seq`.
//! * [`JointFrame`]s on `…/joints`, the high-rate path (simulation, live cell, sliders).
//!
//! Units: metres, radians (prismatic joints: metres), right-handed Z-up.

use std::collections::BTreeMap;

/// RGBA, 0–255.
pub type Rgba = [u8; 4];

/// A file served by `GET /api/v1/assets/<sha256>` (immutable, cacheable forever).
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AssetRef {
    /// Lowercase hex SHA-256 of the file's bytes.
    pub sha256: String,
    pub size: u64,
    /// e.g. `model/obj`, `model/stl`, `model/gltf-binary`, `model/vnd.collada+xml`,
    /// `application/xml` (URDF), `text/plain` (MTL).
    pub media_type: String,
}

/// A mesh file plus the sidecars it references (an OBJ's `mtllib`, an MTL's textures, a glTF's
/// buffers), keyed by the name as written inside the referencing file.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct MeshAsset {
    pub asset: AssetRef,
    pub sidecars: BTreeMap<String, AssetRef>,
    /// The same mesh as self-contained binary glTF (`model/gltf-binary`), materials baked in, when
    /// the producer converted it (e.g. from OBJ+MTL). Preferred over `asset` + `sidecars`: smaller
    /// and needs no parsing beyond glTF.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub glb: Option<AssetRef>,
    /// A decimated GLB of the same mesh for a fast first frame; consumers may show it until `glb`
    /// (or `asset`) has arrived, or keep it where detail doesn't matter.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lod: Option<AssetRef>,
}

/// A URDF and every mesh it references, resolved by the producer (`package://`, relative paths),
/// so consumers never see a filesystem path.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct ModelRef {
    pub urdf: AssetRef,
    /// Keyed by the `<mesh filename="…">` attribute exactly as written in the URDF.
    pub meshes: BTreeMap<String, MeshAsset>,
}

/// A rigid pose: translation in metres, rotation as a unit quaternion `[x, y, z, w]`.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct Pose3 {
    pub xyz: [f64; 3],
    pub quat_xyzw: [f64; 4],
}

impl Default for Pose3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose3 {
    pub const IDENTITY: Self = Self { xyz: [0.0; 3], quat_xyzw: [0.0, 0.0, 0.0, 1.0] };

    pub fn from_xyz(xyz: [f64; 3]) -> Self {
        Self { xyz, ..Self::IDENTITY }
    }
}

/// What a node's pose is relative to: another node, optionally one of a [`NodeKind::Model`]'s
/// links (a TCP on the tool flange). `None` on [`Node::parent`] means the scene's world frame.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Parent {
    pub node: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub link: Option<String>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Node {
    /// Unique within the scene, `/`-separated (`robots/r1`, `tasks/0_weld/points`).
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<Parent>,
    #[serde(default)]
    pub pose: Pose3,
    #[serde(default = "yes")]
    pub visible: bool,
    pub kind: NodeKind,
}

fn yes() -> bool {
    true
}

fn is_yes(b: &bool) -> bool {
    *b
}

impl Node {
    pub fn new(id: impl Into<String>, kind: NodeKind) -> Self {
        Self { id: id.into(), parent: None, pose: Pose3::IDENTITY, visible: true, kind }
    }

    pub fn at(mut self, pose: Pose3) -> Self {
        self.pose = pose;
        self
    }

    pub fn under(mut self, node: impl Into<String>, link: Option<String>) -> Self {
        self.parent = Some(Parent { node: node.into(), link });
        self
    }

    pub fn hidden(mut self) -> Self {
        self.visible = false;
        self
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum NodeKind {
    /// A URDF; `pose` places its root link. Joints not listed are at 0.
    Model {
        model: ModelRef,
        #[serde(default)]
        joints: BTreeMap<String, f64>,
        #[serde(default)]
        show_collision: bool,
    },
    Mesh {
        mesh: MeshAsset,
        #[serde(default = "unit_scale")]
        scale: [f64; 3],
        /// Overrides the mesh's own colours.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        color: Option<Rgba>,
    },
    /// Centred on the node's origin.
    Boxes {
        centers: Vec<[f64; 3]>,
        half_sizes: Vec<[f64; 3]>,
        #[serde(default)]
        quats_xyzw: Vec<[f64; 4]>,
        #[serde(default)]
        colors: Vec<Rgba>,
        #[serde(default)]
        labels: Vec<String>,
        #[serde(default)]
        solid: bool,
        /// Whether `labels` are drawn: `Some(true)` always, `Some(false)` never (only on hover),
        /// `None` the viewer's choice (it may hide them when there are many).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        show_labels: Option<bool>,
    },
    /// Along the node's Z, centred on `center`.
    Cylinder {
        height: f64,
        radius: f64,
        #[serde(default)]
        center: [f64; 3],
        #[serde(default, skip_serializing_if = "Option::is_none")]
        color: Option<Rgba>,
        /// Filled (the default) or a wireframe.
        #[serde(default = "yes", skip_serializing_if = "is_yes")]
        solid: bool,
    },
    Points {
        points: Vec<[f64; 3]>,
        /// One for all, or one per point.
        #[serde(default)]
        colors: Vec<Rgba>,
        #[serde(default)]
        labels: Vec<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<f32>,
        /// As for [`NodeKind::Boxes`].
        #[serde(default, skip_serializing_if = "Option::is_none")]
        show_labels: Option<bool>,
    },
    Lines {
        strips: Vec<Vec<[f64; 3]>>,
        /// One for all, or one per strip.
        #[serde(default)]
        colors: Vec<Rgba>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<f32>,
        /// One for all, or one per strip.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        labels: Vec<String>,
        /// As for [`NodeKind::Boxes`].
        #[serde(default, skip_serializing_if = "Option::is_none")]
        show_labels: Option<bool>,
    },
    Arrows {
        origins: Vec<[f64; 3]>,
        vectors: Vec<[f64; 3]>,
        #[serde(default)]
        colors: Vec<Rgba>,
        /// One for all, or one per arrow.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        labels: Vec<String>,
        /// The shaft's radius in metres; `None`: the viewer's default.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<f32>,
        /// As for [`NodeKind::Boxes`].
        #[serde(default, skip_serializing_if = "Option::is_none")]
        show_labels: Option<bool>,
    },
    /// An inline triangle mesh (small, computed: reach domes). Files go in [`NodeKind::Mesh`].
    TriMesh {
        vertices: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        color: Option<Rgba>,
    },
    /// An RGB coordinate frame of the node (a TCP).
    Axes {
        length: f64,
        /// The arrows' radius in metres; `None`: the viewer's default (in proportion to `length`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<f32>,
    },
    /// Only a frame: groups and positions children.
    Group,
}

fn unit_scale() -> [f64; 3] {
    [1.0; 3]
}

/// How the producer would like the scene shown. Hints only.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct ViewHints {
    pub title: String,
}

/// The whole scene. A new `scene_id` means a new scene (the viewer starts afresh); the same id
/// with a higher `seq` is the same scene, edited.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct SceneDoc {
    pub scene_id: String,
    pub seq: u64,
    pub view: ViewHints,
    /// Parents before children.
    pub nodes: Vec<Node>,
}

/// An edit of scene `scene_id` that moves it from `seq - 1` to `seq`. A consumer that is not at
/// `seq - 1` re-queries the snapshot.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct SceneDelta {
    pub scene_id: String,
    pub seq: u64,
    /// Replaces nodes with the same id, appends new ones.
    pub upsert: Vec<Node>,
    /// Node ids; removes their descendants too (`a` removes `a/b`).
    pub remove: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub view: Option<ViewHints>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum SceneMsg {
    Doc(SceneDoc),
    Delta(SceneDelta),
}

/// Joint values of models in scene `scene_id`, by joint name. Joints not listed keep their value.
/// Also folded into the snapshot's [`NodeKind::Model::joints`].
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct JointFrame {
    pub scene_id: String,
    /// Producer time, seconds since the Unix epoch, if known.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t: Option<f64>,
    pub models: Vec<ModelJoints>,
}

/// New poses of nodes in scene `scene_id` (the high-rate path for moving items without resending
/// them). Also folded into the snapshot's [`Node::pose`]. Does not advance `seq`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct PoseFrame {
    pub scene_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t: Option<f64>,
    pub poses: Vec<NodePose>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct NodePose {
    pub node: String,
    pub pose: Pose3,
}

/// One answer of the backend's asset registration ([`crate::keys::ASSETS_REGISTER`]), per path
/// asked: the file's [`AssetRef`] and, for meshes it converted, the [`MeshAsset::glb`] /
/// [`MeshAsset::lod`] variants; or why it was refused.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Registered {
    Asset {
        asset: AssetRef,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        glb: Option<AssetRef>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        lod: Option<AssetRef>,
    },
    Error(String),
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct ModelJoints {
    pub node: String,
    pub joints: BTreeMap<String, f64>,
}

/// Whether `id` is `ancestor` or below it.
pub fn is_under(id: &str, ancestor: &str) -> bool {
    id.strip_prefix(ancestor).is_some_and(|rest| rest.is_empty() || rest.starts_with('/'))
}

impl SceneDoc {
    pub fn node(&self, id: &str) -> Option<&Node> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// Applies `delta`. Returns `false` (and leaves `self` unchanged) if it is for another scene
    /// or does not follow `self.seq`.
    pub fn apply(&mut self, delta: &SceneDelta) -> bool {
        if delta.scene_id != self.scene_id || delta.seq != self.seq + 1 {
            return false;
        }
        self.nodes.retain(|n| !delta.remove.iter().any(|r| is_under(&n.id, r)));
        for node in &delta.upsert {
            match self.nodes.iter_mut().find(|n| n.id == node.id) {
                Some(slot) => *slot = node.clone(),
                None => self.nodes.push(node.clone()),
            }
        }
        if let Some(view) = &delta.view {
            self.view = view.clone();
        }
        self.seq = delta.seq;
        true
    }

    /// Folds `frame` into the nodes' poses. Returns whether anything changed.
    pub fn apply_poses(&mut self, frame: &PoseFrame) -> bool {
        if frame.scene_id != self.scene_id {
            return false;
        }
        let mut changed = false;
        for p in &frame.poses {
            if let Some(node) = self.nodes.iter_mut().find(|n| n.id == p.node)
                && node.pose != p.pose
            {
                node.pose = p.pose;
                changed = true;
            }
        }
        changed
    }

    /// Folds `frame` into the models' joint values. Returns whether anything changed.
    pub fn apply_joints(&mut self, frame: &JointFrame) -> bool {
        if frame.scene_id != self.scene_id {
            return false;
        }
        let mut changed = false;
        for m in &frame.models {
            if let Some(Node { kind: NodeKind::Model { joints, .. }, .. }) = self.nodes.iter_mut().find(|n| n.id == m.node) {
                for (name, value) in &m.joints {
                    if joints.insert(name.clone(), *value) != Some(*value) {
                        changed = true;
                    }
                }
            }
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn doc() -> SceneDoc {
        SceneDoc {
            scene_id: "s1".into(),
            seq: 1,
            view: ViewHints { title: "Robo Fitter".into() },
            nodes: vec![
                Node::new(
                    "robots/r1",
                    NodeKind::Model {
                        model: ModelRef {
                            urdf: AssetRef { sha256: "ab".into(), size: 3, media_type: "application/xml".into() },
                            meshes: BTreeMap::from([("meshes/base.obj".into(), MeshAsset::default())]),
                        },
                        joints: BTreeMap::from([("A1".into(), 0.5)]),
                        show_collision: false,
                    },
                )
                .at(Pose3::from_xyz([1.0, 2.0, 0.0])),
                Node::new("robots/r1/tcp", NodeKind::Axes { length: 0.1, radius: Some(0.003) }).under("robots/r1", Some("tool0".into())),
                Node::new("robots/r10", NodeKind::Group),
                Node::new(
                    "task/paths",
                    NodeKind::Lines {
                        strips: vec![vec![[0.0; 3], [1.0, 0.0, 0.0]]],
                        colors: vec![],
                        radius: Some(0.005),
                        labels: vec!["seam".into()],
                        show_labels: Some(true),
                    },
                ),
                Node::new(
                    "task/approach",
                    NodeKind::Arrows {
                        origins: vec![[0.0; 3]],
                        vectors: vec![[0.0, 0.0, 0.1]],
                        colors: vec![],
                        labels: vec!["approach".into()],
                        radius: Some(0.005),
                        show_labels: Some(false),
                    },
                ),
                Node::new("pedestal", NodeKind::Cylinder { height: 0.5, radius: 0.2, center: [0.0; 3], color: None, solid: false }),
            ],
        }
    }

    #[test]
    fn round_trips_as_json() {
        let msg = SceneMsg::Doc(doc());
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains(r#""type":"doc""#) && json.contains(r#""type":"model""#) && json.contains(r#""showCollision""#));
        assert!(json.contains(r#""showLabels":true"#) && json.contains(r#""solid":false"#) && json.contains(r#""labels":["seam"]"#));
        assert_eq!(serde_json::from_str::<SceneMsg>(&json).unwrap(), msg);
    }

    /// Scenes written before `labels`/`radius` on lines and arrows, `show_labels` and `solid`
    /// cylinders read as before, and the defaults are not written.
    #[test]
    fn new_fields_are_optional() {
        let kinds = [
            (r#"{"type":"lines","strips":[]}"#, NodeKind::Lines { strips: vec![], colors: vec![], radius: None, labels: vec![], show_labels: None }),
            (
                r#"{"type":"arrows","origins":[],"vectors":[]}"#,
                NodeKind::Arrows { origins: vec![], vectors: vec![], colors: vec![], labels: vec![], radius: None, show_labels: None },
            ),
            (r#"{"type":"axes","length":0.1}"#, NodeKind::Axes { length: 0.1, radius: None }),
            (
                r#"{"type":"cylinder","height":1.0,"radius":0.5}"#,
                NodeKind::Cylinder { height: 1.0, radius: 0.5, center: [0.0; 3], color: None, solid: true },
            ),
            (
                r#"{"type":"points","points":[]}"#,
                NodeKind::Points { points: vec![], colors: vec![], labels: vec![], radius: None, show_labels: None },
            ),
        ];
        for (json, kind) in kinds {
            assert_eq!(serde_json::from_str::<NodeKind>(json).unwrap(), kind, "{json}");
            let written = serde_json::to_string(&kind).unwrap();
            assert!(!written.contains("showLabels") && !written.contains("solid") && !written.contains(r#""radius":null"#), "{written}");
            if matches!(kind, NodeKind::Lines { .. } | NodeKind::Arrows { .. }) {
                assert!(!written.contains("labels"), "{written}");
            }
        }
    }

    #[test]
    fn node_defaults() {
        let n: Node = serde_json::from_str(r#"{"id":"a","kind":{"type":"group"}}"#).unwrap();
        assert!(n.visible);
        assert_eq!(n.pose, Pose3::IDENTITY);
    }

    #[test]
    fn delta_applies_in_order_only() {
        let mut d = doc();
        let delta = SceneDelta {
            scene_id: "s1".into(),
            seq: 2,
            upsert: vec![Node::new("floor", NodeKind::Lines { strips: vec![], colors: vec![], radius: None, labels: vec![], show_labels: None })],
            remove: vec!["robots/r1".into()],
            view: None,
        };
        assert!(!d.apply(&SceneDelta { seq: 3, ..delta.clone() }));
        assert!(d.apply(&delta));
        let ids: Vec<_> = d.nodes.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(ids, ["robots/r10", "task/paths", "task/approach", "pedestal", "floor"]);
        assert_eq!(d.seq, 2);
    }

    #[test]
    fn poses_fold_into_nodes() {
        let mut d = doc();
        let frame = PoseFrame { scene_id: "s1".into(), t: None, poses: vec![NodePose { node: "robots/r1".into(), pose: Pose3::from_xyz([3.0, 0.0, 0.0]) }] };
        assert!(d.apply_poses(&frame));
        assert!(!d.apply_poses(&frame));
        assert_eq!(d.node("robots/r1").unwrap().pose.xyz, [3.0, 0.0, 0.0]);
        assert_eq!(d.seq, 1);
    }

    #[test]
    fn mesh_variants_and_registration_are_optional_on_the_wire() {
        let m: MeshAsset = serde_json::from_str(r#"{"asset":{"sha256":"a","size":1,"mediaType":"model/obj"}}"#).unwrap();
        assert!(m.glb.is_none() && m.lod.is_none());
        assert!(!serde_json::to_string(&m).unwrap().contains("glb"));
        let r: Registered = serde_json::from_str(r#"{"asset":{"asset":{"sha256":"a","size":1,"mediaType":"model/obj"}}}"#).unwrap();
        assert!(matches!(r, Registered::Asset { glb: None, .. }));
        let e: Registered = serde_json::from_str(r#"{"error":"outside"}"#).unwrap();
        assert_eq!(e, Registered::Error("outside".into()));
    }

    #[test]
    fn joints_fold_into_models() {
        let mut d = doc();
        let frame = JointFrame {
            scene_id: "s1".into(),
            t: None,
            models: vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("A1".into(), 0.5), ("A2".into(), 1.0)]) }],
        };
        assert!(d.apply_joints(&frame));
        assert!(!d.apply_joints(&frame));
        let Some(Node { kind: NodeKind::Model { joints, .. }, .. }) = d.node("robots/r1") else { panic!() };
        assert_eq!(joints.get("A2"), Some(&1.0));
    }
}
