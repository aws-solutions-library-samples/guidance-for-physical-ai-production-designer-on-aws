//! Renderer-neutral 3D scenes made by the backend (Robo Dresser / Robo Fitter / Product / Training
//! previews), published on `factoryos/scene/<source>` ([`factoryos_protocol::scene`]).
//!
//! A [`ScenePublisher`] keeps its source's latest [`SceneDoc`]: a new scene goes out whole
//! ([`SceneMsg::Doc`]), edits as [`SceneDelta`]s, joint values as [`JointFrame`]s on
//! `…/joints`, moves of nodes that change nothing else as [`PoseFrame`]s on `…/poses`. Late joiners query `…/snapshot` and get the one current doc, never a replay.

use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use factoryos_protocol::keys;
use factoryos_protocol::scene::{JointFrame, ModelJoints, Node, NodePose, PoseFrame, SceneDelta, SceneDoc, SceneMsg, ViewHints, is_under};

use crate::bus::{self, Bus};

/// Publishes one source's scenes, and answers its snapshot queryable.
pub struct ScenePublisher {
    bus: Bus,
    key: String,
    joints_key: String,
    poses_key: String,
    /// Locked while publishing too, so messages go out in `seq` order.
    state: Mutex<SceneState>,
}

impl ScenePublisher {
    pub fn new(bus: &Bus, source: &str) -> Arc<Self> {
        let publisher = Arc::new(Self {
            bus: bus.clone(),
            key: keys::scene(source),
            joints_key: keys::scene_joints(source),
            poses_key: keys::scene_poses(source),
            state: Mutex::new(SceneState::new(source)),
        });
        publisher.serve_snapshot(keys::scene_snapshot(source));
        publisher
    }

    /// A new scene of `nodes` (parents before children), replacing the previous one. Returns its id.
    pub fn set(&self, nodes: Vec<Node>, view: ViewHints) -> String {
        let mut state = self.state.lock().unwrap();
        let msg = state.set(nodes, view);
        let scene_id = state.doc.as_ref().map(|doc| doc.scene_id.clone()).unwrap_or_default();
        self.bus.put_json(&self.key, &msg);
        scene_id
    }

    /// Edits the current scene: `upsert` replaces nodes with the same id or appends them,
    /// `remove` drops nodes and their descendants. Nothing goes out if nothing changes. Without a
    /// current scene, starts one of `upsert`.
    pub fn update(&self, upsert: Vec<Node>, remove: Vec<String>, view: Option<ViewHints>) {
        let mut state = self.state.lock().unwrap();
        if let Some(msg) = state.update(upsert, remove, view) {
            self.bus.put_json(&self.key, &msg);
        }
    }

    /// Adds or replaces one node.
    pub fn upsert(&self, node: Node) {
        self.update(vec![node], Vec::new(), None);
    }

    /// Joint values of the current scene's models (by node id), stamped now. Folded into the
    /// snapshot; nothing goes out if no value changes or there is no scene.
    pub fn joints(&self, models: Vec<ModelJoints>) {
        let mut state = self.state.lock().unwrap();
        if let Some(frame) = state.joints(models, Some(unix_now())) {
            self.bus.put_json(&self.joints_key, &frame);
        }
    }

    /// New poses of the current scene's nodes (by id), stamped now. Folded into the snapshot; only
    /// poses that change go out, nothing if none does or there is no scene.
    #[allow(dead_code, reason = "today's producers move nodes through `update_moving`")]
    pub fn poses(&self, poses: Vec<NodePose>) {
        let mut state = self.state.lock().unwrap();
        self.put_poses(&mut state, poses);
    }

    fn put_poses(&self, state: &mut SceneState, poses: Vec<NodePose>) {
        if let Some(frame) = state.poses(poses, Some(unix_now())) {
            self.bus.put_json(&self.poses_key, &frame);
        }
    }

    /// Edits the current scene like [`Self::update`], except that upserts of existing nodes that
    /// differ only in their pose go out as a [`PoseFrame`] instead of in the delta.
    pub fn update_moving(&self, upsert: Vec<Node>, remove: Vec<String>, view: Option<ViewHints>) {
        let mut state = self.state.lock().unwrap();
        let (upsert, poses) = state.split_moves(upsert, &remove);
        self.put_poses(&mut state, poses);
        if let Some(msg) = state.update(upsert, remove, view) {
            self.bus.put_json(&self.key, &msg);
        }
    }

    /// The current scene (an empty doc if there is none yet).
    pub fn doc(&self) -> SceneDoc {
        self.state.lock().unwrap().doc.clone().unwrap_or_default()
    }

    fn serve_snapshot(self: &Arc<Self>, key: String) {
        let this = Arc::downgrade(self);
        let session = self.bus.session().clone();
        tokio::spawn(async move {
            let queryable = match session.declare_queryable(&key).await {
                Ok(queryable) => queryable,
                Err(err) => {
                    tracing::error!("{key}: {err}");
                    return;
                }
            };
            while let Ok(query) = queryable.recv_async().await {
                let Some(this) = this.upgrade() else { break };
                let doc = this.state.lock().unwrap().doc.clone();
                if let Some(doc) = doc {
                    bus::reply_json(&query, &doc).await;
                }
            }
        });
    }
}

fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or_default()
}

/// A [`ScenePublisher`]'s state, without the bus: the current doc and how the next message is made.
struct SceneState {
    source: String,
    /// Scenes made by this publisher, part of each scene id.
    scenes: u64,
    /// Tells this process's scene ids from an earlier one's (a backend restart).
    epoch: u64,
    doc: Option<SceneDoc>,
}

impl SceneState {
    fn new(source: &str) -> Self {
        let epoch = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis() as u64).unwrap_or_default();
        Self { source: source.to_owned(), scenes: 0, epoch, doc: None }
    }

    fn set(&mut self, nodes: Vec<Node>, view: ViewHints) -> SceneMsg {
        self.scenes += 1;
        let doc = SceneDoc {
            scene_id: format!("{}-{}-{:x}", self.source, self.scenes, self.epoch),
            seq: 0,
            view,
            nodes,
        };
        self.doc = Some(doc.clone());
        SceneMsg::Doc(doc)
    }

    /// The delta that makes the edit, applied; `None` if it changes nothing.
    fn update(&mut self, upsert: Vec<Node>, remove: Vec<String>, view: Option<ViewHints>) -> Option<SceneMsg> {
        let Some(doc) = &mut self.doc else {
            if upsert.is_empty() {
                return None;
            }
            return Some(self.set(upsert, view.unwrap_or_default()));
        };
        let remove: Vec<String> = remove.into_iter().filter(|id| doc.nodes.iter().any(|n| is_under(&n.id, id))).collect();
        // An upsert identical to the current node is dropped, unless a removal takes that node
        // out first: then it has to go out to put it back.
        let upsert = upsert
            .into_iter()
            .filter(|node| doc.node(&node.id) != Some(node) || remove.iter().any(|r| is_under(&node.id, r)))
            .collect();
        let delta = SceneDelta { scene_id: doc.scene_id.clone(), seq: doc.seq + 1, upsert, remove, view: view.filter(|view| *view != doc.view) };
        if delta.upsert.is_empty() && delta.remove.is_empty() && delta.view.is_none() {
            return None;
        }
        let applied = doc.apply(&delta);
        debug_assert!(applied);
        Some(SceneMsg::Delta(delta))
    }

    fn joints(&mut self, models: Vec<ModelJoints>, t: Option<f64>) -> Option<JointFrame> {
        let doc = self.doc.as_mut()?;
        let frame = JointFrame { scene_id: doc.scene_id.clone(), t, models };
        doc.apply_joints(&frame).then_some(frame)
    }

    /// The poses that change, applied; `None` if none does or there is no scene.
    fn poses(&mut self, poses: Vec<NodePose>, t: Option<f64>) -> Option<PoseFrame> {
        let doc = self.doc.as_mut()?;
        let mut seen = std::collections::BTreeSet::new();
        // The last pose given for a node wins; nodes not in the doc and unchanged poses are left out.
        let mut poses: Vec<NodePose> = poses
            .into_iter()
            .rev()
            .filter(|p| seen.insert(p.node.clone()) && doc.node(&p.node).is_some_and(|n| n.pose != p.pose))
            .collect();
        poses.reverse();
        if poses.is_empty() {
            return None;
        }
        let frame = PoseFrame { scene_id: doc.scene_id.clone(), t, poses };
        doc.apply_poses(&frame).then_some(frame)
    }

    /// Splits `upsert` into the nodes that change more than their pose (or are new, or under a
    /// removal) and the poses of those that only move.
    fn split_moves(&self, upsert: Vec<Node>, remove: &[String]) -> (Vec<Node>, Vec<NodePose>) {
        let Some(doc) = &self.doc else { return (upsert, Vec::new()) };
        let mut nodes = Vec::new();
        let mut poses = Vec::new();
        for node in upsert {
            let moved_only = !remove.iter().any(|r| is_under(&node.id, r))
                && doc.node(&node.id).is_some_and(|old| old.pose != node.pose && Node { pose: node.pose, ..old.clone() } == node);
            if moved_only {
                poses.push(NodePose { node: node.id, pose: node.pose });
            } else {
                nodes.push(node);
            }
        }
        (nodes, poses)
    }
}

/// A robot's name label hangs this far above its shoulder ([`robot_label_node`]).
pub const ROBOT_LABEL_ABOVE_SHOULDER_M: f64 = 0.45;
/// Where a robot's label hangs above its base without a shoulder to go by.
pub const ROBOT_LABEL_DEFAULT_M: f64 = 1.5;
const ROBOT_LABEL_COLOR: factoryos_protocol::scene::Rgba = [0x7c, 0x4d, 0xff, 0xff];

/// The id of robot node `robot`'s name label ([`robot_label_node`]).
pub fn robot_label_id(robot: &str) -> String {
    format!("{robot}/name_label")
}

/// Robot node `robot`'s name label: `name` on a small point above A1, over its shoulder
/// (`shoulder_height` above the root link, [`UrdfModel::shoulder_height`](crate::services::urdf_model::UrdfModel::shoulder_height)),
/// in the robot node's frame so it moves with the robot; always shown (the viewer's label
/// toggle hides it).
pub fn robot_label_node(robot: &str, name: &str, shoulder_height: Option<f64>) -> Node {
    let height = shoulder_height.map_or(ROBOT_LABEL_DEFAULT_M, |z| z.max(0.0) + ROBOT_LABEL_ABOVE_SHOULDER_M);
    Node::new(
        robot_label_id(robot),
        factoryos_protocol::scene::NodeKind::Points {
            points: vec![[0.0, 0.0, height]],
            colors: vec![ROBOT_LABEL_COLOR],
            labels: vec![name.to_owned()],
            radius: Some(0.02),
            show_labels: Some(true),
        },
    )
    .under(robot, None)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use factoryos_protocol::scene::{ModelRef, NodeKind, Pose3};

    use super::*;

    fn robot(a1: f64) -> Node {
        Node::new(
            "robots/r1",
            NodeKind::Model {
                model: ModelRef::default(),
                joints: BTreeMap::from([("A1".into(), a1)]),
                show_collision: false,
            },
        )
    }

    fn tcp() -> Node {
        Node::new("robots/r1/tcp", NodeKind::Axes { length: 0.1, radius: None }).under("robots/r1", Some("tool0".into()))
    }

    #[test]
    fn set_starts_new_scenes() {
        let mut state = SceneState::new("fitter");
        let SceneMsg::Doc(first) = state.set(vec![robot(0.0)], ViewHints { title: "Robo Fitter".into() }) else { panic!() };
        assert_eq!(first.seq, 0);
        assert!(first.scene_id.starts_with("fitter-1-"));
        let SceneMsg::Doc(second) = state.set(vec![], ViewHints::default()) else { panic!() };
        assert_ne!(first.scene_id, second.scene_id);
        assert_eq!(state.doc, Some(second));
    }

    #[test]
    fn updates_are_sequenced_deltas() {
        let mut state = SceneState::new("dresser");
        let SceneMsg::Doc(doc) = state.set(vec![robot(0.0)], ViewHints::default()) else { panic!() };
        let mut mirror = doc.clone();

        let Some(SceneMsg::Delta(delta)) = state.update(vec![robot(0.0), tcp()], vec!["nothing".into()], None) else { panic!() };
        // Unchanged nodes and removals that match nothing are left out.
        assert_eq!((delta.seq, delta.upsert.len(), delta.remove.len()), (1, 1, 0));
        assert!(mirror.apply(&delta));

        assert_eq!(state.update(vec![tcp()], vec![], Some(ViewHints::default())), None);

        let moved = robot(0.0).at(Pose3::from_xyz([1.0, 0.0, 0.0]));
        let Some(SceneMsg::Delta(delta)) = state.update(vec![moved], vec![], None) else { panic!() };
        assert!(mirror.apply(&delta));
        let Some(SceneMsg::Delta(delta)) = state.update(vec![], vec!["robots/r1".into()], Some(ViewHints { title: "t".into() })) else {
            panic!()
        };
        assert_eq!(delta.seq, 3);
        assert!(mirror.apply(&delta));
        assert!(mirror.nodes.is_empty());
        assert_eq!(state.doc, Some(mirror));
    }

    #[test]
    fn removals_keep_unchanged_upserts_under_them() {
        let mut state = SceneState::new("training");
        state.set(vec![robot(0.0), tcp()], ViewHints::default());
        let mut mirror = state.doc.clone().unwrap();
        // Replace `robots/r1` and everything under it with the same `robots/r1/tcp`.
        let Some(SceneMsg::Delta(delta)) = state.update(vec![tcp()], vec!["robots/r1".into()], None) else { panic!() };
        assert_eq!(delta.upsert, vec![tcp()]);
        assert!(mirror.apply(&delta));
        assert_eq!(mirror.nodes, vec![tcp()]);
        assert_eq!(state.doc, Some(mirror));
    }

    #[test]
    fn update_without_a_scene_starts_one() {
        let mut state = SceneState::new("product");
        assert_eq!(state.update(vec![], vec!["x".into()], None), None);
        assert!(matches!(state.update(vec![tcp()], vec![], None), Some(SceneMsg::Doc(doc)) if doc.nodes.len() == 1));
    }

    #[test]
    fn joints_fold_into_the_doc() {
        let mut state = SceneState::new("training");
        let joints = |a1: f64| vec![ModelJoints { node: "robots/r1".into(), joints: BTreeMap::from([("A1".into(), a1)]) }];
        assert_eq!(state.joints(joints(1.0), None), None, "no scene yet");
        state.set(vec![robot(0.0)], ViewHints::default());
        let frame = state.joints(joints(1.0), Some(5.0)).unwrap();
        assert_eq!((frame.scene_id.as_str(), frame.t), (state.doc.as_ref().unwrap().scene_id.as_str(), Some(5.0)));
        assert_eq!(state.joints(joints(1.0), None), None, "unchanged");
        let doc = state.doc.as_ref().unwrap();
        assert_eq!(doc.seq, 0, "joints don't advance seq");
        assert_eq!(doc.node("robots/r1"), Some(&robot(1.0)));
    }

    #[test]
    fn poses_fold_into_the_doc_and_only_changes_go_out() {
        let mut state = SceneState::new("fitter");
        let pose = |node: &str, x: f64| NodePose { node: node.into(), pose: Pose3::from_xyz([x, 0.0, 0.0]) };
        assert_eq!(state.poses(vec![pose("robots/r1", 1.0)], None), None, "no scene yet");
        state.set(vec![robot(0.0), tcp()], ViewHints::default());
        let frame = state.poses(vec![pose("robots/r1", 0.0), pose("robots/r1/tcp", 2.0), pose("nothing", 1.0)], Some(5.0)).unwrap();
        assert_eq!(frame.poses, vec![pose("robots/r1/tcp", 2.0)], "unchanged and unknown nodes are left out");
        assert_eq!(frame.t, Some(5.0));
        assert_eq!(state.poses(vec![pose("robots/r1/tcp", 2.0)], None), None, "unchanged");
        let doc = state.doc.as_ref().unwrap();
        assert_eq!(doc.seq, 0, "poses don't advance seq");
        assert_eq!(doc.node("robots/r1/tcp").unwrap().pose, Pose3::from_xyz([2.0, 0.0, 0.0]));
    }

    #[test]
    fn moves_split_from_real_changes() {
        let mut state = SceneState::new("fitter");
        state.set(vec![robot(0.0), tcp()], ViewHints::default());
        let moved = robot(0.0).at(Pose3::from_xyz([1.0, 0.0, 0.0]));
        let changed = robot(1.0).at(Pose3::from_xyz([1.0, 0.0, 0.0]));
        let (nodes, poses) = state.split_moves(vec![moved.clone(), tcp()], &[]);
        assert_eq!((nodes, poses), (vec![tcp()], vec![NodePose { node: "robots/r1".into(), pose: moved.pose }]));
        let (nodes, poses) = state.split_moves(vec![changed.clone()], &[]);
        assert_eq!((nodes, poses), (vec![changed], vec![]));
        let (nodes, poses) = state.split_moves(vec![moved.clone()], &["robots".into()]);
        assert_eq!((nodes, poses), (vec![moved], vec![]), "under a removal it has to be re-sent");
    }
}
