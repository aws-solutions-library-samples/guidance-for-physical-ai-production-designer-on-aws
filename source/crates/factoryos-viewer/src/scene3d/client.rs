//! Follows one source's scene on Zenoh. Neutral: no Rerun here.
//!
//! Subscribe first, hold live edits back, query the snapshot, then
//! go on live. [`Sequencer`] checks that every [`SceneDelta`] follows the scene shown and resyncs
//! from the snapshot after a gap (a lost sample, a producer restart, a reconnect).
//!
//! A re-join of the backend ([`Bus::on_rejoin`]: it restarted, or the connection was lost) queries
//! the snapshot too: a doc, deltas or joint and pose frames published while away were missed, and
//! a quiet producer sends nothing after them that would show a gap.

use std::sync::{Arc, Mutex};

use factoryos_client::Bus;
use factoryos_client::protocol::keys;
use factoryos_client::protocol::scene::{JointFrame, PoseFrame, SceneDelta, SceneDoc, SceneMsg};

use crate::rr::re_log;

use super::{SceneEvent, Waker};

/// Live deltas held back while resyncing, at most (older ones are dropped; the resync after the
/// snapshot then notices the gap and queries again).
const MAX_HELD: usize = 1024;

pub struct SceneClient {
    bus: Bus,
    source: String,
    shared: Arc<Mutex<Sequencer>>,
    waker: Waker,
}

impl SceneClient {
    /// A client that follows nothing (a backend speaking another protocol version).
    pub fn idle(bus: &Bus, source: &str, waker: Waker) -> Self {
        Self { bus: bus.clone(), source: source.to_owned(), shared: Arc::new(Mutex::new(Sequencer::new())), waker }
    }

    pub fn connect(bus: &Bus, source: &str, waker: Waker) -> Self {
        let shared = Arc::new(Mutex::new(Sequencer::new()));

        let sink = shared.clone();
        let wake = waker.clone();
        let src = source.to_owned();
        let subscribed = bus.subscribe(&keys::scene(source), move |payload| match serde_json::from_slice::<SceneMsg>(&payload) {
            Ok(msg) => {
                sink.lock().unwrap().on_msg(msg);
                // Also when only a re-query is wanted: `drain` issues it.
                wake.wake();
            }
            Err(err) => re_log::warn_once!("Scene {src}: malformed scene message: {err}"),
        });
        if let Err(err) = subscribed {
            re_log::warn!("Scene {source}: {err}");
        }

        let sink = shared.clone();
        let wake = waker.clone();
        let src = source.to_owned();
        let subscribed = bus.subscribe(&keys::scene_joints(source), move |payload| match serde_json::from_slice::<JointFrame>(&payload) {
            Ok(frame) => {
                if sink.lock().unwrap().on_joints(frame) {
                    wake.wake();
                }
            }
            Err(err) => re_log::warn_once!("Scene {src}: malformed joint frame: {err}"),
        });
        if let Err(err) = subscribed {
            re_log::warn!("Scene {source}: {err}");
        }

        let sink = shared.clone();
        let wake = waker.clone();
        let src = source.to_owned();
        let subscribed = bus.subscribe(&keys::scene_poses(source), move |payload| match serde_json::from_slice::<PoseFrame>(&payload) {
            Ok(frame) => {
                if sink.lock().unwrap().on_poses(frame) {
                    wake.wake();
                }
            }
            Err(err) => re_log::warn_once!("Scene {src}: malformed pose frame: {err}"),
        });
        if let Err(err) = subscribed {
            re_log::warn!("Scene {source}: {err}");
        }

        // Until this client is dropped.
        let sink = Arc::downgrade(&shared);
        let wake = waker.clone();
        bus.on_rejoin(move || {
            let Some(sink) = sink.upgrade() else { return false };
            sink.lock().unwrap().on_rejoin();
            // `drain` issues the query.
            wake.wake();
            true
        });

        let client = Self {
            bus: bus.clone(),
            source: source.to_owned(),
            shared,
            waker,
        };
        // The sequencer starts with this query in flight.
        client.query_snapshot();
        client
    }

    /// Events since the last call, in order. Also issues a snapshot query the live messages asked
    /// for (here rather than in the bus callbacks: the web bus is not `Send`).
    pub fn drain(&self) -> Vec<SceneEvent> {
        let (events, query) = {
            let mut seq = self.shared.lock().unwrap();
            let query = seq.take_query_request();
            (seq.drain(), query)
        };
        if query {
            self.query_snapshot();
        }
        events
    }

    /// Query the snapshot again, as after a re-join (the backend restarted, as its health check
    /// says, whether or not the bus noticed).
    pub fn resync(&self) {
        self.shared.lock().unwrap().on_rejoin();
        // `drain` issues the query.
        self.waker.wake();
    }

    /// The backend went away and the session has not re-joined it yet: the re-join's snapshot
    /// query is still to come ([`Bus::backend_away`]).
    pub fn backend_away(&self) -> bool {
        self.bus.backend_away()
    }

    /// Events wait for the next [`Self::drain`], or a snapshot query is wanted or in flight.
    pub fn has_events(&self) -> bool {
        let seq = self.shared.lock().unwrap();
        !seq.out.is_empty() || seq.query_wanted || seq.query_in_flight
    }

    fn query_snapshot(&self) {
        let sink = self.shared.clone();
        let wake = self.waker.clone();
        let src = self.source.clone();
        self.bus.query(&keys::scene_snapshot(&self.source), None, move |reply| {
            let reply = match reply {
                Ok(Some(payload)) => match serde_json::from_slice::<SceneDoc>(&payload) {
                    Ok(doc) => Ok(Some(doc)),
                    Err(err) => {
                        re_log::warn_once!("Scene {src}: malformed snapshot: {err}");
                        Err(err.to_string())
                    }
                },
                Ok(None) => Ok(None),
                Err(err) => {
                    re_log::debug!("Scene {src}: snapshot query: {err}");
                    Err(err)
                }
            };
            sink.lock().unwrap().on_snapshot(reply);
            wake.wake();
        });
    }
}

/// Turns live messages and snapshot replies into a consistent [`SceneEvent`] stream. Pure; the
/// caller does the I/O: when [`Sequencer::take_query_request`] says so it queries the snapshot
/// and hands the reply to [`Sequencer::on_snapshot`].
#[derive(Debug)]
pub(crate) struct Sequencer {
    /// The scene shown: `(scene_id, seq)`.
    cur: Option<(String, u64)>,
    /// A snapshot query is in flight (one at a time).
    query_in_flight: bool,
    /// A snapshot query should be issued (the caller takes it).
    query_wanted: bool,
    /// A live `Doc` arrived since the query in flight was issued: it supersedes a reply for
    /// another scene (any newer scene is announced live as a `Doc` too).
    doc_since_query: bool,
    /// A gap was noticed in live deltas (not held ones) while the query was in flight: its reply
    /// may predate it, so a gap still open after the reply is asked about once more.
    gap_since_query: bool,
    /// A re-join not answered yet: the next snapshot query is for it (wanted, or in flight and
    /// issued before the re-join, whose reply then predates it).
    rejoin_pending: bool,
    /// The query in flight was issued after a re-join: its reply is shown even at the seq shown
    /// (joints and poses folded into it may be newer than the frames seen).
    rejoin_in_flight: bool,
    /// Deltas waiting for the snapshot.
    held: Vec<SceneDelta>,
    out: Vec<SceneEvent>,
}

impl Sequencer {
    /// Starts with the initial snapshot query in flight (the caller issues it).
    pub fn new() -> Self {
        Self {
            cur: None,
            query_in_flight: true,
            query_wanted: false,
            doc_since_query: false,
            gap_since_query: false,
            rejoin_pending: false,
            rejoin_in_flight: false,
            held: Vec::new(),
            out: Vec::new(),
        }
    }

    /// Whether to issue a snapshot query now; if `true`, it counts as in flight.
    pub fn take_query_request(&mut self) -> bool {
        if self.query_wanted && !self.query_in_flight {
            self.query_wanted = false;
            self.query_in_flight = true;
            self.doc_since_query = false;
            self.gap_since_query = false;
            self.rejoin_in_flight = std::mem::take(&mut self.rejoin_pending);
            true
        } else {
            false
        }
    }

    pub fn drain(&mut self) -> Vec<SceneEvent> {
        std::mem::take(&mut self.out)
    }

    /// Waiting for a snapshot before live deltas can be applied.
    fn holding(&self) -> bool {
        (self.query_in_flight && !self.doc_since_query) || self.query_wanted
    }

    /// The session re-joined the backend: what was published while away is missed, so query the
    /// snapshot, after the one in flight (if any) since that may predate the re-join. Live
    /// deltas wait for its reply.
    pub fn on_rejoin(&mut self) {
        self.rejoin_pending = true;
        if !self.query_in_flight {
            self.query_wanted = true;
        }
    }

    pub fn on_msg(&mut self, msg: SceneMsg) {
        match msg {
            SceneMsg::Doc(doc) => self.on_doc(doc),
            SceneMsg::Delta(delta) => self.on_live_delta(delta),
        }
    }

    fn on_doc(&mut self, doc: SceneDoc) {
        // A doc older than the scene shown says nothing new (nor resolves a gap).
        if self.is_stale(&doc.scene_id, doc.seq) {
            return;
        }
        if self.query_in_flight {
            self.doc_since_query = true;
        } else {
            // A gap noticed or a re-join not queried yet: this doc resolves it.
            self.query_wanted = false;
        }
        // Live after the re-join: the scene as it is now, and a reply at its seq is no news.
        self.rejoin_pending = false;
        self.rejoin_in_flight = false;
        let (id, seq) = (doc.scene_id.clone(), doc.seq);
        self.show(doc);
        // Held deltas that follow the new scene apply now; the rest are older.
        let mut held = std::mem::take(&mut self.held);
        held.retain(|d| d.scene_id == id && d.seq > seq);
        if self.holding() {
            self.held = held;
        } else {
            for delta in held {
                self.on_live_delta(delta);
            }
        }
    }

    /// Whether `(scene_id, seq)` is the scene shown, at `seq` or later already.
    fn is_stale(&self, scene_id: &str, seq: u64) -> bool {
        self.cur.as_ref().is_some_and(|(id, cur)| id == scene_id && seq <= *cur)
    }

    /// Whether `(scene_id, seq)` is the scene shown, at a later seq already.
    fn is_older(&self, scene_id: &str, seq: u64) -> bool {
        self.cur.as_ref().is_some_and(|(id, cur)| id == scene_id && seq < *cur)
    }

    fn show(&mut self, doc: SceneDoc) {
        self.cur = Some((doc.scene_id.clone(), doc.seq));
        // Joint and pose frames queued for what this replaces are moot.
        self.out.retain(|event| !matches!(event, SceneEvent::Joints(_) | SceneEvent::Poses(_)));
        self.out.push(SceneEvent::Reset(doc));
    }

    /// A delta, holding it back once a resync is wanted.
    fn on_live_delta(&mut self, delta: SceneDelta) {
        if self.holding() {
            self.hold(delta);
        } else {
            self.on_delta(delta);
        }
    }

    /// A delta, with nothing held back. Returns whether it was applied.
    fn on_delta(&mut self, delta: SceneDelta) -> bool {
        match &self.cur {
            Some((id, cur)) if *id == delta.scene_id => {
                if delta.seq <= *cur {
                    // Stale or a duplicate.
                } else if delta.seq == *cur + 1 {
                    self.cur = Some((delta.scene_id.clone(), delta.seq));
                    self.out.push(SceneEvent::Delta(delta));
                    return true;
                } else {
                    self.resync(delta);
                }
            }
            // An unknown scene: its `Doc` was missed (late join, reconnect).
            _ => self.resync(delta),
        }
        false
    }

    fn resync(&mut self, delta: SceneDelta) {
        self.query_wanted = true;
        if self.query_in_flight {
            self.gap_since_query = true;
        }
        self.hold(delta);
    }

    fn hold(&mut self, delta: SceneDelta) {
        if self.held.len() >= MAX_HELD {
            self.held.remove(0);
        }
        self.held.push(delta);
    }

    /// The snapshot query's reply: `Ok(None)` when there is no scene (yet).
    pub fn on_snapshot(&mut self, reply: Result<Option<SceneDoc>, String>) {
        let superseded = self.doc_since_query;
        let gap_since_query = self.gap_since_query;
        let rejoin = std::mem::take(&mut self.rejoin_in_flight);
        self.query_in_flight = false;
        self.doc_since_query = false;
        self.gap_since_query = false;
        self.query_wanted = false;
        let mut progress = false;
        if let Ok(Some(doc)) = reply {
            let other_scene = self.cur.as_ref().is_some_and(|(id, _)| *id != doc.scene_id);
            // After a re-join, the scene shown at the same seq again: its joints and poses.
            let stale = if rejoin { self.is_older(&doc.scene_id, doc.seq) } else { self.is_stale(&doc.scene_id, doc.seq) };
            if !(other_scene && superseded) && !stale {
                self.show(doc);
                progress = true;
            }
        }
        // Apply what was held against the scene now shown. Deltas of other scenes arrived before
        // the reply, so they are older than it.
        let cur_id = self.cur.as_ref().map(|(id, _)| id.clone());
        for delta in std::mem::take(&mut self.held) {
            if Some(&delta.scene_id) != cur_id.as_ref() {
                continue;
            }
            if self.query_wanted {
                self.hold(delta);
            } else {
                progress |= self.on_delta(delta);
            }
        }
        // A gap remains: ask again if this reply got us further (it raced the deltas), or if the
        // gap was noticed live during the query (the reply may be from before it; once: that next
        // query has no such gap of its own). Otherwise give up on what is held, so a lagging
        // producer can't make us query in a loop. The next live delta asks again.
        if self.query_wanted && !progress && !gap_since_query {
            self.query_wanted = false;
            self.held.clear();
        }
        // A re-join during this query: its reply may predate it, so ask once more (a live doc
        // since would have cleared this).
        if self.rejoin_pending {
            self.query_wanted = true;
        }
    }

    /// A joint frame. Returns whether an event was queued (or merged).
    pub fn on_joints(&mut self, frame: JointFrame) -> bool {
        if !self.cur.as_ref().is_some_and(|(id, _)| *id == frame.scene_id) {
            return false;
        }
        if let Some(SceneEvent::Joints(last)) = self.out.last_mut()
            && last.scene_id == frame.scene_id
        {
            merge_joints(last, frame);
        } else {
            self.out.push(SceneEvent::Joints(frame));
        }
        true
    }

    /// A pose frame, as [`Self::on_joints`]: only for the scene shown, merged into a pose frame
    /// queued last.
    pub fn on_poses(&mut self, frame: PoseFrame) -> bool {
        if !self.cur.as_ref().is_some_and(|(id, _)| *id == frame.scene_id) {
            return false;
        }
        if let Some(SceneEvent::Poses(last)) = self.out.last_mut()
            && last.scene_id == frame.scene_id
        {
            merge_poses(last, frame);
        } else {
            self.out.push(SceneEvent::Poses(frame));
        }
        true
    }
}

/// Folds `next` into `into`: the later pose wins per node.
fn merge_poses(into: &mut PoseFrame, next: PoseFrame) {
    if next.t.is_some() {
        into.t = next.t;
    }
    for pose in next.poses {
        match into.poses.iter_mut().find(|p| p.node == pose.node) {
            Some(slot) => slot.pose = pose.pose,
            None => into.poses.push(pose),
        }
    }
}

/// Folds `next` into `into`: later values win per node and joint.
fn merge_joints(into: &mut JointFrame, next: JointFrame) {
    if next.t.is_some() {
        into.t = next.t;
    }
    for model in next.models {
        match into.models.iter_mut().find(|m| m.node == model.node) {
            Some(slot) => slot.joints.extend(model.joints),
            None => into.models.push(model),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use factoryos_client::protocol::scene::{ModelJoints, NodePose, Pose3};

    use super::*;

    fn doc(id: &str, seq: u64) -> SceneDoc {
        SceneDoc {
            scene_id: id.into(),
            seq,
            ..Default::default()
        }
    }

    fn delta(id: &str, seq: u64) -> SceneMsg {
        SceneMsg::Delta(SceneDelta {
            scene_id: id.into(),
            seq,
            ..Default::default()
        })
    }

    fn joints(id: &str, node: &str, values: &[(&str, f64)]) -> JointFrame {
        JointFrame {
            scene_id: id.into(),
            t: None,
            models: vec![ModelJoints {
                node: node.into(),
                joints: values.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect(),
            }],
        }
    }

    /// `R(id,seq)`, `D(id,seq)`, `J(id)` per event.
    fn summary(seq: &mut Sequencer) -> Vec<String> {
        seq.drain()
            .into_iter()
            .map(|e| match e {
                SceneEvent::Reset(d) => format!("R({},{})", d.scene_id, d.seq),
                SceneEvent::Delta(d) => format!("D({},{})", d.scene_id, d.seq),
                SceneEvent::Joints(j) => format!("J({})", j.scene_id),
                SceneEvent::Poses(p) => format!("P({})", p.scene_id),
            })
            .collect::<Vec<_>>()
    }

    /// A sequencer that has shown `a` at seq 1.
    fn at_a1() -> Sequencer {
        let mut seq = Sequencer::new();
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert_eq!(summary(&mut seq), ["R(a,1)"]);
        seq
    }

    #[test]
    fn live_deltas_wait_for_the_snapshot() {
        let mut seq = Sequencer::new();
        seq.on_msg(delta("a", 2));
        seq.on_msg(delta("a", 3));
        assert!(summary(&mut seq).is_empty());
        seq.on_snapshot(Ok(Some(doc("a", 2))));
        // 2 is in the snapshot already.
        assert_eq!(summary(&mut seq), ["R(a,2)", "D(a,3)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn no_scene_yet_then_a_doc() {
        let mut seq = Sequencer::new();
        seq.on_snapshot(Ok(None));
        assert!(summary(&mut seq).is_empty());
        seq.on_msg(SceneMsg::Doc(doc("a", 0)));
        seq.on_msg(delta("a", 1));
        assert_eq!(summary(&mut seq), ["R(a,0)", "D(a,1)"]);
    }

    #[test]
    fn in_order_stale_and_duplicate_deltas() {
        let mut seq = at_a1();
        seq.on_msg(delta("a", 2));
        seq.on_msg(delta("a", 2));
        seq.on_msg(delta("a", 1));
        seq.on_msg(delta("a", 3));
        assert_eq!(summary(&mut seq), ["D(a,2)", "D(a,3)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn a_gap_resyncs_from_the_snapshot() {
        let mut seq = at_a1();
        seq.on_msg(delta("a", 3));
        seq.on_msg(delta("a", 4));
        seq.on_msg(delta("a", 5));
        assert!(summary(&mut seq).is_empty());
        assert!(seq.take_query_request());
        // Once at a time.
        seq.on_msg(delta("a", 7));
        assert!(!seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("a", 4))));
        assert_eq!(summary(&mut seq), ["R(a,4)", "D(a,5)"]);
        // 7 still does not follow 5: ask again.
        assert!(seq.take_query_request());
        seq.on_msg(delta("a", 8));
        seq.on_snapshot(Ok(Some(doc("a", 7))));
        assert_eq!(summary(&mut seq), ["R(a,7)", "D(a,8)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn an_unknown_scene_resyncs() {
        let mut seq = at_a1();
        seq.on_msg(delta("b", 5));
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("b", 5))));
        seq.on_msg(delta("b", 6));
        assert_eq!(summary(&mut seq), ["R(b,5)", "D(b,6)"]);
    }

    #[test]
    fn a_live_doc_replaces_the_scene() {
        let mut seq = at_a1();
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(delta("b", 1));
        assert_eq!(summary(&mut seq), ["R(b,0)", "D(b,1)"]);
    }

    #[test]
    fn an_older_snapshot_is_ignored() {
        let mut seq = Sequencer::new();
        // The live doc arrives before the initial query's reply, which still has the old scene.
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(delta("b", 1));
        seq.on_snapshot(Ok(Some(doc("a", 9))));
        assert_eq!(summary(&mut seq), ["R(b,0)", "D(b,1)"]);
        // Same scene, older seq.
        let mut seq = at_a1();
        seq.on_msg(delta("a", 2));
        seq.on_msg(delta("a", 9));
        assert!(seq.take_query_request());
        seq.on_msg(delta("a", 10));
        seq.on_snapshot(Ok(Some(doc("a", 2))));
        assert_eq!(summary(&mut seq), ["D(a,2)"]);
        // No progress: what was held is dropped rather than queried for in a loop; the next delta
        // asks again.
        assert!(!seq.take_query_request());
        seq.on_msg(delta("a", 11));
        assert!(seq.take_query_request());
    }

    #[test]
    fn a_gap_noticed_during_the_query_is_asked_about_once_more() {
        let mut seq = at_a1();
        seq.on_msg(delta("a", 3));
        assert!(seq.take_query_request());
        // While it is in flight: a new scene, live, and a gap in it.
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(delta("b", 2));
        assert_eq!(summary(&mut seq), ["R(b,0)"]);
        // The reply is from before `b`: no progress, but the gap in `b` is asked about.
        seq.on_snapshot(Ok(Some(doc("a", 3))));
        assert!(summary(&mut seq).is_empty());
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("b", 2))));
        assert_eq!(summary(&mut seq), ["R(b,2)"]);
        assert!(!seq.take_query_request());

        // Once only: a reply that again doesn't resolve it gives up (no loop).
        let mut seq = at_a1();
        seq.on_msg(delta("a", 3));
        assert!(seq.take_query_request());
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(delta("b", 2));
        seq.on_snapshot(Ok(Some(doc("a", 3))));
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("b", 0))));
        assert!(!seq.take_query_request());
    }

    #[test]
    fn a_stale_live_doc_does_not_resolve_a_gap() {
        let mut seq = at_a1();
        seq.on_msg(delta("a", 2));
        seq.on_msg(delta("a", 4));
        seq.on_msg(SceneMsg::Doc(doc("a", 1)));
        assert_eq!(summary(&mut seq), ["D(a,2)"]);
        assert!(seq.take_query_request());
        // Nor does one during the query stop holding deltas back for the reply.
        seq.on_msg(SceneMsg::Doc(doc("a", 2)));
        seq.on_msg(delta("a", 5));
        seq.on_snapshot(Ok(Some(doc("a", 4))));
        assert_eq!(summary(&mut seq), ["R(a,4)", "D(a,5)"]);
    }

    #[test]
    fn a_failed_query_retries_on_the_next_gap() {
        let mut seq = at_a1();
        seq.on_msg(delta("a", 5));
        assert!(seq.take_query_request());
        seq.on_snapshot(Err("timeout".into()));
        assert!(summary(&mut seq).is_empty());
        assert!(!seq.take_query_request());
        seq.on_msg(delta("a", 6));
        assert!(seq.take_query_request());
    }

    #[test]
    fn a_rejoin_queries_the_snapshot() {
        // The backend restarted with another scene, its doc published before the re-join.
        let mut seq = at_a1();
        seq.on_rejoin();
        // Live deltas wait for the reply.
        seq.on_msg(delta("b", 1));
        assert!(summary(&mut seq).is_empty());
        assert!(seq.take_query_request());
        assert!(!seq.take_query_request(), "once");
        seq.on_snapshot(Ok(Some(doc("b", 0))));
        assert_eq!(summary(&mut seq), ["R(b,0)", "D(b,1)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn after_a_rejoin_the_scene_at_its_seq_is_shown_again() {
        // Its joints and poses, folded into the snapshot, may have moved meanwhile.
        let mut seq = at_a1();
        seq.on_rejoin();
        seq.on_msg(delta("a", 2));
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert_eq!(summary(&mut seq), ["R(a,1)", "D(a,2)"]);
        // But not an older one, nor again without a re-join.
        seq.on_rejoin();
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert!(summary(&mut seq).is_empty());
        seq.on_msg(delta("a", 4));
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("a", 3))));
        assert_eq!(summary(&mut seq), ["R(a,3)", "D(a,4)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn a_rejoin_during_a_query_asks_once_more() {
        // The initial query, in flight when the session re-joins: its reply may predate that.
        let mut seq = Sequencer::new();
        seq.on_rejoin();
        assert!(!seq.take_query_request(), "one at a time");
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert_eq!(summary(&mut seq), ["R(a,1)"]);
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert_eq!(summary(&mut seq), ["R(a,1)"]);
        assert!(!seq.take_query_request(), "no loop");
        // Nor after a failed or empty reply.
        seq.on_rejoin();
        assert!(seq.take_query_request());
        seq.on_snapshot(Err("timeout".into()));
        assert!(!seq.take_query_request());
        seq.on_rejoin();
        assert!(seq.take_query_request());
        seq.on_snapshot(Ok(None));
        assert!(!seq.take_query_request());
        seq.on_msg(delta("a", 2));
        assert_eq!(summary(&mut seq), ["D(a,2)"]);
    }

    #[test]
    fn a_live_doc_after_a_rejoin_answers_it() {
        // Before the query is issued.
        let mut seq = at_a1();
        seq.on_rejoin();
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_msg(delta("b", 1));
        assert_eq!(summary(&mut seq), ["R(b,0)", "D(b,1)"]);
        assert!(!seq.take_query_request());
        // While it is in flight: its reply at that seq is no news.
        let mut seq = at_a1();
        seq.on_rejoin();
        assert!(seq.take_query_request());
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_snapshot(Ok(Some(doc("b", 0))));
        assert_eq!(summary(&mut seq), ["R(b,0)"]);
        assert!(!seq.take_query_request());
        // A re-join during the initial query, then a live doc: no second query.
        let mut seq = Sequencer::new();
        seq.on_rejoin();
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        seq.on_snapshot(Ok(Some(doc("a", 5))));
        assert_eq!(summary(&mut seq), ["R(b,0)"]);
        assert!(!seq.take_query_request());
    }

    #[test]
    fn joints_of_the_scene_shown_only_merged() {
        let mut seq = Sequencer::new();
        assert!(!seq.on_joints(joints("a", "r1", &[("j1", 1.0)])));
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert!(seq.on_joints(joints("a", "r1", &[("j1", 1.0), ("j2", 2.0)])));
        assert!(seq.on_joints(joints("a", "r1", &[("j1", 3.0)])));
        assert!(seq.on_joints(joints("a", "r2", &[("j1", 4.0)])));
        assert!(!seq.on_joints(joints("b", "r1", &[("j1", 5.0)])));
        let events = seq.drain();
        assert_eq!(events.len(), 2);
        let SceneEvent::Joints(frame) = &events[1] else { panic!("{events:?}") };
        assert_eq!(frame.models.len(), 2);
        assert_eq!(frame.models[0].joints, BTreeMap::from([("j1".into(), 3.0), ("j2".into(), 2.0)]));
        assert_eq!(frame.models[1].joints, BTreeMap::from([("j1".into(), 4.0)]));
        // A delta in between is not merged across.
        seq.on_joints(joints("a", "r1", &[("j1", 1.0)]));
        seq.on_msg(delta("a", 2));
        seq.on_joints(joints("a", "r1", &[("j1", 2.0)]));
        assert_eq!(summary(&mut seq), ["J(a)", "D(a,2)", "J(a)"]);
    }

    #[test]
    fn a_reset_drops_queued_joints() {
        let mut seq = at_a1();
        seq.on_joints(joints("a", "r1", &[("j1", 1.0)]));
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        assert_eq!(summary(&mut seq), ["R(b,0)"]);
    }

    fn poses(id: &str, node: &str, x: f64) -> PoseFrame {
        PoseFrame {
            scene_id: id.into(),
            t: None,
            poses: vec![NodePose { node: node.into(), pose: Pose3::from_xyz([x, 0.0, 0.0]) }],
        }
    }

    #[test]
    fn poses_of_the_scene_shown_only_merged() {
        let mut seq = Sequencer::new();
        assert!(!seq.on_poses(poses("a", "p1", 1.0)), "no scene yet");
        seq.on_snapshot(Ok(Some(doc("a", 1))));
        assert!(seq.on_poses(poses("a", "p1", 1.0)));
        assert!(seq.on_poses(poses("a", "p2", 2.0)));
        assert!(seq.on_poses(poses("a", "p1", 3.0)));
        assert!(!seq.on_poses(poses("b", "p1", 5.0)));
        let events = seq.drain();
        assert_eq!(events.len(), 2);
        let SceneEvent::Poses(frame) = &events[1] else { panic!("{events:?}") };
        let xs: Vec<_> = frame.poses.iter().map(|p| (p.node.as_str(), p.pose.xyz[0])).collect();
        assert_eq!(xs, [("p1", 3.0), ("p2", 2.0)], "the later pose wins per node");
        // Not merged across a delta or a joint frame.
        seq.on_poses(poses("a", "p1", 1.0));
        seq.on_joints(joints("a", "r1", &[("j1", 1.0)]));
        seq.on_poses(poses("a", "p1", 2.0));
        seq.on_msg(delta("a", 2));
        seq.on_poses(poses("a", "p1", 3.0));
        assert_eq!(summary(&mut seq), ["P(a)", "J(a)", "P(a)", "D(a,2)", "P(a)"]);
        // A reset drops queued ones.
        seq.on_poses(poses("a", "p1", 4.0));
        seq.on_msg(SceneMsg::Doc(doc("b", 0)));
        assert_eq!(summary(&mut seq), ["R(b,0)"]);
    }
}
