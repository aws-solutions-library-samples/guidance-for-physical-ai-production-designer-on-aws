//! Robo Fitter reach: the task's welds in the scene, each robot's reach dome, and the
//! reachability check (the dome estimate, then the cell planner's IK).
//!
//! * [`task`]: the welds, from the products placed as fixtures or the linked cell's program.
//! * [`kinematics`], [`envelope`], [`tcp`]: the robot's chain, its wrist envelope (the dome) and
//!   its TCP, from the dressed URDF.
//! * [`robot_info`]: what a dressed robot is (joints, dome, tool, placement figures), for the agent.
//! * [`ik`]: the planner's IK on every robot × weld (`deploy/planner-router/reach_check.py`).
//! * [`suggest`]: robot moves (and pedestals) that bring unreached welds into the domes.
//! * [`draw`]: the 3D entities (`world/task/*`, `world/reach/*`).
//! * [`http`]: the stateless `/api/v1/reach/*` routes for the agent.

pub mod comfort;
pub mod draw;
pub mod envelope;
pub mod http;
pub mod ik;
pub mod kinematics;
pub mod robot_info;
pub mod suggest;
pub mod task;
pub mod tcp;

use std::collections::{BTreeMap, HashMap};
use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use factoryos_protocol::fitter::{
    ReachAttempt, ReachMode, ReachReason, ReachReport, SceneFile, TaskSource, TaskWeld, ToolTcp, WeldEstimate, WeldReach,
    WeldStatus,
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use crate::config_store::ConfigStore;
use crate::services::tcp::ToolCatalogues;
use envelope::{Envelope, Side};
use ik::{IkEvent, IkRobot, IkSetup, IkStop, IkWeld};
use task::{ProductBox, SceneTask};

/// Welds further than this outside the dome are not sent to the IK.
pub const PREFILTER_M: f64 = 0.05;
/// A weld this far inside a product (or the tool line this deep) collides.
const PRODUCT_TOLERANCE_M: f64 = 0.005;
pub const IK_TIMEOUT: Duration = Duration::from_secs(600);

pub const NOTE_FIXTURES: &str =
    "the planner IK does not check fixtures: welds and the tool's line are checked against the products' boxes only";

/// A robot model: chain, dome and TCP of one dressed URDF.
pub struct RobotKind {
    pub chain: kinematics::Chain,
    pub envelope: Option<Arc<Envelope>>,
    pub tcp: ToolTcp,
    /// Wrist centre to TCP, metres.
    pub tool_length: f64,
}

/// A robot of the scene.
#[derive(Clone)]
pub struct SceneRobot {
    pub id: String,
    /// The label used in the 3D view (`r1`, `r2`, …: scene order).
    pub short: String,
    pub urdf: PathBuf,
    pub kind: Option<Arc<RobotKind>>,
    /// The URDF root's pose in the scene frame (pedestal included).
    pub base: Isometry3<f64>,
    /// The pedestal it stands on (`pedestal.height_mm`; 0 without one).
    pub pedestal_mm: f64,
    pub yaw_deg: f64,
    pub tilt_bc_deg: [f64; 2],
    pub home_deg: Vec<f64>,
    pub error: Option<String>,
}

impl SceneRobot {
    pub fn tool_length(&self) -> f64 {
        self.kind.as_ref().map_or(0.0, |k| k.tool_length)
    }

    /// The dome margin of `weld` and where it is; `None` without a dome.
    pub fn margin(&self, weld: &TaskWeld) -> Option<(f64, Side)> {
        let kind = self.kind.as_ref()?;
        Some(kind.envelope.as_ref()?.margin(&self.base, kind.tool_length, weld.position, weld.approach))
    }
}

/// Shared by the Robo Fitter and the agent's routes: the robot models (cached per URDF) and where
/// the IK runs (from the config).
pub struct ReachContext {
    config: Option<Arc<ConfigStore>>,
    fixed: Option<(ToolCatalogues, IkSetup)>,
    cache: Mutex<HashMap<PathBuf, Cached>>,
}

/// A robot model and the URDF's modification time it was read at.
type Cached = (Option<SystemTime>, Arc<RobotKind>);

impl ReachContext {
    pub fn new(config: Arc<ConfigStore>) -> Self {
        Self { config: Some(config), fixed: None, cache: Mutex::default() }
    }

    /// Without a config (tests).
    pub fn with(tools: ToolCatalogues, ik: IkSetup) -> Self {
        Self { config: None, fixed: Some((tools, ik)), cache: Mutex::default() }
    }

    pub fn tools(&self) -> ToolCatalogues {
        match (&self.fixed, &self.config) {
            (Some((tools, _)), _) => tools.clone(),
            (None, Some(config)) => ToolCatalogues::from_config(config),
            (None, None) => ToolCatalogues::default(),
        }
    }

    pub fn ik(&self) -> IkSetup {
        match (&self.fixed, &self.config) {
            (Some((_, ik)), _) => ik.clone(),
            (None, Some(store)) => {
                let config = store.get();
                let path = |p: &str| PathBuf::from(store.resolve(p));
                IkSetup {
                    python: path(config.cell_planner_python_or_default()),
                    script: path(config.reach_check_script_or_default()),
                    planner_src: path(config.cell_planner_src_or_default()),
                    timeout: IK_TIMEOUT,
                }
            }
            (None, None) => IkSetup { python: "".into(), script: "".into(), planner_src: "".into(), timeout: IK_TIMEOUT },
        }
    }

    /// The model of the dressed URDF `urdf` (re-read when the file changes).
    pub fn robot_kind(&self, urdf: &Path) -> anyhow::Result<Arc<RobotKind>> {
        let modified = std::fs::metadata(urdf).and_then(|m| m.modified()).ok();
        if let Some((at, kind)) = self.cache.lock().unwrap().get(urdf)
            && *at == modified
        {
            return Ok(kind.clone());
        }
        let robot = urdf_rs::read_file(urdf).map_err(|err| anyhow::anyhow!("{}: {err}", urdf.display()))?;
        let chain = kinematics::Chain::from_urdf(&robot)?;
        let tcp = tcp::resolve(urdf, &robot, &self.tools());
        let envelope = Envelope::compute(&chain).map(Arc::new);
        let tool_length = tool_length(&chain, &tcp);
        let kind = Arc::new(RobotKind { chain, envelope, tcp, tool_length });
        self.cache.lock().unwrap().insert(urdf.to_owned(), (modified, kind.clone()));
        Ok(kind)
    }

    /// The scene's robots, placed (scene frame).
    pub fn scene_robots(&self, scene: &SceneFile, scene_base: &Path) -> Vec<SceneRobot> {
        scene
            .robots
            .iter()
            .enumerate()
            .map(|(i, robot)| {
                let urdf = scene_base.join(&robot.dressed_urdf);
                let lift = robot.pedestal.as_ref().map_or(0.0, |p| p.height_mm / 1000.0);
                let o = robot.orientation;
                let base = Isometry3::from_parts(
                    Translation3::new(robot.position.x, robot.position.y, robot.position.z + lift),
                    UnitQuaternion::from_euler_angles(o.c.to_radians(), o.b.to_radians(), o.a.to_radians()),
                );
                let (kind, error) = match self.robot_kind(&urdf) {
                    Ok(kind) => (Some(kind), None),
                    Err(err) => (None, Some(format!("{err:#}"))),
                };
                let dof = kind.as_ref().map_or(6, |k| k.chain.dof());
                SceneRobot {
                    id: robot.robot_id.clone(),
                    short: format!("r{}", i + 1),
                    urdf,
                    kind,
                    base,
                    pedestal_mm: robot.pedestal.as_ref().map_or(0.0, |p| p.height_mm.max(0.0)),
                    yaw_deg: o.a,
                    tilt_bc_deg: [o.b, o.c],
                    home_deg: robot.home_pose_deg.clone().unwrap_or_else(|| {
                        let mut pose = vec![0.0, -90.0, 90.0, 0.0, 0.0, 0.0];
                        pose.resize(dof, 0.0);
                        pose
                    }),
                    error,
                }
            })
            .collect()
    }
}

/// Wrist centre to the TCP, at the zero pose.
fn tool_length(chain: &kinematics::Chain, tcp: &ToolTcp) -> f64 {
    let q = vec![0.0; chain.dof()];
    let Some(wrist) = chain.actuated_frame(&q, 4) else {
        return Vector3::from(tcp.offset).norm();
    };
    let tip = chain.fk(&q) * Point3::from(tcp.offset);
    (tip - Point3::from(wrist.translation.vector)).norm()
}

/// What the Robo Fitter shows for a scene, without the IK.
pub struct Analysis {
    pub source: TaskSource,
    pub welds: Vec<TaskWeld>,
    pub notes: Vec<String>,
    pub robots: Vec<SceneRobot>,
    pub task: SceneTask,
    pub estimate: Vec<WeldEstimate>,
    pub scene_rev: u64,
}

impl Analysis {
    pub fn tcp(&self) -> BTreeMap<String, ToolTcp> {
        self.robots.iter().filter_map(|r| Some((r.id.clone(), r.kind.as_ref()?.tcp.clone()))).collect()
    }

    pub fn boxes(&self) -> Vec<&ProductBox> {
        self.task.boxes()
    }

    /// The approach most welds share (their mean, normalised): the one the domes are drawn for.
    pub fn dominant_approach(&self) -> [f64; 3] {
        let sum: Vector3<f64> = self.welds.iter().map(|w| Vector3::from(w.approach)).sum();
        if sum.norm() < 1e-6 { [0.0, 0.0, -1.0] } else { (sum / sum.norm()).into() }
    }
}

/// The task, robots and dome estimate of `scene` (paths from `scene_base`; `cell_root`: the
/// linked cell).
pub fn analyse(ctx: &ReachContext, scene: &SceneFile, scene_base: &Path, cell_root: Option<&Path>, source: TaskSource) -> Analysis {
    let task = SceneTask::load(scene, scene_base, cell_root);
    let (source, welds, mut notes) = task.welds(source);
    let robots = ctx.scene_robots(scene, scene_base);
    for robot in &robots {
        if let Some(err) = &robot.error {
            notes.push(format!("{}: {err}", robot.id));
        } else if robot.kind.as_ref().is_some_and(|k| k.envelope.is_none()) {
            notes.push(format!("{}: no reach dome (needs a 6-axis arm with a vertical A1)", robot.id));
        }
        if let Some(kind) = &robot.kind
            && kind.tcp.source == "estimated"
        {
            notes.push(format!("{}: TCP estimated from the tool's meshes ({:?}); declare it in the tool's URDF", robot.id, kind.tcp.offset));
        }
    }
    let boxes = task.boxes();
    let estimate = welds.iter().map(|w| estimate_weld(w, &robots, &boxes)).collect();
    let scene_rev = scene_rev(scene, source, &welds);
    Analysis { source, welds, notes, robots, task, estimate, scene_rev }
}

/// The dome estimate of one weld.
pub fn estimate_weld(weld: &TaskWeld, robots: &[SceneRobot], boxes: &[&ProductBox]) -> WeldEstimate {
    let mut margins = BTreeMap::new();
    let mut reached = Vec::new();
    for robot in robots {
        let Some((margin, _)) = robot.margin(weld) else { continue };
        margins.insert(robot.id.clone(), margin);
        if margin >= 0.0 && product_collision(boxes, weld, robot.tool_length()).is_empty() {
            reached.push(robot.id.clone());
        }
    }
    WeldEstimate { weld_id: weld.id.clone(), status: classify(robots.len(), reached.len()), robots: reached, margins_m: margins }
}

/// Green: at least two robots (or the only one); orange: one of several; red: none; grey: no robots.
pub fn classify(robots: usize, reached: usize) -> WeldStatus {
    match (robots, reached) {
        (0, _) => WeldStatus::Unknown,
        (_, 0) => WeldStatus::NoRobot,
        (1, _) => WeldStatus::AllRobots,
        (_, 1) => WeldStatus::OneRobot,
        _ => WeldStatus::AllRobots,
    }
}

/// A hash of the scene, the resolved task source and its welds: a report of another revision is stale.
pub fn scene_rev(scene: &SceneFile, source: TaskSource, welds: &[TaskWeld]) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    serde_json::to_string(scene).unwrap_or_default().hash(&mut hasher);
    format!("{source:?}").hash(&mut hasher);
    serde_json::to_string(welds).unwrap_or_default().hash(&mut hasher);
    hasher.finish()
}

/// The product collisions of `weld` for a tool of length `l` along its approach: the weld more
/// than 5 mm inside a product, or the tool's line (TCP back to the wrist) entering one.
pub fn product_collision(boxes: &[&ProductBox], weld: &TaskWeld, l: f64) -> Vec<ReachReason> {
    if boxes.iter().any(|b| b.signed_distance(weld.position) < -PRODUCT_TOLERANCE_M) {
        return vec![ReachReason::WeldInsideProduct];
    }
    let steps = ((l / 0.02).ceil() as usize).max(1);
    let hits = (1..=steps).any(|i| {
        let s = l * i as f64 / steps as f64;
        let p = [0, 1, 2].map(|k| weld.position[k] - weld.approach[k] * s);
        boxes.iter().any(|b| b.signed_distance(p) < -PRODUCT_TOLERANCE_M)
    });
    if hits { vec![ReachReason::ToolHitsProduct] } else { Vec::new() }
}

/// What to check.
#[derive(Clone, Debug, Default)]
pub struct CheckOptions {
    pub source: TaskSource,
    pub mode: ReachMode,
    /// Only these robots / welds (empty: all).
    pub robots: Vec<String>,
    pub welds: Vec<String>,
    /// Without the IK (the dome estimate only).
    pub estimate_only: bool,
}

/// An attempt from the dome alone.
fn dome_attempt(robot: &SceneRobot, weld: &TaskWeld, boxes: &[&ProductBox]) -> ReachAttempt {
    let mut attempt = ReachAttempt { robot_id: robot.id.clone(), ..Default::default() };
    match robot.margin(weld) {
        None => {
            attempt.reasons.push(ReachReason::Error);
            attempt.detail = robot.error.clone().unwrap_or_else(|| "no reach dome for this robot".to_owned());
        }
        Some((margin, side)) => {
            attempt.margin_m = margin;
            if margin < 0.0 {
                attempt.reasons.push(if side == Side::InnerVoid { ReachReason::InsideInnerVoid } else { ReachReason::OutOfReach });
                attempt.detail = format!("{:.3} m outside the dome", -margin);
            } else {
                attempt.detail = format!("{margin:.3} m inside the dome");
            }
        }
    }
    attempt.reasons.extend(product_collision(boxes, weld, robot.tool_length()));
    attempt.reachable = attempt.reasons.is_empty();
    attempt
}

/// Fill in `reached_by` and `status` of every weld.
pub fn summarise(report: &mut ReachReport, robots: usize) {
    for weld in &mut report.welds {
        weld.reached_by = weld.attempts.iter().filter(|a| a.reachable).map(|a| a.robot_id.clone()).collect();
        weld.status = classify(robots, weld.reached_by.len());
    }
}

/// Check `scene`: every robot on every weld of the task, with the cell planner's IK (or the dome
/// estimate when that is unavailable), streaming the report as it fills to `progress`.
pub fn check_scene(
    ctx: &ReachContext,
    scene: &SceneFile,
    scene_base: &Path,
    cell_root: Option<&Path>,
    options: &CheckOptions,
    mut progress: impl FnMut(&ReachReport),
    cancel: &AtomicBool,
) -> ReachReport {
    let started = Instant::now();
    let analysis = analyse(ctx, scene, scene_base, cell_root, options.source);
    let robots: Vec<&SceneRobot> =
        analysis.robots.iter().filter(|r| options.robots.is_empty() || options.robots.contains(&r.id)).collect();
    let welds: Vec<&TaskWeld> = analysis.welds.iter().filter(|w| options.welds.is_empty() || options.welds.contains(&w.id)).collect();
    let boxes = analysis.boxes();
    let ik = ctx.ik();
    let unavailable = if options.estimate_only { Some("the dome estimate only".to_owned()) } else { ik.unavailable() };
    let mut report = ReachReport {
        scene_rev: analysis.scene_rev,
        source: analysis.source,
        mode: options.mode,
        method: if unavailable.is_some() { "estimate" } else { "ik" }.to_owned(),
        running: true,
        total: welds.len() * robots.len(),
        notes: analysis.notes.clone(),
        ..Default::default()
    };
    if let Some(why) = &unavailable
        && !options.estimate_only
    {
        report.notes.push(why.clone());
    }
    if !boxes.is_empty() {
        report.notes.push(NOTE_FIXTURES.to_owned());
    }
    // Every pair starts from the dome; the IK replaces the ones it checks.
    let mut pending: Vec<(usize, usize)> = Vec::new(); // (weld, robot) indices sent to the IK
    for (wi, weld) in welds.iter().enumerate() {
        let mut attempts = Vec::new();
        for (ri, robot) in robots.iter().enumerate() {
            let mut attempt = dome_attempt(robot, weld, &boxes);
            let tilted = robot.tilt_bc_deg.iter().any(|v| v.abs() > 1e-6);
            let far = attempt.margin_m < -PREFILTER_M || attempt.reasons.contains(&ReachReason::Error);
            let collides = attempt.reasons.iter().any(|r| matches!(r, ReachReason::WeldInsideProduct | ReachReason::ToolHitsProduct));
            if unavailable.is_none() {
                if tilted {
                    attempt.reachable = false;
                    attempt.reasons.push(ReachReason::BaseTiltUnsupported);
                    attempt.detail = "the IK takes upright bases only (orientation b, c = 0)".to_owned();
                } else if far {
                    attempt.detail = format!("{}; not sent to the IK", attempt.detail);
                } else if collides {
                    attempt.detail = format!("{}; collides with the product, not sent to the IK", attempt.detail);
                } else {
                    attempt.reachable = false;
                    attempt.detail = "checking…".to_owned();
                    pending.push((wi, ri));
                }
            }
            attempts.push(attempt);
        }
        report.welds.push(WeldReach { weld: (*weld).clone(), attempts, ..Default::default() });
    }
    report.done = report.total - pending.len();
    summarise(&mut report, robots.len());
    progress(&report);

    if !pending.is_empty() {
        let origin = scene.scene_origin.unwrap_or_default();
        let shift = |p: [f64; 3]| [p[0] - origin.x, p[1] - origin.y, p[2] - origin.z];
        let jobs: Vec<(IkRobot, Vec<IkWeld>)> = robots
            .iter()
            .enumerate()
            .filter_map(|(ri, robot)| {
                let kind = robot.kind.as_ref()?;
                let welds: Vec<IkWeld> = pending
                    .iter()
                    .filter(|(_, r)| *r == ri)
                    .map(|(wi, _)| IkWeld { id: welds[*wi].id.clone(), position: shift(welds[*wi].position), approach: welds[*wi].approach })
                    .collect();
                let t = robot.base.translation.vector;
                Some((
                    IkRobot {
                        id: robot.id.clone(),
                        urdf: std::fs::canonicalize(&robot.urdf).unwrap_or_else(|_| robot.urdf.clone()),
                        base_xyz: shift([t.x, t.y, t.z]),
                        yaw_deg: robot.yaw_deg,
                        tilt_bc_deg: robot.tilt_bc_deg,
                        home_deg: robot.home_deg.clone(),
                        tcp: (&kind.tcp).into(),
                    },
                    welds,
                ))
            })
            .collect();
        let robot_index = |id: &str| robots.iter().position(|r| r.id == id);
        let weld_index = |id: &str| welds.iter().position(|w| w.id == id);
        let mut checked = std::collections::HashSet::new();
        let mut failures = Vec::new();
        let result = ik::run(
            &ik,
            options.mode,
            -origin.z,
            &jobs,
            |event| {
                match event {
                    IkEvent::Pair(pair) => {
                        let (Some(wi), Some(ri)) = (weld_index(&pair.weld), robot_index(&pair.robot)) else { return };
                        if !checked.insert((wi, ri)) {
                            return;
                        }
                        let attempt = &mut report.welds[wi].attempts[ri];
                        attempt.reachable = pair.reachable;
                        attempt.joints_deg = pair.q_deg;
                        attempt.pos_err_m = pair.pos_m;
                        attempt.axis_err_deg = pair.axis_deg;
                        attempt.reasons = pair.reasons;
                        // No configuration for a weld outside the dome: it is out of reach.
                        if attempt.reasons == [ReachReason::NoSolution] && attempt.margin_m < 0.0 {
                            attempt.reasons = vec![ReachReason::OutOfReach];
                        }
                        attempt.detail = pair.detail;
                        report.done += 1;
                    }
                    IkEvent::Failed { robot, error } => {
                        failures.push(format!("IK failed for {robot}: {error}"));
                        if let Some(ri) = robot_index(&robot) {
                            for (wi, r) in &pending {
                                if *r == ri && checked.insert((*wi, ri)) {
                                    let attempt = &mut report.welds[*wi].attempts[ri];
                                    attempt.reasons = vec![ReachReason::Error];
                                    attempt.detail = format!("IK failed: {error}");
                                    report.done += 1;
                                }
                            }
                        }
                    }
                }
                summarise(&mut report, robots.len());
                report.elapsed_s = started.elapsed().as_secs_f64();
                progress(&report);
            },
            cancel,
        );
        let stopped = match result {
            Ok(()) => None,
            Err(IkStop::Cancelled) => Some("cancelled".to_owned()),
            Err(IkStop::TimedOut) => Some(format!("timed out after {} s", ik.timeout.as_secs())),
        };
        for (wi, ri) in &pending {
            if !checked.contains(&(*wi, *ri)) {
                let attempt = &mut report.welds[*wi].attempts[*ri];
                attempt.reasons = vec![ReachReason::Error];
                attempt.detail = format!("not checked ({})", stopped.as_deref().unwrap_or("no result from the IK"));
            }
        }
        if !failures.is_empty() && failures.len() >= jobs.iter().filter(|(_, w)| !w.is_empty()).count() {
            report.error = Some(failures.join("; "));
        } else {
            report.notes.extend(failures);
        }
        if let Some(stopped) = stopped {
            report.error = Some(match report.error.take() {
                Some(err) => format!("{stopped}; {err}"),
                None => stopped,
            });
        }
    }
    summarise(&mut report, robots.len());
    report.suggestions = suggest::suggest(&analysis.robots, &report, &boxes, 1.0).into_iter().map(|m| m.suggestion).collect();
    report.running = false;
    report.done = report.total;
    report.elapsed_s = started.elapsed().as_secs_f64();
    progress(&report);
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    pub fn ctx(ik: IkSetup) -> ReachContext {
        ReachContext::with(ToolCatalogues::default(), ik)
    }

    pub fn no_ik() -> IkSetup {
        IkSetup { python: "/nonexistent/python".into(), script: "/nonexistent.py".into(), planner_src: "/nonexistent".into(), timeout: IK_TIMEOUT }
    }

    fn base() -> PathBuf {
        task::tests::fixtures().join("scenes/jam2")
    }

    #[test]
    fn the_jam2_estimate() {
        let a = analyse(&ctx(no_ik()), &task::tests::jam2(), &base(), None, TaskSource::Product);
        assert_eq!(a.welds.len(), 4);
        let (r1, r2) = ("kr240r2900__sticktool", "kr240r2900__sticktool_2");
        let est: BTreeMap<&str, &WeldEstimate> = a.estimate.iter().map(|e| (e.weld_id.as_str(), e)).collect();
        // r1 (x = 2.5) reaches the near welds W02 and W04; W01 and W03 are 57 mm out. r2 mirrors it.
        for (weld, near, far) in [("W01", r2, r1), ("W02", r1, r2), ("W03", r2, r1), ("W04", r1, r2)] {
            let e = est[format!("bug_cube/{weld}").as_str()];
            assert_eq!(e.robots, [near], "{weld}: {e:?}");
            assert_eq!(e.status, WeldStatus::OneRobot);
            assert!((e.margins_m[far] + 0.057).abs() < 0.02, "{weld}: {:?}", e.margins_m);
            assert!(e.margins_m[near] > 0.3, "{weld}: {:?}", e.margins_m);
        }
        assert_eq!(a.tcp()[r1].source, "table:sticktool");
        assert!((a.robots[0].tool_length() - 0.9092).abs() < 1e-3);
        assert_eq!(a.dominant_approach(), [0.0, 0.0, -1.0]);
        let rev = a.scene_rev;
        let mut moved = task::tests::jam2();
        moved.fixtures[0].position.x = 0.1;
        let b = analyse(&ctx(no_ik()), &moved, &base(), None, TaskSource::Product);
        assert_ne!(rev, b.scene_rev);
        assert!((b.welds[1].position[0] - 0.6).abs() < 1e-9, "the welds follow the product");
    }

    #[test]
    fn statuses() {
        assert_eq!(classify(0, 0), WeldStatus::Unknown);
        assert_eq!(classify(1, 1), WeldStatus::AllRobots);
        assert_eq!(classify(1, 0), WeldStatus::NoRobot);
        assert_eq!(classify(2, 1), WeldStatus::OneRobot);
        assert_eq!(classify(3, 2), WeldStatus::AllRobots);
    }

    #[test]
    fn without_the_ik_the_check_is_the_estimate() {
        let mut reports = 0;
        let options = CheckOptions { source: TaskSource::Product, ..Default::default() };
        let r = check_scene(&ctx(no_ik()), &task::tests::jam2(), &base(), None, &options, |_| reports += 1, &AtomicBool::new(false));
        assert_eq!(r.method, "estimate");
        assert!(r.notes.iter().any(|n| n.starts_with("IK unavailable: ")), "{:?}", r.notes);
        assert!(r.notes.iter().any(|n| n == NOTE_FIXTURES));
        assert_eq!((r.done, r.total, r.running), (8, 8, false));
        assert!(r.welds.iter().all(|w| w.status == WeldStatus::OneRobot), "{:?}", r.welds);
        assert!(reports >= 2);
        // The suggestion moves each robot towards its far welds.
        assert_eq!(r.suggestions.len(), 2, "{:?}", r.suggestions);
    }

    #[test]
    fn the_cell_program_on_the_walls() {
        let options = CheckOptions { source: TaskSource::CellProgram, ..Default::default() };
        let cell = task::tests::fixtures().join("walls");
        let r = check_scene(&ctx(no_ik()), &task::tests::jam2(), &base(), Some(&cell), &options, |_| {}, &AtomicBool::new(false));
        assert!(r.notes.iter().any(|n| n.contains("program does not match product")), "{:?}", r.notes);
        for w in &r.welds {
            let inside = w.attempts.iter().all(|a| a.reasons.contains(&ReachReason::WeldInsideProduct));
            let wall = ["W07", "W08", "W09", "W10", "W11", "W12"].contains(&w.weld.id.as_str());
            assert_eq!(inside, wall, "{}: {:?}", w.weld.id, w.attempts);
            if wall {
                assert_eq!(w.status, WeldStatus::NoRobot);
            }
        }
    }

    /// jam2 with the cell planner's IK: `FOS_PLANNER_PYTHON=<venv>/bin/python FOS_PLANNER_SRC=<src>
    /// cargo test -p factoryos-backend --bin factoryos-backend jam2_with_the_planner_ik -- --ignored`.
    #[test]
    #[ignore = "needs the cell planner (FOS_PLANNER_PYTHON, FOS_PLANNER_SRC)"]
    fn jam2_with_the_planner_ik() {
        let (Ok(python), Ok(src)) = (std::env::var("FOS_PLANNER_PYTHON"), std::env::var("FOS_PLANNER_SRC")) else {
            panic!("set FOS_PLANNER_PYTHON and FOS_PLANNER_SRC");
        };
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let setup = IkSetup {
            python: python.into(),
            script: repo.join("deploy/planner-router/reach_check.py"),
            planner_src: src.into(),
            timeout: IK_TIMEOUT,
        };
        // The release catalogue's dressed robot, with its meshes and collision hulls.
        let mut scene = task::tests::jam2();
        let urdf = std::fs::canonicalize(repo.join("assets/catalogue/urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf")).unwrap();
        for robot in &mut scene.robots {
            robot.dressed_urdf = urdf.display().to_string();
        }
        let options = CheckOptions { source: TaskSource::Product, ..Default::default() };
        let r = check_scene(&ctx(setup), &scene, &base(), None, &options, |_| {}, &AtomicBool::new(false));
        assert_eq!((r.method.as_str(), r.error.as_deref()), ("ik", None), "{:?}", r.notes);
        let by: BTreeMap<&str, &Vec<String>> = r.welds.iter().map(|w| (w.weld.label.as_str(), &w.reached_by)).collect();
        let (r1, r2) = ("kr240r2900__sticktool".to_owned(), "kr240r2900__sticktool_2".to_owned());
        assert_eq!(by["W02"], &vec![r1.clone()]);
        assert_eq!(by["W04"], &vec![r1]);
        assert_eq!(by["W01"], &vec![r2.clone()]);
        assert_eq!(by["W03"], &vec![r2]);
        assert!(r.welds.iter().all(|w| w.status == WeldStatus::OneRobot));
        let w02 = &r.welds.iter().find(|w| w.weld.label == "W02").unwrap().attempts[0];
        assert_eq!(w02.joints_deg.len(), 6);
        assert!(w02.pos_err_m.unwrap() < 0.01);
        // The planner's joints (degrees, in URDF chain order) put the backend's TCP on the weld
        // with the tool pointing along the approach: the joints the Fitter shows are the IK's.
        let a = analyse(&ctx(no_ik()), &scene, &base(), None, TaskSource::Product);
        for w in &r.welds {
            for attempt in w.attempts.iter().filter(|a| a.reachable) {
                let robot = a.robots.iter().find(|r| r.id == attempt.robot_id).unwrap();
                let kind = robot.kind.as_ref().unwrap();
                let q: Vec<f64> = attempt.joints_deg.iter().map(|d| d.to_radians()).collect();
                let flange = robot.base * kind.chain.fk(&q);
                let tcp = flange * Point3::from(kind.tcp.offset);
                let err = (tcp - Point3::from(w.weld.position)).norm();
                assert!(err < 0.011, "{} by {}: TCP {:?} is {err:.4} m from the weld", w.weld.id, attempt.robot_id, tcp);
                let axis = flange.rotation * Vector3::from(kind.tcp.axis);
                let angle = axis.angle(&Vector3::from(w.weld.approach)).to_degrees();
                assert!(angle < 5.5, "{} by {}: the tool is {angle:.1}° off the approach", w.weld.id, attempt.robot_id);
            }
        }
    }

    #[test]
    fn the_ik_runs_per_robot_and_streams() {
        let dir = std::env::temp_dir().join(format!("factoryos-reach-check-{}", std::process::id()));
        // The stub reaches welds with x > 0 (scene frame): W02 and W04.
        let setup = ik::tests::stub(&dir, 0.0);
        let options = CheckOptions { source: TaskSource::Product, mode: ReachMode::Full, ..Default::default() };
        let mut done = Vec::new();
        let r = check_scene(&ctx(setup), &task::tests::jam2(), &base(), None, &options, |r| done.push(r.done), &AtomicBool::new(false));
        assert_eq!(r.method, "ik");
        assert_eq!(r.error, None);
        // 4 pairs are > 5 cm inside the dome and go to the IK; the far ones (57 mm out) are pre-filtered.
        assert_eq!(done.first(), Some(&4), "{done:?}");
        assert_eq!(done.last(), Some(&8));
        let w02 = r.welds.iter().find(|w| w.weld.id == "bug_cube/W02").unwrap();
        assert_eq!(w02.reached_by, ["kr240r2900__sticktool"]);
        assert_eq!(w02.attempts[0].joints_deg, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let w01 = r.welds.iter().find(|w| w.weld.id == "bug_cube/W01").unwrap();
        assert_eq!(w01.status, WeldStatus::NoRobot, "the stub fails x < 0: {w01:?}");
        assert_eq!(w01.attempts[0].reasons, [ReachReason::OutOfReach], "r1 pre-filtered");
        assert_eq!(w01.attempts[1].reasons, [ReachReason::SelfCollision, ReachReason::JointLimits]);

        let cancel = AtomicBool::new(false);
        let slow = ik::tests::stub(&dir.join("slow"), 30.0);
        let r = check_scene(&ctx(slow), &task::tests::jam2(), &base(), None, &options, |_| cancel.store(true, Ordering::Relaxed), &cancel);
        assert_eq!(r.error.as_deref(), Some("cancelled"));
        assert!(r.welds.iter().flat_map(|w| &w.attempts).any(|a| a.detail == "not checked (cancelled)"));
        std::fs::remove_dir_all(&dir).ok();
    }
}

#[cfg(test)]
pub(crate) mod tests_support {
    use super::*;

    pub fn ctx() -> ReachContext {
        ReachContext::with(
            ToolCatalogues::default(),
            IkSetup { python: "/nonexistent/python".into(), script: "/nonexistent.py".into(), planner_src: "/nonexistent".into(), timeout: IK_TIMEOUT },
        )
    }
}
