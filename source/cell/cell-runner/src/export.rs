//! Headless trajectory export mode.
//!
//! This module orchestrates a deterministic, headless run of a robotic cell and
//! serializes every resolved motion for every robot into a single combined JSON
//! file. Implementation is added in subsequent tasks.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use chrono::NaiveDateTime;
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

use crate::bridge;
use crate::cell::{CellSimulator, RobotState, SignalValue};
use crate::interpreter::{Instruction, KrlInterpreter};
use crate::krl::{RobotKrl, StepStatus};
use crate::path::JointPath;
use crate::traj::{
    LayeredTrajectoryProvider, MotionKind, MotionRequest, SolverTier, TrajectoryProvider,
};

/// Positive floor (seconds) substituted for a resolved `duration_s <= 0`
/// (Req 11.2). Every emitted `duration` is therefore `>= MIN_DURATION_FLOOR`
/// and strictly greater than `0` (Req 11.3).
const MIN_DURATION_FLOOR: f64 = 0.001;

/// Inclusive absolute bound for each `base_position` component, in meters
/// (Req 9.3): every value must lie within `[-1e6, 1e6]`.
const BASE_POSITION_ABS_MAX: f64 = 1_000_000.0;

/// Inclusive upper bound (seconds) for every `start_time`, `duration`, and point
/// `time` (Req 9.4, 9.5): each must lie within `[0.0, 999_999_999.0]`.
const TIME_MAX: f64 = 999_999_999.0;

/// Inclusive maximum number of entries permitted in the `robots`/`groups` maps and
/// in a group's `trajectories` map (Req 9.2).
const MAP_ENTRY_MAX: usize = 10_000;

/// Inclusive maximum number of `points` permitted in a single trajectory (Req 9.4).
const POINTS_MAX: usize = 1_000_000;

/// Top-level export document — ONE file for ALL robots.
///
/// Top-level JSON keys are exactly `timestamp`, `robots`, and `groups`
/// (Req 9.1). `robots` and `groups` use `BTreeMap` for deterministic key
/// ordering in the serialized output.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExportDocument {
    /// ISO-8601 local date-time, e.g. `"2026-05-12T09:12:40"` (Req 9.6).
    pub timestamp: String,
    /// `robot_id` -> metadata (`base_position`). Deterministic key order.
    pub robots: BTreeMap<String, RobotMeta>,
    /// Group name -> group contents. Deterministic key order.
    pub groups: BTreeMap<String, TrajectoryGroup>,
}

/// Per-robot metadata. Contains exactly `base_position` (Req 9.3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotMeta {
    /// World base position in meters `[x, y, z]` (D3).
    pub base_position: [f64; 3],
}

/// Contents of a single named group.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryGroup {
    /// `robot_id` -> ordered trajectories. `IndexMap` preserves run order.
    pub trajectories: IndexMap<String, Vec<ExportedTrajectory>>,
}

/// A single recorded trajectory. Contains exactly `start_time`, `duration`,
/// and `points` (Req 9.4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExportedTrajectory {
    /// Seconds on this robot's timeline at which the trajectory begins (D4).
    pub start_time: f64,
    /// Trajectory duration in seconds (== `TrajectorySolution.duration_s`).
    pub duration: f64,
    /// Sampled waypoints, time-ordered.
    pub points: Vec<TrajectoryPoint>,
}

/// A single sampled point. Contains exactly `positions` and `time` (Req 9.5).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Joint positions (radians); length == `joint_dim`. Axes >= 6 are padded (D1).
    pub positions: Vec<f64>,
    /// Time within the trajectory in seconds, in `[0, duration]`.
    pub time: f64,
}

/// Configuration for a headless export run.
///
/// Numeric ranges are enforced at CLI parse time (`main.rs`); this struct simply
/// carries the validated values into the driver and sampler. Defaults mirror the
/// design's chosen defaults (D1–D5) and the requirements' default values.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportOptions {
    /// Output group name; non-empty, length <= 255 (Req 5.1, 5.2). Default `"structural"`.
    pub group_name: String,
    /// Points emitted per trajectory, evenly spaced in time; `2..=10000` (Req 8.1). Default `10`.
    pub samples_per_trajectory: usize,
    /// Width of each `positions` vector; `6..=64` (Req 4.1). Default `7`.
    pub joint_dim: usize,
    /// Constant value emitted for joint-vector indices >= 6 (D1). Default `0.0`.
    pub external_axis_fill: f64,
    /// Hard cap on PLC cycles that guarantees termination; `1..=100000000` (Req 13.1). Default `100000`.
    pub max_cycles: usize,
    /// When set, omit trajectories whose `JointPath` is `None` (Req 14.3). Default `false`.
    pub skip_jointless: bool,
    /// Positive floor (seconds) substituted for a resolved `duration_s <= 0` (Req 11.2). Default `0.001`.
    pub min_duration_floor: f64,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            group_name: "structural".to_string(),
            samples_per_trajectory: 10,
            joint_dim: 7,
            external_axis_fill: 0.0,
            max_cycles: 100_000,
            skip_jointless: false,
            min_duration_floor: 0.001,
        }
    }
}

/// Resample a resolved `JointPath` into a fixed list of time-ordered points.
///
/// Emits exactly `opts.samples_per_trajectory` (`N`) points. For point index
/// `k in 0..N`, `progress = k / (N - 1)` (evenly spaced over `[0, 1]` inclusive)
/// and `time = progress * duration_s`. When `duration_s == 0.0` every point time
/// is `0.0` (Req 8.8).
///
/// Each `positions` vector has length `opts.joint_dim`:
/// * For a joint-bearing path (`Dense`/`Endpoints`), indices `0..6` hold the six
///   robot joint values (radians) from `path.sample(progress)` and indices `>= 6`
///   are filled with `opts.external_axis_fill` (Req 4.2, 4.3, 4.4).
/// * For `JointPath::None` — and defensively for any path that yields no sample —
///   every value is set to `opts.external_axis_fill` (Req 14.1, 14.2).
///
/// Preconditions (enforced upstream at CLI parse time): `samples_per_trajectory >= 2`
/// and `joint_dim >= 6`.
pub fn sample_points(path: &JointPath, duration_s: f64, opts: &ExportOptions) -> Vec<TrajectoryPoint> {
    let n = opts.samples_per_trajectory;
    let mut out = Vec::with_capacity(n);
    // `N >= 2` is guaranteed upstream, so `(n - 1)` is a safe, non-zero divisor.
    let denom = (n - 1) as f64;
    for k in 0..n {
        let progress = k as f64 / denom;
        let time = progress * duration_s;
        let positions = match path.sample(progress) {
            Some(joints) => pad_joints(&joints, opts.joint_dim, opts.external_axis_fill),
            None => vec![opts.external_axis_fill; opts.joint_dim],
        };
        out.push(TrajectoryPoint { positions, time });
    }
    out
}

/// Build a `joint_dim`-wide positions vector: the six joint values at indices
/// `0..6`, then `external_axis_fill` for every trailing index `>= 6`.
///
/// `joint_dim >= 6` is guaranteed upstream, so all six joint values fit.
fn pad_joints(joints: &[f64; 6], joint_dim: usize, fill: f64) -> Vec<f64> {
    let mut positions = vec![fill; joint_dim];
    positions[..6].copy_from_slice(joints);
    positions
}

/// Build a [`MotionRequest`] from a robot's current interpreter state.
///
/// This mirrors `server.rs::presolve_motion_joints_for_robot` exactly so the
/// headless export resolves motions through the same provider contract as the
/// live server (Req 3.1):
/// * `start_pose` is taken from `motion_state.start_position` as
///   `[x, y, z, a, b, c]` (mm / deg).
/// * `target_pose` is taken from the current `Instruction::Motion.position`.
/// * `kind` comes from `current_motion_kind()`, defaulting to
///   [`MotionKind::Ptp`] when the PC is not on a recognized motion.
/// * `vel_percent` is the motion's `context.vel_ptp` and `approximate` is the
///   instruction's blend flag.
/// * `seed_joints` prefers the interpreter's `current_joints`, falling back to
///   the robot's `home_joints`, and finally to `[0.0; 6]`.
///
/// Returns `None` when the robot has no interpreter, is not mid-motion
/// (`motion_state` is absent), or its program counter is not positioned on a
/// `Motion` instruction — the same guards `presolve_motion_joints_for_robot`
/// applies before building its request. Callers (the `run_export` driver) only
/// invoke this after landing on a Motion, where it always yields `Some`.
pub fn build_motion_request(krl: &RobotKrl) -> Option<MotionRequest> {
    let interp = krl.interpreter.as_ref()?;
    // Must be mid-motion: `start_position` is only captured once a motion begins.
    let motion_state = interp.motion_state.as_ref()?;
    // The PC must currently sit on a Motion instruction.
    let Instruction::Motion {
        position,
        context,
        approximate,
        ..
    } = interp.current_instruction()?
    else {
        return None;
    };

    let kind = interp.current_motion_kind().unwrap_or(MotionKind::Ptp);
    let start = &motion_state.start_position;
    let start_pose = [start.x, start.y, start.z, start.a, start.b, start.c];
    let target_pose = [
        position.x, position.y, position.z, position.a, position.b, position.c,
    ];
    let seed_joints = interp
        .current_joints
        .or(krl.home_joints)
        .unwrap_or([0.0f64; 6]);

    Some(MotionRequest {
        kind,
        start_pose,
        target_pose,
        vel_percent: context.vel_ptp,
        approximate: *approximate,
        seed_joints,
    })
}

/// Accumulates recorded trajectories keyed by group then robot, assigning each
/// trajectory a `start_time` along a per-robot sequential timeline (D4).
///
/// Robots are seeded at construction so that a robot which executes zero motions
/// still appears in the output with an empty `trajectories` collection (Req 2.4).
/// Insertion order is preserved via `IndexMap` for stable, diffable output
/// (Req 12.3, 12.4).
#[derive(Debug, Clone)]
pub struct TrajectoryRecorder {
    /// Name of the single group all robots are nested under.
    group_name: String,
    /// `robot_id` -> ordered list of recorded trajectories. `IndexMap` preserves
    /// the seeding/insertion order.
    per_robot: IndexMap<String, Vec<ExportedTrajectory>>,
    /// `robot_id` -> running end-of-timeline cursor in seconds (D4). Equals the
    /// sum of the durations of all trajectories recorded so far for that robot.
    cursor: HashMap<String, f64>,
    /// The robot ids exactly as supplied to `new`, retained (with any duplicates)
    /// so `finalize` can reject duplicate Robot_Ids (Req 2.2).
    seeded_ids: Vec<String>,
}

impl TrajectoryRecorder {
    /// Create a recorder for `group_name`, seeding every id in `robot_ids` with an
    /// empty timeline so each robot appears in the output exactly once even if it
    /// records no motions (Req 2.1, 2.4). The supplied id order is preserved.
    pub fn new(group_name: &str, robot_ids: &[String]) -> Self {
        let mut per_robot: IndexMap<String, Vec<ExportedTrajectory>> = IndexMap::new();
        let mut cursor: HashMap<String, f64> = HashMap::new();
        for id in robot_ids {
            // First-seen wins for ordering; duplicates are detected in `finalize`.
            per_robot.entry(id.clone()).or_default();
            cursor.entry(id.clone()).or_insert(0.0);
        }
        Self {
            group_name: group_name.to_string(),
            per_robot,
            cursor,
            seeded_ids: robot_ids.to_vec(),
        }
    }

    /// Append one trajectory for `robot_id`, assigning its `start_time` from the
    /// robot's current timeline cursor and then advancing the cursor by the
    /// recorded duration (Req 7.1, 7.2, 7.3).
    ///
    /// Duration handling:
    /// * A non-numeric (NaN) or negative `duration_s` is rejected with `Err` and
    ///   the trajectory is excluded, leaving the robot's cursor (and therefore all
    ///   subsequent `start_time`s) unchanged (Req 7.4).
    /// * A `duration_s` of exactly `0` is clamped up to the positive floor
    ///   (`MIN_DURATION_FLOOR`, 0.001 s) and a warning is logged, so every recorded
    ///   `duration` is `> 0` (Req 11.2, 11.3).
    /// * A positive `duration_s` is recorded as-is (Req 11.1).
    ///
    /// A robot id not present at construction is added on first use, keeping an
    /// independent timeline per distinct id.
    pub fn record(
        &mut self,
        robot_id: &str,
        duration_s: f64,
        points: Vec<TrajectoryPoint>,
    ) -> Result<()> {
        // Reject NaN or negative durations without touching the timeline (Req 7.4).
        if duration_s.is_nan() {
            bail!("invalid trajectory duration for robot '{robot_id}': duration is not a number (NaN)");
        }
        if duration_s < 0.0 {
            bail!(
                "invalid trajectory duration for robot '{robot_id}': duration {duration_s} is negative"
            );
        }

        // Clamp a non-positive (i.e. exactly zero) duration up to the floor (Req 11.2).
        let duration = if duration_s <= 0.0 {
            eprintln!(
                "WARN: robot '{robot_id}' trajectory #{} has non-positive duration {duration_s}; \
                 clamping to floor {MIN_DURATION_FLOOR} s",
                self.per_robot.get(robot_id).map_or(0, Vec::len)
            );
            MIN_DURATION_FLOOR
        } else {
            duration_s
        };

        let start_time = *self.cursor.entry(robot_id.to_string()).or_insert(0.0);
        self.per_robot
            .entry(robot_id.to_string())
            .or_default()
            .push(ExportedTrajectory {
                start_time,
                duration,
                points,
            });
        // Advance the cursor by the recorded (effective) duration so subsequent
        // start_times equal the running sum of recorded durations (Req 7.2).
        *self.cursor.get_mut(robot_id).expect("cursor seeded above") = start_time + duration;
        Ok(())
    }

    /// Build the final `ExportDocument`, nesting every robot's trajectories under
    /// the single configured group.
    ///
    /// Returns `Err` (and produces no document) if the ids supplied to `new`
    /// contained a duplicate Robot_Id (Req 2.2). Every seeded robot appears in the
    /// group exactly once; robots that recorded zero motions carry an empty
    /// `trajectories` collection (Req 2.3, 2.4).
    pub fn finalize(
        self,
        robots: BTreeMap<String, RobotMeta>,
        ts: String,
    ) -> Result<ExportDocument> {
        // Reject duplicate Robot_Ids before emitting any document (Req 2.2).
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for id in &self.seeded_ids {
            if !seen.insert(id.as_str()) {
                bail!("duplicate Robot_Id '{id}' in cell; cannot produce export document");
            }
        }

        let mut groups = BTreeMap::new();
        groups.insert(
            self.group_name,
            TrajectoryGroup {
                trajectories: self.per_robot,
            },
        );

        Ok(ExportDocument {
            timestamp: ts,
            robots,
            groups,
        })
    }
}

/// Resolve each robot's `base_position` (meters, `[x, y, z]`) from an optional
/// scene YAML file (D3).
///
/// Scene schema (matching the viewer scene files): a top-level `robots` sequence
/// whose entries carry a `robot_id` string and a `position: {x, y, z}` mapping in
/// meters.
///
/// Behavior:
/// * When `scene` is `None`, every robot defaults to `[0.0, 0.0, 0.0]` and a single
///   warning is logged noting that no scene file was provided (Req 6.3).
/// * When a scene file is given, it is opened, read, and parsed as YAML; any
///   failure to open/read/parse returns `Err` identifying the file and the cause
///   (Req 6.5).
/// * A robot present in `robots` but absent from the scene defaults to zeros and a
///   warning naming the robot is logged (Req 6.4).
/// * A robot whose scene position is malformed (not exactly 3 finite `x, y, z`
///   values) defaults to zeros and a warning naming the robot is logged (Req 6.6).
/// * A robot whose scene position is well-formed is emitted in meters as supplied
///   (Req 6.2).
///
/// The returned map contains one entry per robot in `robots`, keyed by `robot_id`.
pub fn load_base_positions(
    scene: Option<&Path>,
    robots: &[RobotState],
) -> Result<HashMap<String, [f64; 3]>> {
    // No scene file: every robot defaults to zeros with a single warning (Req 6.3).
    let Some(path) = scene else {
        eprintln!(
            "WARN: no scene file provided; defaulting base_position to [0, 0, 0] for all robots"
        );
        return Ok(robots
            .iter()
            .map(|r| (r.robot_id.clone(), [0.0_f64; 3]))
            .collect());
    };

    // Open/read/parse the scene file; any failure is a hard error (Req 6.5).
    let content = std::fs::read_to_string(path)
        .with_context(|| format!("opening/reading scene file {}", path.display()))?;
    let doc: serde_yaml::Value = serde_yaml::from_str(&content)
        .with_context(|| format!("parsing scene file {} as YAML", path.display()))?;

    // Collect well-formed positions and the ids of malformed entries from the
    // scene's `robots` sequence. A valid YAML file that simply lacks a usable
    // `robots` sequence is not an error: every robot then defaults to zeros via
    // the absent-robot path (Req 6.4).
    let mut parsed: HashMap<String, [f64; 3]> = HashMap::new();
    let mut malformed: HashSet<String> = HashSet::new();
    if let Some(entries) = doc.get("robots").and_then(|v| v.as_sequence()) {
        for entry in entries {
            let Some(id) = entry.get("robot_id").and_then(|v| v.as_str()) else {
                // An entry without a usable robot_id cannot be mapped; skip it.
                continue;
            };
            match parse_position(entry.get("position")) {
                Some(pos) => {
                    parsed.insert(id.to_string(), pos);
                }
                None => {
                    malformed.insert(id.to_string());
                }
            }
        }
    }

    // Resolve one entry per cell robot, warning on absent/malformed positions.
    let mut out = HashMap::with_capacity(robots.len());
    for r in robots {
        let id = &r.robot_id;
        // A scene generated upstream names robots with ITS ids (e.g.
        // "kr240r2900__sticktool") while the cell uses the plant/planner ids
        // ("robot1"). cell.yaml's `scene_id` bridges the two, exactly as the
        // viewers' scene lookup does — so resolve
        // through `scene_id` first and fall back to `robot_id`. Without this a
        // scene using generator ids silently yields origin base transforms for
        // every robot, which makes collision checking meaningless.
        let lookup = r.scene_id.as_deref().unwrap_or(id.as_str());
        if let Some(pos) = parsed.get(lookup) {
            out.insert(id.clone(), *pos);
        } else if malformed.contains(lookup) {
            eprintln!(
                "WARN: robot '{id}' has an invalid scene position (expected 3 finite x, y, z \
                 values); defaulting base_position to [0, 0, 0]"
            );
            out.insert(id.clone(), [0.0; 3]);
        } else {
            eprintln!(
                "WARN: robot '{lookup}' is absent from scene file {}; defaulting \
                 base_position to [0, 0, 0]",
                path.display()
            );
            out.insert(id.clone(), [0.0; 3]);
        }
    }
    Ok(out)
}

/// Parse a scene `position` value into `[x, y, z]`, requiring all three keys to be
/// present and finite. Returns `None` for any malformed position (Req 6.6).
fn parse_position(value: Option<&serde_yaml::Value>) -> Option<[f64; 3]> {
    let map = value?;
    let x = finite_f64(map.get("x"))?;
    let y = finite_f64(map.get("y"))?;
    let z = finite_f64(map.get("z"))?;
    Some([x, y, z])
}

/// Extract a finite `f64` from an optional YAML value, rejecting missing,
/// non-numeric, and non-finite (NaN/inf) values.
fn finite_f64(value: Option<&serde_yaml::Value>) -> Option<f64> {
    let n = value?.as_f64()?;
    n.is_finite().then_some(n)
}

/// Parse a scene `orientation` value into KUKA `[a, b, c]` Euler angles in
/// degrees, requiring all three keys to be present and finite. Returns `None`
/// for any malformed orientation. Sibling of [`parse_position`].
fn parse_orientation(value: Option<&serde_yaml::Value>) -> Option<[f64; 3]> {
    let map = value?;
    let a = finite_f64(map.get("a"))?;
    let b = finite_f64(map.get("b"))?;
    let c = finite_f64(map.get("c"))?;
    Some([a, b, c])
}

/// Load the placed static environment objects declared in a scene YAML's
/// `static_models:` sequence (D-env, Req 1.x/2.x).
///
/// Scene schema (matching the viewer scene files): a top-level `static_models`
/// sequence whose entries carry a non-empty `id` string, a non-empty `file`
/// string (a `.glb` path resolved relative to the scene file's parent
/// directory), a `position: {x, y, z}` mapping in meters, and an optional
/// `orientation: {a, b, c}` mapping (KUKA intrinsic Z-Y-X Euler angles, in
/// degrees).
///
/// Behavior mirrors [`load_base_positions`]'s never-fail-hard, warn-and-skip
/// discipline:
/// * When `scene` is `None`, an empty vec is returned with a single warning
///   noting that no scene file was provided (Req 1.9).
/// * When a scene file is given, it is opened, read, and parsed as YAML; any
///   failure to open/read/parse returns `Err` identifying the file and the
///   cause (Req 1.10) — this is the only error path.
/// * A valid YAML document that simply lacks a usable `static_models`
///   sequence yields an empty vec with a warning (Req 1.9).
/// * Each entry missing/empty `id` or `file`, or carrying a malformed
///   `position` or a present-but-malformed `orientation`, is skipped with a
///   warning naming the entry (by `id` when known) and processing continues
///   with the remaining entries (Req 1.6, 1.7, 1.8, 2.4, 2.6).
/// * An entry with no `orientation` key at all defaults to identity rotation
///   `[0.0, 0.0, 0.0]` (Req 1.4).
/// * `file` is resolved to an absolute path against `scene.parent()` (Req 1.3).
/// * The world transform is composed as nalgebra's KUKA intrinsic Z-Y-X
///   degrees convention: `Isometry3::from_parts(Translation3::new(x, y, z),
///   UnitQuaternion::from_euler_angles(c.to_radians(), b.to_radians(),
///   a.to_radians()))` (Req 1.5, 2.1, 2.3).
#[cfg(feature = "collision")]
pub fn load_static_models(
    scene: Option<&Path>,
) -> Result<Vec<crate::collision::EnvironmentCollisionSpec>> {
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    // No scene file: nothing to load, warn and return empty (Req 1.9).
    let Some(path) = scene else {
        eprintln!("WARN: no scene file provided; loading zero static environment models");
        return Ok(Vec::new());
    };

    // Open/read/parse the scene file; any failure is a hard error (Req 1.10).
    let content = std::fs::read_to_string(path)
        .with_context(|| format!("opening/reading scene file {}", path.display()))?;
    let doc: serde_yaml::Value = serde_yaml::from_str(&content)
        .with_context(|| format!("parsing scene file {} as YAML", path.display()))?;

    // A valid YAML file that simply lacks a usable `static_models` sequence is
    // not an error: it just contributes zero environment objects (Req 1.9).
    let Some(entries) = doc.get("static_models").and_then(|v| v.as_sequence()) else {
        eprintln!(
            "WARN: scene file {} has no 'static_models' sequence; loading zero static \
             environment models",
            path.display()
        );
        return Ok(Vec::new());
    };

    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let mut out = Vec::with_capacity(entries.len());
    for entry in entries {
        let Some(id) = entry.get("id").and_then(|v| v.as_str()).filter(|s| !s.is_empty()) else {
            eprintln!(
                "WARN: static_models entry is missing a non-empty 'id'; skipping entry"
            );
            continue;
        };

        let Some(file) = entry.get("file").and_then(|v| v.as_str()).filter(|s| !s.is_empty())
        else {
            eprintln!(
                "WARN: static_models entry '{id}' is missing a non-empty 'file'; skipping entry"
            );
            continue;
        };

        let Some(position) = parse_position(entry.get("position")) else {
            eprintln!(
                "WARN: static_models entry '{id}' has an invalid 'position' (expected 3 finite \
                 x, y, z values); skipping entry"
            );
            continue;
        };

        // Absent orientation defaults to identity (Req 1.4); a present but
        // malformed orientation is a skip, not a silent identity fallback.
        let orientation = if entry.get("orientation").is_none() {
            [0.0_f64; 3]
        } else {
            match parse_orientation(entry.get("orientation")) {
                Some(o) => o,
                None => {
                    eprintln!(
                        "WARN: static_models entry '{id}' has an invalid 'orientation' \
                         (expected 3 finite a, b, c values); skipping entry"
                    );
                    continue;
                }
            }
        };

        let resolved_file = parent.join(file);
        let [x, y, z] = position;
        let [a, b, c] = orientation;
        let transform = Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::from_euler_angles(c.to_radians(), b.to_radians(), a.to_radians()),
        );

        out.push(crate::collision::EnvironmentCollisionSpec::new(
            id,
            resolved_file,
            transform,
        ));
    }

    Ok(out)
}

/// Serialize `doc` to pretty-printed JSON and write it to `out_path` atomically.
///
/// The document is first validated against the output schema's ranges
/// (`validate_document`); if any value is unavailable, non-finite, or out of range
/// the function aborts before touching the filesystem and returns an `Err` naming
/// the failing key (Req 9.7).
///
/// The write itself is atomic (Req 15.1): the serialized bytes are written to a
/// uniquely-named temporary file in the **same parent directory** as `out_path`,
/// then renamed onto the target in a single operation so the target is either
/// fully replaced or left unchanged. Renaming onto an existing target overwrites
/// it (Req 1.4, 2.5, 2.6).
///
/// Failure handling:
/// * If `out_path`'s parent directory does not exist, the write is aborted before
///   any file is created and an `Err` naming the path and reason is returned
///   (Req 1.5, 15.2). A parent directory that is not writable surfaces as a temp
///   file creation error, again leaving no file behind.
/// * If an I/O error occurs while writing the temporary file or during the rename,
///   the temporary file is deleted, any pre-existing target is left unchanged, and
///   an `Err` naming the reason and affected path is returned (Req 15.3).
pub fn write_json(out_path: &Path, doc: &ExportDocument) -> Result<()> {
    // Validate every schema range before doing any I/O so a bad value never
    // produces a partial file (Req 9.7).
    validate_document(doc)
        .with_context(|| format!("validating export document before writing {}", out_path.display()))?;

    // Resolve the parent directory; an empty parent (bare filename) means the
    // current working directory. Abort if it is missing (Req 15.2).
    let parent = match out_path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p.to_path_buf(),
        _ => PathBuf::from("."),
    };
    if !parent.is_dir() {
        bail!(
            "cannot write export file {}: parent directory {} does not exist",
            out_path.display(),
            parent.display()
        );
    }

    // Serialize to pretty-printed JSON (Req 10.1).
    let json = serde_json::to_string_pretty(doc)
        .with_context(|| format!("serializing export document for {}", out_path.display()))?;

    // Write to a uniquely-named temp file in the SAME parent dir (Req 15.1).
    let tmp_path = temp_path_in(&parent, out_path);
    if let Err(e) = std::fs::write(&tmp_path, json.as_bytes()) {
        // A write failure may have left a partial temp file; remove it and leave
        // any pre-existing target untouched (Req 15.3). An unwritable parent dir
        // surfaces here with no file created (Req 15.2).
        let _ = std::fs::remove_file(&tmp_path);
        return Err(anyhow::Error::new(e)).with_context(|| {
            format!(
                "writing temporary export file {} (parent directory {} may not be writable)",
                tmp_path.display(),
                parent.display()
            )
        });
    }

    // Atomically move the temp file onto the target (Req 15.1, 2.6).
    if let Err(e) = std::fs::rename(&tmp_path, out_path) {
        let _ = std::fs::remove_file(&tmp_path);
        return Err(anyhow::Error::new(e)).with_context(|| {
            format!(
                "atomically renaming {} onto target {}",
                tmp_path.display(),
                out_path.display()
            )
        });
    }

    Ok(())
}

/// Build a unique temporary path in `parent`, derived from the target file name so
/// it is recognizably related to the output. Uniqueness combines the process id, a
/// high-resolution timestamp, and a per-process atomic counter so concurrent or
/// repeated writes never collide.
fn temp_path_in(parent: &Path, target: &Path) -> PathBuf {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let base = target
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("export.json");
    parent.join(format!(".{base}.tmp.{}.{nanos}.{n}", std::process::id()))
}

/// Validate an `ExportDocument` against the output schema's value ranges before
/// serialization (Req 9.7). Returns `Err` naming the failing key when a value is
/// non-finite, out of range, or otherwise unavailable.
fn validate_document(doc: &ExportDocument) -> Result<()> {
    // `timestamp` must be ISO-8601 local date-time `YYYY-MM-DDThh:mm:ss` (Req 9.6).
    if NaiveDateTime::parse_from_str(&doc.timestamp, "%Y-%m-%dT%H:%M:%S").is_err() {
        bail!(
            "schema validation failed at key 'timestamp': value {:?} is not an ISO-8601 local \
             date-time (expected YYYY-MM-DDThh:mm:ss)",
            doc.timestamp
        );
    }

    // Map cardinalities (Req 9.2).
    if doc.robots.len() > MAP_ENTRY_MAX {
        bail!(
            "schema validation failed at key 'robots': {} entries exceed the maximum of {MAP_ENTRY_MAX}",
            doc.robots.len()
        );
    }
    if doc.groups.len() > MAP_ENTRY_MAX {
        bail!(
            "schema validation failed at key 'groups': {} entries exceed the maximum of {MAP_ENTRY_MAX}",
            doc.groups.len()
        );
    }

    // Each robot's `base_position` must be 3 finite values within `[-1e6, 1e6]` (Req 9.3).
    for (id, meta) in &doc.robots {
        for (axis, v) in meta.base_position.iter().enumerate() {
            let key = format!("robots[\"{id}\"].base_position[{axis}]");
            if !v.is_finite() {
                bail!("schema validation failed at key '{key}': value {v} is not finite");
            }
            if *v < -BASE_POSITION_ABS_MAX || *v > BASE_POSITION_ABS_MAX {
                bail!(
                    "schema validation failed at key '{key}': value {v} is outside \
                     [-{BASE_POSITION_ABS_MAX}, {BASE_POSITION_ABS_MAX}]"
                );
            }
        }
    }

    // Each trajectory and point must carry in-range times/durations and finite
    // joint positions (Req 9.4, 9.5).
    for (gname, group) in &doc.groups {
        if group.trajectories.len() > MAP_ENTRY_MAX {
            bail!(
                "schema validation failed at key 'groups[\"{gname}\"].trajectories': {} entries \
                 exceed the maximum of {MAP_ENTRY_MAX}",
                group.trajectories.len()
            );
        }
        for (rid, trajs) in &group.trajectories {
            for (ti, traj) in trajs.iter().enumerate() {
                let key = format!("groups[\"{gname}\"].trajectories[\"{rid}\"][{ti}]");
                validate_time(traj.start_time, &format!("{key}.start_time"))?;
                validate_time(traj.duration, &format!("{key}.duration"))?;
                if traj.points.len() > POINTS_MAX {
                    bail!(
                        "schema validation failed at key '{key}.points': {} elements exceed the \
                         maximum of {POINTS_MAX}",
                        traj.points.len()
                    );
                }
                for (pi, pt) in traj.points.iter().enumerate() {
                    let pkey = format!("{key}.points[{pi}]");
                    for (ji, jv) in pt.positions.iter().enumerate() {
                        if !jv.is_finite() {
                            bail!(
                                "schema validation failed at key '{pkey}.positions[{ji}]': value \
                                 {jv} is not finite"
                            );
                        }
                    }
                    validate_time(pt.time, &format!("{pkey}.time"))?;
                }
            }
        }
    }

    Ok(())
}

/// Validate a single time/duration value is finite and within `[0.0, TIME_MAX]`
/// (Req 9.4, 9.5), naming `key` on failure.
fn validate_time(v: f64, key: &str) -> Result<()> {
    if !v.is_finite() {
        bail!("schema validation failed at key '{key}': value {v} is not finite");
    }
    if v < 0.0 || v > TIME_MAX {
        bail!("schema validation failed at key '{key}': value {v} is outside [0.0, {TIME_MAX}]");
    }
    Ok(())
}

/// Summary of a completed (or capped) headless export run, returned to the
/// caller (`main.rs`) for end-of-run logging.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportSummary {
    /// Number of robots present in the cell (== entries in `doc.robots`).
    pub robot_count: usize,
    /// Total trajectories recorded across all robots.
    pub trajectory_count: usize,
    /// Number of motions resolved at a tier below Ruckig OTG (downgrades).
    pub downgraded_count: usize,
    /// PLC cycles executed by the run loop.
    pub cycles_run: usize,
    /// Whether the loop stopped because the `max_cycles` cap was hit (Req 13.4).
    pub max_cycles_reached: bool,
    /// Robot ids that had not completed when the loop stopped. Empty on a clean
    /// run; populated only when the cap was reached (Req 13.7).
    pub incomplete_robots: Vec<String>,
}

/// Deterministically run the whole cell headless and serialize every resolved
/// motion for every robot into one combined JSON file (Req 1.3, 1.4).
///
/// The driver reuses the live server's building blocks — the [`CellSimulator`]
/// PLC FSM, the per-robot [`RobotKrl`]/`KrlInterpreter`, the PLC↔KRL [`bridge`],
/// and each model's [`LayeredTrajectoryProvider`] — but without the real-time
/// animation thread or any network socket, so the run is fully deterministic.
///
/// # Run loop
///
/// Every robot is first marked ready (`RoboterBereit = true`) so the PLC FSM can
/// leave its `WaitReady` phase without a real hardware handshake (parity with the
/// interactive `set_all_ready`). The loop then repeats, bounded by
/// `opts.max_cycles` (Req 13.4):
///
/// 1. [`CellSimulator::step`] advances the PLC FSM one transition per robot,
///    honoring inter-robot `depends_on` ordering.
/// 2. For each not-yet-done robot: sync PLC↔KRL signals and (when the robot
///    reaches its `Running`/`WaitFM` phase) start its selected folge via
///    [`bridge::handle_folge_selection`] + [`bridge::plc_to_krl`], then drive the
///    interpreter to the next Motion with [`RobotKrl::step_to_motion`].
///    * On a started Motion, build a [`MotionRequest`] from interpreter state,
///      resolve it through the robot's provider, log the note to stderr when the
///      tier is below Ruckig OTG (Req 3.4), sample the path to points, record the
///      trajectory, then advance the interpreter to `path.sample(1.0)` and consume
///      the motion so the next `step_to_motion` lands on the following Motion.
///    * On a blocked wait, auto-satisfy it so the robot advances next cycle
///      (Req 13.8).
/// 3. Then sync KRL→PLC. A robot is considered complete once it has finished one
///    full production pass through its folgen (the FSM otherwise loops forever),
///    surfaced by the simulator's `cycles_completed` counter (Req 13.3).
///
/// # Termination & output
///
/// The loop stops when every robot is complete or `max_cycles` is hit; on the cap
/// it logs a warning naming the incomplete robots (Req 13.7). Either way the
/// accumulated trajectories are finalized with an ISO-8601 local timestamp and
/// written atomically (Req 13.5).
///
/// # Abort
///
/// If a Motion cannot be resolved (no provider), or resolves to `duration_s == 0`
/// or a joint-bearing path with fewer than 2 points, the run aborts with an error
/// to stderr and returns `Err` without writing the file, retaining the
/// already-recorded trajectories in memory (Req 3.5). A jointless
/// (`JointPath::None`) heuristic solution is not an abort: it is either recorded
/// as filled points or omitted under `--skip-jointless` (Req 14.3, 14.4).
pub fn run_export(
    mut sim: CellSimulator,
    mut krls: Vec<RobotKrl>,
    mut providers: HashMap<String, LayeredTrajectoryProvider>,
    base_positions: HashMap<String, [f64; 3]>,
    opts: &ExportOptions,
    out_path: &Path,
) -> Result<ExportSummary> {
    // Robot ids in cell-definition order so the output ordering is stable and
    // diffable (Req 12.3). Duplicates (if any) are rejected by `finalize`.
    let robot_ids: Vec<String> = sim.robots.iter().map(|r| r.robot_id.clone()).collect();
    let mut recorder = TrajectoryRecorder::new(&opts.group_name, &robot_ids);

    // Mark every robot ready up front so the PLC FSM leaves WaitReady without a
    // real hardware handshake (parity with the interactive `set_all_ready`).
    for robot in &mut sim.robots {
        robot
            .inputs
            .insert("RoboterBereit".into(), SignalValue::Bool(true));
    }

    let n = krls.len();
    let mut done = vec![false; n];
    let mut trajectory_count = 0usize;
    let mut downgraded_count = 0usize;
    let mut cycles_run = 0usize;
    let mut max_cycles_reached = false;

    while !done.iter().all(|&d| d) {
        // Hard cycle cap guarantees termination even under unsatisfiable waits or
        // cyclic dependencies (Req 13.4).
        if cycles_run >= opts.max_cycles {
            max_cycles_reached = true;
            break;
        }
        sim.step();
        cycles_run += 1;

        for i in 0..n {
            if done[i] {
                continue;
            }
            let model = sim.robots[i].model.clone();
            let robot_id = sim.robots[i].robot_id.clone();

            // PLC↔KRL signal sync + folge start (no real-time stepping here).
            bridge::handle_folge_selection(&mut sim.robots[i], &mut krls[i]);
            bridge::plc_to_krl(&sim.robots[i], &mut krls[i]);

            if krls[i].interpreter.is_some() && !krls[i].program_complete {
                match krls[i].step_to_motion() {
                    StepStatus::MotionInProgress => {
                        // Landed on (started) a Motion: resolve, record, advance.
                        let req = build_motion_request(&krls[i]).ok_or_else(|| {
                            anyhow::anyhow!(
                                "robot '{robot_id}': landed on a motion but could not build a \
                                 motion request from interpreter state"
                            )
                        })?;
                        let provider = providers.get_mut(&model).ok_or_else(|| {
                            anyhow::anyhow!(
                                "robot '{robot_id}': no trajectory provider for model '{model}'; \
                                 cannot resolve motion"
                            )
                        })?;
                        let sol = provider.resolve_motion(&req);

                        // Abort on an unusable solution, retaining prior trajectories (Req 3.5).
                        if sol.duration_s == 0.0 {
                            eprintln!(
                                "ERROR: robot '{robot_id}' motion resolved to a zero duration; \
                                 aborting export (prior trajectories retained)"
                            );
                            bail!("robot '{robot_id}': motion resolved to a zero duration");
                        }
                        if let Some(count) = joint_point_count(&sol.path) {
                            if count < 2 {
                                eprintln!(
                                    "ERROR: robot '{robot_id}' motion resolved to a path with \
                                     {count} point(s) (need >= 2); aborting export (prior \
                                     trajectories retained)"
                                );
                                bail!(
                                    "robot '{robot_id}': motion path has fewer than 2 points"
                                );
                            }
                        }

                        // Log the downgrade note for non-Ruckig tiers (Req 3.4).
                        if sol.tier != SolverTier::RuckigOtg {
                            if let Some(note) = &sol.note {
                                eprintln!(
                                    "WARN: [{model}] robot '{robot_id}' motion downgraded to {}: {note}",
                                    sol.tier
                                );
                            }
                            downgraded_count += 1;
                        }

                        let jointless = matches!(sol.path, JointPath::None);
                        if jointless && opts.skip_jointless {
                            // Omit a jointless trajectory; one warning per omission (Req 14.3, 14.4).
                            eprintln!(
                                "WARN: robot '{robot_id}' trajectory has no joint information; \
                                 omitting it (--skip-jointless)"
                            );
                        } else {
                            let points = sample_points(&sol.path, sol.duration_s, opts);
                            recorder.record(&robot_id, sol.duration_s, points)?;
                            trajectory_count += 1;
                        }

                        // Advance the interpreter to the motion end (seed the next
                        // motion from this end configuration) and consume the motion.
                        if let Some(interp) = krls[i].interpreter.as_mut() {
                            if let Some(end) = sol.path.sample(1.0) {
                                interp.current_joints = Some(end);
                            }
                            finish_motion(interp);
                        }
                    }
                    StepStatus::Blocked => {
                        // Auto-satisfy the pending wait so the robot advances next
                        // cycle, keeping the headless run terminating (Req 13.8).
                        satisfy_pending_wait(&mut krls[i]);
                    }
                    StepStatus::Executed | StepStatus::Done => {}
                }
            }

            bridge::krl_to_plc(&krls[i], &mut sim.robots[i]);

            // A robot is complete once it has finished one full production pass
            // through its folgen; the FSM otherwise loops production forever
            // (Req 13.3).
            if sim.robots[i].cycles_completed >= 1 {
                done[i] = true;
            }
        }
    }

    // Robots that never completed (only possible when the cap was reached).
    let incomplete_robots: Vec<String> = (0..n)
        .filter(|&i| !done[i])
        .map(|i| sim.robots[i].robot_id.clone())
        .collect();

    if max_cycles_reached {
        // Name the robots that did not complete when the cap was hit (Req 13.7).
        eprintln!(
            "WARN: export stopped after reaching the max-cycles cap ({}); incomplete robots: {:?}",
            opts.max_cycles, incomplete_robots
        );
    }

    // Finalize with an ISO-8601 local timestamp `YYYY-MM-DDThh:mm:ss` (Req 9.6)
    // and write all accumulated trajectories atomically (Req 13.5).
    let ts = chrono::Local::now().format("%Y-%m-%dT%H:%M:%S").to_string();
    let robots_meta: BTreeMap<String, RobotMeta> = robot_ids
        .iter()
        .map(|id| {
            let base_position = base_positions.get(id).copied().unwrap_or([0.0; 3]);
            (id.clone(), RobotMeta { base_position })
        })
        .collect();
    // `finalize` rejects duplicate Robot_Ids before any file is written (Req 2.2).
    let doc = recorder.finalize(robots_meta, ts)?;
    write_json(out_path, &doc)?;

    Ok(ExportSummary {
        robot_count: robot_ids.len(),
        trajectory_count,
        downgraded_count,
        cycles_run,
        max_cycles_reached,
        incomplete_robots,
    })
}

/// Number of joint-bearing points a resolved path carries, or `None` for a path
/// that carries no joint information (`JointPath::None`, the heuristic tier).
///
/// A `None` result is handled by the jointless path logic (record filled points
/// or omit under `--skip-jointless`), not the point-count abort check, so the two
/// concerns stay separate (Req 3.5 vs Req 14).
fn joint_point_count(path: &JointPath) -> Option<usize> {
    match path {
        JointPath::Dense { qs, .. } => Some(qs.len()),
        JointPath::Endpoints { .. } => Some(2),
        JointPath::None => None,
    }
}

/// Consume an in-progress motion on `interp`: fire its end-of-motion triggers,
/// snap the Cartesian position to the motion target, and advance the program
/// counter past the motion and its consumed trigger instructions.
///
/// This mirrors the completion tail of `KrlInterpreter::step` so the headless
/// driver advances motion-by-motion (recording exactly one trajectory per Motion,
/// Req 3.3) without the real-time elapsed-time stepping the live server uses. A
/// no-op when no motion is in progress.
fn finish_motion(interp: &mut KrlInterpreter) {
    let Some(mut ms) = interp.motion_state.take() else {
        return;
    };
    // Fire any not-yet-fired end-of-motion triggers (parity with the interpreter's
    // motion-completion tail) so outputs gating later folge selection are set.
    for (_, channel, value) in ms.pending_triggers.drain(..) {
        interp.io.set_output(channel, value);
    }
    // Snap Cartesian position to the motion target.
    if let Some(Instruction::Motion { position, .. }) = interp.instructions.get(interp.pc).cloned() {
        interp.current_position = position;
    }
    // Advance past the motion instruction and the trigger instructions it consumed.
    interp.pc += 1 + ms.consumed_triggers;
}

/// Auto-satisfy a pending `WaitForInput` by setting its expected input value so
/// the interpreter passes the wait on the next cycle (Req 13.8). A no-op when the
/// program counter is not currently on a `WaitForInput`.
fn satisfy_pending_wait(krl: &mut RobotKrl) {
    if let Some(interp) = krl.interpreter.as_mut() {
        if let Some(Instruction::WaitForInput { channel, value }) =
            interp.current_instruction().cloned()
        {
            interp.io.set_input(channel, value);
        }
    }
}

#[cfg(test)]
mod build_motion_request_tests {
    use super::*;
    use crate::interpreter::{Instruction, KrlInterpreter, MotionState};
    use crate::interpreter::IoState;
    use kuka_krl_parser::types::{KrlPosition, MotionContext, MotionType};

    fn pos(x: f64, y: f64, z: f64, a: f64, b: f64, c: f64) -> KrlPosition {
        KrlPosition { x, y, z, a, b, c, s: 0, t: 0 }
    }

    /// Build a `RobotKrl` whose interpreter sits on a single `Motion` with the
    /// given motion type, target pose, start pose, velocity, blend flag, joints,
    /// and home joints.
    #[allow(clippy::too_many_arguments)]
    fn krl_on_motion(
        motion_type: MotionType,
        start: KrlPosition,
        target: KrlPosition,
        vel_ptp: f64,
        approximate: bool,
        current_joints: Option<[f64; 6]>,
        home_joints: Option<[f64; 6]>,
    ) -> RobotKrl {
        let instr = Instruction::Motion {
            motion_type,
            target: "P1".to_string(),
            position: target,
            context: MotionContext { tool_no: 1, base_no: 2, vel_ptp, vel_cp: vel_ptp },
            approximate,
            desc: String::new(),
            tool_name: String::new(),
            base_name: String::new(),
        };
        let interp = KrlInterpreter {
            io: IoState::new(),
            instructions: vec![instr],
            pc: 0,
            program_name: "folge1".to_string(),
            motion_state: Some(MotionState {
                duration_s: 0.0,
                elapsed_s: 0.0,
                pending_triggers: Vec::new(),
                consumed_triggers: 0,
                start_position: start,
                joint_trajectory: None,
            }),
            plc_cycle_time: 0.012,
            current_position: KrlPosition::default(),
            current_joints,
            wait_cycles: 0,
            wait_timeout_cycles: 100,
        };
        RobotKrl {
            robot_id: "r1".to_string(),
            programs: HashMap::new(),
            interpreter: Some(interp),
            active_folge: 1,
            program_complete: false,
            io: IoState::new(),
            position: KrlPosition::default(),
            home_joints,
        }
    }

    #[test]
    fn maps_poses_kind_velocity_and_blend_from_interpreter_state() {
        let start = pos(10.0, 20.0, 30.0, 1.0, 2.0, 3.0);
        let target = pos(100.0, 200.0, 300.0, 4.0, 5.0, 6.0);
        let krl = krl_on_motion(
            MotionType::Lin,
            start,
            target,
            42.0,
            true,
            Some([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
            None,
        );

        let req = build_motion_request(&krl).expect("on a Motion -> Some");
        assert_eq!(req.kind, MotionKind::Lin);
        assert_eq!(req.start_pose, [10.0, 20.0, 30.0, 1.0, 2.0, 3.0]);
        assert_eq!(req.target_pose, [100.0, 200.0, 300.0, 4.0, 5.0, 6.0]);
        assert_eq!(req.vel_percent, 42.0);
        assert!(req.approximate);
        assert_eq!(req.seed_joints, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
    }

    #[test]
    fn ptp_motion_maps_to_ptp_kind() {
        let krl = krl_on_motion(
            MotionType::Ptp,
            KrlPosition::default(),
            KrlPosition::default(),
            100.0,
            false,
            None,
            None,
        );
        let req = build_motion_request(&krl).unwrap();
        assert_eq!(req.kind, MotionKind::Ptp);
        assert!(!req.approximate);
    }

    #[test]
    fn seed_falls_back_to_home_joints_then_zeros() {
        // No current_joints, but home_joints present -> use home_joints.
        let home = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5];
        let krl = krl_on_motion(
            MotionType::Ptp,
            KrlPosition::default(),
            KrlPosition::default(),
            100.0,
            false,
            None,
            Some(home),
        );
        assert_eq!(build_motion_request(&krl).unwrap().seed_joints, home);

        // Neither current nor home joints -> zeros.
        let krl_zero = krl_on_motion(
            MotionType::Ptp,
            KrlPosition::default(),
            KrlPosition::default(),
            100.0,
            false,
            None,
            None,
        );
        assert_eq!(build_motion_request(&krl_zero).unwrap().seed_joints, [0.0; 6]);
    }

    #[test]
    fn current_joints_take_precedence_over_home_joints() {
        let current = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4];
        let home = [1.0; 6];
        let krl = krl_on_motion(
            MotionType::Ptp,
            KrlPosition::default(),
            KrlPosition::default(),
            100.0,
            false,
            Some(current),
            Some(home),
        );
        assert_eq!(build_motion_request(&krl).unwrap().seed_joints, current);
    }

    #[test]
    fn returns_none_without_interpreter() {
        let krl = RobotKrl {
            robot_id: "r1".to_string(),
            programs: HashMap::new(),
            interpreter: None,
            active_folge: 0,
            program_complete: false,
            io: IoState::new(),
            position: KrlPosition::default(),
            home_joints: None,
        };
        assert!(build_motion_request(&krl).is_none());
    }

    #[test]
    fn returns_none_when_not_mid_motion() {
        let mut krl = krl_on_motion(
            MotionType::Ptp,
            KrlPosition::default(),
            KrlPosition::default(),
            100.0,
            false,
            None,
            None,
        );
        krl.interpreter.as_mut().unwrap().motion_state = None;
        assert!(build_motion_request(&krl).is_none());
    }
}

#[cfg(test)]
mod recorder_tests {
    use super::*;

    fn pt() -> Vec<TrajectoryPoint> {
        vec![
            TrajectoryPoint { positions: vec![0.0; 7], time: 0.0 },
            TrajectoryPoint { positions: vec![0.0; 7], time: 1.0 },
        ]
    }

    fn ids(names: &[&str]) -> Vec<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn first_trajectory_starts_at_zero_and_cursor_accumulates() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1"]));
        rec.record("r1", 2.0, pt()).unwrap();
        rec.record("r1", 3.0, pt()).unwrap();
        rec.record("r1", 1.5, pt()).unwrap();

        let trajs = &rec.per_robot["r1"];
        assert_eq!(trajs[0].start_time, 0.0);
        assert_eq!(trajs[1].start_time, 2.0);
        assert_eq!(trajs[2].start_time, 5.0);
        // start_time[k] == sum of prior durations (Req 7.2).
        assert_eq!(trajs[2].start_time, trajs[0].duration + trajs[1].duration);
    }

    #[test]
    fn timelines_are_independent_per_robot() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1", "r2"]));
        rec.record("r1", 2.0, pt()).unwrap();
        rec.record("r2", 5.0, pt()).unwrap();
        rec.record("r1", 1.0, pt()).unwrap();

        assert_eq!(rec.per_robot["r1"][0].start_time, 0.0);
        assert_eq!(rec.per_robot["r1"][1].start_time, 2.0);
        assert_eq!(rec.per_robot["r2"][0].start_time, 0.0);
    }

    #[test]
    fn zero_duration_is_clamped_to_floor() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1"]));
        rec.record("r1", 0.0, pt()).unwrap();
        rec.record("r1", 4.0, pt()).unwrap();

        assert_eq!(rec.per_robot["r1"][0].duration, MIN_DURATION_FLOOR);
        // The clamped duration advances the cursor, so the next start_time uses it.
        assert_eq!(rec.per_robot["r1"][1].start_time, MIN_DURATION_FLOOR);
    }

    #[test]
    fn negative_duration_is_rejected_and_timeline_unchanged() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1"]));
        rec.record("r1", 2.0, pt()).unwrap();
        let err = rec.record("r1", -1.0, pt());
        assert!(err.is_err());
        rec.record("r1", 3.0, pt()).unwrap();

        // Only the two valid trajectories are present; the rejected one left the
        // cursor untouched so the second valid start_time is exactly 2.0 (Req 7.4).
        assert_eq!(rec.per_robot["r1"].len(), 2);
        assert_eq!(rec.per_robot["r1"][1].start_time, 2.0);
    }

    #[test]
    fn nan_duration_is_rejected_and_timeline_unchanged() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1"]));
        rec.record("r1", 2.0, pt()).unwrap();
        assert!(rec.record("r1", f64::NAN, pt()).is_err());
        rec.record("r1", 3.0, pt()).unwrap();

        assert_eq!(rec.per_robot["r1"].len(), 2);
        assert_eq!(rec.per_robot["r1"][1].start_time, 2.0);
    }

    #[test]
    fn finalize_includes_zero_motion_robot_with_empty_trajectories() {
        let mut rec = TrajectoryRecorder::new("structural", &ids(&["r1", "r2"]));
        rec.record("r1", 2.0, pt()).unwrap();

        let mut robots = BTreeMap::new();
        robots.insert("r1".to_string(), RobotMeta { base_position: [0.0; 3] });
        robots.insert("r2".to_string(), RobotMeta { base_position: [0.0; 3] });

        let doc = rec.finalize(robots, "2026-05-12T09:12:40".to_string()).unwrap();
        let group = &doc.groups["structural"];
        assert_eq!(group.trajectories["r1"].len(), 1);
        assert!(group.trajectories["r2"].is_empty());
        // Insertion order preserved (Req 12.3).
        let order: Vec<&String> = group.trajectories.keys().collect();
        assert_eq!(order, vec!["r1", "r2"]);
    }

    #[test]
    fn finalize_rejects_duplicate_robot_ids() {
        let rec = TrajectoryRecorder::new("structural", &ids(&["r1", "r1"]));
        let robots = BTreeMap::new();
        assert!(rec.finalize(robots, "2026-05-12T09:12:40".to_string()).is_err());
    }
}
