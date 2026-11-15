//! Collision detection subsystem.
//!
//! Gated behind the `collision` Cargo feature so the crate still builds where
//! Coal (hpp-fcl) is unavailable.
//!
//! This module defines the pure-Rust collision data model
//! ([`CollisionKind`], [`CollisionPair`], [`CollisionReport`],
//! [`CollisionConfig`], [`RobotCollisionSpec`], [`TrajectoryCollisionReport`])
//! and the safe RAII [`CollisionModel`] wrapper over the `collision_*` C ABI
//! (`pinocchio_wrapper.{h,cpp}`, Pinocchio `GeometryModel` + Coal backend).
//!
//! The `SceneCollisionChecker` and the narrow/broadphase composition are
//! populated by later tasks and will consume these types. Because some of those
//! consumers do not exist yet, the module allows `dead_code` so the not-yet-wired
//! surface stays warning-clean.

#![allow(dead_code)]

use std::collections::HashMap;
use std::ffi::{c_void, CStr, CString};
use std::os::raw::{c_char, c_int};
use std::path::{Path, PathBuf};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use crate::path::JointPath;

/// Inclusive lower bound (metres) for a configured contact margin.
pub const CONTACT_MARGIN_MIN_M: f64 = 0.0;
/// Inclusive upper bound (metres) for a configured contact margin.
pub const CONTACT_MARGIN_MAX_M: f64 = 10.0;
/// Inclusive lower bound for a configured waypoint stride.
pub const WAYPOINT_STRIDE_MIN: usize = 1;
/// Inclusive upper bound for a configured waypoint stride.
pub const WAYPOINT_STRIDE_MAX: usize = 1_000_000;

/// The class of a detected (or nearest) collision.
///
/// The derived `Ord`/`PartialOrd` provide a stable total ordering used when
/// producing deterministic reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CollisionKind {
    /// A robot colliding with itself.
    SelfCollision,
    /// Two distinct robots colliding.
    RobotRobot,
    /// A robot colliding with the static environment.
    RobotEnvironment,
}

/// A reported colliding (or nearest) object pair.
///
/// `object_a` and `object_b` are fully-qualified object names of the form
/// `<robot_id>/<link_or_geom>` and are normalized so that
/// `object_a <= object_b` by Unicode code point (Req 4.2) and
/// `object_a != object_b` (Req 4.3).
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionPair {
    /// First (lexically smaller) fully-qualified object name.
    pub object_a: String,
    /// Second (lexically larger) fully-qualified object name.
    pub object_b: String,
    /// The class of collision this pair represents.
    pub kind: CollisionKind,
    /// Signed separation distance in metres (`<= 0.0` when penetrating).
    pub distance_m: f64,
}

impl CollisionPair {
    /// Qualify a robot/link into a fully-qualified object name
    /// `<robot_id>/<link_or_geom>` (Req 4.1).
    ///
    /// Returns `None` when either component is empty, since Req 4.1 requires
    /// both parts to be non-empty and separated by a single `/`.
    pub fn qualify_name(robot_id: &str, link_or_geom: &str) -> Option<String> {
        if robot_id.is_empty() || link_or_geom.is_empty() {
            return None;
        }
        Some(format!("{robot_id}/{link_or_geom}"))
    }

    /// Construct a normalized pair from two already fully-qualified object
    /// names.
    ///
    /// - Orders the names so `object_a <= object_b` by Unicode code point
    ///   (Req 4.2). Rust's `str` ordering compares UTF-8 bytes, which is
    ///   identical to Unicode scalar value (code point) ordering.
    /// - Drops identical-name contacts by returning `None` (Req 4.3).
    /// - Rejects empty names by returning `None` (a well-formed qualified name
    ///   is never empty).
    pub fn new(
        name_a: String,
        name_b: String,
        kind: CollisionKind,
        distance_m: f64,
    ) -> Option<Self> {
        if name_a.is_empty() || name_b.is_empty() {
            return None;
        }
        // Req 4.3: identical object names are not a real contact.
        if name_a == name_b {
            return None;
        }
        // Req 4.2: normalize ordering by Unicode code point.
        let (object_a, object_b) = if name_a <= name_b {
            (name_a, name_b)
        } else {
            (name_b, name_a)
        };
        Some(Self {
            object_a,
            object_b,
            kind,
            distance_m,
        })
    }

    /// Qualify both objects as `<robot_id>/<link_or_geom>` and construct a
    /// normalized pair.
    ///
    /// Returns `None` if either object name cannot be qualified (empty
    /// component) or if the two qualified names are identical (Req 4.1–4.3).
    pub fn qualify(
        robot_a: &str,
        link_a: &str,
        robot_b: &str,
        link_b: &str,
        kind: CollisionKind,
        distance_m: f64,
    ) -> Option<Self> {
        let name_a = Self::qualify_name(robot_a, link_a)?;
        let name_b = Self::qualify_name(robot_b, link_b)?;
        Self::new(name_a, name_b, kind, distance_m)
    }
}

/// Reserved identifier used to qualify environment collision objects as
/// `<ENVIRONMENT_ID>/<link_or_geom>` (Req 4.1). The static environment is not a
/// robot and therefore has no `robot_id` of its own.
pub const ENVIRONMENT_ID: &str = "environment";

/// The classification of one scene model (a robot or the environment) after a
/// full-scene configuration check (Req 9.7). Every evaluated model is placed in
/// exactly one of these states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelStatus {
    /// The model was evaluated and no colliding pair involving it was reported.
    CollisionFree,
    /// The model was evaluated and participates in at least one reported pair.
    InCollision,
    /// The model was not evaluated (e.g. no joint configuration supplied, or its
    /// class of checks is disabled), so no collision conclusion was drawn
    /// (Req 9.4).
    Unchecked,
    /// A query for this model failed across the FFI boundary; other models are
    /// still evaluated (Req 9.3, 9.6).
    Errored,
}

/// The per-model classification recorded on a [`CollisionReport`] (Req 9.7).
#[derive(Debug, Clone, PartialEq)]
pub struct ModelClassification {
    /// The model's identifier (`robot_id`, or [`ENVIRONMENT_ID`]).
    pub model_id: String,
    /// The model's evaluated status.
    pub status: ModelStatus,
    /// A descriptive error message when `status == Errored` (Req 9.3), else
    /// `None`.
    pub error: Option<String>,
}

/// The result of a single full-scene configuration check.
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionReport {
    /// `true` iff at least one colliding pair is reported (Req 5.1).
    pub in_collision: bool,
    /// Every reported colliding pair, normalized and deduplicated.
    pub pairs: Vec<CollisionPair>,
    /// Smallest separation across all evaluated pairs, or `None` when no pair
    /// was evaluated (Req 5.4, 5.5).
    pub min_distance_m: Option<f64>,
    /// Per-model classification: every scene model is classified as exactly one
    /// of collision-free / in-collision / unchecked / errored (Req 9.7). Empty
    /// for reports produced outside a full-scene check (e.g. bare
    /// [`CollisionReport::from_pairs`]).
    pub model_status: Vec<ModelClassification>,
}

impl CollisionReport {
    /// A clear report: no evaluated pairs, not in collision.
    pub fn clear() -> Self {
        Self {
            in_collision: false,
            pairs: Vec::new(),
            min_distance_m: None,
            model_status: Vec::new(),
        }
    }

    /// Build a report from a set of colliding pairs and the minimum evaluated
    /// separation.
    ///
    /// - Normalizes to a single entry per pair (Req 4.4).
    /// - Applies a stable total ordering that depends only on pair contents so
    ///   the output is deterministic regardless of insertion order (Req 8.2).
    /// - Sets `in_collision == !pairs.is_empty()` (Req 5.1).
    ///
    /// `min_distance_m` is passed through unchanged: it reflects the smallest
    /// separation across *all evaluated* pairs (which may include non-colliding
    /// pairs), so it is the caller's responsibility to supply it (Req 5.4/5.5).
    pub fn from_pairs(pairs: Vec<CollisionPair>, min_distance_m: Option<f64>) -> Self {
        let pairs = dedup_and_sort_pairs(pairs);
        Self {
            in_collision: !pairs.is_empty(),
            pairs,
            min_distance_m,
            model_status: Vec::new(),
        }
    }

    /// Attach per-model classifications to this report (Req 9.7). Builder style;
    /// used by [`SceneCollisionChecker::check_configuration`].
    pub fn with_model_status(mut self, model_status: Vec<ModelClassification>) -> Self {
        self.model_status = model_status;
        self
    }
}

/// Normalize a list of colliding pairs to a single deterministic entry per
/// normalized name pair (Req 4.4, 8.2).
///
/// Pairs are sorted by `(object_a, object_b, distance_m, kind)` so that the
/// ordering depends only on pair contents, then adjacent duplicates sharing the
/// same normalized name pair are collapsed, keeping the smallest-distance
/// (most severe) representative.
fn dedup_and_sort_pairs(mut pairs: Vec<CollisionPair>) -> Vec<CollisionPair> {
    pairs.sort_by(|a, b| {
        a.object_a
            .cmp(&b.object_a)
            .then_with(|| a.object_b.cmp(&b.object_b))
            .then_with(|| a.distance_m.total_cmp(&b.distance_m))
            .then_with(|| a.kind.cmp(&b.kind))
    });
    // `dedup_by` keeps the first element of each run of equal name pairs; after
    // the sort above that is the smallest-distance representative.
    pairs.dedup_by(|a, b| a.object_a == b.object_a && a.object_b == b.object_b);
    pairs
}

/// Configuration controlling contact margin, sampling stride, report-all
/// behavior, and per-class enable flags.
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionConfig {
    /// Report a pair as colliding when its separation is within this margin
    /// (metres). Valid range is `[0.0, 10.0]` (Req 7.4).
    pub contact_margin_m: f64,
    /// Sample every Nth `JointPath` knot. Valid range is `[1, 1_000_000]`
    /// (Req 7.6).
    pub waypoint_stride: usize,
    /// Collect all colliding pairs (`true`) vs. stop at the first (`false`).
    pub report_all: bool,
    /// Enable self-collision checking.
    pub check_self: bool,
    /// Enable robot-robot checking.
    pub check_robot_robot: bool,
    /// Enable robot-environment checking.
    pub check_environment: bool,
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self {
            contact_margin_m: 0.0,
            waypoint_stride: 1,
            // Stop-at-first by default: cheaper, and callers that want the full
            // set (e.g. the MCP preview tool) opt in explicitly.
            report_all: false,
            check_self: true,
            check_robot_robot: true,
            check_environment: true,
        }
    }
}

impl CollisionConfig {
    /// Validate a contact margin against the accepted range `[0.0, 10.0]`
    /// (Req 7.4). Non-finite values (NaN, ±inf) are rejected because they fall
    /// outside the inclusive range test.
    pub fn validate_contact_margin(margin: f64) -> Result<(), String> {
        if !(CONTACT_MARGIN_MIN_M..=CONTACT_MARGIN_MAX_M).contains(&margin) {
            return Err(format!(
                "contact_margin_m {margin} is out of range; must be within \
                 [{CONTACT_MARGIN_MIN_M}, {CONTACT_MARGIN_MAX_M}] metres"
            ));
        }
        Ok(())
    }

    /// Validate a waypoint stride against the accepted range `[1, 1_000_000]`
    /// (Req 7.6). The `usize` type makes non-integer strides unrepresentable
    /// (Req 7.7).
    pub fn validate_waypoint_stride(stride: usize) -> Result<(), String> {
        if !(WAYPOINT_STRIDE_MIN..=WAYPOINT_STRIDE_MAX).contains(&stride) {
            return Err(format!(
                "waypoint_stride {stride} is invalid; must be within \
                 [{WAYPOINT_STRIDE_MIN}, {WAYPOINT_STRIDE_MAX}]"
            ));
        }
        Ok(())
    }

    /// Set the contact margin, rejecting out-of-range values and leaving the
    /// prior configuration unchanged on rejection (Req 7.5).
    pub fn set_contact_margin(&mut self, margin: f64) -> Result<(), String> {
        Self::validate_contact_margin(margin)?;
        self.contact_margin_m = margin;
        Ok(())
    }

    /// Set the waypoint stride, rejecting invalid values and leaving the prior
    /// configuration unchanged on rejection (Req 7.7).
    pub fn set_waypoint_stride(&mut self, stride: usize) -> Result<(), String> {
        Self::validate_waypoint_stride(stride)?;
        self.waypoint_stride = stride;
        Ok(())
    }

    /// Validate the whole configuration, returning the first error found.
    pub fn validate(&self) -> Result<(), String> {
        Self::validate_contact_margin(self.contact_margin_m)?;
        Self::validate_waypoint_stride(self.waypoint_stride)?;
        Ok(())
    }
}

/// Build-time specification for one robot's collision model.
#[derive(Debug, Clone)]
pub struct RobotCollisionSpec {
    /// Stable robot identifier used to qualify object names.
    pub robot_id: String,
    /// Path to the dressed URDF providing `<collision>` geometry.
    pub urdf: PathBuf,
    /// Optional SRDF path used to disable known-safe self-collision pairs.
    pub srdf: Option<PathBuf>,
    /// Rigid world-frame transform (`world <- robot base`) from `scene.yaml`.
    pub base_transform: Isometry3<f64>,
}

impl RobotCollisionSpec {
    /// Construct a new specification.
    pub fn new(
        robot_id: impl Into<String>,
        urdf: impl Into<PathBuf>,
        srdf: Option<PathBuf>,
        base_transform: Isometry3<f64>,
    ) -> Self {
        Self {
            robot_id: robot_id.into(),
            urdf: urdf.into(),
            srdf,
            base_transform,
        }
    }

    /// Validate that the specification is well-formed: non-empty `robot_id`,
    /// non-empty URDF path, and (when present) a non-empty SRDF path.
    pub fn validate(&self) -> Result<(), String> {
        if self.robot_id.is_empty() {
            return Err("RobotCollisionSpec.robot_id must not be empty".to_string());
        }
        if self.urdf.as_os_str().is_empty() {
            return Err(format!(
                "RobotCollisionSpec.urdf for robot `{}` must not be empty",
                self.robot_id
            ));
        }
        if let Some(srdf) = &self.srdf
            && srdf.as_os_str().is_empty()
        {
            return Err(format!(
                "RobotCollisionSpec.srdf for robot `{}` must not be empty when supplied",
                self.robot_id
            ));
        }
        Ok(())
    }
}

/// A full-scene joint configuration: a mapping from `robot_id` to that robot's
/// 6-DOF joint vector (radians), consumed by
/// [`SceneCollisionChecker::check_configuration`].
///
/// A robot with no entry is treated as *unchecked* and excluded from all
/// cross-model checks while the remaining robots are still evaluated (Req 9.4).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SceneConfig {
    joints: HashMap<String, [f64; 6]>,
}

impl SceneConfig {
    /// An empty configuration (no robot joints supplied).
    pub fn new() -> Self {
        Self {
            joints: HashMap::new(),
        }
    }

    /// Insert (or replace) a robot's joint vector, builder style.
    pub fn with_robot(mut self, robot_id: impl Into<String>, joints: [f64; 6]) -> Self {
        self.joints.insert(robot_id.into(), joints);
        self
    }

    /// Insert (or replace) a robot's joint vector.
    pub fn insert(&mut self, robot_id: impl Into<String>, joints: [f64; 6]) {
        self.joints.insert(robot_id.into(), joints);
    }

    /// The joint vector for `robot_id`, or `None` when the robot is not present
    /// (⇒ unchecked, Req 9.4).
    pub fn get(&self, robot_id: &str) -> Option<&[f64; 6]> {
        self.joints.get(robot_id)
    }

    /// Number of robots with a supplied joint vector.
    pub fn len(&self) -> usize {
        self.joints.len()
    }

    /// Whether no robot joint vectors are supplied.
    pub fn is_empty(&self) -> bool {
        self.joints.is_empty()
    }
}

/// One colliding sampled waypoint along a discrete trajectory check.
#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryCollisionWaypoint {
    /// Normalized progress `s` in `[0.0, 1.0]` at which the collision was
    /// sampled.
    pub progress: f64,
    /// The knot index (0-based) sampled.
    pub waypoint: usize,
    /// Full-scene collision report at this waypoint.
    pub report: CollisionReport,
}

/// The result of a discrete trajectory check across per-robot `JointPath`
/// samples.
#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryCollisionReport {
    /// `true` iff at least one sampled waypoint was in collision.
    pub collided: bool,
    /// The colliding waypoint with the smallest progress value, if any
    /// (Req 6.3).
    pub first: Option<TrajectoryCollisionWaypoint>,
    /// Every colliding sampled waypoint in ascending progress order, populated
    /// when report-all is enabled (Req 6.4).
    pub all: Vec<TrajectoryCollisionWaypoint>,
    /// Set to a descriptive message when the request was rejected because the
    /// sample count was out of range; when set, no collision checking was
    /// performed (Req 6.8).
    pub rejected: Option<String>,
    /// The robots skipped as *unchecked* because their [`JointPath`] is the
    /// `None` variant or absent from the trajectory (Req 6.7). When every robot
    /// is unchecked the trajectory is reported as not colliding (Req 6.9).
    pub unchecked_robots: Vec<String>,
}

impl TrajectoryCollisionReport {
    /// A report indicating the trajectory did not collide (Req 6.6, 6.9).
    pub fn clear() -> Self {
        Self {
            collided: false,
            first: None,
            all: Vec::new(),
            rejected: None,
            unchecked_robots: Vec::new(),
        }
    }

    /// A report indicating the request was rejected (invalid sample count) with
    /// no collision checking performed (Req 6.8).
    pub fn rejected(reason: impl Into<String>) -> Self {
        Self {
            rejected: Some(reason.into()),
            ..Self::clear()
        }
    }

    /// A clear report recording that no checkable robots remained because every
    /// robot's path is the `None` variant or absent (Req 6.9).
    pub fn no_checkable_robots(unchecked_robots: Vec<String>) -> Self {
        Self {
            unchecked_robots,
            ..Self::clear()
        }
    }
}

/// A discrete scene trajectory: per-robot [`JointPath`]s plus the total knot
/// count and the number of synchronized samples `N` to check.
///
/// [`SceneCollisionChecker::check_trajectory`] samples a synchronized progress
/// `s = k / N ∈ [0.0, 1.0]` across all robots at every `waypoint_stride`-th
/// index `k` (Req 6.1). A robot whose path is [`JointPath::None`] (or absent
/// from the map) is treated as *unchecked* and skipped, while the remaining
/// robots are still checked (Req 6.7). The sample count `N` must lie within
/// `[1, total_knots]`; otherwise the request is rejected and no checking is
/// performed (Req 6.8).
#[derive(Debug, Clone, Default)]
pub struct SceneTrajectory {
    /// Per-robot joint paths keyed by `robot_id`.
    paths: HashMap<String, JointPath>,
    /// Total number of knots available across the trajectory; the inclusive
    /// upper bound for a valid sample count (Req 6.1, 6.8).
    total_knots: usize,
    /// Number of synchronized samples `N` to check; valid range `[1, total_knots]`.
    sample_count: usize,
}

impl SceneTrajectory {
    /// Create a trajectory with the given `total_knots` and `sample_count` and
    /// no paths yet.
    pub fn new(total_knots: usize, sample_count: usize) -> Self {
        Self {
            paths: HashMap::new(),
            total_knots,
            sample_count,
        }
    }

    /// Insert (or replace) a robot's joint path, builder style.
    pub fn with_path(mut self, robot_id: impl Into<String>, path: JointPath) -> Self {
        self.paths.insert(robot_id.into(), path);
        self
    }

    /// Insert (or replace) a robot's joint path.
    pub fn insert(&mut self, robot_id: impl Into<String>, path: JointPath) {
        self.paths.insert(robot_id.into(), path);
    }

    /// The joint path for `robot_id`, or `None` when it is absent.
    pub fn get(&self, robot_id: &str) -> Option<&JointPath> {
        self.paths.get(robot_id)
    }

    /// The total knot count (inclusive upper bound for the sample count).
    pub fn total_knots(&self) -> usize {
        self.total_knots
    }

    /// The configured sample count `N`.
    pub fn sample_count(&self) -> usize {
        self.sample_count
    }
}

/* ========================================================================= */
/* C ABI bindings and the safe `CollisionModel` RAII wrapper                 */
/*                                                                           */
/* Mirrors the FFI / RAII / `Send` conventions of `src/pinocchio.rs`         */
/* (`PinocchioModel`). The whole module is gated behind the `collision`      */
/* Cargo feature (see `main.rs`), so these bindings and the Coal-backed       */
/* symbols they reference are excluded from the default artifact entirely.   */
/* ========================================================================= */

/// Upper bound on colliding pairs written by a single `collision_check` call.
///
/// The C ABI honours this as `max_out` and reports truncation via its return
/// contract; a robot's registered self-collision pair set is far smaller than
/// this in practice, so truncation is not expected.
const MAX_COLLISION_PAIRS: usize = 256;

/// Upper bound on collision geometry objects a model may expose.
///
/// `collision_geometry_placements` has no `max_out` parameter in the frozen C
/// header (the caller must pre-size `placements_out` to `7 * num_objects`).
/// We therefore pre-allocate a generous fixed buffer and clamp the reported
/// object count to this capacity so a mis-sized callee can never make us read
/// past the buffer. A 6-DOF industrial URDF exposes on the order of ten
/// collision objects, well under this bound.
const MAX_GEOMETRY_OBJECTS: usize = 512;

/// Number of `f64` per geometry placement: `[x, y, z, qx, qy, qz, qw]`.
const PLACEMENT_STRIDE: usize = 7;

/// Number of `f64` per geometry half-extent triple: `[hx, hy, hz]`.
const HALF_EXTENT_STRIDE: usize = 3;

/// Minimum half-extent (metres) substituted for any missing, out-of-range, or
/// non-finite axis reported by [`CollisionModel::geometry_half_extents`], so
/// every robot link `WorldCollisionObject` always carries a strictly positive
/// half-extent on every axis (Req 7.1, 7.4).
const MIN_HALF_EXTENT_M: f64 = 1e-4;

/// FFI-compatible mirror of the C `CollisionPairOut` struct
/// (`pinocchio_wrapper.h`). `name_a` / `name_b` are borrowed pointers into the
/// context that remain valid only until the next `collision_*` call, so they
/// are copied into owned `String`s before any subsequent FFI call (Req 10.8).
#[repr(C)]
#[derive(Clone, Copy)]
struct CollisionPairOut {
    geom_a: c_int,
    geom_b: c_int,
    name_a: *const c_char,
    name_b: *const c_char,
    distance: f64,
}

impl CollisionPairOut {
    /// A zeroed entry used to initialise the out-buffer before an FFI call.
    const fn empty() -> Self {
        Self {
            geom_a: 0,
            geom_b: 0,
            name_a: std::ptr::null(),
            name_b: std::ptr::null(),
            distance: 0.0,
        }
    }
}

mod ffi {
    use super::CollisionPairOut;
    use std::ffi::c_void;
    use std::os::raw::{c_char, c_int};

    unsafe extern "C" {
        pub fn collision_load_urdf(
            urdf_path: *const c_char,
            srdf_path: *const c_char,
            error_msg: *mut *mut c_char,
        ) -> *mut c_void;

        pub fn collision_num_pairs(ctx: *const c_void) -> c_int;

        pub fn collision_check(
            ctx: *mut c_void,
            q: *const f64,
            stop_at_first: c_int,
            out: *mut CollisionPairOut,
            max_out: c_int,
            num_out: *mut c_int,
            error_msg: *mut *mut c_char,
        ) -> c_int;

        pub fn collision_min_distance(
            ctx: *mut c_void,
            q: *const f64,
            out: *mut CollisionPairOut,
            min_distance_out: *mut f64,
            error_msg: *mut *mut c_char,
        ) -> c_int;

        pub fn collision_geometry_placements(
            ctx: *mut c_void,
            q: *const f64,
            placements_out: *mut f64,
            num_objects_out: *mut c_int,
            error_msg: *mut *mut c_char,
        ) -> c_int;

        pub fn collision_geometry_half_extents(
            ctx: *mut c_void,
            half_extents_out: *mut f64,
            num_objects_out: *mut c_int,
            error_msg: *mut *mut c_char,
        ) -> c_int;

        pub fn collision_geometry_local_centers(
            ctx: *mut c_void,
            centers_out: *mut f64,
            num_objects_out: *mut c_int,
            error_msg: *mut *mut c_char,
        ) -> c_int;

        pub fn collision_free(ctx: *mut c_void);

        // Reused from the existing Pinocchio wrapper: frees an error string
        // allocated by the C side (Req 10 error-string ownership).
        pub fn pinocchio_free_string(s: *mut c_char);
    }
}

/// The world/base-frame placement of one collision geometry object.
///
/// The `collision_geometry_placements` C ABI reports poses (as
/// `[x, y, z, qx, qy, qz, qw]` 7-tuples) and an object count, but not object
/// names. Names are therefore synthesised as `geom{index}` in registration
/// order so the scene layer (task 6.x) can correlate placements across models
/// and qualify them as `<robot_id>/<link_or_geom>` when composing world-frame
/// cross-model checks.
#[derive(Debug, Clone, PartialEq)]
pub struct GeometryPlacement {
    /// Synthesised, registration-order object name (`geom{index}`).
    pub name: String,
    /// Zero-based index of the object in the model's geometry registration order.
    pub index: usize,
    /// Rigid placement of the object in the model's base frame.
    pub pose: Isometry3<f64>,
}

impl GeometryPlacement {
    /// Build a placement from a raw `[x, y, z, qx, qy, qz, qw]` tuple as written
    /// by `collision_geometry_placements`.
    fn from_tuple(index: usize, t: &[f64; PLACEMENT_STRIDE]) -> Self {
        let translation = Translation3::new(t[0], t[1], t[2]);
        // nalgebra's `Quaternion::new` takes (w, i, j, k) == (qw, qx, qy, qz).
        let quat = UnitQuaternion::from_quaternion(Quaternion::new(t[6], t[3], t[4], t[5]));
        Self {
            name: format!("geom{index}"),
            index,
            pose: Isometry3::from_parts(translation, quat),
        }
    }
}

/// Safe RAII wrapper over one `CollisionContext` (one robot model, or the
/// environment). Analogous to [`crate::pinocchio::PinocchioModel`].
///
/// The optional `robot_id` lets [`CollisionModel::check_self`] emit
/// fully-qualified `<robot_id>/<link_or_geom>` object names (Req 4.1). When it
/// is unset the raw link-level geometry names reported by Coal are used, and
/// the scene layer performs qualification. Either way the returned
/// [`CollisionPair`]s are normalized (ordered, deduplicated, identical-name
/// contacts dropped) by [`CollisionPair::new`].
pub struct CollisionModel {
    ctx: *mut c_void,
    robot_id: Option<String>,
}

// Safe for the same one-owner-per-model reason as `PinocchioModel`: a
// `CollisionModel` uniquely owns its context and never shares it.
unsafe impl Send for CollisionModel {}

impl CollisionModel {
    /// Load a collision model from a dressed URDF and an optional SRDF
    /// self-collision filter.
    ///
    /// On success, when the model registers zero collision pairs (e.g. the URDF
    /// carries no `<collision>` geometry) a warning naming the URDF is logged and
    /// loading still succeeds (Req 1.6). On failure the borrowed C error string
    /// is copied into an owned `String`, freed via `pinocchio_free_string`, and
    /// returned as `Err`; this never panics (Req 1.11).
    pub fn from_urdf(urdf: &Path, srdf: Option<&Path>) -> Result<Self, String> {
        let urdf_c = path_to_cstring(urdf, "URDF")?;
        // Keep the SRDF CString alive for the duration of the FFI call; pass a
        // null pointer when no SRDF is supplied (Req 1.4).
        let srdf_c = match srdf {
            Some(p) => Some(path_to_cstring(p, "SRDF")?),
            None => None,
        };
        let srdf_ptr = srdf_c
            .as_ref()
            .map_or(std::ptr::null(), |c| c.as_ptr());

        let mut error_msg: *mut c_char = std::ptr::null_mut();
        let ctx = unsafe { ffi::collision_load_urdf(urdf_c.as_ptr(), srdf_ptr, &mut error_msg) };

        if ctx.is_null() {
            return Err(take_error(error_msg, "unknown error loading collision model"));
        }

        let model = Self {
            ctx,
            robot_id: None,
        };

        // Req 1.6: warn-and-continue when the model has no collision pairs.
        if model.num_pairs() == 0 {
            eprintln!(
                "WARN: collision model loaded from URDF '{}' has zero collision pairs; \
                 this robot cannot self-collide or participate in cross-model checks",
                urdf.display()
            );
        }

        Ok(model)
    }

    /// Associate this model with a robot identifier so reported object names are
    /// fully qualified as `<robot_id>/<link_or_geom>` (Req 4.1). Builder style.
    pub fn with_robot_id(mut self, robot_id: impl Into<String>) -> Self {
        self.robot_id = Some(robot_id.into());
        self
    }

    /// Set (or clear) the robot identifier used to qualify object names.
    pub fn set_robot_id(&mut self, robot_id: Option<String>) {
        self.robot_id = robot_id;
    }

    /// The robot identifier used to qualify reported object names, if any.
    pub fn robot_id(&self) -> Option<&str> {
        self.robot_id.as_deref()
    }

    /// Number of registered (post-SRDF) collision pairs (Req 1.2, 1.5).
    pub fn num_pairs(&self) -> usize {
        let n = unsafe { ffi::collision_num_pairs(self.ctx as *const c_void) };
        // The C ABI returns a non-negative count; clamp defensively.
        n.max(0) as usize
    }

    /// Self-collision check for one 6-DOF configuration (radians).
    ///
    /// Validates that every joint value is finite before touching the FFI
    /// (Req 2.8); the fixed `[f64; 6]` array type enforces the exact-6 length at
    /// compile time (Req 2.7). Returns the colliding self-pairs (empty ⇒ clear),
    /// each with `distance_m <= contact_margin`; with `stop_at_first` the result
    /// holds at most one pair (Req 2.2–2.4). Any FFI error is returned as `Err`
    /// and never panics (Req 2.9, 1.11).
    pub fn check_self(
        &mut self,
        joints: &[f64; 6],
        stop_at_first: bool,
    ) -> Result<Vec<CollisionPair>, String> {
        validate_joints_finite(joints)?;

        let mut out = vec![CollisionPairOut::empty(); MAX_COLLISION_PAIRS];
        let mut num_out: c_int = 0;
        let mut error_msg: *mut c_char = std::ptr::null_mut();

        let ret = unsafe {
            ffi::collision_check(
                self.ctx,
                joints.as_ptr(),
                c_int::from(stop_at_first),
                out.as_mut_ptr(),
                MAX_COLLISION_PAIRS as c_int,
                &mut num_out,
                &mut error_msg,
            )
        };

        // Return codes: 1 => collision(s) found, 0 => clear, -1 => error.
        if ret < 0 {
            return Err(take_error(error_msg, "collision check failed"));
        }
        // A non-error call never allocates an error string, but free defensively
        // in case the C side set one alongside a success code.
        free_error_if_set(error_msg);

        // Clamp the reported count to the buffer capacity so a mis-sized callee
        // cannot make us read uninitialised / out-of-bounds entries (Req 10.1/10.2).
        let n = (num_out.max(0) as usize).min(MAX_COLLISION_PAIRS);

        let mut pairs = Vec::with_capacity(n);
        for entry in &out[..n] {
            // Req 10.8: copy borrowed name pointers into owned Strings now,
            // before any further FFI call could invalidate them.
            let name_a = borrowed_name(entry.name_a);
            let name_b = borrowed_name(entry.name_b);

            // Qualify with robot_id when known (best-effort); otherwise use the
            // raw link-level names and let the scene layer qualify.
            let (qual_a, qual_b) = match &self.robot_id {
                Some(id) => (
                    CollisionPair::qualify_name(id, &name_a).unwrap_or(name_a),
                    CollisionPair::qualify_name(id, &name_b).unwrap_or(name_b),
                ),
                None => (name_a, name_b),
            };

            // `CollisionPair::new` normalizes ordering, drops identical-name
            // contacts, and rejects empty names (Req 4.2, 4.3).
            if let Some(pair) =
                CollisionPair::new(qual_a, qual_b, CollisionKind::SelfCollision, entry.distance)
            {
                pairs.push(pair);
            }
        }

        Ok(pairs)
    }

    /// Base-frame placements of every collision geometry object at `joints`,
    /// later composed with the model's base transform by the scene layer.
    ///
    /// Validates joint finiteness before the FFI (Req 2.8); returns `Err` on any
    /// FFI error without panicking (Req 2.9, 3.2).
    pub fn geometry_placements(
        &mut self,
        joints: &[f64; 6],
    ) -> Result<Vec<GeometryPlacement>, String> {
        validate_joints_finite(joints)?;

        let mut buffer = vec![0.0f64; PLACEMENT_STRIDE * MAX_GEOMETRY_OBJECTS];
        let mut num_objects: c_int = 0;
        let mut error_msg: *mut c_char = std::ptr::null_mut();

        let ret = unsafe {
            ffi::collision_geometry_placements(
                self.ctx,
                joints.as_ptr(),
                buffer.as_mut_ptr(),
                &mut num_objects,
                &mut error_msg,
            )
        };

        if ret != 0 {
            return Err(take_error(error_msg, "geometry placement query failed"));
        }
        free_error_if_set(error_msg);

        let n = (num_objects.max(0) as usize).min(MAX_GEOMETRY_OBJECTS);

        let mut placements = Vec::with_capacity(n);
        for i in 0..n {
            let base = i * PLACEMENT_STRIDE;
            let tuple: [f64; PLACEMENT_STRIDE] = buffer[base..base + PLACEMENT_STRIDE]
                .try_into()
                .expect("slice length equals PLACEMENT_STRIDE");
            placements.push(GeometryPlacement::from_tuple(i, &tuple));
        }

        Ok(placements)
    }

    /// Local-frame axis-aligned half-extents of every collision geometry
    /// object, in the same registration order as
    /// [`CollisionModel::geometry_placements`] (Req 7.1, 7.2).
    ///
    /// This is a static property of the loaded geometry (independent of joint
    /// configuration), so the FFI call takes no joint configuration and can be
    /// made once per model rather than once per check. Every returned
    /// component is guaranteed strictly greater than zero: an FFI error, an
    /// out-of-range object count, or a non-finite/non-positive component is
    /// replaced with [`MIN_HALF_EXTENT_M`] rather than propagated as zero or
    /// `Err` (Req 7.4). This method never panics.
    pub fn geometry_half_extents(&self) -> Result<Vec<Vector3<f64>>, String> {
        let mut buffer = vec![0.0f64; HALF_EXTENT_STRIDE * MAX_GEOMETRY_OBJECTS];
        let mut num_objects: c_int = 0;
        let mut error_msg: *mut c_char = std::ptr::null_mut();

        let ret = unsafe {
            ffi::collision_geometry_half_extents(
                self.ctx,
                buffer.as_mut_ptr(),
                &mut num_objects,
                &mut error_msg,
            )
        };

        if ret != 0 {
            return Err(take_error(error_msg, "geometry half-extent query failed"));
        }
        free_error_if_set(error_msg);

        let n = (num_objects.max(0) as usize).min(MAX_GEOMETRY_OBJECTS);

        let mut half_extents = Vec::with_capacity(n);
        for i in 0..n {
            let base = i * HALF_EXTENT_STRIDE;
            let sanitize = |v: f64| if v.is_finite() && v > 0.0 { v } else { MIN_HALF_EXTENT_M };
            half_extents.push(Vector3::new(
                sanitize(buffer[base]),
                sanitize(buffer[base + 1]),
                sanitize(buffer[base + 2]),
            ));
        }

        Ok(half_extents)
    }

    /// Centre of every collision geometry object's local-frame AABB, in its own
    /// frame and in the registration order of
    /// [`CollisionModel::geometry_placements`]. The box of
    /// [`geometry_half_extents`](Self::geometry_half_extents) sits at
    /// `placement * translation(centre)`: a link mesh is rarely centred on its
    /// link frame. A non-finite component is reported as `0.0`.
    pub fn geometry_local_centers(&self) -> Result<Vec<Vector3<f64>>, String> {
        let mut buffer = vec![0.0f64; HALF_EXTENT_STRIDE * MAX_GEOMETRY_OBJECTS];
        let mut num_objects: c_int = 0;
        let mut error_msg: *mut c_char = std::ptr::null_mut();

        let ret = unsafe {
            ffi::collision_geometry_local_centers(
                self.ctx,
                buffer.as_mut_ptr(),
                &mut num_objects,
                &mut error_msg,
            )
        };
        if ret != 0 {
            return Err(take_error(error_msg, "geometry local-centre query failed"));
        }
        free_error_if_set(error_msg);

        let n = (num_objects.max(0) as usize).min(MAX_GEOMETRY_OBJECTS);
        let finite = |v: f64| if v.is_finite() { v } else { 0.0 };
        Ok((0..n)
            .map(|i| {
                let b = i * HALF_EXTENT_STRIDE;
                Vector3::new(finite(buffer[b]), finite(buffer[b + 1]), finite(buffer[b + 2]))
            })
            .collect())
    }
}

impl Drop for CollisionModel {
    fn drop(&mut self) {
        // Req 10.9: free the context exactly once. The null guard prevents a
        // double free should the pointer ever be taken/cleared in the future.
        if !self.ctx.is_null() {
            unsafe { ffi::collision_free(self.ctx) };
            self.ctx = std::ptr::null_mut();
        }
    }
}

/// Convert a filesystem path into a `CString`, naming the offending path/kind on
/// failure (Req 1.7–1.10 surface these as load errors without panicking).
fn path_to_cstring(path: &Path, kind: &str) -> Result<CString, String> {
    let s = path
        .to_str()
        .ok_or_else(|| format!("{kind} path is not valid UTF-8: '{}'", path.display()))?;
    CString::new(s).map_err(|_| format!("{kind} path contains an interior null byte: '{s}'"))
}

/// Copy a borrowed C string pointer into an owned Rust `String`, treating a null
/// pointer as an empty name (Req 10.8).
fn borrowed_name(ptr: *const c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(ptr) }
        .to_string_lossy()
        .into_owned()
}

/// Take ownership of a C-allocated error string, freeing it via
/// `pinocchio_free_string`, and fall back to `fallback` when unset.
fn take_error(error_msg: *mut c_char, fallback: &str) -> String {
    if error_msg.is_null() {
        return fallback.to_string();
    }
    let s = unsafe { CStr::from_ptr(error_msg) }
        .to_string_lossy()
        .into_owned();
    unsafe { ffi::pinocchio_free_string(error_msg) };
    s
}

/// Free a C-allocated error string if one was set, discarding its contents.
fn free_error_if_set(error_msg: *mut c_char) {
    if !error_msg.is_null() {
        unsafe { ffi::pinocchio_free_string(error_msg) };
    }
}

/// Reject any non-finite joint value (NaN, ±inf) before crossing the FFI
/// boundary (Req 2.8). The `[f64; 6]` array type enforces the length (Req 2.7).
fn validate_joints_finite(joints: &[f64; 6]) -> Result<(), String> {
    if let Some((i, v)) = joints.iter().enumerate().find(|(_, v)| !v.is_finite()) {
        return Err(format!(
            "joint configuration contains a non-finite value at index {i}: {v}"
        ));
    }
    Ok(())
}

/* ========================================================================= */
/* World-frame composition, broadphase (world AABB), and narrowphase         */
/* orchestration between two placement sets (task 5.1)                       */
/*                                                                           */
/* These functions implement the cross-model geometry of the design's       */
/* `narrowphase(placementsA, placementsB, margin)` pseudocode:               */
/*   1. compose each robot's base-frame link placements with its             */
/*      `Base_Transform` into the shared world frame (world <- base <- link);*/
/*   2. compute a margin-expanded world AABB per object and gate every       */
/*      cross-object pair on `aabb_overlap` (broadphase);                    */
/*   3. run an exact distance query (injected `distance_fn`) only for pairs  */
/*      whose expanded AABBs overlap, emitting a normalized `CollisionPair`  */
/*      when the separation is `<= contact_margin` (Req 3.6, 3.7, 3.8).      */
/*                                                                           */
/* The distance query is passed in as a closure so this logic is unit-      */
/* testable with hand-placed AABBs and distances, without a loaded Coal      */
/* model (task 5.2). The scene layer (task 6.x) wires the real Coal query    */
/* (via `CollisionModel`) into `distance_fn`.                                */
/* ========================================================================= */

/// An axis-aligned bounding box in the world frame, defined by its inclusive
/// `min` and `max` corners (`min[i] <= max[i]` on every axis).
///
/// Used as the cheap broadphase proxy that gates the exact narrowphase distance
/// query (Req 3.6).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Lower corner (`min[i] <= max[i]`).
    pub min: Vector3<f64>,
    /// Upper corner (`min[i] <= max[i]`).
    pub max: Vector3<f64>,
}

impl Aabb {
    /// Construct an AABB from two corners, normalizing so that `min <= max` on
    /// every axis regardless of the order the corners are supplied.
    pub fn new(a: Vector3<f64>, b: Vector3<f64>) -> Self {
        Self {
            min: Vector3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: Vector3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// Build an AABB centred at `center` with the given non-negative
    /// `half_extents` on each axis (an axis-aligned box).
    pub fn from_center_half_extents(center: Vector3<f64>, half_extents: Vector3<f64>) -> Self {
        let h = Vector3::new(
            half_extents.x.abs(),
            half_extents.y.abs(),
            half_extents.z.abs(),
        );
        Self {
            min: center - h,
            max: center + h,
        }
    }

    /// Compute the world AABB that tightly encloses an oriented box with the
    /// given local `half_extents`, placed at `pose` (world <- object).
    ///
    /// This is the standard oriented-box → axis-aligned-box projection: the
    /// world half-extent along axis `i` is `sum_j |R[i, j]| * half_extents[j]`,
    /// where `R` is the rotation of `pose`. The result is centred at the pose
    /// translation. It over-approximates the true geometry (never smaller than
    /// the real shape's AABB for a box bound), which keeps the broadphase
    /// conservative — it never wrongly rejects a potentially-colliding pair.
    pub fn from_pose_half_extents(pose: &Isometry3<f64>, half_extents: Vector3<f64>) -> Self {
        let r = pose.rotation.to_rotation_matrix();
        let m = r.matrix();
        let h = Vector3::new(
            half_extents.x.abs(),
            half_extents.y.abs(),
            half_extents.z.abs(),
        );
        let world_half = Vector3::new(
            m[(0, 0)].abs() * h.x + m[(0, 1)].abs() * h.y + m[(0, 2)].abs() * h.z,
            m[(1, 0)].abs() * h.x + m[(1, 1)].abs() * h.y + m[(1, 2)].abs() * h.z,
            m[(2, 0)].abs() * h.x + m[(2, 1)].abs() * h.y + m[(2, 2)].abs() * h.z,
        );
        Self::from_center_half_extents(pose.translation.vector, world_half)
    }

    /// Return a copy of this box grown outward by `margin` on every axis.
    ///
    /// A negative `margin` is clamped to `0.0` so expansion never shrinks the
    /// box (contact margins are non-negative by construction, Req 7.4).
    pub fn expanded(&self, margin: f64) -> Self {
        let m = margin.max(0.0);
        let grow = Vector3::new(m, m, m);
        Self {
            min: self.min - grow,
            max: self.max + grow,
        }
    }

    /// Test whether this box intersects `other`, counting a shared boundary
    /// (touching faces) as an intersection via `<=` comparisons.
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

/// Broadphase gate: do the two world AABBs, each expanded outward by
/// `contact_margin`, overlap? (Req 3.6, 3.8.)
///
/// Expanding each box by the margin follows the design/requirement wording
/// ("each object's world-frame AABB expanded by the Contact_Margin"). Because
/// an AABB is a conservative bound on the enclosed geometry and the expansion
/// only grows the boxes, this gate never rejects a pair whose true world-frame
/// separation is `<= contact_margin`; it only ever prunes pairs that cannot
/// possibly collide, so the exact narrowphase result is preserved. Returns
/// `true` when the narrowphase distance query should be attempted for the pair.
pub fn aabb_overlap(a: &Aabb, b: &Aabb, contact_margin: f64) -> bool {
    a.expanded(contact_margin)
        .overlaps(&b.expanded(contact_margin))
}

/// Signed world-frame separation between two axis-aligned boxes, used by the
/// scene layer as the cross-model proxy distance (see the note on
/// [`SceneCollisionChecker::check_configuration`]).
///
/// - When the boxes are disjoint, returns the positive Euclidean gap between
///   their nearest faces (`> 0`).
/// - When the boxes overlap, returns the negative penetration depth: the
///   smallest per-axis overlap, negated (`<= 0`), so that touching faces yield
///   exactly `0.0` and deeper interpenetration yields a more negative value.
///
/// This signedness lets the margin semantics of Req 7.1 (zero margin ⇒
/// penetration only, `distance < 0`) and Req 7.2 (`distance <= margin`) apply
/// uniformly to cross-model pairs even though the exact convex-hull distance is
/// not exposed across the C ABI for independently loaded models.
pub fn aabb_signed_distance(a: &Aabb, b: &Aabb) -> f64 {
    // Per-axis gap: positive when separated on that axis, non-positive when the
    // intervals overlap (the value is then the negative overlap length).
    let gap = Vector3::new(
        (a.min.x - b.max.x).max(b.min.x - a.max.x),
        (a.min.y - b.max.y).max(b.min.y - a.max.y),
        (a.min.z - b.max.z).max(b.min.z - a.max.z),
    );

    if gap.x > 0.0 || gap.y > 0.0 || gap.z > 0.0 {
        // Disjoint on at least one axis: Euclidean distance over the positive
        // (separating) gaps only.
        let dx = gap.x.max(0.0);
        let dy = gap.y.max(0.0);
        let dz = gap.z.max(0.0);
        (dx * dx + dy * dy + dz * dz).sqrt()
    } else {
        // Overlapping on every axis: penetration depth is the least-overlapping
        // axis (the axis whose gap is closest to zero). Negated so contact == 0.
        gap.x.max(gap.y).max(gap.z)
    }
}

/// Decide whether a signed separation `distance` counts as a collision under
/// `margin` semantics:
///
/// - `margin == 0.0` ⇒ colliding only on strict penetration (`distance < 0.0`,
///   Req 7.1);
/// - `margin > 0.0` ⇒ colliding when `distance <= margin` (Req 7.2).
fn margin_collides(distance: f64, margin: f64) -> bool {
    if margin <= 0.0 {
        distance < 0.0
    } else {
        distance <= margin
    }
}

/// One collision object expressed in the shared world frame: a fully-qualified
/// name, its rigid world pose, and its world-frame AABB used for broadphase.
///
/// `GeometryPlacement` alone carries only a pose (Coal owns the geometry
/// extents), so the world AABB is supplied explicitly — either computed from a
/// pose plus box half-extents ([`WorldCollisionObject::from_pose_half_extents`])
/// or injected directly in tests ([`WorldCollisionObject::new`]).
#[derive(Debug, Clone, PartialEq)]
pub struct WorldCollisionObject {
    /// Fully-qualified object name (`<robot_id>/<link_or_geom>`), used verbatim
    /// when constructing a [`CollisionPair`].
    pub name: String,
    /// Rigid placement of the object in the world frame (world <- object).
    pub pose: Isometry3<f64>,
    /// World-frame axis-aligned bounding box used by the broadphase gate.
    pub aabb: Aabb,
}

impl WorldCollisionObject {
    /// Construct a world object from an explicit name, pose, and AABB. This is
    /// the direct form tests use to hand-place objects without a loaded model.
    pub fn new(name: impl Into<String>, pose: Isometry3<f64>, aabb: Aabb) -> Self {
        Self {
            name: name.into(),
            pose,
            aabb,
        }
    }

    /// Construct a world object from a world pose and box half-extents,
    /// computing the enclosing world AABB via [`Aabb::from_pose_half_extents`].
    pub fn from_pose_half_extents(
        name: impl Into<String>,
        pose: Isometry3<f64>,
        half_extents: Vector3<f64>,
    ) -> Self {
        let aabb = Aabb::from_pose_half_extents(&pose, half_extents);
        Self {
            name: name.into(),
            pose,
            aabb,
        }
    }

    /// Construct a world object from an already world-composed
    /// [`GeometryPlacement`] and box half-extents, qualifying the object name as
    /// `<robot_id>/<placement.name>` (Req 4.1). Returns `None` when the name
    /// cannot be qualified (empty component).
    pub fn from_world_placement(
        robot_id: &str,
        placement: &GeometryPlacement,
        half_extents: Vector3<f64>,
    ) -> Option<Self> {
        let name = CollisionPair::qualify_name(robot_id, &placement.name)?;
        Some(Self::from_pose_half_extents(name, placement.pose, half_extents))
    }
}

/// Compose a robot's base transform with its base-frame link placements to
/// produce world-frame placements (world <- base <- link), per step 2 of the
/// scene-configuration check (Req 3.2).
///
/// Each returned placement keeps the source object's synthesised `name` and
/// registration `index`; only the pose is lifted into the world frame.
pub fn compose_world_placements(
    base_transform: &Isometry3<f64>,
    local: &[GeometryPlacement],
) -> Vec<GeometryPlacement> {
    local
        .iter()
        .map(|p| GeometryPlacement {
            name: p.name.clone(),
            index: p.index,
            pose: *base_transform * p.pose,
        })
        .collect()
}

/// Narrowphase orchestration between two sets of world-frame collision objects
/// (design's `narrowphase(placementsA, placementsB, margin)`).
///
/// For every cross pair `(a, b)` the margin-expanded world AABBs are tested with
/// [`aabb_overlap`]; pairs whose expanded AABBs do not overlap are treated as
/// not colliding and their exact distance query is skipped (Req 3.8). For pairs
/// that pass the gate, `distance_fn` computes the exact world-frame separation;
/// when it is `<= contact_margin` a normalized [`CollisionPair`] with the
/// caller-supplied `kind` is emitted (Req 3.6, 3.7). Pair ordering/dedup across
/// the whole report is the scene layer's responsibility ([`CollisionReport`]);
/// this function only normalizes each pair via [`CollisionPair::new`] (ordering
/// `object_a <= object_b`, dropping identical-name contacts).
///
/// `distance_fn` is fallible: a query error is propagated as `Err` so the scene
/// layer can record a per-model error and continue with the remaining models
/// (never-fail-hard, Req 9.3). Injecting a pure closure makes the broadphase +
/// gating logic testable without a loaded Coal model (task 5.2).
pub fn narrowphase<F>(
    a_objs: &[WorldCollisionObject],
    b_objs: &[WorldCollisionObject],
    contact_margin: f64,
    kind: CollisionKind,
    mut distance_fn: F,
) -> Result<Vec<CollisionPair>, String>
where
    F: FnMut(&WorldCollisionObject, &WorldCollisionObject) -> Result<f64, String>,
{
    let mut result = Vec::new();
    for a in a_objs {
        for b in b_objs {
            // Broadphase gate: skip the exact query when the expanded world
            // AABBs cannot overlap (Req 3.8).
            if !aabb_overlap(&a.aabb, &b.aabb, contact_margin) {
                continue;
            }
            // Narrowphase: exact world-frame separation for the gated pair.
            let distance = distance_fn(a, b)?;
            if distance <= contact_margin {
                if let Some(pair) =
                    CollisionPair::new(a.name.clone(), b.name.clone(), kind, distance)
                {
                    result.push(pair);
                }
            }
        }
    }
    Ok(result)
}

/* ========================================================================= */
/* Scene composition: `SceneCollisionChecker` (task 6.1)                     */
/*                                                                           */
/* Composes one loaded `CollisionModel` per robot at its world-frame         */
/* `Base_Transform`, plus an optional static environment model, under a      */
/* shared `CollisionConfig`. This task implements only the `build`           */
/* constructor (loading + validation, Req 3.1, 3.10); `check_configuration`  */
/* (task 6.2) and `check_trajectory` (task 8.1) are populated later.         */
/* ========================================================================= */

/// One robot participating in the scene: its loaded collision model, its
/// world-frame base transform (`world <- robot base`), and its stable
/// identifier used to qualify reported object names (Req 3.1, 4.1).
pub struct RobotCollision {
    /// Stable robot identifier (`<robot_id>` in fully-qualified object names).
    pub robot_id: String,
    /// Loaded collision model for this robot (owns one `CollisionContext`).
    pub model: CollisionModel,
    /// Rigid world-frame transform (`world <- robot base`) from `scene.yaml`,
    /// composed with base-frame link placements to produce world-frame poses.
    pub base_transform: Isometry3<f64>,
    /// Per geometry (registration order): fixed to the robot base, i.e. no
    /// joint moves it. Such geometry is not checked against what the robot
    /// stands on ([`EnvironmentRole::Support`]). Empty when unknown.
    pub base_fixed: Vec<bool>,
    /// Per geometry (registration order): carried by the last joint (the
    /// flange and the tool). Such geometry may touch a
    /// [`EnvironmentRole::Workpiece`]. Empty when unknown.
    pub tool_end: Vec<bool>,
    /// Per geometry (registration order): its local box, `(centre of its local
    /// AABB in its own frame, half-extents)`, queried once at build. `None`
    /// when that query failed.
    pub local_boxes: Option<Vec<(Vector3<f64>, Vector3<f64>)>>,
}

/// Each geometry's local box `(AABB centre, half-extents)`, or `None` when the
/// queries fail or disagree on the geometry count.
fn local_boxes(model: &CollisionModel) -> Option<Vec<(Vector3<f64>, Vector3<f64>)>> {
    let half_extents = model.geometry_half_extents().ok()?;
    let centers = model.geometry_local_centers().ok()?;
    (half_extents.len() == centers.len()).then(|| centers.into_iter().zip(half_extents).collect())
}

/// Classify a robot's geometry kinematically, as `(base_fixed, tool_end)` per
/// geometry: geometry whose pose no joint changes is fixed to the base, and
/// geometry whose pose the last joint changes is the tool end. The C ABI
/// carries no link names, so this compares placements at a few joint
/// configurations. On a query error both are empty (every geometry is then
/// treated as a moving arm link: checked against everything).
fn classify_geometries(model: &mut CollisionModel) -> (Vec<bool>, Vec<bool>) {
    const Q0: [f64; 6] = [0.0; 6];
    const Q_ALL: [f64; 6] = [0.31, 0.27, 0.23, 0.19, 0.17, 0.13];
    const Q_LAST: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
    let (Ok(at0), Ok(all), Ok(last)) = (
        model.geometry_placements(&Q0),
        model.geometry_placements(&Q_ALL),
        model.geometry_placements(&Q_LAST),
    ) else {
        return (Vec::new(), Vec::new());
    };
    if at0.len() != all.len() || at0.len() != last.len() {
        return (Vec::new(), Vec::new());
    }
    let same = |a: &Isometry3<f64>, b: &Isometry3<f64>| {
        (a.translation.vector - b.translation.vector).norm() < 1e-9
            && a.rotation.angle_to(&b.rotation) < 1e-9
    };
    let base_fixed = at0.iter().zip(&all).map(|(a, b)| same(&a.pose, &b.pose)).collect();
    let tool_end = at0.iter().zip(&last).map(|(a, b)| !same(&a.pose, &b.pose)).collect();
    (base_fixed, tool_end)
}

/// Composes per-robot and (optionally) environment collision models in a shared
/// world frame and answers scene-level queries (design Component 3).
///
/// Built once at startup from a set of [`RobotCollisionSpec`]s and an optional
/// environment URDF. This holds one loaded [`CollisionModel`] per robot together
/// with that robot's world-frame [`Base_Transform`](RobotCollision::base_transform),
/// an optional environment model, and the shared [`CollisionConfig`].
pub struct SceneCollisionChecker {
    /// Per-robot entries (loaded model + world base transform + robot_id).
    robots: Vec<RobotCollision>,
    /// Placed static environment: zero or more world-frame objects computed
    /// once at build time from [`EnvironmentCollisionSpec`]s (Req 5.2-5.6).
    environment: EnvironmentCollision,
    /// Shared configuration (margin, stride, report-all, per-class flags).
    config: CollisionConfig,
}

impl SceneCollisionChecker {
    /// Build the scene checker from robot specifications and a slice of
    /// static environment specifications.
    ///
    /// Validates the [`CollisionConfig`] and each [`RobotCollisionSpec`] up
    /// front, then loads exactly one [`CollisionModel`] per specification —
    /// associating each with its `robot_id` (for fully-qualified object names,
    /// Req 4.1) and its world-frame `base_transform` — plus the placed static
    /// environment (Req 3.1, 5.1).
    ///
    /// On any robot load or validation failure this returns a descriptive
    /// `Err` naming the offending specification or URDF rather than panicking
    /// (Req 3.10). Already-loaded models are dropped (freeing their contexts)
    /// as the error unwinds the partially-built `Vec`. Building the
    /// environment via [`EnvironmentCollision::build`] never fails — a bad or
    /// unloadable environment spec is recorded as an errored model rather than
    /// failing the whole checker (Req 5.5, 5.6, 9.1), so this only returns
    /// `Err` for the same robot/config reasons as before this change.
    pub fn build(
        robots: &[RobotCollisionSpec],
        env_specs: &[EnvironmentCollisionSpec],
        config: CollisionConfig,
    ) -> Result<Self, String> {
        Self::build_with_boxes(robots, env_specs, &[], config)
    }

    /// [`build`](Self::build), plus the cell's environment boxes: fixture
    /// boxes, pedestals and the floor ([`EnvironmentBoxSpec`]). A bad box is
    /// recorded as an errored environment object, never a build failure.
    pub fn build_with_boxes(
        robots: &[RobotCollisionSpec],
        env_specs: &[EnvironmentCollisionSpec],
        boxes: &[EnvironmentBoxSpec],
        config: CollisionConfig,
    ) -> Result<Self, String> {
        // Reject an invalid configuration before loading anything (Req 7.4-7.7).
        config
            .validate()
            .map_err(|e| format!("invalid collision configuration: {e}"))?;

        let mut loaded: Vec<RobotCollision> = Vec::with_capacity(robots.len());
        for spec in robots {
            // Validate the spec first so a malformed entry is named clearly
            // rather than surfacing as an opaque FFI load error (Req 3.10).
            spec.validate().map_err(|e| {
                format!("invalid robot collision spec `{}`: {e}", spec.robot_id)
            })?;

            // Load the model, naming the failing robot and URDF on error
            // (Req 3.10); `?` propagation drops any already-loaded models.
            let mut model = CollisionModel::from_urdf(&spec.urdf, spec.srdf.as_deref())
                .map_err(|e| {
                    format!(
                        "failed to load collision model for robot `{}` from URDF '{}': {e}",
                        spec.robot_id,
                        spec.urdf.display()
                    )
                })?
                .with_robot_id(spec.robot_id.clone());

            let (base_fixed, tool_end) = classify_geometries(&mut model);
            let local_boxes = local_boxes(&model);
            loaded.push(RobotCollision {
                robot_id: spec.robot_id.clone(),
                model,
                base_transform: spec.base_transform,
                base_fixed,
                tool_end,
                local_boxes,
            });
        }

        // Build the placed static environment (Req 5.1, 5.2). This is a total
        // function: per-object load/validation failures never fail the
        // checker (Req 5.5, 5.6, 9.1).
        let mut environment = EnvironmentCollision::build(env_specs);
        environment.add_boxes(boxes);

        Ok(Self {
            robots: loaded,
            environment,
            config,
        })
    }

    /// The scene's configuration.
    pub fn config(&self) -> &CollisionConfig {
        &self.config
    }

    /// Enable or disable "report all" mode on the shared [`CollisionConfig`].
    ///
    /// The default scene config uses stop-at-first (cheaper). The
    /// `preview_trajectory` MCP tool opts into report-all so it can surface
    /// every colliding waypoint/pair ordered by progress (Req 11.5).
    pub fn set_report_all(&mut self, report_all: bool) {
        self.config.report_all = report_all;
    }

    /// The number of robots loaded into the scene.
    pub fn robot_count(&self) -> usize {
        self.robots.len()
    }

    /// Whether at least one static environment object is present.
    pub fn has_environment(&self) -> bool {
        !self.environment.is_empty()
    }

    /// The number of successfully placed static environment objects.
    pub fn environment_object_count(&self) -> usize {
        self.environment.objects().len()
    }

    /// Every placed environment object with its role, in placement order.
    pub fn environment_objects(&self) -> impl Iterator<Item = (&WorldCollisionObject, &EnvironmentRole)> {
        self.environment.objects().iter().zip(self.environment.roles())
    }

    /// One [`ModelClassification`] per environment spec that failed to
    /// validate or load, each carrying `model_id: "environment/<id>"`,
    /// `status: Errored`, and a descriptive error message.
    ///
    /// Unlike the errored entries folded into a [`CollisionReport`]'s
    /// `model_status` by [`check_configuration`](Self::check_configuration),
    /// this reflects environment load failures directly and is available
    /// even when no configuration has been checked yet or when a checked
    /// trajectory never collides — so callers (e.g. `preview_trajectory`,
    /// Req 8.9) can surface a failed-to-load environment object without
    /// depending on a colliding waypoint existing.
    pub fn environment_errors(&self) -> &[ModelClassification] {
        self.environment.errored()
    }

    /// Check one full-scene configuration across every *enabled* collision
    /// class and return an aggregated [`CollisionReport`].
    ///
    /// Enabled classes are evaluated in a fixed, deterministic order (Req 8.1,
    /// 8.2):
    /// 1. **Self-collision**, per robot, via the model's real Coal query
    ///    ([`CollisionModel::check_self`], Req 3.5).
    /// 2. **Robot-robot**, over every unordered pair of distinct robots
    ///    (Req 3.3).
    /// 3. **Robot-environment**, each robot against the static environment
    ///    (Req 3.4).
    ///
    /// Cross-model classes compose each robot's base-frame geometry placements
    /// with its `Base_Transform` into the shared world frame
    /// ([`compose_world_placements`], Req 3.2), then run the broadphase +
    /// narrowphase pipeline ([`narrowphase`]) with a margin-expanded world-AABB
    /// gate (Req 3.6, 3.8).
    ///
    /// # Cross-model distance limitation
    ///
    /// The C ABI exposes an exact convex-hull distance query only *within* a
    /// single loaded model (`collision_check` / `collision_min_distance`); it
    /// cannot compute the exact separation between two objects that live in two
    /// independently loaded models placed at arbitrary world poses. For the
    /// robot-robot and robot-environment classes this method therefore uses the
    /// **signed world-AABB separation** ([`aabb_signed_distance`]) of the
    /// composed world placements as the best available exact-distance proxy.
    /// Self-collision, which stays inside one model, uses the real Coal query
    /// and is unaffected. Because each object's world AABB is a conservative
    /// bound, the proxy is conservative for separated objects; interpenetration
    /// depth is approximated from the AABB overlap. Callers that need
    /// sub-millimetre cross-model distances should treat these values as an
    /// upper bound on contact, not an exact surface distance.
    ///
    /// # Margin semantics (Req 7.1, 7.2)
    ///
    /// A pair is reported as colliding when a zero margin sees strict
    /// penetration (`distance < 0`) or a positive margin sees
    /// `distance <= margin` ([`margin_collides`]). `min_distance_m` is the
    /// smallest separation across every *evaluated* pair, or `None` when nothing
    /// was evaluated (Req 5.4, 5.5).
    ///
    /// # Robustness (Req 9.x)
    ///
    /// The input [`SceneConfig`] is never mutated (Req 9.2). A robot absent from
    /// the configuration is marked [`ModelStatus::Unchecked`] and skipped in all
    /// cross-model checks (Req 9.4). A per-model FFI error is recorded, the model
    /// is marked [`ModelStatus::Errored`], and the remaining models are still
    /// evaluated (Req 9.3, 9.6). Every model is classified as exactly one of
    /// collision-free / in-collision / unchecked / errored (Req 9.7). This
    /// method never panics (Req 9.1).
    pub fn check_configuration(&mut self, joints: &SceneConfig) -> CollisionReport {
        let margin = self.config.contact_margin_m;
        let report_all = self.config.report_all;
        let do_self = self.config.check_self;
        let do_robot_robot = self.config.check_robot_robot;
        let do_environment = self.config.check_environment && !self.environment.is_empty();

        let n = self.robots.len();
        let mut pairs: Vec<CollisionPair> = Vec::new();
        let mut min_eval: Option<f64> = None;

        // Per-robot bookkeeping.
        let mut robot_joints: Vec<Option<[f64; 6]>> = vec![None; n];
        let mut robot_error: Vec<Option<String>> = vec![None; n];
        let mut robot_world: Vec<Option<Vec<WorldCollisionObject>>> = vec![None; n];
        // The geometry index (registration order) of each `robot_world` entry.
        let mut robot_world_index: Vec<Vec<usize>> = vec![Vec::new(); n];

        // Resolve each robot's joints; a missing entry ⇒ unchecked (Req 9.4).
        for (i, robot) in self.robots.iter().enumerate() {
            robot_joints[i] = joints.get(&robot.robot_id).copied();
        }

        // Whether any cross-model class needs world placements at all.
        let need_placements = do_robot_robot || do_environment;

        // 1. Self-collision + world placements, per robot, in scene order.
        for i in 0..n {
            let Some(q) = robot_joints[i] else {
                continue; // unchecked robot: skip entirely.
            };

            // 1a. Self-collision (Req 3.5), skipped when disabled (Req 7.8).
            if do_self {
                let robot_id = self.robots[i].robot_id.clone();
                match self.robots[i].model.check_self(&q, !report_all) {
                    Ok(self_pairs) => {
                        for p in self_pairs {
                            fold_min(&mut min_eval, p.distance_m);
                            if margin_collides(p.distance_m, margin) {
                                pairs.push(p);
                            }
                        }
                    }
                    Err(e) => {
                        robot_error[i] = Some(format!(
                            "self-collision query failed for robot `{robot_id}`: {e}"
                        ));
                    }
                }
            }

            // 1b. World-frame placements for cross-model checks (Req 3.2),
            // with real per-link half-extents (Req 7.1-7.4) rather than
            // degenerate point AABBs.
            if need_placements {
                let base = self.robots[i].base_transform;
                let robot_id = self.robots[i].robot_id.clone();
                match self.robots[i].model.geometry_placements(&q) {
                    Ok(local) => {
                        // Per-link boxes (local AABB centre + half-extents) are a
                        // configuration-independent property of the loaded
                        // geometry, cached at build. Without the cache (its
                        // query failed) fall back to a fresh half-extent query
                        // with the box centred on the link frame, and to a
                        // uniform epsilon extent if that fails too (Req 7.4):
                        // never fail the whole check.
                        let fallback;
                        let boxes: &[(Vector3<f64>, Vector3<f64>)] = match &self.robots[i].local_boxes {
                            Some(boxes) => boxes,
                            None => {
                                let half_extents = self.robots[i]
                                    .model
                                    .geometry_half_extents()
                                    .unwrap_or_else(|e| {
                                        eprintln!(
                                            "WARN: geometry half-extent query failed for robot \
                                             `{robot_id}`: {e}; falling back to epsilon extents \
                                             for every link"
                                        );
                                        Vec::new()
                                    });
                                fallback = half_extents
                                    .into_iter()
                                    .map(|h| (Vector3::zeros(), h))
                                    .collect::<Vec<_>>();
                                &fallback
                            }
                        };
                        let epsilon = Vector3::new(MIN_HALF_EXTENT_M, MIN_HALF_EXTENT_M, MIN_HALF_EXTENT_M);

                        let world = compose_world_placements(&base, &local);
                        let mut indices = Vec::with_capacity(world.len());
                        let objs: Vec<WorldCollisionObject> = world
                            .iter()
                            .filter_map(|p| {
                                // An out-of-range index (fewer boxes than
                                // placements) falls back to the epsilon extent
                                // at the link frame rather than panicking or
                                // dropping the link (Req 7.4).
                                let (center, he) =
                                    boxes.get(p.index).copied().unwrap_or((Vector3::zeros(), epsilon));
                                let placed = GeometryPlacement {
                                    name: p.name.clone(),
                                    index: p.index,
                                    pose: p.pose * Translation3::from(center),
                                };
                                let obj =
                                    WorldCollisionObject::from_world_placement(&robot_id, &placed, he)?;
                                indices.push(p.index);
                                Some(obj)
                            })
                            .collect();
                        robot_world[i] = Some(objs);
                        robot_world_index[i] = indices;
                    }
                    Err(e) => {
                        // Preserve a self-collision error if one was already set.
                        if robot_error[i].is_none() {
                            robot_error[i] = Some(format!(
                                "geometry placement query failed for robot `{robot_id}`: {e}"
                            ));
                        }
                    }
                }
            }
        }

        // 2. Robot-robot, over unordered pairs of distinct robots (Req 3.3).
        if do_robot_robot {
            for i in 0..n {
                for j in (i + 1)..n {
                    let (Some(a_objs), Some(b_objs)) =
                        (robot_world[i].as_ref(), robot_world[j].as_ref())
                    else {
                        continue; // one side unchecked or errored: skip (Req 9.4).
                    };
                    let cross = narrowphase(
                        a_objs,
                        b_objs,
                        margin,
                        CollisionKind::RobotRobot,
                        |oa, ob| {
                            let d = aabb_signed_distance(&oa.aabb, &ob.aabb);
                            fold_min(&mut min_eval, d);
                            Ok(d)
                        },
                    );
                    // The proxy distance_fn is infallible, so `cross` is always
                    // `Ok`; on any future error we simply skip the pair rather
                    // than panic (Req 9.1).
                    if let Ok(cross_pairs) = cross {
                        for p in cross_pairs {
                            if margin_collides(p.distance_m, margin) {
                                pairs.push(p);
                            }
                        }
                    }
                }
            }
        }

        // 3. Robot-environment (Req 3.4, 6.1-6.4). The environment's world
        // objects are precomputed at build time (Req 5.1-5.3), so there is no
        // FFI call and no `.glb` parse here.
        //
        // Same broadphase gate + AABB-proxy distance as `narrowphase`, but each
        // environment object's role decides which robot geometry it is checked
        // against: a workpiece is not checked against the tool end, a support
        // (pedestal, floor) not against the robot's base-fixed geometry.
        if do_environment {
            let env_objs = self.environment.objects();
            let env_roles = self.environment.roles();
            for i in 0..n {
                let Some(a_objs) = robot_world[i].as_ref() else {
                    continue; // unchecked/errored robot: skip (Req 9.4).
                };
                let robot = &self.robots[i];
                for (k, oa) in a_objs.iter().enumerate() {
                    let geom = robot_world_index[i].get(k).copied();
                    let flag = |v: &Vec<bool>| geom.and_then(|g| v.get(g).copied()).unwrap_or(false);
                    let base_fixed = flag(&robot.base_fixed);
                    let tool_end = flag(&robot.tool_end);
                    for (ob, role) in env_objs.iter().zip(env_roles) {
                        let skip = match role {
                            EnvironmentRole::Obstacle => false,
                            EnvironmentRole::Workpiece => tool_end,
                            EnvironmentRole::Support(None) => base_fixed,
                            EnvironmentRole::Support(Some(owner)) => {
                                base_fixed && *owner == robot.robot_id
                            }
                        };
                        if skip || !aabb_overlap(&oa.aabb, &ob.aabb, margin) {
                            continue;
                        }
                        let d = aabb_signed_distance(&oa.aabb, &ob.aabb);
                        fold_min(&mut min_eval, d);
                        if d <= margin && margin_collides(d, margin) {
                            if let Some(p) = CollisionPair::new(
                                oa.name.clone(),
                                ob.name.clone(),
                                CollisionKind::RobotEnvironment,
                                d,
                            ) {
                                pairs.push(p);
                            }
                        }
                    }
                }
            }
        }

        // 4. Aggregate normalized/deduplicated pairs and the minimum distance
        //    (Req 4.4, 5.1, 5.4, 5.5, 8.2).
        let report = CollisionReport::from_pairs(pairs, min_eval);

        // 5. Classify every model (Req 9.7). A robot is in-collision when it
        //    appears (by its qualifying prefix) in any reported pair; an
        //    environment object is in-collision when its own fully-qualified
        //    name (`environment/<id>`) appears in a reported pair.
        let mut in_collision_ids: Vec<&str> = Vec::new();
        let mut in_collision_names: Vec<&str> = Vec::new();
        for p in &report.pairs {
            for name in [p.object_a.as_str(), p.object_b.as_str()] {
                let id = model_id_of(name);
                if !in_collision_ids.contains(&id) {
                    in_collision_ids.push(id);
                }
                if !in_collision_names.contains(&name) {
                    in_collision_names.push(name);
                }
            }
        }

        let mut model_status: Vec<ModelClassification> = Vec::with_capacity(n + 1);
        for i in 0..n {
            let robot_id = self.robots[i].robot_id.clone();
            let status = if robot_joints[i].is_none() {
                ModelStatus::Unchecked
            } else if robot_error[i].is_some() {
                ModelStatus::Errored
            } else if in_collision_ids.contains(&robot_id.as_str()) {
                ModelStatus::InCollision
            } else {
                ModelStatus::CollisionFree
            };
            model_status.push(ModelClassification {
                model_id: robot_id,
                status,
                error: robot_error[i].take(),
            });
        }

        // Classify every successfully-placed environment object individually
        // (Req 9.4, 9.6): each `environment/<id>` world object gets its own
        // `ModelClassification`, rather than one opaque entry for the whole
        // environment, so per-object status is visible on the report.
        for obj in self.environment.objects() {
            let status = if !self.config.check_environment {
                ModelStatus::Unchecked
            } else if in_collision_names.contains(&obj.name.as_str()) {
                ModelStatus::InCollision
            } else {
                ModelStatus::CollisionFree
            };
            model_status.push(ModelClassification {
                model_id: obj.name.clone(),
                status,
                error: None,
            });
        }

        // Fold every environment object that failed to load into
        // `model_status` (Req 9.4, 9.6): these entries already carry
        // `model_id: "environment/<id>"`, `status: Errored`, and the load
        // error message, so failed environment objects are surfaced
        // per-object rather than as a single opaque environment error.
        model_status.extend(self.environment.errored().iter().cloned());

        report.with_model_status(model_status)
    }

    /// Check a discrete scene trajectory at synchronized sampled waypoints and
    /// return a [`TrajectoryCollisionReport`] (design "Discrete trajectory
    /// check").
    ///
    /// Samples a synchronized progress `s = k / N ∈ [0.0, 1.0]` across all
    /// robots at every [`waypoint_stride`](CollisionConfig::waypoint_stride)-th
    /// index `k` (`k = 0, stride, 2·stride, …, ≤ N`), where `N` is the
    /// trajectory's `sample_count`. At each `s` it builds a [`SceneConfig`] from
    /// every robot's [`JointPath::sample`] and delegates to
    /// [`check_configuration`](Self::check_configuration) (Req 6.1, 6.2).
    ///
    /// * The colliding waypoint with the smallest progress is reported as
    ///   `first` (Req 6.3). Because indices are visited in ascending order, the
    ///   first collision encountered is the earliest.
    /// * When [`report_all`](CollisionConfig::report_all) is enabled, every
    ///   colliding waypoint is collected in ascending progress order (Req 6.4);
    ///   otherwise checking stops at the first collision (Req 6.5).
    /// * When no sampled waypoint collides, the trajectory is reported as not
    ///   colliding (Req 6.6).
    ///
    /// # Robustness
    ///
    /// * A robot whose path is [`JointPath::None`] (or absent) is recorded in
    ///   `unchecked_robots` and skipped in every configuration check, while the
    ///   remaining robots are still checked (Req 6.7).
    /// * When every robot is unchecked, a clear report is returned recording
    ///   that no checkable robots remain (Req 6.9).
    /// * When the `sample_count` is `< 1` or `> total_knots`, the request is
    ///   rejected with no collision checking performed (Req 6.8).
    /// * The input trajectory is never mutated (Req 9.2), and this method never
    ///   panics (`sample_count >= 1` guards the `k / N` division).
    pub fn check_trajectory(&mut self, trajectory: &SceneTrajectory) -> TrajectoryCollisionReport {
        let stride = self.config.waypoint_stride.max(1);
        let report_all = self.config.report_all;
        let n = trajectory.sample_count;
        let total = trajectory.total_knots;

        // Req 6.8: an out-of-range sample count is rejected with NO checking.
        if n < 1 || n > total {
            return TrajectoryCollisionReport::rejected(format!(
                "invalid sample count {n}: must be within [1, {total}] (total knot count)"
            ));
        }

        // Req 6.7: classify each scene robot as checkable (a path that carries
        // joint info) or unchecked (`None` variant or absent from the map).
        let mut unchecked_robots: Vec<String> = Vec::new();
        let mut any_checkable = false;
        for robot in &self.robots {
            match trajectory.paths.get(&robot.robot_id) {
                Some(path) if path.is_some() => any_checkable = true,
                _ => unchecked_robots.push(robot.robot_id.clone()),
            }
        }

        // Req 6.9: no checkable robots ⇒ report no collision.
        if !any_checkable {
            return TrajectoryCollisionReport::no_checkable_robots(unchecked_robots);
        }

        let mut first: Option<TrajectoryCollisionWaypoint> = None;
        let mut all: Vec<TrajectoryCollisionWaypoint> = Vec::new();

        // FOR k FROM 0 TO N STEP stride DO; s <- k / N (design pseudocode).
        let mut k = 0usize;
        while k <= n {
            let s = k as f64 / n as f64;

            // Build the synchronized configuration at progress s. Robots whose
            // path samples to `None` (`None` variant, or an empty `Dense`) are
            // simply omitted, so `check_configuration` marks them unchecked and
            // excludes them from cross-model checks (Req 6.7).
            let mut scene_config = SceneConfig::new();
            for robot in &self.robots {
                if let Some(path) = trajectory.paths.get(&robot.robot_id)
                    && let Some(q) = path.sample(s)
                {
                    scene_config.insert(robot.robot_id.clone(), q);
                }
            }

            let report = self.check_configuration(&scene_config);
            if report.in_collision {
                let hit = TrajectoryCollisionWaypoint {
                    progress: s,
                    waypoint: k,
                    report,
                };
                if first.is_none() {
                    first = Some(hit.clone());
                }
                all.push(hit);
                // Req 6.5: stop at the first collision unless report-all is set.
                if !report_all {
                    break;
                }
            }

            // Advance by the stride; guard against usize overflow so the loop
            // always terminates (never panics).
            match k.checked_add(stride) {
                Some(next) => k = next,
                None => break,
            }
        }

        TrajectoryCollisionReport {
            collided: first.is_some(),
            first,
            all,
            rejected: None,
            unchecked_robots,
        }
    }
}

/// Fold a separation `d` into the running minimum, seeding it on the first call
/// (Req 5.4/5.5).
fn fold_min(min: &mut Option<f64>, d: f64) {
    *min = Some(match *min {
        Some(m) => m.min(d),
        None => d,
    });
}

/// Extract the `<robot_id>` (or [`ENVIRONMENT_ID`]) prefix from a fully-qualified
/// `<id>/<link_or_geom>` object name, used to attribute reported pairs to their
/// owning model during classification (Req 9.7).
fn model_id_of(name: &str) -> &str {
    name.split_once('/').map_or(name, |(id, _)| id)
}

/// `.glb` (glTF-binary) visual mesh to collision-proxy loading (task 2.1).
///
/// Turns a `.glb` visual mesh into a lightweight [`CollisionProxy`] the
/// environment can place and check against, without requiring a URDF
/// `<collision>` geometry or a Pinocchio/Coal model (see design "Geometry
/// Approach"). Both the `BoundingBox` primary tier and the optional
/// `ConvexHull` tier (with degrade-to-box for degenerate input) are
/// implemented here.
#[cfg(feature = "collision")]
pub mod glb {
    use std::collections::{BTreeSet, HashMap, HashSet};
    use std::panic::AssertUnwindSafe;
    use std::path::Path;

    use nalgebra::{Matrix4, Vector3, Vector4};

    /// Hard cap (bytes) on the total size of any `.glb` buffer (JSON chunk +
    /// binary chunk) this loader will read. Guards against unbounded
    /// allocation from a malicious or corrupt file (Req 3.9).
    pub const MAX_GLB_BUFFER_BYTES: u64 = 256 * 1024 * 1024;

    /// Hard cap on the number of vertices accumulated across every mesh
    /// primitive in a single `.glb` file. Guards against unbounded allocation
    /// from a mesh with an implausibly large vertex count (Req 3.9).
    pub const MAX_GLB_VERTEX_COUNT: usize = 4_000_000;

    /// A collision proxy derived from a visual mesh: either a local-frame
    /// axis-aligned box (the recommended primary tier) or an optional
    /// local-frame convex hull (higher-fidelity, feature-tier upgrade).
    #[derive(Debug, Clone, PartialEq)]
    pub enum CollisionProxy {
        /// Local-frame box: center offset from the mesh origin plus
        /// non-negative half-extents on every axis.
        Box {
            center: Vector3<f64>,
            half_extents: Vector3<f64>,
        },
        /// Local-frame convex hull vertices (optional, higher-fidelity
        /// tier). Always at least 4 non-coplanar vertices; degenerate input
        /// degrades to `Box` instead (see [`load_collision_proxy`]).
        ConvexHull { vertices: Vec<Vector3<f64>> },
    }

    /// Selects which proxy is computed from the mesh vertices.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub enum ProxyMode {
        /// Axis-aligned box enclosing all vertices (recommended default).
        #[default]
        BoundingBox,
        /// Convex hull of the vertices (higher fidelity). Degrades to
        /// `BoundingBox` when the cloud has fewer than four vertices or is
        /// coplanar (zero-volume hull).
        ConvexHull,
    }

    /// Load a `.glb` file and compute its collision proxy.
    ///
    /// Walks the node hierarchy depth-first, accumulating each node's local
    /// transform, and gathers every mesh primitive's `POSITION` vertices into
    /// a vertex cloud with that transform applied. For [`ProxyMode::BoundingBox`]
    /// the cloud is reduced to a componentwise `min`/`max` box.
    ///
    /// Returns `Err` (never panics) naming `path` when: the file is missing or
    /// unreadable; it is not valid glTF-binary or uses an unreadable position
    /// accessor; it contains zero mesh vertices (never returns a zero-volume
    /// proxy for an empty mesh); or a configured hard cap on buffer size or
    /// vertex count is exceeded (Req 3.1, 3.2, 3.5-3.9, 13.1).
    pub fn load_collision_proxy(path: &Path, mode: ProxyMode) -> Result<CollisionProxy, String> {
        let vertices = load_vertex_cloud(path)?;

        match mode {
            ProxyMode::BoundingBox => Ok(bounding_box_proxy(&vertices)),
            ProxyMode::ConvexHull => match convex_hull(&vertices) {
                // The hull is well-formed (>= 4 non-coplanar vertices): its
                // vertices are a subset of `vertices` and the hull encloses
                // every one of them (Req 3.3).
                Some(hull_vertices) => Ok(CollisionProxy::ConvexHull {
                    vertices: hull_vertices,
                }),
                // Fewer than four vertices, or the cloud is coplanar (the
                // hull construction degenerates to zero volume): degrade to
                // the same enclosing `Box` proxy `BoundingBox` mode would
                // produce (Req 3.4).
                None => Ok(bounding_box_proxy(&vertices)),
            },
        }
    }

    /// Read a `.glb` file's size, parse it, and accumulate every mesh
    /// primitive's `POSITION` vertices (with node transforms applied) into a
    /// single local-frame vertex cloud. Never panics: the third-party parser
    /// call is guarded with [`std::panic::catch_unwind`] so malformed input
    /// cannot escalate into a panic (Req 3.8).
    fn load_vertex_cloud(path: &Path) -> Result<Vec<Vector3<f64>>, String> {
        let metadata = std::fs::metadata(path)
            .map_err(|e| format!("glb: cannot read '{}': {e}", path.display()))?;
        if metadata.len() > MAX_GLB_BUFFER_BYTES {
            return Err(format!(
                "glb: '{}' exceeds the maximum buffer size ({} bytes > {} byte cap)",
                path.display(),
                metadata.len(),
                MAX_GLB_BUFFER_BYTES
            ));
        }

        // `gltf::import` (and the parser it drives) is third-party code that
        // may panic on sufficiently malformed input; guard the call so this
        // loader upholds its own never-panic contract regardless (Req 3.8).
        let owned_path = path.to_path_buf();
        let import_result = std::panic::catch_unwind(AssertUnwindSafe(|| {
            gltf::import(&owned_path)
        }))
        .map_err(|_| {
            format!(
                "glb: parsing '{}' panicked in the glTF importer (treated as a parse failure)",
                path.display()
            )
        })?;

        let (document, buffers, _images) = import_result.map_err(|e| {
            format!(
                "glb: '{}' is not valid glTF-binary or uses an unreadable accessor: {e}",
                path.display()
            )
        })?;

        let buffer_data: Vec<&[u8]> = buffers.iter().map(|b| b.0.as_slice()).collect();
        let get_buffer_data = |buffer: gltf::Buffer<'_>| buffer_data.get(buffer.index()).copied();

        let mut vertices = Vec::new();
        let scene = document
            .default_scene()
            .or_else(|| document.scenes().next());

        // Depth-first node walk using an explicit stack (rather than a
        // recursive helper) so the borrow of each `Primitive` handed to
        // `.reader(...)` stays local to this function body and is not forced
        // to outlive an externally-named generic lifetime parameter.
        if let Some(scene) = scene {
            let mut stack: Vec<(gltf::Node<'_>, Matrix4<f64>)> = scene
                .nodes()
                .map(|node| (node, Matrix4::identity()))
                .collect();

            while let Some((node, parent_transform)) = stack.pop() {
                let world = parent_transform * node_local_transform(&node);

                if let Some(mesh) = node.mesh() {
                    for primitive in mesh.primitives() {
                        let reader = primitive.reader(get_buffer_data);
                        if let Some(iter) = reader.read_positions() {
                            for p in iter {
                                if vertices.len() >= MAX_GLB_VERTEX_COUNT {
                                    return Err(format!(
                                        "glb: '{}' exceeds the maximum vertex count ({} cap)",
                                        path.display(),
                                        MAX_GLB_VERTEX_COUNT
                                    ));
                                }
                                let local_v =
                                    Vector4::new(p[0] as f64, p[1] as f64, p[2] as f64, 1.0);
                                let transformed = world * local_v;
                                vertices.push(Vector3::new(
                                    transformed.x,
                                    transformed.y,
                                    transformed.z,
                                ));
                            }
                        }
                    }
                }

                for child in node.children() {
                    stack.push((child, world));
                }
            }
        }

        if vertices.is_empty() {
            return Err(format!(
                "glb: '{}' is valid glTF-binary but contains no mesh vertices",
                path.display()
            ));
        }

        Ok(vertices)
    }

    /// Convert a node's TRS/matrix transform (row-major `[f32; 4]` rows from
    /// `gltf`) into a column-major `nalgebra::Matrix4<f64>` suitable for
    /// composing with parent transforms and applying to homogeneous vertices.
    fn node_local_transform(node: &gltf::Node<'_>) -> Matrix4<f64> {
        let m = node.transform().matrix();
        // `gltf::scene::Transform::matrix()` returns `[[f32; 4]; 4]` where the
        // outer index is the column (glTF matrices are column-major), matching
        // `Matrix4::from_fn`'s `(row, col)` -> `m[col][row]` mapping below.
        Matrix4::from_fn(|row, col| m[col][row] as f64)
    }

    /// Reduce a non-empty vertex cloud to a componentwise `min`/`max` box
    /// proxy. Every half-extent is `>= 0` and the box encloses every vertex
    /// (Req 3.1, 3.2, 13.1).
    fn bounding_box_proxy(vertices: &[Vector3<f64>]) -> CollisionProxy {
        let mut lo = vertices[0];
        let mut hi = vertices[0];
        for v in &vertices[1..] {
            lo.x = lo.x.min(v.x);
            lo.y = lo.y.min(v.y);
            lo.z = lo.z.min(v.z);
            hi.x = hi.x.max(v.x);
            hi.y = hi.y.max(v.y);
            hi.z = hi.z.max(v.z);
        }
        let center = (lo + hi) * 0.5;
        let half_extents = (hi - lo) * 0.5;
        CollisionProxy::Box {
            center,
            half_extents,
        }
    }

    /// A triangular face of the incremental hull under construction:
    /// `v` are indices into the original `points` slice (winding is
    /// consistently outward-facing), and `normal` is that face's outward
    /// unit normal (or the zero vector for a residual degenerate face, which
    /// the visibility test below then treats as never-visible).
    #[derive(Clone, Debug)]
    struct HullFace {
        v: [usize; 3],
        normal: Vector3<f64>,
    }

    /// Outward-facing unit normal of the triangle `points[v[0..3]]`, or the
    /// zero vector when the triangle is degenerate (collinear vertices).
    /// Never panics or produces `NaN`: a zero-length cross product is left as
    /// the zero vector rather than normalized.
    fn hull_face_normal(points: &[Vector3<f64>], v: [usize; 3]) -> Vector3<f64> {
        let a = points[v[0]];
        let b = points[v[1]];
        let c = points[v[2]];
        let n = (b - a).cross(&(c - a));
        let len = n.norm();
        if len > 1e-15 { n / len } else { Vector3::zeros() }
    }

    /// Compute the 3-D convex hull vertex set of `points` with a
    /// straightforward incremental algorithm (build an initial tetrahedron,
    /// then repeatedly fold in each remaining point that lies outside the
    /// current hull, replacing the faces it can see with new faces to the
    /// horizon).
    ///
    /// Returns `None` -- signalling "degrade to a bounding box" -- when there
    /// are fewer than four points, or the cloud is coplanar/collinear (no
    /// four points found that are not all coincident with a common plane),
    /// per Req 3.4. On success, every returned vertex is one of the input
    /// `points` (a subset of the accumulated cloud, Req 3.3) and the hull
    /// they form encloses every input point, because the algorithm only ever
    /// folds a point in when it lies strictly outside the hull built from the
    /// points already processed, leaving no point outside the final hull.
    /// Never panics: all indexing is guarded by the loops that produce the
    /// indices, and degenerate/zero-length normals are handled explicitly
    /// above rather than dividing by zero.
    fn convex_hull(points: &[Vector3<f64>]) -> Option<Vec<Vector3<f64>>> {
        let n = points.len();
        if n < 4 {
            return None;
        }

        // Scale-aware tolerance derived from the cloud's own extent, so the
        // degeneracy checks below are meaningful for both tiny and huge
        // meshes rather than relying on a fixed absolute epsilon.
        let mut lo = points[0];
        let mut hi = points[0];
        for p in &points[1..] {
            lo.x = lo.x.min(p.x);
            lo.y = lo.y.min(p.y);
            lo.z = lo.z.min(p.z);
            hi.x = hi.x.max(p.x);
            hi.y = hi.y.max(p.y);
            hi.z = hi.z.max(p.z);
        }
        let extent = (hi - lo).norm().max(1.0);
        let eps = extent * 1e-9;

        // --- Seed an initial non-degenerate tetrahedron. ---
        // 1. Anchor a line between the two most-separated of the cloud's
        //    componentwise min/max extreme points (at most 6 candidates),
        //    which avoids an O(n^2) all-pairs scan for the common case.
        let mut extreme_idx: Vec<usize> = Vec::with_capacity(6);
        for axis in 0..3 {
            let (mut min_i, mut max_i) = (0usize, 0usize);
            for (i, p) in points.iter().enumerate() {
                if p[axis] < points[min_i][axis] {
                    min_i = i;
                }
                if p[axis] > points[max_i][axis] {
                    max_i = i;
                }
            }
            extreme_idx.push(min_i);
            extreme_idx.push(max_i);
        }

        let (mut ia, mut ib) = (extreme_idx[0], extreme_idx[1]);
        let mut best = 0.0f64;
        for &i in &extreme_idx {
            for &j in &extreme_idx {
                let d = (points[i] - points[j]).norm_squared();
                if d > best {
                    best = d;
                    ia = i;
                    ib = j;
                }
            }
        }
        if best <= eps * eps {
            // The extreme points all coincide: a degenerate (point-like)
            // cloud, even though it has >= 4 entries.
            return None;
        }

        // 2. Find the point farthest from line `ia`-`ib` to form a triangle.
        let dir = (points[ib] - points[ia]).normalize();
        let mut ic: Option<usize> = None;
        let mut best_dist = eps;
        for (k, p) in points.iter().enumerate() {
            if k == ia || k == ib {
                continue;
            }
            let v = p - points[ia];
            let perp = v - dir * v.dot(&dir);
            let d = perp.norm();
            if d > best_dist {
                best_dist = d;
                ic = Some(k);
            }
        }
        let ic = match ic {
            Some(ic) => ic,
            // Every point is collinear with `ia`-`ib`: a degenerate cloud.
            None => return None,
        };

        // 3. Find the point farthest from plane `ia`-`ib`-`ic` to complete a
        //    tetrahedron.
        let normal = (points[ib] - points[ia]).cross(&(points[ic] - points[ia]));
        let normal_len = normal.norm();
        if normal_len <= eps {
            return None;
        }
        let normal_unit = normal / normal_len;
        let plane_d = normal_unit.dot(&points[ia]);
        let mut id: Option<usize> = None;
        let mut best_plane_dist = eps;
        for (k, p) in points.iter().enumerate() {
            if k == ia || k == ib || k == ic {
                continue;
            }
            let d = (normal_unit.dot(p) - plane_d).abs();
            if d > best_plane_dist {
                best_plane_dist = d;
                id = Some(k);
            }
        }
        let id = match id {
            Some(id) => id,
            // Every remaining point lies on the `ia`-`ib`-`ic` plane: the
            // cloud is coplanar (zero volume) -- degrade to a box (Req 3.4).
            None => return None,
        };

        // Build the tetrahedron's 4 outward-facing triangular faces, each
        // omitting one of the 4 seed vertices; orient each so its normal
        // points away from the omitted (interior) vertex.
        let tetra = [ia, ib, ic, id];
        let mut faces: Vec<HullFace> = Vec::with_capacity(4);
        for skip in 0..4 {
            let tri: Vec<usize> = (0..4).filter(|&k| k != skip).map(|k| tetra[k]).collect();
            let mut v = [tri[0], tri[1], tri[2]];
            let mut normal = hull_face_normal(points, v);
            let inside_point = points[tetra[skip]];
            if normal.dot(&(inside_point - points[v[0]])) > 0.0 {
                v.swap(1, 2);
                normal = hull_face_normal(points, v);
            }
            faces.push(HullFace { v, normal });
        }

        // --- Fold in every remaining point. ---
        for p_idx in 0..n {
            if p_idx == ia || p_idx == ib || p_idx == ic || p_idx == id {
                continue;
            }
            let p = points[p_idx];

            let visible: HashSet<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, f)| f.normal.dot(&(p - points[f.v[0]])) > eps)
                .map(|(i, _)| i)
                .collect();
            if visible.is_empty() {
                // `p` lies within the hull built so far; nothing to do.
                continue;
            }

            // Map each directed edge of every current face to its owning
            // face, so the horizon (the boundary between visible and
            // non-visible faces) can be found without an O(faces^2) scan.
            let mut edge_owner: HashMap<(usize, usize), usize> = HashMap::new();
            for (fi, f) in faces.iter().enumerate() {
                let v = f.v;
                edge_owner.insert((v[0], v[1]), fi);
                edge_owner.insert((v[1], v[2]), fi);
                edge_owner.insert((v[2], v[0]), fi);
            }

            let mut horizon: Vec<(usize, usize)> = Vec::new();
            for &fi in &visible {
                let v = faces[fi].v;
                for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
                    let twin_owner = edge_owner.get(&(b, a)).copied();
                    let twin_is_visible = twin_owner.is_some_and(|o| visible.contains(&o));
                    if !twin_is_visible {
                        horizon.push((a, b));
                    }
                }
            }

            let mut remaining: Vec<HullFace> = faces
                .into_iter()
                .enumerate()
                .filter(|(i, _)| !visible.contains(i))
                .map(|(_, f)| f)
                .collect();
            for (a, b) in horizon {
                let v = [a, b, p_idx];
                let normal = hull_face_normal(points, v);
                remaining.push(HullFace { v, normal });
            }
            faces = remaining;
        }

        // The final hull's vertex set is exactly the (index-)distinct
        // vertices still referenced by a face; `BTreeSet` gives a
        // deterministic, sorted-by-index iteration order.
        let hull_idx: BTreeSet<usize> = faces
            .iter()
            .flat_map(|f| f.v.into_iter())
            .collect();
        if hull_idx.len() < 4 {
            // Should not happen given the checks above, but never silently
            // return a degenerate hull -- degrade to a box instead.
            return None;
        }

        Some(hull_idx.into_iter().map(|i| points[i]).collect())
    }
}

/// Build-time specification for one placed static environment object
/// (design Component 4, `Environment_Spec`).
///
/// Describes a single `static_models:` scene entry: a non-empty `id`, the
/// `.glb` visual mesh `file` to load, the world-frame `transform` at which
/// to place it, and the [`glb::ProxyMode`] used to compute its collision
/// proxy. Consumed by `EnvironmentCollision::build` (task 3.2).
#[cfg(feature = "collision")]
#[derive(Debug, Clone)]
pub struct EnvironmentCollisionSpec {
    /// Stable identifier used to qualify the object name as `environment/<id>`.
    pub id: String,
    /// Path to the `.glb` visual mesh to load into a collision proxy.
    pub file: PathBuf,
    /// Rigid world-frame transform (`world <- object`) from `scene.yaml`.
    pub transform: Isometry3<f64>,
    /// Which collision proxy tier to compute from the mesh vertices.
    pub proxy_mode: glb::ProxyMode,
}

#[cfg(feature = "collision")]
impl EnvironmentCollisionSpec {
    /// Construct a new specification, defaulting `proxy_mode` to
    /// [`glb::ProxyMode::BoundingBox`] (Req 4.5, 5.2).
    pub fn new(id: impl Into<String>, file: impl Into<PathBuf>, transform: Isometry3<f64>) -> Self {
        Self {
            id: id.into(),
            file: file.into(),
            transform,
            proxy_mode: glb::ProxyMode::default(),
        }
    }

    /// Validate that the specification is well-formed: a non-empty `id`
    /// (after trimming) and a non-empty `file` path.
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("EnvironmentCollisionSpec.id must not be empty".to_string());
        }
        if self.file.as_os_str().is_empty() {
            return Err(format!(
                "EnvironmentCollisionSpec.file for environment object `{}` must not be empty",
                self.id
            ));
        }
        Ok(())
    }
}

/// What a placed environment object is to the robots: it decides which robot
/// geometry the object is checked against.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum EnvironmentRole {
    /// Checked against every robot geometry.
    #[default]
    Obstacle,
    /// The surface being processed (`obstacles.yaml` `is_workpiece: true`):
    /// the tool end (the geometry the last joint carries: flange and tool) may
    /// touch it; the arm links are still checked against it.
    Workpiece,
    /// What a robot stands on: its pedestal (`Some(robot_id)`) or the floor
    /// (`None`: every robot). The robot's base-fixed geometry, which no joint
    /// moves and so no plan can put into collision, is not checked against it;
    /// everything that moves is.
    Support(Option<String>),
}

/// One oriented box of the cell's environment (a fixture box, a pedestal, the
/// floor), placed in the world frame. Boxes need no mesh loading and are
/// checked with the same AABB broadphase/proxy distance as the rest.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvironmentBoxSpec {
    /// Identifier, qualified as `environment/<id>` in reports.
    pub id: String,
    /// World pose of the box centre (`world <- box`).
    pub pose: Isometry3<f64>,
    /// Non-negative half-extents along the box's own axes (metres).
    pub half_extents: Vector3<f64>,
    /// Which robot geometry the box is checked against.
    pub role: EnvironmentRole,
}

impl EnvironmentBoxSpec {
    /// An axis-aligned box from its world `center` and `half_extents`.
    pub fn aligned(
        id: impl Into<String>,
        center: Vector3<f64>,
        half_extents: Vector3<f64>,
        role: EnvironmentRole,
    ) -> Self {
        Self {
            id: id.into(),
            pose: Isometry3::from_parts(Translation3::from(center), UnitQuaternion::identity()),
            half_extents,
            role,
        }
    }

    fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("EnvironmentBoxSpec.id must not be empty".to_string());
        }
        let t = self.pose.translation.vector;
        let q = self.pose.rotation.coords;
        let finite = t.iter().chain(q.iter()).all(|v| v.is_finite());
        if !finite || !self.half_extents.iter().all(|h| h.is_finite() && *h >= 0.0) {
            return Err(format!(
                "environment box `{}` needs a finite pose and finite, non-negative half-extents",
                self.id
            ));
        }
        Ok(())
    }
}

/// The placed static environment: every [`EnvironmentCollisionSpec`] that
/// loaded successfully, composed into world-frame [`WorldCollisionObject`]s,
/// plus a per-failure classification for every spec that did not load
/// (design Component 4, task 3.2).
///
/// [`EnvironmentCollision::build`] is a total function: it never returns
/// `Err` and never panics, even given an empty or entirely-invalid slice of
/// specs. Every input spec ends up in exactly one of `objects` (placed) or
/// `errored` (one [`ModelClassification`] per failed spec), so
/// `objects().len() + errored.len() == specs.len()` always holds.
#[cfg(feature = "collision")]
#[derive(Debug, Clone, PartialEq)]
pub struct EnvironmentCollision {
    objects: Vec<WorldCollisionObject>,
    /// The role of each entry of `objects` (same order).
    roles: Vec<EnvironmentRole>,
    errored: Vec<ModelClassification>,
}

#[cfg(feature = "collision")]
impl EnvironmentCollision {
    /// Build the placed environment from a slice of specs.
    ///
    /// For each spec: validate it, then load its `.glb` collision proxy, then
    /// compose the world-frame placement. Any failure at either step records
    /// one [`ModelStatus::Errored`] classification named `environment/<id>`
    /// (Req 4.6-4.8, 9.1) and moves on to the next spec; this method never
    /// fails hard and never panics (Req 4.8, 9.1).
    pub fn build(specs: &[EnvironmentCollisionSpec]) -> Self {
        let mut objects = Vec::with_capacity(specs.len());
        let mut errored = Vec::new();

        for spec in specs {
            if let Err(e) = spec.validate() {
                errored.push(ModelClassification {
                    model_id: format!("{ENVIRONMENT_ID}/{}", spec.id),
                    status: ModelStatus::Errored,
                    error: Some(e),
                });
                continue;
            }

            let proxy = match glb::load_collision_proxy(&spec.file, spec.proxy_mode) {
                Ok(proxy) => proxy,
                Err(e) => {
                    errored.push(ModelClassification {
                        model_id: format!("{ENVIRONMENT_ID}/{}", spec.id),
                        status: ModelStatus::Errored,
                        error: Some(e),
                    });
                    continue;
                }
            };

            // Bound a `ConvexHull` by its own AABB in this primary tier
            // (design "Geometry Approach"): both cases reduce to a local
            // `center` + `half_extents` box before placement.
            let (center, half_extents) = match proxy {
                glb::CollisionProxy::Box {
                    center,
                    half_extents,
                } => (center, half_extents),
                glb::CollisionProxy::ConvexHull { vertices } => aabb_of_vertices(&vertices),
            };

            // Req 4.1: qualify as `environment/<id>` via the same helper the
            // robot side uses, so naming stays consistent scene-wide.
            let Some(name) = CollisionPair::qualify_name(ENVIRONMENT_ID, &spec.id) else {
                errored.push(ModelClassification {
                    model_id: format!("{ENVIRONMENT_ID}/{}", spec.id),
                    status: ModelStatus::Errored,
                    error: Some(format!(
                        "environment object `{}` could not be qualified into a fully-qualified name",
                        spec.id
                    )),
                });
                continue;
            };

            let local_pose =
                Isometry3::from_parts(Translation3::from(center), UnitQuaternion::identity());
            let world_pose = spec.transform * local_pose;

            objects.push(WorldCollisionObject::from_pose_half_extents(
                name,
                world_pose,
                half_extents,
            ));
        }

        let roles = vec![EnvironmentRole::Obstacle; objects.len()];
        Self { objects, roles, errored }
    }

    /// Place `boxes` too. A box that is malformed or whose id is already taken
    /// is recorded as errored (`environment/<id>`); this never fails.
    pub fn add_boxes(&mut self, boxes: &[EnvironmentBoxSpec]) {
        for spec in boxes {
            let model_id = format!("{ENVIRONMENT_ID}/{}", spec.id);
            let name = spec
                .validate()
                .and_then(|()| {
                    CollisionPair::qualify_name(ENVIRONMENT_ID, &spec.id).ok_or_else(|| {
                        format!("environment box `{}` could not be qualified into a name", spec.id)
                    })
                })
                .and_then(|name| {
                    if self.objects.iter().any(|o| o.name == name) {
                        Err(format!("environment object id `{}` is used twice", spec.id))
                    } else {
                        Ok(name)
                    }
                });
            match name {
                Ok(name) => {
                    self.objects.push(WorldCollisionObject::from_pose_half_extents(
                        name,
                        spec.pose,
                        spec.half_extents,
                    ));
                    self.roles.push(spec.role.clone());
                }
                Err(e) => self.errored.push(ModelClassification {
                    model_id,
                    status: ModelStatus::Errored,
                    error: Some(e),
                }),
            }
        }
    }

    /// Every successfully placed world-frame environment object.
    pub fn objects(&self) -> &[WorldCollisionObject] {
        &self.objects
    }

    /// The role of each object of [`objects`](Self::objects), in the same order.
    pub fn roles(&self) -> &[EnvironmentRole] {
        &self.roles
    }

    /// Whether no environment object was successfully placed (an empty spec
    /// slice, or every spec failed to validate/load).
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    /// One [`ModelClassification`] per spec that failed to validate or load,
    /// each already carrying `model_id: "environment/<id>"`,
    /// `status: Errored`, and a descriptive error message (Req 4.6, 9.1).
    /// Consumed by [`SceneCollisionChecker::check_configuration`] to surface
    /// failed environment objects per-object rather than as a single opaque
    /// error (task 5.2).
    pub fn errored(&self) -> &[ModelClassification] {
        &self.errored
    }
}

/// Reduce a non-empty vertex cloud (e.g. [`glb::CollisionProxy::ConvexHull`]
/// vertices) to a local-frame `center` + non-negative `half_extents` box via
/// componentwise `min`/`max` (bounding the hull by its AABB in the primary
/// tier). Returns a zero-sized box centred at the origin for an empty cloud,
/// which should not occur in practice but keeps this helper total.
#[cfg(feature = "collision")]
fn aabb_of_vertices(vertices: &[Vector3<f64>]) -> (Vector3<f64>, Vector3<f64>) {
    let Some(&first) = vertices.first() else {
        return (Vector3::zeros(), Vector3::zeros());
    };
    let mut lo = first;
    let mut hi = first;
    for v in &vertices[1..] {
        lo.x = lo.x.min(v.x);
        lo.y = lo.y.min(v.y);
        lo.z = lo.z.min(v.z);
        hi.x = hi.x.max(v.x);
        hi.y = hi.y.max(v.y);
        hi.z = hi.z.max(v.z);
    }
    let center = (lo + hi) * 0.5;
    let half_extents = (hi - lo) * 0.5;
    (center, half_extents)
}

/* ========================================================================= */
/* Unit tests (task 3.2)                                                     */
/*                                                                           */
/* Pure-Rust example/edge-case coverage for pair normalization and config   */
/* validation. Named `unit_tests` (rather than `tests`) so the property-test */
/* module from task 3.3 can coexist in this same file. The whole module is   */
/* already behind `#[cfg(feature = "collision")]` (see `main.rs`), so these  */
/* tests only compile/run under `--features collision`.                      */
/* ========================================================================= */

#[cfg(test)]
mod unit_tests {
    use super::*;

    /* --- Pair name ordering: object_a <= object_b by code point (Req 4.2) -- */

    #[test]
    fn new_orders_names_by_code_point_when_a_greater_than_b() {
        // Inputs given in reverse (larger first) must be swapped so that
        // object_a <= object_b by Unicode code point (Rust str == UTF-8 byte
        // order == code point order).
        let pair = CollisionPair::new(
            "r1/link_b".to_string(),
            "r1/link_a".to_string(),
            CollisionKind::SelfCollision,
            0.01,
        )
        .expect("distinct non-empty names form a pair");

        assert_eq!(pair.object_a, "r1/link_a");
        assert_eq!(pair.object_b, "r1/link_b");
        assert!(pair.object_a <= pair.object_b);
    }

    #[test]
    fn new_preserves_order_when_a_already_less_than_b() {
        let pair = CollisionPair::new(
            "r1/link_a".to_string(),
            "r1/link_b".to_string(),
            CollisionKind::RobotRobot,
            -0.2,
        )
        .expect("distinct non-empty names form a pair");

        assert_eq!(pair.object_a, "r1/link_a");
        assert_eq!(pair.object_b, "r1/link_b");
        assert!(pair.object_a <= pair.object_b);
    }

    #[test]
    fn new_orders_by_code_point_for_non_ascii_names() {
        // 'Z' (U+005A) < 'a' (U+0061) by code point, matching ASCII/byte order.
        let pair = CollisionPair::new(
            "robot/aaa".to_string(),
            "robot/ZZZ".to_string(),
            CollisionKind::RobotEnvironment,
            0.0,
        )
        .expect("distinct names form a pair");

        assert_eq!(pair.object_a, "robot/ZZZ");
        assert_eq!(pair.object_b, "robot/aaa");
    }

    /* --- Name qualification: <robot_id>/<link_or_geom> (Req 4.1) ----------- */

    #[test]
    fn qualify_name_joins_with_single_slash() {
        assert_eq!(
            CollisionPair::qualify_name("r1", "link_a").as_deref(),
            Some("r1/link_a")
        );
    }

    #[test]
    fn qualify_name_rejects_empty_components() {
        assert_eq!(CollisionPair::qualify_name("", "link_a"), None);
        assert_eq!(CollisionPair::qualify_name("r1", ""), None);
        assert_eq!(CollisionPair::qualify_name("", ""), None);
    }

    #[test]
    fn qualify_builds_normalized_ordered_pair() {
        let pair = CollisionPair::qualify(
            "r1",
            "link_b",
            "r1",
            "link_a",
            CollisionKind::SelfCollision,
            0.5,
        )
        .expect("distinct qualified names form a pair");

        assert_eq!(pair.object_a, "r1/link_a");
        assert_eq!(pair.object_b, "r1/link_b");
    }

    /* --- Identical-name exclusion (Req 4.3) -------------------------------- */

    #[test]
    fn new_excludes_identical_names() {
        let result = CollisionPair::new(
            "r1/link_a".to_string(),
            "r1/link_a".to_string(),
            CollisionKind::SelfCollision,
            0.0,
        );
        assert_eq!(result, None);
    }

    #[test]
    fn qualify_excludes_identical_qualified_names() {
        let result = CollisionPair::qualify(
            "r1",
            "link_a",
            "r1",
            "link_a",
            CollisionKind::SelfCollision,
            0.0,
        );
        assert_eq!(result, None);
    }

    #[test]
    fn new_rejects_empty_names() {
        assert_eq!(
            CollisionPair::new(
                String::new(),
                "r1/link_a".to_string(),
                CollisionKind::SelfCollision,
                0.0,
            ),
            None
        );
        assert_eq!(
            CollisionPair::new(
                "r1/link_a".to_string(),
                String::new(),
                CollisionKind::SelfCollision,
                0.0,
            ),
            None
        );
    }

    /* --- CollisionReport::from_pairs: in_collision == !pairs.is_empty()
           (Req 5.1) ---------------------------------------------------------- */

    #[test]
    fn from_pairs_empty_is_not_in_collision() {
        let report = CollisionReport::from_pairs(Vec::new(), None);
        assert!(!report.in_collision);
        assert!(report.pairs.is_empty());
        assert_eq!(report.in_collision, !report.pairs.is_empty());
    }

    #[test]
    fn from_pairs_non_empty_is_in_collision() {
        let pair = CollisionPair::new(
            "r1/link_a".to_string(),
            "r1/link_b".to_string(),
            CollisionKind::SelfCollision,
            -0.01,
        )
        .unwrap();

        let report = CollisionReport::from_pairs(vec![pair], Some(-0.01));
        assert!(report.in_collision);
        assert_eq!(report.pairs.len(), 1);
        assert_eq!(report.in_collision, !report.pairs.is_empty());
    }

    #[test]
    fn from_pairs_deduplicates_same_normalized_pair() {
        // Two contacts for the same normalized name pair (one given reversed)
        // must collapse to a single reported entry (Req 4.4), and the resulting
        // non-empty report is in collision (Req 5.1).
        let p1 = CollisionPair::new(
            "r1/link_a".to_string(),
            "r1/link_b".to_string(),
            CollisionKind::SelfCollision,
            0.02,
        )
        .unwrap();
        let p2 = CollisionPair::new(
            "r1/link_b".to_string(),
            "r1/link_a".to_string(),
            CollisionKind::SelfCollision,
            -0.03,
        )
        .unwrap();

        let report = CollisionReport::from_pairs(vec![p1, p2], Some(-0.03));
        assert_eq!(report.pairs.len(), 1);
        assert!(report.in_collision);
        assert_eq!(report.in_collision, !report.pairs.is_empty());
        // The smallest-distance (most severe) representative is kept.
        assert_eq!(report.pairs[0].distance_m, -0.03);
    }

    /* --- Contact margin range rejection + prior-config retention (Req 7.5) - */

    #[test]
    fn set_contact_margin_accepts_in_range() {
        let mut config = CollisionConfig::default();
        assert!(config.set_contact_margin(5.0).is_ok());
        assert_eq!(config.contact_margin_m, 5.0);

        // Inclusive bounds are accepted.
        assert!(config.set_contact_margin(CONTACT_MARGIN_MIN_M).is_ok());
        assert_eq!(config.contact_margin_m, CONTACT_MARGIN_MIN_M);
        assert!(config.set_contact_margin(CONTACT_MARGIN_MAX_M).is_ok());
        assert_eq!(config.contact_margin_m, CONTACT_MARGIN_MAX_M);
    }

    #[test]
    fn set_contact_margin_rejects_below_range_and_retains_prior() {
        let mut config = CollisionConfig::default();
        config.set_contact_margin(2.5).unwrap();
        let prior = config.clone();

        let err = config.set_contact_margin(-0.001);
        assert!(err.is_err());
        // Req 7.5: prior configuration is retained unchanged on rejection.
        assert_eq!(config, prior);
        assert_eq!(config.contact_margin_m, 2.5);
    }

    #[test]
    fn set_contact_margin_rejects_above_range_and_retains_prior() {
        let mut config = CollisionConfig::default();
        config.set_contact_margin(1.0).unwrap();
        let prior = config.clone();

        let err = config.set_contact_margin(10.0001);
        assert!(err.is_err());
        assert_eq!(config, prior);
        assert_eq!(config.contact_margin_m, 1.0);
    }

    #[test]
    fn set_contact_margin_rejects_non_finite_and_retains_prior() {
        let mut config = CollisionConfig::default();
        config.set_contact_margin(3.0).unwrap();
        let prior = config.clone();

        assert!(config.set_contact_margin(f64::NAN).is_err());
        assert_eq!(config, prior);
        assert!(config.set_contact_margin(f64::INFINITY).is_err());
        assert_eq!(config, prior);
        assert!(config.set_contact_margin(f64::NEG_INFINITY).is_err());
        assert_eq!(config, prior);
        assert_eq!(config.contact_margin_m, 3.0);
    }

    /* --- Waypoint stride range rejection + prior-config retention (Req 7.7)  */

    #[test]
    fn set_waypoint_stride_accepts_in_range() {
        let mut config = CollisionConfig::default();
        assert!(config.set_waypoint_stride(10).is_ok());
        assert_eq!(config.waypoint_stride, 10);

        // Inclusive bounds are accepted.
        assert!(config.set_waypoint_stride(WAYPOINT_STRIDE_MIN).is_ok());
        assert_eq!(config.waypoint_stride, WAYPOINT_STRIDE_MIN);
        assert!(config.set_waypoint_stride(WAYPOINT_STRIDE_MAX).is_ok());
        assert_eq!(config.waypoint_stride, WAYPOINT_STRIDE_MAX);
    }

    #[test]
    fn set_waypoint_stride_rejects_below_range_and_retains_prior() {
        let mut config = CollisionConfig::default();
        config.set_waypoint_stride(7).unwrap();
        let prior = config.clone();

        let err = config.set_waypoint_stride(0);
        assert!(err.is_err());
        // Req 7.7: prior configuration is retained unchanged on rejection.
        assert_eq!(config, prior);
        assert_eq!(config.waypoint_stride, 7);
    }

    #[test]
    fn set_waypoint_stride_rejects_above_range_and_retains_prior() {
        let mut config = CollisionConfig::default();
        config.set_waypoint_stride(42).unwrap();
        let prior = config.clone();

        let err = config.set_waypoint_stride(WAYPOINT_STRIDE_MAX + 1);
        assert!(err.is_err());
        assert_eq!(config, prior);
        assert_eq!(config.waypoint_stride, 42);
    }
}

/* ========================================================================= */
/* Property-based tests (task 3.3)                                           */
/*                                                                           */
/* Property 1: Symmetry and single-report (design.md "Correctness           */
/* Properties"). Validates Requirements 4.1, 4.2, 4.3, 4.4.                  */
/*                                                                           */
/* Kept in a separate `prop_tests` module so it coexists with the           */
/* `unit_tests` module above. The whole file is behind                       */
/* `#[cfg(feature = "collision")]` (see `main.rs`), so these run under       */
/* `cargo test --features collision`.                                        */
/* ========================================================================= */

#[cfg(test)]
mod prop_tests {
    use super::*;
    use proptest::prelude::*;
    use std::collections::HashSet;

    /// Uniformly pick one of the three collision kinds.
    fn kind_strategy() -> impl Strategy<Value = CollisionKind> {
        prop_oneof![
            Just(CollisionKind::SelfCollision),
            Just(CollisionKind::RobotRobot),
            Just(CollisionKind::RobotEnvironment),
        ]
    }

    /// A fully-qualified `<robot_id>/<link>` name drawn from a deliberately
    /// small pool so identical names (Req 4.3) and duplicate normalized pairs
    /// (Req 4.4) occur frequently across a generated batch.
    fn name_strategy() -> impl Strategy<Value = String> {
        (0usize..3, 0usize..4).prop_map(|(r, l)| format!("r{r}/link_{l}"))
    }

    /// A non-empty robot identifier component for `qualify` (Req 4.1).
    fn robot_id_strategy() -> impl Strategy<Value = String> {
        (0usize..3).prop_map(|r| format!("r{r}"))
    }

    /// A non-empty link/geom component for `qualify` (Req 4.1).
    fn link_strategy() -> impl Strategy<Value = String> {
        (0usize..4).prop_map(|l| format!("link_{l}"))
    }

    /// A raw (name_a, name_b, kind, distance) tuple fed to `CollisionPair::new`.
    fn raw_pair_strategy() -> impl Strategy<Value = (String, String, CollisionKind, f64)> {
        (
            name_strategy(),
            name_strategy(),
            kind_strategy(),
            -10.0f64..10.0,
        )
    }

    proptest! {
        /// Property 1 (symmetry): `new(a, b, ..)` and `new(b, a, ..)` yield the
        /// same normalized pair for any names, including identical names (both
        /// yield `None`). Validates Req 4.2, 4.3.
        #[test]
        fn prop_new_is_symmetric(
            a in name_strategy(),
            b in name_strategy(),
            kind in kind_strategy(),
            d in -10.0f64..10.0,
        ) {
            let forward = CollisionPair::new(a.clone(), b.clone(), kind, d);
            let reversed = CollisionPair::new(b, a, kind, d);
            prop_assert_eq!(forward, reversed);
        }

        /// Property 1 (ordering + single-report): after normalization through
        /// `CollisionReport::from_pairs`, every reported pair satisfies
        /// `object_a <= object_b` (Req 4.2) and `object_a != object_b`
        /// (Req 4.3), and no normalized name pair is reported more than once
        /// (Req 4.4).
        #[test]
        fn prop_report_pairs_are_ordered_distinct_and_unique(
            raws in prop::collection::vec(raw_pair_strategy(), 0..32),
        ) {
            let pairs: Vec<CollisionPair> = raws
                .into_iter()
                .filter_map(|(a, b, kind, d)| CollisionPair::new(a, b, kind, d))
                .collect();

            let report = CollisionReport::from_pairs(pairs, None);

            let mut seen: HashSet<(String, String)> = HashSet::new();
            for p in &report.pairs {
                // Req 4.2: normalized ordering by Unicode code point.
                prop_assert!(
                    p.object_a <= p.object_b,
                    "ordering violated: {} > {}",
                    p.object_a,
                    p.object_b
                );
                // Req 4.3: identical-name contacts are never reported.
                prop_assert_ne!(&p.object_a, &p.object_b);
                // Req 4.4: each normalized name pair appears exactly once.
                let key = (p.object_a.clone(), p.object_b.clone());
                prop_assert!(
                    seen.insert(key),
                    "duplicate normalized pair reported: {} <-> {}",
                    p.object_a,
                    p.object_b
                );
            }
        }

        /// Property 1 (qualified names + symmetry): pairs built via
        /// `CollisionPair::qualify` carry `<robot_id>/<link_or_geom>` names with
        /// exactly one `/` and non-empty components (Req 4.1), are normalized
        /// (Req 4.2, 4.3), and are invariant under swapping the two objects
        /// (symmetry).
        #[test]
        fn prop_qualify_yields_qualified_ordered_symmetric_pair(
            robot_a in robot_id_strategy(),
            link_a in link_strategy(),
            robot_b in robot_id_strategy(),
            link_b in link_strategy(),
            kind in kind_strategy(),
            d in -10.0f64..10.0,
        ) {
            let forward =
                CollisionPair::qualify(&robot_a, &link_a, &robot_b, &link_b, kind, d);

            if let Some(pair) = &forward {
                for name in [&pair.object_a, &pair.object_b] {
                    // Req 4.1: exactly one '/' separating two non-empty parts.
                    let parts: Vec<&str> = name.split('/').collect();
                    prop_assert_eq!(parts.len(), 2, "name not <robot_id>/<link>: {}", name);
                    prop_assert!(!parts[0].is_empty(), "empty robot_id in {}", name);
                    prop_assert!(!parts[1].is_empty(), "empty link in {}", name);
                }
                // Req 4.2 / 4.3.
                prop_assert!(pair.object_a <= pair.object_b);
                prop_assert_ne!(&pair.object_a, &pair.object_b);
            }

            // Symmetry: swapping the two objects yields the same normalized pair
            // (or both `None` for identical qualified names).
            let swapped =
                CollisionPair::qualify(&robot_b, &link_b, &robot_a, &link_a, kind, d);
            prop_assert_eq!(forward, swapped);
        }
    }
}

/* ========================================================================= */
/* CollisionModel unit tests (task 4.2)                                      */
/*                                                                           */
/* Feature-gated behaviour tests for the safe RAII `CollisionModel` wrapper.  */
/* Kept in a distinct `model_tests` module so they coexist with the          */
/* `unit_tests` and `prop_tests` modules above without disturbing them. The  */
/* whole file is behind `#[cfg(feature = "collision")]` (see `main.rs`), so   */
/* these run under `cargo test --features collision`.                        */
/*                                                                           */
/* These tests build a self-contained minimal 6-DOF URDF whose links carry   */
/* primitive <box> <collision> geometry (no external mesh files), written to */
/* a unique temp path. `pinocchio::urdf::buildGeom` handles primitive shapes */
/* internally without a mesh loader, so the fixture loads deterministically   */
/* and is genuinely self-colliding: every joint origin is coincident, so at  */
/* any configuration all link boxes overlap and every registered pair        */
/* collides. This lets us exercise both the input-validation paths (Req 2.7- */
/* 2.9) and the `stop_at_first` count bound (Req 2.4) against a real loaded   */
/* model.                                                                    */
/* ========================================================================= */

#[cfg(test)]
mod model_tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A minimal, self-contained 6-DOF revolute robot. Seven links (base +
    /// six) each carry a coincident 0.3 m collision box; the six revolute
    /// joints give `model.nq == 6` (so a `[f64; 6]` configuration matches the
    /// model's degrees of freedom). Because every joint origin is at the
    /// parent frame origin, all boxes overlap in every configuration, so the
    /// model self-collides and `addAllCollisionPairs` registers many pairs.
    const SELF_COLLIDING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="selfcollide6dof">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <collision>
      <geometry><box size="0.3 0.3 0.3"/></geometry>
    </collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Monotonic counter used to build unique temp file names so tests running
    /// in parallel never share a fixture path.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path =
            std::env::temp_dir().join(format!("cell_runner_collision_fixture_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp URDF fixture");
        path
    }

    /// Load the self-colliding fixture as a `CollisionModel`. The temp file is
    /// removed immediately after loading, since `from_urdf` fully consumes the
    /// file during the load and holds no reference to it afterwards.
    fn load_self_colliding_model() -> CollisionModel {
        let path = write_temp_urdf(SELF_COLLIDING_URDF);
        let model = CollisionModel::from_urdf(&path, None);
        let _ = std::fs::remove_file(&path);
        model.expect("self-contained primitive-box URDF fixture loads successfully")
    }

    /* --- Non-finite input handling for check_self (Req 2.8, 2.9) ----------- */
    /*                                                                          */
    /* Req 2.7 (exactly-6 length) is enforced by the `[f64; 6]` argument type   */
    /* at compile time: a configuration of any other length is unrepresentable  */
    /* and cannot reach `check_self`, so there is no runtime length check to     */
    /* exercise. These tests therefore cover the non-finite path (Req 2.8):      */
    /* NaN, +inf, and -inf must each yield a descriptive `Err` and must never    */
    /* panic (Req 2.9).                                                          */

    #[test]
    fn check_self_rejects_non_finite_joints_without_panic() {
        let mut model = load_self_colliding_model();

        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            // Place the offending value at each of the six indices in turn to
            // confirm the finiteness scan covers the whole configuration.
            for idx in 0..6 {
                let mut q = [0.0f64; 6];
                q[idx] = bad;

                let result = model.check_self(&q, false);
                assert!(
                    result.is_err(),
                    "non-finite joint {bad} at index {idx} must return Err, got {result:?}"
                );
                let msg = result.unwrap_err();
                assert!(
                    !msg.is_empty(),
                    "error message for non-finite joint must be descriptive (non-empty)"
                );
                assert!(
                    msg.contains("non-finite"),
                    "error message should identify the non-finite cause, got: {msg}"
                );
            }
        }
    }

    /* --- Non-finite input handling for geometry_placements (Req 2.8, 2.9) --- */

    #[test]
    fn geometry_placements_rejects_non_finite_joints_without_panic() {
        let mut model = load_self_colliding_model();

        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for idx in 0..6 {
                let mut q = [0.0f64; 6];
                q[idx] = bad;

                let result = model.geometry_placements(&q);
                assert!(
                    result.is_err(),
                    "non-finite joint {bad} at index {idx} must return Err from \
                     geometry_placements, got {result:?}"
                );
                assert!(
                    !result.unwrap_err().is_empty(),
                    "error message for non-finite joint must be descriptive (non-empty)"
                );
            }
        }
    }

    /* --- stop_at_first yields at most one pair (Req 2.4) ------------------- */

    #[test]
    fn stop_at_first_yields_at_most_one_pair() {
        let mut model = load_self_colliding_model();

        // Sanity: the fixture is genuinely self-colliding, and a full
        // (report-all) check surfaces more than one colliding pair. This makes
        // the stop-at-first bound meaningful rather than vacuous.
        let all = model
            .check_self(&[0.0; 6], false)
            .expect("clear-config self-check on a valid model succeeds");
        assert!(
            all.len() > 1,
            "fixture should self-collide on multiple pairs so the stop-at-first \
             bound is non-trivial; got {} pair(s)",
            all.len()
        );

        // Req 2.4: with stop-at-first enabled, at most one colliding pair is
        // returned.
        let first = model
            .check_self(&[0.0; 6], true)
            .expect("stop-at-first self-check on a valid model succeeds");
        assert!(
            first.len() <= 1,
            "stop_at_first must yield at most one pair, got {}",
            first.len()
        );
    }
}

/* ========================================================================= */
/* Base-transform invariance property tests (task 4.3)                       */
/*                                                                           */
/* Property 5: self-collision is intrinsic to the joint configuration. The   */
/* safe `CollisionModel::check_self` takes only the joint config; there is    */
/* no `base_transform` parameter, so a rigid world transform applied to the  */
/* robot's base cannot change the self-collision boolean, the set of         */
/* self-colliding pairs, or (within 1e-6 m) the minimum self-collision       */
/* distance. This module expresses that property with proptest: for random   */
/* 6-DOF joint configurations `q in [-pi, pi]` and random rigid              */
/* `Isometry3<f64>` transforms `T` (random axis-angle rotation + random       */
/* translation, no scale/shear/reflection), `check_self(q)` is invariant,    */
/* and composing the model's geometry placements through `T` via             */
/* `compose_world_placements` is a genuine rigid motion that preserves the   */
/* intrinsic pairwise geometry underlying self-collision.                    */
/*                                                                           */
/* Self-contained: this module defines its own primitive-<box> 6-DOF URDF    */
/* fixture and temp-file helper rather than reaching into `model_tests`. The */
/* whole file is behind `#[cfg(feature = "collision")]` (see `main.rs`), so   */
/* these run under `cargo test --features collision`.                        */
/*                                                                           */
/* Validates: Requirements 8.3, 8.4.                                         */
/* ========================================================================= */

#[cfg(test)]
mod base_transform_prop_tests {
    use super::*;
    use nalgebra::Unit;
    use proptest::prelude::*;
    use std::collections::HashSet;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A minimal, self-contained 6-DOF revolute robot whose seven links each
    /// carry a coincident 0.3 m collision box. Because every joint origin sits
    /// at the parent frame origin, all boxes overlap in every configuration, so
    /// the model self-collides and `addAllCollisionPairs` registers many pairs.
    /// This makes the fixture genuinely self-colliding so the invariance
    /// property is exercised on a non-empty pair set. Kept private to this
    /// module (do not reach into `model_tests`).
    const SELF_COLLIDING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="selfcollide6dof_bt">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <collision>
      <geometry><box size="0.3 0.3 0.3"/></geometry>
    </collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Monotonic counter used to build unique temp file names so tests running
    /// in parallel never share a fixture path.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_bt_fixture_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp URDF fixture");
        path
    }

    /// Load the self-colliding fixture as a `CollisionModel`. The temp file is
    /// removed immediately after loading; `from_urdf` fully consumes the file
    /// during the load and holds no reference to it afterwards.
    fn load_self_colliding_model() -> CollisionModel {
        let path = write_temp_urdf(SELF_COLLIDING_URDF);
        let model = CollisionModel::from_urdf(&path, None);
        let _ = std::fs::remove_file(&path);
        model.expect("self-contained primitive-box URDF fixture loads successfully")
    }

    /// A 6-DOF joint configuration with each value drawn from `[-pi, pi]`.
    fn joints_strategy() -> impl Strategy<Value = [f64; 6]> {
        let pi = std::f64::consts::PI;
        prop::array::uniform6(-pi..pi)
    }

    /// A genuinely rigid `Isometry3<f64>` (rotation + translation only, no
    /// scale, shear, or reflection): a random axis-angle rotation composed with
    /// a random translation. Degenerate (near-zero) rotation axes fall back to
    /// the z-axis so the unit quaternion is always well-defined.
    fn rigid_transform_strategy() -> impl Strategy<Value = Isometry3<f64>> {
        let pi = std::f64::consts::PI;
        (
            -10.0f64..10.0,
            -10.0f64..10.0,
            -10.0f64..10.0,
            -1.0f64..1.0,
            -1.0f64..1.0,
            -1.0f64..1.0,
            -pi..pi,
        )
            .prop_map(|(tx, ty, tz, ax, ay, az, angle)| {
                let axis_raw = Vector3::new(ax, ay, az);
                let axis = if axis_raw.norm() > 1e-9 {
                    Unit::new_normalize(axis_raw)
                } else {
                    Vector3::z_axis()
                };
                let rotation = UnitQuaternion::from_axis_angle(&axis, angle);
                let translation = Translation3::new(tx, ty, tz);
                Isometry3::from_parts(translation, rotation)
            })
    }

    /// The normalized identity of a reported self-collision pair: the ordered
    /// object names plus the kind. Distances are compared separately, so the
    /// key omits them.
    fn pair_key_set(pairs: &[CollisionPair]) -> HashSet<(String, String, CollisionKind)> {
        pairs
            .iter()
            .map(|p| (p.object_a.clone(), p.object_b.clone(), p.kind))
            .collect()
    }

    /// The minimum reported separation across a set of self-collision pairs, or
    /// `None` when no pair was returned. Uses `total_cmp` for a total order over
    /// floats (no NaN is expected from a successful check).
    fn min_distance(pairs: &[CollisionPair]) -> Option<f64> {
        pairs
            .iter()
            .map(|p| p.distance_m)
            .min_by(|a, b| a.total_cmp(b))
    }

    proptest! {
        // Loading + parsing a URDF and running Coal queries per case is
        // non-trivial, so cap the case count to keep the suite responsive while
        // still exercising a broad spread of configurations and transforms.
        #![proptest_config(ProptestConfig::with_cases(48))]

        /// Property 5: self-collision is intrinsic to the joint configuration
        /// and independent of the robot's base transform.
        ///
        /// For a random 6-DOF `q` and a random rigid world transform `T`:
        /// - `check_self(q)` returns the same normalized pair set and the same
        ///   in-collision boolean across repeated calls (Req 8.3): the safe API
        ///   exposes no `base_transform`, so the result cannot depend on it.
        /// - Composing the model's base-frame geometry placements through `T`
        ///   (`compose_world_placements`) is a genuine rigid motion: it
        ///   preserves the pairwise distances between collision-object origins
        ///   that underlie self-collision, confirming `T` introduces no scale,
        ///   shear, or reflection.
        /// - The minimum self-collision distance is stable to within 1e-6 m
        ///   under the transform (Req 8.4).
        ///
        /// Validates: Requirements 8.3, 8.4.
        #[test]
        fn prop_self_collision_is_base_transform_invariant(
            q in joints_strategy(),
            t in rigid_transform_strategy(),
        ) {
            let mut model = load_self_colliding_model();

            // Baseline self-collision result (no transform applied).
            let baseline = model
                .check_self(&q, false)
                .expect("self-check on a valid model succeeds");
            let baseline_set = pair_key_set(&baseline);
            let baseline_in_collision = !baseline.is_empty();
            let baseline_min = min_distance(&baseline);

            // Req 8.3: the pair set and in-collision boolean are intrinsic to
            // `q`; a repeated call yields an identical result. `check_self`
            // takes no base transform, so no base placement can perturb it.
            let repeated = model
                .check_self(&q, false)
                .expect("repeated self-check on a valid model succeeds");
            prop_assert_eq!(pair_key_set(&repeated), baseline_set.clone());
            prop_assert_eq!(!repeated.is_empty(), baseline_in_collision);

            // Applying the rigid world transform `T` to the model's base only
            // rigidly relocates its geometry. Verify that composing the
            // base-frame placements through `T` preserves every pairwise origin
            // distance (a defining property of a rigid motion, i.e. no scale,
            // shear, or reflection). This is the geometric reason self-collision
            // is base-transform invariant.
            let local = model
                .geometry_placements(&q)
                .expect("geometry placements on a valid model succeed");
            let world = compose_world_placements(&t, &local);
            prop_assert_eq!(world.len(), local.len());
            for i in 0..local.len() {
                for j in (i + 1)..local.len() {
                    let d_local = (local[i].pose.translation.vector
                        - local[j].pose.translation.vector)
                        .norm();
                    let d_world = (world[i].pose.translation.vector
                        - world[j].pose.translation.vector)
                        .norm();
                    prop_assert!(
                        (d_local - d_world).abs() <= 1e-9,
                        "rigid transform must preserve pairwise distances: \
                         objects {i},{j} local={d_local} world={d_world}"
                    );
                }
            }

            // Req 8.3: the self-collision result after composing the world
            // placement with `T` is unchanged (structurally, `check_self`
            // ignores the base transform entirely).
            let after = model
                .check_self(&q, false)
                .expect("self-check after transform composition succeeds");
            prop_assert_eq!(pair_key_set(&after), baseline_set);
            prop_assert_eq!(!after.is_empty(), baseline_in_collision);

            // Req 8.4: the minimum self-collision distance is stable to within
            // 1e-6 m under the rigid world transform.
            let after_min = min_distance(&after);
            match (baseline_min, after_min) {
                (Some(a), Some(b)) => prop_assert!(
                    (a - b).abs() <= 1e-6,
                    "min self-distance must be stable within 1e-6 m: {a} vs {b}"
                ),
                (None, None) => {}
                (a, b) => prop_assert!(
                    false,
                    "min self-distance presence must be invariant: {a:?} vs {b:?}"
                ),
            }
        }
    }
}

/* ========================================================================= */
/* Broadphase unit tests (task 5.2)                                          */
/*                                                                           */
/* Hand-placed AABB example/edge-case coverage for the world-AABB broadphase */
/* math: `Aabb::overlaps`, `Aabb::expanded`, and the margin-aware            */
/* `aabb_overlap` gate, plus small `narrowphase` tests using an injected     */
/* distance closure that confirm the gate skips the exact query for          */
/* non-overlapping expanded boxes and emits a normalized pair when the       */
/* injected distance is within the margin (Req 3.6, 3.8). A module distinct  */
/* from `unit_tests` / `prop_tests` / `model_tests` /                        */
/* `base_transform_prop_tests`; it touches none of them.                     */
/* ========================================================================= */

#[cfg(test)]
mod broadphase_tests {
    use super::*;

    /// Build an axis-aligned box directly from its `min`/`max` corners.
    fn boxx(min: [f64; 3], max: [f64; 3]) -> Aabb {
        Aabb::new(
            Vector3::new(min[0], min[1], min[2]),
            Vector3::new(max[0], max[1], max[2]),
        )
    }

    /* ----- Aabb::overlaps (no margin) ------------------------------------ */

    #[test]
    fn clearly_overlapping_boxes_overlap() {
        let a = boxx([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = boxx([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
        assert!(a.overlaps(&b));
        // Overlap is symmetric.
        assert!(b.overlaps(&a));
        // The margin-free gate agrees with the raw overlap test.
        assert!(aabb_overlap(&a, &b, 0.0));
    }

    #[test]
    fn clearly_disjoint_boxes_do_not_overlap() {
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([2.0, 2.0, 2.0], [3.0, 3.0, 3.0]);
        assert!(!a.overlaps(&b));
        assert!(!b.overlaps(&a));
        assert!(!aabb_overlap(&a, &b, 0.0));
    }

    #[test]
    fn separation_on_a_single_axis_is_enough_to_be_disjoint() {
        // Overlapping on x and y, but cleanly separated on z.
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([0.0, 0.0, 2.0], [1.0, 1.0, 3.0]);
        assert!(!a.overlaps(&b));
    }

    #[test]
    fn boxes_sharing_a_face_count_as_touching_overlap() {
        // Req 3.6: touching (a shared boundary) counts as overlap via `<=`.
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]); // shares the face x = 1
        assert!(a.overlaps(&b));
        assert!(b.overlaps(&a));
        assert!(aabb_overlap(&a, &b, 0.0));
    }

    #[test]
    fn boxes_sharing_only_a_corner_count_as_touching_overlap() {
        // Touching at a single corner is still a shared boundary (Req 3.6).
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([1.0, 1.0, 1.0], [2.0, 2.0, 2.0]); // shares corner (1, 1, 1)
        assert!(a.overlaps(&b));
        assert!(aabb_overlap(&a, &b, 0.0));
    }

    /* ----- Aabb::expanded ------------------------------------------------ */

    #[test]
    fn expanded_grows_box_by_margin_on_every_axis() {
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let e = a.expanded(0.5);
        assert_eq!(e.min, Vector3::new(-0.5, -0.5, -0.5));
        assert_eq!(e.max, Vector3::new(1.5, 1.5, 1.5));
    }

    #[test]
    fn expanded_by_zero_is_identity() {
        let a = boxx([-1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);
        assert_eq!(a.expanded(0.0), a);
    }

    #[test]
    fn expanded_clamps_negative_margin_to_zero() {
        // A negative margin must never shrink the box (Req 7.4: margins are
        // non-negative by construction).
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        assert_eq!(a.expanded(-5.0), a);
    }

    #[test]
    fn expansion_can_make_disjoint_boxes_touch() {
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([2.0, 0.0, 0.0], [3.0, 1.0, 1.0]); // gap of 1.0 on x
        assert!(!a.overlaps(&b));
        // Growing `a` alone by the gap makes the faces coincide (touching).
        assert!(a.expanded(1.0).overlaps(&b));
    }

    /* ----- aabb_overlap margin-boundary gate ----------------------------- */
    /* `aabb_overlap` expands *both* boxes by the margin, so two boxes with a */
    /* world-frame gap `g` are gated together when `g <= 2 * margin`.         */

    #[test]
    fn gate_includes_pair_separated_by_exactly_the_margin() {
        // A gap of exactly one margin sits comfortably within the gate's reach,
        // so a within-margin pair is never pruned (Req 3.8).
        let margin = 0.25;
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([1.0 + margin, 0.0, 0.0], [2.0 + margin, 1.0, 1.0]);
        assert!(!a.overlaps(&b)); // disjoint without any margin
        assert!(aabb_overlap(&a, &b, margin));
    }

    #[test]
    fn gate_boundary_is_inclusive_at_twice_the_margin() {
        // Both boxes expand by the margin, so the gate closes exactly at a gap
        // of 2 * margin, and the boundary is inclusive (`<=`).
        let margin = 0.5;
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let gap = 2.0 * margin;
        let b = boxx([1.0 + gap, 0.0, 0.0], [2.0 + gap, 1.0, 1.0]);
        assert!(aabb_overlap(&a, &b, margin));
    }

    #[test]
    fn gate_excludes_pair_separated_just_beyond_reach() {
        // Just over 2 * margin: the expanded boxes no longer overlap, so the
        // narrowphase is skipped (Req 3.8).
        let margin = 0.5;
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let gap = 2.0 * margin + 1e-6;
        let b = boxx([1.0 + gap, 0.0, 0.0], [2.0 + gap, 1.0, 1.0]);
        assert!(!aabb_overlap(&a, &b, margin));
    }

    #[test]
    fn gate_includes_pair_separated_just_under_reach() {
        let margin = 0.5;
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let gap = 2.0 * margin - 1e-6;
        let b = boxx([1.0 + gap, 0.0, 0.0], [2.0 + gap, 1.0, 1.0]);
        assert!(aabb_overlap(&a, &b, margin));
    }

    #[test]
    fn larger_margin_never_prunes_more_than_a_smaller_margin() {
        // The gate is monotonic in the margin: growing the margin only ever
        // adds overlaps, consistent with a conservative broadphase.
        let a = boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = boxx([3.0, 0.0, 0.0], [4.0, 1.0, 1.0]); // gap of 2.0 on x
        assert!(!aabb_overlap(&a, &b, 0.5)); // 2 * 0.5 = 1.0 < 2.0 -> pruned
        assert!(aabb_overlap(&a, &b, 1.0)); // 2 * 1.0 = 2.0 == gap -> inclusive
        assert!(aabb_overlap(&a, &b, 2.0)); // ample reach
    }

    /* ----- narrowphase gating with an injected distance closure ---------- */

    /// Build a world object with an explicit name and AABB. The pose does not
    /// affect the broadphase gate (which uses the supplied AABB), so identity
    /// is sufficient here.
    fn world_obj(name: &str, aabb: Aabb) -> WorldCollisionObject {
        WorldCollisionObject::new(name, Isometry3::identity(), aabb)
    }

    #[test]
    fn narrowphase_skips_exact_query_for_non_overlapping_expanded_boxes() {
        // Req 3.8: when the margin-expanded AABBs cannot overlap, the exact
        // distance query is never invoked and the pair is treated as clear.
        let a = vec![world_obj("r1/link0", boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]))];
        let b = vec![world_obj("r2/link0", boxx([5.0, 5.0, 5.0], [6.0, 6.0, 6.0]))];

        let mut calls = 0usize;
        let result = narrowphase(&a, &b, 0.1, CollisionKind::RobotRobot, |_, _| {
            calls += 1;
            Ok(0.0)
        })
        .expect("narrowphase with an infallible closure succeeds");

        assert_eq!(
            calls, 0,
            "the exact distance query must be skipped by the broadphase gate"
        );
        assert!(result.is_empty());
    }

    #[test]
    fn narrowphase_emits_normalized_pair_when_distance_within_margin() {
        // Req 3.6: overlapping expanded AABBs pass the gate; a separation at or
        // below the margin yields exactly one normalized pair
        // (object_a <= object_b by Unicode code point).
        let a = vec![world_obj("z/link", boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]))];
        let b = vec![world_obj("a/link", boxx([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]))];

        let margin = 0.1;
        let mut calls = 0usize;
        let result = narrowphase(&a, &b, margin, CollisionKind::RobotRobot, |_, _| {
            calls += 1;
            Ok(0.05) // <= margin
        })
        .expect("narrowphase succeeds");

        assert_eq!(calls, 1, "a gated pair must invoke the exact query exactly once");
        assert_eq!(result.len(), 1);
        let pair = &result[0];
        assert_eq!(pair.object_a, "a/link");
        assert_eq!(pair.object_b, "z/link");
        assert_eq!(pair.kind, CollisionKind::RobotRobot);
        assert_eq!(pair.distance_m, 0.05);
    }

    #[test]
    fn narrowphase_runs_query_but_emits_no_pair_when_distance_exceeds_margin() {
        // The gate passes (the expanded AABBs overlap) so the exact query runs,
        // but a separation beyond the margin is not reported as colliding.
        let a = vec![world_obj("r1/link", boxx([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]))];
        let b = vec![world_obj("r2/link", boxx([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]))];

        let margin = 0.1;
        let mut calls = 0usize;
        let result = narrowphase(&a, &b, margin, CollisionKind::RobotRobot, |_, _| {
            calls += 1;
            Ok(0.2) // > margin
        })
        .expect("narrowphase succeeds");

        assert_eq!(calls, 1, "a gated pair must invoke the exact query");
        assert!(
            result.is_empty(),
            "a separation beyond the margin must not be reported"
        );
    }
}

/* ========================================================================= */
/* FFI smoke tests (task 2.6)                                                */
/*                                                                           */
/* End-to-end checks that exercise the real `collision_*` C ABI through the  */
/* safe `CollisionModel` wrapper (Pinocchio `GeometryModel` + Coal backend).  */
/* Unlike the pure-Rust `unit_tests`/`prop_tests`, these load an actual       */
/* model and run `computeCollisions`, so they only build/run under            */
/* `cargo test --features collision`.                                        */
/*                                                                           */
/* `buildGeom` has no mesh loader wired up here, so both fixtures are         */
/* self-contained primitive-<box> 6-DOF URDFs written to a temp file (the     */
/* same approach as `model_tests`); this module defines its own copies rather */
/* than reaching into another test module.                                   */
/*                                                                           */
/*   - SMOKE_SELF_COLLIDING_URDF: seven coincident 0.3 m boxes (every joint   */
/*     origin at the parent frame origin), so the boxes overlap in *every*    */
/*     configuration. It registers many collision pairs (`num_pairs > 0`,     */
/*     Req 1.1/1.2) and self-collides at any config (Req 2.1/2.2).            */
/*   - SMOKE_CLEAR_URDF: seven small 0.05 m boxes whose joint origins each     */
/*     translate the child 1.0 m further along +x, so at a normal            */
/*     (all-zero) configuration the boxes sit ~0.95 m apart and nothing        */
/*     collides (`check_self` returns an empty vec, Req 2.3).                 */
/*                                                                           */
/* Validates: Requirements 1.1, 1.2, 2.1, 2.2, 2.3.                          */
/* ========================================================================= */

#[cfg(test)]
mod ffi_smoke_tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// Seven coincident 0.3 m collision boxes on a 6-DOF revolute chain. Every
    /// joint origin sits at the parent frame origin, so all boxes overlap in
    /// every configuration: the model registers many collision pairs and
    /// self-collides at any joint configuration.
    const SMOKE_SELF_COLLIDING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="smoke_selfcollide6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Seven small 0.05 m collision boxes on a 6-DOF revolute chain whose joint
    /// origins each translate the child 1.0 m further along +x. At a normal
    /// (all-zero) configuration the boxes sit ~0.95 m apart, so no collision
    /// pairs overlap and a self-collision check comes back clear. The model
    /// still registers collision pairs (via `addAllCollisionPairs`), they just
    /// do not collide at this configuration.
    const SMOKE_CLEAR_URDF: &str = r#"<?xml version="1.0"?>
<robot name="smoke_clear6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Monotonic counter so temp fixtures never collide across parallel tests.
    static SMOKE_FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = SMOKE_FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path =
            std::env::temp_dir().join(format!("cell_runner_collision_smoke_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp URDF smoke fixture");
        path
    }

    /// Load a fixture string as a `CollisionModel`, removing the temp file
    /// immediately after the load consumes it.
    fn load_model(urdf: &str) -> CollisionModel {
        let path = write_temp_urdf(urdf);
        let model = CollisionModel::from_urdf(&path, None);
        let _ = std::fs::remove_file(&path);
        model.expect("self-contained primitive-box URDF smoke fixture loads successfully")
    }

    /// Req 1.1 / 1.2: loading a dressed URDF builds a model that registers a
    /// positive number of collision pairs.
    #[test]
    fn dressed_urdf_registers_collision_pairs() {
        let model = load_model(SMOKE_SELF_COLLIDING_URDF);
        assert!(
            model.num_pairs() > 0,
            "a dressed URDF with <collision> geometry must register at least one \
             collision pair, got {}",
            model.num_pairs()
        );
    }

    /// Req 2.1 / 2.2: a deliberately self-colliding configuration reports at
    /// least one colliding pair. The coincident-box fixture self-collides at
    /// any configuration, so a normal (all-zero) config is in collision.
    #[test]
    fn self_colliding_configuration_reports_a_pair() {
        let mut model = load_model(SMOKE_SELF_COLLIDING_URDF);
        let pairs = model
            .check_self(&[0.0; 6], false)
            .expect("self-collision check on a valid model succeeds");
        assert!(
            !pairs.is_empty(),
            "the coincident-box fixture must report at least one self-colliding pair"
        );
        for pair in &pairs {
            assert_eq!(
                pair.kind,
                CollisionKind::SelfCollision,
                "self-collision check must report SelfCollision pairs only"
            );
        }
    }

    /// Req 2.3: a clear configuration reports no colliding pairs. The
    /// spaced-box fixture keeps every box ~0.95 m apart at the all-zero
    /// configuration, so `check_self` returns an empty vec.
    #[test]
    fn clear_configuration_reports_no_pairs() {
        let mut model = load_model(SMOKE_CLEAR_URDF);
        let pairs = model
            .check_self(&[0.0; 6], false)
            .expect("self-collision check on a valid model succeeds");
        assert!(
            pairs.is_empty(),
            "a collision-free configuration must report no self-colliding pairs, got {pairs:?}"
        );
    }
}

/* ========================================================================= */
/* Scene-level property tests (tasks 6.3 - 6.7)                              */
/*                                                                           */
/* Property-based coverage for `SceneCollisionChecker::check_configuration`  */
/* (Properties 2, 3, 4, 6, 7 from the design's Correctness Properties). This */
/* module is distinct from `unit_tests` / `prop_tests` / `model_tests` /     */
/* `base_transform_prop_tests` / `broadphase_tests` / `ffi_smoke_tests`; it   */
/* touches none of them and defines its own self-contained primitive-<box>   */
/* URDF/SRDF fixtures and temp-file helpers.                                  */
/*                                                                           */
/* Design note on the cross-model distance proxy: the C ABI exposes an exact */
/* convex-hull query only *within* one loaded model. Robot-robot and         */
/* robot-environment separations therefore use the signed world-AABB proxy   */
/* (`aabb_signed_distance`). These property tests are written to be robust to */
/* that proxy: they assert the report *invariants* that hold for both the    */
/* exact self-collision query and the AABB proxy (margin predicate,          */
/* in_collision consistency, determinism, monotonicity, totality) rather than */
/* exact separation magnitudes. Self-collision, which stays inside one model  */
/* and uses the exact Coal query, additionally anchors the SRDF-soundness     */
/* property.                                                                  */
/*                                                                           */
/* The whole file is behind `#[cfg(feature = "collision")]` (see `main.rs`),  */
/* so these run under `cargo test --features collision`.                      */
/*                                                                           */
/* Validates: Properties 2, 3, 4, 6, 7 -> Requirements 5.1, 5.2, 5.3, 8.1,   */
/* 8.2, 1.3, 2.5, 9.1, 7.3.                                                   */
/* ========================================================================= */

#[cfg(test)]
mod scene_prop_tests {
    use super::*;
    use proptest::prelude::*;
    use std::collections::HashSet;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A minimal, self-contained 6-DOF revolute robot whose seven links each
    /// carry a coincident 0.3 m collision box. Every joint origin sits at the
    /// parent frame origin, so all boxes overlap in every configuration: the
    /// model self-collides and `addAllCollisionPairs` registers many pairs.
    /// This makes the fixture genuinely self-colliding so the scene properties
    /// are exercised on a non-empty pair set. The robot name matches the SRDF
    /// below so `removeCollisionPairs` applies cleanly. Kept private to this
    /// module.
    const SELF_COLLIDING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="scene_selfcollide6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// An SRDF that disables *every* unordered link pair of
    /// `SELF_COLLIDING_URDF` (C(7,2) = 21 entries). Loading the URDF with this
    /// SRDF removes all registered collision pairs, so the model's post-SRDF
    /// pair count is zero and no self-collision can ever be reported. This is
    /// the strongest possible form of "SRDF-disabled pairs are never reported"
    /// (Req 1.3, 2.5) and needs no fragile correlation between SRDF link names
    /// and Coal geometry-object names.
    const DISABLE_ALL_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="scene_selfcollide6dof">
  <disable_collisions link1="base_link" link2="link1" reason="Test"/>
  <disable_collisions link1="base_link" link2="link2" reason="Test"/>
  <disable_collisions link1="base_link" link2="link3" reason="Test"/>
  <disable_collisions link1="base_link" link2="link4" reason="Test"/>
  <disable_collisions link1="base_link" link2="link5" reason="Test"/>
  <disable_collisions link1="base_link" link2="link6" reason="Test"/>
  <disable_collisions link1="link1" link2="link2" reason="Test"/>
  <disable_collisions link1="link1" link2="link3" reason="Test"/>
  <disable_collisions link1="link1" link2="link4" reason="Test"/>
  <disable_collisions link1="link1" link2="link5" reason="Test"/>
  <disable_collisions link1="link1" link2="link6" reason="Test"/>
  <disable_collisions link1="link2" link2="link3" reason="Test"/>
  <disable_collisions link1="link2" link2="link4" reason="Test"/>
  <disable_collisions link1="link2" link2="link5" reason="Test"/>
  <disable_collisions link1="link2" link2="link6" reason="Test"/>
  <disable_collisions link1="link3" link2="link4" reason="Test"/>
  <disable_collisions link1="link3" link2="link5" reason="Test"/>
  <disable_collisions link1="link3" link2="link6" reason="Test"/>
  <disable_collisions link1="link4" link2="link5" reason="Test"/>
  <disable_collisions link1="link4" link2="link6" reason="Test"/>
  <disable_collisions link1="link5" link2="link6" reason="Test"/>
</robot>
"#;

    /// Monotonic counter used to build unique temp file names so tests running
    /// in parallel never share a fixture path.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp file with the given extension and
    /// return that path.
    fn write_temp(contents: &str, ext: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_scene_{pid}_{n}.{ext}"));
        std::fs::write(&path, contents).expect("write temp fixture");
        path
    }

    /// Load a bare `CollisionModel` from the self-colliding URDF, optionally
    /// applying `DISABLE_ALL_SRDF`. Temp files are removed immediately after the
    /// load consumes them.
    fn load_model(with_srdf: bool) -> CollisionModel {
        let urdf = write_temp(SELF_COLLIDING_URDF, "urdf");
        let srdf = if with_srdf {
            Some(write_temp(DISABLE_ALL_SRDF, "srdf"))
        } else {
            None
        };
        let model = CollisionModel::from_urdf(&urdf, srdf.as_deref());
        let _ = std::fs::remove_file(&urdf);
        if let Some(s) = &srdf {
            let _ = std::fs::remove_file(s);
        }
        model.expect("self-contained primitive-box URDF fixture loads successfully")
    }

    /// Build a single-robot scene (robot id `r0`, identity base transform) from
    /// the self-colliding fixture under `config`. The temp URDF is removed after
    /// the checker has loaded it.
    fn build_single_robot_scene(config: CollisionConfig) -> SceneCollisionChecker {
        let urdf = write_temp(SELF_COLLIDING_URDF, "urdf");
        let specs = [RobotCollisionSpec::new(
            "r0",
            urdf.clone(),
            None,
            Isometry3::identity(),
        )];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("single-robot scene builds from a self-contained fixture")
    }

    /// Build a two-robot scene (`r0` at the origin, `r1` translated +0.2 m in x
    /// so their base boxes overlap in the world frame) from the self-colliding
    /// fixture under `config`. Overlapping bases guarantee the robot-robot class
    /// is exercised in addition to per-robot self-collision. The temp URDF is
    /// removed after the checker has loaded it.
    fn build_two_robot_scene(config: CollisionConfig) -> SceneCollisionChecker {
        let urdf = write_temp(SELF_COLLIDING_URDF, "urdf");
        let specs = [
            RobotCollisionSpec::new("r0", urdf.clone(), None, Isometry3::identity()),
            RobotCollisionSpec::new(
                "r1",
                urdf.clone(),
                None,
                Isometry3::translation(0.2, 0.0, 0.0),
            ),
        ];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("two-robot scene builds from a self-contained fixture")
    }

    /// A 6-DOF joint configuration with each value drawn from `[-pi, pi]`.
    fn joints_strategy() -> impl Strategy<Value = [f64; 6]> {
        let pi = std::f64::consts::PI;
        prop::array::uniform6(-pi..pi)
    }

    /// A single joint value that is frequently finite but sometimes NaN or
    /// +/- infinity, used to exercise the totality / never-panic path (Req 9.1).
    fn maybe_non_finite_strategy() -> impl Strategy<Value = f64> {
        prop_oneof![
            1 => Just(f64::NAN),
            1 => Just(f64::INFINITY),
            1 => Just(f64::NEG_INFINITY),
            5 => (-1.0e6f64..1.0e6),
        ]
    }

    /// A possibly-malformed 6-DOF configuration (may contain non-finite values).
    fn arbitrary_joints_strategy() -> impl Strategy<Value = [f64; 6]> {
        prop::array::uniform6(maybe_non_finite_strategy())
    }

    /// An arbitrary `SceneConfig` for the two-robot scene: each of `r0`, `r1`,
    /// and an unknown robot id `ghost` may independently be present or absent,
    /// and each present entry carries possibly-non-finite joints. This drives
    /// missing-robot (unchecked), unknown-robot (ignored), and non-finite-joint
    /// (errored) paths simultaneously (Req 9.1, 9.4).
    fn arbitrary_scene_config_strategy() -> impl Strategy<Value = SceneConfig> {
        (
            proptest::option::of(arbitrary_joints_strategy()),
            proptest::option::of(arbitrary_joints_strategy()),
            proptest::option::of(arbitrary_joints_strategy()),
        )
            .prop_map(|(r0, r1, ghost)| {
                let mut cfg = SceneConfig::new();
                if let Some(q) = r0 {
                    cfg.insert("r0", q);
                }
                if let Some(q) = r1 {
                    cfg.insert("r1", q);
                }
                if let Some(q) = ghost {
                    // Unknown robot id: not present in the scene, must be ignored.
                    cfg.insert("ghost", q);
                }
                cfg
            })
    }

    /// A NaN-safe bitwise snapshot of a scene configuration over the fixed set
    /// of robot ids the arbitrary strategy can populate (`r0`, `r1`, `ghost`).
    ///
    /// `SceneConfig`'s derived `PartialEq` compares joint values with float
    /// equality, under which `NaN != NaN`; since the totality strategy injects
    /// NaN deliberately, a direct `==` would spuriously report inequality even
    /// when nothing changed. Comparing raw bit patterns (`f64::to_bits`) instead
    /// gives a total, NaN-safe equality suitable for asserting non-mutation.
    fn scene_config_bits(cfg: &SceneConfig) -> Vec<(&'static str, Option<[u64; 6]>)> {
        ["r0", "r1", "ghost"]
            .iter()
            .map(|id| {
                let bits = cfg.get(id).map(|q| {
                    let mut b = [0u64; 6];
                    for (dst, v) in b.iter_mut().zip(q.iter()) {
                        *dst = v.to_bits();
                    }
                    b
                });
                (*id, bits)
            })
            .collect()
    }

    /// The normalized identity of a reported pair (ordered names + kind), used
    /// for subset/superset comparisons. Distances are compared separately when
    /// needed.
    fn pair_key_set(report: &CollisionReport) -> HashSet<(String, String, CollisionKind)> {
        report
            .pairs
            .iter()
            .map(|p| (p.object_a.clone(), p.object_b.clone(), p.kind))
            .collect()
    }

    proptest! {
        // Each case parses a URDF and runs Coal queries, so cap the case count
        // to keep the suite responsive while covering a broad input spread.
        #![proptest_config(ProptestConfig::with_cases(32))]

        /// Property 2: Consistency with distance (task 6.3).
        ///
        /// For a random configuration and a random contact margin in the valid
        /// range, under a full report-all check:
        /// - every reported pair separates by at most the contact margin, and
        ///   for a zero margin only strictly-penetrating pairs are reported
        ///   (Req 5.2, 5.3, and the zero/positive margin semantics of Req 7.1/7.2);
        /// - `in_collision` is true iff at least one pair is reported (Req 5.1).
        ///
        /// Validates: Requirements 5.1, 5.2, 5.3.
        #[test]
        fn prop_report_distance_consistency(
            q in joints_strategy(),
            margin in CONTACT_MARGIN_MIN_M..=CONTACT_MARGIN_MAX_M,
        ) {
            let config = CollisionConfig {
                contact_margin_m: margin,
                report_all: true,
                ..CollisionConfig::default()
            };
            let mut scene = build_two_robot_scene(config);
            let joints = SceneConfig::new().with_robot("r0", q).with_robot("r1", q);

            let report = scene.check_configuration(&joints);

            // Req 5.1: under report-all, in_collision <=> some pair reported.
            prop_assert_eq!(report.in_collision, !report.pairs.is_empty());

            for p in &report.pairs {
                // Req 5.2 / 5.3: every reported pair is within the margin.
                prop_assert!(
                    p.distance_m <= margin,
                    "reported pair {}<->{} has distance {} > margin {}",
                    p.object_a, p.object_b, p.distance_m, margin
                );
                // Zero/positive margin semantics (Req 7.1/7.2): a zero margin
                // reports only strict penetration.
                if margin <= 0.0 {
                    prop_assert!(
                        p.distance_m < 0.0,
                        "with a zero margin only penetrating pairs may be \
                         reported; {}<->{} has distance {}",
                        p.object_a, p.object_b, p.distance_m
                    );
                }
            }
        }

        /// Property 4: Determinism (task 6.4).
        ///
        /// `check_configuration` invoked twice with identical scene,
        /// configuration, and joint inputs yields bit-for-bit identical reports:
        /// same pairs, same ordering, same minimum distance, and same per-model
        /// classification. Two overlapping robots produce multiple pairs so the
        /// ordering determinism of Req 8.2 is non-trivial.
        ///
        /// Validates: Requirements 8.1, 8.2.
        #[test]
        fn prop_check_configuration_deterministic(
            q0 in joints_strategy(),
            q1 in joints_strategy(),
            margin in CONTACT_MARGIN_MIN_M..=CONTACT_MARGIN_MAX_M,
            report_all in any::<bool>(),
        ) {
            let config = CollisionConfig {
                contact_margin_m: margin,
                report_all,
                ..CollisionConfig::default()
            };
            let mut scene = build_two_robot_scene(config);
            let joints = SceneConfig::new().with_robot("r0", q0).with_robot("r1", q1);

            let first = scene.check_configuration(&joints);
            let second = scene.check_configuration(&joints);

            // Full structural equality: pairs (and their order), min distance,
            // and per-model status must all match (Req 8.1, 8.2).
            prop_assert_eq!(first, second);
        }

        /// Property 3: Self-collision filtering soundness (task 6.5).
        ///
        /// SRDF-disabled pairs are never reported. This is validated in its
        /// strongest form by disabling *every* link pair via SRDF: the model's
        /// post-SRDF pair count is zero and `check_self` returns no pair for any
        /// configuration (Req 1.3, 2.5). The unfiltered model additionally
        /// anchors the sound invariants that every reported self pair is a
        /// genuine `SelfCollision` with distinct object names (never
        /// `object_a == object_b`) and that the set of distinct reported pairs
        /// is a subset of the model's registered pair set
        /// (distinct-reported-count <= num_pairs()).
        ///
        /// Limitation: rather than correlate individual SRDF link names against
        /// Coal geometry-object names (a fragile, version-dependent mapping),
        /// soundness is established by the disable-all reduction plus the
        /// reported-subset-of-registered invariant, which together imply no
        /// disabled pair can ever surface.
        ///
        /// Validates: Requirements 1.3, 2.5.
        #[test]
        fn prop_srdf_disabled_pairs_never_reported(q in joints_strategy()) {
            // Disable-all SRDF: post-SRDF pair set is empty (Req 1.3).
            let mut filtered = load_model(true);
            prop_assert_eq!(
                filtered.num_pairs(),
                0,
                "an SRDF disabling every link pair must leave zero registered pairs"
            );
            let filtered_pairs = filtered
                .check_self(&q, false)
                .expect("self-check on the filtered model succeeds");
            prop_assert!(
                filtered_pairs.is_empty(),
                "no self-collision pair may be reported when every pair is \
                 SRDF-disabled; got {:?}",
                filtered_pairs
            );

            // Unfiltered model: sound invariants on whatever is reported.
            let mut unfiltered = load_model(false);
            let registered = unfiltered.num_pairs();
            let reported = unfiltered
                .check_self(&q, false)
                .expect("self-check on the unfiltered model succeeds");

            let mut distinct: HashSet<(String, String)> = HashSet::new();
            for p in &reported {
                prop_assert_ne!(
                    &p.object_a,
                    &p.object_b,
                    "a self-collision pair must involve two distinct objects"
                );
                prop_assert_eq!(
                    p.kind,
                    CollisionKind::SelfCollision,
                    "check_self must only report SelfCollision pairs"
                );
                distinct.insert((p.object_a.clone(), p.object_b.clone()));
            }
            // Reported pairs are a subset of the registered (post-SRDF) set.
            prop_assert!(
                distinct.len() <= registered,
                "distinct reported self pairs ({}) must not exceed registered \
                 pairs ({})",
                distinct.len(),
                registered
            );
        }

        /// Property 6: Totality / no-panic (task 6.6).
        ///
        /// For arbitrary scene configurations - including missing robots,
        /// unknown robot ids, and non-finite (NaN / +/-inf) joint values -
        /// `check_configuration` returns a report and never panics, and it
        /// leaves the input configuration unmodified. Every scene model is
        /// classified exactly once (Req 9.7) and `in_collision` stays consistent
        /// with the reported pairs.
        ///
        /// Validates: Requirements 9.1.
        #[test]
        fn prop_check_configuration_total_no_panic(cfg in arbitrary_scene_config_strategy()) {
            let config = CollisionConfig {
                report_all: true,
                ..CollisionConfig::default()
            };
            let mut scene = build_two_robot_scene(config);

            // Input must not be mutated by the check (Req 9.2 is adjacent; the
            // totality property asserts non-mutation of the supplied config).
            // Compared bitwise because the strategy injects NaN, under which
            // float `==` is not reflexive.
            let before = scene_config_bits(&cfg);
            let report = scene.check_configuration(&cfg);
            prop_assert_eq!(
                scene_config_bits(&cfg),
                before,
                "check_configuration must not mutate its input"
            );

            // A report is always produced (no panic reached this point).
            prop_assert_eq!(report.in_collision, !report.pairs.is_empty());

            // Every scene model (both robots, no environment) is classified
            // exactly once with a unique model id (Req 9.7).
            prop_assert_eq!(report.model_status.len(), 2);
            let ids: HashSet<&str> = report
                .model_status
                .iter()
                .map(|m| m.model_id.as_str())
                .collect();
            prop_assert_eq!(ids.len(), report.model_status.len());
        }

        /// Property 7: Margin monotonicity (task 6.7).
        ///
        /// For a fixed configuration and two margins `m1 <= m2`, the set of
        /// colliding pairs at the smaller margin is a subset of the set at the
        /// larger margin. Uses two overlapping robots so both self-collision
        /// (exact) and robot-robot (AABB proxy) classes contribute, and both the
        /// margin-expanded broadphase gate and the margin collision predicate
        /// are monotone in the margin.
        ///
        /// Validates: Requirements 7.3.
        #[test]
        fn prop_margin_monotonicity(
            q0 in joints_strategy(),
            q1 in joints_strategy(),
            ma in CONTACT_MARGIN_MIN_M..=CONTACT_MARGIN_MAX_M,
            mb in CONTACT_MARGIN_MIN_M..=CONTACT_MARGIN_MAX_M,
        ) {
            let lo = ma.min(mb);
            let hi = ma.max(mb);
            let joints = SceneConfig::new().with_robot("r0", q0).with_robot("r1", q1);

            let mut scene_lo = build_two_robot_scene(CollisionConfig {
                contact_margin_m: lo,
                report_all: true,
                ..CollisionConfig::default()
            });
            let mut scene_hi = build_two_robot_scene(CollisionConfig {
                contact_margin_m: hi,
                report_all: true,
                ..CollisionConfig::default()
            });

            let pairs_lo = pair_key_set(&scene_lo.check_configuration(&joints));
            let pairs_hi = pair_key_set(&scene_hi.check_configuration(&joints));

            // Req 7.3: increasing the margin never removes a reported pair.
            prop_assert!(
                pairs_lo.is_subset(&pairs_hi),
                "pairs(m={}) must be a subset of pairs(m={}); \
                 lo-only pairs: {:?}",
                lo,
                hi,
                pairs_lo.difference(&pairs_hi).collect::<Vec<_>>()
            );
        }
    }
}

/* ========================================================================= */
/* Discrete trajectory sampling unit tests (task 8.2)                        */
/*                                                                           */
/* Example/edge-case coverage for `SceneCollisionChecker::check_trajectory`  */
/* (design "Discrete trajectory check", Req 6.1, 6.5, 6.7, 6.8, 6.9). A      */
/* module distinct from every other test module in this file; it touches     */
/* none of them and defines its own self-contained primitive-<box> URDF      */
/* fixture and temp-file helpers.                                            */
/*                                                                           */
/* `check_trajectory` delegates each sampled waypoint to                     */
/* `check_configuration`, whose per-configuration outcome depends on the      */
/* loaded Coal geometry. Rather than stub that outcome, these tests build     */
/* real scenes whose self-collision status is known by construction:          */
/*   * the coincident-box fixture self-collides at *every* configuration, so  */
/*     every sampled (checkable) waypoint is in collision. This turns the     */
/*     set of reported waypoints into a direct readout of the sampling grid,  */
/*     making stride behaviour and first-hit vs report-all observable without */
/*     depending on any particular collision distance.                        */
/*                                                                           */
/* The whole file is behind `#[cfg(feature = "collision")]` (see `main.rs`),  */
/* so these run under `cargo test --features collision`.                      */
/* ========================================================================= */

#[cfg(test)]
mod trajectory_tests {
    use super::*;
    use crate::path::JointPath;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A minimal, self-contained 6-DOF revolute robot whose seven links each
    /// carry a coincident 0.3 m collision box. Every joint origin sits at the
    /// parent frame origin, so all boxes overlap in *every* configuration: the
    /// model self-collides at any joint vector. This makes each sampled,
    /// checkable waypoint deterministically in-collision, so the reported
    /// waypoints directly reveal the sampling grid. Kept private to this module.
    const COINCIDENT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="traj_selfcollide6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Monotonic counter so temp fixtures never collide across parallel tests.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path =
            std::env::temp_dir().join(format!("cell_runner_collision_traj_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp URDF fixture");
        path
    }

    /// A contact margin comfortably larger than any coincident-box penetration,
    /// so a coincident (fully overlapping) pair is reported as colliding under
    /// either the zero- or positive-margin predicate. Keeps the "every sampled
    /// waypoint collides" invariant robust to the exact reported distance sign.
    const COLLIDE_MARGIN_M: f64 = 0.05;

    /// Build a single-robot scene (`r0`, identity base) from the coincident
    /// fixture under `config`. The temp URDF is removed after the checker loads
    /// it.
    fn build_single_robot_scene(config: CollisionConfig) -> SceneCollisionChecker {
        let urdf = write_temp_urdf(COINCIDENT_URDF);
        let specs = [RobotCollisionSpec::new(
            "r0",
            urdf.clone(),
            None,
            Isometry3::identity(),
        )];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("single-robot coincident scene builds from a self-contained fixture")
    }

    /// Build a two-robot scene (`r0` at the origin, `r1` translated +0.2 m in x)
    /// from the coincident fixture under `config`. The temp URDF is removed after
    /// the checker loads it.
    fn build_two_robot_scene(config: CollisionConfig) -> SceneCollisionChecker {
        let urdf = write_temp_urdf(COINCIDENT_URDF);
        let specs = [
            RobotCollisionSpec::new("r0", urdf.clone(), None, Isometry3::identity()),
            RobotCollisionSpec::new("r1", urdf.clone(), None, Isometry3::translation(0.2, 0.0, 0.0)),
        ];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("two-robot coincident scene builds from a self-contained fixture")
    }

    /// An endpoints path that samples to a finite 6-vector for every progress
    /// `s ∈ [0, 1]` (never `None`), so a robot carrying it is always checkable.
    fn checkable_path() -> JointPath {
        JointPath::Endpoints {
            start: [0.0; 6],
            end: [0.5; 6],
        }
    }

    /// A report-all config with the given stride and the colliding margin.
    fn report_all_config(stride: usize) -> CollisionConfig {
        CollisionConfig {
            contact_margin_m: COLLIDE_MARGIN_M,
            waypoint_stride: stride,
            report_all: true,
            ..CollisionConfig::default()
        }
    }

    /// Collect the sampled knot indices from a report's `all` list.
    fn hit_waypoints(report: &TrajectoryCollisionReport) -> Vec<usize> {
        report.all.iter().map(|w| w.waypoint).collect()
    }

    /* --- Stride behaviour: sample every Nth knot (Req 6.1) ---------------- */

    #[test]
    fn stride_one_samples_every_knot() {
        // N = 10, stride = 1 => k = 0, 1, ..., 10 (11 samples). The coincident
        // fixture collides at each, so every knot appears in `all`.
        let mut scene = build_single_robot_scene(report_all_config(1));
        let traj = SceneTrajectory::new(10, 10).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert!(report.collided, "coincident fixture must collide at every sampled knot");
        assert_eq!(
            hit_waypoints(&report),
            (0..=10).collect::<Vec<_>>(),
            "stride 1 must sample every knot k = 0..=N"
        );
    }

    #[test]
    fn stride_two_samples_every_second_knot() {
        // N = 10, stride = 2 => k = 0, 2, 4, 6, 8, 10 (6 samples).
        let mut scene = build_single_robot_scene(report_all_config(2));
        let traj = SceneTrajectory::new(10, 10).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert_eq!(
            hit_waypoints(&report),
            vec![0, 2, 4, 6, 8, 10],
            "stride 2 must sample every second knot"
        );
    }

    #[test]
    fn stride_three_samples_every_third_knot() {
        // N = 10, stride = 3 => k = 0, 3, 6, 9 (the final k <= N).
        let mut scene = build_single_robot_scene(report_all_config(3));
        let traj = SceneTrajectory::new(10, 10).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert_eq!(
            hit_waypoints(&report),
            vec![0, 3, 6, 9],
            "stride 3 must sample every third knot without overrunning N"
        );
    }

    /* --- First-hit vs report-all (Req 6.3, 6.4, 6.5) --------------------- */

    #[test]
    fn report_all_disabled_stops_at_first_colliding_waypoint() {
        // With report-all off, checking stops at the first colliding waypoint:
        // exactly one hit, and it is the smallest-progress sample (k = 0).
        let config = CollisionConfig {
            contact_margin_m: COLLIDE_MARGIN_M,
            waypoint_stride: 1,
            report_all: false,
            ..CollisionConfig::default()
        };
        let mut scene = build_single_robot_scene(config);
        let traj = SceneTrajectory::new(5, 5).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert!(report.collided, "the trajectory collides at the first sample");
        assert_eq!(report.all.len(), 1, "stop-at-first records exactly one waypoint");
        let first = report.first.expect("a collision was found, so `first` is set");
        assert_eq!(first.waypoint, 0, "the first colliding waypoint is k = 0");
        assert_eq!(first.progress, 0.0, "the first colliding progress is s = 0.0");
    }

    #[test]
    fn report_all_enabled_collects_every_colliding_waypoint_in_ascending_progress() {
        // With report-all on, every sampled waypoint is collected in ascending
        // progress order.
        let mut scene = build_single_robot_scene(report_all_config(1));
        let traj = SceneTrajectory::new(5, 5).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert!(report.collided);
        assert_eq!(
            hit_waypoints(&report),
            (0..=5).collect::<Vec<_>>(),
            "report-all collects every sampled waypoint"
        );
        // Ascending progress order (Req 6.4).
        for w in report.all.windows(2) {
            assert!(
                w[0].progress < w[1].progress,
                "report-all waypoints must be in strictly ascending progress: {} !< {}",
                w[0].progress,
                w[1].progress
            );
        }
        // `first` is the smallest-progress hit (Req 6.3).
        let first = report.first.expect("collision found");
        assert_eq!(first.waypoint, 0);
        assert_eq!(first.progress, 0.0);
    }

    /* --- JointPath::None skipping (Req 6.7) ------------------------------ */

    #[test]
    fn none_path_robot_is_recorded_unchecked_and_skipped() {
        // r0 carries a checkable path; r1's path is `None`. r1 must be recorded
        // as unchecked and excluded from every check, while r0 is still checked
        // (and self-collides).
        let mut scene = build_two_robot_scene(report_all_config(1));
        let traj = SceneTrajectory::new(4, 4)
            .with_path("r0", checkable_path())
            .with_path("r1", JointPath::None);

        let report = scene.check_trajectory(&traj);

        assert_eq!(
            report.unchecked_robots,
            vec!["r1".to_string()],
            "the `None`-path robot must be the only one recorded as unchecked"
        );
        assert!(report.collided, "the checkable robot r0 still self-collides");
        // No reported pair may reference the skipped robot r1.
        for hit in &report.all {
            for p in &hit.report.pairs {
                assert!(
                    !p.object_a.starts_with("r1/") && !p.object_b.starts_with("r1/"),
                    "a `None`-path robot must not appear in any reported pair: {p:?}"
                );
            }
        }
    }

    /* --- Sample-count rejection (Req 6.8) -------------------------------- */

    #[test]
    fn sample_count_below_one_is_rejected_with_no_checking() {
        let mut scene = build_single_robot_scene(report_all_config(1));
        // sample_count = 0 (< 1) with a non-zero knot count.
        let traj = SceneTrajectory::new(10, 0).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert!(report.rejected.is_some(), "a sample count < 1 must be rejected");
        assert!(!report.collided, "a rejected request performs no checking");
        assert!(report.first.is_none());
        assert!(report.all.is_empty());
    }

    #[test]
    fn sample_count_above_total_knots_is_rejected_with_no_checking() {
        let mut scene = build_single_robot_scene(report_all_config(1));
        // sample_count = 6 > total_knots = 5.
        let traj = SceneTrajectory::new(5, 6).with_path("r0", checkable_path());

        let report = scene.check_trajectory(&traj);

        assert!(
            report.rejected.is_some(),
            "a sample count greater than the total knot count must be rejected"
        );
        assert!(!report.collided, "a rejected request performs no checking");
        assert!(report.all.is_empty());
    }

    #[test]
    fn sample_count_at_valid_bounds_is_accepted() {
        // Both the lower (1) and upper (== total_knots) inclusive bounds are
        // accepted: the request is not rejected and checking proceeds.
        for (total, n) in [(5usize, 1usize), (5, 5)] {
            let mut scene = build_single_robot_scene(report_all_config(1));
            let traj = SceneTrajectory::new(total, n).with_path("r0", checkable_path());

            let report = scene.check_trajectory(&traj);

            assert!(
                report.rejected.is_none(),
                "sample_count {n} within [1, {total}] must be accepted"
            );
            assert!(report.collided, "the coincident fixture collides once checked");
        }
    }

    /* --- All paths `None` => no checkable robots, not colliding (Req 6.9) - */

    #[test]
    fn all_none_paths_report_no_checkable_robots_and_no_collision() {
        let mut scene = build_two_robot_scene(report_all_config(1));
        let traj = SceneTrajectory::new(4, 4)
            .with_path("r0", JointPath::None)
            .with_path("r1", JointPath::None);

        let report = scene.check_trajectory(&traj);

        assert!(!report.collided, "no checkable robots => the trajectory does not collide");
        assert!(report.rejected.is_none(), "an all-`None` trajectory is not a rejected request");
        assert!(report.first.is_none());
        assert!(report.all.is_empty());
        assert_eq!(
            report.unchecked_robots,
            vec!["r0".to_string(), "r1".to_string()],
            "every robot must be recorded as unchecked when all paths are `None`"
        );
    }
}

/* ========================================================================= */
/* Trajectory soundness property test (task 8.3)                             */
/*                                                                           */
/* Property 8 (design "Correctness Properties"): if `check_trajectory`        */
/* reports `collided = false` with `waypoint_stride = 1`, then no sampled     */
/* per-takt knot is in collision. `check_trajectory` samples a synchronized   */
/* progress `s = k / N` for `k = 0..=N` and delegates each sample to          */
/* `check_configuration`; discrete checking therefore reports a collision     */
/* exactly when one of those sampled configurations does.                     */
/*                                                                           */
/* This module verifies that equivalence directly: for random per-robot       */
/* `JointPath`s it runs `check_trajectory` (stride 1) and, independently,     */
/* replays the identical sampling grid by calling `check_configuration` at    */
/* every `s = k / N` (building each `SceneConfig` exactly as                   */
/* `check_trajectory` does — a robot is included only when its path samples   */
/* to `Some`). The trajectory-level `collided` flag must equal "some          */
/* independently-checked sample is in collision", which is precisely          */
/* Property 8 (its `collided = false` direction) together with its converse.  */
/*                                                                           */
/* Scene design: a two-robot coincident-box scene self-collides at every      */
/* checkable configuration, so `collided` is true whenever at least one robot */
/* is checkable and false when both paths are `None`. Random `None`/          */
/* `Endpoints`/`Dense` paths therefore exercise both the colliding and the    */
/* not-colliding (`collided = false`) branches, and the equivalence — being   */
/* structural — holds regardless of the concrete collision distances (robust  */
/* to the cross-model AABB proxy, matching the approach in `scene_prop_tests`)*/
/*                                                                           */
/* Self-contained: this module defines its own primitive-<box> fixture and    */
/* helpers. The whole file is behind `#[cfg(feature = "collision")]`, so it    */
/* runs under `cargo test --features collision`.                              */
/*                                                                           */
/* Validates: Requirements 6.1, 6.6.                                          */
/* ========================================================================= */

#[cfg(test)]
mod trajectory_prop_tests {
    use super::*;
    use crate::path::JointPath;
    use proptest::prelude::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A coincident-box 6-DOF revolute robot: seven links each carrying a
    /// coincident 0.3 m collision box with every joint origin at the parent
    /// frame origin. It self-collides at every configuration, so a checkable
    /// robot is in collision at every sampled waypoint. Kept private to this
    /// module.
    const COINCIDENT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="trajprop_selfcollide6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.3 0.3 0.3"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Contact margin large enough that a coincident (fully overlapping) pair is
    /// always reported as colliding.
    const COLLIDE_MARGIN_M: f64 = 0.05;

    /// Monotonic counter so temp fixtures never collide across parallel tests.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path =
            std::env::temp_dir().join(format!("cell_runner_collision_trajprop_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp URDF fixture");
        path
    }

    /// Build a two-robot coincident scene (`r0` at the origin, `r1` translated
    /// +0.2 m in x) with `report_all = false` and stride 1. The temp URDF is
    /// removed after the checker loads it.
    fn build_scene() -> SceneCollisionChecker {
        let config = CollisionConfig {
            contact_margin_m: COLLIDE_MARGIN_M,
            waypoint_stride: 1,
            report_all: false,
            ..CollisionConfig::default()
        };
        let urdf = write_temp_urdf(COINCIDENT_URDF);
        let specs = [
            RobotCollisionSpec::new("r0", urdf.clone(), None, Isometry3::identity()),
            RobotCollisionSpec::new("r1", urdf.clone(), None, Isometry3::translation(0.2, 0.0, 0.0)),
        ];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("two-robot coincident scene builds from a self-contained fixture")
    }

    /// A finite 6-DOF joint vector with each value in `[-pi, pi]`.
    fn joints_strategy() -> impl Strategy<Value = [f64; 6]> {
        let pi = std::f64::consts::PI;
        prop::array::uniform6(-pi..pi)
    }

    /// A random `JointPath`: `None`, `Endpoints`, or a non-empty `Dense`
    /// (1..=4 finite knots). The `Dense`/`Endpoints` variants always sample to
    /// `Some`, so a robot is checkable iff its path is not `None`.
    fn path_strategy() -> impl Strategy<Value = JointPath> {
        prop_oneof![
            Just(JointPath::None),
            (joints_strategy(), joints_strategy())
                .prop_map(|(start, end)| JointPath::Endpoints { start, end }),
            prop::collection::vec(joints_strategy(), 1..=4)
                .prop_map(|qs| JointPath::Dense { qs, takt_s: 0.012 }),
        ]
    }

    /// Build the synchronized `SceneConfig` at progress `s` exactly as
    /// `check_trajectory` does: a robot is included only when its path samples
    /// to `Some(q)` at `s`.
    fn scene_config_at(p0: &JointPath, p1: &JointPath, s: f64) -> SceneConfig {
        let mut cfg = SceneConfig::new();
        if let Some(q) = p0.sample(s) {
            cfg.insert("r0", q);
        }
        if let Some(q) = p1.sample(s) {
            cfg.insert("r1", q);
        }
        cfg
    }

    proptest! {
        // Each case parses a URDF and runs Coal queries over the whole sampling
        // grid twice (trajectory + independent replay), so cap the case count to
        // keep the suite responsive.
        #![proptest_config(ProptestConfig::with_cases(24))]

        /// Property 8: Trajectory soundness.
        ///
        /// With `waypoint_stride = 1`, the discrete trajectory check reports a
        /// collision if and only if at least one of the sampled per-takt
        /// configurations `s = k / N (k = 0..=N)` is independently in collision.
        /// In particular, when `check_trajectory` reports `collided = false`, no
        /// sampled knot is in collision (Req 6.1, 6.6).
        ///
        /// Validates: Requirements 6.1, 6.6.
        #[test]
        fn prop_trajectory_soundness_stride_one(
            p0 in path_strategy(),
            p1 in path_strategy(),
            total in 1usize..=10,
            frac in 0.0f64..1.0,
        ) {
            // sample_count N in [1, total_knots] (Req 6.8 valid range).
            let n = 1 + (frac * total as f64).floor() as usize;
            let n = n.min(total).max(1);

            let mut scene = build_scene();
            let traj = SceneTrajectory::new(total, n)
                .with_path("r0", p0.clone())
                .with_path("r1", p1.clone());

            let traj_report = scene.check_trajectory(&traj);

            // Independently replay the identical stride-1 sampling grid and
            // check each configuration on its own.
            let mut any_sample_in_collision = false;
            for k in 0..=n {
                let s = k as f64 / n as f64;
                let cfg = scene_config_at(&p0, &p1, s);
                if scene.check_configuration(&cfg).in_collision {
                    any_sample_in_collision = true;
                }
            }

            // Soundness (both directions): the trajectory collides exactly when
            // some sampled knot collides. The `collided = false => no sampled
            // knot collides` direction is Property 8.
            prop_assert_eq!(
                traj_report.collided,
                any_sample_in_collision,
                "trajectory `collided` must match the per-sample collision check \
                 over the identical stride-1 grid"
            );

            // Explicit Property 8 restatement for clarity.
            if !traj_report.collided {
                prop_assert!(
                    !any_sample_in_collision,
                    "collided = false must imply no sampled knot is in collision"
                );
            }
        }
    }
}

/* ========================================================================= */
/* Integration tests: two-robot scene (task 9.3)                             */
/*                                                                           */
/* End-to-end coverage for `SceneCollisionChecker::build` +                  */
/* `check_configuration` + `check_trajectory` over a self-contained          */
/* two-robot scene built from a primitive-<box> 6-DOF URDF written to a temp */
/* file. This module is intentionally independent of every other test module */
/* in this file: it defines its own fixture, its own temp-file helpers, and  */
/* its own counter, and touches none of the existing modules.                */
/*                                                                           */
/* Cross-model distance limitation (design "Cross-model distance             */
/* limitation"): the C ABI exposes an exact convex-hull distance query only  */
/* *within* one loaded model. Robot-robot separations therefore use the      */
/* signed world-AABB proxy (`aabb_signed_distance`) over geometry-object     */
/* POINT placements (zero half-extents, see `check_configuration`). A         */
/* robot-robot collision is thus reported when two robots' composed          */
/* collision-object origins coincide (or lie within a positive contact       */
/* margin). The fixture below is engineered around that proxy:               */
/*                                                                           */
/*   * base_link and link2 carry collision boxes; link2's world origin is    */
/*     `base * Rz(q1) * (0.5, 0, 0)`, so joint 1 sweeps link2's origin on a  */
/*     circle of radius 0.5 m about the robot base.                          */
/*   * With both robots at the identity base and identical joints, every     */
/*     geometry-object origin of r0 coincides with r1's, yielding RobotRobot */
/*     pairs at distance ~0 (Req 3.3).                                        */
/*   * For the trajectory test, r0 sits still at the origin while r1's base  */
/*     is translated +1.0 m in x and its joint 1 sweeps 0 -> 2*pi. r1/link2  */
/*     reaches r0/link2 = (0.5, 0, 0) exactly at q1 = pi, i.e. progress      */
/*     s = 0.5. Every other cross pair stays >= 0.5 m apart, so the only     */
/*     colliding waypoint is the synchronized sample at s = 0.5.             */
/*                                                                           */
/* The whole file is behind `#[cfg(feature = "collision")]` (see `main.rs`), */
/* so these run under `cargo test --features collision`.                      */
/*                                                                           */
/* Validates: Requirements 3.3, 11.2, 11.3, 11.4.                            */
/* ========================================================================= */

#[cfg(test)]
mod integration_two_robot {
    use super::*;
    use crate::path::JointPath;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A self-contained primitive-<box> 6-DOF revolute robot.
    ///
    /// `base_link` and `link2` carry a 0.2 m collision box; the intermediate
    /// and trailing links are collision-free kinematic links so the model
    /// exposes exactly two collision objects. Joint 2 offsets `link2` by
    /// `(0.5, 0, 0)` from `link1`, and joint 1 (revolute about z) rotates that
    /// offset, so `link2`'s base-frame origin traces a circle of radius 0.5 m
    /// as `q1` varies while `base_link` stays at the base origin. Joint limits
    /// are widened to +/- 2*pi so a full `q1` sweep is representable.
    const TWO_ROBOT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="integration_arm6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Monotonic counter so temp fixtures never collide across parallel tests.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_integration_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp two-robot URDF fixture");
        path
    }

    /// Build a two-robot scene from `TWO_ROBOT_URDF` with the given base
    /// transforms and config. Both robots share the same URDF (so a given
    /// geometry-object index maps to the same link on both). The temp URDF is
    /// removed after the checker has loaded it.
    fn build_two_robot_scene(
        base_r0: Isometry3<f64>,
        base_r1: Isometry3<f64>,
        config: CollisionConfig,
    ) -> SceneCollisionChecker {
        let urdf = write_temp_urdf(TWO_ROBOT_URDF);
        let specs = [
            RobotCollisionSpec::new("r0", urdf.clone(), None, base_r0),
            RobotCollisionSpec::new("r1", urdf.clone(), None, base_r1),
        ];
        let checker = SceneCollisionChecker::build(&specs, &[], config);
        let _ = std::fs::remove_file(&urdf);
        checker.expect("two-robot scene builds from a self-contained primitive-box fixture")
    }

    /// Assert a reported pair carries the MCP-facing invariants of Req 11.3:
    /// both names are fully qualified as `<robot_id>/<link_or_geom>` (a single
    /// `/` with non-empty parts) and the reported distance is finite.
    fn assert_pair_well_formed(pair: &CollisionPair) {
        for name in [pair.object_a.as_str(), pair.object_b.as_str()] {
            let (robot_id, link) = name
                .split_once('/')
                .unwrap_or_else(|| panic!("object name `{name}` must be `<robot_id>/<link>`"));
            assert!(
                !robot_id.is_empty() && !link.is_empty(),
                "both components of `{name}` must be non-empty"
            );
            assert!(
                !link.contains('/'),
                "object name `{name}` must contain exactly one `/` separator"
            );
        }
        assert!(
            pair.distance_m.is_finite(),
            "reported distance {} must be finite",
            pair.distance_m
        );
    }

    /// Req 3.3: two robots at coincident base transforms and an identical joint
    /// configuration place their composed collision-object origins on top of
    /// one another, so the scene check reports at least one `RobotRobot` pair.
    ///
    /// Self- and environment-collision are disabled so the report isolates the
    /// robot-robot class; under that config every reported pair must be
    /// `RobotRobot`, must be fully qualified, and must carry a finite distance.
    #[test]
    fn two_robot_scene_reports_robot_robot_pair() {
        // Small positive contact margin: the coincident origins separate by
        // exactly 0 m, which a zero margin (penetration-only) would not report,
        // so a positive margin is required to flag the contact (Req 7.2).
        let config = CollisionConfig {
            contact_margin_m: 0.01,
            report_all: true,
            check_self: false,
            check_robot_robot: true,
            check_environment: false,
            ..CollisionConfig::default()
        };
        // Both robots at the identity base with identical (all-zero) joints:
        // every geometry-object origin of r0 coincides with r1's.
        let mut scene =
            build_two_robot_scene(Isometry3::identity(), Isometry3::identity(), config);
        let joints = SceneConfig::new()
            .with_robot("r0", [0.0; 6])
            .with_robot("r1", [0.0; 6]);

        let report = scene.check_configuration(&joints);

        assert!(
            report.in_collision,
            "coincident two-robot bases at identical joints must be in collision"
        );
        assert!(
            !report.pairs.is_empty(),
            "expected at least one reported robot-robot pair"
        );

        // Every reported pair is RobotRobot (self/env disabled), is well-formed,
        // and spans one object from each robot.
        let mut saw_robot_robot = false;
        for pair in &report.pairs {
            assert_eq!(
                pair.kind,
                CollisionKind::RobotRobot,
                "with self/env checks disabled, every pair must be RobotRobot: {pair:?}"
            );
            assert_pair_well_formed(pair);
            let a_robot = pair.object_a.split_once('/').unwrap().0;
            let b_robot = pair.object_b.split_once('/').unwrap().0;
            assert!(
                (a_robot == "r0" && b_robot == "r1") || (a_robot == "r1" && b_robot == "r0"),
                "a RobotRobot pair must span r0 and r1, got {a_robot} <-> {b_robot}"
            );
            saw_robot_robot = true;
        }
        assert!(saw_robot_robot, "at least one RobotRobot pair must be reported (Req 3.3)");
    }

    /// Req 11.2, 11.3, 11.4: driving a synchronized trajectory that brings the
    /// two robots into overlap at a known progress reports a collision at the
    /// expected first waypoint, and every reported collision carries the
    /// MCP-facing invariants (fully-qualified names, finite distance, progress
    /// in `[0, 1]`).
    ///
    /// r0 stays still at the origin (`r0/link2` at `(0.5, 0, 0)`). r1's base is
    /// translated +1.0 m in x and its joint 1 sweeps `0 -> 2*pi`, so `r1/link2`
    /// = `(1.0 + 0.5*cos q1, 0.5*sin q1, 0)` sweeps to `(0.5, 0, 0)` exactly at
    /// `q1 = pi`, i.e. progress `s = 0.5`. With 10 synchronized samples that
    /// lands on waypoint `k = 5`; every other cross pair stays >= 0.5 m apart,
    /// well outside the 0.01 m margin, so `s = 0.5` is the sole colliding
    /// waypoint.
    #[test]
    fn trajectory_into_overlap_collides_at_expected_waypoint() {
        let config = CollisionConfig {
            contact_margin_m: 0.01,
            waypoint_stride: 1,
            report_all: true,
            check_self: false,
            check_robot_robot: true,
            check_environment: false,
        };
        let mut scene = build_two_robot_scene(
            Isometry3::identity(),
            Isometry3::translation(1.0, 0.0, 0.0),
            config,
        );

        // r0 holds still; r1 sweeps joint 1 from 0 to 2*pi across the path.
        let two_pi = std::f64::consts::TAU;
        let n: usize = 10;
        let trajectory = SceneTrajectory::new(n, n)
            .with_path(
                "r0",
                JointPath::Endpoints {
                    start: [0.0; 6],
                    end: [0.0; 6],
                },
            )
            .with_path(
                "r1",
                JointPath::Endpoints {
                    start: [0.0; 6],
                    end: [two_pi, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
            );

        let report = scene.check_trajectory(&trajectory);

        assert!(
            report.rejected.is_none(),
            "sample count 10 within [1, 10] must not be rejected: {:?}",
            report.rejected
        );
        assert!(
            report.collided,
            "the trajectory drives r1 into r0 at progress 0.5 and must collide"
        );

        // Req 6.3: the first colliding waypoint is the earliest by progress.
        let first = report
            .first
            .as_ref()
            .expect("a collided trajectory must report a first waypoint");
        assert_eq!(
            first.waypoint, 5,
            "the sole overlap occurs at the synchronized sample k = 5 (progress 0.5)"
        );
        assert!(
            (first.progress - 0.5).abs() < 1e-9,
            "expected first collision at progress 0.5, got {}",
            first.progress
        );

        // Overlap is engineered to occur at exactly one synchronized sample.
        assert_eq!(
            report.all.len(),
            1,
            "exactly one waypoint (s = 0.5) should collide, got {:?}",
            report.all.iter().map(|w| w.progress).collect::<Vec<_>>()
        );

        // Req 11.3 / 11.4: the first waypoint's report is a non-empty set of
        // well-formed RobotRobot collisions with progress in [0, 1].
        assert!(
            first.report.in_collision && !first.report.pairs.is_empty(),
            "the colliding waypoint's report must be in_collision with pairs"
        );
        assert!(
            (0.0..=1.0).contains(&first.progress),
            "progress {} must lie in [0, 1]",
            first.progress
        );
        for pair in &first.report.pairs {
            assert_eq!(
                pair.kind,
                CollisionKind::RobotRobot,
                "trajectory overlap is a robot-robot contact: {pair:?}"
            );
            assert_pair_well_formed(pair);
        }
    }
}

/* ========================================================================= */
/* Task 10.1: self-contained `.glb` fixture + scene-YAML integration test    */
/*                                                                           */
/* Builds a minimal valid glTF-binary (`.glb`) unit cube at *test time*     */
/* (`build_unit_cube_glb`) rather than checking in a binary fixture file:    */
/* a `.glb` is just a documented binary container (12-byte header + JSON    */
/* chunk + BIN chunk), so the exact bytes for a single 8-vertex/12-triangle */
/* cube mesh can be assembled directly in Rust with `serde_json` (already a */
/* crate dependency) for the JSON chunk. This keeps the fixture in the test */
/* source itself: no new binary blob enters version control, and the mesh   */
/* data driving the assertions is visible right next to the assertions.     */
/*                                                                           */
/* The accompanying scene YAML is likewise constructed as an in-test string */
/* (mirroring `SMOKE_SELF_COLLIDING_URDF` / `TWO_ROBOT_URDF` above) and      */
/* written to a temp file, exercising the real `export::load_static_models` */
/* parser end-to-end rather than constructing `EnvironmentCollisionSpec`s by */
/* hand.                                                                     */
/* ========================================================================= */

#[cfg(test)]
mod glb_environment_integration {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// Monotonic counter so temp fixtures never collide across parallel tests.
    static FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// A minimal, self-contained one-collision-object 6-DOF revolute robot:
    /// only `base_link` carries a 0.4 m collision box (half-extents 0.2 m on
    /// every axis); `link1`..`link6` are collision-free kinematic links so the
    /// model registers exactly one collision geometry object (`geom0`,
    /// qualified as `<robot_id>/geom0`).
    const ONE_LINK_URDF: &str = r#"<?xml version="1.0"?>
<robot name="env_integration_stub6dof">
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.4 0.4 0.4"/></geometry></collision>
  </link>
  <link name="link1">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link2">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link3">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link4">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link5">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="link6">
    <inertial><mass value="1.0"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.3" upper="6.3" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

    /// Write `contents` to a unique temp `.urdf` path and return that path.
    fn write_temp_urdf(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_env_fixture_{pid}_{n}.urdf"));
        std::fs::write(&path, contents).expect("write temp one-link URDF fixture");
        path
    }

    /// Write `contents` to a unique temp `.yaml` scene path and return that
    /// path. The scene lives in the same directory as the `.glb` fixture it
    /// references (via a bare filename), matching `load_static_models`'s
    /// `file` resolution against `scene.parent()`.
    fn write_temp_scene_yaml(contents: &str) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_env_fixture_{pid}_{n}.yaml"));
        std::fs::write(&path, contents).expect("write temp scene YAML fixture");
        path
    }

    /// Write raw `bytes` to a unique temp `.glb` path and return that path.
    fn write_temp_glb(bytes: &[u8]) -> PathBuf {
        let n = FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let path = std::env::temp_dir()
            .join(format!("cell_runner_collision_env_fixture_{pid}_{n}.glb"));
        std::fs::write(&path, bytes).expect("write temp .glb fixture");
        path
    }

    /// Build the raw bytes of a minimal valid glTF-binary (`.glb`) file
    /// containing a single unit-cube mesh: 8 vertices at `+/-0.5` on every
    /// axis (mesh-local half-extents `(0.5, 0.5, 0.5)`) and 12 triangles (2
    /// per face, 6 faces), referenced by index.
    ///
    /// Assembled by hand per the documented glTF-binary container format
    /// (12-byte header + a 4-byte-aligned `JSON` chunk + a 4-byte-aligned
    /// `BIN` chunk holding the raw little-endian vertex/index bytes) rather
    /// than checked in as a binary blob, so the fixture never needs an
    /// external authoring tool and stays reviewable as Rust source.
    fn build_unit_cube_glb() -> Vec<u8> {
        const H: f32 = 0.5;
        let positions: [[f32; 3]; 8] = [
            [-H, -H, -H],
            [H, -H, -H],
            [H, H, -H],
            [-H, H, -H],
            [-H, -H, H],
            [H, -H, H],
            [H, H, H],
            [-H, H, H],
        ];
        // 12 triangles (2 per face x 6 faces); winding is not load-relevant
        // here (the loader only reads `POSITION`), but is kept consistent for
        // a well-formed, plausible mesh.
        let indices: [u16; 36] = [
            0, 1, 2, 0, 2, 3, // bottom (z = -H)
            4, 6, 5, 4, 7, 6, // top (z = +H)
            0, 5, 1, 0, 4, 5, // front (y = -H)
            3, 2, 6, 3, 6, 7, // back (y = +H)
            0, 3, 7, 0, 7, 4, // left (x = -H)
            1, 5, 6, 1, 6, 2, // right (x = +H)
        ];

        let mut bin: Vec<u8> = Vec::new();
        for p in &positions {
            for c in p {
                bin.extend_from_slice(&c.to_le_bytes());
            }
        }
        let indices_byte_offset = bin.len();
        for i in &indices {
            bin.extend_from_slice(&i.to_le_bytes());
        }
        // glTF-binary requires every chunk length to be a multiple of 4; the
        // BIN chunk is padded with zero bytes (positions + indices already
        // total 168 bytes here, but pad defensively for robustness).
        while bin.len() % 4 != 0 {
            bin.push(0);
        }

        let json = serde_json::json!({
            "asset": {"version": "2.0"},
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [{"mesh": 0}],
            "meshes": [{
                "primitives": [{
                    "attributes": {"POSITION": 0},
                    "indices": 1,
                    "mode": 4
                }]
            }],
            "accessors": [
                {
                    "bufferView": 0,
                    "byteOffset": 0,
                    "componentType": 5126,
                    "count": 8,
                    "type": "VEC3",
                    "min": [-H, -H, -H],
                    "max": [H, H, H]
                },
                {
                    "bufferView": 1,
                    "byteOffset": 0,
                    "componentType": 5123,
                    "count": indices.len(),
                    "type": "SCALAR"
                }
            ],
            "bufferViews": [
                {
                    "buffer": 0,
                    "byteOffset": 0,
                    "byteLength": indices_byte_offset,
                    "target": 34962
                },
                {
                    "buffer": 0,
                    "byteOffset": indices_byte_offset,
                    "byteLength": indices.len() * 2,
                    "target": 34963
                }
            ],
            "buffers": [{"byteLength": bin.len()}]
        });
        let mut json_bytes = serde_json::to_vec(&json).expect("serialize glb JSON chunk");
        // Pad the JSON chunk to a 4-byte boundary with ASCII spaces, per the
        // glTF-binary container spec.
        while json_bytes.len() % 4 != 0 {
            json_bytes.push(0x20);
        }

        let total_len = 12 + 8 + json_bytes.len() + 8 + bin.len();

        let mut glb = Vec::with_capacity(total_len);
        glb.extend_from_slice(b"glTF");
        glb.extend_from_slice(&2u32.to_le_bytes());
        glb.extend_from_slice(&(total_len as u32).to_le_bytes());

        glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
        glb.extend_from_slice(b"JSON");
        glb.extend_from_slice(&json_bytes);

        glb.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        glb.extend_from_slice(b"BIN\0");
        glb.extend_from_slice(&bin);

        glb
    }

    /// Req 6.1, 6.2, 10.1: a scene whose `static_models:` places the
    /// self-contained unit-cube `.glb` fixture at world `(2, 0, 0)`, and whose
    /// single robot's collision box is based at the same world point, must
    /// report a `CollisionKind::RobotEnvironment` pair between
    /// `environment/cube` and `<robot_id>/geom0`.
    #[test]
    fn robot_overlapping_environment_cube_reports_robot_environment_pair() {
        let glb_path = write_temp_glb(&build_unit_cube_glb());
        let glb_filename = glb_path
            .file_name()
            .and_then(|f| f.to_str())
            .expect("temp .glb path has a UTF-8 file name")
            .to_string();

        // The cube is placed at world (2, 0, 0) with no orientation (defaults
        // to identity rotation). Its mesh-local half-extents are (0.5, 0.5,
        // 0.5), so its world AABB spans (1.5, -0.5, -0.5) to (2.5, 0.5, 0.5).
        let scene_yaml = format!(
            "static_models:\n  - id: cube\n    file: {glb_filename}\n    position: {{x: 2.0, y: 0.0, z: 0.0}}\n"
        );
        let scene_path = write_temp_scene_yaml(&scene_yaml);

        let env_specs = crate::export::load_static_models(Some(&scene_path))
            .expect("scene YAML with one valid static_models entry parses successfully");
        assert_eq!(
            env_specs.len(),
            1,
            "expected exactly one parsed environment spec from the scene fixture"
        );

        let urdf_path = write_temp_urdf(ONE_LINK_URDF);
        // The robot's base is also at world (2, 0, 0): with an all-zero joint
        // configuration its sole collision box (half-extents (0.2, 0.2, 0.2))
        // is centered on the very same point, so its world AABB (1.8, -0.2,
        // -0.2)-(2.2, 0.2, 0.2) is nested entirely inside the cube's,
        // guaranteeing overlap regardless of exact half-extent rounding.
        let robot_specs = [RobotCollisionSpec::new(
            "stub",
            urdf_path.clone(),
            None,
            Isometry3::translation(2.0, 0.0, 0.0),
        )];

        let config = CollisionConfig {
            contact_margin_m: 0.0,
            report_all: true,
            check_self: false,
            check_robot_robot: false,
            check_environment: true,
            ..CollisionConfig::default()
        };
        let mut scene = SceneCollisionChecker::build(&robot_specs, &env_specs, config)
            .expect("scene builds from the one-link URDF fixture and the parsed cube spec");

        let _ = std::fs::remove_file(&urdf_path);
        let _ = std::fs::remove_file(&scene_path);
        let _ = std::fs::remove_file(&glb_path);

        assert_eq!(
            scene.environment_object_count(),
            1,
            "the cube fixture must load into exactly one placed environment object"
        );
        assert!(
            scene.environment_errors().is_empty(),
            "the cube fixture must load without error: {:?}",
            scene.environment_errors()
        );

        let joints = SceneConfig::new().with_robot("stub", [0.0; 6]);
        let report = scene.check_configuration(&joints);

        assert!(
            report.in_collision,
            "the robot's box nested inside the placed cube must be reported in collision"
        );
        assert_eq!(
            report.pairs.len(),
            1,
            "with self/robot-robot checks disabled, exactly one pair (robot vs. environment) \
             is expected, got {:?}",
            report.pairs
        );
        let pair = &report.pairs[0];
        assert_eq!(
            pair.kind,
            CollisionKind::RobotEnvironment,
            "the sole reported pair must be RobotEnvironment: {pair:?}"
        );
        assert_eq!(pair.object_a, "environment/cube");
        assert_eq!(pair.object_b, "stub/geom0");
        assert!(
            pair.distance_m <= 0.0,
            "a nested box must be reported as penetrating (distance <= 0): {}",
            pair.distance_m
        );
    }

    /// Req 10.1, 10.2: backward compatibility. A checker built with an empty
    /// `env_specs` slice reports zero `RobotEnvironment` pairs even when a
    /// robot is present and would otherwise overlap a (now-absent) static
    /// model at the same world point.
    #[test]
    fn empty_static_models_yields_no_environment_pairs() {
        let urdf_path = write_temp_urdf(ONE_LINK_URDF);
        let robot_specs = [RobotCollisionSpec::new(
            "stub",
            urdf_path.clone(),
            None,
            Isometry3::translation(2.0, 0.0, 0.0),
        )];

        let config = CollisionConfig {
            contact_margin_m: 0.0,
            report_all: true,
            check_self: false,
            check_robot_robot: false,
            check_environment: true,
            ..CollisionConfig::default()
        };
        let mut scene = SceneCollisionChecker::build(&robot_specs, &[], config)
            .expect("scene builds from the one-link URDF fixture with no environment specs");

        let _ = std::fs::remove_file(&urdf_path);

        assert!(
            !scene.has_environment(),
            "an empty env_specs slice must yield no placed environment objects"
        );
        assert_eq!(scene.environment_object_count(), 0);

        let joints = SceneConfig::new().with_robot("stub", [0.0; 6]);
        let report = scene.check_configuration(&joints);

        assert!(
            !report.in_collision,
            "with no static models, the lone robot cannot be in collision"
        );
        assert!(
            report.pairs.is_empty(),
            "an empty environment must produce zero pairs of any kind, got {:?}",
            report.pairs
        );
        assert!(
            !report
                .pairs
                .iter()
                .any(|p| p.kind == CollisionKind::RobotEnvironment),
            "an empty environment must produce zero RobotEnvironment pairs specifically"
        );
    }
}
