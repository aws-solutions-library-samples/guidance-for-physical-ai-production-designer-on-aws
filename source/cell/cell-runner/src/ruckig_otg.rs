//! Ruckig online-trajectory-generator (OTG) + Pinocchio inverse-kinematics
//! pipeline, clocked on the fixed 12 ms KRC4 takt.
//!
//! This module owns the [`RuckigPlanner`], which holds the shared
//! [`crate::pinocchio::PinocchioModel`] (for IK/FK) and the per-model dynamic
//! [`JointLimits`]. The per-motion Ruckig instances themselves are created on
//! demand inside `resolve_ptp` / `resolve_lin` (authored in tasks 3.2 / 3.3):
//! 6 DOF for PTP (joint axes), 3 DOF for LIN (Cartesian translation).
//!
//! # Ownership model (deviation from the design doc)
//!
//! The design sketches `pinocchio: Rc<PinocchioModel>`, but
//! [`crate::pinocchio::PinocchioModel::inverse_kinematics`] /
//! [`forward_kinematics`](crate::pinocchio::PinocchioModel::forward_kinematics)
//! both take `&mut self`, which an `Rc` cannot hand out without interior
//! mutability. `RuckigPlanner` therefore **owns the `PinocchioModel` by value**
//! and the IK/resolve methods take `&mut self`. The `LayeredTrajectoryProvider`
//! (task 5.2) owns the planner.
//!
//! # rsruckig 2.1.3 API notes for tasks 3.2 / 3.3
//!
//! The dependency is the pure-Rust [`rsruckig`] crate (v2.1.3), **not** the C++
//! `ruckig` crate the design pseudocode references. Its surface (verified
//! against the crate source) is:
//!
//! ```ignore
//! use rsruckig::prelude::*;
//!
//! // DOF is a const generic; ThrowErrorHandler returns Err on validation/calc
//! // failure. Stack allocation: Ruckig::<DOF, _>::new(None, cycle_s).
//! // Heap allocation:           Ruckig::<0,  _>::new(Some(dof), cycle_s).
//! let mut otg = Ruckig::<6, ThrowErrorHandler>::new(None, KRC4_TIMESTEP);
//! let mut input = InputParameter::<6>::new(None);
//! let mut output = OutputParameter::<6>::new(None);
//!
//! // Fields are `DataArrayOrVec<f64, DOF>` and index like arrays:
//! //   input.current_position / current_velocity / current_acceleration
//! //   input.target_position  (target_velocity / target_acceleration default 0)
//! //   input.max_velocity / max_acceleration / max_jerk   (jerk MUST be non-empty)
//! // Output:
//! //   output.new_position / new_velocity / new_acceleration / trajectory
//!
//! while otg.update(&input, &mut output)? == RuckigResult::Working {
//!     // ... push output.new_position as the takt sample ...
//!     output.pass_to_input(&mut input); // MANDATORY S-curve State_Transfer
//! }
//! // RuckigResult variants: Working | Finished | Error (target reached on Finished).
//! ```
//!
//! Key points for the OTG loop:
//! * `otg.update(&input, &mut output)` returns `Result<RuckigResult, RuckigError>`.
//! * `output.pass_to_input(&mut input)` performs the mandatory state transfer
//!   (copies `new_position/velocity/acceleration` into `current_*`), satisfying
//!   Requirement 2.2. Doing it manually field-by-field is equivalent.
//! * Non-empty `max_jerk` yields the jerk-limited S-curve (Requirement 2.3);
//!   [`RuckigPlanner::new`] guarantees the jerk band is never empty/zero.
#![allow(dead_code)]

use rsruckig::prelude::{
    InputParameter, OutputParameter, Ruckig, RuckigResult, ThrowErrorHandler,
};

use crate::limits::{JointLimits, KR240_J_ABS};
use crate::path::{position_within_limits, JointPath};
use crate::traj::{MotionKind, MotionRequest, SolverTier, TrajectorySolution};

/// Fixed KRC4 control cycle (seconds). The Ruckig OTG loop emits exactly one
/// trajectory sample per takt, and a motion's duration is
/// `number_of_takts · KRC4_TIMESTEP` (Requirement 2.5).
pub const KRC4_TIMESTEP: f64 = 0.012;

/// Absolute safety bound on the number of takts a single OTG loop may run before
/// it is declared "not reached" (Requirement 2.6). The per-motion bound is
/// `clamp(ceil(est_duration / KRC4_TIMESTEP), 2, MAX_TAKTS)` (tasks 3.2 / 3.3);
/// this constant is the hard ceiling of that clamp.
pub const MAX_TAKTS: usize = 5000;

/// Lower edge of the physical jerk band (rad/s³). Jerk bounds below this are
/// raised to it so the produced profile stays a jerk-limited S-curve.
pub const JERK_BAND_MIN: f64 = 20.0;

/// Upper edge of the physical jerk band (rad/s³). Jerk bounds above this are
/// clamped to it.
pub const JERK_BAND_MAX: f64 = 150.0;

/// Maximum inverse-kinematics iterations for goal/seed pose solves, matching the
/// base system's motion-endpoint IK calls (`server.rs` / `compute_ptp_duration_ik`).
const IK_MAX_ITERS: u32 = 200;

/// Inverse-kinematics convergence tolerance, matching the base system's
/// motion-endpoint IK calls.
const IK_TOLERANCE: f64 = 1e-3;

/// Millimeters per meter — KUKA poses carry translation in **mm**, but the
/// Cartesian Ruckig instance runs in **m** (Requirement 8.1 keeps the pose
/// units; the conversion is purely internal to `resolve_lin`).
const MM_PER_M: f64 = 1000.0;

/// Default Cartesian translational velocity ceiling (m/s) standing in for KUKA's
/// `$VEL.CP`. There is no per-model Cartesian limit in [`JointLimits`] (which is
/// joint-space only), so this module-level constant is the LIN analog of the
/// joint-space `v_abs`. It is scaled by the commanded velocity fraction
/// (`vel_percent / 100`) to form the Ruckig `max_velocity` input
/// (Requirement 10.3). The 2.0 m/s default follows the `$VEL.CP` guidance in
/// `add-ruckig.md` §"Tracking $VEL.CP vs $VEL_AXIS".
pub const CART_VMAX_DEFAULT: f64 = 2.0;

/// Default Cartesian translational acceleration ceiling (m/s²) for the LIN
/// Ruckig instance, mirroring the `add-ruckig.md` LIN example
/// (`max_acceleration = [5.0, 5.0, 5.0]`).
pub const CART_AMAX_DEFAULT: f64 = 5.0;

/// Default Cartesian translational jerk ceiling (m/s³) for the LIN Ruckig
/// instance. This **must be non-empty** so Ruckig produces a jerk-limited
/// S-curve rather than a trapezoidal step-velocity profile (Requirement 2.3);
/// the 50.0 m/s³ value follows the `add-ruckig.md` LIN example
/// (`max_jerk = [50.0, 50.0, 50.0]`).
pub const CART_JMAX_DEFAULT: f64 = 50.0;

/// Translational frame tolerance (mm) for the LIN final-knot fidelity check: the
/// forward-kinematics position of the last Dense knot must land within this
/// distance of the commanded target translation, otherwise the motion is
/// reported not-reached so the layered provider downgrades (Requirement 10.5).
pub const LIN_FRAME_TOL_MM: f64 = 5.0;

/// Holds the shared Pinocchio model (for IK/FK) and the per-axis dynamic limits.
///
/// The Ruckig instances themselves are created per-motion (3 DOF for LIN, 6 DOF
/// for PTP) inside `resolve_ptp` / `resolve_lin`. The model is owned by value so
/// the `&mut self` IK/FK methods can be called (see the module-level ownership
/// note).
pub struct RuckigPlanner {
    /// Existing Pinocchio IK/FK engine, shared across all roles (per-takt LIN IK,
    /// goal-pose PTP IK, one-shot pose solves). Owned by value.
    pinocchio: crate::pinocchio::PinocchioModel,
    /// Per-model joint position/velocity/effort/acceleration/jerk limits, with
    /// the jerk band sanitized to be non-empty (see [`RuckigPlanner::new`]).
    limits: JointLimits,
}

// SAFETY: `RuckigPlanner` owns a `PinocchioModel`, which is itself `unsafe impl
// Send` (the underlying Pinocchio context is only ever touched behind `&mut`,
// one planner per robot, never shared across threads). The remaining field
// (`JointLimits`) is plain `Copy` data. The planner is therefore safe to move
// between threads.
unsafe impl Send for RuckigPlanner {}

impl RuckigPlanner {
    /// Build a planner around an already-loaded Pinocchio model and a per-model
    /// limit set.
    ///
    /// The supplied `limits.j_abs` is sanitized so the jerk bounds are never
    /// empty or zero: each axis whose jerk is non-finite or non-positive is
    /// replaced with the corresponding [`KR240_J_ABS`] fallback, and every axis
    /// is clamped into the physical `[JERK_BAND_MIN, JERK_BAND_MAX]` band
    /// (20–150 rad/s³). This guarantees Ruckig produces a jerk-limited S-curve
    /// rather than degenerating to a trapezoidal, step-velocity profile
    /// (Requirements 2.3, 5.4).
    ///
    /// # Preconditions
    /// * `pinocchio` is a loaded model with a resolved `tool0` frame.
    /// * `limits.q_lower ≤ limits.q_upper` (not re-checked here; honored by the
    ///   limit-extraction in `limits.rs`).
    ///
    /// # Invariant
    /// Shares the existing Pinocchio model rather than loading a second copy.
    pub fn new(pinocchio: crate::pinocchio::PinocchioModel, limits: JointLimits) -> Self {
        let mut limits = limits;
        limits.j_abs = sanitized_jerk_bounds(&limits.j_abs);
        Self { pinocchio, limits }
    }

    /// The per-model joint limit set (with sanitized jerk bounds).
    pub fn limits(&self) -> &JointLimits {
        &self.limits
    }

    /// Forward kinematics: joint angles (radians) -> flange pose
    /// `[x, y, z, a, b, c]` (mm, ZYX Euler degrees). Delegates to the shared
    /// Pinocchio model. Used to derive the home flange pose from home joints.
    pub fn fk(&mut self, joints: &[f64; 6]) -> [f64; 6] {
        self.pinocchio.forward_kinematics(joints)
    }

    /// One-shot pose → joints, delegating to the existing Pinocchio IK
    /// (Requirement 1.7). Used for home presolve / cold joint solving and as the
    /// seed step inside the OTG loops.
    ///
    /// Units and frame are identical to the Pinocchio path (Requirements
    /// 8.1–8.4): `pose` is `[x, y, z, a, b, c]` with positions in **mm** and
    /// orientation as **ZYX Euler A/B/C in degrees**; `seed` and the returned
    /// joints are in **radians**; kinematics are computed against the `tool0`
    /// flange frame. Iteration count and tolerance match the base system's
    /// motion-endpoint IK calls.
    ///
    /// # Postconditions
    /// `Ok(q)` is the IK solution (radians) for `pose` warm-started from `seed`;
    /// `Err(msg)` names the divergence so the caller can downgrade.
    pub fn ik(&mut self, pose: &[f64; 6], seed: &[f64; 6]) -> Result<[f64; 6], String> {
        self.pinocchio
            .inverse_kinematics(seed, pose, IK_MAX_ITERS, IK_TOLERANCE)
    }

    /// Resolve a motion to a Ruckig-OTG [`TrajectorySolution`], dispatching on
    /// [`MotionRequest::kind`] and running a final consolidated validation pass
    /// before the path is handed back (Requirements 2.4, 2.6, 4.1, 4.2, 4.3, 4.5).
    ///
    /// Dispatch branches to [`resolve_ptp`](Self::resolve_ptp) for
    /// [`MotionKind::Ptp`] and [`resolve_lin`](Self::resolve_lin) for
    /// [`MotionKind::Lin`]. Each sub-resolver already performs its own
    /// per-takt position-limit checks and goal/frame-fidelity checks (the goal
    /// is reached within tolerance inside the sub-resolver, so it is not
    /// re-checked here). This method then runs a **single consolidated
    /// validation** over the returned [`JointPath`] using the `path.rs` helpers,
    /// so a `Dense` path is accepted only when:
    ///
    /// * every sample is within the per-model position limits
    ///   ([`JointPath::within_limits`], Requirement 4.2), AND
    /// * every pair of consecutive knots is velocity-feasible at the commanded
    ///   velocity fraction over one [`KRC4_TIMESTEP`]
    ///   ([`JointPath::velocity_feasible`], Requirement 4.3), AND
    /// * the implied per-axis accelerations respect `a_abs`
    ///   ([`JointPath::acceleration_feasible`]), AND
    /// * the implied per-axis jerks respect `j_abs`
    ///   ([`JointPath::jerk_feasible`], Requirements 2.4, 4.1).
    ///
    /// The velocity fraction is `req.vel_percent / 100`, clamped to
    /// `[0.01, 1.0]` for parity with [`resolve_ptp`](Self::resolve_ptp) and
    /// `path::velocity_feasible`. For very short motions a `Dense` path may carry
    /// fewer than three (or four) knots, in which case the acceleration (or
    /// jerk) feasibility check is trivially true — that is handled in `path.rs`
    /// and is expected.
    ///
    /// # Errors
    /// Returns `Err(reason)` naming the specific failure — either propagated
    /// verbatim from the sub-resolver (IK divergence, per-takt limit violation,
    /// goal/frame miss, takt-bound exhaustion) or named by the consolidated
    /// validation pass (position / velocity / acceleration / jerk) — so the
    /// layered provider (task 5.3) downgrades to the next Solver_Tier.
    pub fn resolve(&mut self, req: &MotionRequest) -> Result<TrajectorySolution, String> {
        let solution = match req.kind {
            MotionKind::Ptp => self.resolve_ptp(req)?,
            MotionKind::Lin => self.resolve_lin(req)?,
        };

        // Velocity fraction in [0.01, 1.0] — clamped consistently with
        // resolve_ptp / resolve_lin (vel_percent is a percent; vel_frac = pct/100).
        let vel_frac = (req.vel_percent / 100.0).clamp(0.01, 1.0);
        validate_solution(&solution, &self.limits, vel_frac)?;

        Ok(solution)
    }

    /// PTP (point-to-point) joint-space motion: run a 6-DOF Ruckig instance
    /// directly on the six joint axes (Requirement 9.1).
    ///
    /// The joint goal is derived by a **single** goal-pose inverse-kinematics
    /// solve (never per takt, Requirement 9.2): `q_start` is the IK of
    /// `req.start_pose` warm-started from `req.seed_joints`, and `q_goal` is the
    /// IK of `req.target_pose` warm-started from `q_start`. The Ruckig instance
    /// is seeded at rest at `q_start` and driven to `q_goal` with per-axis
    /// `max_velocity = v_abs · vel%` (Requirements 9.3, 4.1),
    /// `max_acceleration = a_abs`, and non-empty `max_jerk = j_abs` so the
    /// profile is a jerk-limited S-curve (Requirement 2.3).
    ///
    /// The loop iterates `update()` on the fixed 12 ms takt (Requirement 2.1),
    /// pushing each joint sample directly and performing the mandatory
    /// velocity/acceleration State_Transfer via `output.pass_to_input` between
    /// takts (Requirement 2.2). It is bounded by [`MAX_TAKTS`]: exceeding the
    /// bound returns `Err` marked not-reached so the layered provider downgrades
    /// (Requirement 2.6). The returned [`JointPath::Dense`] carries one knot per
    /// takt spaced [`KRC4_TIMESTEP`] apart, its final knot equals `q_goal`
    /// within tolerance (Requirements 2.4, 9.4), and `duration_s` is the number
    /// of takts times [`KRC4_TIMESTEP`] (Requirement 2.5).
    ///
    /// # Errors
    /// Returns `Err(reason)` when the start/goal IK diverges, a produced sample
    /// violates the per-axis position limits, the Ruckig solver reports an
    /// error, or the takt bound is exhausted — in every case naming the reason
    /// so the provider can fall through to the next tier.
    pub fn resolve_ptp(&mut self, req: &MotionRequest) -> Result<TrajectorySolution, String> {
        debug_assert_eq!(req.kind, MotionKind::Ptp, "resolve_ptp called with non-PTP request");

        // Goal joints via a single goal-pose IK (never per takt, Req 9.2).
        let q_start = self
            .ik(&req.start_pose, &req.seed_joints)
            .map_err(|e| format!("PTP start-pose IK diverged: {e}"))?;
        let q_goal = self
            .ik(&req.target_pose, &q_start)
            .map_err(|e| format!("PTP goal-pose IK diverged: {e}"))?;

        // Both endpoints must be inside the per-model position box (Req 4.2).
        let (lo, hi, slack) = (&self.limits.q_lower, &self.limits.q_upper, self.limits.pos_slack);
        if !position_within_limits(&q_start, lo, hi, slack) {
            return Err("PTP start joints outside position limits".to_string());
        }
        if !position_within_limits(&q_goal, lo, hi, slack) {
            return Err("PTP goal joints outside position limits".to_string());
        }

        // Velocity fraction in [0.01, 1.0] — vel_percent is a percent (parity
        // with interpreter::compute_ptp_duration_ik and path::velocity_feasible,
        // which expect vel_frac = vel_percent / 100).
        let vel_frac = (req.vel_percent / 100.0).clamp(0.01, 1.0);

        // 6-DOF joint-space Ruckig instance on the fixed KRC4 takt (Req 9.1, 2.1).
        let mut otg = Ruckig::<6, ThrowErrorHandler>::new(None, KRC4_TIMESTEP);
        let mut input = InputParameter::<6>::new(None);
        let mut output = OutputParameter::<6>::new(None);

        for i in 0..6 {
            input.current_position[i] = q_start[i];
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
            input.target_position[i] = q_goal[i];
            // target_velocity / target_acceleration default to 0 (rest-to-rest).
            input.max_velocity[i] = (self.limits.v_abs[i] * vel_frac).max(1e-6);
            input.max_acceleration[i] = self.limits.a_abs[i];
            input.max_jerk[i] = self.limits.j_abs[i]; // sanitized non-empty (Req 2.3).
        }

        // Knot 0 is the start configuration; subsequent knots are one takt apart.
        let mut qs: Vec<[f64; 6]> = Vec::with_capacity(64);
        qs.push(q_start);

        let mut takts: usize = 0;
        loop {
            let result = otg
                .update(&input, &mut output)
                .map_err(|e| format!("PTP Ruckig update failed: {e:?}"))?;

            match result {
                RuckigResult::Working | RuckigResult::Finished => {}
                other => return Err(format!("PTP Ruckig returned error state: {other:?}")),
            }

            takts += 1;
            if takts > MAX_TAKTS {
                return Err(format!(
                    "PTP OTG exceeded takt bound ({MAX_TAKTS}); goal not reached"
                ));
            }

            // Push the joint sample directly — no per-takt IK for PTP (Req 9.1).
            let mut q_frame = [0.0f64; 6];
            for i in 0..6 {
                q_frame[i] = output.new_position[i];
            }
            if !position_within_limits(&q_frame, lo, hi, slack) {
                return Err(format!("PTP joint-limit violation at takt {takts}"));
            }
            qs.push(q_frame);

            // Mandatory S-curve State_Transfer into the next takt (Req 2.2):
            // copies new_position/velocity/acceleration into current_*.
            output.pass_to_input(&mut input);

            if result == RuckigResult::Finished {
                break;
            }
        }

        // duration_s = number_of_takts · KRC4_TIMESTEP (Req 2.5). With knot 0 the
        // start config, qs.len() == takts + 1, so duration spans the dense knots.
        let duration_s = takts as f64 * KRC4_TIMESTEP;

        Ok(TrajectorySolution {
            duration_s,
            path: JointPath::Dense {
                qs,
                takt_s: KRC4_TIMESTEP,
            },
            tier: SolverTier::RuckigOtg,
            note: None,
        })
    }

    /// LIN (linear Cartesian) motion: run a 3-DOF Ruckig instance on the
    /// Cartesian translation axes and convert each takt's Cartesian sample to
    /// joint angles with a per-takt Pinocchio IK solve (Requirements 10.1, 10.2).
    ///
    /// The joint stream is seeded by a single start-pose IK warm-started from
    /// `req.seed_joints`, producing `q_current`. A 3-DOF Ruckig instance is then
    /// driven from the start translation to the target translation — both
    /// converted **mm → m** because the OTG runs in meters while the KUKA poses
    /// stay in mm (Requirement 8.1). The Cartesian `max_velocity` is
    /// [`CART_VMAX_DEFAULT`] scaled by the commanded velocity fraction
    /// (`$VEL.CP · vel%`, Requirement 10.3), with non-empty
    /// [`CART_AMAX_DEFAULT`] / [`CART_JMAX_DEFAULT`] so the profile is a
    /// jerk-limited S-curve (Requirement 2.3).
    ///
    /// Each takt: the new Cartesian sample (m → mm) is recomposed into a full
    /// 6-DOF pose by **holding the target orientation** (the commanded
    /// `req.target_pose` A/B/C; see the orientation note below), and Pinocchio IK
    /// is solved **warm-started from the previous takt's `q_current`** so the
    /// closest-configuration seeding locks the KUKA Status/Turn branch
    /// (Requirement 10.2). The loop runs on the fixed 12 ms takt (Requirement
    /// 2.1) with the mandatory velocity/acceleration State_Transfer via
    /// `output.pass_to_input` (Requirement 2.2), bounded by [`MAX_TAKTS`]
    /// (Requirement 2.6).
    ///
    /// # Orientation choice
    /// The per-takt full pose holds the **target** orientation for every takt
    /// (`pose_with_orientation` in the design holds or slerps; holding the
    /// target A/B/C is the accepted simplification). This keeps the wrist
    /// configuration fixed along the linear translation and is exact at the
    /// final knot, where orientation fidelity matters for the FK reach check.
    ///
    /// # Errors
    /// Returns `Err(reason)` — a graceful motion fault, never a panic — when the
    /// start-pose IK diverges, a per-takt IK solve diverges (outside the
    /// reachable envelope / near a singularity, Requirement 10.4), a produced
    /// joint sample violates the position limits, the Ruckig solver errors, the
    /// takt bound is exhausted, or the final knot does not reach the target pose
    /// within [`LIN_FRAME_TOL_MM`] under forward kinematics (Requirement 10.5).
    /// In every case the layered provider downgrades to the Pinocchio-IK tier.
    pub fn resolve_lin(&mut self, req: &MotionRequest) -> Result<TrajectorySolution, String> {
        debug_assert_eq!(req.kind, MotionKind::Lin, "resolve_lin called with non-LIN request");

        // Seed the joint stream at the start pose, closest-config to the
        // warm-start seed (Req 10.2). May Err → graceful downgrade.
        let mut q_current = self
            .ik(&req.start_pose, &req.seed_joints)
            .map_err(|e| format!("LIN start-pose IK diverged: {e}"))?;

        let (lo, hi, slack) = (self.limits.q_lower, self.limits.q_upper, self.limits.pos_slack);
        if !position_within_limits(&q_current, &lo, &hi, slack) {
            return Err("LIN start joints outside position limits".to_string());
        }

        // mm → m for the Cartesian OTG (Req 8.1 keeps pose units; conversion is
        // internal). Orientation (A/B/C) is held from the target pose.
        let start_xyz_m = [
            req.start_pose[0] / MM_PER_M,
            req.start_pose[1] / MM_PER_M,
            req.start_pose[2] / MM_PER_M,
        ];
        let target_xyz_m = [
            req.target_pose[0] / MM_PER_M,
            req.target_pose[1] / MM_PER_M,
            req.target_pose[2] / MM_PER_M,
        ];
        let target_abc = [req.target_pose[3], req.target_pose[4], req.target_pose[5]];

        // Velocity fraction in [0.01, 1.0] — vel_percent is a percent, matching
        // the PTP convention and path::velocity_feasible (vel_frac = pct / 100).
        let vel_frac = (req.vel_percent / 100.0).clamp(0.01, 1.0);
        let cart_vmax = (CART_VMAX_DEFAULT * vel_frac).max(1e-6);

        // 3-DOF Cartesian-translation Ruckig instance on the fixed takt (Req 10.1, 2.1).
        let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, KRC4_TIMESTEP);
        let mut input = InputParameter::<3>::new(None);
        let mut output = OutputParameter::<3>::new(None);

        for i in 0..3 {
            input.current_position[i] = start_xyz_m[i];
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
            input.target_position[i] = target_xyz_m[i];
            input.max_velocity[i] = cart_vmax; // $VEL.CP · vel% (Req 10.3).
            input.max_acceleration[i] = CART_AMAX_DEFAULT;
            input.max_jerk[i] = CART_JMAX_DEFAULT; // non-empty → S-curve (Req 2.3).
        }

        // Knot 0 is the start configuration; subsequent knots are one takt apart.
        let mut qs: Vec<[f64; 6]> = Vec::with_capacity(64);
        qs.push(q_current);

        let mut takts: usize = 0;
        loop {
            let result = otg
                .update(&input, &mut output)
                .map_err(|e| format!("LIN Ruckig update failed: {e:?}"))?;

            match result {
                RuckigResult::Working | RuckigResult::Finished => {}
                other => return Err(format!("LIN Ruckig returned error state: {other:?}")),
            }

            takts += 1;
            if takts > MAX_TAKTS {
                return Err(format!(
                    "LIN OTG exceeded takt bound ({MAX_TAKTS}); goal not reached"
                ));
            }

            // Recompose the Cartesian sample (m → mm) into a full 6-DOF pose,
            // holding the target orientation (see orientation note).
            let next_pose = [
                output.new_position[0] * MM_PER_M,
                output.new_position[1] * MM_PER_M,
                output.new_position[2] * MM_PER_M,
                target_abc[0],
                target_abc[1],
                target_abc[2],
            ];

            // Per-takt IK warm-started from the previous takt's joint state, so
            // closest-configuration seeding locks KUKA Status/Turn (Req 10.2).
            let q_new = self.ik(&next_pose, &q_current).map_err(|e| {
                format!("LIN motion fault: IK diverged at takt {takts} (outside envelope / singular): {e}")
            })?;
            if !position_within_limits(&q_new, &lo, &hi, slack) {
                return Err(format!("LIN joint-limit violation at takt {takts}"));
            }

            qs.push(q_new);
            q_current = q_new;

            // Mandatory S-curve State_Transfer into the next takt (Req 2.2):
            // copies new_position/velocity/acceleration into current_*.
            output.pass_to_input(&mut input);

            if result == RuckigResult::Finished {
                break;
            }
        }

        // Final-knot fidelity: FK of the last joint sample must reach the target
        // translation within frame tolerance, else downgrade (Req 10.5).
        let last_q = *qs
            .last()
            .ok_or_else(|| "LIN produced no knots".to_string())?;
        let fk = self.pinocchio.forward_kinematics(&last_q);
        let dx = fk[0] - req.target_pose[0];
        let dy = fk[1] - req.target_pose[1];
        let dz = fk[2] - req.target_pose[2];
        let dist_mm = (dx * dx + dy * dy + dz * dz).sqrt();
        if dist_mm > LIN_FRAME_TOL_MM {
            return Err(format!(
                "LIN final knot missed target: {dist_mm:.3} mm > {LIN_FRAME_TOL_MM} mm tolerance"
            ));
        }

        // duration_s = number_of_takts · KRC4_TIMESTEP (Req 2.5).
        let duration_s = takts as f64 * KRC4_TIMESTEP;

        Ok(TrajectorySolution {
            duration_s,
            path: JointPath::Dense {
                qs,
                takt_s: KRC4_TIMESTEP,
            },
            tier: SolverTier::RuckigOtg,
            note: None,
        })
    }
}

/// Final consolidated validation of a resolved [`TrajectorySolution`] against
/// the per-model [`JointLimits`], using the `path.rs` helpers on its
/// [`JointPath`].
///
/// This is the single accept/reject gate applied by [`RuckigPlanner::resolve`]
/// after the sub-resolver returns (the sub-resolvers themselves already enforce
/// per-takt position limits and goal/frame fidelity, which is why goal-reach is
/// not re-checked here). A path is accepted only when it is within position
/// limits, velocity-feasible at `vel_frac` over one [`KRC4_TIMESTEP`], and
/// acceleration- and jerk-feasible (Requirements 2.4, 4.1, 4.2, 4.3, 4.5).
///
/// On the first failed check it returns `Err(reason)` naming the specific
/// failure so the layered provider downgrades; otherwise `Ok(())`.
///
/// This is a free function (rather than a method) so it can be unit-tested
/// against hand-built [`JointPath`]s without a loaded Pinocchio model.
fn validate_solution(
    solution: &TrajectorySolution,
    limits: &JointLimits,
    vel_frac: f64,
) -> Result<(), String> {
    let path = &solution.path;

    if !path.within_limits(&limits.q_lower, &limits.q_upper, limits.pos_slack) {
        return Err("resolved path violates joint position limits".to_string());
    }
    if !path.velocity_feasible(vel_frac, &limits.v_abs, KRC4_TIMESTEP) {
        return Err("resolved path violates per-axis joint velocity limits".to_string());
    }
    if !path.acceleration_feasible(KRC4_TIMESTEP, limits) {
        return Err("resolved path violates per-axis joint acceleration limits".to_string());
    }
    if !path.jerk_feasible(KRC4_TIMESTEP, limits) {
        return Err("resolved path violates per-axis joint jerk limits".to_string());
    }

    Ok(())
}

/// Sanitize a per-axis jerk-bound array so it is never empty/zero and stays
/// inside the physical `[JERK_BAND_MIN, JERK_BAND_MAX]` band.
///
/// For each axis: a finite, strictly-positive value is clamped into the band; a
/// non-finite or non-positive value is replaced by the corresponding
/// [`KR240_J_ABS`] fallback (itself within the band). This is the guard that
/// keeps Ruckig in jerk-limited S-curve mode (Requirements 2.3, 5.4).
fn sanitized_jerk_bounds(j_abs: &[f64; 6]) -> [f64; 6] {
    let mut out = [0.0f64; 6];
    for i in 0..6 {
        out[i] = if j_abs[i].is_finite() && j_abs[i] > 0.0 {
            j_abs[i].clamp(JERK_BAND_MIN, JERK_BAND_MAX)
        } else {
            // Fallback constant is guaranteed in-band, but clamp defensively.
            KR240_J_ABS[i].clamp(JERK_BAND_MIN, JERK_BAND_MAX)
        };
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-12;

    #[test]
    fn constants_have_expected_values() {
        assert!((KRC4_TIMESTEP - 0.012).abs() <= EPS);
        assert_eq!(MAX_TAKTS, 5000);
        assert!(JERK_BAND_MIN < JERK_BAND_MAX);
        assert!((JERK_BAND_MIN - 20.0).abs() <= EPS);
        assert!((JERK_BAND_MAX - 150.0).abs() <= EPS);
    }

    #[test]
    fn valid_in_band_jerk_is_preserved() {
        // KR240_J_ABS values all lie inside [20, 150], so they pass through.
        let out = sanitized_jerk_bounds(&KR240_J_ABS);
        for i in 0..6 {
            assert!((out[i] - KR240_J_ABS[i]).abs() <= EPS, "axis {i}");
            assert!(out[i] >= JERK_BAND_MIN && out[i] <= JERK_BAND_MAX);
        }
    }

    #[test]
    fn zero_negative_and_nonfinite_jerk_fall_back() {
        // Empty/zero jerk bounds (and non-finite ones) are rejected and replaced
        // by the in-band KR240 fallback for that axis (Req 2.3 / 5.4).
        let bad = [0.0, -5.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.0];
        let out = sanitized_jerk_bounds(&bad);
        for i in 0..6 {
            let expected = KR240_J_ABS[i].clamp(JERK_BAND_MIN, JERK_BAND_MAX);
            assert!((out[i] - expected).abs() <= EPS, "axis {i}");
            assert!(out[i] >= JERK_BAND_MIN && out[i] <= JERK_BAND_MAX);
        }
    }

    #[test]
    fn out_of_band_jerk_is_clamped_into_band() {
        // Above-band values clamp down to the max; below-band positive values
        // clamp up to the min; both stay a non-empty, in-band S-curve bound.
        let out = sanitized_jerk_bounds(&[300.0, 1000.0, 151.0, 5.0, 19.999, 0.001]);
        assert!((out[0] - JERK_BAND_MAX).abs() <= EPS);
        assert!((out[1] - JERK_BAND_MAX).abs() <= EPS);
        assert!((out[2] - JERK_BAND_MAX).abs() <= EPS);
        assert!((out[3] - JERK_BAND_MIN).abs() <= EPS);
        assert!((out[4] - JERK_BAND_MIN).abs() <= EPS);
        assert!((out[5] - JERK_BAND_MIN).abs() <= EPS);
    }

    #[test]
    fn cartesian_constants_are_sane_and_nonempty() {
        // mm↔m conversion factor.
        assert!((MM_PER_M - 1000.0).abs() <= EPS);
        // $VEL.CP default per add-ruckig.md.
        assert!((CART_VMAX_DEFAULT - 2.0).abs() <= EPS);
        assert!((CART_AMAX_DEFAULT - 5.0).abs() <= EPS);
        // Jerk MUST be non-empty / strictly positive so Ruckig stays in
        // S-curve mode (Req 2.3).
        assert!(CART_JMAX_DEFAULT > 0.0);
        assert!((CART_JMAX_DEFAULT - 50.0).abs() <= EPS);
        // A few-mm frame tolerance for the LIN final-knot FK reach check.
        assert!(LIN_FRAME_TOL_MM > 0.0 && LIN_FRAME_TOL_MM <= 10.0);
    }

    // ---- validate_solution (the resolve() final consolidated gate) -------
    //
    // These exercise the limit/velocity/accel/jerk validation against
    // hand-built Dense paths and the KR240 fallback limits, with no Pinocchio
    // model required. The IK-backed full `resolve` path is covered by the
    // fixture-guarded optional tests (tasks 3.5–3.10).

    /// Wrap a hand-built Dense knot sequence into a `TrajectorySolution` tagged
    /// as the Ruckig tier (the only field `validate_solution` inspects is
    /// `path`).
    fn dense_solution(qs: Vec<[f64; 6]>) -> TrajectorySolution {
        TrajectorySolution {
            duration_s: qs.len().saturating_sub(1) as f64 * KRC4_TIMESTEP,
            path: JointPath::Dense {
                qs,
                takt_s: KRC4_TIMESTEP,
            },
            tier: SolverTier::RuckigOtg,
            note: None,
        }
    }

    /// An in-range base configuration: axis 1 is constrained to
    /// `[-2.4435, -0.0873]` rad, so 0.0 would be out of range — use -1.0.
    const BASE_Q: [f64; 6] = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0];

    #[test]
    fn validate_accepts_a_feasible_dense_path() {
        let limits = JointLimits::kr240_fallback();
        // Two identical in-range knots: within limits, zero velocity, accel/jerk
        // trivially feasible (<3 / <4 knots).
        let sol = dense_solution(vec![BASE_Q, BASE_Q]);
        assert!(validate_solution(&sol, &limits, 1.0).is_ok());
    }

    #[test]
    fn validate_accepts_short_paths_with_trivial_accel_jerk() {
        let limits = JointLimits::kr240_fallback();
        // A 3-knot path with small per-takt steps: velocity-feasible, and accel
        // is the only higher-order check that applies (jerk needs 4 knots).
        let mut k1 = BASE_Q;
        k1[0] = 0.005; // |dq| = 0.005 < v_abs[0]*0.012 ≈ 0.023
        let mut k2 = BASE_Q;
        k2[0] = 0.010;
        let sol = dense_solution(vec![BASE_Q, k1, k2]);
        assert!(validate_solution(&sol, &limits, 1.0).is_ok());
    }

    #[test]
    fn validate_rejects_position_limit_violation() {
        let limits = JointLimits::kr240_fallback();
        // Axis 1 is constrained to [-2.4435, -0.0873] rad; +1.0 is outside that
        // box and is not wrapped into range by normalization. A single-knot path
        // makes velocity/accel/jerk trivially feasible, isolating the position
        // check.
        let mut bad = BASE_Q;
        bad[1] = 1.0;
        let sol = dense_solution(vec![bad]);
        let err = validate_solution(&sol, &limits, 1.0).unwrap_err();
        assert!(err.contains("position limits"), "unexpected reason: {err}");
    }

    #[test]
    fn validate_rejects_velocity_limit_violation() {
        let limits = JointLimits::kr240_fallback();
        // Both knots are within the position box, but a 1.0 rad step in one takt
        // (≫ v_abs[0]·0.012 ≈ 0.023 rad) is velocity-infeasible.
        let mut fast = BASE_Q;
        fast[0] = 1.0;
        let sol = dense_solution(vec![BASE_Q, fast]);
        let err = validate_solution(&sol, &limits, 1.0).unwrap_err();
        assert!(err.contains("velocity limits"), "unexpected reason: {err}");
    }

    #[test]
    fn validate_rejects_acceleration_limit_violation() {
        let limits = JointLimits::kr240_fallback();
        // Velocity-feasible per step (|dq| = 0.02 < 0.023) but the direction
        // reversal makes the second difference (-0.04) imply ≈277 rad/s²,
        // far above a_abs[0] = 10 rad/s².
        let mut up = BASE_Q;
        up[0] = 0.02;
        let sol = dense_solution(vec![BASE_Q, up, BASE_Q]);
        let err = validate_solution(&sol, &limits, 1.0).unwrap_err();
        assert!(
            err.contains("acceleration limits"),
            "unexpected reason: {err}"
        );
    }

    #[test]
    fn validate_rejects_jerk_limit_violation() {
        let limits = JointLimits::kr240_fallback();
        // Steps 0.010, 0.011, 0.010 → velocity-feasible; second differences
        // ±0.001 → accel ≈6.9 rad/s² < 10 (feasible); third difference -0.002 →
        // jerk ≈ -1157 rad/s³, above j_abs[0] = 100 rad/s³.
        let k0 = BASE_Q;
        let mut k1 = BASE_Q;
        k1[0] = 0.010;
        let mut k2 = BASE_Q;
        k2[0] = 0.021;
        let mut k3 = BASE_Q;
        k3[0] = 0.031;
        let sol = dense_solution(vec![k0, k1, k2, k3]);
        let err = validate_solution(&sol, &limits, 1.0).unwrap_err();
        assert!(err.contains("jerk limits"), "unexpected reason: {err}");
    }
}
