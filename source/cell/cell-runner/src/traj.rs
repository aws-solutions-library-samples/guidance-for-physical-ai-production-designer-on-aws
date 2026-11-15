//! Provider request/result types for the layered trajectory provider.
//!
//! This module defines the data carried across the `TrajectoryProvider`
//! abstraction:
//!   * [`MotionKind`]         — whether the motion is point-to-point (joint
//!                              space) or linear (Cartesian space), driving the
//!                              Ruckig OTG configuration.
//!   * [`MotionRequest`]      — one motion to resolve, in KUKA units (mm / deg)
//!                              plus a joint-space warm-start seed (radians).
//!   * [`SolverTier`]         — which tier produced a solution (Ruckig OTG →
//!                              Pinocchio IK → heuristic), used for startup and
//!                              per-motion downgrade diagnostics.
//!   * [`TrajectorySolution`] — the resolved duration + [`JointPath`] plus the
//!                              tier and an optional downgrade note.
//!
//! The [`TrajectoryProvider`] trait is defined here; the concrete
//! `LayeredTrajectoryProvider` that implements it is built in a later task
//! (5.2). This module also provides [`estimate_duration_per_axis`], the shared
//! velocity-limited duration estimate used by the fallback tiers.

#![allow(dead_code)]

use crate::path::JointPath;

/// What kind of motion to generate, driving the Ruckig OTG configuration:
/// `Ptp` runs a joint-space (6 DOF) profile, `Lin` runs a Cartesian-space
/// (3 DOF translation) profile with per-takt Pinocchio inverse kinematics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionKind {
    /// Point-to-point: joint-space motion across the six axes.
    Ptp,
    /// Linear: Cartesian-space motion along the translation axes.
    Lin,
}

/// One motion to resolve, in KUKA units (mm / degrees) plus a joint-space seed (radians).
#[derive(Debug, Clone)]
pub struct MotionRequest {
    /// Kind of motion (`Ptp` joint-space | `Lin` Cartesian-space).
    pub kind: MotionKind,
    /// Start pose `[x, y, z, a, b, c]` in mm / deg (flange / `tool0`).
    pub start_pose: [f64; 6],
    /// Target pose `[x, y, z, a, b, c]` in mm / deg (flange / `tool0`).
    pub target_pose: [f64; 6],
    /// Commanded velocity fraction (KRL `$VEL.CP` / `vel_ptp`).
    pub vel_percent: f64,
    /// `C_DIS` approximate (blend) motion flag.
    pub approximate: bool,
    /// Warm-start joint seed (radians).
    pub seed_joints: [f64; 6],
}

/// Which tier produced the solution (for startup + telemetry diagnostics).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverTier {
    /// Ruckig online-trajectory-generation solver (primary tier).
    RuckigOtg,
    /// Pinocchio inverse-kinematics endpoint fallback.
    PinocchioIk,
    /// Heuristic timing with Cartesian-space interpolation (last resort).
    Heuristic,
}

impl SolverTier {
    /// Short, human-readable name for startup tier reporting.
    pub fn name(self) -> &'static str {
        match self {
            SolverTier::RuckigOtg => "Ruckig OTG",
            SolverTier::PinocchioIk => "Pinocchio IK",
            SolverTier::Heuristic => "heuristic timing",
        }
    }
}

impl std::fmt::Display for SolverTier {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// The result of resolving a [`MotionRequest`]: a motion duration plus the
/// joint path that realizes it, tagged with the producing tier and an optional
/// downgrade reason (set whenever `tier` is not [`SolverTier::RuckigOtg`]).
#[derive(Debug, Clone)]
pub struct TrajectorySolution {
    /// Motion duration in seconds (always `> 0` for a usable solution).
    pub duration_s: f64,
    /// The joint path to sample during interpolation.
    pub path: JointPath,
    /// Tier that produced this solution.
    pub tier: SolverTier,
    /// Downgrade reason, present when `tier` is below RuckigOtg.
    pub note: Option<String>,
}

/// The trajectory generation abstraction. Implementors resolve a
/// [`MotionRequest`] into a [`TrajectorySolution`] (duration + [`JointPath`]),
/// resolve a single pose to joints, and report which [`SolverTier`] is active.
///
/// `resolve_motion` MUST NOT fail hard: it always returns a usable solution,
/// downgrading the tier (Ruckig OTG → Pinocchio IK → heuristic) as needed so
/// KRL execution is never blocked or the controller terminated.
///
/// The bound on `Send` lets a provider be owned per-robot across the controller
/// threads.
pub trait TrajectoryProvider: Send {
    /// Resolve a motion to a duration + joint path. MUST NOT fail hard:
    /// always returns a usable solution, downgrading the tier as needed.
    fn resolve_motion(&mut self, req: &MotionRequest) -> TrajectorySolution;

    /// One-shot pose→joints (used for home presolve and cold joint solving).
    /// Returns `None` only when no tier can resolve the pose at all.
    fn solve_pose(&mut self, pose: &[f64; 6], seed: &[f64; 6]) -> Option<[f64; 6]>;

    /// The highest tier this provider can currently produce solutions with.
    fn tier(&self) -> SolverTier;
}

/// Estimate a motion's duration from per-axis joint deltas using the same
/// velocity-limited timing arithmetic as the interpreter's
/// `compute_ptp_duration_ik`.
///
/// This is the shared duration estimate used by the fallback tiers (Pinocchio
/// IK and heuristic). The inverse kinematics that maps poses to `q_from` /
/// `q_to` is done by the caller; this helper does *only* the timing math so the
/// value it returns matches what `compute_ptp_duration_ik` would return for the
/// same joint deltas.
///
/// Semantics (mirrors `interpreter::compute_ptp_duration_ik`):
///   * `vel_frac = (vel_percent / 100).max(0.01)`
///   * per-axis time `= |q_to[i] - q_from[i]| / (vel_frac * v_abs[i])`
///   * take the maximum over all axes
///   * add accel/decel time: `0.05` when `approximate` (blend), else `0.3`
///   * clamp the result to a `0.1 s` floor
///
/// `v_abs` is the per-axis velocity limit (rad/s); pass the model's
/// `JointLimits::v_abs` (the KR240 fallback equals the `JOINT_VEL_LIMITS_RAD`
/// table used by `compute_ptp_duration_ik`).
pub fn estimate_duration_per_axis(
    q_from: &[f64; 6],
    q_to: &[f64; 6],
    vel_percent: f64,
    approximate: bool,
    v_abs: &[f64; 6],
) -> f64 {
    let vel_frac = (vel_percent / 100.0).max(0.01);

    let mut max_axis_time = 0.0f64;
    for i in 0..6 {
        let dq = (q_to[i] - q_from[i]).abs();
        let axis_time = dq / (vel_frac * v_abs[i]);
        max_axis_time = max_axis_time.max(axis_time);
    }

    let accel_time = if approximate { 0.05 } else { 0.3 };
    (max_axis_time + accel_time).max(0.1)
}

#[cfg(test)]
mod estimate_duration_tests {
    use super::estimate_duration_per_axis;

    /// KR240 R2900 joint velocity limits (rad/s) — the same table
    /// `compute_ptp_duration_ik` uses (`JOINT_VEL_LIMITS_RAD`).
    const V_ABS: [f64; 6] = [1.9199, 1.7453, 1.9199, 3.7525, 3.2289, 4.5379];

    /// Reproduce `compute_ptp_duration_ik` arithmetic independently for parity.
    fn reference(q_from: &[f64; 6], q_to: &[f64; 6], vel: f64, approx: bool) -> f64 {
        let vel_frac = (vel / 100.0).max(0.01);
        let mut max_axis_time = 0.0f64;
        for i in 0..6 {
            let dq = (q_to[i] - q_from[i]).abs();
            max_axis_time = max_axis_time.max(dq / (vel_frac * V_ABS[i]));
        }
        let accel_time = if approx { 0.05 } else { 0.3 };
        (max_axis_time + accel_time).max(0.1)
    }

    #[test]
    fn zero_delta_is_accel_time_only() {
        // No travel: duration is just the accel/decel time (0.3 non-approx,
        // 0.05 approx), each above the 0.1 s floor.
        let q = [0.0; 6];
        assert_eq!(estimate_duration_per_axis(&q, &q, 100.0, false, &V_ABS), 0.3);
        assert_eq!(estimate_duration_per_axis(&q, &q, 100.0, true, &V_ABS), 0.1);
    }

    #[test]
    fn slowest_axis_dominates() {
        // Only axis 1 (slowest limit) moves; it should set the duration.
        let from = [0.0; 6];
        let to = [0.0, 1.7453, 0.0, 0.0, 0.0, 0.0]; // 1 s of travel at full speed
        let d = estimate_duration_per_axis(&from, &to, 100.0, false, &V_ABS);
        // 1.0 (axis time) + 0.3 (accel) = 1.3
        assert!((d - 1.3).abs() < 1e-9, "got {d}");
    }

    #[test]
    fn approximate_uses_smaller_accel_time() {
        let from = [0.0; 6];
        let to = [0.5, -0.3, 0.8, 1.0, -0.4, 0.2];
        let exact = estimate_duration_per_axis(&from, &to, 50.0, false, &V_ABS);
        let approx = estimate_duration_per_axis(&from, &to, 50.0, true, &V_ABS);
        assert!(approx < exact);
    }

    #[test]
    fn matches_compute_ptp_duration_ik_semantics() {
        let cases = [
            ([0.0; 6], [0.1, -0.2, 0.3, -0.4, 0.5, -0.6], 100.0, false),
            ([0.2, 0.1, -0.1, 0.0, 0.3, -0.2], [1.0, -1.2, 0.8, 2.0, -1.0, 1.5], 30.0, true),
            ([0.0; 6], [0.0; 6], 10.0, false),
            ([-1.0; 6], [1.0; 6], 1.0, false),
        ];
        for (from, to, vel, approx) in cases {
            let got = estimate_duration_per_axis(&from, &to, vel, approx, &V_ABS);
            let want = reference(&from, &to, vel, approx);
            assert!((got - want).abs() < 1e-12, "from={from:?} to={to:?} vel={vel} approx={approx}: got {got}, want {want}");
        }
    }

    #[test]
    fn vel_percent_clamped_to_floor() {
        // vel_percent of 0 must not divide by zero; clamps to 0.01.
        let from = [0.0; 6];
        let to = [0.1; 6];
        let d = estimate_duration_per_axis(&from, &to, 0.0, false, &V_ABS);
        assert!(d.is_finite() && d > 0.0);
    }
}

// ---------------------------------------------------------------------------
// LayeredTrajectoryProvider — load + startup tier reporting (task 5.2)
// ---------------------------------------------------------------------------

use crate::limits::JointLimits;
use crate::ruckig_otg::RuckigPlanner;
use std::path::Path;

/// Composes the Ruckig OTG tier (primary) over the Pinocchio-IK endpoint tier
/// (fallback) over pure heuristic timing (last resort), selecting the active
/// top [`SolverTier`] once at load time and reporting it at startup
/// (Requirements 1.2–1.6).
///
/// # Struct design choice (ownership)
///
/// The design document sketches three fields —
/// `{ ruckig: Option<RuckigPlanner>, pinocchio: Option<PinocchioModel>, limits }`.
/// That cannot be realized from a single loaded model: [`RuckigPlanner`] **owns
/// its [`crate::pinocchio::PinocchioModel`] by value** (its IK/FK methods take
/// `&mut self`, which an `Rc` cannot hand out), so holding both a planner and a
/// separate `pinocchio` for the fallback tier would require loading the model
/// twice. Instead the model is loaded **once**, moved into the
/// [`RuckigPlanner`], and the Pinocchio-IK fallback tier (task 5.3) routes
/// through the planner's [`RuckigPlanner::ik`] / [`RuckigPlanner::limits`].
///
/// The struct therefore carries:
///   * `ruckig`  — `Some` when the URDF loaded (the planner wraps the model);
///                 `None` when the URDF could not load.
///   * `limits`  — the per-model [`JointLimits`] kept on the provider so the
///                 heuristic / Pinocchio-IK fallback tiers (task 5.3) can size
///                 durations even when no planner is available. (The planner
///                 keeps its own jerk-sanitized copy for the Ruckig inputs.)
///   * `tier`    — the active top tier, fixed at load (Requirement 1.3).
///
/// # Tier selection at load
///
/// Ruckig is a **pure-Rust** crate, so the [`RuckigPlanner`] always constructs
/// successfully once the [`crate::pinocchio::PinocchioModel`] loads. There is
/// thus no "Pinocchio loaded but Ruckig failed" outcome reachable from `load`:
///   * URDF loads  → [`SolverTier::RuckigOtg`] (Requirement 1.4).
///   * URDF fails  → [`SolverTier::Heuristic`], naming the unavailable URDF
///                   (Requirement 1.6).
///
/// The [`SolverTier::PinocchioIk`] startup branch (Requirement 1.5) is therefore
/// currently **unreachable via `load`**, but its reporting message is still
/// implemented in [`startup_report_line`] so the branch exists, is unit-tested,
/// and is ready if a future change makes the Ruckig tier independently
/// disableable.
///
/// # Known limitation — per-model limit extraction
///
/// `load` builds the per-model [`JointLimits`] via [`JointLimits::kr240_fallback`]
/// for now: there is no function that extracts a [`crate::limits::UrdfLimits`]
/// from a loaded [`crate::pinocchio::PinocchioModel`] yet (the FFI surface in
/// `pinocchio.rs` exposes only load / FK / IK). Per-model URDF limit extraction
/// (Requirement 5.1/5.2) can be wired in later by reading the URDF limits and
/// calling [`JointLimits::from_urdf_limits`]; until then every model uses the
/// parity-safe KR240 fallback set. `kr240_fallback()` is equivalent to
/// `from_urdf_limits(&UrdfLimits::default())`.
pub struct LayeredTrajectoryProvider {
    /// The Ruckig OTG planner (which owns the Pinocchio model). `Some` once the
    /// URDF loads; `None` when the URDF is unavailable (heuristic tier).
    ruckig: Option<RuckigPlanner>,
    /// Per-model joint limits, retained on the provider for the fallback tiers.
    limits: JointLimits,
    /// Active top tier, fixed at load time.
    tier: SolverTier,
}

impl LayeredTrajectoryProvider {
    /// Load the trajectory provider for one robot model, building the Ruckig OTG
    /// planner around the Pinocchio model loaded from `urdf` in the
    /// Shared_Env_Directory and reporting the active top [`SolverTier`] once at
    /// startup (Requirements 1.2, 1.3, 1.4, 1.6).
    ///
    /// On success the model is loaded once and moved into a [`RuckigPlanner`]
    /// (tier [`SolverTier::RuckigOtg`]); on failure the provider downgrades to
    /// the heuristic tier ([`SolverTier::Heuristic`]) and names the unavailable
    /// URDF. The startup line is emitted via `eprintln!` (matching the existing
    /// `main.rs` startup logging), exactly once per call.
    ///
    /// `model_name` is the robot model key (e.g. `kr240_r2900_ultra`) used only
    /// for the startup report.
    pub fn load(urdf: &Path, model_name: &str) -> Self {
        // Per-model limits. KNOWN LIMITATION: no URDF-limit extraction from a
        // loaded PinocchioModel exists yet, so use the parity-safe KR240
        // fallback (== from_urdf_limits(&UrdfLimits::default())). See the struct
        // doc for how per-model extraction can be wired later.
        let limits = JointLimits::kr240_fallback();

        match crate::pinocchio::PinocchioModel::from_urdf(urdf) {
            Ok(model) => {
                // Ruckig is pure-Rust and always constructs once the model loads,
                // so the active top tier is RuckigOtg (Req 1.4).
                let planner = RuckigPlanner::new(model, limits);
                let tier = SolverTier::RuckigOtg;
                eprintln!(
                    "  {}",
                    startup_report_line(tier, model_name, urdf, None)
                );
                Self {
                    ruckig: Some(planner),
                    limits,
                    tier,
                }
            }
            Err(e) => {
                // URDF unavailable → heuristic timing, naming the URDF (Req 1.6).
                let tier = SolverTier::Heuristic;
                eprintln!(
                    "  WARNING: {}",
                    startup_report_line(tier, model_name, urdf, Some(&e))
                );
                Self {
                    ruckig: None,
                    limits,
                    tier,
                }
            }
        }
    }

    /// The active top [`SolverTier`] this provider resolves with, fixed at load
    /// (Requirement 1.3). Inherent accessor; the full [`TrajectoryProvider`]
    /// trait implementation (with `resolve_motion` / `solve_pose`) is task
    /// 5.3 / 5.4.
    pub fn tier(&self) -> SolverTier {
        self.tier
    }

    /// The per-model joint limit set retained for the fallback tiers.
    pub fn limits(&self) -> &JointLimits {
        &self.limits
    }

    /// Whether the Ruckig OTG tier is available (the URDF loaded). When `false`
    /// the provider operates on the heuristic tier.
    pub fn has_ruckig(&self) -> bool {
        self.ruckig.is_some()
    }

    /// Forward kinematics for home-pose derivation: joint angles (radians) ->
    /// flange pose `[x, y, z, a, b, c]` (mm, ZYX Euler degrees). Returns `None`
    /// on the heuristic tier (no Pinocchio model loaded).
    pub fn forward_kinematics(&mut self, joints: &[f64; 6]) -> Option<[f64; 6]> {
        self.ruckig.as_mut().map(|r| r.fk(joints))
    }

    /// Mutable access to the Ruckig planner, used by the resolve/solve paths
    /// (tasks 5.3 / 5.4) to drive the OTG loop and route the Pinocchio-IK
    /// fallback through the planner's IK. `None` on the heuristic tier.
    pub(crate) fn ruckig_mut(&mut self) -> Option<&mut RuckigPlanner> {
        self.ruckig.as_mut()
    }

    /// Pinocchio-IK endpoint solve for the second [`SolverTier::PinocchioIk`]
    /// fallback tier, reproducing the base system's `server.rs` motion-start
    /// endpoint solve **including the wrist-flip retry** (Requirement 7.4 — same
    /// IK calls, iteration count, and limit predicate as the base path).
    ///
    /// Both inverse-kinematics solves route **through the Ruckig planner's
    /// [`RuckigPlanner::ik`]** (the planner owns the Pinocchio model), warm-start
    /// the start solve from `req.seed_joints`, and warm-start the end solve from
    /// the solved start joints so the closest-configuration seeding tracks
    /// smoothly. Each result is normalized and validated against the joint
    /// position limits with the base system's predicate
    /// ([`crate::interpreter::normalize_ik_result`], `JOINT_POS_LIMITS` ±0.05
    /// rad), matching `self.limits` (the KR240 fallback).
    ///
    /// On the first attempt's failure (start IK diverges / out of limits, or the
    /// end IK fails) it retries once with a **sign-flipped wrist seed**: flip the
    /// J5 sign and shift J4/J6 by ±π, seeded from the best start joints found so
    /// far (or `req.seed_joints` if none), exactly as the base system does.
    ///
    /// Returns `Some((start_joints, end_joints))` — both within limits — when a
    /// valid endpoint pair is found, else `None` so the provider downgrades to
    /// the heuristic tier. Returns `None` immediately when no Pinocchio model is
    /// available (the planner is absent on the heuristic load tier).
    fn solve_endpoints_ik(&mut self, req: &MotionRequest) -> Option<([f64; 6], [f64; 6])> {
        use std::f64::consts::PI;

        let planner = self.ruckig.as_mut()?;
        let start_target = req.start_pose;
        let end_target = req.target_pose;
        let start_seed = req.seed_joints;

        // Attempt 1: warm-start the start solve from the request seed, then the
        // end solve from the solved start joints.
        let mut best_start: Option<[f64; 6]> = None;
        if let Ok(sj) = planner.ik(&start_target, &start_seed) {
            if let Some(sj) = crate::interpreter::normalize_ik_result(sj) {
                best_start = Some(sj);
                if let Ok(ej) = planner.ik(&end_target, &sj) {
                    if let Some(ej) = crate::interpreter::normalize_ik_result(ej) {
                        return Some((sj, ej));
                    }
                }
            }
        }

        // Attempt 2: wrist-flip retry. Flip J5 sign and shift J4/J6 by ±π,
        // seeded from the best start joints found so far (else the request seed).
        let mut alt_seed = best_start.unwrap_or(start_seed);
        alt_seed[4] = -alt_seed[4];
        alt_seed[3] += if alt_seed[3] > 0.0 { -PI } else { PI };
        alt_seed[5] += if alt_seed[5] > 0.0 { -PI } else { PI };
        if let Ok(sj) = planner.ik(&start_target, &alt_seed) {
            if let Some(sj) = crate::interpreter::normalize_ik_result(sj) {
                if let Ok(ej) = planner.ik(&end_target, &sj) {
                    if let Some(ej) = crate::interpreter::normalize_ik_result(ej) {
                        return Some((sj, ej));
                    }
                }
            }
        }

        None
    }
}

impl TrajectoryProvider for LayeredTrajectoryProvider {
    /// Resolve a motion through the layered fallback chain, always returning a
    /// usable [`TrajectorySolution`] with `duration_s > 0` (design decision D6,
    /// Requirements 3.1, 3.2, 3.6).
    ///
    /// The tiers are attempted in order and the first usable result is returned:
    ///
    /// 1. **[`SolverTier::RuckigOtg`]** — when the Ruckig planner is available,
    ///    delegate to [`RuckigPlanner::resolve`] (which runs the OTG loop and its
    ///    own consolidated position / velocity / acceleration / jerk validation).
    ///    On success the solution is returned verbatim (`note = None`). On
    ///    failure the error names the reason and the chain falls through
    ///    (Requirements 3.3, 3.5).
    /// 2. **[`SolverTier::PinocchioIk`]** — solve start/end joints with the
    ///    base-system wrist-flip retry ([`Self::solve_endpoints_ik`]), accepting
    ///    only when both endpoints are within the joint position limits
    ///    (Requirements 4.4, 4.5). The duration is the shared per-axis
    ///    velocity-limited estimate ([`estimate_duration_per_axis`], parity with
    ///    `compute_ptp_duration_ik`). A [`TrajectorySolution::note`] records the
    ///    reason the Ruckig tier was skipped (Requirement 12.1).
    /// 3. **[`SolverTier::Heuristic`]** — last resort: a positive Cartesian
    ///    timing estimate from the request poses ([`heuristic_duration`], parity
    ///    with `interpreter::compute_ptp_duration`) and a [`JointPath::None`]
    ///    (no joint information), with a note naming the downgrade reason
    ///    (Requirements 3.4, 4.6, 12.1).
    ///
    /// The Pinocchio-IK tier is only reachable when the Ruckig planner is present
    /// (the planner owns the Pinocchio model); on the heuristic load tier (URDF
    /// unavailable) the chain goes straight from the unavailable Ruckig tier to
    /// the heuristic tier.
    ///
    /// This method never panics, never blocks, and never terminates the
    /// controller on a solver failure (Requirement 3.6).
    fn resolve_motion(&mut self, req: &MotionRequest) -> TrajectorySolution {
        let v_abs = self.limits.v_abs;

        // ---- Tier 1: Ruckig OTG (+ Pinocchio IK for LIN) ----
        let ruckig_reason: String = if self.ruckig.is_some() {
            match self.ruckig.as_mut().unwrap().resolve(req) {
                Ok(solution) => return solution, // tier RuckigOtg, note None
                Err(e) => format!("Ruckig OTG tier failed: {e}"),
            }
        } else {
            "Ruckig OTG tier unavailable (URDF not loaded)".to_string()
        };

        // ---- Tier 2: Pinocchio IK endpoints (incl. wrist-flip retry) ----
        // Only reachable when the planner (and its Pinocchio model) is present.
        if self.ruckig.is_some() {
            if let Some((q0, q1)) = self.solve_endpoints_ik(req) {
                let duration_s = estimate_duration_per_axis(
                    &q0,
                    &q1,
                    req.vel_percent,
                    req.approximate,
                    &v_abs,
                );
                return TrajectorySolution {
                    duration_s,
                    path: JointPath::Endpoints { start: q0, end: q1 },
                    tier: SolverTier::PinocchioIk,
                    note: Some(ruckig_reason),
                };
            }
        }

        // ---- Tier 3: heuristic timing + Cartesian interpolation ----
        let duration_s = heuristic_duration(
            &req.start_pose,
            &req.target_pose,
            req.vel_percent,
            req.approximate,
        );
        let note = if self.ruckig.is_some() {
            Some(format!("{ruckig_reason}; Pinocchio IK endpoints also failed"))
        } else {
            Some(ruckig_reason)
        };
        TrajectorySolution {
            duration_s,
            path: JointPath::None,
            tier: SolverTier::Heuristic,
            note,
        }
    }

    /// One-shot pose→joints using the highest available tier, for home presolve
    /// and cold joint solving (Requirement 1.7).
    ///
    /// Reproduces the base system's cold-solve robustness (the `server.rs`
    /// motion-tracking / home-presolve IK block, lines ~360–390, and the
    /// `main.rs` home-IK sweep, lines ~165–210): a **warm-start attempt followed
    /// by a multi-seed sweep**.
    ///
    /// 1. **Warm attempt** — try the caller's `seed` first (the previous joint
    ///    state / current joints), the cheapest and most common success path for
    ///    small pose deltas.
    /// 2. **Cold sweep** — if the warm seed diverges or normalizes out of the
    ///    limit box, sweep the same fixed set of seed variants the base system
    ///    uses for cold solving: the `$A1` base-rotation angle pointing toward
    ///    the target (both signs, `a1_neg` / `a1_pos`) crossed with a small set
    ///    of varied shoulder/elbow/wrist configurations, accepting the first
    ///    variant that normalizes within limits.
    ///
    /// Every IK solve routes through the Ruckig planner's [`RuckigPlanner::ik`]
    /// (the planner owns the Pinocchio model) and every candidate is validated
    /// with the base predicate ([`crate::interpreter::normalize_ik_result`],
    /// `JOINT_POS_LIMITS` ±0.05 rad). Returns the first valid solution, or `None`
    /// when no seed resolves the pose. Returns `None` immediately when no
    /// Pinocchio model is available (the planner is absent on the heuristic load
    /// tier).
    fn solve_pose(&mut self, pose: &[f64; 6], seed: &[f64; 6]) -> Option<[f64; 6]> {
        let planner = self.ruckig.as_mut()?;

        // ---- Warm attempt: the caller-provided seed (previous joint state). ----
        if let Ok(q) = planner.ik(pose, seed) {
            if let Some(q) = crate::interpreter::normalize_ik_result(q) {
                return Some(q);
            }
        }

        // ---- Cold sweep: $A1 toward the target (both signs) × varied configs. ----
        let seed_variants = cold_solve_seed_variants(pose);
        for variant in &seed_variants {
            if let Ok(q) = planner.ik(pose, variant) {
                if let Some(q) = crate::interpreter::normalize_ik_result(q) {
                    return Some(q);
                }
            }
        }

        None
    }

    /// The active top [`SolverTier`], fixed at load. Delegates to the inherent
    /// [`LayeredTrajectoryProvider::tier`].
    fn tier(&self) -> SolverTier {
        LayeredTrajectoryProvider::tier(self)
    }
}

/// Build the fixed cold-solve seed-variant sweep for [`TrajectoryProvider::solve_pose`],
/// identical to the base system's home-presolve / cold-joint-solve seeds
/// (`server.rs` lines ~373–384, `main.rs` lines ~165–200).
///
/// `pose` is a KUKA target pose `[x, y, z, a, b, c]` with translation in **mm**.
/// The `$A1` base-rotation angle is aimed at the target's XY projection in both
/// senses (`a1_neg` points the arm toward the target from one side, `a1_pos`
/// from the other), and crossed with a small set of varied shoulder / elbow /
/// wrist configurations so a cold solve (no useful warm seed) can still find a
/// branch that normalizes within limits.
fn cold_solve_seed_variants(pose: &[f64; 6]) -> [[f64; 6]; 8] {
    let a1_neg = (-pose[1] / 1000.0).atan2(-pose[0] / 1000.0);
    let a1_pos = (pose[1] / 1000.0).atan2(pose[0] / 1000.0);
    [
        [a1_neg, -1.5, 1.5, 0.0, 1.0, 0.0],
        [a1_pos, -1.5, 1.5, 0.0, 1.0, 0.0],
        [a1_neg, -2.0, 2.5, -1.5, -1.0, 0.0],
        [a1_pos, -2.0, 2.5, -1.5, -1.0, 0.0],
        [a1_neg, -0.5, 0.5, 1.5, 1.5, 0.0],
        [a1_pos, -0.5, 0.5, 1.5, 1.5, 0.0],
        [a1_neg, -1.8, 2.0, 3.0, -1.5, -3.0],
        [a1_pos, -1.8, 2.0, 3.0, -1.5, -3.0],
    ]
}

/// Heuristic motion-duration estimate from KUKA poses (mm / deg), used by the
/// last-resort [`SolverTier::Heuristic`] tier when no joint information is
/// available.
///
/// Reproduces `interpreter::compute_ptp_duration` exactly (Cartesian +
/// rotational travel, `2000 mm/s` / `360 °/s` ceilings scaled by the velocity
/// fraction, `0.2 s` blend / `0.5 s` non-blend accel time, `0.4 s` floor) so the
/// heuristic tier preserves the base system's timing behavior (Requirements 3.4,
/// 7.2). The result is always strictly positive (Requirement 3.2).
fn heuristic_duration(
    start_pose: &[f64; 6],
    target_pose: &[f64; 6],
    vel_percent: f64,
    approximate: bool,
) -> f64 {
    let vel_frac = (vel_percent / 100.0).max(0.01);

    // Translational component (mm).
    let dx = target_pose[0] - start_pose[0];
    let dy = target_pose[1] - start_pose[1];
    let dz = target_pose[2] - start_pose[2];
    let dist_mm = (dx * dx + dy * dy + dz * dz).sqrt();
    let max_tcp_speed = 2000.0; // mm/s at 100%
    let trans_time = dist_mm / (vel_frac * max_tcp_speed);

    // Rotational component — angular distance in degrees.
    let da = pose_angle_diff(start_pose[3], target_pose[3]);
    let db = pose_angle_diff(start_pose[4], target_pose[4]);
    let dc = pose_angle_diff(start_pose[5], target_pose[5]);
    let ang_dist_deg = (da * da + db * db + dc * dc).sqrt();
    let max_rot_speed = 360.0; // °/s at 100%
    let rot_time = ang_dist_deg / (vel_frac * max_rot_speed);

    let move_time = trans_time.max(rot_time);
    let accel_time = if approximate { 0.2 } else { 0.5 };
    (move_time + accel_time).max(0.4)
}

/// Shortest-path absolute angular difference (degrees), mirroring
/// `interpreter::angle_diff`.
fn pose_angle_diff(from: f64, to: f64) -> f64 {
    let mut d = to - from;
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d.abs()
}

/// Compose the single startup tier-report line for [`LayeredTrajectoryProvider::load`].
///
/// Centralizes the three-way branch (Requirements 1.4 / 1.5 / 1.6) so each tier
/// has a distinct, descriptive message and the [`SolverTier::PinocchioIk`]
/// branch exists and is testable even though `load` cannot currently reach it
/// (see the struct doc). `reason` carries the downgrade cause for the
/// non-primary tiers (the unavailable-URDF error for `Heuristic`, the
/// Ruckig-unavailability reason for `PinocchioIk`).
fn startup_report_line(
    tier: SolverTier,
    model_name: &str,
    urdf: &Path,
    reason: Option<&str>,
) -> String {
    match tier {
        SolverTier::RuckigOtg => format!(
            "Trajectory provider [{model_name}]: active tier {} (Ruckig OTG + Pinocchio IK) from {}",
            tier.name(),
            urdf.display(),
        ),
        SolverTier::PinocchioIk => format!(
            "Trajectory provider [{model_name}]: active tier {} — Ruckig unavailable ({}); using Pinocchio IK endpoints from {}",
            tier.name(),
            reason.unwrap_or("reason unspecified"),
            urdf.display(),
        ),
        SolverTier::Heuristic => format!(
            "Trajectory provider [{model_name}]: active tier {} — URDF unavailable at {} ({})",
            tier.name(),
            urdf.display(),
            reason.unwrap_or("reason unspecified"),
        ),
    }
}

#[cfg(test)]
mod layered_provider_tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn heuristic_when_urdf_missing() {
        // A non-existent URDF cannot load → the provider downgrades to the
        // heuristic tier with no Ruckig planner (Req 1.6).
        let missing = Path::new("/nonexistent/does_not_exist.urdf");
        let provider = LayeredTrajectoryProvider::load(missing, "test_model");
        assert_eq!(provider.tier(), SolverTier::Heuristic);
        assert!(!provider.has_ruckig());
        // Limits are still populated (KR240 fallback) for the fallback tiers.
        assert_eq!(*provider.limits(), JointLimits::kr240_fallback());
    }

    #[test]
    fn startup_line_names_active_tier_ruckig() {
        // RuckigOtg report names the tier and the source URDF (Req 1.3, 1.4).
        let line = startup_report_line(
            SolverTier::RuckigOtg,
            "kr240_r2900_ultra",
            Path::new("/env/model/kr240_r2900_ultra.urdf"),
            None,
        );
        assert!(line.contains("Ruckig OTG"));
        assert!(line.contains("kr240_r2900_ultra"));
        assert!(line.contains("kr240_r2900_ultra.urdf"));
    }

    #[test]
    fn startup_line_names_unavailable_urdf_for_heuristic() {
        // Heuristic report names the unavailable URDF and the reason (Req 1.6).
        let line = startup_report_line(
            SolverTier::Heuristic,
            "kr210_r2700_extra",
            Path::new("/env/model/kr210_r2700_extra.urdf"),
            Some("file not found"),
        );
        assert!(line.contains("heuristic timing"));
        assert!(line.contains("kr210_r2700_extra.urdf"));
        assert!(line.contains("file not found"));
    }

    #[test]
    fn startup_line_names_ruckig_reason_for_pinocchio_ik() {
        // PinocchioIk report names why Ruckig is unavailable (Req 1.5). This
        // branch is currently unreachable via `load`, but the message exists.
        let line = startup_report_line(
            SolverTier::PinocchioIk,
            "kr270_r2700_ultra",
            Path::new("/env/model/kr270_r2700_ultra.urdf"),
            Some("Ruckig tier disabled"),
        );
        assert!(line.contains("Pinocchio IK"));
        assert!(line.contains("Ruckig unavailable"));
        assert!(line.contains("Ruckig tier disabled"));
    }

    // ---- resolve_motion fallback chain (task 5.3) ----

    fn req(kind: MotionKind) -> MotionRequest {
        MotionRequest {
            kind,
            start_pose: [1500.0, 0.0, 1800.0, 0.0, 90.0, 0.0],
            target_pose: [1200.0, 300.0, 1600.0, 10.0, 90.0, 5.0],
            vel_percent: 100.0,
            approximate: false,
            seed_joints: [0.0, -1.5, 1.5, 0.0, 1.0, 0.0],
        }
    }

    #[test]
    fn resolve_motion_on_heuristic_tier_is_always_usable() {
        // With no URDF the provider is on the heuristic tier (no planner / no
        // Pinocchio model). resolve_motion must still return a usable solution:
        // duration_s > 0, JointPath::None, tier Heuristic, and a downgrade note
        // naming the reason (Req 3.2, 3.4, 3.6, 4.6, 12.1).
        let mut provider =
            LayeredTrajectoryProvider::load(Path::new("/nonexistent/x.urdf"), "test_model");
        assert_eq!(provider.tier(), SolverTier::Heuristic);

        for kind in [MotionKind::Ptp, MotionKind::Lin] {
            let sol = provider.resolve_motion(&req(kind));
            assert!(sol.duration_s > 0.0, "duration must be positive");
            assert!(matches!(sol.path, JointPath::None));
            assert_eq!(sol.tier, SolverTier::Heuristic);
            let note = sol.note.expect("heuristic tier records a downgrade note");
            assert!(note.contains("Ruckig OTG tier unavailable"), "note: {note}");
        }
    }

    #[test]
    fn solve_pose_on_heuristic_tier_returns_none() {
        // No planner ⇒ no tier can resolve a pose (Req 1.7 edge: returns None).
        let mut provider =
            LayeredTrajectoryProvider::load(Path::new("/nonexistent/x.urdf"), "test_model");
        let pose = [1500.0, 0.0, 1800.0, 0.0, 90.0, 0.0];
        let seed = [0.0, -1.5, 1.5, 0.0, 1.0, 0.0];
        assert!(provider.solve_pose(&pose, &seed).is_none());
    }

    #[test]
    fn cold_solve_seeds_match_base_system_constants() {
        // Cold-solve sweep must reproduce the base system's home-presolve /
        // cold-joint-solve seeds (server.rs ~373–384) for parity (Req 1.7).
        let pose: [f64; 6] = [1200.0, 800.0, 1700.0, 10.0, 90.0, -30.0];
        let a1_neg = (-pose[1] / 1000.0).atan2(-pose[0] / 1000.0);
        let a1_pos = (pose[1] / 1000.0).atan2(pose[0] / 1000.0);
        let expected: [[f64; 6]; 8] = [
            [a1_neg, -1.5, 1.5, 0.0, 1.0, 0.0],
            [a1_pos, -1.5, 1.5, 0.0, 1.0, 0.0],
            [a1_neg, -2.0, 2.5, -1.5, -1.0, 0.0],
            [a1_pos, -2.0, 2.5, -1.5, -1.0, 0.0],
            [a1_neg, -0.5, 0.5, 1.5, 1.5, 0.0],
            [a1_pos, -0.5, 0.5, 1.5, 1.5, 0.0],
            [a1_neg, -1.8, 2.0, 3.0, -1.5, -3.0],
            [a1_pos, -1.8, 2.0, 3.0, -1.5, -3.0],
        ];
        let got = cold_solve_seed_variants(&pose);
        assert_eq!(got, expected);
        // Both A1 senses are present and the sweep is non-empty.
        assert!((got[0][0] - a1_neg).abs() < 1e-12);
        assert!((got[1][0] - a1_pos).abs() < 1e-12);
    }

    #[test]
    fn heuristic_duration_is_positive_and_matches_base_arithmetic() {
        // Zero travel collapses to the 0.4 s floor (move_time 0 + 0.5 accel,
        // floored at 0.4 ⇒ 0.5). Approximate uses the smaller 0.2 accel time.
        let p = [1000.0, 0.0, 1500.0, 0.0, 90.0, 0.0];
        assert!((heuristic_duration(&p, &p, 100.0, false) - 0.5).abs() < 1e-9);
        assert!((heuristic_duration(&p, &p, 100.0, true) - 0.4).abs() < 1e-9);

        // A pure 2000 mm translation at 100% takes 1.0 s of travel + 0.5 accel.
        let q = [3000.0, 0.0, 1500.0, 0.0, 90.0, 0.0];
        assert!((heuristic_duration(&p, &q, 100.0, false) - 1.5).abs() < 1e-9);

        // Always strictly positive even at vanishing velocity percent.
        assert!(heuristic_duration(&p, &q, 0.0, false) > 0.0);
    }

    #[test]
    fn pose_angle_diff_takes_shortest_wraparound() {
        assert!((pose_angle_diff(170.0, -170.0) - 20.0).abs() < 1e-9);
        assert!((pose_angle_diff(-170.0, 170.0) - 20.0).abs() < 1e-9);
        assert!((pose_angle_diff(0.0, 90.0) - 90.0).abs() < 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Optional trajectory-provider validation hook (task 10.1)
//
// Feature-gated behind `collision` so the default build is entirely unaffected:
// the `crate::collision` module itself only exists under `#[cfg(feature =
// "collision")]` (see `main.rs`), so any reference to its types must be gated
// too. When the feature is off this block does not compile at all and the
// provider behaves exactly as before.
// ---------------------------------------------------------------------------

/// Optional, opt-in validation of a provider-produced trajectory against the
/// scene collision checker (design: the `PROV -. "optional validation" .-> SCENE`
/// edge — an *optional trajectory-provider validation hook*).
///
/// This is a thin, non-mutating wrapper over
/// [`SceneCollisionChecker::check_trajectory`](crate::collision::SceneCollisionChecker::check_trajectory):
/// it takes the already-resolved, provider-produced [`JointPath`]s carried in a
/// [`SceneTrajectory`](crate::collision::SceneTrajectory) and returns the
/// discrete-trajectory [`TrajectoryCollisionReport`](crate::collision::TrajectoryCollisionReport).
/// It exists so a caller that owns a trajectory provider can run detection-only
/// validation over its output without wiring the collision layer directly.
///
/// # Never-fail-hard contract (Req 9.1, 9.2)
///
/// The hook preserves the provider chain's "never fail hard" contract and adds
/// nothing that could break it:
///
/// * **No mutation.** `traj` is borrowed immutably, so the provider-produced
///   trajectory is byte-for-byte identical before and after the call (Req 9.2).
///   The subsystem is detection-and-reporting only and never re-plans or edits a
///   trajectory.
/// * **No panic; FFI failures degrade to a reported error.**
///   [`check_trajectory`](crate::collision::SceneCollisionChecker::check_trajectory)
///   is itself total: any per-model FFI/collision error is recorded on the
///   returned report (the affected model is marked *errored* while the remaining
///   models are still evaluated) rather than surfacing as a panic or an
///   unrecoverable error (Req 9.1). This wrapper adds no fallible step of its
///   own, so a collision-layer failure can never block trajectory generation —
///   it is only ever *reported*.
///
/// The returned report is advisory: callers inspect
/// [`TrajectoryCollisionReport`](crate::collision::TrajectoryCollisionReport)
/// (its `collided` flag, first/all colliding waypoints, and any per-model error
/// indications) and decide what to do; the provider's own output is unchanged.
#[cfg(feature = "collision")]
pub fn validate_trajectory_collisions(
    scene: &mut crate::collision::SceneCollisionChecker,
    traj: &crate::collision::SceneTrajectory,
) -> crate::collision::TrajectoryCollisionReport {
    // Delegate verbatim: `check_trajectory` borrows the trajectory immutably
    // (no mutation, Req 9.2) and never panics, recording any FFI error on the
    // report instead (never-fail-hard, Req 9.1).
    scene.check_trajectory(traj)
}
