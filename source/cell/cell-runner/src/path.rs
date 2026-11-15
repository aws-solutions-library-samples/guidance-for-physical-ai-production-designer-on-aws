//! Trajectory carrier for the layered trajectory provider.
//!
//! `JointPath` unifies all three solver tiers behind a single `sample(progress)`
//! so the interpreter's per-cycle interpolation stays tier-agnostic:
//!   * `Dense`     — Crocoddyl: time-uniform sampled knots `q[0..=N]` (radians).
//!   * `Endpoints` — Pinocchio IK: two endpoints, linearly interpolated in joint
//!                   space (today's behavior, mirrors `interpreter::interpolate_joints`).
//!   * `None`      — heuristic: no joint info; interpolation falls back to Cartesian.
//!
//! The within-limits validation predicate reuses `interpreter::normalize_joints`
//! so it stays in exact parity with the base system's position-limit check
//! (base spec Requirement 11.4). Limit arrays and slack are passed in as
//! parameters so this module does not depend on a concrete `JointLimits` type
//! (which lives in `limits.rs`, task 2.1) and stays decoupled.

#![allow(dead_code)]

use crate::interpreter::normalize_joints;
use crate::limits::JointLimits;

/// Relative margin applied to the velocity/acceleration/jerk feasibility bounds.
///
/// A jerk-limited Ruckig profile is limit-bounded on the *continuous*
/// trajectory; each per-takt divided difference is the *average* of the
/// corresponding derivative over its window, so it cannot exceed the limit
/// except by floating-point rounding at the boundary (the time-synchronized
/// limiting axis cruises at exactly the velocity limit). `1e-6` is many orders
/// of magnitude above that rounding yet far below any physically meaningful
/// violation — especially since the per-model limits here are the conservative
/// KR240 fallback set, well below the robot's declared axis limits.
pub const FEASIBILITY_REL_TOL: f64 = 1e-6;

/// Absolute feasibility margin (radians / rad·s⁻¹ / …) guarding the degenerate
/// near-zero-limit case so the relative margin never collapses to zero.
pub const FEASIBILITY_ABS_TOL: f64 = 1e-9;

/// The trajectory carrier stored on `MotionState`.
#[derive(Debug, Clone)]
pub enum JointPath {
    /// Ruckig OTG: dense, time-uniform samples `q[0..=N]` (radians), one per
    /// `takt_s` (the KRC4 takt, `KRC4_TIMESTEP`). Always limit-valid.
    Dense { qs: Vec<[f64; 6]>, takt_s: f64 },
    /// Pinocchio IK: two endpoints, linearly interpolated in joint space.
    Endpoints { start: [f64; 6], end: [f64; 6] },
    /// Heuristic: no joint info; interpolation falls back to Cartesian-space.
    None,
}

impl JointPath {
    /// Sample joint angles at the given `progress`.
    ///
    /// `progress` is clamped to `[0, 1]`. Returns `None` only for the `None`
    /// variant (and for a `Dense` path that carries no knots).
    ///
    /// * `Dense`     — piecewise-linear interpolation between the two nearest
    ///                 knots, locating the segment via `progress * (knots - 1)`.
    /// * `Endpoints` — linear interpolation `start + (end - start) * progress`,
    ///                 matching `interpreter::interpolate_joints`.
    /// * `None`      — `None`.
    pub fn sample(&self, progress: f64) -> Option<[f64; 6]> {
        let p = progress.clamp(0.0, 1.0);
        match self {
            JointPath::Dense { qs, .. } => sample_dense(qs, p),
            JointPath::Endpoints { start, end } => Some(lerp_joints(start, end, p)),
            JointPath::None => Option::None,
        }
    }

    /// `true` for `Dense` or `Endpoints` (a path that carries joint information).
    pub fn is_some(&self) -> bool {
        matches!(self, JointPath::Dense { .. } | JointPath::Endpoints { .. })
    }

    /// Within-limits validation: every joint configuration this path can produce
    /// satisfies the per-axis position limits with `slack`.
    ///
    /// Mirrors the base system's `joints_within_limits` (normalize, then compare
    /// against `[lower - slack, upper + slack]`), but accepts the limit arrays as
    /// parameters so the module stays decoupled from a concrete limits type.
    ///
    /// * `Dense`     — checked at every knot.
    /// * `Endpoints` — checked at both `start` and `end`.
    /// * `None`      — trivially `true` (no joint info to validate).
    pub fn within_limits(&self, q_lower: &[f64; 6], q_upper: &[f64; 6], slack: f64) -> bool {
        match self {
            JointPath::Dense { qs, .. } => qs
                .iter()
                .all(|q| position_within_limits(q, q_lower, q_upper, slack)),
            JointPath::Endpoints { start, end } => {
                position_within_limits(start, q_lower, q_upper, slack)
                    && position_within_limits(end, q_lower, q_upper, slack)
            }
            JointPath::None => true,
        }
    }

    /// Velocity-feasibility validation for `Dense` paths: every pair of
    /// consecutive knots satisfies the per-axis velocity limit scaled by the
    /// commanded velocity fraction and the integration step:
    /// `|q[k+1,i] - q[k,i]| <= vel_frac * v_abs[i] * dt`.
    ///
    /// `vel_frac` is the commanded velocity as a fraction in `[0, 1]`
    /// (i.e. `vel_percent / 100`). Non-`Dense` variants are trivially feasible.
    pub fn velocity_feasible(&self, vel_frac: f64, v_abs: &[f64; 6], dt: f64) -> bool {
        match self {
            JointPath::Dense { qs, .. } => knots_velocity_feasible(qs, vel_frac, v_abs, dt),
            JointPath::Endpoints { .. } | JointPath::None => true,
        }
    }

    /// Acceleration-feasibility validation for `Dense` paths: every per-axis
    /// finite-difference acceleration across three consecutive knots respects the
    /// absolute acceleration limit `limits.a_abs`:
    /// `|q[k+2,i] - 2·q[k+1,i] + q[k,i]| / takt_s² <= a_abs[i]`.
    ///
    /// `takt_s` is the knot spacing (the KRC4 takt, `KRC4_TIMESTEP`). A `Dense`
    /// path with fewer than three knots has no defined finite-difference
    /// acceleration and is trivially feasible, as are the non-`Dense` variants.
    pub fn acceleration_feasible(&self, takt_s: f64, limits: &JointLimits) -> bool {
        match self {
            JointPath::Dense { qs, .. } => knots_acceleration_feasible(qs, takt_s, &limits.a_abs),
            JointPath::Endpoints { .. } | JointPath::None => true,
        }
    }

    /// Jerk-feasibility validation for `Dense` paths: every per-axis
    /// finite-difference jerk across four consecutive knots respects the
    /// absolute jerk limit `limits.j_abs`:
    /// `|q[k+3,i] - 3·q[k+2,i] + 3·q[k+1,i] - q[k,i]| / takt_s³ <= j_abs[i]`.
    ///
    /// This is the anti-stutter guarantee that a non-empty jerk bound produces a
    /// jerk-limited S-curve rather than a trapezoidal step-velocity profile.
    ///
    /// `takt_s` is the knot spacing (the KRC4 takt, `KRC4_TIMESTEP`). A `Dense`
    /// path with fewer than four knots has no defined finite-difference jerk and
    /// is trivially feasible, as are the non-`Dense` variants.
    pub fn jerk_feasible(&self, takt_s: f64, limits: &JointLimits) -> bool {
        match self {
            JointPath::Dense { qs, .. } => knots_jerk_feasible(qs, takt_s, &limits.j_abs),
            JointPath::Endpoints { .. } | JointPath::None => true,
        }
    }
}

/// Piecewise-linear sample of a dense knot sequence at clamped progress `p ∈ [0, 1]`.
fn sample_dense(qs: &[[f64; 6]], p: f64) -> Option<[f64; 6]> {
    match qs.len() {
        0 => None,
        1 => Some(qs[0]),
        len => {
            let n_seg = (len - 1) as f64;
            let pos = p * n_seg;
            let lower = pos.floor() as usize;
            if lower >= len - 1 {
                // p == 1.0 (or numerical overshoot) → last knot.
                return Some(qs[len - 1]);
            }
            let frac = pos - lower as f64;
            Some(lerp_joints(&qs[lower], &qs[lower + 1], frac))
        }
    }
}

/// Component-wise linear interpolation, matching `interpreter::interpolate_joints`.
fn lerp_joints(from: &[f64; 6], to: &[f64; 6], t: f64) -> [f64; 6] {
    let mut result = [0.0f64; 6];
    for i in 0..6 {
        result[i] = from[i] + (to[i] - from[i]) * t;
    }
    result
}

/// Position-limit predicate with slack, parameterized by the limit arrays.
///
/// Normalizes wide-range joints first (parity with `interpreter::joints_within_limits`).
pub fn position_within_limits(
    q: &[f64; 6],
    q_lower: &[f64; 6],
    q_upper: &[f64; 6],
    slack: f64,
) -> bool {
    let mut j = *q;
    normalize_joints(&mut j);
    for i in 0..6 {
        if j[i] < q_lower[i] - slack || j[i] > q_upper[i] + slack {
            return false;
        }
    }
    true
}

/// Velocity-feasibility predicate over a dense knot sequence.
///
/// Every consecutive pair satisfies `|q[k+1,i] - q[k,i]| <= vel_frac * v_abs[i] * dt`
/// for all axes. A sequence with fewer than two knots is trivially feasible.
///
/// A small [`FEASIBILITY_REL_TOL`] margin is applied to the bound. A
/// jerk-limited Ruckig profile is velocity-bounded on the *continuous*
/// trajectory, and the per-takt divided difference equals the *average* velocity
/// over the takt (≤ the continuous peak ≤ the limit). It can therefore only
/// exceed `vel_frac·v_abs·dt` by floating-point rounding at the boundary — the
/// time-synchronized limiting axis cruises at *exactly* the limit — so a strict
/// `>` comparison spuriously rejects otherwise-valid paths and forces a needless
/// tier downgrade. The relative margin absorbs that rounding without admitting
/// any physically meaningful violation.
pub fn knots_velocity_feasible(qs: &[[f64; 6]], vel_frac: f64, v_abs: &[f64; 6], dt: f64) -> bool {
    for pair in qs.windows(2) {
        for i in 0..6 {
            let dq = (pair[1][i] - pair[0][i]).abs();
            let bound = vel_frac * v_abs[i] * dt;
            if dq > bound * (1.0 + FEASIBILITY_REL_TOL) + FEASIBILITY_ABS_TOL {
                return false;
            }
        }
    }
    true
}

/// Acceleration-feasibility predicate over a dense knot sequence.
///
/// Uses the central second difference of position to estimate per-axis
/// acceleration across each window of three consecutive knots:
/// `a[k,i] = (q[k+2,i] - 2·q[k+1,i] + q[k,i]) / takt_s²`, and requires
/// `|a[k,i]| <= a_abs[i]` for every axis. A sequence with fewer than three knots
/// has no defined finite-difference acceleration and is trivially feasible.
pub fn knots_acceleration_feasible(qs: &[[f64; 6]], takt_s: f64, a_abs: &[f64; 6]) -> bool {
    let takt_sq = takt_s * takt_s;
    for w in qs.windows(3) {
        for i in 0..6 {
            let accel = (w[2][i] - 2.0 * w[1][i] + w[0][i]) / takt_sq;
            // Same boundary-rounding margin as the velocity check: the second
            // divided difference is the average acceleration over the window
            // (≤ a_abs on a Ruckig profile), so any excess is float rounding.
            if accel.abs() > a_abs[i] * (1.0 + FEASIBILITY_REL_TOL) + FEASIBILITY_ABS_TOL {
                return false;
            }
        }
    }
    true
}

/// Jerk-feasibility predicate over a dense knot sequence.
///
/// Uses the third difference of position to estimate per-axis jerk across each
/// window of four consecutive knots:
/// `j[k,i] = (q[k+3,i] - 3·q[k+2,i] + 3·q[k+1,i] - q[k,i]) / takt_s³`, and
/// requires `|j[k,i]| <= j_abs[i]` for every axis. A sequence with fewer than
/// four knots has no defined finite-difference jerk and is trivially feasible.
pub fn knots_jerk_feasible(qs: &[[f64; 6]], takt_s: f64, j_abs: &[f64; 6]) -> bool {
    let takt_cube = takt_s * takt_s * takt_s;
    for w in qs.windows(4) {
        for i in 0..6 {
            let jerk = (w[3][i] - 3.0 * w[2][i] + 3.0 * w[1][i] - w[0][i]) / takt_cube;
            // Same boundary-rounding margin as the velocity/acceleration checks.
            if jerk.abs() > j_abs[i] * (1.0 + FEASIBILITY_REL_TOL) + FEASIBILITY_ABS_TOL {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tolerance for comparing interpolated f64 joint values.
    const EPS: f64 = 1e-12;

    /// Assert two joint arrays are equal within `EPS` component-wise.
    fn assert_joints_close(actual: [f64; 6], expected: [f64; 6]) {
        for i in 0..6 {
            assert!(
                (actual[i] - expected[i]).abs() <= EPS,
                "axis {i}: actual {} != expected {} (|diff| = {})",
                actual[i],
                expected[i],
                (actual[i] - expected[i]).abs()
            );
        }
    }

    // Distinct, easily-checked knots for the Dense tests.
    const K0: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    const K1: [f64; 6] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    const K2: [f64; 6] = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0];

    fn three_knot_dense() -> JointPath {
        JointPath::Dense {
            qs: vec![K0, K1, K2],
            takt_s: 0.012,
        }
    }

    // ---- Dense -----------------------------------------------------------

    #[test]
    fn dense_progress_zero_returns_first_knot() {
        let path = three_knot_dense();
        assert_joints_close(path.sample(0.0).unwrap(), K0);
    }

    #[test]
    fn dense_progress_one_returns_last_knot() {
        let path = three_knot_dense();
        assert_joints_close(path.sample(1.0).unwrap(), K2);
    }

    #[test]
    fn dense_progress_lands_exactly_on_interior_knot() {
        // With 3 knots (2 segments), progress 0.5 maps to pos = 1.0 → interior knot K1.
        let path = three_knot_dense();
        assert_joints_close(path.sample(0.5).unwrap(), K1);
    }

    #[test]
    fn dense_fractional_progress_interpolates_first_segment() {
        // progress 0.25 → pos 0.5 → halfway between K0 and K1.
        let path = three_knot_dense();
        let expected = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
        assert_joints_close(path.sample(0.25).unwrap(), expected);
    }

    #[test]
    fn dense_fractional_progress_interpolates_second_segment() {
        // progress 0.75 → pos 1.5 → halfway between K1 and K2.
        let path = three_knot_dense();
        let expected = [1.5, 3.0, 4.5, 6.0, 7.5, 9.0];
        assert_joints_close(path.sample(0.75).unwrap(), expected);
    }

    #[test]
    fn dense_out_of_range_progress_clamps_to_endpoints() {
        let path = three_knot_dense();
        // Below 0 clamps to first knot; above 1 clamps to last knot.
        assert_joints_close(path.sample(-0.5).unwrap(), K0);
        assert_joints_close(path.sample(-100.0).unwrap(), K0);
        assert_joints_close(path.sample(1.5).unwrap(), K2);
        assert_joints_close(path.sample(42.0).unwrap(), K2);
    }

    #[test]
    fn dense_single_knot_returns_that_knot_for_any_progress() {
        let path = JointPath::Dense { qs: vec![K1], takt_s: 0.012 };
        assert_joints_close(path.sample(0.0).unwrap(), K1);
        assert_joints_close(path.sample(0.37).unwrap(), K1);
        assert_joints_close(path.sample(1.0).unwrap(), K1);
        assert_joints_close(path.sample(-5.0).unwrap(), K1);
        assert_joints_close(path.sample(5.0).unwrap(), K1);
    }

    #[test]
    fn dense_empty_returns_none() {
        let path = JointPath::Dense { qs: vec![], takt_s: 0.012 };
        assert!(path.sample(0.0).is_none());
        assert!(path.sample(0.5).is_none());
        assert!(path.sample(1.0).is_none());
    }

    // ---- Endpoints -------------------------------------------------------

    const START: [f64; 6] = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0];
    const END: [f64; 6] = [20.0, 40.0, 60.0, 80.0, 100.0, 120.0];

    fn endpoints() -> JointPath {
        JointPath::Endpoints {
            start: START,
            end: END,
        }
    }

    #[test]
    fn endpoints_progress_zero_returns_start() {
        assert_joints_close(endpoints().sample(0.0).unwrap(), START);
    }

    #[test]
    fn endpoints_progress_one_returns_end() {
        assert_joints_close(endpoints().sample(1.0).unwrap(), END);
    }

    #[test]
    fn endpoints_progress_half_returns_midpoint() {
        let expected = [15.0, 30.0, 45.0, 60.0, 75.0, 90.0];
        assert_joints_close(endpoints().sample(0.5).unwrap(), expected);
    }

    #[test]
    fn endpoints_out_of_range_progress_clamps() {
        // Negative progress clamps to start; progress > 1 clamps to end.
        assert_joints_close(endpoints().sample(-0.25).unwrap(), START);
        assert_joints_close(endpoints().sample(-3.0).unwrap(), START);
        assert_joints_close(endpoints().sample(2.0).unwrap(), END);
        assert_joints_close(endpoints().sample(1.0001).unwrap(), END);
    }

    // ---- None ------------------------------------------------------------

    #[test]
    fn none_always_returns_none() {
        let path = JointPath::None;
        assert!(path.sample(0.0).is_none());
        assert!(path.sample(0.5).is_none());
        assert!(path.sample(1.0).is_none());
        assert!(path.sample(-1.0).is_none());
        assert!(path.sample(2.0).is_none());
    }

    // ---- is_some() truth table ------------------------------------------

    #[test]
    fn is_some_truth_table() {
        assert!(three_knot_dense().is_some(), "non-empty Dense carries joint info");
        // is_some() keys off the variant, not knot count.
        assert!(
            JointPath::Dense { qs: vec![], takt_s: 0.012 }.is_some(),
            "empty Dense is still the Dense variant"
        );
        assert!(endpoints().is_some(), "Endpoints carries joint info");
        assert!(!JointPath::None.is_some(), "None carries no joint info");
    }
}
