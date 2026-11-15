//! Per-model joint limit set used to enforce Crocoddyl box constraints and to
//! validate any produced joint path.
//!
//! The hard-coded fallback constants mirror the `KR240 R2900` limits already
//! defined in [`crate::interpreter`] (`JOINT_POS_LIMITS` / `JOINT_VEL_LIMITS_RAD`).
//! When a robot model's URDF exposes its true limits, those are used instead and
//! any missing limit falls back to the corresponding constant, so the four
//! distinct robot models each get their real limits while staying parity-safe.

/// KR240 R2900 joint position lower limits (radians) — mirrors
/// `interpreter::JOINT_POS_LIMITS.*.0`.
pub const KR240_Q_LOWER: [f64; 6] = [-3.2289, -2.4435, -2.0944, -6.1087, -2.1380, -6.1087];

/// KR240 R2900 joint position upper limits (radians) — mirrors
/// `interpreter::JOINT_POS_LIMITS.*.1`.
pub const KR240_Q_UPPER: [f64; 6] = [3.2289, -0.0873, 2.9322, 6.1087, 2.1380, 6.1087];

/// KR240 R2900 absolute joint velocity limits (rad/s) — mirrors
/// `interpreter::JOINT_VEL_LIMITS_RAD`.
pub const KR240_V_ABS: [f64; 6] = [1.9199, 1.7453, 1.9199, 3.7525, 3.2289, 4.5379];

/// KR240 R2900 absolute control effort (torque) limits (N·m), taken from the
/// `<limit effort=...>` tags of `kr240_r2900_ultra.urdf`. Used as the fallback
/// control box when a model's URDF does not expose effort limits.
pub const KR240_U_ABS: [f64; 6] = [
    7.29e3,
    9.24e3,
    7.29542236549771e3,
    2.3237103673905e3,
    2.32603308821414e3,
    1.49286251041991e3,
];

/// KR240 R2900 absolute joint acceleration limits (rad/s²). URDF rarely
/// expresses acceleration, so these per-axis fallback constants are used to
/// configure Ruckig's `max_acceleration` input. The major axes sit around
/// 10 rad/s² (≈ reaching `v_abs` in a fraction of a second) and the lighter
/// wrist axes accelerate faster, mirroring the `add-ruckig.md` guidance.
pub const KR240_A_ABS: [f64; 6] = [10.0, 9.0, 10.0, 19.0, 16.0, 23.0];

/// KR240 R2900 absolute joint jerk limits (rad/s³). URDF does not express jerk,
/// so these per-axis fallback constants configure Ruckig's `max_jerk` input.
/// Every value lies inside the physical 20–150 rad/s³ band (Req 5.4): empty or
/// zero jerk bounds would degenerate Ruckig into a trapezoidal, step-velocity
/// profile (the #1 stutter cause), so a non-empty in-band jerk profile is used.
pub const KR240_J_ABS: [f64; 6] = [100.0, 100.0, 100.0, 140.0, 140.0, 150.0];

/// Position-limit slack (radians) applied by the base system's
/// `joints_within_limits` predicate.
pub const POS_SLACK: f64 = 0.05;

/// Per-axis joint position, velocity, and effort limits for a single robot model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimits {
    /// Lower joint position limits (radians). Mirrors `JOINT_POS_LIMITS.*.0`.
    pub q_lower: [f64; 6],
    /// Upper joint position limits (radians). Mirrors `JOINT_POS_LIMITS.*.1`.
    pub q_upper: [f64; 6],
    /// Absolute per-axis velocity limits (rad/s). Mirrors `JOINT_VEL_LIMITS_RAD`.
    pub v_abs: [f64; 6],
    /// Absolute per-axis control effort/torque box (N·m), from URDF `<limit effort=...>`.
    pub u_abs: [f64; 6],
    /// Absolute per-axis acceleration limits (rad/s²). Required by Ruckig's
    /// `max_acceleration` input; URDF rarely expresses acceleration so this
    /// falls back to [`KR240_A_ABS`].
    pub a_abs: [f64; 6],
    /// Absolute per-axis jerk limits (rad/s³). Required by Ruckig's `max_jerk`
    /// input; URDF does not express jerk so this falls back to [`KR240_J_ABS`],
    /// whose values lie within the physical 20–150 rad/s³ band.
    pub j_abs: [f64; 6],
    /// Position-limit slack (radians) matching `joints_within_limits`.
    pub pos_slack: f64,
}

/// Per-model URDF-derived limits, as optionally exposed by a loaded model. Each
/// field is `None` when the limit could not be read for that model, in which
/// case the hard-coded fallback constant is used for that limit.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct UrdfLimits {
    /// Lower joint position limits (radians), `model.lowerPositionLimit`.
    pub q_lower: Option<[f64; 6]>,
    /// Upper joint position limits (radians), `model.upperPositionLimit`.
    pub q_upper: Option<[f64; 6]>,
    /// Per-axis velocity limits (rad/s), `model.velocityLimit`.
    pub v_abs: Option<[f64; 6]>,
    /// Per-axis effort/torque limits (N·m), `model.effortLimit`.
    pub u_abs: Option<[f64; 6]>,
    /// Per-axis acceleration limits (rad/s²). URDF rarely expresses these; when
    /// `None` the [`KR240_A_ABS`] fallback constants are used.
    pub a_abs: Option<[f64; 6]>,
    /// Per-axis jerk limits (rad/s³). URDF does not express jerk; when `None`
    /// the [`KR240_J_ABS`] fallback constants (within 20–150 rad/s³) are used.
    pub j_abs: Option<[f64; 6]>,
}

impl JointLimits {
    /// Hard-coded `KR240 R2900` fallback limits, identical to the constants the
    /// base interpreter applies.
    pub fn kr240_fallback() -> Self {
        Self {
            q_lower: KR240_Q_LOWER,
            q_upper: KR240_Q_UPPER,
            v_abs: KR240_V_ABS,
            u_abs: KR240_U_ABS,
            a_abs: KR240_A_ABS,
            j_abs: KR240_J_ABS,
            pos_slack: POS_SLACK,
        }
    }

    /// Alias for [`JointLimits::kr240_fallback`].
    pub fn fallback() -> Self {
        Self::kr240_fallback()
    }

    /// Build a limit set from a loaded model's URDF limits, falling back to the
    /// hard-coded constants for any missing or invalid limit.
    ///
    /// Position limits are taken element-wise from the URDF when finite;
    /// velocity and effort limits are taken element-wise when finite and
    /// strictly positive. Any axis whose URDF value is missing or invalid uses
    /// the corresponding fallback constant, so a partially-specified URDF still
    /// yields a complete, valid limit set.
    ///
    /// Acceleration (`a_abs`) and jerk (`j_abs`) are required by Ruckig but are
    /// rarely (acceleration) or never (jerk) expressed in URDF, so they fall
    /// back to [`KR240_A_ABS`] / [`KR240_J_ABS`] unless a model supplies finite,
    /// strictly-positive overrides. The jerk fallbacks lie within the physical
    /// 20–150 rad/s³ band so the produced profile stays a jerk-limited S-curve.
    pub fn from_urdf_limits(urdf: &UrdfLimits) -> Self {
        Self {
            q_lower: merge_position(urdf.q_lower, &KR240_Q_LOWER),
            q_upper: merge_position(urdf.q_upper, &KR240_Q_UPPER),
            v_abs: merge_positive(urdf.v_abs, &KR240_V_ABS),
            u_abs: merge_positive(urdf.u_abs, &KR240_U_ABS),
            a_abs: merge_positive(urdf.a_abs, &KR240_A_ABS),
            j_abs: merge_positive(urdf.j_abs, &KR240_J_ABS),
            pos_slack: POS_SLACK,
        }
    }
}

impl Default for JointLimits {
    fn default() -> Self {
        Self::kr240_fallback()
    }
}

/// Merge a URDF position-limit array with a fallback, keeping finite URDF values
/// and substituting the fallback for any missing or non-finite axis.
fn merge_position(urdf: Option<[f64; 6]>, fallback: &[f64; 6]) -> [f64; 6] {
    let mut out = *fallback;
    if let Some(vals) = urdf {
        for i in 0..6 {
            if vals[i].is_finite() {
                out[i] = vals[i];
            }
        }
    }
    out
}

/// Merge a URDF magnitude-limit array (velocity/effort) with a fallback, keeping
/// finite, strictly-positive URDF values and substituting the fallback otherwise.
fn merge_positive(urdf: Option<[f64; 6]>, fallback: &[f64; 6]) -> [f64; 6] {
    let mut out = *fallback;
    if let Some(vals) = urdf {
        for i in 0..6 {
            if vals[i].is_finite() && vals[i] > 0.0 {
                out[i] = vals[i];
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tight tolerance for exact float comparison of limit values that are
    /// either copied verbatim from the input arrays or from the fallback
    /// constants (no arithmetic is performed, so this is effectively exact).
    const EPS: f64 = 1e-12;

    /// Assert two `[f64; 6]` arrays are element-wise equal within `EPS`.
    fn assert_arr_eq(actual: &[f64; 6], expected: &[f64; 6]) {
        for i in 0..6 {
            assert!(
                (actual[i] - expected[i]).abs() <= EPS,
                "axis {i}: actual {} != expected {}",
                actual[i],
                expected[i]
            );
        }
    }

    /// Representative, deliberately distinct per-model limit sets. These do not
    /// have to match the physical robots exactly; the point is that each model
    /// presents a *different* set of URDF limits and the extraction must carry
    /// each one through faithfully rather than collapsing to the hard-coded
    /// KR240 constants (Req 5.1, 5.2).
    fn model_limits(tag: f64) -> UrdfLimits {
        // Offsetting every axis by a model-specific `tag` makes the four models'
        // arrays mutually distinct while keeping the values plausible/finite.
        let q_lower = [
            -3.0 - tag,
            -2.4 - tag,
            -2.0 - tag,
            -6.1 - tag,
            -2.1 - tag,
            -6.1 - tag,
        ];
        let q_upper = [
            3.0 + tag,
            -0.05 + tag,
            2.9 + tag,
            6.1 + tag,
            2.1 + tag,
            6.1 + tag,
        ];
        let v_abs = [
            1.5 + tag,
            1.6 + tag,
            1.7 + tag,
            3.0 + tag,
            3.1 + tag,
            4.0 + tag,
        ];
        let u_abs = [
            5.0e3 + tag,
            6.0e3 + tag,
            7.0e3 + tag,
            2.0e3 + tag,
            2.1e3 + tag,
            1.4e3 + tag,
        ];
        UrdfLimits {
            q_lower: Some(q_lower),
            q_upper: Some(q_upper),
            v_abs: Some(v_abs),
            u_abs: Some(u_abs),
            a_abs: None,
            j_abs: None,
        }
    }

    #[test]
    fn fallback_matches_kr240_constants() {
        // Req 5.3: the hard-coded fallback set is exactly the KR240 constants.
        let fb = JointLimits::kr240_fallback();
        assert_arr_eq(&fb.q_lower, &KR240_Q_LOWER);
        assert_arr_eq(&fb.q_upper, &KR240_Q_UPPER);
        assert_arr_eq(&fb.v_abs, &KR240_V_ABS);
        assert_arr_eq(&fb.u_abs, &KR240_U_ABS);
        assert!((fb.pos_slack - 0.05).abs() <= EPS);
        assert!((fb.pos_slack - POS_SLACK).abs() <= EPS);
    }

    #[test]
    fn default_equals_kr240_fallback() {
        // `Default` and `fallback()` are aliases of `kr240_fallback()`.
        assert_eq!(JointLimits::default(), JointLimits::kr240_fallback());
        assert_eq!(JointLimits::fallback(), JointLimits::kr240_fallback());
    }

    #[test]
    fn all_fields_present_use_urdf_values() {
        // Req 5.2: when the URDF exposes every limit, the extracted set uses the
        // URDF values for every axis and none of the fallback constants.
        let urdf = model_limits(0.0);
        let limits = JointLimits::from_urdf_limits(&urdf);

        assert_arr_eq(&limits.q_lower, &urdf.q_lower.unwrap());
        assert_arr_eq(&limits.q_upper, &urdf.q_upper.unwrap());
        assert_arr_eq(&limits.v_abs, &urdf.v_abs.unwrap());
        assert_arr_eq(&limits.u_abs, &urdf.u_abs.unwrap());
        // pos_slack is not a URDF quantity and always equals the constant.
        assert!((limits.pos_slack - POS_SLACK).abs() <= EPS);
    }

    #[test]
    fn missing_whole_fields_fall_back_to_constants() {
        // Req 5.3: fields the URDF does not expose (None) fall back to the KR240
        // constants, while fields that are present use the URDF values.
        let present_q_lower = [-1.0, -1.1, -1.2, -1.3, -1.4, -1.5];
        let present_v_abs = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4];
        let urdf = UrdfLimits {
            q_lower: Some(present_q_lower),
            q_upper: None,
            v_abs: Some(present_v_abs),
            u_abs: None,
            a_abs: None,
            j_abs: None,
        };
        let limits = JointLimits::from_urdf_limits(&urdf);

        // Present fields use URDF values.
        assert_arr_eq(&limits.q_lower, &present_q_lower);
        assert_arr_eq(&limits.v_abs, &present_v_abs);
        // Missing fields fall back to the constants.
        assert_arr_eq(&limits.q_upper, &KR240_Q_UPPER);
        assert_arr_eq(&limits.u_abs, &KR240_U_ABS);
    }

    #[test]
    fn all_fields_missing_yields_full_fallback() {
        // Req 5.3: a URDF that exposes nothing yields exactly the fallback set.
        let urdf = UrdfLimits::default();
        let limits = JointLimits::from_urdf_limits(&urdf);
        assert_eq!(limits, JointLimits::kr240_fallback());
    }

    #[test]
    fn non_finite_position_falls_back_per_axis() {
        // Element-wise fallback: a single non-finite (NaN/inf) position value
        // falls back to the constant for *that axis only*, leaving the other
        // axes on their finite URDF values.
        let mut q_lower = [-1.0, -1.1, -1.2, -1.3, -1.4, -1.5];
        let mut q_upper = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5];
        q_lower[2] = f64::NAN; // axis 2 invalid -> fallback
        q_upper[4] = f64::INFINITY; // axis 4 invalid -> fallback

        let urdf = UrdfLimits {
            q_lower: Some(q_lower),
            q_upper: Some(q_upper),
            v_abs: None,
            u_abs: None,
            a_abs: None,
            j_abs: None,
        };
        let limits = JointLimits::from_urdf_limits(&urdf);

        for i in 0..6 {
            let expected_lower = if i == 2 { KR240_Q_LOWER[i] } else { q_lower[i] };
            let expected_upper = if i == 4 { KR240_Q_UPPER[i] } else { q_upper[i] };
            assert!(
                (limits.q_lower[i] - expected_lower).abs() <= EPS,
                "q_lower axis {i}"
            );
            assert!(
                (limits.q_upper[i] - expected_upper).abs() <= EPS,
                "q_upper axis {i}"
            );
        }
    }

    #[test]
    fn non_positive_magnitude_falls_back_per_axis() {
        // Element-wise fallback: velocity/effort values that are non-positive
        // (zero or negative) or non-finite fall back to the constant for that
        // axis only; strictly-positive finite values are kept.
        let mut v_abs = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let mut u_abs = [1.0e3, 2.0e3, 3.0e3, 4.0e3, 5.0e3, 6.0e3];
        v_abs[0] = 0.0; // non-positive -> fallback
        v_abs[3] = -1.5; // negative -> fallback
        u_abs[1] = f64::NAN; // non-finite -> fallback
        u_abs[5] = -2.0e3; // negative -> fallback

        let urdf = UrdfLimits {
            q_lower: None,
            q_upper: None,
            v_abs: Some(v_abs),
            u_abs: Some(u_abs),
            a_abs: None,
            j_abs: None,
        };
        let limits = JointLimits::from_urdf_limits(&urdf);

        for i in 0..6 {
            let expected_v = if i == 0 || i == 3 { KR240_V_ABS[i] } else { v_abs[i] };
            let expected_u = if i == 1 || i == 5 { KR240_U_ABS[i] } else { u_abs[i] };
            assert!((limits.v_abs[i] - expected_v).abs() <= EPS, "v_abs axis {i}");
            assert!((limits.u_abs[i] - expected_u).abs() <= EPS, "u_abs axis {i}");
        }
    }

    #[test]
    fn per_model_limits_are_read_distinctly() {
        // Req 5.1 / 5.2: each robot model presents its own URDF limit set and the
        // extraction must honor each one rather than the hard-coded KR240 set.
        // Distinct `tag` values give each model a mutually distinct limit set.
        let models = [
            ("KR240", model_limits(0.0)),
            ("KR210", model_limits(0.25)),
            ("KR270", model_limits(0.5)),
            ("KR160", model_limits(0.75)),
        ];

        let mut extracted = Vec::new();
        for (name, urdf) in &models {
            let limits = JointLimits::from_urdf_limits(urdf);
            // Each model's limits map through faithfully from its URDF input.
            assert_arr_eq(&limits.q_lower, &urdf.q_lower.unwrap());
            assert_arr_eq(&limits.q_upper, &urdf.q_upper.unwrap());
            assert_arr_eq(&limits.v_abs, &urdf.v_abs.unwrap());
            assert_arr_eq(&limits.u_abs, &urdf.u_abs.unwrap());
            extracted.push((*name, limits));
        }

        // The four models must yield mutually distinct limit sets (no collapse
        // to a single hard-coded model).
        for a in 0..extracted.len() {
            for b in (a + 1)..extracted.len() {
                assert_ne!(
                    extracted[a].1, extracted[b].1,
                    "{} and {} produced identical limits",
                    extracted[a].0, extracted[b].0
                );
            }
        }
    }
}
