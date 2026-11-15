//! How comfortably a robot reaches a weld, from the reach dome alone (no IK, microseconds): what
//! "reachable" misses. A robot folded up close to its weld or stretched to its limit reaches it, but
//! moves awkwardly, slowly and near its joint limits and singularities; so does one that has to
//! twist its wrist far round to point the gun along the approach.
//!
//! Per weld (the wrist is the tool's length behind the weld, against the approach):
//! * `ratio`: the wrist's distance from A1 over the dome's reach at its height. Comfortable in
//!   [`ComfortSpec::ratio_min`]..[`ComfortSpec::ratio_max`] (about 45-80 %), best near
//!   [`ComfortSpec::ratio_target`]; below: cramped, above: stretched.
//! * `align_deg`: the angle between the approach (horizontally) and the line from A1 out to the
//!   wrist. A robot works best pushing the gun outward along its own reach; past
//!   [`ComfortSpec::align_max_deg`] the wrist (A4/A5) has to turn far round: twisted.
//! * `height_m`: the wrist's height above or below the height the robot reaches farthest at (the
//!   sweet height). Comfortable within the dome's own height band, where its reach is at least
//!   90 % of its farthest ([`Envelope::comfort_band`]), or within ± [`ComfortSpec::height_band_m`]
//!   of the sweet height when that is given; above / below: too high / too low (a pedestal or a
//!   lower product).
//!
//! The penalty is 0 for a weld in all bands, and grows by one per tolerance past a band.

use factoryos_protocol::fitter::TaskWeld;
use nalgebra::{Isometry3, Point3, Vector3};

use super::envelope::Envelope;

pub use factoryos_protocol::scene_edit::ComfortSpec;

/// What is wrong with a weld for a robot (the worst band missed).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Fit {
    Comfortable,
    Cramped,
    Stretched,
    Twisted,
    TooHigh,
    TooLow,
    /// Outside the dome (or the tool would hit the product).
    Unreachable,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WeldComfort {
    pub ratio: f64,
    pub align_deg: f64,
    pub height_m: f64,
    pub margin_m: f64,
    pub fit: Fit,
    pub penalty: f64,
    /// How far to move the base along the robot-to-wrist line to reach the target ratio
    /// (positive: away from the weld), metres.
    pub move_m: f64,
}

/// The height (robot root frame) the dome reaches farthest at.
pub fn sweet_height(envelope: &Envelope) -> f64 {
    envelope.sweet_height()
}

/// The comfortable wrist heights `(z_lo, z_hi)` (robot root frame): the sweet height ±
/// [`ComfortSpec::height_band_m`] when given (> 0), else the dome's own band
/// ([`Envelope::comfort_band`]: where its reach is at least 90 % of its farthest).
pub fn height_limits(envelope: &Envelope, spec: &ComfortSpec) -> (f64, f64) {
    match spec.height_band_m.filter(|b| *b > 0.0) {
        Some(b) => (envelope.sweet_height() - b, envelope.sweet_height() + b),
        None => envelope.comfort_band(),
    }
}

/// How comfortably a robot at `base` (tool length `l`) reaches `weld`; `reachable`: the dome and
/// product checks passed.
pub fn weld_comfort(envelope: &Envelope, base: &Isometry3<f64>, l: f64, weld: &TaskWeld, reachable: bool, spec: &ComfortSpec) -> WeldComfort {
    let (margin, _) = envelope.margin(base, l, weld.position, weld.approach);
    let (r, z) = envelope.wrist_rz(base, l, weld.position, weld.approach);
    let r_max = envelope.r_max_at(z).unwrap_or(0.0).max(1e-6);
    let ratio = r / r_max;
    // The approach and the line A1 -> wrist, horizontally, in the robot's frame.
    let wrist = base.inverse_transform_point(&(Point3::from(weld.position) - Vector3::from(weld.approach) * l));
    let out = Vector3::new(wrist.x - envelope.a1_origin[0], wrist.y - envelope.a1_origin[1], 0.0);
    let a = base.inverse_transform_vector(&Vector3::from(weld.approach));
    let a = Vector3::new(a.x, a.y, 0.0);
    let align_deg = if out.norm() < 1e-6 || a.norm() < 0.2 {
        // A vertical approach (from above or below) twists nothing about A1.
        0.0
    } else {
        out.normalize().dot(&a.normalize()).clamp(-1.0, 1.0).acos().to_degrees()
    };
    let height_m = z - sweet_height(envelope);
    let mut worst = (0.0, Fit::Comfortable);
    let mut penalty = 0.0;
    let mut add = |p: f64, fit: Fit| {
        if p > 0.0 {
            penalty += p;
            if p > worst.0 {
                worst = (p, fit);
            }
        }
    };
    add((spec.ratio_min - ratio) / 0.1, Fit::Cramped);
    add((ratio - spec.ratio_max) / 0.1, Fit::Stretched);
    add((align_deg - spec.align_max_deg) / 15.0, Fit::Twisted);
    let (z_lo, z_hi) = height_limits(envelope, spec);
    add((z - z_hi) / 0.2, Fit::TooHigh);
    add((z_lo - z) / 0.2, Fit::TooLow);
    // Inside the band, closer to the target is still better (a tie-break, < 0.5).
    penalty += ((ratio - spec.ratio_target).abs() / 0.4).min(0.5);
    let fit = if !reachable { Fit::Unreachable } else { worst.1 };
    if !reachable {
        penalty += 20.0;
    }
    let move_m = r - spec.ratio_target * r_max;
    WeldComfort { ratio, align_deg, height_m, margin_m: margin, fit, penalty, move_m: -move_m }
}

impl Fit {
    pub fn name(self) -> &'static str {
        match self {
            Fit::Comfortable => "comfortable",
            Fit::Cramped => "cramped",
            Fit::Stretched => "stretched",
            Fit::Twisted => "twisted",
            Fit::TooHigh => "too_high",
            Fit::TooLow => "too_low",
            Fit::Unreachable => "unreachable",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    /// A dome: r_max 3 m from z 0 to 2 m, farthest at z = 1.
    fn dome() -> Envelope {
        let profile = (0..=20).map(|i| (i as f64 * 0.1, 0.2, if i == 10 { 3.0 } else { 2.9 })).collect();
        Envelope::from_profile([0.0, 0.0, 0.0], profile)
    }

    fn weld(x: f64, approach: [f64; 3]) -> TaskWeld {
        TaskWeld { id: "w".into(), label: "w".into(), position: [x, 0.0, 1.0], approach, ..TaskWeld::default() }
    }

    #[test]
    fn folded_stretched_and_comfortable() {
        let (env, spec, base) = (dome(), ComfortSpec::default(), Isometry3::identity());
        // Approach outward along +x, tool 0.5 m: the wrist is 0.5 m before the weld.
        let c = weld_comfort(&env, &base, 0.5, &weld(1.2, [1.0, 0.0, 0.0]), true, &spec);
        assert_eq!(c.fit, Fit::Cramped, "{c:?}");
        assert!(c.move_m > 1.0, "move away by about 1.2 m: {c:?}");
        let c = weld_comfort(&env, &base, 0.5, &weld(2.4, [1.0, 0.0, 0.0]), true, &spec);
        assert_eq!(c.fit, Fit::Comfortable, "{c:?}");
        let c = weld_comfort(&env, &base, 0.5, &weld(3.0, [1.0, 0.0, 0.0]), true, &spec);
        assert_eq!(c.fit, Fit::Stretched, "{c:?}");
        // The gun pointing sideways: twisted.
        let c = weld_comfort(&env, &base, 0.5, &weld(2.0, [0.0, 1.0, 0.0]), true, &spec);
        assert_eq!(c.fit, Fit::Twisted, "{c:?}");
        // A robot turned away (its frame rotated): the same weld in its own frame.
        let turned = Isometry3::from_parts(Translation3::identity(), UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0));
        let c = weld_comfort(&env, &turned, 0.5, &weld(2.4, [1.0, 0.0, 0.0]), true, &spec);
        assert_eq!(c.fit, Fit::Comfortable, "yaw does not matter for the dome: {c:?}");
    }
}
