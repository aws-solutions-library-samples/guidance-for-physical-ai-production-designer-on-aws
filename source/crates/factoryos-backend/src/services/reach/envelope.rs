//! The reach dome: where a robot's wrist centre (the A5 joint's origin) can be, revolved about A1.
//!
//! The wrist positions over the A2 × A3 limits (A1 = 0, the wrist joints at 0) make a region in the
//! arm's plane; as A1 turns it all the way round, the region in `(r, z)` (distance from the A1
//! axis, height in the robot's root frame) is the whole envelope, kept as a profile of
//! `(z, r_min, r_max)` per height bin. For a weld `p` approached along `a` with a tool of length `L`
//! (flange centre to TCP), the wrist must be at `W = p − L·a`: the weld is within reach exactly
//! when `W` is inside the profile (orientation limits of A5 and self collision left to the IK).
//! So the TCP's region for a fixed approach is the wrist's shifted by `L·a`: that is the dome
//! drawn in the Robo Fitter.

use nalgebra::{Isometry3, Point3, Vector3};

use super::kinematics::Chain;

/// Height bins of the profile, metres.
const BIN: f64 = 0.02;
const A2_STEP_DEG: f64 = 0.25;
const A3_STEP_DEG: f64 = 0.5;

#[derive(Clone, Debug)]
pub struct Envelope {
    /// A point on the A1 axis (vertical) in the robot's root frame.
    pub a1_origin: [f64; 3],
    /// `(z, r_min, r_max)` by increasing `z` (root frame).
    pub profile: Vec<(f64, f64, f64)>,
    /// The profile as a closed polygon in `(r, z)`: outer side up, inner side down.
    polygon: Vec<[f64; 2]>,
    /// The height the wrist reaches farthest at, and the band about it where `r_max` is at least
    /// [`COMFORT_BAND_SHARE`] of that reach ([`Envelope::sweet_height`], [`Envelope::comfort_band`]).
    sweet: f64,
    band: (f64, f64),
}

/// The share of the dome's farthest reach that bounds its comfortable height band.
pub const COMFORT_BAND_SHARE: f64 = 0.9;

/// Where a point is relative to the envelope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Inside,
    /// Beyond `r_max`, above or below.
    Outside,
    /// Closer to A1 than `r_min`.
    InnerVoid,
}

impl Envelope {
    /// The envelope of `chain`'s wrist centre (its 5th actuated joint); `None` for chains with
    /// fewer than 5 joints or an A1 that is not vertical.
    pub fn compute(chain: &Chain) -> Option<Self> {
        if chain.dof() < 5 {
            return None;
        }
        let limits = chain.limits();
        let mut q = vec![0.0; chain.dof()];
        let a1 = chain.actuated_frame(&q, 0)?;
        let axis = a1 * chain.actuated().next()?.axis.into_inner();
        if axis.z.abs() < 0.999 {
            return None;
        }
        let a1_origin = a1.translation.vector;
        let steps = |(lo, hi): (f64, f64), step_deg: f64| {
            let n = (((hi - lo).to_degrees() / step_deg).ceil() as usize).max(1);
            (0..=n).map(move |i| lo + (hi - lo) * i as f64 / n as f64)
        };
        let mut bins: std::collections::BTreeMap<i64, (f64, f64)> = std::collections::BTreeMap::new();
        for q2 in steps(limits[1], A2_STEP_DEG) {
            for q3 in steps(limits[2], A3_STEP_DEG) {
                q[1] = q2;
                q[2] = q3;
                let w = chain.actuated_frame(&q, 4)?.translation.vector;
                let r = ((w.x - a1_origin.x).powi(2) + (w.y - a1_origin.y).powi(2)).sqrt();
                let bin = (w.z / BIN).floor() as i64;
                let e = bins.entry(bin).or_insert((f64::MAX, f64::MIN));
                e.0 = e.0.min(r);
                e.1 = e.1.max(r);
            }
        }
        let profile: Vec<(f64, f64, f64)> = bins.into_iter().map(|(b, (lo, hi))| ((b as f64 + 0.5) * BIN, lo, hi)).collect();
        if profile.len() < 2 {
            return None;
        }
        Some(Self::from_profile([a1_origin.x, a1_origin.y, a1_origin.z], profile))
    }

    /// The envelope of a `(z, r_min, r_max)` profile by increasing `z` around A1 at `a1_origin`.
    pub fn from_profile(a1_origin: [f64; 3], profile: Vec<(f64, f64, f64)>) -> Self {
        let mut polygon: Vec<[f64; 2]> = profile.iter().map(|&(z, _, hi)| [hi, z]).collect();
        polygon.extend(profile.iter().rev().map(|&(z, lo, _)| [lo, z]));
        // The farthest bin (the first of equals), then the bins either side while r_max stays high.
        let best = profile.iter().enumerate().fold(None, |a: Option<(usize, f64)>, (i, p)| match a {
            Some((_, r)) if r >= p.2 => a,
            _ => Some((i, p.2)),
        });
        let (sweet, band) = match best {
            Some((i, r)) => {
                let keep = |j: usize| profile[j].2 >= COMFORT_BAND_SHARE * r;
                let (mut lo, mut hi) = (i, i);
                while lo > 0 && keep(lo - 1) {
                    lo -= 1;
                }
                while hi + 1 < profile.len() && keep(hi + 1) {
                    hi += 1;
                }
                (profile[i].0, (profile[lo].0 - BIN / 2.0, profile[hi].0 + BIN / 2.0))
            }
            None => (0.0, (0.0, 0.0)),
        };
        Self { a1_origin, profile, polygon, sweet, band }
    }

    /// The root-frame height the wrist reaches farthest at ("sweet height").
    pub fn sweet_height(&self) -> f64 {
        self.sweet
    }

    /// The comfortable height band `(z_lo, z_hi)` (root frame): the heights about the sweet height
    /// where the wrist's reach `r_max` is at least [`COMFORT_BAND_SHARE`] (90 %) of its farthest.
    pub fn comfort_band(&self) -> (f64, f64) {
        self.band
    }

    /// The farthest the wrist reaches from the A1 axis, at any height.
    pub fn r_max(&self) -> f64 {
        self.profile.iter().map(|p| p.2).fold(0.0, f64::max)
    }

    /// `r_min` (the inner void's radius) of the wrist at root-frame height `z`, if the envelope
    /// reaches that high.
    pub fn r_min_at(&self, z: f64) -> Option<f64> {
        self.r_max_at(z)?;
        let i = self.profile.partition_point(|&(bz, _, _)| bz < z).min(self.profile.len() - 1);
        let j = if i > 0 && (self.profile[i - 1].0 - z).abs() < (self.profile[i].0 - z).abs() { i - 1 } else { i };
        Some(self.profile[j].1)
    }

    /// The lowest root-frame `z` the wrist reaches.
    pub fn z_min(&self) -> f64 {
        self.profile.first().map_or(0.0, |p| p.0 - BIN / 2.0)
    }

    /// The signed distance (metres, positive inside) of the wrist position `(r, z)` from the
    /// envelope's boundary, and where it is.
    pub fn local_margin(&self, r: f64, z: f64) -> (f64, Side) {
        let p = [r, z];
        let n = self.polygon.len();
        let mut dist = f64::MAX;
        let mut inside = false;
        for i in 0..n {
            let (a, b) = (self.polygon[i], self.polygon[(i + 1) % n]);
            dist = dist.min(segment_distance(p, a, b));
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
                inside = !inside;
            }
        }
        if inside {
            return (dist, Side::Inside);
        }
        let void = self.profile.iter().find(|&&(bz, _, _)| (bz - z).abs() <= BIN / 2.0).is_some_and(|&(_, lo, _)| r < lo);
        (-dist, if void { Side::InnerVoid } else { Side::Outside })
    }

    /// The wrist position in this envelope's `(r, z)` for the TCP at `p` approached along `a`
    /// (scene frame), a robot at `base` (its root frame in the scene frame) with a tool of length `l`.
    pub fn wrist_rz(&self, base: &Isometry3<f64>, l: f64, p: [f64; 3], a: [f64; 3]) -> (f64, f64) {
        let w = Point3::from(p) - Vector3::from(a) * l;
        let local = base.inverse_transform_point(&w);
        (((local.x - self.a1_origin[0]).powi(2) + (local.y - self.a1_origin[1]).powi(2)).sqrt(), local.z)
    }

    /// The signed margin (metres, positive inside) of the weld `p` approached along `a`.
    pub fn margin(&self, base: &Isometry3<f64>, l: f64, p: [f64; 3], a: [f64; 3]) -> (f64, Side) {
        let (r, z) = self.wrist_rz(base, l, p, a);
        self.local_margin(r, z)
    }

    /// `r_max` of the wrist at root-frame height `z`, if the envelope reaches that high.
    pub fn r_max_at(&self, z: f64) -> Option<f64> {
        let (first, last) = (self.profile.first()?, self.profile.last()?);
        if z < first.0 - BIN / 2.0 || z > last.0 + BIN / 2.0 {
            return None;
        }
        let i = self.profile.partition_point(|&(bz, _, _)| bz < z);
        let pick = |i: usize| self.profile[i.min(self.profile.len() - 1)];
        if i == 0 {
            return Some(pick(0).2);
        }
        let (lo, hi) = (pick(i - 1), pick(i));
        if hi.0 <= lo.0 {
            return Some(lo.2);
        }
        let t = ((z - lo.0) / (hi.0 - lo.0)).clamp(0.0, 1.0);
        Some(lo.2 + (hi.2 - lo.2) * t)
    }

    /// The highest root-frame `z` the wrist reaches.
    pub fn z_max(&self) -> f64 {
        self.profile.last().map_or(0.0, |p| p.0 + BIN / 2.0)
    }

    /// The outer and inner surfaces of the TCP dome (the envelope shifted by `shift`, scene frame,
    /// e.g. `L·a`), each `(vertices, triangles)`, in the scene frame.
    pub fn mesh(&self, base: &Isometry3<f64>, shift: [f64; 3], segments: usize) -> (Surface, Surface) {
        let stride = (self.profile.len() / 60).max(1);
        let mut rows: Vec<(f64, f64, f64)> = self.profile.iter().step_by(stride).copied().collect();
        if let Some(last) = self.profile.last()
            && rows.last() != Some(last)
        {
            rows.push(*last);
        }
        let outer: Vec<(f64, f64)> = rows.iter().map(|&(z, _, hi)| (hi, z)).collect();
        let inner: Vec<(f64, f64)> = rows.iter().map(|&(z, lo, _)| (lo, z)).collect();
        (self.revolve(base, shift, &outer, segments), self.revolve(base, shift, &inner, segments))
    }

    fn revolve(&self, base: &Isometry3<f64>, shift: [f64; 3], curve: &[(f64, f64)], segments: usize) -> Surface {
        let segments = segments.max(3);
        let mut vertices = Vec::with_capacity(curve.len() * segments);
        for &(r, z) in curve {
            for s in 0..segments {
                let phi = std::f64::consts::TAU * s as f64 / segments as f64;
                vertices.push(self.to_scene(base, shift, r, phi, z));
            }
        }
        let mut triangles = Vec::new();
        for i in 0..curve.len().saturating_sub(1) {
            for s in 0..segments {
                let (a, b) = ((i * segments + s) as u32, (i * segments + (s + 1) % segments) as u32);
                let (c, d) = (a + segments as u32, b + segments as u32);
                triangles.push([a, b, d]);
                triangles.push([a, d, c]);
            }
        }
        Surface { vertices, triangles }
    }

    fn to_scene(&self, base: &Isometry3<f64>, shift: [f64; 3], r: f64, phi: f64, z: f64) -> [f64; 3] {
        let local = Point3::new(self.a1_origin[0] + r * phi.cos(), self.a1_origin[1] + r * phi.sin(), z);
        let p = base * local + Vector3::from(shift);
        [p.x, p.y, p.z]
    }

    /// The TCP dome's `r_max` circle at scene height `z` (the dome shifted by `shift`), if it
    /// reaches that high.
    pub fn ring(&self, base: &Isometry3<f64>, shift: [f64; 3], z: f64, segments: usize) -> Option<Vec<[f64; 3]>> {
        // The ring's height in the root frame (bases are upright in practice: its z axis is the scene's).
        let local = base.inverse_transform_point(&Point3::new(base.translation.x, base.translation.y, z - shift[2]));
        let r = self.r_max_at(local.z)?;
        Some((0..=segments).map(|s| self.to_scene(base, shift, r, std::f64::consts::TAU * s as f64 / segments as f64, local.z)).collect())
    }
}

/// A triangle mesh, scene frame.
#[derive(Clone, Debug, Default)]
pub struct Surface {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 < 1e-18 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) };
    ((p[0] - a[0] - t * dx).powi(2) + (p[1] - a[1] - t * dy).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::reach::kinematics::tests::kr240;

    pub(crate) fn kr240_envelope() -> Envelope {
        Envelope::compute(&Chain::from_urdf(&kr240()).unwrap()).unwrap()
    }

    #[test]
    fn the_kr240_dome() {
        let env = kr240_envelope();
        // The TCP dome for a top-down approach (a = −z, L = 0.9092): the wrist is L above the TCP.
        let l = 0.9092;
        for (h, want) in [(0.0, 3.19), (1.0, 2.98)] {
            let r = env.r_max_at(h + l).unwrap();
            assert!((r - want).abs() < 0.03, "h = {h}: r_max {r}");
        }
        let (m, side) = env.local_margin(1.5, 1.5);
        assert!(m > 0.5 && side == Side::Inside, "{m} {side:?}");
        let (m, side) = env.local_margin(4.0, 1.0);
        assert!(m < -0.5 && side == Side::Outside, "{m} {side:?}");
        let (outer, inner) = env.mesh(&Isometry3::identity(), [0.0, 0.0, -l], 64);
        assert!(!outer.triangles.is_empty() && outer.vertices.len() == inner.vertices.len());
        let ring = env.ring(&Isometry3::identity(), [0.0, 0.0, -l], 1.0, 64).unwrap();
        let r = (ring[0][0].powi(2) + ring[0][1].powi(2)).sqrt();
        assert!((r - 2.98).abs() < 0.03 && (ring[0][2] - 1.0).abs() < 1e-9, "{r}");
    }
}
