//! FR-108: the camera rig of a composed scene. Six fixed views frame the scene's bounding box, and
//! Kit's opening perspective view is seeded the same way; `compose` emits both. Pure math on
//! [`crate::math3`].
//!
//! Cameras follow USD: they look along their local −Z with +Y up. A view's rotation `R` maps
//! camera space to the world, so its columns are the image's right, up and back (target → eye)
//! directions, and the eye sits at `target + D · back`.

use crate::math3::{self, Mat4};

/// Resolution (width, height) of the viewport's render product
/// (`/Render/OmniverseKit/HydraTextures`). The lenses take its aspect ratio, so a view frames
/// what the viewport shows.
pub const RENDER_RESOLUTION: (u32, u32) = (1280, 720);

/// Film-back width (mm) of the rig's lens and of Kit's perspective camera.
pub const HORIZONTAL_APERTURE_MM: f64 = 20.955;

/// Focal length (mm) of the rig's cameras.
pub const RIG_FOCAL_LENGTH_MM: f64 = 32.992786;

/// Focal length (mm) of Kit's default perspective camera, `/OmniverseKit_Persp`.
pub const KIT_PERSPECTIVE_FOCAL_LENGTH_MM: f64 = 18.147562;

/// Shortest camera distance (m), so that a point-sized scene still gets a usable view.
pub const MIN_DISTANCE_M: f64 = 1.0;

/// Near clipping plane as a fraction of the camera distance.
pub const NEAR_CLIP_PER_DISTANCE: f64 = 0.20471;

/// The near clipping plane never comes closer than this (m).
pub const MIN_NEAR_CLIP_M: f64 = 0.01;

/// Far clipping plane (m).
pub const FAR_CLIP_M: f64 = 10000.0;

/// The rig cameras' exposure (`OmniRtxCameraExposureAPI_1`): Kit's own, which Kit writes onto
/// its viewport camera, `/OmniverseKit_Persp`. With ISO 100 the exposure scale is
/// `time / fStop² · responsivity` = 8.821e-4 (1 / 1133.6); USD's fallback (f/1, 1 s,
/// responsivity 1) is 1.0, 10.15 EV brighter. The f-number:
pub const EXPOSURE_F_STOP: f64 = 5.0;

/// The rig cameras' exposure time (s); see [`EXPOSURE_F_STOP`].
pub const EXPOSURE_TIME_S: f64 = 0.02;

/// The rig cameras' sensor responsivity; see [`EXPOSURE_F_STOP`].
pub const EXPOSURE_RESPONSIVITY: f64 = 1.1026709;

const X_AXIS: [f64; 3] = [1.0, 0.0, 0.0];
const Z_AXIS: [f64; 3] = [0.0, 0.0, 1.0];

/// A pinhole lens in USD camera units: focal length and film back (apertures), in mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lens {
    pub focal_length: f64,
    pub horizontal_aperture: f64,
    pub vertical_aperture: f64,
}

impl Lens {
    /// A lens of `focal_length` whose film back has [`RENDER_RESOLUTION`]'s aspect ratio.
    pub fn for_render_product(focal_length: f64) -> Self {
        let (width, height) = RENDER_RESOLUTION;
        Self {
            focal_length,
            horizontal_aperture: HORIZONTAL_APERTURE_MM,
            vertical_aperture: HORIZONTAL_APERTURE_MM * f64::from(height) / f64::from(width),
        }
    }

    /// The lens of the rig's cameras.
    pub fn rig() -> Self {
        Self::for_render_product(RIG_FOCAL_LENGTH_MM)
    }

    /// The lens of Kit's default perspective camera.
    pub fn kit_perspective() -> Self {
        Self::for_render_product(KIT_PERSPECTIVE_FOCAL_LENGTH_MM)
    }

    /// Tangents of half the field of view, horizontal and vertical.
    pub fn half_fov_tangents(&self) -> (f64, f64) {
        (
            self.horizontal_aperture / (2.0 * self.focal_length),
            self.vertical_aperture / (2.0 * self.focal_length),
        )
    }
}

/// An axis-aligned box in world space (m). [`Aabb::EMPTY`] contains no point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Default for Aabb {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Aabb {
    pub const EMPTY: Self = Self {
        min: [f64::INFINITY; 3],
        max: [f64::NEG_INFINITY; 3],
    };

    pub fn is_empty(&self) -> bool {
        self.min.iter().zip(&self.max).any(|(lo, hi)| lo > hi)
    }

    /// Grows the box to contain `p`. A point with a non-finite coordinate is ignored.
    pub fn add_point(&mut self, p: [f64; 3]) {
        if p.iter().all(|v| v.is_finite()) {
            for ((lo, hi), v) in self.min.iter_mut().zip(self.max.iter_mut()).zip(p) {
                *lo = lo.min(v);
                *hi = hi.max(v);
            }
        }
    }

    /// The centre of a non-empty box.
    pub fn center(&self) -> [f64; 3] {
        std::array::from_fn(|i| 0.5 * (self.min[i] + self.max[i]))
    }

    /// The 8 corners of a non-empty box.
    pub fn corners(&self) -> [[f64; 3]; 8] {
        std::array::from_fn(|i| {
            std::array::from_fn(|axis| {
                if i & (1 << axis) == 0 {
                    self.min[axis]
                } else {
                    self.max[axis]
                }
            })
        })
    }
}

/// A rig camera's prim name and orientation: `rotation` maps camera space to the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    pub name: &'static str,
    pub rotation: Mat4,
}

impl View {
    /// A Z-up view from `azimuth_deg` (counter-clockwise from +X: the direction from the target to
    /// the eye) and `elevation_deg` above the horizon: `R = Rz(azimuth + 90°) · Rx(90° − elevation)`.
    pub fn orbit(name: &'static str, azimuth_deg: f64, elevation_deg: f64) -> Self {
        let turn = math3::rotation_about_axis(Z_AXIS, (azimuth_deg + 90.0).to_radians());
        let tilt = math3::rotation_about_axis(X_AXIS, (90.0 - elevation_deg).to_radians());
        Self {
            name,
            rotation: math3::multiply(&turn, &tilt),
        }
    }

    /// Top-down with north (+Y) up, tilted 2° from straight down (`R = Rx(−2°)`): the eye stands
    /// slightly north of the target.
    pub fn plan(name: &'static str) -> Self {
        Self {
            name,
            rotation: math3::rotation_about_axis(X_AXIS, (-2.0f64).to_radians()),
        }
    }

    /// The image's right in the world (the rotation's first column).
    pub fn right(&self) -> [f64; 3] {
        self.column(0)
    }

    /// The image's up in the world (the rotation's second column).
    pub fn up(&self) -> [f64; 3] {
        self.column(1)
    }

    /// From the target towards the eye (the rotation's third column); the camera looks along its
    /// negation.
    pub fn back(&self) -> [f64; 3] {
        self.column(2)
    }

    fn column(&self, j: usize) -> [f64; 3] {
        std::array::from_fn(|i| self.rotation[i][j])
    }

    /// The orientation as USD writes a `quatd`: (w, x, y, z).
    pub fn orient_wxyz(&self) -> [f64; 4] {
        let [x, y, z, w] = math3::quat_of(&self.rotation);
        [w, x, y, z]
    }

    /// The eye position `distance` from `target`.
    pub fn eye(&self, target: [f64; 3], distance: f64) -> [f64; 3] {
        let back = self.back();
        std::array::from_fn(|i| target[i] + distance * back[i])
    }
}

/// The rig's views, in emission order: the four isometric corners (26° up), the top-down plan and
/// a low view from the east (7° up).
pub fn views() -> [View; 6] {
    [
        View::orbit("ISO_NE", 45.0, 26.0),
        View::orbit("ISO_NW", 135.0, 26.0),
        View::plan("PLAN"),
        View::orbit("ISO_SE", -45.0, 26.0),
        View::orbit("ISO_SW", -135.0, 26.0),
        View::orbit("ELEVATION", 0.0, 7.0),
    ]
}

/// The distance from the centre of `bounds` at which each of `views` sees all 8 corners through
/// `lens`, but at least [`MIN_DISTANCE_M`]. It is the smallest such distance, so (above the
/// minimum) a corner touches the frame of some view: with `v = corner − centre` in view
/// coordinates `(x, y, z) = (right·v, up·v, back·v)`, the corner is in the frame iff
/// `D ≥ |x| / tan_h + z` and `D ≥ |y| / tan_v + z`.
pub fn fit_distance(bounds: &Aabb, views: &[View], lens: &Lens) -> f64 {
    let (tan_h, tan_v) = lens.half_fov_tangents();
    let center = bounds.center();
    let mut distance = MIN_DISTANCE_M;
    for view in views {
        let (right, up, back) = (view.right(), view.up(), view.back());
        for corner in bounds.corners() {
            let v: [f64; 3] = std::array::from_fn(|i| corner[i] - center[i]);
            let z = dot(back, v);
            distance = distance
                .max(dot(right, v).abs() / tan_h + z)
                .max(dot(up, v).abs() / tan_v + z);
        }
    }
    distance
}

/// The rig's clipping range at camera distance `distance`: near at
/// `NEAR_CLIP_PER_DISTANCE · distance` (at least [`MIN_NEAR_CLIP_M`]), far at [`FAR_CLIP_M`].
pub fn clipping_range(distance: f64) -> [f64; 2] {
    [
        MIN_NEAR_CLIP_M.max(NEAR_CLIP_PER_DISTANCE * distance),
        FAR_CLIP_M,
    ]
}

/// A placed rig camera.
#[derive(Debug, Clone, PartialEq)]
pub struct RigCamera {
    pub name: &'static str,
    pub translate: [f64; 3],
    /// (w, x, y, z), as USD writes a `quatd`.
    pub orient_wxyz: [f64; 4],
}

/// The six cameras framing a box. They share the target (the box's centre), the distance, the
/// lens and the clipping range.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraRig {
    pub target: [f64; 3],
    pub distance: f64,
    pub lens: Lens,
    pub clipping_range: [f64; 2],
    /// In [`views`] order.
    pub cameras: Vec<RigCamera>,
}

impl CameraRig {
    /// The rig framing `bounds`, which must not be empty.
    pub fn framing(bounds: &Aabb) -> Self {
        let lens = Lens::rig();
        let views = views();
        let target = bounds.center();
        let distance = fit_distance(bounds, &views, &lens);
        let cameras = views
            .iter()
            .map(|view| RigCamera {
                name: view.name,
                translate: view.eye(target, distance),
                orient_wxyz: view.orient_wxyz(),
            })
            .collect();
        Self {
            target,
            distance,
            lens,
            clipping_range: clipping_range(distance),
            cameras,
        }
    }
}

/// Where a camera stands and what it looks at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Viewpoint {
    pub position: [f64; 3],
    pub target: [f64; 3],
}

/// The seed of Kit's opening view (`customLayerData.cameraSettings.Perspective`): ISO_NE's
/// direction, at the distance that frames the non-empty `bounds` through Kit's default lens.
pub fn perspective_seed(bounds: &Aabb) -> Viewpoint {
    let iso_ne = views()[0];
    let target = bounds.center();
    let distance = fit_distance(bounds, &[iso_ne], &Lens::kit_perspective());
    Viewpoint {
        position: iso_ne.eye(target, distance),
        target,
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb(min: [f64; 3], max: [f64; 3]) -> Aabb {
        Aabb { min, max }
    }

    /// FR-108's views as a look-at sees them: each name with the azimuth (counter-clockwise from
    /// +X) and elevation (degrees) of the direction from the target to the eye, and the world
    /// direction the image's up leans to. PLAN looks down from 2° north of the zenith (azimuth
    /// 90°, elevation 88°) with north (+Y) up; the others keep +Z up.
    const VIEW_ANGLES: [(&str, f64, f64, [f64; 3]); 6] = [
        ("ISO_NE", 45.0, 26.0, Z_AXIS),
        ("ISO_NW", 135.0, 26.0, Z_AXIS),
        ("PLAN", 90.0, 88.0, [0.0, 1.0, 0.0]),
        ("ISO_SE", -45.0, 26.0, Z_AXIS),
        ("ISO_SW", -135.0, 26.0, Z_AXIS),
        ("ELEVATION", 0.0, 7.0, Z_AXIS),
    ];

    fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| a[i] - b[i])
    }

    fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    fn normalize(a: [f64; 3]) -> [f64; 3] {
        let len = dot(a, a).sqrt();
        a.map(|v| v / len)
    }

    /// An independent look-at: the camera axes (right, up, back) of an eye at `eye` that looks at
    /// `target`, its image's up as close to `up_hint` as the view allows.
    fn look_at(eye: [f64; 3], target: [f64; 3], up_hint: [f64; 3]) -> [[f64; 3]; 3] {
        let forward = normalize(sub(target, eye));
        let right = normalize(cross(forward, up_hint));
        let up = cross(right, forward);
        [right, up, forward.map(|v| -v)]
    }

    /// The camera axes (right, up, back: the rotation's columns) of the orientation `q`, written
    /// as USD does, (w, x, y, z).
    fn axes_of(q: [f64; 4]) -> [[f64; 3]; 3] {
        let [w, x, y, z] = q;
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y + w * z),
                2.0 * (x * z - w * y),
            ],
            [
                2.0 * (x * y - w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z + w * x),
            ],
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }

    /// The orientation (w, x, y, z) of the rotation whose columns are `axes` (right, up, back), by
    /// Shepperd's method: the largest of the four components first, the others from it.
    fn quat_of_axes(axes: [[f64; 3]; 3]) -> [f64; 4] {
        // Row i of column j.
        let m = |i: usize, j: usize| axes[j][i];
        let trace = m(0, 0) + m(1, 1) + m(2, 2);
        if trace > 0.0 {
            let s = 2.0 * (1.0 + trace).sqrt();
            let (x, y, z) = (m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), m(1, 0) - m(0, 1));
            [0.25 * s, x / s, y / s, z / s]
        } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
            let s = 2.0 * (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt();
            let (w, y, z) = (m(2, 1) - m(1, 2), m(0, 1) + m(1, 0), m(0, 2) + m(2, 0));
            [w / s, 0.25 * s, y / s, z / s]
        } else if m(1, 1) > m(2, 2) {
            let s = 2.0 * (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt();
            let (w, x, z) = (m(0, 2) - m(2, 0), m(0, 1) + m(1, 0), m(1, 2) + m(2, 1));
            [w / s, x / s, 0.25 * s, z / s]
        } else {
            let s = 2.0 * (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt();
            let (w, x, y) = (m(1, 0) - m(0, 1), m(0, 2) + m(2, 0), m(1, 2) + m(2, 1));
            [w / s, x / s, y / s, 0.25 * s]
        }
    }

    fn assert_vec3_close(got: [f64; 3], want: [f64; 3], tol: f64, what: &str) {
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < tol, "{what}: {got:?} != {want:?}");
        }
    }

    #[test]
    fn views_match_an_independent_look_at_construction() {
        // A synthetic centre and distance.
        let target = [3.0, -1.5, 0.75];
        let distance = 12.0;
        let views = views();
        assert_eq!(views.len(), VIEW_ANGLES.len());
        for (view, (name, azimuth, elevation, up_hint)) in views.iter().zip(VIEW_ANGLES) {
            assert_eq!(view.name, name);
            // The eye: C + D · (cos el · cos az, cos el · sin az, sin el).
            let (az, el) = (azimuth.to_radians(), elevation.to_radians());
            let direction = [el.cos() * az.cos(), el.cos() * az.sin(), el.sin()];
            let eye: [f64; 3] = std::array::from_fn(|i| target[i] + distance * direction[i]);
            assert_vec3_close(view.eye(target, distance), eye, 1e-12, name);
            // The look-at's basis: forward = normalize(C − eye), right and up from the up hint.
            let [right, up, back] = look_at(eye, target, up_hint);
            assert_vec3_close(view.right(), right, 1e-12, name);
            assert_vec3_close(view.up(), up, 1e-12, name);
            assert_vec3_close(view.back(), back, 1e-12, name);
            // The orientation as written is the basis's quaternion, up to sign (q and −q are the
            // same rotation), and turns the camera's axes onto the look-at's.
            let q = view.orient_wxyz();
            let want = quat_of_axes([right, up, back]);
            let sign = if q.iter().zip(&want).map(|(a, b)| a * b).sum::<f64>() < 0.0 {
                -1.0
            } else {
                1.0
            };
            for (g, w) in q.iter().zip(want) {
                assert!((g - sign * w).abs() < 1e-12, "{name}: {q:?} != ±{want:?}");
            }
            let norm: f64 = q.iter().map(|v| v * v).sum();
            assert!((norm - 1.0).abs() < 1e-12, "{name}: {q:?}");
            let [r, u, b] = axes_of(q);
            assert_vec3_close(r, right, 1e-12, name);
            assert_vec3_close(u, up, 1e-12, name);
            assert_vec3_close(b, back, 1e-12, name);
        }
    }

    #[test]
    fn views_are_upright_rotations_in_emission_order() {
        let names: Vec<&str> = views().iter().map(|v| v.name).collect();
        assert_eq!(
            names,
            ["ISO_NE", "ISO_NW", "PLAN", "ISO_SE", "ISO_SW", "ELEVATION"]
        );
        for view in views() {
            // right × up = back: a proper rotation.
            let (r, u, b) = (view.right(), view.up(), view.back());
            let cross = [
                r[1] * u[2] - r[2] * u[1],
                r[2] * u[0] - r[0] * u[2],
                r[0] * u[1] - r[1] * u[0],
            ];
            for (c, b) in cross.iter().zip(b) {
                assert!((c - b).abs() < 1e-12, "{}", view.name);
            }
            // The eye is above the target.
            assert!(b[2] > 0.0, "{}", view.name);
            if view.name != "PLAN" {
                // A level horizon, and the image's up points skywards.
                assert!(r[2].abs() < 1e-12 && u[2] > 0.0, "{}", view.name);
            }
        }
        // PLAN looks down with north (+Y) up.
        let plan = views()[2];
        assert_eq!(plan.name, "PLAN");
        assert!(plan.back()[2] > 0.999 && plan.up()[1] > 0.999 && plan.right()[0] > 0.999);
    }

    #[test]
    fn lenses_take_the_render_products_aspect_ratio() {
        assert_eq!(RENDER_RESOLUTION, (1280, 720));
        let rig = Lens::rig();
        assert!((rig.focal_length - 32.992786).abs() < 1e-12);
        assert!((rig.horizontal_aperture - 20.955).abs() < 1e-12);
        assert!((rig.vertical_aperture - 11.7871875).abs() < 1e-12);
        let kit = Lens::kit_perspective();
        assert!((kit.focal_length - 18.147562).abs() < 1e-12);
        assert!((kit.horizontal_aperture - 20.955).abs() < 1e-12);
        assert!((kit.vertical_aperture - 11.7871875).abs() < 1e-12);
    }

    /// `p` in the camera space of a camera at `translate` with orientation `q` (w, x, y, z):
    /// right, up, and the depth along the view direction.
    fn camera_space(translate: [f64; 3], q: [f64; 4], p: [f64; 3]) -> [f64; 3] {
        // The rotation's columns: the camera's axes in the world.
        let axes = axes_of(q);
        let d: [f64; 3] = std::array::from_fn(|i| p[i] - translate[i]);
        [dot(axes[0], d), dot(axes[1], d), -dot(axes[2], d)]
    }

    /// Boxes of many shapes: a cell, a floor plate, a pole, rails, far from the origin, and
    /// pseudo-random ones (a fixed LCG, so the test is deterministic).
    fn test_boxes() -> Vec<Aabb> {
        let mut boxes = vec![
            aabb([0.0; 3], [1.0; 3]),
            aabb([-7.0, -4.0, 0.0], [11.0, 8.5, 3.5]),
            aabb([-10.0, -10.0, 0.0], [10.0, 10.0, 0.01]),
            aabb([-0.1, -0.1, 0.0], [0.1, 0.1, 20.0]),
            aabb([-50.0, -0.5, 0.0], [50.0, 0.5, 1.0]),
            aabb([-0.5, -50.0, 0.0], [0.5, 50.0, 1.0]),
            aabb([100.0, -200.0, 5.0], [103.0, -195.0, 7.0]),
        ];
        let mut state = 0x2545_f491_4f6c_dd1d_u64;
        let mut unit = move || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..500 {
            let min: [f64; 3] = std::array::from_fn(|_| unit() * 200.0 - 100.0);
            let size: [f64; 3] = std::array::from_fn(|_| unit().powi(3) * 60.0);
            boxes.push(aabb(min, std::array::from_fn(|i| min[i] + size[i])));
        }
        boxes
    }

    #[test]
    fn rig_frames_every_corner_tightly_and_clips_none() {
        for bounds in test_boxes() {
            let rig = CameraRig::framing(&bounds);
            assert_eq!(rig.target, bounds.center());
            let (tan_h, tan_v) = rig.lens.half_fov_tangents();
            let [near, far] = rig.clipping_range;
            let mut tightest = 0.0f64;
            for cam in &rig.cameras {
                // Every camera looks at the target, from the shared distance.
                let target = camera_space(cam.translate, cam.orient_wxyz, rig.target);
                assert!(target[0].abs() < 1e-9 && target[1].abs() < 1e-9);
                assert!((target[2] - rig.distance).abs() < 1e-9 * rig.distance);
                for corner in bounds.corners() {
                    let [x, y, depth] = camera_space(cam.translate, cam.orient_wxyz, corner);
                    assert!(
                        near <= depth && depth <= far,
                        "{}: corner {corner:?} of {bounds:?} at depth {depth} is clipped by \
                         [{near}, {far}]",
                        cam.name
                    );
                    let edge = (x.abs() / (tan_h * depth)).max(y.abs() / (tan_v * depth));
                    assert!(
                        edge <= 1.0 + 1e-9,
                        "{}: corner {corner:?} of {bounds:?} out of frame ({edge})",
                        cam.name
                    );
                    tightest = tightest.max(edge);
                }
            }
            // Above the minimum distance, some corner touches some frame's edge.
            if rig.distance > MIN_DISTANCE_M {
                assert!(
                    (tightest - 1.0).abs() < 1e-9,
                    "{bounds:?}: loose fit ({tightest})"
                );
            }
        }
    }

    #[test]
    fn rig_clipping_range_scales_with_the_distance() {
        let rig = CameraRig::framing(&aabb([-7.0, -4.0, 0.0], [11.0, 8.5, 3.5]));
        assert_eq!(
            rig.clipping_range,
            [NEAR_CLIP_PER_DISTANCE * rig.distance, FAR_CLIP_M]
        );
        assert_eq!(clipping_range(0.01), [MIN_NEAR_CLIP_M, FAR_CLIP_M]);
    }

    #[test]
    fn a_point_sized_scene_gets_the_minimum_distance() {
        let mut point = Aabb::EMPTY;
        point.add_point([1.0, 2.0, 3.0]);
        let rig = CameraRig::framing(&point);
        assert_eq!(rig.target, [1.0, 2.0, 3.0]);
        assert_eq!(rig.distance, MIN_DISTANCE_M);
        assert_eq!(rig.clipping_range, [NEAR_CLIP_PER_DISTANCE, FAR_CLIP_M]);
        assert_eq!(rig.cameras.len(), 6);
    }

    #[test]
    fn perspective_seed_frames_the_box_from_iso_ne_through_kits_lens() {
        let bounds = aabb([-7.0, -4.0, 0.0], [11.0, 8.5, 3.5]);
        let seed = perspective_seed(&bounds);
        assert_eq!(seed.target, bounds.center());
        let iso_ne = views()[0];
        assert_eq!(iso_ne.name, "ISO_NE");
        let lens = Lens::kit_perspective();
        let distance = fit_distance(&bounds, &[iso_ne], &lens);
        assert_eq!(seed.position, iso_ne.eye(seed.target, distance));
        // Every corner is in Kit's frame, one on its edge.
        let (tan_h, tan_v) = lens.half_fov_tangents();
        let mut tightest = 0.0f64;
        for corner in bounds.corners() {
            let [x, y, depth] = camera_space(seed.position, iso_ne.orient_wxyz(), corner);
            assert!(depth > 0.0);
            tightest = tightest.max((x.abs() / (tan_h * depth)).max(y.abs() / (tan_v * depth)));
        }
        assert!((tightest - 1.0).abs() < 1e-9, "{tightest}");
        // Kit's wider lens, and one view instead of six, frame the box from closer than the rig.
        assert!(distance < CameraRig::framing(&bounds).distance);
    }

    #[test]
    fn aabb_grows_with_finite_points_only() {
        let mut b = Aabb::default();
        assert!(b.is_empty());
        b.add_point([f64::NAN, 0.0, 0.0]);
        b.add_point([0.0, f64::INFINITY, 0.0]);
        assert!(b.is_empty());
        b.add_point([1.0, -2.0, 3.0]);
        b.add_point([-1.0, 4.0, 0.0]);
        assert!(!b.is_empty());
        assert_eq!(b, aabb([-1.0, -2.0, 0.0], [1.0, 4.0, 3.0]));
        assert_eq!(b.center(), [0.0, 1.0, 1.5]);
        let corners = b.corners();
        for (i, c) in corners.iter().enumerate() {
            for (axis, v) in c.iter().enumerate() {
                assert!(*v == b.min[axis] || *v == b.max[axis]);
            }
            // All 8 corners differ.
            assert!(!corners[..i].contains(c));
        }
    }
}
