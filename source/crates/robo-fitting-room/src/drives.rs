//! Joint drives and the mass model they are tuned on.
//!
//! - FR-157: [`actuated_home_values`] is the one source of every actuated joint's home value: the
//!   link bind poses, the drive targets and the initial joint states are all written from it.
//! - FR-113, FR-159: [`link_mass_properties`] gives the mass properties a link is emitted with: its
//!   URDF inertial unless that is a placeholder, else its collision geometry at an effective link
//!   density, with a tool payload merged in ([`payload_body`] spreads one over the tool's
//!   collision hulls) or else the tool hulls at the same density.
//! - FR-158: [`drive_params`] gives an actuated joint's drive: stiffness 1.5e5 per degree (a linear
//!   drive's from the mass it moves, [`LINEAR_DRIVE_SAG_M`]), the damping for a damping ratio of
//!   0.7 on the composite inertia the joint moves at the home pose, a finite force cap and the URDF
//!   velocity limit.
//!
//! Units are UsdPhysics': angular drives per degree (stiffness N·m/deg, damping N·m·s/deg, targets
//! and states in degrees, velocities in deg/s), linear drives per metre (stiffness N/m, damping
//! N·s/m, targets and states in metres, velocities in m/s). omni.physx converts the angular gains to
//! per radian (× 180/π), so 1.5e5 N·m/deg is 8.6e6 N·m/rad. Its drives are implicit: the gains
//! don't limit the step size, but they set each joint's natural frequency and, with the inertia it
//! moves, its damping ratio.

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use urdf_robo_dresser::{Geometry, Joint, JointType, Link, UrdfRobot};

use crate::math3::{self, Mat3, Mat4, Vec3};

// =============================================================================
// Home values (FR-117, FR-118, FR-157)
// =============================================================================

/// FR-117: the KUKA-style stowed pose (degrees) of a robot whose manifest gives no `home_pose_deg`,
/// by actuated joint: A2 −90°, A3 +90°, the rest 0, which folds a 6-axis arm upright, clear of the
/// floor and its base.
pub const KUKA_HOME_POSE_DEG: [f64; 6] = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0];

/// FR-118: how far (degrees) a revolute joint's home angle may lie outside its URDF limits: the
/// slack of a limit written in radians to 7 digits.
pub const HOME_LIMIT_TOLERANCE_DEG: f64 = 1e-4;

/// FR-118: how far (metres) a prismatic joint's home value may lie outside its URDF limits.
pub const HOME_LIMIT_TOLERANCE_M: f64 = 1e-6;

/// The actuated joints of `robot`: every non-fixed joint, in URDF declaration order. A manifest's
/// `home_pose_deg` and [`actuated_home_values`] are indexed by them.
pub fn actuated_joints(robot: &UrdfRobot) -> impl Iterator<Item = &Joint> {
    robot
        .joints
        .iter()
        .filter(|j| j.joint_type != JointType::Fixed)
}

/// FR-157: each actuated joint's home value ([`actuated_joints`] order), the one source of the link
/// bind poses, the drive targets and the initial joint states:
/// - revolute: the angle (degrees) as given, unwrapped: 190° stays 190°, which a wrist's ±350°
///   range tells apart from −170°;
/// - continuous: the angle wrapped into (−180°, 180°] ([`wrap_degrees`]);
/// - prismatic: the manifest's entry as given, in metres (a slide, not a turn);
/// - floating and planar joints, which aren't emitted: 0.
///
/// The values are the manifest's `home_pose_deg` (0 for an entry it lacks), else
/// [`KUKA_HOME_POSE_DEG`] by index (0 beyond its six joints), whose angles turn the revolute and
/// continuous joints only: without a `home_pose_deg` every prismatic joint sits at 0 m.
pub fn actuated_home_values(robot: &UrdfRobot, home_pose_deg: Option<&[f64]>) -> Vec<f64> {
    actuated_joints(robot)
        .enumerate()
        .map(|(i, joint)| {
            let value = match home_pose_deg {
                Some(values) => values.get(i).copied().unwrap_or(0.0),
                None if joint.joint_type == JointType::Prismatic => 0.0,
                None => KUKA_HOME_POSE_DEG.get(i).copied().unwrap_or(0.0),
            };
            match joint.joint_type {
                JointType::Revolute | JointType::Prismatic => value,
                JointType::Continuous => wrap_degrees(value),
                _ => 0.0,
            }
        })
        .collect()
}

/// The unit vector along a URDF joint axis; a zero (or non-finite) axis, which validation rejects
/// on an actuated joint, is URDF's default axis, +X.
pub fn unit_joint_axis(axis: Vec3) -> Vec3 {
    let len = math3::norm(axis);
    if len.is_finite() && len > 1e-12 {
        math3::scale(axis, 1.0 / len)
    } else {
        [1.0, 0.0, 0.0]
    }
}

/// `deg` wrapped into (−180°, 180°]; a value already inside comes back unchanged.
pub fn wrap_degrees(deg: f64) -> f64 {
    if deg > -180.0 && deg <= 180.0 {
        return deg;
    }
    let wrapped = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if wrapped <= -180.0 {
        wrapped + 360.0
    } else {
        wrapped
    }
}

// =============================================================================
// Mass properties (FR-113, FR-159)
// =============================================================================

/// FR-113: the effective density (kg/m³) a link's mass is derived from where its URDF inertial is
/// a placeholder (and a tool's, without a payload): mass per volume of the link's closed collision
/// geometry, not of its material, since a link's casting, motors and gearboxes fill only part of
/// the hull around them. For heavy six-axis industrial arms (around a tonne each by their
/// datasheets) the arm's mass over the summed convex hulls of its links is of the order of
/// 10³ kg/m³; how closely the hulls wrap the links moves it a good deal (looser hulls, such as a
/// convex decomposition's, give a lower figure), so a derived arm can come out well above or below
/// its datasheet mass. A robot's manifest `link_density_kg_m3` overrides it ([`link_density`]):
/// the arm's datasheet mass over its links' summed hull volume.
pub const DEFAULT_LINK_DENSITY_KG_M3: f64 = 1200.0;

/// FR-113: a URDF inertial is a placeholder (an exporter's default, such as 2 kg on a link the
/// size of a car door), not the link's mass, when its mass spread over the link's robot geometry
/// is less dense than this fraction (a quarter) of the link density.
pub const PLACEHOLDER_DENSITY_FRACTION: f64 = 0.25;

/// FR-113: the mass (kg) and diagonal inertia (kg·m²) of a link welded to the URDF root (through
/// fixed joints only) with neither geometry nor a usable inertial (a URDF `World` root), which the
/// world anchor holds still: heavy enough that PhysX never falls back to its "negative mass, small
/// sphere" approximation; as it can't move, its mass doesn't load any joint.
const FALLBACK_MASS_KG: f64 = 10.0;
const FALLBACK_INERTIA: Vec3 = [1.0, 1.0, 1.0];

/// FR-113: the mass (kg) of a frame link a joint moves (a flange, `tool0` or TCP frame: neither
/// geometry nor a usable inertial): enough for PhysX to take the body as authored (no "negative
/// mass, small sphere" fallback), small enough not to skew the chain it hangs on or the drives
/// tuned on it. Its inertia is a solid sphere's of [`POINT_MASS_RADIUS_M`]: 1e-4 kg·m².
pub const FRAME_LINK_MASS_KG: f64 = 0.1;

/// FR-113, FR-159: the radius (m) of the solid sphere (inertia 2/5·m·r² about every axis) a mass
/// without a shape is given: a frame link's, and a payload's with nothing to spread it over. No
/// rigid body is authored with a zero inertia, which UsdPhysics ignores (and recomputes from the
/// colliders such a body hasn't got).
pub const POINT_MASS_RADIUS_M: f64 = 0.05;

/// Principal axes within this (per matrix entry) of the identity are the identity.
const PRINCIPAL_AXES_IDENTITY_TOLERANCE: f64 = 1e-9;

/// FR-113: the effective link density (kg/m³) of a robot: its manifest's `link_density_kg_m3` when
/// that is finite and > 0 (validation rejects any other), else [`DEFAULT_LINK_DENSITY_KG_M3`].
pub fn link_density(manifest_kg_m3: Option<f64>) -> f64 {
    manifest_kg_m3
        .filter(|d| d.is_finite() && *d > 0.0)
        .unwrap_or(DEFAULT_LINK_DENSITY_KG_M3)
}

/// A link's mass properties as emitted (`PhysicsMassAPI`) and as its drives are tuned on: the mass
/// (kg), the centre of mass (m, link frame), the principal moments of inertia (kg·m²) and the
/// principal axes, the columns of a rotation in the link frame (the identity when the inertia
/// tensor is diagonal in the link's axes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProps {
    pub mass: f64,
    pub center_of_mass: Vec3,
    pub diagonal_inertia: Vec3,
    pub principal_axes: Mat3,
}

impl MassProps {
    /// FR-113: the mass properties of a link welded to the URDF root with neither geometry nor a
    /// usable inertial: 10 kg with a unit inertia at the link origin, so no rigid body is massless.
    pub fn fallback() -> Self {
        Self {
            mass: FALLBACK_MASS_KG,
            center_of_mass: [0.0; 3],
            diagonal_inertia: FALLBACK_INERTIA,
            principal_axes: math3::IDENTITY3,
        }
    }

    /// FR-113: the mass properties of a frame link a joint moves (neither geometry nor a usable
    /// inertial): [`FRAME_LINK_MASS_KG`] as a solid sphere of [`POINT_MASS_RADIUS_M`] at the link
    /// origin.
    pub fn frame() -> Self {
        let moment = point_mass_unit_inertia() * FRAME_LINK_MASS_KG;
        Self {
            mass: FRAME_LINK_MASS_KG,
            center_of_mass: [0.0; 3],
            diagonal_inertia: [moment; 3],
            principal_axes: math3::IDENTITY3,
        }
    }

    /// The inertia tensor about the centre of mass, in the link's axes: `A · diag(I) · Aᵀ`.
    pub fn inertia_tensor(&self) -> Mat3 {
        math3::conjugate(&self.principal_axes, &math3::diag(self.diagonal_inertia))
    }

    /// `physics:principalAxes` as (w, x, y, z), authored with every `physics:diagonalInertia`
    /// (UsdPhysics wants both or neither): (1, 0, 0, 0) for the link's own axes.
    pub fn principal_axes_wxyz(&self) -> [f64; 4] {
        let [x, y, z, w] = math3::quat_of_rotation(&self.principal_axes);
        [w, x, y, z]
    }
}

/// A rigid body in a link's frame: its mass (kg), centre of mass (m) and inertia tensor about its
/// centre of mass (kg·m², link axes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    pub mass: f64,
    pub center_of_mass: Vec3,
    pub inertia: Mat3,
}

/// A closed triangle mesh in a link's frame: a collision hull (or visual mesh) with its origin and
/// scale applied.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HullMesh {
    /// Names the mesh in warnings (its mesh file).
    pub name: String,
    pub points: Vec<Vec3>,
    /// Three indices into `points` per triangle.
    pub triangle_indices: Vec<u32>,
}

/// FR-113: a URDF primitive (box, cylinder or sphere) of a link as a solid in the link's frame:
/// its volume (m³), its centroid (m) and its spread, the second moment about the centroid per unit
/// volume `∫(x − c)·(x − c)ᵀ dV / V` (m², link axes).
#[derive(Debug, Clone, PartialEq)]
pub struct PrimitiveSolid {
    /// Names the primitive in warnings.
    pub name: String,
    pub volume: f64,
    pub centroid: Vec3,
    pub spread: Mat3,
}

impl PrimitiveSolid {
    /// FR-113: the URDF primitive `geometry` (a box, centred; a cylinder along its Z, centred; a
    /// sphere) placed in its link's frame by `pose`, its origin; `None` for a mesh, or for a
    /// primitive whose dimensions aren't finite and > 0.
    pub fn of(geometry: &Geometry, pose: &Mat4, name: &str) -> Option<Self> {
        let positive = |v: &[f64]| v.iter().all(|x| x.is_finite() && *x > 0.0);
        let (volume, spread) = match *geometry {
            Geometry::Box { size: [a, b, c] } if positive(&[a, b, c]) => (
                a * b * c,
                math3::diag([a * a / 12.0, b * b / 12.0, c * c / 12.0]),
            ),
            Geometry::Cylinder {
                radius: r,
                length: l,
            } if positive(&[r, l]) => (
                PI * r * r * l,
                math3::diag([r * r / 4.0, r * r / 4.0, l * l / 12.0]),
            ),
            Geometry::Sphere { radius: r } if positive(&[r]) => {
                (4.0 / 3.0 * PI * r * r * r, math3::diag([r * r / 5.0; 3]))
            }
            _ => return None,
        };
        Some(Self {
            name: name.to_owned(),
            volume,
            centroid: math3::translation_of(pose),
            spread: math3::conjugate(&math3::rotation_of(pose), &spread),
        })
    }

    /// The primitive moved by the rigid `pose` (into another frame).
    pub fn transformed(&self, pose: &Mat4) -> Self {
        Self {
            name: self.name.clone(),
            volume: self.volume,
            centroid: math3::transform_point(pose, self.centroid),
            spread: math3::conjugate(&math3::rotation_of(pose), &self.spread),
        }
    }

    /// The primitive's [`SolidMoments`] about `reference`.
    fn moments(&self, reference: Vec3) -> SolidMoments {
        let d = math3::sub(self.centroid, reference);
        SolidMoments {
            volume: self.volume,
            first: math3::scale(d, self.volume),
            second: math3::mat3_scale(
                &math3::mat3_add(&self.spread, &math3::outer(d, d)),
                self.volume,
            ),
        }
    }
}

/// FR-113: the geometry a link's mass is taken from, in the link's frame: closed triangle meshes
/// and URDF primitives.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkMeshes {
    /// The robot's own geometry: the link's collision meshes that aren't the tool's, or its visual
    /// meshes that aren't when it has no such collision.
    pub robot: Vec<HullMesh>,
    /// The tool's collision hulls (`meshes/tool/`, or a named payload link's, see
    /// [`crate::tool::ToolLayout`]).
    pub tool: Vec<HullMesh>,
    /// The robot's own URDF primitives, as [`Self::robot`] picks them.
    pub robot_primitives: Vec<PrimitiveSolid>,
    /// The tool's URDF primitive colliders.
    pub tool_primitives: Vec<PrimitiveSolid>,
}

/// FR-113, FR-159: the mass properties `link` is emitted with and its joints' drives are tuned on
/// ([`drive_params`]), from its geometry (`meshes`), the tool payload's body when the link carries
/// it (`payload`, [`payload_body`]) and the robot's effective link density (`density`, kg/m³,
/// [`link_density`]). Two bodies make up the link:
/// - The robot's own, of the robot geometry `meshes.robot` and `meshes.robot_primitives` (volume
///   V, by the divergence theorem, and analytically for the primitives):
///   1. the URDF inertial as authored when it is usable (its mass finite and > 0) and no
///      placeholder — the link has no robot geometry (V = 0), or the inertial's mass over V is at
///      least [`PLACEHOLDER_DENSITY_FRACTION`] of `density`: its mass, its origin as the centre of
///      mass and its full tensor (the products of inertia too), turned into the link's axes by the
///      origin's rpy. A URDF inertial is taken as the robot geometry's: the dresser adds a tool's
///      geometry to the flange link and leaves its inertial alone;
///   2. else, with robot geometry, `density` over it: its volume's mass, centroid and inertia;
///   3. else none.
/// - The tool's, of the tool hulls `meshes.tool` and `meshes.tool_primitives`: `payload` when
///   given, else `density` over them.
///
/// The two are combined — the masses add up, the centre of mass is their mass-weighted mean, the
/// inertia about it the parallel-axis sum — and diagonalised ([`math3::eig3`]) into principal
/// moments and axes. A link without either gets [`MassProps::fallback`] when it is `welded` to the
/// URDF root (through fixed joints only: nothing moves it), else [`MassProps::frame`]. `context`
/// names the link in warnings.
pub fn link_mass_properties(
    link: &Link,
    meshes: &LinkMeshes,
    payload: Option<&Body>,
    density: f64,
    welded: bool,
    context: &str,
) -> MassProps {
    let robot = robot_body(
        link,
        &meshes.robot,
        &meshes.robot_primitives,
        density,
        context,
    );
    let tool = match payload {
        Some(payload) => Some(*payload),
        None => {
            solid_of(&meshes.tool, &meshes.tool_primitives, context).map(|tool| tool.body(density))
        }
    };
    let bodies: Vec<Body> = robot
        .into_iter()
        .chain(tool)
        .filter(|body| {
            let usable = body.mass.is_finite() && body.mass > 0.0;
            if !usable {
                log::warn!("{context}: ignoring a body of mass {} kg", body.mass);
            }
            usable
        })
        .collect();
    combine(&bodies).unwrap_or_else(|| {
        if welded {
            MassProps::fallback()
        } else {
            MassProps::frame()
        }
    })
}

/// The robot's own body of `link`, of its robot geometry `robot` and `primitives` (see
/// [`link_mass_properties`]).
fn robot_body(
    link: &Link,
    robot: &[HullMesh],
    primitives: &[PrimitiveSolid],
    density: f64,
    context: &str,
) -> Option<Body> {
    let solid = solid_of(robot, primitives, context);
    if let Some(urdf) = urdf_body(link) {
        match &solid {
            Some(solid) if urdf.mass / solid.volume < PLACEHOLDER_DENSITY_FRACTION * density => {
                log::debug!(
                    "{context}: the URDF mass {} kg over the link's {} m³ is a placeholder; \
                     deriving the mass at {} kg/m³",
                    urdf.mass,
                    solid.volume,
                    density
                );
            }
            _ => return Some(urdf),
        }
    }
    solid.map(|solid| solid.body(density))
}

/// `link`'s URDF inertial as a body in the link's frame, when it is usable: its mass finite and > 0,
/// its origin and tensor finite. URDF gives the tensor in the inertial frame, which the origin's
/// rpy turns from the link's: in the link's axes it is `R · I · Rᵀ`.
fn urdf_body(link: &Link) -> Option<Body> {
    let inertial = link.inertial.as_ref()?;
    let i = &inertial.inertia;
    let tensor = [
        [i.ixx, i.ixy, i.ixz],
        [i.ixy, i.iyy, i.iyz],
        [i.ixz, i.iyz, i.izz],
    ];
    let origin = &inertial.origin;
    let finite = tensor
        .iter()
        .flatten()
        .chain(&origin.xyz)
        .chain(&origin.rpy)
        .all(|v| v.is_finite());
    if !(finite && inertial.mass.is_finite() && inertial.mass > 0.0) {
        return None;
    }
    let turn = math3::rotation_of(&math3::from_translation_rpy([0.0; 3], origin.rpy));
    Some(Body {
        mass: inertial.mass,
        center_of_mass: origin.xyz,
        inertia: math3::conjugate(&turn, &tensor),
    })
}

/// `bodies` (each of positive mass) as one, diagonalised: see [`link_mass_properties`]; `None`
/// without any. A single body keeps its mass and centre of mass exactly.
fn combine(bodies: &[Body]) -> Option<MassProps> {
    let (mass, center_of_mass, tensor) = match bodies {
        [] => return None,
        [body] => (body.mass, body.center_of_mass, body.inertia),
        _ => {
            let mass: f64 = bodies.iter().map(|b| b.mass).sum();
            let moment = bodies.iter().fold([0.0; 3], |acc, b| {
                math3::add(acc, math3::scale(b.center_of_mass, b.mass))
            });
            let center_of_mass = math3::scale(moment, 1.0 / mass);
            let tensor = bodies.iter().fold([[0.0; 3]; 3], |acc, b| {
                math3::mat3_add(
                    &acc,
                    &parallel_axis(
                        &b.inertia,
                        b.mass,
                        math3::sub(b.center_of_mass, center_of_mass),
                    ),
                )
            });
            (mass, center_of_mass, tensor)
        }
    };
    let (diagonal_inertia, axes) = math3::eig3(&tensor);
    let near_identity = (0..3).all(|i| {
        (0..3).all(|j| {
            (axes[i][j] - math3::IDENTITY3[i][j]).abs() < PRINCIPAL_AXES_IDENTITY_TOLERANCE
        })
    });
    Some(MassProps {
        mass,
        center_of_mass,
        diagonal_inertia,
        principal_axes: if near_identity {
            math3::IDENTITY3
        } else {
            axes
        },
    })
}

/// The inertia tensor `about_com` (of a body of `mass`) moved to a point `offset` away from its
/// centre of mass: `about_com + mass · (|d|²·E − d·dᵀ)`, `d = offset`.
fn parallel_axis(about_com: &Mat3, mass: f64, offset: Vec3) -> Mat3 {
    let d2 = math3::dot(offset, offset);
    let shift = math3::mat3_add(
        &math3::diag([d2, d2, d2]),
        &math3::mat3_scale(&math3::outer(offset, offset), -1.0),
    );
    math3::mat3_add(about_com, &math3::mat3_scale(&shift, mass))
}

/// How a mass is distributed, per kilogram: its centre and its inertia tensor about it (m²).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassDistribution {
    pub centroid: Vec3,
    pub unit_inertia: Mat3,
}

/// FR-159: the manifest payload `mass_kg` in its link's frame. An explicit centre of mass
/// (`center_of_mass_m`, link frame) and inertia (`inertia_kgm2`, principal moments about the centre
/// of mass in link axes) are taken as given; whichever is missing comes from spreading the mass over
/// `hulls` and `primitives` (the tool's collision geometry in the link's frame) at uniform density
/// ([`hull_mass_distribution`]): their centroid, and their inertia about it. `context` names the
/// robot in warnings.
pub fn payload_body(
    mass_kg: f64,
    center_of_mass_m: Option<Vec3>,
    inertia_kgm2: Option<Vec3>,
    hulls: &[HullMesh],
    primitives: &[PrimitiveSolid],
    context: &str,
) -> Body {
    let spread = (center_of_mass_m.is_none() || inertia_kgm2.is_none())
        .then(|| hull_mass_distribution(hulls, primitives, context));
    let center_of_mass = center_of_mass_m
        .or_else(|| spread.map(|d| d.centroid))
        .unwrap_or([0.0; 3]);
    let inertia = match (inertia_kgm2, spread) {
        (Some(moments), _) => math3::diag(moments),
        (None, Some(spread)) => math3::mat3_scale(&spread.unit_inertia, mass_kg),
        (None, None) => math3::diag([point_mass_unit_inertia() * mass_kg; 3]),
    };
    Body {
        mass: mass_kg,
        center_of_mass,
        inertia,
    }
}

/// FR-159: a unit mass spread over `hulls` and `primitives` at uniform density: the centroid of the
/// solid they enclose and its inertia about it ([`solid_of`]). Without any volume the mass is a
/// point mass — a solid sphere of [`POINT_MASS_RADIUS_M`], so its inertia is never zero — at the
/// hull vertices' centroid, or at the link origin without vertices. `context` names the robot in
/// warnings.
pub fn hull_mass_distribution(
    hulls: &[HullMesh],
    primitives: &[PrimitiveSolid],
    context: &str,
) -> MassDistribution {
    if let Some(solid) = solid_of(hulls, primitives, context) {
        return MassDistribution {
            centroid: solid.centroid,
            unit_inertia: solid.unit_inertia,
        };
    }
    let point = math3::diag([point_mass_unit_inertia(); 3]);
    let points: Vec<Vec3> = hulls
        .iter()
        .flat_map(|h| h.points.iter().copied())
        .collect();
    match mean(&points) {
        Some(centroid) => {
            log::warn!(
                "{context}: the tool hulls enclose no volume; the payload is a point mass at their vertex centroid"
            );
            MassDistribution {
                centroid,
                unit_inertia: point,
            }
        }
        None => {
            log::warn!(
                "{context}: no tool geometry to spread the payload over; it is a point mass at the link origin \
                 (give the payload's center_of_mass_m and inertia_kgm2, or name its link with payload.link)"
            );
            MassDistribution {
                centroid: [0.0; 3],
                unit_inertia: point,
            }
        }
    }
}

/// The inertia per kilogram (m²) of a solid sphere of [`POINT_MASS_RADIUS_M`] about any axis
/// through its centre: 2/5·r².
fn point_mass_unit_inertia() -> f64 {
    0.4 * POINT_MASS_RADIUS_M * POINT_MASS_RADIUS_M
}

/// A solid: its volume (m³), centroid (m) and inertia per unit mass about the centroid (m²).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Solid {
    volume: f64,
    centroid: Vec3,
    unit_inertia: Mat3,
}

impl Solid {
    /// The solid as a body of uniform `density` (kg/m³).
    fn body(&self, density: f64) -> Body {
        let mass = density * self.volume;
        Body {
            mass,
            center_of_mass: self.centroid,
            inertia: math3::mat3_scale(&self.unit_inertia, mass),
        }
    }
}

/// The solid `meshes` and `primitives` enclose: by the divergence theorem per closed triangle mesh
/// (each triangle and an apex inside the mesh span a signed tetrahedron; an inward-wound mesh
/// counts with its sign flipped), analytically per primitive; `None` without any volume. A
/// degenerate mesh (fewer than 4 points or triangles, or no enclosed volume) is skipped with a
/// warning; `context` names the link in it.
fn solid_of(meshes: &[HullMesh], primitives: &[PrimitiveSolid], context: &str) -> Option<Solid> {
    let points: Vec<Vec3> = meshes
        .iter()
        .flat_map(|h| h.points.iter().copied())
        .chain(primitives.iter().map(|p| p.centroid))
        .collect();
    // Every moment is taken about `reference` (the mean vertex), which keeps the sums well
    // conditioned however far the meshes are from the link origin.
    let reference = mean(&points)?;
    let mut total = SolidMoments::default();
    for mesh in meshes {
        match solid_moments(mesh, reference) {
            Some(moments) => total = total.plus(&moments),
            None => log::warn!(
                "{context}: skipping the degenerate mesh {} (it encloses no volume)",
                mesh.name
            ),
        }
    }
    for primitive in primitives {
        total = total.plus(&primitive.moments(reference));
    }
    if !total.volume.is_finite() || total.volume <= 0.0 {
        return None;
    }
    let centroid = math3::scale(total.first, 1.0 / total.volume);
    // The second moment about the centroid, per unit volume: ∫x·xᵀ dV / V − c·cᵀ.
    let spread = math3::mat3_add(
        &math3::mat3_scale(&total.second, 1.0 / total.volume),
        &math3::mat3_scale(&math3::outer(centroid, centroid), -1.0),
    );
    let trace = spread[0][0] + spread[1][1] + spread[2][2];
    Some(Solid {
        volume: total.volume,
        centroid: math3::add(reference, centroid),
        // I = ρ·(tr(C)·E − C), per kilogram.
        unit_inertia: math3::mat3_add(
            &math3::diag([trace, trace, trace]),
            &math3::mat3_scale(&spread, -1.0),
        ),
    })
}

/// A solid's volume and first and second volume moments, `∫x dV` and `∫x·xᵀ dV`, about some point.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct SolidMoments {
    volume: f64,
    first: Vec3,
    second: Mat3,
}

impl SolidMoments {
    fn plus(&self, other: &SolidMoments) -> SolidMoments {
        SolidMoments {
            volume: self.volume + other.volume,
            first: math3::add(self.first, other.first),
            second: math3::mat3_add(&self.second, &other.second),
        }
    }
}

/// The [`SolidMoments`] about `reference` of the solid `mesh` encloses (see [`solid_of`]); `None`
/// for a degenerate mesh.
fn solid_moments(mesh: &HullMesh, reference: Vec3) -> Option<SolidMoments> {
    if mesh.points.len() < 4 || mesh.triangle_indices.len() < 12 {
        return None;
    }
    let apex = mean(&mesh.points)?;
    let (mut lo, mut hi) = (mesh.points[0], mesh.points[0]);
    for p in &mesh.points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let size = math3::norm(math3::sub(hi, lo));
    // About the apex: a tetrahedron (apex, a, b, c) has the signed volume v = a·(b×c)/6, the first
    // moment v·s/4 and the second moment v/20·(a·aᵀ + b·bᵀ + c·cᵀ + s·sᵀ), s = a + b + c.
    let mut moments = SolidMoments::default();
    for triangle in mesh.triangle_indices.chunks_exact(3) {
        let corner = |i: u32| mesh.points.get(i as usize).map(|p| math3::sub(*p, apex));
        let (Some(a), Some(b), Some(c)) = (
            corner(triangle[0]),
            corner(triangle[1]),
            corner(triangle[2]),
        ) else {
            continue;
        };
        let v = math3::dot(a, math3::cross(b, c)) / 6.0;
        let s = math3::add(math3::add(a, b), c);
        let products = [
            math3::outer(a, a),
            math3::outer(b, b),
            math3::outer(c, c),
            math3::outer(s, s),
        ];
        let sum = products
            .iter()
            .fold([[0.0; 3]; 3], |acc, p| math3::mat3_add(&acc, p));
        moments = moments.plus(&SolidMoments {
            volume: v,
            first: math3::scale(s, v / 4.0),
            second: math3::mat3_scale(&sum, v / 20.0),
        });
    }
    if !moments.volume.is_finite() || moments.volume.abs() <= 1e-9 * size.powi(3) {
        return None;
    }
    if moments.volume < 0.0 {
        // Wound inward: the same solid, every moment with the opposite sign.
        moments = SolidMoments {
            volume: -moments.volume,
            first: math3::scale(moments.first, -1.0),
            second: math3::mat3_scale(&moments.second, -1.0),
        };
    }
    // From the apex to `reference`: x = y + o, o = apex − reference, so
    // ∫x dV = ∫y dV + V·o and ∫x·xᵀ dV = ∫y·yᵀ dV + ∫y dV·oᵀ + o·∫yᵀ dV + V·o·oᵀ.
    let o = math3::sub(apex, reference);
    let second = [
        moments.second,
        math3::outer(moments.first, o),
        math3::outer(o, moments.first),
        math3::mat3_scale(&math3::outer(o, o), moments.volume),
    ]
    .iter()
    .fold([[0.0; 3]; 3], |acc, p| math3::mat3_add(&acc, p));
    Some(SolidMoments {
        volume: moments.volume,
        first: math3::add(moments.first, math3::scale(o, moments.volume)),
        second,
    })
}

/// The mean of `points`; `None` without any.
fn mean(points: &[Vec3]) -> Option<Vec3> {
    if points.is_empty() {
        return None;
    }
    let sum = points.iter().fold([0.0; 3], |acc, p| math3::add(acc, *p));
    Some(math3::scale(sum, 1.0 / points.len() as f64))
}

// =============================================================================
// Drives (FR-154, FR-158)
// =============================================================================

/// FR-154: the angular drive stiffness, per degree (8.6e6 N·m/rad). A linear drive's is the mass it
/// moves over [`LINEAR_DRIVE_SAG_M`].
pub const DRIVE_STIFFNESS: f64 = 1.5e5;

/// FR-158: a linear (prismatic) drive's stiffness, N/m, is the weight of the mass M it moves over
/// this static sag (m): `k = M·g/δ`, so the axis gives 1 mm holding its load against gravity and
/// its natural frequency `√(k/M) = √(g/δ)` is about 99 rad/s (15.8 Hz), whatever it carries — the
/// natural frequency the angular drives' 8.6e6 N·m/rad has on a moved inertia of about
/// 880 kg·m², a large arm's. A fixed stiffness would leave a heavy carriage soft and a light one
/// stiff.
pub const LINEAR_DRIVE_SAG_M: f64 = 1e-3;

/// FR-158: the damping ratio every drive is tuned to, on the composite inertia it moves at home.
pub const DRIVE_DAMPING_RATIO: f64 = 0.7;

/// FR-158: URDF efforts at or above this are placeholders (the dresser writes FLT_MAX), not limits.
pub const PLACEHOLDER_EFFORT: f64 = 1e30;

/// Gravity (m/s²) of the derived force cap's holding torque: the PhysicsScene's.
pub const GRAVITY: f64 = 9.81;

/// FR-158: the derived force cap accelerates the moved inertia from rest to the velocity limit in
/// this time (s).
pub const CAP_ACCELERATION_TIME_S: f64 = 0.25;

/// FR-158: the velocity (rad/s; m/s for prismatic joints) the derived force cap assumes when the
/// URDF gives none.
pub const CAP_FALLBACK_VELOCITY: f64 = 1.0;

/// FR-158: an actuated joint's drive, in UsdPhysics' units (angular per degree, linear per metre).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveParams {
    /// The home value (degrees; metres for prismatic joints): the drive target and the initial
    /// joint state (FR-157).
    pub home: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub max_force: f64,
    /// `physxJoint:maxJointVelocity` (deg/s; m/s for prismatic joints): the URDF velocity limit;
    /// `None` when the URDF gives no positive one.
    pub max_joint_velocity: Option<f64>,
    /// The composite inertia (kg·m²; for prismatic joints the mass, kg) the joint moves at home.
    pub moved_inertia: f64,
}

/// FR-158: the drive of the actuated `joint` of `robot` at its home value `home`
/// ([`actuated_home_values`]), from every link's pose at the home pose (`link_poses`, a common
/// frame) and mass properties (`masses`, [`link_mass_properties`]), both by link name:
/// - stiffness [`DRIVE_STIFFNESS`] per degree; for a prismatic joint `M·g/δ` per metre, M the
///   moved mass and δ [`LINEAR_DRIVE_SAG_M`];
/// - damping `2·ζ·√(k·I)` with ζ = [`DRIVE_DAMPING_RATIO`], k the stiffness in SI units (per radian)
///   and I the composite inertia the joint moves at home ([`composite_inertia_about_axis`]; for a
///   prismatic joint the moved mass), written per degree;
/// - the max force: the URDF effort when it's a limit (finite, > 0 and below
///   [`PLACEHOLDER_EFFORT`]), else [`derived_force_cap`];
/// - the max joint velocity: the URDF velocity in deg/s (m/s for prismatic joints), if positive.
pub fn drive_params(
    robot: &UrdfRobot,
    joint: &Joint,
    home: f64,
    link_poses: &HashMap<String, Mat4>,
    masses: &HashMap<String, MassProps>,
) -> DriveParams {
    let linear = joint.joint_type == JointType::Prismatic;
    let moved: Vec<(Mat4, MassProps, f64)> = moved_links(robot, joint)
        .into_iter()
        .filter_map(|(link, path)| Some((*link_poses.get(link)?, *masses.get(link)?, path)))
        .collect();
    let moved_inertia = if linear {
        moved.iter().map(|(_, props, _)| props.mass).sum()
    } else {
        // The axis runs through the child link's origin (the joint frame) along the joint axis,
        // which turning about it doesn't change.
        let child = link_poses
            .get(&joint.child)
            .copied()
            .unwrap_or(math3::IDENTITY);
        let bodies: Vec<(Mat4, MassProps)> = moved
            .iter()
            .map(|(pose, props, _)| (*pose, *props))
            .collect();
        composite_inertia_about_axis(
            &bodies,
            math3::translation_of(&child),
            math3::rotate(&child, unit_joint_axis(joint.axis)),
        )
    };
    // Gravity's torque is at most g·Σ m_i·r_i in any pose; a linear joint holds the moved weight.
    let holding = GRAVITY
        * moved
            .iter()
            .map(|(_, props, path)| {
                let lever = if linear {
                    1.0
                } else {
                    path + math3::norm(props.center_of_mass)
                };
                props.mass * lever
            })
            .sum::<f64>();
    // The drive's stiffness as authored (per degree; per metre for a prismatic joint) and in SI
    // units (per radian; per metre).
    let (stiffness, stiffness_si) = if linear {
        let k = moved_inertia.max(FRAME_LINK_MASS_KG) * GRAVITY / LINEAR_DRIVE_SAG_M;
        (k, k)
    } else {
        (DRIVE_STIFFNESS, DRIVE_STIFFNESS * 180.0 / PI)
    };
    let damping_si = 2.0 * DRIVE_DAMPING_RATIO * (stiffness_si * moved_inertia).max(0.0).sqrt();
    let limit = joint.limit.as_ref();
    let velocity = limit
        .map(|l| l.velocity)
        .filter(|v| v.is_finite() && *v > 0.0);
    let effort = limit
        .map(|l| l.effort)
        .filter(|e| e.is_finite() && *e > 0.0 && *e < PLACEHOLDER_EFFORT);
    DriveParams {
        home,
        stiffness,
        damping: if linear {
            damping_si
        } else {
            damping_si * PI / 180.0
        },
        max_force: effort.unwrap_or_else(|| {
            derived_force_cap(
                holding,
                moved_inertia,
                damping_si,
                velocity.unwrap_or(CAP_FALLBACK_VELOCITY),
            )
        }),
        max_joint_velocity: velocity.map(|v| if linear { v } else { v.to_degrees() }),
        moved_inertia,
    }
}

/// FR-158: a force cap for a joint whose URDF gives no real effort limit, high enough not to limit
/// nominal tracking but finite: `max(1, 2·τ_hold + I·v/0.25 s + D·v)`, in SI units (N·m, kg·m²,
/// rad/s, N·m·s/rad; N, kg, m/s, N·s/m for prismatic joints). Twice the worst-case holding torque
/// `holding` (τ_hold = g·Σ m_i·r_i over the moved links, r_i the summed length of the joint offsets
/// down to link i plus that of its centre of mass), the torque that brings the moved inertia from
/// rest to the velocity limit `velocity` in [`CAP_ACCELERATION_TIME_S`], and the damping drag at
/// that velocity: the drive tracks position commands only, against its own damping.
pub fn derived_force_cap(holding: f64, moved_inertia: f64, damping_si: f64, velocity: f64) -> f64 {
    (2.0 * holding + moved_inertia * velocity / CAP_ACCELERATION_TIME_S + damping_si * velocity)
        .max(1.0)
}

/// FR-158: the composite moment of inertia (kg·m²) of rigid `bodies`, each its pose in a common
/// frame and its mass properties, about the line through `point` along the unit vector `axis`, in
/// that frame: `Σ aᵀ·(R_i·I_i·R_iᵀ)·a + m_i·|d⊥,i|²`, with `d⊥,i` the offset of body i's centre of
/// mass from the line, perpendicular to it.
pub fn composite_inertia_about_axis(bodies: &[(Mat4, MassProps)], point: Vec3, axis: Vec3) -> f64 {
    bodies
        .iter()
        .map(|(pose, props)| {
            let tensor = math3::conjugate(&math3::rotation_of(pose), &props.inertia_tensor());
            let own = math3::dot(axis, math3::apply3(&tensor, axis));
            let d = math3::sub(math3::transform_point(pose, props.center_of_mass), point);
            let perpendicular = math3::sub(d, math3::scale(axis, math3::dot(d, axis)));
            own + props.mass * math3::dot(perpendicular, perpendicular)
        })
        .sum()
}

/// The links `joint` moves: its child and every link below, depth first in URDF declaration
/// order, each with the summed length of the joint offsets from the child down to it.
fn moved_links<'a>(robot: &'a UrdfRobot, joint: &'a Joint) -> Vec<(&'a str, f64)> {
    let mut out = Vec::new();
    let mut seen: HashSet<&str> = HashSet::new();
    let mut stack = vec![(joint.child.as_str(), 0.0)];
    while let Some((link, path)) = stack.pop() {
        if !seen.insert(link) {
            continue;
        }
        out.push((link, path));
        // Reversed, so the children pop in declaration order.
        for child in robot.joints.iter().rev().filter(|j| j.parent == link) {
            stack.push((child.child.as_str(), path + math3::norm(child.origin.xyz)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use urdf_robo_dresser::{InertiaMatrix, Inertial, JointLimit, Origin};

    fn link(name: &str, inertial: Option<(f64, Vec3, Vec3)>) -> Link {
        Link {
            name: name.into(),
            inertial: inertial.map(|(mass, com, [ixx, iyy, izz])| Inertial {
                origin: Origin {
                    xyz: com,
                    rpy: [0.0; 3],
                },
                mass,
                inertia: InertiaMatrix {
                    ixx,
                    ixy: 0.0,
                    ixz: 0.0,
                    iyy,
                    iyz: 0.0,
                    izz,
                },
            }),
            visuals: vec![],
            collisions: vec![],
        }
    }

    /// (lower, upper, effort, velocity)
    type Limit = (f64, f64, f64, f64);

    fn joint(
        name: &str,
        joint_type: JointType,
        parent: &str,
        child: &str,
        xyz: Vec3,
        axis: Vec3,
        limit: Option<Limit>,
    ) -> Joint {
        Joint {
            name: name.into(),
            joint_type,
            parent: parent.into(),
            child: child.into(),
            origin: Origin { xyz, rpy: [0.0; 3] },
            axis,
            limit: limit.map(|(lower, upper, effort, velocity)| JointLimit {
                lower,
                upper,
                effort,
                velocity,
            }),
        }
    }

    fn robot(links: Vec<Link>, joints: Vec<Joint>) -> UrdfRobot {
        UrdfRobot {
            name: "r".into(),
            links,
            joints,
            reach: None,
        }
    }

    fn assert_near(got: f64, want: f64, tol: f64) {
        assert!((got - want).abs() <= tol, "{got} != {want} (±{tol})");
    }

    fn assert_vec_near(got: Vec3, want: Vec3, tol: f64) {
        for k in 0..3 {
            assert_near(got[k], want[k], tol);
        }
    }

    fn assert_mat_near(got: &Mat3, want: &Mat3, tol: f64) {
        for i in 0..3 {
            assert_vec_near(got[i], want[i], tol);
        }
    }

    #[test]
    fn home_values_keep_revolute_unwrapped_and_wrap_continuous() {
        let z = [0.0, 0.0, 1.0];
        let r = robot(
            vec![],
            vec![
                joint("fixed", JointType::Fixed, "w", "a", [0.0; 3], z, None),
                joint("rev", JointType::Revolute, "a", "b", [0.0; 3], z, None),
                joint("cont", JointType::Continuous, "b", "c", [0.0; 3], z, None),
                joint("slide", JointType::Prismatic, "c", "d", [0.0; 3], z, None),
            ],
        );
        // Fixed joints aren't indexed; 190 stays 190 on the revolute joint, wraps on the
        // continuous one; the prismatic joint's entry is metres, as given.
        assert_eq!(
            actuated_home_values(&r, Some(&[190.0, 190.0, 0.3])),
            [190.0, -170.0, 0.3]
        );
        assert_eq!(
            actuated_home_values(&r, Some(&[0.0, 0.0, -1.25])),
            [0.0, 0.0, -1.25]
        );
        // A missing entry is 0.
        assert_eq!(actuated_home_values(&r, Some(&[-45.0])), [-45.0, 0.0, 0.0]);
        // No override: the KUKA stowed default, by actuated joint, on the turning joints only
        // (its third angle, 90°, is no slide): the prismatic joint at 0 m.
        assert_eq!(actuated_home_values(&r, None), [0.0, -90.0, 0.0]);
        // Beyond the default's six joints: 0.
        let seven: Vec<Joint> = (0..7)
            .map(|_| joint("j", JointType::Revolute, "p", "c", [0.0; 3], z, None))
            .collect();
        assert_eq!(
            actuated_home_values(&robot(vec![], seven), None),
            [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn wrap_degrees_into_the_half_open_turn() {
        for (deg, want) in [
            (190.0, -170.0),
            (-190.0, 170.0),
            (180.0, 180.0),
            (-180.0, 180.0),
            (540.0, 180.0),
            (-540.0, 180.0),
            (0.0, 0.0),
            (359.5, -0.5),
            (1080.0, 0.0),
            (-170.0, -170.0),
            (0.1, 0.1),
        ] {
            assert_eq!(wrap_degrees(deg), want, "{deg}");
        }
    }

    /// `link`'s mass properties without geometry or payload, at the default density, as a link
    /// welded to the root.
    fn props(link: &Link) -> MassProps {
        link_mass_properties(
            link,
            &LinkMeshes::default(),
            None,
            DEFAULT_LINK_DENSITY_KG_M3,
            true,
            "t",
        )
    }

    /// [`props`] of a link a joint moves.
    fn moving_props(link: &Link) -> MassProps {
        link_mass_properties(
            link,
            &LinkMeshes::default(),
            None,
            DEFAULT_LINK_DENSITY_KG_M3,
            false,
            "t",
        )
    }

    /// A link with a URDF inertial of `mass` at `com`, its tensor (ixx, ixy, ixz, iyy, iyz, izz)
    /// given in the inertial frame, which `rpy` turns from the link's.
    fn inertial_link(mass: f64, com: Vec3, rpy: Vec3, tensor: [f64; 6]) -> Link {
        let [ixx, ixy, ixz, iyy, iyz, izz] = tensor;
        Link {
            name: "l".into(),
            inertial: Some(Inertial {
                origin: Origin { xyz: com, rpy },
                mass,
                inertia: InertiaMatrix {
                    ixx,
                    ixy,
                    ixz,
                    iyy,
                    iyz,
                    izz,
                },
            }),
            visuals: vec![],
            collisions: vec![],
        }
    }

    /// A 1 × 0.5 × 0.2 m robot box (0.1 m³) at (0.3, 0, 0.1).
    const ROBOT_BOX: (Vec3, Vec3) = ([0.3, 0.0, 0.1], [1.0, 0.5, 0.2]);
    /// A 0.4 × 0.2 × 0.6 m tool box (0.048 m³) at (−0.5, 0, 0.2).
    const TOOL_BOX: (Vec3, Vec3) = ([-0.5, 0.0, 0.2], [0.4, 0.2, 0.6]);

    fn robot_box_meshes() -> LinkMeshes {
        LinkMeshes {
            robot: vec![box_hull(ROBOT_BOX.0, ROBOT_BOX.1, true)],
            ..LinkMeshes::default()
        }
    }

    /// `props` is the box (`center`, `size`) at uniform `density`.
    fn assert_solid_box(props: &MassProps, (center, size): (Vec3, Vec3), density: f64) {
        let mass = density * size[0] * size[1] * size[2];
        assert_near(props.mass, mass, 1e-9);
        assert_vec_near(props.center_of_mass, center, 1e-12);
        assert_mat_near(
            &props.inertia_tensor(),
            &math3::mat3_scale(&box_unit_inertia(size), mass),
            1e-9,
        );
    }

    #[test]
    fn a_urdf_inertial_is_kept_unless_its_implied_density_marks_a_placeholder() {
        let meshes = robot_box_meshes();
        let with_mass = |mass: f64| link("l", Some((mass, [0.5, 0.0, 0.0], [1.0, 2.0, 3.0])));
        // At 1000 kg/m³ the placeholder line of the 0.1 m³ box is a quarter of it, 25 kg
        // (250 kg/m³). 30 kg (300 kg/m³) is above it: the inertial as authored, on the link's own
        // axes.
        for mass in [30.0, 25.5, 5000.0] {
            let p = link_mass_properties(&with_mass(mass), &meshes, None, 1000.0, true, "t");
            assert_eq!(
                (p.mass, p.center_of_mass, p.diagonal_inertia),
                (mass, [0.5, 0.0, 0.0], [1.0, 2.0, 3.0])
            );
            assert_eq!(p.principal_axes, math3::IDENTITY3);
        }
        // 2 kg (20 kg/m³) and 24.5 kg are placeholders: the box at 1000 kg/m³, 100 kg.
        for mass in [2.0, 24.5] {
            let p = link_mass_properties(&with_mass(mass), &meshes, None, 1000.0, true, "t");
            assert_solid_box(&p, ROBOT_BOX, 1000.0);
        }
        // Without an inertial, or with an unusable one, the same.
        let p = link_mass_properties(&link("l", None), &meshes, None, 1000.0, true, "t");
        assert_solid_box(&p, ROBOT_BOX, 1000.0);
        for mass in [0.0, -3.0, f64::NAN, f64::INFINITY] {
            let p = link_mass_properties(&with_mass(mass), &meshes, None, 1000.0, true, "t");
            assert_solid_box(&p, ROBOT_BOX, 1000.0);
        }
        // Without robot geometry every usable inertial is kept, 2 kg too.
        let p = props(&with_mass(2.0));
        assert_eq!((p.mass, p.center_of_mass), (2.0, [0.5, 0.0, 0.0]));
        // Neither geometry nor a usable inertial, welded to the root: 10 kg, a unit inertia, at
        // the origin.
        for none in [with_mass(f64::NAN), link("World", None)] {
            let p = props(&none);
            assert_eq!(p, MassProps::fallback());
            assert_eq!(
                (p.mass, p.center_of_mass, p.diagonal_inertia),
                (10.0, [0.0; 3], [1.0; 3])
            );
            assert_eq!(p.inertia_tensor(), math3::IDENTITY3);
        }
        // Degenerate robot meshes enclose nothing: no geometry.
        let flat = LinkMeshes {
            robot: vec![box_hull([0.0; 3], [1.0, 1.0, 0.0], true)],
            ..LinkMeshes::default()
        };
        let p = link_mass_properties(&with_mass(2.0), &flat, None, 1000.0, true, "t");
        assert_eq!(p.mass, 2.0);
    }

    #[test]
    fn a_frame_link_a_joint_moves_weighs_almost_nothing() {
        // A flange / tool0 / TCP frame: neither geometry nor a usable inertial, moved by a joint:
        // 0.1 kg as a 5 cm solid sphere (2/5·m·r² = 1e-4 kg·m²) at its origin, not the welded
        // root's 10 kg.
        for frame in [
            link("tool0", None),
            link("flange", Some((f64::NAN, [0.0; 3], [1.0; 3]))),
        ] {
            let p = moving_props(&frame);
            assert_eq!(p, MassProps::frame());
            assert_eq!((p.mass, p.center_of_mass), (FRAME_LINK_MASS_KG, [0.0; 3]));
            assert_eq!(FRAME_LINK_MASS_KG, 0.1);
            assert_vec_near(p.diagonal_inertia, [1e-4; 3], 1e-18);
            assert_eq!(p.principal_axes, math3::IDENTITY3);
        }
        // Geometry or an inertial, it weighs what they make, welded or not.
        let p = moving_props(&link("l6", Some((12.0, [0.0; 3], [0.1; 3]))));
        assert_eq!(p.mass, 12.0);
        let p = link_mass_properties(
            &link("l", None),
            &robot_box_meshes(),
            None,
            1000.0,
            false,
            "t",
        );
        assert_solid_box(&p, ROBOT_BOX, 1000.0);
    }

    #[test]
    fn primitives_weigh_their_analytic_solids() {
        let pose = math3::from_translation_rpy([0.2, -0.1, 0.4], [0.3, -0.5, 1.1]);
        let turn = math3::rotation_of(&pose);
        // A 0.4 × 0.2 × 0.6 m box: as the box hull of the same size and pose.
        let cube = PrimitiveSolid::of(
            &Geometry::Box {
                size: [0.4, 0.2, 0.6],
            },
            &pose,
            "box",
        )
        .unwrap();
        let mut hull = box_hull([0.0; 3], [0.4, 0.2, 0.6], true);
        for p in &mut hull.points {
            *p = math3::transform_point(&pose, *p);
        }
        let from_hull = link_mass_properties(
            &link("l", None),
            &LinkMeshes {
                robot: vec![hull],
                ..LinkMeshes::default()
            },
            None,
            1000.0,
            false,
            "t",
        );
        let from_box = link_mass_properties(
            &link("l", None),
            &LinkMeshes {
                robot_primitives: vec![cube],
                ..LinkMeshes::default()
            },
            None,
            1000.0,
            false,
            "t",
        );
        assert_near(from_box.mass, 48.0, 1e-9);
        assert_near(from_box.mass, from_hull.mass, 1e-9);
        assert_vec_near(from_box.center_of_mass, from_hull.center_of_mass, 1e-12);
        assert_mat_near(
            &from_box.inertia_tensor(),
            &from_hull.inertia_tensor(),
            1e-9,
        );
        // A cylinder of radius 0.1 m and length 0.5 m along its Z: πr²l, diag(r²/4 + l²/12, …,
        // r²/2) per kilogram, turned by the pose.
        let (r, l) = (0.1, 0.5);
        let cylinder = PrimitiveSolid::of(
            &Geometry::Cylinder {
                radius: r,
                length: l,
            },
            &pose,
            "c",
        )
        .unwrap();
        let p = link_mass_properties(
            &link("l", None),
            &LinkMeshes {
                robot_primitives: vec![cylinder],
                ..LinkMeshes::default()
            },
            None,
            1000.0,
            false,
            "t",
        );
        let mass = 1000.0 * PI * r * r * l;
        assert_near(p.mass, mass, 1e-9);
        assert_vec_near(p.center_of_mass, [0.2, -0.1, 0.4], 1e-12);
        let local = math3::diag([
            mass * (r * r / 4.0 + l * l / 12.0),
            mass * (r * r / 4.0 + l * l / 12.0),
            mass * r * r / 2.0,
        ]);
        assert_mat_near(&p.inertia_tensor(), &math3::conjugate(&turn, &local), 1e-9);
        // A sphere of radius 0.2 m: 4/3·πr³, 2/5·m·r² about every axis.
        let sphere = PrimitiveSolid::of(&Geometry::Sphere { radius: 0.2 }, &pose, "s").unwrap();
        let p = link_mass_properties(
            &link("l", None),
            &LinkMeshes {
                tool_primitives: vec![sphere.clone()],
                ..LinkMeshes::default()
            },
            None,
            1000.0,
            false,
            "t",
        );
        let mass = 1000.0 * 4.0 / 3.0 * PI * 0.008;
        assert_near(p.mass, mass, 1e-9);
        assert_vec_near(p.diagonal_inertia, [0.4 * mass * 0.04; 3], 1e-9);
        // A payload spreads over tool primitives too.
        let spread = hull_mass_distribution(&[], &[sphere], "t");
        assert_vec_near(spread.centroid, [0.2, -0.1, 0.4], 1e-12);
        assert_mat_near(&spread.unit_inertia, &math3::diag([0.4 * 0.04; 3]), 1e-12);
        // Moved into another frame: the same solid there.
        let shift = math3::from_translation_rpy([1.0, 2.0, 3.0], [0.0, 0.0, PI / 2.0]);
        let moved = PrimitiveSolid::of(
            &Geometry::Box {
                size: [0.4, 0.2, 0.6],
            },
            &pose,
            "b",
        )
        .unwrap()
        .transformed(&shift);
        let direct = PrimitiveSolid::of(
            &Geometry::Box {
                size: [0.4, 0.2, 0.6],
            },
            &math3::multiply(&shift, &pose),
            "b",
        )
        .unwrap();
        assert_vec_near(moved.centroid, direct.centroid, 1e-12);
        assert_mat_near(&moved.spread, &direct.spread, 1e-12);
        // Degenerate primitives and meshes aren't solids.
        for bad in [
            Geometry::Box {
                size: [1.0, 0.0, 1.0],
            },
            Geometry::Cylinder {
                radius: -1.0,
                length: 1.0,
            },
            Geometry::Sphere { radius: f64::NAN },
            Geometry::Mesh {
                filename: "m.obj".into(),
                scale: [1.0; 3],
            },
        ] {
            assert_eq!(PrimitiveSolid::of(&bad, &math3::IDENTITY, "x"), None);
        }
    }

    #[test]
    fn the_link_density_scales_the_derived_mass_and_the_placeholder_line() {
        let meshes = robot_box_meshes();
        let urdf = link("l", Some((30.0, [0.5, 0.0, 0.0], [1.0, 2.0, 3.0])));
        // 30 kg over 0.1 m³ is kept at 1000 kg/m³ (line 25 kg), a placeholder at 2000 kg/m³
        // (line 50 kg): then the box weighs 200 kg.
        assert_eq!(
            link_mass_properties(&urdf, &meshes, None, 1000.0, true, "t").mass,
            30.0
        );
        let p = link_mass_properties(&urdf, &meshes, None, 2000.0, true, "t");
        assert_solid_box(&p, ROBOT_BOX, 2000.0);
        // A placeholder at the default density: the box at the default.
        let p = link_mass_properties(
            &link("l", None),
            &meshes,
            None,
            DEFAULT_LINK_DENSITY_KG_M3,
            true,
            "t",
        );
        assert_solid_box(&p, ROBOT_BOX, DEFAULT_LINK_DENSITY_KG_M3);
        // The manifest's density when finite and > 0, else the default.
        assert_eq!(link_density(Some(2700.0)), 2700.0);
        assert_eq!(link_density(None), DEFAULT_LINK_DENSITY_KG_M3);
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(link_density(Some(bad)), DEFAULT_LINK_DENSITY_KG_M3);
        }
        assert_eq!(DEFAULT_LINK_DENSITY_KG_M3, 1200.0);
        assert_eq!(PLACEHOLDER_DENSITY_FRACTION, 0.25);
    }

    #[test]
    fn a_urdf_tensor_is_turned_into_the_link_axes_by_its_rpy() {
        let tensor = [2.0, 0.3, -0.2, 3.0, 0.1, 4.0];
        let full = [[2.0, 0.3, -0.2], [0.3, 3.0, 0.1], [-0.2, 0.1, 4.0]];
        let rpy = [0.4, -0.3, 1.1];
        let p = props(&inertial_link(5.0, [0.1, -0.2, 0.3], rpy, tensor));
        assert_eq!((p.mass, p.center_of_mass), (5.0, [0.1, -0.2, 0.3]));
        // In the link's axes: R · I · Rᵀ, R the rpy's rotation, the products of inertia included.
        let r = math3::rotation_of(&math3::from_translation_rpy([0.0; 3], rpy));
        assert_mat_near(&p.inertia_tensor(), &math3::conjugate(&r, &full), 1e-12);
        // Diagonalised: principal moments (their sum, the trace, unchanged) on authored axes, a
        // proper rotation.
        let [a, b, c] = p.diagonal_inertia;
        assert_near(a + b + c, 9.0, 1e-12);
        assert!((math3::det3(&p.principal_axes) - 1.0).abs() < 1e-12);
        let [w, x, y, z] = p.principal_axes_wxyz();
        assert_near(w * w + x * x + y * y + z * z, 1.0, 1e-12);
        assert!(w < 1.0 - 1e-6, "{w} {x} {y} {z}");
        // Products of inertia without an rpy are diagonalised too.
        let p = props(&inertial_link(
            5.0,
            [0.0; 3],
            [0.0; 3],
            [2.0, 0.5, 0.0, 2.0, 0.0, 1.0],
        ));
        assert_mat_near(
            &p.inertia_tensor(),
            &[[2.0, 0.5, 0.0], [0.5, 2.0, 0.0], [0.0, 0.0, 1.0]],
            1e-12,
        );
        let mut moments = p.diagonal_inertia;
        moments.sort_by(f64::total_cmp);
        assert_vec_near(moments, [1.0, 1.5, 2.5], 1e-12);
        // A diagonal tensor without an rpy is authored as is, on the link's own axes.
        let p = props(&inertial_link(
            5.0,
            [0.0; 3],
            [0.0; 3],
            [0.1, 0.0, 0.0, 0.2, 0.0, 0.3],
        ));
        assert_eq!(
            (p.diagonal_inertia, p.principal_axes),
            ([0.1, 0.2, 0.3], math3::IDENTITY3)
        );
        assert_eq!(p.principal_axes_wxyz(), [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn tool_hulls_weigh_the_link_density_without_a_payload() {
        let (rc, rs) = ROBOT_BOX;
        let (tc, ts) = TOOL_BOX;
        let meshes = LinkMeshes {
            robot: vec![box_hull(rc, rs, true)],
            tool: vec![box_hull(tc, ts, true)],
            ..LinkMeshes::default()
        };
        let placeholder = link("flange", Some((2.0, [0.0; 3], [0.01; 3])));
        let p = link_mass_properties(&placeholder, &meshes, None, 1000.0, false, "t");
        // By hand: the robot box at 1000 kg/m³ (100 kg) and the tool box (48 kg), each its own
        // tensor moved to the common centre.
        let (mr, mt) = (100.0, 48.0);
        assert_near(p.mass, mr + mt, 1e-9);
        let com = math3::scale(
            math3::add(math3::scale(rc, mr), math3::scale(tc, mt)),
            1.0 / (mr + mt),
        );
        assert_vec_near(p.center_of_mass, com, 1e-12);
        let want = math3::mat3_add(
            &parallel_axis(
                &math3::mat3_scale(&box_unit_inertia(rs), mr),
                mr,
                math3::sub(rc, com),
            ),
            &parallel_axis(
                &math3::mat3_scale(&box_unit_inertia(ts), mt),
                mt,
                math3::sub(tc, com),
            ),
        );
        assert_mat_near(&p.inertia_tensor(), &want, 1e-9);
        // The tool hulls don't count in the placeholder test: 30 kg over the robot box alone is
        // kept, the tool box at the density added.
        let real = link("flange", Some((30.0, rc, [1.0, 1.0, 1.0])));
        assert_near(
            link_mass_properties(&real, &meshes, None, 1000.0, false, "t").mass,
            30.0 + mt,
            1e-9,
        );
        // A link with tool hulls only is its tool: no fallback mass on top.
        let tool_only = LinkMeshes {
            tool: meshes.tool.clone(),
            ..LinkMeshes::default()
        };
        let p = link_mass_properties(&link("t", None), &tool_only, None, 1000.0, false, "t");
        assert_solid_box(&p, TOOL_BOX, 1000.0);
    }

    #[test]
    fn a_payload_replaces_the_tool_hulls_density() {
        let (rc, _) = ROBOT_BOX;
        let (tc, ts) = TOOL_BOX;
        let meshes = LinkMeshes {
            robot: vec![box_hull(ROBOT_BOX.0, ROBOT_BOX.1, true)],
            tool: vec![box_hull(tc, ts, true)],
            ..LinkMeshes::default()
        };
        let payload = payload_body(30.0, None, None, &meshes.tool, &[], "t");
        let placeholder = link("flange", Some((2.0, [0.0; 3], [0.01; 3])));
        let p = link_mass_properties(&placeholder, &meshes, Some(&payload), 1000.0, false, "t");
        // The robot box's 100 kg and the payload's 30 kg; the tool box isn't weighed again.
        assert_near(p.mass, 130.0, 1e-9);
        let com = math3::scale(
            math3::add(math3::scale(rc, 100.0), math3::scale(tc, 30.0)),
            1.0 / 130.0,
        );
        assert_vec_near(p.center_of_mass, com, 1e-12);
    }

    #[test]
    fn composite_inertia_about_axis_matches_a_hand_calculation() {
        let a = MassProps {
            mass: 2.0,
            center_of_mass: [1.0, 0.0, 0.0],
            diagonal_inertia: [0.1, 0.2, 0.3],
            principal_axes: math3::IDENTITY3,
        };
        // Rotated 90° about X (link Z → world −Y, link Y → world Z) and moved to (0, 2, 0): the
        // centre of mass (0, 0, 1) lands at (0, 1, 0), and the world ZZ moment is link YY's.
        let b = MassProps {
            mass: 3.0,
            center_of_mass: [0.0, 0.0, 1.0],
            diagonal_inertia: [0.4, 0.5, 0.6],
            principal_axes: math3::IDENTITY3,
        };
        let b_pose = math3::from_translation_rpy([0.0, 2.0, 0.0], [PI / 2.0, 0.0, 0.0]);
        let bodies = [(math3::IDENTITY, a), (b_pose, b)];
        let z = [0.0, 0.0, 1.0];
        // About Z through the origin: (0.3 + 2·1²) + (0.5 + 3·1²).
        assert_near(
            composite_inertia_about_axis(&bodies, [0.0; 3], z),
            5.8,
            1e-12,
        );
        // About Z through (1, 0, 0): a's centre is on it; b's is √2 away.
        assert_near(
            composite_inertia_about_axis(&bodies, [1.0, 0.0, 0.0], z),
            0.3 + 0.5 + 3.0 * 2.0,
            1e-12,
        );
        // About the diagonal (1, 1, 0)/√2 through the origin: a: (0.1 + 0.2)/2 + 2·(1 − ½);
        // b: (0.4 + 0.6)/2 (world XX is link XX, world YY link ZZ) + 3·(1 − ½).
        let diagonal = [0.5f64.sqrt(), 0.5f64.sqrt(), 0.0];
        assert_near(
            composite_inertia_about_axis(&bodies, [0.0; 3], diagonal),
            0.15 + 1.0 + 0.5 + 1.5,
            1e-12,
        );
    }

    /// A robot, its links' poses and their mass properties.
    type Arm = (UrdfRobot, HashMap<String, Mat4>, HashMap<String, MassProps>);

    /// A two-link arm: `base`, and `arm` on a revolute joint about Z at the origin, with a 10 kg,
    /// diag(0.1, 0.2, 0.3) inertial 1 m along X.
    fn one_joint_arm(effort: f64, velocity: f64) -> Arm {
        let r = robot(
            vec![
                link("base", None),
                link("arm", Some((10.0, [1.0, 0.0, 0.0], [0.1, 0.2, 0.3]))),
            ],
            vec![joint(
                "j1",
                JointType::Revolute,
                "base",
                "arm",
                [0.0; 3],
                [0.0, 0.0, 1.0],
                Some((-3.0, 3.0, effort, velocity)),
            )],
        );
        let poses = HashMap::from([
            ("base".to_string(), math3::IDENTITY),
            ("arm".to_string(), math3::IDENTITY),
        ]);
        let masses = r.links.iter().map(|l| (l.name.clone(), props(l))).collect();
        (r, poses, masses)
    }

    #[test]
    fn zeta_damping_and_derived_cap_match_a_hand_calculation() {
        let (r, poses, masses) = one_joint_arm(3.4028234663852886e38, 2.0);
        let drive = drive_params(&r, &r.joints[0], 12.5, &poses, &masses);
        // I = 0.3 + 10·1² = 10.3 kg·m²; k = 1.5e5 N·m/deg = 1.5e5·180/π N·m/rad.
        let inertia = 10.3;
        let k_rad = 1.5e5 * 180.0 / PI;
        let d_rad = 2.0 * 0.7 * (k_rad * inertia).sqrt();
        assert_near(drive.moved_inertia, inertia, 1e-12);
        assert_eq!((drive.home, drive.stiffness), (12.5, 150000.0));
        assert_near(drive.damping, d_rad * PI / 180.0, 1e-9);
        // ≈ 229.9 N·m·s/deg, not 2·0.7·√(1.5e5·10.3) ≈ 1740 (gains mixed per degree and radian).
        assert_near(drive.damping, 229.896, 0.001);
        // The placeholder effort gets the derived cap: τ_hold = 9.81·10·(0 + 1) and v = 2 rad/s.
        let cap = 2.0 * 9.81 * 10.0 + inertia * 2.0 / 0.25 + d_rad * 2.0;
        assert_near(drive.max_force, cap, 1e-6);
        assert_near(drive.max_force, 26622.7, 0.1);
        assert_near(drive.max_joint_velocity.unwrap(), 2.0 * 180.0 / PI, 1e-12);

        // A real effort is the limit; no velocity: no limit, and the cap assumes 1 rad/s.
        let (r, poses, masses) = one_joint_arm(100.0, 0.0);
        let drive = drive_params(&r, &r.joints[0], 0.0, &poses, &masses);
        assert_eq!((drive.max_force, drive.max_joint_velocity), (100.0, None));
        for effort in [0.0, -5.0, f64::INFINITY, f64::NAN, 1e30] {
            let (r, poses, masses) = one_joint_arm(effort, f64::NAN);
            let drive = drive_params(&r, &r.joints[0], 0.0, &poses, &masses);
            let cap = 2.0 * 9.81 * 10.0 + inertia * 1.0 / 0.25 + d_rad;
            assert_near(drive.max_force, cap, 1e-6);
            assert_eq!(drive.max_joint_velocity, None);
        }
    }

    #[test]
    fn the_derived_cap_has_a_floor_of_one() {
        assert_eq!(derived_force_cap(0.0, 0.0, 0.0, 1.0), 1.0);
        assert_eq!(
            derived_force_cap(1.0, 2.0, 3.0, 4.0),
            2.0 + 8.0 / 0.25 + 12.0
        );
    }

    #[test]
    fn the_moved_subtree_and_its_lever_arms_follow_the_chain() {
        // base → l1 (revolute about Z) → l2 0.5 m up (revolute about X) → tool 0.3 m along X
        // (fixed); l1 1 m up from the base.
        let x = [1.0, 0.0, 0.0];
        let z = [0.0, 0.0, 1.0];
        let r = robot(
            vec![
                link("base", None),
                link("l1", Some((4.0, [0.0; 3], [0.1, 0.1, 0.2]))),
                link("l2", Some((2.0, [0.0, 0.0, 0.25], [0.3, 0.3, 0.1]))),
                link("tool", Some((1.0, [0.1, 0.0, 0.0], [0.05, 0.05, 0.05]))),
            ],
            vec![
                joint(
                    "j1",
                    JointType::Revolute,
                    "base",
                    "l1",
                    [0.0, 0.0, 1.0],
                    z,
                    Some((-1.0, 1.0, 1e38, 1.0)),
                ),
                joint(
                    "j2",
                    JointType::Revolute,
                    "l1",
                    "l2",
                    [0.0, 0.0, 0.5],
                    x,
                    Some((-1.0, 1.0, 1e38, 1.0)),
                ),
                joint(
                    "fix",
                    JointType::Fixed,
                    "l2",
                    "tool",
                    [0.3, 0.0, 0.0],
                    z,
                    None,
                ),
            ],
        );
        assert_eq!(
            moved_links(&r, &r.joints[0]),
            [("l1", 0.0), ("l2", 0.5), ("tool", 0.8)]
        );
        assert_eq!(moved_links(&r, &r.joints[1]), [("l2", 0.0), ("tool", 0.3)]);
        // Home pose 0: every frame axis-aligned, so the poses are translations.
        let t = |xyz: Vec3| math3::from_translation_rpy(xyz, [0.0; 3]);
        let poses = HashMap::from([
            ("base".to_string(), t([0.0; 3])),
            ("l1".to_string(), t([0.0, 0.0, 1.0])),
            ("l2".to_string(), t([0.0, 0.0, 1.5])),
            ("tool".to_string(), t([0.3, 0.0, 1.5])),
        ]);
        let masses: HashMap<String, MassProps> =
            r.links.iter().map(|l| (l.name.clone(), props(l))).collect();
        // j1 about Z through (0, 0, 1): l1 0.2; l2 0.1 (on the axis); tool 0.05 + 1·(0.3 + 0.1)².
        let j1 = drive_params(&r, &r.joints[0], 0.0, &poses, &masses);
        assert_near(j1.moved_inertia, 0.2 + 0.1 + 0.05 + 0.16, 1e-12);
        // j2 about X through (0, 0, 1.5): l2 0.3 + 2·0.25²; tool 0.05 + 1·0² (its centre is on X).
        let j2 = drive_params(&r, &r.joints[1], 0.0, &poses, &masses);
        assert_near(j2.moved_inertia, 0.3 + 2.0 * 0.0625 + 0.05, 1e-12);
        // Holding torque bound for j1: 9.81·(4·0 + 2·(0.5 + 0.25) + 1·(0.8 + 0.1)).
        let holding = 9.81 * (2.0 * 0.75 + 0.9);
        let d_rad = 2.0 * 0.7 * (1.5e5 * 180.0 / PI * j1.moved_inertia).sqrt();
        assert_near(
            j1.max_force,
            derived_force_cap(holding, j1.moved_inertia, d_rad, 1.0),
            1e-9,
        );
    }

    #[test]
    fn prismatic_drives_are_tuned_on_the_moved_mass() {
        let z = [0.0, 0.0, 1.0];
        let r = robot(
            vec![
                link("base", None),
                link("carriage", Some((5.0, [0.0; 3], [0.1, 0.1, 0.1]))),
            ],
            vec![joint(
                "slide",
                JointType::Prismatic,
                "base",
                "carriage",
                [0.0; 3],
                z,
                Some((-1.0, 1.0, 0.0, 0.5)),
            )],
        );
        let poses = HashMap::from([
            ("base".to_string(), math3::IDENTITY),
            ("carriage".to_string(), math3::IDENTITY),
        ]);
        let masses: HashMap<String, MassProps> =
            r.links.iter().map(|l| (l.name.clone(), props(l))).collect();
        let drive = drive_params(&r, &r.joints[0], 0.25, &poses, &masses);
        // The stiffness: the moved weight over a 1 mm sag, k = 5·9.81/0.001 = 49050 N/m (natural
        // frequency √(g/δ) ≈ 99 rad/s); the damping 2ζ√(k·m).
        let k = 5.0 * 9.81 / 1e-3;
        let d = 2.0 * 0.7 * (k * 5.0f64).sqrt();
        assert_eq!(drive.moved_inertia, 5.0);
        assert_eq!(drive.home, 0.25);
        assert_near(drive.stiffness, 49050.0, 1e-9);
        assert_near(
            (drive.stiffness / drive.moved_inertia).sqrt(),
            (9.81f64 / 1e-3).sqrt(),
            1e-9,
        );
        assert_near(drive.damping, d, 1e-9);
        // m/s, unconverted; the cap: 2·g·m + m·v/0.25 + D·v.
        assert_eq!(drive.max_joint_velocity, Some(0.5));
        assert_near(
            drive.max_force,
            2.0 * 9.81 * 5.0 + 5.0 * 0.5 / 0.25 + d * 0.5,
            1e-9,
        );
    }

    /// An axis-aligned box `size` centred at `center`, wound outward (or inward).
    fn box_hull(center: Vec3, size: Vec3, outward: bool) -> HullMesh {
        let h = math3::scale(size, 0.5);
        let points: Vec<Vec3> = (0..8)
            .map(|i| {
                let s = |bit: usize| if i & (1 << bit) == 0 { -1.0 } else { 1.0 };
                [
                    center[0] + s(0) * h[0],
                    center[1] + s(1) * h[1],
                    center[2] + s(2) * h[2],
                ]
            })
            .collect();
        // Outward-wound quads of the cube (bit 0 = x, 1 = y, 2 = z), two triangles each.
        let quads = [
            [0, 2, 3, 1], // -z
            [4, 5, 7, 6], // +z
            [0, 1, 5, 4], // -y
            [2, 6, 7, 3], // +y
            [0, 4, 6, 2], // -x
            [1, 3, 7, 5], // +x
        ];
        let mut triangle_indices = Vec::new();
        for [a, b, c, d] in quads {
            let tris = [[a, b, c], [a, c, d]];
            for t in tris {
                if outward {
                    triangle_indices.extend(t);
                } else {
                    triangle_indices.extend([t[0], t[2], t[1]]);
                }
            }
        }
        HullMesh {
            name: "box".into(),
            points,
            triangle_indices,
        }
    }

    /// Per kilogram, about its centre: the box's diag((b² + c²)/12, (a² + c²)/12, (a² + b²)/12).
    fn box_unit_inertia([a, b, c]: Vec3) -> Mat3 {
        math3::diag([
            (b * b + c * c) / 12.0,
            (a * a + c * c) / 12.0,
            (a * a + b * b) / 12.0,
        ])
    }

    #[test]
    fn a_payload_spread_over_a_box_hull_is_the_analytic_box() {
        let size = [0.4, 0.2, 0.6];
        let center = [-0.5, 0.1, 0.25];
        for outward in [true, false] {
            let spread = hull_mass_distribution(&[box_hull(center, size, outward)], &[], "t");
            assert_vec_near(spread.centroid, center, 1e-12);
            assert_mat_near(&spread.unit_inertia, &box_unit_inertia(size), 1e-12);
        }
        // Turned and moved by a transform: the turned box tensor about the moved centre.
        let pose = math3::from_translation_rpy([1.0, -2.0, 0.5], [0.3, -0.7, 1.2]);
        let mut turned = box_hull(center, size, true);
        for p in &mut turned.points {
            *p = math3::transform_point(&pose, *p);
        }
        let spread = hull_mass_distribution(&[turned], &[], "t");
        assert_vec_near(
            spread.centroid,
            math3::transform_point(&pose, center),
            1e-12,
        );
        let want = math3::conjugate(&math3::rotation_of(&pose), &box_unit_inertia(size));
        assert_mat_near(&spread.unit_inertia, &want, 1e-12);
    }

    #[test]
    fn two_hulls_share_the_mass_by_volume() {
        // A 1×1×1 box at the origin and a 2×1×1 box (twice the volume) at x = 3: one third and two
        // thirds of the mass, centre at x = 2.
        let a = box_hull([0.0; 3], [1.0, 1.0, 1.0], true);
        let b = box_hull([3.0, 0.0, 0.0], [2.0, 1.0, 1.0], true);
        // A flat hull encloses nothing and is skipped.
        let flat = box_hull([9.0, 9.0, 9.0], [1.0, 1.0, 0.0], true);
        let spread = hull_mass_distribution(&[a, b, flat], &[], "t");
        assert_vec_near(spread.centroid, [2.0, 0.0, 0.0], 1e-12);
        // Per kilogram: each box's share of its own tensor, moved to the common centre (a 2 m and
        // b 1 m away, along X).
        let share = |size: Vec3, fraction: f64, offset: Vec3| {
            parallel_axis(
                &math3::mat3_scale(&box_unit_inertia(size), fraction),
                fraction,
                offset,
            )
        };
        let want = math3::mat3_add(
            &share([1.0, 1.0, 1.0], 1.0 / 3.0, [-2.0, 0.0, 0.0]),
            &share([2.0, 1.0, 1.0], 2.0 / 3.0, [1.0, 0.0, 0.0]),
        );
        assert_mat_near(&spread.unit_inertia, &want, 1e-12);
    }

    #[test]
    fn hulls_without_volume_make_a_point_mass() {
        // A point mass is a 5 cm solid sphere, never a zero tensor (which UsdPhysics ignores):
        // 2/5·r² = 1e-3 m² per kilogram.
        let sphere = math3::diag([1e-3; 3]);
        let flat = box_hull([1.0, 2.0, 3.0], [1.0, 1.0, 0.0], true);
        let spread = hull_mass_distribution(&[flat], &[], "t");
        assert_vec_near(spread.centroid, [1.0, 2.0, 3.0], 1e-12);
        assert_mat_near(&spread.unit_inertia, &sphere, 1e-18);
        let none = hull_mass_distribution(&[], &[], "t");
        assert_eq!(none.centroid, [0.0; 3]);
        assert_mat_near(&none.unit_inertia, &sphere, 1e-18);
        // A payload with nothing to spread over: its mass as that sphere at the link origin.
        let point = payload_body(50.0, None, None, &[], &[], "t");
        assert_eq!((point.mass, point.center_of_mass), (50.0, [0.0; 3]));
        assert_mat_near(&point.inertia, &math3::mat3_scale(&sphere, 50.0), 1e-15);
    }

    #[test]
    fn payload_body_takes_explicit_values_and_spreads_the_rest() {
        let hull = [box_hull([-0.5, 0.0, 0.2], [0.4, 0.2, 0.6], true)];
        let spread = payload_body(100.0, None, None, &hull, &[], "t");
        assert_vec_near(spread.center_of_mass, [-0.5, 0.0, 0.2], 1e-12);
        assert_mat_near(
            &spread.inertia,
            &math3::mat3_scale(&box_unit_inertia([0.4, 0.2, 0.6]), 100.0),
            1e-9,
        );
        let explicit = payload_body(
            100.0,
            Some([0.0, 0.0, 0.5]),
            Some([1.0, 2.0, 3.0]),
            &hull,
            &[],
            "t",
        );
        assert_eq!(
            explicit,
            Body {
                mass: 100.0,
                center_of_mass: [0.0, 0.0, 0.5],
                inertia: math3::diag([1.0, 2.0, 3.0]),
            }
        );
        // An explicit centre with the hulls' spread about it.
        let mixed = payload_body(100.0, Some([0.0, 0.0, 0.5]), None, &hull, &[], "t");
        assert_eq!(mixed.center_of_mass, [0.0, 0.0, 0.5]);
        assert_mat_near(&mixed.inertia, &spread.inertia, 1e-12);
    }

    #[test]
    fn a_payload_merges_by_the_parallel_axis_theorem() {
        // A flange without geometry: its URDF inertial, 25 kg with diag(0.3, 0.3, 0.2).
        let flange = link("flange", Some((25.0, [0.0; 3], [0.3, 0.3, 0.2])));
        let merged = |payload: &Body| {
            link_mass_properties(
                &flange,
                &LinkMeshes::default(),
                Some(payload),
                DEFAULT_LINK_DENSITY_KG_M3,
                false,
                "t",
            )
        };
        let payload = payload_body(
            100.0,
            None,
            None,
            &[box_hull([-0.5, 0.0, 0.2], [0.4, 0.2, 0.6], true)],
            &[],
            "t",
        );
        let props = merged(&payload);
        assert_eq!(props.mass, 125.0);
        let com = math3::scale([-50.0, 0.0, 20.0], 1.0 / 125.0);
        assert_vec_near(props.center_of_mass, com, 1e-12);
        // By hand: each body's tensor moved to the common centre.
        let want = math3::mat3_add(
            &parallel_axis(&math3::diag([0.3, 0.3, 0.2]), 25.0, math3::scale(com, -1.0)),
            &parallel_axis(
                &math3::mat3_scale(&box_unit_inertia([0.4, 0.2, 0.6]), 100.0),
                100.0,
                math3::sub([-0.5, 0.0, 0.2], com),
            ),
        );
        assert_mat_near(&props.inertia_tensor(), &want, 1e-9);
        // The payload sits off the X and Z axes, so the tensor has an XZ product: the principal
        // axes turn about Y and are authored.
        assert!(want[0][2].abs() > 1.0);
        let [w, x, y, z] = props.principal_axes_wxyz();
        assert!(
            x.abs() < 1e-12 && z.abs() < 1e-12 && y.abs() > 1e-3,
            "{w} {x} {y} {z}"
        );
        // A payload centred on the link's centre of mass with a diagonal tensor stays axis-aligned:
        // the identity, (1, 0, 0, 0).
        let centred = Body {
            mass: 5.0,
            center_of_mass: [0.0; 3],
            inertia: math3::diag([0.1, 0.2, 0.3]),
        };
        let props = merged(&centred);
        assert_eq!(props.principal_axes, math3::IDENTITY3);
        assert_eq!(props.principal_axes_wxyz(), [1.0, 0.0, 0.0, 0.0]);
        assert_vec_near(props.diagonal_inertia, [0.4, 0.5, 0.5], 1e-12);
    }
}
