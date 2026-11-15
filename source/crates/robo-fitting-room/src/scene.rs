use std::path::{Path, PathBuf};

use serde::Deserialize;
use urdf_robo_dresser::{parse_urdf, Geometry, JointType, UrdfRobot};

use crate::drives::{
    actuated_home_values, actuated_joints, HOME_LIMIT_TOLERANCE_DEG, HOME_LIMIT_TOLERANCE_M,
};
use crate::error::FittingRoomError;
use crate::floor_plan::{self, FloorPlan};
use crate::tool::{CarrierSource, ToolLayout};

#[derive(Debug, Clone)]
pub struct SceneDefinition {
    pub name: String,
    pub description: String,
    pub robots: Vec<RobotPlacement>,
    pub fixtures: Vec<FixturePlacement>,
    pub floor_plan: Option<FloorPlan>,
}

#[derive(Debug, Clone)]
pub struct RobotPlacement {
    pub robot_id: String,
    pub dressed_urdf: PathBuf,
    pub position: [f64; 3],
    pub orientation_deg: [f64; 3],
    /// FR-118: optional per-robot home pose, one entry per actuated joint
    /// (in URDF declaration order, fixed joints excluded): degrees for a
    /// revolute or continuous joint, metres for a prismatic one. `None` means
    /// "fall back to the KUKA stowed default in compose".
    pub home_pose_deg: Option<Vec<f64>>,
    /// FR-119: optional elevated mounting pedestal. `None` = flush floor mount.
    pub pedestal: Option<Pedestal>,
    /// FR-159: optional tool payload, merged into the link carrying the tool.
    /// `None` = the links' own mass properties only.
    pub payload: Option<Payload>,
    /// FR-113: optional effective density (kg/m³) the robot's links' masses
    /// are derived from where their URDF inertials are placeholders (and the
    /// tool hulls' without a payload); finite and > 0. `None` = the default,
    /// [`crate::drives::DEFAULT_LINK_DENSITY_KG_M3`].
    pub link_density_kg_m3: Option<f64>,
}

/// FR-119: elevated mounting fundament beneath a robot. Height raises the robot
/// off the floor; a static collision cylinder of the given diameter fills the gap.
#[derive(Debug, Clone, Copy)]
pub struct Pedestal {
    pub height_mm: f64,
    pub diameter_mm: f64,
}

/// FR-159: the tool a robot carries, as a rigid payload: merged into its tool
/// link — `link` when given, else the link carrying the dressed tool's
/// collision hulls (`meshes/tool/`), see [`crate::tool::ToolLayout`]. Without
/// an explicit centre of mass or inertia, the mass is spread over the tool's
/// collision geometry at uniform density.
#[derive(Debug, Clone, PartialEq)]
pub struct Payload {
    /// Mass (kg); finite and > 0.
    pub mass_kg: f64,
    /// Centre of mass (m) in the tool link's frame.
    pub center_of_mass_m: Option<[f64; 3]>,
    /// Principal moments (ixx, iyy, izz; kg·m²) about the payload's centre of
    /// mass, in the tool link's axes.
    pub inertia_kgm2: Option<[f64; 3]>,
    /// The URDF link the payload merges into (it and the links fixed below it
    /// are the tool), for a tool outside `meshes/tool/`; must be a link of the
    /// robot's URDF. `None`: found from the `meshes/tool/` collision meshes.
    pub link: Option<String>,
}

#[derive(Debug, Clone)]
pub struct FixturePlacement {
    pub id: String,
    pub urdf: PathBuf,
    pub position: [f64; 3],
    pub orientation_deg: [f64; 3],
}

// --- serde intermediate types (the manifest's YAML layout) ---

#[derive(Debug, Deserialize)]
struct SceneYaml {
    name: String,
    #[serde(default)]
    description: Option<String>,
    /// World-space origin of the cell. All robot/fixture/floor-plan positions
    /// have this subtracted at parse time so the composed scene is centred
    /// around (0, 0, 0). Defaults to (0, 0, 0) — no shift.
    #[serde(default)]
    scene_origin: Option<PositionYaml>,
    #[serde(default)]
    robots: Vec<RobotYaml>,
    #[serde(default)]
    fixtures: Vec<FixtureYaml>,
    #[serde(default)]
    floor_plan: Option<FloorPlanYaml>,
}

#[derive(Debug, Deserialize)]
struct RobotYaml {
    robot_id: String,
    dressed_urdf: String,
    position: PositionYaml,
    orientation: OrientationYaml,
    /// FR-118 per-robot home pose. Optional list of N values, one per
    /// actuated joint of the dressed URDF: degrees for a revolute or
    /// continuous joint, metres for a prismatic one. Length is validated
    /// against the URDF at validate-time. Omitting the field falls back
    /// to the KUKA stowed default in compose.
    #[serde(default)]
    home_pose_deg: Option<Vec<f64>>,
    /// FR-119 optional mounting pedestal. Omit for a flush floor mount.
    #[serde(default)]
    pedestal: Option<PedestalYaml>,
    /// FR-159 optional tool payload. Omit for none.
    #[serde(default)]
    payload: Option<PayloadYaml>,
    /// FR-113 optional effective link density (kg/m³). Omit for the default.
    #[serde(default)]
    link_density_kg_m3: Option<f64>,
}

/// `payload: { mass_kg, center_of_mass_m?: { x, y, z }, inertia_kgm2?: { ixx, iyy, izz },
/// link? }`.
#[derive(Debug, Deserialize)]
struct PayloadYaml {
    mass_kg: f64,
    #[serde(default)]
    center_of_mass_m: Option<PositionYaml>,
    #[serde(default)]
    inertia_kgm2: Option<InertiaYaml>,
    #[serde(default)]
    link: Option<String>,
}

#[derive(Debug, Deserialize)]
struct InertiaYaml {
    ixx: f64,
    iyy: f64,
    izz: f64,
}

#[derive(Debug, Deserialize)]
struct FixtureYaml {
    id: String,
    urdf: String,
    position: PositionYaml,
    orientation: OrientationYaml,
}

#[derive(Debug, Deserialize)]
struct PositionYaml {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug, Deserialize)]
struct OrientationYaml {
    a: f64,
    b: f64,
    c: f64,
}

#[derive(Debug, Deserialize)]
struct PedestalYaml {
    #[serde(default)]
    height_mm: f64,
    #[serde(default = "default_pedestal_diameter_mm")]
    diameter_mm: f64,
}

fn default_pedestal_diameter_mm() -> f64 {
    800.0
}

#[derive(Debug, Deserialize)]
struct FloorPlanYaml {
    geometry_file: String,
    #[serde(default)]
    offset: Option<PositionYaml>,
    /// Optional companion to `offset` for the editor's "pick cell origin"
    /// flow. Coordinate of the DXF/OBJ point that should map to world
    /// (0, 0, 0). When set without `offset`, parse_scene derives
    /// `effective_offset = scene_origin? - cell_origin_dxf`. When both
    /// are set, `offset` wins (it's the canonical math input); a
    /// disagreement emits a warning at validate-time.
    #[serde(default)]
    cell_origin_dxf: Option<PositionYaml>,
    #[serde(default = "default_scale")]
    scale: f64,
    /// FR-105: the layers (`# Layer:` comments) and groups (`o`, `g`) holding
    /// the footprints, by name, case-insensitively. Layers and groups whose
    /// name contains "footprint" hold footprints anyway.
    #[serde(default)]
    footprint_layers: Option<Vec<String>>,
}

fn default_scale() -> f64 {
    1.0
}

// FR-100, FR-101, FR-102
pub fn parse_scene(scene_path: &Path) -> Result<SceneDefinition, FittingRoomError> {
    let content = std::fs::read_to_string(scene_path).map_err(|e| {
        FittingRoomError::SceneParse(format!("{}: {}", scene_path.display(), e))
    })?;
    let scene_dir = scene_path.parent().unwrap_or(Path::new("."));
    parse_scene_from_yaml(&content, scene_dir)
}

/// Parse a manifest from a YAML string. `scene_dir` is the directory used
/// to resolve relative paths (URDFs, floor plan). Useful for the HTTP
/// server's validate endpoint, which has the YAML in memory and a known
/// would-be parent directory but no on-disk file.
pub fn parse_scene_from_yaml(
    content: &str,
    scene_dir: &Path,
) -> Result<SceneDefinition, FittingRoomError> {
    let yaml: SceneYaml = serde_yaml::from_str(content)
        .map_err(|e| FittingRoomError::SceneParse(format!("yaml parse: {}", e)))?;

    let origin = yaml
        .scene_origin
        .as_ref()
        .map(|o| [o.x, o.y, o.z])
        .unwrap_or([0.0; 3]);

    let robots: Vec<RobotPlacement> = yaml
        .robots
        .into_iter()
        .map(|r| RobotPlacement {
            robot_id: r.robot_id,
            dressed_urdf: scene_dir.join(&r.dressed_urdf),
            position: [
                r.position.x - origin[0],
                r.position.y - origin[1],
                r.position.z - origin[2],
            ],
            orientation_deg: [r.orientation.a, r.orientation.b, r.orientation.c],
            home_pose_deg: r.home_pose_deg,
            pedestal: r.pedestal.map(|p| Pedestal {
                height_mm: p.height_mm,
                diameter_mm: p.diameter_mm,
            }),
            // Link-local: not shifted by the scene origin.
            payload: r.payload.map(|p| Payload {
                mass_kg: p.mass_kg,
                center_of_mass_m: p.center_of_mass_m.map(|c| [c.x, c.y, c.z]),
                inertia_kgm2: p.inertia_kgm2.map(|i| [i.ixx, i.iyy, i.izz]),
                link: p.link,
            }),
            link_density_kg_m3: r.link_density_kg_m3,
        })
        .collect();

    let fixtures: Vec<FixturePlacement> = yaml
        .fixtures
        .into_iter()
        .map(|f| FixturePlacement {
            id: f.id,
            urdf: scene_dir.join(&f.urdf),
            position: [
                f.position.x - origin[0],
                f.position.y - origin[1],
                f.position.z - origin[2],
            ],
            orientation_deg: [f.orientation.a, f.orientation.b, f.orientation.c],
        })
        .collect();

    let floor_plan = match yaml.floor_plan {
        Some(fp) => {
            let obj_path = scene_dir.join(&fp.geometry_file);
            // Floor-plan offset shifts the OBJ-local coords into world space.
            // After scene_origin subtraction, the effective offset is
            // (manifest_offset - scene_origin).
            //
            // Step 3a: when `cell_origin_dxf` is set without an explicit
            // `offset`, derive `manifest_offset = -cell_origin_dxf` so the
            // chosen DXF point lands at the world origin. When both are
            // present, `offset` wins (canonical math input); validate_scene
            // reports any disagreement as a warning.
            let manifest_offset = match (fp.offset.as_ref(), fp.cell_origin_dxf.as_ref()) {
                (Some(o), _) => [o.x, o.y, o.z],
                (None, Some(c)) => [-c.x, -c.y, -c.z],
                (None, None) => [0.0; 3],
            };
            let effective_offset = [
                manifest_offset[0] - origin[0],
                manifest_offset[1] - origin[1],
                manifest_offset[2] - origin[2],
            ];
            Some(floor_plan::load_floor_plan_with_footprint_layers(
                &obj_path,
                effective_offset,
                fp.scale,
                fp.footprint_layers.as_deref().unwrap_or_default(),
            )?)
        }
        None => None,
    };

    Ok(SceneDefinition {
        name: yaml.name,
        description: yaml.description.unwrap_or_default(),
        robots,
        fixtures,
        floor_plan,
    })
}

// FR-103
pub fn validate_scene(scene: &SceneDefinition) -> Vec<FittingRoomError> {
    let mut errors = Vec::new();

    for robot in &scene.robots {
        if !robot.dressed_urdf.exists() {
            errors.push(FittingRoomError::UrdfNotFound(
                robot.dressed_urdf.display().to_string(),
            ));
        } else if let Err(e) = validate_urdf_file(&robot.dressed_urdf) {
            errors.push(e);
        }
        if !is_finite_placement(robot.position, robot.orientation_deg) {
            errors.push(FittingRoomError::Validation(format!(
                "Robot '{}': non-finite position or orientation",
                robot.robot_id,
            )));
        }

        // FR-118: when home_pose_deg is set, its entries must be finite
        // and its length must match the URDF's actuated-joint count (fixed
        // joints excluded); and every revolute and prismatic joint's home
        // value, given or the KUKA stowed default, must lie within its URDF
        // limits. The URDF checks only run when the URDF is parseable; URDF
        // errors already get reported above.
        let urdf = if robot.dressed_urdf.exists() {
            parse_urdf(&robot.dressed_urdf).ok()
        } else {
            None
        };
        // FR-157: every actuated joint turns about (slides along) an axis.
        if let Some(urdf) = &urdf {
            for joint in actuated_joints(urdf) {
                let len = joint.axis.iter().map(|a| a * a).sum::<f64>().sqrt();
                if !(len.is_finite() && len > 1e-12) {
                    errors.push(FittingRoomError::Validation(format!(
                        "Robot '{}': joint '{}' has no axis direction (axis {:?})",
                        robot.robot_id, joint.name, joint.axis,
                    )));
                }
            }
        }
        let mut home_pose_complete = true;
        if let Some(angles) = &robot.home_pose_deg {
            if !angles.iter().all(|a| a.is_finite()) {
                home_pose_complete = false;
                errors.push(FittingRoomError::Validation(format!(
                    "Robot '{}': home_pose_deg contains non-finite values",
                    robot.robot_id,
                )));
            }
            if let Some(urdf) = &urdf {
                let actuated = actuated_joints(urdf).count();
                if angles.len() != actuated {
                    home_pose_complete = false;
                    errors.push(FittingRoomError::Validation(format!(
                        "Robot '{}': home_pose_deg has {} entries but URDF has {} actuated joints",
                        robot.robot_id,
                        angles.len(),
                        actuated,
                    )));
                }
            }
        }
        if let Some(urdf) = urdf.as_ref().filter(|_| home_pose_complete) {
            errors.extend(home_limit_errors(robot, urdf));
        }

        // FR-119: pedestal height must be non-negative, diameter positive.
        if let Some(ped) = &robot.pedestal {
            if !ped.height_mm.is_finite() || ped.height_mm < 0.0 {
                errors.push(FittingRoomError::Validation(format!(
                    "Robot '{}': pedestal height_mm must be finite and >= 0",
                    robot.robot_id,
                )));
            }
            if !ped.diameter_mm.is_finite() || ped.diameter_mm <= 0.0 {
                errors.push(FittingRoomError::Validation(format!(
                    "Robot '{}': pedestal diameter_mm must be finite and > 0",
                    robot.robot_id,
                )));
            }
        }

        // FR-159: the tool payload, and the link it names.
        if let Some(payload) = &robot.payload {
            errors.extend(payload_errors(&robot.robot_id, payload));
            if let (Some(link), Some(urdf)) = (&payload.link, &urdf) {
                if !urdf.links.iter().any(|l| &l.name == link) {
                    errors.push(FittingRoomError::Validation(format!(
                        "Robot '{}': payload link '{}' is no link of its URDF",
                        robot.robot_id, link,
                    )));
                }
            }
        }

        // FR-113: the effective link density.
        if let Some(density) = robot.link_density_kg_m3 {
            if !density.is_finite() || density <= 0.0 {
                errors.push(FittingRoomError::Validation(format!(
                    "Robot '{}': link_density_kg_m3 must be finite and > 0",
                    robot.robot_id,
                )));
            }
        }
    }

    for fixture in &scene.fixtures {
        if !fixture.urdf.exists() {
            errors.push(FittingRoomError::UrdfNotFound(
                fixture.urdf.display().to_string(),
            ));
        } else if let Err(e) = validate_urdf_file(&fixture.urdf) {
            errors.push(e);
        }
        if !is_finite_placement(fixture.position, fixture.orientation_deg) {
            errors.push(FittingRoomError::Validation(format!(
                "Fixture '{}': non-finite position or orientation",
                fixture.id,
            )));
        }
    }

    errors
}

/// FR-118: an error for every revolute or prismatic joint of `urdf` whose
/// home value — `robot`'s `home_pose_deg` entry or the KUKA stowed default
/// ([`actuated_home_values`]: degrees, metres on a prismatic joint) — lies
/// outside the joint's URDF limits by more than [`HOME_LIMIT_TOLERANCE_DEG`]
/// ([`HOME_LIMIT_TOLERANCE_M`]). The joint would start (FR-157) violating its
/// limit, which PhysX resolves with a jump before the first step.
fn home_limit_errors(robot: &RobotPlacement, urdf: &UrdfRobot) -> Vec<FittingRoomError> {
    let homes = actuated_home_values(urdf, robot.home_pose_deg.as_deref());
    actuated_joints(urdf)
        .zip(homes)
        .filter_map(|(joint, home)| {
            let limit = joint.limit.as_ref()?;
            let (lower, upper, tolerance, unit, what) = match joint.joint_type {
                JointType::Revolute => (
                    limit.lower.to_degrees(),
                    limit.upper.to_degrees(),
                    HOME_LIMIT_TOLERANCE_DEG,
                    "deg",
                    "angle",
                ),
                JointType::Prismatic => {
                    (limit.lower, limit.upper, HOME_LIMIT_TOLERANCE_M, "m", "position")
                }
                _ => return None,
            };
            let inside = home >= lower - tolerance && home <= upper + tolerance;
            (!inside).then(|| {
                FittingRoomError::Validation(format!(
                    "Robot '{}': home {} {} {} of joint '{}' is outside its URDF limits [{:.4}, {:.4}] {}",
                    robot.robot_id, what, home, unit, joint.name, lower, upper, unit,
                ))
            })
        })
        .collect()
}

/// FR-103: what composes, but probably not as the manifest means it — one
/// message each, for the CLI's `validate` and the validate endpoints to show
/// and compose to log (validation errors block compose; these don't):
/// - a robot's or fixture's URDF meshes that compose can't read as geometry:
///   a mesh that isn't OBJ (STL, DAE: BL-25) composes as an empty prim
///   without geometry, collider or mass; a URI (`package://`, `file://`) or a
///   missing file fails compose where it emits the mesh;
/// - a robot's payload (FR-159) that falls back to the deepest leaf link (no
///   `meshes/tool/` collision mesh, no `payload.link`), or that has neither
///   an explicit centre of mass and inertia nor tool collision geometry to
///   spread over (a point mass);
/// - the floor plan (FR-105): no line segment at all (a DXF file isn't read:
///   BL-22); no footprint layer although the plan has named layers (none in
///   `floor_plan.footprint_layers`, none named like a footprint); a plan that
///   isn't flat in Z, as a Y-up export isn't (it is read Z up: BL-27).
///
/// A URDF that doesn't parse is `validate_scene`'s to report.
pub fn scene_warnings(scene: &SceneDefinition) -> Vec<String> {
    let mut out = Vec::new();
    for robot in &scene.robots {
        let who = format!("Robot '{}'", robot.robot_id);
        let Ok(urdf) = parse_urdf(&robot.dressed_urdf) else {
            continue;
        };
        out.extend(mesh_warnings(&who, &robot.dressed_urdf, &urdf));
        if let Some(payload) = &robot.payload {
            out.extend(payload_warnings(&who, payload, &urdf));
        }
    }
    for fixture in &scene.fixtures {
        let who = format!("Fixture '{}'", fixture.id);
        if let Ok(urdf) = parse_urdf(&fixture.urdf) {
            out.extend(mesh_warnings(&who, &fixture.urdf, &urdf));
        }
    }
    if let Some(plan) = &scene.floor_plan {
        out.extend(floor_plan_warnings(plan));
    }
    out
}

/// [`scene_warnings`]' messages on the meshes of `urdf` (at `urdf_path`), the
/// URDF of `who`: one per kind of problem, with its count and up to three
/// mesh files.
fn mesh_warnings(who: &str, urdf_path: &Path, urdf: &UrdfRobot) -> Vec<String> {
    let dir = urdf_path.parent().unwrap_or(Path::new("."));
    let (mut uris, mut formats, mut missing): (Vec<&str>, Vec<&str>, Vec<&str>) =
        Default::default();
    let mut seen = std::collections::HashSet::new();
    let meshes = urdf.links.iter().flat_map(|l| {
        l.visuals
            .iter()
            .map(|v| &v.geometry)
            .chain(l.collisions.iter().map(|c| &c.geometry))
    });
    for geometry in meshes {
        let Geometry::Mesh { filename, .. } = geometry else {
            continue;
        };
        if !seen.insert(filename.as_str()) {
            continue;
        }
        let is_obj = Path::new(filename)
            .extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("obj"));
        if filename.contains("://") {
            uris.push(filename);
        } else if !is_obj {
            formats.push(filename);
        } else if !dir.join(filename).is_file() {
            missing.push(filename);
        }
    }
    let examples = |files: &[&str]| {
        let shown: Vec<&str> = files.iter().take(3).copied().collect();
        let more = if files.len() > 3 { ", …" } else { "" };
        format!("{}{}", shown.join(", "), more)
    };
    let mut out = Vec::new();
    if !uris.is_empty() {
        out.push(format!(
            "{who}: {} mesh(es) given as a URI, which compose doesn't resolve (meshes are paths \
             relative to the URDF), so compose fails where it emits them: {}",
            uris.len(),
            examples(&uris)
        ));
    }
    if !formats.is_empty() {
        out.push(format!(
            "{who}: {} mesh(es) aren't OBJ files; only OBJ meshes are read, so they compose as \
             empty prims without geometry, collider or mass (STL/DAE support is BL-25; \
             urdf-robo-dresser writes OBJ): {}",
            formats.len(),
            examples(&formats)
        ));
    }
    if !missing.is_empty() {
        out.push(format!(
            "{who}: {} mesh file(s) not found next to the URDF, so compose fails where it emits \
             them: {}",
            missing.len(),
            examples(&missing)
        ));
    }
    out
}

/// [`scene_warnings`]' messages on the payload of `who`, whose URDF is `urdf`.
fn payload_warnings(who: &str, payload: &Payload, urdf: &UrdfRobot) -> Vec<String> {
    let layout = ToolLayout::of(urdf, payload.link.as_deref());
    let Some(carrier) = &layout.carrier else {
        return vec![format!(
            "{who}: no link to carry the payload; it is ignored"
        )];
    };
    let mut out = Vec::new();
    if layout.source == CarrierSource::DeepestLeaf {
        out.push(format!(
            "{who}: no link has a tool collision mesh (meshes/tool/), so the payload goes on the \
             deepest leaf link '{carrier}'; name its link with payload.link"
        ));
    }
    let spread = payload.center_of_mass_m.is_none() || payload.inertia_kgm2.is_none();
    if spread && !layout.has_tool_collisions(urdf) {
        out.push(format!(
            "{who}: the payload has no explicit center_of_mass_m and inertia_kgm2 and its tool \
             has no collision geometry to spread it over, so it is a point mass at the origin of \
             '{carrier}' (with a small sphere's inertia)"
        ));
    }
    out
}

/// [`scene_warnings`]' messages on the floor plan.
fn floor_plan_warnings(plan: &FloorPlan) -> Vec<String> {
    let points: Vec<&[f64; 3]> = plan
        .segments
        .iter()
        .chain(&plan.footprint_segments)
        .flatten()
        .filter(|p| p.iter().all(|v| v.is_finite()))
        .collect();
    if points.is_empty() {
        return vec![
            "The floor plan has no line segment: an OBJ of `l` lines is read (a DXF file isn't: \
             BL-22)"
                .to_owned(),
        ];
    }
    let mut out = Vec::new();
    if plan.footprint_segments.is_empty() && !plan.layers.is_empty() {
        let shown: Vec<&str> = plan.layers.iter().take(8).map(String::as_str).collect();
        let more = if plan.layers.len() > 8 { ", …" } else { "" };
        out.push(format!(
            "The floor plan has no footprint: none of its {} layer(s) ({}{}) is listed in \
             floor_plan.footprint_layers or named like a footprint, so all its lines are drawn as \
             lines",
            plan.layers.len(),
            shown.join(", "),
            more
        ));
    }
    let extent = |k: usize| {
        let (lo, hi) = points
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p[k]), hi.max(p[k]))
            });
        hi - lo
    };
    let (across, height) = (extent(0).max(extent(1)), extent(2));
    if across > 0.0 && height > 0.1 * across {
        out.push(format!(
            "The floor plan spans {height:.3} m in Z against {across:.3} m across: it is read Z up, \
             drawn on the ground as it is (a Y-up export lies on its side; an up axis or rotation \
             is BL-27)"
        ));
    }
    out
}

/// FR-159: what's wrong with a robot's tool `payload`: its mass must be finite
/// and > 0, its centre of mass finite, its principal moments finite, > 0 and
/// those of a rigid body (none above the sum of the other two).
fn payload_errors(robot_id: &str, payload: &Payload) -> Vec<FittingRoomError> {
    let mut problems: Vec<&str> = Vec::new();
    if !payload.mass_kg.is_finite() || payload.mass_kg <= 0.0 {
        problems.push("mass_kg must be finite and > 0");
    }
    if let Some(com) = payload.center_of_mass_m {
        if !com.iter().all(|c| c.is_finite()) {
            problems.push("center_of_mass_m must be finite");
        }
    }
    if let Some(moments) = payload.inertia_kgm2 {
        let sum: f64 = moments.iter().sum();
        if !moments.iter().all(|i| i.is_finite() && *i > 0.0) {
            problems.push("inertia_kgm2 must be finite and > 0");
        } else if moments.iter().any(|i| 2.0 * i > sum * (1.0 + 1e-9)) {
            problems.push(
                "inertia_kgm2 is no rigid body's: each moment must be at most the sum of the other two",
            );
        }
    }
    problems
        .into_iter()
        .map(|p| FittingRoomError::Validation(format!("Robot '{}': payload {}", robot_id, p)))
        .collect()
}

fn validate_urdf_file(path: &Path) -> Result<(), FittingRoomError> {
    let content = std::fs::read_to_string(path).map_err(|e| FittingRoomError::UrdfInvalid {
        path: path.display().to_string(),
        detail: e.to_string(),
    })?;
    if !content.contains("<robot") {
        return Err(FittingRoomError::UrdfInvalid {
            path: path.display().to_string(),
            detail: "file does not contain a <robot> element".into(),
        });
    }
    Ok(())
}

fn is_finite_placement(position: [f64; 3], orientation: [f64; 3]) -> bool {
    position.iter().all(|v| v.is_finite()) && orientation.iter().all(|v| v.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn parse_minimal_scene() {
        let dir = tempfile::tempdir().unwrap();
        let scene_path = dir.path().join("test.yaml");
        let mut f = std::fs::File::create(&scene_path).unwrap();
        write!(
            f,
            "name: \"Test Cell\"\ndescription: \"A test scene\"\nrobots: []\nfixtures: []\n"
        )
        .unwrap();

        let scene = parse_scene(&scene_path).unwrap();
        assert_eq!(scene.name, "Test Cell");
        assert_eq!(scene.description, "A test scene");
        assert!(scene.robots.is_empty());
        assert!(scene.fixtures.is_empty());
        assert!(scene.floor_plan.is_none());
    }

    #[test]
    fn parse_scene_with_placements() {
        let dir = tempfile::tempdir().unwrap();
        let scene_path = dir.path().join("cell.yaml");
        let mut f = std::fs::File::create(&scene_path).unwrap();
        write!(
            f,
            r#"name: "CELL_A"
robots:
  - robot_id: "R7"
    dressed_urdf: "robots/arm.urdf"
    position: {{ x: 12.5, y: 3.25, z: 0.0 }}
    orientation: {{ a: 270.0, b: 0.0, c: 0.0 }}
fixtures:
  - id: "fixture_01"
    urdf: "fixtures/jig.urdf"
    position: {{ x: 13.0, y: 5.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
"#
        )
        .unwrap();

        let scene = parse_scene(&scene_path).unwrap();
        assert_eq!(scene.robots.len(), 1);
        assert_eq!(scene.robots[0].robot_id, "R7");
        assert!((scene.robots[0].position[0] - 12.5).abs() < 1e-6);
        assert!((scene.robots[0].orientation_deg[0] - 270.0).abs() < 1e-6);

        assert_eq!(scene.fixtures.len(), 1);
        assert_eq!(scene.fixtures[0].id, "fixture_01");
    }

    #[test]
    fn parse_scene_with_pedestal() {
        let dir = tempfile::tempdir().unwrap();
        let scene_path = dir.path().join("cell.yaml");
        let mut f = std::fs::File::create(&scene_path).unwrap();
        write!(
            f,
            r#"name: "Pedestal"
robots:
  - robot_id: "R1"
    dressed_urdf: "robots/arm.urdf"
    position: {{ x: 1.0, y: 2.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
    pedestal:
      height_mm: 500.0
  - robot_id: "R2"
    dressed_urdf: "robots/arm.urdf"
    position: {{ x: 3.0, y: 4.0, z: 0.0 }}
    orientation: {{ a: 0.0, b: 0.0, c: 0.0 }}
"#
        )
        .unwrap();

        let scene = parse_scene(&scene_path).unwrap();
        // R1 has a pedestal; diameter defaults to 800mm when omitted.
        let ped = scene.robots[0].pedestal.unwrap();
        assert!((ped.height_mm - 500.0).abs() < 1e-6);
        assert!((ped.diameter_mm - 800.0).abs() < 1e-6);
        // R2 has no pedestal block at all.
        assert!(scene.robots[1].pedestal.is_none());
    }

    #[test]
    fn parse_scene_with_floor_plan() {
        let dir = tempfile::tempdir().unwrap();

        // Create a minimal OBJ floor plan
        let obj_path = dir.path().join("plan.obj");
        std::fs::write(&obj_path, "v 0.0 0.0 0.0\nv 1.0 0.0 0.0\nl 1 2\n").unwrap();

        let scene_path = dir.path().join("cell.yaml");
        let mut f = std::fs::File::create(&scene_path).unwrap();
        write!(
            f,
            "name: \"Test\"\nfloor_plan:\n  geometry_file: \"plan.obj\"\n  scale: 1.0\n"
        )
        .unwrap();

        let scene = parse_scene(&scene_path).unwrap();
        let fp = scene.floor_plan.unwrap();
        assert_eq!(fp.segments.len(), 1);
        assert_eq!(fp.segments[0][0], [0.0, 0.0, 0.0]);
        assert_eq!(fp.segments[0][1], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn validate_missing_urdf() {
        let scene = SceneDefinition {
            name: "test".into(),
            description: String::new(),
            robots: vec![RobotPlacement {
                robot_id: "R1".into(),
                dressed_urdf: PathBuf::from("/nonexistent/robot.urdf"),
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };

        let errors = validate_scene(&scene);
        assert_eq!(errors.len(), 1);
        assert!(matches!(errors[0], FittingRoomError::UrdfNotFound(_)));
    }

    #[test]
    fn validate_non_finite_position() {
        let scene = SceneDefinition {
            name: "test".into(),
            description: String::new(),
            robots: vec![],
            fixtures: vec![FixturePlacement {
                id: "F1".into(),
                urdf: PathBuf::from("/nonexistent/fixture.urdf"),
                position: [f64::NAN, 0.0, 0.0],
                orientation_deg: [0.0; 3],
            }],
            floor_plan: None,
        };

        let errors = validate_scene(&scene);
        assert!(errors
            .iter()
            .any(|e| matches!(e, FittingRoomError::Validation(_))));
    }

    #[test]
    fn validate_invalid_urdf_content() {
        let dir = tempfile::tempdir().unwrap();
        let urdf_path = dir.path().join("bad.urdf");
        std::fs::write(&urdf_path, "this is not a URDF file").unwrap();

        let scene = SceneDefinition {
            name: "test".into(),
            description: String::new(),
            robots: vec![RobotPlacement {
                robot_id: "R1".into(),
                dressed_urdf: urdf_path,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };

        let errors = validate_scene(&scene);
        assert_eq!(errors.len(), 1);
        assert!(matches!(errors[0], FittingRoomError::UrdfInvalid { .. }));
    }

    #[test]
    fn parse_scene_with_payload() {
        let dir = tempfile::tempdir().unwrap();
        let scene_path = dir.path().join("cell.yaml");
        std::fs::write(
            &scene_path,
            r#"name: "Payload"
scene_origin: { x: 10.0, y: 0.0, z: 0.0 }
robots:
  - robot_id: "R1"
    dressed_urdf: "r.urdf"
    position: { x: 10.0, y: 0.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }
    payload:
      mass_kg: 75  # an integer reads as a float
  - robot_id: "R2"
    dressed_urdf: "r.urdf"
    position: { x: 12.0, y: 0.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }
    payload:
      mass_kg: 87.5
      center_of_mass_m: { x: -0.4, y: 0.0, z: 0.2 }
      inertia_kgm2: { ixx: 5.0, iyy: 6.0, izz: 7.0 }
  - robot_id: "R3"
    dressed_urdf: "r.urdf"
    position: { x: 14.0, y: 0.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }
  - robot_id: "R4"
    dressed_urdf: "r.urdf"
    position: { x: 16.0, y: 0.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }
    payload: { mass_kg: 4, link: tool0 }
"#,
        )
        .unwrap();
        let scene = parse_scene(&scene_path).unwrap();
        assert_eq!(
            scene.robots[0].payload,
            Some(Payload {
                mass_kg: 75.0,
                center_of_mass_m: None,
                inertia_kgm2: None,
                link: None,
            })
        );
        // The centre of mass is link-local: the scene origin doesn't shift it.
        assert_eq!(
            scene.robots[1].payload,
            Some(Payload {
                mass_kg: 87.5,
                center_of_mass_m: Some([-0.4, 0.0, 0.2]),
                inertia_kgm2: Some([5.0, 6.0, 7.0]),
                link: None,
            })
        );
        assert_eq!(scene.robots[2].payload, None);
        // The link it merges into, by name.
        assert_eq!(
            scene.robots[3]
                .payload
                .as_ref()
                .and_then(|p| p.link.as_deref()),
            Some("tool0")
        );

        // mass_kg is required.
        let err = parse_scene_from_yaml(
            "name: x\nrobots:\n  - robot_id: R\n    dressed_urdf: r.urdf\n    position: { x: 0, y: 0, z: 0 }\n    orientation: { a: 0, b: 0, c: 0 }\n    payload: { center_of_mass_m: { x: 0, y: 0, z: 0 } }\n",
            dir.path(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("mass_kg"), "{err}");
    }

    /// A robot with one revolute joint (limits ±1 rad), one continuous joint
    /// and one more revolute joint (limits ±1 rad).
    fn write_limited_urdf(path: &Path) {
        let joint = |name: &str, kind: &str, parent: &str, child: &str| {
            format!(
                "  <joint name=\"{name}\" type=\"{kind}\">\n    <parent link=\"{parent}\"/>\n    <child link=\"{child}\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-1\" upper=\"1\" effort=\"10\" velocity=\"1\"/>\n  </joint>\n"
            )
        };
        let urdf = format!(
            "<?xml version=\"1.0\"?>\n<robot name=\"lim\">\n  <link name=\"a\"/>\n  <link name=\"b\"/>\n  <link name=\"c\"/>\n  <link name=\"d\"/>\n{}{}{}</robot>\n",
            joint("j1", "revolute", "a", "b"),
            joint("j2", "continuous", "b", "c"),
            joint("j3", "revolute", "c", "d"),
        );
        std::fs::write(path, urdf).unwrap();
    }

    fn robot_with(urdf: PathBuf, home_pose_deg: Option<Vec<f64>>, payload: Option<Payload>) -> SceneDefinition {
        SceneDefinition {
            name: "t".into(),
            description: String::new(),
            robots: vec![RobotPlacement {
                robot_id: "R1".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg,
                pedestal: None,
                payload,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        }
    }

    #[test]
    fn validate_home_angles_within_the_urdf_limits() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("lim.urdf");
        write_limited_urdf(&urdf);
        let errors = |home: Option<Vec<f64>>| {
            validate_scene(&robot_with(urdf.clone(), home, None))
                .iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
        };
        // ±1 rad = ±57.2958°; the continuous joint has no limits.
        assert!(errors(Some(vec![57.0, 400.0, -57.29])).is_empty());
        let out = errors(Some(vec![60.0, 0.0, -58.0]));
        assert_eq!(out.len(), 2, "{out:?}");
        assert!(out[0].contains("home angle 60 deg of joint 'j1'") && out[0].contains("[-57.2958, 57.2958]"), "{out:?}");
        assert!(out[1].contains("joint 'j3'"), "{out:?}");
        // No home_pose_deg: the KUKA default puts j3 (the third actuated joint) at +90°.
        let out = errors(None);
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(out[0].contains("home angle 90 deg of joint 'j3'"), "{out:?}");
        // A wrong length or a non-finite entry is reported as such, without limit errors.
        let out = errors(Some(vec![90.0]));
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(out[0].contains("1 entries but URDF has 3 actuated joints"));
        let out = errors(Some(vec![f64::NAN, 0.0, 90.0]));
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(out[0].contains("non-finite"));
    }

    #[test]
    fn validate_the_payload() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("lim.urdf");
        write_limited_urdf(&urdf);
        let errors = |payload: Payload| {
            validate_scene(&robot_with(urdf.clone(), Some(vec![0.0; 3]), Some(payload)))
                .iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
        };
        let good = Payload {
            mass_kg: 87.5,
            center_of_mass_m: Some([-0.4, 0.0, 0.2]),
            inertia_kgm2: Some([5.0, 6.0, 7.0]),
            link: None,
        };
        assert!(errors(good.clone()).is_empty());
        for mass_kg in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let out = errors(Payload {
                mass_kg,
                ..good.clone()
            });
            assert_eq!(out.len(), 1, "{mass_kg}: {out:?}");
            assert!(out[0].contains("Robot 'R1': payload mass_kg must be finite and > 0"));
        }
        let out = errors(Payload {
            center_of_mass_m: Some([0.0, f64::NAN, 0.0]),
            ..good.clone()
        });
        assert!(
            out.len() == 1 && out[0].contains("center_of_mass_m must be finite"),
            "{out:?}"
        );
        for moments in [[0.0, 1.0, 1.0], [-1.0, 1.0, 1.0], [1.0, f64::INFINITY, 1.0]] {
            let out = errors(Payload {
                inertia_kgm2: Some(moments),
                ..good.clone()
            });
            assert!(
                out.len() == 1 && out[0].contains("inertia_kgm2 must be finite and > 0"),
                "{out:?}"
            );
        }
        // 3 > 1 + 1: no rigid body has these moments; 2 = 1 + 1 (a flat plate) is fine.
        let out = errors(Payload {
            inertia_kgm2: Some([1.0, 1.0, 3.0]),
            ..good.clone()
        });
        assert!(
            out.len() == 1 && out[0].contains("at most the sum of the other two"),
            "{out:?}"
        );
        assert!(errors(Payload {
            inertia_kgm2: Some([1.0, 1.0, 2.0]),
            ..good.clone()
        })
        .is_empty());
        assert!(errors(Payload {
            mass_kg: 1.0,
            center_of_mass_m: None,
            inertia_kgm2: None,
            link: None
        })
        .is_empty());
        // A named link must be one of the URDF's.
        assert!(errors(Payload {
            link: Some("d".into()),
            ..good.clone()
        })
        .is_empty());
        let out = errors(Payload {
            link: Some("flange".into()),
            ..good.clone()
        });
        assert!(
            out.len() == 1
                && out[0].contains("Robot 'R1': payload link 'flange' is no link of its URDF"),
            "{out:?}"
        );
    }

    /// A robot `base` → `carriage` on a prismatic joint along `axis` with limits [0.5, 2.0] m,
    /// then → `arm` on a revolute joint about Z (±1 rad).
    fn write_slide_urdf(path: &Path, axis: &str) {
        std::fs::write(
            path,
            format!(
                "<robot name=\"slide\">\n  <link name=\"base\"/>\n  <link name=\"carriage\"/>\n  <link name=\"arm\"/>\n  <joint name=\"track\" type=\"prismatic\">\n    <parent link=\"base\"/>\n    <child link=\"carriage\"/>\n    <axis xyz=\"{axis}\"/>\n    <limit lower=\"0.5\" upper=\"2.0\" effort=\"10\" velocity=\"1\"/>\n  </joint>\n  <joint name=\"turn\" type=\"revolute\">\n    <parent link=\"carriage\"/>\n    <child link=\"arm\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-1\" upper=\"1\" effort=\"10\" velocity=\"1\"/>\n  </joint>\n</robot>\n"
            ),
        )
        .unwrap();
    }

    #[test]
    fn validate_prismatic_homes_in_metres_within_the_urdf_limits() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("slide.urdf");
        write_slide_urdf(&urdf, "0 -1 0");
        let errors = |home: Option<Vec<f64>>| {
            validate_scene(&robot_with(urdf.clone(), home, None))
                .iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
        };
        // The prismatic entry is metres: 1.25 m lies in [0.5, 2] m.
        assert!(errors(Some(vec![1.25, 10.0])).is_empty());
        assert!(errors(Some(vec![0.5, 0.0])).is_empty());
        let out = errors(Some(vec![2.5, 0.0]));
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(
            out[0].contains("home position 2.5 m of joint 'track' is outside its URDF limits [0.5000, 2.0000] m"),
            "{out:?}"
        );
        // Without a home_pose_deg the prismatic joint sits at 0 m, outside [0.5, 2]: reported, and
        // the KUKA default's −90° on the second joint too.
        let out = errors(None);
        assert_eq!(out.len(), 2, "{out:?}");
        assert!(
            out[0].contains("home position 0 m of joint 'track'"),
            "{out:?}"
        );
        assert!(
            out[1].contains("home angle -90 deg of joint 'turn'"),
            "{out:?}"
        );
    }

    #[test]
    fn validate_rejects_an_actuated_joint_without_axis_direction() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("slide.urdf");
        write_slide_urdf(&urdf, "0 0 0");
        let out: Vec<String> = validate_scene(&robot_with(urdf, Some(vec![1.0, 0.0]), None))
            .iter()
            .map(|e| e.to_string())
            .collect();
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(
            out[0].contains("Robot 'R1': joint 'track' has no axis direction"),
            "{out:?}"
        );
    }

    #[test]
    fn warnings_name_unreadable_meshes_payload_fallbacks_and_floor_plan_issues() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("meshes")).unwrap();
        std::fs::write(dir.path().join("meshes/ok.obj"), "v 0 0 0\n").unwrap();
        let link = |name: &str, file: &str| {
            format!("  <link name=\"{name}\"><visual><geometry><mesh filename=\"{file}\"/></geometry></visual></link>\n")
        };
        let urdf = dir.path().join("arm.urdf");
        std::fs::write(
            &urdf,
            format!(
                "<robot name=\"a\">\n{}{}{}{}{}  <joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"wrist\"/><axis xyz=\"0 0 1\"/></joint>\n</robot>\n",
                link("base", "./meshes/ok.obj"),
                link("wrist", "./meshes/wrist.STL"),
                link("cover", "package://arm/meshes/cover.dae"),
                link("cap", "./meshes/cap.dae"),
                link("lid", "./meshes/gone.obj"),
            ),
        )
        .unwrap();
        let mut scene = robot_with(
            urdf,
            None,
            Some(Payload {
                mass_kg: 5.0,
                center_of_mass_m: None,
                inertia_kgm2: None,
                link: None,
            }),
        );
        let warnings = scene_warnings(&scene);
        let find = |needle: &str| {
            warnings
                .iter()
                .find(|w| w.contains(needle))
                .unwrap_or_else(|| panic!("no '{needle}' in {warnings:#?}"))
                .clone()
        };
        // Meshes: two aren't OBJ, one is a URI, one is missing; the OBJ that exists is fine.
        let formats = find("2 mesh(es) aren't OBJ files");
        assert!(
            formats.starts_with("Robot 'R1': ") && formats.contains("BL-25"),
            "{formats}"
        );
        assert!(
            formats.contains("./meshes/wrist.STL, ./meshes/cap.dae"),
            "{formats}"
        );
        assert!(find("given as a URI").contains("package://arm/meshes/cover.dae"));
        assert!(find("not found next to the URDF").contains("./meshes/gone.obj"));
        assert!(
            !warnings.iter().any(|w| w.contains("ok.obj")),
            "{warnings:#?}"
        );
        // The payload: no tool meshes, so the deepest leaf, and nothing to spread over.
        assert!(find("so the payload goes on the deepest leaf link").contains("payload.link"));
        assert!(find("it is a point mass at the origin of").contains("center_of_mass_m"));
        assert_eq!(warnings.len(), 5, "{warnings:#?}");
        // Named, with an explicit centre and inertia: no payload warning.
        scene.robots[0].payload = Some(Payload {
            mass_kg: 5.0,
            center_of_mass_m: Some([0.0; 3]),
            inertia_kgm2: Some([0.1; 3]),
            link: Some("wrist".into()),
        });
        assert_eq!(scene_warnings(&scene).len(), 3);

        // Floor plans: no segment; no footprint among named layers; not flat.
        scene.robots.clear();
        scene.floor_plan = Some(FloorPlan::default());
        let only = |scene: &SceneDefinition| {
            let w = scene_warnings(scene);
            assert_eq!(w.len(), 1, "{w:#?}");
            w[0].clone()
        };
        assert!(only(&scene).contains("no line segment") && only(&scene).contains("BL-22"));
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[0.0; 3], [4.0, 3.0, 0.0]]],
            layers: vec!["walls".into(), "machines".into()],
            ..FloorPlan::default()
        });
        let footprint = only(&scene);
        assert!(
            footprint.contains("none of its 2 layer(s) (walls, machines)"),
            "{footprint}"
        );
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[0.0; 3], [4.0, 0.0, 0.0]], [[0.0; 3], [0.0, 0.0, 3.0]]],
            ..FloorPlan::default()
        });
        assert!(
            only(&scene).contains("spans 3.000 m in Z against 4.000 m across")
                && only(&scene).contains("BL-27")
        );
        // A flat plan with footprints: nothing to say.
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[0.0; 3], [4.0, 0.0, 0.001]]],
            footprint_segments: vec![[[1.0, 1.0, 0.0], [2.0, 1.0, 0.0]]],
            layers: vec!["walls".into(), "footprints".into()],
        });
        assert!(scene_warnings(&scene).is_empty());
    }

    #[test]
    fn parse_scene_takes_the_footprint_layers_to_the_floor_plan() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(
            dir.path().join("plan.obj"),
            "v 0 0 0\nv 1 0 0\nl 1 2\n# Layer: Zone-B\nv 2 0 0\nv 3 0 0\nl 3 4\n# Layer: footprints\nl 1 3\n",
        )
        .unwrap();
        let plan = |floor_plan: &str| {
            let yaml = format!("name: t\nfloor_plan:\n  geometry_file: plan.obj\n{floor_plan}");
            let fp = parse_scene_from_yaml(&yaml, dir.path())
                .unwrap()
                .floor_plan
                .unwrap();
            (fp.segments.len(), fp.footprint_segments.len())
        };
        // Listed (case-insensitively), and the layer named like footprints.
        assert_eq!(plan("  footprint_layers: [zone-b]\n"), (1, 2));
        // Not listed: the footprint-named layer only.
        assert_eq!(plan(""), (2, 1));
        assert_eq!(plan("  footprint_layers: []\n"), (2, 1));
        // A list, not a single name.
        let err = parse_scene_from_yaml(
            "name: t\nfloor_plan:\n  geometry_file: plan.obj\n  footprint_layers: Zone-B\n",
            dir.path(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("footprint_layers"), "{err}");
    }

    #[test]
    fn parse_and_validate_the_link_density() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("lim.urdf");
        write_limited_urdf(&urdf);
        let yaml = |density: &str| {
            format!(
                "name: t\nrobots:\n  - robot_id: R1\n    dressed_urdf: lim.urdf\n    position: {{ x: 0, y: 0, z: 0 }}\n    orientation: {{ a: 0, b: 0, c: 0 }}\n    home_pose_deg: [0, 0, 0]\n{density}"
            )
        };
        let scene =
            parse_scene_from_yaml(&yaml("    link_density_kg_m3: 2700\n"), dir.path()).unwrap();
        assert_eq!(scene.robots[0].link_density_kg_m3, Some(2700.0));
        assert!(validate_scene(&scene).is_empty());
        let scene = parse_scene_from_yaml(&yaml(""), dir.path()).unwrap();
        assert_eq!(scene.robots[0].link_density_kg_m3, None);
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut scene = scene.clone();
            scene.robots[0].link_density_kg_m3 = Some(bad);
            let errors: Vec<String> = validate_scene(&scene)
                .iter()
                .map(|e| e.to_string())
                .collect();
            assert_eq!(errors.len(), 1, "{bad}: {errors:?}");
            assert!(
                errors[0].contains("Robot 'R1': link_density_kg_m3 must be finite and > 0"),
                "{errors:?}"
            );
        }
    }
}
