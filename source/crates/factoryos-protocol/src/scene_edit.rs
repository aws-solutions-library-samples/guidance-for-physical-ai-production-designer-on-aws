//! Editing a Robo Fitter scene (`scenes/<file>.yaml`) one item at a time, and placing robots
//! automatically: what the Robo Fitter does by hand, for the chat agent's MCP tools
//! (`/api/v1/scene/*`, `/api/v1/reach/place`). Every edit saves the scene as the Fitter's Save does
//! (its cells are rebuilt) and answers [`SceneEdited`].
//!
//! Scene frame: metres, z up, the floor at z = 0. Yaw (`orientation.a`) in degrees about z.

use crate::fitter::{ReachMode, ReachReport, SceneFile};

/// `POST /api/v1/scene/create`: a new, empty scene file (never overwrites).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct CreateScene {
    /// `<name>.yaml` in `scenes/`.
    pub file_name: String,
    pub description: String,
}

/// `POST /api/v1/scene/{file}/place_product`: put a product (Product & Task) into the scene, on a
/// support if it is to stand higher than the floor.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlaceProduct {
    pub product: String,
    /// The fixture id (default: the product's name, made unique).
    pub id: Option<String>,
    /// Where the product's frame origin goes, `[x, y]` metres (default `[0, 0]`).
    pub position: Option<[f64; 2]>,
    pub yaw_deg: f64,
    /// The height of the product's bottom above the floor, metres. `None` or negative: automatic,
    /// the height at which the scene's robots (or `robot`) reach its welds best.
    pub height_m: Option<f64>,
    /// For the automatic height in a scene without robots: a dressed robot (`<robot>__<tool>`) on
    /// the floor (or on `pedestal_mm`).
    pub robot: Option<String>,
    pub pedestal_mm: Option<f64>,
    /// Put a support (a box fixture) under the product when it stands above the floor (default true).
    pub support: Option<bool>,
    /// Move an existing fixture of this id instead of adding one.
    pub replace: bool,
}

/// `POST /api/v1/scene/{file}/robots/add`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AddRobot {
    /// A dressed robot of the catalogue: `<robot>__<tool>` (urdf-dressed-robots/).
    pub dressed: String,
    /// Default: the dressed robot's name, made unique.
    pub robot_id: Option<String>,
    /// `[x, y]` metres.
    pub position: [f64; 2],
    /// Default: facing the scene's products (or 0).
    pub yaw_deg: Option<f64>,
    pub pedestal_mm: Option<f64>,
    pub home_pose_deg: Option<Vec<f64>>,
}

/// `POST /api/v1/scene/{file}/robots/move`: any of position, yaw, pedestal (0: none), home pose.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MoveRobot {
    pub robot_id: String,
    pub position: Option<[f64; 2]>,
    /// Or: move by `[dx, dy]`.
    pub delta: Option<[f64; 2]>,
    pub yaw_deg: Option<f64>,
    /// Face the scene's products.
    pub face_products: bool,
    pub pedestal_mm: Option<f64>,
    pub home_pose_deg: Option<Vec<f64>>,
    /// Another dressed robot (`<robot>__<tool>`), e.g. after dresser_orient_sweep.
    pub dressed: Option<String>,
}

/// `POST /api/v1/scene/{file}/robots/remove` and `/fixtures/remove`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RemoveItems {
    pub ids: Vec<String>,
}

/// The answer of every scene edit.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SceneEdited {
    pub file_name: String,
    pub scene: SceneFile,
    /// What was done, in words.
    pub changes: Vec<String>,
    /// The cells made from the scene, rebuilt.
    pub cells: Vec<String>,
    pub notes: Vec<String>,
}

/// The bands of a comfortable weld for a robot (see the backend's `reach::comfort`): the wrist's
/// distance from A1 as a share of the dome's reach at its height, the angle between the approach and
/// the robot's line out to the wrist, and the wrist's height off the robot's best height.
///
/// The ratios and the angle are generic (shares of the robot's own reach); the height band is the
/// robot's own: `None` (or ≤ 0) derives it from the dressed robot's dome (the heights where its
/// reach is at least 90 % of its farthest, see `robot::DomeInfo`), a number is ± that many metres
/// about the height it reaches farthest at.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ComfortSpec {
    pub ratio_min: f64,
    pub ratio_target: f64,
    pub ratio_max: f64,
    pub align_max_deg: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub height_band_m: Option<f64>,
}

impl Default for ComfortSpec {
    fn default() -> Self {
        Self { ratio_min: 0.45, ratio_target: 0.62, ratio_max: 0.80, align_max_deg: 50.0, height_band_m: None }
    }
}

/// One weld for the robot that does it best.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct WeldFit {
    pub weld: String,
    pub robot: String,
    /// `comfortable`, `cramped` (too close), `stretched` (too far), `twisted` (the gun points far off the
    /// robot's line), `too_high`, `too_low`, `unreachable`.
    pub fit: String,
    /// The wrist's distance from A1 over the reach there (comfortable about 0.45-0.80).
    pub ratio: f64,
    pub align_deg: f64,
    pub height_m: f64,
    /// Move the base this far away from the weld (negative: closer) for the target ratio, metres.
    pub move_m: f64,
}

/// A robot of a layout and how it fares on the welds it does best.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotFit {
    pub robot: String,
    /// `[x, y, yaw_deg]`.
    pub base: [f64; 3],
    pub welds: usize,
    /// Its welds by fit (`comfortable`: 3, …).
    pub fits: std::collections::BTreeMap<String, usize>,
    pub ratio_min: f64,
    pub ratio_max: f64,
    /// How far its welds spread about A1 (degrees): above ~120 the robot swings a lot between them.
    pub spread_deg: f64,
    /// What to change, in words, when it is not comfortable on all its welds.
    pub advice: Option<String>,
    /// The base that advice gives (`[x, y]`), if it is a move.
    pub suggested_base: Option<[f64; 2]>,
}

/// `POST /api/v1/reach/quality`: how comfortably the scene's robots reach its welds.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct LayoutQualityRequest {
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    pub comfort: ComfortSpec,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct LayoutQuality {
    pub welds: usize,
    /// Welds by the fit of the robot that does them best.
    pub fits: std::collections::BTreeMap<String, usize>,
    pub robots: Vec<RobotFit>,
    pub per_weld: Vec<WeldFit>,
    /// Bases closer than this to each other, metres (the closest pair).
    pub closest_bases_m: Option<f64>,
    pub notes: Vec<String>,
}

/// `POST /api/v1/reach/place`: place 1..N robots around the scene's products so that they reach
/// its welds, smallest N first.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlaceRobots {
    /// A `scenes/` scene with the product(s) placed (its robots are replaced unless `keep_robots`).
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    /// The dressed robot to place (`<robot>__<tool>`); default: the scene's first robot's.
    pub dressed: Option<String>,
    pub min_robots: usize,
    pub max_robots: usize,
    /// Keep the scene's robots and add to them.
    pub keep_robots: bool,
    /// Robot bases at least this far apart, metres. 0 (default): the dressed robot's own
    /// recommendation (`robot::PlacementFigures::min_spacing_m`), 1.5 when its URDF gives nothing.
    pub min_spacing_m: f64,
    /// Bases at least this far from a product's box, metres. 0 (default): the dressed robot's own
    /// recommendation from its base footprint (`robot::PlacementFigures::clearance_m`), 0.4 when
    /// its meshes give nothing.
    pub clearance_m: f64,
    /// Pedestal of the placed robots, millimetres (default none).
    pub pedestal_mm: Option<f64>,
    /// How many robots should reach each weld (default 1; 2 for redundancy).
    pub coverage: usize,
    /// Check the chosen layout with the planner's IK (default: the reach domes only, seconds).
    pub verify: Option<ReachMode>,
    /// Write the best layout as this new `scenes/` file (never overwrites).
    pub write_as: Option<String>,
    /// What counts as a comfortable reach.
    pub comfort: ComfortSpec,
}

impl Default for PlaceRobots {
    fn default() -> Self {
        Self {
            scene_file: None,
            cell: None,
            dressed: None,
            min_robots: 1,
            max_robots: 4,
            keep_robots: false,
            min_spacing_m: 0.0,
            clearance_m: 0.0,
            pedestal_mm: None,
            coverage: 1,
            verify: None,
            write_as: None,
            comfort: ComfortSpec::default(),
        }
    }
}

/// One layout of [`PlaceRobotsResult`].
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Layout {
    pub robots: usize,
    /// `[x, y, yaw_deg]` of each placed robot.
    pub bases: Vec<[f64; 3]>,
    /// Welds reached (by `coverage` robots) out of `welds`, by the domes.
    pub covered: usize,
    pub welds: usize,
    pub uncovered: Vec<String>,
    /// Welds the robot that does them best reaches comfortably (see [`ComfortSpec`]).
    pub comfortable: usize,
    pub robots_fit: Vec<RobotFit>,
    /// Lower is better: the summed penalties of the welds' best robots.
    pub cost: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlaceRobotsResult {
    /// Fewest robots first.
    pub layouts: Vec<Layout>,
    /// The index into `layouts` of the one chosen: the fewest robots covering every weld, else
    /// the most covering.
    pub best: Option<usize>,
    /// The best layout as a scene.
    pub scene: Option<SceneFile>,
    /// Its check (domes, or the IK with `verify`).
    pub report: Option<ReachReport>,
    pub written: Option<String>,
    pub notes: Vec<String>,
}
