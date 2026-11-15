//! What a dressed robot is, discovered from its URDF (`GET /api/v1/robot/info`): its joints, the
//! arm's geometry, its reach dome, its tool and the placement figures derived from them. Nothing
//! here is model-specific: robot placement and the chat agent reason from these numbers.
//!
//! Frames: metres and degrees; heights in the robot's root frame (its mounting face at z = 0: add
//! a pedestal's height for the scene's). The wrist centre is the origin of the 5th actuated joint
//! (A5), as the reach dome is computed; for a weld approached along `a` with a tool of length `L`
//! the wrist is at `weld − L·a`.

use crate::scene_edit::ComfortSpec;

/// `GET /api/v1/robot/info`: `dressed=<robot>__<tool>` (the catalogue's
/// `urdf-dressed-robots/<name>/<name>.urdf`), or a scene's robots (`scene_file` or `cell`, with
/// `robot_id` for one of them; without it every dressed robot the scene uses).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotInfoQuery {
    pub dressed: Option<String>,
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    pub robot_id: Option<String>,
}

/// The answer: one entry per distinct dressed robot.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotInfoList {
    pub robots: Vec<RobotInfo>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotInfo {
    /// The dressed robot's name (`<robot>__<tool>`: the URDF's file stem).
    pub dressed: String,
    /// The scene robots that use it (when described from a scene).
    pub robot_ids: Vec<String>,
    pub root_link: String,
    /// The flange: the child of the last movable joint.
    pub tip_link: String,
    pub dof: usize,
    /// The actuated joints, root to tip.
    pub joints: Vec<JointInfo>,
    /// The height of A1's joint origin (the top of the base): where the turning column starts.
    pub a1_height_m: f64,
    /// The height of A2's joint origin (the shoulder) at the zero pose.
    pub shoulder_height_m: Option<f64>,
    /// A2's horizontal offset from the A1 axis (the shoulder's reach out over the base).
    pub shoulder_offset_m: Option<f64>,
    /// The segments between successive actuated joint origins at the zero pose (the root first).
    pub links: Vec<LinkInfo>,
    /// How the wrist centre is defined (the point the dome is computed for).
    pub wrist_centre: String,
    /// The wrist centre at the zero pose, root frame.
    pub wrist_zero_pose_m: Option<[f64; 3]>,
    /// The reach dome; `None` for arms with fewer than 5 joints or an A1 that is not vertical.
    pub dome: Option<DomeInfo>,
    pub tool: ToolInfo,
    pub placement: PlacementFigures,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct JointInfo {
    pub name: String,
    /// `revolute`, `continuous` or `prismatic`.
    pub kind: String,
    /// Limits: degrees (revolute, continuous: ±360), metres (prismatic).
    pub lower: f64,
    pub upper: f64,
    /// The URDF's `velocity` limit (degrees or metres per second), if it gives one.
    pub max_velocity: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct LinkInfo {
    /// The joint (or `root`) the segment starts at and the joint it ends at.
    pub from: String,
    pub to: String,
    pub length_m: f64,
    /// Its horizontal and vertical parts (root frame, zero pose).
    pub horizontal_m: f64,
    pub vertical_m: f64,
}

/// The wrist's reach dome, revolved about A1: `(r, z)` = distance from the A1 axis, root-frame height.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct DomeInfo {
    /// The A1 axis in the root frame (`[x, y]`).
    pub a1_axis_xy: [f64; 2],
    /// The heights the wrist reaches.
    pub z_min_m: f64,
    pub z_max_m: f64,
    /// The farthest the wrist reaches from A1, at any height.
    pub r_max_m: f64,
    /// The height it reaches farthest at.
    pub sweet_height_m: f64,
    /// The inner void's radius at the sweet height (closer than this the wrist can't fold).
    pub r_min_at_sweet_m: f64,
    /// `[z_lo, z_hi]`: the heights about the sweet height where its reach is at least
    /// `band_share` of `r_max_m`: the comfortable wrist heights.
    pub comfortable_height_band_m: [f64; 2],
    pub band_share: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ToolInfo {
    /// The link the TCP is given in (the flange).
    pub tcp_link: String,
    /// The TCP in the flange frame, metres.
    pub tcp_offset_m: [f64; 3],
    /// The tool's working direction in the flange frame.
    pub tcp_axis: [f64; 3],
    /// Wrist centre to TCP at the zero pose: how far behind a weld the wrist is.
    pub tool_length_m: f64,
    /// Where the TCP comes from: `urdf:tcp`, `tool:<name>`, `table:<name>` (declared), `estimated`
    /// (from the tool's meshes) or `flange` (no tool).
    pub tcp_source: String,
    pub tcp_declared: bool,
}

/// Placement figures derived from the dome, the tool and the base geometry. `R` is the dome's
/// reach at its sweet height, `L` the tool length, ratios from [`ComfortSpec`].
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlacementFigures {
    /// The comfort bands the figures use.
    pub comfort: ComfortSpec,
    /// `[ratio_min·R, ratio_max·R]`: the wrist's comfortable distance from A1.
    pub comfortable_wrist_radius_m: Option<[f64; 2]>,
    /// `ratio_target·R`.
    pub target_wrist_radius_m: Option<f64>,
    /// `ratio_target·R + L`: the horizontal distance from a weld to the base (A1 axis) for a
    /// horizontal approach pointing away from the robot (the gun pushed outward along its reach).
    pub weld_distance_m: Option<f64>,
    /// `[ratio_min·R + L, ratio_max·R + L]`: the comfortable range of that distance.
    pub comfortable_weld_distance_m: Option<[f64; 2]>,
    /// The comfortable weld heights above the robot's mounting face for a horizontal approach
    /// (the wrist band: the tool is level) and a top-down one (the band less `L`).
    pub weld_height_band_horizontal_m: Option<[f64; 2]>,
    pub weld_height_band_top_down_m: Option<[f64; 2]>,
    /// `2·√(ratio_max² − ratio_target²)·R`, at most `2·tan(align_max)·ratio_target·R`: the length
    /// of a straight weld line, parallel to the base and approached square to it, that one robot
    /// at `weld_distance_m` does comfortably (an upper bound: heights and the tool's clearance cut it).
    pub weld_line_span_m: Option<f64>,
    /// The base footprint: the farthest the geometry of the base (the links fixed to the root) and
    /// of the link A1 turns reaches from the A1 axis, from their collision (else visual) meshes.
    pub footprint_radius_m: Option<f64>,
    /// `collision`, `visual` or `none` (no mesh loaded).
    pub footprint_source: String,
    /// Bases at least this far from a product's box: `footprint_radius_m + 0.1`.
    pub clearance_m: Option<f64>,
    /// Bases at least this far apart: `max(ratio_min·R, 2·footprint_radius_m + 0.1)`: a
    /// neighbour's base stays out of the arm's own folded (cramped) zone, and the bodies don't touch.
    pub min_spacing_m: Option<f64>,
    /// The rules above, in words.
    pub rules: Vec<String>,
}
