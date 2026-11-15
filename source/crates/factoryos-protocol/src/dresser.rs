//! Robo Dresser: the backend runs the `urdf_robo_dresser` library; the viewer shows its panels.
//! Actions are HTTP (`/api/v1/dresser/…`), state is published on `factoryos/dresser/state`,
//! the preview is a scene on `factoryos/scene/dresser`, joint sliders go to `factoryos/dresser/joint`.

/// Collision hull buffer presets, in UI order.
pub const HULLS: [&str; 3] = ["tight", "medium", "max"];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssetKind {
    Robot,
    Tool,
    Dressed,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CatalogueEntry {
    pub name: String,
    /// `None` = valid, otherwise why the entry is incomplete or broken.
    pub problem: Option<Problem>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Problem {
    /// An error, rather than just incomplete.
    pub error: bool,
    pub message: String,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Diagnostic {
    pub error: bool,
    pub text: String,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DressOutcome {
    /// Increments with every dress, so clients can tell a new outcome from an old one.
    pub seq: u64,
    /// `urdf-dressed-robots/<combo>/<combo>.urdf`, relative to the catalogue.
    pub output_rel: Option<String>,
    pub diagnostics: Vec<Diagnostic>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Joint {
    pub name: String,
    pub lower: f64,
    pub upper: f64,
    pub value: f64,
}

/// The URDF currently previewed (the scene on `factoryos/scene/dresser`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PreviewInfo {
    pub label: String,
    pub has_reach: bool,
    /// Movable joints.
    pub joints: Vec<Joint>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct DresserState {
    /// The catalogue directory (from the config's `catalogue`).
    pub root: String,
    pub robots: Vec<CatalogueEntry>,
    pub tools: Vec<CatalogueEntry>,
    pub dressed: Vec<CatalogueEntry>,
    pub catalogue_error: Option<String>,
    pub busy: Option<String>,
    pub status: String,
    pub dress: Option<DressOutcome>,
    pub preview: Option<PreviewInfo>,
    pub show_collision: bool,
    pub show_reach: bool,
}

/// `POST /api/v1/dresser/open`: use the config's catalogue, rescanning if it changed (or `rescan`).
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct OpenRequest {
    pub rescan: bool,
}

/// `POST /api/v1/dresser/preview`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PreviewRequest {
    pub kind: AssetKind,
    pub name: String,
}

/// `POST /api/v1/dresser/dress`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DressRequest {
    pub robot: String,
    pub tool: String,
    /// One of [`HULLS`].
    pub hull: String,
    /// Tool orientation in 90° steps about X/Y/Z; all zero = auto-orient.
    pub orient: [u16; 3],
    /// Then turn the tool about the flange axis by this many degrees (any angle; 0: none).
    #[serde(default, skip_serializing_if = "is_zero")]
    pub spin_deg: f64,
    /// Mount the tool on this link instead of the robot's flange.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub attach_to: Option<String>,
}

fn is_zero(v: &f64) -> bool {
    *v == 0.0
}

/// `POST /api/v1/dresser/view`.
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct ViewRequest {
    pub show_collision: bool,
    pub show_reach: bool,
}

/// Put on `factoryos/dresser/joint`: move one joint of the preview.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct JointUpdate {
    pub name: String,
    pub value: f64,
}

/// A tool's TCP, relative to its root link (the docking interface): `GET
/// /api/v1/dresser/tool_tcp?tool=<name>`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolTcp {
    pub tool: String,
    pub xyz_m: [f64; 3],
    /// Roll, pitch, yaw (URDF `rpy`), in degrees.
    pub rpy_deg: [f64; 3],
    /// Declared in the tool URDF (a `tcp` link); false: an estimate from the tool's geometry.
    pub declared: bool,
}

/// `POST /api/v1/dresser/tool_tcp` (save) and `POST /api/v1/dresser/tool_tcp/preview` (draw only):
/// a tool's TCP, relative to its root link.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SetToolTcp {
    pub tool: String,
    pub xyz_m: [f64; 3],
    /// Roll, pitch, yaw (URDF `rpy`), in degrees.
    pub rpy_deg: [f64; 3],
}

/// What saving a tool's TCP changed.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolTcpSaved {
    /// The TCP as now declared.
    pub tcp: ToolTcp,
    /// The dressed robots whose `tcp_joint` was updated: catalogue `urdf-dressed-robots/<combo>`
    /// as `<combo>`, a cell's copy as `<cell>/<combo>`.
    pub updated_robots: Vec<String>,
    /// The cells using those robots: their planner model has the old TCP, so they need Prepare
    /// Cell again (the backend starts it for the active cell, like a rebuild on save does).
    pub cells_needing_prepare: Vec<String>,
}

/// `GET /api/v1/dresser/tool_tip?tool=<name>`: the tool tip, relative to the tool's root link:
/// the farthest point of the tool's collision geometry along its tool axis ("Snap to tool tip").
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolTipEstimate {
    pub xyz_m: [f64; 3],
}

/// One tool mounting to try: `orient` (90° steps about X/Y/Z, `[0,0,0]` = the dresser's
/// auto-orientation) then `spin_deg` about the flange axis.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct OrientCandidate {
    pub orient: [u16; 3],
    pub spin_deg: f64,
}

/// `POST /api/v1/dresser/orient_sweep`: dress `robot` with `tool` once per candidate (into a
/// scratch folder, the catalogue untouched) and check each against a scene's welds. The scene is
/// `scene_file` (a `scenes/` file) or `cell` (its scene); its robots dressed with this
/// robot (or those in `robots`) carry the candidate. `apply` dresses the best one into the
/// catalogue (`urdf-dressed-robots/<robot>__<tool>/`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct OrientSweepRequest {
    pub robot: String,
    pub tool: String,
    /// One of [`HULLS`] (default `tight`).
    pub hull: String,
    /// Default: the auto-orientation spun by 0, 90, 180 and 270°. At most 24.
    pub candidates: Vec<OrientCandidate>,
    pub scene_file: Option<String>,
    pub cell: Option<String>,
    /// The scene's robots that carry the candidate (default: those dressed with `robot`, else all).
    pub robots: Vec<String>,
    /// `quick` (default) or `full` (`crate::fitter::ReachMode`).
    pub mode: crate::fitter::ReachMode,
    /// The reach domes only, no IK (seconds instead of minutes; the orientation then only changes
    /// the tool's length).
    pub estimate_only: bool,
    pub apply: bool,
}

/// One candidate's result.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct OrientScore {
    pub candidate: OrientCandidate,
    /// The orientation the dresser used (the auto-orientation resolved).
    pub orient_used: Option<[u16; 3]>,
    /// Welds reached by at least one robot / by two or more.
    pub reached: usize,
    pub redundant: usize,
    pub welds: usize,
    /// The welds no robot reaches.
    pub unreached: Vec<String>,
    /// The reasons of the failed attempts, counted.
    pub reasons: std::collections::BTreeMap<String, usize>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct OrientSweepResult {
    /// Best first (most welds reached, then most reached twice, then the given order).
    pub scores: Vec<OrientScore>,
    pub method: String,
    pub robots: Vec<String>,
    pub notes: Vec<String>,
    /// With `apply`: the dressed robot written (`urdf-dressed-robots/…`).
    pub applied: Option<String>,
}
