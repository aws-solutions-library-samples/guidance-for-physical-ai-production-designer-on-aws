//! Wire types shared between handlers and external clients. Every type
//! carries `utoipa::ToSchema` so it's reflected in the published OpenAPI
//! spec — the same schema-as-contract discipline the fitting-room uses.
//!
//! These are *the* DTOs; handlers must never return inline JSON or bare
//! strings. Schema changes are enforced via the
//! `tests/openapi_snapshot.rs` test against the checked-in `openapi.json`.

use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

// A binary response body, documented as `type: string, format: binary` (utoipa 5+ documents a
// bare `Vec<u8>` as an array of integers). A comment, not a doc comment: that would become its
// description in the spec.
#[derive(ToSchema)]
#[schema(value_type = String, format = Binary)]
pub struct BinaryBody(pub Vec<u8>);

// ---- Catalogue listings ----

#[derive(Debug, Serialize, ToSchema)]
pub struct CatalogueListing {
    pub robots: Vec<RobotEntry>,
    pub tools: Vec<ToolEntry>,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct RobotEntry {
    pub name: String,
    pub flange_link: String,
    pub joint_count: usize,
    pub link_count: usize,
    pub status: AssetStatusDto,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct ToolEntry {
    pub name: String,
    pub visual_count: usize,
    pub status: AssetStatusDto,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct DressedEntry {
    /// Combo directory name, i.e. `<robot_id>__<tool_id>`.
    pub name: String,
    /// Catalogue-relative path of the dressed URDF — feeds `/assets/<rel>`
    /// and the browser viewer directly.
    pub urdf_rel: String,
}

#[derive(Debug, Serialize, ToSchema)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum AssetStatusDto {
    Valid,
    Incomplete { issues: Vec<String> },
    Error { message: String },
}

// ---- Dress / merge ----

#[derive(Debug, Deserialize, ToSchema)]
pub struct DressRequest {
    /// Robot id: the `name` of a `list_catalogue_robots` entry, naming
    /// `<catalogue>/urdf-robot-collection/<robot_id>/<robot_id>.urdf`. One entry name of
    /// at most 255 bytes: not `.` or `..`, and no `/`, `\`, `:` or NUL.
    pub robot_id: String,
    /// Tool id: the `name` of a `list_catalogue_tools` entry, naming
    /// `<catalogue>/urdf-tool-collection/<tool_id>/<tool_id>.urdf`. One entry name of at
    /// most 255 bytes: not `.` or `..`, and no `/`, `\`, `:` or NUL.
    pub tool_id: String,
    /// Robot link to mount the tool on; it must be a link of the robot. Default (omitted
    /// or `null`): the auto-detected flange, the deepest leaf link of the kinematic chain
    /// (`flange_link` in `list_catalogue_robots`).
    #[serde(default)]
    pub attach_to: Option<String>,
    /// Tool rotation as `[x, y, z]` in degrees, each 0, 90, 180 or 270. Default
    /// `[0, 0, 0]`, which auto-orients: the dresser tries all 64 combinations and keeps
    /// the one that points the tool outward from the flange and overlaps the robot least.
    #[serde(default = "default_orient")]
    #[schema(default = default_orient)]
    pub orient: [u16; 3],
    /// Collision hull buffer: `tight` (hulls hug the meshes), `medium` (hulls 5% larger)
    /// or `max` (15% larger). Default `medium`.
    #[serde(default = "default_hull")]
    #[schema(default = default_hull)]
    pub hull: String,
}

fn default_orient() -> [u16; 3] {
    [0, 0, 0]
}
fn default_hull() -> String {
    "medium".into()
}

#[derive(Debug, Serialize, ToSchema)]
pub struct DressResponse {
    /// True iff the dressed URDF + meshes are written to disk under
    /// `<catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/`.
    pub ok: bool,
    /// Catalogue-relative path of the resulting dressed URDF.
    pub dressed_urdf_rel: String,
    /// Inline-listed validation diagnostics from `validator::validate`
    /// against both inputs. Always present so clients can surface
    /// non-blocking warnings.
    pub diagnostics: Vec<DiagnosticDto>,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct DiagnosticDto {
    pub severity: String, // "error" | "warning"
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
}

// ---- Validate ----

#[derive(Debug, Deserialize, ToSchema)]
pub struct ValidateRequest {
    /// The URDF to validate, relative to the catalogue root, e.g.
    /// `urdf-robot-collection/<id>/<id>.urdf`. Absolute paths and `..` are refused.
    pub path: String,
    /// `true` validates the URDF as a tool (end effector), `false` as a robot. Default
    /// `false`. The tool-only origin check currently emits no diagnostics, so both give
    /// the same result.
    #[serde(default)]
    #[schema(default = false)]
    pub tool: bool,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct ValidateResponse {
    pub ok: bool,
    pub diagnostics: Vec<DiagnosticDto>,
}

// ---- Batch ----

#[derive(Debug, Deserialize, ToSchema)]
pub struct BatchRequest {
    /// Collision hull buffer for every robot × tool pair: `tight`, `medium` or `max`, as
    /// in `dress_robot`. Default `medium`.
    #[serde(default = "default_hull")]
    #[schema(default = default_hull)]
    pub hull: String,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct BatchAccepted {
    /// The job's UUID: pass it as `id` to `get_job_status` and `get_job_artifact`.
    pub job_id: String,
    /// Convenience pointer for the polling endpoint
    /// (`GET /api/v1/jobs/{id}`). Clients are free to construct this
    /// themselves.
    pub status_url: String,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct JobStatus {
    pub job_id: String,
    /// `queued` | `running` | `completed` | `failed`.
    pub state: String,
    /// Best-effort progress from 0.0 to 1.0: set while `running`, and 1.0 once
    /// `completed`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub progress: Option<f32>,
    /// Number of successful permutations (only set when `completed`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub success: Option<usize>,
    /// Number of failed permutations (only set when `completed`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed: Option<usize>,
    /// Failure message if the job state is `failed`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// Artifact download URL (`GET /api/v1/jobs/{id}/artifact`) once
    /// the job is `completed`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub artifact_url: Option<String>,
}

// ---- System ----

#[derive(Debug, Serialize, ToSchema)]
pub struct HealthDto {
    pub status: String,
    pub version: String,
}

#[derive(Debug, Serialize, ToSchema)]
pub struct ReadinessDto {
    pub ready: bool,
    pub catalogue_root: String,
    pub robot_count: usize,
    pub tool_count: usize,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub errors: Vec<String>,
}

/// Uniform error body. Same shape as fitting-room's so cross-service
/// clients see a consistent contract.
#[derive(Debug, Serialize, ToSchema)]
pub struct ErrorEnvelope {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hint: Option<String>,
}
