use axum::extract::State;
use axum::response::Json;

use urdf_robo_dresser::{parse_urdf, JointType};

use crate::server::dto::{ErrorEnvelope, JointInfo, PathQuery, UrdfJointsResponse};
use crate::server::error::ApiError;
use crate::server::state::AppState;

/// List a URDF's actuated joints and their limits.
///
/// Body `{path}`. Parses the URDF at `<asset_root>/<path>` and returns the
/// actuated (non-Fixed) joints in declaration order, with their limits in
/// degrees, or in metres for a prismatic joint. Used by the editor to render
/// one home-pose row per joint.
#[cfg_attr(
    feature = "server",
    utoipa::path(
        post,
        path = "/api/v1/urdf/joints",
        operation_id = "list_urdf_joints",
        tag = "urdf",
        request_body = PathQuery,
        responses(
            (status = 200, description = "Actuated-joint listing", body = UrdfJointsResponse),
            (status = 400, description = "Path resolution or URDF parse error", body = ErrorEnvelope),
        ),
    )
)]
pub async fn joints(
    State(state): State<AppState>,
    Json(q): Json<PathQuery>,
) -> Result<Json<UrdfJointsResponse>, ApiError> {
    let abs = state
        .resolve_asset(&q.path)
        .map_err(|e| ApiError::bad_request("invalid_path", e.to_string()))?;
    let urdf =
        parse_urdf(&abs).map_err(|e| ApiError::bad_request("urdf_parse_failed", e.to_string()))?;
    // Limits in degrees, or metres for a prismatic joint (`JointInfo::of`).
    let joints = urdf
        .joints
        .iter()
        .filter(|j| !matches!(j.joint_type, JointType::Fixed))
        .map(JointInfo::of)
        .collect();
    Ok(Json(UrdfJointsResponse { joints }))
}
