use axum::extract::State;
use axum::response::Json;

use crate::floor_plan::{is_dxf, load_floor_plan_with_footprint_layers};
use crate::server::dto::{ErrorEnvelope, FloorPlanQuery, FloorPlanSegments};
use crate::server::error::ApiError;
use crate::server::state::AppState;

/// Read a floor-plan OBJ's line segments.
///
/// Body `{path, footprint_layers?}`. Parses the OBJ at the given relative
/// path through `floor_plan::load_floor_plan_with_footprint_layers` — the
/// manifest's `floor_plan.footprint_layers` split off as footprints, as
/// compose does — and returns the segment arrays as JSON. The editor's canvas
/// renders these directly. A DXF file isn't read (BL-22): a 400 says so.
///
/// Offset and scale are deliberately set to identity here — the editor
/// renders in DXF/OBJ-local coordinates and applies the user's
/// `cell_origin_dxf` pick on its own. The compose pipeline applies
/// the offset later.
#[cfg_attr(
    feature = "server",
    utoipa::path(
        post,
        path = "/api/v1/floor-plan",
        operation_id = "get_floor_plan_segments",
        tag = "floor-plan",
        request_body = FloorPlanQuery,
        responses(
            (status = 200, description = "Parsed floor-plan segments", body = FloorPlanSegments),
            (status = 400, description = "Path resolution or OBJ parse error, or a DXF file", body = ErrorEnvelope),
        ),
    )
)]
pub async fn segments(
    State(state): State<AppState>,
    Json(q): Json<FloorPlanQuery>,
) -> Result<Json<FloorPlanSegments>, ApiError> {
    let abs = state
        .resolve_asset(&q.path)
        .map_err(|e| ApiError::bad_request("invalid_path", e.to_string()))?;
    if is_dxf(&abs) {
        return Err(ApiError::bad_request(
            "floor_plan_unsupported",
            format!(
                "{}: DXF floor plans aren't read (BL-22); export the plan as an OBJ of `l` lines",
                q.path
            ),
        ));
    }
    let fp = load_floor_plan_with_footprint_layers(&abs, [0.0; 3], 1.0, &q.footprint_layers)
        .map_err(|e| ApiError::bad_request("floor_plan_parse_failed", e.to_string()))?;
    Ok(Json(FloorPlanSegments {
        segments: fp.segments,
        footprint_segments: fp.footprint_segments,
    }))
}
