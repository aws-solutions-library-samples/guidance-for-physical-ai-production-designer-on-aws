use axum::extract::State;
use axum::response::Json;

use crate::parser::parse_urdf;
#[allow(unused_imports)]
use crate::server::dto::ErrorEnvelope;
use crate::server::dto::{DiagnosticDto, ValidateRequest, ValidateResponse};
use crate::server::error::ApiError;
use crate::server::state::AppState;
use crate::validator::validate as lib_validate;
use crate::Severity;

/// Validate a catalogue URDF.
///
/// Parses the URDF at the catalogue-relative `path` and runs the same checks as the
/// CLI's `urdf-robo-dresser validate <path>` (one library function backs both
/// transports). `ok` is false when any diagnostic is an error.
#[utoipa::path(
    post,
    path = "/api/v1/validate",
    operation_id = "validate_urdf",
    tag = "validate",
    request_body = ValidateRequest,
    responses(
        (status = 200, description = "Validation diagnostics", body = ValidateResponse),
        (status = 400, description = "`invalid_path` (an absolute path or one with `..`), or `urdf_parse_failed` (the file is missing or unreadable, or does not parse)", body = ErrorEnvelope),
    ),
)]
pub async fn validate(
    State(state): State<AppState>,
    Json(req): Json<ValidateRequest>,
) -> Result<Json<ValidateResponse>, ApiError> {
    let abs = state
        .resolve_asset(&req.path)
        .map_err(|e| ApiError::bad_request("invalid_path", e.to_string()))?;
    let urdf = parse_urdf(&abs)
        .map_err(|e| ApiError::bad_request("urdf_parse_failed", e.to_string()))?;
    let diags = lib_validate(&urdf, &abs, req.tool);

    let any_error = diags.iter().any(|d| d.severity == Severity::Error);
    Ok(Json(ValidateResponse {
        ok: !any_error,
        diagnostics: diags
            .into_iter()
            .map(|d| DiagnosticDto {
                severity: match d.severity {
                    Severity::Error => "error".to_string(),
                    Severity::Warning => "warning".to_string(),
                },
                code: d.code.to_string(),
                message: d.message,
                location: d
                    .location
                    .map(|l| format!("{}:{}", l.file.display(), l.element)),
            })
            .collect(),
    }))
}
