use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Json, Response};

use crate::catalogue;
use crate::server::dto::{HealthDto, ReadinessDto};
use crate::server::state::AppState;

/// Liveness probe: always 200 with the crate version.
///
/// `status` is always `ok`; `version` is `CARGO_PKG_VERSION` at compile time.
#[utoipa::path(
    get,
    path = "/healthz",
    operation_id = "healthz",
    tag = "system",
    responses(
        (status = 200, description = "Service is alive", body = HealthDto),
    ),
)]
pub async fn healthz() -> Json<HealthDto> {
    Json(HealthDto {
        status: "ok".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
    })
}

/// Readiness probe: 200 once the catalogue holds a robot or a tool, else 503.
///
/// Scans the catalogue on every call and reports `robot_count`, `tool_count` and the
/// `catalogue_root` the server uses. With neither a robot nor a tool, the 503 carries
/// `ready: false` and the reason in `errors`.
#[utoipa::path(
    get,
    path = "/readyz",
    operation_id = "readyz",
    tag = "system",
    responses(
        (status = 200, description = "Service is ready (catalogue scanned)", body = ReadinessDto),
        (status = 503, description = "Service not ready", body = ReadinessDto),
    ),
)]
pub async fn readyz(State(state): State<AppState>) -> Response {
    let catalogue_root = state.catalogue_root.as_path().to_path_buf();
    let canon_str = catalogue_root.display().to_string();

    match catalogue::scan(&catalogue_root) {
        Ok(cat) => {
            let robot_count = cat.robots.len();
            let tool_count = cat.tools.len();
            let ready = robot_count > 0 || tool_count > 0;
            let body = ReadinessDto {
                ready,
                catalogue_root: canon_str,
                robot_count,
                tool_count,
                errors: if ready {
                    vec![]
                } else {
                    vec!["catalogue is empty (no robots or tools)".to_string()]
                },
            };
            if ready {
                Json(body).into_response()
            } else {
                (StatusCode::SERVICE_UNAVAILABLE, Json(body)).into_response()
            }
        }
        Err(e) => (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(ReadinessDto {
                ready: false,
                catalogue_root: canon_str,
                robot_count: 0,
                tool_count: 0,
                errors: vec![format!("catalogue scan: {}", e)],
            }),
        )
            .into_response(),
    }
}
