//! GET /api/v1/jobs/{id} — poll status of a batch job.
//! GET /api/v1/jobs/{id}/artifact — download the resulting zip once
//! the job is `completed`.

use axum::body::Bytes;
use axum::extract::{Path, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Json, Response};
use uuid::Uuid;

#[allow(unused_imports)]
use crate::server::dto::ErrorEnvelope;
use crate::server::dto::JobStatus;
use crate::server::error::ApiError;
use crate::server::state::{AppState, JobState};

/// Get a batch job's state and progress.
///
/// Poll this after `start_batch` until `state` is `completed` (then download the zip
/// with `get_job_artifact`) or `failed` (the reason is in `message`). Jobs live in this
/// server process's memory only, so a restarted server answers 404 `unknown_job`.
#[utoipa::path(
    get,
    path = "/api/v1/jobs/{id}",
    operation_id = "get_job_status",
    tag = "batch",
    params(
        ("id" = String, Path, description = "Job id: the UUID `job_id` that start_batch (POST /api/v1/batch) returned")
    ),
    responses(
        (status = 200, description = "Job status snapshot", body = JobStatus),
        (status = 400, description = "`invalid_job_id`: the id is not a UUID", body = ErrorEnvelope),
        (status = 404, description = "`unknown_job`: no such job in this server process", body = ErrorEnvelope),
    ),
)]
pub async fn status(
    State(state): State<AppState>,
    Path(id): Path<String>,
) -> Result<Json<JobStatus>, ApiError> {
    let id = Uuid::parse_str(&id).map_err(|_| {
        ApiError::bad_request("invalid_job_id", "job id must be a UUID")
    })?;
    let job = {
        let jobs = state.jobs.read().await;
        jobs.get(&id).ok_or_else(|| {
            ApiError::not_found("unknown_job", format!("no job with id {}", id))
        })?
    };
    Ok(Json(job_to_dto(id, &job)))
}

/// Download a completed batch job's zip.
///
/// The zip, named `batch-<id>.zip`, holds a `<robot>__<tool>/` directory with the
/// dressed URDF and meshes for each dressed pair. It is available once `get_job_status`
/// reports `completed`; before that, and for a failed job, this answers 404
/// `job_not_completed`.
#[utoipa::path(
    get,
    path = "/api/v1/jobs/{id}/artifact",
    operation_id = "get_job_artifact",
    tag = "batch",
    params(
        ("id" = String, Path, description = "Job id: the UUID `job_id` that start_batch (POST /api/v1/batch) returned")
    ),
    responses(
        (status = 200, description = "Batch zip", body = inline(crate::server::dto::BinaryBody), content_type = "application/zip"),
        (status = 400, description = "`invalid_job_id`: the id is not a UUID", body = ErrorEnvelope),
        (status = 404, description = "`unknown_job`, or `job_not_completed` (the job is queued, running or failed)", body = ErrorEnvelope),
        (status = 500, description = "`artifact_read_failed`: the zip disappeared between job completion and download", body = ErrorEnvelope),
    ),
)]
pub async fn artifact(
    State(state): State<AppState>,
    Path(id): Path<String>,
) -> Result<Response, ApiError> {
    let id = Uuid::parse_str(&id).map_err(|_| {
        ApiError::bad_request("invalid_job_id", "job id must be a UUID")
    })?;
    let job = {
        let jobs = state.jobs.read().await;
        jobs.get(&id).ok_or_else(|| {
            ApiError::not_found("unknown_job", format!("no job with id {}", id))
        })?
    };
    let path = match job {
        JobState::Completed { artifact_path, .. } => artifact_path,
        _ => {
            return Err(ApiError::not_found(
                "job_not_completed",
                "artifact only available for completed jobs",
            ));
        }
    };

    let bytes = tokio::fs::read(&path)
        .await
        .map_err(|e| ApiError::internal("artifact_read_failed", e.to_string()))?;
    let body = Bytes::from(bytes);
    let resp = Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, "application/zip")
        .header(
            header::CONTENT_DISPOSITION,
            format!("attachment; filename=\"batch-{}.zip\"", id),
        )
        .body(body.into_response().into_body())
        .map_err(|e| ApiError::internal("response_build_failed", e.to_string()))?;
    Ok(resp)
}

fn job_to_dto(id: Uuid, state: &JobState) -> JobStatus {
    let id_s = id.to_string();
    match state {
        JobState::Queued => JobStatus {
            job_id: id_s,
            state: "queued".into(),
            progress: None,
            success: None,
            failed: None,
            message: None,
            artifact_url: None,
        },
        JobState::Running { progress } => JobStatus {
            job_id: id_s,
            state: "running".into(),
            progress: Some(*progress),
            success: None,
            failed: None,
            message: None,
            artifact_url: None,
        },
        JobState::Completed { success, failed, .. } => JobStatus {
            job_id: id_s.clone(),
            state: "completed".into(),
            progress: Some(1.0),
            success: Some(*success),
            failed: Some(*failed),
            message: None,
            artifact_url: Some(format!("/api/v1/jobs/{}/artifact", id_s)),
        },
        JobState::Failed { message } => JobStatus {
            job_id: id_s,
            state: "failed".into(),
            progress: None,
            success: None,
            failed: None,
            message: Some(message.clone()),
            artifact_url: None,
        },
    }
}
