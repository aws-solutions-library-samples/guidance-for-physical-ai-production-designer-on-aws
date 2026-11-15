use std::path::PathBuf;

use axum::body::Bytes;
use axum::extract::{Query, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Json, Response};

use crate::compose::{compose_stage_with_options, compose_urdf_bundle, ComposeOptions};
use crate::scene::{parse_scene, parse_scene_from_yaml, scene_warnings, validate_scene};
use crate::server::dto::{BinaryBody, ErrorEnvelope, ValidateResponse};
use crate::server::error::ApiError;
use crate::server::state::AppState;

/// Query parameters for compose: the CLI's `--full-robots` / `--full-tools`
/// flags, with the same defaults.
#[derive(Debug, serde::Deserialize, utoipa::ToSchema, utoipa::IntoParams)]
pub struct ComposeQuery {
    /// Path under asset_root pointing at an existing manifest YAML, e.g.
    /// `fitted-robots/scene.yaml`. Absolute paths and `..` segments are
    /// rejected.
    pub manifest_path: String,
    /// Keep the robots' full visual meshes. `false` (the default) drops them
    /// and shows the collision hulls instead: a leaner zip, the same physics.
    /// Fixtures always keep theirs.
    #[serde(default)]
    #[param(default = false)]
    #[schema(default = false)]
    pub full_robots: bool,
    /// Keep the tools' full visual meshes (default `true`); `false` shows only
    /// their collision hulls. Independent of `full_robots`.
    #[serde(default = "yes")]
    #[param(default = yes)]
    #[schema(default = yes)]
    pub full_tools: bool,
}

fn yes() -> bool {
    true
}

/// Validate a scene manifest given as YAML text.
///
/// The body is the raw manifest YAML (`application/x-yaml` or `text/plain`);
/// its relative paths resolve against `<asset_root>/fitted-robots/`, as for a
/// manifest saved there. Returns `{ok, errors[], warnings[]?}`. The 200
/// response is used both for valid and invalid manifests; the `errors` array
/// is the success/fail signal, `warnings` (`scene::scene_warnings`, absent
/// when empty) what composes but probably not as meant. YAML that doesn't
/// parse, or a referenced file that can't be found, populates `errors` rather
/// than a 4xx.
#[cfg_attr(
    feature = "server",
    utoipa::path(
        post,
        path = "/api/v1/validate",
        operation_id = "validate_scene",
        tag = "compose",
        request_body(
            content = String,
            content_type = "application/x-yaml",
            description = "The manifest's YAML text.",
        ),
        responses(
            (status = 200, description = "Validation result", body = ValidateResponse),
        ),
    )
)]
pub async fn validate(
    State(state): State<AppState>,
    body: String,
) -> Result<Json<ValidateResponse>, ApiError> {
    // Parse the YAML in memory. Resolve relative paths against
    // `<asset_root>/fitted-robots/` because that's the canonical
    // location for scene manifests in this project — paths like
    // `../floor-plans/foo.obj` reach back into asset_root from there.
    // Same convention compose's on-disk parse_scene uses.
    let scene_dir = state.asset_root.join("fitted-robots");
    let parse_res = parse_scene_from_yaml(&body, &scene_dir);
    let scene = match parse_res {
        Ok(s) => s,
        Err(e) => {
            return Ok(Json(ValidateResponse {
                ok: false,
                errors: vec![e.to_string()],
                warnings: Vec::new(),
            }));
        }
    };
    let errors = validate_scene(&scene)
        .into_iter()
        .map(|e| e.to_string())
        .collect::<Vec<_>>();
    Ok(Json(ValidateResponse {
        ok: errors.is_empty(),
        errors,
        warnings: scene_warnings(&scene),
    }))
}

/// Compose a manifest on disk into a USDA scene zip.
///
/// Runs the compose pipeline against the manifest at `manifest_path` under
/// `asset_root`. The zip, holding the manifest and the composed
/// `<stem>.usda` stage, is streamed back as the response body with
/// Content-Type `application/zip` and the download name `<stem>.zip`. The
/// manifest is validated first: its errors return 400 `manifest_invalid`.
#[cfg_attr(
    feature = "server",
    utoipa::path(
        post,
        path = "/api/v1/compose",
        operation_id = "compose_scene",
        tag = "compose",
        params(ComposeQuery),
        responses(
            (status = 200, description = "Composed scene zip", body = inline(BinaryBody), content_type = "application/zip"),
            (status = 400, description = "Manifest path / validation failed", body = ErrorEnvelope),
            (status = 500, description = "Compose pipeline failure", body = ErrorEnvelope),
        ),
    )
)]
pub async fn compose_zip(
    State(state): State<AppState>,
    Query(q): Query<ComposeQuery>,
) -> Result<Response, ApiError> {
    let manifest_abs = state
        .resolve_asset(&q.manifest_path)
        .map_err(|e| ApiError::bad_request("invalid_manifest_path", e.to_string()))?;
    let scene = parse_scene(&manifest_abs)
        .map_err(|e| ApiError::bad_request("manifest_parse_failed", e.to_string()))?;
    let errs = validate_scene(&scene);
    if !errs.is_empty() {
        let joined = errs
            .into_iter()
            .map(|e| e.to_string())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(ApiError::bad_request("manifest_invalid", joined)
            .with_hint("POST the manifest body to /api/v1/validate to inspect each error"));
    }

    // Write the zip to a temp file adjacent to the manifest, then read it
    // back into memory for the response. Adjacent placement keeps relative
    // asset references working at compose time.
    let stem = manifest_abs
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scene");
    let zip_path: PathBuf = manifest_abs.with_file_name(format!(".__compose_{}.zip", stem));
    let opts = ComposeOptions {
        full_robots: q.full_robots,
        full_tools: q.full_tools,
    };
    compose_stage_with_options(&scene, &manifest_abs, &zip_path, &opts)
        .map_err(|e| ApiError::internal("compose_failed", e.to_string()))?;
    let bytes = tokio::fs::read(&zip_path)
        .await
        .map_err(|e| ApiError::internal("zip_read_failed", e.to_string()))?;
    let _ = tokio::fs::remove_file(&zip_path).await;

    let download_name = format!("{}.zip", stem);
    let body = Bytes::from(bytes);
    let resp = Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, "application/zip")
        .header(
            header::CONTENT_DISPOSITION,
            format!("attachment; filename=\"{}\"", download_name),
        )
        .body(body.into_response().into_body())
        .map_err(|e| ApiError::internal("response_build_failed", e.to_string()))?;
    Ok(resp)
}

/// Query parameters for the URDF-source bundle. The compose fidelity flags
/// (`full_robots`/`full_tools`) are USDA-only knobs and don't apply here, so
/// this endpoint takes just the manifest path.
#[derive(Debug, serde::Deserialize, utoipa::ToSchema, utoipa::IntoParams)]
pub struct UrdfBundleQuery {
    /// Path under asset_root pointing at an existing manifest YAML, e.g.
    /// `fitted-robots/scene.yaml`. Absolute paths and `..` segments are
    /// rejected.
    pub manifest_path: String,
}

/// Bundle a manifest's source URDFs and meshes into one zip.
///
/// Builds the self-contained URDF-source bundle for the manifest at
/// `manifest_path` under `asset_root` and streams it back as
/// `application/zip`, with the download name `<stem>-urdf.zip`. The bundle
/// holds the scene YAML (rewritten to reference the URDFs by relative path)
/// plus every source URDF copied verbatim (each in its own folder with its
/// `meshes/` subtree) and the floor-plan OBJ when present. The manifest is
/// validated first: its errors return 400 `manifest_invalid`. This is a
/// SEPARATE artifact from `/compose`; the editor calls both when the user
/// opts in.
#[cfg_attr(
    feature = "server",
    utoipa::path(
        post,
        path = "/api/v1/compose-urdf-bundle",
        operation_id = "compose_urdf_bundle",
        tag = "compose",
        params(UrdfBundleQuery),
        responses(
            (status = 200, description = "URDF-source bundle zip", body = inline(BinaryBody), content_type = "application/zip"),
            (status = 400, description = "Manifest path / validation failed", body = ErrorEnvelope),
            (status = 500, description = "Bundle build failure", body = ErrorEnvelope),
        ),
    )
)]
pub async fn compose_urdf_bundle_zip(
    State(state): State<AppState>,
    Query(q): Query<UrdfBundleQuery>,
) -> Result<Response, ApiError> {
    let manifest_abs = state
        .resolve_asset(&q.manifest_path)
        .map_err(|e| ApiError::bad_request("invalid_manifest_path", e.to_string()))?;
    let scene = parse_scene(&manifest_abs)
        .map_err(|e| ApiError::bad_request("manifest_parse_failed", e.to_string()))?;
    let errs = validate_scene(&scene);
    if !errs.is_empty() {
        let joined = errs
            .into_iter()
            .map(|e| e.to_string())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(ApiError::bad_request("manifest_invalid", joined)
            .with_hint("POST the manifest body to /api/v1/validate to inspect each error"));
    }

    // Temp file adjacent to the manifest so relative asset references resolve.
    let stem = manifest_abs
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scene");
    let zip_path: PathBuf = manifest_abs.with_file_name(format!(".__urdf_bundle_{}.zip", stem));
    compose_urdf_bundle(&scene, &manifest_abs, &zip_path)
        .map_err(|e| ApiError::internal("bundle_failed", e.to_string()))?;
    let bytes = tokio::fs::read(&zip_path)
        .await
        .map_err(|e| ApiError::internal("zip_read_failed", e.to_string()))?;
    let _ = tokio::fs::remove_file(&zip_path).await;

    let download_name = format!("{}-urdf.zip", stem);
    let body = Bytes::from(bytes);
    let resp = Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, "application/zip")
        .header(
            header::CONTENT_DISPOSITION,
            format!("attachment; filename=\"{}\"", download_name),
        )
        .body(body.into_response().into_body())
        .map_err(|e| ApiError::internal("response_build_failed", e.to_string()))?;
    Ok(resp)
}
