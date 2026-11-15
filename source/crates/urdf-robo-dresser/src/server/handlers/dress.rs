//! POST /api/v1/dress — JSON request pointing into the catalogue
//! (`{robot_id, tool_id, attach_to?, orient?, hull?}`). The handler runs
//! the same library functions (`parse_urdf` → `validate` → `merge` →
//! `serialize_urdf`) that the CLI's `dress` subcommand calls; outputs
//! the dressed URDF + meshes under `<catalogue>/urdf-dressed-robots/<combo>/`.
//!
//! Multipart URDF upload (i.e. dress two URDFs the server doesn't have
//! on disk yet) is intentionally NOT supported in v1 — JSON is the
//! catalogue-driven contract. Multipart support can land as a v1.1
//! additive endpoint without breaking this one.

use std::path::{Component, Path};

use axum::extract::State;
use axum::response::Json;

use crate::collision::{generate_hulls, HullBuffer};
use crate::merger::merge as lib_merge;
use crate::parser::parse_urdf;
use crate::serializer::serialize_urdf;
#[allow(unused_imports)]
use crate::server::dto::ErrorEnvelope;
use crate::server::dto::{DiagnosticDto, DressRequest, DressResponse};
use crate::server::error::ApiError;
use crate::server::state::AppState;
use crate::validator::validate;
use crate::{MergeOptions, Origin, Severity};

/// Dress a catalogue robot with a catalogue tool.
///
/// Mounts tool `tool_id` on robot `robot_id`, at its flange or the `attach_to` link,
/// and writes the dressed URDF with its meshes and collision hulls to
/// `urdf-dressed-robots/<robot_id>__<tool_id>/`. The ids are entry names as
/// `list_catalogue_robots` and `list_catalogue_tools` return them. Both URDFs are
/// validated first: errors block the dress (422), warnings come back in `diagnostics`.
/// Dressing a pair again rewrites its URDF and hulls but keeps the mesh files already
/// in its directory.
#[utoipa::path(
    post,
    path = "/api/v1/dress",
    operation_id = "dress_robot",
    tag = "dress",
    request_body = DressRequest,
    responses(
        (status = 200, description = "Dressed URDF written under <catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/", body = DressResponse),
        (status = 400, description = "Invalid input: `invalid_id` (an id that is not one catalogue entry name), `invalid_orient`, `invalid_hull_token`, or a URDF that does not parse (`robot_urdf_parse_failed`, `tool_urdf_parse_failed`)", body = ErrorEnvelope),
        (status = 404, description = "No such catalogue entry: `robot_not_in_catalogue` or `tool_not_in_catalogue`", body = ErrorEnvelope),
        (status = 422, description = "`validation_failed`: URDF validation errors block the merge; the hint gives their count, `validate_urdf` lists them per URDF", body = ErrorEnvelope),
        (status = 500, description = "`merge_failed` (also for an `attach_to` link the robot does not have), `output_mkdir_failed` or `write_failed`", body = ErrorEnvelope),
    ),
)]
pub async fn dress(
    State(state): State<AppState>,
    Json(req): Json<DressRequest>,
) -> Result<Json<DressResponse>, ApiError> {
    // The ids become path components below, so they are checked before any path is built.
    check_id("robot_id", &req.robot_id)?;
    check_id("tool_id", &req.tool_id)?;

    let robot_path = state
        .catalogue_root
        .join("urdf-robot-collection")
        .join(&req.robot_id)
        .join(format!("{}.urdf", &req.robot_id));
    let tool_path = state
        .catalogue_root
        .join("urdf-tool-collection")
        .join(&req.tool_id)
        .join(format!("{}.urdf", &req.tool_id));

    if !robot_path.exists() {
        return Err(ApiError::not_found(
            "robot_not_in_catalogue",
            format!("robot_id `{}` resolves to {} which does not exist", req.robot_id, robot_path.display()),
        ));
    }
    if !tool_path.exists() {
        return Err(ApiError::not_found(
            "tool_not_in_catalogue",
            format!("tool_id `{}` resolves to {} which does not exist", req.tool_id, tool_path.display()),
        ));
    }

    // Validate orient — same rules as CLI `parse_orient`.
    for v in req.orient {
        if v % 90 != 0 || v >= 360 {
            return Err(ApiError::bad_request(
                "invalid_orient",
                format!("orientation segments must be one of 0/90/180/270, got {}", v),
            ));
        }
    }

    let hull_buffer = HullBuffer::from_str(&req.hull)
        .map_err(|e| ApiError::bad_request("invalid_hull_token", e.to_string()))?;

    // Parse + validate both inputs.
    let base = parse_urdf(&robot_path)
        .map_err(|e| ApiError::bad_request("robot_urdf_parse_failed", e.to_string()))?;
    let mut tool_urdf = parse_urdf(&tool_path)
        .map_err(|e| ApiError::bad_request("tool_urdf_parse_failed", e.to_string()))?;

    let mut diags = Vec::new();
    diags.extend(validate(&base, &robot_path, false));
    diags.extend(validate(&tool_urdf, &tool_path, true));

    let blocking = diags.iter().any(|d| d.severity == Severity::Error);
    let diag_dtos: Vec<DiagnosticDto> = diags
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
        .collect();

    if blocking {
        return Err(ApiError::unprocessable(
            "validation_failed",
            "URDF validation produced errors; see hint for the diagnostics list",
        )
        .with_hint(format!("{} blocking diagnostics — call /api/v1/validate to inspect", diag_dtos.len())));
    }

    // Output directory: <catalogue>/urdf-dressed-robots/<robot>__<tool>/, created only now
    // that every check above has passed, so a refused dress leaves nothing behind.
    let combo_name = format!("{}__{}", req.robot_id, req.tool_id);
    let output_dir = state
        .catalogue_root
        .join("urdf-dressed-robots")
        .join(&combo_name);
    tokio::fs::create_dir_all(&output_dir)
        .await
        .map_err(|e| ApiError::internal("output_mkdir_failed", e.to_string()))?;

    let robot_urdf_dir = robot_path.parent().unwrap().to_path_buf();
    let tool_urdf_dir = tool_path.parent().unwrap().to_path_buf();

    // Generate hulls — warnings rather than blocking errors, mirroring CLI behaviour.
    let _ = generate_hulls(&mut tool_urdf, &tool_urdf_dir, &output_dir, "tool", hull_buffer);
    let mut base_with_collision = base.clone();
    let _ = generate_hulls(
        &mut base_with_collision,
        &robot_urdf_dir,
        &output_dir,
        "robot",
        hull_buffer,
    );

    let opts = MergeOptions {
        attach_to: req.attach_to,
        orient: req.orient,
        offset: Origin::default(),
        tool_prefix: "tool__".to_string(),
        output_dir: output_dir.clone(),
        robot_urdf_dir,
        tool_urdf_dir,
    };
    let dressed = lib_merge(&base_with_collision, &tool_urdf, &opts)
        .map_err(|e| ApiError::internal("merge_failed", e.to_string()))?;

    let xml = serialize_urdf(&dressed);
    let dressed_filename = format!("{}.urdf", combo_name);
    let dressed_abs = output_dir.join(&dressed_filename);
    tokio::fs::write(&dressed_abs, &xml)
        .await
        .map_err(|e| ApiError::internal("write_failed", e.to_string()))?;

    let dressed_rel = format!("urdf-dressed-robots/{}/{}", combo_name, dressed_filename);
    Ok(Json(DressResponse {
        ok: true,
        dressed_urdf_rel: dressed_rel,
        diagnostics: diag_dtos,
    }))
}

/// A catalogue id names one entry directory: exactly one normal path component, so it can
/// neither leave its collection (an absolute path, `..`) nor reach into a subdirectory
/// (`a/b`). `\` and `:` are refused as well, since Windows reads them as a separator and a
/// drive, and so are NUL and names over 255 bytes, the usual file-name limit.
fn check_id(field: &str, id: &str) -> Result<(), ApiError> {
    let mut components = Path::new(id).components();
    let one_name = matches!(
        (components.next(), components.next()),
        (Some(Component::Normal(name)), None) if name == id
    );
    if !one_name || id.len() > 255 || id.contains(['/', '\\', ':', '\0']) {
        return Err(ApiError::bad_request(
            "invalid_id",
            format!(
                "{field} must be one catalogue entry name of at most 255 bytes, not '.' or '..' \
                 and without '/', '\\', ':' or NUL: {id:?}"
            ),
        )
        .with_hint(
            "list_catalogue_robots (GET /api/v1/catalogue/robots) and list_catalogue_tools \
             (GET /api/v1/catalogue/tools) list the valid ids",
        ));
    }
    Ok(())
}
