//! GET /assets/*path — stream raw catalogue bytes (URDF + meshes) to the
//! browser 3D viewer. Sandboxed through AppState::resolve_asset (rejects
//! absolute paths and any '..' segment). Feeds the WebGL viewer: urdf-loader
//! fetches a dressed URDF and resolves its relative meshes against this route.

use axum::body::Bytes;
use axum::extract::{Path, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};

use crate::server::error::ApiError;
use crate::server::state::AppState;

/// Catalogue bytes are immutable for the lifetime of a file version, and a
/// dress writes new files rather than mutating old ones — so the browser may
/// hold them indefinitely, keyed by the ETag below.
const ASSET_CACHE_CONTROL: &str = "public, max-age=31536000, immutable";

// FR-056
pub async fn serve_asset(
    State(state): State<AppState>,
    Path(rel): Path<String>,
    headers: HeaderMap,
) -> Result<Response, ApiError> {
    // resolve_asset already rejects absolute + ".." (axum percent-decodes
    // the wildcard first, so "%2e%2e" arrives as ".." and is rejected too).
    let abs = state
        .resolve_asset(&rel)
        .map_err(|e| ApiError::bad_request("invalid_asset_path", e.to_string()))?;

    // Validate from metadata BEFORE reading: a 304 must not pay for the bytes.
    // That is the whole point over a tunnel, where a re-opened panel would
    // otherwise re-transfer the entire catalogue.
    let etag = etag_for(&abs).await;
    if let Some(tag) = etag.as_deref() {
        if if_none_match_matches(&headers, tag) {
            return Ok((StatusCode::NOT_MODIFIED, cache_headers(tag)).into_response());
        }
    }

    let bytes = tokio::fs::read(&abs)
        .await
        .map_err(|e| ApiError::not_found("asset_not_found", format!("{}: {}", rel, e)))?;

    let ctype = content_type_for(&abs);
    let mut resp = ([(header::CONTENT_TYPE, ctype)], Bytes::from(bytes)).into_response();
    if let Some(tag) = etag.as_deref() {
        let h = resp.headers_mut();
        for (name, value) in cache_headers(tag) {
            if let Ok(v) = value.parse() {
                h.insert(name, v);
            }
        }
    }
    Ok(resp)
}

/// `Cache-Control` + `ETag`, shared by the 200 and the 304 so a revalidating
/// client always sees the same validator.
fn cache_headers(etag: &str) -> [(header::HeaderName, String); 2] {
    [
        (header::CACHE_CONTROL, ASSET_CACHE_CONTROL.to_string()),
        (header::ETAG, etag.to_string()),
    ]
}

/// Weak-ish validator from (mtime, len) — enough to notice a re-exported mesh
/// without hashing megabytes of OBJ on every request. `None` when the file has
/// no usable metadata, in which case the caller simply omits caching.
async fn etag_for(path: &std::path::Path) -> Option<String> {
    let md = tokio::fs::metadata(path).await.ok()?;
    let mtime = md
        .modified()
        .ok()?
        .duration_since(std::time::UNIX_EPOCH)
        .ok()?;
    Some(format!(
        "\"{:x}-{:x}\"",
        mtime.as_nanos(),
        md.len()
    ))
}

/// True when the request's `If-None-Match` covers `etag` (handles `*`, a
/// comma-separated list, and the `W/` prefix).
fn if_none_match_matches(headers: &HeaderMap, etag: &str) -> bool {
    let Some(raw) = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())
    else {
        return false;
    };
    raw.split(',').any(|candidate| {
        let c = candidate.trim();
        c == "*" || c.trim_start_matches("W/") == etag
    })
}

/// Explicit map for the mesh/URDF types urdf-loader fetches; octet-stream
/// fallback via mime_guess. Content-Type is advisory for three.js loaders
/// (they parse by loader, not MIME) but correct types keep DevTools sane.
fn content_type_for(path: &std::path::Path) -> String {
    match path
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_ascii_lowercase())
        .as_deref()
    {
        Some("urdf") | Some("xml") => "application/xml; charset=utf-8".into(),
        Some("obj") => "text/plain; charset=utf-8".into(),
        Some("mtl") => "text/plain; charset=utf-8".into(),
        Some("stl") => "model/stl".into(),
        Some("dae") => "model/vnd.collada+xml".into(),
        _ => mime_guess::from_path(path)
            .first_or_octet_stream()
            .as_ref()
            .to_string(),
    }
}
