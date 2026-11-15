//! Serves the SPA from disk (`--web-root`) or the include_dir! bundle.

use axum::extract::State;
use axum::http::{header, HeaderMap, StatusCode, Uri};
use axum::response::{IntoResponse, Response};

use include_dir::{include_dir, Dir};

use crate::server::state::AppState;

/// SPA bundle baked at compile time. CARGO_MANIFEST_DIR points at the
/// `urdf-robo-dresser` crate root; the `web/` subdir is the SPA source.
static EMBEDDED_WEB: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/web");

/// `no-cache` means "you may store it, but revalidate first" — not "don't
/// store it". Paired with the ETag below that turns a cold panel open from
/// ~1.5 MB of unminified ES modules into a handful of empty 304s, while still
/// picking up a rebuilt extension immediately. The mesh route uses a long
/// max-age instead (see `assets.rs`) because those bytes are immutable.
const SPA_CACHE_CONTROL: &str = "no-cache";

/// FR-052: catch-all GET for non-API paths. Resolves against `--web-root`
/// when set, otherwise from the embedded bundle. Falls back to `index.html`
/// for any unknown path so the SPA owns client-side routing.
pub async fn handler(State(state): State<AppState>, uri: Uri, headers: HeaderMap) -> Response {
    let path = uri.path().trim_start_matches('/');
    let path = if path.is_empty() { "index.html" } else { path };

    if let Some(root) = state.web_root.as_ref() {
        let target = root.join(path);
        // mtime+len: --web-root is a live directory, so edits must invalidate.
        if let Ok(md) = tokio::fs::metadata(&target).await {
            if md.is_file() {
                let etag = disk_etag(&md);
                if let Some(done) = not_modified(&headers, etag.as_deref()) {
                    return done;
                }
                if let Ok(bytes) = tokio::fs::read(&target).await {
                    let mime = mime_guess::from_path(&target).first_or_octet_stream();
                    return cached(mime.as_ref(), bytes, etag.as_deref());
                }
            }
        }
        if let Ok(bytes) = tokio::fs::read(root.join("index.html")).await {
            return cached("text/html; charset=utf-8", bytes, None);
        }
        return (StatusCode::NOT_FOUND, "not found").into_response();
    }

    if let Some(file) = EMBEDDED_WEB.get_file(path) {
        let etag = embedded_etag(path, file.contents().len());
        if let Some(done) = not_modified(&headers, Some(&etag)) {
            return done;
        }
        let mime = mime_guess::from_path(file.path()).first_or_octet_stream();
        return cached(mime.as_ref(), file.contents().to_vec(), Some(&etag));
    }
    if let Some(idx) = EMBEDDED_WEB.get_file("index.html") {
        return cached("text/html; charset=utf-8", idx.contents().to_vec(), None);
    }

    (StatusCode::NOT_FOUND, "not found").into_response()
}

/// A 200 carrying `Cache-Control` and, when we have one, an `ETag`.
fn cached(mime: &str, bytes: Vec<u8>, etag: Option<&str>) -> Response {
    let mut resp = ([(header::CONTENT_TYPE, mime.to_string())], bytes).into_response();
    let h = resp.headers_mut();
    if let Ok(v) = SPA_CACHE_CONTROL.parse() {
        h.insert(header::CACHE_CONTROL, v);
    }
    if let Some(tag) = etag {
        if let Ok(v) = tag.parse() {
            h.insert(header::ETAG, v);
        }
    }
    resp
}

/// `Some(304)` when the client already holds this exact representation.
fn not_modified(headers: &HeaderMap, etag: Option<&str>) -> Option<Response> {
    let tag = etag?;
    let raw = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())?;
    let hit = raw.split(',').any(|candidate| {
        let c = candidate.trim();
        c == "*" || c.trim_start_matches("W/") == tag
    });
    if !hit {
        return None;
    }
    let mut resp = StatusCode::NOT_MODIFIED.into_response();
    let h = resp.headers_mut();
    if let Ok(v) = SPA_CACHE_CONTROL.parse() {
        h.insert(header::CACHE_CONTROL, v);
    }
    if let Ok(v) = tag.parse() {
        h.insert(header::ETAG, v);
    }
    Some(resp)
}

fn disk_etag(md: &std::fs::Metadata) -> Option<String> {
    let mtime = md
        .modified()
        .ok()?
        .duration_since(std::time::UNIX_EPOCH)
        .ok()?;
    Some(format!("\"{:x}-{:x}\"", mtime.as_nanos(), md.len()))
}

/// The embedded bundle is frozen into the binary, so crate version + path +
/// length identifies it: stable across requests, different after a rebuild
/// that changes the file.
fn embedded_etag(path: &str, len: usize) -> String {
    format!(
        "\"{}-{:x}-{:x}\"",
        env!("CARGO_PKG_VERSION"),
        path.len(),
        len
    )
}
