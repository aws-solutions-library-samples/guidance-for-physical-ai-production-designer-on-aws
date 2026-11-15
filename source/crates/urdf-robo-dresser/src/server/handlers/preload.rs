//! Preload support for the browser viewer.
//!
//!   * GET /api/v1/preload          — freshness index over every dressed combo
//!   * GET /api/v1/preload/{combo}  — every asset of one combo in ONE response
//!
//! Why this exists: a dressed combo built from a real gripper holds ~2,700 files
//! (measured: 2,899 files / 88.7 MB for a KR240 R2900 dressed with a production gripper's CAD).
//! Fetched one at a time through `/assets/*path` that is ~2,700 round trips, and
//! a browser caps HTTP/1.1 at 6 connections per origin, so no amount of client
//! concurrency fixes it — over a remote SSH/SSM tunnel the latency dominates the
//! bytes by a wide margin. Bundling collapses it to a single request, which the
//! router's CompressionLayer then gzips (~4.2x on this content).
//!
//! The index exists so the viewer's "Preload all" button can answer "already up
//! to date" without downloading anything: one request returns each combo's ETag,
//! which the client compares against what it has cached. Deliberately an index
//! rather than a conditional request per combo — one round trip instead of N.
//!
//! Both routes derive their ETag from the SAME helper, so the index can never
//! disagree with the bundle about whether a combo has changed.

use axum::body::Bytes;
use axum::extract::{Path as AxumPath, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Json, Response};
use serde::Serialize;
use std::path::{Path, PathBuf};

use crate::server::error::ApiError;
use crate::server::state::AppState;

/// Catalogue subdirectories this preloads, each laid out as
/// `<collection>/<name>/<name>.urdf` plus the meshes that URDF references.
///
/// These three are exactly what the viewer can load: selecting a robot or a tool
/// previews its SOURCE urdf (`web/js/app.js` selectEntry -> preview, which loads
/// `urdf-robot-collection/<n>/<n>.urdf` and `urdf-tool-collection/<n>/<n>.urdf`),
/// and the Dressed list loads `urdf-dressed-robots/<combo>/<combo>.urdf`. So
/// browsing the palette pulls mesh sets just as a dress does.
///
/// Excluded on purpose: `urdf-fixture-collection/` (the dresser never previews
/// fixtures — that is the fitter's job) and `floor-plans/` (flat `.obj` files,
/// not `<name>/<name>.urdf` entries). Adding either would cost transfer for
/// geometry this viewer cannot display.
const COLLECTIONS: [&str; 3] = [
    "urdf-robot-collection",
    "urdf-tool-collection",
    "urdf-dressed-robots",
];

/// Only what the viewer will actually ask for: `web/js/viewer.js`'s loader
/// dispatch handles obj/stl/dae plus the .mtl paired with each .obj, and the
/// URDF itself. Anything else in a dressed directory is skipped rather than
/// shipped across the tunnel for nothing.
const LOADABLE_EXTS: [&str; 5] = ["urdf", "obj", "mtl", "stl", "dae"];

fn is_loadable(name: &str) -> bool {
    match name.rsplit_once('.') {
        Some((_, ext)) => {
            let ext = ext.to_ascii_lowercase();
            LOADABLE_EXTS.contains(&ext.as_str())
        }
        None => false,
    }
}

/// One catalogue entry in the freshness index.
#[derive(Debug, Serialize)]
pub struct PreloadEntry {
    /// Which collection it lives in, e.g. `urdf-tool-collection`.
    pub collection: String,
    /// Entry directory name — a robot/tool/fixture id, or a `<robot>__<tool>` combo.
    pub name: String,
    /// Same value the bundle route sends as its `ETag`. Compare against a
    /// cached copy to decide whether a re-fetch is needed.
    pub etag: String,
    pub files: usize,
    pub bytes: u64,
}

/// One file inside a bundle.
struct Member {
    /// Catalogue-relative path, so the client can rebuild the exact
    /// `/assets/<rel>` URL the three.js loaders will ask for.
    rel: String,
    abs: PathBuf,
    len: u64,
}

/// GET /api/v1/preload — one row per catalogue entry, across every collection.
/// Only stats files, never reads them, so it stays cheap enough to serve uncached.
pub async fn index(State(state): State<AppState>) -> Result<Json<Vec<PreloadEntry>>, ApiError> {
    let mut out = Vec::new();
    for collection in COLLECTIONS {
        let dir = state.catalogue_root.join(collection);
        let Ok(rd) = std::fs::read_dir(&dir) else { continue };
        for entry in rd.filter_map(|e| e.ok()) {
            if !entry.path().is_dir() {
                continue;
            }
            let name = entry.file_name().to_string_lossy().to_string();
            // Convention shared by every collection: <name>/<name>.urdf must exist.
            if !entry.path().join(format!("{}.urdf", name)).exists() {
                continue;
            }
            let (members, etag) = match collect(&state, collection, &name) {
                Ok(v) => v,
                Err(_) => continue, // unreadable entry: omit rather than fail the whole index
            };
            out.push(PreloadEntry {
                collection: collection.to_string(),
                name,
                etag,
                files: members.len(),
                bytes: members.iter().map(|m| m.len).sum(),
            });
        }
    }
    out.sort_by(|a, b| (&a.collection, &a.name).cmp(&(&b.collection, &b.name)));
    Ok(Json(out))
}

/// GET /api/v1/preload/{combo} — the whole combo as one body.
///
/// Wire format, chosen so the client needs no unzip library:
///
/// ```text
///   [u32 LE manifest_len][manifest JSON][file bytes, concatenated]
///   manifest = [{"rel": "...", "off": <u64>, "len": <u64>}, ...]
/// ```
///
/// `application/octet-stream` on purpose: tower-http's DefaultPredicate skips
/// gRPC, images and SSE, so this still gets gzipped. A zip would not — it would
/// be double-compressed for nothing.
pub async fn bundle(
    State(state): State<AppState>,
    AxumPath((collection, name)): AxumPath<(String, String)>,
    headers: HeaderMap,
) -> Result<Response, ApiError> {
    let (members, etag) = collect(&state, &collection, &name)?;

    // Revalidate before reading ~90 MB off disk — the whole point of the index.
    if if_none_match_matches(&headers, &etag) {
        return Ok((StatusCode::NOT_MODIFIED, cache_headers(&etag)).into_response());
    }

    // Build the manifest first so offsets are known before any body is copied.
    let mut manifest = String::from("[");
    let mut offset: u64 = 0;
    for (i, m) in members.iter().enumerate() {
        if i > 0 {
            manifest.push(',');
        }
        // rel is a catalogue-relative path we constructed from directory entries,
        // so it needs JSON escaping only for the characters a filename may hold.
        manifest.push_str(&format!(
            "{{\"rel\":{},\"off\":{},\"len\":{}}}",
            json_string(&m.rel),
            offset,
            m.len
        ));
        offset += m.len;
    }
    manifest.push(']');

    let manifest_bytes = manifest.into_bytes();
    let mut body = Vec::with_capacity(4 + manifest_bytes.len() + offset as usize);
    body.extend_from_slice(&(manifest_bytes.len() as u32).to_le_bytes());
    body.extend_from_slice(&manifest_bytes);
    for m in &members {
        let bytes = tokio::fs::read(&m.abs).await.map_err(|e| {
            ApiError::internal("preload_read_failed", format!("{}: {}", m.rel, e))
        })?;
        // A file changed size between stat and read: the manifest offsets would
        // be wrong, so fail loudly instead of shipping a corrupt bundle.
        if bytes.len() as u64 != m.len {
            return Err(ApiError::internal(
                "preload_changed_during_read",
                format!("{} changed size mid-bundle; retry", m.rel),
            ));
        }
        body.extend_from_slice(&bytes);
    }

    let mut resp = (
        [(header::CONTENT_TYPE, "application/octet-stream".to_string())],
        Bytes::from(body),
    )
        .into_response();
    let h = resp.headers_mut();
    for (name, value) in cache_headers(&etag) {
        if let Ok(v) = value.parse() {
            h.insert(name, v);
        }
    }
    Ok(resp)
}

/// Enumerate one entry's files and derive its ETag.
///
/// Every collection uses a self-contained layout — `<name>.urdf` plus a `meshes/`
/// subtree — so every loadable file under it is something the viewer may fetch.
/// Walking the directory is therefore both simpler and more complete than
/// re-parsing the URDF's `<mesh filename=…>` references, and it cannot drift from
/// what `/assets/*path` would serve.
fn collect(
    state: &AppState,
    collection: &str,
    name: &str,
) -> Result<(Vec<Member>, String), ApiError> {
    // Allowlist the collection rather than trusting the path: this is what stops
    // the route being a general-purpose directory bundler over the catalogue.
    if !COLLECTIONS.contains(&collection) {
        return Err(ApiError::bad_request(
            "invalid_collection",
            format!("not a preloadable collection: {}", collection),
        ));
    }
    // Reject separators before resolve_asset sees them: an entry is ONE path
    // segment, and "a/b" or "../x" must not be able to escape its collection.
    if name.is_empty() || name.contains('/') || name.contains('\\') || name.contains("..") {
        return Err(ApiError::bad_request(
            "invalid_entry",
            format!("not an entry name: {}", name),
        ));
    }
    let rel_root = format!("{}/{}", collection, name);
    let abs_root = state
        .resolve_asset(&rel_root)
        .map_err(|e| ApiError::bad_request("invalid_entry", e.to_string()))?;
    if !abs_root.is_dir() {
        return Err(ApiError::not_found(
            "entry_not_found",
            format!("no entry {}", rel_root),
        ));
    }

    let mut members = Vec::new();
    let mut newest_nanos: u128 = 0;
    walk(&abs_root, &rel_root, &mut members, &mut newest_nanos)?;
    if members.is_empty() {
        return Err(ApiError::not_found(
            "entry_empty",
            format!("{} holds no loadable files", rel_root),
        ));
    }
    // Stable order so the same tree always produces the same bundle and ETag.
    members.sort_by(|a, b| a.rel.cmp(&b.rel));

    let total: u64 = members.iter().map(|m| m.len).sum();
    let etag = format!(
        "\"{:x}-{:x}-{:x}\"",
        newest_nanos,
        total,
        members.len()
    );
    Ok((members, etag))
}

/// Depth-first walk collecting every regular file, tracking the newest mtime.
fn walk(
    dir: &Path,
    rel_dir: &str,
    out: &mut Vec<Member>,
    newest_nanos: &mut u128,
) -> Result<(), ApiError> {
    let rd = std::fs::read_dir(dir).map_err(|e| {
        ApiError::internal("preload_scan_failed", format!("{}: {}", dir.display(), e))
    })?;
    for entry in rd.filter_map(|e| e.ok()) {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        let rel = format!("{}/{}", rel_dir, name);
        let md = match entry.metadata() {
            Ok(m) => m,
            Err(_) => continue,
        };
        if md.is_dir() {
            walk(&path, &rel, out, newest_nanos)?;
        } else if md.is_file() && is_loadable(&name) {
            if let Ok(nanos) = md
                .modified()
                .and_then(|t| Ok(t.duration_since(std::time::UNIX_EPOCH).unwrap_or_default()))
            {
                *newest_nanos = (*newest_nanos).max(nanos.as_nanos());
            }
            out.push(Member {
                rel,
                abs: path,
                len: md.len(),
            });
        }
    }
    Ok(())
}

/// `Cache-Control` + `ETag`, shared by the 200 and the 304 so a revalidating
/// client always sees the same validator. Not `immutable`: unlike a single mesh
/// file, a bundle's identity changes whenever any member is re-dressed, and the
/// client is expected to revalidate through the index.
fn cache_headers(etag: &str) -> [(header::HeaderName, String); 2] {
    [
        (header::CACHE_CONTROL, "public, max-age=0, must-revalidate".to_string()),
        (header::ETAG, etag.to_string()),
    ]
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

/// Minimal JSON string literal. Filenames here come from the filesystem, so
/// quotes, backslashes and control characters are the cases that matter.
fn json_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
