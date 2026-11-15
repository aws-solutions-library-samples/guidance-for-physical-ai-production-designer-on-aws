//! HTTP responses as tool results, and the artifact directory binary responses are saved in.

use std::io::Write as _;
use std::path::{Path, PathBuf};

use http::response::Parts;
use rmcp::model::{CallToolResult, ContentBlock, Resource};
use serde_json::{json, Value};

use crate::Error;

/// A facade-level failure (bad arguments, a failed handler): an error result with
/// `{code, message}`.
pub(crate) fn failure(code: &str, message: impl Into<String>) -> CallToolResult {
    CallToolResult::structured_error(json!({"code": code, "message": message.into()}))
}

/// The result of `tool`'s response. Binary bodies are saved in `dir`.
pub(crate) fn to_result(tool: &str, parts: &Parts, body: &[u8], dir: &Path) -> CallToolResult {
    let media_type = parts
        .headers
        .get(http::header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .map(|v| {
            v.split(';')
                .next()
                .unwrap_or_default()
                .trim()
                .to_ascii_lowercase()
        });
    let status = parts.status;
    if status.as_u16() >= 400 {
        let body = match decode(media_type.as_deref(), body) {
            Decoded::Empty => Value::Null,
            Decoded::Json(v) => v,
            Decoded::Text(text) => text.into(),
            Decoded::Binary => String::from_utf8_lossy(body).into_owned().into(),
        };
        return CallToolResult::structured_error(
            json!({"http_status": status.as_u16(), "body": body}),
        );
    }
    match decode(media_type.as_deref(), body) {
        Decoded::Empty => CallToolResult::success(vec![ContentBlock::text(
            format!(
                "HTTP {} {}",
                status.as_u16(),
                status.canonical_reason().unwrap_or_default()
            )
            .trim_end(),
        )]),
        Decoded::Json(v @ Value::Object(_)) => CallToolResult::structured(v),
        // structuredContent must be an object.
        Decoded::Json(v @ Value::Array(_)) => CallToolResult::structured(json!({"items": v})),
        Decoded::Json(v) => CallToolResult::structured(json!({"value": v})),
        Decoded::Text(text) => CallToolResult::success(vec![ContentBlock::text(text)]),
        Decoded::Binary => {
            let content_type = media_type.as_deref().unwrap_or("application/octet-stream");
            match save(dir, tool, parts, content_type, body) {
                Ok((path, filename)) => saved(&path, &filename, content_type, body.len()),
                Err(e) => failure(
                    "internal_error",
                    format!("saving the {content_type} response failed: {e}"),
                ),
            }
        }
    }
}

enum Decoded {
    Empty,
    Json(Value),
    Text(String),
    Binary,
}

/// JSON if it says so (or says nothing) and parses, text if it is text and UTF-8, else binary.
fn decode(media_type: Option<&str>, body: &[u8]) -> Decoded {
    if body.is_empty() {
        return Decoded::Empty;
    }
    let json = media_type.is_none_or(|mt| mt == "application/json" || mt.ends_with("+json"));
    if json {
        if let Ok(v) = serde_json::from_slice(body) {
            return Decoded::Json(v);
        }
    }
    let text = media_type.is_none_or(|mt| {
        json || mt.starts_with("text/") || mt.contains("yaml") || mt.contains("xml")
    });
    match std::str::from_utf8(body) {
        Ok(s) if text => Decoded::Text(s.to_string()),
        _ => Decoded::Binary,
    }
}

/// A saved binary response: `{path, bytes, content_type, filename}` plus a `file://` link.
fn saved(path: &Path, filename: &str, content_type: &str, bytes: usize) -> CallToolResult {
    let mut result = CallToolResult::structured(json!({
        "path": path.display().to_string(),
        "bytes": bytes,
        "content_type": content_type,
        "filename": filename,
    }));
    if let Ok(uri) = url::Url::from_file_path(path) {
        result.content.push(ContentBlock::resource_link(
            Resource::new(uri.to_string(), filename)
                .with_mime_type(content_type)
                .with_size(bytes as u64),
        ));
    }
    result
}

/// Writes `body` to a new 0600 file in `dir`, named after the response's `Content-Disposition`
/// (else `<tool>.<ext>`) plus a random part. Returns the path and the suggested filename.
fn save(
    dir: &Path,
    tool: &str,
    parts: &Parts,
    content_type: &str,
    body: &[u8],
) -> std::io::Result<(PathBuf, String)> {
    let filename = parts
        .headers
        .get(http::header::CONTENT_DISPOSITION)
        .and_then(|v| v.to_str().ok())
        .and_then(disposition_filename)
        .and_then(|name| sanitize(&name))
        .unwrap_or_else(|| format!("{tool}.{}", extension(content_type)));
    let (stem, ext) = match filename.rsplit_once('.') {
        Some((stem, ext)) if !stem.is_empty() => (stem, format!(".{ext}")),
        _ => (filename.as_str(), String::new()),
    };
    let prefix = format!("{stem}-");
    // A new name every time (O_EXCL): never written through an existing file or link.
    let mut file = tempfile::Builder::new()
        .prefix(&prefix)
        .suffix(&ext)
        .tempfile_in(dir)?;
    file.write_all(body)?;
    file.flush()?;
    let (_, path) = file.keep().map_err(|e| e.error)?;
    Ok((path, filename))
}

/// `filename*=UTF-8''…` (RFC 6266), else `filename=…`, of a `Content-Disposition` value.
fn disposition_filename(value: &str) -> Option<String> {
    let mut plain = None;
    for param in value.split(';').map(str::trim) {
        let Some((key, v)) = param.split_once('=') else {
            continue;
        };
        match key.trim().to_ascii_lowercase().as_str() {
            "filename*" => {
                if let Some((_, encoded)) = v.trim().split_once("''") {
                    if let Ok(decoded) = percent_encoding::percent_decode_str(encoded).decode_utf8()
                    {
                        return Some(decoded.into_owned());
                    }
                }
            }
            "filename" => plain = Some(v.trim().trim_matches('"').to_string()),
            _ => {}
        }
    }
    plain
}

/// The last path component, in `[A-Za-z0-9._-]` (others become `_`), without leading dots and
/// at most 100 characters; `None` if nothing is left.
fn sanitize(name: &str) -> Option<String> {
    let last = name.rsplit(['/', '\\']).next().unwrap_or_default();
    let clean: String = last
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-') {
                c
            } else {
                '_'
            }
        })
        .collect();
    let clean: String = clean.trim_start_matches('.').chars().take(100).collect();
    (!clean.is_empty()).then_some(clean)
}

/// A file extension for a media type.
fn extension(content_type: &str) -> &'static str {
    match content_type {
        "application/zip" => "zip",
        "application/gzip" => "gz",
        "application/x-tar" => "tar",
        "application/pdf" => "pdf",
        "image/png" => "png",
        "image/jpeg" => "jpg",
        _ => "bin",
    }
}

/// The artifact directory: `dir` (created 0700 if missing), or a new 0700 temporary directory
/// named after `server` that is kept after exit. Canonical, so its `file://` URIs are absolute.
pub(crate) fn artifact_dir(server: &str, dir: Option<&Path>) -> Result<PathBuf, Error> {
    let fail = |path: &Path| {
        let path = path.to_path_buf();
        move |source| Error::ArtifactDir { path, source }
    };
    let dir = match dir {
        Some(dir) => {
            if !dir.is_dir() {
                private_dir_builder()
                    .recursive(true)
                    .create(dir)
                    .map_err(fail(dir))?;
            }
            dir.to_path_buf()
        }
        None => {
            let prefix = format!(
                "{}-mcp-",
                sanitize(server).unwrap_or_else(|| "openapi".into())
            );
            let mut builder = tempfile::Builder::new();
            builder.prefix(&prefix);
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt as _;
                builder.permissions(std::fs::Permissions::from_mode(0o700));
            }
            builder
                .tempdir()
                .map_err(fail(&std::env::temp_dir()))?
                .keep()
        }
    };
    std::fs::canonicalize(&dir).map_err(fail(&dir))
}

fn private_dir_builder() -> std::fs::DirBuilder {
    #[allow(unused_mut)]
    let mut builder = std::fs::DirBuilder::new();
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt as _;
        builder.mode(0o700);
    }
    builder
}

#[cfg(test)]
mod tests {
    use http::Response;

    use super::*;
    use crate::testing::zip;

    fn parts(status: u16, headers: &[(&str, &str)]) -> Parts {
        let mut r = Response::builder().status(status);
        for (k, v) in headers {
            r = r.header(*k, *v);
        }
        r.body(()).unwrap().into_parts().0
    }

    fn json_response(status: u16, body: &str) -> CallToolResult {
        let dir = tempfile::tempdir().unwrap();
        to_result(
            "t",
            &parts(status, &[("content-type", "application/json")]),
            body.as_bytes(),
            dir.path(),
        )
    }

    fn text(r: &CallToolResult) -> &str {
        &r.content[0].as_text().unwrap().text
    }

    #[test]
    fn a_json_object_is_text_and_structured_content() {
        let r = json_response(200, r#"{"ok": true, "errors": []}"#);
        assert_eq!(r.is_error, Some(false));
        assert_eq!(
            r.structured_content,
            Some(json!({"ok": true, "errors": []}))
        );
        let sent: Value = serde_json::from_str(text(&r)).unwrap();
        assert_eq!(sent, json!({"ok": true, "errors": []}));
    }

    #[test]
    fn arrays_and_scalars_are_wrapped() {
        let r = json_response(200, r#"[{"id": "r1"}]"#);
        assert_eq!(r.structured_content, Some(json!({"items": [{"id": "r1"}]})));
        assert_eq!(text(&r), r#"{"items":[{"id":"r1"}]}"#);
        let r = json_response(200, "42");
        assert_eq!(r.structured_content, Some(json!({"value": 42})));
    }

    #[test]
    fn text_stays_text() {
        let dir = tempfile::tempdir().unwrap();
        for content_type in ["application/x-yaml", "text/plain; charset=utf-8"] {
            let r = to_result(
                "t",
                &parts(200, &[("content-type", content_type)]),
                b"a: 1\n",
                dir.path(),
            );
            assert_eq!(r.is_error, Some(false));
            assert_eq!(r.structured_content, None);
            assert_eq!(text(&r), "a: 1\n");
            assert_eq!(r.content.len(), 1);
        }
        let r = to_result("t", &parts(204, &[]), b"", dir.path());
        assert_eq!(text(&r), "HTTP 204 No Content");
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 0);
    }

    #[test]
    fn a_zip_is_saved_as_a_private_file() {
        let dir = tempfile::tempdir().unwrap();
        let dir = dir.path().canonicalize().unwrap();
        let r = to_result(
            "export_zip",
            &parts(
                200,
                &[
                    ("content-type", "application/zip"),
                    ("content-disposition", "attachment; filename=\"scene.zip\""),
                ],
            ),
            &zip(),
            &dir,
        );
        assert_eq!(r.is_error, Some(false));
        let s = r.structured_content.clone().unwrap();
        let path = PathBuf::from(s["path"].as_str().unwrap());
        assert_eq!(path.parent().unwrap(), dir);
        let name = path.file_name().unwrap().to_str().unwrap();
        assert!(
            name.starts_with("scene-") && name.ends_with(".zip"),
            "{name}"
        );
        assert_eq!(std::fs::read(&path).unwrap(), zip());
        assert_eq!(s["bytes"], 22);
        assert_eq!(s["content_type"], "application/zip");
        assert_eq!(s["filename"], "scene.zip");
        let link = r.content[1].as_resource_link().unwrap();
        assert_eq!(link.uri, url::Url::from_file_path(&path).unwrap().as_str());
        assert!(link.uri.starts_with("file:///"));
        assert_eq!(link.name, "scene.zip");
        assert_eq!(link.mime_type.as_deref(), Some("application/zip"));
        assert_eq!(link.size, Some(22));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = std::fs::metadata(&path).unwrap().permissions().mode();
            assert_eq!(mode & 0o777, 0o600);
        }
    }

    #[test]
    fn saved_names_come_from_content_disposition_sanitized() {
        let dir = tempfile::tempdir().unwrap();
        for (disposition, filename, prefix) in [
            (
                "attachment; filename=\"../../evil.zip\"",
                "evil.zip",
                "evil-",
            ),
            (
                "attachment; filename*=UTF-8''r%C3%A9sum%C3%A9.pdf",
                "r_sum_.pdf",
                "r_sum_-",
            ),
            (
                "attachment; filename=\"..\"",
                "export_zip.bin",
                "export_zip-",
            ),
            ("inline", "export_zip.bin", "export_zip-"),
        ] {
            let r = to_result(
                "export_zip",
                &parts(
                    200,
                    &[
                        ("content-type", "application/octet-stream"),
                        ("content-disposition", disposition),
                    ],
                ),
                &[0, 159, 146, 150],
                dir.path(),
            );
            let s = r.structured_content.unwrap();
            assert_eq!(s["filename"], filename, "{disposition}");
            let path = PathBuf::from(s["path"].as_str().unwrap());
            assert_eq!(path.parent().unwrap(), dir.path());
            let name = path.file_name().unwrap().to_str().unwrap();
            assert!(name.starts_with(prefix), "{name}");
        }
    }

    #[test]
    fn http_errors_are_error_results() {
        let r = json_response(
            400,
            r#"{"error": {"code": "invalid_path", "message": "m"}}"#,
        );
        assert_eq!(r.is_error, Some(true));
        assert_eq!(
            r.structured_content,
            Some(
                json!({"http_status": 400, "body": {"error": {"code": "invalid_path", "message": "m"}}})
            )
        );
        let r = json_response(503, r#"{"ready": false}"#);
        assert_eq!(r.structured_content.unwrap()["http_status"], 503);
        let dir = tempfile::tempdir().unwrap();
        let r = to_result(
            "t",
            &parts(422, &[("content-type", "text/plain; charset=utf-8")]),
            b"Failed to deserialize the JSON body",
            dir.path(),
        );
        assert_eq!(r.is_error, Some(true));
        assert_eq!(
            r.structured_content,
            Some(json!({"http_status": 422, "body": "Failed to deserialize the JSON body"}))
        );
        let r = to_result("t", &parts(404, &[]), b"", dir.path());
        assert_eq!(
            r.structured_content,
            Some(json!({"http_status": 404, "body": null}))
        );
    }

    #[test]
    fn a_given_artifact_dir_is_created_private() {
        let root = tempfile::tempdir().unwrap();
        let dir = artifact_dir("s", Some(&root.path().join("a/b"))).unwrap();
        assert!(dir.is_absolute() && dir.is_dir());
        assert!(dir.ends_with("a/b"));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = std::fs::metadata(&dir).unwrap().permissions().mode();
            assert_eq!(mode & 0o777, 0o700);
        }
        // An existing directory is used as it is.
        assert_eq!(artifact_dir("s", Some(&dir)).unwrap(), dir);
    }
}
