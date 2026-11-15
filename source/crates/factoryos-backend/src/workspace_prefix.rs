//! Workspace prefixes: `/workspace/<name>/…` reaches the same routes as `/…`. The prefix is
//! stripped before routing, so the full path is the same in the viewer, at the load balancer and
//! here, and nothing in between has to rewrite it. Unprefixed paths keep working (local backends).

use std::sync::Arc;

use axum::extract::{Request, State};
use axum::middleware::Next;
use axum::http::uri::PathAndQuery;
use axum::http::{HeaderValue, StatusCode, Uri, header};
use axum::response::{IntoResponse as _, Response};
use factoryos_protocol::workspace;

/// [`strip`] as middleware around the router; the state is `--workspace`.
pub async fn middleware(State(pinned): State<Option<Arc<str>>>, request: Request, next: Next) -> Response {
    strip(pinned.as_deref(), request, async |request| next.run(request).await).await
}

/// Strip a workspace prefix off `request`, then run `next` (the router). With `pinned`, other
/// workspaces' prefixes are 404, so a misrouted request fails loudly.
pub async fn strip(pinned: Option<&str>, mut request: Request, next: impl AsyncFnOnce(Request) -> Response) -> Response {
    let Some((name, rest)) = workspace::split_path(request.uri().path()) else {
        return next(request).await;
    };
    if pinned.is_some_and(|pinned| pinned != name) {
        return (StatusCode::NOT_FOUND, format!("this backend serves workspace {}, not {name}", pinned.unwrap_or_default()))
            .into_response();
    }
    let prefix = format!("{}{name}", workspace::PREFIX);
    let query = request.uri().query().map(|query| format!("?{query}")).unwrap_or_default();
    if rest.is_empty() {
        // `/workspace/<name>` → `/workspace/<name>/`, so the web viewer's relative URLs resolve.
        return (StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, format!("{prefix}/{query}"))]).into_response();
    }
    match strip_uri(request.uri(), &format!("{rest}{query}")) {
        Some(uri) => *request.uri_mut() = uri,
        None => return StatusCode::BAD_REQUEST.into_response(),
    }
    let mut response = next(request).await;
    // Redirects within the backend (e.g. a directory to its `/`) stay in the workspace.
    if let Some(location) = response.headers().get(header::LOCATION).and_then(|value| value.to_str().ok())
        && location.starts_with('/')
        && !location.starts_with("//")
        && let Ok(value) = HeaderValue::from_str(&format!("{prefix}{location}"))
    {
        response.headers_mut().insert(header::LOCATION, value);
    }
    response
}

fn strip_uri(uri: &Uri, path_and_query: &str) -> Option<Uri> {
    let mut parts = uri.clone().into_parts();
    parts.path_and_query = Some(PathAndQuery::try_from(path_and_query).ok()?);
    Uri::from_parts(parts).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;

    /// Run `strip` with a router stand-in that echoes the path it got, or redirects `/pkg`.
    async fn run(pinned: Option<&str>, uri: &str) -> (StatusCode, String, Option<String>) {
        let request = Request::builder().uri(uri).body(Body::empty()).unwrap();
        let response = strip(pinned, request, async |request: Request| {
            let path = request.uri().path_and_query().unwrap().to_string();
            if request.uri().path() == "/pkg" {
                (StatusCode::TEMPORARY_REDIRECT, [(header::LOCATION, "/pkg/")]).into_response()
            } else {
                path.into_response()
            }
        })
        .await;
        let status = response.status();
        let location = response.headers().get(header::LOCATION).map(|v| v.to_str().unwrap().to_owned());
        let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
        (status, String::from_utf8(body.to_vec()).unwrap(), location)
    }

    #[tokio::test]
    async fn strips_the_prefix() {
        assert_eq!(run(None, "/workspace/a/api/v1/health?x=1").await.1, "/api/v1/health?x=1");
        assert_eq!(run(None, "/workspace/a/").await.1, "/");
        assert_eq!(run(None, "/api/v1/health").await.1, "/api/v1/health");
        assert_eq!(run(None, "/workspace/../x").await.1, "/workspace/../x");
    }

    #[tokio::test]
    async fn redirects_into_the_workspace() {
        let (status, _, location) = run(None, "/workspace/a?x=1").await;
        assert_eq!((status, location.as_deref()), (StatusCode::PERMANENT_REDIRECT, Some("/workspace/a/?x=1")));
        let (_, _, location) = run(None, "/workspace/a/pkg").await;
        assert_eq!(location.as_deref(), Some("/workspace/a/pkg/"));
    }

    #[tokio::test]
    async fn pinned_rejects_other_workspaces() {
        assert_eq!(run(Some("a"), "/workspace/a/x").await.1, "/x");
        assert_eq!(run(Some("a"), "/workspace/b/x").await.0, StatusCode::NOT_FOUND);
        assert_eq!(run(Some("a"), "/x").await.1, "/x");
    }
}
