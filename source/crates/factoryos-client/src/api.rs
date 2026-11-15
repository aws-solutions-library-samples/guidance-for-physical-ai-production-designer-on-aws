use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::Callback;
use crate::protocol::workspace::WorkspaceStatus;

/// Starts the errors [`Api`] returns for a workspace that is not up yet (the provisioner answered
/// `503` with a [`WorkspaceStatus`]); [`workspace_status`] gets that back out.
pub const WORKSPACE_NOT_READY: &str = "workspace not ready";

/// The [`WorkspaceStatus`] in an [`Api`] error for a workspace that is not up yet.
pub fn workspace_status(err: &str) -> Option<WorkspaceStatus> {
    serde_json::from_str(err.strip_prefix(WORKSPACE_NOT_READY)?.strip_prefix(": ")?).ok()
}

/// HTTP client for `/api/v1/…`.
#[derive(Clone, Debug)]
pub struct Api {
    base: String,
    /// Requests answer with an error after this long without a response ([`Api::with_timeout`]).
    timeout: Option<std::time::Duration>,
}

impl Api {
    /// `base` is the backend's origin, e.g. `http://127.0.0.1:8080`.
    pub fn new(base: impl Into<String>) -> Self {
        Self {
            base: base.into().trim_end_matches('/').to_owned(),
            timeout: None,
        }
    }

    /// The same backend, with requests that give up after `timeout` (answering with an error
    /// starting with [`crate::TIMED_OUT`]): for requests something waits on, e.g. the storage
    /// mode or a health check, which must not hang when the backend goes away mid-request. Not
    /// for requests that legitimately take long (a compose, a plan load).
    pub fn with_timeout(&self, timeout: std::time::Duration) -> Self {
        Self { base: self.base.clone(), timeout: Some(timeout) }
    }

    /// `on_done` with this API's deadline, if it has one; and the request's own timeout (natively).
    fn deadline<T: 'static>(&self, request: &mut ehttp::Request, on_done: Callback<T>) -> Callback<T> {
        let Some(timeout) = self.timeout else { return on_done };
        request.timeout = Some(timeout);
        crate::with_deadline(timeout, request.url.clone(), on_done)
    }

    pub fn base(&self) -> &str {
        &self.base
    }

    pub fn get<T: DeserializeOwned + 'static>(&self, path: &str, on_done: impl FnOnce(Result<T, String>) + Send + 'static) {
        self.fetch(ehttp::Request::get(self.url(path)), Box::new(on_done));
    }

    pub fn post<T: DeserializeOwned + 'static>(&self, path: &str, on_done: impl FnOnce(Result<T, String>) + Send + 'static) {
        self.fetch(ehttp::Request::post(self.url(path), Vec::new()), Box::new(on_done));
    }

    pub fn post_json<B: Serialize, T: DeserializeOwned + 'static>(
        &self,
        path: &str,
        body: &B,
        on_done: impl FnOnce(Result<T, String>) + Send + 'static,
    ) {
        self.send_json(ehttp::Method::POST, path, body, Box::new(on_done));
    }

    /// `DELETE`, expecting a JSON answer.
    pub fn delete<T: DeserializeOwned + 'static>(&self, path: &str, on_done: impl FnOnce(Result<T, String>) + Send + 'static) {
        let request = ehttp::Request::new(ehttp::Method::DELETE, self.url(path), &[("Accept", "application/json")]);
        self.fetch(request, Box::new(on_done));
    }

    /// `DELETE` with a JSON body (e.g. a confirmation), expecting a JSON answer.
    pub fn delete_json<B: Serialize, T: DeserializeOwned + 'static>(
        &self,
        path: &str,
        body: &B,
        on_done: impl FnOnce(Result<T, String>) + Send + 'static,
    ) {
        self.send_json(ehttp::Method::DELETE, path, body, Box::new(on_done));
    }

    pub fn put_json<B: Serialize, T: DeserializeOwned + 'static>(
        &self,
        path: &str,
        body: &B,
        on_done: impl FnOnce(Result<T, String>) + Send + 'static,
    ) {
        self.send_json(ehttp::Method::PUT, path, body, Box::new(on_done));
    }

    /// `GET` a raw body (e.g. a generated file) as text: invalid UTF-8 is replaced, and bodies
    /// over `max_bytes` are refused rather than handed over.
    pub fn get_text(&self, path: &str, max_bytes: usize, on_done: impl FnOnce(Result<String, String>) + Send + 'static) {
        let mut request = ehttp::Request::get(self.url(path));
        let on_done = self.deadline(&mut request, Box::new(on_done));
        #[cfg(not(target_arch = "wasm32"))]
        let on_done: Box<dyn FnOnce(Result<String, String>) + Send> = {
            let retry = request.clone();
            Box::new(move |result: Result<String, String>| match result {
                Err(err) if err.starts_with(crate::auth::LOGIN_REQUIRED) && crate::auth::renew(&retry.url) => {
                    ehttp::fetch(with_cookies(retry), move |response| on_done(response.and_then(|r| parse_text(&r, max_bytes))));
                }
                result => on_done(result),
            })
        };
        ehttp::fetch(with_cookies(request), move |response| on_done(response.and_then(|r| parse_text(&r, max_bytes))));
    }

    /// `GET` a raw body (a file to download) as it is; bodies over `max_bytes` are refused. An error
    /// starts with the status (`404 Not Found: …`) when there was one.
    pub fn get_bytes(&self, path: &str, max_bytes: usize, on_done: impl FnOnce(Result<Vec<u8>, String>) + Send + 'static) {
        let request = ehttp::Request::get(self.url(path));
        self.fetch_raw(request, move |r| parse_bytes(r, max_bytes), Box::new(on_done));
    }

    /// `PUT` a raw body, ignoring the answer's body (e.g. to the edge's data path, where S3 answers
    /// with none). An error starts with the status when there was one.
    pub fn put_raw(&self, path: &str, bytes: Vec<u8>, on_done: impl FnOnce(Result<(), String>) + Send + 'static) {
        let request = ehttp::Request::new(ehttp::Method::PUT, self.url(path), &[("Content-Type", "application/octet-stream")]).with_body(bytes);
        self.fetch_raw(request, |r| parse_bytes(r, usize::MAX).map(drop), Box::new(on_done));
    }

    /// The same API at the origin of [`Self::base`] (`https://host/workspace/jdoe` → `https://host`),
    /// with the same timeout: for paths the edge serves at the origin, whatever the backend's prefix.
    pub fn at_origin(&self) -> Self {
        Self { base: origin_of(&self.base).to_owned(), timeout: self.timeout }
    }

    /// `request` with cookies and this API's deadline, its response by `parse`; natively a session
    /// that expired is renewed and the request sent once more (as [`Self::fetch`]).
    fn fetch_raw<T: 'static>(
        &self,
        mut request: ehttp::Request,
        parse: impl Fn(ehttp::Response) -> Result<T, String> + Send + Sync + 'static,
        on_done: Callback<T>,
    ) {
        let on_done = self.deadline(&mut request, on_done);
        let parse = std::sync::Arc::new(parse);
        #[cfg(not(target_arch = "wasm32"))]
        let on_done: Callback<T> = {
            let (retry, parse) = (request.clone(), parse.clone());
            Box::new(move |result: Result<T, String>| match result {
                Err(err) if err.starts_with(crate::auth::LOGIN_REQUIRED) && crate::auth::renew(&retry.url) => {
                    ehttp::fetch(with_cookies(retry), move |response| on_done(response.and_then(|r| parse(r))));
                }
                result => on_done(result),
            })
        };
        ehttp::fetch(with_cookies(request), move |response| on_done(response.and_then(|r| parse(r))));
    }

    /// `PUT` a raw body (an uploaded file), expecting a JSON answer.
    pub fn put_bytes<T: DeserializeOwned + 'static>(&self, path: &str, bytes: Vec<u8>, on_done: impl FnOnce(Result<T, String>) + Send + 'static) {
        let headers = [("Accept", "application/json"), ("Content-Type", "application/octet-stream")];
        let request = ehttp::Request::new(ehttp::Method::PUT, self.url(path), &headers).with_body(bytes);
        self.fetch(request, Box::new(on_done));
    }

    /// The full URL of `path` (for the browser to open, e.g. a download).
    pub fn url_for(&self, path: &str) -> String {
        self.url(path)
    }

    /// Like [`Self::get`], awaitable (for startup, before there's a UI).
    pub async fn get_async<T: DeserializeOwned>(&self, path: &str) -> Result<T, String> {
        let request = ehttp::Request::get(self.url(path));
        let response = ehttp::fetch_async(with_cookies(request.clone())).await?;
        match parse(&response) {
            #[cfg(not(target_arch = "wasm32"))]
            Err(err) if err.starts_with(crate::auth::LOGIN_REQUIRED) => {
                let url = request.url.clone();
                if tokio::task::spawn_blocking(move || crate::auth::renew(&url)).await.unwrap_or(false) {
                    parse(&ehttp::fetch_async(with_cookies(request)).await?)
                } else {
                    Err(err)
                }
            }
            result => result,
        }
    }

    fn send_json<B: Serialize, T: DeserializeOwned + 'static>(
        &self,
        method: ehttp::Method,
        path: &str,
        body: &B,
        on_done: Callback<T>,
    ) {
        let body = match serde_json::to_vec(body) {
            Ok(body) => body,
            Err(err) => return on_done(Err(err.to_string())),
        };
        // Not `Request::post` + `headers.insert`: that would keep its `text/plain` content type first.
        let headers = [("Accept", "application/json"), ("Content-Type", "application/json")];
        let request = ehttp::Request::new(method, self.url(path), &headers).with_body(body);
        self.fetch(request, on_done);
    }

    fn fetch<T: DeserializeOwned + 'static>(&self, mut request: ehttp::Request, on_done: Callback<T>) {
        let on_done = self.deadline(&mut request, on_done);
        // Natively, a session that expired while the viewer ran is renewed (silently, where the
        // build can: `auth::renew`) and the request sent once more.
        #[cfg(not(target_arch = "wasm32"))]
        let on_done: Callback<T> = {
            let retry = request.clone();
            Box::new(move |result: Result<T, String>| match result {
                Err(err) if err.starts_with(crate::auth::LOGIN_REQUIRED) && crate::auth::renew(&retry.url) => {
                    ehttp::fetch(with_cookies(retry), move |response| on_done(response.and_then(|r| parse(&r))));
                }
                result => on_done(result),
            })
        };
        ehttp::fetch(with_cookies(request), move |response| on_done(response.and_then(|r| parse(&r))));
    }

    fn url(&self, path: &str) -> String {
        format!("{}{path}", self.base)
    }
}

/// `scheme://host[:port]` of `url`; a URL without a scheme (relative, `""`) has no origin of its own:
/// `""` (the page's, in the browser).
pub(crate) fn origin_of(url: &str) -> &str {
    let Some(scheme_end) = url.find("://") else { return "" };
    let host_start = scheme_end + 3;
    match url[host_start..].find(['/', '?', '#']) {
        Some(end) => &url[..host_start + end],
        None => url,
    }
}

/// In the browser, send cookies to the page's own origin: behind CloudFront (the workspace edge's login)
/// the session cookie is what lets a request through, and without it the edge redirects to the
/// login page, which `fetch` reports as a network error. ehttp omits credentials by default.
/// Natively, the session cookie goes along explicitly (see [`crate::auth`]).
fn with_cookies(request: ehttp::Request) -> ehttp::Request {
    #[cfg(target_arch = "wasm32")]
    let request = request.with_credentials(ehttp::Credentials::SameOrigin);
    #[cfg(not(target_arch = "wasm32"))]
    let request = {
        let mut request = request;
        if let Some(cookie) = crate::auth::cookie_header(&request.url) {
            request.headers.insert("Cookie", cookie);
        }
        request
    };
    request
}

fn parse_text(response: &ehttp::Response, max_bytes: usize) -> Result<String, String> {
    #[cfg(not(target_arch = "wasm32"))]
    if !response.url.contains("/api/") && response.content_type().is_some_and(|kind| kind.starts_with("text/html")) {
        return Err(format!("{}: the backend redirected to {}", crate::auth::LOGIN_REQUIRED, response.url));
    }
    if !response.ok {
        let body = response.text().unwrap_or_default();
        return Err(format!("{} {}: {body}", response.status, response.status_text));
    }
    if response.bytes.len() > max_bytes {
        return Err(format!("{} bytes, more than the {max_bytes} shown here", response.bytes.len()));
    }
    Ok(String::from_utf8_lossy(&response.bytes).into_owned())
}

fn parse_bytes(response: ehttp::Response, max_bytes: usize) -> Result<Vec<u8>, String> {
    // The edge data path never serves HTML: a page here is a CloudFront default behaviour (no `/_data/*`
    // behaviour deployed yet), not the object. A failure, so the data source takes the next way.
    if response.url.contains("/_data/") && response.content_type().is_some_and(|kind| kind.starts_with("text/html")) {
        return Err(format!("502 an HTML page instead of the object at {}", response.url));
    }
    #[cfg(not(target_arch = "wasm32"))]
    if !response.url.contains("/api/") && response.content_type().is_some_and(|kind| kind.starts_with("text/html")) {
        return Err(format!("{}: the backend redirected to {}", crate::auth::LOGIN_REQUIRED, response.url));
    }
    if !response.ok {
        let body = response.text().unwrap_or_default();
        return Err(format!("{} {}: {body}", response.status, response.status_text));
    }
    if response.bytes.len() > max_bytes {
        return Err(format!("{} bytes, more than {max_bytes}", response.bytes.len()));
    }
    Ok(response.bytes)
}

fn parse<T: DeserializeOwned>(response: &ehttp::Response) -> Result<T, String> {
    // Natively redirects are followed: an API request that ends on a page off the API (the edge's
    // login, its identity provider's) means the session is missing or expired.
    #[cfg(not(target_arch = "wasm32"))]
    if !response.url.contains("/api/") && response.content_type().is_some_and(|kind| kind.starts_with("text/html")) {
        return Err(format!("{}: the backend redirected to {}", crate::auth::LOGIN_REQUIRED, response.url));
    }
    if response.status == 503
        && response.content_type().is_some_and(|kind| kind.starts_with("application/json"))
        && let Ok(status) = serde_json::from_slice::<WorkspaceStatus>(&response.bytes)
    {
        return Err(format!("{WORKSPACE_NOT_READY}: {}", serde_json::to_string(&status).unwrap_or_default()));
    }
    if !response.ok {
        let body = response.text().unwrap_or_default();
        return Err(format!("{} {}: {body}", response.status, response.status_text));
    }
    // Empty bodies (204) parse as `()` / `null`.
    let bytes = if response.bytes.is_empty() { b"null".as_slice() } else { &response.bytes };
    serde_json::from_slice(bytes).map_err(|err| format!("invalid response from {}: {err}", response.url))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::workspace::WorkspaceState;

    fn response(status: u16, content_type: &str, body: &str) -> ehttp::Response {
        ehttp::Response {
            url: "https://h/api/v1/health".to_owned(),
            ok: (200..300).contains(&status),
            status,
            status_text: String::new(),
            headers: ehttp::Headers::new(&[("content-type", content_type)]),
            bytes: body.as_bytes().to_vec(),
        }
    }

    #[test]
    fn an_html_page_on_the_edge_data_path_is_a_failure() {
        let mut page = response(200, "text/html; charset=utf-8", "<!doctype html>");
        page.url = "https://h/_data/user/products/a/product.json".to_owned();
        let err = parse_bytes(page, 1 << 20).unwrap_err();
        assert!(err.starts_with("502 "), "{err}");
        let mut object = response(200, "application/octet-stream", "{}");
        object.url = "https://h/_data/user/products/a/product.json".to_owned();
        assert_eq!(parse_bytes(object, 1 << 20).unwrap(), b"{}");
    }

    #[test]
    fn origins_drop_the_path() {
        assert_eq!(origin_of("https://h.example/workspace/jdoe"), "https://h.example");
        assert_eq!(origin_of("http://127.0.0.1:8080"), "http://127.0.0.1:8080");
        assert_eq!(origin_of("http://127.0.0.1:8080/"), "http://127.0.0.1:8080");
        assert_eq!(origin_of(""), "");
        assert_eq!(origin_of("/workspace/jdoe"), "");
        assert_eq!(Api::new("https://h/workspace/jdoe").at_origin().base(), "https://h");
    }

    #[test]
    fn not_ready_workspaces_are_typed() {
        let body = r#"{"name":"jdoe","state":"booting","step":"installing"}"#;
        let err = parse::<serde_json::Value>(&response(503, "application/json", body)).unwrap_err();
        let status = workspace_status(&err).expect(&err);
        assert_eq!((status.name.as_str(), status.state), ("jdoe", WorkspaceState::Booting));
        // A plain 503 (e.g. from a load balancer) stays an ordinary error.
        let err = parse::<serde_json::Value>(&response(503, "text/plain", "busy")).unwrap_err();
        assert!(workspace_status(&err).is_none(), "{err}");
    }
}
