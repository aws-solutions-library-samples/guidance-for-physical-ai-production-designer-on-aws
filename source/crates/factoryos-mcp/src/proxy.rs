//! The tower service openapi-mcp calls: each request goes to the workspace's backend over HTTP.
//!
//! The virtual paths of cell-runner verbs ([`crate::spec::VERB_PREFIX`]`<verb>`) become
//! `POST /api/v1/cell/command` with `{"type": <verb>, "params": <body>}`. A backend that can't
//! be reached is a `502` whose body says so (the model reads it as the tool's error).

use std::convert::Infallible;
use std::future::Future;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::Duration;

use bytes::Bytes;
use http_body_util::{BodyExt as _, Full};

use crate::spec::VERB_PREFIX;

pub const CELL_COMMAND: &str = "/api/v1/cell/command";

#[derive(Clone)]
pub struct Proxy {
    client: reqwest::Client,
    /// `http://127.0.0.1:80`, no trailing slash.
    base: String,
}

impl Proxy {
    pub fn new(base: &str, timeout: Duration) -> Self {
        let client = reqwest::Client::builder()
            .timeout(timeout)
            .connect_timeout(Duration::from_secs(5))
            .no_proxy()
            .build()
            .expect("a reqwest client");
        Self { client, base: base.trim_end_matches('/').to_owned() }
    }

    async fn forward(self, request: http::Request<Full<Bytes>>) -> http::Response<Full<Bytes>> {
        let (parts, body) = request.into_parts();
        let body = match body.collect().await {
            Ok(b) => b.to_bytes(),
            Err(never) => match never {},
        };
        let path_and_query = parts.uri.path_and_query().map_or("/", |p| p.as_str()).to_owned();
        let verb = parts.uri.path().starts_with(VERB_PREFIX);
        let (method, path_and_query, body, json) = match parts.uri.path().strip_prefix(VERB_PREFIX) {
            Some(verb) => {
                let params: serde_json::Value = if body.is_empty() { serde_json::Value::Null } else { serde_json::from_slice(&body).unwrap_or_default() };
                let params = match params {
                    serde_json::Value::Object(map) if map.is_empty() => serde_json::Value::Null,
                    other => other,
                };
                let mut command = serde_json::json!({"type": verb});
                if !params.is_null() {
                    command["params"] = params;
                }
                (http::Method::POST, CELL_COMMAND.to_owned(), Bytes::from(command.to_string()), true)
            }
            None => {
                let json = parts.headers.get(http::header::CONTENT_TYPE).is_some_and(|v| v.as_bytes().starts_with(b"application/json"));
                (parts.method.clone(), path_and_query, body, json)
            }
        };
        let url = format!("{}{}", self.base, path_and_query);
        let method = reqwest::Method::from_bytes(method.as_str().as_bytes()).unwrap_or(reqwest::Method::POST);
        let mut builder = self.client.request(method, &url);
        if json {
            builder = builder.header(reqwest::header::CONTENT_TYPE, "application/json");
        } else if let Some(ct) = parts.headers.get(http::header::CONTENT_TYPE) {
            builder = builder.header(reqwest::header::CONTENT_TYPE, ct.as_bytes());
        }
        if !body.is_empty() || json {
            builder = builder.body(if body.is_empty() { Bytes::from_static(b"{}") } else { body });
        }
        match builder.send().await {
            Ok(response) => {
                let mut status = response.status().as_u16();
                let content_type = response.headers().get(reqwest::header::CONTENT_TYPE).and_then(|v| v.to_str().ok()).map(str::to_owned);
                let disposition = response.headers().get(reqwest::header::CONTENT_DISPOSITION).and_then(|v| v.to_str().ok()).map(str::to_owned);
                match response.bytes().await {
                    Ok(bytes) => {
                        // The cell-runner answers a refused verb with 200 {"ok": false, "error"}: a tool error.
                        if verb && status == 200 && serde_json::from_slice::<serde_json::Value>(&bytes).is_ok_and(|v| v["ok"] == false) {
                            status = 422;
                        }
                        let mut out = http::Response::builder().status(status);
                        if let Some(ct) = content_type.as_deref() {
                            out = out.header("content-type", ct);
                        }
                        if let Some(cd) = disposition.as_deref() {
                            out = out.header("content-disposition", cd);
                        }
                        out.body(Full::new(bytes)).unwrap_or_else(|e| failure(502, &format!("bad response: {e}")))
                    }
                    Err(e) => failure(502, &format!("reading the backend's answer failed: {e}")),
                }
            }
            Err(e) if e.is_timeout() => failure(504, &format!("the backend did not answer {url} in time: {e}")),
            Err(e) => failure(
                502,
                &format!("the Physical AI Production Designer backend at {} is not reachable ({e}); is factoryos-backend running on this workspace?", self.base),
            ),
        }
    }
}

fn failure(status: u16, message: &str) -> http::Response<Full<Bytes>> {
    http::Response::builder()
        .status(status)
        .header("content-type", "text/plain; charset=utf-8")
        .body(Full::new(Bytes::from(message.to_owned())))
        .expect("a valid response")
}

impl tower::Service<http::Request<Full<Bytes>>> for Proxy {
    type Response = http::Response<Full<Bytes>>;
    type Error = Infallible;
    type Future = Pin<Box<dyn Future<Output = Result<Self::Response, Infallible>> + Send>>;

    fn poll_ready(&mut self, _cx: &mut Context<'_>) -> Poll<Result<(), Infallible>> {
        Poll::Ready(Ok(()))
    }

    fn call(&mut self, request: http::Request<Full<Bytes>>) -> Self::Future {
        let this = self.clone();
        Box::pin(async move { Ok(this.forward(request).await) })
    }
}
