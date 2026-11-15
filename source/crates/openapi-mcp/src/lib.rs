//! An OpenAPI-described HTTP service as a stdio MCP server: one tool per operation.
//!
//! [`Facade::new`] reads an OpenAPI 3.0/3.1 document (as JSON) and turns every operation into an
//! MCP tool. A `tools/call` becomes the operation's HTTP request, which `service` (an axum
//! `Router`, say) answers in process: no socket is opened, and the service's own handlers,
//! validation and path sandboxing apply exactly as they do over REST.
//!
//! - **Tools:** the name is the operationId, the title the summary, and the description the
//!   summary, the description and an `HTTP: POST /path` footer. GET operations and those named in
//!   [`Options::read_only`] are marked read-only.
//! - **Arguments:** path and query parameters and the fields of a JSON object body are top-level
//!   arguments. A body that can't be flattened is `body`, a YAML or text body is the string `yaml`
//!   or `text`, and a multipart body's file fields are base64 strings. An unknown or missing
//!   argument is an error result with code `invalid_arguments`.
//! - **Results:** a JSON object is text plus `structuredContent` (an array is wrapped as
//!   `{items}`, a scalar as `{value}`), and text is text. Anything else (a zip, say) is saved as a
//!   0600 file in the 0700 artifact directory and returned as a `file://` resource link plus
//!   `{path, bytes, content_type, filename}`. HTTP ≥ 400 is an error result with
//!   `{http_status, body}`.
//! - **Calls** run one at a time, each in a task of its own, so that a handler panic becomes an
//!   error result (where panics unwind; with `panic = "abort"` it ends the process).
//! - **Serving:** [`Facade::serve_stdio`] runs until stdin closes and tells whether a client used
//!   the session, so that a server running beside it (REST, say) can stop with the session.

#![forbid(unsafe_code)]

mod request;
mod response;
mod schema;
mod server;
mod spec;

use std::convert::Infallible;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use bytes::Bytes;
use http_body_util::BodyExt as _;
use rmcp::model::{CallToolResult, Implementation, ServerCapabilities, ServerConfig, Tool};
use rmcp::ErrorData;
use serde_json::Value;
use tower::ServiceExt as _;

pub use rmcp;

/// The request body type the service is called with.
pub type Body = http_body_util::Full<Bytes>;

/// How the facade presents itself and where it puts files.
#[derive(Clone, Debug, Default)]
pub struct Options {
    /// `serverInfo.name`, e.g. `env!("CARGO_PKG_NAME")`.
    pub name: String,
    /// `serverInfo.version`, e.g. `env!("CARGO_PKG_VERSION")`.
    pub version: String,
    /// Appended to the generated server instructions.
    pub instructions: Option<String>,
    /// Where binary responses are saved; created 0700 if missing. `None`: a new 0700 temporary
    /// directory, kept after exit.
    pub artifact_dir: Option<PathBuf>,
    /// operationIds that only read although their method isn't GET.
    pub read_only: Vec<String>,
}

impl Options {
    /// `serverInfo` name and version; everything else is left at its default.
    pub fn new(name: impl Into<String>, version: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            version: version.into(),
            ..Self::default()
        }
    }
}

/// Why a facade can't be built or served.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("OpenAPI document: {0}")]
    Spec(String),
    #[error("artifact directory {}: {source}", path.display())]
    ArtifactDir {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("MCP: {0}")]
    Mcp(String),
}

/// Every operation of an OpenAPI document as an MCP tool, executed by `service` in process.
pub struct Facade<S> {
    inner: Arc<Inner<S>>,
}

impl<S> Clone for Facade<S> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

struct Inner<S> {
    service: S,
    /// `tools[i]` is the tool of `operations[i]`.
    tools: Vec<Tool>,
    operations: Vec<spec::Operation>,
    artifact_dir: PathBuf,
    info: ServerConfig,
    /// Held for the whole of a call, so that calls run one at a time.
    turn: tokio::sync::Mutex<()>,
}

impl<S, B> Facade<S>
where
    S: tower::Service<http::Request<Body>, Response = http::Response<B>, Error = Infallible>
        + Clone
        + Send
        + Sync
        + 'static,
    S::Future: Send + 'static,
    B: http_body::Body<Data = Bytes> + Send + 'static,
    B::Error: std::fmt::Display,
{
    /// The operations of `openapi` (an OpenAPI document as JSON) as tools, answered by `service`.
    ///
    /// Fails on an operationId that is no valid tool name (`^[A-Za-z0-9_-]{1,64}$`) or is used
    /// twice, on a [`Options::read_only`] entry that is no operationId, on parts of the document
    /// the facade can't map (header and cookie parameters, request bodies other than JSON,
    /// multipart and text, unresolvable `$ref`s), and when the artifact directory can't be made.
    pub fn new(service: S, openapi: &Value, options: Options) -> Result<Self, Error> {
        let (tools, operations): (Vec<_>, Vec<_>) = spec::operations(openapi, &options.read_only)?
            .into_iter()
            .unzip();
        let artifact_dir = response::artifact_dir(&options.name, options.artifact_dir.as_deref())?;
        let title = openapi
            .pointer("/info/title")
            .and_then(Value::as_str)
            .map(str::trim)
            .filter(|t| !t.is_empty());
        let mut implementation = Implementation::new(options.name, options.version);
        if let Some(title) = title {
            implementation = implementation.with_title(title);
        }
        let info = ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(implementation)
            .with_instructions(instructions(
                title,
                tools.len(),
                &artifact_dir,
                options.instructions.as_deref(),
            ));
        Ok(Self {
            inner: Arc::new(Inner {
                service,
                tools,
                operations,
                artifact_dir,
                info,
                turn: tokio::sync::Mutex::new(()),
            }),
        })
    }

    /// The tools, sorted by name.
    pub fn tools(&self) -> &[Tool] {
        &self.inner.tools
    }

    /// The directory binary responses are saved in (canonical).
    pub fn artifact_dir(&self) -> &Path {
        &self.inner.artifact_dir
    }

    /// Calls the tool `name` with `arguments` (an object, or null for none), as `tools/call` does.
    ///
    /// Only an unknown tool is an `Err` (JSON-RPC invalid params). Argument mistakes, HTTP errors
    /// and handler panics are error results (`is_error`), which the model can read and act on.
    pub async fn call(&self, name: &str, arguments: Value) -> Result<CallToolResult, ErrorData> {
        let Some(i) = self.inner.tools.iter().position(|t| t.name == name) else {
            return Err(ErrorData::invalid_params(
                format!("unknown tool: {name}"),
                None,
            ));
        };
        let arguments = match arguments {
            Value::Object(map) => map,
            Value::Null => serde_json::Map::new(),
            _ => {
                return Ok(response::failure(
                    "invalid_arguments",
                    "the arguments must be an object",
                ))
            }
        };
        let request = match request::build(&self.inner.operations[i], &arguments) {
            Ok(request) => request,
            Err(message) => return Ok(response::failure("invalid_arguments", message)),
        };
        let _turn = self.inner.turn.lock().await;
        log::debug!("{name}: {} {}", request.method(), request.uri());
        // A task of its own: a panicking handler fails this call, not the server.
        let service = self.inner.service.clone();
        let reply = tokio::spawn(async move {
            let Ok(response) = service.oneshot(request).await;
            let (parts, body) = response.into_parts();
            let body = body.collect().await.map_err(|e| e.to_string())?;
            Ok::<_, String>((parts, body.to_bytes()))
        })
        .await;
        Ok(match reply {
            Ok(Ok((parts, body))) => {
                response::to_result(name, &parts, &body, &self.inner.artifact_dir)
            }
            Ok(Err(e)) => response::failure(
                "internal_error",
                format!("reading the response failed: {e}"),
            ),
            Err(e) => {
                log::error!("{name}: {e}");
                response::failure("internal_error", format!("the handler failed: {e}"))
            }
        })
    }

    /// Serves MCP over stdin and stdout until stdin closes, and returns whether a client used the
    /// session, that is sent at least one request: `false` when stdin closed before any (stdin is
    /// `/dev/null`, say, or carried only lines that are no MCP message).
    pub async fn serve_stdio(self) -> Result<bool, Error> {
        let (stdin, stdout) = rmcp::transport::stdio();
        self.serve_io(stdin, stdout).await
    }

    /// [`Self::serve_stdio`] over `read` and `write`.
    async fn serve_io<R, W>(self, read: R, write: W) -> Result<bool, Error>
    where
        R: tokio::io::AsyncRead + Send + Unpin + 'static,
        W: tokio::io::AsyncWrite + Send + Unpin + 'static,
    {
        use rmcp::ServiceExt as _;
        let used = Arc::new(AtomicBool::new(false));
        let transport = server::Watched {
            inner: rmcp::transport::async_rw::AsyncRwTransport::new_server(read, write),
            used: used.clone(),
        };
        match self.serve(transport).await {
            Ok(running) => {
                running
                    .waiting()
                    .await
                    .map_err(|e| Error::Mcp(e.to_string()))?;
            }
            // The input ended before a session began (a `ping` or `server/discover` may have
            // been answered).
            Err(rmcp::service::ServerInitializeError::ConnectionClosed(_)) => {}
            Err(e) => return Err(Error::Mcp(e.to_string())),
        }
        Ok(used.load(Ordering::Relaxed))
    }
}

/// The server instructions: how the tools map to the API, then `extra`.
fn instructions(title: Option<&str>, tools: usize, dir: &Path, extra: Option<&str>) -> String {
    let api = title.map_or_else(|| "an HTTP API".to_string(), |t| format!("\"{t}\""));
    let mut text = format!(
        "Each of these {tools} tools is one operation of {api}, run in process (no network). \
         Arguments: the operation's path and query parameters and its request body's fields; a \
         body whose fields can't be top-level arguments is `body`, a YAML or text body the \
         string `yaml` or `text`, and file fields are base64. Results: JSON as structured \
         content (an array as `items`, a scalar as `value`), text as text, and files such as \
         zips saved under {} and returned as file:// resource links. An HTTP error status is an \
         error result with `http_status` and `body`.",
        dir.display()
    );
    if let Some(extra) = extra.map(str::trim).filter(|e| !e.is_empty()) {
        text.push_str("\n\n");
        text.push_str(extra);
    }
    text
}

#[cfg(test)]
pub(crate) mod testing {
    //! A small synthetic OpenAPI 3.0 document and a fake service that answers its routes.

    use std::convert::Infallible;

    use bytes::Bytes;
    use http::{Request, Response, StatusCode};
    use http_body_util::BodyExt as _;
    use serde_json::{json, Value};
    use tower::util::BoxCloneSyncService;

    use crate::Body;

    pub(crate) type Fake = BoxCloneSyncService<Request<Body>, Response<Body>, Infallible>;

    /// One operation of every kind the facade maps.
    pub(crate) fn spec() -> Value {
        json!({
            "openapi": "3.0.3",
            "info": {"title": "Test API", "version": "1"},
            "paths": {
                "/items": {
                    "get": {
                        "operationId": "list_items",
                        "summary": "List the items.",
                        "description": "Every item, by id.",
                        "responses": {"200": {"description": "The items"}}
                    }
                },
                "/items/{id}": {
                    "parameters": [{
                        "name": "id", "in": "path", "required": true,
                        "description": "The item id.", "schema": {"type": "string"}
                    }],
                    "get": {
                        "operationId": "get_item",
                        "summary": "Get one item.",
                        "parameters": [{
                            "name": "verbose", "in": "query",
                            "schema": {"type": "boolean", "default": false}
                        }, {
                            "name": "tag", "in": "query",
                            "schema": {"type": "array", "items": {"type": "string"}}
                        }],
                        "responses": {"200": {"description": "The item"}}
                    },
                    "post": {
                        "operationId": "rename_item",
                        "summary": "Rename an item.",
                        "requestBody": {
                            "required": true,
                            "content": {"application/json": {
                                "schema": {"$ref": "#/components/schemas/Rename"}
                            }}
                        },
                        "responses": {"200": {"description": "Renamed"}}
                    }
                },
                "/items/{id}/tags": {
                    "post": {
                        "operationId": "tag_item",
                        "parameters": [{"$ref": "#/components/parameters/Id"}],
                        "requestBody": {
                            "description": "The tag to add.",
                            "required": true,
                            "content": {"application/json": {
                                "schema": {"$ref": "#/components/schemas/Tag"}
                            }}
                        },
                        "responses": {"200": {"description": "Tagged"}}
                    }
                },
                "/validate": {
                    "post": {
                        "operationId": "validate_yaml",
                        "requestBody": {
                            "description": "The YAML text.",
                            "required": true,
                            "content": {"application/x-yaml": {"schema": {"type": "string"}}}
                        },
                        "responses": {"200": {"description": "The verdict"}}
                    }
                },
                "/upload": {
                    "post": {
                        "operationId": "upload_file",
                        "requestBody": {
                            "required": true,
                            "content": {"multipart/form-data": {
                                "schema": {"$ref": "#/components/schemas/UploadForm"}
                            }}
                        },
                        "responses": {"200": {"description": "Stored"}}
                    }
                },
                "/export": {
                    "post": {
                        "operationId": "export_zip",
                        "summary": "Export everything as a zip.",
                        "responses": {"200": {
                            "description": "The zip",
                            "content": {"application/zip": {
                                "schema": {"type": "string", "format": "binary"}
                            }}
                        }}
                    }
                },
                "/notes": {
                    "get": {"operationId": "get_notes", "responses": {"200": {"description": "Text"}}}
                },
                "/broken": {
                    "get": {"operationId": "broken", "responses": {"500": {"description": "Panics"}}}
                }
            },
            "components": {
                "parameters": {
                    "Id": {"name": "id", "in": "path", "required": true, "schema": {"type": "string"}}
                },
                "schemas": {
                    "Rename": {
                        "type": "object",
                        "required": ["name"],
                        "properties": {
                            "name": {"type": "string", "description": "The new name."},
                            "note": {"type": "string", "nullable": true},
                            "owner": {
                                "allOf": [{"$ref": "#/components/schemas/Owner"}],
                                "nullable": true
                            }
                        }
                    },
                    "Owner": {
                        "type": "object",
                        "required": ["login"],
                        "properties": {"login": {"type": "string"}}
                    },
                    "Tag": {
                        "type": "object",
                        "required": ["id"],
                        "properties": {"id": {"type": "string", "description": "The tag."}}
                    },
                    "UploadForm": {
                        "type": "object",
                        "required": ["filename", "file"],
                        "properties": {
                            "file": {"type": "string", "format": "binary", "description": "The file."},
                            "filename": {"type": "string"}
                        }
                    }
                }
            }
        })
    }

    /// The fake service for [`spec`].
    pub(crate) fn service() -> Fake {
        BoxCloneSyncService::new(tower::service_fn(fake))
    }

    /// Answers [`spec`]'s routes: `/export` with a zip, `/notes` with text, `/broken` by
    /// panicking, `/items/missing` with a 404 envelope, and everything else by echoing the
    /// request as JSON (`method`, `path`, `query`, `content_type`, `body`).
    pub(crate) async fn fake(request: Request<Body>) -> Result<Response<Body>, Infallible> {
        let (parts, body) = request.into_parts();
        let body = body.collect().await.unwrap().to_bytes();
        let path = parts.uri.path().to_string();
        let reply = |status: StatusCode, content_type: &str, body: Bytes| {
            Response::builder()
                .status(status)
                .header(http::header::CONTENT_TYPE, content_type)
                .body(Body::new(body))
                .unwrap()
        };
        Ok(match path.as_str() {
            "/export" => {
                let mut zip = reply(StatusCode::OK, "application/zip", zip());
                zip.headers_mut().insert(
                    http::header::CONTENT_DISPOSITION,
                    "attachment; filename=\"everything.zip\"".parse().unwrap(),
                );
                zip
            }
            "/notes" => reply(StatusCode::OK, "text/plain; charset=utf-8", "a: 1\n".into()),
            "/broken" => panic!("the handler broke"),
            "/items/missing" => reply(
                StatusCode::NOT_FOUND,
                "application/json",
                r#"{"error":{"code":"unknown_item","message":"no item missing"}}"#.into(),
            ),
            _ => {
                let echo = json!({
                    "method": parts.method.as_str(),
                    "path": path,
                    "query": parts.uri.query(),
                    "content_type": parts
                        .headers
                        .get(http::header::CONTENT_TYPE)
                        .map(|v| v.to_str().unwrap()),
                    "body": String::from_utf8_lossy(&body),
                });
                reply(StatusCode::OK, "application/json", echo.to_string().into())
            }
        })
    }

    /// The 22 bytes of an empty zip archive.
    pub(crate) fn zip() -> Bytes {
        let mut eocd = b"PK\x05\x06".to_vec();
        eocd.extend([0; 18]);
        eocd.into()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    use serde_json::json;

    use super::testing::{fake, service, spec, Fake};
    use super::*;

    fn facade() -> (Facade<Fake>, tempfile::TempDir) {
        let dir = tempfile::tempdir().unwrap();
        let mut options = Options::new("test-api", "1.0.0");
        options.artifact_dir = Some(dir.path().join("artifacts"));
        (Facade::new(service(), &spec(), options).unwrap(), dir)
    }

    #[tokio::test]
    async fn a_call_is_answered_in_process() {
        let (f, _dir) = facade();
        let r = f
            .call("get_item", json!({"id": "a b", "verbose": true}))
            .await
            .unwrap();
        assert_eq!(r.is_error, Some(false));
        let echo = r.structured_content.unwrap();
        assert_eq!(echo["method"], "GET");
        assert_eq!(echo["path"], "/items/a%20b");
        assert_eq!(echo["query"], "verbose=true");
        let r = f
            .call("validate_yaml", json!({"yaml": "a: 1\n"}))
            .await
            .unwrap();
        let echo = r.structured_content.unwrap();
        assert_eq!(echo["content_type"], "application/x-yaml");
        assert_eq!(echo["body"], "a: 1\n");
    }

    #[tokio::test]
    async fn a_zip_lands_in_the_artifact_dir() {
        let (f, _dir) = facade();
        let r = f.call("export_zip", Value::Null).await.unwrap();
        let path = PathBuf::from(r.structured_content.unwrap()["path"].as_str().unwrap());
        assert_eq!(path.parent().unwrap(), f.artifact_dir());
        assert!(std::fs::read(&path).unwrap().starts_with(b"PK"));
    }

    #[tokio::test]
    async fn argument_mistakes_are_error_results() {
        let (f, _dir) = facade();
        for args in [
            json!({"id": "1", "colour": "red"}),
            json!({}),
            json!({"id": ".."}),
            json!([1]),
        ] {
            let r = f.call("get_item", args.clone()).await.unwrap();
            assert_eq!(r.is_error, Some(true), "{args}");
            let s = r.structured_content.unwrap();
            assert_eq!(s["code"], "invalid_arguments", "{args}");
        }
        // Only an unknown tool is a protocol error.
        assert!(f.call("no_such_tool", json!({})).await.is_err());
    }

    #[tokio::test]
    async fn an_http_error_is_an_error_result() {
        let (f, _dir) = facade();
        let r = f.call("get_item", json!({"id": "missing"})).await.unwrap();
        assert_eq!(r.is_error, Some(true));
        let s = r.structured_content.unwrap();
        assert_eq!(s["http_status"], 404);
        assert_eq!(s["body"]["error"]["code"], "unknown_item");
    }

    #[tokio::test]
    async fn a_handler_panic_is_an_error_result() {
        let (f, _dir) = facade();
        let r = f.call("broken", Value::Null).await.unwrap();
        assert_eq!(r.is_error, Some(true));
        assert_eq!(r.structured_content.unwrap()["code"], "internal_error");
        // The facade still works.
        let r = f.call("list_items", Value::Null).await.unwrap();
        assert_eq!(r.is_error, Some(false));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn calls_run_one_at_a_time() {
        let running = Arc::new(AtomicUsize::new(0));
        let most = Arc::new(AtomicUsize::new(0));
        let (r, m) = (running.clone(), most.clone());
        let slow = tower::service_fn(move |request: http::Request<Body>| {
            let (running, most) = (r.clone(), m.clone());
            async move {
                let now = running.fetch_add(1, Ordering::SeqCst) + 1;
                most.fetch_max(now, Ordering::SeqCst);
                tokio::time::sleep(Duration::from_millis(20)).await;
                running.fetch_sub(1, Ordering::SeqCst);
                fake(request).await
            }
        });
        let dir = tempfile::tempdir().unwrap();
        let mut options = Options::new("test-api", "1.0.0");
        options.artifact_dir = Some(dir.path().to_path_buf());
        let f = Facade::new(slow, &spec(), options).unwrap();
        let calls: Vec<_> = (0..4)
            .map(|_| {
                let f = f.clone();
                tokio::spawn(async move { f.call("list_items", Value::Null).await })
            })
            .collect();
        for call in calls {
            assert_eq!(call.await.unwrap().unwrap().is_error, Some(false));
        }
        assert_eq!(most.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn the_server_describes_itself() {
        let (f, _dir) = facade();
        let info = &f.inner.info;
        assert_eq!(info.server_info.name, "test-api");
        assert_eq!(info.server_info.version, "1.0.0");
        assert_eq!(info.server_info.title.as_deref(), Some("Test API"));
        assert!(info.capabilities.tools.is_some());
        let text = info.instructions.as_deref().unwrap();
        assert!(text.contains("these 9 tools"), "{text}");
        assert!(
            text.contains(&f.artifact_dir().display().to_string()),
            "{text}"
        );
    }

    /// Serves `input`, then the end of input; was the session used?
    async fn used(input: &str) -> bool {
        use tokio::io::AsyncWriteExt as _;
        let (f, _dir) = facade();
        let (mut client, from_client) = tokio::io::duplex(1 << 16);
        let (to_client, _replies) = tokio::io::duplex(1 << 16);
        client.write_all(input.as_bytes()).await.unwrap();
        drop(client);
        f.serve_io(from_client, to_client).await.unwrap()
    }

    #[tokio::test]
    async fn a_session_is_used_once_a_request_arrives() {
        assert!(!used("").await);
        // Not JSON, a blank line, and JSON that is no MCP message.
        assert!(!used("not json\n\n{\"a\": 1}\n").await);
        assert!(used("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"ping\"}\n").await);
    }

    #[tokio::test]
    async fn a_client_s_session_is_used() {
        use rmcp::ServiceExt as _;
        let (f, _dir) = facade();
        let (server_io, client_io) = tokio::io::duplex(1 << 16);
        let (read, write) = tokio::io::split(server_io);
        let server = tokio::spawn(f.serve_io(read, write));
        let client = ().serve(client_io).await.unwrap();
        client.cancel().await.unwrap();
        assert!(server.await.unwrap().unwrap());
    }

    #[test]
    fn read_only_must_name_operations() {
        let mut options = Options::new("test-api", "1.0.0");
        options.read_only = vec!["no_such_operation".into()];
        let err = Facade::new(service(), &spec(), options).err().unwrap();
        assert!(err.to_string().contains("no_such_operation"), "{err}");
    }

    #[cfg(unix)]
    #[test]
    fn the_default_artifact_dir_is_a_new_private_one() {
        use std::os::unix::fs::PermissionsExt as _;
        let f = Facade::new(service(), &spec(), Options::new("test-api", "1")).unwrap();
        let dir = f.artifact_dir().to_path_buf();
        let mode = std::fs::metadata(&dir).unwrap().permissions().mode();
        std::fs::remove_dir(&dir).unwrap();
        assert_eq!(mode & 0o777, 0o700);
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        assert!(name.starts_with("test-api-mcp-"), "{name}");
    }
}
