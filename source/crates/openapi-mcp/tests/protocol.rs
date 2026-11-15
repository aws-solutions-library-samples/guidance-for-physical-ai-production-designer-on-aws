//! The facade over the MCP protocol: rmcp's client lists and calls its tools through an in-memory
//! duplex stream, which carries the same newline-delimited JSON-RPC as stdio.

use std::convert::Infallible;

use http_body_util::BodyExt as _;
use openapi_mcp::rmcp::model::CallToolRequestParams;
use openapi_mcp::rmcp::ServiceExt as _;
use openapi_mcp::{Body, Facade, Options};
use serde_json::{json, Value};

fn spec() -> Value {
    json!({
        "openapi": "3.0.3",
        "info": {"title": "Echo API", "version": "1"},
        "paths": {"/echo/{word}": {"post": {
            "operationId": "echo",
            "summary": "Repeat a word.",
            "parameters": [{
                "name": "word", "in": "path", "required": true, "schema": {"type": "string"}
            }],
            "requestBody": {"required": true, "content": {"application/json": {"schema": {
                "type": "object",
                "properties": {"times": {"type": "integer", "format": "int32"}}
            }}}},
            "responses": {"200": {"description": "The words"}}
        }}}
    })
}

/// `POST /echo/{word}` with `{times}`: `{"words": [word, …]}`.
async fn echo(request: http::Request<Body>) -> Result<http::Response<Body>, Infallible> {
    let word = request
        .uri()
        .path()
        .trim_start_matches("/echo/")
        .to_string();
    let body = request.into_body().collect().await.unwrap().to_bytes();
    let times = serde_json::from_slice::<Value>(&body).unwrap()["times"]
        .as_u64()
        .unwrap_or(1);
    let words = json!({"words": vec![word; times as usize]});
    Ok(http::Response::builder()
        .header(http::header::CONTENT_TYPE, "application/json")
        .body(Body::from(words.to_string()))
        .unwrap())
}

fn call(name: &'static str, arguments: Value) -> CallToolRequestParams {
    let Value::Object(arguments) = arguments else {
        panic!("arguments must be an object")
    };
    CallToolRequestParams::new(name).with_arguments(arguments)
}

#[tokio::test]
async fn tools_are_listed_and_called_over_the_protocol() {
    let artifacts = tempfile::tempdir().unwrap();
    let mut options = Options::new("echo-server", "1.0.0");
    options.artifact_dir = Some(artifacts.path().to_path_buf());
    let facade = Facade::new(tower::service_fn(echo), &spec(), options).unwrap();

    let (server_io, client_io) = tokio::io::duplex(1 << 16);
    let server = tokio::spawn(async move {
        let running = facade.serve(server_io).await.unwrap();
        running.waiting().await.unwrap();
    });
    let client = ().serve(client_io).await.unwrap();
    let info = client.peer_info().unwrap();
    let server_info = info.server_info.as_ref().unwrap();
    assert_eq!(server_info.name, "echo-server");
    assert!(info.capabilities.tools.is_some());

    let tools = client.list_all_tools().await.unwrap();
    assert_eq!(tools.len(), 1);
    assert_eq!(tools[0].name, "echo");
    assert_eq!(tools[0].title.as_deref(), Some("Repeat a word."));
    assert_eq!(
        tools[0].schema_as_json_value(),
        json!({
            "type": "object",
            "properties": {"word": {"type": "string"}, "times": {"type": "integer"}},
            "required": ["word"],
            "additionalProperties": false
        })
    );

    let r = client
        .call_tool(call("echo", json!({"word": "hi", "times": 2})))
        .await
        .unwrap();
    assert_eq!(r.is_error, Some(false));
    assert_eq!(r.structured_content, Some(json!({"words": ["hi", "hi"]})));

    // A missing argument is an error result; an unknown tool is a protocol error.
    let r = client
        .call_tool(call("echo", json!({"times": 2})))
        .await
        .unwrap();
    assert_eq!(r.is_error, Some(true));
    assert_eq!(r.structured_content.unwrap()["code"], "invalid_arguments");
    assert!(client
        .call_tool(CallToolRequestParams::new("no_such_tool"))
        .await
        .is_err());

    client.cancel().await.unwrap();
    server.await.unwrap();
}
