//! `factoryos-mcp` spoken to as kiro-cli does (JSON-RPC lines on stdio), against a fake backend:
//! the tools are listed with their hints, a call becomes the backend request (a cell-runner verb
//! wrapped into `cell/command`), and HTTP errors are error results.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};

use axum::Json;
use axum::extract::Path;
use axum::routing::{get, post};
use serde_json::{Value, json};

type Seen = Arc<Mutex<Vec<(String, Value)>>>;

/// A fake backend on a free loopback port, recording what it was sent.
fn backend() -> (String, Seen) {
    let seen: Seen = Arc::default();
    let (s1, s2, s3, s4) = (seen.clone(), seen.clone(), seen.clone(), seen.clone());
    let app = axum::Router::new()
        .route("/api/v1/product/list", get(|| async { Json(json!({"dir": "/p", "products": ["cube1"]})) }))
        .route(
            "/api/v1/product/{name}/welds/add",
            post(move |Path(name): Path<String>, Json(body): Json<Value>| {
                let seen = s1.clone();
                async move {
                    seen.lock().unwrap().push((format!("welds/add {name}"), body.clone()));
                    Json(json!({"changed": ["W01"], "issues": [], "path": "/p/cube1/product.json", "product": {"name": name}}))
                }
            }),
        )
        .route(
            "/api/v1/cell/command",
            post(move |Json(body): Json<Value>| {
                let seen = s2.clone();
                async move {
                    seen.lock().unwrap().push(("cell/command".into(), body.clone()));
                    Json(json!({"ok": true, "result": {"playing": false}}))
                }
            }),
        )
        .route(
            "/api/v1/infra/deploy",
            post(move |Json(body): Json<Value>| {
                let seen = s3.clone();
                async move {
                    seen.lock().unwrap().push(("deploy".into(), body));
                    (axum::http::StatusCode::CONFLICT, "a rule fails: run vcell_deploy_check")
                }
            }),
        )
        .route(
            "/api/v1/infra/kvc",
            post(move |Json(body): Json<Value>| {
                let seen = s4.clone();
                async move {
                    seen.lock().unwrap().push(("kvc".into(), body));
                    Json(json!({"ok": true}))
                }
            }),
        );
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    listener.set_nonblocking(true).unwrap();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async move {
            let listener = tokio::net::TcpListener::from_std(listener).unwrap();
            axum::serve(listener, app).await.unwrap();
        });
    });
    (format!("http://{addr}"), seen)
}

struct Client {
    child: std::process::Child,
    out: BufReader<std::process::ChildStdout>,
    id: u64,
}

impl Client {
    fn start(backend: &str) -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_factoryos-mcp"))
            .args(["--backend", backend])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap();
        let out = BufReader::new(child.stdout.take().unwrap());
        let mut c = Self { child, out, id: 0 };
        let init = c.request("initialize", json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}));
        assert_eq!(init["result"]["serverInfo"]["name"], "factoryos-mcp", "{init}");
        assert!(init["result"]["instructions"].as_str().unwrap().contains("Product & Task"));
        c.send(json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
        c
    }

    fn send(&mut self, message: Value) {
        let stdin = self.child.stdin.as_mut().unwrap();
        writeln!(stdin, "{message}").unwrap();
        stdin.flush().unwrap();
    }

    fn request(&mut self, method: &str, params: Value) -> Value {
        self.id += 1;
        let id = self.id;
        self.send(json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}));
        loop {
            let mut line = String::new();
            assert!(self.out.read_line(&mut line).unwrap() > 0, "the server closed stdout");
            let v: Value = serde_json::from_str(&line).unwrap();
            if v["id"] == id {
                return v;
            }
        }
    }

    fn call(&mut self, name: &str, arguments: Value) -> Value {
        self.request("tools/call", json!({"name": name, "arguments": arguments}))["result"].clone()
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        drop(self.child.stdin.take());
        let _ = self.child.wait();
    }
}

#[test]
fn tools_calls_and_errors() {
    let (url, seen) = backend();
    let mut c = Client::start(&url);

    let list = c.request("tools/list", json!({}));
    let tools = list["result"]["tools"].as_array().unwrap();
    assert!(tools.len() > 80, "{} tools", tools.len());
    let tool = |name: &str| tools.iter().find(|t| t["name"] == name).unwrap_or_else(|| panic!("no tool {name}")).clone();
    assert_eq!(tool("product_list")["annotations"]["readOnlyHint"], true);
    assert_eq!(tool("product_add_welds")["annotations"]["readOnlyHint"], false);
    assert_eq!(tool("vcell_deploy")["annotations"]["destructiveHint"], true);
    assert_eq!(tool("vcell_kvc_read")["annotations"]["readOnlyHint"], true);
    assert!(tool("product_add_welds")["inputSchema"]["properties"]["welds"].is_object());
    assert_eq!(tool("vcell_list_templates")["annotations"]["readOnlyHint"], true);
    assert_eq!(tool("vcell_kvc")["annotations"]["destructiveHint"], true);
    let create = tool("vcell_create_cell")["inputSchema"]["properties"].clone();
    assert!(create["template"].is_object() && create["params"].is_object() && create["robots"].is_object(), "{create}");
    // The backend checks a robot op against the cell's template: the tools take any op name.
    for name in ["vcell_kvc", "vcell_kvc_read"] {
        let props = tool(name)["inputSchema"]["properties"].clone();
        assert!(props["op"].get("enum").is_none() && props["arg"].get("enum").is_none(), "{name}: {props}");
    }

    let r = c.call("product_list", json!({}));
    assert_eq!(r["structuredContent"]["products"][0], "cube1", "{r}");

    let r = c.call("product_add_welds", json!({"name": "cube1", "welds": [{"position": [0.0, 0.0, 0.3]}]}));
    assert_eq!(r["structuredContent"]["changed"][0], "W01", "{r}");

    let r = c.call("bt_step", json!({"direction": "back"}));
    assert_eq!(r["isError"], false, "{r}");
    let r = c.call("bt_play", json!({}));
    assert!(r.get("isError").is_none_or(|e| e == false), "{r}");

    let r = c.call("vcell_deploy", json!({"build": "b1"}));
    assert_eq!(r["isError"], true, "{r}");
    assert!(r.to_string().contains("409"), "{r}");

    let r = c.call("vcell_kvc", json!({"robot": "robot1", "op": "jog", "arg": "a1+"}));
    assert_eq!(r["isError"], false, "{r}");

    let seen = seen.lock().unwrap();
    assert_eq!(seen[0], ("welds/add cube1".to_string(), json!({"welds": [{"position": [0.0, 0.0, 0.3]}]})));
    assert_eq!(seen[1], ("cell/command".to_string(), json!({"type": "trajectory_step", "params": {"direction": "back"}})));
    assert_eq!(seen[2], ("cell/command".to_string(), json!({"type": "trajectory_play"})));
    assert_eq!(seen[3].0, "deploy");
    assert_eq!(seen[4], ("kvc".to_string(), json!({"robot": "robot1", "op": "jog", "arg": "a1+"})));
}

#[test]
fn an_unreachable_backend_is_an_error_result() {
    let mut c = Client::start("http://127.0.0.1:9");
    let r = c.call("product_list", json!({}));
    assert_eq!(r["isError"], true, "{r}");
    assert!(r.to_string().contains("not reachable"), "{r}");
}

#[test]
fn every_tool_path_is_a_backend_route() {
    // The tool files only name routes the protocol crate declares (a typo would be a 404 at run time).
    let out = Command::new(env!("CARGO_BIN_EXE_factoryos-mcp")).arg("--openapi").output().unwrap();
    let doc: Value = serde_json::from_slice(&out.stdout).unwrap();
    let protocol = include_str!("../../factoryos-protocol/src/lib.rs");
    for path in doc["paths"].as_object().unwrap().keys() {
        if path.starts_with("/mcp/cell-verb/") {
            continue;
        }
        let route = path.replace("{path}", "{*path}");
        assert!(protocol.contains(&format!("\"{route}\"")), "{path} is no route of factoryos-protocol's api");
    }
}

/// Every tool's input schema is a JSON Schema the model APIs accept (draft 2020-12): every
/// schema an object or a boolean, known types, `properties` of schemas, `required` names.
#[test]
fn every_input_schema_is_a_valid_json_schema() {
    fn check(path: &str, s: &Value, problems: &mut Vec<String>) {
        let Some(o) = s.as_object() else {
            if !s.is_boolean() {
                problems.push(format!("{path}: {s} is no schema"));
            }
            return;
        };
        let types = ["object", "array", "string", "number", "integer", "boolean", "null"];
        match o.get("type") {
            None => {}
            Some(Value::String(t)) if types.contains(&t.as_str()) => {}
            Some(Value::Array(ts)) if ts.iter().all(|t| t.as_str().is_some_and(|t| types.contains(&t))) => {}
            Some(t) => problems.push(format!("{path}: type {t}")),
        }
        if let Some(props) = o.get("properties") {
            match props.as_object() {
                Some(props) => props.iter().for_each(|(k, v)| check(&format!("{path}.{k}"), v, problems)),
                None => problems.push(format!("{path}: properties is no object")),
            }
        }
        if let Some(items) = o.get("items") {
            check(&format!("{path}[]"), items, problems);
        }
        if let Some(r) = o.get("required") {
            if !r.as_array().is_some_and(|r| r.iter().all(Value::is_string)) {
                problems.push(format!("{path}: required {r}"));
            }
        }
        if let Some(e) = o.get("enum") {
            if !e.is_array() {
                problems.push(format!("{path}: enum {e}"));
            }
        }
        for key in ["description", "title"] {
            if o.get(key).is_some_and(|d| !d.is_string()) {
                problems.push(format!("{path}: {key} is no string"));
            }
        }
    }
    let (url, _) = backend();
    let mut c = Client::start(&url);
    let list = c.request("tools/list", json!({}));
    let mut problems = Vec::new();
    for tool in list["result"]["tools"].as_array().unwrap() {
        check(tool["name"].as_str().unwrap(), &tool["inputSchema"], &mut problems);
    }
    assert!(problems.is_empty(), "{problems:#?}");
}
