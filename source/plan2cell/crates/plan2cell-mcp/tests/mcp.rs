//! The MCP server end to end: an rmcp client talks to it in process (duplex stream, the same
//! protocol stdio carries) and over streamable HTTP, including the bearer token.

use std::path::{Path, PathBuf};

use rmcp::model::{CallToolRequestParams, CallToolResult};
use rmcp::service::RunningService;
use rmcp::{RoleClient, ServiceExt};
use serde_json::{Value, json};

use plan2cell_mcp::{HttpOptions, Plan2cellServer, Settings};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A root with the welding plan and the example config, writing allowed.
fn root() -> tempfile::TempDir {
    let d = tempfile::tempdir().unwrap();
    std::fs::copy(
        repo().join("test_data/bt_1790934549_welding_cell_2_small.json"),
        d.path().join("welding.json"),
    )
    .unwrap();
    std::fs::copy(
        repo().join("examples/plan2cell.toml"),
        d.path().join("plan2cell.toml"),
    )
    .unwrap();
    d
}

fn settings(root: &Path, allow_write: bool) -> Settings {
    Settings {
        root: root.to_path_buf(),
        config: Some("plan2cell.toml".into()),
        allow_write,
    }
}

async fn in_process(s: Settings) -> RunningService<RoleClient, ()> {
    let (server_io, client_io) = tokio::io::duplex(1 << 20);
    let server = Plan2cellServer::new(s).unwrap();
    tokio::spawn(async move {
        if let Ok(running) = server.serve(server_io).await {
            let _ = running.waiting().await;
        }
    });
    ().serve(client_io).await.unwrap()
}

async fn call(c: &RunningService<RoleClient, ()>, tool: &str, args: Value) -> CallToolResult {
    let Value::Object(map) = args else {
        panic!("arguments must be an object")
    };
    c.call_tool(CallToolRequestParams::new(tool.to_string()).with_arguments(map))
        .await
        .unwrap()
}

fn text(r: &CallToolResult) -> String {
    r.content
        .iter()
        .filter_map(|c| c.as_text().map(|t| t.text.clone()))
        .collect::<Vec<_>>()
        .join("")
}

fn json_of(r: &CallToolResult) -> Value {
    serde_json::from_str(&text(r)).unwrap_or_else(|e| panic!("not JSON ({e}): {}", text(r)))
}

fn is_error(r: &CallToolResult) -> bool {
    r.is_error == Some(true)
}

#[tokio::test]
async fn the_tools_are_listed_with_schemas() {
    let d = root();
    let c = in_process(settings(d.path(), false)).await;
    let tools = c.list_all_tools().await.unwrap();
    let names: Vec<String> = tools.iter().map(|t| t.name.to_string()).collect();
    for want in [
        "plan2cell_zones",
        "plan2cell_check",
        "plan2cell_build",
        "plan2cell_explain_robot",
        "plan2cell_default_config",
        "plan2cell_list_files",
        "plan2cell_read_file",
    ] {
        assert!(
            names.iter().any(|n| n == want),
            "{want} missing in {names:?}"
        );
    }
    let build = tools.iter().find(|t| t.name == "plan2cell_build").unwrap();
    let schema = serde_json::to_value(&build.input_schema).unwrap();
    for p in [
        "plan_path",
        "plan_text",
        "profile",
        "krl",
        "output_dir",
        "include_files",
    ] {
        assert!(
            schema["properties"].get(p).is_some(),
            "{p} not in the build schema"
        );
    }
    c.cancel().await.unwrap();
}

#[tokio::test]
async fn zones_check_build_and_read_back() {
    let d = root();
    let c = in_process(settings(d.path(), true)).await;

    let z = json_of(&call(&c, "plan2cell_zones", json!({"plan_path": "welding.json"})).await);
    assert_eq!(z["zones"][0]["name"], "IL01");
    assert_eq!(z["zones"][0]["tickets"][0], "robot1.W01");
    assert_eq!(z["robots"].as_array().unwrap().len(), 2);

    let chk = json_of(
        &call(
            &c,
            "plan2cell_check",
            json!({"plan_path": "welding.json", "st_check": false}),
        )
        .await,
    );
    assert_eq!(chk["ok"], true);
    assert_eq!(chk["summary"]["robots"][0]["folgen"], json!([11, 125]));

    let b = call(
        &c,
        "plan2cell_build",
        json!({
            "plan_path": "welding.json",
            "st_check": false,
            "output_dir": "out/welding",
            "include_files": ["krl/robot1/KRC/R1/Folgen/folge11.src"],
        }),
    )
    .await;
    assert!(!is_error(&b), "{}", text(&b));
    let b = json_of(&b);
    assert_eq!(b["written_to"], "out/welding");
    let src = b["contents"]["krl/robot1/KRC/R1/Folgen/folge11.src"]
        .as_str()
        .unwrap();
    assert!(src.contains("DEF FOLGE11()"));
    assert!(
        d.path()
            .join("out/welding/plc/codesys/head/GVL/GVL_Cell.st")
            .is_file()
    );

    let r = call(
        &c,
        "plan2cell_read_file",
        json!({"path": "out/welding/krl/robot1/INSTALL.md"}),
    )
    .await;
    assert!(text(&r).contains("Never install this on a VW KS24 robot"));

    let l = json_of(&call(&c, "plan2cell_list_files", json!({"dir": "out/welding"})).await);
    let paths: Vec<&str> = l["entries"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| e["path"].as_str().unwrap())
        .collect();
    assert!(paths.contains(&"out/welding/BUILD.md"), "{paths:?}");

    let x = call(
        &c,
        "plan2cell_explain_robot",
        json!({"plan_path": "welding.json", "robot": "robot2"}),
    )
    .await;
    assert!(text(&x).contains("op   1 W13"), "{}", text(&x));

    // A second build into the same directory needs overwrite.
    let again = call(
        &c,
        "plan2cell_build",
        json!({"plan_path": "welding.json", "st_check": false, "output_dir": "out/welding"}),
    )
    .await;
    assert!(is_error(&again));
    assert_eq!(json_of(&again)["exit_code"], 4);
    c.cancel().await.unwrap();
}

#[tokio::test]
async fn inline_plans_and_configs_work_without_files() {
    let d = root();
    let c = in_process(Settings {
        root: d.path().to_path_buf(),
        config: None,
        allow_write: false,
    })
    .await;
    let plan =
        std::fs::read_to_string(repo().join("test_data/bt_1791274820_cell_2robot.json")).unwrap();
    let cfg = "[[krl.robot]]\nns = \"robot1\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n";
    let r = call(
        &c,
        "plan2cell_check",
        json!({"plan_text": plan, "plan_name": "cell_2robot.json", "config_toml": cfg, "st_check": false}),
    )
    .await;
    assert!(!is_error(&r), "{}", text(&r));
    // Without the config, KRL needs e_axes: a tool error with bt2krl's exit code, not a crash.
    let r = call(
        &c,
        "plan2cell_check",
        json!({"plan_text": plan, "st_check": false}),
    )
    .await;
    assert!(is_error(&r));
    assert_eq!(json_of(&r)["exit_code"], 3);
    // ... unless no KRL is asked for.
    let r = call(
        &c,
        "plan2cell_check",
        json!({"plan_text": plan, "st_check": false, "krl": false}),
    )
    .await;
    assert!(!is_error(&r), "{}", text(&r));
    let t = call(&c, "plan2cell_default_config", json!({})).await;
    assert!(text(&t).contains("[krl.motion]"));
    c.cancel().await.unwrap();
}

#[tokio::test]
async fn files_stay_inside_the_root_and_writing_is_opt_in() {
    let d = root();
    let c = in_process(settings(d.path(), false)).await;
    for (tool, args) in [
        ("plan2cell_read_file", json!({"path": "/etc/passwd"})),
        ("plan2cell_read_file", json!({"path": "../../etc/passwd"})),
        ("plan2cell_zones", json!({"plan_path": "../welding.json"})),
        ("plan2cell_list_files", json!({"dir": ".."})),
    ] {
        let r = call(&c, tool, args.clone()).await;
        assert!(is_error(&r), "{tool} {args}: {}", text(&r));
    }
    let r = call(
        &c,
        "plan2cell_build",
        json!({"plan_path": "welding.json", "st_check": false, "output_dir": "out"}),
    )
    .await;
    assert!(is_error(&r));
    assert!(text(&r).contains("does not allow writing"));
    assert!(!d.path().join("out").exists());
    // Building in memory is always fine.
    let r = call(
        &c,
        "plan2cell_build",
        json!({"plan_path": "welding.json", "st_check": false}),
    )
    .await;
    assert!(!is_error(&r), "{}", text(&r));
    c.cancel().await.unwrap();
}

#[tokio::test]
async fn streamable_http_with_a_bearer_token() {
    use rmcp::transport::StreamableHttpClientTransport;
    use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;

    let d = root();
    let token = "0123456789abcdef-test-token";
    let server = plan2cell_mcp::serve_http(
        settings(d.path(), false),
        HttpOptions {
            addr: "127.0.0.1:0".parse().unwrap(),
            token: Some(token.into()),
            allowed_hosts: vec![],
        },
    )
    .await
    .unwrap();
    let base = format!("http://{}", server.local_addr);

    // Health without a token; /mcp without one is 401.
    let http = reqwest::Client::new();
    assert_eq!(
        http.get(format!("{base}/healthz"))
            .send()
            .await
            .unwrap()
            .status(),
        200
    );
    let init = json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "t", "version": "1"}}});
    let r = http
        .post(format!("{base}/mcp"))
        .header("Accept", "application/json, text/event-stream")
        .json(&init)
        .send()
        .await
        .unwrap();
    assert_eq!(r.status(), 401);
    let r = http
        .post(format!("{base}/mcp"))
        .bearer_auth("wrong-token-0123456789")
        .header("Accept", "application/json, text/event-stream")
        .json(&init)
        .send()
        .await
        .unwrap();
    assert_eq!(r.status(), 401);

    // A real MCP client with the token.
    let transport = StreamableHttpClientTransport::with_client(
        reqwest::Client::new(),
        StreamableHttpClientTransportConfig::with_uri(format!("{base}/mcp")).auth_header(token),
    );
    let c = ().serve(transport).await.unwrap();
    let tools = c.list_all_tools().await.unwrap();
    assert!(tools.iter().any(|t| t.name == "plan2cell_build"));
    let z = json_of(&call(&c, "plan2cell_zones", json!({"plan_path": "welding.json"})).await);
    assert_eq!(z["zones"].as_array().unwrap().len(), 2);
    c.cancel().await.unwrap();
    server.shutdown().await;
}

#[tokio::test]
async fn http_refuses_an_open_network_listener() {
    let d = root();
    let r = plan2cell_mcp::serve_http(
        settings(d.path(), false),
        HttpOptions {
            addr: "0.0.0.0:0".parse().unwrap(),
            token: None,
            allowed_hosts: vec![],
        },
    )
    .await;
    assert!(r.is_err());
}
