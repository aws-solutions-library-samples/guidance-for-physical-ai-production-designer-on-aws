//! The curated tools (`tools/*.yaml`) as the OpenAPI document openapi-mcp serves.
//!
//! A tool is one backend route, written compactly:
//!
//! ```yaml
//! - name: product_add_welds          # the MCP tool name (unique, ^[A-Za-z0-9_-]{1,64}$)
//!   title: Add spot welds             # optional
//!   method: POST                      # default POST
//!   path: /api/v1/product/{name}/welds/add
//!   params: {name: "The product."}    # descriptions of the path parameters (strings)
//!   query: {plan: {type: string, description: …}}   # query parameters (schemas; `required: true`)
//!   body: {type: object, properties: …, required: […]}  # the JSON body's schema
//!   read_only: true                   # a POST that only reads (GET always does)
//!   destructive: true                 # hard to undo, or acts on the virtual cell: always asks
//!   verb: step                        # a cell-runner verb: POST /api/v1/cell/command
//!                                     #   {"type": verb, "params": <body>} (path is ignored)
//!   description: |                    # what the model reads
//!     …
//! ```

use serde::Deserialize;
use serde_json::{Map, Value, json};

/// The tool files, in the order of the workflow.
pub const FILES: &[(&str, &str)] = &[
    ("product", include_str!("../tools/product.yaml")),
    ("dresser", include_str!("../tools/dresser.yaml")),
    ("scene", include_str!("../tools/scene.yaml")),
    ("planning", include_str!("../tools/planning.yaml")),
    ("bt_view", include_str!("../tools/bt_view.yaml")),
    ("codegen", include_str!("../tools/codegen.yaml")),
    ("vcell", include_str!("../tools/vcell.yaml")),
];

/// The prefix of the virtual paths of cell-runner verbs, which the proxy turns into
/// `POST /api/v1/cell/command`.
pub const VERB_PREFIX: &str = "/mcp/cell-verb/";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolDef {
    pub name: String,
    #[serde(default)]
    pub title: Option<String>,
    #[serde(default = "post")]
    pub method: String,
    #[serde(default)]
    pub path: String,
    #[serde(default)]
    pub params: Map<String, Value>,
    #[serde(default)]
    pub query: Map<String, Value>,
    #[serde(default)]
    pub body: Option<Value>,
    #[serde(default)]
    pub read_only: bool,
    #[serde(default)]
    pub destructive: bool,
    #[serde(default)]
    pub verb: Option<String>,
    pub description: String,
}

fn post() -> String {
    "POST".to_owned()
}

/// Every tool definition of [`FILES`].
pub fn tools() -> Result<Vec<ToolDef>, String> {
    let mut all = Vec::new();
    for (file, text) in FILES {
        let tools: Vec<ToolDef> = serde_yaml::from_str(text).map_err(|e| format!("tools/{file}.yaml: {e}"))?;
        all.extend(tools);
    }
    let mut names = std::collections::BTreeSet::new();
    for tool in &all {
        if !names.insert(tool.name.as_str()) {
            return Err(format!("tool {} is defined twice", tool.name));
        }
    }
    Ok(all)
}

/// The OpenAPI 3.1 document of `tools`.
pub fn openapi(tools: &[ToolDef]) -> Result<Value, String> {
    let mut paths = Map::new();
    for tool in tools {
        let (method, path) = match &tool.verb {
            Some(verb) => ("post".to_owned(), format!("{VERB_PREFIX}{verb}")),
            None => (tool.method.to_ascii_lowercase(), tool.path.clone()),
        };
        if path.is_empty() {
            return Err(format!("{}: no path", tool.name));
        }
        let mut parameters = Vec::new();
        for name in path_params(&path) {
            let description = tool.params.get(&name).and_then(Value::as_str).unwrap_or_default();
            parameters.push(json!({"name": name, "in": "path", "required": true, "description": description, "schema": {"type": "string"}}));
        }
        for (name, schema) in &tool.query {
            let mut schema = schema.clone();
            let required = schema.as_object_mut().and_then(|s| s.remove("required")).and_then(|r| r.as_bool()).unwrap_or(false);
            let description = schema.get("description").cloned().unwrap_or(Value::Null);
            let mut p = json!({"name": name, "in": "query", "required": required, "schema": schema});
            if !description.is_null() {
                p["description"] = description;
            }
            parameters.push(p);
        }
        let mut op = json!({
            "operationId": tool.name,
            "description": tool.description.trim(),
            "parameters": parameters,
            "responses": {"200": {"description": "OK"}},
        });
        if let Some(title) = &tool.title {
            op["summary"] = json!(title);
        }
        if let Some(body) = &tool.body {
            op["requestBody"] = json!({"required": true, "content": {"application/json": {"schema": body}}});
        } else if tool.verb.is_some() || method == "post" {
            // An empty object: a POST without arguments still sends `{}`.
            op["requestBody"] = json!({"required": true, "content": {"application/json": {"schema": {"type": "object", "properties": {}}}}});
        }
        if tool.read_only {
            op["x-mcp-read-only"] = json!(true);
        }
        if tool.destructive {
            op["x-mcp-destructive"] = json!(true);
        }
        let item = paths.entry(path.clone()).or_insert_with(|| json!({}));
        if item.get(&method).is_some() {
            return Err(format!("{}: {} {path} is another tool's already", tool.name, method.to_uppercase()));
        }
        item[&method] = op;
    }
    Ok(json!({
        "openapi": "3.1.0",
        "info": {"title": "Physical AI Production Designer", "version": env!("CARGO_PKG_VERSION")},
        "paths": paths,
    }))
}

fn path_params(path: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = path;
    while let Some(start) = rest.find('{') {
        let Some(end) = rest[start..].find('}') else { break };
        out.push(rest[start + 1..start + end].trim_start_matches('*').to_owned());
        rest = &rest[start + end + 1..];
    }
    out
}
