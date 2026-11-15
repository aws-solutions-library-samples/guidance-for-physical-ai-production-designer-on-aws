//! The fixture tools/list of Codesys-MCP-Master 0.19.0 and a check of arguments against its
//! input schemas.

#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde_json::{Map, Value};

use plan2cell_cell::project::Manifest;

pub fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/welding_cell_2_small")
}

pub fn manifest() -> Manifest {
    Manifest::parse(&std::fs::read_to_string(example().join("plc/project.json")).unwrap()).unwrap()
}

/// A file of the example's `plc/` directory.
pub fn source(rel: &str) -> plan2cell_cell::Result<String> {
    Ok(std::fs::read_to_string(example().join("plc").join(rel)).unwrap())
}

/// The tools of the fixture: name -> (description, input schema).
pub type Schemas = BTreeMap<String, (String, Map<String, Value>)>;

pub fn fixture() -> Schemas {
    let f: Value = serde_json::from_str(
        &std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/codesys-mcp-tools.json"),
        )
        .unwrap(),
    )
    .unwrap();
    f["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| {
            (
                t["name"].as_str().unwrap().to_string(),
                (
                    t["description"].as_str().unwrap().to_string(),
                    t["inputSchema"].as_object().unwrap().clone(),
                ),
            )
        })
        .collect()
}

/// The overlay's library tools (`export_project_libraries`, `install_libraries`) as its
/// server.ts declares them (every tool also takes `install`), added to the fixture of the plain
/// 0.19.0.
pub fn add_library_tools(f: &mut Schemas) {
    let install = f["download_missing_libraries"].1["properties"]["install"].clone();
    let tool = |props: Value, required: &[&str]| {
        let mut props = props.as_object().unwrap().clone();
        props.insert("install".into(), install.clone());
        serde_json::json!({
            "type": "object",
            "properties": props,
            "required": required,
            "additionalProperties": false,
        })
        .as_object()
        .unwrap()
        .clone()
    };
    f.insert(
        plan2cell_cell::codesys::EXPORT_LIBRARIES.into(),
        (
            "Copies every library the project resolves to into outDir".into(),
            tool(
                serde_json::json!({
                    "projectFilePath": {"type": "string"},
                    "outDir": {"type": "string"},
                    "includeData": {"type": "boolean"},
                    "maxDataBytes": {"type": "integer", "exclusiveMinimum": 0},
                }),
                &["projectFilePath", "outDir"],
            ),
        ),
    );
    f.insert(
        plan2cell_cell::codesys::INSTALL_LIBRARIES.into(),
        (
            "Installs the libraries of dir\\libraries.json".into(),
            tool(serde_json::json!({"dir": {"type": "string"}}), &["dir"]),
        ),
    );
}

pub fn json_type(v: &Value) -> &'static str {
    match v {
        Value::String(_) => "string",
        Value::Bool(_) => "boolean",
        Value::Number(n) if n.is_i64() || n.is_u64() => "integer",
        Value::Number(_) => "number",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
        Value::Null => "null",
    }
}

fn type_ok(schema: &Value, ty: &str) -> bool {
    match schema["type"].as_str() {
        Some("number") => ty == "number" || ty == "integer",
        Some(t) => t == ty,
        None => true,
    }
}

/// Every problem of `args` (plus `deferred` names with their resolved JSON type) against the
/// schema of `tool`: unknown tool, unknown argument (the schemas forbid additional properties),
/// wrong type, missing required argument, out of an enum.
pub fn schema_errors(
    tools: &Schemas,
    tool: &str,
    args: &Map<String, Value>,
    deferred: &[(&str, &str)],
) -> Vec<String> {
    let Some((_, schema)) = tools.get(tool) else {
        return vec![format!("no tool {tool}")];
    };
    let schema = Value::Object(schema.clone());
    let props = schema["properties"]
        .as_object()
        .cloned()
        .unwrap_or_default();
    let mut errs = Vec::new();
    for (k, v) in args {
        match props.get(k) {
            None => errs.push(format!("{tool}: unknown argument {k}")),
            Some(p) => {
                if !type_ok(p, json_type(v)) {
                    errs.push(format!(
                        "{tool}.{k}: {} where the schema wants {}",
                        json_type(v),
                        p["type"]
                    ));
                }
                if let Some(e) = p["enum"].as_array()
                    && !e.contains(v)
                {
                    errs.push(format!("{tool}.{k}: {v} not in {e:?}"));
                }
                if let (Some("array"), Value::Array(items)) = (p["type"].as_str(), v) {
                    for it in items {
                        if !type_ok(&p["items"], json_type(it)) {
                            errs.push(format!("{tool}.{k}[]: {}", json_type(it)));
                        }
                    }
                    if let Some(min) = p["minItems"].as_u64()
                        && (items.len() as u64) < min
                    {
                        errs.push(format!("{tool}.{k}: fewer than {min} items"));
                    }
                }
            }
        }
    }
    for (k, ty) in deferred {
        match props.get(*k) {
            None => errs.push(format!("{tool}: unknown deferred argument {k}")),
            Some(p) if !type_ok(p, ty) => errs.push(format!("{tool}.{k}: deferred {ty}")),
            Some(_) => {}
        }
    }
    for r in schema["required"].as_array().into_iter().flatten() {
        let r = r.as_str().unwrap();
        if !args.contains_key(r) && !deferred.iter().any(|(k, _)| *k == r) {
            errs.push(format!("{tool}: required argument {r} missing"));
        }
    }
    errs
}

/// The DeviceID bt2plc gives a head's KUKA device with `safety_module = false` (DAP `DIM 3`).
pub const KUKA_DIM3: (&str, &str) = (
    "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 3",
    "V8.3 - HW 1",
);

/// The example's `project.json` as bt2plc writes it with `safety_module = false` for the head's
/// PN device `device`: DAP `DIM 3`, slot 1 empty (no module, no IO), `ensure_empty_slot`
/// instead of the slot-1 `ensure_module`, the DIM 3 prerequisite.
pub fn without_safety_module(text: &str, device: &str) -> String {
    let mut v: Value = serde_json::from_str(text).unwrap();
    fn node<'a>(n: &'a mut Value, name: &str) -> Option<&'a mut Value> {
        if n["name"] == name {
            return Some(n);
        }
        n.get_mut("children")?
            .as_array_mut()?
            .iter_mut()
            .find_map(|c| node(c, name))
    }
    let dim3 = serde_json::json!({"type": 81, "id": KUKA_DIM3.0, "version": KUKA_DIM3.1});
    let pre = v["prerequisites"].as_array_mut().unwrap();
    pre.insert(
        1,
        serde_json::json!({"id": "gsdml_dim3", "kind": "device_repository_install",
                           "needed_by": ["head_plc"], "provides": dim3}),
    );
    let head = v["projects"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|p| p["role"] == "head_plc")
        .unwrap();
    let n = node(&mut head["device_tree"], device).unwrap();
    n["device_id"] = dim3;
    n["slots"][0] = serde_json::json!({"slot": 1, "empty": true, "module_id": null,
                                       "profisafe": false, "io": []});
    for st in head["steps"].as_array_mut().unwrap() {
        if st["op"] == "ensure_module" && st["device"] == device && st["slot"] == 1 {
            *st = serde_json::json!({"op": "ensure_empty_slot", "device": device, "slot": 1,
                                     "note": "SIMULATION ONLY (safety_module = false)"});
        }
    }
    serde_json::to_string_pretty(&v).unwrap()
}

/// A copy of the example whose `plc/project.json` is [`without_safety_module`] for `device`
/// (tag `tag`), its head IO mapping CSV without that device's FDIO64 rows.
pub fn example_without_safety_module(device: &str, tag: &str) -> tempfile::TempDir {
    fn copy(from: &Path, to: &Path) {
        std::fs::create_dir_all(to).unwrap();
        for e in std::fs::read_dir(from).unwrap() {
            let e = e.unwrap();
            let t = to.join(e.file_name());
            if e.file_type().unwrap().is_dir() {
                copy(&e.path(), &t);
            } else {
                std::fs::copy(e.path(), t).unwrap();
            }
        }
    }
    let d = tempfile::tempdir().unwrap();
    copy(&example(), d.path());
    let pj = d.path().join("plc/project.json");
    let text = without_safety_module(&std::fs::read_to_string(&pj).unwrap(), device);
    std::fs::write(&pj, text).unwrap();
    let csv = d.path().join("plc/codesys/head/io_mapping.csv");
    let fd = format!(",SafeIO_FDIO64_{tag}");
    let kept: String = std::fs::read_to_string(&csv)
        .unwrap()
        .lines()
        .filter(|l| !l.ends_with(&fd))
        .map(|l| format!("{l}\n"))
        .collect();
    std::fs::write(&csv, kept).unwrap();
    d
}

/// The head's `symbol_config` as bt2plc writes it with `[opcua] symbols = true`.
pub fn head_symbol_config() -> Value {
    serde_json::json!({
        "support_opc_ua": true,
        "symbols": [
            {"signature": "PLC_PRG", "access": "ReadOnly"},
            {"signature": "GVL_Cell", "variable": "PLAN_ID", "access": "ReadOnly", "optional": true}
        ],
        "require": [
            "PLC_PRG.eCellState", "PLC_PRG.eCellError", "PLC_PRG.bCellError",
            "PLC_PRG.bRecoveryRequired", "PLC_PRG.udiCycles", "PLC_PRG.bAutoRun"
        ],
        "opc_ua_paths": ["PLC_PRG.eCellState", "PLC_PRG.fbCell.adwGrant", "GVL_Cell.PLAN_ID"]
    })
}

/// The example's `project.json` with the head's Symbol Configuration (step `symbol_config`
/// between `build` and `save`).
pub fn with_symbols(text: &str) -> String {
    let mut v: Value = serde_json::from_str(text).unwrap();
    let head = v["projects"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|p| p["role"] == "head_plc")
        .unwrap();
    head["symbol_config"] = head_symbol_config();
    let steps = head["steps"].as_array_mut().unwrap();
    let at = steps.iter().position(|s| s["op"] == "save").unwrap();
    steps.insert(at, serde_json::json!({"op": "symbol_config"}));
    serde_json::to_string_pretty(&v).unwrap()
}

/// A copy of the example whose `plc/project.json` is [`with_symbols`], and its manifest.
pub fn example_with_symbols() -> (tempfile::TempDir, Manifest) {
    let d = copy_example();
    let pj = d.path().join("plc/project.json");
    let text = with_symbols(&std::fs::read_to_string(&pj).unwrap());
    std::fs::write(&pj, &text).unwrap();
    let m = Manifest::parse(&text).unwrap();
    (d, m)
}

/// A copy of the example in a temporary directory.
pub fn copy_example() -> tempfile::TempDir {
    fn copy(from: &Path, to: &Path) {
        std::fs::create_dir_all(to).unwrap();
        for e in std::fs::read_dir(from).unwrap() {
            let e = e.unwrap();
            let t = to.join(e.file_name());
            if e.file_type().unwrap().is_dir() {
                copy(&e.path(), &t);
            } else {
                std::fs::copy(e.path(), t).unwrap();
            }
        }
    }
    let d = tempfile::tempdir().unwrap();
    copy(&example(), d.path());
    d
}

/// The manifest of [`without_safety_module`] for `device`.
pub fn manifest_without_safety_module(device: &str) -> Manifest {
    let text = std::fs::read_to_string(example().join("plc/project.json")).unwrap();
    Manifest::parse(&without_safety_module(&text, device)).unwrap()
}
