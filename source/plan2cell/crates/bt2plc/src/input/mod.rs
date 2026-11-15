//! Planner input: the umarell plan envelope, a bare `behaviour_tree` object, or BT.CPP v4 XML.
//!
//! Both the JSON `root` mirror and the XML are normalised to [`NormNode`], so lowering never
//! sees which one it came from.

pub mod blackboard;
pub mod envelope;
pub mod json_tree;
pub mod xml_tree;

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use serde::de::DeserializeOwned;
use serde_json::Value;

use crate::error::{DiagCode, Diagnostics, Error, Result};
use crate::util::hash::fnv1a64;
pub use blackboard::{Blackboard, OpRef, Phase, Trajectory, Waypoint};

// ───────────────────────────── normalised tree ─────────────────────────────

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NodeType {
    Sequence,
    SequenceWithMemory,
    Parallel,
    Delay,
    Timeout,
    SubTree,
    ExecuteTrajectory,
    MoveHome,
    WaitForRobot,
    CheckSafety,
    Unsupported(String),
}

impl NodeType {
    pub fn from_id(id: &str) -> (NodeType, bool) {
        // (type, legacy dialect)
        match id {
            "Sequence" => (NodeType::Sequence, false),
            "SequenceWithMemory" | "SequenceStar" => (NodeType::SequenceWithMemory, false),
            "Parallel" => (NodeType::Parallel, false),
            "Delay" => (NodeType::Delay, false),
            "Timeout" => (NodeType::Timeout, false),
            "SubTree" | "SubTreePlus" => (NodeType::SubTree, false),
            "ExecuteTrajectory" => (NodeType::ExecuteTrajectory, false),
            "MoveHome" => (NodeType::MoveHome, false),
            "WaitForRobot" => (NodeType::WaitForRobot, false),
            "CheckSafety" => (NodeType::CheckSafety, false),
            "SEQUENCE" => (NodeType::Sequence, true),
            "PARALLEL" => (NodeType::Parallel, true),
            "EXECUTE" => (NodeType::ExecuteTrajectory, true),
            "HOME" => (NodeType::MoveHome, true),
            "INTERLOCK_WAIT" => (NodeType::WaitForRobot, true),
            "SAFETY_GATE" => (NodeType::CheckSafety, true),
            other => (
                NodeType::Unsupported(other.to_string()),
                other.chars().all(|c| !c.is_lowercase()),
            ),
        }
    }
}

/// A port value: typed JSON (JSON mirror) or an attribute string (XML).
#[derive(Clone, Debug, PartialEq)]
pub enum PortValue {
    Json(Value),
    Text(String),
}

impl PortValue {
    /// `"{key}"` -> `key`.
    pub fn pointer(&self) -> Option<&str> {
        let s = match self {
            PortValue::Json(Value::String(s)) => s.as_str(),
            PortValue::Text(s) => s.as_str(),
            _ => return None,
        };
        let s = s.trim();
        (s.len() > 2
            && s.starts_with('{')
            && s.ends_with('}')
            && !s[1..s.len() - 1].contains(['{', '}', '"']))
        .then(|| &s[1..s.len() - 1])
    }

    /// The value as JSON; XML text is parsed as JSON when it looks like JSON.
    pub fn to_json(&self) -> Value {
        match self {
            PortValue::Json(v) => v.clone(),
            PortValue::Text(s) => {
                let t = s.trim();
                if t.is_empty() {
                    return Value::String(String::new());
                }
                match t {
                    "true" | "TRUE" | "True" => return Value::Bool(true),
                    "false" | "FALSE" | "False" => return Value::Bool(false),
                    _ => {}
                }
                if (t.starts_with('[') || t.starts_with('{') || t.parse::<f64>().is_ok())
                    && let Ok(v) = serde_json::from_str(t)
                {
                    return v;
                }
                Value::String(s.clone())
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            PortValue::Json(Value::Null) => true,
            PortValue::Json(Value::String(s)) | PortValue::Text(s) => s.trim().is_empty(),
            PortValue::Json(Value::Array(a)) => a.is_empty(),
            _ => false,
        }
    }
}

/// Path of a node in the tree, e.g. `/Sequence/Parallel[3]/ExecuteTrajectory[1]`.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct BtPath(pub String);

impl BtPath {
    pub fn root(ty: &str) -> Self {
        BtPath(format!("/{ty}"))
    }

    pub fn child(&self, ty: &str, index: usize) -> Self {
        BtPath(format!("{}/{ty}[{index}]", self.0))
    }
}

impl fmt::Display for BtPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Clone, Debug)]
pub struct NormNode {
    pub ty: NodeType,
    pub raw_type: String,
    pub legacy: bool,
    pub name: Option<String>,
    pub id: Option<String>,
    pub ports: BTreeMap<String, PortValue>,
    pub children: Vec<NormNode>,
    pub path: BtPath,
    /// Pre-order index (determinism key).
    pub dfs: usize,
    pub line: Option<u32>,
}

impl NormNode {
    pub fn label(&self) -> String {
        self.name.clone().unwrap_or_else(|| self.raw_type.clone())
    }

    pub fn port(&self, name: &str) -> Option<&PortValue> {
        self.ports.get(name).filter(|v| !v.is_empty())
    }

    /// The port as text. XML attributes are returned verbatim (no JSON coercion, so an
    /// `operation_id="1.10"` stays `1.10`).
    pub fn port_str(&self, name: &str) -> Option<String> {
        self.port(name).map(|v| match v {
            PortValue::Text(s) => s.trim().to_string(),
            PortValue::Json(Value::String(s)) => s.clone(),
            PortValue::Json(other) => other.to_string(),
        })
    }

    /// A finite number (strings are parsed; `inf`/`nan` are rejected).
    pub fn port_f64(&self, name: &str) -> Result<Option<f64>> {
        let v = self.port_f64_raw(name)?;
        match v {
            Some(x) if !x.is_finite() => Err(Error::plan(
                DiagCode::E010,
                Some(&self.path.0),
                format!("port {name}: {x} is not a finite number"),
            )),
            other => Ok(other),
        }
    }

    fn port_f64_raw(&self, name: &str) -> Result<Option<f64>> {
        let Some(v) = self.port(name) else {
            return Ok(None);
        };
        match v.to_json() {
            Value::Number(n) => Ok(n.as_f64()),
            Value::String(s) => s.trim().parse::<f64>().map(Some).map_err(|_| {
                Error::plan(
                    DiagCode::E010,
                    Some(&self.path.0),
                    format!("port {name}: '{s}' is not a number"),
                )
            }),
            other => Err(Error::plan(
                DiagCode::E010,
                Some(&self.path.0),
                format!("port {name}: {other} is not a number"),
            )),
        }
    }

    pub fn port_bool(&self, name: &str) -> Result<Option<bool>> {
        let Some(v) = self.port(name) else {
            return Ok(None);
        };
        match v.to_json() {
            Value::Bool(b) => Ok(Some(b)),
            Value::Number(n) => Ok(Some(n.as_f64().unwrap_or(0.0) != 0.0)),
            other => Err(Error::plan(
                DiagCode::E010,
                Some(&self.path.0),
                format!("port {name}: {other} is not a boolean"),
            )),
        }
    }

    /// A list port: inline JSON list, JSON text in an XML attribute, or a `{key}` pointer.
    pub fn port_list<T: DeserializeOwned>(&self, name: &str, bb: &Blackboard) -> Result<Vec<T>> {
        let Some(v) = self.port(name) else {
            return Ok(Vec::new());
        };
        let json = match v.pointer() {
            Some(key) => bb.get(key).cloned().ok_or_else(|| {
                Error::plan(
                    DiagCode::E013,
                    Some(&self.path.0),
                    format!("port {name}: blackboard key '{key}' not found"),
                )
            })?,
            None => v.to_json(),
        };
        let json = match json {
            Value::String(s) if s.trim().is_empty() => return Ok(Vec::new()),
            Value::String(s) => serde_json::from_str(&s).unwrap_or(Value::String(s)),
            other => other,
        };
        let json = match json {
            Value::Array(_) => json,
            Value::Null => return Ok(Vec::new()),
            single => Value::Array(vec![single]),
        };
        serde_json::from_value(json).map_err(|e| {
            Error::plan(
                DiagCode::E010,
                Some(&self.path.0),
                format!("port {name}: {e}"),
            )
        })
    }

    pub fn walk<'a>(&'a self, f: &mut impl FnMut(&'a NormNode)) {
        f(self);
        for c in &self.children {
            c.walk(f);
        }
    }
}

/// Assign pre-order indices and paths after construction.
pub(crate) fn number(node: &mut NormNode, path: BtPath, counter: &mut usize) {
    node.dfs = *counter;
    *counter += 1;
    node.path = path;
    for (i, c) in node.children.iter_mut().enumerate() {
        let p = node.path.child(&c.raw_type.clone(), i);
        number(c, p, counter);
    }
}

// ───────────────────────────── document ─────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceFormat {
    EnvelopeJson,
    TreeJson,
    Xml,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, clap::ValueEnum)]
pub enum SourceSel {
    /// JSON `root` mirror if present, otherwise the embedded `xml`.
    #[default]
    Auto,
    Json,
    Xml,
}

#[derive(Clone, Debug, Default)]
pub struct LoadOptions {
    pub source: SourceSel,
    /// Blackboard JSON for plain XML input.
    pub blackboard: Option<PathBuf>,
}

#[derive(Clone, Debug)]
pub struct InputDoc {
    pub format: SourceFormat,
    /// Which tree representation was lowered: "json" or "xml".
    pub tree_source: &'static str,
    pub file_name: String,
    pub fingerprint: u64,
    pub tree_id: String,
    pub cell_id: String,
    pub created_s: Option<f64>,
    pub num_operations: Option<u32>,
    pub root: NormNode,
    pub blackboard: Blackboard,
    pub xml: Option<String>,
}

pub fn load(path: &Path, opts: &LoadOptions, d: &mut Diagnostics) -> Result<InputDoc> {
    let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let bb = match &opts.blackboard {
        Some(p) => {
            let t = std::fs::read_to_string(p).map_err(|e| Error::io(p, e))?;
            Some(serde_json::from_str::<BTreeMap<String, Value>>(&t)?)
        }
        None => None,
    };
    load_str(&text, &name, opts.source, bb, d)
}

pub fn load_str(
    text: &str,
    file_name: &str,
    source: SourceSel,
    extra_blackboard: Option<BTreeMap<String, Value>>,
    d: &mut Diagnostics,
) -> Result<InputDoc> {
    let fingerprint = fnv1a64(text.as_bytes());
    let stem = file_name
        .trim_end_matches(".json")
        .trim_end_matches(".xml")
        .to_string();
    if text.trim_start().starts_with('<') {
        let root = xml_tree::parse(text, d)?;
        let mut bb = Blackboard::default();
        if let Some(extra) = extra_blackboard {
            bb.0.extend(extra);
        }
        return Ok(InputDoc {
            format: SourceFormat::Xml,
            tree_source: "xml",
            file_name: file_name.to_string(),
            fingerprint,
            tree_id: stem.clone(),
            cell_id: stem,
            created_s: None,
            num_operations: None,
            root,
            blackboard: bb,
            xml: Some(text.to_string()),
        });
    }
    let value: Value = serde_json::from_str(text)?;
    let (format, env) = envelope::from_value(value)?;
    if let Some(status) = env.status.as_deref()
        && status != "ok"
    {
        return Err(Error::plan(
            DiagCode::E001,
            None,
            format!("plan status is '{status}'; only plans with status 'ok' are executable"),
        ));
    }
    let bt = env.behaviour_tree;
    let mut bb = Blackboard(bt.blackboard.clone());
    if let Some(extra) = extra_blackboard {
        bb.0.extend(extra);
    }
    let use_json = match source {
        SourceSel::Json => true,
        SourceSel::Xml => false,
        SourceSel::Auto => bt.root.is_some(),
    };
    let (root, tree_source) = if use_json {
        let raw = bt.root.as_ref().ok_or_else(|| {
            Error::plan(DiagCode::E014, None, "the JSON has no behaviour_tree.root")
        })?;
        (json_tree::normalise(raw, d)?, "json")
    } else {
        let xml = bt.xml.as_deref().ok_or_else(|| {
            Error::plan(DiagCode::E014, None, "the JSON has no behaviour_tree.xml")
        })?;
        (xml_tree::parse(xml, d)?, "xml")
    };
    let tree_id = bt
        .tree_id
        .clone()
        .or(env.tree_id.clone())
        .unwrap_or_else(|| stem.clone());
    let cell_id = bt
        .cell_id
        .clone()
        .or(env.cell_id.clone())
        .unwrap_or_else(|| tree_id.clone());
    Ok(InputDoc {
        format,
        tree_source,
        file_name: file_name.to_string(),
        fingerprint,
        tree_id,
        cell_id,
        created_s: env.created_s,
        num_operations: env.num_operations,
        root,
        blackboard: bb,
        xml: bt.xml,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> String {
        std::fs::read_to_string(format!(
            "{}/../../test_data/{name}",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap()
    }

    #[test]
    fn both_samples_load_from_json_and_xml_identically() {
        for f in [
            "bt_1790934549_welding_cell_2_small.json",
            "bt_1791274820_cell_2robot.json",
        ] {
            let text = fixture(f);
            let mut d = Diagnostics::default();
            let j = load_str(&text, f, SourceSel::Json, None, &mut d).unwrap();
            let x = load_str(&text, f, SourceSel::Xml, None, &mut d).unwrap();
            assert_eq!(shape(&j.root), shape(&x.root), "{f}");
            assert_eq!(j.cell_id, x.cell_id);
        }
    }

    fn shape(n: &NormNode) -> String {
        let mut s = format!("{:?}(", n.ty);
        for k in ["robot_ns", "operation_id", "operation_type", "trajectory"] {
            if let Some(v) = n.port_str(k) {
                s.push_str(&format!("{k}={v},"));
            }
        }
        for c in &n.children {
            s.push_str(&shape(c));
        }
        s.push(')');
        s
    }

    #[test]
    fn failed_status_is_rejected() {
        let mut d = Diagnostics::default();
        let text =
            r#"{"status":"failed","behaviour_tree":{"root":{"type":"Sequence","children":[]}}}"#;
        let e = load_str(text, "x.json", SourceSel::Auto, None, &mut d).unwrap_err();
        assert!(e.to_string().contains("E001"), "{e}");
    }

    #[test]
    fn xml_text_ports_parse_as_json() {
        assert_eq!(
            PortValue::Text("0.5".into()).to_json(),
            serde_json::json!(0.5)
        );
        assert_eq!(
            PortValue::Text("false".into()).to_json(),
            serde_json::json!(false)
        );
        assert_eq!(
            PortValue::Text(r#"[{"robot":"r1","operation_id":"W01"}]"#.into()).to_json()[0]["robot"],
            "r1"
        );
        assert_eq!(PortValue::Text("{traj_a}".into()).pointer(), Some("traj_a"));
        assert_eq!(PortValue::Text(r#"{"a":1}"#.into()).pointer(), None);
    }
}
