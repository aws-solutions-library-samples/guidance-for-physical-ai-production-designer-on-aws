//! JSON mirror of the tree (`behaviour_tree.root`) → [`NormNode`].
//!
//! Also accepts the older uppercase dialect (`SEQUENCE`, `EXECUTE`, …) whose ports are top-level
//! fields, mirroring umarell `btcpp.py` `LEGACY_TO_BTCPP` and `to_btcpp`.

use std::collections::BTreeMap;

use serde::Deserialize;
use serde_json::Value;

use super::{BtPath, NodeType, NormNode, PortValue, number};
use crate::error::{DiagCode, Diagnostics, Error, Result};

/// BT.CPP v4 pre/post-condition attributes (scripts the zone protocol cannot express).
pub(crate) fn is_condition(port: &str) -> bool {
    bt_core::builtins::PRECONDITIONS.contains(&port)
        || bt_core::builtins::POSTCONDITIONS.contains(&port)
}

#[derive(Clone, Debug, Deserialize)]
pub struct RawNode {
    #[serde(rename = "type")]
    pub kind: String,
    pub name: Option<String>,
    pub id: Option<String>,
    #[serde(default)]
    pub ports: BTreeMap<String, Value>,
    #[serde(default)]
    pub children: Vec<RawNode>,
    /// Legacy dialect: ports as top-level fields.
    #[serde(flatten)]
    pub legacy: BTreeMap<String, Value>,
}

pub fn normalise(raw: &RawNode, d: &mut Diagnostics) -> Result<NormNode> {
    let mut legacy_seen = false;
    let mut root = convert(raw, &mut legacy_seen);
    let mut bad = None;
    root.walk(&mut |n| {
        if bad.is_none()
            && let Some(c) = n.ports.keys().find(|k| is_condition(k))
        {
            bad = Some((c.clone(), n.label()));
        }
    });
    if let Some((c, node)) = bad {
        return Err(Error::plan(
            DiagCode::E002,
            None,
            format!("pre/post condition {c} on {node} is not supported"),
        ));
    }
    if legacy_seen {
        d.warn(
            DiagCode::W014,
            None,
            "legacy uppercase BT dialect (SEQUENCE/EXECUTE/...) normalised to BT.CPP v4 names",
        );
    }
    let mut n = 0;
    let p = BtPath::root(&root.raw_type);
    number(&mut root, p, &mut n);
    Ok(root)
}

fn convert(raw: &RawNode, legacy_seen: &mut bool) -> NormNode {
    let (ty, legacy) = NodeType::from_id(&raw.kind);
    *legacy_seen |= legacy;
    let mut ports: BTreeMap<String, PortValue> = raw
        .ports
        .iter()
        .map(|(k, v)| (k.clone(), PortValue::Json(v.clone())))
        .collect();
    for (k, v) in &raw.legacy {
        ports
            .entry(k.clone())
            .or_insert_with(|| PortValue::Json(v.clone()));
    }
    if let Some(v) = ports.remove("precomputed_trajectory") {
        ports.entry("trajectory".into()).or_insert(v);
    }
    let raw_type = match &ty {
        NodeType::Unsupported(s) => s.clone(),
        other => canonical(other).to_string(),
    };
    let children = raw
        .children
        .iter()
        .map(|c| convert(c, legacy_seen))
        .collect();
    let node = NormNode {
        ty,
        raw_type,
        legacy,
        name: raw.name.clone(),
        id: raw.id.clone(),
        ports,
        children,
        path: BtPath::default(),
        dfs: 0,
        line: None,
    };
    wrap_start_delay(node)
}

/// `start_delay_s > 0` on a leaf becomes a synthetic `Delay` parent (as `btcpp.to_btcpp` does).
pub(crate) fn wrap_start_delay(mut node: NormNode) -> NormNode {
    if node.ty != NodeType::ExecuteTrajectory {
        return node;
    }
    let delay = node.port_f64("start_delay_s").ok().flatten().unwrap_or(0.0);
    node.ports.remove("start_delay_s");
    if delay <= 0.0 {
        return node;
    }
    let ms = (delay * 1000.0).round() as i64;
    let mut ports = BTreeMap::new();
    ports.insert("delay_msec".to_string(), PortValue::Json(Value::from(ms)));
    NormNode {
        ty: NodeType::Delay,
        raw_type: "Delay".into(),
        legacy: false,
        name: Some(format!("delay_{}", node.label())),
        id: None,
        ports,
        children: vec![node],
        path: BtPath::default(),
        dfs: 0,
        line: None,
    }
}

pub(crate) fn canonical(t: &NodeType) -> &'static str {
    match t {
        NodeType::Sequence => "Sequence",
        NodeType::SequenceWithMemory => "SequenceWithMemory",
        NodeType::Parallel => "Parallel",
        NodeType::Delay => "Delay",
        NodeType::Timeout => "Timeout",
        NodeType::SubTree => "SubTree",
        NodeType::ExecuteTrajectory => "ExecuteTrajectory",
        NodeType::MoveHome => "MoveHome",
        NodeType::WaitForRobot => "WaitForRobot",
        NodeType::CheckSafety => "CheckSafety",
        NodeType::Unsupported(_) => "Unsupported",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_dialect_and_start_delay() {
        let raw: RawNode = serde_json::from_value(serde_json::json!({
            "type": "SEQUENCE",
            "children": [
                {"type": "EXECUTE", "robot_ns": "r1", "operation_id": "A", "precomputed_trajectory": "{t}", "start_delay_s": 1.5},
                {"type": "HOME", "robot_ns": "r1"}
            ]
        }))
        .unwrap();
        let mut d = Diagnostics::default();
        let n = normalise(&raw, &mut d).unwrap();
        assert_eq!(n.ty, NodeType::Sequence);
        assert_eq!(n.children[0].ty, NodeType::Delay);
        assert_eq!(n.children[0].port_f64("delay_msec").unwrap(), Some(1500.0));
        let exec = &n.children[0].children[0];
        assert_eq!(exec.ty, NodeType::ExecuteTrajectory);
        assert_eq!(exec.port_str("trajectory").as_deref(), Some("{t}"));
        assert_eq!(n.children[1].ty, NodeType::MoveHome);
        assert!(d.items().iter().any(|x| x.code == DiagCode::W014));
        assert_eq!(exec.path.0, "/Sequence/Delay[0]/ExecuteTrajectory[0]");
    }
}
