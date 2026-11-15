//! BT.CPP v4 XML (via btgui's `bt-core`) → [`NormNode`].
//!
//! `bt_core::Document::instantiate` expands SubTrees in place; a SubTree node is kept as a
//! transparent [`NodeType::SubTree`] and `{key}` ports inside it are resolved through the
//! SubTree's remapping ports (literal → value, pointer → continue upward, `_autoremap` → same key).

use std::collections::BTreeMap;

use bt_core::{Document, NodeKind, Tree, Uid};

use super::json_tree::{canonical, wrap_start_delay};
use super::{BtPath, NodeType, NormNode, PortValue, number};
use crate::error::{DiagCode, Diagnostics, Error, Result};

pub fn parse(xml: &str, d: &mut Diagnostics) -> Result<NormNode> {
    let doc = Document::parse(xml).map_err(Error::Xml)?;
    let tree = doc.instantiate(doc.main_tree_id()).map_err(Error::Xml)?;
    // bt-core is lenient (it is a viewer); for code generation a structural problem would
    // silently drop operations, so everything except unknown-model notes is an error.
    for w in doc.warnings.iter().chain(&tree.warnings) {
        if w.contains("is not a builtin and not in <TreeNodesModel>") || w.contains("legacy <Root>")
        {
            continue;
        }
        return Err(Error::plan(DiagCode::E002, None, format!("XML: {w}")));
    }
    let root = tree.root();
    let Some(&first) = root.children.first() else {
        return Err(Error::plan(
            DiagCode::E014,
            None,
            "the XML main tree is empty",
        ));
    };
    let mut legacy_seen = false;
    let mut node = convert(&tree, first, &mut legacy_seen)?;
    if legacy_seen {
        d.warn(
            DiagCode::W014,
            None,
            "legacy uppercase BT dialect normalised to BT.CPP v4 names",
        );
    }
    let mut n = 0;
    let p = BtPath::root(&node.raw_type);
    number(&mut node, p, &mut n);
    Ok(node)
}

fn convert(tree: &Tree, uid: Uid, legacy_seen: &mut bool) -> Result<NormNode> {
    let n = tree.node(uid);
    if let Some((cond, _)) = n.conditions.first() {
        return Err(Error::plan(
            DiagCode::E002,
            None,
            format!(
                "line {}: pre/post condition {cond} on <{}> is not supported",
                n.line, n.id
            ),
        ));
    }
    let (ty, legacy) = if n.kind == NodeKind::SubTree {
        (NodeType::SubTree, false)
    } else {
        NodeType::from_id(&n.id)
    };
    *legacy_seen |= legacy;
    let mut ports = BTreeMap::new();
    for p in &n.ports {
        let Some(v) = p.value.as_deref() else {
            continue;
        };
        let resolved = resolve_remap(tree, uid, v);
        ports.insert(p.name.clone(), PortValue::Text(resolved));
    }
    if let Some(v) = ports.remove("precomputed_trajectory") {
        ports.entry("trajectory".into()).or_insert(v);
    }
    let raw_type = match &ty {
        NodeType::Unsupported(s) => s.clone(),
        other => canonical(other).to_string(),
    };
    let children = n
        .children
        .iter()
        .map(|&c| convert(tree, c, legacy_seen))
        .collect::<Result<Vec<_>>>()?;
    let name = (n.name != n.id).then(|| n.name.clone());
    Ok(wrap_start_delay(NormNode {
        ty,
        raw_type,
        legacy,
        name,
        id: None,
        ports,
        children,
        path: BtPath::default(),
        dfs: 0,
        line: Some(n.line),
    }))
}

/// Resolve a `{key}` written inside an expanded SubTree through the SubTree ports.
fn resolve_remap(tree: &Tree, uid: Uid, value: &str) -> String {
    let Some(mut key) = bt_core::model::blackboard_pointer(value).map(str::to_string) else {
        return value.to_string();
    };
    let mut cur = tree.node(uid).parent;
    while let Some(p) = cur {
        let pn = tree.node(p);
        if pn.kind == NodeKind::SubTree {
            match pn.port_value(&key) {
                Some(v) => match bt_core::model::blackboard_pointer(v) {
                    Some(k2) => key = k2.to_string(),
                    None => return v.to_string(),
                },
                None => {
                    let autoremap = pn.subtree.as_ref().is_some_and(|s| s.autoremap);
                    if !autoremap {
                        // Not remapped: the key is private to the SubTree's blackboard, which
                        // starts empty in BT.CPP - make it unresolvable (E008/E013).
                        return format!("{{{key} (not remapped by SubTree {})}}", pn.id);
                    }
                }
            }
        }
        cur = pn.parent;
    }
    format!("{{{key}}}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subtree_ports_are_remapped() {
        let xml = r#"<root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence>
            <SubTree ID="Job" robot="r1" traj="{traj_r1_A}"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Job">
            <ExecuteTrajectory robot_ns="{robot}" operation_id="A" trajectory="{traj}"/>
          </BehaviorTree>
        </root>"#;
        let mut d = Diagnostics::default();
        let n = parse(xml, &mut d).unwrap();
        let sub = &n.children[0];
        assert_eq!(sub.ty, NodeType::SubTree);
        let exec = &sub.children[0];
        assert_eq!(exec.port_str("robot_ns").as_deref(), Some("r1"));
        assert_eq!(exec.port_str("trajectory").as_deref(), Some("{traj_r1_A}"));
    }

    #[test]
    fn preconditions_are_rejected() {
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="M"><Sequence><MoveHome robot_ns="r1" _skipIf="x"/></Sequence></BehaviorTree></root>"#;
        let mut d = Diagnostics::default();
        assert!(parse(xml, &mut d).unwrap_err().to_string().contains("E002"));
    }
}
