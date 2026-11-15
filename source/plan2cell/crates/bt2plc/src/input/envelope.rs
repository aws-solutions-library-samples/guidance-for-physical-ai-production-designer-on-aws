//! The umarell plan envelope (`plan_store.save_plan`) and the `behaviour_tree` object.

use std::collections::BTreeMap;

use serde::Deserialize;
use serde_json::Value;

use super::SourceFormat;
use super::json_tree::RawNode;
use crate::error::{DiagCode, Error, Result};

#[derive(Debug, Deserialize)]
pub struct Envelope {
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    pub created_s: Option<f64>,
    /// "ok" | "failed"; anything but "ok" is rejected (E001).
    pub status: Option<String>,
    pub num_operations: Option<u32>,
    #[serde(alias = "behavior_tree")]
    pub behaviour_tree: BehaviourTree,
}

#[derive(Debug, Default, Deserialize)]
pub struct BehaviourTree {
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    #[serde(rename = "BTCPP_format")]
    pub btcpp_format: Option<Value>,
    pub main_tree_to_execute: Option<String>,
    pub root: Option<RawNode>,
    #[serde(default)]
    pub blackboard: BTreeMap<String, Value>,
    #[serde(default)]
    pub metadata: Value,
    pub xml: Option<String>,
}

/// Accepts the full envelope, a bare `behaviour_tree` object, or a bare root node.
pub fn from_value(v: Value) -> Result<(SourceFormat, Envelope)> {
    let obj = v
        .as_object()
        .ok_or_else(|| Error::plan(DiagCode::E014, None, "the JSON is not an object"))?;
    if obj.contains_key("behaviour_tree") || obj.contains_key("behavior_tree") {
        return Ok((SourceFormat::EnvelopeJson, serde_json::from_value(v)?));
    }
    if obj.contains_key("root") || obj.contains_key("xml") {
        let bt: BehaviourTree = serde_json::from_value(v)?;
        return Ok((SourceFormat::TreeJson, wrap(bt)));
    }
    if obj.contains_key("type") {
        let root: RawNode = serde_json::from_value(v)?;
        return Ok((
            SourceFormat::TreeJson,
            wrap(BehaviourTree {
                root: Some(root),
                ..Default::default()
            }),
        ));
    }
    Err(Error::plan(
        DiagCode::E014,
        None,
        "no behaviour_tree, root or xml found in the JSON",
    ))
}

fn wrap(bt: BehaviourTree) -> Envelope {
    Envelope {
        tree_id: None,
        cell_id: None,
        created_s: None,
        status: None,
        num_operations: None,
        behaviour_tree: bt,
    }
}
