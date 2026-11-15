//! Plan documents: every file layout cell-runner accepts, normalized to BehaviorTree.CPP v4
//! XML plus the blackboard its `{key}` ports point into.
//!
//! Ported from cell-runner's `trajectory.rs:67-341`, verbatim but for two things:
//!
//! - [`merge_sibling`] is the pure part of `parse_plan_file` (`:135-155`); reading the files
//!   is left to the caller, so this crate never touches the file system.
//! - [`json_tree_to_xml`] appends a `<TreeNodesModel>` for the cell leaves, so the converted
//!   `root` form shows typed ports with cell-runner's defaults instead of "unknown model"
//!   nodes. It declares no `name` port (`name` is an attribute to BehaviorTree.CPP) and
//!   `CheckSafety` as a `<Condition>`.
//!
//! | layout                                          | tree from           | blackboard from        |
//! |-------------------------------------------------|---------------------|------------------------|
//! | `{behaviour_tree: {xml, blackboard, ..}}`       | `xml`               | `blackboard`           |
//! | `{behaviour_tree: {root, ..}}` / `{root}`       | `root`, converted   | `blackboard` + inline `precomputed_trajectory` |
//! | bare `.xml`                                     | the file            | sibling `.json` ([`merge_sibling`]) |

use std::collections::BTreeMap;

use serde::Deserialize;
use serde_json::{Map, Value, json};

use crate::joints::DEFAULT_INTERLOCK_TOL_RAD;
use crate::ports::{default_of, leaf_ports};

/// One trajectory waypoint as written in a plan file. Extra fields (e.g. the
/// planner's `phase`) are ignored.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct PlanWaypoint {
    /// Joint angles, rad (padded with zeros or cut to six when placed).
    #[serde(default)]
    pub positions: Vec<f64>,
    /// Seconds from the trajectory's start.
    #[serde(default)]
    pub time_s: f64,
}

/// One joint-space dependency of a `WaitForRobot` node.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct JointDependency {
    /// The robot waited for.
    pub robot: String,
    /// The configuration it must reach (an empty list is no dependency).
    #[serde(default)]
    pub joint_positions: Vec<f64>,
    /// Largest joint error that counts as there, rad (≤ 0: the default).
    #[serde(default = "default_tolerance")]
    pub tolerance_rad: f64,
}

fn default_tolerance() -> f64 {
    DEFAULT_INTERLOCK_TOL_RAD
}

/// A plan as BehaviorTree.CPP sees it: v4 XML plus the blackboard its `{key}`
/// ports resolve against.
#[derive(Debug, Clone, PartialEq)]
pub struct BtPlan {
    /// The BehaviorTree.CPP v4 XML.
    pub xml: String,
    /// The initial blackboard (trajectories, interlock targets, …).
    pub blackboard: Map<String, Value>,
}

impl BtPlan {
    /// The blackboard as an executor seeds it.
    pub fn initial_blackboard(&self) -> BTreeMap<String, Value> {
        self.blackboard
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
}

/// A plan's identity plus its tree, normalized across every file layout.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedPlan {
    /// The plan's `tree_id`.
    pub tree_id: Option<String>,
    /// The cell it was planned for.
    pub cell_id: Option<String>,
    /// How many operations the planner placed (top-level field only).
    pub num_operations: Option<usize>,
    /// The tree to run.
    pub tree: BtPlan,
}

/// Whether a plan's text is BehaviorTree.CPP XML rather than JSON.
pub fn is_xml(text: &str) -> bool {
    text.trim_start().starts_with('<')
}

/// Parse a plan from its text: BehaviorTree.CPP XML, or any JSON layout. A leading
/// byte-order mark (some Windows editors write one) is ignored.
pub fn parse_plan(text: &str) -> Result<ParsedPlan, String> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    if is_xml(text) {
        return Ok(ParsedPlan {
            tree_id: None,
            cell_id: None,
            num_operations: None,
            tree: BtPlan {
                xml: text.to_string(),
                blackboard: Map::new(),
            },
        });
    }
    let value: Value = serde_json::from_str(text).map_err(|e| format!("invalid plan JSON: {e}"))?;
    parse_plan_value(value)
}

/// Complete a bare `.xml` plan with the `.json` the planner writes next to it
/// (cell-runner's `parse_plan_file`, without the file system).
///
/// A bare `.xml` carries only the tree; its trajectories live in the blackboard of the
/// sibling, which also gives the plan's ids. `sibling_json` is that file's text, if it
/// exists, and `file_stem` the plan file's name without extension, the fallback `tree_id`.
/// If the sibling does not parse, `parsed` keeps its own (empty) blackboard and still gets
/// the stem as its `tree_id`, and the error is returned.
pub fn merge_sibling(
    parsed: &mut ParsedPlan,
    sibling_json: Option<&str>,
    file_stem: Option<&str>,
) -> Result<(), String> {
    let merged = match sibling_json {
        Some(text) => parse_plan(text).map(|side| {
            parsed.tree.blackboard = side.tree.blackboard;
            parsed.tree_id = side.tree_id;
            parsed.cell_id = side.cell_id;
            parsed.num_operations = side.num_operations;
        }),
        None => Ok(()),
    };
    if parsed.tree_id.is_none() {
        parsed.tree_id = file_stem.map(str::to_string);
    }
    merged
}

/// Parse a plan from an already-decoded JSON value (inline `plan` parameter).
pub fn parse_plan_value(value: Value) -> Result<ParsedPlan, String> {
    let obj = value
        .as_object()
        .ok_or_else(|| "invalid plan object: expected a JSON object".to_string())?;
    let str_of =
        |o: &Map<String, Value>, k: &str| o.get(k).and_then(Value::as_str).map(str::to_string);

    let mut tree_id = str_of(obj, "tree_id");
    let mut cell_id = str_of(obj, "cell_id");
    let num_operations = obj
        .get("num_operations")
        .and_then(Value::as_u64)
        .map(|n| n as usize);

    let wrapped = obj
        .get("behaviour_tree")
        .or_else(|| obj.get("behavior_tree"))
        .and_then(Value::as_object);
    let (holder, root) = match wrapped {
        Some(bt) => {
            tree_id = tree_id.or_else(|| str_of(bt, "tree_id"));
            cell_id = cell_id.or_else(|| str_of(bt, "cell_id"));
            (bt, bt.get("root"))
        }
        None => (obj, obj.get("root")),
    };
    let mut blackboard = holder
        .get("blackboard")
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();

    // The XML is the BehaviorTree.CPP representation proper; `root` is the
    // planner's JSON mirror of it and only used when no XML is present.
    let xml = match (holder.get("xml").and_then(Value::as_str), root) {
        (Some(xml), _) => xml.to_string(),
        (None, Some(root)) => json_tree_to_xml(root, &mut blackboard)?,
        (None, None) if wrapped.is_some() => {
            return Err("`behaviour_tree` has neither `xml` nor `root`".to_string());
        }
        (None, None) => return Err("plan has neither `behaviour_tree` nor `root`".to_string()),
    };
    Ok(ParsedPlan {
        tree_id,
        cell_id,
        num_operations,
        tree: BtPlan { xml, blackboard },
    })
}

/// Node type in the older upper-case JSON dialect → BehaviorTree.CPP ID.
pub fn btcpp_node_id(node_type: &str) -> &str {
    match node_type {
        "SEQUENCE" => "Sequence",
        "PARALLEL" => "Parallel",
        "FALLBACK" => "Fallback",
        "EXECUTE" => "ExecuteTrajectory",
        "HOME" => "MoveHome",
        "INTERLOCK_WAIT" => "WaitForRobot",
        "SAFETY_GATE" => "CheckSafety",
        other => other,
    }
}

/// Render a JSON port value as a BehaviorTree.CPP attribute string.
///
/// Structured values (trajectories, dependency lists, interlock targets) go
/// onto the blackboard and the port points at them, as a BehaviorTree.CPP
/// planner writes them.
pub fn port_text(
    port: &str,
    value: &Value,
    blackboard: &mut Map<String, Value>,
    counter: &mut usize,
) -> String {
    match value {
        Value::String(s) => s.clone(),
        Value::Null => String::new(),
        Value::Array(_) | Value::Object(_) => {
            *counter += 1;
            let key = format!("{port}_{counter}");
            blackboard.insert(key.clone(), value.clone());
            format!("{{{key}}}")
        }
        other => other.to_string(),
    }
}

/// Escape text for an XML attribute value.
pub fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Convert a JSON tree (`{type, name, ports, children}`, or the older dialect
/// with flat fields and inline `precomputed_trajectory`) into v4 XML.
///
/// Inline trajectories are moved onto the blackboard and referenced by key, as
/// a BehaviorTree.CPP planner would have written them.
pub fn json_tree_to_xml(
    root: &Value,
    blackboard: &mut Map<String, Value>,
) -> Result<String, String> {
    fn write(
        node: &Value,
        blackboard: &mut Map<String, Value>,
        depth: usize,
        counter: &mut usize,
        out: &mut String,
    ) -> Result<(), String> {
        let obj = node
            .as_object()
            .ok_or_else(|| "tree node is not an object".to_string())?;
        let node_type = obj
            .get("type")
            .and_then(Value::as_str)
            .ok_or_else(|| "tree node without `type`".to_string())?;
        let tag = btcpp_node_id(node_type);

        let mut attrs: Vec<(String, String)> = Vec::new();
        if let Some(name) = obj.get("name").and_then(Value::as_str) {
            attrs.push(("name".into(), name.into()));
        }
        if let Some(ports) = obj.get("ports").and_then(Value::as_object) {
            for (key, value) in ports {
                attrs.push((key.clone(), port_text(key, value, blackboard, counter)));
            }
        }
        // Older dialect: ports as flat fields. Only fields the node declares are
        // carried over, so planner metadata alongside them stays ignorable.
        for key in leaf_ports(tag) {
            if let Some(value) = obj.get(*key)
                && !attrs.iter().any(|(k, _)| k == key)
            {
                attrs.push((key.to_string(), port_text(key, value, blackboard, counter)));
            }
        }
        if let Some(traj) = obj
            .get("precomputed_trajectory")
            .filter(|t| t.as_array().is_some_and(|a| !a.is_empty()))
        {
            attrs.retain(|(k, _)| k != "trajectory");
            let reference = port_text("traj", traj, blackboard, counter);
            attrs.push(("trajectory".into(), reference));
        }

        let indent = "  ".repeat(depth);
        out.push_str(&indent);
        out.push('<');
        out.push_str(tag);
        for (key, value) in &attrs {
            out.push_str(&format!(" {key}=\"{}\"", xml_escape(value)));
        }
        let children = obj.get("children").and_then(Value::as_array);
        match children.filter(|c| !c.is_empty()) {
            None => out.push_str("/>\n"),
            Some(children) => {
                out.push_str(">\n");
                for child in children {
                    write(child, blackboard, depth + 1, counter, out)?;
                }
                out.push_str(&format!("{indent}</{tag}>\n"));
            }
        }
        Ok(())
    }

    // behaviortree-rs cannot parse a bare leaf as a tree's root; a one-child
    // Sequence is equivalent.
    let is_leaf = root
        .get("children")
        .and_then(Value::as_array)
        .is_none_or(|c| c.is_empty());
    let wrapped;
    let root = if is_leaf {
        wrapped = json!({ "type": "Sequence", "children": [root] });
        &wrapped
    } else {
        root
    };
    let mut body = String::new();
    let mut counter = 0usize;
    write(root, blackboard, 2, &mut counter, &mut body)?;
    Ok(format!(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n  \
         <BehaviorTree ID=\"MainTree\">\n{body}  </BehaviorTree>\n{}</root>\n",
        cell_leaves_model()
    ))
}

/// `<TreeNodesModel>` for the cell leaves: their ports as cell-runner declares them, with its
/// defaults, minus `name` (btgui's addition; cell-runner's converter writes none).
fn cell_leaves_model() -> String {
    let mut out = String::from("  <TreeNodesModel>\n");
    for id in crate::CELL_LEAVES {
        let tag = if id == "CheckSafety" {
            "Condition"
        } else {
            "Action"
        };
        out.push_str(&format!("    <{tag} ID=\"{id}\">\n"));
        for port in leaf_ports(id).iter().filter(|p| **p != "name") {
            match default_of(id, port) {
                Some(default) => out.push_str(&format!(
                    "      <input_port name=\"{port}\" default=\"{}\"/>\n",
                    xml_escape(default)
                )),
                None => out.push_str(&format!("      <input_port name=\"{port}\"/>\n")),
            }
        }
        out.push_str(&format!("    </{tag}>\n"));
    }
    out.push_str("  </TreeNodesModel>\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(name: &str) -> ParsedPlan {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
        let text = std::fs::read_to_string(format!("{path}{name}")).unwrap();
        parse_plan(&text).unwrap()
    }

    fn leaf(extra: Value) -> Value {
        let mut node = json!({ "type": "EXECUTE", "robot_ns": "r1", "operation_id": "A" });
        node.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        node
    }

    #[test]
    fn both_spellings_of_the_wrapper_are_accepted() {
        for wrapper in ["behaviour_tree", "behavior_tree"] {
            let plan = json!({ wrapper: { "tree_id": "inner", "cell_id": "c", "xml": "<root/>" } });
            let parsed = parse_plan_value(plan).unwrap();
            assert_eq!(parsed.tree.xml, "<root/>");
            // Ids fall back to the wrapper's.
            assert_eq!(parsed.tree_id.as_deref(), Some("inner"));
            assert_eq!(parsed.cell_id.as_deref(), Some("c"));
        }
    }

    #[test]
    fn top_level_ids_win_and_num_operations_is_top_level_only() {
        let plan = json!({
            "tree_id": "outer",
            "behaviour_tree": { "tree_id": "inner", "num_operations": 3, "xml": "<root/>" }
        });
        let parsed = parse_plan_value(plan).unwrap();
        assert_eq!(parsed.tree_id.as_deref(), Some("outer"));
        assert_eq!(parsed.cell_id, None);
        assert_eq!(parsed.num_operations, None);
        let plan = json!({ "num_operations": 3, "behaviour_tree": { "xml": "<root/>" } });
        assert_eq!(parse_plan_value(plan).unwrap().num_operations, Some(3));
    }

    #[test]
    fn errors_say_what_is_missing() {
        let err = |text: &str| parse_plan(text).unwrap_err();
        assert!(err("{").starts_with("invalid plan JSON: "), "{}", err("{"));
        assert_eq!(err("[1]"), "invalid plan object: expected a JSON object");
        assert_eq!(
            err(r#"{"behaviour_tree": {"blackboard": {}}}"#),
            "`behaviour_tree` has neither `xml` nor `root`"
        );
        assert_eq!(
            err(r#"{"tree_id": "t"}"#),
            "plan has neither `behaviour_tree` nor `root`"
        );
        assert_eq!(err(r#"{"root": 1}"#), "tree node is not an object");
        assert_eq!(
            err(r#"{"root": {"children": [{}]}}"#),
            "tree node without `type`"
        );
    }

    #[test]
    fn bare_xml_is_the_tree() {
        let parsed = parse_plan("  <root BTCPP_format=\"4\"/>").unwrap();
        assert!(is_xml(&parsed.tree.xml));
        assert!(parsed.tree.blackboard.is_empty());
        assert_eq!(parsed.tree_id, None);
    }

    #[test]
    fn a_byte_order_mark_is_ignored() {
        let parsed = parse_plan("\u{feff}<root BTCPP_format=\"4\"/>").unwrap();
        assert_eq!(parsed.tree.xml, "<root BTCPP_format=\"4\"/>");
        let json = r#"{"tree_id": "t", "behaviour_tree": {"xml": "<root/>"}}"#;
        let parsed = parse_plan(&format!("\u{feff}{json}")).unwrap();
        assert_eq!(parsed.tree_id.as_deref(), Some("t"));
        assert_eq!(parsed.tree.xml, "<root/>");
    }

    #[test]
    fn a_sibling_gives_the_blackboard_and_the_ids() {
        let bare = || parse_plan("<root/>").unwrap();
        let sibling = r#"{"tree_id": "t1", "cell_id": "c1", "num_operations": 2,
                          "behaviour_tree": {"xml": "<ignored/>", "blackboard": {"k": 1}}}"#;
        let mut parsed = bare();
        merge_sibling(&mut parsed, Some(sibling), Some("stem")).unwrap();
        assert_eq!(parsed.tree.xml, "<root/>");
        assert_eq!(parsed.tree.blackboard["k"], json!(1));
        assert_eq!(parsed.tree_id.as_deref(), Some("t1"));
        assert_eq!(parsed.cell_id.as_deref(), Some("c1"));
        assert_eq!(parsed.num_operations, Some(2));

        // Without ids of its own, the plan is named after its file.
        let mut parsed = bare();
        merge_sibling(
            &mut parsed,
            Some(r#"{"root": {"type": "Sequence"}}"#),
            Some("stem"),
        )
        .unwrap();
        assert_eq!(parsed.tree_id.as_deref(), Some("stem"));
        let mut parsed = bare();
        merge_sibling(&mut parsed, None, Some("stem")).unwrap();
        assert_eq!(parsed.tree_id.as_deref(), Some("stem"));

        // A sibling without a tree is not a plan; the bare XML stays usable.
        let mut parsed = bare();
        let err = merge_sibling(&mut parsed, Some(r#"{"blackboard": {}}"#), Some("stem"));
        assert_eq!(
            err.unwrap_err(),
            "plan has neither `behaviour_tree` nor `root`"
        );
        assert_eq!(parsed.tree_id.as_deref(), Some("stem"));
        assert!(parsed.tree.blackboard.is_empty());
    }

    #[test]
    fn a_bare_leaf_root_is_wrapped_in_a_sequence() {
        let mut blackboard = Map::new();
        let xml = json_tree_to_xml(&leaf(json!({})), &mut blackboard).unwrap();
        let body: Vec<&str> = xml.lines().skip(2).take(3).map(str::trim).collect();
        assert_eq!(
            body,
            [
                "<Sequence>",
                "<ExecuteTrajectory robot_ns=\"r1\" operation_id=\"A\"/>",
                "</Sequence>"
            ]
        );
    }

    #[test]
    fn attribute_values_are_escaped() {
        let mut blackboard = Map::new();
        let node = json!({ "type": "Sequence", "children": [
            { "type": "CheckSafety", "name": "say \"hi\" & <go>", "ports": { "condition": "a\"b" } }
        ]});
        let xml = json_tree_to_xml(&node, &mut blackboard).unwrap();
        assert!(
            xml.contains(
                r#"<CheckSafety name="say &quot;hi&quot; &amp; &lt;go&gt;" condition="a&quot;b"/>"#
            ),
            "{xml}"
        );
        assert!(bt_core::Document::parse(&xml).is_ok());
    }

    #[test]
    fn structured_ports_move_to_the_blackboard() {
        let mut blackboard = Map::new();
        let node = leaf(json!({
            "hold_duration_s": 0.0,
            "has_interlock": true,
            "interlock_target": { "joint_positions": [1.0] },
            "precomputed_trajectory": [{ "positions": [0.0], "time_s": 0.0 }],
            "planner_note": "ignored"
        }));
        let xml = json_tree_to_xml(&node, &mut blackboard).unwrap();
        assert!(xml.contains(r#"hold_duration_s="0.0""#), "{xml}");
        assert!(xml.contains(r#"has_interlock="true""#), "{xml}");
        assert!(xml.contains(r#"interlock_target="{interlock_target_1}""#));
        assert!(xml.contains(r#"trajectory="{traj_2}""#));
        assert!(!xml.contains("planner_note"));
        assert_eq!(
            blackboard.keys().collect::<Vec<_>>(),
            ["interlock_target_1", "traj_2"]
        );
    }

    #[test]
    fn the_converted_tree_declares_the_cell_leaves() {
        let mut blackboard = Map::new();
        let xml = json_tree_to_xml(&leaf(json!({})), &mut blackboard).unwrap();
        let doc = bt_core::Document::parse(&xml).unwrap();
        let model = &doc.models["ExecuteTrajectory"];
        assert!(!model.builtin);
        assert!(model.ports.iter().all(|p| p.name != "name"));
        let hold = model.ports.iter().find(|p| p.name == "hold_duration_s");
        assert_eq!(hold.unwrap().default.as_deref(), Some("0.5"));
        assert_eq!(doc.models["CheckSafety"].kind, bt_core::NodeKind::Condition);
        assert_eq!(doc.models["WaitForRobot"].ports.len(), 4);
    }

    #[test]
    fn samples_parse() {
        let welding = sample("bt_1790934549_welding_cell_2_small.json");
        assert_eq!(
            welding.tree_id.as_deref(),
            Some("bt_1790934549_welding_cell_2_small")
        );
        assert_eq!(welding.cell_id.as_deref(), Some("welding_cell_2_small"));
        assert_eq!(welding.num_operations, Some(16));
        assert_eq!(welding.tree.blackboard.len(), 18);
        assert!(welding.tree.xml.contains("<Parallel"));

        let two = sample("bt_1791274820_cell_2robot.json");
        assert_eq!(two.cell_id.as_deref(), Some("cell_2robot"));
        assert_eq!(two.num_operations, Some(1));
        assert_eq!(two.tree.blackboard.len(), 2);
        assert_eq!(two.tree.initial_blackboard().len(), 2);
    }
}
