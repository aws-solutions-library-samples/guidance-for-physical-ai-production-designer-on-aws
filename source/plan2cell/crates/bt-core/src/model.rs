//! BehaviorTree.CPP XML → a flat tree of nodes, as Groot builds it.
//!
//! Mirrors Groot's `ReadTreeNodesModel` (models from `<TreeNodesModel>` plus implicit ones) and
//! `BuildTreeFromXML` (model ID from `ID` or the tag, instance name from `name`, every other
//! attribute a port value), with BTCPP_format 4 additions: bare custom tags, `SubTree` port
//! remapping through `{key}`, `_autoremap`, and `_skipIf`-style conditions.
//!
//! Unlike Groot, a `SubTree` is expanded in place right away, so the [`Tree`] is the tree that
//! an executor runs; the view collapses it if asked.

use std::collections::{BTreeMap, HashMap};

use serde::{Deserialize, Serialize};

use crate::builtins;

/// A node's index in [`Tree::nodes`] (DFS pre-order, 0 = the synthetic Root).
pub type Uid = usize;

/// BehaviorTree.CPP's `NodeType`, plus Groot's synthetic Root.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NodeKind {
    Root,
    Action,
    Condition,
    Control,
    Decorator,
    SubTree,
}

impl NodeKind {
    /// The `<TreeNodesModel>` tag / Groot category name.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Root => "Root",
            Self::Action => "Action",
            Self::Condition => "Condition",
            Self::Control => "Control",
            Self::Decorator => "Decorator",
            Self::SubTree => "SubTree",
        }
    }

    fn from_tag(tag: &str) -> Option<Self> {
        Some(match tag {
            "Action" => Self::Action,
            "Condition" => Self::Condition,
            "Control" => Self::Control,
            "Decorator" => Self::Decorator,
            "SubTree" | "SubTreePlus" => Self::SubTree,
            _ => return None,
        })
    }

    pub fn is_leaf(self) -> bool {
        matches!(self, Self::Action | Self::Condition)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum PortDirection {
    Input,
    Output,
    InOut,
}

impl PortDirection {
    /// The prefix Groot shows before a port name.
    pub fn label(self) -> &'static str {
        match self {
            Self::Input => "IN",
            Self::Output => "OUT",
            Self::InOut => "INOUT",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PortModel {
    pub name: String,
    pub direction: PortDirection,
    pub type_name: Option<String>,
    pub default: Option<String>,
    pub description: String,
}

/// A registered node type.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeModel {
    pub id: String,
    pub kind: NodeKind,
    pub ports: Vec<PortModel>,
    /// One of BehaviorTree.CPP's own nodes.
    pub builtin: bool,
}

/// A port of one node instance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Port {
    pub name: String,
    pub direction: PortDirection,
    /// The attribute in the XML, if set.
    pub value: Option<String>,
    pub default: Option<String>,
    pub description: String,
    /// Declared by the node's model (rather than only appearing as an attribute).
    pub declared: bool,
}

impl Port {
    /// What the node reads: the attribute, else the model's default.
    pub fn effective(&self) -> Option<&str> {
        self.value.as_deref().or(self.default.as_deref())
    }

    /// The blackboard key if the value is a `{key}` pointer.
    pub fn blackboard_key(&self) -> Option<&str> {
        self.effective().and_then(blackboard_pointer)
    }
}

/// `{key}` → `key`.
pub fn blackboard_pointer(value: &str) -> Option<&str> {
    let inner = value.trim().strip_prefix('{')?.strip_suffix('}')?;
    (!inner.is_empty()).then_some(inner)
}

/// What a `SubTree` node refers to.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SubTreeRef {
    pub tree_id: String,
    pub autoremap: bool,
    /// v3 `<SubTree>`: a port value is the parent's key name, without braces.
    pub legacy_remap: bool,
    /// The referenced tree exists (and is expanded as this node's child).
    pub resolved: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Node {
    pub uid: Uid,
    pub parent: Option<Uid>,
    pub children: Vec<Uid>,
    /// Registration ID (the model).
    pub id: String,
    /// Instance name: the `name` attribute, else the ID.
    pub name: String,
    pub kind: NodeKind,
    /// Groot's order: inputs, outputs, in/outs, each alphabetical; then undeclared attributes.
    pub ports: Vec<Port>,
    /// `_skipIf`, `_onSuccess`, … in XML order.
    pub conditions: Vec<(String, String)>,
    /// The name of every attribute written in the XML, in order, reserved ones (`name`, `ID`,
    /// `_description`, …) included.
    #[serde(default)]
    pub attributes: Vec<String>,
    pub subtree: Option<SubTreeRef>,
    /// The element's tag as written (`Action` in v3's `<Action ID="…"/>`; empty for the Root).
    #[serde(default)]
    pub tag: String,
    /// Written with an end tag (`<X …></X>`, children or not) rather than as `<X …/>`.
    #[serde(default)]
    pub end_tag: bool,
    /// The `<BehaviorTree>` this node was written in.
    pub tree_id: String,
    pub depth: usize,
    /// Neither builtin nor in `<TreeNodesModel>`; the kind is inferred.
    pub unknown_model: bool,
    /// 1-based line in the XML (0 for the synthetic Root).
    pub line: u32,
}

impl Node {
    pub fn port(&self, name: &str) -> Option<&Port> {
        self.ports.iter().find(|p| p.name == name)
    }

    /// The effective value of a port.
    pub fn port_value(&self, name: &str) -> Option<&str> {
        self.port(name).and_then(Port::effective)
    }

    /// Groot shows the instance name under the caption, except for a SubTree named like its ID.
    pub fn shows_name(&self) -> bool {
        !(self.name == self.id && matches!(self.kind, NodeKind::SubTree | NodeKind::Root))
    }

    pub fn condition(&self, name: &str) -> Option<&str> {
        self.conditions
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }
}

/// One `<BehaviorTree>` instantiated, with every SubTree expanded.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Tree {
    pub tree_id: String,
    pub nodes: Vec<Node>,
    pub format: u32,
    pub warnings: Vec<String>,
}

impl Tree {
    pub fn root(&self) -> &Node {
        &self.nodes[0]
    }

    pub fn node(&self, uid: Uid) -> &Node {
        &self.nodes[uid]
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Whether `ancestor` is `uid` or above it.
    pub fn is_ancestor(&self, ancestor: Uid, mut uid: Uid) -> bool {
        loop {
            if uid == ancestor {
                return true;
            }
            match self.nodes[uid].parent {
                Some(parent) => uid = parent,
                None => return false,
            }
        }
    }
}

/// An element of a `<BehaviorTree>`, before instantiation.
#[derive(Clone, Debug)]
struct Element {
    tag: String,
    attrs: Vec<(String, String)>,
    children: Vec<Element>,
    /// Not self-closing.
    end_tag: bool,
    line: u32,
}

impl Element {
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }
}

/// How BehaviorTree.CPP files are read: a `<!DOCTYPE>` is allowed, as tinyxml2 allows it.
pub const XML_OPTIONS: roxmltree::ParsingOptions = roxmltree::ParsingOptions {
    allow_dtd: true,
    nodes_limit: u32::MAX,
};

/// A parsed BehaviorTree.CPP XML file.
#[derive(Clone, Debug)]
pub struct Document {
    /// `BTCPP_format` (3 when absent, as Groot assumes).
    pub format: u32,
    /// `main_tree_to_execute`, if given.
    pub main_tree: Option<String>,
    /// `<BehaviorTree ID>`s in file order.
    pub tree_ids: Vec<String>,
    /// Builtins, `<TreeNodesModel>` entries and implicit models of unknown nodes.
    pub models: BTreeMap<String, NodeModel>,
    pub warnings: Vec<String>,
    /// `None` for an empty `<BehaviorTree/>` (Groot shows just its Root).
    trees: HashMap<String, Option<Element>>,
}

impl Document {
    pub fn parse(xml: &str) -> Result<Self, String> {
        let doc = roxmltree::Document::parse_with_options(xml, XML_OPTIONS)
            .map_err(|e| format!("invalid XML: {e}"))?;
        let root = doc.root_element();
        if root.tag_name().name() != "root" {
            return Err(format!(
                "the top element must be <root>, not <{}>",
                root.tag_name().name()
            ));
        }
        let format = match root.attribute("BTCPP_format") {
            None => 3,
            Some(v) => v
                .trim()
                .parse()
                .map_err(|_| format!("BTCPP_format=\"{v}\" is not a number"))?,
        };
        let mut document = Document {
            format,
            main_tree: root.attribute("main_tree_to_execute").map(str::to_string),
            tree_ids: Vec::new(),
            models: BTreeMap::new(),
            warnings: Vec::new(),
            trees: HashMap::new(),
        };
        for id in builtins::ids() {
            if let Some(model) = builtins::model(id) {
                document.models.insert(id.to_string(), model);
            }
        }

        for child in root.children().filter(roxmltree::Node::is_element) {
            match child.tag_name().name() {
                "BehaviorTree" => document.read_behavior_tree(&doc, child)?,
                "TreeNodesModel" => document.read_tree_nodes_model(child),
                "include" => document.warnings.push(format!(
                    "<include path=\"{}\"> is not followed",
                    child.attribute("path").unwrap_or("")
                )),
                other => document
                    .warnings
                    .push(format!("ignoring <{other}> under <root>")),
            }
        }
        if document.tree_ids.is_empty() {
            return Err("no <BehaviorTree> in the file".to_string());
        }
        if let Some(main) = &document.main_tree
            && !document.trees.contains_key(main)
        {
            document.warnings.push(format!(
                "main_tree_to_execute=\"{main}\" does not exist; using \"{}\"",
                document.tree_ids[0]
            ));
            document.main_tree = None;
        }
        document.add_implicit_models();
        Ok(document)
    }

    /// The tree to show and run by default.
    pub fn main_tree_id(&self) -> &str {
        self.main_tree.as_deref().unwrap_or(&self.tree_ids[0])
    }

    fn read_behavior_tree(
        &mut self,
        doc: &roxmltree::Document<'_>,
        element: roxmltree::Node<'_, '_>,
    ) -> Result<(), String> {
        let id = element
            .attribute("ID")
            .unwrap_or("BehaviorTree")
            .to_string();
        let mut body: Vec<_> = element
            .children()
            .filter(roxmltree::Node::is_element)
            .collect();
        // Groot (and BT.CPP v3) skip a legacy <Root> wrapper.
        if body.len() == 1 && body[0].tag_name().name() == "Root" {
            self.warnings.push(format!(
                "BehaviorTree \"{id}\": skipping the legacy <Root> element"
            ));
            body = body[0]
                .children()
                .filter(roxmltree::Node::is_element)
                .collect();
        }
        let first = match body.as_slice() {
            [] => {
                self.warnings
                    .push(format!("BehaviorTree \"{id}\" is empty"));
                None
            }
            [only] => Some(*only),
            [first, ..] => {
                self.warnings.push(format!(
                    "BehaviorTree \"{id}\" has {} root nodes; only the first is used",
                    body.len()
                ));
                Some(*first)
            }
        };
        if self.trees.contains_key(&id) {
            return Err(format!("BehaviorTree \"{id}\" is defined twice"));
        }
        self.tree_ids.push(id.clone());
        self.trees
            .insert(id, first.map(|first| read_element(doc, first)));
        Ok(())
    }

    fn read_tree_nodes_model(&mut self, element: roxmltree::Node<'_, '_>) {
        for entry in element.children().filter(roxmltree::Node::is_element) {
            let tag = entry.tag_name().name();
            let Some(kind) = NodeKind::from_tag(tag) else {
                continue; // e.g. v4 <MetadataFields>
            };
            let Some(id) = entry.attribute("ID") else {
                self.warnings
                    .push(format!("<TreeNodesModel>: <{tag}> without ID"));
                continue;
            };
            let mut ports = Vec::new();
            for port in entry.children().filter(roxmltree::Node::is_element) {
                let direction = match port.tag_name().name() {
                    "input_port" => PortDirection::Input,
                    "output_port" => PortDirection::Output,
                    "inout_port" => PortDirection::InOut,
                    _ => continue,
                };
                let Some(name) = port.attribute("name") else {
                    continue;
                };
                ports.push(PortModel {
                    name: name.to_string(),
                    direction,
                    type_name: port.attribute("type").map(str::to_string),
                    default: port.attribute("default").map(str::to_string),
                    description: port.text().unwrap_or("").trim().to_string(),
                });
            }
            // Groot lets the file override builtins of the same ID.
            self.models.insert(
                id.to_string(),
                NodeModel {
                    id: id.to_string(),
                    kind,
                    ports,
                    builtin: false,
                },
            );
        }
    }

    /// Groot's implicit models: a node nobody declared gets every attribute as an INOUT port.
    fn add_implicit_models(&mut self) {
        let mut found: Vec<(String, Option<NodeKind>, Vec<String>)> = Vec::new();
        fn walk(
            document: &Document,
            element: &Element,
            found: &mut Vec<(String, Option<NodeKind>, Vec<String>)>,
        ) {
            let (id, kind) = classify(document, element);
            if !document.models.contains_key(&id) && !found.iter().any(|(f, ..)| *f == id) {
                let ports = element
                    .attrs
                    .iter()
                    .map(|(k, _)| k.clone())
                    .filter(|k| !builtins::is_reserved_attribute(k) && !k.starts_with('_'))
                    .collect();
                found.push((id, kind, ports));
            }
            for child in &element.children {
                walk(document, child, found);
            }
        }
        for element in self.trees.values().flatten() {
            walk(self, element, &mut found);
        }
        for (id, kind, ports) in found {
            let Some(kind) = kind else { continue };
            self.models.insert(
                id.clone(),
                NodeModel {
                    id,
                    kind,
                    ports: ports
                        .into_iter()
                        .map(|name| PortModel {
                            name,
                            direction: PortDirection::InOut,
                            type_name: None,
                            default: None,
                            description: String::new(),
                        })
                        .collect(),
                    builtin: false,
                },
            );
        }
    }

    /// Instantiate a `<BehaviorTree>` with Groot's synthetic Root on top.
    pub fn instantiate(&self, tree_id: &str) -> Result<Tree, String> {
        let element = self
            .trees
            .get(tree_id)
            .ok_or_else(|| format!("no BehaviorTree \"{tree_id}\""))?;
        let mut tree = Tree {
            tree_id: tree_id.to_string(),
            nodes: Vec::new(),
            format: self.format,
            warnings: self.warnings.clone(),
        };
        let root_model = builtins::model(builtins::ROOT_ID).expect("Root is builtin");
        tree.nodes.push(Node {
            uid: 0,
            parent: None,
            children: Vec::new(),
            id: root_model.id.clone(),
            name: root_model.id.clone(),
            kind: NodeKind::Root,
            ports: Vec::new(),
            conditions: Vec::new(),
            attributes: Vec::new(),
            subtree: None,
            tag: String::new(),
            end_tag: false,
            tree_id: tree_id.to_string(),
            depth: 0,
            unknown_model: false,
            line: 0,
        });
        let mut stack = vec![tree_id.to_string()];
        if let Some(element) = element {
            self.add_element(&mut tree, element, 0, tree_id, &mut stack);
        }
        Ok(tree)
    }

    fn add_element(
        &self,
        tree: &mut Tree,
        element: &Element,
        parent: Uid,
        tree_id: &str,
        stack: &mut Vec<String>,
    ) -> Uid {
        let (id, kind) = classify(self, element);
        let unknown_model = kind.is_none();
        let kind = kind.unwrap_or(match element.children.len() {
            0 => NodeKind::Action,
            1 => NodeKind::Decorator,
            _ => NodeKind::Control,
        });
        if unknown_model {
            tree.warnings.push(format!(
                "line {}: <{}> is not a builtin and not in <TreeNodesModel>; shown as {}",
                element.line,
                element.tag,
                kind.as_str()
            ));
        }
        let model = self.models.get(&id);

        let mut ports: Vec<Port> = Vec::new();
        if let Some(model) = model {
            let mut declared: Vec<&PortModel> = model.ports.iter().collect();
            declared.sort_by(|a, b| a.direction.cmp(&b.direction).then(a.name.cmp(&b.name)));
            for p in declared {
                ports.push(Port {
                    name: p.name.clone(),
                    direction: p.direction,
                    value: element.attr(&p.name).map(str::to_string),
                    default: p.default.clone(),
                    description: p.description.clone(),
                    declared: true,
                });
            }
            // Builtins list v3 and v4 spellings; only show the ones in use or v4's.
            if model.builtin {
                ports.retain(|p| {
                    p.value.is_some() || p.default.is_some() || !is_v3_alias(&id, &p.name)
                });
            }
        }
        let mut conditions = Vec::new();
        for (key, value) in &element.attrs {
            if builtins::PRECONDITIONS.contains(&key.as_str())
                || builtins::POSTCONDITIONS.contains(&key.as_str())
            {
                conditions.push((key.clone(), value.clone()));
            } else if !builtins::is_reserved_attribute(key) && !ports.iter().any(|p| p.name == *key)
            {
                ports.push(Port {
                    name: key.clone(),
                    direction: PortDirection::InOut,
                    value: Some(value.clone()),
                    default: None,
                    description: String::new(),
                    declared: false,
                });
            }
        }

        let subtree = (kind == NodeKind::SubTree).then(|| SubTreeRef {
            tree_id: id.clone(),
            autoremap: element
                .attr("_autoremap")
                .is_some_and(|v| matches!(v.trim(), "true" | "1" | "True" | "TRUE")),
            legacy_remap: self.format < 4 && element.tag == "SubTree",
            resolved: self.trees.contains_key(&id),
        });

        let uid = tree.nodes.len();
        let depth = tree.nodes[parent].depth + 1;
        tree.nodes.push(Node {
            uid,
            parent: Some(parent),
            children: Vec::new(),
            name: element
                .attr("name")
                .map_or_else(|| id.clone(), str::to_string),
            id: id.clone(),
            kind,
            ports,
            conditions,
            attributes: element.attrs.iter().map(|(k, _)| k.clone()).collect(),
            subtree,
            tag: element.tag.clone(),
            end_tag: element.end_tag,
            tree_id: tree_id.to_string(),
            depth,
            unknown_model,
            line: element.line,
        });
        tree.nodes[parent].children.push(uid);

        if kind == NodeKind::SubTree {
            if !element.children.is_empty() {
                tree.warnings.push(format!(
                    "line {}: SubTree \"{id}\" has child elements; they are ignored",
                    element.line
                ));
            }
            match self.trees.get(&id) {
                None => tree.warnings.push(format!(
                    "line {}: SubTree \"{id}\" refers to a BehaviorTree that does not exist",
                    element.line
                )),
                Some(_) if stack.contains(&id) => tree.warnings.push(format!(
                    "line {}: SubTree \"{id}\" includes itself ({} → {id}); not expanded",
                    element.line,
                    stack.join(" → ")
                )),
                Some(None) => {}
                Some(Some(body)) => {
                    stack.push(id.clone());
                    self.add_element(tree, body, uid, &id, stack);
                    stack.pop();
                }
            }
        } else {
            for child in &element.children {
                self.add_element(tree, child, uid, tree_id, stack);
            }
            let n = element.children.len();
            let expected = match kind {
                NodeKind::Action | NodeKind::Condition if n > 0 => {
                    Some(format!("{} must not have children", kind.as_str()))
                }
                NodeKind::Decorator if n != 1 => {
                    Some("a Decorator needs exactly one child".to_string())
                }
                NodeKind::Control if n == 0 => Some("a Control node needs children".to_string()),
                _ => None,
            };
            if let Some(problem) = expected {
                tree.warnings.push(format!(
                    "line {}: <{}>: {problem}",
                    element.line, element.tag
                ));
            }
        }
        uid
    }
}

/// v3 port names that builtins also accept but that v4 trees do not use.
fn is_v3_alias(id: &str, port: &str) -> bool {
    id == "Parallel" && matches!(port, "success_threshold" | "failure_threshold")
}

/// Model ID and kind of an element; `None` kind when nothing declares it.
fn classify(document: &Document, element: &Element) -> (String, Option<NodeKind>) {
    if let Some(kind) = NodeKind::from_tag(&element.tag) {
        let id = element.attr("ID").unwrap_or(&element.tag).to_string();
        return (id, Some(kind));
    }
    let id = element.tag.clone();
    let kind = document.models.get(&id).map(|m| m.kind);
    (id, kind)
}

fn read_element(doc: &roxmltree::Document<'_>, node: roxmltree::Node<'_, '_>) -> Element {
    Element {
        tag: node.tag_name().name().to_string(),
        attrs: node
            .attributes()
            .map(|a| (a.name().to_string(), a.value().to_string()))
            .collect(),
        children: node
            .children()
            .filter(roxmltree::Node::is_element)
            .map(|c| read_element(doc, c))
            .collect(),
        end_tag: !doc.input_text()[node.range()].ends_with("/>"),
        line: doc.text_pos_at(node.range().start).row,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_data(name: &str) -> String {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
        std::fs::read_to_string(format!("{path}{name}")).unwrap()
    }

    #[test]
    fn every_sample_parses_and_instantiates() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data");
        let mut count = 0;
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().is_some_and(|e| e == "xml") {
                let xml = std::fs::read_to_string(&path).unwrap();
                if !xml.contains("<BehaviorTree") {
                    continue; // a node palette (crossdoor_palette.xml)
                }
                let doc =
                    Document::parse(&xml).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
                for id in &doc.tree_ids {
                    let tree = doc.instantiate(id).unwrap();
                    assert!(!tree.is_empty(), "{}: {id}", path.display());
                }
                count += 1;
            }
        }
        assert!(count >= 10);
    }

    #[test]
    fn crossdoor_expands_its_subtree() {
        let doc = Document::parse(&test_data("crossdoor_with_subtree.xml")).unwrap();
        assert_eq!(doc.main_tree_id(), "MainTree");
        let tree = doc.instantiate("MainTree").unwrap();
        let ids: Vec<&str> = tree.nodes.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(
            ids,
            [
                "Root",
                "Fallback",
                "Sequence",
                "IsDoorOpen",
                "PassThroughDoor",
                "DoorClosed",
                "Sequence",
                "Inverter",
                "IsDoorOpen",
                "RetryUntilSuccessful",
                "OpenDoor",
                "PassThroughDoor",
                "CloseDoor",
                "PassThroughWindow",
            ]
        );
        let names: Vec<&str> = tree.nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(names[1], "root_Fallback");
        assert_eq!(tree.nodes[5].kind, NodeKind::SubTree);
        assert_eq!(tree.nodes[6].tree_id, "DoorClosed");
        assert_eq!(tree.nodes[3].kind, NodeKind::Condition);
        assert_eq!(tree.nodes[9].port_value("num_attempts"), Some("4"));
        assert!(tree.warnings.is_empty(), "{:?}", tree.warnings);
    }

    #[test]
    fn v4_bare_tags_and_subtree_remaps() {
        let xml = r#"<root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <Script code="msg := 'hi'"/>
              <SubTree ID="Say" text="{msg}" _autoremap="true" _skipIf="done"/>
              <MyAction speed="2"/>
            </Sequence>
          </BehaviorTree>
          <BehaviorTree ID="Say"><SaySomething message="{text}"/></BehaviorTree>
          <TreeNodesModel>
            <Action ID="SaySomething"><input_port name="message" default="x">what to say</input_port></Action>
          </TreeNodesModel>
        </root>"#;
        let doc = Document::parse(xml).unwrap();
        let tree = doc.instantiate("Main").unwrap();
        let subtree = &tree.nodes[3];
        assert_eq!(subtree.kind, NodeKind::SubTree);
        let r = subtree.subtree.as_ref().unwrap();
        assert!(r.autoremap && !r.legacy_remap && r.resolved);
        assert_eq!(
            subtree.conditions,
            [("_skipIf".to_string(), "done".to_string())]
        );
        assert_eq!(subtree.port("text").unwrap().blackboard_key(), Some("msg"));
        let say = &tree.nodes[4];
        assert_eq!(say.id, "SaySomething");
        assert_eq!(say.parent, Some(3));
        assert_eq!(say.port("message").unwrap().description, "what to say");
        let unknown = &tree.nodes[5];
        assert!(unknown.unknown_model);
        assert_eq!(unknown.kind, NodeKind::Action);
        assert_eq!(unknown.port_value("speed"), Some("2"));
        // Reserved attributes are not ports, but what was written is kept.
        assert_eq!(subtree.attributes, ["ID", "text", "_autoremap", "_skipIf"]);
        // So are the tag and whether it is self-closing.
        let tags: Vec<(&str, bool)> = tree
            .nodes
            .iter()
            .map(|n| (n.tag.as_str(), n.end_tag))
            .collect();
        assert_eq!(
            tags,
            [
                ("", false),
                ("Sequence", true),
                ("Script", false),
                ("SubTree", false),
                ("SaySomething", false),
                ("MyAction", false),
            ]
        );
        let v3 = r#"<root><BehaviorTree ID="T"><Action ID="A" ></Action></BehaviorTree></root>"#;
        let tree = Document::parse(v3).unwrap().instantiate("T").unwrap();
        assert_eq!(
            (tree.nodes[1].tag.as_str(), tree.nodes[1].end_tag),
            ("Action", true)
        );
        assert_eq!(tree.nodes[1].id, "A");
    }

    #[test]
    fn a_name_equal_to_the_id_is_still_written() {
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="T">
            <Sequence name="Sequence"><AlwaysSuccess _description="d"/></Sequence>
          </BehaviorTree></root>"#;
        let tree = Document::parse(xml).unwrap().instantiate("T").unwrap();
        let (sequence, leaf) = (&tree.nodes[1], &tree.nodes[2]);
        assert_eq!(sequence.name, sequence.id);
        assert_eq!(sequence.attributes, ["name"]);
        assert_eq!(leaf.name, "AlwaysSuccess");
        assert_eq!(leaf.attributes, ["_description"]);
        assert!(leaf.ports.is_empty());
        assert!(tree.root().attributes.is_empty());
    }

    #[test]
    fn ports_follow_groot_order() {
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="T">
            <A z="1" a="2" o="{x}"/></BehaviorTree>
          <TreeNodesModel><Action ID="A">
            <output_port name="o"/><input_port name="z"/><input_port name="a"/><inout_port name="b"/>
          </Action></TreeNodesModel></root>"#;
        let tree = Document::parse(xml).unwrap().instantiate("T").unwrap();
        let names: Vec<&str> = tree.nodes[1]
            .ports
            .iter()
            .map(|p| p.name.as_str())
            .collect();
        assert_eq!(names, ["a", "z", "o", "b"]);
    }

    #[test]
    fn recursive_subtree_is_not_expanded_forever() {
        let xml = r#"<root BTCPP_format="4"><BehaviorTree ID="T"><Sequence><SubTree ID="T"/></Sequence></BehaviorTree></root>"#;
        let tree = Document::parse(xml).unwrap().instantiate("T").unwrap();
        assert_eq!(tree.len(), 3);
        assert!(tree.warnings.iter().any(|w| w.contains("includes itself")));
    }
}
