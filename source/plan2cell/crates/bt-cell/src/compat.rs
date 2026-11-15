//! What cell-runner would reject: trees btgui runs, but behaviortree-rs (cell-runner's
//! runtime) refuses to build or to tick.
//!
//! | rule | rejected | behaviortree-rs |
//! |------|----------|-----------------|
//! | R0 | anything but the XML declaration before `<root>` (a DOCTYPE too); under it, anything but `<BehaviorTree ID>` and `<TreeNodesModel>` elements with end tags; a `<BehaviorTree>` that is not one element with an end tag (a bare leaf, nothing, or two); several BehaviorTrees and no existing main one | `Factory::register_bt_from_text` and `create_sync_tree_from_text` (`tree.rs:301-321`, `:681-807`); [`check_xml`] |
//! | R1 | a tag it does not register (v3's `<Action ID="…"/>` included) | `Factory` builtins (`tree.rs:830-1011`) plus the cell leaves and `Delay` |
//! | R2 | an attribute the node does not declare: `name` on control nodes (even when it equals the ID), `ID`, `_description` and the like included | `add_ports_to_node` (`tree.rs:430-480`) |
//! | R3 | `_skipIf`-style conditions | not supported: they are undeclared attributes too |
//! | R4 | a `ReactiveSequence` with more than one child that can be RUNNING | fails the tick in which a second child returns RUNNING: "Only a single child can return Running" (`reactive_sequence.rs`); BT.CPP and bt-exec restart the earlier child instead |
//! | R5 | a `SubTree` whose BehaviorTree does not exist or that has an end tag, a decorator without exactly one child, a control node or decorator written as `<X/>`, a cell leaf written with an end tag | `recursively_build_subtree` and `build_child` (`tree.rs:272-299`, `:482-679`); bt-core only warns |
//! | R6 | a `Parallel` whose `success_count` or `failure_count` needs more children than it has | fails every tick of it: "Number of children is less than the threshold" (`parallel.rs`) |
//!
//! [`check_xml`] finds R0 in the file; [`check`] lists the rest in the order behaviortree-rs runs
//! into them, building the tree depth first: a node's tag before its children, its attributes
//! after them. behaviortree-rs stops at the first, which [`compile`](crate::compile::compile)
//! reports with cell-runner's text; R4 and R6 only fail once the run gets there
//! ([`CompatIssue::fails_at`]).
//!
//! The synthetic root is skipped, and `SubTree` nodes are exempt from R1-R4 (behaviortree-rs
//! expands them, and skips their remapping attributes).
//!
//! Not checked: a Sequence whose children are all skipped, literal values on a `SubTree`, a
//! Parallel threshold read from the blackboard, and a SubTree that includes itself.

use bt_core::model::{Node, NodeKind, Tree, Uid};
use bt_core::{NodeStatus, Trace};

use crate::CELL_LEAVES;
use crate::ports::leaf_ports;

/// The control nodes behaviortree-rs 0.2.5 registers itself.
const CONTROLS: &[&str] = &[
    "Sequence",
    "ReactiveSequence",
    "SequenceStar",
    "Parallel",
    "ParallelAll",
    "Fallback",
    "ReactiveFallback",
    "IfThenElse",
    "WhileDoElse",
];

/// Its decorators, and cell-runner's `Delay`.
const DECORATORS: &[&str] = &[
    "ForceFailure",
    "ForceSuccess",
    "Inverter",
    "KeepRunningUntilFailure",
    "Repeat",
    "Retry",
    "RunOnce",
    "Delay",
];

/// Nodes that can return RUNNING under cell-runner, whatever their children do.
const ASYNC: &[&str] = &[
    "ExecuteTrajectory",
    "MoveHome",
    "WaitForRobot",
    "Delay",
    "KeepRunningUntilFailure",
];

/// behaviortree-rs's `ParseError` and `NodeError` texts, verbatim.
const CHILDREN_MISMATCH: &str = "Node type [] didn't had invalid presence/absence of children.";
const MISSING_ROOT: &str =
    "Expected to find <root> start tag at start of XML. Found incorrect tag.";
const NO_ID: &str = "Found BehaviorTree definition without ID. Cannot continue parsing.";
const ONE_CHILD: &str = "Violated node type constraint: BehaviorTree node may only have one child";
const END_TAG: &str = "Violated node type constraint: Expected end tag for BehaviorTree";
const NO_MAIN_TREE: &str =
    "No main tree was provided, either in the XML or as a function parameter.";
const INTERNAL: &str =
    "Errors like this shouldn't happen. Something bad has happened. Please report this.";
const SINGLE_RUNNING: &str = "[ReactiveSequence]: Only a single child can return Running.";

/// When behaviortree-rs gives up on a tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fails {
    /// Building it: cell-runner says "invalid BehaviorTree: …".
    Build,
    /// Ticking the node, whenever it is ticked: "BehaviorTree tick failed at t=…: …".
    Tick,
    /// Ticking the `ReactiveSequence`, once a child other than the first one that ran returns
    /// RUNNING (it never forgets which one that was).
    SecondRunning,
}

/// One reason cell-runner would not run a tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompatIssue {
    /// The node it is about (0, the root, for the file around the trees).
    pub uid: Uid,
    /// 1-based line in the XML.
    pub line: u32,
    /// behaviortree-rs's error text, verbatim.
    pub message: String,
    /// What btgui adds where that text alone would not say enough.
    pub note: Option<String>,
    /// When behaviortree-rs runs into it.
    pub fails: Fails,
}

impl CompatIssue {
    fn new(node: &Node, message: impl Into<String>, note: Option<String>, fails: Fails) -> Self {
        Self {
            uid: node.uid,
            line: node.line,
            message: message.into(),
            note,
            fails,
        }
    }

    /// The message, with the note if there is one.
    pub fn text(&self) -> String {
        match &self.note {
            Some(note) => format!("{} ({note})", self.message),
            None => self.message.clone(),
        }
    }

    /// The tick in which a run with `trace` (of the tree it was checked in) reaches it, where
    /// behaviortree-rs would have failed. `None` for a build issue, and for one the run never
    /// reached.
    pub fn fails_at(&self, tree: &Tree, trace: &Trace) -> Option<u64> {
        match self.fails {
            Fails::Build => None,
            Fails::Tick => trace.of_node(self.uid).map(|t| t.tick).next(),
            Fails::SecondRunning => {
                // The first tick each child returned RUNNING in: it ended that tick RUNNING (a
                // control node is RUNNING while it ticks its children, whatever it returns).
                let mut firsts: Vec<u64> = tree
                    .node(self.uid)
                    .children
                    .iter()
                    .filter_map(|&child| {
                        let transitions: Vec<_> = trace.of_node(child).collect();
                        transitions
                            .iter()
                            .enumerate()
                            .filter(|(i, t)| {
                                transitions
                                    .get(i + 1)
                                    .is_none_or(|next| next.tick != t.tick)
                            })
                            .find(|(_, t)| t.status == NodeStatus::Running)
                            .map(|(_, t)| t.tick)
                    })
                    .collect();
                firsts.sort_unstable();
                firsts.get(1).copied()
            }
        }
    }
}

/// The ports behaviortree-rs declares for `id` (cell-runner's own nodes included).
fn declared(id: &str) -> &'static [&'static str] {
    match id {
        "Parallel" => &["success_count", "failure_count"],
        "ParallelAll" => &["max_failures"],
        "Repeat" => &["num_cycles"],
        "Retry" => &["num_attempts"],
        "RunOnce" => &["then_skip"],
        "Delay" => &["delay_msec"],
        other => leaf_ports(other),
    }
}

/// What behaviortree-rs registers a tag as.
fn category(tag: &str) -> Option<NodeKind> {
    if CONTROLS.contains(&tag) {
        Some(NodeKind::Control)
    } else if DECORATORS.contains(&tag) {
        Some(NodeKind::Decorator)
    } else if CELL_LEAVES.contains(&tag) {
        Some(NodeKind::Action)
    } else {
        None
    }
}

fn unregistered(tag: &str) -> String {
    format!("Attempted to parse node with unregistered name: {tag}")
}

/// Every issue in `tree`, in the order behaviortree-rs runs into them (see the module docs):
/// what fails the build, then what fails a tick (R4, R6). The file around the tree is
/// [`check_xml`]'s.
pub fn check(tree: &Tree) -> Vec<CompatIssue> {
    let mut issues = Vec::new();
    for &child in &tree.root().children {
        build(tree, child, &mut issues);
    }
    for node in &tree.nodes {
        if node.kind == NodeKind::Root || node.kind == NodeKind::SubTree {
            continue;
        }
        match node.tag.as_str() {
            "ReactiveSequence" => {
                let running = node
                    .children
                    .iter()
                    .filter(|&&child| can_run(tree, child))
                    .count();
                if running > 1 {
                    issues.push(CompatIssue::new(
                        node,
                        SINGLE_RUNNING,
                        Some("BT.CPP and btgui restart the earlier child instead".to_string()),
                        Fails::SecondRunning,
                    ));
                }
            }
            "Parallel" if node.end_tag => {
                if let Some((message, note)) = parallel_threshold(node) {
                    issues.push(CompatIssue::new(
                        node,
                        message,
                        Some(note.to_string()),
                        Fails::Tick,
                    ));
                }
            }
            _ => {}
        }
    }
    issues
}

/// behaviortree-rs's `build_child` for `uid`, and what is below it.
fn build(tree: &Tree, uid: Uid, issues: &mut Vec<CompatIssue>) {
    let node = tree.node(uid);
    let mut issue = |message: String, note: Option<String>| {
        issues.push(CompatIssue::new(node, message, note, Fails::Build));
    };
    let n = node.children.len();
    if node.kind == NodeKind::SubTree {
        // Only `<SubTree ID="…"/>` is expanded; anything else is looked up like a node.
        if node.tag != "SubTree" || node.end_tag {
            issue(unregistered(&node.tag), None);
        } else if node.subtree.as_ref().is_some_and(|s| !s.resolved) {
            issue(format!("Can't find tree [{}]", node.id), None);
        }
    } else {
        match category(&node.tag) {
            None => issue(unregistered(&node.tag), None),
            Some(NodeKind::Action) if node.end_tag => {
                let note = if n > 0 {
                    format!("[{}] must not have children", node.tag)
                } else {
                    format!("[{}] has an end tag; write it as <{}/>", node.tag, node.tag)
                };
                issue(CHILDREN_MISMATCH.to_string(), Some(note));
            }
            Some(NodeKind::Control) if !node.end_tag => issue(
                CHILDREN_MISMATCH.to_string(),
                Some(format!("[{}] needs children", node.tag)),
            ),
            Some(NodeKind::Decorator) if !node.end_tag || n == 0 => issue(
                CHILDREN_MISMATCH.to_string(),
                Some(format!("[{}] needs a child", node.tag)),
            ),
            _ => {}
        }
    }
    // A decorator builds its child, then expects its end tag.
    let decorator = category(&node.tag) == Some(NodeKind::Decorator) && node.end_tag;
    for (i, &child) in node.children.iter().enumerate() {
        if decorator && i == 1 {
            issues.push(CompatIssue::new(
                node,
                format!(
                    "Violated node type constraint: Decorator node [{}] may only have one child",
                    node.tag
                ),
                None,
                Fails::Build,
            ));
        }
        build(tree, child, issues);
    }
    if node.kind != NodeKind::SubTree && category(&node.tag).is_some() {
        undeclared(node, issues);
    }
}

/// R2 and R3: the attributes `node` does not declare.
fn undeclared(node: &Node, issues: &mut Vec<CompatIssue>) {
    let ports = declared(&node.tag);
    let invalid = |port: &str| {
        format!(
            "Port name [{port}] did not match Node [{}] port list: {ports:?}",
            node.tag
        )
    };
    // Every attribute written (bt-core keeps `name`, `ID` and `_description`-like ones out of
    // the ports) and every port set, which an edit can add.
    let is_condition = |name: &str| node.conditions.iter().any(|(c, _)| c == name);
    let set = node.ports.iter().filter(|p| p.value.is_some());
    let mut names: Vec<&str> = Vec::new();
    for name in node
        .attributes
        .iter()
        .map(String::as_str)
        .chain(set.map(|p| p.name.as_str()))
    {
        if !ports.contains(&name) && !is_condition(name) && !names.contains(&name) {
            names.push(name);
        }
    }
    for name in names {
        issues.push(CompatIssue::new(node, invalid(name), None, Fails::Build));
    }
    for (condition, _) in &node.conditions {
        issues.push(CompatIssue::new(
            node,
            invalid(condition),
            Some(format!("behaviortree-rs has no {condition} conditions")),
            Fails::Build,
        ));
    }
}

/// R6: what behaviortree-rs's `Parallel` fails every tick with, for literal thresholds (its own
/// defaults when unset: `success_count` -1, `failure_count` 1), and what btgui's does instead.
fn parallel_threshold(node: &Node) -> Option<(&'static str, &'static str)> {
    let threshold =
        |port: &str, default: i64| match node.port(port).and_then(|p| p.value.as_deref()) {
            None => Some(default),
            Some(text) => text.parse::<i64>().ok(),
        };
    let n = node.children.len() as i64;
    let required = |t: i64| if t < 0 { (n + t + 1).max(0) } else { t };
    if n < required(threshold("success_count", -1)?) {
        return Some((
            "Number of children is less than the threshold. Can never succeed.",
            "btgui's Parallel fails instead",
        ));
    }
    if n < required(threshold("failure_count", 1)?) {
        let instead = if n == 0 {
            "btgui's Parallel fails instead"
        } else {
            "btgui's Parallel runs, and cannot fail"
        };
        return Some((
            "Number of children is less than the threshold. Can never fail.",
            instead,
        ));
    }
    None
}

/// Whether anything at or below `uid` can be RUNNING.
fn can_run(tree: &Tree, uid: Uid) -> bool {
    let node: &Node = tree.node(uid);
    ASYNC.contains(&node.id.as_str()) || node.children.iter().any(|&child| can_run(tree, child))
}

/// R0: what behaviortree-rs refuses in the file before it builds a tree, read the way
/// `register_bt_from_text` reads it, and then the choice of the main tree. It stops at the
/// first problem, so there is at most one; it is about the root (uid 0), at the line of what is
/// wrong. Malformed XML is bt-core's to report.
pub fn check_xml(xml: &str) -> Option<CompatIssue> {
    let doc = roxmltree::Document::parse_with_options(xml, bt_core::model::XML_OPTIONS).ok()?;
    let at = |pos: usize, message: String| {
        Some(CompatIssue {
            uid: 0,
            line: doc.text_pos_at(pos).row,
            message,
            note: None,
            fails: Fails::Build,
        })
    };
    let raw = |node: roxmltree::Node<'_, '_>| &xml[node.range()];
    let self_closing = |node: roxmltree::Node<'_, '_>| raw(node).ends_with("/>");
    let blank = |node: &roxmltree::Node<'_, '_>| node.is_text() && raw(*node).trim().is_empty();

    // Only the XML declaration may come before <root>: no comment, DOCTYPE or other
    // processing instruction.
    let root = doc.root_element();
    let start = root.range().start;
    let mut before = xml[..start].trim_start_matches('\u{feff}').trim_start();
    if let Some(rest) = before.strip_prefix("<?xml")
        && rest.starts_with(|c: char| c.is_whitespace() || c == '?')
    {
        before = rest
            .split_once("?>")
            .map_or("", |(_, rest)| rest)
            .trim_start();
    }
    if !before.is_empty() {
        return at(start - before.len(), MISSING_ROOT.to_string());
    }

    let mut trees = Vec::new();
    let mut kids = root
        .children()
        .filter(|n| !n.is_comment() && !blank(n))
        .peekable();
    while let Some(kid) = kids.next() {
        let pos = kid.range().start;
        let text = raw(kid);
        let event = if kid.is_element() && self_closing(kid) {
            Some(format!(
                "Empty(BytesStart {{ buf: Borrowed({}), name_len: {} }})",
                byte_string(&text[1..text.len() - 2]),
                kid.tag_name().name().len()
            ))
        } else if kid.is_text() {
            Some(format!(
                "Text(BytesText {{ content: Borrowed({}) }})",
                byte_string(text.trim())
            ))
        } else if kid.is_pi() {
            Some(format!(
                "PI(BytesText {{ content: Borrowed({}) }})",
                byte_string(&text[2..text.len() - 2])
            ))
        } else {
            None
        };
        if let Some(event) = event {
            return at(pos, format!("{INTERNAL} {event}"));
        }
        match kid.tag_name().name() {
            "TreeNodesModel" => continue,
            "BehaviorTree" => {}
            other => {
                return at(
                    pos,
                    format!(
                        "Expected to find <root> tag at start of XML. Found <{other}> instead."
                    ),
                );
            }
        }
        let Some(id) = kid.attribute("ID") else {
            return at(pos, NO_ID.to_string());
        };
        trees.push(id);
        // Self-closing elements before the first one with an end tag are skipped; after it
        // nothing may follow.
        let body: Vec<_> = kid
            .children()
            .filter(|n| !n.is_comment() && !blank(n))
            .collect();
        match body
            .iter()
            .position(|n| n.is_element() && !self_closing(*n))
        {
            Some(i) => {
                if let Some(extra) = body.get(i + 1) {
                    return at(extra.range().start, ONE_CHILD.to_string());
                }
            }
            // It reads on past `</BehaviorTree>` for the end tag it still expects.
            None if kids.peek().is_some() => return at(pos, ONE_CHILD.to_string()),
            None => return at(pos, END_TAG.to_string()),
        }
    }
    // `create_sync_tree_from_text`: one tree is the main tree whatever the file says.
    if trees.len() > 1 {
        match root.attribute("main_tree_to_execute") {
            None => return at(start, NO_MAIN_TREE.to_string()),
            Some(main) if !trees.contains(&main) => {
                return at(start, format!("Can't find tree [{main}]"));
            }
            Some(_) => {}
        }
    }
    None
}

/// quick-xml's `write_byte_string`: quoted, `"` escaped, other bytes outside ASCII 32..=126
/// in hex.
fn byte_string(text: &str) -> String {
    let mut out = String::from("\"");
    for b in text.bytes() {
        match b {
            34 => out.push_str("\\\""),
            32..=126 => out.push(b as char),
            _ => out.push_str(&format!("{b:#02X}")),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use bt_core::Document;

    fn tree(xml: &str) -> Tree {
        let doc = Document::parse(xml).unwrap();
        doc.instantiate(doc.main_tree_id()).unwrap()
    }

    fn body(body: &str) -> Tree {
        tree(&format!(
            r#"<root BTCPP_format="4"><BehaviorTree ID="T">{body}</BehaviorTree></root>"#
        ))
    }

    fn test_data(name: &str) -> String {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
        std::fs::read_to_string(format!("{path}{name}")).unwrap()
    }

    fn messages(tree: &Tree) -> Vec<String> {
        check(tree).iter().map(CompatIssue::text).collect()
    }

    fn xml_issue(xml: &str) -> Option<(u32, String)> {
        check_xml(xml).map(|i| (i.line, i.message))
    }

    #[test]
    fn both_samples_are_clean() {
        for name in [
            "bt_1790934549_welding_cell_2_small.json",
            "bt_1791274820_cell_2robot.json",
        ] {
            let plan = crate::plan::parse_plan(&test_data(name)).unwrap();
            // The model declares `WaitForRobot depends_on_ops`, which cell-runner does not
            // know, but no node sets it.
            assert!(plan.tree.xml.contains("depends_on_ops"));
            assert_eq!(
                messages(&tree(&plan.tree.xml)),
                Vec::<String>::new(),
                "{name}"
            );
        }
    }

    #[test]
    fn undeclared_attributes_are_rejected() {
        let xml = test_data("orchestrator_two_robots.xml");
        assert!(check(&tree(&xml)).is_empty());
        let issues = check(&tree(
            &xml.replace(r#"has_interlock="false"/>"#, r#"bogus="1"/>"#),
        ));
        assert_eq!(issues.len(), 2);
        assert!(
            issues[0]
                .message
                .starts_with("Port name [bogus] did not match Node [ExecuteTrajectory]")
        );
        assert_eq!(issues[0].line, 6);

        let named = messages(&body(r#"<Sequence name="x"><CheckSafety/></Sequence>"#));
        assert_eq!(
            named,
            ["Port name [name] did not match Node [Sequence] port list: []"]
        );
        // Leaves declare `name`.
        assert!(check(&body(r#"<CheckSafety name="estop" condition="c"/>"#)).is_empty());
    }

    #[test]
    fn reserved_attributes_are_rejected_too() {
        // bt-core keeps these out of the ports; behaviortree-rs takes them for ports.
        let found = messages(&body(
            r#"<Sequence name="Sequence"><CheckSafety _description="d" _uid="1"/></Sequence>"#,
        ));
        // A node's attributes are checked after its children.
        assert_eq!(
            found,
            [
                r#"Port name [_description] did not match Node [CheckSafety] port list: ["name", "condition"]"#,
                r#"Port name [_uid] did not match Node [CheckSafety] port list: ["name", "condition"]"#,
                "Port name [name] did not match Node [Sequence] port list: []",
            ]
        );
        // A cell leaf named like its ID is fine: it declares `name`.
        assert!(check(&body(r#"<CheckSafety name="CheckSafety"/>"#)).is_empty());
        // The v3 form: behaviortree-rs looks the `<Action>` tag up, not the `ID`.
        let v3 = messages(&body(
            r#"<Sequence><Action ID="CheckSafety" name="x"/></Sequence>"#,
        ));
        assert_eq!(
            v3,
            ["Attempted to parse node with unregistered name: Action"]
        );
    }

    /// The order behaviortree-rs builds in (cell-runner's first error, checked against it in
    /// `tests/parity.rs`): a tag before what is below it, attributes after.
    #[test]
    fn issues_come_in_build_order() {
        let found = messages(&body(
            r#"<Sequence name="s"><Fallback name="f"><Bogus/></Fallback></Sequence>"#,
        ));
        assert_eq!(
            found,
            [
                "Attempted to parse node with unregistered name: Bogus",
                "Port name [name] did not match Node [Fallback] port list: []",
                "Port name [name] did not match Node [Sequence] port list: []",
            ]
        );
        // A decorator builds its first child before it sees the second.
        let found = messages(&body(r#"<Inverter><Bogus/><Other/></Inverter>"#));
        assert_eq!(
            found,
            [
                "Attempted to parse node with unregistered name: Bogus",
                "Violated node type constraint: Decorator node [Inverter] may only have one child",
                "Attempted to parse node with unregistered name: Other",
            ]
        );
    }

    #[test]
    fn end_tags_matter() {
        let et = r#"robot_ns="r1" operation_id="A" trajectory="{t}""#;
        let found = messages(&body(&format!(
            r#"<Sequence>
              <ExecuteTrajectory {et}></ExecuteTrajectory>
              <ExecuteTrajectory {et}/>
              <Sequence/>
              <Inverter/>
              <SubTree ID="Job"></SubTree>
              <SubTree ID="Job"/>
              <SubTreePlus ID="Job"/>
            </Sequence>"#
        )));
        let mismatch = "Node type [] didn't had invalid presence/absence of children.";
        assert_eq!(
            found,
            [
                format!(
                    "{mismatch} ([ExecuteTrajectory] has an end tag; write it as \
                     <ExecuteTrajectory/>)"
                ),
                format!("{mismatch} ([Sequence] needs children)"),
                format!("{mismatch} ([Inverter] needs a child)"),
                "Attempted to parse node with unregistered name: SubTree".to_string(),
                "Can't find tree [Job]".to_string(),
                "Attempted to parse node with unregistered name: SubTreePlus".to_string(),
            ]
        );
    }

    #[test]
    fn parallel_thresholds_fail_when_ticked() {
        let et = |op: &str| format!(r#"<ExecuteTrajectory robot_ns="r1" operation_id="{op}"/>"#);
        let parallel = |attrs: &str, n: usize| {
            let kids: String = (0..n).map(|i| et(&i.to_string())).collect();
            check(&body(&format!(
                "<Sequence><Parallel {attrs}>{kids}</Parallel></Sequence>"
            )))
        };
        let never = "Number of children is less than the threshold. Can never";
        let found = parallel(r#"success_count="3""#, 2);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].message, format!("{never} succeed."));
        assert_eq!(found[0].fails, Fails::Tick);
        assert_eq!(
            parallel(r#"success_count="1" failure_count="3""#, 2)[0].message,
            format!("{never} fail.")
        );
        // An empty Parallel cannot fail once (behaviortree-rs's failure_count is 1).
        assert_eq!(parallel("", 0)[0].message, format!("{never} fail."));
        for fine in [
            (r#"success_count="-1""#, 2),
            (r#"success_count="2" failure_count="-1""#, 2),
            (r#"success_count="{many}""#, 1),
        ] {
            assert!(parallel(fine.0, fine.1).is_empty(), "{fine:?}");
        }
    }

    #[test]
    fn the_file_around_the_trees() {
        let tree = r#"<BehaviorTree ID="T"><Sequence><CheckSafety/></Sequence></BehaviorTree>"#;
        let file = |before: &str, inside: &str| {
            xml_issue(&format!(
                r#"{before}<root BTCPP_format="4">{inside}</root>"#
            ))
        };
        let missing_root =
            "Expected to find <root> start tag at start of XML. Found incorrect tag.".to_string();
        assert_eq!(file("", tree), None);
        assert_eq!(file("<?xml version=\"1.0\" ?>\n", tree), None);
        assert_eq!(file("\u{feff}  \n", tree), None);
        for before in [
            "<!-- note -->",
            "<!DOCTYPE root>",
            "<?xml-stylesheet href=\"a\"?>",
        ] {
            assert_eq!(
                file(&format!("<?xml version=\"1.0\"?>\n{before}\n"), tree),
                Some((2, missing_root.clone())),
                "{before}"
            );
        }
        // Comments elsewhere are fine.
        assert_eq!(
            file(
                "",
                &format!("<!-- a -->{tree}<!-- b --><TreeNodesModel></TreeNodesModel>")
            ),
            None
        );

        let one_child = "Violated node type constraint: BehaviorTree node may only have one child";
        let end_tag = "Violated node type constraint: Expected end tag for BehaviorTree";
        let bare = r#"<BehaviorTree ID="B"><CheckSafety/></BehaviorTree>"#;
        assert_eq!(file("", bare).unwrap().1, end_tag);
        assert_eq!(file("", &format!("{bare}{tree}")).unwrap().1, one_child);
        assert_eq!(
            file("", r#"<BehaviorTree ID="E"></BehaviorTree>"#)
                .unwrap()
                .1,
            end_tag
        );
        // Bare leaves before the root node are skipped; nothing may follow it.
        let skipped = r#"<BehaviorTree ID="S"><CheckSafety/><Sequence><CheckSafety/></Sequence></BehaviorTree>"#;
        assert_eq!(file("", skipped), None);
        let two =
            r#"<BehaviorTree ID="S"><Sequence/><Sequence></Sequence><CheckSafety/></BehaviorTree>"#;
        assert_eq!(file("", two).unwrap().1, one_child);
        assert_eq!(
            file("", r#"<BehaviorTree><Sequence></Sequence></BehaviorTree>"#)
                .unwrap()
                .1,
            "Found BehaviorTree definition without ID. Cannot continue parsing."
        );
        assert_eq!(
            file("", &format!("<Foo></Foo>{tree}")).unwrap().1,
            "Expected to find <root> tag at start of XML. Found <Foo> instead."
        );
        assert_eq!(
            file("", &format!(r#"{tree}<include path="a b.xml" />"#))
                .unwrap()
                .1,
            "Errors like this shouldn't happen. Something bad has happened. Please report this. \
             Empty(BytesStart { buf: Borrowed(\"include path=\\\"a b.xml\\\" \"), name_len: 7 })"
        );

        // Several trees need an existing main one; a single tree is the main tree anyway.
        let other = r#"<BehaviorTree ID="U"><Sequence><CheckSafety/></Sequence></BehaviorTree>"#;
        let main = |attr: &str| {
            xml_issue(&format!(
                r#"<root BTCPP_format="4"{attr}>{tree}{other}</root>"#
            ))
            .map(|(_, m)| m)
        };
        assert_eq!(
            main("").as_deref(),
            Some("No main tree was provided, either in the XML or as a function parameter.")
        );
        assert_eq!(
            main(r#" main_tree_to_execute="C""#).as_deref(),
            Some("Can't find tree [C]")
        );
        assert_eq!(main(r#" main_tree_to_execute="U""#), None);
        assert_eq!(
            xml_issue(&format!(r#"<root main_tree_to_execute="C">{tree}</root>"#)),
            None
        );
    }

    #[test]
    fn edits_are_checked() {
        let doc = Document::parse(&test_data("orchestrator_two_robots.xml")).unwrap();
        let mut ex = bt_exec::Executor::new(
            doc.instantiate(doc.main_tree_id()).unwrap(),
            Default::default(),
        );
        assert!(check(ex.tree()).is_empty());
        let leaf = ex
            .tree()
            .nodes
            .iter()
            .find(|n| n.id == "ExecuteTrajectory")
            .unwrap()
            .uid;
        ex.set_port_value(leaf, "bogus", "1").unwrap();
        let found = messages(ex.tree());
        assert_eq!(found.len(), 1);
        assert!(found[0].starts_with("Port name [bogus]"), "{found:?}");
    }

    #[test]
    fn subtrees_are_exempt() {
        let xml = r#"<root BTCPP_format="4" main_tree_to_execute="Main">
            <BehaviorTree ID="Main"><Sequence><SubTree ID="Job" robot="r1" _autoremap="true"/></Sequence></BehaviorTree>
            <BehaviorTree ID="Job"><MoveHome robot_ns="{robot}"/></BehaviorTree>
          </root>"#;
        assert_eq!(messages(&tree(xml)), Vec::<String>::new());
    }

    #[test]
    fn trees_behaviortree_rs_cannot_build_are_rejected() {
        let found = messages(&body(
            r#"<Sequence>
              <SubTree ID="Typo"/>
              <Inverter><CheckSafety/><CheckSafety/></Inverter>
              <Inverter></Inverter>
              <CheckSafety><CheckSafety/></CheckSafety>
            </Sequence>"#,
        ));
        assert_eq!(
            found,
            [
                "Can't find tree [Typo]",
                "Violated node type constraint: Decorator node [Inverter] may only have one child",
                "Node type [] didn't had invalid presence/absence of children. ([Inverter] needs \
                 a child)",
                "Node type [] didn't had invalid presence/absence of children. ([CheckSafety] must \
                 not have children)",
            ]
        );
        assert!(
            check(&body("<Sequence/>"))
                .iter()
                .all(|i| i.fails == Fails::Build)
        );
    }

    #[test]
    fn the_gates_sample_is_rejected() {
        let tree = tree(&test_data("v4_cell_with_gates.xml"));
        let found = messages(&tree);
        for needle in [
            "unregistered name: Script",
            "unregistered name: Timeout",
            "unregistered name: RetryUntilSuccessful",
            "Port name [name] did not match Node [ReactiveSequence]",
            "Port name [name] did not match Node [Sequence]",
            "Port name [name] did not match Node [Parallel]",
            "Port name [name] did not match Node [Fallback]",
        ] {
            assert!(
                found.iter().any(|m| m.contains(needle)),
                "{needle}: {found:#?}"
            );
        }
        // CheckSafety next to one RUNNING branch is fine.
        assert!(!found.iter().any(|m| m.contains("single child")));
    }

    #[test]
    fn conditions_are_rejected() {
        let found = messages(&body(r#"<Sequence><CheckSafety _skipIf="x"/></Sequence>"#));
        assert_eq!(found.len(), 1);
        assert!(found[0].contains("[_skipIf]"), "{found:?}");
    }

    #[test]
    fn a_reactive_sequence_may_have_one_running_child() {
        let et = r#"<ExecuteTrajectory robot_ns="r1" trajectory="{t}"/>"#;
        let wait = r#"<WaitForRobot robot_ns="r1"/>"#;
        let reactive = |a: &str, b: &str| {
            messages(&body(&format!(
                "<ReactiveSequence>{a}<Sequence>{b}</Sequence></ReactiveSequence>"
            )))
        };
        let single = "[ReactiveSequence]: Only a single child can return Running.";
        assert!(reactive(et, et)[0].starts_with(single));
        assert!(reactive(wait, et)[0].starts_with(single));
        assert!(reactive("<CheckSafety/>", et).is_empty());
        assert!(reactive(et, "<CheckSafety/>").is_empty());
        let delayed = format!(r#"<Delay delay_msec="5">{et}</Delay>"#);
        assert!(reactive(&delayed, et)[0].starts_with(single));
    }
}
