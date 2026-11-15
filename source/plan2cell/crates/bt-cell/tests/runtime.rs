//! Where cell-runner's runtime, behaviortree-rs 0.2.5, parts from BehaviorTree.CPP, compiling a
//! plan (and running it with the cell leaves installed) follows behaviortree-rs.
//!
//! Every expected number and error text here is what cell-runner's own `btree.rs` and
//! `trajectory.rs` (with behaviortree-rs `=0.2.5-alpha3`) report for the same plan through
//! `parse_plan_file` and `Schedule::build(&tree, &[], HashMap::new())`.

mod common;

use bt_cell::{Compiled, ParsedPlan, compile, parse_plan};
use bt_core::NodeStatus::{Failure, Success};
use bt_exec::{RunOptions, Stop};
use serde_json::{Value, json};

use common::{
    assert_close, by_name, executor, interactive, messages, parse, schedule, segment, test_data,
    wp, xml_plan,
};

const Z: [f64; 6] = [0.0; 6];

/// The blackboard every plan here uses.
fn blackboard() -> Value {
    json!({
        "t": [wp(Z, 0.0), wp(Z, 1.0)],
        "a": [wp(Z, 0.0), wp(Z, 2.0)],
        "b": [wp(Z, 0.0), wp(Z, 1.0)],
        "flag": true,
    })
}

fn et(op: &str, robot: &str, traj: &str) -> String {
    format!(
        r#"<ExecuteTrajectory robot_ns="{robot}" operation_id="{op}" trajectory="{{{traj}}}"/>"#
    )
}

fn plan(body: &str) -> ParsedPlan {
    parse(xml_plan(body, blackboard()))
}

fn file(xml: &str) -> ParsedPlan {
    parse(json!({ "behaviour_tree": { "xml": xml, "blackboard": blackboard() } }))
}

fn compile_err(plan: &ParsedPlan) -> String {
    match compile(&plan.tree, &[], Default::default()) {
        Ok(c) => panic!("compiled to {:.4} s", c.schedule.duration),
        Err(e) => e,
    }
}

/// cell-runner's `build_bare`: no robots, no homes.
fn compiled(plan: &ParsedPlan) -> Compiled {
    compile(&plan.tree, &[], Default::default()).expect("plan compiles")
}

/// `(op, t0)` of every segment, in placement order.
fn starts(plan: &ParsedPlan) -> Vec<(String, f64)> {
    compiled(plan)
        .schedule
        .segments
        .iter()
        .map(|s| (s.op_id.clone(), s.t0))
        .collect()
}

fn assert_starts(actual: &[(String, f64)], expected: &[(&str, f64)]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?}");
    for ((op, t0), (want_op, want_t0)) in actual.iter().zip(expected) {
        assert_eq!(op, want_op, "{actual:?}");
        assert_close(*t0, *want_t0);
    }
}

/// behaviortree-rs's `SequenceStar` returns FAILURE once every child succeeded
/// (`sequence_star.rs`); BT.CPP's returns SUCCESS.
#[test]
fn sequence_star_fails_like_behaviortree_rs() {
    let p = plan(&format!(
        "<Sequence><SequenceStar>{}{}</SequenceStar>{}</Sequence>",
        et("S1", "r1", "t"),
        et("S2", "r1", "t"),
        et("after", "r1", "t")
    ));
    let c = compiled(&p);
    assert_close(c.schedule.duration, 5.0);
    assert_starts(&starts(&p), &[("S1", 0.0), ("S2", 2.5)]);
    assert_eq!(
        messages(&c.schedule),
        ["BehaviorTree returned FAILURE at t=5.00s"]
    );

    let mut ex = interactive(&p);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Failure));
    assert_close(ex.now(), 5.0);
    assert_eq!(schedule(&ex).segments, c.schedule.segments);
    assert!(
        ex.log().iter().any(|e| e
            .message
            .contains("as behaviortree-rs's SequenceStar returns")),
        "the log says why"
    );

    // Its FAILURE moves a Fallback on to the next child.
    let p = plan(&format!(
        "<Sequence><Fallback><SequenceStar>{}</SequenceStar>{}</Fallback>{}</Sequence>",
        et("S1", "r1", "t"),
        et("fb", "r1", "t"),
        et("after", "r1", "t")
    ));
    assert_close(compiled(&p).schedule.duration, 7.5);
    assert_starts(&starts(&p), &[("S1", 0.0), ("fb", 2.5), ("after", 5.0)]);

    // Without the cell leaves it is BT.CPP's again.
    let mut ex = executor(&p);
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
    assert_eq!(ex.dialect(), bt_exec::Dialect::BtCpp);
}

/// cell-runner's `port_f64`: a blackboard bool is no number, so the Delay is 0 ms.
#[test]
fn delay_reads_its_port_like_cell_runner() {
    let p = plan(&format!(
        r#"<Sequence><Delay delay_msec="{{flag}}">{}</Delay></Sequence>"#,
        et("Dbool", "r1", "t")
    ));
    let c = compiled(&p);
    assert_close(c.schedule.duration, 2.5);
    assert_close(segment(&c.schedule, "Dbool").t0, 0.0);
    let mut ex = interactive(&p);
    ex.run(RunOptions::VIRTUAL);
    assert_eq!(schedule(&ex).segments, c.schedule.segments);

    // An endless delay has nothing to wake it: a stall, as in cell-runner.
    let p = plan(&format!(
        r#"<Sequence><Delay delay_msec="inf">{}</Delay></Sequence>"#,
        et("A", "r1", "t")
    ));
    assert_eq!(
        compile_err(&p),
        "BehaviorTree stalled at t=0.00s: nodes are RUNNING but none can progress"
    );
}

/// behaviortree-rs loops a `Repeat` or `Retry` of an instant child to the end within its tick.
#[test]
fn long_synchronous_loops_finish_at_their_instant() {
    for body in [
        format!(
            r#"<Sequence><Repeat num_cycles="20000"><CheckSafety/></Repeat>{}</Sequence>"#,
            et("A", "r1", "t")
        ),
        format!(
            r#"<Sequence><ForceSuccess><Retry num_attempts="20000"><Inverter><CheckSafety/></Inverter></Retry></ForceSuccess>{}</Sequence>"#,
            et("A", "r1", "t")
        ),
    ] {
        let p = plan(&body);
        let c = compiled(&p);
        assert_close(c.schedule.duration, 2.5);
        assert_starts(&starts(&p), &[("A", 0.0)]);
        let mut ex = interactive(&p);
        assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Finished(Success));
        assert_eq!(schedule(&ex).segments, c.schedule.segments);
    }
}

/// An `inf` hold (a port edit, or `1e400` in a file) never ends: no wakeup, so the tree stalls
/// once everything else is done, with cell-runner's error; interactively it halts there.
#[test]
fn an_endless_motion_stalls_like_cell_runner() {
    let mut btcpp = common::btcpp_plan();
    let xml = btcpp["behaviour_tree"]["xml"].as_str().unwrap().replacen(
        r#"trajectory="{traj_A}" hold_duration_s="0.5""#,
        r#"trajectory="{traj_A}" hold_duration_s="inf""#,
        1,
    );
    btcpp["behaviour_tree"]["xml"] = json!(xml);
    assert_eq!(
        compile_err(&parse(btcpp)),
        "BehaviorTree stalled at t=3.50s: nodes are RUNNING but none can progress"
    );

    let mut welding = common::welding();
    welding.tree.xml = welding.tree.xml.replacen(
        r#"trajectory="{traj_robot1_W01}" hold_duration_s="0.5""#,
        r#"trajectory="{traj_robot1_W01}" hold_duration_s="inf""#,
        1,
    );
    assert_eq!(
        compile_err(&welding),
        "BehaviorTree stalled at t=10.69s: nodes are RUNNING but none can progress"
    );
    let mut ex = interactive(&welding);
    let w01 = by_name(&ex, "weld_robot1_W01");
    assert_eq!(ex.run(RunOptions::VIRTUAL).stop, Stop::Stalled(vec![w01]));
    assert_eq!(ex.now(), ex.time_at_tick(ex.ticks()).unwrap());
    assert!(ex.now().is_finite() && (ex.now() - 10.686).abs() < 0.01);
    let reason = ex.waiting_reason(w01).unwrap();
    assert_eq!(
        reason,
        "robot1 W01 never ends (an infinite hold or duration)"
    );
}

/// What behaviortree-rs builds but fails to tick stops cell-runner at that tick; until then
/// the runs agree, and a tree that never gets there runs to its end.
#[test]
fn tick_failures_stop_where_cell_runner_stops() {
    let two = format!("{}{}", et("X1", "r1", "t"), et("X2", "r2", "t"));
    let never = "Number of children is less than the threshold. Can never";
    for (body, error) in [
        (
            format!(r#"<Parallel success_count="1" failure_count="3">{two}</Parallel>"#),
            format!("BehaviorTree tick failed at t=0.00s: {never} fail."),
        ),
        (
            format!(
                r#"<Sequence>{}<Parallel success_count="3">{two}</Parallel></Sequence>"#,
                et("A", "r1", "a")
            ),
            format!("BehaviorTree tick failed at t=3.50s: {never} succeed."),
        ),
        (
            format!(
                "<Sequence>{}<Parallel></Parallel></Sequence>",
                et("A", "r1", "t")
            ),
            format!("BehaviorTree tick failed at t=2.50s: {never} fail."),
        ),
        (
            r#"<ReactiveSequence>
              <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
              <ExecuteTrajectory name="B" robot_ns="r2" operation_id="B" trajectory="{b}"/>
            </ReactiveSequence>"#
                .to_string(),
            "BehaviorTree tick failed at t=3.50s: [ReactiveSequence]: Only a single child can \
             return Running."
                .to_string(),
        ),
    ] {
        assert_eq!(compile_err(&plan(&body)), error, "{body}");
    }

    // Never reached: the Fallback is done once A succeeds.
    let p = plan(&format!(
        r#"<Fallback>{}<Parallel success_count="3">{two}</Parallel></Fallback>"#,
        et("A", "r1", "a")
    ));
    assert_close(compiled(&p).schedule.duration, 3.5);
    // The second child of the ReactiveSequence fails before anything in it runs.
    let p = plan(&format!(
        "<ReactiveSequence>{}<Sequence><ForceFailure><CheckSafety/></ForceFailure>{}</Sequence></ReactiveSequence>",
        et("A", "r1", "a"),
        et("B", "r2", "t")
    ));
    let c = compiled(&p);
    assert_close(c.schedule.duration, 3.5);
    assert_eq!(
        messages(&c.schedule),
        ["BehaviorTree returned FAILURE at t=3.50s"]
    );
}

/// The first error behaviortree-rs raises building a file, word for word.
#[test]
fn build_errors_are_cell_runners() {
    let a = et("A", "r1", "t");
    let seq = format!("<Sequence>{a}</Sequence>");
    let tree = |id: &str, body: &str| format!(r#"<BehaviorTree ID="{id}">{body}</BehaviorTree>"#);
    let root =
        |attrs: &str, inside: &str| format!(r#"<root BTCPP_format="4"{attrs}>{inside}</root>"#);
    let main = tree("MainTree", &seq);
    let mismatch = "Node type [] didn't had invalid presence/absence of children.";
    let one_child = "Violated node type constraint: BehaviorTree node may only have one child";
    let end_tag = "Violated node type constraint: Expected end tag for BehaviorTree";
    let missing_root = "Expected to find <root> start tag at start of XML. Found incorrect tag.";
    let cases: Vec<(String, String)> = vec![
        (
            format!(
                "<?xml version=\"1.0\"?>\n<!-- note -->\n{}",
                root("", &main)
            ),
            missing_root.into(),
        ),
        (
            format!("<?xml version=\"1.0\"?>\n<?foo bar?>\n{}", root("", &main)),
            missing_root.into(),
        ),
        (root("", &tree("MainTree", &a)), end_tag.into()),
        (root("", &tree("A", "")), end_tag.into()),
        (
            root("", &format!("{}{}", tree("A", &a), tree("B", &seq))),
            one_child.into(),
        ),
        (
            root("", &tree("A", &format!("{seq}{seq}"))),
            one_child.into(),
        ),
        (
            root(
                r#" main_tree_to_execute="MainTree""#,
                &format!("{main}{}", tree("Job", &a)),
            ),
            end_tag.into(),
        ),
        (
            root("", &format!("<BehaviorTree>{seq}</BehaviorTree>")),
            "Found BehaviorTree definition without ID. Cannot continue parsing.".into(),
        ),
        (
            root("", &format!("<Foo></Foo>{main}")),
            "Expected to find <root> tag at start of XML. Found <Foo> instead.".into(),
        ),
        (
            root("", &format!("{main}<TreeNodesModel/>")),
            "Errors like this shouldn't happen. Something bad has happened. Please report this. \
             Empty(BytesStart { buf: Borrowed(\"TreeNodesModel\"), name_len: 14 })"
                .into(),
        ),
        (
            root(
                r#" main_tree_to_execute="C""#,
                &format!("{}{}", tree("A", &seq), tree("B", &seq)),
            ),
            "Can't find tree [C]".into(),
        ),
        (
            root(
                "",
                &tree(
                    "MainTree",
                    r#"<Sequence><ExecuteTrajectory robot_ns="r1" operation_id="A" trajectory="{t}"></ExecuteTrajectory><CheckSafety></CheckSafety></Sequence>"#,
                ),
            ),
            mismatch.into(),
        ),
        (
            root(
                "",
                &tree("MainTree", &format!("<Sequence>{a}<Sequence/></Sequence>")),
            ),
            mismatch.into(),
        ),
        (
            root(
                "",
                &tree("MainTree", &format!("<Sequence>{a}<Inverter/></Sequence>")),
            ),
            mismatch.into(),
        ),
        (
            root(
                r#" main_tree_to_execute="MainTree""#,
                &format!(
                    "{}{}",
                    tree(
                        "MainTree",
                        r#"<Sequence><SubTree ID="Job"></SubTree></Sequence>"#
                    ),
                    tree("Job", &seq)
                ),
            ),
            "Attempted to parse node with unregistered name: SubTree".into(),
        ),
        (
            root(
                "",
                &tree(
                    "MainTree",
                    &format!(r#"<Sequence><Action ID="CheckSafety"/>{a}</Sequence>"#),
                ),
            ),
            "Attempted to parse node with unregistered name: Action".into(),
        ),
        (
            root(
                "",
                &tree(
                    "MainTree",
                    r#"<Sequence name="s"><Fallback name="f"><Bogus/></Fallback></Sequence>"#,
                ),
            ),
            "Attempted to parse node with unregistered name: Bogus".into(),
        ),
        (
            root(
                "",
                &tree(
                    "MainTree",
                    &format!("<Sequence><Inverter>{a}<Other/></Inverter></Sequence>"),
                ),
            ),
            "Violated node type constraint: Decorator node [Inverter] may only have one child"
                .into(),
        ),
        (
            root(
                "",
                &tree(
                    "MainTree",
                    r#"<Sequence name="s"><CheckSafety></CheckSafety></Sequence>"#,
                ),
            ),
            mismatch.into(),
        ),
    ];
    for (xml, error) in cases {
        assert_eq!(
            compile_err(&file(&xml)),
            format!("invalid BehaviorTree: {error}"),
            "{xml}"
        );
    }
    // Comments around and inside the trees are fine, and so are bare leaves before the root
    // node (behaviortree-rs builds the first of them).
    for xml in [
        format!(
            "{}\n<!-- trailing -->\n",
            root("", &format!("<!-- a -->{main}<!-- b -->"))
        ),
        root("", &tree("A", &format!("{a}{seq}"))),
    ] {
        assert_close(compiled(&file(&xml)).schedule.duration, 2.5);
    }
}

/// cell-runner's verdict on every tree in `test_data/` that it cannot run.
#[test]
fn test_data_errors_are_cell_runners() {
    let unregistered = "Attempted to parse node with unregistered name:";
    let one_child = "Violated node type constraint: BehaviorTree node may only have one child";
    for (name, error) in [
        ("Simple.xml", format!("{unregistered} RetryUntilSuccessful")),
        (
            "crossdoor_with_subtree.xml",
            format!("{unregistered} Condition"),
        ),
        ("custom_ports.xml", format!("{unregistered} Action")),
        (
            "issue_24.xml",
            "Found BehaviorTree definition without ID. Cannot continue parsing.".to_string(),
        ),
        ("issue_3.xml", one_child.to_string()),
        ("show_all.xml", format!("{unregistered} AlwaysFailure")),
        ("simple_without_model.xml", format!("{unregistered} Action")),
        (
            "subtree_test_fail.xml",
            "Can't find tree [ThisOne]".to_string(),
        ),
        ("subtree_with_remapping.xml", one_child.to_string()),
        ("test_subtrees_issue_8.xml", one_child.to_string()),
        (
            "test_xml_key_reordering_issue.xml",
            format!("{unregistered} Action"),
        ),
        (
            "v4_cell_with_gates.xml",
            "Expected to find <root> start tag at start of XML. Found incorrect tag.".to_string(),
        ),
    ] {
        let plan = parse_plan(&test_data(name)).unwrap();
        assert_eq!(
            compile_err(&plan),
            format!("invalid BehaviorTree: {error}"),
            "{name}"
        );
    }
}
