//! Behaviour of the builtin nodes against what BehaviorTree.CPP 4 does.

use std::collections::BTreeMap;

use bt_core::{Document, NodeStatus, Uid};
use bt_exec::{Executor, LeafPolicy, Override};

use NodeStatus::{Failure, Idle, Running, Skipped, Success};

fn executor(body: &str) -> Executor {
    let xml = format!(
        r#"<root BTCPP_format="4"><BehaviorTree ID="T">{body}</BehaviorTree>
        <TreeNodesModel><Condition ID="C"/><Condition ID="C2"/><Action ID="A"/></TreeNodesModel></root>"#
    );
    let doc = Document::parse(&xml).unwrap();
    let tree = doc.instantiate("T").unwrap();
    Executor::new(tree, BTreeMap::new())
}

fn uid(ex: &Executor, name: &str) -> Uid {
    ex.tree()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named {name}"))
        .uid
}

fn force(ex: &mut Executor, name: &str, status: NodeStatus) {
    let uid = uid(ex, name);
    ex.set_override(
        uid,
        Some(Override {
            status,
            pinned: false,
        }),
    );
}

fn manual(ex: &mut Executor, name: &str) {
    let uid = uid(ex, name);
    ex.set_policy(uid, LeafPolicy::Manual);
}

fn status(ex: &Executor, name: &str) -> NodeStatus {
    ex.status(uid(ex, name))
}

/// Statuses `uid` went through, in order.
fn history(ex: &Executor, name: &str) -> Vec<NodeStatus> {
    ex.trace()
        .of_node(uid(ex, name))
        .map(|t| t.status)
        .collect()
}

#[test]
fn sequence_runs_children_in_order_and_resets_them() {
    let mut ex = executor(r#"<Sequence name="s"><A name="a"/><A name="b"/></Sequence>"#);
    manual(&mut ex, "a");
    manual(&mut ex, "b");
    assert_eq!(ex.tick(), Running);
    assert_eq!((status(&ex, "a"), status(&ex, "b")), (Running, Idle));
    force(&mut ex, "a", Success);
    assert_eq!(ex.tick(), Running);
    assert_eq!((status(&ex, "a"), status(&ex, "b")), (Success, Running));
    force(&mut ex, "b", Success);
    assert_eq!(ex.tick(), Success);
    // Children are reset when the sequence completes; the trace still has their SUCCESS.
    assert_eq!(history(&ex, "a"), [Running, Success, Idle]);
    assert_eq!(history(&ex, "s"), [Running, Success]);
    let view = ex.trace().view_at(uid(&ex, "a"), ex.ticks());
    assert_eq!(view.was, Some(Success));
}

#[test]
fn sequence_with_memory_resumes_after_failure() {
    let mut ex = executor(r#"<SequenceWithMemory><A name="a"/><A name="b"/></SequenceWithMemory>"#);
    manual(&mut ex, "b");
    ex.run_virtual(10); // a succeeds after 1 s, b waits
    assert_eq!(status(&ex, "b"), Running);
    force(&mut ex, "b", Failure);
    assert_eq!(ex.tick(), Failure);
    // Next run starts at b, a is not ticked again.
    let a_runs = history(&ex, "a").iter().filter(|s| **s == Running).count();
    ex.tick();
    assert_eq!(
        history(&ex, "a").iter().filter(|s| **s == Running).count(),
        a_runs
    );
    assert_eq!(status(&ex, "b"), Running);
}

#[test]
fn reactive_sequence_halts_the_running_child_when_a_condition_fails() {
    let mut ex = executor(r#"<ReactiveSequence><C name="c"/><A name="a"/></ReactiveSequence>"#);
    manual(&mut ex, "a");
    assert_eq!(ex.tick(), Running);
    assert_eq!(status(&ex, "a"), Running);
    // Each tick re-checks the condition (and resets it to IDLE while a runs).
    assert_eq!(status(&ex, "c"), Idle);
    let c = uid(&ex, "c");
    ex.set_policy(c, LeafPolicy::Fail { after_s: 0.0 });
    assert_eq!(ex.tick(), Failure);
    assert_eq!(status(&ex, "a"), Idle);
    assert_eq!(history(&ex, "a"), [Running, Idle]);
}

#[test]
fn fallback_stops_at_first_success() {
    let mut ex = executor(r#"<Fallback><C name="c"/><A name="a"/></Fallback>"#);
    let c = uid(&ex, "c");
    ex.set_policy(c, LeafPolicy::Fail { after_s: 0.0 });
    manual(&mut ex, "a");
    assert_eq!(ex.tick(), Running);
    assert_eq!(status(&ex, "c"), Failure);
    force(&mut ex, "a", Success);
    assert_eq!(ex.tick(), Success);
}

#[test]
fn parallel_thresholds() {
    let body = r#"<Parallel success_count="2" failure_count="2">
        <A name="a"/><A name="b"/><A name="c"/></Parallel>"#;
    let mut ex = executor(body);
    for n in ["a", "b", "c"] {
        manual(&mut ex, n);
    }
    assert_eq!(ex.tick(), Running);
    force(&mut ex, "a", Success);
    assert_eq!(ex.tick(), Running);
    force(&mut ex, "b", Success);
    assert_eq!(ex.tick(), Success);
    // c was still running and is halted.
    assert_eq!(history(&ex, "c"), [Running, Idle]);

    // Default (-1, 1): all must succeed, one failure fails.
    let mut ex = executor(r#"<Parallel><A name="a"/><A name="b"/></Parallel>"#);
    manual(&mut ex, "b");
    let (status, _) = ex.run_virtual(10);
    assert_eq!(status, Running);
    force(&mut ex, "b", Failure);
    assert_eq!(ex.tick(), Failure);
}

#[test]
fn retry_and_repeat_count() {
    let mut ex =
        executor(r#"<RetryUntilSuccessful num_attempts="3"><A name="a"/></RetryUntilSuccessful>"#);
    let a = uid(&ex, "a");
    ex.set_policy(a, LeafPolicy::Fail { after_s: 0.0 });
    assert_eq!(ex.tick(), Failure);
    assert_eq!(
        history(&ex, "a").iter().filter(|s| **s == Failure).count(),
        3
    );

    let mut ex = executor(r#"<Repeat num_cycles="3"><A name="a"/></Repeat>"#);
    let a = uid(&ex, "a");
    ex.set_policy(a, LeafPolicy::Succeed { after_s: 0.0 });
    assert_eq!(ex.tick(), Success);
    assert_eq!(
        history(&ex, "a").iter().filter(|s| **s == Success).count(),
        3
    );
}

#[test]
fn timers_run_in_virtual_time() {
    let mut ex = executor(
        r#"<Sequence><Delay delay_msec="500"><A name="a"/></Delay>
           <Timeout msec="200"><A name="slow"/></Timeout></Sequence>"#,
    );
    let (status, _) = ex.run_virtual(100);
    assert_eq!(status, Failure);
    // a: 0.5 s delay + 1 s; then the timeout fires 0.2 s into `slow`.
    assert!((ex.now() - 1.7).abs() < 1e-6, "now = {}", ex.now());
    assert_eq!(history(&ex, "slow"), [Running, Idle]);
}

#[test]
fn scripts_preconditions_and_blackboard() {
    let mut ex = executor(
        r#"<Sequence>
             <Script code="count := 0; go := true"/>
             <A name="skipped" _skipIf="go"/>
             <Script name="inc" code="count += 1" _onSuccess="done := count == 1"/>
             <ScriptCondition name="check" code="done &amp;&amp; count == 1"/>
             <SetBlackboard value="hello" output_key="msg"/>
             <Precondition if="msg == 'hello'" else="FAILURE"><AlwaysSuccess name="ok"/></Precondition>
           </Sequence>"#,
    );
    assert_eq!(ex.tick(), Success);
    assert_eq!(history(&ex, "skipped"), [Skipped, Idle]);
    assert_eq!(history(&ex, "check"), [Success, Idle]);
    assert_eq!(history(&ex, "ok"), [Success, Idle]);
    let bb: Vec<(&str, String)> = ex
        .blackboard()
        .into_iter()
        .map(|(_, k, v)| (k, v.to_string()))
        .collect();
    assert!(bb.contains(&("msg", "\"hello\"".to_string())), "{bb:?}");
}

#[test]
fn subtree_remapping() {
    let xml = r#"<root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main"><Sequence>
        <Script code="x := 41"/>
        <SubTree ID="Inc" value="{x}" label="lit"/>
        <ScriptCondition name="check" code="x == 42"/>
      </Sequence></BehaviorTree>
      <BehaviorTree ID="Inc"><Sequence>
        <Script code="value += 1"/>
        <ScriptCondition name="lit_ok" code="label == 'lit'"/>
        <Script code="private := 1"/>
      </Sequence></BehaviorTree>
    </root>"#;
    let tree = Document::parse(xml).unwrap().instantiate("Main").unwrap();
    let mut ex = Executor::new(tree, BTreeMap::new());
    assert_eq!(ex.tick(), Success, "{:?}", ex.log());
    // `private` stays in the subtree's blackboard.
    let main: Vec<&str> = ex
        .blackboard()
        .into_iter()
        .filter(|(o, ..)| o.is_none())
        .map(|(_, k, _)| k)
        .collect();
    assert_eq!(main, ["x"]);
}

#[test]
fn switch_and_if_then_else() {
    let mut ex = executor(
        r#"<Sequence>
             <Script code="mode := 'b'"/>
             <Switch2 variable="{mode}" case_1="a" case_2="b">
               <AlwaysFailure name="one"/><AlwaysSuccess name="two"/><AlwaysFailure name="default"/>
             </Switch2>
             <IfThenElse><C name="cond"/><AlwaysSuccess name="then"/><AlwaysFailure name="else"/></IfThenElse>
           </Sequence>"#,
    );
    assert_eq!(ex.tick(), Success, "{:?}", ex.log());
    assert!(history(&ex, "one").is_empty());
    assert_eq!(history(&ex, "two"), [Success, Idle]);
    assert!(history(&ex, "else").is_empty());
}

#[test]
fn waiting_on_lists_manual_leaves() {
    let mut ex = executor(r#"<Parallel><A name="a"/><A name="b"/></Parallel>"#);
    manual(&mut ex, "a");
    manual(&mut ex, "b");
    let (status, ticks) = ex.run_virtual(100);
    assert_eq!((status, ticks), (Running, 1));
    assert_eq!(ex.waiting_on(), [uid(&ex, "a"), uid(&ex, "b")]);
    force(&mut ex, "a", Success);
    force(&mut ex, "b", Success);
    assert_eq!(ex.tick(), Success);
}

#[test]
fn orchestrator_plan_runs() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test_data/orchestrator_two_robots.xml"
    );
    let xml = std::fs::read_to_string(path).unwrap();
    let tree = Document::parse(&xml)
        .unwrap()
        .instantiate("MainTree")
        .unwrap();
    let mut ex = Executor::new(tree, BTreeMap::new());
    let (status, _) = ex.run_virtual(100);
    assert_eq!(status, Success);
    // Two parallel phases of 1 s stub actions.
    assert!((ex.now() - 2.0).abs() < 1e-6);
}

#[test]
fn reset_starts_over() {
    let mut ex = executor(r#"<Sequence><A name="a"/></Sequence>"#);
    ex.run_virtual(10);
    assert!(ex.is_done());
    ex.reset();
    assert_eq!(ex.ticks(), 0);
    assert!(ex.trace().transitions().is_empty());
    assert_eq!(ex.root_status(), Idle);
}
