//! Real time and Step on plain trees: a tree busy without a timer gets a heartbeat only while it
//! stays that way, a looping tree that finishes at once starts over on the heartbeat, and a
//! Step reports a finish like Run to wait does.

use bt_view::NodeStatus::{Failure, Success};
use bt_view::{BtPanel, Executor, Stop, Uid};

const TOL: f64 = 1e-9;

fn tree(body: &str) -> BtPanel {
    let xml =
        format!(r#"<root BTCPP_format="4"><BehaviorTree ID="T">{body}</BehaviorTree></root>"#);
    let mut panel = BtPanel::new("real_time");
    panel.load_text("tree.xml", &xml).unwrap();
    panel
}

fn ex(panel: &BtPanel) -> &Executor {
    panel.executor().expect("a tree is loaded")
}

fn by_name(panel: &BtPanel, name: &str) -> Uid {
    ex(panel)
        .tree()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named {name}"))
        .uid
}

/// When each tick ran.
fn tick_times(panel: &BtPanel) -> Vec<f64> {
    let ex = ex(panel);
    (1..=ex.ticks())
        .map(|k| ex.time_at_tick(k).unwrap())
        .collect()
}

fn assert_times(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
    for (a, e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < TOL, "{actual:?} vs {expected:?}");
    }
}

/// Busy without a timer until `n` counts up to `limit` (one count per tick), then it fails.
fn busy(limit: u32) -> String {
    format!(
        r#"<KeepRunningUntilFailure>
             <Sequence>
               <Script code="n := n + 1"/>
               <ScriptCondition code="n &lt; {limit}"/>
             </Sequence>
           </KeepRunningUntilFailure>"#
    )
}

#[test]
fn a_beat_that_starts_a_timer_hands_over_to_the_events() {
    // At 10 Hz the first beat (0.1) ends the busy loop and starts the Sleep; the run then ticks
    // at its wakeup, not at the next beat.
    let mut panel = tree(&format!(
        r#"<Sequence>
             <Script code="n := 0"/>
             <Fallback>{}<Sleep msec="50"/></Fallback>
           </Sequence>"#,
        busy(2)
    ));
    panel.set_running(true);
    panel.advance_real_time(0.25);
    assert!(!panel.is_running());
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_times(&tick_times(&panel), &[0.0, 0.1, 0.15]);
}

#[test]
fn a_breakpoint_reached_by_a_beat_halts_at_once() {
    let mut panel = tree(&format!(
        r#"<Sequence>
             <Script code="n := 0"/>
             <ForceSuccess>{}</ForceSuccess>
             <Sleep name="nap" msec="1000"/>
             <AlwaysSuccess/>
           </Sequence>"#,
        busy(3)
    ));
    let nap = by_name(&panel, "nap");
    panel.executor_mut().unwrap().set_breakpoint(nap, true);
    panel.set_running(true);
    panel.advance_real_time(1.0);
    // The second beat reaches the breakpoint: the frame notes it, and the clock waits there.
    assert_eq!(panel.last_stop(), Some(&Stop::Breakpoint(vec![nap])));
    assert_times(&tick_times(&panel), &[0.0, 0.1, 0.2]);
    panel.advance_real_time(1.0);
    assert!(panel.is_running());
    assert_eq!(ex(&panel).ticks(), 3);
    assert!((ex(&panel).now() - 0.2).abs() < TOL);

    // Resumed, the Sleep starts at once and the run goes on from event to event.
    panel.executor_mut().unwrap().resume();
    panel.advance_real_time(1.0);
    assert!(!panel.is_running());
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Success)));
    assert_times(&tick_times(&panel), &[0.0, 0.1, 0.2, 0.2, 1.2]);
}

#[test]
fn a_looping_tree_that_finishes_at_once_restarts_on_the_heartbeat() {
    let mut panel = tree(r#"<Sequence><Script code="n := 1"/><AlwaysSuccess/></Sequence>"#);
    panel.set_looping(true);
    panel.set_running(true);
    for _ in 0..60 {
        panel.advance_real_time(1.0 / 60.0);
    }
    assert!(panel.is_running());
    // One tick at once, then one per period of the default 10 Hz as the clock moves on.
    let times = tick_times(&panel);
    assert!((10..=11).contains(&times.len()), "{times:?}");
    for (k, t) in times.iter().enumerate() {
        assert!((t - 0.1 * k as f64).abs() < TOL, "{times:?}");
    }
}

#[test]
fn step_reports_a_finish_like_run_to_wait() {
    // An empty tree fails without its root ever changing.
    let mut panel = tree("");
    panel.step();
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Failure)));
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::Finished(Failure)));
}
