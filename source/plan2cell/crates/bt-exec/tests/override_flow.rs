//! The flow the panel's "force" buttons drive: a tree that waits, then moves on.

use std::collections::BTreeMap;

use bt_core::{Document, NodeKind, NodeStatus};
use bt_exec::{Executor, LeafPolicy, Override};

#[test]
fn forcing_the_waiting_leaves_finishes_a_gated_cell_tree() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test_data/v4_cell_with_gates.xml"
    );
    let xml = std::fs::read_to_string(path).unwrap();
    let tree = Document::parse(&xml).unwrap().instantiate("Cell").unwrap();
    let mut ex = Executor::new(tree, BTreeMap::new());
    // As `btgui --manual` does: no stub action finishes on its own.
    ex.set_all_policies(NodeKind::Action, LeafPolicy::Manual);

    let mut rounds = 0;
    loop {
        let (status, _) = ex.run_virtual(1_000);
        if status != NodeStatus::Running {
            assert_eq!(status, NodeStatus::Success);
            break;
        }
        let waiting = ex.waiting_on();
        assert!(!waiting.is_empty(), "RUNNING but nothing to force");
        for uid in waiting {
            ex.set_override(
                uid,
                Some(Override {
                    status: NodeStatus::Success,
                    pinned: false,
                }),
            );
        }
        rounds += 1;
        assert!(rounds < 30, "the tree never finished");
    }
    // Every robot's work actually ran, and the cycle counter advanced.
    let ran = |name: &str| {
        let uid = ex.tree().nodes.iter().find(|n| n.name == name).unwrap().uid;
        ex.trace()
            .of_node(uid)
            .any(|t| t.status == NodeStatus::Success)
    };
    assert!(ran("approach") && ran("work") && ran("home") && ran("place_part"));
    let cycle = ex
        .blackboard()
        .into_iter()
        .find(|(_, k, _)| *k == "cycle")
        .map(|(_, _, v)| v.to_string());
    assert_eq!(cycle.as_deref(), Some("1"));
}
