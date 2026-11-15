//! An external source (the live cell) next to the interpreter: the motion leaves are found by
//! the operation the cell reports, its running leaves get a ring of their own without changing
//! the interpreter's statuses, and the drift lists where the two disagree.

use bt_view::{Action, BtPanel, Drift, NodeStatus, Uid};

const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";

fn welding() -> BtPanel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
    let text = std::fs::read_to_string(format!("{path}{WELDING}")).unwrap();
    let mut panel = BtPanel::new("external");
    panel.load_text(WELDING, &text).unwrap();
    panel
}

fn by_name(panel: &BtPanel, name: &str) -> Uid {
    let tree = panel.executor().unwrap().tree();
    tree.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("{name}"))
        .uid
}

fn frame(panel: &mut BtPanel) {
    let ctx = egui::Context::default();
    let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
        egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
    });
    output.textures_delta.clear();
}

#[test]
fn operations_map_to_their_leaves() {
    let panel = welding();
    assert_eq!(
        panel.uid_for_operation("robot1", "W01"),
        [by_name(&panel, "weld_robot1_W01")]
    );
    assert_eq!(
        panel.uid_for_operation("robot2", "W13"),
        [by_name(&panel, "weld_robot2_W13")]
    );
    assert_eq!(
        panel.uid_for_operation("robot1", "R1_PARK"),
        [by_name(&panel, "move_robot1_R1_PARK")]
    );
    // cell-runner's names for a MoveHome and an interlock pre-move.
    assert_eq!(
        panel.uid_for_operation("robot2", "home"),
        [by_name(&panel, "home_robot2")]
    );
    assert_eq!(
        panel.uid_for_operation("robot1", "W01:interlock"),
        panel.uid_for_operation("robot1", "W01")
    );
    // The wrong robot, or an operation the tree does not have.
    assert!(panel.uid_for_operation("robot2", "W01").is_empty());
    assert!(panel.uid_for_operation("robot1", "W99").is_empty());
    assert!(
        BtPanel::new("empty")
            .uid_for_operation("robot1", "W01")
            .is_empty()
    );
}

#[test]
fn the_external_source_leaves_the_interpreter_alone() {
    let mut panel = welding();
    let (w01, w13, w02) = (
        by_name(&panel, "weld_robot1_W01"),
        by_name(&panel, "weld_robot2_W13"),
        by_name(&panel, "weld_robot1_W02"),
    );
    assert_eq!(panel.drift(), None, "no source yet");
    panel.apply([Action::RunToTime(0.5)]);
    let statuses = |panel: &BtPanel| -> Vec<NodeStatus> {
        let ex = panel.executor().unwrap();
        (0..ex.tree().len()).map(|uid| ex.status(uid)).collect()
    };
    let before = statuses(&panel);
    assert_eq!(before[w01], NodeStatus::Running);
    assert_eq!(before[w13], NodeStatus::Running);

    // The cell agrees: no drift.
    panel.apply([Action::SetExternalActive(vec![
        (w01, "robot1".into()),
        (w13, "robot2".into()),
    ])]);
    assert_eq!(statuses(&panel), before, "statuses are the interpreter's");
    assert!(panel.drift().unwrap().is_empty());
    frame(&mut panel);

    // The cell is a weld ahead on robot1 (and an unknown node is dropped).
    panel.apply([Action::SetExternalActive(vec![
        (w02, "robot1".into()),
        (w13, "robot2".into()),
        (usize::MAX, "x".into()),
    ])]);
    assert_eq!(statuses(&panel), before);
    assert_eq!(panel.external_active().map(<[_]>::len), Some(2));
    assert_eq!(
        panel.drift(),
        Some(Drift {
            external_only: vec![w02],
            interpreter_only: vec![w01]
        })
    );
    frame(&mut panel);
    assert!(
        panel.user_actions().is_empty(),
        "nothing the user asked for"
    );

    // The cell idles: what the interpreter runs is all drift.
    panel.apply([Action::SetExternalActive(Vec::new())]);
    let mut running = vec![w01, w13];
    running.sort_unstable();
    assert_eq!(
        panel.drift(),
        Some(Drift {
            external_only: vec![],
            interpreter_only: running
        })
    );

    // Loading again forgets the source (its nodes were another tree's).
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test_data/bt_1790934549_welding_cell_2_small.json"
    ))
    .unwrap();
    panel.load_text(WELDING, &text).unwrap();
    assert_eq!(panel.external_active(), None);
    panel.apply([Action::SetExternalActive(vec![(w01, "robot1".into())])]);
    panel.clear_external();
    assert_eq!(panel.drift(), None);
}
