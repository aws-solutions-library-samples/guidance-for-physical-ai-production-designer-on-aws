//! Headless frames over every sample tree and plan: the panel lays out, paints, ticks and
//! scrubs without panicking, and a file that fails to load leaves the loaded tree in place.

use bt_view::{BtPanel, Stop};

fn frame(panel: &mut BtPanel, ctx: &egui::Context) {
    let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
        egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
    });
    // Nothing renders here, but epaint insists the texture deltas are consumed.
    output.textures_delta.clear();
}

const TEST_DATA: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data");

fn read(name: &str) -> String {
    std::fs::read_to_string(format!("{TEST_DATA}/{name}")).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// Every tree (`*.xml` with a `<BehaviorTree>`) and every plan (`*.json`) in `test_data/`.
fn samples() -> Vec<(String, String)> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(TEST_DATA).unwrap() {
        let path = entry.unwrap().path();
        let Some(ext) = path.extension() else {
            continue;
        };
        let text = std::fs::read_to_string(&path).unwrap();
        if (ext == "xml" && text.contains("<BehaviorTree")) || ext == "json" {
            out.push((
                path.file_name().unwrap().to_string_lossy().into_owned(),
                text,
            ));
        }
    }
    out.sort();
    out
}

#[test]
fn every_sample_draws_and_ticks() {
    let ctx = egui::Context::default();
    let samples = samples();
    assert!(samples.iter().any(|(name, _)| name.ends_with(".json")));
    for (name, text) in samples {
        let mut panel = BtPanel::new(("test", &name));
        panel
            .load_text(&name, &text)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let trees = panel.executor().map(|ex| ex.tree().len()).unwrap_or(0);
        assert!(trees >= 1, "{name}");
        frame(&mut panel, &ctx);
        for _ in 0..5 {
            panel.step();
            frame(&mut panel, &ctx);
        }
        // Run on to where it stops, then look back at the middle of the run.
        panel.run_to_wait();
        assert!(panel.last_stop().is_some(), "{name}");
        frame(&mut panel, &ctx);
        let mid = panel.executor().unwrap().now() / 2.0;
        panel.scrub_to_time(Some(mid));
        assert!(panel.shown_time() <= mid + 1e-9, "{name}");
        assert!(
            panel.shown_tick() <= panel.executor().unwrap().ticks(),
            "{name}"
        );
        frame(&mut panel, &ctx);
        panel.scrub_to_time(None);
        // Collapsing a subtree and flipping the layout must not panic either.
        let subtrees: Vec<_> = panel
            .executor()
            .unwrap()
            .tree()
            .nodes
            .iter()
            .filter(|n| n.kind == bt_view::NodeKind::SubTree)
            .map(|n| n.uid)
            .collect();
        if let Some(uid) = subtrees.first() {
            panel.collapse(*uid, true);
            frame(&mut panel, &ctx);
            panel.collapse(*uid, false);
        }
        // The inspector draws for every node, selected in turn.
        for uid in 0..trees {
            panel.select(Some(uid));
            frame(&mut panel, &ctx);
        }
        panel.select(None);
        panel.reset();
        frame(&mut panel, &ctx);
    }
}

#[test]
fn loads_a_factoryos_plan_json() {
    let plan = r#"{
      "tree_id": "bt_1",
      "behaviour_tree": {
        "xml": "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><Sequence><ExecuteTrajectory robot_ns=\"r1\" trajectory=\"{traj}\"/></Sequence></BehaviorTree></root>",
        "blackboard": { "traj": [1, 2, 3] }
      }
    }"#;
    let ctx = egui::Context::default();
    let mut panel = BtPanel::new("plan");
    panel.load_text("bt_1.json", plan).unwrap();
    frame(&mut panel, &ctx);
    let ex = panel.executor().unwrap();
    assert_eq!(ex.tree().len(), 3);
    assert_eq!(
        ex.port_value(2, "trajectory")
            .map(|v| v.to_string())
            .as_deref(),
        Some("[1,2,3]")
    );
}

#[test]
fn a_bad_file_reports_instead_of_panicking() {
    let ctx = egui::Context::default();
    let mut panel = BtPanel::new("bad");
    panel.set_open_button(true);
    assert!(
        panel
            .load_text("broken.xml", "<root><BehaviorTree")
            .is_err()
    );
    frame(&mut panel, &ctx);
    assert!(panel.executor().is_none());
    assert!(panel.error().unwrap().starts_with("broken.xml: "));
    assert!(!panel.take_open_request());
}

const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";

/// The 2robot plan, loaded and run to its end.
fn two_robot_panel(ctx: &egui::Context) -> BtPanel {
    let mut panel = BtPanel::new("kept");
    panel.load_text(TWO_ROBOT, &read(TWO_ROBOT)).unwrap();
    panel.run_to_wait();
    frame(&mut panel, ctx);
    panel
}

/// The panel still shows the 2robot plan, with its run, and names `file` in the error.
fn assert_kept(panel: &BtPanel, file: &str) {
    assert_eq!(panel.tree_id(), Some("MainTree"));
    assert_eq!(
        panel.plan_info().unwrap().tree_id.as_deref(),
        Some("bt_1791274820_cell_2robot")
    );
    let ex = panel.executor().unwrap();
    assert_eq!(ex.tree().len(), 4);
    assert!(ex.is_done(), "the run is kept too");
    assert!(panel.cell().is_some());
    let document = panel.document().unwrap();
    assert_eq!(document.tree_ids, ["MainTree"]);
    assert!(document.models.contains_key("ExecuteTrajectory"));
    let error = panel.error().unwrap();
    assert!(error.contains(file), "{error}");
}

#[test]
fn a_bad_file_keeps_the_loaded_tree() {
    let ctx = egui::Context::default();
    let mut panel = two_robot_panel(&ctx);
    for (file, text) in [
        ("broken.xml", "<root><BehaviorTree"),
        ("broken.json", "{ \"behaviour_tree\": "),
        ("empty.json", "{}"),
    ] {
        assert!(panel.load_text(file, text).is_err(), "{file}");
        assert_kept(&panel, file);
        frame(&mut panel, &ctx);
    }
    // A good file clears the error.
    panel.load_text(TWO_ROBOT, &read(TWO_ROBOT)).unwrap();
    assert_eq!(panel.error(), None);
}

#[test]
fn a_failed_instantiate_keeps_the_loaded_tree() {
    let ctx = egui::Context::default();
    let mut panel = two_robot_panel(&ctx);
    // Only a tree the file does not have fails to instantiate (an unknown
    // `main_tree_to_execute` falls back to the first tree): switching to one keeps the
    // current tree, with its run.
    assert!(panel.show_tree("Nope").is_err());
    assert_eq!(panel.tree_id(), Some("MainTree"));
    assert!(panel.executor().unwrap().is_done());
    assert!(panel.error().unwrap().contains("Nope"));
    assert_eq!(
        panel.last_stop(),
        Some(&Stop::Finished(bt_view::NodeStatus::Success))
    );
    frame(&mut panel, &ctx);

    // A plan whose JSON is fine but whose XML has no tree fails before that, in parsing, and
    // keeps the current tree too (without the plan's blackboard).
    let plan = r#"{
      "tree_id": "other",
      "behaviour_tree": {
        "xml": "<root BTCPP_format=\"4\" main_tree_to_execute=\"Nope\"><TreeNodesModel/></root>",
        "blackboard": { "traj": [1, 2, 3] }
      }
    }"#;
    assert!(panel.load_text("other.json", plan).is_err());
    assert_kept(&panel, "other.json");
    let ex = panel.executor().unwrap();
    assert!(ex.initial_blackboard().get("traj").is_none());
    frame(&mut panel, &ctx);
}

#[test]
fn a_byte_order_mark_is_ignored() {
    // Some Windows editors start every file with one.
    for name in ["Simple.xml", TWO_ROBOT] {
        let mut panel = BtPanel::new(name);
        panel
            .load_text(name, &format!("\u{feff}{}", read(name)))
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(panel.executor().is_some(), "{name}");
    }
}

#[test]
fn a_stop_keeps_the_selection() {
    // btgui's `--manual --select NAME --run`: select a node, then run to where the tree waits.
    let ctx = egui::Context::default();
    let mut panel = BtPanel::new("select");
    let name = "v4_cell_with_gates.xml";
    panel.load_text(name, &read(name)).unwrap();
    panel
        .executor_mut()
        .unwrap()
        .set_all_policies(bt_view::NodeKind::Action, bt_view::LeafPolicy::Manual);
    let program = panel.select_by_name("program").unwrap();
    panel.run_to_wait();
    let Some(Stop::WaitingManual(waiting)) = panel.last_stop() else {
        panic!("{:?}", panel.last_stop());
    };
    assert!(!waiting.contains(&program));
    frame(&mut panel, &ctx);
    assert_eq!(panel.selected(), Some(program));
}
