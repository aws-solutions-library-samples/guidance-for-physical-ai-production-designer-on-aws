//! The panel driven the way a user drives it: pointer clicks, right-clicks and typing on a
//! laptop-sized screen, with the widgets found by their labels in the AccessKit tree egui builds.
//!
//! A value typed in the inspector and committed by the click that asks for a run is in effect
//! for that run; the text fields are wide enough to click; Overrides ▾, "＋ port", "Type a
//! value…" and a node's right-click menu do what they say; and the halt banner offers
//! auto-release only where gates are stalled, and scrolls rather than push the canvas away. The
//! error row says an older tree is still showing for an error, not for a note about this one.

use bt_view::{BtPanel, Executor, LeafPolicy, NodeStatus, Override, Stop, Uid};
use egui::accesskit::Role;
use egui::{Event, Key, Modifiers, PointerButton, Pos2, Rect, Vec2};
use serde_json::json;

const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
const SCREEN: Vec2 = Vec2::new(1800.0, 1100.0);

fn load(name: &str) -> BtPanel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
    let text = std::fs::read_to_string(format!("{path}{name}")).unwrap();
    let mut panel = BtPanel::new(name);
    panel.load_text(name, &text).unwrap();
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

fn port<'a>(panel: &'a BtPanel, uid: Uid, port: &str) -> Option<&'a str> {
    ex(panel).tree().node(uid).port_value(port)
}

/// One widget of the last frame, as AccessKit describes it.
#[derive(Debug)]
struct Widget {
    role: Role,
    label: String,
    value: String,
    number: Option<f64>,
    /// Where it is, in its layer's coordinates (the screen's, except on the canvas).
    rect: Rect,
}

/// Draws frames of one panel with synthetic input, and reads back what they showed.
struct Driver {
    ctx: egui::Context,
    events: Vec<Event>,
    widgets: Vec<Widget>,
    /// The last frame asked for another one at once.
    repaint: bool,
}

impl Driver {
    fn new() -> Self {
        let ctx = egui::Context::default();
        // Text needs fonts to take up room (and to be clicked): egui has none of its own here.
        bt_view::fonts::install(&ctx);
        ctx.enable_accesskit();
        Self {
            ctx,
            events: Vec::new(),
            widgets: Vec::new(),
            repaint: false,
        }
    }

    /// One frame with the input queued so far.
    fn frame(&mut self, panel: &mut BtPanel) {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, SCREEN)),
            events: std::mem::take(&mut self.events),
            ..Default::default()
        };
        let mut output = self.ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
        });
        output.textures_delta.clear();
        self.repaint = output
            .viewport_output
            .get(&egui::ViewportId::ROOT)
            .is_some_and(|viewport| viewport.repaint_delay.is_zero());
        let nodes = output
            .platform_output
            .accesskit_update
            .map(|update| update.nodes)
            .unwrap_or_default();
        self.widgets = nodes
            .into_iter()
            .filter_map(|(_, node)| {
                let b = node.bounds()?;
                Some(Widget {
                    role: node.role(),
                    label: node.label().unwrap_or_default().to_string(),
                    value: node.value().unwrap_or_default().to_string(),
                    number: node.numeric_value(),
                    rect: Rect::from_min_max(
                        Pos2::new(b.x0 as f32, b.y0 as f32),
                        Pos2::new(b.x1 as f32, b.y1 as f32),
                    ),
                })
            })
            .collect();
    }

    /// The widget of this role that `matches`, in the last frame.
    fn find(&self, role: Role, matches: impl Fn(&Widget) -> bool) -> Option<&Widget> {
        self.widgets.iter().find(|w| w.role == role && matches(w))
    }

    /// The button labelled `label` (exactly, or with an icon before or after it).
    fn button(&self, label: &str) -> &Widget {
        let (prefix, suffix) = (format!("{label} "), format!(" {label}"));
        self.find(Role::Button, |w| {
            w.label == label || w.label.starts_with(&prefix) || w.label.ends_with(&suffix)
        })
        .unwrap_or_else(|| {
            let buttons: Vec<_> = self
                .widgets
                .iter()
                .filter(|w| w.role == Role::Button)
                .map(|w| &w.label)
                .collect();
            panic!("no button {label:?} among {buttons:?}")
        })
    }

    /// Move to `pos`, press and release `button`, a frame each.
    fn click_at(&mut self, panel: &mut BtPanel, pos: Pos2, button: PointerButton) {
        self.events.push(Event::PointerMoved(pos));
        self.frame(panel);
        for pressed in [true, false] {
            self.events.push(Event::PointerButton {
                pos,
                button,
                pressed,
                modifiers: Modifiers::NONE,
            });
            self.frame(panel);
        }
    }

    /// Press at `from`, drag to `to` over a few frames, and let go there.
    fn drag(&mut self, panel: &mut BtPanel, from: Pos2, to: Pos2) {
        self.events.push(Event::PointerMoved(from));
        self.frame(panel);
        let button = |pos, pressed| Event::PointerButton {
            pos,
            button: PointerButton::Primary,
            pressed,
            modifiers: Modifiers::NONE,
        };
        self.events.push(button(from, true));
        self.frame(panel);
        for i in 1..=4 {
            self.events
                .push(Event::PointerMoved(from + (to - from) * (i as f32 / 4.0)));
            self.frame(panel);
        }
        self.events.push(button(to, false));
        self.frame(panel);
    }

    /// Click the button labelled `label` (see [`button`](Self::button)). A frame is drawn
    /// first: a popup shown for the first time is laid out before it is placed.
    fn click(&mut self, panel: &mut BtPanel, label: &str) {
        self.frame(panel);
        let pos = self.button(label).rect.center();
        self.click_at(panel, pos, PointerButton::Primary);
    }

    /// Where a node's card is on the screen: the canvas is a scene, whose layer is the only
    /// one with a transform.
    fn card(&self, name: &str) -> Pos2 {
        let card = self
            .find(Role::Button, |w| w.label == name)
            .unwrap_or_else(|| panic!("no card {name}"));
        let layers: Vec<_> = self.ctx.memory(|m| m.layer_ids().collect());
        let to_screen = layers
            .into_iter()
            .find_map(|layer| self.ctx.layer_transform_to_global(layer))
            .expect("the canvas scene");
        to_screen * card.rect.center()
    }

    /// Select everything in the focused field and type `text` instead.
    fn type_text(&mut self, panel: &mut BtPanel, text: &str) {
        self.events.push(Event::Key {
            key: Key::A,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: Modifiers::COMMAND,
        });
        self.events.push(Event::Text(text.to_string()));
        self.frame(panel);
    }

    fn press(&mut self, panel: &mut BtPanel, key: Key) {
        self.events.push(Event::Key {
            key,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: Modifiers::NONE,
        });
        self.frame(panel);
    }

    fn focused(&self) -> bool {
        self.ctx.memory(|m| m.focused()).is_some()
    }
}

/// The welding plan with W01 selected, drawn until the inspector's grids have their widths.
fn welding_w01() -> (BtPanel, Driver, Uid) {
    let mut panel = load(WELDING);
    let w01 = by_name(&panel, "weld_robot1_W01");
    panel.select(Some(w01));
    let mut driver = Driver::new();
    for _ in 0..3 {
        driver.frame(&mut panel);
    }
    (panel, driver, w01)
}

#[test]
fn a_value_typed_before_a_run_is_in_effect_for_it() {
    let (mut panel, mut driver, w01) = welding_w01();
    let field = driver
        .find(Role::TextInput, |w| w.value == "robot1")
        .expect("W01's robot_ns field");
    // Wide enough to show and to click, whatever the column was before.
    assert!(field.rect.width() >= 149.0, "{:?}", field.rect);
    let centre = field.rect.center();
    driver.click_at(&mut panel, centre, PointerButton::Primary);
    assert!(driver.focused());
    driver.type_text(&mut panel, "robot2");
    // Not committed while typing.
    assert_eq!(port(&panel, w01, "robot_ns"), Some("robot1"));

    // The click on Step commits it (the field loses focus) before the Step runs.
    driver.click(&mut panel, "Step");
    assert_eq!(port(&panel, w01, "robot_ns"), Some("robot2"));
    assert_eq!(ex(&panel).ticks(), 1);
    let segment = panel
        .cell()
        .unwrap()
        .schedule()
        .segments
        .iter()
        .find(|s| s.uid == Some(w01))
        .expect("W01 moves");
    assert_eq!(segment.robot, "robot2");
}

#[test]
fn menus_do_what_they_say() {
    let (mut panel, mut driver, w01) = welding_w01();

    // Overrides ▾ → every action waits for the user; the menu closes.
    driver.click(&mut panel, "Overrides");
    driver.click(&mut panel, "All actions manual (wait for override)");
    assert_eq!(ex(&panel).policy(w01), LeafPolicy::Manual);
    driver.frame(&mut panel);
    assert!(
        driver
            .find(Role::Button, |w| w.label.starts_with("All actions"))
            .is_none()
    );
    driver.click(&mut panel, "Overrides");
    driver.click(&mut panel, "Cell leaves: simulate");
    assert_eq!(ex(&panel).policy(w01), LeafPolicy::Native);

    // ＋ port lists the cell leaf's ports it does not have yet.
    assert_eq!(port(&panel, w01, "start_delay_s"), None);
    driver.click(&mut panel, "port");
    assert!(
        driver
            .find(Role::Button, |w| w.label == "robot_ns")
            .is_none()
    );
    driver.click(&mut panel, "start_delay_s");
    assert_eq!(port(&panel, w01, "start_delay_s"), Some(""));

    // A node's right-click menu forces it.
    let card = driver.card("weld_robot1_W01");
    driver.click_at(&mut panel, card, PointerButton::Secondary);
    driver.click(&mut panel, "Force SUCCESS");
    assert_eq!(
        ex(&panel).override_of(w01),
        Some(Override {
            status: NodeStatus::Success,
            pinned: false
        })
    );
}

#[test]
fn a_number_port_can_be_typed_as_a_pointer() {
    let (mut panel, mut driver, w01) = welding_w01();
    assert_eq!(port(&panel, w01, "hold_duration_s"), Some("0.5"));
    let drag = driver
        .find(Role::SpinButton, |w| w.number == Some(0.5))
        .expect("W01's hold_duration_s");
    let centre = drag.rect.center();
    driver.click_at(&mut panel, centre, PointerButton::Secondary);
    driver.click(&mut panel, "Type a value…");
    // The field shows, focused, with the value to replace.
    driver.frame(&mut panel);
    assert!(driver.focused());
    assert!(driver.find(Role::TextInput, |w| w.value == "0.5").is_some());
    driver.type_text(&mut panel, "{speed}");
    driver.press(&mut panel, Key::Enter);
    assert_eq!(port(&panel, w01, "hold_duration_s"), Some("{speed}"));
    assert_eq!(port(&panel, w01, "velocity_scale"), Some("1.0"));
    // A pointer is edited as text from now on.
    driver.frame(&mut panel);
    assert!(
        driver
            .find(Role::TextInput, |w| w.value == "{speed}")
            .is_some()
    );
}

/// r1 moves for 2 s, then `wait_r1` waits for r1 to reach a pose it never reaches.
fn stall_plan() -> String {
    let wp = |t: f64| json!({ "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "time_s": t });
    let body = r#"<Sequence>
        <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
        <WaitForRobot name="wait_r1" robot_ns="r2" depends_on_joints="{deps}"/>
      </Sequence>"#;
    json!({
        "behaviour_tree": {
            "xml": format!(r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree">{body}</BehaviorTree></root>"#),
            "blackboard": {
                "a": [wp(0.0), wp(2.0)],
                "deps": [{ "robot": "r1", "joint_positions": [3.0, 0, 0, 0, 0, 0], "tolerance_rad": 0.05 }]
            }
        }
    })
    .to_string()
}

#[test]
fn auto_release_is_offered_where_gates_stall() {
    // Stopped for a manual leaf while W13 still moves: nothing about gates.
    let (mut panel, mut driver, w01) = welding_w01();
    panel.apply([bt_view::Action::Policy(w01, LeafPolicy::Manual)]);
    panel.run_to_wait();
    assert_eq!(panel.last_stop(), Some(&Stop::WaitingManual(vec![w01])));
    driver.frame(&mut panel);
    assert!(driver.button("Succeed").rect.is_positive());
    let auto_release = |d: &Driver| {
        d.find(Role::CheckBox, |w| w.label == "auto-release")
            .is_some()
    };
    assert!(!auto_release(&driver));

    // Stalled: the banner offers it, and ticking it releases and goes on.
    let mut stall = BtPanel::new("stall");
    stall.load_text("stall.json", &stall_plan()).unwrap();
    stall.run_to_wait();
    assert!(matches!(stall.last_stop(), Some(Stop::Stalled(_))));
    let mut driver = Driver::new();
    driver.frame(&mut stall);
    assert!(auto_release(&driver));
    let pos = driver
        .find(Role::CheckBox, |w| w.label == "auto-release")
        .unwrap()
        .rect
        .center();
    driver.click_at(&mut stall, pos, PointerButton::Primary);
    assert_eq!(
        stall.last_stop(),
        Some(&Stop::Finished(NodeStatus::Success))
    );
}

#[test]
fn a_long_halt_banner_scrolls_and_leaves_the_canvas_its_room() {
    let leaves: String = (0..60).map(|i| format!(r#"<Work name="w{i}"/>"#)).collect();
    let xml = format!(
        r#"<root BTCPP_format="4">
          <BehaviorTree ID="T"><Parallel success_count="-1" failure_count="1">{leaves}</Parallel></BehaviorTree>
          <TreeNodesModel><Action ID="Work"/></TreeNodesModel>
        </root>"#
    );
    let mut panel = BtPanel::new("many");
    panel.load_text("many.xml", &xml).unwrap();
    panel.apply([bt_view::Action::AllPolicies(
        bt_view::NodeKind::Action,
        LeafPolicy::Manual,
    )]);
    panel.run_to_wait();
    assert!(ex(&panel).halted().is_some());
    let mut driver = Driver::new();
    for _ in 0..3 {
        driver.frame(&mut panel);
    }
    // The status row sits right above the banner, at the timeline's top.
    let live = driver
        .find(Role::Button, |w| w.label.ends_with(" Live"))
        .expect("the Live button");
    assert!(live.rect.min.y > SCREEN.y / 2.0, "{:?}", live.rect);
}

/// The cell panel's bar for operation `op` on the Gantt chart (AccessKit names it
/// "{op} {robot} {from}–{to} s").
fn bar(driver: &Driver, op: &str) -> Rect {
    let prefix = format!("{op} ");
    driver
        .find(Role::Button, |w| {
            w.label.starts_with(&prefix) && w.label.ends_with(" s")
        })
        .unwrap_or_else(|| panic!("no bar for {op}"))
        .rect
}

#[test]
fn the_gantt_selects_a_bar_and_looks_back_elsewhere() {
    let mut panel = load(WELDING);
    panel.run_to_wait();
    let mut driver = Driver::new();
    for _ in 0..3 {
        driver.frame(&mut panel);
    }
    let ticks = ex(&panel).ticks();

    // A click on W07's bar selects its node; the run is still shown live.
    let w07 = bar(&driver, "W07");
    driver.click_at(&mut panel, w07.center(), PointerButton::Primary);
    let selected = by_name(&panel, "weld_robot2_W07");
    assert_eq!(panel.selected(), Some(selected));
    assert_eq!(panel.shown_tick(), ticks);

    // robot2's lane is empty between W13 (0 to 9.686 s: the weld and its hold) and W15 (from
    // 12.132 s): a click there looks back to that instant.
    let (w13, w15) = (bar(&driver, "W13"), bar(&driver, "W15"));
    let x = |t: f32| w13.left() + t / 9.686 * w13.width();
    let y = w13.center().y;
    assert!((x(12.132) - w15.left()).abs() < 1.0, "{w13:?} {w15:?}");
    driver.click_at(&mut panel, Pos2::new(x(10.9), y), PointerButton::Primary);
    assert!(
        (panel.shown_time() - 10.9).abs() < 0.05,
        "{}",
        panel.shown_time()
    );
    assert!(panel.shown_tick() < ticks);
    assert_eq!(panel.selected(), Some(selected));

    // A drag looks back along the way, wherever it starts (here on W15's bar).
    driver.drag(&mut panel, Pos2::new(x(15.0), y), Pos2::new(x(30.0), y));
    assert!(
        (panel.shown_time() - 30.0).abs() < 0.05,
        "{}",
        panel.shown_time()
    );
    assert_eq!(panel.selected(), Some(selected));

    // Held still mid-drag, the instant shown stays, and the panel stops asking for frames.
    let press = |pos, pressed| Event::PointerButton {
        pos,
        button: PointerButton::Primary,
        pressed,
        modifiers: Modifiers::NONE,
    };
    let (from, to) = (Pos2::new(x(20.0), y), Pos2::new(x(25.0), y));
    driver.events.push(Event::PointerMoved(from));
    driver.frame(&mut panel);
    driver.events.push(press(from, true));
    driver.frame(&mut panel);
    driver.events.push(Event::PointerMoved(to));
    driver.frame(&mut panel);
    assert!((panel.shown_time() - 25.0).abs() < 0.05);
    // Once the hover animations are done.
    for _ in 0..10 {
        driver.frame(&mut panel);
    }
    assert!(!driver.repaint);
    driver.events.push(press(to, false));
    driver.frame(&mut panel);
    assert!((panel.shown_time() - 25.0).abs() < 0.05);

    // The toolbar's Cell toggle hides the panel, and shows it again.
    for shown in [false, true] {
        let toggle = driver
            .find(Role::Button, |w| w.label == "Cell")
            .expect("the Cell toggle")
            .rect
            .center();
        driver.click_at(&mut panel, toggle, PointerButton::Primary);
        // It slides.
        for _ in 0..10 {
            driver.frame(&mut panel);
        }
        let bars = driver.find(Role::Button, |w| w.label.starts_with("W07 "));
        assert_eq!(bars.is_some(), shown);
    }
}

#[test]
fn a_note_is_about_the_tree_that_is_shown() {
    let (mut panel, mut driver, _) = welding_w01();
    let row = |d: &Driver| {
        d.find(Role::Label, |w| w.value.contains("plan.json"))
            .map(|w| w.value.clone())
    };
    // Something the embedder could not read: the welding plan is still on screen.
    panel.set_error("plan.json: cannot read plan.json");
    driver.frame(&mut panel);
    let error = row(&driver).expect("the error row");
    assert!(
        error.ends_with(&format!("(still showing {WELDING})")),
        "{error}"
    );

    // What the plan on screen loaded without.
    panel.set_note("plan.xml: loaded without the blackboard in plan.json");
    assert_eq!(
        panel.error(),
        Some("plan.xml: loaded without the blackboard in plan.json")
    );
    driver.frame(&mut panel);
    let note = row(&driver).expect("the error row");
    assert!(
        note.ends_with("loaded without the blackboard in plan.json"),
        "{note}"
    );
    driver.click(&mut panel, "Dismiss");
    assert_eq!(panel.error(), None);
}
