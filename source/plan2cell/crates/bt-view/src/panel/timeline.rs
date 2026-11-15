//! The timeline under the canvas: the error row, the tick slider, and the halt banner that says
//! where the tree halts and why, with the decisions that let it go on.
//!
//! The banner shows while the live run cannot move without the user
//! ([`Executor::halted`]: a breakpoint, or nothing scheduled while manual leaves wait or gates
//! are stalled), and after a Run to wait that stopped for manual leaves while timers were still
//! pending (until the tree ticks again). Each decision in it also asks the run to go on
//! ([`Action::ContinueRun`]), so a virtual run continues where it stopped. The canvas rings the
//! stalled leaves this banner lists ([`halt`]).

use bt_core::{NodeStatus, Uid};
use bt_exec::{Executor, LeafPolicy, Stop};
use egui::{Id, RichText, Ui, WidgetText};

use super::{Action, BtPanel, label};
use crate::style;

/// Why the live run waits for the user.
#[derive(Clone, Debug, PartialEq)]
enum HaltKind {
    /// Nothing is scheduled: nothing moves until the user decides.
    Halted,
    /// Nodes are held at their breakpoints.
    Breakpoint,
    /// Run to wait stopped for manual leaves; timers are still pending.
    ManualStop { next: Option<f64> },
}

/// What the halt banner lists.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Halt {
    kind: HaltKind,
    /// Nodes held at their breakpoints.
    paused: Vec<Uid>,
    /// Manual leaves waiting for an override.
    manual: Vec<Uid>,
    /// Leaves nothing scheduled will finish (stalled gates), while the run is halted.
    pub(super) stalled: Vec<Uid>,
}

/// The most height the banner's rows take before they scroll, so the canvas keeps its room.
const MAX_ROWS_HEIGHT: f32 = 180.0;

/// What holds the live run, if anything does (see the module docs).
pub(super) fn halt(ex: &Executor, last_stop: Option<&Stop>, last_stop_tick: u64) -> Option<Halt> {
    match ex.halted() {
        Some(Stop::Breakpoint(paused)) => Some(Halt {
            kind: HaltKind::Breakpoint,
            paused,
            manual: ex.waiting_on(),
            stalled: Vec::new(),
        }),
        Some(_) => Some(Halt {
            kind: HaltKind::Halted,
            paused: Vec::new(),
            manual: ex.waiting_on(),
            stalled: ex.blocked(),
        }),
        None => {
            let manual_stop = matches!(last_stop, Some(Stop::WaitingManual(_)))
                && last_stop_tick == ex.ticks()
                && !ex.retick_pending()
                && !ex.is_done();
            let manual = if manual_stop {
                ex.waiting_on()
            } else {
                Vec::new()
            };
            (!manual.is_empty()).then(|| Halt {
                kind: HaltKind::ManualStop {
                    next: ex.next_wakeup(),
                },
                paused: Vec::new(),
                manual,
                stalled: Vec::new(),
            })
        }
    }
}

impl BtPanel {
    /// What the timeline asks for goes into `actions` (decisions, and the runs they go on with).
    pub(super) fn timeline(&mut self, ui: &mut Ui, actions: &mut Vec<Action>) {
        // A failed load leaves the old tree in place, so the error shows above it.
        if let Some(e) = &self.error {
            ui.horizontal_wrapped(|ui| {
                let cross = style::glyph(ui.ctx(), "✖", "x");
                let text = match &self.executor {
                    Some(_) if !self.error_is_note => {
                        format!("{cross} {e} (still showing {})", self.source)
                    }
                    _ => format!("{cross} {e}"),
                };
                ui.colored_label(style::ERROR, text);
                if ui.small_button("Dismiss").clicked() {
                    actions.push(Action::DismissError);
                }
            });
        }
        if self.executor.is_some() {
            self.status_row(ui);
        }
        self.drift_row(ui, actions);
        if let Some(ex) = &self.executor {
            // What holds the run now; the past has no decisions to offer.
            if self.scrub.is_none() {
                match halt(ex, self.last_stop.as_ref(), self.last_stop_tick) {
                    Some(halt) => banner(ui, self.id, ex, &halt, self.auto_release, actions),
                    None => waiting_row(ui, ex, actions),
                }
            }
            if self.last_stop == Some(Stop::TickLimit) {
                ui.horizontal_wrapped(|ui| {
                    ui.colored_label(
                        style::WARN,
                        format!(
                            "Stopped at the tick limit at t = {:.2} s: a tree that never settles?",
                            ex.now()
                        ),
                    );
                    if ui.small_button("Continue").clicked() {
                        actions.push(Action::ContinueRun);
                    }
                });
            }
            // The latest log line as of the tick shown.
            let shown_tick = self.shown_tick();
            // One line (the full text on hover), so a longer message does not grow the timeline
            // and shrink the canvas above it while the tree runs.
            if let Some(entry) = ex.log().iter().rev().find(|e| e.tick <= shown_tick) {
                ui.add(
                    egui::Label::new(
                        RichText::new(format!("tick {}: {}", entry.tick, entry.message))
                            .small()
                            .color(style::TEXT_DIM),
                    )
                    .truncate(),
                );
            }
        }
    }

    /// With an external source: whether it and the interpreter run the same motion leaves, and
    /// where they do not (each a link that brings the leaf into view).
    fn drift_row(&self, ui: &mut Ui, actions: &mut Vec<Action>) {
        let (Some(drift), Some(ex)) = (self.drift(), &self.executor) else {
            return;
        };
        let live = self.external.as_ref().map_or(0, Vec::len);
        // One line of one height whatever drifts: the canvas above takes the room the timeline
        // leaves, so a row that wraps or grows when a leaf drifts moves the tree with it. Leaves
        // past the pane's edge are clipped (the drift label lists them all).
        let id = self.id.with("drift_row_height");
        let tallest = ui.data(|d| d.get_temp::<f32>(id)).unwrap_or(0.0);
        let height = [
            tallest,
            ui.spacing().interact_size.y,
            ui.text_style_height(&egui::TextStyle::Body),
            ui.text_style_height(&egui::TextStyle::Button) + 2.0,
        ]
        .into_iter()
        .fold(0.0, f32::max);
        let row = ui.horizontal(|ui| {
            ui.set_min_height(height);
            let ring = style::glyph(ui.ctx(), "◯", "o");
            ui.label(RichText::new(format!("{ring} live cell: {live} running")).color(style::LIVE));
            if drift.is_empty() {
                ui.label(
                    RichText::new("· the interpreter agrees")
                        .small()
                        .color(style::TEXT_DIM),
                );
                return;
            }
            let mut list = |ui: &mut Ui, title: &str, uids: &[Uid]| {
                if uids.is_empty() {
                    return;
                }
                ui.label(RichText::new(title).color(style::WARN));
                for &uid in uids {
                    if ui.small_button(&ex.tree().node(uid).name).clicked() {
                        actions.push(Action::Focus(uid));
                    }
                }
            };
            let names = |title: &str, uids: &[Uid]| {
                uids.iter()
                    .map(|&uid| format!("{title}: {}", ex.tree().node(uid).name))
                    .collect::<Vec<_>>()
            };
            let all = [
                names("cell only", &drift.external_only),
                names("interpreter only", &drift.interpreter_only),
            ]
            .concat()
            .join("\n");
            ui.label(RichText::new("· drift:").color(style::WARN).strong())
                .on_hover_text(all);
            list(ui, "cell only", &drift.external_only);
            list(ui, "interpreter only", &drift.interpreter_only);
        });
        let tallest = row.response.rect.height().max(height);
        ui.data_mut(|d| d.insert_temp(id, tallest));
    }

    /// Root status and time at the instant shown (and, looking back, the live ones), the next
    /// event, and the tick slider for looking back.
    fn status_row(&mut self, ui: &mut Ui) {
        let (shown_tick, shown_time) = (self.shown_tick(), self.shown_time());
        let (scrub, scrub_t) = (self.scrub, self.scrub_t);
        let Some(ex) = &self.executor else {
            return;
        };
        let last = ex.trace().last_tick().max(ex.ticks());
        ui.horizontal(|ui| {
            let live = scrub.is_none();
            let root = if live {
                ex.root_status()
            } else {
                ex.trace().view_at(0, shown_tick).status
            };
            let color = style::status(root).unwrap_or(style::TEXT_DIM);
            let dot = style::glyph(ui.ctx(), "●", "■");
            // The labels before the slider change with the clock (and digits are not all as wide
            // in a proportional font): each is in a slot as wide as its widest value, so the
            // slider and the Live toggle after them do not move with every frame. The slots
            // widen once per digit the clock or the tick count gains.
            let root_text = |status: NodeStatus| RichText::new(format!("{dot} {status}")).strong();
            let statuses = [
                NodeStatus::Idle,
                NodeStatus::Running,
                NodeStatus::Success,
                NodeStatus::Failure,
                NodeStatus::Skipped,
            ];
            let width = widest(ui, statuses.map(root_text));
            slot(ui, width, root_text(root).color(color));
            let zeros = |n: f64| "0".repeat((n.max(0.0).floor() as u64).to_string().len().max(3));
            let (ticks, wakeup) = (ex.ticks(), ex.next_wakeup());
            let t_zeros = zeros(ex.now().max(shown_time).max(wakeup.unwrap_or(0.0)));
            let tick_zeros = "0".repeat(last.max(ticks).to_string().len());
            let width = widest(ui, [format!("tick {tick_zeros} · t = {t_zeros}.00 s")]);
            slot(
                ui,
                width,
                format!("tick {shown_tick} · t = {shown_time:.2} s"),
            );
            let next = match wakeup {
                _ if !live => format!(
                    "looking back · live: {} at {:.2} s (tick {ticks})",
                    ex.root_status(),
                    ex.now(),
                ),
                Some(t) if !ex.is_done() => format!("next event at {t:.2} s"),
                _ if ex.retick_pending() && ticks > 0 => "re-tick due now".to_string(),
                _ => String::new(),
            };
            let small = |text: String| RichText::new(text).small();
            let mut templates = vec![
                small(format!("next event at {t_zeros}.00 s")),
                small("re-tick due now".to_string()),
            ];
            if !live {
                templates.extend(statuses.map(|status| {
                    small(format!(
                        "looking back · live: {status} at {t_zeros}.00 s (tick {tick_zeros})"
                    ))
                }));
            }
            let width = widest(ui, templates);
            slot(ui, width, small(next).color(style::TEXT_DIM));
            ui.separator();
            let mut shown = scrub.unwrap_or(last);
            let width = (ui.available_width() - 200.0).max(100.0);
            ui.spacing_mut().slider_width = width;
            let slider = egui::Slider::new(&mut shown, 0..=last.max(1))
                .custom_formatter(|tick, _| {
                    let tick = tick.round().max(0.0) as u64;
                    // Scrubbed to an instant between two ticks: that instant.
                    let t = scrub_t
                        .filter(|_| scrub == Some(tick))
                        .or_else(|| ex.time_at_tick(tick));
                    match t {
                        Some(t) => format!("{tick} · {t:.2} s"),
                        None => tick.to_string(),
                    }
                })
                .custom_parser(|text| text.split('·').next()?.trim().parse().ok())
                .text("tick");
            // Its value field likewise, as wide as its widest value.
            let value = egui::WidgetText::from(format!("{tick_zeros} · {t_zeros}.00 s"))
                .into_galley(
                    ui,
                    Some(egui::TextWrapMode::Extend),
                    f32::INFINITY,
                    ui.style().drag_value_text_style.clone(),
                );
            // The padding, and room for the frame.
            let value = value.size().x + 2.0 * ui.spacing().button_padding.x + 4.0;
            let interact = ui.spacing().interact_size;
            ui.spacing_mut().interact_size.x = interact.x.max(value);
            let changed = ui.add_enabled(last > 0, slider).changed();
            ui.spacing_mut().interact_size = interact;
            if changed {
                self.scrub = (shown < last).then_some(shown);
                self.scrub_t = None;
                self.running = false;
            }
            if ui.selectable_label(live, format!("{dot} Live")).clicked() {
                self.scrub = None;
                self.scrub_t = None;
            }
        });
    }
}

/// How wide the widest of `texts` is, laid out on one line in `ui`.
fn widest(ui: &Ui, texts: impl IntoIterator<Item = impl Into<WidgetText>>) -> f32 {
    texts
        .into_iter()
        .map(|text| {
            text.into()
                .into_galley(
                    ui,
                    Some(egui::TextWrapMode::Extend),
                    f32::INFINITY,
                    egui::FontSelection::Default,
                )
                .size()
                .x
        })
        .fold(0.0, f32::max)
        .ceil()
}

/// A label, left-aligned in a slot `width` points wide (or as wide as it is, if wider): what
/// follows it on the row stays where it is as the text narrows and widens.
fn slot(ui: &mut Ui, width: f32, text: impl Into<WidgetText>) {
    let size = egui::vec2(width, ui.spacing().interact_size.y);
    ui.allocate_ui_with_layout(
        size,
        egui::Layout::left_to_right(egui::Align::Center),
        |ui| {
            ui.set_min_size(size);
            ui.label(text);
        },
    );
}

/// Without a halt: manual leaves that wait while something else is still scheduled.
fn waiting_row(ui: &mut Ui, ex: &Executor, actions: &mut Vec<Action>) {
    let waiting = ex.waiting_on();
    if waiting.is_empty() {
        return;
    }
    ui.horizontal_wrapped(|ui| {
        let hand = style::glyph(ui.ctx(), "✋", "⏸");
        ui.colored_label(style::WARN, format!("{hand} Waiting on manual leaves:"));
        for &uid in &waiting {
            if ui.link(&ex.tree().node(uid).name).clicked() {
                actions.push(Action::Focus(uid));
            }
        }
        ui.label("— right-click a node to force it, or");
        if ui.button("force all SUCCESS").clicked() {
            actions.extend(
                waiting
                    .iter()
                    .map(|&uid| Action::Force(uid, NodeStatus::Success, false)),
            );
        }
    });
}

/// The halt banner: what holds the run, why each node waits, and what the user can do.
fn banner(
    ui: &mut Ui,
    id: Id,
    ex: &Executor,
    halt: &Halt,
    auto_release: bool,
    actions: &mut Vec<Action>,
) {
    egui::Frame::new()
        .fill(style::HALT_FILL)
        .stroke(egui::Stroke::new(1.0, style::WARN))
        .corner_radius(4)
        .inner_margin(6)
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                let (now, tick) = (ex.now(), ex.ticks());
                let header = match &halt.kind {
                    HaltKind::Halted => format!(
                        "{} Halted at t = {now:.2} s (tick {tick}) — nothing moves until you \
                         decide",
                        style::glyph(ui.ctx(), "✋", "!")
                    ),
                    HaltKind::Breakpoint => format!(
                        "{} Paused at a breakpoint at t = {now:.2} s (tick {tick})",
                        style::glyph(ui.ctx(), "⏸", "||")
                    ),
                    HaltKind::ManualStop { next } => format!(
                        "{} Stopped for manual leaves at t = {now:.2} s{}",
                        style::glyph(ui.ctx(), "⏸", "||"),
                        next.map(|t| format!(" (next timer at {t:.2} s)"))
                            .unwrap_or_default()
                    ),
                };
                ui.label(RichText::new(header).strong().color(style::WARN));
                let can_continue = halt.kind != HaltKind::Halted;
                let hover = match halt.kind {
                    HaltKind::Breakpoint => "Let every paused node start, and go on",
                    _ => "Go on; the manual leaves keep waiting while other nodes run",
                };
                if ui
                    .add_enabled(
                        can_continue,
                        egui::Button::new(label(ui, "▶", ">", "Continue")),
                    )
                    .on_hover_text(hover)
                    .on_disabled_hover_text("Nothing can move until you decide below")
                    .clicked()
                {
                    actions.extend([Action::Resume, Action::ContinueRun]);
                }
                if !halt.stalled.is_empty()
                    && ui
                        .button("Release all stalled gates (like cell-runner)")
                        .on_hover_text(
                            "cell-runner's way out of a stall: every wait that started before \
                             now gives up once, with its \"never reaches\" warning",
                        )
                        .clicked()
                {
                    actions.extend([Action::ReleaseAllLikeCellRunner, Action::ContinueRun]);
                }
                // About stalled gates only, so only where gates are stalled.
                let mut on = auto_release;
                if !halt.stalled.is_empty()
                    && ui
                        .checkbox(&mut on, "auto-release")
                        .on_hover_text("Release stalled gates like cell-runner from now on")
                        .changed()
                {
                    actions.push(Action::SetAutoRelease(on));
                    if on {
                        actions.push(Action::ContinueRun);
                    }
                }
            });
            egui::ScrollArea::vertical()
                .id_salt(id.with("halt_scroll"))
                .max_height(MAX_ROWS_HEIGHT)
                .show(ui, |ui| {
                    egui::Grid::new(id.with("halt"))
                        .num_columns(3)
                        .striped(true)
                        .show(ui, |ui| {
                            let rows = halt
                                .paused
                                .iter()
                                .map(|&uid| (uid, Row::Breakpoint))
                                .chain(halt.manual.iter().map(|&uid| (uid, Row::Manual)))
                                .chain(halt.stalled.iter().map(|&uid| (uid, Row::Stalled)));
                            for (uid, row) in rows {
                                halt_row(ui, ex, uid, row, actions);
                                ui.end_row();
                            }
                        });
                });
        });
}

/// Why a node is in the banner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Row {
    Breakpoint,
    Manual,
    Stalled,
}

/// One banner row: the node (click to bring it into view), why it waits, and its decisions.
fn halt_row(ui: &mut Ui, ex: &Executor, uid: Uid, row: Row, actions: &mut Vec<Action>) {
    let node = ex.tree().node(uid);
    if ui
        .link(&node.name)
        .on_hover_text(format!("{} · line {}", node.id, node.line))
        .clicked()
    {
        actions.push(Action::Focus(uid));
    }
    let reason = ex.waiting_reason(uid).unwrap_or_default();
    ui.label(RichText::new(reason).small());
    ui.horizontal(|ui| {
        let mut decide = |ui: &mut Ui, text: &str, hover: &str, action: Action| {
            if ui.small_button(text).on_hover_text(hover).clicked() {
                actions.extend([action, Action::ContinueRun]);
            }
        };
        match row {
            Row::Stalled => {
                decide(
                    ui,
                    "Release",
                    "Force this gate SUCCESS (an override warning says how far its robot was)",
                    Action::Force(uid, NodeStatus::Success, false),
                );
                decide(
                    ui,
                    "Fail",
                    "Force this gate FAILURE",
                    Action::Force(uid, NodeStatus::Failure, false),
                );
            }
            Row::Manual => {
                decide(
                    ui,
                    "Succeed",
                    "Force SUCCESS",
                    Action::Force(uid, NodeStatus::Success, false),
                );
                decide(
                    ui,
                    "Fail",
                    "Force FAILURE",
                    Action::Force(uid, NodeStatus::Failure, false),
                );
                if ex.is_handled(uid) {
                    decide(
                        ui,
                        "Simulate",
                        "Run it with cell-runner's semantics from now",
                        Action::Policy(uid, LeafPolicy::Native),
                    );
                }
            }
            Row::Breakpoint => {
                let play = style::glyph(ui.ctx(), "▶", ">");
                decide(
                    ui,
                    &format!("{play} Continue"),
                    "Let it start",
                    Action::ResumeNode(uid),
                );
                decide(
                    ui,
                    "Remove",
                    "Remove the breakpoint and let it start",
                    Action::Breakpoint(uid, false),
                );
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::accesskit::Role;

    fn executor(xml: &str) -> Executor {
        let doc = bt_core::Document::parse(xml).unwrap();
        let tree = doc.instantiate(doc.main_tree_id()).unwrap();
        Executor::new(tree, Default::default())
    }

    const PARALLEL: &str = r#"<root BTCPP_format="4">
        <BehaviorTree ID="T">
          <Parallel success_count="-1" failure_count="1">
            <Wait name="manual"/><Work name="timed"/>
          </Parallel>
        </BehaviorTree>
        <TreeNodesModel><Action ID="Wait"/><Action ID="Work"/></TreeNodesModel>
      </root>"#;

    #[test]
    fn manual_stops_show_until_the_tree_ticks_again() {
        let mut ex = executor(PARALLEL);
        let (manual, timed) = (2, 3);
        assert_eq!(ex.tree().node(manual).name, "manual");
        ex.set_policy(manual, LeafPolicy::Manual);
        let report = ex.run(bt_exec::RunOptions::VIRTUAL);
        let stop = Stop::WaitingManual(vec![manual]);
        assert_eq!(report.stop, stop);
        // A timer is still pending: not a halt, but the stop shows while it is current.
        assert_eq!(ex.halted(), None);
        let shown = halt(&ex, Some(&stop), ex.ticks()).unwrap();
        assert_eq!(shown.kind, HaltKind::ManualStop { next: Some(1.0) });
        assert_eq!(shown.manual, vec![manual]);
        assert_eq!(halt(&ex, Some(&stop), ex.ticks() - 1), None);
        assert_eq!(halt(&ex, None, ex.ticks()), None);

        // Once the timer is done, nothing moves: a halt whatever the last stop was.
        ex.run(bt_exec::RunOptions {
            stop_on_manual: false,
            ..bt_exec::RunOptions::VIRTUAL
        });
        assert_eq!(ex.status(timed), NodeStatus::Success);
        let shown = halt(&ex, None, 0).unwrap();
        assert_eq!(shown.kind, HaltKind::Halted);
        assert_eq!(shown.manual, vec![manual]);
        assert!(shown.stalled.is_empty());

        // A decision makes the tree due: no banner until it ticked.
        ex.set_override(
            manual,
            Some(bt_exec::Override {
                status: NodeStatus::Success,
                pinned: false,
            }),
        );
        assert_eq!(halt(&ex, Some(&stop), ex.ticks()), None);
    }

    /// r1 moves `a` then `b`; r2 waits until r1 is at `pose`, then moves `c`.
    fn interlock(pose: f64) -> String {
        let wp =
            |t: f64, q: f64| serde_json::json!({ "positions": [q, 0, 0, 0, 0, 0], "time_s": t });
        let body = r#"<Parallel success_count="-1" failure_count="1">
            <Sequence>
              <ExecuteTrajectory name="A" robot_ns="r1" operation_id="A" trajectory="{a}"/>
              <ExecuteTrajectory name="B" robot_ns="r1" operation_id="B" trajectory="{b}"/>
            </Sequence>
            <Sequence>
              <WaitForRobot name="wait_r1" robot_ns="r2" depends_on_joints="{deps}"/>
              <ExecuteTrajectory name="C" robot_ns="r2" operation_id="C" trajectory="{c}"/>
            </Sequence>
          </Parallel>"#;
        serde_json::json!({
            "behaviour_tree": {
                "xml": format!(r#"<root BTCPP_format="4"><BehaviorTree ID="MainTree">{body}</BehaviorTree></root>"#),
                "blackboard": {
                    "a": [wp(0.0, 0.0), wp(2.0, 0.0)],
                    "b": [wp(0.0, 0.0), wp(1.5, 1.0)],
                    "c": [wp(0.0, 0.0), wp(1.0, 0.0)],
                    "deps": [{ "robot": "r1", "joint_positions": [pose, 0, 0, 0, 0, 0], "tolerance_rad": 0.05 }]
                }
            }
        })
        .to_string()
    }

    #[test]
    fn a_gate_is_stalled_only_where_the_run_halts() {
        // Waiting for motion that is still to be placed (B, once A is done): just RUNNING.
        let mut panel = BtPanel::new("interlock");
        panel.load_text("interlock.json", &interlock(1.0)).unwrap();
        panel.step();
        let ex = panel.executor().unwrap();
        let wait = ex
            .tree()
            .nodes
            .iter()
            .find(|n| n.name == "wait_r1")
            .unwrap()
            .uid;
        assert_eq!(ex.blocked(), vec![wait]);
        assert_eq!(ex.halted(), None);
        assert_eq!(
            halt(ex, panel.last_stop.as_ref(), panel.last_stop_tick),
            None
        );
        panel.run_to_wait();
        assert_eq!(panel.last_stop, Some(Stop::Finished(NodeStatus::Success)));

        // A pose r1 never reaches: once B is placed and done, nothing can move.
        let mut panel = BtPanel::new("stall");
        panel.load_text("stall.json", &interlock(3.0)).unwrap();
        panel.run_to_wait();
        assert_eq!(panel.last_stop, Some(Stop::Stalled(vec![wait])));
        let ex = panel.executor().unwrap();
        let shown = halt(ex, panel.last_stop.as_ref(), panel.last_stop_tick).unwrap();
        assert_eq!(shown.kind, HaltKind::Halted);
        assert_eq!(shown.stalled, vec![wait]);
    }

    #[test]
    fn the_tick_limit_line_offers_to_go_on() {
        let mut panel = BtPanel::new("limit");
        panel.load_text("parallel.xml", PARALLEL).unwrap();
        panel.last_stop = Some(Stop::TickLimit);
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            texts
                .iter()
                .any(|(_, t)| t.starts_with("Stopped at the tick limit at t = 0.00 s")),
            "{texts:?}"
        );
        let continues = |(role, text): &(Role, String)| *role == Role::Button && text == "Continue";
        assert_eq!(texts.iter().filter(|t| continues(t)).count(), 1);
    }

    /// With the live cell as an external source, the drift row says whether the interpreter
    /// agrees, and lists the leaves where it does not as buttons.
    #[test]
    fn the_drift_row_lists_the_disagreements() {
        let mut panel = BtPanel::new("drift");
        panel.load_text("stall.json", &interlock(1.0)).unwrap();
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            !texts.iter().any(|(_, t)| t.contains("live cell")),
            "no source, no row"
        );

        panel.run_to_time(0.5);
        let a = panel.uid_for_operation("r1", "A");
        assert_eq!(a.len(), 1);
        panel.apply([Action::SetExternalActive(vec![(a[0], "r1".into())])]);
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            texts.iter().any(|(_, t)| t == "· the interpreter agrees"),
            "{texts:?}"
        );

        panel.apply([Action::SetExternalActive(Vec::new())]);
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        let has = |role: Role, want: &str| texts.iter().any(|(r, t)| *r == role && t == want);
        assert!(has(Role::Label, "interpreter only"), "{texts:?}");
        assert!(has(Role::Button, "A"), "{texts:?}");
        assert!(!has(Role::Label, "cell only"));
    }

    /// Looking back, the status row shows the root and the time on screen, and says what is
    /// live.
    #[test]
    fn the_status_row_shows_the_instant_on_screen() {
        let mut panel = BtPanel::new("scrub");
        panel.load_text("stall.json", &interlock(1.0)).unwrap();
        panel.run_to_wait();
        let ex = panel.executor().unwrap();
        let (end, ticks) = (ex.now(), ex.ticks());
        assert_eq!(ex.root_status(), NodeStatus::Success);
        panel.scrub_to_time(Some(1.25));
        let tick = panel.shown_tick();
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        let has = |want: &str| texts.iter().any(|(_, t)| t == want);
        assert!(
            texts.iter().any(|(_, t)| t.ends_with(" RUNNING")),
            "{texts:?}"
        );
        assert!(
            !texts.iter().any(|(_, t)| t.ends_with(" SUCCESS")),
            "{texts:?}"
        );
        assert!(has(&format!("tick {tick} · t = 1.25 s")), "{texts:?}");
        assert!(
            has(&format!(
                "looking back · live: SUCCESS at {end:.2} s (tick {ticks})"
            )),
            "{texts:?}"
        );
        assert!(has(&format!("{tick} · 1.25 s")), "the slider: {texts:?}");
        // The log line is the latest one as of that tick.
        assert!(
            !texts.iter().any(|(_, t)| t.contains("tree finished")),
            "{texts:?}"
        );
    }

    /// Where the status row's Live toggle and tick slider are, drawn in a 520-point pane, in a
    /// context kept across frames like an app's.
    fn status_row_at(ctx: &egui::Context, panel: &mut BtPanel) -> Vec<f64> {
        let mut at = Vec::new();
        for _ in 0..2 {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(520.0, 980.0),
                )),
                ..Default::default()
            };
            let mut output = ctx.run_ui(input, |ui| {
                egui::CentralPanel::default().show(ui, |ui| panel.tree_ui(ui));
            });
            output.textures_delta.clear();
            let nodes = output.platform_output.accesskit_update.unwrap().nodes;
            let bounds = |n: &egui::accesskit::Node| n.bounds().map(|b| (b.x0, b.y0));
            let live = nodes
                .iter()
                .filter(|(_, n)| n.label().is_some_and(|l| l.ends_with(" Live")))
                .find_map(|(_, n)| bounds(n))
                .expect("a Live toggle");
            // The tick slider is the one in the timeline, under the canvas (the toolbar has one
            // too).
            let slider = nodes
                .iter()
                .filter(|(_, n)| n.role() == Role::Slider)
                .filter_map(|(_, n)| bounds(n))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .expect("a tick slider");
            at = vec![slider.0, live.0];
        }
        at
    }

    /// The status row's time and next event follow the clock while the tree runs: the tick
    /// slider and the Live toggle after them hold still (until the tick count gains a digit).
    #[test]
    fn the_status_row_holds_still_while_the_clock_runs() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../test_data/bt_1790934549_welding_cell_2_small.json"
        );
        let mut panel = BtPanel::new("welding");
        panel
            .load_text("welding.json", &std::fs::read_to_string(path).unwrap())
            .unwrap();
        let ctx = egui::Context::default();
        crate::fonts::install(&ctx);
        ctx.enable_accesskit();
        let mut seen: Vec<(usize, Vec<f64>)> = Vec::new();
        for t in [
            0.5, 1.11, 4.44, 9.99, 10.0, 12.34, 24.36, 24.41, 99.0, 100.0,
        ] {
            panel.run_to_time(t);
            let ex = panel.executor().unwrap();
            if ex.is_done() {
                break;
            }
            let digits = ex.trace().last_tick().max(ex.ticks()).to_string().len();
            let at = status_row_at(&ctx, &mut panel);
            if let Some((_, first)) = seen.iter().find(|(d, _)| *d == digits) {
                assert_eq!(&at, first, "t = {t}");
            }
            seen.push((digits, at));
        }
        assert!(seen.len() >= 4, "{seen:?}");
    }

    #[test]
    fn breakpoints_list_the_paused_nodes() {
        let mut ex = executor(PARALLEL);
        ex.set_breakpoint(3, true);
        ex.run(bt_exec::RunOptions::VIRTUAL);
        let shown = halt(&ex, None, 0).unwrap();
        assert_eq!(shown.kind, HaltKind::Breakpoint);
        assert_eq!(shown.paused, vec![3]);
    }
}
