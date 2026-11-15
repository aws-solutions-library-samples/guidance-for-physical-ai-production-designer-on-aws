//! The cell panel under the canvas, while the cell leaves run natively: the cell at a glance,
//! where each robot is, the motion placed so far as a Gantt chart with one lane per robot, and
//! the warnings raised on the way.
//!
//! Like the rest of the panel it shows the run as of the instant on screen. Looking back, the
//! robots stand where they stood then, the header counts the operations placed by then, the
//! warnings are those about the run up to then, and the chart draws motion placed later as a
//! ghost outline; so is the rest of a motion as it was planned before a later cut, while a cut
//! that already happened is a red line with a ✂. Clicking a bar selects its node and brings it
//! into view; clicking or dragging anywhere else on the time axis looks back to that instant
//! ([`Action::SeekTime`]).
//!
//! The chart's geometry is a [`Layout`]; how a segment looks as of a tick ([`look`]), what is
//! under the pointer ([`hit`]) and the time axis ([`axis`]) are pure functions, tested below.
//! The axis always spans the whole run, and stays finite when an edited port makes a motion
//! endless (an `inf` hold).

use bt_cell::compat::CompatIssue;
use bt_cell::{CellLeaves, CellWarning, HaltedMotion, Schedule, Segment, WarningKind};
use bt_core::Uid;
use bt_exec::{EPS, Executor};
use egui::{
    Align2, Color32, CursorIcon, FontId, Id, Pos2, Rect, RichText, Sense, Shape, Stroke,
    StrokeKind, Ui, accesskit, pos2, vec2,
};

use crate::panel::{Action, BtPanel, PlanInfo};
use crate::style;

/// Height of the time axis under the lanes.
const AXIS_H: f32 = 18.0;
/// The least and the most height of a lane; the panel's height is shared out between them.
const LANE_MIN: f32 = 16.0;
const LANE_MAX: f32 = 30.0;
/// Room between a bar and its lane's edges.
const BAR_INSET: f32 = 3.0;
/// The narrowest a bar is drawn and hit, so that a short motion stays clickable.
const MIN_BAR_W: f32 = 3.0;
/// The least room between two axis labels.
const MIN_LABEL_SPACING: f32 = 64.0;
/// Room right of the lanes for the last axis label.
const RIGHT_PAD: f32 = 14.0;
/// A robot holding its pose: the motion's colour at this alpha.
const HOLD_ALPHA: f32 = 0.45;
/// A ghost's outline and fill: the motion's colour at these alphas.
const GHOST_ALPHA: f32 = 0.7;
const GHOST_FILL: f32 = 0.12;
/// A bar narrower than this has no label.
const MIN_LABELLED_W: f32 = 16.0;
/// A ghost narrower than this is outlined plainly: its dashes would not show.
const MIN_DASHED_W: f32 = 12.0;
/// The most time the chart spans: past it, a motion is drawn up to the axis' end.
const MAX_SPAN: f64 = 1e12;
/// How wide the robots grid starts out, at most (and as a share of the panel's width).
const ROBOTS_W: f32 = 470.0;
const ROBOTS_SHARE: f32 = 0.35;
/// How wide the warnings list starts out, at most (and as a share of the panel's width).
const WARNINGS_W: f32 = 360.0;
const WARNINGS_SHARE: f32 = 0.25;
/// The least width of the robots grid and of the warnings list.
const SIDE_MIN_W: f32 = 120.0;
/// The width the grid and the list leave to the chart, as long as they can.
const GANTT_MIN_W: f32 = 320.0;

/// What the cell panel shows: the run, as of the instant on screen.
struct View<'a> {
    /// The panel's id, salting every grid and side panel.
    id: Id,
    ex: &'a Executor,
    cell: &'a CellLeaves,
    /// The tick and instant on screen (scrubbed, or the latest).
    tick: u64,
    t: f64,
    /// The past is shown.
    scrubbed: bool,
    selected: Option<Uid>,
    /// What cell-runner would refuse in the file and the tree.
    compat: Vec<CompatIssue>,
}

impl View<'_> {
    /// Whether `warning` is about the run up to the instant shown. A warning has no tick: it
    /// is about the instant it was raised, or the one its motion starts at.
    fn sees(&self, warning: &CellWarning) -> bool {
        !self.scrubbed || warning.t <= self.t + EPS
    }
}

fn dim(text: impl Into<String>) -> RichText {
    RichText::new(text).small().color(style::TEXT_DIM)
}

fn plural(n: usize) -> &'static str {
    if n == 1 { "" } else { "s" }
}

/// Draw the cell panel of `panel`'s run (nothing unless its cell leaves run natively). What
/// the user asks for is returned, to be applied once the frame is drawn.
pub(crate) fn ui(panel: &mut BtPanel, ui: &mut Ui) -> Vec<Action> {
    let mut actions = Vec::new();
    let (tick, t, scrubbed, compat) = (
        panel.shown_tick(),
        panel.shown_time(),
        panel.scrub.is_some(),
        panel.compat_issues(),
    );
    let BtPanel {
        id,
        executor,
        plan,
        selected,
        show_cell_warnings,
        ..
    } = panel;
    let Some(ex) = executor.as_ref() else {
        return actions;
    };
    let Some(cell) = ex.leaf_handler::<CellLeaves>() else {
        return actions;
    };
    let view = View {
        id: *id,
        ex,
        cell,
        tick,
        t,
        scrubbed,
        selected: *selected,
        compat,
    };
    let shown_warnings = cell
        .schedule()
        .warnings
        .iter()
        .filter(|w| view.sees(w))
        .count();
    header(
        ui,
        &view,
        plan.as_ref(),
        (shown_warnings, show_cell_warnings),
        &mut actions,
    );
    ui.separator();
    // However narrow the panel, the grid and the list give way to the chart first.
    let list = *show_cell_warnings && shown_warnings > 0;
    let width = ui.available_width();
    let reserved = GANTT_MIN_W + if list { SIDE_MIN_W } else { 0.0 };
    egui::Panel::left(view.id.with("cell_robots"))
        .resizable(true)
        .default_size(ROBOTS_W.min(width * ROBOTS_SHARE))
        .min_size(SIDE_MIN_W)
        .max_size((width - reserved).max(SIDE_MIN_W))
        .show(ui, |ui| robots(ui, &view, &mut actions));
    if list {
        egui::Panel::right(view.id.with("cell_warnings"))
            .resizable(true)
            .default_size(WARNINGS_W.min(width * WARNINGS_SHARE))
            .min_size(SIDE_MIN_W)
            .max_size((ui.available_width() - GANTT_MIN_W).max(SIDE_MIN_W))
            .show(ui, |ui| warnings(ui, &view, shown_warnings, &mut actions));
    }
    gantt(ui, &view, &mut actions);
    actions
}

/// The `ExecuteTrajectory` operations placed by tick `upto` and not dropped by then, and how
/// many of them are process operations: [`Schedule::placed_count`] and
/// [`Schedule::operation_count`] as of a tick.
fn placed_by(schedule: &Schedule, upto: u64) -> (usize, usize) {
    let mut ops: Vec<(&str, bool)> = schedule
        .segments
        .iter()
        .filter(|s| s.visible_at(upto) && s.op_id != "home" && !s.op_id.ends_with(":interlock"))
        .map(|s| (s.op_id.as_str(), s.op_type != "move"))
        .collect();
    ops.sort_unstable();
    ops.dedup_by_key(|(op, _)| *op);
    let process = ops.iter().filter(|(_, process)| *process).count();
    (ops.len(), process)
}

// ───────────────────────────── header ─────────────────────────────

/// The cell and its plan, the instant shown, the warnings toggle (with how many warnings are
/// about the run up to that instant), what a tree halt does to a running motion, and what
/// cell-runner would refuse in this tree.
fn header(
    ui: &mut Ui,
    view: &View<'_>,
    plan: Option<&PlanInfo>,
    (warnings, show_warnings): (usize, &mut bool),
    actions: &mut Vec<Action>,
) {
    let schedule = view.cell.schedule();
    ui.horizontal_wrapped(|ui| {
        let mut about: Vec<String> = plan.and_then(|p| p.cell_id.clone()).into_iter().collect();
        about.push(
            plan.and_then(|p| p.tree_id.clone())
                .unwrap_or_else(|| view.ex.tree().tree_id.clone()),
        );
        let robots = schedule.robots.len();
        about.push(format!("{robots} robot{}", plural(robots)));
        let (placed, process) = placed_by(schedule, view.tick);
        let planned = plan.and_then(|p| p.num_operations);
        about.push(match planned {
            Some(planned) => format!("placed {placed} / {planned} planned"),
            None => format!("placed {placed}"),
        });
        ui.strong(about.join(" · ")).on_hover_ui(|ui| {
            ui.label(format!(
                "{placed} ExecuteTrajectory operation{} placed {} ({process} of them process \
                 operations, the rest moves){}",
                plural(placed),
                if view.scrubbed {
                    "by the tick shown"
                } else {
                    "so far"
                },
                planned.map_or(String::new(), |n| format!(
                    "; the plan has {n} (num_operations)"
                ))
            ));
        });
        ui.separator();
        let when = if view.scrubbed {
            format!("tick {}", view.tick)
        } else {
            "live".to_string()
        };
        ui.label(format!("t = {:.2} s ({when})", view.t));
        ui.separator();
        let total = schedule.warnings.len();
        if total == 0 {
            ui.label(dim("no warnings"));
        } else if warnings == 0 {
            ui.label(dim("no warnings yet"))
                .on_hover_text(format!("{total} later in the run"));
        } else {
            let sign = style::glyph(ui.ctx(), "⚠", "!");
            ui.toggle_value(
                show_warnings,
                RichText::new(format!("{sign} {warnings} warning{}", plural(warnings)))
                    .color(style::WARN),
            )
            .on_hover_text("Show or hide the list of warnings");
            if warnings < total {
                ui.label(dim(format!("+{} later", total - warnings)))
                    .on_hover_text("Warnings about the run after the instant shown");
            }
        }
        ui.separator();
        halted_motion(ui, view, actions);
    });
    compat_row(ui, view, actions);
}

/// "When a parent halts a motion: let it finish (cell-runner) | cut it there".
fn halted_motion(ui: &mut Ui, view: &View<'_>, actions: &mut Vec<Action>) {
    let text = |m: HaltedMotion| match m {
        HaltedMotion::Keep => "let it finish (cell-runner)",
        HaltedMotion::Cut => "cut it there",
    };
    let current = view.cell.options().halted_motion;
    let mut chosen = current;
    ui.label("When a parent halts a motion:");
    egui::ComboBox::from_id_salt(view.id.with("halted_motion"))
        .selected_text(text(current))
        .show_ui(ui, |ui| {
            for m in [HaltedMotion::Keep, HaltedMotion::Cut] {
                ui.selectable_value(&mut chosen, m, text(m));
            }
        })
        .response
        .on_hover_text(
            "What a running motion does when the tree stops it early (a Parallel that has \
             enough successes halts its other children, say), from the next such halt on. \
             cell-runner's motion leaves have no halt, so there it plays to its end. This is \
             not about the halt banner: while the run is halted, nothing moves. A motion \
             stopped by an override is always cut.",
        );
    if chosen != current {
        actions.push(Action::SetHaltedMotion(chosen));
    }
}

/// What cell-runner (behaviortree-rs) would refuse to run in this file and tree, first what
/// it would report; btgui runs it anyway.
fn compat_row(ui: &mut Ui, view: &View<'_>, actions: &mut Vec<Action>) {
    let issues = &view.compat;
    let Some(first) = issues.first() else {
        return;
    };
    let place = |issue: &CompatIssue| {
        format!(
            "{} · line {}",
            view.ex.tree().node(issue.uid).name,
            issue.line
        )
    };
    ui.horizontal_wrapped(|ui| {
        let info = style::glyph(ui.ctx(), "ℹ", "i");
        ui.label(RichText::new(format!("{info} cell-runner would reject:")).color(style::WARN))
            .on_hover_text("btgui runs the tree anyway; cell-runner's runtime refuses it");
        if ui.link(first.text()).on_hover_text(place(first)).clicked() {
            actions.push(Action::Focus(first.uid));
        }
        if issues.len() > 1 {
            ui.menu_button(format!("+{} more", issues.len() - 1), |ui| {
                for issue in &issues[1..] {
                    if ui
                        .button(format!("line {}: {}", issue.line, issue.text()))
                        .on_hover_text(place(issue))
                        .clicked()
                    {
                        actions.push(Action::Focus(issue.uid));
                        ui.close();
                    }
                }
            });
        }
    });
}

// ───────────────────────────── robots and warnings ─────────────────────────────

/// Where each robot is at the instant shown: its motion (click to bring the node into view),
/// its phase and progress, and its joints.
fn robots(ui: &mut Ui, view: &View<'_>, actions: &mut Vec<Action>) {
    let states = view.cell.robot_states(view.t, view.tick);
    if states.is_empty() {
        ui.label(dim("No robot is known yet."));
        return;
    }
    let degrees = style::glyph(ui.ctx(), "°", "deg");
    let scissors = style::glyph(ui.ctx(), "✂", "(cut)");
    egui::ScrollArea::both()
        .id_salt(view.id.with("cell_robots_scroll"))
        .auto_shrink([false, true])
        .show(ui, |ui| {
            egui::Grid::new(view.id.with("cell_robots_grid"))
                .num_columns(5)
                .striped(true)
                .show(ui, |ui| {
                    for heading in ["robot", "motion", "phase", "progress"] {
                        ui.label(dim(heading));
                    }
                    ui.label(dim(format!("joints ({degrees})")));
                    ui.end_row();
                    for robot in &states {
                        ui.label(&robot.robot_id);
                        match (&robot.op_id, robot.uid) {
                            (Some(op), Some(uid)) => {
                                let name = &view.ex.tree().node(uid).name;
                                if ui.link(op).on_hover_text(name).clicked() {
                                    actions.push(Action::Focus(uid));
                                }
                            }
                            (Some(op), None) => {
                                ui.label(op);
                            }
                            (None, _) => {
                                ui.label(dim("none yet"));
                            }
                        }
                        let phase = if robot.cut {
                            format!("{} {scissors}", robot.phase)
                        } else {
                            robot.phase.clone()
                        };
                        let op = robot.op_id.as_deref().unwrap_or_default();
                        ui.label(RichText::new(phase).color(style::phase(&robot.phase, op)));
                        ui.add(
                            egui::ProgressBar::new(robot.progress as f32)
                                .desired_width(90.0)
                                .text(format!("{:.0} %", robot.progress * 100.0)),
                        );
                        let joints: String = robot
                            .joints
                            .iter()
                            .map(|q| format!("{:7.1}", q.to_degrees()))
                            .collect();
                        let radians: Vec<String> =
                            robot.joints.iter().map(|q| format!("{q:.4}")).collect();
                        // Small, and monospace so that the columns line up.
                        let joints = RichText::new(joints)
                            .small()
                            .family(egui::FontFamily::Monospace);
                        ui.label(joints)
                            .on_hover_text(format!("[{}] rad", radians.join(", ")));
                        ui.end_row();
                    }
                });
        });
}

fn kind_name(kind: WarningKind) -> &'static str {
    match kind {
        WarningKind::Runner => "cell-runner",
        WarningKind::Jump => "jump",
        WarningKind::Overlap => "overlap",
        WarningKind::Cut => "cut",
        WarningKind::Override => "override",
        WarningKind::Limit => "limit",
    }
}

fn kind_color(kind: WarningKind) -> Color32 {
    match kind {
        WarningKind::Runner | WarningKind::Jump | WarningKind::Overlap => style::WARN,
        // What the user decided.
        WarningKind::Cut | WarningKind::Override => style::TEXT,
        WarningKind::Limit => style::ERROR,
    }
}

/// The `shown` warnings about the run up to the instant shown, one a line as
/// `kind · t · node · message` (the whole of it on hover); click one to bring its node into
/// view. Only the lines scrolled into view are laid out: a runaway tree raises thousands.
fn warnings(ui: &mut Ui, view: &View<'_>, shown: usize, actions: &mut Vec<Action>) {
    let list = &view.cell.schedule().warnings;
    ui.strong(format!("Warnings ({shown})"));
    let row_h = ui.text_style_height(&egui::TextStyle::Small);
    egui::ScrollArea::vertical()
        .id_salt(view.id.with("cell_warnings_scroll"))
        .auto_shrink([false, false])
        .show_rows(ui, row_h, shown, |ui, rows| {
            let rows = list
                .iter()
                .filter(|w| view.sees(w))
                .skip(rows.start)
                .take(rows.len());
            for warning in rows {
                let node = warning
                    .uid
                    .map_or("—", |uid| view.ex.tree().node(uid).name.as_str());
                let text = RichText::new(format!(
                    "{} · {:.2} s · {node} · {}",
                    kind_name(warning.kind),
                    warning.t,
                    warning.message
                ))
                .small()
                .color(kind_color(warning.kind));
                let response = ui.add(egui::Label::new(text).truncate().sense(Sense::click()));
                if let Some(uid) = warning.uid
                    && response.on_hover_cursor(CursorIcon::PointingHand).clicked()
                {
                    actions.push(Action::Focus(uid));
                }
            }
        });
}

// ───────────────────────────── Gantt chart ─────────────────────────────

/// Where the chart is on the screen: the robots' names on the left, one lane per robot, and
/// the time axis under the lanes. Time runs from 0 at the lanes' left edge to `t_max` at
/// their right edge.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Layout {
    /// The lanes, top to bottom in the schedule's robot order.
    lanes: Rect,
    lane_h: f32,
    t_max: f64,
}

impl Layout {
    /// The chart in `rect`: `names` wide for the robots' names, then `n` lanes `lane_h` high
    /// up to `t_max`.
    fn new(rect: Rect, names: f32, n: usize, lane_h: f32, t_max: f64) -> Self {
        let lanes = Rect::from_min_size(
            pos2(rect.left() + names, rect.top()),
            vec2(
                (rect.width() - names - RIGHT_PAD).max(1.0),
                n as f32 * lane_h,
            ),
        );
        Self {
            lanes,
            lane_h,
            t_max: t_max.max(EPS),
        }
    }

    /// Where instant `t` is (clamped to the axis; an undefined instant is at 0).
    fn x(&self, t: f64) -> f32 {
        let t = t.max(0.0).min(self.t_max);
        self.lanes.left() + (t / self.t_max) as f32 * self.lanes.width()
    }

    /// The instant at `x` (clamped to the axis).
    fn t(&self, x: f32) -> f64 {
        (f64::from((x - self.lanes.left()) / self.lanes.width()) * self.t_max)
            .clamp(0.0, self.t_max)
    }

    fn lane(&self, i: usize) -> Rect {
        Rect::from_min_size(
            pos2(self.lanes.left(), self.lanes.top() + i as f32 * self.lane_h),
            vec2(self.lanes.width(), self.lane_h),
        )
    }

    /// The lane under `pos`, if it is on one.
    fn lane_at(&self, pos: Pos2) -> Option<usize> {
        let n = (self.lanes.height() / self.lane_h).round() as usize;
        self.lanes
            .contains(pos)
            .then(|| (((pos.y - self.lanes.top()) / self.lane_h) as usize).min(n.max(1) - 1))
    }

    /// The bar from `t0` to `t1` in lane `i`, at least [`MIN_BAR_W`] wide.
    fn bar(&self, i: usize, t0: f64, t1: f64) -> Rect {
        let lane = self.lane(i).shrink2(vec2(0.0, BAR_INSET));
        let x0 = self.x(t0);
        Rect::from_x_y_ranges(x0..=self.x(t1).max(x0 + MIN_BAR_W), lane.y_range())
    }
}

/// The chart's span and axis step for motion up to `end`, on an axis `width` wide: 5 % of room
/// after `end`, rounded up to a whole step ([`nice_step`]). Both are finite, whatever `end`
/// is: the span is at least 1 s and at most about [`MAX_SPAN`].
fn axis(end: f64, width: f32) -> (f64, f64) {
    let end = if end.is_nan() { 0.0 } else { end };
    let span = end.clamp(1.0, MAX_SPAN) * 1.05;
    let step = nice_step(span, width);
    ((span / step).ceil() * step, step)
}

/// The smallest 1, 2 or 5 × 10ⁿ that splits `span` into steps at least [`MIN_LABEL_SPACING`]
/// apart on an axis `width` wide.
fn nice_step(span: f64, width: f32) -> f64 {
    let labels = f64::from(width / MIN_LABEL_SPACING).max(1.0);
    let least = (span / labels).max(EPS);
    let magnitude = 10f64.powi(least.log10().floor() as i32);
    [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|m| m * magnitude)
        .find(|&step| step >= least * (1.0 - 1e-9))
        .unwrap_or(10.0 * magnitude)
}

/// How a segment shows as of a tick.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Look {
    /// What it plays, as known at the tick: `(t0, t_end, t_done)`, the motion then the hold
    /// (none for a motion dropped before it started, or placed later).
    solid: Option<(f64, f64, f64)>,
    /// An outline of what is not played: motion placed after the tick, or the rest of a motion
    /// as it was planned before a cut after the tick.
    ghost: Option<(f64, f64)>,
    /// Where it was cut, if that happened by the tick.
    cut_at: Option<f64>,
}

impl Look {
    /// From the start to the end of what is drawn as a bar (none when only a cut is drawn).
    fn extent(&self) -> Option<(f64, f64)> {
        match (self.solid, self.ghost) {
            (Some((t0, _, t_done)), Some((_, end))) => Some((t0, t_done.max(end))),
            (Some((t0, _, t_done)), None) => Some((t0, t_done)),
            (None, ghost) => ghost,
        }
    }
}

/// How `s` shows as of tick `upto` (see the module docs).
fn look(s: &Segment, upto: u64) -> Look {
    if s.placed_tick > upto {
        // What it was planned as (a cut is later still), unless it never played.
        return Look {
            solid: None,
            ghost: (!s.is_dropped()).then(|| (s.t0, s.extent_at(upto).1)),
            cut_at: None,
        };
    }
    Look {
        solid: (!s.is_dropped()).then(|| (s.t0, s.t_end(), s.t_done())),
        ghost: s
            .cut
            .filter(|c| c.tick > upto)
            .map(|c| (c.at.max(s.t0), c.planned_t_done)),
        cut_at: s.cut.filter(|c| c.tick <= upto).map(|c| c.at),
    }
}

/// The segment whose bar is under `pos` as of tick `upto` (the one drawn last where bars
/// overlap), if any.
fn hit(schedule: &Schedule, layout: &Layout, upto: u64, pos: Pos2) -> Option<usize> {
    let lane = layout.lane_at(pos)?;
    let robot = schedule.robots.get(lane)?;
    schedule
        .segments
        .iter()
        .enumerate()
        .rev()
        .filter(|(_, s)| s.robot == *robot)
        .find(|(_, s)| {
            look(s, upto)
                .extent()
                .is_some_and(|(t0, t1)| layout.bar(lane, t0, t1).contains(pos))
        })
        .map(|(i, _)| i)
}

/// A rectangle's outline, dashed.
fn dashed_outline(rect: Rect, stroke: Stroke) -> Vec<Shape> {
    let corners = [
        rect.left_top(),
        rect.right_top(),
        rect.right_bottom(),
        rect.left_bottom(),
        rect.left_top(),
    ];
    Shape::dashed_line(&corners, stroke, 4.0, 3.0)
}

/// What a bar too narrow for a label draws in its lane, to the pixel. One that draws what the
/// one before it in the lane drew is left out: a runaway tree stacks thousands of them.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Sliver {
    /// The bar's left and right edge (the cut's, when only a cut is drawn).
    x: (i32, i32),
    color: Color32,
    solid: bool,
    ghost: bool,
    cut: bool,
    selected: bool,
}

/// The motion placed so far, one lane per robot, with the instant shown (see the module docs).
fn gantt(ui: &mut Ui, view: &View<'_>, actions: &mut Vec<Action>) {
    let schedule = view.cell.schedule();
    let robots = &schedule.robots;
    let n = robots.len().max(1);
    let size = ui.available_size();
    let lane_h = ((size.y - AXIS_H) / n as f32).clamp(LANE_MIN, LANE_MAX);
    let (response, painter) = ui.allocate_painter(
        vec2(size.x.max(0.0), AXIS_H + n as f32 * lane_h),
        Sense::click_and_drag(),
    );
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Other, true, "Motion timeline")
    });
    let font = FontId::proportional(11.0);
    let names = robots
        .iter()
        .map(String::as_str)
        .chain(["t (s)"])
        .map(|name| {
            let galley = painter.layout_no_wrap(name.to_string(), font.clone(), style::TEXT_DIM);
            galley.size().x
        })
        .fold(0.0, f32::max)
        + 12.0;
    let width = response.rect.width() - names - RIGHT_PAD;
    if width < MIN_LABEL_SPACING {
        painter.text(
            response.rect.left_center(),
            Align2::LEFT_CENTER,
            "Widen the panel to see the motion timeline",
            font,
            style::TEXT_DIM,
        );
        return;
    }
    // An edited port can make a motion endless (an `inf` hold): the axis spans what is finite.
    let end = [schedule.horizon(), view.ex.now(), view.t]
        .into_iter()
        .filter(|t| t.is_finite())
        .fold(0.0, f64::max);
    let (t_max, step) = axis(end, width);
    let layout = Layout::new(response.rect, names, n, lane_h, t_max);

    // Lanes with their robots, the grid and the axis.
    for i in 0..n {
        let lane = layout.lane(i);
        if i % 2 == 1 {
            painter.rect_filled(lane, 0, Color32::from_white_alpha(6));
        }
        let (at, align, name) = match robots.get(i) {
            Some(robot) => (lane.left() - 6.0, Align2::RIGHT_CENTER, robot.as_str()),
            None => (
                lane.left() + 6.0,
                Align2::LEFT_CENTER,
                "No robot is known yet.",
            ),
        };
        painter.text(
            pos2(at, lane.center().y),
            align,
            name,
            font.clone(),
            style::TEXT_DIM,
        );
    }
    let axis_y = layout.lanes.bottom();
    painter.hline(
        layout.lanes.x_range(),
        axis_y,
        Stroke::new(1.0, style::EDGE),
    );
    let decimals = (-step.log10().floor()).max(0.0) as usize;
    // `t_max` is a whole number of steps ([`axis`]), about a label's room apart each.
    let steps = (t_max / step).round() as usize;
    for k in 0..=steps {
        let t = k as f64 * step;
        let x = layout.x(t);
        painter.vline(x, layout.lanes.y_range(), Stroke::new(1.0, style::GRID_DOT));
        painter.vline(x, axis_y..=axis_y + 4.0, Stroke::new(1.0, style::EDGE));
        painter.text(
            pos2(x, axis_y + 4.0),
            Align2::CENTER_TOP,
            format!("{t:.decimals$}"),
            font.clone(),
            style::TEXT_DIM,
        );
    }
    painter.text(
        pos2(layout.lanes.left() - 6.0, axis_y + 4.0),
        Align2::RIGHT_TOP,
        "t (s)",
        font.clone(),
        style::TEXT_DIM,
    );

    // The segments, in the order placed: where bars overlap, the later one is on top. The axis
    // spans the whole run, so only lanes clipped away are culled, and a run of slivers that
    // draw the same pixels is drawn once.
    let scissors = style::glyph(ui.ctx(), "✂", "x");
    let clip = painter.clip_rect();
    let mut last_sliver: Vec<Option<Sliver>> = vec![None; n];
    for (index, s) in schedule.segments.iter().enumerate() {
        let Some(lane) = robots.iter().position(|r| *r == s.robot) else {
            continue;
        };
        let look = look(s, view.tick);
        let color = style::op_color(&s.op_type, &s.op_id);
        let span = look.extent();
        let extent = span.map(|(t0, t1)| layout.bar(lane, t0, t1));
        if !clip.intersects(extent.unwrap_or_else(|| layout.lane(lane))) {
            continue;
        }
        let selected = s.uid.is_some() && s.uid == view.selected;
        let sliver = match extent {
            Some(bar) if bar.width() >= MIN_LABELLED_W => None,
            _ => {
                let cut_x = look.cut_at.map_or(0, |at| layout.x(at).round() as i32);
                Some(Sliver {
                    x: extent.map_or((cut_x, cut_x), |bar| {
                        (bar.left().round() as i32, bar.right().round() as i32)
                    }),
                    color,
                    solid: look.solid.is_some(),
                    ghost: look.ghost.is_some(),
                    cut: look.cut_at.is_some(),
                    selected,
                })
            }
        };
        if sliver.is_some() && sliver == last_sliver[lane] {
            continue;
        }
        last_sliver[lane] = sliver;
        if let Some((t0, t_end, t_done)) = look.solid {
            let motion = layout.bar(lane, t0, t_end);
            painter.rect_filled(motion, 2, color);
            if t_done > t_end + EPS {
                let hold = Rect::from_x_y_ranges(
                    motion.right()..=layout.x(t_done).max(motion.right()),
                    motion.y_range(),
                );
                painter.rect_filled(hold, 2, color.gamma_multiply(HOLD_ALPHA));
            }
        }
        if let Some((from, to)) = look.ghost {
            let ghost = layout.bar(lane, from, to);
            let stroke = Stroke::new(1.0, color.gamma_multiply(GHOST_ALPHA));
            painter.rect_filled(ghost, 2, color.gamma_multiply(GHOST_FILL));
            if ghost.width() < MIN_DASHED_W {
                painter.rect_stroke(ghost, 2, stroke, StrokeKind::Inside);
            } else {
                painter.extend(dashed_outline(ghost, stroke));
            }
        }
        if let Some(at) = look.cut_at {
            let x = layout.x(at);
            let lane = layout.lane(lane);
            painter.vline(x, lane.y_range(), Stroke::new(2.0, style::ERROR));
            painter.text(
                pos2(x + 2.0, lane.top()),
                Align2::LEFT_TOP,
                scissors,
                FontId::proportional(10.0),
                style::ERROR,
            );
        }
        let (Some(bar), Some(span)) = (extent, span) else {
            continue;
        };
        let text = if look.solid.is_some() {
            style::TEXT
        } else {
            style::TEXT_DIM
        };
        if bar.width() >= MIN_LABELLED_W {
            let galley = painter.layout_no_wrap(s.op_id.clone(), font.clone(), text);
            if galley.size().x + 6.0 <= bar.width() {
                let at = pos2(bar.left() + 3.0, bar.center().y - galley.size().y / 2.0);
                painter.galley(at, galley, text);
            }
        }
        if selected {
            painter.rect_stroke(
                bar,
                2,
                Stroke::new(2.0, style::SELECTED),
                StrokeKind::Outside,
            );
        }
        accessible_bar(ui, response.id.with(index), s, bar, span, actions);
    }

    // Where the run is, and while looking back, the instant shown.
    let top = layout.lanes.top();
    if view.scrubbed {
        let x = layout.x(view.ex.now());
        painter.extend(Shape::dashed_line(
            &[pos2(x, top), pos2(x, axis_y)],
            Stroke::new(1.0, style::TEXT_DIM),
            4.0,
            3.0,
        ));
        painter.text(
            pos2(x + 3.0, top),
            Align2::LEFT_TOP,
            "now",
            FontId::proportional(10.0),
            style::TEXT_DIM,
        );
    }
    painter.vline(
        layout.x(view.t),
        top..=axis_y + 4.0,
        Stroke::new(1.5, style::TEXT),
    );

    // A bar selects its node; anywhere else on the time axis looks back to that instant.
    if response.clicked()
        && let Some(pos) = response.interact_pointer_pos()
    {
        match hit(schedule, &layout, view.tick, pos).and_then(|i| schedule.segments[i].uid) {
            Some(uid) => actions.push(Action::Focus(uid)),
            None if pos.x >= layout.lanes.left() => actions.push(Action::SeekTime(layout.t(pos.x))),
            None => {}
        }
    } else if response.dragged()
        // Held still, the instant shown stays: asking for it again would ask for a frame.
        && (response.drag_started() || response.drag_delta().x != 0.0)
        && let Some(pos) = response.interact_pointer_pos()
    {
        actions.push(Action::SeekTime(layout.t(pos.x)));
    }
    let hovered = response
        .hover_pos()
        .and_then(|pos| hit(schedule, &layout, view.tick, pos));
    if let Some(index) = hovered {
        ui.ctx().set_cursor_icon(CursorIcon::PointingHand);
        response.on_hover_ui_at_pointer(|ui| tooltip(ui, view, &schedule.segments[index]));
    }
}

/// A bar for assistive technology (and tests): a button named after its motion, which selects
/// its node like a click on the bar.
fn accessible_bar(
    ui: &Ui,
    id: Id,
    s: &Segment,
    bar: Rect,
    (t0, t1): (f64, f64),
    actions: &mut Vec<Action>,
) {
    ui.ctx().accesskit_node_builder(id, |node| {
        node.set_role(accesskit::Role::Button);
        node.set_label(format!("{} {} {t0:.2}–{t1:.2} s", s.op_id, s.robot));
        node.set_bounds(accesskit::Rect {
            x0: bar.min.x.into(),
            y0: bar.min.y.into(),
            x1: bar.max.x.into(),
            y1: bar.max.y.into(),
        });
        node.add_action(accesskit::Action::Click);
    });
    if let Some(uid) = s.uid
        && ui.input(|i| i.has_accesskit_action_request(id, accesskit::Action::Click))
    {
        actions.push(Action::Focus(uid));
    }
}

/// What a bar is: its motion and node, when it was placed, and how it was cut.
fn tooltip(ui: &mut Ui, view: &View<'_>, s: &Segment) {
    ui.strong(format!("{} ({}) · {}", s.op_id, s.op_type, s.robot));
    if let Some(uid) = s.uid {
        ui.label(&view.ex.tree().node(uid).name);
    }
    let (t_end, t_done) = s.extent_at(view.tick);
    let hold = if t_done > t_end + EPS {
        format!(", holds until {t_done:.3} s")
    } else {
        String::new()
    };
    ui.label(format!("moves {:.3}–{t_end:.3} s{hold}", s.t0));
    if s.placed_tick > view.tick {
        ui.label(dim(format!(
            "placed later, in tick {} (not yet at the tick shown)",
            s.placed_tick
        )));
    } else {
        ui.label(dim(format!("placed in tick {}", s.placed_tick)));
    }
    if let Some(cut) = s.cut {
        let text = match (cut.dropped, cut.tick <= view.tick) {
            (true, true) => format!("dropped before it started (tick {})", cut.tick),
            (false, true) => format!(
                "cut at {:.3} s (tick {}); it was planned until {:.3} s",
                cut.at, cut.tick, cut.planned_t_done
            ),
            (_, false) => format!("cut later, at {:.3} s (tick {})", cut.at, cut.tick),
        };
        ui.colored_label(style::ERROR, text);
    }
    ui.label(dim("Click to select its node"));
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bt_cell::{ExecuteOp, Placed, PlacedBy, PlanWaypoint};
    use bt_core::NodeStatus;
    use egui::accesskit::Role;

    use super::*;
    use crate::CellMode;

    const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
    const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";

    /// A weld on `robot` from `t0`: `secs` of motion, then `hold`, placed by `uid` in `tick`.
    fn place(
        schedule: &mut Schedule,
        robot: &str,
        (t0, secs, hold): (f64, f64, f64),
        uid: Uid,
        tick: u64,
    ) -> Placed {
        let wp = |time_s| PlanWaypoint {
            positions: vec![0.0; 6],
            time_s,
        };
        let op = ExecuteOp {
            robot: robot.to_string(),
            op_id: format!("op{uid}"),
            op_type: "weld".to_string(),
            traj: vec![wp(0.0), wp(secs)],
            start_delay_s: 0.0,
            hold_s: hold,
            velocity_scale: 1.0,
            interlock_joints: None,
        };
        let by = PlacedBy {
            uid: Some(uid),
            tick,
        };
        schedule.start_execute(&op, t0, by)
    }

    /// Lanes 1000 points wide for 10 s (100 points a second, from x = 100), 20 high.
    fn layout() -> Layout {
        let rect = Rect::from_min_size(Pos2::ZERO, vec2(1114.0, 58.0));
        Layout::new(rect, 100.0, 2, 20.0, 10.0)
    }

    /// The middle of lane `lane` at `t`.
    fn at(t: f64, lane: usize) -> Pos2 {
        pos2(100.0 + 100.0 * t as f32, 10.0 + 20.0 * lane as f32)
    }

    #[test]
    fn the_layout_maps_time_and_lanes() {
        let layout = layout();
        assert_eq!(
            layout.lanes,
            Rect::from_min_size(pos2(100.0, 0.0), vec2(1000.0, 40.0))
        );
        assert_eq!(layout.x(0.0), 100.0);
        assert_eq!(layout.x(5.0), 600.0);
        assert_eq!(layout.t(600.0), 5.0);
        // Off the axis, the nearest end.
        assert_eq!(layout.t(20.0), 0.0);
        assert_eq!(layout.t(5000.0), 10.0);
        assert_eq!(layout.lane_at(at(5.0, 0)), Some(0));
        assert_eq!(layout.lane_at(at(5.0, 1)), Some(1));
        assert_eq!(layout.lane_at(pos2(600.0, 40.0)), Some(1));
        assert_eq!(layout.lane_at(pos2(600.0, 45.0)), None);
        assert_eq!(layout.lane_at(pos2(50.0, 10.0)), None);
        // A bar keeps clear of its lane's edges, and a short one stays clickable.
        assert_eq!(
            layout.bar(1, 1.0, 2.0),
            Rect::from_min_max(pos2(200.0, 23.0), pos2(300.0, 37.0))
        );
        assert_eq!(layout.bar(0, 1.0, 1.0).width(), MIN_BAR_W);
    }

    #[test]
    fn the_axis_steps_are_one_two_five() {
        for (span, width, step) in [
            (79.06, 1200.0, 5.0),
            (79.06, 300.0, 20.0),
            (8.55, 1200.0, 0.5),
            (8.55, 800.0, 1.0),
            (1.05, 600.0, 0.2),
            (10.0, 64.0, 10.0),
            (20.0, 64.0, 20.0),
            (1000.0, 100.0, 1000.0),
        ] {
            let got = nice_step(span, width);
            assert!(
                close(got, step),
                "{span} s on {width} points: {got}, not {step}"
            );
        }
        // 5 % of room after the end, rounded up to a whole step.
        for (end, t_max, step) in [
            (75.2969, 80.0, 5.0),
            (8.1479, 9.0, 0.5),
            (100.0, 110.0, 10.0),
            // An empty timeline still has an axis.
            (0.0, 1.1, 0.1),
        ] {
            let (got_max, got_step) = axis(end, 1200.0);
            assert!(
                close(got_max, t_max) && close(got_step, step),
                "{end}: {got_max}, {got_step}"
            );
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn an_endless_motion_keeps_the_axis_finite() {
        for end in [
            f64::INFINITY,
            f64::NAN,
            f64::NEG_INFINITY,
            1.75e308,
            f64::MAX,
        ] {
            for width in [64.0, 1200.0] {
                let (t_max, step) = axis(end, width);
                assert!(
                    t_max.is_finite() && step > 0.0 && t_max / step <= 40.0,
                    "{end} on {width} points: {t_max}, {step}"
                );
            }
        }

        // An `inf` hold is accepted, and the chart still draws (up to the axis' end).
        let mut panel = load(WELDING);
        let w01 = uid(&panel, "weld_robot1_W01");
        panel.apply([Action::SetPort {
            uid: w01,
            port: "hold_duration_s".to_string(),
            value: Some("inf".to_string()),
        }]);
        panel.step();
        assert!(panel.cell().unwrap().schedule().horizon().is_infinite());
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            bars(&texts).iter().any(|bar| bar.starts_with("W01 ")),
            "{texts:?}"
        );
    }

    #[test]
    fn a_phase_is_in_the_colour_of_its_bar() {
        for (phase, op, color) in [
            ("move", "home", style::OP_HOME),
            ("move", "W01:interlock", style::OP_INTERLOCK),
            ("move", "STAGE_R1", style::OP_MOVE),
            ("weld", "W01", style::OP_WELD),
            ("hold", "home", style::HOLD),
            ("idle", "", style::TEXT_DIM),
        ] {
            assert_eq!(style::phase(phase, op), color, "{phase} {op}");
        }
    }

    #[test]
    fn a_click_hits_the_bar_drawn_under_it() {
        let robots = ["r1".to_string(), "r2".to_string()];
        let mut schedule = Schedule::new(&robots, BTreeMap::new());
        let a = place(&mut schedule, "r1", (0.0, 2.0, 1.0), 10, 1);
        place(&mut schedule, "r1", (5.0, 2.0, 0.0), 11, 2);
        let c = place(&mut schedule, "r2", (1.0, 4.0, 0.0), 12, 1);
        assert!(schedule.cut(&c, 2.0, 3));
        place(&mut schedule, "r2", (6.0, 1.0, 0.0), 13, 4);
        place(&mut schedule, "r2", (6.5, 2.0, 0.0), 14, 4);
        let f = place(&mut schedule, "r1", (8.0, 1.0, 0.0), 15, 5);
        assert!(schedule.cut(&f, 7.0, 6));
        // Planned to move until 9 s and hold until 9.5 s, then stopped at 8.5 s.
        let g = place(&mut schedule, "r2", (8.0, 1.0, 0.5), 16, 7);
        assert!(schedule.cut(&g, 8.5, 8));
        let (a, c, f, g) = (
            a.segments.start,
            c.segments.start,
            f.segments.start,
            g.segments.start,
        );
        let layout = layout();
        let hit = |pos: Pos2, upto: u64| hit(&schedule, &layout, upto, pos);
        const LIVE: u64 = u64::MAX;

        // The motion and the hold; nothing between two motions.
        assert_eq!(hit(at(1.0, 0), LIVE), Some(a));
        assert_eq!(hit(at(2.5, 0), LIVE), Some(a));
        assert_eq!(hit(at(4.0, 0), LIVE), None);
        // Motion placed after the tick shown is still there, as a ghost.
        assert_eq!(hit(at(6.0, 0), LIVE), Some(a + 1));
        assert_eq!(hit(at(6.0, 0), 1), Some(a + 1));
        // A cut ends the bar where it happened, but not before it happened.
        assert_eq!(hit(at(1.5, 1), LIVE), Some(c));
        assert_eq!(hit(at(3.0, 1), LIVE), None);
        assert_eq!(hit(at(3.0, 1), 2), Some(c));
        // Where bars overlap, the one placed last is on top.
        assert_eq!(hit(at(6.2, 1), LIVE), Some(c + 1));
        assert_eq!(hit(at(6.8, 1), LIVE), Some(c + 2));
        // A motion dropped before it started: only its plan, and only before the drop.
        assert_eq!(hit(at(8.5, 0), LIVE), None);
        assert_eq!(hit(at(8.5, 0), 5), Some(f));
        assert_eq!(hit(at(8.5, 0), 4), None);
        // A motion placed after the tick shown and cut later still: its plan as then.
        assert_eq!(hit(at(9.2, 1), 6), Some(g));
        assert_eq!(hit(at(9.2, 1), 7), Some(g));
        assert_eq!(hit(at(9.2, 1), LIVE), None);
        // The names, a lane's edge and the axis are not bars.
        assert_eq!(hit(pos2(50.0, 10.0), LIVE), None);
        assert_eq!(hit(pos2(200.0, 1.0), LIVE), None);
        assert_eq!(hit(pos2(200.0, 45.0), LIVE), None);

        // What is drawn for those.
        let look = |i: usize, upto| look(&schedule.segments[i], upto);
        assert_eq!(
            look(c, LIVE),
            Look {
                solid: Some((1.0, 2.0, 2.0)),
                ghost: None,
                cut_at: Some(2.0)
            }
        );
        assert_eq!(
            look(c, 2),
            Look {
                solid: Some((1.0, 2.0, 2.0)),
                ghost: Some((2.0, 5.0)),
                cut_at: None
            }
        );
        assert_eq!(look(c, 2).extent(), Some((1.0, 5.0)));
        assert_eq!(
            look(a + 1, 1),
            Look {
                solid: None,
                ghost: Some((5.0, 7.0)),
                cut_at: None
            }
        );
        assert_eq!(
            look(f, LIVE),
            Look {
                solid: None,
                ghost: None,
                cut_at: Some(7.0)
            }
        );
        assert_eq!(look(f, LIVE).extent(), None);
        assert_eq!(
            look(g, 6),
            Look {
                solid: None,
                ghost: Some((8.0, 9.5)),
                cut_at: None
            }
        );
    }

    fn load(name: &str) -> BtPanel {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../test_data/");
        let text = std::fs::read_to_string(format!("{path}{name}")).unwrap();
        let mut panel = BtPanel::new(name);
        panel.load_text(name, &text).unwrap();
        panel
    }

    fn uid(panel: &BtPanel, name: &str) -> Uid {
        let tree = panel.executor().unwrap().tree();
        tree.nodes.iter().find(|n| n.name == name).unwrap().uid
    }

    /// The labels of the Gantt chart's bars, as AccessKit has them.
    fn bars(texts: &[(Role, String)]) -> Vec<&str> {
        texts
            .iter()
            .filter(|(role, text)| *role == Role::Button && text.contains('–'))
            .map(|(_, text)| text.as_str())
            .collect()
    }

    fn has(texts: &[(Role, String)], wanted: impl Fn(&str) -> bool) -> bool {
        texts.iter().any(|(_, text)| wanted(text))
    }

    #[test]
    fn the_sample_plans_draw_live_and_scrubbed() {
        for (name, about, segments, end) in [
            (WELDING, "2 robots · placed 16 / 16 planned", 18, "75.30"),
            (TWO_ROBOT, "1 robot · placed 1 / 1 planned", 2, "8.15"),
        ] {
            let mut panel = load(name);
            let texts = crate::testing::draw(|ui| panel.ui(ui));
            // The robots are known before anything moves.
            assert!(
                has(&texts, |t| t.ends_with(" planned")
                    && t.contains("placed 0 /")),
                "{name}"
            );
            assert!(has(&texts, |t| t == "t = 0.00 s (live)"), "{name}");
            assert!(has(&texts, |t| t == "robot1"), "{name}");
            assert!(has(&texts, |t| t == "Motion timeline"), "{name}");
            assert!(bars(&texts).is_empty(), "{name}");

            panel.run_to_wait();
            let texts = crate::testing::draw(|ui| panel.ui(ui));
            assert!(has(&texts, |t| t.ends_with(about)), "{name}: {texts:?}");
            assert!(
                has(&texts, |t| t == format!("t = {end} s (live)")),
                "{name}"
            );
            assert!(has(&texts, |t| t == "no warnings"), "{name}");
            assert_eq!(bars(&texts).len(), segments, "{name}");

            // Looking back: the motion placed later is still drawn, as ghosts.
            panel.scrub_to_time(Some(3.0));
            let tick = panel.shown_tick();
            let texts = crate::testing::draw(|ui| panel.ui(ui));
            assert!(
                has(&texts, |t| t == format!("t = 3.00 s (tick {tick})")),
                "{name}"
            );
            assert_eq!(bars(&texts).len(), segments, "{name}");
        }
    }

    #[test]
    fn a_cut_shows_where_it_happened_and_as_planned_before() {
        let mut panel = load(WELDING);
        let w01 = uid(&panel, "weld_robot1_W01");
        panel.run_to_time(5.0);
        panel.apply([Action::Force(w01, NodeStatus::Success, false)]);
        panel.run_to_wait();
        assert!((panel.executor().unwrap().now() - 73.8509).abs() < 1e-3);
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            bars(&texts).contains(&"W01 robot1 0.00–5.00 s"),
            "{texts:?}"
        );
        assert!(has(&texts, |t| t.ends_with("2 warnings")));
        assert!(has(&texts, |t| {
            t == "cut · 5.00 s · weld_robot1_W01 · robot1 W01: cut at t=5.000s"
        }));
        assert!(has(&texts, |t| t
            .starts_with("jump · 10.69 s · weld_robot1_W02 · ")));

        // A narrow panel keeps room for the chart, beside the robots and the warnings.
        let texts = crate::testing::draw_on(vec2(800.0, 900.0), |ui| panel.ui(ui));
        assert!(has(&texts, |t| t == "Warnings (2)"), "{texts:?}");
        assert!(
            bars(&texts).contains(&"W01 robot1 0.00–5.00 s"),
            "{texts:?}"
        );

        // Before the cut, W01 runs to its planned end (the rest drawn as a ghost), fewer
        // operations are placed, and no warning is about the run up to then.
        panel.scrub_to_time(Some(3.0));
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            bars(&texts).contains(&"W01 robot1 0.00–11.13 s"),
            "{texts:?}"
        );
        assert!(has(&texts, |t| t.ends_with("2 robots · placed 2 / 16 planned")));
        assert!(has(&texts, |t| t == "no warnings yet"), "{texts:?}");
        assert!(!has(&texts, |t| t.starts_with("Warnings (")));
        // Just after the cut: its warning, not yet the jump it causes.
        panel.scrub_to_time(Some(6.0));
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(has(&texts, |t| t.ends_with(" 1 warning")), "{texts:?}");
        assert!(has(&texts, |t| t == "+1 later"));
        assert!(has(&texts, |t| t.starts_with("cut · 5.00 s · ")));
        assert!(!has(&texts, |t| t.starts_with("jump · ")));

        let mut panel = load(TWO_ROBOT);
        let stage = uid(&panel, "move_robot1_STAGE_R1");
        panel.run_to_time(2.0);
        panel.apply([Action::Force(stage, NodeStatus::Success, false)]);
        panel.run_to_wait();
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        let bars = bars(&texts);
        assert!(bars.contains(&"STAGE_R1 robot1 0.00–2.00 s"), "{bars:?}");
        assert!(bars.contains(&"home robot1 2.00–4.42 s"), "{bars:?}");
    }

    #[test]
    fn the_header_says_what_cell_runner_would_reject() {
        let mut panel = load("v4_cell_with_gates.xml");
        assert!(panel.cell().is_none());
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(!has(&texts, |t| t == "Motion timeline"));
        panel.set_cell_mode(CellMode::On);
        let texts = crate::testing::draw(|ui| panel.ui(ui));
        assert!(has(&texts, |t| t == "Motion timeline"));
        assert!(
            has(&texts, |t| t.ends_with(" cell-runner would reject:")),
            "{texts:?}"
        );
        let issues = panel.compat_issues();
        assert!(issues.len() > 1);
        // First the comment before <root>, which is what cell-runner reports.
        assert_eq!(
            issues[0].message,
            "Expected to find <root> start tag at start of XML. Found incorrect tag."
        );
        assert_eq!((issues[0].uid, issues[0].line), (0, 2));
        let more = format!("+{} more", issues.len() - 1);
        assert!(has(&texts, |t| t == issues[0].text()));
        assert!(has(&texts, |t| t == more), "{texts:?}");
    }
}
