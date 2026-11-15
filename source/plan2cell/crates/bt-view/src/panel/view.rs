//! The tree view in the middle of the panel: the cards with their status and badges (where the
//! run halts, breakpoints, overrides, edits, motion progress, what the live cell runs), inside a
//! pan/zoom scene, or an invitation to open a file while nothing is loaded.
//!
//! A node brought into view (where a run stopped, or one the user asked for) and, while
//! following, the leaves running at the instant shown are [revealed](reveal): the view stays
//! if it already shows them readably, and otherwise centres on them, zoomed in to read them.

use std::collections::{HashMap, HashSet};

use bt_cell::CellLeaves;
use bt_core::layout::Orientation;
use bt_core::status::NodeView;
use bt_core::{NodeKind, NodeStatus, Tree, Uid};
use bt_exec::LeafPolicy;
use egui::{Rect, RichText, Vec2};

use super::timeline::halt;
use super::{Action, BtPanel};
use crate::canvas::{self, Badges, Card};
use crate::{inspector, style};

impl BtPanel {
    pub(super) fn canvas(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let (shown_tick, shown_time) = (self.shown_tick(), self.shown_time());
        let live = self.scrub.is_none();
        let Some(ex) = &self.executor else {
            ui.vertical_centered(|ui| {
                ui.add_space((ui.available_height() / 2.0 - 40.0).max(0.0));
                ui.label(
                    RichText::new(
                        "Open a BehaviorTree.CPP XML file or a factoryos plan JSON (or drop one \
                         here)",
                    )
                    .color(style::TEXT_DIM),
                );
                if self.open_button && ui.button("Open…").clicked() {
                    actions.push(Action::RequestOpen);
                }
                if let Some(e) = &self.error {
                    ui.label(RichText::new(e).color(style::ERROR));
                }
            });
            return;
        };
        let tree = ex.tree();
        // A node to bring into view: open its collapsed ancestors; it is centred once laid out.
        // The selection stays the user's (or the embedder's).
        let focus = self.focus.take().filter(|&uid| uid < tree.len());
        if let Some(uid) = focus {
            let mut up = tree.node(uid).parent;
            while let Some(parent) = up {
                self.collapsed.remove(&parent);
                up = tree.node(parent).parent;
            }
        }
        let collapsed = &self.collapsed;
        let (cards, rects) = lay_out(
            ui.painter(),
            tree,
            collapsed,
            self.compact,
            self.orientation,
        );
        // The canvas changed size since the last frame (a footer row came or went, an inspector
        // or the pane was resized): the graph stays where it was on screen, at its zoom, rather
        // than egui's Scene fitting the same rect into the new size (which zooms and moves it
        // with every such change). A fitted view goes on framing the whole tree.
        let outer = ui.available_size_before_wrap();
        let resized = self.view_outer != outer;
        if self.fitted != Some(self.scene_rect) {
            self.fitted = None;
            self.scene_rect = keep_zoom(self.scene_rect, self.view_outer, outer);
        }
        self.view_outer = outer;
        if self.fit {
            let bounds = rects
                .iter()
                .flatten()
                .fold(Rect::NOTHING, |acc, r| acc.union(*r));
            if bounds.is_positive() {
                self.scene_rect = bounds.expand(60.0);
                self.fitted = Some(self.scene_rect);
            }
            self.fit = false;
        }
        let views: Vec<NodeView> = (0..tree.len())
            .map(|uid| ex.trace().view_at(uid, shown_tick))
            .collect();
        // The leaves running at the instant shown, each as its nearest card in view (a
        // collapsed SubTree hides what runs inside it).
        let mut running: Vec<Uid> = Vec::new();
        for node in tree.nodes.iter().filter(|n| n.kind.is_leaf()) {
            if views[node.uid].status != NodeStatus::Running {
                continue;
            }
            let mut shown = Some(node.uid);
            while let Some(uid) = shown.filter(|&uid| rects[uid].is_none()) {
                shown = tree.node(uid).parent;
            }
            if let Some(uid) = shown.filter(|uid| !running.contains(uid)) {
                running.push(uid);
            }
        }
        // What the external source runs, likewise on the nearest card in view, with its robots.
        let mut on_cell: HashMap<Uid, Vec<&str>> = HashMap::new();
        for (uid, robot) in self.external.iter().flatten() {
            let mut shown = Some(*uid);
            while let Some(uid) = shown.filter(|&uid| rects[uid].is_none()) {
                shown = tree.node(uid).parent;
            }
            if let Some(robots) = shown.map(|uid| on_cell.entry(uid).or_default())
                && !robots.contains(&robot.as_str())
            {
                robots.push(robot);
            }
        }
        let union = |uids: &[Uid]| {
            uids.iter()
                .filter_map(|&uid| rects[uid])
                .reduce(|a, b| a.union(b))
        };
        match focus.and_then(|uid| rects[uid]) {
            Some(target) => self.scene_rect = reveal(self.scene_rect, outer, target),
            None if self.follow && running != self.followed => {
                if let Some(target) = union(&running) {
                    self.scene_rect = reveal(self.scene_rect, outer, target);
                }
            }
            // The same leaves run on a canvas that changed size (the pane or an inspector was
            // resized, a footer row came): they stay in view, the view moving no more than that
            // needs (a footer row that comes and goes leaves a view that shows them alone).
            None if self.follow && resized => {
                if let Some(target) = union(&running) {
                    self.scene_rect = keep_in_view(self.scene_rect, outer, target);
                }
            }
            None => {}
        }
        if self.follow {
            self.followed = running;
        }
        // What holds the run now; the past shows statuses only.
        let now_set = |uids: Vec<Uid>| -> HashSet<Uid> {
            if live {
                uids.into_iter().collect()
            } else {
                HashSet::new()
            }
        };
        let waiting = now_set(ex.waiting_on());
        // Stalled where the halt banner says so: a gate waiting for motion that is still to be
        // placed is just RUNNING.
        let stalled = now_set(
            halt(ex, self.last_stop.as_ref(), self.last_stop_tick)
                .map(|halt| halt.stalled)
                .unwrap_or_default(),
        );
        let held = now_set(ex.paused_at().to_vec());
        let edited: HashSet<Uid> = ex.port_edits().into_iter().map(|(uid, _)| uid).collect();
        let cell = ex.leaf_handler::<CellLeaves>();
        let orientation = self.orientation;
        let selected = self.selected;

        let scene = egui::Scene::new().zoom_range(0.05..=2.5);
        let response = scene
            .show(ui, &mut self.scene_rect, |ui| {
                let painter = ui.painter().clone();
                canvas::paint_grid(&painter, ui.clip_rect());

                // Edges first, coloured by the child's status as in Groot.
                for node in &tree.nodes {
                    let Some(rect) = rects[node.uid] else {
                        continue;
                    };
                    if collapsed.contains(&node.uid) {
                        continue;
                    }
                    for &child in &node.children {
                        let Some(child_rect) = rects[child] else {
                            continue;
                        };
                        let (from, _) = canvas::ports(rect, orientation);
                        let (_, to) = canvas::ports(child_rect, orientation);
                        let view = views[child];
                        let (color, width) =
                            match (style::status(view.status), view.was.and_then(style::was)) {
                                (Some(c), _) => (c, 3.0),
                                (None, Some(c)) => (c, 2.5),
                                (None, None) => (style::EDGE, 2.0),
                            };
                        canvas::paint_edge(&painter, from, to, orientation, color, width);
                    }
                }

                for node in &tree.nodes {
                    let Some(rect) = rects[node.uid] else {
                        continue;
                    };
                    let uid = node.uid;
                    let response = canvas::interact(ui, rect, uid, &node.name);
                    let moves = matches!(node.id.as_str(), "ExecuteTrajectory" | "MoveHome");
                    let badges = Badges {
                        selected: selected == Some(uid),
                        forced: ex.override_of(uid).map(|o| o.status),
                        manual: node.kind == NodeKind::Action
                            && bt_exec::is_stub(&node.id, node.kind)
                            && ex.policy(uid) == LeafPolicy::Manual,
                        waiting: waiting.contains(&uid),
                        stalled: stalled.contains(&uid),
                        breakpoint: ex.breakpoint(uid),
                        held: held.contains(&uid),
                        edited: edited.contains(&uid),
                        progress: cell
                            .filter(|_| moves && node.kind.is_leaf())
                            .and_then(|cell| cell.progress(uid, shown_time, shown_tick)),
                        live: on_cell.contains_key(&uid),
                    };
                    cards[uid].paint(&painter, rect, node, views[uid], badges);
                    if let Some(robots) = on_cell.get(&uid) {
                        // Which robot the cell runs it on, above the live ring.
                        painter.text(
                            rect.left_top() + Vec2::new(0.0, -14.0),
                            egui::Align2::LEFT_BOTTOM,
                            format!("live: {}", robots.join(", ")),
                            egui::FontId::proportional(style::PORT_SIZE),
                            style::LIVE,
                        );
                    }
                    let (out_port, in_port) = canvas::ports(rect, orientation);
                    if node.kind != NodeKind::Root {
                        canvas::paint_port(&painter, in_port);
                    }
                    if !node.children.is_empty() {
                        canvas::paint_port(&painter, out_port);
                    }

                    if response.clicked() {
                        actions.push(Action::Select(Some(uid)));
                    }
                    if response.double_clicked() && node.kind == NodeKind::SubTree {
                        actions.push(Action::ToggleCollapse(uid));
                    }
                    let response = response
                        .on_hover_ui(|ui| inspector::tooltip(ui, ex, uid, views[uid], live));
                    response.context_menu(|ui| {
                        inspector::context_menu(ui, ex, uid, !live, actions);
                    });
                }
            })
            .response;
        if response.clicked() {
            actions.push(Action::Select(None));
        }
    }
}

/// The cards of `tree`, measured with `painter`, and where they are in the scene (`None` for
/// those inside a collapsed SubTree).
pub(super) fn lay_out(
    painter: &egui::Painter,
    tree: &Tree,
    collapsed: &HashSet<Uid>,
    compact: bool,
    orientation: Orientation,
) -> (Vec<Card>, Vec<Option<Rect>>) {
    let cards: Vec<Card> = tree
        .nodes
        .iter()
        .map(|n| Card::new(painter, tree, n, collapsed.contains(&n.uid), compact))
        .collect();
    let rects = canvas::place(tree, &cards, orientation, |uid| !collapsed.contains(&uid));
    (cards, rects)
}

/// The view `scene_rect`, shown on a canvas of `before` points, for a canvas of `now` points: at
/// the same zoom, with the same scene point at the canvas's top-left corner, so a canvas that
/// grows or shrinks shows more or less at its bottom and right and nothing on it moves. (egui's
/// Scene fits `scene_rect` into the canvas, letterboxed and centred, every frame.)
pub(super) fn keep_zoom(scene_rect: Rect, before: Vec2, now: Vec2) -> Rect {
    if !scene_rect.is_positive()
        || before.min_elem() <= 0.0
        || now.min_elem() <= 0.0
        || before == now
    {
        return scene_rect;
    }
    let zoom = (before / scene_rect.size()).min_elem();
    // What the canvas showed: the scene rect, letterboxed.
    let visible = Rect::from_center_size(scene_rect.center(), before / zoom);
    Rect::from_min_size(visible.min, now / zoom)
}

/// Below this zoom a card's text is too small to read.
const READABLE_ZOOM: f32 = 0.6;
/// How far [`reveal`] zooms in, at most (unless the user zoomed further in already).
const REVEAL_ZOOM: f32 = 1.0;
/// Room around what is revealed, in scene units.
const REVEAL_MARGIN: f32 = 40.0;

/// The view (`scene_rect`, on a canvas of `outer` points) that shows `target`: the same one if
/// it already shows `target` whole at a readable zoom; else centred on `target`, at the zoom it
/// had if that is readable and otherwise at 1:1, but zoomed out as far as `target` needs.
pub(super) fn reveal(scene_rect: Rect, outer: Vec2, target: Rect) -> Rect {
    if !scene_rect.is_positive() || outer.min_elem() <= 0.0 {
        return scene_rect;
    }
    let zoom = (outer / scene_rect.size()).min_elem();
    // What the canvas shows: the scene rect, letterboxed.
    let visible = Rect::from_center_size(scene_rect.center(), outer / zoom);
    let target = target.expand(REVEAL_MARGIN);
    let readable = zoom >= READABLE_ZOOM;
    if readable && visible.contains_rect(target) {
        return scene_rect;
    }
    let fits = (outer / target.size()).min_elem();
    let zoom = if readable { zoom } else { REVEAL_ZOOM }.min(fits);
    Rect::from_center_size(target.center(), outer / zoom)
}

/// The view (`scene_rect`, on a canvas of `outer` points) that keeps `target` in view: the same
/// one if it shows `target` whole (within half a point); else at the same zoom unless `target`
/// no longer fits (then zoomed out just as far as it needs, with [`REVEAL_MARGIN`] around it),
/// panned from the same top-left corner as little as brings `target` in, with as much of the
/// margin as there is room for. Unlike [`reveal`] it neither centres nor zooms in: the view, and
/// the zoom, stay the user's.
pub(super) fn keep_in_view(scene_rect: Rect, outer: Vec2, target: Rect) -> Rect {
    if !scene_rect.is_positive() || outer.min_elem() <= 0.0 || !target.is_positive() {
        return scene_rect;
    }
    let zoom = (outer / scene_rect.size()).min_elem();
    // What the canvas shows: the scene rect, letterboxed.
    let visible = Rect::from_center_size(scene_rect.center(), outer / zoom);
    if visible.expand(0.5 / zoom).contains_rect(target) {
        return scene_rect;
    }
    let fits = |room: Rect| (outer / room.size()).min_elem();
    let zoom = if zoom <= fits(target) {
        zoom
    } else {
        zoom.min(fits(target.expand(REVEAL_MARGIN)))
    };
    let size = outer / zoom;
    // Along one axis: the view's start, moved as little as shows `lo..hi` with up to the margin
    // around it.
    let pan = |start: f32, len: f32, lo: f32, hi: f32| {
        let margin = ((len - (hi - lo)) / 2.0).clamp(0.0, REVEAL_MARGIN);
        let (lo, hi) = (lo - margin, hi + margin);
        if hi - lo >= len {
            lo - (len - (hi - lo)) / 2.0
        } else if lo < start {
            lo
        } else if hi > start + len {
            hi - len
        } else {
            start
        }
    };
    let min = egui::pos2(
        pan(visible.min.x, size.x, target.min.x, target.max.x),
        pan(visible.min.y, size.y, target.min.y, target.max.y),
    );
    Rect::from_min_size(min, size)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BtPanel;
    use egui::{pos2, vec2};

    fn zoom(scene: Rect, outer: Vec2) -> f32 {
        (outer / scene.size()).min_elem()
    }

    /// The zoom of a view on a canvas of `outer` points, and the scene point at its top-left.
    fn shown(scene: Rect, outer: Vec2) -> (f32, egui::Pos2) {
        let zoom = zoom(scene, outer);
        (
            zoom,
            Rect::from_center_size(scene.center(), outer / zoom).min,
        )
    }

    #[test]
    fn revealing_zooms_in_only_when_it_must() {
        let outer = vec2(1000.0, 600.0);
        // The welding plan fitted: a scene 6000 wide, zoom 1/6.
        let fitted = Rect::from_min_size(pos2(0.0, 0.0), vec2(6000.0, 1000.0));
        let node = Rect::from_min_size(pos2(5000.0, 500.0), vec2(200.0, 150.0));
        let shown = reveal(fitted, outer, node);
        assert!((zoom(shown, outer) - REVEAL_ZOOM).abs() < 1e-4);
        assert!((shown.center() - node.center()).length() < 1e-3);

        // Readable and in view: the view stays.
        assert_eq!(reveal(shown, outer, node), shown);
        let moved = node.translate(vec2(100.0, 0.0));
        assert_eq!(reveal(shown, outer, moved), shown);
        // Readable but out of view: it pans at the zoom the user chose.
        let mine = Rect::from_center_size(pos2(0.0, 0.0), outer / 0.8);
        let panned = reveal(mine, outer, node);
        assert!((zoom(panned, outer) - 0.8).abs() < 1e-4);
        assert!((panned.center() - node.center()).length() < 1e-3);
        // Too big for 1:1: zoomed out just as far as it needs.
        let wide = Rect::from_min_size(pos2(0.0, 0.0), vec2(1920.0, 100.0));
        let shown = reveal(fitted, outer, wide);
        assert!((zoom(shown, outer) - 0.5).abs() < 1e-4);
        // Nothing laid out yet: nothing to do.
        assert_eq!(reveal(Rect::ZERO, outer, node), Rect::ZERO);
    }

    fn welding() -> BtPanel {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../test_data/bt_1790934549_welding_cell_2_small.json"
        );
        let mut panel = BtPanel::new("welding");
        panel
            .load_text("welding.json", &std::fs::read_to_string(path).unwrap())
            .unwrap();
        panel
    }

    /// On a 1440-point screen the fitted welding plan is far too small to read; following,
    /// a Step zooms in on the welds that run.
    #[test]
    fn following_frames_the_running_leaves() {
        let mut panel = welding();
        crate::testing::draw(|ui| panel.ui(ui));
        let fitted = panel.scene_rect;
        assert!(fitted.width() > 3000.0, "{fitted:?}");
        panel.step();
        crate::testing::draw(|ui| panel.ui(ui));
        let followed = panel.scene_rect;
        assert!(followed.width() <= 1440.0, "{followed:?}");
        // The first Parallel's welds are what it shows.
        let ex = panel.executor().unwrap();
        let running: Vec<&str> = ex
            .tree()
            .nodes
            .iter()
            .filter(|n| n.kind.is_leaf() && ex.status(n.uid) == NodeStatus::Running)
            .map(|n| n.name.as_str())
            .collect();
        assert_eq!(running, ["weld_robot1_W01", "weld_robot2_W13"]);
        assert_eq!(panel.followed.len(), 2);
        // Nothing new runs: the view is the user's (at its zoom and top-left corner, on a
        // canvas that may have changed size).
        panel.scene_rect = fitted;
        let mine = shown(fitted, panel.view_outer);
        crate::testing::draw(|ui| panel.ui(ui));
        let now = shown(panel.scene_rect, panel.view_outer);
        assert!(
            (now.0 - mine.0).abs() < 1e-5 && (now.1 - mine.1).length() < 1e-2,
            "{mine:?} -> {now:?}"
        );

        // Not following, a Step leaves the view alone; Compact cards fit in less room.
        let mut panel = welding();
        panel.set_follow(false);
        crate::testing::draw(|ui| panel.ui(ui));
        let fitted = panel.scene_rect;
        panel.step();
        crate::testing::draw(|ui| panel.ui(ui));
        assert_eq!(panel.scene_rect, fitted);
        panel.set_compact(true);
        crate::testing::draw(|ui| panel.ui(ui));
        assert!(
            panel.scene_rect.height() < fitted.height(),
            "{:?}",
            panel.scene_rect
        );
    }

    #[test]
    fn a_resized_canvas_keeps_the_zoom_and_the_top_left() {
        let before = vec2(400.0, 600.0);
        // Zoom 0.5, letterboxed: the canvas shows 800 x 1200 of the scene around the centre.
        let shown = Rect::from_center_size(pos2(500.0, 500.0), vec2(800.0, 800.0));
        let now = vec2(400.0, 578.0);
        let kept = keep_zoom(shown, before, now);
        assert!((zoom(kept, now) - 0.5).abs() < 1e-5, "{kept:?}");
        assert_eq!(kept.min, pos2(100.0, -100.0));
        assert_eq!(kept.size(), now / 0.5);
        // Back to the first size: the same view (now unletterboxed).
        let back = keep_zoom(kept, now, before);
        assert!((zoom(back, before) - 0.5).abs() < 1e-5);
        assert_eq!(back.min, kept.min);
        // Nothing to keep: unchanged.
        assert_eq!(keep_zoom(shown, before, before), shown);
        assert_eq!(keep_zoom(shown, Vec2::ZERO, now), shown);
        assert_eq!(keep_zoom(Rect::ZERO, before, now), Rect::ZERO);
    }

    /// Where egui's Scene draws the scene point `p`, relative to the canvas's top-left corner:
    /// it fits `scene_rect` into the canvas, letterboxed and centred (egui's `Scene::show`).
    fn on_canvas(panel: &BtPanel, p: egui::Pos2) -> (f32, Vec2) {
        let (scene, outer) = (panel.scene_rect, panel.view_outer);
        let zoom = zoom(scene, outer);
        (zoom, outer / 2.0 + zoom * (p - scene.center()))
    }

    /// One frame of the tree as the viewer shows it (`tree_ui`) in a tall pane `width` points
    /// wide, in a context kept across frames like an app's, with `below` points taken off the
    /// bottom.
    fn tree_frame(ctx: &egui::Context, panel: &mut BtPanel, width: f32, below: f32) {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(pos2(0.0, 0.0), vec2(width, 980.0))),
            ..Default::default()
        };
        let mut output = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                if below > 0.0 {
                    egui::Panel::bottom("below")
                        .exact_size(below)
                        .show(ui, |_| {});
                }
                panel.tree_ui(ui);
            });
        });
        output.textures_delta.clear();
    }

    /// Following the welds on a canvas that gets shorter and taller again (a footer row that
    /// comes and goes as the live cell drifts from the interpreter, a resized inspector): the
    /// tree stays where it is on screen, at its zoom. Before, egui's Scene fitted the same rect
    /// into each size, so the view zoomed and moved back and forth with every change.
    #[test]
    fn a_followed_view_stays_put_while_the_canvas_resizes() {
        let ctx = egui::Context::default();
        crate::fonts::install(&ctx);
        let mut panel = welding();
        for _ in 0..3 {
            tree_frame(&ctx, &mut panel, 420.0, 0.0);
        }
        panel.step();
        for _ in 0..3 {
            tree_frame(&ctx, &mut panel, 420.0, 0.0);
        }
        assert_eq!(panel.followed.len(), 2, "following the two welds");
        let anchor = panel.scene_rect.min;
        let (zoom0, at0) = on_canvas(&panel, anchor);
        let tall = panel.view_outer;
        for i in 0..8 {
            let below = if i % 2 == 0 { 22.0 } else { 0.0 };
            tree_frame(&ctx, &mut panel, 420.0, below);
            tree_frame(&ctx, &mut panel, 420.0, below);
            // The canvas did get shorter (and taller again).
            assert_eq!(
                panel.view_outer.y < tall.y,
                below > 0.0,
                "frame {i}: {:?}",
                panel.view_outer
            );
            let (zoom, at) = on_canvas(&panel, anchor);
            assert!(
                (zoom - zoom0).abs() < 1e-4,
                "frame {i}: zoom {zoom0} -> {zoom}"
            );
            assert!((at - at0).length() < 0.5, "frame {i}: {at0:?} -> {at:?}");
        }
    }

    /// The live cell drifting from the interpreter and agreeing again (its finished motions are
    /// reported a tick before the interpreter's) leaves the canvas, and so the tree, where they
    /// were: the drift row is one line of one height however many leaves drift.
    #[test]
    fn drift_coming_and_going_does_not_move_the_tree() {
        // The welding plan with the leaf names of the recorded cell, two of which drifting do
        // not fit one line of a 520-point pane.
        let (w01, w13) = (
            "weld_robot1_W01_L_B_pillar_foot_to_sill",
            "weld_robot2_W13_R_B_pillar_foot_to_sill",
        );
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../test_data/bt_1790934549_welding_cell_2_small.json"
        );
        let text = std::fs::read_to_string(path)
            .unwrap()
            .replace("weld_robot1_W01", w01)
            .replace("weld_robot2_W13", w13);
        let mut panel = BtPanel::new("welding");
        panel.load_text("welding.json", &text).unwrap();
        let ctx = egui::Context::default();
        crate::fonts::install(&ctx);
        tree_frame(&ctx, &mut panel, 520.0, 0.0);
        panel.step();
        let running: Vec<(Uid, String)> = [(w01, "robot1"), (w13, "robot2")]
            .into_iter()
            .map(|(name, robot)| {
                let tree = panel.executor().unwrap().tree();
                let uid = tree.nodes.iter().find(|n| n.name == name).unwrap().uid;
                (uid, robot.to_string())
            })
            .collect();
        let agree = super::super::Action::SetExternalActive(running.clone());
        let drift_one = super::super::Action::SetExternalActive(running[1..].to_vec());
        let drift_both = super::super::Action::SetExternalActive(Vec::new());
        panel.apply([agree.clone()]);
        for _ in 0..3 {
            tree_frame(&ctx, &mut panel, 520.0, 0.0);
        }
        let (outer, scene) = (panel.view_outer, panel.scene_rect);
        for action in [drift_one, drift_both, agree].into_iter().cycle().take(9) {
            panel.apply([action]);
            tree_frame(&ctx, &mut panel, 520.0, 0.0);
            tree_frame(&ctx, &mut panel, 520.0, 0.0);
            let drift = panel.drift();
            assert_eq!(panel.view_outer, outer, "{drift:?}");
            assert_eq!(panel.scene_rect, scene, "{drift:?}");
        }
    }

    /// Where the leaves followed are in the scene, laid out as the canvas lays them out.
    fn followed_rect(ctx: &egui::Context, panel: &BtPanel) -> Rect {
        let mut union = None;
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            let tree = panel.executor().unwrap().tree();
            let (_, rects) = lay_out(
                ui.painter(),
                tree,
                &panel.collapsed,
                panel.compact,
                panel.orientation,
            );
            union = panel
                .followed
                .iter()
                .filter_map(|&uid| rects[uid])
                .reduce(|a, b| a.union(b));
        });
        output.textures_delta.clear();
        union.expect("leaves followed")
    }

    /// What the canvas shows of the scene: the scene rect, letterboxed.
    fn visible(panel: &BtPanel) -> Rect {
        let (scene, outer) = (panel.scene_rect, panel.view_outer);
        Rect::from_center_size(scene.center(), outer / zoom(scene, outer))
    }

    /// Following the two welds that run side by side in a 520-point pane, the user narrows the
    /// pane: both stay in view (the canvas keeps its zoom and top-left corner as it resizes, so
    /// without more the right-hand weld went off the canvas until another leaf ran), and the
    /// view only ever zooms out to keep them; widened again, nothing moves.
    #[test]
    fn narrowing_the_pane_keeps_the_followed_leaves_in_view() {
        let ctx = egui::Context::default();
        crate::fonts::install(&ctx);
        let mut panel = welding();
        tree_frame(&ctx, &mut panel, 520.0, 0.0);
        panel.step();
        for _ in 0..3 {
            tree_frame(&ctx, &mut panel, 520.0, 0.0);
        }
        assert_eq!(panel.followed.len(), 2, "following the two welds");
        let welds = followed_rect(&ctx, &panel);
        assert!(
            visible(&panel).contains_rect(welds),
            "{:?}",
            visible(&panel)
        );
        let mut last = zoom(panel.scene_rect, panel.view_outer);
        for width in [420.0, 320.0, 250.0] {
            for _ in 0..2 {
                tree_frame(&ctx, &mut panel, width, 0.0);
            }
            let shows = visible(&panel);
            assert!(
                shows.expand(0.5).contains_rect(welds),
                "{width} pt: {shows:?} does not show {welds:?}"
            );
            let now = zoom(panel.scene_rect, panel.view_outer);
            assert!(now <= last + 1e-5, "{width} pt: zoomed in, {last} -> {now}");
            last = now;
        }
        // Wider again: the welds are in view, so the view stays at its zoom and top-left.
        let narrow = shown(panel.scene_rect, panel.view_outer);
        for _ in 0..2 {
            tree_frame(&ctx, &mut panel, 520.0, 0.0);
        }
        let wide = shown(panel.scene_rect, panel.view_outer);
        assert!(
            (wide.0 - narrow.0).abs() < 1e-5 && (wide.1 - narrow.1).length() < 1e-2,
            "{narrow:?} -> {wide:?}"
        );
    }

    #[test]
    fn keeping_in_view_pans_and_zooms_out_only_as_far_as_it_must() {
        let outer = vec2(400.0, 300.0);
        // Zoom 1, showing 0..400 x 0..300.
        let view = Rect::from_min_size(pos2(0.0, 0.0), outer);
        let inside = Rect::from_min_size(pos2(50.0, 50.0), vec2(300.0, 200.0));
        assert_eq!(keep_in_view(view, outer, inside), view);
        // Off the right by 60: panned right just far enough to show it with the margin, at the
        // same zoom, and not at all up or down.
        let right = inside.translate(vec2(60.0, 0.0));
        let kept = keep_in_view(view, outer, right);
        assert!((zoom(kept, outer) - 1.0).abs() < 1e-5);
        assert_eq!(kept.min, pos2(50.0, 0.0));
        // Wider than the canvas: zoomed out to fit it with the full margin, no further.
        let wide = Rect::from_min_size(pos2(0.0, 50.0), vec2(520.0, 100.0));
        let kept = keep_in_view(view, outer, wide);
        assert!((zoom(kept, outer) - 400.0 / 600.0).abs() < 1e-5, "{kept:?}");
        assert!(
            kept.contains_rect(wide.expand(REVEAL_MARGIN - 1e-3)),
            "{kept:?}"
        );
        // Far too small to read but in view: unlike revealing, it does not zoom in.
        let tiny = Rect::from_min_size(pos2(0.0, 0.0), outer * 10.0);
        assert_eq!(keep_in_view(tiny, outer, inside), tiny);
        assert_eq!(keep_in_view(Rect::ZERO, outer, inside), Rect::ZERO);
    }

    /// A finished run starts over, so Run to t may go back to any instant.
    #[test]
    fn run_to_t_may_go_back_once_the_run_is_done() {
        let mut panel = welding();
        panel.run_to_t = 5.0;
        crate::testing::draw(|ui| panel.ui(ui));
        assert_eq!(panel.run_to_t, 5.0);
        panel.run_to_time(10.0);
        crate::testing::draw(|ui| panel.ui(ui));
        assert_eq!(panel.run_to_t, 10.0, "not before now while it runs");
        panel.run_to_wait();
        panel.run_to_t = 5.0;
        crate::testing::draw(|ui| panel.ui(ui));
        assert_eq!(panel.run_to_t, 5.0);
        panel.apply([super::super::Action::RunToTime(5.0)]);
        assert!((panel.executor().unwrap().now() - 5.0).abs() < 1e-9);
    }
}
