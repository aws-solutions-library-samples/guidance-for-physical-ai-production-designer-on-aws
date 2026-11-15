//! The tree drawing: one card per node, laid out with Groot's algorithm, inside a pan/zoom
//! [`egui::Scene`].

use std::sync::Arc;

use bt_core::layout::{self, Orientation};
use bt_core::status::NodeView;
use bt_core::{Node, NodeKind, Tree, Uid};
use egui::epaint::CubicBezierShape;
use egui::{
    Color32, CornerRadius, FontId, Galley, Painter, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2,
};

use crate::style;

/// Text of one node card, laid out.
pub struct Card {
    icon: Arc<Galley>,
    caption: Arc<Galley>,
    name: Option<Arc<Galley>>,
    /// `(label, value, value is a {key} pointer)`; `value` is `None` for an unset port.
    ports: Vec<(Arc<Galley>, Option<Arc<Galley>>, bool)>,
    conditions: Vec<Arc<Galley>>,
    /// "+ N nodes" on a collapsed SubTree.
    collapsed: Option<Arc<Galley>>,
    label_width: f32,
    pub size: Vec2,
}

/// What a node is decorated with besides its status.
#[derive(Clone, Copy, Debug, Default)]
pub struct Badges {
    pub selected: bool,
    pub forced: Option<bt_core::NodeStatus>,
    /// A stub action whose policy waits for an override.
    pub manual: bool,
    /// A manual leaf waiting for the user now.
    pub waiting: bool,
    /// A RUNNING leaf that nothing scheduled will finish (a stalled gate).
    pub stalled: bool,
    /// Has a breakpoint.
    pub breakpoint: bool,
    /// Held at its breakpoint now.
    pub held: bool,
    /// A port value was edited.
    pub edited: bool,
    /// How far its motion is, 0..=1 (cell leaves).
    pub progress: Option<f32>,
    /// An external source (the live cell) reports it running ([`Action::SetExternalActive`]).
    ///
    /// [`Action::SetExternalActive`]: crate::Action::SetExternalActive
    pub live: bool,
}

fn truncate(text: &str, max_chars: usize) -> String {
    if text.chars().count() <= max_chars {
        text.to_string()
    } else {
        let mut s: String = text.chars().take(max_chars.saturating_sub(1)).collect();
        s.push('…');
        s
    }
}

impl Card {
    /// `compact`: without the port and condition rows.
    pub fn new(
        painter: &Painter,
        tree: &Tree,
        node: &Node,
        collapsed: bool,
        compact: bool,
    ) -> Self {
        let layout = |text: String, size: f32, color: Color32| {
            painter.layout_no_wrap(text, FontId::proportional(size), color)
        };
        let mono = |text: String, size: f32, color: Color32| {
            painter.layout_no_wrap(text, FontId::monospace(size), color)
        };

        let (preferred, plain) = style::icon(&node.id, node.kind);
        let icon = layout(
            style::glyph(painter.ctx(), preferred, plain).to_string(),
            style::CAPTION_SIZE,
            style::TEXT,
        );
        let caption_text = if node.kind == NodeKind::Root {
            "ROOT".to_string()
        } else {
            node.id.clone()
        };
        let caption = layout(caption_text, style::CAPTION_SIZE, style::TEXT);
        let name = (node.shows_name() && node.kind != NodeKind::Root)
            .then(|| layout(node.name.clone(), style::NAME_SIZE, style::TEXT));

        let shown = if compact { &[][..] } else { &node.ports[..] };
        let ports: Vec<_> = shown
            .iter()
            .map(|p| {
                let prefix = match p.direction {
                    bt_core::PortDirection::Input => "[IN] ",
                    bt_core::PortDirection::Output => "[OUT] ",
                    bt_core::PortDirection::InOut => "",
                };
                let label = mono(
                    format!("{prefix}{}", p.name),
                    style::PORT_SIZE,
                    style::TEXT_DIM,
                );
                let value = p.effective().map(|v| {
                    let color = if p.value.is_none() {
                        Color32::from_rgb(90, 90, 90)
                    } else {
                        style::CHIP_TEXT
                    };
                    mono(truncate(v, 22), style::PORT_SIZE, color)
                });
                (label, value, p.blackboard_key().is_some())
            })
            .collect();
        let conditions = node
            .conditions
            .iter()
            .filter(|_| !compact)
            .map(|(k, v)| {
                mono(
                    format!("{k}: {}", truncate(v, 28)),
                    style::PORT_SIZE,
                    style::WARN,
                )
            })
            .collect::<Vec<_>>();
        let collapsed = collapsed.then(|| {
            let hidden = tree
                .nodes
                .iter()
                .filter(|n| n.uid != node.uid && tree.is_ancestor(node.uid, n.uid))
                .count();
            let arrow = style::glyph(painter.ctx(), "▸", ">");
            layout(
                format!("{arrow} {hidden} nodes (double-click)"),
                style::PORT_SIZE,
                style::TEXT_DIM,
            )
        });

        let label_width = ports.iter().map(|(l, ..)| l.size().x).fold(0.0, f32::max);
        let chip_width = ports
            .iter()
            .map(|(_, v, _)| {
                v.as_ref()
                    .map_or(30.0, |v| v.size().x + 10.0)
                    .clamp(50.0, style::MAX_CHIP_WIDTH)
            })
            .fold(0.0, f32::max);
        let badges = style::BADGE_SLOTS as f32 * (style::BADGE_DIAMETER + 2.0);
        let mut width = (icon.size().x + 6.0 + caption.size().x + badges)
            .max(style::MIN_WIDTH - 2.0 * style::PAD);
        if let Some(name) = &name {
            width = width.max(name.size().x);
        }
        if !ports.is_empty() {
            width = width.max(label_width + 8.0 + chip_width);
        }
        for c in conditions.iter().chain(collapsed.iter()) {
            width = width.max(c.size().x);
        }
        width += 2.0 * style::PAD;

        let rows = ports.len()
            + conditions.len()
            + usize::from(name.is_some())
            + usize::from(collapsed.is_some());
        let body = if rows == 0 {
            6.0
        } else {
            style::PAD + rows as f32 * (style::ROW_H + style::ROW_GAP) - style::ROW_GAP + style::PAD
        };
        Self {
            icon,
            caption,
            name,
            ports,
            conditions,
            collapsed,
            label_width,
            size: Vec2::new(width.ceil(), (style::HEADER_H + body).ceil()),
        }
    }

    pub fn paint(
        &self,
        painter: &Painter,
        rect: Rect,
        node: &Node,
        view: NodeView,
        badges: Badges,
    ) {
        let corner = CornerRadius::same(style::CORNER);
        let header_rect = Rect::from_min_size(rect.min, Vec2::new(rect.width(), style::HEADER_H));

        // Shadow, card, header.
        painter.rect_filled(
            rect.translate(Vec2::splat(3.0)),
            corner,
            Color32::from_black_alpha(90),
        );
        painter.rect_filled(rect, corner, style::CARD);
        let mut header = style::header(node.kind);
        if view.status == bt_core::NodeStatus::Idle && view.was.is_none() {
            header = header.gamma_multiply(0.85);
        }
        painter.rect_filled(
            header_rect,
            CornerRadius {
                nw: style::CORNER,
                ne: style::CORNER,
                sw: 0,
                se: 0,
            },
            header,
        );
        let y_mid = header_rect.center().y;
        let mut x = rect.min.x + style::PAD;
        painter.galley(
            Pos2::new(x, y_mid - self.icon.size().y / 2.0),
            self.icon.clone(),
            style::TEXT,
        );
        x += self.icon.size().x + 6.0;
        painter.galley(
            Pos2::new(x, y_mid - self.caption.size().y / 2.0),
            self.caption.clone(),
            style::TEXT,
        );

        // Body rows.
        let mut y = header_rect.max.y + style::PAD;
        let left = rect.min.x + style::PAD;
        let right = rect.max.x - style::PAD;
        if let Some(name) = &self.name {
            let pos = Pos2::new(
                rect.center().x - name.size().x / 2.0,
                y + (style::ROW_H - name.size().y) / 2.0,
            );
            painter.galley(pos, name.clone(), style::TEXT);
            y += style::ROW_H + style::ROW_GAP;
        }
        for (label, value, is_bb) in &self.ports {
            painter.galley(
                Pos2::new(left, y + (style::ROW_H - label.size().y) / 2.0),
                label.clone(),
                style::TEXT_DIM,
            );
            let chip_left = left + self.label_width + 8.0;
            let chip =
                Rect::from_min_max(Pos2::new(chip_left, y), Pos2::new(right, y + style::ROW_H));
            match value {
                Some(value) => {
                    let fill = if *is_bb {
                        style::CHIP_BLACKBOARD
                    } else {
                        style::CHIP
                    };
                    painter.rect_filled(chip, CornerRadius::same(3), fill);
                    let text_pos = Pos2::new(
                        chip.center().x - value.size().x / 2.0,
                        chip.center().y - value.size().y / 2.0,
                    );
                    painter
                        .with_clip_rect(chip.intersect(painter.clip_rect()))
                        .galley(text_pos, value.clone(), style::CHIP_TEXT);
                }
                None => {
                    painter.rect_filled(chip, CornerRadius::same(3), style::CHIP_EMPTY);
                }
            }
            y += style::ROW_H + style::ROW_GAP;
        }
        for c in self.conditions.iter().chain(self.collapsed.iter()) {
            painter.galley(
                Pos2::new(left, y + (style::ROW_H - c.size().y) / 2.0),
                c.clone(),
                style::WARN,
            );
            y += style::ROW_H + style::ROW_GAP;
        }

        // Border: status (Groot monitor) or selection.
        let (border, width) = match (style::status(view.status), view.was.and_then(style::was)) {
            (Some(color), _) => (color, 3.0),
            (None, Some(muted)) => (muted, 3.0),
            (None, None) => (style::CARD_BORDER, 1.0),
        };
        painter.rect_stroke(
            rect,
            corner,
            Stroke::new(width, border),
            StrokeKind::Outside,
        );
        if badges.selected {
            painter.rect_stroke(
                rect.expand(width + 2.0),
                CornerRadius::same(style::CORNER + 3),
                Stroke::new(1.5, style::SELECTED),
                StrokeKind::Outside,
            );
        }
        // Where the run halts: a ring outside the selection's.
        if badges.held || badges.stalled {
            let color = if badges.held {
                style::BREAK
            } else {
                style::STALLED
            };
            painter.rect_stroke(
                rect.expand(width + 6.0),
                CornerRadius::same(style::CORNER + 6),
                Stroke::new(2.0, color),
                StrokeKind::Outside,
            );
        }
        // What the live cell runs: a ring of its own, outermost, whatever the interpreter says.
        if badges.live {
            painter.rect_stroke(
                rect.expand(width + 10.0),
                CornerRadius::same(style::CORNER + 10),
                Stroke::new(2.5, style::LIVE),
                StrokeKind::Outside,
            );
        }
        if badges.breakpoint {
            painter.circle(
                rect.left_top() + Vec2::splat(2.0),
                5.0,
                style::BREAK,
                Stroke::new(1.5, style::CANVAS),
            );
        }
        if let Some(progress) = badges.progress {
            let inset = f32::from(style::CORNER);
            let track = Rect::from_min_max(
                Pos2::new(rect.min.x + inset, rect.max.y - 5.0),
                Pos2::new(rect.max.x - inset, rect.max.y - 2.0),
            );
            painter.rect_filled(track, CornerRadius::same(1), style::CHIP_EMPTY);
            let mut done = track;
            done.max.x = track.min.x + track.width() * progress.clamp(0.0, 1.0);
            let color = style::status(bt_core::NodeStatus::Running).unwrap_or(style::TEXT);
            painter.rect_filled(done, CornerRadius::same(1), color);
        }

        // Status and override dots on the header's right (the card reserves room for them).
        let mut bx = header_rect.max.x - style::PAD - style::BADGE_DIAMETER / 2.0;
        let mut dot = |fill: Color32, glyph: &str, fg: Color32| {
            let centre = Pos2::new(bx, y_mid);
            painter.circle_filled(centre, style::BADGE_DIAMETER / 2.0, fill);
            if !glyph.is_empty() {
                let galley =
                    painter.layout_no_wrap(glyph.to_string(), FontId::proportional(9.5), fg);
                painter.galley(centre - galley.size() / 2.0, galley, fg);
            }
            bx -= style::BADGE_DIAMETER + 2.0;
        };
        match (style::status(view.status), view.was.and_then(style::was)) {
            (Some(color), _) => dot(color, "", Color32::BLACK),
            (None, Some(muted)) => dot(muted, "", Color32::BLACK),
            (None, None) => {}
        }
        if badges.forced.is_some() {
            dot(Color32::from_rgb(235, 235, 235), "!", Color32::BLACK);
        }
        // Waiting for the user, stalled, or a manual leaf: one slot.
        if badges.waiting {
            dot(style::WARN, "?", Color32::BLACK);
        } else if badges.stalled {
            let square = style::glyph(painter.ctx(), "■", "#");
            dot(style::STALLED, square, Color32::BLACK);
        } else if badges.manual {
            dot(Color32::from_rgb(70, 74, 80), "?", style::TEXT);
        }
        if badges.edited {
            let pen = style::glyph(painter.ctx(), "✎", "*");
            dot(style::EDITED, pen, Color32::BLACK);
        }
        if node.unknown_model {
            dot(style::WARN, "!", Color32::BLACK);
        }
    }
}

/// Where edges attach: the parent's output and the child's input.
pub fn ports(rect: Rect, orientation: Orientation) -> (Pos2, Pos2) {
    match orientation {
        Orientation::Vertical => (rect.center_bottom(), rect.center_top()),
        Orientation::Horizontal => {
            let y = rect.min.y + style::HEADER_H / 2.0;
            (Pos2::new(rect.max.x, y), Pos2::new(rect.min.x, y))
        }
    }
}

pub fn paint_edge(
    painter: &Painter,
    from: Pos2,
    to: Pos2,
    orientation: Orientation,
    color: Color32,
    width: f32,
) {
    let (c1, c2) = match orientation {
        Orientation::Vertical => {
            let d = ((to.y - from.y) * 0.5).max(20.0);
            (from + Vec2::new(0.0, d), to - Vec2::new(0.0, d))
        }
        Orientation::Horizontal => {
            let d = ((to.x - from.x) * 0.5).max(20.0);
            (from + Vec2::new(d, 0.0), to - Vec2::new(d, 0.0))
        }
    };
    painter.add(CubicBezierShape::from_points_stroke(
        [from, c1, c2, to],
        false,
        Color32::TRANSPARENT,
        Stroke::new(width, color),
    ));
}

pub fn paint_port(painter: &Painter, at: Pos2) {
    painter.circle_filled(at, style::PORT_RADIUS, Color32::from_rgb(150, 150, 169));
}

pub fn paint_grid(painter: &Painter, rect: Rect) {
    let step = 40.0;
    let x0 = (rect.min.x / step).floor() as i64;
    let y0 = (rect.min.y / step).floor() as i64;
    let nx = (rect.width() / step).ceil() as i64 + 1;
    let ny = (rect.height() / step).ceil() as i64 + 1;
    if nx * ny > 40_000 {
        return; // zoomed far out
    }
    for i in 0..nx {
        for j in 0..ny {
            painter.circle_filled(
                Pos2::new((x0 + i) as f32 * step, (y0 + j) as f32 * step),
                1.2,
                style::GRID_DOT,
            );
        }
    }
}

/// Node rectangles in scene coordinates, from Groot's layout of the measured cards.
pub fn place(
    tree: &Tree,
    cards: &[Card],
    orientation: Orientation,
    expanded: impl Fn(Uid) -> bool,
) -> Vec<Option<Rect>> {
    let sizes: Vec<layout::Vec2> = cards
        .iter()
        .map(|c| layout::Vec2::new(c.size.x, c.size.y))
        .collect();
    let placed = layout::layout(tree, &sizes, orientation, expanded);
    placed
        .positions
        .iter()
        .zip(cards)
        .map(|(p, c)| p.map(|p| Rect::from_min_size(Pos2::new(p.x, p.y), c.size)))
        .collect()
}

/// Interaction for one node card; `name` is what assistive technology calls it.
pub fn interact(ui: &egui::Ui, rect: Rect, uid: Uid, name: &str) -> egui::Response {
    let response = ui.interact(rect, ui.id().with(("bt_node", uid)), Sense::click());
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, name));
    response
}
