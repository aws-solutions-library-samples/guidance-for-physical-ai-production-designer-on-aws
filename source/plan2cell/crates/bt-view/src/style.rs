//! Colours and sizes: a Groot2-like card per node, Groot's monitor colours for statuses.

use bt_core::{NodeKind, NodeStatus};
use egui::Color32;

pub const CANVAS: Color32 = Color32::from_rgb(22, 24, 27);
pub const GRID_DOT: Color32 = Color32::from_rgb(40, 43, 48);
pub const CARD: Color32 = Color32::from_rgb(48, 51, 56);
pub const CARD_BORDER: Color32 = Color32::from_rgb(82, 86, 94);
pub const SELECTED: Color32 = Color32::from_rgb(255, 165, 0);
pub const TEXT: Color32 = Color32::from_rgb(230, 232, 235);
pub const TEXT_DIM: Color32 = Color32::from_rgb(150, 155, 162);
pub const CHIP: Color32 = Color32::from_rgb(200, 200, 200);
pub const CHIP_TEXT: Color32 = Color32::from_rgb(30, 30, 30);
/// `{key}` port values.
pub const CHIP_BLACKBOARD: Color32 = Color32::from_rgb(255, 225, 140);
pub const CHIP_EMPTY: Color32 = Color32::from_rgb(70, 74, 80);
pub const EDGE: Color32 = Color32::from_rgb(110, 115, 125);
pub const WARN: Color32 = Color32::from_rgb(255, 200, 60);
/// Load errors and rejected edits.
pub const ERROR: Color32 = Color32::from_rgb(250, 80, 80);
/// Breakpoints, and the ring around a node held at one.
pub const BREAK: Color32 = Color32::from_rgb(230, 60, 60);
/// A RUNNING leaf that nothing scheduled will finish (a stalled gate).
pub const STALLED: Color32 = Color32::from_rgb(255, 130, 40);
/// Edited port and blackboard values.
pub const EDITED: Color32 = Color32::from_rgb(120, 190, 255);
/// The ring around a leaf an external source (the live cell) reports running, apart from the
/// interpreter's own status colours.
pub const LIVE: Color32 = Color32::from_rgb(0, 200, 235);
/// The halt banner's background.
pub const HALT_FILL: Color32 = Color32::from_rgb(64, 52, 24);

/// Motion on the cell panel's Gantt chart, by what it does ([`op_color`]).
pub const OP_HOME: Color32 = Color32::from_rgb(38, 150, 150);
pub const OP_INTERLOCK: Color32 = Color32::from_rgb(150, 100, 200);
pub const OP_WELD: Color32 = Color32::from_rgb(214, 112, 48);
pub const OP_MOVE: Color32 = Color32::from_rgb(70, 130, 180);
/// Other process operations, picked by their `op_type`.
const OP_OTHER: [Color32; 4] = [
    Color32::from_rgb(86, 160, 86),
    Color32::from_rgb(196, 168, 60),
    Color32::from_rgb(190, 90, 150),
    Color32::from_rgb(120, 120, 205),
];
/// A robot holding its pose after a motion.
pub const HOLD: Color32 = Color32::from_rgb(200, 190, 150);

pub const CORNER: u8 = 6;
pub const PAD: f32 = 8.0;
pub const HEADER_H: f32 = 24.0;
pub const ROW_H: f32 = 18.0;
pub const ROW_GAP: f32 = 3.0;
pub const MIN_WIDTH: f32 = 120.0;
/// Groot's value fields are at most 140 px wide.
pub const MAX_CHIP_WIDTH: f32 = 140.0;
pub const PORT_RADIUS: f32 = 4.5;
/// Space kept free on the header's right for the badge dots: status, override, waiting (manual
/// or stalled), edited, unknown model.
pub const BADGE_DIAMETER: f32 = 14.0;
pub const BADGE_SLOTS: usize = 5;

pub const CAPTION_SIZE: f32 = 13.5;
pub const NAME_SIZE: f32 = 12.5;
pub const PORT_SIZE: f32 = 11.5;

/// Header colour per category.
pub fn header(kind: NodeKind) -> Color32 {
    match kind {
        NodeKind::Root => Color32::from_rgb(90, 94, 102),
        NodeKind::Action => Color32::from_rgb(58, 104, 160),
        NodeKind::Condition => Color32::from_rgb(56, 138, 82),
        NodeKind::Control => Color32::from_rgb(122, 86, 168),
        NodeKind::Decorator => Color32::from_rgb(176, 128, 44),
        NodeKind::SubTree => Color32::from_rgb(42, 140, 140),
    }
}

/// Fall back to `plain` when the bundled fonts have no glyph for `preferred`.
///
/// The fonts ([`crate::fonts`]: JetBrains Mono and two emoji fonts, or the host's) cover only part
/// of the arrows and symbols Groot uses; a missing glyph would otherwise be drawn as a box.
pub fn glyph(ctx: &egui::Context, preferred: &'static str, plain: &'static str) -> &'static str {
    let font = egui::FontId::proportional(CAPTION_SIZE);
    if ctx.fonts_mut(|f| f.has_glyphs(&font, preferred)) {
        preferred
    } else {
        plain
    }
}

/// `(preferred glyph, fallback)` for the header, by node ID first and category otherwise
/// (Groot's icons).
pub fn icon(id: &str, kind: NodeKind) -> (&'static str, &'static str) {
    match id {
        "Sequence" => ("→", ">"),
        "SequenceWithMemory" | "SequenceStar" => ("⇒", ">>"),
        "ReactiveSequence" => ("↻→", "↻>"),
        "Fallback" => ("?", "?"),
        "ReactiveFallback" => ("↻?", "↻?"),
        "Parallel" | "ParallelAll" => ("⇉", "||"),
        "IfThenElse" | "WhileDoElse" => ("⑂", "if"),
        "Inverter" => ("≠", "!"),
        "Repeat" => ("⟳", "↺"),
        "RetryUntilSuccessful" | "Retry" => ("↺", "↺"),
        "Timeout" | "Delay" | "Sleep" => ("⏱", "t"),
        "ForceSuccess" | "AlwaysSuccess" => ("✔", "✔"),
        "ForceFailure" | "AlwaysFailure" => ("✖", "✖"),
        "SetBlackboard" | "UnsetBlackboard" | "Script" | "ScriptCondition" => ("✎", "="),
        "RunOnce" => ("①", "1"),
        _ if id.starts_with("Switch") => ("⑂", "sw"),
        _ => match kind {
            NodeKind::Root => ("◉", "•"),
            NodeKind::Action => ("A", "A"),
            NodeKind::Condition => ("C", "C"),
            NodeKind::Control => ("◆", "◆"),
            NodeKind::Decorator => ("◇", "◇"),
            NodeKind::SubTree => ("⊞", "+"),
        },
    }
}

/// Groot's monitor colours (`getStyleFromStatus`).
pub fn status(status: NodeStatus) -> Option<Color32> {
    match status {
        NodeStatus::Idle => None,
        NodeStatus::Running => Some(Color32::from_rgb(220, 140, 20)),
        NodeStatus::Success => Some(Color32::from_rgb(51, 200, 51)),
        NodeStatus::Failure => Some(Color32::from_rgb(250, 50, 50)),
        NodeStatus::Skipped => Some(Color32::from_rgb(120, 130, 160)),
    }
}

/// Groot's muted colours for a node that went back to IDLE.
pub fn was(status: NodeStatus) -> Option<Color32> {
    match status {
        NodeStatus::Idle => None,
        NodeStatus::Running => Some(Color32::from_rgb(150, 130, 40)),
        NodeStatus::Success => Some(Color32::from_rgb(100, 150, 100)),
        NodeStatus::Failure => Some(Color32::from_rgb(150, 80, 80)),
        NodeStatus::Skipped => Some(Color32::from_rgb(95, 100, 120)),
    }
}

/// A motion's colour on the cell panel: a move home, an interlock pre-move, a weld, any other
/// move, and a colour of its own for every other kind of process operation.
pub fn op_color(op_type: &str, op_id: &str) -> Color32 {
    if op_id == "home" {
        return OP_HOME;
    }
    if op_id.ends_with(":interlock") {
        return OP_INTERLOCK;
    }
    match op_type {
        "weld" => OP_WELD,
        "move" => OP_MOVE,
        other => {
            // FNV-1a: the same kind gets the same colour on every run.
            let hash = other.bytes().fold(0x811c_9dc5_u32, |h, b| {
                (h ^ u32::from(b)).wrapping_mul(0x0100_0193)
            });
            OP_OTHER[hash as usize % OP_OTHER.len()]
        }
    }
}

/// A robot's phase (cell-runner's playback phases: the motion's `op_type`, `hold`, `idle`)
/// during operation `op_id`: a motion in its bar's colour ([`op_color`]).
pub fn phase(phase: &str, op_id: &str) -> Color32 {
    match phase {
        "idle" => TEXT_DIM,
        "hold" => HOLD,
        op_type => op_color(op_type, op_id),
    }
}
