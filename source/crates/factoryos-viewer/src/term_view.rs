//! A terminal screen in egui: the shell's ANSI output parsed by `vt100`, drawn as a monospace grid
//! (colours, bold, underline, inverse, the cursor), with the keyboard mapped to the bytes a shell
//! expects, scrollback on the wheel, and a mouse selection to copy.

use std::num::NonZeroU16;

use crate::rr::egui::{self, Color32, FontId, Key, Modifiers, Pos2, Rect, Stroke, Vec2};
use crate::theme::{self, Palette};

const SCROLLBACK: usize = 5000;
const PAD: Vec2 = Vec2::new(10.0, 6.0);

fn nz(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n.max(1)).unwrap_or(NonZeroU16::MIN)
}

/// What the view wants sent to the shell this frame.
#[derive(Default, Debug, PartialEq, Eq)]
pub struct TermOutput {
    pub input: Vec<u8>,
    /// The grid's new size (cols, rows).
    pub resize: Option<(u16, u16)>,
}

pub struct TermView {
    parser: vt100::Parser,
    /// (cols, rows) the grid has.
    size: (u16, u16),
    /// Selected cells, from the anchor to the end (row, col), in screen rows.
    selection: Option<((u16, u16), (u16, u16))>,
    /// Wheel movement not yet a whole line.
    scroll_rest: f32,
}

impl Default for TermView {
    fn default() -> Self {
        Self { parser: vt100::Parser::new(nz(24), nz(80), SCROLLBACK), size: (80, 24), selection: None, scroll_rest: 0.0 }
    }
}

impl TermView {
    pub fn feed(&mut self, bytes: &[u8]) {
        self.parser.process(bytes);
    }

    /// Back to an empty screen (before a new socket replays the history), at the same size.
    pub fn reset(&mut self) {
        let (cols, rows) = self.size;
        self.parser = vt100::Parser::new(nz(rows), nz(cols), SCROLLBACK);
        self.selection = None;
    }

    pub fn size(&self) -> (u16, u16) {
        self.size
    }

    /// The screen filling `ui`'s space; keystrokes go to the shell while it has the focus.
    pub fn ui(&mut self, ui: &mut egui::Ui, id: egui::Id) -> TermOutput {
        let p = theme::palette(ui);
        let mut out = TermOutput::default();
        let font = FontId::monospace(13.0);
        let (cell_w, row_h) = ui.fonts_mut(|f| (f.glyph_width(&font, 'M'), f.row_height(&font)));
        let rect = ui.available_rect_before_wrap();
        let response = ui.interact(rect, id, egui::Sense::click_and_drag());
        ui.advance_cursor_after_rect(rect);

        let cols = (((rect.width() - 2.0 * PAD.x) / cell_w).floor() as u16).max(2);
        let rows = (((rect.height() - 2.0 * PAD.y) / row_h).floor() as u16).max(2);
        if (cols, rows) != self.size {
            self.size = (cols, rows);
            self.parser.set_size(nz(rows), nz(cols));
            out.resize = Some((cols, rows));
        }

        if response.clicked() || response.drag_started() {
            response.request_focus();
        }
        let focused = response.has_focus();
        if focused {
            ui.memory_mut(|m| {
                m.set_focus_lock_filter(id, egui::EventFilter { tab: true, horizontal_arrows: true, vertical_arrows: true, escape: true });
            });
        }
        let origin = rect.min + PAD;
        let cell_at = |pos: Pos2| -> (u16, u16) {
            let col = ((pos.x - origin.x) / cell_w).floor().clamp(0.0, f32::from(cols - 1)) as u16;
            let row = ((pos.y - origin.y) / row_h).floor().clamp(0.0, f32::from(rows - 1)) as u16;
            (row, col)
        };
        if response.drag_started()
            && let Some(pos) = response.interact_pointer_pos()
        {
            let at = cell_at(pos);
            self.selection = Some((at, at));
        } else if response.dragged()
            && let (Some(pos), Some((anchor, _))) = (response.interact_pointer_pos(), self.selection)
        {
            self.selection = Some((anchor, cell_at(pos)));
        } else if response.clicked() {
            self.selection = None;
        }

        if response.hovered() {
            let delta = ui.input(|i| i.smooth_scroll_delta.y);
            if delta != 0.0 {
                self.scroll_rest += delta / row_h;
                let lines = self.scroll_rest.trunc();
                self.scroll_rest -= lines;
                let screen = self.parser.screen_mut();
                let now = screen.scrollback() as i64;
                screen.set_scrollback((now + lines as i64).max(0) as usize);
            }
        }

        if focused {
            let screen = self.parser.screen();
            let (app_cursor, bracketed) = (screen.application_cursor(), screen.bracketed_paste());
            let selected = self.selected_text();
            let events = ui.input(|i| i.events.clone());
            for event in &events {
                match event {
                    egui::Event::Copy => match &selected {
                        Some(text) => ui.ctx().copy_text(text.clone()),
                        // Ctrl+C on Linux / Windows arrives as Copy: the interrupt, without a selection.
                        None => out.input.push(0x03),
                    },
                    egui::Event::Cut => out.input.push(0x18),
                    egui::Event::Paste(text) => out.input.extend(paste_bytes(text, bracketed)),
                    _ => {}
                }
            }
            out.input.extend(input_bytes(&events, app_cursor));
            if !out.input.is_empty() {
                self.parser.screen_mut().set_scrollback(0);
                self.selection = None;
            }
        }

        self.paint(ui, rect, origin, cell_w, row_h, &font, focused, p);
        out
    }

    fn selected_text(&self) -> Option<String> {
        let (a, b) = self.selection?;
        let (start, end) = if a <= b { (a, b) } else { (b, a) };
        if start == end {
            return None;
        }
        Some(self.parser.screen().contents_between(start.0, start.1, end.0, end.1 + 1))
    }

    fn selected(&self, row: u16, col: u16) -> bool {
        let Some((a, b)) = self.selection else { return false };
        let (start, end) = if a <= b { (a, b) } else { (b, a) };
        start != end && (row, col) >= start && (row, col) <= end
    }

    #[allow(clippy::too_many_arguments)]
    fn paint(&self, ui: &egui::Ui, rect: Rect, origin: Pos2, cell_w: f32, row_h: f32, font: &FontId, focused: bool, p: &Palette) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0, p.bg);
        let screen = self.parser.screen();
        let dark = ui.visuals().dark_mode;
        let (rows, cols) = (self.size.1, self.size.0);
        for row in 0..rows {
            let y = origin.y + f32::from(row) * row_h;
            let mut col = 0;
            while col < cols {
                let Some(cell) = screen.cell(row, col) else { break };
                let style = Style::of(cell, dark, p);
                let selected = self.selected(row, col);
                let start = col;
                let mut text = String::new();
                while col < cols {
                    let Some(next) = screen.cell(row, col) else { break };
                    if Style::of(next, dark, p) != style || self.selected(row, col) != selected {
                        break;
                    }
                    if !next.is_wide_continuation() {
                        text.push_str(if next.has_contents() { next.contents() } else { " " });
                    }
                    col += 1;
                }
                let run = Rect::from_min_size(egui::pos2(origin.x + f32::from(start) * cell_w, y), egui::vec2(f32::from(col - start) * cell_w, row_h));
                if selected {
                    painter.rect_filled(run, 0, p.accent.gamma_multiply(0.35));
                } else if let Some(bg) = style.bg {
                    painter.rect_filled(run, 0, bg);
                }
                if !text.trim().is_empty() {
                    painter.text(run.left_top(), egui::Align2::LEFT_TOP, &text, font.clone(), style.fg);
                    if style.underline {
                        painter.hline(run.x_range(), run.bottom() - 1.5, Stroke::new(1.0, style.fg));
                    }
                }
            }
        }
        if screen.scrollback() == 0 && !screen.hide_cursor() {
            let (row, col) = screen.cursor_position();
            let at = Rect::from_min_size(egui::pos2(origin.x + f32::from(col) * cell_w, origin.y + f32::from(row) * row_h), egui::vec2(cell_w, row_h));
            if focused {
                painter.rect_filled(at, 1, p.text);
                if let Some(cell) = screen.cell(row, col).filter(|c| c.has_contents()) {
                    painter.text(at.left_top(), egui::Align2::LEFT_TOP, cell.contents(), font.clone(), p.bg);
                }
            } else {
                painter.rect_stroke(at.shrink(0.5), 1, Stroke::new(1.0, p.muted), egui::StrokeKind::Inside);
            }
        }
        if screen.scrollback() > 0 {
            let label = format!("↑ {} lines", screen.scrollback());
            painter.text(rect.right_top() + egui::vec2(-10.0, 6.0), egui::Align2::RIGHT_TOP, label, FontId::proportional(11.0), p.muted);
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct Style {
    fg: Color32,
    bg: Option<Color32>,
    underline: bool,
}

impl Style {
    fn of(cell: &vt100::Cell, dark: bool, p: &Palette) -> Self {
        let mut fg = match cell.fgcolor() {
            vt100::Color::Default => if cell.bold() { p.text_strong } else { p.text },
            vt100::Color::Idx(i) => indexed(if cell.bold() && i < 8 { i + 8 } else { i }, dark),
            vt100::Color::Rgb(r, g, b) => Color32::from_rgb(r, g, b),
        };
        let mut bg = match cell.bgcolor() {
            vt100::Color::Default => None,
            vt100::Color::Idx(i) => Some(indexed(i, dark)),
            vt100::Color::Rgb(r, g, b) => Some(Color32::from_rgb(r, g, b)),
        };
        if cell.inverse() {
            let back = bg.unwrap_or(p.bg);
            bg = Some(fg);
            fg = back;
        }
        if cell.dim() {
            fg = fg.gamma_multiply(0.6);
        }
        Self { fg, bg, underline: cell.underline() }
    }
}

/// The 16 ANSI colours in Kiro's tones, then xterm's 256-colour cube and greys.
pub fn indexed(i: u8, dark: bool) -> Color32 {
    const DARK: [u32; 16] = [
        0x3b3540, 0xf94359, 0x3fb950, 0xe0b94d, 0x6cb6ff, 0xb07fff, 0x56d4dd, 0xdcdadf, 0x6e6874, 0xff6b7d, 0x7ee787, 0xf2cc60, 0x91cbff, 0xc19aff, 0x7ae0e8, 0xffffff,
    ];
    const LIGHT: [u32; 16] = [
        0x2a2730, 0xd92544, 0x16a34a, 0xa38a00, 0x2f6fd6, 0x723acc, 0x0e8a96, 0x71717a, 0x52525b, 0xef4466, 0x22a352, 0xb59600, 0x4b8bf5, 0x8041e6, 0x14a8b6, 0x18181b,
    ];
    let rgb = |v: u32| Color32::from_rgb((v >> 16) as u8, (v >> 8) as u8, v as u8);
    match i {
        0..=15 => rgb(if dark { DARK } else { LIGHT }[usize::from(i)]),
        16..=231 => {
            let n = i - 16;
            let level = |c: u8| if c == 0 { 0 } else { 55 + 40 * c };
            Color32::from_rgb(level(n / 36), level((n / 6) % 6), level(n % 6))
        }
        232..=255 => {
            let v = 8 + 10 * (i - 232);
            Color32::from_rgb(v, v, v)
        }
    }
}

fn paste_bytes(text: &str, bracketed: bool) -> Vec<u8> {
    let text = text.replace("\r\n", "\r").replace('\n', "\r");
    if bracketed { [b"\x1b[200~".as_slice(), text.as_bytes(), b"\x1b[201~"].concat() } else { text.into_bytes() }
}

/// The bytes for typed text and keys (Copy / Cut / Paste are handled by the caller).
pub fn input_bytes(events: &[egui::Event], app_cursor: bool) -> Vec<u8> {
    let mut out = Vec::new();
    let alt_held = events.iter().any(|e| matches!(e, egui::Event::Key { modifiers, pressed: true, .. } if modifiers.alt && !modifiers.command));
    for event in events {
        match event {
            // With Alt held, macOS types symbols (⌥b = ∫): the key event sends ESC + the letter instead.
            egui::Event::Text(text) if !alt_held => out.extend(text.as_bytes()),
            egui::Event::Key { key, pressed: true, modifiers, .. } => {
                if let Some(bytes) = key_bytes(*key, *modifiers, app_cursor) {
                    out.extend(bytes);
                }
            }
            _ => {}
        }
    }
    out
}

fn key_bytes(key: Key, m: Modifiers, app_cursor: bool) -> Option<Vec<u8>> {
    let csi = |s: &str| [b"\x1b[".as_slice(), s.as_bytes()].concat();
    let arrow = |c: u8| {
        if m.ctrl || m.shift || m.alt {
            let code = 1 + u8::from(m.shift) + 2 * u8::from(m.alt) + 4 * u8::from(m.ctrl);
            format!("\x1b[1;{code}{}", c as char).into_bytes()
        } else if app_cursor {
            vec![0x1b, b'O', c]
        } else {
            vec![0x1b, b'[', c]
        }
    };
    let bytes = match key {
        Key::Enter => vec![b'\r'],
        Key::Tab if m.shift => csi("Z"),
        Key::Tab => vec![b'\t'],
        Key::Backspace if m.alt => vec![0x1b, 0x7f],
        Key::Backspace => vec![0x7f],
        Key::Escape => vec![0x1b],
        Key::ArrowUp => arrow(b'A'),
        Key::ArrowDown => arrow(b'B'),
        Key::ArrowRight => arrow(b'C'),
        Key::ArrowLeft => arrow(b'D'),
        Key::Home => csi("H"),
        Key::End => csi("F"),
        Key::PageUp => csi("5~"),
        Key::PageDown => csi("6~"),
        Key::Delete => csi("3~"),
        Key::Insert => csi("2~"),
        Key::F1 => b"\x1bOP".to_vec(),
        Key::F2 => b"\x1bOQ".to_vec(),
        Key::F3 => b"\x1bOR".to_vec(),
        Key::F4 => b"\x1bOS".to_vec(),
        _ => {
            let name = key.name();
            let letter = (name.len() == 1).then(|| name.as_bytes()[0].to_ascii_lowercase()).filter(u8::is_ascii_lowercase)?;
            // Cmd is the app's (copy, paste, save); Ctrl+letter is the control character.
            if m.ctrl && !m.mac_cmd {
                vec![letter - b'a' + 1]
            } else if m.alt && !m.command {
                vec![0x1b, if m.shift { letter.to_ascii_uppercase() } else { letter }]
            } else {
                return None;
            }
        }
    };
    Some(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(key: Key, modifiers: Modifiers) -> egui::Event {
        egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers }
    }

    #[test]
    fn keys_become_what_a_shell_expects() {
        let none = Modifiers::NONE;
        assert_eq!(input_bytes(&[egui::Event::Text("ls".into()), key(Key::Enter, none)], false), b"ls\r");
        assert_eq!(input_bytes(&[key(Key::Backspace, none), key(Key::Tab, none), key(Key::Escape, none)], false), b"\x7f\t\x1b");
        assert_eq!(input_bytes(&[key(Key::ArrowUp, none)], false), b"\x1b[A");
        assert_eq!(input_bytes(&[key(Key::ArrowUp, none)], true), b"\x1bOA", "application cursor mode (vim, less)");
        assert_eq!(input_bytes(&[key(Key::ArrowRight, Modifiers::CTRL)], false), b"\x1b[1;5C");
        assert_eq!(input_bytes(&[key(Key::C, Modifiers::CTRL)], false), [0x03]);
        assert_eq!(input_bytes(&[key(Key::D, Modifiers::CTRL)], false), [0x04]);
        assert_eq!(input_bytes(&[key(Key::C, Modifiers::MAC_CMD | Modifiers::COMMAND)], false), b"", "Cmd+C is the app's copy");
        assert_eq!(input_bytes(&[key(Key::B, Modifiers::ALT), egui::Event::Text("∫".into())], false), b"\x1bb", "Alt+B: back a word");
        assert_eq!(input_bytes(&[key(Key::Delete, none), key(Key::PageUp, none)], false), b"\x1b[3~\x1b[5~");
    }

    #[test]
    fn pastes_are_bracketed_when_the_shell_asks() {
        assert_eq!(paste_bytes("a\nb", false), b"a\rb");
        assert_eq!(paste_bytes("a\r\nb", true), b"\x1b[200~a\rb\x1b[201~");
    }

    #[test]
    fn colours_follow_the_palette_and_xterm() {
        assert_eq!(indexed(1, true), Color32::from_rgb(0xf9, 0x43, 0x59));
        assert_eq!(indexed(16, true), Color32::from_rgb(0, 0, 0));
        assert_eq!(indexed(231, true), Color32::from_rgb(255, 255, 255));
        assert_eq!(indexed(196, true), Color32::from_rgb(255, 0, 0));
        assert_eq!(indexed(244, true), Color32::from_rgb(128, 128, 128));
    }

    #[test]
    fn ansi_output_lands_in_the_grid_with_its_colours() {
        let mut view = TermView::default();
        view.feed(b"plain \x1b[31mred\x1b[0m \x1b[1;32mok\x1b[0m\r\n$ ");
        let screen = view.parser.screen();
        assert_eq!(screen.contents_between(0, 0, 0, 13), "plain red ok");
        let p = &theme::KIRO_DARK;
        assert_eq!(Style::of(screen.cell(0, 6).unwrap(), true, p).fg, indexed(1, true));
        assert_eq!(Style::of(screen.cell(0, 10).unwrap(), true, p).fg, indexed(10, true), "bold green is bright green");
        assert_eq!(Style::of(screen.cell(0, 0).unwrap(), true, p).fg, p.text);
        assert_eq!(screen.cursor_position(), (1, 2));
        view.reset();
        assert_eq!(view.parser.screen().contents(), "");
    }
}
