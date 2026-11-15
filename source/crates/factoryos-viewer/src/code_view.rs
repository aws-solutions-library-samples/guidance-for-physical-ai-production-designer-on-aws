//! The read-only code viewer of the Control Code Export page: a file of a build with line numbers,
//! in monospace, with Structured Text (PLC) and KRL (robot) keywords, comments, strings, numbers
//! and system variables coloured. Lines are laid out only while visible, so large files scroll
//! smoothly.

use crate::theme;
use crate::rr::egui::{self, Color32, FontId, RichText, TextFormat, text::LayoutJob};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lang {
    /// IEC 61131-3 Structured Text (CODESYS).
    St,
    /// KUKA KRL.
    Krl,
    Plain,
}

/// The language of a build file, by its extension.
pub fn language(path: &str) -> Lang {
    let ext = path.rsplit_once('.').map(|(_, ext)| ext.to_ascii_lowercase()).unwrap_or_default();
    match ext.as_str() {
        "st" | "exp" | "iecst" | "gvl" | "pou" => Lang::St,
        "src" | "dat" | "sub" => Lang::Krl,
        _ => Lang::Plain,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Keyword,
    Type,
    Comment,
    String,
    Number,
    /// KRL's `$` system variables and `#` enum values.
    System,
}

/// A coloured range of a line (bytes); the rest of the line is plain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Span {
    pub start: usize,
    pub end: usize,
    pub kind: Kind,
}

const ST_KEYWORDS: &[&str] = &[
    "PROGRAM", "END_PROGRAM", "FUNCTION_BLOCK", "END_FUNCTION_BLOCK", "FUNCTION", "END_FUNCTION", "VAR", "VAR_INPUT",
    "VAR_OUTPUT", "VAR_IN_OUT", "VAR_GLOBAL", "VAR_TEMP", "VAR_STAT", "VAR_EXTERNAL", "VAR_CONFIG", "CONSTANT", "RETAIN",
    "PERSISTENT", "END_VAR", "TYPE", "END_TYPE", "STRUCT", "END_STRUCT", "IF", "THEN", "ELSIF", "ELSE", "END_IF", "CASE",
    "OF", "END_CASE", "FOR", "TO", "BY", "DO", "END_FOR", "WHILE", "END_WHILE", "REPEAT", "UNTIL", "END_REPEAT", "EXIT",
    "CONTINUE", "RETURN", "AND", "AND_THEN", "OR", "OR_ELSE", "XOR", "NOT", "MOD", "TRUE", "FALSE", "ARRAY", "POINTER",
    "REFERENCE", "REF_TO", "AT", "METHOD", "END_METHOD", "PROPERTY", "END_PROPERTY", "INTERFACE", "END_INTERFACE", "EXTENDS",
    "IMPLEMENTS", "THIS", "SUPER", "ACTION", "END_ACTION", "PUBLIC", "PRIVATE", "PROTECTED", "INTERNAL", "ABSTRACT", "FINAL",
    "NAMESPACE", "END_NAMESPACE", "USING",
];
const ST_TYPES: &[&str] = &[
    "BOOL", "BYTE", "WORD", "DWORD", "LWORD", "SINT", "INT", "DINT", "LINT", "USINT", "UINT", "UDINT", "ULINT", "REAL",
    "LREAL", "TIME", "LTIME", "STRING", "WSTRING", "DATE", "TOD", "TIME_OF_DAY", "DT", "DATE_AND_TIME", "CHAR", "WCHAR",
    "ANY", "TON", "TOF", "TP", "R_TRIG", "F_TRIG", "CTU", "CTD", "SR", "RS",
];
const KRL_KEYWORDS: &[&str] = &[
    "DEF", "END", "DEFFCT", "ENDFCT", "DEFDAT", "ENDDAT", "PUBLIC", "GLOBAL", "DECL", "CONST", "STRUC", "ENUM", "SIGNAL",
    "IF", "THEN", "ELSE", "ENDIF", "WHILE", "ENDWHILE", "LOOP", "ENDLOOP", "FOR", "TO", "STEP", "ENDFOR", "REPEAT", "UNTIL",
    "SWITCH", "CASE", "DEFAULT", "ENDSWITCH", "WAIT", "SEC", "FOR", "GOTO", "HALT", "RETURN", "EXIT", "CONTINUE", "PTP",
    "LIN", "CIRC", "SPLINE", "ENDSPLINE", "SPL", "SLIN", "SCIRC", "SPTP", "PTP_REL", "LIN_REL", "CIRC_REL", "C_PTP", "C_DIS",
    "C_VEL", "C_ORI", "C_SPL", "INTERRUPT", "WHEN", "DO", "ON", "OFF", "TRIGGER", "DISTANCE", "DELAY", "PATH", "BRAKE",
    "RESUME", "IMPORT", "IS", "EXT", "EXTFCT", "AND", "OR", "NOT", "EXOR", "B_AND", "B_OR", "B_NOT", "B_EXOR", "TRUE", "FALSE",
    "IN", "OUT",
];
const KRL_TYPES: &[&str] = &["INT", "REAL", "BOOL", "CHAR", "FRAME", "POS", "E6POS", "AXIS", "E6AXIS", "LOAD"];

/// The coloured spans of each line of `text` (as [`str::lines`] splits it).
pub fn highlight(text: &str, lang: Lang) -> Vec<Vec<Span>> {
    // The closing of a block comment that runs over the end of a line.
    let mut open: Option<&'static str> = None;
    text.lines().map(|line| if lang == Lang::Plain { Vec::new() } else { line_spans(line, lang, &mut open) }).collect()
}

fn line_spans(line: &str, lang: Lang, open: &mut Option<&'static str>) -> Vec<Span> {
    let b = line.as_bytes();
    let mut spans = Vec::new();
    let mut i = 0;
    let push = |spans: &mut Vec<Span>, start, end, kind| {
        if end > start {
            spans.push(Span { start, end, kind });
        }
    };
    if let Some(close) = *open {
        match line.find(close) {
            Some(at) => {
                i = at + close.len();
                *open = None;
            }
            None => i = b.len(),
        }
        push(&mut spans, 0, i, Kind::Comment);
    }
    // KRL's `&ACCESS`, `&REL` … header lines.
    if lang == Lang::Krl && i == 0 && line.trim_start().starts_with('&') {
        push(&mut spans, 0, b.len(), Kind::Comment);
        return spans;
    }
    let ident = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    while i < b.len() {
        let c = b[i];
        let rest = &line[i..];
        let comment_to_end = match lang {
            Lang::St => rest.starts_with("//"),
            Lang::Krl => c == b';',
            Lang::Plain => false,
        };
        if comment_to_end {
            push(&mut spans, i, b.len(), Kind::Comment);
            break;
        }
        if lang == Lang::St
            && let Some((opening, close)) = [("(*", "*)"), ("/*", "*/")].into_iter().find(|(o, _)| rest.starts_with(o))
        {
            let start = i;
            match line[i + opening.len()..].find(close) {
                Some(at) => i += opening.len() + at + close.len(),
                None => {
                    i = b.len();
                    *open = Some(close);
                }
            }
            push(&mut spans, start, i, Kind::Comment);
            continue;
        }
        let quote = match lang {
            Lang::St => c == b'\'' || c == b'"',
            Lang::Krl => c == b'"',
            Lang::Plain => false,
        };
        if quote {
            let start = i;
            i += 1;
            while i < b.len() && b[i] != c {
                // `$'` / `$"` are escapes in ST strings.
                i += if lang == Lang::St && b[i] == b'$' { 2 } else { 1 };
            }
            i = (i + 1).min(b.len());
            push(&mut spans, start, i, Kind::String);
            continue;
        }
        if lang == Lang::Krl && (c == b'$' || c == b'#') && b.get(i + 1).is_some_and(|&n| ident(n)) {
            let start = i;
            i += 1;
            while i < b.len() && ident(b[i]) {
                i += 1;
            }
            push(&mut spans, start, i, Kind::System);
            continue;
        }
        if c.is_ascii_digit() {
            let start = i;
            while i < b.len() && (ident(b[i]) || b[i] == b'.' || b[i] == b'#') {
                i += 1;
            }
            push(&mut spans, start, i, Kind::Number);
            continue;
        }
        if ident(c) {
            let start = i;
            while i < b.len() && ident(b[i]) {
                i += 1;
            }
            // ST typed literals: `T#5S`, `16#FF`, `INT#3`.
            if lang == Lang::St && b.get(i) == Some(&b'#') {
                i += 1;
                while i < b.len() && (ident(b[i]) || b[i] == b'.') {
                    i += 1;
                }
                push(&mut spans, start, i, Kind::Number);
                continue;
            }
            let word = line[start..i].to_ascii_uppercase();
            let (keywords, types) = if lang == Lang::St { (ST_KEYWORDS, ST_TYPES) } else { (KRL_KEYWORDS, KRL_TYPES) };
            if keywords.contains(&word.as_str()) {
                push(&mut spans, start, i, Kind::Keyword);
            } else if types.contains(&word.as_str()) {
                push(&mut spans, start, i, Kind::Type);
            }
            continue;
        }
        // Not ASCII, or punctuation: plain (a multi-byte character's bytes are all skipped here,
        // so spans start and end on character boundaries).
        i += 1;
    }
    spans
}

/// Kiro Crew's code colours (its JSON and diff tokens), in either theme.
fn color(kind: Kind, dark: bool) -> Color32 {
    let p = if dark { &theme::KIRO_DARK } else { &theme::KIRO_LIGHT };
    match kind {
        Kind::Keyword => p.code_keyword,
        Kind::Type => p.info,
        Kind::Comment => p.muted,
        Kind::String => p.code_string,
        Kind::Number => p.code_number,
        Kind::System => p.warn,
    }
}

/// A file's text, split into lines and highlighted once.
pub struct Code {
    pub lines: Vec<String>,
    spans: Vec<Vec<Span>>,
    /// The whole text (for Copy).
    pub text: String,
}

impl Code {
    pub fn new(path: &str, text: &str) -> Self {
        let text = text.replace('\t', "    ");
        let spans = highlight(&text, language(path));
        let lines = text.lines().map(str::to_owned).collect();
        Self { lines, spans, text }
    }

    fn line_job(&self, index: usize, font: &FontId, plain: Color32, dark: bool) -> LayoutJob {
        let line = &self.lines[index];
        let mut job = LayoutJob::default();
        let format = |color| TextFormat { font_id: font.clone(), color, ..Default::default() };
        let mut at = 0;
        for span in self.spans.get(index).into_iter().flatten() {
            if span.start > at {
                job.append(&line[at..span.start], 0.0, format(plain));
            }
            job.append(&line[span.start..span.end], 0.0, format(color(span.kind, dark)));
            at = span.end;
        }
        if at < line.len() || line.is_empty() {
            job.append(&line[at..], 0.0, format(plain));
        }
        job
    }

    /// The lines with their numbers, in a scroll area that fills `ui`; `mark`: a line (1-based) to
    /// highlight, `scroll_to`: one to scroll to (this frame).
    pub fn ui(&self, ui: &mut egui::Ui, id: egui::Id, mark: Option<usize>, scroll_to: Option<usize>) {
        let font = egui::TextStyle::Monospace.resolve(ui.style());
        let row_height = ui.ctx().fonts_mut(|f| f.row_height(&font));
        let dark = ui.visuals().dark_mode;
        let p = theme::palette(ui);
        let plain = p.text;
        let gutter = p.muted_strong;
        let digits = self.lines.len().max(1).to_string().len();
        let mut area = egui::ScrollArea::both().id_salt(id).auto_shrink([false, false]);
        if let Some(line) = scroll_to {
            area = area.vertical_scroll_offset(row_height * line.saturating_sub(4) as f32);
        }
        egui::Frame::new().fill(p.bg).inner_margin(egui::Margin::symmetric(18, 14)).show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 0.0;
            // A row is as high as a line of text (not a button), so the rows line up.
            ui.spacing_mut().interact_size.y = row_height;
            area.show_rows(ui, row_height, self.lines.len(), |ui, rows| {
                for index in rows {
                    let marked = mark == Some(index + 1);
                    if marked {
                        // The diagnostic's line, across the whole width.
                        let row = egui::Rect::from_min_size(ui.cursor().min, egui::vec2(ui.clip_rect().right() - ui.cursor().min.x, row_height));
                        ui.painter().rect_filled(row.expand2(egui::vec2(6.0, 0.0)), 2, p.warn_subtle);
                    }
                    ui.horizontal(|ui| {
                        let number = RichText::new(format!("{:>digits$}  ", index + 1)).font(font.clone());
                        let number = number.color(if marked { p.warn } else { gutter });
                        ui.add(egui::Label::new(number).selectable(false));
                        ui.add(egui::Label::new(self.line_job(index, &font, plain, dark)).extend());
                    });
                }
            });
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kinds(line: &str, lang: Lang) -> Vec<(String, Kind)> {
        highlight(line, lang)[0].iter().map(|s| (line[s.start..s.end].to_owned(), s.kind)).collect()
    }

    #[test]
    fn languages_by_extension() {
        assert_eq!(language("plc/codesys/PRG_Cell.st"), Lang::St);
        assert_eq!(language("krl/robot1/Folge1.SRC"), Lang::Krl);
        assert_eq!(language("krl/robot1/Folge1.dat"), Lang::Krl);
        assert_eq!(language("BUILD.md"), Lang::Plain);
        assert_eq!(language("Makefile"), Lang::Plain);
    }

    #[test]
    fn structured_text() {
        use Kind::*;
        assert_eq!(
            kinds("IF bStart AND nStep = 16#0A THEN // go", Lang::St),
            [("IF".into(), Keyword), ("AND".into(), Keyword), ("16#0A".into(), Number), ("THEN".into(), Keyword), ("// go".into(), Comment)]
        );
        assert_eq!(
            kinds("tWait : TON := (PT := T#5S); sName : STRING := 'it$'s';", Lang::St),
            [("TON".into(), Type), ("T#5S".into(), Number), ("STRING".into(), Type), ("'it$'s'".into(), String)]
        );
        assert_eq!(kinds("end_if;", Lang::St), [("end_if".into(), Keyword)], "case-insensitive");
    }

    #[test]
    fn st_block_comments_span_lines() {
        let text = "x := 1; (* one\ntwo\nthree *) y := 2.5;\n/* c */ VAR";
        let spans = highlight(text, Lang::St);
        let lines: Vec<&str> = text.lines().collect();
        let of = |n: usize| spans[n].iter().map(|s| (&lines[n][s.start..s.end], s.kind)).collect::<Vec<_>>();
        assert_eq!(of(0), [("1", Kind::Number), ("(* one", Kind::Comment)]);
        assert_eq!(of(1), [("two", Kind::Comment)]);
        assert_eq!(of(2), [("three *)", Kind::Comment), ("2.5", Kind::Number)]);
        assert_eq!(of(3), [("/* c */", Kind::Comment), ("VAR", Kind::Keyword)]);
    }

    #[test]
    fn krl() {
        use Kind::*;
        assert_eq!(kinds("&ACCESS RVP", Lang::Krl), [("&ACCESS RVP".into(), Comment)]);
        assert_eq!(
            kinds("PTP XP1 C_PTP ; approach", Lang::Krl),
            [("PTP".into(), Keyword), ("C_PTP".into(), Keyword), ("; approach".into(), Comment)]
        );
        assert_eq!(
            kinds("$OUT[17] = TRUE", Lang::Krl),
            [("$OUT".into(), System), ("17".into(), Number), ("TRUE".into(), Keyword)]
        );
        assert_eq!(kinds("DECL E6POS XP1 = {X 1.5}", Lang::Krl)[..2], [("DECL".into(), Keyword), ("E6POS".into(), Type)]);
        assert_eq!(kinds("MSG \"Weld; done\" #NONE", Lang::Krl), [("\"Weld; done\"".into(), String), ("#NONE".into(), System)]);
    }

    #[test]
    fn plain_and_unicode_text() {
        assert!(highlight("IF x THEN", Lang::Plain)[0].is_empty());
        // Spans stay on character boundaries.
        let line = "x := 'Größe'; // Schweißen ✓";
        for span in &highlight(line, Lang::St)[0] {
            assert!(line.is_char_boundary(span.start) && line.is_char_boundary(span.end));
        }
        let code = Code::new("a.st", "VAR\tx : INT;\r\nEND_VAR\r\n");
        assert_eq!(code.lines, ["VAR    x : INT;", "END_VAR"]);
    }
}
