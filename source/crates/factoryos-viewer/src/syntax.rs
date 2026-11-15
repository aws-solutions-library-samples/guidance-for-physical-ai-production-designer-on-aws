//! Syntax highlighting for the Workspace IDE's editor ([`crate::ide`]), with syntect: its bundled
//! syntaxes (Python, shell, Rust, TOML, YAML, JSON, Markdown, …) plus Structured Text and KRL
//! (`assets/syntaxes/`), coloured like the Control Code Export page ([`crate::code_view`]) from the
//! Kiro palette rather than a TextMate theme file.

use std::str::FromStr as _;
use std::sync::{Arc, OnceLock};

use syntect::easy::HighlightLines;
use syntect::highlighting::{Color, ScopeSelectors, StyleModifier, Theme, ThemeItem, ThemeSettings};
use syntect::parsing::{SyntaxDefinition, SyntaxReference, SyntaxSet};
use syntect::util::LinesWithEndings;

use crate::rr::egui::{self, Color32, FontId, TextFormat, text::LayoutJob};
use crate::theme::{self, Palette};

/// Larger texts are not highlighted (every edit re-highlights the whole text).
pub const HIGHLIGHT_MAX: usize = 192 * 1024;

const STRUCTURED_TEXT: &str = include_str!("../assets/syntaxes/structured_text.sublime-syntax");
const KRL: &str = include_str!("../assets/syntaxes/krl.sublime-syntax");

fn syntaxes() -> &'static SyntaxSet {
    static SET: OnceLock<SyntaxSet> = OnceLock::new();
    SET.get_or_init(|| {
        let mut builder = SyntaxSet::load_defaults_newlines().into_builder();
        for (name, source) in [("Structured Text", STRUCTURED_TEXT), ("KRL", KRL)] {
            match SyntaxDefinition::load_from_str(source, true, None) {
                Ok(syntax) => builder.add(syntax),
                Err(err) => crate::rr::re_log::warn!("{name} syntax: {err}"),
            }
        }
        builder.build()
    })
}

fn themes() -> &'static [Theme; 2] {
    static THEMES: OnceLock<[Theme; 2]> = OnceLock::new();
    THEMES.get_or_init(|| [kiro_theme(&theme::KIRO_LIGHT), kiro_theme(&theme::KIRO_DARK)])
}

fn color(c: Color32) -> Color {
    Color { r: c.r(), g: c.g(), b: c.b(), a: c.a() }
}

/// The code page's colours by TextMate scope: keywords, types, comments, strings, numbers and
/// KRL's system variables as [`crate::code_view`] has them; operators and the rest in the text colour.
fn kiro_theme(p: &Palette) -> Theme {
    let rules: [(&str, Color32); 10] = [
        ("comment, punctuation.definition.comment", p.muted),
        ("string, punctuation.definition.string", p.code_string),
        ("constant.numeric", p.code_number),
        ("keyword - keyword.operator, storage.modifier, constant.language, support.function, markup.heading, entity.name.tag", p.code_keyword),
        ("keyword.operator, punctuation", p.text),
        ("storage.type, entity.name.type, support.type, support.class, entity.other.attribute-name", p.info),
        ("variable.language, constant.other.placeholder", p.warn),
        ("entity.name.function", p.text_strong),
        ("markup.bold, markup.italic, markup.inline.raw, markup.raw", p.code_string),
        ("invalid", p.danger),
    ];
    Theme {
        name: Some("Kiro".to_owned()),
        author: None,
        settings: ThemeSettings { foreground: Some(color(p.text)), background: Some(color(p.bg)), ..Default::default() },
        scopes: rules
            .into_iter()
            .map(|(selector, c)| ThemeItem {
                scope: ScopeSelectors::from_str(selector).expect("a valid scope selector"),
                style: StyleModifier { foreground: Some(color(c)), background: None, font_style: None },
            })
            .collect(),
    }
}

/// The syntax for a file, by its extension (else its first line, e.g. a `#!/usr/bin/env bash`).
pub fn syntax_for<'a>(path: &str, text: &str) -> &'a SyntaxReference {
    let set = syntaxes();
    let name = path.rsplit('/').next().unwrap_or(path);
    let ext = name.rsplit_once('.').map(|(_, ext)| ext).unwrap_or(name);
    set.find_syntax_by_extension(ext)
        .or_else(|| set.find_syntax_by_extension(&ext.to_ascii_lowercase()))
        .or_else(|| set.find_syntax_by_first_line(text.lines().next().unwrap_or("")))
        .unwrap_or_else(|| set.find_syntax_plain_text())
}

/// The language's name, for the editor's status ("Plain Text" if none).
pub fn language_name(path: &str, text: &str) -> &'static str {
    &syntax_for(path, text).name
}

/// `text` coloured by `path`'s syntax, unwrapped (the editor scrolls sideways).
pub fn layout_job(text: &str, path: &str, dark: bool, font: FontId) -> LayoutJob {
    let p = if dark { &theme::KIRO_DARK } else { &theme::KIRO_LIGHT };
    let mut job = LayoutJob::default();
    job.wrap.max_width = f32::INFINITY;
    let plain = |job: &mut LayoutJob, chunk: &str| job.append(chunk, 0.0, TextFormat::simple(font.clone(), p.text));
    if text.len() > HIGHLIGHT_MAX {
        plain(&mut job, text);
        return job;
    }
    let set = syntaxes();
    let mut lines = HighlightLines::new(syntax_for(path, text), &themes()[usize::from(dark)]);
    for line in LinesWithEndings::from(text) {
        match lines.highlight_line(line, set) {
            Ok(ranges) => {
                for (style, chunk) in ranges {
                    let c = style.foreground;
                    job.append(chunk, 0.0, TextFormat::simple(font.clone(), Color32::from_rgba_unmultiplied(c.r, c.g, c.b, c.a)));
                }
            }
            Err(_) => plain(&mut job, line),
        }
    }
    job
}

#[derive(Default)]
struct Highlighter;

impl egui::cache::ComputerMut<(&str, &str, bool, &FontId), LayoutJob> for Highlighter {
    fn compute(&mut self, (text, path, dark, font): (&str, &str, bool, &FontId)) -> LayoutJob {
        layout_job(text, path, dark, font.clone())
    }
}

type HighlightCache = egui::cache::FrameCache<LayoutJob, Highlighter>;

/// The editor's galley: highlighted once per change of the text (cached between frames).
pub fn galley(ui: &egui::Ui, text: &str, path: &str, font: &FontId) -> Arc<egui::Galley> {
    let dark = ui.visuals().dark_mode;
    let job = ui.ctx().memory_mut(|m| m.caches.cache::<HighlightCache>().get((text, path, dark, font)).clone());
    ui.fonts_mut(|f| f.layout_job(job))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn colors(path: &str, text: &str, dark: bool) -> Vec<(String, Color32)> {
        let job = layout_job(text, path, dark, FontId::monospace(12.0));
        job.sections
            .iter()
            .map(|s| (job.text[s.byte_range.start.0..s.byte_range.end.0].to_owned(), s.format.color))
            .filter(|(t, _)| !t.trim().is_empty())
            .collect()
    }

    fn color_of(path: &str, text: &str, word: &str) -> Color32 {
        colors(path, text, true).into_iter().find(|(t, _)| t.trim() == word).unwrap_or_else(|| panic!("{word} in {:?}", colors(path, text, true))).1
    }

    #[test]
    fn plc_and_robot_code_by_extension() {
        assert_eq!(language_name("plc/codesys/main.st", ""), "Structured Text");
        assert_eq!(language_name("plc/codesys/GVL.gvl", ""), "Structured Text");
        assert_eq!(language_name("krl/robot1/weld.src", ""), "KRL");
        assert_eq!(language_name("krl/robot1/weld.dat", ""), "KRL");
        assert_eq!(language_name("biw/_ssm.sh", ""), "Bourne Again Shell (bash)");
        assert_eq!(language_name("biw/biw_generator.py", ""), "Python");
        assert_eq!(language_name("biw/run", "#!/usr/bin/env bash\n"), "Bourne Again Shell (bash)");
        assert_eq!(language_name("notes", "hello"), "Plain Text");
    }

    #[test]
    fn colours_match_the_code_page() {
        let p = &theme::KIRO_DARK;
        let st = "PROGRAM main\nVAR\n  x : INT := 16#FF; // count\nEND_VAR\n(* note *)\nEND_PROGRAM\n";
        assert_eq!(color_of("a.st", st, "PROGRAM"), p.code_keyword);
        assert_eq!(color_of("a.st", st, "INT"), p.info);
        assert_eq!(color_of("a.st", st, "16#FF"), p.code_number);
        assert_eq!(color_of("a.st", st, "// count"), p.muted);
        assert_eq!(color_of("a.st", st, "(* note *)"), p.muted);
        let krl = "DEF weld()\n  $VEL.CP = 0.2 ; slow\n  PTP HOME\nEND\n";
        assert_eq!(color_of("a.src", krl, "DEF"), p.code_keyword);
        assert_eq!(color_of("a.src", krl, "$VEL"), p.warn);
        assert_eq!(color_of("a.src", krl, "; slow"), p.muted);
        assert_eq!(color_of("a.src", krl, "PTP"), p.code_keyword);
        assert_eq!(color_of("a.sh", "set -euo pipefail\n", "set"), p.code_keyword, "shell builtins as in VS Code");
    }

    #[test]
    fn keywords_are_case_insensitive_and_whole_words() {
        let p = &theme::KIRO_DARK;
        assert_eq!(color_of("a.st", "if x then y := 1; end_if\n", "if"), p.code_keyword);
        assert!(colors("a.st", "iffy := 1;\n", true).iter().all(|(t, c)| !t.contains("iffy") || *c != p.code_keyword));
    }

    #[test]
    fn large_texts_stay_plain() {
        let text = "PROGRAM x\n".repeat(HIGHLIGHT_MAX / 10 + 1);
        let job = layout_job(&text, "a.st", true, FontId::monospace(12.0));
        assert_eq!(job.sections.len(), 1);
        assert_eq!(job.text, text, "the text is kept as it is");
    }
}
