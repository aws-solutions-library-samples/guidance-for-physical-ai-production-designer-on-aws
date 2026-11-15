//! The fonts of FactoryOS's egui windows (btgui, and the factoryos viewer's fallbacks), bundled
//! here instead of egui's `default_fonts`: those include Ubuntu-Light, whose Ubuntu Font Licence
//! is not one the public sample may carry. Every font here is under the SIL Open Font License 1.1
//! or MIT (its licence text beside it in `assets/fonts/`).
//!
//! Without `default_fonts`, [`egui::FontDefinitions::default`] has no fonts, and egui draws
//! nothing for text: an app sets [`definitions`] (or adds [`add_symbol_fallbacks`] to its own)
//! when it creates its context.

use std::sync::Arc;

use egui::{FontData, FontDefinitions, FontFamily, FontTweak};

/// JetBrains Mono Regular 2.305 (SIL OFL 1.1, `assets/fonts/JetBrainsMono-OFL.txt`).
pub const JETBRAINS_MONO: &[u8] = include_bytes!("../assets/fonts/JetBrainsMono-Regular.ttf");
/// Noto Emoji Regular 1.05, black-and-white emoji (SIL OFL 1.1, `assets/fonts/NotoEmoji-OFL.txt`),
/// as egui's `default_fonts` (epaint_default_fonts 0.36.2) ship it.
pub const NOTO_EMOJI: &[u8] = include_bytes!("../assets/fonts/NotoEmoji-Regular.ttf");
/// emoji-icon-font 1.1, symbols and icons (MIT, `assets/fonts/LICENSE-emoji-icon-font.txt`), as egui's
/// `default_fonts` (epaint_default_fonts 0.36.2) ship it.
pub const EMOJI_ICON: &[u8] = include_bytes!("../assets/fonts/emoji-icon-font.ttf");

pub const JETBRAINS_MONO_NAME: &str = "JetBrainsMono-Regular";
pub const NOTO_EMOJI_NAME: &str = "NotoEmoji-Regular";
pub const EMOJI_ICON_NAME: &str = "emoji-icon-font";

/// JetBrains Mono for both families (proportional text too), then the symbol fallbacks.
pub fn definitions() -> FontDefinitions {
    let mut fonts = FontDefinitions::empty();
    fonts.font_data.insert(
        JETBRAINS_MONO_NAME.to_owned(),
        Arc::new(FontData::from_static(JETBRAINS_MONO)),
    );
    for family in [FontFamily::Proportional, FontFamily::Monospace] {
        fonts
            .families
            .entry(family)
            .or_default()
            .push(JETBRAINS_MONO_NAME.to_owned());
    }
    add_symbol_fallbacks(&mut fonts);
    fonts
}

/// Append Noto Emoji and emoji-icon-font to the proportional and monospace families, after the
/// fonts already there, scaled as egui's defaults do (the emoji, arrows and media symbols the UI
/// uses: ⏸, 🔍, ⚙, …).
pub fn add_symbol_fallbacks(fonts: &mut FontDefinitions) {
    for (name, data, scale) in [
        (NOTO_EMOJI_NAME, NOTO_EMOJI, 0.81),
        (EMOJI_ICON_NAME, EMOJI_ICON, 0.90),
    ] {
        fonts.font_data.insert(
            name.to_owned(),
            Arc::new(FontData::from_static(data).tweak(FontTweak {
                scale,
                ..Default::default()
            })),
        );
        for family in [FontFamily::Proportional, FontFamily::Monospace] {
            let list = fonts.families.entry(family).or_default();
            if !list.iter().any(|n| n == name) {
                list.push(name.to_owned());
            }
        }
    }
}

/// [`definitions`] on `ctx` (effective from its next frame).
pub fn install(ctx: &egui::Context) {
    ctx.set_fonts(definitions());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> egui::Context {
        let ctx = egui::Context::default();
        install(&ctx);
        // set_fonts applies at the start of the next pass.
        let mut output = ctx.run_ui(egui::RawInput::default(), |_| {});
        output.textures_delta.clear();
        ctx
    }

    #[test]
    fn text_and_symbols_have_glyphs() {
        let ctx = context();
        for font in [
            egui::FontId::proportional(13.0),
            egui::FontId::monospace(13.0),
        ] {
            assert!(ctx.fonts_mut(|f| f.has_glyphs(&font, "Plan ready: 3 robots, 42 s")));
            // From emoji-icon-font: what the panels' buttons and headers use. (Noto Emoji's own
            // glyphs, 🔍 or ✔, are drawn but not testable this way: it is the face of egui's
            // replacement glyph ◻, so `has_glyphs` reports every glyph of it as missing.)
            for symbol in ["⏸", "⏵", "⏭", "🗑", "⚙", "⏱", "⟳", "☰"] {
                assert!(
                    ctx.fonts_mut(|f| f.has_glyphs(&font, symbol)),
                    "{symbol} in {font:?}"
                );
            }
        }
    }

    #[test]
    fn fallbacks_come_after_the_hosts_fonts_once() {
        let mut fonts = FontDefinitions::empty();
        fonts
            .families
            .get_mut(&FontFamily::Proportional)
            .unwrap()
            .push("Host".to_owned());
        add_symbol_fallbacks(&mut fonts);
        add_symbol_fallbacks(&mut fonts);
        assert_eq!(
            fonts.families[&FontFamily::Proportional],
            ["Host", NOTO_EMOJI_NAME, EMOJI_ICON_NAME].map(str::to_owned)
        );
        assert_eq!(
            fonts.families[&FontFamily::Monospace],
            [NOTO_EMOJI_NAME, EMOJI_ICON_NAME].map(str::to_owned)
        );
    }
}
