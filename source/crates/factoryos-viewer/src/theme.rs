//! The Kiro look of the viewer: Kiro Crew's `kiro-dark` and `kiro-light` palettes as Rerun's
//! design tokens (so Rerun's own panels wear them too), its fonts (Space Grotesk, JetBrains Mono),
//! the [`Palette`] our panes draw with, and the Dark / Light / System choice of the settings.
//!
//! The token files in `assets/theme/` are generated from re_ui's by `assets/theme/generate.py`.

use std::sync::Arc;

use crate::rr::egui::{self, Color32, FontData, FontDefinitions, FontFamily, ThemePreference};
use crate::rr::{eframe, re_log, re_ui};

/// The colours of one theme, named as in Kiro Crew's palettes (website/src/styles/theme-palettes.css).
pub struct Palette {
    pub accent: Color32,
    pub accent_hover: Color32,
    /// Selected rows and chips: the accent, faint.
    pub accent_subtle: Color32,
    /// Text and icons on an accent fill.
    pub accent_fg: Color32,
    /// The page.
    pub bg: Color32,
    /// Cards, fields and inactive tabs.
    pub card: Color32,
    pub hover: Color32,
    pub border: Color32,
    pub border_strong: Color32,
    pub text: Color32,
    pub text_strong: Color32,
    pub muted: Color32,
    pub muted_strong: Color32,
    pub ok: Color32,
    pub warn: Color32,
    pub warn_subtle: Color32,
    pub danger: Color32,
    pub danger_subtle: Color32,
    pub info: Color32,
    /// Code: keywords, strings, numbers (Kiro's JSON key / string / number colours).
    pub code_keyword: Color32,
    pub code_string: Color32,
    pub code_number: Color32,
}

const fn rgba(r: u8, g: u8, b: u8, alpha: f32) -> Color32 {
    Color32::from_rgba_unmultiplied_const(r, g, b, (alpha * 255.0) as u8)
}

pub const KIRO_DARK: Palette = Palette {
    accent: Color32::from_rgb(0x8e, 0x48, 0xff),
    accent_hover: Color32::from_rgb(0x9f, 0x63, 0xff),
    accent_subtle: rgba(178, 127, 255, 0.16),
    accent_fg: Color32::WHITE,
    bg: Color32::from_rgb(0x19, 0x16, 0x1d),
    card: Color32::from_rgb(0x21, 0x1d, 0x25),
    hover: Color32::from_rgb(0x28, 0x24, 0x2e),
    border: Color32::from_rgb(0x35, 0x2f, 0x3d),
    border_strong: Color32::from_rgb(0x4a, 0x46, 0x4f),
    text: Color32::from_rgb(0xdc, 0xda, 0xdf),
    text_strong: Color32::from_rgb(0xf2, 0xf1, 0xf4),
    muted: Color32::from_rgb(0x93, 0x8f, 0x9b),
    muted_strong: Color32::from_rgb(0x5e, 0x59, 0x66),
    // Kiro's `--ok` (#008543) is a fill; as text on the dark page it needs to be lighter.
    ok: Color32::from_rgb(0x3f, 0xb9, 0x50),
    warn: Color32::from_rgb(0xe0, 0xb9, 0x4d),
    warn_subtle: rgba(224, 185, 77, 0.12),
    danger: Color32::from_rgb(0xf9, 0x43, 0x59),
    danger_subtle: rgba(249, 67, 89, 0.12),
    info: Color32::from_rgb(0x6f, 0x9a, 0xe0),
    code_keyword: Color32::from_rgb(0xb0, 0x7f, 0xff),
    code_string: Color32::from_rgb(0x7e, 0xe7, 0x87),
    code_number: Color32::from_rgb(0xc1, 0x9a, 0xff),
};

pub const KIRO_LIGHT: Palette = Palette {
    accent: Color32::from_rgb(0x8e, 0x48, 0xff),
    accent_hover: Color32::from_rgb(0x7a, 0x35, 0xe0),
    accent_subtle: rgba(142, 72, 255, 0.12),
    accent_fg: Color32::WHITE,
    bg: Color32::WHITE,
    card: Color32::WHITE,
    hover: Color32::from_rgb(0xf0, 0xf0, 0xf0),
    border: Color32::from_rgb(0xe4, 0xe4, 0xe7),
    border_strong: Color32::from_rgb(0xd4, 0xd4, 0xd8),
    text: Color32::from_rgb(0x4a, 0x46, 0x4f),
    text_strong: Color32::from_rgb(0x19, 0x16, 0x1d),
    muted: Color32::from_rgb(0x5e, 0x59, 0x66),
    muted_strong: Color32::from_rgb(0x4a, 0x46, 0x4f),
    ok: Color32::from_rgb(0x00, 0x85, 0x43),
    warn: Color32::from_rgb(0xa3, 0x8a, 0x00),
    warn_subtle: rgba(163, 138, 0, 0.1),
    danger: Color32::from_rgb(0xd9, 0x25, 0x44),
    danger_subtle: rgba(217, 37, 68, 0.1),
    info: Color32::from_rgb(0x28, 0x50, 0x92),
    code_keyword: Color32::from_rgb(0x72, 0x3a, 0xcc),
    code_string: Color32::from_rgb(0x16, 0xa3, 0x4a),
    code_number: Color32::from_rgb(0x80, 0x41, 0xe6),
};

/// The palette of the theme `ui` is drawn in.
pub fn palette(ui: &egui::Ui) -> &'static Palette {
    of_dark_mode(ui.visuals().dark_mode)
}

/// The palette of this frame's theme, for code without a `Ui` or context at hand (most status
/// colours). The app sets it at the start of every frame ([`begin_frame`]).
pub fn current() -> &'static Palette {
    of_dark_mode(DARK.load(std::sync::atomic::Ordering::Relaxed))
}

static DARK: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Follows the theme (which `System` may change at any time) for [`current`].
pub fn begin_frame(ctx: &egui::Context) {
    DARK.store(ctx.theme() == egui::Theme::Dark, std::sync::atomic::Ordering::Relaxed);
}


fn of_dark_mode(dark: bool) -> &'static Palette {
    if dark { &KIRO_DARK } else { &KIRO_LIGHT }
}

/// Hands Rerun the Kiro design tokens. Before `re_viewer::customize_eframe_and_setup_renderer`,
/// which applies them. Once per process (the backend chooser and the viewer both call it).
pub fn install_design_tokens() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(install_design_tokens_now);
}

fn install_design_tokens_now() {
    let load = |theme, tokens, colors| re_ui::DesignTokens::load_with_color_table(theme, tokens, colors);
    let dark = load(
        egui::Theme::Dark,
        include_str!("../assets/theme/kiro_dark_theme.ron"),
        include_str!("../assets/theme/kiro_dark_colors.ron"),
    );
    let light = load(
        egui::Theme::Light,
        include_str!("../assets/theme/kiro_light_theme.ron"),
        include_str!("../assets/theme/kiro_light_colors.ron"),
    );
    match (dark, light) {
        (Ok(dark), Ok(light)) => {
            if let Err(err) = re_ui::try_set_design_tokens(dark, light) {
                re_log::warn!("Kiro theme not installed: {err}");
            }
        }
        (Err(err), _) | (_, Err(err)) => re_log::warn!("Kiro theme not loaded: {err:#}"),
    }
}

/// The viewer's fonts, every one bundled (egui's `default_fonts` are off: Ubuntu-Light's licence):
/// Space Grotesk, then Inter (Rerun's) for the glyphs it lacks, then the emoji fonts of
/// [`bt_view::fonts`], then JetBrains Mono for the symbols none of them has (✕, ▸, ▾, …); JetBrains
/// Mono and the emoji fonts for monospace.
pub(crate) fn font_definitions() -> FontDefinitions {
    use bt_view::fonts;
    let mut defs = FontDefinitions::empty();
    let mut add = |name: &str, data: &'static [u8], family: FontFamily| {
        defs.font_data.insert(name.to_owned(), Arc::new(FontData::from_static(data)));
        defs.families.entry(family).or_default().push(name.to_owned());
    };
    add("SpaceGrotesk-Medium", include_bytes!("../assets/fonts/SpaceGrotesk-Medium.ttf"), FontFamily::Proportional);
    add("Inter-Medium", include_bytes!("../../../third_party/rerun/src/crates/viewer_support/re_ui/data/Inter-Medium.otf"), FontFamily::Proportional);
    add(fonts::JETBRAINS_MONO_NAME, fonts::JETBRAINS_MONO, FontFamily::Monospace);
    fonts::add_symbol_fallbacks(&mut defs);
    defs.families.entry(FontFamily::Proportional).or_default().push(fonts::JETBRAINS_MONO_NAME.to_owned());
    defs
}

/// Kiro's fonts and rounder corners, over what Rerun set up (after
/// `re_viewer::customize_eframe_and_setup_renderer`).
pub fn apply_style(ctx: &egui::Context) {
    ctx.set_fonts(font_definitions());

    ctx.all_styles_mut(|style| {
        let visuals = &mut style.visuals;
        visuals.window_corner_radius = 8.into();
        visuals.menu_corner_radius = 8.into();
        for widget in [
            &mut visuals.widgets.noninteractive,
            &mut visuals.widgets.inactive,
            &mut visuals.widgets.hovered,
            &mut visuals.widgets.active,
            &mut visuals.widgets.open,
        ] {
            widget.corner_radius = 6.into();
        }
    });
}

/// The whole Kiro look for a window without Rerun's viewer (the native backend chooser).
#[cfg(not(target_arch = "wasm32"))]
pub fn setup_plain_window(cc: &eframe::CreationContext<'_>) {
    install_design_tokens();
    re_ui::apply_style_and_install_loaders(&cc.egui_ctx);
    apply_style(&cc.egui_ctx);
    set_preference(&cc.egui_ctx, load_preference(cc.storage));
}

/// Where the choice is kept (eframe's storage: a file natively, localStorage on the web).
const STORAGE_KEY: &str = "factoryos.theme";

/// `FACTORY_OS_THEME` (dark, light or system; e.g. for screenshots), else the stored choice; Dark
/// when there is none.
pub fn load_preference(storage: Option<&dyn eframe::Storage>) -> ThemePreference {
    #[cfg(not(target_arch = "wasm32"))]
    let forced = std::env::var("FACTORY_OS_THEME").ok();
    #[cfg(target_arch = "wasm32")]
    let forced = None;
    match forced.or_else(|| storage.and_then(|s| s.get_string(STORAGE_KEY))).as_deref() {
        Some("light") => ThemePreference::Light,
        Some("system") => ThemePreference::System,
        _ => ThemePreference::Dark,
    }
}

pub fn save_preference(storage: &mut dyn eframe::Storage, ctx: &egui::Context) {
    // A forced theme is for this run only.
    #[cfg(not(target_arch = "wasm32"))]
    if std::env::var_os("FACTORY_OS_THEME").is_some() {
        return;
    }
    storage.set_string(STORAGE_KEY, name(preference(ctx)).to_owned());
}

pub fn preference(ctx: &egui::Context) -> ThemePreference {
    ctx.options(|o| o.theme_preference)
}

pub fn set_preference(ctx: &egui::Context, preference: ThemePreference) {
    ctx.set_theme(preference);
    begin_frame(ctx);
    #[cfg(target_arch = "wasm32")]
    crate::web::set_page_background(of_dark_mode(ctx.theme() == egui::Theme::Dark).bg);
}

fn name(preference: ThemePreference) -> &'static str {
    match preference {
        ThemePreference::Dark => "dark",
        ThemePreference::Light => "light",
        ThemePreference::System => "system",
    }
}

/// The settings' Appearance row: Dark · Light · System, as chips.
pub fn settings_ui(ui: &mut egui::Ui) {
    let current = preference(ui.ctx());
    ui.horizontal(|ui| {
        ui.strong("Appearance");
        ui.add_space(8.0);
        ui.spacing_mut().button_padding = egui::vec2(10.0, 3.0);
        for (choice, label) in [
            (ThemePreference::Dark, "Dark"),
            (ThemePreference::Light, "Light"),
            (ThemePreference::System, "System"),
        ] {
            if chip(ui, label, current == choice).clicked() && current != choice {
                set_preference(ui.ctx(), choice);
            }
        }
    });
    ui.small(egui::RichText::new("Kiro dark or light; System follows the operating system.").color(palette(ui).muted));
}

/// The top bar's quick switch between dark and light (from System, to the other one).
pub fn toggle_button(ui: &mut egui::Ui) {
    let dark = ui.visuals().dark_mode;
    let (icon, tip) = if dark { ("☀", "Light theme") } else { ("🌙", "Dark theme") };
    if ui.add(egui::Button::new(icon).frame(false)).on_hover_text(tip).clicked() {
        set_preference(ui.ctx(), if dark { ThemePreference::Light } else { ThemePreference::Dark });
    }
}

/// A rounded chip, filled with the accent when selected (Kiro's segmented controls).
pub fn chip(ui: &mut egui::Ui, label: &str, selected: bool) -> egui::Response {
    let p = palette(ui);
    let (fill, text, stroke) = if selected {
        (p.accent, p.accent_fg, p.accent)
    } else {
        (Color32::TRANSPARENT, p.text, p.border)
    };
    ui.add(
        egui::Button::new(egui::RichText::new(label).color(text))
            .fill(fill)
            .stroke(egui::Stroke::new(1.0, stroke))
            .corner_radius(10),
    )
}

/// Our panes' widgets drawn as controls. Rerun draws buttons and combo boxes without a fill or
/// border until hovered (right for its icon rows), so our text buttons ("Save") and pickers read
/// as plain labels: here they get a fill, a border, an accent border on hover and some padding.
/// For our own panes and dialogs only (see [`with_controls`]); Rerun's panels keep their look.
pub fn controls_style(ui: &mut egui::Ui) {
    let p = palette(ui);
    let style = ui.style_mut();
    let widgets = &mut style.visuals.widgets;
    // Rerun's fill of unchecked checkboxes (its secondary button fill): set off from the panel.
    widgets.inactive.weak_bg_fill = widgets.inactive.bg_fill;
    widgets.inactive.bg_stroke = egui::Stroke::new(1.0, p.border_strong);
    widgets.hovered.bg_stroke = egui::Stroke::new(1.0, p.accent);
    widgets.open.bg_stroke = egui::Stroke::new(1.0, p.accent);
    widgets.active.bg_stroke = egui::Stroke::new(1.0, p.accent);
    widgets.active.weak_bg_fill = p.accent_subtle;
    style.spacing.button_padding = egui::vec2(8.0, 3.0);
    // Rerun keeps combo boxes as narrow as their text; a picker should look like one.
    style.spacing.combo_width = style.spacing.combo_width.max(120.0);
}

/// `add` with [`controls_style`], restored afterwards (not a child `Ui`: side panels inside take
/// their room from `ui` itself).
pub fn with_controls<R>(ui: &mut egui::Ui, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let saved = ui.style().clone();
    controls_style(ui);
    let result = add(ui);
    ui.set_style(saved);
    result
}

/// The main action of a pane (Save, Apply, Run, Create): accent-filled, strong text. Add it with
/// `ui.add(..)` / `ui.add_enabled(..)` right away (its hover state is read for the next widget).
pub fn primary(ui: &egui::Ui, text: impl Into<String>) -> egui::Button<'static> {
    let p = palette(ui);
    let hovered = ui.ctx().read_response(ui.next_auto_id()).is_some_and(|r| r.hovered());
    let fill = if hovered { p.accent_hover } else { p.accent };
    egui::Button::new(egui::RichText::new(text).strong().color(p.accent_fg))
        .fill(fill)
        .stroke(egui::Stroke::new(1.0, fill))
}

/// [`primary`], added.
pub fn primary_button(ui: &mut egui::Ui, text: impl Into<String>) -> egui::Response {
    let button = primary(ui, text);
    ui.add(button)
}

/// One choice of a row of exclusive choices (Box · Mesh · URDF): framed even when not selected,
/// so the choices it is not on still read as clickable (a `selectable_label` shows only text).
pub fn segment<'a>(selected: bool, text: impl egui::IntoAtoms<'a>) -> egui::Button<'a> {
    egui::Button::selectable(selected, text).frame_when_inactive(true)
}

/// A pane's main actions, pinned to its bottom so they stay in view while its content scrolls:
/// an `egui::Panel::bottom` inside `ui`, so call it before the pane's (scrolling) content, which
/// then gets the room above. `add` draws hints first, then the buttons in a [`action_buttons`]
/// row: the main action as [`primary`] first, framed secondary buttons after it. `id`: unique per
/// pane.
pub fn action_row<R>(ui: &mut egui::Ui, id: impl Into<egui::Id>, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    egui::Panel::bottom(id)
        .resizable(false)
        .frame(egui::Frame::new().fill(ui.visuals().panel_fill).inner_margin(egui::Margin::symmetric(0, 6)))
        .show(ui, |ui| {
            controls_style(ui);
            ui.spacing_mut().item_spacing.y = 4.0;
            add(ui)
        })
        .inner
}

/// The buttons of an [`action_row`], wrapping onto more lines in a narrow pane.
pub fn action_buttons<R>(ui: &mut egui::Ui, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 6.0;
        add(ui)
    })
    .inner
}

/// A combo box at most `width` wide (e.g. `ui.available_width()`, or what a button beside it
/// leaves), its selected text cut with an ellipsis to fit, the full text on hover when it is cut;
/// the opened list shows its entries at full width (the popup may be wider than the box).
///
/// egui's `ComboBox::width(..).truncate()` alone does not do this: `width` is only the minimum and
/// the text is cut at all the room left in the row, so a long selection takes the row and pushes
/// what follows it (an "Edit" button) past the pane's edge, which widens a resizable pane until
/// the whole text fits.
pub fn truncated_combo<R>(
    ui: &mut egui::Ui,
    id_salt: impl egui::AsIdSalt,
    selected: impl Into<egui::WidgetText>,
    width: f32,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<Option<R>> {
    let selected: egui::WidgetText = selected.into();
    let full = selected.text().to_owned();
    let width = width.max(ui.spacing().icon_width + 2.0 * ui.spacing().button_padding.x);
    let text_room = width - 2.0 * ui.spacing().button_padding.x - ui.spacing().icon_spacing - ui.spacing().icon_width;
    let elided = selected
        .clone()
        .into_galley(ui, Some(egui::TextWrapMode::Extend), f32::INFINITY, egui::TextStyle::Button)
        .size()
        .x
        > text_room + 0.5;
    let shown = ui
        .scope(|ui| {
            ui.set_max_width(width);
            egui::ComboBox::from_id_salt(id_salt)
                .selected_text(selected)
                .width(width)
                .truncate()
                .show_ui(ui, add_contents)
        })
        .inner;
    let response = if elided { shown.response.on_hover_text(full) } else { shown.response };
    egui::InnerResponse { inner: shown.inner, response }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fonts_are_bundled_and_in_order() {
        let defs = font_definitions();
        let family = |f: FontFamily| defs.families[&f].iter().map(String::as_str).collect::<Vec<_>>();
        assert_eq!(
            family(FontFamily::Proportional),
            ["SpaceGrotesk-Medium", "Inter-Medium", "NotoEmoji-Regular", "emoji-icon-font", "JetBrainsMono-Regular"]
        );
        assert_eq!(family(FontFamily::Monospace), ["JetBrainsMono-Regular", "NotoEmoji-Regular", "emoji-icon-font"]);
        for name in defs.families.values().flatten() {
            assert!(defs.font_data.contains_key(name), "{name} has no data");
        }
        // Nothing of egui's `default_fonts` (Ubuntu-Light, Hack).
        assert_eq!(defs.font_data.len(), 5);
    }

    #[test]
    fn text_and_symbols_have_glyphs() {
        let ctx = egui::Context::default();
        ctx.set_fonts(font_definitions());
        let mut output = ctx.run_ui(egui::RawInput::default(), |_| {});
        output.textures_delta.clear();
        for font in [egui::FontId::proportional(13.0), egui::FontId::monospace(13.0)] {
            // Text, then symbols the panes use from emoji-icon-font and JetBrains Mono. (Noto
            // Emoji's own, 🔍 or ✔, are drawn but `has_glyphs` cannot tell: it holds egui's
            // replacement glyph ◻, so every glyph of it counts as missing.)
            for text in ["Cell ready: 2 robots, 42.5 s", "⏸", "⏭", "🗑", "⚙", "☰", "⟳", "✕", "▸", "▾"] {
                assert!(ctx.fonts_mut(|f| f.has_glyphs(&font, text)), "{text:?} in {font:?}");
            }
        }
    }
}
