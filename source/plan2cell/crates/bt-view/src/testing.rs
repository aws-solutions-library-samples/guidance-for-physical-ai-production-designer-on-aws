//! Unit tests draw headless and read back what was drawn, through the AccessKit tree egui builds.

use egui::accesskit::Role;

/// Draw `add_contents` in one frame of a fresh context, on a laptop-sized screen: every widget's
/// role and text (a label's text, a button's label, a field's value).
pub(crate) fn draw(add_contents: impl FnMut(&mut egui::Ui)) -> Vec<(Role, String)> {
    draw_on(egui::vec2(1440.0, 900.0), add_contents)
}

/// [`draw`] on a screen of this size.
pub(crate) fn draw_on(
    screen: egui::Vec2,
    mut add_contents: impl FnMut(&mut egui::Ui),
) -> Vec<(Role, String)> {
    let ctx = egui::Context::default();
    // Applies from this first pass (fonts are taken at its start).
    crate::fonts::install(&ctx);
    ctx.enable_accesskit();
    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, screen)),
        ..Default::default()
    };
    let mut output = ctx.run_ui(input, |ui| {
        egui::CentralPanel::default().show(ui, |ui| add_contents(ui));
    });
    output.textures_delta.clear();
    output
        .platform_output
        .accesskit_update
        .map(|update| update.nodes)
        .unwrap_or_default()
        .into_iter()
        .map(|(_, node)| {
            let text = node.label().or_else(|| node.value()).unwrap_or_default();
            (node.role(), text.to_string())
        })
        .collect()
}
