//! Centralized UI palette, egui visuals, and light/dark theme toggle.
//!
//! The palette constants are the single source of truth for viewport and
//! panel colors.  [`apply_theme`] installs a compact, CAD-like
//! [`egui::Visuals`] / [`egui::Style`] for the application chrome.

use egui;

// ── Palette ─────────────────────────────────────────────────────────────
// Viewport / semantic colors.  These are deliberately dark-tuned regardless
// of the egui chrome theme, since the 3D canvas always renders on a dark
// background.
pub const BG: egui::Color32 = egui::Color32::from_rgb(25, 25, 30);
pub const GRID_C: egui::Color32 = egui::Color32::from_rgb(50, 50, 55);
pub const AXIS_X: egui::Color32 = egui::Color32::from_rgb(255, 80, 80);
pub const AXIS_Y: egui::Color32 = egui::Color32::from_rgb(80, 255, 80);
pub const AXIS_Z: egui::Color32 = egui::Color32::from_rgb(80, 120, 255);
pub const NODE_C: egui::Color32 = egui::Color32::from_rgb(255, 220, 0);
pub const MEMBER_C: egui::Color32 = egui::Color32::from_rgb(180, 210, 255);
pub const SELECTED_C: egui::Color32 = egui::Color32::from_rgb(255, 165, 0);
pub const DEF_NODE: egui::Color32 = egui::Color32::from_rgb(255, 100, 255);
pub const DEF_MEMBER: egui::Color32 = egui::Color32::from_rgb(180, 100, 255);
pub const LABEL_C: egui::Color32 = egui::Color32::from_rgb(180, 180, 180);
pub const HELP_C: egui::Color32 = egui::Color32::from_rgb(130, 130, 130);
pub const STALE_C: egui::Color32 = egui::Color32::from_rgb(255, 200, 0);
pub const ERROR_C: egui::Color32 = egui::Color32::from_rgb(255, 80, 80);
pub const SUPPORT_C: egui::Color32 = egui::Color32::from_rgb(100, 255, 200);
pub const LOAD_F_C: egui::Color32 = egui::Color32::from_rgb(255, 180, 80);
pub const LOAD_M_C: egui::Color32 = egui::Color32::from_rgb(200, 130, 255);
pub const MEMBER_LOAD_C: egui::Color32 = egui::Color32::from_rgb(100, 255, 100);

// ── Theme ───────────────────────────────────────────────────────────────

/// Install the application theme on `ctx`.
///
/// `dark == true` selects the dark chrome; otherwise the light chrome is
/// used.  A small amount of spacing tightening gives the dense, tool-like
/// feel expected of a structural-analysis package.
pub fn apply_theme(ctx: &egui::Context, dark: bool) {
    let mut visuals = if dark {
        egui::Visuals::dark()
    } else {
        egui::Visuals::light()
    };
    visuals.window_rounding = 4.0.into();
    visuals.menu_rounding = 4.0.into();
    ctx.set_visuals(visuals);
    ctx.style_mut(|style| {
        style.spacing.item_spacing = egui::vec2(6.0, 4.0);
        style.spacing.button_padding = egui::vec2(6.0, 3.0);
    });
}

/// Menu label for the *next* theme (the one a click would switch to).
pub fn toggle_label(dark: bool) -> &'static str {
    if dark {
        "Switch to light theme"
    } else {
        "Switch to dark theme"
    }
}
