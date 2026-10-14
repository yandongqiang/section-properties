//! Centralized UI palette, egui visuals, and light/dark theme toggle.
//!
//! Two layers of colour live here:
//!
//! * the **panel / semantic** constants below are the single source of truth
//!   for the egui panels (tree, inspector, dock, section preview).  They are
//!   dark-tuned by design and are *not* theme-dependent.
//! * [`ViewportPalette`] is the theme-dependent palette for the 3-D canvas.
//!   It is chosen by [`ViewportPalette::for_theme`] and consumed by **both**
//!   renderers — the painter path ([`super::viewport`]) and the GPU path
//!   ([`super::gl_renderer`]) — so the viewport follows the light/dark toggle
//!   instead of staying dark-tuned.
//!
//! [`apply_theme`] installs a compact, CAD-like [`egui::Visuals`] /
//! [`egui::Style`] for the application chrome.

use egui;

// ── Panel / semantic palette (not theme-dependent) ───────────────────────
// Used by the egui panels and the 2-D section preview.  Kept dark-tuned
// regardless of the chrome theme; viewport colours live in
// [`ViewportPalette`].
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

// ── Viewport palette (theme-dependent) ───────────────────────────────────

/// Every colour the 3-D viewport draws, resolved for one theme.
///
/// The dark values are exactly the historical hardcoded constants, so the
/// dark theme is pixel-identical to before.  The light values form a CAD
/// light theme: a near-white canvas, dark steel geometry, and saturated but
/// darkened axes / glyph colours so they stay legible on a bright
/// background.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViewportPalette {
    /// Viewport background (also the GPU clear colour).
    pub bg: egui::Color32,
    /// Ground-grid lines.
    pub grid: egui::Color32,
    /// Global-axis lines / letters (x / y / z).
    pub axis_x: egui::Color32,
    pub axis_y: egui::Color32,
    pub axis_z: egui::Color32,
    /// Member / solid base colour.
    pub member: egui::Color32,
    /// Node markers.
    pub node: egui::Color32,
    /// Selection highlight (members, nodes, end-force arrows, band border).
    pub selected: egui::Color32,
    /// Deformed-shape members / nodes.
    pub def_member: egui::Color32,
    pub def_node: egui::Color32,
    /// Support / reaction glyphs.
    pub support: egui::Color32,
    /// Nodal force arrows / moment arcs.
    pub load_f: egui::Color32,
    pub load_m: egui::Color32,
    /// Member distributed / point load arrows.
    pub member_load: egui::Color32,
    /// Overlay / annotation text drawn on the canvas (labels, legend, help).
    pub label: egui::Color32,
    pub help: egui::Color32,
    /// Box-select rubber band (fill + stroke).
    pub band_fill: egui::Color32,
    pub band_stroke: egui::Color32,
    /// Section-cut plane indicator (translucent quad + border).
    pub plane_fill: egui::Color32,
    pub plane_stroke: egui::Color32,
    /// View-cube gizmo: backdrop, border, lit face base, face text, and the
    /// mid-edge highlight stroke (P7-2).
    pub cube_panel: egui::Color32,
    pub cube_border: egui::Color32,
    pub cube_face: egui::Color32,
    pub cube_text: egui::Color32,
    pub cube_edge: egui::Color32,
}

impl ViewportPalette {
    /// The dark CAD theme (the historical viewport colours).
    pub fn dark() -> Self {
        Self {
            bg: BG,
            grid: GRID_C,
            axis_x: AXIS_X,
            axis_y: AXIS_Y,
            axis_z: AXIS_Z,
            member: MEMBER_C,
            node: NODE_C,
            selected: SELECTED_C,
            def_member: DEF_MEMBER,
            def_node: DEF_NODE,
            support: SUPPORT_C,
            load_f: LOAD_F_C,
            load_m: LOAD_M_C,
            member_load: MEMBER_LOAD_C,
            label: LABEL_C,
            help: HELP_C,
            band_fill: egui::Color32::from_rgba_unmultiplied(120, 190, 255, 40),
            band_stroke: egui::Color32::from_rgba_unmultiplied(120, 190, 255, 130),
            plane_fill: egui::Color32::from_rgba_unmultiplied(120, 190, 255, 28),
            plane_stroke: egui::Color32::from_rgba_unmultiplied(120, 190, 255, 130),
            cube_panel: egui::Color32::from_rgba_premultiplied(18, 18, 24, 175),
            cube_border: GRID_C,
            cube_face: NODE_C,
            cube_text: egui::Color32::from_rgb(30, 30, 36),
            cube_edge: egui::Color32::from_rgba_unmultiplied(255, 235, 150, 90),
        }
    }

    /// The light CAD theme: near-white canvas, dark steel geometry.
    pub fn light() -> Self {
        Self {
            bg: egui::Color32::from_rgb(243, 244, 247),
            grid: egui::Color32::from_rgb(198, 202, 210),
            axis_x: egui::Color32::from_rgb(200, 45, 45),
            axis_y: egui::Color32::from_rgb(35, 150, 60),
            axis_z: egui::Color32::from_rgb(45, 85, 210),
            member: egui::Color32::from_rgb(60, 90, 140),
            node: egui::Color32::from_rgb(185, 145, 0),
            selected: egui::Color32::from_rgb(235, 125, 0),
            def_member: egui::Color32::from_rgb(130, 60, 200),
            def_node: egui::Color32::from_rgb(200, 50, 200),
            support: egui::Color32::from_rgb(0, 135, 105),
            load_f: egui::Color32::from_rgb(200, 105, 15),
            load_m: egui::Color32::from_rgb(135, 60, 195),
            member_load: egui::Color32::from_rgb(25, 135, 60),
            label: egui::Color32::from_rgb(55, 60, 66),
            help: egui::Color32::from_rgb(105, 110, 116),
            band_fill: egui::Color32::from_rgba_unmultiplied(30, 100, 200, 45),
            band_stroke: egui::Color32::from_rgba_unmultiplied(30, 100, 200, 160),
            plane_fill: egui::Color32::from_rgba_unmultiplied(30, 100, 200, 50),
            plane_stroke: egui::Color32::from_rgba_unmultiplied(30, 100, 200, 150),
            cube_panel: egui::Color32::from_rgba_premultiplied(238, 239, 243, 215),
            cube_border: egui::Color32::from_rgb(150, 155, 162),
            cube_face: egui::Color32::from_rgb(75, 115, 170),
            cube_text: egui::Color32::from_rgb(255, 255, 255),
            cube_edge: egui::Color32::from_rgba_unmultiplied(30, 50, 80, 110),
        }
    }

    /// The palette for the current chrome theme (`dark == true` → dark).
    pub fn for_theme(dark: bool) -> Self {
        if dark {
            Self::dark()
        } else {
            Self::light()
        }
    }
}

// ── Theme ───────────────────────────────────────────────────────────────

/// Install the application theme on `ctx`.
///
/// `dark == true` selects the dark chrome; otherwise the light chrome is
/// used.  A small amount of spacing tightening gives the dense, tool-like
/// feel expected of a structural-analysis package.
///
/// The egui light visuals already provide light panels, dark text, visible
/// widget borders and the accent (`Visuals::selection`, read by the ribbon),
/// so both themes are symmetric here; the *viewport* colours follow the
/// theme separately via [`ViewportPalette`].
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The dark palette must reproduce the historical hardcoded viewport
    /// colours exactly, so switching renderers / themes changes nothing in
    /// dark mode.
    #[test]
    fn dark_palette_matches_legacy_constants() {
        let p = ViewportPalette::dark();
        assert_eq!(p.bg, BG);
        assert_eq!(p.grid, GRID_C);
        assert_eq!(p.axis_x, AXIS_X);
        assert_eq!(p.axis_y, AXIS_Y);
        assert_eq!(p.axis_z, AXIS_Z);
        assert_eq!(p.member, MEMBER_C);
        assert_eq!(p.node, NODE_C);
        assert_eq!(p.selected, SELECTED_C);
        assert_eq!(p.def_member, DEF_MEMBER);
        assert_eq!(p.def_node, DEF_NODE);
        assert_eq!(p.support, SUPPORT_C);
        assert_eq!(p.load_f, LOAD_F_C);
        assert_eq!(p.load_m, LOAD_M_C);
        assert_eq!(p.member_load, MEMBER_LOAD_C);
        assert_eq!(p.label, LABEL_C);
        assert_eq!(p.help, HELP_C);
    }

    /// The light theme must actually differ from the dark one on the canvas
    /// colours (a near-white background with dark geometry), and every
    /// glyph colour must stay distinguishable from the light background.
    #[test]
    fn light_palette_is_a_real_light_theme() {
        let d = ViewportPalette::dark();
        let l = ViewportPalette::light();
        assert_ne!(l.bg, d.bg);
        // Light background is bright; dark geometry is dark.
        assert!(l.bg.r() > 200 && l.bg.g() > 200 && l.bg.b() > 200);
        assert!(l.member.r() < 120 && l.member.g() < 120 && l.member.b() < 160);
        // Annotation text is dark on the light canvas.
        assert!(l.label.r() < 100 && l.help.r() < 130);
        // Axis colours stay legible: none of them is near-white.
        for c in [l.axis_x, l.axis_y, l.axis_z, l.support, l.load_f, l.load_m, l.member_load, l.node, l.def_node, l.def_member] {
            assert!(c.r() < 240 || c.g() < 240 || c.b() < 240, "{c:?} too close to white");
        }
    }

    #[test]
    fn for_theme_selects_the_matching_palette() {
        assert_eq!(ViewportPalette::for_theme(true), ViewportPalette::dark());
        assert_eq!(ViewportPalette::for_theme(false), ViewportPalette::light());
    }
}
