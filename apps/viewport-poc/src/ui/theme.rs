//! Centralized UI palette, egui visuals, and light/dark theme toggle.
//!
//! Three layers of colour live here:
//!
//! * the **panel / semantic** constants below are the dark-tuned historical
//!   values.  They remain the single source of truth for the *dark* theme:
//!   [`ViewportPalette::dark`] and [`TextPalette::dark`] reproduce them
//!   byte-for-byte.
//! * [`ViewportPalette`] is the theme-dependent palette for the 3-D canvas
//!   (and the 2-D section preview).  It is chosen by
//!   [`ViewportPalette::for_theme`] and consumed by **both** renderers — the
//!   painter path ([`super::viewport`]) and the GPU path
//!   ([`super::gl_renderer`]) — so the viewport follows the light/dark toggle
//!   instead of staying dark-tuned.
//! * [`TextPalette`] is the theme-dependent counterpart of the semantic
//!   constants for **chrome text** (panel labels, status badges, message
//!   levels): the dark values are the historical constants, the light values
//!   are darkened so they stay legible on the light egui panels.
//!
//! [`apply_theme`] installs a compact, CAD-like [`egui::Visuals`] /
//! [`egui::Style`] for the application chrome.

use egui;

// ── Panel / semantic palette (dark-tuned historical values) ─────────────
// Used as the *dark* half of [`ViewportPalette`] / [`TextPalette`]; the light
// halves are tuned separately in those palettes.
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

/// Every colour the 3-D viewport (and the 2-D section preview) draws,
/// resolved for one theme.
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
    /// Force-diagram legend box (fill + border) floating on the canvas
    /// (Phase P7-3).
    pub legend_bg: egui::Color32,
    pub legend_border: egui::Color32,
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
            legend_bg: egui::Color32::from_rgba_premultiplied(20, 20, 26, 200),
            legend_border: GRID_C,
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
            legend_bg: egui::Color32::from_rgba_premultiplied(231, 232, 234, 235),
            legend_border: egui::Color32::from_rgb(168, 174, 182),
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

// ── Chrome text palette (theme-dependent) ───────────────────────────────

/// Semantic **text** colours for the egui chrome (panels, dock, ribbon),
/// resolved for one theme.
///
/// These are the theme-aware replacements for the dark-tuned semantic
/// constants: the dark values are byte-identical to the historical
/// constants, while the light values are darkened so status badges, hints
/// and message levels stay legible on the light panels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TextPalette {
    /// Neutral informational text (message level Info).
    pub label: egui::Color32,
    /// De-emphasised hints / secondary text.
    pub help: egui::Color32,
    /// Warning / stale-analysis amber.
    pub stale: egui::Color32,
    /// Error red.
    pub error: egui::Color32,
    /// Success / "current" green.
    pub ok: egui::Color32,
}

impl TextPalette {
    /// The dark chrome text colours (the historical constants).
    pub fn dark() -> Self {
        Self {
            label: LABEL_C,
            help: HELP_C,
            stale: STALE_C,
            error: ERROR_C,
            ok: SUPPORT_C,
        }
    }

    /// The light chrome text colours: the same hues, darkened for a light
    /// background.
    pub fn light() -> Self {
        Self {
            label: egui::Color32::from_rgb(64, 69, 75),
            help: egui::Color32::from_rgb(105, 110, 116),
            stale: egui::Color32::from_rgb(140, 95, 0),
            error: egui::Color32::from_rgb(190, 30, 30),
            ok: egui::Color32::from_rgb(0, 120, 80),
        }
    }

    /// The text palette for the current chrome theme (`dark == true` → dark).
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
/// theme separately via [`ViewportPalette`] and the semantic *text* colours
/// via [`TextPalette`].
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

    /// Rec. 709 relative luminance of a colour, in `0.0..=1.0`.
    ///
    /// Used by the palette tests to assert that every theme colour stays
    /// legible against its own background (dark colours on light, light
    /// colours on dark).
    fn luminance(c: egui::Color32) -> f32 {
        fn channel(v: u8) -> f32 {
            let s = v as f32 / 255.0;
            if s <= 0.03928 {
                s / 12.92
            } else {
                ((s + 0.055) / 1.055).powf(2.4)
            }
        }
        0.2126 * channel(c.r()) + 0.7152 * channel(c.g()) + 0.0722 * channel(c.b())
    }

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
        // The diagram legend box keeps its historical dark fill.
        assert_eq!(
            p.legend_bg,
            egui::Color32::from_rgba_premultiplied(20, 20, 26, 200)
        );
        assert_eq!(p.legend_border, GRID_C);
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
        // The light legend box is a light fill with a visible border.
        assert_ne!(l.legend_bg, d.legend_bg);
        assert_ne!(l.legend_border, d.legend_border);
    }

    #[test]
    fn for_theme_selects_the_matching_palette() {
        assert_eq!(ViewportPalette::for_theme(true), ViewportPalette::dark());
        assert_eq!(ViewportPalette::for_theme(false), ViewportPalette::light());
    }

    /// The dark text palette reproduces the historical semantic constants
    /// exactly.
    #[test]
    fn dark_text_palette_matches_legacy_constants() {
        let t = TextPalette::dark();
        assert_eq!(t.label, LABEL_C);
        assert_eq!(t.help, HELP_C);
        assert_eq!(t.stale, STALE_C);
        assert_eq!(t.error, ERROR_C);
        assert_eq!(t.ok, SUPPORT_C);
    }

    /// The light text palette must actually differ from the dark one, and
    /// every colour must be dark enough to read on the light egui panels
    /// (relative luminance well below mid-grey).
    #[test]
    fn light_text_palette_is_legible_on_light_chrome() {
        let d = TextPalette::dark();
        let l = TextPalette::light();
        assert_ne!(l, d);
        for c in [l.label, l.help, l.stale, l.error, l.ok] {
            assert!(
                luminance(c) < 0.30,
                "{c:?} too bright for light chrome (luminance {})",
                luminance(c)
            );
        }
        // The dark text colours stay bright on the dark chrome.
        for c in [d.label, d.help, d.stale, d.error, d.ok] {
            assert!(
                luminance(c) > 0.20,
                "{c:?} too dim for dark chrome (luminance {})",
                luminance(c)
            );
        }
    }

    #[test]
    fn text_for_theme_selects_the_matching_palette() {
        assert_eq!(TextPalette::for_theme(true), TextPalette::dark());
        assert_eq!(TextPalette::for_theme(false), TextPalette::light());
    }
}
