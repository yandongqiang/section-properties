//! Hand-painted line-art icons for the application chrome.
//!
//! Deliberately dependency-free: every glyph is drawn with
//! [`egui::Painter`] primitives inside a caller-supplied [`egui::Rect`], so no
//! icon-font crate or `egui_extras` feature is required.  All icons share one
//! stroke width and take their colour from the caller (normally
//! `ui.visuals`), which keeps them visually consistent.
//!
//! The set is curated as a small **glyph library**: the title-bar panel
//! toggles, the right-panel mode switcher and the bottom dock tabs draw the
//! `panel_*` / `search` / `agent` / `outline` / `problems` / `history` glyphs,
//! while the earlier geometry / boundary / view glyphs are kept for the
//! planned centre-workspace toolbars (Phase 159-C/D).  Because a binary crate
//! has no public API, a glyph that no surface draws yet would otherwise trip
//! the `dead_code` lint, so the module opts out of that lint — the library is
//! intentionally larger than any single frame's usage.
#![allow(dead_code)]

use egui::{Color32, Painter, Pos2, Rect, Stroke};

/// Stroke width shared by every icon (points).
const W: f32 = 1.4;

/// Signature of an icon-painting function.
pub type IconFn = fn(&Painter, Rect, Color32);

fn stroke(c: Color32) -> Stroke {
    Stroke::new(W, c)
}

/// Map unit coordinates (`0.0..=1.0` inside `r`) to an absolute position.
fn p(r: Rect, x: f32, y: f32) -> Pos2 {
    Pos2::new(r.left() + x * r.width(), r.top() + y * r.height())
}

fn seg(pa: &Painter, r: Rect, c: Color32, a: (f32, f32), b: (f32, f32)) {
    pa.line_segment([p(r, a.0, a.1), p(r, b.0, b.1)], stroke(c));
}

/// Draw an open polyline through unit coordinates.
fn poly(pa: &Painter, r: Rect, c: Color32, pts: &[(f32, f32)]) {
    for w in pts.windows(2) {
        seg(pa, r, c, w[0], w[1]);
    }
}

/// Draw a closed outline through unit coordinates.
fn closed(pa: &Painter, r: Rect, c: Color32, pts: &[(f32, f32)]) {
    poly(pa, r, c, pts);
    if let (Some(first), Some(last)) = (pts.first(), pts.last()) {
        seg(pa, r, c, *last, *first);
    }
}

fn circle(pa: &Painter, r: Rect, c: Color32, cx: f32, cy: f32, rad: f32, filled: bool) {
    let center = p(r, cx, cy);
    let radius = rad * r.width().min(r.height());
    if filled {
        pa.circle_filled(center, radius, c);
    } else {
        pa.circle_stroke(center, radius, stroke(c));
    }
}

// ── File ────────────────────────────────────────────────────────────────

/// A blank page with a folded corner — "New".
pub fn new_file(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[
            (0.30, 0.12),
            (0.66, 0.12),
            (0.78, 0.24),
            (0.78, 0.88),
            (0.30, 0.88),
        ],
    );
    poly(pa, r, c, &[(0.66, 0.12), (0.66, 0.24), (0.78, 0.24)]);
}

/// An open folder — "Open".
pub fn open_folder(pa: &Painter, r: Rect, c: Color32) {
    poly(
        pa,
        r,
        c,
        &[
            (0.14, 0.78),
            (0.14, 0.28),
            (0.42, 0.28),
            (0.50, 0.38),
            (0.76, 0.38),
        ],
    );
    closed(
        pa,
        r,
        c,
        &[(0.14, 0.78), (0.88, 0.78), (0.78, 0.46), (0.06, 0.46)],
    );
}

/// A floppy disk — "Save".
pub fn save_floppy(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[
            (0.18, 0.18),
            (0.70, 0.18),
            (0.82, 0.30),
            (0.82, 0.82),
            (0.18, 0.82),
        ],
    );
    closed(
        pa,
        r,
        c,
        &[(0.36, 0.18), (0.64, 0.18), (0.64, 0.42), (0.36, 0.42)],
    );
    seg(pa, r, c, (0.54, 0.24), (0.54, 0.36));
    closed(
        pa,
        r,
        c,
        &[(0.32, 0.54), (0.68, 0.54), (0.68, 0.82), (0.32, 0.82)],
    );
}

// ── Edit ────────────────────────────────────────────────────────────────

/// A hooked arrow to the left — "Undo".
pub fn undo(pa: &Painter, r: Rect, c: Color32) {
    poly(pa, r, c, &[(0.80, 0.70), (0.80, 0.42), (0.40, 0.42)]);
    seg(pa, r, c, (0.40, 0.42), (0.55, 0.31));
    seg(pa, r, c, (0.40, 0.42), (0.55, 0.53));
}

/// A hooked arrow to the right — "Redo".
pub fn redo(pa: &Painter, r: Rect, c: Color32) {
    poly(pa, r, c, &[(0.20, 0.70), (0.20, 0.42), (0.60, 0.42)]);
    seg(pa, r, c, (0.60, 0.42), (0.45, 0.31));
    seg(pa, r, c, (0.60, 0.42), (0.45, 0.53));
}

/// A waste basket — "Delete".
pub fn trash(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.24, 0.28), (0.76, 0.28));
    poly(
        pa,
        r,
        c,
        &[(0.42, 0.28), (0.42, 0.18), (0.58, 0.18), (0.58, 0.28)],
    );
    closed(
        pa,
        r,
        c,
        &[(0.32, 0.28), (0.68, 0.28), (0.64, 0.84), (0.36, 0.84)],
    );
    seg(pa, r, c, (0.45, 0.40), (0.44, 0.72));
    seg(pa, r, c, (0.55, 0.40), (0.56, 0.72));
}

// ── Geometry ────────────────────────────────────────────────────────────

/// A node dot with a plus — "Add Node".
pub fn node_plus(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.36, 0.62, 0.16, false);
    seg(pa, r, c, (0.74, 0.22), (0.74, 0.44));
    seg(pa, r, c, (0.63, 0.33), (0.85, 0.33));
}

/// A line between two nodes — "Add Member".
pub fn member(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.26, 0.74), (0.74, 0.26));
    circle(pa, r, c, 0.24, 0.76, 0.10, true);
    circle(pa, r, c, 0.76, 0.24, 0.10, true);
}

/// An I-section outline — "Section".
pub fn section_i(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.24, 0.16), (0.76, 0.16), (0.76, 0.30), (0.24, 0.30)],
    );
    closed(
        pa,
        r,
        c,
        &[(0.24, 0.70), (0.76, 0.70), (0.76, 0.84), (0.24, 0.84)],
    );
    seg(pa, r, c, (0.44, 0.30), (0.44, 0.70));
    seg(pa, r, c, (0.56, 0.30), (0.56, 0.70));
}

// ── Boundary ────────────────────────────────────────────────────────────

/// A pinned-support triangle — "Support".
pub fn support(pa: &Painter, r: Rect, c: Color32) {
    closed(pa, r, c, &[(0.50, 0.20), (0.16, 0.72), (0.84, 0.72)]);
    seg(pa, r, c, (0.10, 0.80), (0.90, 0.80));
    seg(pa, r, c, (0.28, 0.72), (0.20, 0.86));
    seg(pa, r, c, (0.50, 0.72), (0.42, 0.86));
    seg(pa, r, c, (0.72, 0.72), (0.64, 0.86));
}

/// A downward arrow onto a node — "Nodal Load".
pub fn load_nodal(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.50, 0.16), (0.50, 0.58));
    seg(pa, r, c, (0.50, 0.58), (0.38, 0.44));
    seg(pa, r, c, (0.50, 0.58), (0.62, 0.44));
    circle(pa, r, c, 0.50, 0.74, 0.10, true);
}

/// A distributed load on a beam — "Member Load".
pub fn load_dist(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.14, 0.82), (0.86, 0.82));
    seg(pa, r, c, (0.30, 0.26), (0.70, 0.26));
    for x in [0.30f32, 0.50, 0.70] {
        seg(pa, r, c, (x, 0.26), (x, 0.70));
        seg(pa, r, c, (x, 0.70), (x - 0.08, 0.58));
        seg(pa, r, c, (x, 0.70), (x + 0.08, 0.58));
    }
}

// ── Analysis ────────────────────────────────────────────────────────────

/// A filled play triangle — "Run Analysis".
pub fn play(pa: &Painter, r: Rect, c: Color32) {
    let pts = vec![p(r, 0.34, 0.22), p(r, 0.34, 0.78), p(r, 0.78, 0.50)];
    pa.add(egui::Shape::convex_polygon(pts, c, Stroke::NONE));
}

/// Three slider tracks — "Solver / Job options".
pub fn sliders(pa: &Painter, r: Rect, c: Color32) {
    for (y, knob) in [(0.30f32, 0.40f32), (0.50, 0.66), (0.70, 0.32)] {
        seg(pa, r, c, (0.20, y), (0.80, y));
        circle(pa, r, c, knob, y, 0.09, true);
    }
}

// ── Results ─────────────────────────────────────────────────────────────

/// A displaced node — "Displacement".
pub fn displacement(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.26, 0.70, 0.12, false);
    seg(pa, r, c, (0.40, 0.56), (0.74, 0.30));
    seg(pa, r, c, (0.74, 0.30), (0.60, 0.30));
    seg(pa, r, c, (0.74, 0.30), (0.74, 0.44));
}

/// A moment-style diagram over a baseline — "Force (M/V/N)".
pub fn diagram(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.12, 0.74), (0.88, 0.74));
    poly(pa, r, c, &[(0.14, 0.74), (0.50, 0.24), (0.86, 0.74)]);
}

/// A results table grid — "Tables".
pub fn table(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.18, 0.22), (0.82, 0.22), (0.82, 0.78), (0.18, 0.78)],
    );
    seg(pa, r, c, (0.18, 0.40), (0.82, 0.40));
    seg(pa, r, c, (0.18, 0.59), (0.82, 0.59));
    seg(pa, r, c, (0.50, 0.22), (0.50, 0.78));
}

// ── View ────────────────────────────────────────────────────────────────

/// Four outward arrows — "Fit / zoom extents".
pub fn fit(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.32, 0.32), (0.68, 0.32), (0.68, 0.68), (0.32, 0.68)],
    );
    for (a, tip, b1, b2) in [
        ((0.40, 0.40), (0.12, 0.12), (0.12, 0.28), (0.28, 0.12)),
        ((0.60, 0.40), (0.88, 0.12), (0.88, 0.28), (0.72, 0.12)),
        ((0.40, 0.60), (0.12, 0.88), (0.12, 0.72), (0.28, 0.88)),
        ((0.60, 0.60), (0.88, 0.88), (0.88, 0.72), (0.72, 0.88)),
    ] {
        seg(pa, r, c, a, tip);
        seg(pa, r, c, tip, b1);
        seg(pa, r, c, tip, b2);
    }
}

/// An isometric cube — "Iso".
pub fn cube(pa: &Painter, r: Rect, c: Color32) {
    let top = (0.50, 0.16);
    let tr = (0.84, 0.36);
    let br = (0.84, 0.66);
    let bottom = (0.50, 0.86);
    let bl = (0.16, 0.66);
    let tl = (0.16, 0.36);
    let center = (0.50, 0.56);
    closed(pa, r, c, &[top, tr, br, bottom, bl, tl]);
    seg(pa, r, c, center, top);
    seg(pa, r, c, center, bl);
    seg(pa, r, c, center, br);
}

/// A front elevation glyph — "Front".
pub fn view_front(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.22, 0.28), (0.78, 0.28), (0.78, 0.72), (0.22, 0.72)],
    );
    seg(pa, r, c, (0.50, 0.28), (0.50, 0.72));
    seg(pa, r, c, (0.22, 0.50), (0.78, 0.50));
}

/// A top-face parallelogram — "Top".
pub fn view_top(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.50, 0.24), (0.82, 0.44), (0.50, 0.64), (0.18, 0.44)],
    );
    seg(pa, r, c, (0.50, 0.24), (0.50, 0.64));
    seg(pa, r, c, (0.18, 0.44), (0.82, 0.44));
}

/// A side-face parallelogram — "Right".
pub fn view_right(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.30, 0.28), (0.74, 0.36), (0.74, 0.72), (0.30, 0.64)],
    );
    seg(pa, r, c, (0.52, 0.32), (0.52, 0.68));
}

/// A four-way move cross — "Pan".
pub fn pan(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.50, 0.18), (0.50, 0.82));
    seg(pa, r, c, (0.18, 0.50), (0.82, 0.50));
    seg(pa, r, c, (0.50, 0.18), (0.40, 0.30));
    seg(pa, r, c, (0.50, 0.18), (0.60, 0.30));
    seg(pa, r, c, (0.50, 0.82), (0.40, 0.70));
    seg(pa, r, c, (0.50, 0.82), (0.60, 0.70));
    seg(pa, r, c, (0.18, 0.50), (0.30, 0.40));
    seg(pa, r, c, (0.18, 0.50), (0.30, 0.60));
    seg(pa, r, c, (0.82, 0.50), (0.70, 0.40));
    seg(pa, r, c, (0.82, 0.50), (0.70, 0.60));
}

/// A magnifier with a plus — "Zoom".
pub fn zoom(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.44, 0.42, 0.24, false);
    seg(pa, r, c, (0.60, 0.58), (0.82, 0.80));
    seg(pa, r, c, (0.34, 0.42), (0.54, 0.42));
    seg(pa, r, c, (0.44, 0.32), (0.44, 0.52));
}

// ── Tools ───────────────────────────────────────────────────────────────

/// A sun — "Theme".
pub fn sun(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.50, 0.50, 0.18, false);
    for (dx, dy) in [
        (0.0f32, -1.0f32),
        (0.0, 1.0),
        (-1.0, 0.0),
        (1.0, 0.0),
        (0.7, -0.7),
        (-0.7, -0.7),
        (0.7, 0.7),
        (-0.7, 0.7),
    ] {
        seg(
            pa,
            r,
            c,
            (0.50 + dx * 0.26, 0.50 + dy * 0.26),
            (0.50 + dx * 0.40, 0.50 + dy * 0.40),
        );
    }
}

// ── Chrome / workspace (Phase 159-B) ─────────────────────────────────────

/// A magnifier without a plus — "Search / command palette".
pub fn search(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.42, 0.42, 0.24, false);
    seg(pa, r, c, (0.60, 0.60), (0.84, 0.84));
}

/// A rectangle with its left strip filled — "Project panel".
pub fn panel_left(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.16, 0.22), (0.84, 0.22), (0.84, 0.78), (0.16, 0.78)],
    );
    pa.rect_filled(
        Rect::from_min_max(p(r, 0.16, 0.22), p(r, 0.38, 0.78)),
        0.0,
        c,
    );
}

/// A rectangle with its right strip filled — "Inspector panel".
pub fn panel_right(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.16, 0.22), (0.84, 0.22), (0.84, 0.78), (0.16, 0.78)],
    );
    pa.rect_filled(
        Rect::from_min_max(p(r, 0.62, 0.22), p(r, 0.84, 0.78)),
        0.0,
        c,
    );
}

/// A rectangle with its bottom strip filled — "Bottom dock".
pub fn panel_bottom(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.16, 0.22), (0.84, 0.22), (0.84, 0.78), (0.16, 0.78)],
    );
    pa.rect_filled(
        Rect::from_min_max(p(r, 0.16, 0.60), p(r, 0.84, 0.78)),
        0.0,
        c,
    );
}

/// A four-point sparkle — "Agent" (reserved AI surface).
pub fn agent(pa: &Painter, r: Rect, c: Color32) {
    poly(
        pa,
        r,
        c,
        &[
            (0.46, 0.14),
            (0.55, 0.41),
            (0.82, 0.50),
            (0.55, 0.59),
            (0.46, 0.86),
            (0.37, 0.59),
            (0.10, 0.50),
            (0.37, 0.41),
            (0.46, 0.14),
        ],
    );
    circle(pa, r, c, 0.78, 0.76, 0.06, true);
}

/// An indented hierarchy list — "Outline".
pub fn outline(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.18, 0.26), (0.84, 0.26));
    seg(pa, r, c, (0.36, 0.50), (0.84, 0.50));
    seg(pa, r, c, (0.36, 0.74), (0.84, 0.74));
    // Parent stem linking the top row to the indented children.
    seg(pa, r, c, (0.26, 0.26), (0.26, 0.62));
    seg(pa, r, c, (0.26, 0.62), (0.36, 0.62));
}

/// A warning triangle with an exclamation — "Problems".
pub fn problems(pa: &Painter, r: Rect, c: Color32) {
    closed(pa, r, c, &[(0.50, 0.14), (0.88, 0.82), (0.12, 0.82)]);
    seg(pa, r, c, (0.50, 0.40), (0.50, 0.62));
    circle(pa, r, c, 0.50, 0.72, 0.05, true);
}

/// A clock with a rewind arrow — "History".
pub fn history(pa: &Painter, r: Rect, c: Color32) {
    circle(pa, r, c, 0.54, 0.54, 0.30, false);
    seg(pa, r, c, (0.54, 0.54), (0.54, 0.34));
    seg(pa, r, c, (0.54, 0.54), (0.68, 0.62));
    // Rewind arrow tucked into the top-left.
    seg(pa, r, c, (0.20, 0.40), (0.20, 0.18));
    seg(pa, r, c, (0.20, 0.18), (0.40, 0.26));
}

// ── Window controls (custom, undecorated chrome) ─────────────────────────

/// A single horizontal bar — "Minimize".
pub fn window_minimize(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.28, 0.52), (0.72, 0.52));
}

/// A square outline — "Maximize".
pub fn window_maximize(pa: &Painter, r: Rect, c: Color32) {
    closed(
        pa,
        r,
        c,
        &[(0.26, 0.26), (0.74, 0.26), (0.74, 0.74), (0.26, 0.74)],
    );
}

/// A front square with a second square peeking out behind — "Restore".
pub fn window_restore(pa: &Painter, r: Rect, c: Color32) {
    // Back square (only its top and right edges show).
    poly(
        pa,
        r,
        c,
        &[
            (0.36, 0.36),
            (0.36, 0.20),
            (0.80, 0.20),
            (0.80, 0.64),
            (0.64, 0.64),
        ],
    );
    // Front square.
    closed(
        pa,
        r,
        c,
        &[(0.20, 0.36), (0.64, 0.36), (0.64, 0.80), (0.20, 0.80)],
    );
}

/// A cross — "Close".
pub fn window_close(pa: &Painter, r: Rect, c: Color32) {
    seg(pa, r, c, (0.30, 0.30), (0.70, 0.70));
    seg(pa, r, c, (0.70, 0.30), (0.30, 0.70));
}

// ── Chrome button helpers ────────────────────────────────────────────────

/// Size of an icon-only toggle button (points).
const TOGGLE_W: f32 = 26.0;
/// Icon box size inside a chrome button.
const BTN_ICON: f32 = super::theme::ICON_SIZE;

/// Paint a compact icon-only toggle button and return its interaction
/// response.
///
/// `active` highlights the button (used by the title-bar panel toggles to
/// show a panel that is currently visible); the caller supplies the tooltip
/// and handles the click.
pub fn icon_toggle(ui: &mut egui::Ui, icon: IconFn, active: bool, tooltip: &str) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(TOGGLE_W, super::theme::ROW_HEIGHT),
        egui::Sense::click(),
    );
    if ui.is_rect_visible(rect) {
        let visuals = ui.visuals().clone();
        let bg = if active {
            visuals.selection.bg_fill
        } else if response.hovered() {
            visuals.widgets.hovered.weak_bg_fill
        } else {
            egui::Color32::TRANSPARENT
        };
        let painter = ui.painter();
        if bg != egui::Color32::TRANSPARENT {
            painter.rect_filled(rect, 4.0, bg);
        }
        let fg = if active {
            visuals.selection.stroke.color
        } else {
            visuals.text_color()
        };
        let icon_rect = egui::Rect::from_center_size(
            rect.center(),
            egui::vec2(rect.width() - 8.0, rect.height() - 6.0),
        );
        icon(painter, icon_rect, fg);
    }
    response.on_hover_text(tooltip)
}

/// Paint a compact window-control button (minimize / maximize / close) and
/// return its interaction response.
///
/// Mirrors [`icon_toggle`]'s sizing and icon box so the custom title bar reads
/// as one row, but with window-control semantics: `close` swaps the hover
/// highlight for the theme's warning red — the standard "this closes the app"
/// affordance — while the others use the shared accent hover tint.
pub fn window_control_button(
    ui: &mut egui::Ui,
    icon: IconFn,
    tooltip: &str,
    close: bool,
) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(TOGGLE_W, super::theme::ROW_HEIGHT),
        egui::Sense::click(),
    );
    if ui.is_rect_visible(rect) {
        let visuals = ui.visuals().clone();
        let dark = visuals.dark_mode;
        let hovered = response.hovered();
        let bg = if !hovered {
            egui::Color32::TRANSPARENT
        } else if close {
            super::theme::close_hover(dark)
        } else {
            visuals.widgets.hovered.weak_bg_fill
        };
        let painter = ui.painter();
        if bg != egui::Color32::TRANSPARENT {
            painter.rect_filled(rect, 4.0, bg);
        }
        let fg = if hovered && close {
            super::theme::close_hover_fg(dark)
        } else {
            visuals.text_color()
        };
        let icon_rect = egui::Rect::from_center_size(
            rect.center(),
            egui::vec2(rect.width() - 10.0, rect.height() - 8.0),
        );
        icon(painter, icon_rect, fg);
    }
    response.on_hover_text(tooltip)
}

/// Paint a compact button with a leading icon and a text label, returning its
/// interaction response.
///
/// `selected` gives the segmented-control highlight; `enabled == false`
/// dims the button and disables clicks.
pub fn icon_text_button(
    ui: &mut egui::Ui,
    icon: IconFn,
    text: &str,
    selected: bool,
    enabled: bool,
) -> egui::Response {
    let font = super::theme::body_font();
    let galley = ui
        .painter()
        .layout_no_wrap(text.to_owned(), font, ui.visuals().text_color());
    let pad_x = 7.0;
    let gap = 5.0;
    let width = pad_x * 2.0 + BTN_ICON + gap + galley.size().x;
    let height = super::theme::ROW_HEIGHT;
    let sense = if enabled {
        egui::Sense::click()
    } else {
        egui::Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, height), sense);
    if ui.is_rect_visible(rect) {
        let visuals = ui.visuals().clone();
        let bg = if !enabled {
            egui::Color32::TRANSPARENT
        } else if selected {
            visuals.selection.bg_fill
        } else if response.hovered() {
            visuals.widgets.hovered.weak_bg_fill
        } else {
            egui::Color32::TRANSPARENT
        };
        let painter = ui.painter();
        if bg != egui::Color32::TRANSPARENT {
            painter.rect_filled(rect, 4.0, bg);
        }
        let fg = if !enabled {
            visuals.weak_text_color()
        } else if selected {
            visuals.selection.stroke.color
        } else {
            visuals.text_color()
        };
        let icon_rect = egui::Rect::from_min_size(
            egui::pos2(rect.left() + pad_x, rect.center().y - BTN_ICON * 0.5),
            egui::vec2(BTN_ICON, BTN_ICON),
        );
        icon(painter, icon_rect, fg);
        let text_pos = egui::pos2(
            icon_rect.right() + gap,
            rect.center().y - galley.size().y * 0.5,
        );
        painter.galley(text_pos, galley, fg);
    }
    response
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every chrome glyph must paint without panicking on a real painter, and
    /// the button helpers must be usable headlessly (a disabled button is
    /// never clickable).
    #[test]
    fn chrome_glyphs_and_helpers_paint_headlessly() {
        egui::__run_test_ctx(|ctx| {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::pos2(0.0, 0.0),
                    egui::vec2(400.0, 200.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    for glyph in [
                        search,
                        panel_left,
                        panel_right,
                        panel_bottom,
                        agent,
                        outline,
                        problems,
                        history,
                        window_minimize,
                        window_maximize,
                        window_restore,
                        window_close,
                    ] {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(20.0, 20.0), egui::Sense::hover());
                        glyph(ui.painter(), rect, egui::Color32::WHITE);
                    }
                    let _ = icon_toggle(ui, panel_left, true, "left");
                    let _ = icon_toggle(ui, panel_right, false, "right");
                    let _ = icon_text_button(ui, outline, "Outline", true, true);
                    let disabled = icon_text_button(ui, problems, "Problems", false, false);
                    assert!(!disabled.clicked(), "a disabled button cannot be clicked");
                    // The window-control buttons are freshly drawn here, so
                    // none of them may report a click in this frame.
                    for close in [false, true] {
                        let r = window_control_button(
                            ui,
                            if close { window_close } else { window_minimize },
                            "control",
                            close,
                        );
                        assert!(!r.clicked(), "a freshly drawn control is not clicked");
                    }
                });
            });
        });
    }
}
