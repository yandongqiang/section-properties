//! Hand-painted line-art icons for the ribbon toolbar.
//!
//! Deliberately dependency-free: every glyph is drawn with
//! [`egui::Painter`] primitives inside a caller-supplied [`egui::Rect`], so no
//! icon-font crate or `egui_extras` feature is required.  All icons share one
//! stroke width and take their colour from the caller (normally
//! `ui.visuals`), which keeps them visually consistent across the ribbon.

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
    poly(pa, r, c, &[(0.42, 0.28), (0.42, 0.18), (0.58, 0.18), (0.58, 0.28)]);
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
