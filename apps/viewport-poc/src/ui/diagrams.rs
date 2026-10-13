//! Member internal-force diagram overlay (N / V / M), MIDAS/Abaqus style.
//!
//! For the selected member — or every member, per the *all members* toggle —
//! the sampled axial (N), shear (Vy / Vz) and bending-moment (Mz / My)
//! diagrams are drawn as polylines offset perpendicular to the member, with
//! the member axis as a baseline, faint hatching back to the baseline,
//! peak-value labels and a scale legend.
//!
//! Values come from
//! [`ViewportScene::member_section_forces`](viewport_poc::scene::ViewportScene::member_section_forces),
//! which recovers them from statics of the member's end forces plus its
//! distributed / point loads.  Offsets are along the member's **local**
//! `y` axis (N, Vy, Mz) or `z` axis (Vz, My); see [`SectionForces3D`] for the
//! sign convention.  Units are SI (N, N·m).

use egui;

use viewport_poc::camera::Camera3D;
use viewport_poc::display::fmt_eng;
use viewport_poc::scene::{SectionForces3D, ViewportScene};
use viewport_poc::selection::SelectionState;

use crate::ViewportApp;

/// Samples per member along its length.
const SAMPLES: usize = 21;
/// Draw a hatch line back to the baseline every `HATCH_STEP` samples.
const HATCH_STEP: usize = 2;
/// Fraction of the member length used as the on-screen amplitude at peak.
const AMP_FRACTION: f64 = 0.30;

const C_AXIAL: egui::Color32 = egui::Color32::from_rgb(120, 230, 120);
const C_SHEAR_Y: egui::Color32 = egui::Color32::from_rgb(255, 150, 60);
const C_SHEAR_Z: egui::Color32 = egui::Color32::from_rgb(255, 215, 130);
const C_MOMENT_Z: egui::Color32 = egui::Color32::from_rgb(90, 150, 255);
const C_MOMENT_Y: egui::Color32 = egui::Color32::from_rgb(160, 205, 255);
const C_BASELINE: egui::Color32 = egui::Color32::from_rgb(120, 120, 130);
const C_LABEL: egui::Color32 = egui::Color32::from_rgb(225, 225, 235);
const C_LEGEND_BG: egui::Color32 = egui::Color32::from_rgba_premultiplied(20, 20, 26, 200);

/// One selectable internal-force component.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Comp {
    N,
    Vy,
    Vz,
    Mz,
    My,
}

impl Comp {
    fn value(self, f: &SectionForces3D) -> f64 {
        match self {
            Comp::N => f.n,
            Comp::Vy => f.vy,
            Comp::Vz => f.vz,
            Comp::Mz => f.mz,
            Comp::My => f.my,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Comp::N => "N",
            Comp::Vy => "Vy",
            Comp::Vz => "Vz",
            Comp::Mz => "Mz",
            Comp::My => "My",
        }
    }

    fn color(self) -> egui::Color32 {
        match self {
            Comp::N => C_AXIAL,
            Comp::Vy => C_SHEAR_Y,
            Comp::Vz => C_SHEAR_Z,
            Comp::Mz => C_MOMENT_Z,
            Comp::My => C_MOMENT_Y,
        }
    }

    /// `1` = offset along local `+y`, `2` = along local `+z`.
    fn offset_axis(self) -> usize {
        match self {
            Comp::Vz | Comp::My => 2,
            _ => 1,
        }
    }
}

/// Per-member sampled geometry + forces for drawing.
struct MemberDiagram {
    pi: [f64; 3],
    pj: [f64; 3],
    length: f64,
    axes: ([f64; 3], [f64; 3], [f64; 3]),
    xs: Vec<f64>,
    forces: Vec<SectionForces3D>,
}

/// Peak absolute value of each component across all drawn members.
#[derive(Default)]
struct Peaks {
    n: f64,
    vy: f64,
    vz: f64,
    mz: f64,
    my: f64,
}

impl Peaks {
    fn update(&mut self, f: &SectionForces3D) {
        self.n = self.n.max(f.n.abs());
        self.vy = self.vy.max(f.vy.abs());
        self.vz = self.vz.max(f.vz.abs());
        self.mz = self.mz.max(f.mz.abs());
        self.my = self.my.max(f.my.abs());
    }

    fn get(&self, c: Comp) -> f64 {
        match c {
            Comp::N => self.n,
            Comp::Vy => self.vy,
            Comp::Vz => self.vz,
            Comp::Mz => self.mz,
            Comp::My => self.my,
        }
    }
}

/// Draw the internal-force diagram overlay (no-op when disabled or when no
/// current analysis result / selection is available).
pub fn show_diagrams(
    app: &ViewportApp,
    scene: &ViewportScene,
    rect: egui::Rect,
    painter: &egui::Painter,
) {
    if !app.display.show_diagrams {
        return;
    }
    let Some(result) = app.analysis_result() else {
        return;
    };
    let w = rect.width() as f64;
    let h = rect.height() as f64;
    if w < 10.0 || h < 10.0 {
        return;
    }

    // Which members to draw.
    let members: Vec<usize> = if app.display.diagram_all_members {
        (0..scene.n_members()).collect()
    } else {
        match app.selection {
            SelectionState::Member(mid) => scene.member_index(mid).into_iter().collect(),
            _ => return,
        }
    };
    if members.is_empty() {
        return;
    }

    // First pass: sample forces and accumulate per-component peaks.
    let mut data: Vec<MemberDiagram> = Vec::new();
    let mut peaks = Peaks::default();
    for &mi in &members {
        let Some((ni, nj)) = scene.member_nodes.get(mi).copied() else {
            continue;
        };
        let pi = scene.node_positions[ni];
        let pj = scene.node_positions[nj];
        let Some(length) = scene.member_length(mi) else {
            continue;
        };
        if length < 1e-9 {
            continue;
        }
        let Some(axes) = scene.member_local_axes(mi) else {
            continue;
        };
        let mut xs = Vec::with_capacity(SAMPLES);
        let mut forces = Vec::with_capacity(SAMPLES);
        for s in 0..SAMPLES {
            let xi = s as f64 / (SAMPLES - 1) as f64;
            let Some(f) = scene.member_section_forces(mi, xi, result) else {
                continue;
            };
            if !f.max_abs().is_finite() {
                continue;
            }
            peaks.update(&f);
            xs.push(xi);
            forces.push(f);
        }
        if forces.len() >= 2 {
            data.push(MemberDiagram {
                pi,
                pj,
                length,
                axes,
                xs,
                forces,
            });
        }
    }
    if data.is_empty() {
        return;
    }

    // Active components.
    let mut comps: Vec<Comp> = Vec::new();
    if app.display.diagram_axial {
        comps.push(Comp::N);
    }
    if app.display.diagram_shear {
        comps.push(Comp::Vy);
        comps.push(Comp::Vz);
    }
    if app.display.diagram_moment {
        comps.push(Comp::Mz);
        comps.push(Comp::My);
    }
    if comps.is_empty() {
        return;
    }

    for m in &data {
        // Baseline along the undeformed member axis.
        if let (Some(a), Some(b)) = (
            project(&app.camera, m.pi, w, h, rect),
            project(&app.camera, m.pj, w, h, rect),
        ) {
            painter.line_segment([a, b], egui::Stroke::new(1.0_f32, C_BASELINE));
        }
        let amp = AMP_FRACTION * m.length * app.display.diagram_scale;
        for &c in &comps {
            draw_component(painter, &app.camera, m, c, peaks.get(c), amp, w, h, rect);
        }
    }

    draw_legend(painter, rect, &comps, &peaks, app.display.diagram_scale);
}

/// Draw one component's offset polyline, hatching and peak label for a member.
#[allow(clippy::too_many_arguments)]
fn draw_component(
    painter: &egui::Painter,
    cam: &Camera3D,
    m: &MemberDiagram,
    c: Comp,
    peak: f64,
    amp: f64,
    w: f64,
    h: f64,
    rect: egui::Rect,
) {
    let dir = if c.offset_axis() == 2 { m.axes.2 } else { m.axes.1 };
    let color = c.color();
    let inv_peak = if peak > 0.0 { 1.0 / peak } else { 0.0 };

    // Build the offset polyline, remembering the peak sample.
    let mut pts: Vec<Option<egui::Pos2>> = Vec::with_capacity(m.xs.len());
    let mut base_pts: Vec<Option<egui::Pos2>> = Vec::with_capacity(m.xs.len());
    let mut peak_idx = 0usize;
    let mut peak_abs = -1.0_f64;
    for (k, &xi) in m.xs.iter().enumerate() {
        let value = c.value(&m.forces[k]);
        if value.abs() > peak_abs {
            peak_abs = value.abs();
            peak_idx = k;
        }
        let offset = value * inv_peak * amp;
        let base = lerp(m.pi, m.pj, xi);
        let tip = [
            base[0] + dir[0] * offset,
            base[1] + dir[1] * offset,
            base[2] + dir[2] * offset,
        ];
        base_pts.push(project(cam, base, w, h, rect));
        pts.push(project(cam, tip, w, h, rect));
    }

    // Hatching from the baseline to the offset curve.
    let hatch_stroke = egui::Stroke::new(1.0_f32, color);
    for k in (0..pts.len()).step_by(HATCH_STEP.max(1)) {
        if let (Some(a), Some(b)) = (base_pts[k], pts[k]) {
            painter.line_segment([a, b], hatch_stroke);
        }
    }

    // Offset polyline (split across points that fail to project).
    let stroke = egui::Stroke::new(2.0_f32, color);
    for k in 1..pts.len() {
        if let (Some(a), Some(b)) = (pts[k - 1], pts[k]) {
            painter.line_segment([a, b], stroke);
        }
    }

    // Peak-value label (only when there is something to report).
    let label_point = if peak > 0.0 { pts[peak_idx] } else { None };
    if let Some(p) = label_point {
        let text = format!("{}={}", c.label(), fmt_eng(c.value(&m.forces[peak_idx])));
        painter.text(
            p + egui::vec2(4.0, -4.0),
            egui::Align2::LEFT_BOTTOM,
            text,
            egui::FontId::proportional(10.0),
            color,
        );
    }
}

/// Short scale legend with the peak value of each active component.
fn draw_legend(
    painter: &egui::Painter,
    rect: egui::Rect,
    comps: &[Comp],
    peaks: &Peaks,
    scale: f64,
) {
    let pad = 8.0;
    let line_h = 14.0;
    let swatch = 10.0;
    let width = 150.0;
    let height = line_h * (comps.len() as f32 + 2.0) + pad;
    let origin = egui::pos2(rect.max.x - width - 10.0, rect.max.y - height - 10.0);
    painter.rect_filled(
        egui::Rect::from_min_size(origin, egui::vec2(width, height)),
        4.0,
        C_LEGEND_BG,
    );

    let mut y = origin.y + pad * 0.5;
    painter.text(
        egui::pos2(origin.x + pad, y),
        egui::Align2::LEFT_TOP,
        format!("Force diagrams (x{scale:.2})"),
        egui::FontId::proportional(11.0),
        C_LABEL,
    );
    y += line_h;
    for &c in comps {
        let sw = egui::Rect::from_min_size(
            egui::pos2(origin.x + pad, y + 2.0),
            egui::vec2(swatch, swatch),
        );
        painter.rect_filled(sw, 1.0, c.color());
        painter.text(
            egui::pos2(origin.x + pad + swatch + 6.0, y),
            egui::Align2::LEFT_TOP,
            format!("{}  peak {}", c.label(), fmt_eng(peaks.get(c))),
            egui::FontId::proportional(10.0),
            C_LABEL,
        );
        y += line_h;
    }
    painter.text(
        egui::pos2(origin.x + pad, y),
        egui::Align2::LEFT_TOP,
        "local axes; N, N\u{00b7}m",
        egui::FontId::proportional(9.0),
        egui::Color32::from_rgb(170, 170, 180),
    );
}

/// Project a world point to screen space, or `None` if not visible.
fn project(
    cam: &Camera3D,
    p: [f64; 3],
    w: f64,
    h: f64,
    rect: egui::Rect,
) -> Option<egui::Pos2> {
    cam.project(p, w, h)
        .map(|s| egui::pos2(rect.min.x + s.x as f32, rect.min.y + s.y as f32))
}

/// Linear interpolation between two world points.
fn lerp(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    ]
}
