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
//!
//! # Painter vs. GPU
//!
//! [`compute`] turns the samples into world-space line segments + label
//! anchors.  The Software renderer ([`show_diagrams`]) projects and paints
//! them; the GPU renderer consumes the same segments as depth-tested 3-D line
//! strips and only [`show_diagram_labels`] (labels + legend) stays on the
//! painter.  Both paths therefore agree exactly.

use egui;

use viewport_poc::camera::Camera3D;
use viewport_poc::display::{fmt_eng, ResultDisplayState};
use viewport_poc::scene::{SectionForces3D, ViewportScene};
use viewport_poc::selection::SelectionState;
use structural_analysis::FrameAnalysisResult3D;

use super::theme::ViewportPalette;
use crate::ViewportApp;

/// Samples per member along its length.
const SAMPLES: usize = 21;
/// Draw a hatch line back to the baseline every `HATCH_STEP` samples.
const HATCH_STEP: usize = 2;
/// Fraction of the member length used as the on-screen amplitude at peak.
const AMP_FRACTION: f64 = 0.30;

/// Polyline / hatching stroke width (px) — mirror of the painter values.
const POLYLINE_W: f32 = 2.0;
const HATCH_W: f32 = 1.0;
const BASELINE_W: f32 = 1.0;

const C_AXIAL: egui::Color32 = egui::Color32::from_rgb(120, 230, 120);
const C_SHEAR_Y: egui::Color32 = egui::Color32::from_rgb(255, 150, 60);
const C_SHEAR_Z: egui::Color32 = egui::Color32::from_rgb(255, 215, 130);
const C_MOMENT_Z: egui::Color32 = egui::Color32::from_rgb(90, 150, 255);
const C_MOMENT_Y: egui::Color32 = egui::Color32::from_rgb(160, 205, 255);
const C_BASELINE: egui::Color32 = egui::Color32::from_rgb(120, 120, 130);
// The legend box fill / text follow the active theme via `ViewportPalette`
// (`legend_bg` / `legend_border` / `label`), so the legend reads correctly on
// both the dark and the light canvas.  The per-component curve colours above
// are saturated mid-tones that stay legible on either background.

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

    /// RGB triple of the component colour.
    fn rgb(self) -> [u8; 3] {
        let c = self.color();
        [c.r(), c.g(), c.b()]
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

/// A world-space line segment of the diagram overlay (colour + stroke width).
pub(crate) struct DiagramLine {
    pub a: [f64; 3],
    pub b: [f64; 3],
    pub color: [u8; 3],
    pub width: f32,
}

/// A world-space anchor for a painter peak-value label.
pub(crate) struct DiagramLabel {
    pub pos: [f64; 3],
    pub text: String,
    pub color: [u8; 3],
}

/// One legend row: component label, colour, and its peak value.
pub(crate) struct DiagramLegendRow {
    pub label: &'static str,
    pub color: [u8; 3],
    pub peak: f64,
}

/// Fully-computed diagram overlay in world space, shared by both renderers.
pub(crate) struct Diagrams {
    pub lines: Vec<DiagramLine>,
    pub labels: Vec<DiagramLabel>,
    pub legend: Vec<DiagramLegendRow>,
    pub scale: f64,
}

/// Sample the diagram overlay for the current selection / display flags.
///
/// Returns `None` when diagrams are disabled, no result is available, nothing
/// is selected (in selected-member mode), or every sample was degenerate.
///
/// View-only hidden members are skipped, matching the per-entity visibility of
/// the geometry ([`ResultDisplayState::hidden_members`]).  In selected-member
/// mode a hidden current member therefore draws nothing (returns `None`); in
/// *all members* mode only the visible members contribute lines, labels and
/// peaks.  The caller threads in the same [`ResultDisplayState`] the other
/// renderers already receive, so there is no second source of truth.
pub(crate) fn compute(
    scene: &ViewportScene,
    display: &ResultDisplayState,
    result: &FrameAnalysisResult3D,
    selection: SelectionState,
) -> Option<Diagrams> {
    if !display.show_diagrams {
        return None;
    }

    // Which members to draw.  Hidden members are filtered out here so both the
    // painter path and the GPU strip path honour the same visibility.
    let members: Vec<usize> = if display.diagram_all_members {
        (0..scene.n_members())
            .filter(|&mi| {
                scene
                    .member_ids
                    .get(mi)
                    .map_or(true, |&id| !display.is_member_hidden(id))
            })
            .collect()
    } else {
        match selection {
            SelectionState::Member(mid) => {
                // A hidden member draws nothing — even when it is the current
                // (selected) one.
                if display.is_member_hidden(mid) {
                    return None;
                }
                scene.member_index(mid).into_iter().collect()
            }
            _ => return None,
        }
    };
    if members.is_empty() {
        return None;
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
        return None;
    }

    // Active components.
    let mut comps: Vec<Comp> = Vec::new();
    if display.diagram_axial {
        comps.push(Comp::N);
    }
    if display.diagram_shear {
        comps.push(Comp::Vy);
        comps.push(Comp::Vz);
    }
    if display.diagram_moment {
        comps.push(Comp::Mz);
        comps.push(Comp::My);
    }
    if comps.is_empty() {
        return None;
    }

    let mut lines: Vec<DiagramLine> = Vec::new();
    let mut labels: Vec<DiagramLabel> = Vec::new();

    for m in &data {
        // Baseline along the undeformed member axis.
        lines.push(DiagramLine {
            a: m.pi,
            b: m.pj,
            color: [C_BASELINE.r(), C_BASELINE.g(), C_BASELINE.b()],
            width: BASELINE_W,
        });
        let amp = AMP_FRACTION * m.length * display.diagram_scale;
        for &c in &comps {
            let dir = if c.offset_axis() == 2 { m.axes.2 } else { m.axes.1 };
            let rgb = c.rgb();
            let inv_peak = {
                let p = peaks.get(c);
                if p > 0.0 { 1.0 / p } else { 0.0 }
            };

            // Offset polyline, remembering the peak sample.
            let mut bases: Vec<[f64; 3]> = Vec::with_capacity(m.xs.len());
            let mut tips: Vec<[f64; 3]> = Vec::with_capacity(m.xs.len());
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
                bases.push(base);
                tips.push(tip);
            }

            // Hatching from baseline to the offset curve.
            for k in (0..tips.len()).step_by(HATCH_STEP.max(1)) {
                lines.push(DiagramLine {
                    a: bases[k],
                    b: tips[k],
                    color: rgb,
                    width: HATCH_W,
                });
            }
            // Offset polyline.
            for k in 1..tips.len() {
                lines.push(DiagramLine {
                    a: tips[k - 1],
                    b: tips[k],
                    color: rgb,
                    width: POLYLINE_W,
                });
            }
            if peaks.get(c) > 0.0 {
                labels.push(DiagramLabel {
                    pos: tips[peak_idx],
                    text: format!("{}={}", c.label(), fmt_eng(c.value(&m.forces[peak_idx]))),
                    color: rgb,
                });
            }
        }
    }

    let legend = comps
        .iter()
        .map(|&c| DiagramLegendRow {
            label: c.label(),
            color: c.rgb(),
            peak: peaks.get(c),
        })
        .collect();

    Some(Diagrams {
        lines,
        labels,
        legend,
        scale: display.diagram_scale,
    })
}

/// Draw the full internal-force diagram overlay with the painter (Software
/// renderer).  No-op when disabled or when nothing is available to draw.
pub fn show_diagrams(
    app: &ViewportApp,
    scene: &ViewportScene,
    rect: egui::Rect,
    painter: &egui::Painter,
) {
    let Some(result) = app.analysis_result() else {
        return;
    };
    let Some(d) = compute(scene, &app.display, result, app.selection) else {
        return;
    };
    let (w, h) = (rect.width() as f64, rect.height() as f64);
    if w < 10.0 || h < 10.0 {
        return;
    }
    for l in &d.lines {
        if let (Some(a), Some(b)) = (
            project(&app.camera, l.a, w, h, rect),
            project(&app.camera, l.b, w, h, rect),
        ) {
            let col = egui::Color32::from_rgb(l.color[0], l.color[1], l.color[2]);
            painter.line_segment([a, b], egui::Stroke::new(l.width, col));
        }
    }
    paint_labels(&app.camera, &d, w, h, rect, painter);
    draw_legend(painter, rect, &d, ViewportPalette::for_theme(app.dark_mode));
}

/// Painter-only companion used in GPU mode: the 3-D strips are drawn by the
/// renderer, so only the peak-value labels and the legend are painted here.
pub fn show_diagram_labels(
    app: &ViewportApp,
    scene: &ViewportScene,
    rect: egui::Rect,
    painter: &egui::Painter,
) {
    let Some(result) = app.analysis_result() else {
        return;
    };
    let Some(d) = compute(scene, &app.display, result, app.selection) else {
        return;
    };
    let (w, h) = (rect.width() as f64, rect.height() as f64);
    if w < 10.0 || h < 10.0 {
        return;
    }
    paint_labels(&app.camera, &d, w, h, rect, painter);
    draw_legend(painter, rect, &d, ViewportPalette::for_theme(app.dark_mode));
}

/// Paint the projected peak-value labels.
fn paint_labels(
    cam: &Camera3D,
    d: &Diagrams,
    w: f64,
    h: f64,
    rect: egui::Rect,
    painter: &egui::Painter,
) {
    for lb in &d.labels {
        if let Some(p) = project(cam, lb.pos, w, h, rect) {
            painter.text(
                p + egui::vec2(4.0, -4.0),
                egui::Align2::LEFT_BOTTOM,
                &lb.text,
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgb(lb.color[0], lb.color[1], lb.color[2]),
            );
        }
    }
}

/// Short scale legend with the peak value of each active component.
///
/// The box fill / border / text come from the active theme's
/// [`ViewportPalette`], so the legend reads correctly on both the dark and
/// the light canvas (Phase P7-3).
fn draw_legend(painter: &egui::Painter, rect: egui::Rect, d: &Diagrams, pal: ViewportPalette) {
    if d.legend.is_empty() {
        return;
    }
    let pad = 8.0;
    let line_h = 14.0;
    let swatch = 10.0;
    let width = 150.0;
    let height = line_h * (d.legend.len() as f32 + 2.0) + pad;
    let origin = egui::pos2(rect.max.x - width - 10.0, rect.max.y - height - 10.0);
    painter.rect_filled(
        egui::Rect::from_min_size(origin, egui::vec2(width, height)),
        4.0,
        pal.legend_bg,
    );
    painter.rect_stroke(
        egui::Rect::from_min_size(origin, egui::vec2(width, height)),
        4.0,
        egui::Stroke::new(1.0_f32, pal.legend_border),
    );

    let mut y = origin.y + pad * 0.5;
    painter.text(
        egui::pos2(origin.x + pad, y),
        egui::Align2::LEFT_TOP,
        format!("Force diagrams (x{:.2})", d.scale),
        egui::FontId::proportional(11.0),
        pal.label,
    );
    y += line_h;
    for row in &d.legend {
        let sw = egui::Rect::from_min_size(
            egui::pos2(origin.x + pad, y + 2.0),
            egui::vec2(swatch, swatch),
        );
        painter.rect_filled(
            sw,
            1.0,
            egui::Color32::from_rgb(row.color[0], row.color[1], row.color[2]),
        );
        painter.text(
            egui::pos2(origin.x + pad + swatch + 6.0, y),
            egui::Align2::LEFT_TOP,
            format!("{}  peak {}", row.label, fmt_eng(row.peak)),
            egui::FontId::proportional(10.0),
            pal.label,
        );
        y += line_h;
    }
    painter.text(
        egui::pos2(origin.x + pad, y),
        egui::Align2::LEFT_TOP,
        "local axes; N, N\u{00b7}m",
        egui::FontId::proportional(9.0),
        pal.help,
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

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::{EditModel, MemberId, NodalLoadSpec};

    /// Two collinear members (N0–N1–N2): fixed at N0, loaded at N2.
    fn solved_two_member() -> (ViewportScene, FrameAnalysisResult3D, [MemberId; 2]) {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(6.0, 0.0, 0.0).unwrap();
        let m0 = m.add_member(a, b).unwrap();
        let m1 = m.add_member(b, c).unwrap();
        m.fix_node(a).unwrap();
        m.set_nodal_load(c, NodalLoadSpec::new(0.0, -1000.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();
        (scene, res, [m0, m1])
    }

    fn baseline_count(d: &Diagrams) -> usize {
        let c = [C_BASELINE.r(), C_BASELINE.g(), C_BASELINE.b()];
        d.lines.iter().filter(|l| l.color == c).count()
    }

    #[test]
    fn all_members_mode_skips_hidden_members() {
        let (scene, res, [m0, _m1]) = solved_two_member();
        let mut display = ResultDisplayState::default();
        display.show_diagrams = true;
        display.diagram_all_members = true;

        // Nothing hidden → one baseline per member.
        let all = compute(&scene, &display, &res, SelectionState::None).unwrap();
        assert_eq!(baseline_count(&all), 2);

        // Hide one member → it contributes no lines (or labels).
        display.hide_member(m0);
        let filtered = compute(&scene, &display, &res, SelectionState::None).unwrap();
        assert_eq!(baseline_count(&filtered), 1);
        assert!(!filtered.legend.is_empty());
    }

    #[test]
    fn hidden_selected_member_draws_nothing() {
        let (scene, res, [m0, _m1]) = solved_two_member();
        let mut display = ResultDisplayState::default();
        display.show_diagrams = true;
        display.diagram_all_members = false;

        // Visible selected member → Some.
        assert!(compute(&scene, &display, &res, SelectionState::Member(m0)).is_some());

        // Hidden selected member → None (draw nothing).
        display.hide_member(m0);
        assert!(compute(&scene, &display, &res, SelectionState::Member(m0)).is_none());
    }

    #[test]
    fn hiding_every_member_draws_nothing() {
        let (scene, res, [m0, m1]) = solved_two_member();
        let mut display = ResultDisplayState::default();
        display.show_diagrams = true;
        display.diagram_all_members = true;
        display.hide_member(m0);
        display.hide_member(m1);
        assert!(compute(&scene, &display, &res, SelectionState::None).is_none());
    }
}
