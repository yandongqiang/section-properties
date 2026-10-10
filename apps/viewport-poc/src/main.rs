//! 3D Viewport Proof of Concept — egui + eframe.
//!
//! Run with: `cargo run -p viewport-poc`

use eframe;
use egui;
use structural_analysis::{Dof3D, FrameAnalysisResult3D, FrameModel3D};
use viewport_poc::{
    camera::{Camera3D, ScreenPos},
    display::ResultDisplayState,
    model, project,
    scene::ViewportScene,
    selection::SelectionState,
};

// ── Colors ──────────────────────────────────────────────────────────────
const BG: egui::Color32 = egui::Color32::from_rgb(25, 25, 30);
const GRID_C: egui::Color32 = egui::Color32::from_rgb(50, 50, 55);
const AXIS_X: egui::Color32 = egui::Color32::from_rgb(255, 80, 80);
const AXIS_Y: egui::Color32 = egui::Color32::from_rgb(80, 255, 80);
const AXIS_Z: egui::Color32 = egui::Color32::from_rgb(80, 120, 255);
const NODE_C: egui::Color32 = egui::Color32::from_rgb(255, 220, 0);
const MEMBER_C: egui::Color32 = egui::Color32::from_rgb(180, 210, 255);
const SELECTED_C: egui::Color32 = egui::Color32::from_rgb(255, 165, 0);
const DEF_NODE: egui::Color32 = egui::Color32::from_rgb(255, 100, 255);
const DEF_MEMBER: egui::Color32 = egui::Color32::from_rgb(180, 100, 255);
const LABEL_C: egui::Color32 = egui::Color32::from_rgb(180, 180, 180);
const HELP_C: egui::Color32 = egui::Color32::from_rgb(130, 130, 130);

// ── App ─────────────────────────────────────────────────────────────────
struct ViewportApp {
    model: FrameModel3D,
    result: Option<FrameAnalysisResult3D>,
    scene: Option<ViewportScene>,
    camera: Camera3D,
    selection: SelectionState,
    display: ResultDisplayState,
}

impl ViewportApp {
    fn new() -> Self {
        let model = model::build_portal_frame();
        let result = model.solve().ok();
        let scene = result.as_ref().map(ViewportScene::from_result);
        let camera = scene
            .as_ref()
            .map(|s| {
                let (min, max) = s.bounding_box();
                Camera3D::fit(min, max)
            })
            .unwrap_or_default();
        Self {
            model,
            result,
            scene,
            camera,
            selection: SelectionState::None,
            display: ResultDisplayState::default(),
        }
    }

    fn pick(&self, mouse: [f64; 2], screen: [f64; 2]) -> SelectionState {
        const THRESH: f64 = 12.0;
        let Some(scene) = &self.scene else {
            return SelectionState::None;
        };
        let w = screen[0];
        let h = screen[1];

        let mut best_node: Option<(usize, f64)> = None;
        for (i, &pos) in scene.node_positions.iter().enumerate() {
            if let Some(p) = self.camera.project(pos, w, h) {
                let d = (p.x - mouse[0]).hypot(p.y - mouse[1]);
                if d < THRESH && best_node.map_or(true, |(_, bd)| d < bd) {
                    best_node = Some((i, d));
                }
            }
        }
        let mut best_mem: Option<(usize, f64)> = None;
        for (i, &(ni, nj)) in scene.member_nodes.iter().enumerate() {
            let Some(pi) = scene.node_positions.get(ni) else {
                continue;
            };
            let Some(pj) = scene.node_positions.get(nj) else {
                continue;
            };
            let Some(ps_i) = self.camera.project(*pi, w, h) else {
                continue;
            };
            let Some(ps_j) = self.camera.project(*pj, w, h) else {
                continue;
            };
            let d = project::point_to_segment_dist_2d(mouse, [ps_i.x, ps_i.y], [ps_j.x, ps_j.y]);
            if d < THRESH && best_mem.map_or(true, |(_, bd)| d < bd) {
                best_mem = Some((i, d));
            }
        }
        match (best_node, best_mem) {
            (Some((ni, nd)), Some((mi, md))) => {
                if nd < md {
                    SelectionState::Node(ni)
                } else {
                    SelectionState::Member(mi)
                }
            }
            (Some((ni, _)), None) => SelectionState::Node(ni),
            (None, Some((mi, _))) => SelectionState::Member(mi),
            (None, None) => SelectionState::None,
        }
    }
}

impl eframe::App for ViewportApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Side panel ───────────────────────────────────────────────────
        egui::SidePanel::right("side")
            .default_width(350.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.show_model_section(ui);
                    self.show_display_section(ui);
                    self.show_selection_section(ui);
                    self.show_camera_section(ui);
                });
            });

        // ── Viewport ─────────────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            let w = rect.width() as f64;
            let h = rect.height() as f64;
            if w < 10.0 || h < 10.0 {
                return;
            }

            // Background
            painter.rect_filled(rect, 0.0, BG);

            // Camera input
            if response.dragged_by(egui::PointerButton::Primary) {
                let d = response.drag_delta();
                self.camera.orbit(-d.x as f64 * 0.01, d.y as f64 * 0.01);
            }
            if response.dragged_by(egui::PointerButton::Secondary) {
                let d = response.drag_delta();
                let s = self.camera.distance * 0.0015;
                self.camera.pan(d.x as f64 * s, d.y as f64 * s);
            }
            if response.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(2.0_f64.powf(-scroll as f64 * 0.01));
                }
            }
            if ctx.input(|i| i.key_pressed(egui::Key::R)) {
                self.camera.reset();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.selection = SelectionState::None;
            }

            // Selection on click
            if response.clicked() {
                if let Some(pos) = response.interact_pointer_pos() {
                    let mx = pos.x as f64 - rect.min.x as f64;
                    let my = pos.y as f64 - rect.min.y as f64;
                    self.selection = self.pick([mx, my], [w, h]);
                }
            }

            let Some(scene) = &self.scene else {
                painter.text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "No model loaded",
                    egui::FontId::proportional(16.0),
                    LABEL_C,
                );
                return;
            };

            // ── Draw grid ──
            if self.display.show_grid {
                let gmin = -2.0;
                let gmax = 8.0;
                let step = 1.0;
                let mut i = gmin;
                while i <= gmax {
                    if let (Some(a), Some(b)) = (
                        self.camera.project([gmin, 0.0, i], w, h),
                        self.camera.project([gmax, 0.0, i], w, h),
                    ) {
                        painter.line_segment(
                            [scr(&a, rect), scr(&b, rect)],
                            egui::Stroke::new(1.0_f32, GRID_C),
                        );
                    }
                    if let (Some(a), Some(b)) = (
                        self.camera.project([i, 0.0, gmin], w, h),
                        self.camera.project([i, 0.0, gmax], w, h),
                    ) {
                        painter.line_segment(
                            [scr(&a, rect), scr(&b, rect)],
                            egui::Stroke::new(1.0_f32, GRID_C),
                        );
                    }
                    i += step;
                }
            }

            // ── Draw global axes ──
            if self.display.show_global_axes {
                draw_axes(&self.camera, w, h, rect, &painter, 1.0);
            }

            // ── Depth-sort members ──
            let mut order: Vec<usize> = (0..scene.n_members()).collect();
            order.sort_by(|&a, &b| {
                let da = scene
                    .member_midpoint(a)
                    .and_then(|m| self.camera.project(m, w, h))
                    .map(|p| p.depth)
                    .unwrap_or(f64::INFINITY);
                let db = scene
                    .member_midpoint(b)
                    .and_then(|m| self.camera.project(m, w, h))
                    .map(|p| p.depth)
                    .unwrap_or(f64::INFINITY);
                db.partial_cmp(&da).unwrap_or(std::cmp::Ordering::Equal)
            });

            // ── Draw undeformed members ──
            if self.display.show_undeformed {
                for &mi in &order {
                    let (ni, nj) = scene.member_nodes[mi];
                    let pi = scene.node_positions[ni];
                    let pj = scene.node_positions[nj];
                    let Some(pa) = self.camera.project(pi, w, h) else {
                        continue;
                    };
                    let Some(pb) = self.camera.project(pj, w, h) else {
                        continue;
                    };
                    let is_sel = self.selection == SelectionState::Member(mi);
                    let (wd, col) = if is_sel {
                        (4.0, SELECTED_C)
                    } else {
                        (2.5, MEMBER_C)
                    };
                    painter.line_segment(
                        [scr(&pa, rect), scr(&pb, rect)],
                        egui::Stroke::new(wd as f32, col),
                    );
                }
            }

            // ── Draw undeformed nodes ──
            if self.display.show_undeformed {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    let Some(p) = self.camera.project(pos, w, h) else {
                        continue;
                    };
                    let is_sel = self.selection == SelectionState::Node(i);
                    let r = if is_sel { 7.0 } else { 5.0 };
                    let col = if is_sel { SELECTED_C } else { NODE_C };
                    painter.circle_filled(scr(&p, rect), r, col);
                }
            }

            // ── Draw deformed shape ──
            if self.display.show_deformed {
                if let Some(ref result) = self.result {
                    let deformed = scene
                        .deformed_positions(&result.displacements, self.display.deformation_scale);
                    for &mi in &order {
                        let (ni, nj) = scene.member_nodes[mi];
                        let pi = deformed[ni];
                        let pj = deformed[nj];
                        let Some(pa) = self.camera.project(pi, w, h) else {
                            continue;
                        };
                        let Some(pb) = self.camera.project(pj, w, h) else {
                            continue;
                        };
                        painter.line_segment(
                            [scr(&pa, rect), scr(&pb, rect)],
                            egui::Stroke::new(1.5_f32, DEF_MEMBER),
                        );
                    }
                    for (_, &pos) in deformed.iter().enumerate() {
                        let Some(p) = self.camera.project(pos, w, h) else {
                            continue;
                        };
                        painter.circle_filled(scr(&p, rect), 3.5, DEF_NODE);
                    }
                }
            }

            // ── Draw local axes for selected member ──
            if let (SelectionState::Member(mi), Some(result)) = (self.selection, &self.result) {
                if self.display.show_local_axes {
                    if let Ok((xh, yh, zh)) = result.member_local_axes(mi) {
                        if let Some(mid) = scene.member_midpoint(mi) {
                            draw_local_axes(
                                &self.camera,
                                w,
                                h,
                                rect,
                                &painter,
                                mid,
                                [xh, yh, zh],
                                0.5,
                            );
                        }
                    }
                }
            }

            // ── Node labels ──
            if self.display.show_node_labels {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    let Some(p) = self.camera.project(pos, w, h) else {
                        continue;
                    };
                    painter.text(
                        scr(&p, rect) + egui::vec2(8.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        format!("N{i}"),
                        egui::FontId::proportional(10.0),
                        LABEL_C,
                    );
                }
            }

            // ── Help text ──
            painter.text(
                rect.min + egui::vec2(10.0, rect.height() - 10.0),
                egui::Align2::LEFT_BOTTOM,
                "Drag: orbit | R-drag: pan | Scroll: zoom | R: reset | Click: select | Esc: clear",
                egui::FontId::proportional(11.0),
                HELP_C,
            );
        });
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────
fn scr(p: &ScreenPos, rect: egui::Rect) -> egui::Pos2 {
    egui::pos2(rect.min.x + p.x as f32, rect.min.y + p.y as f32)
}

fn draw_axes(cam: &Camera3D, w: f64, h: f64, rect: egui::Rect, painter: &egui::Painter, len: f64) {
    let origin = [0.0, 0.0, 0.0];
    let Some(p0) = cam.project(origin, w, h) else {
        return;
    };
    for (end, col, lbl) in [
        ([len, 0.0, 0.0], AXIS_X, "X"),
        ([0.0, len, 0.0], AXIS_Y, "Y"),
        ([0.0, 0.0, len], AXIS_Z, "Z"),
    ] {
        if let Some(p1) = cam.project(end, w, h) {
            painter.line_segment(
                [scr(&p0, rect), scr(&p1, rect)],
                egui::Stroke::new(2.5_f32, col),
            );
            painter.text(
                scr(&p1, rect),
                egui::Align2::CENTER_CENTER,
                lbl,
                egui::FontId::proportional(12.0),
                col,
            );
        }
    }
}

fn draw_local_axes(
    cam: &Camera3D,
    w: f64,
    h: f64,
    rect: egui::Rect,
    painter: &egui::Painter,
    origin: [f64; 3],
    axes: [[f64; 3]; 3],
    len: f64,
) {
    let Some(p0) = cam.project(origin, w, h) else {
        return;
    };
    for (axis, col, lbl) in [
        (axes[0], AXIS_X, "x"),
        (axes[1], AXIS_Y, "y"),
        (axes[2], AXIS_Z, "z"),
    ] {
        let end = [
            origin[0] + axis[0] * len,
            origin[1] + axis[1] * len,
            origin[2] + axis[2] * len,
        ];
        if let Some(p1) = cam.project(end, w, h) {
            painter.line_segment(
                [scr(&p0, rect), scr(&p1, rect)],
                egui::Stroke::new(2.0_f32, col),
            );
            painter.text(
                scr(&p1, rect),
                egui::Align2::CENTER_CENTER,
                lbl,
                egui::FontId::proportional(11.0),
                col,
            );
        }
    }
}

fn fmt(v: f64) -> String {
    if !v.is_finite() {
        return "NaN".into();
    }
    if v.abs() < 1e-12 {
        return "0".into();
    }
    format!("{:.4e}", v)
}

// ── Side panel sections ─────────────────────────────────────────────────
impl ViewportApp {
    fn show_model_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Model")
            .default_open(true)
            .show(ui, |ui| {
                ui.label("Portal frame (deterministic)");
                ui.label(format!("Nodes: {}", self.model.n_nodes()));
                ui.label(format!("Members: {}", self.model.n_members()));
                if self.result.is_some() {
                    ui.label("Status: ✓ Analyzed");
                } else {
                    ui.label("Status: ✗ Not analyzed");
                }
                if ui.button("Re-solve").clicked() {
                    self.result = self.model.solve().ok();
                    self.scene = self.result.as_ref().map(ViewportScene::from_result);
                    self.selection = SelectionState::None;
                }
            });
    }

    fn show_display_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Display")
            .default_open(true)
            .show(ui, |ui| {
                ui.checkbox(&mut self.display.show_undeformed, "Undeformed shape");
                ui.checkbox(&mut self.display.show_deformed, "Deformed shape");
                ui.checkbox(&mut self.display.show_local_axes, "Local axes (selected)");
                ui.checkbox(&mut self.display.show_grid, "Ground grid");
                ui.checkbox(&mut self.display.show_global_axes, "Global axes");
                ui.checkbox(&mut self.display.show_node_labels, "Node labels");
                ui.add(
                    egui::Slider::new(&mut self.display.deformation_scale, 0.0..=2000.0)
                        .text("Deform scale (× actual)")
                        .logarithmic(true),
                );
                ui.label(format!(
                    "Scale: {:.1}× actual displacement",
                    self.display.deformation_scale
                ));
            });
    }

    fn show_selection_section(&self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Selection")
            .default_open(true)
            .show(ui, |ui| match self.selection {
                SelectionState::None => {
                    ui.label("Nothing selected. Click a node or member.");
                }
                SelectionState::Node(idx) => {
                    ui.strong(format!("Node N{idx}"));
                    if let Some(s) = &self.scene {
                        if let Some(&p) = s.node_positions.get(idx) {
                            ui.label(format!(
                                "Position: ({:.3}, {:.3}, {:.3}) m",
                                p[0], p[1], p[2]
                            ));
                        }
                    }
                    if let Some(r) = &self.result {
                        let disp = Dof3D::ALL.map(|d| r.displacement(idx, d).unwrap_or(0.0));
                        ui.label("Displacement:");
                        ui.label(format!(
                            "  u = ({}, {}, {})",
                            fmt(disp[0]),
                            fmt(disp[1]),
                            fmt(disp[2])
                        ));
                        ui.label(format!(
                            "  θ = ({}, {}, {})",
                            fmt(disp[3]),
                            fmt(disp[4]),
                            fmt(disp[5])
                        ));
                        let reac = Dof3D::ALL.map(|d| r.reaction(idx, d).unwrap_or(0.0));
                        ui.label("Reaction:");
                        ui.label(format!(
                            "  R = ({}, {}, {})",
                            fmt(reac[0]),
                            fmt(reac[1]),
                            fmt(reac[2])
                        ));
                        ui.label(format!(
                            "  M = ({}, {}, {})",
                            fmt(reac[3]),
                            fmt(reac[4]),
                            fmt(reac[5])
                        ));
                    }
                }
                SelectionState::Member(idx) => {
                    ui.strong(format!("Member M{idx}"));
                    if let Some(s) = &self.scene {
                        if let Some(&(ni, nj)) = s.member_nodes.get(idx) {
                            ui.label(format!("Nodes: N{ni} → N{nj}"));
                        }
                        if let Some(len) = s.member_length(idx) {
                            ui.label(format!("Length: {:.4} m", len));
                        }
                    }
                    if let Some(r) = &self.result {
                        if let Ok(f) = r.member_end_forces(idx) {
                            ui.label("Local end forces:");
                            forces_grid(ui, "local", f);
                        }
                        if let Ok(f) = r.member_end_forces_global(idx) {
                            ui.label("Global end forces:");
                            forces_grid(ui, "global", &f);
                        }
                        if let Ok((x, y, z)) = r.member_local_axes(idx) {
                            ui.label("Local axes (in global coords):");
                            ui.label(format!("  x̂ = ({:.4}, {:.4}, {:.4})", x[0], x[1], x[2]));
                            ui.label(format!("  ŷ = ({:.4}, {:.4}, {:.4})", y[0], y[1], y[2]));
                            ui.label(format!("  ẑ = ({:.4}, {:.4}, {:.4})", z[0], z[1], z[2]));
                        }
                    }
                }
            });
    }

    fn show_camera_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Camera")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(format!(
                    "Target: ({:.2}, {:.2}, {:.2})",
                    self.camera.target[0], self.camera.target[1], self.camera.target[2]
                ));
                ui.label(format!("Distance: {:.2} m", self.camera.distance));
                ui.label(format!("Azimuth: {:.1}°", self.camera.azimuth.to_degrees()));
                ui.label(format!(
                    "Elevation: {:.1}°",
                    self.camera.elevation.to_degrees()
                ));
                ui.label(format!("FOV: {:.1}°", self.camera.fov.to_degrees()));
                if ui.button("Reset view").clicked() {
                    self.camera.reset();
                }
            });
    }
}

fn forces_grid(ui: &mut egui::Ui, id: &str, f: &[f64; 12]) {
    egui::Grid::new(format!("forces_{id}"))
        .num_columns(4)
        .spacing([8.0, 3.0])
        .show(ui, |ui| {
            ui.strong("End");
            ui.strong("Fx");
            ui.strong("Fy");
            ui.strong("Fz");
            ui.end_row();
            ui.label("i");
            ui.label(fmt(f[0]));
            ui.label(fmt(f[1]));
            ui.label(fmt(f[2]));
            ui.end_row();
            ui.label("j");
            ui.label(fmt(f[6]));
            ui.label(fmt(f[7]));
            ui.label(fmt(f[8]));
            ui.end_row();
            ui.strong("End");
            ui.strong("Mx");
            ui.strong("My");
            ui.strong("Mz");
            ui.end_row();
            ui.label("i");
            ui.label(fmt(f[3]));
            ui.label(fmt(f[4]));
            ui.label(fmt(f[5]));
            ui.end_row();
            ui.label("j");
            ui.label(fmt(f[9]));
            ui.label(fmt(f[10]));
            ui.label(fmt(f[11]));
            ui.end_row();
        });
}

// ── Entry point ─────────────────────────────────────────────────────────
fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("3D Viewport PoC — Phase 139"),
        ..Default::default()
    };
    eframe::run_native(
        "3D Viewport PoC",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(ViewportApp::new()))
        }),
    )
}
