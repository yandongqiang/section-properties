//! Desktop application — egui + eframe.
//!
//! Run with: `cargo run -p viewport-poc`

use eframe;
use egui;
use structural_analysis::{Dof3D, FrameAnalysisResult3D};
use viewport_poc::{
    app_state::{AnalysisStatus, AppMessage, MessageLevel, Project},
    camera::{Camera3D, ScreenPos},
    display::{
        ResultDisplayState, ResultViewMode, displacement_magnitudes, fmt_eng, magnitude_color,
        max_abs, normalize,
    },
    edit_model::{EditModel, MemberId, NodalLoadSpec, NodeId, SupportSpec},
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
const STALE_C: egui::Color32 = egui::Color32::from_rgb(255, 200, 0);
const ERROR_C: egui::Color32 = egui::Color32::from_rgb(255, 80, 80);
const SUPPORT_C: egui::Color32 = egui::Color32::from_rgb(100, 255, 200);
const LOAD_F_C: egui::Color32 = egui::Color32::from_rgb(255, 180, 80);
const LOAD_M_C: egui::Color32 = egui::Color32::from_rgb(200, 130, 255);

// ── App ─────────────────────────────────────────────────────────────────
struct ViewportApp {
    project: Project,
    scene: Option<ViewportScene>,
    camera: Camera3D,
    selection: SelectionState,
    display: ResultDisplayState,
    messages: Vec<AppMessage>,
    new_node_x: String,
    new_node_y: String,
    new_node_z: String,
    new_member_i: String,
    new_member_j: String,
    member_edit: MemberEditBuffer,
}

/// Temporary buffer for member property editing.
///
/// Holds section (E, G, A, Iy, Iz, J) and ref_vec (rx, ry, rz) values
/// being edited in the inspector.  Values are only applied to the
/// EditModel when the user clicks "Apply".
struct MemberEditBuffer {
    /// The MemberId this buffer corresponds to.
    member_id: Option<MemberId>,
    e: f64,
    g: f64,
    area: f64,
    iy: f64,
    iz: f64,
    j: f64,
    ref_x: f64,
    ref_y: f64,
    ref_z: f64,
}

impl Default for MemberEditBuffer {
    fn default() -> Self {
        Self {
            member_id: None,
            e: 200e9,
            g: 80e9,
            area: 1e-3,
            iy: 2e-6,
            iz: 1e-6,
            j: 3e-6,
            ref_x: 0.0,
            ref_y: 0.0,
            ref_z: 1.0,
        }
    }
}

impl ViewportApp {
    fn new() -> Self {
        Self {
            project: Project::empty(),
            scene: None,
            camera: Camera3D::default(),
            selection: SelectionState::None,
            display: ResultDisplayState::default(),
            messages: vec![AppMessage::info(
                "Application started. Select a model from the browser.",
            )],
            new_node_x: String::new(),
            new_node_y: String::new(),
            new_node_z: String::new(),
            new_member_i: String::new(),
            new_member_j: String::new(),
            member_edit: MemberEditBuffer::default(),
        }
    }

    /// Borrow the current analysis result, or `None` if stale/failed/none.
    ///
    /// Delegates to [`Project::analysis_result`] (Policy A).
    fn analysis_result(&self) -> Option<&FrameAnalysisResult3D> {
        self.project.analysis_result()
    }

    /// Load a preset model and immediately solve it.
    fn load_preset(&mut self, preset: model::PresetModel) {
        let name = preset.name().to_string();
        let m = preset.build_edit();
        self.project.load_model(name, m);
        self.messages
            .push(AppMessage::info(format!("Loaded '{}'", preset.name())));
        self.rebuild_scene();
        self.run_solve();
    }

    /// Rebuild the viewport scene from the current edit model.
    fn rebuild_scene(&mut self) {
        if let Some(ref model) = self.project.model {
            self.scene = Some(ViewportScene::from_edit_model(model));
            if let Some(ref s) = self.scene {
                let (min, max) = s.bounding_box();
                self.camera = Camera3D::fit(min, max);
            }
        } else {
            self.scene = None;
        }
    }

    /// Sync the member edit buffer when the selected member changes.
    fn sync_member_edit_buffer(&mut self, id: MemberId) {
        if self.member_edit.member_id == Some(id) {
            return;
        }
        self.reset_member_edit(id);
    }

    /// Reset the edit buffer to the current model values for a member.
    fn reset_member_edit(&mut self, id: MemberId) {
        if let Some(model) = &self.project.model {
            if let Some(member) = model.members().iter().find(|m| m.id == id) {
                let sec = &member.section;
                self.member_edit = MemberEditBuffer {
                    member_id: Some(id),
                    e: sec.E(),
                    g: sec.G(),
                    area: sec.area(),
                    iy: sec.iy(),
                    iz: sec.iz(),
                    j: sec.j(),
                    ref_x: member.ref_vec[0],
                    ref_y: member.ref_vec[1],
                    ref_z: member.ref_vec[2],
                };
            }
        }
    }

    /// Apply the edit buffer to the model for a member.
    fn apply_member_edit(&mut self, id: MemberId) {
        let buf = &self.member_edit;
        let (e, g, area, iy, iz, j, rv) = (
            buf.e,
            buf.g,
            buf.area,
            buf.iy,
            buf.iz,
            buf.j,
            [buf.ref_x, buf.ref_y, buf.ref_z],
        );
        if let Some(model) = self.project.model.as_mut() {
            match model.set_member_section(id, e, g, area, iy, iz, j) {
                Ok(()) => {}
                Err(e) => {
                    self.messages.push(AppMessage::error(e.message()));
                    return;
                }
            }
            match model.set_member_ref_vec(id, rv) {
                Ok(()) => {
                    self.rebuild_scene();
                    self.messages
                        .push(AppMessage::info(format!("Applied changes to M{}", id.0)));
                }
                Err(e) => {
                    self.messages.push(AppMessage::error(e.message()));
                    self.reset_member_edit(id);
                }
            }
        }
    }

    /// Run the FEM analysis and rebuild the viewport scene.
    ///
    /// This is the **only** place the scene is rebuilt from analysis data.
    /// Camera, selection, and display-option changes never call this.
    fn run_solve(&mut self) {
        match self.project.solve() {
            Ok(()) => {
                self.messages.push(AppMessage::info("Analysis complete."));
            }
            Err(e) => {
                self.messages
                    .push(AppMessage::error(format!("Analysis failed: {e}")));
            }
        }
    }

    /// Clear the project to the empty state.
    fn clear_project(&mut self) {
        self.project.clear();
        self.scene = None;
        self.selection = SelectionState::None;
        self.messages.push(AppMessage::info("Project cleared."));
    }

    /// Clear selection if it points to a node/member that no longer exists.
    fn validate_selection(&mut self) {
        if let Some(ref scene) = self.scene {
            match self.selection {
                SelectionState::Node(id) if scene.node_index(id).is_none() => {
                    self.selection = SelectionState::None;
                }
                SelectionState::Member(id) if scene.member_index(id).is_none() => {
                    self.selection = SelectionState::None;
                }
                _ => {}
            }
        } else {
            self.selection = SelectionState::None;
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
                    SelectionState::Node(scene.node_ids[ni])
                } else {
                    SelectionState::Member(scene.member_ids[mi])
                }
            }
            (Some((ni, _)), None) => SelectionState::Node(scene.node_ids[ni]),
            (None, Some((mi, _))) => SelectionState::Member(scene.member_ids[mi]),
            (None, None) => SelectionState::None,
        }
    }
}

impl eframe::App for ViewportApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── Top panel: toolbar ──────────────────────────────────────────
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New (empty model)").clicked() {
                        self.project.load_model("Untitled".into(), EditModel::new());
                        self.rebuild_scene();
                        self.selection = SelectionState::None;
                        self.messages.push(AppMessage::info("Created empty model."));
                        ui.close_menu();
                    }
                    if ui.button("Clear project").clicked() {
                        self.clear_project();
                        ui.close_menu();
                    }
                });
                ui.separator();
                let solve_enabled = !self.project.is_empty();
                if ui
                    .add_enabled(solve_enabled, egui::Button::new("Solve"))
                    .clicked()
                {
                    self.run_solve();
                }
                if ui.button("Clear selection").clicked() {
                    self.selection = SelectionState::None;
                }
                ui.separator();
                match self.project.analysis_status() {
                    AnalysisStatus::Stale => {
                        ui.label(
                            egui::RichText::new(
                                "\u{26a0} Results are stale \u{2014} re-solve to update",
                            )
                            .color(STALE_C),
                        );
                    }
                    AnalysisStatus::Failed => {
                        ui.label(
                            egui::RichText::new(
                                "\u{26a0} Last solve failed \u{2014} fix and re-solve",
                            )
                            .color(ERROR_C),
                        );
                    }
                    _ => {}
                }
            });
        });

        // ── Bottom panel: status / diagnostics ─────────────────────────
        egui::TopBottomPanel::bottom("status")
            .exact_height(28.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if self.project.is_empty() {
                        ui.label("No model loaded");
                    } else {
                        ui.label(format!("Model: {}", self.project.name));
                        if let Some(ref m) = self.project.model {
                            ui.label(format!("Nodes: {}", m.n_nodes()));
                            ui.label(format!("Members: {}", m.n_members()));
                        }
                        match self.project.analysis_status() {
                            AnalysisStatus::Current => {
                                ui.label("Analysis: \u{2713}");
                            }
                            AnalysisStatus::Stale => {
                                ui.label(egui::RichText::new("Analysis: STALE").color(STALE_C));
                            }
                            AnalysisStatus::Failed => {
                                ui.label(egui::RichText::new("Analysis: FAILED").color(ERROR_C));
                            }
                            AnalysisStatus::NeverSolved => {
                                ui.label("Analysis: not run");
                            }
                        }
                    }
                    ui.separator();
                    if let Some(last) = self.messages.last() {
                        let color = match last.level {
                            MessageLevel::Info => LABEL_C,
                            MessageLevel::Warning => STALE_C,
                            MessageLevel::Error => ERROR_C,
                        };
                        ui.label(egui::RichText::new(&last.text).color(color));
                    }
                });
            });

        // ── Left panel: model browser ──────────────────────────────────
        egui::SidePanel::left("browser")
            .default_width(180.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.heading("Models");
                ui.separator();
                for &preset in model::PresetModel::all() {
                    let is_active = self.project.name == preset.name();
                    if ui
                        .add_enabled(!is_active, egui::Button::new(preset.name()))
                        .clicked()
                    {
                        self.load_preset(preset);
                    }
                }
                ui.separator();
                if ui.button("New empty model").clicked() {
                    self.project.load_model("Untitled".into(), EditModel::new());
                    self.rebuild_scene();
                    self.selection = SelectionState::None;
                    self.messages.push(AppMessage::info("Created empty model."));
                }
                if ui.button("Clear project").clicked() {
                    self.clear_project();
                }
            });

        // ── Right panel: inspector ─────────────────────────────────────
        egui::SidePanel::right("inspector")
            .default_width(350.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.show_edit_section(ui);
                    self.show_model_contents_section(ui);
                    self.show_display_section(ui);
                    self.show_selection_section(ui);
                    self.show_camera_section(ui);
                });
            });

        // ── Central panel: 3D viewport ─────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            self.validate_selection();

            let (rect, response) =
                ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            let w = rect.width() as f64;
            let h = rect.height() as f64;
            if w < 10.0 || h < 10.0 {
                return;
            }

            painter.rect_filled(rect, 0.0, BG);

            // Camera input — does NOT trigger solve.
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

            // Selection on click — does NOT trigger solve.
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
                painter.text(
                    rect.min + egui::vec2(10.0, rect.height() - 10.0),
                    egui::Align2::LEFT_BOTTOM,
                    "Drag: orbit | R-drag: pan | Scroll: zoom | R: reset | Click: select | Esc: clear",
                    egui::FontId::proportional(11.0),
                    HELP_C,
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
            let order: Vec<usize> = (0..scene.n_members()).collect();

            // Determine whether to show base (undeformed) geometry.
            let show_base = self.display.mode != ResultViewMode::Deformed
                || self.display.show_undeformed;

            // ── Draw undeformed members ──
            if show_base {
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
                    let is_sel = self.selection == SelectionState::Member(scene.member_ids[mi]);
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
            if show_base {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    let Some(p) = self.camera.project(pos, w, h) else {
                        continue;
                    };
                    let is_sel = self.selection == SelectionState::Node(scene.node_ids[i]);
                    let r = if is_sel { 7.0 } else { 5.0 };
                    let col = if is_sel { SELECTED_C } else { NODE_C };
                    painter.circle_filled(scr(&p, rect), r, col);
                }
            }

            // ── Draw support symbols ──
            if show_base {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    let Some(spec) = scene.node_supports.get(i).copied().flatten() else {
                        continue;
                    };
                    let Some(p) = self.camera.project(pos, w, h) else {
                        continue;
                    };
                    let center = scr(&p, rect);
                    painter.circle_stroke(
                        center,
                        10.0,
                        egui::Stroke::new(2.0_f32, SUPPORT_C),
                    );
                    let label = if spec.is_fixed() {
                        "F".to_string()
                    } else if spec.is_pinned() {
                        "P".to_string()
                    } else {
                        format!("C{}", spec.n_constrained())
                    };
                    painter.text(
                        center,
                        egui::Align2::CENTER_CENTER,
                        label,
                        egui::FontId::proportional(10.0),
                        SUPPORT_C,
                    );
                }
            }

            // ── Draw load indicators ──
            if show_base {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    let Some(load) = scene.node_loads.get(i).copied().flatten() else {
                        continue;
                    };
                    if load.is_zero() {
                        continue;
                    }
                    let Some(p) = self.camera.project(pos, w, h) else {
                        continue;
                    };
                    let center = scr(&p, rect);

                    let fmag = load.force_magnitude();
                    if fmag > 1e-12 {
                        let dir = [
                            load.fx / fmag,
                            load.fy / fmag,
                            load.fz / fmag,
                        ];
                        let tip_world = [
                            pos[0] + dir[0] * 0.5,
                            pos[1] + dir[1] * 0.5,
                            pos[2] + dir[2] * 0.5,
                        ];
                        if let Some(p_tip) = self.camera.project(tip_world, w, h) {
                            let tip = scr(&p_tip, rect);
                            painter.line_segment(
                                [center, tip],
                                egui::Stroke::new(2.5_f32, LOAD_F_C),
                            );
                            painter.circle_filled(tip, 4.0, LOAD_F_C);
                        }
                    }

                    let mmag = load.moment_magnitude();
                    if mmag > 1e-12 {
                        painter.circle_stroke(
                            center + egui::vec2(14.0, -14.0),
                            6.0,
                            egui::Stroke::new(2.0_f32, LOAD_M_C),
                        );
                        painter.text(
                            center + egui::vec2(14.0, -14.0),
                            egui::Align2::CENTER_CENTER,
                            "M",
                            egui::FontId::proportional(8.0),
                            LOAD_M_C,
                        );
                    }
                }
            }

            // ── Draw deformed shape ──
            if self.display.mode == ResultViewMode::Deformed {
                if let Some(result) = self.analysis_result() {
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

            // ── Draw displacement color-coding ──
            if self.display.mode == ResultViewMode::Displacement {
                if let Some(result) = self.analysis_result() {
                    let mags = displacement_magnitudes(
                        &result.displacements,
                        scene.n_nodes(),
                    );
                    let max_mag = max_abs(&mags);
                    for (i, &mag) in mags.iter().enumerate() {
                        let &pos = &scene.node_positions[i];
                        let Some(p) = self.camera.project(pos, w, h) else {
                            continue;
                        };
                        let t = normalize(mag, max_mag);
                        let [r, g, b] = magnitude_color(t);
                        let col = egui::Color32::from_rgb(r, g, b);
                        let radius = (5.0 + t * 4.0) as f32;
                        painter.circle_filled(scr(&p, rect), radius, col);
                    }
                    // Legend
                    if max_mag > 0.0 {
                        let ly = rect.min.y + 20.0;
                        let lx = rect.max.x - 180.0;
                        painter.text(
                            egui::pos2(lx, ly),
                            egui::Align2::LEFT_TOP,
                            format!("|u| max = {}", fmt_eng(max_mag)),
                            egui::FontId::proportional(11.0),
                            LABEL_C,
                        );
                        for j in 0..40 {
                            let t = j as f64 / 39.0;
                            let [r, g, b] = magnitude_color(t);
                            painter.line_segment(
                                [
                                    egui::pos2(lx + j as f32 * 3.0, ly + 20.0),
                                    egui::pos2(lx + j as f32 * 3.0, ly + 30.0),
                                ],
                                egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(r, g, b)),
                            );
                        }
                        painter.text(
                            egui::pos2(lx, ly + 32.0),
                            egui::Align2::LEFT_TOP,
                            "0",
                            egui::FontId::proportional(9.0),
                            LABEL_C,
                        );
                        painter.text(
                            egui::pos2(lx + 120.0, ly + 32.0),
                            egui::Align2::LEFT_TOP,
                            format!("{}", fmt_eng(max_mag)),
                            egui::FontId::proportional(9.0),
                            LABEL_C,
                        );
                    }
                }
            }

            // ── Draw reaction indicators ──
            if self.display.mode == ResultViewMode::Reaction {
                if let Some(result) = self.analysis_result() {
                    for (i, &pos) in scene.node_positions.iter().enumerate() {
                        let Some(_) = scene.node_supports.get(i).copied().flatten() else {
                            continue;
                        };
                        let fx = result.reactions.get(i * 6).copied().unwrap_or(0.0);
                        let fy = result.reactions.get(i * 6 + 1).copied().unwrap_or(0.0);
                        let fz = result.reactions.get(i * 6 + 2).copied().unwrap_or(0.0);
                        let mx = result.reactions.get(i * 6 + 3).copied().unwrap_or(0.0);
                        let my = result.reactions.get(i * 6 + 4).copied().unwrap_or(0.0);
                        let mz = result.reactions.get(i * 6 + 5).copied().unwrap_or(0.0);

                        let Some(p) = self.camera.project(pos, w, h) else {
                            continue;
                        };
                        let center = scr(&p, rect);

                        let fmag = (fx * fx + fy * fy + fz * fz).sqrt();
                        if fmag > 1e-9 {
                            let dir = [fx / fmag, fy / fmag, fz / fmag];
                            let scale = 0.3_f64.min(fmag * 1e-3);
                            let tip_world = [
                                pos[0] + dir[0] * scale,
                                pos[1] + dir[1] * scale,
                                pos[2] + dir[2] * scale,
                            ];
                            if let Some(p_tip) = self.camera.project(tip_world, w, h) {
                                let tip = scr(&p_tip, rect);
                                painter.line_segment(
                                    [center, tip],
                                    egui::Stroke::new(3.0_f32, SUPPORT_C),
                                );
                                painter.circle_filled(tip, 5.0, SUPPORT_C);
                            }
                        }

                        let mmag = (mx * mx + my * my + mz * mz).sqrt();
                        if mmag > 1e-9 {
                            painter.circle_stroke(
                                center + egui::vec2(14.0, -14.0),
                                7.0,
                                egui::Stroke::new(2.5_f32, LOAD_M_C),
                            );
                            painter.text(
                                center + egui::vec2(14.0, -14.0),
                                egui::Align2::CENTER_CENTER,
                                "R",
                                egui::FontId::proportional(8.0),
                                LOAD_M_C,
                            );
                        }
                    }
                }
            }

            // ── Draw member end-force indicators ──
            if self.display.mode == ResultViewMode::MemberEndForces {
                if let (SelectionState::Member(mid), Some(result)) =
                    (self.selection, self.analysis_result())
                {
                    if let Some(mi) = scene.member_index(mid) {
                        if let Ok(f_global) = result.member_end_forces_global(mi) {
                            let (ni, nj) = scene.member_nodes[mi];
                            let pi = scene.node_positions[ni];
                            let pj = scene.node_positions[nj];

                            for (end, f) in [(pi, &f_global[0..3]), (pj, &f_global[6..9])] {
                                let fx = f[0];
                                let fy = f[1];
                                let fz = f[2];
                                let fmag = (fx * fx + fy * fy + fz * fz).sqrt();
                                if fmag <= 1e-9 {
                                    continue;
                                }
                                let dir = [fx / fmag, fy / fmag, fz / fmag];
                                let scale = 0.3_f64.min(fmag * 1e-3);
                                let tip_world = [
                                    end[0] + dir[0] * scale,
                                    end[1] + dir[1] * scale,
                                    end[2] + dir[2] * scale,
                                ];
                                let Some(p0) = self.camera.project(end, w, h) else {
                                    continue;
                                };
                                let Some(p1) = self.camera.project(tip_world, w, h) else {
                                    continue;
                                };
                                let c0 = scr(&p0, rect);
                                let c1 = scr(&p1, rect);
                                painter.line_segment(
                                    [c0, c1],
                                    egui::Stroke::new(3.0_f32, SELECTED_C),
                                );
                                painter.circle_filled(c1, 5.0, SELECTED_C);
                            }
                        }
                    }
                }
            }

            // ── Draw local axes for selected member ──
            if let (SelectionState::Member(mid), Some(result)) =
                (self.selection, self.analysis_result())
            {
                if self.display.show_local_axes {
                    if let Some(mi) = scene.member_index(mid) {
                        if let Ok((xh, yh, zh)) = result.member_local_axes(mi) {
                            if let Some(mpos) = scene.member_midpoint(mi) {
                                draw_local_axes(
                                    &self.camera,
                                    w,
                                    h,
                                    rect,
                                    &painter,
                                    mpos,
                                    [xh, yh, zh],
                                    0.5,
                                );
                            }
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
                        format!("N{}", scene.node_ids[i].0),
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
    fn show_display_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Display")
            .default_open(true)
            .show(ui, |ui| {
                ui.label("Result view mode:");
                ui.horizontal_wrapped(|ui| {
                    for mode in ResultViewMode::ALL {
                        let selected = self.display.mode == mode;
                        if ui
                            .add(egui::SelectableLabel::new(selected, mode.label()))
                            .clicked()
                        {
                            self.display.mode = mode;
                        }
                    }
                });

                let result_available = self.analysis_result().is_some();
                if self.display.mode.requires_result() && !result_available {
                    ui.label(
                        egui::RichText::new("(no current result — solve first)").color(STALE_C),
                    );
                }

                ui.separator();

                if self.display.mode == ResultViewMode::Deformed {
                    ui.checkbox(&mut self.display.show_undeformed, "Undeformed reference");
                    ui.add(
                        egui::Slider::new(&mut self.display.deformation_scale, 0.0..=2000.0)
                            .text("Deform scale")
                            .logarithmic(true),
                    );
                    ui.label(format!(
                        "Scale: {:.1}x actual displacement",
                        self.display.deformation_scale
                    ));
                    if ui.button("Reset scale").clicked() {
                        self.display.deformation_scale = 100.0;
                    }
                    ui.separator();
                }

                ui.checkbox(&mut self.display.show_local_axes, "Local axes (selected)");
                ui.checkbox(&mut self.display.show_grid, "Ground grid");
                ui.checkbox(&mut self.display.show_global_axes, "Global axes");
                ui.checkbox(&mut self.display.show_node_labels, "Node labels");
            });
    }

    fn show_selection_section(&mut self, ui: &mut egui::Ui) {
        let sel = self.selection;
        egui::CollapsingHeader::new("Selection")
            .default_open(true)
            .show(ui, |ui| match sel {
                SelectionState::None => {
                    ui.label("Nothing selected. Click a node or member.");
                }
                SelectionState::Node(id) => {
                    ui.strong(format!("Node N{}", id.0));

                    let mut changed = false;
                    let mut new_coords = (0.0, 0.0, 0.0);
                    if let Some(model) = self.project.model.as_ref() {
                        if let Some(node) = model.node(id) {
                            let mut x = node.x;
                            let mut y = node.y;
                            let mut z = node.z;
                            ui.horizontal(|ui| {
                                ui.label("x:");
                                ui.add(egui::DragValue::new(&mut x).speed(0.1).range(-1e6..=1e6));
                                ui.label("y:");
                                ui.add(egui::DragValue::new(&mut y).speed(0.1).range(-1e6..=1e6));
                                ui.label("z:");
                                ui.add(egui::DragValue::new(&mut z).speed(0.1).range(-1e6..=1e6));
                            });
                            if (x, y, z) != (node.x, node.y, node.z) {
                                changed = true;
                                new_coords = (x, y, z);
                            }
                        }
                    }
                    if changed {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.update_node(id, new_coords.0, new_coords.1, new_coords.2) {
                                Ok(()) => {
                                    self.rebuild_scene();
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    }

                    if let Some(s) = &self.scene {
                        if let Some(idx) = s.node_index(id) {
                            if let Some(&p) = s.node_positions.get(idx) {
                                ui.label(format!(
                                    "Position: ({:.3}, {:.3}, {:.3}) m",
                                    p[0], p[1], p[2]
                                ));
                            }
                            if let Some(r) = self.analysis_result() {
                                let disp =
                                    Dof3D::ALL.map(|d| r.displacement(idx, d).unwrap_or(0.0));
                                ui.label("Displacement:");
                                ui.label(format!(
                                    "  u = ({}, {}, {})",
                                    fmt(disp[0]),
                                    fmt(disp[1]),
                                    fmt(disp[2])
                                ));
                                ui.label(format!(
                                    "  \u{03b8} = ({}, {}, {})",
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
                    }

                    // ── Support editing ──
                    ui.separator();
                    ui.label("Support:");
                    let mut support_changed = false;
                    let mut new_spec = SupportSpec::free();
                    let mut has_support = false;
                    let mut clear_clicked = false;
                    if let Some(model) = self.project.model.as_ref() {
                        if let Some(spec) = model.support(id) {
                            new_spec = *spec;
                            has_support = true;
                        }
                    }
                    ui.label(format!(
                        "Current: {}",
                        if has_support {
                            new_spec.label()
                        } else {
                            "Free (no support)"
                        }
                    ));
                    ui.horizontal(|ui| {
                        if ui.button("Fixed").clicked() {
                            new_spec = SupportSpec::fixed();
                            support_changed = true;
                        }
                        if ui.button("Pinned").clicked() {
                            new_spec = SupportSpec::pinned();
                            support_changed = true;
                        }
                        if ui.button("Custom").clicked() {
                            if !has_support {
                                new_spec = SupportSpec::free();
                            }
                            support_changed = true;
                        }
                        if has_support && ui.button("Clear").clicked() {
                            clear_clicked = true;
                        }
                    });

                    if has_support || support_changed {
                        ui.label("DOFs (checked = constrained to 0):");
                        let mut dof_changed = false;
                        for dof in Dof3D::ALL {
                            let mut constrained = new_spec.dof(dof).is_some();
                            if ui.checkbox(&mut constrained, dof.name()).changed() {
                                new_spec.set_dof(dof, if constrained { Some(0.0) } else { None });
                                dof_changed = true;
                            }
                        }
                        if dof_changed {
                            support_changed = true;
                        }
                    }

                    if clear_clicked {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.clear_support(id) {
                                Ok(()) => {
                                    self.rebuild_scene();
                                    self.messages.push(AppMessage::info(format!(
                                        "Cleared support at node N{}",
                                        id.0
                                    )));
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    } else if support_changed {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.set_support(id, new_spec) {
                                Ok(()) => {
                                    self.rebuild_scene();
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    }

                    // ── Nodal load editing ──
                    ui.separator();
                    ui.label("Nodal load (global coords, N and N\u{00b7}m):");
                    let mut load_changed = false;
                    let mut new_load = NodalLoadSpec::zero();
                    let mut has_load = false;
                    let mut clear_load_clicked = false;
                    if let Some(model) = self.project.model.as_ref() {
                        if let Some(spec) = model.nodal_load(id) {
                            new_load = *spec;
                            has_load = true;
                        }
                    }
                    ui.label(format!(
                        "Current: {}",
                        if has_load {
                            if new_load.is_zero() {
                                "zero load"
                            } else {
                                "nonzero load"
                            }
                        } else {
                            "no load"
                        }
                    ));
                    egui::Grid::new("nodal_load_grid")
                        .num_columns(2)
                        .spacing([8.0, 3.0])
                        .show(ui, |ui| {
                            ui.strong("Force (N)");
                            ui.strong("Moment (N\u{00b7}m)");
                            ui.end_row();
                            ui.label("x:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.fx)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.label("x:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.mx)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.end_row();
                            ui.label("y:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.fy)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.label("y:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.my)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.end_row();
                            ui.label("z:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.fz)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.label("z:");
                            let r = ui.add(
                                egui::DragValue::new(&mut new_load.mz)
                                    .speed(100.0)
                                    .range(-1e12..=1e12),
                            );
                            if r.changed() {
                                load_changed = true;
                            }
                            ui.end_row();
                        });
                    ui.horizontal(|ui| {
                        if has_load && ui.button("Clear load").clicked() {
                            clear_load_clicked = true;
                        }
                    });

                    if clear_load_clicked {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.clear_nodal_load(id) {
                                Ok(()) => {
                                    self.rebuild_scene();
                                    self.messages.push(AppMessage::info(format!(
                                        "Cleared load at node N{}",
                                        id.0
                                    )));
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    } else if load_changed {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.set_nodal_load(id, new_load) {
                                Ok(()) => {
                                    self.rebuild_scene();
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    }

                    if ui.button("Delete node").clicked() {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.delete_node(id) {
                                Ok(()) => {
                                    self.selection = SelectionState::None;
                                    self.rebuild_scene();
                                    self.messages
                                        .push(AppMessage::info(format!("Deleted node N{}", id.0)));
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    }
                }
                SelectionState::Member(id) => {
                    ui.strong(format!("Member M{}", id.0));
                    if let Some(s) = &self.scene {
                        if let Some(idx) = s.member_index(id) {
                            if let Some(&(ni, nj)) = s.member_nodes.get(idx) {
                                ui.label(format!(
                                    "Nodes: N{} \u{2192} N{}",
                                    s.node_ids[ni].0, s.node_ids[nj].0
                                ));
                            }
                            if let Some(len) = s.member_length(idx) {
                                ui.label(format!("Length: {:.4} m", len));
                            }
                        }
                    }

                    // ── Section / Material editing ──
                    self.sync_member_edit_buffer(id);
                    ui.collapsing("Section / Material", |ui| {
                        egui::Grid::new("member_section_grid")
                            .num_columns(2)
                            .spacing([10.0, 4.0])
                            .show(ui, |ui| {
                                ui.label("E (Pa):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.e)
                                        .speed(1e8)
                                        .range(1.0..=1e12),
                                );
                                ui.end_row();
                                ui.label("G (Pa):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.g)
                                        .speed(1e8)
                                        .range(1.0..=1e12),
                                );
                                ui.end_row();
                                ui.label("A (m\u{00b2}):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.area)
                                        .speed(1e-5)
                                        .range(1e-12..=1.0),
                                );
                                ui.end_row();
                                ui.label("Iy (m\u{2074}):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.iy)
                                        .speed(1e-8)
                                        .range(1e-15..=1.0),
                                );
                                ui.end_row();
                                ui.label("Iz (m\u{2074}):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.iz)
                                        .speed(1e-8)
                                        .range(1e-15..=1.0),
                                );
                                ui.end_row();
                                ui.label("J (m\u{2074}):");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.j)
                                        .speed(1e-8)
                                        .range(1e-15..=1.0),
                                );
                                ui.end_row();
                            });
                    });

                    // ── Reference vector editing ──
                    ui.collapsing("Reference Vector", |ui| {
                        egui::Grid::new("member_refvec_grid")
                            .num_columns(2)
                            .spacing([10.0, 4.0])
                            .show(ui, |ui| {
                                ui.label("ref_x:");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.ref_x)
                                        .speed(0.1)
                                        .range(-10.0..=10.0),
                                );
                                ui.end_row();
                                ui.label("ref_y:");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.ref_y)
                                        .speed(0.1)
                                        .range(-10.0..=10.0),
                                );
                                ui.end_row();
                                ui.label("ref_z:");
                                ui.add(
                                    egui::DragValue::new(&mut self.member_edit.ref_z)
                                        .speed(0.1)
                                        .range(-10.0..=10.0),
                                );
                                ui.end_row();
                            });
                    });

                    ui.horizontal(|ui| {
                        if ui.button("Apply Changes").clicked() {
                            self.apply_member_edit(id);
                        }
                        if ui.button("Reset").clicked() {
                            self.reset_member_edit(id);
                        }
                    });

                    // ── Analysis results (read-only) ──
                    if let Some(s) = &self.scene {
                        if let Some(idx) = s.member_index(id) {
                            if let Some(r) = self.analysis_result() {
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
                                    ui.label(format!(
                                        "  x = ({:.4}, {:.4}, {:.4})",
                                        x[0], x[1], x[2]
                                    ));
                                    ui.label(format!(
                                        "  y = ({:.4}, {:.4}, {:.4})",
                                        y[0], y[1], y[2]
                                    ));
                                    ui.label(format!(
                                        "  z = ({:.4}, {:.4}, {:.4})",
                                        z[0], z[1], z[2]
                                    ));
                                }
                            }
                        }
                    }

                    if ui.button("Delete member").clicked() {
                        if let Some(model) = self.project.model.as_mut() {
                            match model.delete_member(id) {
                                Ok(()) => {
                                    self.selection = SelectionState::None;
                                    self.rebuild_scene();
                                    self.messages.push(AppMessage::info(format!(
                                        "Deleted member M{}",
                                        id.0
                                    )));
                                }
                                Err(e) => self.messages.push(AppMessage::error(e.message())),
                            }
                        }
                    }
                }
            });
    }

    fn show_edit_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Add")
            .default_open(true)
            .show(ui, |ui| {
                ui.label("Add node:");
                ui.horizontal(|ui| {
                    ui.label("x:");
                    ui.text_edit_singleline(&mut self.new_node_x);
                    ui.label("y:");
                    ui.text_edit_singleline(&mut self.new_node_y);
                    ui.label("z:");
                    ui.text_edit_singleline(&mut self.new_node_z);
                    if ui.button("Add node").clicked() {
                        self.add_node_from_input();
                    }
                });

                ui.separator();

                ui.label("Add member:");
                ui.horizontal(|ui| {
                    ui.label("N i:");
                    ui.text_edit_singleline(&mut self.new_member_i);
                    ui.label("N j:");
                    ui.text_edit_singleline(&mut self.new_member_j);
                    if ui.button("Add member").clicked() {
                        self.add_member_from_input();
                    }
                });
            });
    }

    fn show_model_contents_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Model contents")
            .default_open(false)
            .show(ui, |ui| {
                if let Some(model) = self.project.model.as_ref() {
                    ui.label(format!("Nodes ({}):", model.n_nodes()));
                    for node in model.nodes() {
                        let is_sel = self.selection == SelectionState::Node(node.id);
                        let sup_label = match model.support(node.id) {
                            Some(spec) => format!(" [{}]", spec.label()),
                            None => String::new(),
                        };
                        let load_label = match model.nodal_load(node.id) {
                            Some(spec) if !spec.is_zero() => " [L]".to_string(),
                            _ => String::new(),
                        };
                        if ui
                            .selectable_label(
                                is_sel,
                                format!(
                                    "N{} ({:.1}, {:.1}, {:.1}){}{}",
                                    node.id.0, node.x, node.y, node.z, sup_label, load_label
                                ),
                            )
                            .clicked()
                        {
                            self.selection = SelectionState::Node(node.id);
                        }
                    }
                    ui.separator();
                    ui.label(format!("Members ({}):", model.n_members()));
                    for member in model.members() {
                        let is_sel = self.selection == SelectionState::Member(member.id);
                        if ui
                            .selectable_label(
                                is_sel,
                                format!(
                                    "M{}: N{} \u{2192} N{}",
                                    member.id.0, member.node_i.0, member.node_j.0
                                ),
                            )
                            .clicked()
                        {
                            self.selection = SelectionState::Member(member.id);
                        }
                    }
                }
            });
    }

    fn add_node_from_input(&mut self) {
        let x: f64 = match self.new_node_x.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid x coordinate"));
                return;
            }
        };
        let y: f64 = match self.new_node_y.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid y coordinate"));
                return;
            }
        };
        let z: f64 = match self.new_node_z.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid z coordinate"));
                return;
            }
        };
        let Some(model) = self.project.model.as_mut() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return;
        };
        match model.add_node(x, y, z) {
            Ok(id) => {
                self.rebuild_scene();
                self.messages.push(AppMessage::info(format!(
                    "Added node N{} at ({}, {}, {})",
                    id.0, x, y, z
                )));
                self.new_node_x.clear();
                self.new_node_y.clear();
                self.new_node_z.clear();
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
    }

    fn add_member_from_input(&mut self) {
        let ni: u32 = match self.new_member_i.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages.push(AppMessage::error("Invalid node i ID"));
                return;
            }
        };
        let nj: u32 = match self.new_member_j.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages.push(AppMessage::error("Invalid node j ID"));
                return;
            }
        };
        let Some(model) = self.project.model.as_mut() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return;
        };
        match model.add_member(NodeId(ni), NodeId(nj)) {
            Ok(id) => {
                self.rebuild_scene();
                self.messages.push(AppMessage::info(format!(
                    "Added member M{} (N{} \u{2192} N{})",
                    id.0, ni, nj
                )));
                self.new_member_i.clear();
                self.new_member_j.clear();
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
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
                ui.label(format!(
                    "Azimuth: {:.1}\u{00b0}",
                    self.camera.azimuth.to_degrees()
                ));
                ui.label(format!(
                    "Elevation: {:.1}\u{00b0}",
                    self.camera.elevation.to_degrees()
                ));
                ui.label(format!("FOV: {:.1}\u{00b0}", self.camera.fov.to_degrees()));
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
            .with_title("Structural Analysis \u{2014} Phase 148"),
        ..Default::default()
    };
    eframe::run_native(
        "structural-analysis-gui",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(ViewportApp::new()))
        }),
    )
}
