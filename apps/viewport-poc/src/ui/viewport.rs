//! Central 3D viewport canvas: geometry, loads, supports, and result
//! overlays.
//!
//! This is a straight move of the former `update()` central-panel body —
//! the drawing order and colors are unchanged.

use egui;
use eframe::egui_glow;

use viewport_poc::app_state::AppMessage;
use viewport_poc::camera::{Camera3D, ScreenPos, StandardView};
use viewport_poc::display::{
    displacement_magnitudes, fmt_eng, magnitude_color, max_abs, normalize, ResultViewMode,
};
use viewport_poc::project;
use viewport_poc::selection::SelectionState;

use super::gl_renderer::{self, RenderMode};
use super::module::WorkModule;
use super::theme::{
    AXIS_X, AXIS_Y, AXIS_Z, BG, DEF_MEMBER, DEF_NODE, GRID_C, HELP_C, LABEL_C, LOAD_F_C, LOAD_M_C,
    MEMBER_C, MEMBER_LOAD_C, NODE_C, SELECTED_C, SUPPORT_C,
};
use crate::ViewportApp;

/// Samples per member for the deformed-shape curve.
const DEFORM_SAMPLES: usize = 12;

/// Side length of the view-cube gizmo (points).
const VIEW_CUBE_SIZE: f32 = 90.0;
/// Margin between the view cube and the viewport corner (points).
const VIEW_CUBE_MARGIN: f32 = 12.0;

/// Draw the central viewport panel.
pub fn show_viewport(app: &mut ViewportApp, ctx: &egui::Context) {
    // Surface any one-shot GPU-renderer message (e.g. a runtime fallback).
    if let Some(msg) = gl_renderer::take_message() {
        app.messages.push(AppMessage::warning(msg));
    }

    egui::CentralPanel::default().show(ctx, |ui| {
        app.validate_selection();

        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        let w = rect.width() as f64;
        let h = rect.height() as f64;
        if w < 10.0 || h < 10.0 {
            return;
        }

        painter.rect_filled(rect, 0.0, BG);

        // The view cube lives in the bottom-right corner; its rect is fixed for
        // the whole frame, so viewport clicks inside it are not treated as
        // selections.
        let gizmo_rect = view_cube_rect(rect);

        // Camera input — does NOT trigger solve.  A primary drag that starts on
        // the view cube is left to the gizmo (which orbits itself), so the two
        // handlers never double up.
        if response.dragged_by(egui::PointerButton::Primary) {
            let over_gizmo = response
                .interact_pointer_pos()
                .is_some_and(|p| gizmo_rect.contains(p));
            if !over_gizmo {
                let d = response.drag_delta();
                app.camera.orbit(-d.x as f64 * 0.01, d.y as f64 * 0.01);
            }
        }
        if response.dragged_by(egui::PointerButton::Secondary) {
            let d = response.drag_delta();
            let s = app.camera.distance * 0.0015;
            app.camera.pan(d.x as f64 * s, d.y as f64 * s);
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                app.camera.zoom(2.0_f64.powf(-scroll as f64 * 0.01));
            }
        }
        if ctx.input(|i| i.key_pressed(egui::Key::R)) {
            app.camera.reset();
        }
        // Escape closes the quick-edit popup first; only clear the selection
        // when nothing else is open.
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) && app.quick_edit.is_none() {
            app.selection = SelectionState::None;
        }

        // Selection on click — does NOT trigger solve.  Clicks on the view cube
        // are handled by the gizmo, not by picking.  A click in the viewport
        // also dismisses the quick-edit popup ("click away to close").
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if !gizmo_rect.contains(pos) {
                    app.selection = pick_at(app, pos, rect, w, h);
                    app.quick_edit = None;
                }
            }
        }

        // Double-click opens the quick-edit popup for the entity under the
        // cursor (Ctrl is not needed; a plain double-click).
        if response.double_clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if !gizmo_rect.contains(pos) {
                    let hit = pick_at(app, pos, rect, w, h);
                    app.open_quick_edit(hit, pos);
                }
            }
        }

        // Remember what was under the cursor when the context menu opens, so
        // its items stay stable while the menu is up, then draw the menu.
        if response.secondary_clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let hit = pick_at(app, pos, rect, w, h);
                app.ctx_menu_target = hit;
            }
        }
        let menu_target = app.ctx_menu_target;
        response.context_menu(|ui| {
            viewport_context_menu(app, ui, menu_target);
        });

        let Some(scene) = &app.scene else {
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
                "Drag: orbit | R-drag: pan | Scroll: zoom | R: reset | F: fit | 1-4: views | Click: select | Dbl-click: edit | R-click: menu | Esc: clear",
                egui::FontId::proportional(11.0),
                HELP_C,
            );
            show_view_cube(app, ui, rect, &painter);
            return;
        };

        // Determine whether to show base (undeformed) geometry.
        let show_base = app.display.mode != ResultViewMode::Deformed || app.display.show_undeformed;

        // Renderer selection (Phase P1): the GPU path draws grid / axes /
        // members / nodes into a depth-tested offscreen buffer and blits it
        // here; the software path keeps drawing them with the painter.
        let use_gpu = gl_renderer::mode() == RenderMode::Gpu;

        if use_gpu {
            let geom = gl_renderer::GlGeometry::build(
                &app.camera,
                scene,
                &app.display,
                app.selection,
                app.analysis_result(),
                show_base,
                w / h,
                ctx.pixels_per_point(),
            );
            let callback = egui_glow::CallbackFn::new(move |info, painter| {
                gl_renderer::paint(painter, &info, &geom);
            });
            painter.add(egui::PaintCallback {
                rect,
                callback: std::sync::Arc::new(callback),
            });
        } else if app.display.show_grid {
            // ── Draw grid (software) ──
            let gmin = -2.0;
            let gmax = 8.0;
            let step = 1.0;
            let mut i = gmin;
            while i <= gmax {
                if let (Some(a), Some(b)) = (
                    app.camera.project([gmin, 0.0, i], w, h),
                    app.camera.project([gmax, 0.0, i], w, h),
                ) {
                    painter.line_segment(
                        [scr(&a, rect), scr(&b, rect)],
                        egui::Stroke::new(1.0_f32, GRID_C),
                    );
                }
                if let (Some(a), Some(b)) = (
                    app.camera.project([i, 0.0, gmin], w, h),
                    app.camera.project([i, 0.0, gmax], w, h),
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
        // The axis letters stay on the painter in both modes; the coloured
        // lines are drawn by GL only in GPU mode.
        if app.display.show_global_axes {
            if use_gpu {
                draw_axis_labels(&app.camera, w, h, rect, &painter, 1.0);
            } else {
                draw_axes(&app.camera, w, h, rect, &painter, 1.0);
            }
        }

        // ── Depth-sort members ──
        let order: Vec<usize> = (0..scene.n_members()).collect();

        // ── Draw undeformed members (software renderer only) ──
        if show_base && !use_gpu {
            for &mi in &order {
                if app.display.is_member_hidden(scene.member_ids[mi]) {
                    continue;
                }
                let (ni, nj) = scene.member_nodes[mi];
                let pi = scene.node_positions[ni];
                let pj = scene.node_positions[nj];
                let Some(pa) = app.camera.project(pi, w, h) else {
                    continue;
                };
                let Some(pb) = app.camera.project(pj, w, h) else {
                    continue;
                };
                let is_sel = app.selection == SelectionState::Member(scene.member_ids[mi]);
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

        // ── Draw undeformed nodes (software renderer only) ──
        if show_base && !use_gpu {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if app.display.is_node_hidden(scene.node_ids[i]) {
                    continue;
                }
                let Some(p) = app.camera.project(pos, w, h) else {
                    continue;
                };
                let is_sel = app.selection == SelectionState::Node(scene.node_ids[i]);
                let r = if is_sel { 7.0 } else { 5.0 };
                let col = if is_sel { SELECTED_C } else { NODE_C };
                painter.circle_filled(scr(&p, rect), r, col);
            }
        }

        // ── Draw support symbols (Software only; GPU draws 3-D glyphs) ──
        // Fixed = square, Pinned = triangle, Custom = circle
        if show_base && !use_gpu {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if app.display.is_node_hidden(scene.node_ids[i]) {
                    continue;
                }
                let Some(spec) = scene.node_supports.get(i).copied().flatten() else {
                    continue;
                };
                let Some(p) = app.camera.project(pos, w, h) else {
                    continue;
                };
                let center = scr(&p, rect);
                let stroke = egui::Stroke::new(2.0_f32, SUPPORT_C);
                let label = if spec.is_fixed() {
                    "F".to_string()
                } else if spec.is_pinned() {
                    "P".to_string()
                } else {
                    format!("C{}", spec.n_constrained())
                };
                if spec.is_fixed() {
                    let half = 10.0_f32;
                    let sq = egui::Rect::from_center_size(center, egui::vec2(half * 2.0, half * 2.0));
                    painter.rect_stroke(sq, 0.0, stroke);
                } else if spec.is_pinned() {
                    let r = 12.0_f32;
                    let p1 = center + egui::vec2(0.0, -r);
                    let p2 = center + egui::vec2(-r * 0.866, r * 0.5);
                    let p3 = center + egui::vec2(r * 0.866, r * 0.5);
                    painter.line_segment([p1, p2], stroke);
                    painter.line_segment([p2, p3], stroke);
                    painter.line_segment([p3, p1], stroke);
                } else {
                    painter.circle_stroke(center, 10.0, stroke);
                }
                painter.text(
                    center,
                    egui::Align2::CENTER_CENTER,
                    label,
                    egui::FontId::proportional(10.0),
                    SUPPORT_C,
                );
            }
        }

        // ── Draw load indicators (Software only; GPU draws 3-D arrows) ──
        if show_base && !use_gpu {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if app.display.is_node_hidden(scene.node_ids[i]) {
                    continue;
                }
                let Some(load) = scene.node_loads.get(i).copied().flatten() else {
                    continue;
                };
                if load.is_zero() {
                    continue;
                }
                let Some(p) = app.camera.project(pos, w, h) else {
                    continue;
                };
                let center = scr(&p, rect);

                let fmag = load.force_magnitude();
                if fmag > 1e-12 {
                    let dir = [load.fx / fmag, load.fy / fmag, load.fz / fmag];
                    let tip_world = [
                        pos[0] + dir[0] * 0.5,
                        pos[1] + dir[1] * 0.5,
                        pos[2] + dir[2] * 0.5,
                    ];
                    if let Some(p_tip) = app.camera.project(tip_world, w, h) {
                        let tip = scr(&p_tip, rect);
                        painter.line_segment([center, tip], egui::Stroke::new(2.5_f32, LOAD_F_C));
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

        // ── Draw member distributed load indicators (Software only; GPU 3-D) ──
        if show_base && !use_gpu {
            for (mi, &_) in scene.member_nodes.iter().enumerate() {
                if app.display.is_member_hidden(scene.member_ids[mi]) {
                    continue;
                }
                let Some(load) = scene.member_loads.get(mi).copied().flatten() else {
                    continue;
                };
                if load.is_zero() || !load.is_finite() {
                    continue;
                }
                let (ni, nj) = scene.member_nodes[mi];
                let pi = scene.node_positions[ni];
                let pj = scene.node_positions[nj];
                let ref_vec = scene.member_ref_vecs[mi];

                let dx = pj[0] - pi[0];
                let dy = pj[1] - pi[1];
                let dz = pj[2] - pi[2];
                let len = dx.hypot(dy).hypot(dz);
                if len < 1e-12 {
                    continue;
                }
                let x_hat = [dx / len, dy / len, dz / len];
                let cross = [
                    x_hat[1] * ref_vec[2] - x_hat[2] * ref_vec[1],
                    x_hat[2] * ref_vec[0] - x_hat[0] * ref_vec[2],
                    x_hat[0] * ref_vec[1] - x_hat[1] * ref_vec[0],
                ];
                let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);
                if cross_norm < 1e-12 {
                    continue;
                }
                let z_hat = [
                    cross[0] / cross_norm,
                    cross[1] / cross_norm,
                    cross[2] / cross_norm,
                ];
                let y_hat = [
                    z_hat[1] * x_hat[2] - z_hat[2] * x_hat[1],
                    z_hat[2] * x_hat[0] - z_hat[0] * x_hat[2],
                    z_hat[0] * x_hat[1] - z_hat[1] * x_hat[0],
                ];

                let q_global = [
                    load.qx * x_hat[0] + load.qy * y_hat[0] + load.qz * z_hat[0],
                    load.qx * x_hat[1] + load.qy * y_hat[1] + load.qz * z_hat[1],
                    load.qx * x_hat[2] + load.qy * y_hat[2] + load.qz * z_hat[2],
                ];
                let qmag = q_global[0].hypot(q_global[1]).hypot(q_global[2]);
                if qmag < 1e-12 {
                    continue;
                }
                let dir = [q_global[0] / qmag, q_global[1] / qmag, q_global[2] / qmag];

                let arrow_len = 0.3 * len.min(1.0);
                let n_arrows = 5;
                for k in 1..n_arrows {
                    let t = k as f64 / n_arrows as f64;
                    let base = [pi[0] + t * dx, pi[1] + t * dy, pi[2] + t * dz];
                    let tip = [
                        base[0] + dir[0] * arrow_len,
                        base[1] + dir[1] * arrow_len,
                        base[2] + dir[2] * arrow_len,
                    ];
                    let Some(p_base) = app.camera.project(base, w, h) else {
                        continue;
                    };
                    let Some(p_tip) = app.camera.project(tip, w, h) else {
                        continue;
                    };
                    let s_base = scr(&p_base, rect);
                    let s_tip = scr(&p_tip, rect);
                    painter.line_segment(
                        [s_base, s_tip],
                        egui::Stroke::new(2.0_f32, MEMBER_LOAD_C),
                    );
                    painter.circle_filled(s_tip, 3.0, MEMBER_LOAD_C);
                }
            }
        }

        // ── Draw member point load indicators (Software only; GPU 3-D arrows) ──
        if show_base && !use_gpu {
            for (mi, &_) in scene.member_nodes.iter().enumerate() {
                if app.display.is_member_hidden(scene.member_ids[mi]) {
                    continue;
                }
                let Some(loads) = scene.member_point_loads.get(mi) else {
                    continue;
                };
                if loads.is_empty() {
                    continue;
                }
                let (ni, nj) = scene.member_nodes[mi];
                let pi = scene.node_positions[ni];
                let pj = scene.node_positions[nj];
                let ref_vec = scene.member_ref_vecs[mi];

                let dx = pj[0] - pi[0];
                let dy = pj[1] - pi[1];
                let dz = pj[2] - pi[2];
                let len = dx.hypot(dy).hypot(dz);
                if len < 1e-12 {
                    continue;
                }
                let x_hat = [dx / len, dy / len, dz / len];
                let cross = [
                    x_hat[1] * ref_vec[2] - x_hat[2] * ref_vec[1],
                    x_hat[2] * ref_vec[0] - x_hat[0] * ref_vec[2],
                    x_hat[0] * ref_vec[1] - x_hat[1] * ref_vec[0],
                ];
                let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);
                if cross_norm < 1e-12 {
                    continue;
                }
                let z_hat = [
                    cross[0] / cross_norm,
                    cross[1] / cross_norm,
                    cross[2] / cross_norm,
                ];
                let y_hat = [
                    z_hat[1] * x_hat[2] - z_hat[2] * x_hat[1],
                    z_hat[2] * x_hat[0] - z_hat[0] * x_hat[2],
                    z_hat[0] * x_hat[1] - z_hat[1] * x_hat[0],
                ];

                for pl in loads {
                    if pl.is_zero() || !pl.is_finite() {
                        continue;
                    }
                    let f_global = [
                        pl.fx * x_hat[0] + pl.fy * y_hat[0] + pl.fz * z_hat[0],
                        pl.fx * x_hat[1] + pl.fy * y_hat[1] + pl.fz * z_hat[1],
                        pl.fx * x_hat[2] + pl.fy * y_hat[2] + pl.fz * z_hat[2],
                    ];
                    let fmag = f_global[0].hypot(f_global[1]).hypot(f_global[2]);
                    if fmag < 1e-12 {
                        continue;
                    }
                    let dir = [f_global[0] / fmag, f_global[1] / fmag, f_global[2] / fmag];

                    let base = [pi[0] + pl.xi * dx, pi[1] + pl.xi * dy, pi[2] + pl.xi * dz];
                    let arrow_len = 0.25 * len.min(1.0);
                    let tip = [
                        base[0] + dir[0] * arrow_len,
                        base[1] + dir[1] * arrow_len,
                        base[2] + dir[2] * arrow_len,
                    ];
                    let Some(p_base) = app.camera.project(base, w, h) else {
                        continue;
                    };
                    let Some(p_tip) = app.camera.project(tip, w, h) else {
                        continue;
                    };
                    let s_base = scr(&p_base, rect);
                    let s_tip = scr(&p_tip, rect);
                    painter.line_segment(
                        [s_base, s_tip],
                        egui::Stroke::new(2.5_f32, MEMBER_LOAD_C),
                    );
                    painter.circle_filled(s_tip, 4.0, MEMBER_LOAD_C);
                }
            }
        }

        // ── Draw deformed shape (cubic Hermite beam curve) ──
        // Software only; in GPU mode the renderer sweeps the sampled curve as
        // depth-tested solids (see `GlGeometry::build`).
        if !use_gpu && app.display.mode == ResultViewMode::Deformed {
            if let Some(result) = app.analysis_result() {
                let scale = app.display.deformation_scale;
                let deformed = scene.deformed_positions(&result.displacements, scale);
                let stroke = egui::Stroke::new(1.5_f32, DEF_MEMBER);
                for &mi in &order {
                    if app.display.is_member_hidden(scene.member_ids[mi]) {
                        continue;
                    }
                    // Sample the true beam curve (Hermite) member by member.
                    let mut prev: Option<egui::Pos2> = None;
                    let mut drawn = false;
                    for s in 0..DEFORM_SAMPLES {
                        let xi = s as f64 / (DEFORM_SAMPLES - 1) as f64;
                        let Some(pos) =
                            scene.member_deformed_point(mi, xi, &result.displacements, scale)
                        else {
                            break;
                        };
                        let Some(pa) = app.camera.project(pos, w, h) else {
                            prev = None;
                            continue;
                        };
                        let p = scr(&pa, rect);
                        if let Some(q) = prev {
                            painter.line_segment([q, p], stroke);
                            drawn = true;
                        }
                        prev = Some(p);
                    }
                    if !drawn {
                        // Degenerate local axes: fall back to the straight chord.
                        let (ni, nj) = scene.member_nodes[mi];
                        if let (Some(pa), Some(pb)) = (
                            app.camera.project(deformed[ni], w, h),
                            app.camera.project(deformed[nj], w, h),
                        ) {
                            painter.line_segment([scr(&pa, rect), scr(&pb, rect)], stroke);
                        }
                    }
                }
                for (i, &pos) in deformed.iter().enumerate() {
                    if scene
                        .node_ids
                        .get(i)
                        .is_some_and(|&id| app.display.is_node_hidden(id))
                    {
                        continue;
                    }
                    let Some(p) = app.camera.project(pos, w, h) else {
                        continue;
                    };
                    painter.circle_filled(scr(&p, rect), 3.5, DEF_NODE);
                }
            }
        }

        // ── Draw displacement color-coding ──
        // In GPU mode the member prisms are contoured by the renderer; only the
        // node markers (software) and the legend stay on the painter.
        if app.display.mode == ResultViewMode::Displacement {
            if let Some(result) = app.analysis_result() {
                let mags = displacement_magnitudes(&result.displacements, scene.n_nodes());
                let max_mag = max_abs(&mags);
                if !use_gpu {
                    for (i, &mag) in mags.iter().enumerate() {
                        if app.display.is_node_hidden(scene.node_ids[i]) {
                            continue;
                        }
                        let &pos = &scene.node_positions[i];
                        let Some(p) = app.camera.project(pos, w, h) else {
                            continue;
                        };
                        let t = normalize(mag, max_mag);
                        let [r, g, b] = magnitude_color(t);
                        let col = egui::Color32::from_rgb(r, g, b);
                        let radius = (5.0 + t * 4.0) as f32;
                        painter.circle_filled(scr(&p, rect), radius, col);
                    }
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

        // ── Draw reaction indicators (Software only; GPU draws 3-D arrows) ──
        if !use_gpu && app.display.mode == ResultViewMode::Reaction {
            if let Some(result) = app.analysis_result() {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    if app.display.is_node_hidden(scene.node_ids[i]) {
                        continue;
                    }
                    let Some(_) = scene.node_supports.get(i).copied().flatten() else {
                        continue;
                    };
                    let fx = result.reactions.get(i * 6).copied().unwrap_or(0.0);
                    let fy = result.reactions.get(i * 6 + 1).copied().unwrap_or(0.0);
                    let fz = result.reactions.get(i * 6 + 2).copied().unwrap_or(0.0);
                    let mx = result.reactions.get(i * 6 + 3).copied().unwrap_or(0.0);
                    let my = result.reactions.get(i * 6 + 4).copied().unwrap_or(0.0);
                    let mz = result.reactions.get(i * 6 + 5).copied().unwrap_or(0.0);

                    let Some(p) = app.camera.project(pos, w, h) else {
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
                        if let Some(p_tip) = app.camera.project(tip_world, w, h) {
                            let tip = scr(&p_tip, rect);
                            painter.line_segment([center, tip], egui::Stroke::new(3.0_f32, SUPPORT_C));
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

        // ── Draw member end-force indicators (Software only; GPU 3-D arrows) ──
        if !use_gpu && app.display.mode == ResultViewMode::MemberEndForces {
            if let (SelectionState::Member(mid), Some(result)) =
                (app.selection, app.analysis_result())
            {
                if let Some(mi) = scene.member_index(mid) {
                    if !app.display.is_member_hidden(mid) {
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
                                let Some(p0) = app.camera.project(end, w, h) else {
                                    continue;
                                };
                                let Some(p1) = app.camera.project(tip_world, w, h) else {
                                    continue;
                                };
                                let c0 = scr(&p0, rect);
                                let c1 = scr(&p1, rect);
                                painter
                                    .line_segment([c0, c1], egui::Stroke::new(3.0_f32, SELECTED_C));
                                painter.circle_filled(c1, 5.0, SELECTED_C);
                            }
                        }
                    }
                }
            }
        }

        // ── Internal-force diagrams (N / V / M) ──
        // Software: the full painter diagram (polylines, hatching, labels,
        // legend).  GPU: the 3-D depth-tested line strips are emitted by the
        // renderer, so only the numeric labels + legend stay on the painter.
        if use_gpu {
            super::diagrams::show_diagram_labels(app, scene, rect, &painter);
        } else {
            super::diagrams::show_diagrams(app, scene, rect, &painter);
        }

        // ── Draw local axes for selected member (Software only; GPU triads) ──
        if !use_gpu {
            if let (SelectionState::Member(mid), Some(result)) =
                (app.selection, app.analysis_result())
            {
                if app.display.show_local_axes && !app.display.is_member_hidden(mid) {
                    if let Some(mi) = scene.member_index(mid) {
                        if let Ok((xh, yh, zh)) = result.member_local_axes(mi) {
                            if let Some(mpos) = scene.member_midpoint(mi) {
                                draw_local_axes(
                                    &app.camera,
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
        }

        // ── Node labels ──
        if app.display.show_node_labels {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if app.display.is_node_hidden(scene.node_ids[i]) {
                    continue;
                }
                let Some(p) = app.camera.project(pos, w, h) else {
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
            "Drag: orbit | R-drag: pan | Scroll: zoom | R: reset | F: fit | 1-4: views | Click: select | Dbl-click: edit | R-click: menu | Esc: clear",
            egui::FontId::proportional(11.0),
            HELP_C,
        );

        // ── View cube (bottom-right) ──
        show_view_cube(app, ui, rect, &painter);
    });
}

// ── Helpers ─────────────────────────────────────────────────────────────

fn scr(p: &ScreenPos, rect: egui::Rect) -> egui::Pos2 {
    egui::pos2(rect.min.x + p.x as f32, rect.min.y + p.y as f32)
}

/// Hit-test the viewport at `pos` (screen points) using the CPU picker.
fn pick_at(app: &ViewportApp, pos: egui::Pos2, rect: egui::Rect, w: f64, h: f64) -> SelectionState {
    app.pick(
        [pos.x as f64 - rect.min.x as f64, pos.y as f64 - rect.min.y as f64],
        [w, h],
    )
}

/// Bottom-right square reserved for the view-cube gizmo.
fn view_cube_rect(rect: egui::Rect) -> egui::Rect {
    egui::Rect::from_min_size(
        egui::pos2(
            rect.max.x - VIEW_CUBE_SIZE - VIEW_CUBE_MARGIN,
            rect.max.y - VIEW_CUBE_SIZE - VIEW_CUBE_MARGIN,
        ),
        egui::vec2(VIEW_CUBE_SIZE, VIEW_CUBE_SIZE),
    )
}

/// Right-click context menu, with items depending on `target`.
fn viewport_context_menu(app: &mut ViewportApp, ui: &mut egui::Ui, target: SelectionState) {
    match target {
        SelectionState::Node(id) => {
            ui.label(egui::RichText::new(format!("Node N{}", id.0)).strong());
            ui.separator();
            if ui.button("Delete node").clicked() {
                app.delete_node(id);
                ui.close_menu();
            }
            if ui.button("Hide node").clicked() {
                app.selection = SelectionState::Node(id);
                app.hide_node(id);
                ui.close_menu();
            }
            if ui.button("Hide others").clicked() {
                app.selection = SelectionState::Node(id);
                app.hide_others_for_selection();
                ui.close_menu();
            }
            if ui.button("Show all").clicked() {
                app.show_all_hidden();
                ui.close_menu();
            }
            ui.separator();
            if ui.button("Add support\u{2026}").clicked() {
                app.select_and_open(SelectionState::Node(id), WorkModule::Model);
                ui.close_menu();
            }
            if ui.button("Add nodal load\u{2026}").clicked() {
                app.select_and_open(SelectionState::Node(id), WorkModule::Load);
                ui.close_menu();
            }
            ui.separator();
            if ui.button("Zoom to node").clicked() {
                app.zoom_to_node(id);
                ui.close_menu();
            }
        }
        SelectionState::Member(id) => {
            ui.label(egui::RichText::new(format!("Member M{}", id.0)).strong());
            ui.separator();
            if ui.button("Delete member").clicked() {
                app.delete_member(id);
                ui.close_menu();
            }
            if ui.button("Hide member").clicked() {
                app.selection = SelectionState::Member(id);
                app.hide_member(id);
                ui.close_menu();
            }
            if ui.button("Hide others").clicked() {
                app.selection = SelectionState::Member(id);
                app.hide_others_for_selection();
                ui.close_menu();
            }
            if ui.button("Show all").clicked() {
                app.show_all_hidden();
                ui.close_menu();
            }
            ui.separator();
            if ui.button("Assign section\u{2026}").clicked() {
                app.select_and_open(SelectionState::Member(id), WorkModule::Section);
                ui.close_menu();
            }
            if ui.button("Add member load\u{2026}").clicked() {
                app.select_and_open(SelectionState::Member(id), WorkModule::Load);
                ui.close_menu();
            }
            if ui.button("Show end forces").clicked() {
                app.show_member_end_forces(id);
                ui.close_menu();
            }
            ui.separator();
            if ui.button("Zoom to member").clicked() {
                app.zoom_to_member(id);
                ui.close_menu();
            }
        }
        SelectionState::None => {
            if ui.button("Fit").clicked() {
                app.fit_view();
                ui.close_menu();
            }
            if ui.button("Show all").clicked() {
                app.show_all_hidden();
                ui.close_menu();
            }
            ui.separator();
            let mut grid = app.display.show_grid;
            if ui.checkbox(&mut grid, "Ground grid").changed() {
                app.display.show_grid = grid;
            }
            let mut labels = app.display.show_node_labels;
            if ui.checkbox(&mut labels, "Node labels").changed() {
                app.display.show_node_labels = labels;
            }
            let mut axes = app.display.show_local_axes;
            if ui.checkbox(&mut axes, "Local axes").changed() {
                app.display.show_local_axes = axes;
            }
        }
    }
}

/// Axis letter labels only (GPU mode draws the coloured lines itself).
fn draw_axis_labels(
    cam: &Camera3D,
    w: f64,
    h: f64,
    rect: egui::Rect,
    painter: &egui::Painter,
    len: f64,
) {
    for (end, col, lbl) in [
        ([len, 0.0, 0.0], AXIS_X, "X"),
        ([0.0, len, 0.0], AXIS_Y, "Y"),
        ([0.0, 0.0, len], AXIS_Z, "Z"),
    ] {
        if let Some(p1) = cam.project(end, w, h) {
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

// ── View cube (orientation gizmo) ───────────────────────────────────────

/// One face of the view cube (world-space normal + corners + target view).
struct CubeFace {
    normal: [f64; 3],
    corners: [[f64; 3]; 4],
    label: &'static str,
    view: StandardView,
}

/// The six axis-aligned faces.  Corners run around the face (order only
/// matters for filling, which uses a winding-independent polygon test).
const CUBE_FACES: [CubeFace; 6] = [
    CubeFace {
        normal: [0.0, 0.0, 1.0],
        corners: [
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ],
        label: "F",
        view: StandardView::Front,
    },
    CubeFace {
        normal: [0.0, 0.0, -1.0],
        corners: [
            [1.0, -1.0, -1.0],
            [-1.0, -1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
        ],
        label: "B",
        view: StandardView::Back,
    },
    CubeFace {
        normal: [1.0, 0.0, 0.0],
        corners: [
            [1.0, -1.0, 1.0],
            [1.0, -1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
        ],
        label: "R",
        view: StandardView::Right,
    },
    CubeFace {
        normal: [-1.0, 0.0, 0.0],
        corners: [
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, 1.0, 1.0],
            [-1.0, 1.0, -1.0],
        ],
        label: "L",
        view: StandardView::Left,
    },
    CubeFace {
        normal: [0.0, 1.0, 0.0],
        corners: [
            [-1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, -1.0],
            [-1.0, 1.0, -1.0],
        ],
        label: "T",
        view: StandardView::Top,
    },
    CubeFace {
        normal: [0.0, -1.0, 0.0],
        corners: [
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [-1.0, -1.0, 1.0],
        ],
        label: "D",
        view: StandardView::Bottom,
    },
];

/// Draw the view cube and handle its interactions.
///
/// The cube is drawn with the camera's own basis (a small orthographic
/// projection of the world axes), so it rotates with the view.  Clicking a
/// visible face snaps to that standard view; clicking a corner snaps to `Iso`;
/// dragging the gizmo orbits the camera.  Reuses [`Camera3D::set_view`] /
/// [`ViewportApp::set_standard_view`], so it never bypasses the existing
/// view machinery.
fn show_view_cube(app: &mut ViewportApp, ui: &mut egui::Ui, rect: egui::Rect, painter: &egui::Painter) {
    let region = view_cube_rect(rect);
    if !rect.contains_rect(region) {
        return;
    }

    painter.rect_filled(
        region,
        4.0,
        egui::Color32::from_rgba_premultiplied(18, 18, 24, 175),
    );
    painter.rect_stroke(region, 4.0, egui::Stroke::new(1.0_f32, GRID_C));

    let (forward, right, up) = app.camera.basis();
    let eye_dir = [-forward[0], -forward[1], -forward[2]];
    let light = project::normalize([0.35, 0.85, 0.45]);
    let center = region.center();
    let scale = VIEW_CUBE_SIZE as f64 * 0.24;
    let project_dir = |w: [f64; 3]| -> egui::Pos2 {
        let x = project::dot(w, right);
        let y = project::dot(w, up);
        egui::pos2(center.x + (x * scale) as f32, center.y - (y * scale) as f32)
    };

    let mut face_polys: Vec<(StandardView, [egui::Pos2; 4])> = Vec::new();
    for face in &CUBE_FACES {
        if project::dot(face.normal, eye_dir) <= 1e-6 {
            continue; // back-facing
        }
        let pts = [
            project_dir(face.corners[0]),
            project_dir(face.corners[1]),
            project_dir(face.corners[2]),
            project_dir(face.corners[3]),
        ];
        let lit = 0.45 + 0.55 * project::dot(face.normal, light).max(0.0);
        let shade = |c: u8| (c as f64 * lit).clamp(0.0, 255.0) as u8;
        let fill = egui::Color32::from_rgb(shade(NODE_C.r()), shade(NODE_C.g()), shade(NODE_C.b()));
        painter.add(egui::Shape::convex_polygon(
            pts.to_vec(),
            fill,
            egui::Stroke::new(1.0_f32, GRID_C),
        ));
        let centroid = egui::pos2(
            (pts[0].x + pts[1].x + pts[2].x + pts[3].x) * 0.25,
            (pts[0].y + pts[1].y + pts[2].y + pts[3].y) * 0.25,
        );
        painter.text(
            centroid,
            egui::Align2::CENTER_CENTER,
            face.label,
            egui::FontId::proportional(13.0),
            egui::Color32::from_rgb(30, 30, 36),
        );
        face_polys.push((face.view, pts));
    }

    // The eight cube corners map to the isometric view.
    let mut corners: Vec<egui::Pos2> = Vec::with_capacity(8);
    for i in 0..8 {
        let sx = if i & 1 == 0 { -1.0 } else { 1.0 };
        let sy = if i & 2 == 0 { -1.0 } else { 1.0 };
        let sz = if i & 4 == 0 { -1.0 } else { 1.0 };
        corners.push(project_dir([sx, sy, sz]));
    }

    let response = ui.interact(region, ui.id().with("view_cube"), egui::Sense::click_and_drag());
    if response.dragged() {
        let d = response.drag_delta();
        app.camera.orbit(-d.x as f64 * 0.02, d.y as f64 * 0.02);
    } else if response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            let mut chosen = None;
            if corners.iter().any(|c| c.distance(pos) < 7.0) {
                chosen = Some(StandardView::Iso);
            } else {
                for (view, poly) in &face_polys {
                    if point_in_poly(pos, poly) {
                        chosen = Some(*view);
                        break;
                    }
                }
            }
            if let Some(view) = chosen {
                app.set_standard_view(view);
            }
        }
    }
}

/// Even-odd (winding-independent) point-in-polygon test.
fn point_in_poly(p: egui::Pos2, poly: &[egui::Pos2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i].x as f64, poly[i].y as f64);
        let (xj, yj) = (poly[j].x as f64, poly[j].y as f64);
        let crosses = ((yi > p.y as f64) != (yj > p.y as f64))
            && ((p.x as f64) < (xj - xi) * (p.y as f64 - yi) / (yj - yi) + xi);
        if crosses {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ── Double-click quick edit ─────────────────────────────────────────────

/// Draw the double-click quick-edit popup, if one is open.
///
/// A node offers `x / y / z`; a member offers its `i / j` nodes plus the
/// section numbers, reusing the shared [`MemberEditBuffer`](crate::MemberEditBuffer)
/// and the inspector's apply path so no edit mechanics are duplicated.  Closing
/// happens on Escape, the title-bar ✕, the Close button, or a viewport click.
pub fn show_quick_edit(app: &mut ViewportApp, ctx: &egui::Context) {
    let Some(mut qe) = app.quick_edit.take() else {
        return;
    };
    let target = qe.target;
    let mut open = true;
    let mut apply = false;
    let mut close = false;

    egui::Window::new("Quick edit")
        .id(egui::Id::new("quick_edit_popup"))
        .collapsible(false)
        .resizable(false)
        .open(&mut open)
        .default_pos(qe.anchor + egui::vec2(14.0, 14.0))
        .show(ctx, |ui| {
            match target {
                SelectionState::Node(id) => {
                    ui.label(egui::RichText::new(format!("Node N{}", id.0)).strong());
                    egui::Grid::new("quick_node_grid")
                        .num_columns(2)
                        .spacing([8.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("x:");
                            ui.add(egui::DragValue::new(&mut qe.x).speed(0.1).range(-1e6..=1e6));
                            ui.end_row();
                            ui.label("y:");
                            ui.add(egui::DragValue::new(&mut qe.y).speed(0.1).range(-1e6..=1e6));
                            ui.end_row();
                            ui.label("z:");
                            ui.add(egui::DragValue::new(&mut qe.z).speed(0.1).range(-1e6..=1e6));
                            ui.end_row();
                        });
                }
                SelectionState::Member(id) => {
                    ui.label(egui::RichText::new(format!("Member M{}", id.0)).strong());
                    ui.horizontal(|ui| {
                        ui.label("N i:");
                        ui.add(egui::TextEdit::singleline(&mut qe.node_i).desired_width(48.0));
                        ui.label("N j:");
                        ui.add(egui::TextEdit::singleline(&mut qe.node_j).desired_width(48.0));
                    });
                    ui.separator();
                    ui.label("Section / material:");
                    egui::Grid::new("quick_section_grid")
                        .num_columns(2)
                        .spacing([8.0, 3.0])
                        .show(ui, |ui| {
                            ui.label("E:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.e)
                                    .speed(1e8)
                                    .range(1.0..=1e12),
                            );
                            ui.end_row();
                            ui.label("G:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.g)
                                    .speed(1e8)
                                    .range(1.0..=1e12),
                            );
                            ui.end_row();
                            ui.label("A:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.area)
                                    .speed(1e-5)
                                    .range(1e-12..=1.0),
                            );
                            ui.end_row();
                            ui.label("Iy:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.iy)
                                    .speed(1e-8)
                                    .range(1e-15..=1.0),
                            );
                            ui.end_row();
                            ui.label("Iz:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.iz)
                                    .speed(1e-8)
                                    .range(1e-15..=1.0),
                            );
                            ui.end_row();
                            ui.label("J:");
                            ui.add(
                                egui::DragValue::new(&mut app.member_edit.j)
                                    .speed(1e-8)
                                    .range(1e-15..=1.0),
                            );
                            ui.end_row();
                        });
                }
                SelectionState::None => {}
            }
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Apply").clicked() {
                    apply = true;
                }
                if ui.button("Close").clicked() {
                    close = true;
                }
            });
        });

    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        close = true;
    }
    if !open {
        close = true;
    }

    if apply {
        app.quick_edit = Some(qe);
        app.apply_quick_edit();
    } else if !close {
        app.quick_edit = Some(qe);
    }
}
