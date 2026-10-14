//! Central 3D viewport canvas: geometry, loads, supports, and result
//! overlays.
//!
//! This is a straight move of the former `update()` central-panel body —
//! the drawing order and colors are unchanged.

use egui;
use eframe::egui_glow;
use structural_analysis::Dof3D;

use viewport_poc::app_state::AppMessage;
use viewport_poc::camera::{Camera3D, ScreenPos, StandardView, ViewMode};
use viewport_poc::display::{
    displacement_magnitudes, fmt_eng, magnitude_color, max_abs, normalize, ResultViewMode,
};
use viewport_poc::edit_model::{MemberPointLoad3D, SupportSpec};
use viewport_poc::project;
use viewport_poc::scene::ViewportScene;
use viewport_poc::selection::{ScreenRect, SelectionState};

use super::gl_renderer::{self, RenderMode};
use super::module::WorkModule;
use super::theme::{TextPalette, ViewportPalette};
use crate::{BoundaryPopup, ViewportApp};

/// Samples per member for the deformed-shape curve.
const DEFORM_SAMPLES: usize = 12;

/// Side length of the view-cube gizmo (points).
const VIEW_CUBE_SIZE: f32 = 90.0;
/// Margin between the view cube and the viewport corner (points).
const VIEW_CUBE_MARGIN: f32 = 12.0;
/// Click radius (points) around a projected cube corner that snaps to `Iso`.
const CUBE_CORNER_SNAP_PX: f32 = 7.0;
/// Click band (points) around a visible cube edge that snaps to the edge's
/// 45° two-axis view (Phase P7-2).
const CUBE_EDGE_SNAP_PX: f32 = 8.0;

/// Draw the central viewport panel.
pub fn show_viewport(app: &mut ViewportApp, ctx: &egui::Context) {
    // Surface any one-shot GPU-renderer message (e.g. a runtime fallback).
    if let Some(msg) = gl_renderer::take_message() {
        app.messages.push(AppMessage::warning(msg));
    }

    // Theme-dependent viewport palette (Phase P7-2): the 3-D canvas follows
    // the light/dark toggle, re-read every frame so a theme switch is
    // immediate in both renderers.
    let pal = ViewportPalette::for_theme(app.dark_mode);

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

        painter.rect_filled(rect, 0.0, pal.bg);

        // The view cube lives in the bottom-right corner; its rect is fixed for
        // the whole frame, so viewport clicks inside it are not treated as
        // selections.
        let gizmo_rect = view_cube_rect(rect);

        // Camera input — does NOT trigger solve.  A primary drag that starts on
        // the view cube is left to the gizmo (which orbits itself), so the two
        // handlers never double up.
        //
        // Phase P6: a Shift (or Ctrl/Cmd) + primary drag is a **box select**
        // rubber band instead of an orbit.  The plain primary drag still orbits
        // and the Alt + primary drag still slides the section cut.
        //
        // Phase B2: with the ribbon's Pan / Zoom mode active the plain primary
        // drag pans / dollies instead; box select, the Alt cut drag and the
        // right-button pan keep their behaviour in every mode.
        let box_modifier = ctx.input(|i| i.modifiers.shift || i.modifiers.command);
        if response.drag_started_by(egui::PointerButton::Primary) {
            app.box_select_start = response
                .interact_pointer_pos()
                .filter(|p| box_modifier && !gizmo_rect.contains(*p));
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            let over_gizmo = response
                .interact_pointer_pos()
                .is_some_and(|p| gizmo_rect.contains(p));
            // Alt + primary-drag slides the section-cut plane along its normal
            // (only meaningful once the cut is enabled); otherwise it orbits.
            let cut_drag = app.display.section_cut.enabled && ctx.input(|i| i.modifiers.alt);
            if app.box_select_start.is_some() && !over_gizmo {
                // Rubber band in progress — leave the camera untouched.
            } else if cut_drag {
                drag_section_cut(app, response.drag_delta(), w, h);
            } else if !over_gizmo {
                let d = response.drag_delta();
                match app.view_mode {
                    ViewMode::Orbit => app.camera.orbit(-d.x as f64 * 0.01, d.y as f64 * 0.01),
                    ViewMode::Pan => {
                        let s = app.camera.distance * 0.0015;
                        app.camera.pan(d.x as f64 * s, d.y as f64 * s);
                    }
                    // Vertical drag dollies: dragging up (negative dy) zooms in,
                    // mirroring the scroll-wheel direction.
                    ViewMode::Zoom => app.camera.zoom(2.0_f64.powf(d.y as f64 * 0.01)),
                }
            }
        }
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            if let Some(start) = app.box_select_start.take() {
                let end = response
                    .interact_pointer_pos()
                    .or_else(|| ctx.input(|i| i.pointer.interact_pos()));
                if let Some(end) = end {
                    let local = |p: egui::Pos2| {
                        [p.x as f64 - rect.min.x as f64, p.y as f64 - rect.min.y as f64]
                    };
                    let sr = ScreenRect::from_corners(local(start), local(end));
                    if !sr.is_click() {
                        // Ctrl/Cmd held ⇒ add to the existing selection.
                        let add = ctx.input(|i| i.modifiers.command);
                        app.box_select(sr, [w, h], add);
                        app.quick_edit = None;
                        app.boundary_popup = None;
                    }
                }
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
        // Escape closes the quick-edit / boundary popups first; then returns a
        // Pan / Zoom mode to Orbit; only clears the selection when nothing
        // else is open.
        if ctx.input(|i| i.key_pressed(egui::Key::Escape))
            && app.quick_edit.is_none()
            && app.boundary_popup.is_none()
        {
            if app.view_mode != ViewMode::Orbit {
                app.view_mode = ViewMode::Orbit;
            } else {
                app.clear_selection();
            }
        }

        // Selection on click — does NOT trigger solve.  Clicks on the view cube
        // are handled by the gizmo, not by picking.  A click in the viewport
        // also dismisses the quick-edit popup ("click away to close").
        //
        // Phase P6 modifiers: plain = replace, Shift = toggle, Ctrl/Cmd = add.
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if !gizmo_rect.contains(pos) {
                    let hit = pick_at(app, pos, rect, w, h);
                    let mods = ctx.input(|i| i.modifiers);
                    if mods.shift {
                        app.select_toggle(hit);
                    } else if mods.command {
                        app.select_add(hit);
                    } else {
                        app.select_only(hit);
                    }
                    app.quick_edit = None;
                    app.boundary_popup = None;
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
                pal.label,
            );
            painter.text(
                rect.min + egui::vec2(10.0, rect.height() - 10.0),
                egui::Align2::LEFT_BOTTOM,
                format!(
                    "Drag: {} | R-drag: pan | Scroll: zoom | R: reset | F: fit | 1-4: views | Click: select | Dbl-click: edit | R-click: menu | Esc: clear",
                    drag_hint(app.view_mode)
                ),
                egui::FontId::proportional(11.0),
                pal.help,
            );
            show_view_cube(app, ui, rect, &painter, pal);
            return;
        };

        // Determine whether to show base (undeformed) geometry.
        let show_base = app.display.mode != ResultViewMode::Deformed || app.display.show_undeformed;

        // Section-cut plane (Phase P5).  A cheap owned snapshot avoids holding
        // a borrow of `app` across the rest of the frame.  The **software**
        // path approximates the GPU clip distance by clipping member lines and
        // polyline segments at the plane and skipping fully-clipped markers.
        // The grid and global axes are exempt (drawn unclipped); the
        // internal-force diagram polylines (owned by `diagrams`) are not
        // clipped in software mode — see the module docs.
        //
        // Phase P7-1: the *capped* cut (a solid-looking fill over the opening)
        // is a GPU-only effect — it needs a second depth-tested pass over the
        // interior back faces, which the painter cannot do.  The Software
        // renderer therefore keeps this open-cut behaviour; the `Cap cut faces`
        // toggle is a no-op here.
        let cut = app.display.section_cut.clone();

        // Renderer selection (Phase P1): the GPU path draws grid / axes /
        // members / nodes into a depth-tested offscreen buffer and blits it
        // here; the software path keeps drawing them with the painter.
        let use_gpu = gl_renderer::mode() == RenderMode::Gpu;

        if use_gpu {
            let geom = gl_renderer::GlGeometry::build(
                &app.camera,
                scene,
                &app.display,
                &app.selected,
                app.analysis_result(),
                show_base,
                w / h,
                ctx.pixels_per_point(),
                pal,
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
                        egui::Stroke::new(1.0_f32, pal.grid),
                    );
                }
                if let (Some(a), Some(b)) = (
                    app.camera.project([i, 0.0, gmin], w, h),
                    app.camera.project([i, 0.0, gmax], w, h),
                ) {
                    painter.line_segment(
                        [scr(&a, rect), scr(&b, rect)],
                        egui::Stroke::new(1.0_f32, pal.grid),
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
                draw_axis_labels(&app.camera, w, h, rect, &painter, 1.0, pal);
            } else {
                draw_axes(&app.camera, w, h, rect, &painter, 1.0, pal);
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
                // Clip the centreline at the section-cut plane (best effort).
                let Some((ca, cb)) = cut.clip_segment(pi, pj) else {
                    continue;
                };
                let Some(pa) = app.camera.project(ca, w, h) else {
                    continue;
                };
                let Some(pb) = app.camera.project(cb, w, h) else {
                    continue;
                };
                let is_sel = app.selected.contains_member(scene.member_ids[mi]);
                let (wd, col) = if is_sel {
                    (4.0, pal.selected)
                } else {
                    (2.5, pal.member)
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
                if !cut.keeps(pos) {
                    continue;
                }
                let Some(p) = app.camera.project(pos, w, h) else {
                    continue;
                };
                let is_sel = app.selected.contains_node(scene.node_ids[i]);
                let r = if is_sel { 7.0 } else { 5.0 };
                let col = if is_sel { pal.selected } else { pal.node };
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
                if !cut.keeps(pos) {
                    continue;
                }
                let Some(p) = app.camera.project(pos, w, h) else {
                    continue;
                };
                let center = scr(&p, rect);
                let stroke = egui::Stroke::new(2.0_f32, pal.support);
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
                    pal.support,
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
                // The load symbol is anchored on the node; drop it when the
                // node is cut away.
                if !cut.keeps(pos) {
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
                        painter.line_segment([center, tip], egui::Stroke::new(2.5_f32, pal.load_f));
                        painter.circle_filled(tip, 4.0, pal.load_f);
                    }
                }

                let mmag = load.moment_magnitude();
                if mmag > 1e-12 {
                    painter.circle_stroke(
                        center + egui::vec2(14.0, -14.0),
                        6.0,
                        egui::Stroke::new(2.0_f32, pal.load_m),
                    );
                    painter.text(
                        center + egui::vec2(14.0, -14.0),
                        egui::Align2::CENTER_CENTER,
                        "M",
                        egui::FontId::proportional(8.0),
                        pal.load_m,
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
                    if !cut.keeps(base) {
                        continue;
                    }
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
                        egui::Stroke::new(2.0_f32, pal.member_load),
                    );
                    painter.circle_filled(s_tip, 3.0, pal.member_load);
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
                    if !cut.keeps(base) {
                        continue;
                    }
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
                        egui::Stroke::new(2.5_f32, pal.member_load),
                    );
                    painter.circle_filled(s_tip, 4.0, pal.member_load);
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
                let stroke = egui::Stroke::new(1.5_f32, pal.def_member);
                for &mi in &order {
                    if app.display.is_member_hidden(scene.member_ids[mi]) {
                        continue;
                    }
                    // Sample the true beam curve (Hermite) member by member.
                    let mut prev: Option<[f64; 3]> = None;
                    let mut drawn = false;
                    for s in 0..DEFORM_SAMPLES {
                        let xi = s as f64 / (DEFORM_SAMPLES - 1) as f64;
                        let Some(pos) =
                            scene.member_deformed_point(mi, xi, &result.displacements, scale)
                        else {
                            break;
                        };
                        if let Some(q) = prev {
                            // Clip each curve segment at the section-cut plane.
                            if let Some((ca, cb)) = cut.clip_segment(q, pos) {
                                if let (Some(pa), Some(pb)) =
                                    (app.camera.project(ca, w, h), app.camera.project(cb, w, h))
                                {
                                    painter
                                        .line_segment([scr(&pa, rect), scr(&pb, rect)], stroke);
                                    drawn = true;
                                }
                            }
                        }
                        prev = Some(pos);
                    }
                    if !drawn {
                        // Degenerate local axes: fall back to the straight chord.
                        let (ni, nj) = scene.member_nodes[mi];
                        if let Some((ca, cb)) = cut.clip_segment(deformed[ni], deformed[nj]) {
                            if let (Some(pa), Some(pb)) =
                                (app.camera.project(ca, w, h), app.camera.project(cb, w, h))
                            {
                                painter.line_segment([scr(&pa, rect), scr(&pb, rect)], stroke);
                            }
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
                    if !cut.keeps(pos) {
                        continue;
                    }
                    let Some(p) = app.camera.project(pos, w, h) else {
                        continue;
                    };
                    painter.circle_filled(scr(&p, rect), 3.5, pal.def_node);
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
                        if !cut.keeps(pos) {
                            continue;
                        }
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
                        pal.label,
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
                        pal.label,
                    );
                    painter.text(
                        egui::pos2(lx + 120.0, ly + 32.0),
                        egui::Align2::LEFT_TOP,
                        format!("{}", fmt_eng(max_mag)),
                        egui::FontId::proportional(9.0),
                        pal.label,
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
                    // Reaction symbols are anchored on the node.
                    if !cut.keeps(pos) {
                        continue;
                    }
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
                            painter.line_segment([center, tip], egui::Stroke::new(3.0_f32, pal.support));
                            painter.circle_filled(tip, 5.0, pal.support);
                        }
                    }

                    let mmag = (mx * mx + my * my + mz * mz).sqrt();
                    if mmag > 1e-9 {
                        painter.circle_stroke(
                            center + egui::vec2(14.0, -14.0),
                            7.0,
                            egui::Stroke::new(2.5_f32, pal.load_m),
                        );
                        painter.text(
                            center + egui::vec2(14.0, -14.0),
                            egui::Align2::CENTER_CENTER,
                            "R",
                            egui::FontId::proportional(8.0),
                            pal.load_m,
                        );
                    }
                }
            }
        }

        // ── Draw member end-force indicators (Software only; GPU 3-D arrows) ──
        if !use_gpu && app.display.mode == ResultViewMode::MemberEndForces {
            if let (Some(mid), Some(result)) =
                (app.selected.primary().member_id(), app.analysis_result())
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
                                // End-force arrow is anchored at the member end.
                                if !cut.keeps(end) {
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
                                    .line_segment([c0, c1], egui::Stroke::new(3.0_f32, pal.selected));
                                painter.circle_filled(c1, 5.0, pal.selected);
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
            if let (Some(mid), Some(result)) =
                (app.selected.primary().member_id(), app.analysis_result())
            {
                if app.display.show_local_axes && !app.display.is_member_hidden(mid) {
                    if let Some(mi) = scene.member_index(mid) {
                        if let Ok((xh, yh, zh)) = result.member_local_axes(mi) {
                            if let Some(mpos) = scene.member_midpoint(mi) {
                                if cut.keeps(mpos) {
                                    draw_local_axes(
                                        &app.camera,
                                        w,
                                        h,
                                        rect,
                                        &painter,
                                        mpos,
                                        [xh, yh, zh],
                                        0.5,
                                        pal,
                                    );
                                }
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
                if !cut.keeps(pos) {
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
                    pal.label,
                );
            }
        }

        // ── Help text ──
        painter.text(
            rect.min + egui::vec2(10.0, rect.height() - 10.0),
            egui::Align2::LEFT_BOTTOM,
            format!(
                "Drag: {} | Shift-drag: box select | R-drag: pan | Scroll: zoom | R: reset | F: fit | 1-4: views | Click: select | Shift/Ctrl-click: add | Ctrl+A: all | Del: delete | Dbl-click: edit | R-click: menu | Esc: clear | Alt-drag: cut",
                drag_hint(app.view_mode)
            ),
            egui::FontId::proportional(11.0),
            pal.help,
        );

        // ── Box-select rubber band (drawn over the scene) ──
        if let Some(start) = app.box_select_start {
            if response.dragged_by(egui::PointerButton::Primary) {
                if let Some(cur) = response.interact_pointer_pos() {
                    let band = egui::Rect::from_two_pos(start, cur);
                    painter.rect_filled(band, 0.0, pal.band_fill);
                    painter.rect_stroke(band, 0.0, egui::Stroke::new(1.0_f32, pal.band_stroke));
                }
            }
        }

        // ── Section-cut plane indicator (both renderers) ──
        draw_section_cut_plane(app, scene, rect, &painter, w, h, pal);

        // ── View cube (bottom-right) ──
        show_view_cube(app, ui, rect, &painter, pal);
    });
}

// ── Helpers ─────────────────────────────────────────────────────────────

fn scr(p: &ScreenPos, rect: egui::Rect) -> egui::Pos2 {
    egui::pos2(rect.min.x + p.x as f32, rect.min.y + p.y as f32)
}

fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l.is_finite() && l > 1e-12 {
        [v[0] / l, v[1] / l, v[2] / l]
    } else {
        [0.0, 0.0, 0.0]
    }
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// In-plane orthonormal basis `(u, v)` for a plane with unit normal `n`.
fn plane_basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let reference = if n[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let u = normalize3(cross3(n, reference));
    let v = cross3(n, u);
    (u, v)
}

/// Draw a translucent indicator for the active section-cut plane.
///
/// A painter overlay, so it works in both renderers: a quad spanning the
/// model bounding box, centred on the point of the plane nearest the model
/// centre.  Skipped when the cut is off or any corner projects behind the
/// camera.
fn draw_section_cut_plane(
    app: &ViewportApp,
    scene: &ViewportScene,
    rect: egui::Rect,
    painter: &egui::Painter,
    w: f64,
    h: f64,
    pal: ViewportPalette,
) {
    let cut = &app.display.section_cut;
    if !cut.enabled {
        return;
    }
    let Some(n) = cut.unit_normal() else {
        return;
    };
    let (bmin, bmax) = scene.bounding_box();
    let center = [
        (bmin[0] + bmax[0]) * 0.5,
        (bmin[1] + bmax[1]) * 0.5,
        (bmin[2] + bmax[2]) * 0.5,
    ];
    // Nearest point of the plane to the model centre.
    let sd = n[0] * center[0] + n[1] * center[1] + n[2] * center[2] - cut.offset;
    let pc = [
        center[0] - sd * n[0],
        center[1] - sd * n[1],
        center[2] - sd * n[2],
    ];
    let (u, v) = plane_basis(n);
    let extent = ((bmax[0] - bmin[0])
        .hypot(bmax[1] - bmin[1])
        .hypot(bmax[2] - bmin[2]))
    .max(1.0)
        * 0.6;
    let corners = [
        [
            pc[0] - u[0] * extent - v[0] * extent,
            pc[1] - u[1] * extent - v[1] * extent,
            pc[2] - u[2] * extent - v[2] * extent,
        ],
        [
            pc[0] + u[0] * extent - v[0] * extent,
            pc[1] + u[1] * extent - v[1] * extent,
            pc[2] + u[2] * extent - v[2] * extent,
        ],
        [
            pc[0] + u[0] * extent + v[0] * extent,
            pc[1] + u[1] * extent + v[1] * extent,
            pc[2] + u[2] * extent + v[2] * extent,
        ],
        [
            pc[0] - u[0] * extent + v[0] * extent,
            pc[1] - u[1] * extent + v[1] * extent,
            pc[2] - u[2] * extent + v[2] * extent,
        ],
    ];
    let mut pts = Vec::with_capacity(4);
    for c in corners {
        let Some(p) = app.camera.project(c, w, h) else {
            return;
        };
        pts.push(scr(&p, rect));
    }
    painter.add(egui::Shape::convex_polygon(
        pts,
        pal.plane_fill,
        egui::Stroke::new(1.0_f32, pal.plane_stroke),
    ));
}

/// Alt + primary-drag: slide the section-cut plane along its normal.
///
/// The offset advances by the component of the drag along the screen-space
/// direction of the normal, converted to world units at the plane's depth
/// (first-order — accurate enough for a drag).  Clamped to the scene range.
fn drag_section_cut(app: &mut ViewportApp, delta: egui::Vec2, w: f64, h: f64) {
    let Some(n) = app.display.section_cut.unit_normal() else {
        return;
    };
    let offset = app.display.section_cut.offset;
    let base = [n[0] * offset, n[1] * offset, n[2] * offset];
    let tip = [base[0] + n[0], base[1] + n[1], base[2] + n[2]];
    let (Some(p0), Some(p1)) = (
        app.camera.project(base, w, h),
        app.camera.project(tip, w, h),
    ) else {
        return;
    };
    let sx = p1.x - p0.x;
    let sy = p1.y - p0.y;
    let len = (sx * sx + sy * sy).sqrt();
    if !len.is_finite() || len < 1e-6 {
        return;
    }
    // `len` screen pixels corresponds to one world unit at the plane's depth.
    let amount = (delta.x as f64 * sx / len + delta.y as f64 * sy / len) / len;
    if !amount.is_finite() {
        return;
    }
    app.display.section_cut.offset += amount;
    app.sync_section_cut();
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
            let target = SelectionState::Node(id);
            let multi = app.selected.contains_state(target) && app.selected.len() > 1;
            let del_label = if multi {
                format!("Delete selection ({})", app.selected.len())
            } else {
                "Delete node".to_string()
            };
            if ui.button(del_label).clicked() {
                app.delete_target_or_selection(target);
                ui.close_menu();
            }
            let hide_label = if multi { "Hide selection" } else { "Hide node" };
            if ui.button(hide_label).clicked() {
                app.hide_target_or_selection(target);
                ui.close_menu();
            }
            if ui.button("Hide others").clicked() {
                app.select_only(target);
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
            let target = SelectionState::Member(id);
            let multi = app.selected.contains_state(target) && app.selected.len() > 1;
            let del_label = if multi {
                format!("Delete selection ({})", app.selected.len())
            } else {
                "Delete member".to_string()
            };
            if ui.button(del_label).clicked() {
                app.delete_target_or_selection(target);
                ui.close_menu();
            }
            let hide_label = if multi { "Hide selection" } else { "Hide member" };
            if ui.button(hide_label).clicked() {
                app.hide_target_or_selection(target);
                ui.close_menu();
            }
            if ui.button("Hide others").clicked() {
                app.select_only(target);
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
    pal: ViewportPalette,
) {
    for (end, col, lbl) in [
        ([len, 0.0, 0.0], pal.axis_x, "X"),
        ([0.0, len, 0.0], pal.axis_y, "Y"),
        ([0.0, 0.0, len], pal.axis_z, "Z"),
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

fn draw_axes(
    cam: &Camera3D,
    w: f64,
    h: f64,
    rect: egui::Rect,
    painter: &egui::Painter,
    len: f64,
    pal: ViewportPalette,
) {
    let origin = [0.0, 0.0, 0.0];
    let Some(p0) = cam.project(origin, w, h) else {
        return;
    };
    for (end, col, lbl) in [
        ([len, 0.0, 0.0], pal.axis_x, "X"),
        ([0.0, len, 0.0], pal.axis_y, "Y"),
        ([0.0, 0.0, len], pal.axis_z, "Z"),
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
    pal: ViewportPalette,
) {
    let Some(p0) = cam.project(origin, w, h) else {
        return;
    };
    for (axis, col, lbl) in [
        (axes[0], pal.axis_x, "x"),
        (axes[1], pal.axis_y, "y"),
        (axes[2], pal.axis_z, "z"),
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

/// One mid-edge snap of the view cube (Phase P7-2).
///
/// `a` / `b` are corner indices into [`cube_corner`] (bit 0 = x, bit 1 = y,
/// bit 2 = z, set = +1), so an edge always joins two corners that differ in
/// exactly one bit.  Clicking the edge snaps to the 45° two-axis `view`.
struct CubeEdge {
    a: usize,
    b: usize,
    view: StandardView,
}

/// The twelve cube edges and the mid-edge view each one snaps to.
const CUBE_EDGES: [CubeEdge; 12] = [
    CubeEdge { a: 6, b: 7, view: StandardView::FrontTop },
    CubeEdge { a: 4, b: 5, view: StandardView::FrontBottom },
    CubeEdge { a: 5, b: 7, view: StandardView::FrontRight },
    CubeEdge { a: 4, b: 6, view: StandardView::FrontLeft },
    CubeEdge { a: 2, b: 3, view: StandardView::BackTop },
    CubeEdge { a: 0, b: 1, view: StandardView::BackBottom },
    CubeEdge { a: 1, b: 3, view: StandardView::BackRight },
    CubeEdge { a: 0, b: 2, view: StandardView::BackLeft },
    CubeEdge { a: 3, b: 7, view: StandardView::TopRight },
    CubeEdge { a: 2, b: 6, view: StandardView::TopLeft },
    CubeEdge { a: 1, b: 5, view: StandardView::BottomRight },
    CubeEdge { a: 0, b: 4, view: StandardView::BottomLeft },
];

/// Corner `i` of the unit cube (`[-1, 1]³`); bit layout documented on
/// [`CubeEdge`].
fn cube_corner(i: usize) -> [f64; 3] {
    [
        if i & 1 == 0 { -1.0 } else { 1.0 },
        if i & 2 == 0 { -1.0 } else { 1.0 },
        if i & 4 == 0 { -1.0 } else { 1.0 },
    ]
}

/// Draw the view cube and handle its interactions.
///
/// The cube is drawn with the camera's own basis (a small orthographic
/// projection of the world axes), so it rotates with the view.  Clicking a
/// visible face snaps to that standard view; clicking a **mid-edge**
/// (Phase P7-2) snaps to the edge's 45° two-axis view; clicking a corner
/// snaps to `Iso`; dragging the gizmo orbits the camera.  Reuses
/// [`Camera3D::set_view`] / [`ViewportApp::set_standard_view`], so it never
/// bypasses the existing view machinery.
fn show_view_cube(
    app: &mut ViewportApp,
    ui: &mut egui::Ui,
    rect: egui::Rect,
    painter: &egui::Painter,
    pal: ViewportPalette,
) {
    let region = view_cube_rect(rect);
    if !rect.contains_rect(region) {
        return;
    }

    painter.rect_filled(region, 4.0, pal.cube_panel);
    painter.rect_stroke(region, 4.0, egui::Stroke::new(1.0_f32, pal.cube_border));

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
        let fill = egui::Color32::from_rgb(
            shade(pal.cube_face.r()),
            shade(pal.cube_face.g()),
            shade(pal.cube_face.b()),
        );
        painter.add(egui::Shape::convex_polygon(
            pts.to_vec(),
            fill,
            egui::Stroke::new(1.0_f32, pal.cube_border),
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
            pal.cube_text,
        );
        face_polys.push((face.view, pts));
    }

    // The eight cube corners map to the isometric view.
    let corners: Vec<egui::Pos2> = (0..8).map(|i| project_dir(cube_corner(i))).collect();

    // Mid-edge snap zones (Phase P7-2): a visible edge is drawn with a faint
    // highlight so the 45° two-axis zones read as clickable, and is hit-tested
    // with a screen-space band around its projected segment.
    let mut edge_segments: Vec<(StandardView, egui::Pos2, egui::Pos2)> = Vec::with_capacity(12);
    for edge in &CUBE_EDGES {
        let a = cube_corner(edge.a);
        let b = cube_corner(edge.b);
        // Visible when the edge midpoint faces the camera (both adjacent
        // faces contribute; at the exact silhouette either answer is fine).
        let mid = [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
        if project::dot(mid, eye_dir) <= 0.0 {
            continue;
        }
        let (pa, pb) = (project_dir(a), project_dir(b));
        painter.line_segment(
            [pa, pb],
            egui::Stroke::new(2.5_f32, pal.cube_edge),
        );
        edge_segments.push((edge.view, pa, pb));
    }

    let response = ui.interact(region, ui.id().with("view_cube"), egui::Sense::click_and_drag());
    if response.dragged() {
        let d = response.drag_delta();
        app.camera.orbit(-d.x as f64 * 0.02, d.y as f64 * 0.02);
    } else if response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some(view) = cube_pick(pos, &corners, &edge_segments, &face_polys) {
                app.set_standard_view(view);
            }
        }
    }
    // Hover names the snap target under the cursor.
    if response.hovered() {
        if let Some(view) = response
            .interact_pointer_pos()
            .and_then(|pos| cube_pick(pos, &corners, &edge_segments, &face_polys))
        {
            response.on_hover_text(format!("{} view — click to snap", view.label()));
        }
    }
}

/// Resolve a position on the view cube to the view it snaps to.
///
/// Priority: a corner zone snaps to `Iso`, then a mid-edge band snaps to the
/// edge's 45° two-axis view, then a face interior snaps to the face view.
/// Returns `None` when `pos` misses every zone.
fn cube_pick(
    pos: egui::Pos2,
    corners: &[egui::Pos2],
    edge_segments: &[(StandardView, egui::Pos2, egui::Pos2)],
    face_polys: &[(StandardView, [egui::Pos2; 4])],
) -> Option<StandardView> {
    if corners.iter().any(|c| c.distance(pos) < CUBE_CORNER_SNAP_PX) {
        return Some(StandardView::Iso);
    }
    for (view, a, b) in edge_segments {
        if dist_to_segment(pos, *a, *b) <= CUBE_EDGE_SNAP_PX {
            return Some(*view);
        }
    }
    for (view, poly) in face_polys {
        if point_in_poly(pos, poly) {
            return Some(*view);
        }
    }
    None
}

/// Screen-space distance (points) from `p` to the segment `a`–`b`.
fn dist_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let len2 = ab.length_sq();
    let t = if len2 <= 1e-12 {
        0.0
    } else {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    };
    (a + ab * t - p).length()
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

/// One-word drag description for the viewport help line, per [`ViewMode`].
fn drag_hint(mode: ViewMode) -> &'static str {
    match mode {
        ViewMode::Orbit => "orbit",
        ViewMode::Pan => "pan",
        ViewMode::Zoom => "zoom",
    }
}

// ── Boundary quick-add popups (ribbon group) ────────────────────────────

/// Draw the Boundary quick-add popup, if one is open.
///
/// Mirrors the double-click quick-edit popup: a small non-collapsible window
/// anchored near the cursor.  **Support** / **Node Load** target the primary
/// selected node, **Mem Load** the primary selected member.  Apply delegates
/// to the shared [`ViewportApp`] edit paths (`apply_support_quick`,
/// `apply_nodal_load_quick`, `apply_member_load_quick`), so every popup write
/// is a single undoable [`commit_edit`](ViewportApp::commit_edit) — no edit
/// mechanics are duplicated here.
///
/// Closing: Escape, the title-bar ✕, the Close button, a successful Apply,
/// or a viewport click.  A failed Apply (invalid input) keeps the popup open
/// and shows the error both inside the popup and in the Messages dock.
pub fn show_boundary_popup(app: &mut ViewportApp, ctx: &egui::Context) {
    let Some(mut pop) = app.boundary_popup.take() else {
        return;
    };
    let mut open = true;
    let mut close = false;
    let mut apply = false;

    egui::Window::new(boundary_title(&pop))
        .id(egui::Id::new("boundary_popup"))
        .collapsible(false)
        .resizable(false)
        .open(&mut open)
        .default_pos(pop.anchor() + egui::vec2(14.0, 14.0))
        .show(ctx, |ui| {
            boundary_body(app, ui, &mut pop, &mut apply, &mut close)
        });

    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        close = true;
    }
    if !open {
        close = true;
    }

    if apply {
        let result = match &pop {
            BoundaryPopup::Support { node, spec, .. } => app.apply_support_quick(*node, *spec),
            BoundaryPopup::NodeLoad { node, load, .. } => app.apply_nodal_load_quick(*node, *load),
            BoundaryPopup::MemLoad {
                member,
                qx,
                qy,
                qz,
                add_point,
                xi,
                fx,
                fy,
                fz,
                ..
            } => {
                let point = if *add_point {
                    Some(MemberPointLoad3D::new(*xi, *fx, *fy, *fz))
                } else {
                    None
                };
                app.apply_member_load_quick(*member, *qx, *qy, *qz, point)
            }
        };
        match result {
            Ok(()) => close = true,
            Err(msg) => pop.set_error(msg),
        }
    }
    if !close {
        app.boundary_popup = Some(pop);
    }
}

/// Popup window title naming the target entity.
fn boundary_title(pop: &BoundaryPopup) -> String {
    match pop {
        BoundaryPopup::Support { node, .. } => format!("Add support \u{2014} Node N{}", node.0),
        BoundaryPopup::NodeLoad { node, .. } => {
            format!("Add nodal load \u{2014} Node N{}", node.0)
        }
        BoundaryPopup::MemLoad { member, .. } => {
            format!("Add member load \u{2014} Member M{}", member.0)
        }
    }
}

/// Popup contents: the per-kind editors plus Apply / Close.
fn boundary_body(
    app: &mut ViewportApp,
    ui: &mut egui::Ui,
    pop: &mut BoundaryPopup,
    apply: &mut bool,
    close: &mut bool,
) {
    let text = TextPalette::for_theme(app.dark_mode);
    if let Some(err) = pop.error() {
        ui.label(egui::RichText::new(err).color(text.error));
    }
    match pop {
        BoundaryPopup::Support { spec, .. } => {
            ui.label("Support type:");
            ui.horizontal(|ui| {
                for (label, preset) in [
                    ("Fixed", SupportSpec::fixed as fn() -> SupportSpec),
                    ("Pinned", SupportSpec::pinned),
                    ("Custom", SupportSpec::free),
                ] {
                    if ui.selectable_label(spec.label() == label, label).clicked() {
                        *spec = preset();
                    }
                }
            });
            ui.label("DOFs (checked = constrained to 0):");
            for dof in Dof3D::ALL {
                let mut constrained = spec.dof(dof).is_some();
                if ui.checkbox(&mut constrained, dof.name()).changed() {
                    spec.set_dof(dof, if constrained { Some(0.0) } else { None });
                }
            }
        }
        BoundaryPopup::NodeLoad { load, .. } => {
            ui.label("Nodal load (global coords, N and N\u{00b7}m):");
            egui::Grid::new("boundary_nodal_load_grid")
                .num_columns(2)
                .spacing([8.0, 3.0])
                .show(ui, |ui| {
                    ui.strong("Force (N)");
                    ui.strong("Moment (N\u{00b7}m)");
                    ui.end_row();
                    for (axis, f, m) in [
                        ("x", &mut load.fx, &mut load.mx),
                        ("y", &mut load.fy, &mut load.my),
                        ("z", &mut load.fz, &mut load.mz),
                    ] {
                        ui.label(format!("{axis}:"));
                        ui.add(egui::DragValue::new(f).speed(100.0).range(-1e12..=1e12));
                        ui.label(format!("{axis}:"));
                        ui.add(egui::DragValue::new(m).speed(100.0).range(-1e12..=1e12));
                        ui.end_row();
                    }
                });
        }
        BoundaryPopup::MemLoad {
            qx,
            qy,
            qz,
            add_point,
            xi,
            fx,
            fy,
            fz,
            ..
        } => {
            ui.label("Uniform load (local coords, N/m):");
            egui::Grid::new("boundary_member_load_grid")
                .num_columns(2)
                .spacing([8.0, 3.0])
                .show(ui, |ui| {
                    for (label, value) in [
                        ("qx (axial):", qx),
                        ("qy (transverse):", qy),
                        ("qz (transverse):", qz),
                    ] {
                        ui.label(label);
                        ui.add(egui::DragValue::new(value).speed(100.0).range(-1e8..=1e8));
                        ui.end_row();
                    }
                });
            ui.checkbox(add_point, "Add a point load");
            if *add_point {
                ui.label("Point load (local coords, N):");
                egui::Grid::new("boundary_point_load_grid")
                    .num_columns(2)
                    .spacing([8.0, 3.0])
                    .show(ui, |ui| {
                        ui.label("xi (position):");
                        ui.add(
                            egui::DragValue::new(xi)
                                .speed(0.05)
                                .range(0.0..=1.0)
                                .fixed_decimals(2),
                        );
                        ui.end_row();
                        ui.label("fx (axial):");
                        ui.add(egui::DragValue::new(fx).speed(100.0).range(-1e8..=1e8));
                        ui.end_row();
                        ui.label("fy (transverse):");
                        ui.add(egui::DragValue::new(fy).speed(100.0).range(-1e8..=1e8));
                        ui.end_row();
                        ui.label("fz (transverse):");
                        ui.add(egui::DragValue::new(fz).speed(100.0).range(-1e8..=1e8));
                        ui.end_row();
                    });
            }
        }
    }
    ui.separator();
    ui.horizontal(|ui| {
        if ui.button("Apply").clicked() {
            *apply = true;
        }
        if ui.button("Close").clicked() {
            *close = true;
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The edge table is well formed: twelve distinct mid-edge views, each
    /// joining two cube corners that differ in exactly one coordinate.
    #[test]
    fn cube_edge_table_is_well_formed() {
        assert_eq!(CUBE_EDGES.len(), 12);
        let mut views: Vec<StandardView> = Vec::new();
        for e in &CUBE_EDGES {
            assert!(e.a < 8 && e.b < 8, "corner index out of range");
            let a = cube_corner(e.a);
            let b = cube_corner(e.b);
            let diffs = (0..3).filter(|&k| (a[k] - b[k]).abs() > 1e-12).count();
            assert_eq!(diffs, 1, "corners {}–{} are not a cube edge", e.a, e.b);
            assert!(
                StandardView::EDGES.contains(&e.view),
                "{:?} is not a mid-edge view",
                e.view
            );
            assert!(!views.contains(&e.view), "duplicate edge view {:?}", e.view);
            views.push(e.view);
        }
        assert_eq!(views.len(), StandardView::EDGES.len());
    }

    /// Each edge's view direction matches the axes its corners share: the two
    /// fixed coordinates carry the view's signs, the varying one is zero.
    #[test]
    fn cube_edge_views_match_their_fixed_corner_axes() {
        for e in &CUBE_EDGES {
            let a = cube_corner(e.a);
            let b = cube_corner(e.b);
            let d = e.view.eye_dir();
            for k in 0..3 {
                if (a[k] - b[k]).abs() > 1e-12 {
                    assert!(
                        d[k].abs() < 1e-9,
                        "{:?}: varying axis {k} should be the zero axis",
                        e.view
                    );
                } else {
                    assert!(
                        d[k].abs() > 0.5,
                        "{:?}: fixed axis {k} should dominate",
                        e.view
                    );
                    assert_eq!(
                        d[k] > 0.0,
                        a[k] > 0.0,
                        "{:?}: sign mismatch on axis {k}",
                        e.view
                    );
                }
            }
        }
    }

    /// `cube_pick` resolves corner → edge → face in that priority order.
    #[test]
    fn cube_pick_prefers_corner_then_edge_then_face() {
        let corners: Vec<egui::Pos2> = (0..8)
            .map(|i| {
                let c = cube_corner(i);
                egui::pos2(50.0 + c[0] as f32 * 50.0, 50.0 + c[1] as f32 * 50.0)
            })
            .collect();
        let edge = StandardView::FrontTop;
        let edges = vec![(edge, egui::pos2(0.0, 50.0), egui::pos2(100.0, 50.0))];
        let face = StandardView::Front;
        let faces = vec![(
            face,
            [
                egui::pos2(0.0, 0.0),
                egui::pos2(100.0, 0.0),
                egui::pos2(100.0, 100.0),
                egui::pos2(0.0, 100.0),
            ],
        )];
        // Near a corner → Iso, even though the point is also inside the edge band.
        assert_eq!(
            cube_pick(egui::pos2(2.0, 2.0), &corners, &edges, &faces),
            Some(StandardView::Iso)
        );
        // On the mid-edge band → the edge view.
        assert_eq!(
            cube_pick(egui::pos2(50.0, 50.0), &corners, &edges, &faces),
            Some(edge)
        );
        // Face interior, away from corners and edges → the face view.
        assert_eq!(
            cube_pick(egui::pos2(50.0, 90.0), &corners, &edges, &faces),
            Some(face)
        );
        // Outside every zone → nothing.
        assert_eq!(
            cube_pick(egui::pos2(500.0, 500.0), &corners, &edges, &faces),
            None
        );
    }
}
