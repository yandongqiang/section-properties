//! Left-hand hierarchical model browser (MIDAS-style tree).
//!
//! The tree is the primary left panel in every module.  Clicking a leaf
//! selects the corresponding entity through the existing
//! [`SelectionState`](viewport_poc::selection::SelectionState).

use egui;

use structural_analysis::FrameAnalysisResult3D;
use viewport_poc::display::{displacement_magnitudes, fmt_eng};
use viewport_poc::edit_model::EditModel;
use viewport_poc::selection::{SelectionSet, SelectionState};

use super::layout;
use crate::ViewportApp;

/// Smallest the left panel may be dragged.  Keeps the resize handle grabbable.
const TREE_MIN_WIDTH: f32 = 180.0;
/// Ceiling for the default left-panel width (points).
const TREE_DEFAULT_MAX: f32 = 340.0;
/// Floor for the default left-panel width (points).
const TREE_DEFAULT_MIN: f32 = 220.0;
/// Default left-panel width as a fraction of the window width.
const TREE_DEFAULT_FRACTION: f32 = 0.2;
/// The left panel may never exceed this fraction of the window width, so it can
/// never swallow the viewport.
const TREE_MAX_FRACTION: f32 = 0.45;

/// Draw the left model-browser panel.
pub fn show_model_tree(app: &mut ViewportApp, ctx: &egui::Context) {
    let window_w = ctx.screen_rect().width();
    let max_w = (window_w * TREE_MAX_FRACTION).max(TREE_DEFAULT_MAX);
    // Phase P7-3 C2: a persisted layout preset overrides the window-fraction
    // default width (egui still clamps it into the panel's allowed range).
    let default_w = layout::restored_tree_width(ctx)
        .unwrap_or_else(|| (window_w * TREE_DEFAULT_FRACTION).clamp(TREE_DEFAULT_MIN, TREE_DEFAULT_MAX));

    egui::SidePanel::left("model_tree")
        .resizable(true)
        .min_width(TREE_MIN_WIDTH)
        .max_width(max_w)
        .default_width(default_w)
        .show(ctx, |ui| {
            ui.heading("Model Browser");
            ui.separator();

            let selection = app.selected.clone();
            let result = app.analysis_result();
            let mut clicked: Option<SelectionState> = None;

            if let Some(model) = app.project.model.as_ref() {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        clicked = render_tree(ui, model, result, &selection);
                    });
            } else {
                ui.label("No model loaded");
            }

            // Phase P6: modifier-aware tree clicks, matching the viewport
            // (plain = replace, Shift = toggle, Ctrl/Cmd = add).
            if let Some(sel) = clicked {
                let mods = ctx.input(|i| i.modifiers);
                if mods.shift {
                    app.select_toggle(sel);
                } else if mods.command {
                    app.select_add(sel);
                } else {
                    app.select_only(sel);
                }
            }

            ui.separator();
            project_file_section(app, ui);

            // Publish the live panel width for layout-preset persistence
            // (Phase P7-3 C2).  `ui.max_rect()` is the panel's *inner* rect;
            // add the frame's inner margin back to recover the outer width.
            let margin = egui::Frame::side_top_panel(ui.style()).inner_margin;
            layout::publish_tree_width(
                ctx,
                ui.max_rect().width() + (margin.left + margin.right) as f32,
            );
        });
}

/// Render all tree branches, returning any newly requested selection.
fn render_tree(
    ui: &mut egui::Ui,
    model: &EditModel,
    result: Option<&FrameAnalysisResult3D>,
    selection: &SelectionSet,
) -> Option<SelectionState> {
    let mut clicked: Option<SelectionState> = None;

    egui::CollapsingHeader::new("Model")
        .default_open(true)
        .show(ui, |ui| {
            nodes_branch(ui, model, selection, &mut clicked);
            members_branch(ui, model, selection, &mut clicked);
            materials_branch(ui, model, selection, &mut clicked);
            sections_branch(ui, model, selection, &mut clicked);
            supports_branch(ui, model, selection, &mut clicked);
            loads_branch(ui, model, selection, &mut clicked);
            results_branch(ui, model, result);
        });

    clicked
}

fn nodes_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    egui::CollapsingHeader::new(format!("Nodes ({})", model.n_nodes()))
        .default_open(true)
        .show(ui, |ui| {
            for node in model.nodes() {
                let is_sel = selection.contains_node(node.id);
                let sup_label = match model.support(node.id) {
                    Some(spec) => format!(" [{}]", spec.label()),
                    None => String::new(),
                };
                let load_label = match model.nodal_load(node.id) {
                    Some(spec) if !spec.is_zero() => " [L]".to_string(),
                    _ => String::new(),
                };
                let text = format!(
                    "N{} ({:.1}, {:.1}, {:.1}){}{}",
                    node.id.0, node.x, node.y, node.z, sup_label, load_label
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Node(node.id));
                }
            }
        });
}

fn members_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    egui::CollapsingHeader::new(format!("Members ({})", model.n_members()))
        .default_open(true)
        .show(ui, |ui| {
            for member in model.members() {
                let is_sel = selection.contains_member(member.id);
                let load_label = match model.member_uniform_load(member.id) {
                    Some(spec) if !spec.is_zero() => " [L]".to_string(),
                    _ => String::new(),
                };
                let point_label = if model.has_member_point_loads(member.id) {
                    " [P]".to_string()
                } else {
                    String::new()
                };
                let text = format!(
                    "M{}: N{} \u{2192} N{}{}{}",
                    member.id.0, member.node_i.0, member.node_j.0, load_label, point_label
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Member(member.id));
                }
            }
        });
}

fn materials_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    egui::CollapsingHeader::new(format!("Materials ({})", model.n_members()))
        .default_open(false)
        .show(ui, |ui| {
            for member in model.members() {
                let Some(sec) = model.member_section(member.id) else {
                    continue;
                };
                let is_sel = selection.contains_member(member.id);
                let text = format!(
                    "M{}: E={:.3e} G={:.3e}",
                    member.id.0,
                    sec.E(),
                    sec.G()
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Member(member.id));
                }
            }
        });
}

fn sections_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    egui::CollapsingHeader::new(format!("Sections ({})", model.n_members()))
        .default_open(false)
        .show(ui, |ui| {
            for member in model.members() {
                let Some(sec) = model.member_section(member.id) else {
                    continue;
                };
                let is_sel = selection.contains_member(member.id);
                let text = format!(
                    "M{}: A={:.3e} Iy={:.3e} Iz={:.3e} J={:.3e}",
                    member.id.0,
                    sec.area(),
                    sec.iy(),
                    sec.iz(),
                    sec.j()
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Member(member.id));
                }
            }
        });
}

fn supports_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    let supported: Vec<_> = model
        .nodes()
        .iter()
        .filter_map(|n| model.support(n.id).map(|s| (n.id, s.label())))
        .collect();
    egui::CollapsingHeader::new(format!("Supports ({})", supported.len()))
        .default_open(false)
        .show(ui, |ui| {
            for (id, label) in supported {
                let is_sel = selection.contains_node(id);
                if ui
                    .selectable_label(is_sel, format!("N{}: {}", id.0, label))
                    .clicked()
                {
                    *clicked = Some(SelectionState::Node(id));
                }
            }
        });
}

fn loads_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
    clicked: &mut Option<SelectionState>,
) {
    let nodal: Vec<_> = model
        .nodes()
        .iter()
        .filter_map(|n| {
            model
                .nodal_load(n.id)
                .filter(|s| !s.is_zero())
                .map(|s| (n.id, *s))
        })
        .collect();
    let uniform: Vec<_> = model
        .members()
        .iter()
        .filter_map(|m| {
            model
                .member_uniform_load(m.id)
                .filter(|s| !s.is_zero())
                .map(|s| (m.id, *s))
        })
        .collect();
    let points: Vec<_> = model
        .members()
        .iter()
        .filter_map(|m| {
            let loads = model.member_point_loads_for(m.id);
            if loads.is_empty() {
                None
            } else {
                Some((m.id, loads))
            }
        })
        .collect();
    let total = nodal.len() + uniform.len() + points.iter().map(|(_, l)| l.len()).sum::<usize>();

    egui::CollapsingHeader::new(format!("Loads ({total})"))
        .default_open(false)
        .show(ui, |ui| {
            for (id, spec) in &nodal {
                let is_sel = selection.contains_node(*id);
                let text = format!(
                    "N{}: F=({:.3e}, {:.3e}, {:.3e}) M=({:.3e}, {:.3e}, {:.3e})",
                    id.0, spec.fx, spec.fy, spec.fz, spec.mx, spec.my, spec.mz
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Node(*id));
                }
            }
            for (id, spec) in &uniform {
                let is_sel = selection.contains_member(*id);
                let text = format!(
                    "M{}: q=({:.3e}, {:.3e}, {:.3e})",
                    id.0, spec.qx, spec.qy, spec.qz
                );
                if ui.selectable_label(is_sel, text).clicked() {
                    *clicked = Some(SelectionState::Member(*id));
                }
            }
            for (id, loads) in &points {
                let is_sel = selection.contains_member(*id);
                for pl in loads {
                    let text = format!(
                        "M{}: P @ {:.2} = ({:.3e}, {:.3e}, {:.3e})",
                        id.0, pl.xi, pl.fx, pl.fy, pl.fz
                    );
                    if ui.selectable_label(is_sel, text).clicked() {
                        *clicked = Some(SelectionState::Member(*id));
                    }
                }
            }
        });
}

fn results_branch(
    ui: &mut egui::Ui,
    model: &EditModel,
    result: Option<&FrameAnalysisResult3D>,
) {
    egui::CollapsingHeader::new("Results")
        .default_open(false)
        .show(ui, |ui| {
            let Some(result) = result else {
                ui.label("(no current result \u{2014} solve first)");
                return;
            };
            let mags = displacement_magnitudes(&result.displacements, model.n_nodes());
            ui.label("Nodal |u|:");
            for (i, node) in model.nodes().iter().enumerate() {
                let mag = mags.get(i).copied().unwrap_or(0.0);
                ui.label(format!("N{}: {}", node.id.0, fmt_eng(mag)));
            }
            ui.separator();
            ui.label("Member end forces:");
            for (i, member) in model.members().iter().enumerate() {
                let peak = result
                    .member_end_forces_global(i)
                    .ok()
                    .map(|f| f.iter().fold(0.0_f64, |a, b| a.max(b.abs())))
                    .unwrap_or(0.0);
                ui.label(format!("M{}: |F|max = {}", member.id.0, fmt_eng(peak)));
            }
        });
}

/// File path / open / save controls, kept at the bottom of the browser.
fn project_file_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = super::theme::TextPalette::for_theme(app.dark_mode);
    egui::CollapsingHeader::new("Project file")
        .default_open(false)
        .show(ui, |ui| {
            ui.label("File path:");
            ui.text_edit_singleline(&mut app.file_path_input);
            ui.horizontal(|ui| {
                if ui.button("Open").clicked() {
                    app.request_open();
                }
                if ui.button("Save").clicked() {
                    app.do_save();
                }
                if ui.button("Save As").clicked() {
                    app.do_save_as();
                }
            });
            if app.is_dirty() {
                ui.label(egui::RichText::new("\u{26a0} Unsaved changes").color(text.stale));
            } else if app.project_path.is_some() {
                ui.label(egui::RichText::new("\u{2713} Saved").color(text.ok));
            }
        });
}
