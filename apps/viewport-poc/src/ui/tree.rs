//! Left-hand **Project** panel (Zed-like).
//!
//! The panel header is "Project"; under it sits the "Project file" path
//! section (open / save) and the **Model Explorer**, whose groups are
//! `Nodes`, `Members`, `Supports` and `Loads`.  Clicking a leaf selects the
//! corresponding entity through the authoritative
//! [`SelectionSet`](viewport_poc::selection::SelectionSet).
//!
//! The panel is collapsible; its width is preserved when reopened through the
//! shared layout-preset mechanism (Phase P7-3 / 159-B).

use egui;

use viewport_poc::edit_model::EditModel;
use viewport_poc::selection::{SelectionSet, SelectionState};

use super::icons;
use super::layout;
use super::theme;
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

/// Draw the left Project panel.
pub fn show_model_tree(app: &mut ViewportApp, ctx: &egui::Context) {
    // A collapsed panel is not created at all, so the viewport reclaims its
    // space with no residual gap (matching the right / bottom panels).
    if app.left_panel_collapsed {
        return;
    }
    let window_w = ctx.screen_rect().width();
    let max_w = (window_w * TREE_MAX_FRACTION).max(TREE_DEFAULT_MAX);
    // Phase P7-3 C2: a persisted layout preset overrides the window-fraction
    // default width (egui still clamps it into the panel's allowed range).
    let default_w = layout::restored_tree_width(ctx).unwrap_or_else(|| {
        (window_w * TREE_DEFAULT_FRACTION).clamp(TREE_DEFAULT_MIN, TREE_DEFAULT_MAX)
    });

    egui::SidePanel::left("model_tree")
        .resizable(true)
        .min_width(TREE_MIN_WIDTH)
        .max_width(max_w)
        .default_width(default_w)
        .show(ctx, |ui| {
            // Compact panel header: leading glyph + label + trailing Hide.
            theme::panel_header(ui, icons::panel_left, "Project", |ui| {
                if ui
                    .button("\u{25c0} Hide")
                    .on_hover_text("Hide the Project panel")
                    .clicked()
                {
                    app.left_panel_collapsed = true;
                }
            });
            ui.separator();

            // File controls are drawn *above* the scrolling tree so they are
            // always visible: the tree's ScrollArea fills the remaining panel
            // height (egui `auto_shrink([false, false])`), which would otherwise
            // push this section below the fold and make it unreachable.
            project_file_section(app, ui);
            ui.separator();
            ui.label(theme::header("Model Explorer"));

            let selection = app.selected.clone();
            let mut clicked: Option<SelectionState> = None;

            if let Some(model) = app.project.model.as_ref() {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        clicked = render_tree(ui, model, &selection);
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

/// Render the Model Explorer groups, returning any newly requested selection.
///
/// Groups are `Nodes`, `Members`, `Supports` and `Loads` (Phase 159-B, B3).
/// The former Materials / Sections / Results branches are gone: their content
/// is available in the Inspector (Section / Results modules), the Outline mode
/// and the bottom Results tab, so no capability is lost.
fn render_tree(
    ui: &mut egui::Ui,
    model: &EditModel,
    selection: &SelectionSet,
) -> Option<SelectionState> {
    let mut clicked: Option<SelectionState> = None;

    nodes_branch(ui, model, selection, &mut clicked);
    members_branch(ui, model, selection, &mut clicked);
    supports_branch(ui, model, selection, &mut clicked);
    loads_branch(ui, model, selection, &mut clicked);

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
                // Pin marker for rotational end releases (MIDAS-style).
                let release_label = match EditModel::release_label(member.releases) {
                    Some("Pin i") => " [\u{25cb}i]".to_string(),
                    Some("Pin j") => " [\u{25cb}j]".to_string(),
                    Some(_) => " [\u{25cb}i+j]".to_string(),
                    None => String::new(),
                };
                let text = format!(
                    "M{}: N{} \u{2192} N{}{}{}{}",
                    member.id.0,
                    member.node_i.0,
                    member.node_j.0,
                    load_label,
                    point_label,
                    release_label
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

/// File path / open / save controls, pinned to the top of the browser so they
/// are always visible.
///
/// A dead-ended **Open** / **Save** (no usable path) raises
/// [`ViewportApp::focus_project_path`]; the section is then forced open and its
/// field focused, so the command guides the user to type a path instead of
/// erroring.  The flag is only cleared once the field has actually been drawn
/// and focused — the header's open *animation* can delay the body by a frame —
/// and a request raised by a button inside this section survives to the next
/// frame.
fn project_file_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = super::theme::TextPalette::for_theme(app.dark_mode);
    let reveal = app.focus_project_path;
    egui::CollapsingHeader::new("Project file")
        .default_open(false)
        .open(reveal.then_some(true))
        .show(ui, |ui| {
            ui.label("File path:");
            let response = ui.add(
                egui::TextEdit::singleline(&mut app.file_path_input)
                    .hint_text("path to a .saproject file"),
            );
            if app.focus_project_path {
                response.request_focus();
                // Consumed only now that the field is really on screen.
                app.focus_project_path = false;
            }
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

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::persistence;

    fn app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// Run one headless frame of the left panel.
    fn frame(app: &mut ViewportApp, ctx: &egui::Context) {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(1000.0, 700.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            show_model_tree(app, ctx);
        });
    }

    #[test]
    fn reveal_expands_and_focuses_the_project_file_field() {
        let mut app = app();
        app.focus_project_path = true;
        let ctx = egui::Context::default();
        // The section animates open, so the field only appears a few frames
        // after the request; the flag is cleared exactly when it is focused.
        for _ in 0..30 {
            frame(&mut app, &ctx);
            if !app.focus_project_path {
                break;
            }
        }
        assert!(
            !app.focus_project_path,
            "the reveal request must be consumed once the field is drawn"
        );
        assert!(
            ctx.memory(|m| m.focused().is_some()),
            "the revealed path field must take keyboard focus"
        );
    }
}
