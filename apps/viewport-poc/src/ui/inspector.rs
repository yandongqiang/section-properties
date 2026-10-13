//! Right-hand inspector panel.
//!
//! The inspector content is selected by the active
//! [`WorkModule`](super::module::WorkModule).  The entity-editing code was
//! split out of the former monolithic `show_selection_section` so that
//! geometry/support editing (Model) and load editing (Load) can live on
//! separate tabs while sharing the same handlers.

use egui;

use structural_analysis::Dof3D;
use viewport_poc::app_state::{AnalysisStatus, ModelSummary, ResultsSummary, SolverBackend};
use viewport_poc::display::ResultViewMode;
use viewport_poc::edit_model::{MemberPointLoad3D, NodalLoadSpec, SupportSpec};
use viewport_poc::selection::SelectionState;

use super::module::WorkModule;
use super::theme;
use super::{fmt, forces_grid};
use crate::ViewportApp;

/// Smallest the inspector may be dragged.  Keeps the resize handle grabbable.
const INSPECTOR_MIN_WIDTH: f32 = 200.0;
/// Ceiling for the default inspector width (points).
const INSPECTOR_DEFAULT_MAX: f32 = 360.0;
/// Floor for the default inspector width (points).
const INSPECTOR_DEFAULT_MIN: f32 = 280.0;
/// Default inspector width as a fraction of the window width.
const INSPECTOR_DEFAULT_FRACTION: f32 = 0.25;
/// The inspector may never exceed this fraction of the window width, so it can
/// never swallow the viewport.
const INSPECTOR_MAX_FRACTION: f32 = 0.45;

/// Draw the right inspector panel for the active module.
///
/// The panel is resizable within a bounded width range (a small fraction of the
/// window by default, hard-capped so it can never swallow the viewport) and can
/// be collapsed entirely — when collapsed its panel is not created at all, so
/// the central viewport reclaims the space with no residual gap.
pub fn show_inspector(app: &mut ViewportApp, ctx: &egui::Context) {
    if app.inspector_collapsed {
        return;
    }
    let window_w = ctx.screen_rect().width();
    let max_w = (window_w * INSPECTOR_MAX_FRACTION).max(INSPECTOR_DEFAULT_MAX);
    let default_w =
        (window_w * INSPECTOR_DEFAULT_FRACTION).clamp(INSPECTOR_DEFAULT_MIN, INSPECTOR_DEFAULT_MAX);

    egui::SidePanel::right("inspector")
        .resizable(true)
        .min_width(INSPECTOR_MIN_WIDTH)
        .max_width(max_w)
        .default_width(default_w)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.strong("Inspector");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .button("\u{25b6} Hide")
                        .on_hover_text("Hide the inspector panel (Ctrl+I)")
                        .clicked()
                    {
                        app.inspector_collapsed = true;
                    }
                });
            });
            ui.separator();
            egui::ScrollArea::vertical().show(ui, |ui| {
                match app.module {
                    WorkModule::Model => {
                        show_add_section(app, ui);
                        show_selection_geometry(app, ui);
                    }
                    WorkModule::Section => super::section::show_section_module(app, ui),
                    WorkModule::Load => show_selection_loads(app, ui),
                    WorkModule::Analysis => show_analysis_section(app, ui),
                    WorkModule::Results => {
                        show_display_section(app, ui);
                        show_selection_results(app, ui);
                    }
                }
                ui.separator();
                show_camera_section(app, ui);
            });
        });
}

// ── Analysis ────────────────────────────────────────────────────────────

/// MIDAS/Abaqus-style analysis job panel.
///
/// Presents the solve action, the current [`AnalysisStatus`], a model
/// summary, the one solver option the crate really honours (the linear
/// backend), a post-solve results summary, and a warnings/errors list.
fn show_analysis_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.heading("Analysis");

    // Snapshot the model-derived data up-front so the immutable borrow of
    // the model ends before any button mutates the project.
    let summary = app.project.model.as_ref().map(ModelSummary::from_model);

    // The Run control is deliberately the first and most prominent element:
    // the primary action must be unmistakable the moment the tab is opened.
    show_run_control(app, ui);
    ui.separator();
    show_job_status(app, ui);
    ui.separator();
    show_model_summary(ui, summary.as_ref());
    ui.separator();
    show_solver_options(app, ui);
    ui.separator();
    show_results_summary(app, ui);
    ui.separator();
    show_analysis_diagnostics(app, ui, summary.as_ref());
}

/// Primary, always-visible Run control, placed at the very top of the panel.
///
/// The button is never hidden: when the model cannot be solved it stays
/// visible but disabled, with an adjacent one-line reason explaining why, so
/// the run action can never silently appear absent.  It shares its handler
/// ([`ViewportApp::run_solve`]) and its enabled predicate
/// ([`ViewportApp::can_run_analysis`]) with the Analysis menu and the
/// quick-action Solve button.
fn show_run_control(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let can_run = app.can_run_analysis();
    let status = app.project.analysis_status();
    let label = if status == AnalysisStatus::Stale {
        "Re-run Analysis"
    } else {
        "Run Analysis"
    };

    let text = egui::RichText::new(format!("\u{25b6} {label}")).strong();
    let mut button =
        egui::Button::new(text).min_size(egui::vec2(ui.available_width(), 0.0));
    if can_run {
        // Accent the primary action.  The palette exposes no dedicated accent
        // colour, so reuse egui's selection fill; leave the disabled button on
        // the theme's default muted styling so "not runnable" reads clearly.
        button = button.fill(ui.visuals().selection.bg_fill);
    }
    let response = ui.add_enabled(can_run, button);

    let response = if can_run {
        let hint = if status == AnalysisStatus::Stale {
            "Model or solver changed \u{2014} recompute results"
        } else {
            "Run the linear-static FEM solve"
        };
        response.on_hover_text(hint)
    } else {
        let reason = if app.project.model.is_none() {
            "Load a model, or add at least one node, to run the analysis."
        } else {
            "Add at least one node to run the analysis."
        };
        let response = response.on_disabled_hover_text(reason);
        ui.label(egui::RichText::new(reason).color(theme::HELP_C).small());
        response
    };

    if response.clicked() {
        app.run_solve();
    }
}

/// Current analysis-status badge.
fn show_job_status(app: &ViewportApp, ui: &mut egui::Ui) {
    let status = app.project.analysis_status();
    ui.horizontal(|ui| {
        ui.label("Status:");
        let (text, color) = match status {
            AnalysisStatus::Current => ("Current \u{2713}", theme::SUPPORT_C),
            AnalysisStatus::Stale => ("Stale \u{2014} re-run", theme::STALE_C),
            AnalysisStatus::Failed => ("Failed", theme::ERROR_C),
            AnalysisStatus::NeverSolved => ("Not run", theme::HELP_C),
        };
        ui.label(egui::RichText::new(text).color(color).strong());
    });
}

/// Counts table + a compact validity-warning indicator.
fn show_model_summary(ui: &mut egui::Ui, summary: Option<&ModelSummary>) {
    egui::CollapsingHeader::new("Model summary")
        .default_open(true)
        .show(ui, |ui| {
            let Some(s) = summary else {
                ui.label(egui::RichText::new("No model loaded.").color(theme::HELP_C));
                return;
            };
            egui::Grid::new("analysis_model_summary")
                .num_columns(2)
                .striped(true)
                .spacing([12.0, 3.0])
                .show(ui, |ui| {
                    count_row(ui, "Nodes", s.n_nodes);
                    count_row(ui, "Members", s.n_members);
                    count_row(ui, "Total DOF", s.total_dof);
                    count_row(ui, "Free DOF", s.free_dof());
                    count_row(ui, "Supports", s.n_supports);
                    count_row(ui, "Constrained DOF", s.constrained_dof);
                    count_row(ui, "Materials", s.n_materials);
                    count_row(ui, "Nodal loads", s.n_nodal_loads);
                    count_row(ui, "Member loads", s.n_member_loads);
                });
            if !s.warnings.is_empty() {
                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new(format!(
                        "\u{26a0} {} model warning(s) \u{2014} see below",
                        s.warnings.len()
                    ))
                    .color(theme::STALE_C),
                );
            }
        });
}

/// The one real solver option: the linear backend. Plus reset-to-defaults.
fn show_solver_options(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Solver options")
        .default_open(true)
        .show(ui, |ui| {
            let current = app.project.solver_backend();
            ui.horizontal(|ui| {
                ui.label("Linear backend:");
                egui::ComboBox::from_id_salt("solver_backend_combo")
                    .selected_text(current.label())
                    .show_ui(ui, |ui| {
                        for backend in SolverBackend::ALL {
                            if ui
                                .selectable_label(backend == current, backend.label())
                                .clicked()
                                && backend != current
                            {
                                app.project.set_solver_backend(backend);
                            }
                        }
                    });
            });
            ui.label(
                egui::RichText::new(solver_hint(current))
                    .color(theme::HELP_C)
                    .small(),
            );
            ui.add_space(2.0);
            let is_default = current == SolverBackend::default();
            if ui
                .add_enabled(!is_default, egui::Button::new("Reset to defaults"))
                .clicked()
            {
                app.project.reset_solver_options();
            }
        });
}

/// Post-solve headline figures. The 3D solver exposes no equilibrium report,
/// so none is shown.
fn show_results_summary(app: &ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Results summary")
        .default_open(true)
        .show(ui, |ui| {
            let Some(result) = app.analysis_result() else {
                ui.label(
                    egui::RichText::new("(no current result \u{2014} run analysis)")
                        .color(theme::STALE_C),
                );
                return;
            };
            let s = ResultsSummary::from_result(result);
            egui::Grid::new("analysis_results_summary")
                .num_columns(2)
                .striped(true)
                .spacing([12.0, 3.0])
                .show(ui, |ui| {
                    count_row(ui, "Nodes", s.n_nodes);
                    count_row(ui, "Members", s.n_members);
                    count_row(ui, "Total DOF", s.total_dof);
                    count_row(ui, "Free DOF", s.free_dof());
                    value_row(ui, "Max |u| (m)", fmt(s.max_translation));
                    value_row(ui, "Max |\u{03b8}| (rad)", fmt(s.max_rotation));
                    value_row(ui, "Max |R| (N or N\u{00b7}m)", fmt(s.max_reaction));
                });
            if let Some(backend) = app.project.solved_backend() {
                ui.add_space(2.0);
                ui.label(format!("Solver backend: {}", backend.label()));
            }
            ui.label(
                egui::RichText::new(
                    "Solver returns displacements, reactions and member end forces; \
                     it provides no equilibrium/iteration report.",
                )
                .color(theme::HELP_C)
                .small(),
            );
        });
}

/// Aggregated warnings and errors: solver failures plus model-validity
/// warnings.  A failed computation is always surfaced, never a silent zero.
fn show_analysis_diagnostics(
    app: &ViewportApp,
    ui: &mut egui::Ui,
    summary: Option<&ModelSummary>,
) {
    egui::CollapsingHeader::new("Warnings / errors")
        .default_open(true)
        .show(ui, |ui| {
            let mut any = false;
            if let Some(err) = app.project.last_solve_error() {
                any = true;
                ui.label(
                    egui::RichText::new(format!("\u{2717} Solve failed: {err}"))
                        .color(theme::ERROR_C),
                );
            }
            if let Some(s) = summary {
                for w in &s.warnings {
                    any = true;
                    ui.label(egui::RichText::new(format!("\u{26a0} {w}")).color(theme::STALE_C));
                }
            }
            if !any {
                ui.label(egui::RichText::new("No issues.").color(theme::HELP_C));
            }
        });
}

/// Two-column grid row for an integer count.
fn count_row(ui: &mut egui::Ui, label: &str, value: usize) {
    ui.label(label);
    ui.label(format!("{value}"));
    ui.end_row();
}

/// Two-column grid row for a pre-formatted value.
fn value_row(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(label);
    ui.label(value);
    ui.end_row();
}

/// Help text describing the selected backend.
fn solver_hint(backend: SolverBackend) -> &'static str {
    match backend {
        SolverBackend::Auto => {
            "Auto: dense (n \u{2264} 500), skyline LDL\u{1d40} (symmetric SPD), else sparse LU."
        }
        SolverBackend::Dense => {
            "Dense Gaussian elimination. General; best for small systems."
        }
        SolverBackend::SkylineLdlt => {
            "Skyline LDL\u{1d40}. Symmetric positive-definite systems only."
        }
        SolverBackend::SparseLu => "Sparse LU. General; best for medium/large systems.",
        SolverBackend::Cg => "Conjugate gradient. Symmetric positive-definite systems only.",
        SolverBackend::Iccg => {
            "ICCG (incomplete-Cholesky PCG). Symmetric positive-definite systems only."
        }
    }
}

// ── Display ──────────────────────────────────────────────────────────────

fn show_display_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Display")
        .default_open(true)
        .show(ui, |ui| {
            ui.label("Result view mode:");
            ui.horizontal_wrapped(|ui| {
                for mode in ResultViewMode::ALL {
                    let selected = app.display.mode == mode;
                    if ui
                        .add(egui::SelectableLabel::new(selected, mode.label()))
                        .clicked()
                    {
                        app.display.mode = mode;
                    }
                }
            });

            let result_available = app.analysis_result().is_some();
            if app.display.mode.requires_result() && !result_available {
                ui.label(egui::RichText::new("(no current result \u{2014} solve first)").color(theme::STALE_C));
            }

            ui.separator();

            if app.display.mode == ResultViewMode::Deformed {
                ui.checkbox(&mut app.display.show_undeformed, "Undeformed reference");
                ui.add(
                    egui::Slider::new(&mut app.display.deformation_scale, 0.0..=2000.0)
                        .text("Deform scale")
                        .logarithmic(true),
                );
                ui.label(format!(
                    "Scale: {:.1}x actual displacement",
                    app.display.deformation_scale
                ));
                if ui.button("Reset scale").clicked() {
                    app.display.deformation_scale = 100.0;
                }
                ui.separator();
            }

            ui.checkbox(&mut app.display.show_local_axes, "Local axes (selected)");
            ui.checkbox(&mut app.display.show_grid, "Ground grid");
            ui.checkbox(&mut app.display.show_global_axes, "Global axes");
            ui.checkbox(&mut app.display.show_node_labels, "Node labels");

            ui.separator();
            ui.label(egui::RichText::new("Internal-force diagrams").strong());
            ui.checkbox(&mut app.display.show_diagrams, "Show diagrams");
            ui.add_enabled_ui(app.display.show_diagrams, |ui| {
                ui.checkbox(&mut app.display.diagram_moment, "Moment (M)");
                ui.checkbox(&mut app.display.diagram_shear, "Shear (V)");
                ui.checkbox(&mut app.display.diagram_axial, "Axial (N)");
                ui.checkbox(&mut app.display.diagram_all_members, "All members");
                ui.add(
                    egui::Slider::new(&mut app.display.diagram_scale, 0.1..=5.0).text("Scale"),
                );
            });
        });
}

// ── Add node / member ───────────────────────────────────────────────────

fn show_add_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Add")
        .default_open(true)
        .show(ui, |ui| {
            ui.label("Add node:");
            ui.horizontal(|ui| {
                ui.label("x:");
                ui.text_edit_singleline(&mut app.new_node_x);
                ui.label("y:");
                ui.text_edit_singleline(&mut app.new_node_y);
                ui.label("z:");
                ui.text_edit_singleline(&mut app.new_node_z);
                if ui.button("Add node").clicked() {
                    app.add_node_from_input();
                }
            });

            ui.separator();

            ui.label("Add member:");
            ui.horizontal(|ui| {
                ui.label("N i:");
                ui.text_edit_singleline(&mut app.new_member_i);
                ui.label("N j:");
                ui.text_edit_singleline(&mut app.new_member_j);
                if ui.button("Add member").clicked() {
                    app.add_member_from_input();
                }
            });
        });
}

// ── Selection: geometry & supports (Model module) ───────────────────────

fn show_selection_geometry(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let sel = app.selection;
    egui::CollapsingHeader::new("Selection")
        .default_open(true)
        .show(ui, |ui| match sel {
            SelectionState::None => {
                ui.label("Nothing selected. Click a node or member.");
            }
            SelectionState::Node(id) => {
                show_node_geometry(app, ui, id);
            }
            SelectionState::Member(id) => {
                show_member_geometry(app, ui, id);
            }
        });
}

fn show_node_geometry(app: &mut ViewportApp, ui: &mut egui::Ui, id: viewport_poc::edit_model::NodeId) {
    ui.strong(format!("Node N{}", id.0));

    let mut changed = false;
    let mut new_coords = (0.0, 0.0, 0.0);
    if let Some(model) = app.project.model.as_ref() {
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
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.update_node(id, new_coords.0, new_coords.1, new_coords.2) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }

    if let Some(s) = &app.scene {
        if let Some(idx) = s.node_index(id) {
            if let Some(&p) = s.node_positions.get(idx) {
                ui.label(format!(
                    "Position: ({:.3}, {:.3}, {:.3}) m",
                    p[0], p[1], p[2]
                ));
            }
            if let Some(r) = app.analysis_result() {
                let disp = Dof3D::ALL.map(|d| r.displacement(idx, d).unwrap_or(0.0));
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
    if let Some(model) = app.project.model.as_ref() {
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
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.clear_support(id) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                        app.messages.push(viewport_poc::app_state::AppMessage::info(format!(
                            "Cleared support at node N{}",
                            id.0
                        )));
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    } else if support_changed {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.set_support(id, new_spec) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }

    if ui.button("Delete node").clicked() {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.delete_node(id) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                        app.selection = SelectionState::None;
                        app.messages.push(viewport_poc::app_state::AppMessage::info(format!(
                            "Deleted node N{}",
                            id.0
                        )));
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}

fn show_member_geometry(app: &mut ViewportApp, ui: &mut egui::Ui, id: viewport_poc::edit_model::MemberId) {
    ui.strong(format!("Member M{}", id.0));
    if let Some(s) = &app.scene {
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

    app.sync_member_edit_buffer(id);

    // ── Section / Material editing ──
    ui.collapsing("Section / Material", |ui| {
        egui::Grid::new("member_section_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("E (Pa):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.e)
                        .speed(1e8)
                        .range(1.0..=1e12),
                );
                ui.end_row();
                ui.label("G (Pa):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.g)
                        .speed(1e8)
                        .range(1.0..=1e12),
                );
                ui.end_row();
                ui.label("A (m\u{00b2}):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.area)
                        .speed(1e-5)
                        .range(1e-12..=1.0),
                );
                ui.end_row();
                ui.label("Iy (m\u{2074}):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.iy)
                        .speed(1e-8)
                        .range(1e-15..=1.0),
                );
                ui.end_row();
                ui.label("Iz (m\u{2074}):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.iz)
                        .speed(1e-8)
                        .range(1e-15..=1.0),
                );
                ui.end_row();
                ui.label("J (m\u{2074}):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.j)
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
                    egui::DragValue::new(&mut app.member_edit.ref_x)
                        .speed(0.1)
                        .range(-10.0..=10.0),
                );
                ui.end_row();
                ui.label("ref_y:");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.ref_y)
                        .speed(0.1)
                        .range(-10.0..=10.0),
                );
                ui.end_row();
                ui.label("ref_z:");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.ref_z)
                        .speed(0.1)
                        .range(-10.0..=10.0),
                );
                ui.end_row();
            });
    });

    ui.horizontal(|ui| {
        if ui.button("Apply Changes").clicked() {
            app.apply_member_edit(id);
        }
        if ui.button("Reset").clicked() {
            app.reset_member_edit(id);
        }
    });

    if ui.button("Delete member").clicked() {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.delete_member(id) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                        app.selection = SelectionState::None;
                        app.messages.push(viewport_poc::app_state::AppMessage::info(format!(
                            "Deleted member M{}",
                            id.0
                        )));
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}

// ── Selection: loads (Load module) ──────────────────────────────────────

fn show_selection_loads(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let sel = app.selection;
    egui::CollapsingHeader::new("Selection")
        .default_open(true)
        .show(ui, |ui| match sel {
            SelectionState::None => {
                ui.label("Nothing selected. Click a node or member.");
            }
            SelectionState::Node(id) => {
                ui.strong(format!("Node N{}", id.0));
                show_node_loads(app, ui, id);
            }
            SelectionState::Member(id) => {
                ui.strong(format!("Member M{}", id.0));
                show_member_loads(app, ui, id);
            }
        });
}

fn show_node_loads(app: &mut ViewportApp, ui: &mut egui::Ui, id: viewport_poc::edit_model::NodeId) {
    ui.separator();
    ui.label("Nodal load (global coords, N and N\u{00b7}m):");
    let mut load_changed = false;
    let mut new_load = NodalLoadSpec::zero();
    let mut has_load = false;
    let mut clear_load_clicked = false;
    if let Some(model) = app.project.model.as_ref() {
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
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.clear_nodal_load(id) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                        app.messages.push(viewport_poc::app_state::AppMessage::info(format!(
                            "Cleared load at node N{}",
                            id.0
                        )));
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    } else if load_changed {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.set_nodal_load(id, new_load) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate);
                    }
                }
                Err(e) => app.messages.push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}

fn show_member_loads(app: &mut ViewportApp, ui: &mut egui::Ui, id: viewport_poc::edit_model::MemberId) {
    app.sync_member_edit_buffer(id);

    // ── Distributed load editing ──
    ui.collapsing("Distributed Load (local)", |ui| {
        ui.label("Uniform load in local coordinates (N/m):");
        egui::Grid::new("member_load_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("qx (axial):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.load_qx)
                        .speed(100.0)
                        .range(-1e8..=1e8),
                );
                ui.end_row();
                ui.label("qy (transverse):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.load_qy)
                        .speed(100.0)
                        .range(-1e8..=1e8),
                );
                ui.end_row();
                ui.label("qz (transverse):");
                ui.add(
                    egui::DragValue::new(&mut app.member_edit.load_qz)
                        .speed(100.0)
                        .range(-1e8..=1e8),
                );
                ui.end_row();
            });
        if ui.button("Clear Load").clicked() {
            app.member_edit.load_qx = 0.0;
            app.member_edit.load_qy = 0.0;
            app.member_edit.load_qz = 0.0;
        }
    });

    // ── Point load editing ──
    ui.collapsing("Point Loads (local)", |ui| {
        ui.label("Concentrated forces in local coordinates (N):");
        let mut remove_idx = None;
        let mut add_new = false;
        for (i, pl) in app.member_edit.point_loads.iter_mut().enumerate() {
            ui.group(|ui| {
                egui::Grid::new(format!("point_load_grid_{i}"))
                    .num_columns(2)
                    .spacing([10.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("xi (position):");
                        ui.add(
                            egui::DragValue::new(&mut pl.xi)
                                .speed(0.05)
                                .range(0.0..=1.0)
                                .fixed_decimals(2),
                        );
                        ui.end_row();
                        ui.label("fx (axial):");
                        ui.add(
                            egui::DragValue::new(&mut pl.fx)
                                .speed(100.0)
                                .range(-1e8..=1e8),
                        );
                        ui.end_row();
                        ui.label("fy (transverse):");
                        ui.add(
                            egui::DragValue::new(&mut pl.fy)
                                .speed(100.0)
                                .range(-1e8..=1e8),
                        );
                        ui.end_row();
                        ui.label("fz (transverse):");
                        ui.add(
                            egui::DragValue::new(&mut pl.fz)
                                .speed(100.0)
                                .range(-1e8..=1e8),
                        );
                        ui.end_row();
                    });
                if ui.button("Delete").clicked() {
                    remove_idx = Some(i);
                }
            });
        }
        if ui.button("Add Point Load").clicked() {
            add_new = true;
        }
        if ui.button("Clear All").clicked() {
            app.member_edit.point_loads.clear();
        }
        if add_new {
            app.member_edit
                .point_loads
                .push(MemberPointLoad3D::new(0.5, 0.0, 0.0, -1000.0));
        }
        if let Some(i) = remove_idx {
            app.member_edit.point_loads.remove(i);
        }
    });

    ui.horizontal(|ui| {
        if ui.button("Apply Changes").clicked() {
            app.apply_member_edit(id);
        }
        if ui.button("Reset").clicked() {
            app.reset_member_edit(id);
        }
    });
}

// ── Selection: read-only results (Results module) ───────────────────────

fn show_selection_results(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let SelectionState::Member(id) = app.selection else {
        return;
    };
    egui::CollapsingHeader::new(format!("Member M{} results", id.0))
        .default_open(true)
        .show(ui, |ui| {
            let Some(s) = &app.scene else {
                return;
            };
            let Some(idx) = s.member_index(id) else {
                return;
            };
            let Some(r) = app.analysis_result() else {
                ui.label(egui::RichText::new("(no current result \u{2014} solve first)").color(theme::STALE_C));
                return;
            };
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
                ui.label(format!("  x = ({:.4}, {:.4}, {:.4})", x[0], x[1], x[2]));
                ui.label(format!("  y = ({:.4}, {:.4}, {:.4})", y[0], y[1], y[2]));
                ui.label(format!("  z = ({:.4}, {:.4}, {:.4})", z[0], z[1], z[2]));
            }
        });
}

// ── Camera ──────────────────────────────────────────────────────────────

fn show_camera_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Camera")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(format!(
                "Target: ({:.2}, {:.2}, {:.2})",
                app.camera.target[0], app.camera.target[1], app.camera.target[2]
            ));
            ui.label(format!("Distance: {:.2} m", app.camera.distance));
            ui.label(format!(
                "Azimuth: {:.1}\u{00b0}",
                app.camera.azimuth.to_degrees()
            ));
            ui.label(format!(
                "Elevation: {:.1}\u{00b0}",
                app.camera.elevation.to_degrees()
            ));
            ui.label(format!("FOV: {:.1}\u{00b0}", app.camera.fov.to_degrees()));
            if ui.button("Reset view").clicked() {
                app.camera.reset();
            }
        });
}
