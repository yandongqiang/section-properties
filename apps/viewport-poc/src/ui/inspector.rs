//! Right-hand panel: Inspector | Agent | Outline (Phase 159-B). //! //! A compact mode switcher at the top selects one of three modes: //! //! * **Inspector** —— the context-sensitive property editing, selected by the //!   active [`WorkModule`](super::module::WorkModule).  The entity-editing code //!   was split out of the former monolithic `show_selection_section` so that //!   geometry/support editing (Model) and load editing (Load) can live on //!   separate tabs while sharing the same handlers. //! * **Agent** —— an honest, disconnected placeholder for a future assistant. //!   It never fakes a connection and never mutates the model. //! * **Outline** —— a compact read-only summary of the model, built from the //!   existing model data. //! //! Switching modes is pure view state: it never touches the model, the //!selection or the analysis.
use super::icons::{self, IconFn};
use super::layout;
use super::module::WorkModule;
use super::theme::{self, TextPalette};
use super::{fmt, forces_grid};
use crate::ViewportApp;
use egui;
use structural_analysis::Dof3D;
use viewport_poc::app_state::{
    ActiveResult, AnalysisStatus, ModelSummary, ResultsSummary, SolverBackend,
};
use viewport_poc::display::ResultViewMode;
use viewport_poc::edit_model::{
    ComboTerm, EditModel, LoadCaseId, MemberPointLoad3D, NodalLoadSpec, SupportSpec,
};
use viewport_poc::selection::{SelectionSet, SelectionState};

/// Which of the three right-panel modes is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RightPanelMode {
    /// Context-sensitive property inspector (module-dependent).
    #[default]
    Inspector,
    /// Reserved AI assistant surface (disconnected placeholder).
    Agent,
    /// Compact read-only model summary.
    Outline,
}

impl RightPanelMode {
    /// All modes in switcher order.
    pub const ALL: [RightPanelMode; 3] = [
        RightPanelMode::Inspector,
        RightPanelMode::Agent,
        RightPanelMode::Outline,
    ];

    /// Short switcher label.
    pub const fn label(self) -> &'static str {
        match self {
            RightPanelMode::Inspector => "Inspector",
            RightPanelMode::Agent => "Agent",
            RightPanelMode::Outline => "Outline",
        }
    }

    /// Leading glyph for the mode-switcher button.
    pub const fn icon(self) -> IconFn {
        match self {
            RightPanelMode::Inspector => icons::sliders,
            RightPanelMode::Agent => icons::agent,
            RightPanelMode::Outline => icons::table,
        }
    }

    /// Stable index of this mode, used to persist the active mode (Phase 159-D).
    pub const fn index(self) -> u8 {
        match self {
            RightPanelMode::Inspector => 0,
            RightPanelMode::Agent => 1,
            RightPanelMode::Outline => 2,
        }
    }

    /// Resolve a persisted index back to a mode; an unset or out-of-range value
    /// falls back to the default mode (Phase 159-D).
    pub const fn from_index(index: Option<u8>) -> Self {
        match index {
            Some(0) => RightPanelMode::Inspector,
            Some(1) => RightPanelMode::Agent,
            Some(2) => RightPanelMode::Outline,
            _ => RightPanelMode::Inspector,
        }
    }
}
impl ViewportApp {
    /// Switch the right-panel mode.     ///     /// Pure view state: it deliberately leaves the model, the selection and     /// the analysis result untouched, so switching modes can never mutate the     ///project (Phase 159-B, B4).
    pub(crate) fn set_right_panel_mode(&mut self, mode: RightPanelMode) {
        self.right_panel_mode = mode;
    }
}

///Smallest the inspector may be dragged.  Keeps the resize handle grabbable.
const INSPECTOR_MIN_WIDTH: f32 = 200.0;

///Ceiling for the default inspector width (points).
const INSPECTOR_DEFAULT_MAX: f32 = 360.0;

///Floor for the default inspector width (points).
const INSPECTOR_DEFAULT_MIN: f32 = 280.0;

///Default inspector width as a fraction of the window width.
const INSPECTOR_DEFAULT_FRACTION: f32 = 0.25;

/// The inspector may never exceed this fraction of the window width, so it can ///never swallow the viewport.
const INSPECTOR_MAX_FRACTION: f32 = 0.45;

/// Desired width for a compact single-line entry field. /// /// A `TextEdit` defaults to a fixed `ui.spacing().text_edit_width` of 280 px; /// several of those side-by-side in a non-wrapping row would impose a minimum /// width far larger than [`INSPECTOR_MIN_WIDTH`], pinning the resizable panel /// open.  Sizing the fields from the current row width instead keeps the row ///narrower than the panel's own minimum.
fn entry_width(ui: &egui::Ui) -> f32 {
    (ui.available_width() * 0.30).clamp(48.0, 110.0)
}

/// Draw the right panel (Inspector / Agent / Outline) for the active mode. /// /// The panel is resizable within a bounded width range (a small fraction of the /// window by default, hard-capped so it can never swallow the viewport) and can /// be collapsed entirely —— when collapsed its panel is not created at all, so ///the central viewport reclaims the space with no residual gap.
pub fn show_inspector(app: &mut ViewportApp, ctx: &egui::Context) {
    if app.inspector_collapsed {
        return;
    }
    let window_w = ctx.screen_rect().width();
    let max_w = (window_w * INSPECTOR_MAX_FRACTION).max(INSPECTOR_DEFAULT_MAX);

    //  Phase P7-3 C2: a persisted layout preset overrides the window-fraction
    // default width (egui still clamps it into the panel's allowed range).
    let default_w = layout::restored_inspector_width(ctx).unwrap_or_else(|| {
        (window_w * INSPECTOR_DEFAULT_FRACTION).clamp(INSPECTOR_DEFAULT_MIN, INSPECTOR_DEFAULT_MAX)
    });
    egui::SidePanel::right("inspector")
        .resizable(true)
        .min_width(INSPECTOR_MIN_WIDTH)
        .max_width(max_w)
        .default_width(default_w)
        .show(ctx, |ui| {
            // ──── Header row: mode switcher (compact segmented control) ——————
            ui.horizontal(|ui| {
                ui.set_min_height(theme::ROW_HEIGHT);
                ui.spacing_mut().item_spacing.x = 2.0;
                for mode in RightPanelMode::ALL {
                    let selected = app.right_panel_mode == mode;
                    if icons::icon_text_button(ui, mode.icon(), mode.label(), selected, true)
                        .clicked()
                    {
                        app.set_right_panel_mode(mode);
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .button("\u{25b6} Hide")
                        .on_hover_text("Hide the right panel (Ctrl+I)")
                        .clicked()
                    {
                        app.inspector_collapsed = true;
                    }
                });
            });
            ui.separator();
            egui::ScrollArea::vertical().show(ui, |ui| {
                //  Make the content fill the panel's width.  A resizable
                // side panel is only a stable fixed point at the width the user drags it to when its content is at least that wide; content narrower than the panel lets egui shrink the panel back down to its minimum every frame.
                ui.set_min_width(ui.available_width());
                match app.right_panel_mode {
                    RightPanelMode::Inspector => {
                        show_module_inspector(app, ui);
                    }
                    RightPanelMode::Agent => show_agent_placeholder(app, ui),
                    RightPanelMode::Outline => show_outline(app, ui),
                }
            });

            //  Publish the live panel width for layout-preset persistence
            // (Phase P7-3 C2).  `ui.max_rect()` is the panel's *inner* rect; add the frame's inner margin back to recover the outer width.
            let margin = egui::Frame::side_top_panel(ui.style()).inner_margin;
            layout::publish_inspector_width(
                ctx,
                ui.max_rect().width() + (margin.left + margin.right) as f32,
            );
        });
}

///Inspector mode: the module-dependent property editing (unchanged content).
fn show_module_inspector(app: &mut ViewportApp, ui: &mut egui::Ui) {
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
}

// ──── Agent (reserved, honestly disconnected) ——————————————————————————————————————————————————————————  /// Agent mode: a clearly-labelled, non-functional placeholder. /// /// There is deliberately **no** LLM / HTTP / fake behaviour here: the status /// is "Not connected", the input and command-preview region are disabled, and ///the note forwards the user to the (already real) command layer.
fn show_agent_placeholder(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.label(theme::header("Agent"));
    ui.horizontal(|ui| {
        ui.label(
            theme::body("\u{25cf} Not connected")
                .color(text.stale)
                .strong(),
        );
    });
    ui.label(
        theme::body("This surface reserves the AI-assistant panel. No model is connected.")
            .color(text.help),
    );
    ui.separator();

    //Disabled input + command-preview region (non-interactive by design).
    ui.add_enabled_ui(false, |ui| {
        let mut prompt = String::new();
        ui.add(
            egui::TextEdit::singleline(&mut prompt)
                .hint_text("Ask the agent to modify the model\u{2026}")
                .desired_width(ui.available_width()),
        );
        ui.add_space(6.0);
        ui.label(theme::small("Command preview"));
        let mut preview = String::new();
        ui.add(
            egui::TextEdit::singleline(&mut preview)
                .hint_text("no command")
                .desired_width(ui.available_width()),
        );
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let _ = ui.button("Run");
            let _ = ui.button("Cancel");
        });
    });
    ui.separator();
    ui.label(         theme::small(             "When implemented, AI commands will be dispatched through the same command layer as the palette and menus (Ctrl+Shift+P) \u{2014} never by editing the model directly.",         )         .color(text.help),     );
}

// ──── Outline (compact read-only summary) ——————————————————————————————————————————————————————————————————  ///Outline mode: model counts, groups and the selected entity path.
fn show_outline(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.label(theme::header("Outline"));
    let Some(model) = app.project.model.as_ref() else {
        ui.label(egui::RichText::new("No model loaded").color(text.help));
        return;
    };
    let summary = ModelSummary::from_model(model);
    ui.label(theme::body("Model").strong());
    ui.label(format!("Nodes ({})", summary.n_nodes));
    ui.label(format!("Members ({})", summary.n_members));
    ui.label(format!("Supports ({})", summary.n_supports));
    ui.label(format!(
        "Loads ({})",
        summary.n_nodal_loads + summary.n_member_loads
    ));
    ui.separator();
    ui.label(theme::body("Selection").strong());
    match app.selected.primary() {
        SelectionState::Node(id) => {
            ui.label(format!("Model / Nodes / N{}", id.0));
        }
        SelectionState::Member(id) => {
            ui.label(format!("Model / Members / M{}", id.0));
        }
        SelectionState::None => {
            ui.label(egui::RichText::new("(nothing selected)").color(text.help));
        }
    }
    if app.selected.len() > 1 {
        ui.label(
            theme::small(format!("+{} more selected", app.selected.len() - 1)).color(text.help),
        );
    }
    ui.separator();
    ui.label(theme::body("Analysis").strong());
    let (status, color) = match app.project.analysis_status() {
        AnalysisStatus::Current => ("Current", text.ok),
        AnalysisStatus::Stale => ("Stale \u{2014} re-run", text.stale),
        AnalysisStatus::Failed => ("Failed", text.error),
        AnalysisStatus::NeverSolved => ("Not run", text.help),
    };
    ui.label(egui::RichText::new(status).color(color));
}

// ──── Analysis ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————  /// MIDAS/Abaqus-style analysis job panel. /// /// Presents the solve action, the current [`AnalysisStatus`], a model /// summary, the one solver option the crate really honours (the linear /// backend), a post-solve results summary, and a warnings/errors list. /// /// Shared with the centre workspace's **Analysis** module /// (`ui::workspace::show_analysis_content`), which calls this same function —— /// the body is *mirrored*, never duplicated, so both surfaces read the one /// `Project`.
pub(crate) fn show_analysis_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.label(theme::header("Analysis"));

    // Snapshot the model-derived data up-front so the immutable borrow of
    // the model ends before any button mutates the project.
    let summary = app.project.model.as_ref().map(ModelSummary::from_model);

    // The Run control is deliberately the first and most prominent element:
    // the primary action must be unmistakable the moment the tab is opened.
    show_run_control(app, ui);
    ui.separator();
    show_job_status(app, ui);
    ui.separator();
    show_model_summary(ui, summary.as_ref(), TextPalette::for_theme(app.dark_mode));
    ui.separator();
    show_solver_options(app, ui);
    ui.separator();
    show_results_summary(app, ui);
    ui.separator();
    show_analysis_diagnostics(app, ui, summary.as_ref());
}

/// Primary, always-visible Run control, placed at the very top of the panel. /// /// The button is never hidden: when the model cannot be solved it stays /// visible but disabled, with an adjacent one-line reason explaining why, so /// the run action can never silently appear absent.  It shares its handler /// ([`ViewportApp::run_solve`]) and its enabled predicate /// ([`ViewportApp::can_run_analysis`]) with the Analysis menu and the ///quick-action Solve button.
/// Primary, always-visible Run control, placed at the very top of the panel.
///
/// The button is never hidden: when the model cannot be solved it stays
/// visible but disabled, with an adjacent one-line reason explaining why, so
/// the run action can never silently appear absent.  It shares its handler
/// ([`ViewportApp::run_solve`]) and its enabled predicate
/// ([`ViewportApp::can_run_analysis`]) with the Analysis menu and the
/// quick-action Solve button.
fn show_run_control(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let colors = TextPalette::for_theme(app.dark_mode);
    let can_run = app.can_run_analysis();
    let status = app.project.analysis_status();
    let label = if status == AnalysisStatus::Stale {
        "Re-run Analysis"
    } else {
        "Run Analysis"
    };

    let mut text = theme::body(format!("\u{25b6} {label}")).strong();
    if can_run {
        // Accent the primary action with the theme's on-accent foreground so
        // the label stays legible on the accent fill in both themes.
        text = text.color(theme::on_accent(app.dark_mode));
    }
    let mut button = egui::Button::new(text).min_size(egui::vec2(ui.available_width(), 0.0));
    if can_run {
        button = button.fill(theme::accent(app.dark_mode));
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
        ui.label(theme::small(reason).color(colors.help));
        response
    };

    if response.clicked() {
        app.run_solve();
    }
}
///Current analysis-status badge.
fn show_job_status(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let status = app.project.analysis_status();
    ui.horizontal(|ui| {
        ui.label("Status:");
        let (text_label, color) = match status {
            AnalysisStatus::Current => ("Current \u{2713}", text.ok),
            AnalysisStatus::Stale => ("Stale \u{2014} re-run", text.stale),
            AnalysisStatus::Failed => ("Failed", text.error),
            AnalysisStatus::NeverSolved => ("Not run", text.help),
        };
        ui.label(theme::body(text_label).color(color).strong());
    });
}

///Counts table + a compact validity-warning indicator.
fn show_model_summary(ui: &mut egui::Ui, summary: Option<&ModelSummary>, text: TextPalette) {
    egui::CollapsingHeader::new("Model summary")
        .default_open(true)
        .show(ui, |ui| {
            let Some(s) = summary else {
                ui.label(egui::RichText::new("No model loaded.").color(text.help));
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
                    .color(text.stale),
                );
            }
        });
}

///The one real solver option: the linear backend. Plus reset-to-defaults.
fn show_solver_options(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
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
            ui.label(theme::small(solver_hint(current)).color(text.help));
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

/// Post-solve headline figures. The 3D solver exposes no equilibrium report, ///so none is shown.
fn show_results_summary(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    egui::CollapsingHeader::new("Results summary")         .default_open(true)         .show(ui, |ui| {
             let Some(result) = app.analysis_result() else {
                 ui.label(                     egui::RichText::new("(no current result \u{2014} run analysis)")                         .color(text.stale),                 );
                 return;
             }
;
             let s = ResultsSummary::from_result(result);
             egui::Grid::new("analysis_results_summary")                 .num_columns(2)                 .striped(true)                 .spacing([12.0, 3.0])                 .show(ui, |ui| {
                     count_row(ui, "Nodes", s.n_nodes);
                     count_row(ui, "Members", s.n_members);
                     count_row(ui, "Total DOF", s.total_dof);
                     count_row(ui, "Free DOF", s.free_dof());
                     value_row(ui, "Max |u| (m)", fmt(s.max_translation));
                     value_row(ui, "Max |\u{03b8}| (rad)", fmt(s.max_rotation));
                     value_row(ui, "Max |R| (N or N\u{00b7}m)", fmt(s.max_reaction));
                 }
);
             if let Some(backend) = app.project.solved_backend() {
                 ui.add_space(2.0);
                 ui.label(format!("Solver backend: {}", backend.label()));
             }
             ui.label(                 theme::small(                     "Solver returns displacements, reactions and member end forces; it provides no equilibrium/iteration report.",                 )                 .color(text.help),             );
         }
);
}

/// Aggregated warnings and errors: solver failures plus model-validity ///warnings.  A failed computation is always surfaced, never a silent zero.
fn show_analysis_diagnostics(app: &ViewportApp, ui: &mut egui::Ui, summary: Option<&ModelSummary>) {
    let text = TextPalette::for_theme(app.dark_mode);
    egui::CollapsingHeader::new("Warnings / errors")
        .default_open(true)
        .show(ui, |ui| {
            let mut any = false;
            if let Some(err) = app.project.last_solve_error() {
                any = true;
                ui.label(
                    egui::RichText::new(format!("\u{2717} Solve failed: {err}")).color(text.error),
                );
            }
            if let Some(s) = summary {
                for w in &s.warnings {
                    any = true;
                    ui.label(egui::RichText::new(format!("\u{26a0} {w}")).color(text.stale));
                }
            }
            if !any {
                ui.label(egui::RichText::new("No issues.").color(text.help));
            }
        });
}

///Two-column grid row for an integer count.
fn count_row(ui: &mut egui::Ui, label: &str, value: usize) {
    ui.label(label);
    ui.label(format!("{value}"));
    ui.end_row();
}

///Two-column grid row for a pre-formatted value.
fn value_row(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(label);
    ui.label(value);
    ui.end_row();
}

///Help text describing the selected backend.
fn solver_hint(backend: SolverBackend) -> &'static str {
    match backend {
        SolverBackend::Auto => {
            "Auto: dense (n \u{2264} 500), skyline LDL\u{1d40} (symmetric SPD), else sparse LU."
        }
        SolverBackend::Dense => "Dense Gaussian elimination. General; best for small systems.",
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

// ──── Display ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————  /// Result selector: which solved load case or load combination the Results /// module displays (the viewport, diagrams and result tables all follow it). /// /// Pure view state —— switching never marks the results stale, because the ///model they were solved from has not changed.
pub(crate) fn show_result_selector(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.label(theme::header("Displayed result"));
    if app.project.analysis_status() != AnalysisStatus::Current {
        ui.label(
            egui::RichText::new("No current results \u{2014} run the analysis.").color(text.help),
        );
        return;
    }

    // Snapshot the labels (no live borrow of `project.analysis` while the
    // handlers below call `&mut project`).
    let (kind, active, cases, combos) = {
        let Some(a) = app.project.analysis.as_ref() else {
            return;
        };
        (
            app.project.active_result_kind(),
            app.project.active_result_id(),
            a.case_labels(),
            a.combination_labels(),
        )
    };
    for (id, name) in &cases {
        let selected = kind == ActiveResult::Case && active == *id;
        if ui
            .selectable_label(selected, format!("Case: {name}"))
            .clicked()
            && !selected
        {
            app.project.set_active_case(*id);
        }
    }
    for (id, name) in &combos {
        let selected = kind == ActiveResult::Combination && active == *id;
        if ui
            .selectable_label(selected, format!("Combo: {name}"))
            .clicked()
            && !selected
        {
            app.project.set_active_combination(*id);
        }
    }
    if let Some(label) = app.project.active_result_label() {
        ui.label(
            egui::RichText::new(format!("Showing {label}"))
                .color(text.help)
                .small(),
        );
    }
    ui.separator();
}
fn show_display_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    egui::CollapsingHeader::new("Display")
        .default_open(true)
        .show(ui, |ui| {
            show_result_selector(app, ui);
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
                ui.label(
                    egui::RichText::new("(no current result \u{2014} solve first)")
                        .color(text.stale),
                );
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
            ui.label(theme::body("Internal-force diagrams").strong());
            ui.checkbox(&mut app.display.show_diagrams, "Show diagrams");
            ui.add_enabled_ui(app.display.show_diagrams, |ui| {
                ui.checkbox(&mut app.display.diagram_moment, "Moment (M)");
                ui.checkbox(&mut app.display.diagram_shear, "Shear (V)");
                ui.checkbox(&mut app.display.diagram_axial, "Axial (N)");
                ui.checkbox(&mut app.display.diagram_all_members, "All members");
                ui.add(egui::Slider::new(&mut app.display.diagram_scale, 0.1..=5.0).text("Scale"));
            });
        });
}

// ──── Add node / member ——————————————————————————————————————————————————————————————————————————————————————————————————————
fn show_add_section(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Add")
        .default_open(true)
        .show(ui, |ui| {
            ui.label("Add node:");
            let node_field = entry_width(ui);
            ui.horizontal_wrapped(|ui| {
                ui.label("x:");
                ui.add(egui::TextEdit::singleline(&mut app.new_node_x).desired_width(node_field));
                ui.label("y:");
                ui.add(egui::TextEdit::singleline(&mut app.new_node_y).desired_width(node_field));
                ui.label("z:");
                ui.add(egui::TextEdit::singleline(&mut app.new_node_z).desired_width(node_field));
                if ui.button("Add node").clicked() {
                    app.add_node_from_input();
                }
            });
            ui.separator();
            ui.label("Add member:");
            let member_field = entry_width(ui);
            ui.horizontal_wrapped(|ui| {
                ui.label("N i:");
                ui.add(
                    egui::TextEdit::singleline(&mut app.new_member_i).desired_width(member_field),
                );
                ui.label("N j:");
                ui.add(
                    egui::TextEdit::singleline(&mut app.new_member_j).desired_width(member_field),
                );
                if ui.button("Add member").clicked() {
                    app.add_member_from_input();
                }
            });
        });
}

// ──── Selection: geometry & supports (Model module) ——————————————————————————————————————————————
fn show_selection_geometry(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let set = app.selected.clone();
    egui::CollapsingHeader::new("Selection")
        .default_open(true)
        .show(ui, |ui| match set.single() {
            Some(SelectionState::Node(id)) => show_node_geometry(app, ui, id),
            Some(SelectionState::Member(id)) => show_member_geometry(app, ui, id),
            _ if set.is_empty() => {
                ui.label("Nothing selected. Click a node or member.");
            }
            _ => show_multi_selection(app, ui, &set),
        });
}

/// Compact summary + bulk actions shown when more than one entity is selected. /// /// Replaces the per-entity detail fields (which are meaningless for a mixed /// selection) with the actions that *are* meaningful in bulk: delete the whole /// selection in one undo step, hide it, or route to the Section module when the ///selection contains members.
fn show_multi_selection(app: &mut ViewportApp, ui: &mut egui::Ui, set: &SelectionSet) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.strong(format!("{} entities selected", set.len()));
    ui.label(format!(
        "{} node(s), {} member(s)",
        set.node_count(),
        set.member_count()
    ));
    ui.separator();
    ui.horizontal_wrapped(|ui| {
        if ui
            .button("Delete")
            .on_hover_text("Delete the whole selection (single undo step)")
            .clicked()
        {
            app.delete_selection();
        }
        if ui
            .button("Hide")
            .on_hover_text("Hide the whole selection (view only)")
            .clicked()
        {
            app.hide_selected();
        }
    });
    if set.member_count() > 0 && ui.button("Assign section\u{2026}").clicked() {
        app.module = WorkModule::Section;
    }

    // A short, deterministic preview of what is selected.
    let preview: Vec<String> = set
        .iter()
        .take(8)
        .map(|s| match s {
            SelectionState::Node(id) => format!("N{}", id.0),
            SelectionState::Member(id) => format!("M{}", id.0),
            SelectionState::None => String::new(),
        })
        .collect();
    if !preview.is_empty() {
        let more = set.len().saturating_sub(preview.len());
        let suffix = if more > 0 {
            format!(" \u{2026} (+{more})")
        } else {
            String::new()
        };
        ui.label(theme::small(format!("{}{}", preview.join(", "), suffix)).color(text.help));
    }
}
fn show_node_geometry(
    app: &mut ViewportApp,
    ui: &mut egui::Ui,
    id: viewport_poc::edit_model::NodeId,
) {
    ui.strong(format!("Node N{}", id.0));
    let mut changed = false;
    let mut new_coords = (0.0, 0.0, 0.0);
    if let Some(model) = app.project.model.as_ref() {
        if let Some(node) = model.node(id) {
            let mut x = node.x;
            let mut y = node.y;
            let mut z = node.z;
            ui.horizontal_wrapped(|ui| {
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
                        app.commit_edit(candidate, format!("Edit node N{}", id.0));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
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

    // ──── Support editing ————
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
    ui.horizontal_wrapped(|ui| {
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
                        app.commit_edit(candidate, format!("Clear support N{}", id.0));
                        app.messages
                            .push(viewport_poc::app_state::AppMessage::info(format!(
                                "Cleared support at node N{}",
                                id.0
                            )));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    } else if support_changed {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.set_support(id, new_spec) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate, format!("Support N{}", id.0));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
    if ui.button("Delete node").clicked() {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.delete_node(id) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate, format!("Delete node N{}", id.0));
                        app.clear_selection();
                        app.messages
                            .push(viewport_poc::app_state::AppMessage::info(format!(
                                "Deleted node N{}",
                                id.0
                            )));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}

/// 	rue when every rotational DOF of a member is released (a pin at both ///ends: no moment, no bending stiffness anywhere).
fn all_released(releases: &[bool; 6]) -> bool {
    releases.iter().all(|&r| r)
}
fn show_member_geometry(
    app: &mut ViewportApp,
    ui: &mut egui::Ui,
    id: viewport_poc::edit_model::MemberId,
) {
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

    // ──── Section / Material editing ————
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

    //  ──── End releases (pins) ————————————————————————————————————————————————————————————————————————————————————————
    // MIDAS-style member end release: a released rotation transmits no moment.  The freed rotation is only restrained by the joint's other members (or a support), so releasing both ends of a lone member is a mechanism ——the label warns about exactly that.
    ui.collapsing("End Releases", |ui| {
        let current = app
            .project
            .model
            .as_ref()
            .and_then(|m| m.member_releases(id))
            .unwrap_or([false; 6]);
        let mut releases = current;
        let mut changed = false;
        egui::Grid::new("member_release_grid")
            .num_columns(3)
            .spacing([10.0, 3.0])
            .show(ui, |ui| {
                ui.strong("");
                ui.strong("Rx");
                ui.strong("Ry");
                ui.strong("Rz");
                ui.end_row();
                for (label, base) in [("End i", 0usize), ("End j", 3usize)] {
                    ui.strong(label);
                    for k in 0..3 {
                        if ui.checkbox(&mut releases[base + k], "").changed() {
                            changed = true;
                        }
                    }
                    ui.end_row();
                }
            });
        let label = EditModel::release_label(releases);
        ui.label(match label {
            None => theme::small("Rigid connection at both ends.")
                .color(TextPalette::for_theme(app.dark_mode).help),
            Some(l) => theme::small(format!(
                "{l} \u{2014} no moment transferred at the released end(s)."
            )),
        });
        if all_released(&releases) {
            ui.label(
                theme::small(
                    "\u{26a0} Both ends released: the member gives no bending stiffness at all.",
                )
                .color(TextPalette::for_theme(app.dark_mode).stale),
            );
        }
        if changed {
            if let Some(model) = app.project.model.as_ref() {
                let mut candidate = model.clone();
                match candidate.set_member_releases(id, releases) {
                    Ok(()) => {
                        if candidate.revision() != model.revision() {
                            app.commit_edit(candidate, format!("Releases M{}", id.0));
                        }
                    }
                    Err(e) => app
                        .messages
                        .push(viewport_poc::app_state::AppMessage::error(e.message())),
                }
            }
        }
    });

    // ──── Reference vector editing ————
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
    ui.horizontal_wrapped(|ui| {
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
                        app.commit_edit(candidate, format!("Delete member M{}", id.0));
                        app.clear_selection();
                        app.messages
                            .push(viewport_poc::app_state::AppMessage::info(format!(
                                "Deleted member M{}",
                                id.0
                            )));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}

// ──── Selection: loads (Load module) ————————————————————————————————————————————————————————————————————————————
impl ViewportApp {
    ///Add a new load case (undoable engineering edit) and view it.
    pub(crate) fn add_load_case(&mut self, name: &str) -> Result<LoadCaseId, String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let mut candidate = model.clone();
        match candidate.add_load_case(name) {
            Ok(id) => {
                self.commit_edit(candidate, format!("Add load case {name}"));
                Ok(id)
            }
            Err(e) => Err(e.message()),
        }
    }

    ///Delete a load case and its loads (undoable engineering edit).
    pub(crate) fn delete_load_case(&mut self, id: LoadCaseId) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let name = model
            .load_case(id)
            .map(|c| c.name.clone())
            .unwrap_or_else(|| format!("#{}", id.0));
        let mut candidate = model.clone();
        match candidate.delete_load_case(id) {
            Ok(()) => {
                self.commit_edit(candidate, format!("Delete load case {name}"));
                Ok(())
            }
            Err(e) => Err(e.message()),
        }
    }

    ///Toggle a load case's self-weight option (undoable engineering edit).
    pub(crate) fn set_case_self_weight(&mut self, id: LoadCaseId, on: bool) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let mut candidate = model.clone();
        match candidate.set_case_self_weight(id, on) {
            Ok(()) => {
                if candidate.revision() == model.revision() {
                    return Ok(());
                }
                self.commit_edit(candidate, "Toggle self weight");
                Ok(())
            }
            Err(e) => Err(e.message()),
        }
    }

    ///Add a load combination (undoable engineering edit).
    pub(crate) fn add_load_combination(&mut self, name: &str) -> Result<LoadCaseId, String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let mut candidate = model.clone();
        match candidate.add_load_combination(name) {
            Ok(id) => {
                self.commit_edit(candidate, format!("Add load combination {name}"));
                Ok(id)
            }
            Err(e) => Err(e.message()),
        }
    }

    ///Delete a load combination (undoable engineering edit).
    pub(crate) fn delete_load_combination(&mut self, id: LoadCaseId) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let mut candidate = model.clone();
        match candidate.delete_load_combination(id) {
            Ok(()) => {
                self.commit_edit(candidate, "Delete load combination");
                Ok(())
            }
            Err(e) => Err(e.message()),
        }
    }

    ///Apply the staged combination terms (and optional rename).
    pub(crate) fn apply_combination_terms(
        &mut self,
        id: LoadCaseId,
        name: Option<String>,
        terms: Vec<ComboTerm>,
    ) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".into());
        };
        let mut candidate = model.clone();
        if let Some(name) = name {
            candidate
                .rename_load_combination(id, &name)
                .map_err(|e| e.message())?;
        }
        candidate
            .set_combination_terms(id, terms)
            .map_err(|e| e.message())?;
        if candidate.revision() == model.revision() {
            return Ok(());
        }
        self.commit_edit(candidate, "Edit load combination");
        Ok(())
    }
}

/// Load-case manager + load-combination editor (Load module). /// /// The active case drives every load editor in this panel (and the load /// symbols drawn in the viewport); combinations are staged in a buffer and /// applied explicitly, so an abandoned edit never silently changes results. /// /// All model reads are snapshotted into owned values first: the immediate- /// mode body below needs `&mut ViewportApp` for its handlers, so no borrow of ///`project.model` may stay live across them.
fn show_load_cases(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);

    // ── Snapshot ──
    let Some(model) = app.project.model.as_ref() else {
        ui.label(theme::header("Load cases"));
        ui.label(egui::RichText::new("No model loaded.").color(text.help));
        return;
    };
    let active = model.active_case();
    let cases: Vec<(LoadCaseId, String, bool)> = model
        .load_cases()
        .iter()
        .map(|c| (c.id, c.name.clone(), c.self_weight))
        .collect();
    let case_names: Vec<(LoadCaseId, String)> = model
        .load_cases()
        .iter()
        .map(|c| (c.id, c.name.clone()))
        .collect();
    let combos: Vec<(LoadCaseId, String, String)> = model
        .combinations()
        .iter()
        .map(|c| {
            let terms = c
                .terms
                .iter()
                .map(|t| {
                    let name = case_names
                        .iter()
                        .find(|(cid, _)| *cid == t.case)
                        .map(|(_, n)| n.clone())
                        .unwrap_or_else(|| format!("#{}", t.case.0));
                    format!("{:.3}{}", t.factor, name)
                })
                .collect::<Vec<_>>()
                .join(" + ");
            (c.id, c.name.clone(), terms)
        })
        .collect();
    let name_of = |id: LoadCaseId| -> String {
        case_names
            .iter()
            .find(|(cid, _)| *cid == id)
            .map(|(_, n)| n.clone())
            .unwrap_or_else(|| format!("#{}", id.0))
    };

    //Combination being edited: (name, staged terms from the model, dangling).
    let editing = app.editing_combination.and_then(|id| {
        model.combination(id).map(|c| {
            (
                c.name.clone(),
                c.terms
                    .iter()
                    .map(|t| (t.case, t.factor))
                    .collect::<Vec<_>>(),
                model.dangling_combination_cases(id),
            )
        })
    });

    // ──── Active case ——————————————————————————————————————————————————————————————————————————————————————————————————————
    ui.label(theme::header("Load case"));
    for (id, name, sw) in &cases {
        ui.horizontal(|ui| {
            let selected = active == *id;
            if ui.selectable_label(selected, name.as_str()).clicked() && !selected {
                if let Some(m) = app.project.model.as_mut() {
                    let _ = m.set_active_case(*id);
                    app.rebuild_scene();
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let mut on = *sw;
                if ui.checkbox(&mut on, "Self weight").changed() {
                    if let Err(e) = app.set_case_self_weight(*id, on) {
                        app.messages
                            .push(viewport_poc::app_state::AppMessage::error(e));
                    }
                }
            });
        });
    }

    // ──── Case management ——————————————————————————————————————————————————————————————————————————————————————————————
    ui.horizontal(|ui| {
        ui.add(
            egui::TextEdit::singleline(&mut app.new_case_name)
                .hint_text("New case name")
                .desired_width(entry_width(ui)),
        );
        if ui.button("Add").clicked() {
            let name = app.new_case_name.clone();
            if let Err(e) = app.add_load_case(&name) {
                app.messages
                    .push(viewport_poc::app_state::AppMessage::error(e));
            } else {
                app.new_case_name.clear();
            }
        }
        if ui
            .add_enabled(cases.len() > 1, egui::Button::new("Delete"))
            .on_disabled_hover_text("A model must keep at least one load case")
            .clicked()
            && let Err(e) = app.delete_load_case(active)
        {
            app.messages
                .push(viewport_poc::app_state::AppMessage::error(e));
        }
    });
    ui.separator();

    // ──── Load combinations ——————————————————————————————————————————————————————————————————————————————————————————
    ui.label(theme::header("Load combinations"));
    if combos.is_empty() {
        ui.label(
            egui::RichText::new("None \u{2014} combinations superpose load cases.")
                .color(text.help)
                .small(),
        );
    }
    for (id, name, terms) in &combos {
        let selected_combo = app.editing_combination == Some(*id);
        if ui
            .selectable_label(selected_combo, format!("{name}  [{terms}]"))
            .clicked()
            && !selected_combo
        {
            //  Selecting a (re)loads the staging buffer from the model, so an
            // abandoned edit never writes anything.
            app.editing_combination = Some(*id);
            app.combo_terms_buffer = terms_buffer_from_model(app, *id);
        }
    }
    ui.horizontal(|ui| {
        ui.add(
            egui::TextEdit::singleline(&mut app.new_combo_name)
                .hint_text("New combination name")
                .desired_width(entry_width(ui)),
        );
        if ui.button("Add").clicked() {
            let name = app.new_combo_name.clone();
            if let Err(e) = app.add_load_combination(&name) {
                app.messages
                    .push(viewport_poc::app_state::AppMessage::error(e));
            } else {
                app.new_combo_name.clear();
                app.editing_combination = app
                    .project
                    .model
                    .as_ref()
                    .and_then(|m| m.combinations().last().map(|c| c.id));
                app.combo_terms_buffer.clear();
            }
        }
    });

    // ──── Combination editor ————————————————————————————————————————————————————————————————————————————————————————
    if let (Some(combo_id), Some((name, _, danglers))) = (app.editing_combination, editing.as_ref())
    {
        ui.separator();
        ui.label(theme::body(format!("Terms of \u{201c}{name}\u{201d}")).strong());
        let mut remove_row = None;
        for (i, (case_id, factor)) in app.combo_terms_buffer.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(format!("combo_term_case_{i}"))
                    .selected_text(name_of(*case_id))
                    .show_ui(ui, |ui| {
                        for (cid, cname) in &case_names {
                            if ui
                                .selectable_label(*case_id == *cid, cname.as_str())
                                .clicked()
                            {
                                *case_id = *cid;
                            }
                        }
                    });
                ui.add(egui::DragValue::new(factor).speed(0.05).range(-99.0..=99.0));
                if ui.button("\u{2716}").clicked() {
                    remove_row = Some(i);
                }
            });
        }
        if let Some(i) = remove_row {
            app.combo_terms_buffer.remove(i);
        }
        ui.horizontal(|ui| {
            if ui.button("Add term").clicked() {
                let first = case_names
                    .first()
                    .map(|(id, _)| (*id, 1.0))
                    .unwrap_or((LoadCaseId(0), 1.0));
                app.combo_terms_buffer.push(first);
            }
            let terms: Vec<ComboTerm> = app
                .combo_terms_buffer
                .iter()
                .map(|(case, factor)| ComboTerm {
                    case: *case,
                    factor: *factor,
                })
                .collect();
            if ui.button("Apply").clicked() {
                if let Err(e) = app.apply_combination_terms(combo_id, None, terms) {
                    app.messages
                        .push(viewport_poc::app_state::AppMessage::error(e));
                }
            }
            if ui.button("Delete").clicked() {
                if let Err(e) = app.delete_load_combination(combo_id) {
                    app.messages
                        .push(viewport_poc::app_state::AppMessage::error(e));
                }
                app.editing_combination = None;
                app.combo_terms_buffer.clear();
            }
        });
        if !danglers.is_empty() {
            let ids: Vec<String> = danglers.iter().map(|d| format!("#{}", d.0)).collect();
            ui.label(
                egui::RichText::new(format!(
                    "Missing load case(s): {} \u{2014} fix or remove the term(s) to solve.",
                    ids.join(", ")
                ))
                .color(text.error),
            );
        }
    }
}

/// Staged term buffer for the currently-edited combination, read from the ///model (an empty vec when the combination has no terms yet).
fn terms_buffer_from_model(app: &ViewportApp, id: LoadCaseId) -> Vec<(LoadCaseId, f64)> {
    app.project
        .model
        .as_ref()
        .and_then(|m| m.combination(id))
        .map(|c| c.terms.iter().map(|t| (t.case, t.factor)).collect())
        .unwrap_or_default()
}
fn show_selection_loads(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let set = app.selected.clone();
    show_load_cases(app, ui);
    ui.separator();
    egui::CollapsingHeader::new("Selection")
        .default_open(true)
        .show(ui, |ui| match set.single() {
            Some(SelectionState::Node(id)) => {
                ui.strong(format!("Node N{}", id.0));
                show_node_loads(app, ui, id);
            }
            Some(SelectionState::Member(id)) => {
                ui.strong(format!("Member M{}", id.0));
                show_member_loads(app, ui, id);
            }
            _ if set.is_empty() => {
                ui.label("Nothing selected. Click a node or member.");
            }
            _ => {
                ui.strong(format!("{} entities selected", set.len()));
                ui.label(format!(
                    "{} node(s), {} member(s)",
                    set.node_count(),
                    set.member_count()
                ));
                ui.label(
                    egui::RichText::new("Select a single entity to edit its load.")
                        .color(text.help),
                );
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
                        app.commit_edit(candidate, format!("Clear load N{}", id.0));
                        app.messages
                            .push(viewport_poc::app_state::AppMessage::info(format!(
                                "Cleared load at node N{}",
                                id.0
                            )));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    } else if load_changed {
        if let Some(model) = app.project.model.as_ref() {
            let mut candidate = model.clone();
            match candidate.set_nodal_load(id, new_load) {
                Ok(()) => {
                    if candidate.revision() != model.revision() {
                        app.commit_edit(candidate, format!("Load N{}", id.0));
                    }
                }
                Err(e) => app
                    .messages
                    .push(viewport_poc::app_state::AppMessage::error(e.message())),
            }
        }
    }
}
fn show_member_loads(
    app: &mut ViewportApp,
    ui: &mut egui::Ui,
    id: viewport_poc::edit_model::MemberId,
) {
    app.sync_member_edit_buffer(id);

    // ──── Distributed load editing ————
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

    // ──── Point load editing ————
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

// ──── Selection: read-only results (Results module) ——————————————————————————————————————————————
fn show_selection_results(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);

    // Read-only member results need a single member; with a multi-selection we
    // simply show nothing here (the detail lives on the primary member via the
    // viewport's end-force / local-axis overlays).
    let Some(SelectionState::Member(id)) = app.selected.single() else {
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
                ui.label(
                    egui::RichText::new("(no current result \u{2014} solve first)")
                        .color(text.stale),
                );
                return;
            };
            if let Ok(f) = r.member_end_forces(idx) {
                ui.label("Local end forces:");

                //  The 4-column force table is intentionally wider than a
                // narrow panel; scroll it rather than let it pin the panel open.
                egui::ScrollArea::horizontal().show(ui, |ui| {
                    forces_grid(ui, "local", f);
                });
            }
            if let Ok(f) = r.member_end_forces_global(idx) {
                ui.label("Global end forces:");
                egui::ScrollArea::horizontal().show(ui, |ui| {
                    forces_grid(ui, "global", &f);
                });
            }
            if let Ok((x, y, z)) = r.member_local_axes(idx) {
                ui.label("Local axes (in global coords):");
                ui.label(format!("  x = ({:.4}, {:.4}, {:.4})", x[0], x[1], x[2]));
                ui.label(format!("  y = ({:.4}, {:.4}, {:.4})", y[0], y[1], y[2]));
                ui.label(format!("  z = ({:.4}, {:.4}, {:.4})", z[0], z[1], z[2]));
            }
        });
}

// ──── Camera ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
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
#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::{EditModel, NodeId};
    use viewport_poc::persistence;
    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// An app holding one 4 m beam; returns `(app, node)`.
    fn beam_app() -> (ViewportApp, NodeId) {
        let mut app = empty_app();
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        app.project.load_model("test".to_string(), m);
        app.rebuild_scene();
        (app, a)
    }
    #[test]
    fn mode_labels_default_and_icons() {
        assert_eq!(RightPanelMode::ALL.len(), 3);
        let labels: Vec<&str> = RightPanelMode::ALL.iter().map(|m| m.label()).collect();
        assert_eq!(labels, vec!["Inspector", "Agent", "Outline"]);
        assert_eq!(RightPanelMode::default(), RightPanelMode::Inspector);
        for mode in RightPanelMode::ALL {
            let _ = mode.icon();
        }
    }

    /// The persisted index round-trips for every mode, and an unset or     ///out-of-range value falls back to the default mode (Phase 159-D).
    #[test]
    fn mode_index_round_trips_and_tolerates_bad_values() {
        for mode in RightPanelMode::ALL {
            assert_eq!(RightPanelMode::from_index(Some(mode.index())), mode);
        }
        assert_eq!(RightPanelMode::from_index(None), RightPanelMode::Inspector);
        assert_eq!(
            RightPanelMode::from_index(Some(9)),
            RightPanelMode::Inspector
        );
    }

    /// Switching the right-panel mode must preserve the current selection and     ///never mutate the model (Phase 159-B, B4).
    #[test]
    fn switching_mode_preserves_selection_and_model() {
        let (mut app, a) = beam_app();
        app.select_only(SelectionState::Node(a));
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        for mode in RightPanelMode::ALL {
            app.set_right_panel_mode(mode);
            assert_eq!(app.right_panel_mode, mode);
            assert_eq!(
                app.selected.primary(),
                SelectionState::Node(a),
                "switching to {mode:?} must keep the selection"
            );
            assert_eq!(
                app.project.model.as_ref().unwrap().fingerprint(),
                fingerprint,
                "switching to {mode:?} must not touch the model"
            );
        }
    }

    /// Drawing each mode headlessly must not panic and must leave the     ///selection intact.
    #[test]
    fn all_modes_draw_headlessly() {
        let (mut app, a) = beam_app();
        app.select_only(SelectionState::Node(a));
        let ctx = egui::Context::default();
        for mode in RightPanelMode::ALL {
            app.set_right_panel_mode(mode);
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::pos2(0.0, 0.0),
                    egui::vec2(1000.0, 700.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                show_inspector(&mut app, ctx);
            });
        }
        assert_eq!(app.selected.primary(), SelectionState::Node(a));
    }

    // ──── Load-case manager (app-level helpers) ————————————————————————————————————————————————————
    #[test]
    fn add_load_case_commits_and_becomes_active() {
        let (mut app, _) = beam_app();
        let before = app.project.model.as_ref().unwrap().revision();
        let id = app.add_load_case("LL").unwrap();
        let m = app.project.model.as_ref().unwrap();
        assert_eq!(m.n_load_cases(), 2);
        assert_eq!(m.active_case(), id);
        assert!(
            m.revision() > before,
            "an engineering edit must bump the revision"
        );

        //Undo restores the single-case model.
        app.do_undo();
        assert_eq!(app.project.model.as_ref().unwrap().n_load_cases(), 1);
    }
    #[test]
    fn add_load_case_rejects_empty_name_without_editing() {
        let (mut app, _) = beam_app();
        assert!(app.add_load_case("  ").is_err());
        assert_eq!(app.project.model.as_ref().unwrap().n_load_cases(), 1);
    }
    #[test]
    fn delete_load_case_refuses_the_last_one() {
        let (mut app, _) = beam_app();
        let err = app.delete_load_case(LoadCaseId(0)).unwrap_err();
        assert!(err.contains("at least one"));
        assert_eq!(app.project.model.as_ref().unwrap().n_load_cases(), 1);
    }
    #[test]
    fn set_case_self_weight_commits_an_undoable_edit() {
        let (mut app, _) = beam_app();
        app.set_case_self_weight(LoadCaseId(0), true).unwrap();
        assert!(
            app.project
                .model
                .as_ref()
                .unwrap()
                .load_case(LoadCaseId(0))
                .unwrap()
                .self_weight
        );
        app.do_undo();
        assert!(
            !app.project
                .model
                .as_ref()
                .unwrap()
                .load_case(LoadCaseId(0))
                .unwrap()
                .self_weight
        );
    }
    #[test]
    fn switching_the_active_case_is_view_state() {
        let (mut app, _) = beam_app();
        let ll = app.add_load_case("LL").unwrap();
        if let Some(m) = app.project.model.as_mut() {
            m.set_active_case(LoadCaseId(0)).unwrap();
        }
        let revision = app.project.model.as_ref().unwrap().revision();
        if let Some(m) = app.project.model.as_mut() {
            m.set_active_case(ll).unwrap();
        }
        assert_eq!(
            app.project.model.as_ref().unwrap().revision(),
            revision,
            "switching the active case must not dirty the results"
        );
        assert_eq!(app.project.model.as_ref().unwrap().active_case(), ll);
    }
    #[test]
    fn combination_terms_apply_and_delete() {
        let (mut app, _) = beam_app();
        let ll = app.add_load_case("LL").unwrap();
        let combo = app.add_load_combination("1.4LL").unwrap();
        app.apply_combination_terms(
            combo,
            None,
            vec![ComboTerm {
                case: ll,
                factor: 1.4,
            }],
        )
        .unwrap();
        let m = app.project.model.as_ref().unwrap();
        let terms = &m.combination(combo).unwrap().terms;
        assert_eq!(terms.len(), 1);
        assert_eq!(terms[0].factor, 1.4);
        app.delete_load_combination(combo).unwrap();
        assert_eq!(app.project.model.as_ref().unwrap().n_combinations(), 0);

        //The referenced case survives its combination's deletion.
        assert!(app.project.model.as_ref().unwrap().load_case(ll).is_some());
    }
    #[test]
    fn applying_combination_terms_names_the_missing_case() {
        let (mut app, _) = beam_app();
        let combo = app.add_load_combination("Bad").unwrap();
        let err = app
            .apply_combination_terms(
                combo,
                None,
                vec![ComboTerm {
                    case: LoadCaseId(42),
                    factor: 1.0,
                }],
            )
            .unwrap_err();
        assert!(err.contains("#42"), "error should name the case: {err}");
        assert_eq!(app.project.model.as_ref().unwrap().n_combinations(), 1);
    }

    // ── Member end releases (inspector surface) ──

    /// Drawing the Model module with a member selected exercises the End
    /// Releases panel; toggling a release then goes through the shared,
    /// undoable edit path.
    #[test]
    fn end_release_panel_draws_and_edits_the_model() {
        use viewport_poc::edit_model::{MemberId, SupportSpec};
        let (mut app, _) = beam_app();
        let a = app.project.model.as_ref().unwrap().nodes()[0].id;
        {
            let m = app.project.model.as_mut().unwrap();
            m.set_support(a, SupportSpec::fixed()).unwrap();
        }
        let mid = app.project.model.as_ref().unwrap().members()[0].id;
        app.select_only(SelectionState::Member(mid));
        app.module = WorkModule::Model;
        let ctx = egui::Context::default();
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(1000.0, 700.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            show_inspector(&mut app, ctx);
        });

        //Drawing the panel must not mutate the model.
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();

        //Apply a release the way the panel does (clone + commit + undo).
        let releases = [false, false, false, true, true, true];
        {
            let model = app.project.model.as_ref().unwrap();
            let mut candidate = model.clone();
            candidate.set_member_releases(mid, releases).unwrap();
            app.commit_edit(candidate, "Releases M0");
        }
        assert_eq!(
            app.project.model.as_ref().unwrap().member_releases(mid),
            Some(releases)
        );
        assert_ne!(
            app.project.model.as_ref().unwrap().fingerprint(),
            fingerprint
        );
        app.do_undo();
        assert_eq!(
            app.project.model.as_ref().unwrap().member_releases(mid),
            Some([false; 6])
        );
    }
}
