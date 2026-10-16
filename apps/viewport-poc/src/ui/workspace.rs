//! Centre workspace: the active module's content plus the centre-scoped
//! bottom output dock (Phase 159-B2/C, revised for the 方案 B chrome).
//!
//! # Why a workspace module
//!
//! The left **Project** panel and the right panel are root-level
//! [`egui::SidePanel`]s, so the remaining screen region is the *centre
//! workspace*.  Everything that belongs to the centre — the active content and
//! the resizable bottom output dock — is laid out **inside** that region with
//! the nested [`egui::TopBottomPanel::show_inside`] API.  A root-level
//! `TopBottomPanel::bottom(...).show(ctx, …)` would span the *whole* window
//! width, which is exactly what this phase forbids: the dock's left and right
//! edges must align with the centre workspace (between the two side panels).
//!
//! # Layout hierarchy
//!
//! ```text
//! Status Bar                (root bottom panel — spans the full width)
//! Main Workspace (horizontal):
//!   Left Project Panel      (root SidePanel::left, full main-workspace height)
//!   Centre Workspace        (root CentralPanel):
//!       Flexible content    (the active work module's body)
//!       Bottom Dock         (nested TopBottomPanel::bottom::show_inside, resizable)
//!   Right Auxiliary Panel   (root SidePanel::right, full main-workspace height)
//! ```
//!
//! # One switcher: the top module strip
//!
//! The top module strip (`ui/module.rs`, `Model | Section | Load | Analysis |
//! Results`) is the **single** switcher.  There is no separate centre tab bar:
//! [`WorkModule`] drives the centre content directly via
//! [`CentreContent::for_module`]:
//!
//! | active module | centre content |
//! |---|---|
//! | `Model` / `Section` / `Load` | the 3-D viewport |
//! | `Analysis` | the analysis job panel (shared body with the inspector) |
//! | `Results` | the tabular results (nodal displacements / member end forces) |
//!
//! This is pure **view** state: switching a module only writes
//! [`ViewportApp::module`](crate::ViewportApp) and never touches the `Project`,
//! the `EditModel`, the `EditHistory`, the selection, the camera or the
//! display options, so camera / display / selection / model revision /
//! analysis status are all preserved across module switches (asserted by
//! test).
//!
//! [`Project`]: viewport_poc::app_state::Project

use egui;

use structural_analysis::Dof3D;
use viewport_poc::app_state::{AnalysisStatus, ModelSummary};

use super::theme::{self, TextPalette};
use super::{WorkModule, fmt, forces_grid};
use crate::ViewportApp;

/// Which centre body the active work module shows.
///
/// A tiny **typed mapping** of the module strip onto the centre: it is the
/// single place that decides what the centre displays, so the module strip is
/// the one and only switcher.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CentreContent {
    /// The 3-D viewport (Model / Section / Load modules).
    Viewport,
    /// The analysis job panel.
    Analysis,
    /// The tabular analysis results.
    Results,
}

impl CentreContent {
    /// The centre body for `module`.
    ///
    /// `Model`, `Section` and `Load` all show the viewport — the Section
    /// builder and load editing live in the right panel, so the centre stays
    /// the 3-D view for every editing module.
    pub(crate) const fn for_module(module: WorkModule) -> Self {
        match module {
            WorkModule::Model | WorkModule::Section | WorkModule::Load => CentreContent::Viewport,
            WorkModule::Analysis => CentreContent::Analysis,
            WorkModule::Results => CentreContent::Results,
        }
    }
}

/// Draw the centre workspace: the active module's content plus the
/// centre-scoped, resizable bottom output dock.
///
/// Must be called **after** the left / right side panels and the status bar so
/// the root [`egui::CentralPanel`] naturally receives the centre region only.
pub fn show_center_workspace(app: &mut ViewportApp, ctx: &egui::Context) {
    egui::CentralPanel::default().show(ctx, |ui| {
        // ── Bottom output dock (nested → centre-scoped width) ───────────
        // Stays visible for every module.  Laying it out *inside* this `Ui` is
        // what keeps its left/right edges aligned with the centre workspace.
        // The dock code is unchanged — only the (removed) tab bar used to sit
        // above it.
        super::status::show_output_dock(app, ui);

        // ── Flexible content region (the active module) ─────────────────
        match CentreContent::for_module(app.module) {
            CentreContent::Viewport => super::viewport::show_viewport(app, ui),
            CentreContent::Analysis => show_analysis_content(app, ui),
            CentreContent::Results => show_results_content(app, ui),
        }
    });
}

/// Analysis content: the analysis job panel (the same body the right panel's
/// Analysis module shows — mirrored, not duplicated, so both read the one
/// [`Project`](viewport_poc::app_state::Project)).
fn show_analysis_content(app: &mut ViewportApp, ui: &mut egui::Ui) {
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            super::inspector::show_analysis_section(app, ui);
        });
}

/// Results content: the tabular analysis results (nodal displacements /
/// reactions and member end forces).
fn show_results_content(app: &ViewportApp, ui: &mut egui::Ui) {
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            results_view(app, ui);
        });
}

/// The tabular results body: nodal displacements / reactions and member end
/// forces, built from the current analysis result (real data only).
fn results_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let Some(model) = app.project.model.as_ref() else {
        ui.label("No model loaded");
        return;
    };
    let Some(result) = app.analysis_result() else {
        ui.label(egui::RichText::new("(no current result \u{2014} solve first)").color(text.stale));
        return;
    };

    // A short header repeats the headline status so the panel is self-describing.
    let summary = ModelSummary::from_model(model);
    ui.label(
        theme::small(format!(
            "{} node(s) \u{b7} {} member(s) \u{b7} {}",
            summary.n_nodes,
            summary.n_members,
            match app.project.analysis_status() {
                AnalysisStatus::Current => "analysis current",
                AnalysisStatus::Stale => "analysis STALE \u{2014} re-run",
                AnalysisStatus::Failed => "analysis FAILED",
                AnalysisStatus::NeverSolved => "analysis not run",
            }
        ))
        .color(text.help),
    );
    ui.separator();

    egui::CollapsingHeader::new(format!(
        "Nodal displacements / reactions ({})",
        model.n_nodes()
    ))
    .default_open(true)
    .show(ui, |ui| {
        egui::Grid::new("node_results_grid")
            .striped(true)
            .num_columns(5)
            .spacing([12.0, 3.0])
            .show(ui, |ui| {
                ui.strong("Node");
                ui.strong("ux");
                ui.strong("uy");
                ui.strong("uz");
                ui.strong("|R|max");
                ui.end_row();
                for (i, node) in model.nodes().iter().enumerate() {
                    let ux = result.displacement(i, Dof3D::Ux).unwrap_or(0.0);
                    let uy = result.displacement(i, Dof3D::Uy).unwrap_or(0.0);
                    let uz = result.displacement(i, Dof3D::Uz).unwrap_or(0.0);
                    let rmax = Dof3D::ALL
                        .iter()
                        .map(|d| result.reaction(i, *d).unwrap_or(0.0).abs())
                        .fold(0.0_f64, f64::max);
                    ui.label(theme::mono(format!("N{}", node.id.0)));
                    ui.label(theme::mono(fmt(ux)));
                    ui.label(theme::mono(fmt(uy)));
                    ui.label(theme::mono(fmt(uz)));
                    ui.label(theme::mono(fmt(rmax)));
                    ui.end_row();
                }
            });
    });

    egui::CollapsingHeader::new(format!("Member end forces ({})", model.n_members()))
        .default_open(false)
        .show(ui, |ui| {
            for (i, member) in model.members().iter().enumerate() {
                if let Ok(f) = result.member_end_forces_global(i) {
                    ui.label(format!("M{} (global):", member.id.0));
                    forces_grid(ui, &format!("workspace_{}", member.id.0), &f);
                }
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::EditModel;
    use viewport_poc::persistence;
    use viewport_poc::selection::SelectionState;

    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// An app with one 4 m beam; returns `(app, node)`.
    fn beam_app() -> (ViewportApp, viewport_poc::edit_model::NodeId) {
        let mut app = empty_app();
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        app.project.load_model("test".to_string(), m);
        app.rebuild_scene();
        (app, a)
    }

    fn run_frame(app: &mut ViewportApp, ctx: &egui::Context) {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(1200.0, 800.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            show_center_workspace(app, ctx);
        });
    }

    /// The centre content follows the active module exactly as specified: the
    /// editing modules show the viewport, Analysis the job panel, Results the
    /// tables.
    #[test]
    fn centre_content_follows_the_active_module() {
        assert_eq!(
            CentreContent::for_module(WorkModule::Model),
            CentreContent::Viewport
        );
        assert_eq!(
            CentreContent::for_module(WorkModule::Section),
            CentreContent::Viewport
        );
        assert_eq!(
            CentreContent::for_module(WorkModule::Load),
            CentreContent::Viewport
        );
        assert_eq!(
            CentreContent::for_module(WorkModule::Analysis),
            CentreContent::Analysis
        );
        assert_eq!(
            CentreContent::for_module(WorkModule::Results),
            CentreContent::Results
        );
        // Every module is mapped (no module falls outside the table).
        for module in WorkModule::ALL {
            let _ = CentreContent::for_module(module);
        }
    }

    /// Switching the active module is pure view state: camera, display
    /// options, selection, model revision and analysis status must all be
    /// preserved (方案 B requirement).
    #[test]
    fn switching_module_preserves_state() {
        let (mut app, a) = beam_app();
        app.select_only(SelectionState::Node(a));
        app.apply_support_quick(a, viewport_poc::edit_model::SupportSpec::fixed())
            .unwrap();
        app.run_solve();

        let status = app.project.analysis_status();
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        let revision = app.project.model.as_ref().unwrap().revision();
        let camera = (
            app.camera.target,
            app.camera.azimuth,
            app.camera.elevation,
            app.camera.distance,
            app.camera.fov,
        );
        let display = (
            app.display.mode,
            app.display.show_grid,
            app.display.show_global_axes,
            app.display.show_node_labels,
            app.display.show_diagrams,
        );

        for module in WorkModule::ALL {
            app.module = module;
            assert_eq!(app.module, module);
            assert_eq!(app.selected.primary(), SelectionState::Node(a));
            assert_eq!(
                (
                    app.camera.target,
                    app.camera.azimuth,
                    app.camera.elevation,
                    app.camera.distance,
                    app.camera.fov,
                ),
                camera,
                "switching to {module:?} must not touch the camera"
            );
            assert_eq!(
                (
                    app.display.mode,
                    app.display.show_grid,
                    app.display.show_global_axes,
                    app.display.show_node_labels,
                    app.display.show_diagrams,
                ),
                display,
                "switching to {module:?} must not touch the display options"
            );
            assert_eq!(
                app.project.model.as_ref().unwrap().fingerprint(),
                fingerprint,
                "switching to {module:?} must not touch the model"
            );
            assert_eq!(app.project.model.as_ref().unwrap().revision(), revision);
            assert_eq!(app.project.analysis_status(), status);
        }
    }

    /// Drawing every module headlessly must not panic and must leave the
    /// selection, model and analysis untouched.
    #[test]
    fn all_modules_draw_headlessly_without_mutation() {
        let ctx = egui::Context::default();
        let (mut app, a) = beam_app();
        app.select_only(SelectionState::Node(a));
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        for module in WorkModule::ALL {
            app.module = module;
            run_frame(&mut app, &ctx);
        }
        assert_eq!(app.selected.primary(), SelectionState::Node(a));
        assert_eq!(
            app.project.model.as_ref().unwrap().fingerprint(),
            fingerprint
        );
    }
    /// End-to-end through the centre workspace: a two-case model solves each
    /// case, the result selector switches which one the Results tables draw,
    /// and a combination is drawable too (superposed, never re-solved).
    #[test]
    fn results_content_follows_the_selected_case_and_combination() {
        use viewport_poc::edit_model::{ComboTerm, LoadCaseId, NodalLoadSpec, SupportSpec};

        let (mut app, a) = beam_app();
        let b = app.project.model.as_ref().unwrap().nodes()[1].id;
        app.apply_support_quick(a, SupportSpec::fixed()).unwrap();
        {
            let m = app.project.model.as_mut().unwrap();
            m.set_nodal_load(b, NodalLoadSpec::new(0.0, -1e3, 0.0, 0.0, 0.0, 0.0))
                .unwrap();
            let ll = m.add_load_case("LL").unwrap();
            m.set_active_case(ll).unwrap();
            m.set_nodal_load(b, NodalLoadSpec::new(2e3, 0.0, 0.0, 0.0, 0.0, 0.0))
                .unwrap();
            m.set_active_case(LoadCaseId(0)).unwrap();
            let combo = m.add_load_combination("1.0DL+1.0LL").unwrap();
            m.set_combination_terms(
                combo,
                vec![
                    ComboTerm {
                        case: LoadCaseId(0),
                        factor: 1.0,
                    },
                    ComboTerm {
                        case: ll,
                        factor: 1.0,
                    },
                ],
            )
            .unwrap();
        }
        app.run_solve();

        let dl = app.analysis_result().unwrap().displacements.clone();
        app.project.set_active_case(LoadCaseId(1));
        let ll = app.analysis_result().unwrap().displacements.clone();
        assert_ne!(dl, ll, "each load case must report its own response");

        // Drawing the centre Results view (selector + tables) for both a
        // single case and a combination must not panic or mutate the model.
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        let ctx = egui::Context::default();
        app.module = WorkModule::Results;
        run_frame(&mut app, &ctx);
        app.project.set_active_combination(LoadCaseId(0));
        let combo_disps = app.analysis_result().unwrap().displacements.clone();
        for (c, (d, l)) in combo_disps.iter().zip(dl.iter().zip(ll.iter())) {
            assert!((c - (d + l)).abs() < 1e-9 * (1.0 + c.abs()));
        }
        run_frame(&mut app, &ctx);
        assert_eq!(
            app.project.model.as_ref().unwrap().fingerprint(),
            fingerprint,
            "viewing results must not touch the model"
        );
    }
}
