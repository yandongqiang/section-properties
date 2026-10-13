//! Top menu bar and the compact quick-action row.

use egui;

use viewport_poc::app_state::AnalysisStatus;
use viewport_poc::camera::StandardView;
use viewport_poc::display::ResultViewMode;
use viewport_poc::model::PresetModel;

use super::module::WorkModule;
use super::theme;
use crate::ViewportApp;

/// Draw the application menu bar as a top panel.
pub fn show_menu_bar(app: &mut ViewportApp, ctx: &egui::Context) {
    egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            file_menu(app, ui, ctx);
            edit_menu(app, ui);
            view_menu(app, ui);
            model_menu(app, ui);
            analysis_menu(app, ui);
            results_menu(app, ui);
            help_menu(app, ui);

            ui.separator();
            quick_actions(app, ui);

            ui.separator();
            view_toolbar(app, ui);

            ui.separator();
            inspector_toggle(app, ui);
        });
    });
}

fn file_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("File", |ui| {
        ui.menu_button("New", |ui| {
            for &preset in PresetModel::all() {
                let is_active = app.project.name == preset.name();
                if ui
                    .add_enabled(!is_active, egui::Button::new(preset.name()))
                    .clicked()
                {
                    app.request_load_preset(preset);
                    ui.close_menu();
                }
            }
            ui.separator();
            if ui.button("Empty model").clicked() {
                app.request_new();
                ui.close_menu();
            }
        });
        ui.separator();
        if ui.button("Open...").clicked() {
            app.request_open();
            ui.close_menu();
        }
        if ui.button("Save").clicked() {
            app.do_save();
            ui.close_menu();
        }
        if ui.button("Save As...").clicked() {
            app.do_save_as();
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Clear project").clicked() {
            app.request_clear();
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Exit").clicked() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    });
}

fn edit_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("Edit", |ui| {
        if ui
            .add_enabled(app.history.can_undo(), egui::Button::new("Undo"))
            .clicked()
        {
            app.do_undo();
            ui.close_menu();
        }
        if ui
            .add_enabled(app.history.can_redo(), egui::Button::new("Redo"))
            .clicked()
        {
            app.do_redo();
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Clear selection").clicked() {
            app.selection = viewport_poc::selection::SelectionState::None;
            ui.close_menu();
        }
    });
}

fn view_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("View", |ui| {
        ui.checkbox(&mut app.display.show_grid, "Ground grid");
        ui.checkbox(&mut app.display.show_global_axes, "Global axes");
        ui.checkbox(&mut app.display.show_node_labels, "Node labels");
        ui.checkbox(&mut app.display.show_local_axes, "Local axes (selected)");
        ui.separator();
        ui.label(egui::RichText::new("Standard views").strong());
        for view in StandardView::ALL {
            if ui.button(view.label()).clicked() {
                app.set_standard_view(view);
                ui.close_menu();
            }
        }
        ui.separator();
        if ui.button("Fit / Zoom extents").clicked() {
            app.fit_view();
            ui.close_menu();
        }
        if ui.button("Reset view").clicked() {
            app.camera.reset();
            ui.close_menu();
        }
        ui.separator();
        let mut show_dock = !app.bottom_dock_collapsed;
        if ui
            .checkbox(&mut show_dock, "Messages / Results dock (Ctrl+M)")
            .changed()
        {
            app.bottom_dock_collapsed = !show_dock;
        }
        let mut show_inspector = !app.inspector_collapsed;
        if ui
            .checkbox(&mut show_inspector, "Inspector panel (Ctrl+I)")
            .changed()
        {
            app.inspector_collapsed = !show_inspector;
        }
        ui.separator();
        if ui.button(theme::toggle_label(app.dark_mode)).clicked() {
            app.dark_mode = !app.dark_mode;
            ui.close_menu();
        }
    });
}

/// Compact view-control group in the menu bar row.
///
/// Fit / zoom-extents plus the standard preset viewpoints.
fn view_toolbar(app: &mut ViewportApp, ui: &mut egui::Ui) {
    if ui.button("Fit").on_hover_text("Fit / zoom extents (F)").clicked() {
        app.fit_view();
    }
    for view in StandardView::ALL {
        if ui.button(view.label()).clicked() {
            app.set_standard_view(view);
        }
    }
}

/// Always-visible toggle for the right-hand inspector panel.
///
/// This lives in the menu-bar row — a [`egui::TopBottomPanel::top`] drawn
/// unconditionally by [`show_menu_bar`] — so it stays reachable even when the
/// inspector is collapsed and its own in-panel `Hide` chevron no longer
/// exists.  It flips the same `inspector_collapsed` field as the View-menu
/// checkbox and the Ctrl+I shortcut (single source of truth).
fn inspector_toggle(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let label = if app.inspector_collapsed {
        "\u{25b6} Inspector"
    } else {
        "\u{25c0} Inspector"
    };
    if ui
        .button(label)
        .on_hover_text("Show/hide the inspector panel (Ctrl+I)")
        .clicked()
    {
        app.inspector_collapsed = !app.inspector_collapsed;
    }
}

fn model_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("Model", |ui| {
        for module in [WorkModule::Model, WorkModule::Load] {
            if ui.button(module.label()).clicked() {
                app.module = module;
                ui.close_menu();
            }
        }
        ui.separator();
        if ui.button("Add node / member").clicked() {
            app.module = WorkModule::Model;
            ui.close_menu();
        }
    });
}

fn analysis_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("Analysis", |ui| {
        let solve_enabled = app.can_run_analysis();
        if ui
            .add_enabled(solve_enabled, egui::Button::new("Solve"))
            .clicked()
        {
            app.run_solve();
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Solver options...").clicked() {
            app.module = WorkModule::Analysis;
            ui.close_menu();
        }
    });
}

fn results_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("Results", |ui| {
        for mode in ResultViewMode::ALL {
            let selected = app.display.mode == mode;
            if ui
                .add(egui::SelectableLabel::new(selected, mode.label()))
                .clicked()
            {
                app.display.mode = mode;
                ui.close_menu();
            }
        }
        ui.separator();
        // Convenience toggle only — the detailed diagram controls (M/N/V,
        // all-members, scale) live in the inspector's Results section.
        ui.checkbox(&mut app.display.show_diagrams, "Internal-force diagrams");
    });
}

fn help_menu(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.menu_button("Help", |ui| {
        ui.label(egui::RichText::new("Shortcuts").strong());
        ui.label("Drag: orbit");
        ui.label("R-drag: pan");
        ui.label("Scroll: zoom");
        ui.label("R: reset view");
        ui.label("Click: select");
        ui.label("Esc: clear selection");
        ui.label("Ctrl+Z / Ctrl+Y: undo / redo");
        ui.separator();
        if ui.button(theme::toggle_label(app.dark_mode)).clicked() {
            app.dark_mode = !app.dark_mode;
            ui.close_menu();
        }
    });
}

/// Compact quick-action row folded from the old flat toolbar.
fn quick_actions(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let solve_enabled = app.can_run_analysis();
    if ui
        .add_enabled(solve_enabled, egui::Button::new("Solve"))
        .clicked()
    {
        app.run_solve();
    }
    if ui
        .add_enabled(app.history.can_undo(), egui::Button::new("Undo"))
        .clicked()
    {
        app.do_undo();
    }
    if ui
        .add_enabled(app.history.can_redo(), egui::Button::new("Redo"))
        .clicked()
    {
        app.do_redo();
    }
    ui.separator();
    match app.project.analysis_status() {
        AnalysisStatus::Stale => {
            ui.label(
                egui::RichText::new("\u{26a0} Results are stale \u{2014} re-solve to update")
                    .color(theme::STALE_C),
            );
        }
        AnalysisStatus::Failed => {
            ui.label(
                egui::RichText::new("\u{26a0} Last solve failed \u{2014} fix and re-solve")
                    .color(theme::ERROR_C),
            );
        }
        _ => {}
    }
}
