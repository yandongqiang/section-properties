//! Top menu bar and the MIDAS-style ribbon toolbar below it.

use egui;

use viewport_poc::camera::StandardView;
use viewport_poc::display::{CutAxis, ResultViewMode};
use viewport_poc::model::PresetModel;

use super::gl_renderer::{self, RenderMode};
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
            inspector_toggle(app, ui);
        });
        // Ribbon row, on the line directly below the menu bar.
        ui.separator();
        super::ribbon::show_ribbon(app, ui);
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
            app.clear_selection();
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
        if ui
            .add_enabled(
                app.display.has_hidden(),
                egui::Button::new("Show all (unhide)"),
            )
            .on_hover_text("Unhide every node and member hidden from the viewport")
            .clicked()
        {
            app.show_all_hidden();
            ui.close_menu();
        }
        ui.separator();
        // Section cut plane (Phase P5).
        ui.menu_button("Section cut", |ui| {
            let mut enabled = app.display.section_cut.enabled;
            if ui
                .checkbox(&mut enabled, "Enable cut plane")
                .on_hover_text("Clip the model with a plane so the interior is visible")
                .changed()
            {
                app.set_cut_enabled(enabled);
            }

            let cut_enabled = app.display.section_cut.enabled;
            ui.add_enabled_ui(cut_enabled, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Axis:");
                    for axis in CutAxis::ALL {
                        let selected = app.display.section_cut.axis == axis
                            && app.display.section_cut.normal.is_none();
                        if ui.selectable_label(selected, axis.label()).clicked() {
                            app.set_cut_axis(axis);
                        }
                    }
                });
                let mut flip = app.display.section_cut.flip;
                if ui.checkbox(&mut flip, "Flip kept side").changed() {
                    app.display.section_cut.flip = flip;
                }
                // Phase P7-1: fill the open cut with a solid-looking cap
                // (back-face fill).  GPU only — the Software renderer always
                // shows the open cut.
                let mut cap = app.display.section_cut.cap;
                if ui
                    .checkbox(&mut cap, "Cap cut faces")
                    .on_hover_text(
                        "Fill the hollow interior at the cut with a solid cap \
                         (GPU renderer only; ignored in Software mode)",
                    )
                    .changed()
                {
                    app.display.section_cut.cap = cap;
                }
                // Slider span follows the current model bounds (any model size).
                if let Some((lo, hi)) = app.cut_range() {
                    let range = if (hi - lo).abs() < 1e-9 {
                        (lo - 0.5)..=(hi + 0.5)
                    } else {
                        lo..=hi
                    };
                    let mut offset = app.display.section_cut.offset;
                    if ui
                        .add(egui::Slider::new(&mut offset, range).text("Position"))
                        .changed()
                    {
                        app.display.section_cut.offset = offset;
                    }
                }
                if ui
                    .button("Center on model")
                    .on_hover_text("Re-centre the plane on the model bounds")
                    .clicked()
                {
                    app.recenter_section_cut();
                }
            });
        });
        ui.separator();
        ui.label(egui::RichText::new("Renderer").strong());
        let gpu_active = gl_renderer::mode() == RenderMode::Gpu;
        if ui
            .selectable_label(!gpu_active, RenderMode::Software.label())
            .on_hover_text("Legacy software-projected view (always available)")
            .clicked()
        {
            gl_renderer::set_mode(RenderMode::Software);
            ui.close_menu();
        }
        let gpu_available = gl_renderer::available();
        let mut gpu_label = ui.add_enabled(
            gpu_available,
            egui::SelectableLabel::new(gpu_active, RenderMode::Gpu.label()),
        );
        gpu_label = gpu_label.on_hover_text("GPU 3-D view with depth testing");
        if !gpu_available {
            gpu_label = gpu_label.on_disabled_hover_text("GPU renderer unavailable — see Messages");
        }
        if gpu_label.clicked() && gl_renderer::set_mode(RenderMode::Gpu) {
            ui.close_menu();
        }
        // GPU options (Phase P2).  Harmless in software mode; disabled when the
        // GPU renderer never came up.
        let mut shading = gl_renderer::shading_enabled();
        if ui
            .add_enabled(gpu_available, egui::Checkbox::new(&mut shading, "Shading / lighting"))
            .on_hover_text("Lambert shading on the extruded members and nodes")
            .changed()
        {
            gl_renderer::set_shading(shading);
        }
        let mut msaa = gl_renderer::msaa_enabled();
        if ui
            .add_enabled(gpu_available, egui::Checkbox::new(&mut msaa, "MSAA (4×)"))
            .on_hover_text("Multisampled anti-aliasing on the offscreen target")
            .changed()
        {
            gl_renderer::set_msaa(msaa);
        }
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
        ui.label("Shift+click: toggle in selection");
        ui.label("Ctrl+click: add to selection");
        ui.label("Shift-drag: box select");
        ui.label("Ctrl+A: select all visible");
        ui.label("Delete: delete selection");
        ui.label("Esc: clear selection");
        ui.label("Ctrl+Z / Ctrl+Y: undo / redo");
        ui.separator();
        if ui.button(theme::toggle_label(app.dark_mode)).clicked() {
            app.dark_mode = !app.dark_mode;
            ui.close_menu();
        }
    });
}
