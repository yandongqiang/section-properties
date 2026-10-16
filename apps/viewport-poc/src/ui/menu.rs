//! Application menu bar.
//!
//! Menu items no longer call the handlers directly: each one constructs a
//! [`Command`] and dispatches it through
//! [`ViewportApp::execute_command`](crate::ViewportApp) — the single
//! execution path shared with the command palette and the viewport context
//! menu (Phase 159-A).  Keyboard shortcuts keep calling the same handlers
//! directly.
//!
//! Phase 159-B removed the CAD ribbon that used to sit below the menu bar;
//! the menus (plus the palette and the viewport context menu) are now the
//! complete surface for every former ribbon action.  The `Boundary` menu hosts
//! the Support / Node Load / Member Load quick-adds.

use egui;

use viewport_poc::camera::StandardView;
use viewport_poc::display::{CutAxis, ResultViewMode};
use viewport_poc::model::PresetModel;

use super::commands::Command;
use super::gl_renderer::{self, RenderMode};
use super::module::WorkModule;
use super::theme;
use crate::ViewportApp;

/// Draw the application menu bar as a top panel.
pub fn show_menu_bar(app: &mut ViewportApp, ctx: &egui::Context) {
    egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            file_menu(app, ui, ctx);
            edit_menu(app, ui, ctx);
            view_menu(app, ui, ctx);
            model_menu(app, ui, ctx);
            boundary_menu(app, ui, ctx);
            analysis_menu(app, ui, ctx);
            results_menu(app, ui, ctx);
            help_menu(app, ui, ctx);
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
                    app.execute_command(Command::LoadPreset(preset), ctx);
                    ui.close_menu();
                }
            }
            ui.separator();
            if ui.button("Empty model").clicked() {
                app.execute_command(Command::New, ctx);
                ui.close_menu();
            }
        });
        ui.separator();
        if ui
            .button("Open\u{2026}")
            .on_hover_text("Choose a .saproject file to open (native file dialog)")
            .clicked()
        {
            app.execute_command(Command::Open, ctx);
            ui.close_menu();
        }
        if ui
            .button("Save")
            .on_hover_text("Save to the current file path (asks for a location when none)")
            .clicked()
        {
            app.execute_command(Command::Save, ctx);
            ui.close_menu();
        }
        if ui
            .button("Save As\u{2026}")
            .on_hover_text("Choose where to save the project (native file dialog)")
            .clicked()
        {
            app.execute_command(Command::SaveAs, ctx);
            ui.close_menu();
        }
        ui.separator();
        ui.menu_button("Export", |ui| {
            if ui
                .button("Viewport screenshot\u{2026}")
                .on_hover_text(
                    "Save the viewport as a BMP image in Pictures\\section-properties (Ctrl+Shift+S)",
                )
                .clicked()
            {
                // Phase P7-3 C1: asks egui/eframe to capture the composited
                // window; the reply is handled in `main.rs` and the saved
                // path is reported in the Messages dock.
                app.execute_command(Command::Screenshot, ctx);
                ui.close_menu();
            }
        });
        ui.separator();
        if ui.button("Clear project").clicked() {
            app.execute_command(Command::ClearProject, ctx);
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Exit").clicked() {
            app.execute_command(Command::Exit, ctx);
        }
    });
}

fn edit_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Edit", |ui| {
        if ui
            .add_enabled(
                app.command_is_enabled(Command::Undo),
                egui::Button::new("Undo"),
            )
            .clicked()
        {
            app.execute_command(Command::Undo, ctx);
            ui.close_menu();
        }
        if ui
            .add_enabled(
                app.command_is_enabled(Command::Redo),
                egui::Button::new("Redo"),
            )
            .clicked()
        {
            app.execute_command(Command::Redo, ctx);
            ui.close_menu();
        }
        ui.separator();
        if ui
            .add_enabled(
                app.command_is_enabled(Command::ClearSelection),
                egui::Button::new("Clear selection"),
            )
            .clicked()
        {
            app.execute_command(Command::ClearSelection, ctx);
            ui.close_menu();
        }
    });
}

fn view_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("View", |ui| {
        // Overlay checkboxes: the click flips a local mirror, the command
        // flips the real state (net effect unchanged, single dispatch path).
        let mut show_grid = app.display.show_grid;
        if ui.checkbox(&mut show_grid, "Ground grid").changed() {
            app.execute_command(Command::ToggleGrid, ctx);
        }
        let mut show_axes = app.display.show_global_axes;
        if ui.checkbox(&mut show_axes, "Global axes").changed() {
            app.execute_command(Command::ToggleGlobalAxes, ctx);
        }
        let mut show_node_labels = app.display.show_node_labels;
        if ui.checkbox(&mut show_node_labels, "Node labels").changed() {
            app.execute_command(Command::ToggleNodeLabels, ctx);
        }
        let mut show_local_axes = app.display.show_local_axes;
        if ui
            .checkbox(&mut show_local_axes, "Local axes (selected)")
            .changed()
        {
            app.execute_command(Command::ToggleLocalAxes, ctx);
        }
        if ui
            .add_enabled(
                app.command_is_enabled(Command::ShowAll),
                egui::Button::new("Show all (unhide)"),
            )
            .on_hover_text("Unhide every node and member hidden from the viewport")
            .clicked()
        {
            app.execute_command(Command::ShowAll, ctx);
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
                app.execute_command(Command::ToggleSectionCut, ctx);
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
                    app.execute_command(Command::ToggleCap, ctx);
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
        ui.label(theme::header("Renderer"));
        let gpu_active = gl_renderer::mode() == RenderMode::Gpu;
        if ui
            .selectable_label(!gpu_active, RenderMode::Software.label())
            .on_hover_text("Legacy software-projected view (always available)")
            .clicked()
        {
            app.execute_command(Command::RendererSoftware, ctx);
            ui.close_menu();
        }
        let gpu_available = gl_renderer::available();
        let mut gpu_label = ui.add_enabled(
            app.command_is_enabled(Command::RendererGpu),
            egui::SelectableLabel::new(gpu_active, RenderMode::Gpu.label()),
        );
        gpu_label = gpu_label.on_hover_text("GPU 3-D view with depth testing");
        if !gpu_available {
            gpu_label = gpu_label.on_disabled_hover_text("GPU renderer unavailable — see Messages");
        }
        if gpu_label.clicked() {
            app.execute_command(Command::RendererGpu, ctx);
            ui.close_menu();
        }
        // GPU options (Phase P2).  Harmless in software mode; disabled when the
        // GPU renderer never came up.
        let mut shading = gl_renderer::shading_enabled();
        if ui
            .add_enabled(
                app.command_is_enabled(Command::ToggleShading),
                egui::Checkbox::new(&mut shading, "Shading / lighting"),
            )
            .on_hover_text("Lambert shading on the extruded members and nodes")
            .changed()
        {
            app.execute_command(Command::ToggleShading, ctx);
        }
        let mut msaa = gl_renderer::msaa_enabled();
        if ui
            .add_enabled(
                app.command_is_enabled(Command::ToggleMsaa),
                egui::Checkbox::new(&mut msaa, "MSAA (4\u{d7})"),
            )
            .on_hover_text("Multisampled anti-aliasing on the offscreen target")
            .changed()
        {
            app.execute_command(Command::ToggleMsaa, ctx);
        }
        ui.separator();
        ui.label(theme::header("Standard views"));
        for view in StandardView::ALL {
            if ui.button(view.label()).clicked() {
                app.execute_command(Command::StandardView(view), ctx);
                ui.close_menu();
            }
        }
        ui.separator();
        // Camera navigation has no mode to pick any more: the mouse bindings
        // are fixed (see the viewport help line and the Help menu).
        if ui.button("Fit / Zoom extents").clicked() {
            app.execute_command(Command::Fit, ctx);
            ui.close_menu();
        }
        if ui.button("Reset view").clicked() {
            app.execute_command(Command::ResetView, ctx);
            ui.close_menu();
        }
        ui.separator();
        let mut show_project = !app.left_panel_collapsed;
        if ui.checkbox(&mut show_project, "Project panel").changed() {
            app.execute_command(Command::ToggleLeftPanel, ctx);
        }
        let mut show_dock = !app.bottom_dock_collapsed;
        if ui
            .checkbox(&mut show_dock, "Output dock (Ctrl+M)")
            .changed()
        {
            app.execute_command(Command::ToggleBottomDock, ctx);
        }
        let mut show_inspector = !app.inspector_collapsed;
        if ui
            .checkbox(&mut show_inspector, "Right panel (Ctrl+I)")
            .changed()
        {
            app.execute_command(Command::ToggleRightPanel, ctx);
        }
        ui.separator();
        if ui.button(theme::toggle_label(app.dark_mode)).clicked() {
            app.execute_command(Command::ToggleTheme, ctx);
            ui.close_menu();
        }
    });
}

fn model_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Model", |ui| {
        // Geometry group equivalents: the module switches that the ribbon's
        // Node / Member / Section buttons performed.
        for module in [WorkModule::Model, WorkModule::Section, WorkModule::Load] {
            if ui.button(module.label()).clicked() {
                app.execute_command(Command::SwitchModule(module), ctx);
                ui.close_menu();
            }
        }
        ui.separator();
        if ui.button("Add node / member").clicked() {
            app.execute_command(Command::SwitchModule(WorkModule::Model), ctx);
            ui.close_menu();
        }
    });
}

/// Boundary quick-adds (the former ribbon Boundary group).
///
/// Each entry dispatches the command that opens the matching quick-add popup
/// for the **primary** selected entity; the entries are disabled (with the
/// palette's explanation on hover) when the wrong entity is selected.
fn boundary_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Boundary", |ui| {
        let items: [(Command, &str, &str); 3] = [
            (
                Command::AddSupport,
                "Add Support\u{2026}",
                "Quick-add a support at the selected node",
            ),
            (
                Command::AddNodeLoad,
                "Add Nodal Load\u{2026}",
                "Quick-add a nodal load at the selected node",
            ),
            (
                Command::AddMemberLoad,
                "Add Member Load\u{2026}",
                "Quick-add a member load on the selected member",
            ),
        ];
        for (cmd, label, hint) in items {
            let enabled = app.command_is_enabled(cmd);
            if ui
                .add_enabled(enabled, egui::Button::new(label))
                .on_hover_text(hint)
                .clicked()
            {
                app.execute_command(cmd, ctx);
                ui.close_menu();
            }
        }
    });
}

fn analysis_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Analysis", |ui| {
        if ui
            .add_enabled(
                app.command_is_enabled(Command::RunAnalysis),
                egui::Button::new("Solve"),
            )
            .clicked()
        {
            app.execute_command(Command::RunAnalysis, ctx);
            ui.close_menu();
        }
        ui.separator();
        if ui.button("Solver options...").clicked() {
            app.execute_command(Command::SwitchModule(WorkModule::Analysis), ctx);
            ui.close_menu();
        }
    });
}

fn results_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Results", |ui| {
        for mode in ResultViewMode::ALL {
            let selected = app.display.mode == mode;
            if ui
                .add(egui::SelectableLabel::new(selected, mode.label()))
                .clicked()
            {
                app.execute_command(Command::ResultMode(mode), ctx);
                ui.close_menu();
            }
        }
        ui.separator();
        // Convenience toggle only — the detailed diagram controls (M/N/V,
        // all-members, scale) live in the inspector's Results section.
        let mut show_diagrams = app.display.show_diagrams;
        if ui
            .checkbox(&mut show_diagrams, "Internal-force diagrams")
            .changed()
        {
            app.execute_command(Command::ToggleDiagrams, ctx);
        }
    });
}

fn help_menu(app: &mut ViewportApp, ui: &mut egui::Ui, ctx: &egui::Context) {
    ui.menu_button("Help", |ui| {
        ui.label(theme::header("Mouse"));
        ui.label("Left click: select");
        ui.label("Left drag: box select");
        ui.label("Shift+click: toggle in selection");
        ui.label("Ctrl+click: add to selection");
        ui.label("Middle drag: pan");
        ui.label("Shift+middle drag: orbit");
        ui.label("Right drag: pan");
        ui.label("Wheel: zoom");
        ui.label("Alt+left drag: slide section cut");
        ui.label("Double-click: quick edit");
        ui.label("Right-click: context menu");
        ui.separator();
        ui.label(theme::header("Keyboard"));
        ui.label("R: reset view");
        ui.label("F: fit / zoom extents");
        ui.label("1-4: standard views");
        ui.label("Ctrl+A: select all visible");
        ui.label("Delete: delete selection");
        ui.label("Esc: clear selection");
        ui.label("Ctrl+Z / Ctrl+Y: undo / redo");
        ui.label("Ctrl+Shift+S: viewport screenshot");
        ui.label("Ctrl+Shift+P: command palette");
        ui.separator();
        if ui
            .button("Command Palette\u{2026} (Ctrl+Shift+P)")
            .on_hover_text("Search and run every command by name")
            .clicked()
        {
            app.execute_command(Command::TogglePalette, ctx);
            ui.close_menu();
        }
        ui.separator();
        if ui.button(theme::toggle_label(app.dark_mode)).clicked() {
            app.execute_command(Command::ToggleTheme, ctx);
            ui.close_menu();
        }
    });
}
