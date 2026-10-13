//! Modal dialogs (currently: the unsaved-changes confirmation).

use egui;

use viewport_poc::app_state::AppMessage;

use crate::ViewportApp;

/// Draw the unsaved-changes dialog when it is pending.
pub fn show_unsaved_dialog(app: &mut ViewportApp, ctx: &egui::Context) {
    if !app.show_unsaved_dialog {
        return;
    }
    egui::Window::new("Unsaved Changes")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("You have unsaved changes.");
            ui.label("Save before proceeding?");
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    let has_path = app.project_path.is_some();
                    if has_path {
                        if app.do_save() {
                            app.execute_pending();
                        }
                    } else {
                        let p = std::path::PathBuf::from(app.file_path_input.trim());
                        if p.as_os_str().is_empty() {
                            app.messages
                                .push(AppMessage::error("No file path. Enter a path or cancel."));
                        } else if app.save_to_path(&p) {
                            app.execute_pending();
                        }
                    }
                }
                if ui.button("Don't Save").clicked() {
                    app.execute_pending();
                }
                if ui.button("Cancel").clicked() {
                    app.show_unsaved_dialog = false;
                    app.pending_action = crate::PendingAction::None;
                }
            });
        });
}
