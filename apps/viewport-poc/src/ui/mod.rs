//! egui application shell.
//!
//! All rendering lives here, split into focused modules (menu bar, module
//! tabs, model tree, viewport, inspector, bottom dock, dialogs, theme).
//! `main.rs` keeps the [`ViewportApp`](crate::ViewportApp) state and its
//! command logic, and orchestrates these panels.

pub mod actions;
pub mod diagrams;
pub mod dialogs;
pub mod gl_renderer;
pub mod icons;
pub mod inspector;
pub mod menu;
pub mod module;
pub mod ribbon;
pub mod section;
pub mod status;
pub mod theme;
pub mod tree;
pub mod viewport;

pub use module::WorkModule;

/// Format an engineering value with adaptive precision.
///
/// * Non-finite → `"NaN"`
/// * `|v| < 1e-12` → `"0"`
/// * Otherwise → 4 significant figures in scientific notation
pub(crate) fn fmt(v: f64) -> String {
    if !v.is_finite() {
        return "NaN".into();
    }
    if v.abs() < 1e-12 {
        return "0".into();
    }
    format!("{:.4e}", v)
}

/// Render a 12-DOF member end-force table (forces and moments at ends i/j).
pub(crate) fn forces_grid(ui: &mut egui::Ui, id: &str, f: &[f64; 12]) {
    egui::Grid::new(format!("forces_{id}"))
        .num_columns(4)
        .spacing([8.0, 3.0])
        .show(ui, |ui| {
            ui.strong("End");
            ui.strong("Fx");
            ui.strong("Fy");
            ui.strong("Fz");
            ui.end_row();
            ui.label("i");
            ui.label(fmt(f[0]));
            ui.label(fmt(f[1]));
            ui.label(fmt(f[2]));
            ui.end_row();
            ui.label("j");
            ui.label(fmt(f[6]));
            ui.label(fmt(f[7]));
            ui.label(fmt(f[8]));
            ui.end_row();
            ui.strong("End");
            ui.strong("Mx");
            ui.strong("My");
            ui.strong("Mz");
            ui.end_row();
            ui.label("i");
            ui.label(fmt(f[3]));
            ui.label(fmt(f[4]));
            ui.label(fmt(f[5]));
            ui.end_row();
            ui.label("j");
            ui.label(fmt(f[9]));
            ui.label(fmt(f[10]));
            ui.label(fmt(f[11]));
            ui.end_row();
        });
}
