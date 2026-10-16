//! MIDAS-style work modules and the top module tab strip.
//!
//! A [`WorkModule`] is the coarse-grained task the user is currently
//! performing.  The active module selects which inspector sections are
//! shown.  Modules are *not* documents — there is still exactly one model.

use egui;

use super::theme;
use crate::ViewportApp;

/// The active work module.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WorkModule {
    /// Geometry, materials and support editing.
    #[default]
    Model,
    /// Section builder (placeholder — implemented in a later task).
    Section,
    /// Nodal and member load editing.
    Load,
    /// Solve trigger and solver options.
    Analysis,
    /// Result display controls.
    Results,
}

impl WorkModule {
    /// All modules in tab order.
    pub const ALL: [WorkModule; 5] = [
        WorkModule::Model,
        WorkModule::Section,
        WorkModule::Load,
        WorkModule::Analysis,
        WorkModule::Results,
    ];

    /// Short tab label.
    pub const fn label(self) -> &'static str {
        match self {
            WorkModule::Model => "Model",
            WorkModule::Section => "Section",
            WorkModule::Load => "Load",
            WorkModule::Analysis => "Analysis",
            WorkModule::Results => "Results",
        }
    }
}

/// Draw the module tab strip as a top panel.
pub fn show_module_tabs(app: &mut ViewportApp, ctx: &egui::Context) {
    egui::TopBottomPanel::top("module_tabs").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.add_space(2.0);
            for module in WorkModule::ALL {
                let selected = app.module == module;
                let text = theme::header(module.label());
                if ui.add(egui::SelectableLabel::new(selected, text)).clicked() {
                    app.module = module;
                }
            }
        });
    });
}
