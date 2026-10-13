//! MIDAS Gen NX-style ribbon toolbar.
//!
//! A row of grouped large-icon buttons with captions, drawn inside the same
//! [`egui::TopBottomPanel::top`](egui::TopBottomPanel) as the menu bar (one row
//! below it).  Groups are separated by vertical dividers and labelled by a
//! small caption underneath.
//!
//! Every button with a real action calls the **same** [`ViewportApp`] command
//! used elsewhere in the app (menu items, module switching, view presets) — no
//! logic is duplicated.  Controls that are not implemented yet are rendered
//! disabled with a "Not yet implemented" tooltip and a dimmed look; they are
//! listed via [`RibbonAction::Placeholder`] so wiring them later is a
//! one-line change per button.

use egui;

use viewport_poc::app_state::AnalysisStatus;
use viewport_poc::camera::StandardView;
use viewport_poc::display::ResultViewMode;

use super::icons::{self, IconFn};
use super::module::WorkModule;
use super::theme;
use crate::ViewportApp;

/// Fixed button size (points) so every group lines up.
const BUTTON_W: f32 = 46.0;
const BUTTON_H: f32 = 38.0;
/// Icon glyph box size inside a button.
const ICON_SIZE: f32 = 20.0;
/// Caption font size.
const CAPTION_SIZE: f32 = 9.0;
/// Tooltip shown on every control that is not wired up yet.
const NOT_IMPLEMENTED: &str = "Not yet implemented";

/// A single ribbon command.
///
/// [`RibbonAction::Placeholder`] marks a not-yet-implemented control.  Adding
/// a real variant here (and pointing a [`Button`] at it) is all it takes to
/// enable one later.
#[derive(Clone, Copy)]
enum RibbonAction {
    New,
    Open,
    Save,
    Undo,
    Redo,
    /// Switch the active work module (sidebar / tab target).
    Module(WorkModule),
    Solve,
    /// Select a primary result visualization mode.
    ResultMode(ResultViewMode),
    /// Toggle the internal-force (M/V/N) diagrams overlay.
    ToggleDiagrams,
    Fit,
    Standard(StandardView),
    ToggleTheme,
    /// Not implemented yet — rendered disabled with a "Not yet implemented"
    /// tooltip.
    Placeholder,
}

impl RibbonAction {
    fn is_placeholder(self) -> bool {
        matches!(self, RibbonAction::Placeholder)
    }

    /// Whether the button is clickable right now.
    fn enabled(self, app: &ViewportApp) -> bool {
        match self {
            RibbonAction::Undo => app.history.can_undo(),
            RibbonAction::Redo => app.history.can_redo(),
            RibbonAction::Solve => app.can_run_analysis(),
            RibbonAction::Placeholder => false,
            _ => true,
        }
    }

    /// Whether the button reflects an active state (highlighted).
    fn active(self, app: &ViewportApp) -> bool {
        match self {
            RibbonAction::ResultMode(m) => app.display.mode == m,
            RibbonAction::ToggleDiagrams => app.display.show_diagrams,
            // Module buttons deliberately do not highlight: the module tab
            // strip already shows the active module, and both "Node" and
            // "Member" target the Model module.
            _ => false,
        }
    }

    /// Run the command — always delegates to the shared `ViewportApp` method.
    fn run(self, app: &mut ViewportApp) {
        match self {
            RibbonAction::New => app.request_new(),
            RibbonAction::Open => app.request_open(),
            RibbonAction::Save => {
                let _ = app.do_save();
            }
            RibbonAction::Undo => app.do_undo(),
            RibbonAction::Redo => app.do_redo(),
            RibbonAction::Module(m) => app.module = m,
            RibbonAction::Solve => app.run_solve(),
            RibbonAction::ResultMode(m) => app.display.mode = m,
            RibbonAction::ToggleDiagrams => {
                app.display.show_diagrams = !app.display.show_diagrams;
            }
            RibbonAction::Fit => app.fit_view(),
            RibbonAction::Standard(v) => app.set_standard_view(v),
            RibbonAction::ToggleTheme => app.dark_mode = !app.dark_mode,
            RibbonAction::Placeholder => {}
        }
    }
}

/// One ribbon button: icon + caption + tooltip + command.
struct Button {
    icon: IconFn,
    caption: &'static str,
    tooltip: &'static str,
    action: RibbonAction,
}

// ── Group contents (single source of truth) ─────────────────────────────
// Placeholder buttons use `RibbonAction::Placeholder` and are rendered
// disabled.  The full set of placeholders is: Delete, Support, Node Load,
// Mem Load, Tables, Pan, Zoom.

const FILE: &[Button] = &[
    Button {
        icon: icons::new_file,
        caption: "New",
        tooltip: "New empty model",
        action: RibbonAction::New,
    },
    Button {
        icon: icons::open_folder,
        caption: "Open",
        tooltip: "Open a project file",
        action: RibbonAction::Open,
    },
    Button {
        icon: icons::save_floppy,
        caption: "Save",
        tooltip: "Save the current project",
        action: RibbonAction::Save,
    },
];

const EDIT: &[Button] = &[
    Button {
        icon: icons::undo,
        caption: "Undo",
        tooltip: "Undo (Ctrl+Z)",
        action: RibbonAction::Undo,
    },
    Button {
        icon: icons::redo,
        caption: "Redo",
        tooltip: "Redo (Ctrl+Y)",
        action: RibbonAction::Redo,
    },
    Button {
        icon: icons::trash,
        caption: "Delete",
        tooltip: "Delete the current selection",
        action: RibbonAction::Placeholder,
    },
];

const GEOMETRY: &[Button] = &[
    Button {
        icon: icons::node_plus,
        caption: "Node",
        tooltip: "Add a node (Model module)",
        action: RibbonAction::Module(WorkModule::Model),
    },
    Button {
        icon: icons::member,
        caption: "Member",
        tooltip: "Add a member (Model module)",
        action: RibbonAction::Module(WorkModule::Model),
    },
    Button {
        icon: icons::section_i,
        caption: "Section",
        tooltip: "Open the Section module",
        action: RibbonAction::Module(WorkModule::Section),
    },
];

const BOUNDARY: &[Button] = &[
    Button {
        icon: icons::support,
        caption: "Support",
        tooltip: "Add a nodal support",
        action: RibbonAction::Placeholder,
    },
    Button {
        icon: icons::load_nodal,
        caption: "Node Load",
        tooltip: "Add a nodal load",
        action: RibbonAction::Placeholder,
    },
    Button {
        icon: icons::load_dist,
        caption: "Mem Load",
        tooltip: "Add a member load",
        action: RibbonAction::Placeholder,
    },
];

const ANALYSIS: &[Button] = &[
    Button {
        icon: icons::play,
        caption: "Run",
        tooltip: "Run the analysis",
        action: RibbonAction::Solve,
    },
    Button {
        icon: icons::sliders,
        caption: "Solver",
        tooltip: "Solver options (Analysis module)",
        action: RibbonAction::Module(WorkModule::Analysis),
    },
];

const RESULTS: &[Button] = &[
    Button {
        icon: icons::displacement,
        caption: "Displ",
        tooltip: "Displacement result view",
        action: RibbonAction::ResultMode(ResultViewMode::Displacement),
    },
    Button {
        icon: icons::diagram,
        caption: "M/V/N",
        tooltip: "Internal-force diagrams (M/V/N)",
        action: RibbonAction::ToggleDiagrams,
    },
    Button {
        icon: icons::table,
        caption: "Tables",
        tooltip: "Tabular results",
        action: RibbonAction::Placeholder,
    },
];

const VIEW: &[Button] = &[
    Button {
        icon: icons::fit,
        caption: "Fit",
        tooltip: "Fit / zoom extents (F)",
        action: RibbonAction::Fit,
    },
    Button {
        icon: icons::cube,
        caption: "Iso",
        tooltip: "Isometric view (1)",
        action: RibbonAction::Standard(StandardView::Iso),
    },
    Button {
        icon: icons::view_front,
        caption: "Front",
        tooltip: "Front view (2)",
        action: RibbonAction::Standard(StandardView::Front),
    },
    Button {
        icon: icons::view_top,
        caption: "Top",
        tooltip: "Top view (3)",
        action: RibbonAction::Standard(StandardView::Top),
    },
    Button {
        icon: icons::view_right,
        caption: "Right",
        tooltip: "Right view (4)",
        action: RibbonAction::Standard(StandardView::Right),
    },
    Button {
        icon: icons::pan,
        caption: "Pan",
        tooltip: "Pan the view",
        action: RibbonAction::Placeholder,
    },
    Button {
        icon: icons::zoom,
        caption: "Zoom",
        tooltip: "Zoom the view",
        action: RibbonAction::Placeholder,
    },
];

const TOOLS: &[Button] = &[Button {
    icon: icons::sun,
    caption: "Theme",
    tooltip: "Toggle light / dark theme",
    action: RibbonAction::ToggleTheme,
}];

/// Draw the ribbon row.  Called from `menu.rs` on the line below the menu bar,
/// inside the same top panel.
pub fn show_ribbon(app: &mut ViewportApp, ui: &mut egui::Ui) {
    ui.horizontal_top(|ui| {
        ui.spacing_mut().item_spacing = egui::vec2(3.0, 2.0);
        group(ui, app, "File", FILE);
        divider(ui);
        group(ui, app, "Edit", EDIT);
        divider(ui);
        group(ui, app, "Geometry", GEOMETRY);
        divider(ui);
        group(ui, app, "Boundary", BOUNDARY);
        divider(ui);
        group(ui, app, "Analysis", ANALYSIS);
        divider(ui);
        group(ui, app, "Results", RESULTS);
        divider(ui);
        group(ui, app, "View", VIEW);
        divider(ui);
        group(ui, app, "Tools", TOOLS);

        ui.with_layout(
            egui::Layout::right_to_left(egui::Align::Center),
            |ui| analysis_status(app, ui),
        );
    });
}

/// A bordered group: a horizontal row of buttons with a small caption below.
fn group(ui: &mut egui::Ui, app: &mut ViewportApp, caption: &str, buttons: &[Button]) {
    egui::Frame::group(ui.style())
        .inner_margin(egui::Margin::symmetric(4.0, 2.0))
        .show(ui, |ui| {
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(2.0, 0.0);
                ui.horizontal(|ui| {
                    for b in buttons {
                        ribbon_button(ui, app, b);
                    }
                });
                let color = ui.visuals().weak_text_color();
                ui.vertical_centered(|ui| {
                    ui.label(egui::RichText::new(caption).size(CAPTION_SIZE).color(color));
                });
            });
        });
}

/// A thin vertical separator between groups.
fn divider(ui: &mut egui::Ui) {
    ui.add_space(2.0);
    ui.separator();
    ui.add_space(2.0);
}

/// Draw and handle one ribbon button.
fn ribbon_button(ui: &mut egui::Ui, app: &mut ViewportApp, b: &Button) {
    let enabled = b.action.enabled(app);
    let active = b.action.active(app);
    let response = paint_button(ui, b.icon, b.caption, enabled, active);
    let tooltip = if b.action.is_placeholder() {
        NOT_IMPLEMENTED
    } else {
        b.tooltip
    };
    let response = response.on_hover_text(tooltip);
    if enabled && response.clicked() {
        b.action.run(app);
    }
}

/// Paint a large-icon + caption button and return its interaction response.
fn paint_button(
    ui: &mut egui::Ui,
    icon: IconFn,
    caption: &str,
    enabled: bool,
    active: bool,
) -> egui::Response {
    let sense = if enabled {
        egui::Sense::click()
    } else {
        egui::Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(egui::vec2(BUTTON_W, BUTTON_H), sense);
    if !ui.is_rect_visible(rect) {
        return response;
    }

    let visuals = ui.visuals().clone();
    let rounding = 4.0;

    // Background: active > hovered > resting; disabled is drawn flat.
    let bg = if !enabled {
        visuals.faint_bg_color
    } else if active {
        visuals.selection.bg_fill
    } else if response.hovered() {
        visuals.widgets.hovered.weak_bg_fill
    } else {
        visuals.widgets.inactive.weak_bg_fill
    };
    let painter = ui.painter();
    painter.rect_filled(rect, rounding, bg);
    if active {
        painter.rect_stroke(rect, rounding, visuals.selection.stroke);
    }

    // Foreground colour: dimmed when disabled, highlighted when active.
    let fg = if !enabled {
        visuals.text_color().gamma_multiply(0.45)
    } else if active {
        visuals.selection.stroke.color
    } else {
        visuals.text_color()
    };

    let icon_rect = egui::Rect::from_center_size(
        egui::pos2(rect.center().x, rect.top() + 3.0 + ICON_SIZE * 0.5),
        egui::vec2(ICON_SIZE, ICON_SIZE),
    );
    icon(painter, icon_rect, fg);

    let caption_rect = egui::Rect::from_min_max(
        egui::pos2(rect.left(), icon_rect.bottom() + 1.0),
        egui::pos2(rect.right(), rect.bottom()),
    );
    painter.text(
        caption_rect.center(),
        egui::Align2::CENTER_CENTER,
        caption,
        egui::FontId::proportional(CAPTION_SIZE),
        fg,
    );

    response
}

/// The stale / failed analysis warning, kept reachable from the ribbon.
fn analysis_status(app: &ViewportApp, ui: &mut egui::Ui) {
    match app.project.analysis_status() {
        AnalysisStatus::Stale => {
            ui.label(
                egui::RichText::new("\u{26a0} Results are stale \u{2014} re-solve")
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
