//! Bottom tabbed dock (Messages / Results) plus the thin status line.

use egui;

use structural_analysis::Dof3D;
use viewport_poc::app_state::{AnalysisStatus, MessageLevel};
use viewport_poc::selection::{SelectionSet, SelectionState};

use super::layout;
use super::module::WorkModule;
use super::theme::TextPalette;
use super::{fmt, forces_grid};
use crate::ViewportApp;

/// Tabs available in the bottom dock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BottomTab {
    /// Full message history.
    #[default]
    Messages,
    /// Tabular analysis results.
    Results,
}

impl BottomTab {
    const ALL: [BottomTab; 2] = [BottomTab::Messages, BottomTab::Results];

    const fn label(self) -> &'static str {
        match self {
            BottomTab::Messages => "Messages",
            BottomTab::Results => "Results",
        }
    }
}

/// Smallest the dock may be dragged.  Keeps the resize handle grabbable and
/// the tab row visible even when the dock is at its minimum.
const DOCK_MIN_HEIGHT: f32 = 28.0;
/// Ceiling for the default dock height (points).
const DOCK_DEFAULT_MAX: f32 = 160.0;
/// Floor for the default dock height (points).
const DOCK_DEFAULT_MIN: f32 = 110.0;
/// Default dock height as a fraction of the window height.
const DOCK_DEFAULT_FRACTION: f32 = 0.18;
/// The dock may never exceed this fraction of the window height.
const DOCK_MAX_FRACTION: f32 = 0.45;

/// Draw the bottom Messages / Results dock and the always-visible status line.
///
/// The status line is a separate, non-collapsible one-line bottom panel pinned
/// to the very bottom of the window; the dock sits directly above it.
///
/// The dock is resizable within a small height range (small fraction of the
/// window by default, hard-capped so it can never swallow the viewport) and
/// can be collapsed entirely — when collapsed its panel is not created at all,
/// so the central viewport reclaims the space with no residual gap.
pub fn show_bottom_dock(app: &mut ViewportApp, ctx: &egui::Context) {
    // ── Always-visible status strip (bottom-most panel) ─────────────────
    egui::TopBottomPanel::bottom("status_line").show(ctx, |ui| {
        ui.horizontal(|ui| {
            collapse_toggle(app, ui);
            ui.separator();
            status_line(app, ui);
        });
    });

    // ── Collapsible Messages / Results dock, directly above it ──────────
    if app.bottom_dock_collapsed {
        return;
    }
    let window_h = ctx.screen_rect().height();
    let max_h = (window_h * DOCK_MAX_FRACTION).max(DOCK_DEFAULT_MAX);
    // Phase P7-3 C2: a persisted layout preset overrides the window-fraction
    // default height (egui still clamps it into the dock's allowed range).
    let default_h = layout::restored_dock_height(ctx)
        .unwrap_or_else(|| (window_h * DOCK_DEFAULT_FRACTION).clamp(DOCK_DEFAULT_MIN, DOCK_DEFAULT_MAX));

    egui::TopBottomPanel::bottom("bottom_dock")
        .resizable(true)
        .min_height(DOCK_MIN_HEIGHT)
        .max_height(max_h)
        .default_height(default_h)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                for tab in BottomTab::ALL {
                    let selected = app.bottom_tab == tab;
                    if ui
                        .add(egui::SelectableLabel::new(selected, tab.label()))
                        .clicked()
                    {
                        app.bottom_tab = tab;
                    }
                }
                if app.bottom_tab == BottomTab::Messages {
                    ui.separator();
                    if ui.button("Clear").clicked() {
                        app.messages.clear();
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .button("\u{25bc} Hide")
                        .on_hover_text("Hide the messages / results dock (Ctrl+M)")
                        .clicked()
                    {
                        app.bottom_dock_collapsed = true;
                    }
                });
            });
            ui.separator();

            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| match app.bottom_tab {
                    BottomTab::Messages => messages_view(app, ui),
                    BottomTab::Results => results_view(app, ui),
                });

            // Publish the live dock height for layout-preset persistence
            // (Phase P7-3 C2).  `ui.max_rect()` is the dock's *inner* rect;
            // add the frame's inner margin back to recover the outer height.
            let margin = egui::Frame::side_top_panel(ui.style()).inner_margin;
            layout::publish_dock_height(
                ctx,
                ui.max_rect().height() + (margin.top + margin.bottom) as f32,
            );
        });
}

/// Small chevron in the status line that shows/hides the dock above it.
fn collapse_toggle(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let (label, hint) = if app.bottom_dock_collapsed {
        ("\u{25b2} Messages", "Show the messages / results dock (Ctrl+M)")
    } else {
        ("\u{25bc} Messages", "Hide the messages / results dock (Ctrl+M)")
    };
    if ui
        .selectable_label(false, label)
        .on_hover_text(hint)
        .clicked()
    {
        app.bottom_dock_collapsed = !app.bottom_dock_collapsed;
    }
}

fn messages_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    if app.messages.is_empty() {
        ui.label(egui::RichText::new("(no messages)").color(text.help));
        return;
    }
    for msg in &app.messages {
        let color = match msg.level {
            MessageLevel::Info => text.label,
            MessageLevel::Warning => text.stale,
            MessageLevel::Error => text.error,
        };
        let prefix = match msg.level {
            MessageLevel::Info => "",
            MessageLevel::Warning => "\u{26a0} ",
            MessageLevel::Error => "\u{2717} ",
        };
        ui.label(egui::RichText::new(format!("{prefix}{}", msg.text)).color(color));
    }
}

fn results_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let Some(model) = app.project.model.as_ref() else {
        ui.label("No model loaded");
        return;
    };
    let Some(result) = app.analysis_result() else {
        ui.label(
            egui::RichText::new("(no current result \u{2014} solve first)").color(text.stale),
        );
        return;
    };

    egui::CollapsingHeader::new(format!("Nodal displacements / reactions ({})", model.n_nodes()))
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
                        ui.label(format!("N{}", node.id.0));
                        ui.label(fmt(ux));
                        ui.label(fmt(uy));
                        ui.label(fmt(uz));
                        ui.label(fmt(rmax));
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
                    forces_grid(ui, &format!("dock_{}", member.id.0), &f);
                }
            }
        });
}

fn status_line(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.horizontal(|ui| {
        if app.project.is_empty() {
            ui.label("No model loaded");
        } else {
            ui.label(format!("Model: {}", app.project.name));
            if let Some(m) = &app.project.model {
                ui.label(format!("Nodes: {}", m.n_nodes()));
                ui.label(format!("Members: {}", m.n_members()));
            }
            match app.project.analysis_status() {
                AnalysisStatus::Current => {
                    ui.label("Analysis: \u{2713}");
                }
                AnalysisStatus::Stale => {
                    ui.label(egui::RichText::new("Analysis: STALE").color(text.stale));
                }
                AnalysisStatus::Failed => {
                    ui.label(egui::RichText::new("Analysis: FAILED").color(text.error));
                }
                AnalysisStatus::NeverSolved => {
                    ui.label("Analysis: not run");
                }
            }
        }
        ui.separator();
        ui.label(format!("Module: {}", WorkModule::label(app.module)));
        ui.separator();
        ui.label(selection_summary(&app.selected));
    });
}

/// Status-bar summary of the current multi-selection.
///
/// A single entity keeps the compact `N{id}` / `M{id}` form (unchanged
/// behaviour); a multi-selection reports counts, e.g. `3 nodes, 2 members`.
fn selection_summary(selection: &SelectionSet) -> String {
    match selection.single() {
        Some(SelectionState::Node(id)) => format!("Selection: N{}", id.0),
        Some(SelectionState::Member(id)) => format!("Selection: M{}", id.0),
        _ if selection.is_empty() => "Selection: none".to_string(),
        _ => {
            let mut parts = Vec::new();
            let n = selection.node_count();
            if n > 0 {
                parts.push(format!("{n} node{}", if n == 1 { "" } else { "s" }));
            }
            let m = selection.member_count();
            if m > 0 {
                parts.push(format!("{m} member{}", if m == 1 { "" } else { "s" }));
            }
            format!("Selection: {}", parts.join(", "))
        }
    }
}
