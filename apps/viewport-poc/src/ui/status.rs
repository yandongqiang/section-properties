//! Centre-scoped Output dock (Problems / Solver Log / History) plus the thin
//! status line.
//!
//! Phase 159-B introduced the three purpose-built dock tabs; Phase 159-B2/C
//! **scoped the dock to the centre workspace**: [`show_output_dock`] lays the
//! panel out *inside* the caller's `Ui` with
//! [`egui::TopBottomPanel::bottom`]`::show_inside`, so its left/right edges
//! align exactly with the centre workspace (between the left Project panel and
//! the right auxiliary panel) instead of spanning the whole window.  The
//! compositional entry point is `ui::workspace::show_center_workspace`.
//!
//! * **Problems** — the warning / error diagnostics from the message stream;
//! * **Solver Log** — the informational messages (analysis / solve output);
//! * **History** — the edit history, newest first, with a short label per
//!   committed edit.
//!
//! (The former temporary **Results** tab was retired in 159-C: the tabular
//! results now live in the centre workspace, shown when the **Results** work
//! module is active — see `ui::workspace`.)
//!
//! [`show_status_line`] is the separate, always-visible one-line status strip.
//! It is a **root-level** bottom panel, added before the side panels, so it
//! spans the full window width and sits *outside* the main workspace; it also
//! carries a problem count and the output-dock collapse toggle so diagnostics
//! remain visible while the dock is collapsed.

use egui;

use viewport_poc::app_state::{AnalysisStatus, AppMessage, MessageLevel};
use viewport_poc::selection::{SelectionSet, SelectionState};

use super::icons::{self, IconFn};
use super::layout;
use super::module::WorkModule;
use super::theme::{self, TextPalette};
use crate::ViewportApp;

/// Tabs available in the bottom output dock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BottomTab {
    /// Warning / error diagnostics.
    #[default]
    Problems,
    /// Informational messages (analysis / solve output).
    SolverLog,
    /// Edit history, newest first.
    History,
}

impl BottomTab {
    const ALL: [BottomTab; 3] = [
        BottomTab::Problems,
        BottomTab::SolverLog,
        BottomTab::History,
    ];

    const fn label(self) -> &'static str {
        match self {
            BottomTab::Problems => "Problems",
            BottomTab::SolverLog => "Solver Log",
            BottomTab::History => "History",
        }
    }

    /// Leading glyph for the tab button.
    fn icon(self) -> IconFn {
        match self {
            BottomTab::Problems => icons::problems,
            BottomTab::SolverLog => icons::sliders,
            BottomTab::History => icons::history,
        }
    }
}

/// Smallest the dock may be dragged.  Keeps the resize handle grabbable and
/// the tab row visible even when the dock is at its minimum.
const DOCK_MIN_HEIGHT: f32 = 28.0;
/// Ceiling for the default dock height (points).
const DOCK_DEFAULT_MAX: f32 = 200.0;
/// Floor for the default dock height (points).
const DOCK_DEFAULT_MIN: f32 = 110.0;
/// Default dock height as a fraction of the central workspace height.
const DOCK_DEFAULT_FRACTION: f32 = 0.25;
/// The dock may never exceed this fraction of the **central workspace** height,
/// so it can never swallow the viewport (Phase 159-B2 requirement: ≤ ~60–70%).
const DOCK_MAX_FRACTION: f32 = 0.70;

/// Is this message a problem (warning or error)?
fn is_problem(msg: &AppMessage) -> bool {
    matches!(msg.level, MessageLevel::Warning | MessageLevel::Error)
}

/// Count of warning / error messages.
fn problem_count(app: &ViewportApp) -> usize {
    app.messages.iter().filter(|m| is_problem(m)).count()
}

/// Count of error messages.
fn error_count(app: &ViewportApp) -> usize {
    app.messages
        .iter()
        .filter(|m| m.level == MessageLevel::Error)
        .count()
}

/// Count of warning messages.
fn warning_count(app: &ViewportApp) -> usize {
    app.messages
        .iter()
        .filter(|m| m.level == MessageLevel::Warning)
        .count()
}

/// Maximum allowed dock height for a central workspace of `central_h` points.
///
/// The ceiling is a fraction of the **central** height (not the window), and it
/// never drops below the grab handle.  Callers additionally rely on egui
/// clamping the panel to the available height, so the dock can never consume
/// the whole viewport.
pub(crate) fn dock_max_height(central_h: f32) -> f32 {
    (central_h * DOCK_MAX_FRACTION).max(DOCK_MIN_HEIGHT)
}

/// Default dock height for a central workspace of `central_h` points, honouring
/// a restored (persisted) height when one exists, then clamped into the dock's
/// allowed range.
pub(crate) fn dock_default_height(central_h: f32, restored: Option<f32>) -> f32 {
    let desired = restored.unwrap_or_else(|| {
        (central_h * DOCK_DEFAULT_FRACTION).clamp(DOCK_DEFAULT_MIN, DOCK_DEFAULT_MAX)
    });
    desired.clamp(DOCK_MIN_HEIGHT, dock_max_height(central_h))
}

/// Draw the always-visible status strip.
///
/// A **root-level** bottom panel added before the side panels, so it spans the
/// full window width and lies outside the main workspace.  It carries the
/// output-dock collapse toggle plus the model / analysis / module / selection
/// summary.
pub fn show_status_line(app: &mut ViewportApp, ctx: &egui::Context) {
    egui::TopBottomPanel::bottom("status_line").show(ctx, |ui| {
        ui.horizontal(|ui| {
            collapse_toggle(app, ui);
            ui.separator();
            status_line(app, ui);
        });
    });
}

/// Draw the resizable Output dock **inside the centre workspace**.
///
/// The panel is laid out with [`egui::TopBottomPanel::bottom`]`::show_inside`
/// on the caller's `Ui` (the centre workspace), so its width equals the centre
/// workspace width — it never spans the full application width, and expanding /
/// collapsing it only ever changes the centre region's height.  When collapsed
/// the panel is not created at all; egui keeps its last stored height, so
/// reopening restores it (Phase 159-B2 requirement 7).
pub(crate) fn show_output_dock(app: &mut ViewportApp, ui: &mut egui::Ui) {
    if app.bottom_dock_collapsed {
        return;
    }
    let ctx = ui.ctx().clone();
    // The dock is capped against the *central* height — the height still
    // available after the workspace tab bar has been laid out.
    let central_h = ui.available_rect_before_wrap().height();
    let max_h = dock_max_height(central_h);
    let default_h = dock_default_height(central_h, layout::restored_dock_height(&ctx));

    egui::TopBottomPanel::bottom("workspace_output")
        .resizable(true)
        .min_height(DOCK_MIN_HEIGHT)
        .max_height(max_h)
        .default_height(default_h)
        .show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.set_min_height(theme::ROW_HEIGHT);
                for tab in BottomTab::ALL {
                    let label = match tab {
                        BottomTab::Problems => format!("Problems ({})", problem_count(app)),
                        BottomTab::History => format!("History ({})", app.history.undo_depth()),
                        _ => tab.label().to_string(),
                    };
                    let selected = app.bottom_tab == tab;
                    let response = icons::icon_text_button(ui, tab.icon(), &label, selected, true);
                    if response.clicked() {
                        app.bottom_tab = tab;
                    }
                    if response.hovered() {
                        response.on_hover_text(tab_hint(tab));
                    }
                }
                // Per-tab clear action (Problems / Solver Log only).
                match app.bottom_tab {
                    BottomTab::Problems if problem_count(app) > 0 => {
                        ui.separator();
                        if ui
                            .button("Clear")
                            .on_hover_text("Clear warning / error messages")
                            .clicked()
                        {
                            app.messages.retain(|m| !is_problem(m));
                        }
                    }
                    BottomTab::SolverLog => {
                        ui.separator();
                        if ui
                            .button("Clear")
                            .on_hover_text("Clear informational messages")
                            .clicked()
                        {
                            app.messages.retain(|m| is_problem(m));
                        }
                    }
                    _ => {}
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .button("\u{25bc} Hide")
                        .on_hover_text("Hide the Output dock (Ctrl+M)")
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
                    BottomTab::Problems => problems_view(app, ui),
                    BottomTab::SolverLog => solver_log_view(app, ui),
                    BottomTab::History => history_view(app, ui),
                });

            // Publish the live dock height for layout-preset persistence
            // (Phase P7-3 C2).  `ui.max_rect()` is the dock's *inner* rect;
            // add the frame's inner margin back to recover the outer height.
            let margin = egui::Frame::side_top_panel(ui.style()).inner_margin;
            layout::publish_dock_height(
                &ctx,
                ui.max_rect().height() + (margin.top + margin.bottom) as f32,
            );
        });
}

/// Tooltip for a dock tab button.
fn tab_hint(tab: BottomTab) -> &'static str {
    match tab {
        BottomTab::Problems => "Warnings and errors",
        BottomTab::SolverLog => "Informational messages (analysis / solve output)",
        BottomTab::History => "Edit history (newest first)",
    }
}

/// Small chevron in the status line that shows/hides the dock above it.
///
/// When the dock is collapsed it carries the problem count, so diagnostics
/// stay visible without expanding the panel.
fn collapse_toggle(app: &mut ViewportApp, ui: &mut egui::Ui) {
    let problems = problem_count(app);
    let (label, hint) = if app.bottom_dock_collapsed {
        let label = if problems > 0 {
            format!("\u{25b2} Output \u{26a0}{problems}")
        } else {
            "\u{25b2} Output".to_string()
        };
        (label, "Show the Output dock (Ctrl+M)")
    } else {
        (
            "\u{25bc} Output".to_string(),
            "Hide the Output dock (Ctrl+M)",
        )
    };
    if ui
        .selectable_label(false, label)
        .on_hover_text(hint)
        .clicked()
    {
        app.bottom_dock_collapsed = !app.bottom_dock_collapsed;
    }
}

/// Problems tab: warnings and errors, colour-coded.
fn problems_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    if problem_count(app) == 0 {
        ui.label(egui::RichText::new("(no problems)").color(text.help));
        return;
    }
    for msg in app.messages.iter().filter(|m| is_problem(m)) {
        let (color, prefix) = match msg.level {
            MessageLevel::Error => (text.error, "\u{2717} "),
            _ => (text.stale, "\u{26a0} "),
        };
        ui.label(egui::RichText::new(format!("{prefix}{}", msg.text)).color(color));
    }
}

/// Solver Log tab: informational messages.
fn solver_log_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let mut any = false;
    for msg in app.messages.iter().filter(|m| !is_problem(m)) {
        any = true;
        ui.label(egui::RichText::new(&msg.text).color(text.label));
    }
    if !any {
        ui.label(egui::RichText::new("(no messages)").color(text.help));
    }
}

/// History tab: committed edits, newest first, with redoable edits below.
fn history_view(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let undo = app.history.undo_labels();
    let redo = app.history.redo_labels();

    if undo.is_empty() && redo.is_empty() {
        ui.label(
            egui::RichText::new("(no edits yet \u{2014} model changes appear here)")
                .color(text.help),
        );
        return;
    }

    ui.label(
        theme::small(format!(
            "{} committed edit(s) \u{b7} {} undo available \u{b7} Ctrl+Z / Ctrl+Y",
            undo.len(),
            app.history.undo_depth()
        ))
        .color(text.help),
    );
    ui.add_space(2.0);

    // Newest first at the top of the list.
    let mut n = undo.len();
    for label in &undo {
        let display = if label.is_empty() { "(edit)" } else { label };
        ui.label(format!("{n:>3}  {display}"));
        n -= 1;
    }

    if !redo.is_empty() {
        ui.separator();
        ui.label(theme::small("Redo available (newest undone first)").color(text.help));
        for label in &redo {
            let display = if label.is_empty() { "(edit)" } else { label };
            ui.label(
                egui::RichText::new(format!("\u{21b7}  {display}"))
                    .color(text.help)
                    .italics(),
            );
        }
    }
}

fn status_line(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    ui.horizontal(|ui| {
        if app.project.is_empty() {
            ui.label(theme::body("No model loaded"));
        } else {
            ui.label(theme::body(format!("Model: {}", app.project.name)));
            if let Some(m) = &app.project.model {
                ui.label(theme::body(format!("Nodes: {}", m.n_nodes())));
                ui.label(theme::body(format!("Members: {}", m.n_members())));
            }
            match app.project.analysis_status() {
                AnalysisStatus::Current => {
                    ui.label(theme::body("Analysis: \u{2713}"));
                }
                AnalysisStatus::Stale => {
                    ui.label(theme::body("Analysis: STALE").color(text.stale));
                }
                AnalysisStatus::Failed => {
                    ui.label(theme::body("Analysis: FAILED").color(text.error));
                }
                AnalysisStatus::NeverSolved => {
                    ui.label(theme::body("Analysis: not run"));
                }
            }
            // MIDAS-style status read-out: which load case the editors are
            // writing to, and which solved result is on screen (they differ
            // once combinations exist).
            if let Some(m) = &app.project.model
                && let Some(case) = m.load_case(m.active_case())
            {
                ui.separator();
                ui.label(theme::body(format!("Editing: {}", case.name)));
                if case.self_weight {
                    ui.label(theme::small("(SW)").color(text.help));
                }
            }
            if app.project.analysis_status() == AnalysisStatus::Current
                && let Some(label) = app.project.active_result_label()
            {
                ui.separator();
                ui.label(theme::body(format!("Result: {label}")));
            }
        }
        // Diagnostics badge: keeps the error / warning count visible even when
        // the Output dock is collapsed.
        let errors = error_count(app);
        let warnings = warning_count(app);
        if errors > 0 {
            ui.separator();
            ui.label(theme::body(format!("\u{2717} {errors}")).color(text.error));
        }
        if warnings > 0 {
            if errors == 0 {
                ui.separator();
            }
            ui.label(theme::body(format!("\u{26a0} {warnings}")).color(text.stale));
        }
        ui.separator();
        ui.label(theme::body(format!(
            "Module: {}",
            WorkModule::label(app.module)
        )));
        ui.separator();
        ui.label(theme::body(selection_summary(&app.selected)));
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

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::EditModel;
    use viewport_poc::persistence;

    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// The egui panel id under which the centre output dock stores its height.
    const DOCK_ID: &str = "workspace_output";

    fn stored_dock_height(ctx: &egui::Context) -> Option<f32> {
        egui::containers::panel::PanelState::load(ctx, egui::Id::new(DOCK_ID)).map(|s| s.size().y)
    }

    /// Force the dock's stored panel height (as a drag would).
    fn force_dock_height(ctx: &egui::Context, height: f32) {
        let state = egui::containers::panel::PanelState {
            rect: egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, height)),
        };
        ctx.data_mut(|d| d.insert_persisted(egui::Id::new(DOCK_ID), state));
    }

    /// Run one headless frame of just the centre output dock.
    fn run_dock_frame(app: &mut ViewportApp, ctx: &egui::Context) {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(800.0, 600.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                show_output_dock(app, ui);
            });
        });
    }

    /// Every tab has a distinct, non-empty label and a glyph.
    #[test]
    fn tab_labels_are_distinct_and_complete() {
        assert_eq!(BottomTab::ALL.len(), 3);
        let labels: Vec<&str> = BottomTab::ALL.iter().map(|t| t.label()).collect();
        assert_eq!(labels, vec!["Problems", "Solver Log", "History"]);
        for tab in BottomTab::ALL {
            let _ = tab.icon();
            assert!(!tab_hint(tab).is_empty());
        }
        assert_eq!(BottomTab::default(), BottomTab::Problems);
    }

    /// The message stream splits cleanly into problems (warn/error) and the
    /// solver log (info): every message lands in exactly one bucket.
    #[test]
    fn messages_partition_into_problems_and_log() {
        let mut app = empty_app();
        app.messages.clear();
        app.messages.push(AppMessage::info("started"));
        app.messages.push(AppMessage::warning("stale"));
        app.messages.push(AppMessage::error("boom"));
        assert_eq!(problem_count(&app), 2);
        assert_eq!(error_count(&app), 1);
        assert_eq!(warning_count(&app), 1);
        let log = app.messages.iter().filter(|m| !is_problem(m)).count();
        assert_eq!(log, 1);
        assert_eq!(log + problem_count(&app), app.messages.len());
    }

    /// Switching the dock tab is pure view state: it must never touch the
    /// model, the selection or the analysis result.
    #[test]
    fn switching_dock_tab_does_not_mutate_state() {
        let mut app = empty_app();
        let mut model = EditModel::new();
        let a = model.add_node(0.0, 0.0, 0.0).unwrap();
        model.add_node(1.0, 0.0, 0.0).unwrap();
        app.project.load_model("t".into(), model);
        app.rebuild_scene();
        app.select_only(SelectionState::Node(a));
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        let selection = app.selected.clone();

        for tab in BottomTab::ALL {
            app.bottom_tab = tab;
            assert_eq!(
                app.project.model.as_ref().unwrap().fingerprint(),
                fingerprint
            );
            assert_eq!(app.selected, selection);
        }
        assert_eq!(app.selected.primary(), SelectionState::Node(a));
    }

    /// The dock's ceiling is always a fraction of the central height, so it can
    /// never consume the whole viewport.
    #[test]
    fn dock_max_height_never_consumes_the_workspace() {
        for central_h in [200.0_f32, 400.0, 800.0, 1600.0] {
            let max_h = dock_max_height(central_h);
            assert!(
                max_h < central_h,
                "max {max_h} must be < central {central_h}"
            );
            assert!(
                max_h <= central_h * DOCK_MAX_FRACTION + 1e-3,
                "max {max_h} exceeds {}% of {central_h}",
                DOCK_MAX_FRACTION * 100.0
            );
            assert!(max_h >= DOCK_MIN_HEIGHT);
        }
    }

    /// The restored height wins, then is clamped into the dock's range.
    #[test]
    fn dock_default_height_honours_restored_value_and_clamps() {
        // A restored value inside the range is used verbatim.
        assert_eq!(dock_default_height(1000.0, Some(180.0)), 180.0);
        // A restored value below the minimum clamps up.
        assert_eq!(dock_default_height(1000.0, Some(4.0)), DOCK_MIN_HEIGHT);
        // A restored value above the maximum clamps down to the ceiling.
        assert_eq!(
            dock_default_height(1000.0, Some(9000.0)),
            dock_max_height(1000.0)
        );
        // No restored value → a window-fraction default inside the range.
        let d = dock_default_height(800.0, None);
        assert!((DOCK_DEFAULT_MIN..=DOCK_DEFAULT_MAX).contains(&d));
    }

    /// Collapsing the dock must not forget its expanded height; reopening
    /// restores it (egui keeps the panel's stored height while it is not drawn).
    #[test]
    fn dock_height_is_remembered_across_collapse() {
        let ctx = egui::Context::default();
        let mut app = empty_app();

        // One frame with the dock expanded so the panel stores a height.
        run_dock_frame(&mut app, &ctx);
        assert!(
            stored_dock_height(&ctx).is_some(),
            "the dock stores its height"
        );

        // Simulate a resize.
        force_dock_height(&ctx, 180.0);
        run_dock_frame(&mut app, &ctx);
        let expanded = stored_dock_height(&ctx).unwrap();
        assert!(
            (expanded - 180.0).abs() < 1.0,
            "resized height in use: {expanded}"
        );

        // Collapse → not created; the stored height is untouched.
        app.bottom_dock_collapsed = true;
        run_dock_frame(&mut app, &ctx);
        assert_eq!(
            stored_dock_height(&ctx),
            Some(expanded),
            "collapse must not forget the dock height"
        );

        // Reopen → the remembered height is restored.
        app.bottom_dock_collapsed = false;
        run_dock_frame(&mut app, &ctx);
        assert_eq!(stored_dock_height(&ctx), Some(expanded));
    }

    /// An over-large stored height is clamped to the ceiling on the next frame.
    #[test]
    fn dock_height_is_clamped_to_the_maximum() {
        let ctx = egui::Context::default();
        let mut app = empty_app();
        force_dock_height(&ctx, 10_000.0);
        run_dock_frame(&mut app, &ctx);
        let h = stored_dock_height(&ctx).unwrap();
        // Central panel is 600 pt high here → ceiling = 420.
        assert!(
            h <= 0.70 * 600.0 + 1.0,
            "clamped height {h} exceeds the ceiling"
        );
        assert!(h > DOCK_MIN_HEIGHT);
    }
}
