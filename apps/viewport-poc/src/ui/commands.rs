//! Typed command layer + single execution path (Phase 159-A).
//!
//! [`Command`] enumerates every user-visible action of the application with a
//! stable [`id`](Command::id), a human [`label`](Command::label) (used by the
//! command palette), a [`category`](Command::category) and an optional
//! keyboard [`shortcut`](Command::shortcut).  [`Command::ALL`] is the single
//! registry.
//!
//! Every command is executed by exactly one dispatcher —
//! [`ViewportApp::execute_command`] — which performs the action by calling the
//! **existing** handler / method for it (the same `run_solve()`, `fit_view()`,
//! `do_save()`, `select_and_open()`, theme toggle, panel toggles…).  There is
//! deliberately **no second model-editing
//! implementation**: every command ends in the same EditModel / Project /
//! EditHistory lifecycle as the menu, ribbon and shortcut paths.
//!
//! [`ViewportApp::command_is_enabled`] / [`ViewportApp::command_disabled_reason`]
//! provide the enablement predicate so disabled commands can be *listed* by
//! the palette (dimmed, with the reason) while remaining non-executable.

use egui;

use viewport_poc::app_state::{AppMessage, SolverBackend};
use viewport_poc::camera::StandardView;
use viewport_poc::display::ResultViewMode;
use viewport_poc::model::PresetModel;

use super::gl_renderer::{self, RenderMode};
use super::module::WorkModule;
use crate::ViewportApp;

/// Lower-case, underscore-separated slug used to build stable command ids.
fn slug(s: &str) -> String {
    s.to_lowercase().replace([' ', '-'], "_")
}

// ── Category ────────────────────────────────────────────────────────────

/// Coarse grouping shown by the palette next to each command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandCategory {
    /// File / project lifecycle.
    File,
    /// Undo / redo / selection edits.
    Edit,
    /// Camera, renderer and display toggles.
    View,
    /// Work-module (workspace) switching.
    Workspace,
    /// Panel / dock visibility.
    Panel,
    /// Solver and backend selection.
    Analysis,
    /// Selection commands.
    Selection,
    /// Boundary quick-adds (support / nodal load / member load).
    Boundary,
    /// Tools.
    Tools,
}

impl CommandCategory {
    /// Human label used by the palette.
    pub const fn label(self) -> &'static str {
        match self {
            CommandCategory::File => "File",
            CommandCategory::Edit => "Edit",
            CommandCategory::View => "View",
            CommandCategory::Workspace => "Workspace",
            CommandCategory::Panel => "Panel",
            CommandCategory::Analysis => "Analysis",
            CommandCategory::Selection => "Selection",
            CommandCategory::Boundary => "Boundary",
            CommandCategory::Tools => "Tools",
        }
    }
}

// ── Command registry ────────────────────────────────────────────────────

/// A single user-visible command.
///
/// Constructing a `Command` never performs the action; it is only executed by
/// [`ViewportApp::execute_command`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    // ── File ──
    /// New empty model (dirty-guarded).
    New,
    /// New from a deterministic preset model (dirty-guarded).
    LoadPreset(PresetModel),
    /// Open a project file (dirty-guarded).
    Open,
    /// Save to the current path (or the path input).
    Save,
    /// Save to the path in the file path input.
    SaveAs,
    /// Clear the project back to the empty state (dirty-guarded).
    ClearProject,
    /// Close the application.
    Exit,

    // ── Edit ──
    Undo,
    Redo,
    /// Delete the current selection (nodes cascade to attached members).
    DeleteSelection,
    /// Clear the selection.
    ClearSelection,

    // ── View ──
    /// Fit / zoom extents.
    Fit,
    /// Reset the camera to its default orientation.
    ResetView,
    /// Apply a standard preset camera orientation.
    StandardView(StandardView),
    /// Toggle the ground grid overlay.
    ToggleGrid,
    /// Toggle the global axes overlay.
    ToggleGlobalAxes,
    /// Toggle node labels.
    ToggleNodeLabels,
    /// Toggle local axes for the selected member.
    ToggleLocalAxes,
    /// Toggle between the deformed shape and the plain model.
    ToggleDeformed,
    /// Toggle the internal-force (M/V/N) diagrams overlay.
    ToggleDiagrams,
    /// Enable / disable the section-cut plane.
    ToggleSectionCut,
    /// Toggle the solid cap over the section cut (GPU renderer only).
    ToggleCap,
    /// Unhide every node and member.
    ShowAll,
    /// Switch to the software renderer.
    RendererSoftware,
    /// Switch to the GPU renderer.
    RendererGpu,
    /// Toggle per-face shading / lighting (GPU solids).
    ToggleShading,
    /// Toggle 4× MSAA on the offscreen GPU target.
    ToggleMsaa,
    /// Primary result visualization mode.
    ResultMode(ResultViewMode),
    /// Toggle the light / dark theme.
    ToggleTheme,

    // ── Workspace ──
    /// Switch the active work module.
    SwitchModule(WorkModule),

    // ── Panel ──
    /// Show / hide the left Project panel.
    ToggleLeftPanel,
    /// Show / hide the right-hand inspector panel.
    ToggleRightPanel,
    /// Show / hide the bottom Messages / Results dock.
    ToggleBottomDock,

    // ── Analysis ──
    /// Run the FEM analysis.
    RunAnalysis,
    /// Select the linear-solver backend.
    SolverBackend(SolverBackend),

    // ── Selection ──
    /// Select every visible node and member.
    SelectAll,

    // ── Boundary ──
    /// Quick-add a support at the primary selected node.
    AddSupport,
    /// Quick-add a nodal load at the primary selected node.
    AddNodeLoad,
    /// Quick-add a member load on the primary selected member.
    AddMemberLoad,

    // ── Tools ──
    /// Capture the viewport as a BMP image.
    Screenshot,
    /// Open / close the command palette itself.
    TogglePalette,
}

impl Command {
    /// The single registry — every command the palette and menus can run.
    pub const ALL: [Command; 61] = [
        // File
        Command::New,
        Command::LoadPreset(PresetModel::Portal),
        Command::LoadPreset(PresetModel::Inclined),
        Command::LoadPreset(PresetModel::Grid1x1),
        Command::LoadPreset(PresetModel::Grid3x3),
        Command::LoadPreset(PresetModel::Grid10x10),
        Command::Open,
        Command::Save,
        Command::SaveAs,
        Command::ClearProject,
        Command::Exit,
        // Edit
        Command::Undo,
        Command::Redo,
        Command::DeleteSelection,
        Command::ClearSelection,
        // View
        Command::Fit,
        Command::ResetView,
        Command::StandardView(StandardView::Iso),
        Command::StandardView(StandardView::Front),
        Command::StandardView(StandardView::Top),
        Command::StandardView(StandardView::Right),
        Command::ToggleGrid,
        Command::ToggleGlobalAxes,
        Command::ToggleNodeLabels,
        Command::ToggleLocalAxes,
        Command::ToggleDeformed,
        Command::ToggleDiagrams,
        Command::ToggleSectionCut,
        Command::ToggleCap,
        Command::ShowAll,
        Command::RendererSoftware,
        Command::RendererGpu,
        Command::ToggleShading,
        Command::ToggleMsaa,
        Command::ResultMode(ResultViewMode::Model),
        Command::ResultMode(ResultViewMode::Deformed),
        Command::ResultMode(ResultViewMode::Displacement),
        Command::ResultMode(ResultViewMode::Reaction),
        Command::ResultMode(ResultViewMode::MemberEndForces),
        Command::ToggleTheme,
        // Workspace
        Command::SwitchModule(WorkModule::Model),
        Command::SwitchModule(WorkModule::Section),
        Command::SwitchModule(WorkModule::Load),
        Command::SwitchModule(WorkModule::Analysis),
        Command::SwitchModule(WorkModule::Results),
        // Panel
        Command::ToggleLeftPanel,
        Command::ToggleRightPanel,
        Command::ToggleBottomDock,
        // Analysis
        Command::RunAnalysis,
        Command::SolverBackend(SolverBackend::Auto),
        Command::SolverBackend(SolverBackend::Dense),
        Command::SolverBackend(SolverBackend::SkylineLdlt),
        Command::SolverBackend(SolverBackend::SparseLu),
        Command::SolverBackend(SolverBackend::Cg),
        Command::SolverBackend(SolverBackend::Iccg),
        // Selection
        Command::SelectAll,
        // Boundary
        Command::AddSupport,
        Command::AddNodeLoad,
        Command::AddMemberLoad,
        // Tools
        Command::Screenshot,
        Command::TogglePalette,
    ];

    /// Stable identifier (also searchable by the palette).
    pub fn id(&self) -> String {
        match self {
            Command::New => "file.new".into(),
            Command::LoadPreset(p) => format!("file.preset.{}", slug(p.name())),
            Command::Open => "file.open".into(),
            Command::Save => "file.save".into(),
            Command::SaveAs => "file.save_as".into(),
            Command::ClearProject => "file.clear".into(),
            Command::Exit => "file.exit".into(),
            Command::Undo => "edit.undo".into(),
            Command::Redo => "edit.redo".into(),
            Command::DeleteSelection => "edit.delete".into(),
            Command::ClearSelection => "edit.clear_selection".into(),
            Command::Fit => "view.fit".into(),
            Command::ResetView => "view.reset".into(),
            Command::StandardView(v) => format!("view.standard.{}", slug(v.label())),
            Command::ToggleGrid => "view.grid".into(),
            Command::ToggleGlobalAxes => "view.global_axes".into(),
            Command::ToggleNodeLabels => "view.node_labels".into(),
            Command::ToggleLocalAxes => "view.local_axes".into(),
            Command::ToggleDeformed => "view.deformed".into(),
            Command::ToggleDiagrams => "view.diagrams".into(),
            Command::ToggleSectionCut => "view.section_cut".into(),
            Command::ToggleCap => "view.cap".into(),
            Command::ShowAll => "view.show_all".into(),
            Command::RendererSoftware => "view.renderer.software".into(),
            Command::RendererGpu => "view.renderer.gpu".into(),
            Command::ToggleShading => "view.shading".into(),
            Command::ToggleMsaa => "view.msaa".into(),
            Command::ResultMode(m) => format!("results.mode.{}", slug(m.label())),
            Command::ToggleTheme => "view.theme".into(),
            Command::SwitchModule(m) => format!("workspace.module.{}", slug(m.label())),
            Command::ToggleLeftPanel => "panel.left".into(),
            Command::ToggleRightPanel => "panel.right".into(),
            Command::ToggleBottomDock => "panel.bottom".into(),
            Command::RunAnalysis => "analysis.run".into(),
            Command::SolverBackend(b) => {
                format!("analysis.backend.{}", b.name().unwrap_or("auto"))
            }
            Command::SelectAll => "selection.all".into(),
            Command::AddSupport => "boundary.support".into(),
            Command::AddNodeLoad => "boundary.node_load".into(),
            Command::AddMemberLoad => "boundary.member_load".into(),
            Command::Screenshot => "tools.screenshot".into(),
            Command::TogglePalette => "tools.palette".into(),
        }
    }

    /// Human-readable label shown by the palette and menus.
    pub fn label(&self) -> String {
        match self {
            Command::New => "New Empty Model".into(),
            Command::LoadPreset(p) => format!("New: {}", p.name()),
            Command::Open => "Open Project\u{2026}".into(),
            Command::Save => "Save".into(),
            Command::SaveAs => "Save Project As\u{2026}".into(),
            Command::ClearProject => "Clear Project".into(),
            Command::Exit => "Exit".into(),
            Command::Undo => "Undo".into(),
            Command::Redo => "Redo".into(),
            Command::DeleteSelection => "Delete Selection".into(),
            Command::ClearSelection => "Clear Selection".into(),
            Command::Fit => "Fit / Zoom Extents".into(),
            Command::ResetView => "Reset View".into(),
            Command::StandardView(v) => format!("Standard View: {}", v.label()),
            Command::ToggleGrid => "Toggle Ground Grid".into(),
            Command::ToggleGlobalAxes => "Toggle Global Axes".into(),
            Command::ToggleNodeLabels => "Toggle Node Labels".into(),
            Command::ToggleLocalAxes => "Toggle Local Axes".into(),
            Command::ToggleDeformed => "Toggle Deformed Shape".into(),
            Command::ToggleDiagrams => "Toggle Internal-Force Diagrams".into(),
            Command::ToggleSectionCut => "Toggle Section Cut".into(),
            Command::ToggleCap => "Toggle Cut-Face Cap".into(),
            Command::ShowAll => "Show All (Unhide)".into(),
            Command::RendererSoftware => "Renderer: Software".into(),
            Command::RendererGpu => "Renderer: GPU".into(),
            Command::ToggleShading => "Toggle Shading / Lighting".into(),
            Command::ToggleMsaa => "Toggle MSAA (4\u{d7})".into(),
            Command::ResultMode(m) => format!("Result View: {}", m.label()),
            Command::ToggleTheme => "Toggle Light / Dark Theme".into(),
            Command::SwitchModule(m) => format!("Switch to {} Module", m.label()),
            Command::ToggleLeftPanel => "Toggle Project Panel".into(),
            Command::ToggleRightPanel => "Toggle Right Panel".into(),
            Command::ToggleBottomDock => "Toggle Output Dock".into(),
            Command::RunAnalysis => "Run Analysis".into(),
            Command::SolverBackend(b) => format!("Solver Backend: {}", b.label()),
            Command::SelectAll => "Select All Visible".into(),
            Command::AddSupport => "Boundary: Add Support".into(),
            Command::AddNodeLoad => "Boundary: Add Nodal Load".into(),
            Command::AddMemberLoad => "Boundary: Add Member Load".into(),
            Command::Screenshot => "Viewport Screenshot".into(),
            Command::TogglePalette => "Command Palette".into(),
        }
    }

    /// Category used by the palette for grouping / display.
    pub const fn category(&self) -> CommandCategory {
        match self {
            Command::New
            | Command::LoadPreset(_)
            | Command::Open
            | Command::Save
            | Command::SaveAs
            | Command::ClearProject
            | Command::Exit => CommandCategory::File,
            Command::Undo | Command::Redo | Command::DeleteSelection | Command::ClearSelection => {
                CommandCategory::Edit
            }
            Command::Fit
            | Command::ResetView
            | Command::StandardView(_)
            | Command::ToggleGrid
            | Command::ToggleGlobalAxes
            | Command::ToggleNodeLabels
            | Command::ToggleLocalAxes
            | Command::ToggleDeformed
            | Command::ToggleDiagrams
            | Command::ToggleSectionCut
            | Command::ToggleCap
            | Command::ShowAll
            | Command::RendererSoftware
            | Command::RendererGpu
            | Command::ToggleShading
            | Command::ToggleMsaa
            | Command::ResultMode(_)
            | Command::ToggleTheme => CommandCategory::View,
            Command::SwitchModule(_) => CommandCategory::Workspace,
            Command::ToggleLeftPanel | Command::ToggleRightPanel | Command::ToggleBottomDock => {
                CommandCategory::Panel
            }
            Command::RunAnalysis | Command::SolverBackend(_) => CommandCategory::Analysis,
            Command::SelectAll => CommandCategory::Selection,
            Command::AddSupport | Command::AddNodeLoad | Command::AddMemberLoad => {
                CommandCategory::Boundary
            }
            Command::Screenshot | Command::TogglePalette => CommandCategory::Tools,
        }
    }

    /// Keyboard shortcut for the command, when one is bound.
    ///
    /// Only *real* bindings are advertised (the ones listed in the Help menu
    /// and handled in `handle_shortcuts` / `main.rs`); commands without a
    /// binding return `None`.
    pub const fn shortcut(&self) -> Option<&'static str> {
        match self {
            Command::Undo => Some("Ctrl+Z"),
            Command::Redo => Some("Ctrl+Y"),
            Command::DeleteSelection => Some("Del"),
            Command::ClearSelection => Some("Esc"),
            Command::Fit => Some("F"),
            Command::ResetView => Some("R"),
            Command::StandardView(StandardView::Iso) => Some("1"),
            Command::StandardView(StandardView::Front) => Some("2"),
            Command::StandardView(StandardView::Top) => Some("3"),
            Command::StandardView(StandardView::Right) => Some("4"),
            Command::SelectAll => Some("Ctrl+A"),
            Command::ToggleBottomDock => Some("Ctrl+M"),
            Command::ToggleRightPanel => Some("Ctrl+I"),
            Command::Screenshot => Some("Ctrl+Shift+S"),
            Command::TogglePalette => Some("Ctrl+Shift+P"),
            _ => None,
        }
    }
}

/// Rank the whole registry against `query` (best match first).
///
/// Matching runs over `label`, `category` and `id`; ties are broken by label
/// so the order is deterministic.  An empty query returns every command.
pub fn search(query: &str) -> Vec<Command> {
    let mut scored: Vec<(i32, Command)> = Command::ALL
        .iter()
        .filter_map(|&cmd| {
            let haystacks = [cmd.label(), cmd.category().label().to_string(), cmd.id()];
            let refs: [&str; 3] = [&haystacks[0], &haystacks[1], &haystacks[2]];
            viewport_poc::commands::best_score(query, &refs).map(|score| (score, cmd))
        })
        .collect();
    scored.sort_by(|a, b| {
        b.0.cmp(&a.0)
            .then_with(|| a.1.label().cmp(&b.1.label()))
            .then_with(|| a.1.id().cmp(&b.1.id()))
    });
    scored.into_iter().map(|(_, cmd)| cmd).collect()
}

// ── Enablement + dispatch ───────────────────────────────────────────────

impl ViewportApp {
    /// Whether `cmd` can run right now (the palette's "unavailable" test).
    pub(crate) fn command_is_enabled(&self, cmd: Command) -> bool {
        self.command_disabled_reason(cmd).is_none()
    }

    /// Why `cmd` cannot run, or `None` when it is enabled.
    ///
    /// The reason is shown by the palette (and used as the menu's
    /// `add_enabled` guard) so a disabled command is explained, never silent.
    pub(crate) fn command_disabled_reason(&self, cmd: Command) -> Option<&'static str> {
        match cmd {
            Command::Undo => (!self.history.can_undo()).then_some("Nothing to undo"),
            Command::Redo => (!self.history.can_redo()).then_some("Nothing to redo"),
            Command::DeleteSelection | Command::ClearSelection => {
                self.selected.is_empty().then_some("Nothing is selected")
            }
            Command::SelectAll => self.project.model.is_none().then_some("No model loaded"),
            Command::Fit | Command::ResetView => self.scene.is_none().then_some("No model loaded"),
            Command::RunAnalysis => {
                (!self.can_run_analysis()).then_some("Load a model with at least one node")
            }
            Command::RendererGpu | Command::ToggleShading | Command::ToggleMsaa => {
                (!gl_renderer::available()).then_some("GPU renderer unavailable")
            }
            Command::ShowAll => (!self.display.has_hidden()).then_some("Nothing is hidden"),
            Command::AddSupport | Command::AddNodeLoad => self
                .selected
                .primary()
                .node_id()
                .is_none()
                .then_some("Select a node first"),
            Command::AddMemberLoad => self
                .selected
                .primary()
                .member_id()
                .is_none()
                .then_some("Select a member first"),
            _ => None,
        }
    }

    /// The single execution path: run `cmd` through the **existing** handler.
    ///
    /// A disabled command is refused (a warning naming the reason is pushed
    /// to the Messages dock) and nothing else happens.  Every model-editing
    /// command below reuses the shared `commit_edit` / undo lifecycle — there
    /// is no second implementation.
    pub(crate) fn execute_command(&mut self, cmd: Command, ctx: &egui::Context) {
        if let Some(reason) = self.command_disabled_reason(cmd) {
            self.messages.push(AppMessage::warning(format!(
                "{} \u{2014} unavailable: {reason}",
                cmd.label()
            )));
            return;
        }
        match cmd {
            // ── File ──
            Command::New => self.request_new(),
            Command::LoadPreset(preset) => self.request_load_preset(preset),
            Command::Open => self.request_open(),
            Command::Save => {
                self.do_save();
            }
            Command::SaveAs => {
                self.do_save_as();
            }
            Command::ClearProject => self.request_clear(),
            Command::Exit => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
            // ── Edit ──
            Command::Undo => self.do_undo(),
            Command::Redo => self.do_redo(),
            Command::DeleteSelection => {
                self.delete_selection();
            }
            Command::ClearSelection => self.clear_selection(),
            // ── View ──
            Command::Fit => self.fit_view(),
            Command::ResetView => self.camera.reset(),
            Command::StandardView(view) => self.set_standard_view(view),
            Command::ToggleGrid => self.display.show_grid = !self.display.show_grid,
            Command::ToggleGlobalAxes => {
                self.display.show_global_axes = !self.display.show_global_axes
            }
            Command::ToggleNodeLabels => {
                self.display.show_node_labels = !self.display.show_node_labels
            }
            Command::ToggleLocalAxes => {
                self.display.show_local_axes = !self.display.show_local_axes
            }
            Command::ToggleDeformed => {
                self.display.mode = if self.display.mode == ResultViewMode::Deformed {
                    ResultViewMode::Model
                } else {
                    ResultViewMode::Deformed
                };
            }
            Command::ToggleDiagrams => self.display.show_diagrams = !self.display.show_diagrams,
            Command::ToggleSectionCut => {
                let on = !self.display.section_cut.enabled;
                self.set_cut_enabled(on);
            }
            Command::ToggleCap => self.display.section_cut.cap = !self.display.section_cut.cap,
            Command::ShowAll => self.show_all_hidden(),
            Command::RendererSoftware => {
                gl_renderer::set_mode(RenderMode::Software);
            }
            Command::RendererGpu => {
                if !gl_renderer::set_mode(RenderMode::Gpu) {
                    self.messages.push(AppMessage::error(
                        "GPU renderer unavailable \u{2014} see Messages",
                    ));
                }
            }
            Command::ToggleShading => {
                let on = gl_renderer::shading_enabled();
                gl_renderer::set_shading(!on);
            }
            Command::ToggleMsaa => {
                let on = gl_renderer::msaa_enabled();
                gl_renderer::set_msaa(!on);
            }
            Command::ResultMode(mode) => self.display.mode = mode,
            Command::ToggleTheme => self.dark_mode = !self.dark_mode,
            // ── Workspace ──
            Command::SwitchModule(module) => self.module = module,
            // ── Panel ──
            Command::ToggleLeftPanel => self.left_panel_collapsed = !self.left_panel_collapsed,
            Command::ToggleRightPanel => self.inspector_collapsed = !self.inspector_collapsed,
            Command::ToggleBottomDock => self.bottom_dock_collapsed = !self.bottom_dock_collapsed,
            // ── Analysis ──
            Command::RunAnalysis => self.run_solve(),
            Command::SolverBackend(backend) => self.project.set_solver_backend(backend),
            // ── Selection ──
            Command::SelectAll => self.select_all_visible(),
            // ── Boundary ──
            Command::AddSupport => self.open_support_popup(ctx),
            Command::AddNodeLoad => self.open_nodal_load_popup(ctx),
            Command::AddMemberLoad => self.open_member_load_popup(ctx),
            // ── Tools ──
            Command::Screenshot => super::screenshot::request(ctx),
            Command::TogglePalette => super::palette::toggle(self),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::EditModel;
    use viewport_poc::persistence;
    use viewport_poc::selection::SelectionState;

    /// An app with no model, for the guard paths.
    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// An app holding one 4 m beam (nodes N0/N1, member M0) with a live scene.
    fn beam_app() -> (
        ViewportApp,
        viewport_poc::edit_model::NodeId,
        viewport_poc::edit_model::MemberId,
    ) {
        let mut app = empty_app();
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        app.project.load_model("test".to_string(), m);
        app.rebuild_scene();
        (app, a, mid)
    }

    /// An app holding a solvable preset (portal frame, fixed bases).
    fn solved_app() -> ViewportApp {
        let mut app = empty_app();
        app.project
            .load_model("portal".to_string(), PresetModel::Portal.build_edit());
        app.rebuild_scene();
        app.run_solve();
        app
    }

    fn last_message(app: &ViewportApp) -> &str {
        &app.messages.last().expect("no message").text
    }

    // ── Registry invariants ─────────────────────────────────────────────

    #[test]
    fn registry_ids_are_unique_and_nonempty() {
        let mut ids: Vec<String> = Command::ALL.iter().map(|c| c.id()).collect();
        let before = ids.len();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), before, "command ids must be unique");
        for cmd in Command::ALL {
            assert!(!cmd.id().is_empty());
            assert!(!cmd.label().is_empty(), "{:?} has an empty label", cmd);
            assert!(!cmd.category().label().is_empty());
        }
    }

    #[test]
    fn every_category_is_populated() {
        const CATEGORIES: [CommandCategory; 9] = [
            CommandCategory::File,
            CommandCategory::Edit,
            CommandCategory::View,
            CommandCategory::Workspace,
            CommandCategory::Panel,
            CommandCategory::Analysis,
            CommandCategory::Selection,
            CommandCategory::Boundary,
            CommandCategory::Tools,
        ];
        for cat in CATEGORIES {
            assert!(
                Command::ALL.iter().any(|c| c.category() == cat),
                "category {:?} has no commands",
                cat
            );
        }
    }

    #[test]
    fn disabled_commands_have_a_reason_and_enabled_ones_do_not() {
        let (app, _a, _mid) = beam_app();
        for cmd in Command::ALL {
            match app.command_disabled_reason(cmd) {
                Some(reason) => {
                    assert!(!reason.is_empty());
                    assert!(!app.command_is_enabled(cmd));
                }
                None => assert!(app.command_is_enabled(cmd)),
            }
        }
    }

    // ── Palette search (pure) ───────────────────────────────────────────

    #[test]
    fn search_empty_query_returns_every_command() {
        assert_eq!(search("").len(), Command::ALL.len());
    }

    #[test]
    fn search_ranks_the_exact_label_first() {
        let undo = search("undo");
        assert_eq!(undo[0], Command::Undo);
        let fit = search("fit");
        assert_eq!(fit[0], Command::Fit);
        let theme = search("theme");
        assert_eq!(theme[0], Command::ToggleTheme);
    }

    #[test]
    fn search_matches_ids_and_categories() {
        assert!(search("file.save").contains(&Command::Save));
        assert!(search("workspace").contains(&Command::SwitchModule(WorkModule::Model)));
        assert!(search("boundary").contains(&Command::AddSupport));
        assert!(search("zzzz-not-a-command").is_empty());
    }

    #[test]
    fn palette_flow_executes_the_highlighted_enabled_command() {
        // Mimic the palette: query → ranked list → highlight row 0 → dispatch.
        let (mut app, _a, _mid) = beam_app();
        let results = search("select all");
        let highlighted = results[0];
        assert_eq!(highlighted, Command::SelectAll);
        assert!(app.command_is_enabled(highlighted));
        let ctx = egui::Context::default();
        app.execute_command(highlighted, &ctx);
        assert_eq!(app.selected.node_count(), 2);
        assert_eq!(app.selected.member_count(), 1);
    }

    #[test]
    fn palette_flow_refuses_the_highlighted_disabled_command() {
        let mut app = empty_app();
        let results = search("run analysis");
        let highlighted = results[0];
        assert_eq!(highlighted, Command::RunAnalysis);
        assert!(!app.command_is_enabled(highlighted));
        assert_eq!(
            app.command_disabled_reason(highlighted),
            Some("Load a model with at least one node")
        );
        let ctx = egui::Context::default();
        app.execute_command(highlighted, &ctx);
        // Refused: no result, and the reason is surfaced.
        assert!(app.project.model.is_none());
        assert!(app.analysis_result().is_none());
        assert!(last_message(&app).contains("unavailable"));
    }

    // ── Dispatch: enabled commands perform the action ───────────────────

    #[test]
    fn dispatch_toggle_grid_flips_the_overlay() {
        let (mut app, _a, _mid) = beam_app();
        let before = app.display.show_grid;
        let ctx = egui::Context::default();
        app.execute_command(Command::ToggleGrid, &ctx);
        assert_eq!(app.display.show_grid, !before);
        app.execute_command(Command::ToggleGrid, &ctx);
        assert_eq!(app.display.show_grid, before);
    }

    #[test]
    fn dispatch_standard_view_moves_the_camera() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::StandardView(StandardView::Front), &ctx);
        assert_eq!(app.camera.azimuth, std::f64::consts::FRAC_PI_2);
        // Fit re-frames the scene bounding box after the camera moved away.
        app.camera.distance = 99.0;
        app.execute_command(Command::Fit, &ctx);
        assert_ne!(app.camera.distance, 99.0);
    }

    #[test]
    fn dispatch_view_module_theme_and_panels() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::StandardView(StandardView::Top), &ctx);
        assert_eq!(app.camera.azimuth, StandardView::Top.angles().0);
        app.execute_command(Command::SwitchModule(WorkModule::Load), &ctx);
        assert_eq!(app.module, WorkModule::Load);
        let dark = app.dark_mode;
        app.execute_command(Command::ToggleTheme, &ctx);
        assert_eq!(app.dark_mode, !dark);
        app.execute_command(Command::ToggleBottomDock, &ctx);
        assert!(app.bottom_dock_collapsed);
        app.execute_command(Command::ToggleRightPanel, &ctx);
        assert!(app.inspector_collapsed);
        app.execute_command(Command::ToggleRightPanel, &ctx);
        assert!(!app.inspector_collapsed);
    }

    /// Panel visibility toggles are pure view state (Phase 159-B): they flip
    /// the collapsed flags and never touch the model or the selection.
    #[test]
    fn dispatch_panel_toggles_are_pure_view_state() {
        let (mut app, a, _mid) = beam_app();
        app.select_only(SelectionState::Node(a));
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        let ctx = egui::Context::default();

        assert!(!app.left_panel_collapsed);
        app.execute_command(Command::ToggleLeftPanel, &ctx);
        assert!(app.left_panel_collapsed);
        app.execute_command(Command::ToggleLeftPanel, &ctx);
        assert!(!app.left_panel_collapsed);

        app.execute_command(Command::ToggleRightPanel, &ctx);
        assert!(app.inspector_collapsed);
        app.execute_command(Command::ToggleBottomDock, &ctx);
        assert!(app.bottom_dock_collapsed);

        assert_eq!(
            app.project.model.as_ref().unwrap().fingerprint(),
            fingerprint,
            "panel toggles must not touch the model"
        );
        assert_eq!(app.selected.primary(), SelectionState::Node(a));
    }

    #[test]
    fn dispatch_solver_backend_command_selects_backend() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::SolverBackend(SolverBackend::Dense), &ctx);
        assert_eq!(app.project.solver_backend(), SolverBackend::Dense);
    }

    #[test]
    fn dispatch_section_cut_and_cap_toggles() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        assert!(!app.display.section_cut.enabled);
        app.execute_command(Command::ToggleSectionCut, &ctx);
        assert!(app.display.section_cut.enabled);
        app.execute_command(Command::ToggleSectionCut, &ctx);
        assert!(!app.display.section_cut.enabled);
        let cap = app.display.section_cut.cap;
        app.execute_command(Command::ToggleCap, &ctx);
        assert_eq!(app.display.section_cut.cap, !cap);
    }

    #[test]
    fn dispatch_add_support_popup_requires_a_node_selection() {
        let (mut app, a, _mid) = beam_app();
        let ctx = egui::Context::default();
        // Disabled without a node selection — refused.
        assert!(!app.command_is_enabled(Command::AddSupport));
        app.execute_command(Command::AddSupport, &ctx);
        assert!(app.boundary_popup.is_none());
        assert!(last_message(&app).contains("Select a node first"));
        // With a node selected the popup opens (same path as the ribbon).
        app.select_only(SelectionState::Node(a));
        assert!(app.command_is_enabled(Command::AddSupport));
        app.execute_command(Command::AddSupport, &ctx);
        assert!(matches!(
            app.boundary_popup,
            Some(crate::BoundaryPopup::Support { .. })
        ));
    }

    #[test]
    fn dispatch_undo_redo_through_commands() {
        let (mut app, a, _mid) = beam_app();
        let ctx = egui::Context::default();
        // Fresh app: undo/redo are disabled and refused.
        assert!(!app.command_is_enabled(Command::Undo));
        assert!(!app.command_is_enabled(Command::Redo));
        app.execute_command(Command::Undo, &ctx);
        assert!(last_message(&app).contains("Nothing to undo"));
        // A real edit (through the shared quick path) makes undo available.
        app.apply_support_quick(a, viewport_poc::edit_model::SupportSpec::fixed())
            .unwrap();
        assert!(app.command_is_enabled(Command::Undo));
        app.execute_command(Command::Undo, &ctx);
        assert!(app.project.model.as_ref().unwrap().support(a).is_none());
        app.execute_command(Command::Redo, &ctx);
        assert!(app.project.model.as_ref().unwrap().support(a).is_some());
    }

    #[test]
    fn dispatch_delete_selection_is_one_undo_step() {
        let (mut app, _a, mid) = beam_app();
        let ctx = egui::Context::default();
        let fingerprint = app.project.model.as_ref().unwrap().fingerprint();
        let revision = app.project.model.as_ref().unwrap().revision();
        app.select_only(SelectionState::Member(mid));
        app.execute_command(Command::DeleteSelection, &ctx);
        {
            let model = app.project.model.as_ref().unwrap();
            assert_eq!(model.n_members(), 0);
            assert_eq!(model.n_nodes(), 2, "member delete must not cascade here");
            assert!(model.revision() > revision, "edit must bump the revision");
        }
        assert!(app.selected.is_empty(), "selection is cleared after delete");
        // One undo restores the model content exactly.
        app.execute_command(Command::Undo, &ctx);
        let restored = app.project.model.as_ref().unwrap();
        assert_eq!(restored.n_members(), 1);
        assert_eq!(restored.fingerprint(), fingerprint);
    }

    #[test]
    fn dispatch_edit_marks_a_solved_result_stale() {
        let mut app = solved_app();
        assert_eq!(
            app.project.analysis_status(),
            viewport_poc::app_state::AnalysisStatus::Current,
            "the portal preset must solve: {}",
            last_message(&app)
        );
        let ctx = egui::Context::default();
        let node = app.project.model.as_ref().unwrap().nodes()[0].id;
        app.select_only(SelectionState::Node(node));
        app.execute_command(Command::DeleteSelection, &ctx);
        assert!(
            app.project.results_are_stale(),
            "edit must mark results stale"
        );
        assert_eq!(
            app.project.analysis_status(),
            viewport_poc::app_state::AnalysisStatus::Stale
        );
    }

    #[test]
    fn dispatch_run_analysis_solves_the_model() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        // An unsupported beam cannot solve — the failure is surfaced, not hidden.
        app.execute_command(Command::RunAnalysis, &ctx);
        assert!(last_message(&app).contains("Analysis failed"));
        // Add a fixed base through the shared quick path, then solve.
        let node = app.project.model.as_ref().unwrap().nodes()[0].id;
        app.select_only(SelectionState::Node(node));
        app.apply_support_quick(node, viewport_poc::edit_model::SupportSpec::fixed())
            .unwrap();
        app.execute_command(Command::RunAnalysis, &ctx);
        assert!(last_message(&app).contains("Analysis complete"));
        assert!(app.analysis_result().is_some());
    }

    #[test]
    fn dispatch_save_surfaces_errors_without_a_model() {
        let mut app = empty_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::Save, &ctx);
        assert!(last_message(&app).contains("No file path"));
        app.execute_command(Command::SaveAs, &ctx);
        assert!(last_message(&app).contains("No file path"));
    }

    // ── File path: Open / Save never dead-end ───────────────────────────

    #[test]
    fn open_without_a_path_reveals_the_field_and_guides_the_user() {
        let mut app = empty_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::Open, &ctx);
        assert!(app.focus_project_path, "the path field must be revealed");
        let msg = last_message(&app);
        assert!(msg.contains("No file path"), "{msg}");
        assert!(msg.contains("choose Open"), "{msg}");
        // Nothing was loaded and no bogus path was recorded.
        assert!(app.project.model.is_none());
        assert!(app.project_path.is_none());
    }

    #[test]
    fn save_without_a_path_reveals_the_field_and_guides_the_user() {
        let mut app = empty_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::Save, &ctx);
        assert!(app.focus_project_path, "the path field must be revealed");
        let msg = last_message(&app);
        assert!(msg.contains("No file path"), "{msg}");
        assert!(msg.contains("choose Save"), "{msg}");
    }

    #[test]
    fn successful_open_remembers_and_persists_the_path() {
        let dir = std::env::temp_dir();
        let path = dir.join("viewport_poc_open_remembers.saproject");
        let settings_path = dir.join("viewport_poc_open_remembers_settings.json");
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&settings_path);

        let mut model = EditModel::new();
        let a = model.add_node(0.0, 0.0, 0.0).unwrap();
        let b = model.add_node(2.0, 0.0, 0.0).unwrap();
        model.add_member(a, b).unwrap();
        persistence::save_project_with_section(&path, &model, None).unwrap();

        let mut app = ViewportApp::new(
            persistence::AppSettings::default(),
            settings_path.clone(),
            None,
        );
        app.file_path_input = path.to_string_lossy().to_string();
        let ctx = egui::Context::default();
        app.execute_command(Command::Open, &ctx);

        assert!(app.project.model.is_some(), "{}", last_message(&app));
        assert_eq!(app.last_project_path.as_deref(), Some(path.as_path()));
        // The remembered path is persisted, so the next launch can use it too.
        let (settings, warning) = persistence::load_settings_or_default(&settings_path);
        assert!(warning.is_none());
        assert_eq!(settings.last_project_path.as_deref(), path.to_str());

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&settings_path);
    }

    #[test]
    fn dispatch_show_all_unhides_everything() {
        let (mut app, a, mid) = beam_app();
        let ctx = egui::Context::default();
        assert!(!app.command_is_enabled(Command::ShowAll));
        app.hide_node(a);
        app.hide_member(mid);
        assert!(app.command_is_enabled(Command::ShowAll));
        app.execute_command(Command::ShowAll, &ctx);
        assert!(!app.display.has_hidden());
    }

    #[test]
    fn dispatch_palette_command_toggles_the_palette() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        assert!(app.palette.is_open());
        app.execute_command(Command::TogglePalette, &ctx);
        assert!(!app.palette.is_open());
    }

    #[test]
    fn dispatch_screenshot_is_a_no_op_without_a_window() {
        let (mut app, _a, _mid) = beam_app();
        let ctx = egui::Context::default();
        // Must not panic on a headless context (the command is queued).
        app.execute_command(Command::Screenshot, &ctx);
    }

    #[test]
    fn selection_projection_stays_in_lock_step_through_commands() {
        let (mut app, a, mid) = beam_app();
        let ctx = egui::Context::default();
        app.select_only(SelectionState::Node(a));
        assert_eq!(app.selection, SelectionState::Node(a));
        app.execute_command(Command::SelectAll, &ctx);
        assert_eq!(app.selection, app.selected.primary());
        app.execute_command(Command::ClearSelection, &ctx);
        assert!(app.selected.is_empty());
        assert_eq!(app.selection, SelectionState::None);
        // Clear Selection with nothing selected is refused, not silent.
        let before = app.messages.len();
        app.execute_command(Command::ClearSelection, &ctx);
        assert_eq!(app.messages.len(), before + 1);
        assert!(last_message(&app).contains("Nothing is selected"));
        let _ = mid;
    }
}
