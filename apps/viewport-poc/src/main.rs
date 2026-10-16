//! Desktop application — egui + eframe.
//!
//! Run with: `cargo run -p viewport-poc`
//!
//! This file defines the [`ViewportApp`] state, its `eframe::App`
//! implementation (which orchestrates the panel modules in [`ui`]), and the
//! `main()` entry point.  The application command logic lives in
//! [`ui::actions`]; all rendering lives in the other [`ui`] submodules.

// The UI module tree is part of the binary crate so it can reach the
// application state defined below.
mod ui;

use viewport_poc::{
    app_state::{AppMessage, Project},
    camera::Camera3D,
    display::ResultDisplayState,
    edit_history::EditHistory,
    edit_model::{MemberId, MemberPointLoad3D, NodalLoadSpec, NodeId, SupportSpec},
    model, persistence,
    scene::ViewportScene,
    selection::{SelectionSet, SelectionState},
};

use ui::WorkModule;
use ui::inspector::RightPanelMode;
use ui::status::BottomTab;

/// Minimum inner window size (points).
///
/// The window is undecorated and its chrome (title bar, module strip, status
/// line) is drawn by the app, so it must not be squeezable into a size where
/// the custom title bar and resize grips become unusable.
const MIN_INNER_SIZE: [f32; 2] = [700.0, 450.0];

/// The window title.  Taskbar / OS text and the handle `ui::titlebar` uses to
/// ask Windows for rounded corners — a single constant so the two can never
/// drift apart.  The string itself is unchanged.
const WINDOW_TITLE: &str = "Section Properties & Frame Analysis";

// ── App state ───────────────────────────────────────────────────────────
struct ViewportApp {
    project: Project,
    scene: Option<ViewportScene>,
    camera: Camera3D,
    /// Authoritative multi-selection (Phase P6).
    selected: SelectionSet,
    /// Single-entity projection of [`selected`](Self::selected): the
    /// *primary* (last-clicked) entity.  Kept in lock-step with
    /// [`selected`](Self::selected) by [`ViewportApp::set_selection`]; it exists
    /// only so the legacy single-entity consumers (`ui/diagrams.rs`,
    /// `ui/section.rs`) keep reading a plain [`SelectionState`].
    selection: SelectionState,
    display: ResultDisplayState,
    messages: Vec<AppMessage>,
    new_node_x: String,
    new_node_y: String,
    new_node_z: String,
    new_member_i: String,
    new_member_j: String,
    member_edit: MemberEditBuffer,
    project_path: Option<std::path::PathBuf>,
    /// Path of the last successfully opened / saved project.  Restored from
    /// settings at startup and re-persisted on every successful open / save, so
    /// **Open** / **Save** work immediately without re-typing a path.
    last_project_path: Option<std::path::PathBuf>,
    saved_fingerprint: Option<u64>,
    file_path_input: String,
    /// One-shot request raised when Open / Save has no usable path: the left
    /// panel's "Project file" section expands and focuses its path field on the
    /// next frame, guiding the user instead of dead-ending.  In-memory only.
    focus_project_path: bool,
    show_unsaved_dialog: bool,
    pending_action: PendingAction,
    history: EditHistory,
    /// Active MIDAS-style work module.  The top module strip is the **single**
    /// switcher: it selects both the inspector sections and the centre
    /// workspace content (`ui::workspace::CentreContent::for_module`).  Pure
    /// view state — it never changes the model.
    module: WorkModule,
    /// Active bottom-dock tab.
    bottom_tab: BottomTab,
    /// Whether the Messages / Results dock is collapsed.  Kept in memory only
    /// (not persisted): a collapsed dock frees its space for the viewport.
    bottom_dock_collapsed: bool,
    /// Whether the right-hand inspector panel is collapsed.  Kept in memory
    /// only (not persisted): a collapsed panel is not created at all, so the
    /// viewport reclaims its space.
    inspector_collapsed: bool,
    /// Whether the left Project panel is collapsed (Phase 159-B).
    left_panel_collapsed: bool,
    /// Active right-panel mode (Inspector / Agent / Outline, Phase 159-B).
    right_panel_mode: RightPanelMode,
    /// Section-module builder state (definition + cached properties).
    section_editor: ui::section::SectionEditorState,
    /// egui chrome theme (true = dark).
    dark_mode: bool,
    /// Path of the persisted app-settings file.
    settings_path: std::path::PathBuf,
    /// `dark_mode` value last written to the settings file.
    last_persisted_dark_mode: bool,
    /// Last observed window size (points), captured each frame for persistence.
    window_size: Option<[f32; 2]>,
    /// Entity that was under the cursor when the viewport context menu opened
    /// (captured on the right-click frame so the menu content is stable).
    ctx_menu_target: SelectionState,
    /// Double-click quick-edit popup state; `None` = closed.
    quick_edit: Option<QuickEdit>,
    /// Boundary quick-add popup state (Support / Node Load / Mem Load,
    /// opened from the Boundary menu or the command palette); `None` = closed.
    boundary_popup: Option<BoundaryPopup>,
    /// Screen position where a rubber-band box-select drag began
    /// (a plain primary drag), or `None` when no box select is active.
    box_select_start: Option<egui::Pos2>,
    /// Command Palette state (Phase 159-A, Ctrl+Shift+P); in-memory only.
    palette: ui::palette::PaletteState,
    /// Load-module view state: staging buffer for a new load case's name.
    new_case_name: String,
    /// Load-module view state: staging buffer for a new combination's name.
    new_combo_name: String,
    /// Combination currently open in the combination term editor.
    editing_combination: Option<viewport_poc::edit_model::LoadCaseId>,
    /// Staged combination terms `(case, factor)` for the term editor; the
    /// model is updated only when the user applies.
    combo_terms_buffer: Vec<(viewport_poc::edit_model::LoadCaseId, f64)>,
}

/// Action waiting for unsaved-changes confirmation.
#[derive(Clone, Copy, PartialEq)]
enum PendingAction {
    None,
    New,
    Open,
    Clear,
    LoadPreset(model::PresetModel),
}

/// Temporary buffer for member property editing.
///
/// Holds section (E, G, A, Iy, Iz, J) and ref_vec (rx, ry, rz) values
/// being edited in the inspector.  Values are only applied to the
/// EditModel when the user clicks "Apply".
struct MemberEditBuffer {
    /// The MemberId this buffer corresponds to.
    member_id: Option<MemberId>,
    e: f64,
    g: f64,
    area: f64,
    iy: f64,
    iz: f64,
    j: f64,
    ref_x: f64,
    ref_y: f64,
    ref_z: f64,
    load_qx: f64,
    load_qy: f64,
    load_qz: f64,
    point_loads: Vec<MemberPointLoad3D>,
}

impl Default for MemberEditBuffer {
    fn default() -> Self {
        Self {
            member_id: None,
            e: 200e9,
            g: 80e9,
            area: 1e-3,
            iy: 2e-6,
            iz: 1e-6,
            j: 3e-6,
            ref_x: 0.0,
            ref_y: 0.0,
            ref_z: 1.0,
            load_qx: 0.0,
            load_qy: 0.0,
            load_qz: 0.0,
            point_loads: Vec::new(),
        }
    }
}

/// Transient state for the double-click quick-edit popup.
///
/// Node edits use the `x/y/z` fields; member edits reuse the shared
/// [`MemberEditBuffer`] for the section numbers and carry the connectivity
/// text fields here.  Kept in memory only — never serialized.
struct QuickEdit {
    /// Entity being edited (always `Node` or `Member`).
    target: SelectionState,
    /// Screen position to anchor the popup near the cursor.
    anchor: egui::Pos2,
    /// Node coordinates (used for `Node` targets).
    x: f64,
    y: f64,
    z: f64,
    /// Member `i` / `j` node IDs as text (used for `Member` targets).
    node_i: String,
    node_j: String,
}

/// Transient state for the Boundary quick-add popups.
///
/// Each variant targets the **primary** selected entity (a node for Support /
/// Node Load, a member for Mem Load) and carries the values being edited plus
/// a one-line validation error to surface inside the popup.  Applying reuses
/// the shared [`ViewportApp`] edit paths (`commit_edit` / undo) — the popup
/// never mutates the model itself.  Kept in memory only — never serialized.
enum BoundaryPopup {
    /// Support: Fixed / Pinned / Custom (per-DOF) specification.
    Support {
        node: NodeId,
        anchor: egui::Pos2,
        spec: SupportSpec,
        error: Option<String>,
    },
    /// Nodal load: six global components (N, N·m).
    NodeLoad {
        node: NodeId,
        anchor: egui::Pos2,
        load: NodalLoadSpec,
        error: Option<String>,
    },
    /// Member load: uniform local qx/qy/qz plus an optional point load
    /// (position xi + local fx/fy/fz).
    MemLoad {
        member: MemberId,
        anchor: egui::Pos2,
        qx: f64,
        qy: f64,
        qz: f64,
        add_point: bool,
        xi: f64,
        fx: f64,
        fy: f64,
        fz: f64,
        error: Option<String>,
    },
}

impl BoundaryPopup {
    /// Screen anchor the popup was opened at.
    fn anchor(&self) -> egui::Pos2 {
        match self {
            BoundaryPopup::Support { anchor, .. }
            | BoundaryPopup::NodeLoad { anchor, .. }
            | BoundaryPopup::MemLoad { anchor, .. } => *anchor,
        }
    }

    /// Validation error to surface inside the popup, if any.
    fn error(&self) -> Option<&str> {
        match self {
            BoundaryPopup::Support { error, .. }
            | BoundaryPopup::NodeLoad { error, .. }
            | BoundaryPopup::MemLoad { error, .. } => error.as_deref(),
        }
    }

    /// Record a validation error; keeps the popup open.
    fn set_error(&mut self, msg: String) {
        match self {
            BoundaryPopup::Support { error, .. }
            | BoundaryPopup::NodeLoad { error, .. }
            | BoundaryPopup::MemLoad { error, .. } => *error = Some(msg),
        }
    }
}

impl eframe::App for ViewportApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ui::theme::apply_theme(ctx, self.dark_mode);

        // Keep the legacy single-entity projection in lock-step with the
        // authoritative multi-selection (see `ViewportApp::set_selection`).
        self.selection = self.selected.primary();

        // Capture the current window size (cheap) so it can be persisted.
        let size = ctx.input(|i| i.screen_rect().size());
        self.window_size = Some([size.x, size.y]);

        // Phase P7-3 C1: Ctrl+Shift+S captures the viewport.  Handled here
        // rather than in `handle_shortcuts` (which owns the undo / redo /
        // view keys) with the same no-text-focus guard, so the chord does
        // not fire while a DragValue / TextEdit has focus.
        if ctx.memory(|m| m.focused().is_none())
            && ctx.input(|i| i.modifiers.ctrl && i.modifiers.shift && i.key_pressed(egui::Key::S))
        {
            ui::screenshot::request(ctx);
        }

        // Phase 159-A: Ctrl+Shift+P toggles the command palette.  Same
        // no-text-focus guard as the screenshot chord (and no clash with
        // Ctrl+Shift+S); while the palette is open its search field holds
        // focus, so this cannot re-trigger — Esc closes the palette.
        if ctx.memory(|m| m.focused().is_none())
            && ctx.input(|i| i.modifiers.ctrl && i.modifiers.shift && i.key_pressed(egui::Key::P))
        {
            ui::palette::toggle(self);
        }

        self.handle_shortcuts(ctx);

        // ── Top: title bar + menu bar + module strip ────────────────────
        ui::titlebar::show_title_bar(self, ctx);
        ui::menu::show_menu_bar(self, ctx);
        ui::module::show_module_tabs(self, ctx);

        // ── Bottom: full-width status line (outside the main workspace) ──
        // Added *before* the side panels so it spans the whole window width;
        // the main workspace (left + centre + right) is laid out above it.
        ui::status::show_status_line(self, ctx);

        // ── Left: model browser tree ────────────────────────────────────
        ui::tree::show_model_tree(self, ctx);

        // ── Right: module-dependent inspector ───────────────────────────
        ui::inspector::show_inspector(self, ctx);

        // ── Modal dialogs ───────────────────────────────────────────────
        ui::dialogs::show_unsaved_dialog(self, ctx);

        // ── Centre: active module content + centre-scoped dock ───────────
        // Must be last: the root `CentralPanel` claims the remaining region
        // (the centre workspace), inside which the module's content and the
        // output dock are laid out with nested `show_inside` panels.
        ui::workspace::show_center_workspace(self, ctx);

        // ── Custom window chrome: resize grips (undecorated window) ─────
        // With the OS decorations off there are no native resize borders, so
        // the app lays its own thin grips along the window edges (suppressed
        // while maximized).  Drawn on a foreground layer so they sit above the
        // panels' outer pixels.
        ui::titlebar::show_resize_grips(ctx, self.dark_mode);

        // ── Double-click quick-edit popup (floats over the viewport) ────
        ui::viewport::show_quick_edit(self, ctx);

        // ── Boundary quick-add popup (floats over the viewport) ─────────
        ui::viewport::show_boundary_popup(self, ctx);

        // ── Command Palette (Phase 159-A; modal overlay, Ctrl+Shift+P) ───
        ui::palette::show(self, ctx);

        // ── Screenshot reply (Phase P7-3 C1) ────────────────────────────
        // `ViewportCommand::Screenshot` is answered by eframe *after* the
        // frame is painted: the composited window is read back (renderer-
        // agnostic — see `ui::screenshot`) and delivered as
        // `Event::Screenshot` on a later frame.  Drain every pending reply
        // and write each to disk, surfacing the full path in the dock.
        let shots: Vec<std::sync::Arc<egui::ColorImage>> = ctx.input(|i| {
            i.events
                .iter()
                .filter_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
                .collect()
        });
        for image in &shots {
            match ui::screenshot::save(image) {
                Ok(path) => {
                    self.messages.push(AppMessage::info(format!(
                        "Viewport screenshot saved to {}",
                        path.display()
                    )));
                    // Make sure the path is seen even when the dock sits on
                    // another tab (the screenshot note is informational).
                    self.bottom_tab = BottomTab::SolverLog;
                }
                Err(e) => self.messages.push(AppMessage::error(format!(
                    "Viewport screenshot failed: {e}"
                ))),
            }
        }

        // ── Persist settings ────────────────────────────────────────────
        // The theme is toggled directly by the menu module, so detect the
        // change here; also flush the latest window size on close.
        let closing = ctx.input(|i| i.viewport().close_requested());
        if closing || self.dark_mode != self.last_persisted_dark_mode {
            self.persist_settings();
        }

        // ── Persist the layout preset (Phase P7-3 C2) ───────────────────
        // The panels publish their live sizes while drawing (above), so the
        // capture is fresh here; persist whenever it changes, and on exit.
        let mut layout = ui::layout::capture(
            ctx,
            self.left_panel_collapsed,
            self.inspector_collapsed,
            self.bottom_dock_collapsed,
        );
        // Phase 159-D: also persist the right-panel mode (pure view state
        // carried on the same additive layout preset).
        layout.right_panel_mode = Some(self.right_panel_mode.index());
        if closing || Some(layout) != ui::layout::last_saved(ctx) {
            // Record the attempted value either way so a permanently failing
            // save does not retry (and re-warn) on every subsequent frame —
            // the same policy as `persist_settings`.
            ui::layout::set_last_saved(ctx, layout);
            if let Err(e) = persistence::save_layout(&self.settings_path, &layout) {
                self.messages
                    .push(AppMessage::warning(format!("Could not save layout: {e}")));
            }
        }
    }
}

// ── Entry point ─────────────────────────────────────────────────────────
fn main() -> eframe::Result {
    let settings_path = persistence::settings_path();
    let (settings, settings_warning) = persistence::load_settings_or_default(&settings_path);
    // Phase P7-3 C2: the layout preset shares the settings file under the
    // additive "layout" key; a file without it simply yields defaults.
    let (layout, layout_warning) = persistence::load_layout_or_default(&settings_path);
    let inner_size = settings
        .window
        .map(|w| {
            [
                w.width.max(MIN_INNER_SIZE[0]),
                w.height.max(MIN_INNER_SIZE[1]),
            ]
        })
        .unwrap_or([1200.0, 800.0]);

    // Zed-style custom window chrome: an undecorated (borderless) window whose
    // title bar and resize grips the app draws itself (`ui::titlebar`).  The
    // title string is unchanged — it is still what the taskbar / OS show — and
    // the persisted window size still drives the initial inner size.
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(inner_size)
            .with_min_inner_size(MIN_INNER_SIZE)
            .with_decorations(false)
            .with_title(WINDOW_TITLE),
        ..Default::default()
    };
    eframe::run_native(
        "structural-analysis-gui",
        options,
        Box::new(move |cc| {
            ui::theme::apply_theme(&cc.egui_ctx, settings.dark_mode);
            // Change 1: round the borderless window's corners natively via DWM
            // (Windows 11).  The window exists by the time the app creator runs,
            // so its handle is looked up here; a failure is only cosmetic and is
            // reported in the Messages dock rather than being swallowed.
            let corners = ui::titlebar::apply_rounded_corners(WINDOW_TITLE);
            // Phase P1: try to bring up the GPU (glow) viewport renderer.  Any
            // failure leaves the software renderer active and is reported in
            // the Messages dock.
            let gl_status = ui::gl_renderer::init(cc.gl.clone());
            let mut app = ViewportApp::new(settings, settings_path, settings_warning);
            if let Err(e) = corners {
                app.messages.push(AppMessage::warning(format!(
                    "Rounded window corners unavailable ({e}); using square corners."
                )));
            }
            // Phase P7-3 C2: restore the persisted layout preset.  The panel
            // widths are installed into egui's data map for the panels to
            // read back; the collapsed flags apply to the app directly.  The
            // theme stays owned by `AppSettings::dark_mode` (already applied
            // above) so there is a single source of truth.
            app.inspector_collapsed = layout.inspector_collapsed;
            app.bottom_dock_collapsed = layout.dock_collapsed;
            app.left_panel_collapsed = layout.tree_collapsed;
            // Phase 159-D: the right panel reopens on the mode the user left
            // it on (an additive layout field; absent → default).  The centre
            // is driven by the work module, which is not persisted.
            app.right_panel_mode = RightPanelMode::from_index(layout.right_panel_mode);
            if let Some(warning) = layout_warning {
                app.messages
                    .push(AppMessage::warning(format!("Settings: {warning}")));
            }
            ui::layout::install_restored(&cc.egui_ctx, layout);
            match gl_status {
                Ok(()) => app.messages.push(AppMessage::info(
                    "GPU viewport renderer ready (depth-tested 3-D view).",
                )),
                Err(e) => app.messages.push(AppMessage::warning(format!(
                    "GPU renderer unavailable ({e}); using the Software renderer."
                ))),
            }
            Ok(Box::new(app))
        }),
    )
}
