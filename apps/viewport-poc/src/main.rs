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
    edit_model::{MemberId, MemberPointLoad3D},
    model, persistence,
    scene::ViewportScene,
    selection::{SelectionSet, SelectionState},
};

use ui::status::BottomTab;
use ui::WorkModule;

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
    saved_fingerprint: Option<u64>,
    file_path_input: String,
    show_unsaved_dialog: bool,
    pending_action: PendingAction,
    history: EditHistory,
    /// Active MIDAS-style work module.
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
    /// Screen position where a rubber-band box-select drag began
    /// (Shift/Ctrl + primary drag), or `None` when no box select is active.
    box_select_start: Option<egui::Pos2>,
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

impl eframe::App for ViewportApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ui::theme::apply_theme(ctx, self.dark_mode);

        // Keep the legacy single-entity projection in lock-step with the
        // authoritative multi-selection (see `ViewportApp::set_selection`).
        self.selection = self.selected.primary();

        // Capture the current window size (cheap) so it can be persisted.
        let size = ctx.input(|i| i.screen_rect().size());
        self.window_size = Some([size.x, size.y]);

        self.handle_shortcuts(ctx);

        // ── Top: menu bar + module tabs ─────────────────────────────────
        ui::menu::show_menu_bar(self, ctx);
        ui::module::show_module_tabs(self, ctx);

        // ── Bottom: messages / results dock ─────────────────────────────
        ui::status::show_bottom_dock(self, ctx);

        // ── Left: model browser tree ────────────────────────────────────
        ui::tree::show_model_tree(self, ctx);

        // ── Right: module-dependent inspector ───────────────────────────
        ui::inspector::show_inspector(self, ctx);

        // ── Modal dialogs ───────────────────────────────────────────────
        ui::dialogs::show_unsaved_dialog(self, ctx);

        // ── Center: 3D viewport (must be last) ──────────────────────────
        ui::viewport::show_viewport(self, ctx);

        // ── Double-click quick-edit popup (floats over the viewport) ────
        ui::viewport::show_quick_edit(self, ctx);

        // ── Persist settings ────────────────────────────────────────────
        // The theme is toggled directly by the menu module, so detect the
        // change here; also flush the latest window size on close.
        let closing = ctx.input(|i| i.viewport().close_requested());
        if closing || self.dark_mode != self.last_persisted_dark_mode {
            self.persist_settings();
        }
    }
}

// ── Entry point ─────────────────────────────────────────────────────────
fn main() -> eframe::Result {
    let settings_path = persistence::settings_path();
    let (settings, settings_warning) = persistence::load_settings_or_default(&settings_path);
    let inner_size = settings
        .window
        .map(|w| [w.width.max(320.0), w.height.max(240.0)])
        .unwrap_or([1200.0, 800.0]);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(inner_size)
            .with_title("Section Properties & Frame Analysis"),
        ..Default::default()
    };
    eframe::run_native(
        "structural-analysis-gui",
        options,
        Box::new(move |cc| {
            ui::theme::apply_theme(&cc.egui_ctx, settings.dark_mode);
            // Phase P1: try to bring up the GPU (glow) viewport renderer.  Any
            // failure leaves the software renderer active and is reported in
            // the Messages dock.
            let gl_status = ui::gl_renderer::init(cc.gl.clone());
            let mut app = ViewportApp::new(settings, settings_path, settings_warning);
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
