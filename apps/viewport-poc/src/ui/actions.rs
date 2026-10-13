//! Application command logic on [`ViewportApp`](crate::ViewportApp).
//!
//! These are the non-rendering operations (load/save, undo/redo, solve,
//! selection validation, the member-edit buffer, dirty-guarded requests)
//! shared by the menu bar, tree, inspector and dialogs.  They live in a
//! child module of the crate root so they can reach the private state
//! fields declared in `main.rs`.

use egui;

use structural_analysis::FrameAnalysisResult3D;
use viewport_poc::app_state::{AppMessage, Project};
use viewport_poc::camera::{Camera3D, StandardView};
use viewport_poc::display::ResultDisplayState;
use viewport_poc::edit_history::EditHistory;
use viewport_poc::edit_model::{EditModel, MemberId, MemberUniformLoad3D, NodeId};
use viewport_poc::scene::ViewportScene;
use viewport_poc::selection::SelectionState;
use viewport_poc::{model, persistence, project};

use super::module::WorkModule;
use super::status::BottomTab;
use crate::{MemberEditBuffer, PendingAction, ViewportApp};

impl ViewportApp {
    /// Build the application, threading in settings loaded from disk (and an
    /// optional warning if that file was present but unreadable/corrupt).
    pub(crate) fn new(
        settings: persistence::AppSettings,
        settings_path: std::path::PathBuf,
        settings_warning: Option<String>,
    ) -> Self {
        let mut messages = vec![AppMessage::info(
            "Application started. Select a model from the browser.",
        )];
        if let Some(warning) = settings_warning {
            messages.push(AppMessage::warning(format!("Settings: {warning}")));
        }
        Self {
            project: Project::empty(),
            scene: None,
            camera: Camera3D::default(),
            selection: SelectionState::None,
            display: ResultDisplayState::default(),
            messages,
            new_node_x: String::new(),
            new_node_y: String::new(),
            new_node_z: String::new(),
            new_member_i: String::new(),
            new_member_j: String::new(),
            member_edit: MemberEditBuffer::default(),
            project_path: None,
            saved_fingerprint: None,
            file_path_input: String::new(),
            show_unsaved_dialog: false,
            pending_action: PendingAction::None,
            history: EditHistory::default(),
            module: WorkModule::Model,
            bottom_tab: BottomTab::Messages,
            bottom_dock_collapsed: false,
            inspector_collapsed: false,
            section_editor: super::section::SectionEditorState::default(),
            dark_mode: settings.dark_mode,
            settings_path,
            last_persisted_dark_mode: settings.dark_mode,
            window_size: settings.window.map(|w| [w.width, w.height]),
        }
    }

    /// Persist the current app settings (theme + window size) to disk.
    ///
    /// Called when the theme toggles and when the window is closing.  A failed
    /// write is surfaced in the message dock rather than silently swallowed.
    pub(crate) fn persist_settings(&mut self) {
        let settings = persistence::AppSettings {
            dark_mode: self.dark_mode,
            window: self
                .window_size
                .map(|[width, height]| persistence::WindowPrefs { width, height }),
        };
        if let Err(e) = persistence::save_settings(&self.settings_path, &settings) {
            self.messages
                .push(AppMessage::warning(format!("Could not save settings: {e}")));
        }
        // Record the attempted value either way so a permanently failing save
        // does not retry on every subsequent frame.
        self.last_persisted_dark_mode = self.dark_mode;
    }

    /// Borrow the current analysis result, or `None` if stale/failed/none.
    ///
    /// Delegates to [`Project::analysis_result`] (Policy A).
    pub(crate) fn analysis_result(&self) -> Option<&FrameAnalysisResult3D> {
        self.project.analysis_result()
    }

    /// Whether the analysis may be run.
    ///
    /// Single source of truth for the enabled state of every run control —
    /// the inspector's "Run Analysis" button, the Analysis menu's "Solve"
    /// entry and the quick-action "Solve" button.  A model must be loaded
    /// *and* contain at least one node: an empty model has nothing to solve,
    /// and [`Project::solve`] on it would only fail.
    pub(crate) fn can_run_analysis(&self) -> bool {
        self.project
            .model
            .as_ref()
            .is_some_and(|m| m.n_nodes() > 0)
    }

    /// Commit a model transition as the result of an engineering edit.
    ///
    /// Records the previous model in the undo history, replaces the
    /// current model, clears the redo stack, invalidates analysis, and
    /// rebuilds the viewport scene.  The caller must ensure the new model
    /// differs from the old one (no-op edits should not call this).
    pub(crate) fn commit_edit(&mut self, new_model: EditModel) {
        if let Some(old_model) = self.project.model.take() {
            self.history.record(old_model);
        }
        self.project.replace_model(new_model);
        self.rebuild_scene();
    }

    /// Undo the most recent edit.
    pub(crate) fn do_undo(&mut self) {
        let Some(current) = self.project.model.clone() else {
            return;
        };
        let Some(mut restored) = self.history.undo(current.clone()) else {
            return;
        };
        let current_rev = current.revision();
        restored.ensure_revision_at_least(current_rev + 1);
        self.project.replace_model(restored);
        self.rebuild_scene();
        self.validate_selection();
        self.messages.push(AppMessage::info("Undo"));
    }

    /// Redo the most recently undone edit.
    pub(crate) fn do_redo(&mut self) {
        let Some(current) = self.project.model.clone() else {
            return;
        };
        let Some(mut restored) = self.history.redo(current.clone()) else {
            return;
        };
        let current_rev = current.revision();
        restored.ensure_revision_at_least(current_rev + 1);
        self.project.replace_model(restored);
        self.rebuild_scene();
        self.validate_selection();
        self.messages.push(AppMessage::info("Redo"));
    }

    /// Load a preset model and immediately solve it.
    pub(crate) fn load_preset(&mut self, preset: model::PresetModel) {
        let name = preset.name().to_string();
        let m = preset.build_edit();
        self.project.load_model(name, m);
        self.messages
            .push(AppMessage::info(format!("Loaded '{}'", preset.name())));
        self.project_path = None;
        self.saved_fingerprint = None;
        self.history.clear();
        self.rebuild_scene();
        self.run_solve();
    }

    /// Rebuild the viewport scene from the current edit model.
    pub(crate) fn rebuild_scene(&mut self) {
        if let Some(ref model) = self.project.model {
            self.scene = Some(ViewportScene::from_edit_model(model));
            if let Some(ref s) = self.scene {
                let (min, max) = s.bounding_box();
                self.camera = Camera3D::fit(min, max);
            }
        } else {
            self.scene = None;
        }
    }

    /// Fit / zoom-extents: aim the camera at the scene bounding box.
    ///
    /// No-op when no model is loaded.
    pub(crate) fn fit_view(&mut self) {
        if let Some(ref scene) = self.scene {
            let (min, max) = scene.bounding_box();
            self.camera.frame(min, max);
        }
    }

    /// Apply a standard preset camera orientation (keeps target / distance).
    pub(crate) fn set_standard_view(&mut self, view: StandardView) {
        self.camera.set_view(view);
    }

    /// Sync the member edit buffer when the selected member changes.
    pub(crate) fn sync_member_edit_buffer(&mut self, id: MemberId) {
        if self.member_edit.member_id == Some(id) {
            return;
        }
        self.reset_member_edit(id);
    }

    /// Reset the edit buffer to the current model values for a member.
    pub(crate) fn reset_member_edit(&mut self, id: MemberId) {
        if let Some(model) = &self.project.model {
            if let Some(member) = model.members().iter().find(|m| m.id == id) {
                let sec = &member.section;
                let load = model.member_uniform_load(id).copied().unwrap_or_default();
                let point_loads = model.member_point_loads_for(id);
                self.member_edit = MemberEditBuffer {
                    member_id: Some(id),
                    e: sec.E(),
                    g: sec.G(),
                    area: sec.area(),
                    iy: sec.iy(),
                    iz: sec.iz(),
                    j: sec.j(),
                    ref_x: member.ref_vec[0],
                    ref_y: member.ref_vec[1],
                    ref_z: member.ref_vec[2],
                    load_qx: load.qx,
                    load_qy: load.qy,
                    load_qz: load.qz,
                    point_loads,
                };
            }
        }
    }

    /// Apply the edit buffer to the model for a member.
    pub(crate) fn apply_member_edit(&mut self, id: MemberId) {
        let buf = &self.member_edit;
        let (e, g, area, iy, iz, j, rv, load, point_loads) = (
            buf.e,
            buf.g,
            buf.area,
            buf.iy,
            buf.iz,
            buf.j,
            [buf.ref_x, buf.ref_y, buf.ref_z],
            MemberUniformLoad3D::new(buf.load_qx, buf.load_qy, buf.load_qz),
            buf.point_loads.clone(),
        );
        let Some(model) = self.project.model.as_ref() else {
            return;
        };
        let mut candidate = model.clone();
        match candidate.set_member_section(id, e, g, area, iy, iz, j) {
            Ok(()) => {}
            Err(e) => {
                self.messages.push(AppMessage::error(e.message()));
                return;
            }
        }
        match candidate.set_member_ref_vec(id, rv) {
            Ok(()) => {}
            Err(e) => {
                self.messages.push(AppMessage::error(e.message()));
                self.reset_member_edit(id);
                return;
            }
        }
        let load_result = if load.is_zero() {
            candidate.clear_member_uniform_load(id)
        } else {
            candidate.set_member_uniform_load(id, load)
        };
        match load_result {
            Ok(()) => {}
            Err(e) => {
                self.messages.push(AppMessage::error(e.message()));
                self.reset_member_edit(id);
                return;
            }
        }

        let existing_point_loads = candidate.member_point_loads_for(id);
        if existing_point_loads != point_loads {
            candidate.clear_member_point_loads(id).ok();
            for pl in &point_loads {
                if let Err(e) = candidate.add_member_point_load(id, *pl) {
                    self.messages.push(AppMessage::error(e.message()));
                    self.reset_member_edit(id);
                    return;
                }
            }
        }

        if candidate.revision() != model.revision() {
            self.commit_edit(candidate);
            self.messages
                .push(AppMessage::info(format!("Applied changes to M{}", id.0)));
        } else {
            self.messages.push(AppMessage::info("No changes to apply."));
        }
    }

    /// Run the FEM analysis and rebuild the viewport scene.
    ///
    /// This is the **only** place the scene is rebuilt from analysis data.
    /// Camera, selection, and display-option changes never call this.
    pub(crate) fn run_solve(&mut self) {
        match self.project.solve() {
            Ok(()) => {
                self.messages.push(AppMessage::info("Analysis complete."));
            }
            Err(e) => {
                self.messages
                    .push(AppMessage::error(format!("Analysis failed: {e}")));
            }
        }
    }

    /// Clear the project to the empty state.
    pub(crate) fn clear_project(&mut self) {
        self.project.clear();
        self.scene = None;
        self.selection = SelectionState::None;
        self.project_path = None;
        self.saved_fingerprint = None;
        self.history.clear();
        self.messages.push(AppMessage::info("Project cleared."));
    }

    /// `true` if the current model has unsaved changes.
    ///
    /// Dirty state is based on a canonical fingerprint of the engineering
    /// data (nodes, members, supports, loads, ID allocation), NOT on the
    /// runtime revision counter.  This ensures that undoing back to the
    /// saved state correctly reports `dirty = false`.
    pub(crate) fn is_dirty(&self) -> bool {
        self.project
            .model
            .as_ref()
            .map_or(false, |m| self.saved_fingerprint != Some(m.fingerprint()))
    }

    /// Create a new empty model.
    pub(crate) fn do_new(&mut self) {
        self.project.load_model("Untitled".into(), EditModel::new());
        self.rebuild_scene();
        self.selection = SelectionState::None;
        self.project_path = None;
        self.saved_fingerprint = None;
        self.history.clear();
        self.messages.push(AppMessage::info("Created empty model."));
    }

    /// Open a project from the file path input.
    ///
    /// On failure, the current model/path/dirty state are preserved.
    pub(crate) fn do_open(&mut self) {
        let path = std::path::PathBuf::from(self.file_path_input.trim());
        if path.as_os_str().is_empty() {
            self.messages
                .push(AppMessage::error("No file path specified."));
            return;
        }
        match persistence::load_project_with_section(&path) {
            Ok(loaded) => {
                let name = path
                    .file_stem()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_else(|| "Untitled".to_string());
                let fp = loaded.model.fingerprint();
                self.project.load_model(name, loaded.model);
                // Restore the Section module's definition, if the file has one
                // (files written before this feature simply won't).
                if let Some(section) = loaded.section {
                    self.section_editor.set_def(section);
                }
                self.rebuild_scene();
                self.selection = SelectionState::None;
                self.project_path = Some(path);
                self.saved_fingerprint = Some(fp);
                self.history.clear();
                self.messages
                    .push(AppMessage::info("Project loaded successfully."));
            }
            Err(e) => {
                self.messages
                    .push(AppMessage::error(format!("Load failed: {e}")));
            }
        }
    }

    /// Save the project to the current path (or the file path input if no
    /// current path is set).  On failure, `saved_revision` is not updated.
    /// Returns `true` on success.
    pub(crate) fn do_save(&mut self) -> bool {
        let path = match &self.project_path {
            Some(p) => p.clone(),
            None => {
                let p = std::path::PathBuf::from(self.file_path_input.trim());
                if p.as_os_str().is_empty() {
                    self.messages.push(AppMessage::error(
                        "No file path. Use \u{201c}Save As\u{201d} to specify one.",
                    ));
                    return false;
                }
                p
            }
        };
        self.save_to_path(&path)
    }

    /// Save the project to the path in the file path input.
    /// Returns `true` on success.
    pub(crate) fn do_save_as(&mut self) -> bool {
        let path = std::path::PathBuf::from(self.file_path_input.trim());
        if path.as_os_str().is_empty() {
            self.messages
                .push(AppMessage::error("No file path specified."));
            return false;
        }
        self.save_to_path(&path)
    }

    /// Internal: save the model to `path` and update state on success.
    /// Returns `true` on success.
    pub(crate) fn save_to_path(&mut self, path: &std::path::Path) -> bool {
        let Some(model) = self.project.model.as_ref() else {
            self.messages.push(AppMessage::error("No model to save."));
            return false;
        };
        let fp = model.fingerprint();
        // Capture the Section module's current definition alongside the model.
        let section = self.section_editor.def.clone();
        let result = persistence::save_project_with_section(path, model, Some(&section));
        match result {
            Ok(()) => {
                self.project_path = Some(path.to_path_buf());
                self.saved_fingerprint = Some(fp);
                self.file_path_input = path.to_string_lossy().to_string();
                self.messages
                    .push(AppMessage::info(format!("Saved to {}", path.display())));
                true
            }
            Err(e) => {
                self.messages
                    .push(AppMessage::error(format!("Save failed: {e}")));
                false
            }
        }
    }

    /// Execute the pending action after the unsaved-changes dialog.
    pub(crate) fn execute_pending(&mut self) {
        match self.pending_action {
            PendingAction::None => {}
            PendingAction::New => self.do_new(),
            PendingAction::Open => self.do_open(),
            PendingAction::Clear => self.clear_project(),
            PendingAction::LoadPreset(preset) => self.load_preset(preset),
        }
        self.pending_action = PendingAction::None;
        self.show_unsaved_dialog = false;
    }

    /// Clear selection if it points to a node/member that no longer exists.
    pub(crate) fn validate_selection(&mut self) {
        if let Some(ref scene) = self.scene {
            match self.selection {
                SelectionState::Node(id) if scene.node_index(id).is_none() => {
                    self.selection = SelectionState::None;
                }
                SelectionState::Member(id) if scene.member_index(id).is_none() => {
                    self.selection = SelectionState::None;
                }
                _ => {}
            }
        } else {
            self.selection = SelectionState::None;
        }
    }

    /// Hit-test the mouse position against nodes and members.
    pub(crate) fn pick(&self, mouse: [f64; 2], screen: [f64; 2]) -> SelectionState {
        const THRESH: f64 = 12.0;
        let Some(scene) = &self.scene else {
            return SelectionState::None;
        };
        let w = screen[0];
        let h = screen[1];

        let mut best_node: Option<(usize, f64)> = None;
        for (i, &pos) in scene.node_positions.iter().enumerate() {
            if let Some(p) = self.camera.project(pos, w, h) {
                let d = (p.x - mouse[0]).hypot(p.y - mouse[1]);
                if d < THRESH && best_node.map_or(true, |(_, bd)| d < bd) {
                    best_node = Some((i, d));
                }
            }
        }
        let mut best_mem: Option<(usize, f64)> = None;
        for (i, &(ni, nj)) in scene.member_nodes.iter().enumerate() {
            let Some(pi) = scene.node_positions.get(ni) else {
                continue;
            };
            let Some(pj) = scene.node_positions.get(nj) else {
                continue;
            };
            let Some(ps_i) = self.camera.project(*pi, w, h) else {
                continue;
            };
            let Some(ps_j) = self.camera.project(*pj, w, h) else {
                continue;
            };
            let d = project::point_to_segment_dist_2d(mouse, [ps_i.x, ps_i.y], [ps_j.x, ps_j.y]);
            if d < THRESH && best_mem.map_or(true, |(_, bd)| d < bd) {
                best_mem = Some((i, d));
            }
        }
        match (best_node, best_mem) {
            (Some((ni, nd)), Some((mi, md))) => {
                if nd < md {
                    SelectionState::Node(scene.node_ids[ni])
                } else {
                    SelectionState::Member(scene.member_ids[mi])
                }
            }
            (Some((ni, _)), None) => SelectionState::Node(scene.node_ids[ni]),
            (None, Some((mi, _))) => SelectionState::Member(scene.member_ids[mi]),
            (None, None) => SelectionState::None,
        }
    }

    /// Add a node from the text inputs, with validation and feedback.
    pub(crate) fn add_node_from_input(&mut self) {
        let x: f64 = match self.new_node_x.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid x coordinate"));
                return;
            }
        };
        let y: f64 = match self.new_node_y.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid y coordinate"));
                return;
            }
        };
        let z: f64 = match self.new_node_z.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages
                    .push(AppMessage::error("Invalid z coordinate"));
                return;
            }
        };
        let Some(model) = self.project.model.as_ref() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return;
        };
        let mut candidate = model.clone();
        match candidate.add_node(x, y, z) {
            Ok(id) => {
                self.commit_edit(candidate);
                self.messages.push(AppMessage::info(format!(
                    "Added node N{} at ({}, {}, {})",
                    id.0, x, y, z
                )));
                self.new_node_x.clear();
                self.new_node_y.clear();
                self.new_node_z.clear();
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
    }

    /// Add a member from the text inputs, with validation and feedback.
    pub(crate) fn add_member_from_input(&mut self) {
        let ni: u32 = match self.new_member_i.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages.push(AppMessage::error("Invalid node i ID"));
                return;
            }
        };
        let nj: u32 = match self.new_member_j.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.messages.push(AppMessage::error("Invalid node j ID"));
                return;
            }
        };
        let Some(model) = self.project.model.as_ref() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return;
        };
        let mut candidate = model.clone();
        match candidate.add_member(NodeId(ni), NodeId(nj)) {
            Ok(id) => {
                self.commit_edit(candidate);
                self.messages.push(AppMessage::info(format!(
                    "Added member M{} (N{} \u{2192} N{})",
                    id.0, ni, nj
                )));
                self.new_member_i.clear();
                self.new_member_j.clear();
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
    }

    // ── Dirty-guarded requests (shared by the File menu) ────────────────

    /// Request a new model, guarding unsaved changes.
    pub(crate) fn request_new(&mut self) {
        if self.is_dirty() {
            self.show_unsaved_dialog = true;
            self.pending_action = PendingAction::New;
        } else {
            self.do_new();
        }
    }

    /// Request opening a project, guarding unsaved changes.
    pub(crate) fn request_open(&mut self) {
        if self.is_dirty() {
            self.show_unsaved_dialog = true;
            self.pending_action = PendingAction::Open;
        } else {
            self.do_open();
        }
    }

    /// Request clearing the project, guarding unsaved changes.
    pub(crate) fn request_clear(&mut self) {
        if self.is_dirty() {
            self.show_unsaved_dialog = true;
            self.pending_action = PendingAction::Clear;
        } else {
            self.clear_project();
        }
    }

    /// Request loading a preset, guarding unsaved changes.
    pub(crate) fn request_load_preset(&mut self, preset: model::PresetModel) {
        if self.is_dirty() {
            self.show_unsaved_dialog = true;
            self.pending_action = PendingAction::LoadPreset(preset);
        } else {
            self.load_preset(preset);
        }
    }

    /// Process keyboard shortcuts (undo/redo).
    pub(crate) fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        // Only process when no widget has focus, to avoid interfering
        // with DragValue / TextEdit keyboard handling (e.g. Ctrl+Z in
        // a text field should do text undo, not model undo).
        let no_text_focus = ctx.memory(|m| m.focused().is_none());
        if no_text_focus {
            if ctx.input(|i| i.key_pressed(egui::Key::Z) && i.modifiers.ctrl && !i.modifiers.shift)
            {
                self.do_undo();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Y) && i.modifiers.ctrl) {
                self.do_redo();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Z) && i.modifiers.ctrl && i.modifiers.shift) {
                self.do_redo();
            }
            // Ctrl+M toggles the bottom messages / results dock.
            if ctx.input(|i| i.key_pressed(egui::Key::M) && i.modifiers.ctrl) {
                self.bottom_dock_collapsed = !self.bottom_dock_collapsed;
            }
            // Ctrl+I toggles the right-hand inspector panel.
            if ctx.input(|i| i.key_pressed(egui::Key::I) && i.modifiers.ctrl) {
                self.inspector_collapsed = !self.inspector_collapsed;
            }
            // View shortcuts: F = fit, 1..4 = standard views.
            if ctx.input(|i| i.key_pressed(egui::Key::F)) {
                self.fit_view();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Num1)) {
                self.set_standard_view(StandardView::Iso);
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Num2)) {
                self.set_standard_view(StandardView::Front);
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Num3)) {
                self.set_standard_view(StandardView::Top);
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Num4)) {
                self.set_standard_view(StandardView::Right);
            }
        }
    }
}
