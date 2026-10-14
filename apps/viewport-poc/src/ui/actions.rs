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
use viewport_poc::display::{CutAxis, ResultDisplayState, ResultViewMode};
use viewport_poc::edit_history::EditHistory;
use viewport_poc::edit_model::{
    EditModel, MemberId, MemberPointLoad3D, MemberUniformLoad3D, NodalLoadSpec, NodeId, SupportSpec,
};
use viewport_poc::scene::ViewportScene;
use viewport_poc::selection::{ScreenRect, SelectionSet, SelectionState};
use viewport_poc::{model, persistence, project};

use super::module::WorkModule;
use super::status::BottomTab;
use crate::{BoundaryPopup, MemberEditBuffer, PendingAction, QuickEdit, ViewportApp};

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
            selected: SelectionSet::default(),
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
            ctx_menu_target: SelectionState::None,
            quick_edit: None,
            boundary_popup: None,
            box_select_start: None,
            view_mode: viewport_poc::camera::ViewMode::default(),
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
    ///
    /// Uses [`Project::replace_model_after_edit`] so a model that *was*
    /// solved is reported as `Stale` after the edit (its result is kept but
    /// hidden by Policy A), rather than falling back to `NeverSolved`.
    /// Undo/redo use the wholesale [`Project::replace_model`], which discards
    /// the result.
    pub(crate) fn commit_edit(&mut self, new_model: EditModel) {
        if let Some(old_model) = self.project.model.take() {
            self.history.record(old_model);
        }
        self.project.replace_model_after_edit(new_model);
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
            // Keep the section-cut offset valid for the new model bounds.
            self.sync_section_cut();
        } else {
            self.scene = None;
        }
    }

    // ── Phase P5: section cut ───────────────────────────────────────────

    /// The projection range `[min, max]` the section-cut plane can sweep, or
    /// `None` when there is no scene / a degenerate normal.
    pub(crate) fn cut_range(&self) -> Option<(f64, f64)> {
        let scene = self.scene.as_ref()?;
        let n = self.display.section_cut.unit_normal()?;
        Some(scene.projection_range(n))
    }

    /// Re-centre (first time) or clamp the section-cut offset against the
    /// current scene bounds so it works at any model scale.  A no-op with no
    /// scene.
    pub(crate) fn sync_section_cut(&mut self) {
        if let Some((lo, hi)) = self.cut_range() {
            self.display.section_cut.sync_bounds(lo, hi);
        }
    }

    /// Switch the section-cut axis (re-centres the offset on the scene).
    pub(crate) fn set_cut_axis(&mut self, axis: CutAxis) {
        let cut = &mut self.display.section_cut;
        cut.axis = axis;
        cut.normal = None;
        cut.recenter();
        self.sync_section_cut();
    }

    /// Enable / disable the section cut, positioning it on first enable.
    pub(crate) fn set_cut_enabled(&mut self, on: bool) {
        self.display.section_cut.enabled = on;
        if on {
            self.sync_section_cut();
        }
    }

    /// Re-centre the section-cut offset on the scene (`Center on model`).
    pub(crate) fn recenter_section_cut(&mut self) {
        self.display.section_cut.recenter();
        self.sync_section_cut();
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

    // ── Phase P4: viewport interaction commands ────────────────────────

    /// Zoom the camera onto a node (keep orientation, move the target).
    pub(crate) fn zoom_to_node(&mut self, id: NodeId) {
        let Some(scene) = &self.scene else {
            return;
        };
        let Some(idx) = scene.node_index(id) else {
            return;
        };
        let p = scene.node_positions[idx];
        self.camera.target = p;
        self.camera.distance = (self.scene_diag() * 0.35).clamp(0.2, 1e5);
    }

    /// Zoom the camera onto a member (frame its two ends).
    pub(crate) fn zoom_to_member(&mut self, id: MemberId) {
        let Some(scene) = &self.scene else {
            return;
        };
        let Some(idx) = scene.member_index(id) else {
            return;
        };
        let Some(mid) = scene.member_midpoint(idx) else {
            return;
        };
        let len = scene.member_length(idx).unwrap_or(1.0);
        self.camera.target = mid;
        self.camera.distance = (len * 2.0).clamp(0.2, 1e5);
    }

    /// Bounding-box diagonal of the scene (≥ 1.0), for framing zooms.
    fn scene_diag(&self) -> f64 {
        let Some(scene) = &self.scene else {
            return 1.0;
        };
        let (min, max) = scene.bounding_box();
        (max[0] - min[0])
            .hypot(max[1] - min[1])
            .hypot(max[2] - min[2])
            .max(1.0)
    }

    // ── Phase P6: multi-selection ──────────────────────────────────────

    /// Replace the whole selection with `set`, keeping the single-entity
    /// projection ([`selection`](crate::ViewportApp::selection)) in lock-step.
    ///
    /// This is the **only** place the two representations are written, so the
    /// authoritative [`selected`](crate::ViewportApp::selected) set and its
    /// primary projection can never drift apart.
    pub(crate) fn set_selection(&mut self, set: SelectionSet) {
        self.selected = set;
        self.selection = self.selected.primary();
    }

    /// Replace the selection with a single entity (plain click / tree click).
    pub(crate) fn select_only(&mut self, state: SelectionState) {
        self.set_selection(SelectionSet::from_single(state));
    }

    /// Toggle one entity in/out of the selection (Shift+click).  `None` is a
    /// no-op.
    pub(crate) fn select_toggle(&mut self, state: SelectionState) {
        if state == SelectionState::None {
            return;
        }
        let mut set = self.selected.clone();
        set.toggle_state(state);
        self.set_selection(set);
    }

    /// Add one entity to the selection (Ctrl+click).  `None` is a no-op.
    pub(crate) fn select_add(&mut self, state: SelectionState) {
        if state == SelectionState::None {
            return;
        }
        let mut set = self.selected.clone();
        set.add_state(state);
        self.set_selection(set);
    }

    /// Clear the selection.
    pub(crate) fn clear_selection(&mut self) {
        self.set_selection(SelectionSet::default());
    }

    /// Select every currently **visible** node and member (Ctrl+A).  Hidden
    /// entities (and, implicitly, ones removed by the section cut's markers)
    /// are skipped, matching the box-select rule.
    pub(crate) fn select_all_visible(&mut self) {
        let (nodes, members) = {
            let Some(scene) = self.scene.as_ref() else {
                self.clear_selection();
                return;
            };
            let nodes: Vec<NodeId> = scene
                .node_ids
                .iter()
                .copied()
                .filter(|&id| !self.display.is_node_hidden(id))
                .collect();
            let members: Vec<MemberId> = scene
                .member_ids
                .iter()
                .copied()
                .filter(|&id| !self.display.is_member_hidden(id))
                .collect();
            (nodes, members)
        };
        let mut set = SelectionSet::default();
        for id in nodes {
            set.add_node(id);
        }
        for id in members {
            set.add_member(id);
        }
        self.set_selection(set);
    }

    /// Rubber-band box select (Shift/Ctrl + primary drag).
    ///
    /// `box` is in **viewport-local points** (origin at the viewport's top-left,
    /// matching [`Camera3D::project`]); `screen` is `[w, h]`.  Every visible
    /// node whose projection falls inside the box is selected, and every
    /// visible member whose **both** projected endpoints fall inside it.  When
    /// `add` is `true` the hits are added to the current selection, otherwise
    /// they replace it.
    pub(crate) fn box_select(&mut self, box_: ScreenRect, screen: [f64; 2], add: bool) {
        let (w, h) = (screen[0], screen[1]);
        let mut set = if add {
            self.selected.clone()
        } else {
            SelectionSet::default()
        };
        {
            let Some(scene) = self.scene.as_ref() else {
                self.set_selection(set);
                return;
            };
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                let Some(&id) = scene.node_ids.get(i) else {
                    continue;
                };
                if self.display.is_node_hidden(id) || !self.display.section_cut.keeps(pos) {
                    continue;
                }
                if let Some(p) = self.camera.project(pos, w, h) {
                    if box_.hits_point([p.x, p.y]) {
                        set.add_node(id);
                    }
                }
            }
            for mi in 0..scene.n_members() {
                let Some(&id) = scene.member_ids.get(mi) else {
                    continue;
                };
                if self.display.is_member_hidden(id) {
                    continue;
                }
                let Some(&(ni, nj)) = scene.member_nodes.get(mi) else {
                    continue;
                };
                let (Some(&pi), Some(&pj)) = (
                    scene.node_positions.get(ni),
                    scene.node_positions.get(nj),
                ) else {
                    continue;
                };
                let Some((ca, cb)) = self.display.section_cut.clip_segment(pi, pj) else {
                    continue;
                };
                if let (Some(pa), Some(pb)) = (
                    self.camera.project(ca, w, h),
                    self.camera.project(cb, w, h),
                ) {
                    if box_.hits_segment([pa.x, pa.y], [pb.x, pb.y]) {
                        set.add_member(id);
                    }
                }
            }
        }
        self.set_selection(set);
    }

    /// Select an entity and switch to the module whose inspector edits it.
    ///
    /// Reuses the inspector's existing edit paths — this never duplicates the
    /// support / load / section mechanics, it only routes the user there.
    pub(crate) fn select_and_open(&mut self, sel: SelectionState, module: WorkModule) {
        self.select_only(sel);
        self.module = module;
    }

    /// Select a member and switch the result view to its end forces.
    pub(crate) fn show_member_end_forces(&mut self, id: MemberId) {
        self.select_only(SelectionState::Member(id));
        self.display.mode = ResultViewMode::MemberEndForces;
    }

    /// Hide one node (view-only; never touches the model).
    pub(crate) fn hide_node(&mut self, id: NodeId) {
        self.display.hide_node(id);
    }

    /// Hide one member (view-only).
    pub(crate) fn hide_member(&mut self, id: MemberId) {
        self.display.hide_member(id);
    }

    /// Hide the whole current selection (view-only; never touches the model).
    pub(crate) fn hide_selected(&mut self) {
        let nodes: Vec<NodeId> = self.selected.nodes().to_vec();
        let members: Vec<MemberId> = self.selected.members().to_vec();
        for id in nodes {
            self.hide_node(id);
        }
        for id in members {
            self.hide_member(id);
        }
    }

    /// Context-menu helper: hide the clicked `target`.  When `target` is part
    /// of a multi-selection the whole selection is hidden, otherwise the single
    /// target is selected and hidden.
    pub(crate) fn hide_target_or_selection(&mut self, target: SelectionState) {
        if !self.selected.contains_state(target) {
            self.select_only(target);
        }
        self.hide_selected();
    }

    /// `Hide others` for the current selection.
    pub(crate) fn hide_others_for_selection(&mut self) {
        match self.selection {
            SelectionState::Node(id) => self.hide_others_node(id),
            SelectionState::Member(id) => self.hide_others_member(id),
            SelectionState::None => {}
        }
    }

    fn hide_others_node(&mut self, keep: NodeId) {
        let Some(scene) = &self.scene else {
            return;
        };
        let all_nodes: Vec<NodeId> = scene.node_ids.clone();
        let all_members: Vec<MemberId> = scene.member_ids.clone();
        // Members attached to the kept node.
        let keep_members: Vec<MemberId> = (0..scene.n_members())
            .filter(|&mi| {
                let (a, b) = scene.member_nodes[mi];
                scene.node_ids.get(a) == Some(&keep) || scene.node_ids.get(b) == Some(&keep)
            })
            .map(|mi| scene.member_ids[mi])
            .collect();
        self.display
            .isolate_node(&all_nodes, &all_members, keep, &keep_members);
    }

    fn hide_others_member(&mut self, keep: MemberId) {
        let Some(scene) = &self.scene else {
            return;
        };
        let all_nodes: Vec<NodeId> = scene.node_ids.clone();
        let all_members: Vec<MemberId> = scene.member_ids.clone();
        let Some(mi) = scene.member_index(keep) else {
            return;
        };
        let (a, b) = scene.member_nodes[mi];
        let keep_nodes: Vec<NodeId> =
            [scene.node_ids.get(a).copied(), scene.node_ids.get(b).copied()]
                .into_iter()
                .flatten()
                .collect();
        self.display
            .isolate_member(&all_nodes, &all_members, keep, &keep_nodes);
    }

    /// Unhide everything (`Show all`).
    pub(crate) fn show_all_hidden(&mut self) {
        self.display.show_all();
    }

    /// Update a node's coordinates (undoable; reuses the inspector's rule).
    pub(crate) fn apply_node_edit(&mut self, id: NodeId, x: f64, y: f64, z: f64) {
        let Some(model) = self.project.model.as_ref() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return;
        };
        let mut candidate = model.clone();
        match candidate.update_node(id, x, y, z) {
            Ok(()) => {
                if candidate.revision() != model.revision() {
                    self.commit_edit(candidate);
                    self.messages
                        .push(AppMessage::info(format!("Updated node N{}", id.0)));
                }
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
    }

    /// Delete the current selection (undoable) in **one atomic step**.
    ///
    /// Members are removed explicitly first, then nodes are cascade-deleted
    /// (taking any still-attached members with them).  Because the whole
    /// mutation runs on a single cloned model committed once via
    /// [`commit_edit`](Self::commit_edit), it produces exactly one undo entry.
    /// Returns `true` when something was selected and deleted.
    pub(crate) fn delete_selection(&mut self) -> bool {
        if self.selected.is_empty() {
            return false;
        }
        let node_ids: Vec<NodeId> = self.selected.nodes().to_vec();
        let member_ids: Vec<MemberId> = self.selected.members().to_vec();
        let Some(model) = self.project.model.as_ref() else {
            self.messages.push(AppMessage::error("No model loaded"));
            return false;
        };
        let mut candidate = model.clone();
        let mut n_nodes = 0usize;
        let mut n_members = 0usize;
        // Explicit member deletions first; then cascade node deletions (which
        // may remove further attached members).
        for mid in member_ids {
            if candidate.member(mid).is_some() && candidate.delete_member(mid).is_ok() {
                n_members += 1;
            }
        }
        for nid in node_ids {
            if candidate.node(nid).is_some() {
                if let Ok(removed) = candidate.delete_node_cascade(nid) {
                    n_nodes += 1;
                    n_members += removed.len();
                }
            }
        }
        if n_nodes == 0 && n_members == 0 {
            return false;
        }
        self.commit_edit(candidate);
        self.clear_selection();
        self.messages.push(AppMessage::info(format!(
            "Deleted {n_nodes} node(s) and {n_members} member(s)"
        )));
        true
    }

    /// Context-menu helper: delete the clicked `target`.  When `target` is part
    /// of a multi-selection the whole selection is deleted atomically,
    /// otherwise the single target is selected and deleted.
    pub(crate) fn delete_target_or_selection(&mut self, target: SelectionState) {
        if !self.selected.contains_state(target) {
            self.select_only(target);
        }
        self.delete_selection();
    }

    // ── Phase P4: double-click quick edit ──────────────────────────────

    /// Open the double-click quick-edit popup for the entity under the cursor.
    pub(crate) fn open_quick_edit(&mut self, hit: SelectionState, anchor: egui::Pos2) {
        match hit {
            SelectionState::Node(id) => {
                let Some((x, y, z)) = self
                    .project
                    .model
                    .as_ref()
                    .and_then(|m| m.node(id))
                    .map(|n| (n.x, n.y, n.z))
                else {
                    return;
                };
                self.quick_edit = Some(QuickEdit {
                    target: hit,
                    anchor,
                    x,
                    y,
                    z,
                    node_i: String::new(),
                    node_j: String::new(),
                });
            }
            SelectionState::Member(id) => {
                self.sync_member_edit_buffer(id);
                let Some((ni, nj)) = self
                    .project
                    .model
                    .as_ref()
                    .and_then(|m| m.member(id))
                    .map(|m| (m.node_i.0, m.node_j.0))
                else {
                    return;
                };
                self.quick_edit = Some(QuickEdit {
                    target: hit,
                    anchor,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    node_i: ni.to_string(),
                    node_j: nj.to_string(),
                });
            }
            SelectionState::None => {}
        }
    }

    /// Apply the quick-edit popup and close it.  Nodes reuse the coordinate
    /// update rule; members reuse the inspector's member-edit apply path (with
    /// an optional connectivity change first).
    pub(crate) fn apply_quick_edit(&mut self) {
        let Some(qe) = self.quick_edit.take() else {
            return;
        };
        match qe.target {
            SelectionState::Node(id) => self.apply_node_edit(id, qe.x, qe.y, qe.z),
            SelectionState::Member(id) => {
                self.apply_member_connectivity_from_strings(id, &qe.node_i, &qe.node_j);
                self.apply_member_edit(id);
            }
            SelectionState::None => {}
        }
    }

    // ── Phase B1: Boundary quick-add popups ─────────────────────────────

    /// Screen anchor for a ribbon-opened popup: the pointer position when
    /// the cursor is available, else the centre of the screen.
    fn popup_anchor(ctx: &egui::Context) -> egui::Pos2 {
        ctx.input(|i| i.pointer.interact_pos())
            .unwrap_or_else(|| ctx.screen_rect().center())
    }

    /// Open the Support popup for the primary selected node.
    ///
    /// No-op (and no popup) when the primary selection is not a node — the
    /// ribbon button is disabled in that case, so this is a defensive guard.
    pub(crate) fn open_support_popup(&mut self, ctx: &egui::Context) {
        let Some(id) = self.selected.primary().node_id() else {
            return;
        };
        let spec = self
            .project
            .model
            .as_ref()
            .and_then(|m| m.support(id))
            .copied()
            .unwrap_or_else(SupportSpec::free);
        self.boundary_popup = Some(BoundaryPopup::Support {
            node: id,
            anchor: Self::popup_anchor(ctx),
            spec,
            error: None,
        });
    }

    /// Open the Node Load popup for the primary selected node, prefilled
    /// with the node's existing load when it has one.
    pub(crate) fn open_nodal_load_popup(&mut self, ctx: &egui::Context) {
        let Some(id) = self.selected.primary().node_id() else {
            return;
        };
        let load = self
            .project
            .model
            .as_ref()
            .and_then(|m| m.nodal_load(id))
            .copied()
            .unwrap_or_default();
        self.boundary_popup = Some(BoundaryPopup::NodeLoad {
            node: id,
            anchor: Self::popup_anchor(ctx),
            load,
            error: None,
        });
    }

    /// Open the Mem Load popup for the primary selected member, prefilled
    /// with the member's existing uniform load when it has one.
    pub(crate) fn open_member_load_popup(&mut self, ctx: &egui::Context) {
        let Some(id) = self.selected.primary().member_id() else {
            return;
        };
        let (qx, qy, qz) = self
            .project
            .model
            .as_ref()
            .and_then(|m| m.member_uniform_load(id))
            .map(|l| (l.qx, l.qy, l.qz))
            .unwrap_or((0.0, 0.0, 0.0));
        self.boundary_popup = Some(BoundaryPopup::MemLoad {
            member: id,
            anchor: Self::popup_anchor(ctx),
            qx,
            qy,
            qz,
            add_point: false,
            xi: 0.5,
            fx: 0.0,
            fy: 0.0,
            fz: 0.0,
            error: None,
        });
    }

    /// Apply the Support popup: write `spec` through the model's support
    /// setter and the shared edit/undo path ([`commit_edit`](Self::commit_edit)).
    /// An all-free spec clears the support instead of storing a free one.
    ///
    /// Returns `Err(message)` when the edit is rejected (e.g. the node no
    /// longer exists) so the popup can surface it; the same message is pushed
    /// to the Messages dock.  A no-op apply (unchanged support) reports
    /// "No changes to apply." and does not touch the undo history.
    pub(crate) fn apply_support_quick(
        &mut self,
        id: NodeId,
        spec: SupportSpec,
    ) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".to_string());
        };
        let mut candidate = model.clone();
        let result = if spec.is_free() {
            candidate.clear_support(id)
        } else {
            candidate.set_support(id, spec)
        };
        match result {
            Ok(()) => {
                if candidate.revision() != model.revision() {
                    self.commit_edit(candidate);
                    let msg = if spec.is_free() {
                        format!("Cleared support at node N{}", id.0)
                    } else {
                        format!("Applied {} support at node N{}", spec.label(), id.0)
                    };
                    self.messages.push(AppMessage::info(msg));
                } else {
                    self.messages.push(AppMessage::info("No changes to apply."));
                }
                Ok(())
            }
            Err(e) => {
                let msg = e.message();
                self.messages.push(AppMessage::error(msg.clone()));
                Err(msg)
            }
        }
    }

    /// Apply the Node Load popup: set the six-component nodal load through
    /// the model's setter and the shared edit/undo path.  An all-zero load
    /// clears the node's load instead of storing a zero one.
    ///
    /// Error / no-op reporting matches [`apply_support_quick`](Self::apply_support_quick).
    pub(crate) fn apply_nodal_load_quick(
        &mut self,
        id: NodeId,
        load: NodalLoadSpec,
    ) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".to_string());
        };
        let mut candidate = model.clone();
        let result = if load.is_zero() {
            candidate.clear_nodal_load(id)
        } else {
            candidate.set_nodal_load(id, load)
        };
        match result {
            Ok(()) => {
                if candidate.revision() != model.revision() {
                    self.commit_edit(candidate);
                    let msg = if load.is_zero() {
                        format!("Cleared load at node N{}", id.0)
                    } else {
                        format!("Applied nodal load at node N{}", id.0)
                    };
                    self.messages.push(AppMessage::info(msg));
                } else {
                    self.messages.push(AppMessage::info("No changes to apply."));
                }
                Ok(())
            }
            Err(e) => {
                let msg = e.message();
                self.messages.push(AppMessage::error(msg.clone()));
                Err(msg)
            }
        }
    }

    /// Apply the Mem Load popup: uniform local `qx/qy/qz` plus an optional
    /// point load, **all on one cloned model** so the whole popup is a single
    /// undo step.  An all-zero uniform load clears the member's uniform load;
    /// the optional point load is appended (multiple point loads per member
    /// are allowed by the model).
    ///
    /// Every mutation runs on the candidate clone, so a rejected point load
    /// (e.g. `xi` outside `[0, 1]`) leaves the real model untouched and
    /// nothing is committed — the failure is surfaced, never silent.
    pub(crate) fn apply_member_load_quick(
        &mut self,
        id: MemberId,
        qx: f64,
        qy: f64,
        qz: f64,
        point: Option<MemberPointLoad3D>,
    ) -> Result<(), String> {
        let Some(model) = self.project.model.as_ref() else {
            return Err("No model loaded".to_string());
        };
        let uniform = MemberUniformLoad3D::new(qx, qy, qz);
        let mut candidate = model.clone();
        let uniform_result = if uniform.is_zero() {
            candidate.clear_member_uniform_load(id)
        } else {
            candidate.set_member_uniform_load(id, uniform)
        };
        if let Err(e) = uniform_result {
            let msg = e.message();
            self.messages.push(AppMessage::error(msg.clone()));
            return Err(msg);
        }
        if let Some(pl) = point {
            if let Err(e) = candidate.add_member_point_load(id, pl) {
                let msg = e.message();
                self.messages.push(AppMessage::error(msg.clone()));
                return Err(msg);
            }
        }
        if candidate.revision() != model.revision() {
            self.commit_edit(candidate);
            self.messages
                .push(AppMessage::info(format!("Applied member load to M{}", id.0)));
        } else {
            self.messages.push(AppMessage::info("No changes to apply."));
        }
        Ok(())
    }

    /// Change a member's `i` / `j` nodes from text fields, if they parse and
    /// differ from the current values.
    fn apply_member_connectivity_from_strings(&mut self, id: MemberId, i: &str, j: &str) {
        let (Ok(ni), Ok(nj)) = (i.trim().parse::<u32>(), j.trim().parse::<u32>()) else {
            return;
        };
        let Some(model) = self.project.model.as_ref() else {
            return;
        };
        let Some(m) = model.member(id) else {
            return;
        };
        if m.node_i == NodeId(ni) && m.node_j == NodeId(nj) {
            return;
        }
        let mut candidate = model.clone();
        match candidate.set_member_nodes(id, NodeId(ni), NodeId(nj)) {
            Ok(()) => {
                let changed = candidate.revision() != model.revision();
                if changed {
                    self.commit_edit(candidate);
                }
            }
            Err(e) => self.messages.push(AppMessage::error(e.message())),
        }
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
        self.clear_selection();
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
        self.clear_selection();
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
                self.clear_selection();
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

    /// Drop selected IDs that no longer exist in the model (after an edit /
    /// delete / undo), keeping the rest of the selection valid.
    pub(crate) fn validate_selection(&mut self) {
        let retained: Option<SelectionSet> = self.scene.as_ref().map(|scene| {
            let mut set = self.selected.clone();
            set.retain(
                |n| scene.node_index(n).is_some(),
                |m| scene.member_index(m).is_some(),
            );
            set
        });
        match retained {
            Some(set) => self.set_selection(set),
            None => self.clear_selection(),
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
            if self.display.is_node_hidden(scene.node_ids[i]) {
                continue;
            }
            // Ignore nodes the section cut has removed from the view.
            if !self.display.section_cut.keeps(pos) {
                continue;
            }
            if let Some(p) = self.camera.project(pos, w, h) {
                let d = (p.x - mouse[0]).hypot(p.y - mouse[1]);
                if d < THRESH && best_node.map_or(true, |(_, bd)| d < bd) {
                    best_node = Some((i, d));
                }
            }
        }
        let mut best_mem: Option<(usize, f64)> = None;
        for (i, &(ni, nj)) in scene.member_nodes.iter().enumerate() {
            if self.display.is_member_hidden(scene.member_ids[i]) {
                continue;
            }
            let Some(pi) = scene.node_positions.get(ni) else {
                continue;
            };
            let Some(pj) = scene.node_positions.get(nj) else {
                continue;
            };
            // Hit-test only the part of the centreline that survives the cut.
            let Some((ca, cb)) = self.display.section_cut.clip_segment(*pi, *pj) else {
                continue;
            };
            let Some(ps_i) = self.camera.project(ca, w, h) else {
                continue;
            };
            let Some(ps_j) = self.camera.project(cb, w, h) else {
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
            // Phase P6: Ctrl+A selects every visible node/member; Delete
            // removes the whole selection as one undoable step.
            if ctx.input(|i| i.key_pressed(egui::Key::A) && i.modifiers.ctrl) {
                self.select_all_visible();
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
                self.delete_selection();
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

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::edit_model::EditModel;

    /// An app with no model, for the guard paths.
    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// An app holding one 4 m beam (nodes N0/N1, member M0).
    fn beam_app() -> (ViewportApp, NodeId, MemberId) {
        let mut app = empty_app();
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        app.project.load_model("test".to_string(), m);
        (app, a, mid)
    }

    fn last_message(app: &ViewportApp) -> &str {
        &app.messages.last().expect("no message").text
    }

    // ── Phase B1: Boundary quick-add popups ─────────────────────────────

    #[test]
    fn support_quick_fixed_commits_and_is_undoable() {
        let (mut app, a, _mid) = beam_app();
        app.select_only(SelectionState::Node(a));
        app.apply_support_quick(a, SupportSpec::fixed()).unwrap();
        let model = app.project.model.as_ref().unwrap();
        assert!(model.support(a).unwrap().is_fixed());
        assert!(app.history.can_undo(), "edit must be undoable");
        assert!(last_message(&app).contains("Fixed support"));
        // One undo restores the support-free model.
        app.do_undo();
        assert!(app.project.model.as_ref().unwrap().support(a).is_none());
    }

    #[test]
    fn support_quick_custom_spec_is_stored() {
        let (mut app, a, _mid) = beam_app();
        let mut spec = SupportSpec::free();
        spec.set_dof(structural_analysis::Dof3D::Ux, Some(0.0));
        app.apply_support_quick(a, spec).unwrap();
        let model = app.project.model.as_ref().unwrap();
        let stored = model.support(a).unwrap();
        assert_eq!(stored.n_constrained(), 1);
        assert_eq!(stored.label(), "Custom");
    }

    #[test]
    fn support_quick_free_spec_clears_existing_support() {
        let (mut app, a, _mid) = beam_app();
        app.apply_support_quick(a, SupportSpec::fixed()).unwrap();
        app.apply_support_quick(a, SupportSpec::free()).unwrap();
        assert!(app.project.model.as_ref().unwrap().support(a).is_none());
        assert!(last_message(&app).contains("Cleared support"));
    }

    #[test]
    fn support_quick_unchanged_reports_no_change_without_history() {
        let (mut app, a, _mid) = beam_app();
        app.apply_support_quick(a, SupportSpec::fixed()).unwrap();
        let rev = app.project.model.as_ref().unwrap().revision();
        app.apply_support_quick(a, SupportSpec::fixed()).unwrap();
        assert_eq!(app.project.model.as_ref().unwrap().revision(), rev);
        assert_eq!(last_message(&app), "No changes to apply.");
    }

    #[test]
    fn support_quick_requires_a_model() {
        let mut app = empty_app();
        let err = app
            .apply_support_quick(NodeId(0), SupportSpec::fixed())
            .unwrap_err();
        assert_eq!(err, "No model loaded");
    }

    #[test]
    fn nodal_load_quick_sets_and_clears() {
        let (mut app, a, _mid) = beam_app();
        app.apply_nodal_load_quick(a, NodalLoadSpec::new(1e3, 0.0, -2e3, 0.0, 0.0, 0.0))
            .unwrap();
        {
            let model = app.project.model.as_ref().unwrap();
            let load = model.nodal_load(a).unwrap();
            assert_eq!(load.fx, 1e3);
            assert_eq!(load.fz, -2e3);
        }
        assert!(app.history.can_undo());
        // An all-zero load clears instead of storing a zero record.
        app.apply_nodal_load_quick(a, NodalLoadSpec::zero())
            .unwrap();
        assert!(app.project.model.as_ref().unwrap().nodal_load(a).is_none());
        assert!(last_message(&app).contains("Cleared load"));
    }

    #[test]
    fn member_load_quick_applies_uniform_and_point_in_one_undo_step() {
        let (mut app, _a, mid) = beam_app();
        let point = MemberPointLoad3D::new(0.25, 0.0, 0.0, -500.0);
        app.apply_member_load_quick(mid, 0.0, -1e3, 0.0, Some(point))
            .unwrap();
        {
            let model = app.project.model.as_ref().unwrap();
            let q = model.member_uniform_load(mid).unwrap();
            assert_eq!(q.qy, -1e3);
            let pls = model.member_point_loads_for(mid);
            assert_eq!(pls.len(), 1);
            assert_eq!(pls[0].xi, 0.25);
        }
        // The whole popup is one undo entry.
        app.do_undo();
        let model = app.project.model.as_ref().unwrap();
        assert!(model.member_uniform_load(mid).is_none());
        assert!(model.member_point_loads_for(mid).is_empty());
    }

    #[test]
    fn member_load_quick_zero_uniform_without_point_clears() {
        let (mut app, _a, mid) = beam_app();
        app.apply_member_load_quick(mid, 0.0, -1e3, 0.0, None)
            .unwrap();
        app.apply_member_load_quick(mid, 0.0, 0.0, 0.0, None)
            .unwrap();
        let model = app.project.model.as_ref().unwrap();
        assert!(model.member_uniform_load(mid).is_none());
    }

    #[test]
    fn member_load_quick_rejects_out_of_range_xi_atomically() {
        let (mut app, _a, mid) = beam_app();
        let bad = MemberPointLoad3D::new(1.5, 0.0, 0.0, -100.0);
        let err = app.apply_member_load_quick(mid, 0.0, -1e3, 0.0, Some(bad));
        assert!(err.is_err(), "xi = 1.5 must be rejected");
        let err = err.unwrap_err();
        assert!(err.contains("xi"), "error should name the position: {err}");
        // Nothing was committed: not the point load, not even the uniform.
        let model = app.project.model.as_ref().unwrap();
        assert!(model.member_uniform_load(mid).is_none());
        assert!(model.member_point_loads_for(mid).is_empty());
        assert!(!app.history.can_undo());
    }

    #[test]
    fn member_load_quick_requires_a_model() {
        let mut app = empty_app();
        let err = app.apply_member_load_quick(MemberId(0), 0.0, 0.0, 0.0, None);
        assert_eq!(err.unwrap_err(), "No model loaded");
    }

    #[test]
    fn open_support_popup_requires_a_node_selection() {
        let (mut app, a, _mid) = beam_app();
        let ctx = egui::Context::default();
        // No selection → no popup.
        app.open_support_popup(&ctx);
        assert!(app.boundary_popup.is_none());
        // A member primary → still no support popup.
        app.select_only(SelectionState::Member(_mid));
        app.open_support_popup(&ctx);
        assert!(app.boundary_popup.is_none());
        // A node primary → popup targeting that node.
        app.select_only(SelectionState::Node(a));
        app.open_support_popup(&ctx);
        match &app.boundary_popup {
            Some(BoundaryPopup::Support { node, spec, .. }) => {
                assert_eq!(*node, a);
                assert!(spec.is_free(), "no existing support ⇒ free spec");
            }
            _ => panic!("expected a Support popup"),
        }
    }

    #[test]
    fn open_nodal_load_popup_prefills_existing_load() {
        let (mut app, a, _mid) = beam_app();
        app.apply_nodal_load_quick(a, NodalLoadSpec::new(7.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        app.select_only(SelectionState::Node(a));
        let ctx = egui::Context::default();
        app.open_nodal_load_popup(&ctx);
        match &app.boundary_popup {
            Some(BoundaryPopup::NodeLoad { node, load, .. }) => {
                assert_eq!(*node, a);
                assert_eq!(load.fx, 7.0);
            }
            _ => panic!("expected a NodeLoad popup"),
        }
    }

    #[test]
    fn open_member_load_popup_prefills_existing_uniform_load() {
        let (mut app, _a, mid) = beam_app();
        app.apply_member_load_quick(mid, 1.0, 2.0, 3.0, None)
            .unwrap();
        app.select_only(SelectionState::Member(mid));
        let ctx = egui::Context::default();
        app.open_member_load_popup(&ctx);
        match &app.boundary_popup {
            Some(BoundaryPopup::MemLoad {
                member,
                qx,
                qy,
                qz,
                add_point,
                ..
            }) => {
                assert_eq!(*member, mid);
                assert_eq!((*qx, *qy, *qz), (1.0, 2.0, 3.0));
                assert!(!add_point);
            }
            _ => panic!("expected a MemLoad popup"),
        }
    }
}
