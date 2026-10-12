//! Phase 152 regression tests: bounded undo/redo for GUI model editing.
//!
//! These tests exercise the library-level components (`EditHistory`,
//! `EditModel::fingerprint`, `EditModel::ensure_revision_at_least`,
//! `Project::replace_model`) and a test controller that simulates the
//! `ViewportApp` undo/redo behaviour.

use viewport_poc::{
    Project,
    edit_history::EditHistory,
    edit_model::{EditError, EditModel, NodalLoadSpec, NodeId, SupportSpec},
    model,
};

// ── Test controller (simulates ViewportApp undo/redo logic) ─────────────

struct Controller {
    project: Project,
    history: EditHistory,
    saved_fingerprint: Option<u64>,
}

impl Controller {
    fn new() -> Self {
        Self {
            project: Project::empty(),
            history: EditHistory::default(),
            saved_fingerprint: None,
        }
    }

    fn load_preset(&mut self, preset: model::PresetModel) {
        let name = preset.name().to_string();
        let m = preset.build_edit();
        self.project.load_model(name, m);
        self.history.clear();
        self.saved_fingerprint = None;
    }

    fn load_model(&mut self, name: &str, model: EditModel) {
        self.project.load_model(name.to_string(), model);
        self.history.clear();
        self.saved_fingerprint = None;
    }

    fn do_new(&mut self) {
        self.project.load_model("Untitled".into(), EditModel::new());
        self.history.clear();
        self.saved_fingerprint = None;
    }

    fn clear_project(&mut self) {
        self.project.clear();
        self.history.clear();
        self.saved_fingerprint = None;
    }

    /// Simulates `ViewportApp::commit_edit`.
    fn commit_edit(&mut self, new_model: EditModel) {
        if let Some(old) = self.project.model.take() {
            self.history.record(old);
        }
        self.project.replace_model(new_model);
    }

    /// Simulates a candidate-model edit: clone, mutate, commit if effective.
    /// Returns `true` if the edit was committed.
    fn edit<T, F>(&mut self, f: F) -> bool
    where
        F: FnOnce(&mut EditModel) -> Result<T, EditError>,
    {
        let Some(model) = self.project.model.as_ref() else {
            return false;
        };
        let mut candidate = model.clone();
        match f(&mut candidate) {
            Ok(_) => {
                if candidate.revision() != model.revision() {
                    self.commit_edit(candidate);
                    true
                } else {
                    false
                }
            }
            Err(_) => false,
        }
    }

    /// Simulates `ViewportApp::do_undo`.
    fn do_undo(&mut self) -> bool {
        let Some(current) = self.project.model.clone() else {
            return false;
        };
        let Some(mut restored) = self.history.undo(current.clone()) else {
            return false;
        };
        restored.ensure_revision_at_least(current.revision() + 1);
        self.project.replace_model(restored);
        true
    }

    /// Simulates `ViewportApp::do_redo`.
    fn do_redo(&mut self) -> bool {
        let Some(current) = self.project.model.clone() else {
            return false;
        };
        let Some(mut restored) = self.history.redo(current.clone()) else {
            return false;
        };
        restored.ensure_revision_at_least(current.revision() + 1);
        self.project.replace_model(restored);
        true
    }

    /// Simulates `ViewportApp::is_dirty` (fingerprint-based).
    fn is_dirty(&self) -> bool {
        self.project
            .model
            .as_ref()
            .map_or(false, |m| self.saved_fingerprint != Some(m.fingerprint()))
    }

    /// Simulates saving the project to a file.
    fn save(&mut self) {
        if let Some(m) = self.project.model.as_ref() {
            self.saved_fingerprint = Some(m.fingerprint());
        }
    }

    /// Simulates opening a project from a file.
    fn open(&mut self, model: EditModel, name: &str) {
        self.project.load_model(name.to_string(), model);
        self.history.clear();
        self.saved_fingerprint = self.project.model.as_ref().map(|m| m.fingerprint());
    }

    fn model(&self) -> &EditModel {
        self.project.model.as_ref().unwrap()
    }

    fn revision(&self) -> u64 {
        self.model().revision()
    }

    fn fingerprint(&self) -> u64 {
        self.model().fingerprint()
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────

fn make_portal_controller() -> Controller {
    let mut c = Controller::new();
    c.load_preset(model::PresetModel::Portal);
    c
}

// ── 1. EditModel::ensure_revision_at_least ──────────────────────────────

#[test]
fn ensure_revision_bumps_when_lower() {
    let mut m = EditModel::new();
    assert_eq!(m.revision(), 0);
    m.ensure_revision_at_least(10);
    assert_eq!(m.revision(), 10);
}

#[test]
fn ensure_revision_noop_when_already_higher() {
    let mut m = EditModel::new();
    m.add_node(1.0, 0.0, 0.0).unwrap();
    m.add_node(2.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.ensure_revision_at_least(rev - 1);
    assert_eq!(m.revision(), rev);
}

#[test]
fn ensure_revision_saturates() {
    let mut m = EditModel::new();
    m.ensure_revision_at_least(u64::MAX);
    assert_eq!(m.revision(), u64::MAX);
    m.ensure_revision_at_least(u64::MAX);
    assert_eq!(m.revision(), u64::MAX);
}

// ── 2. EditModel::fingerprint ───────────────────────────────────────────

#[test]
fn fingerprint_identical_models_match() {
    let m1 = model::build_portal_edit();
    let m2 = model::build_portal_edit();
    assert_eq!(m1.fingerprint(), m2.fingerprint());
}

#[test]
fn fingerprint_different_models_differ() {
    let m1 = model::build_portal_edit();
    let m2 = model::build_inclined_edit();
    assert_ne!(m1.fingerprint(), m2.fingerprint());
}

#[test]
fn fingerprint_excludes_revision() {
    let mut m1 = model::build_portal_edit();
    let m2 = model::build_portal_edit();
    m1.ensure_revision_at_least(999);
    assert_eq!(m1.fingerprint(), m2.fingerprint());
}

#[test]
fn fingerprint_changes_on_node_edit() {
    let mut m = model::build_portal_edit();
    let fp0 = m.fingerprint();
    let id = m.nodes()[0].id;
    m.update_node(id, 999.0, 0.0, 0.0).unwrap();
    assert_ne!(m.fingerprint(), fp0);
}

#[test]
fn fingerprint_changes_on_member_delete() {
    let mut m = model::build_portal_edit();
    let fp0 = m.fingerprint();
    let mid = m.members()[0].id;
    m.delete_member(mid).unwrap();
    assert_ne!(m.fingerprint(), fp0);
}

#[test]
fn fingerprint_changes_on_support() {
    let mut m = model::build_portal_edit();
    let fp0 = m.fingerprint();
    let nid = m.nodes()[0].id;
    let new_spec = if m.support(nid).map_or(false, |s| s.is_fixed()) {
        SupportSpec::pinned()
    } else {
        SupportSpec::fixed()
    };
    m.set_support(nid, new_spec).unwrap();
    assert_ne!(m.fingerprint(), fp0);
}

#[test]
fn fingerprint_changes_on_nodal_load() {
    let mut m = model::build_portal_edit();
    let fp0 = m.fingerprint();
    let nid = m.nodes()[0].id;
    m.set_nodal_load(nid, NodalLoadSpec::new(100.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    assert_ne!(m.fingerprint(), fp0);
}

#[test]
fn fingerprint_empty_model_consistent() {
    let m1 = EditModel::new();
    let m2 = EditModel::new();
    assert_eq!(m1.fingerprint(), m2.fingerprint());
}

// ── 3. Project::replace_model ───────────────────────────────────────────

#[test]
fn replace_model_invalidates_analysis() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(p.has_results());
    p.replace_model(model::build_inclined_edit());
    assert!(!p.has_results());
    assert_eq!(
        p.analysis_status(),
        viewport_poc::app_state::AnalysisStatus::NeverSolved
    );
}

#[test]
fn replace_model_preserves_name() {
    let mut p = Project::empty();
    p.load_model("MyProject".into(), model::build_portal_edit());
    p.replace_model(model::build_inclined_edit());
    assert_eq!(p.name, "MyProject");
}

// ── 4. Undo/Redo basic flow ─────────────────────────────────────────────

#[test]
fn undo_restores_previous_model() {
    let mut c = make_portal_controller();
    let fp_original = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert_ne!(c.fingerprint(), fp_original);

    assert!(c.do_undo());
    assert_eq!(c.fingerprint(), fp_original);
}

#[test]
fn redo_restores_edited_model() {
    let mut c = make_portal_controller();
    let fp_original = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    let fp_edited = c.fingerprint();

    c.do_undo();
    assert_eq!(c.fingerprint(), fp_original);

    assert!(c.do_redo());
    assert_eq!(c.fingerprint(), fp_edited);
}

#[test]
fn undo_when_empty_is_noop() {
    let mut c = make_portal_controller();
    assert!(!c.history.can_undo());
    assert!(!c.do_undo());
}

#[test]
fn redo_when_nothing_to_redo_is_noop() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert!(!c.history.can_redo());
    assert!(!c.do_redo());
}

#[test]
fn redo_after_new_edit_is_empty() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 2.0, 0.0, 0.0)
    });
    c.do_undo();
    assert!(c.history.can_redo());
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 3.0, 0.0, 0.0)
    });
    assert!(!c.history.can_redo());
}

// ── 5. Revision invariant ───────────────────────────────────────────────

#[test]
fn undo_increases_revision() {
    let mut c = make_portal_controller();
    let rev_before = c.revision();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    let rev_after_edit = c.revision();
    assert!(rev_after_edit > rev_before);

    c.do_undo();
    let rev_after_undo = c.revision();
    assert!(rev_after_undo > rev_after_edit);
}

#[test]
fn redo_increases_revision() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    let rev_after_edit = c.revision();

    c.do_undo();
    let rev_after_undo = c.revision();

    c.do_redo();
    let rev_after_redo = c.revision();
    assert!(rev_after_redo > rev_after_undo);
    assert!(rev_after_redo > rev_after_edit);
}

#[test]
fn undo_does_not_make_stale_result_current() {
    let mut c = make_portal_controller();
    c.project.solve().unwrap();
    assert!(c.project.has_results());
    let solved_rev = c.project.analysis.as_ref().unwrap().solved_revision;

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert!(!c.project.has_results());

    c.do_undo();
    assert!(!c.project.has_results());
    let current_rev = c.revision();
    assert_ne!(solved_rev, current_rev);
}

// ── 6. Dirty-state semantics ────────────────────────────────────────────

#[test]
fn dirty_false_after_save() {
    let mut c = make_portal_controller();
    c.save();
    assert!(!c.is_dirty());
}

#[test]
fn dirty_true_after_edit() {
    let mut c = make_portal_controller();
    c.save();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert!(c.is_dirty());
}

#[test]
fn dirty_false_after_save_edit_undo() {
    let mut c = make_portal_controller();
    c.save();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert!(c.is_dirty());
    c.do_undo();
    assert!(!c.is_dirty());
}

#[test]
fn dirty_true_after_save_edit_save_undo() {
    let mut c = make_portal_controller();
    c.save();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    c.save();
    c.do_undo();
    assert!(c.is_dirty());
}

#[test]
fn dirty_true_for_unsaved_new_model() {
    let mut c = Controller::new();
    c.do_new();
    assert!(c.is_dirty());
}

#[test]
fn dirty_true_for_unsaved_preset() {
    let mut c = make_portal_controller();
    assert!(c.is_dirty());
}

#[test]
fn dirty_false_after_open() {
    let mut c = Controller::new();
    let m = model::build_portal_edit();
    c.open(m, "Portal");
    assert!(!c.is_dirty());
}

// ── 7. History boundaries ───────────────────────────────────────────────

#[test]
fn new_clears_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    assert!(c.history.can_undo());
    c.do_new();
    assert!(!c.history.can_undo());
}

#[test]
fn open_clears_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    assert!(c.history.can_undo());
    c.open(model::build_inclined_edit(), "Inclined");
    assert!(!c.history.can_undo());
}

#[test]
fn preset_clears_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    assert!(c.history.can_undo());
    c.load_preset(model::PresetModel::Inclined);
    assert!(!c.history.can_undo());
}

#[test]
fn clear_project_clears_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    assert!(c.history.can_undo());
    c.clear_project();
    assert!(!c.history.can_undo());
}

#[test]
fn save_preserves_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    c.save();
    assert!(c.history.can_undo());
}

#[test]
fn solve_preserves_history() {
    let mut c = make_portal_controller();
    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    c.project.solve().ok();
    assert!(c.history.can_undo());
}

// ── 8. No-op edit ───────────────────────────────────────────────────────

#[test]
fn noop_edit_does_not_create_history() {
    let mut c = make_portal_controller();
    let id = c.model().nodes()[0].id;
    let (x, y, z) = {
        let n = c.model().node(id).unwrap();
        (n.x, n.y, n.z)
    };
    let committed = c.edit(|m| m.update_node(id, x, y, z));
    assert!(!committed);
    assert!(!c.history.can_undo());
}

#[test]
fn failed_edit_does_not_create_history() {
    let mut c = make_portal_controller();
    let committed = c.edit(|m| m.add_node(f64::NAN, 0.0, 0.0));
    assert!(!committed);
    assert!(!c.history.can_undo());
}

// ── 9. Bounded capacity ─────────────────────────────────────────────────

#[test]
fn capacity_drops_oldest_undo() {
    let mut c = Controller::new();
    c.history = EditHistory::new(3);
    c.load_preset(model::PresetModel::Portal);

    for i in 1..=5 {
        c.edit(|m| {
            let id = m.nodes()[0].id;
            m.update_node(id, i as f64, 0.0, 0.0)
        });
    }
    assert_eq!(c.history.undo_depth(), 3);

    for _ in 0..3 {
        assert!(c.do_undo());
    }
    assert!(!c.history.can_undo());
}

// ── 10. Multiple edits and undo/redo chains ─────────────────────────────

#[test]
fn multiple_edits_then_sequential_undo() {
    let mut c = make_portal_controller();
    let fp0 = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    let fp1 = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[1].id;
        m.update_node(id, 2.0, 0.0, 0.0)
    });
    let fp2 = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 3.0, 0.0, 0.0)
    });
    let fp3 = c.fingerprint();

    assert_eq!(c.history.undo_depth(), 3);

    c.do_undo();
    assert_eq!(c.fingerprint(), fp2);
    c.do_undo();
    assert_eq!(c.fingerprint(), fp1);
    c.do_undo();
    assert_eq!(c.fingerprint(), fp0);
    assert!(!c.history.can_undo());

    c.do_redo();
    assert_eq!(c.fingerprint(), fp1);
    c.do_redo();
    assert_eq!(c.fingerprint(), fp2);
    c.do_redo();
    assert_eq!(c.fingerprint(), fp3);
    assert!(!c.history.can_redo());
}

#[test]
fn undo_redo_preserves_node_ids() {
    let mut c = make_portal_controller();
    let original_ids: Vec<NodeId> = c.model().nodes().iter().map(|n| n.id).collect();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    c.do_undo();

    let restored_ids: Vec<NodeId> = c.model().nodes().iter().map(|n| n.id).collect();
    assert_eq!(original_ids, restored_ids);
}

#[test]
fn undo_after_add_node_removes_it() {
    let mut c = make_portal_controller();
    let n_before = c.model().n_nodes();

    c.edit(|m| m.add_node(10.0, 10.0, 10.0));
    assert_eq!(c.model().n_nodes(), n_before + 1);

    c.do_undo();
    assert_eq!(c.model().n_nodes(), n_before);
}

#[test]
fn undo_after_delete_node_restores_it() {
    let mut c = make_portal_controller();

    c.edit(|m| m.add_node(10.0, 10.0, 10.0));
    let n_after_add = c.model().n_nodes();
    let new_node_id = c.model().nodes().last().unwrap().id;

    let committed = c.edit(|m| m.delete_node(new_node_id));
    assert!(committed);
    assert_eq!(c.model().n_nodes(), n_after_add - 1);

    c.do_undo();
    assert_eq!(c.model().n_nodes(), n_after_add);
    assert!(c.model().node(new_node_id).is_some());
}

#[test]
fn undo_after_add_member_removes_it() {
    let mut c = make_portal_controller();

    c.edit(|m| m.add_node(10.0, 10.0, 10.0));
    let new_node_id = c.model().nodes().last().unwrap().id;
    let existing_node = c.model().nodes()[0].id;
    let n_before = c.model().n_members();

    let committed = c.edit(|m| m.add_member(new_node_id, existing_node));
    assert!(committed);
    assert_eq!(c.model().n_members(), n_before + 1);

    c.do_undo();
    assert_eq!(c.model().n_members(), n_before);
}

// ── 11. Analysis invalidation ───────────────────────────────────────────

#[test]
fn edit_invalidates_analysis() {
    let mut c = make_portal_controller();
    c.project.solve().unwrap();
    assert!(c.project.has_results());

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    assert!(!c.project.has_results());
}

#[test]
fn undo_invalidates_analysis() {
    let mut c = make_portal_controller();
    c.project.solve().unwrap();
    assert!(c.project.has_results());

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 42.0, 0.0, 0.0)
    });
    c.do_undo();
    assert!(!c.project.has_results());
}

// ── Integration Scenario A: Save → Edit → Undo → Redo → Solve ──────────

#[test]
fn scenario_a_save_edit_undo_redo_solve() {
    let mut c = make_portal_controller();
    c.project.solve().unwrap();
    c.save();
    assert!(!c.is_dirty());
    assert!(c.project.has_results());

    let fp_saved = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 5.0, 0.0, 0.0)
    });
    assert!(c.is_dirty());
    assert!(!c.project.has_results());

    c.do_undo();
    assert!(!c.is_dirty());
    assert_eq!(c.fingerprint(), fp_saved);
    assert!(!c.project.has_results());

    c.do_redo();
    assert!(c.is_dirty());
    assert_ne!(c.fingerprint(), fp_saved);

    c.project.solve().unwrap();
    assert!(c.project.has_results());
}

// ── Integration Scenario B: Solve → Edit → Undo, result not Current ─────

#[test]
fn scenario_b_solve_edit_undo_result_not_current() {
    let mut c = make_portal_controller();
    c.project.solve().unwrap();
    assert!(c.project.has_results());
    let solved_rev = c.project.analysis.as_ref().unwrap().solved_revision;

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 5.0, 0.0, 0.0)
    });
    assert!(!c.project.has_results());

    c.do_undo();
    assert!(!c.project.has_results());
    let current_rev = c.revision();
    assert_ne!(
        solved_rev, current_rev,
        "Undo must not restore a revision that matches the solved revision"
    );
    assert!(
        current_rev > solved_rev,
        "Undo must produce a revision greater than the solved revision"
    );
}

// ── Integration Scenario C: Edit → Edit → Undo → Edit, Redo cleared ─────

#[test]
fn scenario_c_edit_edit_undo_edit_redo_cleared() {
    let mut c = make_portal_controller();
    let fp0 = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 1.0, 0.0, 0.0)
    });
    let fp1 = c.fingerprint();

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 2.0, 0.0, 0.0)
    });
    let fp2 = c.fingerprint();

    c.do_undo();
    assert_eq!(c.fingerprint(), fp1);
    assert!(c.history.can_redo());

    c.edit(|m| {
        let id = m.nodes()[0].id;
        m.update_node(id, 3.0, 0.0, 0.0)
    });
    let fp3 = c.fingerprint();
    assert_ne!(fp3, fp2);
    assert_ne!(fp3, fp1);
    assert_ne!(fp3, fp0);

    assert!(
        !c.history.can_redo(),
        "Redo must be cleared after a new edit following an undo"
    );
}
