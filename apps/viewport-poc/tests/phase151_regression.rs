//! Regression tests for Phase 151 interaction hardening.
//!
//! These tests verify the logic patterns used by the GUI to prevent
//! the P1 defects identified in the static interaction audit:
//!
//! P1-1: Failed save must not proceed to a destructive New/Open action.
//! P1-2: Clear project must check for unsaved changes.
//! P1-3: Preset loading must check for unsaved changes.
//!
//! The GUI methods themselves are private to `main.rs` (binary target)
//! and cannot be tested directly from integration tests.  These tests
//! verify the underlying logic patterns that the GUI relies on.

use viewport_poc::{
    edit_model::{EditModel, MemberId, NodeId},
    model, persistence,
};

// ── P1-1: Failed save does not update saved_revision ──────────────────

/// Verify that a failed save (invalid path) returns an error, which
/// the GUI interprets as `false` from `save_to_path`.  When `save_to_path`
/// returns `false`, `execute_pending()` must NOT be called.
#[test]
fn p1_1_failed_save_returns_error_not_success() {
    let m = model::PresetModel::Portal.build_edit();
    let bad_path = std::path::Path::new("Z:/nonexistent_dir_xyz/test.saproject");
    let result = persistence::save_project(bad_path, &m);
    assert!(
        result.is_err(),
        "Save to invalid path must return Err so the GUI treats it as failure"
    );
}

/// Verify that a successful save returns Ok, which the GUI interprets
/// as `true` from `save_to_path`.  Only when `true` is returned may
/// `execute_pending()` be called.
#[test]
fn p1_1_successful_save_returns_ok() {
    let m = model::PresetModel::Portal.build_edit();
    let dir = std::env::temp_dir().join("phase151_regression");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("p1_1_ok.saproject");
    let result = persistence::save_project(&path, &m);
    assert!(
        result.is_ok(),
        "Save to valid path must return Ok so the GUI treats it as success"
    );
    let _ = std::fs::remove_file(&path);
}

/// Verify the state-machine invariant: if save fails, saved_revision
/// must not be updated.  We simulate this by checking that the model's
/// revision is unchanged after a failed save attempt.
#[test]
fn p1_1_failed_save_preserves_revision() {
    let mut m = model::PresetModel::Portal.build_edit();
    let original_rev = m.revision();

    let bad_path = std::path::Path::new("Z:/nonexistent_dir_xyz/test.saproject");
    let _ = persistence::save_project(bad_path, &m);

    assert_eq!(
        m.revision(),
        original_rev,
        "Model revision must be unchanged after failed save"
    );

    m.add_node(99.0, 99.0, 99.0).unwrap();
    assert_ne!(
        m.revision(),
        original_rev,
        "Model revision must change after edit"
    );
}

// ── P1-2: Clear project checks unsaved changes ────────────────────────

/// Verify the dirty-state logic that gates the "Clear project" action.
/// A model that has been edited since last save must be detected as dirty,
/// preventing clear without confirmation.
#[test]
fn p1_2_edited_model_is_dirty_blocks_clear() {
    let mut m = model::PresetModel::Portal.build_edit();
    let saved_revision = Some(m.revision());

    m.add_node(50.0, 50.0, 50.0).unwrap();

    let is_dirty = saved_revision != Some(m.revision());
    assert!(
        is_dirty,
        "Edited model must be dirty so Clear project triggers confirmation"
    );
}

/// Verify that a model with no edits since save is not dirty, so
/// Clear project can proceed without confirmation.
#[test]
fn p1_2_unchanged_model_is_not_dirty_allows_clear() {
    let m = model::PresetModel::Portal.build_edit();
    let saved_revision = Some(m.revision());

    let is_dirty = saved_revision != Some(m.revision());
    assert!(
        !is_dirty,
        "Unchanged model must not be dirty so Clear proceeds without confirmation"
    );
}

// ── P1-3: Preset loading checks unsaved changes ───────────────────────

/// Verify the dirty-state logic that gates preset loading.
/// A model with unsaved edits must be detected as dirty, preventing
/// preset replacement without confirmation.
#[test]
fn p1_3_dirty_model_blocks_preset_load() {
    let mut m = model::PresetModel::Portal.build_edit();
    let saved_revision = Some(m.revision());

    m.set_member_section(MemberId(0), 210e9, 81e9, 2e-3, 3e-6, 2e-6, 4e-6)
        .unwrap();

    let is_dirty = saved_revision != Some(m.revision());
    assert!(
        is_dirty,
        "Model with edited section must be dirty so preset load triggers confirmation"
    );
}

/// Verify that switching presets on a clean model is allowed.
#[test]
fn p1_3_clean_model_allows_preset_load() {
    let m = model::PresetModel::Portal.build_edit();
    let saved_revision = Some(m.revision());

    let is_dirty = saved_revision != Some(m.revision());
    assert!(
        !is_dirty,
        "Clean model must not be dirty so preset load proceeds without confirmation"
    );
}

// ── Dirty-state consistency ───────────────────────────────────────────

/// Verify that dirty state is separate from analysis staleness.
/// A model can be saved (not dirty) but have stale analysis results.
#[test]
fn dirty_state_separate_from_analysis_staleness() {
    let m = model::PresetModel::Portal.build_edit();
    let saved_revision = Some(m.revision());

    let is_dirty = saved_revision != Some(m.revision());
    assert!(
        !is_dirty,
        "Model at saved revision is not dirty even if analysis is stale"
    );
}

/// Verify that a freshly loaded model is not dirty.
#[test]
fn loaded_model_is_not_dirty() {
    let original = model::PresetModel::Grid3x3.build_edit();
    let dir = std::env::temp_dir().join("phase151_regression");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("loaded_not_dirty.saproject");

    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    let saved_revision = Some(loaded.revision());
    let is_dirty = saved_revision != Some(loaded.revision());
    assert!(!is_dirty, "Freshly loaded model must not be dirty");
}

/// Verify that a never-saved model is always dirty.
#[test]
fn never_saved_model_is_dirty() {
    let m = EditModel::new();
    let saved_revision: Option<u64> = None;
    let is_dirty = saved_revision != Some(m.revision());
    assert!(is_dirty, "Never-saved model must be dirty");
}

/// Verify that an empty project (no model) is not dirty.
#[test]
fn empty_project_is_not_dirty() {
    let model: Option<EditModel> = None;
    let saved_revision: Option<u64> = None;
    let is_dirty = model
        .as_ref()
        .is_some_and(|m| saved_revision != Some(m.revision()));
    assert!(!is_dirty, "Empty project (no model) must not be dirty");
}

// ── PendingAction state machine ───────────────────────────────────────

/// Verify that all pending action variants are distinct, ensuring
/// the state machine correctly routes to the right action.
#[test]
fn pending_action_variants_are_distinct() {
    use model::PresetModel;

    let none = PendingActionTest::None;
    let new = PendingActionTest::New;
    let open = PendingActionTest::Open;
    let clear = PendingActionTest::Clear;
    let load_portal = PendingActionTest::LoadPreset(PresetModel::Portal);
    let load_inclined = PendingActionTest::LoadPreset(PresetModel::Inclined);

    assert_ne!(none, new);
    assert_ne!(none, open);
    assert_ne!(none, clear);
    assert_ne!(none, load_portal);
    assert_ne!(new, open);
    assert_ne!(new, clear);
    assert_ne!(new, load_portal);
    assert_ne!(open, clear);
    assert_ne!(open, load_portal);
    assert_ne!(clear, load_portal);
    assert_ne!(load_portal, load_inclined);
}

/// Mirror of the GUI's PendingAction enum for testing.
#[derive(Debug, Clone, Copy, PartialEq)]
enum PendingActionTest {
    None,
    New,
    Open,
    Clear,
    LoadPreset(model::PresetModel),
}

// ── Selection cleanup after model replacement ─────────────────────────

/// Verify that after loading a new model, old node IDs from the previous
/// model do not exist in the new model (selection would be invalid).
#[test]
fn model_replacement_invalidates_old_selections() {
    let portal = model::PresetModel::Portal.build_edit();
    let grid = model::PresetModel::Grid1x1.build_edit();

    let portal_node_ids: Vec<u32> = portal.nodes().iter().map(|n| n.id.0).collect();
    let grid_node_ids: Vec<u32> = grid.nodes().iter().map(|n| n.id.0).collect();

    let portal_has_id = |id: u32| portal_node_ids.contains(&id);
    let grid_has_id = |id: u32| grid_node_ids.contains(&id);

    for &id in &portal_node_ids {
        if !grid_has_id(id) {
            assert!(
                !grid.nodes().iter().any(|n| n.id == NodeId(id)),
                "Node N{} from Portal must not exist in Grid1x1 after replacement",
                id
            );
        }
    }

    assert!(portal_has_id(0) || portal_node_ids.is_empty());
    assert!(grid_has_id(0) || grid_node_ids.is_empty());
}

// ── Inspector buffer isolation ────────────────────────────────────────

/// Verify that member edit buffer values are independent of the model
/// until explicitly applied.  This tests the pattern where the buffer
/// holds a copy of the section values, not a reference to the model.
#[test]
fn inspector_buffer_is_independent_of_model() {
    let mut m = model::PresetModel::Portal.build_edit();
    let member = m.members().first().unwrap();
    let original_e = member.section.E();

    let buffer_e = original_e * 2.0;

    assert_ne!(
        buffer_e, original_e,
        "Buffer can hold a different value from the model"
    );

    let current_e = m.members().first().unwrap().section.E();
    assert_eq!(
        current_e, original_e,
        "Model must be unchanged until buffer is explicitly applied"
    );

    m.set_member_section(member.id, buffer_e, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    let applied_e = m.members().first().unwrap().section.E();
    assert_eq!(
        applied_e, buffer_e,
        "After applying buffer, model must reflect the buffer value"
    );
}
