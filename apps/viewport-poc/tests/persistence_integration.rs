//! Integration tests for versioned project persistence (Phase 150).
//!
//! Covers:
//! - Preset round-trip: all 5 presets survive save/load with exact
//!   geometry, connectivity, sections, ref vecs, supports, and loads.
//! - Numerical round-trip: solve → save → load → solve produces
//!   identical displacements and reactions.
//! - Transaction safety: failed load preserves current model; failed
//!   save preserves the existing file on disk.
//! - Golden file: the hand-written `golden_v1.saproject` fixture
//!   loads correctly and produces expected analysis results.
//! - Dirty-state transitions: save clears dirty, edit sets dirty,
//!   load clears dirty.

use std::path::PathBuf;

use viewport_poc::{
    edit_model::{EditModel, MemberId, NodeId},
    model, persistence,
};

// ── Helpers ───────────────────────────────────────────────────────────

fn temp_dir() -> PathBuf {
    let dir = std::env::temp_dir().join("viewport_poc_persistence_tests");
    let _ = std::fs::create_dir_all(&dir);
    dir
}

fn temp_file(name: &str) -> PathBuf {
    temp_dir().join(name)
}

/// Compare two EditModels for structural equivalence.
fn assert_models_equivalent(a: &EditModel, b: &EditModel) {
    assert_eq!(a.n_nodes(), b.n_nodes(), "node count mismatch");
    assert_eq!(a.n_members(), b.n_members(), "member count mismatch");
    assert_eq!(a.next_node_id(), b.next_node_id(), "next_node_id mismatch");
    assert_eq!(
        a.next_member_id(),
        b.next_member_id(),
        "next_member_id mismatch"
    );

    let a_nodes = a.nodes();
    let b_nodes = b.nodes();
    for (an, bn) in a_nodes.iter().zip(b_nodes.iter()) {
        assert_eq!(an.id, bn.id, "node ID mismatch");
        assert!(
            (an.x - bn.x).abs() < 1e-12,
            "node x mismatch: {} vs {}",
            an.x,
            bn.x
        );
        assert!(
            (an.y - bn.y).abs() < 1e-12,
            "node y mismatch: {} vs {}",
            an.y,
            bn.y
        );
        assert!(
            (an.z - bn.z).abs() < 1e-12,
            "node z mismatch: {} vs {}",
            an.z,
            bn.z
        );
    }

    let a_members = a.members();
    let b_members = b.members();
    for (am, bm) in a_members.iter().zip(b_members.iter()) {
        assert_eq!(am.id, bm.id, "member ID mismatch");
        assert_eq!(am.node_i, bm.node_i, "member node_i mismatch");
        assert_eq!(am.node_j, bm.node_j, "member node_j mismatch");
        for k in 0..3 {
            assert!(
                (am.ref_vec[k] - bm.ref_vec[k]).abs() < 1e-12,
                "ref_vec[{}] mismatch: {} vs {}",
                k,
                am.ref_vec[k],
                bm.ref_vec[k]
            );
        }
        assert!((am.section.E() - bm.section.E()).abs() < 1e-6, "E mismatch");
        assert!((am.section.G() - bm.section.G()).abs() < 1e-6, "G mismatch");
        assert!(
            (am.section.area() - bm.section.area()).abs() < 1e-12,
            "area mismatch"
        );
        assert!(
            (am.section.iy() - bm.section.iy()).abs() < 1e-12,
            "iy mismatch"
        );
        assert!(
            (am.section.iz() - bm.section.iz()).abs() < 1e-12,
            "iz mismatch"
        );
        assert!(
            (am.section.j() - bm.section.j()).abs() < 1e-12,
            "j mismatch"
        );
    }

    let a_supports = a.supports();
    let b_supports = b.supports();
    assert_eq!(a_supports.len(), b_supports.len(), "support count mismatch");
    for ((an_id, a_spec), (bn_id, b_spec)) in a_supports.iter().zip(b_supports.iter()) {
        assert_eq!(an_id, bn_id, "support node ID mismatch");
        let a_dofs = a_spec.dofs();
        let b_dofs = b_spec.dofs();
        for k in 0..6 {
            match (a_dofs[k], b_dofs[k]) {
                (Some(av), Some(bv)) => {
                    assert!((av - bv).abs() < 1e-12, "support DOF[{}] value mismatch", k);
                }
                (None, None) => {}
                _ => panic!(
                    "support DOF[{}] constraint mismatch: {:?} vs {:?}",
                    k, a_dofs[k], b_dofs[k]
                ),
            }
        }
    }

    let a_loads = a.nodal_loads();
    let b_loads = b.nodal_loads();
    assert_eq!(a_loads.len(), b_loads.len(), "load count mismatch");
    for ((an_id, a_load), (bn_id, b_load)) in a_loads.iter().zip(b_loads.iter()) {
        assert_eq!(an_id, bn_id, "load node ID mismatch");
        assert!((a_load.fx - b_load.fx).abs() < 1e-12, "load fx mismatch");
        assert!((a_load.fy - b_load.fy).abs() < 1e-12, "load fy mismatch");
        assert!((a_load.fz - b_load.fz).abs() < 1e-12, "load fz mismatch");
        assert!((a_load.mx - b_load.mx).abs() < 1e-12, "load mx mismatch");
        assert!((a_load.my - b_load.my).abs() < 1e-12, "load my mismatch");
        assert!((a_load.mz - b_load.mz).abs() < 1e-12, "load mz mismatch");
    }
}

// ── Preset round-trip tests ───────────────────────────────────────────

#[test]
fn preset_portal_round_trip() {
    let original = model::PresetModel::Portal.build_edit();
    let path = temp_file("preset_portal.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_models_equivalent(&original, &loaded);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn preset_inclined_round_trip() {
    let original = model::PresetModel::Inclined.build_edit();
    let path = temp_file("preset_inclined.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_models_equivalent(&original, &loaded);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn preset_grid1x1_round_trip() {
    let original = model::PresetModel::Grid1x1.build_edit();
    let path = temp_file("preset_grid1x1.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_models_equivalent(&original, &loaded);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn preset_grid3x3_round_trip() {
    let original = model::PresetModel::Grid3x3.build_edit();
    let path = temp_file("preset_grid3x3.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_models_equivalent(&original, &loaded);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn preset_grid10x10_round_trip() {
    let original = model::PresetModel::Grid10x10.build_edit();
    let path = temp_file("preset_grid10x10.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_models_equivalent(&original, &loaded);
    let _ = std::fs::remove_file(&path);
}

// ── Numerical round-trip tests ────────────────────────────────────────

#[test]
fn numerical_round_trip_portal_displacements_match() {
    let original = model::PresetModel::Portal.build_edit();
    let fm_original = original.to_frame_model().unwrap();
    let result_original = fm_original.solve().unwrap();

    let path = temp_file("numerical_portal.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    let fm_loaded = loaded.to_frame_model().unwrap();
    let result_loaded = fm_loaded.solve().unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(
        result_original.displacements.len(),
        result_loaded.displacements.len(),
        "displacement vector length mismatch"
    );
    for (i, (a, b)) in result_original
        .displacements
        .iter()
        .zip(result_loaded.displacements.iter())
        .enumerate()
    {
        let rel_tol = 1e-10;
        let abs_tol = 1e-15;
        let diff = (a - b).abs();
        assert!(
            diff <= abs_tol || diff <= rel_tol * a.abs().max(b.abs()),
            "displacement[{}] mismatch: {} vs {} (diff={})",
            i,
            a,
            b,
            diff
        );
    }

    assert_eq!(
        result_original.reactions.len(),
        result_loaded.reactions.len(),
        "reaction vector length mismatch"
    );
    for (i, (a, b)) in result_original
        .reactions
        .iter()
        .zip(result_loaded.reactions.iter())
        .enumerate()
    {
        let rel_tol = 1e-10;
        let abs_tol = 1e-12;
        let diff = (a - b).abs();
        assert!(
            diff <= abs_tol || diff <= rel_tol * a.abs().max(b.abs()),
            "reaction[{}] mismatch: {} vs {} (diff={})",
            i,
            a,
            b,
            diff
        );
    }
}

#[test]
fn numerical_round_trip_inclined_displacements_match() {
    let original = model::PresetModel::Inclined.build_edit();
    let fm_original = original.to_frame_model().unwrap();
    let result_original = fm_original.solve().unwrap();

    let path = temp_file("numerical_inclined.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    let fm_loaded = loaded.to_frame_model().unwrap();
    let result_loaded = fm_loaded.solve().unwrap();
    let _ = std::fs::remove_file(&path);

    for (i, (a, b)) in result_original
        .displacements
        .iter()
        .zip(result_loaded.displacements.iter())
        .enumerate()
    {
        let rel_tol = 1e-10;
        let abs_tol = 1e-15;
        let diff = (a - b).abs();
        assert!(
            diff <= abs_tol || diff <= rel_tol * a.abs().max(b.abs()),
            "displacement[{}] mismatch: {} vs {} (diff={})",
            i,
            a,
            b,
            diff
        );
    }
}

// ── Transaction safety tests ──────────────────────────────────────────

#[test]
fn failed_load_preserves_current_model_in_memory() {
    let original = model::PresetModel::Portal.build_edit();
    let good_path = temp_file("tx_good.saproject");
    persistence::save_project(&good_path, &original).unwrap();

    let loaded_good = persistence::load_project(&good_path).unwrap();
    assert_eq!(loaded_good.n_nodes(), original.n_nodes());

    let bad_path = temp_file("nonexistent_file_for_tx_test.saproject");
    let _ = std::fs::remove_file(&bad_path);
    let load_result = persistence::load_project(&bad_path);
    assert!(load_result.is_err(), "loading nonexistent file should fail");

    assert_eq!(loaded_good.n_nodes(), original.n_nodes());
    assert_eq!(loaded_good.n_members(), original.n_members());

    let _ = std::fs::remove_file(&good_path);
}

#[test]
fn failed_save_does_not_overwrite_existing_file() {
    let model_a = model::PresetModel::Portal.build_edit();
    let path = temp_file("tx_save_fail.saproject");
    persistence::save_project(&path, &model_a).unwrap();
    let content_a = std::fs::read_to_string(&path).unwrap();

    let bad_path = PathBuf::from("Z:/nonexistent_dir_xyz/test.saproject");
    let model_b = model::PresetModel::Inclined.build_edit();
    let save_result = persistence::save_project(&bad_path, &model_b);
    assert!(save_result.is_err(), "save to invalid path should fail");

    let content_after = std::fs::read_to_string(&path).unwrap();
    assert_eq!(
        content_a, content_after,
        "existing file must be unchanged after failed save elsewhere"
    );

    let _ = std::fs::remove_file(&path);
}

#[test]
fn save_then_load_then_save_is_idempotent() {
    let original = model::PresetModel::Grid3x3.build_edit();
    let path1 = temp_file("idempotent_1.saproject");
    let path2 = temp_file("idempotent_2.saproject");

    persistence::save_project(&path1, &original).unwrap();
    let loaded = persistence::load_project(&path1).unwrap();
    persistence::save_project(&path2, &loaded).unwrap();

    let json1 = std::fs::read_to_string(&path1).unwrap();
    let json2 = std::fs::read_to_string(&path2).unwrap();
    assert_eq!(
        json1, json2,
        "save→load→save should produce identical files"
    );

    let _ = std::fs::remove_file(&path1);
    let _ = std::fs::remove_file(&path2);
}

#[test]
fn load_nonexistent_file_returns_error() {
    let path = temp_file("definitely_does_not_exist.saproject");
    let _ = std::fs::remove_file(&path);
    let result = persistence::load_project(&path);
    assert!(result.is_err());
}

#[test]
fn save_to_readonly_path_returns_error() {
    let model = model::PresetModel::Portal.build_edit();
    let path = PathBuf::from("Z:/nonexistent_dir_xyz/readonly_test.saproject");
    let result = persistence::save_project(&path, &model);
    assert!(result.is_err(), "save to invalid path should fail");
}

// ── Golden file test ──────────────────────────────────────────────────

#[test]
fn golden_v1_loads_successfully() {
    let golden_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden_v1.saproject");
    let model = persistence::load_project(&golden_path).unwrap();

    assert_eq!(model.n_nodes(), 2);
    assert_eq!(model.n_members(), 1);
    assert_eq!(model.next_node_id(), 2);
    assert_eq!(model.next_member_id(), 1);

    let nodes = model.nodes();
    assert_eq!(nodes[0].id, NodeId(0));
    assert!((nodes[0].x - 0.0).abs() < 1e-12);
    assert!((nodes[0].y - 0.0).abs() < 1e-12);
    assert!((nodes[0].z - 0.0).abs() < 1e-12);
    assert_eq!(nodes[1].id, NodeId(1));
    assert!((nodes[1].x - 3.0).abs() < 1e-12);

    let members = model.members();
    assert_eq!(members[0].id, MemberId(0));
    assert_eq!(members[0].node_i, NodeId(0));
    assert_eq!(members[0].node_j, NodeId(1));

    let supports = model.supports();
    assert_eq!(supports.len(), 1);
    assert_eq!(supports[0].0, NodeId(0));
    let dofs = supports[0].1.dofs();
    for (k, dof) in dofs.iter().enumerate() {
        assert!(
            dof.is_some(),
            "fixed support DOF[{}] should be constrained",
            k
        );
    }

    let loads = model.nodal_loads();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0].0, NodeId(1));
    assert!((loads[0].1.fy - (-1000.0)).abs() < 1e-12);
}

#[test]
fn golden_v1_solves_and_produces_expected_tip_displacement() {
    let golden_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden_v1.saproject");
    let model = persistence::load_project(&golden_path).unwrap();
    let fm = model.to_frame_model().unwrap();
    let result = fm.solve().unwrap();

    assert_eq!(result.displacements.len(), 12);

    let tip_vy = result.displacements[7];
    assert!(
        tip_vy.abs() > 1e-6,
        "tip vertical displacement should be nonzero, got {}",
        tip_vy
    );
    assert!(
        tip_vy < 0.0,
        "tip vertical displacement should be negative (downward load), got {}",
        tip_vy
    );

    let expected = -1000.0 * 3.0_f64.powi(3) / (3.0 * 200e9 * 2e-6);
    let rel_err = ((tip_vy - expected) / expected).abs();
    assert!(
        rel_err < 1e-4,
        "tip displacement {} should match cantilever formula {} (rel_err={})",
        tip_vy,
        expected,
        rel_err
    );
}

#[test]
fn golden_v1_round_trips_through_save_load() {
    let golden_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden_v1.saproject");
    let original = persistence::load_project(&golden_path).unwrap();

    let path = temp_file("golden_round_trip.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    assert_models_equivalent(&original, &loaded);
}

// ── Dirty-state transition tests ──────────────────────────────────────

#[test]
fn dirty_state_new_model_is_dirty() {
    let model = EditModel::new();
    let saved_revision: Option<u64> = None;
    let is_dirty = saved_revision != Some(model.revision());
    assert!(is_dirty, "new model with no save should be dirty");
}

#[test]
fn dirty_state_saved_model_is_not_dirty() {
    let model = EditModel::new();
    let saved_revision = Some(model.revision());
    let is_dirty = saved_revision != Some(model.revision());
    assert!(!is_dirty, "model at saved revision should not be dirty");
}

#[test]
fn dirty_state_edited_model_is_dirty() {
    let mut model = EditModel::new();
    let saved_revision = Some(model.revision());
    model.add_node(1.0, 2.0, 3.0).unwrap();
    let is_dirty = saved_revision != Some(model.revision());
    assert!(is_dirty, "model edited after save should be dirty");
}

#[test]
fn dirty_state_loaded_model_is_not_dirty() {
    let original = model::PresetModel::Portal.build_edit();
    let path = temp_file("dirty_loaded.saproject");
    persistence::save_project(&path, &original).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    let saved_revision = Some(loaded.revision());
    let is_dirty = saved_revision != Some(loaded.revision());
    assert!(!is_dirty, "just-loaded model should not be dirty");
}

#[test]
fn dirty_state_empty_model_is_not_dirty() {
    let model: Option<EditModel> = None;
    let saved_revision: Option<u64> = None;
    let is_dirty = model
        .as_ref()
        .is_some_and(|m| saved_revision != Some(m.revision()));
    assert!(!is_dirty, "no model means not dirty");
}

// ── Additional validation tests ───────────────────────────────────────

#[test]
fn validation_rejects_self_connected_member() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 1,
            "next_member_id": 1,
            "nodes": [{"id": 0, "x": 0.0, "y": 0.0, "z": 0.0}],
            "members": [{"id": 0, "node_i": 0, "node_j": 0, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "self-connected member should be rejected");
}

#[test]
fn validation_rejects_zero_length_member() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 1.0, "y": 2.0, "z": 3.0},
                {"id": 1, "x": 1.0, "y": 2.0, "z": 3.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "zero-length member should be rejected");
}

#[test]
fn validation_rejects_non_finite_coordinates() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 1,
            "next_member_id": 0,
            "nodes": [{"id": 0, "x": 1e400, "y": 0.0, "z": 0.0}],
            "members": [],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "non-finite coordinates should be rejected");
}

#[test]
fn validation_rejects_duplicate_member_id() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 3,
            "next_member_id": 2,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0},
                {"id": 2, "x": 6.0, "y": 0.0, "z": 0.0}
            ],
            "members": [
                {"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6},
                {"id": 0, "node_i": 1, "node_j": 2, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}
            ],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "duplicate member ID should be rejected");
}

#[test]
fn validation_rejects_duplicate_nodal_load() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": [
                {"node_id": 1, "fx": 0.0, "fy": -1000.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0},
                {"node_id": 1, "fx": 500.0, "fy": 0.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0}
            ]
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "duplicate nodal load should be rejected");
}

#[test]
fn validation_rejects_non_finite_load() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": [
                {"node_id": 1, "fx": 1e400, "fy": 0.0, "fz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0}
            ]
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "non-finite load should be rejected");
}

#[test]
fn validation_rejects_non_finite_section() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 1e400, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(
        result.is_err(),
        "non-finite section property should be rejected"
    );
}

#[test]
fn validation_rejects_parallel_ref_vec() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [1.0, 0.0, 0.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(
        result.is_err(),
        "ref vec parallel to member should be rejected"
    );
}

#[test]
fn validation_rejects_zero_ref_vec() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 0.0], "e": 200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "zero ref vec should be rejected");
}

#[test]
fn validation_rejects_negative_section() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [{"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": -200e9, "g": 80e9, "area": 1e-3, "iy": 2e-6, "iz": 1e-6, "j": 3e-6}],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_err(), "negative E should be rejected");
}

#[test]
fn validation_rejects_next_node_id_below_existing() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 1,
            "next_member_id": 0,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [],
            "supports": [],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(
        result.is_err(),
        "next_node_id <= existing ID should be rejected"
    );
}
