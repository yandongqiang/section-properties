//! Phase 158 — End-to-end state integration tests.
//!
//! These tests exercise the **combined state** of Project, ViewportScene,
//! ResultDisplayState, SelectionState, EditHistory, and persistence through
//! realistic multi-step workflows.  They complement Phase 157 (which focused
//! on analysis-status transitions in isolation) by verifying that the
//! display-layer state stays consistent with the engineering data across
//! edits, solves, undo/redo, and save/load cycles.
//!
//! ## Coverage areas (not overlapping with Phase 157)
//!
//! 1. Scene ↔ model consistency after edits
//! 2. Scene with real FEM displacements
//! 3. Display-state workflow
//! 4. Selection-state workflow
//! 5. Dirty-state (fingerprint) tracking
//! 6. EditHistory capacity & overflow
//! 7. Multi-step E2E workflow
//! 8. Message accumulation
//! 9. Camera fit

use viewport_poc::{
    AppMessage, Camera3D, EditHistory, EditModel, MemberId, MemberPointLoad3D, MemberUniformLoad3D,
    MessageLevel, NodalLoadSpec, NodeId, Project, ResultDisplayState, SelectionState, SupportSpec,
    ViewportScene, app_state::AnalysisStatus, display::ResultViewMode, model::PresetModel,
    persistence,
};

// ── Helpers ───────────────────────────────────────────────────────────

fn cantilever() -> EditModel {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_section(MemberId(0), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: -1000.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    m
}

fn cantilever_with_all_loads() -> EditModel {
    let mut m = cantilever();
    m.set_member_uniform_load(MemberId(0), MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(MemberId(0), MemberPointLoad3D::new(0.5, 0.0, 0.0, -200.0))
        .unwrap();
    m
}

// ══════════════════════════════════════════════════════════════════════
// 1. Scene ↔ Model Consistency
// ══════════════════════════════════════════════════════════════════════

#[test]
fn scene_reflects_node_count() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_nodes(), 2);
    assert_eq!(scene.node_positions.len(), 2);
    assert_eq!(scene.node_ids.len(), 2);
}

#[test]
fn scene_reflects_member_count() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_members(), 1);
    assert_eq!(scene.member_nodes.len(), 1);
    assert_eq!(scene.member_ids.len(), 1);
}

#[test]
fn scene_node_positions_match_model() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_positions[0], [0.0, 0.0, 0.0]);
    assert_eq!(scene.node_positions[1], [3.0, 0.0, 0.0]);
}

#[test]
fn scene_member_connectivity_correct() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    let (ni, nj) = scene.member_nodes[0];
    assert_eq!(ni, 0);
    assert_eq!(nj, 1);
}

#[test]
fn scene_reflects_supports() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_some());
    assert!(scene.node_supports[1].is_none());
    assert!(scene.node_supports[0].unwrap().is_fixed());
}

#[test]
fn scene_reflects_nodal_loads() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_loads[0].is_none());
    assert!(scene.node_loads[1].is_some());
    let load = scene.node_loads[1].unwrap();
    assert!((load.fz - (-1000.0)).abs() < 1e-12);
}

#[test]
fn scene_reflects_member_uniform_loads() {
    let m = cantilever_with_all_loads();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.member_loads[0].is_some());
    let ul = scene.member_loads[0].unwrap();
    assert!((ul.qy - (-500.0)).abs() < 1e-12);
}

#[test]
fn scene_reflects_member_point_loads() {
    let m = cantilever_with_all_loads();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_point_loads[0].len(), 1);
    let pl = &scene.member_point_loads[0][0];
    assert!((pl.xi - 0.5).abs() < 1e-12);
    assert!((pl.fz - (-200.0)).abs() < 1e-12);
}

#[test]
fn scene_reflects_ref_vecs() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_ref_vecs[0], [0.0, 0.0, 1.0]);
}

#[test]
fn scene_stable_ids_match_model_ids() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    for (i, &sid) in scene.node_ids.iter().enumerate() {
        assert_eq!(sid, m.nodes()[i].id);
    }
    for (i, &sid) in scene.member_ids.iter().enumerate() {
        assert_eq!(sid, m.members()[i].id);
    }
}

#[test]
fn scene_bounding_box_correct() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    let (min, max) = scene.bounding_box();
    assert_eq!(min, [0.0, 0.0, 0.0]);
    assert_eq!(max, [3.0, 0.0, 0.0]);
}

#[test]
fn scene_empty_model() {
    let m = EditModel::new();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_nodes(), 0);
    assert_eq!(scene.n_members(), 0);
    let (min, max) = scene.bounding_box();
    assert_eq!(min, [0.0; 3]);
    assert_eq!(max, [0.0; 3]);
}

// ══════════════════════════════════════════════════════════════════════
// 1b. Scene After Edits
// ══════════════════════════════════════════════════════════════════════

#[test]
fn scene_after_add_node() {
    let mut m = cantilever();
    m.add_node(5.0, 0.0, 0.0).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_nodes(), 3);
    assert_eq!(scene.node_positions[2], [5.0, 0.0, 0.0]);
}

#[test]
fn scene_after_delete_node() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let c = m.add_node(6.0, 0.0, 0.0).unwrap();
    let _ = m.add_member(a, b).unwrap();
    let _ = m.add_member(b, c).unwrap();
    m.set_member_section(MemberId(0), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    m.set_member_section(MemberId(1), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(c, SupportSpec::fixed()).unwrap();

    m.delete_member(MemberId(1)).unwrap();
    m.delete_member(MemberId(0)).unwrap();
    m.delete_node(b).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_nodes(), 2);
    assert_eq!(scene.n_members(), 0);
}

#[test]
fn scene_after_add_member() {
    let mut m = cantilever();
    let c = m.add_node(0.0, 3.0, 0.0).unwrap();
    m.add_member(NodeId(0), c).unwrap();
    m.set_member_section(MemberId(1), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_members(), 2);
}

#[test]
fn scene_after_update_node() {
    let mut m = cantilever();
    let b_id = m.nodes()[1].id;
    m.update_node(b_id, 5.0, 0.0, 0.0).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_positions[1], [5.0, 0.0, 0.0]);
}

#[test]
fn scene_after_set_support() {
    let mut m = cantilever();
    let b_id = m.nodes()[1].id;
    m.set_support(b_id, SupportSpec::pinned()).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[1].is_some());
    assert!(scene.node_supports[1].unwrap().is_pinned());
}

#[test]
fn scene_after_clear_support() {
    let mut m = cantilever();
    let a_id = m.nodes()[0].id;
    m.clear_support(a_id).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_none());
}

#[test]
fn scene_after_set_nodal_load() {
    let mut m = cantilever();
    let a_id = m.nodes()[0].id;
    m.set_nodal_load(
        a_id,
        NodalLoadSpec {
            fx: 500.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_loads[0].is_some());
    assert!((scene.node_loads[0].unwrap().fx - 500.0).abs() < 1e-12);
}

#[test]
fn scene_after_clear_member_point_loads() {
    let mut m = cantilever_with_all_loads();
    m.clear_member_point_loads(MemberId(0)).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.member_point_loads[0].is_empty());
}

#[test]
fn scene_index_lookups_consistent() {
    let m = PresetModel::Grid3x3.build_edit();
    let scene = ViewportScene::from_edit_model(&m);
    for node in m.nodes() {
        let idx = scene.node_index(node.id);
        assert!(idx.is_some(), "node {:?} not found in scene", node.id);
        assert_eq!(scene.node_ids[idx.unwrap()], node.id);
    }
    for member in m.members() {
        let idx = scene.member_index(member.id);
        assert!(idx.is_some(), "member {:?} not found in scene", member.id);
        assert_eq!(scene.member_ids[idx.unwrap()], member.id);
    }
}

// ══════════════════════════════════════════════════════════════════════
// 2. Scene with Real FEM Displacements
// ══════════════════════════════════════════════════════════════════════

#[test]
fn scene_deformed_positions_with_real_displacements() {
    let mut p = Project::empty();
    p.load_model("Cantilever".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);

    let disp = &result.displacements;
    assert!(disp.len() >= 12);

    let deformed = scene.deformed_positions(disp, 1.0);
    assert_eq!(deformed.len(), 2);

    assert_eq!(deformed[0], scene.node_positions[0]);

    let dz = disp[6 + 2];
    assert!(dz.abs() > 1e-12, "free-end dz should be non-zero");
    assert!((deformed[1][2] - (0.0 + dz)).abs() < 1e-12);
}

#[test]
fn scene_deformed_zero_scale_is_original() {
    let mut p = Project::empty();
    p.load_model("Cantilever".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);
    let deformed = scene.deformed_positions(&result.displacements, 0.0);
    assert_eq!(deformed, scene.node_positions);
}

#[test]
fn scene_deformed_scale_proportional() {
    let mut p = Project::empty();
    p.load_model("Cantilever".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);
    let disp = &result.displacements;

    let d1 = scene.deformed_positions(disp, 1.0);
    let d2 = scene.deformed_positions(disp, 2.0);

    for i in 0..scene.n_nodes() {
        let orig = scene.node_positions[i];
        let delta1 = d1[i][2] - orig[2];
        let delta2 = d2[i][2] - orig[2];
        assert!((delta2 - 2.0 * delta1).abs() < 1e-12);
    }
}

#[test]
fn scene_deformed_all_presets() {
    for &preset in PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset.name().into(), preset.build_edit());
        p.solve()
            .unwrap_or_else(|e| panic!("{preset:?} solve failed: {e}"));
        let result = p.analysis_result().unwrap();
        let m = p.model.as_ref().unwrap();
        let scene = ViewportScene::from_edit_model(m);
        let deformed = scene.deformed_positions(&result.displacements, 100.0);
        assert_eq!(deformed.len(), scene.n_nodes());
    }
}

// ══════════════════════════════════════════════════════════════════════
// 3. Display-State Workflow
// ══════════════════════════════════════════════════════════════════════

#[test]
fn display_default_is_model_mode() {
    let d = ResultDisplayState::default();
    assert_eq!(d.mode, ResultViewMode::Model);
    assert!(!d.needs_result());
}

#[test]
fn display_mode_switch_to_deformed_after_solve() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();

    let d = ResultDisplayState {
        mode: ResultViewMode::Deformed,
        ..Default::default()
    };
    assert!(d.needs_result());
    assert!(p.analysis_result().is_some());
}

#[test]
fn display_mode_switch_to_deformed_before_solve() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());

    let d = ResultDisplayState {
        mode: ResultViewMode::Deformed,
        ..Default::default()
    };
    assert!(d.needs_result());
    assert!(p.analysis_result().is_none());
}

#[test]
fn display_mode_switch_to_deformed_when_stale() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();

    let id = p.model.as_ref().unwrap().nodes()[1].id;
    p.model
        .as_mut()
        .unwrap()
        .update_node(id, 3.5, 0.0, 0.0)
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);

    let d = ResultDisplayState {
        mode: ResultViewMode::Deformed,
        ..Default::default()
    };
    assert!(d.needs_result());
    assert!(p.analysis_result().is_none());
}

#[test]
fn display_deformation_scale_change() {
    let mut d = ResultDisplayState::default();
    assert!((d.deformation_scale - 100.0).abs() < 1e-15);
    d.deformation_scale = 500.0;
    assert!((d.deformation_scale - 500.0).abs() < 1e-15);
    d.deformation_scale = 0.0;
    assert!(d.deformation_scale.abs() < 1e-15);
}

#[test]
fn display_reset_restores_defaults() {
    let mut d = ResultDisplayState {
        mode: ResultViewMode::Reaction,
        deformation_scale: 999.0,
        show_undeformed: false,
        show_grid: false,
        ..Default::default()
    };
    d.reset();
    assert_eq!(d.mode, ResultViewMode::Model);
    assert!((d.deformation_scale - 100.0).abs() < 1e-15);
    assert!(d.show_undeformed);
    assert!(d.show_grid);
}

#[test]
fn display_all_modes_require_result_except_model() {
    for &mode in &ResultViewMode::ALL {
        let d = ResultDisplayState {
            mode,
            ..Default::default()
        };
        assert_eq!(d.needs_result(), mode.requires_result());
    }
}

#[test]
fn display_overlay_toggles_independent_of_mode() {
    let d = ResultDisplayState {
        mode: ResultViewMode::Deformed,
        show_undeformed: false,
        show_local_axes: false,
        show_grid: false,
        show_global_axes: false,
        show_node_labels: false,
        ..Default::default()
    };
    assert!(!d.show_undeformed);
    assert!(!d.show_local_axes);
    assert!(!d.show_grid);
    assert!(!d.show_global_axes);
    assert!(!d.show_node_labels);
}

// ══════════════════════════════════════════════════════════════════════
// 4. Selection-State Workflow
// ══════════════════════════════════════════════════════════════════════

#[test]
fn selection_default_is_none() {
    assert_eq!(SelectionState::default(), SelectionState::None);
}

#[test]
fn selection_node() {
    let s = SelectionState::Node(NodeId(3));
    assert!(s.is_node());
    assert!(!s.is_member());
    assert_eq!(s.node_id(), Some(NodeId(3)));
    assert_eq!(s.member_id(), None);
}

#[test]
fn selection_member() {
    let s = SelectionState::Member(MemberId(5));
    assert!(!s.is_node());
    assert!(s.is_member());
    assert_eq!(s.node_id(), None);
    assert_eq!(s.member_id(), Some(MemberId(5)));
}

#[test]
fn selection_survives_model_rebuild() {
    let m = cantilever();
    let node_id = m.nodes()[1].id;
    let member_id = m.members()[0].id;

    let s_node = SelectionState::Node(node_id);
    let s_member = SelectionState::Member(member_id);

    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_index(node_id).is_some());
    assert!(scene.member_index(member_id).is_some());

    assert_eq!(s_node.node_id(), Some(node_id));
    assert_eq!(s_member.member_id(), Some(member_id));
}

#[test]
fn selection_clear_to_none() {
    let s = SelectionState::Node(NodeId(0));
    let cleared = SelectionState::None;
    assert_ne!(s, cleared);
}

#[test]
fn selection_stable_id_after_save_load() {
    let m = cantilever();
    let original_id = m.nodes()[1].id;
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();

    let loaded_id = m2.nodes()[1].id;
    assert_eq!(original_id, loaded_id);

    let s = SelectionState::Node(original_id);
    assert_eq!(s.node_id(), Some(loaded_id));
}

// ══════════════════════════════════════════════════════════════════════
// 5. Dirty-State (Fingerprint) Tracking
// ══════════════════════════════════════════════════════════════════════

#[test]
fn dirty_new_model_is_clean() {
    let m = cantilever();
    let saved_fp = Some(m.fingerprint());
    assert_eq!(saved_fp, Some(m.fingerprint()));
}

#[test]
fn dirty_after_node_edit() {
    let mut m = cantilever();
    let fp_before = m.fingerprint();
    let id = m.nodes()[1].id;
    m.update_node(id, 5.0, 0.0, 0.0).unwrap();
    let fp_after = m.fingerprint();
    assert_ne!(fp_before, fp_after);
}

#[test]
fn dirty_after_support_edit() {
    let mut m = cantilever();
    let fp_before = m.fingerprint();
    let id = m.nodes()[1].id;
    m.set_support(id, SupportSpec::pinned()).unwrap();
    let fp_after = m.fingerprint();
    assert_ne!(fp_before, fp_after);
}

#[test]
fn dirty_after_load_edit() {
    let mut m = cantilever();
    let fp_before = m.fingerprint();
    let id = m.nodes()[0].id;
    m.set_nodal_load(
        id,
        NodalLoadSpec {
            fx: 100.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    let fp_after = m.fingerprint();
    assert_ne!(fp_before, fp_after);
}

#[test]
fn dirty_after_section_edit() {
    let mut m = cantilever();
    let fp_before = m.fingerprint();
    m.set_member_section(MemberId(0), 210e9, 81e9, 2e-3, 3e-6, 2e-6, 4e-6)
        .unwrap();
    let fp_after = m.fingerprint();
    assert_ne!(fp_before, fp_after);
}

#[test]
fn dirty_clean_after_undo_to_saved_state() {
    let mut m = cantilever();
    let saved_fp = m.fingerprint();

    let id = m.nodes()[1].id;
    m.update_node(id, 5.0, 0.0, 0.0).unwrap();
    assert_ne!(Some(m.fingerprint()), Some(saved_fp));

    m.update_node(id, 3.0, 0.0, 0.0).unwrap();
    assert_eq!(Some(m.fingerprint()), Some(saved_fp));
}

#[test]
fn dirty_unchanged_by_camera_or_display() {
    let m = cantilever();
    let fp = m.fingerprint();
    let _cam = Camera3D::default();
    let _disp = ResultDisplayState::default();
    assert_eq!(m.fingerprint(), fp);
}

#[test]
fn dirty_preserved_across_save_load() {
    let m = cantilever();
    let fp = m.fingerprint();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.fingerprint(), fp);
}

// ══════════════════════════════════════════════════════════════════════
// 6. EditHistory Capacity & Overflow
// ══════════════════════════════════════════════════════════════════════

fn make_model_with_node(x: f64) -> EditModel {
    let mut m = EditModel::new();
    m.add_node(x, 0.0, 0.0).unwrap();
    m
}

#[test]
fn history_small_capacity_overflow() {
    let mut h = EditHistory::new(3);
    h.record(make_model_with_node(0.0));
    h.record(make_model_with_node(1.0));
    h.record(make_model_with_node(2.0));
    h.record(make_model_with_node(3.0));
    assert_eq!(h.undo_depth(), 3);

    let r1 = h.undo(make_model_with_node(4.0)).unwrap();
    assert_eq!(r1.nodes()[0].x, 3.0);
    let r2 = h.undo(make_model_with_node(4.0)).unwrap();
    assert_eq!(r2.nodes()[0].x, 2.0);
    let r3 = h.undo(make_model_with_node(4.0)).unwrap();
    assert_eq!(r3.nodes()[0].x, 1.0);
    assert!(!h.can_undo());
}

#[test]
fn history_zero_capacity() {
    let mut h = EditHistory::new(0);
    h.record(make_model_with_node(0.0));
    assert!(!h.can_undo());
    assert_eq!(h.undo_depth(), 0);
}

#[test]
fn history_undo_redo_sequence() {
    let mut h = EditHistory::default();
    let m0 = make_model_with_node(0.0);
    let m1 = make_model_with_node(1.0);
    let m2 = make_model_with_node(2.0);

    h.record(m0.clone());
    h.record(m1.clone());

    let restored = h.undo(m2.clone()).unwrap();
    assert_eq!(restored.nodes()[0].x, 1.0);
    assert!(h.can_redo());

    let restored = h.redo(m1.clone()).unwrap();
    assert_eq!(restored.nodes()[0].x, 2.0);
    assert!(!h.can_redo());
    assert!(h.can_undo());
}

#[test]
fn history_record_clears_redo() {
    let mut h = EditHistory::default();
    h.record(make_model_with_node(0.0));
    h.record(make_model_with_node(1.0));
    let _ = h.undo(make_model_with_node(2.0));
    assert!(h.can_redo());

    h.record(make_model_with_node(3.0));
    assert!(!h.can_redo());
}

#[test]
fn history_clear_empties_both() {
    let mut h = EditHistory::default();
    h.record(make_model_with_node(0.0));
    h.record(make_model_with_node(1.0));
    let _ = h.undo(make_model_with_node(2.0));
    h.clear();
    assert!(!h.can_undo());
    assert!(!h.can_redo());
    assert_eq!(h.undo_depth(), 0);
    assert_eq!(h.redo_depth(), 0);
}

#[test]
fn history_capacity_one() {
    let mut h = EditHistory::new(1);
    h.record(make_model_with_node(0.0));
    h.record(make_model_with_node(1.0));
    assert_eq!(h.undo_depth(), 1);
    let r = h.undo(make_model_with_node(2.0)).unwrap();
    assert_eq!(r.nodes()[0].x, 1.0);
    assert!(!h.can_undo());
}

// ══════════════════════════════════════════════════════════════════════
// 7. Multi-Step E2E Workflow
// ══════════════════════════════════════════════════════════════════════

#[test]
fn e2e_load_solve_edit_stale_resolve() {
    let mut p = Project::empty();
    assert!(p.is_empty());

    p.load_model("Cantilever".into(), cantilever());
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    let disp1 = p.analysis_result().unwrap().displacements.clone();

    let id = p.model.as_ref().unwrap().nodes()[1].id;
    p.model
        .as_mut()
        .unwrap()
        .update_node(id, 4.0, 0.0, 0.0)
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
    assert!(p.analysis_result().is_none());

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    let disp2 = p.analysis_result().unwrap().displacements.clone();

    assert_ne!(disp1, disp2);
}

#[test]
fn e2e_solve_undo_solve_redo() {
    let mut p = Project::empty();
    let mut h = EditHistory::default();

    let m0 = cantilever();
    p.load_model("C".into(), m0.clone());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let mut m1 = p.model.as_ref().unwrap().clone();
    let id = m1.nodes()[1].id;
    m1.update_node(id, 4.0, 0.0, 0.0).unwrap();
    h.record(p.model.as_ref().unwrap().clone());
    p.replace_model(m1);
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let current = p.model.as_ref().unwrap().clone();
    let restored = h.undo(current).unwrap();
    p.replace_model(restored);
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    assert_eq!(p.model.as_ref().unwrap().nodes()[1].x, 3.0);

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
}

#[test]
fn e2e_solve_save_load_resolve() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();
    let disp1 = p.analysis_result().unwrap().displacements.clone();

    let json = persistence::serialize_project(p.model.as_ref().unwrap()).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();

    let mut p2 = Project::empty();
    p2.load_model("C".into(), m2);
    assert_eq!(p2.analysis_status(), AnalysisStatus::NeverSolved);
    assert!(p2.analysis_result().is_none());

    p2.solve().unwrap();
    assert_eq!(p2.analysis_status(), AnalysisStatus::Current);
    let disp2 = p2.analysis_result().unwrap().displacements.clone();

    for (a, b) in disp1.iter().zip(disp2.iter()) {
        assert!(
            (a - b).abs() < 1e-10,
            "displacement mismatch after round-trip"
        );
    }
}

#[test]
fn e2e_preset_switch_scene_rebuild() {
    let mut p = Project::empty();

    p.load_model("Portal".into(), PresetModel::Portal.build_edit());
    let scene1 = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let n1 = scene1.n_nodes();

    p.load_model("Grid3x3".into(), PresetModel::Grid3x3.build_edit());
    let scene2 = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let n2 = scene2.n_nodes();

    assert_ne!(n1, n2);
    assert!(n2 > n1);
}

#[test]
fn e2e_edit_solve_edit_solve_disp_changes() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());

    p.solve().unwrap();
    let d1 = p.analysis_result().unwrap().displacements[6 + 2];

    let id = p.model.as_ref().unwrap().nodes()[1].id;
    p.model
        .as_mut()
        .unwrap()
        .set_nodal_load(
            id,
            NodalLoadSpec {
                fx: 0.0,
                fy: 0.0,
                fz: -2000.0,
                mx: 0.0,
                my: 0.0,
                mz: 0.0,
            },
        )
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);

    p.solve().unwrap();
    let d2 = p.analysis_result().unwrap().displacements[6 + 2];

    assert!(
        d2.abs() > d1.abs(),
        "doubling load should increase displacement"
    );
    assert!(
        (d2 / d1 - 2.0).abs() < 1e-6,
        "displacement should scale ~2x"
    );
}

#[test]
fn e2e_failed_then_fix_then_solve_workflow() {
    let mut p = Project::empty();

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_section(MemberId(0), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: -1000.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    p.load_model("Unstable".into(), m);
    p.solve().unwrap_err();
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
    assert!(p.analysis_result().is_none());

    let mut fixed = p.model.as_ref().unwrap().clone();
    fixed.set_support(a, SupportSpec::fixed()).unwrap();
    p.replace_model(fixed);
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    assert!(p.analysis_result().is_some());
}

#[test]
fn e2e_clear_resets_all_state() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();
    assert!(!p.is_empty());
    assert!(p.has_results());

    p.clear();
    assert!(p.is_empty());
    assert!(!p.has_results());
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    assert_eq!(p.name, "Untitled");
}

#[test]
fn e2e_scene_rebuild_after_each_edit() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());

    let s0 = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    assert_eq!(s0.n_nodes(), 2);

    let c = p.model.as_mut().unwrap().add_node(5.0, 0.0, 0.0).unwrap();
    let s1 = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    assert_eq!(s1.n_nodes(), 3);

    p.model.as_mut().unwrap().add_member(NodeId(1), c).unwrap();
    p.model
        .as_mut()
        .unwrap()
        .set_member_section(MemberId(1), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
        .unwrap();
    let s2 = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    assert_eq!(s2.n_members(), 2);
}

// ══════════════════════════════════════════════════════════════════════
// 8. Message Accumulation
// ══════════════════════════════════════════════════════════════════════

#[test]
fn message_info_construction() {
    let m = AppMessage::info("Analysis complete.");
    assert_eq!(m.level, MessageLevel::Info);
    assert_eq!(m.text, "Analysis complete.");
}

#[test]
fn message_error_construction() {
    let m = AppMessage::error("Analysis failed: singular matrix");
    assert_eq!(m.level, MessageLevel::Error);
    assert_eq!(m.text, "Analysis failed: singular matrix");
}

#[test]
fn message_warning_construction() {
    let m = AppMessage::warning("Results are stale");
    assert_eq!(m.level, MessageLevel::Warning);
    assert_eq!(m.text, "Results are stale");
}

#[test]
fn message_accumulation_workflow() {
    let mut messages: Vec<AppMessage> = Vec::new();
    let mut p = Project::empty();

    p.load_model("C".into(), cantilever());
    messages.push(AppMessage::info("Model loaded."));

    match p.solve() {
        Ok(()) => messages.push(AppMessage::info("Analysis complete.")),
        Err(e) => messages.push(AppMessage::error(format!("Analysis failed: {e}"))),
    }
    assert_eq!(messages.len(), 2);
    assert_eq!(messages[1].level, MessageLevel::Info);

    let id = p.model.as_ref().unwrap().nodes()[1].id;
    p.model
        .as_mut()
        .unwrap()
        .update_node(id, 4.0, 0.0, 0.0)
        .unwrap();
    if p.results_are_stale() {
        messages.push(AppMessage::warning(
            "Results are stale — re-solve to update",
        ));
    }
    assert_eq!(messages.len(), 3);
    assert_eq!(messages[2].level, MessageLevel::Warning);

    match p.solve() {
        Ok(()) => messages.push(AppMessage::info("Analysis complete.")),
        Err(e) => messages.push(AppMessage::error(format!("Analysis failed: {e}"))),
    }
    assert_eq!(messages.len(), 4);
    assert_eq!(messages[3].level, MessageLevel::Info);
}

#[test]
fn message_error_on_failed_solve() {
    let mut messages: Vec<AppMessage> = Vec::new();
    let mut p = Project::empty();

    match p.solve() {
        Ok(()) => messages.push(AppMessage::info("Analysis complete.")),
        Err(e) => messages.push(AppMessage::error(format!("Analysis failed: {e}"))),
    }
    assert_eq!(messages.len(), 1);
    assert_eq!(messages[0].level, MessageLevel::Error);
    assert!(messages[0].text.contains("No model"));
}

// ══════════════════════════════════════════════════════════════════════
// 9. Camera Fit
// ══════════════════════════════════════════════════════════════════════

#[test]
fn camera_default_values() {
    let cam = Camera3D::default();
    assert_eq!(cam.target, [0.0, 0.0, 0.0]);
    assert!(cam.distance > 0.0);
    assert!(cam.fov > 0.0);
    assert!(cam.near > 0.0);
    assert!(cam.far > cam.near);
}

#[test]
fn camera_fit_cantilever_bbox() {
    let m = cantilever();
    let scene = ViewportScene::from_edit_model(&m);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);
    assert!(cam.distance > 0.0);
}

#[test]
fn camera_fit_grid_bbox() {
    let m = PresetModel::Grid3x3.build_edit();
    let scene = ViewportScene::from_edit_model(&m);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);
    assert!(cam.distance > 0.0);
}

#[test]
fn camera_fit_empty_bbox() {
    let cam = Camera3D::fit([0.0; 3], [0.0; 3]);
    assert!(cam.distance > 0.0);
}

#[test]
fn camera_fit_large_bbox() {
    let cam = Camera3D::fit([-100.0, -100.0, -100.0], [100.0, 100.0, 100.0]);
    assert!(cam.distance > 0.0);
}

// ══════════════════════════════════════════════════════════════════════
// 10. Persistence Version Compatibility
// ══════════════════════════════════════════════════════════════════════

#[test]
fn persistence_v1_loads_correctly() {
    let json = r#"{
        "format_version": 1,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [
                {"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200000000000.0, "g": 80000000000.0, "area": 0.001, "iy": 0.000002, "iz": 0.000001, "j": 0.000003}
            ],
            "supports": [
                {"node_id": 0, "dofs": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ],
            "nodal_loads": []
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_ok(), "V1 load failed: {:?}", result.err());
    let m = result.unwrap();
    assert_eq!(m.nodes().len(), 2);
    assert_eq!(m.members().len(), 1);
    assert_eq!(m.supports().len(), 1);
    assert_eq!(m.member_uniform_loads().len(), 0);
    assert_eq!(m.member_point_loads().len(), 0);
}

#[test]
fn persistence_v2_loads_correctly() {
    let json = r#"{
        "format_version": 2,
        "model": {
            "next_node_id": 2,
            "next_member_id": 1,
            "nodes": [
                {"id": 0, "x": 0.0, "y": 0.0, "z": 0.0},
                {"id": 1, "x": 3.0, "y": 0.0, "z": 0.0}
            ],
            "members": [
                {"id": 0, "node_i": 0, "node_j": 1, "ref_vec": [0.0, 0.0, 1.0], "e": 200000000000.0, "g": 80000000000.0, "area": 0.001, "iy": 0.000002, "iz": 0.000001, "j": 0.000003}
            ],
            "supports": [
                {"node_id": 0, "dofs": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ],
            "nodal_loads": [],
            "member_uniform_loads": [
                {"member_id": 0, "qx": 0.0, "qy": -500.0, "qz": 0.0}
            ]
        }
    }"#;
    let result = persistence::deserialize_project(json);
    assert!(result.is_ok(), "V2 load failed: {:?}", result.err());
    let m = result.unwrap();
    assert_eq!(m.member_uniform_loads().len(), 1);
    assert_eq!(m.member_point_loads().len(), 0);
}

#[test]
fn persistence_v3_round_trip_all_loads() {
    let m = cantilever_with_all_loads();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();

    assert_eq!(m2.nodes().len(), m.nodes().len());
    assert_eq!(m2.members().len(), m.members().len());
    assert_eq!(m2.supports().len(), m.supports().len());
    assert_eq!(m2.nodal_loads().len(), m.nodal_loads().len());
    assert_eq!(
        m2.member_uniform_loads().len(),
        m.member_uniform_loads().len()
    );
    assert_eq!(m2.member_point_loads().len(), m.member_point_loads().len());

    assert!((m2.member_uniform_loads()[0].1.qy - (-500.0)).abs() < 1e-12);
    assert!((m2.member_point_loads()[0].1.xi - 0.5).abs() < 1e-12);
    assert!((m2.member_point_loads()[0].1.fz - (-200.0)).abs() < 1e-12);
}

#[test]
fn persistence_unsupported_version_rejected() {
    let json = r#"{"format_version": 99, "model": {"next_node_id": 0, "next_member_id": 0, "nodes": [], "members": [], "supports": [], "nodal_loads": []}}"#;
    assert!(persistence::deserialize_project(json).is_err());
}

#[test]
fn persistence_malformed_json_rejected() {
    let json = r#"{"format_version": 1, "model": invalid}"#;
    assert!(persistence::deserialize_project(json).is_err());
}

// ══════════════════════════════════════════════════════════════════════
// 11. Scene ↔ Solve Consistency
// ══════════════════════════════════════════════════════════════════════

#[test]
fn scene_node_count_matches_solver_dofs() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);

    assert_eq!(result.displacements.len(), scene.n_nodes() * 6);
}

#[test]
fn scene_all_presets_dof_consistency() {
    for &preset in PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset.name().into(), preset.build_edit());
        p.solve().unwrap_or_else(|e| panic!("{preset:?}: {e}"));
        let result = p.analysis_result().unwrap();
        let m = p.model.as_ref().unwrap();
        let scene = ViewportScene::from_edit_model(m);
        assert_eq!(
            result.displacements.len(),
            scene.n_nodes() * 6,
            "{preset:?}: DOF mismatch"
        );
    }
}

#[test]
fn scene_deformed_fixed_node_zero_displacement() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);

    let fixed_idx = scene.node_index(m.nodes()[0].id).unwrap();
    let base = fixed_idx * 6;
    let disp = &result.displacements;
    assert!(disp[base].abs() < 1e-10, "fixed node ux should be ~0");
    assert!(disp[base + 1].abs() < 1e-10, "fixed node uy should be ~0");
    assert!(disp[base + 2].abs() < 1e-10, "fixed node uz should be ~0");
}

#[test]
fn scene_deformed_positions_match_displacements() {
    let mut p = Project::empty();
    p.load_model("C".into(), cantilever());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let m = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(m);
    let disp = &result.displacements;

    let deformed = scene.deformed_positions(disp, 1.0);
    for (i, (&orig, &def)) in scene.node_positions.iter().zip(deformed.iter()).enumerate() {
        let base = i * 6;
        let expected_x = orig[0] + disp[base];
        let expected_y = orig[1] + disp[base + 1];
        let expected_z = orig[2] + disp[base + 2];
        assert!((def[0] - expected_x).abs() < 1e-12);
        assert!((def[1] - expected_y).abs() < 1e-12);
        assert!((def[2] - expected_z).abs() < 1e-12);
    }
}
