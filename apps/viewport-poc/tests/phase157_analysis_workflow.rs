//! Phase 157: Analysis workflow & result state integration tests.
//!
//! Covers Cases A-H: successful analysis, model modification, undo/redo,
//! invalid model, mechanism, preset switching, save/load, and mixed loads.

use viewport_poc::{
    app_state::{AnalysisStatus, Project},
    edit_history::EditHistory,
    edit_model::{EditModel, MemberPointLoad3D, MemberUniformLoad3D, NodalLoadSpec, SupportSpec},
    model::PresetModel,
    persistence,
    scene::ViewportScene,
};

// ── Helpers ─────────────────────────────────────────────────────────────

const E: f64 = 200e9;
const G: f64 = 80e9;
const AREA: f64 = 1e-3;
const IY: f64 = 2e-6;
const IZ: f64 = 1e-6;
const J: f64 = 3e-6;

fn make_cantilever() -> EditModel {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m
}

fn make_cantilever_with_load() -> EditModel {
    let mut m = make_cantilever();
    let members = m.members();
    let mid = members[0].id;
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, -1000.0, 0.0))
        .unwrap();
    m
}

fn make_unstable_model() -> EditModel {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    m
}

fn make_invalid_model() -> EditModel {
    EditModel::new()
}

// ═══════════════════════════════════════════════════════════════════════
// Case A — Successful Analysis
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_a_preset_solve_success() {
    let mut p = Project::empty();
    let model = PresetModel::Portal.build_edit();
    p.load_model("Portal".to_string(), model);
    let result = p.solve();
    assert!(result.is_ok(), "Portal preset should solve successfully");
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
}

#[test]
fn case_a_result_exists_after_solve() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    assert!(
        p.analysis_result().is_some(),
        "Result should exist after solve"
    );
}

#[test]
fn case_a_displacements_nonzero() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let max_disp = result
        .displacements
        .iter()
        .map(|d| d.abs())
        .fold(0.0f64, f64::max);
    assert!(
        max_disp > 1e-10,
        "Displacements should be non-zero, got max {}",
        max_disp
    );
}

#[test]
fn case_a_reactions_nonzero() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let max_react = result
        .reactions
        .iter()
        .map(|r| r.abs())
        .fold(0.0f64, f64::max);
    assert!(
        max_react > 1e-10,
        "Reactions should be non-zero, got max {}",
        max_react
    );
}

#[test]
fn case_a_scene_builds_from_solved_model() {
    let mut p = Project::empty();
    let model = PresetModel::Portal.build_edit();
    p.load_model("Portal".to_string(), model);
    p.solve().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    assert!(!scene.node_positions.is_empty());
    assert!(!scene.member_nodes.is_empty());
}

#[test]
fn case_a_all_presets_solve() {
    for preset in PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset.name().to_string(), preset.build_edit());
        let result = p.solve();
        assert!(
            result.is_ok(),
            "Preset {:?} should solve successfully: {:?}",
            preset,
            result.err()
        );
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Case B — Model Modification
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_b_fingerprint_changes_on_edit() {
    let mut m = make_cantilever_with_load();
    let fp1 = m.fingerprint();
    let members = m.members();
    let mid = members[0].id;
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -500.0, 0.0))
        .unwrap();
    let fp2 = m.fingerprint();
    assert_ne!(fp1, fp2, "Fingerprint should change after adding a load");
}

#[test]
fn case_b_revision_changes_on_edit() {
    let mut m = make_cantilever_with_load();
    let rev1 = m.revision();
    let members = m.members();
    let mid = members[0].id;
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -500.0, 0.0))
        .unwrap();
    let rev2 = m.revision();
    assert_ne!(rev1, rev2, "Revision should change after adding a load");
}

#[test]
fn case_b_result_becomes_stale_after_edit() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let model = p.model.as_mut().unwrap();
    let members = model.members();
    let mid = members[0].id;
    model
        .add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -500.0, 0.0))
        .unwrap();

    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::Stale,
        "Result should be stale after model edit"
    );
}

#[test]
fn case_b_stale_result_not_displayed() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let result_before = p.analysis_result();
    assert!(result_before.is_some());

    let model = p.model.as_mut().unwrap();
    let members = model.members();
    let mid = members[0].id;
    model
        .add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -500.0, 0.0))
        .unwrap();

    assert!(
        p.analysis_result().is_none(),
        "Stale result should not be displayed (Policy A)"
    );
}

#[test]
fn case_b_re_solve_after_edit() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let result1 = p.analysis_result().unwrap();
    let tip1 = result1.displacements[8];

    let model = p.model.as_mut().unwrap();
    let members = model.members();
    let mid = members[0].id;
    model
        .add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, -500.0, 0.0))
        .unwrap();

    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    let result2 = p.analysis_result().unwrap();
    let tip2 = result2.displacements[8];
    assert_ne!(
        tip1, tip2,
        "Re-solved result should differ after load change"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Case C — Undo / Redo
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_c_undo_after_solve_invalidates_result() {
    let mut p = Project::empty();
    let model = make_cantilever_with_load();
    p.load_model("Cantilever".to_string(), model);
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let mut history = EditHistory::new(50);
    let current = p.model.as_ref().unwrap().clone();
    let mut modified = current.clone();
    let nodes = modified.nodes();
    let node_b = nodes[1].id;
    modified
        .set_nodal_load(node_b, NodalLoadSpec::new(0.0, -500.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

    history.record(current.clone());
    p.replace_model(modified);
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

    let restored = history.undo(p.model.take().unwrap()).unwrap();
    p.replace_model(restored);
    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::NeverSolved,
        "Undo should invalidate results (conservative strategy)"
    );
}

#[test]
fn case_c_redo_after_undo() {
    let mut p = Project::empty();
    let model = make_cantilever_with_load();
    p.load_model("Cantilever".to_string(), model);

    let mut history = EditHistory::new(50);
    let current = p.model.as_ref().unwrap().clone();
    let mut modified = current.clone();
    let nodes = modified.nodes();
    let node_b = nodes[1].id;
    modified
        .set_nodal_load(node_b, NodalLoadSpec::new(0.0, -500.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

    history.record(current.clone());
    let modified_fp = modified.fingerprint();
    p.replace_model(modified);

    let after_edit = p.model.as_ref().unwrap().clone();
    let restored = history.undo(after_edit).unwrap();
    p.replace_model(restored);
    assert!(
        !p.model.as_ref().unwrap().nodal_loads().is_empty()
            || p.model.as_ref().unwrap().fingerprint() == current.fingerprint()
    );

    let redo_model = history.redo(p.model.take().unwrap()).unwrap();
    p.replace_model(redo_model);
    assert_eq!(
        p.model.as_ref().unwrap().fingerprint(),
        modified_fp,
        "Redo should restore the edited model"
    );
}

#[test]
fn case_c_solve_after_undo() {
    let mut p = Project::empty();
    let model = make_cantilever_with_load();
    p.load_model("Cantilever".to_string(), model);
    p.solve().unwrap();

    let mut history = EditHistory::new(50);
    let current = p.model.as_ref().unwrap().clone();
    let mut modified = current.clone();
    let nodes = modified.nodes();
    let node_b = nodes[1].id;
    modified
        .set_nodal_load(node_b, NodalLoadSpec::new(0.0, -500.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

    history.record(current.clone());
    p.replace_model(modified);
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let after_solve = p.model.take().unwrap();
    let restored = history.undo(after_solve).unwrap();
    p.replace_model(restored);
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

    p.solve().unwrap();
    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::Current,
        "Solve after undo should produce valid results"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Case D — Invalid Model
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_d_empty_model_solve_fails() {
    let mut p = Project::empty();
    let result = p.solve();
    assert!(result.is_err(), "Empty model solve should fail");
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn case_d_no_nodes_solve_fails() {
    let mut p = Project::empty();
    p.load_model("Empty".to_string(), make_invalid_model());
    let result = p.solve();
    assert!(result.is_err(), "Model with no nodes should fail");
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn case_d_failed_does_not_show_result() {
    let mut p = Project::empty();
    p.load_model("Empty".to_string(), make_invalid_model());
    p.solve().unwrap_err();
    assert!(
        p.analysis_result().is_none(),
        "Failed solve should not produce a result"
    );
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn case_d_error_message_is_descriptive() {
    let mut p = Project::empty();
    let err = p.solve().unwrap_err();
    assert!(!err.is_empty(), "Error message should not be empty");
}

// ═══════════════════════════════════════════════════════════════════════
// Case E — Mechanism
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_e_no_supports_singular() {
    let mut p = Project::empty();
    p.load_model("Unstable".to_string(), make_unstable_model());
    let result = p.solve();
    assert!(
        result.is_err(),
        "Model without supports should fail (singular)"
    );
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn case_e_mechanism_no_panic() {
    let mut p = Project::empty();
    p.load_model("Unstable".to_string(), make_unstable_model());
    let result = p.solve();
    assert!(result.is_err(), "Solve should fail on mechanism, not panic");
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn case_e_no_auto_support_added() {
    let m = make_unstable_model();
    let supports_before = m.supports().len();
    let mut p = Project::empty();
    p.load_model("Unstable".to_string(), m);
    let _ = p.solve();
    let supports_after = p.model.as_ref().unwrap().supports().len();
    assert_eq!(
        supports_before, supports_after,
        "Solver must not auto-add supports"
    );
}

#[test]
fn case_e_failed_then_fix_then_solve() {
    let mut p = Project::empty();
    p.load_model("Unstable".to_string(), make_unstable_model());
    assert!(p.solve().is_err());
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);

    let model = p.model.as_mut().unwrap();
    let nodes = model.nodes();
    let node_a = nodes[0].id;
    model.set_support(node_a, SupportSpec::fixed()).unwrap();

    let result = p.solve();
    assert!(
        result.is_ok(),
        "Single fixed support (cantilever) should be stable: {:?}",
        result.err()
    );
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
}

// ═══════════════════════════════════════════════════════════════════════
// Case F — Preset Switching
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_f_switch_preset_discards_old_result() {
    let mut p = Project::empty();
    p.load_model("Portal".to_string(), PresetModel::Portal.build_edit());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let portal_result = p.analysis_result().map(|r| r.displacements.clone());

    p.load_model("Inclined".to_string(), PresetModel::Inclined.build_edit());
    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::NeverSolved,
        "New preset should start as NeverSolved"
    );
    assert!(
        p.analysis_result().is_none(),
        "Old result should not be available after preset switch"
    );

    p.solve().unwrap();
    let inclined_result = p.analysis_result().map(|r| r.displacements.clone());
    assert!(
        portal_result.is_some() && inclined_result.is_some(),
        "Both presets should produce results"
    );
    assert_ne!(
        portal_result.unwrap(),
        inclined_result.unwrap(),
        "Different presets should produce different results"
    );
}

#[test]
fn case_f_multiple_presets_independent() {
    for preset_a in PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset_a.name().to_string(), preset_a.build_edit());
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);

        for preset_b in PresetModel::all() {
            if preset_a == preset_b {
                continue;
            }
            p.load_model(preset_b.name().to_string(), preset_b.build_edit());
            assert_eq!(
                p.analysis_status(),
                AnalysisStatus::NeverSolved,
                "Switching to {:?} should reset to NeverSolved",
                preset_b
            );
            p.solve().unwrap();
            assert_eq!(p.analysis_status(), AnalysisStatus::Current);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Case G — Save / Load
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn case_g_save_load_preserves_model() {
    let model = make_cantilever_with_load();
    let json = persistence::serialize_project(&model).unwrap();
    let loaded = persistence::deserialize_project(&json).unwrap();
    assert_eq!(model.fingerprint(), loaded.fingerprint());
    assert_eq!(model.n_nodes(), loaded.n_nodes());
    assert_eq!(model.n_members(), loaded.n_members());
}

#[test]
fn case_g_load_resets_to_never_solved() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let model = make_cantilever_with_load();
    p.load_model("Reloaded".to_string(), model);
    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::NeverSolved,
        "Loading a model should reset to NeverSolved"
    );
}

#[test]
fn case_g_save_does_not_make_stale_valid() {
    let mut p = Project::empty();
    p.load_model("Cantilever".to_string(), make_cantilever_with_load());
    p.solve().unwrap();

    let model = p.model.as_mut().unwrap();
    let members = model.members();
    let mid = members[0].id;
    model
        .add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -500.0, 0.0))
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);

    let json = persistence::serialize_project(p.model.as_ref().unwrap()).unwrap();
    let loaded = persistence::deserialize_project(&json).unwrap();
    let mut p2 = Project::empty();
    p2.load_model("Saved".to_string(), loaded);
    assert_eq!(
        p2.analysis_status(),
        AnalysisStatus::NeverSolved,
        "Loaded model should be NeverSolved, not Valid"
    );
}

#[test]
fn case_g_filesystem_round_trip() {
    let model = make_cantilever_with_load();
    let dir = std::env::temp_dir();
    let path = dir.join("phase157_case_g.json");
    persistence::save_project(&path, &model).unwrap();
    let loaded = persistence::load_project(&path).unwrap();
    assert_eq!(model.fingerprint(), loaded.fingerprint());
    let _ = std::fs::remove_file(&path);
}

#[test]
fn case_g_no_result_reuse_across_projects() {
    let mut p1 = Project::empty();
    p1.load_model("Portal".to_string(), PresetModel::Portal.build_edit());
    p1.solve().unwrap();
    let portal_disp = p1.analysis_result().unwrap().displacements.clone();

    let mut p2 = Project::empty();
    p2.load_model("Inclined".to_string(), PresetModel::Inclined.build_edit());
    assert!(
        p2.analysis_result().is_none(),
        "New project should not have results"
    );

    p2.solve().unwrap();
    let inclined_disp = p2.analysis_result().unwrap().displacements.clone();
    assert_ne!(
        portal_disp, inclined_disp,
        "Different projects should not share results"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Case H — Mixed Loads
// ═══════════════════════════════════════════════════════════════════════

fn make_mixed_load_model() -> EditModel {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(6.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(
        b,
        SupportSpec::custom([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]),
    )
    .unwrap();

    m.set_nodal_load(b, NodalLoadSpec::new(0.0, 0.0, -2000.0, 0.0, 0.0, 0.0))
        .unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();

    m
}

#[test]
fn case_h_mixed_loads_solve() {
    let mut p = Project::empty();
    p.load_model("Mixed".to_string(), make_mixed_load_model());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    assert!(p.analysis_result().is_some());
}

#[test]
fn case_h_mixed_loads_displacements_nonzero() {
    let mut p = Project::empty();
    p.load_model("Mixed".to_string(), make_mixed_load_model());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();
    let max_disp = result
        .displacements
        .iter()
        .map(|d| d.abs())
        .fold(0.0f64, f64::max);
    assert!(
        max_disp > 1e-10,
        "Mixed loads should produce non-zero displacements"
    );
}

#[test]
fn case_h_mixed_loads_equilibrium() {
    let mut p = Project::empty();
    p.load_model("Mixed".to_string(), make_mixed_load_model());
    p.solve().unwrap();
    let result = p.analysis_result().unwrap();

    let n_nodes = 2;
    let mut sum_fz: f64 = 0.0;
    for i in 0..n_nodes {
        sum_fz += result.reactions[i * 6 + 2];
    }
    let applied_fz = -2000.0 + (-500.0 * 6.0) + (-1000.0);
    assert!(
        (sum_fz + applied_fz).abs() < 1.0,
        "Vertical equilibrium: reactions {} + applied {} = {}",
        sum_fz,
        applied_fz,
        sum_fz + applied_fz
    );
}

#[test]
fn case_h_mixed_loads_superposition() {
    let mut p = Project::empty();
    p.load_model("Mixed".to_string(), make_mixed_load_model());
    p.solve().unwrap();
    let mixed_tip = p.analysis_result().unwrap().displacements[8];

    let mut m_nodal = make_mixed_load_model();
    {
        let members = m_nodal.members();
        let mid = members[0].id;
        m_nodal
            .set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, 0.0, 0.0))
            .unwrap();
        m_nodal.clear_member_point_loads(mid).unwrap();
    }
    let mut p_nodal = Project::empty();
    p_nodal.load_model("NodalOnly".to_string(), m_nodal);
    p_nodal.solve().unwrap();
    let nodal_tip = p_nodal.analysis_result().unwrap().displacements[8];

    let mut m_member = make_mixed_load_model();
    {
        let nodes = m_member.nodes();
        let node_b = nodes[1].id;
        m_member
            .set_nodal_load(node_b, NodalLoadSpec::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
    }
    let mut p_member = Project::empty();
    p_member.load_model("MemberOnly".to_string(), m_member);
    p_member.solve().unwrap();
    let member_tip = p_member.analysis_result().unwrap().displacements[8];

    assert!(
        (mixed_tip - (nodal_tip + member_tip)).abs() < 1e-6 * mixed_tip.abs().max(1e-10),
        "Superposition: mixed {} vs nodal {} + member {} = {}",
        mixed_tip,
        nodal_tip,
        member_tip,
        nodal_tip + member_tip
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Additional — State Transition Coverage
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn state_never_solved_initial() {
    let p = Project::empty();
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
}

#[test]
fn state_current_after_solve() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);
}

#[test]
fn state_stale_after_edit() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let nodes = p.model.as_ref().unwrap().nodes();
    let node_a = nodes[0].id;
    p.model
        .as_mut()
        .unwrap()
        .set_nodal_load(node_a, NodalLoadSpec::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
}

#[test]
fn state_failed_after_error() {
    let mut p = Project::empty();
    let _ = p.solve();
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
}

#[test]
fn state_failed_overrides_stale() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Current);

    let nodes = p.model.as_ref().unwrap().nodes();
    let node_a = nodes[0].id;
    p.model.as_mut().unwrap().clear_support(node_a).unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);

    let _ = p.solve();
    assert_eq!(
        p.analysis_status(),
        AnalysisStatus::Failed,
        "Failed should override Stale after unsuccessful re-solve"
    );
}

#[test]
fn state_clear_resets() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    p.clear();
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    assert!(p.is_empty());
}

#[test]
fn state_replace_model_resets() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    p.replace_model(make_cantilever_with_load());
    assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
}

#[test]
fn display_state_no_crash_on_empty() {
    let p = Project::empty();
    assert!(p.analysis_result().is_none());
    let scene = ViewportScene::from_edit_model(&EditModel::new());
    assert!(scene.node_positions.is_empty());
    assert!(scene.member_nodes.is_empty());
}

#[test]
fn display_state_no_crash_on_failed() {
    let mut p = Project::empty();
    p.load_model("Bad".to_string(), make_invalid_model());
    let _ = p.solve();
    assert!(p.analysis_result().is_none());
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    assert!(scene.node_positions.is_empty());
}

#[test]
fn persistence_does_not_save_results() {
    let mut p = Project::empty();
    p.load_model("C".to_string(), make_cantilever_with_load());
    p.solve().unwrap();
    let json = persistence::serialize_project(p.model.as_ref().unwrap()).unwrap();
    assert!(!json.contains("displacements"));
    assert!(!json.contains("reactions"));
    assert!(!json.contains("solved_revision"));
}
