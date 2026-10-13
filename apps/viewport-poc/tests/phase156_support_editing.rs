//! Phase 156: 3D frame support & boundary condition editing.
//!
//! Covers P2-1 regression (ref_vec deserialization validation),
//! support CRUD, DOF mapping, numerical validation (Cases A-E),
//! persistence round-trip, undo/redo, and Phase 153/154 regression.

use structural_analysis::Dof3D;
use viewport_poc::{
    edit_model::{
        EditModel, MemberId, MemberPointLoad3D, MemberUniformLoad3D, NodeId, SupportSpec,
    },
    persistence,
    scene::ViewportScene,
};

const E: f64 = 200e9;
const G: f64 = 80e9;
const AREA: f64 = 1e-3;
const IY: f64 = 2e-6;
const IZ: f64 = 1e-6;
const J: f64 = 3e-6;

fn make_beam(l: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(l, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    (m, a, b, mid)
}

fn make_cantilever(l: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let (mut m, a, b, mid) = make_beam(l);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    (m, a, b, mid)
}

const L_NUM: f64 = 4.0;
const P_NUM: f64 = -1000.0;
const TOL_REL: f64 = 1e-4;
const TOL_ABS: f64 = 1e-6;

// ════════════════════════════════════════════════════════════════════════
// 1. P2-1 regression — ref_vec%_vec deserialization validation
// ════════════════════════════════════════════════════════════════════════

#[test]
fn p2_1_valid_ref_vec_loads_ok() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let m = persistence::deserialize_project(json).unwrap();
    assert_eq!(m.n_members(), 1);
}

#[test]
fn p2_1_zero_ref_vec_rejected() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,0.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn p2_1_nan_ref_vec_rejected() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,"NaN"],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::JsonError(_)));
}

#[test]
fn p2_1_infinity_ref_vec_rejected() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,"Infinity"],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::JsonError(_)));
}

#[test]
fn p2_1_parallel_ref_vec_rejected() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[1.0,0.0,0.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn p2_1_nearly_parallel_ref_vec_rejected() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[1.0,0.0,1e-10],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn p2_1_v1_ref_vec_validation() {
    let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[1.0,0.0,0.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

// ════════════════════════════════════════════════════════════════════════
// 2. Support CRUD
// ════════════════════════════════════════════════════════════════════════

#[test]
fn set_support_fixed() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    assert!(m.has_support(a));
    let spec = m.support(a).unwrap();
    assert!(spec.is_fixed());
    assert_eq!(spec.n_constrained(), 6);
}

#[test]
fn set_support_pinned() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::pinned()).unwrap();
    let spec = m.support(a).unwrap();
    assert!(spec.is_pinned());
    assert_eq!(spec.n_constrained(), 3);
}

#[test]
fn set_support_custom() {
    let (mut m, a, _, _) = make_beam(4.0);
    let spec = SupportSpec::custom([Some(0.0), None, Some(0.0), None, Some(0.0), None]);
    m.set_support(a, spec).unwrap();
    let s = m.support(a).unwrap();
    assert_eq!(s.n_constrained(), 3);
    assert!(s.dof(Dof3D::Ux).is_some());
    assert!(s.dof(Dof3D::Uy).is_none());
    assert!(s.dof(Dof3D::Uz).is_some());
    assert!(s.dof(Dof3D::Rx).is_none());
    assert!(s.dof(Dof3D::Ry).is_some());
    assert!(s.dof(Dof3D::Rz).is_none());
}

#[test]
fn clear_support() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    assert!(m.has_support(a));
    m.clear_support(a).unwrap();
    assert!(!m.has_support(a));
}

#[test]
fn clear_support_without_existing_is_ok() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.clear_support(a).unwrap();
    assert!(!m.has_support(a));
}

#[test]
fn clear_support_nonexistent_node_fails() {
    let (mut m, _, _, _) = make_beam(4.0);
    assert!(m.clear_support(NodeId(99)).is_err());
}

#[test]
fn set_support_nonexistent_node_fails() {
    let (mut m, _, _, _) = make_beam(4.0);
    assert!(m.set_support(NodeId(99), SupportSpec::fixed()).is_err());
}

#[test]
fn set_support_replaces_existing() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(a, SupportSpec::pinned()).unwrap();
    let spec = m.support(a).unwrap();
    assert!(spec.is_pinned());
    assert_eq!(spec.n_constrained(), 3);
}

#[test]
fn supports_list() {
    let (mut m, a, b, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    assert_eq!(m.supports().len(), 2);
}

#[test]
fn delete_node_removes_support() {
    let (mut m, _a, b, _) = make_beam(4.0);
    m.set_support(b, SupportSpec::fixed()).unwrap();
    assert!(m.has_support(b));
    m.delete_member(m.members()[0].id).unwrap();
    m.delete_node(b).unwrap();
    assert!(!m.has_support(b));
}

// ════════════════════════════════════════════════════════════════════════
// 3. Fingerprint & stale
// ════════════════════════════════════════════════════════════════════════

#[test]
fn fingerprint_changes_with_support() {
    let (mut m, a, _, _) = make_beam(4.0);
    let fp0 = m.fingerprint();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    let fp1 = m.fingerprint();
    assert_ne!(fp0, fp1);
}

#[test]
fn fingerprint_differs_for_different_supports() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    let fp_fixed = m.fingerprint();
    m.set_support(a, SupportSpec::pinned()).unwrap();
    let fp_pinned = m.fingerprint();
    assert_ne!(fp_fixed, fp_pinned);
}

#[test]
fn fingerprint_same_for_same_support() {
    let (mut m, a, _, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    let fp1 = m.fingerprint();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    let fp2 = m.fingerprint();
    assert_eq!(fp1, fp2);
}

// ════════════════════════════════════════════════════════════════════════
// 4. Case A — Cantilever (fixed support)
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_a_cantilever_tip_deflection() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let tip_uz = result.displacements[8];
    let analytical = P_NUM * L_NUM.powi(3) / (3.0 * E * IZ);
    assert!(
        (tip_uz - analytical).abs() / analytical.abs() < TOL_REL,
        "tip deflection: FEM {} vs analytical {}",
        tip_uz,
        analytical
    );
}

#[test]
fn case_a_cantilever_reaction() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let rz0 = result.reactions[2];
    assert!(
        (rz0 - (-P_NUM)).abs() < TOL_ABS,
        "reaction = -P = {}, got {}",
        -P_NUM,
        rz0
    );
}

#[test]
fn case_a_cantilever_fixed_end_moment() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let my0 = result.reactions[4];
    let analytical = P_NUM * L_NUM;
    assert!(
        (my0 - analytical).abs() / analytical.abs() < TOL_REL,
        "fixed-end moment: PL = {}, got {}",
        analytical,
        my0
    );
}

#[test]
fn case_a_cantilever_all_reactions_nonzero() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    assert!((result.reactions[2] - (-P_NUM)).abs() < TOL_ABS);
    assert!(result.reactions[0].abs() < TOL_ABS);
    assert!(result.reactions[1].abs() < TOL_ABS);
    assert!(result.reactions[3].abs() < TOL_ABS);
    assert!((result.reactions[4] - P_NUM * L_NUM).abs() < TOL_ABS);
    assert!(result.reactions[5].abs() < TOL_ABS);
}

// ════════════════════════════════════════════════════════════════════════
// 5. Case B — Pinned support (with torsional restraint)
// ════════════════════════════════════════════════════════════════════════

fn make_simply_supported(l: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let (mut m, a, b, mid) = make_beam(l);
    m.set_support(
        a,
        SupportSpec::custom([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]),
    )
    .unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    (m, a, b, mid)
}

#[test]
fn case_b_pinned_reactions() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let rz0 = result.reactions[2];
    let rz1 = result.reactions[8];
    assert!(
        (rz0 - (-P_NUM / 2.0)).abs() < TOL_ABS,
        "reaction A = -P/2 = {}, got {}",
        -P_NUM / 2.0,
        rz0
    );
    assert!(
        (rz1 - (-P_NUM / 2.0)).abs() < TOL_ABS,
        "reaction B = -P/2 = {}, got {}",
        -P_NUM / 2.0,
        rz1
    );
}

#[test]
fn case_b_pinned_rotation_free() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let ry_a = result.displacements[4];
    let ry_b = result.displacements[10];
    assert!(ry_a.abs() > 1e-10, "rotation at A should be non-zero");
    assert!(ry_b.abs() > 1e-10, "rotation at B should be non-zero");
    assert!(
        (ry_a + ry_b).abs() < 1e-10,
        "symmetric load: rotations should be antisymmetric"
    );
}

#[test]
fn case_b_pinned_no_moment_reaction() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let my_b = result.reactions[10];
    assert!(
        my_b.abs() < TOL_ABS,
        "pinned support should have zero moment reaction, got {}",
        my_b
    );
}

#[test]
fn case_b_pinned_support_rotation() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    // For a simply supported beam with midspan point load P:
    // Rotation at supports: |θ| = |P|*L²/(16*E*I)
    let ry_a = result.displacements[4];
    let ry_b = result.displacements[10];
    let rot_mag = P_NUM.abs() * L_NUM.powi(2) / (16.0 * E * IZ);
    assert!(
        (ry_a.abs() - rot_mag).abs() / rot_mag < TOL_REL,
        "rotation magnitude at A: FEM {} vs analytical {}",
        ry_a.abs(),
        rot_mag
    );
    assert!(
        (ry_b.abs() - rot_mag).abs() / rot_mag < TOL_REL,
        "rotation magnitude at B: FEM {} vs analytical {}",
        ry_b.abs(),
        rot_mag
    );
    assert!(
        (ry_a + ry_b).abs() < TOL_ABS,
        "rotations should be antisymmetric: {} + {} = {}",
        ry_a,
        ry_b,
        ry_a + ry_b
    );
}

// ════════════════════════════════════════════════════════════════════════
// 6. Case C — Custom support (DOF mapping)
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_c_custom_support_dof_mapping() {
    let (mut m, a, b, mid) = make_beam(L_NUM);
    m.set_support(
        a,
        SupportSpec::custom([
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.0),
        ]),
    )
    .unwrap();
    m.set_support(
        b,
        SupportSpec::custom([Some(0.0), None, Some(0.0), None, None, None]),
    )
    .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    assert!(
        result.displacements[0].abs() < TOL_ABS,
        "Ux at A should be 0 (fixed)"
    );
    assert!(
        result.displacements[1].abs() < TOL_ABS,
        "Uy at A should be 0 (fixed)"
    );
    assert!(
        result.displacements[2].abs() < TOL_ABS,
        "Uz at A should be 0 (fixed)"
    );
    assert!(
        result.displacements[6].abs() < TOL_ABS,
        "Ux at B should be 0 (constrained)"
    );
    assert!(
        result.displacements[8].abs() < TOL_ABS,
        "Uz at B should be 0 (constrained)"
    );
    let ry_b = result.displacements[10];
    assert!(
        ry_b.abs() > 1e-10,
        "Ry at B should be free (non-zero), got {}",
        ry_b
    );
}

#[test]
fn case_c_custom_only_ux_constrained() {
    let (mut m, a, b, mid) = make_beam(L_NUM);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(
        b,
        SupportSpec::custom([Some(0.0), None, None, None, None, None]),
    )
    .unwrap();
    let f_axial = 5000.0;
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, f_axial, 0.0, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    assert!(result.displacements[6].abs() < TOL_ABS);
    let rx_b = result.reactions[6];
    assert!(
        (rx_b - (-f_axial)).abs() < TOL_ABS,
        "axial reaction at B = -F = {}, got {}",
        -f_axial,
        rx_b
    );
}

// ════════════════════════════════════════════════════════════════════════
// 7. Case D — Unstable model
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_d_no_supports_singular() {
    let (mut m, _a, _b, mid) = make_beam(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve();
    assert!(result.is_err(), "model with no supports should fail");
}

#[test]
fn case_d_insufficient_support_singular() {
    let (mut m, a, _b, mid) = make_beam(L_NUM);
    m.set_support(a, SupportSpec::pinned()).unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve();
    assert!(
        result.is_err(),
        "single pinned support should be insufficient (mechanism)"
    );
}

#[test]
fn case_d_free_floating_member_singular() {
    let (mut m, _a, _b, mid) = make_beam(L_NUM);
    m.set_support(NodeId(0), SupportSpec::free()).unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve();
    assert!(result.is_err(), "free-floating member should fail");
}

// ════════════════════════════════════════════════════════════════════════
// 8. Case E — Undo / Redo
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_e_undo_redo_support() {
    use viewport_poc::edit_history::EditHistory;

    let (m, a, _, _) = make_cantilever(L_NUM);
    let mut history = EditHistory::new(50);
    let mut current = m.clone();

    let m2 = {
        let mut candidate = current.clone();
        candidate.set_support(a, SupportSpec::pinned()).unwrap();
        candidate
    };
    history.record(current.clone());
    current = m2;
    assert!(current.support(a).unwrap().is_pinned());

    current = history.undo(current).unwrap();
    assert!(current.support(a).unwrap().is_fixed());

    current = history.redo(current).unwrap();
    assert!(current.support(a).unwrap().is_pinned());
}

#[test]
fn case_e_stale_after_support_change() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let tip1 = result.displacements[8];
    let fp_after_solve = m.fingerprint();
    m.clear_support(_a).unwrap();
    let fp_after_clear = m.fingerprint();
    assert_ne!(fp_after_solve, fp_after_clear, "fingerprint should change");
    m.set_support(_a, SupportSpec::fixed()).unwrap();
    let result2 = m.to_frame_model().unwrap().solve().unwrap();
    let tip2 = result2.displacements[8];
    assert!(
        (tip1 - tip2).abs() < 1e-10,
        "restoring same support should give same result"
    );
}

// ════════════════════════════════════════════════════════════════════════
// 9. Persistence round-trip
// ════════════════════════════════════════════════════════════════════════

#[test]
fn persistence_round_trip_fixed_support() {
    let (mut m, a, b, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    let fp = m.fingerprint();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.fingerprint(), fp);
    assert!(m2.support(a).unwrap().is_fixed());
    assert!(m2.support(b).unwrap().is_pinned());
}

#[test]
fn persistence_round_trip_custom_support() {
    let (mut m, a, _, _) = make_beam(4.0);
    let spec = SupportSpec::custom([Some(0.0), None, Some(0.0), Some(0.0), None, Some(0.0)]);
    m.set_support(a, spec).unwrap();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    let s = m2.support(a).unwrap();
    assert!(s.dof(Dof3D::Ux).is_some());
    assert!(s.dof(Dof3D::Uy).is_none());
    assert!(s.dof(Dof3D::Uz).is_some());
    assert!(s.dof(Dof3D::Rx).is_some());
    assert!(s.dof(Dof3D::Ry).is_none());
    assert!(s.dof(Dof3D::Rz).is_some());
}

#[test]
fn persistence_round_trip_no_support() {
    let (m, _, _, _) = make_beam(4.0);
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.supports().len(), 0);
}

#[test]
fn persistence_v1_supports_readable() {
    let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[],"supports":[{"node_id":0,"dofs":[0.0,0.0,0.0,0.0,0.0,0.0]},{"node_id":1,"dofs":[0.0,0.0,0.0,null,null,null]}],"nodal_loads":[]}}"#;
    let m = persistence::deserialize_project(json).unwrap();
    assert!(m.support(NodeId(0)).unwrap().is_fixed());
    assert!(m.support(NodeId(1)).unwrap().is_pinned());
}

#[test]
fn persistence_rejects_support_nonexistent_node() {
    let json = r#"{"format_version":3,"model":{"next_node_id":1,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0}],"members":[],"supports":[{"node_id":99,"dofs":[0.0,0.0,0.0,0.0,0.0,0.0]}],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn persistence_rejects_duplicate_support() {
    let json = r#"{"format_version":3,"model":{"next_node_id":1,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0}],"members":[],"supports":[{"node_id":0,"dofs":[0.0,0.0,0.0,0.0,0.0,0.0]},{"node_id":0,"dofs":[0.0,0.0,0.0,null,null,null]}],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn persistence_filesystem_round_trip() {
    let (mut m, a, b, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    let dir = std::env::temp_dir();
    let path = dir.join("test_phase156_support.saproject");
    persistence::save_project(&path, &m).unwrap();
    let m2 = persistence::load_project(&path).unwrap();
    assert!(m2.support(a).unwrap().is_fixed());
    assert!(m2.support(b).unwrap().is_pinned());
    let _ = std::fs::remove_file(&path);
}

// ════════════════════════════════════════════════════════════════════════
// 10. Scene
// ════════════════════════════════════════════════════════════════════════

#[test]
fn scene_includes_supports() {
    let (mut m, a, b, _) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_some());
    assert!(scene.node_supports[1].is_some());
}

#[test]
fn scene_no_supports() {
    let (m, _, _, _) = make_beam(4.0);
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_none());
    assert!(scene.node_supports[1].is_none());
}

// ════════════════════════════════════════════════════════════════════════
// 11. Phase 153/154 regression
// ════════════════════════════════════════════════════════════════════════

#[test]
fn regression_uniform_load_with_supports() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let tip_uz = result.displacements[8];
    let analytical = P_NUM * L_NUM.powi(4) / (8.0 * E * IZ);
    assert!(
        (tip_uz - analytical).abs() / analytical.abs() < TOL_REL,
        "uniform load cantilever: FEM {} vs analytical {}",
        tip_uz,
        analytical
    );
}

#[test]
fn regression_point_load_with_supports() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let tip_uz = result.displacements[8];
    let analytical = P_NUM * L_NUM.powi(3) / (3.0 * E * IZ);
    assert!(
        (tip_uz - analytical).abs() / analytical.abs() < TOL_REL,
        "point load cantilever: FEM {} vs analytical {}",
        tip_uz,
        analytical
    );
}

#[test]
fn regression_combined_loads_with_supports() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, P_NUM, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let tip_uz = result.displacements[8];
    let analytical_uniform = P_NUM * L_NUM.powi(4) / (8.0 * E * IZ);
    let analytical_point = P_NUM * L_NUM.powi(3) / (3.0 * E * IZ);
    let analytical_total = analytical_uniform + analytical_point;
    assert!(
        (tip_uz - analytical_total).abs() / analytical_total.abs() < TOL_REL,
        "combined loads: FEM {} vs analytical {}",
        tip_uz,
        analytical_total
    );
}

#[test]
fn regression_persistence_v3_with_loads_and_supports() {
    let (mut m, a, b, mid) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    let fp = m.fingerprint();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.fingerprint(), fp);
    assert!(m2.support(a).unwrap().is_fixed());
    assert!(m2.support(b).unwrap().is_pinned());
    assert_eq!(m2.member_uniform_load(mid).unwrap().qy, -500.0);
    assert_eq!(m2.member_point_loads_for(mid).len(), 1);
}
