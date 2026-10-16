//! Phase 154: 3D frame member point loads (concentrated forces).
//!
//! Covers data model, API, validation, fingerprint, to_frame_model,
//! delete_member cleanup, multiple loads per member, persistence V3,
//! numerical correctness (analytical comparison + equilibrium), and
//! Phase 153 regression.
//!
//! Local coordinate system for a beam along global x with ref_vec [0,0,1]:
//!   local x = global x  → DOF 0,6 (trans), 3,9 (rot)
//!   local y = global z  → DOF 2,8 (trans), 5,11 (rot)
//!   local z = -global y → DOF 1,7 (trans, negated), 4,10 (rot, negated)

use viewport_poc::{
    edit_model::{
        EditError, EditModel, MemberId, MemberPointLoad3D, MemberUniformLoad3D, NodeId, SupportSpec,
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

fn make_beam(L: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(L, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    (m, a, b, mid)
}

fn make_fixed_fixed_beam(L: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let (mut m, a, b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    (m, a, b, mid)
}

fn make_cantilever(L: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let (mut m, a, b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    (m, a, b, mid)
}

fn make_simply_supported(L: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let (mut m, a, b, mid) = make_beam(L);
    m.set_support(
        a,
        SupportSpec::custom([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]),
    )
    .unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    (m, a, b, mid)
}

// ════════════════════════════════════════════════════════════════════════
// 1. Data model tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn point_load_construction() {
    let pl = MemberPointLoad3D::new(0.5, 100.0, -200.0, 300.0);
    assert_eq!(pl.xi, 0.5);
    assert_eq!(pl.fx, 100.0);
    assert_eq!(pl.fy, -200.0);
    assert_eq!(pl.fz, 300.0);
}

#[test]
fn point_load_zero() {
    let z = MemberPointLoad3D::zero();
    assert_eq!(z.xi, 0.5);
    assert!(z.is_zero());
}

#[test]
fn point_load_is_zero() {
    assert!(MemberPointLoad3D::new(0.3, 0.0, 0.0, 0.0).is_zero());
    assert!(!MemberPointLoad3D::new(0.3, 1.0, 0.0, 0.0).is_zero());
}

#[test]
fn point_load_is_finite() {
    assert!(MemberPointLoad3D::new(0.5, 1.0, 2.0, 3.0).is_finite());
    assert!(!MemberPointLoad3D::new(f64::NAN, 1.0, 0.0, 0.0).is_finite());
    assert!(!MemberPointLoad3D::new(0.5, f64::INFINITY, 0.0, 0.0).is_finite());
}

#[test]
fn point_load_magnitude() {
    let pl = MemberPointLoad3D::new(0.5, 3.0, 4.0, 0.0);
    assert!((pl.magnitude() - 5.0).abs() < 1e-15);
}

// ════════════════════════════════════════════════════════════════════════
// 2. API tests — add / query / update / remove / clear
// ════════════════════════════════════════════════════════════════════════

#[test]
fn add_point_load_success() {
    let (mut m, _, _, mid) = make_beam(4.0);
    let pl = MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0);
    m.add_member_point_load(mid, pl).unwrap();
    let loads = m.member_point_loads_for(mid);
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0], pl);
    assert!(m.has_member_point_loads(mid));
}

#[test]
fn add_multiple_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.75, 0.0, 0.0, -300.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 100.0, 0.0, 0.0))
        .unwrap();
    let loads = m.member_point_loads_for(mid);
    assert_eq!(loads.len(), 3);
    assert_eq!(loads[0].xi, 0.25);
    assert_eq!(loads[1].xi, 0.75);
    assert_eq!(loads[2].xi, 0.5);
}

#[test]
fn add_point_load_nonexistent_member() {
    let mut m = EditModel::new();
    let err = m
        .add_member_point_load(MemberId(99), MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap_err();
    assert_eq!(err, EditError::MemberNotFound(MemberId(99)));
}

#[test]
fn add_point_load_rejects_nan() {
    let (mut m, _, _, mid) = make_beam(4.0);
    let err = m
        .add_member_point_load(mid, MemberPointLoad3D::new(f64::NAN, 0.0, 0.0, 0.0))
        .unwrap_err();
    assert_eq!(err, EditError::NonFiniteLoad);
}

#[test]
fn add_point_load_rejects_xi_out_of_range() {
    let (mut m, _, _, mid) = make_beam(4.0);
    let err = m
        .add_member_point_load(mid, MemberPointLoad3D::new(-0.1, 0.0, 0.0, 0.0))
        .unwrap_err();
    assert_eq!(err, EditError::InvalidLoadPosition);
    let err = m
        .add_member_point_load(mid, MemberPointLoad3D::new(1.1, 0.0, 0.0, 0.0))
        .unwrap_err();
    assert_eq!(err, EditError::InvalidLoadPosition);
}

#[test]
fn add_point_load_accepts_boundary_xi() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.0, 0.0, -1000.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, -2000.0, 0.0))
        .unwrap();
    assert_eq!(m.member_point_loads_for(mid).len(), 2);
}

#[test]
fn remove_point_load_by_index() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.75, 0.0, -300.0, 0.0))
        .unwrap();
    m.remove_member_point_load(mid, 0).unwrap();
    let loads = m.member_point_loads_for(mid);
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0].xi, 0.75);
}

#[test]
fn remove_point_load_invalid_index() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    let err = m.remove_member_point_load(mid, 5).unwrap_err();
    assert_eq!(err, EditError::MemberNotFound(mid));
}

#[test]
fn update_point_load() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m.update_member_point_load(mid, 0, MemberPointLoad3D::new(0.3, 0.0, -500.0, 0.0))
        .unwrap();
    let loads = m.member_point_loads_for(mid);
    assert_eq!(loads[0].xi, 0.3);
    assert_eq!(loads[0].fy, -500.0);
}

#[test]
fn clear_all_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.75, 0.0, -300.0, 0.0))
        .unwrap();
    m.clear_member_point_loads(mid).unwrap();
    assert!(!m.has_member_point_loads(mid));
    assert_eq!(m.member_point_loads_for(mid).len(), 0);
}

#[test]
fn clear_point_loads_when_empty_is_noop() {
    let (mut m, _, _, mid) = make_beam(4.0);
    let rev_before = m.revision();
    m.clear_member_point_loads(mid).unwrap();
    assert_eq!(m.revision(), rev_before);
}

// ════════════════════════════════════════════════════════════════════════
// 3. Fingerprint tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn fingerprint_changes_with_point_load() {
    let (mut m, _, _, mid) = make_beam(4.0);
    let fp0 = m.fingerprint();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    let fp1 = m.fingerprint();
    assert_ne!(fp0, fp1);
}

#[test]
fn fingerprint_same_for_same_point_loads() {
    let (mut m1, _, _, mid1) = make_beam(4.0);
    let (mut m2, _, _, mid2) = make_beam(4.0);
    m1.add_member_point_load(mid1, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m2.add_member_point_load(mid2, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    assert_eq!(m1.fingerprint(), m2.fingerprint());
}

#[test]
fn fingerprint_differs_for_different_xi() {
    let (mut m1, _, _, mid1) = make_beam(4.0);
    let (mut m2, _, _, mid2) = make_beam(4.0);
    m1.add_member_point_load(mid1, MemberPointLoad3D::new(0.25, 0.0, -1000.0, 0.0))
        .unwrap();
    m2.add_member_point_load(mid2, MemberPointLoad3D::new(0.75, 0.0, -1000.0, 0.0))
        .unwrap();
    assert_ne!(m1.fingerprint(), m2.fingerprint());
}

// ════════════════════════════════════════════════════════════════════════
// 4. to_frame_model tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn to_frame_model_with_point_load_solves() {
    let (mut m, _, _, mid) = make_fixed_fixed_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    let result = fm.solve();
    assert!(result.is_ok(), "solve should succeed: {:?}", result.err());
}

#[test]
fn to_frame_model_transfers_multiple_point_loads() {
    let (mut m, _, _, mid) = make_fixed_fixed_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.75, 0.0, 0.0, -300.0))
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().unwrap();
    assert!(result.displacements.iter().all(|d| d.is_finite()));
}

// ════════════════════════════════════════════════════════════════════════
// 5. delete_member cleanup
// ════════════════════════════════════════════════════════════════════════

#[test]
fn delete_member_removes_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.3, 100.0, 0.0, 0.0))
        .unwrap();
    assert!(m.has_member_point_loads(mid));
    m.delete_member(mid).unwrap();
    assert_eq!(m.member_point_loads().len(), 0);
}

#[test]
fn delete_member_removes_both_uniform_and_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m.delete_member(mid).unwrap();
    assert_eq!(m.member_uniform_loads().len(), 0);
    assert_eq!(m.member_point_loads().len(), 0);
}

// ════════════════════════════════════════════════════════════════════════
// 6. Scene tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn scene_includes_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 100.0, 0.0, 0.0))
        .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_point_loads.len(), 1);
    assert_eq!(scene.member_point_loads[0].len(), 2);
}

#[test]
fn scene_empty_point_loads() {
    let (m, _, _, _) = make_beam(4.0);
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_point_loads.len(), 1);
    assert!(scene.member_point_loads[0].is_empty());
}

// ════════════════════════════════════════════════════════════════════════
// 7. Persistence V3 tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn v3_round_trip_point_loads() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 100.0, -200.0, 300.0))
        .unwrap();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    let loads = m2.member_point_loads_for(mid);
    assert_eq!(loads.len(), 2);
    assert_eq!(loads[0], MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0));
    assert_eq!(loads[1], MemberPointLoad3D::new(0.25, 100.0, -200.0, 300.0));
}

#[test]
fn v4_format_version_in_json() {
    let m = EditModel::new();
    let json = persistence::serialize_project(&m).unwrap();
    assert!(
        json.contains("\"format_version\": 4"),
        "should contain format_version 4, got: {json}"
    );
}

#[test]
fn v1_file_still_readable() {
    let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
    let m = persistence::deserialize_project(json).unwrap();
    assert_eq!(m.n_nodes(), 2);
    assert_eq!(m.member_point_loads().len(), 0);
}

#[test]
fn v2_file_still_readable() {
    let json = r#"{"format_version":2,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[{"member_id":0,"qx":0.0,"qy":-500.0,"qz":0.0}]}}"#;
    let m = persistence::deserialize_project(json).unwrap();
    assert_eq!(m.n_nodes(), 2);
    assert_eq!(m.member_point_loads().len(), 0);
    let ul = m.member_uniform_load(MemberId(0)).unwrap();
    assert_eq!(ul.qy, -500.0);
}

#[test]
fn v3_rejects_xi_out_of_range() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[{"member_id":0,"xi":1.5,"fx":0.0,"fy":-1000.0,"fz":0.0}]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn v3_rejects_nonexistent_member() {
    let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":4.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[{"member_id":99,"xi":0.5,"fx":0.0,"fy":-1000.0,"fz":0.0}]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn v3_round_trip_with_both_load_types() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1000.0, 0.0))
        .unwrap();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.member_uniform_load(mid).unwrap().qy, -500.0);
    assert_eq!(m2.member_point_loads_for(mid).len(), 1);
    assert_eq!(m2.member_point_loads_for(mid)[0].fy, -1000.0);
}

#[test]
fn v3_filesystem_round_trip() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.3, 100.0, -200.0, 300.0))
        .unwrap();
    let dir = std::env::temp_dir();
    let path = dir.join("test_phase154_point_loads.saproject");
    persistence::save_project(&path, &m).unwrap();
    let m2 = persistence::load_project(&path).unwrap();
    let loads = m2.member_point_loads_for(mid);
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0].xi, 0.3);
    assert_eq!(loads[0].fx, 100.0);
    assert_eq!(loads[0].fy, -200.0);
    assert_eq!(loads[0].fz, 300.0);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn unknown_version_rejected() {
    let json = r#"{"format_version":99,"model":{"next_node_id":0,"next_member_id":0,"nodes":[],"members":[],"supports":[],"nodal_loads":[]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(
        err,
        persistence::PersistenceError::UnsupportedVersion { .. }
    ));
}

// ════════════════════════════════════════════════════════════════════════
// 8. Numerical tests — analytical comparison & equilibrium
//
// For beam along global x, ref_vec [0,0,1]:
//   local y = global z  → check DOF 2,8 (translations), 5,11 (rotations)
//   local z = -global y → check DOF 1,7 (translations, negated), 4,10 (rot, negated)
//   Bending: local y uses Iz, local z uses Iy
// ════════════════════════════════════════════════════════════════════════

const L_NUM: f64 = 4.0;
const P_NUM: f64 = -1000.0;
const TOL_REL: f64 = 1e-4;
const TOL_ABS: f64 = 1e-6;

#[test]
fn case_a_fixed_fixed_midspan_y_reaction_equilibrium() {
    let (mut m, _a, _b, mid) = make_fixed_fixed_beam(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let rz0 = result.reactions[2];
    let rz1 = result.reactions[8];
    let total = rz0 + rz1;
    assert!(
        (total - (-P_NUM)).abs() < TOL_ABS,
        "sum of z-reactions should be -P = {}, got {}",
        -P_NUM,
        total
    );
    assert!(
        (rz0 - (-P_NUM / 2.0)).abs() < TOL_ABS,
        "symmetric: each reaction = -P/2 = {}, got {}",
        -P_NUM / 2.0,
        rz0
    );
}

#[test]
fn case_a_fixed_fixed_midspan_y_moment_equilibrium() {
    let (mut m, _a, _b, mid) = make_fixed_fixed_beam(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let rz0 = result.reactions[2];
    let mz0 = result.reactions[5];
    let rz1 = result.reactions[8];
    let mz1 = result.reactions[11];

    let moment_about_i = mz0 + rz1 * L_NUM + mz1;
    let applied_moment = -P_NUM * (L_NUM / 2.0);
    assert!(
        (moment_about_i - applied_moment).abs() < 1e-3,
        "moment equilibrium: reactions {} vs applied {}",
        moment_about_i,
        applied_moment
    );
}

#[test]
fn case_b_cantilever_tip_load_y_deflection() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_uz = result.displacements[8];
    let analytical = P_NUM * L_NUM.powi(3) / (3.0 * E * IZ);
    assert!(
        (tip_uz - analytical).abs() / analytical.abs() < TOL_REL,
        "tip deflection (local y → global z): FEM {} vs analytical {}",
        tip_uz,
        analytical
    );
}

#[test]
fn case_b_cantilever_tip_load_y_rotation() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_ry = result.displacements[10];
    let analytical = -P_NUM * L_NUM * L_NUM / (2.0 * E * IZ);
    assert!(
        (tip_ry - analytical).abs() / analytical.abs() < TOL_REL,
        "tip rotation (local y → global y rot): FEM {} vs analytical {}",
        tip_ry,
        analytical
    );
}

#[test]
fn case_b_cantilever_tip_load_reaction() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let rz0 = result.reactions[2];
    assert!(
        (rz0 - (-P_NUM)).abs() < TOL_ABS,
        "fixed-end reaction = -P = {}, got {}",
        -P_NUM,
        rz0
    );
    let my0 = result.reactions[4];
    let analytical_moment = P_NUM * L_NUM;
    assert!(
        (my0 - analytical_moment).abs() / analytical_moment.abs() < TOL_REL,
        "fixed-end moment (local y → global y): -PL = {}, got {}",
        analytical_moment,
        my0
    );
}

#[test]
fn case_c_cantilever_midspan_load_deflection() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_uz = result.displacements[8];
    let a = L_NUM / 2.0;
    let analytical = P_NUM * a * a * (3.0 * L_NUM - a) / (6.0 * E * IZ);
    assert!(
        (tip_uz - analytical).abs() / analytical.abs() < TOL_REL,
        "tip deflection (midspan local y): FEM {} vs analytical {}",
        tip_uz,
        analytical
    );
}

#[test]
fn case_c_cantilever_midspan_load_rotation() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_ry = result.displacements[10];
    let a = L_NUM / 2.0;
    let analytical = -P_NUM * a * a / (2.0 * E * IZ);
    assert!(
        (tip_ry - analytical).abs() / analytical.abs() < TOL_REL,
        "tip rotation (midspan local y → global y rot): FEM {} vs analytical {}",
        tip_ry,
        analytical
    );
}

#[test]
fn case_d_simply_supported_quarter_span_reactions() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let rz0 = result.reactions[2];
    let rz1 = result.reactions[8];

    let a = 0.25 * L_NUM;
    let b = 0.75 * L_NUM;
    let r_a_analytical = -P_NUM * b / L_NUM;
    let r_b_analytical = -P_NUM * a / L_NUM;

    assert!(
        (rz0 - r_a_analytical).abs() / r_a_analytical.abs() < TOL_REL,
        "reaction A: FEM {} vs analytical {}",
        rz0,
        r_a_analytical
    );
    assert!(
        (rz1 - r_b_analytical).abs() / r_b_analytical.abs() < TOL_REL,
        "reaction B: FEM {} vs analytical {}",
        rz1,
        r_b_analytical
    );
}

#[test]
fn case_d_simply_supported_quarter_span_moment_equilibrium() {
    let (mut m, _a, _b, mid) = make_simply_supported(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.25, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let rz0 = result.reactions[2];
    let rz1 = result.reactions[8];

    let moment_about_a = rz1 * L_NUM;
    let applied_moment = -P_NUM * 0.25 * L_NUM;
    assert!(
        (moment_about_a - applied_moment).abs() < 1e-3,
        "moment equilibrium: reactions {} vs applied {}",
        moment_about_a,
        applied_moment
    );
}

#[test]
fn case_e_axial_point_load_cantilever() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    let f_axial = 5000.0;
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, f_axial, 0.0, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_ux = result.displacements[6];
    let analytical = f_axial * L_NUM / (E * AREA);
    assert!(
        (tip_ux - analytical).abs() / analytical.abs() < TOL_REL,
        "axial deflection: FEM {} vs analytical {}",
        tip_ux,
        analytical
    );

    let rx0 = result.reactions[0];
    assert!(
        (rx0 - (-f_axial)).abs() < TOL_ABS,
        "axial reaction = -F = {}, got {}",
        -f_axial,
        rx0
    );
}

#[test]
fn case_f_z_direction_cantilever() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    let p_z = -800.0;
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, 0.0, p_z))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let tip_uy = result.displacements[7];
    let analytical_local_z = p_z * L_NUM.powi(3) / (3.0 * E * IY);
    let analytical_global_y = -analytical_local_z;
    assert!(
        (tip_uy - analytical_global_y).abs() / analytical_global_y.abs() < TOL_REL,
        "z-direction tip deflection: FEM {} vs analytical {}",
        tip_uy,
        analytical_global_y
    );
}

#[test]
fn case_g_multiple_loads_superposition() {
    let (mut m, _a, _b, mid) = make_cantilever(L_NUM);
    m.add_member_point_load(mid, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result1 = m.to_frame_model().unwrap().solve().unwrap();
    let tip1 = result1.displacements[8];

    let (mut m2, _, _, mid2) = make_cantilever(L_NUM);
    m2.add_member_point_load(mid2, MemberPointLoad3D::new(1.0, 0.0, P_NUM / 2.0, 0.0))
        .unwrap();
    m2.add_member_point_load(mid2, MemberPointLoad3D::new(1.0, 0.0, P_NUM / 2.0, 0.0))
        .unwrap();
    let result2 = m2.to_frame_model().unwrap().solve().unwrap();
    let tip2 = result2.displacements[8];

    assert!(
        (tip1 - tip2).abs() < 1e-10,
        "superposition: single load {} vs two half loads {}",
        tip1,
        tip2
    );
}

#[test]
fn case_h_end_positions_match_nodal_load() {
    let (mut m1, _a, _b, mid1) = make_cantilever(L_NUM);
    m1.add_member_point_load(mid1, MemberPointLoad3D::new(1.0, 0.0, P_NUM, 0.0))
        .unwrap();
    let result1 = m1.to_frame_model().unwrap().solve().unwrap();
    let tip1 = result1.displacements[8];

    let (mut m2, _a2, b2, _mid2) = make_cantilever(L_NUM);
    m2.set_nodal_load(
        b2,
        viewport_poc::edit_model::NodalLoadSpec::new(0.0, 0.0, P_NUM, 0.0, 0.0, 0.0),
    )
    .unwrap();
    let result2 = m2.to_frame_model().unwrap().solve().unwrap();
    let tip2 = result2.displacements[8];

    assert!(
        (tip1 - tip2).abs() < 1e-10,
        "xi=1 local y point load should match global z nodal load: point {} vs nodal {}",
        tip1,
        tip2
    );
}

#[test]
fn case_i_total_force_equilibrium() {
    let (mut m, _a, _b, mid) = make_fixed_fixed_beam(L_NUM);
    let fx_applied = 100.0;
    let fy_applied = -500.0;
    let fz_applied = 300.0;
    m.add_member_point_load(
        mid,
        MemberPointLoad3D::new(0.25, fx_applied, fy_applied, fz_applied),
    )
    .unwrap();
    let fx_applied2 = -200.0;
    let fy_applied2 = -300.0;
    let fz_applied2 = -400.0;
    m.add_member_point_load(
        mid,
        MemberPointLoad3D::new(0.75, fx_applied2, fy_applied2, fz_applied2),
    )
    .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let total_fx = result.reactions[0] + result.reactions[6];
    let total_fy = result.reactions[1] + result.reactions[7];
    let total_fz = result.reactions[2] + result.reactions[8];

    let expected_fx = -(fx_applied + fx_applied2);
    let expected_fy = fz_applied + fz_applied2;
    let expected_fz = -(fy_applied + fy_applied2);

    assert!(
        (total_fx - expected_fx).abs() < TOL_ABS,
        "fx equilibrium: {} vs {}",
        total_fx,
        expected_fx
    );
    assert!(
        (total_fy - expected_fy).abs() < TOL_ABS,
        "fy equilibrium: {} vs {}",
        total_fy,
        expected_fy
    );
    assert!(
        (total_fz - expected_fz).abs() < TOL_ABS,
        "fz equilibrium: {} vs {}",
        total_fz,
        expected_fz
    );
}

// ════════════════════════════════════════════════════════════════════════
// 9. Phase 153 regression — uniform loads still work
// ════════════════════════════════════════════════════════════════════════

#[test]
fn regression_uniform_load_still_works() {
    let (mut m, _a, _b, mid) = make_fixed_fixed_beam(L_NUM);
    let q = -500.0;
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, q, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();
    let total_rz = result.reactions[2] + result.reactions[8];
    let expected = -q * L_NUM;
    assert!(
        (total_rz - expected).abs() < TOL_ABS,
        "uniform local y load → global z reaction: {} vs {}",
        total_rz,
        expected
    );
}

#[test]
fn regression_both_load_types_solve() {
    let (mut m, _a, _b, mid) = make_fixed_fixed_beam(L_NUM);
    let q = -500.0;
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, q, 0.0))
        .unwrap();
    m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, P_NUM, 0.0))
        .unwrap();
    let result = m.to_frame_model().unwrap().solve().unwrap();

    let total_rz = result.reactions[2] + result.reactions[8];
    let expected = -q * L_NUM + (-P_NUM);
    assert!(
        (total_rz - expected).abs() < TOL_ABS,
        "combined load reaction: {} vs {}",
        total_rz,
        expected
    );
}

#[test]
fn regression_fingerprint_unchanged_for_uniform_only() {
    let (mut m, _, _, mid) = make_beam(4.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();
    let fp = m.fingerprint();

    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.fingerprint(), fp);
}
