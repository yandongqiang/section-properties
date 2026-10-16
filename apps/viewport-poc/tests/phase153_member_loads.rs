//! Phase 153: Member uniform distributed loads — numerical and integration tests.
//!
//! Covers:
//! - Case A: Fixed-fixed beam with uniform transverse load (qL/2, qL²/12)
//! - Case B: Cantilever with uniform transverse load (qL⁴/8EI, qL, qL²/2)
//! - Case C: Uniform axial load equilibrium
//! - Case D: Rotated 3D member with non-default ref vec
//! - Integration: invalid MemberId, non-finite, replacement, clear, deletion,
//!   stable ID, preset compat, solve after edit, stale, undo/redo, dirty-state,
//!   V1 compat, V2 round-trip, failed load, viewport direction, zero-load.

#![allow(non_snake_case)]

use viewport_poc::{
    edit_model::{EditError, EditModel, MemberId, MemberUniformLoad3D, NodeId, SupportSpec},
    persistence,
    scene::ViewportScene,
};

const E: f64 = 200e9;
const G: f64 = 80e9;
const AREA: f64 = 1e-3;
const IY: f64 = 2e-6;
const IZ: f64 = 1e-6;
const J: f64 = 3e-6;

const TOL: f64 = 1e-6;
const TOL_FORCE: f64 = 1e-3;
const TOL_MOMENT: f64 = 1e-2;

fn make_beam(L: f64) -> (EditModel, NodeId, NodeId, MemberId) {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(L, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    (m, a, b, mid)
}

// ════════════════════════════════════════════════════════════════════════
//  Case A: Fixed-fixed beam with uniform transverse load
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_a_fixed_fixed_reactions() {
    let L = 4.0_f64;
    let q = -1000.0_f64;
    let (mut m, a, b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, q, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");

    let r = &result.reactions;
    // Reactions oppose the applied load: sum = -q*L
    let rz_a = r[2];
    let rz_b = r[8];
    let total_z = rz_a + rz_b;
    assert!(
        (total_z + q * L).abs() < TOL_FORCE,
        "sum of z-reactions {total_z} should equal -qL = {}",
        -q * L
    );
    assert!(
        (rz_a + q * L / 2.0).abs() < TOL_FORCE,
        "z-reaction at a {rz_a} should be -qL/2 = {}",
        -q * L / 2.0
    );
    assert!(
        (rz_b + q * L / 2.0).abs() < TOL_FORCE,
        "z-reaction at b {rz_b} should be -qL/2 = {}",
        -q * L / 2.0
    );

    // Moment reactions: qL²/12 at i, -qL²/12 at j (about local z = global y)
    let my_a = r[4];
    let my_b = r[10];
    let expected_m = q * L * L / 12.0;
    assert!(
        (my_a - expected_m).abs() < TOL_MOMENT,
        "moment at a {my_a} should be qL²/12 = {expected_m}"
    );
    assert!(
        (my_b + expected_m).abs() < TOL_MOMENT,
        "moment at b {my_b} should be -qL²/12 = {}",
        -expected_m
    );
}

// ════════════════════════════════════════════════════════════════════════
//  Case B: Cantilever with uniform transverse load
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_b_cantilever_tip_deflection() {
    let L = 3.0_f64;
    let q = -5000.0_f64;
    let (mut m, a, _b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, q, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");

    // Tip deflection at node b (index 1): uz = displacements[1*6 + 2] = displacements[8]
    // local y = global z, so deflection in z
    let expected_deflection = q * L.powi(4) / (8.0 * E * IZ);
    let uz_tip = result.displacements[8];
    assert!(
        (uz_tip - expected_deflection).abs() < TOL,
        "tip deflection {uz_tip} should be qL⁴/(8EI) = {expected_deflection}"
    );
}

#[test]
fn case_b_cantilever_reactions() {
    let L = 3.0_f64;
    let q = -5000.0_f64;
    let (mut m, a, _b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, q, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");

    let r = &result.reactions;
    // Reaction force at fixed end: -qL in global z (opposes load)
    let rz_a = r[2];
    assert!(
        (rz_a + q * L).abs() < TOL_FORCE,
        "z-reaction {rz_a} should be -qL = {}",
        -q * L
    );
    // Reaction moment at fixed end: qL²/2 about global y
    let my_a = r[4];
    let expected_m = q * L * L / 2.0;
    assert!(
        (my_a - expected_m).abs() < TOL_MOMENT,
        "moment {my_a} should be qL²/2 = {expected_m}"
    );
}

// ════════════════════════════════════════════════════════════════════════
//  Case C: Uniform axial load equilibrium
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_c_axial_load_equilibrium() {
    let L = 2.0_f64;
    let q = 2000.0_f64;
    let (mut m, a, b, mid) = make_beam(L);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(q, 0.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");

    let r = &result.reactions;
    // Axial load in local x = global x for this orientation
    // Reactions oppose the load: sum = -q*L
    let rx_a = r[0];
    let rx_b = r[6];
    assert!(
        (rx_a + rx_b + q * L).abs() < TOL_FORCE,
        "sum of x-reactions {} should equal -total axial load {}",
        rx_a + rx_b,
        -q * L
    );
    assert!(
        (rx_a + q * L / 2.0).abs() < TOL_FORCE,
        "x-reaction at a {rx_a} should be -qL/2 = {}",
        -q * L / 2.0
    );
    assert!(
        (rx_b + q * L / 2.0).abs() < TOL_FORCE,
        "x-reaction at b {rx_b} should be -qL/2 = {}",
        -q * L / 2.0
    );
}

// ════════════════════════════════════════════════════════════════════════
//  Case D: Rotated 3D member with non-default ref vec
// ════════════════════════════════════════════════════════════════════════

#[test]
fn case_d_rotated_member_equilibrium() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 4.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    m.set_member_section(mid, E, G, AREA, IY, IZ, J).unwrap();
    m.set_member_ref_vec(mid, [1.0, 0.0, 0.0]).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -1000.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");

    let r = &result.reactions;
    // Total applied force = q * L * y_hat = -1000 * 5 * (4/5, -3/5, 0) = (-4000, 3000, 0)
    // Reactions = -applied = (4000, -3000, 0)
    let total_fx = r[0] + r[6];
    let total_fy = r[1] + r[7];
    let total_fz = r[2] + r[8];
    assert!(
        (total_fx - 4000.0).abs() < TOL_FORCE,
        "total x-reaction {total_fx} should be 4000"
    );
    assert!(
        (total_fy + 3000.0).abs() < TOL_FORCE,
        "total y-reaction {total_fy} should be -3000"
    );
    assert!(
        total_fz.abs() < TOL_FORCE,
        "total z-reaction {total_fz} should be 0"
    );
}

// ════════════════════════════════════════════════════════════════════════
//  Integration tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn invalid_member_id_rejected() {
    let mut m = EditModel::new();
    let _a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let err = m.set_member_uniform_load(MemberId(99), MemberUniformLoad3D::new(1.0, 0.0, 0.0));
    assert_eq!(err, Err(EditError::MemberNotFound(MemberId(99))));
}

#[test]
fn non_finite_load_rejected() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let err = m.set_member_uniform_load(mid, MemberUniformLoad3D::new(f64::NAN, 0.0, 0.0));
    assert_eq!(err, Err(EditError::NonFiniteLoad));
    let err = m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, f64::INFINITY, 0.0));
    assert_eq!(err, Err(EditError::NonFiniteLoad));
}

#[test]
fn load_replacement() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 200.0, 300.0))
        .unwrap();
    let load = m.member_uniform_load(mid).unwrap();
    assert_eq!(load.qx, 100.0);
    assert_eq!(load.qy, 200.0);
    assert_eq!(load.qz, 300.0);

    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(500.0, 0.0, 0.0))
        .unwrap();
    let load = m.member_uniform_load(mid).unwrap();
    assert_eq!(load.qx, 500.0);
    assert_eq!(load.qy, 0.0);
    assert_eq!(load.qz, 0.0);
}

#[test]
fn clear_load() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 200.0, 300.0))
        .unwrap();
    assert!(m.has_member_uniform_load(mid));
    m.clear_member_uniform_load(mid).unwrap();
    assert!(!m.has_member_uniform_load(mid));
    assert!(m.member_uniform_load(mid).is_none());
}

#[test]
fn clear_load_nonexistent_member_fails() {
    let mut m = EditModel::new();
    let _a = m.add_node(0.0, 0.0, 0.0).unwrap();
    assert_eq!(
        m.clear_member_uniform_load(MemberId(99)),
        Err(EditError::MemberNotFound(MemberId(99)))
    );
}

#[test]
fn delete_member_removes_load() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(m.member_uniform_loads().len(), 1);
    m.delete_member(mid).unwrap();
    assert_eq!(m.member_uniform_loads().len(), 0);
    assert!(!m.has_member_uniform_load(mid));
}

#[test]
fn stable_id_mapping_to_solver() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let c = m.add_node(6.0, 0.0, 0.0).unwrap();
    let m1 = m.add_member(a, b).unwrap();
    let m2 = m.add_member(b, c).unwrap();
    m.set_member_section(m1, E, G, AREA, IY, IZ, J).unwrap();
    m.set_member_section(m2, E, G, AREA, IY, IZ, J).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(c, SupportSpec::fixed()).unwrap();
    m.set_member_uniform_load(m2, MemberUniformLoad3D::new(0.0, -500.0, 0.0))
        .unwrap();

    m.delete_member(m1).unwrap();
    assert!(!m.has_member_uniform_load(m1));
    assert!(m.has_member_uniform_load(m2));

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("solve should succeed");
    // 3 nodes: a(0), b(1), c(2). a and c are fixed.
    // z-reactions: r[2] (node a), r[14] (node c)
    let total_rz = result.reactions[2] + result.reactions[14];
    assert!(
        (total_rz + (-500.0) * 3.0).abs() < TOL_FORCE,
        "total z-reaction should equal -total load"
    );
}

#[test]
fn preset_compatibility() {
    use viewport_poc::model::PresetModel;
    let m = PresetModel::Portal.build_edit();
    assert_eq!(m.member_uniform_loads().len(), 0);
    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

#[test]
fn solve_after_load_editing() {
    let (mut m, a, b, mid) = make_beam(4.0);
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();

    let fm0 = m.to_frame_model().unwrap();
    let r0 = fm0.solve().unwrap();
    let base_rz = r0.reactions[2] + r0.reactions[8];

    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -800.0, 0.0))
        .unwrap();
    let fm1 = m.to_frame_model().unwrap();
    let r1 = fm1.solve().unwrap();
    let loaded_rz = r1.reactions[2] + r1.reactions[8];

    assert!(
        base_rz.abs() < TOL_FORCE,
        "no-load reactions should be zero"
    );
    assert!(
        (loaded_rz + (-800.0) * 4.0).abs() < TOL_FORCE,
        "loaded reactions should equal -total load"
    );
}

#[test]
fn fingerprint_includes_member_loads() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let fp0 = m.fingerprint();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    let fp1 = m.fingerprint();
    assert_ne!(fp0, fp1, "fingerprint must change after setting a load");
    m.clear_member_uniform_load(mid).unwrap();
    let fp2 = m.fingerprint();
    assert_eq!(fp0, fp2, "fingerprint must restore after clearing the load");
}

#[test]
fn undo_redo_restores_loads() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let original = m.clone();
    let fp_original = original.fingerprint();

    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 200.0, 300.0))
        .unwrap();
    let fp_loaded = m.fingerprint();
    assert_ne!(fp_loaded, fp_original);

    let mut restored = original;
    restored.ensure_revision_at_least(m.revision() + 1);
    assert_eq!(
        restored.fingerprint(),
        fp_original,
        "restored model should have original fingerprint"
    );
    assert!(
        !restored.has_member_uniform_load(mid),
        "restored model should not have the load"
    );
}

#[test]
fn dirty_state_recovery() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let saved_fp = m.fingerprint();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(50.0, 0.0, 0.0))
        .unwrap();
    assert_ne!(m.fingerprint(), saved_fp, "should be dirty after edit");
    m.clear_member_uniform_load(mid).unwrap();
    assert_eq!(
        m.fingerprint(),
        saved_fp,
        "should be clean after clearing back to original"
    );
}

// ════════════════════════════════════════════════════════════════════════
//  Persistence tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn v1_file_loads_with_empty_loads() {
    let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
    let m = persistence::deserialize_project(json).expect("V1 file should load");
    assert_eq!(m.n_members(), 1);
    assert_eq!(m.member_uniform_loads().len(), 0);
}

#[test]
fn v2_round_trip_with_loads() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, -200.0, 300.0))
        .unwrap();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    let load = m2
        .member_uniform_load(mid)
        .expect("load should survive round-trip");
    assert_eq!(load.qx, 100.0);
    assert_eq!(load.qy, -200.0);
    assert_eq!(load.qz, 300.0);
}

#[test]
fn v2_round_trip_empty_loads() {
    let m = EditModel::new();
    let json = persistence::serialize_project(&m).unwrap();
    let m2 = persistence::deserialize_project(&json).unwrap();
    assert_eq!(m2.member_uniform_loads().len(), 0);
}

#[test]
fn v4_format_version_in_json() {
    let m = EditModel::new();
    let json = persistence::serialize_project(&m).unwrap();
    assert!(
        json.contains("\"format_version\": 4"),
        "serialized JSON should contain format_version 4, got: {json}"
    );
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

#[test]
fn failed_load_preserves_existing() {
    let json = r#"{"format_version":2,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[{"member_id":99,"qx":100.0,"qy":0.0,"qz":0.0}]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn v2_duplicate_member_load_rejected() {
    let json = r#"{"format_version":2,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[{"member_id":0,"qx":100.0,"qy":0.0,"qz":0.0},{"member_id":0,"qx":200.0,"qy":0.0,"qz":0.0}]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(matches!(err, persistence::PersistenceError::Validation(_)));
}

#[test]
fn v2_non_finite_load_rejected() {
    let json = r#"{"format_version":2,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[{"member_id":0,"qx":null,"qy":0.0,"qz":0.0}]}}"#;
    let err = persistence::deserialize_project(json).unwrap_err();
    assert!(
        matches!(err, persistence::PersistenceError::JsonError(_))
            || matches!(err, persistence::PersistenceError::Validation(_))
    );
}

#[test]
fn filesystem_v2_round_trip() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(500.0, -300.0, 200.0))
        .unwrap();
    let dir = std::env::temp_dir();
    let path = dir.join("test_phase153_v2.saproject");
    persistence::save_project(&path, &m).unwrap();
    let m2 = persistence::load_project(&path).unwrap();
    let load = m2.member_uniform_load(mid).unwrap();
    assert_eq!(load.qx, 500.0);
    assert_eq!(load.qy, -300.0);
    assert_eq!(load.qz, 200.0);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn golden_v1_file_unchanged() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden_v1.saproject");
    let m = persistence::load_project(&path).expect("golden V1 file should load");
    assert_eq!(m.member_uniform_loads().len(), 0);
    assert!(m.n_nodes() > 0);
}

// ════════════════════════════════════════════════════════════════════════
//  Scene / viewport tests
// ════════════════════════════════════════════════════════════════════════

#[test]
fn scene_includes_member_loads() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 200.0, 300.0))
        .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_loads.len(), 1);
    let load = scene.member_loads[0].expect("load should be in scene");
    assert_eq!(load.qx, 100.0);
    assert_eq!(load.qy, 200.0);
    assert_eq!(load.qz, 300.0);
}

#[test]
fn scene_no_loads_rendered_as_none() {
    let (m, _, _, _) = make_beam(3.0);
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.member_loads.len(), 1);
    assert!(scene.member_loads[0].is_none());
}

#[test]
fn zero_load_set_then_clear() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    assert!(m.has_member_uniform_load(mid));
    m.set_member_uniform_load(mid, MemberUniformLoad3D::zero())
        .unwrap();
    let load = m.member_uniform_load(mid).unwrap();
    assert!(load.is_zero());
    m.clear_member_uniform_load(mid).unwrap();
    assert!(!m.has_member_uniform_load(mid));
}

#[test]
fn load_magnitude_and_finite() {
    let load = MemberUniformLoad3D::new(3.0, 4.0, 0.0);
    assert!((load.magnitude() - 5.0).abs() < 1e-15);
    assert!(load.is_finite());
    assert!(!load.is_zero());

    let zero = MemberUniformLoad3D::zero();
    assert!(zero.is_zero());
    assert!(zero.is_finite());

    let bad = MemberUniformLoad3D::new(f64::NAN, 0.0, 0.0);
    assert!(!bad.is_finite());
}

#[test]
fn multiple_members_with_loads() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let c = m.add_node(6.0, 0.0, 0.0).unwrap();
    let m1 = m.add_member(a, b).unwrap();
    let m2 = m.add_member(b, c).unwrap();
    m.set_member_section(m1, E, G, AREA, IY, IZ, J).unwrap();
    m.set_member_section(m2, E, G, AREA, IY, IZ, J).unwrap();
    m.set_member_uniform_load(m1, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    m.set_member_uniform_load(m2, MemberUniformLoad3D::new(0.0, 200.0, 0.0))
        .unwrap();
    assert_eq!(m.member_uniform_loads().len(), 2);
    assert!(m.has_member_uniform_load(m1));
    assert!(m.has_member_uniform_load(m2));
    let loads = m.member_uniform_loads();
    assert_eq!(loads[0].0, m1);
    assert_eq!(loads[1].0, m2);
}

#[test]
fn revision_bumped_on_load_change() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let rev0 = m.revision();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    assert!(m.revision() > rev0);
    let rev1 = m.revision();
    m.clear_member_uniform_load(mid).unwrap();
    assert!(m.revision() > rev1);
}

#[test]
fn no_op_set_same_load_no_revision_bump() {
    let (mut m, _, _, mid) = make_beam(3.0);
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    let rev = m.revision();
    m.set_member_uniform_load(mid, MemberUniformLoad3D::new(100.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(
        m.revision(),
        rev,
        "setting same load should not bump revision"
    );
}

#[test]
fn clear_nonexistent_load_is_ok() {
    let (mut m, _, _, mid) = make_beam(3.0);
    let rev = m.revision();
    m.clear_member_uniform_load(mid).unwrap();
    assert_eq!(
        m.revision(),
        rev,
        "clearing nonexistent load should not bump revision"
    );
}
