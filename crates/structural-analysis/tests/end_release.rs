//! Phase 91 — Member end releases (hinges) tests.

#![allow(non_snake_case)]

use section_properties::SolverSelection;
use section_properties::material::Material;
use structural_analysis::MemberHandle;
use structural_analysis::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, EndRelease, FemError,
};
use structural_analysis::frame::FrameModel;

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn section() -> BeamSection {
    BeamSection::new(5e-3, 2e-5)
}

const E: f64 = 200e9;
const I: f64 = 2e-5;

// §8 Test 1 — No-release regression: rigid-rigid matches baseline

#[test]
fn test_no_release_matches_rigid_baseline() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    frame.add_member(a, b, steel(), section()).unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    let m = MemberHandle::from_index(0);
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();

    let M_fixed = q * L * L / 12.0;
    let forces = result.member_end_forces(m).unwrap();
    assert!(
        (forces[2] + M_fixed).abs() < 1e-3,
        "M_i = {}, expected {}",
        forces[2],
        -M_fixed
    );
    assert!(
        (forces[5] - M_fixed).abs() < 1e-3,
        "M_j = {}, expected {}",
        forces[5],
        M_fixed
    );
    assert!(result.equilibrium().is_balanced());
}

// §8 Test 2 — Cantilever: verify correct result (release at free end = mechanism)

#[test]
fn test_cantilever_correct_result() {
    let L = 3.0_f64;
    let P = 10000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    frame.add_member(a, b, steel(), section()).unwrap();
    frame.fix(a).unwrap();
    frame.nodal_load(b, 0.0, -P).unwrap();
    let result = frame.solve().unwrap();
    let uy = result.displacement(b, Dof::Uy).unwrap();

    let delta = P * L.powi(3) / (3.0 * E * I);
    assert!(
        (uy + delta).abs() < 1e-6,
        "uy = {}, expected {}",
        uy,
        -delta
    );

    let forces = result
        .member_end_forces(MemberHandle::from_index(0))
        .unwrap();
    assert!(
        forces[5].abs() < 1e-3,
        "Free-end moment ~ 0, got {}",
        forces[5]
    );
}

// §8 Test 3 — Rigid-Pinned: M_end ~ 0 (propped cantilever)

#[test]
fn test_rigid_pinned_end_moment_zero() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(forces[5].abs() < 1.0, "M_j ~ 0, got {}", forces[5]);

    let M_fixed = q * L * L / 8.0;
    assert!(
        (forces[2] + M_fixed).abs() < 1.0,
        "M_i = {}, expected {}",
        forces[2],
        -M_fixed
    );

    let r_a = result.reaction(a, Dof::Uy).unwrap();
    let r_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (r_a - 5.0 * q * L / 8.0).abs() < 1.0,
        "R_a = {}, expected {}",
        r_a,
        5.0 * q * L / 8.0
    );
    assert!(
        (r_b - 3.0 * q * L / 8.0).abs() < 1.0,
        "R_b = {}, expected {}",
        r_b,
        3.0 * q * L / 8.0
    );
    assert!(result.equilibrium().is_balanced());
}

// §8 Test 4 — Pinned-Rigid: M_start ~ 0

#[test]
fn test_pinned_rigid_start_moment_zero() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::start_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(forces[2].abs() < 1.0, "M_i ~ 0, got {}", forces[2]);

    let M_fixed = q * L * L / 8.0;
    assert!(
        (forces[5] - M_fixed).abs() < 1.0,
        "M_j = {}, expected {}",
        forces[5],
        M_fixed
    );

    let r_a = result.reaction(a, Dof::Uy).unwrap();
    let r_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (r_a - 3.0 * q * L / 8.0).abs() < 1.0,
        "R_a = {}, expected {}",
        r_a,
        3.0 * q * L / 8.0
    );
    assert!(
        (r_b - 5.0 * q * L / 8.0).abs() < 1.0,
        "R_b = {}, expected {}",
        r_b,
        5.0 * q * L / 8.0
    );
    assert!(result.equilibrium().is_balanced());
}

// §8 Test 5 — Pinned-Pinned: both end moments ~ 0 (simply supported)

#[test]
fn test_pinned_pinned_both_moments_zero() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::both_pins())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(forces[2].abs() < 1.0, "M_i ~ 0, got {}", forces[2]);
    assert!(forces[5].abs() < 1.0, "M_j ~ 0, got {}", forces[5]);

    let r_a = result.reaction(a, Dof::Uy).unwrap();
    let r_b = result.reaction(b, Dof::Uy).unwrap();
    let R = q * L / 2.0;
    assert!((r_a - R).abs() < 1.0, "R_a = {}, expected {}", r_a, R);
    assert!((r_b - R).abs() < 1.0, "R_b = {}, expected {}", r_b, R);

    let sf_mid = result.section_forces(m, 0.5).unwrap();
    let M_mid = q * L * L / 8.0;
    assert!(
        (sf_mid.moment - M_mid).abs() < 1.0,
        "M_mid = {}, expected {}",
        sf_mid.moment,
        M_mid
    );
    assert!(result.equilibrium().is_balanced());
}

// §7 — Node DOF vs. member end release distinction

#[test]
fn test_node_rz_exists_after_member_release() {
    //   A ====== B
    //            |
    //            C
    // AB released at B. BC rigid. B.Rz must still exist.
    // Node B moment equilibrium: M_AB_end + M_BC_start = 0.
    // Since M_AB_end = 0 (released), M_BC_start = 0 too.
    // But BC end C should have non-zero moment (proves B.Rz is active).

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let c = frame.add_node(4.0, -3.0).unwrap();

    frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.add_member(b, c, steel(), section()).unwrap();

    frame.fix(a).unwrap();
    frame.fix(c).unwrap();
    frame.nodal_load(b, 5000.0, 0.0).unwrap(); // horizontal load at B

    let result = frame.solve().unwrap();
    let f_ab = result
        .member_end_forces(MemberHandle::from_index(0))
        .unwrap();
    let f_bc = result
        .member_end_forces(MemberHandle::from_index(1))
        .unwrap();

    assert!(
        f_ab[5].abs() < 1.0,
        "AB end moment at B ~ 0, got {}",
        f_ab[5]
    );
    assert!(
        f_bc[2].abs() < 1.0,
        "BC start moment at B ~ 0 (equilibrium), got {}",
        f_bc[2]
    );
    // BC end C has non-zero moment — horizontal load bends BC
    assert!(
        f_bc[5].abs() > 10.0,
        "BC end moment at C non-zero, got {}",
        f_bc[5]
    );
    assert!(result.equilibrium().is_balanced());
}

// §9 — Transformation: angled member with release

#[test]
fn test_angled_member_with_release() {
    let L = 5.0_f64;
    let s45 = std::f64::consts::FRAC_1_SQRT_2;
    let dx = L * s45;
    let dy = L * s45;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(dx, dy).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let q = 1000.0_f64;
    let qx_local = -q * s45;
    let qy_local = -q * s45;
    frame.member_udl(m, qx_local, qy_local).unwrap();

    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();
    assert!(
        forces[5].abs() < 1.0,
        "Released end moment ~ 0 for angled member, got {}",
        forces[5]
    );
    assert!(result.equilibrium().is_balanced());
}

// §10 — Distributed load with release

#[test]
fn test_distributed_load_with_release_equilibrium() {
    let L = 6.0_f64;
    let q = 2000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(forces[5].abs() < 1.0, "M_j ~ 0, got {}", forces[5]);

    let r_a = result.reaction(a, Dof::Uy).unwrap();
    let r_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (r_a + r_b - q * L).abs() < 1.0,
        "Vertical equilibrium failed"
    );
    assert!(result.equilibrium().is_balanced());
}

// §11 — Point load with release

#[test]
fn test_point_load_with_release() {
    let L = 4.0_f64;
    let P = 8000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_point_load(m, 0.5, 0.0, -P, 0.0).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(
        forces[5].abs() < 1.0,
        "M_j ~ 0 with point load, got {}",
        forces[5]
    );

    let r_a = result.reaction(a, Dof::Uy).unwrap();
    let r_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (r_a + r_b - P).abs() < 1.0,
        "Vertical equilibrium with point load"
    );
    assert!(result.equilibrium().is_balanced());
}

// §11 — Applied moment with release

#[test]
fn test_applied_moment_with_release() {
    let L = 4.0_f64;
    let M_app = 5000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.nodal_moment(a, M_app).unwrap();
    let result = frame.solve().unwrap();
    let forces = result.member_end_forces(m).unwrap();

    assert!(
        forces[5].abs() < 1.0,
        "M_j ~ 0 with applied moment, got {}",
        forces[5]
    );
    assert!(result.equilibrium().is_balanced());
}

// §12 — Multi-member frame with release on one member

#[test]
fn test_multi_member_frame_with_release() {
    //   A ====== B
    //            |
    //            C
    // AB released at B. BC rigid. Horizontal load at B.
    // B moment equilibrium: M_AB_end + M_BC_start = 0 → both ~ 0.
    // BC end C should have non-zero moment.

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let c = frame.add_node(4.0, -3.0).unwrap();

    let m_ab = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    let m_bc = frame.add_member(b, c, steel(), section()).unwrap();

    frame.fix(a).unwrap();
    frame.fix(c).unwrap();
    frame.nodal_load(b, 10000.0, 0.0).unwrap();

    let result = frame.solve().unwrap();
    let f_ab = result.member_end_forces(m_ab).unwrap();
    let f_bc = result.member_end_forces(m_bc).unwrap();

    assert!(
        f_ab[5].abs() < 1.0,
        "AB released end moment ~ 0, got {}",
        f_ab[5]
    );
    assert!(
        f_bc[2].abs() < 1.0,
        "BC start moment ~ 0 (equilibrium), got {}",
        f_bc[2]
    );
    assert!(
        f_bc[5].abs() > 10.0,
        "BC end moment at C non-zero, got {}",
        f_bc[5]
    );
    assert!(result.equilibrium().is_balanced());
}

// §13 — Mechanism: release can create mechanism

#[test]
fn test_release_creates_mechanism() {
    let L = 4.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::both_pins())
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();
    frame
        .member_udl(MemberHandle::from_index(0), 0.0, -1000.0)
        .unwrap();

    let result = frame.solve();
    assert!(
        result.is_err(),
        "Pinned-pinned with pin supports should be a mechanism"
    );
    assert!(matches!(result.unwrap_err(), FemError::SolverError { .. }));
}

#[test]
fn test_release_stable_structure() {
    let L = 4.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::both_pins())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame
        .member_udl(MemberHandle::from_index(0), 0.0, -1000.0)
        .unwrap();

    assert!(
        frame.solve().is_ok(),
        "Pinned-pinned with fix supports should be stable"
    );
}

// §14 — Solver selection with release

#[test]
fn test_release_with_default_solver() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();
    let result = frame.solve().unwrap();

    let forces = result.member_end_forces(m).unwrap();
    assert!(forces[5].abs() < 1.0, "M_j ~ 0 with default solver");
    assert!(result.equilibrium().is_balanced());
}

#[test]
fn test_release_with_sparse_lu_solver() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(L, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    frame.member_udl(m, 0.0, -q).unwrap();

    let result = frame
        .solve_with(SolverSelection::Named("sparse_lu".to_string()))
        .unwrap();
    let forces = result.member_end_forces(m).unwrap();
    assert!(forces[5].abs() < 1.0, "M_j ~ 0 with sparse_lu solver");
    assert!(result.equilibrium().is_balanced());
}

// §8 supplementary — Section forces at released end

#[test]
fn test_section_forces_at_released_end() {
    let L = 4.0_f64;
    let q = 1000.0_f64;

    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, L, 0.0));
    model.add_element(
        BeamElement::with_end_release(0, 1, steel(), section(), EndRelease::end_pin()).unwrap(),
    );
    model.fix_node(0);
    model.fix_node(1);
    model.add_distributed_load(0, 0.0, -q).unwrap();

    let mut solver = BeamSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let sf_end = solver.element_section_forces(0, 1.0).unwrap();
    assert!(
        sf_end.moment.abs() < 1.0,
        "Section moment at released end ~ 0, got {}",
        sf_end.moment
    );

    let sf_mid = solver.element_section_forces(0, 0.5).unwrap();
    assert!(
        sf_mid.moment > 100.0,
        "Midspan moment positive (sagging), got {}",
        sf_mid.moment
    );
}

// §7 supplementary — EndRelease API

#[test]
fn test_end_release_api() {
    assert_eq!(EndRelease::none(), EndRelease::default());
    assert!(EndRelease::none().is_empty());
    assert!(!EndRelease::start_pin().is_empty());
    assert!(!EndRelease::end_pin().is_empty());
    assert!(!EndRelease::both_pins().is_empty());
    assert_ne!(EndRelease::start_pin(), EndRelease::end_pin());
    assert_eq!(
        EndRelease::both_pins(),
        EndRelease {
            start_rotation: true,
            end_rotation: true
        }
    );
}

// §12 supplementary — Three-member frame with release

#[test]
fn test_three_member_frame_with_release() {
    //   A ====== B
    //   |
    //   D
    // AB released at B, BD rigid. Load at A.

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let d = frame.add_node(0.0, -3.0).unwrap();

    let m_ab = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    let m_bd = frame.add_member(a, d, steel(), section()).unwrap();

    frame.fix(b).unwrap();
    frame.fix(d).unwrap();
    frame.nodal_load(a, 0.0, -8000.0).unwrap();

    let result = frame.solve().unwrap();
    let f_ab = result.member_end_forces(m_ab).unwrap();
    let f_bd = result.member_end_forces(m_bd).unwrap();

    assert!(
        f_ab[5].abs() < 1.0,
        "AB released end moment ~ 0, got {}",
        f_ab[5]
    );
    assert!(
        f_bd[2].abs() + f_bd[5].abs() > 10.0,
        "BD should have non-zero moments"
    );
    assert!(result.equilibrium().is_balanced());
}

// Regression — No release gives identical results to plain member

#[test]
fn test_no_release_identical_to_plain_member() {
    let L = 4.0_f64;
    let P = 10000.0_f64;

    let mut frame0 = FrameModel::new();
    let a0 = frame0.add_node(0.0, 0.0).unwrap();
    let b0 = frame0.add_node(L, 0.0).unwrap();
    frame0.add_member(a0, b0, steel(), section()).unwrap();
    frame0.fix(a0).unwrap();
    frame0.nodal_load(b0, 0.0, -P).unwrap();
    let r0 = frame0.solve().unwrap();

    let mut frame1 = FrameModel::new();
    let a1 = frame1.add_node(0.0, 0.0).unwrap();
    let b1 = frame1.add_node(L, 0.0).unwrap();
    frame1
        .add_member_with_release(a1, b1, steel(), section(), EndRelease::none())
        .unwrap();
    frame1.fix(a1).unwrap();
    frame1.nodal_load(b1, 0.0, -P).unwrap();
    let r1 = frame1.solve().unwrap();

    for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
        let d0 = r0.displacement(a0, dof).unwrap();
        let d1 = r1.displacement(a1, dof).unwrap();
        assert!(
            (d0 - d1).abs() < 1e-12,
            "Displacement mismatch at a: {} vs {}",
            d0,
            d1
        );
        let d0 = r0.displacement(b0, dof).unwrap();
        let d1 = r1.displacement(b1, dof).unwrap();
        assert!(
            (d0 - d1).abs() < 1e-12,
            "Displacement mismatch at b: {} vs {}",
            d0,
            d1
        );
    }

    let f0 = r0.member_end_forces(MemberHandle::from_index(0)).unwrap();
    let f1 = r1.member_end_forces(MemberHandle::from_index(0)).unwrap();
    for i in 0..6 {
        assert!(
            (f0[i] - f1[i]).abs() < 1e-6,
            "Force {} mismatch: {} vs {}",
            i,
            f0[i],
            f1[i]
        );
    }
}
