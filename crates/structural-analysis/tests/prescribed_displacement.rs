//! Tests for prescribed displacement (support settlement) — Phase 111.
//!
//! Covers:
//! - Non-zero prescribed displacement via LoadCase
//! - Zero prescribed displacement equivalence with existing supports
//! - Axial, transverse, and rotational prescribed displacement
//! - Truss prescribed displacement
//! - LoadCase isolation (no cross-case contamination)
//! - LoadCombination rejection
//! - Equilibrium verification
//! - Reaction envelope regression

use section_properties::Material;
use structural_analysis::{BeamSection, Dof, FemError, FrameModel, LoadCase, LoadCombination};

const E: f64 = 200e9;
const NU: f64 = 0.3;
const RHO: f64 = 7850.0;

fn make_steel() -> Material {
    Material::new(E, NU, RHO, "Steel")
}

fn make_cantilever() -> (FrameModel, [structural_analysis::NodeHandle; 2]) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(2.0, 0.0).unwrap();
    frame
        .add_member(a, b, make_steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();
    (frame, [a, b])
}

fn make_fixed_beam() -> (FrameModel, [structural_analysis::NodeHandle; 2]) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    frame
        .add_member(a, b, make_steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    (frame, [a, b])
}

#[test]
fn a_axial_prescribed_displacement() {
    let (frame, [a, b]) = make_fixed_beam();
    let delta = 0.001;

    let mut case = LoadCase::new("settlement");
    case.prescribed_displacement(b, Dof::Ux, delta).unwrap();

    let result = frame.solve_case(&case).unwrap();

    let ux_b = result.displacement(b, Dof::Ux).unwrap();
    assert!(
        (ux_b - delta).abs() < 1e-12,
        "ux_b = {ux_b}, expected {delta}"
    );

    let ux_a = result.displacement(a, Dof::Ux).unwrap();
    assert!(ux_a.abs() < 1e-12, "ux_a = {ux_a}, expected 0");

    let rx_b = result.reaction(b, Dof::Ux).unwrap();
    let area = 5e-3;
    let length = 4.0;
    let expected_force = E * area / length * delta;
    assert!(
        (rx_b - expected_force).abs() / expected_force.abs() < 1e-6,
        "rx_b = {rx_b}, expected {expected_force}"
    );

    assert!(result.equilibrium().is_balanced());
}

#[test]
fn b_zero_prescribed_equivalent_to_no_prescription() {
    let (frame, [a, b]) = make_cantilever();

    let mut case_with_zero = LoadCase::new("zero_settlement");
    case_with_zero.nodal_load(b, 0.0, -1000.0).unwrap();
    case_with_zero
        .prescribed_displacement(a, Dof::Ux, 0.0)
        .unwrap();
    case_with_zero
        .prescribed_displacement(a, Dof::Uy, 0.0)
        .unwrap();
    case_with_zero
        .prescribed_displacement(a, Dof::Rz, 0.0)
        .unwrap();

    let mut case_plain = LoadCase::new("plain");
    case_plain.nodal_load(b, 0.0, -1000.0).unwrap();

    let r1 = frame.solve_case(&case_with_zero).unwrap();
    let r2 = frame.solve_case(&case_plain).unwrap();

    let uy1 = r1.displacement(b, Dof::Uy).unwrap();
    let uy2 = r2.displacement(b, Dof::Uy).unwrap();
    assert!((uy1 - uy2).abs() < 1e-12, "uy1 = {uy1}, uy2 = {uy2}");

    let ry1 = r1.reaction(a, Dof::Uy).unwrap();
    let ry2 = r2.reaction(a, Dof::Uy).unwrap();
    assert!((ry1 - ry2).abs() < 1e-6, "ry1 = {ry1}, ry2 = {ry2}");

    let m1 = r1
        .member_end_forces(structural_analysis::MemberHandle::from_index(0))
        .unwrap();
    let m2 = r2
        .member_end_forces(structural_analysis::MemberHandle::from_index(0))
        .unwrap();
    for i in 0..6 {
        assert!(
            (m1[i] - m2[i]).abs() < 1e-6,
            "end force {i}: {m1:?} vs {m2:?}"
        );
    }
}

#[test]
fn c_transverse_prescribed_displacement() {
    let (frame, [a, b]) = make_fixed_beam();
    let delta = -0.002;

    let mut case = LoadCase::new("transverse_settlement");
    case.prescribed_displacement(b, Dof::Uy, delta).unwrap();

    let result = frame.solve_case(&case).unwrap();

    let uy_b = result.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy_b - delta).abs() < 1e-12,
        "uy_b = {uy_b}, expected {delta}"
    );

    let ry_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        ry_b.abs() > 1.0,
        "ry_b = {ry_b}, expected non-zero reaction"
    );

    assert!(result.equilibrium().is_balanced());
}

#[test]
fn d_rotational_prescribed_displacement() {
    let (frame, [a, b]) = make_fixed_beam();
    let theta = 0.001;

    let mut case = LoadCase::new("rotation_settlement");
    case.prescribed_displacement(b, Dof::Rz, theta).unwrap();

    let result = frame.solve_case(&case).unwrap();

    let rz_b = result.displacement(b, Dof::Rz).unwrap();
    assert!(
        (rz_b - theta).abs() < 1e-12,
        "rz_b = {rz_b}, expected {theta}"
    );

    let mz_b = result.reaction(b, Dof::Rz).unwrap();
    assert!(
        mz_b.abs() > 1.0,
        "mz_b = {mz_b}, expected non-zero moment reaction"
    );

    assert!(result.equilibrium().is_balanced());
}

#[test]
fn e_truss_prescribed_displacement() {
    use structural_analysis::truss::{TrussDof, TrussElement, TrussModel, TrussNode, TrussSolver};

    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 2.0, 3.0));
    model.add_node(TrussNode::new(2, 5.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &make_steel(), 1e-3).unwrap());
    model.add_element(TrussElement::new(1, 2, &make_steel(), 1e-3).unwrap());
    model.add_element(TrussElement::new(0, 2, &make_steel(), 1e-3).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(2, TrussDof::Uy, 0.0).unwrap();

    let delta = 0.0005;
    model.fix_dof(2, TrussDof::Ux, delta).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux2 = solver.displacement(2, TrussDof::Ux).unwrap();
    assert!((ux2 - delta).abs() < 1e-12, "ux2 = {ux2}, expected {delta}");

    let rx2 = solver.reaction(2, TrussDof::Ux).unwrap();
    assert!(rx2.abs() > 1.0, "rx2 = {rx2}, expected non-zero reaction");

    let ux1 = solver.displacement(1, TrussDof::Ux).unwrap();
    let uy1 = solver.displacement(1, TrussDof::Uy).unwrap();
    assert!(
        ux1.is_finite() && uy1.is_finite(),
        "node 1 displacements should be finite"
    );
}

#[test]
fn f_load_case_isolation() {
    let (frame, [a, b]) = make_fixed_beam();

    let mut case_a = LoadCase::new("settlement_a");
    case_a.prescribed_displacement(b, Dof::Uy, -0.001).unwrap();

    let mut case_b = LoadCase::new("settlement_b");
    case_b.prescribed_displacement(b, Dof::Uy, -0.003).unwrap();

    let ra = frame.solve_case(&case_a).unwrap();
    let rb = frame.solve_case(&case_b).unwrap();

    let uy_a = ra.displacement(b, Dof::Uy).unwrap();
    let uy_b = rb.displacement(b, Dof::Uy).unwrap();

    assert!((uy_a - (-0.001)).abs() < 1e-12, "case_a uy = {uy_a}");
    assert!((uy_b - (-0.003)).abs() < 1e-12, "case_b uy = {uy_b}");

    let ry_a = ra.reaction(b, Dof::Uy).unwrap();
    let ry_b = rb.reaction(b, Dof::Uy).unwrap();
    assert!(
        (ry_a - ry_b).abs() > 1.0,
        "reactions should differ: {ry_a} vs {ry_b}"
    );
}

#[test]
fn g_load_combination_rejects_prescribed_displacement() {
    let (frame, [a, b]) = make_fixed_beam();

    let mut dead = LoadCase::new("dead");
    dead.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -500.0).unwrap();
    live.prescribed_displacement(a, Dof::Uy, -0.001).unwrap();

    let mut combo = LoadCombination::new("1.4D + 1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let result = frame.solve_combination(&combo);
    assert!(
        result.is_err(),
        "should reject combination with prescribed displacement"
    );
    match &result.unwrap_err() {
        FemError::InvalidInput(msg) => {
            assert!(msg.contains("prescribed displacements"), "msg: {msg}");
        }
        other => panic!("expected InvalidInput, got {other:?}"),
    }

    let mut combo_ok = LoadCombination::new("1.4D + 1.6L");
    combo_ok.add_case(&dead, 1.4).unwrap();

    let mut live_no_presc = LoadCase::new("live_no_presc");
    live_no_presc.nodal_load(b, 0.0, -500.0).unwrap();
    combo_ok.add_case(&live_no_presc, 1.6).unwrap();

    let ok_result = frame.solve_combination(&combo_ok);
    assert!(
        ok_result.is_ok(),
        "combination without prescribed displacement should succeed"
    );
}

#[test]
fn h_equilibrium_with_prescribed_displacement() {
    let (frame, [a, b]) = make_fixed_beam();

    let mut case = LoadCase::new("mixed");
    case.nodal_load(b, 1000.0, -2000.0).unwrap();
    case.prescribed_displacement(a, Dof::Uy, 0.0005).unwrap();
    case.prescribed_displacement(b, Dof::Rz, 0.001).unwrap();

    let result = frame.solve_case(&case).unwrap();

    let report = result.equilibrium();
    assert!(
        report.is_balanced(),
        "equilibrium not balanced: fx={:.3e} fy={:.3e} mz={:.3e}",
        report.fx_residual,
        report.fy_residual,
        report.mz_residual
    );
}

#[test]
fn i_prepared_analysis_prescribed_override() {
    let (frame, [a, b]) = make_fixed_beam();

    let mut case1 = LoadCase::new("settlement_1");
    case1.prescribed_displacement(a, Dof::Uy, -0.001).unwrap();

    let mut case2 = LoadCase::new("settlement_2");
    case2.prescribed_displacement(a, Dof::Uy, -0.002).unwrap();

    let prepared = frame.prepare().unwrap();
    let r1 = prepared.solve_case(&case1).unwrap();
    let r2 = prepared.solve_case(&case2).unwrap();

    let uy1 = r1.displacement(a, Dof::Uy).unwrap();
    let uy2 = r2.displacement(a, Dof::Uy).unwrap();
    assert!((uy1 - (-0.001)).abs() < 1e-12, "case1 uy = {uy1}");
    assert!((uy2 - (-0.002)).abs() < 1e-12, "case2 uy = {uy2}");

    assert!(r1.equilibrium().is_balanced());
    assert!(r2.equilibrium().is_balanced());
}

#[test]
fn j_prepared_analysis_rejects_new_constraint() {
    let (frame, [a, b]) = make_cantilever();

    let mut case = LoadCase::new("new_constraint");
    case.prescribed_displacement(b, Dof::Uy, -0.001).unwrap();

    let prepared = frame.prepare().unwrap();
    let result = prepared.solve_case(&case);
    assert!(
        result.is_err(),
        "should reject prescribed displacement at free DOF"
    );
    match &result.unwrap_err() {
        FemError::InvalidInput(msg) => {
            assert!(msg.contains("previously free DOF"), "msg: {msg}");
        }
        other => panic!("expected InvalidInput, got {other:?}"),
    }

    let direct_result = frame.solve_case(&case);
    assert!(
        direct_result.is_ok(),
        "direct solve_case should succeed for new constraint"
    );
}

#[test]
fn k_prescribed_displacement_rejects_spring_support() {
    let (mut frame, [_a, b]) = make_cantilever();
    frame.spring(b, Dof::Uy, 1.0e6).unwrap();

    let mut case = LoadCase::new("spring_conflict");
    case.prescribed_displacement(b, Dof::Uy, -0.001).unwrap();

    match frame.solve_case(&case).unwrap_err() {
        FemError::InvalidInput(message) => assert!(message.contains("spring support"), "{message}"),
        other => panic!("expected spring conflict, got {other:?}"),
    }

    let prepared = frame.prepare().unwrap();
    match prepared.solve_case(&case).unwrap_err() {
        FemError::InvalidInput(message) => assert!(message.contains("spring support"), "{message}"),
        other => panic!("expected spring conflict, got {other:?}"),
    }
}

#[test]
fn l_prescribed_displacement_rejects_inclined_roller() {
    let (mut frame, [_a, b]) = make_cantilever();
    frame.inclined_roller(b, 1.0, 1.0, 0.0).unwrap();

    let mut case = LoadCase::new("roller_conflict");
    case.prescribed_displacement(b, Dof::Uy, -0.001).unwrap();

    match frame.solve_case(&case).unwrap_err() {
        FemError::InvalidInput(message) => {
            assert!(message.contains("inclined roller"), "{message}")
        }
        other => panic!("expected roller conflict, got {other:?}"),
    }
}
