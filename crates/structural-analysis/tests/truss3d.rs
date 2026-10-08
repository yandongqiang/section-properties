//! Phase 114 — 3D Truss MVP tests.

#![allow(non_snake_case)]

use structural_analysis::StructuralDiagnostic;
use structural_analysis::truss3d::{
    TrussDof3D, TrussElement3D, TrussModel3D, TrussNode3D, TrussSolver3D,
};

fn make_bar_x(E: f64, A: f64, L: f64, F: f64) -> (TrussSolver3D, f64, f64) {
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, L, 0.0, 0.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.add_nodal_force(1, F, 0.0, 0.0).unwrap();
    let solver = TrussSolver3D::from_model(&model).unwrap();
    (solver, E, A)
}

// ---------------------------------------------------------------------------
// Test 1 — axial bar along X
// ---------------------------------------------------------------------------

#[test]
fn test_axial_bar_along_x() {
    let E = 200e9;
    let A = 1e-4;
    let L = 2.0;
    let F = 1e4;

    let (mut solver, _, _) = make_bar_x(E, A, L, F);
    solver.solve_configured().unwrap();

    let ux = solver.displacement(1, TrussDof3D::Ux).unwrap();
    let uy = solver.displacement(1, TrussDof3D::Uy).unwrap();
    let uz = solver.displacement(1, TrussDof3D::Uz).unwrap();
    let n = solver.axial_force(0).unwrap();

    let expected_ux = F * L / (E * A);
    assert!(
        (ux - expected_ux).abs() / expected_ux.abs() < 1e-10,
        "ux = {ux}, expected {expected_ux}"
    );
    assert!(uy.abs() < 1e-12, "uy = {uy}");
    assert!(uz.abs() < 1e-12, "uz = {uz}");
    assert!((n - F).abs() / F.abs() < 1e-10, "N = {n}, expected {F}");
}

// ---------------------------------------------------------------------------
// Test 2 — stiffness symmetry for arbitrary direction
// ---------------------------------------------------------------------------

#[test]
fn test_stiffness_symmetry() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");
    let el = TrussElement3D::new(0, 1, &mat, 1.0).unwrap();
    let pi = (0.0, 0.0, 0.0);
    let pj = (1.5, 2.3, 0.7);

    let k = el.global_stiffness(pi, pj).unwrap();

    for (i, row) in k.iter().enumerate() {
        for (j, &kij) in row.iter().enumerate() {
            assert!(
                (kij - k[j][i]).abs() < 1e-14,
                "K[{i}][{j}] = {kij} but K[{j}][{i}] = {}",
                k[j][i]
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Test 3 — tension
// ---------------------------------------------------------------------------

#[test]
fn test_tension() {
    let (mut solver, _, _) = make_bar_x(1.0, 1.0, 1.0, 5.0);
    solver.solve_configured().unwrap();
    let n = solver.axial_force(0).unwrap();
    assert!(n > 0.0, "tension should be positive, got N = {n}");
    assert!((n - 5.0).abs() < 1e-10, "N = {n}, expected 5.0");
}

// ---------------------------------------------------------------------------
// Test 4 — compression
// ---------------------------------------------------------------------------

#[test]
fn test_compression() {
    let (mut solver, _, _) = make_bar_x(1.0, 1.0, 1.0, -5.0);
    solver.solve_configured().unwrap();
    let n = solver.axial_force(0).unwrap();
    assert!(n < 0.0, "compression should be negative, got N = {n}");
    assert!((n - (-5.0)).abs() < 1e-10, "N = {n}, expected -5.0");
}

// ---------------------------------------------------------------------------
// Test 5 — 3D geometry (dx, dy, dz all nonzero)
// ---------------------------------------------------------------------------

#[test]
fn test_3d_geometry() {
    let E = 1.0;
    let A = 1.0;
    let F = 1.0;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 2.0, 3.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.add_nodal_force(1, F, 0.0, 0.0).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let n = solver.axial_force(0).unwrap();
    let L = 14f64.sqrt();
    let expected_n = F * L;
    assert!(
        (n - expected_n).abs() / expected_n.abs() < 1e-10,
        "N = {n}, expected {expected_n}"
    );

    let result = solver.results().unwrap();
    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 6 — nonzero prescribed displacement
// ---------------------------------------------------------------------------

#[test]
fn test_prescribed_displacement() {
    let E = 1.0;
    let A = 1.0;
    let L = 1.0;
    let delta = 0.001;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, L, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 2.0 * L, 0.0, 0.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(1, 2, &mat, A).unwrap());

    model.fix_dof(0, TrussDof3D::Ux, delta).unwrap();
    model.fix_dof(0, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(0, TrussDof3D::Uz, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.fix_node(2).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux1 = solver.displacement(1, TrussDof3D::Ux).unwrap();
    let expected_ux1 = delta / 2.0;
    assert!(
        (ux1 - expected_ux1).abs() < 1e-14,
        "ux1 = {ux1}, expected {expected_ux1}"
    );

    let n01 = solver.axial_force(0).unwrap();
    let n12 = solver.axial_force(1).unwrap();
    let expected_n = -delta / 2.0;
    assert!(
        (n01 - expected_n).abs() < 1e-14,
        "N01 = {n01}, expected {expected_n}"
    );
    assert!(
        (n12 - expected_n).abs() < 1e-14,
        "N12 = {n12}, expected {expected_n}"
    );

    let rx0 = solver.reaction(0, TrussDof3D::Ux).unwrap();
    let expected_rx0 = delta / 2.0;
    assert!(
        (rx0 - expected_rx0).abs() < 1e-14,
        "Rx0 = {rx0}, expected {expected_rx0}"
    );

    let result = solver.results().unwrap();
    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 7 — mixed constraints
// ---------------------------------------------------------------------------

#[test]
fn test_mixed_constraints() {
    let E = 1.0;
    let A = 1.0;
    let F = 1.0;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 1.0, 1.0));
    model.add_node(TrussNode3D::new(2, 2.0, 0.0, 0.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(1, 2, &mat, A).unwrap());

    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.fix_node(2).unwrap();
    model.add_nodal_force(1, F, 0.0, 0.0).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux1 = solver.displacement(1, TrussDof3D::Ux).unwrap();
    assert!(
        ux1.is_finite() && ux1 > 0.0,
        "ux1 should be finite and positive, got {ux1}"
    );

    let n0 = solver.axial_force(0).unwrap();
    let n1 = solver.axial_force(1).unwrap();
    assert!(n0.is_finite(), "N0 should be finite");
    assert!(n1.is_finite(), "N1 should be finite");

    let result = solver.results().unwrap();
    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 8 — mechanism (under-constrained truss)
// ---------------------------------------------------------------------------

#[test]
fn test_mechanism() {
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 1.0, 1.0));
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());

    let solver = TrussSolver3D::from_model(&model).unwrap();
    let diag = solver.diagnostic().unwrap();

    assert!(
        matches!(diag, StructuralDiagnostic::RigidBodyMode { rigid_modes, .. } if rigid_modes >= 5),
        "expected RigidBodyMode with >= 5 rigid modes, got {diag}"
    );
}

// ---------------------------------------------------------------------------
// Test 9 — zero-length member rejection
// ---------------------------------------------------------------------------

#[test]
fn test_zero_length_member() {
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 1.0, 2.0, 3.0));
    model.add_node(TrussNode3D::new(1, 1.0, 2.0, 3.0));
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());

    let result = TrussSolver3D::from_model(&model);
    assert!(result.is_err(), "zero-length member should be rejected");
}

// ---------------------------------------------------------------------------
// Test 10 — invalid material / area rejection
// ---------------------------------------------------------------------------

#[test]
fn test_invalid_material_area() {
    let mat_bad_e = section_properties::Material::new(-1.0, 0.3, 1.0, "bad");
    let result = TrussElement3D::new(0, 1, &mat_bad_e, 1.0);
    assert!(result.is_err(), "E <= 0 should be rejected");

    let mat_good = section_properties::Material::new(1.0, 0.3, 1.0, "good");
    let result = TrussElement3D::new(0, 1, &mat_good, -1.0);
    assert!(result.is_err(), "A <= 0 should be rejected");

    let result = TrussElement3D::new(0, 1, &mat_good, 0.0);
    assert!(result.is_err(), "A = 0 should be rejected");
}

// ---------------------------------------------------------------------------
// Test 11 — equilibrium
// ---------------------------------------------------------------------------

#[test]
fn test_equilibrium() {
    let E = 200e9;
    let A = 5e-3;
    let F = 1e4;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 3.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 4.0, 0.0));
    model.add_node(TrussNode3D::new(3, 0.0, 0.0, 5.0));
    model.add_node(TrussNode3D::new(4, 3.0, 4.0, 5.0));
    let mat = section_properties::Material::new(E, 0.3, 7850.0, "steel");
    model.add_element(TrussElement3D::new(0, 4, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(1, 4, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(2, 4, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(3, 4, &mat, A).unwrap());

    model.fix_node(0).unwrap();
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();
    model.add_nodal_force(4, F, -2.0 * F, 0.5 * F).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let result = solver.results().unwrap();
    let report = result.equilibrium();
    assert!(
        report.is_balanced(),
        "equilibrium not balanced: {:?}",
        report
    );

    let n0 = result.member_axial_force(0).unwrap();
    let n1 = result.member_axial_force(1).unwrap();
    let n2 = result.member_axial_force(2).unwrap();
    let n3 = result.member_axial_force(3).unwrap();
    assert!(n0.is_finite(), "N0 not finite");
    assert!(n1.is_finite(), "N1 not finite");
    assert!(n2.is_finite(), "N2 not finite");
    assert!(n3.is_finite(), "N3 not finite");
}
