//! Phase 114 — 3D Truss MVP tests.
//! Phase 117 — 3D Truss LoadCase / LoadCombination tests.

#![allow(non_snake_case)]

use structural_analysis::LoadCase;
use structural_analysis::LoadCombination;
use structural_analysis::LoadSource;
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

// ---------------------------------------------------------------------------
// Test 12 — axial bar along Y (direction cosines check)
// ---------------------------------------------------------------------------

#[test]
fn test_axial_bar_along_y() {
    let E = 1.0;
    let A = 1.0;
    let L = 2.0;
    let F = 3.0;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 0.0, L, 0.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Ux, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.add_nodal_force(1, 0.0, F, 0.0).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let uy = solver.displacement(1, TrussDof3D::Uy).unwrap();
    let n = solver.axial_force(0).unwrap();
    let expected_uy = F * L / (E * A);
    assert!(
        (uy - expected_uy).abs() / expected_uy.abs() < 1e-10,
        "uy = {uy}, expected {expected_uy}"
    );
    assert!((n - F).abs() / F.abs() < 1e-10, "N = {n}, expected {F}");
}

// ---------------------------------------------------------------------------
// Test 13 — axial bar along Z (direction cosines check)
// ---------------------------------------------------------------------------

#[test]
fn test_axial_bar_along_z() {
    let E = 1.0;
    let A = 1.0;
    let L = 2.0;
    let F = 3.0;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 0.0, 0.0, L));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Ux, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.add_nodal_force(1, 0.0, 0.0, F).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let uz = solver.displacement(1, TrussDof3D::Uz).unwrap();
    let n = solver.axial_force(0).unwrap();
    let expected_uz = F * L / (E * A);
    assert!(
        (uz - expected_uz).abs() / expected_uz.abs() < 1e-10,
        "uz = {uz}, expected {expected_uz}"
    );
    assert!((n - F).abs() / F.abs() < 1e-10, "N = {n}, expected {F}");
}

// ---------------------------------------------------------------------------
// Test 14 — node-order invariance (element i→j vs j→i)
// ---------------------------------------------------------------------------

#[test]
fn test_node_order_invariance() {
    let E = 200e9;
    let A = 1e-4;
    let L = 2.0;
    let F = 1e4;
    let mat = section_properties::Material::new(E, 0.3, 1.0, "steel");

    // Model A: element(0, 1)
    let mut model_a = TrussModel3D::new();
    model_a.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(1, L, 0.0, 0.0));
    model_a.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model_a.add_nodal_force(1, F, 0.0, 0.0).unwrap();
    let mut solver_a = TrussSolver3D::from_model(&model_a).unwrap();
    solver_a.solve_configured().unwrap();

    // Model B: element(1, 0) — reversed node order
    let mut model_b = TrussModel3D::new();
    model_b.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(1, L, 0.0, 0.0));
    model_b.add_element(TrussElement3D::new(1, 0, &mat, A).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model_b.add_nodal_force(1, F, 0.0, 0.0).unwrap();
    let mut solver_b = TrussSolver3D::from_model(&model_b).unwrap();
    solver_b.solve_configured().unwrap();

    let ux_a = solver_a.displacement(1, TrussDof3D::Ux).unwrap();
    let ux_b = solver_b.displacement(1, TrussDof3D::Ux).unwrap();
    assert!((ux_a - ux_b).abs() < 1e-9, "ux differs: A={ux_a}, B={ux_b}");

    let n_a = solver_a.axial_force(0).unwrap();
    let n_b = solver_b.axial_force(0).unwrap();
    assert!(
        (n_a - n_b).abs() < 1e-9,
        "axial force differs: A={n_a}, B={n_b}"
    );
    assert!(n_a > 0.0, "tension should be positive, got {n_a}");
}

// ---------------------------------------------------------------------------
// Test 15 — tetrahedral benchmark (tripod with independent analytical solution)
// ---------------------------------------------------------------------------
//
// Nodes: 0 at origin (free), 1 at (3,0,0), 2 at (0,4,0), 3 at (0,0,5).
// Members 0-1, 0-2, 0-3 with E=A=1.
// Nodes 1-3 fully fixed. Force (1, 2, 3) applied at node 0.
//
// Analytical:
//   K_ff = diag(1/3, 1/4, 1/5)  (each member contributes EA/L along its axis)
//   ux = 1/(1/3) = 3,  uy = 2/(1/4) = 8,  uz = 3/(1/5) = 15
//   N_01 = (1/3)*1*(0-3) = -1  (compression)
//   N_02 = (1/4)*1*(0-8) = -2  (compression)
//   N_03 = (1/5)*1*(0-15) = -3 (compression)
//   R_1 = (-1, 0, 0), R_2 = (0, -2, 0), R_3 = (0, 0, -3)

#[test]
fn test_tetrahedral_benchmark() {
    let E = 1.0;
    let A = 1.0;
    let mat = section_properties::Material::new(E, 0.3, 1.0, "unit");

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 3.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 4.0, 0.0));
    model.add_node(TrussNode3D::new(3, 0.0, 0.0, 5.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, A).unwrap());
    model.add_element(TrussElement3D::new(0, 3, &mat, A).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();
    model.add_nodal_force(0, 1.0, 2.0, 3.0).unwrap();

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    // Displacements
    let ux = solver.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = solver.displacement(0, TrussDof3D::Uy).unwrap();
    let uz = solver.displacement(0, TrussDof3D::Uz).unwrap();
    assert!((ux - 3.0).abs() < 1e-10, "ux = {ux}, expected 3.0");
    assert!((uy - 8.0).abs() < 1e-10, "uy = {uy}, expected 8.0");
    assert!((uz - 15.0).abs() < 1e-10, "uz = {uz}, expected 15.0");

    // Axial forces (compression = negative)
    let n01 = solver.axial_force(0).unwrap();
    let n02 = solver.axial_force(1).unwrap();
    let n03 = solver.axial_force(2).unwrap();
    assert!((n01 - (-1.0)).abs() < 1e-10, "N01 = {n01}, expected -1.0");
    assert!((n02 - (-2.0)).abs() < 1e-10, "N02 = {n02}, expected -2.0");
    assert!((n03 - (-3.0)).abs() < 1e-10, "N03 = {n03}, expected -3.0");

    // Reactions
    let r1x = solver.reaction(1, TrussDof3D::Ux).unwrap();
    let r2y = solver.reaction(2, TrussDof3D::Uy).unwrap();
    let r3z = solver.reaction(3, TrussDof3D::Uz).unwrap();
    assert!((r1x - (-1.0)).abs() < 1e-10, "R1x = {r1x}, expected -1.0");
    assert!((r2y - (-2.0)).abs() < 1e-10, "R2y = {r2y}, expected -2.0");
    assert!((r3z - (-3.0)).abs() < 1e-10, "R3z = {r3z}, expected -3.0");

    // Off-axis reactions should be zero
    let r1y = solver.reaction(1, TrussDof3D::Uy).unwrap();
    let r1z = solver.reaction(1, TrussDof3D::Uz).unwrap();
    assert!(r1y.abs() < 1e-12, "R1y = {r1y}, expected 0");
    assert!(r1z.abs() < 1e-12, "R1z = {r1z}, expected 0");

    // Equilibrium
    let result = solver.results().unwrap();
    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 16 — stable diagnostic for well-constrained 3D truss
// ---------------------------------------------------------------------------

#[test]
fn test_stable_diagnostic() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 1.0, 0.0));
    model.add_node(TrussNode3D::new(3, 0.0, 0.0, 1.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 3, &mat, 1.0).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();

    let solver = TrussSolver3D::from_model(&model).unwrap();
    let diag = solver.diagnostic().unwrap();
    assert!(
        matches!(diag, StructuralDiagnostic::Stable),
        "expected Stable, got {diag}"
    );
}

// ---------------------------------------------------------------------------
// Test 17 — non-finite input rejection
// ---------------------------------------------------------------------------

#[test]
fn test_non_finite_input() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");

    // NaN force
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 0.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    assert!(
        model.add_nodal_force(1, f64::NAN, 0.0, 0.0).is_err(),
        "NaN force should be rejected"
    );

    // Inf force
    assert!(
        model.add_nodal_force(1, 0.0, f64::INFINITY, 0.0).is_err(),
        "Inf force should be rejected"
    );

    // NaN prescribed value
    assert!(
        model.fix_dof(0, TrussDof3D::Ux, f64::NAN).is_err(),
        "NaN prescribed value should be rejected"
    );

    // Inf prescribed value
    assert!(
        model.fix_dof(0, TrussDof3D::Uy, f64::INFINITY).is_err(),
        "Inf prescribed value should be rejected"
    );

    // Invalid node index for force
    assert!(
        model.add_nodal_force(99, 1.0, 0.0, 0.0).is_err(),
        "out-of-bounds node should be rejected"
    );

    // Invalid node index for fix_dof
    assert!(
        model.fix_dof(99, TrussDof3D::Ux, 0.0).is_err(),
        "out-of-bounds node should be rejected for fix_dof"
    );
}

// ===========================================================================
// Phase 117 — LoadCase / LoadCombination tests
// ===========================================================================
//
// Tripod benchmark: node 0 free at origin, nodes 1–3 fixed at (3,0,0),
// (0,4,0), (0,0,5). E=A=1.
//
// Analytical (independent of the FEM implementation):
//   K_ff = diag(1/3, 1/4, 1/5)
//   ux = 3·Fx,  uy = 4·Fy,  uz = 5·Fz
//   N_01 = −Fx,  N_02 = −Fy,  N_03 = −Fz   (compression = negative)
//   R_1x = −Fx,  R_2y = −Fy,  R_3z = −Fz

fn make_tripod() -> TrussModel3D {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 3.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 4.0, 0.0));
    model.add_node(TrussNode3D::new(3, 0.0, 0.0, 5.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 3, &mat, 1.0).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();
    model
}

// ---------------------------------------------------------------------------
// Test 18 — LoadCase X-direction (independent analytical solution)
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_x_direction() {
    let model = make_tripod();
    let fx = 6.0;

    let mut case = LoadCase::new("fx");
    case.nodal_load_3d(0, fx, 0.0, 0.0).unwrap();

    let result = model.solve_case(&case).unwrap();

    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    let uz = result.displacement(0, TrussDof3D::Uz).unwrap();
    assert!(
        (ux - 3.0 * fx).abs() < 1e-10,
        "ux = {ux}, expected {}",
        3.0 * fx
    );
    assert!(uy.abs() < 1e-12, "uy = {uy}");
    assert!(uz.abs() < 1e-12, "uz = {uz}");

    let n01 = result.member_axial_force(0).unwrap();
    assert!((n01 - (-fx)).abs() < 1e-10, "N01 = {n01}, expected {}", -fx);
}

// ---------------------------------------------------------------------------
// Test 19 — LoadCase Y-direction
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_y_direction() {
    let model = make_tripod();
    let fy = 8.0;

    let mut case = LoadCase::new("fy");
    case.nodal_load_3d(0, 0.0, fy, 0.0).unwrap();

    let result = model.solve_case(&case).unwrap();

    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    assert!(
        (uy - 4.0 * fy).abs() < 1e-10,
        "uy = {uy}, expected {}",
        4.0 * fy
    );

    let n02 = result.member_axial_force(1).unwrap();
    assert!((n02 - (-fy)).abs() < 1e-10, "N02 = {n02}, expected {}", -fy);
}

// ---------------------------------------------------------------------------
// Test 20 — LoadCase Z-direction
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_z_direction() {
    let model = make_tripod();
    let fz = 10.0;

    let mut case = LoadCase::new("fz");
    case.nodal_load_3d(0, 0.0, 0.0, fz).unwrap();

    let result = model.solve_case(&case).unwrap();

    let uz = result.displacement(0, TrussDof3D::Uz).unwrap();
    assert!(
        (uz - 5.0 * fz).abs() < 1e-10,
        "uz = {uz}, expected {}",
        5.0 * fz
    );

    let n03 = result.member_axial_force(2).unwrap();
    assert!((n03 - (-fz)).abs() < 1e-10, "N03 = {n03}, expected {}", -fz);
}

// ---------------------------------------------------------------------------
// Test 21 — LoadCase arbitrary 3D member (all three force components)
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_arbitrary_3d() {
    let model = make_tripod();
    let fx = 1.0;
    let fy = 2.0;
    let fz = 3.0;

    let mut case = LoadCase::new("fxyz");
    case.nodal_load_3d(0, fx, fy, fz).unwrap();

    let result = model.solve_case(&case).unwrap();

    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    let uz = result.displacement(0, TrussDof3D::Uz).unwrap();
    assert!((ux - 3.0 * fx).abs() < 1e-10, "ux = {ux}");
    assert!((uy - 4.0 * fy).abs() < 1e-10, "uy = {uy}");
    assert!((uz - 5.0 * fz).abs() < 1e-10, "uz = {uz}");

    let n01 = result.member_axial_force(0).unwrap();
    let n02 = result.member_axial_force(1).unwrap();
    let n03 = result.member_axial_force(2).unwrap();
    assert!((n01 - (-fx)).abs() < 1e-10, "N01 = {n01}");
    assert!((n02 - (-fy)).abs() < 1e-10, "N02 = {n02}");
    assert!((n03 - (-fz)).abs() < 1e-10, "N03 = {n03}");

    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 22 — LoadCase isolation (no cross-case contamination)
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_isolation() {
    let model = make_tripod();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(0, 100.0, 0.0, 0.0).unwrap();

    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(0, 0.0, 200.0, 0.0).unwrap();

    let result_a = model.solve_case(&case_a).unwrap();
    let result_b = model.solve_case(&case_b).unwrap();

    let uy_a = result_a.displacement(0, TrussDof3D::Uy).unwrap();
    let ux_b = result_b.displacement(0, TrussDof3D::Ux).unwrap();
    assert!(
        uy_a.abs() < 1e-12,
        "case A should not produce uy, got {uy_a}"
    );
    assert!(
        ux_b.abs() < 1e-12,
        "case B should not produce ux, got {ux_b}"
    );

    let ux_a = result_a.displacement(0, TrussDof3D::Ux).unwrap();
    let uy_b = result_b.displacement(0, TrussDof3D::Uy).unwrap();
    assert!((ux_a - 300.0).abs() < 1e-9, "ux_a = {ux_a}, expected 300");
    assert!((uy_b - 800.0).abs() < 1e-9, "uy_b = {uy_b}, expected 800");
}

// ---------------------------------------------------------------------------
// Test 23 — LoadCombination
// ---------------------------------------------------------------------------

#[test]
fn test_loadcombination() {
    let model = make_tripod();

    let mut dead = LoadCase::new("dead");
    dead.nodal_load_3d(0, 10.0, 0.0, 0.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load_3d(0, 0.0, 20.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("1.4D + 1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let result = model.solve_combination(&combo).unwrap();

    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    assert!((ux - 3.0 * 1.4 * 10.0).abs() < 1e-9, "ux = {ux}");
    assert!((uy - 4.0 * 1.6 * 20.0).abs() < 1e-9, "uy = {uy}");

    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 24 — Combination equivalence: solve_combination(A,B) == solve(A+B)
// ---------------------------------------------------------------------------

#[test]
fn test_combination_equivalence() {
    let model = make_tripod();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(0, 7.0, 11.0, 13.0).unwrap();

    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(0, 3.0, 5.0, 17.0).unwrap();

    let mut combo = LoadCombination::new("a+b");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();

    let result_combo = model.solve_combination(&combo).unwrap();

    let mut case_sum = LoadCase::new("sum");
    case_sum.nodal_load_3d(0, 10.0, 16.0, 30.0).unwrap();
    let result_sum = model.solve_case(&case_sum).unwrap();

    for dof in [TrussDof3D::Ux, TrussDof3D::Uy, TrussDof3D::Uz] {
        let d_combo = result_combo.displacement(0, dof).unwrap();
        let d_sum = result_sum.displacement(0, dof).unwrap();
        assert!(
            (d_combo - d_sum).abs() < 1e-10,
            "disp mismatch for {dof:?}: combo={d_combo}, sum={d_sum}"
        );
    }

    for i in 0..3 {
        let n_combo = result_combo.member_axial_force(i).unwrap();
        let n_sum = result_sum.member_axial_force(i).unwrap();
        assert!(
            (n_combo - n_sum).abs() < 1e-10,
            "axial force mismatch for element {i}: combo={n_combo}, sum={n_sum}"
        );
    }
}

// ---------------------------------------------------------------------------
// Test 25 — LoadSource provenance
// ---------------------------------------------------------------------------

#[test]
fn test_loadsource_provenance() {
    let model = make_tripod();

    let mut case = LoadCase::new("test_case");
    case.nodal_load_3d(0, 1.0, 2.0, 3.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    match result.load_source() {
        LoadSource::LoadCase {
            name,
            has_prescribed_displacements,
        } => {
            assert_eq!(name, "test_case");
            assert!(!*has_prescribed_displacements);
        }
        other => panic!("expected LoadCase, got {other:?}"),
    }

    let mut dead = LoadCase::new("dead");
    dead.nodal_load_3d(0, 1.0, 0.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_3d(0, 0.0, 1.0, 0.0).unwrap();
    let mut combo = LoadCombination::new("combo");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();
    let result = model.solve_combination(&combo).unwrap();
    match result.load_source() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "combo");
            assert_eq!(terms.len(), 2);
            assert_eq!(terms[0].case_name, "dead");
            assert!((terms[0].factor - 1.4).abs() < 1e-15);
            assert_eq!(terms[1].case_name, "live");
            assert!((terms[1].factor - 1.6).abs() < 1e-15);
        }
        other => panic!("expected LoadCombination, got {other:?}"),
    }

    let mut solver = TrussSolver3D::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results().unwrap();
    assert_eq!(result.load_source(), &LoadSource::ModelLoads);
}

// ---------------------------------------------------------------------------
// Test 26 — Reaction equilibrium via LoadCase
// ---------------------------------------------------------------------------

#[test]
fn test_reaction_equilibrium() {
    let model = make_tripod();
    let fx = 12.0;
    let fy = 20.0;
    let fz = 25.0;

    let mut case = LoadCase::new("eq");
    case.nodal_load_3d(0, fx, fy, fz).unwrap();
    let result = model.solve_case(&case).unwrap();

    let r1x = result.reaction(1, TrussDof3D::Ux).unwrap();
    let r2y = result.reaction(2, TrussDof3D::Uy).unwrap();
    let r3z = result.reaction(3, TrussDof3D::Uz).unwrap();
    assert!((r1x - (-fx)).abs() < 1e-10, "R1x = {r1x}, expected {}", -fx);
    assert!((r2y - (-fy)).abs() < 1e-10, "R2y = {r2y}, expected {}", -fy);
    assert!((r3z - (-fz)).abs() < 1e-10, "R3z = {r3z}, expected {}", -fz);

    let r1y = result.reaction(1, TrussDof3D::Uy).unwrap();
    let r1z = result.reaction(1, TrussDof3D::Uz).unwrap();
    let r2x = result.reaction(2, TrussDof3D::Ux).unwrap();
    let r2z = result.reaction(2, TrussDof3D::Uz).unwrap();
    let r3x = result.reaction(3, TrussDof3D::Ux).unwrap();
    let r3y = result.reaction(3, TrussDof3D::Uy).unwrap();
    assert!(r1y.abs() < 1e-12, "R1y = {r1y}");
    assert!(r1z.abs() < 1e-12, "R1z = {r1z}");
    assert!(r2x.abs() < 1e-12, "R2x = {r2x}");
    assert!(r2z.abs() < 1e-12, "R2z = {r2z}");
    assert!(r3x.abs() < 1e-12, "R3x = {r3x}");
    assert!(r3y.abs() < 1e-12, "R3y = {r3y}");

    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 27 — 2D regression: planar problem in 3D solver matches analytical 2D
// ---------------------------------------------------------------------------

#[test]
fn test_2d_regression() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 3.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 4.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, 1.0).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_dof(0, TrussDof3D::Uz, 0.0).unwrap();

    let fx = 3.0;
    let fy = 8.0;
    let mut case = LoadCase::new("2d");
    case.nodal_load_3d(0, fx, fy, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();

    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    let uz = result.displacement(0, TrussDof3D::Uz).unwrap();
    assert!(
        (ux - 3.0 * fx).abs() < 1e-10,
        "ux = {ux}, expected {}",
        3.0 * fx
    );
    assert!(
        (uy - 4.0 * fy).abs() < 1e-10,
        "uy = {uy}, expected {}",
        4.0 * fy
    );
    assert!(uz.abs() < 1e-12, "uz = {uz}, expected 0");

    let n01 = result.member_axial_force(0).unwrap();
    let n02 = result.member_axial_force(1).unwrap();
    assert!((n01 - (-fx)).abs() < 1e-10, "N01 = {n01}, expected {}", -fx);
    assert!((n02 - (-fy)).abs() < 1e-10, "N02 = {n02}, expected {}", -fy);
}

// ===========================================================================
// Phase 118 — Deep Audit tests
// ===========================================================================

// ---------------------------------------------------------------------------
// Test 28 — Combination with zero and negative factors
// ---------------------------------------------------------------------------

#[test]
fn test_combination_zero_negative_factors() {
    let model = make_tripod();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(0, 10.0, 0.0, 0.0).unwrap();

    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(0, 0.0, 20.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("1.0A + 0.0B - 0.5A");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 0.0).unwrap();
    combo.add_case(&case_a, -0.5).unwrap();

    let result = model.solve_combination(&combo).unwrap();

    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    let uy = result.displacement(0, TrussDof3D::Uy).unwrap();
    assert!(
        (ux - 3.0 * 0.5 * 10.0).abs() < 1e-9,
        "ux = {ux}, expected {}",
        3.0 * 0.5 * 10.0
    );
    assert!(uy.abs() < 1e-12, "uy = {uy}, expected 0 (zero factor on B)");
}

// ---------------------------------------------------------------------------
// Test 29 — Prescribed displacement via LoadCase with concurrent nodal force
// ---------------------------------------------------------------------------

#[test]
fn test_loadcase_prescribed_with_force() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 1.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 2.0, 0.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(1, 2, &mat, 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model.fix_node(2).unwrap();

    let delta = 0.002;
    let f = 5.0;
    let mut case = LoadCase::new("settlement+force");
    case.prescribed_displacement_3d(2, 0, delta).unwrap();
    case.nodal_load_3d(1, f, 0.0, 0.0).unwrap();

    let result = model.solve_case(&case).unwrap();

    let ux1 = result.displacement(1, TrussDof3D::Ux).unwrap();
    assert!(ux1.is_finite(), "ux1 should be finite, got {ux1}");

    let n01 = result.member_axial_force(0).unwrap();
    let n12 = result.member_axial_force(1).unwrap();
    assert!(n01.is_finite(), "N01 should be finite");
    assert!(n12.is_finite(), "N12 should be finite");

    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium not balanced"
    );
}

// ---------------------------------------------------------------------------
// Test 30 — LoadCombination rejects prescribed displacements
// ---------------------------------------------------------------------------

#[test]
fn test_combination_rejects_prescribed() {
    let model = make_tripod();

    let mut case = LoadCase::new("with_settlement");
    case.nodal_load_3d(0, 1.0, 0.0, 0.0).unwrap();
    case.prescribed_displacement_3d(1, 0, 0.001).unwrap();

    let mut combo = LoadCombination::new("bad");
    combo.add_case(&case, 1.0).unwrap();

    let result = model.solve_combination(&combo);
    assert!(
        result.is_err(),
        "combination with prescribed displacement should be rejected"
    );
}

// ---------------------------------------------------------------------------
// Test 31 — Invalid DOF rejected by prescribed_displacement_3d
// ---------------------------------------------------------------------------

#[test]
fn test_prescribed_displacement_3d_invalid_dof() {
    let mut case = LoadCase::new("bad_dof");
    assert!(
        case.prescribed_displacement_3d(0, 3, 0.001).is_err(),
        "dof=3 should be rejected"
    );
    assert!(
        case.prescribed_displacement_3d(0, 99, 0.001).is_err(),
        "dof=99 should be rejected"
    );
    assert!(
        case.prescribed_displacement_3d(0, 0, 0.001).is_ok(),
        "dof=0 should be accepted"
    );
    assert!(
        case.prescribed_displacement_3d(0, 2, 0.001).is_ok(),
        "dof=2 should be accepted"
    );
}

// ---------------------------------------------------------------------------
// Test 32 — Repeated solve isolation (results don't pollute each other)
// ---------------------------------------------------------------------------

#[test]
fn test_repeated_solve_isolation() {
    let model = make_tripod();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(0, 100.0, 0.0, 0.0).unwrap();

    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(0, 0.0, 0.0, 200.0).unwrap();

    let result_a = model.solve_case(&case_a).unwrap();
    let result_b = model.solve_case(&case_b).unwrap();
    let result_a2 = model.solve_case(&case_a).unwrap();

    let ux_a = result_a.displacement(0, TrussDof3D::Ux).unwrap();
    let ux_a2 = result_a2.displacement(0, TrussDof3D::Ux).unwrap();
    assert!(
        (ux_a - ux_a2).abs() < 1e-12,
        "repeated solve should be identical: {ux_a} vs {ux_a2}"
    );

    let uz_b = result_b.displacement(0, TrussDof3D::Uz).unwrap();
    let ux_b = result_b.displacement(0, TrussDof3D::Ux).unwrap();
    assert!(
        ux_b.abs() < 1e-12,
        "case B should not produce ux, got {ux_b}"
    );
    assert!((uz_b - 5.0 * 200.0).abs() < 1e-9, "uz_b = {uz_b}");

    let uz_a = result_a.displacement(0, TrussDof3D::Uz).unwrap();
    assert!(
        uz_a.abs() < 1e-12,
        "case A result should not be polluted by case B: uz_a = {uz_a}"
    );
}

// ---------------------------------------------------------------------------
// Test 33 — Multi-node load accumulation and cancellation
// ---------------------------------------------------------------------------

#[test]
fn test_multi_node_accumulation_cancellation() {
    let mat = section_properties::Material::new(1.0, 0.3, 1.0, "unit");

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 3.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, 0.0, 4.0, 0.0));
    model.add_node(TrussNode3D::new(3, 0.0, 0.0, 5.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, 1.0).unwrap());
    model.add_element(TrussElement3D::new(0, 3, &mat, 1.0).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();

    let mut case_cancel = LoadCase::new("cancel");
    case_cancel.nodal_load_3d(0, 50.0, 0.0, 0.0).unwrap();
    case_cancel.nodal_load_3d(0, -50.0, 0.0, 0.0).unwrap();

    let result = model.solve_case(&case_cancel).unwrap();
    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    assert!(
        ux.abs() < 1e-12,
        "cancelling forces should produce zero displacement, got ux = {ux}"
    );

    let mut case_accum = LoadCase::new("accum");
    case_accum.nodal_load_3d(0, 30.0, 0.0, 0.0).unwrap();
    case_accum.nodal_load_3d(0, 20.0, 0.0, 0.0).unwrap();

    let result = model.solve_case(&case_accum).unwrap();
    let ux = result.displacement(0, TrussDof3D::Ux).unwrap();
    assert!(
        (ux - 3.0 * 50.0).abs() < 1e-10,
        "accumulated 30+20=50, ux = {ux}, expected {}",
        3.0 * 50.0
    );
}

// ===========================================================================
// Phase 120 — 3D Truss Envelope tests
// ===========================================================================

use structural_analysis::FemError;
use structural_analysis::Truss3DEnvelope;

fn steel() -> section_properties::Material {
    section_properties::Material::new(200e9, 0.3, 7850.0, "Steel")
}

/// Simple axial bar along X: node 0 fixed, node 1 free (uy/uz constrained).
fn axial_bar() -> TrussModel3D {
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &steel(), 1e-4).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model
}

/// 3D tripod: apex node 0 supported by 3 fixed nodes.
fn tripod() -> TrussModel3D {
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 3.0));
    model.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(2, -1.0, 1.732, 0.0));
    model.add_node(TrussNode3D::new(3, -1.0, -1.732, 0.0));
    let mat = steel();
    model.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model.add_element(TrussElement3D::new(0, 2, &mat, 1e-4).unwrap());
    model.add_element(TrussElement3D::new(0, 3, &mat, 1e-4).unwrap());
    model.fix_node(1).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();
    model
}

// Test 1 — Single-result envelope: min and max equal the original values
#[test]
fn p120_single_result_envelope_matches_original() {
    let model = axial_bar();
    let mut case = LoadCase::new("single");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&result]).unwrap();

    let ux = result.displacement(1, TrussDof3D::Ux).unwrap();
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.min - ux).abs() < 1e-12);
    assert!((disp.ux.max - ux).abs() < 1e-12);
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(0));

    let axial = result.member_axial_force(0).unwrap();
    let af = envelope.axial_force(0).unwrap();
    assert!((af.axial.min - axial).abs() < 1e-9);
    assert!((af.axial.max - axial).abs() < 1e-9);
}

// Test 2 — Multiple results: component-wise ux/uy/uz minima and maxima
#[test]
fn p120_multiple_results_component_wise_extrema() {
    let model = axial_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_3d(1, 4e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let ux1 = r1.displacement(1, TrussDof3D::Ux).unwrap();
    let ux2 = r2.displacement(1, TrussDof3D::Ux).unwrap();
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.min - ux1).abs() < 1e-12);
    assert!((disp.ux.max - ux2).abs() < 1e-12);
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(1));
}

// Test 3 — Reaction extrema for supported DOFs
#[test]
fn p120_reaction_extrema_for_supported_dofs() {
    let model = axial_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_3d(1, 4e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let support = envelope.support_reaction(0).unwrap();
    assert!(support.rx.is_populated());
    assert!(support.ry.is_populated());
    assert!(support.rz.is_populated());
    let rx1 = r1.reactions[0];
    let rx2 = r2.reactions[0];
    assert!((support.rx.min - rx1.min(rx2)).abs() < 1e-9);
    assert!((support.rx.max - rx1.max(rx2)).abs() < 1e-9);
}

// Test 4 — Free DOFs do not produce artificial zero reaction extrema
#[test]
fn p120_free_dofs_no_artificial_reactions() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&result]).unwrap();

    // Node 1 has ux free (only uy/uz constrained)
    let node1 = envelope.support_reaction(1).unwrap();
    assert!(
        !node1.rx.is_populated(),
        "free DOF rx should not be populated"
    );
    assert!(
        node1.ry.is_populated(),
        "constrained DOF ry should be populated"
    );
    assert!(
        node1.rz.is_populated(),
        "constrained DOF rz should be populated"
    );
}

// Test 5 — Mixed support configurations via prescribed displacement
#[test]
fn p120_mixed_support_configurations() {
    let model = tripod();
    // Case A: normal load, no prescribed displacement
    let mut case_a = LoadCase::new("normal");
    case_a.nodal_load_3d(0, 0.0, 0.0, -1e4).unwrap();
    // Case B: prescribed displacement at node 0 uz (adds a constraint)
    let mut case_b = LoadCase::new("prescribed");
    case_b.prescribed_displacement_3d(0, 2, -0.001).unwrap();
    let r_a = model.solve_case(&case_a).unwrap();
    let r_b = model.solve_case(&case_b).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_a, &r_b]).unwrap();

    // Node 0 rz is free in case_a but constrained in case_b
    let node0 = envelope.support_reaction(0).unwrap();
    assert!(
        node0.rz.is_populated(),
        "rz should be populated from case_b"
    );
    assert_eq!(node0.rz.min_source, Some(1));
    assert_eq!(node0.rz.max_source, Some(1));
}

// Test 6 — Per-member axial-force minimum and maximum
#[test]
fn p120_axial_force_min_max() {
    let model = tripod();
    let mut down = LoadCase::new("down");
    down.nodal_load_3d(0, 0.0, 0.0, -1e4).unwrap();
    let mut up = LoadCase::new("up");
    up.nodal_load_3d(0, 0.0, 0.0, 1e4).unwrap();
    let r_down = model.solve_case(&down).unwrap();
    let r_up = model.solve_case(&up).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_down, &r_up]).unwrap();

    for elem in 0..3 {
        let af = envelope.axial_force(elem).unwrap();
        assert!(af.axial.is_populated());
        assert!(af.axial.min <= af.axial.max);
        let n_down = r_down.member_axial_force(elem).unwrap();
        let n_up = r_up.member_axial_force(elem).unwrap();
        assert!((af.axial.min - n_down.min(n_up)).abs() < 1e-6);
        assert!((af.axial.max - n_down.max(n_up)).abs() < 1e-6);
    }
}

// Test 7 — Correct governing LoadSource for every extremum
#[test]
fn p120_governing_load_source_correct() {
    let model = axial_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_3d(1, 5e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&dead).unwrap();
    let r2 = model.solve_case(&live).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(1));
    assert_eq!(envelope.source(0).unwrap().name(), Some("dead"));
    assert_eq!(envelope.source(1).unwrap().name(), Some("live"));
}

// Test 8 — Results from both solve_case and solve_combination
#[test]
fn p120_case_and_combination_sources() {
    let model = axial_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_3d(1, 2e4, 0.0, 0.0).unwrap();
    let mut combo = LoadCombination::new("1.4D+1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let case_result = model.solve_case(&dead).unwrap();
    let combo_result = model.solve_combination(&combo).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&case_result, &combo_result]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert!(disp.ux.is_populated());
    match envelope.source(1).unwrap() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "1.4D+1.6L");
            assert_eq!(terms.len(), 2);
        }
        other => panic!("expected combination, got {other:?}"),
    }
}

// Test 9 — Deterministic tie behavior
#[test]
fn p120_ties_keep_first_source() {
    let model = axial_bar();
    let mut first = LoadCase::new("first");
    first.nodal_load_3d(1, 2e4, 0.0, 0.0).unwrap();
    let mut second = LoadCase::new("second");
    second.nodal_load_3d(1, 2e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&first).unwrap();
    let r2 = model.solve_case(&second).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(0));
}

// Test 10 — Empty input behavior
#[test]
fn p120_empty_input_rejected() {
    let err = Truss3DEnvelope::from_results(&[]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 11 — Incompatible node/member counts
#[test]
fn p120_incompatible_counts_rejected() {
    let model1 = axial_bar();
    let mut case1 = LoadCase::new("c1");
    case1.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model1.solve_case(&case1).unwrap();

    let mut model2 = TrussModel3D::new();
    model2.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model2.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model2.add_node(TrussNode3D::new(2, 4.0, 0.0, 0.0));
    model2.add_element(TrussElement3D::new(0, 1, &steel(), 1e-4).unwrap());
    model2.add_element(TrussElement3D::new(1, 2, &steel(), 1e-4).unwrap());
    model2.fix_node(0).unwrap();
    model2.fix_node(2).unwrap();
    model2.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model2.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    let mut case2 = LoadCase::new("c2");
    case2.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r2 = model2.solve_case(&case2).unwrap();

    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    match err {
        FemError::InvalidInput(msg) => {
            assert!(msg.contains("node count") || msg.contains("element count"));
        }
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

// Test 12 — Same-count but incompatible topology (now rejected, Phase 127)
#[test]
fn p120_same_count_incompatible_topology() {
    // Model A: bar 0-1
    let mut model_a = TrussModel3D::new();
    model_a.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model_a.add_element(TrussElement3D::new(0, 1, &steel(), 1e-4).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    // Model B: bar 0-1 but different length (same counts, different geometry)
    let mut model_b = TrussModel3D::new();
    model_b.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(1, 5.0, 0.0, 0.0));
    model_b.add_element(TrussElement3D::new(0, 1, &steel(), 1e-4).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case_a).unwrap();
    let r_b = model_b.solve_case(&case_b).unwrap();

    // Phase 127: topology validation now rejects different coordinates
    let err = Truss3DEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 13 — Reordering input results does not change numeric extrema
#[test]
fn p120_reorder_preserves_extrema() {
    let model = axial_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_3d(1, 4e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();

    let env_12 = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();
    let env_21 = Truss3DEnvelope::from_results(&[&r2, &r1]).unwrap();

    let d12 = env_12.node_displacement(1).unwrap();
    let d21 = env_21.node_displacement(1).unwrap();
    assert!((d12.ux.min - d21.ux.min).abs() < 1e-12);
    assert!((d12.ux.max - d21.ux.max).abs() < 1e-12);

    let a12 = env_12.axial_force(0).unwrap();
    let a21 = env_21.axial_force(0).unwrap();
    assert!((a12.axial.min - a21.axial.min).abs() < 1e-9);
    assert!((a12.axial.max - a21.axial.max).abs() < 1e-9);
}

// Test 14 — Analytical 3D truss benchmark
#[test]
fn p120_analytical_benchmark() {
    let E = 200e9;
    let A = 1e-4;
    let L = 3.0;
    let F = 5e4;

    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, L, 0.0, 0.0));
    let mat = section_properties::Material::new(E, 0.3, 1.0, "mat");
    model.add_element(TrussElement3D::new(0, 1, &mat, A).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    let mut case = LoadCase::new("axial");
    case.nodal_load_3d(1, F, 0.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&result]).unwrap();

    let expected_ux = F * L / (E * A);
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.max - expected_ux).abs() / expected_ux.abs() < 1e-10);

    let af = envelope.axial_force(0).unwrap();
    assert!((af.axial.max - F).abs() / F.abs() < 1e-10);

    let support = envelope.support_reaction(0).unwrap();
    assert!((support.rx.max - (-F)).abs() / F.abs() < 1e-10);
}

// Test 15 — Frame Envelope regression (unchanged behavior)
#[test]
fn p120_frame_envelope_regression() {
    use structural_analysis::{BeamSection, Envelope, FrameModel};

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(5.0, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();

    let mut light = LoadCase::new("light");
    light.nodal_load(b, 0.0, -1000.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load(b, 0.0, -4000.0).unwrap();
    let r1 = frame.solve_case(&light).unwrap();
    let r2 = frame.solve_case(&heavy).unwrap();
    let envelope = Envelope::from_frame_results(&[&r1, &r2], 5).unwrap();

    assert_eq!(envelope.n_results, 2);
    assert_eq!(envelope.n_members, 1);
    assert_eq!(envelope.n_nodes, 2);
    let tip = envelope.node_displacement(b.index()).unwrap();
    assert_eq!(tip.uy.min_source, Some(1));
    assert_eq!(tip.uy.max_source, Some(0));
}

// ===========================================================================
// Phase 121 — Audit-driven tests
// ===========================================================================

// Test 16 — Tension and compression axial force signs
#[test]
fn p121_axial_force_tension_compression_signs() {
    let model = axial_bar();
    // Tension: force pulling node 1 away from fixed node 0
    let mut tension = LoadCase::new("tension");
    tension.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    // Compression: force pushing node 1 toward fixed node 0
    let mut compression = LoadCase::new("compression");
    compression.nodal_load_3d(1, -1e4, 0.0, 0.0).unwrap();
    let r_t = model.solve_case(&tension).unwrap();
    let r_c = model.solve_case(&compression).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_t, &r_c]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    let n_t = r_t.member_axial_force(0).unwrap();
    let n_c = r_c.member_axial_force(0).unwrap();
    assert!(
        n_t > 0.0,
        "tension case should produce positive axial force, got {n_t}"
    );
    assert!(
        n_c < 0.0,
        "compression case should produce negative axial force, got {n_c}"
    );
    assert!((af.axial.max - n_t).abs() < 1e-9, "max should be tension");
    assert!(
        (af.axial.min - n_c).abs() < 1e-9,
        "min should be compression"
    );
    assert_eq!(af.axial.max_source, Some(0));
    assert_eq!(af.axial.min_source, Some(1));
}

// Test 17 — constrained_dofs correctly populated from model
#[test]
fn p121_constrained_dofs_match_model() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();

    // Node 0 is fully fixed (ux, uy, uz)
    // Node 1 has uy, uz constrained
    let dofs: Vec<(usize, usize)> = result.constrained_dofs().to_vec();
    assert!(dofs.contains(&(0, 0)), "node 0 ux should be constrained");
    assert!(dofs.contains(&(0, 1)), "node 0 uy should be constrained");
    assert!(dofs.contains(&(0, 2)), "node 0 uz should be constrained");
    assert!(!dofs.contains(&(1, 0)), "node 1 ux should be free");
    assert!(dofs.contains(&(1, 1)), "node 1 uy should be constrained");
    assert!(dofs.contains(&(1, 2)), "node 1 uz should be constrained");
}

// Test 18 — Source index alignment with load_source
#[test]
fn p121_source_index_alignment() {
    let model = axial_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_3d(1, 3e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&dead).unwrap();
    let r2 = model.solve_case(&live).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    assert_eq!(envelope.source(0).unwrap(), r1.load_source());
    assert_eq!(envelope.source(1).unwrap(), r2.load_source());
    assert!(envelope.source(2).is_none());
}

// Test 19 — All three displacement components independently tracked
#[test]
fn p121_all_three_displacement_components() {
    let model = tripod();
    // Load in x direction
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_3d(0, 1e4, 0.0, 0.0).unwrap();
    // Load in y direction
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_3d(0, 0.0, 1e4, 0.0).unwrap();
    // Load in z direction
    let mut fz = LoadCase::new("fz");
    fz.nodal_load_3d(0, 0.0, 0.0, 1e4).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let r_z = model.solve_case(&fz).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_x, &r_y, &r_z]).unwrap();

    let disp = envelope.node_displacement(0).unwrap();
    // Each load case should produce a different governing source for each component
    assert!(disp.ux.is_populated());
    assert!(disp.uy.is_populated());
    assert!(disp.uz.is_populated());

    // Verify values match independent results
    let ux_x = r_x.displacement(0, TrussDof3D::Ux).unwrap();
    let ux_y = r_y.displacement(0, TrussDof3D::Ux).unwrap();
    let ux_z = r_z.displacement(0, TrussDof3D::Ux).unwrap();
    let ux_expected_max = ux_x.max(ux_y).max(ux_z);
    let ux_expected_min = ux_x.min(ux_y).min(ux_z);
    assert!((disp.ux.max - ux_expected_max).abs() < 1e-9);
    assert!((disp.ux.min - ux_expected_min).abs() < 1e-9);
}

// Test 20 — Reaction values match R = K·u − f convention
#[test]
fn p121_reaction_sign_convention() {
    let model = axial_bar();
    let mut case = LoadCase::new("pull");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&result]).unwrap();

    // For a bar pulled in +x at node 1, the reaction at node 0 should be -F in x
    let support = envelope.support_reaction(0).unwrap();
    assert!(
        support.rx.max < 0.0,
        "reaction at fixed node should be negative (opposing), got {}",
        support.rx.max
    );
    assert!((support.rx.max - result.reactions[0]).abs() < 1e-9);
}

// ===========================================================================
// Phase 126 — 3D Truss Envelope Topology Audit regression tests
//
// Audit found P0=0, P1=0, P2=1, P3=0.
// P2: Truss3DEnvelope does not validate node coordinates or element
//     connectivity, while TrussEnvelope (2D) does. This is a documented
//     design limitation (see p120_same_count_incompatible_topology).
// No production code changes — these tests characterize the current
// behavior and fill gaps analogous to Phase 125.
// ===========================================================================

// Test 22 — All-positive axial force extrema (3D)
#[test]
fn p126_all_positive_axial_extrema() {
    let model = axial_bar();
    let mut light_tension = LoadCase::new("light_tension");
    light_tension.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut heavy_tension = LoadCase::new("heavy_tension");
    heavy_tension.nodal_load_3d(1, 3e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&light_tension).unwrap();
    let r2 = model.solve_case(&heavy_tension).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    assert!(
        af.axial.min > 0.0,
        "all-positive min should be > 0, got {}",
        af.axial.min
    );
    assert!(
        af.axial.max > 0.0,
        "all-positive max should be > 0, got {}",
        af.axial.max
    );
    assert!(af.axial.min < af.axial.max, "min should be < max");
    assert_eq!(af.axial.min_source, Some(0));
    assert_eq!(af.axial.max_source, Some(1));
}

// Test 23 — All-negative axial force extrema (3D)
#[test]
fn p126_all_negative_axial_extrema() {
    let model = axial_bar();
    let mut light_comp = LoadCase::new("light_comp");
    light_comp.nodal_load_3d(1, -1e4, 0.0, 0.0).unwrap();
    let mut heavy_comp = LoadCase::new("heavy_comp");
    heavy_comp.nodal_load_3d(1, -3e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&light_comp).unwrap();
    let r2 = model.solve_case(&heavy_comp).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    assert!(
        af.axial.min < 0.0,
        "all-negative min should be < 0, got {}",
        af.axial.min
    );
    assert!(
        af.axial.max < 0.0,
        "all-negative max should be < 0, got {}",
        af.axial.max
    );
    assert!(af.axial.min < af.axial.max, "min should be < max");
    assert_eq!(af.axial.min_source, Some(1));
    assert_eq!(af.axial.max_source, Some(0));
}

// Test 24 — Independent governing source for axial force across elements (3D)
#[test]
fn p126_independent_axial_governing_source() {
    let model = tripod();
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_3d(0, 1e4, 0.0, 0.0).unwrap();
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_3d(0, 0.0, 1e4, 0.0).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_x, &r_y]).unwrap();

    let mut max_sources: Vec<Option<usize>> = (0..3)
        .map(|i| envelope.axial_force(i).unwrap().axial.max_source)
        .collect();
    max_sources.sort();
    let mut min_sources: Vec<Option<usize>> = (0..3)
        .map(|i| envelope.axial_force(i).unwrap().axial.min_source)
        .collect();
    min_sources.sort();
    let both_sources_present = max_sources.contains(&Some(0)) && max_sources.contains(&Some(1))
        || min_sources.contains(&Some(0)) && min_sources.contains(&Some(1));
    assert!(
        both_sources_present,
        "different elements should be governed by different sources, \
         max_sources={max_sources:?}, min_sources={min_sources:?}"
    );
}

// Test 25 — Zero axial force: exact zero is a valid extremum (3D)
#[test]
fn p126_zero_axial_force_extrema() {
    let model = axial_bar();
    let mut zero = LoadCase::new("zero");
    zero.nodal_load_3d(1, 0.0, 0.0, 0.0).unwrap();
    let mut tension = LoadCase::new("tension");
    tension.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r0 = model.solve_case(&zero).unwrap();
    let r1 = model.solve_case(&tension).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r0, &r1]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    assert!(
        af.axial.min.abs() < 1e-6,
        "min should be ~0, got {}",
        af.axial.min
    );
    assert!(
        af.axial.max > 0.0,
        "max should be positive, got {}",
        af.axial.max
    );
}

// Test 26 — Independent governing source for ux/uy/uz displacement (3D)
#[test]
fn p126_independent_displacement_governing_sources() {
    let model = tripod();
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_3d(0, 1e4, 0.0, 0.0).unwrap();
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_3d(0, 0.0, 1e4, 0.0).unwrap();
    let mut fz = LoadCase::new("fz");
    fz.nodal_load_3d(0, 0.0, 0.0, 1e4).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let r_z = model.solve_case(&fz).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r_x, &r_y, &r_z]).unwrap();

    let disp = envelope.node_displacement(0).unwrap();
    assert!(disp.ux.is_populated(), "ux should be populated");
    assert!(disp.uy.is_populated(), "uy should be populated");
    assert!(disp.uz.is_populated(), "uz should be populated");

    let ux_sources = [disp.ux.min_source, disp.ux.max_source];
    let uy_sources = [disp.uy.min_source, disp.uy.max_source];
    let uz_sources = [disp.uz.min_source, disp.uz.max_source];
    let ux_has_unique = ux_sources.contains(&Some(0))
        || ux_sources.contains(&Some(1))
        || ux_sources.contains(&Some(2));
    let uy_has_unique = uy_sources.contains(&Some(0))
        || uy_sources.contains(&Some(1))
        || uy_sources.contains(&Some(2));
    let uz_has_unique = uz_sources.contains(&Some(0))
        || uz_sources.contains(&Some(1))
        || uz_sources.contains(&Some(2));
    assert!(
        ux_has_unique && uy_has_unique && uz_has_unique,
        "all three components should have governing sources, \
         ux={ux_sources:?}, uy={uy_sources:?}, uz={uz_sources:?}"
    );
}

// Test 27 — Different element connectivity (same counts) rejected (3D, Phase 127)
//
// Phase 127 added topology validation: the 3D envelope now rejects results
// with different element connectivity, matching the 2D envelope contract.
#[test]
fn p126_different_connectivity_accepted() {
    let mat = steel();

    // Model A: chain 0-1-2 (elements: (0,1), (1,2))
    let mut model_a = TrussModel3D::new();
    model_a.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(1, 1.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(2, 2.0, 0.0, 0.0));
    model_a.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model_a.add_element(TrussElement3D::new(1, 2, &mat, 1e-4).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    model_a.fix_dof(2, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(2, TrussDof3D::Uz, 0.0).unwrap();

    // Model B: V-shape (elements: (0,2), (1,2)) — same counts, different connectivity
    let mut model_b = TrussModel3D::new();
    model_b.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(2, 1.0, 1.0, 0.0));
    model_b.add_element(TrussElement3D::new(0, 2, &mat, 1e-4).unwrap());
    model_b.add_element(TrussElement3D::new(1, 2, &mat, 1e-4).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_node(1).unwrap();
    model_b.fix_dof(2, TrussDof3D::Uz, 0.0).unwrap();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(2, 1e3, 0.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(2, 1e3, 0.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case_a).unwrap();
    let r_b = model_b.solve_case(&case_b).unwrap();

    // Verify connectivity actually differs
    assert_ne!(
        r_a.element_nodes, r_b.element_nodes,
        "models should have different connectivity"
    );

    // Phase 127: topology validation now rejects different connectivity
    let err = Truss3DEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 28 — Different node coordinates (same counts) rejected (3D, Phase 127)
//
// Phase 127 added topology validation: the 3D envelope now rejects results
// with different node coordinates, matching the 2D envelope contract.
#[test]
fn p126_different_coordinates_accepted() {
    let mat = steel();

    // Model A: bar in xy-plane
    let mut model_a = TrussModel3D::new();
    model_a.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model_a.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    // Model B: bar tilted in z direction — same counts, different coordinates
    let mut model_b = TrussModel3D::new();
    model_b.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(1, 2.0, 0.0, 3.0));
    model_b.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case_a).unwrap();
    let r_b = model_b.solve_case(&case_b).unwrap();

    // Verify coordinates actually differ
    assert_ne!(
        r_a.node_coords, r_b.node_coords,
        "models should have different coordinates"
    );

    // Phase 127: topology validation now rejects different coordinates
    let err = Truss3DEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// ===========================================================================
// Phase 127 — 3D Truss Envelope Topology Validation tests
// ===========================================================================

// Test 29 — Identical topology accepted
#[test]
fn p127_identical_topology_accepted() {
    let model = axial_bar();
    let mut case1 = LoadCase::new("case1");
    case1.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut case2 = LoadCase::new("case2");
    case2.nodal_load_3d(1, 2e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case1).unwrap();
    let r2 = model.solve_case(&case2).unwrap();
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap();
    assert_eq!(envelope.n_results, 2);
    assert_eq!(envelope.n_nodes, 2);
    assert_eq!(envelope.n_elements, 1);
}

// Test 30 — x-coordinate mismatch independently detected
#[test]
fn p127_x_mismatch_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 += 1.0;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 31 — y-coordinate mismatch independently detected
#[test]
fn p127_y_mismatch_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].1 += 1.0;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 32 — z-coordinate mismatch independently detected
#[test]
fn p127_z_mismatch_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].2 += 1.0;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 33 — Near-zero coordinate tolerance: tiny perturbation passes
#[test]
fn p127_near_zero_coordinate_tolerance() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1e-12;
    r2.node_coords[1].1 += 1e-13;
    r2.node_coords[0].2 += 1e-14;
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]);
    assert!(
        envelope.is_ok(),
        "near-zero perturbation within absolute tolerance should pass"
    );
}

// Test 34 — Near-zero coordinate tolerance: larger perturbation rejected
#[test]
fn p127_near_zero_coordinate_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 += 1e-6;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 35 — Large-coordinate tolerance: relative perturbation passes
#[test]
fn p127_large_coordinate_tolerance() {
    let mat = steel();
    let big = 1e8;
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, big, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, big + 5.0, 0.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1e-4;
    let envelope = Truss3DEnvelope::from_results(&[&r1, &r2]);
    assert!(
        envelope.is_ok(),
        "large-coordinate perturbation within relative tolerance should pass"
    );
}

// Test 36 — Large-coordinate tolerance: excessive perturbation rejected
#[test]
fn p127_large_coordinate_rejected() {
    let mat = steel();
    let big = 1e8;
    let mut model = TrussModel3D::new();
    model.add_node(TrussNode3D::new(0, big, 0.0, 0.0));
    model.add_node(TrussNode3D::new(1, big + 5.0, 0.0, 0.0));
    model.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1.0;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 37 — NaN coordinate rejected deterministically
#[test]
fn p127_nan_coordinate_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 = f64::NAN;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 38 — Positive infinity coordinate rejected deterministically
#[test]
fn p127_positive_infinity_coordinate_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].1 = f64::INFINITY;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 39 — Negative infinity coordinate rejected deterministically
#[test]
fn p127_negative_infinity_coordinate_rejected() {
    let model = axial_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].2 = f64::NEG_INFINITY;
    let err = Truss3DEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 40 — Mismatch in a later result does not partially contaminate the envelope
#[test]
fn p127_later_mismatch_no_partial_contamination() {
    let model = axial_bar();
    let mut case0 = LoadCase::new("case0");
    case0.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let mut case1 = LoadCase::new("case1");
    case1.nodal_load_3d(1, 2e4, 0.0, 0.0).unwrap();
    let r0 = model.solve_case(&case0).unwrap();
    let r1 = model.solve_case(&case1).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 += 10.0;
    let result = Truss3DEnvelope::from_results(&[&r0, &r1, &r2]);
    assert!(
        result.is_err(),
        "mismatch in result[2] must reject the entire envelope"
    );
    assert!(matches!(result.unwrap_err(), FemError::InvalidInput(_)));
}

// Test 41 — Reversed element endpoints are rejected (exact match contract)
#[test]
fn p127_reversed_endpoints_rejected() {
    let mat = steel();
    let mut model_a = TrussModel3D::new();
    model_a.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_a.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model_a.add_element(TrussElement3D::new(0, 1, &mat, 1e-4).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_a.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    let mut model_b = TrussModel3D::new();
    model_b.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
    model_b.add_node(TrussNode3D::new(1, 2.0, 0.0, 0.0));
    model_b.add_element(TrussElement3D::new(1, 0, &mat, 1e-4).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uy, 0.0).unwrap();
    model_b.fix_dof(1, TrussDof3D::Uz, 0.0).unwrap();

    let mut case = LoadCase::new("case");
    case.nodal_load_3d(1, 1e4, 0.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case).unwrap();
    let r_b = model_b.solve_case(&case).unwrap();

    assert_ne!(
        r_a.element_nodes, r_b.element_nodes,
        "models should have reversed endpoint ordering"
    );
    let err = Truss3DEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}
