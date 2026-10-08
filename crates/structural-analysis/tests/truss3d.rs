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
