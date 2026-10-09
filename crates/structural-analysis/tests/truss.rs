//! Tests for the 2D truss element and solver.

use section_properties::Material;
use section_properties::SolverSelection;
use section_properties::geometry::Point;
use structural_analysis::truss::{TrussDof, TrussElement, TrussModel, TrussNode, TrussSolver};
use structural_analysis::{FemError, LoadCase, LoadCombination, LoadSource};

fn unit_mat() -> Material {
    Material::new(1.0, 0.3, 1.0, "unit")
}

// ===========================================================================
// Element mathematics
// ===========================================================================

#[test]
fn horizontal_element_stiffness() {
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    let pi = Point::new(0.0, 0.0);
    let pj = Point::new(1.0, 0.0);
    let k = el.global_stiffness(pi, pj).unwrap();
    // EA/L = 1, c=1, s=0
    let expected = [
        [1.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [-1.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ];
    for i in 0..4 {
        for j in 0..4 {
            assert!((k[i][j] - expected[i][j]).abs() < 1e-12, "[{i}][{j}]");
        }
    }
}

#[test]
fn vertical_element_stiffness() {
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    let pi = Point::new(0.0, 0.0);
    let pj = Point::new(0.0, 1.0);
    let k = el.global_stiffness(pi, pj).unwrap();
    // EA/L = 1, c=0, s=1
    let expected = [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -1.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 1.0],
    ];
    for i in 0..4 {
        for j in 0..4 {
            assert!((k[i][j] - expected[i][j]).abs() < 1e-12, "[{i}][{j}]");
        }
    }
}

#[test]
fn inclined_element_stiffness_45deg() {
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    let pi = Point::new(0.0, 0.0);
    let pj = Point::new(1.0, 1.0);
    let k = el.global_stiffness(pi, pj).unwrap();
    // L = sqrt(2), EA/L = 1/sqrt(2), c = s = 1/sqrt(2)
    // c² = s² = cs = 0.5, k = 0.5/sqrt(2) = 1/(2*sqrt(2))
    let k_val = 1.0 / (2.0 * std::f64::consts::SQRT_2);
    for i in 0..4 {
        for j in 0..4 {
            assert!(
                (k[i][j] - k[j][i]).abs() < 1e-12,
                "not symmetric [{i}][{j}]"
            );
        }
    }
    assert!((k[0][0] - k_val).abs() < 1e-12);
    assert!((k[0][1] - k_val).abs() < 1e-12);
    assert!((k[1][1] - k_val).abs() < 1e-12);
    assert!((k[2][2] - k_val).abs() < 1e-12);
    assert!((k[0][2] + k_val).abs() < 1e-12);
}

#[test]
fn stiffness_symmetry() {
    let el = TrussElement::new(0, 1, &Material::new(200e9, 0.3, 7850.0, "Steel"), 5e-3).unwrap();
    let pi = Point::new(1.0, 2.0);
    let pj = Point::new(4.0, 6.0);
    let k = el.global_stiffness(pi, pj).unwrap();
    for i in 0..4 {
        for j in 0..4 {
            assert!((k[i][j] - k[j][i]).abs() < 1e-6, "not symmetric [{i}][{j}]");
        }
    }
}

#[test]
fn rigid_body_null_space() {
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    let pi = Point::new(1.0, 3.0);
    let pj = Point::new(4.0, 7.0);
    let k = el.global_stiffness(pi, pj).unwrap();

    // Tx: u = [1, 0, 1, 0]
    let tx = [1.0, 0.0, 1.0, 0.0];
    // Ty: u = [0, 1, 0, 1]
    let ty = [0.0, 1.0, 0.0, 1.0];
    // Rz: u = [-y_i, x_i, -y_j, x_j]
    let rz = [-3.0, 1.0, -7.0, 4.0];

    for u in [tx, ty, rz] {
        let mut ku = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                ku[i] += k[i][j] * u[j];
            }
        }
        for i in 0..4 {
            assert!(
                ku[i].abs() < 1e-9,
                "rigid body mode not in null space: ku[{i}] = {}",
                ku[i]
            );
        }
    }
}

// ===========================================================================
// Input validation
// ===========================================================================

#[test]
fn zero_length_rejected() {
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    let p = Point::new(1.0, 2.0);
    assert!(el.global_stiffness(p, p).is_err());
}

#[test]
fn zero_area_rejected() {
    let result = TrussElement::new(0, 1, &unit_mat(), 0.0);
    assert!(result.is_err());
}

#[test]
fn negative_area_rejected() {
    let result = TrussElement::new(0, 1, &unit_mat(), -1.0);
    assert!(result.is_err());
}

#[test]
fn zero_modulus_rejected() {
    let result = TrussElement::new(0, 1, &Material::new(0.0, 0.3, 1.0, "zero"), 1.0);
    assert!(result.is_err());
}

#[test]
fn nan_area_rejected() {
    let result = TrussElement::new(0, 1, &unit_mat(), f64::NAN);
    assert!(result.is_err());
}

#[test]
fn infinity_area_rejected() {
    let result = TrussElement::new(0, 1, &unit_mat(), f64::INFINITY);
    assert!(result.is_err());
}

#[test]
fn invalid_node_index_in_model() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    // element references node 1 which doesn't exist
    let el = TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap();
    model.add_element(el);
    assert!(TrussSolver::from_model(&model).is_err());
}

// ===========================================================================
// Analytical validation: horizontal bar
// ===========================================================================

#[test]
fn horizontal_bar_tip_load() {
    // Horizontal bar: L=1, E=1, A=1, F=10 at tip in x-direction
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap(); // no lateral stiffness
    model.add_nodal_force(1, 10.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    // delta = FL/EA = 10
    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux - 10.0).abs() < 1e-9, "ux = {ux}, expected 10.0");

    let uy = solver.displacement(1, TrussDof::Uy).unwrap();
    assert!(uy.abs() < 1e-9, "uy = {uy}, expected 0.0");

    // Reaction at support: Rx = -F = -10
    let rx = solver.reaction(0, TrussDof::Ux).unwrap();
    assert!((rx + 10.0).abs() < 1e-9, "rx = {rx}, expected -10.0");

    // Axial force: N = F = 10 (tension)
    let n = solver.axial_force(0).unwrap();
    assert!((n - 10.0).abs() < 1e-9, "N = {n}, expected 10.0 (tension)");
}

#[test]
fn horizontal_bar_compression() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(1, -10.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux + 10.0).abs() < 1e-9, "ux = {ux}, expected -10.0");

    let n = solver.axial_force(0).unwrap();
    assert!(
        (n + 10.0).abs() < 1e-9,
        "N = {n}, expected -10.0 (compression)"
    );
}

// ===========================================================================
// Analytical validation: inclined member
// ===========================================================================

#[test]
fn inclined_member_axial_load() {
    // 45-degree member, L=sqrt(2), E=1, A=1
    // Apply force along the member axis: F = (5, 5) at tip
    // delta = FL/EA = 5*sqrt(2), N = 5*sqrt(2) (tension)
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 1.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    // Constrain perpendicular DOF: fix uy at tip to prevent perpendicular mechanism
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    // Apply Fx = 5 at tip. With uy=0, axial force N = Fx * sqrt(2) = 5*sqrt(2)
    model.add_nodal_force(1, 5.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let s2 = std::f64::consts::SQRT_2;
    let n = solver.axial_force(0).unwrap();
    assert!(
        (n - 5.0 * s2).abs() < 1e-9,
        "N = {n}, expected {}",
        5.0 * s2
    );

    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    // u1x = Fx / K_ff = 5 * 2*sqrt(2) = 10*sqrt(2)
    assert!(
        (ux - 10.0 * s2).abs() < 1e-9,
        "ux = {ux}, expected {}",
        10.0 * s2
    );
}

// ===========================================================================
// Analytical validation: triangular truss
// ===========================================================================

#[test]
fn triangular_truss_vertical_load() {
    // Simple triangular truss:
    //   node 0 at (0, 0) — pinned
    //   node 1 at (2, 0) — roller (uy=0)
    //   node 2 at (1, 1) — free, loaded with (0, -P)
    //
    // Members: 0-2 (left), 1-2 (right), 0-1 (bottom)
    // E=1, A=1 for all members.
    // P = 10 (downward)

    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 2.0, 0.0));
    model.add_node(TrussNode::new(2, 1.0, 1.0));
    model.add_element(TrussElement::new(0, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());

    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    model.add_nodal_force(2, 0.0, -10.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    // By symmetry, the vertical load is split equally between the two inclined members.
    // Each inclined member has L = sqrt(2), angle = 45 degrees.
    // For equilibrium: 2 * N_inclined * sin(45) = P
    // N_inclined = P / (2 * sin(45)) = 10 / sqrt(2) = 5*sqrt(2) (compression)
    let s2 = std::f64::consts::SQRT_2;
    let n_left = solver.axial_force(0).unwrap();
    let n_right = solver.axial_force(1).unwrap();

    // Members 0-2 and 1-2 are in compression (load pushes down, members push back)
    // node_i=0, node_j=2 for left: compression means N < 0
    // node_i=1, node_j=2 for right: compression means N < 0
    assert!(
        (n_left + 5.0 * s2).abs() < 1e-9,
        "N_left = {n_left}, expected {}",
        -5.0 * s2
    );
    assert!(
        (n_right + 5.0 * s2).abs() < 1e-9,
        "N_right = {n_right}, expected {}",
        -5.0 * s2
    );

    // Bottom member 0-1: carries tension from horizontal components of inclined members
    // N_bottom = P/(2*tan(45)) = 10/2 = 5 (tension)
    let n_bottom = solver.axial_force(2).unwrap();
    assert!(
        (n_bottom - 5.0).abs() < 1e-9,
        "N_bottom = {n_bottom}, expected 5.0"
    );

    // Equilibrium check
    let rx0 = solver.reaction(0, TrussDof::Ux).unwrap();
    let ry0 = solver.reaction(0, TrussDof::Uy).unwrap();
    let rx1 = solver.reaction(1, TrussDof::Ux).unwrap();
    let ry1 = solver.reaction(1, TrussDof::Uy).unwrap();

    // Sum Fx = 0
    assert!((rx0 + rx1).abs() < 1e-9, "sum Fx = {}", rx0 + rx1);
    // Sum Fy = 0: Ry0 + Ry1 = P = 10
    assert!(
        (ry0 + ry1 - 10.0).abs() < 1e-9,
        "sum Fy = {}, expected 10.0",
        ry0 + ry1
    );
    // By symmetry: Ry0 = Ry1 = 5
    assert!((ry0 - 5.0).abs() < 1e-9, "Ry0 = {ry0}, expected 5.0");
    assert!((ry1 - 5.0).abs() < 1e-9, "Ry1 = {ry1}, expected 5.0");
}

// ===========================================================================
// Mechanism diagnostics
// ===========================================================================

#[test]
fn insufficient_supports_mechanism() {
    // Single bar with only one DOF constrained — should be unstable
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    // Only fix ux at node 0 — leaves uy free → rigid body mode
    model.fix_dof(0, TrussDof::Ux, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    let result = solver.solve_configured();
    assert!(result.is_err(), "should fail — insufficient supports");
}

#[test]
fn stable_truss_solves() {
    // Well-supported single bar
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(1, 5.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux - 5.0).abs() < 1e-9);

    let diag = solver.diagnostic().unwrap();
    assert!(diag.is_stable(), "should be stable");
}

#[test]
fn diagnostic_on_stable_truss() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 2.0, 0.0));
    model.add_node(TrussNode::new(2, 1.0, 1.0));
    model.add_element(TrussElement::new(0, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(2, 0.0, -10.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let diag = solver.diagnostic().unwrap();
    assert!(diag.is_stable());
    assert!(!diag.is_mechanism());
}

// ===========================================================================
// Two-bar system
// ===========================================================================

#[test]
fn two_bar_system() {
    // Two bars in series: 0 -- 1 -- 2
    // Node 0 fixed, node 2 loaded with Fx = 20
    // Both bars: E=1, A=1, L=1
    // delta_total = F*(L1/EA + L2/EA) = 20*(1+1) = 40
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_node(TrussNode::new(2, 2.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.fix_dof(2, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(2, 20.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux2 = solver.displacement(2, TrussDof::Ux).unwrap();
    assert!((ux2 - 40.0).abs() < 1e-9, "ux2 = {ux2}, expected 40.0");

    let ux1 = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux1 - 20.0).abs() < 1e-9, "ux1 = {ux1}, expected 20.0");

    // Both bars have the same axial force = 20 (tension)
    let n1 = solver.axial_force(0).unwrap();
    let n2 = solver.axial_force(1).unwrap();
    assert!((n1 - 20.0).abs() < 1e-9, "N1 = {n1}, expected 20.0");
    assert!((n2 - 20.0).abs() < 1e-9, "N2 = {n2}, expected 20.0");
}

// ===========================================================================
// Results API
// ===========================================================================

#[test]
fn results_collection() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(1, 10.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let results = solver.results().unwrap();
    assert_eq!(results.n_nodes(), 2);
    assert_eq!(results.n_elements(), 1);
    assert_eq!(results.displacements.len(), 4);
    assert_eq!(results.reactions.len(), 4);
    assert_eq!(results.axial_forces.len(), 1);
    assert!((results.axial_forces[0] - 10.0).abs() < 1e-9);
    assert!(results.solver_name().is_some());
}

// ===========================================================================
// DOF enum
// ===========================================================================

#[test]
fn truss_dof_indexing() {
    assert_eq!(TrussDof::Ux.index(), 0);
    assert_eq!(TrussDof::Uy.index(), 1);
    assert_eq!(TrussDof::Ux.name(), "ux");
    assert_eq!(TrussDof::Uy.name(), "uy");
    assert_eq!(TrussDof::ALL.len(), 2);
}

// ===========================================================================
// Equilibrium
// ===========================================================================

#[test]
fn equilibrium_horizontal_bar() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 3.0, 0.0));
    model.add_element(
        TrussElement::new(0, 1, &Material::new(200e9, 0.3, 7850.0, "Steel"), 5e-3).unwrap(),
    );
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(1, 15e3, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let rx0 = solver.reaction(0, TrussDof::Ux).unwrap();
    let ry0 = solver.reaction(0, TrussDof::Uy).unwrap();

    // Sum Fx = Rx + F = 0 → Rx = -F
    assert!((rx0 + 15e3).abs() < 1e-6, "Rx = {rx0}");
    assert!(ry0.abs() < 1e-6, "Ry = {ry0}");
}

#[test]
fn equilibrium_triangular_truss() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 4.0, 0.0));
    model.add_node(TrussNode::new(2, 2.0, 3.0));
    model.add_element(
        TrussElement::new(0, 2, &Material::new(200e9, 0.3, 7850.0, "Steel"), 5e-3).unwrap(),
    );
    model.add_element(
        TrussElement::new(1, 2, &Material::new(200e9, 0.3, 7850.0, "Steel"), 5e-3).unwrap(),
    );
    model.add_element(
        TrussElement::new(0, 1, &Material::new(200e9, 0.3, 7850.0, "Steel"), 5e-3).unwrap(),
    );
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(2, 5e3, -10e3).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let rx0 = solver.reaction(0, TrussDof::Ux).unwrap();
    let ry0 = solver.reaction(0, TrussDof::Uy).unwrap();
    let rx1 = solver.reaction(1, TrussDof::Ux).unwrap();
    let ry1 = solver.reaction(1, TrussDof::Uy).unwrap();

    // Sum Fx = Rx0 + Rx1 + Fx = 0
    assert!(
        (rx0 + rx1 + 5e3).abs() < 1e-3,
        "sum Fx = {}",
        rx0 + rx1 + 5e3
    );
    // Sum Fy = Ry0 + Ry1 + Fy = 0
    assert!(
        (ry0 + ry1 - 10e3).abs() < 1e-3,
        "sum Fy = {}",
        ry0 + ry1 - 10e3
    );

    // Moment about node 0 (origin): M = x*Fy - y*Fx
    // Node 1 at (4, 0): M1 = 4*ry1 - 0*rx1 = 4*ry1
    // Node 2 at (2, 3): M2 = 2*(-10e3) - 3*(5e3) = -35e3
    // Sum M = 4*ry1 - 35e3 = 0
    let moment = 4.0 * ry1 - 35e3;
    assert!(moment.abs() < 1e-3, "sum M = {moment}");
}

// ===========================================================================
// Solver selection
// ===========================================================================

#[test]
fn explicit_solver_selection() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model.add_nodal_force(1, 10.0, 0.0).unwrap();

    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.set_solver(SolverSelection::named("sparse_lu"));
    solver.solve_configured().unwrap();

    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux - 10.0).abs() < 1e-9);
    assert_eq!(solver.solver_name(), Some("sparse_lu"));
}

// ===========================================================================
// Phase 122 — LoadCase / LoadCombination tests
// ===========================================================================

fn simple_bar() -> TrussModel {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    model
}

fn triangular_truss() -> TrussModel {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 4.0, 0.0));
    model.add_node(TrussNode::new(2, 2.0, 3.0));
    model.add_element(TrussElement::new(0, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_node(1).unwrap();
    model
}

#[test]
fn p122_single_case_matches_direct_load() {
    let model = simple_bar();
    let mut case = LoadCase::new("fx");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let result_case = model.solve_case(&case).unwrap();

    let mut model2 = simple_bar();
    model2.add_nodal_force(1, 10.0, 0.0).unwrap();
    let mut solver = TrussSolver::from_model(&model2).unwrap();
    solver.solve_configured().unwrap();
    let result_direct = solver.results().unwrap();

    assert!(
        (result_case.displacements[2] - result_direct.displacements[2]).abs() < 1e-12,
        "ux mismatch"
    );
    assert!((result_case.displacements[2] - 10.0).abs() < 1e-9);
}

#[test]
fn p122_two_cases_no_pollution() {
    let model = simple_bar();
    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_2d(1, 20.0, 0.0).unwrap();

    let result_a = model.solve_case(&case_a).unwrap();
    let result_b = model.solve_case(&case_b).unwrap();

    assert!(
        (result_a.displacements[2] - 10.0).abs() < 1e-9,
        "case A polluted"
    );
    assert!(
        (result_b.displacements[2] - 20.0).abs() < 1e-9,
        "case B polluted"
    );
}

#[test]
fn p122_combination_linear_superposition() {
    let model = simple_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_2d(1, 20.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("1.4D+1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();
    let result_combo = model.solve_combination(&combo).unwrap();

    let r_dead = model.solve_case(&dead).unwrap();
    let r_live = model.solve_case(&live).unwrap();
    let ux_super = 1.4 * r_dead.displacements[2] + 1.6 * r_live.displacements[2];

    assert!(
        (result_combo.displacements[2] - ux_super).abs() < 1e-9,
        "combo = {}, super = {}",
        result_combo.displacements[2],
        ux_super
    );
    assert!((result_combo.displacements[2] - 46.0).abs() < 1e-9);
}

#[test]
fn p122_fx_fy_independent() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 2.0, 0.0));
    model.add_node(TrussNode::new(2, 1.0, 1.0));
    model.add_element(TrussElement::new(0, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_node(1).unwrap();

    let mut case_fx = LoadCase::new("fx");
    case_fx.nodal_load_2d(2, 10.0, 0.0).unwrap();
    let mut case_fy = LoadCase::new("fy");
    case_fy.nodal_load_2d(2, 0.0, 10.0).unwrap();

    let r_fx = model.solve_case(&case_fx).unwrap();
    let r_fy = model.solve_case(&case_fy).unwrap();

    let ux_fx = r_fx.displacements[4];
    let uy_fx = r_fx.displacements[5];
    let ux_fy = r_fy.displacements[4];
    let uy_fy = r_fy.displacements[5];

    assert!(ux_fx.abs() > 1e-6, "fx should produce nonzero ux");
    assert!(uy_fy.abs() > 1e-6, "fy should produce nonzero uy");
    assert!(
        uy_fx.abs() < 1e-9,
        "fx should not produce uy in symmetric config"
    );
    assert!(
        ux_fy.abs() < 1e-9,
        "fy should not produce ux in symmetric config"
    );
}

#[test]
fn p122_cancellation_same_dof() {
    let model = simple_bar();
    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_2d(1, -10.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("cancel");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();
    let result = model.solve_combination(&combo).unwrap();

    assert!(
        result.displacements.iter().all(|&v| v.abs() < 1e-9),
        "cancellation failed: {:?}",
        result.displacements
    );
    assert!(
        result.axial_forces.iter().all(|&v| v.abs() < 1e-9),
        "axial forces should be zero"
    );
}

#[test]
fn p122_positive_negative_zero_factors() {
    let model = simple_bar();
    let mut case = LoadCase::new("f");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let mut combo_pos = LoadCombination::new("pos");
    combo_pos.add_case(&case, 2.0).unwrap();
    let r_pos = model.solve_combination(&combo_pos).unwrap();
    assert!((r_pos.displacements[2] - 20.0).abs() < 1e-9);

    let mut combo_neg = LoadCombination::new("neg");
    combo_neg.add_case(&case, -1.5).unwrap();
    let r_neg = model.solve_combination(&combo_neg).unwrap();
    assert!((r_neg.displacements[2] + 15.0).abs() < 1e-9);

    let mut combo_zero = LoadCombination::new("zero");
    combo_zero.add_case(&case, 0.0).unwrap();
    let r_zero = model.solve_combination(&combo_zero).unwrap();
    assert!(r_zero.displacements[2].abs() < 1e-9);
}

#[test]
fn p122_invalid_node_error() {
    let model = simple_bar();
    let mut case = LoadCase::new("bad");
    case.nodal_load_2d(5, 10.0, 0.0).unwrap();
    let result = model.solve_case(&case);
    assert!(result.is_err());
}

#[test]
fn p122_duplicate_case_in_combination() {
    let model = simple_bar();
    let mut case = LoadCase::new("f");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("dup");
    combo.add_case(&case, 1.0).unwrap();
    combo.add_case(&case, 1.0).unwrap();
    let result = model.solve_combination(&combo).unwrap();

    assert!(
        (result.displacements[2] - 20.0).abs() < 1e-9,
        "duplicate case should accumulate: got {}",
        result.displacements[2]
    );
}

#[test]
fn p122_empty_combination() {
    let model = simple_bar();
    let combo = LoadCombination::new("empty");
    let result = model.solve_combination(&combo).unwrap();

    assert!(
        result.displacements.iter().all(|&v| v.abs() < 1e-9),
        "empty combo should produce zero displacements"
    );
    assert!(
        result.reactions.iter().all(|&v| v.abs() < 1e-9),
        "empty combo should produce zero reactions"
    );
    assert!(
        result.axial_forces.iter().all(|&v| v.abs() < 1e-9),
        "empty combo should produce zero axial forces"
    );
}

#[test]
fn p122_reaction_equilibrium() {
    let model = triangular_truss();
    let mut case = LoadCase::new("down");
    case.nodal_load_2d(2, 0.0, -100.0).unwrap();
    let result = model.solve_case(&case).unwrap();

    let eq = result.equilibrium();
    assert!(eq.is_balanced(), "equilibrium not balanced: {eq:?}");

    let ry0 = result.reactions[1];
    let ry1 = result.reactions[3];
    assert!(
        (ry0 + ry1 + (-100.0)).abs() < 1e-6,
        "sum Fy != 0: {ry0} + {ry1} - 100"
    );
    assert!((ry0 - 50.0).abs() < 1e-6, "by symmetry Ry0 = 50, got {ry0}");
    assert!((ry1 - 50.0).abs() < 1e-6, "by symmetry Ry1 = 50, got {ry1}");
}

#[test]
fn p122_axial_force_sign() {
    let model = simple_bar();

    let mut case_tension = LoadCase::new("tension");
    case_tension.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r_tension = model.solve_case(&case_tension).unwrap();
    assert!(
        r_tension.axial_forces[0] > 0.0,
        "tension should be positive"
    );

    let mut case_compression = LoadCase::new("compression");
    case_compression.nodal_load_2d(1, -10.0, 0.0).unwrap();
    let r_compression = model.solve_case(&case_compression).unwrap();
    assert!(
        r_compression.axial_forces[0] < 0.0,
        "compression should be negative"
    );

    let mut combo = LoadCombination::new("mixed");
    combo.add_case(&case_tension, 1.0).unwrap();
    combo.add_case(&case_compression, 0.5).unwrap();
    let r_combo = model.solve_combination(&combo).unwrap();
    assert!(
        r_combo.axial_forces[0] > 0.0,
        "1.0*tension + 0.5*compression should be positive"
    );
    assert!((r_combo.axial_forces[0] - 5.0).abs() < 1e-9);
}

#[test]
fn p122_load_source_case() {
    let model = simple_bar();
    let mut case = LoadCase::new("test_case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();

    match result.load_source() {
        LoadSource::LoadCase { name, .. } => {
            assert_eq!(name, "test_case");
        }
        other => panic!("expected LoadCase, got {other:?}"),
    }
}

#[test]
fn p122_load_source_combination() {
    let model = simple_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_2d(1, 5.0, 0.0).unwrap();

    let mut combo = LoadCombination::new("combo");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();
    let result = model.solve_combination(&combo).unwrap();

    match result.load_source() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "combo");
            assert_eq!(terms.len(), 2);
            assert_eq!(terms[0].case_name, "dead");
            assert!((terms[0].factor - 1.4).abs() < 1e-12);
            assert_eq!(terms[1].case_name, "live");
            assert!((terms[1].factor - 1.6).abs() < 1e-12);
        }
        other => panic!("expected LoadCombination, got {other:?}"),
    }
}

#[test]
fn p122_load_source_model_loads() {
    let mut model = simple_bar();
    model.add_nodal_force(1, 10.0, 0.0).unwrap();
    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results().unwrap();

    assert_eq!(result.load_source(), &LoadSource::ModelLoads);
}

#[test]
fn p122_direct_api_regression() {
    let mut model = simple_bar();
    model.add_nodal_force(1, 10.0, 0.0).unwrap();
    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();

    let ux = solver.displacement(1, TrussDof::Ux).unwrap();
    assert!((ux - 10.0).abs() < 1e-9);

    let result = solver.results().unwrap();
    assert!((result.displacements[2] - 10.0).abs() < 1e-9);
    assert!(result.axial_forces[0] > 0.0);
}

#[test]
fn p122_results_not_mutated_by_subsequent_solve() {
    let model = simple_bar();
    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_2d(1, 20.0, 0.0).unwrap();

    let result_a = model.solve_case(&case_a).unwrap();
    let ux_a = result_a.displacements[2];
    let _result_b = model.solve_case(&case_b).unwrap();

    assert!(
        (result_a.displacements[2] - ux_a).abs() < 1e-12,
        "result_a was mutated by subsequent solve"
    );
    assert!((ux_a - 10.0).abs() < 1e-9);
}

#[test]
fn p122_combination_rejects_prescribed_displacement() {
    let model = simple_bar();
    let mut case = LoadCase::new("with_settlement");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    case.prescribed_displacement_2d(0, 0, 0.001).unwrap();

    let mut combo = LoadCombination::new("bad");
    combo.add_case(&case, 1.0).unwrap();
    let result = model.solve_combination(&combo);
    assert!(result.is_err());
}

#[test]
fn p122_prescribed_displacement_2d() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_node(TrussNode::new(2, 2.0, 0.0));
    model.add_node(TrussNode::new(3, 1.0, 1.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model.add_element(TrussElement::new(1, 3, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_node(2).unwrap();
    model.fix_node(3).unwrap();

    let mut case = LoadCase::new("settlement");
    case.prescribed_displacement_2d(2, 0, 0.01).unwrap();
    let result = model.solve_case(&case).unwrap();

    let ux1 = result.displacements[2];
    assert!(
        ux1.abs() > 1e-9,
        "settlement should produce nonzero displacement"
    );
    assert!(
        (ux1 - 0.005).abs() < 1e-9,
        "ux1 = L1/(L1+L2) * delta = 0.5 * 0.01, got {ux1}"
    );
}

#[test]
fn p122_prescribed_displacement_2d_invalid_dof() {
    let mut case = LoadCase::new("bad_dof");
    let result = case.prescribed_displacement_2d(0, 2, 0.01);
    assert!(result.is_err());
}

#[test]
fn p122_nodal_load_2d_non_finite() {
    let mut case = LoadCase::new("bad");
    assert!(case.nodal_load_2d(0, f64::NAN, 0.0).is_err());
    assert!(case.nodal_load_2d(0, 0.0, f64::INFINITY).is_err());
}

// ===========================================================================
// Phase 123 — Cross-model DOF validation tests
// ===========================================================================

#[test]
fn p123_solve_case_rejects_3d_dof_in_nodal_load() {
    let model = simple_bar();
    let mut case = LoadCase::new("cross_model");
    case.nodal_load_3d(1, 10.0, 20.0, 30.0).unwrap();
    let result = model.solve_case(&case);
    assert!(
        matches!(result, Err(FemError::InvalidInput(_))),
        "solve_case should return InvalidInput for DOF >= 2, got {result:?}"
    );
}

#[test]
fn p123_solve_case_rejects_3d_dof_in_prescribed_displacement() {
    let model = simple_bar();
    let mut case = LoadCase::new("cross_model");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    case.prescribed_displacement_3d(0, 2, 0.001).unwrap();
    let result = model.solve_case(&case);
    assert!(
        matches!(result, Err(FemError::InvalidInput(_))),
        "solve_case should return InvalidInput for prescribed DOF >= 2, got {result:?}"
    );
}

#[test]
fn p123_solve_combination_rejects_3d_dof() {
    let model = simple_bar();
    let mut case = LoadCase::new("cross_model");
    case.nodal_load_3d(1, 10.0, 20.0, 30.0).unwrap();

    let mut combo = LoadCombination::new("bad");
    combo.add_case(&case, 1.0).unwrap();
    let result = model.solve_combination(&combo);
    assert!(
        matches!(result, Err(FemError::InvalidInput(_))),
        "solve_combination should return InvalidInput for DOF >= 2, got {result:?}"
    );
}

#[test]
fn p123_solve_combination_rejects_3d_dof_in_second_case() {
    let model = simple_bar();
    let mut case_a = LoadCase::new("valid");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("invalid");
    case_b.nodal_load_3d(0, 5.0, 5.0, 5.0).unwrap();

    let mut combo = LoadCombination::new("mixed");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();
    let result = model.solve_combination(&combo);
    assert!(
        matches!(result, Err(FemError::InvalidInput(_))),
        "solve_combination should reject invalid DOF in second case, got {result:?}"
    );
}

#[test]
fn p123_valid_dofs_0_and_1_unaffected() {
    let model = simple_bar();
    let mut case = LoadCase::new("valid");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let result = model.solve_case(&case);
    assert!(result.is_ok(), "valid DOFs 0 and 1 should not be rejected");
    let r = result.unwrap();
    assert!((r.displacements[2] - 10.0).abs() < 1e-9);
}

// ===========================================================================
// Phase 124 — 2D Truss Envelope tests
// ===========================================================================

use structural_analysis::TrussEnvelope;

// Test 1 — Single-result envelope: min and max equal the original values
#[test]
fn p124_single_result_envelope_matches_original() {
    let model = simple_bar();
    let mut case = LoadCase::new("single");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = TrussEnvelope::from_results(&[&result]).unwrap();

    let ux = result.displacements[2];
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.min - ux).abs() < 1e-12);
    assert!((disp.ux.max - ux).abs() < 1e-12);
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(0));

    let axial = result.axial_forces[0];
    let af = envelope.axial_force(0).unwrap();
    assert!((af.axial.min - axial).abs() < 1e-9);
    assert!((af.axial.max - axial).abs() < 1e-9);
    assert_eq!(envelope.n_results, 1);
}

// Test 2 — Two results: displacement extrema
#[test]
fn p124_two_results_displacement_extrema() {
    let model = simple_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_2d(1, 40.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    let ux1 = r1.displacements[2];
    let ux2 = r2.displacements[2];
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.min - ux1).abs() < 1e-12);
    assert!((disp.ux.max - ux2).abs() < 1e-12);
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(1));
}

// Test 3 — Two results: reaction extrema at supported DOFs
#[test]
fn p124_two_results_reaction_extrema() {
    let model = simple_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_2d(1, 40.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    let support = envelope.support_reaction(0).unwrap();
    assert!(support.rx.is_populated());
    assert!(support.ry.is_populated());
    let rx1 = r1.reactions[0];
    let rx2 = r2.reactions[0];
    assert!((support.rx.min - rx1.min(rx2)).abs() < 1e-9);
    assert!((support.rx.max - rx1.max(rx2)).abs() < 1e-9);
    assert_eq!(support.rx.min_source, Some(1));
    assert_eq!(support.rx.max_source, Some(0));
}

// Test 4 — Axial force tension and compression signs
#[test]
fn p124_axial_force_tension_compression() {
    let model = simple_bar();
    let mut tension = LoadCase::new("tension");
    tension.nodal_load_2d(1, 1e4, 0.0).unwrap();
    let mut compression = LoadCase::new("compression");
    compression.nodal_load_2d(1, -1e4, 0.0).unwrap();
    let r_t = model.solve_case(&tension).unwrap();
    let r_c = model.solve_case(&compression).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_t, &r_c]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    let n_t = r_t.axial_forces[0];
    let n_c = r_c.axial_forces[0];
    assert!(n_t > 0.0, "tension should be positive, got {n_t}");
    assert!(n_c < 0.0, "compression should be negative, got {n_c}");
    assert!((af.axial.max - n_t).abs() < 1e-6, "max should be tension");
    assert!(
        (af.axial.min - n_c).abs() < 1e-6,
        "min should be compression"
    );
    assert_eq!(af.axial.max_source, Some(0));
    assert_eq!(af.axial.min_source, Some(1));
}

// Test 5 — Governing source correctly paired with extremum
#[test]
fn p124_governing_source_correct() {
    let model = simple_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_2d(1, 50.0, 0.0).unwrap();
    let r1 = model.solve_case(&dead).unwrap();
    let r2 = model.solve_case(&live).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(1));
    assert_eq!(envelope.source(0).unwrap().name(), Some("dead"));
    assert_eq!(envelope.source(1).unwrap().name(), Some("live"));
}

// Test 6 — Multiple LoadCases
#[test]
fn p124_multiple_load_cases() {
    let model = triangular_truss();
    let mut case_a = LoadCase::new("case_a");
    case_a.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut case_b = LoadCase::new("case_b");
    case_b.nodal_load_2d(2, 0.0, -2e3).unwrap();
    let mut case_c = LoadCase::new("case_c");
    case_c.nodal_load_2d(2, -3e3, 1e3).unwrap();
    let r_a = model.solve_case(&case_a).unwrap();
    let r_b = model.solve_case(&case_b).unwrap();
    let r_c = model.solve_case(&case_c).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_a, &r_b, &r_c]).unwrap();

    assert_eq!(envelope.n_results, 3);
    let disp = envelope.node_displacement(2).unwrap();
    assert!(disp.ux.is_populated());
    assert!(disp.uy.is_populated());
    for i in 0..3 {
        let af = envelope.axial_force(i).unwrap();
        assert!(af.axial.is_populated());
    }
}

// Test 7 — Multiple LoadCombinations
#[test]
fn p124_multiple_load_combinations() {
    let model = simple_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_2d(1, 20.0, 0.0).unwrap();

    let mut combo1 = LoadCombination::new("1.4D");
    combo1.add_case(&dead, 1.4).unwrap();
    let mut combo2 = LoadCombination::new("1.4D+1.6L");
    combo2.add_case(&dead, 1.4).unwrap();
    combo2.add_case(&live, 1.6).unwrap();

    let r1 = model.solve_combination(&combo1).unwrap();
    let r2 = model.solve_combination(&combo2).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert!(disp.ux.is_populated());
    match envelope.source(0).unwrap() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "1.4D");
            assert_eq!(terms.len(), 1);
        }
        other => panic!("expected combination, got {other:?}"),
    }
    match envelope.source(1).unwrap() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "1.4D+1.6L");
            assert_eq!(terms.len(), 2);
        }
        other => panic!("expected combination, got {other:?}"),
    }
}

// Test 8 — Repeated source names remain unambiguous by index
#[test]
fn p124_repeated_source_names() {
    let model = simple_bar();
    let mut first = LoadCase::new("same_name");
    first.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut second = LoadCase::new("same_name");
    second.nodal_load_2d(1, 30.0, 0.0).unwrap();
    let r1 = model.solve_case(&first).unwrap();
    let r2 = model.solve_case(&second).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    assert_eq!(envelope.source(0).unwrap().name(), Some("same_name"));
    assert_eq!(envelope.source(1).unwrap().name(), Some("same_name"));
    let disp = envelope.node_displacement(1).unwrap();
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(1));
}

// Test 9 — Tie behavior: first input result wins
#[test]
fn p124_ties_keep_first_source() {
    let model = simple_bar();
    let mut first = LoadCase::new("first");
    first.nodal_load_2d(1, 20.0, 0.0).unwrap();
    let mut second = LoadCase::new("second");
    second.nodal_load_2d(1, 20.0, 0.0).unwrap();
    let r1 = model.solve_case(&first).unwrap();
    let r2 = model.solve_case(&second).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    let disp = envelope.node_displacement(1).unwrap();
    assert_eq!(disp.ux.min_source, Some(0));
    assert_eq!(disp.ux.max_source, Some(0));
}

// Test 10 — Empty input rejected
#[test]
fn p124_empty_input_rejected() {
    let err = TrussEnvelope::from_results(&[]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 11 — Mismatched node counts rejected
#[test]
fn p124_mismatched_node_counts_rejected() {
    let model1 = simple_bar();
    let mut case1 = LoadCase::new("c1");
    case1.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model1.solve_case(&case1).unwrap();

    let mut model2 = TrussModel::new();
    model2.add_node(TrussNode::new(0, 0.0, 0.0));
    model2.add_node(TrussNode::new(1, 1.0, 0.0));
    model2.add_node(TrussNode::new(2, 2.0, 0.0));
    model2.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model2.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model2.fix_node(0).unwrap();
    model2.fix_node(2).unwrap();
    model2.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    let mut case2 = LoadCase::new("c2");
    case2.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r2 = model2.solve_case(&case2).unwrap();

    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 12 — Same node count but different node coordinates rejected
#[test]
fn p124_same_count_different_node_coords_rejected() {
    let mut model_a = TrussModel::new();
    model_a.add_node(TrussNode::new(0, 0.0, 0.0));
    model_a.add_node(TrussNode::new(1, 2.0, 0.0));
    model_a.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    let mut model_b = TrussModel::new();
    model_b.add_node(TrussNode::new(0, 0.0, 0.0));
    model_b.add_node(TrussNode::new(1, 5.0, 0.0));
    model_b.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case_a).unwrap();
    let r_b = model_b.solve_case(&case_b).unwrap();

    let err = TrussEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    match err {
        FemError::InvalidInput(msg) => assert!(msg.contains("coordinates differ")),
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

// Test 13 — Different element connectivity rejected
#[test]
fn p124_different_element_connectivity_rejected() {
    let mut model_a = TrussModel::new();
    model_a.add_node(TrussNode::new(0, 0.0, 0.0));
    model_a.add_node(TrussNode::new(1, 1.0, 0.0));
    model_a.add_node(TrussNode::new(2, 2.0, 0.0));
    model_a.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model_a.add_element(TrussElement::new(1, 2, &unit_mat(), 1.0).unwrap());
    model_a.fix_node(0).unwrap();
    model_a.fix_node(2).unwrap();
    model_a.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    let mut model_b = TrussModel::new();
    model_b.add_node(TrussNode::new(0, 0.0, 0.0));
    model_b.add_node(TrussNode::new(1, 1.0, 0.0));
    model_b.add_node(TrussNode::new(2, 2.0, 0.0));
    model_b.add_element(TrussElement::new(0, 2, &unit_mat(), 1.0).unwrap());
    model_b.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model_b.fix_node(0).unwrap();
    model_b.fix_node(2).unwrap();
    model_b.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    let mut case_a = LoadCase::new("a");
    case_a.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r_a = model_a.solve_case(&case_a).unwrap();
    let r_b = model_b.solve_case(&case_b).unwrap();

    let err = TrussEnvelope::from_results(&[&r_a, &r_b]).unwrap_err();
    match err {
        FemError::InvalidInput(msg) => assert!(msg.contains("connectivity")),
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

// Test 14 — Different constraint sets: DOF constrained in some results only
#[test]
fn p124_different_constraint_sets() {
    let model = triangular_truss();
    let mut normal = LoadCase::new("normal");
    normal.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut prescribed = LoadCase::new("prescribed");
    prescribed.prescribed_displacement_2d(2, 0, 0.001).unwrap();
    let r_n = model.solve_case(&normal).unwrap();
    let r_p = model.solve_case(&prescribed).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_n, &r_p]).unwrap();

    let node2 = envelope.support_reaction(2).unwrap();
    assert!(
        node2.rx.is_populated(),
        "rx should be populated from prescribed case"
    );
    assert_eq!(node2.rx.min_source, Some(1));
    assert_eq!(node2.rx.max_source, Some(1));
}

// Test 15 — Non-finite values rejected
#[test]
fn p124_non_finite_values_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut result = model.solve_case(&case).unwrap();
    result.displacements[0] = f64::NAN;
    let err = TrussEnvelope::from_results(&[&result]).unwrap_err();
    match err {
        FemError::InvalidInput(msg) => assert!(msg.contains("non-finite")),
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

// Test 16 — Direct load solve result compatible with envelope
#[test]
fn p124_direct_load_result_compatible() {
    let mut model = simple_bar();
    model.add_nodal_force(1, 15.0, 0.0).unwrap();
    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results().unwrap();
    let envelope = TrussEnvelope::from_results(&[&result]).unwrap();

    assert_eq!(envelope.n_results, 1);
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.max - 15.0).abs() < 1e-9);
    match envelope.source(0).unwrap() {
        LoadSource::ModelLoads => {}
        other => panic!("expected ModelLoads, got {other:?}"),
    }
}

// Test 17 — Analytical benchmark: ux = FL/EA, N = F, R = -F
#[test]
fn p124_analytical_benchmark() {
    let e = 200e9;
    let a = 1e-4;
    let l = 3.0;
    let f = 5e4;

    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, l, 0.0));
    let mat = Material::new(e, 0.3, 1.0, "mat");
    model.add_element(TrussElement::new(0, 1, &mat, a).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();

    let mut case = LoadCase::new("axial");
    case.nodal_load_2d(1, f, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = TrussEnvelope::from_results(&[&result]).unwrap();

    let expected_ux = f * l / (e * a);
    let disp = envelope.node_displacement(1).unwrap();
    assert!((disp.ux.max - expected_ux).abs() / expected_ux.abs() < 1e-10);

    let af = envelope.axial_force(0).unwrap();
    assert!((af.axial.max - f).abs() / f.abs() < 1e-10);

    let support = envelope.support_reaction(0).unwrap();
    assert!((support.rx.max - (-f)).abs() / f.abs() < 1e-10);
}

// Test 18 — Reordering input results preserves numeric extrema
#[test]
fn p124_reorder_preserves_extrema() {
    let model = simple_bar();
    let mut light = LoadCase::new("light");
    light.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load_2d(1, 40.0, 0.0).unwrap();
    let r1 = model.solve_case(&light).unwrap();
    let r2 = model.solve_case(&heavy).unwrap();

    let env_12 = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();
    let env_21 = TrussEnvelope::from_results(&[&r2, &r1]).unwrap();

    let d12 = env_12.node_displacement(1).unwrap();
    let d21 = env_21.node_displacement(1).unwrap();
    assert!((d12.ux.min - d21.ux.min).abs() < 1e-12);
    assert!((d12.ux.max - d21.ux.max).abs() < 1e-12);

    let a12 = env_12.axial_force(0).unwrap();
    let a21 = env_21.axial_force(0).unwrap();
    assert!((a12.axial.min - a21.axial.min).abs() < 1e-9);
    assert!((a12.axial.max - a21.axial.max).abs() < 1e-9);
}

// Test 19 — Free DOFs do not produce artificial reaction extrema
#[test]
fn p124_free_dofs_not_populated() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 1e4, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = TrussEnvelope::from_results(&[&result]).unwrap();

    let node1 = envelope.support_reaction(1).unwrap();
    assert!(
        !node1.rx.is_populated(),
        "free DOF rx should not be populated"
    );
    assert!(
        node1.ry.is_populated(),
        "constrained DOF ry should be populated"
    );
}

// Test 20 — constrained_dofs correctly populated from model
#[test]
fn p124_constrained_dofs_match_model() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();

    let dofs: Vec<(usize, usize)> = result.constrained_dofs().to_vec();
    assert!(dofs.contains(&(0, 0)), "node 0 ux should be constrained");
    assert!(dofs.contains(&(0, 1)), "node 0 uy should be constrained");
    assert!(!dofs.contains(&(1, 0)), "node 1 ux should be free");
    assert!(dofs.contains(&(1, 1)), "node 1 uy should be constrained");
}

// Test 21 — Source index alignment with load_source
#[test]
fn p124_source_index_alignment() {
    let model = simple_bar();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load_2d(1, 30.0, 0.0).unwrap();
    let r1 = model.solve_case(&dead).unwrap();
    let r2 = model.solve_case(&live).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

    assert_eq!(envelope.source(0).unwrap(), r1.load_source());
    assert_eq!(envelope.source(1).unwrap(), r2.load_source());
    assert!(envelope.source(2).is_none());
}

// Test 22 — Both ux and uy independently tracked
#[test]
fn p124_both_displacement_components_independent() {
    let model = triangular_truss();
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_2d(2, 0.0, 1e3).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_x, &r_y]).unwrap();

    let disp = envelope.node_displacement(2).unwrap();
    assert!(disp.ux.is_populated());
    assert!(disp.uy.is_populated());

    let ux_x = r_x.displacements[4];
    let ux_y = r_y.displacements[4];
    assert!((disp.ux.max - ux_x.max(ux_y)).abs() < 1e-9);
    assert!((disp.ux.min - ux_x.min(ux_y)).abs() < 1e-9);
}

// Test 23 — Reaction sign convention: R = K·u - f
#[test]
fn p124_reaction_sign_convention() {
    let model = simple_bar();
    let mut case = LoadCase::new("pull");
    case.nodal_load_2d(1, 1e4, 0.0).unwrap();
    let result = model.solve_case(&case).unwrap();
    let envelope = TrussEnvelope::from_results(&[&result]).unwrap();

    let support = envelope.support_reaction(0).unwrap();
    assert!(
        support.rx.max < 0.0,
        "reaction at fixed node should be negative (opposing), got {}",
        support.rx.max
    );
    assert!((support.rx.max - result.reactions[0]).abs() < 1e-9);
}

// ===========================================================================
// Phase 125 — Audit-driven regression tests
// ===========================================================================

// Test 24 — All-positive axial force extrema
#[test]
fn p125_all_positive_axial_extrema() {
    let model = simple_bar();
    let mut light_tension = LoadCase::new("light_tension");
    light_tension.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut heavy_tension = LoadCase::new("heavy_tension");
    heavy_tension.nodal_load_2d(1, 30.0, 0.0).unwrap();
    let r1 = model.solve_case(&light_tension).unwrap();
    let r2 = model.solve_case(&heavy_tension).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

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

// Test 25 — All-negative axial force extrema
#[test]
fn p125_all_negative_axial_extrema() {
    let model = simple_bar();
    let mut light_comp = LoadCase::new("light_comp");
    light_comp.nodal_load_2d(1, -10.0, 0.0).unwrap();
    let mut heavy_comp = LoadCase::new("heavy_comp");
    heavy_comp.nodal_load_2d(1, -30.0, 0.0).unwrap();
    let r1 = model.solve_case(&light_comp).unwrap();
    let r2 = model.solve_case(&heavy_comp).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();

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

// Test 26 — Independent governing source for axial force across elements
#[test]
fn p125_independent_axial_governing_source() {
    let model = triangular_truss();
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_2d(2, 0.0, -2e3).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_x, &r_y]).unwrap();

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

// Test 27 — Near-zero coordinate tolerance: tiny perturbation passes
#[test]
fn p125_near_zero_coordinate_tolerance() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1e-12;
    r2.node_coords[1].1 += 1e-13;
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]);
    assert!(
        envelope.is_ok(),
        "near-zero perturbation within absolute tolerance should pass"
    );
}

// Test 28 — Near-zero coordinate tolerance: larger perturbation rejected
#[test]
fn p125_near_zero_coordinate_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 += 1e-6;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 29 — Large-coordinate tolerance: relative perturbation passes
#[test]
fn p125_large_coordinate_tolerance() {
    let mut model = TrussModel::new();
    let big = 1e8;
    model.add_node(TrussNode::new(0, big, 0.0));
    model.add_node(TrussNode::new(1, big + 5.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1e-4;
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]);
    assert!(
        envelope.is_ok(),
        "large-coordinate perturbation within relative tolerance should pass"
    );
}

// Test 30 — Large-coordinate tolerance: excessive perturbation rejected
#[test]
fn p125_large_coordinate_rejected() {
    let mut model = TrussModel::new();
    let big = 1e8;
    model.add_node(TrussNode::new(0, big, 0.0));
    model.add_node(TrussNode::new(1, big + 5.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_dof(1, TrussDof::Uy, 0.0).unwrap();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].0 += 1.0;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 31 — Independent governing source for ux and uy displacement
#[test]
fn p125_independent_displacement_governing_sources() {
    let model = triangular_truss();
    let mut fx = LoadCase::new("fx");
    fx.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut fy = LoadCase::new("fy");
    fy.nodal_load_2d(2, 0.0, 1e3).unwrap();
    let r_x = model.solve_case(&fx).unwrap();
    let r_y = model.solve_case(&fy).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r_x, &r_y]).unwrap();

    let disp = envelope.node_displacement(2).unwrap();
    let ux_sources = [disp.ux.min_source, disp.ux.max_source];
    let uy_sources = [disp.uy.min_source, disp.uy.max_source];
    let ux_has_both = ux_sources.contains(&Some(0)) && ux_sources.contains(&Some(1));
    let uy_has_both = uy_sources.contains(&Some(0)) && uy_sources.contains(&Some(1));
    assert!(
        ux_has_both || uy_has_both,
        "ux and uy should have independent governing sources, \
         ux={ux_sources:?}, uy={uy_sources:?}"
    );
}

// Test 32 — Zero axial force: exact zero is a valid extremum
#[test]
fn p125_zero_axial_force_extrema() {
    let model = simple_bar();
    let mut zero = LoadCase::new("zero");
    zero.nodal_load_2d(1, 0.0, 0.0).unwrap();
    let mut tension = LoadCase::new("tension");
    tension.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r0 = model.solve_case(&zero).unwrap();
    let r1 = model.solve_case(&tension).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r0, &r1]).unwrap();

    let af = envelope.axial_force(0).unwrap();
    assert!(
        af.axial.min.abs() < 1e-12,
        "min should be ~0, got {}",
        af.axial.min
    );
    assert!(
        af.axial.max > 0.0,
        "max should be positive, got {}",
        af.axial.max
    );
}

// ===========================================================================
// Phase 128 — Reject non-finite 2D truss envelope coordinates
//
// Phase 125 P3 found that NaN/∞ coordinates could pass 2D topology validation
// due to IEEE 754 semantics. Phase 127 added is_finite() guards to 3D.
// This phase adds the same guard to 2D, achieving parity.
// ===========================================================================

// Test 33 — NaN in x-coordinate rejected
#[test]
fn p128_nan_x_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 = f64::NAN;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 34 — NaN in y-coordinate rejected
#[test]
fn p128_nan_y_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].1 = f64::NAN;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 35 — Positive infinity rejected
#[test]
fn p128_positive_infinity_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].0 = f64::INFINITY;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 36 — Negative infinity rejected
#[test]
fn p128_negative_infinity_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let r1 = model.solve_case(&case).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[0].1 = f64::NEG_INFINITY;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 37 — Non-finite in reference result rejected
#[test]
fn p128_non_finite_reference_rejected() {
    let model = simple_bar();
    let mut case = LoadCase::new("case");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut r1 = model.solve_case(&case).unwrap();
    let r2 = r1.clone();
    r1.node_coords[0].0 = f64::NAN;
    let err = TrussEnvelope::from_results(&[&r1, &r2]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

// Test 38 — Non-finite in later candidate rejected (no partial contamination)
#[test]
fn p128_non_finite_later_candidate_rejected() {
    let model = simple_bar();
    let mut case0 = LoadCase::new("case0");
    case0.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case1 = LoadCase::new("case1");
    case1.nodal_load_2d(1, 20.0, 0.0).unwrap();
    let r0 = model.solve_case(&case0).unwrap();
    let r1 = model.solve_case(&case1).unwrap();
    let mut r2 = r1.clone();
    r2.node_coords[1].1 = f64::INFINITY;
    let result = TrussEnvelope::from_results(&[&r0, &r1, &r2]);
    assert!(
        result.is_err(),
        "non-finite in result[2] must reject the entire envelope"
    );
    assert!(matches!(result.unwrap_err(), FemError::InvalidInput(_)));
}

// Test 39 — Finite matching topology still accepted
#[test]
fn p128_finite_matching_topology_accepted() {
    let model = simple_bar();
    let mut case1 = LoadCase::new("case1");
    case1.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut case2 = LoadCase::new("case2");
    case2.nodal_load_2d(1, 20.0, 0.0).unwrap();
    let r1 = model.solve_case(&case1).unwrap();
    let r2 = model.solve_case(&case2).unwrap();
    let envelope = TrussEnvelope::from_results(&[&r1, &r2]).unwrap();
    assert_eq!(envelope.n_results, 2);
    assert_eq!(envelope.n_nodes, 2);
    assert_eq!(envelope.n_elements, 1);
}

// ===========================================================================
// Phase 130 — Batch solving with reusable factorization (2D)
// ===========================================================================

#[test]
fn p130_empty_cases_returns_empty() {
    let model = simple_bar();
    let results = model.solve_cases(&[]).unwrap();
    assert!(results.is_empty());
}

#[test]
fn p130_single_case_matches_solve_case() {
    let model = simple_bar();
    let mut case = LoadCase::new("fx");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let batch = model.solve_cases(&[&case]).unwrap();
    let single = model.solve_case(&case).unwrap();

    assert_eq!(batch.len(), 1);
    for i in 0..4 {
        assert!(
            (batch[0].displacements[i] - single.displacements[i]).abs() < 1e-12,
            "disp mismatch at {i}"
        );
        assert!(
            (batch[0].reactions[i] - single.reactions[i]).abs() < 1e-12,
            "reaction mismatch at {i}"
        );
    }
    assert!(
        (batch[0].axial_forces[0] - single.axial_forces[0]).abs() < 1e-12,
        "axial force mismatch"
    );
}

#[test]
fn p130_multiple_cases_match_solve_case_each() {
    let model = triangular_truss();
    let mut a = LoadCase::new("a");
    a.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut b = LoadCase::new("b");
    b.nodal_load_2d(2, 0.0, 2e3).unwrap();
    let mut c = LoadCase::new("c");
    c.nodal_load_2d(2, -500.0, 800.0).unwrap();

    let batch = model.solve_cases(&[&a, &b, &c]).unwrap();
    let sa = model.solve_case(&a).unwrap();
    let sb = model.solve_case(&b).unwrap();
    let sc = model.solve_case(&c).unwrap();

    assert_eq!(batch.len(), 3);
    let singles = [sa, sb, sc];
    for (k, r) in batch.iter().enumerate() {
        for i in 0..r.displacements.len() {
            assert!(
                (r.displacements[i] - singles[k].displacements[i]).abs() < 1e-9,
                "case {k} disp mismatch at dof {i}"
            );
        }
        for i in 0..r.reactions.len() {
            assert!(
                (r.reactions[i] - singles[k].reactions[i]).abs() < 1e-9,
                "case {k} reaction mismatch at dof {i}"
            );
        }
        for i in 0..r.axial_forces.len() {
            assert!(
                (r.axial_forces[i] - singles[k].axial_forces[i]).abs() < 1e-9,
                "case {k} axial force mismatch at element {i}"
            );
        }
    }
}

#[test]
fn p130_results_preserve_input_order() {
    let model = simple_bar();
    let mut first = LoadCase::new("first");
    first.nodal_load_2d(1, 10.0, 0.0).unwrap();
    let mut second = LoadCase::new("second");
    second.nodal_load_2d(1, 20.0, 0.0).unwrap();

    let results = model.solve_cases(&[&first, &second]).unwrap();
    assert_eq!(results.len(), 2);
    assert!(
        (results[0].displacements[2] - 10.0).abs() < 1e-9,
        "first case should give ux=10"
    );
    assert!(
        (results[1].displacements[2] - 20.0).abs() < 1e-9,
        "second case should give ux=20"
    );
}

#[test]
fn p130_load_source_is_load_case() {
    let model = simple_bar();
    let mut case = LoadCase::new("my_case");
    case.nodal_load_2d(1, 5.0, 0.0).unwrap();

    let results = model.solve_cases(&[&case]).unwrap();
    assert!(matches!(
        results[0].load_source(),
        LoadSource::LoadCase { .. }
    ));
}

#[test]
fn p130_prescribed_displacement_modifies_fixed_dof() {
    let model = simple_bar();
    let mut plain = LoadCase::new("plain");
    plain.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let mut shifted = LoadCase::new("shifted");
    shifted.nodal_load_2d(1, 10.0, 0.0).unwrap();
    shifted.prescribed_displacement_2d(0, 0, 0.001).unwrap();

    let results = model.solve_cases(&[&plain, &shifted]).unwrap();
    assert_eq!(results.len(), 2);

    let single_shifted = model.solve_case(&shifted).unwrap();
    for i in 0..4 {
        assert!(
            (results[1].displacements[i] - single_shifted.displacements[i]).abs() < 1e-12,
            "prescribed disp mismatch at dof {i}"
        );
    }
}

#[test]
fn p130_new_fixed_dof_rejected() {
    let model = simple_bar();
    let mut bad = LoadCase::new("bad");
    bad.nodal_load_2d(1, 10.0, 0.0).unwrap();
    bad.prescribed_displacement_2d(1, 0, 0.0).unwrap();

    let err = model.solve_cases(&[&bad]).unwrap_err();
    assert!(matches!(err, FemError::InvalidInput(_)));
}

#[test]
fn p130_invalid_node_rejected() {
    let model = simple_bar();
    let mut bad = LoadCase::new("bad");
    bad.nodal_load_2d(5, 10.0, 0.0).unwrap();

    let err = model.solve_cases(&[&bad]).unwrap_err();
    assert!(matches!(err, FemError::InvalidNode(_)));
}

#[test]
fn p130_all_constrained_n_free_zero() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 1.0, 0.0));
    model.add_element(TrussElement::new(0, 1, &unit_mat(), 1.0).unwrap());
    model.fix_node(0).unwrap();
    model.fix_node(1).unwrap();

    let mut case = LoadCase::new("constrained");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let results = model.solve_cases(&[&case]).unwrap();
    assert_eq!(results.len(), 1);
    for &u in &results[0].displacements {
        assert!(u.abs() < 1e-12, "all displacements should be zero");
    }
    assert_eq!(results[0].reactions.len(), 4);
}

#[test]
fn p130_batch_then_envelope() {
    use structural_analysis::postprocessing::TrussEnvelope;

    let model = triangular_truss();
    let mut a = LoadCase::new("a");
    a.nodal_load_2d(2, 1e3, 0.0).unwrap();
    let mut b = LoadCase::new("b");
    b.nodal_load_2d(2, 0.0, 2e3).unwrap();
    let mut c = LoadCase::new("c");
    c.nodal_load_2d(2, -500.0, 800.0).unwrap();

    let batch = model.solve_cases(&[&a, &b, &c]).unwrap();
    let refs: Vec<&_> = batch.iter().collect();
    let envelope = TrussEnvelope::from_results(&refs).unwrap();
    assert_eq!(envelope.n_results, 3);
}

#[test]
fn p130_solver_name_populated_when_free_dofs_exist() {
    let model = simple_bar();
    let mut case = LoadCase::new("fx");
    case.nodal_load_2d(1, 10.0, 0.0).unwrap();

    let results = model.solve_cases(&[&case]).unwrap();
    assert!(
        results[0].solver_name.is_some(),
        "solver_name should be populated when free DOFs exist"
    );
}
