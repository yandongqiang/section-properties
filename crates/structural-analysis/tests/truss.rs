//! Tests for the 2D truss element and solver.

use section_properties::Material;
use section_properties::SolverSelection;
use section_properties::geometry::Point;
use structural_analysis::truss::{TrussDof, TrussElement, TrussModel, TrussNode, TrussSolver};

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
