//! Phase 2.4 — Beam section internal force (N, V, M) recovery tests.
//!
//! These tests validate [`BeamSolver::element_section_forces`] against
//! closed-form Euler–Bernoulli solutions and against the element end-force
//! API. See the method documentation for the full sign convention; the key
//! points are:
//!
//! * local axes: +x from node_i to node_j, +y transverse, θ CCW;
//! * `axial` is tension positive;
//! * `shear` satisfies `d(moment)/dx = shear` and `d(shear)/dx = qy`
//!   (qy = upward load intensity);
//! * `moment` is sagging positive;
//! * evaluating exactly at an interior point load returns the left limit.

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, SectionForces,
};
use section_properties::fea::solver::SolverRegistry;
use section_properties::material::Material;

const E: f64 = 200e9;
const A: f64 = 0.02;

fn second_moment() -> f64 {
    0.1 * 0.2_f64.powi(3) / 12.0
}

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}

fn section() -> BeamSection {
    BeamSection::new(A, second_moment())
}

/// Solve a model with the dense solver and return the solver.
fn solve(model: &BeamModel) -> BeamSolver {
    let mut solver = BeamSolver::from_model(model).unwrap();
    let registry = SolverRegistry::default();
    let mut linear = registry.create("dense").unwrap();
    solver.solve(&mut *linear).unwrap();
    solver
}

/// Horizontal cantilever nodes `0 -> 1 -> ...` of total length `L` with
/// `n_elem` elements, fixed at node 0.
fn cantilever_model(n_elem: usize, length: f64) -> BeamModel {
    let mut model = BeamModel::new();
    let dx = length / n_elem as f64;
    for i in 0..=n_elem {
        model.add_node(BeamNode::new(i, i as f64 * dx, 0.0));
    }
    for i in 0..n_elem {
        model.add_element(BeamElement::new(i, i + 1, steel(), section()).unwrap());
    }
    model.fix_node(0);
    model
}

/// Assert `actual` is within `tol` (relative to `expected`, floored at 1) of
/// `expected`.
fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    let scale = expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{}: got {:.12e}, expected {:.12e} (tol {:e}, scale {:e})",
        label,
        actual,
        expected,
        tol,
        scale
    );
}

/// Evaluate element section forces at a physical position `x` on a uniform
/// mesh of `n_elem` elements of length `L`.
fn section_at_x(solver: &BeamSolver, n_elem: usize, length: f64, x: f64) -> SectionForces {
    let dx = length / n_elem as f64;
    let mut elem = (x / dx).floor() as usize;
    if elem >= n_elem {
        elem = n_elem - 1;
    }
    let xi = ((x - elem as f64 * dx) / dx).clamp(0.0, 1.0);
    solver.element_section_forces(elem, xi).unwrap()
}

// ===========================================================================
// A. Cantilever with tip transverse force P
// ===========================================================================

#[test]
fn test_section_forces_cantilever_tip_force() {
    let l = 1.0;
    let p = 1000.0; // downward tip load

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p); // v DOF, downward
    let solver = solve(&model);

    // Analytical (sagging positive): V(x) = +P, M(x) = -P(L - x), N = 0.
    let mut max_dv = 0.0f64;
    let mut max_dm = 0.0f64;
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = xi * l;
        let f = solver.element_section_forces(0, xi).unwrap();
        assert_close(f.axial, 0.0, 1e-12, "N");
        assert_close(f.shear, p, 1e-9, &format!("V at xi={}", xi));
        assert_close(f.moment, -p * (l - x), 1e-9, &format!("M at xi={}", xi));
        max_dv = max_dv.max((f.shear - p).abs());
        max_dm = max_dm.max((f.moment + p * (l - x)).abs());
    }
    println!(
        "[tip force] max |dV| = {:.3e} N, max |dM| = {:.3e} Nm",
        max_dv, max_dm
    );
}

// ===========================================================================
// B. Cantilever with uniform distributed load q (downward)
// ===========================================================================

#[test]
fn test_section_forces_cantilever_udl() {
    let l = 1.0;
    let q = 1000.0; // downward load per unit length

    let mut model = cantilever_model(1, l);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    let solver = solve(&model);

    // V(x) = q(L - x); M(x) = -q (L - x)^2 / 2; N = 0.
    let mut max_dv = 0.0f64;
    let mut max_dm = 0.0f64;
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = xi * l;
        let f = solver.element_section_forces(0, xi).unwrap();
        assert_close(f.axial, 0.0, 1e-12, "N");
        assert_close(f.shear, q * (l - x), 1e-8, &format!("V at xi={}", xi));
        assert_close(
            f.moment,
            -q * (l - x).powi(2) / 2.0,
            1e-8,
            &format!("M at xi={}", xi),
        );
        max_dv = max_dv.max((f.shear - q * (l - x)).abs());
        max_dm = max_dm.max((f.moment + q * (l - x).powi(2) / 2.0).abs());
    }
    println!(
        "[udl] max |dV| = {:.3e} N, max |dM| = {:.3e} Nm",
        max_dv, max_dm
    );

    // Verify the equilibrium identities dV/dx = -q and dM/dx = V numerically.
    let v1 = solver.element_section_forces(0, 0.4).unwrap();
    let v2 = solver.element_section_forces(0, 0.6).unwrap();
    assert_close(
        (v2.shear - v1.shear) / 0.2,
        -q,
        1e-8,
        "dV/dx (should equal -q)",
    );

    // M is quadratic, so a central difference recovers V exactly.
    let m1 = solver.element_section_forces(0, 0.45).unwrap();
    let m2 = solver.element_section_forces(0, 0.55).unwrap();
    let v_mid = solver.element_section_forces(0, 0.5).unwrap();
    assert_close(
        (m2.moment - m1.moment) / 0.1,
        v_mid.shear,
        1e-8,
        "dM/dx = V",
    );
}

// ===========================================================================
// C. Cantilever with axial tip force P
// ===========================================================================

#[test]
fn test_section_forces_cantilever_axial_force() {
    let l = 1.0;
    let p = 1000.0; // tension at the tip (+x)

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 0, p); // u DOF
    let solver = solve(&model);

    let mut max_dn = 0.0f64;
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let f = solver.element_section_forces(0, xi).unwrap();
        assert_close(f.axial, p, 1e-9, &format!("N at xi={}", xi));
        assert_close(f.shear, 0.0, 1e-12, "V");
        assert_close(f.moment, 0.0, 1e-12, "M");
        max_dn = max_dn.max((f.axial - p).abs());
    }
    println!("[axial force] max |dN| = {:.3e} N", max_dn);
}

// ===========================================================================
// D. Interior point force (shear jump, moment continuity)
// ===========================================================================

#[test]
fn test_section_forces_interior_point_force() {
    let l = 1.0;
    let p = 1000.0; // downward force at midspan
    let xi_p = 0.5;

    let mut model = cantilever_model(1, l);
    model.add_point_load(0, xi_p, 0.0, -p, 0.0).unwrap();
    let solver = solve(&model);

    let left = solver.element_section_forces(0, xi_p - 1e-6).unwrap();
    let right = solver.element_section_forces(0, xi_p + 1e-6).unwrap();

    // Shear: +P for x < L/2, 0 for x > L/2.
    assert_close(left.shear, p, 1e-6, "V left of load");
    assert_close(right.shear, 0.0, 1e-6, "V right of load");
    // Jump equals the applied transverse load fy_p: right - left = fy_p = -P.
    assert_close(right.shear - left.shear, -p, 1e-6, "shear jump = fy_p");

    // Moment is continuous at the point force: both one-sided limits are 0.
    // (Evaluating exactly at xi_p returns the left limit, whose moment limit
    // is 0; the finite-epsilon value differs only by the smooth O(eps) slope.)
    let left_limit = solver.element_section_forces(0, xi_p).unwrap();
    assert_close(left_limit.moment, 0.0, 1e-6, "M left limit at load");
    assert_close(right.moment, 0.0, 1e-6, "M right limit at load");

    // Moment at the fixed end is -P·L/2.
    let f0 = solver.element_section_forces(0, 0.0).unwrap();
    assert_close(f0.moment, -p * l / 2.0, 1e-9, "M at fixed end");

    // Evaluating exactly at the load returns the LEFT limit for the shear.
    assert_close(
        left_limit.shear,
        left.shear,
        1e-9,
        "V at xi_p == left limit",
    );
    assert_close(left_limit.shear, p, 1e-9, "V left limit value");
}

#[test]
fn test_section_forces_interior_axial_point_force() {
    let l = 1.0;
    let p = 1000.0; // axial tension applied inside the element

    let mut model = cantilever_model(1, l);
    model.add_point_load(0, 0.5, p, 0.0, 0.0).unwrap();
    let solver = solve(&model);

    let left = solver.element_section_forces(0, 0.5 - 1e-6).unwrap();
    let right = solver.element_section_forces(0, 0.5 + 1e-6).unwrap();

    // Axial jump equals -fx_p.
    assert_close(right.axial - left.axial, -p, 1e-6, "axial jump = -fx_p");
    // To the right of the load there is no axial force (free end).
    assert_close(right.axial, 0.0, 1e-6, "N right of load");
}

// ===========================================================================
// E. Interior applied moment (moment jump)
// ===========================================================================

#[test]
fn test_section_forces_interior_point_moment() {
    let l = 1.0;
    let m = 1000.0; // CCW applied moment at midspan
    let xi_p = 0.5;

    let mut model = cantilever_model(1, l);
    model.add_point_moment(0, xi_p, m).unwrap();
    let solver = solve(&model);

    let left = solver.element_section_forces(0, xi_p - 1e-6).unwrap();
    let right = solver.element_section_forces(0, xi_p + 1e-6).unwrap();

    // No shear for a pure interior moment.
    assert_close(left.shear, 0.0, 1e-9, "V left");
    assert_close(right.shear, 0.0, 1e-9, "V right");

    // Moment jump equals -mz_p.
    assert_close(right.moment - left.moment, -m, 1e-6, "moment jump = -mz_p");
    // Right of the moment there is no bending moment (free end).
    assert_close(right.moment, 0.0, 1e-6, "M right of moment");
    // At the fixed end the section moment equals the element end moment +M.
    let f0 = solver.element_section_forces(0, 0.0).unwrap();
    assert_close(f0.moment, m, 1e-6, "M at fixed end");
}

// ===========================================================================
// F. Boundary equivalence with element end forces
// ===========================================================================

#[test]
fn test_section_forces_boundary_matches_end_forces() {
    let l = 1.0;
    let p = 1000.0;

    // Cantilever with a transverse tip force and an axial tip force so that
    // all three section components are exercised.
    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p);
    model.add_nodal_force(1, 0, 500.0);
    let solver = solve(&model);

    let end = solver.element_end_forces().unwrap()[0];
    let (n_i, v_i, m_i, n_j, v_j, m_j) = (end[0], end[1], end[2], end[3], end[4], end[5]);

    let at0 = solver.element_section_forces(0, 0.0).unwrap();
    assert_close(at0.axial, n_i, 1e-9, "N(0) = +N_i");
    assert_close(at0.shear, -v_i, 1e-9, "V(0) = -V_i");
    assert_close(at0.moment, m_i, 1e-9, "M(0) = +M_i");

    let at1 = solver.element_section_forces(0, 1.0).unwrap();
    assert_close(at1.axial, -n_j, 1e-9, "N(1) = -N_j");
    assert_close(at1.shear, v_j, 1e-9, "V(1) = +V_j");
    assert_close(at1.moment, -m_j, 1e-9, "M(1) = -M_j");
}

/// A point force at xi = 1 on the element must give the same section-force
/// field as the equivalent nodal force at node j (matching the Phase 2.3
/// boundary-equivalence property).
#[test]
fn test_section_forces_boundary_point_load_equivalence() {
    let l = 1.0;
    let p = 1000.0;

    let mut model_point = cantilever_model(1, l);
    model_point.add_point_load(0, 1.0, 0.0, -p, 0.0).unwrap();
    let solver_point = solve(&model_point);

    let mut model_nodal = cantilever_model(1, l);
    model_nodal.add_nodal_force(1, 1, -p);
    let solver_nodal = solve(&model_nodal);

    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let a = solver_point.element_section_forces(0, xi).unwrap();
        let b = solver_nodal.element_section_forces(0, xi).unwrap();
        assert_close(a.axial, b.axial, 1e-9, "N point vs nodal");
        assert_close(a.shear, b.shear, 1e-9, "V point vs nodal");
        assert_close(a.moment, b.moment, 1e-9, "M point vs nodal");
    }
}

// ===========================================================================
// G. Multi-element shared-node continuity / jump
// ===========================================================================

#[test]
fn test_section_forces_shared_node_continuity() {
    let l = 1.0;
    let p = 1000.0;

    // Two-element cantilever with a tip nodal force only (no load at node 1).
    let mut model = cantilever_model(2, l);
    model.add_nodal_force(2, 1, -p);
    let solver = solve(&model);

    let left = solver.element_section_forces(0, 1.0).unwrap();
    let right = solver.element_section_forces(1, 0.0).unwrap();

    assert_close(left.axial, right.axial, 1e-9, "N continuity");
    assert_close(left.shear, right.shear, 1e-9, "V continuity");
    assert_close(left.moment, right.moment, 1e-9, "M continuity");
}

#[test]
fn test_section_forces_shared_node_with_external_force() {
    let l = 1.0;
    let p = 1000.0;
    let f_ext = -p; // downward external nodal force at the shared node

    let mut model = cantilever_model(2, l);
    model.add_nodal_force(1, 1, f_ext); // at shared node
    model.add_nodal_force(2, 1, -p); // tip force keeps it well-posed/loaded
    let solver = solve(&model);

    let left = solver.element_section_forces(0, 1.0).unwrap();
    let right = solver.element_section_forces(1, 0.0).unwrap();

    // For no external moment, the moment stays continuous.
    assert_close(left.moment, right.moment, 1e-9, "M continuity");

    // Shear jump at the shared node equals the external nodal force:
    // V_right - V_left = F_ext.
    assert_close(right.shear - left.shear, f_ext, 1e-9, "V jump = F_ext");
}

// ===========================================================================
// H. Rotated beam (45 degrees) — local section forces
// ===========================================================================

#[test]
fn test_section_forces_rotated_beam() {
    use std::f64::consts::FRAC_1_SQRT_2;

    let l = 1.0;
    let p = 1000.0; // global downward tip force
    let c = FRAC_1_SQRT_2;
    let s = FRAC_1_SQRT_2;

    // 45° cantilever: element length L along direction (c, s).
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l * c, l * s));
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.fix_node(0);
    // Global downward tip force applied on global v DOF.
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);

    // Local decomposition of the global tip force F = (0, -P):
    //   fx = F·e_x = -P/√2,  fy = F·e_y = -P/√2.
    let fx = -p * FRAC_1_SQRT_2;
    let fy = -p * FRAC_1_SQRT_2;

    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = xi * l;
        let f = solver.element_section_forces(0, xi).unwrap();
        assert_close(f.axial, fx, 1e-9, &format!("N at xi={}", xi));
        assert_close(f.shear, -fy, 1e-9, &format!("V at xi={}", xi));
        assert_close(f.moment, fy * (l - x), 1e-9, &format!("M at xi={}", xi));
    }
    println!("[rotated 45°] local N/V/M match global tip force decomposition");

    // Transformation consistency: the global tip displacement must equal the
    // transformation of the analytical local tip displacement
    // (u_l = fx·L/(EA), v_l = fy·L³/(3EI)).
    let u_l = fx * l / (E * A);
    let v_l = fy * l.powi(3) / (3.0 * E * second_moment());
    let u_g = c * u_l - s * v_l;
    let v_g = s * u_l + c * v_l;
    assert_close(solver.displacement(1, 0), u_g, 1e-9, "global U tip");
    assert_close(solver.displacement(1, 1), v_g, 1e-9, "global V tip");
}

// ===========================================================================
// I. Mesh convergence (exact for statically determinate loading)
// ===========================================================================

#[test]
fn test_section_forces_mesh_convergence_udl() {
    let l = 1.0;
    let q = 1000.0;

    // Analytical: V(x) = q(L-x), M(x) = -q(L-x)^2 / 2.
    let characteristic = q * l * l; // scale for the moment tolerance
    let mut worst_v = 0.0f64;
    let mut worst_m = 0.0f64;

    for &n_elem in &[1usize, 2, 4, 8] {
        let mut model = cantilever_model(n_elem, l);
        model.add_distributed_load(0, 0.0, -q).unwrap();
        // Uniform load on every element.
        for e in 1..n_elem {
            model.add_distributed_load(e, 0.0, -q).unwrap();
        }
        let solver = solve(&model);

        for &x in &[0.25, 0.5, 0.75, 1.0] {
            let f = section_at_x(&solver, n_elem, l, x);
            let v_exact = q * (l - x);
            let m_exact = -q * (l - x).powi(2) / 2.0;

            assert!(
                (f.shear - v_exact).abs() / (q * l) < 1e-8,
                "n_elem={}, x={}: V={} expected {}",
                n_elem,
                x,
                f.shear,
                v_exact
            );
            assert!(
                (f.moment - m_exact).abs() / characteristic < 1e-8,
                "n_elem={}, x={}: M={} expected {}",
                n_elem,
                x,
                f.moment,
                m_exact
            );
            worst_v = worst_v.max((f.shear - v_exact).abs() / (q * l));
            worst_m = worst_m.max((f.moment - m_exact).abs() / characteristic);
        }
    }
    println!(
        "[udl convergence n=1,2,4,8] worst rel |dV| = {:.3e}, worst rel |dM| = {:.3e}",
        worst_v, worst_m
    );
}

#[test]
fn test_section_forces_mesh_convergence_tip_force() {
    let l = 1.0;
    let p = 1000.0;

    let mut worst_v = 0.0f64;
    let mut worst_m = 0.0f64;
    for &n_elem in &[1usize, 2, 4, 8] {
        let mut model = cantilever_model(n_elem, l);
        model.add_nodal_force(n_elem, 1, -p);
        let solver = solve(&model);

        for &x in &[0.25, 0.5, 0.75, 1.0] {
            let f = section_at_x(&solver, n_elem, l, x);
            let m_exact = -p * (l - x);
            assert!(
                (f.shear - p).abs() / p < 1e-8,
                "n_elem={}, x={}: V={} expected {}",
                n_elem,
                x,
                f.shear,
                p
            );
            assert!(
                (f.moment - m_exact).abs() / (p * l) < 1e-8,
                "n_elem={}, x={}: M={} expected {}",
                n_elem,
                x,
                f.moment,
                m_exact
            );
            worst_v = worst_v.max((f.shear - p).abs() / p);
            worst_m = worst_m.max((f.moment - m_exact).abs() / (p * l));
        }
    }
    println!(
        "[tip-force convergence n=1,2,4,8] worst rel |dV| = {:.3e}, worst rel |dM| = {:.3e}",
        worst_v, worst_m
    );
}

// ===========================================================================
// J. Error handling
// ===========================================================================

#[test]
fn test_section_forces_invalid_inputs() {
    let mut model = cantilever_model(1, 1.0);
    model.add_nodal_force(1, 1, -1000.0);
    let solver = solve(&model);

    // Valid queries succeed.
    assert!(solver.element_section_forces(0, 0.0).is_ok());
    assert!(solver.element_section_forces(0, 0.5).is_ok());
    assert!(solver.element_section_forces(0, 1.0).is_ok());

    // xi outside [0, 1].
    assert!(solver.element_section_forces(0, -0.1).is_err());
    assert!(solver.element_section_forces(0, 1.1).is_err());
    assert!(solver.element_section_forces(0, f64::NAN).is_err());

    // Invalid element index.
    assert!(solver.element_section_forces(1, 0.5).is_err());
    assert!(solver.element_section_forces(999, 0.5).is_err());
}
