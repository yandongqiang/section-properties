//! Phase 7 — Beam FEM numerical final audit.
//!
//! Independent verification of the complete calculation chain
//! (`load -> element load vector -> assembly -> BC -> solve -> u -> end forces
//! -> section forces -> reactions/equilibrium`) against closed-form
//! Euler–Bernoulli solutions, with solver cross-checks and scaling sanity.
//!
//! Established conventions (never "fixed" here): `[ux, uy, rz]`; local x
//! node_i->node_j; sagging-positive moment; `M(x) = -P(L-x)` for a downward tip
//! load; `M(0) = -qL^2/2`; `M_j = -M` for a positive tip applied moment;
//! `N_j = -N(xi=1)`; `f_end = f_equiv - K_e u_e`.
//!
//! Realistic magnitudes: E = 200 GPa, A = 5e-3 m^2, I = 2e-5 m^4, L = 2 m,
//! P = 10 kN, q = 5 kN/m, M = 8 kN·m  =>  EI = 4e6 N·m^2, EA = 1e9 N.
//! Tolerances: displacements ~1e-3 m (abs 1e-12, rel 1e-9); forces/moments
//! ~1e4 N / N·m (abs 1e-6, rel 1e-9), justified by conditioning (not inflated).

use section_properties::SolverSelection;
use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];
const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const EI: f64 = E * I; // 4e6
const EA: f64 = E * A; // 1e9
const L: f64 = 2.0;
const P: f64 = 10e3;
const Q: f64 = 5e3;
const M0: f64 = 8e3;

fn mat() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}
fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

/// Horizontal cantilever with `n` equal elements, fixed at node 0.
fn cantilever(n: usize) -> BeamModel {
    let mut m = BeamModel::new();
    let dx = L / n as f64;
    for k in 0..=n {
        m.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n {
        m.add_element(BeamElement::new(k, k + 1, mat(), sec()).unwrap());
    }
    m.fix_node(0);
    m
}

fn solve_named(model: &BeamModel, name: &str) -> BeamSolver {
    let mut s = BeamSolver::from_model(model).unwrap();
    s.set_solver(SolverSelection::named(name));
    s.solve_configured().unwrap();
    s
}

fn assert_num(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: {} vs {} (|diff| = {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

/// Equilibrium residual helper: reaction + sum of applied loads = 0.
fn assert_equilibrium(s: &BeamSolver, fx: f64, fy: f64, mz: f64, label: &str) {
    let r = s.reactions();
    assert_num(r[0] + fx, 0.0, 1e-6, 1e-9, &format!("{} ΣFx", label));
    assert_num(r[1] + fy, 0.0, 1e-6, 1e-9, &format!("{} ΣFy", label));
    assert_num(r[2] + mz, 0.0, 1e-6, 1e-9, &format!("{} ΣMz", label));
}

// ===========================================================================
// 3. Axial verification
// ===========================================================================

#[test]
fn test_axial_tip_force_analytical() {
    let mut m = cantilever(2);
    m.add_nodal_force(2, 0, P);

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // u(L) = P L / EA
        assert_num(s.displacement(2, 0), P * L / EA, 1e-14, 1e-9, "u(L)");
        // Support reaction + P = 0.
        assert_equilibrium(&s, P, 0.0, 0.0, "axial tip");
        // Axial force is constant +P in both elements.
        for e in 0..2 {
            let n = s.element_section_forces(e, 0.5).unwrap().axial;
            assert_num(n, P, 1e-6, 1e-9, "N(x)");
            let fe = s.element_end_forces().unwrap()[e];
            assert_num(fe[0], P, 1e-6, 1e-9, "N_i");
            assert_num(fe[3], -P, 1e-6, 1e-9, "N_j");
        }
    }
}

#[test]
fn test_axial_udl_analytical() {
    let mut m = cantilever(2);
    let qx = 4e3; // N/m tensile
    m.add_distributed_load(0, qx, 0.0).unwrap();
    m.add_distributed_load(1, qx, 0.0).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // u(L) = qx L^2 / (2 EA)
        assert_num(
            s.displacement(2, 0),
            qx * L * L / (2.0 * EA),
            1e-14,
            1e-9,
            "u(L)",
        );
        // Reaction = -qx L.
        assert_equilibrium(&s, qx * L, 0.0, 0.0, "axial udl");
        // N(x) = qx (L - x): check both elements at several stations.
        for (e, x0) in [(0usize, 0.0), (1usize, L / 2.0)] {
            for &xi in &[0.0, 0.5, 1.0] {
                let x = x0 + xi * (L / 2.0);
                assert_num(
                    s.element_section_forces(e, xi).unwrap().axial,
                    qx * (L - x),
                    1e-6,
                    1e-9,
                    "N(x)=qx(L-x)",
                );
            }
        }
        // Established boundary relation: N_j = -N(xi=1).
        let n_at_end = s.element_section_forces(0, 1.0).unwrap().axial;
        assert_num(
            s.element_end_forces().unwrap()[0][3],
            -n_at_end,
            1e-6,
            1e-9,
            "N_j = -N(xi=1)",
        );
    }
}

#[test]
fn test_axial_scaling() {
    for &alpha in &[0.5_f64, 1.0, 4.0] {
        // P -> alpha P
        let mut mp = cantilever(1);
        mp.add_nodal_force(1, 0, alpha * P);
        let sp = solve_named(&mp, "dense");
        assert_num(
            sp.displacement(1, 0),
            alpha * P * L / EA,
            1e-14,
            1e-9,
            "u ∝ P",
        );
        // L -> alpha L (single element of length alpha*L)
        let mut ml = BeamModel::new();
        ml.add_node(BeamNode::new(0, 0.0, 0.0));
        ml.add_node(BeamNode::new(1, alpha * L, 0.0));
        ml.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        ml.fix_node(0);
        ml.add_nodal_force(1, 0, P);
        assert_num(
            solve_named(&ml, "dense").displacement(1, 0),
            P * alpha * L / EA,
            1e-14,
            1e-9,
            "u ∝ L",
        );
        // E -> alpha E  =>  u / alpha ; A -> alpha A  =>  u / alpha
        let mut me = cantilever(1);
        me.add_nodal_force(1, 0, P);
        me.elements[0].material = Material::new(alpha * E, 0.3, 7850.0, "S");
        assert_num(
            solve_named(&me, "dense").displacement(1, 0),
            P * L / (alpha * EA),
            1e-14,
            1e-9,
            "u ∝ 1/E",
        );
        let mut ma = cantilever(1);
        ma.add_nodal_force(1, 0, P);
        ma.elements[0].section = BeamSection::new(alpha * A, I);
        assert_num(
            solve_named(&ma, "dense").displacement(1, 0),
            P * L / (alpha * EA),
            1e-14,
            1e-9,
            "u ∝ 1/A",
        );
    }
}

// ===========================================================================
// 4. Euler-Bernoulli bending
// ===========================================================================

#[test]
fn test_bending_tip_force_analytical() {
    let mut m = cantilever(2);
    m.add_nodal_force(2, 1, -P);

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        assert_num(
            s.displacement(2, 1),
            -P * L.powi(3) / (3.0 * EI),
            1e-14,
            1e-9,
            "v(L)",
        );
        assert_num(
            s.displacement(2, 2),
            -P * L.powi(2) / (2.0 * EI),
            1e-14,
            1e-9,
            "theta(L)",
        );
        // Reactions: Ry = +P, Rz = +P L (moment of the load about node 0).
        assert_equilibrium(&s, 0.0, -P, -P * L, "bending tip");
        // V(x) = +P, M(x) = -P (L - x).
        for (e, x0) in [(0usize, 0.0), (1usize, L / 2.0)] {
            for &xi in &[0.0, 0.5, 1.0] {
                let x = x0 + xi * (L / 2.0);
                let sf = s.element_section_forces(e, xi).unwrap();
                assert_num(sf.shear, P, 1e-6, 1e-9, "V(x)");
                assert_num(sf.moment, -P * (L - x), 1e-6, 1e-9, "M(x)");
            }
        }
        // At the free end the shear just inside the element is +P (it drops to
        // 0 only at the load application point, which lies outside the
        // element); the moment there is zero.
        let free = s.element_section_forces(1, 1.0).unwrap();
        assert_num(free.shear, P, 1e-6, 1e-9, "V(free-inside)=P");
        assert_num(free.moment.abs(), 0.0, 1e-9, 1e-9, "M(free)=0");
    }
}

#[test]
fn test_bending_udl_analytical() {
    let mut m = cantilever(4);
    for e in 0..4 {
        m.add_distributed_load(e, 0.0, -Q).unwrap();
    }

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        assert_num(
            s.displacement(4, 1),
            -Q * L.powi(4) / (8.0 * EI),
            1e-14,
            1e-9,
            "v(L)",
        );
        assert_num(
            s.displacement(4, 2),
            -Q * L.powi(3) / (6.0 * EI),
            1e-14,
            1e-9,
            "theta(L)",
        );
        // M(0) = -q L^2 / 2 ; V(0) = q L.
        let s0 = s.element_section_forces(0, 0.0).unwrap();
        assert_num(s0.moment, -Q * L * L / 2.0, 1e-6, 1e-9, "M(0)");
        assert_num(s0.shear, Q * L, 1e-6, 1e-9, "V(0)");
        // Diagrams: V(x) = q(L-x), M(x) = -q(L-x)^2/2.
        let dx = L / 4.0;
        for e in 0..4 {
            for &xi in &[0.0, 0.5, 1.0] {
                let x = e as f64 * dx + xi * dx;
                let sf = s.element_section_forces(e, xi).unwrap();
                assert_num(sf.shear, Q * (L - x), 1e-6, 1e-9, "V(x)");
                assert_num(sf.moment, -Q * (L - x).powi(2) / 2.0, 1e-6, 1e-9, "M(x)");
            }
        }
        // Reactions: Ry = qL, Rz = qL^2/2.
        assert_equilibrium(&s, 0.0, -Q * L, -Q * L * (L / 2.0), "udl");
    }
}

#[test]
fn test_bending_tip_moment_analytical() {
    let mut m = cantilever(2);
    m.add_applied_moment(2, M0).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // theta(L) = M L / EI ; v(L) = M L^2 / (2 EI)
        assert_num(s.displacement(2, 2), M0 * L / EI, 1e-14, 1e-9, "theta(L)");
        assert_num(
            s.displacement(2, 1),
            M0 * L * L / (2.0 * EI),
            1e-14,
            1e-9,
            "v(L)",
        );
        // Reaction moment + applied moment = 0.
        assert_equilibrium(&s, 0.0, 0.0, M0, "tip moment");
        // Established convention: free-end element-on-node moment M_j = -M,
        // while the internal section moment is +M everywhere.
        let fe = s.element_end_forces().unwrap()[1];
        assert_num(fe[5], -M0, 1e-6, 1e-9, "M_j = -M");
        assert_num(fe[2], M0, 1e-6, 1e-9, "M_i = +M");
        for e in 0..2 {
            for &xi in &[0.0, 0.5, 1.0] {
                assert_num(
                    s.element_section_forces(e, xi).unwrap().moment,
                    M0,
                    1e-6,
                    1e-9,
                    "M(x) = +M",
                );
            }
        }
    }
}

// ===========================================================================
// 5. Interior point load (piecewise analytical solution)
// ===========================================================================

#[test]
fn test_interior_point_load_piecewise_analytical() {
    // 2-element cantilever (each L/2 = 1 m); load P downward at xi = 0.5 of
    // element 1, i.e. a = L/2 + 0.5*(L/2) = 1.5 m from the fixed end.
    let a = 1.5;
    let mut m = cantilever(2);
    m.add_point_load(1, 0.5, 0.0, -P, 0.0).unwrap();

    // Closed form (magnitude; deflection is negative for a downward load):
    //   x <= a : v = -P x^2 (3a - x) / (6 EI),  theta = -P x (2a - x) / (2 EI)
    //   x >= a : v = -P a^2 (3x - a) / (6 EI),  theta = -P a^2 / (2 EI)
    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // node 1 at x = 1.0 (< a)
        let v1 = -P * 1.0f64.powi(2) * (3.0 * a - 1.0) / (6.0 * EI);
        let t1 = -P * 1.0 * (2.0 * a - 1.0) / (2.0 * EI);
        assert_num(s.displacement(1, 1), v1, 1e-14, 1e-9, "v(x<a)");
        assert_num(s.displacement(1, 2), t1, 1e-14, 1e-9, "theta(x<a)");
        // tip at x = L = 2.0 (> a)
        let v2 = -P * a * a * (3.0 * L - a) / (6.0 * EI);
        let t2 = -P * a * a / (2.0 * EI);
        assert_num(s.displacement(2, 1), v2, 1e-14, 1e-9, "v(x>a)");
        assert_num(s.displacement(2, 2), t2, 1e-14, 1e-9, "theta(x>a)");
        // Equilibrium: reaction + applied load (moment arm a).
        assert_equilibrium(&s, 0.0, -P, -P * a, "interior point load");
    }

    // Shear jump = fy across the load; moment is continuous there.
    let s = solve_named(&m, "dense");
    let v_left = s.element_section_forces(1, 0.499).unwrap().shear;
    let v_right = s.element_section_forces(1, 0.501).unwrap().shear;
    assert_num(v_right - v_left, -P, 1e-4, 1e-6, "ΔV = fy");
    let eps = 1e-9;
    let m_left = s.element_section_forces(1, 0.5 - eps).unwrap().moment;
    let m_right = s.element_section_forces(1, 0.5 + eps).unwrap().moment;
    assert_num(
        m_right - m_left,
        0.0,
        1e-3,
        1e-9,
        "M continuous at point force",
    );
}

#[test]
fn test_point_load_at_boundary_matches_nodal() {
    // xi = 0 and xi = 1 at a shared node must equal one nodal load (no double
    // counting): each is applied exactly once.
    let mut nodal = cantilever(2);
    nodal.add_nodal_force(1, 1, -P);
    let mut at_end = cantilever(2);
    at_end.add_point_load(0, 1.0, 0.0, -P, 0.0).unwrap();
    let mut at_start = cantilever(2);
    at_start.add_point_load(1, 0.0, 0.0, -P, 0.0).unwrap();

    let (sn, se, ss) = (
        solve_named(&nodal, "dense"),
        solve_named(&at_end, "dense"),
        solve_named(&at_start, "dense"),
    );
    for i in 0..sn.displacements().len() {
        assert_num(
            se.displacements()[i],
            sn.displacements()[i],
            1e-14,
            1e-9,
            "xi=1 vs nodal",
        );
        assert_num(
            ss.displacements()[i],
            sn.displacements()[i],
            1e-14,
            1e-9,
            "xi=0 vs nodal",
        );
    }
    // Total reaction is exactly one P (not 2P).
    for s in [&sn, &se, &ss] {
        assert_num(s.reactions()[1], P, 1e-6, 1e-9, "Ry = P (counted once)");
    }
}

// ===========================================================================
// 6. Distributed loads: transverse, axial, combined
// ===========================================================================

#[test]
fn test_distributed_loads_equilibrium_and_continuity() {
    let (qx, qy) = (4e3, -5e3);
    let mut m = cantilever(3);
    for e in 0..3 {
        m.add_distributed_load(e, qx, qy).unwrap();
    }
    // A transverse point load at the middle node to test a shear jump.
    m.add_nodal_force(2, 1, -2e3);

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // Global equilibrium: resultants qx*L, qy*L plus the nodal force.
        let fy = qy * L - 2e3;
        let mz = (qy * L) * (L / 2.0) + (-2e3) * (2.0 * L / 3.0);
        assert_equilibrium(&s, qx * L, fy, mz, "distributed + nodal");

        // Moment continuity at internal element boundaries (no concentrated
        // moment between elements 0/1).
        let m_end = s.element_section_forces(0, 1.0).unwrap().moment;
        let m_next = s.element_section_forces(1, 0.0).unwrap().moment;
        assert_num(m_end, m_next, 1e-6, 1e-9, "M continuous at boundary 0/1");
        // Shear continuous where no concentrated transverse force exists.
        let v_end = s.element_section_forces(0, 1.0).unwrap().shear;
        let v_next = s.element_section_forces(1, 0.0).unwrap().shear;
        assert_num(v_end, v_next, 1e-6, 1e-9, "V continuous at boundary 0/1");
        // Shear jumps by exactly the nodal force at node 2 (boundary 1/2).
        let v_l = s.element_section_forces(1, 1.0).unwrap().shear;
        let v_r = s.element_section_forces(2, 0.0).unwrap().shear;
        assert_num(v_r - v_l, -2e3, 1e-6, 1e-9, "ΔV at nodal force");
        // Axial force varies as qx (L - x) and is continuous between elements.
        let n_end = s.element_section_forces(0, 1.0).unwrap().axial;
        let n_next = s.element_section_forces(1, 0.0).unwrap().axial;
        assert_num(n_end, n_next, 1e-6, 1e-9, "N continuous");
    }
}

// ===========================================================================
// 7. Applied moments
// ===========================================================================

#[test]
fn test_applied_moments_tip_interior_and_multiple() {
    // (a) tip moment
    let mut tip = cantilever(2);
    tip.add_applied_moment(2, M0).unwrap();
    let s = solve_named(&tip, "dense");
    assert_equilibrium(&s, 0.0, 0.0, M0, "tip moment");
    assert_num(
        s.element_end_forces().unwrap()[1][5],
        -M0,
        1e-6,
        1e-9,
        "M_j = -M",
    );

    // (b) interior point moment via the element API (mz at xi = 0.5 of elem 1)
    let mut ipm = cantilever(2);
    ipm.add_point_moment(1, 0.5, 3e3).unwrap();
    let s2 = solve_named(&ipm, "dense");
    // Reaction moment balances the total applied moment (point moment is local,
    // same orientation for a horizontal beam).
    assert_equilibrium(&s2, 0.0, 0.0, 3e3, "interior point moment");

    // (c) two applied moments: reaction moment + sum(applied) = 0
    let mut two = cantilever(2);
    two.add_applied_moment(1, 2e3).unwrap();
    two.add_applied_moment(2, -5e3).unwrap();
    let s3 = solve_named(&two, "dense");
    assert_equilibrium(&s3, 0.0, 0.0, 2e3 - 5e3, "two applied moments");
    // The section moment jumps by -mz across an applied nodal moment.
    let left = s3.element_section_forces(0, 1.0).unwrap().moment;
    let right = s3.element_section_forces(1, 0.0).unwrap().moment;
    assert_num(
        right - left + 2e3,
        0.0,
        1e-6,
        1e-9,
        "M_right - M_left + M_app = 0",
    );
    // An applied moment is not an element end force by itself: the end-force
    // pair at the moment node still satisfies f_left + f_right + M_app = 0.
    let fe = s3.element_end_forces().unwrap();
    assert_num(
        fe[0][5] + fe[1][2] + 2e3,
        0.0,
        1e-6,
        1e-9,
        "end forces + M_app = 0",
    );
}

// ===========================================================================
// 8. Rotated elements: transformation identities
// ===========================================================================

#[test]
fn test_rotated_transformation_identities() {
    for &deg in &[0.0_f64, 45.0, 90.0, -45.0, 135.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, L * c, L * s));
        m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        m.fix_node(0);

        let (ni, nj) = (m.nodes[0].point(), m.nodes[1].point());
        let el = &m.elements[0];
        let t = el.transformation_matrix(ni, nj);

        // (1) k_global must equal T^T k_local T.
        let k_local = el.local_stiffness(ni, nj);
        let k_global = el.global_stiffness(ni, nj);
        // tt = T^T ; kt = T^T k_local ; prod = kt T
        let mut prod = [[0.0f64; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += t[k][i] * k_local[k][j]; // (T^T k_local)[i][j]
                }
                prod[i][j] = sum;
            }
        }
        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    sum += prod[i][k] * t[k][j];
                }
                let scale = k_global[i][j].abs().max(1.0);
                assert!(
                    (sum - k_global[i][j]).abs() <= 1e-6 * scale,
                    "{:.0}° k_global != T^T k_local T at [{},{}]: {} vs {}",
                    deg,
                    i,
                    j,
                    sum,
                    k_global[i][j]
                );
            }
        }

        // (2) Element equivalent load: f_global must equal T^T f_local.
        let f_local = el.consistent_nodal_load(ni, nj, 0.0, -Q).unwrap();
        let mut f_glob = [0.0f64; 6];
        for i in 0..6 {
            for j in 0..6 {
                f_glob[i] += t[j][i] * f_local[j];
            }
        }
        // The resultant of a LOCAL qy load has magnitude |qy|*L along the
        // rotated local y axis = (-s, c) in global coordinates, so its global
        // components are (qy*L)*(-s, qy*L)*c.
        let total_x: f64 = (0..6).filter(|i| i % 3 == 0).map(|i| f_glob[i]).sum();
        let total_y: f64 = (0..6).filter(|i| i % 3 == 1).map(|i| f_glob[i]).sum();
        assert_num(total_x, (-Q * L) * (-s), 1e-6, 1e-9, "equivalent load Fx");
        assert_num(total_y, (-Q * L) * c, 1e-6, 1e-9, "equivalent load Fy");

        // (3) Physics: local tip load (0, -P) applied as a global force.
        let mut mp = BeamModel::new();
        mp.add_node(BeamNode::new(0, 0.0, 0.0));
        mp.add_node(BeamNode::new(1, L * c, L * s));
        mp.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        mp.fix_node(0);
        let gx = -s * (-P);
        let gy = c * (-P);
        mp.add_nodal_force(1, 0, gx);
        mp.add_nodal_force(1, 1, gy);

        for &name in &DIRECT {
            let sol = solve_named(&mp, name);
            // Global displacement = R(theta) * (0, -P L^3 / 3EI)
            let v_loc = -P * L.powi(3) / (3.0 * EI);
            assert_num(
                sol.displacement(1, 0),
                -s * v_loc,
                1e-14,
                1e-9,
                &format!("{:.0}° ux", deg),
            );
            assert_num(
                sol.displacement(1, 1),
                c * v_loc,
                1e-14,
                1e-9,
                &format!("{:.0}° uy", deg),
            );
            // Reaction = -applied global force; reaction moment = P*L.
            let (rx, ry) = (sol.reactions()[0], sol.reactions()[1]);
            assert_num(rx + gx, 0.0, 1e-6, 1e-9, "ΣFx");
            assert_num(ry + gy, 0.0, 1e-6, 1e-9, "ΣFy");
            assert_num(sol.reactions()[2], P * L, 1e-6, 1e-9, "Rz = P·L");
            // Local end forces are rotation invariant.
            let fe = sol.element_end_forces().unwrap()[0];
            assert_num(fe[1], -P, 1e-6, 1e-9, "local V_i");
            assert_num(fe[4], P, 1e-6, 1e-9, "local V_j");
            // Global end forces = T^T local end forces (applied exactly once).
            let feg = sol.element_end_forces_global().unwrap()[0];
            for i in 0..6 {
                let mut g = 0.0;
                for j in 0..6 {
                    g += t[j][i] * fe[j];
                }
                assert_num(g, feg[i], 1e-6, 1e-9, "f_global = T^T f_local");
            }
        }
    }
}

// ===========================================================================
// 9. Multi-element internal-force relations
// ===========================================================================

#[test]
fn test_multielement_internal_force_relations() {
    // 3-element cantilever with a UDL and a tip transverse force.
    let mut m = cantilever(3);
    for e in 0..3 {
        m.add_distributed_load(e, 0.0, -Q).unwrap();
    }
    m.add_nodal_force(3, 1, -P);

    let s = solve_named(&m, "dense");
    let dx = L / 3.0;

    // dM/dx = V: across each element (no interior concentrated load) the moment
    // change equals V * Δx using the mid-element shear.
    for e in 0..3 {
        let m0 = s.element_section_forces(e, 0.0).unwrap();
        let m1 = s.element_section_forces(e, 1.0).unwrap();
        let v_mid = s.element_section_forces(e, 0.5).unwrap().shear;
        assert_num(m1.moment - m0.moment, v_mid * dx, 1e-3, 1e-6, "ΔM = V Δx");
        // dV/dx = qy: with V(x) = -qy (L - x), the shear changes by qy*dx
        // (qy = -Q => the shear decreases by Q*dx).
        assert_num(m1.shear - m0.shear, -Q * dx, 1e-6, 1e-9, "ΔV = qy Δx");
    }

    // At the free end the shear just inside the element equals the tip force
    // (P); the UDL contributes zero shear there. The moment is zero.
    let free = s.element_section_forces(2, 1.0).unwrap();
    assert_num(free.shear, P, 1e-6, 1e-9, "V(free-inside)=P");
    assert_num(free.moment.abs(), 0.0, 1e-9, 1e-9, "M(free)=0");
    // Support moment = -(q L^2 / 2 + P L), support shear = qL + P.
    let sup = s.element_section_forces(0, 0.0).unwrap();
    assert_num(
        sup.moment,
        -(Q * L * L / 2.0 + P * L),
        1e-6,
        1e-9,
        "M(support)",
    );
    assert_num(sup.shear, Q * L + P, 1e-6, 1e-9, "V(support)");
    // Equilibrium.
    assert_equilibrium(
        &s,
        0.0,
        -Q * L - P,
        -(Q * L) * (L / 2.0) - P * L,
        "multi-element",
    );
}

// ===========================================================================
// 10. Mesh refinement classification
// ===========================================================================

#[test]
fn test_mesh_refinement_classification() {
    println!("[mesh audit] n  tip-force v(L) err | udl v(L) err | interior-point-load x=a/2 err");
    for &n in &[1usize, 2, 4, 8] {
        // (i) tip force: cubic element is nodally exact (equilibrium-determined).
        let mut mt = cantilever(n);
        mt.add_nodal_force(n, 1, -P);
        let st = solve_named(&mt, "dense");
        let e_tip = (st.displacement(n, 1) - (-P * L.powi(3) / (3.0 * EI))).abs()
            / (P * L.powi(3) / (3.0 * EI));
        assert!(
            e_tip < 1e-9,
            "n={}: tip-force error {:.3e} (should be nodal-exact)",
            n,
            e_tip
        );

        // (ii) UDL: nodal-exact as well (consistent loads).
        let mut mu = cantilever(n);
        for e in 0..n {
            mu.add_distributed_load(e, 0.0, -Q).unwrap();
        }
        let su = solve_named(&mu, "dense");
        let e_udl = (su.displacement(n, 1) - (-Q * L.powi(4) / (8.0 * EI))).abs()
            / (Q * L.powi(4) / (8.0 * EI));
        assert!(
            e_udl < 1e-9,
            "n={}: udl error {:.3e} (nodal-exact)",
            n,
            e_udl
        );

        // (iii) section moment at a NON-nodal station under UDL: post-processing
        // from the exact cubic field, also exact for a single element; for
        // multi-element meshes the value at x = L/4 must match closed form.
        let xq = L / 4.0;
        let exact_m = -Q * (L - xq).powi(2) / 2.0;
        let (e_idx, xi) = {
            // locate x = L/4 inside the mesh
            let dx = L / n as f64;
            let idx = (xq / dx).floor().min((n - 1) as f64) as usize;
            (idx, (xq - idx as f64 * dx) / dx)
        };
        let got_m = su.element_section_forces(e_idx, xi).unwrap().moment;
        let e_sec = (got_m - exact_m).abs() / exact_m.abs();
        assert!(e_sec < 1e-9, "n={}: section M error {:.3e}", n, e_sec);

        println!(
            "             {}  {:.3e}            {:.3e}       {:.3e}",
            n, e_tip, e_udl, e_sec
        );
    }
}

// ===========================================================================
// 11-14. Solver cross-check, scaling, equilibrium, result recovery
// ===========================================================================

#[test]
fn test_solver_cross_validation_representative() {
    let mut m = cantilever(3);
    m.add_distributed_load(0, 3e3, -Q).unwrap();
    m.add_point_load(1, 0.5, 0.0, -2e3, 1e3).unwrap();
    m.add_applied_moment(2, 4e3).unwrap();
    m.add_nodal_force(3, 1, -P);

    let reference = solve_named(&m, "dense");
    let u_ref = reference.displacements().to_vec();
    let r_ref = reference.reactions();
    let fe_ref = reference.element_end_forces().unwrap();

    for &name in &["skyline_ldlt", "sparse_lu"] {
        let s = solve_named(&m, name);
        for (i, &v) in u_ref.iter().enumerate() {
            assert_num(s.displacements()[i], v, 1e-13, 1e-9, "u cross-solver");
        }
        for (i, &v) in r_ref.iter().enumerate() {
            assert_num(s.reactions()[i], v, 1e-6, 1e-9, "R cross-solver");
        }
        let fe = s.element_end_forces().unwrap();
        for e in 0..3 {
            for k in 0..6 {
                assert_num(fe[e][k], fe_ref[e][k], 1e-6, 1e-9, "end force cross-solver");
                // section forces at matching stations
                for &xi in &[0.0, 0.5, 1.0] {
                    let a = s.element_section_forces(e, xi).unwrap();
                    let b = reference.element_section_forces(e, xi).unwrap();
                    assert_num(a.axial, b.axial, 1e-6, 1e-9, "N cross-solver");
                    assert_num(a.shear, b.shear, 1e-6, 1e-9, "V cross-solver");
                    assert_num(a.moment, b.moment, 1e-6, 1e-9, "M cross-solver");
                }
            }
        }
    }
}

#[test]
fn test_scaling_and_dimensional_sanity() {
    for &alpha in &[0.5_f64, 2.0] {
        // Load scaling: u -> alpha u, internal forces -> alpha * forces.
        let mut m1 = cantilever(2);
        m1.add_nodal_force(2, 1, -P);
        m1.add_nodal_force(2, 0, P);
        let s1 = solve_named(&m1, "dense");
        let mut m2 = cantilever(2);
        m2.add_nodal_force(2, 1, -alpha * P);
        m2.add_nodal_force(2, 0, alpha * P);
        let s2 = solve_named(&m2, "dense");
        for i in 0..s1.displacements().len() {
            assert_num(
                s2.displacements()[i],
                alpha * s1.displacements()[i],
                1e-14,
                1e-9,
                "u ∝ load",
            );
        }
        for i in 0..s1.reactions().len() {
            assert_num(
                s2.reactions()[i],
                alpha * s1.reactions()[i],
                1e-6,
                1e-9,
                "R ∝ load",
            );
        }

        // Material scaling: E -> alpha E => u/alpha; determinate forces unchanged.
        let mut m3 = cantilever(2);
        m3.add_nodal_force(2, 1, -P);
        for e in &mut m3.elements {
            e.material = Material::new(alpha * E, 0.3, 7850.0, "S");
        }
        let s3 = solve_named(&m3, "dense");
        for i in 0..s1.displacements().len() {
            // s1 carries both an axial and a transverse load; compare only the
            // bending slots against the bending-only model built above.
            if i % 3 == 1 || i % 3 == 2 {
                let mut mb = cantilever(2);
                mb.add_nodal_force(2, 1, -P);
                let sb = solve_named(&mb, "dense");
                assert_num(
                    s3.displacements()[i],
                    sb.displacements()[i] / alpha,
                    1e-14,
                    1e-9,
                    "u ∝ 1/E",
                );
            }
        }
        assert_num(s3.reactions()[1], P, 1e-6, 1e-9, "Ry independent of E");

        // Section scaling: I -> alpha I => bending u/alpha; A -> alpha A => axial u/alpha.
        let mut m4 = cantilever(2);
        m4.add_nodal_force(2, 1, -P);
        for e in &mut m4.elements {
            e.section = BeamSection::new(A, alpha * I);
        }
        let s4 = solve_named(&m4, "dense");
        assert_num(
            s4.displacement(2, 1),
            -P * L.powi(3) / (3.0 * EI * alpha),
            1e-14,
            1e-9,
            "u_bending ∝ 1/I",
        );
        // Geometry scaling for bending: v ∝ L^3, theta ∝ L^2 (single element).
        for &lg in &[1.0_f64, 3.0] {
            let mut mg = BeamModel::new();
            mg.add_node(BeamNode::new(0, 0.0, 0.0));
            mg.add_node(BeamNode::new(1, lg, 0.0));
            mg.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
            mg.fix_node(0);
            mg.add_nodal_force(1, 1, -P);
            let sg = solve_named(&mg, "dense");
            assert_num(
                sg.displacement(1, 1),
                -P * lg.powi(3) / (3.0 * EI),
                1e-14,
                1e-9,
                "v ∝ L^3",
            );
            assert_num(
                sg.displacement(1, 2),
                -P * lg.powi(2) / (2.0 * EI),
                1e-14,
                1e-9,
                "θ ∝ L^2",
            );
        }
    }
}

#[test]
fn test_equilibrium_identities_independent() {
    // Statically determinate cases: equilibrium must hold independently of the
    // displacement formulas.
    let mut m = cantilever(2);
    m.add_distributed_load(0, 2e3, -Q).unwrap();
    m.add_point_load(1, 0.5, 1e3, -3e3, 5e2).unwrap();
    m.add_applied_moment(1, 2e3).unwrap();
    m.add_nodal_force(2, 0, 4e3);
    m.add_nodal_force(2, 1, -6e3);

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        // External resultants (global X/Y, moments about node 0).
        let fx = 2e3 * (L / 2.0) + 1e3 + 4e3;
        let fy = -Q * (L / 2.0) - 3e3 - 6e3;
        let x_point = L / 2.0 + 0.5 * (L / 2.0); // xi=0.5 of element 1
        let mz = 2e3 // applied moment
            + (-Q * (L / 2.0)) * (L / 4.0) // UDL on element 0 at its centroid
            + (-3e3) * x_point // point force
            + 5e2 // point moment
            + (-6e3) * L // tip transverse
            + (2e3 * (L / 2.0)) * 0.0 // axial loads act on y = 0
            + (1e3) * 0.0
            + (4e3) * 0.0;
        assert_equilibrium(&s, fx, fy, mz, "combined equilibrium");

        // Element-level equilibrium against recovered end forces: for every
        // element, the sum of the two nodal transverse end forces equals the
        // element's applied transverse load (equivalent loads counted once).
        let fe = s.element_end_forces().unwrap();
        let loads_per_elem = [-Q * (L / 2.0), -3e3];
        for (e, load) in loads_per_elem.iter().enumerate() {
            assert_num(fe[e][1] + fe[e][4], *load, 1e-6, 1e-9, "element ΣV = load");
        }
        // Moment equilibrium of element 0 free body: couples + force arms
        // balance the distributed-load moment.
        let m0 = fe[0][2] + fe[0][5] + fe[0][4] * (L / 2.0);
        assert_num(
            m0,
            (-Q * (L / 2.0)) * (L / 4.0),
            1e-6,
            1e-9,
            "element ΣM = load moment",
        );
    }
}

#[test]
fn test_result_recovery_chain_equilibrium() {
    // End forces must place every element in equilibrium with its true loads,
    // including when an applied moment is present (which must NOT appear as an
    // element equivalent load).
    let mut m = cantilever(2);
    m.add_distributed_load(1, 0.0, -Q).unwrap();
    m.add_applied_moment(2, M0).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        let fe = s.element_end_forces().unwrap();
        // Element 0: no span load, but carries the effects of the rest; its end
        // force pair is self-equilibrated (ΣV = 0, ΣM = 0 about node_i).
        assert_num(fe[0][1] + fe[0][4], 0.0, 1e-6, 1e-9, "e0 ΣV = 0");
        assert_num(
            fe[0][2] + fe[0][5] + fe[0][4] * (L / 2.0),
            0.0,
            1e-6,
            1e-9,
            "e0 ΣM = 0",
        );
        // Element 1: UDL over L/2 => ΣV = -Q*(L/2), ΣM about node_i = load * centroid.
        let load = -Q * (L / 2.0);
        assert_num(fe[1][1] + fe[1][4], load, 1e-6, 1e-9, "e1 ΣV = load");
        assert_num(
            fe[1][2] + fe[1][5] + fe[1][4] * (L / 2.0),
            load * (L / 4.0),
            1e-6,
            1e-9,
            "e1 ΣM = load moment",
        );
        // The applied tip moment shows up in the nodal equilibrium at node 2:
        // end moment + applied = 0 (and NOT as an extra element load, which the
        // element ΣV/ΣM checks above already exclude).
        assert_num(fe[1][5] + M0, 0.0, 1e-6, 1e-9, "M_j + M_applied = 0");
    }
}

// ===========================================================================
// 15. Snapshot semantics (contract check)
// ===========================================================================

#[test]
fn test_snapshot_semantics_contract() {
    let mut m = cantilever(2);
    m.add_nodal_force(2, 1, -P);
    let mut s = BeamSolver::from_model(&m).unwrap();
    s.solve_configured().unwrap();
    let u_before = s.displacements().to_vec();

    // Mutating the source model must not affect the already-built solver.
    m.add_nodal_force(2, 1, -P);
    s.solve_configured().unwrap();
    for (i, &v) in u_before.iter().enumerate() {
        assert_eq!(
            s.displacements()[i],
            v,
            "snapshot: solver must not see mutations"
        );
    }
    // A rebuilt solver reflects the mutation (response doubles).
    let s2 = solve_named(&m, "dense");
    for (i, &v) in u_before.iter().enumerate() {
        assert_num(
            s2.displacements()[i],
            2.0 * v,
            1e-14,
            1e-9,
            "rebuild sees mutation",
        );
    }
}

// ===========================================================================
// Regression: symmetry capability check must be scale aware
// ===========================================================================

#[test]
fn test_symmetry_check_is_scale_aware() {
    // A matrix assembled in floating point is symmetric only up to round-off,
    // and that round-off scales with the entry magnitude. Regression: with a
    // purely absolute tolerance, a stiffness matrix with EA/L ~ 1e8 (round-off
    // asymmetry ~1e-8) was misclassified as non-symmetric, which silently
    // disabled skyline_ldlt / cg / iccg for realistic models.
    use section_properties::fea::SparseMatrix;

    let build = |scale: f64, perturb: f64| {
        let mut a = SparseMatrix::new(3);
        a.add(0, 0, scale);
        a.add(0, 1, 0.3714 * scale);
        a.add(1, 0, 0.3714 * scale + perturb);
        a.add(1, 1, 0.6 * scale);
        a.add(2, 2, 0.25 * scale);
        a.compress();
        a
    };

    // Round-off sized asymmetry (relative ~1e-16) must still be symmetric...
    let big = build(5e8, 3e-8);
    assert!(
        big.is_symmetric(1e-12),
        "scale 5e8 with 3e-8 round-off asymmetry must be symmetric"
    );
    // ... at every magnitude.
    for &scale in &[1.0_f64, 1e4, 1e8, 5e8] {
        assert!(
            build(scale, scale * 1e-15).is_symmetric(1e-12),
            "scale {} with relative round-off must be symmetric",
            scale
        );
    }
    // A genuinely non-symmetric matrix must still be rejected.
    let nonsym = build(1.0, 0.5);
    assert!(
        !nonsym.is_symmetric(1e-12),
        "genuine asymmetry must be detected"
    );
    let nonsym_big = build(5e8, 0.25 * 5e8);
    assert!(
        !nonsym_big.is_symmetric(1e-12),
        "genuine asymmetry must be detected at large magnitude too"
    );
}

#[test]
fn test_realistic_magnitude_symmetric_backends_accepted() {
    // End-to-end: at realistic magnitudes (E = 200 GPa) the rotated model's
    // condensed matrix is symmetric to machine precision, so skyline_ldlt must
    // be accepted and agree with dense / sparse_lu. Regression for the
    // absolute-tolerance symmetry check above.
    for &deg in &[45.0_f64, 135.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, L * c, L * s));
        m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        m.fix_node(0);
        m.add_nodal_force(1, 0, -s * (-P));
        m.add_nodal_force(1, 1, c * (-P));

        let reference = solve_named(&m, "dense");
        let s_sky = solve_named(&m, "skyline_ldlt");
        assert_eq!(
            s_sky.solver_name(),
            Some("skyline_ldlt"),
            "skyline must be usable"
        );
        for i in 0..reference.displacements().len() {
            assert_num(
                s_sky.displacements()[i],
                reference.displacements()[i],
                1e-13,
                1e-9,
                "skyline vs dense at realistic magnitude",
            );
        }
    }
}

// ===========================================================================
// Phase 8/§6 — audit of the scale-aware symmetry predicate
// ===========================================================================

#[test]
fn test_symmetry_predicate_audit() {
    use section_properties::fea::SparseMatrix;

    // Build a 3x3 sparse matrix from an explicit dense array.
    let from_dense = |m: [[f64; 3]; 3]| {
        let mut a = SparseMatrix::new(3);
        for (i, row) in m.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                if v != 0.0 {
                    a.add(i, j, v);
                }
            }
        }
        a.compress();
        a
    };
    let diag = |scale: f64| {
        [
            [scale, 0.0, 0.0],
            [0.0, 0.5 * scale, 0.0],
            [0.0, 0.0, 0.25 * scale],
        ]
    };

    // (1) Exactly symmetric matrices across 12 orders of magnitude.
    for &scale in &[1e-12_f64, 1.0, 1e4, 1e8, 1e12] {
        assert!(
            from_dense(diag(scale)).is_symmetric(1e-12),
            "exactly symmetric at scale {} must be detected",
            scale
        );
    }
    // (2) Negative and mixed-scale entries, still symmetric.
    assert!(
        from_dense([[-5e8, 1e-9, 0.0], [1e-9, -2.0, 3e6], [0.0, 3e6, -1e12]]).is_symmetric(1e-12)
    );
    // (3) Zero matrix / all-zero off-diagonals.
    assert!(from_dense([[0.0; 3]; 3]).is_symmetric(1e-12));
    // (4) Very small but exactly mirrored nonzero entries.
    assert!(
        from_dense([[1e-14, 2e-14, 0.0], [2e-14, 1e-14, 0.0], [0.0, 0.0, 1.0]]).is_symmetric(1e-12)
    );

    // (5) T^T K T at realistic engineering magnitudes (round-off asymmetry).
    for &deg in &[45.0_f64, 135.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let (e, a, i, l) = (200e9, 5e-3, 2e-5, 2.0);
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, l * c, l * s));
        m.add_element(
            BeamElement::new(
                0,
                1,
                Material::new(e, 0.3, 7850.0, "S"),
                BeamSection::new(a, i),
            )
            .unwrap(),
        );
        let (ni, nj) = (m.nodes[0].point(), m.nodes[1].point());
        let el = &m.elements[0];
        let k_local = el.local_stiffness(ni, nj);
        let t = el.transformation_matrix(ni, nj);
        // kg = T^T k_local T
        let mut kt = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for cc in 0..6 {
                for k in 0..6 {
                    kt[r][cc] += t[k][r] * k_local[k][cc];
                }
            }
        }
        let mut kg = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for cc in 0..6 {
                for k in 0..6 {
                    kg[r][cc] += kt[r][k] * t[k][cc];
                }
            }
        }
        let mut sp = SparseMatrix::new(6);
        for (r, row) in kg.iter().enumerate() {
            for (cc, &v) in row.iter().enumerate() {
                if v != 0.0 {
                    sp.add(r, cc, v);
                }
            }
        }
        sp.compress();
        assert!(
            sp.is_symmetric(1e-12),
            "{:.0}°: T^T K T at realistic magnitude must be symmetric (asym rel ~1e-16)",
            deg
        );
    }

    // (6) Genuine asymmetry must NOT be hidden by large magnitudes.
    //     Small matrix:
    assert!(!from_dense([[1.0, 2.0, 0.0], [0.5, 1.0, 0.0], [0.0, 0.0, 1.0]]).is_symmetric(1e-12));
    //     Large matrix, O(1) relative asymmetry:
    assert!(!from_dense([[5e8, 1e8, 0.0], [4e8, 5e8, 0.0], [0.0, 0.0, 5e8]]).is_symmetric(1e-12));
    //     One-sided missing entry at large magnitude (a_ij present, a_ji zero):
    assert!(!from_dense([[5e8, 7e7, 0.0], [0.0, 5e8, 0.0], [0.0, 0.0, 5e8]]).is_symmetric(1e-12));
    //     Mixed-scale: a genuine 1e-6 *relative* asymmetry at 1e8 magnitude is
    //     far above the 1e-12 relative threshold and must be rejected.
    assert!(
        !from_dense([
            [1e8, 1e8 * (1.0 + 1e-6), 0.0],
            [1e8, 1e8, 0.0],
            [0.0, 0.0, 1e8]
        ])
        .is_symmetric(1e-12)
    );
}
