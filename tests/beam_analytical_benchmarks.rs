//! Phase 2.7 — Analytical benchmarks and physics validation for the full Beam
//! FEM pipeline (model → solver → displacements → reactions → end forces →
//! section N/V/M → diagrams → result API).
//!
//! This file is a **validation** suite: it does not add FEM functionality.
//! Every expected value is derived from the project's documented conventions:
//!
//! * global DOFs per node `[ux, uy, rz]`, `rz` counter-clockwise (CCW) positive;
//! * nodal forces / applied moments are GLOBAL; distributed & point loads are
//!   LOCAL (`qy`/`fy` positive upward, `qx`/`fx` positive tensile, `mz` CCW);
//! * section forces are LOCAL: `axial` tension+, `shear = d(moment)/dx`,
//!   `moment` sagging positive (`M = E·I·v''`);
//! * reactions are GLOBAL external support forces/moments.
//!
//! For a horizontal beam along +x, local and global axes coincide.

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, SectionForces,
};
use section_properties::fea::solver::SolverRegistry;
use section_properties::material::Material;

// ---------------------------------------------------------------------------
// Shared parameters and helpers
// ---------------------------------------------------------------------------

const E0: f64 = 200e9;
const A0: f64 = 0.02;

fn i0() -> f64 {
    0.1 * 0.2_f64.powi(3) / 12.0
}

fn material(e: f64) -> Material {
    Material::new(e, 0.3, 7850.0, "Steel")
}

fn solve(model: &BeamModel) -> BeamSolver {
    let mut solver = BeamSolver::from_model(model).unwrap();
    let registry = SolverRegistry::default();
    let mut linear = registry.create("dense").unwrap();
    solver.solve(&mut *linear).unwrap();
    solver
}

/// Horizontal cantilever: `node0` fully fixed, `n_elem` equal elements of
/// total length `total`, section `(a, i)`, material modulus `e`.
fn beam_chain(n_elem: usize, total: f64, e: f64, a: f64, i: f64) -> BeamModel {
    let mut model = BeamModel::new();
    let dx = total / n_elem as f64;
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n_elem {
        model.add_element(BeamElement::new(k, k + 1, material(e), BeamSection::new(a, i)).unwrap());
    }
    model.fix_node(0);
    model
}

/// Section forces at physical position `x` on a uniform `n_elem` mesh.
fn section_at(solver: &BeamSolver, n_elem: usize, total: f64, x: f64) -> SectionForces {
    let dx = total / n_elem as f64;
    let mut e = (x / dx).floor() as usize;
    if e >= n_elem {
        e = n_elem - 1;
    }
    let xi = ((x - e as f64 * dx) / dx).clamp(0.0, 1.0);
    solver.element_section_forces(e, xi).unwrap()
}

fn rel_err(actual: f64, expected: f64) -> f64 {
    let scale = expected.abs().max(1e-30);
    (actual - expected).abs() / scale
}

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    let scale = expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{}: got {:.12e}, expected {:.12e} (tol {:e})",
        label,
        actual,
        expected,
        tol
    );
}

// Analytical formulas (magnitudes; signs applied by callers per convention).
fn cantilever_tip_disp(p: f64, l: f64, ei: f64) -> f64 {
    p * l.powi(3) / (3.0 * ei)
}
fn cantilever_tip_rot(p: f64, l: f64, ei: f64) -> f64 {
    p * l.powi(2) / (2.0 * ei)
}
fn cantilever_udl_tip_disp(q: f64, l: f64, ei: f64) -> f64 {
    q * l.powi(4) / (8.0 * ei)
}
fn cantilever_udl_tip_rot(q: f64, l: f64, ei: f64) -> f64 {
    q * l.powi(3) / (6.0 * ei)
}
fn cantilever_moment_tip_rot(m: f64, l: f64, ei: f64) -> f64 {
    m * l / ei
}

/// Composite Simpson integration of `f` over a uniform grid of `n` intervals.
fn simpson(f: &[f64], dx: f64) -> f64 {
    let n = f.len() - 1;
    assert!(
        n.is_multiple_of(2),
        "Simpson requires an even number of intervals"
    );
    let mut s = f[0] + f[n];
    for (k, &v) in f.iter().enumerate().take(n).skip(1) {
        s += if k % 2 == 0 { 2.0 * v } else { 4.0 * v };
    }
    s * dx / 3.0
}

// ===========================================================================
// Benchmark A — axial bar
// ===========================================================================

#[test]
fn test_benchmark_a_axial_bar() {
    let (l, e, a) = (1.0, E0, A0);
    let p = 1000.0;

    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l, 0.0));
    model.add_element(BeamElement::new(0, 1, material(e), BeamSection::new(a, i0())).unwrap());
    model.fix_node(0);
    model.add_nodal_force(1, 0, p); // global +x tension
    let solver = solve(&model);
    let r = solver.results();

    let ea = e * a;
    let u_tip = p * l / ea;
    assert_close(r.displacement(1).unwrap().ux, u_tip, 1e-12, "tip ux");
    assert_close(r.displacement(1).unwrap().uy, 0.0, 1e-12, "tip uy");

    // Reaction balances the applied load: ΣFx = 0.
    let rx = r.reaction(0).unwrap().fx;
    assert_close(rx, -p, 1e-10, "Rx");
    assert_close(rx + p, 0.0, 1e-10, "ΣFx");

    // Axial force and section force.
    for &xi in &[0.0, 0.5, 1.0] {
        assert_close(
            solver.element_section_forces(0, xi).unwrap().axial,
            p,
            1e-12,
            "N(x)",
        );
    }

    // Element-on-node end forces: [+N_i, 0,0, -N_j, 0,0] for tension.
    let f = solver.element_end_forces().unwrap()[0];
    assert_close(f[0], p, 1e-10, "N_i");
    assert_close(f[3], -p, 1e-10, "N_j");
    for k in [1, 2, 4, 5] {
        assert_close(f[k], 0.0, 1e-12, "zero end-force component");
    }

    println!(
        "[A axial] u_tip FE={:.12e} an={:.12e} | Rx FE={:.12e} an={:.12e}",
        r.displacement(1).unwrap().ux,
        u_tip,
        rx,
        -p
    );
}

// ===========================================================================
// Benchmark B — cantilever tip transverse force
// ===========================================================================

#[test]
fn test_benchmark_b_cantilever_tip_force() {
    let (l, e, i, p) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    let mut model = beam_chain(1, l, e, A0, i);
    model.add_nodal_force(1, 1, -p); // downward
    let solver = solve(&model);
    let r = solver.results();

    // Tip displacement / rotation.
    let uy = r.displacement(1).unwrap().uy;
    let rz = r.displacement(1).unwrap().rz;
    assert_close(uy, -cantilever_tip_disp(p, l, ei), 1e-10, "tip uy");
    assert_close(rz, -cantilever_tip_rot(p, l, ei), 1e-10, "tip rz");

    // Reactions: Ry = +P, Rz = +P·L; ΣFy = 0, ΣMz = 0.
    let (ry, mr) = (r.reaction(0).unwrap().fy, r.reaction(0).unwrap().mz);
    assert_close(ry, p, 1e-10, "Ry");
    assert_close(ry + (-p), 0.0, 1e-10, "ΣFy");
    assert_close(mr, p * l, 1e-10, "Rz");
    assert_close(mr + (-p) * l, 0.0, 1e-10, "ΣMz");

    // Section diagram: V = +P, M = -P(L - x), M(L) = 0.
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = xi * l;
        let s = solver.element_section_forces(0, xi).unwrap();
        assert_close(s.shear, p, 1e-10, "V");
        assert_close(s.moment, -p * (l - x), 1e-10, "M");
        assert_close(s.axial, 0.0, 1e-12, "N");
    }
    assert_close(
        solver.element_section_forces(0, 1.0).unwrap().moment,
        0.0,
        1e-12,
        "free-end M",
    );

    println!(
        "[B tip force] uy FE={:.12e} an={:.12e} err={:.3e}",
        uy,
        -cantilever_tip_disp(p, l, ei),
        rel_err(uy, -cantilever_tip_disp(p, l, ei))
    );
}

// ===========================================================================
// Benchmark C — cantilever tip moment
// ===========================================================================

#[test]
fn test_benchmark_c_cantilever_tip_moment() {
    let (l, e, i, m) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    let mut model = beam_chain(1, l, e, A0, i);
    model.add_applied_moment(1, m).unwrap(); // CCW at free end
    let solver = solve(&model);
    let r = solver.results();

    // Tip rotation θ = M·L/(E·I); constant curvature gives uy = M·L²/(2EI).
    assert_close(
        r.displacement(1).unwrap().rz,
        cantilever_moment_tip_rot(m, l, ei),
        1e-10,
        "tip rz",
    );
    assert_close(
        r.displacement(1).unwrap().uy,
        m * l * l / (2.0 * ei),
        1e-10,
        "tip uy",
    );

    // Reaction moment balances M: Rz = -M, Ry = 0.
    let rr = r.reaction(0).unwrap();
    assert_close(rr.mz, -m, 1e-10, "Rz");
    assert_close(rr.mz + m, 0.0, 1e-10, "ΣMz");
    assert_close(rr.fy, 0.0, 1e-12, "Ry");

    // Section: constant +M (sagging), zero shear.
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let s = solver.element_section_forces(0, xi).unwrap();
        assert_close(s.moment, m, 1e-10, "M constant");
        assert_close(s.shear, 0.0, 1e-12, "V=0");
        assert_close(s.axial, 0.0, 1e-12, "N=0");
    }

    // Phase 2.4 nodal-moment convention: free-end element-on-node moment = -M.
    let f = solver.element_end_forces().unwrap()[0];
    assert_close(f[5], -m, 1e-10, "M_j = -M");

    println!(
        "[C tip moment] rz FE={:.12e} an={:.12e} err={:.3e}",
        r.displacement(1).unwrap().rz,
        cantilever_moment_tip_rot(m, l, ei),
        rel_err(
            r.displacement(1).unwrap().rz,
            cantilever_moment_tip_rot(m, l, ei)
        )
    );
}

// ===========================================================================
// Benchmark D — cantilever uniform transverse load
// ===========================================================================

#[test]
fn test_benchmark_d_cantilever_udl() {
    let (l, e, i, q) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    let mut model = beam_chain(1, l, e, A0, i);
    model.add_distributed_load(0, 0.0, -q).unwrap(); // downward
    let solver = solve(&model);
    let r = solver.results();

    // Reactions: Ry = qL, Rz = qL²/2.
    let rr = r.reaction(0).unwrap();
    assert_close(rr.fy, q * l, 1e-10, "Ry = qL");
    assert_close(rr.fy + (-q * l), 0.0, 1e-10, "ΣFy");
    assert_close(rr.mz, q * l * l / 2.0, 1e-10, "Rz = qL²/2");
    assert_close(rr.mz + (-q * l * l / 2.0), 0.0, 1e-10, "ΣMz");

    // Tip displacement / rotation.
    let uy = r.displacement(1).unwrap().uy;
    let rz = r.displacement(1).unwrap().rz;
    assert_close(uy, -cantilever_udl_tip_disp(q, l, ei), 1e-10, "tip uy");
    assert_close(rz, -cantilever_udl_tip_rot(q, l, ei), 1e-10, "tip rz");

    // Diagram: |V| = q(L-x), |M| = q(L-x)²/2 with the project signs
    // V = +q(L-x), M = -q(L-x)²/2.
    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let x = xi * l;
        let s = solver.element_section_forces(0, xi).unwrap();
        assert_close(s.shear, q * (l - x), 1e-9, "V");
        assert_close(s.moment, -q * (l - x).powi(2) / 2.0, 1e-9, "M");
    }
    assert_close(
        solver.element_section_forces(0, 1.0).unwrap().moment,
        0.0,
        1e-12,
        "free-end M",
    );

    println!(
        "[D udl] uy FE={:.12e} an={:.12e} err={:.3e}",
        uy,
        -cantilever_udl_tip_disp(q, l, ei),
        rel_err(uy, -cantilever_udl_tip_disp(q, l, ei))
    );
}

// ===========================================================================
// Benchmark E — combined loading (superposition)
// ===========================================================================

#[test]
fn test_benchmark_e_superposition() {
    let (l, e, i) = (1.0, E0, i0());
    let (p_ax, p_tr, m, q) = (500.0, 1000.0, 800.0, 400.0);

    let base = || beam_chain(1, l, e, A0, i);

    // Individual load cases.
    let mut ax = base();
    ax.add_nodal_force(1, 0, p_ax);
    let mut tr = base();
    tr.add_nodal_force(1, 1, -p_tr);
    let mut mo = base();
    mo.add_applied_moment(1, m).unwrap();
    let mut ud = base();
    ud.add_distributed_load(0, 0.0, -q).unwrap();

    let s_ax = solve(&ax);
    let s_tr = solve(&tr);
    let s_mo = solve(&mo);
    let s_ud = solve(&ud);

    // Combined load case.
    let mut comb = base();
    comb.add_nodal_force(1, 0, p_ax);
    comb.add_nodal_force(1, 1, -p_tr);
    comb.add_applied_moment(1, m).unwrap();
    comb.add_distributed_load(0, 0.0, -q).unwrap();
    let s_c = solve(&comb);
    let r_c = s_c.results();

    // Superposition of all DOFs.
    for dof in 0..3 {
        let sum = s_ax.displacement(1, dof)
            + s_tr.displacement(1, dof)
            + s_mo.displacement(1, dof)
            + s_ud.displacement(1, dof);
        let got = s_c.displacement(1, dof);
        assert_close(got, sum, 1e-9, &format!("superposed disp dof{}", dof));
    }

    // Superposition of reactions and section forces.
    let sum_ry =
        s_ax.reactions()[1] + s_tr.reactions()[1] + s_mo.reactions()[1] + s_ud.reactions()[1];
    assert_close(r_c.reaction(0).unwrap().fy, sum_ry, 1e-9, "superposed Ry");
    let sum_mz =
        s_ax.reactions()[2] + s_tr.reactions()[2] + s_mo.reactions()[2] + s_ud.reactions()[2];
    assert_close(r_c.reaction(0).unwrap().mz, sum_mz, 1e-9, "superposed Rz");

    for &xi in &[0.0, 0.3, 0.7, 1.0] {
        let sc = s_c.element_section_forces(0, xi).unwrap();
        let n_sum = s_ax.element_section_forces(0, xi).unwrap().axial
            + s_tr.element_section_forces(0, xi).unwrap().axial
            + s_mo.element_section_forces(0, xi).unwrap().axial
            + s_ud.element_section_forces(0, xi).unwrap().axial;
        let v_sum = s_ax.element_section_forces(0, xi).unwrap().shear
            + s_tr.element_section_forces(0, xi).unwrap().shear
            + s_mo.element_section_forces(0, xi).unwrap().shear
            + s_ud.element_section_forces(0, xi).unwrap().shear;
        let m_sum = s_ax.element_section_forces(0, xi).unwrap().moment
            + s_tr.element_section_forces(0, xi).unwrap().moment
            + s_mo.element_section_forces(0, xi).unwrap().moment
            + s_ud.element_section_forces(0, xi).unwrap().moment;
        assert_close(sc.axial, n_sum, 1e-9, &format!("superposed N xi={}", xi));
        assert_close(sc.shear, v_sum, 1e-9, &format!("superposed V xi={}", xi));
        assert_close(sc.moment, m_sum, 1e-9, &format!("superposed M xi={}", xi));
    }

    println!("[E superposition] combined == sum of 4 individual load cases");
}

// ===========================================================================
// Benchmark F — mesh convergence
// ===========================================================================

#[test]
fn test_benchmark_f_mesh_convergence() {
    let (l, e, i) = (1.0, E0, i0());
    let ei = e * i;
    let p = 1000.0;
    let q = 1000.0;
    let counts = [1usize, 2, 4, 8, 16];

    println!("[F mesh convergence] tip-force and UDL, n = 1,2,4,8,16");
    for &n in &counts {
        // --- tip force ---
        let mut mt = beam_chain(n, l, e, A0, i);
        mt.add_nodal_force(n, 1, -p);
        let st = solve(&mt);
        let uy = st.displacement(n, 1);
        let e_uy = rel_err(uy, -cantilever_tip_disp(p, l, ei));
        let ry = st.reactions()[1];
        let e_ry = rel_err(ry, p);
        let m_mid = section_at(&st, n, l, 0.5).moment;
        let e_m = rel_err(m_mid, -p * (l - 0.5));
        let v_mid = section_at(&st, n, l, 0.5).shear;
        let e_v = rel_err(v_mid, p);

        // --- udl ---
        let mut mu = beam_chain(n, l, e, A0, i);
        for k in 0..n {
            mu.add_distributed_load(k, 0.0, -q).unwrap();
        }
        let su = solve(&mu);
        let uy_u = su.displacement(n, 1);
        let e_uy_u = rel_err(uy_u, -cantilever_udl_tip_disp(q, l, ei));
        let ry_u = su.reactions()[1];
        let e_ry_u = rel_err(ry_u, q * l);
        let m_mid_u = section_at(&su, n, l, 0.5).moment;
        let e_m_u = rel_err(m_mid_u, -q * (l - 0.5).powi(2) / 2.0);
        let v_mid_u = section_at(&su, n, l, 0.5).shear;
        let e_v_u = rel_err(v_mid_u, q * (l - 0.5));

        println!(
            "  n={:2} | tipF: uy_err={:.2e} Ry_err={:.2e} M_err={:.2e} V_err={:.2e} | udl: uy_err={:.2e} Ry_err={:.2e} M_err={:.2e} V_err={:.2e}",
            n, e_uy, e_ry, e_m, e_v, e_uy_u, e_ry_u, e_m_u, e_v_u
        );

        // Statically determinate reactions / section forces must be exact for
        // every mesh (equilibrium-determined).
        assert!(e_ry < 1e-9, "n={} tip Ry err {}", n, e_ry);
        assert!(e_m < 1e-9, "n={} tip M err {}", n, e_m);
        assert!(e_v < 1e-9, "n={} tip V err {}", n, e_v);
        assert!(e_ry_u < 1e-9, "n={} udl Ry err {}", n, e_ry_u);
        assert!(e_m_u < 1e-9, "n={} udl M err {}", n, e_m_u);
        assert!(e_v_u < 1e-9, "n={} udl V err {}", n, e_v_u);

        // Nodal displacements are exact for this formulation.
        assert!(e_uy < 1e-9, "n={} tip uy err {}", n, e_uy);
        assert!(e_uy_u < 1e-9, "n={} udl uy err {}", n, e_uy_u);
    }
}

// ===========================================================================
// Benchmark G — unequal element lengths
// ===========================================================================

#[test]
fn test_benchmark_g_unequal_lengths() {
    let (e, i, p) = (E0, i0(), 1000.0);
    let ei = e * i;
    let l_total = 6.0;

    // Nodes at x = 0, 1, 3, 6 (L0=1, L1=2, L2=3).
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_node(BeamNode::new(2, 3.0, 0.0));
    model.add_node(BeamNode::new(3, 6.0, 0.0));
    model.add_element(BeamElement::new(0, 1, material(e), BeamSection::new(A0, i)).unwrap());
    model.add_element(BeamElement::new(1, 2, material(e), BeamSection::new(A0, i)).unwrap());
    model.add_element(BeamElement::new(2, 3, material(e), BeamSection::new(A0, i)).unwrap());
    model.fix_node(0);
    model.add_nodal_force(3, 1, -p);
    let solver = solve(&model);

    // Phase 2.5 global x positions must reflect true element lengths.
    let samples = solver.sample_beam_forces(3).unwrap();
    let expected_x = [0.0, 0.5, 1.0, 1.0, 2.0, 3.0, 3.0, 4.5, 6.0];
    for (s, xe) in samples.iter().zip(expected_x.iter()) {
        assert_close(s.x, *xe, 1e-12, "global x");
    }

    // Reactions (total length 6): Ry = P, Rz = 6P.
    let r = solver.results();
    assert_close(r.reaction(0).unwrap().fy, p, 1e-10, "Ry");
    assert_close(r.reaction(0).unwrap().mz, p * l_total, 1e-10, "Rz");

    // Tip displacement equals that of a single length-6 cantilever.
    assert_close(
        r.displacement(3).unwrap().uy,
        -cantilever_tip_disp(p, l_total, ei),
        1e-9,
        "tip uy",
    );

    // Section forces are equilibrium-exact regardless of mesh.
    for (elem, x) in [(0usize, 0.5), (1, 2.0), (2, 4.5)] {
        let xi = {
            let dx = [1.0, 2.0, 3.0][elem];
            let x0 = [0.0, 1.0, 3.0][elem];
            (x - x0) / dx
        };
        let s = solver.element_section_forces(elem, xi).unwrap();
        assert_close(s.shear, p, 1e-9, "V");
        assert_close(s.moment, -p * (l_total - x), 1e-9, "M");
    }

    println!("[G unequal lengths] x positions, reactions, tip disp, section forces OK");
}

// ===========================================================================
// Benchmark H — coordinate rotation (0°, 45°, 90°)
// ===========================================================================

#[test]
fn test_benchmark_h_rotation_invariance() {
    let (l, e, i, p) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    for &deg in &[0.0_f64, 45.0, 90.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());

        // Rotated cantilever of length L along direction (c, s).
        let mut model = BeamModel::new();
        model.add_node(BeamNode::new(0, 0.0, 0.0));
        model.add_node(BeamNode::new(1, l * c, l * s));
        model.add_element(BeamElement::new(0, 1, material(e), BeamSection::new(A0, i)).unwrap());
        model.fix_node(0);
        // Global tip force equivalent to local (0, -P):  f_global = Tᵀ f_local.
        let fgx = s * p;
        let fgy = -c * p;
        model.add_nodal_force(1, 0, fgx);
        model.add_nodal_force(1, 1, fgy);
        let solver = solve(&model);
        let r = solver.results();

        // Local displacement via T (global -> local): u_local = T u_global.
        let ux_g = r.displacement(1).unwrap().ux;
        let uy_g = r.displacement(1).unwrap().uy;
        let uy_l = -s * ux_g + c * uy_g;
        assert_close(
            uy_l,
            -cantilever_tip_disp(p, l, ei),
            1e-9,
            &format!("local tip deflection @ {}deg", deg),
        );

        // Reaction magnitude invariant; moment = P·L about z.
        let rr = r.reaction(0).unwrap();
        let rmag = (rr.fx * rr.fx + rr.fy * rr.fy).sqrt();
        assert_close(rmag, p, 1e-9, &format!("reaction magnitude @ {}deg", deg));
        assert_close(rr.mz, p * l, 1e-9, &format!("reaction moment @ {}deg", deg));

        // Local section N/V/M are identical to the unrotated case.
        for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let x = xi * l;
            let sf = solver.element_section_forces(0, xi).unwrap();
            assert_close(sf.axial, 0.0, 1e-10, "local N");
            assert_close(sf.shear, p, 1e-9, &format!("local V @ {}deg", deg));
            assert_close(
                sf.moment,
                -p * (l - x),
                1e-9,
                &format!("local M @ {}deg", deg),
            );
        }

        println!(
            "[H rotation {:>4}deg] local tip uy={:.9e} (an {:.9e}), |R|={:.6e}, Rz={:.6e}",
            deg,
            uy_l,
            -cantilever_tip_disp(p, l, ei),
            rmag,
            rr.mz
        );
    }
}

// ===========================================================================
// Benchmark I — material / load / length scaling
// ===========================================================================

#[test]
fn test_benchmark_i_scaling_laws() {
    let (l, i) = (1.0, i0());
    let (p, q, m) = (1000.0, 1000.0, 1000.0);
    let alpha = 3.0;

    // --- E scaling: displacements/rotations ∝ 1/E, reactions/forces fixed ---
    {
        let mut a = beam_chain(1, l, E0, A0, i);
        a.add_nodal_force(1, 1, -p);
        let sa = solve(&a);
        let mut b = beam_chain(1, l, alpha * E0, A0, i);
        b.add_nodal_force(1, 1, -p);
        let sb = solve(&b);

        assert_close(
            sb.displacement(1, 1) * alpha,
            sa.displacement(1, 1),
            1e-9,
            "uy ∝ 1/E",
        );
        assert_close(
            sb.displacement(1, 2) * alpha,
            sa.displacement(1, 2),
            1e-9,
            "rz ∝ 1/E",
        );
        assert_close(
            sb.reactions()[1],
            sa.reactions()[1],
            1e-9,
            "Ry independent of E",
        );
        assert_close(
            sb.element_section_forces(0, 0.5).unwrap().moment,
            sa.element_section_forces(0, 0.5).unwrap().moment,
            1e-9,
            "M independent of E",
        );
    }

    // --- load scaling: everything ∝ α ---
    for (label, build) in [("P", 0usize), ("q", 1usize), ("M", 2usize)] {
        let make = |scale: f64| {
            let mut m0 = beam_chain(1, l, E0, A0, i);
            match build {
                0 => {
                    m0.add_nodal_force(1, 1, -p * scale);
                }
                1 => {
                    m0.add_distributed_load(0, 0.0, -q * scale).unwrap();
                }
                _ => {
                    m0.add_applied_moment(1, m * scale).unwrap();
                }
            }
            m0
        };
        let s1 = solve(&make(1.0));
        let s2 = solve(&make(alpha));
        assert_close(
            s2.displacement(1, 1),
            alpha * s1.displacement(1, 1),
            1e-9,
            &format!("{} uy ∝ α", label),
        );
        assert_close(
            s2.displacement(1, 2),
            alpha * s1.displacement(1, 2),
            1e-9,
            &format!("{} rz ∝ α", label),
        );
        assert_close(
            s2.reactions()[1],
            alpha * s1.reactions()[1],
            1e-9,
            &format!("{} Ry ∝ α", label),
        );
        assert_close(
            s2.element_section_forces(0, 0.5).unwrap().moment,
            alpha * s1.element_section_forces(0, 0.5).unwrap().moment,
            1e-9,
            &format!("{} M ∝ α", label),
        );
    }

    // --- length scaling (I held fixed): tip force ∝ L³, UDL ∝ L⁴ ---
    {
        let (l1, l2) = (1.0, 2.0);
        let mut a = beam_chain(1, l1, E0, A0, i);
        a.add_nodal_force(1, 1, -p);
        let sa = solve(&a);
        let mut b = beam_chain(1, l2, E0, A0, i);
        b.add_nodal_force(1, 1, -p);
        let sb = solve(&b);
        let ratio = (l2 / l1).powi(3);
        assert_close(
            sb.displacement(1, 1) / sa.displacement(1, 1),
            ratio,
            1e-9,
            "tip-force uy ∝ L³",
        );

        let mut a = beam_chain(1, l1, E0, A0, i);
        a.add_distributed_load(0, 0.0, -q).unwrap();
        let sa = solve(&a);
        let mut b = beam_chain(1, l2, E0, A0, i);
        b.add_distributed_load(0, 0.0, -q).unwrap();
        let sb = solve(&b);
        let ratio4 = (l2 / l1).powi(4);
        assert_close(
            sb.displacement(1, 1) / sa.displacement(1, 1),
            ratio4,
            1e-9,
            "udl uy ∝ L⁴",
        );
    }

    println!("[I scaling] E→1/α, loads→α, tip-force L³, udl L⁴ all verified");
}

// ===========================================================================
// Benchmark J — section-property scaling (A and I)
// ===========================================================================

#[test]
fn test_benchmark_j_section_property_scaling() {
    let (l, e) = (1.0, E0);
    let (p, f) = (1000.0, 1000.0);
    let i_ref = i0();
    let a_ref = A0;

    // Bending: u ∝ 1/I at fixed A and E.
    let mut bend_ref = beam_chain(1, l, e, a_ref, i_ref);
    bend_ref.add_nodal_force(1, 1, -p);
    let s_bend_ref = solve(&bend_ref);
    let mut bend_2i = beam_chain(1, l, e, a_ref, 2.0 * i_ref);
    bend_2i.add_nodal_force(1, 1, -p);
    let s_bend_2i = solve(&bend_2i);
    assert_close(
        s_bend_2i.displacement(1, 1),
        s_bend_ref.displacement(1, 1) / 2.0,
        1e-9,
        "bending u ∝ 1/I",
    );

    // Axial: u ∝ 1/A at fixed E and I.
    let mut ax_ref = beam_chain(1, l, e, a_ref, i_ref);
    ax_ref.add_nodal_force(1, 0, f);
    let s_ax_ref = solve(&ax_ref);
    let mut ax_4a = beam_chain(1, l, e, 4.0 * a_ref, i_ref);
    ax_4a.add_nodal_force(1, 0, f);
    let s_ax_4a = solve(&ax_4a);
    assert_close(
        s_ax_4a.displacement(1, 0),
        s_ax_ref.displacement(1, 0) / 4.0,
        1e-9,
        "axial u ∝ 1/A",
    );

    // Stiffness products EA and EI set the response.
    assert_close(
        s_ax_ref.displacement(1, 0),
        f * l / (e * a_ref),
        1e-12,
        "EA scaling",
    );
    assert_close(
        s_bend_ref.displacement(1, 1),
        -p * l.powi(3) / (3.0 * e * i_ref),
        1e-10,
        "EI scaling",
    );

    println!("[J section scaling] u_bending ∝ 1/(EI), u_axial ∝ 1/(EA)");
}

// ===========================================================================
// Benchmark K — global equilibrium (combined, independent analytical check)
// ===========================================================================

#[test]
fn test_benchmark_k_global_equilibrium_combined() {
    let (l, e, i) = (1.0, E0, i0());
    let (fx, fy, m, q) = (500.0, 1000.0, 800.0, 400.0);

    let mut model = beam_chain(1, l, e, A0, i);
    model.add_nodal_force(1, 0, fx);
    model.add_nodal_force(1, 1, -fy);
    model.add_applied_moment(1, m).unwrap();
    model.add_distributed_load(0, 0.0, -q).unwrap();
    let solver = solve(&model);
    let r = solver.results().reaction(0).unwrap();

    // Analytical equilibrium about node 0 (origin):
    //   ΣFx = Rx + fx = 0
    //   ΣFy = Ry - fy - q·L = 0
    //   ΣMz = Rz + m - fy·L - q·L·(L/2) = 0
    assert_close(r.fx + fx, 0.0, 1e-9, "ΣFx");
    assert_close(r.fy - fy - q * l, 0.0, 1e-9, "ΣFy");
    assert_close(r.mz + m - fy * l - q * l * (l / 2.0), 0.0, 1e-9, "ΣMz");

    println!(
        "[K equilibrium] Rx={:.6e} Ry={:.6e} Rz={:.6e} | ΣFx={:.2e} ΣFy={:.2e} ΣMz={:.2e}",
        r.fx,
        r.fy,
        r.mz,
        r.fx + fx,
        r.fy - fy - q * l,
        r.mz + m - fy * l - q * l * (l / 2.0)
    );
}

// ===========================================================================
// Benchmark L — internal-force equilibrium (finite-difference + jumps)
// ===========================================================================

#[test]
fn test_benchmark_l_internal_force_equilibrium() {
    let (l, e, i) = (1.0, E0, i0());
    let (qx, qy) = (300.0, 400.0); // qx tensile (+), qy upward (+)

    // Distributed load: dN/dx = -qx, dV/dx = qy, dM/dx = V.
    let mut model = beam_chain(1, l, e, A0, i);
    model.add_distributed_load(0, qx, qy).unwrap();
    let solver = solve(&model);

    let n = 201;
    let dx = l / (n - 1) as f64;
    let samples = solver.sample_element_forces(0, n).unwrap();
    for k in 0..(n - 1) {
        let dn = (samples[k + 1].section_forces.axial - samples[k].section_forces.axial) / dx;
        let dv = (samples[k + 1].section_forces.shear - samples[k].section_forces.shear) / dx;
        assert_close(dn, -qx, 1e-8, "dN/dx = -qx");
        assert_close(dv, qy, 1e-8, "dV/dx = qy");
    }
    for k in 1..(n - 1) {
        let dm = (samples[k + 1].section_forces.moment - samples[k - 1].section_forces.moment)
            / (2.0 * dx);
        assert_close(dm, samples[k].section_forces.shear, 1e-7, "dM/dx = V");
    }

    // Interior point force jumps.
    let (fx, fy, mz) = (700.0, -900.0, 600.0);
    let mut mp = beam_chain(1, l, e, A0, i);
    mp.add_point_load(0, 0.5, fx, fy, mz).unwrap();
    let sp = solve(&mp);
    let left = sp.element_section_forces(0, 0.5).unwrap(); // left limit
    let right = sp.element_section_forces(0, 0.5 + 1e-6).unwrap();
    assert_close(right.axial - left.axial, -fx, 1e-6, "N jump = -Fx");
    assert_close(right.shear - left.shear, fy, 1e-6, "V jump = +Fy");
    assert_close(right.moment - left.moment, -mz, 1e-6, "M jump = -Mz");

    // Nodal applied moment: shared-node equilibrium (Phase 2.4).
    let m = 1000.0;
    let mut mm = beam_chain(2, l, e, A0, i);
    mm.add_applied_moment(1, m).unwrap();
    let sm = solve(&mm);
    let lft = sm.element_section_forces(0, 1.0).unwrap();
    let rgt = sm.element_section_forces(1, 0.0).unwrap();
    assert_close(
        rgt.moment - lft.moment + m,
        0.0,
        1e-9,
        "M_right - M_left + M_ext = 0",
    );

    println!("[L internal equilibrium] FD identities and jumps verified");
}

// ===========================================================================
// Benchmark M — energy consistency (work–energy, public API only)
// ===========================================================================

#[test]
fn test_benchmark_m_energy_consistency() {
    let (l, e, i, p) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    // Cantilever with a tip transverse force.
    let mut model = beam_chain(1, l, e, A0, i);
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);
    let u_tip = solver.displacement(1, 1);

    // Bending strain energy U = ∫ M²/(2EI) dx, using the public section recovery.
    let n = 200; // even
    let dx = l / n as f64;
    let mut m2_vals = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let xi = k as f64 / n as f64;
        let m = solver.element_section_forces(0, xi).unwrap().moment;
        m2_vals.push(m * m);
    }
    let u: f64 = simpson(&m2_vals, dx) / (2.0 * ei);

    // External work W = ½ · (applied load) · (displacement at the loaded DOF).
    let w = 0.5 * p * (-u_tip); // load -P, displacement u_tip (<0)

    assert_close(u, w, 1e-6, "U (from M field) == W (1/2 P·δ)");
    // Both equal the closed form P²L³/(6EI).
    assert_close(u, p * p * l.powi(3) / (6.0 * ei), 1e-6, "U closed form");

    // Axial energy for a bar: U = ∫ N²/(2EA) dx == ½ F·δ.
    let f = 800.0;
    let mut ma = BeamModel::new();
    ma.add_node(BeamNode::new(0, 0.0, 0.0));
    ma.add_node(BeamNode::new(1, l, 0.0));
    ma.add_element(BeamElement::new(0, 1, material(e), BeamSection::new(A0, i)).unwrap());
    ma.fix_node(0);
    ma.add_nodal_force(1, 0, f);
    let sa = solve(&ma);
    let mut n2_vals = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let xi = k as f64 / n as f64;
        let nn = sa.element_section_forces(0, xi).unwrap().axial;
        n2_vals.push(nn * nn);
    }
    let u_ax: f64 = simpson(&n2_vals, dx) / (2.0 * e * A0);
    let w_ax = 0.5 * f * sa.displacement(1, 0);
    assert_close(u_ax, w_ax, 1e-9, "axial U == W");

    println!(
        "[M energy] bending U={:.9e} W={:.9e} rel={:.2e}; axial U={:.9e} W={:.9e}",
        u,
        w,
        rel_err(u, w),
        u_ax,
        w_ax
    );
}

// ===========================================================================
// Benchmark N — result API consistency (delegation, not physics)
// ===========================================================================

#[test]
fn test_benchmark_n_result_api_consistency() {
    let (l, e, i) = (1.0, E0, i0());
    let mut model = beam_chain(3, l, e, A0, i);
    model.add_nodal_force(3, 1, -1000.0);
    model.add_distributed_load(1, 0.0, -500.0).unwrap();
    model.add_point_moment(2, 0.5, 400.0).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    for node in 0..4 {
        assert_close(
            r.displacement(node).unwrap().uy,
            solver.displacement(node, 1),
            1e-15,
            "result displacement == solver",
        );
    }
    let raw = solver.reactions();
    for node in 0..4 {
        let base = node * 3;
        // Constrained node 0 matches exactly; free nodes are masked to 0.
        if node == 0 {
            assert_close(
                r.reaction(node).unwrap().fy,
                raw[base + 1],
                1e-12,
                "result reaction == solver",
            );
        } else {
            assert_close(
                r.reaction(node).unwrap().fy,
                0.0,
                1e-12,
                "masked free reaction",
            );
        }
    }

    let direct_end = solver.element_end_forces().unwrap();
    for (elem, de) in direct_end.iter().enumerate() {
        let a = r.element_end_forces(elem).unwrap();
        for (k, (av, dv)) in a.iter().zip(de.iter()).enumerate() {
            assert_close(
                *av,
                *dv,
                1e-15,
                &format!("result end force == solver [{}]", k),
            );
        }
    }
    for elem in 0..3 {
        for &xi in &[0.0, 0.5, 1.0] {
            let a = r.section_forces(elem, xi).unwrap();
            let b = solver.element_section_forces(elem, xi).unwrap();
            assert_close(a.moment, b.moment, 1e-15, "result section == solver");
        }
    }
    let sa = r.sample_forces(4).unwrap();
    let sb = solver.sample_beam_forces(4).unwrap();
    assert_eq!(sa.len(), sb.len());
    for (x, y) in sa.iter().zip(sb.iter()) {
        assert_eq!(x.element_index, y.element_index);
        assert_close(x.x, y.x, 1e-15, "result sample x == solver");
        assert_close(
            x.section_forces.moment,
            y.section_forces.moment,
            1e-15,
            "result sample M == solver",
        );
    }

    println!("[N result API] all accessors delegate consistently to BeamSolver");
}
